// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_EdgeConnect.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_EdgeConnect.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Makes vertices shared to connect edges (moved from ShapeBuild): rebuilds the edges with the shared vertices and
/// updates the positions and tolerances of the shared vertices. Accepts edges bounded by two vertices each.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). <see cref="Add(in Edge, in Edge)"/> and
/// <see cref="Add(in Shape)"/> only record the connections; OCCT's <see cref="Build()"/> then writes into the
/// recorded edges and vertices in place (vertices removed and added, a shared vertex moved and its tolerance raised),
/// so it is internal: ported code calls it on shapes it owns.
/// </para>
/// <para>
/// The public form is <see cref="Built(in Shape)"/>: the connections recorded on a shape's edges are built on
/// that shape in a working overlay (unit 10H4), so the result's unchanged sub-shapes are the caller's own, and the
/// result is frozen.
/// </para>
/// </remarks>
internal sealed class EdgeConnectFixer
{
    private Dictionary<Shape, Shape>                   _vertices;
    private Dictionary<Shape, NCollection_List<Shape>> _lists;

    // #define POSITION_USES_MEAN_POINT

    /// <summary>Empty constructor.</summary>
    public EdgeConnectFixer()
    {
        _vertices = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        _lists    = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
    }

    /// <summary>
    /// Adds the connection between the end vertex of the first edge and the start vertex of the second edge, taking
    /// the edges' orientations into account.
    /// </summary>
    public void Add(in Edge aFirst, in Edge aSecond)
    {
        // Select vertices to connect
        Vertex theFirstVertex  = Exploration.LastVertex(aFirst, true);
        Vertex theSecondVertex = Exploration.FirstVertex(aSecond, true);

        // Make necessary bindings
        if (_vertices.ContainsKey(theFirstVertex))
        {
            // First vertex is bound - find shared vertex
            Vertex theFirstShared = ShapeCasts.Vertex(_vertices.GetValue(theFirstVertex));
            if (_vertices.ContainsKey(theSecondVertex))
            {
                // Second vertex is bound - find shared vertex
                Vertex theSecondShared = ShapeCasts.Vertex(_vertices.GetValue(theSecondVertex));
                if (!theFirstShared.IsSame(theSecondShared))
                {
                    // Concatenate lists
                    NCollection_List<Shape> theFirstList  = _lists.GetValue(theFirstShared);
                    NCollection_List<Shape> theSecondList = _lists.GetValue(theSecondShared);
                    for (var theIterator = new NCollection_List<Shape>.Iterator(theSecondList);
                         theIterator.More();
                         theIterator.Next())
                    {
                        // Rebind shared vertex for current one
                        _vertices.GetValueRef(theIterator.Value()) = theFirstShared;
                        // Skip the following edge
                        theIterator.Next();
                    }
                    // Append second list to the first one
                    theFirstList.Append(theSecondList);
                    // Unbind the second shared vertex
                    _lists.Remove(theSecondShared);
                }
            }
            else
            {
                // Bind second vertex with shared vertex of the first one
                _vertices[theSecondVertex] = theFirstShared;
                // Add second vertex and second edge to the list
                NCollection_List<Shape> theFirstList = _lists.GetValue(theFirstShared);
                theFirstList.Append(theSecondVertex);
                theFirstList.Append(aSecond);
            }
        }
        else
        {
            if (_vertices.ContainsKey(theSecondVertex))
            {
                // Second vertex is bound - find shared vertex
                Vertex theSecondShared = ShapeCasts.Vertex(_vertices.GetValue(theSecondVertex));
                // Bind first vertex with shared vertex of the second one
                _vertices[theFirstVertex] = theSecondShared;
                // Add first vertex and first edge to the list
                NCollection_List<Shape> theSecondList = _lists.GetValue(theSecondShared);
                theSecondList.Append(theFirstVertex);
                theSecondList.Append(aFirst);
            }
            else
            {
                // None is bound - create new bindings
                _vertices[theFirstVertex] = theFirstVertex;
                _vertices[theSecondVertex] = theFirstVertex;
                var theNewList = new NCollection_List<Shape>();
                theNewList.Append(theFirstVertex);
                theNewList.Append(aFirst);
                theNewList.Append(theSecondVertex);
                theNewList.Append(aSecond);
                _lists[theFirstVertex] = theNewList;
            }
        }
    }

    /// <summary>
    /// Adds the connections of the whole shape: the consecutive edges of each wire (well ordered), and the last and
    /// first edges of each wire flagged Closed.
    /// </summary>
    public void Add(in Shape aShape)
    {
        for (var expw = new Explorer(aShape, ShapeType.Wire); expw.More(); expw.Next())
        {
            Wire theWire = ShapeCasts.Wire(expw.Current());
            var         expe    = new Explorer(theWire, ShapeType.Edge);
            if (expe.More())
            {
                // Obtain the first edge and remember it
                Edge theEdge  = ShapeCasts.Edge(expe.Current());
                Edge theFirst = theEdge;
                expe.Next();
                for (; expe.More(); expe.Next())
                {
                    // Obtain second edge and connect it
                    Edge theNext = ShapeCasts.Edge(expe.Current());
                    Add(theEdge, theNext);
                    theEdge = theNext;
                }
                // Connect first and last edges if wire is closed
                if (theWire.IsClosed)
                {
                    Add(theEdge, theFirst);
                }
            }
        }
    }

    /// <summary>
    /// Builds the shared vertices, updates their positions and tolerances, puts them into the recorded edges in place
    /// and clears the connections. Throws on frozen edges or vertices: see <see cref="Built(in Shape)"/>.
    /// </summary>
    internal void Build()
    {
        var theLIterator = new NCollection_List<Shape>.Iterator();

        var    thePositions = new NCollection_Sequence<Xyz>();
        Xyz thePosition;
        double theMaxDev;
        var    theBuilder = new BrepBuilder();

        // Iterate on shared vertices (the port's map order, decision 18: each shared vertex is built on its own)
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> theSIterator in _lists)
        {
            Vertex                  theSharedVertex = ShapeCasts.Vertex(theSIterator.Key);
            NCollection_List<Shape> theList         = theSIterator.Value;

            thePositions.Clear();

            // Iterate on edges, accumulating positions
            for (theLIterator.Initialize(theList); theLIterator.More(); theLIterator.Next())
            {
                Vertex theVertex = ShapeCasts.Vertex(theLIterator.Value());
                theLIterator.Next();
                ref Shape theEdgeShape = ref theLIterator.ChangeValue();

                // Determine usage of curve bound points
                theEdgeShape.Orient(Forward);
                Edge theEdge = ShapeCasts.Edge(theEdgeShape);
                Exploration.Vertices(theEdge, out Vertex theStart, out Vertex theEnd);
                bool use_start = (theVertex.IsSame(theStart));
                bool use_end   = (theVertex.IsSame(theEnd));

                // Iterate on edge curves, accumulating positions (OCCT.NET: frozen: read through Curves(), where OCCT
                // walks ChangeCurves())
                var theCIterator =
                    new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)theEdge.TShape()!).Curves());
                for (; theCIterator.More(); theCIterator.Next())
                {
                    if (theCIterator.Value() is not GCurve gc)
                    {
                        continue;
                    }
                    // Calculate vertex position for this curve
                    gc.Range(out double theFParam, out double theLParam);
                    Point3D thePoint;
                    if (use_start)
                    {
                        gc.D0(theFParam, out thePoint);
                        thePositions.Append(thePoint.XYZ());
                    }
                    if (use_end)
                    {
                        gc.D0(theLParam, out thePoint);
                        thePositions.Append(thePoint.XYZ());
                    }
                }
            }

            int i, theNbPos = thePositions.Length();

            // Calculate vertex position
            thePosition = new Xyz(0.0, 0.0, 0.0);

            // (OCCT's POSITION_USES_MEAN_POINT branch, disabled there, is left out)
            var theLBound = new Xyz(0.0, 0.0, 0.0);
            var theRBound = new Xyz(0.0, 0.0, 0.0);
            for (i = 1; i <= theNbPos; i++)
            {
                thePosition = thePositions.Value(i);
                if (i == 1)
                {
                    theLBound = theRBound = thePosition;
                }
                double val = thePosition.X();
                if (val < theLBound.X())
                {
                    theLBound.SetX(val);
                }
                else if (val > theRBound.X())
                {
                    theRBound.SetX(val);
                }
                val = thePosition.Y();
                if (val < theLBound.Y())
                {
                    theLBound.SetY(val);
                }
                else if (val > theRBound.Y())
                {
                    theRBound.SetY(val);
                }
                val = thePosition.Z();
                if (val < theLBound.Z())
                {
                    theLBound.SetZ(val);
                }
                else if (val > theRBound.Z())
                {
                    theRBound.SetZ(val);
                }
            }
            if (theNbPos > 1)
            {
                thePosition = (theLBound + theRBound) / 2.0;
            }

            // Calculate maximal deviation
            theMaxDev = 0.0;

            for (i = 1; i <= theNbPos; i++)
            {
                double theDeviation = (thePosition - thePositions.Value(i)).Modulus();
                if (theDeviation > theMaxDev)
                {
                    theMaxDev = theDeviation;
                }
            }
            theMaxDev *= 1.0001; // To avoid numerical roundings
            if (theMaxDev < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                theMaxDev = global::Kerf.Engine.Foundation.Precision.Confusion();
            }

            // Update shared vertex
            theBuilder.UpdateVertex(theSharedVertex, new Point3D(thePosition), theMaxDev);

            // Iterate on edges, adding shared vertex
            for (theLIterator.Initialize(theList); theLIterator.More(); theLIterator.Next())
            {
                Vertex theVertex = ShapeCasts.Vertex(theLIterator.Value());
                theLIterator.Next();
                ref Shape theEdgeShape = ref theLIterator.ChangeValue();

                // Determine usage of old vertices
                theEdgeShape.Orient(Forward);
                Edge theEdge = ShapeCasts.Edge(theEdgeShape);
                Exploration.Vertices(theEdge, out Vertex theStart, out Vertex theEnd);
                bool use_start = (theVertex.IsSame(theStart));
                bool use_end   = (theVertex.IsSame(theEnd));

                // Prepare vertex to remove
                Vertex theOldVertex;
                if (use_start)
                {
                    theOldVertex = theStart; // start is preferred for closed edges
                }
                else
                {
                    theOldVertex = theEnd;
                }

                // Prepare vertex to add
                Vertex theNewVertex;
                // smh#8 Porting AIX
                if (use_start)
                {
                    Shape tmpshapeFwd = theSharedVertex.Oriented(Forward);
                    theNewVertex             = ShapeCasts.Vertex(tmpshapeFwd);
                }
                else
                {
                    Shape tmpshapeRev = theSharedVertex.Oriented(Reversed);
                    theNewVertex             = ShapeCasts.Vertex(tmpshapeRev);
                }
                if (!theOldVertex.IsSame(theNewVertex))
                {
                    // Replace vertices
                    bool freeflag = theEdge.Free();
                    theEdge.Free(true); // smh
                    theBuilder.Remove(theEdge, theOldVertex);
                    theBuilder.Add(theEdge, theNewVertex);
                    if (use_start && use_end)
                    {
                        // process special case for closed edge
                        theBuilder.Remove(theEdge, theOldVertex.Oriented(Reversed)); // remove reversed from closed edge
                        theBuilder.Add(theEdge, theNewVertex.Oriented(Reversed)); // add reversed to closed edge
                    }
                    theEdge.Free(freeflag);
                }
            }
        }

        // Clear maps after build
        Clear();
    }

    /// <summary>
    /// The functional form of <see cref="Build()"/>: builds the recorded connections on <paramref name="shape"/>, which
    /// must hold every edge recorded, in a working overlay, and returns it frozen, with the caller's own sub-shapes
    /// wherever nothing changed (<paramref name="shape"/> itself when nothing did). Clears the connections.
    /// </summary>
    public Shape Built(in Shape shape)
    {
        // the caller's unfrozen shapes (a compound of frozen edges) are copied, and the recorded shapes mapped onto
        // the copies; the frozen ones are written in the overlay
        Shape work = WorkingOverlay.WithUnfrozenCopied(shape, out Dictionary<TShape, TShape>? copies);
        if (copies is not null)
        {
            var vertices = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
            foreach (KeyValuePair<Shape, Shape> pair in _vertices)
            {
                vertices[WorkingOverlay.CopiedIn(pair.Key, copies)] = WorkingOverlay.CopiedIn(pair.Value, copies);
            }
            var lists = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> pair in _lists)
            {
                var list = new NCollection_List<Shape>();
                for (var it = new NCollection_List<Shape>.Iterator(pair.Value); it.More(); it.Next())
                {
                    list.Append(WorkingOverlay.CopiedIn(it.Value(), copies));
                }
                lists[WorkingOverlay.CopiedIn(pair.Key, copies)] = list;
            }
            _vertices = vertices;
            _lists    = lists;
        }
        using var overlay = WorkingOverlay.Begin(work);
        Build();
        Shape result = WorkingOverlay.UncopiedWhereUnchanged(overlay.Finish(work), copies);
        result.TShape()?.FreezeGraph();
        return result;
    }

    /// <summary>Clears the recorded connections.</summary>
    public void Clear()
    {
        _vertices.Clear();
        _lists.Clear();
    }
}
