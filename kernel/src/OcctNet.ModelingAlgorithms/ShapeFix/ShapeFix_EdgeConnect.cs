// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_EdgeConnect.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_EdgeConnect.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Makes vertices shared to connect edges (moved from ShapeBuild): rebuilds the edges with the shared vertices and
/// updates the positions and tolerances of the shared vertices. Accepts edges bounded by two vertices each.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). <see cref="Add(in TopoDS_Edge, in TopoDS_Edge)"/> and
/// <see cref="Add(in TopoDS_Shape)"/> only record the connections; OCCT's <see cref="Build()"/> then writes into the
/// recorded edges and vertices in place (vertices removed and added, a shared vertex moved and its tolerance raised),
/// so it is internal: ported code calls it on shapes it owns.
/// </para>
/// <para>
/// The public form is <see cref="Built(in TopoDS_Shape)"/>: the connections recorded on a shape's edges are built on
/// an exact copy of that shape (<see cref="BRepTools_CopyAtEntry"/>, decision 583), whose unchanged sub-shapes are the
/// caller's own, and the result is frozen.
/// </para>
/// </remarks>
public sealed class ShapeFix_EdgeConnect
{
    private NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>                   myVertices;
    private NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myLists;

    // #define POSITION_USES_MEAN_POINT

    /// <summary>Empty constructor.</summary>
    public ShapeFix_EdgeConnect()
    {
        myVertices = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        myLists    = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
    }

    /// <summary>
    /// Adds the connection between the end vertex of the first edge and the start vertex of the second edge, taking
    /// the edges' orientations into account.
    /// </summary>
    public void Add(in TopoDS_Edge aFirst, in TopoDS_Edge aSecond)
    {
        // Select vertices to connect
        TopoDS_Vertex theFirstVertex  = TopExp.LastVertex(aFirst, true);
        TopoDS_Vertex theSecondVertex = TopExp.FirstVertex(aSecond, true);

        // Make necessary bindings
        if (myVertices.IsBound(theFirstVertex))
        {
            // First vertex is bound - find shared vertex
            TopoDS_Vertex theFirstShared = TopoDS.Vertex(myVertices[theFirstVertex]);
            if (myVertices.IsBound(theSecondVertex))
            {
                // Second vertex is bound - find shared vertex
                TopoDS_Vertex theSecondShared = TopoDS.Vertex(myVertices[theSecondVertex]);
                if (!theFirstShared.IsSame(theSecondShared))
                {
                    // Concatenate lists
                    NCollection_List<TopoDS_Shape> theFirstList  = myLists[theFirstShared];
                    NCollection_List<TopoDS_Shape> theSecondList = myLists[theSecondShared];
                    for (var theIterator = new NCollection_List<TopoDS_Shape>.Iterator(theSecondList);
                         theIterator.More();
                         theIterator.Next())
                    {
                        // Rebind shared vertex for current one
                        myVertices[theIterator.Value()] = theFirstShared;
                        // Skip the following edge
                        theIterator.Next();
                    }
                    // Append second list to the first one
                    theFirstList.Append(theSecondList);
                    // Unbind the second shared vertex
                    myLists.UnBind(theSecondShared);
                }
            }
            else
            {
                // Bind second vertex with shared vertex of the first one
                myVertices.Bind(theSecondVertex, theFirstShared);
                // Add second vertex and second edge to the list
                NCollection_List<TopoDS_Shape> theFirstList = myLists[theFirstShared];
                theFirstList.Append(theSecondVertex);
                theFirstList.Append(aSecond);
            }
        }
        else
        {
            if (myVertices.IsBound(theSecondVertex))
            {
                // Second vertex is bound - find shared vertex
                TopoDS_Vertex theSecondShared = TopoDS.Vertex(myVertices[theSecondVertex]);
                // Bind first vertex with shared vertex of the second one
                myVertices.Bind(theFirstVertex, theSecondShared);
                // Add first vertex and first edge to the list
                NCollection_List<TopoDS_Shape> theSecondList = myLists[theSecondShared];
                theSecondList.Append(theFirstVertex);
                theSecondList.Append(aFirst);
            }
            else
            {
                // None is bound - create new bindings
                myVertices.Bind(theFirstVertex, theFirstVertex);
                myVertices.Bind(theSecondVertex, theFirstVertex);
                var theNewList = new NCollection_List<TopoDS_Shape>();
                theNewList.Append(theFirstVertex);
                theNewList.Append(aFirst);
                theNewList.Append(theSecondVertex);
                theNewList.Append(aSecond);
                myLists.Bind(theFirstVertex, theNewList);
            }
        }
    }

    /// <summary>
    /// Adds the connections of the whole shape: the consecutive edges of each wire (well ordered), and the last and
    /// first edges of each wire flagged Closed.
    /// </summary>
    public void Add(in TopoDS_Shape aShape)
    {
        for (var expw = new TopExp_Explorer(aShape, TopAbs_WIRE); expw.More(); expw.Next())
        {
            TopoDS_Wire theWire = TopoDS.Wire(expw.Current());
            var         expe    = new TopExp_Explorer(theWire, TopAbs_EDGE);
            if (expe.More())
            {
                // Obtain the first edge and remember it
                TopoDS_Edge theEdge  = TopoDS.Edge(expe.Current());
                TopoDS_Edge theFirst = theEdge;
                expe.Next();
                for (; expe.More(); expe.Next())
                {
                    // Obtain second edge and connect it
                    TopoDS_Edge theNext = TopoDS.Edge(expe.Current());
                    Add(theEdge, theNext);
                    theEdge = theNext;
                }
                // Connect first and last edges if wire is closed
                if (theWire.Closed())
                {
                    Add(theEdge, theFirst);
                }
            }
        }
    }

    /// <summary>
    /// Builds the shared vertices, updates their positions and tolerances, puts them into the recorded edges in place
    /// and clears the connections. Throws on frozen edges or vertices: see <see cref="Built(in TopoDS_Shape)"/>.
    /// </summary>
    internal void Build()
    {
        var theLIterator = new NCollection_List<TopoDS_Shape>.Iterator();

        var    thePositions = new NCollection_Sequence<Xyz>();
        Xyz thePosition;
        double theMaxDev;
        var    theBuilder = new BRep_Builder();

        // Iterate on shared vertices (the port's map order, decision 18: each shared vertex is built on its own)
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theSIterator in myLists)
        {
            TopoDS_Vertex                  theSharedVertex = TopoDS.Vertex(theSIterator.Key);
            NCollection_List<TopoDS_Shape> theList         = theSIterator.Value;

            thePositions.Clear();

            // Iterate on edges, accumulating positions
            for (theLIterator.Initialize(theList); theLIterator.More(); theLIterator.Next())
            {
                TopoDS_Vertex theVertex = TopoDS.Vertex(theLIterator.Value());
                theLIterator.Next();
                ref TopoDS_Shape theEdgeShape = ref theLIterator.ChangeValue();

                // Determine usage of curve bound points
                theEdgeShape.Orientation(TopAbs_FORWARD);
                TopoDS_Edge theEdge = TopoDS.Edge(theEdgeShape);
                TopExp.Vertices(theEdge, out TopoDS_Vertex theStart, out TopoDS_Vertex theEnd);
                bool use_start = (theVertex.IsSame(theStart));
                bool use_end   = (theVertex.IsSame(theEnd));

                // Iterate on edge curves, accumulating positions (OCCT.NET: frozen: read through Curves(), where OCCT
                // walks ChangeCurves())
                var theCIterator =
                    new NCollection_List<BRep_CurveRepresentation>.Iterator(((BRep_TEdge)theEdge.TShape()!).Curves());
                for (; theCIterator.More(); theCIterator.Next())
                {
                    if (theCIterator.Value() is not BRep_GCurve GC)
                    {
                        continue;
                    }
                    // Calculate vertex position for this curve
                    GC.Range(out double theFParam, out double theLParam);
                    Point3D thePoint;
                    if (use_start)
                    {
                        GC.D0(theFParam, out thePoint);
                        thePositions.Append(thePoint.XYZ());
                    }
                    if (use_end)
                    {
                        GC.D0(theLParam, out thePoint);
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
                TopoDS_Vertex theVertex = TopoDS.Vertex(theLIterator.Value());
                theLIterator.Next();
                ref TopoDS_Shape theEdgeShape = ref theLIterator.ChangeValue();

                // Determine usage of old vertices
                theEdgeShape.Orientation(TopAbs_FORWARD);
                TopoDS_Edge theEdge = TopoDS.Edge(theEdgeShape);
                TopExp.Vertices(theEdge, out TopoDS_Vertex theStart, out TopoDS_Vertex theEnd);
                bool use_start = (theVertex.IsSame(theStart));
                bool use_end   = (theVertex.IsSame(theEnd));

                // Prepare vertex to remove
                TopoDS_Vertex theOldVertex;
                if (use_start)
                {
                    theOldVertex = theStart; // start is preferred for closed edges
                }
                else
                {
                    theOldVertex = theEnd;
                }

                // Prepare vertex to add
                TopoDS_Vertex theNewVertex;
                // smh#8 Porting AIX
                if (use_start)
                {
                    TopoDS_Shape tmpshapeFwd = theSharedVertex.Oriented(TopAbs_FORWARD);
                    theNewVertex             = TopoDS.Vertex(tmpshapeFwd);
                }
                else
                {
                    TopoDS_Shape tmpshapeRev = theSharedVertex.Oriented(TopAbs_REVERSED);
                    theNewVertex             = TopoDS.Vertex(tmpshapeRev);
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
                        theBuilder.Remove(theEdge, theOldVertex.Oriented(TopAbs_REVERSED)); // remove reversed from closed edge
                        theBuilder.Add(theEdge, theNewVertex.Oriented(TopAbs_REVERSED)); // add reversed to closed edge
                    }
                    theEdge.Free(freeflag);
                }
            }
        }

        // Clear maps after build
        Clear();
    }

    /// <summary>
    /// The functional form of <see cref="Build()"/>: builds the recorded connections on an exact copy of
    /// <paramref name="theShape"/>, which must hold every edge recorded, and returns it frozen, with the caller's own
    /// sub-shapes wherever nothing changed (<paramref name="theShape"/> itself when nothing did). Clears the connections.
    /// </summary>
    public TopoDS_Shape Built(in TopoDS_Shape theShape)
    {
        var anEntry = new BRepTools_CopyAtEntry(theShape);
        if (anEntry.IsCopied())
        {
            // the recorded vertices and edges onto their copies
            var aVertices = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> aPair in myVertices)
            {
                aVertices.Bind(anEntry.Copied(aPair.Key), anEntry.Copied(aPair.Value));
            }
            var aLists = new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
            foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aPair in myLists)
            {
                var aList = new NCollection_List<TopoDS_Shape>();
                for (var anIt = new NCollection_List<TopoDS_Shape>.Iterator(aPair.Value); anIt.More(); anIt.Next())
                {
                    aList.Append(anEntry.Copied(anIt.Value()));
                }
                aLists.Bind(anEntry.Copied(aPair.Key), aList);
            }
            myVertices = aVertices;
            myLists    = aLists;
        }
        Build();
        TopoDS_Shape aResult = anEntry.Finish(anEntry.Shape());
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary>Clears the recorded connections.</summary>
    public void Clear()
    {
        myVertices.Clear();
        myLists.Clear();
    }
}
