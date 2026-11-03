// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_DraftAngle.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffsetAPI/BRepOffsetAPI_DraftAngle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Features.Draft;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Schematiq.Collections;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features.Api;

/// <summary>
/// Taper-adding transformations on a shape: faces turned by a draft angle around their intersection with a neutral
/// plane (planes, cylinders and cones), with their tangent neighbours, and the shape rebuilt around them.
/// </summary>
/// <remarks>
/// <para>
/// Give the shape, <see cref="Add"/> each face with its direction, angle and neutral plane (check
/// <see cref="AddDone"/>; on a failure <see cref="ProblematicShape"/> is the face and <see cref="Remove"/> takes it
/// back), then <see cref="Build"/>. The result is a <see cref="Modifier"/>'s, corrected around seams
/// (<see cref="CorrectWires"/>) and for the vertex tolerances.
/// </para>
/// <para>
/// A maker (decision 414). The caller's shape is never written: the modifier rebuilds what changes and shares the rest
/// (decision 973); OCCT's <c>CorrectWires</c> then edits the modifier's result in place, freeing and changing wires,
/// faces, vertices and pcurves that may be the caller's, so when it has something to change it works on a copy of the
/// result (<see cref="CopyAtEntry"/>) and gives back the caller's sub-shapes it left alone (decision 1044).
/// </para>
/// </remarks>
public class DraftAngle : ModifyShape
{
    private readonly Dictionary<Shape, Shape> _vtxToReplace = new(ShapeHasher.Instance);
    private readonly ReShape _subs = new();

    // OCCT.NET: the copy of the modifier's result that CorrectWires works on (null when it had nothing to change).
    private CopyAtEntry? _resultCopy;

    //=================================================================================================

    /// <summary>An empty algorithm; <see cref="Init"/> gives the shape.</summary>
    public DraftAngle()
    {
    }

    //=================================================================================================

    /// <summary>The algorithm on the shape <paramref name="s"/>.</summary>
    public DraftAngle(in Shape s)
    {
        _initialShape = s;
        _modification = new DraftModification(s);
    }

    private DraftModification Draft() => (DraftModification)_modification!;

    //=================================================================================================

    /// <summary>Cancels the faces added, for the same shape.</summary>
    public void Clear()
    {
        if (_modification is not null)
        {
            this.Draft().Clear();
        }
    }

    //=================================================================================================

    /// <summary>Changes the shape and cancels the faces added.</summary>
    public void Init(in Shape s)
    {
        _initialShape = s;
        NotDone();
        if (_modification is not null)
        {
            this.Draft().Init(s);
        }
        else
        {
            _modification = new DraftModification(s);
        }
    }

    //=================================================================================================

    /// <summary>
    /// Adds the face <paramref name="f"/> of the shape, to be turned by <paramref name="angle"/> (radians; ignored
    /// up to 1e-4) about its intersection with <paramref name="plane"/>, with the pull direction <paramref name="d"/>.
    /// Its tangent neighbours are added too. <paramref name="flag"/> false keeps cylindrical neighbours' tangency.
    /// Check <see cref="AddDone"/> after.
    /// </summary>
    /// <exception cref="NullObjectException">No shape was given.</exception>
    /// <exception cref="ConstructionErrorException">An earlier Add failed and its face wasn't removed.</exception>
    public void Add(in Face f, in Direction3D d, double angle, in Plane3D plane, bool flag = true)
    {
        // POP-DPF : protection
        if (Math.Abs(angle) <= 1.0e-04)
        {
            return;
        }
        Standard_NullObject_Raise_if(_initialShape.IsNull, "BRepOffsetAPI_DraftAngle::Add() - initial shape is not set");
        this.Draft().Add(f, d, angle, plane, flag);
    }

    //=================================================================================================

    /// <summary>True if the last <see cref="Add"/> succeeded.</summary>
    /// <exception cref="NullObjectException">No shape was given.</exception>
    public bool AddDone()
    {
        Standard_NullObject_Raise_if(_initialShape.IsNull, "BRepOffsetAPI_DraftAngle::AddDone() - initial shape is not set");
        return this.Draft().ProblematicShape().IsNull;
    }

    //=================================================================================================

    /// <summary>Cancels the face <paramref name="f"/> and the faces added with it (after a failed Add).</summary>
    /// <exception cref="NullObjectException">No shape was given.</exception>
    /// <exception cref="NoSuchObjectException">The face wasn't added, or the draft was built.</exception>
    public void Remove(in Face f)
    {
        Standard_NullObject_Raise_if(_initialShape.IsNull, "BRepOffsetAPI_DraftAngle::Remove() - initial shape is not set");
        this.Draft().Remove(f);
    }

    //=================================================================================================

    /// <summary>The face, edge or vertex on which the draft failed (null if none).</summary>
    /// <exception cref="NullObjectException">No shape was given.</exception>
    public Shape ProblematicShape()
    {
        Standard_NullObject_Raise_if(_initialShape.IsNull,
                                     "BRepOffsetAPI_DraftAngle::ProblematicShape() - initial shape is not set");
        return this.Draft().ProblematicShape();
    }

    //=================================================================================================

    /// <summary>The error status.</summary>
    /// <exception cref="NullObjectException">No shape was given.</exception>
    public DraftErrorStatus Status()
    {
        Standard_NullObject_Raise_if(_initialShape.IsNull, "BRepOffsetAPI_DraftAngle::Status() - initial shape is not set");
        return this.Draft().Error();
    }

    //=================================================================================================

    /// <summary>The faces of the shape added together with the face <paramref name="f"/>.</summary>
    /// <exception cref="NullObjectException">No shape was given.</exception>
    /// <exception cref="NoSuchObjectException">The face wasn't added.</exception>
    /// <exception cref="NotDoneException">The draft isn't built.</exception>
    public ISchematiqList<Shape> ConnectedFaces(in Face f)
    {
        Standard_NullObject_Raise_if(_initialShape.IsNull,
                                     "BRepOffsetAPI_DraftAngle::ConnectedFaces() - initial shape is not set");
        return PublishedList(this.Draft().ConnectedFaces(f));
    }

    //=================================================================================================

    /// <summary>The faces of the shape on which a draft was given.</summary>
    /// <exception cref="NullObjectException">No shape was given.</exception>
    /// <exception cref="NotDoneException">An Add failed.</exception>
    public ISchematiqList<Shape> ModifiedFaces()
    {
        Standard_NullObject_Raise_if(_initialShape.IsNull,
                                     "BRepOffsetAPI_DraftAngle::ModifiedFaces() - initial shape is not set");
        return PublishedList(this.Draft().ModifiedFaces());
    }

    //=================================================================================================

    internal override NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        _generated.Clear();
        Standard_NullObject_Raise_if(_initialShape.IsNull, "BRepOffsetAPI_DraftAngle::Generated() - initial shape is not set");
        DraftModification dMod = this.Draft();

        if (s.ShapeType() == ShapeType.Face)
        {
            if (dMod.NewSurface(ShapeCasts.Face(s), out _, out _, out _, out _, out _))
            {
                if (_vtxToReplace.Count == 0)
                {
                    _generated.Append(InternalModifiedShape(s));
                }
                else
                {
                    _generated.Append(_subs.InternalValue(InternalModifiedShape(s)));
                }
            }
        }
        return _generated;
    }

    //=================================================================================================

    internal override NCollection_List<Shape> InternalModified(in Shape s)
    {
        _generated.Clear();
        Standard_NullObject_Raise_if(_initialShape.IsNull, "BRepOffsetAPI_DraftAngle::Modified() - initial shape is not set");
        DraftModification dMod = this.Draft();

        if (s.ShapeType() == ShapeType.Face)
        {
            if (!dMod.NewSurface(ShapeCasts.Face(s), out _, out _, out _, out _, out _))
            {
                // Ce n est pas une generation => peut etre une  modif
                if (_vtxToReplace.Count == 0)
                {
                    _generated.Append(InternalModifiedShape(s));
                }
                else
                {
                    _generated.Append(_subs.InternalValue(InternalModifiedShape(s)));
                }
                if (_generated.Extent() == 1 && _generated.First().IsSame(s))
                {
                    _generated.Clear();
                }
            }
        }
        return _generated;
    }

    //=================================================================================================

    internal override Shape InternalModifiedShape(in Shape s)
    {
        if (s.ShapeType() == ShapeType.Vertex)
        {
            if (_vtxToReplace.ContainsKey(s))
            {
                return _vtxToReplace.GetValue(s);
            }
        }
        // OCCT.NET: through the copy CorrectWires worked on, if it made one.
        Shape ns = _modifier.InternalModifiedShape(s);
        if (_resultCopy is not null)
        {
            ns = _resultCopy.Final(_resultCopy.Copied(ns));
        }
        if (_vtxToReplace.Count == 0)
        {
            return ns;
        }
        else
        {
            return _subs.InternalValue(ns);
        }
    }

    //=================================================================================================

    /// <summary>Builds the draft. Check IsDone, then <see cref="Status"/> and <see cref="ProblematicShape"/>.</summary>
    /// <exception cref="ConstructionErrorException">An Add failed and its face wasn't removed.</exception>
    public override void Build(ProgressRange range = default)
    {
        _resultCopy = null;
        this.Draft().Perform();
        if (!this.Draft().IsDone)
        {
            NotDone();
        }
        else
        {
            DoModif(_initialShape);
            CorrectWires();
            CorrectVertexTol();
        }
    }

    //=================================================================================================

    /// <summary>
    /// Splits the edges of the faces' other wires where they cross a seam, and the seams there, so that the wires on
    /// periodic faces stay valid after the draft. <see cref="Build"/> calls it.
    /// </summary>
    public void CorrectWires()
    {
        double tolInter = 1.0e-7;
        int i, j, k;

        NCollection_Sequence<Shape> eseq = new();
        NCollection_Sequence<Shape> wseq = new();
        NCollection_Sequence<Shape> fseq = new();
        Shape curEdge, curWire, curFace;
        Iterator wit = new(), eit = new();

        Explorer fexp = new(_shape, ShapeType.Face);
        for (; fexp.More(); fexp.Next())
        {
            curFace = fexp.Current();
            wit.Initialize(curFace);
            for (; wit.More(); wit.Next())
            {
                curWire = wit.Value();
                // OCCT iterates a map of the wire's edges (by address); the port in the wire's order.
                IndexedSet<Shape> emap = new(ShapeHasher.Instance);
                eit.Initialize(curWire);
                for (; eit.More(); eit.Next())
                {
                    emap.Add(eit.Value());
                }
                for (int m = 0; m < emap.Count; m++)
                {
                    curEdge = emap[m];
                    if (BrepTools.IsReallyClosed(ShapeCasts.Edge(curEdge), ShapeCasts.Face(curFace)))
                    {
                        eseq.Append(curEdge);
                        wseq.Append(curWire);
                        fseq.Append(curFace);
                    }
                }
            }
        }

        Dictionary<Shape, NCollection_Sequence<double>> Emap = new(ShapeHasher.Instance);

        NCollection_Sequence<Shape> nonSeam = new();
        NCollection_Sequence<Shape> nonSeamWires = new();
        NCollection_Sequence<NCollection_Sequence<double>> parsNonSeam = new();
        NCollection_Sequence<NCollection_Sequence<Shape>> seam = new();
        NCollection_Sequence<NCollection_Sequence<double>> parsSeam = new();

        Dictionary<Shape, Shape> wFmap = new(ShapeHasher.Instance);
        Dictionary<Shape, NCollection_List<Shape>> wWmap = new(ShapeHasher.Instance);
        for (i = 1; i <= eseq.Length(); i++)
        {
            curEdge = eseq[i];
            curWire = wseq[i];
            curFace = fseq[i];
            //
            Face face = ShapeCasts.Face(curFace);
            //
            // Prepare 2D adaptors for intersection.
            // The seam edge has two 2D curve, thus we have to create 2 adaptors
            EdgeCurve2DAdaptor bac2D1 = new(ShapeCasts.Edge(curEdge), face);
            EdgeCurve2DAdaptor bac2D1R = new(ShapeCasts.Edge(curEdge.Reversed()), face);
            // Get surface of the face to get 3D intersection point
            Surface surf = Tool.Surface(face, out Placement loc)!;
            // Get the tolerance of the current edge to compare intersection points
            double tolCurE = Tool.Tolerance(ShapeCasts.Edge(curEdge));
            //
            wit.Initialize(curFace);
            for (; wit.More(); wit.Next())
            {
                Shape wire = wit.Value();
                if (!wire.IsSame(curWire))
                {
                    NCollection_Sequence<Point3D> pts = new();
                    bool wadd = false;
                    eit.Initialize(wire);
                    for (; eit.More(); eit.Next())
                    {
                        Edge edge = ShapeCasts.Edge(eit.Value());
                        //
                        // Prepare 2D adaptor for intersection
                        EdgeCurve2DAdaptor bac2D2 = new(edge, face);
                        // Perform intersection
                        GInter gInter = new();
                        gInter.Perform(bac2D1, bac2D2, tolInter, tolInter);
                        if (!gInter.IsDone || gInter.IsEmpty)
                        {
                            // If first intersection is empty try intersection with reversed edge
                            gInter.Perform(bac2D1R, bac2D2, tolInter, tolInter);
                            if (!gInter.IsDone || gInter.IsEmpty)
                            {
                                continue;
                            }
                        }
                        //
                        wadd = true;
                        if (!wFmap.ContainsKey(wire))
                        {
                            wFmap[wire] = curFace;
                        }
                        int ind = 0;
                        for (j = 1; j <= nonSeam.Length(); j++)
                        {
                            if (edge.IsSame(nonSeam[j]))
                            {
                                ind = j;
                                break;
                            }
                        }
                        if (ind == 0)
                        {
                            nonSeam.Append(edge);
                            nonSeamWires.Append(wire);
                            ind = nonSeam.Length();
                            parsNonSeam.Append(new NCollection_Sequence<double>());
                            seam.Append(new NCollection_Sequence<Shape>());
                            parsSeam.Append(new NCollection_Sequence<double>());
                        }
                        if (!Emap.ContainsKey(curEdge))
                        {
                            Emap[curEdge] = new NCollection_Sequence<double>();
                        }
                        //
                        // Get the tolerance of edge to compare intersection points
                        double tolE = Tool.Tolerance(edge);
                        // Tolerance to compare the intersection points is the maximal
                        // tolerance of intersecting edges
                        double tolCmp = Max(tolCurE, tolE);
                        //
                        int nbIntPnt = gInter.PointCount;
                        for (k = 1; k <= nbIntPnt; ++k)
                        {
                            IntersectionPoint2D p2DInt = gInter.Point(k);
                            Point2D p2D = p2DInt.Value();
                            Point3D p3D = surf.Value(p2D.X(), p2D.Y());
                            //
                            // Check if the intersection point is new
                            int ied = 0;
                            for (j = 1; j <= pts.Length(); j++)
                            {
                                if (p3D.IsEqual(pts[j], tolCmp))
                                {
                                    ied = j;
                                    break;
                                }
                            }
                            if (ied == 0)
                            {
                                pts.Append(p3D);
                                Emap.GetValue(curEdge).Append(p2DInt.ParamOnFirst());
                                parsNonSeam[ind].Append(p2DInt.ParamOnSecond());
                                seam[ind].Append(curEdge);
                                parsSeam[ind].Append(p2DInt.ParamOnFirst());
                            }
                        }
                    } // for (; eit.More(); eit.Next())
                    if (wadd)
                    {
                        if (!wWmap.ContainsKey(curWire))
                        {
                            wWmap[curWire] = new NCollection_List<Shape>();
                        }
                        wWmap.GetValue(curWire).Append(wire);
                    }
                } // if (! aWire.IsSame( CurWire ))
            } // for (; wit.More(); wit.Next())
        } // for (i = 1; i <= Eseq.Length(); i++)

        // OCCT.NET: the rest writes into the result: vertices, wires and faces it frees and edits, pcurves it moves. Where
        // the result holds the caller's sub-shapes (frozen), the result is copied first and the analysis made again on
        // the copy (decision 1044).
        if (Emap.Count != 0 && _resultCopy is null && BrepTools.HoldsFrozen(_shape))
        {
            _resultCopy = new CopyAtEntry(_shape);
            _shape = _resultCopy.Shape();
            CorrectWires();
            _shape = _resultCopy.Finish(_shape);
            return;
        }

        // Sorting
        for (i = 1; i <= nonSeam.Length(); i++)
        {
            for (j = 1; j < parsNonSeam[i].Length(); j++)
            {
                for (k = j + 1; k <= parsNonSeam[i].Length(); k++)
                {
                    if (parsNonSeam[i][k] < parsNonSeam[i][j])
                    {
                        double temp = parsNonSeam[i][j];
                        parsNonSeam[i][j] = parsNonSeam[i][k];
                        parsNonSeam[i][k] = temp;
                        Shape tmp = seam[i][j];
                        seam[i][j] = seam[i][k];
                        seam[i][k] = tmp;
                        temp = parsSeam[i][j];
                        parsSeam[i][j] = parsSeam[i][k];
                        parsSeam[i][k] = temp;
                    }
                }
            }
        }
        // OCCT iterates Emap (and EPmap below) by the edges' addresses; the port in insertion order.
        foreach (Shape key in Emap.Keys.ToList())
        {
            NCollection_Sequence<double> seq = new(Emap.GetValue(key));
            for (i = 1; i < seq.Length(); i++)
            {
                for (j = i + 1; j <= seq.Length(); j++)
                {
                    if (seq[j] < seq[i])
                    {
                        double temp = seq[i];
                        seq[i] = seq[j];
                        seq[j] = temp;
                    }
                }
            }
            Emap.GetValueRef(key) = seq;
        }
        Dictionary<Shape, NCollection_Sequence<double>> ePmap = new(ShapeHasher.Instance);
        Dictionary<Shape, NCollection_Sequence<Shape>> eVmap =
            new(ShapeHasher.Instance); // Seam
        Dictionary<Shape, NCollection_Sequence<Shape>> eWmap =
            new(ShapeHasher.Instance); // Seam and wires intersecting it
        foreach (KeyValuePair<Shape, NCollection_Sequence<double>> entry in Emap)
        {
            ePmap[entry.Key] = new NCollection_Sequence<double>();
            eVmap[entry.Key] = new NCollection_Sequence<Shape>();
            eWmap[entry.Key] = new NCollection_Sequence<Shape>();
        }

        // Reconstruction of non-seam edges
        Substitution sub = new();
        BrepBuilder bb = new();
        for (i = 1; i <= nonSeam.Length(); i++)
        {
            Edge edge = ShapeCasts.Edge(nonSeam[i]);
            NCollection_List<Shape> newEdges = new();
            Edge newE;
            Exploration.Vertices(edge, out Vertex vfirst, out Vertex vlast);
            double par;
            Tool.Range(edge, out double firstPar, out double lastPar);
            int firstind = 1;
            par = parsNonSeam[i][1];
            Edge seamEdge = ShapeCasts.Edge(seam[i][1]);
            // Find the face
            for (j = 1; j <= eseq.Length(); j++)
            {
                if (seamEdge.IsSame(eseq[j]))
                {
                    break;
                }
            }
            Face theFace = ShapeCasts.Face(fseq[j]);
            Surface? theSurf = Tool.Surface(theFace, out Placement l);
            if (Math.Abs(par - firstPar) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                bb.UpdateVertex(vfirst, parsSeam[i][1], seamEdge, Tool.Tolerance(vfirst));
                ePmap.GetValue(seamEdge).Append(parsSeam[i][1]);
                eVmap.GetValue(seamEdge).Append(vfirst);
                eWmap.GetValue(seamEdge).Append(nonSeamWires[i]);
                firstind = 2;
            }
            double prevpar = firstPar;
            Vertex prevV = vfirst;
            for (j = firstind; j <= parsNonSeam[i].Length(); j++)
            {
                Shape localShape = edge.EmptyCopied();
                newE = ShapeCasts.Edge(localShape);
                // NewE = TopoDS::Edge( anEdge.EmptyCopied() );
                Vertex newV;
                par = parsNonSeam[i][j];
                bb.Range(newE, prevpar, par);
                seamEdge = ShapeCasts.Edge(seam[i][j]);
                if (j == parsNonSeam[i].Length() && Math.Abs(par - lastPar) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    newV = vlast;
                    if (firstind == 2 && j == 2)
                    {
                        bb.UpdateVertex(vlast, parsSeam[i][j], seamEdge, Tool.Tolerance(vlast));
                        ePmap.GetValue(seamEdge).Append(parsSeam[i][j]);
                        eVmap.GetValue(seamEdge).Append(vlast);
                        eWmap.GetValue(seamEdge).Append(nonSeamWires[i]);
                        break;
                    }
                }
                else
                {
                    EdgeCurveAdaptor bcur = new(newE);
                    Point3D point = bcur.Value(par);
                    newV = new LibraryMakeVertex(point).InternalVertex();
                    bb.UpdateVertex(newV, par, newE, 10.0 * global::Kerf.Engine.Foundation.Precision.Confusion());
                }
                bb.UpdateVertex(newV, parsSeam[i][j], seamEdge, 10.0 * global::Kerf.Engine.Foundation.Precision.Confusion());
                newE.Orientation(Forward);
                bb.Add(newE, prevV.Oriented(Forward));
                bb.Add(newE, newV.Oriented(Reversed));

                newEdges.Append(newE);
                ePmap.GetValue(seamEdge).Append(parsSeam[i][j]);
                eVmap.GetValue(seamEdge).Append(newV);
                eWmap.GetValue(seamEdge).Append(nonSeamWires[i]);

                prevpar = par;
                prevV = newV;
            }
            {
                // The last edge
                Shape localShape = edge.EmptyCopied();
                newE = ShapeCasts.Edge(localShape);
                // NewE = TopoDS::Edge( anEdge.EmptyCopied() );
                par = lastPar;
                if (Math.Abs(prevpar - par) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    bb.Range(newE, prevpar, par);
                    newE.Orientation(Forward);
                    bb.Add(newE, prevV.Oriented(Forward));
                    bb.Add(newE, vlast.Oriented(Reversed));
                    newEdges.Append(newE);
                }
            }

            // Substitute anEdge by NewEdges
            sub.Substitute(edge, newEdges);
        }

        // Sorting of EPmap and EVmap and removing repeating points from them
        foreach (Shape key in ePmap.Keys.ToList())
        {
            NCollection_Sequence<double> seq = new(ePmap.GetValue(key));
            NCollection_Sequence<Shape> seqShape = new(eVmap.GetValue(key));
            NCollection_Sequence<Shape> seqShape2 = new(eWmap.GetValue(key));
            for (i = 1; i < seq.Length(); i++)
            {
                for (j = i + 1; j <= seq.Length(); j++)
                {
                    if (seq[j] < seq[i])
                    {
                        double temp = seq[i];
                        seq[i] = seq[j];
                        seq[j] = temp;
                        Shape tmp = seqShape[i];
                        seqShape[i] = seqShape[j];
                        seqShape[j] = tmp;
                        tmp = seqShape2[i];
                        seqShape2[i] = seqShape2[j];
                        seqShape2[j] = tmp;
                    }
                }
            }

            ePmap.GetValueRef(key) = seq;
            eVmap.GetValueRef(key) = seqShape;
            eWmap.GetValueRef(key) = seqShape2;
        }
        foreach (Shape key in ePmap.Keys.ToList())
        {
            NCollection_Sequence<double> seq = new(ePmap.GetValue(key));
            NCollection_Sequence<Shape> seqShape = new(eVmap.GetValue(key));
            NCollection_Sequence<Shape> seqShape2 = new(eWmap.GetValue(key));
            bool remove = true;
            while (remove)
            {
                remove = false;
                for (i = 1; i < seq.Length(); i++)
                {
                    if (Math.Abs(seq[i] - seq[i + 1]) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        seq.Remove(i + 1);
                        seqShape.Remove(i + 1);
                        seqShape2.Remove(i + 1);
                        remove = true;
                    }
                }
            }
            ePmap.GetValueRef(key) = seq;
            eVmap.GetValueRef(key) = seqShape;
            eWmap.GetValueRef(key) = seqShape2;
        }

        // Reconstruction of seam edges
        Dictionary<Shape, Shape> vEmap = new(ShapeHasher.Instance);
        foreach (KeyValuePair<Shape, NCollection_Sequence<double>> entry in Emap)
        {
            Edge edge = ShapeCasts.Edge(entry.Key);
            bool onepoint = false;
            NCollection_List<Shape> newEdges = new();
            NCollection_Sequence<double> seq = new(entry.Value);
            NCollection_Sequence<double> seq2 = new(ePmap.GetValue(edge));
            NCollection_Sequence<Shape> seqVer = new(eVmap.GetValue(edge));
            NCollection_Sequence<Shape> seqWire = new(eWmap.GetValue(edge));
            Exploration.Vertices(edge, out Vertex vfirst, out Vertex vlast);
            double fpar, lpar;
            Tool.Range(edge, out double firstPar, out double lastPar);
            fpar = firstPar;
            lpar = seq[1];
            Edge newE;
            int firstind = 1;
            if (Math.Abs(fpar - lpar) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                firstind = 2;
                fpar = seq[1];
                lpar = seq[2];
            }
            else
            {
                if (seq.Length() % 2 != 0)
                {
                    vEmap[vfirst] = edge;
                    firstind = 2;
                    fpar = seq[1];
                    if (seq.Length() > 2)
                    {
                        lpar = seq[2];
                    }
                    else
                    {
                        onepoint = true;
                    }
                }
            }
            if (!onepoint)
            {
                Shape localShape = edge.EmptyCopied();
                newE = ShapeCasts.Edge(localShape);
                // NewE = TopoDS::Edge( anEdge.EmptyCopied() );
                bb.Range(newE, fpar, lpar);
                newE.Orientation(Forward);
                if (firstind == 1)
                {
                    bb.Add(newE, vfirst.Oriented(Forward));
                    localShape = seqVer[1].Oriented(Reversed);
                    bb.Add(newE, ShapeCasts.Vertex(localShape));
                    // BB.Add( NewE, TopoDS::Vertex( SeqVer(1).Oriented(TopAbs_REVERSED) ) );
                }
                else
                {
                    localShape = seqVer[1].Oriented(Forward);
                    bb.Add(newE, ShapeCasts.Vertex(localShape));
                    localShape = seqVer[2].Oriented(Reversed);
                    bb.Add(newE, ShapeCasts.Vertex(localShape));
                    // BB.Add( NewE, TopoDS::Vertex( SeqVer(1).Oriented(TopAbs_FORWARD) ) );
                    // BB.Add( NewE, TopoDS::Vertex( SeqVer(2).Oriented(TopAbs_REVERSED) ) );
                }
                newEdges.Append(newE);

                firstind++;
                for (i = firstind; i < seq.Length(); i += 2)
                {
                    localShape = edge.EmptyCopied();
                    newE = ShapeCasts.Edge(localShape);
                    // NewE = TopoDS::Edge( anEdge.EmptyCopied() );
                    fpar = seq[i];
                    lpar = seq[i + 1];
                    bb.Range(newE, fpar, lpar);
                    // Find vertices
                    for (j = 1; j <= seq2.Length(); j++)
                    {
                        if (Math.Abs(fpar - seq2[j]) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            break;
                        }
                    }
                    newE.Orientation(Forward);
                    Shape localShapeCur = seqVer[j].Oriented(Forward);
                    bb.Add(newE, ShapeCasts.Vertex(localShapeCur));
                    localShapeCur = seqVer[j + 1].Oriented(Reversed);
                    bb.Add(newE, ShapeCasts.Vertex(localShapeCur));
                    // BB.Add( NewE, TopoDS::Vertex( SeqVer(j).Oriented(TopAbs_FORWARD) ) );
                    // BB.Add( NewE, TopoDS::Vertex( SeqVer(j+1).Oriented(TopAbs_REVERSED) ) );
                    newEdges.Append(newE);
                }
            }

            i = seq.Length();
            fpar = seq[i];
            lpar = lastPar;
            if (Math.Abs(fpar - lpar) <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                continue;
            }
            {
                Shape localShape = edge.EmptyCopied();
                newE = ShapeCasts.Edge(localShape);
                // NewE = TopoDS::Edge( anEdge.EmptyCopied() );
                bb.Range(newE, fpar, lpar);
                newE.Orientation(Forward);
                localShape = seqVer[seqVer.Length()].Oriented(Forward);
                bb.Add(newE, ShapeCasts.Vertex(localShape));
                // BB.Add( NewE, TopoDS::Vertex( SeqVer(SeqVer.Length()).Oriented(TopAbs_FORWARD) ) );
                bb.Add(newE, vlast.Oriented(Reversed));
                newEdges.Append(newE);
            }

            // Substitute anEdge by NewEdges
            sub.Substitute(edge, newEdges);
        }

        // Removing edges connected with missing extremities of seam edges
        foreach (KeyValuePair<Shape, Shape> entry in vEmap)
        {
            Shape v = entry.Key;
            Shape e = entry.Value;
            Shape w = new();
            for (i = 1; i <= eseq.Length(); i++)
            {
                if (e.IsSame(eseq[i]))
                {
                    w = wseq[i];
                    break;
                }
            }
            Shape etoremove = new();
            eit.Initialize(w);
            for (; eit.More(); eit.Next())
            {
                Edge curE = ShapeCasts.Edge(eit.Value());
                if (curE.IsSame(e))
                {
                    continue;
                }
                Exploration.Vertices(curE, out Vertex vfirst, out Vertex vlast);
                if (vfirst.IsSame(v) || vlast.IsSame(v))
                {
                    etoremove = curE;
                    break;
                }
            }
            if (!etoremove.IsNull)
            {
                w.Free(true);
                bb.Remove(w, etoremove);
            }
        }

        sub.Build(_shape);
        if (sub.IsCopied(_shape))
        {
            NCollection_List<Shape> listSh = sub.InternalCopy(_shape);
            if (!listSh.IsEmpty())
            {
                _shape = listSh.First();
            }
        }

        // Reconstruction of wires
        NCollection_List<Shape> theCopy;
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> entry in wWmap)
        {
            curWire = entry.Key;
            theCopy = sub.InternalCopy(curWire);
            curWire = theCopy.First();
            curWire.Free(true);
            NCollection_List<Shape>.Iterator itl = new(entry.Value);
            for (; itl.More(); itl.Next())
            {
                Shape wire = itl.Value();
                curFace = wFmap.GetValue(wire);
                theCopy = sub.InternalCopy(wire);
                wire = theCopy.First();
                // Adjusting period
                Surface? theSurf = Tool.Surface(ShapeCasts.Face(curFace), out Placement l);
                eit.Initialize(wire);
                for (; eit.More(); eit.Next())
                {
                    Edge edge = ShapeCasts.Edge(eit.Value());
                    Point2D pmid;
                    Tool.UVPoints(edge, ShapeCasts.Face(curFace), out Point2D pfirst, out Point2D plast);
                    EdgeCurve2DAdaptor bc2d = new(edge, ShapeCasts.Face(curFace));
                    pmid = bc2d.Value((bc2d.FirstParameter() + bc2d.LastParameter()) / 2.0);
                    Vector2D offset = new();
                    bool translate = false;
                    if (pfirst.X() - 2.0 * M_PI > global::Kerf.Engine.Foundation.Precision.Confusion() || plast.X() - 2.0 * M_PI > global::Kerf.Engine.Foundation.Precision.Confusion()
                        || pmid.X() - 2.0 * M_PI > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        offset = offset.WithCoord(-2.0 * M_PI, 0);
                        translate = true;
                    }
                    if (pfirst.X() < -global::Kerf.Engine.Foundation.Precision.Confusion() || plast.X() < -global::Kerf.Engine.Foundation.Precision.Confusion()
                        || pmid.X() < -global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        offset = offset.WithCoord(2.0 * M_PI, 0);
                        translate = true;
                    }
                    if (translate)
                    {
                        BrepTEdge te = (BrepTEdge)edge.TShape()!;
                        NCollection_List<CurveRepresentation> curves = te.ChangeCurves();
                        for (NCollection_List<CurveRepresentation>.Iterator itcr = new(curves); itcr.More(); itcr.Next())
                        {
                            if (itcr.Value() is GCurve gc && gc.IsCurveOnSurface(theSurf, l))
                            {
                                Curve2D pc = gc.PCurve()!;
                                pc = (Curve2D)pc.Translated(offset);
                                gc.PCurve(pc);
                                curves.Remove(itcr);
                                curves.Append(gc);
                                break;
                            }
                        }
                    }
                }
                ///////////////////
                eit.Initialize(wire, false);
                for (; eit.More(); eit.Next())
                {
                    Shape edge = eit.Value();
                    bb.Add(curWire, edge);
                }
                if (sub.IsCopied(curFace))
                {
                    theCopy = sub.InternalCopy(curFace);
                    curFace = theCopy.First();
                }
                curFace.Free(true);
                bb.Remove(curFace, wire);
            }
        }
    }

    //=================================================================================================

    private void CorrectVertexTol()
    {
        HashSet<Shape> initVertices = new(ShapeHasher.Instance);
        HashSet<Shape> initEdges = new(ShapeHasher.Instance);
        HashSet<Shape> newEdges = new(ShapeHasher.Instance);
        Explorer exp = new(_initialShape, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            initEdges.Add(exp.Current());
            for (Iterator iter = new(exp.Current()); iter.More(); iter.Next())
            {
                initVertices.Add(iter.Value());
            }
        }
        //

        BrepBuilder bb = new();
        _vtxToReplace.Clear();
        exp.Init(_shape, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Shape e = exp.Current();
            // Skip old (not modified) edges
            if (initEdges.Contains(e))
            {
                continue;
            }
            //
            // Skip processed edges
            if (newEdges.Contains(e))
            {
                continue;
            }
            //
            newEdges.Add(e);
            //
            double eTol = Tool.Tolerance(ShapeCasts.Edge(e));
            for (Iterator iter = new(e); iter.More(); iter.Next())
            {
                Vertex vtx = ShapeCasts.Vertex(iter.Value());
                if (initVertices.Contains(vtx))
                {
                    if (_vtxToReplace.ContainsKey(vtx))
                    {
                        bb.UpdateVertex(ShapeCasts.Vertex(_vtxToReplace.GetValue(vtx)), eTol + Epsilon(eTol));
                    }
                    else
                    {
                        double vTol = Tool.Tolerance(vtx);
                        if (vTol < eTol)
                        {
                            Vertex newVtx = new();
                            Point3D vPnt = Tool.Pnt(vtx);
                            bb.MakeVertex(ref newVtx, vPnt, eTol + Epsilon(eTol));
                            newVtx.Orientation(vtx.Orientation());
                            _vtxToReplace[vtx] = newVtx;
                        }
                    }
                }
                else
                {
                    bb.UpdateVertex(vtx, eTol + Epsilon(eTol));
                }
            }
        }
        //
        if (_vtxToReplace.Count == 0)
        {
            return;
        }
        //
        _subs.Clear();
        // OCCT iterates the map by the vertices' addresses; the port in insertion order (the requests are independent).
        foreach (KeyValuePair<Shape, Shape> entry in _vtxToReplace)
        {
            _subs.Replace(entry.Key, entry.Value);
        }
        _subs.InternalApply(_shape);
        _shape = _subs.InternalValue(_shape);
        //
    }
}
