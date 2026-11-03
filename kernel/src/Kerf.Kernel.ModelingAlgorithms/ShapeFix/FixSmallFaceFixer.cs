// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FixSmallFace.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_FixSmallFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Fixes faces of small size: spot faces (all vertices within tolerance of a point) are removed and their vertices
/// merged; strip faces (two long edges within tolerance) are removed and their two long edges merged into one; faces
/// with a splitting vertex are split. The faces touched are then fixed with <see cref="FaceFixer"/>.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). <see cref="Init"/> loads the shape; <see cref="Perform"/> (spot and strip faces),
/// <see cref="FixSpotFace"/>, <see cref="FixStripFace"/>, <see cref="FixSplitFace"/> and <see cref="FixShape"/> fix it,
/// recording the changes in the context. They also write into the shape in place (a merged strip's vertex tolerances,
/// the faces' fixes), in a working overlay (Kerf unit 10H4) that takes the writes into frozen shapes (any shape a
/// caller holds): this tool's own from the first fix to <see cref="Shape"/>, or, inside another operation's overlay,
/// that one. Set the context before the first fix. <see cref="Shape"/> gives new TShapes for what changed and the
/// caller's own sub-shapes for the rest (the shape itself when nothing changed), records in the context the
/// sub-shapes changed in place, and freezes the result; a fix after it starts a new overlay. The shapes the fixes
/// return show their changes only inside the overlay: read <see cref="Shape"/>.
/// </para>
/// <para>
/// OCCT's public per-face steps (ReplaceVerticesInCaseOfSpot, RemoveFacesInCaseOfSpot, ReplaceInCaseOfStrip,
/// RemoveFacesInCaseOfStrip, ComputeSharedEdgeForStripFace, SplitOneFace, FixFace) work on the faces of the working
/// copy and write into them: they are internal here.
/// </para>
/// </remarks>
public class FixSmallFaceFixer : RootFixer
{
    // OCCT declares its own private myShape, hiding ShapeFix_Root's (which the messages are attached to).
    private new Shape _shape;
    private Shape _result;
    private int _status;
    private readonly CheckSmallFaceAnalysis _analyzer;

    // Kerf: the working overlay, from the first fix to the public result (unit 10H4): this tool's own (detached) or the
    // current operation's, which it doesn't finish.
    private WorkingOverlay? _overlay;
    private bool _ownsOverlay;
    // True when the next fix starts a new overlay.
    private bool _toEnter;

    /// <summary>Constructor (precision Confusion).</summary>
    public FixSmallFaceFixer()
    {
        _shape    = new Shape();
        _result   = new Shape();
        _analyzer = new CheckSmallFaceAnalysis();
        _status   = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        SetPrecision(Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>Loads the shape (creates a context if none is set).</summary>
    public void Init(in Shape s)
    {
        _shape = s;
        if (Context() is null)
        {
            SetContext(new ReShapeBuilder());
        }
        _result = _shape;
        Context()!.InternalApply(_shape);
        _overlay = null;
        _toEnter = true;
    }

    /// <summary>Fixes the spot faces, then the strip faces.</summary>
    public void Perform()
    {
        FixSpotFace();
        FixStripFace();
    }

    /// <summary>
    /// Fixes the spot faces (removed, their vertices merged into one) and returns the shape, unfrozen until
    /// <see cref="Shape"/> publishes it.
    /// </summary>
    public Shape FixSpotFace()
    {
        using WorkingOverlay.Scope work = EnterWork();
        // gp_Pnt spot;
        // double spotol;
        bool done = false;
        ShapeType st = _shape.ShapeType();
        if (st == ShapeType.Compound || st == ShapeType.CompSolid || st == ShapeType.Solid || st == ShapeType.Shell
            || st == ShapeType.Face)
        {
            for (var itf = new Explorer(_shape, ShapeType.Face); itf.More(); itf.Next())
            {
                // smh#8
                Shape tmpFace = Context()!.InternalApply(itf.Current());
                Face f = ShapeCasts.Face(tmpFace);
                if (f.IsNull)
                {
                    continue;
                }
                if (_analyzer.CheckSpotFace(f, Precision()))
                {
                    ReplaceVerticesInCaseOfSpot(ref f, Precision());
                    RemoveFacesInCaseOfSpot(f);
                    _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                    done = true;
                }
            }
            _shape = Context()!.InternalApply(_shape);
            if (done)
            {
                if (_shape.IsNull)
                {
                    return _shape;
                }
                _shape = FixShape();
            }
            // myShape = Context()->Apply(myShape);
            _result = _shape;
        }
        return _shape;
    }

    /// <summary>Computes the mean vertex of the face and records the replacement of its vertices by it.</summary>
    internal bool ReplaceVerticesInCaseOfSpot(ref Face f, double tol)
    {
        var thePositions = new NCollection_Sequence<Xyz>();
        Xyz thePosition;
        var theBuilder = new BrepBuilder();
        double theMaxDev;
        double theMaxTol = 0.0;
        thePositions.Clear();
        Point3D thePoint;
        // smh#8
        Shape tmpFace = Context()!.InternalApply(f);
        f = ShapeCasts.Face(tmpFace);
        // gka Mar2000 Protection against faces without wires
        // but they occur due to bugs in the algorithm itself, it needs to be fixed
        bool isWir = false;
        for (var itw = new Iterator(f, false); itw.More(); itw.Next())
        {
            if (itw.Value().ShapeType() != ShapeType.Wire)
            {
                continue;
            }
            Wire w1 = ShapeCasts.Wire(itw.Value());
            if (!w1.IsNull)
            {
                isWir = true;
                break;
            }
        }
        if (!isWir)
        {
            return true;
        }

        // Accumulating positions and maximal vertex tolerance
        for (var iter_vertex = new Explorer(f, ShapeType.Vertex); iter_vertex.More(); iter_vertex.Next())
        {
            Vertex v = ShapeCasts.Vertex(iter_vertex.Current());
            thePoint = Tool.Pnt(v);
            if (theMaxTol <= (Tool.Tolerance(v)))
            {
                theMaxTol = Tool.Tolerance(v);
            }
            thePositions.Append(thePoint.XYZ());
        }

        // Calculate common vertex position
        thePosition = new Xyz(0.0, 0.0, 0.0);
        int theNbPos = thePositions.Length();
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= theNbPos; i++)
        {
            thePosition += thePositions.Value(i);
        }
        if (theNbPos > 1)
        {
            thePosition /= theNbPos;
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
        theMaxDev *= 1.00001;

        // Create new vertex with mean point
        var theSharedVertex = new Vertex();
        theBuilder.MakeVertex(ref theSharedVertex);
        theBuilder.UpdateVertex(theSharedVertex, new Point3D(thePosition), theMaxDev + theMaxTol / 2);
        // Use external tolerance
        //   if (tol!=-1.0) theBuilder.UpdateVertex( theSharedVertex, tol);

        // Replacing all vertices in the face by new one
        Vertex theNewVertex;
        for (var iter_vert = new Explorer(f, ShapeType.Vertex); iter_vert.More(); iter_vert.Next())
        {
            Vertex v = ShapeCasts.Vertex(iter_vert.Current());
            if (v.Orientation() == Forward)
            // smh#8
            {
                Shape tmpVertexFwd = theSharedVertex.Oriented(Forward);
                theNewVertex = ShapeCasts.Vertex(tmpVertexFwd);
            }
            else
            // smh#8
            {
                Shape tmpVertexRev = theSharedVertex.Oriented(Reversed);
                theNewVertex = ShapeCasts.Vertex(tmpVertexRev);
            }
            Context()!.Replace(v, theNewVertex);
        }
        return true;
    }

    /// <summary>Records the removal of the spot face and its edges.</summary>
    internal bool RemoveFacesInCaseOfSpot(in Face f)
    {
        for (var iter_vert = new Explorer(f, ShapeType.Edge); iter_vert.More(); iter_vert.Next())
        {
            Edge ed = ShapeCasts.Edge(iter_vert.Current());
            Context()!.Remove(ed);
        }
        Context()!.Remove(f);
        SendWarning(f, new Msg("FixAdvFace.FixSpotFace.MSG0"));
        return true;
    }

    /// <summary>
    /// Fixes the strip faces (removed, their two long edges merged into one) and returns the shape, unfrozen until
    /// <see cref="Shape"/> publishes it.
    /// </summary>
    public Shape FixStripFace(bool wasdone = false)
    {
        using WorkingOverlay.Scope work = EnterWork();
        if (_shape.IsNull)
        {
            return _shape;
        }
        ShapeType st = _shape.ShapeType();
        // BRep_Builder theBuilder;
        bool done = wasdone;
        if (st == ShapeType.Compound || st == ShapeType.CompSolid || st == ShapeType.Solid || st == ShapeType.Shell
            || st == ShapeType.Face)
        {
            for (var itf = new Explorer(_shape, ShapeType.Face); itf.More(); itf.Next())
            {
                Face f = ShapeCasts.Face(itf.Current());
                // smh#8
                Shape tmpFace = Context()!.InternalApply(f);
                f = ShapeCasts.Face(tmpFace);
                if (f.IsNull)
                {
                    continue;
                }
                // double dmax = 1;
                Edge e1 = new(), e2 = new();
                if (_analyzer.CheckStripFace(f, ref e1, ref e2, Precision()))
                {
                    if (ReplaceInCaseOfStrip(ref f, ref e1, ref e2, Precision()))
                    {
                        RemoveFacesInCaseOfStrip(f);
                    }
                    _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                    done = true;
                }
            }
            _shape = Context()!.InternalApply(_shape);
            // Particular case of empty shell
            if (!_shape.IsNull)
            {
                for (var exp_s = new Explorer(_shape, ShapeType.Shell); exp_s.More(); exp_s.Next())
                {
                    Shell sh = ShapeCasts.Shell(exp_s.Current());
                    var ex_sh = new Explorer(sh, ShapeType.Face);
                    if (!ex_sh.More())
                    {
                        Context()!.Remove(sh);
                        // std::cout << "Empty shell was removed" << std::endl;
                    }
                }
                _shape = Context()!.InternalApply(_shape);
                // Fixing of missing pcurves on new edges, if they were inserted
                if (done)
                {
                    if (_shape.IsNull)
                    {
                        return _shape;
                    }
                    _shape = FixShape();
                    // myShape = Context()->Apply(myShape);
                    _result = _shape;
                }
            }
        }
        return _shape;
    }

    /// <summary>Records the replacement of the strip face's two long edges by one and the removal of its short ones.</summary>
    internal bool ReplaceInCaseOfStrip(ref Face f, ref Edge e1, ref Edge e2, double tol)
    {
        if (e1.IsNull || e2.IsNull)
        {
            return false;
        }
        Edge theSharedEdge;
        Face f1 = new(), f2 = new();
        // smh#8
        Shape tmpFace = Context()!.InternalApply(f);
        f = ShapeCasts.Face(tmpFace);
        for (var expf = new Explorer(_shape, ShapeType.Face); expf.More(); expf.Next())
        {
            // smh#8
            Shape tmpShape = Context()!.InternalApply(expf.Current());
            Face tempF = ShapeCasts.Face(tmpShape);
            if (tempF.IsNull || tempF.IsSame(f))
            {
                continue;
            }
            for (var expe = new Explorer(tempF, ShapeType.Edge); expe.More(); expe.Next())
            {
                Edge tempE = ShapeCasts.Edge(expe.Current());
                if (tempE.IsSame(e1))
                {
                    f1 = tempF;
                }
                if (tempE.IsSame(e2))
                {
                    f2 = tempF;
                }
                if (!f1.IsNull)
                {
                    break; // && !F2.IsNull()) break;
                }
            }
        }

        // Compute shared edge for this face
        if (f1.IsNull && f2.IsNull)
        {
            return true;
        }
        Edge e1tmp = e1;
        Edge e2tmp = e2;
        if (f1.IsNull)
        {
            e1tmp = e2;
            e2tmp = e1;
            f1 = f2;
        }
        theSharedEdge = ComputeSharedEdgeForStripFace(f, e1tmp, e2tmp, f1, tol);

        // Replace two long edges by new one
        if (theSharedEdge.IsNull)
        {
            return false;
        }
        if (e1.Orientation() == Reversed)
        {
            Context()!.Replace(e1tmp, theSharedEdge.Oriented(Reversed));
            if (f.Orientation() == f1.Orientation())
            {
                Context()!.Replace(e2tmp, theSharedEdge);
            }
            else
            {
                Context()!.Replace(e2tmp, theSharedEdge.Oriented(Reversed));
            }
        }
        else
        {
            Context()!.Replace(e1tmp, theSharedEdge);
            if (f.Orientation() == f1.Orientation())
            {
                Context()!.Replace(e2tmp, theSharedEdge.Oriented(Reversed));
            }
            else
            {
                Context()!.Replace(e2tmp, theSharedEdge);
            }
        }

        // Remove short edges
        for (var exp_e = new Explorer(f, ShapeType.Edge); exp_e.More(); exp_e.Next())
        {
            Edge shortedge = ShapeCasts.Edge(exp_e.Current());
            if (!shortedge.IsSame(e1tmp) && !shortedge.IsSame(e2tmp))
            {
                Context()!.Remove(shortedge);
            }
        }

        return true;
    }

    /// <summary>Records the removal of the strip face.</summary>
    internal bool RemoveFacesInCaseOfStrip(in Face f)
    {
        Context()!.Remove(f);
        SendWarning(f, new Msg("FixAdvFace.FixStripFace.MSG0"));
        return true;
    }

    /// <summary>
    /// Computes the edge replacing the two long edges of a strip face, recording the replacement of their vertices;
    /// a kept vertex's tolerance is raised in place.
    /// </summary>
    internal Edge ComputeSharedEdgeForStripFace(in Face F,
                                                       in Edge e1,
                                                       in Edge e2,
                                                       in Face f1,
                                                       double tol)
    {
        var theBuilder = new BrepBuilder();
        // Compute deviation between two vertices and create new vertices
        var theNewEdge = new Edge();
        Exploration.Vertices(e1, out Vertex v1, out Vertex v2);
        Exploration.Vertices(e2, out Vertex v3, out Vertex v4);
        Point3D p1, p2;
        double dev;
        p1 = Tool.Pnt(v1);
        p2 = Tool.Pnt(v3);
        dev = p1.Distance(p2);
        var theFirstVer = new Vertex();
        var theSecondVer = new Vertex();
        theBuilder.MakeVertex(ref theFirstVer);
        theBuilder.MakeVertex(ref theSecondVer);
        Xyz thePosition;

        if ((dev <= Tool.Tolerance(v1)) || (dev <= Tool.Tolerance(v3)) || (dev <= tol))
        {
            if (v1.IsSame(v3))
            {
                // #ifdef AIX  CKY : applies to all platforms
                theFirstVer = v1;
                //	theFirstVer = TopoDS::Vertex(V1);
            }
            else
            {
                dev = (dev / 2) * 1.0001;
                thePosition = (p1.XYZ() + p2.XYZ()) / 2;
                theBuilder.UpdateVertex(theFirstVer, new Point3D(thePosition), dev);

                if (v1.Orientation() == Forward)
                {
                    Context()!.Replace(v1, theFirstVer.Oriented(Forward));
                }
                else
                {
                    Context()!.Replace(v1, theFirstVer.Oriented(Reversed));
                }
                if (v3.Orientation() == Forward)
                {
                    Context()!.Replace(v3, theFirstVer.Oriented(Forward));
                }
                else
                {
                    Context()!.Replace(v3, theFirstVer.Oriented(Reversed));
                }
            }
            if (v1.IsSame(v2) || v3.IsSame(v4))
            {
                theSecondVer = theFirstVer;
            }
            else
            {
                if (!v2.IsSame(v4))
                {
                    // #ifdef AIX  CKY : applies to all platforms
                    p1 = Tool.Pnt(v2);
                    p2 = Tool.Pnt(v4);
                    dev = p1.Distance(p2);
                    thePosition = (p1.XYZ() + p2.XYZ()) / 2;
                    theBuilder.UpdateVertex(theSecondVer, new Point3D(thePosition), dev);
                }
                else
                {
                    theSecondVer = v2;
                }
            }
            if (!v2.IsSame(theSecondVer))
            {
                if (v2.Orientation() == Forward)
                {
                    Context()!.Replace(v2, theSecondVer.Oriented(Forward));
                }
                else
                {
                    Context()!.Replace(v2, theSecondVer.Oriented(Reversed));
                }
                if (v4.Orientation() == Forward)
                {
                    Context()!.Replace(v4, theSecondVer.Oriented(Forward));
                }
                else
                {
                    Context()!.Replace(v4, theSecondVer.Oriented(Reversed));
                }
            }
        }
        else
        {
            p2 = Tool.Pnt(v4);
            dev = p1.Distance(p2);
            if ((dev <= Tool.Tolerance(v1)) || (dev <= Tool.Tolerance(v4)) || (dev <= tol))
            {
                if (v1.IsSame(v4))
                {
                    // #ifdef AIX  CKY : applies to all platforms
                    theFirstVer = v1;
                    //	    theFirstVer = TopoDS::Vertex(V1);
                }
                else
                {
                    dev = (dev / 2) * 1.0001;
                    thePosition = (p1.XYZ() + p2.XYZ()) / 2;
                    theBuilder.UpdateVertex(theFirstVer, new Point3D(thePosition), dev);
                    if (v1.Orientation() == Forward)
                    {
                        Context()!.Replace(v1, theFirstVer.Oriented(Forward));
                    }
                    else
                    {
                        Context()!.Replace(v1, theFirstVer.Oriented(Reversed));
                    }
                    if (v4.Orientation() == Forward)
                    {
                        Context()!.Replace(v4, theFirstVer.Oriented(Forward));
                    }
                    else
                    {
                        Context()!.Replace(v4, theFirstVer.Oriented(Reversed));
                    }
                }
                if (v1.IsSame(v2) || v3.IsSame(v4))
                {
                    theSecondVer = theFirstVer;
                }
                else
                {
                    if (!v2.IsSame(v3))
                    {
                        p1 = Tool.Pnt(v2);
                        p2 = Tool.Pnt(v3);
                        dev = p1.Distance(p2);
                        thePosition = (p1.XYZ() + p2.XYZ()) / 2;
                        theBuilder.UpdateVertex(theSecondVer, new Point3D(thePosition), dev);
                    }
                    else
                    {
                        theSecondVer = v2;
                    }
                }
                if (!v2.IsSame(theSecondVer))
                {
                    if (v2.Orientation() == Forward)
                    {
                        Context()!.Replace(v2, theSecondVer.Oriented(Forward));
                    }
                    else
                    {
                        Context()!.Replace(v2, theSecondVer.Oriented(Reversed));
                    }
                    if (v3.Orientation() == Forward)
                    {
                        Context()!.Replace(v3, theSecondVer.Oriented(Forward));
                    }
                    else
                    {
                        Context()!.Replace(v3, theSecondVer.Oriented(Reversed));
                    }
                }
            }
            else
            {
                // The face is not strip face (OCCT_DEBUG message)
                return theNewEdge;
            }
        }
        if (theFirstVer.IsNull || theSecondVer.IsNull)
        {
            return theNewEdge;
        }

        // Create new edge
        theBuilder.MakeEdge(ref theNewEdge);
        double f, l, fp1 = 0.0, lp1 = 0.0 /*, fp2, lp2*/;
        Curve3D? the3dcurve;
        the3dcurve = Tool.Curve(e1, out f, out l);
        Curve2D? the2dcurve1, thenew1 = null;
        if (!f1.IsNull)
        {
            the2dcurve1 = Tool.CurveOnSurface(e1, f1, out fp1, out lp1);
            if (the2dcurve1 is not null && fp1 != f && lp1 != l)
            {
                global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(Kerf.Engine.Foundation.Precision.Confusion(), the2dcurve1, fp1, lp1, f, l, ref thenew1);
            }
        }

        double maxdev;
        if ((Tool.Tolerance(theFirstVer)) <= (Tool.Tolerance(theSecondVer)))
        {
            maxdev = (Tool.Tolerance(theSecondVer));
        }
        else
        {
            maxdev = (Tool.Tolerance(theFirstVer));
        }
        // In place: when theFirstVer (theSecondVer) is V1 (V2), the strip's own vertex.
        theBuilder.UpdateVertex(theFirstVer, maxdev);
        theBuilder.UpdateVertex(theSecondVer, maxdev);
        // bool IsFree = true;
        theBuilder.SameParameter(theNewEdge, false);
        the3dcurve = Tool.Curve(e1, out f, out l);
        theBuilder.UpdateEdge(theNewEdge, the3dcurve, maxdev);
        theBuilder.Range(theNewEdge, f, l);
        if (!f1.IsNull && thenew1 is not null)
        {
            theBuilder.UpdateEdge(theNewEdge, thenew1, f1, maxdev);
            // IsFree = false;
        }
        theBuilder.Add(theNewEdge, theFirstVer.Oriented(Forward));
        theBuilder.Add(theNewEdge, theSecondVer.Oriented(Reversed));
        return theNewEdge;
    }

    /// <summary>
    /// Splits the faces of the shape at their splitting vertices (<paramref name="s"/> is ignored, as in OCCT) and
    /// returns the shape, unfrozen until <see cref="Shape"/> publishes it.
    /// </summary>
    public Shape FixSplitFace(in Shape s)
    {
        using WorkingOverlay.Scope work = EnterWork();
        if (_shape.IsNull)
        {
            return _shape;
        }
        ShapeType st = _shape.ShapeType();
        bool done = false;
        var theBuilder = new BrepBuilder();
        if (st == ShapeType.Compound || st == ShapeType.CompSolid || st == ShapeType.Solid || st == ShapeType.Shell
            || st == ShapeType.Face)
        {
            for (var itf = new Explorer(_shape, ShapeType.Face); itf.More(); itf.Next())
            {
                Face f = ShapeCasts.Face(itf.Current());
                var compSplittedFaces = new Compound();
                theBuilder.MakeCompound(ref compSplittedFaces);
                if (SplitOneFace(ref f, compSplittedFaces))
                {
                    done = true;
                    Context()!.Replace(f, compSplittedFaces);
                }
            }
        }
        if (done)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
        }
        _shape = Context()!.InternalApply(_shape);
        _result = _shape;
        return _shape;
    }

    /// <summary>
    /// Splits the face at its first splitting vertex, recursively, adding the faces to
    /// <paramref name="splittedFaces"/> (an unfrozen compound the caller owns); false if it isn't split.
    /// </summary>
    internal bool SplitOneFace(ref Face F, in Compound splittedFaces)
    {
        var mapEdges = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var mapParam = new NCollection_DataMap<Shape, NCollection_List<double>>(ShapeHasher.Instance);
        var theAllVert = new Compound();
        var theBuilder = new BrepBuilder();
        theBuilder.MakeCompound(ref theAllVert);
        // smh#8
        Shape tmpShape = Context()!.InternalApply(F);
        F = ShapeCasts.Face(tmpShape);
        // OCCT.NET: the in-place form, into the compound this method owns (each splitting vertex comes twice, as in OCCT)
        if (_analyzer.InternalCheckSplittingVertices(F, mapEdges, mapParam, theAllVert) != 0)
        {
            Wire tempwire = new();
            // Take information about splitting vertices
            if (theAllVert.IsNull)
            {
                return false;
            }
            // int i;
            Vertex v;
            var itc = new Explorer(theAllVert, ShapeType.Vertex);
            v = ShapeCasts.Vertex(itc.Current());
            if (v.IsNull)
            {
                return false;
            }
            Point3D proj = new();
            Point3D vp = Tool.Pnt(v);
            var theNewVertex = new Vertex();
            var e = new Edge();
            var theFirstEdge = new Edge();
            var theSecondEdge = new Edge();
            {
                // If one vertex presents do splitting by two faces
                var sac = new CurveAnalysis();
                for (var ite = new Explorer(F, ShapeType.Edge); ite.More(); ite.Next())
                {
                    e = ShapeCasts.Edge(ite.Current());
                    Exploration.Vertices(e, out Vertex v1, out Vertex v2);
                    Curve3D? c3D = Tool.Curve(e, out double cf, out double cl);
                    if (c3D is null)
                    {
                        continue;
                    }
                    if (v.IsSame(v1) || v.IsSame(v2))
                    {
                        continue;
                    }
                    double vt = Tool.Tolerance(v);
                    double dist = sac.Project(c3D, vp, vt * 10.0, out proj, out double param, cf, cl);
                    if (dist == 0)
                    {
                        continue; // Projection on same curve but on other edge ?
                    }
                    if (dist <= vt)
                    {
                        theBuilder.MakeVertex(ref theNewVertex);
                        theBuilder.UpdateVertex(theNewVertex, proj, Kerf.Engine.Foundation.Precision.Confusion());
                        theBuilder.MakeEdge(ref theFirstEdge);
                        theBuilder.MakeEdge(ref theSecondEdge);
                        Curve3D? the3dcurve = Tool.Curve(e, out double f, out double l);
                        theBuilder.UpdateEdge(theFirstEdge, the3dcurve, Kerf.Engine.Foundation.Precision.Confusion());
                        theBuilder.UpdateEdge(theSecondEdge, the3dcurve, Kerf.Engine.Foundation.Precision.Confusion());
                        if (v1.Orientation() == Forward)
                        {
                            theBuilder.Add(theFirstEdge, v1);
                            theBuilder.Add(theFirstEdge, theNewVertex.Oriented(Reversed));
                            theBuilder.Add(theSecondEdge, theNewVertex.Oriented(Forward));
                            theBuilder.Add(theSecondEdge, v2);
                        }
                        else
                        {
                            theBuilder.Add(theFirstEdge, v2);
                            theBuilder.Add(theFirstEdge, theNewVertex.Oriented(Reversed));
                            theBuilder.Add(theSecondEdge, theNewVertex.Oriented(Forward));
                            theBuilder.Add(theSecondEdge, v1);
                        }
                        theBuilder.Range(theFirstEdge, cf, param);
                        theBuilder.Range(theSecondEdge, param, cl);

                        // Replace old edge by two new edges
                        var twoedges = new Wire();
                        theBuilder.MakeWire(ref twoedges);
                        if (e.Orientation() == Forward)
                        {
                            theBuilder.Add(twoedges, theFirstEdge.Oriented(Forward));
                            theBuilder.Add(twoedges, theSecondEdge.Oriented(Forward));
                        }
                        else
                        {
                            theBuilder.Add(twoedges, theFirstEdge.Oriented(Reversed));
                            theBuilder.Add(twoedges, theSecondEdge.Oriented(Reversed));
                        }
                        Context()!.Replace(e, twoedges);
                        break;
                    }
                }
                if (theNewVertex.IsNull)
                {
                    return false;
                }

                // Create split edge
                var theSplitEdge = new Edge();
                var lin = new Line3D(vp, new Direction3D(new Vector3D(vp, proj)));
                double firstparam = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, vp);
                double lastparam = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, proj);
                var L = new LineCurve3D(vp, new Direction3D(new Vector3D(vp, proj))); // C++ converts the gp_Vec implicitly
                Curve3D the3dc = L;
                theBuilder.MakeEdge(ref theSplitEdge, the3dc, Kerf.Engine.Foundation.Precision.Confusion());
                theBuilder.Add(theSplitEdge, v.Oriented(Forward));
                theBuilder.Add(theSplitEdge, theNewVertex.Oriented(Reversed));
                theBuilder.Range(theSplitEdge, firstparam, lastparam);

                // Add pcurve in new edge
                var sfe = new EdgeFixer();
                sfe.FixAddPCurve(theSplitEdge, F, false);

                // Reorder the wire
                Wire wireonface;
                // Inher loop is not support yet !!!
                var itw = new Explorer(F, ShapeType.Wire);
                wireonface = ShapeCasts.Wire(itw.Current());
                itw.Next();
                if (itw.More())
                {
                    return false; // if face contains more than one wire
                }
                var sfw = new WireFixer();
                sfw.Init(wireonface, F, Kerf.Engine.Foundation.Precision.Confusion());
                sfw.FixReorder();
                wireonface = sfw.InternalWire(); // OCCT.NET: written below

                // Create two new wires
                Wire w1 = new(), w2 = new();
                theBuilder.MakeWire(ref w1);
                theBuilder.MakeWire(ref w2);
                theBuilder.MakeWire(ref tempwire);
                // OCCT removes and adds edges of the wire while exploring it (its list iterator then reads the removed,
                // freed node to go on); the port explores the edges the wire had first, then the two it appends, which
                // OCCT visits too (occt-issues.md).
                var edges = new List<Edge>();
                for (var itnew = new Explorer(wireonface, ShapeType.Edge); itnew.More(); itnew.Next())
                {
                    edges.Add(ShapeCasts.Edge(itnew.Current()));
                }
                foreach (Edge ce in edges)
                {
                    if (ce.IsSame(e))
                    {
                        theBuilder.Remove(wireonface, ce);
                        theBuilder.Add(wireonface, theFirstEdge.Oriented(Forward));
                        theBuilder.Add(wireonface, theSecondEdge.Oriented(Forward));
                    }
                }
                sfw.Init(wireonface, F, Kerf.Engine.Foundation.Precision.Confusion());
                sfw.FixReorder();
                wireonface = sfw.InternalWire();
                for (var itere = new Explorer(wireonface, ShapeType.Edge); itere.More(); itere.Next())
                {
                    Edge ce = ShapeCasts.Edge(itere.Current());
                    Vertex thecontrol;
                    if (ce.Orientation() == Forward)
                    {
                        thecontrol = Exploration.LastVertex(ce);
                    }
                    else
                    {
                        thecontrol = Exploration.FirstVertex(ce);
                    }
                    theBuilder.Add(w1, ce);
                    if (thecontrol.IsSame(v))
                    {
                        theBuilder.Add(w1, theSplitEdge.Oriented(Forward));
                        Wire wtemp = w1;
                        w1 = w2;
                        w2 = wtemp;
                    }
                    if (thecontrol.IsSame(theNewVertex))
                    {
                        theBuilder.Add(w1, theSplitEdge.Oriented(Reversed));
                        Wire wtemp = w1;
                        w1 = w2;
                        w2 = wtemp;
                    }
                }
                if (w1.IsNull || w2.IsNull)
                {
                    return false;
                }

                // Create two new faces and replace old one
                var f1 = new Face();
                var f2 = new Face();
                theBuilder.MakeFace(ref f1, Tool.Surface(F), Kerf.Engine.Foundation.Precision.Confusion());
                theBuilder.MakeFace(ref f2, Tool.Surface(F), Kerf.Engine.Foundation.Precision.Confusion());
                theBuilder.Add(f1, w1);
                theBuilder.Add(f2, w2);
                var tf = new Compound();
                theBuilder.MakeCompound(ref tf);
                theBuilder.Add(tf, f1);
                theBuilder.Add(tf, f2);

                // Call recursive spliteoneface() for each face
                if (!SplitOneFace(ref f1, splittedFaces))
                {
                    theBuilder.Add(splittedFaces, f1);
                }
                if (!SplitOneFace(ref f2, splittedFaces))
                {
                    theBuilder.Add(splittedFaces, f2);
                }
            }
            return true;
        }
        return false;
    }

    /// <summary>Fixes a face with ShapeFix_Face (sharing the context); returns the fixed face, unfrozen.</summary>
    internal Face FixFace(in Face f)
    {
        // smh#8
        Shape emptyCopied = f.EmptyCopied();
        Face theFixedFace = ShapeCasts.Face(emptyCopied);
        var sff = new FaceFixer();
        sff.SetContext(Context());
        sff.Init(f);
        sff.Perform();
        // sff->Init(theFixedFace);
        // sff->FixOrientation();
        theFixedFace = sff.InternalFace(); // OCCT.NET: the face of the shape being fixed
        return theFixedFace;
    }

    /// <summary>
    /// Fixes every face of the shape with ShapeFix_Face and returns the shape with the context applied, unfrozen
    /// until <see cref="Shape"/> publishes it.
    /// </summary>
    public Shape FixShape()
    {
        using WorkingOverlay.Scope work = EnterWork();
        Shape fixSh = new();
        if (_shape.IsNull)
        {
            return fixSh;
        }
        for (var expf = new Explorer(_shape, ShapeType.Face); expf.More(); expf.Next())
        {
            Face f = ShapeCasts.Face(expf.Current());
            // smh#8
            Shape tmpFace = Context()!.InternalApply(f);
            f = ShapeCasts.Face(tmpFace);
            Face newF = FixFace(f);
            Context()!.Replace(f, newF);
        }
        fixSh = Context()!.InternalApply(_shape);
        return fixSh;
    }

    /// <summary>
    /// The resulting shape, frozen: the caller's own sub-shapes wherever the fixes left them unchanged (the loaded
    /// shape itself when nothing changed). The context then answers for the caller's shapes.
    /// </summary>
    public Shape Shape()
    {
        EndWork();
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    /// <summary>The resulting shape, unfrozen (ported code fixing a shape it owns).</summary>
    internal Shape InternalShape() => _shape;

    /// <summary>Fixes a pin face: does nothing (OCCT's stub).</summary>
    public bool FixPinFace(ref Face f) => true;

    // Kerf: at the first fix after Init or a public result, takes the overlay the fixes work in; then enters it.
    private WorkingOverlay.Scope EnterWork()
    {
        if (_toEnter)
        {
            _toEnter = false;
            _overlay = WorkingOverlay.ForTool(_shape, Context(), out _ownsOverlay);
        }
        return _overlay?.Enter() ?? default;
    }

    // Kerf: before a public result, finishes this tool's overlay: new TShapes for what changed, the caller's own for
    // the rest, the context brought up to date.
    private void EndWork()
    {
        _toEnter = true;
        WorkingOverlay? overlay = _ownsOverlay ? _overlay : null;
        _overlay     = null;
        _ownsOverlay = false;
        if (overlay is null)
        {
            return;
        }
        _shape = overlay.Finish(_shape, [_result]);
        _result = overlay.Final(_result);
    }
}
