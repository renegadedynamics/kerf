// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_ComposeShell.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_ComposeShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//    pdn  01.06.99 S4205: handling not-SameRange edges
//    abv  22.07.99 implementing patch indices
//    svv  10.01.00 porting on DEC

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.TransformForm;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Creates a shell from a composite surface (a grid of surfaces) and a set of wires: splits a face, if needed, by the
/// joint lines of the grid (its U- and V-isolines), each new face lying on one patch. ShapeFix_Face uses it with a
/// one-patch grid to add a missing seam.
/// </summary>
/// <remarks>
/// <para>
/// Two modes, which differ on periodic surfaces: with <see cref="ClosedMode"/> false (the default) the splitting is
/// done as if the surface weren't periodic, the splitting lines being repeated over the face's whole parametric range;
/// with it true everything on a periodic surface is taken modulo the period, which deals with wires closed in 3d but
/// not in 2d. The work is based on the pcurves of the edges on the given face. All modifications are recorded in the
/// context, which must be set.
/// </para>
/// <para>
/// A working object (decision 21). It edits the edges of the face it is given (it splits them, records the splits in
/// the context and replaces their vertices), as OCCT's, in a working overlay (Kerf unit 10H4) that takes the writes
/// into frozen shapes: this tool's own from <see cref="Perform"/> to <see cref="Result"/>, or, inside another
/// operation's overlay (ShapeFix_Face's, ShapeUpgrade's dividers'), that one. <see cref="Result"/> gives new TShapes
/// for what changed and the caller's own sub-shapes for the rest, records in the context the sub-shapes changed in
/// place, and freezes the result.
/// </para>
/// </remarks>
internal class ComposeShellFixer : RootFixer
{
    /// <summary>The orientation of the face given.</summary>
    protected Orientation _orient;
    /// <summary>The resulting shell or face.</summary>
    protected Shape _result;
    /// <summary>The status of the last <see cref="Perform"/>.</summary>
    protected int _status;

    private CompositeSurface?     _grid;
    private Placement                   _loc;
    private Face                       _face;
    private double                            _uResolution;
    private double                            _vResolution;
    private TransferParametersAnalysis  _transferParamTool;
    private bool                              _invertEdgeStatus;
    private bool                              _closedMode;
    private bool                              _uClosed;
    private bool                              _vClosed;
    private double                            _uPeriod;
    private double                            _vPeriod;

    // Kerf: the working overlay (unit 10H4): this tool's own (detached) or the current operation's, which it doesn't
    // finish; null before Perform.
    private WorkingOverlay? _overlay;
    private bool _ownsOverlay;
    private bool _toEnter;

    /// <summary>Creates an empty tool.</summary>
    public ComposeShellFixer()
    {
        _orient            = Forward;
        _status            = 0;
        _result            = new Shape();
        _loc               = new Placement();
        _face              = new Face();
        _uResolution       = RealLast();
        _vResolution       = RealLast();
        _invertEdgeStatus  = true;
        _closedMode        = false;
        _uClosed           = false;
        _vClosed           = false;
        _uPeriod           = 0.0;
        _vPeriod           = 0.0;
        _transferParamTool = new TransferParametersProjAnalysis();
    }

    /// <summary>
    /// Initializes with a composite surface, a face and a precision. The face gives both the set of wires and the way
    /// of getting pcurves. The precision is used (with the edges' tolerances) for subtle cases, such as tangential
    /// intersections.
    /// </summary>
    public void Init(CompositeSurface grid, in Placement l, in Face face, double prec)
    {
        _grid    = grid;
        _uClosed = _grid.IsUClosed;
        _vClosed = _grid.IsVClosed;
        _uPeriod = _grid.UJointValue(_grid.UPatchCount + 1) - _grid.UJointValue(1);
        _vPeriod = _grid.VJointValue(_grid.VPatchCount + 1) - _grid.VJointValue(1);

        //  DTK-CKY 100531 : protection against very thin face
        //  Test "isclosed" should be filtered on the overall (non trimmed) surface, must be closed
        Surface theSurface = Tool.Surface(face, out _loc)!;
        // avoid false detection of 'Closed' on very thin faces
        if (theSurface is ElementarySurface)
        {
            _uClosed = _uClosed && theSurface.IsUClosed;
            _vClosed = _vClosed && theSurface.IsVClosed;
        }
        else
        {
            double gu0 = 0.0, gu1 = 0.0, gv0 = 0.0, gv1 = 0.0;
            theSurface.ParameterBounds(out double u0, out double U1, out double v0, out double V1);
            if (Kerf.Engine.Foundation.Precision.IsInfinite(u0) || Kerf.Engine.Foundation.Precision.IsInfinite(U1) || Kerf.Engine.Foundation.Precision.IsInfinite(v0)
                || Kerf.Engine.Foundation.Precision.IsInfinite(V1))
            {
                BrepTools.UVBounds(face, out gu0, out gu1, out gv0, out gv1);
            }
            if (_uClosed)
            {
                if (Kerf.Engine.Foundation.Precision.IsInfinite(v0))
                {
                    v0 = gv0;
                }
                if (Kerf.Engine.Foundation.Precision.IsInfinite(V1))
                {
                    V1 = gv1;
                }
                Point3D p0 = theSurface.Value(u0, (v0 + V1) / 2.0);
                Point3D p1 = theSurface.Value(U1, (v0 + V1) / 2.0);
                if (p0.DistanceTo(p1) > Kerf.Engine.Foundation.Precision.Confusion() * 10)
                {
                    _uClosed = false;
                }
            }
            if (_vClosed)
            {
                if (Kerf.Engine.Foundation.Precision.IsInfinite(u0))
                {
                    u0 = gu0;
                }
                if (Kerf.Engine.Foundation.Precision.IsInfinite(U1))
                {
                    U1 = gu1;
                }
                Point3D p0 = theSurface.Value((u0 + U1) / 2.0, v0);
                Point3D p1 = theSurface.Value((u0 + U1) / 2.0, V1);
                if (p0.DistanceTo(p1) > Kerf.Engine.Foundation.Precision.Confusion() * 10)
                {
                    _vClosed = false;
                }
            }
        }
        //  DTK-CKY 100531 end

        _loc = l;
        // smh#8
        Shape tmpF = face.Oriented(Forward);
        _face   = ShapeCasts.Face(tmpF); // for correct dealing with seams
        _orient = face.Orientation;
        SetPrecision(prec);
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);

        // Compute resolution (checking in 2d is necessary for splitting
        // degenerated edges and avoiding NotClosed)
        _uResolution = _vResolution = RealLast();
        for (int i = 1; i <= _grid.UPatchCount; i++)
        {
            double uRange = _grid.UJointValue(i + 1) - _grid.UJointValue(i);
            for (int j = 1; j <= _grid.VPatchCount; j++)
            {
                double vRange = _grid.VJointValue(j + 1) - _grid.VJointValue(j);
                _grid.Patch(i, j).ParameterBounds(out double u1, out double u2, out double v1, out double v2);
                var gas = new SurfaceAdaptor(_grid.Patch(i, j));
                double ures = gas.UResolution(1.0) * uRange / (u2 - u1);
                double vres = gas.VResolution(1.0) * vRange / (v2 - v1);
                if (ures > 0.0 && _uResolution > ures)
                {
                    _uResolution = ures;
                }
                if (vres > 0.0 && _vResolution > vres)
                {
                    _vResolution = vres;
                }
            }
        }
        if (_uResolution == RealLast())
        {
            _uResolution = Kerf.Engine.Foundation.Precision.Parametric(1.0);
        }
        if (_vResolution == RealLast())
        {
            _vResolution = Kerf.Engine.Foundation.Precision.Parametric(1.0);
        }
        _toEnter = true;
        _overlay = null;
    }

    /// <summary>
    /// Performs the work on the loaded data: splits the face by the grid and builds the faces. Returns true if done
    /// (status DONE1; FAIL1 misoriented wire handled, FAIL2 recoverable parity error, FAIL3 edge without pcurve, FAIL4
    /// unrecoverable parity error, FAIL6 no wire).
    /// </summary>
    public virtual bool Perform()
    {
        using WorkingOverlay.Scope work = EnterWork();
        _status           = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        _invertEdgeStatus = false;

        var seqw = new NCollection_Sequence<WireSegmentFixer>(); // working data: wire segments

        // Init seqw by initial set of wires (with corresponding orientation)
        LoadWires(seqw);
        if (seqw.Length() == 0)
        {
            _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL6);
            return false;
        }

        // Split edges in the wires by grid and add internal segments of grid (parts of cutting lines)
        SplitByGrid(seqw);

        // Split all the wires into segments by common vertices (intersections)
        BreakWires(seqw);

        // Then, collect resulting wires
        var wires = new NCollection_Sequence<WireSegmentFixer>(); // resulting wires
        CollectWires(wires, seqw);

        // And construct resulting faces
        var faces = new NCollection_Sequence<Shape>();
        DispatchWires(faces, wires);

        // Finally, construct resulting shell
        if (faces.Length() != 1)
        {
            var s = new Shell();
            var b = new BrepBuilder();
            b.MakeShell(ref s);
            for (int i = 1; i <= faces.Length(); i++)
            {
                b.Add(s, faces[i]);
            }
            _result = s;
        }
        else
        {
            _result = faces[1];
        }
        _result.Orient(_orient);

        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        return true;
    }

    /// <summary>
    /// Splits the edges of the face by the grid (a part of <see cref="Perform"/> producing no shape: the splits are
    /// recorded in the context, an edge replaced by a wire whose edges follow its FORWARD orientation).
    /// </summary>
    public void SplitEdges()
    {
        using WorkingOverlay.Scope work = EnterWork();
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);

        var seqw = new NCollection_Sequence<WireSegmentFixer>(); // working data: wire segments

        // Init seqw by initial set of wires (with corresponding orientation)
        LoadWires(seqw);

        // Split edges in the wires by grid and add internal segments of grid (parts of cutting lines)
        SplitByGrid(seqw);
    }

    /// <summary>
    /// Returns the resulting shell or face (null if not done), frozen. Where the face given was frozen, its sub-shapes
    /// left unchanged are the caller's own, and the context answers for the caller's shapes.
    /// </summary>
    public Shape Result()
    {
        Shape result = InternalFinishedResult();
        result.TShape()?.FreezeGraph();
        return result;
    }

    /// <summary>The resulting shell or face, unfrozen, for ported callers (ShapeFix_Face).</summary>
    internal Shape InternalResult() => _result;

    /// <summary>
    /// <see cref="Result"/> without freezing it, for ported callers that may give a frozen face (ShapeUpgrade's face
    /// dividers): where this tool's own overlay took the writes, it is finished, as <see cref="Result"/> does; else the
    /// result is the one built in place.
    /// </summary>
    internal Shape InternalFinishedResult()
    {
        _toEnter = true;
        WorkingOverlay? overlay = _ownsOverlay ? _overlay : null;
        _overlay     = null;
        _ownsOverlay = false;
        if (overlay is not null)
        {
            _result = overlay.Finish(_result, [_face]);
            _face = ShapeCasts.Face(overlay.Final(_face));
        }
        return _result;
    }

    /// <summary>Queries the status of the last <see cref="Perform"/>.</summary>
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    // Kerf: takes the overlay the work runs in (after Init or a result), and enters it (unit 10H4).
    private WorkingOverlay.Scope EnterWork()
    {
        if (_toEnter)
        {
            _toEnter     = false;
            _overlay     = null;
            _ownsOverlay = false;
            if (!_face.IsNull)
            {
                _overlay = WorkingOverlay.ForTool(_face, Context(), out _ownsOverlay);
            }
        }
        return _overlay?.Enter() ?? default;
    }

    //=======================================================================
    // PRIVATE (working) METHODS
    //=======================================================================

    private const double Tolint = 1.0e-10; // precision for intersection

    // Local definitions: characteristics of intersection point

    private const int IOR_UNDEF = 0; // undefined side
    private const int IOR_LEFT  = 1; // to left side of cutting line
    private const int IOR_RIGHT = 2; // to right side of cutting line
    private const int IOR_BOTH  = 3; // crossing
    private const int IOR_POS   = 4; // in case of cycle on full period, whether first point is right

    private const int ITP_INTER  = 8;  // crossing
    private const int ITP_BEGSEG = 16; // start of tangential segment
    private const int ITP_ENDSEG = 32; // stop of tangential segment
    private const int ITP_TANG   = 64; // tangential point

    // Return (signed) deviation of point from line
    private static double PointLineDeviation(in Point2D p, in Line2D line)
    {
        Direction2D dir = line.Direction;
        var n = new Direction2D(-dir.Y, dir.X);
        return n.XY() * (p.XY() - line.Location.XY());
    }

    // Define position of point relative to line
    private static int PointLinePosition(in Point2D p, in Line2D line, out double dev)
    {
        dev = PointLineDeviation(p, line);
        return (dev > Tolint ? IOR_LEFT : (dev < -Tolint ? IOR_RIGHT : IOR_UNDEF));
    }

    // Define position of point relative to line
    private static int PointLinePosition(in Point2D p, in Line2D line)
    {
        return PointLinePosition(p, line, out _);
    }

    // Compute parameter of point on line
    private static double ParamPointOnLine(in Point2D p, in Line2D line)
    {
        return line.Direction.XY() * (p.XY() - line.Location.XY());
    }

    // Compute parameter of two points on line (as intersection of segment)
    private static double ParamPointsOnLine(in Point2D p1, in Point2D p2, in Line2D line)
    {
        double dist1 = PointLineDeviation(p1, line);
        double dist2 = PointLineDeviation(p2, line);
        // in most cases, one of points is on line
        if (Math.Abs(dist1) < Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (Math.Abs(dist2) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return 0.5 * (ParamPointOnLine(p1, line) + ParamPointOnLine(p2, line));
            }
            return ParamPointOnLine(p1, line);
        }
        if (Math.Abs(dist2) < Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return ParamPointOnLine(p2, line);
        }
        // just protection
        if (dist2 * dist1 > 0)
        {
            return 0.5 * (ParamPointOnLine(p1, line) + ParamPointOnLine(p2, line));
        }
        // else compute intersection
        return (ParamPointOnLine(p1, line) * dist2 - ParamPointOnLine(p2, line) * dist1) / (dist2 - dist1);
    }

    // Compute projection of point on line
    private static Point2D ProjectPointOnLine(in Point2D p, in Line2D line)
    {
        return new Point2D(line.Location.XY() + line.Direction.XY() * ParamPointOnLine(p, line));
    }

    // Apply context to one edge in the wire and put result into this wire
    private static int ApplyContext(in WireSegmentFixer wire, int iedge, ReShapeBuilder context)
    {
        Edge edge = wire.Edge(iedge);
        Shape res = context.InternalApply(edge);

        if (res.IsSame(edge))
        {
            return 1;
        }

        if (res.Type == ShapeType.Edge)
        {
            wire.SetEdge(iedge, ShapeCasts.Edge(res));
            return 1;
        }

        int index = iedge;

        var segw = new ExtendWireData();
        segw.ManifoldMode() = false;
        for (var it = new Iterator(res); it.More(); it.Next())
        {
            Edge E = ShapeCasts.Edge(it.Value());
            if (!E.IsNull)
            {
                segw.Add(E);
            }
        }

        // add edges into the wire in correct order
        if (segw.EdgeCount > 0)
        {
            int ind;
            wire.GetPatchIndex(iedge, out int iumin, out int iumax, out int ivmin, out int ivmax);
            int nbEdges = segw.EdgeCount;
            for (int i = 1; i <= nbEdges; i++, index++)
            {
                ind = (edge.Orientation == Forward || edge.Orientation == Internal
                         ? i
                         : segw.EdgeCount - i + 1);
                Edge e = segw.Edge(ind);
                if (i == 1)
                {
                    wire.SetEdge(index, e);
                }
                else
                {
                    wire.AddEdge(index, e, iumin, iumax, ivmin, ivmax);
                }
            }
        }

        return index - iedge;
    }

    // check points coincidence
    private static bool IsCoincided(in Point2D p1, in Point2D p2, double uResolution, double vResolution, double tol)
    {
        // pdn Maximal accuracy is working precision of intersector.
        double uTolerance = uResolution * tol;
        double vTolerance = vResolution * tol;
        return Math.Abs(p1.X - p2.X) <= Max(Tolint, uTolerance) && Math.Abs(p1.Y - p2.Y) <= Max(Tolint, vTolerance);
    }

    // computes index for the patch by given parameter Param
    private static int GetPatchIndex(double param, NCollection_HArray1<double> Params, bool isClosed)
    {
        int np = Params.Upper();
        double period = Params.Value(np) - Params.Value(1);
        double shift = 0;
        if (isClosed)
        {
            shift = ShapeAnalysisAnalysis.AdjustToPeriod(param, Params.Value(1), Params.Value(np));
        }
        double p = param + shift;

        // locate patch: the same algo as in SE_CS::LocateParameter()
        int i; // svv #1
        for (i = 2; i < np; i++)
        {
            //    double par = Params->Value(i);
            if (p < Params.Value(i))
            {
                break;
            }
        }
        i--;

        double ish = shift / period;
        int ishift = (int)(ish < 0 ? ish - 0.5 : ish + 0.5);
        return i - ishift * (np - 1);
    }

    /// <summary>
    /// Fills the sequence of wire segments with the wires of the face (each reordered), applying the context first.
    /// </summary>
    protected void LoadWires(NCollection_Sequence<WireSegmentFixer> seqw)
    {
        seqw.Clear();

        // Init seqw by initial set of wires (with corresponding orientation)
        for (var iw = new Iterator(_face, false); iw.More(); iw.Next())
        {
            Shape tmpW = Context()!.InternalApply(iw.Value());
            if (tmpW.Type != ShapeType.Wire)
            {
                if (tmpW.Type == ShapeType.Vertex)
                {
                    var seg = new WireSegmentFixer(); //(( isOuter ? TopAbs_REVERSED : TopAbs_FORWARD ) );
                    seg.SetVertex(ShapeCasts.Vertex(tmpW));
                    seg.Orientation(tmpW.Orientation);
                    seqw.Append(seg);
                }

                continue;
            }

            Wire wire = ShapeCasts.Wire(tmpW);

            bool isNonManifold = (wire.Orientation != Reversed && wire.Orientation != Forward);

            // protect against INTERNAL/EXTERNAL wires
            //    if ( wire.Orientation() != TopAbs_REVERSED &&
            //	 wire.Orientation() != TopAbs_FORWARD ) continue;

            // determine orientation of the wire
            //    TopoDS_Face face = TopoDS::Face ( myFace.EmptyCopied() );
            //    B.Add ( face, wire );
            //    bool isOuter = ShapeAnalysis::IsOuterBound ( face );

            if (isNonManifold)
            {
                var sbwd = new ExtendWireData(wire, true, false);
                // pdn protection against wires w/o edges
                int nbEdges = sbwd.EdgeCount;
                if (nbEdges != 0)
                {
                    // wire segments for non-manifold topology should have INTERNAL orientation
                    var seg = new WireSegmentFixer(sbwd, Internal);
                    seqw.Append(seg);
                }
            }
            else
            {
                // splitting wires containing manifold and non-manifold parts on a separate
                // wire segment
                var sbwdM = new ExtendWireData();
                var sbwdNM = new ExtendWireData();
                sbwdNM.ManifoldMode() = false;
                var it = new Iterator(wire);
                for (; it.More(); it.Next())
                {
                    Edge e = ShapeCasts.Edge(it.Value());
                    if (e.Orientation == Forward || e.Orientation == Reversed)
                    {
                        sbwdM.Add(e);
                    }
                    else
                    {
                        sbwdNM.Add(e);
                    }
                }

                int nbMEdges  = sbwdM.EdgeCount;
                int nbNMEdges = sbwdNM.EdgeCount;

                if (nbNMEdges != 0)
                {
                    var seg = new WireSegmentFixer(sbwdNM, Internal); //(( isOuter ? TopAbs_REVERSED : TopAbs_FORWARD ) );
                    seqw.Append(seg);
                }

                if (nbMEdges != 0)
                {
                    // Orientation is set so as to allow the segment to be traversed in only one direction
                    // skl 01.04.2002
                    var sfw = new WireFixer();
                    sfw.Load(sbwdM);
                    int stat = 0;
                    Surface gs = Tool.Surface(_face)!;
                    if (gs.IsUPeriodic && gs.IsVPeriodic)
                    {
                        // For torus-like shapes, first reorder in 2d since reorder is indifferent in 3d
                        var sawo = new WireOrderAnalysis(false, 0);
                        var sae = new EdgeAnalysis();
                        for (int i = 1; i <= nbMEdges; i++)
                        {
                            // smh#8
                            Shape tmpF = _face.Oriented(Forward);
                            if (!sae.PCurve(sbwdM.Edge(i), ShapeCasts.Face(tmpF), out Curve2D? c2d, out double f, out double l))
                            {
                                continue;
                            }
                            sawo.Add(c2d!.Value(f).XY(), c2d.Value(l).XY());
                        }

                        sawo.Perform();
                        stat = (sawo.Status() < 0 ? -1 : 1);
                        sfw.FixReorder(sawo);
                    }

                    sfw.FixReorder();
                    if (sfw.StatusReorder(ExtendStatus.DONE3))
                    {
                        stat = -1;
                    }

                    if (stat < 0)
                    {
                        var b = new BrepBuilder();
                        Shape dummy = _face.EmptyCopied();
                        Face face = ShapeCasts.Face(dummy);
                        b.Add(face, wire);
                        bool isOuter = ShapeAnalysisAnalysis.IsOuterBound(face);
                        Wire w = sbwdM.InternalWire();
                        dummy = _face.EmptyCopied();
                        face = ShapeCasts.Face(dummy);
                        b.Add(face, w);
                        bool isOuterAfter = ShapeAnalysisAnalysis.IsOuterBound(face);
                        if (isOuter != isOuterAfter)
                        {
                            sbwdM.Reverse(face);
                        }
                    }

                    var seg = new WireSegmentFixer(sbwdM, Reversed); //(( isOuter ? TopAbs_REVERSED : TopAbs_FORWARD ) );
                    seqw.Append(seg);
                }
            }
        }
    }

    /// <summary>
    /// Analyzes tangencies and computes the orientation code of the wire segment between two intersections: whether it
    /// is left or right of the cutting line, or tangent to it.
    /// </summary>
    protected int ComputeCode(ExtendWireData wire, in Line2D line, int begInd, int endInd, double begPar,
                              double endPar, bool isInternal = false)
    {
        int code = IOR_UNDEF;

        var sae = new EdgeAnalysis();
        const int npoints = 5; // number of points for measuring deviation

        // track special closed case: segment starts at end of edge and ends at its beginning
        int special = (begInd == endInd
                           && (wire.Edge(begInd).Orientation == Forward
                               || wire.Edge(begInd).Orientation == Internal)
                                == (begPar > endPar)
                         ? 1
                         : 0);
        if (special == 0 && begInd == endInd && begPar == endPar && (_closedMode || isInternal))
        {
            special = 1;
        }

        // for tracking cases in closed mode
        bool begin = true;
        double shift = 0;
        var p2d0 = new Point2D();

        // check if segment is tangency
        // Segment is considered as tangency if deviation of pcurve from line
        // (in 2d) measured by NPOINTS points is less than tolerance of edge
        // (recomputed to 2d using Resolution).

        int nb = wire.EdgeCount;

        int i; // svv #1
        for (i = begInd;; i++)
        {
            if (i > nb)
            {
                i = 1;
            }
            Edge edge = wire.Edge(i);

            if (!sae.PCurve(edge, _face, out Curve2D? c2d, out double f, out double l, false))
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
                continue;
            }
            double tol = LimitTolerance(Tool.Tolerance(edge));
            bool isreversed = (edge.Orientation == Reversed);

            double par1 = (i == begInd && special >= 0 ? begPar : (isreversed ? l : f));
            double par2 = (i == endInd && special <= 0 ? endPar : (isreversed ? f : l));
            double dpar = (par2 - par1) / (npoints - 1);
            int np = (Math.Abs(dpar) < Kerf.Engine.Foundation.Precision.PConfusion() ? 1 : npoints);
            int j; // svv #1
            for (j = 0; j < np; j++)
            {
                double par = par1 + dpar * j;
                Point2D p2d = c2d!.Value(par);
                if (_closedMode)
                {
                    if (_uClosed && Math.Abs(line.Direction.X) < Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        if (begin)
                        {
                            shift = ShapeAnalysisAnalysis.AdjustByPeriod(p2d.X, line.Location.X, _uPeriod);
                        }
                        else if (j == 0)
                        {
                            shift = ShapeAnalysisAnalysis.AdjustByPeriod(p2d.X - p2d0.X, 0.0, _uPeriod);
                        }
                        p2d = p2d.WithX(p2d.X + shift);
                    }
                    if (_vClosed && Math.Abs(line.Direction.Y) < Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        if (begin)
                        {
                            shift = ShapeAnalysisAnalysis.AdjustByPeriod(p2d.Y, line.Location.Y, _vPeriod);
                        }
                        else if (j == 0)
                        {
                            shift = ShapeAnalysisAnalysis.AdjustByPeriod(p2d.Y - p2d0.Y, 0.0, _vPeriod);
                        }
                        p2d = p2d.WithY(p2d.Y + shift);
                    }
                    begin = false;
                }
                p2d0 = p2d;
                int pos = PointLinePosition(p2d, line);
                if (pos == IOR_UNDEF)
                {
                    continue;
                }

                // analyse the deviation
                Point2D p2dl = ProjectPointOnLine(p2d, line);
                if (!IsCoincided(p2d, p2dl, _uResolution, _vResolution, tol))
                {
                    if (!_closedMode)
                    {
                        code = pos;
                        break;
                    }
                    else
                    {
                        code |= pos;
                    }
                }
            }
            if (j < np)
            {
                i = 0;
                break;
            } // not tangency
            if (i == endInd)
            {
                if (special <= 0)
                {
                    break;
                }
                else
                {
                    special = -1;
                }
            }
        }
        if (_closedMode)
        {
            if (code != IOR_UNDEF && !begin)
            {
                // in closed mode, if segment is of 2*pi length, it is BOTH
                double dev = PointLineDeviation(p2d0, line);
                if (_uClosed && Math.Abs(line.Direction.X) < Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    if (Math.Abs(Math.Abs(dev) - _uPeriod) < 0.1 * _uPeriod)
                    {
                        code = IOR_BOTH;
                        if (dev > 0)
                        {
                            code |= IOR_POS;
                        }
                    }
                    else if (code == IOR_BOTH)
                    {
                        code = IOR_UNDEF;
                    }
                }
                if (_vClosed && Math.Abs(line.Direction.Y) < Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    if (Math.Abs(Math.Abs(dev) - _vPeriod) < 0.1 * _vPeriod)
                    {
                        code = IOR_BOTH;
                        if (dev > 0)
                        {
                            code |= IOR_POS;
                        }
                    }
                    else if (code == IOR_BOTH)
                    {
                        code = IOR_UNDEF;
                    }
                }
            }
            return code;
        }
        if (i != 0)
        {
            code = IOR_UNDEF; // tangency
        }
        else if (code == IOR_BOTH)
        { // parity error in intersector
            code = IOR_LEFT;
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
        }
        return code;
    }

    // After applying context to (seam) edge, distribute its indices on new edges,
    // according to their parameters on that edge
    private static void DistributeSplitPoints(ExtendWireData sbwd, in Face myFace, int index, int nsplit,
                                              NCollection_Sequence<int> indexes, NCollection_Sequence<double> values)
    {
        bool isreversed = (nsplit > 0 && sbwd.Edge(index).Orientation == Reversed);

        var params_ = new NCollection_Array1<double>(0, nsplit);
        int i; // svv #1
        for (i = 0; i < nsplit; i++)
        {
            Tool.Range(sbwd.Edge(index + i), myFace, out double f, out double l);
            params_.SetValue(i, (isreversed ? l : f));
        }

        for (i = 1; i <= indexes.Length() && indexes[i] < index; i++)
        {
            ;
        }
        for (int shift = 1; i <= indexes.Length() && indexes[i] == index; i++)
        {
            while (shift < nsplit && isreversed != (values[i] > params_[shift]))
            {
                shift++;
            }
            indexes.SetValue(i, index + shift - 1);
        }
        for (; i <= indexes.Length(); i++)
        {
            indexes.SetValue(i, indexes[i] + nsplit - 1);
        }
    }

    // OCCT returns int (a bool converted): 1 when there is no 3d curve.
    private static int CheckByCurve3d(in Point3D pos, Curve3D? c3d, double param, in Transform3D t, double tol)
    {
        if (c3d is null)
        {
            return 1;
        }
        Point3D p = c3d.Value(param);
        if (t.Form != Identity)
        {
            p = p.Transformed(t);
        }
        return pos.SquareDistanceTo(p) <= tol * tol ? 1 : 0;
    }

    private static void DefinePatch(in WireSegmentFixer wire, int code, bool isCutByU, int cutIndex, int number = -1)
    {
        int nb = (number > 0 ? number : wire.EdgeCount);
        if (isCutByU)
        {
            if ((code & IOR_LEFT) == 0)
            {
                wire.DefineIUMin(nb, cutIndex);
            }
            if ((code & IOR_RIGHT) == 0)
            {
                wire.DefineIUMax(nb, cutIndex);
            }
        }
        else
        {
            if ((code & IOR_RIGHT) == 0)
            {
                wire.DefineIVMin(nb, cutIndex);
            }
            if ((code & IOR_LEFT) == 0)
            {
                wire.DefineIVMax(nb, cutIndex);
            }
        }
    }

    private static double GetGridResolution(NCollection_HArray1<double> splitValues, int cutIndex)
    {
        int nb = splitValues.Length();
        double leftLen = (cutIndex > 1 ? splitValues.Value(cutIndex) - splitValues.Value(cutIndex - 1)
                                       : splitValues.Value(nb) - splitValues.Value(nb - 1));
        double rigthLen = (cutIndex < nb ? splitValues.Value(cutIndex + 1) - splitValues.Value(cutIndex)
                                         : splitValues.Value(2) - splitValues.Value(1));
        return Min(leftLen, rigthLen) / 3.0;
    }

    /// <summary>
    /// Splits the edges of the wire at the given edge indices and parameters; returns the resulting wire and the
    /// vertices at the splitting points. Two splits too near one another (by the edge's tolerance) make one point; a
    /// split too near an end of the edge isn't made (the end vertex is returned instead).
    /// </summary>
    protected WireSegmentFixer SplitWire(ref WireSegmentFixer wire, NCollection_Sequence<int> indexes,
                                             NCollection_Sequence<double> values, NCollection_Sequence<Shape> vertices,
                                             NCollection_Sequence<int> segmentCodes, bool isCutByU, int cutIndex)
    {
        var b = new BrepBuilder();
        var result = new WireSegmentFixer();
        var surfTool = new SurfaceAnalysis(Tool.Surface(_face));
        int nbSplits = indexes.Length();
        var sae = new EdgeAnalysis();
        int start = 1;
        Orientation wireOrient = wire.Orientation();
        var t = new Transform3D();
        if (!_loc.IsIdentity)
        {
            t = _loc.Inverted().Transformation;
        }

        // Processing edge by edge (assuming that split points are sorted along the wire)
        for (int i = 1; i <= wire.EdgeCount; i++)
        {
            // for already split seam edge, redistribute its splitting points
            int nsplit = ApplyContext(wire, i, Context()!);
            if (nsplit != 1)
            {
                DistributeSplitPoints(wire.WireData(), _face, i, nsplit, indexes, values);
                if (nsplit <= 0)
                {
                    i--;
                    continue;
                }
            }
            Edge edge = wire.Edge(i);

            wire.GetPatchIndex(i, out int iumin, out int iumax, out int ivmin, out int ivmax);

            // Position code for first segment of edge
            int code = segmentCodes[start > 1 ? start - 1 : segmentCodes.Length()];

            // Defining split parameters on edge
            int stop = start;
            while (stop <= nbSplits && indexes[stop] == i)
            {
                stop++;
            }
            if (stop == start)
            {
                result.AddEdge(0, edge, iumin, iumax, ivmin, ivmax);
                if (code != 0 || wire.Orientation() != External)
                { // pdn 0 code handling for extertnal wires
                    DefinePatch(result, code, isCutByU, cutIndex);
                }
                continue;
            }
            // find non-manifold vertices on edge
            var nmVertices = new NCollection_Sequence<Shape>();
            var it = new Iterator(edge, false);
            for (; it.More(); it.Next())
            {
                if (it.Value().Orientation != Forward && it.Value().Orientation != Reversed)
                {
                    nmVertices.Append(it.Value());
                }
            }

            // Collect data on edge
            double tolEdge = Tool.Tolerance(edge);
            Vertex prevV = sae.FirstVertex(edge);
            Vertex lastV = sae.LastVertex(edge);
            double prevVTol = LimitTolerance(Tool.Tolerance(prevV));
            double lastVTol = LimitTolerance(Tool.Tolerance(lastV));
            Point3D prevVPnt = Tool.Pnt(prevV);
            Point3D lastVPnt = Tool.Pnt(lastV);
            if (t.Form != Identity)
            {
                prevVPnt = prevVPnt.Transformed(t);
                lastVPnt = lastVPnt.Transformed(t);
            }

            if (!sae.Curve3d(edge, out Curve3D? c3d, out double f3d, out double l3d))
            { // not a crime
                c3d = null;
                f3d = l3d = 0;
            }

            if (!sae.PCurve(edge, _face, out Curve2D? c2d, out double firstPar, out double lastPar))
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            }
            // finding sequence of non-manifold parameters
            int nbNMVert = nmVertices.Length();
            var nmVertParams = new NCollection_Sequence<double>();
            if (nbNMVert != 0)
            {
                var adc = new CurveAdaptor2D(c2d!);
                int n = 1;
                for (; n <= nbNMVert; n++)
                {
                    Point3D apV = Tool.Pnt(ShapeCasts.Vertex(nmVertices.Value(n)));
                    double apar = firstPar;
                    double adist2 = RealLast();
                    if (c3d is not null)
                    {
                        var asae = new CurveAnalysis();
                        adist2 = asae.Project(c3d, apV, Kerf.Engine.Foundation.Precision.Confusion(), out _, out apar);
                        adist2 *= adist2;
                    }
                    else
                    {
                        Point2D p2d = surfTool.ValueOfUV(apV, Kerf.Engine.Foundation.Precision.Confusion());
                        var extr = new ExtPC2d(p2d, adc);
                        if (extr.IsDone && extr.ExtCount != 0)
                        {
                            adist2 = extr.SquareDistance(1);
                            int index = 1;
                            int k = 2;
                            for (; k <= extr.ExtCount; k++)
                            {
                                double ad2 = extr.SquareDistance(k);
                                if (ad2 < adist2)
                                {
                                    adist2 = ad2;
                                    index = k;
                                }
                            }
                            apar = extr.Point(index).Parameter();
                        }
                    }
                    nmVertParams.Append(apar);
                }
            }

            // pdn Calculating parametric shift
            bool sp = (f3d == firstPar && l3d == lastPar);
            double span2d = lastPar - firstPar;
            //    double ln2d  = lastPar-prevPar;
            //    double ln3d  = l3d - f3d;
            //    double fact = ln2d/ln3d;
            //    double shift =  prevPar - f3d*fact;
            double prevPar = firstPar;
            Point2D prevPnt2d = c2d!.Value(prevPar);
            Point2D lastPnt2d = c2d.Value(lastPar);
            Point3D prevPnt = _grid!.Value(prevPnt2d);
            Point3D lastPnt = _grid.Value(lastPnt2d);
            bool isPeriodic = c2d.IsPeriodic;
            double period = (isPeriodic ? c2d.Period : 0.0);

            // Splitting edge
            int nbEdgesStart = result.EdgeCount;
            bool splitted = false;
            double currPar = lastPar; // SK
            for (int j = start; j <= stop; prevPar = currPar, j++)
            {
                if (!splitted && j >= stop)
                { // no splitting at all
                    // code = SegmentCodes ( j >1 ? j-1 : SegmentCodes.Length() ); // classification code
                    break;
                }
                currPar = (j < stop ? values.Value(j) : lastPar);
                // fix for case when pcurve is periodic and first parameter of edge is more than 2P
                // method ShapeBuild_Edge::CopyRanges shift pcurve to range 0-2P and parameters of cutting
                // should be shifted too. gka SAMTECH 28.07.06
                if (isPeriodic)
                {
                    if (currPar > (Max(lastPar, firstPar) + Kerf.Engine.Foundation.Precision.PConfusion())
                        || currPar < (Min(firstPar, lastPar) - Kerf.Engine.Foundation.Precision.PConfusion()))
                    {
                        double shift = ShapeAnalysisAnalysis.AdjustByPeriod(currPar, (firstPar + lastPar) * 0.5, period);
                        currPar += shift;
                    }
                }

                var currPnt2d = new Point2D();
                var currPnt = new Point3D();

                // Try to adjust current splitting point to previous or end of edge
                bool doCut = true;
                var v = new Vertex();
                if (Math.Abs(currPar - lastPar) < Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    v = lastV;
                    doCut = false;
                }
                else if (Math.Abs(currPar - prevPar) < Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    vertices.Append(prevV);
                    code = segmentCodes[j]; // classification code - update for next segment
                    continue;               // no splitting at this point, go to next one
                }
                else
                {
                    currPnt2d = c2d.Value(currPar);
                    currPnt = _grid.Value(currPnt2d);
                    if (currPnt.DistanceTo(lastVPnt) <= lastVTol &&
                        // Tolerance is increased to prevent degenerated cuts in cases where all vertex
                        // tolerance is covered by distance of the edge curve from vertex point.
                        // Doubled to prevent edge being fully covered by its vertices tolerance (invalid edge).
                        CheckByCurve3d(lastVPnt, c3d, f3d + (currPar - firstPar) * (l3d - f3d) / span2d, t,
                                       lastVTol + 2 * Kerf.Engine.Foundation.Precision.Confusion()) != 0
                        && lastPnt.DistanceTo(_grid.Value(c2d.Value(0.5 * (currPar + lastPar)))) <= lastVTol)
                    {
                        v = lastV;
                        double uRes = _uResolution;
                        double vRes = _vResolution;
                        if (isCutByU)
                        {
                            double gridRes = GetGridResolution(_grid.InternalUJointValues()!, cutIndex) / lastVTol;
                            uRes = Min(_uResolution, gridRes);
                        }
                        else
                        {
                            double gridRes = GetGridResolution(_grid.InternalVJointValues()!, cutIndex) / lastVTol;
                            vRes = Min(_vResolution, gridRes);
                        }
                        if (IsCoincided(lastPnt2d, currPnt2d, uRes, vRes, lastVTol)
                            && IsCoincided(lastPnt2d, c2d.Value(0.5 * (currPar + lastPar)), uRes, vRes, lastVTol))
                        {
                            doCut = false;
                        }
                    }
                    else if (currPnt.DistanceTo(prevVPnt) <= prevVTol &&
                             // Tolerance is increased to prevent degenerated cuts in cases where all vertex
                             // tolerance is covered by distance of the edge curve from vertex point.
                             // Doubled to prevent edge being fully covered by its vertices tolerance (invalid
                             // edge).
                             CheckByCurve3d(prevVPnt, c3d, f3d + (currPar - firstPar) * (l3d - f3d) / span2d, t,
                                            prevVTol + 2 * Kerf.Engine.Foundation.Precision.Confusion()) != 0
                             && prevPnt.DistanceTo(_grid.Value(c2d.Value(0.5 * (currPar + prevPar)))) <= prevVTol)
                    {
                        v = prevV;
                        double uRes = _uResolution;
                        double vRes = _vResolution;
                        if (isCutByU)
                        {
                            double gridRes = GetGridResolution(_grid.InternalUJointValues()!, cutIndex) / prevVTol;
                            uRes = Min(_uResolution, gridRes);
                        }
                        else
                        {
                            double gridRes = GetGridResolution(_grid.InternalVJointValues()!, cutIndex) / prevVTol;
                            vRes = Min(_vResolution, gridRes);
                        }
                        if (IsCoincided(prevPnt2d, currPnt2d, uRes, vRes, prevVTol)
                            && IsCoincided(prevPnt2d, c2d.Value(0.5 * (currPar + prevPar)), uRes, vRes, prevVTol))
                        {
                            vertices.Append(prevV);
                            code = segmentCodes[j]; // classification code - update for next segment
                            continue;               // no splitting at this point, go to next one
                        }
                    }
                    //: abv 28.05.02: OCC320 Sample_2: if maxtol = 1e-7, the vertex tolerance
                    // is actually ignored - protect against new vertex on degenerated edge
                    else if (Tool.Degenerated(edge) && prevV.IsSame(lastV))
                    {
                        v = prevV;
                    }
                }
                // classification code for current segment
                if (j > start)
                {
                    code = segmentCodes[j > 1 ? j - 1 : segmentCodes.Length()];
                }

                // if not adjusted, make new vertex
                if (v.IsNull)
                {
                    b.MakeVertex(ref v, currPnt.Transformed(_loc.Transformation), tolEdge);
                    vertices.Append(v);
                }
                // else adjusted to end, fill all resting vertices
                else if (!doCut)
                {
                    for (; j < stop; j++)
                    {
                        vertices.Append(lastV);
                    }
                    if (!splitted)
                    {
                        break; // no splitting at all
                    }
                    currPar = lastPar;
                }
                else
                {
                    vertices.Append(v);
                }

                // When edge is about to be split, copy end vertices to protect
                // original shape from increasing tolerance after fixing SameParameter
                if (!splitted)
                {
                    // smh#8
                    Shape emptyCopiedfV = prevV.EmptyCopied();
                    Vertex fV = ShapeCasts.Vertex(emptyCopiedfV);
                    Context()!.Replace(prevV, fV);
                    Vertex lV;
                    if (prevV.IsSame(lastV))
                    {
                        // smh#8
                        Shape tmpV = fV.Oriented(lastV.Orientation);
                        lV = ShapeCasts.Vertex(tmpV);
                    }
                    else
                    {
                        // smh#8
                        Shape emptyCopied = lastV.EmptyCopied();
                        lV = ShapeCasts.Vertex(emptyCopied);
                        Context()!.Replace(lastV, lV);
                    }
                    if (v.IsSame(lastV))
                    {
                        v = lV;
                    }
                    else if (v.IsSame(prevV))
                    {
                        v = fV;
                    }
                    lastV = lV;
                    prevV = fV;
                }

                // Splitting of the edge
                splitted = true;
                prevV.Orient(Forward);
                v.Orient(Reversed);
                var sbe = new BuildEdgeBuilder();
                Edge initEdge = edge;
                bool ismanifold = (edge.Orientation == Forward || edge.Orientation == Reversed);
                if (!ismanifold)
                {
                    initEdge.Orient(Forward);
                }
                Edge newEdge = sbe.InternalCopyReplaceVertices(initEdge, prevV, v);

                // addition internal vertices if they exists on edge
                int n = 1;
                for (; n <= nmVertParams.Length(); n++)
                {
                    double apar = nmVertParams.Value(n);
                    Vertex nmVert = ShapeCasts.Vertex(nmVertices.Value(n));
                    Vertex atmpV = ShapeCasts.Vertex(Context()!.InternalApply(nmVert));
                    if (Math.Abs(apar - prevPar) <= Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        Context()!.Replace(atmpV, prevV);
                        nmVertParams.Remove(n);
                        nmVertices.Remove(n);
                        n--;
                    }
                    else if (Math.Abs(apar - currPar) <= Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        Context()!.Replace(atmpV, v);
                        nmVertParams.Remove(n);
                        nmVertices.Remove(n);
                        n--;
                    }
                    if (apar > prevPar && apar < currPar)
                    {
                        b.Add(newEdge, atmpV);
                        nmVertParams.Remove(n);
                        nmVertices.Remove(n);
                        n--;
                    }
                }

                sbe.CopyPCurves(newEdge, initEdge);

                TransferParametersAnalysis theTransferParamtool = GetTransferParamTool();
                theTransferParamtool.SetMaxTolerance(MaxTolerance());
                theTransferParamtool.Init(initEdge, _face);
                theTransferParamtool.TransferRange(newEdge, prevPar, currPar, true);

                if (!ismanifold)
                {
                    if (code == IOR_UNDEF)
                    { // tangential segment
                        newEdge.Orient(External);
                    }
                    else
                    {
                        newEdge.Orient(edge.Orientation);
                    }
                }

                if (!sp && !Tool.Degenerated(newEdge))
                {
                    b.SameRange(newEdge, false);
                }
                // pdn take into account 0 codes (if ext)
                if (code == 0 && wire.Orientation() == External)
                {
                    code = ((isCutByU == (j == 1)) ? 1 : 2);
                }

                result.AddEdge(0, newEdge, iumin, iumax, ivmin, ivmax);
                DefinePatch(result, code, isCutByU, cutIndex);

                // Changing prev parameters
                prevV = v;
                prevVTol = LimitTolerance(Tool.Tolerance(v));
                prevVPnt = Tool.Pnt(v);
                prevPnt = currPnt;
                prevPnt2d = currPnt2d;
            }
            start = stop;

            if (splitted)
            {
                // record replacement in context
                // NOTE: order of edges in the replacing wire corresponds to FORWARD orientation of the edge
                var resWire = new Wire();
                b.MakeWire(ref resWire);
                for (int k = nbEdgesStart; k < result.EdgeCount; k++)
                {
                    if (edge.Orientation == Forward || edge.Orientation == Internal)
                    {
                        b.Add(resWire, result.Edge(k + 1));
                    }
                    else
                    {
                        b.Add(resWire, result.Edge(result.EdgeCount - k + nbEdgesStart));
                    }
                }
                Context()!.Replace(edge, resWire);
            }
            else
            {
                if (wireOrient == Internal && code == 0)
                {
                    var sbe = new BuildEdgeBuilder();
                    if (edge.Orientation == Internal)
                    {
                        edge.Orient(Forward);
                    }
                    Edge e1 = sbe.InternalCopy(edge, false);
                    var c2d2 = (Curve2D)c2d.Copy();
                    b.UpdateEdge(e1, c2d, c2d2, _face, 0.0);
                    e1.Orient(External);
                    Context()!.Replace(edge, e1);
                    result.AddEdge(0, e1, iumin, iumax, ivmin, ivmax);
                }
                else
                {
                    result.AddEdge(0, edge, iumin, iumax, ivmin, ivmax);
                }
                if (code == 0 && wire.Orientation() == External)
                {
                    // pdn defining code for intersection of two isos
                    code = ((isCutByU == (Math.Abs(firstPar - currPar) < Math.Abs(lastPar - currPar))) ? 2 : 1);
                }
                DefinePatch(result, code, isCutByU, cutIndex);
            }
        }
        result.Orientation(wireOrient);
        return result;
    }

    /// <summary>
    /// Splits the edges of the wire by the cutting line (wires FORWARD or REVERSED are taken as closed), recording the
    /// splits in the context (but for EXTERNAL wires, former cutting lines), and fills the parameters, types and
    /// vertices of the intersection points of the line with the edges.
    /// </summary>
    protected bool SplitByLine(ref WireSegmentFixer wire, in Line2D line, bool isCutByU, int cutIndex,
                               NCollection_Sequence<double> splitLinePar, NCollection_Sequence<int> splitLineCode,
                               NCollection_Sequence<Shape> splitLineVertex)
    {
        var sae = new EdgeAnalysis();
        // prepare data on cutting line
        var jC2d = new LineCurve2D(line);
        var jGAC = new CurveAdaptor2D(jC2d);

        var intEdgeInd = new NCollection_Sequence<int>();    // index of intersecting edge
        var intEdgePar = new NCollection_Sequence<double>(); // parameter of intersection point on edge
        var intLinePar = new NCollection_Sequence<double>(); // parameter of intersection point on line

        bool isnonmanifold = (wire.Orientation() == Internal);
        // gka correction for non-manifold vertices SAMTECH
        if (wire.IsVertex)
        {
            var surfTool = new SurfaceAnalysis(Tool.Surface(_face));
            Vertex vert = wire.GetVertex();
            Point3D p3d = Tool.Pnt(vert);
            Point2D p2d = surfTool.ValueOfUV(p3d, Kerf.Engine.Foundation.Precision.Confusion());
            int code0 = PointLinePosition(p2d, line, out _);
            if (code0 != IOR_UNDEF)
            {
                return false;
            }
            double par = ParamPointOnLine(p2d, line);
            splitLinePar.Append(par);
            // splitting codes for non-manifold topology should be tangential
            splitLineCode.Append(ITP_TANG); // ITP_INTER);
            var vertNew = new Vertex();
            var b = new BrepBuilder();
            b.MakeVertex(ref vertNew, p3d, Tool.Tolerance(vert));
            vertNew.Orient(Forward);
            Context()!.Replace(vert, vertNew);
            splitLineVertex.Append(vertNew);
            wire.SetVertex(vertNew);
            return true;
        }
        ExtendWireData sewd = wire.WireData();

        int nbe = sewd.EdgeCount;

        //: abv 31.10.01: for closed mode
        int closedDir = 0;
        if (_closedMode)
        {
            if (_uClosed && Math.Abs(line.Direction.X) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                closedDir = -1;
            }
            else if (_vClosed && Math.Abs(line.Direction.Y) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                closedDir = 1;
            }
        }
        double halfPeriod = 0.5 * (closedDir != 0 ? closedDir < 0 ? _uPeriod : _vPeriod : 0.0);

        //============================================
        // make intersections and collect all data on intersection points
        int firstCode = 0, prevCode = 0;
        Point2D firstPos = new Point2D(), prevPos = new Point2D();
        double firstDev = 0.0, prevDev = 0.0;
        for (int iedge = 1; iedge <= nbe; iedge++)
        {
            Edge e = sewd.Edge(iedge);
            bool isreversed = (e.Orientation == Reversed);

            if (!sae.PCurve(e, _face, out Curve2D? c2d, out double f, out double l, false))
            {
                continue;
            }

            // get end points
            Point2D posf = c2d!.Value(f), posl = c2d.Value(l);
            Xy pppf = posf.XY(), pppl = posl.XY();

            // In case of ClosedMode, adjust curve and end points to period on closed surface
            //: abv 16.10.01: Ziegler_CADDY01.sat -18: if pcurve is longer than period,
            // ensure processing of all intersections
            int nbIter = 1;
            var shiftNext = new Vector2D(0.0, 0.0);
            if (_closedMode)
            {
                // get bounding box of pcurve
                var sac = new CurveAnalysis();
                var box = new Box2d();
                const int nbPoints = 41;
                sac.FillBndBox(c2d, f, l, nbPoints, true, ref box);
                box.Get(out double umin, out double vmin, out double umax, out double vmax);

                // compute shifts and adjust points adjust
                if (closedDir < 0)
                {
                    double x = line.Location.X;
                    double shift = ShapeAnalysisAnalysis.AdjustToPeriod(umin, x - _uPeriod, x);
                    if (shift != 0.0)
                    {
                        c2d = (Curve2D)c2d.Copy();
                        var v = new Vector2D(shift, 0.0);
                        c2d.Translate(v);
                        pppf.SetX(pppf.X() + shift);
                        pppl.SetX(pppl.X() + shift);
                    }
                    double dUmax = umax + shift - x;
                    shiftNext = shiftNext.WithX(dUmax > 0 ? -_uPeriod : _uPeriod);
                    nbIter = (int)(1 + Math.Abs(dUmax) / _uPeriod);
                    shift = ShapeAnalysisAnalysis.AdjustByPeriod(posf.X, x, _uPeriod);
                    posf = posf.WithX(posf.X + shift);
                    shift = ShapeAnalysisAnalysis.AdjustByPeriod(posl.X, x, _uPeriod);
                    posl = posl.WithX(posl.X + shift);
                }
                else if (closedDir > 0)
                {
                    double y = line.Location.Y;
                    double shift = ShapeAnalysisAnalysis.AdjustToPeriod(vmin, y - _vPeriod, y);
                    if (shift != 0.0)
                    {
                        c2d = (Curve2D)c2d.Copy();
                        var v = new Vector2D(0.0, shift);
                        c2d.Translate(v);
                        pppf.SetY(pppf.Y() + shift);
                        pppl.SetY(pppl.Y() + shift);
                    }
                    double dVmax = vmax + shift - y;
                    shiftNext = shiftNext.WithY(dVmax > 0 ? -_vPeriod : _vPeriod);
                    nbIter = (int)(1 + Math.Abs(dVmax) / _vPeriod);
                    shift = ShapeAnalysisAnalysis.AdjustByPeriod(posf.Y, y, _vPeriod);
                    posf = posf.WithY(posf.Y + shift);
                    shift = ShapeAnalysisAnalysis.AdjustByPeriod(posl.Y, y, _vPeriod);
                    posl = posl.WithY(posl.Y + shift);
                }
            }

            // detect intersections at junction of two edges
            Point2D pos = (isreversed ? posl : posf);
            int code = PointLinePosition(pos, line, out double dev);
            if (iedge == 1)
            {
                firstCode = code;
                firstPos = pos;
                firstDev = dev;
            }
            else if (code == IOR_UNDEF || code != prevCode)
            {
                if (closedDir == 0 || Math.Abs(dev - prevDev) < halfPeriod)
                {
                    intLinePar.Append(ParamPointsOnLine(pos, prevPos, line)); // !! - maybe compute exactly ?
                    intEdgePar.Append(isreversed ? l : f);
                    intEdgeInd.Append(iedge);
                }
            }

            // fill data on end point (for next edge)
            pos = (isreversed ? posf : posl);
            prevCode = PointLinePosition(pos, line, out prevDev);
            prevPos = pos;

            // cycle with shift in order to track all possible intersections
            for (int iter = 1; iter <= nbIter; iter++)
            {
                // data for intersection
                var iDom = new TwoDDomain(new Point2D(pppf), f, Tolint, new Point2D(pppl), l, Tolint);
                var iGAC = new CurveAdaptor2D(c2d);

                // intersection
                var inter = new GInter();
                inter.Perform(jGAC, /*jDom,*/ iGAC, iDom, Tolint, Tolint);

                // Fill arrays with new intersection points
                if (inter.IsDone)
                {
                    int i1;
                    for (i1 = 1; i1 <= inter.PointCount; i1++)
                    {
                        IntersectionPoint2D ip = inter.Point(i1);
                        if (ip.TransitionOfSecond().PositionOnCurve() == Position2D.Middle
                            || (code != IOR_UNDEF && prevCode != IOR_UNDEF))
                        {
                            intLinePar.Append(ip.ParamOnFirst());
                            intEdgePar.Append(ip.ParamOnSecond());
                        }
                    }
                    for (i1 = 1; i1 <= inter.SegmentCount; i1++)
                    {
                        IntersectionSegment2D IS = inter.Segment(i1);
                        if (IS.HasFirstPoint)
                        {
                            IntersectionPoint2D ip = IS.FirstPoint();
                            intLinePar.Append(ip.ParamOnFirst());
                            intEdgePar.Append(ip.ParamOnSecond());
                        }
                        if (IS.HasLastPoint)
                        {
                            IntersectionPoint2D ip = IS.LastPoint();
                            intLinePar.Append(ip.ParamOnFirst());
                            intEdgePar.Append(ip.ParamOnSecond());
                        }
                    }
                }
                if (iter < nbIter)
                {
                    if (iter == 1)
                    {
                        c2d = (Curve2D)c2d.Copy();
                    }
                    pppf += shiftNext.XY();
                    pppl += shiftNext.XY();
                    c2d.Translate(shiftNext);
                }
            }

            int start = intEdgeInd.Length() + 1; // first of the new points

            // Move all points into range [f,l] (intersector sometimes gives params out of range)
            int i;
            for (i = start; i <= intEdgePar.Length(); i++)
            {
                if (intEdgePar[i] < f)
                {
                    intEdgePar.SetValue(i, f);
                }
                else if (intEdgePar[i] > l)
                {
                    intEdgePar.SetValue(i, l);
                }
            }

            // Sort by parameter on edge
            for (i = intEdgePar.Length(); i > start; i--)
            {
                for (int j = start; j < i; j++)
                {
                    if (isreversed == (intEdgePar[j + 1] < intEdgePar[j]))
                    {
                        continue;
                    }
                    intLinePar.Exchange(j, j + 1);
                    intEdgePar.Exchange(j, j + 1);
                }
            }

            // and fill indices
            for (i = start; i <= intEdgePar.Length(); i++)
            {
                intEdgeInd.Append(iedge);
            }

            // Detect intersection at closing point
            // Only wires which are not EXTERNAL are considered (as closed)
            if (iedge == nbe && wire.Orientation() != External && wire.Orientation() != Internal
                && (prevCode == IOR_UNDEF || prevCode != firstCode))
            {
                if (closedDir == 0 || Math.Abs(firstDev - prevDev) < halfPeriod)
                {
                    intLinePar.Append(ParamPointsOnLine(pos, firstPos, line));
                    intEdgePar.Append(isreversed ? f : l);
                    intEdgeInd.Append(iedge);
                }
            }
        }

        if (intEdgePar.Length() < 1)
        {
            // pdn Defining position of wire. There is no intersection, so by any point.
            // DefinePatchForWire ( wire, firstCode, isCutByU, cutIndex );
            return false; // pdn ??
        }

        //======================================
        // Fill sequence of transition codes for intersection points
        var intCode = new NCollection_Sequence<int>();      // parameter of intersection point on line
        var segmentCodes = new NCollection_Sequence<int>(); // classification codes for segments of wire

        // remove duplicated points to ensure correct results of ComputeCode
        int ii, jj = intEdgePar.Length();
        if (_closedMode && jj > 1)
        {
            for (ii = 1; ii <= intEdgePar.Length();)
            {
                if (ii == jj)
                {
                    break;
                }
                if (intEdgeInd[ii] == intEdgeInd[jj] && Math.Abs(intEdgePar[ii] - intEdgePar[jj]) < Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    intLinePar.Remove(ii);
                    intEdgePar.Remove(ii);
                    intEdgeInd.Remove(ii);
                    if (jj > ii)
                    {
                        jj--;
                    }
                    continue;
                }
                else if (nbe == 1 || intEdgeInd[ii] == (intEdgeInd[jj] % nbe) + 1)
                {
                    Edge e1 = sewd.Edge(intEdgeInd[jj]);
                    Edge e2 = sewd.Edge(intEdgeInd[ii]);
                    Tool.Range(e1, _face, out double a1, out double b1);
                    Tool.Range(e2, _face, out double a2, out double b2);
                    if (Math.Abs(intEdgePar[jj] - (e1.Orientation == Forward ? b1 : a1)) < Kerf.Engine.Foundation.Precision.PConfusion()
                        && Math.Abs(intEdgePar[ii] - (e2.Orientation == Forward ? a2 : b2)) < Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        intLinePar.Remove(ii);
                        intEdgePar.Remove(ii);
                        intEdgeInd.Remove(ii);
                        if (jj > ii)
                        {
                            jj--;
                        }
                        continue;
                    }
                }
                jj = ii++;
            }
        }
        // sequence of real codes for each segment
        var newSegCodes = new NCollection_Sequence<int>();
        // Compute segment codes (left side of line, right or tangential)
        for (ii = 1; ii <= intEdgePar.Length(); ii++)
        {
            jj = (ii < intEdgePar.Length() ? ii + 1 : 1);
            int code = ComputeCode(sewd, line, intEdgeInd[ii], intEdgeInd[jj], intEdgePar[ii], intEdgePar[jj], isnonmanifold);
            segmentCodes.Append(code);
        }

        // for EXTERNAL wire, i.e. another joint line, every point is double intersection
        if (wire.Orientation() == External)
        {
            for (ii = 1; ii <= intEdgePar.Length(); ii++)
            {
                intCode.Append(ITP_TANG | IOR_BOTH);
                newSegCodes.Append(segmentCodes[ii]);
            }
        }
        // For real (closed) wire, analyze tangencies
        else
        {
            if (wire.Orientation() != Internal)
            {
                // Two consecutive tangential segments are considered as one, merge them.
                for (ii = 1; ii <= intEdgePar.Length(); ii++)
                {
                    jj = (ii > 1 ? ii - 1 : intEdgePar.Length());

                    int k = (ii < intEdgePar.Length() ? ii + 1 : 1); // [ACIS22539]

                    if (segmentCodes[jj] == IOR_UNDEF && segmentCodes[ii] == IOR_UNDEF)
                    {
                        // Very specific case when the constructed seam edge
                        // overlaps with spur edge [ACIS22539]
                        if (_closedMode && (intLinePar[ii] - intLinePar[jj]) * (intLinePar[k] - intLinePar[ii]) <= 0.0)
                        {
                            continue;
                        }

                        intEdgeInd.Remove(ii);
                        intEdgePar.Remove(ii);
                        intLinePar.Remove(ii);
                        segmentCodes.Remove(ii);
                        ii--;
                    }
                }
            }
            // pdn exit if all split points removed
            if (intEdgePar.Length() < 1)
            {
                return false; // pdn ??
            }

            // Analyze type of intersection point and encode it
            // Three kinds of points (ITP): clear intersection, tangency in-point,
            // beginning and end of tangential segment.
            // Orientation (IOR) tells on which side of line edge crosses it
            jj = intEdgePar.Length();

            for (ii = 1; ii <= intEdgePar.Length(); jj = ii++)
            {
                int codej = segmentCodes[jj];
                int codei = segmentCodes[ii];
                if (_closedMode)
                {
                    if ((codej & IOR_BOTH) == IOR_BOTH)
                    { // IOR_LEFT : IOR_RIGHT
                        codej = ((codej & IOR_POS) != 0 ? IOR_RIGHT : IOR_LEFT);
                    }
                    if ((codei & IOR_BOTH) == IOR_BOTH)
                    { // IOR_RIGHT : IOR_LEFT
                        codei = ((codei & IOR_POS) != 0 ? IOR_LEFT : IOR_RIGHT);
                    }
                    newSegCodes.Append(codei);
                    if (intEdgeInd[ii] == intEdgeInd[jj])
                    {
                        newSegCodes.Append(codej);
                    }
                }
                else
                {
                    newSegCodes.Append(codei);
                }
                int ipcode = (codej | codei);
                if (codej == IOR_UNDEF)
                { // previous segment was tangency
                    if (intLinePar[ii] > intLinePar[jj])
                    {
                        ipcode |= ITP_ENDSEG; // end of segment
                    }
                    else
                    {
                        ipcode |= ITP_BEGSEG; // beginning of segment
                    }
                }
                else if (codei == IOR_UNDEF)
                { // current segment is tangency
                    if (intLinePar[ii < intLinePar.Length() ? ii + 1 : 1] > intLinePar[ii])
                    {
                        ipcode |= ITP_BEGSEG; // beginning of segment
                    }
                    else
                    {
                        ipcode |= ITP_ENDSEG; // end of segment
                    }
                }
                // internal wire can be only tangent
                else if (ii == jj)
                {
                    ipcode |= ((ipcode & IOR_BOTH) == IOR_BOTH && !isnonmanifold ? ITP_INTER : ITP_TANG);
                }
                else if (codei == codej || isnonmanifold)
                {
                    ipcode |= ITP_TANG; // tangency in-point
                }
                else
                {
                    ipcode |= ITP_INTER; // standard crossing
                }
                intCode.Append(ipcode);
            }
        }

        //=======================================
        // Split edges in the wire by intersection points and fill vertices array
        var intVertices = new NCollection_Sequence<Shape>();
        wire = SplitWire(ref wire, intEdgeInd, intEdgePar, intVertices, newSegCodes, isCutByU, cutIndex);

        // add all data to input arrays
        for (ii = 1; ii <= intLinePar.Length(); ii++)
        {
            splitLinePar.Append(intLinePar[ii]);
            splitLineCode.Append(intCode[ii]);
            splitLineVertex.Append(intVertices[ii]);
        }

        return true;
    }

    /// <summary>
    /// Splits the edges of the sequence of wires by the cutting line; the parts of the line inside the face (by parity
    /// of the intersections with all wires) are added to the sequence as EXTERNAL one-edge segments without 3d curve,
    /// sharing vertices with the wires they cross.
    /// </summary>
    protected void SplitByLine(NCollection_Sequence<WireSegmentFixer> wires, in Line2D line, bool isCutByU,
                               int cutIndex)
    {
        var splitLinePar = new NCollection_Sequence<double>();
        var splitLineCode = new NCollection_Sequence<int>();
        var splitLineVertex = new NCollection_Sequence<Shape>();

        // split wires one by one, collecting data on intersection points
        int i; // svv #1
        for (i = 1; i <= wires.Length(); i++)
        {
            SplitByLine(ref wires.ChangeValue(i), line, isCutByU, cutIndex, splitLinePar, splitLineCode, splitLineVertex);
        }

        // sort intersection points along parameter on cutting line
        for (i = splitLinePar.Length(); i > 1; i--)
        {
            for (int j = 1; j < i; j++)
            {
                if (splitLinePar[j] > splitLinePar[j + 1])
                {
                    splitLinePar.Exchange(j, j + 1);
                    splitLineCode.Exchange(j, j + 1);
                    splitLineVertex.Exchange(j, j + 1);
                }
            }
        }

        // merge null-length tangential segments into one-point tangencies or intersections
        for (i = 1; i < splitLinePar.Length(); i++)
        {
            if (Math.Abs(splitLinePar[i + 1] - splitLinePar[i]) > Kerf.Engine.Foundation.Precision.PConfusion()
                && !splitLineVertex[i].IsSame(splitLineVertex[i + 1]))
            {
                continue;
            }
            if (((splitLineCode[i] & ITP_ENDSEG) != 0 && (splitLineCode[i + 1] & ITP_BEGSEG) != 0)
                || ((splitLineCode[i] & ITP_BEGSEG) != 0 && (splitLineCode[i + 1] & ITP_ENDSEG) != 0))
            {
                int code = (splitLineCode[i] | splitLineCode[i + 1]) & IOR_BOTH;
                splitLineCode.SetValue(i, code | (code == IOR_BOTH ? ITP_INTER : ITP_TANG));
                splitLinePar.Remove(i + 1);
                splitLineCode.Remove(i + 1);
                splitLineVertex.Remove(i + 1);
            }
        }

        // go along line, split it by intersection points and create edges
        // (only for internal parts, in particular not for tangential segments)
        var b = new BrepBuilder();
        int parity = 0;     // 0 - out, 1 - in
        int halfparity = 0; // left/right for tangential segments
        int tanglevel = 0;  // tangency nesting level
        for (i = 1; i <= splitLinePar.Length(); i++)
        {
            int code = splitLineCode[i];
            bool interior = (tanglevel == 0 && parity % 2 != 0); // create an edge
            if ((code & ITP_INTER) != 0)
            { // crossing
                parity++;
            }
            else if ((code & ITP_BEGSEG) != 0)
            { // beginning of tangential segment
                tanglevel++;
                if (halfparity == 0)
                {
                    halfparity = (code & IOR_BOTH);
                }
                else if (halfparity != (code & IOR_BOTH))
                {
                    parity++;
                }
            }
            else if ((code & ITP_ENDSEG) != 0)
            { // end of tangential segment
                tanglevel--;
                if (halfparity == 0)
                {
                    halfparity = (code & IOR_BOTH);
                }
                else if (halfparity != (code & IOR_BOTH))
                {
                    parity++;
                }
            }
            if (!interior)
            {
                continue;
            }

            // apply context to vertices (to perform replacing/merging vertices)
            // smh#8
            Shape tmpV1 = Context()!.InternalApply(splitLineVertex[i - 1]);
            Shape tmpV2 = Context()!.InternalApply(splitLineVertex[i]);
            Vertex v1 = ShapeCasts.Vertex(tmpV1);
            Vertex v2 = ShapeCasts.Vertex(tmpV2);
            // protection against creating null-length edges or edges lying inside tolerance of vertices
            // first and last vertices for split line can not be merged to each other
            bool canbeMerged = (/*myClosedMode &&*/ (i - 1 > 1 || i < splitLinePar.Length()));
            double maxTol = MaxTolerance();
            // case when max tolerance is not defined tolerance of vertices will be used as is
            if (maxTol <= 2.0 * Kerf.Engine.Foundation.Precision.Confusion())
            {
                maxTol = Kerf.Engine.Foundation.Precision.Infinite();
            }
            double tol1 = Min(Tool.Tolerance(v1), maxTol);
            double tol2 = Min(Tool.Tolerance(v2), maxTol);
            Point3D p1 = Tool.Pnt(v1);
            Point3D p2 = Tool.Pnt(v2);
            double d = p1.SquareDistanceTo(p2);
            if (splitLinePar[i] - splitLinePar[i - 1] < Kerf.Engine.Foundation.Precision.PConfusion()
                || (canbeMerged && (d <= (tol1 * tol1) || d <= (tol2 * tol2))))
            { // BRepTools::Compare(V1, V2)) ) {
                if (!v1.IsSame(v2))
                { // merge coincident vertices
                    var sbv = new VertexBuilder();
                    Vertex v = sbv.InternalCombineVertex(v1, v2);
                    Context()!.Replace(v1, v.Oriented(v1.Orientation));
                    Context()!.Replace(v2, v.Oriented(v2.Orientation));
                    v1 = v2 = v;
                }
                continue;
            }

            // create an edge (without 3d curve), put it in wire segment and add to sequence
            // NOTE: i here is always >1
            var edge = new Edge();
            b.MakeEdge(ref edge);
            v1.Orient(Forward);
            v2.Orient(Reversed);
            b.Add(edge, v1);
            b.Add(edge, v2);
            var lin1 = new LineCurve2D(line);
            var lin2 = new LineCurve2D(line);
            b.UpdateEdge(edge, lin1, lin2, _face, Kerf.Engine.Foundation.Precision.Confusion());
            b.Range(edge, _face, splitLinePar[i - 1], splitLinePar[i]);

            var sbwd = new ExtendWireData();
            sbwd.Add(edge);
            var seg = new WireSegmentFixer(sbwd, External);

            // set patch indices
            DefinePatch(seg, IOR_UNDEF, isCutByU, cutIndex);
            if (!isCutByU)
            {
                double shiftU = (_closedMode && _uClosed
                                   ? ShapeAnalysisAnalysis.AdjustToPeriod(splitLinePar[i - 1] - Tolint, _grid!.UJointValue(1),
                                                                  _grid.UJointValue(2))
                                   : 0.0);
                double par = splitLinePar[i - 1] + shiftU;

                seg.DefineIUMin(1, GetPatchIndex(par + Kerf.Engine.Foundation.Precision.PConfusion(), _grid!.InternalUJointValues()!, _uClosed));
                seg.DefineIUMax(1, GetPatchIndex(par - Kerf.Engine.Foundation.Precision.PConfusion(), _grid.InternalUJointValues()!, _uClosed) + 1);
            }
            else
            {
                double shiftV = (_closedMode && _vClosed
                                   ? ShapeAnalysisAnalysis.AdjustToPeriod(splitLinePar[i - 1] - Tolint, _grid!.VJointValue(1),
                                                                  _grid.VJointValue(2))
                                   : 0.0);
                double par = splitLinePar[i - 1] + shiftV;
                seg.DefineIVMin(1, GetPatchIndex(par + Kerf.Engine.Foundation.Precision.PConfusion(), _grid!.InternalVJointValues()!, _vClosed));
                seg.DefineIVMax(1, GetPatchIndex(par - Kerf.Engine.Foundation.Precision.PConfusion(), _grid.InternalVJointValues()!, _vClosed) + 1);
            }
            wires.Append(seg);
        }
        if (parity % 2 != 0)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL4);
        }

        // Apply context to all wires to perform all recorded replacements/merging
        for (i = 1; i <= wires.Length(); i++)
        {
            for (int j = 1; j <= wires[i].EdgeCount;)
            {
                j += ApplyContext(wires[i], j, Context()!);
            }
        }
    }

    /// <summary>
    /// Splits the initial set of (closed) wires by the grid's joint lines; the parts of the joint lines inside the face
    /// are added as EXTERNAL wires sharing vertices with the wires they cross.
    /// </summary>
    protected void SplitByGrid(NCollection_Sequence<WireSegmentFixer> seqw)
    {
        // process splitting by U- anv V-seams (i.e. U=const and V=const curves)
        // closed composite surface is processed as periodic
        BrepTools.UVBounds(_face, out double uf, out double ul, out double vf, out double vl);
        _grid!.ParameterBounds(out double umin, out double umax, out double vmin, out double vmax);

        // value of precision to define number of patch should be the same as used in the definitin
        // position of point relatively to seam edge (TOLINT)
        double pprec = Tolint; //::Precision::PConfusion();
        int i = 1;
        if (_closedMode)
        {
            // for closed mode when only one patch exist and location of the splitting line is coincident
            // with first joint value Therefore in this case it is necessary to move all wire segments in
            // the range of the patch between first and last joint values. Then all wire segments are lie
            // between -period and period in order to have valid split ranges after splitting. Because for
            // closed mode cut index always equal to 1 and parts of segments after splitting always should
            // have index either (0,1) or (1,2).
            for (i = 1; i <= seqw.Length(); i++)
            {
                ref WireSegmentFixer wire = ref seqw.ChangeValue(i);

                Shape atmpF = _face.EmptyCopied();
                var b = new BrepBuilder();
                atmpF.Orient(Forward);
                b.Add(atmpF, wire.WireData().InternalWire());
                ShapeAnalysisAnalysis.GetFaceUVBounds(ShapeCasts.Face(atmpF), out double uf1, out double ul1, out double vf1, out double vl1);

                // for closed mode it is necessary to move wire segment in the interval defined by first and
                // last grid UV values
                double shiftU = (_closedMode && _uClosed
                                   ? ShapeAnalysisAnalysis.AdjustToPeriod(ul1 - pprec, _grid.UJointValue(1), _grid.UJointValue(2))
                                   : 0.0);
                double shiftV = (_closedMode && _vClosed
                                   ? ShapeAnalysisAnalysis.AdjustToPeriod(vl1 - pprec, _grid.VJointValue(1), _grid.VJointValue(2))
                                   : 0.0);
                uf1 += shiftU;
                ul1 += shiftU;
                vf1 += shiftV;
                vl1 += shiftV;
                // limit patch indices to be in range of grid (extended for periodic) (0, 2)
                // in same cases for example trj4_pm2-ug-203.stp (entity #8024) wire in 2D space has length
                // greater then period
                int iumin = Max(0, GetPatchIndex(uf1 + pprec, _grid.InternalUJointValues()!, _uClosed));
                int iumax = GetPatchIndex(ul1 - pprec, _grid.InternalUJointValues()!, _uClosed) + 1;

                for (int j = 1; j <= wire.EdgeCount; j++)
                {
                    wire.DefineIUMin(j, iumin);
                    wire.DefineIUMax(j, iumax);
                }

                int ivmin = Max(0, GetPatchIndex(vf1 + pprec, _grid.InternalVJointValues()!, _vClosed));
                int ivmax = GetPatchIndex(vl1 - pprec, _grid.InternalVJointValues()!, _vClosed) + 1;

                for (int j = 1; j <= wire.EdgeCount; j++)
                {
                    wire.DefineIVMin(j, ivmin);
                    wire.DefineIVMax(j, ivmax);
                }
            }
        }
        else
        {
            // limit patch indices to be in range of grid (extended for periodic)
            int iumin = GetPatchIndex(uf + pprec, _grid.InternalUJointValues()!, _uClosed);
            int iumax = GetPatchIndex(ul - pprec, _grid.InternalUJointValues()!, _uClosed) + 1;
            for (i = 1; i <= seqw.Length(); i++)
            {
                ref WireSegmentFixer wire = ref seqw.ChangeValue(i);
                for (int j = 1; j <= wire.EdgeCount; j++)
                {
                    wire.DefineIUMin(j, iumin);
                    wire.DefineIUMax(j, iumax);
                }
            }
            int ivmin = GetPatchIndex(vf + pprec, _grid.InternalVJointValues()!, _vClosed);
            int ivmax = GetPatchIndex(vl - pprec, _grid.InternalVJointValues()!, _vClosed) + 1;
            for (i = 1; i <= seqw.Length(); i++)
            {
                ref WireSegmentFixer wire = ref seqw.ChangeValue(i);
                for (int j = 1; j <= wire.EdgeCount; j++)
                {
                    wire.DefineIVMin(j, ivmin);
                    wire.DefineIVMax(j, ivmax);
                }
            }
        }
        // split by u lines

        for (i = (_uClosed ? 1 : 2); i <= _grid.UPatchCount; i++)
        {
            var pos = new Point2D(_grid.UJointValue(i), 0.0); // 0. - for infinite ranges: myGrid->VJointValue(1) ;
            var line = new Line2D(pos, new Direction2D(Direction2D.D.Y));
            if (!_closedMode && _uClosed)
            {
                double period = umax - umin;
                double x = pos.X;
                double sh = ShapeAnalysisAnalysis.AdjustToPeriod(x, uf, uf + period);
                for (; x + sh <= ul + pprec; sh += period)
                {
                    Line2D ln = line.Translated(new Vector2D(sh, 0));
                    int cutIndex = GetPatchIndex(x + sh + pprec, _grid.InternalUJointValues()!, _uClosed);
                    SplitByLine(seqw, ln, true, cutIndex);
                }
            }
            else
            {
                SplitByLine(seqw, line, true, i);
            }
        }

        // split by v lines
        for (i = (_vClosed ? 1 : 2); i <= _grid.VPatchCount; i++)
        {
            var pos = new Point2D(0.0, _grid.VJointValue(i));
            var line = new Line2D(pos, new Direction2D(Direction2D.D.X));
            if (!_closedMode && _vClosed)
            {
                double period = vmax - vmin;
                double y = pos.Y;
                double sh = ShapeAnalysisAnalysis.AdjustToPeriod(y, vf, vf + period);
                for (; y + sh <= vl + pprec; sh += period)
                {
                    Line2D ln = line.Translated(new Vector2D(0, sh));
                    int cutIndex = GetPatchIndex(y + sh + pprec, _grid.InternalVJointValues()!, _vClosed);
                    SplitByLine(seqw, ln, false, cutIndex);
                }
            }
            else
            {
                SplitByLine(seqw, line, false, i);
            }
        }
    }

    /// <summary>
    /// Breaks the wires into open segments at the common (splitting) vertices, so that each segment is either closed
    /// and touches no other, or touches others at its ends; each then lies on one patch.
    /// </summary>
    protected void BreakWires(NCollection_Sequence<WireSegmentFixer> seqw)
    {
        // split all the wires by vertices
        var splitVertices = new HashSet<Shape>(ShapeHasher.Instance);
        var sae = new EdgeAnalysis();

        // first collect splitting vertices
        int i; // svv #1
        for (i = 1; i <= seqw.Length(); i++)
        {
            Orientation ori_wire = seqw[i].Orientation();
            if (ori_wire != External && ori_wire != Internal)
            {
                continue;
            }

            ExtendWireData sbwd = seqw[i].WireData();
            for (int j = 1; j <= sbwd.EdgeCount; j++)
            {
                Edge edge = sbwd.Edge(j);
                Orientation ori_edge = (ori_wire == External ? ori_wire : edge.Orientation);
                if (ori_edge == External)
                {
                    splitVertices.Add(sae.FirstVertex(edge));
                    splitVertices.Add(sae.LastVertex(edge));
                }
            }
        }

        // and then split each vire
        // Here each wire is supposed to be connected (while probably not closed)
        for (i = 1; i <= seqw.Length(); i++)
        {
            Orientation ori = seqw[i].Orientation();
            WireSegmentFixer wire = seqw[i];
            if (wire.IsVertex)
            {
                continue;
            }
            ExtendWireData sbwd = wire.WireData();

            // find first vertex for split
            int j; // svv #1
            for (j = 1; j <= sbwd.EdgeCount; j++)
            {
                Vertex v = sae.FirstVertex(sbwd.Edge(j));
                if (splitVertices.Contains(v))
                {
                    break;
                }
            }
            if (j > sbwd.EdgeCount)
            {
                continue; // splitting not needed
            }

            // if first split of closed edge is not its start, make permutation
            int shift = 0;
            if (j > 1 && !_closedMode && wire.IsClosed())
            {
                Vertex v = sae.FirstVertex(sbwd.Edge(1));
                if (!splitVertices.Contains(v))
                {
                    shift = j - 1;
                }
                // wire.SetLast ( j-1 );
            }

            // perform splitting
            int nbnew = 0;
            var newwire = new WireSegmentFixer();
            Orientation curOri = ori;
            for (int ind = 1; ind <= sbwd.EdgeCount; ind++)
            {
                j = 1 + (ind - 1 + shift) % sbwd.EdgeCount;
                Edge edge = sbwd.Edge(j);
                Vertex v = sae.FirstVertex(edge);
                if (ind == 1 || splitVertices.Contains(v))
                {
                    if (newwire.EdgeCount != 0)
                    {
                        newwire.Orientation(curOri);
                        // ShapeFix_WireSegment seg ( newwire, ori );
                        seqw.InsertBefore(i++, newwire);
                        nbnew++;
                    }
                    newwire.Clear();
                    curOri = ori;
                }
                wire.GetPatchIndex(j, out int iumin, out int iumax, out int ivmin, out int ivmax);
                if (ori == Internal && edge.Orientation == External)
                {
                    curOri = External;
                    edge.Orient(Forward);
                    nbnew++;
                }
                newwire.AddEdge(0, edge, iumin, iumax, ivmin, ivmax);
            }
            if (nbnew != 0)
            {
                newwire.Orientation(curOri);
                // ShapeFix_WireSegment seg ( newwire, ori );
                seqw.SetValue(i, newwire);
            }
        }
    }

    // BUC60035 2053: check if wire segment is very short (in order not to skip it)
    // 0  - long
    // 1  - short even in 2d (to be taken always)
    // -1 - short in 3d but not in 2d (to be checked after algo and atteching to
    //      another wire if alone)
    private static int IsShortSegment(in WireSegmentFixer seg, in Face myFace, Surface myGrid,
                                      in Placement myLoc, double uResolution, double vResolution)
    {
        Vertex vf = seg.FirstVertex();
        if (!vf.IsSame(seg.LastVertex()))
        {
            return 0;
        }

        Point3D pnt = Tool.Pnt(vf);
        double tol = Tool.Tolerance(vf);
        double tol2 = tol * tol;

        int code = 1;
        var sae = new EdgeAnalysis();
        ExtendWireData sbwd = seg.WireData();
        for (int i = 1; i <= sbwd.EdgeCount; i++)
        {
            Edge edge = sbwd.Edge(i);
            if (!vf.IsSame(sae.LastVertex(edge)))
            {
                return 0;
            }
            if (!sae.PCurve(edge, myFace, out Curve2D? c2d, out double f, out double l))
            {
                continue;
            }

            // check 2d
            Point2D endPnt = c2d!.Value(l);
            Point2D midPnt = c2d.Value((f + l) / 2);
            if (!IsCoincided(endPnt, midPnt, uResolution, vResolution, tol))
            {
                code = -1;
            }

            // check 3d
            Point3D midPnt3d = myGrid.Value(midPnt.X, midPnt.Y);
            if (!myLoc.IsIdentity)
            {
                midPnt3d = midPnt3d.Transformed(myLoc.Transformation);
            }
            if (midPnt3d.SquareDistanceTo(pnt) > tol2)
            {
                return 0;
            }
        }
        return code;
    }

    private static bool IsSamePatch(in WireSegmentFixer wire, int nu, int nv, ref int iumin, ref int iumax,
                                    ref int ivmin, ref int ivmax, bool extend = false)
    {
        // get patch indices for current segment
        wire.GetPatchIndex(1, out int jumin, out int jumax, out int jvmin, out int jvmax);

        // shift to the same period
        int du = 0, dv = 0;
        if (jumin - iumin > nu)
        {
            du = -(jumin - iumin) / nu;
        }
        else if (iumin - jumin > nu)
        {
            du = (iumin - jumin) / nu;
        }
        if (jvmin - ivmin > nv)
        {
            dv = -(jvmin - ivmin) / nv;
        }
        else if (ivmin - jvmin > nv)
        {
            dv = (ivmin - jvmin) / nv;
        }
        if (du != 0)
        {
            jumin += du * nu;
            jumax += du * nu;
        }
        if (dv != 0)
        {
            jvmin += dv * nv;
            jvmax += dv * nv;
        }

        // compute common (extended) indices
        int iun = Math.Min(iumin, jumin);
        int iux = Math.Max(iumax, jumax);
        int ivn = Math.Min(ivmin, jvmin);
        int ivx = Math.Max(ivmax, jvmax);
        bool ok = (iun == iux || iun + 1 == iux) && (ivn == ivx || ivn + 1 == ivx);
        if (ok && extend)
        {
            iumin = iun;
            iumax = iux;
            ivmin = ivn;
            ivmax = ivx;
        }
        return ok;
    }

    /// <summary>
    /// Collects the (already split) wire segments into closed wires, starting from REVERSED and FORWARD segments and
    /// taking EXTERNAL ones at forks, where the leftmost way is chosen.
    /// </summary>
    protected void CollectWires(NCollection_Sequence<WireSegmentFixer> wires,
                                NCollection_Sequence<WireSegmentFixer> seqw)
    {
        var sae = new EdgeAnalysis();
        int i; // svv #1
        // Collect information on short closed segments
        var shorts = new NCollection_Array1<int>(1, seqw.Length());
        for (i = 1; i <= seqw.Length(); i++)
        {
            if (seqw[i].IsVertex || seqw[i].Orientation() == Internal)
            {
                wires.Append(seqw[i]);
                seqw.Remove(i);
                i--;
                continue;
            }
            int isshort = IsShortSegment(seqw[i], _face, _grid!, _loc, _uResolution, _vResolution);
            shorts.SetValue(i, isshort);
            if (isshort > 0
                && (seqw[i].Orientation() == External
                    || (seqw[i].EdgeCount == 1 && //: abv 13.05.02: OCC320 - remove if degenerated
                        Tool.Degenerated(seqw[i].Edge(1)))))
            {
                seqw[i].Orientation(Internal);
            }
        }

        ExtendWireData? sbwd = null;
        Point2D endPnt = new Point2D(), firstPnt = new Point2D();
        Vector2D endTan = new Vector2D(), firstTan = new Vector2D();
        Vertex firstV = new Vertex(), endV = new Vertex();
        Edge firstEdge = new Edge(), lastEdge = new Edge();
        double tol = 0;
        int iumin = 0, iumax = 0, ivmin = 0, ivmax = 0;
        double dsu = 0.0, dsv = 0.0;
        bool canBeClosed = false;
        for (;;)
        {
            int index = 0;
            bool misoriented = true, samepatch = false;
            bool reverse = false, connected = false;
            double angle = -M_PI, mindist = RealLast();
            int weigth = 0;
            double shiftu = 0.0, shiftv = 0.0;

            // find next segment to connect (or first if sbwd is NULL)
            for (i = 1; i <= seqw.Length(); i++)
            {
                WireSegmentFixer seg = seqw.Value(i);
                if (seg.IsVertex)
                {
                    continue;
                }
                Orientation or = seg.Orientation();
                if (or == Internal)
                {
                    continue;
                }

                // for first segment, take any
                if (sbwd is null)
                {
                    if (shorts[i] > 0)
                    {
                        continue;
                    }
                    if (or == External)
                    {
                        continue;
                    }
                    if (or == Forward)
                    {
                        reverse = true;
                    }
                    index = i;
                    seg.GetPatchIndex(1, out iumin, out iumax, out ivmin, out ivmax);

                    misoriented = false;
                    dsu = dsv = 0.0;
                    break;
                }

                // check whether current segment is on the same patch with previous
                bool sp = IsSamePatch(seg, _grid!.UPatchCount, _grid.VPatchCount, ref iumin, ref iumax, ref ivmin, ref ivmax);

                // not same patch has lowest priority
                if (!sp && (canBeClosed || (index != 0 && samepatch)))
                {
                    continue;
                }

                // try to connect, with the following priorities:
                // The name of property      Weigth:
                // sharing vertex            auto
                // samepatch = 1             16
                // ! sameedge                auto
                // misorientation = 0        8
                // connected in 2d           4
                // distance                  2
                // short                     auto
                // angle ->> PI              1
                ExtendWireData wire = seg.WireData();
                for (int j = 0; j < 2; j++)
                {
                    if (!endV.IsSame(j != 0 ? seg.LastVertex() : seg.FirstVertex()))
                    {
                        continue;
                    }

                    // check for misorientation only if nothing better is found
                    bool misor = (or == (j != 0 ? Reversed : Forward));
                    // if ( misor ) continue; // temporarily, to be improved

                    // returning back by the same edge is lowest priority
                    if (lastEdge.IsSame(wire.Edge(j != 0 ? wire.EdgeCount : 1)))
                    {
                        if (index == 0 && !canBeClosed)
                        { // || ( sp && ! samepatch ) ) {
                            index       = i;
                            reverse     = j != 0;
                            connected   = true;
                            misoriented = misor;
                            samepatch   = sp;
                            weigth      = (sp ? 16 : 0) + (connected ? 8 : 0) + (!misor ? 4 : 0);
                            dsu = dsv = 0.0;
                        }
                        continue;
                    }

                    // compute starting tangent
                    var lPnt = new Point2D();
                    var lVec = new Vector2D();
                    int k;
                    double edgeTol = 0;
                    for (k = 1; k <= wire.EdgeCount; k++)
                    {
                        Shape tmpE = wire.Edge(wire.EdgeCount - k + 1).Reversed();
                        Edge edge = (j != 0 ? ShapeCasts.Edge(tmpE) : wire.Edge(k));
                        edgeTol = Tool.Tolerance(edge);
                        // if ( sae.GetEndTangent2d ( edge, myFace, false, lPnt, lVec ) ) break;
                        if (sae.GetEndTangent2d(edge, _face, false, out lPnt, out lVec, 1.0e-3))
                        {
                            break;
                        }
                    }
                    if (k > wire.EdgeCount)
                    {
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                    }

                    if (_closedMode)
                    {
                        if (_uClosed)
                        {
                            shiftu = ShapeAnalysisAnalysis.AdjustByPeriod(lPnt.X, endPnt.X, _uPeriod);
                            lPnt = lPnt.WithX(lPnt.X + shiftu);
                        }
                        if (_vClosed)
                        {
                            shiftv = ShapeAnalysisAnalysis.AdjustByPeriod(lPnt.Y, endPnt.Y, _vPeriod);
                            lPnt = lPnt.WithY(lPnt.Y + shiftv);
                        }
                    }

                    // short segment is to be taken with highest priority by angle
                    double ang = (shorts[i] > 0 ? M_PI : endTan.AngleTo(lVec));
                    if (_closedMode && shorts[i] <= 0 && M_PI - ang < Kerf.Engine.Foundation.Precision.Angular())
                    {
                        ang = 0.0; // abv 21 Mar 00: trj3_s1-md-214.stp #2471: avoid going back
                    }

                    // abv 05 Feb 02: face from Parasolid: use tolerance of edges for check
                    // for coincidence (instead of vertex tolerance) in order
                    // this check to be in agreement with check for position of wire segments
                    // thus avoiding bad effects on overlapping edges
                    double ctol = Max(edgeTol, Tool.Tolerance(endV /*lastEdge*/));
                    bool conn = IsCoincided(endPnt, lPnt, _uResolution, _vResolution, ctol);
                    double dist = endPnt.SquareDistanceTo(lPnt);

                    // check if case is better than last found
                    int w1 = (sp ? 16 : 0) + (conn ? 4 : 0) + (!misor ? 8 : 0);
                    int tail1 = (!conn && (dist < mindist) ? 2 : 0) + (ang > angle ? 1 : 0);
                    int tail2 = (!connected && (dist > mindist) ? 2 : 0) + (ang < angle ? 1 : 0);
                    if (w1 + tail1 <= weigth + tail2)
                    {
                        continue;
                    }

                    index       = i;
                    reverse     = j != 0;
                    angle       = ang;
                    mindist     = dist;
                    connected   = conn;
                    misoriented = misor;
                    samepatch   = sp;
                    weigth      = w1;
                    dsu         = shiftu;
                    dsv         = shiftv;
                }
            }

            // if next segment found, connect it
            if (index != 0)
            {
                if (misoriented)
                {
                    _invertEdgeStatus = true;
                }
                WireSegmentFixer seg = seqw.Value(index);
                if (sbwd is null)
                {
                    sbwd = new ExtendWireData();
                }
                else if (samepatch)
                { // extend patch indices
                    IsSamePatch(seg, _grid!.UPatchCount, _grid.VPatchCount, ref iumin, ref iumax, ref ivmin, ref ivmax, true);
                }

                // for closed mode in case if current segment is seam segment it is necessary to detect
                // crossing seam edge in order to have possibility to take candidate from other patch
                if (_closedMode)
                {
                    seg.GetPatchIndex(1, out iumin, out iumax, out ivmin, out ivmax);
                }

                // TopAbs_Orientation or = seg.Orientation();
                if (!reverse)
                {
                    sbwd.Add(seg.WireData());
                }
                else
                {
                    var wire = new ExtendWireData();
                    wire.ManifoldMode() = false;
                    wire.Add(seg.WireData());
                    wire.Reverse(_face);
                    sbwd.Add(wire);
                }
                if (seg.Orientation() == External)
                {
                    seg.Orientation(reverse ? Reversed : Forward);
                }
                else
                {
                    seg.Orientation(Internal);
                }
                seqw.SetValue(index, seg);
            }
            else if (sbwd is null)
            {
                break; // stop when no free segments available
            }
            // for first segment, remember start point
            if (endV.IsNull)
            {
                firstEdge = sbwd.Edge(1);
                firstV = sae.FirstVertex(firstEdge);
                // sae.GetEndTangent2d ( firstEdge, myFace, false, firstPnt, firstTan );
                sae.GetEndTangent2d(firstEdge, _face, false, out firstPnt, out firstTan, 1.0e-3);
            }

            // update last edge and vertex (only for not short segments)
            bool doupdate = (index != 0 && (shorts[index] <= 0 || endV.IsNull));
            if (doupdate)
            {
                lastEdge = sbwd.Edge(sbwd.EdgeCount);
                endV = sae.LastVertex(lastEdge);
                tol = Tool.Tolerance(endV);
                // BUC60035 2053: iteration on edges is required
                int k; // svv #1
                for (k = sbwd.EdgeCount; k >= 1; k--)
                {
                    // if ( sae.GetEndTangent2d ( sbwd->Edge ( k ), myFace, true, endPnt, endTan ) )
                    if (sae.GetEndTangent2d(sbwd.Edge(k), _face, true, out endPnt, out endTan, 1.0e-3))
                    {
                        break;
                    }
                }
                if (k < 1)
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                }
                if (_uClosed)
                {
                    endPnt = endPnt.WithX(endPnt.X + dsu);
                }
                if (_vClosed)
                {
                    endPnt = endPnt.WithY(endPnt.Y + dsv);
                }
            }

            // if closed or no next segment found, add to wires
            canBeClosed = endV.IsSame(firstV);
            if (index == 0
                || (canBeClosed && !lastEdge.IsSame(firstEdge) && // cylinder (seam)
                    IsCoincided(endPnt, firstPnt, _uResolution, _vResolution, 2.0 * tol)))
            {
                if (!endV.IsSame(sae.FirstVertex(firstEdge)))
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL5);
                }
                var s = new WireSegmentFixer(sbwd, Forward);
                s.DefineIUMin(1, iumin);
                s.DefineIUMax(1, iumax);
                s.DefineIVMin(1, ivmin);
                s.DefineIVMax(1, ivmax);
                wires.Append(s);
                sbwd = null;
                endV.Nullify();
                canBeClosed = false;
            }
        }

        // Check if some wires are short in 3d (lie entirely inside one vertex),
        // and if yes try to merge them with others
        // pdn The short seqments are still placed in "in" sequence.

        for (i = 1; i <= seqw.Length(); i++)
        {
            if (shorts[i] != 1 || seqw[i].IsVertex || seqw[i].Orientation() == Internal
                || seqw[i].Orientation() == External)
            {
                continue;
            }

            // find any other wire containing the same vertex
            ExtendWireData wd = seqw[i].WireData();
            Vertex v = seqw[i].FirstVertex();
            int minj = 0, mink = 0;
            double mindist = 0;
            bool samepatch = false;
            // int iumin, iumax, ivmin, ivmax;
            seqw[i].GetPatchIndex(1, out iumin, out iumax, out ivmin, out ivmax);
            sae.GetEndTangent2d(wd.Edge(1), _face, false, out Point2D p2d, out Vector2D vec);
            for (int j = 1; j <= wires.Length(); j++)
            {
                // if ( j == i ) continue;
                // occ::handle<ShapeExtend_WireData>
                sbwd = wires[j].WireData();
                for (int k = 1; k <= sbwd.EdgeCount; k++)
                {
                    if (!v.IsSame(sae.FirstVertex(sbwd.Edge(k))))
                    {
                        continue; // pdn I suppose that short segment should be inserted into the SAME vertex.
                    }

                    bool sp = IsSamePatch(wires[j], _grid!.UPatchCount, _grid.VPatchCount, ref iumin, ref iumax, ref ivmin, ref ivmax);
                    if (samepatch && !sp)
                    {
                        continue;
                    }
                    sae.GetEndTangent2d(sbwd.Edge(k), _face, false, out Point2D pp, out vec);
                    double dist = pp.SquareDistanceTo(p2d);
                    if (sp && !samepatch)
                    {
                        minj = j;
                        mink = k;
                        mindist = dist;
                        samepatch = sp;
                    }
                    else if (minj == 0 || mindist > dist)
                    {
                        minj = j;
                        mink = k;
                        mindist = dist;
                        samepatch = sp;
                    }
                }
            }
            if (minj == 0)
            {
                // pdn add into resulting sequence!
                var s = new WireSegmentFixer(wd, Forward);
                wires.Append(s);
                continue;
            }

            // and if found, merge
            // occ::handle<ShapeExtend_WireData>
            sbwd = wires[minj].WireData();
            for (int n = 1; n <= wd.EdgeCount; n++)
            {
                sbwd.Add(wd.Edge(n), mink++);
            }

            // wires.Remove ( i );
            // i--;
        }
    }

    private static Point2D GetMiddlePoint(in WireSegmentFixer wire, in Face face)
    {
        if (wire.IsVertex)
        {
            Vertex v = wire.GetVertex();
            Point3D p3D = Tool.Pnt(v);
            Surface? surf = Tool.Surface(face);
            var surfTool = new SurfaceAnalysis(surf);
            return surfTool.ValueOfUV(p3D, Kerf.Engine.Foundation.Precision.Confusion());
        }
        var box = new Box2d();
        var sae = new EdgeAnalysis();
        var sac = new CurveAnalysis();
        ExtendWireData wd = wire.WireData();
        for (int i = 1; i <= wd.EdgeCount; i++)
        {
            Edge e = wd.Edge(i);
            if (sae.PCurve(e, face, out Curve2D? c2d, out double cf, out double cl, false))
            {
                sac.FillBndBox(c2d!, cf, cl, 3, false, ref box);
                // box.Add(c2d->Value(cf));
                // box.Add(c2d->Value(cl));
                // box.Add(c2d->Value((cl+cf)/2.));
            }
        }
        if (box.IsVoid)
        {
            return new Point2D(0.0, 0.0);
        }
        box.Get(out double xmin, out double ymin, out double xmax, out double ymax);
        return new Point2D(0.5 * (xmax + xmin), 0.5 * (ymax + ymin));
    }

    /// <summary>
    /// Creates the faces on one patch of the grid, dispatching the given loops (wires) into one or several faces by
    /// their mutual position.
    /// </summary>
    protected void MakeFacesOnPatch(NCollection_Sequence<Shape> faces, Surface surf,
                                    NCollection_Sequence<Shape> loops)
    {
        var b = new BrepBuilder();

        // Case of single loop: just add it to face
        if (loops.Length() == 1)
        {
            var newFace = new Face();
            b.MakeFace(ref newFace, surf, _loc, Kerf.Engine.Foundation.Precision.Confusion());
            Shape aSH = loops.Value(1);
            if (aSH.Type != ShapeType.Wire)
            {
                return;
            }
            Wire wire = ShapeCasts.Wire(loops.Value(1));

            b.Add(newFace, wire);
            if (_invertEdgeStatus)
            {
                var sff = new FaceFixer(newFace);
                sff.FixAddNaturalBoundMode() = 0;
                var mapWires = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
                mapWires.Clear();
                sff.FixOrientation(mapWires);
                newFace = sff.InternalFace();
            }

            faces.Append(newFace);
            return;
        }

        // For several loops, first find roots
        // make pseudo-face,
        var pf = new Face();
        b.MakeFace(ref pf, surf, _loc, Kerf.Engine.Foundation.Precision.Confusion());
        Surface? atSurf = Tool.Surface(pf);

        var surfTool = new SurfaceAnalysis(atSurf);
        var roots = new NCollection_Sequence<Shape>();
        int i; // svv #1
        for (i = 1; i <= loops.Length(); i++)
        {
            Point2D unp;
            Wire wr;
            Shape shape = loops[i];
            if (shape.Type != ShapeType.Wire
                || (shape.Orientation != Forward && shape.Orientation != Reversed))
            {
                continue;
            }

            wr = ShapeCasts.Wire(loops[i]);
            var ew = new Iterator(wr);
            if (!ew.More())
            {
                continue;
            }

            Edge ed = ShapeCasts.Edge(ew.Value());
            while (ed.Orientation != Forward && ed.Orientation != Reversed)
            {
                ew.Next();
                if (ew.More())
                {
                    ed = ShapeCasts.Edge(ew.Value());
                }
                else
                {
                    break;
                }
            }
            if (!ew.More())
            {
                continue;
            }
            Curve2D? cw = Tool.CurveOnSurface(ed, pf, out double cf, out double cl);
            if (cw is null)
            {
                continue;
            }
            unp = cw.Value(0.5 * (cf + cl));

            int j; // svv #1
            for (j = 1; j <= loops.Length(); j++)
            {
                if (i == j)
                {
                    continue;
                }
                Shape shape2 = loops[j];
                if (shape2.Type != ShapeType.Wire
                    || (shape2.Orientation != Forward && shape2.Orientation != Reversed))
                {
                    continue;
                }
                Wire w1 = ShapeCasts.Wire(shape2);
                var awtmp = new Wire();
                b.MakeWire(ref awtmp);
                awtmp.Orient(Forward);
                var it = new Iterator(w1);
                int nbe = 0;
                for (; it.More(); it.Next())
                {
                    if (it.Value().Orientation == Forward || it.Value().Orientation == Reversed)
                    {
                        b.Add(awtmp, it.Value());
                        nbe++;
                    }
                }
                if (nbe == 0)
                {
                    continue;
                }
                var fc = new Face();
                b.MakeFace(ref fc, surf, _loc, Kerf.Engine.Foundation.Precision.Confusion());
                b.Add(fc, awtmp);
                var clas = new FClass2d(fc, Kerf.Engine.Foundation.Precision.PConfusion());
                Containment stPoint = clas.Perform(unp, false);
                if (stPoint == On || stPoint == Unknown)
                {
                    Edge anEdge = ShapeCasts.Edge(ew.Value());
                    Curve2D? aCW = Tool.CurveOnSurface(anEdge, pf, out double aCF, out double aCL);
                    // handle tangential case (ON)
                    while (stPoint == On || stPoint == Unknown)
                    {
                        stPoint = clas.Perform(aCW!.Value(aCL), false);
                        if (!ew.More())
                        {
                            break;
                        }
                        ew.Next();
                        if (!ew.More())
                        {
                            break;
                        }
                        Edge edge = ShapeCasts.Edge(ew.Value());
                        if (edge.Orientation != Forward && edge.Orientation != Reversed)
                        {
                            continue;
                        }
                        Curve2D? c2d = Tool.CurveOnSurface(edge, pf, out aCF, out aCL);
                        if (c2d is not null)
                        {
                            aCW = c2d;
                        }
                    }
                }
                Containment stInfin = clas.PerformInfinitePoint();
                if (stPoint != stInfin)
                {
                    break;
                }
            }
            if (j > loops.Length())
            {
                roots.Append(wr);
                // loops.Remove ( i-- );
            }
        }

        // And remove them from the list of loops
        for (i = 1; i <= loops.Length(); i++)
        {
            for (int j = 1; j <= roots.Length(); j++)
            {
                if (loops[i].IsSame(roots[j]))
                {
                    loops.Remove(i--);
                    break;
                }
            }
        }

        // check for lost wires, and if they are, make them roots
        if (roots.Length() <= 0 && loops.Length() > 0)
        {
            for (int j = 1; j <= loops.Length(); j++)
            {
                roots.Append(loops[j]);
            }
            loops.Clear();
        }

        // Then iterate on loops
        for (i = 1; i <= roots.Length(); i++)
        {
            bool reverse = false;
            Wire wire = ShapeCasts.Wire(roots[i]);
            var fc = new Face();
            b.MakeFace(ref fc, surf, _loc, Kerf.Engine.Foundation.Precision.Confusion());
            b.Add(fc, wire);
            var clas = new FClass2d(fc, Kerf.Engine.Foundation.Precision.PConfusion());
            if (clas.PerformInfinitePoint() == In)
            {
                reverse = true;
            }

            // find all holes for that loop
            var holes = new NCollection_Sequence<Shape>(); // holes in holes not supported
            int j; // svv #1
            for (j = 1; j <= loops.Length(); j++)
            {
                Point2D unp;
                if (loops[j].Type == ShapeType.Wire)
                {
                    Wire bw = ShapeCasts.Wire(loops[j]);
                    var ew = new Iterator(bw);
                    if (!ew.More())
                    {
                        continue;
                    }
                    Edge ed = ShapeCasts.Edge(ew.Value());
                    Curve2D? cw = Tool.CurveOnSurface(ed, pf, out double cf, out double cl);
                    if (cw is null)
                    {
                        continue;
                    }
                    unp = cw.Value(0.5 * (cf + cl));
                }
                else if (loops[j].Type == ShapeType.Vertex)
                {
                    Vertex v = ShapeCasts.Vertex(loops[j]);
                    Point3D p = Tool.Pnt(v);
                    unp = surfTool.ValueOfUV(p, Kerf.Engine.Foundation.Precision.Confusion());
                }
                else
                {
                    continue;
                }
                Containment state = clas.Perform(unp, false);
                if ((state == Out) == reverse)
                {
                    holes.Append(loops[j]);
                    loops.Remove(j--);
                }
            }

            // and add them to new face (no orienting is done)
            var newFace = new Face();
            b.MakeFace(ref newFace, surf, _loc, Kerf.Engine.Foundation.Precision.Confusion());
            b.Add(newFace, wire);
            for (j = 1; j <= holes.Length(); j++)
            {
                Shape sh = holes[j];
                if (sh.Type == ShapeType.Vertex)
                {
                    Vertex newV = TransferParametersProjAnalysis.InternalCopyNMVertex(ShapeCasts.Vertex(sh), newFace, _face);
                    Context()!.Replace(sh, newV);
                    b.Add(newFace, newV);
                }
                else
                {
                    b.Add(newFace, holes[j]);
                }
            }
            faces.Append(newFace);

            // check for lost wires, and if they are, make them roots
            if (i == roots.Length() && loops.Length() > 0)
            {
                for (j = 1; j <= loops.Length(); j++)
                {
                    Shape sh = loops[j];
                    if (sh.Type == ShapeType.Wire
                        && (sh.Orientation == Forward || sh.Orientation == Reversed))
                    {
                        roots.Append(loops[j]);
                    }
                }
                loops.Clear();
            }
        }
    }

    /// <summary>
    /// Creates the faces from the set of (closed) wires: each wire is put on its patch of the composite surface, its
    /// pcurves on the initial (pseudo-)face reassigned to that surface; wires one inside another make one face.
    /// </summary>
    public void DispatchWires(NCollection_Sequence<Shape> faces, NCollection_Sequence<WireSegmentFixer> wires)
    {
        var b = new BrepBuilder();

        // in closed mode, apply FixShifted to all wires before dispatching them
        if (_closedMode)
        {
            var sfw = new WireFixer();
            sfw.SetFace(_face);
            sfw.SetPrecision(Precision());

            // pdn: shift pcurves in the seam to make OK shape w/o fixshifted
            int i1;
            for (i1 = 1; i1 <= wires.Length(); i1++)
            {
                if (wires[i1].IsVertex)
                {
                    continue;
                }
                ExtendWireData sbwd = wires[i1].WireData();

                for (int jL = 1; jL <= sbwd.EdgeCount; jL++)
                {
                    Edge e = sbwd.Edge(jL);
                    if (e.Orientation == Reversed && Tool.IsClosed(e, _face))
                    {
                        Curve2D c21 = Tool.CurveOnSurface(e, _face, out double f1, out double l1)!;
                        Shape dummy = e.Reversed();
                        Curve2D c22 = Tool.CurveOnSurface(ShapeCasts.Edge(dummy), _face, out double f2, out double l2)!;
                        double dPreci = Kerf.Engine.Foundation.Precision.PConfusion() * Kerf.Engine.Foundation.Precision.PConfusion();
                        Point2D pf1 = c21.Value(f1);
                        Point2D pl1 = c21.Value(l1);
                        Point2D pf2 = c22.Value(f2);
                        Point2D pl2 = c22.Value(l2);
                        if (ReferenceEquals(c21, c22) || pf1.SquareDistanceTo(pf2) < dPreci || pl1.SquareDistanceTo(pl2) < dPreci)
                        {
                            var shift = new Vector2D(0.0, 0.0);
                            if (_uClosed && Math.Abs(pf2.X - pl2.X) < Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                shift = shift.WithX(_uPeriod);
                            }
                            if (_vClosed && Math.Abs(pf2.Y - pl2.Y) < Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                shift = shift.WithY(_vPeriod);
                            }
                            // Kerf: frozen. OCCT translates the stored pcurve in place.
                            FaceFixer.EditPCurve(e, c22, c => c.Translate(shift));
                        }
                    }
                }
            }

            for (i1 = 1; i1 <= wires.Length(); i1++)
            {
                if (wires[i1].IsVertex)
                {
                    continue;
                }
                ExtendWireData sbwd = wires[i1].WireData();

                //: abv 30.08.01: torHalf2.sat: if wire contains single degenerated
                // edge, skip that wire
                if (sbwd.EdgeCount <= 0 || (sbwd.EdgeCount == 1 && Tool.Degenerated(sbwd.Edge(1))))
                {
                    wires.Remove(i1--);
                    continue;
                }

                sfw.Load(sbwd);
                sfw.FixShifted();

                // force recomputation of degenerated edges (clear pcurves)
                var sbe1 = new BuildEdgeBuilder();
                for (int jL = 1; jL <= sbwd.EdgeCount; jL++)
                {
                    if (Tool.Degenerated(sbwd.Edge(jL)))
                    {
                        sbe1.RemovePCurve(sbwd.Edge(jL), _face);
                    }
                    // sfw.FixDegenerated(jL);
                }
                sfw.FixDegenerated();
            }
        }

        // Compute center points for wires
        var mPnts = new NCollection_Sequence<Point2D>();
        int nb = wires.Length();

        // pdn protection on empty sequence
        if (nb == 0)
        {
            return;
        }

        int i; // svv #1
        for (i = 1; i <= nb; i++)
        {
            mPnts.Append(GetMiddlePoint(wires[i], _face));
        }

        // Put each wire on its own surface patch (by reassigning pcurves)
        // and build 3d curve if necessary
        var rs = new ReShapeBuilder();
        var sbe = new BuildEdgeBuilder();
        var sae = new EdgeAnalysis();
        var sfe = new EdgeFixer();

        _grid!.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        for (i = 1; i <= nb; i++)
        {
            Point2D pnt = mPnts[i];
            double ush = 0.0, vsh = 0.0;
            if (_uClosed)
            {
                ush = ShapeAnalysisAnalysis.AdjustToPeriod(pnt.X, u1, u2);
                pnt = pnt.WithX(pnt.X + ush);
            }
            if (_vClosed)
            {
                vsh = ShapeAnalysisAnalysis.AdjustToPeriod(pnt.Y, v1, v2);
                pnt = pnt.WithY(pnt.Y + vsh);
            }
            mPnts[i] = pnt;
            int indU = _grid.LocateUParameter(pnt.X);
            int indV = _grid.LocateVParameter(pnt.Y);

            // compute parametric transformation
            bool needT = _grid.GlobalToLocalTransformation(indU, indV, out double uFact, out Transform2D t);
            if (ush != 0.0 || vsh != 0.0)
            {
                var sh = new Transform2D();
                sh = sh.WithTranslation(new Vector2D(ush, vsh));
                t = t.Multiplied(sh);
                needT = true;
            }
            if (wires[i].IsVertex)
            {
                continue;
            }
            Surface surf = _grid.Patch(indU, indV);
            var face = new Face();
            b.MakeFace(ref face, surf, _loc, Kerf.Engine.Foundation.Precision.Confusion());
            ExtendWireData sewd = wires[i].WireData();
            for (int j = 1; j <= sewd.EdgeCount; j++)
            {
                // int nsplit = ApplyContext ( sewd, j, context );
                // if ( nsplit <1 ) { j--; continue; }

                Edge edge = sewd.Edge(j);

                // !! Accurately copy pcurves for SEAMS and SEAM-like edges !!

                // if edge is already copied, don`t copy any more
                Edge newEdge;
                Edge initEdge = edge;
                bool ismanifold = (edge.Orientation == Forward || edge.Orientation == Reversed);
                if (rs.IsRecorded(edge))
                {
                    // smh#8
                    Shape tmpNE = rs.InternalValue(edge);
                    newEdge = ShapeCasts.Edge(tmpNE);
                }
                else
                {
                    if (!ismanifold)
                    {
                        initEdge.Orient(Forward);
                    }

                    newEdge = sbe.InternalCopy(initEdge, false);
                    if (!ismanifold)
                    {
                        newEdge.Orient(edge.Orientation);
                    }
                    rs.Replace(edge, newEdge);
                    Context()!.Replace(edge, newEdge);
                }

                sbe.ReassignPCurve(newEdge, _face, face);

                // transform pcurve to parametric space of patch
                if (needT)
                {
                    if (sae.PCurve(newEdge, face, out Curve2D? c2d, out double f, out double l, false))
                    {
                        double newf = f, newl = l;
                        Curve2D c2dnew = sbe.TransformPCurve(c2d!, t, uFact, ref newf, ref newl);
                        if (Tool.IsClosed(newEdge, face))
                        {
                            // smh#8
                            Shape tmpE = newEdge.Reversed();
                            Edge e2 = ShapeCasts.Edge(tmpE);
                            if (sae.PCurve(e2, face, out Curve2D? c2d2, out _, out _, false))
                            {
                                if (newEdge.Orientation == Forward)
                                {
                                    b.UpdateEdge(newEdge, c2dnew, c2d2!, face, 0.0);
                                }
                                else
                                {
                                    b.UpdateEdge(newEdge, c2d2!, c2dnew, face, 0.0);
                                }
                            }
                            else
                            {
                                b.UpdateEdge(newEdge, c2dnew, face, 0.0);
                            }
                        }
                        else
                        {
                            b.UpdateEdge(newEdge, c2dnew, face, 0.0);
                        }
                        b.Range(newEdge, face, newf, newl);
                        if ((newf != f || newl != l) && !Tool.Degenerated(newEdge))
                        {
                            b.SameRange(newEdge, false);
                        }
                    }
                }

                if (!Tool.SameRange(newEdge))
                {
                    Edge etmp;
                    if (!ismanifold)
                    {
                        Edge afe = ShapeCasts.Edge(newEdge.Oriented(Forward));
                        etmp = sbe.InternalCopy(afe, false);
                    }
                    else
                    {
                        etmp = sbe.InternalCopy(newEdge, false);
                    }
                    sfe.FixAddCurve3d(etmp);
                    if (sae.Curve3d(etmp, out Curve3D? c3d, out double cf, out double cl, false))
                    {
                        b.UpdateEdge(newEdge, c3d, 0.0);
                        sbe.SetRange3d(newEdge, cf, cl);
                    }
                }
                else
                {
                    sfe.FixAddCurve3d(newEdge);
                }
                sewd.Set(newEdge, j);
            }
        }

        // Collect wires in packets lying on same surface and dispatch them
        var used = new NCollection_Array1<bool>(1, nb);
        used.Init(false);
        for (;;)
        {
            var loops = new NCollection_Sequence<Shape>();

            Surface? Surf = null;
            for (i = 1; i <= nb; i++)
            {
                if (used[i])
                {
                    continue;
                }
                Surface s = _grid.Patch(mPnts[i]);
                if (Surf is null)
                {
                    Surf = s;
                }
                else if (!ReferenceEquals(s, Surf))
                {
                    continue;
                }
                used[i] = true;
                WireSegmentFixer seg = wires[i];
                if (seg.IsVertex)
                {
                    Vertex vert = seg.GetVertex();
                    if (vert.Orientation == Internal)
                    {
                        loops.Append(wires[i].GetVertex());
                    }
                }
                else
                {
                    ExtendWireData? wd = seg.WireData();
                    if (wd is not null)
                    {
                        loops.Append(wd.InternalWire());
                    }
                }
            }
            if (Surf is null)
            {
                break;
            }

            MakeFacesOnPatch(faces, Surf, loops);
        }
    }

    /// <summary>Sets the tool transferring parameters from 3d to 2d and back.</summary>
    public void SetTransferParamTool(TransferParametersAnalysis transferParam)
    {
        _transferParamTool = transferParam;
    }

    /// <summary>Gets the tool transferring parameters from 3d to 2d and back.</summary>
    public TransferParametersAnalysis GetTransferParamTool() => _transferParamTool;

    /// <summary>
    /// Returns (modifiable) the 'closed' mode, which makes the tool take all pcurves on a closed surface modulo the
    /// period: less reliable, but deals with wires closed in 3d and open in 2d (a missing seam). Default false.
    /// </summary>
    public ref bool ClosedMode() => ref _closedMode;
}
