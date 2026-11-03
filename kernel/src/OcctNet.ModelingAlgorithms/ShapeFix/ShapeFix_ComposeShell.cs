// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_ComposeShell.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_ComposeShell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//    pdn  01.06.99 S4205: handling not-SameRange edges
//    abv  22.07.99 implementing patch indices
//    svv  10.01.00 porting on DEC

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.TransformForm;
using static Occt.IntRes2d_Position;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

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
/// the context and replaces their vertices), as OCCT's. A face holding frozen shapes is copied at
/// <see cref="Perform"/> (<see cref="CopyAtEntry"/>, decision 583); <see cref="Result"/> gives back the
/// caller's own sub-shapes wherever they were left unchanged, re-keys the context to the caller's shapes and freezes
/// the result. ShapeFix_Face runs it on its own unfrozen faces.
/// </para>
/// </remarks>
public class ShapeFix_ComposeShell : ShapeFix_Root
{
    /// <summary>The orientation of the face given.</summary>
    protected Orientation myOrient;
    /// <summary>The resulting shell or face.</summary>
    protected Shape myResult;
    /// <summary>The status of the last <see cref="Perform"/>.</summary>
    protected int myStatus;

    private ShapeExtend_CompositeSurface?     myGrid;
    private Placement                   myLoc;
    private Face                       myFace;
    private double                            myUResolution;
    private double                            myVResolution;
    private ShapeAnalysis_TransferParameters  myTransferParamTool;
    private bool                              myInvertEdgeStatus;
    private bool                              myClosedMode;
    private bool                              myUClosed;
    private bool                              myVClosed;
    private double                            myUPeriod;
    private double                            myVPeriod;

    // OCCT.NET: the copy at the entry (decision 583); null when the face holds no frozen shape.
    private CopyAtEntry? myEntry;
    private bool myToEnter;

    /// <summary>Creates an empty tool.</summary>
    public ShapeFix_ComposeShell()
    {
        myOrient            = Forward;
        myStatus            = 0;
        myResult            = new Shape();
        myLoc               = new Placement();
        myFace              = new Face();
        myUResolution       = RealLast();
        myVResolution       = RealLast();
        myInvertEdgeStatus  = true;
        myClosedMode        = false;
        myUClosed           = false;
        myVClosed           = false;
        myUPeriod           = 0.0;
        myVPeriod           = 0.0;
        myTransferParamTool = new ShapeAnalysis_TransferParametersProj();
    }

    /// <summary>
    /// Initializes with a composite surface, a face and a precision. The face gives both the set of wires and the way
    /// of getting pcurves. The precision is used (with the edges' tolerances) for subtle cases, such as tangential
    /// intersections.
    /// </summary>
    public void Init(ShapeExtend_CompositeSurface Grid, in Placement L, in Face Face, double Prec)
    {
        myGrid    = Grid;
        myUClosed = myGrid.IsUClosed;
        myVClosed = myGrid.IsVClosed;
        myUPeriod = myGrid.UJointValue(myGrid.NbUPatches() + 1) - myGrid.UJointValue(1);
        myVPeriod = myGrid.VJointValue(myGrid.NbVPatches() + 1) - myGrid.VJointValue(1);

        //  DTK-CKY 100531 : protection against very thin face
        //  Test "isclosed" should be filtered on the overall (non trimmed) surface, must be closed
        Surface theSurface = Tool.Surface(Face, out myLoc)!;
        // avoid false detection of 'Closed' on very thin faces
        if (theSurface is ElementarySurface)
        {
            myUClosed = myUClosed && theSurface.IsUClosed;
            myVClosed = myVClosed && theSurface.IsVClosed;
        }
        else
        {
            double GU0 = 0.0, GU1 = 0.0, GV0 = 0.0, GV1 = 0.0;
            theSurface.Bounds(out double U0, out double U1, out double V0, out double V1);
            if (Kerf.Engine.Foundation.Precision.IsInfinite(U0) || Kerf.Engine.Foundation.Precision.IsInfinite(U1) || Kerf.Engine.Foundation.Precision.IsInfinite(V0)
                || Kerf.Engine.Foundation.Precision.IsInfinite(V1))
            {
                BrepTools.UVBounds(Face, out GU0, out GU1, out GV0, out GV1);
            }
            if (myUClosed)
            {
                if (Kerf.Engine.Foundation.Precision.IsInfinite(V0))
                {
                    V0 = GV0;
                }
                if (Kerf.Engine.Foundation.Precision.IsInfinite(V1))
                {
                    V1 = GV1;
                }
                Point3D P0 = theSurface.Value(U0, (V0 + V1) / 2.0);
                Point3D P1 = theSurface.Value(U1, (V0 + V1) / 2.0);
                if (P0.Distance(P1) > Kerf.Engine.Foundation.Precision.Confusion() * 10)
                {
                    myUClosed = false;
                }
            }
            if (myVClosed)
            {
                if (Kerf.Engine.Foundation.Precision.IsInfinite(U0))
                {
                    U0 = GU0;
                }
                if (Kerf.Engine.Foundation.Precision.IsInfinite(U1))
                {
                    U1 = GU1;
                }
                Point3D P0 = theSurface.Value((U0 + U1) / 2.0, V0);
                Point3D P1 = theSurface.Value((U0 + U1) / 2.0, V1);
                if (P0.Distance(P1) > Kerf.Engine.Foundation.Precision.Confusion() * 10)
                {
                    myVClosed = false;
                }
            }
        }
        //  DTK-CKY 100531 end

        myLoc = L;
        // smh#8
        Shape tmpF = Face.Oriented(Forward);
        myFace   = ShapeCasts.Face(tmpF); // for correct dealing with seams
        myOrient = Face.Orientation();
        SetPrecision(Prec);
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);

        // Compute resolution (checking in 2d is necessary for splitting
        // degenerated edges and avoiding NotClosed)
        myUResolution = myVResolution = RealLast();
        for (int i = 1; i <= myGrid.NbUPatches(); i++)
        {
            double uRange = myGrid.UJointValue(i + 1) - myGrid.UJointValue(i);
            for (int j = 1; j <= myGrid.NbVPatches(); j++)
            {
                double vRange = myGrid.VJointValue(j + 1) - myGrid.VJointValue(j);
                myGrid.Patch(i, j).Bounds(out double u1, out double u2, out double v1, out double v2);
                var GAS = new SurfaceAdaptor(myGrid.Patch(i, j));
                double ures = GAS.UResolution(1.0) * uRange / (u2 - u1);
                double vres = GAS.VResolution(1.0) * vRange / (v2 - v1);
                if (ures > 0.0 && myUResolution > ures)
                {
                    myUResolution = ures;
                }
                if (vres > 0.0 && myVResolution > vres)
                {
                    myVResolution = vres;
                }
            }
        }
        if (myUResolution == RealLast())
        {
            myUResolution = Kerf.Engine.Foundation.Precision.Parametric(1.0);
        }
        if (myVResolution == RealLast())
        {
            myVResolution = Kerf.Engine.Foundation.Precision.Parametric(1.0);
        }
        myToEnter = true;
        myEntry   = null;
    }

    /// <summary>
    /// Performs the work on the loaded data: splits the face by the grid and builds the faces. Returns true if done
    /// (status DONE1; FAIL1 misoriented wire handled, FAIL2 recoverable parity error, FAIL3 edge without pcurve, FAIL4
    /// unrecoverable parity error, FAIL6 no wire).
    /// </summary>
    public virtual bool Perform()
    {
        enterWork();
        myStatus           = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        myInvertEdgeStatus = false;

        var seqw = new NCollection_Sequence<ShapeFix_WireSegment>(); // working data: wire segments

        // Init seqw by initial set of wires (with corresponding orientation)
        LoadWires(seqw);
        if (seqw.Length() == 0)
        {
            myStatus = ShapeExtend.EncodeStatus(ShapeExtend_FAIL6);
            return false;
        }

        // Split edges in the wires by grid and add internal segments of grid (parts of cutting lines)
        SplitByGrid(seqw);

        // Split all the wires into segments by common vertices (intersections)
        BreakWires(seqw);

        // Then, collect resulting wires
        var wires = new NCollection_Sequence<ShapeFix_WireSegment>(); // resulting wires
        CollectWires(wires, seqw);

        // And construct resulting faces
        var faces = new NCollection_Sequence<Shape>();
        DispatchWires(faces, wires);

        // Finally, construct resulting shell
        if (faces.Length() != 1)
        {
            var S = new Shell();
            var B = new BrepBuilder();
            B.MakeShell(ref S);
            for (int i = 1; i <= faces.Length(); i++)
            {
                B.Add(S, faces[i]);
            }
            myResult = S;
        }
        else
        {
            myResult = faces[1];
        }
        myResult.Orientation(myOrient);

        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        return true;
    }

    /// <summary>
    /// Splits the edges of the face by the grid (a part of <see cref="Perform"/> producing no shape: the splits are
    /// recorded in the context, an edge replaced by a wire whose edges follow its FORWARD orientation).
    /// </summary>
    public void SplitEdges()
    {
        enterWork();
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);

        var seqw = new NCollection_Sequence<ShapeFix_WireSegment>(); // working data: wire segments

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
        Shape aResult = InternalFinishedResult();
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary>The resulting shell or face, unfrozen, for ported callers (ShapeFix_Face).</summary>
    internal Shape InternalResult() => myResult;

    /// <summary>
    /// <see cref="Result"/> without freezing it, for ported callers that may give a frozen face (ShapeUpgrade's face
    /// dividers): where the face given was copied at the entry, the copy's unchanged sub-shapes are given back and the
    /// context re-keyed, as <see cref="Result"/> does; else the result is the one built in place.
    /// </summary>
    internal Shape InternalFinishedResult()
    {
        myToEnter = true;
        CopyAtEntry? anEntry = myEntry;
        myEntry = null;
        if (anEntry is not null)
        {
            myResult = anEntry.Finish(myResult, [myFace]);
            myFace = ShapeCasts.Face(anEntry.Final(myFace));
        }
        return myResult;
    }

    /// <summary>Queries the status of the last <see cref="Perform"/>.</summary>
    public bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);

    // OCCT.NET: the copy at the entry of a frozen face (decision 583).
    private void enterWork()
    {
        if (!myToEnter)
        {
            return;
        }
        myToEnter = false;
        myEntry   = null;
        if (myFace.IsNull || !BrepTools.HoldsFrozen(myFace))
        {
            return;
        }
        myEntry = new CopyAtEntry(myFace, Context());
        myFace  = ShapeCasts.Face(myEntry.Shape());
    }

    //=======================================================================
    // PRIVATE (working) METHODS
    //=======================================================================

    private const double TOLINT = 1.0e-10; // precision for intersection

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
        Direction2D dir = line.Direction();
        var n = new Direction2D(-dir.Y(), dir.X());
        return n.XY() * (p.XY() - line.Location().XY());
    }

    // Define position of point relative to line
    private static int PointLinePosition(in Point2D p, in Line2D line, out double dev)
    {
        dev = PointLineDeviation(p, line);
        return (dev > TOLINT ? IOR_LEFT : (dev < -TOLINT ? IOR_RIGHT : IOR_UNDEF));
    }

    // Define position of point relative to line
    private static int PointLinePosition(in Point2D p, in Line2D line)
    {
        return PointLinePosition(p, line, out _);
    }

    // Compute parameter of point on line
    private static double ParamPointOnLine(in Point2D p, in Line2D line)
    {
        return line.Direction().XY() * (p.XY() - line.Location().XY());
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
        return new Point2D(line.Location().XY() + line.Direction().XY() * ParamPointOnLine(p, line));
    }

    // Apply context to one edge in the wire and put result into this wire
    private static int ApplyContext(in ShapeFix_WireSegment wire, int iedge, ShapeBuild_ReShape context)
    {
        Edge edge = wire.Edge(iedge);
        Shape res = context.InternalApply(edge);

        if (res.IsSame(edge))
        {
            return 1;
        }

        if (res.ShapeType() == ShapeType.Edge)
        {
            wire.SetEdge(iedge, ShapeCasts.Edge(res));
            return 1;
        }

        int index = iedge;

        var segw = new ShapeExtend_WireData();
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
        if (segw.NbEdges() > 0)
        {
            int ind;
            wire.GetPatchIndex(iedge, out int iumin, out int iumax, out int ivmin, out int ivmax);
            int nbEdges = segw.NbEdges();
            for (int i = 1; i <= nbEdges; i++, index++)
            {
                ind = (edge.Orientation() == Forward || edge.Orientation() == Internal
                         ? i
                         : segw.NbEdges() - i + 1);
                Edge aE = segw.Edge(ind);
                if (i == 1)
                {
                    wire.SetEdge(index, aE);
                }
                else
                {
                    wire.AddEdge(index, aE, iumin, iumax, ivmin, ivmax);
                }
            }
        }

        return index - iedge;
    }

    // check points coincidence
    private static bool IsCoincided(in Point2D p1, in Point2D p2, double UResolution, double VResolution, double tol)
    {
        // pdn Maximal accuracy is working precision of intersector.
        double UTolerance = UResolution * tol;
        double VTolerance = VResolution * tol;
        return Math.Abs(p1.X() - p2.X()) <= Max(TOLINT, UTolerance) && Math.Abs(p1.Y() - p2.Y()) <= Max(TOLINT, VTolerance);
    }

    // computes index for the patch by given parameter Param
    private static int GetPatchIndex(double Param, NCollection_HArray1<double> Params, bool isClosed)
    {
        int NP = Params.Upper();
        double period = Params.Value(NP) - Params.Value(1);
        double shift = 0;
        if (isClosed)
        {
            shift = ShapeAnalysis.AdjustToPeriod(Param, Params.Value(1), Params.Value(NP));
        }
        double p = Param + shift;

        // locate patch: the same algo as in SE_CS::LocateParameter()
        int i; // svv #1
        for (i = 2; i < NP; i++)
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
        return i - ishift * (NP - 1);
    }

    /// <summary>
    /// Fills the sequence of wire segments with the wires of the face (each reordered), applying the context first.
    /// </summary>
    protected void LoadWires(NCollection_Sequence<ShapeFix_WireSegment> seqw)
    {
        seqw.Clear();

        // Init seqw by initial set of wires (with corresponding orientation)
        for (var iw = new Iterator(myFace, false); iw.More(); iw.Next())
        {
            Shape tmpW = Context()!.InternalApply(iw.Value());
            if (tmpW.ShapeType() != ShapeType.Wire)
            {
                if (tmpW.ShapeType() == ShapeType.Vertex)
                {
                    var seg = new ShapeFix_WireSegment(); //(( isOuter ? TopAbs_REVERSED : TopAbs_FORWARD ) );
                    seg.SetVertex(ShapeCasts.Vertex(tmpW));
                    seg.Orientation(tmpW.Orientation());
                    seqw.Append(seg);
                }

                continue;
            }

            Wire wire = ShapeCasts.Wire(tmpW);

            bool isNonManifold = (wire.Orientation() != Reversed && wire.Orientation() != Forward);

            // protect against INTERNAL/EXTERNAL wires
            //    if ( wire.Orientation() != TopAbs_REVERSED &&
            //	 wire.Orientation() != TopAbs_FORWARD ) continue;

            // determine orientation of the wire
            //    TopoDS_Face face = TopoDS::Face ( myFace.EmptyCopied() );
            //    B.Add ( face, wire );
            //    bool isOuter = ShapeAnalysis::IsOuterBound ( face );

            if (isNonManifold)
            {
                var sbwd = new ShapeExtend_WireData(wire, true, false);
                // pdn protection against wires w/o edges
                int nbEdges = sbwd.NbEdges();
                if (nbEdges != 0)
                {
                    // wire segments for non-manifold topology should have INTERNAL orientation
                    var seg = new ShapeFix_WireSegment(sbwd, Internal);
                    seqw.Append(seg);
                }
            }
            else
            {
                // splitting wires containing manifold and non-manifold parts on a separate
                // wire segment
                var sbwdM = new ShapeExtend_WireData();
                var sbwdNM = new ShapeExtend_WireData();
                sbwdNM.ManifoldMode() = false;
                var aIt = new Iterator(wire);
                for (; aIt.More(); aIt.Next())
                {
                    Edge E = ShapeCasts.Edge(aIt.Value());
                    if (E.Orientation() == Forward || E.Orientation() == Reversed)
                    {
                        sbwdM.Add(E);
                    }
                    else
                    {
                        sbwdNM.Add(E);
                    }
                }

                int nbMEdges  = sbwdM.NbEdges();
                int nbNMEdges = sbwdNM.NbEdges();

                if (nbNMEdges != 0)
                {
                    var seg = new ShapeFix_WireSegment(sbwdNM, Internal); //(( isOuter ? TopAbs_REVERSED : TopAbs_FORWARD ) );
                    seqw.Append(seg);
                }

                if (nbMEdges != 0)
                {
                    // Orientation is set so as to allow the segment to be traversed in only one direction
                    // skl 01.04.2002
                    var sfw = new ShapeFix_Wire();
                    sfw.Load(sbwdM);
                    int stat = 0;
                    Surface gs = Tool.Surface(myFace)!;
                    if (gs.IsUPeriodic && gs.IsVPeriodic)
                    {
                        // For torus-like shapes, first reorder in 2d since reorder is indifferent in 3d
                        var sawo = new ShapeAnalysis_WireOrder(false, 0);
                        var sae = new ShapeAnalysis_Edge();
                        for (int i = 1; i <= nbMEdges; i++)
                        {
                            // smh#8
                            Shape tmpF = myFace.Oriented(Forward);
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
                    if (sfw.StatusReorder(ShapeExtend_DONE3))
                    {
                        stat = -1;
                    }

                    if (stat < 0)
                    {
                        var B = new BrepBuilder();
                        Shape dummy = myFace.EmptyCopied();
                        Face face = ShapeCasts.Face(dummy);
                        B.Add(face, wire);
                        bool isOuter = ShapeAnalysis.IsOuterBound(face);
                        Wire w = sbwdM.InternalWire();
                        dummy = myFace.EmptyCopied();
                        face = ShapeCasts.Face(dummy);
                        B.Add(face, w);
                        bool isOuterAfter = ShapeAnalysis.IsOuterBound(face);
                        if (isOuter != isOuterAfter)
                        {
                            sbwdM.Reverse(face);
                        }
                    }

                    var seg = new ShapeFix_WireSegment(sbwdM, Reversed); //(( isOuter ? TopAbs_REVERSED : TopAbs_FORWARD ) );
                    seqw.Append(seg);
                }
            }
        }
    }

    /// <summary>
    /// Analyzes tangencies and computes the orientation code of the wire segment between two intersections: whether it
    /// is left or right of the cutting line, or tangent to it.
    /// </summary>
    protected int ComputeCode(ShapeExtend_WireData wire, in Line2D line, int begInd, int endInd, double begPar,
                              double endPar, bool isInternal = false)
    {
        int code = IOR_UNDEF;

        var sae = new ShapeAnalysis_Edge();
        const int NPOINTS = 5; // number of points for measuring deviation

        // track special closed case: segment starts at end of edge and ends at its beginning
        int special = (begInd == endInd
                           && (wire.Edge(begInd).Orientation() == Forward
                               || wire.Edge(begInd).Orientation() == Internal)
                                == (begPar > endPar)
                         ? 1
                         : 0);
        if (special == 0 && begInd == endInd && begPar == endPar && (myClosedMode || isInternal))
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

        int nb = wire.NbEdges();

        int i; // svv #1
        for (i = begInd;; i++)
        {
            if (i > nb)
            {
                i = 1;
            }
            Edge edge = wire.Edge(i);

            if (!sae.PCurve(edge, myFace, out Curve2D? c2d, out double f, out double l, false))
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
                continue;
            }
            double tol = LimitTolerance(Tool.Tolerance(edge));
            bool isreversed = (edge.Orientation() == Reversed);

            double par1 = (i == begInd && special >= 0 ? begPar : (isreversed ? l : f));
            double par2 = (i == endInd && special <= 0 ? endPar : (isreversed ? f : l));
            double dpar = (par2 - par1) / (NPOINTS - 1);
            int np = (Math.Abs(dpar) < Kerf.Engine.Foundation.Precision.PConfusion() ? 1 : NPOINTS);
            int j; // svv #1
            for (j = 0; j < np; j++)
            {
                double par = par1 + dpar * j;
                Point2D p2d = c2d!.Value(par);
                if (myClosedMode)
                {
                    if (myUClosed && Math.Abs(line.Direction().X()) < Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        if (begin)
                        {
                            shift = ShapeAnalysis.AdjustByPeriod(p2d.X(), line.Location().X(), myUPeriod);
                        }
                        else if (j == 0)
                        {
                            shift = ShapeAnalysis.AdjustByPeriod(p2d.X() - p2d0.X(), 0.0, myUPeriod);
                        }
                        p2d.SetX(p2d.X() + shift);
                    }
                    if (myVClosed && Math.Abs(line.Direction().Y()) < Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        if (begin)
                        {
                            shift = ShapeAnalysis.AdjustByPeriod(p2d.Y(), line.Location().Y(), myVPeriod);
                        }
                        else if (j == 0)
                        {
                            shift = ShapeAnalysis.AdjustByPeriod(p2d.Y() - p2d0.Y(), 0.0, myVPeriod);
                        }
                        p2d.SetY(p2d.Y() + shift);
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
                if (!IsCoincided(p2d, p2dl, myUResolution, myVResolution, tol))
                {
                    if (!myClosedMode)
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
        if (myClosedMode)
        {
            if (code != IOR_UNDEF && !begin)
            {
                // in closed mode, if segment is of 2*pi length, it is BOTH
                double dev = PointLineDeviation(p2d0, line);
                if (myUClosed && Math.Abs(line.Direction().X()) < Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    if (Math.Abs(Math.Abs(dev) - myUPeriod) < 0.1 * myUPeriod)
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
                if (myVClosed && Math.Abs(line.Direction().Y()) < Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    if (Math.Abs(Math.Abs(dev) - myVPeriod) < 0.1 * myVPeriod)
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
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
        }
        return code;
    }

    // After applying context to (seam) edge, distribute its indices on new edges,
    // according to their parameters on that edge
    private static void DistributeSplitPoints(ShapeExtend_WireData sbwd, in Face myFace, int index, int nsplit,
                                              NCollection_Sequence<int> indexes, NCollection_Sequence<double> values)
    {
        bool isreversed = (nsplit > 0 && sbwd.Edge(index).Orientation() == Reversed);

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
    private static int CheckByCurve3d(in Point3D pos, Curve3D? c3d, double param, in Transform3D T, double tol)
    {
        if (c3d is null)
        {
            return 1;
        }
        Point3D p = c3d.Value(param);
        if (T.Form() != Identity)
        {
            p.Transform(T);
        }
        return pos.SquareDistance(p) <= tol * tol ? 1 : 0;
    }

    private static void DefinePatch(in ShapeFix_WireSegment wire, int code, bool isCutByU, int cutIndex, int number = -1)
    {
        int nb = (number > 0 ? number : wire.NbEdges());
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

    private static double GetGridResolution(NCollection_HArray1<double> SplitValues, int cutIndex)
    {
        int nb = SplitValues.Length();
        double leftLen = (cutIndex > 1 ? SplitValues.Value(cutIndex) - SplitValues.Value(cutIndex - 1)
                                       : SplitValues.Value(nb) - SplitValues.Value(nb - 1));
        double rigthLen = (cutIndex < nb ? SplitValues.Value(cutIndex + 1) - SplitValues.Value(cutIndex)
                                         : SplitValues.Value(2) - SplitValues.Value(1));
        return Min(leftLen, rigthLen) / 3.0;
    }

    /// <summary>
    /// Splits the edges of the wire at the given edge indices and parameters; returns the resulting wire and the
    /// vertices at the splitting points. Two splits too near one another (by the edge's tolerance) make one point; a
    /// split too near an end of the edge isn't made (the end vertex is returned instead).
    /// </summary>
    protected ShapeFix_WireSegment SplitWire(ref ShapeFix_WireSegment wire, NCollection_Sequence<int> indexes,
                                             NCollection_Sequence<double> values, NCollection_Sequence<Shape> vertices,
                                             NCollection_Sequence<int> SegmentCodes, bool isCutByU, int cutIndex)
    {
        var B = new BrepBuilder();
        var result = new ShapeFix_WireSegment();
        var aSurfTool = new ShapeAnalysis_Surface(Tool.Surface(myFace));
        int nbSplits = indexes.Length();
        var sae = new ShapeAnalysis_Edge();
        int start = 1;
        Orientation anWireOrient = wire.Orientation();
        var T = new Transform3D();
        if (!myLoc.IsIdentity)
        {
            T = myLoc.Inverted().Transformation();
        }

        // Processing edge by edge (assuming that split points are sorted along the wire)
        for (int i = 1; i <= wire.NbEdges(); i++)
        {
            // for already split seam edge, redistribute its splitting points
            int nsplit = ApplyContext(wire, i, Context()!);
            if (nsplit != 1)
            {
                DistributeSplitPoints(wire.WireData(), myFace, i, nsplit, indexes, values);
                if (nsplit <= 0)
                {
                    i--;
                    continue;
                }
            }
            Edge edge = wire.Edge(i);

            wire.GetPatchIndex(i, out int iumin, out int iumax, out int ivmin, out int ivmax);

            // Position code for first segment of edge
            int code = SegmentCodes[start > 1 ? start - 1 : SegmentCodes.Length()];

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
            var aNMVertices = new NCollection_Sequence<Shape>();
            var aIt = new Iterator(edge, false);
            for (; aIt.More(); aIt.Next())
            {
                if (aIt.Value().Orientation() != Forward && aIt.Value().Orientation() != Reversed)
                {
                    aNMVertices.Append(aIt.Value());
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
            if (T.Form() != Identity)
            {
                prevVPnt.Transform(T);
                lastVPnt.Transform(T);
            }

            if (!sae.Curve3d(edge, out Curve3D? c3d, out double f3d, out double l3d))
            { // not a crime
                c3d = null;
                f3d = l3d = 0;
            }

            if (!sae.PCurve(edge, myFace, out Curve2D? C2d, out double firstPar, out double lastPar))
            {
                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }
            // finding sequence of non-manifold parameters
            int nbNMVert = aNMVertices.Length();
            var aNMVertParams = new NCollection_Sequence<double>();
            if (nbNMVert != 0)
            {
                var adc = new CurveAdaptor2D(C2d!);
                int n = 1;
                for (; n <= nbNMVert; n++)
                {
                    Point3D apV = Tool.Pnt(ShapeCasts.Vertex(aNMVertices.Value(n)));
                    double apar = firstPar;
                    double adist2 = RealLast();
                    if (c3d is not null)
                    {
                        var asae = new ShapeAnalysis_Curve();
                        adist2 = asae.Project(c3d, apV, Kerf.Engine.Foundation.Precision.Confusion(), out _, out apar);
                        adist2 *= adist2;
                    }
                    else
                    {
                        Point2D aP2d = aSurfTool.ValueOfUV(apV, Kerf.Engine.Foundation.Precision.Confusion());
                        var aExtr = new ExtPC2d(aP2d, adc);
                        if (aExtr.IsDone && aExtr.ExtCount != 0)
                        {
                            adist2 = aExtr.SquareDistance(1);
                            int index = 1;
                            int k = 2;
                            for (; k <= aExtr.ExtCount; k++)
                            {
                                double ad2 = aExtr.SquareDistance(k);
                                if (ad2 < adist2)
                                {
                                    adist2 = ad2;
                                    index = k;
                                }
                            }
                            apar = aExtr.Point(index).Parameter();
                        }
                    }
                    aNMVertParams.Append(apar);
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
            Point2D prevPnt2d = C2d!.Value(prevPar);
            Point2D lastPnt2d = C2d.Value(lastPar);
            Point3D prevPnt = myGrid!.Value(prevPnt2d);
            Point3D lastPnt = myGrid.Value(lastPnt2d);
            bool isPeriodic = C2d.IsPeriodic;
            double aPeriod = (isPeriodic ? C2d.Period() : 0.0);

            // Splitting edge
            int NbEdgesStart = result.NbEdges();
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
                        double aShift = ShapeAnalysis.AdjustByPeriod(currPar, (firstPar + lastPar) * 0.5, aPeriod);
                        currPar += aShift;
                    }
                }

                var currPnt2d = new Point2D();
                var currPnt = new Point3D();

                // Try to adjust current splitting point to previous or end of edge
                bool doCut = true;
                var V = new Vertex();
                if (Math.Abs(currPar - lastPar) < Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    V = lastV;
                    doCut = false;
                }
                else if (Math.Abs(currPar - prevPar) < Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    vertices.Append(prevV);
                    code = SegmentCodes[j]; // classification code - update for next segment
                    continue;               // no splitting at this point, go to next one
                }
                else
                {
                    currPnt2d = C2d.Value(currPar);
                    currPnt = myGrid.Value(currPnt2d);
                    if (currPnt.Distance(lastVPnt) <= lastVTol &&
                        // Tolerance is increased to prevent degenerated cuts in cases where all vertex
                        // tolerance is covered by distance of the edge curve from vertex point.
                        // Doubled to prevent edge being fully covered by its vertices tolerance (invalid edge).
                        CheckByCurve3d(lastVPnt, c3d, f3d + (currPar - firstPar) * (l3d - f3d) / span2d, T,
                                       lastVTol + 2 * Kerf.Engine.Foundation.Precision.Confusion()) != 0
                        && lastPnt.Distance(myGrid.Value(C2d.Value(0.5 * (currPar + lastPar)))) <= lastVTol)
                    {
                        V = lastV;
                        double uRes = myUResolution;
                        double vRes = myVResolution;
                        if (isCutByU)
                        {
                            double gridRes = GetGridResolution(myGrid.InternalUJointValues()!, cutIndex) / lastVTol;
                            uRes = Min(myUResolution, gridRes);
                        }
                        else
                        {
                            double gridRes = GetGridResolution(myGrid.InternalVJointValues()!, cutIndex) / lastVTol;
                            vRes = Min(myVResolution, gridRes);
                        }
                        if (IsCoincided(lastPnt2d, currPnt2d, uRes, vRes, lastVTol)
                            && IsCoincided(lastPnt2d, C2d.Value(0.5 * (currPar + lastPar)), uRes, vRes, lastVTol))
                        {
                            doCut = false;
                        }
                    }
                    else if (currPnt.Distance(prevVPnt) <= prevVTol &&
                             // Tolerance is increased to prevent degenerated cuts in cases where all vertex
                             // tolerance is covered by distance of the edge curve from vertex point.
                             // Doubled to prevent edge being fully covered by its vertices tolerance (invalid
                             // edge).
                             CheckByCurve3d(prevVPnt, c3d, f3d + (currPar - firstPar) * (l3d - f3d) / span2d, T,
                                            prevVTol + 2 * Kerf.Engine.Foundation.Precision.Confusion()) != 0
                             && prevPnt.Distance(myGrid.Value(C2d.Value(0.5 * (currPar + prevPar)))) <= prevVTol)
                    {
                        V = prevV;
                        double uRes = myUResolution;
                        double vRes = myVResolution;
                        if (isCutByU)
                        {
                            double gridRes = GetGridResolution(myGrid.InternalUJointValues()!, cutIndex) / prevVTol;
                            uRes = Min(myUResolution, gridRes);
                        }
                        else
                        {
                            double gridRes = GetGridResolution(myGrid.InternalVJointValues()!, cutIndex) / prevVTol;
                            vRes = Min(myVResolution, gridRes);
                        }
                        if (IsCoincided(prevPnt2d, currPnt2d, uRes, vRes, prevVTol)
                            && IsCoincided(prevPnt2d, C2d.Value(0.5 * (currPar + prevPar)), uRes, vRes, prevVTol))
                        {
                            vertices.Append(prevV);
                            code = SegmentCodes[j]; // classification code - update for next segment
                            continue;               // no splitting at this point, go to next one
                        }
                    }
                    //: abv 28.05.02: OCC320 Sample_2: if maxtol = 1e-7, the vertex tolerance
                    // is actually ignored - protect against new vertex on degenerated edge
                    else if (Tool.Degenerated(edge) && prevV.IsSame(lastV))
                    {
                        V = prevV;
                    }
                }
                // classification code for current segment
                if (j > start)
                {
                    code = SegmentCodes[j > 1 ? j - 1 : SegmentCodes.Length()];
                }

                // if not adjusted, make new vertex
                if (V.IsNull)
                {
                    B.MakeVertex(ref V, currPnt.Transformed(myLoc.Transformation()), tolEdge);
                    vertices.Append(V);
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
                    vertices.Append(V);
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
                        Shape tmpV = fV.Oriented(lastV.Orientation());
                        lV = ShapeCasts.Vertex(tmpV);
                    }
                    else
                    {
                        // smh#8
                        Shape emptyCopied = lastV.EmptyCopied();
                        lV = ShapeCasts.Vertex(emptyCopied);
                        Context()!.Replace(lastV, lV);
                    }
                    if (V.IsSame(lastV))
                    {
                        V = lV;
                    }
                    else if (V.IsSame(prevV))
                    {
                        V = fV;
                    }
                    lastV = lV;
                    prevV = fV;
                }

                // Splitting of the edge
                splitted = true;
                prevV.Orientation(Forward);
                V.Orientation(Reversed);
                var sbe = new ShapeBuild_Edge();
                Edge anInitEdge = edge;
                bool ismanifold = (edge.Orientation() == Forward || edge.Orientation() == Reversed);
                if (!ismanifold)
                {
                    anInitEdge.Orientation(Forward);
                }
                Edge newEdge = sbe.InternalCopyReplaceVertices(anInitEdge, prevV, V);

                // addition internal vertices if they exists on edge
                int n = 1;
                for (; n <= aNMVertParams.Length(); n++)
                {
                    double apar = aNMVertParams.Value(n);
                    Vertex aNMVert = ShapeCasts.Vertex(aNMVertices.Value(n));
                    Vertex atmpV = ShapeCasts.Vertex(Context()!.InternalApply(aNMVert));
                    if (Math.Abs(apar - prevPar) <= Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        Context()!.Replace(atmpV, prevV);
                        aNMVertParams.Remove(n);
                        aNMVertices.Remove(n);
                        n--;
                    }
                    else if (Math.Abs(apar - currPar) <= Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        Context()!.Replace(atmpV, V);
                        aNMVertParams.Remove(n);
                        aNMVertices.Remove(n);
                        n--;
                    }
                    if (apar > prevPar && apar < currPar)
                    {
                        B.Add(newEdge, atmpV);
                        aNMVertParams.Remove(n);
                        aNMVertices.Remove(n);
                        n--;
                    }
                }

                sbe.CopyPCurves(newEdge, anInitEdge);

                ShapeAnalysis_TransferParameters theTransferParamtool = GetTransferParamTool();
                theTransferParamtool.SetMaxTolerance(MaxTolerance());
                theTransferParamtool.Init(anInitEdge, myFace);
                theTransferParamtool.TransferRange(newEdge, prevPar, currPar, true);

                if (!ismanifold)
                {
                    if (code == IOR_UNDEF)
                    { // tangential segment
                        newEdge.Orientation(External);
                    }
                    else
                    {
                        newEdge.Orientation(edge.Orientation());
                    }
                }

                if (!sp && !Tool.Degenerated(newEdge))
                {
                    B.SameRange(newEdge, false);
                }
                // pdn take into account 0 codes (if ext)
                if (code == 0 && wire.Orientation() == External)
                {
                    code = ((isCutByU == (j == 1)) ? 1 : 2);
                }

                result.AddEdge(0, newEdge, iumin, iumax, ivmin, ivmax);
                DefinePatch(result, code, isCutByU, cutIndex);

                // Changing prev parameters
                prevV = V;
                prevVTol = LimitTolerance(Tool.Tolerance(V));
                prevVPnt = Tool.Pnt(V);
                prevPnt = currPnt;
                prevPnt2d = currPnt2d;
            }
            start = stop;

            if (splitted)
            {
                // record replacement in context
                // NOTE: order of edges in the replacing wire corresponds to FORWARD orientation of the edge
                var resWire = new Wire();
                B.MakeWire(ref resWire);
                for (int k = NbEdgesStart; k < result.NbEdges(); k++)
                {
                    if (edge.Orientation() == Forward || edge.Orientation() == Internal)
                    {
                        B.Add(resWire, result.Edge(k + 1));
                    }
                    else
                    {
                        B.Add(resWire, result.Edge(result.NbEdges() - k + NbEdgesStart));
                    }
                }
                Context()!.Replace(edge, resWire);
            }
            else
            {
                if (anWireOrient == Internal && code == 0)
                {
                    var sbe = new ShapeBuild_Edge();
                    if (edge.Orientation() == Internal)
                    {
                        edge.Orientation(Forward);
                    }
                    Edge e1 = sbe.InternalCopy(edge, false);
                    var C2d2 = (Curve2D)C2d.Copy();
                    B.UpdateEdge(e1, C2d, C2d2, myFace, 0.0);
                    e1.Orientation(External);
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
        result.Orientation(anWireOrient);
        return result;
    }

    /// <summary>
    /// Splits the edges of the wire by the cutting line (wires FORWARD or REVERSED are taken as closed), recording the
    /// splits in the context (but for EXTERNAL wires, former cutting lines), and fills the parameters, types and
    /// vertices of the intersection points of the line with the edges.
    /// </summary>
    protected bool SplitByLine(ref ShapeFix_WireSegment wire, in Line2D line, bool isCutByU, int cutIndex,
                               NCollection_Sequence<double> SplitLinePar, NCollection_Sequence<int> SplitLineCode,
                               NCollection_Sequence<Shape> SplitLineVertex)
    {
        var sae = new ShapeAnalysis_Edge();
        // prepare data on cutting line
        var jC2d = new LineCurve2D(line);
        var jGAC = new CurveAdaptor2D(jC2d);

        var IntEdgeInd = new NCollection_Sequence<int>();    // index of intersecting edge
        var IntEdgePar = new NCollection_Sequence<double>(); // parameter of intersection point on edge
        var IntLinePar = new NCollection_Sequence<double>(); // parameter of intersection point on line

        bool isnonmanifold = (wire.Orientation() == Internal);
        // gka correction for non-manifold vertices SAMTECH
        if (wire.IsVertex())
        {
            var aSurfTool = new ShapeAnalysis_Surface(Tool.Surface(myFace));
            Vertex aVert = wire.GetVertex();
            Point3D aP3d = Tool.Pnt(aVert);
            Point2D aP2d = aSurfTool.ValueOfUV(aP3d, Kerf.Engine.Foundation.Precision.Confusion());
            int code0 = PointLinePosition(aP2d, line, out _);
            if (code0 != IOR_UNDEF)
            {
                return false;
            }
            double par = ParamPointOnLine(aP2d, line);
            SplitLinePar.Append(par);
            // splitting codes for non-manifold topology should be tangential
            SplitLineCode.Append(ITP_TANG); // ITP_INTER);
            var aVertNew = new Vertex();
            var aB = new BrepBuilder();
            aB.MakeVertex(ref aVertNew, aP3d, Tool.Tolerance(aVert));
            aVertNew.Orientation(Forward);
            Context()!.Replace(aVert, aVertNew);
            SplitLineVertex.Append(aVertNew);
            wire.SetVertex(aVertNew);
            return true;
        }
        ShapeExtend_WireData sewd = wire.WireData();

        int nbe = sewd.NbEdges();

        //: abv 31.10.01: for closed mode
        int closedDir = 0;
        if (myClosedMode)
        {
            if (myUClosed && Math.Abs(line.Direction().X()) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                closedDir = -1;
            }
            else if (myVClosed && Math.Abs(line.Direction().Y()) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                closedDir = 1;
            }
        }
        double halfPeriod = 0.5 * (closedDir != 0 ? closedDir < 0 ? myUPeriod : myVPeriod : 0.0);

        //============================================
        // make intersections and collect all data on intersection points
        int firstCode = 0, prevCode = 0;
        Point2D firstPos = new Point2D(), prevPos = new Point2D();
        double firstDev = 0.0, prevDev = 0.0;
        for (int iedge = 1; iedge <= nbe; iedge++)
        {
            Edge E = sewd.Edge(iedge);
            bool isreversed = (E.Orientation() == Reversed);

            if (!sae.PCurve(E, myFace, out Curve2D? c2d, out double f, out double l, false))
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
            if (myClosedMode)
            {
                // get bounding box of pcurve
                var sac = new ShapeAnalysis_Curve();
                var box = new Box2d();
                const int aNbPoints = 41;
                sac.FillBndBox(c2d, f, l, aNbPoints, true, ref box);
                box.Get(out double umin, out double vmin, out double umax, out double vmax);

                // compute shifts and adjust points adjust
                if (closedDir < 0)
                {
                    double x = line.Location().X();
                    double shift = ShapeAnalysis.AdjustToPeriod(umin, x - myUPeriod, x);
                    if (shift != 0.0)
                    {
                        c2d = (Curve2D)c2d.Copy();
                        var V = new Vector2D(shift, 0.0);
                        c2d.Translate(V);
                        pppf.SetX(pppf.X() + shift);
                        pppl.SetX(pppl.X() + shift);
                    }
                    double dUmax = umax + shift - x;
                    shiftNext.SetX(dUmax > 0 ? -myUPeriod : myUPeriod);
                    nbIter = (int)(1 + Math.Abs(dUmax) / myUPeriod);
                    shift = ShapeAnalysis.AdjustByPeriod(posf.X(), x, myUPeriod);
                    posf.SetX(posf.X() + shift);
                    shift = ShapeAnalysis.AdjustByPeriod(posl.X(), x, myUPeriod);
                    posl.SetX(posl.X() + shift);
                }
                else if (closedDir > 0)
                {
                    double y = line.Location().Y();
                    double shift = ShapeAnalysis.AdjustToPeriod(vmin, y - myVPeriod, y);
                    if (shift != 0.0)
                    {
                        c2d = (Curve2D)c2d.Copy();
                        var V = new Vector2D(0.0, shift);
                        c2d.Translate(V);
                        pppf.SetY(pppf.Y() + shift);
                        pppl.SetY(pppl.Y() + shift);
                    }
                    double dVmax = vmax + shift - y;
                    shiftNext.SetY(dVmax > 0 ? -myVPeriod : myVPeriod);
                    nbIter = (int)(1 + Math.Abs(dVmax) / myVPeriod);
                    shift = ShapeAnalysis.AdjustByPeriod(posf.Y(), y, myVPeriod);
                    posf.SetY(posf.Y() + shift);
                    shift = ShapeAnalysis.AdjustByPeriod(posl.Y(), y, myVPeriod);
                    posl.SetY(posl.Y() + shift);
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
                    IntLinePar.Append(ParamPointsOnLine(pos, prevPos, line)); // !! - maybe compute exactly ?
                    IntEdgePar.Append(isreversed ? l : f);
                    IntEdgeInd.Append(iedge);
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
                var iDom = new IntRes2d_Domain(new Point2D(pppf), f, TOLINT, new Point2D(pppl), l, TOLINT);
                var iGAC = new CurveAdaptor2D(c2d);

                // intersection
                var Inter = new Geom2dInt_GInter();
                Inter.Perform(jGAC, /*jDom,*/ iGAC, iDom, TOLINT, TOLINT);

                // Fill arrays with new intersection points
                if (Inter.IsDone())
                {
                    int i1;
                    for (i1 = 1; i1 <= Inter.NbPoints(); i1++)
                    {
                        IntRes2d_IntersectionPoint IP = Inter.Point(i1);
                        if (IP.TransitionOfSecond().PositionOnCurve() == IntRes2d_Middle
                            || (code != IOR_UNDEF && prevCode != IOR_UNDEF))
                        {
                            IntLinePar.Append(IP.ParamOnFirst());
                            IntEdgePar.Append(IP.ParamOnSecond());
                        }
                    }
                    for (i1 = 1; i1 <= Inter.NbSegments(); i1++)
                    {
                        IntRes2d_IntersectionSegment IS = Inter.Segment(i1);
                        if (IS.HasFirstPoint())
                        {
                            IntRes2d_IntersectionPoint IP = IS.FirstPoint();
                            IntLinePar.Append(IP.ParamOnFirst());
                            IntEdgePar.Append(IP.ParamOnSecond());
                        }
                        if (IS.HasLastPoint())
                        {
                            IntRes2d_IntersectionPoint IP = IS.LastPoint();
                            IntLinePar.Append(IP.ParamOnFirst());
                            IntEdgePar.Append(IP.ParamOnSecond());
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

            int start = IntEdgeInd.Length() + 1; // first of the new points

            // Move all points into range [f,l] (intersector sometimes gives params out of range)
            int i;
            for (i = start; i <= IntEdgePar.Length(); i++)
            {
                if (IntEdgePar[i] < f)
                {
                    IntEdgePar.SetValue(i, f);
                }
                else if (IntEdgePar[i] > l)
                {
                    IntEdgePar.SetValue(i, l);
                }
            }

            // Sort by parameter on edge
            for (i = IntEdgePar.Length(); i > start; i--)
            {
                for (int j = start; j < i; j++)
                {
                    if (isreversed == (IntEdgePar[j + 1] < IntEdgePar[j]))
                    {
                        continue;
                    }
                    IntLinePar.Exchange(j, j + 1);
                    IntEdgePar.Exchange(j, j + 1);
                }
            }

            // and fill indices
            for (i = start; i <= IntEdgePar.Length(); i++)
            {
                IntEdgeInd.Append(iedge);
            }

            // Detect intersection at closing point
            // Only wires which are not EXTERNAL are considered (as closed)
            if (iedge == nbe && wire.Orientation() != External && wire.Orientation() != Internal
                && (prevCode == IOR_UNDEF || prevCode != firstCode))
            {
                if (closedDir == 0 || Math.Abs(firstDev - prevDev) < halfPeriod)
                {
                    IntLinePar.Append(ParamPointsOnLine(pos, firstPos, line));
                    IntEdgePar.Append(isreversed ? f : l);
                    IntEdgeInd.Append(iedge);
                }
            }
        }

        if (IntEdgePar.Length() < 1)
        {
            // pdn Defining position of wire. There is no intersection, so by any point.
            // DefinePatchForWire ( wire, firstCode, isCutByU, cutIndex );
            return false; // pdn ??
        }

        //======================================
        // Fill sequence of transition codes for intersection points
        var IntCode = new NCollection_Sequence<int>();      // parameter of intersection point on line
        var SegmentCodes = new NCollection_Sequence<int>(); // classification codes for segments of wire

        // remove duplicated points to ensure correct results of ComputeCode
        int ii, jj = IntEdgePar.Length();
        if (myClosedMode && jj > 1)
        {
            for (ii = 1; ii <= IntEdgePar.Length();)
            {
                if (ii == jj)
                {
                    break;
                }
                if (IntEdgeInd[ii] == IntEdgeInd[jj] && Math.Abs(IntEdgePar[ii] - IntEdgePar[jj]) < Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    IntLinePar.Remove(ii);
                    IntEdgePar.Remove(ii);
                    IntEdgeInd.Remove(ii);
                    if (jj > ii)
                    {
                        jj--;
                    }
                    continue;
                }
                else if (nbe == 1 || IntEdgeInd[ii] == (IntEdgeInd[jj] % nbe) + 1)
                {
                    Edge E1 = sewd.Edge(IntEdgeInd[jj]);
                    Edge E2 = sewd.Edge(IntEdgeInd[ii]);
                    Tool.Range(E1, myFace, out double a1, out double b1);
                    Tool.Range(E2, myFace, out double a2, out double b2);
                    if (Math.Abs(IntEdgePar[jj] - (E1.Orientation() == Forward ? b1 : a1)) < Kerf.Engine.Foundation.Precision.PConfusion()
                        && Math.Abs(IntEdgePar[ii] - (E2.Orientation() == Forward ? a2 : b2)) < Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        IntLinePar.Remove(ii);
                        IntEdgePar.Remove(ii);
                        IntEdgeInd.Remove(ii);
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
        var aNewSegCodes = new NCollection_Sequence<int>();
        // Compute segment codes (left side of line, right or tangential)
        for (ii = 1; ii <= IntEdgePar.Length(); ii++)
        {
            jj = (ii < IntEdgePar.Length() ? ii + 1 : 1);
            int code = ComputeCode(sewd, line, IntEdgeInd[ii], IntEdgeInd[jj], IntEdgePar[ii], IntEdgePar[jj], isnonmanifold);
            SegmentCodes.Append(code);
        }

        // for EXTERNAL wire, i.e. another joint line, every point is double intersection
        if (wire.Orientation() == External)
        {
            for (ii = 1; ii <= IntEdgePar.Length(); ii++)
            {
                IntCode.Append(ITP_TANG | IOR_BOTH);
                aNewSegCodes.Append(SegmentCodes[ii]);
            }
        }
        // For real (closed) wire, analyze tangencies
        else
        {
            if (wire.Orientation() != Internal)
            {
                // Two consecutive tangential segments are considered as one, merge them.
                for (ii = 1; ii <= IntEdgePar.Length(); ii++)
                {
                    jj = (ii > 1 ? ii - 1 : IntEdgePar.Length());

                    int k = (ii < IntEdgePar.Length() ? ii + 1 : 1); // [ACIS22539]

                    if (SegmentCodes[jj] == IOR_UNDEF && SegmentCodes[ii] == IOR_UNDEF)
                    {
                        // Very specific case when the constructed seam edge
                        // overlaps with spur edge [ACIS22539]
                        if (myClosedMode && (IntLinePar[ii] - IntLinePar[jj]) * (IntLinePar[k] - IntLinePar[ii]) <= 0.0)
                        {
                            continue;
                        }

                        IntEdgeInd.Remove(ii);
                        IntEdgePar.Remove(ii);
                        IntLinePar.Remove(ii);
                        SegmentCodes.Remove(ii);
                        ii--;
                    }
                }
            }
            // pdn exit if all split points removed
            if (IntEdgePar.Length() < 1)
            {
                return false; // pdn ??
            }

            // Analyze type of intersection point and encode it
            // Three kinds of points (ITP): clear intersection, tangency in-point,
            // beginning and end of tangential segment.
            // Orientation (IOR) tells on which side of line edge crosses it
            jj = IntEdgePar.Length();

            for (ii = 1; ii <= IntEdgePar.Length(); jj = ii++)
            {
                int codej = SegmentCodes[jj];
                int codei = SegmentCodes[ii];
                if (myClosedMode)
                {
                    if ((codej & IOR_BOTH) == IOR_BOTH)
                    { // IOR_LEFT : IOR_RIGHT
                        codej = ((codej & IOR_POS) != 0 ? IOR_RIGHT : IOR_LEFT);
                    }
                    if ((codei & IOR_BOTH) == IOR_BOTH)
                    { // IOR_RIGHT : IOR_LEFT
                        codei = ((codei & IOR_POS) != 0 ? IOR_LEFT : IOR_RIGHT);
                    }
                    aNewSegCodes.Append(codei);
                    if (IntEdgeInd[ii] == IntEdgeInd[jj])
                    {
                        aNewSegCodes.Append(codej);
                    }
                }
                else
                {
                    aNewSegCodes.Append(codei);
                }
                int ipcode = (codej | codei);
                if (codej == IOR_UNDEF)
                { // previous segment was tangency
                    if (IntLinePar[ii] > IntLinePar[jj])
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
                    if (IntLinePar[ii < IntLinePar.Length() ? ii + 1 : 1] > IntLinePar[ii])
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
                IntCode.Append(ipcode);
            }
        }

        //=======================================
        // Split edges in the wire by intersection points and fill vertices array
        var IntVertices = new NCollection_Sequence<Shape>();
        wire = SplitWire(ref wire, IntEdgeInd, IntEdgePar, IntVertices, aNewSegCodes, isCutByU, cutIndex);

        // add all data to input arrays
        for (ii = 1; ii <= IntLinePar.Length(); ii++)
        {
            SplitLinePar.Append(IntLinePar[ii]);
            SplitLineCode.Append(IntCode[ii]);
            SplitLineVertex.Append(IntVertices[ii]);
        }

        return true;
    }

    /// <summary>
    /// Splits the edges of the sequence of wires by the cutting line; the parts of the line inside the face (by parity
    /// of the intersections with all wires) are added to the sequence as EXTERNAL one-edge segments without 3d curve,
    /// sharing vertices with the wires they cross.
    /// </summary>
    protected void SplitByLine(NCollection_Sequence<ShapeFix_WireSegment> wires, in Line2D line, bool isCutByU,
                               int cutIndex)
    {
        var SplitLinePar = new NCollection_Sequence<double>();
        var SplitLineCode = new NCollection_Sequence<int>();
        var SplitLineVertex = new NCollection_Sequence<Shape>();

        // split wires one by one, collecting data on intersection points
        int i; // svv #1
        for (i = 1; i <= wires.Length(); i++)
        {
            SplitByLine(ref wires.ChangeValue(i), line, isCutByU, cutIndex, SplitLinePar, SplitLineCode, SplitLineVertex);
        }

        // sort intersection points along parameter on cutting line
        for (i = SplitLinePar.Length(); i > 1; i--)
        {
            for (int j = 1; j < i; j++)
            {
                if (SplitLinePar[j] > SplitLinePar[j + 1])
                {
                    SplitLinePar.Exchange(j, j + 1);
                    SplitLineCode.Exchange(j, j + 1);
                    SplitLineVertex.Exchange(j, j + 1);
                }
            }
        }

        // merge null-length tangential segments into one-point tangencies or intersections
        for (i = 1; i < SplitLinePar.Length(); i++)
        {
            if (Math.Abs(SplitLinePar[i + 1] - SplitLinePar[i]) > Kerf.Engine.Foundation.Precision.PConfusion()
                && !SplitLineVertex[i].IsSame(SplitLineVertex[i + 1]))
            {
                continue;
            }
            if (((SplitLineCode[i] & ITP_ENDSEG) != 0 && (SplitLineCode[i + 1] & ITP_BEGSEG) != 0)
                || ((SplitLineCode[i] & ITP_BEGSEG) != 0 && (SplitLineCode[i + 1] & ITP_ENDSEG) != 0))
            {
                int code = (SplitLineCode[i] | SplitLineCode[i + 1]) & IOR_BOTH;
                SplitLineCode.SetValue(i, code | (code == IOR_BOTH ? ITP_INTER : ITP_TANG));
                SplitLinePar.Remove(i + 1);
                SplitLineCode.Remove(i + 1);
                SplitLineVertex.Remove(i + 1);
            }
        }

        // go along line, split it by intersection points and create edges
        // (only for internal parts, in particular not for tangential segments)
        var B = new BrepBuilder();
        int parity = 0;     // 0 - out, 1 - in
        int halfparity = 0; // left/right for tangential segments
        int tanglevel = 0;  // tangency nesting level
        for (i = 1; i <= SplitLinePar.Length(); i++)
        {
            int code = SplitLineCode[i];
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
            Shape tmpV1 = Context()!.InternalApply(SplitLineVertex[i - 1]);
            Shape tmpV2 = Context()!.InternalApply(SplitLineVertex[i]);
            Vertex V1 = ShapeCasts.Vertex(tmpV1);
            Vertex V2 = ShapeCasts.Vertex(tmpV2);
            // protection against creating null-length edges or edges lying inside tolerance of vertices
            // first and last vertices for split line can not be merged to each other
            bool canbeMerged = (/*myClosedMode &&*/ (i - 1 > 1 || i < SplitLinePar.Length()));
            double aMaxTol = MaxTolerance();
            // case when max tolerance is not defined tolerance of vertices will be used as is
            if (aMaxTol <= 2.0 * Kerf.Engine.Foundation.Precision.Confusion())
            {
                aMaxTol = Kerf.Engine.Foundation.Precision.Infinite();
            }
            double aTol1 = Min(Tool.Tolerance(V1), aMaxTol);
            double aTol2 = Min(Tool.Tolerance(V2), aMaxTol);
            Point3D aP1 = Tool.Pnt(V1);
            Point3D aP2 = Tool.Pnt(V2);
            double aD = aP1.SquareDistance(aP2);
            if (SplitLinePar[i] - SplitLinePar[i - 1] < Kerf.Engine.Foundation.Precision.PConfusion()
                || (canbeMerged && (aD <= (aTol1 * aTol1) || aD <= (aTol2 * aTol2))))
            { // BRepTools::Compare(V1, V2)) ) {
                if (!V1.IsSame(V2))
                { // merge coincident vertices
                    var sbv = new ShapeBuild_Vertex();
                    Vertex V = sbv.InternalCombineVertex(V1, V2);
                    Context()!.Replace(V1, V.Oriented(V1.Orientation()));
                    Context()!.Replace(V2, V.Oriented(V2.Orientation()));
                    V1 = V2 = V;
                }
                continue;
            }

            // create an edge (without 3d curve), put it in wire segment and add to sequence
            // NOTE: i here is always >1
            var edge = new Edge();
            B.MakeEdge(ref edge);
            V1.Orientation(Forward);
            V2.Orientation(Reversed);
            B.Add(edge, V1);
            B.Add(edge, V2);
            var Lin1 = new LineCurve2D(line);
            var Lin2 = new LineCurve2D(line);
            B.UpdateEdge(edge, Lin1, Lin2, myFace, Kerf.Engine.Foundation.Precision.Confusion());
            B.Range(edge, myFace, SplitLinePar[i - 1], SplitLinePar[i]);

            var sbwd = new ShapeExtend_WireData();
            sbwd.Add(edge);
            var seg = new ShapeFix_WireSegment(sbwd, External);

            // set patch indices
            DefinePatch(seg, IOR_UNDEF, isCutByU, cutIndex);
            if (!isCutByU)
            {
                double shiftU = (myClosedMode && myUClosed
                                   ? ShapeAnalysis.AdjustToPeriod(SplitLinePar[i - 1] - TOLINT, myGrid!.UJointValue(1),
                                                                  myGrid.UJointValue(2))
                                   : 0.0);
                double aPar = SplitLinePar[i - 1] + shiftU;

                seg.DefineIUMin(1, GetPatchIndex(aPar + Kerf.Engine.Foundation.Precision.PConfusion(), myGrid!.InternalUJointValues()!, myUClosed));
                seg.DefineIUMax(1, GetPatchIndex(aPar - Kerf.Engine.Foundation.Precision.PConfusion(), myGrid.InternalUJointValues()!, myUClosed) + 1);
            }
            else
            {
                double shiftV = (myClosedMode && myVClosed
                                   ? ShapeAnalysis.AdjustToPeriod(SplitLinePar[i - 1] - TOLINT, myGrid!.VJointValue(1),
                                                                  myGrid.VJointValue(2))
                                   : 0.0);
                double aPar = SplitLinePar[i - 1] + shiftV;
                seg.DefineIVMin(1, GetPatchIndex(aPar + Kerf.Engine.Foundation.Precision.PConfusion(), myGrid!.InternalVJointValues()!, myVClosed));
                seg.DefineIVMax(1, GetPatchIndex(aPar - Kerf.Engine.Foundation.Precision.PConfusion(), myGrid.InternalVJointValues()!, myVClosed) + 1);
            }
            wires.Append(seg);
        }
        if (parity % 2 != 0)
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL4);
        }

        // Apply context to all wires to perform all recorded replacements/merging
        for (i = 1; i <= wires.Length(); i++)
        {
            for (int j = 1; j <= wires[i].NbEdges();)
            {
                j += ApplyContext(wires[i], j, Context()!);
            }
        }
    }

    /// <summary>
    /// Splits the initial set of (closed) wires by the grid's joint lines; the parts of the joint lines inside the face
    /// are added as EXTERNAL wires sharing vertices with the wires they cross.
    /// </summary>
    protected void SplitByGrid(NCollection_Sequence<ShapeFix_WireSegment> seqw)
    {
        // process splitting by U- anv V-seams (i.e. U=const and V=const curves)
        // closed composite surface is processed as periodic
        BrepTools.UVBounds(myFace, out double Uf, out double Ul, out double Vf, out double Vl);
        myGrid!.Bounds(out double Umin, out double Umax, out double Vmin, out double Vmax);

        // value of precision to define number of patch should be the same as used in the definitin
        // position of point relatively to seam edge (TOLINT)
        double pprec = TOLINT; //::Precision::PConfusion();
        int i = 1;
        if (myClosedMode)
        {
            // for closed mode when only one patch exist and location of the splitting line is coincident
            // with first joint value Therefore in this case it is necessary to move all wire segments in
            // the range of the patch between first and last joint values. Then all wire segments are lie
            // between -period and period in order to have valid split ranges after splitting. Because for
            // closed mode cut index always equal to 1 and parts of segments after splitting always should
            // have index either (0,1) or (1,2).
            for (i = 1; i <= seqw.Length(); i++)
            {
                ref ShapeFix_WireSegment wire = ref seqw.ChangeValue(i);

                Shape atmpF = myFace.EmptyCopied();
                var aB = new BrepBuilder();
                atmpF.Orientation(Forward);
                aB.Add(atmpF, wire.WireData().InternalWire());
                ShapeAnalysis.GetFaceUVBounds(ShapeCasts.Face(atmpF), out double Uf1, out double Ul1, out double Vf1, out double Vl1);

                // for closed mode it is necessary to move wire segment in the interval defined by first and
                // last grid UV values
                double shiftU = (myClosedMode && myUClosed
                                   ? ShapeAnalysis.AdjustToPeriod(Ul1 - pprec, myGrid.UJointValue(1), myGrid.UJointValue(2))
                                   : 0.0);
                double shiftV = (myClosedMode && myVClosed
                                   ? ShapeAnalysis.AdjustToPeriod(Vl1 - pprec, myGrid.VJointValue(1), myGrid.VJointValue(2))
                                   : 0.0);
                Uf1 += shiftU;
                Ul1 += shiftU;
                Vf1 += shiftV;
                Vl1 += shiftV;
                // limit patch indices to be in range of grid (extended for periodic) (0, 2)
                // in same cases for example trj4_pm2-ug-203.stp (entity #8024) wire in 2D space has length
                // greater then period
                int iumin = Max(0, GetPatchIndex(Uf1 + pprec, myGrid.InternalUJointValues()!, myUClosed));
                int iumax = GetPatchIndex(Ul1 - pprec, myGrid.InternalUJointValues()!, myUClosed) + 1;

                for (int j = 1; j <= wire.NbEdges(); j++)
                {
                    wire.DefineIUMin(j, iumin);
                    wire.DefineIUMax(j, iumax);
                }

                int ivmin = Max(0, GetPatchIndex(Vf1 + pprec, myGrid.InternalVJointValues()!, myVClosed));
                int ivmax = GetPatchIndex(Vl1 - pprec, myGrid.InternalVJointValues()!, myVClosed) + 1;

                for (int j = 1; j <= wire.NbEdges(); j++)
                {
                    wire.DefineIVMin(j, ivmin);
                    wire.DefineIVMax(j, ivmax);
                }
            }
        }
        else
        {
            // limit patch indices to be in range of grid (extended for periodic)
            int iumin = GetPatchIndex(Uf + pprec, myGrid.InternalUJointValues()!, myUClosed);
            int iumax = GetPatchIndex(Ul - pprec, myGrid.InternalUJointValues()!, myUClosed) + 1;
            for (i = 1; i <= seqw.Length(); i++)
            {
                ref ShapeFix_WireSegment wire = ref seqw.ChangeValue(i);
                for (int j = 1; j <= wire.NbEdges(); j++)
                {
                    wire.DefineIUMin(j, iumin);
                    wire.DefineIUMax(j, iumax);
                }
            }
            int ivmin = GetPatchIndex(Vf + pprec, myGrid.InternalVJointValues()!, myVClosed);
            int ivmax = GetPatchIndex(Vl - pprec, myGrid.InternalVJointValues()!, myVClosed) + 1;
            for (i = 1; i <= seqw.Length(); i++)
            {
                ref ShapeFix_WireSegment wire = ref seqw.ChangeValue(i);
                for (int j = 1; j <= wire.NbEdges(); j++)
                {
                    wire.DefineIVMin(j, ivmin);
                    wire.DefineIVMax(j, ivmax);
                }
            }
        }
        // split by u lines

        for (i = (myUClosed ? 1 : 2); i <= myGrid.NbUPatches(); i++)
        {
            var pos = new Point2D(myGrid.UJointValue(i), 0.0); // 0. - for infinite ranges: myGrid->VJointValue(1) ;
            var line = new Line2D(pos, new Direction2D(Direction2D.D.Y));
            if (!myClosedMode && myUClosed)
            {
                double period = Umax - Umin;
                double X = pos.X();
                double sh = ShapeAnalysis.AdjustToPeriod(X, Uf, Uf + period);
                for (; X + sh <= Ul + pprec; sh += period)
                {
                    Line2D ln = line.Translated(new Vector2D(sh, 0));
                    int cutIndex = GetPatchIndex(X + sh + pprec, myGrid.InternalUJointValues()!, myUClosed);
                    SplitByLine(seqw, ln, true, cutIndex);
                }
            }
            else
            {
                SplitByLine(seqw, line, true, i);
            }
        }

        // split by v lines
        for (i = (myVClosed ? 1 : 2); i <= myGrid.NbVPatches(); i++)
        {
            var pos = new Point2D(0.0, myGrid.VJointValue(i));
            var line = new Line2D(pos, new Direction2D(Direction2D.D.X));
            if (!myClosedMode && myVClosed)
            {
                double period = Vmax - Vmin;
                double Y = pos.Y();
                double sh = ShapeAnalysis.AdjustToPeriod(Y, Vf, Vf + period);
                for (; Y + sh <= Vl + pprec; sh += period)
                {
                    Line2D ln = line.Translated(new Vector2D(0, sh));
                    int cutIndex = GetPatchIndex(Y + sh + pprec, myGrid.InternalVJointValues()!, myVClosed);
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
    protected void BreakWires(NCollection_Sequence<ShapeFix_WireSegment> seqw)
    {
        // split all the wires by vertices
        var splitVertices = new NCollection_Map<Shape>(ShapeHasher.Instance);
        var sae = new ShapeAnalysis_Edge();

        // first collect splitting vertices
        int i; // svv #1
        for (i = 1; i <= seqw.Length(); i++)
        {
            Orientation ori_wire = seqw[i].Orientation();
            if (ori_wire != External && ori_wire != Internal)
            {
                continue;
            }

            ShapeExtend_WireData sbwd = seqw[i].WireData();
            for (int j = 1; j <= sbwd.NbEdges(); j++)
            {
                Edge edge = sbwd.Edge(j);
                Orientation ori_edge = (ori_wire == External ? ori_wire : edge.Orientation());
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
            ShapeFix_WireSegment wire = seqw[i];
            if (wire.IsVertex())
            {
                continue;
            }
            ShapeExtend_WireData sbwd = wire.WireData();

            // find first vertex for split
            int j; // svv #1
            for (j = 1; j <= sbwd.NbEdges(); j++)
            {
                Vertex V = sae.FirstVertex(sbwd.Edge(j));
                if (splitVertices.Contains(V))
                {
                    break;
                }
            }
            if (j > sbwd.NbEdges())
            {
                continue; // splitting not needed
            }

            // if first split of closed edge is not its start, make permutation
            int shift = 0;
            if (j > 1 && !myClosedMode && wire.IsClosed())
            {
                Vertex V = sae.FirstVertex(sbwd.Edge(1));
                if (!splitVertices.Contains(V))
                {
                    shift = j - 1;
                }
                // wire.SetLast ( j-1 );
            }

            // perform splitting
            int nbnew = 0;
            var newwire = new ShapeFix_WireSegment();
            Orientation curOri = ori;
            for (int ind = 1; ind <= sbwd.NbEdges(); ind++)
            {
                j = 1 + (ind - 1 + shift) % sbwd.NbEdges();
                Edge edge = sbwd.Edge(j);
                Vertex V = sae.FirstVertex(edge);
                if (ind == 1 || splitVertices.Contains(V))
                {
                    if (newwire.NbEdges() != 0)
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
                if (ori == Internal && edge.Orientation() == External)
                {
                    curOri = External;
                    edge.Orientation(Forward);
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
    private static int IsShortSegment(in ShapeFix_WireSegment seg, in Face myFace, Surface myGrid,
                                      in Placement myLoc, double UResolution, double VResolution)
    {
        Vertex Vf = seg.FirstVertex();
        if (!Vf.IsSame(seg.LastVertex()))
        {
            return 0;
        }

        Point3D pnt = Tool.Pnt(Vf);
        double tol = Tool.Tolerance(Vf);
        double tol2 = tol * tol;

        int code = 1;
        var sae = new ShapeAnalysis_Edge();
        ShapeExtend_WireData sbwd = seg.WireData();
        for (int i = 1; i <= sbwd.NbEdges(); i++)
        {
            Edge edge = sbwd.Edge(i);
            if (!Vf.IsSame(sae.LastVertex(edge)))
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
            if (!IsCoincided(endPnt, midPnt, UResolution, VResolution, tol))
            {
                code = -1;
            }

            // check 3d
            Point3D midPnt3d = myGrid.Value(midPnt.X(), midPnt.Y());
            if (!myLoc.IsIdentity)
            {
                midPnt3d.Transform(myLoc.Transformation());
            }
            if (midPnt3d.SquareDistance(pnt) > tol2)
            {
                return 0;
            }
        }
        return code;
    }

    private static bool IsSamePatch(in ShapeFix_WireSegment wire, int NU, int NV, ref int iumin, ref int iumax,
                                    ref int ivmin, ref int ivmax, bool extend = false)
    {
        // get patch indices for current segment
        wire.GetPatchIndex(1, out int jumin, out int jumax, out int jvmin, out int jvmax);

        // shift to the same period
        int du = 0, dv = 0;
        if (jumin - iumin > NU)
        {
            du = -(jumin - iumin) / NU;
        }
        else if (iumin - jumin > NU)
        {
            du = (iumin - jumin) / NU;
        }
        if (jvmin - ivmin > NV)
        {
            dv = -(jvmin - ivmin) / NV;
        }
        else if (ivmin - jvmin > NV)
        {
            dv = (ivmin - jvmin) / NV;
        }
        if (du != 0)
        {
            jumin += du * NU;
            jumax += du * NU;
        }
        if (dv != 0)
        {
            jvmin += dv * NV;
            jvmax += dv * NV;
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
    protected void CollectWires(NCollection_Sequence<ShapeFix_WireSegment> wires,
                                NCollection_Sequence<ShapeFix_WireSegment> seqw)
    {
        var sae = new ShapeAnalysis_Edge();
        int i; // svv #1
        // Collect information on short closed segments
        var shorts = new NCollection_Array1<int>(1, seqw.Length());
        for (i = 1; i <= seqw.Length(); i++)
        {
            if (seqw[i].IsVertex() || seqw[i].Orientation() == Internal)
            {
                wires.Append(seqw[i]);
                seqw.Remove(i);
                i--;
                continue;
            }
            int isshort = IsShortSegment(seqw[i], myFace, myGrid!, myLoc, myUResolution, myVResolution);
            shorts.SetValue(i, isshort);
            if (isshort > 0
                && (seqw[i].Orientation() == External
                    || (seqw[i].NbEdges() == 1 && //: abv 13.05.02: OCC320 - remove if degenerated
                        Tool.Degenerated(seqw[i].Edge(1)))))
            {
                seqw[i].Orientation(Internal);
            }
        }

        ShapeExtend_WireData? sbwd = null;
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
                ShapeFix_WireSegment seg = seqw.Value(i);
                if (seg.IsVertex())
                {
                    continue;
                }
                Orientation anOr = seg.Orientation();
                if (anOr == Internal)
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
                    if (anOr == External)
                    {
                        continue;
                    }
                    if (anOr == Forward)
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
                bool sp = IsSamePatch(seg, myGrid!.NbUPatches(), myGrid.NbVPatches(), ref iumin, ref iumax, ref ivmin, ref ivmax);

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
                ShapeExtend_WireData wire = seg.WireData();
                for (int j = 0; j < 2; j++)
                {
                    if (!endV.IsSame(j != 0 ? seg.LastVertex() : seg.FirstVertex()))
                    {
                        continue;
                    }

                    // check for misorientation only if nothing better is found
                    bool misor = (anOr == (j != 0 ? Reversed : Forward));
                    // if ( misor ) continue; // temporarily, to be improved

                    // returning back by the same edge is lowest priority
                    if (lastEdge.IsSame(wire.Edge(j != 0 ? wire.NbEdges() : 1)))
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
                    for (k = 1; k <= wire.NbEdges(); k++)
                    {
                        Shape tmpE = wire.Edge(wire.NbEdges() - k + 1).Reversed();
                        Edge edge = (j != 0 ? ShapeCasts.Edge(tmpE) : wire.Edge(k));
                        edgeTol = Tool.Tolerance(edge);
                        // if ( sae.GetEndTangent2d ( edge, myFace, false, lPnt, lVec ) ) break;
                        if (sae.GetEndTangent2d(edge, myFace, false, out lPnt, out lVec, 1.0e-3))
                        {
                            break;
                        }
                    }
                    if (k > wire.NbEdges())
                    {
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                    }

                    if (myClosedMode)
                    {
                        if (myUClosed)
                        {
                            shiftu = ShapeAnalysis.AdjustByPeriod(lPnt.X(), endPnt.X(), myUPeriod);
                            lPnt.SetX(lPnt.X() + shiftu);
                        }
                        if (myVClosed)
                        {
                            shiftv = ShapeAnalysis.AdjustByPeriod(lPnt.Y(), endPnt.Y(), myVPeriod);
                            lPnt.SetY(lPnt.Y() + shiftv);
                        }
                    }

                    // short segment is to be taken with highest priority by angle
                    double ang = (shorts[i] > 0 ? M_PI : endTan.Angle(lVec));
                    if (myClosedMode && shorts[i] <= 0 && M_PI - ang < Kerf.Engine.Foundation.Precision.Angular())
                    {
                        ang = 0.0; // abv 21 Mar 00: trj3_s1-md-214.stp #2471: avoid going back
                    }

                    // abv 05 Feb 02: face from Parasolid: use tolerance of edges for check
                    // for coincidence (instead of vertex tolerance) in order
                    // this check to be in agreement with check for position of wire segments
                    // thus avoiding bad effects on overlapping edges
                    double ctol = Max(edgeTol, Tool.Tolerance(endV /*lastEdge*/));
                    bool conn = IsCoincided(endPnt, lPnt, myUResolution, myVResolution, ctol);
                    double dist = endPnt.SquareDistance(lPnt);

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
                    myInvertEdgeStatus = true;
                }
                ShapeFix_WireSegment seg = seqw.Value(index);
                if (sbwd is null)
                {
                    sbwd = new ShapeExtend_WireData();
                }
                else if (samepatch)
                { // extend patch indices
                    IsSamePatch(seg, myGrid!.NbUPatches(), myGrid.NbVPatches(), ref iumin, ref iumax, ref ivmin, ref ivmax, true);
                }

                // for closed mode in case if current segment is seam segment it is necessary to detect
                // crossing seam edge in order to have possibility to take candidate from other patch
                if (myClosedMode)
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
                    var wire = new ShapeExtend_WireData();
                    wire.ManifoldMode() = false;
                    wire.Add(seg.WireData());
                    wire.Reverse(myFace);
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
                sae.GetEndTangent2d(firstEdge, myFace, false, out firstPnt, out firstTan, 1.0e-3);
            }

            // update last edge and vertex (only for not short segments)
            bool doupdate = (index != 0 && (shorts[index] <= 0 || endV.IsNull));
            if (doupdate)
            {
                lastEdge = sbwd.Edge(sbwd.NbEdges());
                endV = sae.LastVertex(lastEdge);
                tol = Tool.Tolerance(endV);
                // BUC60035 2053: iteration on edges is required
                int k; // svv #1
                for (k = sbwd.NbEdges(); k >= 1; k--)
                {
                    // if ( sae.GetEndTangent2d ( sbwd->Edge ( k ), myFace, true, endPnt, endTan ) )
                    if (sae.GetEndTangent2d(sbwd.Edge(k), myFace, true, out endPnt, out endTan, 1.0e-3))
                    {
                        break;
                    }
                }
                if (k < 1)
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                }
                if (myUClosed)
                {
                    endPnt.SetX(endPnt.X() + dsu);
                }
                if (myVClosed)
                {
                    endPnt.SetY(endPnt.Y() + dsv);
                }
            }

            // if closed or no next segment found, add to wires
            canBeClosed = endV.IsSame(firstV);
            if (index == 0
                || (canBeClosed && !lastEdge.IsSame(firstEdge) && // cylinder (seam)
                    IsCoincided(endPnt, firstPnt, myUResolution, myVResolution, 2.0 * tol)))
            {
                if (!endV.IsSame(sae.FirstVertex(firstEdge)))
                {
                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL5);
                }
                var s = new ShapeFix_WireSegment(sbwd, Forward);
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
            if (shorts[i] != 1 || seqw[i].IsVertex() || seqw[i].Orientation() == Internal
                || seqw[i].Orientation() == External)
            {
                continue;
            }

            // find any other wire containing the same vertex
            ShapeExtend_WireData wd = seqw[i].WireData();
            Vertex V = seqw[i].FirstVertex();
            int minj = 0, mink = 0;
            double mindist = 0;
            bool samepatch = false;
            // int iumin, iumax, ivmin, ivmax;
            seqw[i].GetPatchIndex(1, out iumin, out iumax, out ivmin, out ivmax);
            sae.GetEndTangent2d(wd.Edge(1), myFace, false, out Point2D p2d, out Vector2D vec);
            for (int j = 1; j <= wires.Length(); j++)
            {
                // if ( j == i ) continue;
                // occ::handle<ShapeExtend_WireData>
                sbwd = wires[j].WireData();
                for (int k = 1; k <= sbwd.NbEdges(); k++)
                {
                    if (!V.IsSame(sae.FirstVertex(sbwd.Edge(k))))
                    {
                        continue; // pdn I suppose that short segment should be inserted into the SAME vertex.
                    }

                    bool sp = IsSamePatch(wires[j], myGrid!.NbUPatches(), myGrid.NbVPatches(), ref iumin, ref iumax, ref ivmin, ref ivmax);
                    if (samepatch && !sp)
                    {
                        continue;
                    }
                    sae.GetEndTangent2d(sbwd.Edge(k), myFace, false, out Point2D pp, out vec);
                    double dist = pp.SquareDistance(p2d);
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
                var s = new ShapeFix_WireSegment(wd, Forward);
                wires.Append(s);
                continue;
            }

            // and if found, merge
            // occ::handle<ShapeExtend_WireData>
            sbwd = wires[minj].WireData();
            for (int n = 1; n <= wd.NbEdges(); n++)
            {
                sbwd.Add(wd.Edge(n), mink++);
            }

            // wires.Remove ( i );
            // i--;
        }
    }

    private static Point2D GetMiddlePoint(in ShapeFix_WireSegment wire, in Face face)
    {
        if (wire.IsVertex())
        {
            Vertex aV = wire.GetVertex();
            Point3D aP3D = Tool.Pnt(aV);
            Surface? surf = Tool.Surface(face);
            var aSurfTool = new ShapeAnalysis_Surface(surf);
            return aSurfTool.ValueOfUV(aP3D, Kerf.Engine.Foundation.Precision.Confusion());
        }
        var box = new Box2d();
        var sae = new ShapeAnalysis_Edge();
        var sac = new ShapeAnalysis_Curve();
        ShapeExtend_WireData wd = wire.WireData();
        for (int i = 1; i <= wd.NbEdges(); i++)
        {
            Edge E = wd.Edge(i);
            if (sae.PCurve(E, face, out Curve2D? c2d, out double cf, out double cl, false))
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
        box.Get(out double aXmin, out double aYmin, out double aXmax, out double aYmax);
        return new Point2D(0.5 * (aXmax + aXmin), 0.5 * (aYmax + aYmin));
    }

    /// <summary>
    /// Creates the faces on one patch of the grid, dispatching the given loops (wires) into one or several faces by
    /// their mutual position.
    /// </summary>
    protected void MakeFacesOnPatch(NCollection_Sequence<Shape> faces, Surface surf,
                                    NCollection_Sequence<Shape> loops)
    {
        var B = new BrepBuilder();

        // Case of single loop: just add it to face
        if (loops.Length() == 1)
        {
            var newFace = new Face();
            B.MakeFace(ref newFace, surf, myLoc, Kerf.Engine.Foundation.Precision.Confusion());
            Shape aSH = loops.Value(1);
            if (aSH.ShapeType() != ShapeType.Wire)
            {
                return;
            }
            Wire wire = ShapeCasts.Wire(loops.Value(1));

            B.Add(newFace, wire);
            if (myInvertEdgeStatus)
            {
                var sff = new ShapeFix_Face(newFace);
                sff.FixAddNaturalBoundMode() = 0;
                var MapWires = new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
                MapWires.Clear();
                sff.FixOrientation(MapWires);
                newFace = sff.InternalFace();
            }

            faces.Append(newFace);
            return;
        }

        // For several loops, first find roots
        // make pseudo-face,
        var pf = new Face();
        B.MakeFace(ref pf, surf, myLoc, Kerf.Engine.Foundation.Precision.Confusion());
        Surface? atSurf = Tool.Surface(pf);

        var aSurfTool = new ShapeAnalysis_Surface(atSurf);
        var roots = new NCollection_Sequence<Shape>();
        int i; // svv #1
        for (i = 1; i <= loops.Length(); i++)
        {
            Point2D unp;
            Wire wr;
            Shape aShape = loops[i];
            if (aShape.ShapeType() != ShapeType.Wire
                || (aShape.Orientation() != Forward && aShape.Orientation() != Reversed))
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
            while (ed.Orientation() != Forward && ed.Orientation() != Reversed)
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
                Shape aShape2 = loops[j];
                if (aShape2.ShapeType() != ShapeType.Wire
                    || (aShape2.Orientation() != Forward && aShape2.Orientation() != Reversed))
                {
                    continue;
                }
                Wire w1 = ShapeCasts.Wire(aShape2);
                var awtmp = new Wire();
                B.MakeWire(ref awtmp);
                awtmp.Orientation(Forward);
                var aIt = new Iterator(w1);
                int nbe = 0;
                for (; aIt.More(); aIt.Next())
                {
                    if (aIt.Value().Orientation() == Forward || aIt.Value().Orientation() == Reversed)
                    {
                        B.Add(awtmp, aIt.Value());
                        nbe++;
                    }
                }
                if (nbe == 0)
                {
                    continue;
                }
                var fc = new Face();
                B.MakeFace(ref fc, surf, myLoc, Kerf.Engine.Foundation.Precision.Confusion());
                B.Add(fc, awtmp);
                var clas = new BRepTopAdaptor_FClass2d(fc, Kerf.Engine.Foundation.Precision.PConfusion());
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
                        if (edge.Orientation() != Forward && edge.Orientation() != Reversed)
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
            B.MakeFace(ref fc, surf, myLoc, Kerf.Engine.Foundation.Precision.Confusion());
            B.Add(fc, wire);
            var clas = new BRepTopAdaptor_FClass2d(fc, Kerf.Engine.Foundation.Precision.PConfusion());
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
                if (loops[j].ShapeType() == ShapeType.Wire)
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
                else if (loops[j].ShapeType() == ShapeType.Vertex)
                {
                    Vertex aV = ShapeCasts.Vertex(loops[j]);
                    Point3D aP = Tool.Pnt(aV);
                    unp = aSurfTool.ValueOfUV(aP, Kerf.Engine.Foundation.Precision.Confusion());
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
            B.MakeFace(ref newFace, surf, myLoc, Kerf.Engine.Foundation.Precision.Confusion());
            B.Add(newFace, wire);
            for (j = 1; j <= holes.Length(); j++)
            {
                Shape aSh = holes[j];
                if (aSh.ShapeType() == ShapeType.Vertex)
                {
                    Vertex aNewV = ShapeAnalysis_TransferParametersProj.InternalCopyNMVertex(ShapeCasts.Vertex(aSh), newFace, myFace);
                    Context()!.Replace(aSh, aNewV);
                    B.Add(newFace, aNewV);
                }
                else
                {
                    B.Add(newFace, holes[j]);
                }
            }
            faces.Append(newFace);

            // check for lost wires, and if they are, make them roots
            if (i == roots.Length() && loops.Length() > 0)
            {
                for (j = 1; j <= loops.Length(); j++)
                {
                    Shape aSh = loops[j];
                    if (aSh.ShapeType() == ShapeType.Wire
                        && (aSh.Orientation() == Forward || aSh.Orientation() == Reversed))
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
    public void DispatchWires(NCollection_Sequence<Shape> faces, NCollection_Sequence<ShapeFix_WireSegment> wires)
    {
        var B = new BrepBuilder();

        // in closed mode, apply FixShifted to all wires before dispatching them
        if (myClosedMode)
        {
            var sfw = new ShapeFix_Wire();
            sfw.SetFace(myFace);
            sfw.SetPrecision(Precision());

            // pdn: shift pcurves in the seam to make OK shape w/o fixshifted
            int i1;
            for (i1 = 1; i1 <= wires.Length(); i1++)
            {
                if (wires[i1].IsVertex())
                {
                    continue;
                }
                ShapeExtend_WireData sbwd = wires[i1].WireData();

                for (int jL = 1; jL <= sbwd.NbEdges(); jL++)
                {
                    Edge E = sbwd.Edge(jL);
                    if (E.Orientation() == Reversed && Tool.IsClosed(E, myFace))
                    {
                        Curve2D c21 = Tool.CurveOnSurface(E, myFace, out double f1, out double l1)!;
                        Shape dummy = E.Reversed();
                        Curve2D c22 = Tool.CurveOnSurface(ShapeCasts.Edge(dummy), myFace, out double f2, out double l2)!;
                        double dPreci = Kerf.Engine.Foundation.Precision.PConfusion() * Kerf.Engine.Foundation.Precision.PConfusion();
                        Point2D pf1 = c21.Value(f1);
                        Point2D pl1 = c21.Value(l1);
                        Point2D pf2 = c22.Value(f2);
                        Point2D pl2 = c22.Value(l2);
                        if (ReferenceEquals(c21, c22) || pf1.SquareDistance(pf2) < dPreci || pl1.SquareDistance(pl2) < dPreci)
                        {
                            var shift = new Vector2D(0.0, 0.0);
                            if (myUClosed && Math.Abs(pf2.X() - pl2.X()) < Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                shift.SetX(myUPeriod);
                            }
                            if (myVClosed && Math.Abs(pf2.Y() - pl2.Y()) < Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                shift.SetY(myVPeriod);
                            }
                            // OCCT.NET: frozen. OCCT translates the stored pcurve in place.
                            ShapeFix_Face.EditPCurve(E, c22, c => c.Translate(shift));
                        }
                    }
                }
            }

            for (i1 = 1; i1 <= wires.Length(); i1++)
            {
                if (wires[i1].IsVertex())
                {
                    continue;
                }
                ShapeExtend_WireData sbwd = wires[i1].WireData();

                //: abv 30.08.01: torHalf2.sat: if wire contains single degenerated
                // edge, skip that wire
                if (sbwd.NbEdges() <= 0 || (sbwd.NbEdges() == 1 && Tool.Degenerated(sbwd.Edge(1))))
                {
                    wires.Remove(i1--);
                    continue;
                }

                sfw.Load(sbwd);
                sfw.FixShifted();

                // force recomputation of degenerated edges (clear pcurves)
                var sbe1 = new ShapeBuild_Edge();
                for (int jL = 1; jL <= sbwd.NbEdges(); jL++)
                {
                    if (Tool.Degenerated(sbwd.Edge(jL)))
                    {
                        sbe1.RemovePCurve(sbwd.Edge(jL), myFace);
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
            mPnts.Append(GetMiddlePoint(wires[i], myFace));
        }

        // Put each wire on its own surface patch (by reassigning pcurves)
        // and build 3d curve if necessary
        var rs = new ShapeBuild_ReShape();
        var sbe = new ShapeBuild_Edge();
        var sae = new ShapeAnalysis_Edge();
        var sfe = new ShapeFix_Edge();

        myGrid!.Bounds(out double U1, out double U2, out double V1, out double V2);
        for (i = 1; i <= nb; i++)
        {
            Point2D pnt = mPnts[i];
            double ush = 0.0, vsh = 0.0;
            if (myUClosed)
            {
                ush = ShapeAnalysis.AdjustToPeriod(pnt.X(), U1, U2);
                pnt.SetX(pnt.X() + ush);
            }
            if (myVClosed)
            {
                vsh = ShapeAnalysis.AdjustToPeriod(pnt.Y(), V1, V2);
                pnt.SetY(pnt.Y() + vsh);
            }
            mPnts[i] = pnt;
            int indU = myGrid.LocateUParameter(pnt.X());
            int indV = myGrid.LocateVParameter(pnt.Y());

            // compute parametric transformation
            bool needT = myGrid.GlobalToLocalTransformation(indU, indV, out double uFact, out Transform2D T);
            if (ush != 0.0 || vsh != 0.0)
            {
                var Sh = new Transform2D();
                Sh.SetTranslation(new Vector2D(ush, vsh));
                T.Multiply(Sh);
                needT = true;
            }
            if (wires[i].IsVertex())
            {
                continue;
            }
            Surface surf = myGrid.Patch(indU, indV);
            var face = new Face();
            B.MakeFace(ref face, surf, myLoc, Kerf.Engine.Foundation.Precision.Confusion());
            ShapeExtend_WireData sewd = wires[i].WireData();
            for (int j = 1; j <= sewd.NbEdges(); j++)
            {
                // int nsplit = ApplyContext ( sewd, j, context );
                // if ( nsplit <1 ) { j--; continue; }

                Edge edge = sewd.Edge(j);

                // !! Accurately copy pcurves for SEAMS and SEAM-like edges !!

                // if edge is already copied, don`t copy any more
                Edge newEdge;
                Edge anInitEdge = edge;
                bool ismanifold = (edge.Orientation() == Forward || edge.Orientation() == Reversed);
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
                        anInitEdge.Orientation(Forward);
                    }

                    newEdge = sbe.InternalCopy(anInitEdge, false);
                    if (!ismanifold)
                    {
                        newEdge.Orientation(edge.Orientation());
                    }
                    rs.Replace(edge, newEdge);
                    Context()!.Replace(edge, newEdge);
                }

                sbe.ReassignPCurve(newEdge, myFace, face);

                // transform pcurve to parametric space of patch
                if (needT)
                {
                    if (sae.PCurve(newEdge, face, out Curve2D? c2d, out double f, out double l, false))
                    {
                        double newf = f, newl = l;
                        Curve2D c2dnew = sbe.TransformPCurve(c2d!, T, uFact, ref newf, ref newl);
                        if (Tool.IsClosed(newEdge, face))
                        {
                            // smh#8
                            Shape tmpE = newEdge.Reversed();
                            Edge e2 = ShapeCasts.Edge(tmpE);
                            if (sae.PCurve(e2, face, out Curve2D? c2d2, out _, out _, false))
                            {
                                if (newEdge.Orientation() == Forward)
                                {
                                    B.UpdateEdge(newEdge, c2dnew, c2d2!, face, 0.0);
                                }
                                else
                                {
                                    B.UpdateEdge(newEdge, c2d2!, c2dnew, face, 0.0);
                                }
                            }
                            else
                            {
                                B.UpdateEdge(newEdge, c2dnew, face, 0.0);
                            }
                        }
                        else
                        {
                            B.UpdateEdge(newEdge, c2dnew, face, 0.0);
                        }
                        B.Range(newEdge, face, newf, newl);
                        if ((newf != f || newl != l) && !Tool.Degenerated(newEdge))
                        {
                            B.SameRange(newEdge, false);
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
                        B.UpdateEdge(newEdge, c3d, 0.0);
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
                Surface S = myGrid.Patch(mPnts[i]);
                if (Surf is null)
                {
                    Surf = S;
                }
                else if (!ReferenceEquals(S, Surf))
                {
                    continue;
                }
                used[i] = true;
                ShapeFix_WireSegment aSeg = wires[i];
                if (aSeg.IsVertex())
                {
                    Vertex aVert = aSeg.GetVertex();
                    if (aVert.Orientation() == Internal)
                    {
                        loops.Append(wires[i].GetVertex());
                    }
                }
                else
                {
                    ShapeExtend_WireData? aWD = aSeg.WireData();
                    if (aWD is not null)
                    {
                        loops.Append(aWD.InternalWire());
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
    public void SetTransferParamTool(ShapeAnalysis_TransferParameters TransferParam)
    {
        myTransferParamTool = TransferParam;
    }

    /// <summary>Gets the tool transferring parameters from 3d to 2d and back.</summary>
    public ShapeAnalysis_TransferParameters GetTransferParamTool() => myTransferParamTool;

    /// <summary>
    /// Returns (modifiable) the 'closed' mode, which makes the tool take all pcurves on a closed surface modulo the
    /// period: less reliable, but deals with wires closed in 3d and open in 2d (a missing seam). Default false.
    /// </summary>
    public ref bool ClosedMode() => ref myClosedMode;
}
