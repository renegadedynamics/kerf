// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Face.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Face.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Face.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//: l2 abv 10.01.99: USA60022 7289: corrections for reversed face
// gka 11.01.99 file PRO7755.stp #2018: work-around error in BRepLib_MakeFace
//: p4 abv, pdn 23.02.99: PRO9234 #15720: call BRepTools::Update() for faces
//    rln 03.03.99 S4135: transmission of parameter precision to SA_Surface::NbSingularities
//: q5 abv 19.03.99 code improvement
//%14 pdn 15.03.99 adding function for fixing null area wires
//%15 pdn 20.03.99 code improvement
//    abv 09.04.99 S4136: improve tolerance management; remove unused flag Closed
// #4  szv          S4163: optimization
//    smh 31.01.01 BUC60810 : Case of small wire on face in solid
// sln 25.09.2001  checking order of 3d and 2d representation curves
// abv 19.10.2001  FixAddNaturalBound improved and extracted as separate fix
// skl,pdn 14.05.2002  OCC55 (correction precision for small faces)

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Fixes a face: its wires (through <see cref="WireFixer"/>), their orientation, a missing seam on a closed
/// surface, missing natural bounds on a sphere or a torus, small-area wires, loop wires, intersecting wires, the
/// apex of a cone, and the split of a face holding several outer wires.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). Load a face with <see cref="Init(in Face)"/>, or a surface with
/// <see cref="Init(Surface, double, bool)"/> and its wires with <see cref="Add"/>; then call
/// <see cref="Perform"/> or the fixes one by one. The modes (<c>...Mode()</c>) are three-position flags: -1 is the
/// default, 0 off, 1 on. Fixes may change the face into a shell or a compound of faces (<see cref="Result"/>).
/// </para>
/// <para>
/// The fixes edit the face in place, as OCCT's, in a working overlay (Kerf unit 10H4) that takes the writes into frozen
/// shapes (any face a caller holds, or one built from a caller's wires), so the caller's shapes never change. The
/// overlay is this tool's from the first fix to <see cref="Face"/> or <see cref="Result"/>, entered by each fix, or,
/// when the tool runs inside another operation's overlay (ShapeFix_Shell's), that one. <see cref="Face"/> and
/// <see cref="Result"/> give the faces with new TShapes for what changed and the caller's own sub-shapes for the rest,
/// record in the context the sub-shapes changed in place, and freeze what they return. Set the context before the
/// first fix. After a public result, a further fix starts a new overlay.
/// </para>
/// </remarks>
internal partial class FaceFixer : RootFixer
{
    protected SurfaceAnalysis? _surf;
    protected Face            _face;
    protected Shape           _result;
    protected WireFixer          _fixWire;
    protected bool                   _fwd;
    protected int                    _status;

    private int _fixWireMode;
    private int _fixOrientationMode;
    private int _fixAddNaturalBoundMode;
    private int _fixMissingSeamMode;
    private int _fixSmallAreaWireMode;
    private int _removeSmallAreaFaceMode;
    private int _fixLoopWiresMode;
    private int _fixIntersectingWiresMode;
    private int _fixSplitFaceMode;
    private int _autoCorrectPrecisionMode;
    private int _fixPeriodicDegenerated;

    private static bool IsSurfaceUVInfinite(Surface surf)
    {
        surf.ParameterBounds(out double uMin, out double uMax, out double vMin, out double vMax);

        return (Kerf.Engine.Foundation.Precision.IsInfinite(uMin) || Kerf.Engine.Foundation.Precision.IsInfinite(uMax) || Kerf.Engine.Foundation.Precision.IsInfinite(vMin)
                || Kerf.Engine.Foundation.Precision.IsInfinite(vMax));
    }

    private static bool IsSurfaceUVPeriodic(SurfaceAdaptor surf)
    {
        return ((surf.IsUPeriodic && surf.IsVPeriodic) || surf.GetType() == SurfaceKind.Sphere);
    }

    /// <summary>Creates an empty tool.</summary>
    public FaceFixer()
    {
        _face    = new Face();
        _result  = new Shape();
        _fwd     = true;
        _status  = 0;
        _fixWire = new WireFixer();
        ClearModes();
    }

    /// <summary>Creates a tool and loads a face.</summary>
    public FaceFixer(in Face face)
    {
        _face    = new Face();
        _result  = new Shape();
        _fwd     = true;
        _status  = 0;
        _fixWire = new WireFixer();
        ClearModes();
        Init(face);
    }

    /// <summary>Sets all modes to their defaults.</summary>
    public virtual void ClearModes()
    {
        _fixWireMode              = -1;
        _fixOrientationMode       = -1;
        _fixAddNaturalBoundMode   = -1;
        _fixMissingSeamMode       = -1;
        _fixSmallAreaWireMode     = -1;
        _removeSmallAreaFaceMode  = -1;
        _fixIntersectingWiresMode = -1;
        _fixLoopWiresMode         = -1;
        _fixSplitFaceMode         = -1;
        _autoCorrectPrecisionMode = 1;
        _fixPeriodicDegenerated   = -1;
    }

    /// <summary>Sets the message registrator (and the wire tool's).</summary>
    public override void SetMsgRegistrator(BasicMsgRegistrator? msgreg)
    {
        base.SetMsgRegistrator(msgreg);
        _fixWire.SetMsgRegistrator(msgreg);
    }

    /// <summary>Sets the basic precision (and the wire tool's).</summary>
    public override void SetPrecision(double preci)
    {
        base.SetPrecision(preci);
        _fixWire.SetPrecision(preci);
    }

    /// <summary>Sets the minimal tolerance (and the wire tool's).</summary>
    public override void SetMinTolerance(double mintol)
    {
        base.SetMinTolerance(mintol);
        _fixWire.SetMinTolerance(mintol);
    }

    /// <summary>Sets the maximal tolerance (and the wire tool's).</summary>
    public override void SetMaxTolerance(double maxtol)
    {
        base.SetMaxTolerance(maxtol);
        _fixWire.SetMaxTolerance(maxtol);
    }

    /// <summary>
    /// Starts the creation of a face on a surface (with no wire yet): wires are added with <see cref="Add"/>. With
    /// <paramref name="fwd"/> false the face is reversed.
    /// </summary>
    public void Init(Surface surf, double preci, bool fwd = true)
    {
        _status = 0;
        var sas = new SurfaceAnalysis(surf);
        Init(sas, preci, fwd);
    }

    /// <summary>As the surface form, sharing an analyser of the surface.</summary>
    public void Init(SurfaceAnalysis surf, double preci, bool fwd = true)
    {
        _status = 0;
        _surf   = surf;
        SetPrecision(preci);
        var b = new BrepBuilder();
        b.MakeFace(ref _face, _surf.Surface()!, Kerf.Engine.Foundation.Precision.Confusion());
        _shape = _face;
        _fwd   = fwd;
        if (!fwd)
        {
            _face.Orient(Reversed);
        }
        Loaded();
    }

    /// <summary>Loads a face (the precision isn't changed).</summary>
    public void Init(in Face face)
    {
        _status = 0;
        // Check if surface is null. It doesn't make sense to create ShapeAnalysis_Surface in that case.
        Surface? surface = Tool.Surface(face);
        if (surface is not null)
        {
            _surf = new SurfaceAnalysis(surface);
        }
        _fwd   = (face.Orientation != Reversed);
        _face  = face;
        _shape = _face;
        Loaded();
    }

    /// <summary>Returns (modifiable) the mode for fixing the wires (default on).</summary>
    public ref int FixWireMode() => ref _fixWireMode;

    /// <summary>Returns (modifiable) the mode for fixing the orientation of the wires (default on).</summary>
    public ref int FixOrientationMode() => ref _fixOrientationMode;

    /// <summary>Returns (modifiable) the mode for adding the natural bounds of a sphere or a torus (default on).</summary>
    public ref int FixAddNaturalBoundMode() => ref _fixAddNaturalBoundMode;

    /// <summary>Returns (modifiable) the mode for adding a missing seam (default on).</summary>
    public ref int FixMissingSeamMode() => ref _fixMissingSeamMode;

    /// <summary>Returns (modifiable) the mode for removing small-area wires (default off).</summary>
    public ref int FixSmallAreaWireMode() => ref _fixSmallAreaWireMode;

    /// <summary>Returns (modifiable) the mode for removing a face whose wires all have small areas (default off).</summary>
    public ref int RemoveSmallAreaFaceMode() => ref _removeSmallAreaFaceMode;

    /// <summary>Returns (modifiable) the mode for fixing intersecting wires (default on).</summary>
    public ref int FixIntersectingWiresMode() => ref _fixIntersectingWiresMode;

    /// <summary>Returns (modifiable) the mode for splitting loop wires (default on).</summary>
    public ref int FixLoopWiresMode() => ref _fixLoopWiresMode;

    /// <summary>Returns (modifiable) the mode for splitting a face with several outer wires (default on).</summary>
    public ref int FixSplitFaceMode() => ref _fixSplitFaceMode;

    /// <summary>Returns (modifiable) the mode for lowering the precision to half the smallest edge (default on).</summary>
    public ref int AutoCorrectPrecisionMode() => ref _autoCorrectPrecisionMode;

    /// <summary>Returns (modifiable) the mode for adding the degenerated edge at a cone's apex (default on).</summary>
    public ref int FixPeriodicDegeneratedMode() => ref _fixPeriodicDegenerated;

    /// <summary>
    /// Adds a wire to the face being built (it must lie on the face's surface). The face loaded by
    /// <see cref="Init(in Face)"/> is the caller's: the wire is added to it in the working overlay of the fixes, as OCCT
    /// adds it to the caller's face.
    /// </summary>
    public void Add(in Wire wire)
    {
        if (wire.IsNull)
        {
            return;
        }
        using WorkingOverlay.Scope work = EnterWork();
        var b = new BrepBuilder();
        // szv#4:S4163:12Mar99 SGI warns
        Shape fc = _face.Oriented(Forward); //: l2 abv 10 Jan 99: Oriented()
        b.Add(fc, wire);
    }

    /// <summary>
    /// Tries to split a wire (needed if some segments were removed in ShapeFix_Wire::FixSelfIntersection).
    /// </summary>
    private static bool SplitWire(in Face face, in Wire wire, NCollection_Sequence<Shape> aResWires)
    {
        var usedEdges = new HashSet<int>();
        var sewd = new ExtendWireData(wire);
        int i, j, k;
        var sae = new EdgeAnalysis();
        for (i = 1; i <= sewd.EdgeCount; i++)
        {
            if (usedEdges.Contains(i))
            {
                continue;
            }
            Edge e1 = sewd.Edge(i);
            usedEdges.Add(i);
            Vertex V0, V1;
            V0 = sae.FirstVertex(e1);
            V1 = sae.LastVertex(e1);
            var sewd1 = new ExtendWireData();
            sewd1.Add(e1);
            bool isConnectedEdge = true;
            // Set to true when null pcurves prevent the 2D closure verification:
            // the partial wire collected so far is discarded for this start edge.
            bool isAbandonedSplit = false;
            for (j = 2; j <= sewd.EdgeCount && isConnectedEdge; j++)
            {
                var e2 = new Edge();
                for (k = 2; k <= sewd.EdgeCount; k++)
                {
                    if (usedEdges.Contains(k))
                    {
                        continue;
                    }
                    e2 = sewd.Edge(k);
                    if (sae.FirstVertex(e2).IsSame(V1))
                    {
                        sewd1.Add(e2);
                        usedEdges.Add(k);
                        V1 = sae.LastVertex(e2);
                        break;
                    }
                }
                if (k > sewd.EdgeCount)
                {
                    isConnectedEdge = false;
                    break;
                }
                if (V1.IsSame(V0))
                {
                    // check that V0 and V1 are same in 2d too
                    Curve2D? curve1 = Tool.CurveOnSurface(e1, face, out double a1, out double b1);
                    Curve2D? curve2 = Tool.CurveOnSurface(e2, face, out double a2, out double b2);
                    if (curve1 is null || curve2 is null)
                    {
                        isAbandonedSplit = true;
                        break;
                    }
                    if (e1.Orientation == Reversed)
                    {
                        a1 = b1;
                    }
                    if (e2.Orientation == Reversed)
                    {
                        b2 = a2;
                    }
                    curve1.D0(a1, out Point2D v0);
                    curve2.D0(b2, out Point2D v1);
                    var adaptor = new SurfaceAdaptor(Tool.Surface(face)!);
                    double tol = Max(Tool.Tolerance(V0), Tool.Tolerance(V1));
                    double maxResolution = 2 * Max(adaptor.UResolution(tol), adaptor.VResolution(tol));
                    if (v0.SquareDistanceTo(v1) < maxResolution)
                    {
                        // new wire is closed, put it into sequence
                        aResWires.Append(sewd1.InternalWire());
                        break;
                    }
                }
            }
            if (isAbandonedSplit)
            {
                continue;
            }
            if (!isConnectedEdge)
            {
                // create new notclosed wire
                aResWires.Append(sewd1.InternalWire());
            }
            if (usedEdges.Count == sewd.EdgeCount)
            {
                break;
            }
        }

        return true;
    }

    /// <summary>
    /// Performs all the fixes, as the modes allow. Returns true if something was done (see <see cref="Status"/>:
    /// DONE1 wires fixed, DONE2 orientation fixed, DONE3 missing seam added, DONE4 small-area wire removed, DONE5
    /// natural bounds added, DONE6 intersecting wires fixed, DONE7 loop wire split (or two-edge coincident wire
    /// removed), DONE8 face split).
    /// </summary>
    public bool Perform(ProgressRange progress = default)
    {
        using WorkingOverlay.Scope work = EnterWork();
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        _fixWire.SetContext(Context());
        WireFixer theAdvFixWire = _fixWire;

        // Skip if an earlier fix already removed this face or replaced it with a non-face.
        if (Context() is not null)
        {
            Shape applied = Context()!.InternalApply(_face);
            if (applied.IsNull || applied.Type != global::Kerf.Topology.ShapeType.Face)
            {
                _result = applied;
                return false;
            }
        }

        var b = new BrepBuilder();
        Shape initFace = _face;
        // perform first part of fixes on wires
        bool isfixReorder = false;
        bool isReplaced   = false;

        // gka fix in order to avoid lost messages (following OCC21771)
        var mapReorderedWires = new Dictionary<Shape, Shape>(ShapeHasher.Instance);

        double savPreci = Precision();
        if (NeedFix(_fixWireMode))
        {
            theAdvFixWire.SetFace(_face, _surf);

            int usFixLackingMode = theAdvFixWire.FixLackingMode();
            // int usFixNotchedEdgesMode = theAdvFixWire->FixNotchedEdgesMode(); // CR0024983
            int usFixSelfIntersectionMode = theAdvFixWire.FixSelfIntersectionMode();
            theAdvFixWire.FixLackingMode() = 0;
            // theAdvFixWire->FixNotchedEdgesMode() = false; // CR0024983
            theAdvFixWire.FixSelfIntersectionMode() = 0;

            bool         @fixed = false;
            Shape s     = _face;
            if (Context() is not null)
            {
                s = Context()!.InternalApply(_face);
            }
            Shape emptyCopied = s.EmptyCopied();
            Face  tmpFace     = ShapeCasts.Face(emptyCopied);
            tmpFace.Orient(Forward);

            // skl 29.03.2010 (OCC21623)
            if (_autoCorrectPrecisionMode != 0)
            {
                double size     = ShapeFixFixer.LeastEdgeSize(s);
                double newpreci = Min(savPreci, size / 2.0);
                newpreci        = newpreci * 1.00001;
                if (savPreci > newpreci && newpreci > Kerf.Engine.Foundation.Precision.Confusion())
                {
                    SetPrecision(newpreci);
                    theAdvFixWire.SetPrecision(newpreci);
                }
            }

            isfixReorder = false;

            for (var iter = new Iterator(s, false); iter.More(); iter.Next())
            {
                if (iter.Value().Type != ShapeType.Wire)
                {
                    b.Add(tmpFace, iter.Value());
                    continue;
                }
                Wire wire = ShapeCasts.Wire(iter.Value());
                theAdvFixWire.Load(wire);
                if (theAdvFixWire.NbEdges() == 0)
                {
                    if (theAdvFixWire.WireData()!.NonManifoldEdgeCount != 0)
                    {
                        b.Add(tmpFace, wire);
                    }
                    else
                    {
                        @fixed = true;
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5);
                    }
                    continue;
                }
                if (theAdvFixWire.Perform(progress))
                {
                    // fixed = true;
                    isfixReorder = (theAdvFixWire.StatusReorder(ExtendStatus.Done) || isfixReorder);
                    @fixed        = (theAdvFixWire.StatusSmall(ExtendStatus.Done)
                                    || theAdvFixWire.StatusConnected(ExtendStatus.Done)
                                    || theAdvFixWire.StatusEdgeCurves(ExtendStatus.Done)
                                    || theAdvFixWire.StatusNotches(ExtendStatus.Done) || // CR0024983
                                    theAdvFixWire.StatusFixTails(ExtendStatus.Done)
                                    || theAdvFixWire.StatusDegenerated(ExtendStatus.Done)
                                    || theAdvFixWire.StatusClosed(ExtendStatus.Done));
                    Wire w = theAdvFixWire.InternalWire();
                    if (@fixed)
                    {
                        Context()?.Replace(wire, w);
                        if (theAdvFixWire.NbEdges() == 0)
                        {
                            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5);
                            continue;
                        }
                    }
                    else if (!wire.IsSame(w))
                    {
                        mapReorderedWires[wire] = w;
                    }

                    wire = w;
                }
                b.Add(tmpFace, wire);
            }

            theAdvFixWire.FixLackingMode()          = usFixLackingMode;
            theAdvFixWire.FixSelfIntersectionMode() = usFixSelfIntersectionMode;
            if (!_fwd)
            {
                tmpFace.Orient(Reversed);
            }

            if (@fixed)
            {
                Context()?.Replace(s, tmpFace);
                isReplaced = true;
            }
            if (@fixed || isfixReorder)
            {
                _face = tmpFace;
                if (!theAdvFixWire.StatusReorder(ExtendStatus.DONE5))
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                }
            }
        }

        _result = _face;
        Shape savShape = _face; // gka BUG 6555

        // Specific case for conic surfaces
        if (NeedFix(_fixPeriodicDegenerated))
        {
            this.FixPeriodicDegeneratedCore();
        }

        // fix missing seam
        if (NeedFix(_fixMissingSeamMode))
        {
            if (FixMissingSeamCore())
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
            }
        }

        // cycle by all possible faces coming from FixMissingSeam
        // each face is processed as if it was single
        var exp = new Explorer(_result, global::Kerf.Topology.ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            _face = ShapeCasts.Face(exp.Current());
            bool needCheckSplitWire = false;

            // perform second part of fixes on wires
            if (NeedFix(_fixWireMode))
            {
                theAdvFixWire.SetFace(_face, _surf);

                int usFixSmallMode       = theAdvFixWire.FixSmallMode();
                int usFixConnectedMode   = theAdvFixWire.FixConnectedMode();
                int usFixEdgeCurvesMode  = theAdvFixWire.FixEdgeCurvesMode();
                int usFixDegeneratedMode = theAdvFixWire.FixDegeneratedMode();
                theAdvFixWire.FixSmallMode()       = 0;
                theAdvFixWire.FixConnectedMode()   = 0;
                theAdvFixWire.FixEdgeCurvesMode()  = 0;
                theAdvFixWire.FixDegeneratedMode() = 0;

                bool         @fixed = false;
                Shape s     = _face;
                if (Context() is not null)
                {
                    s = Context()!.InternalApply(_face);
                }
                Shape emptyCopied = s.EmptyCopied();
                Face  tmpFace     = ShapeCasts.Face(emptyCopied);
                tmpFace.Orient(Forward);
                for (var iter = new Iterator(s, false); iter.More(); iter.Next())
                {
                    if (iter.Value().Type != ShapeType.Wire)
                    {
                        b.Add(tmpFace, iter.Value());
                        continue;
                    }

                    Wire wire = ShapeCasts.Wire(iter.Value());
                    theAdvFixWire.Load(wire);
                    if (theAdvFixWire.NbEdges() == 0)
                    {
                        if (theAdvFixWire.WireData()!.NonManifoldEdgeCount != 0)
                        {
                            b.Add(tmpFace, wire);
                        }
                        else
                        {
                            @fixed = true;
                            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5);
                        }
                        continue;
                    }
                    if (theAdvFixWire.Perform())
                    {
                        isfixReorder = theAdvFixWire.StatusReorder(ExtendStatus.Done);
                        @fixed        = (theAdvFixWire.StatusLacking(ExtendStatus.Done)
                                        || theAdvFixWire.StatusSelfIntersection(ExtendStatus.Done)
                                        || theAdvFixWire.StatusNotches(ExtendStatus.Done)
                                        || theAdvFixWire.StatusFixTails(ExtendStatus.Done));
                        Wire w = theAdvFixWire.InternalWire();
                        if (@fixed)
                        {
                            Context()?.Replace(wire, w);
                        }
                        else if (!wire.IsSame(w))
                        {
                            mapReorderedWires[wire] = w;
                        }

                        wire = w;
                    }
                    if (theAdvFixWire.StatusRemovedSegment())
                    {
                        needCheckSplitWire = true;
                    }

                    // fix for loop of wire
                    var loopWires = new NCollection_Sequence<Shape>();
                    if (NeedFix(_fixLoopWiresMode) && FixLoopWireCore(loopWires))
                    {
                        if (loopWires.Length() > 1)
                        {
                            SendWarning(wire, new Msg("FixAdvFace.FixLoopWire.MSG0")); // Wire was split on several wires
                        }
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE7);
                        @fixed = true;
                        int k = 1;
                        for (; k <= loopWires.Length(); k++)
                        {
                            b.Add(tmpFace, loopWires.Value(k));
                        }
                    }
                    else
                    {
                        b.Add(tmpFace, wire);
                    }
                }

                theAdvFixWire.FixSmallMode()       = usFixSmallMode;
                theAdvFixWire.FixConnectedMode()   = usFixConnectedMode;
                theAdvFixWire.FixEdgeCurvesMode()  = usFixEdgeCurvesMode;
                theAdvFixWire.FixDegeneratedMode() = usFixDegeneratedMode;

                if (@fixed)
                {
                    if (!_fwd)
                    {
                        tmpFace.Orient(Reversed);
                    }
                    if (!isReplaced && !initFace.IsSame(_result) && Context() is not null)
                    { // gka 06.09.04 BUG 6555
                        Context()!.Replace(initFace, savShape);
                    }
                    Context()?.Replace(s, tmpFace);
                    _face = tmpFace;
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                }
            }

            if (needCheckSplitWire)
            {
                // try to split wire - it is needed if some segments were removed
                // in ShapeFix_Wire::FixSelfIntersection()
                Shape s = _face;
                if (Context() is not null)
                {
                    s = Context()!.InternalApply(_face);
                }
                Shape emptyCopied = s.EmptyCopied();
                Face  tmpFace     = ShapeCasts.Face(emptyCopied);
                tmpFace.Orient(Forward);
                var wires = new NCollection_Sequence<Shape>();
                int nbw = 0;
                for (var iter = new Iterator(s, false); iter.More(); iter.Next())
                {
                    if (iter.Value().Type != ShapeType.Wire)
                    {
                        b.Add(tmpFace, iter.Value());
                        continue;
                    }
                    if (iter.Value().Orientation != Forward && iter.Value().Orientation != Reversed)
                    {
                        b.Add(tmpFace, ShapeCasts.Wire(iter.Value()));
                        continue;
                    }
                    nbw++;
                    Wire wire = ShapeCasts.Wire(iter.Value());
                    SplitWire(tmpFace, wire, wires);
                }
                if (nbw < wires.Length())
                {
                    for (int iw = 1; iw <= wires.Length(); iw++)
                    {
                        b.Add(tmpFace, wires.Value(iw));
                    }
                    Context()?.Replace(s, tmpFace);
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE8);
                    _face = tmpFace;
                }
            }

            // fix intersecting wires
            if (FixWiresTwoCoincEdgesCore())
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE7);
            }
            if (NeedFix(_fixIntersectingWiresMode))
            {
                if (FixIntersectingWiresCore())
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE6);
                }
            }

            // fix orientation
            var mapWires = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            mapWires.Clear();
            if (NeedFix(_fixOrientationMode))
            {
                if (FixOrientationCore(mapWires))
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                }
            }

            BrepTools.Update(_face);

            // fix natural bounds
            bool needSplit = true;
            if (FixAddNaturalBoundCore())
            {
                needSplit = false;
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5);
            }

            // split face
            if (NeedFix(_fixSplitFaceMode) && needSplit && mapWires.Count > 1)
            {
                if (FixSplitFaceCore(mapWires))
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE8);
                }
            }
        }

        // return the original preci
        SetPrecision(savPreci);
        theAdvFixWire.SetPrecision(savPreci);

        // cycle by all possible faces coming from FixAddNaturalBound
        // each face is processed as if it was single
        for (exp.Init(_result, global::Kerf.Topology.ShapeType.Face); exp.More(); exp.Next())
        {
            _face = ShapeCasts.Face(exp.Current());

            // fix small-area wires
            if (NeedFix(_fixSmallAreaWireMode, false))
            {
                bool isRemoveFace = NeedFix(_removeSmallAreaFaceMode, false);
                if (FixSmallAreaWireCore(isRemoveFace))
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                }
            }
        }

        if (Context() is not null)
        {
            if (Status(ExtendStatus.Done) && !isReplaced && !initFace.IsSame(savShape))
            {
                // gka fix in order to avoid lost messages (following OCC21771)
                if (mapReorderedWires.Count != 0)
                {
                    var itW = new Iterator(initFace, false);
                    for (; itW.More(); itW.Next())
                    {
                        Shape curW = itW.Value();
                        while (mapReorderedWires.ContainsKey(curW))
                        {
                            Shape fixW = mapReorderedWires.GetValue(curW);
                            Context()!.Replace(curW, fixW);
                            curW = fixW;
                        }
                    }
                }
                Context()!.Replace(initFace, savShape);
            }
            _result = Context()!.InternalApply(initFace); // gka 06.09.04
        }
        else if (!Status(ExtendStatus.Done))
        {
            _result = initFace;
        }

        return Status(ExtendStatus.Done);
    }

    // Shift all pcurves of edges in the given wire on the given face
    // to vector <vec>
    private static void Shift2dWire(in Wire w, in Face f, Vector2D vec, SurfaceAnalysis surface,
                                    bool recompute3d = false)
    {
        var tr2d = new Transform2D();
        tr2d = tr2d.WithTranslation(new Vector2D(vec.XY()));
        var sae = new EdgeAnalysis();
        var sbe = new BuildEdgeBuilder();
        var b = new BrepBuilder();
        for (var ei = new Iterator(w, false); ei.More(); ei.Next())
        {
            Edge edge = ShapeCasts.Edge(ei.Value());
            if (!sae.PCurve(edge, f, out Curve2D? c2d, out double cf, out double cl, true))
            {
                continue;
            }
            c2d = EditPCurve(edge, c2d!, c => c.Transform(tr2d));
            if (recompute3d)
            {
                // recompute 3d curve and vertex
                sbe.RemoveCurve3d(edge);
                sbe.BuildCurve3d(edge);
                b.UpdateVertex(sae.FirstVertex(edge), surface.Value(c2d.Value(cf)), 0.0);
            }
        }
    }

    /// <summary>
    /// OCCT.NET: frozen. Edits in place the pcurve <paramref name="c2d"/> of the edge, as OCCT does: when it is
    /// frozen (a curve a caller's shape holds), the edit is made on a copy, which is put in its place in the edge's
    /// representations (OCCT edits the stored curve in place, and with it any other edge sharing it). Returns the
    /// curve edited.
    /// </summary>
    internal static Curve2D EditPCurve(in Edge edge, Curve2D c2d, Action<Curve2D> edit)
    {
        if (!c2d.IsFrozen)
        {
            edit(c2d);
            return c2d;
        }
        var copy = (Curve2D)c2d.Copy();
        edit(copy);
        var it = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)edge.TShape()!).ChangeCurves());
        for (; it.More(); it.Next())
        {
            if (it.Value() is not GCurve gc)
            {
                continue;
            }
            if (gc.IsCurveOnSurface() && ReferenceEquals(gc.PCurve(), c2d))
            {
                gc.PCurve(copy);
            }
            if (gc.IsCurveOnClosedSurface && ReferenceEquals(gc.PCurve2(), c2d))
            {
                gc.PCurve2(copy);
            }
        }
        return copy;
    }

    // Cut interval from the sequence of intervals
    private static bool CutInterval(NCollection_Sequence<Point2D> intervals, in Point2D toAddI, double period)
    {
        if (intervals.Length() <= 0)
        {
            return false;
        }
        for (int j = 0; j < 2; j++)
        { // try twice, align to bottom and to top
            for (int i = 1; i <= intervals.Length(); i++)
            {
                Point2D interval = intervals[i];
                // ACIS907, OCC921 a054a.sat (face 124)
                double shift = ShapeAnalysisAnalysis.AdjustByPeriod((j != 0 ? toAddI.X : toAddI.Y),
                                                            0.5 * (interval.X + interval.Y),
                                                            period);
                var toAdd = new Point2D(toAddI.X + shift, toAddI.Y + shift);
                if (toAdd.Y <= interval.X || toAdd.X >= interval.Y)
                {
                    continue;
                }
                if (toAdd.X > interval.X)
                {
                    if (toAdd.Y < interval.Y)
                    {
                        intervals.InsertBefore(i, interval);
                        intervals.ChangeValue(i + 1) = intervals.ChangeValue(i + 1).WithX(toAdd.Y); // i++...
                    }
                    intervals.ChangeValue(i) = intervals.ChangeValue(i).WithY(toAdd.X);
                }
                else if (toAdd.Y < interval.Y)
                {
                    intervals.ChangeValue(i) = intervals.ChangeValue(i).WithX(toAdd.Y);
                }
                else
                {
                    intervals.Remove(i--);
                }
            }
        }
        return true;
    }

    // Find middle of the biggest interval
    private static double FindBestInterval(NCollection_Sequence<Point2D> intervals)
    {
        double shift = 0.0, max = -1.0;
        for (int i = 1; i <= intervals.Length(); i++)
        {
            Point2D interval = intervals[i];
            if (interval.Y - interval.X <= max)
            {
                continue;
            }
            max   = interval.Y - interval.X;
            shift = interval.X + 0.5 * max;
        }
        return shift;
    }

    /// <summary>
    /// Adds the natural boundary of a sphere or a torus to a face that misses it (the face's wires are then holes), or
    /// makes a face without wires naturally bounded. Returns true if done.
    /// </summary>
    public bool FixAddNaturalBound()
    {
        using WorkingOverlay.Scope work = EnterWork();
        return FixAddNaturalBoundCore();
    }

    // Detect missing natural boundary on spherical surfaces and add it if
    // necessary
    // pdn 981202: add natural bounds if missing (on sphere only)
    //: abv 28.08.01: rewritten and extended for toruses
    private bool FixAddNaturalBoundCore()
    {
        if (_surf is null)
        {
            return false;
        }

        if (Context() is not null)
        {
            Shape s = Context()!.InternalApply(_face);
            _face = ShapeCasts.Face(s);
        }

        // collect wires in sequence
        var ws = new NCollection_Sequence<Shape>();
        var vs = new NCollection_Sequence<Shape>();
        var wi = new Iterator(_face, false);
        for (; wi.More(); wi.Next())
        {
            if (wi.Value().Type == ShapeType.Wire
                && (wi.Value().Orientation == Forward || wi.Value().Orientation == Reversed))
            {
                ws.Append(wi.Value());
            }
            else
            {
                vs.Append(wi.Value());
            }
        }

        // deal with the case of an empty face: just create a new face by a standard tool
        if (ws.IsEmpty() && !IsSurfaceUVInfinite(_surf.Surface()!))
        {
            var faceBuilder = new MakeFace(_surf.Surface()!, Kerf.Engine.Foundation.Precision.Confusion());

            Face newFace = faceBuilder.InternalFace();
            newFace.Orient(_face.Orientation);

            Context()?.Replace(_face, newFace);

            // taking into account orientation
            _face = newFace;

            // gka 11.01.99 file PRO7755.stp entity #2018 surface #1895: error BRepLib_MakeFace func
            // IsDegenerated
            EdgeFixer sfe = _fixWire.FixEdgeTool();
            for (var eed = new Explorer(_face, ShapeType.Edge); eed.More(); eed.Next())
            {
                Edge edg = ShapeCasts.Edge(eed.Current());
                sfe.FixVertexTolerance(edg, _face);
            }

            //    B.UpdateFace (myFace,myPrecision);
            SendWarning(_face, new Msg("FixAdvFace.FixOrientation.MSG0")); // Face created with natural bounds
            BrepTools.Update(_face);
            _result = _face;
            return true;
        }

        // check if surface doesn't need natural bounds
        if (!IsNeedAddNaturalBound(ws))
        {
            return false;
        }

        // Collect information on free intervals in U and V
        var intU = new NCollection_Sequence<Point2D>();
        var intV = new NCollection_Sequence<Point2D>();
        var centers = new NCollection_Sequence<Point2D>();
        _surf.Bounds(out double suf, out double sul, out double svf, out double svl);
        intU.Append(new Point2D(suf, sul));
        intV.Append(new Point2D(svf, svl));
        int nb = ws.Length();
        int i;

        for (i = 1; i <= nb; i++)
        {
            //     Bnd_Box2d B;
            Wire aw        = ShapeCasts.Wire(ws.Value(i));
            Face wireFace = ShapeCasts.Face(_face.EmptyCopied());
            var aB = new BrepBuilder();
            aB.Add(wireFace, aw);
            ShapeAnalysisAnalysis.GetFaceUVBounds(wireFace, out double umin, out double umax, out double vmin, out double vmax);

            // PTV 01.11.2002 ACIS907, OCC921 end
            if (_surf.IsUClosed())
            {
                CutInterval(intU, new Point2D(umin, umax), sul - suf);
            }
            if (_surf.IsVClosed())
            {
                CutInterval(intV, new Point2D(vmin, vmax), svl - svf);
            }
            centers.Append(new Point2D(0.5 * (umin + umax), 0.5 * (vmin + vmax)));
        }

        // find best interval and thus compute shift
        var shift = new Point2D(0.0, 0.0);
        if (_surf.IsUClosed())
        {
            shift = shift.WithX(FindBestInterval(intU));
        }
        if (_surf.IsVClosed())
        {
            shift = shift.WithY(FindBestInterval(intV));
        }

        // Adjust all other wires to be inside outer one
        var center = new Point2D(shift.X + 0.5 * (sul - suf), shift.Y + 0.5 * (svl - svf));
        for (i = 1; i <= nb; i++)
        {
            Wire wire = ShapeCasts.Wire(ws.Value(i));
            var sh = new Point2D(0.0, 0.0);
            if (_surf.IsUClosed())
            {
                sh = sh.WithX(ShapeAnalysisAnalysis.AdjustByPeriod(centers[i].X, center.X, sul - suf));
            }
            if (_surf.IsVClosed())
            {
                sh = sh.WithY(ShapeAnalysisAnalysis.AdjustByPeriod(centers[i].Y, center.Y, svl - svf));
            }
            Shift2dWire(wire, _face, new Vector2D(sh.XY()), _surf);
        }

        // Create naturally bounded surface and add that wire to sequence
        Surface surf = Tool.Surface(_face, out Placement l)!;
        var mf = new MakeFace(surf, Kerf.Engine.Foundation.Precision.Confusion());
        Face ftmp = mf.InternalFace();
        ftmp.Location(l);
        for (wi.Initialize(ftmp, false); wi.More(); wi.Next())
        {
            if (wi.Value().Type != ShapeType.Wire)
            {
                continue;
            }
            Wire wire = ShapeCasts.Wire(wi.Value());
            ws.Append(wire);
            if (shift.XY().Modulus() < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                continue;
            }
            Shift2dWire(wire, _face, new Vector2D(shift.XY()), _surf, true);
        }

        // Fix possible case on sphere when gap contains degenerated edge
        // and thus has a common part with natural boundary
        // Such hole should be merged with boundary
        if (_surf.Adaptor3d()!.GetType() == SurfaceKind.Sphere && ws.Length() == nb + 1)
        {
            var bnd = new ExtendWireData(ShapeCasts.Wire(ws.Last()));
            // code to become separate method FixTouchingWires()
            for (i = 1; i <= nb; i++)
            {
                var sbwd = new ExtendWireData(ShapeCasts.Wire(ws.Value(i)));
                for (int j = 1; j <= sbwd.EdgeCount; j++)
                {
                    if (!Tool.Degenerated(sbwd.Edge(j)))
                    {
                        continue;
                    }
                    // find corresponding place in boundary
                    var sae = new EdgeAnalysis();
                    Vertex v = sae.FirstVertex(sbwd.Edge(j));
                    int k;
                    for (k = 1; k <= bnd.EdgeCount; k++)
                    {
                        if (!Tool.Degenerated(bnd.Edge(k)))
                        {
                            continue;
                        }
                        if (BrepTools.Compare(v, sae.FirstVertex(bnd.Edge(k))))
                        {
                            break;
                        }
                    }
                    if (k > bnd.EdgeCount)
                    {
                        continue;
                    }
                    // and insert hole to that place
                    var b2 = new BrepBuilder();
                    b2.Degenerated(sbwd.Edge(j), false);
                    b2.Degenerated(bnd.Edge(k), false);
                    sbwd.SetLast(j);
                    bnd.Add(sbwd, k + 1);
                    ws.Remove(i--);
                    nb--;
                    _fixWire.SetFace(_face, _surf);
                    _fixWire.Load(bnd);
                    _fixWire.FixConnected();
                    _fixWire.FixDegenerated();
                    ws.SetValue(ws.Length(), bnd.InternalWire());
                    break;
                }
            }
        }

        // Create resulting face
        var b = new BrepBuilder();
        Shape s2 = _face.EmptyCopied();
        s2.Orient(Forward);
        for (i = 1; i <= ws.Length(); i++)
        {
            b.Add(s2, ws.Value(i));
        }
        for (i = 1; i <= vs.Length(); i++)
        {
            b.Add(s2, vs.Value(i));
        }
        if (!_fwd)
        {
            s2.Orient(Reversed);
        }
        Context()?.Replace(_face, s2);
        _face = ShapeCasts.Face(s2);
        BrepTools.Update(_face);

        SendWarning(_face, new Msg("FixAdvFace.FixOrientation.MSG0")); // Face created with natural bounds
        return true;
    }

    /// <summary>Fixes the orientation of the wires on the face. Returns true if a wire was reversed.</summary>
    public bool FixOrientation()
    {
        using WorkingOverlay.Scope work = EnterWork();
        var mapWires = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        mapWires.Clear();
        return FixOrientationCore(mapWires);
    }

    private bool IsNeedAddNaturalBound(NCollection_Sequence<Shape> orientedWires)
    {
        // if fix is not needed
        if (!NeedFix(_fixAddNaturalBoundMode))
        {
            return false;
        }
        // if surface is not double-closed
        if (_surf is not null && !IsSurfaceUVPeriodic(_surf.Adaptor3d()!))
        {
            return false;
        }
        // if face has an OUTER bound
        if (ShapeAnalysisAnalysis.IsOuterBound(_face))
        {
            return false;
        }
        // check that not any wire has a seam edge and not any edge is degenerated.
        // because the presence of a seam or degenerated edge indicates that this wire should be an
        // external one, and in case of its incorrect orientation, this will be corrected.
        int nbOriented = orientedWires.Length();
        for (int i = 1; i <= nbOriented; i++)
        {
            Wire wire = ShapeCasts.Wire(orientedWires.Value(i));
            for (var edgeIt = new Iterator(wire); edgeIt.More(); edgeIt.Next())
            {
                Edge edge = ShapeCasts.Edge(edgeIt.Value());
                if (Tool.Degenerated(edge))
                {
                    return false;
                }
                if (Tool.IsClosed(edge, _face))
                {
                    return false;
                }
            }
        }

        return true;
    }

    /// <summary>
    /// Fixes the orientation of the wires on the face, and fills <paramref name="mapWires"/> with the outer wires and
    /// the wires inside each. Returns true if a wire was reversed.
    /// </summary>
    public bool FixOrientation(Dictionary<Shape, NCollection_List<Shape>> mapWires)
    {
        using WorkingOverlay.Scope work = EnterWork();
        return FixOrientationCore(mapWires);
    }

    private bool FixOrientationCore(Dictionary<Shape, NCollection_List<Shape>> mapWires)
    {
        bool done = false;

        if (Context() is not null)
        {
            Shape s = Context()!.InternalApply(_face);
            _face = ShapeCasts.Face(s);
        }
        var ws = new NCollection_Sequence<Shape>();
        var allSubShapes = new NCollection_Sequence<Shape>();
        // smh: BUC60810 : protection against very small wires (one-edge, null-length)
        var verySmallWires = new NCollection_Sequence<Shape>();
        for (var wi = new Iterator(_face, false); wi.More(); wi.Next())
        {
            if (wi.Value().Type == ShapeType.Vertex
                || (wi.Value().Orientation != Forward && wi.Value().Orientation != Reversed))
            {
                allSubShapes.Append(wi.Value());
                // ws.Append (wi.Value());
                continue;
            }

            var ei = new Iterator(wi.Value(), false);
            Edge edge;
            double length = RealLast();
            if (ei.More())
            {
                edge = ShapeCasts.Edge(ei.Value());
                ei.Next();
                if (!ei.More())
                {
                    length = 0;
                    var sae = new EdgeAnalysis();
                    if (sae.Curve3d(edge, out Curve3D? c3d, out double First, out double Last))
                    {
                        Point3D pntIni = c3d!.Value(First);
                        Xyz prev;
                        prev = pntIni.XYZ();
                        int nbControl = 10;
                        for (int j = 1; j < nbControl; j++)
                        {
                            double prm     = ((nbControl - 1 - j) * First + j * Last) / (nbControl - 1);
                            Point3D pntCurr = c3d.Value(prm);
                            Xyz curr    = pntCurr.XYZ();
                            Xyz delta   = curr - prev;
                            length += delta.Modulus();
                            prev = curr;
                        }
                    }
                }
            }
            else
            {
                length = 0;
            }
            if (length > Kerf.Engine.Foundation.Precision.Confusion())
            {
                ws.Append(wi.Value());
                allSubShapes.Append(wi.Value());
            }
            else
            {
                verySmallWires.Append(wi.Value());
            }
        }
        if (verySmallWires.Length() > 0)
        {
            done = true;
        }

        int nb    = ws.Length();
        int nbAll = allSubShapes.Length();
        var b = new BrepBuilder();

        // if no wires, just do nothing
        if (nb <= 0)
        {
            return false;
        }

        bool isAddNaturalBounds = IsNeedAddNaturalBound(ws);
        var seqReversed = new NCollection_Sequence<int>();
        // if wire is only one, check its orientation
        if (nb == 1)
        {
            // skl 12.04.2002 for cases with nbwires>1 (VerySmallWires>1)
            // make face with only one wire (ws.Value(1))
            Shape dummy = _face.EmptyCopied();
            Face af = ShapeCasts.Face(dummy);
            af.Orient(Forward);
            b.Add(af, ws.Value(1));

            if (!isAddNaturalBounds && !ShapeAnalysisAnalysis.IsOuterBound(af))
            {
                var sbdw = new ExtendWireData(ShapeCasts.Wire(ws.Value(1)));
                sbdw.Reverse(_face);
                ws.SetValue(1, sbdw.InternalWire());
                SendWarning(sbdw.InternalWire(), new Msg("FixAdvFace.FixOrientation.MSG5")); // Wire on face was reversed
                done = true;
            }
        }
        // in case of several wires, perform complex analysis
        // TEST CAUTION
        // Multiple wires: relative orientations
        // Each wire must "contain" all the others
        // Obviously, in the case of leopard skin, there may be a problem
        else if (_surf is not null)
        {
            // Take each wire (NB: curves present!)
            // In principle, we should reject unclosed wires (see missing seam?)
            // We classify it relative to the others, which must all be either IN or
            // OUT. Otherwise, there is nesting -> SDB. If IN, OK, if OUT, we reverse
            // (NB: not myClos here, so no stitching problem)
            // If there is at least one inversion, the face must be redone (see myRebil)
            bool uclosed = _surf.IsUClosed();
            bool vclosed = _surf.IsVClosed();
            _surf.Bounds(out double suf, out double sul, out double svf, out double svl);
            double uRange = sul - suf;
            double vRange = svl - svf;

            var mw = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            var si = new Dictionary<Shape, int>(ShapeHasher.Instance);
            var mapIntWires = new HashSet<Shape>(ShapeHasher.Instance);
            mw.Clear();
            si.Clear();
            mapIntWires.Clear();
            int i;

            var wireBoxes = new NCollection_Array1<Box2d>(1, nb);
            double uMiddle = 0, vMiddle = 0;
            bool isFirst = true;
            // create Bounding boxes for each wire
            for (i = 1; i <= nb; i++)
            {
                Shape shape = ws.Value(i);
                Wire wire = ShapeCasts.Wire(shape);
                var box = new Box2d();
                var ew = new Iterator(wire);
                for (; ew.More(); ew.Next())
                {
                    Edge ed = ShapeCasts.Edge(ew.Value());
                    Curve2D? cw = Tool.CurveOnSurface(ed, _face, out double cf, out double cl);
                    if (cw is null)
                    {
                        continue;
                    }
                    var gac = new CurveAdaptor2D();
                    double first = cw.FirstParameter;
                    double last  = cw.LastParameter;
                    if (cw is BSplineCurve2D && (cf < first || cl > last))
                    {
                        // avoiding problems with segment in Bnd_Box
                        gac.Load(cw);
                    }
                    else
                    {
                        gac.Load(cw, cf, cl);
                    }
                    Add2dCurve.Add(gac, Kerf.Engine.Foundation.Precision.Confusion(), ref box);
                }

                box.Get(out double xMin, out double yMin, out double xMax, out double yMax);
                if (isFirst)
                {
                    isFirst = false;
                    uMiddle = (xMin + xMax) * 0.5;
                    vMiddle = (yMin + yMax) * 0.5;
                }
                else
                {
                    double xShift = 0, yShift = 0;
                    if (_surf.IsUClosed())
                    {
                        xShift = ShapeAnalysisAnalysis.AdjustByPeriod(0.5 * (xMin + xMax), uMiddle, uRange);
                    }
                    if (_surf.IsVClosed())
                    {
                        yShift = ShapeAnalysisAnalysis.AdjustByPeriod(0.5 * (yMin + yMax), vMiddle, vRange);
                    }
                    box.Update(xMin + xShift, yMin + yShift, xMax + xShift, yMax + yShift);
                }
                wireBoxes[i] = box;
            }

            for (i = 1; i <= nb; i++)
            {
                Shape asw = ws.Value(i);
                Wire aw = ShapeCasts.Wire(asw);
                Box2d box1 = wireBoxes.Value(i);
                Shape dummy = _face.EmptyCopied();
                Face af = ShapeCasts.Face(dummy);
                af.Orient(Forward);
                b.Add(af, aw);
                // PTV OCC945 06.11.2002 files ie_exhaust-A.stp (entities 3782,  3787)
                // tolerance is too big. It is seems that to identify placement of 2d point
                // it is enough Precision::PConfusion(), cause wea re know that 2d point in TopAbs_ON
                // BRepTopAdaptor_FClass2d clas (af,toluv);
                bool checkShift = true;
                var clas = new FClass2d(af, Kerf.Engine.Foundation.Precision.PConfusion());
                Containment sta = Out;
                Containment staout = clas.PerformInfinitePoint();
                var intWires = new NCollection_List<Shape>();
                int wireIt = 0;
                for (int j = 1; j <= nbAll; j++)
                {
                    wireIt++;
                    // if(i==j) continue;
                    Shape sh2 = allSubShapes.Value(j);
                    if (aw == sh2)
                    {
                        continue;
                    }
                    Containment stb = Unknown;
                    if (sh2.Type == ShapeType.Vertex)
                    {
                        wireIt--;
                        Point3D p = Tool.Pnt(ShapeCasts.Vertex(sh2));
                        Point2D p2d = _surf.ValueOfUV(p, Kerf.Engine.Foundation.Precision.Confusion());
                        stb = clas.Perform(p2d, false);
                        if (stb == staout && (uclosed || vclosed))
                        {
                            var p2d1 = new Point2D();
                            if (uclosed)
                            {
                                p2d1 = p2d1.WithCoord(p2d.X + uRange, p2d.Y);
                                stb = clas.Perform(p2d1, false);
                            }
                            if (stb == staout && vclosed)
                            {
                                p2d1 = p2d1.WithCoord(p2d.X, p2d.Y + vRange);
                                stb = clas.Perform(p2d1, false);
                            }
                        }
                    }
                    else if (sh2.Type == ShapeType.Wire)
                    {
                        checkShift = true;
                        Wire bw = ShapeCasts.Wire(sh2);
                        // int numin =0;
                        Box2d box2 = wireBoxes.Value(wireIt);
                        if (box2.IsOut(box1))
                        {
                            continue;
                        }

                        var ew = new Iterator(bw);
                        for (; ew.More(); ew.Next())
                        {
                            Edge ed = ShapeCasts.Edge(ew.Value());
                            Curve2D? cw = Tool.CurveOnSurface(ed, _face, out double cf, out double cl);
                            if (cw is null)
                            {
                                continue;
                            }
                            Point2D unp = cw.Value((cf + cl) / 2.0);
                            Containment ste = clas.Perform(unp, false);
                            if (ste == Out || ste == In)
                            {
                                if (stb == Unknown)
                                {
                                    stb = ste;
                                }
                                else
                                {
                                    if (!(stb == ste))
                                    {
                                        sta = Unknown;
                                        si[aw] = 0;
                                        j = nbAll;
                                        break;
                                    }
                                }
                            }

                            bool found = false;
                            var unp1 = new Point2D();
                            if (stb == staout && checkShift)
                            {
                                checkShift = false;
                                if (uclosed)
                                {
                                    unp1 = unp1.WithCoord(unp.X + uRange, unp.Y);
                                    found = (staout != clas.Perform(unp1, false));
                                    if (!found)
                                    {
                                        unp1 = unp1.WithX(unp.X - uRange);
                                        found = (staout != clas.Perform(unp1, false));
                                    }
                                }
                                if (vclosed && !found)
                                {
                                    unp1 = unp1.WithCoord(unp.X, unp.Y + vRange);
                                    found = (staout != clas.Perform(unp1, false));
                                    if (!found)
                                    {
                                        unp1 = unp1.WithY(unp.Y - vRange);
                                        found = (staout != clas.Perform(unp1, false));
                                    }
                                }
                                // Additional check of diagonal steps for toroidal surfaces
                                if (!found && uclosed && vclosed)
                                {
                                    for (double dX = -1.0; dX <= 1.0 && !found; dX += 2.0)
                                    {
                                        for (double dY = -1.0; dY <= 1.0 && !found; dY += 2.0)
                                        {
                                            unp1 = unp1.WithCoord(unp.X + uRange * dX, unp.Y + vRange * dY);
                                            found = (staout != clas.Perform(unp1, false));
                                        }
                                    }
                                }
                            }
                            if (found)
                            {
                                if (stb == In)
                                {
                                    stb = Out;
                                }
                                else
                                {
                                    stb = In;
                                }
                                Shift2dWire(bw, _face, new Vector2D(unp1.XY() - unp.XY()), _surf);
                            }
                        }
                    }
                    if (stb == staout)
                    {
                        sta = In;
                    }
                    else
                    {
                        intWires.Append(sh2);
                        mapIntWires.Add(sh2);
                    }
                }

                if (sta == Unknown)
                { // ERREUR
                    SendWarning(aw, new Msg("FixAdvFace.FixOrientation.MSG11")); // Cannot orient wire
                }
                else
                {
                    mw[aw] = intWires;
                    if (sta == Out)
                    {
                        if (staout == In)
                        {
                            // wire is OUT but InfinitePoint is IN => need to reverse
                            var sewd = new ExtendWireData(aw);
                            sewd.Reverse(_face);
                            ws.SetValue(i, sewd.InternalWire());
                            SendWarning(sewd.InternalWire(), new Msg("FixAdvFace.FixOrientation.MSG5")); // Wire on face was reversed
                            seqReversed.Append(i);
                            done = true;
                            si[ws.Value(i)] = 1;
                            mapWires[ws.Value(i)] = intWires;
                        }
                        else
                        {
                            si[aw] = 1;
                            mapWires[aw] = intWires;
                        }
                    }
                    else
                    {
                        if (staout == Out)
                        {
                            si[aw] = 2;
                        }
                        else
                        {
                            si[aw] = 3;
                        }
                    }
                }
            }

            for (i = 1; i <= nb; i++)
            {
                Wire aw = ShapeCasts.Wire(ws.Value(i));
                int tmpi = si.GetValue(aw);
                if (tmpi > 1)
                {
                    if (!mapIntWires.Contains(aw))
                    {
                        NCollection_List<Shape> iw = mw.GetValue(aw);
                        if (tmpi == 3)
                        {
                            // wire is OUT but InfinitePoint is IN => need to reverse
                            var sewd = new ExtendWireData(aw);
                            sewd.Reverse(_face);
                            ws.SetValue(i, sewd.InternalWire());
                            SendWarning(sewd.InternalWire(), new Msg("FixAdvFace.FixOrientation.MSG5")); // Wire on face was reversed
                            seqReversed.Append(i);
                            done = true;
                            mapWires[ws.Value(i)] = iw;
                        }
                        else
                        {
                            mapWires[aw] = iw;
                        }
                    }
                    else
                    {
                        if (tmpi == 2)
                        {
                            // wire is IN but InfinitePoint is OUT => need to reverse
                            var sewd = new ExtendWireData(aw);
                            sewd.Reverse(_face);
                            ws.SetValue(i, sewd.InternalWire());
                            SendWarning(sewd.InternalWire(), new Msg("FixAdvFace.FixOrientation.MSG5")); // Wire on face was reversed
                            seqReversed.Append(i);
                            done = true;
                        }
                    }
                }
            }
        }

        if (isAddNaturalBounds && nb == seqReversed.Length())
        {
            done = false;
        }

        // Should I rebuild? if myRebil is set
        if (done)
        {
            Shape s = _face.EmptyCopied();
            s.Orient(Forward);
            int i = 1;
            for (; i <= nb; i++)
            {
                b.Add(s, ws.Value(i));
            }

            if (nb < nbAll)
            {
                for (i = 1; i <= nbAll; i++)
                {
                    Shape s2 = allSubShapes.Value(i);
                    if (s2.Type != ShapeType.Wire
                        || (s2.Orientation != Forward && s2.Orientation != Reversed))
                    {
                        b.Add(s, s2);
                    }
                }
            }

            if (!_fwd)
            {
                s.Orient(Reversed);
            }
            Context()?.Replace(_face, s);
            _face = ShapeCasts.Face(s);
            BrepTools.Update(_face);
        }
        return done;
    }

    //: i7 abv 18 Sep 98: ProSTEP TR9 r0501-ug.stp: algorithm of fixing missing seam changed
    // test whether the wire is opened on period of periodical surface
    private static bool CheckWire(in Wire wire, in Face face, double dU, double dV,
                                  out int isuopen, out int isvopen, out bool isDeg)
    {
        var vec = new Xy();
        vec.SetX(0);
        vec.SetY(0);
        var sae = new EdgeAnalysis();

        isuopen = isvopen = 0;
        isDeg = true;
        for (var ed = new Iterator(wire); ed.More(); ed.Next())
        {
            Edge edge = ShapeCasts.Edge(ed.Value());
            if (!Tool.Degenerated(edge))
            {
                isDeg = false;
            }
            if (!sae.PCurve(edge, face, out Curve2D? c2d, out double f, out double l, true))
            {
                return false;
            }
            vec += c2d!.Value(l).XY() - c2d.Value(f).XY();
        }

        double delta = Math.Abs(vec.X()) - dU;
        if (Math.Abs(delta) < 0.1 * dU)
        {
            if (vec.X() > 0.0)
            {
                isuopen = 1;
            }
            else
            {
                isuopen = -1;
            }
        }
        else
        {
            isuopen = 0;
        }

        delta = Math.Abs(vec.Y()) - dV;
        if (Math.Abs(delta) < 0.1 * dV)
        {
            if (vec.Y() > 0.0)
            {
                isvopen = 1;
            }
            else
            {
                isvopen = -1;
            }
        }
        else
        {
            isvopen = 0;
        }

        return isuopen != 0 || isvopen != 0;
    }

    /// <summary>
    /// Adds a missing seam to a face on a closed surface whose wires are open in 2d by a period: the face is split by
    /// ShapeFix_ComposeShell. Returns true if done; the result may then be a shell (<see cref="Result"/>).
    /// </summary>
    public bool FixMissingSeam()
    {
        using WorkingOverlay.Scope work = EnterWork();
        return FixMissingSeamCore();
    }

    private bool FixMissingSeamCore()
    {
        if (_surf is null)
        {
            return false;
        }

        bool uclosed = _surf.IsUClosed();
        bool vclosed = _surf.IsVClosed();

        if (!uclosed && !vclosed)
        {
            return false;
        }

        if (Context() is not null)
        {
            Shape s0 = Context()!.InternalApply(_face);
            _face = ShapeCasts.Face(s0);
        }

        //%pdn: surface should be made periodic before (see ShapeCustom_Surface)!
        if (_surf.Surface() is global::Kerf.Geometry.BSplineSurface bSpl)
        {
            if (!bSpl.IsUPeriodic && !bSpl.IsVPeriodic)
            {
                return false;
            }
        }

        double uRange, vRange;
        _surf.Bounds(out double suf, out double sul, out double svf, out double svl);
        BrepTools.UVBounds(_face, out double fU1, out double fU2, out double fV1, out double fV2);

        // pdn OCC55 fix to faces without the wires to avoid identical first and last parameters
        if (Kerf.Engine.Foundation.Precision.IsInfinite(suf) || Kerf.Engine.Foundation.Precision.IsInfinite(sul))
        {
            if (Kerf.Engine.Foundation.Precision.IsInfinite(suf))
            {
                suf = fU1;
            }
            if (Kerf.Engine.Foundation.Precision.IsInfinite(sul))
            {
                sul = fU2;
            }
            if (Math.Abs(sul - suf) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                if (Kerf.Engine.Foundation.Precision.IsInfinite(suf))
                {
                    suf -= 1000.0;
                }
                else
                {
                    sul += 1000.0;
                }
            }
        }
        if (Kerf.Engine.Foundation.Precision.IsInfinite(svf) || Kerf.Engine.Foundation.Precision.IsInfinite(svl))
        {
            if (Kerf.Engine.Foundation.Precision.IsInfinite(svf))
            {
                svf = fV1;
            }
            if (Kerf.Engine.Foundation.Precision.IsInfinite(svl))
            {
                svl = fV2;
            }
            if (Math.Abs(svl - svf) < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                if (Kerf.Engine.Foundation.Precision.IsInfinite(svf))
                {
                    svf -= 1000.0;
                }
                else
                {
                    svl += 1000.0;
                }
            }
        }

        uRange = Min(Math.Abs(sul - suf), Kerf.Engine.Foundation.Precision.Infinite());
        vRange = Min(Math.Abs(svl - svf), Kerf.Engine.Foundation.Precision.Infinite());
        //  double UTol = 0.2 * URange, VTol = 0.2 * VRange;
        int ismodeu = 0, ismodev = 0; // szv#4:S4163:12Mar99 was Boolean
        int isdeg1 = 0, isdeg2 = 0;

        var ws = new NCollection_Sequence<Shape>();
        var seqNonManif = new NCollection_Sequence<Shape>();
        for (var wi = new Iterator(_face, false); wi.More(); wi.Next())
        {
            if (wi.Value().Type != ShapeType.Wire
                || (wi.Value().Orientation != Forward && wi.Value().Orientation != Reversed))
            {
                seqNonManif.Append(wi.Value());
                continue;
            }
            ws.Append(wi.Value());
        }

        Wire w1 = new Wire(), w2 = new Wire();
        int i;
        for (i = 1; i <= ws.Length(); i++)
        {
            Wire wire = ShapeCasts.Wire(ws.Value(i));
            if (!CheckWire(wire, _face, uRange, vRange, out int isuopen, out int isvopen, out bool isdeg))
            {
                continue;
            }
            if (w1.IsNull)
            {
                w1      = wire;
                ismodeu = isuopen;
                ismodev = isvopen;
                isdeg1  = isdeg ? i : 0;
            }
            else if (w2.IsNull)
            {
                if (ismodeu == -isuopen && ismodev == -isvopen)
                {
                    w2     = wire;
                    isdeg2 = isdeg ? i : 0;
                }
                else if (ismodeu == isuopen && ismodev == isvopen)
                {
                    w2     = wire;
                    isdeg2 = isdeg ? 1 : 0; // OCCT: the flag, not the index i as in the other branch (isdeg2 = isdeg)
                    //: abv 29.08.01: If wires are contraversal, reverse one of them
                    // If first one is single degenerated edge, reverse it; else second
                    if (isdeg1 != 0)
                    {
                        w1.Reverse();
                        ismodeu = -ismodeu;
                        ismodev = -ismodev;
                    }
                    else
                    {
                        w2.Reverse();
                    }
                }
            }
            //    else return false; //  abort
            else
            {
                //: abv 30.08.09: if more than one open wires and more than two of them are
                // completely degenerated, remove any of them
                if (isdeg || isdeg1 != 0 || isdeg2 != 0)
                {
                    ws.Remove(isdeg ? i : isdeg2 != 0 ? isdeg2 : isdeg1);
                    w1.Nullify();
                    w2.Nullify();
                    i = 0;
                    continue;
                }
            }
        }

        var b = new BrepBuilder();
        var torSurf = _surf.Surface() as ToroidalSurface;
        bool isDegeneratedTor = (torSurf is null ? false : torSurf.MajorRadius < torSurf.MinorRadius);
        // if the second wire is not null, we don't need mark the torus as degenerated
        // and should process it as a regular one.
        if (isDegeneratedTor && !w2.IsNull)
        {
            isDegeneratedTor = false;
        }

        if (w1.IsNull)
        {
            return false;
        }
        else if (w2.IsNull)
        {
            // For spheres and BSpline cone-like surfaces(bug 24055):
            // If only one of wires limiting face on surface is open in 2d,
            // this may means that degenerated edge should be added, and
            // then usual procedure applied
            var p = new Point2D();
            var d = new Direction2D();
            double range;

            if (ismodeu != 0 && isDegeneratedTor)
            {
                double ra  = torSurf!.MajorRadius;
                double ri  = torSurf.MinorRadius;
                double phi = Math.Acos(-ra / ri);
                p = p.WithCoord(0.0, (ismodeu > 0 ? M_PI + phi : phi));

                double xCoord = -ismodeu;
                d = d.WithCoord(xCoord, 0.0);
                range = 2.0 * M_PI;
            }
            else if (ismodeu != 0 && _surf.Surface() is SphericalSurface)
            {
                p = p.WithCoord((ismodeu < 0 ? 0.0 : 2.0 * M_PI), ismodeu * 0.5 * M_PI);
                double xCoord = -ismodeu;
                d = d.WithCoord(xCoord, 0.0);
                range = 2.0 * M_PI;
            }
            else if (ismodev != 0 && _surf.Surface() is global::Kerf.Geometry.BSplineSurface)
            {
                double uCoord;
                if (_surf.Value(suf, svf).DistanceTo(_surf.Value(suf, (svf + svl) / 2)) < Kerf.Engine.Foundation.Precision.Confusion())
                {
                    uCoord = suf;
                }
                else if (_surf.Value(sul, svf).DistanceTo(_surf.Value(sul, (svf + svl) / 2)) < Kerf.Engine.Foundation.Precision.Confusion())
                {
                    uCoord = sul;
                }
                else
                {
                    return false;
                }

                p = p.WithCoord(uCoord, (ismodev < 0 ? 0.0 : vRange));
                d = d.WithCoord(0.0, -ismodev);
                range = vRange;
            }
            else if (ismodeu != 0 && _surf.Surface() is global::Kerf.Geometry.BSplineSurface)
            {
                double vCoord;
                if (_surf.Value(suf, svf).DistanceTo(_surf.Value((suf + sul) / 2, svf)) < Kerf.Engine.Foundation.Precision.Confusion())
                {
                    vCoord = svf;
                }
                else if (_surf.Value(sul, svl).DistanceTo(_surf.Value((suf + sul) / 2, svl)) < Kerf.Engine.Foundation.Precision.Confusion())
                {
                    vCoord = svl;
                }
                else
                {
                    return false;
                }

                p = p.WithCoord((ismodeu < 0 ? 0.0 : uRange), vCoord);
                double xCoord = -ismodeu;
                d = d.WithCoord(xCoord, 0.0);
                range = uRange;
            }
            else
            {
                return false;
            }

            var line = new LineCurve2D(p, d);
            var edge = new Edge();
            b.MakeEdge(ref edge);
            b.Degenerated(edge, true);
            b.UpdateEdge(edge, line, _face, Kerf.Engine.Foundation.Precision.Confusion());
            b.Range(edge, _face, 0.0, range);
            var v = new Vertex();
            b.MakeVertex(ref v, _surf.Value(p.X, p.Y), Kerf.Engine.Foundation.Precision.Confusion());
            v.Orient(Forward);
            b.Add(edge, v);
            v.Orient(Reversed);
            b.Add(edge, v);
            b.MakeWire(ref w2);
            b.Add(w2, edge);
            ws.Append(w2);
        }

        // Check consistency of orientations of the two wires that need to be connected by a seam
        double uf = suf, vf = svf;
        int coord = (ismodeu != 0 ? 1 : 0);
        int isneg = (ismodeu != 0 ? ismodeu : -ismodev);
        double period = (ismodeu != 0 ? uRange : vRange);
        Shape s;
        Span<double> m1 = stackalloc double[4]; // m1[c][k] is m1[2 * c + k]
        Span<double> m2 = stackalloc double[4];
        s = _face.EmptyCopied();
        b.Add(s, w1);
        ShapeAnalysisAnalysis.GetFaceUVBounds(ShapeCasts.Face(s), out m1[0], out m1[1], out m1[2], out m1[3]);
        s = _face.EmptyCopied();
        b.Add(s, w2);
        ShapeAnalysisAnalysis.GetFaceUVBounds(ShapeCasts.Face(s), out m2[0], out m2[1], out m2[2], out m2[3]);

        // For the case when surface is closed only in one direction it is necessary to check
        // validity of orientation of the open wires in parametric space.
        // In case of U closed surface wire with minimal V coordinate should be directed in positive
        // direction by U In case of V closed surface wire with minimal U coordinate should be directed in
        // negative direction by V
        if (!vclosed || !uclosed || isDegeneratedTor)
        {
            double deltaOther = 0.5 * (m2[2 * coord + 0] + m2[2 * coord + 1]) - 0.5 * (m1[2 * coord + 0] + m1[2 * coord + 1]);
            if (deltaOther * isneg < 0)
            {
                w1.Reverse();
                w2.Reverse();
            }
        }

        // sort original wires
        var sfw = new WireFixer();
        sfw.SetFace(_face, _surf);
        sfw.SetPrecision(Precision());
        var wd1 = new ExtendWireData(w1);
        var wd2 = new ExtendWireData(w2);
        sfw.Load(wd1);
        sfw.FixReorder();
        sfw.Load(wd2);
        sfw.FixReorder();
        Wire w11 = wd1.InternalWire();
        Wire w21 = wd2.InternalWire();

        //: abv 29.08.01: reconstruct face taking into account reversing
        Shape dummy = _face.EmptyCopied();
        Face tmpF = ShapeCasts.Face(dummy);
        tmpF.Orient(Forward);
        for (i = 1; i <= ws.Length(); i++)
        {
            Wire wire = ShapeCasts.Wire(ws.Value(i));
            if (wire.IsSame(w1))
            {
                wire = w11;
            }
            else if (wire.IsSame(w2))
            {
                wire = w21;
            }
            else
            {
                // other wires (not boundary) are considered as holes; make sure to have them oriented
                // accordingly
                Shape curface = tmpF.EmptyCopied();
                b.Add(curface, wire);
                curface.Orient(_face.Orientation);
                if (ShapeAnalysisAnalysis.IsOuterBound(ShapeCasts.Face(curface)))
                {
                    wire.Reverse();
                }
            }
            b.Add(tmpF, wire);
        }

        tmpF.Orient(_face.Orientation);

        // A special kind of FixShifted is necessary for torus-like
        // surfaces to adjust wires by period ALONG the missing SEAM direction
        // tr9_r0501-ug.stp #187640
        if (uclosed && vclosed && !isDegeneratedTor)
        {
            double shiftw2 = ShapeAnalysisAnalysis.AdjustByPeriod(
              0.5 * (m2[2 * coord + 0] + m2[2 * coord + 1]),
              0.5 * (m1[2 * coord + 0] + m1[2 * coord + 1] + isneg * (period + Kerf.Engine.Foundation.Precision.PConfusion())),
              period);
            m1[2 * coord + 0] = Min(m1[2 * coord + 0], m2[2 * coord + 0] + shiftw2);
            m1[2 * coord + 1] = Max(m1[2 * coord + 1], m2[2 * coord + 1] + shiftw2);
            for (var it = new Iterator(tmpF, false); it.More(); it.Next())
            {
                if (it.Value().Type != ShapeType.Wire)
                {
                    continue;
                }
                Wire w = ShapeCasts.Wire(it.Value());
                if (w == w11)
                {
                    continue;
                }
                double shift;
                if (w == w21)
                {
                    shift = shiftw2;
                }
                else
                {
                    s = tmpF.EmptyCopied();
                    b.Add(s, w);
                    ShapeAnalysisAnalysis.GetFaceUVBounds(ShapeCasts.Face(s), out m2[0], out m2[1], out m2[2], out m2[3]);
                    shift = ShapeAnalysisAnalysis.AdjustByPeriod(0.5 * (m2[2 * coord + 0] + m2[2 * coord + 1]),
                                                         0.5 * (m1[2 * coord + 0] + m1[2 * coord + 1]),
                                                         period);
                }
                if (shift != 0.0)
                {
                    var v = new Vector2D(0.0, 0.0);
                    v = v.WithCoord(coord + 1, shift);
                    var sae1 = new EdgeAnalysis();
                    for (var iw = new Iterator(w); iw.More(); iw.Next())
                    {
                        Edge e = ShapeCasts.Edge(iw.Value());
                        if (!sae1.PCurve(e, tmpF, out Curve2D? C, out _, out _))
                        {
                            continue;
                        }
                        EditPCurve(e, C!, c => c.Translate(v));
                    }
                }
            }
            // abv 05 Feb 02: OCC34
            // by the way, select proper split place by V to avoid extra intersections
            if (m1[2 * coord + 1] - m1[2 * coord + 0] <= period)
            {
                double other = 0.5 * (m1[2 * coord + 0] + m1[2 * coord + 1] - period);
                if (ismodeu != 0)
                {
                    vf = other;
                }
                else
                {
                    uf = other;
                }
            }
        }

        // find the best place by u and v to insert a seam
        // (so as to minimize splitting edges as possible)
        var sae = new EdgeAnalysis();
        int foundU = 0, foundV = 0;
        int nb1 = wd1.EdgeCount;
        int nb2 = wd2.EdgeCount;
        for (int i1 = 1; i1 <= nb1 + nb2; i1++)
        {
            Edge edge1 = (i1 <= nb1 ? wd1.Edge(i1) : wd2.Edge(i1 - nb1));
            if (!sae.PCurve(edge1, tmpF, out Curve2D? c2d, out double f, out double l, true))
            {
                return false;
            }
            Point2D pos1 = new Point2D(c2d!.Value(l).XY());
            // the best place is end of edge which is nearest to 0
            bool skipU = !uclosed;
            if (uclosed && ismodeu != 0)
            {
                pos1 = pos1.WithX(pos1.X + ShapeAnalysisAnalysis.AdjustByPeriod(pos1.X, suf, uRange));
                if (foundU == 2 && Math.Abs(pos1.X) > Math.Abs(uf))
                {
                    skipU = true;
                }
                else if (foundU == 0 || (foundU == 1 && Math.Abs(pos1.X) < Math.Abs(uf)))
                {
                    foundU = 1;
                    uf     = pos1.X;
                }
            }
            bool skipV = !vclosed;
            if (vclosed && ismodeu == 0)
            {
                pos1 = pos1.WithY(pos1.Y + ShapeAnalysisAnalysis.AdjustByPeriod(pos1.Y, svf, vRange));
                if (foundV == 2 && Math.Abs(pos1.Y) > Math.Abs(vf))
                {
                    skipV = true;
                }
                else if (foundV == 0 || (foundV == 1 && Math.Abs(pos1.Y) < Math.Abs(vf)))
                {
                    foundV = 1;
                    vf     = pos1.Y;
                }
            }
            if (skipU && skipV)
            {
                if (i1 <= nb1)
                {
                    continue;
                }
                else
                {
                    break;
                }
            }
            // or yet better - if it is end of some edges on both wires
            for (int i2 = 1; i1 <= nb1 && i2 <= nb2; i2++)
            {
                Edge edge2 = wd2.Edge(i2);
                if (!sae.PCurve(edge2, tmpF, out c2d, out f, out l, true))
                {
                    return false;
                }
                Point2D pos2 = new Point2D(c2d!.Value(f).XY());
                if (uclosed && ismodeu != 0)
                {
                    pos2 = pos2.WithX(pos2.X + ShapeAnalysisAnalysis.AdjustByPeriod(pos2.X, pos1.X, uRange));
                    if (Math.Abs(pos2.X - pos1.X) < Kerf.Engine.Foundation.Precision.PConfusion()
                        && (foundU != 2 || Math.Abs(pos1.X) < Math.Abs(uf)))
                    {
                        foundU = 2;
                        uf     = pos1.X;
                    }
                }
                if (vclosed && ismodeu == 0)
                {
                    pos2 = pos2.WithY(pos2.Y + ShapeAnalysisAnalysis.AdjustByPeriod(pos2.Y, pos1.Y, vRange));
                    if (Math.Abs(pos2.Y - pos1.Y) < Kerf.Engine.Foundation.Precision.PConfusion()
                        && (foundV != 2 || Math.Abs(pos1.Y) < Math.Abs(vf)))
                    {
                        foundV = 2;
                        vf     = pos1.Y;
                    }
                }
            }
        }

        // pdn fixing RTS on offsets
        if (uf < suf || uf > sul)
        {
            uf += ShapeAnalysisAnalysis.AdjustToPeriod(uf, suf, suf + uRange);
        }
        if (vf < svf || vf > svl)
        {
            vf += ShapeAnalysisAnalysis.AdjustToPeriod(vf, svf, svf + vRange);
        }

        // Create fictive grid and call ComposeShell to insert a seam
        var rts = new TrimmedSurface(_surf.Surface()!, uf, uf + uRange, vf, vf + vRange);
        var grid = new NCollection_HArray2<Surface>(1, 1, 1, 1);
        grid.SetValue(1, 1, rts); // mySurf->Surface() );
        var g = new CompositeSurface(grid);
        var L = new Placement();

        // addition non-manifold topology
        int j = 1;
        for (; j <= seqNonManif.Length(); j++)
        {
            b.Add(tmpF, seqNonManif.Value(j));
        }

        var compShell = new ComposeShellFixer();
        compShell.Init(g, L, tmpF, Kerf.Engine.Foundation.Precision.Confusion()); // myPrecision
        if (Context() is null)
        {
            SetContext(new ReShapeBuilder());
        }
        compShell.ClosedMode() = true;
        compShell.SetContext(Context());
        compShell.SetMaxTolerance(MaxTolerance());
        compShell.Perform();

        // abv 03.07.00: CAX-IF TRJ4: trj4_k1_goe-tu-214.stp: #785: reset mySurf
        _surf = new SurfaceAnalysis(rts);

        _result = compShell.InternalResult();

        Context()!.Replace(_face, _result);

        // Remove small wires and / or faces that can be generated by ComposeShell
        // (see tests bugs step bug30052_4, de step_3 E6)
        int nbFaces = 0;
        var expF = new Explorer(_result, global::Kerf.Topology.ShapeType.Face);
        for (; expF.More(); expF.Next())
        {
            Face face = ShapeCasts.Face(expF.Value());
            var expW = new Explorer(face, ShapeType.Wire);
            int nbWires = 0;
            for (; expW.More(); expW.Next())
            {
                var aSfw = new WireFixer(ShapeCasts.Wire(expW.Value()), face, Precision());
                aSfw.SetContext(Context());
                if (aSfw.NbEdges() != 0)
                {
                    aSfw.FixSmall(true, Precision());
                }
                if (aSfw.NbEdges() == 0)
                {
                    Context()!.Remove(expW.Value());
                    continue;
                }
                nbWires++;
            }
            if (nbWires == 0)
            {
                Context()!.Remove(face);
                continue;
            }
            nbFaces++;
        }

        _result = Context()!.InternalApply(_result);
        for (var exp = new Explorer(_result, global::Kerf.Topology.ShapeType.Face); exp.More(); exp.Next())
        {
            _face = ShapeCasts.Face(Context()!.InternalApply(exp.Current()));
            if (_face.IsNull)
            {
                continue;
            }
            if (nbFaces > 1)
            {
                FixSmallAreaWireCore(true);
                Shape shape = Context()!.InternalApply(_face);
                if (shape.IsNull)
                {
                    continue;
                }
                _face = ShapeCasts.Face(shape);
            }
            BrepTools.Update(_face); //: p4
        }
        _result = Context()!.InternalApply(_result);

        SendWarning(new Msg("FixAdvFace.FixMissingSeam.MSG0")); // Missing seam-edge added
        return true;
    }

    /// <summary>
    /// Removes the wires of null area from the face; with <paramref name="isRemoveSmallFace"/>, removes the face
    /// (through the context) when all its wires are. Returns true if a wire was removed and the face kept.
    /// </summary>
    public bool FixSmallAreaWire(bool isRemoveSmallFace)
    {
        using WorkingOverlay.Scope work = EnterWork();
        return FixSmallAreaWireCore(isRemoveSmallFace);
    }

    //%14 pdn 24.02.99 PRO10109, USA60293 fix wire on face with small area.
    private bool FixSmallAreaWireCore(bool isRemoveSmallFace)
    {
        if (Context() is not null)
        {
            Shape shape0 = Context()!.InternalApply(_face);
            _face = ShapeCasts.Face(shape0);
        }

        var builder = new BrepBuilder();
        int nbRemoved = 0, nbWires = 0;

        Shape emptyCopy = _face.EmptyCopied();
        Face face = ShapeCasts.Face(emptyCopy);
        face.Orient(Forward);

        double tolerance3d = base.Precision();
        for (var wIt = new Iterator(_face, false); wIt.More(); wIt.Next())
        {
            Shape shape = wIt.Value();
            if (shape.Type != ShapeType.Wire && shape.Orientation != Forward
                && shape.Orientation != Reversed)
            {
                continue;
            }

            // OCCT: the test is && where || is meant, so a FORWARD or REVERSED vertex (or an INTERNAL wire) gets here, and
            // TopoDS::Wire, unchecked in OCCT's release build, reinterprets the vertex as a wire: it has no edge, is not
            // small, and is kept. The port reinterprets it the same way rather than throw.
            Wire wire = shape.Type == ShapeType.Wire
                ? ShapeCasts.Wire(shape)
                : new Wire(shape.TShape(), shape.Placement, shape.Orientation);
            var analyzer = new WireAnalysis(wire, _face, tolerance3d);
            if (analyzer.CheckSmallArea(wire))
            {
                // Null area wire detected, wire skipped
                SendWarning(wire, new Msg("FixAdvFace.FixSmallAreaWire.MSG0"));
                ++nbRemoved;
            }
            else
            {
                builder.Add(face, wire);
                ++nbWires;
            }
        }

        if (nbRemoved <= 0)
        {
            return false;
        }

        if (nbWires <= 0)
        {
            if (isRemoveSmallFace && Context() is not null)
            {
                Context()!.Remove(_face);
            }

            return false;
        }
        face.Orient(_face.Orientation);
        Context()?.Replace(_face, face);

        _face = face;
        return true;
    }

    private static void FindNext(in Shape aVert, in Shape ainitEdge,
                                 IndexedSet<Shape> aMapVertices,
                                 Dictionary<Shape, NCollection_List<Shape>> aMapVertexEdges,
                                 HashSet<Shape> aMapSmallEdges,
                                 HashSet<Shape> aMapSeemEdges,
                                 HashSet<Shape> aMapEdges,
                                 ExtendWireData aWireData)
    {
        var itV = new Iterator(ainitEdge);
        Shape anextVert = aVert;
        bool isFind = false;
        for (; itV.More() && !isFind; itV.Next())
        {
            if (!itV.Value().IsSame(aVert))
            {
                isFind    = true;
                anextVert = itV.Value();
            }
        }

        if (!isFind && !aMapSmallEdges.Contains(ainitEdge))
        {
            return;
        }
        if (isFind && aMapVertices.Contains(anextVert))
        {
            return;
        }

        NCollection_List<Shape> aledges = aMapVertexEdges.GetValue(anextVert);
        var liter = new NCollection_List<Shape>.Iterator(aledges);
        isFind = false;
        Shape anextEdge;
        for (; liter.More() && !isFind; liter.Next())
        {
            if (!aMapEdges.Contains(liter.Value()) && !liter.Value().IsSame(ainitEdge))
            {
                anextEdge = liter.Value();
                aWireData.Add(anextEdge);
                if (aMapSeemEdges.Contains(anextEdge))
                {
                    aWireData.Add(anextEdge.Reversed());
                }
                isFind = true;
                aMapEdges.Add(anextEdge);
                FindNext(anextVert, anextEdge, aMapVertices, aMapVertexEdges, aMapSmallEdges, aMapSeemEdges, aMapEdges,
                         aWireData);
            }
        }
    }

    private static bool IsClosed2D(in Face aFace, in Wire aWire)
    {
        bool isClosed = true;
        var asaw = new WireAnalysis(aWire, aFace, Kerf.Engine.Foundation.Precision.Confusion());
        for (int i = 1; i <= asaw.EdgeCount && isClosed; i++)
        {
            Edge edge1 = asaw.WireData()!.Edge(i);
            // checking that wire is closed in 2D space with tolerance of vertex.
            var sae = new EdgeAnalysis();
            Vertex v1 = sae.FirstVertex(edge1);
            asaw.SetPrecision(Tool.Tolerance(v1));
            asaw.CheckGap2d(i);
            isClosed = (asaw.LastCheckStatus(ExtendStatus.OK));
        }
        return isClosed;
    }

    /// <summary>
    /// Splits the wire loaded in the wire tool (<see cref="FixWireTool"/>) at its loops (vertices shared by more than
    /// two edges) into the wires appended to <paramref name="aResWires"/>. Returns true if done and the wires are
    /// closed in 2d.
    /// </summary>
    public bool FixLoopWire(NCollection_Sequence<Shape> aResWires)
    {
        using WorkingOverlay.Scope work = EnterWork();
        return FixLoopWireCore(aResWires);
    }

    private bool FixLoopWireCore(NCollection_Sequence<Shape> aResWires)
    {
        var mapVertices = new IndexedSet<Shape>(ShapeHasher.Instance);
        var mapVertexEdges = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var mapSmallEdges = new HashSet<Shape>(ShapeHasher.Instance);
        var mapSeemEdges = new HashSet<Shape>(ShapeHasher.Instance);
        if (!FixWireTool().Analyzer().CheckLoop(mapVertices, mapVertexEdges, mapSmallEdges, mapSeemEdges))
        {
            return false;
        }

        var mapEdges = new HashSet<Shape>(ShapeHasher.Instance);
        var seqWires = new NCollection_Sequence<Shape>();

        // collecting wires from common vertex belonging more than 2 edges
        int i = 1;
        for (; i <= mapVertices.Count; i++)
        {
            Shape vert = mapVertices[i - 1];
            NCollection_List<Shape> aledges = mapVertexEdges.GetValue(vert);
            var liter = new NCollection_List<Shape>.Iterator(aledges);
            for (; liter.More(); liter.Next())
            {
                Edge edge = ShapeCasts.Edge(liter.Value());
                if (mapEdges.Contains(edge))
                {
                    continue;
                }

                var wireData = new ExtendWireData();
                wireData.Add(edge);
                if (mapSeemEdges.Contains(edge))
                {
                    wireData.Add(edge.Reversed());
                }
                mapEdges.Add(edge);
                FindNext(vert, edge, mapVertices, mapVertexEdges, mapSmallEdges, mapSeemEdges, mapEdges, wireData);
                if (wireData.EdgeCount == 1 && mapSmallEdges.Contains(wireData.Edge(1)))
                {
                    continue;
                }
                Wire wire = wireData.InternalWire();
                Exploration.Vertices(wire, out Vertex v1, out Vertex v2);

                if (v1.IsSame(v2))
                {
                    var asewd = new ExtendWireData(wire);
                    var asfw = new WireFixer();
                    asfw.Load(asewd);
                    asfw.FixReorder();
                    Wire awire2 = asfw.InternalWire();
                    aResWires.Append(awire2);
                }
                else
                {
                    seqWires.Append(wireData.InternalWire());
                }
            }
        }

        if (seqWires.Length() == 1)
        {
            aResWires.Append(seqWires.Value(1));
        }
        else
        {
            // collecting whole wire from two not closed wires having two common vertices.
            for (i = 1; i <= seqWires.Length(); i++)
            {
                Wire wire = ShapeCasts.Wire(seqWires.Value(i));
                Exploration.Vertices(wire, out Vertex v1, out Vertex v2);
                int j = i + 1;
                for (; j <= seqWires.Length(); j++)
                {
                    Wire wire2 = ShapeCasts.Wire(seqWires.Value(j));
                    Exploration.Vertices(wire2, out Vertex v21, out Vertex v22);
                    if ((v1.IsSame(v21) || v1.IsSame(v22)) && (v2.IsSame(v21) || v2.IsSame(v22)))
                    {
                        var asewd = new ExtendWireData(wire);
                        asewd.Add(wire2);
                        var asfw = new WireFixer();
                        asfw.Load(asewd);
                        asfw.FixReorder();
                        aResWires.Append(asfw.InternalWire());
                        seqWires.Remove(j--);
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE7);
                        break;
                    }
                }
                if (j <= seqWires.Length())
                {
                    seqWires.Remove(i--);
                }
            }
            if (seqWires.Length() < 3)
            {
                for (i = 1; i <= seqWires.Length(); i++)
                {
                    aResWires.Append(seqWires.Value(i));
                }
            }
            else
            {
                // collecting wires having one common vertex
                for (i = 1; i <= seqWires.Length(); i++)
                {
                    Wire wire = ShapeCasts.Wire(seqWires.Value(i));
                    Exploration.Vertices(wire, out Vertex v1, out Vertex v2);
                    int j = i + 1;
                    for (; j <= seqWires.Length(); j++)
                    {
                        Wire wire2 = ShapeCasts.Wire(seqWires.Value(j));
                        Exploration.Vertices(wire2, out Vertex v21, out Vertex v22);
                        if ((v1.IsSame(v21) || v1.IsSame(v22)) || (v2.IsSame(v21) || v2.IsSame(v22)))
                        {
                            var asewd = new ExtendWireData(wire);
                            asewd.Add(wire2);
                            var asfw = new WireFixer();
                            asfw.Load(asewd);
                            asfw.FixReorder();
                            wire = asfw.InternalWire();
                            Exploration.Vertices(wire, out v1, out v2);
                            seqWires.Remove(j--);
                            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE7);
                        }
                    }
                    aResWires.Append(wire);
                }
            }
        }
        bool isClosed = true;

        // checking that obtained wires is closed in 2D space
        if (_surf is not null && _surf.Adaptor3d()!.GetType() != Plane)
        {
            Shape emptyCopied = _face.EmptyCopied();
            Face tmpFace = ShapeCasts.Face(emptyCopied);
            tmpFace.Orient(Forward);

            for (i = 1; i <= aResWires.Length() && isClosed; i++)
            {
                Wire awire = ShapeCasts.Wire(aResWires.Value(i));
                isClosed = IsClosed2D(tmpFace, awire);
            }
        }

        return !aResWires.IsEmpty() && isClosed;
    }

    private bool SplitEdge(ExtendWireData sewd, int num, double param, in Vertex vert, double preci,
                           Dictionary<Shape, Box2d> boxes)
    {
        Edge edge = sewd.Edge(num);
        Edge newE1 = new Edge(), newE2 = new Edge();
        var tool = new SplitToolFixer();
        if (tool.SplitEdge(edge, param, vert, _face, ref newE1, ref newE2, preci, 0.01 * preci))
        {
            SplitEdgeDone(sewd, num, edge, newE1, newE2, boxes);
            return true;
        }
        return false;
    }

    private bool SplitEdge(ExtendWireData sewd, int num, double param1, double param2, in Vertex vert,
                           double preci, Dictionary<Shape, Box2d> boxes)
    {
        Edge edge = sewd.Edge(num);
        Edge newE1 = new Edge(), newE2 = new Edge();
        var tool = new SplitToolFixer();
        if (tool.SplitEdge(edge, param1, param2, vert, _face, ref newE1, ref newE2, preci, 0.01 * preci))
        {
            SplitEdgeDone(sewd, num, edge, newE1, newE2, boxes);
            return true;
        }
        return false;
    }

    // The common tail of OCCT's two SplitEdge methods (identical in OCCT).
    private void SplitEdgeDone(ExtendWireData sewd, int num, in Edge edge, in Edge newE1,
                               in Edge newE2, Dictionary<Shape, Box2d> boxes)
    {
        // change context
        var wd = new ExtendWireData();
        wd.Add(newE1);
        wd.Add(newE2);
        Context()?.Replace(edge, wd.InternalWire());
        for (var exp = new Explorer(wd.InternalWire(), ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            BrepTools.Update(e);
        }

        // change sewd and boxes
        sewd.Set(newE1, num);
        if (num == sewd.EdgeCount)
        {
            sewd.Add(newE2);
        }
        else
        {
            sewd.Add(newE2, num + 1);
        }

        boxes.Remove(edge);
        Surface s = Tool.Surface(_face, out Placement l)!;
        var sae = new EdgeAnalysis();
        foreach (Edge newE in (ReadOnlySpan<Edge>)[newE1, newE2])
        {
            if (sae.PCurve(newE, s, l, out Curve2D? c2d, out double cf, out double cl, false))
            {
                var box = new Box2d();
                var gac = new CurveAdaptor2D();
                double first = c2d!.FirstParameter;
                double last  = c2d.LastParameter;
                if (c2d is BSplineCurve2D && (cf < first || cl > last))
                {
                    // pdn avoiding problems with segment in Bnd_Box
                    gac.Load(c2d);
                }
                else
                {
                    gac.Load(c2d, cf, cl);
                }
                Add2dCurve.Add(gac, Kerf.Engine.Foundation.Precision.Confusion(), ref box);
                boxes[newE] = box;
            }
        }
    }

    /// <summary>Fixes the intersections between the wires of the face. Returns true if done.</summary>
    public bool FixIntersectingWires()
    {
        using WorkingOverlay.Scope work = EnterWork();
        return FixIntersectingWiresCore();
    }

    private bool FixIntersectingWiresCore()
    {
        var iTool = new IntersectionToolFixer(Context(), Precision(), MaxTolerance());
        return iTool.FixIntersectingWires(ref _face);
    }

    /// <summary>Removes the wires made of two coincident edges (a face must keep another wire). Returns true if done.</summary>
    public bool FixWiresTwoCoincEdges()
    {
        using WorkingOverlay.Scope work = EnterWork();
        return FixWiresTwoCoincEdgesCore();
    }

    private bool FixWiresTwoCoincEdgesCore()
    {
        if (Context() is not null)
        {
            Shape s = Context()!.InternalApply(_face);
            _face = ShapeCasts.Face(s);
        }

        Orientation ori = _face.Orientation;
        Shape emptyCopied = _face.EmptyCopied();
        Face face = ShapeCasts.Face(emptyCopied);
        face.Orient(Forward);
        int nbWires = 0;
        var b = new BrepBuilder();

        for (var it = new Iterator(_face, false); it.More(); it.Next())
        {
            if (it.Value().Type != ShapeType.Wire
                || (it.Value().Orientation != Forward && it.Value().Orientation != Reversed))
            {
                continue;
            }
            nbWires++;
        }
        if (nbWires < 2)
        {
            return false;
        }
        bool isFixed = false;
        for (var wi = new Iterator(_face, false); wi.More(); wi.Next())
        {
            if (wi.Value().Type != ShapeType.Wire
                || (wi.Value().Orientation != Forward && wi.Value().Orientation != Reversed))
            {
                b.Add(face, wi.Value());
                continue;
            }
            Wire wire = ShapeCasts.Wire(wi.Value());
            var sewd = new ExtendWireData(wire);
            if (sewd.EdgeCount == 2)
            {
                Edge e1 = sewd.Edge(1);
                Edge e2 = sewd.Edge(2);
                e1.Orient(Forward);
                e2.Orient(Forward);
                if (!(e1 == e2))
                {
                    b.Add(face, wire);
                }
                else
                {
                    isFixed = true;
                }
            }
            else
            {
                b.Add(face, wire);
            }
        }
        if (isFixed)
        {
            face.Orient(ori);
            Context()?.Replace(_face, face);
            _face = face;
        }

        return isFixed;
    }

    /// <summary>
    /// Splits the face into several faces, one per outer wire of <paramref name="mapWires"/> (filled by
    /// <see cref="FixOrientation(Dictionary{Shape, NCollection_List{Shape}})"/>) with the wires
    /// inside it. Returns true if done; the result is then a compound (<see cref="Result"/>).
    /// </summary>
    public bool FixSplitFace(Dictionary<Shape, NCollection_List<Shape>> mapWires)
    {
        using WorkingOverlay.Scope work = EnterWork();
        return FixSplitFaceCore(mapWires);
    }

    private bool FixSplitFaceCore(Dictionary<Shape, NCollection_List<Shape>> mapWires)
    {
        var b = new BrepBuilder();
        var faces = new NCollection_Sequence<Shape>();
        Shape s = _face;
        if (Context() is not null)
        {
            s = Context()!.InternalApply(_face);
        }
        int nbWires = 0, nbWiresNew = 0, nbEdges;
        for (var iter = new Iterator(s, false); iter.More(); iter.Next())
        {
            Shape shape = iter.Value();
            if (shape.Type != ShapeType.Wire
                || (shape.Orientation != Forward && shape.Orientation != Reversed))
            {
                continue;
            }
            Wire wire = ShapeCasts.Wire(shape);
            nbWires++;
            if (mapWires.ContainsKey(wire))
            {
                // if wire not closed --> stop split and return false
                var sewd = new ExtendWireData(wire);
                nbEdges = sewd.EdgeCount;
                if (nbEdges == 0)
                {
                    continue;
                }
                //
                Edge e1 = sewd.Edge(1);
                Edge e2 = sewd.Edge(nbEdges);
                var sae = new EdgeAnalysis();
                Vertex v1 = sae.FirstVertex(e1);
                Vertex v2 = sae.LastVertex(e2);
                if (!v1.IsSame(v2))
                {
                    return false;
                }
                // create face
                Shape emptyCopied = s.EmptyCopied();
                Face tmpFace = ShapeCasts.Face(emptyCopied);
                tmpFace.Orient(Forward);
                b.Add(tmpFace, wire);
                nbWiresNew++;
                NCollection_List<Shape> intWires = mapWires.GetValue(wire);
                var liter = new NCollection_List<Shape>.Iterator(intWires);
                for (; liter.More(); liter.Next())
                {
                    Shape shapeEmptyCopied = tmpFace.EmptyCopied();
                    Face face = ShapeCasts.Face(shapeEmptyCopied);
                    face.Orient(Forward);
                    b.Add(face, liter.Value());
                    var clas = new FClass2d(face, Kerf.Engine.Foundation.Precision.PConfusion());
                    Containment staout = clas.PerformInfinitePoint();
                    if (staout == In)
                    {
                        b.Add(tmpFace, liter.Value());
                    }
                    else
                    {
                        b.Add(tmpFace, liter.Value().Reversed());
                    }
                    nbWiresNew++;
                }
                if (!_fwd)
                {
                    tmpFace.Orient(Reversed);
                }
                faces.Append(tmpFace);
            }
        }

        if (nbWires != nbWiresNew)
        {
            return false;
        }

        if (faces.Length() > 1)
        {
            var comp = new Compound();
            b.MakeCompound(ref comp);
            for (int i = 1; i <= faces.Length(); i++)
            {
                b.Add(comp, faces[i]);
            }
            _result = comp;

            Context()?.Replace(_face, _result);

            for (var exp = new Explorer(_result, global::Kerf.Topology.ShapeType.Face); exp.More(); exp.Next())
            {
                _face = ShapeCasts.Face(exp.Current());
                BrepTools.Update(_face);
            }
            return true;
        }

        return false;
    }

    // Checks whether the passed wire makes up a periodic loop on passed conical surface
    private static bool IsPeriodicConicalLoop(ConicalSurface? surf, in Wire wire, double tolerance,
                                              ref double minU, ref double maxU, ref double minV,
                                              ref double maxV, ref bool isUDecrease)
    {
        if (surf is null)
        {
            return false;
        }

        var sae = new EdgeAnalysis();
        var loc = new Placement();

        double cumulDeltaU = 0.0, cumulDeltaUAbs = 0.0;
        double aMinU = RealLast();
        double aMinV = aMinU;
        double aMaxU = -aMinU;
        double aMaxV = aMaxU;

        // Iterate over the edges to check whether the wire is periodic on conical surface
        var wireIter = new Iterator(wire, false);
        for (; wireIter.More(); wireIter.Next())
        {
            Edge currentEdge = ShapeCasts.Edge(wireIter.Value());

            sae.PCurve(currentEdge, surf, loc, out Curve2D? c2d, out double pFirst, out double pLast, true);

            if (c2d is null)
            {
                return false;
            }

            Point2D uvFirst = c2d.Value(pFirst), uvLast = c2d.Value(pLast);

            double uFirst = uvFirst.X, uLast = uvLast.X;
            double vFirst = uvFirst.Y, vLast = uvLast.Y;

            double curMaxU = Max(uFirst, uLast), curMinU = Min(uFirst, uLast);
            double curMaxV = Max(vFirst, vLast), curMinV = Min(vFirst, vLast);

            if (curMinU < aMinU)
            {
                aMinU = curMinU;
            }
            if (curMaxU > aMaxU)
            {
                aMaxU = curMaxU;
            }
            if (curMinV < aMinV)
            {
                aMinV = curMinV;
            }
            if (curMaxV > aMaxV)
            {
                aMaxV = curMaxV;
            }

            double deltaU = uLast - uFirst;

            cumulDeltaU += deltaU;
            cumulDeltaUAbs += Math.Abs(deltaU);
        }

        minU     = aMinU;
        maxU     = aMaxU;
        minV     = aMinV;
        maxV     = aMaxV;
        isUDecrease = (cumulDeltaU < 0);

        bool is2PIDelta   = Math.Abs(cumulDeltaUAbs - 2 * M_PI) <= tolerance;
        bool isAroundApex = Math.Abs(maxU - minU) > 2 * M_PI - tolerance;

        return is2PIDelta && isAroundApex;
    }

    /// <summary>
    /// Adds the degenerated edge at the apex of a cone to a face whose single wire goes around the cone by a period.
    /// Returns true if done.
    /// </summary>
    public bool FixPeriodicDegenerated()
    {
        using WorkingOverlay.Scope work = EnterWork();
        return FixPeriodicDegeneratedCore();
    }

    private bool FixPeriodicDegeneratedCore()
    {
        /* =====================
         *  Prepare fix routine
         * ===================== */

        if (Context() is not null)
        {
            Shape sh = Context()!.InternalApply(_face);
            _face = ShapeCasts.Face(sh);
        }

        /* ================================================
         *  Check if fix can be applied on the passed face
         * ================================================ */

        // Collect all wires owned by the face
        var wireSeq = new NCollection_Sequence<Shape>();
        for (var wireIt = new Iterator(_face, false); wireIt.More(); wireIt.Next())
        {
            Shape subSh = wireIt.Value();
            if (subSh.Type != ShapeType.Wire
                || (subSh.Orientation != Forward && subSh.Orientation != Reversed))
            {
                continue;
            }

            wireSeq.Append(wireIt.Value());
        }

        // Get number of wires and surface
        int nbWires = wireSeq.Length();
        Surface? surface = Tool.Surface(_face);

        // Only single wires on conical surfaces are checked
        if (nbWires != 1 || surface is null || surface.GetType() != typeof(ConicalSurface))
        {
            return false;
        }

        // Get the single wire
        Wire soleWire = ShapeCasts.Wire(wireSeq.Value(1));

        // Check whether this wire is belting the conical surface by period
        var coneSurf = (ConicalSurface)surface;
        double minLoopU = 0.0, maxLoopU = 0.0, minLoopV = 0.0, maxLoopV = 0.0;
        bool isUDecrease = false;

        bool isConicLoop = IsPeriodicConicalLoop(coneSurf, soleWire, Precision(), ref minLoopU, ref maxLoopU,
                                                 ref minLoopV, ref maxLoopV, ref isUDecrease);

        if (!isConicLoop)
        {
            return false;
        }

        /* ===============
         *  Retrieve apex
         * =============== */

        // Get base circle of the conical surface (the circle it was built from)
        Curve3D coneBaseCrv = coneSurf.VIso(0.0);
        var coneBaseCirc = (CircleCurve3D)coneBaseCrv;

        // Retrieve conical props
        double coneBaseR = coneBaseCirc.Radius;
        double semiAngle = coneSurf.SemiAngle;

        if (Math.Abs(semiAngle) <= Kerf.Engine.Foundation.Precision.Confusion())
        {
            return false; // Bad surface
        }

        // Find the V parameter of the apex
        double coneBaseH = coneBaseR / Math.Sin(semiAngle);
        double apexV    = -coneBaseH;

        // Get apex vertex
        Vertex apex = new MakeVertex(coneSurf.Apex).InternalVertex();

        // ====================================
        //  Build degenerated edge in the apex
        // ====================================

        var apexEdge = new Edge();
        var builder = new BrepBuilder();
        builder.MakeEdge(ref apexEdge);

        // Check if positional relationship between the initial wire and apex
        // line in 2D is going to be consistent
        if (Math.Abs(apexV - minLoopV) <= Precision() || Math.Abs(apexV - maxLoopV) <= Precision()
            || (apexV < maxLoopV && apexV > minLoopV))
        {
            return false;
        }

        LineCurve2D? apexCurve2d = null;

        // Apex curve below the wire
        if (apexV < minLoopV)
        {
            apexCurve2d = new LineCurve2D(new Point2D(minLoopU, apexV), new Direction2D(Direction2D.D.X));
            if (!isUDecrease)
            {
                soleWire.Reverse();
            }
        }

        // Apex curve above the wire
        if (apexV > maxLoopV)
        {
            apexCurve2d = new LineCurve2D(new Point2D(maxLoopU, apexV), new Direction2D(Direction2D.D.NX));
            if (isUDecrease)
            {
                soleWire.Reverse();
            }
        }

        // Create degenerated edge & wire for apex
        builder.UpdateEdge(apexEdge, apexCurve2d!, _face, Precision());
        builder.Add(apexEdge, apex);
        builder.Add(apexEdge, apex.Reversed());
        builder.Degenerated(apexEdge, true);
        builder.Range(apexEdge, 0, Math.Abs(maxLoopU - minLoopU));
        Wire apexWire = new MakeWire(apexEdge).InternalWire();

        // ===============================================================
        //  Finalize the fix building new face and setting up the results
        // ===============================================================

        // Collect the resulting set of wires
        var newWireSeq = new NCollection_Sequence<Shape>();
        newWireSeq.Append(soleWire);
        newWireSeq.Append(apexWire);

        // Assemble new face
        Face newFace = ShapeCasts.Face(_face.EmptyCopied());
        newFace.Orient(Forward);
        var faceBuilder = new BrepBuilder();
        for (int i = 1; i <= newWireSeq.Length(); i++)
        {
            Wire newWire = ShapeCasts.Wire(newWireSeq.Value(i));
            faceBuilder.Add(newFace, newWire);
        }
        newFace.Orient(_face.Orientation);

        // Adjust the resulting state of the healing tool
        _result = newFace;
        Context()?.Replace(_face, _result);

        return true;
    }

    /// <summary>Returns the status of the last <see cref="Perform"/> (see there for the DONE bits).</summary>
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>Returns the tool fixing the wires.</summary>
    public WireFixer FixWireTool() => _fixWire;
}
