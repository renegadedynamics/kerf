// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Intersection.TwoD;
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
/// Fixes a wire: the order of its edges, small edges, the connection of adjacent edges, the edges' curves (pcurves,
/// 3d curves, seams, shifted pcurves, same parameter, vertex tolerances), degenerated edges, self-intersections,
/// lacking edges, notched edges, tails and gaps in 3d and 2d. The wire may be on a face (the 2d fixes need one).
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). Load a wire (with <see cref="Init(in Wire, in Face, double)"/> or
/// <see cref="Load(in Wire)"/>), set its face and precision, then call <see cref="Perform"/> or the fixes one by
/// one; each sets its own status (<c>Status...</c>), and the advanced fixes (by edge number) share
/// <see cref="LastFixStatus"/>. The modes (<c>...Mode()</c>) are three-position flags: -1 is the default, 0 off, 1 on.
/// </para>
/// <para>
/// The fixes edit the edges of the loaded wire in place, as OCCT's. A loaded wire holding frozen shapes (any wire a
/// caller holds) is copied at the first fix (<see cref="CopyAtEntry"/>, decision 583), so the caller's wire
/// never changes; <see cref="Wire"/> and <see cref="WireAPIMake"/> give back the caller's own edges and vertices
/// wherever the fixes left them unchanged, record in the context the ones changed in place, and freeze what they
/// return. Set the context before the first fix. After a public result, a further fix copies again. The edges of
/// <see cref="WireData"/> are the working copies between the first fix and the result.
/// </para>
/// </remarks>
public partial class WireFixer : RootFixer
{
    protected EdgeFixer      _fixEdge;
    protected WireAnalysis _analyzer;
    protected bool               _geomMode;
    protected bool               _topoMode;
    protected bool               _closedMode;
    protected bool               _preference2d;
    protected bool               _fixGapsByRanges;
    protected int                _fixReversed2dMode;
    protected int                _fixRemovePCurveMode;
    protected int                _fixAddPCurveMode;
    protected int                _fixRemoveCurve3dMode;
    protected int                _fixAddCurve3dMode;
    protected int                _fixSeamMode;
    protected int                _fixShiftedMode;
    protected int                _fixSameParameterMode;
    protected int                _fixVertexToleranceMode;
    protected int                _fixNotchedEdgesMode;
    protected int                _fixSelfIntersectingEdgeMode;
    protected int                _fixIntersectingEdgesMode;
    protected int                _fixNonAdjacentIntersectingEdgesMode;
    protected int                _fixTailMode;
    protected int                _removeLoopMode;
    protected int                _fixReorderMode;
    protected int                _fixSmallMode;
    protected int                _fixConnectedMode;
    protected int                _fixEdgeCurvesMode;
    protected int                _fixDegeneratedMode;
    protected int                _fixSelfIntersectionMode;
    protected int                _fixLackingMode;
    protected int                _fixGaps3dMode;
    protected int                _fixGaps2dMode;
    protected int                _lastFixStatus;
    protected int                _statusReorder;
    protected int                _statusSmall;
    protected int                _statusConnected;
    protected int                _statusEdgeCurves;
    protected int                _statusDegenerated;
    protected int                _statusClosed;
    protected int                _statusSelfIntersection;
    protected int                _statusLacking;
    protected int                _statusGaps3d;
    protected int                _statusGaps2d;
    protected bool               _statusRemovedSegment;
    protected int                _statusNotches;
    protected int                _statusFixTails;
    protected double             _maxTailAngleSine;
    protected double             _maxTailWidth;

    // ---- ShapeFix_Wire.lxx ----

    /// <summary>Sets the face of the wire.</summary>
    public void SetFace(in Face face)
    {
        _analyzer.SetFace(face);
    }

    /// <summary>Sets the face of the wire with an analyser of its surface.</summary>
    public void SetFace(in Face face, SurfaceAnalysis? surfaceAnalysis)
    {
        _analyzer.SetFace(face, surfaceAnalysis);
    }

    /// <summary>Sets the surface analyser (and through it the surface) of the wire.</summary>
    public void SetSurface(SurfaceAnalysis? surfaceAnalysis)
    {
        _analyzer.SetSurface(surfaceAnalysis);
    }

    /// <summary>Sets the surface of the wire.</summary>
    public void SetSurface(Surface? surf)
    {
        _analyzer.SetSurface(surf);
    }

    /// <summary>Sets the surface of the wire with a location.</summary>
    public void SetSurface(Surface? surf, in Placement loc)
    {
        _analyzer.SetSurface(surf, loc);
    }

    /// <summary>True if a wire is loaded.</summary>
    public bool IsLoaded => _analyzer.IsLoaded;

    /// <summary>True if a wire and a face are loaded.</summary>
    public bool IsReady => _analyzer.IsReady;

    /// <summary>Returns the analyser of the wire.</summary>
    public WireAnalysis Analyzer() => _analyzer;

    /// <summary>
    /// Returns the working wire data: the loaded edges, which the fixes edit (between the first fix and a public
    /// result, copies of a frozen wire's edges).
    /// </summary>
    public ExtendWireData? WireData() => _analyzer.WireData();

    /// <summary>Returns the face of the wire.</summary>
    public Face Face() => _analyzer.Face();

    /// <summary>The mode allowing the fixes to modify the topology (default false).</summary>
    public ref bool ModifyTopologyMode() => ref _topoMode;

    /// <summary>The mode allowing the fixes to modify the geometry (default true).</summary>
    public ref bool ModifyGeometryMode() => ref _geomMode;

    /// <summary>The mode for removing loops (-1 default, 0 keep, 1 remove).</summary>
    public ref int ModifyRemoveLoopMode() => ref _removeLoopMode;

    /// <summary>The mode telling that the wire is to be closed (default true).</summary>
    public ref bool ClosedWireMode() => ref _closedMode;

    /// <summary>The mode preferring pcurves to 3d curves for gaps (default true).</summary>
    public ref bool PreferencePCurveMode() => ref _preference2d;

    /// <summary>The mode fixing gaps by ranges (default false).</summary>
    public ref bool FixGapsByRangesMode() => ref _fixGapsByRanges;

    public ref int FixReorderMode() => ref _fixReorderMode;

    public ref int FixSmallMode() => ref _fixSmallMode;

    public ref int FixConnectedMode() => ref _fixConnectedMode;

    public ref int FixEdgeCurvesMode() => ref _fixEdgeCurvesMode;

    public ref int FixDegeneratedMode() => ref _fixDegeneratedMode;

    public ref int FixReversed2dMode() => ref _fixReversed2dMode;

    public ref int FixRemovePCurveMode() => ref _fixRemovePCurveMode;

    public ref int FixRemoveCurve3dMode() => ref _fixRemoveCurve3dMode;

    public ref int FixAddPCurveMode() => ref _fixAddPCurveMode;

    public ref int FixAddCurve3dMode() => ref _fixAddCurve3dMode;

    public ref int FixSeamMode() => ref _fixSeamMode;

    public ref int FixShiftedMode() => ref _fixShiftedMode;

    public ref int FixSameParameterMode() => ref _fixSameParameterMode;

    public ref int FixVertexToleranceMode() => ref _fixVertexToleranceMode;

    public ref int FixLackingMode() => ref _fixLackingMode;

    public ref int FixSelfIntersectionMode() => ref _fixSelfIntersectionMode;

    public ref int FixGaps3dMode() => ref _fixGaps3dMode;

    public ref int FixGaps2dMode() => ref _fixGaps2dMode;

    public ref int FixNotchedEdgesMode() => ref _fixNotchedEdgesMode;

    public ref int FixSelfIntersectingEdgeMode() => ref _fixSelfIntersectingEdgeMode;

    public ref int FixIntersectingEdgesMode() => ref _fixIntersectingEdgesMode;

    public ref int FixNonAdjacentIntersectingEdgesMode() => ref _fixNonAdjacentIntersectingEdgesMode;

    public ref int FixTailMode() => ref _fixTailMode;

    public bool StatusReorder(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusReorder, status);

    public bool StatusSmall(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusSmall, status);

    public bool StatusConnected(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusConnected, status);

    public bool StatusEdgeCurves(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusEdgeCurves, status);

    public bool StatusDegenerated(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusDegenerated, status);

    public bool StatusLacking(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusLacking, status);

    public bool StatusSelfIntersection(ExtendStatus status) =>
        global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusSelfIntersection, status);

    public bool StatusGaps3d(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusGaps3d, status);

    public bool StatusGaps2d(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusGaps2d, status);

    public bool StatusClosed(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusClosed, status);

    public bool StatusNotches(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusNotches, status);

    public bool StatusFixTails(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_statusFixTails, status);

    /// <summary>The status of the last advanced fix (by edge number).</summary>
    public bool LastFixStatus(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_lastFixStatus, status);

    /// <summary>Returns the tool fixing the edges.</summary>
    public EdgeFixer FixEdgeTool() => _fixEdge;

    /// <summary>True if a segment was removed by the last fix of self-intersection.</summary>
    public bool StatusRemovedSegment() => _statusRemovedSegment;

    // ---- ShapeFix_Wire.cxx ----

    // #######################################################################
    //   Constructors, initializations, modes, querying
    // #######################################################################

    /// <summary>Empty constructor: default modes, no wire loaded.</summary>
    public WireFixer()
    {
        _maxTailAngleSine = 0;
        _maxTailWidth     = -1;
        _fixEdge          = new EdgeFixer();
        _analyzer         = new WireAnalysis();
        ClearModes();
        ClearStatuses();
        _statusRemovedSegment = false;
    }

    /// <summary>Creates the tool and loads a wire on a face with a precision.</summary>
    public WireFixer(in Wire wire, in Face face, double prec)
    {
        _maxTailAngleSine = 0;
        _maxTailWidth     = -1;
        _fixEdge          = new EdgeFixer();
        _analyzer         = new WireAnalysis();
        ClearModes();
        SetMaxTolerance(prec);
        _statusRemovedSegment = false;
        Init(wire, face, prec);
    }

    /// <summary>Sets the working precision (also of the analyser).</summary>
    public override void SetPrecision(double prec)
    {
        base.SetPrecision(prec);
        _analyzer.SetPrecision(prec);
    }

    /// <summary>Sets the maximal angle of tails (FixTails).</summary>
    public void SetMaxTailAngle(double maxTailAngle)
    {
        _maxTailAngleSine = Math.Sin(maxTailAngle);
        _maxTailAngleSine = (_maxTailAngleSine >= 0) ? _maxTailAngleSine : 0;
    }

    /// <summary>Sets the maximal width of tails (FixTails).</summary>
    public void SetMaxTailWidth(double maxTailWidth)
    {
        _maxTailWidth = maxTailWidth;
    }

    /// <summary>Sets all the modes to their defaults.</summary>
    public void ClearModes()
    {
        _topoMode        = false;
        _geomMode        = true;
        _closedMode      = true;
        _preference2d    = true;
        _fixGapsByRanges = false;

        _removeLoopMode = -1;

        _fixReversed2dMode      = -1;
        _fixRemovePCurveMode    = -1;
        _fixRemoveCurve3dMode   = -1;
        _fixAddPCurveMode       = -1;
        _fixAddCurve3dMode      = -1;
        _fixSeamMode            = -1;
        _fixShiftedMode         = -1;
        _fixSameParameterMode   = -1;
        _fixVertexToleranceMode = -1;

        _fixNotchedEdgesMode                 = -1;
        _fixSelfIntersectingEdgeMode         = -1;
        _fixIntersectingEdgesMode            = -1;
        _fixNonAdjacentIntersectingEdgesMode = -1;
        _fixTailMode                         = 0;

        _fixReorderMode          = -1;
        _fixSmallMode            = -1;
        _fixConnectedMode        = -1;
        _fixEdgeCurvesMode       = -1;
        _fixDegeneratedMode      = -1;
        _fixSelfIntersectionMode = -1;
        _fixLackingMode          = -1;
        _fixGaps3dMode           = -1;
        _fixGaps2dMode           = -1;
    }

    /// <summary>Clears all the statuses.</summary>
    public void ClearStatuses()
    {
        int emptyStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);

        _lastFixStatus = emptyStatus;

        _statusReorder          = emptyStatus;
        _statusSmall            = emptyStatus;
        _statusConnected        = emptyStatus;
        _statusEdgeCurves       = emptyStatus;
        _statusDegenerated      = emptyStatus;
        _statusSelfIntersection = emptyStatus;
        _statusLacking          = emptyStatus;
        _statusGaps3d           = emptyStatus; // szv#9:S4244:19Aug99 new method introduced
        _statusGaps2d           = emptyStatus; // szv#9:S4244:19Aug99 new method introduced
        _statusClosed           = emptyStatus;
        _statusNotches          = emptyStatus;
        _statusFixTails         = emptyStatus;
    }

    /// <summary>Loads a wire, sets its face and the precision.</summary>
    public void Init(in Wire wire, in Face face, double prec)
    {
        Load(wire);
        SetFace(face);
        SetPrecision(prec);
    }

    /// <summary>Takes the analyser (with its wire, face and precision) as the tool's own.</summary>
    public void Init(WireAnalysis saw)
    {
        ClearStatuses();
        _analyzer = saw;
        _shape.Nullify();
        //  SetPrecision ( saw.Precision() );
        _toEnter = true; // OCCT.NET: frozen
    }

    /// <summary>Loads a wire (the context, if any, applied to it first).</summary>
    public void Load(in Wire wire)
    {
        ClearStatuses();

        Wire w = wire;
        if (Context() is not null)
        {
            Shape s = Context()!.InternalApply(wire);
            w = ShapeCasts.Wire(s);
        }

        _analyzer.Load(w);
        _shape   = wire;
        _toEnter = true; // OCCT.NET: frozen
    }

    /// <summary>Loads a wire data object (the context, if any, applied to its edges).</summary>
    public void Load(ExtendWireData sbwd)
    {
        ClearStatuses();
        _analyzer.Load(sbwd);
        if (Context() is not null)
        {
            UpdateWire();
        }
        _shape.Nullify();
        _toEnter = true; // OCCT.NET: frozen
    }

    /// <summary>Returns the number of edges in the working wire.</summary>
    public int NbEdges()
    {
        ExtendWireData? sbwd = _analyzer.WireData();
        return sbwd is null ? 0 : sbwd.EdgeCount;
    }

    // #######################################################################
    //   Fixing methods of API level
    // #######################################################################

    /// <summary>
    /// Performs all the available fixes. A fix turned on or off explicitly by its mode is called or not accordingly;
    /// with the default mode it is called depending on the situation (some are skipped or limited if the order of the
    /// edges isn't OK).
    /// </summary>
    public bool Perform(ProgressRange progress = default)
    {
        EnterWork(); // OCCT.NET: frozen
        ClearStatuses();
        if (!IsLoaded)
        {
            return false;
        }

        if (Context() is not null)
        {
            _fixEdge.SetContext(Context());
        }

        bool Fixed = false;

        // FixReorder is first, because as a rule wire is required to be ordered
        // We shall analyze the order of edges in the wire and set appropriate
        // status even if FixReorder should not be called (if it is forbidden)

        var  sawo      = new WireOrderAnalysis();
        bool reorderOK = !_analyzer.CheckOrder(sawo, _closedMode);
        if (NeedFix(_fixReorderMode, !reorderOK))
        {
            if (FixReorder())
            {
                Fixed = true;
            }
            reorderOK = !StatusReorder(ExtendStatus.Fail);
        }

        if (progress.UserBreak())
        {
            return false;
        }

        // FixSmall is allowed to change topology only if mode is set and FixReorder
        // did not failed
        if (NeedFix(_fixSmallMode, _topoMode))
        {
            if (FixSmall(!_topoMode || !reorderOK, MinTolerance()) != 0)
            {
                Fixed = true;
                // retry reorder if necessary (after FixSmall it can work better)
                if (NeedFix(_fixReorderMode, !reorderOK))
                {
                    FixReorder();
                    reorderOK = !StatusReorder(ExtendStatus.Fail);
                }
            }
        }

        if (progress.UserBreak())
        {
            return false;
        }

        if (NeedFix(_fixConnectedMode, reorderOK))
        {
            if (FixConnected())
            {
                Fixed = true;
            }
        }

        if (progress.UserBreak())
        {
            return false;
        }

        if (NeedFix(_fixEdgeCurvesMode))
        {
            int savFixShiftedMode = _fixShiftedMode;
            // turn out FixShifted if reorder not done
            if (_fixShiftedMode == -1 && !reorderOK)
            {
                _fixShiftedMode = 0;
            }
            if (FixEdgeCurves())
            {
                Fixed = true;
            }
            _fixShiftedMode = savFixShiftedMode;
        }

        if (progress.UserBreak())
        {
            return false;
        }

        if (NeedFix(_fixDegeneratedMode))
        {
            if (FixDegenerated())
            {
                Fixed = true; // ?? if ! ReorderOK ??
            }
        }

        if (progress.UserBreak())
        {
            return false;
        }

        // pdn - temporary to test
        if (_fixTailMode <= 0 && NeedFix(_fixNotchedEdgesMode, reorderOK))
        {
            Fixed |= FixNotchedEdges();
            if (Fixed)
            {
                FixShifted(); // skl 07.03.2002 for OCC180
            }
        }

        if (progress.UserBreak())
        {
            return false;
        }

        if (_fixTailMode != 0)
        {
            if (FixTails())
            {
                Fixed = true;
                FixShifted();
            }
        }

        if (progress.UserBreak())
        {
            return false;
        }

        if (NeedFix(_fixSelfIntersectionMode, _closedMode))
        {
            int savFixIntersectingEdgesMode = _fixIntersectingEdgesMode;
            // switch off FixIntEdges if reorder not done
            if (_fixIntersectingEdgesMode == -1 && !reorderOK)
            {
                _fixIntersectingEdgesMode = 0;
            }
            if (FixSelfIntersection())
            {
                Fixed = true;
            }
            FixReorder();
            _fixIntersectingEdgesMode = savFixIntersectingEdgesMode;
        }

        if (progress.UserBreak())
        {
            return false;
        }

        if (NeedFix(_fixLackingMode, reorderOK))
        {
            if (FixLacking())
            {
                Fixed = true;
            }
        }

        if (progress.UserBreak())
        {
            return false;
        }

        // TEMPORARILY without special mode !!!
        ExtendWireData sbwd = WireData()!;
        for (int iedge = 1; iedge <= sbwd.EdgeCount; iedge++)
        {
            if (_fixEdge.FixVertexTolerance(sbwd.Edge(iedge), Face()))
            {
                Fixed = true;
            }
        }

        if (progress.UserBreak())
        {
            return false;
        }

        if (Context() is not null)
        {
            UpdateWire();
        }

        return Fixed;
    }

    /// <summary>
    /// Analyses the order of the edges with ShapeAnalysis_WireOrder and reorders them; in Both mode
    /// (<paramref name="modeBoth"/>, for bi-periodic surfaces) the order uses 3d and 2d data.
    /// </summary>
    public bool FixReorder(bool modeBoth = false)
    {
        EnterWork(); // OCCT.NET: frozen
        _statusReorder = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsLoaded)
        {
            return false;
        }

        // fix in Both mode for bi-periodic surface
        var sawo = new WireOrderAnalysis();
        if (_analyzer.Surface() is not null && _analyzer.Surface()!.Surface()!.IsUPeriodic
            && _analyzer.Surface()!.Surface()!.IsVPeriodic && modeBoth)
        {
            _analyzer.CheckOrder(sawo, _closedMode, true, true);
        }
        else
        {
            _analyzer.CheckOrder(sawo, _closedMode, true, false);
        }

        FixReorder(sawo);

        if (LastFixStatus(ExtendStatus.Fail))
        {
            _statusReorder |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(
                LastFixStatus(ExtendStatus.FAIL1) ? ExtendStatus.FAIL1 : ExtendStatus.FAIL2);
        }
        if (!LastFixStatus(ExtendStatus.Done))
        {
            return false;
        }

        _statusReorder |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        if (sawo.Status() == 2 || sawo.Status() == -2)
        {
            _statusReorder |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }
        if (sawo.Status() < 0)
        {
            _statusReorder |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
        }
        if (sawo.Status() == 3)
        {
            // only shifted
            _statusReorder |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5);
        }
        return true;
    }

    /// <summary>
    /// Applies FixSmall(num) to all the edges, from the last; returns 1 if some small edge was fixed (OCCT returns its
    /// bool as an int).
    /// </summary>
    public int FixSmall(bool lockvtx, double precsmall = 0.0)
    {
        EnterWork(); // OCCT.NET: frozen
        _statusSmall = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsLoaded)
        {
            return 0;
        }

        for (int i = NbEdges(); i > 0; i--)
        {
            FixSmall(i, lockvtx, precsmall);
            _statusSmall |= _lastFixStatus;
        }

        return StatusSmall(ExtendStatus.Done) ? 1 : 0;
    }

    /// <summary>Applies FixConnected(num) to all the edges (from the last), connecting them by their vertices.</summary>
    public bool FixConnected(double prec = -1.0)
    {
        EnterWork(); // OCCT.NET: frozen
        _statusConnected = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsLoaded)
        {
            return false;
        }

        int                  stop     = (_closedMode ? 0 : 1);
        ExtendWireData wireSBWD = WireData()!;
        for (int i = NbEdges(); i > stop; i--)
        {
            FixConnected(i, prec, false);
            _statusConnected |= _lastFixStatus;
            // Refresh the edge that the next iteration will analyze as n1.
            if (Context() is null || i - 1 <= stop)
            {
                continue;
            }
            int n1Next = (i - 1 > 1) ? i - 2 : wireSBWD.EdgeCount;
            if (n1Next == i || n1Next == i - 1)
            {
                continue;
            }
            Edge  ePrev   = wireSBWD.Edge(n1Next);
            Shape refresh = Context()!.InternalApply(ePrev);
            if (refresh.IsNull || refresh.IsSame(ePrev) || refresh.ShapeType() != ShapeType.Edge)
            {
                continue;
            }
            wireSBWD.Set(ShapeCasts.Edge(refresh), n1Next);
        }

        if (Context() is not null)
        {
            UpdateWire();
        }

        return StatusConnected(ExtendStatus.Done);
    }

    /// <summary>
    /// Fixes the curves of the edges: reversed 2d curves, missing or invalid pcurves and 3d curves, seams, shifted
    /// pcurves, same parameter and the vertices' tolerances, as the modes allow.
    /// </summary>
    public bool FixEdgeCurves()
    {
        EnterWork(); // OCCT.NET: frozen
        _statusEdgeCurves = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsLoaded)
        {
            return false;
        }
        bool isReady = IsReady;

        ExtendWireData sbwd = WireData()!;
        int                  i, nb = sbwd.EdgeCount;
        Face          face          = Face();
        EdgeFixer        theAdvFixEdge = _fixEdge;
        if (theAdvFixEdge is null)
        {
            _fixReversed2dMode = 0;
        }

        // fix revesred 2d / 3d curves
        if (isReady && NeedFix(_fixReversed2dMode))
        {
            for (i = 1; i <= nb; i++)
            {
                theAdvFixEdge!.FixReversed2d(sbwd.Edge(i), face);
                if (theAdvFixEdge.Status(ExtendStatus.Done))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                }
                if (theAdvFixEdge.Status(ExtendStatus.Fail))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                }
            }
        }

        // add / remove pcurve
        if (isReady && NeedFix(_fixRemovePCurveMode, false))
        {
            for (i = 1; i <= nb; i++)
            {
                _fixEdge.FixRemovePCurve(sbwd.Edge(i), face);
                if (_fixEdge.Status(ExtendStatus.Done))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                }
                if (_fixEdge.Status(ExtendStatus.Fail))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                }
            }
        }

        if (isReady && NeedFix(_fixAddPCurveMode))
        {
            int overdegen = 0; //: c0
            for (i = 1; i <= nb; i++)
            {
                _fixEdge.FixAddPCurve(sbwd.Edge(i), face, sbwd.IsSeam(i), _analyzer.Surface()!, Precision());
                if (_fixEdge.Status(ExtendStatus.Done))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                }
                if (_fixEdge.Status(ExtendStatus.Fail))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
                }

                // if ( !sbwd->IsSeam(i) && myFixEdge->Status ( ShapeExtend_DONE2 )
                //      && BRep_Tool::SameParameter(sbwd->Edge(i)) ) {
                if (!sbwd.IsSeam(i) && _fixEdge.Status(ExtendStatus.DONE2))
                {
                    // abv 24 Feb 00: trj3_s1-ac-214.stp #1631 etc.: try to split the edge in singularity
                    if (Context() is not null)
                    {
                        var                   sbe = new BuildEdgeBuilder();
                        Edge           e   = sbwd.Edge(i);
                        var                   sac = new CurveAnalysis();
                        Curve3D            c   = Tool.Curve(e, out double a, out double b)!;
                        SurfaceAnalysis s   = _analyzer.Surface()!;
                        int                   nbs = s.NbSingularities(MinTolerance());
                        var                   gac = new CurveAdaptor(c, a, b);
                        var                   seq = new NCollection_Sequence<double>();
                        for (int j = 1; j <= nbs; j++)
                        {
                            double   preci = 0.0;
                            Point2D pd1 = new Point2D(), pd2 = new Point2D();
                            Point3D   p3d = new Point3D();
                            double   par1 = 0.0, par2 = 0.0;
                            bool     tmpUIsoDeg = false;
                            s.Singularity(j, ref preci, ref p3d, ref pd1, ref pd2, ref par1, ref par2, ref tmpUIsoDeg);
                            if (sac.Project(gac, p3d, MinTolerance(), out Point3D pr, out double split, true)
                                < Max(preci, MinTolerance()))
                            {
                                if (split - a > Kerf.Engine.Foundation.Precision.PConfusion() && b - split > Kerf.Engine.Foundation.Precision.PConfusion())
                                {
                                    int k;
                                    for (k = 1; k <= seq.Length(); k++)
                                    {
                                        if (split < seq[k] - Kerf.Engine.Foundation.Precision.PConfusion())
                                        {
                                            seq.InsertBefore(k, split);
                                            break;
                                        }
                                        else if (split < seq[k] + Kerf.Engine.Foundation.Precision.PConfusion())
                                        {
                                            break;
                                        }
                                    }
                                    if (k > seq.Length())
                                    {
                                        seq.Append(split);
                                    }
                                }
                            }
                        }
                        if (seq.Length() > 0)
                        { // supposed that edge is SP
                            bool isFwd = (e.Orientation() == Forward);
                            e.Orientation(Forward);

                            // if( BRep_Tool::SameParameter(sbwd->Edge(i)) )
                            //   sbe.RemovePCurve ( E, face );

                            // 10.04.2003 skl for using trimmed lines as pcurves
                            var sae = new EdgeAnalysis();
                            if (Tool.SameParameter(sbwd.Edge(i)))
                            {
                                sbe.RemovePCurve(e, face);
                            }
                            else
                            {
                                if (sae.HasPCurve(e, face))
                                {
                                    if (sae.PCurve(e, face, out Curve2D? C2d, out double fp2d, out double lp2d))
                                    {
                                        if (C2d is not TrimmedCurve2D)
                                        {
                                            sbe.RemovePCurve(e, face);
                                        }
                                    }
                                }
                            }

                            //    myFixEdge->FixSameParameter ( E ); // to ensure SameRange & SP
                            var           B = new BrepBuilder();
                            Vertex v1, v2, v = new Vertex();
                            // ShapeAnalysis_Edge sae;
                            v1 = sae.FirstVertex(e);
                            v2 = sae.LastVertex(e);

                            var sw = new ExtendWireData();
                            for (int k = 0; k <= seq.Length(); k++)
                            {
                                double split = (k < seq.Length() ? seq[k + 1] : b);
                                if (k < seq.Length())
                                {
                                    B.MakeVertex(ref v, c.Value(split), Tool.Tolerance(e));
                                    // try increase tolerance before splitting
                                    double dist = Tool.Pnt(v1).Distance(Tool.Pnt(v));
                                    if (dist < Tool.Tolerance(v1) * 1.01)
                                    {
                                        B.UpdateVertex(v1, Max(dist, Tool.Tolerance(v1)));
                                        a  = split;
                                        v1 = v;
                                        continue;
                                    }
                                    else
                                    {
                                        dist = Tool.Pnt(v2).Distance(Tool.Pnt(v));
                                        if (dist < Tool.Tolerance(v2) * 1.01)
                                        {
                                            B.UpdateVertex(v, Max(dist, Tool.Tolerance(v2)));
                                            b  = split;
                                            v2 = v;
                                            continue;
                                        }
                                    }
                                }
                                else
                                {
                                    v = v2;
                                }

                                Edge edge = sbe.InternalCopyReplaceVertices(e, v1, v);
                                if (Tool.SameParameter(sbwd.Edge(i)))
                                {
                                    // TopoDS_Edge edge = sbe.CopyReplaceVertices ( E, V1, V );
                                    B.Range(edge, a, split);
                                    sw.Add(edge);
                                }
                                else
                                {
                                    // TopoDS_Edge edge = sbe.CopyReplaceVertices(sbwd->Edge(i),V1,V);
                                    var sftp = new TransferParametersAnalysis(e, face);
                                    sftp.TransferRange(edge, a, split, false);
                                    sw.Add(edge);
                                }
                                // sw->Add(edge);
                                a  = split;
                                v1 = v;
                            }
                            if (!isFwd)
                            {
                                sw.Reverse();
                                e.Orientation(Reversed);
                            }
                            Context()!.Replace(e, sw.InternalWire());
                            UpdateWire();
                            nb = sbwd.EdgeCount;
                            i--;
                            continue;
                        }
                    }

                    overdegen = i;
                }
            }

            //: c0 abv 20 Feb 98: treat case of curve going over degenerated pole and seam
            if (overdegen != 0 && _analyzer.Surface()!.IsUClosed(Precision()))
            {
                var    sbe = new BuildEdgeBuilder();
                double uRange;
                _analyzer.Surface()!.Bounds(out double suf, out double sul, out double svf, out double svl);
                uRange = (Math.Abs(sul - suf));
                Xy vec = new Xy(0, 0);
                var   sae = new EdgeAnalysis();
                int   k;
                for (k = 1; k <= nb; k++)
                {
                    if (!sae.PCurve(sbwd.Edge(k), face, out Curve2D? c2d, out double cf, out double cl, true))
                    {
                        break;
                    }
                    vec += c2d!.Value(cl).XY() - c2d.Value(cf).XY();
                }
                if (k > nb && Math.Abs(Math.Abs(vec.X()) - uRange) < 0.1 * uRange)
                {
                    sbe.RemovePCurve(sbwd.Edge(overdegen), face);
                    _fixEdge.Projector().AdjustOverDegenMode() = 0;
                    _fixEdge.FixAddPCurve(sbwd.Edge(overdegen), face, sbwd.IsSeam(overdegen), _analyzer.Surface()!,
                                           Precision());
                }
            }
        }

        // add / remove pcurve
        if (isReady && NeedFix(_fixRemoveCurve3dMode, false))
        {
            for (i = 1; i <= nb; i++)
            {
                _fixEdge.FixRemoveCurve3d(sbwd.Edge(i));
                if (_fixEdge.Status(ExtendStatus.Done))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                }
                if (_fixEdge.Status(ExtendStatus.Fail))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL4);
                }
            }
        }
        if (NeedFix(_fixAddCurve3dMode))
        {
            for (i = 1; i <= nb; i++)
            {
                _fixEdge.FixAddCurve3d(sbwd.Edge(i));
                if (_fixEdge.Status(ExtendStatus.Done))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5);
                }
                if (_fixEdge.Status(ExtendStatus.Fail))
                {
                    //: abv 29.08.01: Spatial_firex_lofting.sat: if 3d curve cannot
                    // be built because edge has no pcurves either, remove that edge
                    Tool.CurveOnSurface(sbwd.Edge(i), out Curve2D? c, out Surface? s, out Placement l,
                                             out double first, out double last);
                    if (c is null || Math.Abs(last - first) < Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        // Incomplete edge (with no pcurves or 3d curve) removed
                        SendWarning(sbwd.Edge(i), new Msg("FixWire.FixCurve3d.Removed"));
                        sbwd.Remove(i--);
                        nb--;
                        _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5);
                        if (i == nb)
                        {
                            FixClosed(Precision());
                        }
                        else
                        {
                            FixConnected(i + 1, Precision());
                        }
                    }
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL5);
                }
            }
        }

        // fix seam
        if (isReady && NeedFix(_fixSeamMode, false))
        {
            for (i = 1; i <= nb; i++)
            {
                FixSeam(i);
                if (LastFixStatus(ExtendStatus.Done))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE6);
                }
                if (LastFixStatus(ExtendStatus.Fail))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL6);
                }
            }
        }

        // fix shifted
        if (isReady && NeedFix(_fixShiftedMode))
        {
            FixShifted();
            if (LastFixStatus(ExtendStatus.Done))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE7);
            }
            if (LastFixStatus(ExtendStatus.Fail))
            {
                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL7);
            }
        }

        // fix same parameter
        if (isReady && NeedFix(_fixSameParameterMode))
        {
            for (i = 1; i <= nb; i++)
            {
                // skl 28.10.2004 for OCC6366 - check SameRange
                var        sae    = new EdgeAnalysis();
                Curve3D? tmpc3d = Tool.Curve(sbwd.Edge(i), out double First, out double Last);
                if (sae.HasPCurve(sbwd.Edge(i), face))
                {
                    if (sae.PCurve(sbwd.Edge(i), face, out Curve2D? C2d, out double fp2d, out double lp2d, false))
                    {
                        if (Math.Abs(First - fp2d) > Kerf.Engine.Foundation.Precision.PConfusion()
                            || Math.Abs(Last - lp2d) > Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            var B = new BrepBuilder();
                            B.SameRange(sbwd.Edge(i), false);
                        }
                        else if (!sae.CheckPCurveRange(First, Last, C2d!))
                        {
                            // Replace pcurve
                            Surface? s = Tool.Surface(face, out Placement l);
                            new BuildEdgeBuilder().RemovePCurve(sbwd.Edge(i), s, l);
                            _fixEdge.FixAddPCurve(sbwd.Edge(i), face, sbwd.IsSeam(i), _analyzer.Surface()!, Precision());
                            if (_fixEdge.Status(ExtendStatus.Done))
                            {
                                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                            }
                            if (_fixEdge.Status(ExtendStatus.Fail))
                            {
                                _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
                            }
                        }
                    }
                }
                _fixEdge.FixSameParameter(sbwd.Edge(i), Face());
                if (_fixEdge.Status(ExtendStatus.Done))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE8);
                }
                if (_fixEdge.Status(ExtendStatus.Fail))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL8);
                }
            }
        }

        //: abv 10.06.02: porting C40 -> dev (CC670-12608.stp): moved from Perform()
        // Update with face is needed for plane surfaces (w/o stored pcurves)
        if (NeedFix(_fixVertexToleranceMode))
        {
            for (i = 1; i <= nb; i++)
            {
                _fixEdge.FixVertexTolerance(sbwd.Edge(i), Face());
                if (_fixEdge.Status(ExtendStatus.Done))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE8);
                }
                if (_fixEdge.Status(ExtendStatus.Fail))
                {
                    _statusEdgeCurves |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL8);
                }
            }
            if (Context() is not null)
            {
                UpdateWire();
            }
        }

        return StatusEdgeCurves(ExtendStatus.Done);
    }

    /// <summary>Applies FixDegenerated(num) to all the edges, and removes duplicated degenerated edges.</summary>
    public bool FixDegenerated()
    {
        EnterWork(); // OCCT.NET: frozen
        _statusDegenerated = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        //  if ( ! myAnalyzer->Surface()->HasSingularities ( Precision() ) ) return;

        int lastcoded = -1, prevcoded = 0;
        int stop = (_closedMode ? 0 : 1);
        for (int i = NbEdges(); i > stop; i--)
        {
            FixDegenerated(i);
            _statusDegenerated |= _lastFixStatus;
            //: r0 abv 19 Mar 99: PRO7226.stp #489490: remove duplicated degenerated edges
            int coded = (LastFixStatus(ExtendStatus.DONE2) ? 1 : 0);
            if (lastcoded == -1)
            {
                lastcoded = coded;
            }
            if (coded != 0 && (prevcoded != 0 || (i == 1 && lastcoded != 0)) && NbEdges() > 1)
            {
                ExtendWireData sbwd = WireData()!;
                var                  b    = new BrepBuilder();
                sbwd.Remove(i);
                if (prevcoded == 0)
                {
                    i = NbEdges();
                }
                b.Degenerated(sbwd.Edge(i++), false);
                prevcoded = 0;
            }
            else
            {
                prevcoded = coded;
            }
        }

        return StatusDegenerated(ExtendStatus.Done);
    }

    /// <summary>
    /// Applies FixSelfIntersectingEdge and FixIntersectingEdges (adjacent and, with its mode, non-adjacent), and
    /// removes wrong edges.
    /// </summary>
    public bool FixSelfIntersection()
    {
        EnterWork(); // OCCT.NET: frozen
        _statusSelfIntersection = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        ExtendWireData sbwd = WireData()!;
        int                  nb   = sbwd.EdgeCount;

        if (NeedFix(_fixSelfIntersectingEdgeMode))
        {
            if (_removeLoopMode < 1)
            {
                for (int num = 1; num <= nb; num++)
                {
                    FixSelfIntersectingEdge(num);
                    _statusSelfIntersection |= _lastFixStatus;
                }
            }
            else if (_removeLoopMode == 1)
            {
                for (int num = 1; num <= nb; num++)
                {
                    FixSelfIntersectingEdge(num);
                    _statusSelfIntersection |= _lastFixStatus;
                    if (nb < sbwd.EdgeCount)
                    {
                        num--;
                    }
                    nb = sbwd.EdgeCount;
                }
                FixClosed(Precision());
            }
        }

        if (NeedFix(_fixIntersectingEdgesMode))
        {
            int num = (_closedMode ? 1 : 2);
            for (; nb > 1 && num <= nb; num++)
            {
                FixIntersectingEdges(num);
                if (LastFixStatus(ExtendStatus.FAIL1))
                {
                    _statusSelfIntersection |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                }
                if (LastFixStatus(ExtendStatus.FAIL2))
                {
                    _statusSelfIntersection |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                }
                if (!LastFixStatus(ExtendStatus.Done))
                {
                    continue;
                }

                if (LastFixStatus(ExtendStatus.DONE1))
                {
                    _statusSelfIntersection |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                }
                if (LastFixStatus(ExtendStatus.DONE2))
                {
                    _statusSelfIntersection |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                }
                if (LastFixStatus(ExtendStatus.DONE6))
                {
                    _statusSelfIntersection |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE6);
                }

                if (/*! myTopoMode ||*/ nb < 3)
                {
                    // #86 rln 22.03.99 sim2.igs, entity 4292: After fixing of self-intersecting
                    // BRepCheck finds one more self-intersection not found by ShapeAnalysis
                    //%15 pdn 06.04.99 repeat until fixed CTS18546-2 entity 777

                    // if the tolerance was modified we should recheck the result, if it was enough
                    if (LastFixStatus(ExtendStatus.DONE7))
                    { // num--;
                        FixIntersectingEdges(num);
                    }
                    continue;
                }

                if (LastFixStatus(ExtendStatus.DONE4))
                {
                    sbwd.Remove(num);
                }
                if (LastFixStatus(ExtendStatus.DONE3))
                {
                    sbwd.Remove(num > 1 ? num - 1 : nb + num - 1);
                }
                if (LastFixStatus(ExtendStatus.DONE4) || LastFixStatus(ExtendStatus.DONE3))
                {
                    _statusSelfIntersection |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                    num = (_closedMode ? 1 : 2);
                    nb  = sbwd.EdgeCount;
                }
                else
                {
                    // #86 rln 22.03.99
                    //%15 pdn 06.04.99 repeat until fixed CTS18546-2 entity 777
                    FixIntersectingEdges(num);
                    /*if ( LastFixStatus ( ShapeExtend_Status.ShapeExtend_DONE7 ) )*/
                    // Always revisit the fixed edge
                    // num--;
                }
            }
            if (Context() is not null)
            {
                UpdateWire();
            }
        }

        // pdn 17.03.99 S4135 to avoid regression fixing not adjacent intersection
        if (NeedFix(_fixNonAdjacentIntersectingEdgesMode))
        {
            var iTool   = new IntersectionToolFixer(Context(), Precision());
            int nbSplit = 0, nbCut = 0, nbRemoved = 0;
            if (iTool.FixSelfIntersectWire(sbwd, _analyzer.Face(), ref nbSplit, ref nbCut, ref nbRemoved))
            {
                _statusSelfIntersection |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5); // gka 06.09.04
            }
            if (nbSplit > 0 || nbRemoved > 0)
            {
                if (nbRemoved > 0)
                {
                    _statusRemovedSegment = true;
                }
                // Load(sbwd); commented by skl 29.12.2004 for OCC7624, instead this
                //             string inserted following three strings:
                _analyzer.Load(sbwd);
                if (Context() is not null)
                {
                    UpdateWire();
                }
                _shape.Nullify();
            }
            // OCCT's commented-out pairwise check of non-adjacent edges by boxes is left out.
        }

        return StatusSelfIntersection(ExtendStatus.Done);
    }

    /// <summary>Applies FixLacking(num) to all the edges (from the first, or the second on an open wire).</summary>
    public bool FixLacking(bool force = false)
    {
        EnterWork(); // OCCT.NET: frozen
        _statusLacking = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        int start = (_closedMode ? 1 : 2);
        for (int i = start; i <= NbEdges(); i++)
        {
            FixLacking(i, force);
            _statusLacking |= _lastFixStatus;
        }

        return StatusLacking(ExtendStatus.Done);
    }

    /// <summary>
    /// Fixes the closure of the wire: FixConnected, FixDegenerated and FixLacking between its last and first edges.
    /// </summary>
    public bool FixClosed(double prec = -1.0)
    {
        EnterWork(); // OCCT.NET: frozen
        _statusClosed = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsLoaded || NbEdges() < 1)
        {
            return false;
        }

        FixConnected(1, prec);
        if (LastFixStatus(ExtendStatus.Done))
        {
            _statusClosed |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }
        if (LastFixStatus(ExtendStatus.Fail))
        {
            _statusClosed |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
        }

        FixDegenerated(1);
        if (LastFixStatus(ExtendStatus.Done))
        {
            _statusClosed |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }
        if (LastFixStatus(ExtendStatus.Fail))
        {
            _statusClosed |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
        }

        FixLacking(1);
        if (LastFixStatus(ExtendStatus.Done))
        {
            _statusClosed |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
        }
        if (LastFixStatus(ExtendStatus.Fail))
        {
            _statusClosed |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
        }

        return StatusClosed(ExtendStatus.Done);
    }

    // #######################################################################
    //   Fixing methods of advanced level
    // #######################################################################

    /// <summary>Reorders the edges in the wire as the given wire order says.</summary>
    public bool FixReorder(WireOrderAnalysis wi)
    {
        EnterWork(); // OCCT.NET: frozen
        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsLoaded)
        {
            return false;
        }

        int status = wi.Status();
        if (status == 0)
        {
            return false;
        }
        if (status <= -10)
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        ExtendWireData sbwd = WireData()!;
        int                  i, nb = sbwd.EdgeCount;
        if (nb != wi.EdgeCount)
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            return false;
        }
        // D abord on protege
        for (i = 1; i <= nb; i++)
        {
            if (wi.Ordered(i) == 0)
            {
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
                return false;
            }
        }

        var newedges = new NCollection_HSequence<Shape>();
        for (i = 1; i <= nb; i++)
        {
            newedges.Append(sbwd.Edge(wi.Ordered(i)));
        }
        for (i = 1; i <= nb; i++)
        {
            sbwd.Set(ShapeCasts.Edge(newedges.Value(i)), i);
        }

        _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        return true;
    }

    /// <summary>
    /// Removes the edge <paramref name="num"/> if it is small (shorter than <paramref name="precsmall"/>); if its
    /// vertices differ, only when the topology may be modified and <paramref name="lockvtx"/> is false, and then the
    /// neighbours are connected.
    /// </summary>
    public bool FixSmall(int num, bool lockvtx, double precsmall)
    {
        EnterWork(); // OCCT.NET: frozen
        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsLoaded || NbEdges() <= 1)
        {
            return false;
        }

        // analysis:
        WireAnalysis theAdvAnalyzer = _analyzer;
        if (theAdvAnalyzer is null)
        {
            return false;
        }
        int n = (num > 0 ? num : NbEdges());
        theAdvAnalyzer.CheckSmall(n, precsmall);
        if (theAdvAnalyzer.LastCheckStatus(ExtendStatus.Fail))
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            //: n2    return false;
        }

        if (!theAdvAnalyzer.LastCheckStatus(ExtendStatus.Done))
        {
            return false;
        }

        // OUI cette edge est NULLE

        if (theAdvAnalyzer.LastCheckStatus(ExtendStatus.DONE2))
        {
            // edge is small, but vertices are not the same..
            if (lockvtx || !_topoMode)
            {
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                return false;
            }
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }
        else
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }

        // action: remove edge
        if (Context() is not null)
        {
            Context()!.Remove(WireData()!.Edge(n));
        }
        SendWarning(WireData()!.Edge(n), new Msg("FixAdvWire.FixSmall.MSG0")); // Small edge(s) removed
        WireData()!.Remove(n);

        // call FixConnected in the case if vertices of the small edge were not the same
        if (LastFixStatus(ExtendStatus.DONE2))
        {
            int savLastFixStatus = _lastFixStatus;
            // #43 rln 20.11.98 S4054 CTS18544 entity 21734 removing last edge
            FixConnected(n <= NbEdges() ? n : 1, precsmall);
            if (LastFixStatus(ExtendStatus.Fail))
            {
                savLastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            }
            _lastFixStatus = savLastFixStatus;
        }

        return true;
    }

    /// <summary>
    /// Connects the edge <paramref name="num"/> to the previous one by a common vertex (the closer of the two, or a
    /// combined one), if their ends are within <paramref name="prec"/> (the maximal tolerance when negative).
    /// </summary>
    public bool FixConnected(int num, double prec, bool updateWire = true)
    {
        EnterWork(); // OCCT.NET: frozen
        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsLoaded || NbEdges() <= 0)
        {
            return false;
        }

        // analysis
        _analyzer.CheckConnected(num, prec < 0 ? MaxTolerance() : prec);
        if (_analyzer.LastCheckStatus(ExtendStatus.Fail))
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
        }
        if (!_analyzer.LastCheckStatus(ExtendStatus.Done))
        {
            return false;
        }

        // action: replacing vertex
        ExtendWireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.EdgeCount);
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.EdgeCount);
        Edge          e1   = sbwd.Edge(n1);
        Edge          e2   = sbwd.Edge(n2);

        var           sae = new EdgeAnalysis();
        Vertex v1  = sae.LastVertex(e1);
        Vertex v2  = sae.FirstVertex(e2);
        Vertex v;

        if (_analyzer.LastCheckStatus(ExtendStatus.DONE1))
        { // absolutely confused
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            // #40 rln 18.11.98 S4054 BUC60035 entity 2393 (2-nd sub-curve is edge with the same vertex)
            if (v2.IsSame(sae.LastVertex(e2)))
            {
                v = v2;
                if (Context() is not null)
                {
                    Context()!.Replace(v1, v.Oriented(v1.Orientation()));
                }
            }
            else
            {
                v = v1;
                if (Context() is not null)
                {
                    Context()!.Replace(v2, v.Oriented(v2.Orientation()));
                }
            }
        }
        else
        { // on moyenne ...
            if (_analyzer.LastCheckStatus(ExtendStatus.DONE2))
            {
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
            }
            else
            {
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
            }
            var sbv = new VertexBuilder();
            v = sbv.InternalCombineVertex(v1, v2, 1.0001);
            if (Context() is not null)
            {
                Context()!.Replace(v1, v.Oriented(v1.Orientation()));
                Context()!.Replace(v2, v.Oriented(v2.Orientation()));
            }
        }

        // replace vertices to a new one
        var sbe = new BuildEdgeBuilder();
        if (sbwd.EdgeCount < 2)
        {
            if (e2.Free() && _topoMode)
            {
                var b = new BrepBuilder();
                b.Remove(e2, sae.FirstVertex(e2));
                b.Remove(e2, sae.LastVertex(e2));
                b.Add(e2, v.Oriented(Forward));
                b.Add(e2, v.Oriented(Reversed));
            }
            else
            {
                Edge tmpE = sbe.InternalCopyReplaceVertices(e2, v, v);
                sbwd.Set(tmpE, n2);
                if (Context() is not null)
                {
                    Context()!.Replace(e2, tmpE);
                }
            }
        }
        else
        {
            if (e2.Free() && e1.Free() && _topoMode)
            {
                var b = new BrepBuilder();
                b.Remove(e2, sae.FirstVertex(e2));
                b.Add(e2, v.Oriented(Forward));
                if (!_analyzer.LastCheckStatus(ExtendStatus.DONE1) || sae.FirstVertex(e2).IsSame(sae.LastVertex(e2)))
                {
                    b.Remove(e1, sae.LastVertex(e1));
                    b.Add(e1, v.Oriented(Reversed));
                }
            }
            else
            {
                Edge tmpE2 = sbe.InternalCopyReplaceVertices(e2, v, new Vertex());
                sbwd.Set(tmpE2, n2);
                if (Context() is not null)
                {
                    Context()!.Replace(e2, tmpE2);
                }
                if (!_analyzer.LastCheckStatus(ExtendStatus.DONE1) || sae.FirstVertex(e2).IsSame(sae.LastVertex(e2)))
                {
                    Edge tmpE1 = sbe.InternalCopyReplaceVertices(e1, new Vertex(), v);
                    sbwd.Set(tmpE1, n1);
                    if (Context() is not null)
                    {
                        Context()!.Replace(e1, tmpE1);
                    }
                }
            }
        }

        // Optionally update wire data with context replacements
        if (updateWire && Context() is not null)
        {
            UpdateWire();
        }

        return true;
    }

    /// <summary>Fixes the seam edge <paramref name="num"/>: makes its two pcurves the analyser's seam pair.</summary>
    public bool FixSeam(int num)
    {
        EnterWork(); // OCCT.NET: frozen
        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        Curve2D? c1 = null, c2 = null;
        double        cf = 0.0, cl = 0.0;
        if (!_analyzer.CheckSeam(num, ref c1, ref c2, ref cf, ref cl))
        {
            return false;
        }

        var         b = new BrepBuilder();
        Edge e = WireData()!.Edge(num > 0 ? num : NbEdges());
        b.UpdateEdge(e, c2, c1, Face(), 0.0); //: S4136: BRep_Tool::Tolerance(E)
        b.Range(e, Face(), cf, cl);
        _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);

        return true;
    }

    //=======================================================================
    // function : FixShifted
    // purpose  : fix parametric curves which may be shifted
    // to whole parametric range of closed surface as result of recomputing
    // from 3d representation.
    // This can be a curve on a seam or near it.
    // This function is to be called before FixDegenerated.
    // LIMITATION: this function cannot fix cases when, e.g., closed wire is
    // composed of two meridians of the sphere and one of them is seam.
    // NOTE: wire is supposed to be closed and sorted !
    //=======================================================================

    //: p4 abv 23.02.99: PRO9234 #15720: update UV points of edge
    private static void UpdateEdgeUVPoints(in Edge e, in Face f)
    {
        Tool.Range(e, f, out double first, out double last);
        new BrepBuilder().Range(e, f, first, last);
    }

    /// <summary>
    /// Fixes pcurves shifted by a period of a closed surface (recomputed from 3d curves), on the seam or near it; to be
    /// called before FixDegenerated. The wire is supposed closed and ordered.
    /// </summary>
    public bool FixShifted()
    {
        EnterWork(); // OCCT.NET: frozen
        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        SurfaceAnalysis surf = _analyzer.Surface()!;
        // #78 rln 12.03.99 S4135: checking spatial closure with Precision
        bool uclosed = surf.IsUClosed(Precision());
        bool vclosed = surf.IsVClosed(Precision()) || surf.Surface() is SphericalSurface;
        // #67 rln 01.03.99 S4135: ims010.igs entity D11900 (2D contour is 2*PI higher than V range
        // [-pi/2,p/2])

        // PTV 26.06.2002 begin
        // CTS18546-2.igs entity 2222: base curve is periodic and 2dcurve is shifted
        bool   isVCrvClosed = false;
        double vRange       = 1.0;
        if (surf.Surface() is RevolutionSurface surOfRev)
        {
            Curve3D baseCrv = surOfRev.BasisCurve();
            while ((baseCrv is OffsetCurve3D) || (baseCrv is TrimmedCurve3D))
            {
                if (baseCrv is OffsetCurve3D offset)
                {
                    baseCrv = offset.BasisCurve();
                }
                if (baseCrv is TrimmedCurve3D trimmed)
                {
                    baseCrv = trimmed.BasisCurve();
                }
            }
            if (baseCrv.IsPeriodic)
            {
                vclosed      = true;
                vRange       = baseCrv.Period();
                isVCrvClosed = true;
            }
        }
        // PTV 26.06.2002 end
        if (!uclosed && !vclosed)
        {
            return false;
        }

        double uRange;
        surf.Surface()!.Bounds(out double suf, out double sul, out double svf, out double svl);
        double suMid, svMid;
        suMid = 0.5 * (suf + sul);
        svMid = 0.5 * (svf + svl);
        if (uclosed)
        {
            uRange = Math.Abs(sul - suf);
        }
        else
        {
            uRange = RealLast();
        }
        if (!isVCrvClosed)
        {
            if (vclosed)
            {
                vRange = Math.Abs(svl - svf);
            }
            else
            {
                vRange = RealLast();
            }
        }
        double uTol = 0.2 * uRange, vTol = 0.2 * vRange;

        ExtendWireData sbwdOring = WireData()!;
        var                  sae       = new EdgeAnalysis();
        var                  sbwd      = new ExtendWireData();
        for (int i = 1; i <= sbwdOring.EdgeCount; i++)
        {
            Edge e1 = sbwdOring.Edge(i);
            if (Tool.Degenerated(e1) && !sae.HasPCurve(e1, Face()))
            {
                continue;
            }

            sbwd.Add(e1);
        }

        var    sbe = new BuildEdgeBuilder();
        int    nb  = sbwd.EdgeCount;
        bool   end = (nb == 0), degstop = false;
        int    stop  = nb;
        int    degn2 = 0;
        Point3D pdeg  = new Point3D();
        // pdn 17.12.98 r0901_ec 38237 to shift wire at 0

        var box = new Box2d();
        for (int n2 = 1, n1 = nb; !end; n1 = n2++)
        {
            if (n2 > nb)
            {
                n2 = 1;
            }
            if (n2 == stop)
            {
                end = true;
            }

            Edge e1 = sbwd.Edge(n1);
            Edge e2 = sbwd.Edge(n2);

            if (Tool.Degenerated(e1) || Tool.Degenerated(e2))
            {
                if (!degstop)
                {
                    stop    = n2;
                    degstop = true;
                }
                continue;
            }

            Vertex v = sae.FirstVertex(e2);
            if (v.IsNull)
            {
                continue;
            }

            Point3D p = Tool.Pnt(v);

            double        a1 = 0.0, b1 = 0.0, a2 = 0.0, b2 = 0.0;
            Curve2D? c2d1 = null, c2d2 = null;

            //: abv 29.08.01: torCuts.sat: distinguish degeneration by U and by V;
            // only corresponding move is prohibited
            //    bool isDeg = surf->IsDegenerated ( p, Max ( Precision(),
            //    BRep_Tool::Tolerance(V) ) );
            int      isDeg = 0;
            Point2D degP1 = new Point2D(), degP2 = new Point2D();
            double   degT1 = 0.0, degT2 = 0.0;
            if (surf.DegeneratedValues(p, Max(Precision(), Tool.Tolerance(v)), ref degP1, ref degP2, ref degT1,
                                       ref degT2))
            {
                isDeg = (Math.Abs(degP1.X() - degP2.X()) > Math.Abs(degP1.Y() - degP2.Y()) ? 1 : 2);
            }

            // abv 23 Feb 00: UKI60107-6 210: additional check for near-degenerated case
            // smh#15 PRO19800. Check if the surface is surface of revolution.
            if (surf.Surface() is RevolutionSurface)
            {
                if (isDeg == 0 && !vclosed)
                {
                    if (c2d1 is null && !sae.PCurve(e1, Face(), out c2d1, out a1, out b1, true))
                    {
                        continue;
                    }
                    Point2D p1 = new Point2D(suf, c2d1!.Value(b1).Y());
                    Point2D p2 = new Point2D(sul, c2d1.Value(b1).Y());
                    if (surf.IsDegenerated(p1, p2, MaxTolerance(), 10)
                        && !surf.IsDegenerated(c2d1.Value(a1), c2d1.Value(b1), MaxTolerance(), 10))
                    { // abv 31.07.00: trj4_pm1-ec-214.stp #31274: still allow
                        // work if edge already exists
                        isDeg = 1;
                    }
                }
                if (isDeg == 0 && !uclosed)
                {
                    if (c2d1 is null && !sae.PCurve(e1, Face(), out c2d1, out a1, out b1, true))
                    {
                        continue;
                    }
                    Point2D p1 = new Point2D(c2d1!.Value(b1).X(), svf);
                    Point2D p2 = new Point2D(c2d1.Value(b1).X(), svl);
                    if (surf.IsDegenerated(p1, p2, MaxTolerance(), 10)
                        && !surf.IsDegenerated(c2d1.Value(a1), c2d1.Value(b1), MaxTolerance(), 10))
                    { // abv 31.07.00: trj4_pm1-ec-214.stp #31274: still allow
                        // work if edge already exists
                        isDeg = 2;
                    }
                }
            }

            if (isDeg != 0)
            {
                if (!degstop)
                {
                    stop    = n2;
                    degstop = true;
                }

                if (degn2 == 0)
                {
                    degn2 = n2;
                    pdeg  = p;
                }
                else
                {
                    if (pdeg.SquareDistance(p) < Precision() * Precision())
                    {
                        degn2 = n2;
                        // if ( stop < n2 ) { stop = n2; degstop = true; }
                    }
                    else
                    {
                        double        ax1 = 0.0, bx1 = 0.0, ax2 = 0.0, bx2 = 0.0;
                        Curve2D? cx1 = null, cx2 = null;
                        if ((c2d1 is null && !sae.PCurve(e1, Face(), out c2d1, out a1, out b1, true))
                            || (c2d2 is null && !sae.PCurve(e2, Face(), out c2d2, out a2, out b2, true))
                            || !sae.PCurve(sbwd.Edge(degn2 > 1 ? degn2 - 1 : nb), Face(), out cx1, out ax1, out bx1, true)
                            || !sae.PCurve(sbwd.Edge(degn2), Face(), out cx2, out ax2, out bx2, true))
                        {
                            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                            continue;
                        }
                        Point2D pd1 = cx1!.Value(bx1);
                        Point2D pd2 = cx2!.Value(ax2);
                        Point2D pn1 = c2d1!.Value(b1);
                        Point2D pn2 = c2d2!.Value(a2);
                        Vector2D x   = new Vector2D(0.0, 0.0); // shift vector
                        double   period;
                        if (uclosed)
                        {
                            x.SetX(1.0);
                            period = uRange;
                        }
                        else
                        {
                            x.SetY(1.0);
                            period = vRange;
                        }
                        double rot1 = (pn1.XY() - pd2.XY()) ^ x.XY();
                        double rot2 = (pd1.XY() - pn2.XY()) ^ x.XY();
                        double scld = (pd2.XY() - pd1.XY()) * x.XY();
                        double scln = (pn2.XY() - pn1.XY()) * x.XY();
                        if (rot1 * rot2 < -Kerf.Engine.Foundation.Precision.PConfusion() && scld * scln < -Kerf.Engine.Foundation.Precision.PConfusion()
                            && Math.Abs(scln) > 0.1 * period && Math.Abs(scld) > 0.1 * period
                            && rot1 * scld > Kerf.Engine.Foundation.Precision.PConfusion() && rot2 * scln > Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            // abv 02 Mar 00: trying more sophisticated analysis (ie_exhaust-A.stp #37520)
                            double sign  = (rot2 > 0 ? 1.0 : -1.0);
                            double deep1 = Min(
                                sign * (pn2.XY() * x.XY()),
                                Min(sign * (pd1.XY() * x.XY()),
                                    Min(sign * (c2d2.Value(b2).XY() * x.XY()),
                                        Min(sign * (cx1.Value(ax1).XY() * x.XY()),
                                            Min(sign * (c2d2.Value(0.5 * (a2 + b2)).XY() * x.XY()),
                                                sign * (cx1.Value(0.5 * (ax1 + bx1)).XY() * x.XY()))))));
                            double deep2 = Max(
                                sign * (pn1.XY() * x.XY()),
                                Max(sign * (pd2.XY() * x.XY()),
                                    Max(sign * (c2d1.Value(a1).XY() * x.XY()),
                                        Max(sign * (cx2.Value(bx2).XY() * x.XY()),
                                            Max(sign * (c2d1.Value(0.5 * (a1 + b1)).XY() * x.XY()),
                                                sign * (cx2.Value(0.5 * (ax2 + bx2)).XY() * x.XY()))))));
                            double deep = deep2 - deep1; // estimated current size of wire by x
                            // pdn 30 Oct 00: trying correct period [0,period] (trj5_k1-tc-203.stp #4698)
                            double dx = ShapeAnalysisAnalysis.AdjustToPeriod(deep, Kerf.Engine.Foundation.Precision.PConfusion(),
                                                                     period + Kerf.Engine.Foundation.Precision.PConfusion());
                            x *= (scld > 0 ? -dx : dx);
                            // x *= ( std::abs(scld-scln) > 1.5 * period ? 2. : 1. ) *
                            //      ( scld >0 ? -period : period );
                            var shift = new Transform2D();
                            shift.SetTranslation(x);
                            for (int k = degn2;; k++)
                            {
                                if (k > nb)
                                {
                                    k = 1;
                                }
                                if (k == n2)
                                {
                                    break;
                                }
                                Edge edge = sbwd.Edge(k);
                                if (!sae.PCurve(edge, Face(), out cx1, out ax1, out bx1, true))
                                {
                                    continue;
                                }
                                // cx1->Transform ( Shift );
                                //  skl 15.05.2002 for OCC208 (if few edges have reference to one pcurve)
                                var cx1new = cx1!.Transformed(shift) as Curve2D;
                                sbe.ReplacePCurve(edge, cx1new, Face());
                                UpdateEdgeUVPoints(edge, Face());
                            }
                            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                            continue;
                        }
                        // degn2 = n2; pdeg = p; // ie_exhaust-A.stp #37520
                    }
                }
                // OCCT's commented-out fix of half spheres ("pdn to fix half sphere") is left out.
                //: abv 29.08.01: torCuts.sat:      continue;
            }

            if ((c2d1 is null && !sae.PCurve(e1, Face(), out c2d1, out a1, out b1, true))
                || (c2d2 is null && !sae.PCurve(e2, Face(), out c2d2, out a2, out b2, true)))
            {
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                continue;
            }
            Point2D p2d1 = c2d1!.Value(b1);
            Point2D p2d2 = c2d2!.Value(a2);
            box.Add(p2d1);

            double du = 0.0, dv = 0.0;
            if (uclosed && isDeg != 1)
            {
                double dx = Math.Abs(p2d2.X() - p2d1.X());
                if (dx > uRange - uTol)
                {
                    du = ShapeAnalysisAnalysis.AdjustByPeriod(p2d2.X(), p2d1.X(), uRange);
                }
                else if (dx > uTol && stop == nb)
                {
                    stop = n2; //: abv 29.08.01: torCuts2.stp
                }
            }
            if (vclosed && isDeg != 2)
            {
                double dy = Math.Abs(p2d2.Y() - p2d1.Y());
                if (dy > vRange - vTol)
                {
                    dv = ShapeAnalysisAnalysis.AdjustByPeriod(p2d2.Y(), p2d1.Y(), vRange);
                }
                else if (dy > vTol && stop == nb)
                {
                    stop = n2;
                }
            }
            if (du == 0.0 && dv == 0.0)
            {
                continue;
            }

            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            var shift2 = new Transform2D();
            shift2.SetTranslation(new Vector2D(du, dv));
            // c2d2->Transform ( Shift );
            //  skl 15.05.2002 for OCC208 (if few edges have reference to one pcurve)
            var c2d2new = c2d2.Transformed(shift2) as Curve2D;
            sbe.ReplacePCurve(e2, c2d2new, Face());
            UpdateEdgeUVPoints(e2, Face());
        }
        if (box.IsVoid)
        {
            return false; // #3 smh 01.04.99. S4163: Overflow, when box is void.
        }

        box.Get(out double umin, out double vmin, out double umax, out double vmax);
        if (Math.Abs(umin + umax - suf - sul) < uRange && Math.Abs(vmin + vmax - svf - svl) < vRange
            && !LastFixStatus(ExtendStatus.Done))
        {
            return false;
        }

        box.SetVoid();
        int n; // svv Jan11 2000 : porting on DEC
        for (n = 1; n <= nb; n++)
        {
            if (!sae.PCurve(sbwd.Edge(n), Face(), out Curve2D? c2d, out double a, out double b, true))
            {
                continue;
            }
            box.Add(c2d!.Value(a));
            box.Add(c2d.Value(0.5 * (a + b)));
        }
        box.Get(out umin, out vmin, out umax, out vmax);

        double du2 = 0.0, dv2 = 0.0;

        if (uclosed)
        {
            double umid = 0.5 * (umin + umax);
            //     du = ShapeAnalysis::AdjustToPeriod(umid, SUF, SUL);
            // PTV 26.06.2002 xloop torus-apple iges face mode
            du2 = ShapeAnalysisAnalysis.AdjustByPeriod(umid, suMid, uRange);
        }
        if (vclosed)
        {
            double vmid = 0.5 * (vmin + vmax);
            //     dv = ShapeAnalysis::AdjustToPeriod(vmid, SVF, SVL);
            // PTV 26.06.2002 xloop torus-apple iges face mode
            dv2 = ShapeAnalysisAnalysis.AdjustByPeriod(vmid, svMid, vRange);
        }

        if (du2 == 0.0 && dv2 == 0.0)
        {
            return true;
        }

        _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);

        var shift3 = new Transform2D();
        shift3.SetTranslation(new Vector2D(du2, dv2));

        for (n = 1; n <= sbwdOring.EdgeCount; n++)
        {
            Edge ed = sbwdOring.Edge(n);
            if (!sae.PCurve(ed, Face(), out Curve2D? c2d, out double a, out double b, true))
            {
                continue;
            }
            // skl 15.05.2002 for OCC208 (if few edges have reference to one pcurve)
            var c2d2 = c2d!.Transformed(shift3) as Curve2D;
            sbe.ReplacePCurve(ed, c2d2, Face());
            UpdateEdgeUVPoints(ed, Face());
        }
        return true;
    }

    /// <summary>
    /// Fixes the degenerated edge <paramref name="num"/> (or before it): inserts a degenerated edge where the wire
    /// passes a singularity of the surface, or replaces the edge by one; removes an edge encoded degenerated with no
    /// pcurve and no singularity.
    /// </summary>
    public bool FixDegenerated(int num)
    {
        EnterWork(); // OCCT.NET: frozen
        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        // analysis
        Point2D p2d1 = new Point2D(), p2d2 = new Point2D();
        _analyzer.CheckDegenerated(num, ref p2d1, ref p2d2);
        if (_analyzer.LastCheckStatus(ExtendStatus.FAIL1))
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
        }
        //: abv 29.08.01: torHalf2.sat: if edge was encoded as degenerated but
        //  has no pcurve and no singularity is found at that point, remove it
        if (_analyzer.LastCheckStatus(ExtendStatus.FAIL2))
        {
            WireData()!.Remove(num);
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
            return true;
        }
        if (!_analyzer.LastCheckStatus(ExtendStatus.Done))
        {
            return false;
        }

        // action: create degenerated edge and insert it (or replace)

        Vector2D    vect2d = new Vector2D(p2d1, p2d2);
        Direction2D    dir2d  = new Direction2D(vect2d);
        LineCurve2D line2d = new LineCurve2D(p2d1, dir2d);

        Edge  degEdge = new Edge();
        BrepBuilder b       = new BrepBuilder();
        b.MakeEdge(ref degEdge);
        b.Degenerated(degEdge, true);
        b.UpdateEdge(degEdge, line2d, Face(), Kerf.Engine.Foundation.Precision.Confusion());
        b.Range(degEdge, Face(), 0.0, vect2d.Magnitude());

        ExtendWireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.EdgeCount);
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.EdgeCount);

        bool lack = _analyzer.LastCheckStatus(ExtendStatus.DONE1);
        int  n3   = (lack ? n2 : (n2 < sbwd.EdgeCount ? n2 + 1 : 1));

        EdgeAnalysis sae = new EdgeAnalysis();
        Vertex      v1  = sae.LastVertex(sbwd.Edge(n1));
        Vertex      v2  = sae.FirstVertex(sbwd.Edge(n3));

        v1.Orientation(Forward);
        v2.Orientation(Reversed);
        b.Add(degEdge, v1);
        b.Add(degEdge, v2);
        degEdge.Orientation(Forward);

        if (lack)
        {
            sbwd.Add(degEdge, n2);
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }
        else
        {
            sbwd.Set(degEdge, n2);
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }

        //  commented to avoid extra messages
        //  SendWarning ( degEdge, Message_Msg ( "FixWire.FixDegenerated.MSG0" ) );// Degenerated edge(s)
        //  detected

        return true;
    }

    //=======================================================================
    // function : FixSelfIntersectingEdge
    // purpose  : Tests edge for self-intersection and updates tolerance of vertex
    //           if intersection is found
    //           Returns True if tolerance was increased
    //=======================================================================

    // Create edge on basis of E with new pcurve and call FixSP
    // Return resulting tolerance and modified pcurve
    private static bool TryNewPCurve(in Edge e, in Face face, ref Curve2D? c2d, ref double first,
                                     ref double last, ref double tol)
    {
        Curve3D? crv = Tool.Curve(e, out double f, out double l);
        if (crv is null)
        {
            return false;
        }

        // make temp edge and compute tolerance
        MakeEdge mkedge = new MakeEdge(crv, f, l);

        BuildEdgeBuilder sbe = new BuildEdgeBuilder();     // skl 17.07.2001
        sbe.SetRange3d(mkedge.InternalEdge(), f, l); // skl 17.07.2001

        if (!mkedge.IsDone)
        {
            return false;
        }

        Edge  edge = mkedge.InternalEdge();
        BrepBuilder b    = new BrepBuilder();
        b.UpdateEdge(edge, c2d, face, 0.0);
        b.Range(edge, face, first, last);
        b.SameRange(edge, false);
        // no call to BRepLib:  B.SameParameter ( edge, false );

        EdgeFixer sfe = new EdgeFixer();
        sfe.FixSameParameter(edge, face);
        c2d = Tool.CurveOnSurface(edge, face, out first, out last);
        tol = Tool.Tolerance(edge);
        return true;
    }

    //: k3 abv 24 Dec 98: BUC50070 #26682 and #30087:
    // Try to cut out the loop on the pcurve and make new pcurve by concatenating
    // the parts.
    // If result is not SameParameter with prec, does nothing and returns False
    // Warning: resulting pcurve will be C0

    private static int HowMuchPCurves(in Edge e)
    {
        int count = 0;
        // OCCT.NET: frozen: the list is only read (OCCT walks ChangeCurves())
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)e.TShape()!).Curves());

        while (itcr.More())
        {
            CurveRepresentation cr = itcr.Value();
            if (cr.IsCurveOnSurface())
            {
                count++;
            }
            itcr.Next();
        }

        return count;
    }

    private static bool RemoveLoop(in Edge e, in Face face, in IntersectionPoint2D ip, double tolfact,
                                   double prec, bool removeLoop3d)
    {
        bool loopRemoved3d;
        if (Tool.IsClosed(e, face))
        {
            return false;
        }

        Curve3D? crv = Tool.Curve(e, out double f, out double l);

        double t1 = ip.ParamOnFirst();
        double t2 = ip.ParamOnSecond();
        if (t1 > t2)
        {
            double t = t1;
            t1       = t2;
            t2       = t;
        }

        EdgeAnalysis sae = new EdgeAnalysis();
        if (!sae.PCurve(e, face, out Curve2D? c2d, out double a, out double b, false))
        {
            return false;
        }

        CurveAdaptor gac = new CurveAdaptor(crv!, f, l);
        double            dt  = tolfact * gac.Resolution(prec);
        t1 -= dt; // 1e-3;//::Precision::PConfusion();
        t2 += dt; // 1e-3;//::Precision::PConfusion();

        if (t1 <= a || t2 >= b)
        { // should not be so, but to be sure ..
            return false;
        }

        // direct construction causes error on osf system.
        Point3D p   = new Point3D(0, 0, 0);
        Direction3D d   = new Direction3D(Direction3D.D.Z);
        CoordinateSystem ax  = new CoordinateSystem(p, d);
        Plane3D pln = new Plane3D(ax);

        // PTV OCC884
        PlaneSurface? planeSurf = Tool.Surface(face) as PlaneSurface;
        Curve3D? pcurve3d   = crv;
        if (planeSurf is not null)
        {
            pln = planeSurf.Pln();
        }
        else
        {
            pcurve3d = Projections.To3d(c2d!, pln);
        }

        // first segment
        // occ::handle<Geom_Curve> pcurve3d = GeomAPI::To3d ( c2d, Pln );
        TrimmedCurve3D                   trim    = new TrimmedCurve3D(pcurve3d!, a, t1);
        CompCurveToBSplineCurve connect = new CompCurveToBSplineCurve(trim);

        // null-length segment patch instead of loop
        NCollection_Array1<Point3D> poles = new NCollection_Array1<Point3D>(1, 2);
        NCollection_Array1<double> knots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int>    mults = new NCollection_Array1<int>(1, 2);

        poles.SetValue(1, trim.Value(t1));
        knots.SetValue(1, t1);
        mults.SetValue(1, 2);

        trim = new TrimmedCurve3D(pcurve3d!, t2, b);
        poles.SetValue(2, trim.Value(t2));
        knots.SetValue(2, t2);
        mults.SetValue(2, 2);

        BSplineCurve3D patch = new BSplineCurve3D(poles, knots, mults, 1);
        if (!connect.Add(patch, Kerf.Engine.Foundation.Precision.PConfusion(), true, false))
        {
            return false;
        }

        // last segment
        if (!connect.Add(trim, Kerf.Engine.Foundation.Precision.PConfusion(), true, false))
        {
            return false;
        }
        // PTV OCC884
        // keep created 3d curve
        Curve3D? new3dCrv = connect.InternalBSplineCurve();

        Curve2D? bs = Projections.To2d(new3dCrv!, pln);
        if (bs is null)
        {
            return false;
        }

        // make temp edge and compute tolerance
        BrepBuilder B = new BrepBuilder();

        if (!removeLoop3d)
        { // old variant (not remove loop 3d)
            double newtol = 0;
            // OCC901
            int nbC2d = HowMuchPCurves(e);
            if (nbC2d <= 1 && planeSurf is not null)
            {
                B.UpdateEdge(e, bs, face, 0);
            }
            else if (!TryNewPCurve(e, face, ref bs, ref a, ref b, ref newtol))
            {
                return false;
            }

            double tol = Tool.Tolerance(e);
            if (newtol > Max(prec, tol))
            {
                return false;
            }
            //: s2  bs = BRep_Tool::CurveOnSurface ( edge, face, a, b );
            if (Math.Abs(a - f) > Kerf.Engine.Foundation.Precision.PConfusion() || // smth strange, cancel
                Math.Abs(b - l) > Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return false;
            }
            // PTV OCC884
            if (planeSurf is not null)
            {
                B.UpdateEdge(e, new3dCrv, Max(newtol, tol));
                // OCC901
                if (!TryNewPCurve(e, face, ref bs, ref a, ref b, ref newtol))
                {
                    return false;
                }
            }
            B.UpdateEdge(e, bs, face, newtol);
            B.UpdateVertex(sae.FirstVertex(e), newtol);
            B.UpdateVertex(sae.LastVertex(e), newtol);
            return true;
        }

        //: q1
        Surface?       s  = Tool.Surface(face, out Placement L);
        CurveAdaptor2D ac = new CurveAdaptor2D(c2d!);
        SurfaceAdaptor AS = new SurfaceAdaptor(s!);

        CurveOnSurface acs = new CurveOnSurface(ac, AS);
        Point3D                   p1  = acs.Value(t1);
        Point3D                   p2  = acs.Value(t2);
        Point3D pcurPnt = new Point3D((p1.X() + p2.X()) / 2, (p1.Y() + p2.Y()) / 2, (p1.Z() + p2.Z()) / 2);

        TransferParametersProjAnalysis sftp  = new TransferParametersProjAnalysis(e, face);
        NCollection_HSequence<double>        seq2d = new NCollection_HSequence<double>();
        seq2d.Append(t1);
        seq2d.Append(t2);
        seq2d.Append((t1 + t2) / 2);
        NCollection_HSequence<double> seq3d = sftp.Perform(seq2d, false);

        double dist1 = pcurPnt.Distance(crv!.Value(seq3d.Value(1))); // correcting Seq3d already project
        double dist2 = pcurPnt.Distance(crv.Value(seq3d.Value(2)));
        double dist3 = pcurPnt.Distance(crv.Value(seq3d.Value(3)));
        double ftrim, ltrim;
        loopRemoved3d = dist3 <= Max(dist1, dist2);

        Curve3D? bs1 = null;

        if (!loopRemoved3d)
        {
            // create new 3d curve
            ftrim = seq3d.Value(1);
            ltrim = seq3d.Value(2);
            ftrim -= dt;
            ltrim += dt;
            TrimmedCurve3D                   trim1    = new TrimmedCurve3D(crv, f, ftrim);
            CompCurveToBSplineCurve connect1 = new CompCurveToBSplineCurve(trim1);
            NCollection_Array1<Point3D>          poles1   = new NCollection_Array1<Point3D>(1, 2);
            NCollection_Array1<double>          knots1   = new NCollection_Array1<double>(1, 2);
            NCollection_Array1<int>             mults1   = new NCollection_Array1<int>(1, 2);

            poles1.SetValue(1, trim1.Value(ftrim));
            knots1.SetValue(1, ftrim);
            mults1.SetValue(1, 2);

            trim1 = new TrimmedCurve3D(crv, ltrim, l);
            poles1.SetValue(2, trim1.Value(ltrim));
            knots1.SetValue(2, ltrim);
            mults1.SetValue(2, 2);

            // create b-spline curve
            BSplineCurve3D patch1 = new BSplineCurve3D(poles1, knots1, mults1, 1);

            if (!connect1.Add(patch1, Kerf.Engine.Foundation.Precision.PConfusion(), true, false))
            {
                return false;
            }

            // last segment
            if (!connect1.Add(trim1, Kerf.Engine.Foundation.Precision.PConfusion(), true, false))
            {
                return false;
            }

            bs1 = connect1.InternalBSplineCurve();

            if (bs1 is null)
            {
                return false;
            }
        }
        //  double oldtol = BRep_Tool::Tolerance ( E );

        if (!loopRemoved3d)
        {
            B.UpdateEdge(e, bs1, L, 0.0);
        }
        B.UpdateEdge(e, bs, face, 0.0);
        B.Range(e, face, f, l);
        B.SameRange(e, false);

        EdgeFixer sfe = new EdgeFixer();
        sfe.FixSameParameter(e);

        return true;
    }

    private static bool RemoveLoop(in Edge e, in Face face, in IntersectionPoint2D ip2d,
                                   ref Edge e1, ref Edge e2)
    {
        if (Tool.IsClosed(e, face))
        {
            return false;
        }

        Curve3D? crv = Tool.Curve(e, out double f, out double l);

        double t1 = ip2d.ParamOnFirst();
        double t2 = ip2d.ParamOnSecond();

        if (t1 > t2)
        {
            double t = t1;
            t1       = t2;
            t2       = t;
        }

        EdgeAnalysis sae = new EdgeAnalysis();

        // define vertexes Vfirst , Vlast, Vmid
        Vertex vfirst, vlast, vmid = new Vertex();
        // initialize Vfirst and Vlast
        vfirst = sae.FirstVertex(e);
        vlast  = sae.LastVertex(e);

        // find a 2d curve and parameters from edge E
        if (!sae.PCurve(e, face, out Curve2D? c2d, out double a, out double b, false))
        {
            return false;
        }

        // first segment for 2d curve
        TrimmedCurve2D? trim1 = null;
        if ((t1 - a) > Kerf.Engine.Foundation.Precision.PConfusion())
        {
            trim1 = new TrimmedCurve2D(c2d!, a, t1);
        }
        // second segment for 2d curve
        TrimmedCurve2D? trim2 = new TrimmedCurve2D(c2d!, t2, b);

        //  if ( trim1.IsNull() || trim2.IsNull()  )
        if (trim2 is null)
        {
            return false;
        }

        Surface?       s  = Tool.Surface(face, out Placement L);
        CurveAdaptor2D ac = new CurveAdaptor2D(c2d!);
        SurfaceAdaptor AS = new SurfaceAdaptor(s!);

        CurveOnSurface acs = new CurveOnSurface(ac, AS);
        Point3D                   p1  = acs.Value(t1);
        Point3D                   p2  = acs.Value(t2);
        Point3D pcurPnt = new Point3D((p1.X() + p2.X()) / 2, (p1.Y() + p2.Y()) / 2, (p1.Z() + p2.Z()) / 2);

        // transfer parameters from pcurve to 3d curve
        TransferParametersProjAnalysis sftp  = new TransferParametersProjAnalysis(e, face);
        NCollection_HSequence<double>        seq2d = new NCollection_HSequence<double>();
        seq2d.Append(t1);
        seq2d.Append(t2);
        seq2d.Append((t1 + t2) / 2);
        NCollection_HSequence<double> seq3d = sftp.Perform(seq2d, false);

        double dist1 = pcurPnt.Distance(crv!.Value(seq3d.Value(1))); // correcting Seq3d already project
        double dist2 = pcurPnt.Distance(crv.Value(seq3d.Value(2)));
        double dist3 = pcurPnt.Distance(crv.Value(seq3d.Value(3)));
        double ftrim, ltrim;
        if (dist3 > Max(dist1, dist2))
        { // is loop in 3d
            ftrim = seq3d.Value(1);
            ltrim = seq3d.Value(2);
        }
        else
        { // not loop in 3d
            ftrim = seq3d.Value(3);
            ltrim = seq3d.Value(3);
        }

        //  ftrim = Seq3d->Value(1);
        //  ltrim = Seq3d->Value(2);

        // trim for 3d curve 'crv' with parameters from 'f' to 'l'
        TrimmedCurve3D? trim3 = null;
        if (trim1 is not null)
        {
            trim3 = new TrimmedCurve3D(crv, f, ftrim);
        }
        // second segment for 3d curve
        TrimmedCurve3D? trim4 = new TrimmedCurve3D(crv, ltrim, l);

        //  if ( trim3.IsNull() || trim4.IsNull()  )
        if (trim4 is null)
        {
            return false;
        }

        // create a point for middle vertex
        Point3D pnt1 = crv.Value(ftrim);
        Point3D pnt2 = crv.Value(ltrim);
        Point3D pmid = new Point3D((pnt1.X() + pnt2.X()) / 2, (pnt1.Y() + pnt2.Y()) / 2, (pnt1.Z() + pnt2.Z()) / 2);

        BrepBuilder B = new BrepBuilder();

        // create new copies for E1 and E2
        if (trim1 is not null)
        {
            e1 = ShapeCasts.Edge(e.EmptyCopied());
        }
        e2 = ShapeCasts.Edge(e.EmptyCopied());

        // initialize middle vertex Vmid
        if (trim1 is null)
        {
            B.MakeVertex(ref vmid, pnt2, 0.0);
        }
        else
        {
            B.MakeVertex(ref vmid, pmid, 0.0);
        }

        BuildEdgeBuilder sbe = new BuildEdgeBuilder();

        // replace verteces for new edges E1 and E2
        if (e.Orientation() == Forward)
        {
            if (!e1.IsNull)
            {
                e1 = sbe.InternalCopyReplaceVertices(e1, vfirst, vmid);
            }
            e2 = sbe.InternalCopyReplaceVertices(e2, vmid, vlast);
        }
        else
        {
            if (!e1.IsNull)
            {
                e1 = sbe.InternalCopyReplaceVertices(e1, vmid, vlast);
            }
            e2 = sbe.InternalCopyReplaceVertices(e2, vfirst, vmid);
        }

        // Update edges by 2d and 3d curves
        EdgeFixer mySfe = new EdgeFixer();
        if (!e1.IsNull)
        {
            B.UpdateEdge(e1, trim1, face, 0.0);
            B.UpdateEdge(e1, trim3, 0.0);
            B.Range(e1, f, ftrim);
            B.SameRange(e1, false);
            //    B.SameParameter(E1,false);
            mySfe.FixSameParameter(e1);
            mySfe.FixVertexTolerance(e1);
        }
        B.UpdateEdge(e2, trim2, face, 0.0);
        B.UpdateEdge(e2, trim4, 0.0);
        B.Range(e2, ltrim, l);
        B.SameRange(e2, false);
        //  B.SameParameter(E2, false);
        mySfe.FixSameParameter(e2);
        mySfe.FixVertexTolerance(e2);

        return true;
    }

    private bool FixSelfIntersectingEdge(int num)
    {
        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        // analysis
        NCollection_Sequence<IntersectionPoint2D> points2d       = new NCollection_Sequence<IntersectionPoint2D>();
        NCollection_Sequence<Point3D>                     points3d       = new NCollection_Sequence<Point3D>();
        WireAnalysis?                              theAdvAnalyzer = _analyzer;
        if (theAdvAnalyzer is null)
        {
            return false;
        }
        theAdvAnalyzer.CheckSelfIntersectingEdge(num, points2d, points3d);
        if (theAdvAnalyzer.LastCheckStatus(ExtendStatus.Fail))
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
        }
        if (!theAdvAnalyzer.LastCheckStatus(ExtendStatus.Done))
        {
            return false;
        }

        // action: increase tolerance of vertex

        Edge e = WireData()!.Edge(num > 0 ? num : NbEdges());

        EdgeAnalysis sae  = new EdgeAnalysis();
        Vertex      v1   = sae.FirstVertex(e);
        Vertex      v2   = sae.LastVertex(e);
        double             tol1 = Tool.Tolerance(v1);
        double             tol2 = Tool.Tolerance(v2);
        Point3D             pnt1 = Tool.Pnt(v1);
        Point3D             pnt2 = Tool.Pnt(v2);

        // cycle is to verify fix in case of RemoveLoop
        double        tolfact = 0.1; // factor for shifting by parameter in RemoveLoop
        double        f2d = 0.0, l2d = 0.0;
        Curve2D? c2d    = null;
        double        newtol = 0.0; // = Precision();

        if (_removeLoopMode < 1)
        {
            for (int iter = 0; iter < 30; iter++)
            {
                bool   loopRemoved = false;
                double prevFirst = 0, prevLast = 0;
                for (int i = 1; i <= points2d.Length(); i++)
                {
                    Point3D pint   = points3d.Value(i);
                    double dist21 = pnt1.SquareDistance(pint);
                    double dist22 = pnt2.SquareDistance(pint);
                    if (dist21 < tol1 * tol1 || dist22 < tol2 * tol2)
                    {
                        continue;
                    }
                    newtol = 1.001 * Math.Sqrt(Min(dist21, dist22)); //: f8

                    //: k3 abv 24 Dec 98: BUC50070 #26682 and #30087: try to remove loop
                    if (_geomMode)
                    {
                        if (c2d is null)
                        {
                            sae.PCurve(e, Face(), out c2d, out f2d, out l2d, false);
                        }
                        double firstpar = points2d.Value(i).ParamOnFirst();
                        double lastpar  = points2d.Value(i).ParamOnSecond();
                        if (firstpar > prevFirst && lastpar < prevLast)
                        {
                            continue;
                        }
                        if (RemoveLoop(e,
                                       Face(),
                                       points2d.Value(i),
                                       tolfact,
                                       Min(MaxTolerance(), Max(newtol, Precision())),
                                       _removeLoopMode == 0))
                        {
                            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                            loopRemoved = true;
                            prevFirst   = firstpar;
                            prevLast    = lastpar;
                            continue; // repeat of fix on that edge required (to be done by caller)
                        }
                    }
                    if (newtol < MaxTolerance())
                    {
                        _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                        BrepBuilder b = new BrepBuilder();
                        if (dist21 < dist22)
                        {
                            tol1 = newtol;
                            b.UpdateVertex(v1, tol1);
                        }
                        else
                        {
                            tol2 = newtol;
                            b.UpdateVertex(v2, tol2);
                        }
                    }
                    else
                    {
                        _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                    }
                }

                // after RemoveLoop, check that self-intersection disappeared
                if (loopRemoved)
                {
                    NCollection_Sequence<IntersectionPoint2D> pnts2d = new NCollection_Sequence<IntersectionPoint2D>();
                    NCollection_Sequence<Point3D>                     pnts3d = new NCollection_Sequence<Point3D>();
                    theAdvAnalyzer.CheckSelfIntersectingEdge(num, pnts2d, pnts3d);
                    if (!theAdvAnalyzer.LastCheckStatus(ExtendStatus.Done))
                    {
                        break;
                    }
                    // points3d.Append(pnts3d);
                    // points2d.Append(pnts2d);
                    points3d = pnts3d;
                    points2d = pnts2d;
                    BrepBuilder b = new BrepBuilder();
                    b.UpdateEdge(e, c2d, Face(), 0.0);
                    b.Range(e, Face(), f2d, l2d);
                    // newtol+=Precision();
                }
                else
                {
                    break;
                }
            }
        }

        //===============================================
        // RemoveLoopMode = 1 , insert vertex
        //===============================================
        if (_removeLoopMode == 1)
        {
            // after fixing will be nb+1 edges
            bool loopRemoved;
            // create a sequence of resulting edges
            NCollection_HSequence<Shape> ttss = new NCollection_HSequence<Shape>();
            Edge                         e2   = new Edge();

            loopRemoved = false;
            //: k3 abv 24 Dec 98: BUC50070 #26682 and #30087: try to remove loop
            if (_geomMode)
            {
                if (c2d is null)
                {
                    sae.PCurve(e, Face(), out c2d, out f2d, out l2d, false);
                }
                Edge e1 = new Edge();
                if (RemoveLoop(e, Face(), points2d.Value(1), ref e1, ref e2))
                {
                    _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                    loopRemoved = true;
                    if (!e1.IsNull)
                    {
                        ttss.Append(e1);
                        newtol = Max(Tool.Tolerance(e1), Tool.Tolerance(e2));
                    }
                    else
                    {
                        newtol = Tool.Tolerance(e2);
                    }
                }
            }

            ttss.Append(e2);

            if (newtol > MaxTolerance())
            {
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            }

            ExtendWireData sewd = new ExtendWireData();
            for (int i = 1; i <= ttss.Length(); i++)
            {
                sewd.Add(ShapeCasts.Edge(ttss.Value(i)));
            }
            if (Context() is not null)
            {
                Context()!.Replace(e, sewd.InternalWire());
                UpdateWire();
            }
            else
            {
                WireData()!.Remove(num > 0 ? num : NbEdges());
                WireData()!.Add(sewd.InternalWire(), num > 0 ? num : NbEdges());
            }
            if (loopRemoved)
            {
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE8);
            }
        }

        if (LastFixStatus(ExtendStatus.Done) && !_shape.IsNull)
        {
            SendWarning(e, new Msg("FixAdvWire.FixIntersection.MSG5")); // Edge was self-intersecting, corrected
        }

        return LastFixStatus(ExtendStatus.Done);
    }

    private static double ComputeLocalDeviation(in Edge edge, in Point3D pint, in Point3D pnt, double f, double l,
                                                in Face face)
    {
        EdgeAnalysis sae = new EdgeAnalysis();
        if (!sae.Curve3d(edge, out Curve3D? c3d, out double a, out double b, false))
        {
            return RealLast();
        }

        Line3D line = new Line3D(pint, new Direction3D(new Vector3D(pint, pnt)));

        if (sae.PCurve(edge, face, out Curve2D? crv, out double fp, out double lp, false))
        {
            if (crv is TrimmedCurve2D tc)
            {
                if (tc.BasisCurve() is LineCurve2D)
                {
                    f = a + (f - fp) * (b - a) / (lp - fp);
                    l = a + (l - fp) * (b - a) / (lp - fp);
                }
            }
        }

        const int nseg = 10;
        double    step = (l - f) / nseg;
        double    dev  = 0.0;
        for (int i = 1; i < nseg; i++)
        {
            Point3D p = c3d!.Value(f + i * step);
            double d = line.Distance(p);
            if (dev < d)
            {
                dev = d;
            }
        }
        return dev;
    }

    private bool FixIntersectingEdges(int num)
    {
        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady || NbEdges() < 2)
        {
            return false;
        }

        // analysis
        NCollection_Sequence<IntersectionPoint2D> points2d       = new NCollection_Sequence<IntersectionPoint2D>();
        NCollection_Sequence<Point3D>                     points3d       = new NCollection_Sequence<Point3D>();
        NCollection_Sequence<double>                     errors         = new NCollection_Sequence<double>();
        WireAnalysis?                              theAdvAnalyzer = _analyzer;
        if (theAdvAnalyzer is null)
        {
            return false;
        }
        theAdvAnalyzer.CheckIntersectingEdges(num, points2d, points3d, errors);
        if (theAdvAnalyzer.LastCheckStatus(ExtendStatus.Fail))
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
        }
        if (!theAdvAnalyzer.LastCheckStatus(ExtendStatus.Done))
        {
            return false;
        }

        // rln 03/02/98: CSR#BUC50004 entity 56 (to avoid later inserting lacking edge)
        //: l0  bool isLacking = myAnalyzer->CheckLacking ( num );

        // action: increase tolerance of vertex

        ExtendWireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.EdgeCount);
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.EdgeCount);
        Edge          e1   = sbwd.Edge(n1);
        Edge          e2   = sbwd.Edge(n2);
        if (Context() is not null)
        {
            e1 = ShapeCasts.Edge(Context()!.InternalApply(sbwd.Edge(n1)));
            e2 = ShapeCasts.Edge(Context()!.InternalApply(sbwd.Edge(n2)));
        }

        bool isForward1 = (e1.Orientation() == Forward);
        bool isForward2 = (e2.Orientation() == Forward);
        Tool.Range(e1, Face(), out double a1, out double b1);
        Tool.Range(e2, Face(), out double a2, out double b2);

        EdgeAnalysis sae = new EdgeAnalysis();
        Vertex      vp  = sae.FirstVertex(e1);
        Vertex      v1  = sae.LastVertex(e1);
        Vertex      v2  = sae.FirstVertex(e2);
        Vertex      vn  = sae.LastVertex(e2);

        double tol = Tool.Tolerance(v1);
        Point3D pnt = Tool.Pnt(v1);

        double prevRange1 = RealLast(), prevRange2 = RealLast();
        bool   cutEdge1 = false, cutEdge2 = false;
        bool   isCutLine     = false;
        bool   isChangedEdge = false;

        BrepBuilder b = new BrepBuilder();

        int nb = points3d.Length();
        for (int i = 1; i <= nb; i++)
        {
            IntersectionPoint2D ip     = points2d.Value(i);
            double                     param1 = (num == 1 ? ip.ParamOnSecond() : ip.ParamOnFirst());
            double                     param2 = (num == 1 ? ip.ParamOnFirst() : ip.ParamOnSecond());

            double newRange1 = Math.Abs((isForward1 ? a1 : b1) - param1);
            double newRange2 = Math.Abs((isForward2 ? b2 : a2) - param2);
            if (newRange1 > prevRange1 && newRange2 > prevRange2)
            {
                continue;
            }

            Point3D pint   = points3d.Value(i);
            double rad    = errors.Value(i);
            double newtol = 1.0001 * (pnt.Distance(pint) + rad);

            // GeomAdaptor_Surface& Ads = myAnalyzer->Surface()->Adaptor3d()->ChangeSurface();

            //: r8 abv 12 Apr 99: try increasing tolerance of edge

            bool locMayEdit = _topoMode;
            // Always try to modify the tolerance firstly as a better solution
            if (/*! myTopoMode &&*/ newtol > tol)
            {
                double te1 =
                    rad + ComputeLocalDeviation(e1, pint, pnt, param1, (isForward1 ? b1 : a1), Face());
                double te2 =
                    rad + ComputeLocalDeviation(e2, pint, pnt, (isForward2 ? a2 : b2), param2, Face());
                double maxte = Max(te1, te2);
                if (maxte < MaxTolerance() && maxte < newtol)
                {
                    if (Tool.Tolerance(e1) < te1 || Tool.Tolerance(e2) < te2)
                    {
                        // Make copy of edges.
                        if (Context() is not null)
                        {
                            isChangedEdge = true; // To avoid double copying of vertexes.

                            // Intersection point of two base edges.
                            BuildEdgeBuilder sbe = new BuildEdgeBuilder();
                            Vertex   vv1  = Context()!.CopyVertex(v1);

                            Vertex vVp = vp;
                            Vertex vVn = vn;
                            if (vp.IsSame(vn))
                            {
                                // Should modify only one vertex.
                                vVp = Context()!.CopyVertex(vp);
                                vVn = vVp;
                            }
                            else
                            {
                                vVp = Context()!.CopyVertex(vp);
                                vVn = Context()!.CopyVertex(vn);
                            }

                            Edge ee1 = sbe.InternalCopyReplaceVertices(e1, vVp, vv1);
                            Edge ee2 = sbe.InternalCopyReplaceVertices(e2, vv1, vVn);

                            Context()!.Replace(e1, ee1);
                            Context()!.Replace(e2, ee2);

                            UpdateWire();
                            e1 = sbwd.Edge(n1);
                            e2 = sbwd.Edge(n2);
                            vp = sae.FirstVertex(e1);
                            v1 = sae.LastVertex(e1);
                            v2 = sae.FirstVertex(e2);
                            vn = sae.LastVertex(e2);
                        }

                        b.UpdateEdge(e1, 1.000001 * te1);
                        b.UpdateVertex(sae.FirstVertex(e1), 1.000001 * te1);
                        b.UpdateVertex(sae.LastVertex(e1), 1.000001 * te1);
                        b.UpdateEdge(e2, 1.000001 * te2);
                        b.UpdateVertex(sae.FirstVertex(e2), 1.000001 * te2);
                        b.UpdateVertex(sae.LastVertex(e2), 1.000001 * te2);

                        _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE6);
                        locMayEdit = false;
                    }
                    newtol = 1.000001 * maxte;
                }
            }

            if (locMayEdit || newtol <= MaxTolerance())
            {
                prevRange1 = newRange1;
                prevRange2 = newRange2;
                if (locMayEdit)
                {
                    newtol = 1.0001 * (pnt.Distance(pint) + rad);
                    //: j6 abv 7 Dec 98: ProSTEP TR10 r0601_id.stp #57676 & #58586: do not cut edges because of
                    //: influence on adjacent faces
                    SplitToolFixer tool = new SplitToolFixer();

                    if (!tool.CutEdge(e1, (isForward1 ? a1 : b1), param1, Face(), ref isCutLine))
                    {
                        if (v1.IsSame(vp))
                        {
                            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                        }
                        else
                        {
                            locMayEdit = false;
                        }
                    }
                    else
                    {
                        cutEdge1 = true; //: h4
                    }

                    if (!tool.CutEdge(e2, (isForward2 ? b2 : a2), param2, Face(), ref isCutLine))
                    {
                        if (v2.IsSame(vn))
                        {
                            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                        }
                        else
                        {
                            locMayEdit = false;
                        }
                    }
                    else
                    {
                        cutEdge2 = true; //: h4
                    }
                }

                if (locMayEdit && newRange1 <= prevRange1 && newRange2 <= prevRange2 && // rln 09/01/98
                    Tool.SameParameter(e1) && Tool.SameParameter(e2))
                {
                    _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                    pnt = pint;
                    if (tol <= rad)
                    {
                        _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                        tol = 1.001 * rad;
                    }
                }
                else
                {
                    if (isCutLine)
                    {
                        _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                        pnt = pint;
                        if (tol <= rad)
                        {
                            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                            tol = 1.001 * rad;
                        }
                    }
                    else
                    { // else increase tolerance
                        if (tol < newtol)
                        { // rln 07.04.99 CCI60005-brep.igs
                            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                            tol = newtol;
                        }
                    }
                }
            }
            else
            {
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            }
        }

        if (!LastFixStatus(ExtendStatus.Done))
        {
            return false;
        }

        if (isChangedEdge)
        {
            b.UpdateVertex(v1, pnt, tol);
            b.UpdateVertex(v2, pnt, tol);
        }
        else
        {
            if (Context() is not null)
            {
                if (v1.IsSame(v2))
                {
                    Context()!.CopyVertex(v1, pnt, tol);
                }
                else
                {
                    Context()!.CopyVertex(v1, pnt, tol);
                    Context()!.CopyVertex(v2, pnt, tol);
                }
            }
            else
            {
                b.UpdateVertex(v1, pnt, tol);
                b.UpdateVertex(v2, pnt, tol);
            }
        }

        //: h4: make edges SP (after all cuts: t4mug.stp #3730+#6460)
        if (cutEdge1)
        {
            if (Context() is not null)
            {
                e1 = ShapeCasts.Edge(Context()!.InternalApply(e1));
            }
            _fixEdge.FixSameParameter(e1);
        }
        if (cutEdge2 && !isCutLine)
        {
            if (Context() is not null)
            {
                e2 = ShapeCasts.Edge(Context()!.InternalApply(e2));
            }
            _fixEdge.FixSameParameter(e2);
        }
        if (cutEdge1 || cutEdge2)
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE7);
        }
        if (!_shape.IsNull)
        {
            SendWarning(new Msg("FixAdvWire.FixIntersection.MSG10")); // Edges were intersecting, corrected
        }
        return true;
    }

    // pdn 17.03.99 fixing non adjacent intersection by increasing tolerance of vertex

    private bool FixIntersectingEdges(int num1, int num2)
    {
        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }
        NCollection_Sequence<IntersectionPoint2D> points2d       = new NCollection_Sequence<IntersectionPoint2D>();
        NCollection_Sequence<Point3D>                     points3d       = new NCollection_Sequence<Point3D>();
        NCollection_Sequence<double>                     errors         = new NCollection_Sequence<double>();
        WireAnalysis?                              theAdvAnalyzer = _analyzer;
        if (theAdvAnalyzer is null)
        {
            return false;
        }
        theAdvAnalyzer.CheckIntersectingEdges(num1, num2, points2d, points3d, errors);
        if (theAdvAnalyzer.LastCheckStatus(ExtendStatus.Fail))
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
        }
        if (!theAdvAnalyzer.LastCheckStatus(ExtendStatus.Done))
        {
            return false;
        }
        NCollection_Array1<Point3D>       vertexPoints = new NCollection_Array1<Point3D>(1, 4);
        NCollection_Array1<double>       vertexTolers = new NCollection_Array1<double>(1, 4);
        NCollection_Array1<double>       newTolers    = new NCollection_Array1<double>(1, 4);
        NCollection_Array1<Shape> vertices     = new NCollection_Array1<Shape>(1, 4);
        newTolers.Init(0);

        ExtendWireData sbwd = WireData()!;
        int                  n2   = (num1 > 0 ? num1 : sbwd.EdgeCount);
        int                  n1   = (num2 > 1 ? num2 : sbwd.EdgeCount);
        if (n1 == n2)
        {
            return false;
        }

        Edge edge1 = sbwd.Edge(n1);
        Edge edge2 = sbwd.Edge(n2);

        EdgeAnalysis sae = new EdgeAnalysis();
        vertices[1] = sae.FirstVertex(edge1);
        vertices[2] = sae.LastVertex(edge1);
        vertices[3] = sae.FirstVertex(edge2);
        vertices[4] = sae.LastVertex(edge2);

        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= 4; i++)
        {
            vertexPoints[i] = Tool.Pnt(ShapeCasts.Vertex(vertices[i]));
            vertexTolers[i] = Tool.Tolerance(ShapeCasts.Vertex(vertices[i]));
        }

        double newTolEdge1 = 0.0, newTolEdge2 = 0.0;
        int    nb = points3d.Length();
        for (i = 1; i <= nb; i++)
        {
            Point3D pint = points3d.Value(i);

            // searching for the nearest vertexies to the intersection point
            double vtx1Param = 0.0, vtx2Param = 0.0;
            int    vc1, vc2;
            double minDist          = RealLast();
            Point3D nearestVertex    = new Point3D();
            double necessaryVtxTole = 0.0;
            for (vc1 = 1; vc1 <= 2; vc1++)
            {
                for (vc2 = 3; vc2 <= 4; vc2++)
                {

                    double vtxIPDist  = pint.Distance(vertexPoints[vc1]);
                    double vtxVtxDist = vertexPoints[vc1].Distance(vertexPoints[vc2]);
                    if (minDist > vtxIPDist && vtxIPDist > vtxVtxDist)
                    {
                        necessaryVtxTole = vtxVtxDist;
                        nearestVertex    = vertexPoints[vc1];
                        minDist          = vtxIPDist;
                        vtx1Param        = Tool.Parameter(ShapeCasts.Vertex(vertices[vc1]), edge1);
                        vtx2Param        = Tool.Parameter(ShapeCasts.Vertex(vertices[vc2]), edge2);
                    }
                }
            }

            // calculation of necessary tolerances of edges
            IntersectionPoint2D ip     = points2d.Value(i);
            double                     param1 = ip.ParamOnFirst();
            double                     param2 = ip.ParamOnSecond();
            Curve3D?                curve1, curve2;
            curve1 = Tool.Curve(edge1, out Placement l1, out _, out _);
            curve2 = Tool.Curve(edge2, out Placement l2, out _, out _);

            // if aMinDist lower than resolution than the intersection point lyes inside the vertex
            if (minDist < Constants.Resolution())
            {
                continue;
            }

            double maxEdgeTol1 = 0.0, maxEdgeTol2 = 0.0;
            if (minDist < RealLast() && curve1 is not null && curve2 is not null)
            {
                Line3D lig = new Line3D(nearestVertex, new Direction3D(new Vector3D(nearestVertex, pint)));
                int    pointsC;
                double du1   = 0.05 * (param1 - vtx1Param);
                double du2   = 0.05 * (param2 - vtx2Param);
                double tole1 = Tool.Tolerance(edge1);
                double tole2 = Tool.Tolerance(edge2);
                for (pointsC = 2; pointsC < 19; pointsC++)
                {
                    double u  = vtx1Param + pointsC * du1;
                    Point3D p1 = curve1.Value(u);
                    p1.Transform(l1.Transformation());
                    double d1 = lig.Distance(p1) * 2.0000001;
                    if (d1 > tole1 && d1 > maxEdgeTol1)
                    {
                        maxEdgeTol1 = d1;
                    }

                    u         = vtx2Param + pointsC * du2;
                    Point3D p2 = curve2.Value(u);
                    p2.Transform(l2.Transformation());
                    double d2 = lig.Distance(p2) * 2.0000001;
                    if (d2 > tole2 && d2 > maxEdgeTol2)
                    {
                        maxEdgeTol2 = d2;
                    }
                }
                if (maxEdgeTol1 == 0.0 && maxEdgeTol2 == 0.0)
                {
                    continue;
                }
                // if the vertexies are far than tolerances so
                // we do not need to increase edge tolerance
                if (necessaryVtxTole > Max(maxEdgeTol1, tole1)
                    || necessaryVtxTole > Max(maxEdgeTol2, tole2))
                {
                    maxEdgeTol1 = 0.0;
                    maxEdgeTol2 = 0.0;
                }
            }

            double rad    = errors.Value(i);
            double finTol = RealLast();
            int    rank   = 1;
            for (int j = 1; j <= 4; j++)
            {
                double newtol = 1.0001 * (pint.Distance(vertexPoints[j]) + rad);
                if (newtol < finTol)
                {
                    rank   = j;
                    finTol = newtol;
                }
            }
            if (finTol <= MaxTolerance())
            {
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                if (newTolers[rank] < finTol)
                {
                    if (Max(maxEdgeTol1, maxEdgeTol2) < finTol && (maxEdgeTol1 > 0 || maxEdgeTol2 > 0))
                    {
                        newTolEdge1 = Max(newTolEdge1, maxEdgeTol1);
                        newTolEdge2 = Max(newTolEdge2, maxEdgeTol2);
                    }
                    else
                    {
                        newTolers[rank] = finTol;
                    }
                }
            }
            else
            {
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
            }
        }

        BrepBuilder b = new BrepBuilder();
        // update of tolerances of edges
        if (newTolEdge1 > 0)
        {
            for (i = 1; i <= 2; i++)
            {
                if (newTolEdge1 > Max(vertexTolers[i], newTolers[i]))
                {
                    newTolers[i] = newTolEdge1;
                }
            }
            b.UpdateEdge(edge1, newTolEdge1);
        }
        if (newTolEdge2 > 0)
        {
            for (i = 3; i <= 4; i++)
            {
                if (newTolEdge2 > Max(vertexTolers[i], newTolers[i]))
                {
                    newTolers[i] = newTolEdge2;
                }
            }
            b.UpdateEdge(edge2, newTolEdge2);
        }

        // update of tolerances of vertexies
        for (i = 1; i <= 4; i++)
        {
            if (newTolers[i] > 0)
            {
                b.UpdateVertex(ShapeCasts.Vertex(vertices[i]), newTolers[i]);
            }
        }

        if (!_shape.IsNull)
        {
            SendWarning(new Msg("FixAdvWire.FixIntersection.MSG10")); // Edges were intersecting, corrected
        }
        return true;
    }

    //=======================================================================
    // function : FixLacking
    // purpose  : Test if two adjucent edges are disconnected in 2d (while connected
    //           in 3d), and in that case either increase tolerance of the vertex or
    //           add a new edge (straight in 2d space), in order to close wire in 2d.
    //           Returns True if edge was added or tolerance was increased.
    // NOTE     : Is to be run after FixDegenerated
    // Algorithm: 1. Compute the 2d gap between edges and calculate a tolerance
    //              which should have vertex in order to comprise the gap
    //              (using GeomAdaptor_Surface); computed value is inctol
    //           2. If inctol < tol of vertex, return False (everything is OK)
    //           3. If inctol < Precision, just increase tolerance of vertex to inctol
    //           4. Else (if both edges are not degenerated) try to add new edge
    //              with straight pcurve (in order to close the gap):
    //              a) if flag MayEdit is False
    //                 1. if inctol < MaxTolerance, increase tolerance of vertex to inctol
    //                 2. else try to add degenerated edge (check that middle point of
    //                    that pcurveis inside the vertex)
    //              b) if MayEdit is True
    //                 1. try to replace big vertex with two new small vertices
    //                    connected by new edge. This is made if there is a 3d space
    //                    between ends of adjacent edges.
    //                 2. if inctol < MaxTolerance, increase tolerance of vertex to inctol
    //                 3. else add either degenerated or closed edge (if middle point
    //                    of a pcurve of a new edge is inside the vertex, then
    //                    degenerated edge is added, else new edge is closed).
    //           5. If new edge cannot be added, but inctol < MaxTolerance,
    //              when increase tolerance of vertex to a value of inctol
    // Short list of some internal variables:
    // tol    - tolerance of vertex
    // tol2d  - tolerance in parametric space of the surface corresponding to 2*tol
    // dist2d - distance between ends of pcurves of edges (2d)
    // inctol - tolerance required for vertex to close 2d gap (=tol*dist2d/tol2d)
    // tol1, tol2 - tolerances of edges, tol0 = tol1 + tol2
    // p3d1, p3d2 - ends of 3d curves of edges
    //=======================================================================
    //: h2 abv 28 May 98: merged modifications by abv 22 Apr 98, gka 27 May 98
    // and pdn 25 May 98 concerning lacking closed or degenerated edges
    // Example files: r0501_pe #107813, UKI60107-6 250, UKI60107-3 1577.

    //: s2 abv 21 Apr 99: add functionality for bending pcurve
    private static bool TryBendingPCurve(in Edge e, in Face face, in Point2D p2d, bool end,
                                         ref Curve2D? c2d, ref double first, ref double last, ref double tol)
    {
        EdgeAnalysis sae = new EdgeAnalysis();
        if (!sae.PCurve(e, face, out c2d, out first, out last, false))
        {
            return false;
        }

        {
            try
            {
                BSplineCurve2D? bs;
                if (c2d is BSplineCurve2D)
                {
                    bs = c2d.Copy() as BSplineCurve2D;
                }
                else // if ( c2d->IsKind(STANDARD_TYPE(Geom2d_Line)) )
                {
                    TrimmedCurve2D trim = new TrimmedCurve2D(c2d!, first, last);
                    bs                       = Geom2dConvert2D.CurveToBSplineCurve(trim);
                }
                if (bs is null)
                {
                    return false;
                }
                // OCCT.NET: frozen: the poles are moved and the curve segmented in place; a copy above in OCCT too,
                // but guard against a converter that hands back a shared curve
                if (bs.IsFrozen)
                {
                    bs = (BSplineCurve2D)bs.Copy();
                }

                double par = (end ? last : first);
                if (Math.Abs(bs.FirstParameter() - par) < Kerf.Engine.Foundation.Precision.PConfusion()
                    && bs.Multiplicity(1) > bs.Degree())
                {
                    bs.SetPole(1, p2d);
                }
                else if (Math.Abs(bs.LastParameter() - par) < Kerf.Engine.Foundation.Precision.PConfusion()
                         && bs.Multiplicity(bs.KnotCount) > bs.Degree())
                {
                    bs.SetPole(bs.PoleCount, p2d);
                }
                else
                {
                    bs.Segment(first, last);
                    if (Math.Abs(bs.FirstParameter() - par) < Kerf.Engine.Foundation.Precision.PConfusion()
                        && bs.Multiplicity(1) > bs.Degree())
                    {
                        bs.SetPole(1, p2d);
                    }
                    else if (Math.Abs(bs.LastParameter() - par) < Kerf.Engine.Foundation.Precision.PConfusion()
                             && bs.Multiplicity(bs.KnotCount) > bs.Degree())
                    {
                        bs.SetPole(bs.PoleCount, p2d);
                    }
                    else
                    {
                        return false;
                    }
                }
                c2d = bs;

                if (!TryNewPCurve(e, face, ref c2d, ref first, ref last, ref tol))
                {
                    return false;
                }
            }
            catch (FailureException)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Fixes a lacking edge between the edge <paramref name="num"/> and the one before it (connected in 3d, not in
    /// 2d): increases the vertex tolerance, bends the pcurves, or adds an edge (straight in 2d).
    /// </summary>
    public bool FixLacking(int num, bool force = false)
    {
        EnterWork(); // OCCT.NET: frozen
        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        //=============
        // First phase: analysis whether the problem (gap) exists
        Point2D p2d1 = new Point2D(), p2d2 = new Point2D();
        _analyzer.CheckLacking(num, (force ? Precision() : 0.0), ref p2d1, ref p2d2);
        if (_analyzer.LastCheckStatus(ExtendStatus.Fail))
        {
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
        }
        if (!_analyzer.LastCheckStatus(ExtendStatus.Done))
        {
            return false;
        }

        //=============
        // Second phase: collection of data necessary for further analysis

        ExtendWireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.EdgeCount);
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.EdgeCount);
        Edge          e1   = sbwd.Edge(n1);
        Edge          e2   = sbwd.Edge(n2);

        EdgeAnalysis sae = new EdgeAnalysis();
        Vertex      v1  = sae.LastVertex(e1);
        Vertex      v2  = sae.FirstVertex(e2);
        double             tol = Max(Tool.Tolerance(v1), Tool.Tolerance(v2));

        double prec   = Precision();
        double dist2d = _analyzer.MaxDistance2d();
        double inctol = _analyzer.MaxDistance3d();

        Face            face = _analyzer.Face();
        SurfaceAnalysis? surf = _analyzer.Surface();

        Point3D p3d1 = new Point3D(), p3d2 = new Point3D();
        double tol1 = Kerf.Engine.Foundation.Precision.Confusion(), tol2 = Kerf.Engine.Foundation.Precision.Confusion(); // SK

        //=============
        //: s2 abv 21 Apr 99: Speculation: try bending pcurves
        double        bendtol1 = 0.0, bendtol2 = 0.0;
        Curve2D? bendc1 = null, bendc2 = null;
        double        bendf1 = 0.0, bendl1 = 0.0, bendf2 = 0.0, bendl2 = 0.0;
        if (_geomMode && !Tool.IsClosed(e1, face) && !Tool.IsClosed(e2, face))
        {
            Point2D p2d = new Point2D(0.5 * (p2d1.XY() + p2d2.XY()));
            bool     ok1 = TryBendingPCurve(e1,
                                        face,
                                        p2d,
                                        e1.Orientation() == Forward,
                                        ref bendc1,
                                        ref bendf1,
                                        ref bendl1,
                                        ref bendtol1);
            bool     ok2 = TryBendingPCurve(e2,
                                        face,
                                        p2d,
                                        e2.Orientation() == Reversed,
                                        ref bendc2,
                                        ref bendf2,
                                        ref bendl2,
                                        ref bendtol2);
            if (ok1 && !ok2)
            {
                bendtol2 = Tool.Tolerance(e2);
                ok1      = TryBendingPCurve(e1,
                                       face,
                                       p2d2,
                                       e1.Orientation() == Forward,
                                       ref bendc1,
                                       ref bendf1,
                                       ref bendl1,
                                       ref bendtol1);
            }
            else if (!ok1 && ok2)
            {
                bendtol1 = Tool.Tolerance(e1);
                ok2      = TryBendingPCurve(e2,
                                       face,
                                       p2d1,
                                       e2.Orientation() == Forward,
                                       ref bendc2,
                                       ref bendf2,
                                       ref bendl2,
                                       ref bendtol2);
            }
            if (!ok1 && !ok2)
            {
                bendc1 = null;
            }
        }

        //=============
        // Third phase: analyse how to fix the problem

        // selector of solutions
        bool doIncrease  = false; // increase tolerance
        bool doAddLong   = false; // add long 3d edge in replacement of a vertex
        bool doAddClosed = false; // add closed 3d edge
        bool doAddDegen  = false; // add degenerated edge
        bool doBend      = false; //: s2 bend pcurves

        // if bending is OK with existing tolerances of edges, take it
        if (bendc1 is not null && bendc2 is not null
            && ((bendtol1 < Tool.Tolerance(e1) && bendtol2 < Tool.Tolerance(e2))
                || (inctol < prec && bendtol1 < inctol && bendtol2 < inctol)))
        {
            doBend = true;

            // is it OK just to increase tolerance (to a value less than preci)?
        }
        else if (inctol < prec)
        {
            doIncrease = true;

            // If increase is not OK or force, try to find other solutions (adding edge)
        }
        else if (!Tool.Degenerated(e2) && !Tool.Degenerated(e1))
        {

            // analyze the 3d space btw edges: is it enough to add long 3d edge?
            if (_topoMode)
            {
                if (!sae.Curve3d(e1, out Curve3D? c3d, out double a, out double b, true))
                { // cannot work
                    _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                    return false;
                }
                p3d1             = c3d!.Value(b);
                double dist2d3d1 = p3d1.Distance(surf!.Value(p2d1));
                if (!sae.Curve3d(e2, out c3d, out a, out b, true))
                { // cannot work
                    _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                    return false;
                }
                p3d2             = c3d!.Value(a);
                double dist2d3d2 = p3d2.Distance(surf.Value(p2d2));

                tol1 = Max(Tool.Tolerance(e1), dist2d3d1);
                tol2 = Max(Tool.Tolerance(e2), dist2d3d2);
                //: c5  double tol0 = Max ( tol1 + tol2, thepreci );
                double tol0    = tol1 + tol2; //: c5 abv 26 Feb 98: CTS17806 #44418
                double dist3d2 = p3d1.SquareDistance(p3d2);

                // is it OK to add a long 3d edge?
                if (!_analyzer.LastCheckStatus(ExtendStatus.DONE2)
                    && //: 81 abv 20 Jan 98: don`t add back-going edges (zigzags)
                    dist3d2 > 1.25 * tol0 * tol0
                    && (force || dist3d2 > prec * prec || inctol > MaxTolerance()))
                {
                    doAddLong = true;
                }
            }

            //: h6 abv 25 Jun 98: BUC40132 6361: try to increase tol up to MaxTol if not add
            if (!doAddLong && inctol < MaxTolerance()
                && !_analyzer.Surface()!.IsDegenerated(p2d1, p2d2, 2.0 * tol, 10.0))
            { //: p7
                if (bendc1 is not null && bendc2 is not null && bendtol1 < inctol && bendtol2 < inctol)
                {
                    doBend = true;
                }
                else
                {
                    doIncrease = true;
                }
            }
            else

            // else try to add either degenerated or closed edge
            if (!doAddLong)
            {
                Point3D pV = new Point3D(0.5 * (Tool.Pnt(v1).XYZ() + Tool.Pnt(v2).XYZ()));
                Point3D pm = _analyzer.Surface()!.Value(new Point2D(0.5 * (p2d1.XY() + p2d2.XY())));

                double dist = pV.Distance(pm);
                if (dist <= tol)
                {
                    doAddDegen = true;
                }
                else if (_topoMode)
                {
                    doAddClosed = true;
                }
                else if (dist <= MaxTolerance())
                { //: r7 abv 12 Apr 99: t3d_opt.stp #14245 after S4136
                    doAddDegen = true;
                    doIncrease = true;
                    inctol     = dist;
                }
            }
        }

        else if (!Tool.Degenerated(e2) && Tool.Degenerated(e1))
        {
            // create new degenerated edge and replace E1 to new edge
        }
        else if (Tool.Degenerated(e2) && !Tool.Degenerated(e1))
        {
            // create new degenerated edge and replace E2 to new edge
        }

        //=============
        // Third phase - do the fixes
        BrepBuilder B = new BrepBuilder();

        // add edge
        if (doAddLong || doAddDegen || doAddClosed)
        {

            // construct new vertices
            Vertex newV1, newV2;
            if (doAddLong)
            {
                newV1 = new MakeVertex(p3d1).InternalVertex();
                newV1.Reverse();
                newV2 = new MakeVertex(p3d2).InternalVertex();
                B.UpdateVertex(newV1, 1.001 * tol1);
                B.UpdateVertex(newV2, 1.001 * tol2);
            }
            else
            {
                newV1 = v1;
                newV2 = v2;
            }

            // prepare new edge
            Edge edge = new Edge();
            B.MakeEdge(ref edge);
            if (doAddDegen)
            {
                B.Degenerated(edge, true); // sln: do it before adding curve
            }
            Vector2D    v12       = new Vector2D(p2d1, p2d2);
            LineCurve2D theLine2d = new LineCurve2D(p2d1, new Direction2D(v12));
            B.UpdateEdge(edge, theLine2d, face, Kerf.Engine.Foundation.Precision.Confusion());
            B.Range(edge, face, 0, dist2d);
            B.Add(edge, newV1.Oriented(Forward));
            B.Add(edge, newV2.Oriented(Reversed));
            BuildEdgeBuilder sbe = new BuildEdgeBuilder();
            if (!doAddDegen && !sbe.BuildCurve3d(edge))
            {
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
                return false;
            }

            // if long edge is added, replace vertices of adjacent edges
            if (doAddLong)
            {

                // replace 1st edge (n1==n2 - special case: wire consists of one edge)
                Edge edge1 = sbe.InternalCopyReplaceVertices(e1, (n1 == n2 ? newV2 : new Vertex()), newV1);
                sbwd.Set(edge1, n1);
                if (Context() is not null)
                {
                    Context()!.Replace(e1, edge1);
                    // actually, this will occur only in context of single face
                    // hence, recording to ReShape is rather for tracking modifications
                    // than for keeping sharing
                    Context()!.Replace(v1, newV1.Oriented(v1.Orientation()));
                    if (!v1.IsSame(v2))
                    {
                        Context()!.Replace(v2, newV2.Oriented(v2.Orientation()));
                    }
                }
                // replace 2nd edge
                if (n1 != n2)
                {
                    Edge edge2 = sbe.InternalCopyReplaceVertices(e2, newV2, new Vertex());
                    sbwd.Set(edge2, n2);
                    if (Context() is not null)
                    {
                        Context()!.Replace(e2, edge2);
                    }
                }
                if (Context() is not null)
                {
                    UpdateWire();
                }
            }

            // insert new edge
            if (doAddDegen)
            {
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
            }
            else if (!doAddLong)
            {
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
            }
            sbwd.Add(edge, n2);
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        }

        // else try to increase tol up to MaxTol
        else if (inctol > tol && inctol < MaxTolerance())
        {
            if (bendc1 is not null && bendc2 is not null && bendtol1 < inctol && bendtol2 < inctol)
            {
                doBend = true;
            }
            else
            {
                doIncrease = true;
            }
        }

        // bend pcurves
        if (doBend)
        { //: s2 abv 21 Apr 99
            B.UpdateEdge(e1, bendc1, face, bendtol1);
            B.Range(e1, face, bendf1, bendl1);
            B.UpdateEdge(e2, bendc2, face, bendtol2);
            B.Range(e2, face, bendf2, bendl2);
            B.UpdateVertex(sae.FirstVertex(e1), bendtol1);
            B.UpdateVertex(sae.LastVertex(e1), bendtol1);
            B.UpdateVertex(sae.FirstVertex(e2), bendtol2);
            B.UpdateVertex(sae.LastVertex(e2), bendtol2);
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5);
            //: s3 abv 22 Apr 99: PRO7187 #11534: self-intersection not detected unitil curve is bent (!)
            FixSelfIntersectingEdge(n1);
            FixSelfIntersectingEdge(n2);
            FixIntersectingEdges(n2); // skl 24.04.2003 for OCC58
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE5);
        }

        // increase vertex tolerance
        if (doIncrease)
        {
            B.UpdateVertex(v1, 1.001 * inctol);
            B.UpdateVertex(v2, 1.001 * inctol);
            _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
        }

        if (LastFixStatus(ExtendStatus.Done))
        {
            return true;
        }

        _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
        return false;
    }

    /// <summary>Fixes notched edges (an edge going back along its neighbour): splits it and removes the notch.</summary>
    public bool FixNotchedEdges()
    {
        EnterWork(); // OCCT.NET: frozen
        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (!IsReady)
        {
            return false;
        }

        WireAnalysis theAdvAnalyzer = _analyzer;
        Face        face           = Face();
        if (Context() is not null)
        {
            UpdateWire();
        }
        ExtendWireData sewd = WireData()!;

        for (int i = 1; i <= NbEdges() && NbEdges() > 2; i++)
        {
            double param    = 0;
            int    toRemove = 0;
            if (theAdvAnalyzer.CheckNotchedEdges(i, ref toRemove, ref param, MinTolerance()))
            {
                int                n2            = (i > 0) ? i : NbEdges();
                int                n1            = (n2 > 1) ? n2 - 1 : NbEdges();
                bool               isRemoveFirst = (n1 == toRemove);
                int                toSplit       = (n2 == toRemove ? n1 : n2);
                Edge        splitE        = sewd.Edge(toSplit);
                EdgeAnalysis sae           = new EdgeAnalysis();
                sae.PCurve(splitE, face, out Curve2D? c2d, out double a, out double b, true);
                BuildEdgeBuilder    sbe    = new BuildEdgeBuilder();
                Orientation orient = splitE.Orientation();

                // check whether the whole edges should be removed - this is the case
                // when split point coincides with the end of the edge;
                // for closed edges split point may fall at the other end (see issue #0029780)
                if (Math.Abs(param - (isRemoveFirst ? b : a)) <= Kerf.Engine.Foundation.Precision.PConfusion()
                    || (sae.IsClosed3d(splitE)
                        && Math.Abs(param - (isRemoveFirst ? a : b)) <= Kerf.Engine.Foundation.Precision.PConfusion()))
                {
                    FixDummySeam(n1);
                    // The seam edge is removed from the list. So, need to step back to avoid missing of edge
                    // processing
                    i--;
                }
                else // perform splitting of the edge and adding to wire
                {
                    // pdn check if it is necessary
                    if (Math.Abs((isRemoveFirst ? a : b) - param) < Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        continue;
                    }

                    TransferParametersProjAnalysis transferParameters = new TransferParametersProjAnalysis();
                    transferParameters.SetMaxTolerance(MaxTolerance());
                    transferParameters.Init(splitE, face);
                    double first, last;
                    if (a < b)
                    {
                        first = a;
                        last  = b;
                    }
                    else
                    {
                        first = b;
                        last  = a;
                    }
                    Vertex vnew = new Vertex();
                    BrepBuilder  B    = new BrepBuilder();
                    B.MakeVertex(ref vnew,
                                 Analyzer().Surface()!.Value(c2d!.Value(param)),
                                 Kerf.Engine.Foundation.Precision.Confusion());
                    Edge wE = splitE;
                    wE.Orientation(Forward);
                    Shape tmpShape = vnew.Oriented(Reversed); // for porting
                    Edge newE1 =
                        sbe.InternalCopyReplaceVertices(wE, sae.FirstVertex(wE), ShapeCasts.Vertex(tmpShape));
                    sbe.CopyPCurves(newE1, wE);
                    transferParameters.TransferRange(newE1, first, param, true);
                    B.SameRange(newE1, false);
                    B.SameParameter(newE1, false);
                    tmpShape = vnew.Oriented(Forward);
                    Edge newE2 =
                        sbe.InternalCopyReplaceVertices(wE, ShapeCasts.Vertex(tmpShape), sae.LastVertex(wE));
                    sbe.CopyPCurves(newE2, wE);
                    transferParameters.TransferRange(newE2, param, last, true);
                    B.SameRange(newE2, false);
                    B.SameParameter(newE2, false);

                    if (Context() is not null)
                    {
                        Wire wire = new Wire();
                        B.MakeWire(ref wire);
                        B.Add(wire, newE1);
                        B.Add(wire, newE2);
                        Context()!.Replace(wE, wire);
                    }

                    newE1.Orientation(orient);
                    newE2.Orientation(orient);
                    if (orient == Reversed)
                    {
                        Edge tmp = newE2;
                        newE2           = newE1;
                        newE1           = tmp;
                    }

                    bool isRemoveLast = ((n1 == NbEdges()) && (n2 == 1));
                    sewd.Set(newE1, toSplit);
                    sewd.Add(newE2, (toSplit == NbEdges() ? 0 : toSplit + 1));

                    FixDummySeam(isRemoveLast ? NbEdges() : toRemove);
                    _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                }

                i--;
                if (Context() is not null)
                { // skl 07.03.2002 for OCC180
                    UpdateWire();
                }
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
            }
        }
        _statusNotches = _lastFixStatus;
        return LastFixStatus(ExtendStatus.Done);
    }

    private static void CopyReversePcurves(in Edge toedge, in Edge fromedge, bool reverse)
    {
        Placement fromLoc = fromedge.Location();
        Placement toLoc   = toedge.Location();
        // OCCT.NET: frozen: the source edge's list is only read (OCCT walks ChangeCurves())
        for (var fromitcr = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)fromedge.TShape()!).Curves());
             fromitcr.More();
             fromitcr.Next())
        {
            if (fromitcr.Value() is not GCurve fromGC)
            {
                continue;
            }
            if (fromGC.IsCurveOnSurface())
            {
                Surface?                              surface = fromGC.Surface();
                Placement                            l       = fromGC.Location();
                bool                                       found   = false;
                NCollection_List<CurveRepresentation> tolist  = ((BrepTEdge)toedge.TShape()!).ChangeCurves();
                GCurve?                               toGC    = null;
                for (var toitcr = new NCollection_List<CurveRepresentation>.Iterator(tolist);
                     toitcr.More() && !found;
                     toitcr.Next())
                {
                    toGC = toitcr.Value() as GCurve;
                    if (toGC is null || !toGC.IsCurveOnSurface() || surface != toGC.Surface()
                        || l != toGC.Location())
                    {
                        continue;
                    }
                    found = true;
                    break;
                }
                if (!found)
                {
                    double fp = fromGC.First();
                    double lp = fromGC.Last();
                    toGC      = (GCurve)fromGC.Copy();
                    tolist.Append(toGC);
                    Curve2D pcurve = (Curve2D)fromGC.PCurve()!.Copy();
                    if (reverse)
                    {
                        fp = pcurve.ReversedParameter(fp);
                        lp = pcurve.ReversedParameter(lp);
                        pcurve.Reverse();
                        double tmp = fp;
                        fp         = lp;
                        lp         = tmp;
                    }
                    // bug OCC209 invalid location of pcurve in the edge after copying
                    Placement newLoc = (fromLoc * l).Predivided(toLoc);
                    toGC.SetRange(fp, lp);
                    toGC.PCurve(pcurve);
                    toGC.Location(newLoc);
                    if (fromGC.IsCurveOnClosedSurface)
                    {
                        Curve2D pcurve2 = fromGC.PCurve2()!;
                        toGC.PCurve2((Curve2D)pcurve2.Copy());
                    }
                }
            }
        }
    }

    //  Note:    HasNewPCurves, which FixDummySeam's commented-out code would call, is commented out in OCCT too.

    private void FixDummySeam(int num)
    {
        EdgeAnalysis   sae  = new EdgeAnalysis();
        BuildEdgeBuilder      sbe  = new BuildEdgeBuilder();
        VertexBuilder    sbv  = new VertexBuilder();
        int                  num1 = (num == NbEdges()) ? 1 : num + 1;
        ExtendWireData sewd = WireData()!;
        Edge          e1 = sewd.Edge(num), e2 = sewd.Edge(num1);
        Vertex        v1 = sae.FirstVertex(e1), v2 = sae.LastVertex(e2);
        Vertex        vm = sbv.InternalCombineVertex(v1, v2, 1.0001);

        // pnd defining if new pcurves exists
        // pdn Temporary not removed
        //  bool toRemove = !(HasNewPCurves(E1,E2)||HasNewPCurves(E2,E1));
        bool toRemove = false;

        // creating new edge with pcurves and new vertex
        Vertex vs = sae.FirstVertex(e2);
        if (vs.IsSame(v1) || vs.IsSame(v2))
        {
            vs = vm;
        }
        Edge newEdge = sbe.InternalCopyReplaceVertices(e2, vs, vm);
        CopyReversePcurves(newEdge, e1, e1.Orientation() == e2.Orientation());
        BrepBuilder b = new BrepBuilder();
        b.SameRange(newEdge, false);
        b.SameParameter(newEdge, false);

        if (Context() is not null)
        {
            if (toRemove)
            {
                Context()!.Remove(e2);
                Context()!.Remove(e1);
            }
            else
            {
                Context()!.Replace(e2, newEdge);
                Context()!.Replace(e1, newEdge.Reversed());
            }
            Context()!.Replace(v1, vm.Oriented(v1.Orientation()));
            Context()!.Replace(v2, vm.Oriented(v2.Orientation()));
        }

        int         next  = (num1 == NbEdges()) ? 1 : num1 + 1;
        int         prev  = (num > 1) ? num - 1 : NbEdges();
        Edge prevE = sewd.Edge(prev), nextE = sewd.Edge(next);

        Edge tmpE1 = sbe.InternalCopyReplaceVertices(prevE, new Vertex(), vm);
        sewd.Set(tmpE1, prev);
        if (Context() is not null)
        {
            Context()!.Replace(prevE, tmpE1);
        }

        tmpE1 = sbe.InternalCopyReplaceVertices(nextE, vm, new Vertex());
        sewd.Set(tmpE1, next);
        if (Context() is not null)
        {
            Context()!.Replace(nextE, tmpE1);
        }

        // removing edges from wire
        int n1, n2;
        if (num < num1)
        {
            n1 = num;
            n2 = num1;
        }
        else
        {
            n1 = num1;
            n2 = num;
        }
        sewd.Remove(n2);
        sewd.Remove(n1);
    }

    /// <summary>Updates the edges of the wire data with the context's replacements.</summary>
    protected void UpdateWire()
    {
        ExtendWireData sbwd = WireData()!;
        for (int i = 1; i <= sbwd.EdgeCount; i++)
        {
            Edge  e = sbwd.Edge(i);
            Shape s = Context()!.InternalApply(e);
            if (s == e)
            {
                continue;
            }
            for (Explorer exp = new Explorer(s, ShapeType.Edge); exp.More(); exp.Next())
            {
                sbwd.Add(exp.Current(), i++);
            }
            sbwd.Remove(i--);
        }
    }

    /// <summary>Fixes tails (thin spikes between adjacent edges), with the maximal angle and width set.</summary>
    public bool FixTails()
    {
        EnterWork(); // OCCT.NET: frozen
        if (_maxTailWidth < 0 || !IsReady)
        {
            return false;
        }

        _lastFixStatus = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (Context() is not null)
        {
            UpdateWire();
        }
        ExtendWireData sewd   = WireData()!;
        int                  eCount = NbEdges();
        int[]                eNs    = [eCount, 1];
        bool                 checkAngle = true;
        while (eCount >= 2 && eNs[1] <= eCount)
        {
            Edge[] es     = [sewd.Edge(eNs[0]), sewd.Edge(eNs[1])];
            var           eParts = new Edge[2, 2];
            for (int k = 0; k < 2; ++k)
            {
                eParts[k, 0] = new Edge();
                eParts[k, 1] = new Edge();
            }
            // OCCT.NET: frozen: the internal form leaves the cut parts unfrozen, since they go into the working wire
            if (!_analyzer.InternalCheckTail(es[0],
                                              es[1],
                                              checkAngle ? _maxTailAngleSine : -1,
                                              _maxTailWidth,
                                              MaxTolerance(),
                                              ref eParts[0, 0],
                                              ref eParts[0, 1],
                                              ref eParts[1, 0],
                                              ref eParts[1, 1]))
            {
                eNs[0]     = eNs[1]++;
                checkAngle = true;
                continue;
            }

            // Provide not less than 1 edge in the result wire.
            int[] splitCounts = [eParts[0, 1].IsNull ? 0 : 1, eParts[1, 1].IsNull ? 0 : 1];
            int   removeCount = (eParts[0, 0].IsNull ? 0 : 1) + (eParts[1, 0].IsNull ? 0 : 1);
            if (eCount + splitCounts[0] + splitCounts[1] < 1 + removeCount)
            {
                eNs[0]     = eNs[1]++;
                checkAngle = true;
                continue;
            }

            // Split the edges.
            for (int ei = 0; ei < 2; ++ei)
            {
                if (splitCounts[ei] == 0)
                {
                    continue;
                }

                // Replace the edge by the wire of its parts in the shape.
                Edge e = es[ei];
                if (Context() is not null)
                {
                    Wire eWire = new Wire();
                    new BrepBuilder().MakeWire(ref eWire);
                    new BrepBuilder().Add(eWire, eParts[ei, 0]);
                    new BrepBuilder().Add(eWire, eParts[ei, 1]);
                    Edge fe = ShapeCasts.Edge(e.Oriented(Forward));
                    Context()!.Replace(fe, eWire);
                }

                // Replace the edge by its parts in the edge wire.
                Orientation orient = e.Orientation();
                eParts[ei, 0].Orientation(orient);
                eParts[ei, 1].Orientation(orient);
                int firstPI = (orient != Reversed) ? 0 : 1;
                int add     = (ei == 0 || eNs[1] < eNs[0]) ? 0 : splitCounts[0];
                sewd.Set(eParts[ei, firstPI], eNs[ei] + add);
                sewd.Add(eParts[ei, 1 - firstPI], eNs[ei] + 1 + add);
            }

            // Remove the tail.
            if (removeCount == 2)
            {
                checkAngle = true;
                FixDummySeam(eNs[0] + splitCounts[0] + ((eNs[0] < eNs[1]) ? 0 : splitCounts[1]));
                if (Context() is not null)
                {
                    UpdateWire();
                }
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.Done);

                if (splitCounts[0] + splitCounts[1] == 2)
                {
                    eNs[0] = eNs[1]++;
                    continue;
                }

                if (splitCounts[0] == splitCounts[1])
                {
                    eCount -= 2;
                    if (eNs[1] >= 3)
                    {
                        --eNs[0];
                        --eNs[1];
                    }
                    else
                    {
                        eNs[0] = eCount;
                        eNs[1] = 1;
                    }
                    checkAngle = false;
                }
                else
                {
                    --eCount;
                    if (splitCounts[0] != 0)
                    {
                        eNs[0] = (eNs[0] <= eCount) ? eNs[0] : eCount;
                    }
                    else
                    {
                        if (eNs[1] >= 3)
                        {
                            --eNs[0];
                            --eNs[1];
                        }
                        else
                        {
                            eNs[0] = eCount;
                            eNs[1] = 1;
                        }
                    }
                }
            }
            else
            {
                checkAngle = false;
                --eCount;
                int ri = eParts[0, 0].IsNull ? 1 : 0;
                if (splitCounts[ri] != 0)
                {
                    if (ri == 0)
                    {
                        if (eNs[1] >= 3)
                        {
                            --eNs[0];
                            --eNs[1];
                        }
                        else
                        {
                            eNs[0] = eCount;
                            eNs[1] = 1;
                        }
                    }
                    else
                    {
                        eNs[0] = (eNs[1] > 1) ? eNs[0] : eCount;
                    }
                }
                sewd.Remove(eNs[ri] + ((ri != 0 || splitCounts[0] == 0) ? 0 : 1));
                if (Context() is not null)
                {
                    Context()!.Remove(es[ri].Oriented(Forward));
                    UpdateWire();
                }
                _lastFixStatus |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.Done);
            }
        }
        // OCCT: the tails' status goes into myStatusNotches; StatusFixTails is never set
        _statusNotches = _lastFixStatus;
        return global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_lastFixStatus, ExtendStatus.Done);
    }
}
