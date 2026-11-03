// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

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
public partial class ShapeFix_Wire : ShapeFix_Root
{
    protected ShapeFix_Edge      myFixEdge;
    protected ShapeAnalysis_Wire myAnalyzer;
    protected bool               myGeomMode;
    protected bool               myTopoMode;
    protected bool               myClosedMode;
    protected bool               myPreference2d;
    protected bool               myFixGapsByRanges;
    protected int                myFixReversed2dMode;
    protected int                myFixRemovePCurveMode;
    protected int                myFixAddPCurveMode;
    protected int                myFixRemoveCurve3dMode;
    protected int                myFixAddCurve3dMode;
    protected int                myFixSeamMode;
    protected int                myFixShiftedMode;
    protected int                myFixSameParameterMode;
    protected int                myFixVertexToleranceMode;
    protected int                myFixNotchedEdgesMode;
    protected int                myFixSelfIntersectingEdgeMode;
    protected int                myFixIntersectingEdgesMode;
    protected int                myFixNonAdjacentIntersectingEdgesMode;
    protected int                myFixTailMode;
    protected int                myRemoveLoopMode;
    protected int                myFixReorderMode;
    protected int                myFixSmallMode;
    protected int                myFixConnectedMode;
    protected int                myFixEdgeCurvesMode;
    protected int                myFixDegeneratedMode;
    protected int                myFixSelfIntersectionMode;
    protected int                myFixLackingMode;
    protected int                myFixGaps3dMode;
    protected int                myFixGaps2dMode;
    protected int                myLastFixStatus;
    protected int                myStatusReorder;
    protected int                myStatusSmall;
    protected int                myStatusConnected;
    protected int                myStatusEdgeCurves;
    protected int                myStatusDegenerated;
    protected int                myStatusClosed;
    protected int                myStatusSelfIntersection;
    protected int                myStatusLacking;
    protected int                myStatusGaps3d;
    protected int                myStatusGaps2d;
    protected bool               myStatusRemovedSegment;
    protected int                myStatusNotches;
    protected int                myStatusFixTails;
    protected double             myMaxTailAngleSine;
    protected double             myMaxTailWidth;

    // ---- ShapeFix_Wire.lxx ----

    /// <summary>Sets the face of the wire.</summary>
    public void SetFace(in Face face)
    {
        myAnalyzer.SetFace(face);
    }

    /// <summary>Sets the face of the wire with an analyser of its surface.</summary>
    public void SetFace(in Face theFace, ShapeAnalysis_Surface? theSurfaceAnalysis)
    {
        myAnalyzer.SetFace(theFace, theSurfaceAnalysis);
    }

    /// <summary>Sets the surface analyser (and through it the surface) of the wire.</summary>
    public void SetSurface(ShapeAnalysis_Surface? theSurfaceAnalysis)
    {
        myAnalyzer.SetSurface(theSurfaceAnalysis);
    }

    /// <summary>Sets the surface of the wire.</summary>
    public void SetSurface(Surface? surf)
    {
        myAnalyzer.SetSurface(surf);
    }

    /// <summary>Sets the surface of the wire with a location.</summary>
    public void SetSurface(Surface? surf, in Placement loc)
    {
        myAnalyzer.SetSurface(surf, loc);
    }

    /// <summary>True if a wire is loaded.</summary>
    public bool IsLoaded() => myAnalyzer.IsLoaded();

    /// <summary>True if a wire and a face are loaded.</summary>
    public bool IsReady() => myAnalyzer.IsReady();

    /// <summary>Returns the analyser of the wire.</summary>
    public ShapeAnalysis_Wire Analyzer() => myAnalyzer;

    /// <summary>
    /// Returns the working wire data: the loaded edges, which the fixes edit (between the first fix and a public
    /// result, copies of a frozen wire's edges).
    /// </summary>
    public ShapeExtend_WireData? WireData() => myAnalyzer.WireData();

    /// <summary>Returns the face of the wire.</summary>
    public Face Face() => myAnalyzer.Face();

    /// <summary>The mode allowing the fixes to modify the topology (default false).</summary>
    public ref bool ModifyTopologyMode() => ref myTopoMode;

    /// <summary>The mode allowing the fixes to modify the geometry (default true).</summary>
    public ref bool ModifyGeometryMode() => ref myGeomMode;

    /// <summary>The mode for removing loops (-1 default, 0 keep, 1 remove).</summary>
    public ref int ModifyRemoveLoopMode() => ref myRemoveLoopMode;

    /// <summary>The mode telling that the wire is to be closed (default true).</summary>
    public ref bool ClosedWireMode() => ref myClosedMode;

    /// <summary>The mode preferring pcurves to 3d curves for gaps (default true).</summary>
    public ref bool PreferencePCurveMode() => ref myPreference2d;

    /// <summary>The mode fixing gaps by ranges (default false).</summary>
    public ref bool FixGapsByRangesMode() => ref myFixGapsByRanges;

    public ref int FixReorderMode() => ref myFixReorderMode;

    public ref int FixSmallMode() => ref myFixSmallMode;

    public ref int FixConnectedMode() => ref myFixConnectedMode;

    public ref int FixEdgeCurvesMode() => ref myFixEdgeCurvesMode;

    public ref int FixDegeneratedMode() => ref myFixDegeneratedMode;

    public ref int FixReversed2dMode() => ref myFixReversed2dMode;

    public ref int FixRemovePCurveMode() => ref myFixRemovePCurveMode;

    public ref int FixRemoveCurve3dMode() => ref myFixRemoveCurve3dMode;

    public ref int FixAddPCurveMode() => ref myFixAddPCurveMode;

    public ref int FixAddCurve3dMode() => ref myFixAddCurve3dMode;

    public ref int FixSeamMode() => ref myFixSeamMode;

    public ref int FixShiftedMode() => ref myFixShiftedMode;

    public ref int FixSameParameterMode() => ref myFixSameParameterMode;

    public ref int FixVertexToleranceMode() => ref myFixVertexToleranceMode;

    public ref int FixLackingMode() => ref myFixLackingMode;

    public ref int FixSelfIntersectionMode() => ref myFixSelfIntersectionMode;

    public ref int FixGaps3dMode() => ref myFixGaps3dMode;

    public ref int FixGaps2dMode() => ref myFixGaps2dMode;

    public ref int FixNotchedEdgesMode() => ref myFixNotchedEdgesMode;

    public ref int FixSelfIntersectingEdgeMode() => ref myFixSelfIntersectingEdgeMode;

    public ref int FixIntersectingEdgesMode() => ref myFixIntersectingEdgesMode;

    public ref int FixNonAdjacentIntersectingEdgesMode() => ref myFixNonAdjacentIntersectingEdgesMode;

    public ref int FixTailMode() => ref myFixTailMode;

    public bool StatusReorder(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusReorder, status);

    public bool StatusSmall(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusSmall, status);

    public bool StatusConnected(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusConnected, status);

    public bool StatusEdgeCurves(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusEdgeCurves, status);

    public bool StatusDegenerated(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusDegenerated, status);

    public bool StatusLacking(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusLacking, status);

    public bool StatusSelfIntersection(ShapeExtend_Status status) =>
        ShapeExtend.DecodeStatus(myStatusSelfIntersection, status);

    public bool StatusGaps3d(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusGaps3d, status);

    public bool StatusGaps2d(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusGaps2d, status);

    public bool StatusClosed(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusClosed, status);

    public bool StatusNotches(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusNotches, status);

    public bool StatusFixTails(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatusFixTails, status);

    /// <summary>The status of the last advanced fix (by edge number).</summary>
    public bool LastFixStatus(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myLastFixStatus, status);

    /// <summary>Returns the tool fixing the edges.</summary>
    public ShapeFix_Edge FixEdgeTool() => myFixEdge;

    /// <summary>True if a segment was removed by the last fix of self-intersection.</summary>
    public bool StatusRemovedSegment() => myStatusRemovedSegment;

    // ---- ShapeFix_Wire.cxx ----

    // #######################################################################
    //   Constructors, initializations, modes, querying
    // #######################################################################

    /// <summary>Empty constructor: default modes, no wire loaded.</summary>
    public ShapeFix_Wire()
    {
        myMaxTailAngleSine = 0;
        myMaxTailWidth     = -1;
        myFixEdge          = new ShapeFix_Edge();
        myAnalyzer         = new ShapeAnalysis_Wire();
        ClearModes();
        ClearStatuses();
        myStatusRemovedSegment = false;
    }

    /// <summary>Creates the tool and loads a wire on a face with a precision.</summary>
    public ShapeFix_Wire(in Wire wire, in Face face, double prec)
    {
        myMaxTailAngleSine = 0;
        myMaxTailWidth     = -1;
        myFixEdge          = new ShapeFix_Edge();
        myAnalyzer         = new ShapeAnalysis_Wire();
        ClearModes();
        SetMaxTolerance(prec);
        myStatusRemovedSegment = false;
        Init(wire, face, prec);
    }

    /// <summary>Sets the working precision (also of the analyser).</summary>
    public override void SetPrecision(double prec)
    {
        base.SetPrecision(prec);
        myAnalyzer.SetPrecision(prec);
    }

    /// <summary>Sets the maximal angle of tails (FixTails).</summary>
    public void SetMaxTailAngle(double theMaxTailAngle)
    {
        myMaxTailAngleSine = Math.Sin(theMaxTailAngle);
        myMaxTailAngleSine = (myMaxTailAngleSine >= 0) ? myMaxTailAngleSine : 0;
    }

    /// <summary>Sets the maximal width of tails (FixTails).</summary>
    public void SetMaxTailWidth(double theMaxTailWidth)
    {
        myMaxTailWidth = theMaxTailWidth;
    }

    /// <summary>Sets all the modes to their defaults.</summary>
    public void ClearModes()
    {
        myTopoMode        = false;
        myGeomMode        = true;
        myClosedMode      = true;
        myPreference2d    = true;
        myFixGapsByRanges = false;

        myRemoveLoopMode = -1;

        myFixReversed2dMode      = -1;
        myFixRemovePCurveMode    = -1;
        myFixRemoveCurve3dMode   = -1;
        myFixAddPCurveMode       = -1;
        myFixAddCurve3dMode      = -1;
        myFixSeamMode            = -1;
        myFixShiftedMode         = -1;
        myFixSameParameterMode   = -1;
        myFixVertexToleranceMode = -1;

        myFixNotchedEdgesMode                 = -1;
        myFixSelfIntersectingEdgeMode         = -1;
        myFixIntersectingEdgesMode            = -1;
        myFixNonAdjacentIntersectingEdgesMode = -1;
        myFixTailMode                         = 0;

        myFixReorderMode          = -1;
        myFixSmallMode            = -1;
        myFixConnectedMode        = -1;
        myFixEdgeCurvesMode       = -1;
        myFixDegeneratedMode      = -1;
        myFixSelfIntersectionMode = -1;
        myFixLackingMode          = -1;
        myFixGaps3dMode           = -1;
        myFixGaps2dMode           = -1;
    }

    /// <summary>Clears all the statuses.</summary>
    public void ClearStatuses()
    {
        int emptyStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);

        myLastFixStatus = emptyStatus;

        myStatusReorder          = emptyStatus;
        myStatusSmall            = emptyStatus;
        myStatusConnected        = emptyStatus;
        myStatusEdgeCurves       = emptyStatus;
        myStatusDegenerated      = emptyStatus;
        myStatusSelfIntersection = emptyStatus;
        myStatusLacking          = emptyStatus;
        myStatusGaps3d           = emptyStatus; // szv#9:S4244:19Aug99 new method introduced
        myStatusGaps2d           = emptyStatus; // szv#9:S4244:19Aug99 new method introduced
        myStatusClosed           = emptyStatus;
        myStatusNotches          = emptyStatus;
        myStatusFixTails         = emptyStatus;
    }

    /// <summary>Loads a wire, sets its face and the precision.</summary>
    public void Init(in Wire wire, in Face face, double prec)
    {
        Load(wire);
        SetFace(face);
        SetPrecision(prec);
    }

    /// <summary>Takes the analyser (with its wire, face and precision) as the tool's own.</summary>
    public void Init(ShapeAnalysis_Wire saw)
    {
        ClearStatuses();
        myAnalyzer = saw;
        myShape.Nullify();
        //  SetPrecision ( saw.Precision() );
        myToEnter = true; // OCCT.NET: frozen
    }

    /// <summary>Loads a wire (the context, if any, applied to it first).</summary>
    public void Load(in Wire wire)
    {
        ClearStatuses();

        Wire W = wire;
        if (Context() is not null)
        {
            Shape S = Context()!.InternalApply(wire);
            W = ShapeCasts.Wire(S);
        }

        myAnalyzer.Load(W);
        myShape   = wire;
        myToEnter = true; // OCCT.NET: frozen
    }

    /// <summary>Loads a wire data object (the context, if any, applied to its edges).</summary>
    public void Load(ShapeExtend_WireData sbwd)
    {
        ClearStatuses();
        myAnalyzer.Load(sbwd);
        if (Context() is not null)
        {
            UpdateWire();
        }
        myShape.Nullify();
        myToEnter = true; // OCCT.NET: frozen
    }

    /// <summary>Returns the number of edges in the working wire.</summary>
    public int NbEdges()
    {
        ShapeExtend_WireData? sbwd = myAnalyzer.WireData();
        return sbwd is null ? 0 : sbwd.NbEdges();
    }

    // #######################################################################
    //   Fixing methods of API level
    // #######################################################################

    /// <summary>
    /// Performs all the available fixes. A fix turned on or off explicitly by its mode is called or not accordingly;
    /// with the default mode it is called depending on the situation (some are skipped or limited if the order of the
    /// edges isn't OK).
    /// </summary>
    public bool Perform(ProgressRange theProgress = default)
    {
        enterWork(); // OCCT.NET: frozen
        ClearStatuses();
        if (!IsLoaded())
        {
            return false;
        }

        if (Context() is not null)
        {
            myFixEdge.SetContext(Context());
        }

        bool Fixed = false;

        // FixReorder is first, because as a rule wire is required to be ordered
        // We shall analyze the order of edges in the wire and set appropriate
        // status even if FixReorder should not be called (if it is forbidden)

        var  sawo      = new ShapeAnalysis_WireOrder();
        bool ReorderOK = !myAnalyzer.CheckOrder(sawo, myClosedMode);
        if (NeedFix(myFixReorderMode, !ReorderOK))
        {
            if (FixReorder())
            {
                Fixed = true;
            }
            ReorderOK = !StatusReorder(ShapeExtend_FAIL);
        }

        if (theProgress.UserBreak())
        {
            return false;
        }

        // FixSmall is allowed to change topology only if mode is set and FixReorder
        // did not failed
        if (NeedFix(myFixSmallMode, myTopoMode))
        {
            if (FixSmall(!myTopoMode || !ReorderOK, MinTolerance()) != 0)
            {
                Fixed = true;
                // retry reorder if necessary (after FixSmall it can work better)
                if (NeedFix(myFixReorderMode, !ReorderOK))
                {
                    FixReorder();
                    ReorderOK = !StatusReorder(ShapeExtend_FAIL);
                }
            }
        }

        if (theProgress.UserBreak())
        {
            return false;
        }

        if (NeedFix(myFixConnectedMode, ReorderOK))
        {
            if (FixConnected())
            {
                Fixed = true;
            }
        }

        if (theProgress.UserBreak())
        {
            return false;
        }

        if (NeedFix(myFixEdgeCurvesMode))
        {
            int savFixShiftedMode = myFixShiftedMode;
            // turn out FixShifted if reorder not done
            if (myFixShiftedMode == -1 && !ReorderOK)
            {
                myFixShiftedMode = 0;
            }
            if (FixEdgeCurves())
            {
                Fixed = true;
            }
            myFixShiftedMode = savFixShiftedMode;
        }

        if (theProgress.UserBreak())
        {
            return false;
        }

        if (NeedFix(myFixDegeneratedMode))
        {
            if (FixDegenerated())
            {
                Fixed = true; // ?? if ! ReorderOK ??
            }
        }

        if (theProgress.UserBreak())
        {
            return false;
        }

        // pdn - temporary to test
        if (myFixTailMode <= 0 && NeedFix(myFixNotchedEdgesMode, ReorderOK))
        {
            Fixed |= FixNotchedEdges();
            if (Fixed)
            {
                FixShifted(); // skl 07.03.2002 for OCC180
            }
        }

        if (theProgress.UserBreak())
        {
            return false;
        }

        if (myFixTailMode != 0)
        {
            if (FixTails())
            {
                Fixed = true;
                FixShifted();
            }
        }

        if (theProgress.UserBreak())
        {
            return false;
        }

        if (NeedFix(myFixSelfIntersectionMode, myClosedMode))
        {
            int savFixIntersectingEdgesMode = myFixIntersectingEdgesMode;
            // switch off FixIntEdges if reorder not done
            if (myFixIntersectingEdgesMode == -1 && !ReorderOK)
            {
                myFixIntersectingEdgesMode = 0;
            }
            if (FixSelfIntersection())
            {
                Fixed = true;
            }
            FixReorder();
            myFixIntersectingEdgesMode = savFixIntersectingEdgesMode;
        }

        if (theProgress.UserBreak())
        {
            return false;
        }

        if (NeedFix(myFixLackingMode, ReorderOK))
        {
            if (FixLacking())
            {
                Fixed = true;
            }
        }

        if (theProgress.UserBreak())
        {
            return false;
        }

        // TEMPORARILY without special mode !!!
        ShapeExtend_WireData sbwd = WireData()!;
        for (int iedge = 1; iedge <= sbwd.NbEdges(); iedge++)
        {
            if (myFixEdge.FixVertexTolerance(sbwd.Edge(iedge), Face()))
            {
                Fixed = true;
            }
        }

        if (theProgress.UserBreak())
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
    /// (<paramref name="theModeBoth"/>, for bi-periodic surfaces) the order uses 3d and 2d data.
    /// </summary>
    public bool FixReorder(bool theModeBoth = false)
    {
        enterWork(); // OCCT.NET: frozen
        myStatusReorder = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsLoaded())
        {
            return false;
        }

        // fix in Both mode for bi-periodic surface
        var sawo = new ShapeAnalysis_WireOrder();
        if (myAnalyzer.Surface() is not null && myAnalyzer.Surface()!.Surface()!.IsUPeriodic
            && myAnalyzer.Surface()!.Surface()!.IsVPeriodic && theModeBoth)
        {
            myAnalyzer.CheckOrder(sawo, myClosedMode, true, true);
        }
        else
        {
            myAnalyzer.CheckOrder(sawo, myClosedMode, true, false);
        }

        FixReorder(sawo);

        if (LastFixStatus(ShapeExtend_FAIL))
        {
            myStatusReorder |= ShapeExtend.EncodeStatus(
                LastFixStatus(ShapeExtend_FAIL1) ? ShapeExtend_FAIL1 : ShapeExtend_FAIL2);
        }
        if (!LastFixStatus(ShapeExtend_DONE))
        {
            return false;
        }

        myStatusReorder |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        if (sawo.Status() == 2 || sawo.Status() == -2)
        {
            myStatusReorder |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }
        if (sawo.Status() < 0)
        {
            myStatusReorder |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
        }
        if (sawo.Status() == 3)
        {
            // only shifted
            myStatusReorder |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
        }
        return true;
    }

    /// <summary>
    /// Applies FixSmall(num) to all the edges, from the last; returns 1 if some small edge was fixed (OCCT returns its
    /// bool as an int).
    /// </summary>
    public int FixSmall(bool lockvtx, double precsmall = 0.0)
    {
        enterWork(); // OCCT.NET: frozen
        myStatusSmall = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsLoaded())
        {
            return 0;
        }

        for (int i = NbEdges(); i > 0; i--)
        {
            FixSmall(i, lockvtx, precsmall);
            myStatusSmall |= myLastFixStatus;
        }

        return StatusSmall(ShapeExtend_DONE) ? 1 : 0;
    }

    /// <summary>Applies FixConnected(num) to all the edges (from the last), connecting them by their vertices.</summary>
    public bool FixConnected(double prec = -1.0)
    {
        enterWork(); // OCCT.NET: frozen
        myStatusConnected = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsLoaded())
        {
            return false;
        }

        int                  aStop     = (myClosedMode ? 0 : 1);
        ShapeExtend_WireData aWireSBWD = WireData()!;
        for (int aI = NbEdges(); aI > aStop; aI--)
        {
            FixConnected(aI, prec, false);
            myStatusConnected |= myLastFixStatus;
            // Refresh the edge that the next iteration will analyze as n1.
            if (Context() is null || aI - 1 <= aStop)
            {
                continue;
            }
            int aN1Next = (aI - 1 > 1) ? aI - 2 : aWireSBWD.NbEdges();
            if (aN1Next == aI || aN1Next == aI - 1)
            {
                continue;
            }
            Edge  aEPrev   = aWireSBWD.Edge(aN1Next);
            Shape aRefresh = Context()!.InternalApply(aEPrev);
            if (aRefresh.IsNull || aRefresh.IsSame(aEPrev) || aRefresh.ShapeType() != ShapeType.Edge)
            {
                continue;
            }
            aWireSBWD.Set(ShapeCasts.Edge(aRefresh), aN1Next);
        }

        if (Context() is not null)
        {
            UpdateWire();
        }

        return StatusConnected(ShapeExtend_DONE);
    }

    /// <summary>
    /// Fixes the curves of the edges: reversed 2d curves, missing or invalid pcurves and 3d curves, seams, shifted
    /// pcurves, same parameter and the vertices' tolerances, as the modes allow.
    /// </summary>
    public bool FixEdgeCurves()
    {
        enterWork(); // OCCT.NET: frozen
        myStatusEdgeCurves = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsLoaded())
        {
            return false;
        }
        bool isReady = IsReady();

        ShapeExtend_WireData sbwd = WireData()!;
        int                  i, nb = sbwd.NbEdges();
        Face          face          = Face();
        ShapeFix_Edge        theAdvFixEdge = myFixEdge;
        if (theAdvFixEdge is null)
        {
            myFixReversed2dMode = 0;
        }

        // fix revesred 2d / 3d curves
        if (isReady && NeedFix(myFixReversed2dMode))
        {
            for (i = 1; i <= nb; i++)
            {
                theAdvFixEdge!.FixReversed2d(sbwd.Edge(i), face);
                if (theAdvFixEdge.Status(ShapeExtend_DONE))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                }
                if (theAdvFixEdge.Status(ShapeExtend_FAIL))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                }
            }
        }

        // add / remove pcurve
        if (isReady && NeedFix(myFixRemovePCurveMode, false))
        {
            for (i = 1; i <= nb; i++)
            {
                myFixEdge.FixRemovePCurve(sbwd.Edge(i), face);
                if (myFixEdge.Status(ShapeExtend_DONE))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                }
                if (myFixEdge.Status(ShapeExtend_FAIL))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                }
            }
        }

        if (isReady && NeedFix(myFixAddPCurveMode))
        {
            int overdegen = 0; //: c0
            for (i = 1; i <= nb; i++)
            {
                myFixEdge.FixAddPCurve(sbwd.Edge(i), face, sbwd.IsSeam(i), myAnalyzer.Surface()!, Precision());
                if (myFixEdge.Status(ShapeExtend_DONE))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                }
                if (myFixEdge.Status(ShapeExtend_FAIL))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
                }

                // if ( !sbwd->IsSeam(i) && myFixEdge->Status ( ShapeExtend_DONE2 )
                //      && BRep_Tool::SameParameter(sbwd->Edge(i)) ) {
                if (!sbwd.IsSeam(i) && myFixEdge.Status(ShapeExtend_DONE2))
                {
                    // abv 24 Feb 00: trj3_s1-ac-214.stp #1631 etc.: try to split the edge in singularity
                    if (Context() is not null)
                    {
                        var                   sbe = new ShapeBuild_Edge();
                        Edge           E   = sbwd.Edge(i);
                        var                   SAC = new ShapeAnalysis_Curve();
                        Curve3D            C   = Tool.Curve(E, out double a, out double b)!;
                        ShapeAnalysis_Surface S   = myAnalyzer.Surface()!;
                        int                   nbs = S.NbSingularities(MinTolerance());
                        var                   GAC = new CurveAdaptor(C, a, b);
                        var                   seq = new NCollection_Sequence<double>();
                        for (int j = 1; j <= nbs; j++)
                        {
                            double   Preci = 0.0;
                            Point2D pd1 = new Point2D(), pd2 = new Point2D();
                            Point3D   P3d = new Point3D();
                            double   par1 = 0.0, par2 = 0.0;
                            bool     tmpUIsoDeg = false;
                            S.Singularity(j, ref Preci, ref P3d, ref pd1, ref pd2, ref par1, ref par2, ref tmpUIsoDeg);
                            if (SAC.Project(GAC, P3d, MinTolerance(), out Point3D pr, out double split, true)
                                < Max(Preci, MinTolerance()))
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
                            bool isFwd = (E.Orientation() == Forward);
                            E.Orientation(Forward);

                            // if( BRep_Tool::SameParameter(sbwd->Edge(i)) )
                            //   sbe.RemovePCurve ( E, face );

                            // 10.04.2003 skl for using trimmed lines as pcurves
                            var sae = new ShapeAnalysis_Edge();
                            if (Tool.SameParameter(sbwd.Edge(i)))
                            {
                                sbe.RemovePCurve(E, face);
                            }
                            else
                            {
                                if (sae.HasPCurve(E, face))
                                {
                                    if (sae.PCurve(E, face, out Curve2D? C2d, out double fp2d, out double lp2d))
                                    {
                                        if (C2d is not TrimmedCurve2D)
                                        {
                                            sbe.RemovePCurve(E, face);
                                        }
                                    }
                                }
                            }

                            //    myFixEdge->FixSameParameter ( E ); // to ensure SameRange & SP
                            var           B = new BrepBuilder();
                            Vertex V1, V2, V = new Vertex();
                            // ShapeAnalysis_Edge sae;
                            V1 = sae.FirstVertex(E);
                            V2 = sae.LastVertex(E);

                            var sw = new ShapeExtend_WireData();
                            for (int k = 0; k <= seq.Length(); k++)
                            {
                                double split = (k < seq.Length() ? seq[k + 1] : b);
                                if (k < seq.Length())
                                {
                                    B.MakeVertex(ref V, C.Value(split), Tool.Tolerance(E));
                                    // try increase tolerance before splitting
                                    double aDist = Tool.Pnt(V1).Distance(Tool.Pnt(V));
                                    if (aDist < Tool.Tolerance(V1) * 1.01)
                                    {
                                        B.UpdateVertex(V1, Max(aDist, Tool.Tolerance(V1)));
                                        a  = split;
                                        V1 = V;
                                        continue;
                                    }
                                    else
                                    {
                                        aDist = Tool.Pnt(V2).Distance(Tool.Pnt(V));
                                        if (aDist < Tool.Tolerance(V2) * 1.01)
                                        {
                                            B.UpdateVertex(V, Max(aDist, Tool.Tolerance(V2)));
                                            b  = split;
                                            V2 = V;
                                            continue;
                                        }
                                    }
                                }
                                else
                                {
                                    V = V2;
                                }

                                Edge edge = sbe.InternalCopyReplaceVertices(E, V1, V);
                                if (Tool.SameParameter(sbwd.Edge(i)))
                                {
                                    // TopoDS_Edge edge = sbe.CopyReplaceVertices ( E, V1, V );
                                    B.Range(edge, a, split);
                                    sw.Add(edge);
                                }
                                else
                                {
                                    // TopoDS_Edge edge = sbe.CopyReplaceVertices(sbwd->Edge(i),V1,V);
                                    var sftp = new ShapeAnalysis_TransferParameters(E, face);
                                    sftp.TransferRange(edge, a, split, false);
                                    sw.Add(edge);
                                }
                                // sw->Add(edge);
                                a  = split;
                                V1 = V;
                            }
                            if (!isFwd)
                            {
                                sw.Reverse();
                                E.Orientation(Reversed);
                            }
                            Context()!.Replace(E, sw.InternalWire());
                            UpdateWire();
                            nb = sbwd.NbEdges();
                            i--;
                            continue;
                        }
                    }

                    overdegen = i;
                }
            }

            //: c0 abv 20 Feb 98: treat case of curve going over degenerated pole and seam
            if (overdegen != 0 && myAnalyzer.Surface()!.IsUClosed(Precision()))
            {
                var    sbe = new ShapeBuild_Edge();
                double URange;
                myAnalyzer.Surface()!.Bounds(out double SUF, out double SUL, out double SVF, out double SVL);
                URange = (Math.Abs(SUL - SUF));
                Xy vec = new Xy(0, 0);
                var   sae = new ShapeAnalysis_Edge();
                int   k;
                for (k = 1; k <= nb; k++)
                {
                    if (!sae.PCurve(sbwd.Edge(k), face, out Curve2D? c2d, out double cf, out double cl, true))
                    {
                        break;
                    }
                    vec += c2d!.Value(cl).XY() - c2d.Value(cf).XY();
                }
                if (k > nb && Math.Abs(Math.Abs(vec.X()) - URange) < 0.1 * URange)
                {
                    sbe.RemovePCurve(sbwd.Edge(overdegen), face);
                    myFixEdge.Projector().AdjustOverDegenMode() = 0;
                    myFixEdge.FixAddPCurve(sbwd.Edge(overdegen), face, sbwd.IsSeam(overdegen), myAnalyzer.Surface()!,
                                           Precision());
                }
            }
        }

        // add / remove pcurve
        if (isReady && NeedFix(myFixRemoveCurve3dMode, false))
        {
            for (i = 1; i <= nb; i++)
            {
                myFixEdge.FixRemoveCurve3d(sbwd.Edge(i));
                if (myFixEdge.Status(ShapeExtend_DONE))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                }
                if (myFixEdge.Status(ShapeExtend_FAIL))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL4);
                }
            }
        }
        if (NeedFix(myFixAddCurve3dMode))
        {
            for (i = 1; i <= nb; i++)
            {
                myFixEdge.FixAddCurve3d(sbwd.Edge(i));
                if (myFixEdge.Status(ShapeExtend_DONE))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
                }
                if (myFixEdge.Status(ShapeExtend_FAIL))
                {
                    //: abv 29.08.01: Spatial_firex_lofting.sat: if 3d curve cannot
                    // be built because edge has no pcurves either, remove that edge
                    Tool.CurveOnSurface(sbwd.Edge(i), out Curve2D? C, out Surface? S, out Placement L,
                                             out double first, out double last);
                    if (C is null || Math.Abs(last - first) < Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        // Incomplete edge (with no pcurves or 3d curve) removed
                        SendWarning(sbwd.Edge(i), new Msg("FixWire.FixCurve3d.Removed"));
                        sbwd.Remove(i--);
                        nb--;
                        myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
                        if (i == nb)
                        {
                            FixClosed(Precision());
                        }
                        else
                        {
                            FixConnected(i + 1, Precision());
                        }
                    }
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL5);
                }
            }
        }

        // fix seam
        if (isReady && NeedFix(myFixSeamMode, false))
        {
            for (i = 1; i <= nb; i++)
            {
                FixSeam(i);
                if (LastFixStatus(ShapeExtend_DONE))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE6);
                }
                if (LastFixStatus(ShapeExtend_FAIL))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL6);
                }
            }
        }

        // fix shifted
        if (isReady && NeedFix(myFixShiftedMode))
        {
            FixShifted();
            if (LastFixStatus(ShapeExtend_DONE))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE7);
            }
            if (LastFixStatus(ShapeExtend_FAIL))
            {
                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL7);
            }
        }

        // fix same parameter
        if (isReady && NeedFix(myFixSameParameterMode))
        {
            for (i = 1; i <= nb; i++)
            {
                // skl 28.10.2004 for OCC6366 - check SameRange
                var        sae    = new ShapeAnalysis_Edge();
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
                            Surface? S = Tool.Surface(face, out Placement L);
                            new ShapeBuild_Edge().RemovePCurve(sbwd.Edge(i), S, L);
                            myFixEdge.FixAddPCurve(sbwd.Edge(i), face, sbwd.IsSeam(i), myAnalyzer.Surface()!, Precision());
                            if (myFixEdge.Status(ShapeExtend_DONE))
                            {
                                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                            }
                            if (myFixEdge.Status(ShapeExtend_FAIL))
                            {
                                myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
                            }
                        }
                    }
                }
                myFixEdge.FixSameParameter(sbwd.Edge(i), Face());
                if (myFixEdge.Status(ShapeExtend_DONE))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE8);
                }
                if (myFixEdge.Status(ShapeExtend_FAIL))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL8);
                }
            }
        }

        //: abv 10.06.02: porting C40 -> dev (CC670-12608.stp): moved from Perform()
        // Update with face is needed for plane surfaces (w/o stored pcurves)
        if (NeedFix(myFixVertexToleranceMode))
        {
            for (i = 1; i <= nb; i++)
            {
                myFixEdge.FixVertexTolerance(sbwd.Edge(i), Face());
                if (myFixEdge.Status(ShapeExtend_DONE))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_DONE8);
                }
                if (myFixEdge.Status(ShapeExtend_FAIL))
                {
                    myStatusEdgeCurves |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL8);
                }
            }
            if (Context() is not null)
            {
                UpdateWire();
            }
        }

        return StatusEdgeCurves(ShapeExtend_DONE);
    }

    /// <summary>Applies FixDegenerated(num) to all the edges, and removes duplicated degenerated edges.</summary>
    public bool FixDegenerated()
    {
        enterWork(); // OCCT.NET: frozen
        myStatusDegenerated = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        //  if ( ! myAnalyzer->Surface()->HasSingularities ( Precision() ) ) return;

        int lastcoded = -1, prevcoded = 0;
        int stop = (myClosedMode ? 0 : 1);
        for (int i = NbEdges(); i > stop; i--)
        {
            FixDegenerated(i);
            myStatusDegenerated |= myLastFixStatus;
            //: r0 abv 19 Mar 99: PRO7226.stp #489490: remove duplicated degenerated edges
            int coded = (LastFixStatus(ShapeExtend_DONE2) ? 1 : 0);
            if (lastcoded == -1)
            {
                lastcoded = coded;
            }
            if (coded != 0 && (prevcoded != 0 || (i == 1 && lastcoded != 0)) && NbEdges() > 1)
            {
                ShapeExtend_WireData sbwd = WireData()!;
                var                  B    = new BrepBuilder();
                sbwd.Remove(i);
                if (prevcoded == 0)
                {
                    i = NbEdges();
                }
                B.Degenerated(sbwd.Edge(i++), false);
                prevcoded = 0;
            }
            else
            {
                prevcoded = coded;
            }
        }

        return StatusDegenerated(ShapeExtend_DONE);
    }

    /// <summary>
    /// Applies FixSelfIntersectingEdge and FixIntersectingEdges (adjacent and, with its mode, non-adjacent), and
    /// removes wrong edges.
    /// </summary>
    public bool FixSelfIntersection()
    {
        enterWork(); // OCCT.NET: frozen
        myStatusSelfIntersection = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        ShapeExtend_WireData sbwd = WireData()!;
        int                  nb   = sbwd.NbEdges();

        if (NeedFix(myFixSelfIntersectingEdgeMode))
        {
            if (myRemoveLoopMode < 1)
            {
                for (int num = 1; num <= nb; num++)
                {
                    FixSelfIntersectingEdge(num);
                    myStatusSelfIntersection |= myLastFixStatus;
                }
            }
            else if (myRemoveLoopMode == 1)
            {
                for (int num = 1; num <= nb; num++)
                {
                    FixSelfIntersectingEdge(num);
                    myStatusSelfIntersection |= myLastFixStatus;
                    if (nb < sbwd.NbEdges())
                    {
                        num--;
                    }
                    nb = sbwd.NbEdges();
                }
                FixClosed(Precision());
            }
        }

        if (NeedFix(myFixIntersectingEdgesMode))
        {
            int num = (myClosedMode ? 1 : 2);
            for (; nb > 1 && num <= nb; num++)
            {
                FixIntersectingEdges(num);
                if (LastFixStatus(ShapeExtend_FAIL1))
                {
                    myStatusSelfIntersection |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                }
                if (LastFixStatus(ShapeExtend_FAIL2))
                {
                    myStatusSelfIntersection |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                }
                if (!LastFixStatus(ShapeExtend_DONE))
                {
                    continue;
                }

                if (LastFixStatus(ShapeExtend_DONE1))
                {
                    myStatusSelfIntersection |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                }
                if (LastFixStatus(ShapeExtend_DONE2))
                {
                    myStatusSelfIntersection |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                }
                if (LastFixStatus(ShapeExtend_DONE6))
                {
                    myStatusSelfIntersection |= ShapeExtend.EncodeStatus(ShapeExtend_DONE6);
                }

                if (/*! myTopoMode ||*/ nb < 3)
                {
                    // #86 rln 22.03.99 sim2.igs, entity 4292: After fixing of self-intersecting
                    // BRepCheck finds one more self-intersection not found by ShapeAnalysis
                    //%15 pdn 06.04.99 repeat until fixed CTS18546-2 entity 777

                    // if the tolerance was modified we should recheck the result, if it was enough
                    if (LastFixStatus(ShapeExtend_DONE7))
                    { // num--;
                        FixIntersectingEdges(num);
                    }
                    continue;
                }

                if (LastFixStatus(ShapeExtend_DONE4))
                {
                    sbwd.Remove(num);
                }
                if (LastFixStatus(ShapeExtend_DONE3))
                {
                    sbwd.Remove(num > 1 ? num - 1 : nb + num - 1);
                }
                if (LastFixStatus(ShapeExtend_DONE4) || LastFixStatus(ShapeExtend_DONE3))
                {
                    myStatusSelfIntersection |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
                    num = (myClosedMode ? 1 : 2);
                    nb  = sbwd.NbEdges();
                }
                else
                {
                    // #86 rln 22.03.99
                    //%15 pdn 06.04.99 repeat until fixed CTS18546-2 entity 777
                    FixIntersectingEdges(num);
                    /*if ( LastFixStatus ( ShapeExtend_DONE7 ) )*/
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
        if (NeedFix(myFixNonAdjacentIntersectingEdgesMode))
        {
            var ITool   = new ShapeFix_IntersectionTool(Context(), Precision());
            int NbSplit = 0, NbCut = 0, NbRemoved = 0;
            if (ITool.FixSelfIntersectWire(sbwd, myAnalyzer.Face(), ref NbSplit, ref NbCut, ref NbRemoved))
            {
                myStatusSelfIntersection |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5); // gka 06.09.04
            }
            if (NbSplit > 0 || NbRemoved > 0)
            {
                if (NbRemoved > 0)
                {
                    myStatusRemovedSegment = true;
                }
                // Load(sbwd); commented by skl 29.12.2004 for OCC7624, instead this
                //             string inserted following three strings:
                myAnalyzer.Load(sbwd);
                if (Context() is not null)
                {
                    UpdateWire();
                }
                myShape.Nullify();
            }
            // OCCT's commented-out pairwise check of non-adjacent edges by boxes is left out.
        }

        return StatusSelfIntersection(ShapeExtend_DONE);
    }

    /// <summary>Applies FixLacking(num) to all the edges (from the first, or the second on an open wire).</summary>
    public bool FixLacking(bool force = false)
    {
        enterWork(); // OCCT.NET: frozen
        myStatusLacking = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        int start = (myClosedMode ? 1 : 2);
        for (int i = start; i <= NbEdges(); i++)
        {
            FixLacking(i, force);
            myStatusLacking |= myLastFixStatus;
        }

        return StatusLacking(ShapeExtend_DONE);
    }

    /// <summary>
    /// Fixes the closure of the wire: FixConnected, FixDegenerated and FixLacking between its last and first edges.
    /// </summary>
    public bool FixClosed(double prec = -1.0)
    {
        enterWork(); // OCCT.NET: frozen
        myStatusClosed = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsLoaded() || NbEdges() < 1)
        {
            return false;
        }

        FixConnected(1, prec);
        if (LastFixStatus(ShapeExtend_DONE))
        {
            myStatusClosed |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }
        if (LastFixStatus(ShapeExtend_FAIL))
        {
            myStatusClosed |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }

        FixDegenerated(1);
        if (LastFixStatus(ShapeExtend_DONE))
        {
            myStatusClosed |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }
        if (LastFixStatus(ShapeExtend_FAIL))
        {
            myStatusClosed |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
        }

        FixLacking(1);
        if (LastFixStatus(ShapeExtend_DONE))
        {
            myStatusClosed |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
        }
        if (LastFixStatus(ShapeExtend_FAIL))
        {
            myStatusClosed |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
        }

        return StatusClosed(ShapeExtend_DONE);
    }

    // #######################################################################
    //   Fixing methods of advanced level
    // #######################################################################

    /// <summary>Reorders the edges in the wire as the given wire order says.</summary>
    public bool FixReorder(ShapeAnalysis_WireOrder wi)
    {
        enterWork(); // OCCT.NET: frozen
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsLoaded())
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
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        ShapeExtend_WireData sbwd = WireData()!;
        int                  i, nb = sbwd.NbEdges();
        if (nb != wi.NbEdges())
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            return false;
        }
        // D abord on protege
        for (i = 1; i <= nb; i++)
        {
            if (wi.Ordered(i) == 0)
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
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

        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        return true;
    }

    /// <summary>
    /// Removes the edge <paramref name="num"/> if it is small (shorter than <paramref name="precsmall"/>); if its
    /// vertices differ, only when the topology may be modified and <paramref name="lockvtx"/> is false, and then the
    /// neighbours are connected.
    /// </summary>
    public bool FixSmall(int num, bool lockvtx, double precsmall)
    {
        enterWork(); // OCCT.NET: frozen
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsLoaded() || NbEdges() <= 1)
        {
            return false;
        }

        // analysis:
        ShapeAnalysis_Wire theAdvAnalyzer = myAnalyzer;
        if (theAdvAnalyzer is null)
        {
            return false;
        }
        int n = (num > 0 ? num : NbEdges());
        theAdvAnalyzer.CheckSmall(n, precsmall);
        if (theAdvAnalyzer.LastCheckStatus(ShapeExtend_FAIL))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            //: n2    return false;
        }

        if (!theAdvAnalyzer.LastCheckStatus(ShapeExtend_DONE))
        {
            return false;
        }

        // OUI cette edge est NULLE

        if (theAdvAnalyzer.LastCheckStatus(ShapeExtend_DONE2))
        {
            // edge is small, but vertices are not the same..
            if (lockvtx || !myTopoMode)
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                return false;
            }
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
        }
        else
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }

        // action: remove edge
        if (Context() is not null)
        {
            Context()!.Remove(WireData()!.Edge(n));
        }
        SendWarning(WireData()!.Edge(n), new Msg("FixAdvWire.FixSmall.MSG0")); // Small edge(s) removed
        WireData()!.Remove(n);

        // call FixConnected in the case if vertices of the small edge were not the same
        if (LastFixStatus(ShapeExtend_DONE2))
        {
            int savLastFixStatus = myLastFixStatus;
            // #43 rln 20.11.98 S4054 CTS18544 entity 21734 removing last edge
            FixConnected(n <= NbEdges() ? n : 1, precsmall);
            if (LastFixStatus(ShapeExtend_FAIL))
            {
                savLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
            }
            myLastFixStatus = savLastFixStatus;
        }

        return true;
    }

    /// <summary>
    /// Connects the edge <paramref name="num"/> to the previous one by a common vertex (the closer of the two, or a
    /// combined one), if their ends are within <paramref name="prec"/> (the maximal tolerance when negative).
    /// </summary>
    public bool FixConnected(int num, double prec, bool theUpdateWire = true)
    {
        enterWork(); // OCCT.NET: frozen
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsLoaded() || NbEdges() <= 0)
        {
            return false;
        }

        // analysis
        myAnalyzer.CheckConnected(num, prec < 0 ? MaxTolerance() : prec);
        if (myAnalyzer.LastCheckStatus(ShapeExtend_FAIL))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }
        if (!myAnalyzer.LastCheckStatus(ShapeExtend_DONE))
        {
            return false;
        }

        // action: replacing vertex
        ShapeExtend_WireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.NbEdges());
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.NbEdges());
        Edge          E1   = sbwd.Edge(n1);
        Edge          E2   = sbwd.Edge(n2);

        var           sae = new ShapeAnalysis_Edge();
        Vertex V1  = sae.LastVertex(E1);
        Vertex V2  = sae.FirstVertex(E2);
        Vertex V;

        if (myAnalyzer.LastCheckStatus(ShapeExtend_DONE1))
        { // absolutely confused
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            // #40 rln 18.11.98 S4054 BUC60035 entity 2393 (2-nd sub-curve is edge with the same vertex)
            if (V2.IsSame(sae.LastVertex(E2)))
            {
                V = V2;
                if (Context() is not null)
                {
                    Context()!.Replace(V1, V.Oriented(V1.Orientation()));
                }
            }
            else
            {
                V = V1;
                if (Context() is not null)
                {
                    Context()!.Replace(V2, V.Oriented(V2.Orientation()));
                }
            }
        }
        else
        { // on moyenne ...
            if (myAnalyzer.LastCheckStatus(ShapeExtend_DONE2))
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
            }
            else
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
            }
            var sbv = new ShapeBuild_Vertex();
            V = sbv.InternalCombineVertex(V1, V2, 1.0001);
            if (Context() is not null)
            {
                Context()!.Replace(V1, V.Oriented(V1.Orientation()));
                Context()!.Replace(V2, V.Oriented(V2.Orientation()));
            }
        }

        // replace vertices to a new one
        var sbe = new ShapeBuild_Edge();
        if (sbwd.NbEdges() < 2)
        {
            if (E2.Free() && myTopoMode)
            {
                var B = new BrepBuilder();
                B.Remove(E2, sae.FirstVertex(E2));
                B.Remove(E2, sae.LastVertex(E2));
                B.Add(E2, V.Oriented(Forward));
                B.Add(E2, V.Oriented(Reversed));
            }
            else
            {
                Edge tmpE = sbe.InternalCopyReplaceVertices(E2, V, V);
                sbwd.Set(tmpE, n2);
                if (Context() is not null)
                {
                    Context()!.Replace(E2, tmpE);
                }
            }
        }
        else
        {
            if (E2.Free() && E1.Free() && myTopoMode)
            {
                var B = new BrepBuilder();
                B.Remove(E2, sae.FirstVertex(E2));
                B.Add(E2, V.Oriented(Forward));
                if (!myAnalyzer.LastCheckStatus(ShapeExtend_DONE1) || sae.FirstVertex(E2).IsSame(sae.LastVertex(E2)))
                {
                    B.Remove(E1, sae.LastVertex(E1));
                    B.Add(E1, V.Oriented(Reversed));
                }
            }
            else
            {
                Edge tmpE2 = sbe.InternalCopyReplaceVertices(E2, V, new Vertex());
                sbwd.Set(tmpE2, n2);
                if (Context() is not null)
                {
                    Context()!.Replace(E2, tmpE2);
                }
                if (!myAnalyzer.LastCheckStatus(ShapeExtend_DONE1) || sae.FirstVertex(E2).IsSame(sae.LastVertex(E2)))
                {
                    Edge tmpE1 = sbe.InternalCopyReplaceVertices(E1, new Vertex(), V);
                    sbwd.Set(tmpE1, n1);
                    if (Context() is not null)
                    {
                        Context()!.Replace(E1, tmpE1);
                    }
                }
            }
        }

        // Optionally update wire data with context replacements
        if (theUpdateWire && Context() is not null)
        {
            UpdateWire();
        }

        return true;
    }

    /// <summary>Fixes the seam edge <paramref name="num"/>: makes its two pcurves the analyser's seam pair.</summary>
    public bool FixSeam(int num)
    {
        enterWork(); // OCCT.NET: frozen
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        Curve2D? C1 = null, C2 = null;
        double        cf = 0.0, cl = 0.0;
        if (!myAnalyzer.CheckSeam(num, ref C1, ref C2, ref cf, ref cl))
        {
            return false;
        }

        var         B = new BrepBuilder();
        Edge E = WireData()!.Edge(num > 0 ? num : NbEdges());
        B.UpdateEdge(E, C2, C1, Face(), 0.0); //: S4136: BRep_Tool::Tolerance(E)
        B.Range(E, Face(), cf, cl);
        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);

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
    private static void UpdateEdgeUVPoints(in Edge E, in Face F)
    {
        Tool.Range(E, F, out double first, out double last);
        new BrepBuilder().Range(E, F, first, last);
    }

    /// <summary>
    /// Fixes pcurves shifted by a period of a closed surface (recomputed from 3d curves), on the seam or near it; to be
    /// called before FixDegenerated. The wire is supposed closed and ordered.
    /// </summary>
    public bool FixShifted()
    {
        enterWork(); // OCCT.NET: frozen
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        ShapeAnalysis_Surface surf = myAnalyzer.Surface()!;
        // #78 rln 12.03.99 S4135: checking spatial closure with Precision
        bool uclosed = surf.IsUClosed(Precision());
        bool vclosed = surf.IsVClosed(Precision()) || surf.Surface() is SphericalSurface;
        // #67 rln 01.03.99 S4135: ims010.igs entity D11900 (2D contour is 2*PI higher than V range
        // [-pi/2,p/2])

        // PTV 26.06.2002 begin
        // CTS18546-2.igs entity 2222: base curve is periodic and 2dcurve is shifted
        bool   IsVCrvClosed = false;
        double VRange       = 1.0;
        if (surf.Surface() is RevolutionSurface aSurOfRev)
        {
            Curve3D aBaseCrv = aSurOfRev.BasisCurve();
            while ((aBaseCrv is OffsetCurve3D) || (aBaseCrv is TrimmedCurve3D))
            {
                if (aBaseCrv is OffsetCurve3D anOffset)
                {
                    aBaseCrv = anOffset.BasisCurve();
                }
                if (aBaseCrv is TrimmedCurve3D aTrimmed)
                {
                    aBaseCrv = aTrimmed.BasisCurve();
                }
            }
            if (aBaseCrv.IsPeriodic)
            {
                vclosed      = true;
                VRange       = aBaseCrv.Period();
                IsVCrvClosed = true;
            }
        }
        // PTV 26.06.2002 end
        if (!uclosed && !vclosed)
        {
            return false;
        }

        double URange;
        surf.Surface()!.Bounds(out double SUF, out double SUL, out double SVF, out double SVL);
        double SUMid, SVMid;
        SUMid = 0.5 * (SUF + SUL);
        SVMid = 0.5 * (SVF + SVL);
        if (uclosed)
        {
            URange = Math.Abs(SUL - SUF);
        }
        else
        {
            URange = RealLast();
        }
        if (!IsVCrvClosed)
        {
            if (vclosed)
            {
                VRange = Math.Abs(SVL - SVF);
            }
            else
            {
                VRange = RealLast();
            }
        }
        double UTol = 0.2 * URange, VTol = 0.2 * VRange;

        ShapeExtend_WireData sbwdOring = WireData()!;
        var                  sae       = new ShapeAnalysis_Edge();
        var                  sbwd      = new ShapeExtend_WireData();
        for (int i = 1; i <= sbwdOring.NbEdges(); i++)
        {
            Edge E1 = sbwdOring.Edge(i);
            if (Tool.Degenerated(E1) && !sae.HasPCurve(E1, Face()))
            {
                continue;
            }

            sbwd.Add(E1);
        }

        var    sbe = new ShapeBuild_Edge();
        int    nb  = sbwd.NbEdges();
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

            Edge E1 = sbwd.Edge(n1);
            Edge E2 = sbwd.Edge(n2);

            if (Tool.Degenerated(E1) || Tool.Degenerated(E2))
            {
                if (!degstop)
                {
                    stop    = n2;
                    degstop = true;
                }
                continue;
            }

            Vertex V = sae.FirstVertex(E2);
            if (V.IsNull)
            {
                continue;
            }

            Point3D p = Tool.Pnt(V);

            double        a1 = 0.0, b1 = 0.0, a2 = 0.0, b2 = 0.0;
            Curve2D? c2d1 = null, c2d2 = null;

            //: abv 29.08.01: torCuts.sat: distinguish degeneration by U and by V;
            // only corresponding move is prohibited
            //    bool isDeg = surf->IsDegenerated ( p, Max ( Precision(),
            //    BRep_Tool::Tolerance(V) ) );
            int      isDeg = 0;
            Point2D degP1 = new Point2D(), degP2 = new Point2D();
            double   degT1 = 0.0, degT2 = 0.0;
            if (surf.DegeneratedValues(p, Max(Precision(), Tool.Tolerance(V)), ref degP1, ref degP2, ref degT1,
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
                    if (c2d1 is null && !sae.PCurve(E1, Face(), out c2d1, out a1, out b1, true))
                    {
                        continue;
                    }
                    Point2D p1 = new Point2D(SUF, c2d1!.Value(b1).Y());
                    Point2D p2 = new Point2D(SUL, c2d1.Value(b1).Y());
                    if (surf.IsDegenerated(p1, p2, MaxTolerance(), 10)
                        && !surf.IsDegenerated(c2d1.Value(a1), c2d1.Value(b1), MaxTolerance(), 10))
                    { // abv 31.07.00: trj4_pm1-ec-214.stp #31274: still allow
                        // work if edge already exists
                        isDeg = 1;
                    }
                }
                if (isDeg == 0 && !uclosed)
                {
                    if (c2d1 is null && !sae.PCurve(E1, Face(), out c2d1, out a1, out b1, true))
                    {
                        continue;
                    }
                    Point2D p1 = new Point2D(c2d1!.Value(b1).X(), SVF);
                    Point2D p2 = new Point2D(c2d1.Value(b1).X(), SVL);
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
                        if ((c2d1 is null && !sae.PCurve(E1, Face(), out c2d1, out a1, out b1, true))
                            || (c2d2 is null && !sae.PCurve(E2, Face(), out c2d2, out a2, out b2, true))
                            || !sae.PCurve(sbwd.Edge(degn2 > 1 ? degn2 - 1 : nb), Face(), out cx1, out ax1, out bx1, true)
                            || !sae.PCurve(sbwd.Edge(degn2), Face(), out cx2, out ax2, out bx2, true))
                        {
                            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
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
                            period = URange;
                        }
                        else
                        {
                            x.SetY(1.0);
                            period = VRange;
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
                            double dx = ShapeAnalysis.AdjustToPeriod(deep, Kerf.Engine.Foundation.Precision.PConfusion(),
                                                                     period + Kerf.Engine.Foundation.Precision.PConfusion());
                            x *= (scld > 0 ? -dx : dx);
                            // x *= ( std::abs(scld-scln) > 1.5 * period ? 2. : 1. ) *
                            //      ( scld >0 ? -period : period );
                            var Shift = new Transform2D();
                            Shift.SetTranslation(x);
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
                                var cx1new = cx1!.Transformed(Shift) as Curve2D;
                                sbe.ReplacePCurve(edge, cx1new, Face());
                                UpdateEdgeUVPoints(edge, Face());
                            }
                            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                            continue;
                        }
                        // degn2 = n2; pdeg = p; // ie_exhaust-A.stp #37520
                    }
                }
                // OCCT's commented-out fix of half spheres ("pdn to fix half sphere") is left out.
                //: abv 29.08.01: torCuts.sat:      continue;
            }

            if ((c2d1 is null && !sae.PCurve(E1, Face(), out c2d1, out a1, out b1, true))
                || (c2d2 is null && !sae.PCurve(E2, Face(), out c2d2, out a2, out b2, true)))
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                continue;
            }
            Point2D p2d1 = c2d1!.Value(b1);
            Point2D p2d2 = c2d2!.Value(a2);
            box.Add(p2d1);

            double du = 0.0, dv = 0.0;
            if (uclosed && isDeg != 1)
            {
                double dx = Math.Abs(p2d2.X() - p2d1.X());
                if (dx > URange - UTol)
                {
                    du = ShapeAnalysis.AdjustByPeriod(p2d2.X(), p2d1.X(), URange);
                }
                else if (dx > UTol && stop == nb)
                {
                    stop = n2; //: abv 29.08.01: torCuts2.stp
                }
            }
            if (vclosed && isDeg != 2)
            {
                double dy = Math.Abs(p2d2.Y() - p2d1.Y());
                if (dy > VRange - VTol)
                {
                    dv = ShapeAnalysis.AdjustByPeriod(p2d2.Y(), p2d1.Y(), VRange);
                }
                else if (dy > VTol && stop == nb)
                {
                    stop = n2;
                }
            }
            if (du == 0.0 && dv == 0.0)
            {
                continue;
            }

            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            var Shift2 = new Transform2D();
            Shift2.SetTranslation(new Vector2D(du, dv));
            // c2d2->Transform ( Shift );
            //  skl 15.05.2002 for OCC208 (if few edges have reference to one pcurve)
            var c2d2new = c2d2.Transformed(Shift2) as Curve2D;
            sbe.ReplacePCurve(E2, c2d2new, Face());
            UpdateEdgeUVPoints(E2, Face());
        }
        if (box.IsVoid)
        {
            return false; // #3 smh 01.04.99. S4163: Overflow, when box is void.
        }

        box.Get(out double umin, out double vmin, out double umax, out double vmax);
        if (Math.Abs(umin + umax - SUF - SUL) < URange && Math.Abs(vmin + vmax - SVF - SVL) < VRange
            && !LastFixStatus(ShapeExtend_DONE))
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
            du2 = ShapeAnalysis.AdjustByPeriod(umid, SUMid, URange);
        }
        if (vclosed)
        {
            double vmid = 0.5 * (vmin + vmax);
            //     dv = ShapeAnalysis::AdjustToPeriod(vmid, SVF, SVL);
            // PTV 26.06.2002 xloop torus-apple iges face mode
            dv2 = ShapeAnalysis.AdjustByPeriod(vmid, SVMid, VRange);
        }

        if (du2 == 0.0 && dv2 == 0.0)
        {
            return true;
        }

        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);

        var Shift3 = new Transform2D();
        Shift3.SetTranslation(new Vector2D(du2, dv2));

        for (n = 1; n <= sbwdOring.NbEdges(); n++)
        {
            Edge ed = sbwdOring.Edge(n);
            if (!sae.PCurve(ed, Face(), out Curve2D? c2d, out double a, out double b, true))
            {
                continue;
            }
            // skl 15.05.2002 for OCC208 (if few edges have reference to one pcurve)
            var c2d2 = c2d!.Transformed(Shift3) as Curve2D;
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
        enterWork(); // OCCT.NET: frozen
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        // analysis
        Point2D p2d1 = new Point2D(), p2d2 = new Point2D();
        myAnalyzer.CheckDegenerated(num, ref p2d1, ref p2d2);
        if (myAnalyzer.LastCheckStatus(ShapeExtend_FAIL1))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }
        //: abv 29.08.01: torHalf2.sat: if edge was encoded as degenerated but
        //  has no pcurve and no singularity is found at that point, remove it
        if (myAnalyzer.LastCheckStatus(ShapeExtend_FAIL2))
        {
            WireData()!.Remove(num);
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
            return true;
        }
        if (!myAnalyzer.LastCheckStatus(ShapeExtend_DONE))
        {
            return false;
        }

        // action: create degenerated edge and insert it (or replace)

        Vector2D    vect2d = new Vector2D(p2d1, p2d2);
        Direction2D    dir2d  = new Direction2D(vect2d);
        LineCurve2D line2d = new LineCurve2D(p2d1, dir2d);

        Edge  degEdge = new Edge();
        BrepBuilder B       = new BrepBuilder();
        B.MakeEdge(ref degEdge);
        B.Degenerated(degEdge, true);
        B.UpdateEdge(degEdge, line2d, Face(), Kerf.Engine.Foundation.Precision.Confusion());
        B.Range(degEdge, Face(), 0.0, vect2d.Magnitude());

        ShapeExtend_WireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.NbEdges());
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.NbEdges());

        bool lack = myAnalyzer.LastCheckStatus(ShapeExtend_DONE1);
        int  n3   = (lack ? n2 : (n2 < sbwd.NbEdges() ? n2 + 1 : 1));

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        Vertex      V1  = sae.LastVertex(sbwd.Edge(n1));
        Vertex      V2  = sae.FirstVertex(sbwd.Edge(n3));

        V1.Orientation(Forward);
        V2.Orientation(Reversed);
        B.Add(degEdge, V1);
        B.Add(degEdge, V2);
        degEdge.Orientation(Forward);

        if (lack)
        {
            sbwd.Add(degEdge, n2);
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }
        else
        {
            sbwd.Set(degEdge, n2);
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
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
    private static bool TryNewPCurve(in Edge E, in Face face, ref Curve2D? c2d, ref double first,
                                     ref double last, ref double tol)
    {
        Curve3D? crv = Tool.Curve(E, out double f, out double l);
        if (crv is null)
        {
            return false;
        }

        // make temp edge and compute tolerance
        BRepBuilderAPI_MakeEdge mkedge = new BRepBuilderAPI_MakeEdge(crv, f, l);

        ShapeBuild_Edge SBE = new ShapeBuild_Edge();     // skl 17.07.2001
        SBE.SetRange3d(mkedge.InternalEdge(), f, l); // skl 17.07.2001

        if (!mkedge.IsDone())
        {
            return false;
        }

        Edge  edge = mkedge.InternalEdge();
        BrepBuilder B    = new BrepBuilder();
        B.UpdateEdge(edge, c2d, face, 0.0);
        B.Range(edge, face, first, last);
        B.SameRange(edge, false);
        // no call to BRepLib:  B.SameParameter ( edge, false );

        ShapeFix_Edge sfe = new ShapeFix_Edge();
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

    private static int howMuchPCurves(in Edge E)
    {
        int count = 0;
        // OCCT.NET: frozen: the list is only read (OCCT walks ChangeCurves())
        var itcr = new NCollection_List<CurveRepresentation>.Iterator(((BrepTEdge)E.TShape()!).Curves());

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

    private static bool RemoveLoop(in Edge E, in Face face, in IntRes2d_IntersectionPoint IP, double tolfact,
                                   double prec, bool RemoveLoop3d)
    {
        bool loopRemoved3d;
        if (Tool.IsClosed(E, face))
        {
            return false;
        }

        Curve3D? crv = Tool.Curve(E, out double f, out double l);

        double t1 = IP.ParamOnFirst();
        double t2 = IP.ParamOnSecond();
        if (t1 > t2)
        {
            double t = t1;
            t1       = t2;
            t2       = t;
        }

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        if (!sae.PCurve(E, face, out Curve2D? c2d, out double a, out double b, false))
        {
            return false;
        }

        CurveAdaptor GAC = new CurveAdaptor(crv!, f, l);
        double            dt  = tolfact * GAC.Resolution(prec);
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
        Plane3D Pln = new Plane3D(ax);

        // PTV OCC884
        PlaneSurface? aPlaneSurf = Tool.Surface(face) as PlaneSurface;
        Curve3D? pcurve3d   = crv;
        if (aPlaneSurf is not null)
        {
            Pln = aPlaneSurf.Pln();
        }
        else
        {
            pcurve3d = GeomAPI.To3d(c2d!, Pln);
        }

        // first segment
        // occ::handle<Geom_Curve> pcurve3d = GeomAPI::To3d ( c2d, Pln );
        TrimmedCurve3D                   trim    = new TrimmedCurve3D(pcurve3d!, a, t1);
        CompCurveToBSplineCurve connect = new CompCurveToBSplineCurve(trim);

        // null-length segment patch instead of loop
        NCollection_Array1<Point3D> Poles = new NCollection_Array1<Point3D>(1, 2);
        NCollection_Array1<double> Knots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int>    Mults = new NCollection_Array1<int>(1, 2);

        Poles.SetValue(1, trim.Value(t1));
        Knots.SetValue(1, t1);
        Mults.SetValue(1, 2);

        trim = new TrimmedCurve3D(pcurve3d!, t2, b);
        Poles.SetValue(2, trim.Value(t2));
        Knots.SetValue(2, t2);
        Mults.SetValue(2, 2);

        BSplineCurve3D patch = new BSplineCurve3D(Poles, Knots, Mults, 1);
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
        Curve3D? aNew3dCrv = connect.InternalBSplineCurve();

        Curve2D? bs = GeomAPI.To2d(aNew3dCrv!, Pln);
        if (bs is null)
        {
            return false;
        }

        // make temp edge and compute tolerance
        BrepBuilder B = new BrepBuilder();

        if (!RemoveLoop3d)
        { // old variant (not remove loop 3d)
            double newtol = 0;
            // OCC901
            int nbC2d = howMuchPCurves(E);
            if (nbC2d <= 1 && aPlaneSurf is not null)
            {
                B.UpdateEdge(E, bs, face, 0);
            }
            else if (!TryNewPCurve(E, face, ref bs, ref a, ref b, ref newtol))
            {
                return false;
            }

            double tol = Tool.Tolerance(E);
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
            if (aPlaneSurf is not null)
            {
                B.UpdateEdge(E, aNew3dCrv, Max(newtol, tol));
                // OCC901
                if (!TryNewPCurve(E, face, ref bs, ref a, ref b, ref newtol))
                {
                    return false;
                }
            }
            B.UpdateEdge(E, bs, face, newtol);
            B.UpdateVertex(sae.FirstVertex(E), newtol);
            B.UpdateVertex(sae.LastVertex(E), newtol);
            return true;
        }

        //: q1
        Surface?       S  = Tool.Surface(face, out Placement L);
        CurveAdaptor2D AC = new CurveAdaptor2D(c2d!);
        SurfaceAdaptor AS = new SurfaceAdaptor(S!);

        CurveOnSurface ACS = new CurveOnSurface(AC, AS);
        Point3D                   P1  = ACS.Value(t1);
        Point3D                   P2  = ACS.Value(t2);
        Point3D pcurPnt = new Point3D((P1.X() + P2.X()) / 2, (P1.Y() + P2.Y()) / 2, (P1.Z() + P2.Z()) / 2);

        ShapeAnalysis_TransferParametersProj SFTP  = new ShapeAnalysis_TransferParametersProj(E, face);
        NCollection_HSequence<double>        Seq2d = new NCollection_HSequence<double>();
        Seq2d.Append(t1);
        Seq2d.Append(t2);
        Seq2d.Append((t1 + t2) / 2);
        NCollection_HSequence<double> Seq3d = SFTP.Perform(Seq2d, false);

        double dist1 = pcurPnt.Distance(crv!.Value(Seq3d.Value(1))); // correcting Seq3d already project
        double dist2 = pcurPnt.Distance(crv.Value(Seq3d.Value(2)));
        double dist3 = pcurPnt.Distance(crv.Value(Seq3d.Value(3)));
        double ftrim, ltrim;
        loopRemoved3d = dist3 <= Max(dist1, dist2);

        Curve3D? bs1 = null;

        if (!loopRemoved3d)
        {
            // create new 3d curve
            ftrim = Seq3d.Value(1);
            ltrim = Seq3d.Value(2);
            ftrim -= dt;
            ltrim += dt;
            TrimmedCurve3D                   trim1    = new TrimmedCurve3D(crv, f, ftrim);
            CompCurveToBSplineCurve connect1 = new CompCurveToBSplineCurve(trim1);
            NCollection_Array1<Point3D>          Poles1   = new NCollection_Array1<Point3D>(1, 2);
            NCollection_Array1<double>          Knots1   = new NCollection_Array1<double>(1, 2);
            NCollection_Array1<int>             Mults1   = new NCollection_Array1<int>(1, 2);

            Poles1.SetValue(1, trim1.Value(ftrim));
            Knots1.SetValue(1, ftrim);
            Mults1.SetValue(1, 2);

            trim1 = new TrimmedCurve3D(crv, ltrim, l);
            Poles1.SetValue(2, trim1.Value(ltrim));
            Knots1.SetValue(2, ltrim);
            Mults1.SetValue(2, 2);

            // create b-spline curve
            BSplineCurve3D patch1 = new BSplineCurve3D(Poles1, Knots1, Mults1, 1);

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
            B.UpdateEdge(E, bs1, L, 0.0);
        }
        B.UpdateEdge(E, bs, face, 0.0);
        B.Range(E, face, f, l);
        B.SameRange(E, false);

        ShapeFix_Edge sfe = new ShapeFix_Edge();
        sfe.FixSameParameter(E);

        return true;
    }

    private static bool RemoveLoop(in Edge E, in Face face, in IntRes2d_IntersectionPoint IP2d,
                                   ref Edge E1, ref Edge E2)
    {
        if (Tool.IsClosed(E, face))
        {
            return false;
        }

        Curve3D? crv = Tool.Curve(E, out double f, out double l);

        double t1 = IP2d.ParamOnFirst();
        double t2 = IP2d.ParamOnSecond();

        if (t1 > t2)
        {
            double t = t1;
            t1       = t2;
            t2       = t;
        }

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();

        // define vertexes Vfirst , Vlast, Vmid
        Vertex Vfirst, Vlast, Vmid = new Vertex();
        // initialize Vfirst and Vlast
        Vfirst = sae.FirstVertex(E);
        Vlast  = sae.LastVertex(E);

        // find a 2d curve and parameters from edge E
        if (!sae.PCurve(E, face, out Curve2D? c2d, out double a, out double b, false))
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

        Surface?       S  = Tool.Surface(face, out Placement L);
        CurveAdaptor2D AC = new CurveAdaptor2D(c2d!);
        SurfaceAdaptor AS = new SurfaceAdaptor(S!);

        CurveOnSurface ACS = new CurveOnSurface(AC, AS);
        Point3D                   P1  = ACS.Value(t1);
        Point3D                   P2  = ACS.Value(t2);
        Point3D pcurPnt = new Point3D((P1.X() + P2.X()) / 2, (P1.Y() + P2.Y()) / 2, (P1.Z() + P2.Z()) / 2);

        // transfer parameters from pcurve to 3d curve
        ShapeAnalysis_TransferParametersProj SFTP  = new ShapeAnalysis_TransferParametersProj(E, face);
        NCollection_HSequence<double>        Seq2d = new NCollection_HSequence<double>();
        Seq2d.Append(t1);
        Seq2d.Append(t2);
        Seq2d.Append((t1 + t2) / 2);
        NCollection_HSequence<double> Seq3d = SFTP.Perform(Seq2d, false);

        double dist1 = pcurPnt.Distance(crv!.Value(Seq3d.Value(1))); // correcting Seq3d already project
        double dist2 = pcurPnt.Distance(crv.Value(Seq3d.Value(2)));
        double dist3 = pcurPnt.Distance(crv.Value(Seq3d.Value(3)));
        double ftrim, ltrim;
        if (dist3 > Max(dist1, dist2))
        { // is loop in 3d
            ftrim = Seq3d.Value(1);
            ltrim = Seq3d.Value(2);
        }
        else
        { // not loop in 3d
            ftrim = Seq3d.Value(3);
            ltrim = Seq3d.Value(3);
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
        Point3D Pmid = new Point3D((pnt1.X() + pnt2.X()) / 2, (pnt1.Y() + pnt2.Y()) / 2, (pnt1.Z() + pnt2.Z()) / 2);

        BrepBuilder B = new BrepBuilder();

        // create new copies for E1 and E2
        if (trim1 is not null)
        {
            E1 = ShapeCasts.Edge(E.EmptyCopied());
        }
        E2 = ShapeCasts.Edge(E.EmptyCopied());

        // initialize middle vertex Vmid
        if (trim1 is null)
        {
            B.MakeVertex(ref Vmid, pnt2, 0.0);
        }
        else
        {
            B.MakeVertex(ref Vmid, Pmid, 0.0);
        }

        ShapeBuild_Edge sbe = new ShapeBuild_Edge();

        // replace verteces for new edges E1 and E2
        if (E.Orientation() == Forward)
        {
            if (!E1.IsNull)
            {
                E1 = sbe.InternalCopyReplaceVertices(E1, Vfirst, Vmid);
            }
            E2 = sbe.InternalCopyReplaceVertices(E2, Vmid, Vlast);
        }
        else
        {
            if (!E1.IsNull)
            {
                E1 = sbe.InternalCopyReplaceVertices(E1, Vmid, Vlast);
            }
            E2 = sbe.InternalCopyReplaceVertices(E2, Vfirst, Vmid);
        }

        // Update edges by 2d and 3d curves
        ShapeFix_Edge mySfe = new ShapeFix_Edge();
        if (!E1.IsNull)
        {
            B.UpdateEdge(E1, trim1, face, 0.0);
            B.UpdateEdge(E1, trim3, 0.0);
            B.Range(E1, f, ftrim);
            B.SameRange(E1, false);
            //    B.SameParameter(E1,false);
            mySfe.FixSameParameter(E1);
            mySfe.FixVertexTolerance(E1);
        }
        B.UpdateEdge(E2, trim2, face, 0.0);
        B.UpdateEdge(E2, trim4, 0.0);
        B.Range(E2, ltrim, l);
        B.SameRange(E2, false);
        //  B.SameParameter(E2, false);
        mySfe.FixSameParameter(E2);
        mySfe.FixVertexTolerance(E2);

        return true;
    }

    private bool FixSelfIntersectingEdge(int num)
    {
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        // analysis
        NCollection_Sequence<IntRes2d_IntersectionPoint> points2d       = new NCollection_Sequence<IntRes2d_IntersectionPoint>();
        NCollection_Sequence<Point3D>                     points3d       = new NCollection_Sequence<Point3D>();
        ShapeAnalysis_Wire?                              theAdvAnalyzer = myAnalyzer;
        if (theAdvAnalyzer is null)
        {
            return false;
        }
        theAdvAnalyzer.CheckSelfIntersectingEdge(num, points2d, points3d);
        if (theAdvAnalyzer.LastCheckStatus(ShapeExtend_FAIL))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }
        if (!theAdvAnalyzer.LastCheckStatus(ShapeExtend_DONE))
        {
            return false;
        }

        // action: increase tolerance of vertex

        Edge E = WireData()!.Edge(num > 0 ? num : NbEdges());

        ShapeAnalysis_Edge sae  = new ShapeAnalysis_Edge();
        Vertex      V1   = sae.FirstVertex(E);
        Vertex      V2   = sae.LastVertex(E);
        double             tol1 = Tool.Tolerance(V1);
        double             tol2 = Tool.Tolerance(V2);
        Point3D             pnt1 = Tool.Pnt(V1);
        Point3D             pnt2 = Tool.Pnt(V2);

        // cycle is to verify fix in case of RemoveLoop
        double        tolfact = 0.1; // factor for shifting by parameter in RemoveLoop
        double        f2d = 0.0, l2d = 0.0;
        Curve2D? c2d    = null;
        double        newtol = 0.0; // = Precision();

        if (myRemoveLoopMode < 1)
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
                    if (myGeomMode)
                    {
                        if (c2d is null)
                        {
                            sae.PCurve(E, Face(), out c2d, out f2d, out l2d, false);
                        }
                        double firstpar = points2d.Value(i).ParamOnFirst();
                        double lastpar  = points2d.Value(i).ParamOnSecond();
                        if (firstpar > prevFirst && lastpar < prevLast)
                        {
                            continue;
                        }
                        if (RemoveLoop(E,
                                       Face(),
                                       points2d.Value(i),
                                       tolfact,
                                       Min(MaxTolerance(), Max(newtol, Precision())),
                                       myRemoveLoopMode == 0))
                        {
                            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                            loopRemoved = true;
                            prevFirst   = firstpar;
                            prevLast    = lastpar;
                            continue; // repeat of fix on that edge required (to be done by caller)
                        }
                    }
                    if (newtol < MaxTolerance())
                    {
                        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                        BrepBuilder B = new BrepBuilder();
                        if (dist21 < dist22)
                        {
                            tol1 = newtol;
                            B.UpdateVertex(V1, tol1);
                        }
                        else
                        {
                            tol2 = newtol;
                            B.UpdateVertex(V2, tol2);
                        }
                    }
                    else
                    {
                        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                    }
                }

                // after RemoveLoop, check that self-intersection disappeared
                if (loopRemoved)
                {
                    NCollection_Sequence<IntRes2d_IntersectionPoint> pnts2d = new NCollection_Sequence<IntRes2d_IntersectionPoint>();
                    NCollection_Sequence<Point3D>                     pnts3d = new NCollection_Sequence<Point3D>();
                    theAdvAnalyzer.CheckSelfIntersectingEdge(num, pnts2d, pnts3d);
                    if (!theAdvAnalyzer.LastCheckStatus(ShapeExtend_DONE))
                    {
                        break;
                    }
                    // points3d.Append(pnts3d);
                    // points2d.Append(pnts2d);
                    points3d = pnts3d;
                    points2d = pnts2d;
                    BrepBuilder B = new BrepBuilder();
                    B.UpdateEdge(E, c2d, Face(), 0.0);
                    B.Range(E, Face(), f2d, l2d);
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
        if (myRemoveLoopMode == 1)
        {
            // after fixing will be nb+1 edges
            bool loopRemoved;
            // create a sequence of resulting edges
            NCollection_HSequence<Shape> TTSS = new NCollection_HSequence<Shape>();
            Edge                         E2   = new Edge();

            loopRemoved = false;
            //: k3 abv 24 Dec 98: BUC50070 #26682 and #30087: try to remove loop
            if (myGeomMode)
            {
                if (c2d is null)
                {
                    sae.PCurve(E, Face(), out c2d, out f2d, out l2d, false);
                }
                Edge E1 = new Edge();
                if (RemoveLoop(E, Face(), points2d.Value(1), ref E1, ref E2))
                {
                    myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
                    loopRemoved = true;
                    if (!E1.IsNull)
                    {
                        TTSS.Append(E1);
                        newtol = Max(Tool.Tolerance(E1), Tool.Tolerance(E2));
                    }
                    else
                    {
                        newtol = Tool.Tolerance(E2);
                    }
                }
            }

            TTSS.Append(E2);

            if (newtol > MaxTolerance())
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }

            ShapeExtend_WireData sewd = new ShapeExtend_WireData();
            for (int i = 1; i <= TTSS.Length(); i++)
            {
                sewd.Add(ShapeCasts.Edge(TTSS.Value(i)));
            }
            if (Context() is not null)
            {
                Context()!.Replace(E, sewd.InternalWire());
                UpdateWire();
            }
            else
            {
                WireData()!.Remove(num > 0 ? num : NbEdges());
                WireData()!.Add(sewd.InternalWire(), num > 0 ? num : NbEdges());
            }
            if (loopRemoved)
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE8);
            }
        }

        if (LastFixStatus(ShapeExtend_DONE) && !myShape.IsNull)
        {
            SendWarning(E, new Msg("FixAdvWire.FixIntersection.MSG5")); // Edge was self-intersecting, corrected
        }

        return LastFixStatus(ShapeExtend_DONE);
    }

    private static double ComputeLocalDeviation(in Edge edge, in Point3D pint, in Point3D pnt, double f, double l,
                                                in Face face)
    {
        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        if (!sae.Curve3d(edge, out Curve3D? c3d, out double a, out double b, false))
        {
            return RealLast();
        }

        Line3D line = new Line3D(pint, new Direction3D(new Vector3D(pint, pnt)));

        if (sae.PCurve(edge, face, out Curve2D? Crv, out double fp, out double lp, false))
        {
            if (Crv is TrimmedCurve2D tc)
            {
                if (tc.BasisCurve() is LineCurve2D)
                {
                    f = a + (f - fp) * (b - a) / (lp - fp);
                    l = a + (l - fp) * (b - a) / (lp - fp);
                }
            }
        }

        const int NSEG = 10;
        double    step = (l - f) / NSEG;
        double    dev  = 0.0;
        for (int i = 1; i < NSEG; i++)
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
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady() || NbEdges() < 2)
        {
            return false;
        }

        // analysis
        NCollection_Sequence<IntRes2d_IntersectionPoint> points2d       = new NCollection_Sequence<IntRes2d_IntersectionPoint>();
        NCollection_Sequence<Point3D>                     points3d       = new NCollection_Sequence<Point3D>();
        NCollection_Sequence<double>                     errors         = new NCollection_Sequence<double>();
        ShapeAnalysis_Wire?                              theAdvAnalyzer = myAnalyzer;
        if (theAdvAnalyzer is null)
        {
            return false;
        }
        theAdvAnalyzer.CheckIntersectingEdges(num, points2d, points3d, errors);
        if (theAdvAnalyzer.LastCheckStatus(ShapeExtend_FAIL))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }
        if (!theAdvAnalyzer.LastCheckStatus(ShapeExtend_DONE))
        {
            return false;
        }

        // rln 03/02/98: CSR#BUC50004 entity 56 (to avoid later inserting lacking edge)
        //: l0  bool isLacking = myAnalyzer->CheckLacking ( num );

        // action: increase tolerance of vertex

        ShapeExtend_WireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.NbEdges());
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.NbEdges());
        Edge          E1   = sbwd.Edge(n1);
        Edge          E2   = sbwd.Edge(n2);
        if (Context() is not null)
        {
            E1 = ShapeCasts.Edge(Context()!.InternalApply(sbwd.Edge(n1)));
            E2 = ShapeCasts.Edge(Context()!.InternalApply(sbwd.Edge(n2)));
        }

        bool isForward1 = (E1.Orientation() == Forward);
        bool isForward2 = (E2.Orientation() == Forward);
        Tool.Range(E1, Face(), out double a1, out double b1);
        Tool.Range(E2, Face(), out double a2, out double b2);

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        Vertex      Vp  = sae.FirstVertex(E1);
        Vertex      V1  = sae.LastVertex(E1);
        Vertex      V2  = sae.FirstVertex(E2);
        Vertex      Vn  = sae.LastVertex(E2);

        double tol = Tool.Tolerance(V1);
        Point3D pnt = Tool.Pnt(V1);

        double prevRange1 = RealLast(), prevRange2 = RealLast();
        bool   cutEdge1 = false, cutEdge2 = false;
        bool   IsCutLine     = false;
        bool   isChangedEdge = false;

        BrepBuilder B = new BrepBuilder();

        int nb = points3d.Length();
        for (int i = 1; i <= nb; i++)
        {
            IntRes2d_IntersectionPoint IP     = points2d.Value(i);
            double                     param1 = (num == 1 ? IP.ParamOnSecond() : IP.ParamOnFirst());
            double                     param2 = (num == 1 ? IP.ParamOnFirst() : IP.ParamOnSecond());

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

            bool locMayEdit = myTopoMode;
            // Always try to modify the tolerance firstly as a better solution
            if (/*! myTopoMode &&*/ newtol > tol)
            {
                double te1 =
                    rad + ComputeLocalDeviation(E1, pint, pnt, param1, (isForward1 ? b1 : a1), Face());
                double te2 =
                    rad + ComputeLocalDeviation(E2, pint, pnt, (isForward2 ? a2 : b2), param2, Face());
                double maxte = Max(te1, te2);
                if (maxte < MaxTolerance() && maxte < newtol)
                {
                    if (Tool.Tolerance(E1) < te1 || Tool.Tolerance(E2) < te2)
                    {
                        // Make copy of edges.
                        if (Context() is not null)
                        {
                            isChangedEdge = true; // To avoid double copying of vertexes.

                            // Intersection point of two base edges.
                            ShapeBuild_Edge aSBE = new ShapeBuild_Edge();
                            Vertex   VV1  = Context()!.CopyVertex(V1);

                            Vertex VVp = Vp;
                            Vertex VVn = Vn;
                            if (Vp.IsSame(Vn))
                            {
                                // Should modify only one vertex.
                                VVp = Context()!.CopyVertex(Vp);
                                VVn = VVp;
                            }
                            else
                            {
                                VVp = Context()!.CopyVertex(Vp);
                                VVn = Context()!.CopyVertex(Vn);
                            }

                            Edge EE1 = aSBE.InternalCopyReplaceVertices(E1, VVp, VV1);
                            Edge EE2 = aSBE.InternalCopyReplaceVertices(E2, VV1, VVn);

                            Context()!.Replace(E1, EE1);
                            Context()!.Replace(E2, EE2);

                            UpdateWire();
                            E1 = sbwd.Edge(n1);
                            E2 = sbwd.Edge(n2);
                            Vp = sae.FirstVertex(E1);
                            V1 = sae.LastVertex(E1);
                            V2 = sae.FirstVertex(E2);
                            Vn = sae.LastVertex(E2);
                        }

                        B.UpdateEdge(E1, 1.000001 * te1);
                        B.UpdateVertex(sae.FirstVertex(E1), 1.000001 * te1);
                        B.UpdateVertex(sae.LastVertex(E1), 1.000001 * te1);
                        B.UpdateEdge(E2, 1.000001 * te2);
                        B.UpdateVertex(sae.FirstVertex(E2), 1.000001 * te2);
                        B.UpdateVertex(sae.LastVertex(E2), 1.000001 * te2);

                        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE6);
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
                    ShapeFix_SplitTool aTool = new ShapeFix_SplitTool();

                    if (!aTool.CutEdge(E1, (isForward1 ? a1 : b1), param1, Face(), ref IsCutLine))
                    {
                        if (V1.IsSame(Vp))
                        {
                            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
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

                    if (!aTool.CutEdge(E2, (isForward2 ? b2 : a2), param2, Face(), ref IsCutLine))
                    {
                        if (V2.IsSame(Vn))
                        {
                            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
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
                    Tool.SameParameter(E1) && Tool.SameParameter(E2))
                {
                    myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                    pnt = pint;
                    if (tol <= rad)
                    {
                        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                        tol = 1.001 * rad;
                    }
                }
                else
                {
                    if (IsCutLine)
                    {
                        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                        pnt = pint;
                        if (tol <= rad)
                        {
                            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                            tol = 1.001 * rad;
                        }
                    }
                    else
                    { // else increase tolerance
                        if (tol < newtol)
                        { // rln 07.04.99 CCI60005-brep.igs
                            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                            tol = newtol;
                        }
                    }
                }
            }
            else
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }
        }

        if (!LastFixStatus(ShapeExtend_DONE))
        {
            return false;
        }

        if (isChangedEdge)
        {
            B.UpdateVertex(V1, pnt, tol);
            B.UpdateVertex(V2, pnt, tol);
        }
        else
        {
            if (Context() is not null)
            {
                if (V1.IsSame(V2))
                {
                    Context()!.CopyVertex(V1, pnt, tol);
                }
                else
                {
                    Context()!.CopyVertex(V1, pnt, tol);
                    Context()!.CopyVertex(V2, pnt, tol);
                }
            }
            else
            {
                B.UpdateVertex(V1, pnt, tol);
                B.UpdateVertex(V2, pnt, tol);
            }
        }

        //: h4: make edges SP (after all cuts: t4mug.stp #3730+#6460)
        if (cutEdge1)
        {
            if (Context() is not null)
            {
                E1 = ShapeCasts.Edge(Context()!.InternalApply(E1));
            }
            myFixEdge.FixSameParameter(E1);
        }
        if (cutEdge2 && !IsCutLine)
        {
            if (Context() is not null)
            {
                E2 = ShapeCasts.Edge(Context()!.InternalApply(E2));
            }
            myFixEdge.FixSameParameter(E2);
        }
        if (cutEdge1 || cutEdge2)
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE7);
        }
        if (!myShape.IsNull)
        {
            SendWarning(new Msg("FixAdvWire.FixIntersection.MSG10")); // Edges were intersecting, corrected
        }
        return true;
    }

    // pdn 17.03.99 fixing non adjacent intersection by increasing tolerance of vertex

    private bool FixIntersectingEdges(int num1, int num2)
    {
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }
        NCollection_Sequence<IntRes2d_IntersectionPoint> points2d       = new NCollection_Sequence<IntRes2d_IntersectionPoint>();
        NCollection_Sequence<Point3D>                     points3d       = new NCollection_Sequence<Point3D>();
        NCollection_Sequence<double>                     errors         = new NCollection_Sequence<double>();
        ShapeAnalysis_Wire?                              theAdvAnalyzer = myAnalyzer;
        if (theAdvAnalyzer is null)
        {
            return false;
        }
        theAdvAnalyzer.CheckIntersectingEdges(num1, num2, points2d, points3d, errors);
        if (theAdvAnalyzer.LastCheckStatus(ShapeExtend_FAIL))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }
        if (!theAdvAnalyzer.LastCheckStatus(ShapeExtend_DONE))
        {
            return false;
        }
        NCollection_Array1<Point3D>       vertexPoints = new NCollection_Array1<Point3D>(1, 4);
        NCollection_Array1<double>       vertexTolers = new NCollection_Array1<double>(1, 4);
        NCollection_Array1<double>       newTolers    = new NCollection_Array1<double>(1, 4);
        NCollection_Array1<Shape> vertices     = new NCollection_Array1<Shape>(1, 4);
        newTolers.Init(0);

        ShapeExtend_WireData sbwd = WireData()!;
        int                  n2   = (num1 > 0 ? num1 : sbwd.NbEdges());
        int                  n1   = (num2 > 1 ? num2 : sbwd.NbEdges());
        if (n1 == n2)
        {
            return false;
        }

        Edge edge1 = sbwd.Edge(n1);
        Edge edge2 = sbwd.Edge(n2);

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
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

        double aNewTolEdge1 = 0.0, aNewTolEdge2 = 0.0;
        int    nb = points3d.Length();
        for (i = 1; i <= nb; i++)
        {
            Point3D pint = points3d.Value(i);

            // searching for the nearest vertexies to the intersection point
            double aVtx1Param = 0.0, aVtx2Param = 0.0;
            int    aVC1, aVC2;
            double aMinDist          = RealLast();
            Point3D aNearestVertex    = new Point3D();
            double aNecessaryVtxTole = 0.0;
            for (aVC1 = 1; aVC1 <= 2; aVC1++)
            {
                for (aVC2 = 3; aVC2 <= 4; aVC2++)
                {

                    double aVtxIPDist  = pint.Distance(vertexPoints[aVC1]);
                    double aVtxVtxDist = vertexPoints[aVC1].Distance(vertexPoints[aVC2]);
                    if (aMinDist > aVtxIPDist && aVtxIPDist > aVtxVtxDist)
                    {
                        aNecessaryVtxTole = aVtxVtxDist;
                        aNearestVertex    = vertexPoints[aVC1];
                        aMinDist          = aVtxIPDist;
                        aVtx1Param        = Tool.Parameter(ShapeCasts.Vertex(vertices[aVC1]), edge1);
                        aVtx2Param        = Tool.Parameter(ShapeCasts.Vertex(vertices[aVC2]), edge2);
                    }
                }
            }

            // calculation of necessary tolerances of edges
            IntRes2d_IntersectionPoint IP     = points2d.Value(i);
            double                     param1 = IP.ParamOnFirst();
            double                     param2 = IP.ParamOnSecond();
            Curve3D?                aCurve1, aCurve2;
            aCurve1 = Tool.Curve(edge1, out Placement L1, out _, out _);
            aCurve2 = Tool.Curve(edge2, out Placement L2, out _, out _);

            // if aMinDist lower than resolution than the intersection point lyes inside the vertex
            if (aMinDist < Constants.Resolution())
            {
                continue;
            }

            double aMaxEdgeTol1 = 0.0, aMaxEdgeTol2 = 0.0;
            if (aMinDist < RealLast() && aCurve1 is not null && aCurve2 is not null)
            {
                Line3D aLig = new Line3D(aNearestVertex, new Direction3D(new Vector3D(aNearestVertex, pint)));
                int    aPointsC;
                double du1   = 0.05 * (param1 - aVtx1Param);
                double du2   = 0.05 * (param2 - aVtx2Param);
                double tole1 = Tool.Tolerance(edge1);
                double tole2 = Tool.Tolerance(edge2);
                for (aPointsC = 2; aPointsC < 19; aPointsC++)
                {
                    double u  = aVtx1Param + aPointsC * du1;
                    Point3D P1 = aCurve1.Value(u);
                    P1.Transform(L1.Transformation());
                    double d1 = aLig.Distance(P1) * 2.0000001;
                    if (d1 > tole1 && d1 > aMaxEdgeTol1)
                    {
                        aMaxEdgeTol1 = d1;
                    }

                    u         = aVtx2Param + aPointsC * du2;
                    Point3D P2 = aCurve2.Value(u);
                    P2.Transform(L2.Transformation());
                    double d2 = aLig.Distance(P2) * 2.0000001;
                    if (d2 > tole2 && d2 > aMaxEdgeTol2)
                    {
                        aMaxEdgeTol2 = d2;
                    }
                }
                if (aMaxEdgeTol1 == 0.0 && aMaxEdgeTol2 == 0.0)
                {
                    continue;
                }
                // if the vertexies are far than tolerances so
                // we do not need to increase edge tolerance
                if (aNecessaryVtxTole > Max(aMaxEdgeTol1, tole1)
                    || aNecessaryVtxTole > Max(aMaxEdgeTol2, tole2))
                {
                    aMaxEdgeTol1 = 0.0;
                    aMaxEdgeTol2 = 0.0;
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
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                if (newTolers[rank] < finTol)
                {
                    if (Max(aMaxEdgeTol1, aMaxEdgeTol2) < finTol && (aMaxEdgeTol1 > 0 || aMaxEdgeTol2 > 0))
                    {
                        aNewTolEdge1 = Max(aNewTolEdge1, aMaxEdgeTol1);
                        aNewTolEdge2 = Max(aNewTolEdge2, aMaxEdgeTol2);
                    }
                    else
                    {
                        newTolers[rank] = finTol;
                    }
                }
            }
            else
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
            }
        }

        BrepBuilder B = new BrepBuilder();
        // update of tolerances of edges
        if (aNewTolEdge1 > 0)
        {
            for (i = 1; i <= 2; i++)
            {
                if (aNewTolEdge1 > Max(vertexTolers[i], newTolers[i]))
                {
                    newTolers[i] = aNewTolEdge1;
                }
            }
            B.UpdateEdge(edge1, aNewTolEdge1);
        }
        if (aNewTolEdge2 > 0)
        {
            for (i = 3; i <= 4; i++)
            {
                if (aNewTolEdge2 > Max(vertexTolers[i], newTolers[i]))
                {
                    newTolers[i] = aNewTolEdge2;
                }
            }
            B.UpdateEdge(edge2, aNewTolEdge2);
        }

        // update of tolerances of vertexies
        for (i = 1; i <= 4; i++)
        {
            if (newTolers[i] > 0)
            {
                B.UpdateVertex(ShapeCasts.Vertex(vertices[i]), newTolers[i]);
            }
        }

        if (!myShape.IsNull)
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
    private static bool TryBendingPCurve(in Edge E, in Face face, in Point2D p2d, bool end,
                                         ref Curve2D? c2d, ref double first, ref double last, ref double tol)
    {
        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        if (!sae.PCurve(E, face, out c2d, out first, out last, false))
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

                if (!TryNewPCurve(E, face, ref c2d, ref first, ref last, ref tol))
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
        enterWork(); // OCCT.NET: frozen
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        //=============
        // First phase: analysis whether the problem (gap) exists
        Point2D p2d1 = new Point2D(), p2d2 = new Point2D();
        myAnalyzer.CheckLacking(num, (force ? Precision() : 0.0), ref p2d1, ref p2d2);
        if (myAnalyzer.LastCheckStatus(ShapeExtend_FAIL))
        {
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
        }
        if (!myAnalyzer.LastCheckStatus(ShapeExtend_DONE))
        {
            return false;
        }

        //=============
        // Second phase: collection of data necessary for further analysis

        ShapeExtend_WireData sbwd = WireData()!;
        int                  n2   = (num > 0 ? num : sbwd.NbEdges());
        int                  n1   = (n2 > 1 ? n2 - 1 : sbwd.NbEdges());
        Edge          E1   = sbwd.Edge(n1);
        Edge          E2   = sbwd.Edge(n2);

        ShapeAnalysis_Edge sae = new ShapeAnalysis_Edge();
        Vertex      V1  = sae.LastVertex(E1);
        Vertex      V2  = sae.FirstVertex(E2);
        double             tol = Max(Tool.Tolerance(V1), Tool.Tolerance(V2));

        double Prec   = Precision();
        double dist2d = myAnalyzer.MaxDistance2d();
        double inctol = myAnalyzer.MaxDistance3d();

        Face            face = myAnalyzer.Face();
        ShapeAnalysis_Surface? surf = myAnalyzer.Surface();

        Point3D p3d1 = new Point3D(), p3d2 = new Point3D();
        double tol1 = Kerf.Engine.Foundation.Precision.Confusion(), tol2 = Kerf.Engine.Foundation.Precision.Confusion(); // SK

        //=============
        //: s2 abv 21 Apr 99: Speculation: try bending pcurves
        double        bendtol1 = 0.0, bendtol2 = 0.0;
        Curve2D? bendc1 = null, bendc2 = null;
        double        bendf1 = 0.0, bendl1 = 0.0, bendf2 = 0.0, bendl2 = 0.0;
        if (myGeomMode && !Tool.IsClosed(E1, face) && !Tool.IsClosed(E2, face))
        {
            Point2D p2d = new Point2D(0.5 * (p2d1.XY() + p2d2.XY()));
            bool     ok1 = TryBendingPCurve(E1,
                                        face,
                                        p2d,
                                        E1.Orientation() == Forward,
                                        ref bendc1,
                                        ref bendf1,
                                        ref bendl1,
                                        ref bendtol1);
            bool     ok2 = TryBendingPCurve(E2,
                                        face,
                                        p2d,
                                        E2.Orientation() == Reversed,
                                        ref bendc2,
                                        ref bendf2,
                                        ref bendl2,
                                        ref bendtol2);
            if (ok1 && !ok2)
            {
                bendtol2 = Tool.Tolerance(E2);
                ok1      = TryBendingPCurve(E1,
                                       face,
                                       p2d2,
                                       E1.Orientation() == Forward,
                                       ref bendc1,
                                       ref bendf1,
                                       ref bendl1,
                                       ref bendtol1);
            }
            else if (!ok1 && ok2)
            {
                bendtol1 = Tool.Tolerance(E1);
                ok2      = TryBendingPCurve(E2,
                                       face,
                                       p2d1,
                                       E2.Orientation() == Forward,
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
            && ((bendtol1 < Tool.Tolerance(E1) && bendtol2 < Tool.Tolerance(E2))
                || (inctol < Prec && bendtol1 < inctol && bendtol2 < inctol)))
        {
            doBend = true;

            // is it OK just to increase tolerance (to a value less than preci)?
        }
        else if (inctol < Prec)
        {
            doIncrease = true;

            // If increase is not OK or force, try to find other solutions (adding edge)
        }
        else if (!Tool.Degenerated(E2) && !Tool.Degenerated(E1))
        {

            // analyze the 3d space btw edges: is it enough to add long 3d edge?
            if (myTopoMode)
            {
                if (!sae.Curve3d(E1, out Curve3D? c3d, out double a, out double b, true))
                { // cannot work
                    myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                    return false;
                }
                p3d1             = c3d!.Value(b);
                double dist2d3d1 = p3d1.Distance(surf!.Value(p2d1));
                if (!sae.Curve3d(E2, out c3d, out a, out b, true))
                { // cannot work
                    myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                    return false;
                }
                p3d2             = c3d!.Value(a);
                double dist2d3d2 = p3d2.Distance(surf.Value(p2d2));

                tol1 = Max(Tool.Tolerance(E1), dist2d3d1);
                tol2 = Max(Tool.Tolerance(E2), dist2d3d2);
                //: c5  double tol0 = Max ( tol1 + tol2, thepreci );
                double tol0    = tol1 + tol2; //: c5 abv 26 Feb 98: CTS17806 #44418
                double dist3d2 = p3d1.SquareDistance(p3d2);

                // is it OK to add a long 3d edge?
                if (!myAnalyzer.LastCheckStatus(ShapeExtend_DONE2)
                    && //: 81 abv 20 Jan 98: don`t add back-going edges (zigzags)
                    dist3d2 > 1.25 * tol0 * tol0
                    && (force || dist3d2 > Prec * Prec || inctol > MaxTolerance()))
                {
                    doAddLong = true;
                }
            }

            //: h6 abv 25 Jun 98: BUC40132 6361: try to increase tol up to MaxTol if not add
            if (!doAddLong && inctol < MaxTolerance()
                && !myAnalyzer.Surface()!.IsDegenerated(p2d1, p2d2, 2.0 * tol, 10.0))
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
                Point3D pV = new Point3D(0.5 * (Tool.Pnt(V1).XYZ() + Tool.Pnt(V2).XYZ()));
                Point3D pm = myAnalyzer.Surface()!.Value(new Point2D(0.5 * (p2d1.XY() + p2d2.XY())));

                double dist = pV.Distance(pm);
                if (dist <= tol)
                {
                    doAddDegen = true;
                }
                else if (myTopoMode)
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

        else if (!Tool.Degenerated(E2) && Tool.Degenerated(E1))
        {
            // create new degenerated edge and replace E1 to new edge
        }
        else if (Tool.Degenerated(E2) && !Tool.Degenerated(E1))
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
                newV1 = new BRepBuilderAPI_MakeVertex(p3d1).InternalVertex();
                newV1.Reverse();
                newV2 = new BRepBuilderAPI_MakeVertex(p3d2).InternalVertex();
                B.UpdateVertex(newV1, 1.001 * tol1);
                B.UpdateVertex(newV2, 1.001 * tol2);
            }
            else
            {
                newV1 = V1;
                newV2 = V2;
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
            ShapeBuild_Edge sbe = new ShapeBuild_Edge();
            if (!doAddDegen && !sbe.BuildCurve3d(edge))
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
                return false;
            }

            // if long edge is added, replace vertices of adjacent edges
            if (doAddLong)
            {

                // replace 1st edge (n1==n2 - special case: wire consists of one edge)
                Edge edge1 = sbe.InternalCopyReplaceVertices(E1, (n1 == n2 ? newV2 : new Vertex()), newV1);
                sbwd.Set(edge1, n1);
                if (Context() is not null)
                {
                    Context()!.Replace(E1, edge1);
                    // actually, this will occur only in context of single face
                    // hence, recording to ReShape is rather for tracking modifications
                    // than for keeping sharing
                    Context()!.Replace(V1, newV1.Oriented(V1.Orientation()));
                    if (!V1.IsSame(V2))
                    {
                        Context()!.Replace(V2, newV2.Oriented(V2.Orientation()));
                    }
                }
                // replace 2nd edge
                if (n1 != n2)
                {
                    Edge edge2 = sbe.InternalCopyReplaceVertices(E2, newV2, new Vertex());
                    sbwd.Set(edge2, n2);
                    if (Context() is not null)
                    {
                        Context()!.Replace(E2, edge2);
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
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE3);
            }
            else if (!doAddLong)
            {
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE4);
            }
            sbwd.Add(edge, n2);
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
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
            B.UpdateEdge(E1, bendc1, face, bendtol1);
            B.Range(E1, face, bendf1, bendl1);
            B.UpdateEdge(E2, bendc2, face, bendtol2);
            B.Range(E2, face, bendf2, bendl2);
            B.UpdateVertex(sae.FirstVertex(E1), bendtol1);
            B.UpdateVertex(sae.LastVertex(E1), bendtol1);
            B.UpdateVertex(sae.FirstVertex(E2), bendtol2);
            B.UpdateVertex(sae.LastVertex(E2), bendtol2);
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
            //: s3 abv 22 Apr 99: PRO7187 #11534: self-intersection not detected unitil curve is bent (!)
            FixSelfIntersectingEdge(n1);
            FixSelfIntersectingEdge(n2);
            FixIntersectingEdges(n2); // skl 24.04.2003 for OCC58
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE5);
        }

        // increase vertex tolerance
        if (doIncrease)
        {
            B.UpdateVertex(V1, 1.001 * inctol);
            B.UpdateVertex(V2, 1.001 * inctol);
            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
        }

        if (LastFixStatus(ShapeExtend_DONE))
        {
            return true;
        }

        myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
        return false;
    }

    /// <summary>Fixes notched edges (an edge going back along its neighbour): splits it and removes the notch.</summary>
    public bool FixNotchedEdges()
    {
        enterWork(); // OCCT.NET: frozen
        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (!IsReady())
        {
            return false;
        }

        ShapeAnalysis_Wire theAdvAnalyzer = myAnalyzer;
        Face        face           = Face();
        if (Context() is not null)
        {
            UpdateWire();
        }
        ShapeExtend_WireData sewd = WireData()!;

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
                ShapeAnalysis_Edge sae           = new ShapeAnalysis_Edge();
                sae.PCurve(splitE, face, out Curve2D? c2d, out double a, out double b, true);
                ShapeBuild_Edge    sbe    = new ShapeBuild_Edge();
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

                    ShapeAnalysis_TransferParametersProj transferParameters = new ShapeAnalysis_TransferParametersProj();
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
                    Vertex Vnew = new Vertex();
                    BrepBuilder  B    = new BrepBuilder();
                    B.MakeVertex(ref Vnew,
                                 Analyzer().Surface()!.Value(c2d!.Value(param)),
                                 Kerf.Engine.Foundation.Precision.Confusion());
                    Edge wE = splitE;
                    wE.Orientation(Forward);
                    Shape aTmpShape = Vnew.Oriented(Reversed); // for porting
                    Edge newE1 =
                        sbe.InternalCopyReplaceVertices(wE, sae.FirstVertex(wE), ShapeCasts.Vertex(aTmpShape));
                    sbe.CopyPCurves(newE1, wE);
                    transferParameters.TransferRange(newE1, first, param, true);
                    B.SameRange(newE1, false);
                    B.SameParameter(newE1, false);
                    aTmpShape = Vnew.Oriented(Forward);
                    Edge newE2 =
                        sbe.InternalCopyReplaceVertices(wE, ShapeCasts.Vertex(aTmpShape), sae.LastVertex(wE));
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
                    myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                }

                i--;
                if (Context() is not null)
                { // skl 07.03.2002 for OCC180
                    UpdateWire();
                }
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
            }
        }
        myStatusNotches = myLastFixStatus;
        return LastFixStatus(ShapeExtend_DONE);
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
                Placement                            L       = fromGC.Location();
                bool                                       found   = false;
                NCollection_List<CurveRepresentation> tolist  = ((BrepTEdge)toedge.TShape()!).ChangeCurves();
                GCurve?                               toGC    = null;
                for (var toitcr = new NCollection_List<CurveRepresentation>.Iterator(tolist);
                     toitcr.More() && !found;
                     toitcr.Next())
                {
                    toGC = toitcr.Value() as GCurve;
                    if (toGC is null || !toGC.IsCurveOnSurface() || surface != toGC.Surface()
                        || L != toGC.Location())
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
                    Placement newLoc = (fromLoc * L).Predivided(toLoc);
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
        ShapeAnalysis_Edge   sae  = new ShapeAnalysis_Edge();
        ShapeBuild_Edge      sbe  = new ShapeBuild_Edge();
        ShapeBuild_Vertex    sbv  = new ShapeBuild_Vertex();
        int                  num1 = (num == NbEdges()) ? 1 : num + 1;
        ShapeExtend_WireData sewd = WireData()!;
        Edge          E1 = sewd.Edge(num), E2 = sewd.Edge(num1);
        Vertex        V1 = sae.FirstVertex(E1), V2 = sae.LastVertex(E2);
        Vertex        Vm = sbv.InternalCombineVertex(V1, V2, 1.0001);

        // pnd defining if new pcurves exists
        // pdn Temporary not removed
        //  bool toRemove = !(HasNewPCurves(E1,E2)||HasNewPCurves(E2,E1));
        bool toRemove = false;

        // creating new edge with pcurves and new vertex
        Vertex Vs = sae.FirstVertex(E2);
        if (Vs.IsSame(V1) || Vs.IsSame(V2))
        {
            Vs = Vm;
        }
        Edge newEdge = sbe.InternalCopyReplaceVertices(E2, Vs, Vm);
        CopyReversePcurves(newEdge, E1, E1.Orientation() == E2.Orientation());
        BrepBuilder B = new BrepBuilder();
        B.SameRange(newEdge, false);
        B.SameParameter(newEdge, false);

        if (Context() is not null)
        {
            if (toRemove)
            {
                Context()!.Remove(E2);
                Context()!.Remove(E1);
            }
            else
            {
                Context()!.Replace(E2, newEdge);
                Context()!.Replace(E1, newEdge.Reversed());
            }
            Context()!.Replace(V1, Vm.Oriented(V1.Orientation()));
            Context()!.Replace(V2, Vm.Oriented(V2.Orientation()));
        }

        int         next  = (num1 == NbEdges()) ? 1 : num1 + 1;
        int         prev  = (num > 1) ? num - 1 : NbEdges();
        Edge prevE = sewd.Edge(prev), nextE = sewd.Edge(next);

        Edge tmpE1 = sbe.InternalCopyReplaceVertices(prevE, new Vertex(), Vm);
        sewd.Set(tmpE1, prev);
        if (Context() is not null)
        {
            Context()!.Replace(prevE, tmpE1);
        }

        tmpE1 = sbe.InternalCopyReplaceVertices(nextE, Vm, new Vertex());
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
        ShapeExtend_WireData sbwd = WireData()!;
        for (int i = 1; i <= sbwd.NbEdges(); i++)
        {
            Edge  E = sbwd.Edge(i);
            Shape S = Context()!.InternalApply(E);
            if (S == E)
            {
                continue;
            }
            for (Explorer exp = new Explorer(S, ShapeType.Edge); exp.More(); exp.Next())
            {
                sbwd.Add(exp.Current(), i++);
            }
            sbwd.Remove(i--);
        }
    }

    /// <summary>Fixes tails (thin spikes between adjacent edges), with the maximal angle and width set.</summary>
    public bool FixTails()
    {
        enterWork(); // OCCT.NET: frozen
        if (myMaxTailWidth < 0 || !IsReady())
        {
            return false;
        }

        myLastFixStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (Context() is not null)
        {
            UpdateWire();
        }
        ShapeExtend_WireData aSEWD   = WireData()!;
        int                  aECount = NbEdges();
        int[]                aENs    = [aECount, 1];
        bool                 aCheckAngle = true;
        while (aECount >= 2 && aENs[1] <= aECount)
        {
            Edge[] aEs     = [aSEWD.Edge(aENs[0]), aSEWD.Edge(aENs[1])];
            var           aEParts = new Edge[2, 2];
            for (int k = 0; k < 2; ++k)
            {
                aEParts[k, 0] = new Edge();
                aEParts[k, 1] = new Edge();
            }
            // OCCT.NET: frozen: the internal form leaves the cut parts unfrozen, since they go into the working wire
            if (!myAnalyzer.InternalCheckTail(aEs[0],
                                              aEs[1],
                                              aCheckAngle ? myMaxTailAngleSine : -1,
                                              myMaxTailWidth,
                                              MaxTolerance(),
                                              ref aEParts[0, 0],
                                              ref aEParts[0, 1],
                                              ref aEParts[1, 0],
                                              ref aEParts[1, 1]))
            {
                aENs[0]     = aENs[1]++;
                aCheckAngle = true;
                continue;
            }

            // Provide not less than 1 edge in the result wire.
            int[] aSplitCounts = [aEParts[0, 1].IsNull ? 0 : 1, aEParts[1, 1].IsNull ? 0 : 1];
            int   aRemoveCount = (aEParts[0, 0].IsNull ? 0 : 1) + (aEParts[1, 0].IsNull ? 0 : 1);
            if (aECount + aSplitCounts[0] + aSplitCounts[1] < 1 + aRemoveCount)
            {
                aENs[0]     = aENs[1]++;
                aCheckAngle = true;
                continue;
            }

            // Split the edges.
            for (int aEI = 0; aEI < 2; ++aEI)
            {
                if (aSplitCounts[aEI] == 0)
                {
                    continue;
                }

                // Replace the edge by the wire of its parts in the shape.
                Edge aE = aEs[aEI];
                if (Context() is not null)
                {
                    Wire aEWire = new Wire();
                    new BrepBuilder().MakeWire(ref aEWire);
                    new BrepBuilder().Add(aEWire, aEParts[aEI, 0]);
                    new BrepBuilder().Add(aEWire, aEParts[aEI, 1]);
                    Edge aFE = ShapeCasts.Edge(aE.Oriented(Forward));
                    Context()!.Replace(aFE, aEWire);
                }

                // Replace the edge by its parts in the edge wire.
                Orientation aOrient = aE.Orientation();
                aEParts[aEI, 0].Orientation(aOrient);
                aEParts[aEI, 1].Orientation(aOrient);
                int aFirstPI = (aOrient != Reversed) ? 0 : 1;
                int aAdd     = (aEI == 0 || aENs[1] < aENs[0]) ? 0 : aSplitCounts[0];
                aSEWD.Set(aEParts[aEI, aFirstPI], aENs[aEI] + aAdd);
                aSEWD.Add(aEParts[aEI, 1 - aFirstPI], aENs[aEI] + 1 + aAdd);
            }

            // Remove the tail.
            if (aRemoveCount == 2)
            {
                aCheckAngle = true;
                FixDummySeam(aENs[0] + aSplitCounts[0] + ((aENs[0] < aENs[1]) ? 0 : aSplitCounts[1]));
                if (Context() is not null)
                {
                    UpdateWire();
                }
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE);

                if (aSplitCounts[0] + aSplitCounts[1] == 2)
                {
                    aENs[0] = aENs[1]++;
                    continue;
                }

                if (aSplitCounts[0] == aSplitCounts[1])
                {
                    aECount -= 2;
                    if (aENs[1] >= 3)
                    {
                        --aENs[0];
                        --aENs[1];
                    }
                    else
                    {
                        aENs[0] = aECount;
                        aENs[1] = 1;
                    }
                    aCheckAngle = false;
                }
                else
                {
                    --aECount;
                    if (aSplitCounts[0] != 0)
                    {
                        aENs[0] = (aENs[0] <= aECount) ? aENs[0] : aECount;
                    }
                    else
                    {
                        if (aENs[1] >= 3)
                        {
                            --aENs[0];
                            --aENs[1];
                        }
                        else
                        {
                            aENs[0] = aECount;
                            aENs[1] = 1;
                        }
                    }
                }
            }
            else
            {
                aCheckAngle = false;
                --aECount;
                int aRI = aEParts[0, 0].IsNull ? 1 : 0;
                if (aSplitCounts[aRI] != 0)
                {
                    if (aRI == 0)
                    {
                        if (aENs[1] >= 3)
                        {
                            --aENs[0];
                            --aENs[1];
                        }
                        else
                        {
                            aENs[0] = aECount;
                            aENs[1] = 1;
                        }
                    }
                    else
                    {
                        aENs[0] = (aENs[1] > 1) ? aENs[0] : aECount;
                    }
                }
                aSEWD.Remove(aENs[aRI] + ((aRI != 0 || aSplitCounts[0] == 0) ? 0 : 1));
                if (Context() is not null)
                {
                    Context()!.Remove(aEs[aRI].Oriented(Forward));
                    UpdateWire();
                }
                myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE);
            }
        }
        // OCCT: the tails' status goes into myStatusNotches; StatusFixTails is never set
        myStatusNotches = myLastFixStatus;
        return ShapeExtend.DecodeStatus(myLastFixStatus, ShapeExtend_DONE);
    }
}
