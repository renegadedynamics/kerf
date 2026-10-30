// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Wire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Fixes a wire: the order of its edges, small edges, the connection of adjacent edges, the edges' curves (pcurves,
/// 3d curves, seams, shifted pcurves, same parameter, vertex tolerances), degenerated edges, self-intersections,
/// lacking edges, notched edges, tails and gaps in 3d and 2d. The wire may be on a face (the 2d fixes need one).
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). Load a wire (with <see cref="Init(in TopoDS_Wire, in TopoDS_Face, double)"/> or
/// <see cref="Load(in TopoDS_Wire)"/>), set its face and precision, then call <see cref="Perform"/> or the fixes one by
/// one; each sets its own status (<c>Status...</c>), and the advanced fixes (by edge number) share
/// <see cref="LastFixStatus"/>. The modes (<c>...Mode()</c>) are three-position flags: -1 is the default, 0 off, 1 on.
/// </para>
/// <para>
/// The fixes edit the edges of the loaded wire in place, as OCCT's. A loaded wire holding frozen shapes (any wire a
/// caller holds) is copied at the first fix (<see cref="BRepTools_CopyAtEntry"/>, decision 583), so the caller's wire
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
    public void SetFace(in TopoDS_Face face)
    {
        myAnalyzer.SetFace(face);
    }

    /// <summary>Sets the face of the wire with an analyser of its surface.</summary>
    public void SetFace(in TopoDS_Face theFace, ShapeAnalysis_Surface? theSurfaceAnalysis)
    {
        myAnalyzer.SetFace(theFace, theSurfaceAnalysis);
    }

    /// <summary>Sets the surface analyser (and through it the surface) of the wire.</summary>
    public void SetSurface(ShapeAnalysis_Surface? theSurfaceAnalysis)
    {
        myAnalyzer.SetSurface(theSurfaceAnalysis);
    }

    /// <summary>Sets the surface of the wire.</summary>
    public void SetSurface(Geom_Surface? surf)
    {
        myAnalyzer.SetSurface(surf);
    }

    /// <summary>Sets the surface of the wire with a location.</summary>
    public void SetSurface(Geom_Surface? surf, in TopLoc_Location loc)
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
    public TopoDS_Face Face() => myAnalyzer.Face();

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
    public ShapeFix_Wire(in TopoDS_Wire wire, in TopoDS_Face face, double prec)
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
    public void Init(in TopoDS_Wire wire, in TopoDS_Face face, double prec)
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
    public void Load(in TopoDS_Wire wire)
    {
        ClearStatuses();

        TopoDS_Wire W = wire;
        if (Context() is not null)
        {
            TopoDS_Shape S = Context()!.InternalApply(wire);
            W = TopoDS.Wire(S);
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
    public bool Perform(Message_ProgressRange theProgress = default)
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
        if (myAnalyzer.Surface() is not null && myAnalyzer.Surface()!.Surface()!.IsUPeriodic()
            && myAnalyzer.Surface()!.Surface()!.IsVPeriodic() && theModeBoth)
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
            TopoDS_Edge  aEPrev   = aWireSBWD.Edge(aN1Next);
            TopoDS_Shape aRefresh = Context()!.InternalApply(aEPrev);
            if (aRefresh.IsNull() || aRefresh.IsSame(aEPrev) || aRefresh.ShapeType() != TopAbs_EDGE)
            {
                continue;
            }
            aWireSBWD.Set(TopoDS.Edge(aRefresh), aN1Next);
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
        TopoDS_Face          face          = Face();
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
                        TopoDS_Edge           E   = sbwd.Edge(i);
                        var                   SAC = new ShapeAnalysis_Curve();
                        Geom_Curve            C   = BRep_Tool.Curve(E, out double a, out double b)!;
                        ShapeAnalysis_Surface S   = myAnalyzer.Surface()!;
                        int                   nbs = S.NbSingularities(MinTolerance());
                        var                   GAC = new GeomAdaptor_Curve(C, a, b);
                        var                   seq = new NCollection_Sequence<double>();
                        for (int j = 1; j <= nbs; j++)
                        {
                            double   Preci = 0.0;
                            gp_Pnt2d pd1 = new gp_Pnt2d(), pd2 = new gp_Pnt2d();
                            gp_Pnt   P3d = new gp_Pnt();
                            double   par1 = 0.0, par2 = 0.0;
                            bool     tmpUIsoDeg = false;
                            S.Singularity(j, ref Preci, ref P3d, ref pd1, ref pd2, ref par1, ref par2, ref tmpUIsoDeg);
                            if (SAC.Project(GAC, P3d, MinTolerance(), out gp_Pnt pr, out double split, true)
                                < Max(Preci, MinTolerance()))
                            {
                                if (split - a > Occt.Precision.PConfusion() && b - split > Occt.Precision.PConfusion())
                                {
                                    int k;
                                    for (k = 1; k <= seq.Length(); k++)
                                    {
                                        if (split < seq[k] - Occt.Precision.PConfusion())
                                        {
                                            seq.InsertBefore(k, split);
                                            break;
                                        }
                                        else if (split < seq[k] + Occt.Precision.PConfusion())
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
                            bool isFwd = (E.Orientation() == TopAbs_FORWARD);
                            E.Orientation(TopAbs_FORWARD);

                            // if( BRep_Tool::SameParameter(sbwd->Edge(i)) )
                            //   sbe.RemovePCurve ( E, face );

                            // 10.04.2003 skl for using trimmed lines as pcurves
                            var sae = new ShapeAnalysis_Edge();
                            if (BRep_Tool.SameParameter(sbwd.Edge(i)))
                            {
                                sbe.RemovePCurve(E, face);
                            }
                            else
                            {
                                if (sae.HasPCurve(E, face))
                                {
                                    if (sae.PCurve(E, face, out Geom2d_Curve? C2d, out double fp2d, out double lp2d))
                                    {
                                        if (C2d is not Geom2d_TrimmedCurve)
                                        {
                                            sbe.RemovePCurve(E, face);
                                        }
                                    }
                                }
                            }

                            //    myFixEdge->FixSameParameter ( E ); // to ensure SameRange & SP
                            var           B = new BRep_Builder();
                            TopoDS_Vertex V1, V2, V = new TopoDS_Vertex();
                            // ShapeAnalysis_Edge sae;
                            V1 = sae.FirstVertex(E);
                            V2 = sae.LastVertex(E);

                            var sw = new ShapeExtend_WireData();
                            for (int k = 0; k <= seq.Length(); k++)
                            {
                                double split = (k < seq.Length() ? seq[k + 1] : b);
                                if (k < seq.Length())
                                {
                                    B.MakeVertex(ref V, C.Value(split), BRep_Tool.Tolerance(E));
                                    // try increase tolerance before splitting
                                    double aDist = BRep_Tool.Pnt(V1).Distance(BRep_Tool.Pnt(V));
                                    if (aDist < BRep_Tool.Tolerance(V1) * 1.01)
                                    {
                                        B.UpdateVertex(V1, Max(aDist, BRep_Tool.Tolerance(V1)));
                                        a  = split;
                                        V1 = V;
                                        continue;
                                    }
                                    else
                                    {
                                        aDist = BRep_Tool.Pnt(V2).Distance(BRep_Tool.Pnt(V));
                                        if (aDist < BRep_Tool.Tolerance(V2) * 1.01)
                                        {
                                            B.UpdateVertex(V, Max(aDist, BRep_Tool.Tolerance(V2)));
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

                                TopoDS_Edge edge = sbe.InternalCopyReplaceVertices(E, V1, V);
                                if (BRep_Tool.SameParameter(sbwd.Edge(i)))
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
                                E.Orientation(TopAbs_REVERSED);
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
                gp_XY vec = new gp_XY(0, 0);
                var   sae = new ShapeAnalysis_Edge();
                int   k;
                for (k = 1; k <= nb; k++)
                {
                    if (!sae.PCurve(sbwd.Edge(k), face, out Geom2d_Curve? c2d, out double cf, out double cl, true))
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
                    BRep_Tool.CurveOnSurface(sbwd.Edge(i), out Geom2d_Curve? C, out Geom_Surface? S, out TopLoc_Location L,
                                             out double first, out double last);
                    if (C is null || Math.Abs(last - first) < Occt.Precision.PConfusion())
                    {
                        // Incomplete edge (with no pcurves or 3d curve) removed
                        SendWarning(sbwd.Edge(i), new Message_Msg("FixWire.FixCurve3d.Removed"));
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
                Geom_Curve? tmpc3d = BRep_Tool.Curve(sbwd.Edge(i), out double First, out double Last);
                if (sae.HasPCurve(sbwd.Edge(i), face))
                {
                    if (sae.PCurve(sbwd.Edge(i), face, out Geom2d_Curve? C2d, out double fp2d, out double lp2d, false))
                    {
                        if (Math.Abs(First - fp2d) > Occt.Precision.PConfusion()
                            || Math.Abs(Last - lp2d) > Occt.Precision.PConfusion())
                        {
                            var B = new BRep_Builder();
                            B.SameRange(sbwd.Edge(i), false);
                        }
                        else if (!sae.CheckPCurveRange(First, Last, C2d!))
                        {
                            // Replace pcurve
                            Geom_Surface? S = BRep_Tool.Surface(face, out TopLoc_Location L);
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
                var                  B    = new BRep_Builder();
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

        var newedges = new NCollection_HSequence<TopoDS_Shape>();
        for (i = 1; i <= nb; i++)
        {
            newedges.Append(sbwd.Edge(wi.Ordered(i)));
        }
        for (i = 1; i <= nb; i++)
        {
            sbwd.Set(TopoDS.Edge(newedges.Value(i)), i);
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
        SendWarning(WireData()!.Edge(n), new Message_Msg("FixAdvWire.FixSmall.MSG0")); // Small edge(s) removed
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
        TopoDS_Edge          E1   = sbwd.Edge(n1);
        TopoDS_Edge          E2   = sbwd.Edge(n2);

        var           sae = new ShapeAnalysis_Edge();
        TopoDS_Vertex V1  = sae.LastVertex(E1);
        TopoDS_Vertex V2  = sae.FirstVertex(E2);
        TopoDS_Vertex V;

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
                var B = new BRep_Builder();
                B.Remove(E2, sae.FirstVertex(E2));
                B.Remove(E2, sae.LastVertex(E2));
                B.Add(E2, V.Oriented(TopAbs_FORWARD));
                B.Add(E2, V.Oriented(TopAbs_REVERSED));
            }
            else
            {
                TopoDS_Edge tmpE = sbe.InternalCopyReplaceVertices(E2, V, V);
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
                var B = new BRep_Builder();
                B.Remove(E2, sae.FirstVertex(E2));
                B.Add(E2, V.Oriented(TopAbs_FORWARD));
                if (!myAnalyzer.LastCheckStatus(ShapeExtend_DONE1) || sae.FirstVertex(E2).IsSame(sae.LastVertex(E2)))
                {
                    B.Remove(E1, sae.LastVertex(E1));
                    B.Add(E1, V.Oriented(TopAbs_REVERSED));
                }
            }
            else
            {
                TopoDS_Edge tmpE2 = sbe.InternalCopyReplaceVertices(E2, V, new TopoDS_Vertex());
                sbwd.Set(tmpE2, n2);
                if (Context() is not null)
                {
                    Context()!.Replace(E2, tmpE2);
                }
                if (!myAnalyzer.LastCheckStatus(ShapeExtend_DONE1) || sae.FirstVertex(E2).IsSame(sae.LastVertex(E2)))
                {
                    TopoDS_Edge tmpE1 = sbe.InternalCopyReplaceVertices(E1, new TopoDS_Vertex(), V);
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

        Geom2d_Curve? C1 = null, C2 = null;
        double        cf = 0.0, cl = 0.0;
        if (!myAnalyzer.CheckSeam(num, ref C1, ref C2, ref cf, ref cl))
        {
            return false;
        }

        var         B = new BRep_Builder();
        TopoDS_Edge E = WireData()!.Edge(num > 0 ? num : NbEdges());
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
    private static void UpdateEdgeUVPoints(in TopoDS_Edge E, in TopoDS_Face F)
    {
        BRep_Tool.Range(E, F, out double first, out double last);
        new BRep_Builder().Range(E, F, first, last);
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
        bool vclosed = surf.IsVClosed(Precision()) || surf.Surface() is Geom_SphericalSurface;
        // #67 rln 01.03.99 S4135: ims010.igs entity D11900 (2D contour is 2*PI higher than V range
        // [-pi/2,p/2])

        // PTV 26.06.2002 begin
        // CTS18546-2.igs entity 2222: base curve is periodic and 2dcurve is shifted
        bool   IsVCrvClosed = false;
        double VRange       = 1.0;
        if (surf.Surface() is Geom_SurfaceOfRevolution aSurOfRev)
        {
            Geom_Curve aBaseCrv = aSurOfRev.BasisCurve();
            while ((aBaseCrv is Geom_OffsetCurve) || (aBaseCrv is Geom_TrimmedCurve))
            {
                if (aBaseCrv is Geom_OffsetCurve anOffset)
                {
                    aBaseCrv = anOffset.BasisCurve();
                }
                if (aBaseCrv is Geom_TrimmedCurve aTrimmed)
                {
                    aBaseCrv = aTrimmed.BasisCurve();
                }
            }
            if (aBaseCrv.IsPeriodic())
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
            TopoDS_Edge E1 = sbwdOring.Edge(i);
            if (BRep_Tool.Degenerated(E1) && !sae.HasPCurve(E1, Face()))
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
        gp_Pnt pdeg  = new gp_Pnt();
        // pdn 17.12.98 r0901_ec 38237 to shift wire at 0

        var box = new Bnd_Box2d();
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

            TopoDS_Edge E1 = sbwd.Edge(n1);
            TopoDS_Edge E2 = sbwd.Edge(n2);

            if (BRep_Tool.Degenerated(E1) || BRep_Tool.Degenerated(E2))
            {
                if (!degstop)
                {
                    stop    = n2;
                    degstop = true;
                }
                continue;
            }

            TopoDS_Vertex V = sae.FirstVertex(E2);
            if (V.IsNull())
            {
                continue;
            }

            gp_Pnt p = BRep_Tool.Pnt(V);

            double        a1 = 0.0, b1 = 0.0, a2 = 0.0, b2 = 0.0;
            Geom2d_Curve? c2d1 = null, c2d2 = null;

            //: abv 29.08.01: torCuts.sat: distinguish degeneration by U and by V;
            // only corresponding move is prohibited
            //    bool isDeg = surf->IsDegenerated ( p, Max ( Precision(),
            //    BRep_Tool::Tolerance(V) ) );
            int      isDeg = 0;
            gp_Pnt2d degP1 = new gp_Pnt2d(), degP2 = new gp_Pnt2d();
            double   degT1 = 0.0, degT2 = 0.0;
            if (surf.DegeneratedValues(p, Max(Precision(), BRep_Tool.Tolerance(V)), ref degP1, ref degP2, ref degT1,
                                       ref degT2))
            {
                isDeg = (Math.Abs(degP1.X() - degP2.X()) > Math.Abs(degP1.Y() - degP2.Y()) ? 1 : 2);
            }

            // abv 23 Feb 00: UKI60107-6 210: additional check for near-degenerated case
            // smh#15 PRO19800. Check if the surface is surface of revolution.
            if (surf.Surface() is Geom_SurfaceOfRevolution)
            {
                if (isDeg == 0 && !vclosed)
                {
                    if (c2d1 is null && !sae.PCurve(E1, Face(), out c2d1, out a1, out b1, true))
                    {
                        continue;
                    }
                    gp_Pnt2d p1 = new gp_Pnt2d(SUF, c2d1!.Value(b1).Y());
                    gp_Pnt2d p2 = new gp_Pnt2d(SUL, c2d1.Value(b1).Y());
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
                    gp_Pnt2d p1 = new gp_Pnt2d(c2d1!.Value(b1).X(), SVF);
                    gp_Pnt2d p2 = new gp_Pnt2d(c2d1.Value(b1).X(), SVL);
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
                        Geom2d_Curve? cx1 = null, cx2 = null;
                        if ((c2d1 is null && !sae.PCurve(E1, Face(), out c2d1, out a1, out b1, true))
                            || (c2d2 is null && !sae.PCurve(E2, Face(), out c2d2, out a2, out b2, true))
                            || !sae.PCurve(sbwd.Edge(degn2 > 1 ? degn2 - 1 : nb), Face(), out cx1, out ax1, out bx1, true)
                            || !sae.PCurve(sbwd.Edge(degn2), Face(), out cx2, out ax2, out bx2, true))
                        {
                            myLastFixStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
                            continue;
                        }
                        gp_Pnt2d pd1 = cx1!.Value(bx1);
                        gp_Pnt2d pd2 = cx2!.Value(ax2);
                        gp_Pnt2d pn1 = c2d1!.Value(b1);
                        gp_Pnt2d pn2 = c2d2!.Value(a2);
                        gp_Vec2d x   = new gp_Vec2d(0.0, 0.0); // shift vector
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
                        if (rot1 * rot2 < -Occt.Precision.PConfusion() && scld * scln < -Occt.Precision.PConfusion()
                            && Math.Abs(scln) > 0.1 * period && Math.Abs(scld) > 0.1 * period
                            && rot1 * scld > Occt.Precision.PConfusion() && rot2 * scln > Occt.Precision.PConfusion())
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
                            double dx = ShapeAnalysis.AdjustToPeriod(deep, Occt.Precision.PConfusion(),
                                                                     period + Occt.Precision.PConfusion());
                            x *= (scld > 0 ? -dx : dx);
                            // x *= ( std::abs(scld-scln) > 1.5 * period ? 2. : 1. ) *
                            //      ( scld >0 ? -period : period );
                            var Shift = new gp_Trsf2d();
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
                                TopoDS_Edge edge = sbwd.Edge(k);
                                if (!sae.PCurve(edge, Face(), out cx1, out ax1, out bx1, true))
                                {
                                    continue;
                                }
                                // cx1->Transform ( Shift );
                                //  skl 15.05.2002 for OCC208 (if few edges have reference to one pcurve)
                                var cx1new = cx1!.Transformed(Shift) as Geom2d_Curve;
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
            gp_Pnt2d p2d1 = c2d1!.Value(b1);
            gp_Pnt2d p2d2 = c2d2!.Value(a2);
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
            var Shift2 = new gp_Trsf2d();
            Shift2.SetTranslation(new gp_Vec2d(du, dv));
            // c2d2->Transform ( Shift );
            //  skl 15.05.2002 for OCC208 (if few edges have reference to one pcurve)
            var c2d2new = c2d2.Transformed(Shift2) as Geom2d_Curve;
            sbe.ReplacePCurve(E2, c2d2new, Face());
            UpdateEdgeUVPoints(E2, Face());
        }
        if (box.IsVoid())
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
            if (!sae.PCurve(sbwd.Edge(n), Face(), out Geom2d_Curve? c2d, out double a, out double b, true))
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

        var Shift3 = new gp_Trsf2d();
        Shift3.SetTranslation(new gp_Vec2d(du2, dv2));

        for (n = 1; n <= sbwdOring.NbEdges(); n++)
        {
            TopoDS_Edge ed = sbwdOring.Edge(n);
            if (!sae.PCurve(ed, Face(), out Geom2d_Curve? c2d, out double a, out double b, true))
            {
                continue;
            }
            // skl 15.05.2002 for OCC208 (if few edges have reference to one pcurve)
            var c2d2 = c2d!.Transformed(Shift3) as Geom2d_Curve;
            sbe.ReplacePCurve(ed, c2d2, Face());
            UpdateEdgeUVPoints(ed, Face());
        }
        return true;
    }
}
