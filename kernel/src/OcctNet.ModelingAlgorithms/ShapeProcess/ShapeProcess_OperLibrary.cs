// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_OperLibrary.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_OperLibrary.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The library of Shape Healing's operators: DirectFaces, SameParameter, SetTolerance, SplitAngle,
/// BSplineRestriction, ElementaryToRevolution, SweptToElementary, SurfaceToBSpline, ToBezier, SplitContinuity,
/// SplitClosedFaces, FixWireGaps, FixFaceSize, DropSmallSolids, DropSmallEdges, FixShape, SplitClosedEdges and
/// SplitCommonVertex.
/// </summary>
/// <remarks>
/// <see cref="Init"/> registers them once per process (OCCT's static <c>done</c> flag, plan 2.3; the port registers
/// under a lock, so a second thread waits until they are all registered). The operators work on the context's
/// <see cref="ShapeProcess_ShapeContext.InternalResult"/>, which is the context's working copy while they run
/// (decision 1022), and read the tools' unfrozen results (decisions 973, 999); the context freezes at its own
/// publication points.
/// </remarks>
public static class ShapeProcess_OperLibrary
{
    private static readonly Lock theInitLock = new();
    private static bool done;

    /// <summary>
    /// Applies a BRepTools_Modification to a shape, taking the sharing of the components of compounds into account.
    /// With <paramref name="theMutableInput"/>, the modifier may update the input's vertices in place; on a shape holding
    /// frozen shapes the port then runs it in its immutable mode (decision 1022). The result is frozen.
    /// </summary>
    public static TopoDS_Shape ApplyModifier(in TopoDS_Shape S,
                                             ShapeProcess_ShapeContext context,
                                             BRepTools_Modification M,
                                             NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> map,
                                             ShapeExtend_MsgRegistrator? msg = null,
                                             bool theMutableInput = false)
    {
        TopoDS_Shape aResult = InternalApplyModifier(S, context, M, map, msg, theMutableInput && !BRepTools.HoldsFrozen(S));
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary><see cref="ApplyModifier"/> without freezing the result, for the operators.</summary>
    internal static TopoDS_Shape InternalApplyModifier(in TopoDS_Shape S,
                                                       ShapeProcess_ShapeContext context,
                                                       BRepTools_Modification M,
                                                       NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> map,
                                                       ShapeExtend_MsgRegistrator? msg,
                                                       bool theMutableInput)
    {
        // protect against INTERNAL/EXTERNAL shapes
        TopoDS_Shape SF = S.Oriented(TopAbs_FORWARD);

        // Process COMPOUNDs separately in order to handle sharing in assemblies
        if (SF.ShapeType() == TopAbs_COMPOUND)
        {
            bool locModified = false;
            var C = new TopoDS_Compound();
            var B = new BRep_Builder();
            B.MakeCompound(ref C);
            for (var it = new TopoDS_Iterator(SF); it.More(); it.Next())
            {
                TopoDS_Shape shape = it.Value();
                Placement L = shape.Location(), nullLoc = new();
                shape.Location(nullLoc);
                TopoDS_Shape res;
                if (map.IsBound(shape))
                {
                    res = map.Find(shape).Oriented(shape.Orientation());
                }
                else
                {
                    res = InternalApplyModifier(shape, context, M, map, null, theMutableInput);
                    map.Bind(shape, res);
                }
                if (!res.IsSame(shape))
                {
                    locModified = true;
                }
                res.Location(L, false);
                B.Add(C, res);
            }
            if (!locModified)
            {
                return S;
            }

            map.Bind(SF, C);
            return C.Oriented(S.Orientation());
        }

        // Modify the shape
        var MD = new BRepTools_Modifier(SF);
        MD.SetMutableInput(theMutableInput);
        MD.Perform(M);
        context.RecordModification(SF, MD, msg);
        return MD.InternalModifiedShape(SF).Oriented(S.Orientation());
    }

    // activate message mechanism if it is supported by context
    private static ShapeExtend_MsgRegistrator? NewMessages(ShapeProcess_ShapeContext ctx)
        => ctx.Messages() is not null ? new ShapeExtend_MsgRegistrator() : null;

    // The modifiers run on the context's working copy, in the mutable-input mode as OCCT's (decision 1022)
    private static bool ApplyModification(ShapeProcess_ShapeContext ctx, BRepTools_Modification theModification,
                                          ShapeExtend_MsgRegistrator? msg)
    {
        var map = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopoDS_Shape aWork = ctx.InternalResult();
        TopoDS_Shape res = InternalApplyModifier(aWork, ctx, theModification, map, msg, !BRepTools.HoldsFrozen(aWork));
        ctx.RecordModification(map, msg);
        ctx.SetResult(res);
        return true;
    }

    private static bool directfaces(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        var DM = new ShapeCustom_DirectModification();
        DM.SetMsgRegistrator(msg);
        return ApplyModification(ctx, DM, msg);
    }

    private static bool sameparam(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        ShapeFix.SameParameter(ctx.InternalResult(),
                               ctx.BooleanVal("Force", false),
                               ctx.RealVal("Tolerance3d", global::Kerf.Engine.Foundation.Precision.Confusion() /* -1 */),
                               new ProgressRange(),
                               msg);

        if (msg is not null)
        {
            // WARNING: not FULL update of context yet!
            var reshape = new ShapeBuild_ReShape();
            ctx.RecordModification(reshape, msg);
        }
        return true;
    }

    private static bool settol(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        double val = 0;
        if (ctx.IntegerVal("Mode", 0) > 0 && ctx.GetReal("Value", ref val))
        {
            double rat = ctx.RealVal("Ratio", 1.0);
            if (rat >= 1)
            {
                var SFST = new ShapeFix_ShapeTolerance();
                SFST.LimitTolerance(ctx.InternalResult(), val / rat, val * rat);
            }
        }

        BRepLib.UpdateTolerances(ctx.InternalResult(), true);

        double reg = 0;
        if (ctx.GetReal("Regularity", ref reg))
        {
            BRepLib.EncodeRegularity(ctx.InternalResult(), reg);
        }

        // WARNING: no update of context yet!
        return true;
    }

    private static bool splitangle(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        var SDA = new ShapeUpgrade_ShapeDivideAngle(ctx.RealVal("Angle", 2 * Math.PI), ctx.InternalResult());
        SDA.SetMaxTolerance(ctx.RealVal("MaxTolerance", 1.0));
        SDA.SetMsgRegistrator(msg);

        if (!SDA.Perform() && SDA.Status(ShapeExtend_FAIL))
        {
            // OCCT_DEBUG: "ShapeDivideAngle failed", left out
            return false;
        }

        ctx.RecordModification(SDA.GetContext()!, msg);
        ctx.SetResult(SDA.InternalResult());
        return true;
    }

    private static bool bsplinerestriction(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        bool ModeSurf = ctx.BooleanVal("SurfaceMode", true);
        bool ModeC3d = ctx.BooleanVal("Curve3dMode", true);
        bool ModeC2d = ctx.BooleanVal("Curve2dMode", true);

        double aTol3d = ctx.RealVal("Tolerance3d", 0.01);
        double aTol2d = ctx.RealVal("Tolerance2d", 1e-06);

        Continuity aCont3d = ctx.ContinuityVal("Continuity3d", C1);
        Continuity aCont2d = ctx.ContinuityVal("Continuity2d", C2);

        int aMaxDeg = ctx.IntegerVal("RequiredDegree", 9);
        int aMaxSeg = ctx.IntegerVal("RequiredNbSegments", 10000);

        bool ModeDeg = ctx.BooleanVal("PreferDegree", true);
        bool Rational = ctx.BooleanVal("RationalToPolynomial", false);

        var aParameters = new ShapeCustom_RestrictionParameters();
        ctx.GetInteger("MaxDegree", ref aParameters.GMaxDegree());
        ctx.GetInteger("MaxNbSegments", ref aParameters.GMaxSeg());
        ctx.GetBoolean("OffsetSurfaceMode", ref aParameters.ConvertOffsetSurf());
        ctx.GetBoolean("OffsetCurve3dMode", ref aParameters.ConvertOffsetCurv3d());
        ctx.GetBoolean("OffsetCurve2dMode", ref aParameters.ConvertOffsetCurv2d());
        ctx.GetBoolean("LinearExtrusionMode", ref aParameters.ConvertExtrusionSurf());
        ctx.GetBoolean("RevolutionMode", ref aParameters.ConvertRevolutionSurf());
        ctx.GetBoolean("SegmentSurfaceMode", ref aParameters.SegmentSurfaceMode());
        ctx.GetBoolean("ConvCurve3dMode", ref aParameters.ConvertCurve3d());
        ctx.GetBoolean("ConvCurve2dMode", ref aParameters.ConvertCurve2d());
        ctx.GetBoolean("BezierMode", ref aParameters.ConvertBezierSurf());
        // modes to convert elementary surfaces
        ctx.GetBoolean("PlaneMode", ref aParameters.ConvertPlane());
        // ctx->GetBoolean ("ElementarySurfMode", aParameters->ConvertElementarySurf());
        ctx.GetBoolean("ConicalSurfMode", ref aParameters.ConvertConicalSurf());
        ctx.GetBoolean("CylindricalSurfMode", ref aParameters.ConvertCylindricalSurf());
        ctx.GetBoolean("ToroidalSurfMode", ref aParameters.ConvertToroidalSurf());
        ctx.GetBoolean("SphericalSurfMode", ref aParameters.ConvertSphericalSurf());

        var LD = new ShapeCustom_BSplineRestriction(ModeSurf,
                                                    ModeC3d,
                                                    ModeC2d,
                                                    aTol3d,
                                                    aTol2d,
                                                    aCont3d,
                                                    aCont2d,
                                                    aMaxDeg,
                                                    aMaxSeg,
                                                    ModeDeg,
                                                    Rational,
                                                    aParameters);
        LD.SetMsgRegistrator(msg);
        return ApplyModification(ctx, LD, msg);
    }

    private static bool torevol(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        var CR = new ShapeCustom_ConvertToRevolution();
        CR.SetMsgRegistrator(msg);
        return ApplyModification(ctx, CR, msg);
    }

    private static bool swepttoelem(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        var SE = new ShapeCustom_SweptToElementary();
        SE.SetMsgRegistrator(msg);
        return ApplyModification(ctx, SE, msg);
    }

    private static bool shapetobezier(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        bool ModeC3d = ctx.BooleanVal("Curve3dMode", false);
        bool ModeC2d = ctx.BooleanVal("Curve2dMode", false);
        bool ModeSurf = ctx.BooleanVal("SurfaceMode", false);
        bool ModeLine3d = ctx.BooleanVal("Line3dMode", true);
        bool ModeCircle3d = ctx.BooleanVal("Circle3dMode", true);
        bool ModeConic3d = ctx.BooleanVal("Conic3dMode", true);
        bool SegmentMode = ctx.BooleanVal("SegmentSurfaceMode", true);
        bool PlaneMode = ctx.BooleanVal("PlaneMode", true);
        bool RevolutionMode = ctx.BooleanVal("RevolutionMode", true);
        bool ExtrusionMode = ctx.BooleanVal("ExtrusionMode", true);
        bool BSplineMode = ctx.BooleanVal("BSplineMode", true);

        var SCB = new ShapeUpgrade_ShapeConvertToBezier(ctx.InternalResult());
        SCB.SetMsgRegistrator(msg);
        SCB.SetSurfaceSegmentMode(SegmentMode);
        SCB.SetSurfaceConversion(ModeSurf);
        SCB.Set2dConversion(ModeC2d);
        SCB.Set3dConversion(ModeC3d);
        if (ModeC3d)
        {
            SCB.Set3dLineConversion(ModeLine3d);
            SCB.Set3dCircleConversion(ModeCircle3d);
            SCB.Set3dConicConversion(ModeConic3d);
        }
        if (ModeSurf)
        {
            SCB.SetPlaneMode(PlaneMode);
            SCB.SetRevolutionMode(RevolutionMode);
            SCB.SetExtrusionMode(ExtrusionMode);
            SCB.SetBSplineMode(BSplineMode);
        }

        double maxTol = 0, minTol = 0;
        if (ctx.GetReal("MaxTolerance", ref maxTol))
        {
            SCB.SetMaxTolerance(maxTol);
        }
        if (ctx.GetReal("MinCurveLength", ref minTol))
        {
            SCB.SetMinTolerance(minTol);
        }

        bool EdgeMode = false;
        if (ctx.GetBoolean("EdgeMode", ref EdgeMode))
        {
            // OCCT: the bool converts to EdgeMode's int (0 or 1)
            SCB.SetEdgeMode(EdgeMode ? 1 : 0);
        }

        if (!SCB.Perform() && SCB.Status(ShapeExtend_FAIL))
        {
            // OCCT_DEBUG: "Shape::ShapeConvertToBezier failed", left out
            return false;
        }

        ctx.RecordModification(SCB.GetContext()!, msg);
        ctx.SetResult(SCB.InternalResult());
        return true;
    }

    private static bool converttobspline(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        bool extrMode = ctx.BooleanVal("LinearExtrusionMode", true);
        bool revolMode = ctx.BooleanVal("RevolutionMode", true);
        bool offsetMode = ctx.BooleanVal("OffsetMode", true);

        var CBspl = new ShapeCustom_ConvertToBSpline();
        CBspl.SetExtrusionMode(extrMode);
        CBspl.SetRevolutionMode(revolMode);
        CBspl.SetOffsetMode(offsetMode);
        CBspl.SetMsgRegistrator(msg);

        return ApplyModification(ctx, CBspl, msg);
    }

    private static bool splitcontinuity(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        double aTol = ctx.RealVal("Tolerance3d", 1.0e-7);
        double aTol2D = ctx.RealVal("Tolerance2d", 1.0e-9);
        Continuity aCrvCont = ctx.ContinuityVal("CurveContinuity", C1);
        Continuity aSrfCont = ctx.ContinuityVal("SurfaceContinuity", C1);
        Continuity aCrv2dCont = ctx.ContinuityVal("Curve2dContinuity", C1);
        var tool = new ShapeUpgrade_ShapeDivideContinuity(ctx.InternalResult());
        tool.SetBoundaryCriterion(aCrvCont);
        tool.SetSurfaceCriterion(aSrfCont);
        tool.SetPCurveCriterion(aCrv2dCont);
        tool.SetTolerance(aTol);
        tool.SetTolerance2d(aTol2D);

        tool.SetMsgRegistrator(msg);

        double maxTol = 0;
        if (ctx.GetReal("MaxTolerance", ref maxTol))
        {
            tool.SetMaxTolerance(maxTol);
        }

        if (!tool.Perform() && tool.Status(ShapeExtend_FAIL))
        {
            // OCCT_DEBUG: "SplitContinuity failed", left out
            return false;
        }

        ctx.RecordModification(tool.GetContext()!, msg);
        ctx.SetResult(tool.InternalResult());
        return true;
    }

    private static bool splitclosedfaces(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        var tool = new ShapeUpgrade_ShapeDivideClosed(ctx.InternalResult());
        tool.SetMsgRegistrator(msg);

        double closeTol = 0;
        if (ctx.GetReal("CloseTolerance", ref closeTol))
        {
            tool.SetPrecision(closeTol);
        }

        double maxTol = 0;
        if (ctx.GetReal("MaxTolerance", ref maxTol))
        {
            tool.SetMaxTolerance(maxTol);
        }

        int num = ctx.IntegerVal("NbSplitPoints", 1);
        bool hasSeg = true;
        ctx.GetBoolean("SegmentSurfaceMode", ref hasSeg);

        tool.SetNbSplitPoints(num);
        tool.SetSurfaceSegmentMode(hasSeg);
        if (!tool.Perform() && tool.Status(ShapeExtend_FAIL))
        {
            // OCCT_DEBUG: "Splitting of closed faces failed", left out
            return false;
        }

        ctx.RecordModification(tool.GetContext()!, msg);
        ctx.SetResult(tool.InternalResult());
        return true;
    }

    private static bool fixfacesize(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        var reshape = new ShapeBuild_ReShape();
        var FSC = new ShapeFix_FixSmallFace();
        FSC.SetContext(reshape);
        FSC.Init(ctx.InternalResult());
        FSC.SetMsgRegistrator(msg);

        double aTol = 0;
        if (ctx.GetReal("Tolerance", ref aTol))
        {
            FSC.SetPrecision(aTol);
        }

        FSC.Perform();
        TopoDS_Shape newsh = FSC.InternalShape();

        if (newsh != ctx.InternalResult())
        {
            ctx.RecordModification(reshape, msg);
            ctx.SetResult(newsh);
        }

        return true;
    }

    private static bool fixwgaps(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        double aTol3d = ctx.RealVal("Tolerance3d", global::Kerf.Engine.Foundation.Precision.Confusion());

        var reshape = new ShapeBuild_ReShape();
        var sfwf = new ShapeFix_Wireframe(ctx.InternalResult());
        sfwf.SetMsgRegistrator(msg);
        sfwf.SetContext(reshape);
        sfwf.SetPrecision(aTol3d);
        sfwf.FixWireGaps();
        TopoDS_Shape result = sfwf.InternalShape();

        if (result != ctx.InternalResult())
        {
            ctx.RecordModification(reshape, msg);
            ctx.SetResult(result);
        }
        return true;
    }

    private static bool dropsmallsolids(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        var FSS = new ShapeFix_FixSmallSolid();
        FSS.SetMsgRegistrator(msg);

        double aThreshold = 0;
        int aMode = 0;
        if (ctx.GetInteger("FixMode", ref aMode))
        {
            FSS.SetFixMode(aMode);
        }
        if (ctx.GetReal("VolumeThreshold", ref aThreshold))
        {
            FSS.SetVolumeThreshold(aThreshold);
        }
        if (ctx.GetReal("WidthFactorThreshold", ref aThreshold))
        {
            FSS.SetWidthFactorThreshold(aThreshold);
        }

        bool aMerge = false;
        ctx.GetBoolean("MergeSolids", ref aMerge);

        var aReShape = new ShapeBuild_ReShape();

        TopoDS_Shape aResult;
        if (aMerge)
        {
            aResult = FSS.InternalMerge(ctx.InternalResult(), aReShape);
        }
        else
        {
            aResult = FSS.InternalRemove(ctx.InternalResult(), aReShape);
        }

        if (aResult != ctx.InternalResult())
        {
            ctx.RecordModification(aReShape, msg);
            ctx.SetResult(aResult);
        }

        return true;
    }

    // OCCT: dropsmalledges, an older version written for another framework, is commented out

    private static bool mergesmalledges(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        double aTol3d = ctx.RealVal("Tolerance3d", global::Kerf.Engine.Foundation.Precision.Confusion());

        var reshape = new ShapeBuild_ReShape();
        var ShapeFixWireframe = new ShapeFix_Wireframe(ctx.InternalResult());
        ShapeFixWireframe.SetContext(reshape);
        ShapeFixWireframe.SetPrecision(aTol3d);
        ShapeFixWireframe.SetMsgRegistrator(msg);

        if (ShapeFixWireframe.FixSmallEdges())
        {
            ctx.RecordModification(reshape, msg);
        }
        return true;
    }

    private static bool fixshape(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        var sfs = new ShapeFix_Shape();
        ShapeFix_Face sff = sfs.FixFaceTool();
        ShapeFix_Wire sfw = sfs.FixWireTool();
        sfs.SetMsgRegistrator(msg);

        sfs.SetPrecision(ctx.RealVal("Tolerance3d", global::Kerf.Engine.Foundation.Precision.Confusion()));
        sfs.SetMinTolerance(ctx.RealVal("MinTolerance3d", global::Kerf.Engine.Foundation.Precision.Confusion()));
        sfs.SetMaxTolerance(ctx.RealVal("MaxTolerance3d", global::Kerf.Engine.Foundation.Precision.Confusion()));

        sfs.FixFreeShellMode() = ctx.IntegerVal("FixFreeShellMode", -1);
        sfs.FixFreeFaceMode() = ctx.IntegerVal("FixFreeFaceMode", -1);
        sfs.FixFreeWireMode() = ctx.IntegerVal("FixFreeWireMode", -1);
        sfs.FixSameParameterMode() = ctx.IntegerVal("FixSameParameterMode", -1);
        sfs.FixSolidMode() = ctx.IntegerVal("FixSolidMode", -1);
        sfs.FixVertexPositionMode() = ctx.IntegerVal("FixVertexPositionMode", 0);
        sfs.FixVertexTolMode() = ctx.IntegerVal("FixVertexToleranceMode", -1);

        sfs.FixSolidTool().FixShellMode() = ctx.IntegerVal("FixShellMode", -1);
        sfs.FixSolidTool().FixShellOrientationMode() = ctx.IntegerVal("FixShellOrientationMode", -1);
        sfs.FixSolidTool().CreateOpenSolidMode() = ctx.BooleanVal("CreateOpenSolidMode", true);

        sfs.FixShellTool().FixFaceMode() = ctx.IntegerVal("FixFaceMode", -1);
        sfs.FixShellTool().SetNonManifoldFlag(ctx.IsNonManifold());
        sfs.FixShellTool().FixOrientationMode() = ctx.IntegerVal("FixFaceOrientationMode", -1);

        // parameters for ShapeFix_Face
        sff.FixWireMode() = ctx.IntegerVal("FixWireMode", -1);
        sff.FixOrientationMode() = ctx.IntegerVal("FixOrientationMode", -1);
        sff.FixAddNaturalBoundMode() = ctx.IntegerVal("FixAddNaturalBoundMode", -1);
        sff.FixMissingSeamMode() = ctx.IntegerVal("FixMissingSeamMode", -1);
        sff.FixSmallAreaWireMode() = ctx.IntegerVal("FixSmallAreaWireMode", -1);
        sff.RemoveSmallAreaFaceMode() = ctx.IntegerVal("RemoveSmallAreaFaceMode", -1);
        sff.FixIntersectingWiresMode() = ctx.IntegerVal("FixIntersectingWiresMode", -1);
        sff.FixLoopWiresMode() = ctx.IntegerVal("FixLoopWiresMode", -1);
        sff.FixSplitFaceMode() = ctx.IntegerVal("FixSplitFaceMode", -1);

        // parameters for ShapeFix_Wire
        sfw.ModifyTopologyMode() = ctx.BooleanVal("ModifyTopologyMode", false);
        sfw.ModifyGeometryMode() = ctx.BooleanVal("ModifyGeometryMode", true);
        sfw.ClosedWireMode() = ctx.BooleanVal("ClosedWireMode", true);
        sfw.PreferencePCurveMode() = ctx.BooleanVal("PreferencePCurveMode", true);
        sfw.FixReorderMode() = ctx.IntegerVal("FixReorderMode", -1);
        sfw.FixSmallMode() = ctx.IntegerVal("FixSmallMode", -1);
        sfw.FixConnectedMode() = ctx.IntegerVal("FixConnectedMode", -1);
        sfw.FixEdgeCurvesMode() = ctx.IntegerVal("FixEdgeCurvesMode", -1);
        sfw.FixDegeneratedMode() = ctx.IntegerVal("FixDegeneratedMode", -1);
        sfw.FixLackingMode() = ctx.IntegerVal("FixLackingMode", -1);
        sfw.FixSelfIntersectionMode() = ctx.IntegerVal("FixSelfIntersectionMode", -1);
        sfw.ModifyRemoveLoopMode() = ctx.IntegerVal("RemoveLoopMode", -1);
        sfw.FixReversed2dMode() = ctx.IntegerVal("FixReversed2dMode", -1);
        sfw.FixRemovePCurveMode() = ctx.IntegerVal("FixRemovePCurveMode", -1);
        sfw.FixRemoveCurve3dMode() = ctx.IntegerVal("FixRemoveCurve3dMode", -1);
        sfw.FixAddPCurveMode() = ctx.IntegerVal("FixAddPCurveMode", -1);
        sfw.FixAddCurve3dMode() = ctx.IntegerVal("FixAddCurve3dMode", -1);
        sfw.FixShiftedMode() = ctx.IntegerVal("FixShiftedMode", -1);
        sfw.FixSeamMode() = ctx.IntegerVal("FixSeamMode", -1);
        sfw.FixSameParameterMode() = ctx.IntegerVal("FixEdgeSameParameterMode", -1);
        sfw.FixNotchedEdgesMode() = ctx.IntegerVal("FixNotchedEdgesMode", -1);
        sfw.FixTailMode() = ctx.IntegerVal("FixTailMode", 0);
        sfw.SetMaxTailAngle(ctx.RealVal("MaxTailAngle", 0) * (Math.PI / 180));
        sfw.SetMaxTailWidth(ctx.RealVal("MaxTailWidth", -1));
        sfw.FixSelfIntersectingEdgeMode() = ctx.IntegerVal("FixSelfIntersectingEdgeMode", -1);
        sfw.FixIntersectingEdgesMode() = ctx.IntegerVal("FixIntersectingEdgesMode", -1);
        sfw.FixNonAdjacentIntersectingEdgesMode() = ctx.IntegerVal("FixNonAdjacentIntersectingEdgesMode", -1);
        using var aPS = new ProgressScope(theProgress, null, 2);
        if (sfw.FixTailMode() == 1)
        {
            sfw.FixTailMode() = 0;
            sfs.Init(ctx.InternalResult());
            sfs.Perform(aPS.Next());
            sfw.FixTailMode() = 1;
            if (aPS.UserBreak())
            {
                return false;
            }

            TopoDS_Shape result0 = sfs.InternalShape();
            if (result0 != ctx.InternalResult() || (msg is not null && !msg.MapShape().IsEmpty()))
            {
                ctx.RecordModification(sfs.Context()!, msg);
                ctx.SetResult(result0);
            }
        }

        sfs.Init(ctx.InternalResult());
        sfs.Perform(aPS.Next());
        if (aPS.UserBreak())
        {
            return false;
        }

        TopoDS_Shape result = sfs.InternalShape();
        if ((result != ctx.InternalResult()) || (msg is not null && !msg.MapShape().IsEmpty()))
        {
            ctx.RecordModification(sfs.Context()!, msg);
            ctx.SetResult(result);
        }
        return true;
    }

    private static bool spltclosededges(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        int nbSplits = ctx.IntegerVal("NbSplitPoints", 1);

        var tool = new ShapeUpgrade_ShapeDivideClosedEdges(ctx.InternalResult());
        tool.SetNbSplitPoints(nbSplits);
        tool.SetMsgRegistrator(msg);

        if (!tool.Perform() && tool.Status(ShapeExtend_FAIL))
        {
            // OCCT_DEBUG: "Splitting of closed edges failed", left out
            return false;
        }

        ctx.RecordModification(tool.GetContext()!, msg);
        ctx.SetResult(tool.InternalResult());
        return true;
    }

    // Two wires have common vertex - this case is valid in BRep model
    // and isn't valid in STEP => before writing into STEP it is necessary
    // to split this vertex (each wire must has one vertex)
    private static bool splitcommonvertex(ShapeProcess_Context context, ProgressRange theProgress)
    {
        if (context is not ShapeProcess_ShapeContext ctx)
        {
            return false;
        }

        ShapeExtend_MsgRegistrator? msg = NewMessages(ctx);

        var reshape = new ShapeBuild_ReShape();
        var SCV = new ShapeFix_SplitCommonVertex();
        SCV.SetContext(reshape);
        SCV.Init(ctx.InternalResult());

        SCV.SetMsgRegistrator(msg);

        SCV.Perform();
        TopoDS_Shape newsh = SCV.InternalShape();

        if (newsh != ctx.InternalResult())
        {
            ctx.RecordModification(reshape, msg);
            ctx.SetResult(newsh);
        }

        return true;
    }

    /// <summary>Registers all the operators (the first call in the process only); loads Shape Healing's messages.</summary>
    public static void Init()
    {
        lock (theInitLock)
        {
            if (done)
            {
                return;
            }
            done = true;

            ShapeExtend.Init();

            ShapeProcess.RegisterOperator("DirectFaces", new ShapeProcess_UOperator(directfaces));
            ShapeProcess.RegisterOperator("SameParameter", new ShapeProcess_UOperator(sameparam));
            ShapeProcess.RegisterOperator("SetTolerance", new ShapeProcess_UOperator(settol));
            ShapeProcess.RegisterOperator("SplitAngle", new ShapeProcess_UOperator(splitangle));
            ShapeProcess.RegisterOperator("BSplineRestriction", new ShapeProcess_UOperator(bsplinerestriction));
            ShapeProcess.RegisterOperator("ElementaryToRevolution", new ShapeProcess_UOperator(torevol));
            ShapeProcess.RegisterOperator("SweptToElementary", new ShapeProcess_UOperator(swepttoelem));
            ShapeProcess.RegisterOperator("SurfaceToBSpline", new ShapeProcess_UOperator(converttobspline));
            ShapeProcess.RegisterOperator("ToBezier", new ShapeProcess_UOperator(shapetobezier));
            ShapeProcess.RegisterOperator("SplitContinuity", new ShapeProcess_UOperator(splitcontinuity));
            ShapeProcess.RegisterOperator("SplitClosedFaces", new ShapeProcess_UOperator(splitclosedfaces));
            ShapeProcess.RegisterOperator("FixWireGaps", new ShapeProcess_UOperator(fixwgaps));
            ShapeProcess.RegisterOperator("FixFaceSize", new ShapeProcess_UOperator(fixfacesize));
            ShapeProcess.RegisterOperator("DropSmallSolids", new ShapeProcess_UOperator(dropsmallsolids));
            ShapeProcess.RegisterOperator("DropSmallEdges", new ShapeProcess_UOperator(mergesmalledges));
            ShapeProcess.RegisterOperator("FixShape", new ShapeProcess_UOperator(fixshape));
            ShapeProcess.RegisterOperator("SplitClosedEdges", new ShapeProcess_UOperator(spltclosededges));
            ShapeProcess.RegisterOperator("SplitCommonVertex", new ShapeProcess_UOperator(splitcommonvertex));
        }
    }
}
