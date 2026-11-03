// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_OperLibrary.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_OperLibrary.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Custom;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Collections.Frozen;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Process;

/// <summary>
/// The library of Shape Healing's operators: DirectFaces, SameParameter, SetTolerance, SplitAngle,
/// BSplineRestriction, ElementaryToRevolution, SweptToElementary, SurfaceToBSpline, ToBezier, SplitContinuity,
/// SplitClosedFaces, FixWireGaps, FixFaceSize, DropSmallSolids, DropSmallEdges, FixShape, SplitClosedEdges and
/// SplitCommonVertex.
/// </summary>
/// <remarks>
/// <see cref="Init"/> registers them once per process (OCCT's static <c>done</c> flag, plan 2.3), as
/// <see cref="ShapeProcess"/>'s default operators, immutable once published (a one-time initialisation, unit 10I). The operators work on the context's
/// <see cref="ShapeContext.InternalResult"/>, in the context's working overlay while they run
/// (decision 1022), and read the tools' unfrozen results (decisions 973, 999); the context freezes at its own
/// publication points.
/// </remarks>
public static class OperLibrary
{
    private static readonly Lazy<bool> Registered = new(RegisterAll);

    /// <summary>
    /// Applies a BRepTools_Modification to a shape, taking the sharing of the components of compounds into account.
    /// With <paramref name="mutableInput"/>, the modifier may update the input's vertices in place; on a shape holding
    /// frozen shapes outside a working overlay the port then runs it in its immutable mode (decision 1022). The result is
    /// frozen.
    /// </summary>
    public static Shape ApplyModifier(in Shape s,
                                             ShapeContext context,
                                             Modification m,
                                             NCollection_DataMap<Shape, Shape> map,
                                             MsgRegistrator? msg = null,
                                             bool mutableInput = false)
    {
        Shape result = InternalApplyModifier(s, context, m, map, msg, mutableInput && IsWritable(s));
        result.TShape()?.FreezeGraph();
        return result;
    }

    /// <summary><see cref="ApplyModifier"/> without freezing the result, for the operators.</summary>
    internal static Shape InternalApplyModifier(in Shape s,
                                                       ShapeContext context,
                                                       Modification m,
                                                       NCollection_DataMap<Shape, Shape> map,
                                                       MsgRegistrator? msg,
                                                       bool mutableInput)
    {
        // protect against INTERNAL/EXTERNAL shapes
        Shape sf = s.Oriented(Forward);

        // Process COMPOUNDs separately in order to handle sharing in assemblies
        if (sf.ShapeType() == ShapeType.Compound)
        {
            bool locModified = false;
            var c = new Compound();
            var b = new BrepBuilder();
            b.MakeCompound(ref c);
            for (var it = new Iterator(sf); it.More(); it.Next())
            {
                Shape shape = it.Value();
                Placement l = shape.Location(), nullLoc = new();
                shape.Location(nullLoc);
                Shape res;
                if (map.IsBound(shape))
                {
                    res = map.Find(shape).Oriented(shape.Orientation());
                }
                else
                {
                    res = InternalApplyModifier(shape, context, m, map, null, mutableInput);
                    map.Bind(shape, res);
                }
                if (!res.IsSame(shape))
                {
                    locModified = true;
                }
                res.Location(l, false);
                b.Add(c, res);
            }
            if (!locModified)
            {
                return s;
            }

            map.Bind(sf, c);
            return c.Oriented(s.Orientation());
        }

        // Modify the shape
        var md = new Modifier(sf);
        md.SetMutableInput(mutableInput);
        md.Perform(m);
        context.RecordModification(sf, md, msg);
        return md.InternalModifiedShape(sf).Oriented(s.Orientation());
    }

    // activate message mechanism if it is supported by context
    private static MsgRegistrator? NewMessages(ShapeContext ctx)
        => ctx.Messages() is not null ? new MsgRegistrator() : null;

    // True if the modifier may write into the shape: inside a working overlay (the context's, while operators run),
    // which takes the writes, or when it holds no frozen shape (Kerf units 10H4, 10H6).
    private static bool IsWritable(in Shape s) => WorkingOverlay.Writing is not null || WorkingOverlay.IsWritableInPlace(s);

    // The modifiers run on the context's result in its working overlay, in the mutable-input mode as OCCT's (decision
    // 1022)
    private static bool ApplyModification(ShapeContext ctx, Modification modification,
                                          MsgRegistrator? msg)
    {
        var map = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        Shape work = ctx.InternalResult();
        Shape res = InternalApplyModifier(work, ctx, modification, map, msg, IsWritable(work));
        ctx.RecordModification(map, msg);
        ctx.SetResult(res);
        return true;
    }

    private static bool Directfaces(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        var dm = new DirectModification();
        dm.SetMsgRegistrator(msg);
        return ApplyModification(ctx, dm, msg);
    }

    private static bool Sameparam(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        ShapeFixFixer.SameParameter(ctx.InternalResult(),
                               ctx.BooleanVal("Force", false),
                               ctx.RealVal("Tolerance3d", global::Kerf.Engine.Foundation.Precision.Confusion() /* -1 */),
                               new ProgressRange(),
                               msg);

        if (msg is not null)
        {
            // WARNING: not FULL update of context yet!
            var reshape = new ReShapeBuilder();
            ctx.RecordModification(reshape, msg);
        }
        return true;
    }

    private static bool Settol(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        double val = 0;
        if (ctx.IntegerVal("Mode", 0) > 0 && ctx.GetReal("Value", ref val))
        {
            double rat = ctx.RealVal("Ratio", 1.0);
            if (rat >= 1)
            {
                var sfst = new ShapeToleranceFixer();
                sfst.LimitTolerance(ctx.InternalResult(), val / rat, val * rat);
            }
        }

        BrepLibrary.UpdateTolerances(ctx.InternalResult(), true);

        double reg = 0;
        if (ctx.GetReal("Regularity", ref reg))
        {
            BrepLibrary.EncodeRegularity(ctx.InternalResult(), reg);
        }

        // WARNING: no update of context yet!
        return true;
    }

    private static bool Splitangle(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        var sda = new ShapeDivideAngle(ctx.RealVal("Angle", 2 * Math.PI), ctx.InternalResult());
        sda.SetMaxTolerance(ctx.RealVal("MaxTolerance", 1.0));
        sda.SetMsgRegistrator(msg);

        if (!sda.Perform() && sda.Status(ExtendStatus.Fail))
        {
            // OCCT_DEBUG: "ShapeDivideAngle failed", left out
            return false;
        }

        ctx.RecordModification(sda.GetContext()!, msg);
        ctx.SetResult(sda.InternalResult());
        return true;
    }

    private static bool Bsplinerestriction(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        bool modeSurf = ctx.BooleanVal("SurfaceMode", true);
        bool modeC3d = ctx.BooleanVal("Curve3dMode", true);
        bool modeC2d = ctx.BooleanVal("Curve2dMode", true);

        double tol3d = ctx.RealVal("Tolerance3d", 0.01);
        double tol2d = ctx.RealVal("Tolerance2d", 1e-06);

        Continuity cont3d = ctx.ContinuityVal("Continuity3d", C1);
        Continuity cont2d = ctx.ContinuityVal("Continuity2d", C2);

        int maxDeg = ctx.IntegerVal("RequiredDegree", 9);
        int maxSeg = ctx.IntegerVal("RequiredNbSegments", 10000);

        bool modeDeg = ctx.BooleanVal("PreferDegree", true);
        bool rational = ctx.BooleanVal("RationalToPolynomial", false);

        var parameters = new RestrictionParameters();
        ctx.GetInteger("MaxDegree", ref parameters.GMaxDegree());
        ctx.GetInteger("MaxNbSegments", ref parameters.GMaxSeg());
        ctx.GetBoolean("OffsetSurfaceMode", ref parameters.ConvertOffsetSurf());
        ctx.GetBoolean("OffsetCurve3dMode", ref parameters.ConvertOffsetCurv3d());
        ctx.GetBoolean("OffsetCurve2dMode", ref parameters.ConvertOffsetCurv2d());
        ctx.GetBoolean("LinearExtrusionMode", ref parameters.ConvertExtrusionSurf());
        ctx.GetBoolean("RevolutionMode", ref parameters.ConvertRevolutionSurf());
        ctx.GetBoolean("SegmentSurfaceMode", ref parameters.SegmentSurfaceMode());
        ctx.GetBoolean("ConvCurve3dMode", ref parameters.ConvertCurve3d());
        ctx.GetBoolean("ConvCurve2dMode", ref parameters.ConvertCurve2d());
        ctx.GetBoolean("BezierMode", ref parameters.ConvertBezierSurf());
        // modes to convert elementary surfaces
        ctx.GetBoolean("PlaneMode", ref parameters.ConvertPlane());
        // ctx->GetBoolean ("ElementarySurfMode", aParameters->ConvertElementarySurf());
        ctx.GetBoolean("ConicalSurfMode", ref parameters.ConvertConicalSurf());
        ctx.GetBoolean("CylindricalSurfMode", ref parameters.ConvertCylindricalSurf());
        ctx.GetBoolean("ToroidalSurfMode", ref parameters.ConvertToroidalSurf());
        ctx.GetBoolean("SphericalSurfMode", ref parameters.ConvertSphericalSurf());

        var ld = new BSplineRestriction(modeSurf,
                                                    modeC3d,
                                                    modeC2d,
                                                    tol3d,
                                                    tol2d,
                                                    cont3d,
                                                    cont2d,
                                                    maxDeg,
                                                    maxSeg,
                                                    modeDeg,
                                                    rational,
                                                    parameters);
        ld.SetMsgRegistrator(msg);
        return ApplyModification(ctx, ld, msg);
    }

    private static bool Torevol(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        var cr = new ConvertToRevolution();
        cr.SetMsgRegistrator(msg);
        return ApplyModification(ctx, cr, msg);
    }

    private static bool Swepttoelem(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        var se = new SweptToElementary();
        se.SetMsgRegistrator(msg);
        return ApplyModification(ctx, se, msg);
    }

    private static bool Shapetobezier(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        bool modeC3d = ctx.BooleanVal("Curve3dMode", false);
        bool modeC2d = ctx.BooleanVal("Curve2dMode", false);
        bool modeSurf = ctx.BooleanVal("SurfaceMode", false);
        bool modeLine3d = ctx.BooleanVal("Line3dMode", true);
        bool modeCircle3d = ctx.BooleanVal("Circle3dMode", true);
        bool modeConic3d = ctx.BooleanVal("Conic3dMode", true);
        bool segmentMode = ctx.BooleanVal("SegmentSurfaceMode", true);
        bool planeMode = ctx.BooleanVal("PlaneMode", true);
        bool revolutionMode = ctx.BooleanVal("RevolutionMode", true);
        bool extrusionMode = ctx.BooleanVal("ExtrusionMode", true);
        bool bSplineMode = ctx.BooleanVal("BSplineMode", true);

        var scb = new ShapeConvertToBezier(ctx.InternalResult());
        scb.SetMsgRegistrator(msg);
        scb.SetSurfaceSegmentMode(segmentMode);
        scb.SetSurfaceConversion(modeSurf);
        scb.Set2dConversion(modeC2d);
        scb.Set3dConversion(modeC3d);
        if (modeC3d)
        {
            scb.Set3dLineConversion(modeLine3d);
            scb.Set3dCircleConversion(modeCircle3d);
            scb.Set3dConicConversion(modeConic3d);
        }
        if (modeSurf)
        {
            scb.SetPlaneMode(planeMode);
            scb.SetRevolutionMode(revolutionMode);
            scb.SetExtrusionMode(extrusionMode);
            scb.SetBSplineMode(bSplineMode);
        }

        double maxTol = 0, minTol = 0;
        if (ctx.GetReal("MaxTolerance", ref maxTol))
        {
            scb.SetMaxTolerance(maxTol);
        }
        if (ctx.GetReal("MinCurveLength", ref minTol))
        {
            scb.SetMinTolerance(minTol);
        }

        bool edgeMode = false;
        if (ctx.GetBoolean("EdgeMode", ref edgeMode))
        {
            // OCCT: the bool converts to EdgeMode's int (0 or 1)
            scb.SetEdgeMode(edgeMode ? 1 : 0);
        }

        if (!scb.Perform() && scb.Status(ExtendStatus.Fail))
        {
            // OCCT_DEBUG: "Shape::ShapeConvertToBezier failed", left out
            return false;
        }

        ctx.RecordModification(scb.GetContext()!, msg);
        ctx.SetResult(scb.InternalResult());
        return true;
    }

    private static bool Converttobspline(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        bool extrMode = ctx.BooleanVal("LinearExtrusionMode", true);
        bool revolMode = ctx.BooleanVal("RevolutionMode", true);
        bool offsetMode = ctx.BooleanVal("OffsetMode", true);

        var cBspl = new ConvertToBSpline();
        cBspl.SetExtrusionMode(extrMode);
        cBspl.SetRevolutionMode(revolMode);
        cBspl.SetOffsetMode(offsetMode);
        cBspl.SetMsgRegistrator(msg);

        return ApplyModification(ctx, cBspl, msg);
    }

    private static bool Splitcontinuity(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        double tol = ctx.RealVal("Tolerance3d", 1.0e-7);
        double tol2D = ctx.RealVal("Tolerance2d", 1.0e-9);
        Continuity crvCont = ctx.ContinuityVal("CurveContinuity", C1);
        Continuity srfCont = ctx.ContinuityVal("SurfaceContinuity", C1);
        Continuity crv2dCont = ctx.ContinuityVal("Curve2dContinuity", C1);
        var tool = new ShapeDivideContinuity(ctx.InternalResult());
        tool.SetBoundaryCriterion(crvCont);
        tool.SetSurfaceCriterion(srfCont);
        tool.SetPCurveCriterion(crv2dCont);
        tool.SetTolerance(tol);
        tool.SetTolerance2d(tol2D);

        tool.SetMsgRegistrator(msg);

        double maxTol = 0;
        if (ctx.GetReal("MaxTolerance", ref maxTol))
        {
            tool.SetMaxTolerance(maxTol);
        }

        if (!tool.Perform() && tool.Status(ExtendStatus.Fail))
        {
            // OCCT_DEBUG: "SplitContinuity failed", left out
            return false;
        }

        ctx.RecordModification(tool.GetContext()!, msg);
        ctx.SetResult(tool.InternalResult());
        return true;
    }

    private static bool Splitclosedfaces(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        var tool = new ShapeDivideClosed(ctx.InternalResult());
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
        if (!tool.Perform() && tool.Status(ExtendStatus.Fail))
        {
            // OCCT_DEBUG: "Splitting of closed faces failed", left out
            return false;
        }

        ctx.RecordModification(tool.GetContext()!, msg);
        ctx.SetResult(tool.InternalResult());
        return true;
    }

    private static bool Fixfacesize(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        var reshape = new ReShapeBuilder();
        var fsc = new FixSmallFaceFixer();
        fsc.SetContext(reshape);
        fsc.Init(ctx.InternalResult());
        fsc.SetMsgRegistrator(msg);

        double tol = 0;
        if (ctx.GetReal("Tolerance", ref tol))
        {
            fsc.SetPrecision(tol);
        }

        fsc.Perform();
        Shape newsh = fsc.InternalShape();

        if (newsh != ctx.InternalResult())
        {
            ctx.RecordModification(reshape, msg);
            ctx.SetResult(newsh);
        }

        return true;
    }

    private static bool Fixwgaps(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        double tol3d = ctx.RealVal("Tolerance3d", global::Kerf.Engine.Foundation.Precision.Confusion());

        var reshape = new ReShapeBuilder();
        var sfwf = new WireframeFixer(ctx.InternalResult());
        sfwf.SetMsgRegistrator(msg);
        sfwf.SetContext(reshape);
        sfwf.SetPrecision(tol3d);
        sfwf.FixWireGaps();
        Shape result = sfwf.InternalShape();

        if (result != ctx.InternalResult())
        {
            ctx.RecordModification(reshape, msg);
            ctx.SetResult(result);
        }
        return true;
    }

    private static bool Dropsmallsolids(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        var fss = new FixSmallSolidFixer();
        fss.SetMsgRegistrator(msg);

        double threshold = 0;
        int mode = 0;
        if (ctx.GetInteger("FixMode", ref mode))
        {
            fss.SetFixMode(mode);
        }
        if (ctx.GetReal("VolumeThreshold", ref threshold))
        {
            fss.SetVolumeThreshold(threshold);
        }
        if (ctx.GetReal("WidthFactorThreshold", ref threshold))
        {
            fss.SetWidthFactorThreshold(threshold);
        }

        bool merge = false;
        ctx.GetBoolean("MergeSolids", ref merge);

        var reShape = new ReShapeBuilder();

        Shape result;
        if (merge)
        {
            result = fss.InternalMerge(ctx.InternalResult(), reShape);
        }
        else
        {
            result = fss.InternalRemove(ctx.InternalResult(), reShape);
        }

        if (result != ctx.InternalResult())
        {
            ctx.RecordModification(reShape, msg);
            ctx.SetResult(result);
        }

        return true;
    }

    // OCCT: dropsmalledges, an older version written for another framework, is commented out

    private static bool Mergesmalledges(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        double tol3d = ctx.RealVal("Tolerance3d", global::Kerf.Engine.Foundation.Precision.Confusion());

        var reshape = new ReShapeBuilder();
        var shapeFixWireframe = new WireframeFixer(ctx.InternalResult());
        shapeFixWireframe.SetContext(reshape);
        shapeFixWireframe.SetPrecision(tol3d);
        shapeFixWireframe.SetMsgRegistrator(msg);

        if (shapeFixWireframe.FixSmallEdges())
        {
            ctx.RecordModification(reshape, msg);
        }
        return true;
    }

    private static bool Fixshape(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        var sfs = new ShapeFixer();
        FaceFixer sff = sfs.FixFaceTool();
        WireFixer sfw = sfs.FixWireTool();
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
        sfs.FixShellTool().SetNonManifoldFlag(ctx.IsNonManifold);
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
        using var ps = new ProgressScope(progress, null, 2);
        if (sfw.FixTailMode() == 1)
        {
            sfw.FixTailMode() = 0;
            sfs.Init(ctx.InternalResult());
            sfs.Perform(ps.Next());
            sfw.FixTailMode() = 1;
            if (ps.UserBreak())
            {
                return false;
            }

            Shape result0 = sfs.InternalShape();
            if (result0 != ctx.InternalResult() || (msg is not null && !msg.MapShape().IsEmpty()))
            {
                ctx.RecordModification(sfs.Context()!, msg);
                ctx.SetResult(result0);
            }
        }

        sfs.Init(ctx.InternalResult());
        sfs.Perform(ps.Next());
        if (ps.UserBreak())
        {
            return false;
        }

        Shape result = sfs.InternalShape();
        if ((result != ctx.InternalResult()) || (msg is not null && !msg.MapShape().IsEmpty()))
        {
            ctx.RecordModification(sfs.Context()!, msg);
            ctx.SetResult(result);
        }
        return true;
    }

    private static bool Spltclosededges(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        int nbSplits = ctx.IntegerVal("NbSplitPoints", 1);

        var tool = new ShapeDivideClosedEdges(ctx.InternalResult());
        tool.SetNbSplitPoints(nbSplits);
        tool.SetMsgRegistrator(msg);

        if (!tool.Perform() && tool.Status(ExtendStatus.Fail))
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
    private static bool Splitcommonvertex(ProcessContext context, ProgressRange progress)
    {
        if (context is not ShapeContext ctx)
        {
            return false;
        }

        MsgRegistrator? msg = NewMessages(ctx);

        var reshape = new ReShapeBuilder();
        var scv = new SplitCommonVertexFixer();
        scv.SetContext(reshape);
        scv.Init(ctx.InternalResult());

        scv.SetMsgRegistrator(msg);

        scv.Perform();
        Shape newsh = scv.InternalShape();

        if (newsh != ctx.InternalResult())
        {
            ctx.RecordModification(reshape, msg);
            ctx.SetResult(newsh);
        }

        return true;
    }

    /// <summary>Registers all the operators (the first call in the process only); loads Shape Healing's messages.</summary>
    public static void Init() => _ = Registered.Value;

    private static bool RegisterAll()
    {
        global::Kerf.Engine.Healing.Extend.ShapeExtend.Init();

        var operators = new Dictionary<string, Operator>(StringComparer.Ordinal)
        {
            ["DirectFaces"] = new UOperator(Directfaces),
            ["SameParameter"] = new UOperator(Sameparam),
            ["SetTolerance"] = new UOperator(Settol),
            ["SplitAngle"] = new UOperator(Splitangle),
            ["BSplineRestriction"] = new UOperator(Bsplinerestriction),
            ["ElementaryToRevolution"] = new UOperator(Torevol),
            ["SweptToElementary"] = new UOperator(Swepttoelem),
            ["SurfaceToBSpline"] = new UOperator(Converttobspline),
            ["ToBezier"] = new UOperator(Shapetobezier),
            ["SplitContinuity"] = new UOperator(Splitcontinuity),
            ["SplitClosedFaces"] = new UOperator(Splitclosedfaces),
            ["FixWireGaps"] = new UOperator(Fixwgaps),
            ["FixFaceSize"] = new UOperator(Fixfacesize),
            ["DropSmallSolids"] = new UOperator(Dropsmallsolids),
            ["DropSmallEdges"] = new UOperator(Mergesmalledges),
            ["FixShape"] = new UOperator(Fixshape),
            ["SplitClosedEdges"] = new UOperator(Spltclosededges),
            ["SplitCommonVertex"] = new UOperator(Splitcommonvertex),
        };
        global::Kerf.Engine.Healing.Process.ShapeProcess.SetDefaultOperators(operators.ToFrozenDictionary(StringComparer.Ordinal));
        return true;
    }
}
