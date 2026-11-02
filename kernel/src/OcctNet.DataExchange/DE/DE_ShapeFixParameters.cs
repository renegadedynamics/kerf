// Port of: src/DataExchange/TKDE/DE/DE_ShapeFixParameters.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Struct for shape healing parameters storage.</summary>
/// <remarks>
/// <para>
/// OCCT's struct with OCCT's public fields (as IMeshTools_Parameters, decision 489): a caller writes
/// <c>new DE_ShapeFixParameters { Tolerance3d = 1e-4, FixFaceMode = DE_ShapeFixParameters.FixMode.Fix }</c>. It is a
/// value: changing a copy changes nothing shared; it is equatable field by field.
/// </para>
/// <para>
/// <c>new DE_ShapeFixParameters()</c> has OCCT's defaults. <c>default(DE_ShapeFixParameters)</c> doesn't (every field
/// zero, the modes <see cref="FixMode.NotFix"/>): ported code writes <c>new DE_ShapeFixParameters()</c>. The rest of
/// TKDE (the DE_Wrapper framework) isn't ported (decision 1171); this struct is, since XSAlgo_ShapeProcessor and the
/// actors take it.
/// </para>
/// </remarks>
public record struct DE_ShapeFixParameters
{
    /// <summary>Enum, classifying a type of value for parameters.</summary>
    public enum FixMode
    {
        FixOrNot = -1, // Procedure will be executed or not (depending on the situation)
        NotFix = 0,    // Procedure will be executed
        Fix = 1,       // Procedure will be executed anyway
    }

    /// <summary>OCCT's defaults.</summary>
    public DE_ShapeFixParameters()
    {
    }

    public double Tolerance3d = 1.0e-6;
    public double MaxTolerance3d = 1.0;
    public double MinTolerance3d = 1.0e-7;
    public TopAbs_ShapeEnum DetalizationLevel = TopAbs_VERTEX;
    public bool NonManifold = false;
    public FixMode FixFreeShellMode = FixMode.FixOrNot;
    public FixMode FixFreeFaceMode = FixMode.FixOrNot;
    public FixMode FixFreeWireMode = FixMode.FixOrNot;
    public FixMode FixSameParameterMode = FixMode.FixOrNot;
    public FixMode FixSolidMode = FixMode.FixOrNot;
    public FixMode FixShellOrientationMode = FixMode.FixOrNot;
    public FixMode CreateOpenSolidMode = FixMode.NotFix;
    public FixMode FixShellMode = FixMode.FixOrNot;
    public FixMode FixFaceOrientationMode = FixMode.FixOrNot;
    public FixMode FixFaceMode = FixMode.FixOrNot;
    public FixMode FixWireMode = FixMode.FixOrNot;
    public FixMode FixOrientationMode = FixMode.FixOrNot;
    public FixMode FixAddNaturalBoundMode = FixMode.FixOrNot;
    public FixMode FixMissingSeamMode = FixMode.FixOrNot;
    public FixMode FixSmallAreaWireMode = FixMode.FixOrNot;
    public FixMode RemoveSmallAreaFaceMode = FixMode.FixOrNot;
    public FixMode FixIntersectingWiresMode = FixMode.FixOrNot;
    public FixMode FixLoopWiresMode = FixMode.FixOrNot;
    public FixMode FixSplitFaceMode = FixMode.FixOrNot;
    public FixMode AutoCorrectPrecisionMode = FixMode.Fix;
    public FixMode ModifyTopologyMode = FixMode.NotFix;
    public FixMode ModifyGeometryMode = FixMode.Fix;
    public FixMode ClosedWireMode = FixMode.Fix;
    public FixMode PreferencePCurveMode = FixMode.Fix;
    public FixMode FixReorderMode = FixMode.FixOrNot;
    public FixMode FixSmallMode = FixMode.FixOrNot;
    public FixMode FixConnectedMode = FixMode.FixOrNot;
    public FixMode FixEdgeCurvesMode = FixMode.FixOrNot;
    public FixMode FixDegeneratedMode = FixMode.FixOrNot;
    public FixMode FixLackingMode = FixMode.FixOrNot;
    public FixMode FixSelfIntersectionMode = FixMode.FixOrNot;
    public FixMode RemoveLoopMode = FixMode.FixOrNot;
    public FixMode FixReversed2dMode = FixMode.FixOrNot;
    public FixMode FixRemovePCurveMode = FixMode.FixOrNot;
    public FixMode FixRemoveCurve3dMode = FixMode.FixOrNot;
    public FixMode FixAddPCurveMode = FixMode.FixOrNot;
    public FixMode FixAddCurve3dMode = FixMode.FixOrNot;
    public FixMode FixSeamMode = FixMode.FixOrNot;
    public FixMode FixShiftedMode = FixMode.FixOrNot;
    public FixMode FixEdgeSameParameterMode = FixMode.NotFix;
    public FixMode FixNotchedEdgesMode = FixMode.FixOrNot;
    public FixMode FixTailMode = FixMode.NotFix;
    public FixMode MaxTailAngle = FixMode.NotFix;
    public FixMode MaxTailWidth = FixMode.FixOrNot;
    public FixMode FixSelfIntersectingEdgeMode = FixMode.FixOrNot;
    public FixMode FixIntersectingEdgesMode = FixMode.FixOrNot;
    public FixMode FixNonAdjacentIntersectingEdgesMode = FixMode.FixOrNot;
    public FixMode FixVertexPositionMode = FixMode.NotFix;
    public FixMode FixVertexToleranceMode = FixMode.FixOrNot;
}
