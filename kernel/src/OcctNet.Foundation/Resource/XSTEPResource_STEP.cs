// Port of: resources/XSTEPResource/STEP
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// OCCT's resource file resources/XSTEPResource/STEP (ShapeProcess's sequences and parameters for STEP data
/// exchange), compiled in byte for byte; <see cref="Resource_Manager(string, bool)"/> reads it for the name
/// <c>STEP</c> (open question 26: OCCT's default resource files, embedded).
/// </summary>
internal static class XSTEPResource_STEP
{
    // Generated from resources/XSTEPResource/STEP, line for line (CR LF kept)
    internal const string Text =
        "\r\n" +
        "! Resource file for data exchange ...\r\n" +
        "!\r\n" +
        "! For meaning of parameters refer to the end of the file\r\n" +
        "\r\n" +
        "! ==============================================================================\r\n" +
        "!\r\n" +
        "\r\n" +
        "ToSTEP.exec.op : SplitCommonVertex,DirectFaces\r\n" +
        "\r\n" +
        "FromSTEP.exec.op : FixShape\r\n" +
        "\r\n" +
        "FromSTEP.FixShape.Tolerance3d\t\t\t: &Runtime.Tolerance\r\n" +
        "FromSTEP.FixShape.MaxTolerance3d\t\t: &Runtime.MaxTolerance\r\n" +
        "FromSTEP.FixShape.MinTolerance3d\t\t: 1.e-7\r\n" +
        "FromSTEP.FixShape.FixFreeShellMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixFreeFaceMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixFreeWireMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixSameParameterMode\t\t: -1\r\n" +
        "\r\n" +
        "FromSTEP.FixShape.FixSolidMode\t\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixShellOrientationMode\t: -1\r\n" +
        "FromSTEP.FixShape.CreateOpenSolidMode           :  0\r\n" +
        "\r\n" +
        "FromSTEP.FixShape.FixShellMode\t\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixFaceOrientationMode\t: -1\r\n" +
        "\r\n" +
        "FromSTEP.FixShape.FixFaceMode\t\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixWireMode\t\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixOrientationMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixAddNaturalBoundMode\t: -1\r\n" +
        "FromSTEP.FixShape.FixMissingSeamMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixSmallAreaWireMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.RemoveSmallAreaFaceMode\t: -1\r\n" +
        "FromSTEP.FixShape.FixIntersectingWiresMode\t: -1\r\n" +
        "FromSTEP.FixShape.FixLoopWiresMode              : -1\r\n" +
        "FromSTEP.FixShape.FixSplitFaceMode              : -1\r\n" +
        "FromSTEP.FixShape.AutoCorrectPrecisionMode      :  1\r\n" +
        "FromSTEP.FixShape.ModifyTopologyMode\t\t:  0\r\n" +
        "FromSTEP.FixShape.ModifyGeometryMode\t\t:  1\r\n" +
        "FromSTEP.FixShape.ClosedWireMode\t\t:  1\r\n" +
        "FromSTEP.FixShape.PreferencePCurveMode\t\t:  1\r\n" +
        "FromSTEP.FixShape.FixReorderMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixSmallMode\t\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixConnectedMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixEdgeCurvesMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixDegeneratedMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixLackingMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixSelfIntersectionMode\t: -1\r\n" +
        "FromSTEP.FixShape.RemoveLoopMode                : -1\r\n" +
        "FromSTEP.FixShape.FixReversed2dMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixRemovePCurveMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixRemoveCurve3dMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixAddPCurveMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixAddCurve3dMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixSeamMode\t\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixShiftedMode\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixEdgeSameParameterMode\t:  0\r\n" +
        "FromSTEP.FixShape.FixNotchedEdgesMode \t\t: -1\r\n" +
        "FromSTEP.FixShape.FixTailMode : 0\r\n" +
        "FromSTEP.FixShape.MaxTailAngle : 0\r\n" +
        "FromSTEP.FixShape.MaxTailWidth : -1\r\n" +
        "FromSTEP.FixShape.FixSelfIntersectingEdgeMode\t: -1\r\n" +
        "FromSTEP.FixShape.FixIntersectingEdgesMode\t: -1\r\n" +
        "FromSTEP.FixShape.FixNonAdjacentIntersectingEdgesMode : -1\r\n" +
        "\r\n" +
        "FromSTEP.FixShape.FixVertexPositionMode : 0\r\n" +
        "FromSTEP.FixShape.FixVertexToleranceMode : -1\r\n";
}
