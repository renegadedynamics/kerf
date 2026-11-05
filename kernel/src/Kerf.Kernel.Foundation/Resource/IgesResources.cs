// Port of: resources/XSTEPResource/IGES
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Foundation.Resources;

/// <summary>
/// OCCT's resource file resources/XSTEPResource/IGES (ShapeProcess's sequences and parameters for IGES data
/// exchange), compiled in byte for byte; <see cref="Manager(string, bool)"/> reads it for the name
/// <c>IGES</c> (open question 26: OCCT's default resource files, embedded).
/// </summary>
internal static class IgesResources
{
    // Generated from resources/XSTEPResource/IGES, line for line (CR LF kept)
    internal const string Text =
        "\r\n" +
        "! Resource file for data exchange ...\r\n" +
        "!\r\n" +
        "! For meaning of parameters refer to the end of the file\r\n" +
        "\r\n" +
        "! ==============================================================================\r\n" +
        "!\r\n" +
        "\r\n" +
        "ToIGES.exec.op: DirectFaces\r\n" +
        "\r\n" +
        "FromIGES.exec.op : FixShape\r\n" +
        "\r\n" +
        "FromIGES.FixShape.Tolerance3d\t\t\t: &Runtime.Tolerance\r\n" +
        "FromIGES.FixShape.MaxTolerance3d\t\t: &Runtime.MaxTolerance\r\n" +
        "FromIGES.FixShape.MinTolerance3d\t\t: 1.e-7\r\n" +
        "FromIGES.FixShape.FixFreeShellMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixFreeFaceMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixFreeWireMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixSameParameterMode\t\t: -1\r\n" +
        "\r\n" +
        "FromIGES.FixShape.FixSolidMode\t\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixShellOrientationMode\t: -1\r\n" +
        "FromIGES.FixShape.CreateOpenSolidMode           :  1\r\n" +
        "\r\n" +
        "FromIGES.FixShape.FixShellMode\t\t\t: -1\r\n" +
        "FromSTEP.FixShape.FixFaceOrientationMode\t: -1\r\n" +
        "\r\n" +
        "FromIGES.FixShape.FixFaceMode\t\t\t: -1\r\n" +
        "FromIGES.FixShape.FixWireMode\t\t\t: -1\r\n" +
        "FromIGES.FixShape.FixOrientationMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixAddNaturalBoundMode\t: -1\r\n" +
        "FromIGES.FixShape.FixMissingSeamMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixSmallAreaWireMode\t\t: -1\r\n" +
        "FromIGES.FixShape.RemoveSmallAreaFaceMode\t: -1\r\n" +
        "FromIGES.FixShape.FixIntersectingWiresMode\t: -1\r\n" +
        "FromIGES.FixShape.FixLoopWiresMode              : -1\r\n" +
        "FromIGES.FixShape.FixSplitFaceMode              : -1\r\n" +
        "FromIGES.FixShape.AutoCorrectPrecisionMode      :  1\r\n" +
        "FromIGES.FixShape.ModifyTopologyMode\t\t:  0\r\n" +
        "FromIGES.FixShape.ModifyGeometryMode\t\t:  1\r\n" +
        "FromIGES.FixShape.ClosedWireMode\t\t:  1\r\n" +
        "FromIGES.FixShape.PreferencePCurveMode\t\t:  1\r\n" +
        "FromIGES.FixShape.FixReorderMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixSmallMode\t\t\t: -1\r\n" +
        "FromIGES.FixShape.FixConnectedMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixEdgeCurvesMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixDegeneratedMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixLackingMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixSelfIntersectionMode\t: -1\r\n" +
        "FromIGES.FixShape.RemoveLoopMode                : -1\r\n" +
        "FromIGES.FixShape.FixReversed2dMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixRemovePCurveMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixRemoveCurve3dMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixAddPCurveMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixAddCurve3dMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixSeamMode\t\t\t: -1\r\n" +
        "FromIGES.FixShape.FixShiftedMode\t\t: -1\r\n" +
        "FromIGES.FixShape.FixEdgeSameParameterMode\t:  0\r\n" +
        "FromIGES.FixShape.FixNotchedEdgesMode \t\t: -1\r\n" +
        "FromIGES.FixShape.FixSelfIntersectingEdgeMode\t: -1\r\n" +
        "FromIGES.FixShape.FixIntersectingEdgesMode\t: -1\r\n" +
        "FromIGES.FixShape.FixNonAdjacentIntersectingEdgesMode : -1\r\n" +
        "\r\n" +
        "FromIGES.FixShape.FixVertexPositionMode : 0\r\n" +
        "FromIGES.FixShape.FixVertexToleranceMode : -1\r\n";
}
