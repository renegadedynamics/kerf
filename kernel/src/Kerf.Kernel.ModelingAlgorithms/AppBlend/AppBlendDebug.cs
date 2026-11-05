// Port of: src/ModelingAlgorithms/TKGeomAlgo/AppBlend/AppBlend_Debug.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// AppBlend_Debug.cxx's free functions: two switches of the AppSurf approximations (the spline approximation, and
/// approximation without tangents), process-wide in OCCT, settings of the <see cref="OperationContext"/> here (unit
/// 10I): a setter changes them for the rest of the current flow of execution. Internal (decision 415).
/// </summary>
internal static class AppBlendDebug
{
    private static readonly ContextKey<bool> SplineApproxSetting = new("AppBlend.ContextSplineApprox", false);
    private static readonly ContextKey<bool> ApproxWithNoTgtSetting = new("AppBlend.ContextApproxWithNoTgt", false);

    internal static void AppBlend_SetContextSplineApprox(bool b) => OperationContext.Set(SplineApproxSetting, b);

    internal static bool AppBlend_GetContextSplineApprox() => SplineApproxSetting.Value;

    internal static void AppBlend_SetContextApproxWithNoTgt(bool b) => OperationContext.Set(ApproxWithNoTgtSetting, b);

    internal static bool AppBlend_GetContextApproxWithNoTgt() => ApproxWithNoTgtSetting.Value;
}
