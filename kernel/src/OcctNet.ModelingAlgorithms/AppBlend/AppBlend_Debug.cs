// Port of: src/ModelingAlgorithms/TKGeomAlgo/AppBlend/AppBlend_Debug.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// AppBlend_Debug.cxx's free functions: two process-wide switches of the AppSurf approximations (the spline
/// approximation, and approximation without tangents). Internal setters, as other process-wide statics (decision 415).
/// </summary>
internal static class AppBlend_Debug
{
    private static volatile bool AppBlend_ContextSplineApprox;
    private static volatile bool AppBlend_ContextApproxWithNoTgt;

    internal static void AppBlend_SetContextSplineApprox(bool b)
    {
        AppBlend_ContextSplineApprox = b;
    }

    internal static bool AppBlend_GetContextSplineApprox() => AppBlend_ContextSplineApprox;

    internal static void AppBlend_SetContextApproxWithNoTgt(bool b)
    {
        AppBlend_ContextApproxWithNoTgt = b;
    }

    internal static bool AppBlend_GetContextApproxWithNoTgt() => AppBlend_ContextApproxWithNoTgt;
}
