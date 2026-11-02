// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_FacetedError.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The error status of TopoDSToStep_Faceted.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.TopoDSToStep_FacetedError;</c>.</remarks>
public enum TopoDSToStep_FacetedError
{
    TopoDSToStep_FacetedDone,
    TopoDSToStep_SurfaceNotPlane,
    TopoDSToStep_PCurveNotLinear,
}
