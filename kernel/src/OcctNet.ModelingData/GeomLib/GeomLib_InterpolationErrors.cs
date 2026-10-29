// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_InterpolationErrors.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>What went wrong when a GeomLib_Interpolate failed.</summary>
public enum GeomLib_InterpolationErrors
{
    GeomLib_NoError,
    GeomLib_NotEnoughtPoints,
    GeomLib_DegreeSmallerThan3,
    GeomLib_InversionProblem,
}
