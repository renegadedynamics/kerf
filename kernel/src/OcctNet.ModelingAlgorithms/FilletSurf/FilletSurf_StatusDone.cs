// Port of: src/ModelingAlgorithms/TKFillet/FilletSurf/FilletSurf_StatusDone.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The status of the computation of a <see cref="FilletSurf_Builder"/>.</summary>
public enum FilletSurf_StatusDone
{
    FilletSurf_IsOk,
    FilletSurf_IsNotOk,
    FilletSurf_IsPartial,
}
