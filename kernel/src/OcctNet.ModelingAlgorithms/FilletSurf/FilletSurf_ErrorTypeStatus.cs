// Port of: src/ModelingAlgorithms/TKFillet/FilletSurf/FilletSurf_ErrorTypeStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The error of a <see cref="FilletSurf_Builder"/> whose computation gave no result.</summary>
public enum FilletSurf_ErrorTypeStatus
{
    FilletSurf_EmptyList,
    FilletSurf_EdgeNotG1,
    FilletSurf_FacesNotG1,
    FilletSurf_EdgeNotOnShape,
    FilletSurf_NotSharpEdge,
    FilletSurf_PbFilletCompute,
}
