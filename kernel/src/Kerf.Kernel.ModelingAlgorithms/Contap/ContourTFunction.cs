// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_TFunction.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.HiddenLines.Contours;

/// <summary>The kind of contour: silhouette for a direction or an eye, or draft for a direction or an eye.</summary>
internal enum ContourTFunction
{
    ContourStd,
    ContourPrs,
    DraftStd,
    DraftPrs,
}
