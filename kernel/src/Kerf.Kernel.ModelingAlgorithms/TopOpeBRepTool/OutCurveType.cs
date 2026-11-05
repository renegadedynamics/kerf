// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_OutCurveType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Booleans.Legacy.Tools;

/// <summary>
/// The kind of 3D curve the legacy Booleans compute on a walking intersection line: a BSpline of degree 1 on its
/// points, an approximation, or an interpolation.
/// </summary>
internal enum OutCurveType
{
    BSPLINE1,
    Approx,
    Interpol,
}
