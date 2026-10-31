// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_IType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The kind of an intersection line: a conic, analytic, walking or a restriction.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.IntPatch_IType;</c>.</remarks>
public enum IntPatch_IType
{
    IntPatch_Lin,
    IntPatch_Circle,
    IntPatch_Ellipse,
    IntPatch_Parabola,
    IntPatch_Hyperbola,
    IntPatch_Analytic,
    IntPatch_Walking,
    IntPatch_Restriction,
}
