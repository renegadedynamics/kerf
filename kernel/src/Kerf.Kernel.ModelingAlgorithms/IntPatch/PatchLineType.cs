// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_IType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>The kind of an intersection line: a conic, analytic, walking or a restriction.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.IntPatch_IType;</c>.</remarks>
internal enum PatchLineType
{
    Lin,
    Circle,
    Ellipse,
    Parabola,
    Hyperbola,
    Analytic,
    Walking,
    Restriction,
}
