// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Position.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The position of an intersection point on a curve: at its start (Head), inside, or at its end.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.IntRes2d_Position;</c>.</remarks>
public enum IntRes2d_Position
{
    IntRes2d_Head,
    IntRes2d_Middle,
    IntRes2d_End,
}
