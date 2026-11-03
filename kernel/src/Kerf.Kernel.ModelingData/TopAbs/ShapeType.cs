// Port of: src/ModelingData/TKG3d/TopAbs/TopAbs_ShapeEnum.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Topology;

/// <summary>The type of a shape, from the most complex (COMPOUND) to the simplest (VERTEX); SHAPE means any type.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.TopAbs_ShapeEnum;</c>.</remarks>
public enum ShapeType
{
    Compound,
    CompSolid,
    Solid,
    Shell,
    Face,
    Wire,
    Edge,
    Vertex,
    Shape,
}
