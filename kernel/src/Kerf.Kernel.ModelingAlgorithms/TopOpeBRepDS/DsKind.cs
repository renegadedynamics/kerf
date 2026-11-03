// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Kind.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>The kinds of object in the legacy data structure: its geometries (point, curve, surface) and shapes.</summary>
public enum DsKind
{
    Point,
    Curve,
    Surface,
    Vertex,
    Edge,
    Wire,
    Face,
    Shell,
    Solid,
    CompSolid,
    Compound,
    Unknown,
}
