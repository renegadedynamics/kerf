// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Kind.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The kinds of object in the legacy data structure: its geometries (point, curve, surface) and shapes.</summary>
public enum TopOpeBRepDS_Kind
{
    TopOpeBRepDS_POINT,
    TopOpeBRepDS_CURVE,
    TopOpeBRepDS_SURFACE,
    TopOpeBRepDS_VERTEX,
    TopOpeBRepDS_EDGE,
    TopOpeBRepDS_WIRE,
    TopOpeBRepDS_FACE,
    TopOpeBRepDS_SHELL,
    TopOpeBRepDS_SOLID,
    TopOpeBRepDS_COMPSOLID,
    TopOpeBRepDS_COMPOUND,
    TopOpeBRepDS_UNKNOWN,
}
