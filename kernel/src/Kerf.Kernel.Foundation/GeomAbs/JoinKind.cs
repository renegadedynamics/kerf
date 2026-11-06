// Port of: src/FoundationClasses/TKMath/GeomAbs/GeomAbs_JoinType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>The type of join between two consecutive arcs of a contour parallel to a given contour.</summary>
public enum JoinKind
{
    /// <summary>A circular arc between the two parallel arcs: a rounded corner.</summary>
    Arc,
    /// <summary>The two parallel arcs are joined along their tangents.</summary>
    Tangent,
    /// <summary>The two parallel arcs are extended until they meet: a sharp corner.</summary>
    Intersection,
}
