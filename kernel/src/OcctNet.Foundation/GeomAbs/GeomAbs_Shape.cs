// Port of: src/FoundationClasses/TKMath/GeomAbs/GeomAbs_Shape.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The continuity of a curve or surface, in increasing order.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.GeomAbs_Shape;</c>.</remarks>
public enum GeomAbs_Shape
{
    /// <summary>Only geometric continuity.</summary>
    GeomAbs_C0,
    /// <summary>The tangent vectors on either side are collinear with the same orientation.</summary>
    GeomAbs_G1,
    /// <summary>Continuity of the first derivative (so also G1).</summary>
    GeomAbs_C1,
    /// <summary>The normalized normal vectors on either side are equal (for surfaces: B-splines only, the main curvature radii and directions agree at a knot).</summary>
    GeomAbs_G2,
    /// <summary>Continuity of the second derivative.</summary>
    GeomAbs_C2,
    /// <summary>Continuity of the third derivative.</summary>
    GeomAbs_C3,
    /// <summary>Continuity of the derivatives of every order.</summary>
    GeomAbs_CN,
}
