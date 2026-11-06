// Port of: src/FoundationClasses/TKMath/GeomAbs/GeomAbs_SurfaceType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>The type of a surface.</summary>
public enum SurfaceKind
{
    /// <summary>A plane.</summary>
    Plane,
    /// <summary>A cylinder of revolution.</summary>
    Cylinder,
    /// <summary>A cone of revolution.</summary>
    Cone,
    /// <summary>A sphere.</summary>
    Sphere,
    /// <summary>A torus.</summary>
    Torus,
    /// <summary>A Bézier surface, rational or not.</summary>
    BezierSurface,
    /// <summary>A B-spline surface, rational or not.</summary>
    BSplineSurface,
    /// <summary>A curve swept round an axis.</summary>
    SurfaceOfRevolution,
    /// <summary>A curve swept along a straight line.</summary>
    SurfaceOfExtrusion,
    /// <summary>A surface at a constant distance from another.</summary>
    OffsetSurface,
    /// <summary>Any other surface.</summary>
    OtherSurface,
}
