// Port of: src/FoundationClasses/TKMath/GeomAbs/GeomAbs_CurveType.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>The type of a curve.</summary>
public enum CurveKind
{
    /// <summary>A straight line.</summary>
    Line,
    /// <summary>A circle.</summary>
    Circle,
    /// <summary>An ellipse.</summary>
    Ellipse,
    /// <summary>One branch of a hyperbola.</summary>
    Hyperbola,
    /// <summary>A parabola.</summary>
    Parabola,
    /// <summary>A Bézier curve, rational or not.</summary>
    BezierCurve,
    /// <summary>A B-spline curve, rational or not.</summary>
    BSplineCurve,
    /// <summary>A curve at a constant distance from another.</summary>
    OffsetCurve,
    /// <summary>Any other curve, such as an adaptor over a curve on a surface.</summary>
    OtherCurve,
}
