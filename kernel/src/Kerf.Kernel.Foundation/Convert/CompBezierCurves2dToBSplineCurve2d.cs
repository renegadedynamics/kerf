// Port of: src/FoundationClasses/TKMath/Convert/Convert_CompBezierCurves2dToBSplineCurve2d.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_CompBezierCurves2dToBSplineCurve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Engine;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts a sequence of adjacent non-rational 2D Bezier curves into a B-spline curve; where possible, the
/// continuity of the B-spline is raised above C0.
/// </summary>
internal class CompBezierCurves2dToBSplineCurve2d
    : CompBezierCurvesToBSplineCurveBase<Point2D, Vector2D, CompBezierCurvesTraits2d>
{
    /// <summary>
    /// A converter checking the parallelism of the tangents at the junctions with
    /// <paramref name="angularTolerance"/> (radians).
    /// </summary>
    public CompBezierCurves2dToBSplineCurve2d(double angularTolerance = 1.0e-4)
        : base(angularTolerance)
    {
    }
}
