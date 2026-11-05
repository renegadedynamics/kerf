// Port of: src/FoundationClasses/TKMath/Convert/Convert_CompBezierCurvesToBSplineCurve.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_CompBezierCurvesToBSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Engine;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>Converts a sequence of adjacent non-rational 3D Bezier curves into a B-spline curve.</summary>
internal class CompBezierCurvesToBSplineCurve
    : CompBezierCurvesToBSplineCurveBase<Point3D, Vector3D, CompBezierCurvesTraits3d>
{
    /// <summary>
    /// A converter checking the parallelism of the tangents at the junctions with
    /// <paramref name="angularTolerance"/> (radians).
    /// </summary>
    public CompBezierCurvesToBSplineCurve(double angularTolerance = 1.0e-4)
        : base(angularTolerance)
    {
    }
}
