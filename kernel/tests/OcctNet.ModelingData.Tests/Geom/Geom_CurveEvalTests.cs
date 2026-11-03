// Port of: src/ModelingData/TKG3d/GTests/Geom_CurveEval_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <remarks>
/// Not ported yet, because they build their curve with GeomAPI_PointsToBSpline (not ported):
/// BSplineCurve_EvalD1_ConsistentWithOldAPI and BSplineCurve_EvalDN_InvalidN_Throws.
/// </remarks>
public class Geom_CurveEvalTest
{
    private static Frame3D XY() => new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));

    [Fact]
    public void Circle_EvalD0_ReturnsValidPoint()
    {
        Circle3D aCirc = new(XY(), 5.0);
        Geom_Circle aCurve = new(aCirc);

        double aU = M_PI / 4.0;
        Point3D aResult = aCurve.EvalD0(aU);

        // Point should lie on circle of radius 5
        double aDist = aResult.Distance(new Point3D(0.0, 0.0, 0.0));
        Assert.Equal(5.0, aDist, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Circle_EvalD1D2D3_ReturnsValidDerivatives()
    {
        Circle3D aCirc = new(XY(), 5.0);
        Geom_Circle aCurve = new(aCirc);

        double aU = M_PI / 3.0;

        // EvalD1
        Geom_Curve.ResD1 aD1 = aCurve.EvalD1(aU);
        Assert.Equal(5.0, aD1.Point.Distance(new Point3D(0.0, 0.0, 0.0)), global::Kerf.Engine.Foundation.Precision.Confusion());
        // D1 should be perpendicular to radius vector for a circle
        Vector3D aRadial = new(new Point3D(0.0, 0.0, 0.0), aD1.Point);
        Assert.Equal(0.0, aRadial.Dot(aD1.D1), 1.0e-10);

        // EvalD2
        Geom_Curve.ResD2 aD2 = aCurve.EvalD2(aU);
        Assert.Equal(0.0, aD2.Point.Distance(aD1.Point), global::Kerf.Engine.Foundation.Precision.Confusion());
        // D2 of circle points toward center (centripetal acceleration)
        Assert.Equal(5.0, aD2.D2.Magnitude(), 1.0e-10);

        // EvalD3
        Geom_Curve.ResD3 aD3 = aCurve.EvalD3(aU);
        Assert.Equal(0.0, aD3.Point.Distance(aD1.Point), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aD3.D3.Magnitude() > 0.0);
    }

    // Line tests

    [Fact]
    public void Line_EvalD2_ZeroSecondDerivative()
    {
        Geom_Line aCurve = new(new Point3D(1.0, 2.0, 3.0), new Direction3D(1.0, 0.0, 0.0));

        Geom_Curve.ResD2 aD2 = aCurve.EvalD2(5.0);
        Assert.Equal(0.0, aD2.D2.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
        // D1 should be the direction vector
        Assert.Equal(1.0, aD2.D1.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aD2.D1.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aD2.D1.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // EvalDN consistency tests

    [Fact]
    public void EvalDN_N1_ConsistentWithEvalD1()
    {
        Circle3D aCirc = new(XY(), 3.0);
        Geom_Circle aCurve = new(aCirc);

        double aU = M_PI / 6.0;
        Geom_Curve.ResD1 aD1 = aCurve.EvalD1(aU);
        Vector3D aDN1 = aCurve.EvalDN(aU, 1);
        Assert.Equal(0.0, (aD1.D1 - aDN1).Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // OffsetCurve failure-path tests

    [Fact]
    public void OffsetCurve_EvalD0_ReturnsNulloptAtSingular()
    {
        // Create a circle with radius equal to offset -> singular at all points
        Circle3D aCirc = new(XY(), 5.0);
        Geom_Circle aBasis = new(aCirc);

        // Create a zero-radius circle by offsetting inward by -radius
        Geom_OffsetCurve anOffset = new(aBasis, -5.0, new Direction3D(0.0, 0.0, 1.0));

        Point3D aResult = anOffset.EvalD0(0.0);
        // For a circle offset by -radius, this should still return a valid point (the center)
        Assert.Equal(0.0, aResult.Distance(new Point3D(0.0, 0.0, 0.0)), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void OffsetCurve_D0Wrapper_ThrowsAtSingular()
    {
        Circle3D aCirc = new(XY(), 5.0);
        Geom_Circle aBasis = new(aCirc);
        Geom_OffsetCurve anOffset = new(aBasis, 2.0, new Direction3D(0.0, 0.0, 1.0));

        // The backward-compatibility wrapper should NOT throw for valid evaluation
        anOffset.D0(0.0, out _);
    }

    // Parabola tests

    [Fact]
    public void Parabola_EvalD3_ZeroThirdDerivative()
    {
        // Parabola: y = x^2 / (4*f), third derivative is zero
        Geom_Parabola aCurve = new(XY(), 2.0);

        Geom_Curve.ResD3 aD3 = aCurve.EvalD3(1.0);
        Assert.Equal(0.0, aD3.D3.Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // TrimmedCurve delegation test

    [Fact]
    public void TrimmedCurve_EvalD1_DelegatesToBasis()
    {
        Circle3D aCirc = new(XY(), 5.0);
        Geom_Circle aBasis = new(aCirc);
        Geom_TrimmedCurve aTrimmed = new(aBasis, 0.0, M_PI);

        double aU = M_PI / 4.0;
        Geom_Curve.ResD1 aBasisD1 = aBasis.EvalD1(aU);
        Geom_Curve.ResD1 aTrimmedD1 = aTrimmed.EvalD1(aU);

        Assert.Equal(0.0, aBasisD1.Point.Distance(aTrimmedD1.Point), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, (aBasisD1.D1 - aTrimmedD1.D1).Magnitude(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    // OffsetCurve EvalD1 singular test

    [Fact]
    public void OffsetCurve_EvalD1_DegenerateAtCollapsed()
    {
        Circle3D aCirc = new(XY(), 5.0);
        Geom_Circle aBasis = new(aCirc);

        // Offset by -radius creates a collapsed curve at the center.
        Geom_OffsetCurve anOffset = new(aBasis, -5.0, new Direction3D(0.0, 0.0, 1.0));

        Geom_Curve.ResD1 aResult = anOffset.EvalD1(0.0);
        // The point should be near the center
        Assert.Equal(0.0, aResult.Point.Distance(new Point3D(0.0, 0.0, 0.0)), global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}
