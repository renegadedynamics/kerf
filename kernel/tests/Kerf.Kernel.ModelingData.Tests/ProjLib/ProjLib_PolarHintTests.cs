// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Geometry;
using P = Kerf.Engine.Foundation.Precision;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>Upstream ec0167b409's two GeomProjLib tests of the polar projection's hint at a sphere's poles.</summary>
public class ProjLib_PolarHintTests
{
    private static BSplineSurface SphereFrom(CoordinateSystem position, double u) =>
        global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(
            new TrimmedSurface(new SphericalSurface(position, 25.0), u, 2.0 * M_PI, -0.5 * M_PI, 0.5 * M_PI));

    [Fact]
    public void PolarProjectionPreservesIsolineAtSpherePoles()
    {
        double aU = 1.5 * M_PI;
        BSplineSurface aSurface =
            SphereFrom(new CoordinateSystem(new Point3D(5.0, 7.0, 11.0), new Direction3D(1.0, 1.0, 1.0)), aU);
        var aCurve = (Curve3D)aSurface.UIso(aU).Translated(new Vector3D(0.0, 0.0, 0.5 * P.Confusion()));
        var aPCurve = new LineCurve2D(new Point2D(aU, 0.0), new Direction2D(0.0, 1.0));
        var aProjection = new ComputeApproxOnPolarSurface(
            new CurveAdaptor2D(aPCurve, aCurve.FirstParameter, aCurve.LastParameter),
            new CurveAdaptor(aCurve), new SurfaceAdaptor(aSurface), P.Confusion());
        Assert.True(aProjection.IsDone);
        BSplineCurve2D? aResult = aProjection.BSpline();
        Assert.NotNull(aResult);
        for (int i = 0; i <= 32; ++i)
        {
            double aV = aCurve.FirstParameter + (aCurve.LastParameter - aCurve.FirstParameter) * i / 32.0;
            Point2D aUV = aResult.Value(aV);
            Assert.True(Math.Abs(aUV.X - aU) <= P.PConfusion(), $"U at {i}: {aUV.X - aU:R}");
            Assert.True(aSurface.Value(aUV.X, aUV.Y).DistanceTo(aCurve.Value(aV)) <= P.Confusion(), $"distance at {i}");
        }
    }

    [Fact]
    public void PolarProjectionRefinesHintOutsideGeometricConfusion()
    {
        const double aProjectionTolerance = 1.0e-3;
        double aU = 1.5 * M_PI;
        BSplineSurface aSurface = SphereFrom(new CoordinateSystem(), aU);
        Curve3D aCurve = new TrimmedCurve3D(aSurface.UIso(aU), -0.25 * M_PI, 0.25 * M_PI);
        var aHint = new LineCurve2D(new Point2D(aU + P.Approximation(), 0.0), new Direction2D(0.0, 1.0));
        var aProjection = new ComputeApproxOnPolarSurface(
            new CurveAdaptor2D(aHint, aCurve.FirstParameter, aCurve.LastParameter),
            new CurveAdaptor(aCurve), new SurfaceAdaptor(aSurface), aProjectionTolerance);
        Assert.True(aProjection.IsDone);
        BSplineCurve2D? aResult = aProjection.BSpline();
        Assert.NotNull(aResult);
        double aMiddle = 0.5 * (aCurve.FirstParameter + aCurve.LastParameter);
        Point2D anInitialUV = aHint.Value(aMiddle);
        Point2D aFinalUV = aResult.Value(aMiddle);
        Point3D aPoint = aCurve.Value(aMiddle);
        Assert.True(Math.Abs(aFinalUV.X - aU) <= P.PConfusion());
        Assert.True(aSurface.Value(aFinalUV.X, aFinalUV.Y).DistanceTo(aPoint)
                    < aSurface.Value(anInitialUV.X, anInitialUV.Y).DistanceTo(aPoint));
    }
}
