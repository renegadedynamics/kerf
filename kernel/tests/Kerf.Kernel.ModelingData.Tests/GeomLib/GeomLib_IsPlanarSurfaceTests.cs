// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Library;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomLib_IsPlanarSurfaceTests
{
    private static readonly CoordinateSystem TheAx3 = new(new Point3D(1, 2, 3), new Direction3D(1, 1, 1), new Direction3D(1, -1, 0));

    private static BezierSurface PlanarBezier(double theNoise)
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 4, 1, 3);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                double aZ = (i == 2 && j == 2) ? theNoise : 0.0;
                Xyz aP = TheAx3.Location.XYZ() + (i + 0.1 * j) * TheAx3.XDirection.XYZ()
                            + (j * 1.5 - 0.2 * i) * TheAx3.YDirection.XYZ() + aZ * TheAx3.Direction.XYZ();
                aPoles[i, j] = new Point3D(aP);
            }
        }
        return new BezierSurface(aPoles);
    }

    private static void AssertNormal(Plane3D thePln, in Direction3D theExpected)
    {
        Assert.True(thePln.Axis.Direction.IsParallel(theExpected, 1e-9));
    }

    [Fact]
    public void Plane_IsPlanar()
    {
        var aPlane = new PlaneSurface(TheAx3);
        var anIs = new IsPlanarSurface(aPlane);
        Assert.True(anIs.IsPlanar);
        Assert.Equal(aPlane.Plane, anIs.Plan());
    }

    [Fact]
    public void Quadrics_AreNotPlanar()
    {
        Surface[] aSurfaces =
        [
            new CylindricalSurface(TheAx3, 2), new ConicalSurface(TheAx3, 0.3, 2),
            new SphericalSurface(TheAx3, 2), new ToroidalSurface(TheAx3, 5, 1),
        ];
        foreach (Surface aS in aSurfaces)
        {
            var anIs = new IsPlanarSurface(aS, 1.0);
            Assert.False(anIs.IsPlanar);
            EngineAssert.ThrowsAny<NotDoneException>(() => anIs.Plan());
        }
    }

    [Fact]
    public void PlanarBSpline_IsPlanar_WithItsNormal()
    {
        var aTrimmed = new TrimmedSurface(new PlaneSurface(TheAx3), -3, 4, -1, 2);
        BSplineSurface aBS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(aTrimmed);
        var anIs = new IsPlanarSurface(aBS);
        Assert.True(anIs.IsPlanar);
        AssertNormal(anIs.Plan(), TheAx3.Direction);
        Assert.True(anIs.Plan().DistanceTo(TheAx3.Location) < 1e-12);
    }

    [Fact]
    public void NearlyPlanarBezier_DependsOnTolerance()
    {
        BezierSurface aBz = PlanarBezier(1e-3);
        Assert.False(new IsPlanarSurface(aBz).IsPlanar);
        var anIs = new IsPlanarSurface(aBz, 1e-2);
        Assert.True(anIs.IsPlanar);
        Assert.True(anIs.Plan().Axis.Direction.AngleTo(TheAx3.Direction) < 1e-3
                    || anIs.Plan().Axis.Direction.AngleTo(TheAx3.Direction) > M_PI - 1e-3);
        Assert.True(new IsPlanarSurface(PlanarBezier(0.0)).IsPlanar);
    }

    [Fact]
    public void PlanarBezier_XDirectionFollowsDU()
    {
        var anIs = new IsPlanarSurface(PlanarBezier(0.0));
        Assert.True(anIs.IsPlanar);
        PlanarBezier(0.0).D1(0.5, 0.5, out Point3D _, out Vector3D DU, out Vector3D DV);
        CoordinateSystem aPos = anIs.Plan().Position;
        Assert.True(aPos.XDirection.AngleTo(new Direction3D(DU)) <= M_PI / 2);
        Assert.True(aPos.YDirection.AngleTo(new Direction3D(DV)) <= M_PI / 2);
    }

    [Fact]
    public void RevolutionOfPerpendicularSegment_IsPlanar()
    {
        var anAxis = new Axis3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1));
        var aSeg = new TrimmedCurve3D(new LineCurve3D(new Point3D(0, 0, 1), new Direction3D(1, 0, 0)), 1, 3);
        var anIs = new IsPlanarSurface(new RevolutionSurface(aSeg, anAxis));
        Assert.True(anIs.IsPlanar);
        AssertNormal(anIs.Plan(), new Direction3D(0, 0, 1));
        Assert.Equal(0.0, anIs.Plan().DistanceTo(new Point3D(5, 7, 1)), 1e-12);

        // A slightly tilted segment sweeps a flat cone: planar within a loose tolerance only
        var aTilted = new TrimmedCurve3D(new LineCurve3D(new Point3D(0, 0, 1), new Direction3D(1, 0, 1e-4)), 1, 3);
        var aCone = new RevolutionSurface(aTilted, anAxis);
        Assert.False(new IsPlanarSurface(aCone).IsPlanar);
        Assert.True(new IsPlanarSurface(aCone, 1e-2).IsPlanar);
    }

    [Fact]
    public void RevolutionOfCircle_IsNotPlanar()
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(5, 0, 0), new Direction3D(0, 1, 0)), 1);
        var aTorus = new RevolutionSurface(aCircle, new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        Assert.False(new IsPlanarSurface(aTorus).IsPlanar);
    }

    [Fact]
    public void ExtrusionOfPlanarCurve_InItsPlane_IsPlanar()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0, 0, 2);
        aPoles[2] = new Point3D(1, 3, 2);
        aPoles[3] = new Point3D(3, -1, 2);
        aPoles[4] = new Point3D(4, 2, 2);
        var aCurve = new BezierCurve3D(aPoles);
        var anExt = new ExtrusionSurface(aCurve, new Direction3D(1, 1, 0));
        var aBounded = new TrimmedSurface(anExt, -1, 1, false);
        var anIs = new IsPlanarSurface(aBounded);
        // A trimmed extrusion is not GeomAbs_SurfaceOfExtrusion for the adaptor: the default (sampling) branch
        Assert.True(anIs.IsPlanar);
        AssertNormal(anIs.Plan(), new Direction3D(0, 0, 1));
        var anIs2 = new IsPlanarSurface(anExt);
        Assert.True(anIs2.IsPlanar);
        AssertNormal(anIs2.Plan(), new Direction3D(0, 0, 1));

        // Out of the plane: a cylinder-like surface
        var anUp = new TrimmedSurface(new ExtrusionSurface(aCurve, new Direction3D(0, 0, 1)),
                                                      -1, 1, false);
        Assert.False(new IsPlanarSurface(anUp).IsPlanar);
    }

    [Fact]
    public void ExtrusionOfLine_IsPlanar()
    {
        var aLine = new TrimmedCurve3D(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 5);
        var anExt = new TrimmedSurface(
            new ExtrusionSurface(aLine, new Direction3D(0, 1, 1)), -2, 2, false);
        var anIs = new IsPlanarSurface(anExt);
        Assert.True(anIs.IsPlanar);
        AssertNormal(anIs.Plan(), new Direction3D(0, -1, 1));
    }

    [Fact]
    public void OffsetOfPlanarBSpline_IsPlanar()
    {
        var aTrimmed = new TrimmedSurface(new PlaneSurface(TheAx3), -3, 4, -1, 2);
        BSplineSurface aBS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(aTrimmed);
        var anIs = new IsPlanarSurface(new OffsetSurface(aBS, 2.5));
        Assert.True(anIs.IsPlanar);
        AssertNormal(anIs.Plan(), TheAx3.Direction);
        Assert.Equal(2.5, anIs.Plan().DistanceTo(TheAx3.Location), 1e-9);
    }

    [Fact]
    public void CollinearSamples_AreNotPlanar()
    {
        // A degenerate surface: every pole on one line (the second inertia is below Confusion)
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[1, 2] = new Point3D(1, 0, 0);
        aPoles[2, 1] = new Point3D(2, 0, 0);
        aPoles[2, 2] = new Point3D(3, 0, 0);
        Assert.False(new IsPlanarSurface(new BezierSurface(aPoles)).IsPlanar);
    }
}
