// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <summary>GeomConvert_Units (OCCT has no GTests for it).</summary>
public class GeomConvert_UnitsTests
{
    private const double DegToRad = Math.PI / 180.0;
    private const double RadToDeg = 180.0 / Math.PI;

    private static void AssertNear(in Point2D theExpected, in Point2D theActual, double theTol = 1e-12)
        => Assert.True(theExpected.Distance(theActual) <= theTol, $"{theExpected} vs {theActual}");

    [Fact]
    public void DegreeToRadian_LineOnCylinder()
    {
        var aCyl = new CylindricalSurface(new CoordinateSystem(Constants.XOY()), 3.0);
        var aLine = new LineCurve2D(new Point2D(180.0, 2.0), new Direction2D(0.0, 1.0));
        aLine.Freeze();
        var aRes = Assert.IsType<LineCurve2D>(Units.DegreeToRadian(aLine, aCyl, 10.0, DegToRad));
        AssertNear(new Point2D(Math.PI, 20.0), aRes.Location());
        Assert.True(aRes.Direction().IsEqual(new Direction2D(0.0, 1.0), 1e-15));
        Assert.NotSame(aLine, aRes);
        Assert.Equal(180.0, aLine.Location().X());

        // And back
        var aBack = Assert.IsType<LineCurve2D>(Units.RadianToDegree(aRes, aCyl, 10.0, RadToDeg));
        AssertNear(new Point2D(180.0, 2.0), aBack.Location(), 1e-12);
    }

    [Fact]
    public void DegreeToRadian_BSplineOnSphere_And_Cone()
    {
        var aPoles = new NCollection_Array1<Point2D>(1, 3);
        aPoles[1] = new Point2D(0.0, -45.0);
        aPoles[2] = new Point2D(90.0, 0.0);
        aPoles[3] = new Point2D(180.0, 30.0);
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 3;
        aMults[2] = 3;
        var aBSpline = new BSplineCurve2D(aPoles, aKnots, aMults, 2);

        var aSphere = new SphericalSurface(new CoordinateSystem(Constants.XOY()), 1.0);
        var aRes = Assert.IsType<BSplineCurve2D>(Units.DegreeToRadian(aBSpline, aSphere, 1.0, DegToRad));
        AssertNear(new Point2D(0.0, -Math.PI / 4), aRes.Pole(1));
        AssertNear(new Point2D(Math.PI, 30.0 * DegToRad), aRes.Pole(3));
        Assert.Equal(90.0, aBSpline.Pole(2).X());

        var aCone = new ConicalSurface(new CoordinateSystem(Constants.XOY()), 0.3, 1.0);
        var aConeRes = Assert.IsType<BSplineCurve2D>(Units.DegreeToRadian(aBSpline, aCone, 2.0, DegToRad));
        AssertNear(new Point2D(Math.PI / 2, 0.0), aConeRes.Pole(2));
        Assert.Equal(-45.0 * 2.0 / Math.Cos(0.3), aConeRes.Pole(1).Y(), 12);

        var aConeBack = Assert.IsType<BSplineCurve2D>(Units.RadianToDegree(aConeRes, aCone, 2.0, RadToDeg));
        AssertNear(aBSpline.Pole(1), aConeBack.Pole(1), 1e-11);
    }

    [Fact]
    public void Plane_ScalesCircles_AndOtherSurfacesKeepTheCurve()
    {
        var aPlane = new PlaneSurface(new CoordinateSystem(Constants.XOY()));
        var aCircle = new CircleCurve2D(new Axis2D(new Point2D(1.0, 2.0), new Direction2D(1.0, 0.0)), 3.0);
        var aScaled = Assert.IsType<CircleCurve2D>(Units.DegreeToRadian(aCircle, aPlane, 10.0, DegToRad));
        Assert.Equal(30.0, aScaled.Radius(), 12);
        AssertNear(new Point2D(10.0, 20.0), aScaled.Location());
        var aBack = Assert.IsType<CircleCurve2D>(Units.RadianToDegree(aScaled, aPlane, 10.0, RadToDeg));
        Assert.Equal(3.0, aBack.Radius(), 12);

        // A B-spline surface: the curve is returned as a copy
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0, 0, 0);
        aPoles[1, 2] = new Point3D(0, 1, 0);
        aPoles[2, 1] = new Point3D(1, 0, 0);
        aPoles[2, 2] = new Point3D(1, 1, 1);
        var aBez = new BezierSurface(aPoles);
        var aCopy = Assert.IsType<CircleCurve2D>(Units.DegreeToRadian(aCircle, aBez, 10.0, DegToRad));
        Assert.NotSame(aCircle, aCopy);
        Assert.Equal(3.0, aCopy.Radius());
    }

    [Fact]
    public void MirrorPCurve()
    {
        var aLine = new LineCurve2D(new Point2D(1.0, 2.0), new Direction2D(1.0, 1.0));
        var aMirror = Assert.IsType<LineCurve2D>(Units.MirrorPCurve(aLine));
        AssertNear(new Point2D(1.0, -2.0), aMirror.Location());
        Assert.True(aMirror.Direction().IsEqual(new Direction2D(1.0, -1.0), 1e-15));
    }

    [Fact]
    public void CircleOnPeriodicSurface_BecomesScaledBSpline()
    {
        // Circles on a periodic surface go through Geom2dConvert::CurveToBSplineCurve, then the poles are scaled
        var aCyl = new CylindricalSurface(new CoordinateSystem(Constants.XOY()), 3.0);
        var aCircle = new CircleCurve2D(new Axis2D(new Point2D(100.0, 2.0), new Direction2D(1.0, 0.0)), 30.0);
        var aRes = Assert.IsType<BSplineCurve2D>(Units.DegreeToRadian(aCircle, aCyl, 10.0, DegToRad));
        BSplineCurve2D aRef = Geom2dConvert2D.CurveToBSplineCurve(aCircle);
        Assert.Equal(aRef.PoleCount, aRes.PoleCount);
        for (int i = 0; i <= 8; i++)
        {
            double aU = aRef.FirstParameter() + i * (aRef.LastParameter() - aRef.FirstParameter()) / 8;
            Point2D aP = aRef.Value(aU);
            AssertNear(new Point2D(aP.X() * DegToRad, aP.Y() * 10.0), aRes.Value(aU), 1e-12);
        }
    }
}
