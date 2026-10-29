// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>GeomConvert_Units (OCCT has no GTests for it).</summary>
public class GeomConvert_UnitsTests
{
    private const double DegToRad = Math.PI / 180.0;
    private const double RadToDeg = 180.0 / Math.PI;

    private static void AssertNear(in gp_Pnt2d theExpected, in gp_Pnt2d theActual, double theTol = 1e-12)
        => Assert.True(theExpected.Distance(theActual) <= theTol, $"{theExpected} vs {theActual}");

    [Fact]
    public void DegreeToRadian_LineOnCylinder()
    {
        var aCyl = new Geom_CylindricalSurface(new gp_Ax3(gp.XOY()), 3.0);
        var aLine = new Geom2d_Line(new gp_Pnt2d(180.0, 2.0), new gp_Dir2d(0.0, 1.0));
        aLine.Freeze();
        var aRes = Assert.IsType<Geom2d_Line>(GeomConvert_Units.DegreeToRadian(aLine, aCyl, 10.0, DegToRad));
        AssertNear(new gp_Pnt2d(Math.PI, 20.0), aRes.Location());
        Assert.True(aRes.Direction().IsEqual(new gp_Dir2d(0.0, 1.0), 1e-15));
        Assert.NotSame(aLine, aRes);
        Assert.Equal(180.0, aLine.Location().X());

        // And back
        var aBack = Assert.IsType<Geom2d_Line>(GeomConvert_Units.RadianToDegree(aRes, aCyl, 10.0, RadToDeg));
        AssertNear(new gp_Pnt2d(180.0, 2.0), aBack.Location(), 1e-12);
    }

    [Fact]
    public void DegreeToRadian_BSplineOnSphere_And_Cone()
    {
        var aPoles = new NCollection_Array1<gp_Pnt2d>(1, 3);
        aPoles[1] = new gp_Pnt2d(0.0, -45.0);
        aPoles[2] = new gp_Pnt2d(90.0, 0.0);
        aPoles[3] = new gp_Pnt2d(180.0, 30.0);
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 3;
        aMults[2] = 3;
        var aBSpline = new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 2);

        var aSphere = new Geom_SphericalSurface(new gp_Ax3(gp.XOY()), 1.0);
        var aRes = Assert.IsType<Geom2d_BSplineCurve>(GeomConvert_Units.DegreeToRadian(aBSpline, aSphere, 1.0, DegToRad));
        AssertNear(new gp_Pnt2d(0.0, -Math.PI / 4), aRes.Pole(1));
        AssertNear(new gp_Pnt2d(Math.PI, 30.0 * DegToRad), aRes.Pole(3));
        Assert.Equal(90.0, aBSpline.Pole(2).X());

        var aCone = new Geom_ConicalSurface(new gp_Ax3(gp.XOY()), 0.3, 1.0);
        var aConeRes = Assert.IsType<Geom2d_BSplineCurve>(GeomConvert_Units.DegreeToRadian(aBSpline, aCone, 2.0, DegToRad));
        AssertNear(new gp_Pnt2d(Math.PI / 2, 0.0), aConeRes.Pole(2));
        Assert.Equal(-45.0 * 2.0 / Math.Cos(0.3), aConeRes.Pole(1).Y(), 12);

        var aConeBack = Assert.IsType<Geom2d_BSplineCurve>(GeomConvert_Units.RadianToDegree(aConeRes, aCone, 2.0, RadToDeg));
        AssertNear(aBSpline.Pole(1), aConeBack.Pole(1), 1e-11);
    }

    [Fact]
    public void Plane_ScalesCircles_AndOtherSurfacesKeepTheCurve()
    {
        var aPlane = new Geom_Plane(new gp_Ax3(gp.XOY()));
        var aCircle = new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(1.0, 0.0)), 3.0);
        var aScaled = Assert.IsType<Geom2d_Circle>(GeomConvert_Units.DegreeToRadian(aCircle, aPlane, 10.0, DegToRad));
        Assert.Equal(30.0, aScaled.Radius(), 12);
        AssertNear(new gp_Pnt2d(10.0, 20.0), aScaled.Location());
        var aBack = Assert.IsType<Geom2d_Circle>(GeomConvert_Units.RadianToDegree(aScaled, aPlane, 10.0, RadToDeg));
        Assert.Equal(3.0, aBack.Radius(), 12);

        // A B-spline surface: the curve is returned as a copy
        var aPoles = new NCollection_Array2<gp_Pnt>(1, 2, 1, 2);
        aPoles[1, 1] = new gp_Pnt(0, 0, 0);
        aPoles[1, 2] = new gp_Pnt(0, 1, 0);
        aPoles[2, 1] = new gp_Pnt(1, 0, 0);
        aPoles[2, 2] = new gp_Pnt(1, 1, 1);
        var aBez = new Geom_BezierSurface(aPoles);
        var aCopy = Assert.IsType<Geom2d_Circle>(GeomConvert_Units.DegreeToRadian(aCircle, aBez, 10.0, DegToRad));
        Assert.NotSame(aCircle, aCopy);
        Assert.Equal(3.0, aCopy.Radius());
    }

    [Fact]
    public void MirrorPCurve()
    {
        var aLine = new Geom2d_Line(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(1.0, 1.0));
        var aMirror = Assert.IsType<Geom2d_Line>(GeomConvert_Units.MirrorPCurve(aLine));
        AssertNear(new gp_Pnt2d(1.0, -2.0), aMirror.Location());
        Assert.True(aMirror.Direction().IsEqual(new gp_Dir2d(1.0, -1.0), 1e-15));
    }

    [Fact]
    public void CircleOnPeriodicSurface_NotImplementedUntil4CMerge()
    {
        // Needs Geom2dConvert::CurveToBSplineCurve (ported in another part of unit 4C)
        var aCyl = new Geom_CylindricalSurface(new gp_Ax3(gp.XOY()), 3.0);
        var aCircle = new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(1.0, 2.0), new gp_Dir2d(1.0, 0.0)), 3.0);
        Assert.ThrowsAny<Standard_NotImplemented>(() => GeomConvert_Units.DegreeToRadian(aCircle, aCyl, 1.0, DegToRad));
    }
}
