// Port of OCCT's src/ModelingData/TKGeomBase/GTests/GeomBndLib_SurfaceOfRevolution_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class GeomBndLib_RevolutionTest
{
    private static readonly double Conf = Precision.Confusion();

    private static void ExpectNoLarger(in Bnd_Box theNew, in Bnd_Box theOld, double theTol)
        => GeomBndLibSurfaceTestUtils.ExpectNoLarger(theNew, theOld, theTol);

    private static void ExpectContainsSurface(in Bnd_Box theBox, Geom_SurfaceOfRevolution theSurf, double theUMin,
                                              double theUMax, double theVMin, double theVMax, int theNbU, int theNbV)
        => GeomBndLibSurfaceTestUtils.ExpectContainsSurface(theBox, theSurf, theUMin, theUMax, theVMin, theVMax, theNbU, theNbV);

    private static Geom_BSplineCurve TestBSpline()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 4);
        aPoles.SetValue(1, new gp_Pnt(3.0, 0.0, 0.0));
        aPoles.SetValue(2, new gp_Pnt(5.0, 0.0, 3.0));
        aPoles.SetValue(3, new gp_Pnt(4.0, 0.0, 7.0));
        aPoles.SetValue(4, new gp_Pnt(6.0, 0.0, 10.0));
        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From([0.0, 0.5, 1.0]);
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From([3, 1, 3]);
        return new Geom_BSplineCurve(aPoles, aKnots, aMults, 2);
    }

    [Fact]
    public void LineAroundZ_Full_CompareWithBndLib()
    {
        Geom_Curve aCurve = new Geom_Line(new gp_Pnt(5.0, 0.0, 0.0), gp.DZ());
        gp_Ax1 anAxis = new(gp.Origin(), gp.DZ());
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, 2.0 * M_PI, 0.0, 10.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 0.0, 10.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, 2.0 * M_PI, 0.0, 10.0, 36, 10);
    }

    [Fact]
    public void LineAroundZ_HalfTurn_CompareWithBndLib()
    {
        Geom_Curve aCurve = new Geom_Line(new gp_Pnt(5.0, 0.0, 0.0), gp.DZ());
        gp_Ax1 anAxis = new(gp.Origin(), gp.DZ());
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, M_PI, 0.0, 5.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, M_PI, 0.0, 5.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, M_PI, 0.0, 5.0, 18, 5);
    }

    [Fact]
    public void InfiniteV_OpenInAxisDirection()
    {
        // Line along Z through (5, 0, 0) revolved around Z axis (full 2*pi).
        // P(U, V) = (5*cos(U), 5*sin(U), V); V in (-inf, +inf).
        // Expected: X in [-5, 5], Y in [-5, 5], Z open both ways.
        Geom_Line aLine = new(new gp_Pnt(5.0, 0.0, 0.0), gp.DZ());
        gp_Ax1 anAxis = new(gp.Origin(), gp.DZ());
        Geom_SurfaceOfRevolution aRevSurf = new(aLine, anAxis);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, 2.0 * M_PI, -Precision.Infinite(), Precision.Infinite(), Conf, ref aNewBox);

        Assert.False(aNewBox.IsVoid());
        Assert.False(aNewBox.IsWhole());
        Assert.False(aNewBox.IsOpenXmin());
        Assert.False(aNewBox.IsOpenXmax());
        Assert.False(aNewBox.IsOpenYmin());
        Assert.False(aNewBox.IsOpenYmax());
        Assert.True(aNewBox.IsOpenZmin());
        Assert.True(aNewBox.IsOpenZmax());

        var (aXmin, aXmax, aYmin, aYmax, _, _) = aNewBox.Get();
        double aTol = 2.0 * Conf; // box is enlarged by theTol on construction
        Assert.Equal(-5.0, aXmin, aTol);
        Assert.Equal(5.0, aXmax, aTol);
        Assert.Equal(-5.0, aYmin, aTol);
        Assert.Equal(5.0, aYmax, aTol);
    }

    [Fact]
    public void LineAroundZ_QuarterTurn_CompareWithBndLib()
    {
        Geom_Curve aCurve = new Geom_Line(new gp_Pnt(5.0, 0.0, 0.0), gp.DZ());
        gp_Ax1 anAxis = new(gp.Origin(), gp.DZ());
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, M_PI / 2.0, 0.0, 5.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, M_PI / 2.0, 0.0, 5.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, M_PI / 2.0, 0.0, 5.0, 9, 5);
    }

    [Fact]
    public void LineAroundX_Full_CompareWithBndLib()
    {
        Geom_Curve aCurve = new Geom_Line(new gp_Pnt(0.0, 3.0, 0.0), gp.DX());
        gp_Ax1 anAxis = new(gp.Origin(), gp.DX());
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, 2.0 * M_PI, 0.0, 8.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 0.0, 8.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, 2.0 * M_PI, 0.0, 8.0, 36, 10);
    }

    [Fact]
    public void LineAroundTiltedAxis_CompareWithBndLib()
    {
        Geom_Curve aCurve = new Geom_Line(new gp_Pnt(4.0, 0.0, 0.0), gp.DZ());
        gp_Ax1 anAxis = new(gp.Origin(), new gp_Dir(1.0, 1.0, 0.0));
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, 2.0 * M_PI, 0.0, 5.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 0.0, 5.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, 2.0 * M_PI, 0.0, 5.0, 36, 10);
    }

    [Fact]
    public void CircleAroundZ_Full_CompareWithBndLib()
    {
        Geom_Curve aCurve = new Geom_Circle(new gp_Ax2(new gp_Pnt(5.0, 0.0, 0.0), gp.DY(), gp.DX()), 2.0);
        gp_Ax1 anAxis = new(gp.Origin(), gp.DZ());
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI, 36, 36);
    }

    [Fact]
    public void CircleAroundZ_HalfTurn_CompareWithBndLib()
    {
        Geom_Curve aCurve = new Geom_Circle(new gp_Ax2(new gp_Pnt(5.0, 0.0, 0.0), gp.DY(), gp.DX()), 2.0);
        gp_Ax1 anAxis = new(gp.Origin(), gp.DZ());
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, M_PI, 0.0, 2.0 * M_PI, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, M_PI, 0.0, 2.0 * M_PI, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, M_PI, 0.0, 2.0 * M_PI, 18, 36);
    }

    [Fact]
    public void ParabolaAroundZ_CompareWithBndLib()
    {
        Geom_Curve aCurve = new Geom_Parabola(gp.XOY(), 2.0);
        gp_Ax1 anAxis = new(gp.Origin(), gp.DZ());
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, 2.0 * M_PI, -3.0, 3.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, -3.0, 3.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, 2.0 * M_PI, -3.0, 3.0, 36, 10);
    }

    [Fact]
    public void EllipseAroundZ_CompareWithBndLib()
    {
        Geom_Curve aCurve = new Geom_Ellipse(new gp_Ax2(new gp_Pnt(5.0, 0.0, 0.0), gp.DY(), gp.DX()), 3.0, 1.5);
        gp_Ax1 anAxis = new(gp.Origin(), gp.DZ());
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI, 36, 36);
    }

    [Fact]
    public void BSplineAroundZ_CompareWithBndLib()
    {
        Geom_Curve aCurve = TestBSpline();
        gp_Ax1 anAxis = new(gp.Origin(), gp.DZ());
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, 2.0 * M_PI, 0.0, 1.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 0.0, 1.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, 2.0 * M_PI, 0.0, 1.0, 36, 10);
    }

    [Fact]
    public void BSplineAroundZ_HalfTurn_CompareWithBndLib()
    {
        Geom_Curve aCurve = TestBSpline();
        gp_Ax1 anAxis = new(gp.Origin(), gp.DZ());
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, M_PI, 0.0, 1.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, M_PI, 0.0, 1.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, M_PI, 0.0, 1.0, 18, 10);
    }

    [Fact]
    public void LineSmallRadius_CompareWithBndLib()
    {
        Geom_Curve aCurve = new Geom_Line(new gp_Pnt(0.5, 0.0, 0.0), gp.DZ());
        gp_Ax1 anAxis = new(gp.Origin(), gp.DZ());
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, 2.0 * M_PI, -5.0, 5.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, -5.0, 5.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, 2.0 * M_PI, -5.0, 5.0, 36, 10);
    }

    [Fact]
    public void LineLargeRadius_CompareWithBndLib()
    {
        Geom_Curve aCurve = new Geom_Line(new gp_Pnt(100.0, 0.0, 0.0), gp.DZ());
        gp_Ax1 anAxis = new(gp.Origin(), gp.DZ());
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, 2.0 * M_PI, 0.0, 10.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 0.0, 10.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, 2.0 * M_PI, 0.0, 10.0, 36, 10);
    }

    [Fact]
    public void LineAroundZ_PartialV_CompareWithBndLib()
    {
        Geom_Curve aCurve = new Geom_Line(new gp_Pnt(5.0, 0.0, 0.0), gp.DZ());
        gp_Ax1 anAxis = new(gp.Origin(), gp.DZ());
        Geom_SurfaceOfRevolution aRevSurf = new(aCurve, anAxis);
        GeomAdaptor_Surface anAdaptor = new(aRevSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aRevSurf).Add(0.0, 2.0 * M_PI, 3.0, 7.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 3.0, 7.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, 0.5);
        ExpectContainsSurface(aNewBox, aRevSurf, 0.0, 2.0 * M_PI, 3.0, 7.0, 36, 10);
    }
}
