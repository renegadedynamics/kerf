// Port of OCCT's src/ModelingData/TKGeomBase/GTests/GeomBndLib_SurfaceOfExtrusion_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class GeomBndLib_ExtrusionTest
{
    private static readonly double Conf = global::Kerf.Engine.Foundation.Precision.Confusion();

    private static void ExpectNoLarger(in Box theNew, in Box theOld, double theTol)
        => GeomBndLibSurfaceTestUtils.ExpectNoLarger(theNew, theOld, theTol);

    private static void ExpectContainsSurface(in Box theBox, Geom_SurfaceOfLinearExtrusion theSurf, double theUMin,
                                              double theUMax, double theVMin, double theVMax, int theNbU, int theNbV)
        => GeomBndLibSurfaceTestUtils.ExpectContainsSurface(theBox, theSurf, theUMin, theUMax, theVMin, theVMax, theNbU, theNbV);

    private static Geom_BSplineCurve TestBSpline()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles.SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoles.SetValue(2, new Point3D(3.0, 5.0, 0.0));
        aPoles.SetValue(3, new Point3D(7.0, -2.0, 0.0));
        aPoles.SetValue(4, new Point3D(10.0, 3.0, 0.0));
        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From([0.0, 0.5, 1.0]);
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From([3, 1, 3]);
        return new Geom_BSplineCurve(aPoles, aKnots, aMults, 2);
    }

    // =========================================================================
    // Circle along Z
    // =========================================================================

    [Fact]
    public void CircleAlongZ_Full_CompareWithBndLib()
    {
        Geom_Circle aCircle = new(Constants.XOY(), 5.0);
        Geom_SurfaceOfLinearExtrusion anExtr = new(aCircle, Constants.DZ());
        GeomAdaptor_Surface anAdaptor = new(anExtr);

        Box aNewBox = new();
        new GeomBndLib_Surface(anExtr).Add(0.0, 2.0 * M_PI, 0.0, 10.0, Conf, ref aNewBox);

        Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 0.0, 10.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsSurface(aNewBox, anExtr, 0.0, 2.0 * M_PI, 0.0, 10.0, 36, 10);
    }

    [Fact]
    public void CircleAlongZ_Arc_CompareWithBndLib()
    {
        Geom_Circle aCircle = new(Constants.XOY(), 5.0);
        Geom_SurfaceOfLinearExtrusion anExtr = new(aCircle, Constants.DZ());
        GeomAdaptor_Surface anAdaptor = new(anExtr);

        Box aNewBox = new();
        new GeomBndLib_Surface(anExtr).Add(0.0, M_PI, 0.0, 5.0, Conf, ref aNewBox);

        Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, M_PI, 0.0, 5.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsSurface(aNewBox, anExtr, 0.0, M_PI, 0.0, 5.0, 18, 5);
    }

    [Fact]
    public void InfiniteU_OpenInExtrusionBasisDirection()
    {
        // Line along Z through (5, 0, 0) extruded along X.
        // P(U, V) = (5 + V, 0, U); U in (-inf, +inf), V in [0, 10].
        // Expected: X in [5, 15], Y finite (= 0), Z open both ways.
        Geom_Line aLine = new(new Point3D(5.0, 0.0, 0.0), Constants.DZ());
        Geom_SurfaceOfLinearExtrusion anExtr = new(aLine, Constants.DX());

        Box aNewBox = new();
        new GeomBndLib_Surface(anExtr).Add(-global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite(), 0.0, 10.0, Conf, ref aNewBox);

        Assert.False(aNewBox.IsVoid);
        Assert.False(aNewBox.IsWhole);
        Assert.False(aNewBox.IsOpenXmin);
        Assert.False(aNewBox.IsOpenXmax);
        Assert.False(aNewBox.IsOpenYmin);
        Assert.False(aNewBox.IsOpenYmax);
        Assert.True(aNewBox.IsOpenZmin);
        Assert.True(aNewBox.IsOpenZmax);

        var (aXmin, aXmax, aYmin, aYmax, _, _) = aNewBox.Get();
        double aTol = 2.0 * Conf; // box is enlarged by theTol on construction
        Assert.Equal(5.0, aXmin, aTol);
        Assert.Equal(15.0, aXmax, aTol);
        Assert.Equal(0.0, aYmin, aTol);
        Assert.Equal(0.0, aYmax, aTol);
    }

    // =========================================================================
    // Circle along diagonal
    // =========================================================================

    [Fact]
    public void CircleAlongDiagonal_CompareWithBndLib()
    {
        Geom_Circle aCircle = new(Constants.XOY(), 5.0);
        Geom_SurfaceOfLinearExtrusion anExtr = new(aCircle, new Direction3D(1.0, 0.0, 1.0));
        GeomAdaptor_Surface anAdaptor = new(anExtr);

        Box aNewBox = new();
        new GeomBndLib_Surface(anExtr).Add(0.0, 2.0 * M_PI, 0.0, 10.0, Conf, ref aNewBox);

        Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 0.0, 10.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsSurface(aNewBox, anExtr, 0.0, 2.0 * M_PI, 0.0, 10.0, 36, 10);
    }

    // =========================================================================
    // Ellipse along Z
    // =========================================================================

    [Fact]
    public void EllipseAlongZ_CompareWithBndLib()
    {
        Geom_Ellipse anEllipse = new(Constants.XOY(), 8.0, 3.0);
        Geom_SurfaceOfLinearExtrusion anExtr = new(anEllipse, Constants.DZ());
        GeomAdaptor_Surface anAdaptor = new(anExtr);

        Box aNewBox = new();
        new GeomBndLib_Surface(anExtr).Add(0.0, 2.0 * M_PI, 0.0, 5.0, Conf, ref aNewBox);

        Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 0.0, 5.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsSurface(aNewBox, anExtr, 0.0, 2.0 * M_PI, 0.0, 5.0, 36, 5);
    }

    [Fact]
    public void EllipseAlongZ_Arc_CompareWithBndLib()
    {
        Geom_Ellipse anEllipse = new(Constants.XOY(), 8.0, 3.0);
        Geom_SurfaceOfLinearExtrusion anExtr = new(anEllipse, Constants.DZ());
        GeomAdaptor_Surface anAdaptor = new(anExtr);

        Box aNewBox = new();
        new GeomBndLib_Surface(anExtr).Add(0.0, M_PI, -2.0, 5.0, Conf, ref aNewBox);

        Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, M_PI, -2.0, 5.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsSurface(aNewBox, anExtr, 0.0, M_PI, -2.0, 5.0, 18, 7);
    }

    // =========================================================================
    // Line along Z (planar surface)
    // =========================================================================

    [Fact]
    public void LineAlongZ_CompareWithBndLib()
    {
        Geom_Line aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Geom_SurfaceOfLinearExtrusion anExtr = new(aLine, Constants.DZ());
        GeomAdaptor_Surface anAdaptor = new(anExtr);

        Box aNewBox = new();
        new GeomBndLib_Surface(anExtr).Add(0.0, 10.0, 0.0, 5.0, Conf, ref aNewBox);

        Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 10.0, 0.0, 5.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsSurface(aNewBox, anExtr, 0.0, 10.0, 0.0, 5.0, 10, 5);
    }

    // =========================================================================
    // Line along diagonal
    // =========================================================================

    [Fact]
    public void LineAlongDiagonal_CompareWithBndLib()
    {
        Geom_Line aLine = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0));
        Geom_SurfaceOfLinearExtrusion anExtr = new(aLine, new Direction3D(1.0, 0.0, 1.0));
        GeomAdaptor_Surface anAdaptor = new(anExtr);

        Box aNewBox = new();
        new GeomBndLib_Surface(anExtr).Add(0.0, 5.0, 0.0, 10.0, Conf, ref aNewBox);

        Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 5.0, 0.0, 10.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsSurface(aNewBox, anExtr, 0.0, 5.0, 0.0, 10.0, 5, 10);
    }

    // =========================================================================
    // BSpline along Z
    // =========================================================================

    [Fact]
    public void BSplineAlongZ_CompareWithBndLib()
    {
        Geom_BSplineCurve aBSpl = TestBSpline();
        Geom_SurfaceOfLinearExtrusion anExtr = new(aBSpl, Constants.DZ());
        GeomAdaptor_Surface anAdaptor = new(anExtr);

        Box aNewBox = new();
        new GeomBndLib_Surface(anExtr).AddOptimal(0.0, 1.0, 0.0, 5.0, Conf, ref aNewBox);

        Box anOldBox = new();
        BndLib_AddSurface.AddOptimal(anAdaptor, 0.0, 1.0, 0.0, 5.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsSurface(aNewBox, anExtr, 0.0, 1.0, 0.0, 5.0, 10, 5);
    }

    [Fact]
    public void BSplineAlongDiagonal_CompareWithBndLib()
    {
        Geom_BSplineCurve aBSpl = TestBSpline();
        Geom_SurfaceOfLinearExtrusion anExtr = new(aBSpl, new Direction3D(1.0, 1.0, 1.0));

        Box aNewBox = new();
        new GeomBndLib_Surface(anExtr).AddOptimal(0.0, 1.0, -3.0, 7.0, Conf, ref aNewBox);

        // P(u,v) = C(u) + v*(1,1,1)/sqrt(3), u in [0,1], v in [-3, 7].
        // BSpline C(0)=(0,0,0), C(1)=(10,3,0). Extrusion shifts: v_min/sqrt(3) = -sqrt(3),
        // v_max/sqrt(3) = 7/sqrt(3). Reference bounds derive from endpoint extremes + extrusion range.
        double aSqrt3 = Math.Sqrt(3.0);
        double aVminShift = -3.0 / aSqrt3; // ~= -1.7321
        double aVmaxShift = 7.0 / aSqrt3;  // ~=  4.0415
        double aTolRef = 1e-3;
        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aNewBox.Get();
        Assert.Equal(0.0 + aVminShift, aXmin, aTolRef);
        Assert.Equal(10.0 + aVmaxShift, aXmax, aTolRef);
        Assert.Equal(0.0 + aVminShift, aYmin, aTolRef);
        Assert.Equal(3.0 + aVmaxShift, aYmax, aTolRef);
        Assert.Equal(aVminShift, aZmin, aTolRef);
        Assert.Equal(aVmaxShift, aZmax, aTolRef);

        ExpectContainsSurface(aNewBox, anExtr, 0.0, 1.0, -3.0, 7.0, 10, 10);
    }

    // =========================================================================
    // Negative V range
    // =========================================================================

    [Fact]
    public void CircleNegativeV_CompareWithBndLib()
    {
        Geom_Circle aCircle = new(Constants.XOY(), 5.0);
        Geom_SurfaceOfLinearExtrusion anExtr = new(aCircle, Constants.DZ());
        GeomAdaptor_Surface anAdaptor = new(anExtr);

        Box aNewBox = new();
        new GeomBndLib_Surface(anExtr).Add(0.0, 2.0 * M_PI, -5.0, 5.0, Conf, ref aNewBox);

        Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, -5.0, 5.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsSurface(aNewBox, anExtr, 0.0, 2.0 * M_PI, -5.0, 5.0, 36, 10);
    }
}
