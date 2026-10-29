// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_ExtendedGeometry_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class ExtremaPC_ExtendedGeometryTest
{
    private const double THE_TOL = 1.0e-6;

    //==================================================================================================
    // Test fixture for extended geometry tests
    //==================================================================================================

    // Compares minimum distance between new and old implementations
    void CompareMinDistance(ExtremaPC.Result theNew,
            Extrema_ExtPC theOld,
            string theTestName,
            double theTol = THE_TOL)
    {
        if (!theOld.IsDone() || theOld.NbExt() == 0)
        {
            return;
        }

        Assert.True(theNew.IsDone()); // theTestName << ": New implementation failed"
        Assert.True(theNew.NbExt() > 0); // theTestName << ": No extrema found"

        double aOldMinSqDist = RealLast();
        for (int i = 1; i <= theOld.NbExt(); ++i)
        {
            if (theOld.SquareDistance(i) < aOldMinSqDist)
            {
                aOldMinSqDist = theOld.SquareDistance(i);
            }
        }

        double aNewMinDist = Math.Sqrt(theNew.MinSquareDistance());
        double aOldMinDist = Math.Sqrt(aOldMinSqDist);

        Assert.Equal(aOldMinDist, aNewMinDist, theTol); // theTestName << ": Distance mismatch. New: " << aNewMinDist << ", Old: " << aOldMinDist
    }

    // Compares maximum distance between new and old implementations
    void CompareMaxDistance(ExtremaPC.Result theNew,
            Extrema_ExtPC theOld,
            string theTestName,
            double theTol = THE_TOL)
    {
        if (!theOld.IsDone() || theOld.NbExt() == 0)
        {
            return;
        }

        Assert.True(theNew.IsDone()); // theTestName << ": New implementation failed"
        Assert.True(theNew.NbExt() > 0); // theTestName << ": No extrema found"

        double aOldMaxSqDist = 0.0;
        for (int i = 1; i <= theOld.NbExt(); ++i)
        {
            if (theOld.SquareDistance(i) > aOldMaxSqDist)
            {
                aOldMaxSqDist = theOld.SquareDistance(i);
            }
        }

        double aNewMaxDist = Math.Sqrt(theNew.MaxSquareDistance());
        double aOldMaxDist = Math.Sqrt(aOldMaxSqDist);

        Assert.Equal(aOldMaxDist, aNewMaxDist, theTol); // theTestName << ": Distance mismatch. New: " << aNewMaxDist << ", Old: " << aOldMaxDist
    }

    // Compares both min and max distances
    void CompareMinMax(ExtremaPC.Result theNew,
            Extrema_ExtPC theOld,
            string theTestName,
            double theTol = THE_TOL)
    {
        CompareMinDistance(theNew, theOld, theTestName, theTol);
        CompareMaxDistance(theNew, theOld, theTestName, theTol);
    }

    //==================================================================================================
    // Transformed geometry tests - Circle
    //==================================================================================================

    [Fact]
    public void Circle_Translated()
    {
        Geom_Circle aGeomCircle =
            new Geom_Circle(new gp_Ax2(new gp_Pnt(100, 200, 50), new gp_Dir(0, 0, 1)), 25.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomCircle);

        gp_Pnt aPoint = new(150.0, 200.0, 50.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_Translated");
    }

    [Fact]
    public void Circle_Rotated_XY_Plane()
    {
        // Circle in XY plane, rotated 45 degrees
        gp_Dir aNormal = new(1.0 / Math.Sqrt(2.0), 1.0 / Math.Sqrt(2.0), 0);
        Geom_Circle aGeomCircle = new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), aNormal), 15.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomCircle);

        gp_Pnt aPoint = new(20.0, 20.0, 5.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_Rotated_XY");
    }

    [Fact]
    public void Circle_Rotated_XZ_Plane()
    {
        // Circle in XZ plane
        Geom_Circle aGeomCircle =
            new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 0)), 10.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomCircle);

        gp_Pnt aPoint = new(15.0, 5.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_Rotated_XZ");
    }

    [Fact]
    public void Circle_Rotated_YZ_Plane()
    {
        // Circle in YZ plane
        Geom_Circle aGeomCircle =
            new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 10.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomCircle);

        gp_Pnt aPoint = new(5.0, 15.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_Rotated_YZ");
    }

    [Fact]
    public void Circle_Arbitrary_Orientation()
    {
        // Circle with arbitrary orientation
        gp_Dir aNormal = new(1, 1, 1);
        Geom_Circle aGeomCircle = new Geom_Circle(new gp_Ax2(new gp_Pnt(10, 20, 30), aNormal), 8.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomCircle);

        gp_Pnt aPoint = new(15.0, 25.0, 35.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_Arbitrary");
    }

    [Fact]
    public void Circle_VerySmall()
    {
        Geom_Circle aGeomCircle =
            new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 0.001);
        GeomAdaptor_Curve anAdaptor = new(aGeomCircle);

        gp_Pnt aPoint = new(0.002, 0.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_VerySmall", 1.0e-9);
    }

    [Fact]
    public void Circle_VeryLarge()
    {
        Geom_Circle aGeomCircle =
            new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1000.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomCircle);

        gp_Pnt aPoint = new(1500.0, 0.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_VeryLarge");
    }

    //==================================================================================================
    // Transformed geometry tests - Ellipse
    //==================================================================================================

    [Fact]
    public void Ellipse_Translated()
    {
        Geom_Ellipse aGeomEllipse =
            new Geom_Ellipse(new gp_Ax2(new gp_Pnt(50, 100, 25), new gp_Dir(0, 0, 1)), 30.0, 15.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomEllipse);

        gp_Pnt aPoint = new(90.0, 100.0, 25.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Ellipse_Translated");
    }

    [Fact]
    public void Ellipse_Rotated()
    {
        gp_Dir aNormal = new(0, 1, 0); // In XZ plane
        Geom_Ellipse aGeomEllipse =
            new Geom_Ellipse(new gp_Ax2(new gp_Pnt(0, 0, 0), aNormal), 20.0, 8.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomEllipse);

        gp_Pnt aPoint = new(25.0, 10.0, 5.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Ellipse_Rotated");
    }

    [Fact]
    public void Ellipse_HighEccentricity()
    {
        // Very elongated ellipse (major = 50, minor = 2)
        Geom_Ellipse aGeomEllipse =
            new Geom_Ellipse(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 50.0, 2.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomEllipse);

        gp_Pnt aPoint = new(40.0, 5.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Ellipse_HighEccentricity");
    }

    [Fact]
    public void Ellipse_NearlyCircular()
    {
        // Nearly circular ellipse (major = 10.1, minor = 10.0)
        Geom_Ellipse aGeomEllipse =
            new Geom_Ellipse(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.1, 10.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomEllipse);

        gp_Pnt aPoint = new(15.0, 8.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Ellipse_NearlyCircular");
    }

    //==================================================================================================
    // Transformed geometry tests - Line
    //==================================================================================================

    [Fact]
    public void Line_Diagonal()
    {
        Geom_Line aGeomLine = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 1, 1));
        GeomAdaptor_Curve anAdaptor = new(aGeomLine, -100.0, 100.0);

        gp_Pnt aPoint = new(10.0, 0.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Line_Diagonal");
    }

    [Fact]
    public void Line_Arbitrary()
    {
        Geom_Line aGeomLine = new Geom_Line(new gp_Pnt(10, 20, 30), new gp_Dir(2, 3, 1));
        GeomAdaptor_Curve anAdaptor = new(aGeomLine, -50.0, 50.0);

        gp_Pnt aPoint = new(25.0, 35.0, 40.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Line_Arbitrary");
    }

    [Fact]
    public void Line_PointOnLine()
    {
        Geom_Line aGeomLine = new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        GeomAdaptor_Curve anAdaptor = new(aGeomLine, -100.0, 100.0);

        gp_Pnt aPoint = new(50.0, 0.0, 0.0); // Exactly on line

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone());
        Assert.Equal(0.0, Math.Sqrt(aNewResult.MinSquareDistance()), THE_TOL);
    }

    //==================================================================================================
    // Parabola tests with different configurations
    //==================================================================================================

    [Fact]
    public void Parabola_Translated()
    {
        Geom_Parabola aGeomParabola =
            new Geom_Parabola(new gp_Ax2(new gp_Pnt(10, 20, 0), new gp_Dir(0, 0, 1)), 3.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomParabola, -15.0, 15.0);

        gp_Pnt aPoint = new(20.0, 30.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Parabola_Translated");
    }

    [Fact]
    public void Parabola_InXZPlane()
    {
        Geom_Parabola aGeomParabola =
            new Geom_Parabola(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 0)), 5.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomParabola, -10.0, 10.0);

        gp_Pnt aPoint = new(5.0, 5.0, 10.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Parabola_InXZPlane");
    }

    [Fact]
    public void Parabola_SmallFocalLength()
    {
        Geom_Parabola aGeomParabola =
            new Geom_Parabola(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 0.5);
        GeomAdaptor_Curve anAdaptor = new(aGeomParabola, -5.0, 5.0);

        gp_Pnt aPoint = new(2.0, 3.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Parabola_SmallFocalLength");
    }

    [Fact]
    public void Parabola_LargeFocalLength()
    {
        Geom_Parabola aGeomParabola =
            new Geom_Parabola(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 50.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomParabola, -100.0, 100.0);

        gp_Pnt aPoint = new(50.0, 25.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Parabola_LargeFocalLength");
    }

    //==================================================================================================
    // Hyperbola tests with different configurations
    //==================================================================================================

    [Fact]
    public void Hyperbola_Translated()
    {
        Geom_Hyperbola aGeomHyperbola =
            new Geom_Hyperbola(new gp_Ax2(new gp_Pnt(50, 50, 0), new gp_Dir(0, 0, 1)), 15.0, 8.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomHyperbola, -1.5, 1.5);

        gp_Pnt aPoint = new(80.0, 50.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Hyperbola_Translated");
    }

    [Fact]
    public void Hyperbola_InXZPlane()
    {
        Geom_Hyperbola aGeomHyperbola =
            new Geom_Hyperbola(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 0)), 10.0, 5.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomHyperbola, -2.0, 2.0);

        gp_Pnt aPoint = new(20.0, 5.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Hyperbola_InXZPlane");
    }

    [Fact]
    public void Hyperbola_HighEccentricity()
    {
        // Hyperbola with high eccentricity (a >> b)
        Geom_Hyperbola aGeomHyperbola =
            new Geom_Hyperbola(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 30.0, 3.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomHyperbola, -1.0, 1.0);

        gp_Pnt aPoint = new(50.0, 10.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aNewResult.IsDone());
        Assert.True(aNewResult.NbExt() > 0);

        // Verify that the minimum extremum is correct by checking
        // the computed point is actually on the curve
        int aMinIdx = aNewResult.MinIndex();
        gp_Pnt aMinPoint = aNewResult[aMinIdx].Point;

        // Verify the distance is consistent
        double aComputedSqDist = aPoint.SquareDistance(aMinPoint);
        Assert.Equal(aNewResult[aMinIdx].SquareDistance, aComputedSqDist, THE_TOL);
    }

    //==================================================================================================
    // BSpline with different configurations
    //==================================================================================================

    [Fact]
    public void BSpline_Linear()
    {
        // Degree 1 BSpline (linear segments)
        NCollection_Array1<gp_Pnt> aPoles = new(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(1, 1, 0);
        aPoles[3] = new gp_Pnt(2, 0, 0);
        aPoles[4] = new gp_Pnt(3, 1, 0);

        NCollection_Array1<double> aKnots = new(1, 4);
        aKnots[1] = 0.0;
        aKnots[2] = 0.33;
        aKnots[3] = 0.67;
        aKnots[4] = 1.0;

        NCollection_Array1<int> aMults = new(1, 4);
        aMults[1] = 2;
        aMults[2] = 1;
        aMults[3] = 1;
        aMults[4] = 2;

        Geom_BSplineCurve aBSpline = new Geom_BSplineCurve(aPoles, aKnots, aMults, 1);
        GeomAdaptor_Curve anAdaptor = new(aBSpline);

        gp_Pnt aPoint = new(1.5, 1.5, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "BSpline_Linear", 0.01);
    }

    [Fact]
    public void BSpline_Quadratic()
    {
        // Degree 2 BSpline
        NCollection_Array1<gp_Pnt> aPoles = new(1, 5);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(1, 2, 0);
        aPoles[3] = new gp_Pnt(2, 0, 0);
        aPoles[4] = new gp_Pnt(3, 2, 0);
        aPoles[5] = new gp_Pnt(4, 0, 0);

        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 0.5;
        aKnots[3] = 1.0;

        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 3;
        aMults[2] = 2;
        aMults[3] = 3;

        Geom_BSplineCurve aBSpline = new Geom_BSplineCurve(aPoles, aKnots, aMults, 2);
        GeomAdaptor_Curve anAdaptor = new(aBSpline);

        gp_Pnt aPoint = new(2.0, 2.5, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "BSpline_Quadratic", 0.01);
    }

    [Fact]
    public void BSpline_3D_Helix()
    {
        // 3D helix-like BSpline
        NCollection_Array1<gp_Pnt> aPoles = new(1, 7);
        aPoles[1] = new gp_Pnt(10, 0, 0);
        aPoles[2] = new gp_Pnt(7, 7, 5);
        aPoles[3] = new gp_Pnt(0, 10, 10);
        aPoles[4] = new gp_Pnt(-7, 7, 15);
        aPoles[5] = new gp_Pnt(-10, 0, 20);
        aPoles[6] = new gp_Pnt(-7, -7, 25);
        aPoles[7] = new gp_Pnt(0, -10, 30);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 7;
        aMults[2] = 7;

        Geom_BSplineCurve aBSpline = new Geom_BSplineCurve(aPoles, aKnots, aMults, 6);
        GeomAdaptor_Curve anAdaptor = new(aBSpline);

        gp_Pnt aPoint = new(0.0, 0.0, 15.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "BSpline_3D_Helix", 0.05);
    }

    //==================================================================================================
    // Bezier with different configurations
    //==================================================================================================

    [Fact]
    public void Bezier_Quadratic()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 3);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(5, 10, 0);
        aPoles[3] = new gp_Pnt(10, 0, 0);

        Geom_BezierCurve aBezier = new Geom_BezierCurve(aPoles);
        GeomAdaptor_Curve anAdaptor = new(aBezier);

        gp_Pnt aPoint = new(5.0, 8.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Bezier_Quadratic", 0.01);
    }

    [Fact]
    public void Bezier_HighDegree()
    {
        // Degree 6 Bezier
        NCollection_Array1<gp_Pnt> aPoles = new(1, 7);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(1, 3, 0);
        aPoles[3] = new gp_Pnt(2, -1, 0);
        aPoles[4] = new gp_Pnt(3, 4, 0);
        aPoles[5] = new gp_Pnt(4, 0, 0);
        aPoles[6] = new gp_Pnt(5, 2, 0);
        aPoles[7] = new gp_Pnt(6, 1, 0);

        Geom_BezierCurve aBezier = new Geom_BezierCurve(aPoles);
        GeomAdaptor_Curve anAdaptor = new(aBezier);

        gp_Pnt aPoint = new(3.0, 3.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Bezier_HighDegree", 0.01);
    }

    [Fact]
    public void Bezier_3D_SCurve()
    {
        // 3D S-curve
        NCollection_Array1<gp_Pnt> aPoles = new(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(3, 5, 2);
        aPoles[3] = new gp_Pnt(6, 0, 4);
        aPoles[4] = new gp_Pnt(9, 5, 6);

        Geom_BezierCurve aBezier = new Geom_BezierCurve(aPoles);
        GeomAdaptor_Curve anAdaptor = new(aBezier);

        gp_Pnt aPoint = new(4.5, 3.0, 3.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Bezier_3D_SCurve", 0.01);
    }

    //==================================================================================================
    // Offset curves with different bases
    //==================================================================================================

    [Fact]
    public void OffsetEllipse()
    {
        Geom_Ellipse aBaseEllipse =
            new Geom_Ellipse(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        Geom_OffsetCurve anOffsetCurve =
            new Geom_OffsetCurve(aBaseEllipse, 3.0, new gp_Dir(0, 0, 1));
        GeomAdaptor_Curve anAdaptor = new(anOffsetCurve);

        gp_Pnt aPoint = new(30.0, 5.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "OffsetEllipse", 0.05);
    }

    [Fact]
    public void OffsetBezier()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(2, 4, 0);
        aPoles[3] = new gp_Pnt(4, 4, 0);
        aPoles[4] = new gp_Pnt(6, 0, 0);

        Geom_BezierCurve aBaseBezier = new Geom_BezierCurve(aPoles);
        Geom_OffsetCurve anOffsetCurve =
            new Geom_OffsetCurve(aBaseBezier, 1.0, new gp_Dir(0, 0, 1));
        GeomAdaptor_Curve anAdaptor = new(anOffsetCurve);

        gp_Pnt aPoint = new(3.0, 5.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "OffsetBezier", 0.05);
    }

    [Fact]
    public void OffsetNegative()
    {
        // Offset with negative distance (inside)
        Geom_Circle aBaseCircle =
            new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 15.0);
        Geom_OffsetCurve anOffsetCurve =
            new Geom_OffsetCurve(aBaseCircle, -5.0, new gp_Dir(0, 0, 1));
        GeomAdaptor_Curve anAdaptor = new(anOffsetCurve);

        gp_Pnt aPoint = new(20.0, 0.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "OffsetNegative", 0.01);
        Assert.Equal(10.0, Math.Sqrt(aNewResult.MinSquareDistance()), 0.01); // 20 - 10 = 10
    }

    //==================================================================================================
    // Arc (partial curve) tests
    //==================================================================================================

    [Fact]
    public void CircleArc_FirstQuadrant()
    {
        Geom_Circle aGeomCircle =
            new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomCircle, 0.0, M_PI / 2.0);

        gp_Pnt aPoint = new(15.0, 15.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        // Only compare minimum - max may differ between implementations for arcs
        // due to endpoint handling
        CompareMinDistance(aNewResult, anOldExtPC, "CircleArc_FirstQuadrant");
    }

    [Fact]
    public void CircleArc_ThirdQuadrant()
    {
        Geom_Circle aGeomCircle =
            new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomCircle, M_PI, 3.0 * M_PI / 2.0);

        gp_Pnt aPoint = new(-15.0, -5.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        // Only compare minimum - max may differ between implementations for arcs
        // due to endpoint handling
        CompareMinDistance(aNewResult, anOldExtPC, "CircleArc_ThirdQuadrant");
    }

    [Fact]
    public void EllipseArc_SemiEllipse()
    {
        Geom_Ellipse aGeomEllipse =
            new Geom_Ellipse(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomEllipse, 0.0, M_PI);

        gp_Pnt aPoint = new(0.0, 20.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        // Only compare minimum - max may differ between implementations for arcs
        CompareMinDistance(aNewResult, anOldExtPC, "EllipseArc_SemiEllipse");
    }

    //==================================================================================================
    // Edge cases and degenerate situations
    //==================================================================================================

    [Fact]
    public void Point_ExactlyOnCurve_Circle()
    {
        Geom_Circle aGeomCircle =
            new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomCircle);

        gp_Pnt aPoint = new(10.0, 0.0, 0.0); // Exactly on circle

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aNewResult.IsDone());
        Assert.Equal(0.0, Math.Sqrt(aNewResult.MinSquareDistance()), THE_TOL);
    }

    [Fact]
    public void Point_ExactlyOnCurve_BSpline()
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(1, 2, 0);
        aPoles[3] = new gp_Pnt(3, 2, 0);
        aPoles[4] = new gp_Pnt(4, 0, 0);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        Geom_BSplineCurve aBSpline = new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
        GeomAdaptor_Curve anAdaptor = new(aBSpline);

        // Point exactly on curve at t=0.5
        gp_Pnt aPoint = anAdaptor.Value(0.5);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aNewResult.IsDone());
        Assert.Equal(0.0, Math.Sqrt(aNewResult.MinSquareDistance()), 0.01);
    }

    [Fact]
    public void Point_VeryFar()
    {
        Geom_Circle aGeomCircle =
            new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomCircle);

        gp_Pnt aPoint = new(10000.0, 10000.0, 0.0);

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Point_VeryFar");
    }

    [Fact]
    public void Point_VeryClose()
    {
        Geom_Circle aGeomCircle =
            new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomCircle);

        gp_Pnt aPoint = new(10.0001, 0.0, 0.0); // Very close to curve

        ExtremaPC_Curve anExtPC = new(anAdaptor);
        ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone());
        Assert.Equal(0.0001, Math.Sqrt(aNewResult.MinSquareDistance()), 1.0e-5);
    }

    //==================================================================================================
    // Random stress tests for all geometry types
    //==================================================================================================

    [Fact]
    public void Random_Circle_100Points()
    {
        Geom_Circle aGeomCircle =
            new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomCircle);

        Mt19937 aGen = new(12345); // std::mt19937

        int aPassCount = 0;
        for (int i = 0; i < 100; ++i)
        {
            gp_Pnt aPoint = new(aGen.NextReal(-100.0, 100.0), aGen.NextReal(-100.0, 100.0), aGen.NextReal(-100.0, 100.0));

            ExtremaPC_Curve anExtPC = new(anAdaptor);
            ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

            Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

            if (!anOldExtPC.IsDone())
            {
                continue;
            }

            if (aNewResult.IsDone() && aNewResult.NbExt() > 0)
            {
                double aNewMin = Math.Sqrt(aNewResult.MinSquareDistance());
                double aOldMinSq = RealLast();
                for (int j = 1; j <= anOldExtPC.NbExt(); ++j)
                {
                    if (anOldExtPC.SquareDistance(j) < aOldMinSq)
                    {
                        aOldMinSq = anOldExtPC.SquareDistance(j);
                    }
                }
                double aOldMin = Math.Sqrt(aOldMinSq);
                if (Math.Abs(aNewMin - aOldMin) < THE_TOL)
                {
                    ++aPassCount;
                }
            }
        }

        Assert.True(aPassCount >= 95); // "At least 95% of random tests should pass"
    }

    [Fact]
    public void Random_Ellipse_100Points()
    {
        Geom_Ellipse aGeomEllipse =
            new Geom_Ellipse(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 10.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomEllipse);

        Mt19937 aGen = new(54321); // std::mt19937

        int aPassCount = 0;
        for (int i = 0; i < 100; ++i)
        {
            gp_Pnt aPoint = new(aGen.NextReal(-100.0, 100.0), aGen.NextReal(-100.0, 100.0), aGen.NextReal(-100.0, 100.0));

            ExtremaPC_Curve anExtPC = new(anAdaptor);
            ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

            Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

            if (!anOldExtPC.IsDone())
            {
                continue;
            }

            if (aNewResult.IsDone() && aNewResult.NbExt() > 0)
            {
                double aNewMin = Math.Sqrt(aNewResult.MinSquareDistance());
                double aOldMinSq = RealLast();
                for (int j = 1; j <= anOldExtPC.NbExt(); ++j)
                {
                    if (anOldExtPC.SquareDistance(j) < aOldMinSq)
                    {
                        aOldMinSq = anOldExtPC.SquareDistance(j);
                    }
                }
                double aOldMin = Math.Sqrt(aOldMinSq);
                if (Math.Abs(aNewMin - aOldMin) < THE_TOL)
                {
                    ++aPassCount;
                }
            }
        }

        Assert.True(aPassCount >= 95); // "At least 95% of random tests should pass"
    }

    [Fact]
    public void Random_Parabola_100Points()
    {
        Geom_Parabola aGeomParabola =
            new Geom_Parabola(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 5.0);
        GeomAdaptor_Curve anAdaptor = new(aGeomParabola, -20.0, 20.0);

        Mt19937 aGen = new(11111); // std::mt19937

        int aPassCount = 0;
        int aValidCount = 0;
        for (int i = 0; i < 100; ++i)
        {
            gp_Pnt aPoint = new(aGen.NextReal(-50.0, 50.0), aGen.NextReal(-50.0, 50.0), aGen.NextReal(-50.0, 50.0));

            ExtremaPC_Curve anExtPC = new(anAdaptor);
            ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

            Extrema_ExtPC anOldExtPC = new(aPoint, anAdaptor);

            if (!anOldExtPC.IsDone() || anOldExtPC.NbExt() == 0)
            {
                continue;
            }

            if (!aNewResult.IsDone() || aNewResult.NbExt() == 0)
            {
                continue;
            }

            ++aValidCount;

            double aNewMin = Math.Sqrt(aNewResult.MinSquareDistance());
            double aOldMin = RealLast();
            for (int j = 1; j <= anOldExtPC.NbExt(); ++j)
            {
                if (anOldExtPC.SquareDistance(j) < aOldMin * aOldMin)
                {
                    aOldMin = Math.Sqrt(anOldExtPC.SquareDistance(j));
                }
            }
            // Use relaxed tolerance for random tests
            if (Math.Abs(aNewMin - aOldMin) < 0.01)
            {
                ++aPassCount;
            }
        }

        // At least 80% of valid random tests should pass
        if (aValidCount > 0)
        {
            double aPassRate = (double)aPassCount / aValidCount;
            Assert.True(aPassRate >= 0.80); // "At least 80% of random tests should pass"
        }
    }
}
