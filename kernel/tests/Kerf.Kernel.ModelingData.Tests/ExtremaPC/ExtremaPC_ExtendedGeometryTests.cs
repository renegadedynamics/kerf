// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_ExtendedGeometry_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class ExtremaPC_ExtendedGeometryTest
{
    private const double THE_TOL = 1.0e-6;

    //==================================================================================================
    // Test fixture for extended geometry tests
    //==================================================================================================

    // Compares minimum distance between new and old implementations
    void CompareMinDistance(global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result theNew,
            CurvePointExtrema theOld,
            string theTestName,
            double theTol = THE_TOL)
    {
        if (!theOld.IsDone || theOld.ExtCount == 0)
        {
            return;
        }

        Assert.True(theNew.IsDone); // theTestName << ": New implementation failed"
        Assert.True(theNew.ExtCount > 0); // theTestName << ": No extrema found"

        double aOldMinSqDist = RealLast();
        for (int i = 1; i <= theOld.ExtCount; ++i)
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
    void CompareMaxDistance(global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result theNew,
            CurvePointExtrema theOld,
            string theTestName,
            double theTol = THE_TOL)
    {
        if (!theOld.IsDone || theOld.ExtCount == 0)
        {
            return;
        }

        Assert.True(theNew.IsDone); // theTestName << ": New implementation failed"
        Assert.True(theNew.ExtCount > 0); // theTestName << ": No extrema found"

        double aOldMaxSqDist = 0.0;
        for (int i = 1; i <= theOld.ExtCount; ++i)
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
    void CompareMinMax(global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result theNew,
            CurvePointExtrema theOld,
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
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(100, 200, 50), new Direction3D(0, 0, 1)), 25.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(150.0, 200.0, 50.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_Translated");
    }

    [Fact]
    public void Circle_Rotated_XY_Plane()
    {
        // Circle in XY plane, rotated 45 degrees
        Direction3D aNormal = new(1.0 / Math.Sqrt(2.0), 1.0 / Math.Sqrt(2.0), 0);
        CircleCurve3D aGeomCircle = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), aNormal), 15.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(20.0, 20.0, 5.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_Rotated_XY");
    }

    [Fact]
    public void Circle_Rotated_XZ_Plane()
    {
        // Circle in XZ plane
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(15.0, 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_Rotated_XZ");
    }

    [Fact]
    public void Circle_Rotated_YZ_Plane()
    {
        // Circle in YZ plane
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(5.0, 15.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_Rotated_YZ");
    }

    [Fact]
    public void Circle_Arbitrary_Orientation()
    {
        // Circle with arbitrary orientation
        Direction3D aNormal = new(1, 1, 1);
        CircleCurve3D aGeomCircle = new CircleCurve3D(new Frame3D(new Point3D(10, 20, 30), aNormal), 8.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(15.0, 25.0, 35.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_Arbitrary");
    }

    [Fact]
    public void Circle_VerySmall()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0.001);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(0.002, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_VerySmall", 1.0e-9);
    }

    [Fact]
    public void Circle_VeryLarge()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1000.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(1500.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Circle_VeryLarge");
    }

    //==================================================================================================
    // Transformed geometry tests - Ellipse
    //==================================================================================================

    [Fact]
    public void Ellipse_Translated()
    {
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(50, 100, 25), new Direction3D(0, 0, 1)), 30.0, 15.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse);

        Point3D aPoint = new(90.0, 100.0, 25.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Ellipse_Translated");
    }

    [Fact]
    public void Ellipse_Rotated()
    {
        Direction3D aNormal = new(0, 1, 0); // In XZ plane
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), aNormal), 20.0, 8.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse);

        Point3D aPoint = new(25.0, 10.0, 5.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Ellipse_Rotated");
    }

    [Fact]
    public void Ellipse_HighEccentricity()
    {
        // Very elongated ellipse (major = 50, minor = 2)
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 50.0, 2.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse);

        Point3D aPoint = new(40.0, 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Ellipse_HighEccentricity");
    }

    [Fact]
    public void Ellipse_NearlyCircular()
    {
        // Nearly circular ellipse (major = 10.1, minor = 10.0)
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.1, 10.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse);

        Point3D aPoint = new(15.0, 8.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Ellipse_NearlyCircular");
    }

    //==================================================================================================
    // Transformed geometry tests - Line
    //==================================================================================================

    [Fact]
    public void Line_Diagonal()
    {
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 1, 1));
        CurveAdaptor anAdaptor = new(aGeomLine, -100.0, 100.0);

        Point3D aPoint = new(10.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Line_Diagonal");
    }

    [Fact]
    public void Line_Arbitrary()
    {
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(10, 20, 30), new Direction3D(2, 3, 1));
        CurveAdaptor anAdaptor = new(aGeomLine, -50.0, 50.0);

        Point3D aPoint = new(25.0, 35.0, 40.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Line_Arbitrary");
    }

    [Fact]
    public void Line_PointOnLine()
    {
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        CurveAdaptor anAdaptor = new(aGeomLine, -100.0, 100.0);

        Point3D aPoint = new(50.0, 0.0, 0.0); // Exactly on line

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);
        Assert.Equal(0.0, Math.Sqrt(aNewResult.MinSquareDistance()), THE_TOL);
    }

    //==================================================================================================
    // Parabola tests with different configurations
    //==================================================================================================

    [Fact]
    public void Parabola_Translated()
    {
        ParabolaCurve3D aGeomParabola =
            new ParabolaCurve3D(new Frame3D(new Point3D(10, 20, 0), new Direction3D(0, 0, 1)), 3.0);
        CurveAdaptor anAdaptor = new(aGeomParabola, -15.0, 15.0);

        Point3D aPoint = new(20.0, 30.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Parabola_Translated");
    }

    [Fact]
    public void Parabola_InXZPlane()
    {
        ParabolaCurve3D aGeomParabola =
            new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0)), 5.0);
        CurveAdaptor anAdaptor = new(aGeomParabola, -10.0, 10.0);

        Point3D aPoint = new(5.0, 5.0, 10.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Parabola_InXZPlane");
    }

    [Fact]
    public void Parabola_SmallFocalLength()
    {
        ParabolaCurve3D aGeomParabola =
            new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0.5);
        CurveAdaptor anAdaptor = new(aGeomParabola, -5.0, 5.0);

        Point3D aPoint = new(2.0, 3.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Parabola_SmallFocalLength");
    }

    [Fact]
    public void Parabola_LargeFocalLength()
    {
        ParabolaCurve3D aGeomParabola =
            new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 50.0);
        CurveAdaptor anAdaptor = new(aGeomParabola, -100.0, 100.0);

        Point3D aPoint = new(50.0, 25.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Parabola_LargeFocalLength");
    }

    //==================================================================================================
    // Hyperbola tests with different configurations
    //==================================================================================================

    [Fact]
    public void Hyperbola_Translated()
    {
        HyperbolaCurve3D aGeomHyperbola =
            new HyperbolaCurve3D(new Frame3D(new Point3D(50, 50, 0), new Direction3D(0, 0, 1)), 15.0, 8.0);
        CurveAdaptor anAdaptor = new(aGeomHyperbola, -1.5, 1.5);

        Point3D aPoint = new(80.0, 50.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Hyperbola_Translated");
    }

    [Fact]
    public void Hyperbola_InXZPlane()
    {
        HyperbolaCurve3D aGeomHyperbola =
            new HyperbolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0)), 10.0, 5.0);
        CurveAdaptor anAdaptor = new(aGeomHyperbola, -2.0, 2.0);

        Point3D aPoint = new(20.0, 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Hyperbola_InXZPlane");
    }

    [Fact]
    public void Hyperbola_HighEccentricity()
    {
        // Hyperbola with high eccentricity (a >> b)
        HyperbolaCurve3D aGeomHyperbola =
            new HyperbolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 30.0, 3.0);
        CurveAdaptor anAdaptor = new(aGeomHyperbola, -1.0, 1.0);

        Point3D aPoint = new(50.0, 10.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aNewResult.IsDone);
        Assert.True(aNewResult.ExtCount > 0);

        // Verify that the minimum extremum is correct by checking
        // the computed point is actually on the curve
        int aMinIdx = aNewResult.MinIndex();
        Point3D aMinPoint = aNewResult[aMinIdx].Point;

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
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 1, 0);
        aPoles[3] = new Point3D(2, 0, 0);
        aPoles[4] = new Point3D(3, 1, 0);

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

        BSplineCurve3D aBSpline = new BSplineCurve3D(aPoles, aKnots, aMults, 1);
        CurveAdaptor anAdaptor = new(aBSpline);

        Point3D aPoint = new(1.5, 1.5, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "BSpline_Linear", 0.01);
    }

    [Fact]
    public void BSpline_Quadratic()
    {
        // Degree 2 BSpline
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 0);
        aPoles[3] = new Point3D(2, 0, 0);
        aPoles[4] = new Point3D(3, 2, 0);
        aPoles[5] = new Point3D(4, 0, 0);

        NCollection_Array1<double> aKnots = new(1, 3);
        aKnots[1] = 0.0;
        aKnots[2] = 0.5;
        aKnots[3] = 1.0;

        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 3;
        aMults[2] = 2;
        aMults[3] = 3;

        BSplineCurve3D aBSpline = new BSplineCurve3D(aPoles, aKnots, aMults, 2);
        CurveAdaptor anAdaptor = new(aBSpline);

        Point3D aPoint = new(2.0, 2.5, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "BSpline_Quadratic", 0.01);
    }

    [Fact]
    public void BSpline_3D_Helix()
    {
        // 3D helix-like BSpline
        NCollection_Array1<Point3D> aPoles = new(1, 7);
        aPoles[1] = new Point3D(10, 0, 0);
        aPoles[2] = new Point3D(7, 7, 5);
        aPoles[3] = new Point3D(0, 10, 10);
        aPoles[4] = new Point3D(-7, 7, 15);
        aPoles[5] = new Point3D(-10, 0, 20);
        aPoles[6] = new Point3D(-7, -7, 25);
        aPoles[7] = new Point3D(0, -10, 30);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 7;
        aMults[2] = 7;

        BSplineCurve3D aBSpline = new BSplineCurve3D(aPoles, aKnots, aMults, 6);
        CurveAdaptor anAdaptor = new(aBSpline);

        Point3D aPoint = new(0.0, 0.0, 15.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "BSpline_3D_Helix", 0.05);
    }

    //==================================================================================================
    // Bezier with different configurations
    //==================================================================================================

    [Fact]
    public void Bezier_Quadratic()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(5, 10, 0);
        aPoles[3] = new Point3D(10, 0, 0);

        BezierCurve3D aBezier = new BezierCurve3D(aPoles);
        CurveAdaptor anAdaptor = new(aBezier);

        Point3D aPoint = new(5.0, 8.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Bezier_Quadratic", 0.01);
    }

    [Fact]
    public void Bezier_HighDegree()
    {
        // Degree 6 Bezier
        NCollection_Array1<Point3D> aPoles = new(1, 7);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 3, 0);
        aPoles[3] = new Point3D(2, -1, 0);
        aPoles[4] = new Point3D(3, 4, 0);
        aPoles[5] = new Point3D(4, 0, 0);
        aPoles[6] = new Point3D(5, 2, 0);
        aPoles[7] = new Point3D(6, 1, 0);

        BezierCurve3D aBezier = new BezierCurve3D(aPoles);
        CurveAdaptor anAdaptor = new(aBezier);

        Point3D aPoint = new(3.0, 3.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Bezier_HighDegree", 0.01);
    }

    [Fact]
    public void Bezier_3D_SCurve()
    {
        // 3D S-curve
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(3, 5, 2);
        aPoles[3] = new Point3D(6, 0, 4);
        aPoles[4] = new Point3D(9, 5, 6);

        BezierCurve3D aBezier = new BezierCurve3D(aPoles);
        CurveAdaptor anAdaptor = new(aBezier);

        Point3D aPoint = new(4.5, 3.0, 3.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "Bezier_3D_SCurve", 0.01);
    }

    //==================================================================================================
    // Offset curves with different bases
    //==================================================================================================

    [Fact]
    public void OffsetEllipse()
    {
        EllipseCurve3D aBaseEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        OffsetCurve3D anOffsetCurve =
            new OffsetCurve3D(aBaseEllipse, 3.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        Point3D aPoint = new(30.0, 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "OffsetEllipse", 0.05);
    }

    [Fact]
    public void OffsetBezier()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(2, 4, 0);
        aPoles[3] = new Point3D(4, 4, 0);
        aPoles[4] = new Point3D(6, 0, 0);

        BezierCurve3D aBaseBezier = new BezierCurve3D(aPoles);
        OffsetCurve3D anOffsetCurve =
            new OffsetCurve3D(aBaseBezier, 1.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        Point3D aPoint = new(3.0, 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "OffsetBezier", 0.05);
    }

    [Fact]
    public void OffsetNegative()
    {
        // Offset with negative distance (inside)
        CircleCurve3D aBaseCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 15.0);
        OffsetCurve3D anOffsetCurve =
            new OffsetCurve3D(aBaseCircle, -5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinDistance(aNewResult, anOldExtPC, "OffsetNegative", 0.01);
        Assert.Equal(10.0, Math.Sqrt(aNewResult.MinSquareDistance()), 0.01); // 20 - 10 = 10
    }

    //==================================================================================================
    // Arc (partial curve) tests
    //==================================================================================================

    [Fact]
    public void CircleArc_FirstQuadrant()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle, 0.0, M_PI / 2.0);

        Point3D aPoint = new(15.0, 15.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        // Only compare minimum - max may differ between implementations for arcs
        // due to endpoint handling
        CompareMinDistance(aNewResult, anOldExtPC, "CircleArc_FirstQuadrant");
    }

    [Fact]
    public void CircleArc_ThirdQuadrant()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle, M_PI, 3.0 * M_PI / 2.0);

        Point3D aPoint = new(-15.0, -5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        // Only compare minimum - max may differ between implementations for arcs
        // due to endpoint handling
        CompareMinDistance(aNewResult, anOldExtPC, "CircleArc_ThirdQuadrant");
    }

    [Fact]
    public void EllipseArc_SemiEllipse()
    {
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse, 0.0, M_PI);

        Point3D aPoint = new(0.0, 20.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        // Only compare minimum - max may differ between implementations for arcs
        CompareMinDistance(aNewResult, anOldExtPC, "EllipseArc_SemiEllipse");
    }

    //==================================================================================================
    // Edge cases and degenerate situations
    //==================================================================================================

    [Fact]
    public void Point_ExactlyOnCurve_Circle()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(10.0, 0.0, 0.0); // Exactly on circle

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aNewResult.IsDone);
        Assert.Equal(0.0, Math.Sqrt(aNewResult.MinSquareDistance()), THE_TOL);
    }

    [Fact]
    public void Point_ExactlyOnCurve_BSpline()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 0);
        aPoles[3] = new Point3D(3, 2, 0);
        aPoles[4] = new Point3D(4, 0, 0);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        BSplineCurve3D aBSpline = new BSplineCurve3D(aPoles, aKnots, aMults, 3);
        CurveAdaptor anAdaptor = new(aBSpline);

        // Point exactly on curve at t=0.5
        Point3D aPoint = anAdaptor.Value(0.5);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        Assert.True(aNewResult.IsDone);
        Assert.Equal(0.0, Math.Sqrt(aNewResult.MinSquareDistance()), 0.01);
    }

    [Fact]
    public void Point_VeryFar()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(10000.0, 10000.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        CompareMinMax(aNewResult, anOldExtPC, "Point_VeryFar");
    }

    [Fact]
    public void Point_VeryClose()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(10.0001, 0.0, 0.0); // Very close to curve

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);
        Assert.Equal(0.0001, Math.Sqrt(aNewResult.MinSquareDistance()), 1.0e-5);
    }

    //==================================================================================================
    // Random stress tests for all geometry types
    //==================================================================================================

    [Fact]
    public void Random_Circle_100Points()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Mt19937 aGen = new(12345); // std::mt19937

        int aPassCount = 0;
        for (int i = 0; i < 100; ++i)
        {
            Point3D aPoint = new(aGen.NextReal(-100.0, 100.0), aGen.NextReal(-100.0, 100.0), aGen.NextReal(-100.0, 100.0));

            global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

            CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

            if (!anOldExtPC.IsDone)
            {
                continue;
            }

            if (aNewResult.IsDone && aNewResult.ExtCount > 0)
            {
                double aNewMin = Math.Sqrt(aNewResult.MinSquareDistance());
                double aOldMinSq = RealLast();
                for (int j = 1; j <= anOldExtPC.ExtCount; ++j)
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
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse);

        Mt19937 aGen = new(54321); // std::mt19937

        int aPassCount = 0;
        for (int i = 0; i < 100; ++i)
        {
            Point3D aPoint = new(aGen.NextReal(-100.0, 100.0), aGen.NextReal(-100.0, 100.0), aGen.NextReal(-100.0, 100.0));

            global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

            CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

            if (!anOldExtPC.IsDone)
            {
                continue;
            }

            if (aNewResult.IsDone && aNewResult.ExtCount > 0)
            {
                double aNewMin = Math.Sqrt(aNewResult.MinSquareDistance());
                double aOldMinSq = RealLast();
                for (int j = 1; j <= anOldExtPC.ExtCount; ++j)
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
        ParabolaCurve3D aGeomParabola =
            new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0);
        CurveAdaptor anAdaptor = new(aGeomParabola, -20.0, 20.0);

        Mt19937 aGen = new(11111); // std::mt19937

        int aPassCount = 0;
        int aValidCount = 0;
        for (int i = 0; i < 100; ++i)
        {
            Point3D aPoint = new(aGen.NextReal(-50.0, 50.0), aGen.NextReal(-50.0, 50.0), aGen.NextReal(-50.0, 50.0));

            global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult = anExtPC.Perform(aPoint, THE_TOL);

            CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

            if (!anOldExtPC.IsDone || anOldExtPC.ExtCount == 0)
            {
                continue;
            }

            if (!aNewResult.IsDone || aNewResult.ExtCount == 0)
            {
                continue;
            }

            ++aValidCount;

            double aNewMin = Math.Sqrt(aNewResult.MinSquareDistance());
            double aOldMin = RealLast();
            for (int j = 1; j <= anOldExtPC.ExtCount; ++j)
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
