// Port of OCCT's src/ModelingData/TKGeomBase/GTests/ExtremaPC_SearchMode_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class ExtremaPC_SearchModeTest
{
    private const double THE_TOL = 1.0e-6;

    //==================================================================================================
    // Test fixture for SearchMode tests
    //==================================================================================================

    // Gets minimum distance from old Extrema_ExtPC
    double GetOldMinDistance(CurvePointExtrema theOld)
    {
        if (!theOld.IsDone || theOld.ExtCount == 0)
        {
            return RealLast();
        }
        double aMinSqDist = RealLast();
        for (int i = 1; i <= theOld.ExtCount; ++i)
        {
            if (theOld.SquareDistance(i) < aMinSqDist)
            {
                aMinSqDist = theOld.SquareDistance(i);
            }
        }
        return Math.Sqrt(aMinSqDist);
    }

    // Gets maximum distance from old Extrema_ExtPC
    double GetOldMaxDistance(CurvePointExtrema theOld)
    {
        if (!theOld.IsDone || theOld.ExtCount == 0)
        {
            return 0.0;
        }
        double aMaxSqDist = 0.0;
        for (int i = 1; i <= theOld.ExtCount; ++i)
        {
            if (theOld.SquareDistance(i) > aMaxSqDist)
            {
                aMaxSqDist = theOld.SquareDistance(i);
            }
        }
        return Math.Sqrt(aMaxSqDist);
    }

    // Gets minimum parameter from old Extrema_ExtPC
    double GetOldMinParameter(CurvePointExtrema theOld)
    {
        if (!theOld.IsDone || theOld.ExtCount == 0)
        {
            return 0.0;
        }
        double aMinSqDist = RealLast();
        int aMinIdx = 1;
        for (int i = 1; i <= theOld.ExtCount; ++i)
        {
            if (theOld.SquareDistance(i) < aMinSqDist)
            {
                aMinSqDist = theOld.SquareDistance(i);
                aMinIdx = i;
            }
        }
        return theOld.Point(aMinIdx).Parameter();
    }

    // Gets maximum parameter from old Extrema_ExtPC
    double GetOldMaxParameter(CurvePointExtrema theOld)
    {
        if (!theOld.IsDone || theOld.ExtCount == 0)
        {
            return 0.0;
        }
        double aMaxSqDist = 0.0;
        int aMaxIdx = 1;
        for (int i = 1; i <= theOld.ExtCount; ++i)
        {
            if (theOld.SquareDistance(i) > aMaxSqDist)
            {
                aMaxSqDist = theOld.SquareDistance(i);
                aMaxIdx = i;
            }
        }
        return theOld.Point(aMaxIdx).Parameter();
    }

    //==================================================================================================
    // Line SearchMode tests
    //==================================================================================================

    [Fact]
    public void Line_MinMode()
    {
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        CurveAdaptor anAdaptor = new(aGeomLine, -100.0, 100.0);

        Point3D aPoint = new(25.0, 10.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);
        Assert.Equal(1, aNewResult.ExtCount); // Min mode should return only 1 result

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);

        Assert.Equal(aOldMinDist, aNewMinDist, THE_TOL);
        Assert.Equal(10.0, aNewMinDist, THE_TOL); // Distance to line is 10
    }

    [Fact]
    public void Line_MaxMode()
    {
        // For bounded lines, Max mode with endpoints includes endpoint maxima.
        // The maximum distance is at the farthest endpoint, not the perpendicular projection.
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        CurveAdaptor anAdaptor = new(aGeomLine, -50.0, 50.0);

        Point3D aPoint = new(0.0, 10.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.PerformWithEndpoints(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

        Assert.True(aNewResult.IsDone);
        // With endpoints included, max mode finds the farthest endpoint
        Assert.True(aNewResult.ExtCount >= 1);

        // Maximum distance is to endpoint at (-50, 0, 0) or (50, 0, 0)
        // Distance = sqrt(50^2 + 10^2) = sqrt(2600) ~= 50.99
        double aExpectedMaxDist = Math.Sqrt(2600.0);
        double aNewMaxDist = Math.Sqrt(aNewResult.MaxSquareDistance());
        Assert.Equal(aExpectedMaxDist, aNewMaxDist, THE_TOL);
    }

    [Fact]
    public void Line_MinMaxMode()
    {
        LineCurve3D aGeomLine = new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        CurveAdaptor anAdaptor = new(aGeomLine, -50.0, 50.0);

        Point3D aPoint = new(10.0, 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);
        // MinMax mode should find all extrema (min + both endpoints for bounded line)

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);

        Assert.Equal(aOldMinDist, aNewMinDist, THE_TOL);
        Assert.Equal(5.0, aNewMinDist, THE_TOL);
    }

    //==================================================================================================
    // Circle SearchMode tests
    //==================================================================================================

    [Fact]
    public void Circle_MinMode_PointOutside()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);
        Assert.Equal(1, aNewResult.ExtCount);

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);

        Assert.Equal(aOldMinDist, aNewMinDist, THE_TOL);
        Assert.Equal(10.0, aNewMinDist, THE_TOL); // 20 - 10 = 10
    }

    [Fact]
    public void Circle_MaxMode_PointOutside()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);
        Assert.Equal(1, aNewResult.ExtCount);

        double aNewMaxDist = Math.Sqrt(aNewResult.MaxSquareDistance());
        double aOldMaxDist = GetOldMaxDistance(anOldExtPC);

        Assert.Equal(aOldMaxDist, aNewMaxDist, THE_TOL);
        Assert.Equal(30.0, aNewMaxDist, THE_TOL); // 20 + 10 = 30
    }

    [Fact]
    public void Circle_MinMode_PointInside()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(3.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        Assert.True(aNewResult.IsDone);
        Assert.Equal(1, aNewResult.ExtCount);

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        Assert.Equal(7.0, aNewMinDist, THE_TOL); // 10 - 3 = 7
    }

    [Fact]
    public void Circle_MaxMode_PointInside()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(3.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

        Assert.True(aNewResult.IsDone);
        Assert.Equal(1, aNewResult.ExtCount);

        double aNewMaxDist = Math.Sqrt(aNewResult.MaxSquareDistance());
        Assert.Equal(13.0, aNewMaxDist, THE_TOL); // 10 + 3 = 13
    }

    [Fact]
    public void Circle_MinMaxMode()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Point3D aPoint = new(15.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);
        Assert.Equal(2, aNewResult.ExtCount); // Circle has 2 extrema: min and max

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aNewMaxDist = Math.Sqrt(aNewResult.MaxSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);
        double aOldMaxDist = GetOldMaxDistance(anOldExtPC);

        Assert.Equal(aOldMinDist, aNewMinDist, THE_TOL);
        Assert.Equal(aOldMaxDist, aNewMaxDist, THE_TOL);
        Assert.Equal(5.0, aNewMinDist, THE_TOL);
        Assert.Equal(25.0, aNewMaxDist, THE_TOL);
    }

    [Fact]
    public void Circle_PartialArc_MinMode()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        // Only first quadrant: 0 to PI/2
        CurveAdaptor anAdaptor = new(aGeomCircle, 0.0, M_PI / 2.0);

        Point3D aPoint = new(15.0, 15.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);

        Assert.Equal(aOldMinDist, aNewMinDist, THE_TOL);
    }

    [Fact]
    public void Circle_PartialArc_MaxMode()
    {
        // NOTE: Max mode for partial arcs may not find extrema if the max is at an
        // endpoint (which is not a true extremum in the mathematical sense).
        // This test verifies the implementation completes successfully.
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        // Only first quadrant: 0 to PI/2
        CurveAdaptor anAdaptor = new(aGeomCircle, 0.0, M_PI / 2.0);

        Point3D aPoint = new(15.0, 15.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

        Assert.True(aNewResult.IsDone);

        // If extrema were found, verify they are valid
        if (aNewResult.ExtCount > 0)
        {
            var aExtremum = aNewResult[aNewResult.MaxIndex()];
            // Parameter should be within bounds
            Assert.True(aExtremum.Parameter >= 0.0 - THE_TOL);
            Assert.True(aExtremum.Parameter <= M_PI / 2.0 + THE_TOL);
            // Point should be on the curve
            Point3D aCurvePnt = anAdaptor.Value(aExtremum.Parameter);
            Assert.Equal(0.0, aCurvePnt.Distance(aExtremum.Point), THE_TOL);
        }
    }

    //==================================================================================================
    // Ellipse SearchMode tests
    //==================================================================================================

    [Fact]
    public void Ellipse_MinMode_OnMajorAxis()
    {
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse);

        Point3D aPoint = new(30.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);
        Assert.Equal(1, aNewResult.ExtCount);

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);

        Assert.Equal(aOldMinDist, aNewMinDist, THE_TOL);
        Assert.Equal(10.0, aNewMinDist, THE_TOL); // 30 - 20 = 10
    }

    [Fact]
    public void Ellipse_MaxMode_OnMajorAxis()
    {
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse);

        Point3D aPoint = new(30.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);
        Assert.Equal(1, aNewResult.ExtCount);

        double aNewMaxDist = Math.Sqrt(aNewResult.MaxSquareDistance());
        double aOldMaxDist = GetOldMaxDistance(anOldExtPC);

        Assert.Equal(aOldMaxDist, aNewMaxDist, THE_TOL);
        Assert.Equal(50.0, aNewMaxDist, THE_TOL); // 30 + 20 = 50
    }

    [Fact]
    public void Ellipse_MinMode_General()
    {
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse);

        Point3D aPoint = new(15.0, 12.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);

        Assert.Equal(aOldMinDist, aNewMinDist, THE_TOL);
    }

    [Fact]
    public void Ellipse_MinMaxMode()
    {
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse);

        Point3D aPoint = new(15.0, 8.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);
        // Ellipse should have 4 extrema for general point position

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aNewMaxDist = Math.Sqrt(aNewResult.MaxSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);
        double aOldMaxDist = GetOldMaxDistance(anOldExtPC);

        Assert.Equal(aOldMinDist, aNewMinDist, THE_TOL);
        Assert.Equal(aOldMaxDist, aNewMaxDist, THE_TOL);
    }

    //==================================================================================================
    // Parabola SearchMode tests
    //==================================================================================================

    [Fact]
    public void Parabola_MinMode_AtVertex()
    {
        ParabolaCurve3D aGeomParabola =
            new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0);
        CurveAdaptor anAdaptor = new(aGeomParabola, -20.0, 20.0);

        Point3D aPoint = new(0.0, 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);

        Assert.Equal(aOldMinDist, aNewMinDist, THE_TOL);
    }

    [Fact]
    public void Parabola_MaxMode()
    {
        ParabolaCurve3D aGeomParabola =
            new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0);
        CurveAdaptor anAdaptor = new(aGeomParabola, -20.0, 20.0);

        Point3D aPoint = new(0.0, 5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        // Use PerformWithEndpoints to find maximum which includes endpoints
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.PerformWithEndpoints(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

        Assert.True(aNewResult.IsDone);
        Assert.True(aNewResult.ExtCount >= 1);

        // With endpoints, Max mode finds the maximum at one of the endpoints.
        if (aNewResult.ExtCount > 0)
        {
            var aExtremum = aNewResult[aNewResult.MaxIndex()];
            Assert.True(aExtremum.Parameter >= -20.0 - THE_TOL);
            Assert.True(aExtremum.Parameter <= 20.0 + THE_TOL);
            Point3D aCurvePnt = anAdaptor.Value(aExtremum.Parameter);
            Assert.Equal(0.0, aCurvePnt.Distance(aExtremum.Point), THE_TOL);
        }
    }

    [Fact]
    public void Parabola_MinMode_OffAxis()
    {
        ParabolaCurve3D aGeomParabola =
            new ParabolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0);
        CurveAdaptor anAdaptor = new(aGeomParabola, -20.0, 20.0);

        Point3D aPoint = new(10.0, -5.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);

        Assert.Equal(aOldMinDist, aNewMinDist, THE_TOL);
    }

    //==================================================================================================
    // Hyperbola SearchMode tests
    //==================================================================================================

    [Fact]
    public void Hyperbola_MinMode()
    {
        HyperbolaCurve3D aGeomHyperbola =
            new HyperbolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0, 5.0);
        CurveAdaptor anAdaptor = new(aGeomHyperbola, -2.0, 2.0);

        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);

        Assert.Equal(aOldMinDist, aNewMinDist, THE_TOL);
    }

    [Fact]
    public void Hyperbola_MaxMode()
    {
        // NOTE: Max mode for bounded curves may not find extrema if the max is at
        // an endpoint (which is not a true extremum in the mathematical sense).
        HyperbolaCurve3D aGeomHyperbola =
            new HyperbolaCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0, 5.0);
        CurveAdaptor anAdaptor = new(aGeomHyperbola, -2.0, 2.0);

        Point3D aPoint = new(20.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

        Assert.True(aNewResult.IsDone);

        // For hyperbola, verify the found point is valid if extrema exist
        if (aNewResult.ExtCount > 0)
        {
            var aExtremum = aNewResult[aNewResult.MaxIndex()];
            Assert.True(aExtremum.Parameter >= -2.0 - THE_TOL);
            Assert.True(aExtremum.Parameter <= 2.0 + THE_TOL);
            Point3D aCurvePnt = anAdaptor.Value(aExtremum.Parameter);
            Assert.Equal(0.0, aCurvePnt.Distance(aExtremum.Point), THE_TOL);
        }
    }

    //==================================================================================================
    // BSpline SearchMode tests
    //==================================================================================================

    [Fact]
    public void BSpline_MinMode()
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

        Point3D aPoint = new(2.0, 3.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);

        // Allow slightly larger tolerance for BSpline
        Assert.Equal(aOldMinDist, aNewMinDist, 0.01);
    }

    [Fact]
    public void BSpline_MaxMode()
    {
        // NOTE: Max mode for BSpline may not find extrema if the max is at an endpoint.
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

        Point3D aPoint = new(2.0, 1.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

        Assert.True(aNewResult.IsDone);

        // Verify the found point is valid if extrema exist
        if (aNewResult.ExtCount > 0)
        {
            var aExtremum = aNewResult[aNewResult.MaxIndex()];
            Assert.True(aExtremum.Parameter >= 0.0 - THE_TOL);
            Assert.True(aExtremum.Parameter <= 1.0 + THE_TOL);
            Point3D aCurvePnt = anAdaptor.Value(aExtremum.Parameter);
            Assert.Equal(0.0, aCurvePnt.Distance(aExtremum.Point), 0.01);
        }
    }

    [Fact]
    public void BSpline_MinMaxMode()
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

        Point3D aPoint = new(2.0, 1.5, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.MinMax);

        Assert.True(aNewResult.IsDone);
        Assert.True(aNewResult.ExtCount >= 1);

        // Verify all found points are valid
        for (int i = 0; i < aNewResult.ExtCount; ++i)
        {
            var aExtremum = aNewResult[i];
            Assert.True(aExtremum.Parameter >= 0.0 - THE_TOL);
            Assert.True(aExtremum.Parameter <= 1.0 + THE_TOL);
            Point3D aCurvePnt = anAdaptor.Value(aExtremum.Parameter);
            Assert.Equal(0.0, aCurvePnt.Distance(aExtremum.Point), 0.01);
        }

        // Compare minimum with old implementation
        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);
        if (anOldExtPC.IsDone && anOldExtPC.ExtCount > 0)
        {
            double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
            double aOldMinDist = GetOldMinDistance(anOldExtPC);
            Assert.Equal(aOldMinDist, aNewMinDist, 0.05); // Relaxed tolerance for BSpline
        }
    }

    //==================================================================================================
    // Bezier SearchMode tests
    //==================================================================================================

    [Fact]
    public void Bezier_MinMode()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 3, 0);
        aPoles[3] = new Point3D(3, 3, 0);
        aPoles[4] = new Point3D(4, 0, 0);

        BezierCurve3D aBezier = new BezierCurve3D(aPoles);
        CurveAdaptor anAdaptor = new(aBezier);

        Point3D aPoint = new(2.0, 4.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);

        Assert.Equal(aOldMinDist, aNewMinDist, 0.01);
    }

    [Fact]
    public void Bezier_MaxMode()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 3, 0);
        aPoles[3] = new Point3D(3, 3, 0);
        aPoles[4] = new Point3D(4, 0, 0);

        BezierCurve3D aBezier = new BezierCurve3D(aPoles);
        CurveAdaptor anAdaptor = new(aBezier);

        Point3D aPoint = new(2.0, 1.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.PerformWithEndpoints(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

        Assert.True(aNewResult.IsDone);
        Assert.True(aNewResult.ExtCount >= 1);

        // With endpoint handling, max is at either endpoint (0,0,0) or (4,0,0)
        // Distance from (2,1,0) to endpoints = sqrt(4 + 1) = sqrt(5) ~= 2.236
        double aExpectedMaxDist = Math.Sqrt(5.0);
        double aNewMaxDist = Math.Sqrt(aNewResult.MaxSquareDistance());
        Assert.Equal(aExpectedMaxDist, aNewMaxDist, 0.01);
    }

    //==================================================================================================
    // Offset Curve SearchMode tests
    //==================================================================================================

    [Fact]
    public void OffsetCircle_MinMode()
    {
        CircleCurve3D aBaseCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        OffsetCurve3D anOffsetCurve =
            new OffsetCurve3D(aBaseCircle, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        Point3D aPoint = new(25.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

        CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

        Assert.True(aNewResult.IsDone);

        double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
        double aOldMinDist = GetOldMinDistance(anOldExtPC);

        Assert.Equal(aOldMinDist, aNewMinDist, 0.01);
        Assert.Equal(10.0, aNewMinDist, 0.01); // 25 - 15 = 10
    }

    [Fact]
    public void OffsetCircle_MaxMode()
    {
        // NOTE: Max mode for offset curves may not find extrema depending on the
        // BVH subdivision strategy.
        CircleCurve3D aBaseCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        OffsetCurve3D anOffsetCurve =
            new OffsetCurve3D(aBaseCircle, 5.0, new Direction3D(0, 0, 1));
        CurveAdaptor anAdaptor = new(anOffsetCurve);

        Point3D aPoint = new(25.0, 0.0, 0.0);

        global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
        global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
            anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

        Assert.True(aNewResult.IsDone);

        // Verify the found max extremum is valid if extrema exist
        if (aNewResult.ExtCount > 0)
        {
            var aExtremum = aNewResult[aNewResult.MaxIndex()];
            Point3D aCurvePnt = anAdaptor.Value(aExtremum.Parameter);
            Assert.Equal(0.0, aCurvePnt.Distance(aExtremum.Point), 0.01);

            // The max distance should be around 40 (25 + 15)
            double aNewMaxDist = Math.Sqrt(aNewResult.MaxSquareDistance());
            Assert.Equal(40.0, aNewMaxDist, 0.5);
        }
    }

    //==================================================================================================
    // Random points stress tests for SearchMode
    //==================================================================================================

    [Fact]
    public void Circle_RandomPoints_MinMode()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Mt19937 aGen = new(12345); // std::mt19937

        for (int i = 0; i < 50; ++i)
        {
            Point3D aPoint = new(aGen.NextReal(-50.0, 50.0), aGen.NextReal(-50.0, 50.0), 0.0);

            global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
                anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

            CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

            if (!anOldExtPC.IsDone)
            {
                continue;
            }

            Assert.True(aNewResult.IsDone); // "Iteration " << i

            double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
            double aOldMinDist = GetOldMinDistance(anOldExtPC);

            Assert.Equal(aOldMinDist, aNewMinDist, THE_TOL); // "Iteration " << i
        }
    }

    [Fact]
    public void Circle_RandomPoints_MaxMode()
    {
        CircleCurve3D aGeomCircle =
            new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 10.0);
        CurveAdaptor anAdaptor = new(aGeomCircle);

        Mt19937 aGen = new(12345); // std::mt19937

        for (int i = 0; i < 50; ++i)
        {
            Point3D aPoint = new(aGen.NextReal(-50.0, 50.0), aGen.NextReal(-50.0, 50.0), 0.0);

            global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
                anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Max);

            CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

            if (!anOldExtPC.IsDone)
            {
                continue;
            }

            Assert.True(aNewResult.IsDone); // "Iteration " << i

            double aNewMaxDist = Math.Sqrt(aNewResult.MaxSquareDistance());
            double aOldMaxDist = GetOldMaxDistance(anOldExtPC);

            Assert.Equal(aOldMaxDist, aNewMaxDist, THE_TOL); // "Iteration " << i
        }
    }

    [Fact]
    public void Ellipse_RandomPoints_MinMode()
    {
        EllipseCurve3D aGeomEllipse =
            new EllipseCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 20.0, 10.0);
        CurveAdaptor anAdaptor = new(aGeomEllipse);

        Mt19937 aGen = new(54321); // std::mt19937

        for (int i = 0; i < 50; ++i)
        {
            Point3D aPoint = new(aGen.NextReal(-50.0, 50.0), aGen.NextReal(-50.0, 50.0), 0.0);

            global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
                anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

            CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

            if (!anOldExtPC.IsDone)
            {
                continue;
            }

            Assert.True(aNewResult.IsDone); // "Iteration " << i

            double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
            double aOldMinDist = GetOldMinDistance(anOldExtPC);

            Assert.Equal(aOldMinDist, aNewMinDist, THE_TOL); // "Iteration " << i
        }
    }

    [Fact]
    public void BSpline_RandomPoints_MinMode()
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

        Mt19937 aGen = new(11111); // std::mt19937

        int aPassCount = 0;
        int aTestCount = 0;

        for (int i = 0; i < 30; ++i)
        {
            Point3D aPoint = new(aGen.NextReal(-2.0, 6.0), aGen.NextReal(-2.0, 6.0), 0.0);

            global::Kerf.Engine.Extrema.PointCurve.Curve anExtPC = new(anAdaptor);
            global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.Result aNewResult =
                anExtPC.Perform(aPoint, THE_TOL, global::Kerf.Engine.Extrema.PointCurve.ExtremaPC.SearchMode.Min);

            CurvePointExtrema anOldExtPC = new(aPoint, anAdaptor);

            if (!anOldExtPC.IsDone || anOldExtPC.ExtCount == 0)
            {
                continue;
            }

            if (!aNewResult.IsDone || aNewResult.ExtCount == 0)
            {
                continue;
            }

            ++aTestCount;

            double aNewMinDist = Math.Sqrt(aNewResult.MinSquareDistance());
            double aOldMinDist = GetOldMinDistance(anOldExtPC);

            // Allow larger tolerance for BSpline and count passes
            if (Math.Abs(aNewMinDist - aOldMinDist) < 0.1)
            {
                ++aPassCount;
            }
        }

        // At least 80% of valid tests should pass
        if (aTestCount > 0)
        {
            double aPassRate = (double)aPassCount / aTestCount;
            Assert.True(aPassRate >= 0.8); // "Pass rate: " << aPassRate * 100 << "% (" << aPassCount << "/" << aTestCount << ")"
        }
    }
}
