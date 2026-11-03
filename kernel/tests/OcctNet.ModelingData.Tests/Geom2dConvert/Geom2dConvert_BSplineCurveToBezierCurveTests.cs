// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/Geom2dConvert_BSplineCurveToBezierCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class Geom2dConvert_BSplineCurveToBezierCurveTest
{
    // OCC7372: Invalid conversion of 2D periodic BSpline curve to Bezier segments after IncreaseDegree.
    // Tests that a periodic BSpline curve (5 points, degree increased to 8) converts to exactly
    // 5 Bezier arcs, and the 5th arc has the expected length ~73.3203.
    [Fact]
    public void OCC7372_PeriodicBSplineToBeziersAfterIncreaseDegree()
    {
        // OCCT interpolates (100, 0), (100, 100), (0, 100), (0, 0), (50, -50) with Geom2dAPI_Interpolate (periodic,
        // tolerance 1e-6), which isn't ported (GeomAPI). The curve here is native GeomAPI_Interpolate's for the same
        // points at z = 0 (the same algorithm in 3D), printed with full precision through the oracle bindings.
        NCollection_Array1<Point2D> aPoles = new(1, 6);
        aPoles[1] = new Point2D(72.50140295385648, -35.35533905932737);
        aPoles[2] = new Point2D(138.88888888888889, 49.99999999999999);
        aPoles[3] = new Point2D(119.28047312677221, 114.82619883884264);
        aPoles[4] = new Point2D(-25.81498927703611, 123.10830406405078);
        aPoles[5] = new Point2D(-16.97676095170697, 5.468525202912922);
        aPoles[6] = new Point2D(56.93090102243691, -69.52493824671032);
        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From([0.0, 100.0, 200.0, 300.0, 370.71067811865476, 441.4213562373095]);
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From([2, 1, 1, 1, 1, 2]);

        Geom2d_BSplineCurve aBSpline = new(aPoles, aKnots, aMults, 3, true);
        Assert.NotNull(aBSpline);
        Assert.True(aBSpline.IsPeriodic());

        aBSpline = aBSpline.DegreeIncreased(8);

        Geom2dConvert_BSplineCurveToBezierCurve aConverter = new(aBSpline);
        int aNbArcs = aConverter.NbArcs();
        Assert.Equal(5, aNbArcs);

        Geom2d_BezierCurve anArc5 = aConverter.Arc(5);
        Assert.NotNull(anArc5);

        Geom2dAdaptor_Curve anAdaptor = new(anArc5);
        double aLen = GCPnts_AbscissaPoint.Length(anAdaptor);
        Assert.Equal(73.3203, aLen, 0.01);
    }
}
