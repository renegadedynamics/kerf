// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.AppParCurves_Constraint;
using static Occt.Approx_ParametrizationType;

namespace OcctNet.ModelingData.Tests;

// AppDef_BSplineCompute (Approx_BSplComputeLine over AppDef_MultiLine) on sampled curves with known shapes. OCCT
// has no GTests for AppDef; the oracle tests compare it with native GeomAPI_PointsToBSpline.
public class AppDef_BSplineComputeTests
{
    internal static NCollection_Array1<Point3D> Sample3d(int theNb, Func<double, Point3D> theF, bool theUneven = false)
    {
        NCollection_Array1<Point3D> aPnts = new(1, theNb);
        for (int i = 1; i <= theNb; i++)
        {
            double t = (i - 1) / (double)(theNb - 1);
            if (theUneven)
            {
                t = t * t * (3.0 - 2.0 * t); // denser at the ends
            }
            aPnts[i] = theF(t);
        }
        return aPnts;
    }

    internal static NCollection_Array1<Point2D> Sample2d(int theNb, Func<double, Point2D> theF)
    {
        NCollection_Array1<Point2D> aPnts = new(1, theNb);
        for (int i = 1; i <= theNb; i++)
        {
            aPnts[i] = theF((i - 1) / (double)(theNb - 1));
        }
        return aPnts;
    }

    internal static Point3D Cubic(double t) => new(1.0 + 2.0 * t - t * t + 3.0 * t * t * t, -t + 4.0 * t * t, 2.0 * t * t * t);

    internal static Point3D Circle(double t) => new(5.0 * Math.Cos(t * 3.0), 5.0 * Math.Sin(t * 3.0), 0.0);

    internal static Point3D Sine(double t) => new(10.0 * t, Math.Sin(6.0 * t), 0.5 * Math.Cos(4.0 * t));

    // The largest distance between the points and the curve at the computed parameters.
    private static double MaxDistance(AppDef_BSplineCompute theComputer, NCollection_Array1<Point3D> thePoints)
    {
        AppParCurves_MultiBSpCurve aC = theComputer.Value();
        NCollection_Array1<double> aPar = theComputer.Parameters();
        double aMax = 0.0;
        for (int i = thePoints.Lower(); i <= thePoints.Upper(); i++)
        {
            aC.Value(1, aPar[i], out Point3D aP);
            aMax = Math.Max(aMax, aP.Distance(thePoints[i]));
        }
        return aMax;
    }

    public static TheoryData<Approx_ParametrizationType, bool> ParTypesAndSquares()
    {
        TheoryData<Approx_ParametrizationType, bool> aData = [];
        foreach (Approx_ParametrizationType aPar in new[] { Approx_ChordLength, Approx_Centripetal, Approx_IsoParametric })
        {
            aData.Add(aPar, true);
            aData.Add(aPar, false);
        }
        return aData;
    }

    [Theory]
    [MemberData(nameof(ParTypesAndSquares))]
    public void Cubic_IsReproduced(Approx_ParametrizationType thePar, bool theSquares)
    {
        NCollection_Array1<Point3D> aPnts = Sample3d(25, Cubic);
        AppDef_BSplineCompute aComputer = new(3, 8, 1.0e-6, 1.0e-6, 5, true, thePar, theSquares);
        aComputer.Perform(new AppDef_MultiLine(aPnts));

        Assert.True(aComputer.IsToleranceReached());
        aComputer.Error(out double aTol3d, out double aTol2d);
        Assert.True(aTol3d <= 1.0e-6, $"error {aTol3d}");
        Assert.Equal(0.0, aTol2d);
        Assert.True(MaxDistance(aComputer, aPnts) <= 1.0e-6);
        AppParCurves_MultiBSpCurve aC = aComputer.Value();
        aC.Value(1, 0.0, out Point3D aFirst);
        aC.Value(1, 1.0, out Point3D aLast);
        Assert.True(aFirst.Distance(aPnts[1]) < 1.0e-9);
        Assert.True(aLast.Distance(aPnts[25]) < 1.0e-9);
    }

    [Theory]
    [MemberData(nameof(ParTypesAndSquares))]
    public void CircleAndSine_AreFittedWithinTolerance(Approx_ParametrizationType thePar, bool theSquares)
    {
        foreach (Func<double, Point3D> aF in new Func<double, Point3D>[] { Circle, Sine })
        {
            NCollection_Array1<Point3D> aPnts = Sample3d(40, aF, theUneven: true);
            AppDef_BSplineCompute aComputer = new(3, 8, 1.0e-4, 1.0e-6, 5, true, thePar, theSquares);
            aComputer.SetContinuity(2);
            aComputer.Perform(new AppDef_MultiLine(aPnts));

            Assert.True(aComputer.IsToleranceReached());
            aComputer.Error(out double aTol3d, out _);
            Assert.True(aTol3d <= 1.0e-4, $"error {aTol3d}");
            Assert.True(MaxDistance(aComputer, aPnts) <= 1.0e-4);
            AppParCurves_MultiBSpCurve aC = aComputer.Value();
            NCollection_Array1<int> aMults = aC.Multiplicities();
            for (int i = aMults.Lower() + 1; i < aMults.Upper(); i++)
            {
                Assert.Equal(aC.Degree() - 2, aMults[i]); // C2 interior knots
            }
            Assert.Equal(aC.Degree() + 1, aMults[aMults.Lower()]);
        }
    }

    [Fact]
    public void Parametrisations_GiveTheirParameters()
    {
        // Points at x = 0, 1, 3, 7: chord lengths 1, 2, 4.
        NCollection_Array1<Point3D> aPnts = new(1, 4);
        double[] aX = [0.0, 1.0, 3.0, 7.0];
        for (int i = 1; i <= 4; i++)
        {
            aPnts[i] = new Point3D(aX[i - 1], 0.0, 0.0);
        }
        AppDef_MultiLine aLine = new(aPnts);

        AppDef_BSplineCompute aChord = new(aLine, 1, 1, 1.0e-3, 1.0e-6, 0, false, Approx_ChordLength, true);
        Assert.Equal([0.0, 1.0 / 7.0, 3.0 / 7.0, 1.0], aChord.Parameters().ToArray());

        AppDef_BSplineCompute aIso = new(aLine, 1, 1, 1.0e-3, 1.0e-6, 0, false, Approx_IsoParametric, true);
        Assert.Equal([0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0], aIso.Parameters().ToArray());

        AppDef_BSplineCompute aCentri = new(aLine, 1, 1, 1.0e-3, 1.0e-6, 0, false, Approx_Centripetal, true);
        double aS = 1.0 + Math.Sqrt(2.0) + 2.0;
        double[] aCentriPar = aCentri.Parameters().ToArray();
        Assert.Equal(1.0 / aS, aCentriPar[1], 1e-15);
        Assert.Equal((1.0 + Math.Sqrt(2.0)) / aS, aCentriPar[2], 1e-15);
    }

    [Fact]
    public void Line2d_IsFitted()
    {
        NCollection_Array1<Point2D> aPnts = Sample2d(30, t => new Point2D(3.0 * Math.Cos(2.0 * t), 2.0 * Math.Sin(2.0 * t)));
        AppDef_BSplineCompute aComputer = new(new AppDef_MultiLine(aPnts), 3, 8, 1.0e-6, 1.0e-5, 5, true,
                                              Approx_ChordLength, true);
        Assert.True(aComputer.IsToleranceReached());
        aComputer.Error(out _, out double aTol2d);
        Assert.True(aTol2d <= 1.0e-5);
        AppParCurves_MultiBSpCurve aC = aComputer.Value();
        Assert.Equal(1, aC.NbCurves());
        Assert.Equal(2, aC.Dimension(1));
        NCollection_Array1<double> aPar = aComputer.Parameters();
        for (int i = 1; i <= 30; i++)
        {
            aC.Value(1, aPar[i], out Point2D aP);
            Assert.True(aP.Distance(aPnts[i]) <= 1.0e-5);
        }
    }

    [Fact]
    public void ThreeAndTwoDimensionalLines_AreFittedTogether()
    {
        int aNb = 20;
        AppDef_MultiLine aLine = new(aNb);
        for (int i = 1; i <= aNb; i++)
        {
            double t = (i - 1) / (double)(aNb - 1);
            NCollection_Array1<Point3D> aP3d = NCollection_Array1<Point3D>.From([Cubic(t)]);
            NCollection_Array1<Point2D> aP2d = NCollection_Array1<Point2D>.From([new Point2D(t, t * t)]);
            aLine.SetValue(i, new AppDef_MultiPointConstraint(aP3d, aP2d));
        }
        AppDef_BSplineCompute aComputer = new(aLine, 3, 6, 1.0e-6, 1.0e-6, 5, true, Approx_IsoParametric, true);
        Assert.True(aComputer.IsToleranceReached());
        AppParCurves_MultiBSpCurve aC = aComputer.Value();
        Assert.Equal(2, aC.NbCurves());
        aC.Value(2, 0.5, out Point2D aMid);
        Assert.Equal(0.5, aMid.X(), 1e-9);
        Assert.Equal(0.25, aMid.Y(), 1e-9);
        aC.Value(1, 0.5, out Point3D aMid3d);
        Assert.True(aMid3d.Distance(Cubic(0.5)) < 1e-9);
    }

    [Fact]
    public void EndTangents_AreHonoured()
    {
        int aNb = 15;
        AppDef_MultiLine aLine = new(aNb);
        for (int i = 1; i <= aNb; i++)
        {
            double t = (i - 1) / (double)(aNb - 1);
            NCollection_Array1<Point3D> aP = NCollection_Array1<Point3D>.From([Circle(t)]);
            if (i == 1 || i == aNb)
            {
                Vector3D aTangent = new(-15.0 * Math.Sin(3.0 * t), 15.0 * Math.Cos(3.0 * t), 0.0);
                aLine.SetValue(i, new AppDef_MultiPointConstraint(aP, NCollection_Array1<Vector3D>.From([aTangent])));
            }
            else
            {
                aLine.SetValue(i, new AppDef_MultiPointConstraint(aP));
            }
        }
        // The default constraints are TangencyPoint at both ends.
        AppDef_BSplineCompute aComputer = new(aLine, 3, 8, 1.0e-5, 1.0e-6, 5, true, Approx_ChordLength, true);
        AppParCurves_MultiBSpCurve aC = aComputer.Value();
        aC.D1(1, 0.0, out Point3D aP0, out Vector3D aV0);
        aC.D1(1, 1.0, out Point3D aP1, out Vector3D aV1);
        Assert.True(aP0.Distance(Circle(0.0)) < 1e-9);
        Assert.True(aP1.Distance(Circle(1.0)) < 1e-9);
        Assert.True(aV0.IsParallel(new Vector3D(0.0, 1.0, 0.0), 1e-9));
        Assert.True(aV1.IsParallel(new Vector3D(-Math.Sin(3.0), Math.Cos(3.0), 0.0), 1e-9));

        // Without tangents, a TangencyPoint constraint falls back to a passing point.
        AppDef_BSplineCompute aNoTangents = new(new AppDef_MultiLine(Sample3d(aNb, Circle)), 3, 8, 1.0e-5, 1.0e-6, 5,
                                                true, Approx_ChordLength, true);
        Assert.True(aNoTangents.IsToleranceReached());
    }

    [Fact]
    public void FewPoints_AreInterpolated()
    {
        // 5 points, degree 4 to 8 (the defaults): a degree 4 Bezier passes through them.
        NCollection_Array1<Point3D> aPnts = Sample3d(5, Sine);
        AppDef_BSplineCompute aComputer = new(new AppDef_MultiLine(aPnts));
        AppParCurves_MultiBSpCurve aC = aComputer.Value();
        Assert.True(aComputer.IsToleranceReached());
        Assert.Equal(4, aC.Degree());
        Assert.True(MaxDistance(aComputer, aPnts) < 1e-9);

        // 4 points: 5 poles to compute for 4 points, so the algorithm interpolates with a C2 cubic.
        NCollection_Array1<Point3D> aFour = Sample3d(4, Sine);
        AppDef_BSplineCompute aCubic = new(new AppDef_MultiLine(aFour));
        Assert.Equal(3, aCubic.Value().Degree());
        Assert.Equal(6, aCubic.Value().NbPoles());
        Assert.True(MaxDistance(aCubic, aFour) < 1e-9);

        // Two points: a segment.
        NCollection_Array1<Point3D> aTwo = Sample3d(2, Sine);
        AppDef_BSplineCompute aSegment = new(new AppDef_MultiLine(aTwo));
        Assert.Equal(1, aSegment.Value().Degree());
        Assert.Equal(2, aSegment.Value().NbPoles());
        Assert.True(MaxDistance(aSegment, aTwo) < 1e-12);
    }

    [Fact]
    public void GivenKnots_AreUsed()
    {
        NCollection_Array1<Point3D> aPnts = Sample3d(40, Sine);
        AppDef_BSplineCompute aComputer = new(3, 3, 1.0e-3, 1.0e-6, 0, false, Approx_ChordLength, true);
        aComputer.SetKnots(NCollection_Array1<double>.From([0.0, 0.25, 0.5, 0.75, 1.0]));
        aComputer.SetContinuity(2);
        aComputer.Perform(new AppDef_MultiLine(aPnts));
        AppParCurves_MultiBSpCurve aC = aComputer.Value();
        Assert.Equal([0.0, 0.25, 0.5, 0.75, 1.0], aC.Knots().ToArray());
        Assert.Equal([4, 1, 1, 1, 4], aC.Multiplicities().ToArray());
        Assert.Equal(7, aC.NbPoles());

        AppDef_BSplineCompute aWithMults = new(3, 3, 1.0e-3, 1.0e-6, 0, false, Approx_ChordLength, true);
        aWithMults.SetKnotsAndMultiplicities(NCollection_Array1<double>.From([0.0, 0.5, 1.0]),
                                             NCollection_Array1<int>.From([4, 3, 4]));
        aWithMults.Perform(new AppDef_MultiLine(aPnts));
        Assert.Equal([4, 3, 4], aWithMults.Value().Multiplicities().ToArray());
        Assert.Equal(7, aWithMults.Value().NbPoles());
    }

    [Fact]
    public void GivenParameters_AreUsed()
    {
        NCollection_Array1<Point3D> aPnts = Sample3d(20, Cubic);
        math_Vector aPar = new(1, 20);
        for (int i = 1; i <= 20; i++)
        {
            aPar[i] = (i - 1) / 19.0;
        }
        // The cubic is sampled uniformly in t: with t as parameters, a single cubic Bezier is exact.
        AppDef_BSplineCompute aComputer = new(new AppDef_MultiLine(aPnts), aPar, 3, 3, 1.0e-9, 1.0e-9, 0, false, true);
        Assert.True(aComputer.IsToleranceReached());
        AppParCurves_MultiBSpCurve aC = aComputer.Value();
        Assert.Equal(4, aC.NbPoles());
        aC.Value(1, 0.3, out Point3D aP);
        Assert.True(aP.Distance(Cubic(0.3)) < 1e-12);
    }

    [Fact]
    public void ToleranceNotReached_KeepsTheBestCurve()
    {
        // Without cutting and with a low degree, a wavy curve can't be reached: the best curve is kept.
        NCollection_Array1<Point3D> aPnts = Sample3d(50, Sine);
        AppDef_BSplineCompute aComputer = new(new AppDef_MultiLine(aPnts), 3, 4, 1.0e-6, 1.0e-6, 5, false,
                                              Approx_ChordLength, true);
        Assert.False(aComputer.IsAllApproximated());
        aComputer.Error(out double aTol3d, out _);
        Assert.True(aTol3d > 1.0e-6 && aTol3d < 1.0);
        Assert.True(aComputer.Value().NbPoles() > 0);
    }

    [Fact]
    public void Periodic_InterpolationJoinsTangents()
    {
        // A closed curve with few points is interpolated; SetPeriodic averages the end tangents.
        int aNb = 12;
        NCollection_Array1<Point3D> aPnts = new(1, aNb);
        for (int i = 1; i <= aNb; i++)
        {
            double a = 2.0 * Math.PI * (i - 1) / (aNb - 1);
            aPnts[i] = new Point3D(Math.Cos(a), Math.Sin(a), 0.0);
        }
        AppDef_BSplineCompute aComputer = new(8, 8, 1.0e-6, 1.0e-6, 0, true, Approx_ChordLength, true);
        aComputer.SetPeriodic(true);
        aComputer.SetConstraints(AppParCurves_PassPoint, AppParCurves_PassPoint);
        aComputer.Interpol(new AppDef_MultiLine(aPnts));
        AppParCurves_MultiBSpCurve aC = aComputer.Value();
        aC.D1(1, 0.0, out _, out Vector3D aV0);
        aC.D1(1, 1.0, out _, out Vector3D aV1);
        Assert.True(aV0.IsParallel(aV1, 1e-9));
        Assert.Equal(3, aC.Degree());
    }
}
