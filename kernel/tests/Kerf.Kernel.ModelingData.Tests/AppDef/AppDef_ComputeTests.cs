// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Engine.Approximation.ParametrizationType;
using static Kerf.Kernel.ModelingData.Tests.AppDef_BSplineComputeTests;

namespace Kerf.Kernel.ModelingData.Tests;

// AppDef_Compute (Approx_ComputeLine over AppDef_MultiLine): Bezier approximations, cut where needed.
public class AppDef_ComputeTests
{
    // The largest distance between the points and the Bezier curves at their parameters.
    private static double MaxDistance(Compute theComputer, NCollection_Array1<Point3D> thePoints)
    {
        double aMax = 0.0;
        for (int c = 1; c <= theComputer.MultiCurveCount; c++)
        {
            MultiCurve aC = theComputer.Value(c);
            NCollection_Array1<double> aPar = theComputer.Parameters(c);
            for (int i = aPar.Lower(); i <= aPar.Upper(); i++)
            {
                aC.Value(1, aPar[i], out Point3D aP);
                aMax = Math.Max(aMax, aP.Distance(thePoints[i]));
            }
        }
        return aMax;
    }

    public static TheoryData<ParametrizationType, bool> ParTypesAndSquares() => AppDef_BSplineComputeTests.ParTypesAndSquares();

    // The largest distance between the points of the MultiCurve theIndex and the curve, at their parameters.
    private static double MaxDistance(Compute theComputer, int theIndex, NCollection_Array1<Point3D> thePoints)
    {
        double aMax = 0.0;
        MultiCurve aC = theComputer.Value(theIndex);
        NCollection_Array1<double> aPar = theComputer.Parameters(theIndex);
        for (int i = aPar.Lower(); i <= aPar.Upper(); i++)
        {
            aC.Value(1, aPar[i], out Point3D aP);
            aMax = Math.Max(aMax, aP.Distance(thePoints[i]));
        }
        return aMax;
    }

    [Theory]
    [InlineData(ChordLength)]
    [InlineData(Centripetal)]
    [InlineData(IsoParametric)]
    public void Cubic_IsOneBezier(ParametrizationType thePar)
    {
        NCollection_Array1<Point3D> aPnts = Sample3d(25, Cubic);
        Compute aComputer = new(new MultiLine(aPnts), 3, 8, 1.0e-6, 1.0e-6, 5, true, thePar, false);
        Assert.True(aComputer.IsAllApproximated);
        Assert.True(aComputer.IsToleranceReached);
        Assert.Equal(1, aComputer.MultiCurveCount);
        aComputer.Error(1, out double aTol3d, out double aTol2d);
        Assert.True(aTol3d <= 1.0e-6);
        Assert.Equal(0.0, aTol2d);
        Assert.True(MaxDistance(aComputer, aPnts) <= 1.0e-6);
        Assert.Equal(thePar, aComputer.Parametrization());
    }

    [Fact]
    public void Cubic_IsExactWithSquaresAndUniformParameters()
    {
        // The cubic is sampled uniformly in t: with iso-parametric parameters, a cubic Bezier reproduces it.
        NCollection_Array1<Point3D> aPnts = Sample3d(25, Cubic);
        Compute aComputer = new(new MultiLine(aPnts), 3, 8, 1.0e-9, 1.0e-9, 5, true, IsoParametric, true);
        Assert.True(aComputer.IsToleranceReached);
        Assert.Equal(1, aComputer.MultiCurveCount);
        Assert.Equal(3, aComputer.Value().Degree());
        aComputer.Value().Value(1, 0.3, out Point3D aP);
        Assert.True(aP.Distance(Cubic(0.3)) < 1e-12);
    }

    [Theory]
    [MemberData(nameof(ParTypesAndSquares))]
    public void Sine_IsCutIntoBeziers(ParametrizationType thePar, bool theSquares)
    {
        NCollection_Array1<Point3D> aPnts = Sample3d(80, t => new Point3D(10.0 * t, Math.Sin(12.0 * t), 0.5 * Math.Cos(9.0 * t)));
        Compute aComputer = new(new MultiLine(aPnts), 3, 8, 1.0e-3, 1.0e-6, 5, true, thePar, theSquares);
        Assert.True(aComputer.IsAllApproximated);
        Assert.True(aComputer.MultiCurveCount > 1);
        // AppDef_MyLineTool adds no points, so the algorithm stops halving a part at degreemax + 5 points and keeps
        // its best curve: with chord-length parameters and no parameter optimisation, 1e-3 isn't reached here.
        Assert.Equal(!(theSquares && thePar == ChordLength), aComputer.IsToleranceReached);
        for (int c = 1; c <= aComputer.MultiCurveCount; c++)
        {
            aComputer.Error(c, out double aTol3d, out _);
            if (aComputer.IsToleranceReached)
            {
                Assert.True(aTol3d <= 1.0e-3, $"curve {c}: {aTol3d}");
            }
            // The error reported is the distance at the parameters reported.
            Assert.Equal(aTol3d, MaxDistance(aComputer, c, aPnts), 1e-12);
        }

        // The pieces join: each starts where the previous one ends, at the shared point.
        for (int c = 2; c <= aComputer.MultiCurveCount; c++)
        {
            aComputer.Value(c - 1).Value(1, 1.0, out Point3D aEnd);
            aComputer.Value(c).Value(1, 0.0, out Point3D aStart);
            Assert.True(aEnd.Distance(aStart) < 1e-9);
            Assert.Equal(aComputer.Parameters(c - 1).Upper(), aComputer.Parameters(c).Lower());
        }

        // SplineValue joins the pieces into one B-spline, with a knot per junction.
        MultiBSpCurve aSpline = aComputer.SplineValue();
        Assert.Equal(aComputer.MultiCurveCount + 1, aSpline.Knots().Length());
        aSpline.Value(1, aSpline.Knots()[aSpline.Knots().Upper()], out Point3D aLast);
        Assert.True(aLast.Distance(aPnts[aPnts.Upper()]) < 1e-9);
    }

    [Fact]
    public void Circle_IsFittedWithinTolerance()
    {
        NCollection_Array1<Point3D> aPnts = Sample3d(30, Circle);
        Compute aComputer = new(new MultiLine(aPnts), 4, 8, 1.0e-6, 1.0e-6, 5, true, ChordLength, false);
        Assert.True(aComputer.IsAllApproximated);
        Assert.True(MaxDistance(aComputer, aPnts) <= 1.0e-6);
    }

    [Fact]
    public void Line2d_IsFitted()
    {
        NCollection_Array1<Point2D> aPnts = Sample2d(20, t => new Point2D(t, t * t * t));
        Compute aComputer = new(new MultiLine(aPnts), 3, 6, 1.0e-6, 1.0e-6, 5, true, IsoParametric, true);
        Assert.Equal(1, aComputer.MultiCurveCount);
        MultiCurve aC = aComputer.Value();
        Assert.Equal(2, aC.Dimension(1));
        aC.Value(1, 0.5, out Point2D aP);
        Assert.Equal(0.5, aP.X(), 1e-9);
        Assert.Equal(0.125, aP.Y(), 1e-9);
    }

    [Fact]
    public void NoCutting_KeepsTheBestBezier()
    {
        NCollection_Array1<Point3D> aPnts = Sample3d(60, Sine);
        Compute aComputer = new(new MultiLine(aPnts), 3, 5, 1.0e-8, 1.0e-8, 5, false, ChordLength, true);
        Assert.False(aComputer.IsAllApproximated);
        Assert.Equal(1, aComputer.MultiCurveCount);
        aComputer.Error(1, out double aTol3d, out _);
        Assert.True(aTol3d > 1.0e-8);
        Assert.Equal(60, aComputer.Parameters().Length());
    }

    [Fact]
    public void Constraints_HoldAtTheEnds()
    {
        NCollection_Array1<Point3D> aPnts = Sample3d(20, Circle);
        Compute aComputer = new(4, 8, 1.0e-4, 1.0e-6, 5, true, ChordLength, false);
        aComputer.SetConstraints(PassPoint, PassPoint);
        aComputer.Perform(new MultiLine(aPnts));
        MultiCurve aFirst = aComputer.Value(1);
        MultiCurve aLastCurve = aComputer.Value(aComputer.MultiCurveCount);
        aFirst.Value(1, 0.0, out Point3D aP0);
        aLastCurve.Value(1, 1.0, out Point3D aP1);
        Assert.True(aP0.Distance(aPnts[1]) < 1e-9);
        Assert.True(aP1.Distance(aPnts[20]) < 1e-9);
    }

    [Fact]
    public void GivenParameters_AreUsed()
    {
        NCollection_Array1<Point3D> aPnts = Sample3d(12, Cubic);
        math_Vector aPar = new(1, 12);
        for (int i = 1; i <= 12; i++)
        {
            aPar[i] = (i - 1) / 11.0;
        }
        Compute aComputer = new(new MultiLine(aPnts), aPar, 3, 3, 1.0e-9, 1.0e-9, 0, false, true);
        Assert.Equal(1, aComputer.MultiCurveCount);
        Assert.Equal(3, aComputer.Value().Degree());
        aComputer.Value().Value(1, 0.7, out Point3D aP);
        Assert.True(aP.Distance(Cubic(0.7)) < 1e-12);
    }

    [Fact]
    public void MCurvesToBSpCurve_JoinsBeziers()
    {
        // Two cubic Bezier halves of one cubic join into a B-spline with a C2 junction (multiplicity 1 is not
        // recovered: the junction gets the multiplicity the converter finds, C1 at most).
        NCollection_Array1<Point3D> aPoles1 = NCollection_Array1<Point3D>.From(
            [new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 1, 0), new Point3D(3, 0, 0)]);
        NCollection_Array1<Point3D> aPoles2 = NCollection_Array1<Point3D>.From(
            [new Point3D(3, 0, 0), new Point3D(4, -1, 0), new Point3D(5, 0, 1), new Point3D(6, 2, 2)]);
        MCurvesToBSpCurve aTrans = new();
        aTrans.Append(MultiCurve(aPoles1));
        aTrans.Append(MultiCurve(aPoles2));
        aTrans.Perform();
        MultiBSpCurve aSpline = aTrans.Value();
        Assert.Equal(3, aSpline.Degree());
        Assert.Equal(3, aSpline.Knots().Length());
        Assert.Equal(new[] { 4, 2, 4 }, aSpline.Multiplicities().ToArray()); // the tangents line up: C1
        Assert.Equal(6, aSpline.NbPoles());
        aSpline.Value(1, aSpline.Knots()[1], out Point3D aStart);
        aSpline.Value(1, aSpline.Knots()[3], out Point3D aEnd);
        Assert.True(aStart.Distance(new Point3D(0, 0, 0)) < 1e-12);
        Assert.True(aEnd.Distance(new Point3D(6, 2, 2)) < 1e-12);

        // One curve: a Bezier on [0, 1].
        aTrans.Reset();
        aTrans.Append(MultiCurve(aPoles1));
        aTrans.Perform();
        Assert.Equal(new[] { 0.0, 1.0 }, aTrans.Value().Knots().ToArray());
        Assert.Equal(new[] { 4, 4 }, aTrans.Value().Multiplicities().ToArray());
    }

    private static MultiCurve MultiCurve(NCollection_Array1<Point3D> thePoles)
    {
        MultiCurve aC = new(thePoles.Length());
        for (int i = 1; i <= thePoles.Length(); i++)
        {
            aC.SetValue(i, new MultiPoint(NCollection_Array1<Point3D>.From([thePoles[i]])));
        }
        return aC;
    }
}
