// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.AppParCurves_Constraint;
using static Occt.Approx_ParametrizationType;
using static OcctNet.ModelingData.Tests.AppDef_BSplineComputeTests;

namespace OcctNet.ModelingData.Tests;

// AppDef_Compute (Approx_ComputeLine over AppDef_MultiLine): Bezier approximations, cut where needed.
public class AppDef_ComputeTests
{
    // The largest distance between the points and the Bezier curves at their parameters.
    private static double MaxDistance(AppDef_Compute theComputer, NCollection_Array1<gp_Pnt> thePoints)
    {
        double aMax = 0.0;
        for (int c = 1; c <= theComputer.NbMultiCurves(); c++)
        {
            AppParCurves_MultiCurve aC = theComputer.Value(c);
            NCollection_Array1<double> aPar = theComputer.Parameters(c);
            for (int i = aPar.Lower(); i <= aPar.Upper(); i++)
            {
                aC.Value(1, aPar[i], out gp_Pnt aP);
                aMax = Math.Max(aMax, aP.Distance(thePoints[i]));
            }
        }
        return aMax;
    }

    public static TheoryData<Approx_ParametrizationType, bool> ParTypesAndSquares() => AppDef_BSplineComputeTests.ParTypesAndSquares();

    // The largest distance between the points of the MultiCurve theIndex and the curve, at their parameters.
    private static double MaxDistance(AppDef_Compute theComputer, int theIndex, NCollection_Array1<gp_Pnt> thePoints)
    {
        double aMax = 0.0;
        AppParCurves_MultiCurve aC = theComputer.Value(theIndex);
        NCollection_Array1<double> aPar = theComputer.Parameters(theIndex);
        for (int i = aPar.Lower(); i <= aPar.Upper(); i++)
        {
            aC.Value(1, aPar[i], out gp_Pnt aP);
            aMax = Math.Max(aMax, aP.Distance(thePoints[i]));
        }
        return aMax;
    }

    [Theory]
    [InlineData(Approx_ChordLength)]
    [InlineData(Approx_Centripetal)]
    [InlineData(Approx_IsoParametric)]
    public void Cubic_IsOneBezier(Approx_ParametrizationType thePar)
    {
        NCollection_Array1<gp_Pnt> aPnts = Sample3d(25, Cubic);
        AppDef_Compute aComputer = new(new AppDef_MultiLine(aPnts), 3, 8, 1.0e-6, 1.0e-6, 5, true, thePar, false);
        Assert.True(aComputer.IsAllApproximated());
        Assert.True(aComputer.IsToleranceReached());
        Assert.Equal(1, aComputer.NbMultiCurves());
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
        NCollection_Array1<gp_Pnt> aPnts = Sample3d(25, Cubic);
        AppDef_Compute aComputer = new(new AppDef_MultiLine(aPnts), 3, 8, 1.0e-9, 1.0e-9, 5, true, Approx_IsoParametric, true);
        Assert.True(aComputer.IsToleranceReached());
        Assert.Equal(1, aComputer.NbMultiCurves());
        Assert.Equal(3, aComputer.Value().Degree());
        aComputer.Value().Value(1, 0.3, out gp_Pnt aP);
        Assert.True(aP.Distance(Cubic(0.3)) < 1e-12);
    }

    [Theory]
    [MemberData(nameof(ParTypesAndSquares))]
    public void Sine_IsCutIntoBeziers(Approx_ParametrizationType thePar, bool theSquares)
    {
        NCollection_Array1<gp_Pnt> aPnts = Sample3d(80, t => new gp_Pnt(10.0 * t, Math.Sin(12.0 * t), 0.5 * Math.Cos(9.0 * t)));
        AppDef_Compute aComputer = new(new AppDef_MultiLine(aPnts), 3, 8, 1.0e-3, 1.0e-6, 5, true, thePar, theSquares);
        Assert.True(aComputer.IsAllApproximated());
        Assert.True(aComputer.NbMultiCurves() > 1);
        // AppDef_MyLineTool adds no points, so the algorithm stops halving a part at degreemax + 5 points and keeps
        // its best curve: with chord-length parameters and no parameter optimisation, 1e-3 isn't reached here.
        Assert.Equal(!(theSquares && thePar == Approx_ChordLength), aComputer.IsToleranceReached());
        for (int c = 1; c <= aComputer.NbMultiCurves(); c++)
        {
            aComputer.Error(c, out double aTol3d, out _);
            if (aComputer.IsToleranceReached())
            {
                Assert.True(aTol3d <= 1.0e-3, $"curve {c}: {aTol3d}");
            }
            // The error reported is the distance at the parameters reported.
            Assert.Equal(aTol3d, MaxDistance(aComputer, c, aPnts), 1e-12);
        }

        // The pieces join: each starts where the previous one ends, at the shared point.
        for (int c = 2; c <= aComputer.NbMultiCurves(); c++)
        {
            aComputer.Value(c - 1).Value(1, 1.0, out gp_Pnt aEnd);
            aComputer.Value(c).Value(1, 0.0, out gp_Pnt aStart);
            Assert.True(aEnd.Distance(aStart) < 1e-9);
            Assert.Equal(aComputer.Parameters(c - 1).Upper(), aComputer.Parameters(c).Lower());
        }

        // SplineValue joins the pieces into one B-spline, with a knot per junction.
        AppParCurves_MultiBSpCurve aSpline = aComputer.SplineValue();
        Assert.Equal(aComputer.NbMultiCurves() + 1, aSpline.Knots().Length());
        aSpline.Value(1, aSpline.Knots()[aSpline.Knots().Upper()], out gp_Pnt aLast);
        Assert.True(aLast.Distance(aPnts[aPnts.Upper()]) < 1e-9);
    }

    [Fact]
    public void Circle_IsFittedWithinTolerance()
    {
        NCollection_Array1<gp_Pnt> aPnts = Sample3d(30, Circle);
        AppDef_Compute aComputer = new(new AppDef_MultiLine(aPnts), 4, 8, 1.0e-6, 1.0e-6, 5, true, Approx_ChordLength, false);
        Assert.True(aComputer.IsAllApproximated());
        Assert.True(MaxDistance(aComputer, aPnts) <= 1.0e-6);
    }

    [Fact]
    public void Line2d_IsFitted()
    {
        NCollection_Array1<gp_Pnt2d> aPnts = Sample2d(20, t => new gp_Pnt2d(t, t * t * t));
        AppDef_Compute aComputer = new(new AppDef_MultiLine(aPnts), 3, 6, 1.0e-6, 1.0e-6, 5, true, Approx_IsoParametric, true);
        Assert.Equal(1, aComputer.NbMultiCurves());
        AppParCurves_MultiCurve aC = aComputer.Value();
        Assert.Equal(2, aC.Dimension(1));
        aC.Value(1, 0.5, out gp_Pnt2d aP);
        Assert.Equal(0.5, aP.X(), 1e-9);
        Assert.Equal(0.125, aP.Y(), 1e-9);
    }

    [Fact]
    public void NoCutting_KeepsTheBestBezier()
    {
        NCollection_Array1<gp_Pnt> aPnts = Sample3d(60, Sine);
        AppDef_Compute aComputer = new(new AppDef_MultiLine(aPnts), 3, 5, 1.0e-8, 1.0e-8, 5, false, Approx_ChordLength, true);
        Assert.False(aComputer.IsAllApproximated());
        Assert.Equal(1, aComputer.NbMultiCurves());
        aComputer.Error(1, out double aTol3d, out _);
        Assert.True(aTol3d > 1.0e-8);
        Assert.Equal(60, aComputer.Parameters().Length());
    }

    [Fact]
    public void Constraints_HoldAtTheEnds()
    {
        NCollection_Array1<gp_Pnt> aPnts = Sample3d(20, Circle);
        AppDef_Compute aComputer = new(4, 8, 1.0e-4, 1.0e-6, 5, true, Approx_ChordLength, false);
        aComputer.SetConstraints(AppParCurves_PassPoint, AppParCurves_PassPoint);
        aComputer.Perform(new AppDef_MultiLine(aPnts));
        AppParCurves_MultiCurve aFirst = aComputer.Value(1);
        AppParCurves_MultiCurve aLastCurve = aComputer.Value(aComputer.NbMultiCurves());
        aFirst.Value(1, 0.0, out gp_Pnt aP0);
        aLastCurve.Value(1, 1.0, out gp_Pnt aP1);
        Assert.True(aP0.Distance(aPnts[1]) < 1e-9);
        Assert.True(aP1.Distance(aPnts[20]) < 1e-9);
    }

    [Fact]
    public void GivenParameters_AreUsed()
    {
        NCollection_Array1<gp_Pnt> aPnts = Sample3d(12, Cubic);
        math_Vector aPar = new(1, 12);
        for (int i = 1; i <= 12; i++)
        {
            aPar[i] = (i - 1) / 11.0;
        }
        AppDef_Compute aComputer = new(new AppDef_MultiLine(aPnts), aPar, 3, 3, 1.0e-9, 1.0e-9, 0, false, true);
        Assert.Equal(1, aComputer.NbMultiCurves());
        Assert.Equal(3, aComputer.Value().Degree());
        aComputer.Value().Value(1, 0.7, out gp_Pnt aP);
        Assert.True(aP.Distance(Cubic(0.7)) < 1e-12);
    }

    [Fact]
    public void MCurvesToBSpCurve_JoinsBeziers()
    {
        // Two cubic Bezier halves of one cubic join into a B-spline with a C2 junction (multiplicity 1 is not
        // recovered: the junction gets the multiplicity the converter finds, C1 at most).
        NCollection_Array1<gp_Pnt> aPoles1 = NCollection_Array1<gp_Pnt>.From(
            [new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(2, 1, 0), new gp_Pnt(3, 0, 0)]);
        NCollection_Array1<gp_Pnt> aPoles2 = NCollection_Array1<gp_Pnt>.From(
            [new gp_Pnt(3, 0, 0), new gp_Pnt(4, -1, 0), new gp_Pnt(5, 0, 1), new gp_Pnt(6, 2, 2)]);
        Approx_MCurvesToBSpCurve aTrans = new();
        aTrans.Append(MultiCurve(aPoles1));
        aTrans.Append(MultiCurve(aPoles2));
        aTrans.Perform();
        AppParCurves_MultiBSpCurve aSpline = aTrans.Value();
        Assert.Equal(3, aSpline.Degree());
        Assert.Equal(3, aSpline.Knots().Length());
        Assert.Equal(new[] { 4, 2, 4 }, aSpline.Multiplicities().ToArray()); // the tangents line up: C1
        Assert.Equal(6, aSpline.NbPoles());
        aSpline.Value(1, aSpline.Knots()[1], out gp_Pnt aStart);
        aSpline.Value(1, aSpline.Knots()[3], out gp_Pnt aEnd);
        Assert.True(aStart.Distance(new gp_Pnt(0, 0, 0)) < 1e-12);
        Assert.True(aEnd.Distance(new gp_Pnt(6, 2, 2)) < 1e-12);

        // One curve: a Bezier on [0, 1].
        aTrans.Reset();
        aTrans.Append(MultiCurve(aPoles1));
        aTrans.Perform();
        Assert.Equal(new[] { 0.0, 1.0 }, aTrans.Value().Knots().ToArray());
        Assert.Equal(new[] { 4, 4 }, aTrans.Value().Multiplicities().ToArray());
    }

    private static AppParCurves_MultiCurve MultiCurve(NCollection_Array1<gp_Pnt> thePoles)
    {
        AppParCurves_MultiCurve aC = new(thePoles.Length());
        for (int i = 1; i <= thePoles.Length(); i++)
        {
            aC.SetValue(i, new AppParCurves_MultiPoint(NCollection_Array1<gp_Pnt>.From([thePoles[i]])));
        }
        return aC;
    }
}
