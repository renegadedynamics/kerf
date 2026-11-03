// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Engine.Numerics.Polynomials;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Occt.AppParCurves_Constraint;
using static Kerf.Geometry.Continuity;

namespace OcctNet.ModelingData.Tests;

// AppDef_Variational and AppDef_LinearCriteria on sampled curves with known shapes (OCCT has no GTests for them).
public class AppDef_VariationalTests(ITestOutputHelper output)
{
    private static AppDef_MultiLine Line3d(int theNb, Func<double, Point3D> theF)
    {
        NCollection_Array1<Point3D> aPnts = new(1, theNb);
        for (int i = 1; i <= theNb; i++)
        {
            aPnts[i] = theF((i - 1) / (double)(theNb - 1));
        }
        return new AppDef_MultiLine(aPnts);
    }

    private static AppDef_MultiLine Line2d(int theNb, Func<double, Point2D> theF)
    {
        NCollection_Array1<Point2D> aPnts = new(1, theNb);
        for (int i = 1; i <= theNb; i++)
        {
            aPnts[i] = theF((i - 1) / (double)(theNb - 1));
        }
        return new AppDef_MultiLine(aPnts);
    }

    private static NCollection_HArray1<AppParCurves_ConstraintCouple> Constraints(
        params (int Index, AppParCurves_Constraint Cons)[] theCouples)
    {
        NCollection_HArray1<AppParCurves_ConstraintCouple> aTab = new(1, theCouples.Length);
        for (int i = 0; i < theCouples.Length; i++)
        {
            aTab[i + 1] = new AppParCurves_ConstraintCouple(theCouples[i].Index, theCouples[i].Cons);
        }
        return aTab;
    }

    private static Point3D Circle(double t) => new(Math.Cos(t * 2.0), Math.Sin(t * 2.0), 0.0);

    // The largest distance between the points of a 3d line and the result, at the computed parameters.
    private static double MaxDistance3d(AppDef_Variational theV, AppDef_MultiLine theLine)
    {
        AppParCurves_MultiBSpCurve aC = theV.Value();
        NCollection_HArray1<double> aPar = theV.Parameters();
        double aMax = 0.0;
        for (int i = 1; i <= theLine.NbMultiPoints(); i++)
        {
            aC.Value(1, aPar[i], out Point3D aP);
            aMax = Math.Max(aMax, aP.Distance(theLine.Value(i).Point(1)));
        }
        return aMax;
    }

    [Fact]
    public void Line3d_IsReproduced()
    {
        AppDef_MultiLine aLine = Line3d(20, t => new Point3D(1.0 + 2.0 * t, -t, 0.5 + 3.0 * t));
        AppDef_Variational aV = new(aLine, 1, 20, Constraints((1, AppParCurves_PassPoint), (20, AppParCurves_PassPoint)),
                                    8, 10, C2, false, true, 1.0e-6);
        Assert.True(aV.IsCreated());
        aV.Approximate();
        Assert.True(aV.IsDone());
        output.WriteLine($"MaxError {aV.MaxError()}, average {aV.AverageError()}, quadratic {aV.QuadraticError()}");
        Assert.True(aV.MaxError() < 1.0e-9);
        Assert.True(MaxDistance3d(aV, aLine) < 1.0e-9);
        Assert.InRange(aV.MaxErrorIndex(), 1, 20);
    }

    [Fact]
    public void Parabola3d_IsApproximated()
    {
        AppDef_MultiLine aLine = Line3d(30, t => new Point3D(t, t * t, 0.5 * t * t - t));
        AppDef_Variational aV = new(aLine, 1, 30, Constraints((1, AppParCurves_PassPoint), (30, AppParCurves_PassPoint)),
                                    8, 10, C2, false, true, 1.0e-5);
        aV.Approximate();
        Assert.True(aV.IsDone());
        output.WriteLine($"MaxError {aV.MaxError()}, average {aV.AverageError()}");
        Assert.True(aV.MaxError() < 1.0e-5);
        // MaxError is measured on the FEmTool curve, before its conversion to a B-spline
        output.WriteLine($"B-spline distance {MaxDistance3d(aV, aLine)}");
        Assert.Equal(aV.MaxError(), MaxDistance3d(aV, aLine), 1.0e-9);
        // the pass points are interpolated
        AppParCurves_MultiBSpCurve aC = aV.Value();
        aC.Value(1, aV.Parameters()[1], out Point3D aFirst);
        aC.Value(1, aV.Parameters()[30], out Point3D aLast);
        Assert.True(aFirst.Distance(aLine.Value(1).Point(1)) < 1.0e-10);
        Assert.True(aLast.Distance(aLine.Value(30).Point(1)) < 1.0e-10);
        // the final projection moves the end parameters by rounding only
        Assert.Equal(0.0, aV.Parameters()[1], 1.0e-12);
        Assert.Equal(1.0, aV.Parameters()[30], 1.0e-12);
    }

    [Theory]
    [InlineData(C0)]
    [InlineData(C1)]
    [InlineData(C2)]
    public void Circle3d_WithEachContinuity(Continuity theCont)
    {
        AppDef_MultiLine aLine = Line3d(40, Circle);
        AppDef_Variational aV = new(aLine, 1, 40, Constraints((1, AppParCurves_PassPoint), (40, AppParCurves_PassPoint)),
                                    10, 20, theCont, false, true, 1.0e-4);
        Assert.Equal(theCont, aV.Continuity());
        aV.Approximate();
        Assert.True(aV.IsDone());
        AppParCurves_MultiBSpCurve aC = aV.Value();
        output.WriteLine($"{theCont}: MaxError {aV.MaxError()}, degree {aC.Degree()}, knots {aV.Knots().Length()}");
        Assert.True(aV.MaxError() < 1.0e-4);
        Assert.True(MaxDistance3d(aV, aLine) < 1.0e-4);
        // the interior multiplicities give the continuity asked for
        NCollection_Array1<int> aMults = aC.Multiplicities();
        int aCont = theCont == C0 ? 0 : theCont == C1 ? 1 : 2;
        for (int i = aMults.Lower() + 1; i < aMults.Upper(); i++)
        {
            Assert.Equal(aC.Degree() - aCont, aMults[i]);
        }
    }

    [Fact]
    public void Circle3d_TangencyAtEnds()
    {
        AppDef_MultiLine aLine = Line3d(25, Circle);
        foreach (int i in new[] { 1, 25 })
        {
            double t = (i - 1) / 24.0;
            AppDef_MultiPointConstraint aMPC = aLine.Value(i);
            aMPC.SetTang(1, new Vector3D(-Math.Sin(2.0 * t), Math.Cos(2.0 * t), 0.0));
        }
        AppDef_Variational aV = new(aLine, 1, 25,
                                    Constraints((1, AppParCurves_TangencyPoint), (25, AppParCurves_TangencyPoint)), 10,
                                    20, C2, false, true, 1.0e-4);
        aV.Approximate();
        Assert.True(aV.IsDone());
        output.WriteLine($"MaxError {aV.MaxError()}");
        Assert.True(aV.MaxError() < 1.0e-4);
        AppParCurves_MultiBSpCurve aC = aV.Value();
        foreach (int i in new[] { 1, 25 })
        {
            aC.D1(1, aV.Parameters()[i], out Point3D aP, out Vector3D aD);
            Assert.True(aP.Distance(aLine.Value(i).Point(1)) < 1.0e-10);
            Assert.True(aD.IsParallel(aLine.Value(i).Tang(1), 1.0e-8), $"tangent at {i}");
        }
    }

    [Fact]
    public void Circle2d_CurvatureAndInteriorPassPoint()
    {
        AppDef_MultiLine aLine = Line2d(31, t => new Point2D(2.0 * Math.Cos(t * 1.5), 2.0 * Math.Sin(t * 1.5)));
        double t0 = 0.0, t1 = 1.0;
        AppDef_MultiPointConstraint aFirst = aLine.Value(1), aLast = aLine.Value(31);
        // unit tangents and the curvature vectors (towards the centre, length 1/R)
        aFirst.SetTang2d(1, new Vector2D(-Math.Sin(1.5 * t0), Math.Cos(1.5 * t0)));
        aFirst.SetCurv2d(1, new Vector2D(-0.5 * Math.Cos(1.5 * t0), -0.5 * Math.Sin(1.5 * t0)));
        aLast.SetTang2d(1, new Vector2D(-Math.Sin(1.5 * t1), Math.Cos(1.5 * t1)));
        aLast.SetCurv2d(1, new Vector2D(-0.5 * Math.Cos(1.5 * t1), -0.5 * Math.Sin(1.5 * t1)));
        AppDef_Variational aV = new(aLine, 1, 31,
                                    Constraints((1, AppParCurves_CurvaturePoint), (16, AppParCurves_PassPoint),
                                                (31, AppParCurves_CurvaturePoint)),
                                    12, 20, C2, false, true, 1.0e-5);
        aV.Approximate();
        Assert.True(aV.IsDone());
        output.WriteLine($"MaxError {aV.MaxError()}");
        Assert.True(aV.MaxError() < 1.0e-4);
        AppParCurves_MultiBSpCurve aC = aV.Value();
        foreach (int i in new[] { 1, 16, 31 })
        {
            aC.D2(1, aV.Parameters()[i], out Point2D aP, out Vector2D aD1, out Vector2D aD2);
            Assert.True(aP.Distance(aLine.Value(i).Point2d(1)) < 1.0e-9, $"pass point {i}");
            if (i != 16)
            {
                Assert.True(aD1.IsParallel(aLine.Value(i).Tang2d(1), 1.0e-8), $"tangent at {i}");
                // curvature of the result
                double aK = Math.Abs(aD1.Crossed(aD2)) / Math.Pow(aD1.Magnitude(), 3);
                Assert.Equal(0.5, aK, 1.0e-6);
            }
        }
    }

    [Fact]
    public void Line3dAnd2d_TwoCurvesInParallel()
    {
        int aNb = 21;
        NCollection_Array1<AppDef_MultiPointConstraint> aTab = new(1, aNb);
        for (int i = 1; i <= aNb; i++)
        {
            double t = (i - 1) / (double)(aNb - 1);
            NCollection_Array1<Point3D> aP3 = new(1, 1);
            NCollection_Array1<Point2D> aP2 = new(1, 1);
            aP3[1] = new Point3D(t, Math.Sin(t), t * t);
            aP2[1] = new Point2D(2.0 * t, 1.0 - t * t);
            aTab[i] = new AppDef_MultiPointConstraint(aP3, aP2);
        }
        AppDef_MultiLine aLine = new(aTab);
        AppDef_Variational aV = new(aLine, 1, aNb,
                                    Constraints((1, AppParCurves_PassPoint), (aNb, AppParCurves_PassPoint)), 8, 10,
                                    C2, false, true, 1.0e-5);
        aV.Approximate();
        Assert.True(aV.IsDone());
        AppParCurves_MultiBSpCurve aC = aV.Value();
        Assert.Equal(2, aC.NbCurves());
        Matrix aDist = new(1, 2, 1, aNb);
        aV.Distance(aDist);
        double aMax = 0.0;
        for (int i = 1; i <= aNb; i++)
        {
            aMax = Math.Max(aMax, Math.Max(aDist[1, i], aDist[2, i]));
        }
        output.WriteLine($"MaxError {aV.MaxError()}, from Distance {aMax}");
        Assert.True(aV.MaxError() < 1.0e-4);
        Assert.True(aMax <= aV.MaxError() + 1.0e-12);
    }

    [Fact]
    public void NoConstraints_IsApproximated()
    {
        AppDef_MultiLine aLine = Line3d(30, t => new Point3D(t, Math.Sin(3.0 * t), 0.0));
        AppDef_Variational aV = new(aLine, 1, 30, new NCollection_HArray1<AppParCurves_ConstraintCouple>(1, 0), 10, 10,
                                    C2, false, true, 1.0e-5);
        aV.Approximate();
        Assert.True(aV.IsDone());
        output.WriteLine($"MaxError {aV.MaxError()}");
        Assert.True(aV.MaxError() < 1.0e-4);
    }

    [Fact]
    public void WithMinMax_ReducesTheMaximumError()
    {
        AppDef_MultiLine aLine = Line3d(40, t => new Point3D(t, Math.Sin(6.0 * t), Math.Cos(4.0 * t)));
        NCollection_HArray1<AppParCurves_ConstraintCouple> aCons =
            Constraints((1, AppParCurves_PassPoint), (40, AppParCurves_PassPoint));
        AppDef_Variational aV = new(aLine, 1, 40, aCons, 8, 4, C2, true, true, 1.0e-3);
        Assert.True(aV.WithMinMax());
        aV.Approximate();
        Assert.True(aV.IsDone());
        output.WriteLine($"MinMax: MaxError {aV.MaxError()}, average {aV.AverageError()}");
        Assert.True(aV.MaxError() < 1.0e-2);
        Assert.True(aV.AverageError() <= aV.MaxError());
    }

    [Fact]
    public void CriteriumWeight_IsNormalised()
    {
        AppDef_MultiLine aLine = Line3d(20, Circle);
        AppDef_Variational aV = new(aLine, 1, 20, Constraints((1, AppParCurves_PassPoint), (20, AppParCurves_PassPoint)));
        aV.CriteriumWeight(out double p1, out double p2, out double p3);
        Assert.Equal((0.4, 0.2, 0.4), (p1, p2, p3));
        aV.SetCriteriumWeight(1.0, 1.0, 2.0);
        aV.CriteriumWeight(out p1, out p2, out p3);
        Assert.Equal((0.25, 0.25, 0.5), (p1, p2, p3));
        aV.SetCriteriumWeight(3, 0.5);
        aV.CriteriumWeight(out p1, out p2, out p3);
        Assert.Equal(1.0, p1 + p2 + p3, 1.0e-15);
        Assert.Equal(0.25 / 1.0, p1, 1.0e-15);
        Assert.Throws<DomainErrorException>(() => aV.SetCriteriumWeight(-1.0, 1.0, 1.0));
        Assert.Throws<DomainErrorException>(() => aV.SetCriteriumWeight(1, -1.0));
        Assert.Throws<ConstructionErrorException>(() => aV.SetCriteriumWeight(4, 1.0));
    }

    [Fact]
    public void SmoothingWeights_ChangeTheResult()
    {
        AppDef_MultiLine aLine = Line3d(30, t => new Point3D(t, 0.2 * Math.Sin(9.0 * t), 0.0));
        NCollection_HArray1<AppParCurves_ConstraintCouple> aCons =
            Constraints((1, AppParCurves_PassPoint), (30, AppParCurves_PassPoint));
        AppDef_Variational aTension = new(aLine, 1, 30, aCons, 8, 1, C2, false, false, 0.05);
        aTension.SetCriteriumWeight(1.0, 0.0, 0.0);
        aTension.Approximate();
        AppDef_Variational aJerk = new(aLine, 1, 30, aCons, 8, 1, C2, false, false, 0.05);
        aJerk.SetCriteriumWeight(0.0, 0.0, 1.0);
        aJerk.Approximate();
        Assert.True(aTension.IsDone() && aJerk.IsDone());
        aTension.Criterium(out double t1, out double t2, out double t3);
        aJerk.Criterium(out double j1, out double j2, out double j3);
        output.WriteLine($"tension-weighted: {t1} {t2} {t3}, err {aTension.MaxError()}");
        output.WriteLine($"jerk-weighted:    {j1} {j2} {j3}, err {aJerk.MaxError()}");
        Assert.NotEqual(aTension.MaxError(), aJerk.MaxError());
        Assert.True(t1 <= j1 * (1 + 1e-9), "the tension-weighted fit has the smaller tension");
        Assert.True(j3 <= t3 * (1 + 1e-9), "the jerk-weighted fit has the smaller jerk");
    }

    [Fact]
    public void SetKnots_WithoutCutting_KeepsTheKnots()
    {
        AppDef_MultiLine aLine = Line3d(60, t => new Point3D(t, Math.Sin(9.0 * t), Math.Cos(7.0 * t)));
        AppDef_Variational aV = new(aLine, 1, 60, Constraints((1, AppParCurves_PassPoint), (60, AppParCurves_PassPoint)),
                                    6, 10, C2, false, false, 1.0e-5);
        Assert.False(aV.WithCutting());
        aV.Approximate();
        Assert.True(aV.IsDone());
        Assert.Equal(2, aV.Knots().Length());
        Assert.Equal(0.0, aV.Knots()[1]);
        Assert.Equal(1.0, aV.Knots()[2]);

        // SetKnots copies the values into the stored knots (two values: one segment)
        NCollection_HArray1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        Assert.True(aV.SetKnots(aKnots));
        aKnots[2] = 7.0;
        Assert.Equal(1.0, aV.Knots()[2]);

        // cutting allowed: more segments
        Assert.True(aV.SetWithCutting(true));
        double aOneSegmentError = aV.MaxError();
        aV.Approximate();
        output.WriteLine($"one segment: MaxError {aOneSegmentError}; cut: knots {aV.Knots().Length()}, MaxError {aV.MaxError()}");
        Assert.True(aV.Knots().Length() > 2);
        Assert.True(aV.MaxError() < aOneSegmentError);
    }

    [Fact]
    public void Accessors_AndSetters()
    {
        AppDef_MultiLine aLine = Line3d(20, Circle);
        AppDef_Variational aV = new(aLine, 1, 20, Constraints((1, AppParCurves_PassPoint), (20, AppParCurves_PassPoint)),
                                    9, 7, C1, false, true, 0.01, 3);
        Assert.Equal(9, aV.MaxDegree());
        Assert.Equal(7, aV.MaxSegment());
        Assert.Equal(C1, aV.Continuity());
        Assert.Equal(0.01, aV.Tolerance());
        Assert.Equal(3, aV.NbIterations());
        Assert.False(aV.IsDone());
        Assert.Throws<NotDoneException>(() => aV.Value());
        Assert.Throws<NotDoneException>(() => aV.MaxError());
        Assert.True(aV.SetMaxDegree(6));
        Assert.Equal(6, aV.MaxDegree());
        Assert.True(aV.SetMaxSegment(5));
        Assert.True(aV.SetContinuity(C2));
        Assert.Throws<ConstructionErrorException>(() => aV.SetContinuity(C3));
        aV.SetNbIterations(1);
        Assert.Equal(1, aV.NbIterations());
        aV.Approximate();
        Assert.True(aV.IsDone());
        Assert.True(aV.Value().Degree() <= 6);
        // OCCT clamps MaxDegree to 30, but FEmTool's criteria take at most 14 (decision 148)
        Assert.Throws<ConstructionErrorException>(() => new AppDef_Variational(aLine, 1, 20, Constraints(), 40));
    }

    [Fact]
    public void OverConstrained_IsDetected()
    {
        AppDef_MultiLine aLine = Line3d(10, Circle);
        (int, AppParCurves_Constraint)[] aAll = new (int, AppParCurves_Constraint)[10];
        for (int i = 0; i < 10; i++)
        {
            aAll[i] = (i + 1, AppParCurves_PassPoint);
        }
        // (6 - 2) * 1 - 10 < 0
        AppDef_Variational aV = new(aLine, 1, 10, Constraints(aAll), 6, 1, C2, false, false);
        Assert.True(aV.IsOverConstrained());
        Assert.False(aV.IsCreated());
        Assert.Throws<NotDoneException>(() => aV.Approximate());
        // fewer constraints: no longer over-constrained, but OCCT's Init never clears the flag, so SetConstraints
        // still returns false (occt-issues.md)
        Assert.False(aV.SetConstraints(Constraints((1, AppParCurves_PassPoint))));
        Assert.True(aV.IsCreated());
        Assert.True(aV.IsOverConstrained());
        aV.Approximate();
        Assert.True(aV.IsDone());
    }

    [Fact]
    public void InvalidArguments_Throw()
    {
        AppDef_MultiLine aLine = Line3d(10, Circle);
        Assert.Throws<DomainErrorException>(() => new AppDef_Variational(aLine, 1, 10, Constraints(), 0));
        Assert.Throws<DomainErrorException>(() => new AppDef_Variational(aLine, 1, 10, Constraints(), 8, 0));
        Assert.Throws<ConstructionErrorException>(() => new AppDef_Variational(aLine, 1, 10, Constraints(), 8, 10, G1));
        // a tangency point without tangents
        Assert.Throws<ConstructionErrorException>(
            () => new AppDef_Variational(aLine, 1, 10, Constraints((1, AppParCurves_TangencyPoint))));
        // coincident points: no length
        AppDef_MultiLine aPoint = Line3d(5, _ => new Point3D(1, 2, 3));
        Assert.Throws<ConstructionErrorException>(() => new AppDef_Variational(aPoint, 1, 5, Constraints()));
    }

    [Fact]
    public void TwoPoints_ReadPastThePoints()
    {
        // OCCT estimates a tangent at the third point of a two-point line, reading past the points (occt-issues.md);
        // the port's range checks throw.
        AppDef_MultiLine aLine = Line3d(2, t => new Point3D(t, 0, 0));
        Assert.ThrowsAny<FailureException>(() => new AppDef_Variational(aLine, 1, 2, Constraints()));
    }

    [Fact]
    public void LinearCriteria_Basics()
    {
        AppDef_MultiLine aLine = Line3d(10, Circle);
        AppDef_LinearCriteria aJ = new(aLine, 1, 10);
        Assert.Null(aJ.Curve());
        Assert.Throws<DomainErrorException>(() => aJ.AssemblyTable());
        Assert.Throws<DomainErrorException>(() => aJ.SetWeight(-1.0, 1.0, 1.0, 1.0, 1.0));
        Assert.Throws<DomainErrorException>(() => aJ.SetWeight(1.0, 1.0, -1.0, 1.0, 1.0));

        aJ.SetWeight(2.0, 3.0, 1.0, 1.0, 2.0);
        aJ.GetWeight(out double aQuad, out double aQual);
        Assert.Equal((2.0, 3.0), (aQuad, aQual));

        aJ.EstLength() = 4.5;
        Assert.Equal(4.5, aJ.EstLength());
        aJ.SetEstimation(1.0, 2.0, 3.0);
        aJ.GetEstimation(out double e1, out double e2, out double e3);
        Assert.Equal((1.0, 2.0, 3.0), (e1, e2, e3));

        // a curve of 2 elements, dimension 3, degree 7 with C1 Hermite constraints
        HermitJacobi aBase = new(7, C1);
        FEmTool_Curve aCurve = new(3, 2, aBase, 1.0e-9);
        aCurve.Knots()[1] = 0.0;
        aCurve.Knots()[2] = 0.5;
        aCurve.Knots()[3] = 1.0;
        aJ.SetCurve(aCurve);
        Assert.Same(aCurve, aJ.Curve());

        NCollection_HArray2<int> aDep = aJ.DependenceTable();
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                Assert.Equal(i == j ? 1 : 0, aDep[i, j]);
            }
        }

        // the assembly table: (dimension, element) -> global indices of the 8 coefficients; the elements share their
        // two C1 end values (2 coefficients), so a dimension has 8 + 6 = 14 variables
        NCollection_HArray2<NCollection_HArray1<int>> aTab = aJ.AssemblyTable();
        Assert.Equal(3, aTab.ColLength());
        Assert.Equal(2, aTab.RowLength());
        HashSet<int> aDim1 = [];
        for (int el = 1; el <= 2; el++)
        {
            for (int i = 0; i <= 7; i++)
            {
                aDim1.Add(aTab[1, el][i]);
            }
        }
        Assert.Equal(14, aDim1.Count);
        Assert.Equal(15, aTab[2, 1][0] - aTab[1, 1][0] + 1);
        // the end values of element 1 are the start values of element 2
        Assert.Equal(aTab[1, 1][2], aTab[1, 2][0]);
        Assert.Equal(aTab[1, 1][3], aTab[1, 2][1]);
    }
}
