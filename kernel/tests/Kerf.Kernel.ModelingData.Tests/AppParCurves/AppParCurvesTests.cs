// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.Constraint;

namespace Kerf.Kernel.ModelingData.Tests;

using LeastSquare = global::Kerf.Engine.Approximation.LeastSquare<AppParCurves_TestLine, AppParCurves_TestLineTool>;
using ResolConstraint = global::Kerf.Engine.Approximation.ResolConstraint<AppParCurves_TestLine, AppParCurves_TestLineTool>;
using Function = global::Kerf.Engine.Approximation.Function<AppParCurves_TestLine, AppParCurves_TestLineTool>;
using Gradient = global::Kerf.Engine.Approximation.Gradient<AppParCurves_TestLine, AppParCurves_TestLineTool>;
using BSpFunction = global::Kerf.Engine.Approximation.BSpFunction<AppParCurves_TestLine, AppParCurves_TestLineTool>;
using BSpGradient = global::Kerf.Engine.Approximation.BSpGradient<AppParCurves_TestLine, AppParCurves_TestLineTool>;

/// <summary>
/// AppParCurves (no OCCT GTests): the basis functions, the data classes, and the generic algorithms instantiated
/// over a test MultiLine, on data sampled from known Bezier and B-spline curves, which the least squares must
/// reproduce exactly.
/// </summary>
public class AppParCurvesTests
{
    // A cubic MultiCurve: one 3d curve and one 2d curve
    private static global::Kerf.Engine.Approximation.MultiCurve Cubic()
    {
        Point3D[] aP3d = [new(0, 0, 0), new(1, 2, 0.5), new(3, 2.5, -1), new(4, 0, 1)];
        Point2D[] aP2d = [new(0, 1), new(1, 3), new(2, -1), new(4, 0.5)];
        global::Kerf.Engine.Approximation.MultiCurve aCurve = new(4);
        for (int i = 1; i <= 4; i++)
        {
            global::Kerf.Engine.Approximation.MultiPoint aMP = new(1, 1);
            aMP.SetPoint(1, aP3d[i - 1]);
            aMP.SetPoint2d(2, aP2d[i - 1]);
            aCurve.SetValue(i, aMP);
        }
        return aCurve;
    }

    // A cubic B-spline MultiCurve with an interior knot at 0.4: 5 poles, one 3d curve and one 2d curve
    private static global::Kerf.Engine.Approximation.MultiBSpCurve CubicBSpline()
    {
        Point3D[] aP3d = [new(0, 0, 0), new(1, 2, 0.5), new(2, 2.5, -1), new(3, 0, 1), new(4, 1, 0)];
        Point2D[] aP2d = [new(0, 1), new(1, 3), new(2, -1), new(3, 0.5), new(5, 1)];
        NCollection_Array1<global::Kerf.Engine.Approximation.MultiPoint> aPoles = new(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            global::Kerf.Engine.Approximation.MultiPoint aMP = new(1, 1);
            aMP.SetPoint(1, aP3d[i - 1]);
            aMP.SetPoint2d(2, aP2d[i - 1]);
            aPoles[i] = aMP;
        }
        return new global::Kerf.Engine.Approximation.MultiBSpCurve(aPoles, Knots(), Mults());
    }

    private static NCollection_Array1<double> Knots() => NCollection_Array1<double>.From([0.0, 0.4, 1.0]);

    private static NCollection_Array1<int> Mults() => NCollection_Array1<int>.From([4, 1, 4]);

    private static double[] Params(int theNb)
    {
        double[] aParams = new double[theNb];
        for (int i = 0; i < theNb; i++)
        {
            // uneven but increasing
            double t = (double)i / (theNb - 1);
            aParams[i] = t * t * (3 - 2 * t) * 0.5 + t * 0.5;
        }
        return aParams;
    }

    private static math_Vector Vector(double[] theValues)
    {
        math_Vector aV = new(1, theValues.Length);
        for (int i = 0; i < theValues.Length; i++)
        {
            aV[i + 1] = theValues[i];
        }
        return aV;
    }

    private static NCollection_HArray1<global::Kerf.Engine.Approximation.ConstraintCouple> Constraints(params (int, global::Kerf.Engine.Approximation.Constraint)[] theCouples)
    {
        NCollection_HArray1<global::Kerf.Engine.Approximation.ConstraintCouple> aTab = new(1, theCouples.Length);
        for (int i = 0; i < theCouples.Length; i++)
        {
            aTab[i + 1] = new global::Kerf.Engine.Approximation.ConstraintCouple(theCouples[i].Item1, theCouples[i].Item2);
        }
        return aTab;
    }

    private static void AssertSamePoles(global::Kerf.Engine.Approximation.MultiCurve theExpected, global::Kerf.Engine.Approximation.MultiCurve theActual, double theTol)
    {
        Assert.Equal(theExpected.NbPoles(), theActual.NbPoles());
        for (int i = 1; i <= theExpected.NbPoles(); i++)
        {
            Assert.True(theExpected.Pole(1, i).DistanceTo(theActual.Pole(1, i)) < theTol, $"3d pole {i}");
            Assert.True(theExpected.Pole2d(2, i).DistanceTo(theActual.Pole2d(2, i)) < theTol, $"2d pole {i}");
        }
    }

    // ---------------------------------------------------------------- basis functions

    [Fact]
    public void Bernstein_PartitionOfUnityAndDerivatives()
    {
        const int aNbPoles = 6;
        math_Vector U = Vector([0.0, 0.1, 0.37, 0.5, 0.9, 1.0]);
        global::Kerf.Engine.Numerics.Matrix A = new(1, 6, 1, aNbPoles), DA = new(1, 6, 1, aNbPoles), A2 = new(1, 6, 1, aNbPoles);
        global::Kerf.Engine.Approximation.AppParCurves.Bernstein(aNbPoles, U, A, DA);
        global::Kerf.Engine.Approximation.AppParCurves.BernsteinMatrix(aNbPoles, U, A2);
        const double h = 1e-6;
        for (int i = 1; i <= 6; i++)
        {
            double aSum = 0, aDSum = 0;
            for (int j = 1; j <= aNbPoles; j++)
            {
                aSum += A[i, j];
                aDSum += DA[i, j];
                Assert.Equal(A[i, j], A2[i, j]);
                // B_j(u) = C(n, j-1) u^(j-1) (1-u)^(n-j+1)
                int n = aNbPoles - 1, k = j - 1;
                double aBinom = Binomial(n, k);
                Func<double, double> B = u => aBinom * Math.Pow(u, k) * Math.Pow(1 - u, n - k);
                Assert.Equal(B(U[i]), A[i, j], 1e-14);
                Assert.Equal((B(U[i] + h) - B(U[i] - h)) / (2 * h), DA[i, j], 1e-6);
            }
            Assert.Equal(1.0, aSum, 1e-14);
            Assert.Equal(0.0, aDSum, 1e-12);
        }
    }

    private static double Binomial(int n, int k)
    {
        double r = 1;
        for (int i = 1; i <= k; i++)
        {
            r = r * (n - k + i) / i;
        }
        return r;
    }

    [Theory]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(7)]
    public void SecondDerivativeBernstein_MatchesDifferences(int theDegree)
    {
        int aNbPoles = theDegree + 1;
        const double h = 1e-4;
        foreach (double u in new[] { 0.2, 0.5, 0.83 })
        {
            math_Vector DDA = new(1, aNbPoles);
            global::Kerf.Engine.Approximation.AppParCurves.SecondDerivativeBernstein(u, DDA);
            math_Vector U = Vector([u - h, u + h]);
            global::Kerf.Engine.Numerics.Matrix A = new(1, 2, 1, aNbPoles), DA = new(1, 2, 1, aNbPoles);
            global::Kerf.Engine.Approximation.AppParCurves.Bernstein(aNbPoles, U, A, DA);
            for (int j = 1; j <= aNbPoles; j++)
            {
                Assert.Equal((DA[2, j] - DA[1, j]) / (2 * h), DDA[j], 1e-5);
            }
        }
    }

    [Fact]
    public void SecondDerivativeBernstein_DegreeOne()
    {
        math_Vector DDA = new(1, 2, 7.0);
        global::Kerf.Engine.Approximation.AppParCurves.SecondDerivativeBernstein(0.3, DDA);
        Assert.Equal(0.0, DDA[1]);
        Assert.Equal(0.0, DDA[2]);
    }

    [Fact]
    public void SplineFunction_IsTheBSplineBasis()
    {
        // flat knots of the cubic with an interior knot at 0.4
        math_Vector aFlat = Vector([0, 0, 0, 0, 0.4, 1, 1, 1, 1]);
        math_Vector U = Vector([0.0, 0.1, 0.4, 0.55, 0.99, 1.0]);
        global::Kerf.Engine.Numerics.Matrix A = new(1, 6, 1, 5), DA = new(1, 6, 1, 5);
        math_IntegerVector anIndex = new(1, 6);
        global::Kerf.Engine.Approximation.AppParCurves.SplineFunction(5, 3, U, aFlat, A, DA, anIndex);

        // compare with the curve: sum_j A(i, j) P_j against BSplCLib's evaluation of 1d poles 1..5
        global::Kerf.Engine.Approximation.MultiBSpCurve aCurve = CubicBSpline();
        for (int i = 1; i <= 6; i++)
        {
            double aSum = 0;
            Xyz aP = new(), aD = new();
            for (int j = 1; j <= 5; j++)
            {
                aSum += A[i, j];
                aP += aCurve.Pole(1, j).XYZ() * A[i, j];
                aD += aCurve.Pole(1, j).XYZ() * DA[i, j];
                if (j <= anIndex[i] || j > anIndex[i] + 4)
                {
                    Assert.Equal(0.0, A[i, j]);
                }
            }
            Assert.Equal(1.0, aSum, 1e-14);
            aCurve.D1(1, U[i], out Point3D aPnt, out Vector3D aV);
            Assert.True(aPnt.XYZ().IsEqual(aP, 1e-13));
            Assert.True(aV.XYZ().IsEqual(aD, 1e-12));
        }
    }

    // ---------------------------------------------------------------- data classes

    [Fact]
    public void ConstraintCouple_DefaultsAndTwins()
    {
        global::Kerf.Engine.Approximation.ConstraintCouple aC = new();
        Assert.Equal(-1, aC.Index());
        Assert.Equal(NoConstraint, aC.Constraint());
        global::Kerf.Engine.Approximation.ConstraintCouple aD = aC.WithIndex(3).WithConstraint(TangencyPoint);
        Assert.Equal(new global::Kerf.Engine.Approximation.ConstraintCouple(3, TangencyPoint), aD);
        Assert.Equal(-1, aC.Index());
        // arrays default-construct their elements as C++ does
        NCollection_Array1<global::Kerf.Engine.Approximation.ConstraintCouple> anArray = new(1, 2);
        Assert.Equal(-1, anArray[2].Index());
    }

    [Fact]
    public void MultiPoint_IndexesAndTransform()
    {
        global::Kerf.Engine.Approximation.MultiPoint aMP = new(NCollection_Array1<Point3D>.From([new Point3D(1, 2, 3)]),
                                          NCollection_Array1<Point2D>.From([new Point2D(4, 5), new Point2D(6, 7)], 0));
        Assert.Equal(1, aMP.PointCount);
        Assert.Equal(2, aMP.Points2dCount);
        Assert.Equal(3, aMP.Dimension(1));
        Assert.Equal(2, aMP.Dimension(3));
        Assert.Equal(new Point2D(6, 7), aMP.Point2d(3));
        Assert.ThrowsAny<OutOfRangeException>(() => aMP.Point(2));
        Assert.ThrowsAny<OutOfRangeException>(() => aMP.Point2d(1));
        Assert.ThrowsAny<OutOfRangeException>(() => aMP.Dimension(4));
        aMP.Transform(1, 1, 2, 0, 1, 0, -1);
        Assert.Equal(new Point3D(3, 2, -3), aMP.Point(1));
        aMP.Transform2d(2, 1, 1, 1, 1);
        Assert.Equal(new Point2D(5, 6), aMP.Point2d(2));
        Assert.ThrowsAny<OutOfRangeException>(() => aMP.Transform2d(1, 0, 1, 0, 1));
        Assert.Contains("3D-Point #1", aMP.ToString());
        Assert.Contains(" Pole z = -3", aMP.ToString());

        // OCCT's copy shares the points
        global::Kerf.Engine.Approximation.MultiPoint aCopy = new(aMP);
        aCopy.SetPoint(1, new Point3D(9, 9, 9));
        Assert.Equal(new Point3D(9, 9, 9), aMP.Point(1));
    }

    [Fact]
    public void MultiCurve_EvaluatesBezierCurves()
    {
        global::Kerf.Engine.Approximation.MultiCurve aCurve = Cubic();
        Assert.Equal(2, aCurve.NbCurves());
        Assert.Equal(4, aCurve.NbPoles());
        Assert.Equal(3, aCurve.Degree());
        Assert.Equal(3, aCurve.Dimension(1));
        Assert.Equal(2, aCurve.Dimension(2));
        Assert.Equal(0, new global::Kerf.Engine.Approximation.MultiCurve().NbCurves());
        Assert.Equal(0, new global::Kerf.Engine.Approximation.MultiCurve(3).NbCurves()); // empty MultiPoints, as C++ default-constructs them

        aCurve.D2(1, 0.0, out Point3D aP, out Vector3D aV1, out Vector3D aV2);
        Assert.Equal(new Point3D(0, 0, 0), aP);
        Assert.True(aV1.IsEqual(new Vector3D(3, 6, 1.5), 1e-14, 1e-14));
        aCurve.D2(2, 1.0, out Point2D aP2, out Vector2D aV12, out _);
        Assert.Equal(new Point2D(4, 0.5), aP2);
        Assert.True(aV12.IsEqual(new Vector2D(6, 4.5), 1e-14, 1e-14));
        Assert.ThrowsAny<OutOfRangeException>(() => aCurve.Value(2, 0.5, out Point3D _));

        NCollection_Array1<Point2D> aPoles2d = new(1, 4);
        aCurve.Curve(2, aPoles2d);
        Assert.Equal(new Point2D(2, -1), aPoles2d[3]);

        aCurve.Transform(1, 1, 1, 0, 1, 0, 1);
        Assert.Equal(new Point3D(5, 0, 1), aCurve.Pole(1, 4));
        Assert.Contains("2 Bezier curves of degree 3", aCurve.ToString());
    }

    [Fact]
    public void MultiBSpCurve_EvaluatesAndCopiesKnots()
    {
        global::Kerf.Engine.Approximation.MultiBSpCurve aCurve = CubicBSpline();
        Assert.Equal(3, aCurve.Degree());
        Assert.Equal(5, aCurve.NbPoles());
        aCurve.Value(1, 0.0, out Point3D aP0);
        aCurve.Value(1, 1.0, out Point3D aP1);
        Assert.True(aP0.IsEqual(new Point3D(0, 0, 0), 1e-15));
        Assert.True(aP1.IsEqual(new Point3D(4, 1, 0), 1e-14));
        aCurve.D1(2, 1.0, out Point2D _, out Vector2D aV);
        // end derivative of a clamped cubic: 3 (P5 - P4) / (1 - 0.4)
        Assert.True(aV.IsEqual(new Vector2D(10, 2.5), 1e-12, 1e-12));

        NCollection_Array1<double> aKnots = aCurve.Knots();
        aKnots[2] = 0.9;
        Assert.Equal(0.4, aCurve.Knots()[2]);
        Assert.Equal(4, aCurve.Multiplicities()[3]);

        // OCCT's copy shares the poles and copies the degree
        global::Kerf.Engine.Approximation.MultiBSpCurve aCopy = new(aCurve);
        Assert.Equal(3, aCopy.Degree());
        aCopy.Transform2d(2, 0, 2, 0, 2);
        Assert.Equal(new Point2D(10, 2), aCurve.Pole2d(2, 5));
        Assert.Contains("2 BSpline curves", aCurve.ToString());
    }

    // ---------------------------------------------------------------- least squares

    [Theory]
    [InlineData(NoConstraint, NoConstraint)]
    [InlineData(PassPoint, NoConstraint)]
    [InlineData(NoConstraint, PassPoint)]
    [InlineData(PassPoint, PassPoint)]
    [InlineData(TangencyPoint, PassPoint)]
    [InlineData(PassPoint, TangencyPoint)]
    public void LeastSquare_ReproducesABezierCurve(global::Kerf.Engine.Approximation.Constraint theFirst, global::Kerf.Engine.Approximation.Constraint theLast)
    {
        global::Kerf.Engine.Approximation.MultiCurve aRef = Cubic();
        double[] aParams = Params(12);
        AppParCurves_TestLine aLine = AppParCurves_TestLine.Sample(aRef, 1, 1, aParams);
        aLine.SetTangents(aRef, 1, aParams[0]);
        aLine.SetTangents(aRef, 12, aParams[11]);

        LeastSquare aLSQ = new(aLine, 1, 12, theFirst, theLast, Vector(aParams), 4);
        Assert.True(aLSQ.IsDone);
        AssertSamePoles(aRef, aLSQ.BezierValue(), 1e-10);

        aLSQ.Error(out double F, out double aMax3d, out double aMax2d);
        Assert.True(F < 1e-20);
        Assert.True(aMax3d < 1e-10);
        Assert.True(aMax2d < 1e-10);
        global::Kerf.Engine.Numerics.Matrix aDist = aLSQ.Distance();
        Assert.True(aDist[5, 1] < 1e-10);
        Assert.Throws<NoSuchObjectException>(() => new LeastSquare(aLine, Knots(), Mults(), 1, 12, theFirst, theLast,
                                                                   Vector(aParams), 5).BezierValue());

        if (theFirst == TangencyPoint)
        {
            // P2 = P1 + lambda1 * V1, with V1 = 3 (P2 - P1)
            Assert.Equal(1.0 / 3.0, aLSQ.FirstLambda(), 1e-10);
        }
        if (theLast == TangencyPoint)
        {
            Assert.Equal(1.0 / 3.0, aLSQ.LastLambda(), 1e-10);
        }
    }

    [Fact]
    public void LeastSquare_BothTangencies()
    {
        global::Kerf.Engine.Approximation.MultiCurve aRef = Cubic();
        double[] aParams = Params(12);
        AppParCurves_TestLine aLine = AppParCurves_TestLine.Sample(aRef, 1, 1, aParams);
        aLine.SetTangents(aRef, 1, aParams[0]);
        aLine.SetTangents(aRef, 12, aParams[11]);

        // four poles are all fixed by two tangencies: OCCT's protection for overconstrained cases
        LeastSquare aLSQ = new(aLine, 1, 12, TangencyPoint, TangencyPoint, Vector(aParams), 4);
        Assert.False(aLSQ.IsDone);

        // with five poles, the degree-4 curve is the cubic
        aLSQ = new LeastSquare(aLine, 1, 12, TangencyPoint, TangencyPoint, Vector(aParams), 5);
        Assert.True(aLSQ.IsDone);
        global::Kerf.Engine.Approximation.MultiCurve aResult = aLSQ.BezierValue();
        foreach (double u in new[] { 0.0, 0.3, 0.71, 1.0 })
        {
            aRef.Value(1, u, out Point3D aP);
            aResult.Value(1, u, out Point3D aQ);
            Assert.True(aP.DistanceTo(aQ) < 1e-10);
            aRef.Value(2, u, out Point2D aP2);
            aResult.Value(2, u, out Point2D aQ2);
            Assert.True(aP2.DistanceTo(aQ2) < 1e-10);
        }
        // P2 = P1 + lambda1 * V1, with V1 = 4 (P2 - P1) for the degree-4 form
        Assert.Equal(0.25, aLSQ.FirstLambda(), 1e-10);
        Assert.Equal(0.25, aLSQ.LastLambda(), 1e-10);
    }

    [Fact]
    public void LeastSquare_TangencyWithoutTangentsBecomesPassPoint()
    {
        global::Kerf.Engine.Approximation.MultiCurve aRef = Cubic();
        double[] aParams = Params(10);
        AppParCurves_TestLine aLine = AppParCurves_TestLine.Sample(aRef, 1, 1, aParams);
        LeastSquare aLSQ = new(aLine, 1, 10, TangencyPoint, CurvaturePoint, Vector(aParams), 4);
        Assert.True(aLSQ.IsDone);
        AssertSamePoles(aRef, aLSQ.BezierValue(), 1e-10);
    }

    [Fact]
    public void LeastSquare_PerformWithGivenTangents()
    {
        global::Kerf.Engine.Approximation.MultiCurve aRef = Cubic();
        double[] aParams = Params(10);
        AppParCurves_TestLine aLine = AppParCurves_TestLine.Sample(aRef, 1, 1, aParams);
        LeastSquare aLSQ = new(aLine, 1, 10, PassPoint, PassPoint, 5);

        // tangents (3d then 2d coordinates) of the cubic, as a degree-4 curve: P2 = P1 + l1 V1 with V1 = derivative
        aRef.D1(1, 0.0, out Point3D _, out Vector3D aV3a);
        aRef.D1(2, 0.0, out Point2D _, out Vector2D aV2a);
        aRef.D1(1, 1.0, out Point3D _, out Vector3D aV3b);
        aRef.D1(2, 1.0, out Point2D _, out Vector2D aV2b);
        math_Vector V1t = Vector([aV3a.X, aV3a.Y, aV3a.Z, aV2a.X, aV2a.Y]);
        math_Vector V2t = Vector([aV3b.X, aV3b.Y, aV3b.Z, aV2b.X, aV2b.Y]);
        aLSQ.Perform(Vector(aParams), V1t, V2t, 0.25, 0.25);
        Assert.True(aLSQ.IsDone);
        global::Kerf.Engine.Approximation.MultiCurve aResult = aLSQ.BezierValue();
        Assert.Equal(5, aResult.NbPoles());
        foreach (double u in new[] { 0.0, 0.3, 0.71, 1.0 })
        {
            aRef.Value(1, u, out Point3D aP);
            aResult.Value(1, u, out Point3D aQ);
            Assert.True(aP.DistanceTo(aQ) < 1e-10);
            aRef.Value(2, u, out Point2D aP2);
            aResult.Value(2, u, out Point2D aQ2);
            Assert.True(aP2.DistanceTo(aQ2) < 1e-10);
        }
    }

    [Fact]
    public void LeastSquare_ReproducesABSplineCurve()
    {
        global::Kerf.Engine.Approximation.MultiBSpCurve aRef = CubicBSpline();
        double[] aParams = Params(15);
        AppParCurves_TestLine aLine = AppParCurves_TestLine.Sample(aRef, 1, 1, aParams);

        foreach (var (aFirst, aLast) in new[] { (NoConstraint, NoConstraint),
                                                (PassPoint, PassPoint) })
        {
            LeastSquare aLSQ = new(aLine, Knots(), Mults(), 1, 15, aFirst, aLast, Vector(aParams), 5);
            Assert.True(aLSQ.IsDone);
            global::Kerf.Engine.Approximation.MultiBSpCurve aResult = aLSQ.BSplineValue();
            Assert.Equal(3, aResult.Degree());
            Assert.Equal(0.4, aResult.Knots()[2]);
            AssertSamePoles(aRef, aResult, 1e-10);

            math_Vector aGrad = new(1, 15);
            aLSQ.ErrorGradient(aGrad, out double F, out double aMax3d, out double aMax2d);
            Assert.True(F < 1e-20 && aMax3d < 1e-10 && aMax2d < 1e-10);
            Assert.True(aGrad.Norm() < 1e-9);
            Assert.Equal(15, aLSQ.KIndex().Length());
            Assert.Equal(15, aLSQ.FunctionMatrix().RowNumber());
            Assert.Equal(5, aLSQ.Poles().RowNumber());
            Assert.Equal(5, aLSQ.Points().ColNumber());
        }
    }

    [Fact]
    public void LeastSquare_NotDoneThrows()
    {
        AppParCurves_TestLine aLine = AppParCurves_TestLine.Sample(Cubic(), 1, 1, Params(6));
        // two poles, both fixed by tangency constraints: nothing left to compute
        LeastSquare aLSQ = new(aLine, 1, 6, PassPoint, PassPoint, 2);
        aLSQ.Perform(Vector(Params(6)));
        Assert.False(aLSQ.IsDone);
        Assert.Throws<NotDoneException>(() => aLSQ.BSplineValue());
        Assert.Throws<NotDoneException>(() => aLSQ.Error(out _, out _, out _));
        Assert.Throws<NotDoneException>(() => aLSQ.FunctionMatrix());
    }

    // ---------------------------------------------------------------- constraints and gradients

    // Points near a helix, which no cubic reproduces
    private static AppParCurves_TestLine Helix(int theNb, out double[] theParams, int theNbP2d = 1)
    {
        theParams = new double[theNb];
        AppParCurves_TestLine aLine = new(theNb, 1, theNbP2d);
        for (int i = 0; i < theNb; i++)
        {
            double t = (double)i / (theNb - 1);
            theParams[i] = t;
            aLine.P3d[i][0] = new Point3D(Math.Cos(2.5 * t), Math.Sin(2.5 * t), 0.8 * t);
            if (theNbP2d > 0)
            {
                aLine.P2d[i][0] = new Point2D(2 * t, Math.Sin(3 * t));
            }
        }
        return aLine;
    }

    [Fact]
    public void ResolConstraint_MakesTheCurvePassThroughInteriorPoints()
    {
        // 3d only: OCCT reads the 2d passing points out of bounds (occt-issues.md)
        AppParCurves_TestLine aLine = Helix(15, out double[] aParams, 0);
        math_Vector U = Vector(aParams);
        LeastSquare aLSQ = new(aLine, 1, 15, PassPoint, PassPoint, U, 4);
        global::Kerf.Engine.Approximation.MultiCurve aCurve = aLSQ.BezierValue();
        aCurve.Value(1, aParams[7], out Point3D aBefore);
        Assert.True(aBefore.DistanceTo(aLine.P3d[7][0]) > 1e-4);

        var aConstraints = Constraints((1, PassPoint), (8, PassPoint), (15, PassPoint));
        ResolConstraint aResol = new(aLine, aCurve, 1, 15, aConstraints, aLSQ.FunctionMatrix(), aLSQ.DerivativeFunctionMatrix());
        Assert.True(aResol.IsDone);
        // 3 passing points, 3 coordinates each
        Assert.Equal(9, aResol.ConstraintMatrix().RowNumber());
        Assert.Equal(12, aResol.ConstraintMatrix().ColNumber());
        Assert.Equal(9, aResol.Duale().Length());
        Assert.Equal(9, aResol.InverseMatrix().RowNumber());

        foreach (int k in new[] { 0, 7, 14 })
        {
            aCurve.Value(1, aParams[k], out Point3D aP);
            Assert.True(aP.DistanceTo(aLine.P3d[k][0]) < 1e-8, $"3d point {k + 1}");
        }

        global::Kerf.Engine.Numerics.Matrix aDK = aResol.ConstraintDerivative(aLine, U, 3, aLSQ.DerivativeFunctionMatrix());
        Assert.Equal(aLSQ.DerivativeFunctionMatrix()[8, 2], aDK[2, 2]);
    }

    [Fact]
    public void ResolConstraint_InteriorTangency3d()
    {
        AppParCurves_TestLine aLine = new(15, 1, 0);
        double[] aParams = new double[15];
        for (int i = 0; i < 15; i++)
        {
            double t = i / 14.0;
            aParams[i] = t;
            aLine.P3d[i][0] = new Point3D(Math.Cos(2.5 * t), Math.Sin(2.5 * t), 0.8 * t);
        }
        Vector3D aTangent = new(-Math.Sin(2.5 * 0.5), Math.Cos(2.5 * 0.5), 0.3);
        aLine.Tangents[8] = ([aTangent], []);
        math_Vector U = Vector(aParams);
        LeastSquare aLSQ = new(aLine, 1, 15, PassPoint, PassPoint, U, 5);
        global::Kerf.Engine.Approximation.MultiCurve aCurve = aLSQ.BezierValue();

        var aConstraints = Constraints((1, PassPoint), (8, TangencyPoint), (15, PassPoint));
        ResolConstraint aResol = new(aLine, aCurve, 1, 15, aConstraints, aLSQ.FunctionMatrix(), aLSQ.DerivativeFunctionMatrix());
        Assert.True(aResol.IsDone);
        aCurve.D1(1, aParams[7], out Point3D aP, out Vector3D aV);
        Assert.True(aP.DistanceTo(aLine.P3d[7][0]) < 1e-8);
        Assert.True(aV.Crossed(aTangent).Magnitude < 1e-8 * aV.Magnitude);
    }

    [Fact]
    public void Function_GradientMatchesDifferences()
    {
        AppParCurves_TestLine aLine = Helix(12, out double[] aParams);
        var aConstraints = Constraints((1, PassPoint), (12, PassPoint));
        Function aF = new(aLine, 1, 12, aConstraints, Vector(aParams), 4);
        Assert.Equal(12, aF.VariableCount);
        Assert.Equal(PassPoint, aF.FirstConstraint(aConstraints, 1));

        math_Vector X = Vector(aParams);
        math_Vector G = new(1, 12);
        Assert.True(aF.Values(X, out double F, G));
        Assert.True(aF.Value(X, out double F2));
        Assert.Equal(F, F2, 1e-15);
        Assert.True(aF.MaxError3d() > 1e-4);
        Assert.Equal(5, aF.CurveValue().NbPoles());

        const double h = 1e-6;
        for (int i = 2; i <= 11; i++)
        {
            math_Vector Xp = new(X), Xm = new(X);
            Xp[i] += h;
            Xm[i] -= h;
            aF.Value(Xp, out double Fp);
            aF.Value(Xm, out double Fm);
            Assert.Equal((Fp - Fm) / (2 * h), G[i], 1e-6);
        }
        aF.Value(X, out _);
        Assert.Equal(X[3], aF.NewParameters()[3]);
    }

    [Fact]
    public void BSpFunction_GradientMatchesDifferences()
    {
        AppParCurves_TestLine aLine = Helix(14, out double[] aParams);
        var aConstraints = Constraints((1, PassPoint), (14, PassPoint));
        BSpFunction aF = new(aLine, 1, 14, aConstraints, Vector(aParams), Knots(), Mults(), 5);
        math_Vector X = Vector(aParams);
        math_Vector G = new(1, 14);
        Assert.True(aF.Values(X, out double F, G));
        Assert.True(F > 0);
        Assert.Equal(3, aF.CurveValue().Degree());
        Assert.Equal(14, aF.Index().Length());
        const double h = 1e-6;
        for (int i = 2; i <= 13; i++)
        {
            math_Vector Xp = new(X), Xm = new(X);
            Xp[i] += h;
            Xm[i] -= h;
            aF.Value(Xp, out double Fp);
            aF.Value(Xm, out double Fm);
            Assert.Equal((Fp - Fm) / (2 * h), G[i], 1e-6);
        }
        aF.Value(X, out _);
        Assert.Equal(aF.MaxError3d(), MaxOver(aF, 14, 1), 1e-15);
    }

    private static double MaxOver(BSpFunction theF, int theNb, int theCurve)
    {
        double aMax = 0;
        for (int i = 1; i <= theNb; i++)
        {
            aMax = Math.Max(aMax, theF.Error(i, theCurve));
        }
        return aMax;
    }

    [Fact]
    public void Gradient_KeepsInteriorConstraintAndImproves()
    {
        AppParCurves_TestLine aLine = Helix(15, out double[] aParams, 0);
        math_Vector U = Vector(aParams);
        var aConstraints = Constraints((1, PassPoint), (8, PassPoint), (15, PassPoint));
        Gradient aGrad = new(aLine, 1, 15, aConstraints, U, 5, 1e-3, 1e-3, 50);
        global::Kerf.Engine.Approximation.MultiCurve aCurve = aGrad.Value();
        Assert.Equal(6, aCurve.NbPoles());
        Assert.True(aGrad.MaxError3d() < 1e-2, $"MaxError3d {aGrad.MaxError3d()}");
        Assert.True(aGrad.AverageError() <= aGrad.MaxError3d());
        Assert.Equal(aGrad.IsDone, aGrad.MaxError3d() <= 1e-3);

        // the interior passing point holds at its (possibly moved) parameter
        aCurve.Value(1, U[8], out Point3D aP);
        Assert.True(aP.DistanceTo(aLine.P3d[7][0]) < 1e-6);
        Assert.Equal(0.0, U[1]);
        Assert.Equal(1.0, U[15]);
    }

    [Fact]
    public void Function_InteriorConstraintsWithMixedDimensions_ReadOutOfBounds()
    {
        // OCCT reads the 2d points of a mixed line at the curve index (occt-issues.md); the port's range check throws
        AppParCurves_TestLine aLine = Helix(10, out double[] aParams);
        var aConstraints = Constraints((1, PassPoint), (5, PassPoint), (10, PassPoint));
        Assert.ThrowsAny<OutOfRangeException>(() => new Function(aLine, 1, 10, aConstraints, Vector(aParams), 4));
    }

    [Fact]
    public void BSpGradient_ImprovesTheParameters()
    {
        AppParCurves_TestLine aLine = Helix(20, out double[] aParams);
        // chord-length-like parameters, off the ones the points were made with
        for (int i = 0; i < 20; i++)
        {
            double t = aParams[i];
            aParams[i] = t + 0.1 * t * (1 - t);
        }
        var aConstraints = Constraints((1, PassPoint), (20, PassPoint));
        LeastSquare aStart = new(aLine, Knots(), Mults(), 1, 20, PassPoint, PassPoint,
                                 Vector(aParams), 5);
        aStart.Error(out _, out double aStart3d, out double aStart2d);

        math_Vector U = Vector(aParams);
        BSpGradient aGrad = new(aLine, 1, 20, aConstraints, U, Knots(), Mults(), 3, 1e-6, 1e-6, 30);
        Assert.True(aGrad.MaxError3d() < aStart3d, $"{aGrad.MaxError3d()} >= {aStart3d}");
        Assert.True(aGrad.MaxError2d() < aStart2d, $"{aGrad.MaxError2d()} >= {aStart2d}");
        global::Kerf.Engine.Approximation.MultiBSpCurve aCurve = aGrad.Value();
        Assert.Equal(3, aCurve.Degree());
        Assert.Equal(5, aCurve.NbPoles());
        Assert.True(aGrad.Error(10) <= aGrad.MaxError3d() + aGrad.MaxError2d());
        // the parameters were moved, ends kept
        Assert.NotEqual(aParams[9], U[10]);
        Assert.Equal(aParams[0], U[1]);

        // the curve passes through the end points
        aCurve.Value(1, U[20], out Point3D aEnd);
        Assert.True(aEnd.DistanceTo(aLine.P3d[19][0]) < 1e-12);
    }

    [Fact]
    public void BSpGradient_TangencyEndsWithGivenLambdas()
    {
        global::Kerf.Engine.Approximation.MultiBSpCurve aRef = CubicBSpline();
        double[] aParams = Params(15);
        AppParCurves_TestLine aLine = AppParCurves_TestLine.Sample(aRef, 1, 1, aParams);
        aLine.SetTangents(aRef, 1, 0.0);
        aLine.SetTangents(aRef, 15, 1.0);
        var aConstraints = Constraints((1, TangencyPoint), (15, TangencyPoint));

        // the lambdas of the fit: P2 = P1 + l1 V1, with V1 = 3 (P2 - P1) / 0.4
        BSpGradient aGrad = new(aLine, 1, 15, aConstraints, Vector(aParams), Knots(), Mults(), 3, 1e-9, 1e-9, 0);
        Assert.True(aGrad.IsDone);
        AssertSamePoles(aRef, aGrad.Value(), 1e-9);

        BSpGradient aGiven = new(aLine, 1, 15, aConstraints, Vector(aParams), Knots(), Mults(), 3, 1e-9, 1e-9, 0,
                                 0.4 / 3, 0.6 / 3);
        Assert.True(aGiven.IsDone);
        AssertSamePoles(aRef, aGiven.Value(), 1e-9);
    }
}
