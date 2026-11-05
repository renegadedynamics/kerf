// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using static Kerf.Engine.Numerics.Integration.MathInteg;
using static Kerf.Engine.Numerics.Linear.MathLin;
using static Kerf.Engine.Numerics.Optimisation.MathOpt;
using static Kerf.Engine.Numerics.Roots.MathRoot;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// OCCT's newer numerics against problems with known answers (unit 11I): the solvers and helpers that reach no test through
/// the ports of OCCT's GTests. Each checks the answer, not only that a solver ran.
/// </summary>
public class MathNumericsKnownAnswersTests
{
    private static math_Vector Vec(params double[] theValues)
    {
        math_Vector aVec = new(1, theValues.Length);
        for (int i = 0; i < theValues.Length; ++i)
        {
            aVec[i + 1] = theValues[i];
        }
        return aVec;
    }

    private static Matrix Mat(double[,] theValues)
    {
        Matrix aMatrix = new(1, theValues.GetLength(0), 1, theValues.GetLength(1));
        for (int i = 0; i < theValues.GetLength(0); ++i)
        {
            for (int j = 0; j < theValues.GetLength(1); ++j)
            {
                aMatrix[i + 1, j + 1] = theValues[i, j];
            }
        }
        return aMatrix;
    }

    /// <summary>f = (x - 1)^2 + 2 (y + 3)^2: its minimum, 0, at (1, -3).</summary>
    private sealed class Bowl : IVectorFunctionWithHessian
    {
        public bool Value(math_Vector theX, out double theY)
        {
            theY = (theX[1] - 1) * (theX[1] - 1) + 2 * (theX[2] + 3) * (theX[2] + 3);
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theG)
        {
            theG[1] = 2 * (theX[1] - 1);
            theG[2] = 4 * (theX[2] + 3);
            return true;
        }

        public bool Hessian(math_Vector theX, Matrix theH)
        {
            theH[1, 1] = 2;
            theH[1, 2] = 0;
            theH[2, 1] = 0;
            theH[2, 2] = 4;
            return true;
        }
    }

    private static void Finds(VectorResult theResult, double theX, double theY, double theTol)
    {
        Assert.True(theResult.IsDone, theResult.Status.ToString());
        Assert.Equal(theX, theResult.Solution![1], theTol);
        Assert.Equal(theY, theResult.Solution[2], theTol);
    }

    // Minimisers

    [Fact]
    public void Each_minimiser_finds_the_bottom_of_a_bowl()
    {
        math_Vector aStart = Vec(5, 5);
        Finds(Newton(new Bowl(), aStart), 1, -3, 1e-6);
        Finds(NewtonModified(new Bowl(), aStart), 1, -3, 1e-6);
        Finds(NewtonNumericalHessian(new Bowl(), aStart), 1, -3, 1e-4);
        Finds(NewtonNumerical(new Bowl(), aStart), 1, -3, 1e-3);
        Finds(BFGS(new Bowl(), aStart), 1, -3, 1e-5);
        Finds(BFGSNumerical(new Bowl(), aStart), 1, -3, 1e-3);
        Finds(LBFGS(new Bowl(), aStart), 1, -3, 1e-5);
        Finds(LBFGS(new Bowl(), aStart, 2), 1, -3, 1e-5);
        Finds(Powell(new Bowl(), aStart), 1, -3, 1e-4);
        Finds(FRPR(new Bowl(), aStart), 1, -3, 1e-4);
        Finds(FRPRNumerical(new Bowl(), aStart), 1, -3, 1e-3);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    public void Each_conjugate_gradient_formula_finds_the_bottom(int theFormula)
    {
        FRPRConfig aConfig = new() { Formula = (ConjugateGradientFormula)theFormula, MaxIterations = 5000 };
        Finds(FRPR(new Bowl(), Vec(5, 5), aConfig), 1, -3, 1e-4);
    }

    [Fact]
    public void Powell_from_given_directions_finds_it_too()
    {
        // not the axes: along (1, 1) and (1, -1)
        Finds(PowellWithDirections(new Bowl(), Vec(5, 5), Mat(new double[,] { { 1, 1 }, { 1, -1 } })), 1, -3, 1e-4);
    }

    [Fact]
    public void A_bounded_minimiser_stops_at_the_bound_when_the_bottom_is_beyond_it()
    {
        // x in [2, 4]: the best x is the bound, 2; y is free to reach -3
        math_Vector aLower = Vec(2, -10), anUpper = Vec(4, 10);
        Finds(NewtonBounded(new Bowl(), Vec(3, 5), aLower, anUpper), 2, -3, 1e-5);
        Finds(BFGSBounded(new Bowl(), Vec(3, 5), aLower, anUpper), 2, -3, 1e-5);
        // the same with the bottom inside
        Finds(NewtonBounded(new Bowl(), Vec(3, 5), Vec(-10, -10), Vec(10, 10)), 1, -3, 1e-5);
        Finds(BFGSBounded(new Bowl(), Vec(3, 5), Vec(-10, -10), Vec(10, 10)), 1, -3, 1e-5);
    }

    // Uzawa: min ||X - X0||^2 subject to C X = S and C X <= S

    [Fact]
    public void Uzawa_projects_onto_equalities_exactly()
    {
        // x + y = 2, from the origin: the nearest point is (1, 1)
        UzawaResult aResult = UzawaEquality(Mat(new double[,] { { 1, 1 } }), Vec(2), Vec(0, 0));
        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Solution![1], 1e-9);
        Assert.Equal(1.0, aResult.Solution[2], 1e-9);
    }

    [Fact]
    public void Uzawa_with_inequalities_leaves_a_satisfied_start_and_projects_a_violated_one()
    {
        // x + y <= 2
        UzawaResult anInside = Uzawa(Mat(new double[,] { { 1, 1 } }), Vec(2), Vec(0, 0), 0, 1);
        Assert.True(anInside.IsDone);
        Assert.Equal(0.0, anInside.Solution![1], 1e-6);
        Assert.Equal(0.0, anInside.Solution[2], 1e-6);

        UzawaResult anOutside = Uzawa(Mat(new double[,] { { 1, 1 } }), Vec(2), Vec(3, 3), 0, 1);
        Assert.True(anOutside.IsDone);
        Assert.Equal(1.0, anOutside.Solution![1], 1e-4);
        Assert.Equal(1.0, anOutside.Solution[2], 1e-4);

        // an equality and an inequality: x + y = 2 and x <= 0.5 from (3, 3): (0.5, 1.5)
        UzawaResult aBoth = Uzawa(Mat(new double[,] { { 1, 1 }, { 1, 0 } }), Vec(2, 0.5), Vec(3, 3), 1, 1, new UzawaConfig { EpsLix = 1e-8, MaxIterations = 100000 });
        Assert.True(aBoth.IsDone);
        Assert.Equal(0.5, aBoth.Solution![1], 1e-4);
        Assert.Equal(1.5, aBoth.Solution[2], 1e-4);

        UzawaConfig aFewSteps = new(new UzawaConfig { MaxIterations = 1, EpsLix = 1e-14, EpsLic = 1e-14 });
        Assert.False(Uzawa(Mat(new double[,] { { 1, 1 }, { 1, -1 } }), Vec(2, 0.5), Vec(30, 3), 0, 2, aFewSteps).IsDone);
    }

    // Crout: A = L D L^T

    [Fact]
    public void Crout_factors_solves_and_inverts_a_symmetric_matrix()
    {
        // determinant 11; the solution of A x = (6, 7) is (1, 2) by hand
        Matrix aMatrix = Mat(new double[,] { { 4, 1 }, { 1, 3 } });
        CroutResult aFactors = Crout(aMatrix);
        Assert.True(aFactors.IsDone);
        Assert.Equal(11.0, aFactors.Determinant!.Value, 1e-12);
        Assert.Equal(4.0, aFactors.D![1], 1e-12);
        // L is what the result documents, with A = L D L^T (decision 1678)
        Assert.Equal(0.25, aFactors.L![2, 1], 1e-12);
        Assert.Equal(1.0, aFactors.L[1, 1], 1e-12);
        Assert.Equal(1.0, aFactors.L[2, 2], 1e-12);
        Assert.Equal(0.25 * 0.25 * 4.0 + aFactors.D[2], aMatrix[2, 2], 1e-12);
        Assert.Equal(3.0 / 11.0, aFactors.Inverse![1, 1], 1e-12);

        LinearResult aSolved = SolveCrout(aMatrix, Vec(6, 7));
        Assert.True(aSolved.IsDone);
        Assert.Equal(1.0, aSolved.Solution![1], 1e-12);
        Assert.Equal(2.0, aSolved.Solution[2], 1e-12);

        InverseResult anInverse = InvertCrout(aMatrix);
        Assert.True(anInverse.IsDone);
        Assert.Equal(4.0 / 11.0, anInverse.Inverse![2, 2], 1e-12);
        Assert.Equal(-1.0 / 11.0, anInverse.Inverse[1, 2], 1e-12);
    }

    [Fact]
    public void Crout_refuses_singular_and_non_square_matrices()
    {
        Matrix aSingular = Mat(new double[,] { { 1, 1 }, { 1, 1 } });
        Assert.False(Crout(aSingular).IsDone);
        Assert.False(SolveCrout(aSingular, Vec(1, 1)).IsDone);
        Assert.False(InvertCrout(aSingular).IsDone);
        Matrix aWide = Mat(new double[,] { { 1, 0, 0 }, { 0, 1, 0 } });
        Assert.False(Crout(aWide).IsDone);
        Assert.False(SolveCrout(aWide, Vec(1, 1)).IsDone);
        Assert.False(InvertCrout(aWide).IsDone);
        Assert.False(SolveCrout(Mat(new double[,] { { 4, 1 }, { 1, 3 } }), Vec(1, 1, 1)).IsDone);
    }

    // Integration

    private sealed class Product : IVectorFunction
    {
        public bool Value(math_Vector theX, out double theY)
        {
            theY = theX[theX.Lower()] * theX[theX.Lower()] * theX[theX.Lower() + 1];
            return true;
        }
    }

    [Fact]
    public void Multiple_integrals_of_a_polynomial_are_exact()
    {
        // integral over [0, 1] x [0, 2] of x^2 y: (1/3) (2)
        math_IntegerVector anOrder = new(1, 2);
        anOrder[1] = 4;
        anOrder[2] = 4;
        IntegResult aResult = GaussMultiple(new Product(), 2, Vec(0, 0), Vec(1, 2), anOrder);
        Assert.True(aResult.IsDone);
        Assert.Equal(2.0 / 3.0, aResult.Value!.Value, 1e-12);
        IntegResult aUniform = GaussMultipleUniform(new Product(), 2, Vec(0, 0), Vec(1, 2), 4);
        Assert.Equal(2.0 / 3.0, aUniform.Value!.Value, 1e-12);
    }

    private sealed class TwoComponents : ISetFunction
    {
        public int NbEquations() => 2;

        public bool Value(math_Vector theX, math_Vector theF)
        {
            double aX = theX[theX.Lower()];
            theF[theF.Lower()] = aX;
            theF[theF.Lower() + 1] = aX * aX;
            return true;
        }
    }

    [Fact]
    public void A_vector_integral_is_exact_componentwise()
    {
        SetResult aResult = GaussSet(new TwoComponents(), 0.0, 3.0, 4);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.NbEquations);
        Assert.Equal(4.5, aResult.Values![aResult.Values.Lower()], 1e-12);
        Assert.Equal(9.0, aResult.Values[aResult.Values.Lower() + 1], 1e-12);
    }

    // All the roots, and where a function's nil

    private sealed class Scalar(Func<double, double> theF) : IScalarFunction, IScalarFunctionWithDerivative
    {
        public bool Value(double theX, out double theY)
        {
            theY = theF(theX);
            return true;
        }

        public bool Values(double theX, out double theY, out double theDy)
        {
            theY = theF(theX);
            theDy = (theF(theX + 1e-6) - theF(theX - 1e-6)) / 2e-6;
            return true;
        }
    }

    [Fact]
    public void All_roots_of_a_cubic_are_found()
    {
        AllRootsResult aResult = FindAllRootsWithIntervals(new Scalar(x => (x - 1) * (x - 2) * (x + 3)), -5, 5, 41);
        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.RootCount);
        Assert.Equal(0, aResult.IntervalCount);
        double[] aRoots = [.. aResult.Roots.OrderBy(r => r)];
        Assert.Equal(-3.0, aRoots[0], 1e-7);
        Assert.Equal(1.0, aRoots[1], 1e-7);
        Assert.Equal(2.0, aRoots[2], 1e-7);
    }

    [Fact]
    public void A_null_interval_is_reported_as_one_with_its_ends()
    {
        // zero on [1, 2], positive outside
        AllRootsResult aResult = FindAllRootsWithIntervals(new Scalar(x => x < 1 ? 1 - x : x > 2 ? x - 2 : 0), -1, 4, 51);
        Assert.True(aResult.IsDone);
        Assert.Equal(1, aResult.IntervalCount);
        Assert.Equal(1.0, aResult.NullIntervals[0].A, 1e-1);
        Assert.Equal(2.0, aResult.NullIntervals[0].B, 1e-1);
    }

    [Fact]
    public void Roots_from_the_multiple_root_form_match()
    {
        MultipleResult aResult = FindMultipleRoots(new Scalar(Math.Sin), 0.5, 7, 40, 1e-9, 1e-9);
        Assert.True(aResult.IsDone);
        Assert.Equal(2, aResult.Roots.Count);
        Assert.Equal(Math.PI, aResult.Roots[0], 1e-7);
        Assert.Equal(2 * Math.PI, aResult.Roots[1], 1e-7);
    }

    // The helpers

    [Fact]
    public void Line_searches_descend_along_the_direction()
    {
        // along -gradient from (2, 0) on x^2 + y^2 - the exact minimum's at alpha = 1/2
        var aFunction = new Round();
        math_Vector aX = Vec(2, 0), aDir = Vec(-4, 0), aGrad = Vec(4, 0);
        LineSearchResult anArmijo = ArmijoBacktrack(aFunction, aX, aDir, aGrad, 4);
        Assert.True(anArmijo.IsValid);
        Assert.True(anArmijo.FNew < 4);
        Assert.Equal(0.5, anArmijo.Alpha, 1e-12);

        LineSearchResult aWolfe = WolfeSearch(aFunction, aX, aDir, aGrad, 4);
        Assert.True(aWolfe.IsValid);
        Assert.True(aWolfe.FNew < 4);

        // an ascent direction has nothing to give
        Assert.False(WolfeSearch(aFunction, aX, Vec(4, 0), aGrad, 4).IsValid);

        LineSearchResult anExact = ExactLineSearch(aFunction, aX, aDir, 2);
        Assert.True(anExact.IsValid);
        Assert.Equal(0.5, anExact.Alpha, 1e-5);
        Assert.Equal(0.0, anExact.FNew, 1e-8);

        // through (0, 0): f(0) = 4, f'(0) = -16, f(1) = 4: the parabola's minimum at 1/2
        Assert.Equal(0.5, QuadraticInterpolation(4, -16, 1, 4), 1e-12);
    }

    private sealed class Round : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theY)
        {
            theY = theX[1] * theX[1] + theX[2] * theX[2];
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theG)
        {
            theG[1] = 2 * theX[1];
            theG[2] = 2 * theX[2];
            return true;
        }
    }

    [Fact]
    public void Brent_along_a_coordinate_finds_that_coordinates_minimum()
    {
        math_Vector aPoint = Vec(3, 1);
        double aF = 36; // f(3, 1)
        int aCount = 0;
        Assert.True(BrentAlongCoordinate(new Bowl(), aPoint, 1, -5, 5, ref aF, 1e-8, 100, ref aCount));
        Assert.Equal(1.0, aPoint[1], 1e-6);
        Assert.Equal(1.0, aPoint[2]);
        Assert.Equal(2.0 * 16.0, aF, 1e-6);
        Assert.True(aCount > 0);
    }

    [Fact]
    public void Numerical_derivatives_match_the_analytic_ones()
    {
        double aDeriv = 0;
        Assert.True(CentralDifference(new Scalar(Math.Sin), 1.0, ref aDeriv));
        Assert.Equal(Math.Cos(1.0), aDeriv, 1e-6);
        Assert.True(ForwardDifference(new Scalar(Math.Sin), 1.0, Math.Sin(1.0), ref aDeriv));
        Assert.Equal(Math.Cos(1.0), aDeriv, 1e-5);
        double aSecond = 0;
        Assert.True(SecondDerivative(new Scalar(Math.Sin), 1.0, Math.Sin(1.0), ref aSecond));
        Assert.Equal(-Math.Sin(1.0), aSecond, 1e-4);

        math_Vector aGrad = new(1, 2);
        Assert.True(NumericalGradient(new Bowl(), Vec(2, 0), aGrad));
        Assert.Equal(2.0, aGrad[1], 1e-5);
        Assert.Equal(12.0, aGrad[2], 1e-5);
        Assert.True(NumericalGradientAdaptive(new Bowl(), Vec(2, 0), aGrad));
        Assert.Equal(2.0, aGrad[1], 1e-5);
        Assert.Equal(12.0, aGrad[2], 1e-5);

        Matrix aHessian = new(1, 2, 1, 2);
        Assert.True(NumericalHessian(new Bowl(), Vec(2, 0), aHessian));
        Assert.Equal(2.0, aHessian[1, 1], 1e-3);
        Assert.Equal(4.0, aHessian[2, 2], 1e-3);
        Assert.Equal(0.0, aHessian[1, 2], 1e-3);

        Matrix aJacobian = new(1, 2, 1, 2);
        Assert.True(NumericalJacobian(new Swap(), Vec(2, 3), aJacobian));
        Assert.Equal(0.0, aJacobian[1, 1], 1e-5);
        Assert.Equal(1.0, aJacobian[1, 2], 1e-5);
        Assert.Equal(2.0 * 2.0, aJacobian[2, 1], 1e-5);
        Assert.Equal(0.0, aJacobian[2, 2], 1e-5);
    }

    private sealed class Swap : ISystemFunction
    {
        public bool Value(math_Vector theX, math_Vector theF)
        {
            theF[1] = theX[2];
            theF[2] = theX[1] * theX[1];
            return true;
        }
    }

    [Fact]
    public void The_polynomial_helpers()
    {
        // 2 + 3x + x^2 at 2, lowest first; x^2 + 3x + 2 highest first
        double[] aRising = [2, 3, 1], aFalling = [1, 3, 2];
        Assert.Equal(12.0, EvalPoly(aRising, 2, 2));
        Assert.Equal(12.0, EvalPolyDesc(aFalling, 2, 2));
        EvalPolyDeriv(aRising, 2, 2, out double aValue, out double aDeriv);
        Assert.Equal((12.0, 7.0), (aValue, aDeriv));
        // its roots are -1 and -2
        Assert.Equal(-1.0, RefinePolyRoot(aRising, 2, -1.1), 1e-9);
        Assert.Equal(-2.0, RefinePolyRootDesc(aFalling, 2, -2.1), 1e-9);

        double[] aRoots = [3, 1, 2, 1 + 1e-12];
        SortRoots(aRoots, 4);
        Assert.Equal(1.0, aRoots[0]);
        Assert.Equal(3.0, aRoots[3]);
        Assert.Equal(3, RemoveDuplicateRoots(aRoots, 4));
        Assert.Equal([1.0, 2.0, 3.0], aRoots[..3]);

        // x^3 + 3x^2 + 3x + 1 = (x + 1)^3: with t = x + 1, t^3 + 0 t + 0
        DepressCubic(3, 3, 1, out double aP, out double aQ, out double aShift);
        Assert.Equal((0.0, 0.0, 1.0), (aP, aQ, aShift));
        // (x + 1)^4: p = q = r = 0
        DepressQuartic(4, 6, 4, 1, out double aP4, out double aQ4, out double aR4, out double aShift4);
        Assert.Equal(0.0, aP4, 1e-12);
        Assert.Equal(0.0, aQ4, 1e-12);
        Assert.Equal(0.0, aR4, 1e-12);
        Assert.Equal(1.0, aShift4);
    }

    [Fact]
    public void The_core_helpers()
    {
        Assert.Equal(1.0, Clamp(5, 0, 1));
        Assert.Equal(0.0, Clamp(-5, 0, 1));
        Assert.True(IsZero(1e-20));
        Assert.False(IsZero(1));
        Assert.True(IsEqual(1, 1 + 1e-20));
        Assert.False(IsEqual(1, 2));
        Assert.Equal(2.0, SafeDiv(4, 2));
        Assert.Equal(7.0, SafeDiv(4, 0, 7));
        Assert.Equal((-1, 0, 1), (Sign(-3), Sign(0), Sign(3)));
        Assert.Equal(-3.0, SignTransfer(3, -1));
        Assert.Equal(9.0, Sqr(3));
        Assert.Equal(27.0, Cube(3));
        Assert.Equal(-3.0, CubeRoot(-27), 1e-12);
        Assert.True(MathFinite(1));
        Assert.False(MathFinite(double.NaN));
        Assert.Equal(32.0, DotProduct(Vec(1, 2, 3), Vec(4, 5, 6)));
        Assert.Equal(5.0, VectorNorm(Vec(3, 4)));
        Assert.Equal(4.0, VectorInfNorm(Vec(3, -4)));
        Assert.True(ComputeScaleFactor([1024.0, 1.0], 2) > 0);

        static bool MathFinite(double theValue) => global::Kerf.Engine.Numerics.Utilities.MathUtils.IsFinite(theValue);
    }

    [Fact]
    public void The_convergence_tests()
    {
        Assert.True(IsXConverged(1, 1 + 1e-12, 1e-9));
        Assert.False(IsXConverged(1, 2, 1e-9));
        // relative to the size of x
        Assert.True(IsXConverged(1e6, 1e6 + 1e-4, 1e-9));
        Assert.True(IsFConverged(1e-12, 1e-9));
        Assert.False(IsFConverged(1, 1e-9));
        Config aConfig = new() { XTolerance = 1e-9, FTolerance = 1e-9 };
        Assert.True(IsConverged(1, 2, 1e-12, aConfig));
        Assert.True(IsConverged(1, 1, 1, aConfig));
        Assert.False(IsConverged(1, 2, 1, aConfig));
        Assert.True(IsMinConverged(1, 1, 3, 3, aConfig));
        Assert.False(IsMinConverged(1, 2, 3, 4, aConfig));
        Assert.True(IsVectorConverged(Vec(1, 2), Vec(1, 2 + 1e-12), 1e-9));
        Assert.False(IsVectorConverged(Vec(1, 2), Vec(1, 3), 1e-9));
        Assert.True(IsGradientConverged(Vec(1e-12, 0), 1e-9));
        Assert.False(IsGradientConverged(Vec(1, 0), 1e-9));
        Assert.Equal(4.0, InfinityNorm(Vec(3, -4)));
        Assert.Equal(5.0, EuclideanNorm(Vec(3, 4)));
    }

    [Fact]
    public void Gauss_points_integrate_a_polynomial_of_their_degree_within_and_past_the_tables()
    {
        // n points are exact to degree 2n - 1: x^(2n-2) over [-1, 1] is 2 / (2n - 1); both within the tables and past them
        foreach (int aN in new[] { 1, 5, 61, 70 })
        {
            math_Vector aPoints = new(1, aN), aWeights = new(1, aN);
            Assert.True(GetGaussPointsAndWeights(aN, aPoints, aWeights));
            double aSum = 0, aWeightSum = 0;
            for (int i = 1; i <= aN; i++)
            {
                aSum += aWeights[i] * Math.Pow(aPoints[i], 2 * aN - 2);
                aWeightSum += aWeights[i];
                Assert.True(i == 1 || aPoints[i] > aPoints[i - 1]);
            }
            Assert.Equal(2.0 / (2 * aN - 1), aSum, 1e-10);
            Assert.Equal(2.0, aWeightSum, 1e-10);
        }
        Assert.False(GetGaussPointsAndWeights(0, new math_Vector(1, 1), new math_Vector(1, 1)));
        Assert.False(GetGaussPointsAndWeights(2, new math_Vector(1, 1), new math_Vector(1, 2)));
    }

    [Fact]
    public void The_domains()
    {
        Domain1D aD = new(1, 3);
        Assert.Equal((2.0, 2.0), (aD.Length(), aD.Mid()));
        Assert.True(aD.Contains(3.5, 1));
        Assert.False(aD.Contains(3.5));
        Assert.Equal((1.0, 3.0, 2.0), (aD.Clamp(0), aD.Clamp(5), aD.Clamp(2)));
        Assert.False(aD.IsLarge());
        Assert.True(new Domain1D(0, 2000).IsLarge());
        Assert.True(aD.IsFullPeriod(2));
        Assert.False(aD.IsFullPeriod(3));
        Assert.Equal(2.0, aD.Lerp(0.5));
        Assert.Equal(0.5, aD.Normalize(2));
        Assert.Equal(0.5, new Domain1D(1, 1).Normalize(1));
        Assert.True(aD.IsFinite());
        Assert.False(new Domain1D(-2e100, 0).IsFinite());
        Assert.True(aD.IsEqual(new Domain1D(1, 3 + 1e-12)));
        Assert.False(aD.IsEqual(new Domain1D(1, 4)));
        Assert.True(aD == new Domain1D(1, 3));
        Assert.True(aD != new Domain1D(1, 4));
        Assert.True(aD.Equals((object)new Domain1D(1, 3)));
        Assert.False(aD.Equals("a domain"));
        Assert.Equal(aD.GetHashCode(), new Domain1D(1, 3).GetHashCode());
        Assert.NotEmpty(aD.ToString());

        Domain2D aD2 = new(aD, new Domain1D(0, 4));
        Assert.Equal(aD, aD2.U());
        Assert.Equal(new Domain1D(0, 4), aD2.V());
        Assert.Equal((2.0, 4.0, 2.0, 2.0), (aD2.ULength(), aD2.VLength(), aD2.UMid(), aD2.VMid()));
        Assert.True(aD2.Contains(2, 2));
        Assert.False(aD2.Contains(0, 2));
        Assert.False(aD2.Contains(2, 5));
        double aU = 0, aV = 9;
        aD2.Clamp(ref aU, ref aV);
        Assert.Equal((1.0, 4.0), (aU, aV));
        Assert.False(aD2.IsLarge());
        // large only when both directions are
        Assert.False(new Domain2D(0, 2000, 0, 1).IsLarge());
        Assert.False(new Domain2D(0, 1, 0, 2000).IsLarge());
        Assert.True(new Domain2D(0, 2000, 0, 2000).IsLarge());
        Assert.True(aD2.IsUFullPeriod(2));
        Assert.True(aD2.IsVFullPeriod(4));
        Assert.False(aD2.IsVFullPeriod(5));
        Assert.True(aD2.IsFinite());
        Assert.False(new Domain2D(0, 1, 0, 2e100).IsFinite());
        Assert.True(aD2 == new Domain2D(1, 3, 0, 4));
        Assert.True(aD2 != new Domain2D(1, 3, 0, 5));
        Assert.True(aD2.Equals((object)new Domain2D(1, 3, 0, 4)));
        Assert.False(aD2.Equals("a domain"));
        Assert.Equal(aD2.GetHashCode(), new Domain2D(1, 3, 0, 4).GetHashCode());
        Assert.NotEmpty(aD2.ToString());
    }

    [Fact]
    public void The_configurations_copy_their_settings()
    {
        BoundedConfig aBounded = new(1, 2, 1e-3, 7);
        Assert.Equal((1.0, 2.0), (aBounded.LowerBound, aBounded.UpperBound));
        BoundedConfig aCopy = new(aBounded);
        Assert.Equal((1.0, 2.0), (aCopy.LowerBound, aCopy.UpperBound));
        Assert.Equal(aBounded.MaxIterations, aCopy.MaxIterations);
        LinConfig aLin = new() { SingularityTolerance = 1e-3, UsePivoting = false };
        LinConfig aLinCopy = new(aLin);
        Assert.Equal((1e-3, false), (aLinCopy.SingularityTolerance, aLinCopy.UsePivoting));
    }
}
