// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Numerics;
using static Kerf.Engine.Numerics.Integration.MathInteg;
using static Kerf.Engine.Numerics.Polynomials.MathPoly;
using static Kerf.Engine.Numerics.Roots.MathRoot;
using static Kerf.Engine.Numerics.Systems.MathSys;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// Polynomial solvers on degenerate leading terms and high degrees, the integrators' other forms, and the system solvers
/// with bounds, against answers worked by hand (unit 11I).
/// </summary>
public class MathNumericsFormsTests
{
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

    private static math_Vector Vec(params double[] theValues)
    {
        math_Vector aVec = new(1, theValues.Length);
        for (int i = 0; i < theValues.Length; ++i)
        {
            aVec[i + 1] = theValues[i];
        }
        return aVec;
    }

    private static void HasRoots(PolyResult theResult, params double[] theRoots)
    {
        Assert.True(theResult.IsDone, theResult.Status.ToString());
        Assert.Equal(theRoots.Length, theResult.NbRoots);
        double[] aFound = new double[theResult.NbRoots];
        for (int i = 0; i < aFound.Length; i++)
        {
            aFound[i] = theResult.Roots[i];
        }
        Array.Sort(aFound);
        for (int i = 0; i < aFound.Length; i++)
        {
            Assert.Equal(theRoots[i], aFound[i], 1e-7);
        }
    }

    // Polynomials

    [Fact]
    public void A_vanishing_leading_term_drops_a_degree()
    {
        // 0 x^4 + (x - 1)(x - 2)(x - 3)
        HasRoots(Quartic(0, 1, -6, 11, -6), 1, 2, 3);
        // 0 x^3 + (x - 1)(x + 1)
        HasRoots(Cubic(0, 1, 0, -1), -1, 1);
        HasRoots(Quadratic(0, 2, -4), 2);
    }

    [Fact]
    public void Quartics_by_their_special_forms()
    {
        // biquadratic: x^4 - 5x^2 + 4
        HasRoots(Quartic(1, 0, -5, 0, 4), -2, -1, 1, 2);
        // a root at zero: x (x - 1)(x - 2)(x - 3)
        HasRoots(Quartic(1, -6, 11, -6, 0), 0, 1, 2, 3);
        // none: x^4 + 1
        HasRoots(Quartic(1, 0, 0, 0, 1));
        // the general case: (x - 1)(x - 2)(x + 3)(x + 4)
        HasRoots(Quartic(1, 4, -7, -22, 24), -4, -3, 1, 2);
    }

    [Fact]
    public void Cubics_by_their_discriminants()
    {
        // one real root: x^3 + x + 2 = (x + 1)(x^2 - x + 2)
        HasRoots(Cubic(1, 0, 1, 2), -1);
        // a double root and a simple one: (x - 1)^2 (x + 2)
        PolyResult aDouble = Cubic(1, 0, -3, 2);
        Assert.True(aDouble.IsDone);
        Assert.Contains(-2.0, new[] { Math.Round(aDouble.Roots[0], 6), Math.Round(aDouble.Roots[1], 6), Math.Round(aDouble.Roots[2], 6) });
        // a triple root: (x - 2)^3
        PolyResult aTriple = Cubic(1, -6, 12, -8);
        Assert.True(aTriple.IsDone);
        Assert.Equal(2.0, aTriple.Roots[0], 1e-4);
    }

    [Fact]
    public void Higher_degrees_find_their_real_roots()
    {
        // (x - 1)(x - 2)(x - 3)(x - 4)(x - 5)
        PolyResult aQuintic = Quintic(1, -15, 85, -225, 274, -120);
        Assert.True(aQuintic.IsDone);
        Assert.Equal(4, aQuintic.NbRoots); // PolyResult holds four roots
        // (x^2 - 1)(x^2 - 4)(x^2 - 9): x^6 - 14 x^4 + 49 x^2 - 36
        PolyResult aSextic = Sextic(1, 0, -14, 0, 49, 0, -36);
        Assert.True(aSextic.IsDone);
        Assert.Equal(4, aSextic.NbRoots);
        for (int i = 0; i < 4; i++)
        {
            double aX = aSextic.Roots[i];
            Assert.Equal(0.0, aX * aX * aX * aX * aX * aX - 14 * aX * aX * aX * aX + 49 * aX * aX - 36, 1e-6);
        }

        // (x^2 - 1)(x^2 - 4)(x^2 - 9)(x^2 - 16), lowest coefficient first: all eight are real
        GeneralPolyResult anOctic = Octic([576, 0, -820, 0, 273, 0, -30, 0, 1]);
        Assert.True(anOctic.IsDone);
        Assert.Equal(8, anOctic.NbRoots);
        Assert.Equal(-4.0, anOctic.Roots[0], 1e-7);
        Assert.Equal(4.0, anOctic.Roots[7], 1e-7);
    }

    [Fact]
    public void Laguerre_finds_complex_pairs_too()
    {
        // x^2 + 1, and (x - 2)(x^2 + 1) = -2 + x - 2x^2 + x^3, lowest coefficient first
        GeneralPolyResult aPair = Laguerre([1.0, 0.0, 1.0], 2);
        Assert.True(aPair.IsDone);
        Assert.Equal(0, aPair.NbRoots);
        Assert.Equal(2, aPair.NbComplexRoots);
        Assert.Equal(1.0, Math.Abs(aPair.ComplexRoots[0].Imaginary), 1e-8);

        GeneralPolyResult aMixed = LaguerreN([-2.0, 1.0, -2.0, 1.0], 4);
        Assert.True(aMixed.IsDone);
        Assert.Equal(1, aMixed.NbRoots);
        Assert.Equal(2.0, aMixed.Roots[0], 1e-8);
    }

    // Integration: x^2 over [0, 3] is 9, exp(-x) over [0, inf) is 1, exp(-x^2) over the line is sqrt(pi)

    [Fact]
    public void Each_integrator_on_a_finite_range()
    {
        Scalar aSquare = new(x => x * x);
        Assert.Equal(9.0, KronrodRule(aSquare, 0, 3).Value!.Value, 1e-10);
        Assert.Equal(9.0, Kronrod(aSquare, 0, 3).Value!.Value, 1e-10);
        Assert.Equal(9.0, Kronrod(aSquare, 0, 3, new KronrodConfig(1e-8, 50)).Value!.Value, 1e-7);
        Assert.Equal(9.0, KronrodAuto(aSquare, 0, 3).Value!.Value, 1e-8);
        Assert.Equal(9.0, Gauss(aSquare, 0, 3).Value!.Value, 1e-10);
        Assert.Equal(9.0, GaussAdaptive(aSquare, 0, 3).Value!.Value, 1e-8);
        Assert.Equal(9.0, GaussComposite(aSquare, 0, 3, 4).Value!.Value, 1e-10);
        Assert.Equal(9.0, TanhSinh(aSquare, 0, 3).Value!.Value, 1e-8);
        Assert.Equal(9.0, DoubleExponential(aSquare, 0, 3).Value!.Value, 1e-8);
    }

    [Fact]
    public void Each_integrator_on_an_infinite_range()
    {
        Scalar aDecay = new(x => Math.Exp(-x));
        Scalar aBell = new(x => Math.Exp(-x * x));
        Assert.Equal(1.0, KronrodSemiInfinite(aDecay, 0).Value!.Value, 1e-5);
        Assert.Equal(Math.Sqrt(Math.PI), KronrodInfinite(aBell).Value!.Value, 1e-5);
        Assert.Equal(1.0, ExpSinh(aDecay, 0).Value!.Value, 1e-6);
        Assert.Equal(Math.Sqrt(Math.PI), SinhSinh(aBell).Value!.Value, 1e-6);
        Assert.Equal(1.0, DoubleExponential(aDecay, 0, double.MaxValue).Value!.Value, 1e-6);
        Assert.Equal(Math.Sqrt(Math.PI), DoubleExponential(aBell, -double.MaxValue, double.MaxValue).Value!.Value, 1e-6);
        // the other half-line: exp(x) up to 0
        Assert.Equal(1.0, DoubleExponential(new Scalar(Math.Exp), -double.MaxValue, 0).Value!.Value, 1e-6);
    }

    [Fact]
    public void A_singular_integrand_is_integrated_by_tanh_sinh()
    {
        // 1 / sqrt(x) over [0, 1] is 2, its singularity at the end; 1 / sqrt|x - 1/2| over [0, 1] is 2 sqrt(2)
        Assert.Equal(2.0, TanhSinhSingular(new Scalar(x => 1 / Math.Sqrt(x)), 0, 1).Value!.Value, 1e-5);
        IntegResult aWithin = TanhSinhWithSingularity(new Scalar(x => 1 / Math.Sqrt(Math.Abs(x - 0.5))), 0, 1, 0.5);
        Assert.True(aWithin.IsDone);
        Assert.Equal(2 * Math.Sqrt(2), aWithin.Value!.Value, 1e-5);
    }

    // Scalar roots

    [Fact]
    public void Each_scalar_root_form_finds_the_cube_root_of_two()
    {
        Scalar aF = new(x => x * x * x - 2);
        double aRoot = Math.Cbrt(2);
        Assert.Equal(aRoot, Bisection(aF, 0, 3).Root!.Value, 1e-7);
        Assert.Equal(aRoot, BisectionNewton(aF, 0, 3).Root!.Value, 1e-7);
        Assert.Equal(aRoot, Newton(aF, 1).Root!.Value, 1e-7);
        Assert.Equal(aRoot, NewtonBounded(aF, 1, 0, 3).Root!.Value, 1e-7);
        Assert.Equal(aRoot, Secant(aF, 1, 2).Root!.Value, 1e-7);
        Assert.Equal(aRoot, SecantAuto(aF, 1).Root!.Value, 1e-7);
    }

    [Fact]
    public void A_range_with_no_sign_change_is_refused()
    {
        Scalar aF = new(x => x * x + 1);
        Assert.False(Bisection(aF, 0, 3).IsDone);
        Assert.False(BisectionNewton(aF, 0, 3).IsDone);
        // a stationary guess: Newton has nowhere to go
        Assert.False(Newton(new Scalar(x => x * x + 1), 0).IsDone);
    }

    [Fact]
    public void Trigonometric_equations_by_their_special_forms()
    {
        // sin x = 1/2 on [0, 2 pi]: d sin + e = 0
        TrigResult aLinear = TrigonometricLinear(1, -0.5);
        Assert.True(aLinear.IsDone);
        Assert.Equal(2, aLinear.NbRoots);
        Assert.Equal(Math.PI / 6, aLinear.Roots[0], 1e-9);
        Assert.Equal(5 * Math.PI / 6, aLinear.Roots[1], 1e-9);

        // cos x + sin x = 0: 3 pi / 4 and 7 pi / 4
        TrigResult aCD = TrigonometricCDE(1, 1, 0);
        Assert.True(aCD.IsDone);
        Assert.Equal(2, aCD.NbRoots);
        Assert.Equal(3 * Math.PI / 4, aCD.Roots[0], 1e-9);

        // all coefficients nil: every angle
        Assert.True(Trigonometric(0, 0, 0, 0, 0).InfiniteRoots);
        // sin x = 2: none
        TrigResult aNone = TrigonometricLinear(1, -2);
        Assert.True(aNone.IsDone);
        Assert.Equal(0, aNone.NbRoots);
    }

    // Systems: x^2 + y^2 = 25, y = x + 1: (3, 4) and (-4, -3)

    private sealed class CircleAndLine : IFunctionSetWithDerivatives, INewton2DFunction
    {
        public int NbVariables() => 2;

        public int NbEquations() => 2;

        public bool Value(math_Vector theX, math_Vector theF)
        {
            theF[1] = theX[1] * theX[1] + theX[2] * theX[2] - 25;
            theF[2] = theX[2] - theX[1] - 1;
            return true;
        }

        public bool Derivatives(math_Vector theX, Matrix theD)
        {
            theD[1, 1] = 2 * theX[1];
            theD[1, 2] = 2 * theX[2];
            theD[2, 1] = -1;
            theD[2, 2] = 1;
            return true;
        }

        public bool Values(math_Vector theX, math_Vector theF, Matrix theD)
            => Value(theX, theF) && Derivatives(theX, theD);

        public bool Invoke(double theU, double theV, ref Array2<double> theF, ref Array2<Array2<double>> theJ)
        {
            theF[0] = theU * theU + theV * theV - 25;
            theF[1] = theV - theU - 1;
            theJ[0][0] = 2 * theU;
            theJ[0][1] = 2 * theV;
            theJ[1][0] = -1;
            theJ[1][1] = 1;
            return true;
        }
    }

    [Fact]
    public void A_bounded_newton_finds_the_root_in_its_bounds()
    {
        VectorResult aPositive = NewtonBounded(new CircleAndLine(), Vec(1, 1), Vec(0, 0), Vec(10, 10), Vec(1e-9, 1e-9), 1e-9);
        Assert.True(aPositive.IsDone);
        Assert.Equal(3.0, aPositive.Solution![1], 1e-7);
        Assert.Equal(4.0, aPositive.Solution[2], 1e-7);

        VectorResult aNegative = NewtonBounded(new CircleAndLine(), Vec(-1, -2), Vec(-10, -10), Vec(0, 0), Vec(1e-9, 1e-9), 1e-9);
        Assert.True(aNegative.IsDone);
        Assert.Equal(-4.0, aNegative.Solution![1], 1e-7);

        VectorResult aPlain = Newton(new CircleAndLine(), Vec(1, 1), 1e-9, 1e-9);
        Assert.True(aPlain.IsDone);
        Assert.Equal(3.0, aPlain.Solution![1], 1e-7);
    }

    [Fact]
    public void A_bounded_levenberg_marquardt_finds_the_root_in_its_bounds()
    {
        VectorResult aResult = LevenbergMarquardtBounded(new CircleAndLine(), Vec(1, 1), Vec(0, 0), Vec(10, 10));
        Assert.True(aResult.IsDone);
        Assert.Equal(3.0, aResult.Solution![1], 1e-5);
        Assert.Equal(4.0, aResult.Solution[2], 1e-5);

        VectorResult aFree = LevenbergMarquardt(new CircleAndLine(), Vec(-1, -2));
        Assert.True(aFree.IsDone);
        Assert.Equal(-4.0, aFree.Solution![1], 1e-5);
    }

    [Fact]
    public void A_2d_newton_within_bounds_and_with_its_jacobian_singular()
    {
        NewtonBoundsN2 aBounds = new();
        aBounds.Min[0] = 0;
        aBounds.Min[1] = 0;
        aBounds.Max[0] = 10;
        aBounds.Max[1] = 10;
        Array2<double> aStart = default;
        aStart[0] = 1;
        aStart[1] = 1;
        NewtonResultN2 aResult = Solve2D(new CircleAndLine(), aStart, aBounds);
        Assert.True(aResult.IsDone);
        Assert.Equal(3.0, aResult.X[0], 1e-6);
        Assert.Equal(4.0, aResult.X[1], 1e-6);

        // at (0, 0) the Jacobian's first row is nil, so it's singular: the solver has to get off it
        aStart[0] = 0;
        aStart[1] = 0;
        NewtonResultN2 aFromSingular = Solve2D(new CircleAndLine(), aStart, aBounds);
        Assert.True(aFromSingular.IsDone, aFromSingular.Status.ToString());
        Assert.Equal(3.0, aFromSingular.X[0], 1e-6);
        Assert.Equal(4.0, aFromSingular.X[1], 1e-6);
    }
}
