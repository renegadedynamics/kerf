// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Numerics;
using Kerf.Engine.Numerics.Roots;
using Kerf.Engine.Numerics.Utilities;
using static Kerf.Engine.Numerics.Roots.MathRoot;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// The root finders on functions that misbehave (decision 19805): one that fails to evaluate at some points is reported
/// and not hidden, every root returned must make the function vanish, and every sign change between samples holds a root.
/// </summary>
public class MathRoot_LawsTests
{
    // A function that can refuse to evaluate from some point on, or at its first call
    private readonly struct Fn(Func<double, double> theF, Func<double, double>? theDF = null, Func<double, bool>? theGood = null)
        : IScalarFunction, IScalarFunctionWithDerivative
    {
        public bool Value(double x, out double y)
        {
            y = theF(x);
            return theGood?.Invoke(x) ?? true;
        }

        public bool Values(double x, out double y, out double d)
        {
            y = theF(x);
            d = theDF?.Invoke(x) ?? 0;
            return theGood?.Invoke(x) ?? true;
        }
    }

    private static Fn Cubic => new(x => (x * x * x) - x, x => (3 * x * x) - 1);

    private static Fn Refusing(double theFrom) => new(x => x - 0.5, _ => 1, x => x < theFrom);

    [Theory]
    [InlineData(-5.0)] // refuses everywhere
    public void A_function_that_refuses_its_first_value_is_a_numerical_error(double theFrom)
    {
        Fn aFn = Refusing(theFrom);
        Assert.Equal(Status.NumericalError, MathRoot.Bisection(aFn, 0, 1).Status);
        Assert.Equal(Status.NumericalError, MathRoot.BisectionNewton(aFn, 0, 1).Status);
        Assert.Equal(Status.NumericalError, MathRoot.Brent(aFn, 0, 1).Status);
        Assert.Equal(Status.NumericalError, MathRoot.Newton(aFn, 0.1).Status);
        Assert.Equal(Status.NumericalError, MathRoot.Secant(aFn, 0, 1).Status);
    }

    [Fact]
    public void A_function_that_refuses_its_second_end_is_a_numerical_error()
    {
        Fn aFn = Refusing(0.75);
        Assert.Equal(Status.NumericalError, MathRoot.Bisection(aFn, 0, 1).Status);
        Assert.Equal(Status.NumericalError, MathRoot.BisectionNewton(aFn, 0, 1).Status);
        Assert.Equal(Status.NumericalError, MathRoot.Brent(aFn, 0, 1).Status);
        Assert.Equal(Status.NumericalError, MathRoot.Secant(aFn, 0, 1).Status);
    }

    [Fact]
    public void A_function_that_refuses_on_the_way_is_a_numerical_error()
    {
        // inside the bracket, around the root: the ends are fine
        Fn aFn = new(x => x - 0.5, _ => 1, x => x is <= 0.1 or >= 0.9);
        Assert.Equal(Status.NumericalError, MathRoot.Bisection(aFn, 0, 1).Status);
        Assert.Equal(Status.NumericalError, MathRoot.Brent(aFn, 0, 1).Status);
        Assert.Equal(Status.NumericalError, MathRoot.Secant(aFn, 0, 1).Status);
        Assert.Equal(Status.NumericalError, MathRoot.Newton(aFn, 0.05).Status);
    }

    [Fact]
    public void Running_out_of_iterations_is_reported_with_the_best_so_far()
    {
        var aConfig = new Config { MaxIterations = 2, XTolerance = 1e-14, FTolerance = 1e-14 };
        Fn aFn = new(x => Math.Cos(x) - x, x => -Math.Sin(x) - 1);
        ScalarResult aBisect = MathRoot.Bisection(aFn, 0, 1, aConfig);
        Assert.Equal(Status.MaxIterations, aBisect.Status);
        Assert.InRange(aBisect.Root!.Value, 0, 1);
        Assert.Equal(Status.MaxIterations, MathRoot.BisectionNewton(aFn, 0, 1, aConfig).Status);
        Assert.Equal(Status.MaxIterations, MathRoot.Brent(aFn, 0, 1, aConfig).Status);
        Assert.Equal(Status.MaxIterations, MathRoot.Secant(aFn, 0, 1, aConfig).Status);
        Assert.Equal(Status.MaxIterations, MathRoot.Newton(aFn, 5, aConfig).Status);
    }

    [Fact]
    public void A_flat_stretch_stops_the_secant_and_Newton_without_a_root()
    {
        Fn aFlat = new(_ => 1, _ => 0);
        Assert.Equal(Status.NumericalError, MathRoot.Secant(aFlat, 0, 1).Status);
        Assert.NotEqual(Status.OK, MathRoot.Newton(aFlat, 0).Status);
    }

    [Fact]
    public void An_end_of_the_bracket_that_is_the_root_is_found_at_once()
    {
        Fn aFn = new(x => x - 1, _ => 1);
        (string Name, ScalarResult Result)[] aResults =
        [
            ("Bisection up", MathRoot.Bisection(aFn, 1, 3)), ("Bisection down", MathRoot.Bisection(aFn, -1, 1)),
            ("Brent up", MathRoot.Brent(aFn, 1, 3)), ("Brent down", MathRoot.Brent(aFn, -1, 1)),
            ("BisectionNewton up", MathRoot.BisectionNewton(aFn, 1, 3)), ("BisectionNewton down", MathRoot.BisectionNewton(aFn, -1, 1)),
        ];
        foreach ((string aName, ScalarResult aResult) in aResults)
        {
            Assert.True(aResult.IsDone, aName);
            Assert.True(Math.Abs(aResult.Root!.Value - 1) < 1e-9, aName + " " + aResult.Root);
        }
    }

    // ---- all the roots ----

    private static void AssertAreRoots(Func<double, double> theF, IEnumerable<double> theRoots)
    {
        foreach (double aRoot in theRoots)
        {
            Assert.True(Math.Abs(theF(aRoot)) < 1e-6, $"f({aRoot}) = {theF(aRoot)}");
        }
    }

    [Fact]
    public void All_the_roots_of_a_cubic_with_or_without_its_derivative()
    {
        MultipleResult aPlain = MathRoot.FindAllRoots(Cubic, -2, 2);
        MultipleResult aDerived = MathRoot.FindAllRootsWithDerivative(Cubic, -2, 2);
        foreach (MultipleResult aResult in new[] { aPlain, aDerived })
        {
            Assert.True(aResult.IsDone);
            Assert.Equal(3, aResult.Roots.Count);
            Assert.Equal(-1, aResult.Roots[0], 6);
            Assert.Equal(0, aResult.Roots[1], 6);
            Assert.Equal(1, aResult.Roots[2], 6);
        }
    }

    [Fact]
    public void A_double_root_needs_the_derivative_to_be_seen()
    {
        // (x - 1)^2 touches without a change of sign
        Fn aFn = new(x => (x - 1) * (x - 1), x => 2 * (x - 1));
        MultipleResult aDerived = MathRoot.FindAllRootsWithDerivative(aFn, -3, 4, new MultipleConfig { NbSamples = 37 });
        Assert.True(aDerived.IsDone);
        Assert.Contains(aDerived.Roots, r => Math.Abs(r - 1) < 1e-4);
    }

    [Fact]
    public void All_the_roots_with_an_offset_and_reversed_bounds()
    {
        MultipleResult aResult = MathRoot.FindAllRootsWithDerivative(new Fn(Math.Sin, Math.Cos), 7, 0, new MultipleConfig { Offset = 0.5 });
        Assert.True(aResult.IsDone);
        // sin x = 1/2 at pi/6, 5 pi/6 and 13 pi/6
        Assert.Equal(3, aResult.Roots.Count);
        Assert.Equal(Math.PI / 6, aResult.Roots[0], 6);
        Assert.Equal(5 * Math.PI / 6, aResult.Roots[1], 6);
        Assert.Equal(13 * Math.PI / 6, aResult.Roots[2], 6);
    }

    [Fact]
    public void A_function_that_is_nothing_is_all_null()
    {
        Fn aZero = new(_ => 0, _ => 0);
        Assert.True(MathRoot.FindAllRoots(aZero, -1, 1).IsAllNull);
        Assert.True(MathRoot.FindAllRootsWithDerivative(aZero, -1, 1).IsAllNull);
    }

    [Fact]
    public void A_function_that_refuses_is_not_listed_with_roots()
    {
        Fn aRefusing = Refusing(-5);
        Assert.False(MathRoot.FindAllRootsWithDerivative(aRefusing, -1, 1).IsDone);
        Assert.False(MathRoot.FindAllRoots(aRefusing, -1, 1).IsDone);
        // halfway through the samples
        Fn aHalf = Refusing(0.25);
        Assert.False(MathRoot.FindAllRootsWithDerivative(aHalf, -1, 1).IsDone);
    }

    [Fact]
    public void Roots_at_or_between_the_samples_and_null_intervals()
    {
        // zero at each integer sample and across [2, 3]
        static double F(double x) => x switch
        {
            <= 1 => x - 1,
            < 2 => (x - 1) * (x - 2),
            <= 3 => 0,
            _ => x - 3,
        };
        var aSamples = new math_Vector(1, 41);
        for (int i = 1; i <= 41; i++)
        {
            aSamples[i] = (i - 1) * 0.125;
        }
        AllRootsResult aResult = MathRoot.FindAllRootsWithIntervals(new Fn(F), aSamples);
        Assert.True(aResult.IsDone);
        AssertAreRoots(F, aResult.Roots);
        Assert.Contains(aResult.NullIntervals, n => Math.Abs(n.A - 2) < 0.2 && Math.Abs(n.B - 3) < 0.2);
    }

    [Fact]
    public void All_roots_with_intervals_on_a_plain_function_and_its_failures()
    {
        var aSamples = new math_Vector(1, 41);
        for (int i = 1; i <= 41; i++)
        {
            aSamples[i] = -2 + ((i - 1) * 0.1);
        }
        AllRootsResult aRoots = MathRoot.FindAllRootsWithIntervals(Cubic, aSamples);
        Assert.True(aRoots.IsDone);
        Assert.Equal(3, aRoots.Roots.Count);
        Assert.Empty(aRoots.NullIntervals);

        var aOne = new math_Vector(1, 1);
        Assert.Equal(Status.InvalidInput, MathRoot.FindAllRootsWithIntervals(Cubic, aOne).Status);
        Assert.Equal(Status.NotConverged, MathRoot.FindAllRootsWithIntervals(Refusing(-5), aSamples).Status);
        Assert.Equal(Status.NotConverged, MathRoot.FindAllRootsWithIntervals(Refusing(0), aSamples).Status);
        // a function that's nothing everywhere is one null interval
        AllRootsResult aNull = MathRoot.FindAllRootsWithIntervals(new Fn(_ => 0), aSamples);
        Assert.True(aNull.IsDone);
        Assert.Single(aNull.NullIntervals);
        Assert.Empty(aNull.Roots);
    }

    // ---- trigonometric equations: a cos^2 + 2 b cos sin + c cos + d sin + e = 0 ----

    private static double Trig(double a, double b, double c, double d, double e, double x) =>
        (a * Math.Cos(x) * Math.Cos(x)) + (2 * b * Math.Cos(x) * Math.Sin(x)) + (c * Math.Cos(x)) + (d * Math.Sin(x)) + e;

    [Theory]
    [InlineData(1, 0, 0, 0, -0.25)] // cos^2 x = 1/4
    [InlineData(0, 1, 0, 0, 0)] // sin 2x = 0
    [InlineData(0, 0, 1, 0, -0.5)] // cos x = 1/2
    [InlineData(0, 0, 0, 1, -0.5)] // sin x = 1/2
    [InlineData(0, 0, 1, 1, 0)] // tan x = -1
    [InlineData(1, 0.5, 0.5, 0.25, -0.2)]
    [InlineData(1, 1, 1, 1, -1)]
    [InlineData(0.5, -0.25, 0.1, -0.3, -0.1)]
    public void The_roots_of_a_trigonometric_equation_satisfy_it(double a, double b, double c, double d, double e)
    {
        TrigResult aResult = MathRoot.Trigonometric(a, b, c, d, e);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbRoots > 0);
        for (int i = 0; i < aResult.NbRoots; i++)
        {
            Assert.Equal(0, Trig(a, b, c, d, e, aResult.Roots[i]), 6);
            Assert.InRange(aResult.Roots[i], 0, 2 * Math.PI);
        }
    }

    [Theory]
    [InlineData(0.5, 1.5)]
    [InlineData(-3, 1)]
    [InlineData(4, 10)]
    public void The_roots_of_a_trigonometric_equation_stay_in_their_bounds(double theInf, double theSup)
    {
        TrigResult aResult = MathRoot.Trigonometric(0, 1, 0, 0, 0, theInf, theSup); // sin 2x = 0
        Assert.True(aResult.IsDone);
        for (int i = 0; i < aResult.NbRoots; i++)
        {
            Assert.InRange(aResult.Roots[i], theInf - 1e-9, theSup + 1e-9);
            Assert.Equal(0, Math.Sin(2 * aResult.Roots[i]), 6);
        }
    }

    [Fact]
    public void A_trigonometric_equation_with_no_terms_or_no_solution()
    {
        TrigResult anAll = MathRoot.Trigonometric(0, 0, 0, 0, 0);
        Assert.True(anAll.InfiniteRoots);
        TrigResult aNone = MathRoot.Trigonometric(0, 0, 0, 0, 5);
        Assert.Equal(0, aNone.NbRoots);
        TrigResult aFar = MathRoot.Trigonometric(0, 0, 1, 0, 3); // cos x = -3
        Assert.Equal(0, aFar.NbRoots);
    }
}
