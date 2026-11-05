// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Numerics;
using static Kerf.Engine.Numerics.Roots.MathRoot;
using static Kerf.Engine.Numerics.Systems.MathSys;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// The numerics' brackets, composable functions, full trigonometric equations and symmetric systems, against values worked by
/// hand (unit 11I).
/// </summary>
public class MathNumericsHelpersTests
{
    private static double At(IScalarFunction theF, double theX)
    {
        Assert.True(theF.Value(theX, out double aY));
        return aY;
    }

    private sealed class Failing : IScalarFunction
    {
        public bool Value(double theX, out double theY)
        {
            theY = 0;
            return false;
        }
    }

    // Brackets

    [Fact]
    public void A_root_is_bracketed_by_expanding_outwards()
    {
        // the root of x - 10 is outside [0, 1]
        BracketResult aResult = BracketRoot(new Linear(1, -10), 0, 1);
        Assert.True(aResult.IsValid);
        Assert.True(aResult.Fa * aResult.Fb <= 0);
        Assert.InRange(10.0, Math.Min(aResult.A, aResult.B), Math.Max(aResult.A, aResult.B));

        // already bracketed, stays as it is
        BracketResult aGiven = BracketRoot(new Linear(1, -0.5), 0, 1);
        Assert.Equal((0.0, 1.0), (aGiven.A, aGiven.B));

        // no root to find, and a function that fails
        Assert.False(BracketRoot(new Constant(1), 0, 1, 5).IsValid);
        Assert.False(BracketRoot(new Failing(), 0, 1).IsValid);
    }

    [Fact]
    public void A_minimum_is_bracketed_by_a_low_point_between_two_high_ones()
    {
        // (x - 7)^2
        ScalarLambda aParabola = new((double theX, out double theY) => { theY = (theX - 7) * (theX - 7); return true; });
        MinBracketResult aResult = BracketMinimum(aParabola, 0, 1);
        Assert.True(aResult.IsValid);
        Assert.True(aResult.Fb <= aResult.Fa && aResult.Fb <= aResult.Fc);
        Assert.InRange(7.0, Math.Min(aResult.A, aResult.C), Math.Max(aResult.A, aResult.C));

        // from the other side, given the wrong way round
        MinBracketResult aBackwards = BracketMinimum(aParabola, 20, 19);
        Assert.True(aBackwards.IsValid);
        Assert.InRange(7.0, Math.Min(aBackwards.A, aBackwards.C), Math.Max(aBackwards.A, aBackwards.C));

        // nothing to bracket on a falling line within few steps, or if the function fails
        Assert.False(BracketMinimum(new Linear(-1, 0), 0, 1, 3).IsValid);
        Assert.False(BracketMinimum(new Failing(), 0, 1).IsValid);
    }

    // Functions composed

    [Fact]
    public void Scalar_functions_compose_as_their_names_say()
    {
        Linear aF = new(2, 1);   // 2x + 1
        Constant aG = new(3);
        Assert.Equal(7.0, At(new Composite<Linear, Linear>(aF, new Linear(1, 2)), 1)); // 2 (x + 2) + 1
        Assert.Equal(6.0, At(new Sum<Linear, Constant>(aF, aG), 1));
        Assert.Equal(0.0, At(new Difference<Linear, Constant>(aF, aG), 1));
        Assert.Equal(9.0, At(new Product<Linear, Constant>(aF, aG), 1));
        Assert.Equal(1.0, At(new Quotient<Linear, Constant>(aF, aG), 1));
        Assert.Equal(6.0, At(new Scaled<Linear>(aF, 2), 1));
        Assert.Equal(4.0, At(new Shifted<Linear>(aF, 1), 1)); // f(x) + 1
        Assert.Equal(-3.0, At(new Negated<Linear>(aF), 1));

        // a quotient by nil has no value, and failures pass on
        Assert.False(new Quotient<Linear, Constant>(aF, new Constant(0)).Value(1, out _));
        Failing aBad = new();
        Assert.False(new Composite<Failing, Linear>(aBad, aF).Value(1, out _));
        Assert.False(new Composite<Linear, Failing>(aF, aBad).Value(1, out _));
        Assert.False(new Sum<Linear, Failing>(aF, aBad).Value(1, out _));
        Assert.False(new Sum<Failing, Linear>(aBad, aF).Value(1, out _));
        Assert.False(new Difference<Linear, Failing>(aF, aBad).Value(1, out _));
        Assert.False(new Difference<Failing, Linear>(aBad, aF).Value(1, out _));
        Assert.False(new Product<Linear, Failing>(aF, aBad).Value(1, out _));
        Assert.False(new Product<Failing, Linear>(aBad, aF).Value(1, out _));
        Assert.False(new Quotient<Linear, Failing>(aF, aBad).Value(1, out _));
        Assert.False(new Quotient<Failing, Linear>(aBad, aF).Value(1, out _));
        Assert.False(new Scaled<Failing>(aBad, 2).Value(1, out _));
        Assert.False(new Shifted<Failing>(aBad, 2).Value(1, out _));
        Assert.False(new Negated<Failing>(aBad).Value(1, out _));
    }

    [Fact]
    public void Elementary_functions_and_their_derivatives()
    {
        // 1 + 2x + 3x^2 at 2
        Polynomial aPoly = new(new[] { 1.0, 2.0, 3.0 });
        Assert.Equal(17.0, At(aPoly, 2));
        Assert.True(aPoly.Values(2, out double aY, out double aDy));
        Assert.Equal((17.0, 14.0), (aY, aDy));

        Exponential anExp = new(2, 3); // 2 e^(3x)
        Assert.Equal(2 * Math.Exp(3), At(anExp, 1), 12);
        Assert.True(anExp.Values(1, out aY, out aDy));
        Assert.Equal(6 * Math.Exp(3), aDy, 12);

        Power aPower = new(3, 2); // 2 x^3
        Assert.Equal(16.0, At(aPower, 2), 12);
        Assert.True(aPower.Values(2, out aY, out aDy));
        Assert.Equal((16.0, 24.0), (aY, aDy));

        Gaussian aBell = new(2, 1, 3); // 2 exp(-(x - 1)^2 / (2 * 9))
        Assert.Equal(2.0, At(aBell, 1), 12);
        Assert.True(aBell.Values(4, out aY, out aDy));
        Assert.Equal(2 * Math.Exp(-0.5), aY, 12);
        Assert.Equal(-aY * 3 / 9, aDy, 12);
    }

    // Trigonometric equations: a cos^2 + 2 b cos sin + c cos + d sin + e = 0

    public static TheoryData<double, double, double, double, double, int> Equations => new()
    {
        { 1, 0, 0, 0, -0.25, 4 },      // cos^2 = 1/4
        { 0, 0.5, 0, 0, 0, 4 },        // cos sin = 0, at 0 and pi/2 and on
        { 0, 0.5, 1, 1, 0, 2 },        // the general solver, with a linear part
        { 0, 0, 1, 0, -0.5, 2 },       // cos = 1/2
        { 0, 0, 0, 2, -1, 2 },         // sin = 1/2
        { 1, 0, 1, 0, 0, 3 },          // cos (cos + 1)
        { 1, 0.25, 0.5, -0.5, -0.1, 2 },
        { 1, 0, 0, 0, 0.5, 0 },        // cos^2 = -1/2: none
        { 0, 0, 1, 1, 2, 0 },          // cos + sin = -2: none
    };

    [Theory]
    [MemberData(nameof(Equations))]
    public void Each_trigonometric_root_satisfies_the_equation(double theA, double theB, double theC, double theD, double theE, int theCount)
    {
        TrigResult aResult = Trigonometric(theA, theB, theC, theD, theE);
        Assert.True(aResult.IsDone, aResult.Status.ToString());
        Assert.Equal(theCount, aResult.NbRoots);
        for (int i = 0; i < aResult.NbRoots; i++)
        {
            double aX = aResult.Roots[i], aCos = Math.Cos(aX), aSin = Math.Sin(aX);
            Assert.Equal(0.0, theA * aCos * aCos + 2 * theB * aCos * aSin + theC * aCos + theD * aSin + theE, 8);
            Assert.InRange(aX, 0, 2 * Math.PI + 1e-9);
        }
    }

    [Fact]
    public void Trigonometric_roots_keep_to_their_range()
    {
        // sin x = 1/2 has pi/6 and 5 pi/6: only the first in [0, 1]
        TrigResult aResult = Trigonometric(0, 0, 0, 1, -0.5, 0, 1);
        Assert.Equal(1, aResult.NbRoots);
        Assert.Equal(Math.PI / 6, aResult.Roots[0], 1e-9);
        // a nil equation on a range: every angle
        Assert.True(TrigonometricLinear(0, 0, 0, 1).InfiniteRoots);
        // no roots from a non-zero constant
        Assert.Equal(0, TrigonometricLinear(0, 1).NbRoots);
        // a range wider than a turn counts one turn
        Assert.Equal(2, Trigonometric(0, 0, 0, 1, -0.5, 0, 4 * Math.PI).NbRoots);
    }

    // Symmetric 2D Newton: the gradient of (u - 1)^2 + 2 (v + 3)^2 = 0, its Jacobian the Hessian

    private sealed class Gradient2D(bool theFails = false, bool theSingular = false) : INewton2DSymmetricFunction, INewton2DFunction
    {
        public bool ValueAndJacobian(double theU, double theV, out double theF1, out double theF2, out double theJ11, out double theJ12, out double theJ22)
        {
            Value(theU, theV, out theF1, out theF2);
            theJ11 = theSingular ? 0 : 2;
            theJ12 = 0;
            theJ22 = theSingular ? 0 : 4;
            return !theFails;
        }

        public bool Value(double theU, double theV, out double theF1, out double theF2)
        {
            theF1 = 2 * (theU - 1);
            theF2 = 4 * (theV + 3);
            return !theFails;
        }

        public bool Invoke(double theU, double theV, ref Array2<double> theF, ref Array2<Array2<double>> theJ)
        {
            theF[0] = 2 * (theU - 1);
            theF[1] = 4 * (theV + 3);
            theJ[0][0] = theSingular ? 0 : 2;
            theJ[0][1] = 0;
            theJ[1][0] = 0;
            theJ[1][1] = theSingular ? 0 : 4;
            return !theFails;
        }
    }

    private static NewtonBoundsN2 Bounds2(double theMin, double theMax)
    {
        NewtonBoundsN2 aBounds = new();
        aBounds.Min[0] = aBounds.Min[1] = theMin;
        aBounds.Max[0] = aBounds.Max[1] = theMax;
        return aBounds;
    }

    private static Array2<double> Start2(double theU, double theV)
    {
        Array2<double> aStart = default;
        aStart[0] = theU;
        aStart[1] = theV;
        return aStart;
    }

    [Fact]
    public void A_symmetric_system_is_solved_within_bounds_or_reported()
    {
        NewtonResultN2 aResult = Solve2DSymmetric(new Gradient2D(), Start2(5, 5), Bounds2(-10, 10));
        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.X[0], 1e-9);
        Assert.Equal(-3.0, aResult.X[1], 1e-9);

        // a failing function, and a singular Jacobian, are reported
        Assert.False(Solve2DSymmetric(new Gradient2D(true), Start2(5, 5), Bounds2(-10, 10)).IsDone);
        Assert.False(Solve2DSymmetric(new Gradient2D(false, true), Start2(5, 5), Bounds2(-10, 10)).IsDone);
        Assert.False(Solve2D(new Gradient2D(true), Start2(5, 5), Bounds2(-10, 10)).IsDone);
        Assert.False(Solve2D(new Gradient2D(false, true), Start2(5, 5), Bounds2(-10, 10)).IsDone);

        // the answer outside the bounds isn't reached
        NewtonResultN2 aBeyond = Solve2DSymmetric(new Gradient2D(), Start2(5, 5), Bounds2(2, 10));
        Assert.False(aBeyond.IsDone && aBeyond.X[1] < 2);
    }

    private sealed class Linear3 : INewton3DFunction
    {
        public bool Invoke(double theX1, double theX2, double theX3, ref Array3<double> theF, ref Array3<Array3<double>> theJ)
        {
            theF[0] = theX1 - 1;
            theF[1] = theX2 - 2;
            theF[2] = theX3 - 3;
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    theJ[i][j] = i == j ? 1 : 0;
                }
            }
            return true;
        }
    }

    private sealed class Linear4 : INewton4DFunction
    {
        public bool Invoke(double theX1, double theX2, double theX3, double theX4, ref Array4<double> theF, ref Array4<Array4<double>> theJ)
        {
            theF[0] = theX1 - 1;
            theF[1] = theX2 - 2;
            theF[2] = theX3 - 3;
            theF[3] = theX4 - 4;
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    theJ[i][j] = i == j ? 1 : 0;
                }
            }
            return true;
        }
    }

    [Fact]
    public void Three_and_four_dimensional_newton_on_linear_systems_arrive_at_once()
    {
        NewtonBoundsN3 aBounds3 = new();
        for (int i = 0; i < 3; i++)
        {
            aBounds3.Min[i] = -10;
            aBounds3.Max[i] = 10;
        }
        Array3<double> aStart3 = default;
        NewtonResultN3 aResult3 = Solve3D(new Linear3(), aStart3, aBounds3);
        Assert.True(aResult3.IsDone);
        Assert.Equal((1.0, 2.0, 3.0), (Math.Round(aResult3.X[0], 9), Math.Round(aResult3.X[1], 9), Math.Round(aResult3.X[2], 9)));

        NewtonBoundsN4 aBounds4 = new();
        for (int i = 0; i < 4; i++)
        {
            aBounds4.Min[i] = -10;
            aBounds4.Max[i] = 10;
        }
        Array4<double> aStart4 = default;
        NewtonResultN4 aResult4 = Solve4D(new Linear4(), aStart4, aBounds4);
        Assert.True(aResult4.IsDone);
        Assert.Equal(4.0, aResult4.X[3], 9);
    }

    [Fact]
    public void All_roots_on_the_edges_of_a_range_and_tangent_roots()
    {
        // x^2 touches zero at 0, in the middle of the range: a double root
        MultipleResult aTangent = FindMultipleRoots(new Power(1, 2), -1, 1, 21, 1e-9, 1e-9);
        Assert.True(aTangent.IsDone);
        Assert.True(aTangent.Roots.Count >= 1);
        Assert.All(aTangent.Roots, r => Assert.True(Math.Abs(r) < 1e-3));

        // the range's end is a root
        MultipleResult anEnd = FindMultipleRoots(new Linear(1, -1), 0, 1, 11, 1e-9, 1e-9);
        Assert.True(anEnd.IsDone);
        Assert.Contains(anEnd.Roots, r => Math.Abs(r - 1) < 1e-7);

        // with an offset: sin x = 1/2 from the shifted function sin x - offset
        MultipleResult anOffset = FindMultipleRoots(new Sine(), 0, 3, 31, 1e-9, 1e-9, 0.5);
        Assert.True(anOffset.IsDone);
        Assert.Equal(2, anOffset.Roots.Count);
        Assert.Equal(Math.PI / 6, anOffset.Roots.Min(), 1e-6);

        // the whole of a nil function
        MultipleResult aNil = FindMultipleRoots(new Constant(0), 0, 1, 11, 1e-9, 1e-9);
        Assert.True(aNil.IsAllNull);
    }
}
