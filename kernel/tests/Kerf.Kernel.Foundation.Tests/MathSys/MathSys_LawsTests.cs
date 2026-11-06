// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Numerics;
using Kerf.Engine.Numerics.Systems;
using Kerf.Engine.Numerics.Utilities;
using static Kerf.Engine.Numerics.Systems.MathSys;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// The system solvers on systems that misbehave (decision 19805): a function that can't be evaluated or a singular
/// Jacobian is reported, never returned as a root; a bounded root stays in its bounds; a root found makes the residual vanish.
/// </summary>
public class MathSys_LawsTests
{
    // x + y = 3, x - y = 1, with a fail switch for each stage
    private sealed class Linear(bool theValueFails = false, bool theDerivativeFails = false, bool theSingular = false, int theFailAfter = int.MaxValue)
        : IFunctionSetWithDerivatives
    {
        private int myCalls;

        public int NbVariables() => 2;

        public int NbEquations() => 2;

        public bool Value(math_Vector theX, math_Vector theF)
        {
            theF[1] = theX[1] + theX[2] - 3;
            theF[2] = theSingular ? theX[1] + theX[2] - 1 : theX[1] - theX[2] - 1;
            return !theValueFails && ++myCalls <= theFailAfter;
        }

        public bool Derivatives(math_Vector theX, Matrix theD)
        {
            theD[1, 1] = 1;
            theD[1, 2] = 1;
            theD[2, 1] = 1;
            theD[2, 2] = theSingular ? 1 : -1;
            return !theDerivativeFails;
        }

        public bool Values(math_Vector theX, math_Vector theF, Matrix theD) => Value(theX, theF) && Derivatives(theX, theD);
    }

    private static math_Vector Vec(double a, double b)
    {
        var aV = new math_Vector(1, 2);
        aV[1] = a;
        aV[2] = b;
        return aV;
    }

    [Fact]
    public void Newton_solves_a_linear_system_at_once_and_stays_in_bounds()
    {
        VectorResult aFree = Newton(new Linear(), Vec(0, 0), Vec(1e-10, 1e-10), 1e-10);
        Assert.True(aFree.IsDone);
        Assert.Equal(2, aFree.Solution![1], 9);
        Assert.Equal(1, aFree.Solution![2], 9);
        VectorResult aBounded = NewtonBounded(new Linear(), Vec(0, 0), Vec(0, 0), Vec(1.5, 1.5), Vec(1e-10, 1e-10), 1e-10);
        Assert.InRange(aBounded.Solution![1], 0, 1.5);
        Assert.InRange(aBounded.Solution![2], 0, 1.5);
    }

    [Fact]
    public void Newton_reports_what_it_cannot_do()
    {
        var aStart = Vec(0, 0);
        var aTol = Vec(1e-10, 1e-10);
        Assert.Equal(Status.NumericalError, Newton(new Linear(theValueFails: true), aStart, aTol, 1e-10).Status);
        Assert.Equal(Status.NumericalError, Newton(new Linear(theDerivativeFails: true), aStart, aTol, 1e-10).Status);
        Assert.NotEqual(Status.OK, Newton(new Linear(theSingular: true), aStart, aTol, 1e-10).Status);
        Assert.Equal(Status.InvalidInput, Newton(new Linear(), new math_Vector(1, 3), aTol, 1e-10).Status);
        Assert.Equal(Status.InvalidInput, Newton(new Linear(), aStart, new math_Vector(1, 3), 1e-10).Status);
        var aMin = Vec(0, 0);
        var aMax = Vec(5, 5);
        Assert.Equal(Status.NumericalError, NewtonBounded(new Linear(theValueFails: true), aStart, aMin, aMax, aTol, 1e-10).Status);
        Assert.Equal(Status.NumericalError, NewtonBounded(new Linear(theDerivativeFails: true), aStart, aMin, aMax, aTol, 1e-10).Status);
        Assert.NotEqual(Status.OK, NewtonBounded(new Linear(theSingular: true), aStart, aMin, aMax, aTol, 1e-10).Status);
        Assert.Equal(Status.InvalidInput, NewtonBounded(new Linear(), new math_Vector(1, 3), aMin, aMax, aTol, 1e-10).Status);
        Assert.Equal(Status.InvalidInput, NewtonBounded(new Linear(), aStart, new math_Vector(1, 3), aMax, aTol, 1e-10).Status);
    }

    [Fact]
    public void Levenberg_Marquardt_reports_what_it_cannot_do_and_finds_the_least_squares_of_the_rest()
    {
        var aStart = Vec(0, 0);
        Assert.Equal(Status.NumericalError, LevenbergMarquardt(new Linear(theValueFails: true), aStart).Status);
        Assert.Equal(Status.NumericalError, LevenbergMarquardt(new Linear(theDerivativeFails: true), aStart).Status);
        Assert.Equal(Status.InvalidInput, LevenbergMarquardt(new Linear(), new math_Vector(1, 3)).Status);
        var aMin = Vec(0, 0);
        var aMax = Vec(5, 5);
        Assert.Equal(Status.NumericalError, LevenbergMarquardtBounded(new Linear(theValueFails: true), aStart, aMin, aMax).Status);
        Assert.Equal(Status.NumericalError, LevenbergMarquardtBounded(new Linear(theDerivativeFails: true), aStart, aMin, aMax).Status);
        Assert.Equal(Status.InvalidInput, LevenbergMarquardtBounded(new Linear(), new math_Vector(1, 3), aMin, aMax).Status);
        Assert.Equal(Status.InvalidInput, LevenbergMarquardtBounded(new Linear(), aStart, new math_Vector(1, 3), aMax).Status);

        // x + y = 3 and x + y = 1 can't both hold: halfway, x + y = 2, is the best there is
        VectorResult aBest = LevenbergMarquardt(new Linear(theSingular: true), aStart);
        Assert.Equal(2, aBest.Solution![1] + aBest.Solution![2], 4);
        // a bound that rules out the root stops at it
        VectorResult aBounded = LevenbergMarquardtBounded(new Linear(), aStart, Vec(0, 0), Vec(1, 1));
        Assert.InRange(aBounded.Solution![1], 0, 1);
        Assert.InRange(aBounded.Solution![2], 0, 1);
        // a function that fails on the way isn't taken as far as its failure
        VectorResult aLater = LevenbergMarquardt(new Linear(theFailAfter: 1), aStart);
        Assert.NotEqual(Status.OK, aLater.Status);
    }

    private struct Failing2D : INewton2DFunction
    {
        public bool Invoke(double theU, double theV, ref Array2<double> theF, ref Array2<Array2<double>> theJ) => false;
    }

    private struct Singular2D : INewton2DFunction
    {
        public bool Invoke(double theU, double theV, ref Array2<double> theF, ref Array2<Array2<double>> theJ)
        {
            theF[0] = theU + theV - 1;
            theF[1] = theU + theV - 3;
            theJ[0][0] = 1;
            theJ[0][1] = 1;
            theJ[1][0] = 1;
            theJ[1][1] = 1;
            return true;
        }
    }

    private static NewtonBoundsN2 Bounds2D(double theMin, double theMax)
    {
        var aBounds = new NewtonBoundsN2();
        aBounds.Min[0] = theMin;
        aBounds.Min[1] = theMin;
        aBounds.Max[0] = theMax;
        aBounds.Max[1] = theMax;
        return aBounds;
    }

    [Fact]
    public void The_2D_solvers_report_a_function_that_fails_or_is_singular()
    {
        var aStart = new Array2<double>();
        NewtonBoundsN2 aBounds = Bounds2D(-5, 5);
        Assert.Equal(Status.NumericalError, Solve2D(new Failing2D(), aStart, aBounds).Status);
        Assert.NotEqual(Status.OK, Solve2D(new Singular2D(), aStart, aBounds).Status);
        Assert.Equal(Status.InvalidInput, Solve2D(new Failing2D(), aStart, Bounds2D(5, -5)).Status);
    }
}
