// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Numerics;
using static Kerf.Engine.Numerics.Integration.MathInteg;
using static Kerf.Engine.Numerics.Optimisation.MathOpt;
using static Kerf.Engine.Numerics.Roots.MathRoot;
using static Kerf.Engine.Numerics.Systems.MathSys;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Kernel.Foundation.Tests;

/// <summary>
/// What each solver does when its function can't be evaluated, or it isn't given a problem it can solve (unit 11I): it reports
/// that, by its status, rather than go on with a value it doesn't have. The callers' fallbacks all depend on it.
/// </summary>
public class MathNumericsFailuresTests
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

    /// <summary>Can't be evaluated from the given call on, so a solver meets it at the start or part way.</summary>
    private sealed class Failing(int theGoodCalls = 0) : IScalarFunction, IScalarFunctionWithDerivative, IVectorFunctionWithHessian, IFunctionSetWithDerivatives, ISetFunction
    {
        private int myCalls;

        private bool Good() => myCalls++ < theGoodCalls;

        public bool Value(double theX, out double theY)
        {
            theY = theX * theX - 2;
            return Good();
        }

        public bool Values(double theX, out double theY, out double theDy)
        {
            theY = theX * theX - 2;
            theDy = 2 * theX;
            return Good();
        }

        public bool Value(math_Vector theX, out double theY)
        {
            theY = theX[1] * theX[1] + theX[2] * theX[2];
            return Good();
        }

        public bool Gradient(math_Vector theX, math_Vector theG)
        {
            theG[1] = 2 * theX[1];
            theG[2] = 2 * theX[2];
            return Good();
        }

        public bool Hessian(math_Vector theX, Matrix theH)
        {
            theH[1, 1] = 2;
            theH[1, 2] = 0;
            theH[2, 1] = 0;
            theH[2, 2] = 2;
            return Good();
        }

        public int NbVariables() => 2;

        public int NbEquations() => 2;

        public bool Value(math_Vector theX, math_Vector theF)
        {
            theF[theF.Lower()] = theX[theX.Lower()];
            theF[theF.Lower() + 1] = theX[theX.Lower()];
            return Good();
        }

        public bool Derivatives(math_Vector theX, Matrix theD)
        {
            theD[1, 1] = 1;
            theD[1, 2] = 0;
            theD[2, 1] = 0;
            theD[2, 2] = 1;
            return Good();
        }

        public bool Values(math_Vector theX, math_Vector theF, Matrix theD) => Value(theX, theF) && Derivatives(theX, theD);
    }

    private static void Refuses(bool theIsDone, Status theStatus)
    {
        Assert.False(theIsDone);
        Assert.NotEqual(Status.OK, theStatus);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void Minimisers_report_a_function_that_fails(int theGoodCalls)
    {
        Failing F() => new(theGoodCalls);
        math_Vector aStart = Vec(3, 4);
        void Check(VectorResult theResult) => Refuses(theResult.IsDone, theResult.Status);

        Check(global::Kerf.Engine.Numerics.Optimisation.MathOpt.Newton(F(), aStart));
        Check(NewtonModified(F(), aStart));
        Check(NewtonNumericalHessian(F(), aStart));
        Check(global::Kerf.Engine.Numerics.Optimisation.MathOpt.NewtonBounded(F(), aStart, Vec(-5, -5), Vec(5, 5)));
        Check(BFGS(F(), aStart));
        Check(BFGSNumerical(F(), aStart));
        Check(LBFGS(F(), aStart));
        Check(BFGSBounded(F(), aStart, Vec(-5, -5), Vec(5, 5)));
        Check(FRPR(F(), aStart));
        Check(FRPRNumerical(F(), aStart));
    }

    [Fact]
    public void Powell_refuses_a_function_failing_at_the_start()
    {
        VectorResult aResult = Powell(new Failing(), Vec(3, 4));
        Refuses(aResult.IsDone, aResult.Status);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void Scalar_root_finders_report_a_function_that_fails(int theGoodCalls)
    {
        Failing F() => new(theGoodCalls);
        Refuses(global::Kerf.Engine.Numerics.Roots.MathRoot.Newton(F(), 1).IsDone, global::Kerf.Engine.Numerics.Roots.MathRoot.Newton(F(), 1).Status);
        ScalarResult aBounded = global::Kerf.Engine.Numerics.Roots.MathRoot.NewtonBounded(F(), 1, 0, 3);
        Refuses(aBounded.IsDone, aBounded.Status);
        ScalarResult aSecant = Secant(F(), 1, 2);
        Refuses(aSecant.IsDone, aSecant.Status);
        ScalarResult anAuto = SecantAuto(F(), 1);
        Refuses(anAuto.IsDone, anAuto.Status);
        ScalarResult aBisection = Bisection(F(), 0, 3);
        Refuses(aBisection.IsDone, aBisection.Status);
        ScalarResult aBoth = BisectionNewton(F(), 0, 3);
        Refuses(aBoth.IsDone, aBoth.Status);
        ScalarResult aBrent = global::Kerf.Engine.Numerics.Roots.MathRoot.Brent(F(), 0, 3);
        Refuses(aBrent.IsDone, aBrent.Status);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void System_solvers_report_a_function_that_fails(int theGoodCalls)
    {
        Failing F() => new(theGoodCalls);
        math_Vector aStart = Vec(3, 4);
        void Check(VectorResult theResult) => Refuses(theResult.IsDone, theResult.Status);
        Check(global::Kerf.Engine.Numerics.Systems.MathSys.Newton(F(), aStart, 1e-9, 1e-9));
        Check(global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBounded(F(), aStart, Vec(-5, -5), Vec(5, 5), Vec(1e-9, 1e-9), 1e-9));
        Check(LevenbergMarquardt(F(), aStart));
        Check(LevenbergMarquardtBounded(F(), aStart, Vec(-5, -5), Vec(5, 5)));
    }

    [Fact]
    public void Integrators_report_a_function_that_fails()
    {
        void Check(IntegResult theResult) => Refuses(theResult.IsDone, theResult.Status);
        Check(Gauss(new Failing(), 0, 1));
        Check(GaussAdaptive(new Failing(), 0, 1));
        Check(GaussComposite(new Failing(), 0, 1, 3));
        Check(KronrodRule(new Failing(), 0, 1));
        Check(Kronrod(new Failing(), 0, 1));
        Check(KronrodAuto(new Failing(), 0, 1));
        SetResult aSet = GaussSet(new Failing(), 0.0, 1.0, 4);
        Refuses(aSet.IsDone, aSet.Status);
    }

    private sealed class HoleInTheRange : IScalarFunction
    {
        public bool Value(double theX, out double theY)
        {
            theY = theX * theX;
            return theX is < 0.2 or > 0.2001;
        }
    }

    [Fact]
    public void The_tanh_sinh_rules_step_over_points_that_fail()
    {
        // meant for integrands with singularities: a point it can't evaluate counts for nothing, so x^2 still gives 1/3
        IntegResult aResult = TanhSinh(new HoleInTheRange(), 0, 1);
        Assert.True(aResult.IsDone);
        Assert.Equal(1.0 / 3.0, aResult.Value!.Value, 1e-3);
    }

    [Fact]
    public void A_null_range_or_mismatched_sizes_are_refused()
    {
        // dimensions that don't match
        math_Vector aStart = Vec(1, 2);
        Refuses(global::Kerf.Engine.Numerics.Optimisation.MathOpt.NewtonBounded(new Failing(100), aStart, Vec(0), Vec(1, 1)).IsDone, Status.InvalidInput);
        Refuses(BFGSBounded(new Failing(100), aStart, Vec(0), Vec(1, 1)).IsDone, Status.InvalidInput);
        Refuses(LevenbergMarquardtBounded(new Failing(100), aStart, Vec(0), Vec(1, 1)).IsDone, Status.InvalidInput);

        // the Gauss integrators take no orders below one, and a uniform order past the tables is clamped: both integrate x
        IntegResult aClamped = GaussMultipleUniform(new OneVariable(), 1, Vec(0), Vec(2), 1000);
        Assert.True(aClamped.IsDone);
        Assert.Equal(2.0, aClamped.Value!.Value, 1e-9);
    }

    private sealed class OneVariable : IVectorFunction
    {
        public bool Value(math_Vector theX, out double theY)
        {
            theY = theX[theX.Lower()];
            return true;
        }
    }

    [Fact]
    public void The_results_convert_to_their_success()
    {
        CroutResultConversions();

        static void CroutResultConversions()
        {
            ScalarResult aScalar = new() { Status = Status.OK };
            Assert.True((bool)aScalar);
            Assert.False((bool)new ScalarResult());
            VectorResult aVector = new() { Status = Status.OK };
            Assert.True((bool)aVector);
            Assert.False((bool)new VectorResult());
            PolyResult aPoly = new() { Status = Status.OK };
            Assert.True((bool)aPoly);
            LinearResult aLinear = new() { Status = Status.OK };
            Assert.True((bool)aLinear);
            InverseResult anInverse = new() { Status = Status.OK };
            Assert.True((bool)anInverse);
            IntegResult anInteg = new() { Status = Status.OK };
            Assert.True((bool)anInteg);
            SetResult aSet = new() { Status = Status.OK };
            Assert.True((bool)aSet);
            NewtonResultN2 aNewton2 = new() { Status = Status.OK };
            Assert.True((bool)aNewton2);
            NewtonResultN3 aNewton3 = new() { Status = Status.OK };
            Assert.True((bool)aNewton3);
            NewtonResultN4 aNewton4 = new() { Status = Status.OK };
            Assert.True((bool)aNewton4);
        }
    }
}
