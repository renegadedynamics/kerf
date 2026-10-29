// Port of: src/FoundationClasses/TKMath/GTests/MathRoot_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// C++ reaches MathUtils' names through MathRoot:: (MathRoot::ScalarResult, MathRoot::Config) by its
// `using namespace MathUtils`; C# names them MathUtils.ScalarResult and so on.

namespace OcctNet.Foundation.Tests;

internal static class MathRootTestFunctions
{
    internal const double THE_TOLERANCE = 1.0e-10;
    internal const double THE_PI = 3.14159265358979323846;

    // Function with derivative: f(x) = x^2 - 2, f'(x) = 2x
    internal sealed class SqrtTwoFunc : MathUtils.IScalarFunction, MathUtils.IScalarFunctionWithDerivative
    {
        public bool Value(double theX, out double theF)
        {
            theF = theX * theX - 2.0;
            return true;
        }

        public bool Values(double theX, out double theF, out double theDf)
        {
            theF = theX * theX - 2.0;
            theDf = 2.0 * theX;
            return true;
        }
    }

    // Function: f(x) = cos(x) - x
    internal sealed class CosMinusXFunc : MathUtils.IScalarFunction, MathUtils.IScalarFunctionWithDerivative
    {
        public bool Value(double theX, out double theF)
        {
            theF = Math.Cos(theX) - theX;
            return true;
        }

        public bool Values(double theX, out double theF, out double theDf)
        {
            theF = Math.Cos(theX) - theX;
            theDf = -Math.Sin(theX) - 1.0;
            return true;
        }
    }

    // Function: f(x) = x^3 - x - 2
    internal sealed class CubicFunc : MathUtils.IScalarFunction, MathUtils.IScalarFunctionWithDerivative
    {
        public bool Value(double theX, out double theF)
        {
            theF = theX * theX * theX - theX - 2.0;
            return true;
        }

        public bool Values(double theX, out double theF, out double theDf)
        {
            theF = theX * theX * theX - theX - 2.0;
            theDf = 3.0 * theX * theX - 1.0;
            return true;
        }
    }

    // Function: f(x) = sin(x)
    internal sealed class SinFunc : MathUtils.IScalarFunction, MathUtils.IScalarFunctionWithDerivative
    {
        public bool Value(double theX, out double theF)
        {
            theF = Math.Sin(theX);
            return true;
        }

        public bool Values(double theX, out double theF, out double theDf)
        {
            theF = Math.Sin(theX);
            theDf = Math.Cos(theX);
            return true;
        }
    }

    // Function: f(x) = e^x - 3
    internal sealed class ExpMinusThreeFunc : MathUtils.IScalarFunction, MathUtils.IScalarFunctionWithDerivative
    {
        public bool Value(double theX, out double theF)
        {
            theF = Math.Exp(theX) - 3.0;
            return true;
        }

        public bool Values(double theX, out double theF, out double theDf)
        {
            theF = Math.Exp(theX) - 3.0;
            theDf = Math.Exp(theX);
            return true;
        }
    }

    // Linear function: f(x) = 2x - 4
    internal sealed class LinearFunc : MathUtils.IScalarFunction, MathUtils.IScalarFunctionWithDerivative
    {
        public bool Value(double theX, out double theF)
        {
            theF = 2.0 * theX - 4.0;
            return true;
        }

        public bool Values(double theX, out double theF, out double theDf)
        {
            theF = 2.0 * theX - 4.0;
            theDf = 2.0;
            return true;
        }
    }
}

public class MathRoot_NewtonTest
{
    private const double THE_TOLERANCE = MathRootTestFunctions.THE_TOLERANCE;

    [Fact]
    public void SqrtTwo()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Newton(aFunc, 1.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(Math.Sqrt(2.0), aResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CosMinusX()
    {
        MathRootTestFunctions.CosMinusXFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Newton(aFunc, 0.5);
        Assert.True(aResult.IsDone());
        // Verify root satisfies equation
        double aFx = Math.Cos(aResult.Root!.Value) - aResult.Root.Value;
        Assert.Equal(0.0, aFx, THE_TOLERANCE);
    }

    [Fact]
    public void CubicEquation()
    {
        MathRootTestFunctions.CubicFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Newton(aFunc, 1.5);
        Assert.True(aResult.IsDone());
        double r = aResult.Root!.Value;
        double aFx = r * r * r - r - 2.0;
        Assert.Equal(0.0, aFx, THE_TOLERANCE);
    }

    [Fact]
    public void LinearFunction()
    {
        MathRootTestFunctions.LinearFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Newton(aFunc, 0.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(2.0, aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void ExpFunction()
    {
        MathRootTestFunctions.ExpMinusThreeFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Newton(aFunc, 1.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(Math.Log(3.0), aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CustomTolerance()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        MathUtils.Config aConfig = new();
        aConfig.XTolerance = 1.0e-14;
        aConfig.FTolerance = 1.0e-14;
        aConfig.MaxIterations = 100;

        MathUtils.ScalarResult aResult = MathRoot.Newton(aFunc, 1.0, aConfig);
        Assert.True(aResult.IsDone());
        Assert.Equal(Math.Sqrt(2.0), aResult.Root!.Value, 1.0e-14);
    }
}

public class MathRoot_NewtonBoundedTest
{
    private const double THE_TOLERANCE = MathRootTestFunctions.THE_TOLERANCE;

    [Fact]
    public void SqrtTwoWithBounds()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.NewtonBounded(aFunc, 1.5, 1.0, 2.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(Math.Sqrt(2.0), aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void SinWithBounds()
    {
        MathRootTestFunctions.SinFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.NewtonBounded(aFunc, 3.0, 2.0, 4.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(MathRootTestFunctions.THE_PI, aResult.Root!.Value, THE_TOLERANCE);
    }
}

public class MathRoot_SecantTest
{
    private const double THE_TOLERANCE = MathRootTestFunctions.THE_TOLERANCE;

    [Fact]
    public void SqrtTwo()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Secant(aFunc, 1.0, 2.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(Math.Sqrt(2.0), aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CosMinusX()
    {
        MathRootTestFunctions.CosMinusXFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Secant(aFunc, 0.0, 1.0);
        Assert.True(aResult.IsDone());
        double aFx = Math.Cos(aResult.Root!.Value) - aResult.Root.Value;
        Assert.Equal(0.0, aFx, THE_TOLERANCE);
    }

    [Fact]
    public void ExpFunction()
    {
        MathRootTestFunctions.ExpMinusThreeFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Secant(aFunc, 0.0, 2.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(Math.Log(3.0), aResult.Root!.Value, THE_TOLERANCE);
    }
}

public class MathRoot_BrentTest
{
    private const double THE_TOLERANCE = MathRootTestFunctions.THE_TOLERANCE;

    [Fact]
    public void SqrtTwo()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Brent(aFunc, 1.0, 2.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(Math.Sqrt(2.0), aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CosMinusX()
    {
        MathRootTestFunctions.CosMinusXFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Brent(aFunc, 0.0, 1.0);
        Assert.True(aResult.IsDone());
        double aFx = Math.Cos(aResult.Root!.Value) - aResult.Root.Value;
        Assert.Equal(0.0, aFx, THE_TOLERANCE);
    }

    [Fact]
    public void CubicEquation()
    {
        MathRootTestFunctions.CubicFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Brent(aFunc, 1.0, 2.0);
        Assert.True(aResult.IsDone());
        double r = aResult.Root!.Value;
        double aFx = r * r * r - r - 2.0;
        Assert.Equal(0.0, aFx, THE_TOLERANCE);
    }

    [Fact]
    public void SinPi()
    {
        MathRootTestFunctions.SinFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Brent(aFunc, 2.0, 4.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(MathRootTestFunctions.THE_PI, aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void ExpFunction()
    {
        MathRootTestFunctions.ExpMinusThreeFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Brent(aFunc, 0.0, 2.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(Math.Log(3.0), aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void InvalidBracket()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        // Both endpoints positive - no sign change
        MathUtils.ScalarResult aResult = MathRoot.Brent(aFunc, 2.0, 3.0);
        Assert.Equal(MathUtils.Status.InvalidInput, aResult.Status);
    }

    [Fact]
    public void ReversedBracket()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        // Reversed bracket (upper < lower)
        MathUtils.ScalarResult aResult = MathRoot.Brent(aFunc, 2.0, 1.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(Math.Sqrt(2.0), aResult.Root!.Value, THE_TOLERANCE);
    }
}

public class MathRoot_BisectionTest
{
    private const double THE_TOLERANCE = MathRootTestFunctions.THE_TOLERANCE;

    [Fact]
    public void SqrtTwo()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Bisection(aFunc, 1.0, 2.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(Math.Sqrt(2.0), aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CosMinusX()
    {
        MathRootTestFunctions.CosMinusXFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Bisection(aFunc, 0.0, 1.0);
        Assert.True(aResult.IsDone());
        double aFx = Math.Cos(aResult.Root!.Value) - aResult.Root.Value;
        Assert.Equal(0.0, aFx, THE_TOLERANCE);
    }

    [Fact]
    public void SinPi()
    {
        MathRootTestFunctions.SinFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Bisection(aFunc, 2.0, 4.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(MathRootTestFunctions.THE_PI, aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void InvalidBracket()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        // No sign change
        MathUtils.ScalarResult aResult = MathRoot.Bisection(aFunc, 2.0, 3.0);
        Assert.Equal(MathUtils.Status.InvalidInput, aResult.Status);
    }
}

public class MathRoot_BisectionNewtonTest
{
    private const double THE_TOLERANCE = MathRootTestFunctions.THE_TOLERANCE;

    [Fact]
    public void SqrtTwo()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.BisectionNewton(aFunc, 1.0, 2.0);
        Assert.True(aResult.IsDone());
        Assert.Equal(Math.Sqrt(2.0), aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CosMinusX()
    {
        MathRootTestFunctions.CosMinusXFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.BisectionNewton(aFunc, 0.0, 1.0);
        Assert.True(aResult.IsDone());
        double aFx = Math.Cos(aResult.Root!.Value) - aResult.Root.Value;
        Assert.Equal(0.0, aFx, THE_TOLERANCE);
    }

    [Fact]
    public void FasterThanPureBisection()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        MathUtils.Config aConfig = new();
        aConfig.MaxIterations = 100;

        MathUtils.ScalarResult aBisec = MathRoot.Bisection(aFunc, 1.0, 2.0, aConfig);
        MathUtils.ScalarResult aHybrid = MathRoot.BisectionNewton(aFunc, 1.0, 2.0, aConfig);

        Assert.True(aBisec.IsDone());
        Assert.True(aHybrid.IsDone());

        // Hybrid should converge in fewer iterations
        Assert.True(aHybrid.NbIterations <= aBisec.NbIterations);
    }
}

public class MathRoot_ConvergenceTest
{
    [Fact]
    public void NewtonIterationCount()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Newton(aFunc, 1.0);
        Assert.True(aResult.IsDone());
        // Newton should converge quickly (typically < 10 iterations)
        Assert.True(aResult.NbIterations < 15);
    }

    [Fact]
    public void BrentIterationCount()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Brent(aFunc, 1.0, 2.0);
        Assert.True(aResult.IsDone());
        // Brent should converge reasonably fast
        Assert.True(aResult.NbIterations < 50);
    }
}

public class MathRoot_BoolConversionTest
{
    [Fact]
    public void SuccessfulResultIsTrue()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Newton(aFunc, 1.0);
        Assert.True((bool)aResult);
    }

    [Fact]
    public void InvalidInputIsFalse()
    {
        MathRootTestFunctions.SqrtTwoFunc aFunc = new();
        MathUtils.ScalarResult aResult = MathRoot.Brent(aFunc, 2.0, 3.0);
        Assert.False((bool)aResult);
    }
}
