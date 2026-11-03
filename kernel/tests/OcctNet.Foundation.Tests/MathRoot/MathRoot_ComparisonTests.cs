// Port of: src/FoundationClasses/TKMath/GTests/MathRoot_Comparison_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace OcctNet.Foundation.Tests;

public class MathRoot_ComparisonTest
{
    private const double THE_TOLERANCE = 1.0e-9;
    private const double THE_PI = 3.14159265358979323846;

    // ============================================================================
    // Adapter classes for old API (inherit from math_FunctionWithDerivative)
    // ============================================================================

    // f(x) = x^2 - 2, f'(x) = 2x
    private sealed class SqrtTwoFuncOld : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = theX * theX - 2.0;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = 2.0 * theX;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = theX * theX - 2.0;
            theD = 2.0 * theX;
            return true;
        }
    }

    // f(x) = cos(x) - x, f'(x) = -sin(x) - 1
    private sealed class CosMinusXFuncOld : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Cos(theX) - theX;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = -Math.Sin(theX) - 1.0;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = Math.Cos(theX) - theX;
            theD = -Math.Sin(theX) - 1.0;
            return true;
        }
    }

    // f(x) = sin(x), f'(x) = cos(x)
    private sealed class SinFuncOld : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Sin(theX);
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = Math.Cos(theX);
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = Math.Sin(theX);
            theD = Math.Cos(theX);
            return true;
        }
    }

    // f(x) = x^3 - x - 2, f'(x) = 3x^2 - 1
    private sealed class CubicFuncOld : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = theX * theX * theX - theX - 2.0;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = 3.0 * theX * theX - 1.0;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = theX * theX * theX - theX - 2.0;
            theD = 3.0 * theX * theX - 1.0;
            return true;
        }
    }

    // f(x) = e^x - 3, f'(x) = e^x
    private sealed class ExpMinusThreeFuncOld : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Exp(theX) - 3.0;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = Math.Exp(theX);
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            theF = Math.Exp(theX) - 3.0;
            theD = Math.Exp(theX);
            return true;
        }
    }

    // ============================================================================
    // Function structs for new API (simple structs with Value/Values methods)
    // ============================================================================

    private readonly struct SqrtTwoFuncNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction, global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
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

    private readonly struct CosMinusXFuncNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction, global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
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

    private readonly struct SinFuncNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction, global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
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

    private readonly struct CubicFuncNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction, global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
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

    private readonly struct ExpMinusThreeFuncNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction, global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
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

    // ============================================================================
    // math_BissecNewton vs MathRoot::BisectionNewton comparison tests
    // ============================================================================

    [Fact]
    public void BisectionNewton_SqrtTwo()
    {
        SqrtTwoFuncOld anOldFunc = new();
        SqrtTwoFuncNew aNewFunc = new();

        BissecNewton anOldSolver = new(THE_TOLERANCE);
        anOldSolver.Perform(anOldFunc, 1.0, 2.0, 100);

        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.BisectionNewton(aNewFunc, 1.0, 2.0);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Root(), THE_TOLERANCE);
        Assert.Equal(Math.Sqrt(2.0), anOldSolver.Root(), THE_TOLERANCE);
    }

    [Fact]
    public void BisectionNewton_CosMinusX()
    {
        CosMinusXFuncOld anOldFunc = new();
        CosMinusXFuncNew aNewFunc = new();

        BissecNewton anOldSolver = new(THE_TOLERANCE);
        anOldSolver.Perform(anOldFunc, 0.0, 1.0, 100);

        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.BisectionNewton(aNewFunc, 0.0, 1.0);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Verify both find the same root
        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Root(), THE_TOLERANCE);

        // Verify the root is correct
        double aFx = Math.Cos(aNewResult.Root.Value) - aNewResult.Root.Value;
        Assert.Equal(0.0, aFx, THE_TOLERANCE);
    }

    [Fact]
    public void BisectionNewton_SinPi()
    {
        SinFuncOld anOldFunc = new();
        SinFuncNew aNewFunc = new();

        BissecNewton anOldSolver = new(THE_TOLERANCE);
        anOldSolver.Perform(anOldFunc, 2.0, 4.0, 100);

        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.BisectionNewton(aNewFunc, 2.0, 4.0);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Root(), THE_TOLERANCE);
        Assert.Equal(THE_PI, aNewResult.Root.Value, THE_TOLERANCE);
    }

    [Fact]
    public void BisectionNewton_Cubic()
    {
        CubicFuncOld anOldFunc = new();
        CubicFuncNew aNewFunc = new();

        BissecNewton anOldSolver = new(THE_TOLERANCE);
        anOldSolver.Perform(anOldFunc, 1.0, 2.0, 100);

        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.BisectionNewton(aNewFunc, 1.0, 2.0);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Root(), THE_TOLERANCE);
    }

    // ============================================================================
    // math_NewtonFunctionRoot vs MathRoot::Newton comparison tests
    // ============================================================================

    [Fact]
    public void Newton_SqrtTwo()
    {
        SqrtTwoFuncOld anOldFunc = new();
        SqrtTwoFuncNew aNewFunc = new();

        NewtonFunctionRoot anOldSolver = new(anOldFunc, 1.5, THE_TOLERANCE, THE_TOLERANCE, 100);

        global::Kerf.Engine.Numerics.Utilities.MathUtils.Config aConfig = new();
        aConfig.XTolerance = THE_TOLERANCE;
        aConfig.FTolerance = THE_TOLERANCE;
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Newton(aNewFunc, 1.5, aConfig);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Root(), THE_TOLERANCE);
        Assert.Equal(Math.Sqrt(2.0), aNewResult.Root.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Newton_ExpMinusThree()
    {
        ExpMinusThreeFuncOld anOldFunc = new();
        ExpMinusThreeFuncNew aNewFunc = new();

        NewtonFunctionRoot anOldSolver = new(anOldFunc, 1.0, THE_TOLERANCE, THE_TOLERANCE, 100);

        global::Kerf.Engine.Numerics.Utilities.MathUtils.Config aConfig = new();
        aConfig.XTolerance = THE_TOLERANCE;
        aConfig.FTolerance = THE_TOLERANCE;
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Newton(aNewFunc, 1.0, aConfig);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Root(), THE_TOLERANCE);
        Assert.Equal(Math.Log(3.0), aNewResult.Root.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Newton_CosMinusX()
    {
        CosMinusXFuncOld anOldFunc = new();
        CosMinusXFuncNew aNewFunc = new();

        NewtonFunctionRoot anOldSolver = new(anOldFunc, 0.5, THE_TOLERANCE, THE_TOLERANCE, 100);

        global::Kerf.Engine.Numerics.Utilities.MathUtils.Config aConfig = new();
        aConfig.XTolerance = THE_TOLERANCE;
        aConfig.FTolerance = THE_TOLERANCE;
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Newton(aNewFunc, 0.5, aConfig);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Root(), THE_TOLERANCE);
    }

    // ============================================================================
    // math_BracketedRoot vs MathRoot::Brent comparison tests
    // ============================================================================

    [Fact]
    public void Brent_SqrtTwo()
    {
        SqrtTwoFuncOld anOldFunc = new();
        SqrtTwoFuncNew aNewFunc = new();

        BracketedRoot anOldSolver = new(anOldFunc, 1.0, 2.0, THE_TOLERANCE, 100);

        global::Kerf.Engine.Numerics.Utilities.MathUtils.Config aConfig = new();
        aConfig.XTolerance = THE_TOLERANCE;
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Brent(aNewFunc, 1.0, 2.0, aConfig);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Root(), THE_TOLERANCE);
        Assert.Equal(Math.Sqrt(2.0), aNewResult.Root.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Brent_CosMinusX()
    {
        CosMinusXFuncOld anOldFunc = new();
        CosMinusXFuncNew aNewFunc = new();

        BracketedRoot anOldSolver = new(anOldFunc, 0.0, 1.0, THE_TOLERANCE, 100);

        global::Kerf.Engine.Numerics.Utilities.MathUtils.Config aConfig = new();
        aConfig.XTolerance = THE_TOLERANCE;
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Brent(aNewFunc, 0.0, 1.0, aConfig);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Root(), THE_TOLERANCE);
    }

    [Fact]
    public void Brent_SinPi()
    {
        SinFuncOld anOldFunc = new();
        SinFuncNew aNewFunc = new();

        BracketedRoot anOldSolver = new(anOldFunc, 2.0, 4.0, THE_TOLERANCE, 100);

        global::Kerf.Engine.Numerics.Utilities.MathUtils.Config aConfig = new();
        aConfig.XTolerance = THE_TOLERANCE;
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Brent(aNewFunc, 2.0, 4.0, aConfig);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Root(), THE_TOLERANCE);
        Assert.Equal(THE_PI, aNewResult.Root.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Brent_ExpMinusThree()
    {
        ExpMinusThreeFuncOld anOldFunc = new();
        ExpMinusThreeFuncNew aNewFunc = new();

        BracketedRoot anOldSolver = new(anOldFunc, 0.0, 2.0, THE_TOLERANCE, 100);

        global::Kerf.Engine.Numerics.Utilities.MathUtils.Config aConfig = new();
        aConfig.XTolerance = THE_TOLERANCE;
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Brent(aNewFunc, 0.0, 2.0, aConfig);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Root(), THE_TOLERANCE);
        Assert.Equal(Math.Log(3.0), aNewResult.Root.Value, THE_TOLERANCE);
    }

    // ============================================================================
    // math_FunctionRoot vs MathRoot::NewtonBounded comparison tests
    // ============================================================================

    [Fact]
    public void FunctionRoot_SqrtTwo()
    {
        SqrtTwoFuncOld anOldFunc = new();
        SqrtTwoFuncNew aNewFunc = new();

        FunctionRoot anOldSolver = new(anOldFunc, 1.5, THE_TOLERANCE, 1.0, 2.0, 100);

        global::Kerf.Engine.Numerics.Utilities.MathUtils.Config aConfig = new();
        aConfig.XTolerance = THE_TOLERANCE;
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.NewtonBounded(aNewFunc, 1.5, 1.0, 2.0, aConfig);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Root(), THE_TOLERANCE);
        Assert.Equal(Math.Sqrt(2.0), aNewResult.Root.Value, THE_TOLERANCE);
    }

    [Fact]
    public void FunctionRoot_SinPi()
    {
        SinFuncOld anOldFunc = new();
        SinFuncNew aNewFunc = new();

        FunctionRoot anOldSolver = new(anOldFunc, 3.0, THE_TOLERANCE, 2.0, 4.0, 100);

        global::Kerf.Engine.Numerics.Utilities.MathUtils.Config aConfig = new();
        aConfig.XTolerance = THE_TOLERANCE;
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.NewtonBounded(aNewFunc, 3.0, 2.0, 4.0, aConfig);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Root(), THE_TOLERANCE);
        Assert.Equal(THE_PI, aNewResult.Root.Value, THE_TOLERANCE);
    }

    // ============================================================================
    // Iteration count comparison tests
    // ============================================================================

    [Fact]
    public void IterationCount_NewtonVsBrent()
    {
        SqrtTwoFuncNew aFunc = new();

        global::Kerf.Engine.Numerics.Utilities.MathUtils.Config aConfig = new();
        aConfig.MaxIterations = 100;

        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewtonResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Newton(aFunc, 1.5, aConfig);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aBrentResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Brent(aFunc, 1.0, 2.0, aConfig);

        Assert.True(aNewtonResult.IsDone);
        Assert.True(aBrentResult.IsDone);

        // Newton should typically converge faster for smooth functions
        Assert.True(aNewtonResult.NbIterations <= aBrentResult.NbIterations);

        // Both should find the same root
        Assert.Equal(aBrentResult.Root!.Value, aNewtonResult.Root!.Value, THE_TOLERANCE);
    }

    // ============================================================================
    // Robustness comparison - challenging cases
    // ============================================================================

    // f(x) = x^3 at x=0 has derivative near zero
    private sealed class CubicNearZeroOld : FunctionWithDerivative
    {
        public override bool Value(double theX, out double theF)
        {
            theF = theX * theX * theX - 0.001;
            return true;
        }

        public override bool Derivative(double theX, out double theD)
        {
            theD = 3.0 * theX * theX;
            return true;
        }

        public override bool Values(double theX, out double theF, out double theD)
        {
            Value(theX, out theF);
            Derivative(theX, out theD);
            return true;
        }
    }

    private readonly struct CubicNearZeroNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction, global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
    {
        public bool Value(double theX, out double theF)
        {
            theF = theX * theX * theX - 0.001;
            return true;
        }

        public bool Values(double theX, out double theF, out double theDf)
        {
            theF = theX * theX * theX - 0.001;
            theDf = 3.0 * theX * theX;
            return true;
        }
    }

    [Fact]
    public void ChallengingCase_CloseToZeroDerivative()
    {
        CubicNearZeroOld anOldFunc = new();
        CubicNearZeroNew aNewFunc = new();

        // Old API - Brent (more robust for this case)
        BracketedRoot anOldSolver = new(anOldFunc, 0.0, 1.0, THE_TOLERANCE, 100);

        // New API - Brent
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult aNewResult = global::Kerf.Engine.Numerics.Roots.MathRoot.Brent(aNewFunc, 0.0, 1.0);

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Root should be at 0.1 (cube root of 0.001)
        Assert.Equal(0.1, anOldSolver.Root(), 1.0e-6);
        Assert.Equal(0.1, aNewResult.Root!.Value, 1.0e-6);
        Assert.Equal(aNewResult.Root.Value, anOldSolver.Root(), THE_TOLERANCE);
    }
}
