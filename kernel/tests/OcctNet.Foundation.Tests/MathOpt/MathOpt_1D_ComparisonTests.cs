// Port of: src/FoundationClasses/TKMath/GTests/MathOpt_1D_Comparison_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace OcctNet.Foundation.Tests;

// The suite MathOpt_1D_ComparisonTest is also used by MathOpt_1D_Test.cxx, hence partial.
public partial class MathOpt_1D_ComparisonTest
{
    private const double THE_TOLERANCE = 1.0e-7;
    private const double THE_PI = 3.14159265358979323846;

    // Adapter classes for old API (inherit from math_Function)

    // f(x) = (x - 3)^2 + 1, minimum at x = 3, f(3) = 1
    private sealed class ParabolaFuncOld : math_Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = (theX - 3.0) * (theX - 3.0) + 1.0;
            return true;
        }
    }

    // f(x) = x^2 - 4x + 3, minimum at x = 2, f(2) = -1
    private sealed class ParabolaFunc2Old : math_Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = theX * theX - 4.0 * theX + 3.0;
            return true;
        }
    }

    // f(x) = cos(x), local minimum at x = PI, f(PI) = -1
    private sealed class CosFuncOld : math_Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Cos(theX);
            return true;
        }
    }

    // f(x) = x^4 - 2x^2, local minima at x = -1 and x = 1
    private sealed class QuarticFuncOld : math_Function
    {
        public override bool Value(double theX, out double theF)
        {
            double aX2 = theX * theX;
            theF = aX2 * aX2 - 2.0 * aX2;
            return true;
        }
    }

    // f(x) = e^x + e^(-x), minimum at x = 0, f(0) = 2
    private sealed class CoshLikeFuncOld : math_Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = Math.Exp(theX) + Math.Exp(-theX);
            return true;
        }
    }

    // f(x) = x^2, minimum at x = 0, f(0) = 0
    private sealed class SquareFuncOld : math_Function
    {
        public override bool Value(double theX, out double theF)
        {
            theF = theX * theX;
            return true;
        }
    }

    // Function structs for new API

    private struct ParabolaFuncNew : IScalarFunction
    {
        public readonly bool Value(double theX, out double theF)
        {
            theF = (theX - 3.0) * (theX - 3.0) + 1.0;
            return true;
        }
    }

    private struct ParabolaFunc2New : IScalarFunction
    {
        public readonly bool Value(double theX, out double theF)
        {
            theF = theX * theX - 4.0 * theX + 3.0;
            return true;
        }
    }

    private struct CosFuncNew : IScalarFunction
    {
        public readonly bool Value(double theX, out double theF)
        {
            theF = Math.Cos(theX);
            return true;
        }
    }

    private struct QuarticFuncNew : IScalarFunction
    {
        public readonly bool Value(double theX, out double theF)
        {
            double aX2 = theX * theX;
            theF = aX2 * aX2 - 2.0 * aX2;
            return true;
        }
    }

    private struct CoshLikeFuncNew : IScalarFunction
    {
        public readonly bool Value(double theX, out double theF)
        {
            theF = Math.Exp(theX) + Math.Exp(-theX);
            return true;
        }
    }

    private struct SquareFuncNew : IScalarFunction
    {
        public readonly bool Value(double theX, out double theF)
        {
            theF = theX * theX;
            return true;
        }
    }

    // math_BrentMinimum vs MathOpt::Brent comparison tests

    [Fact]
    public void Brent_Parabola()
    {
        ParabolaFuncOld anOldFunc = new();
        ParabolaFuncNew aNewFunc = new();

        // Old API requires a bracketing triplet (Ax, Bx, Cx) where f(Bx) < f(Ax) and f(Bx) < f(Cx)
        // For f(x) = (x-3)^2 + 1, minimum at x=3
        // Use Ax=0, Bx=3, Cx=6 as the bracket
        math_BrentMinimum anOldSolver = new(THE_TOLERANCE, 100);
        anOldSolver.Perform(anOldFunc, 0.0, 3.0, 6.0);

        // New API takes interval bounds
        ScalarResult aNewResult = MathOpt.Brent(aNewFunc, 0.0, 6.0);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Location(), THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);
        Assert.Equal(3.0, aNewResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(1.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Brent_ParabolaNegativeMinimum()
    {
        ParabolaFunc2Old anOldFunc = new();
        ParabolaFunc2New aNewFunc = new();

        // f(x) = x^2 - 4x + 3, minimum at x=2, f(2)=-1
        math_BrentMinimum anOldSolver = new(THE_TOLERANCE, 100);
        anOldSolver.Perform(anOldFunc, 0.0, 2.0, 5.0);

        ScalarResult aNewResult = MathOpt.Brent(aNewFunc, 0.0, 5.0);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Location(), THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);
        Assert.Equal(2.0, aNewResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(-1.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Brent_Cosine()
    {
        CosFuncOld anOldFunc = new();
        CosFuncNew aNewFunc = new();

        // cos(x) minimum at x=PI, f(PI)=-1
        math_BrentMinimum anOldSolver = new(THE_TOLERANCE, 100);
        anOldSolver.Perform(anOldFunc, 2.0, THE_PI, 4.0);

        ScalarResult aNewResult = MathOpt.Brent(aNewFunc, 2.0, 4.0);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Location(), THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);
        Assert.Equal(THE_PI, aNewResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(-1.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Brent_QuarticRight()
    {
        QuarticFuncOld anOldFunc = new();
        QuarticFuncNew aNewFunc = new();

        // x^4 - 2x^2, right minimum at x=1
        math_BrentMinimum anOldSolver = new(THE_TOLERANCE, 100);
        anOldSolver.Perform(anOldFunc, 0.0, 1.0, 2.0);

        ScalarResult aNewResult = MathOpt.Brent(aNewFunc, 0.0, 2.0);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Location(), THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);
        Assert.Equal(1.0, aNewResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(-1.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Brent_QuarticLeft()
    {
        QuarticFuncOld anOldFunc = new();
        QuarticFuncNew aNewFunc = new();

        // x^4 - 2x^2, left minimum at x=-1
        math_BrentMinimum anOldSolver = new(THE_TOLERANCE, 100);
        anOldSolver.Perform(anOldFunc, -2.0, -1.0, 0.0);

        ScalarResult aNewResult = MathOpt.Brent(aNewFunc, -2.0, 0.0);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Location(), THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);
        Assert.Equal(-1.0, aNewResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(-1.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Brent_CoshLike()
    {
        CoshLikeFuncOld anOldFunc = new();
        CoshLikeFuncNew aNewFunc = new();

        // e^x + e^(-x), minimum at x=0, f(0)=2
        math_BrentMinimum anOldSolver = new(THE_TOLERANCE, 100);
        anOldSolver.Perform(anOldFunc, -5.0, 0.0, 5.0);

        ScalarResult aNewResult = MathOpt.Brent(aNewFunc, -5.0, 5.0);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Location(), THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);
        Assert.Equal(0.0, aNewResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(2.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Brent_Square()
    {
        SquareFuncOld anOldFunc = new();
        SquareFuncNew aNewFunc = new();

        // x^2, minimum at x=0, f(0)=0
        math_BrentMinimum anOldSolver = new(THE_TOLERANCE, 100);
        anOldSolver.Perform(anOldFunc, -10.0, 0.0, 10.0);

        ScalarResult aNewResult = MathOpt.Brent(aNewFunc, -10.0, 10.0);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Location(), THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);
        Assert.Equal(0.0, aNewResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(0.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    // Iteration count comparison tests

    [Fact]
    public void IterationCount_BrentVsGolden()
    {
        ParabolaFuncNew aFunc = new();

        Config aConfig = new();
        aConfig.MaxIterations = 200;

        ScalarResult aBrentResult = MathOpt.Brent(aFunc, 0.0, 10.0, aConfig);
        ScalarResult aGoldenResult = MathOpt.Golden(aFunc, 0.0, 10.0, aConfig);

        Assert.True(aBrentResult.IsDone());
        Assert.True(aGoldenResult.IsDone());

        // Both should find the same minimum
        Assert.Equal(aGoldenResult.Root!.Value, aBrentResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(aGoldenResult.Value!.Value, aBrentResult.Value!.Value, THE_TOLERANCE);

        // Brent typically converges faster due to parabolic interpolation
        Assert.True(aBrentResult.NbIterations <= aGoldenResult.NbIterations);
    }

    // Accuracy comparison tests

    [Fact]
    public void Accuracy_BrentVsOld()
    {
        ParabolaFuncOld anOldFunc = new();
        ParabolaFuncNew aNewFunc = new();

        // Test with tighter tolerance
        const double aTightTol = 1.0e-12;

        math_BrentMinimum anOldSolver = new(aTightTol, 200);
        anOldSolver.Perform(anOldFunc, 0.0, 3.0, 10.0);

        Config aConfig = new();
        aConfig.XTolerance = aTightTol;
        aConfig.MaxIterations = 200;
        ScalarResult aNewResult = MathOpt.Brent(aNewFunc, 0.0, 10.0, aConfig);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        // Both should achieve good accuracy
        Assert.Equal(3.0, anOldSolver.Location(), 1.0e-8);
        Assert.Equal(3.0, aNewResult.Root!.Value, 1.0e-8);
    }

    // Wide interval comparison tests

    [Fact]
    public void WideInterval_Parabola()
    {
        ParabolaFuncOld anOldFunc = new();
        ParabolaFuncNew aNewFunc = new();

        // Very wide interval
        math_BrentMinimum anOldSolver = new(THE_TOLERANCE, 200);
        anOldSolver.Perform(anOldFunc, -100.0, 3.0, 100.0);

        ScalarResult aNewResult = MathOpt.Brent(aNewFunc, -100.0, 100.0);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Location(), THE_TOLERANCE);
        Assert.Equal(3.0, aNewResult.Root!.Value, THE_TOLERANCE);
    }

    // Narrow interval comparison tests

    [Fact]
    public void NarrowInterval_Parabola()
    {
        ParabolaFuncOld anOldFunc = new();
        ParabolaFuncNew aNewFunc = new();

        // Narrow interval around minimum
        math_BrentMinimum anOldSolver = new(THE_TOLERANCE, 100);
        anOldSolver.Perform(anOldFunc, 2.9, 3.0, 3.1);

        ScalarResult aNewResult = MathOpt.Brent(aNewFunc, 2.9, 3.1);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        Assert.Equal(aNewResult.Root!.Value, anOldSolver.Location(), THE_TOLERANCE);
        Assert.Equal(3.0, aNewResult.Root!.Value, THE_TOLERANCE);
    }

    // MathOpt::Golden comparison tests

    [Fact]
    public void Golden_Parabola()
    {
        ParabolaFuncNew aNewFunc = new();

        // Golden section should find the same minimum as Brent
        ScalarResult aBrentResult = MathOpt.Brent(aNewFunc, 0.0, 10.0);
        ScalarResult aGoldenResult = MathOpt.Golden(aNewFunc, 0.0, 10.0);

        Assert.True(aBrentResult.IsDone());
        Assert.True(aGoldenResult.IsDone());

        Assert.Equal(aGoldenResult.Root!.Value, aBrentResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(aGoldenResult.Value!.Value, aBrentResult.Value!.Value, THE_TOLERANCE);
        Assert.Equal(3.0, aGoldenResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Golden_CoshLike()
    {
        CoshLikeFuncNew aNewFunc = new();

        ScalarResult aBrentResult = MathOpt.Brent(aNewFunc, -5.0, 5.0);
        ScalarResult aGoldenResult = MathOpt.Golden(aNewFunc, -5.0, 5.0);

        Assert.True(aBrentResult.IsDone());
        Assert.True(aGoldenResult.IsDone());

        Assert.Equal(aGoldenResult.Root!.Value, aBrentResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(aGoldenResult.Value!.Value, aBrentResult.Value!.Value, THE_TOLERANCE);
        Assert.Equal(0.0, aGoldenResult.Root!.Value, THE_TOLERANCE);
    }
}
