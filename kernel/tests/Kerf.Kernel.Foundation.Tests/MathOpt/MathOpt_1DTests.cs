// Port of: src/FoundationClasses/TKMath/GTests/MathOpt_1D_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Kernel.Foundation.Tests;

internal static class MathOpt_1D_TestFunctions
{
    public const double THE_TOLERANCE = 1.0e-7;
    public const double THE_PI = 3.14159265358979323846;

    // Parabola: f(x) = (x - 3)^2 + 1, minimum at x = 3, f(3) = 1
    public sealed class ParabolaFunc : IScalarFunction
    {
        public bool Value(double theX, out double theF)
        {
            theF = (theX - 3.0) * (theX - 3.0) + 1.0;
            return true;
        }
    }

    // Parabola with negative minimum: f(x) = x^2 - 4x + 3, minimum at x = 2, f(2) = -1
    public sealed class ParabolaFunc2 : IScalarFunction
    {
        public bool Value(double theX, out double theF)
        {
            theF = theX * theX - 4.0 * theX + 3.0;
            return true;
        }
    }

    // Cosine function: f(x) = cos(x), local minimum at x = PI
    public sealed class CosFunc : IScalarFunction
    {
        public bool Value(double theX, out double theF)
        {
            theF = Math.Cos(theX);
            return true;
        }
    }

    // Quartic: f(x) = x^4 - 2x^2, local minima at x = -1 and x = 1
    public sealed class QuarticFunc : IScalarFunction
    {
        public bool Value(double theX, out double theF)
        {
            double aX2 = theX * theX;
            theF = aX2 * aX2 - 2.0 * aX2;
            return true;
        }
    }

    // Exponential: f(x) = e^x + e^(-x), minimum at x = 0, f(0) = 2
    public sealed class CoshLikeFunc : IScalarFunction
    {
        public bool Value(double theX, out double theF)
        {
            theF = Math.Exp(theX) + Math.Exp(-theX);
            return true;
        }
    }

    // Absolute value shifted: f(x) = |x - 2| + 1, minimum at x = 2 (non-smooth)
    public sealed class AbsShiftedFunc : IScalarFunction
    {
        public bool Value(double theX, out double theF)
        {
            theF = Math.Abs(theX - 2.0) + 1.0;
            return true;
        }
    }
}

public class MathOpt_1D_BrentTest
{
    private const double THE_TOLERANCE = MathOpt_1D_TestFunctions.THE_TOLERANCE;

    [Fact]
    public void SimpleParabola()
    {
        MathOpt_1D_TestFunctions.ParabolaFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 0.0, 10.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(3.0, aResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(1.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void ParabolaWithNegativeMinimum()
    {
        MathOpt_1D_TestFunctions.ParabolaFunc2 aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 0.0, 5.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(2.0, aResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(-1.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CosineMinimum()
    {
        MathOpt_1D_TestFunctions.CosFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 2.0, 4.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(MathOpt_1D_TestFunctions.THE_PI, aResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(-1.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void QuarticMinimumRight()
    {
        MathOpt_1D_TestFunctions.QuarticFunc aFunc = new();
        // Search for right minimum at x = 1
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 0.0, 2.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(-1.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void QuarticMinimumLeft()
    {
        MathOpt_1D_TestFunctions.QuarticFunc aFunc = new();
        // Search for left minimum at x = -1
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, -2.0, 0.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(-1.0, aResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(-1.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CoshLikeMinimum()
    {
        MathOpt_1D_TestFunctions.CoshLikeFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, -5.0, 5.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void NarrowInterval()
    {
        MathOpt_1D_TestFunctions.ParabolaFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 2.9, 3.1);
        Assert.True(aResult.IsDone);
        Assert.Equal(3.0, aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void WideInterval()
    {
        MathOpt_1D_TestFunctions.ParabolaFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, -100.0, 100.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(3.0, aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CustomTolerance()
    {
        MathOpt_1D_TestFunctions.ParabolaFunc aFunc = new();
        Config aConfig = new();
        aConfig.XTolerance = 1.0e-10;
        aConfig.MaxIterations = 200;

        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 0.0, 10.0, aConfig);
        Assert.True(aResult.IsDone);
        Assert.Equal(3.0, aResult.Root!.Value, 1.0e-9);
    }

    [Fact]
    public void NonSmoothFunction()
    {
        MathOpt_1D_TestFunctions.AbsShiftedFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 0.0, 5.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(2.0, aResult.Root!.Value, 1.0e-6); // Lower tolerance for non-smooth
        Assert.Equal(1.0, aResult.Value!.Value, 1.0e-6);
    }
}

public class MathOpt_1D_GoldenTest
{
    private const double THE_TOLERANCE = MathOpt_1D_TestFunctions.THE_TOLERANCE;

    [Fact]
    public void SimpleParabola()
    {
        MathOpt_1D_TestFunctions.ParabolaFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Golden(aFunc, 0.0, 10.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(3.0, aResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(1.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CosineMinimum()
    {
        MathOpt_1D_TestFunctions.CosFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Golden(aFunc, 2.0, 4.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(MathOpt_1D_TestFunctions.THE_PI, aResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(-1.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void QuarticMinimum()
    {
        MathOpt_1D_TestFunctions.QuarticFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Golden(aFunc, 0.0, 2.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(-1.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CoshLikeMinimum()
    {
        MathOpt_1D_TestFunctions.CoshLikeFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Golden(aFunc, -5.0, 5.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Value!.Value, THE_TOLERANCE);
    }
}

public class MathOpt_1D_BrentWithBracketTest
{
    private const double THE_TOLERANCE = MathOpt_1D_TestFunctions.THE_TOLERANCE;

    [Fact]
    public void SimpleParabola()
    {
        MathOpt_1D_TestFunctions.ParabolaFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BrentWithBracket(aFunc, 0.0, 1.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(3.0, aResult.Root!.Value, THE_TOLERANCE);
        Assert.Equal(1.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void StartNearMinimum()
    {
        MathOpt_1D_TestFunctions.ParabolaFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BrentWithBracket(aFunc, 2.5, 0.5);
        Assert.True(aResult.IsDone);
        Assert.Equal(3.0, aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CoshLike()
    {
        MathOpt_1D_TestFunctions.CoshLikeFunc aFunc = new();
        // Start from a point closer to the minimum with a small step
        // The algorithm expands to find a bracket containing the minimum
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BrentWithBracket(aFunc, 0.1, 0.2);
        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Root!.Value, THE_TOLERANCE);
    }
}

// The suite MathOpt_1D_ComparisonTest is also used by MathOpt_1D_Comparison_Test.cxx, hence partial.
public partial class MathOpt_1D_ComparisonTest
{
    [Fact]
    public void BrentVsGoldenSameResult()
    {
        MathOpt_1D_TestFunctions.ParabolaFunc aFunc = new();

        ScalarResult aBrent = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 0.0, 10.0);
        ScalarResult aGolden = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Golden(aFunc, 0.0, 10.0);

        Assert.True(aBrent.IsDone);
        Assert.True(aGolden.IsDone);

        Assert.Equal(aGolden.Root!.Value, aBrent.Root!.Value, MathOpt_1D_TestFunctions.THE_TOLERANCE);
        Assert.Equal(aGolden.Value!.Value, aBrent.Value!.Value, MathOpt_1D_TestFunctions.THE_TOLERANCE);
    }

    [Fact]
    public void BrentFasterThanGolden()
    {
        MathOpt_1D_TestFunctions.ParabolaFunc aFunc = new();
        Config aConfig = new();
        aConfig.MaxIterations = 200;

        ScalarResult aBrent = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 0.0, 10.0, aConfig);
        ScalarResult aGolden = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Golden(aFunc, 0.0, 10.0, aConfig);

        Assert.True(aBrent.IsDone);
        Assert.True(aGolden.IsDone);

        // Brent typically converges faster due to parabolic interpolation
        Assert.True(aBrent.NbIterations <= aGolden.NbIterations);
    }
}

public class MathOpt_1D_RobustnessTest
{
    private const double THE_TOLERANCE = MathOpt_1D_TestFunctions.THE_TOLERANCE;

    private sealed class SquareFunc : IScalarFunction
    {
        public bool Value(double theX, out double theF)
        {
            theF = theX * theX;
            return true;
        }
    }

    private sealed class ShiftedSquareFunc : IScalarFunction
    {
        public bool Value(double theX, out double theF)
        {
            theF = (10.0 - theX) * (10.0 - theX);
            return true;
        }
    }

    private sealed class FlatQuarticFunc : IScalarFunction
    {
        public bool Value(double theX, out double theF)
        {
            double aDiff = theX - 5.0;
            theF = aDiff * aDiff * aDiff * aDiff;
            return true;
        }
    }

    [Fact]
    public void MinimumAtLeftBoundary()
    {
        // f(x) = x^2 on [0, 10] - minimum at x = 0
        SquareFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 0.0, 10.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void MinimumAtRightBoundary()
    {
        // f(x) = (10 - x)^2 on [0, 10] - minimum at x = 10
        ShiftedSquareFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 0.0, 10.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(10.0, aResult.Root!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void VeryFlatFunction()
    {
        // f(x) = (x - 5)^4 - very flat near minimum
        FlatQuarticFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 0.0, 10.0);
        Assert.True(aResult.IsDone);
        Assert.Equal(5.0, aResult.Root!.Value, 1.0e-5); // Lower tolerance for flat function
    }
}

public class MathOpt_1D_BoolConversionTest
{
    [Fact]
    public void SuccessfulResultIsTrue()
    {
        MathOpt_1D_TestFunctions.ParabolaFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 0.0, 10.0);
        Assert.True((bool)aResult);
    }
}

public class MathOpt_1D_IterationTest
{
    [Fact]
    public void BrentConvergesQuickly()
    {
        MathOpt_1D_TestFunctions.ParabolaFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Brent(aFunc, 0.0, 10.0);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbIterations < 50);
    }

    [Fact]
    public void GoldenConverges()
    {
        MathOpt_1D_TestFunctions.ParabolaFunc aFunc = new();
        ScalarResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Golden(aFunc, 0.0, 10.0);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.NbIterations < 100);
    }
}
