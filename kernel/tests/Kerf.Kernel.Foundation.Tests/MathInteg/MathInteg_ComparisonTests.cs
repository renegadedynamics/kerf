// Port of: src/FoundationClasses/TKMath/GTests/MathInteg_Comparison_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

// Adapter classes for old API (inherit from math_Function)

file sealed class SinFuncOld : Function
{
    public override bool Value(double theX, out double theF)
    {
        theF = Math.Sin(theX);
        return true;
    }
}

file sealed class QuadraticFuncOld : Function
{
    public override bool Value(double theX, out double theF)
    {
        theF = theX * theX;
        return true;
    }
}

file sealed class ExpFuncOld : Function
{
    public override bool Value(double theX, out double theF)
    {
        theF = Math.Exp(theX);
        return true;
    }
}

file sealed class CosFuncOld : Function
{
    public override bool Value(double theX, out double theF)
    {
        theF = Math.Cos(theX);
        return true;
    }
}

file sealed class LogIntegrandOld : Function
{
    public override bool Value(double theX, out double theF)
    {
        theF = 1.0 / (1.0 + theX);
        return true;
    }
}

file sealed class LinearFuncOld : Function
{
    public override bool Value(double theX, out double theF)
    {
        theF = theX;
        return true;
    }
}

file sealed class CubicFuncOld : Function
{
    public override bool Value(double theX, out double theF)
    {
        theF = theX * theX * theX;
        return true;
    }
}

file sealed class ConstantFuncOld : Function
{
    public override bool Value(double theX, out double theF)
    {
        theF = 1.0;
        return true;
    }
}

// Function classes for new API

file struct SinFuncNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public readonly bool Value(double theX, out double theF)
    {
        theF = Math.Sin(theX);
        return true;
    }
}

file struct QuadraticFuncNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public readonly bool Value(double theX, out double theF)
    {
        theF = theX * theX;
        return true;
    }
}

file struct ExpFuncNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public readonly bool Value(double theX, out double theF)
    {
        theF = Math.Exp(theX);
        return true;
    }
}

file struct CosFuncNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public readonly bool Value(double theX, out double theF)
    {
        theF = Math.Cos(theX);
        return true;
    }
}

file struct LogIntegrandNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public readonly bool Value(double theX, out double theF)
    {
        theF = 1.0 / (1.0 + theX);
        return true;
    }
}

file struct LinearFuncNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public readonly bool Value(double theX, out double theF)
    {
        theF = theX;
        return true;
    }
}

file struct CubicFuncNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public readonly bool Value(double theX, out double theF)
    {
        theF = theX * theX * theX;
        return true;
    }
}

file struct ConstantFuncNew : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public readonly bool Value(double theX, out double theF)
    {
        theF = 1.0;
        return true;
    }
}

// x^5
file struct Poly5Func : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public readonly bool Value(double theX, out double theF)
    {
        double aX2 = theX * theX;
        theF = aX2 * aX2 * theX;
        return true;
    }
}

// x^7
file struct Poly7Func : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public readonly bool Value(double theX, out double theF)
    {
        double aX2 = theX * theX;
        double aX4 = aX2 * aX2;
        theF = aX4 * aX2 * theX;
        return true;
    }
}

public class MathInteg_ComparisonTest
{
    private const double THE_TOLERANCE = 1.0e-10;
    private const double THE_PI = 3.14159265358979323846;

    [Fact]
    public void Gauss_SinPi()
    {
        SinFuncOld anOldFunc = new();
        SinFuncNew aNewFunc = new();

        const int anOrder = 15;

        GaussSingleIntegration anOldInteg = new(anOldFunc, 0.0, THE_PI, anOrder);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aNewFunc, 0.0, THE_PI, anOrder);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), THE_TOLERANCE);
        Assert.Equal(2.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Gauss_Quadratic()
    {
        QuadraticFuncOld anOldFunc = new();
        QuadraticFuncNew aNewFunc = new();

        const int anOrder = 7; // Should be exact for degree 2 polynomial

        GaussSingleIntegration anOldInteg = new(anOldFunc, 0.0, 1.0, anOrder);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aNewFunc, 0.0, 1.0, anOrder);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), THE_TOLERANCE);
        Assert.Equal(1.0 / 3.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Gauss_Exponential()
    {
        ExpFuncOld anOldFunc = new();
        ExpFuncNew aNewFunc = new();

        const int anOrder = 15;

        GaussSingleIntegration anOldInteg = new(anOldFunc, 0.0, 1.0, anOrder);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aNewFunc, 0.0, 1.0, anOrder);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), THE_TOLERANCE);
        Assert.Equal(Math.Exp(1.0) - 1.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Gauss_Cosine()
    {
        CosFuncOld anOldFunc = new();
        CosFuncNew aNewFunc = new();

        const int anOrder = 15;

        GaussSingleIntegration anOldInteg = new(anOldFunc, 0.0, THE_PI / 2.0, anOrder);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aNewFunc, 0.0, THE_PI / 2.0, anOrder);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), THE_TOLERANCE);
        Assert.Equal(1.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Gauss_LogIntegrand()
    {
        LogIntegrandOld anOldFunc = new();
        LogIntegrandNew aNewFunc = new();

        const int anOrder = 15;

        GaussSingleIntegration anOldInteg = new(anOldFunc, 0.0, 1.0, anOrder);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aNewFunc, 0.0, 1.0, anOrder);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), THE_TOLERANCE);
        Assert.Equal(Math.Log(2.0), aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Gauss_Linear()
    {
        LinearFuncOld anOldFunc = new();
        LinearFuncNew aNewFunc = new();

        const int anOrder = 3; // Should be exact for degree 1 polynomial

        GaussSingleIntegration anOldInteg = new(anOldFunc, 0.0, 1.0, anOrder);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aNewFunc, 0.0, 1.0, anOrder);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), THE_TOLERANCE);
        Assert.Equal(0.5, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Gauss_Cubic()
    {
        CubicFuncOld anOldFunc = new();
        CubicFuncNew aNewFunc = new();

        const int anOrder = 7; // Should be exact for degree 3 polynomial

        GaussSingleIntegration anOldInteg = new(anOldFunc, 0.0, 1.0, anOrder);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aNewFunc, 0.0, 1.0, anOrder);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), THE_TOLERANCE);
        Assert.Equal(0.25, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Gauss_Constant()
    {
        ConstantFuncOld anOldFunc = new();
        ConstantFuncNew aNewFunc = new();

        const int anOrder = 3; // Should be exact for constant

        GaussSingleIntegration anOldInteg = new(anOldFunc, 0.0, 5.0, anOrder);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aNewFunc, 0.0, 5.0, anOrder);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), THE_TOLERANCE);
        Assert.Equal(5.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    private static void CompareOrder(int theOrder)
    {
        SinFuncOld anOldFunc = new();
        SinFuncNew aNewFunc = new();

        GaussSingleIntegration anOldInteg = new(anOldFunc, 0.0, THE_PI, theOrder);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aNewFunc, 0.0, THE_PI, theOrder);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), THE_TOLERANCE);
    }

    [Fact]
    public void Order3_Comparison() => CompareOrder(3);

    [Fact]
    public void Order5_Comparison() => CompareOrder(5);

    [Fact]
    public void Order7_Comparison() => CompareOrder(7);

    [Fact]
    public void Order15_Comparison() => CompareOrder(15);

    [Fact]
    public void Order21_Comparison() => CompareOrder(21);

    [Fact]
    public void Order41_Comparison() => CompareOrder(41);

    [Fact]
    public void HigherOrderMoreAccurate()
    {
        ExpFuncNew aFunc = new();

        double aExact = Math.Exp(1.0) - 1.0;

        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult3 = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 1.0, 3);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult7 = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 1.0, 7);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult15 = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 1.0, 15);

        Assert.True(aResult3.IsDone);
        Assert.True(aResult7.IsDone);
        Assert.True(aResult15.IsDone);

        double aError3 = Math.Abs(aResult3.Value!.Value - aExact);
        double aError7 = Math.Abs(aResult7.Value!.Value - aExact);
        double aError15 = Math.Abs(aResult15.Value!.Value - aExact);

        // Higher order should be more accurate (or equally accurate at machine precision)
        Assert.True(aError7 <= aError3 + 1.0e-15);
        Assert.True(aError15 <= aError7 + 1.0e-15);
    }

    [Fact]
    public void NegativeBounds()
    {
        QuadraticFuncOld anOldFunc = new();
        QuadraticFuncNew aNewFunc = new();

        const int anOrder = 7;

        GaussSingleIntegration anOldInteg = new(anOldFunc, -1.0, 1.0, anOrder);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aNewFunc, -1.0, 1.0, anOrder);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), THE_TOLERANCE);
        Assert.Equal(2.0 / 3.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void ReversedBounds()
    {
        SinFuncNew aFunc = new();

        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResultNormal = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, THE_PI, 15);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResultReversed = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, THE_PI, 0.0, 15);

        Assert.True(aResultNormal.IsDone);
        Assert.True(aResultReversed.IsDone);

        // Reversed bounds should give negative of normal result
        Assert.Equal(-aResultReversed.Value!.Value, aResultNormal.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void PolynomialExactness_Order3()
    {
        // Order 3 -> exact for degree up to 5
        Poly5Func aFunc = new();

        // Integral of x^5 from 0 to 1 = 1/6
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 1.0, 3);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0 / 6.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void PolynomialExactness_Order7()
    {
        // Order 7 -> exact for degree up to 13
        Poly7Func aFunc = new();

        // Integral of x^7 from 0 to 1 = 1/8
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 1.0, 7);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0 / 8.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void LargeInterval()
    {
        ExpFuncOld anOldFunc = new();
        ExpFuncNew aNewFunc = new();

        const int anOrder = 21;

        GaussSingleIntegration anOldInteg = new(anOldFunc, 0.0, 10.0, anOrder);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aNewFunc, 0.0, 10.0, anOrder);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), 1.0e-6);
        Assert.Equal(Math.Exp(10.0) - 1.0, aNewResult.Value!.Value, 1.0e-6);
    }

    [Fact]
    public void SmallInterval()
    {
        SinFuncOld anOldFunc = new();
        SinFuncNew aNewFunc = new();

        const int anOrder = 15;

        GaussSingleIntegration anOldInteg = new(anOldFunc, 0.0, 0.001, anOrder);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aNewFunc, 0.0, 0.001, anOrder);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), THE_TOLERANCE);
        // sin(x) ~= x for small x, so integral ~= x^2/2
        Assert.Equal(0.0000005, aNewResult.Value!.Value, 1.0e-12);
    }
}
