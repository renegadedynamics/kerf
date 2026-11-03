// Port of: src/FoundationClasses/TKMath/GTests/MathInteg_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// MathInteg::IntegResult, MathInteg::IntegConfig and MathInteg::Status are MathUtils' types, reached in C++ through
// `using namespace MathUtils`; here they are written MathUtils.IntegResult and so on.

namespace Kerf.Kernel.Foundation.Tests;

file static class MathInteg_TestConstants
{
    public const double THE_TOLERANCE = 1.0e-10;
    public const double THE_PI = 3.14159265358979323846;
}

// Constant function: f(x) = 5
file sealed class ConstantFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = 5.0;
        return true;
    }
}

// Linear function: f(x) = 2x + 1
file sealed class LinearFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = 2.0 * theX + 1.0;
        return true;
    }
}

// Quadratic function: f(x) = x^2
file sealed class QuadraticFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = theX * theX;
        return true;
    }
}

// Sine function: f(x) = sin(x)
file sealed class SinFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = Math.Sin(theX);
        return true;
    }
}

// Cosine function: f(x) = cos(x)
file sealed class CosFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = Math.Cos(theX);
        return true;
    }
}

// Exponential function: f(x) = e^x
file sealed class ExpFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = Math.Exp(theX);
        return true;
    }
}

// Polynomial: f(x) = x^5 - 3x^3 + 2x
file sealed class PolynomialFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        double aX2 = theX * theX;
        double aX3 = aX2 * theX;
        theF = aX2 * aX3 - 3.0 * aX3 + 2.0 * theX;
        return true;
    }
}

// Gaussian function: f(x) = e^(-x^2)
file sealed class GaussianFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = Math.Exp(-theX * theX);
        return true;
    }
}

// Reciprocal function: f(x) = 1/x
file sealed class ReciprocalFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = 1.0 / theX;
        return true;
    }
}

// Oscillatory function: f(x) = sin(10x)
file sealed class OscillatoryFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = Math.Sin(10.0 * theX);
        return true;
    }
}

// Square root: f(x) = sqrt(x)
file sealed class SqrtFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = Math.Sqrt(theX);
        return true;
    }
}

// f(x) = 1/(1+x^2), integral from -inf to inf = PI
file sealed class LorentzianFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = 1.0 / (1.0 + theX * theX);
        return true;
    }
}

public class MathInteg_GaussTest
{
    private const double THE_TOLERANCE = MathInteg_TestConstants.THE_TOLERANCE;
    private const double THE_PI = MathInteg_TestConstants.THE_PI;

    [Fact]
    public void ConstantFunction()
    {
        ConstantFunc aFunc = new();
        // Integral of 5 from 0 to 2 = 10
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 2.0, 3);
        Assert.True(aResult.IsDone);
        Assert.Equal(10.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void LinearFunction()
    {
        LinearFunc aFunc = new();
        // Integral of (2x+1) from 0 to 3 = [x^2 + x]_0^3 = 9 + 3 = 12
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 3.0, 3);
        Assert.True(aResult.IsDone);
        Assert.Equal(12.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void QuadraticFunction()
    {
        QuadraticFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 1.0, 3);
        Assert.True(aResult.IsDone);
        Assert.Equal(1.0 / 3.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void SineFunction()
    {
        SinFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, THE_PI, 15);
        Assert.True(aResult.IsDone);
        Assert.Equal(2.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CosineFunction()
    {
        CosFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, THE_PI / 2.0, 15);
        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void ExponentialFunction()
    {
        ExpFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 1.0, 15);
        Assert.True(aResult.IsDone);
        Assert.Equal(Math.Exp(1.0) - 1.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void ReciprocalFunction()
    {
        ReciprocalFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 1.0, Math.Exp(1.0), 15);
        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Order3()
    {
        QuadraticFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 1.0, 3);
        Assert.True(aResult.IsDone);
        Assert.Equal(3, aResult.NbPoints);
        Assert.Equal(1.0 / 3.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Order5()
    {
        QuadraticFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 1.0, 5);
        Assert.True(aResult.IsDone);
        Assert.Equal(5, aResult.NbPoints);
        Assert.Equal(1.0 / 3.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Order7()
    {
        QuadraticFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 1.0, 7);
        Assert.True(aResult.IsDone);
        Assert.Equal(7, aResult.NbPoints);
        Assert.Equal(1.0 / 3.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Order15()
    {
        SinFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, THE_PI, 15);
        Assert.True(aResult.IsDone);
        Assert.Equal(15, aResult.NbPoints);
        Assert.Equal(2.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Order21()
    {
        SinFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, THE_PI, 21);
        Assert.True(aResult.IsDone);
        Assert.Equal(21, aResult.NbPoints);
        Assert.Equal(2.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Order9()
    {
        SinFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, THE_PI, 9);
        Assert.True(aResult.IsDone);
        Assert.Equal(9, aResult.NbPoints);
        Assert.Equal(2.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Order61()
    {
        SinFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, THE_PI, 61);
        Assert.True(aResult.IsDone);
        Assert.Equal(61, aResult.NbPoints);
        Assert.Equal(2.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void InvalidOrderNonPositive()
    {
        SinFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, THE_PI, 0);
        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput, aResult.Status);
    }

    [Fact]
    public void PolynomialExactness_Order3()
    {
        // 3-point Gauss should be exact for polynomials up to degree 5
        PolynomialFunc aFunc = new();
        // Integral of (x^5 - 3x^3 + 2x) from -1 to 1 = 0 (odd function)
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, -1.0, 1.0, 3);
        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void PolynomialExactness_Order7()
    {
        // 7-point Gauss should be exact for polynomials up to degree 13
        PolynomialFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, -1.0, 1.0, 7);
        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }
}

public class MathInteg_GaussAdaptiveTest
{
    private const double THE_TOLERANCE = MathInteg_TestConstants.THE_TOLERANCE;
    private const double THE_PI = MathInteg_TestConstants.THE_PI;

    [Fact]
    public void SineFunction()
    {
        SinFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegConfig aConfig = new();
        aConfig.Tolerance = 1.0e-12;

        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.GaussAdaptive(aFunc, 0.0, THE_PI, aConfig);
        Assert.True(aResult.IsDone);
        Assert.Equal(2.0, aResult.Value!.Value, 1.0e-11);
    }

    [Fact]
    public void ExponentialFunction()
    {
        ExpFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegConfig aConfig = new();
        aConfig.Tolerance = 1.0e-12;

        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.GaussAdaptive(aFunc, 0.0, 1.0, aConfig);
        Assert.True(aResult.IsDone);
        Assert.Equal(Math.Exp(1.0) - 1.0, aResult.Value!.Value, 1.0e-11);
    }

    [Fact]
    public void OscillatoryFunction()
    {
        OscillatoryFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegConfig aConfig = new();
        aConfig.Tolerance = 1.0e-8;
        aConfig.MaxIterations = 20;

        // Integral of sin(10x) from 0 to PI = (1 - cos(10*PI))/10 = 0
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.GaussAdaptive(aFunc, 0.0, THE_PI, aConfig);
        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-6);
    }

    [Fact]
    public void ProvidesErrorEstimate()
    {
        SinFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegConfig aConfig = new();
        aConfig.Tolerance = 1.0e-8;

        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.GaussAdaptive(aFunc, 0.0, THE_PI, aConfig);
        Assert.True(aResult.IsDone);
        Assert.True(aResult.AbsoluteError.HasValue);
        Assert.True(aResult.RelativeError.HasValue);
        Assert.True(double.IsFinite(aResult.AbsoluteError!.Value));
        Assert.True(double.IsFinite(aResult.RelativeError!.Value));
        Assert.True(aResult.AbsoluteError.Value >= 0.0);
        Assert.True(aResult.RelativeError.Value >= 0.0);
        Assert.True(aResult.AbsoluteError.Value < 1.0e-6);
    }

    [Fact]
    public void UsesConfiguredOrders()
    {
        QuadraticFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegConfig aConfig = new();
        aConfig.InitialOrder = 9;
        aConfig.MaxOrder = 18;
        aConfig.Tolerance = 1.0e-12;
        aConfig.MaxIterations = 2;

        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.GaussAdaptive(aFunc, 0.0, 1.0, aConfig);
        Assert.True(aResult.IsDone);
        Assert.Equal(18, aResult.NbPoints);
        Assert.Equal(1.0 / 3.0, aResult.Value!.Value, THE_TOLERANCE);
    }
}

public class MathInteg_GaussCompositeTest
{
    private const double THE_TOLERANCE = MathInteg_TestConstants.THE_TOLERANCE;
    private const double THE_PI = MathInteg_TestConstants.THE_PI;

    [Fact]
    public void SineFunction()
    {
        SinFunc aFunc = new();
        // 10 subintervals with 7-point Gauss each
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.GaussComposite(aFunc, 0.0, THE_PI, 10, 7);
        Assert.True(aResult.IsDone);
        Assert.Equal(2.0, aResult.Value!.Value, THE_TOLERANCE);
        Assert.Equal(70, aResult.NbPoints); // 10 * 7
    }

    [Fact]
    public void OscillatoryFunction()
    {
        OscillatoryFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.GaussComposite(aFunc, 0.0, THE_PI, 50, 7);
        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-8);
    }

    [Fact]
    public void GaussianFunction()
    {
        GaussianFunc aFunc = new();
        // Integral from -3 to 3 (approximates -inf to inf)
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.GaussComposite(aFunc, -3.0, 3.0, 50, 15);
        Assert.True(aResult.IsDone);
        // Should be close to sqrt(PI) ~= 1.7724538509
        Assert.Equal(Math.Sqrt(THE_PI), aResult.Value!.Value, 1.0e-4);
    }

    [Fact]
    public void InvalidSubintervals()
    {
        SinFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.GaussComposite(aFunc, 0.0, THE_PI, 0, 7);
        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput, aResult.Status);
    }
}

public class MathInteg_BoundsTest
{
    private const double THE_TOLERANCE = MathInteg_TestConstants.THE_TOLERANCE;
    private const double THE_PI = MathInteg_TestConstants.THE_PI;

    [Fact]
    public void ReversedBounds()
    {
        SinFunc aFunc = new();
        // Integral from PI to 0 should be -2
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, THE_PI, 0.0, 15);
        Assert.True(aResult.IsDone);
        Assert.Equal(-2.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void NegativeBounds()
    {
        QuadraticFunc aFunc = new();
        // Integral of x^2 from -1 to 1 = 2/3
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, -1.0, 1.0, 7);
        Assert.True(aResult.IsDone);
        Assert.Equal(2.0 / 3.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void SmallInterval()
    {
        SinFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 0.001, 15);
        Assert.True(aResult.IsDone);
        // sin(x) ~= x for small x, so integral ~= x^2/2 = 0.0000005
        Assert.Equal(0.0000005, aResult.Value!.Value, 1.0e-10);
    }

    [Fact]
    public void LargeInterval()
    {
        LorentzianFunc aFunc = new();
        // Approximate with large but finite bounds
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.GaussComposite(aFunc, -100.0, 100.0, 200, 15);
        Assert.True(aResult.IsDone);
        Assert.Equal(THE_PI, aResult.Value!.Value, 0.05); // Wide tolerance for infinite integral approximation
    }
}

public class MathInteg_SpecialTest
{
    [Fact]
    public void SqrtFunction()
    {
        SqrtFunc aFunc = new();
        // Integral of sqrt(x) from 0.001 to 1, avoiding the derivative singularity at x=0
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.GaussComposite(aFunc, 0.001, 1.0, 20, 15);
        Assert.True(aResult.IsDone);
        double aExpected = (2.0 / 3.0) * (1.0 - Math.Pow(0.001, 1.5));
        Assert.Equal(aExpected, aResult.Value!.Value, 1.0e-6);
    }
}

public class MathInteg_BoolConversionTest
{
    private const double THE_PI = MathInteg_TestConstants.THE_PI;

    [Fact]
    public void SuccessfulResultIsTrue()
    {
        SinFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, THE_PI, 15);
        Assert.True((bool)aResult);
    }

    [Fact]
    public void InvalidInputIsFalse()
    {
        SinFunc aFunc = new();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, THE_PI, 0);
        Assert.False((bool)aResult);
    }
}

public class MathInteg_AccuracyTest
{
    private const double THE_PI = MathInteg_TestConstants.THE_PI;

    [Fact]
    public void HigherOrderMoreAccurate()
    {
        SinFunc aFunc = new();
        double aExact = 2.0;

        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult3 = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, THE_PI, 3);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult7 = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, THE_PI, 7);
        global::Kerf.Engine.Numerics.Utilities.MathUtils.IntegResult aResult15 = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, THE_PI, 15);

        Assert.True(aResult3.IsDone);
        Assert.True(aResult7.IsDone);
        Assert.True(aResult15.IsDone);

        double aError3 = Math.Abs(aResult3.Value!.Value - aExact);
        double aError7 = Math.Abs(aResult7.Value!.Value - aExact);
        double aError15 = Math.Abs(aResult15.Value!.Value - aExact);

        Assert.True(aError3 > aError7);
        Assert.True(aError7 > aError15);
    }
}
