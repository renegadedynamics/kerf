// Port of: src/FoundationClasses/TKMath/GTests/MathInteg_New_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// The C++ test functors are structs with a non-const Value; they stay structs here.

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

// Polynomial: f(x) = x^2
file struct PolynomialFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = theX * theX;
        return true;
    }
}

// Sine function: f(x) = sin(x)
file struct SineFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = Math.Sin(theX);
        return true;
    }
}

// Exponential: f(x) = exp(-x^2)
file struct GaussianFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = Math.Exp(-theX * theX);
        return true;
    }
}

// Oscillatory: f(x) = cos(10*x)
file struct OscillatoryFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = Math.Cos(10.0 * theX);
        return true;
    }
}

// Square root singularity: f(x) = 1/sqrt(x)
file struct SqrtSingularityFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = 0.0;
        if (theX <= 0.0)
        {
            return false;
        }
        theF = 1.0 / Math.Sqrt(theX);
        return true;
    }
}

// Log singularity: f(x) = -log(x)
file struct LogSingularityFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = 0.0;
        if (theX <= 0.0)
        {
            return false;
        }
        theF = -Math.Log(theX);
        return true;
    }
}

// Decaying exponential for semi-infinite: f(x) = exp(-x)
file struct ExponentialDecayFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = Math.Exp(-theX);
        return true;
    }
}

// Gaussian for infinite interval: f(x) = exp(-x^2)
file struct InfiniteGaussianFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = Math.Exp(-theX * theX);
        return true;
    }
}

// Function with known singularity at x = 0.5
file struct SingularFunc : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
{
    public bool Value(double theX, out double theF)
    {
        theF = 0.0;
        double aDist = Math.Abs(theX - 0.5);
        if (aDist < 1.0e-10)
        {
            return false;
        }
        theF = 1.0 / Math.Sqrt(aDist);
        return true;
    }
}

// Old API adapter
file sealed class SineFuncOld : Function
{
    public override bool Value(double theX, out double theF)
    {
        theF = Math.Sin(theX);
        return true;
    }
}

file sealed class PolynomialFuncOld : Function
{
    public override bool Value(double theX, out double theF)
    {
        theF = theX * theX;
        return true;
    }
}

public class MathInteg_Kronrod_NewTest
{
    private const double THE_TOLERANCE = 1.0e-8;

    [Fact]
    public void KronrodRule_Polynomial()
    {
        PolynomialFunc aFunc = new();

        // Integral of x^2 from 0 to 1 = 1/3
        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.KronrodRule(aFunc, 0.0, 1.0, 7);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0 / 3.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void KronrodRule_Sine()
    {
        SineFunc aFunc = new();

        // Integral of sin(x) from 0 to pi = 2
        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.KronrodRule(aFunc, 0.0, M_PI, 7);

        Assert.True(aResult.IsDone);
        Assert.Equal(2.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void AdaptiveKronrod_Oscillatory()
    {
        OscillatoryFunc aFunc = new();

        // Integral of cos(10x) from 0 to pi = 0
        global::Kerf.Engine.Numerics.Integration.MathInteg.KronrodConfig aConfig = new();
        aConfig.Tolerance = 1.0e-10;
        aConfig.NbGaussPoints = 7;
        aConfig.Adaptive = true;
        aConfig.MaxIterations = 100;

        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Kronrod(aFunc, 0.0, M_PI, aConfig);

        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-6);
    }

    [Fact]
    public void AdaptiveKronrod_Gaussian()
    {
        GaussianFunc aFunc = new();

        // Integral of exp(-x^2) from -inf to +inf = sqrt(pi)
        // Use SinhSinh for proper infinite interval integration
        global::Kerf.Engine.Numerics.Integration.MathInteg.DoubleExpConfig aConfig = new();
        aConfig.Tolerance = 1.0e-10;

        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.SinhSinh(aFunc, aConfig);

        Assert.True(aResult.IsDone);
        Assert.Equal(Math.Sqrt(M_PI), aResult.Value!.Value, 1.0e-6);
    }

    [Fact]
    public void KronrodAuto()
    {
        SineFunc aFunc = new();

        // Integral of sin(x) from 0 to 2*pi = 0
        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.KronrodAuto(aFunc, 0.0, 2.0 * M_PI);

        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-8);
    }

    [Fact]
    public void DifferentOrders()
    {
        PolynomialFunc aFunc = new();

        // Test with different Gauss orders
        int[] aOrders = [3, 5, 7, 10, 15];

        foreach (int aOrder in aOrders)
        {
            var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.KronrodRule(aFunc, 0.0, 1.0, aOrder);

            Assert.True(aResult.IsDone, $"Failed with order {aOrder}");
            Assert.Equal(1.0 / 3.0, aResult.Value!.Value, 1.0e-8);
        }
    }

    // Infinite interval tests
    // Note: For infinite intervals, DoubleExp (TanhSinh/ExpSinh/SinhSinh) methods
    // are preferred over Kronrod transformations due to better numerical stability.

    [Fact]
    public void InfiniteInterval()
    {
        InfiniteGaussianFunc aFunc = new();

        // Integral of exp(-x^2) from -inf to +inf = sqrt(pi)
        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.SinhSinh(aFunc);

        Assert.True(aResult.IsDone);
        Assert.Equal(Math.Sqrt(M_PI), aResult.Value!.Value, 1.0e-4);
    }

    [Fact]
    public void SemiInfiniteInterval()
    {
        ExponentialDecayFunc aFunc = new();

        // Integral of exp(-x) from 0 to +inf = 1
        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.ExpSinh(aFunc, 0.0);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Value!.Value, 1.0e-4);
    }
}

public class MathInteg_DoubleExp_NewTest
{
    [Fact]
    public void TanhSinh_Polynomial()
    {
        PolynomialFunc aFunc = new();

        // Integral of x^2 from 0 to 1 = 1/3
        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.TanhSinh(aFunc, 0.0, 1.0);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0 / 3.0, aResult.Value!.Value, 1.0e-6);
    }

    [Fact]
    public void TanhSinh_Sine()
    {
        SineFunc aFunc = new();

        // Integral of sin(x) from 0 to pi = 2
        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.TanhSinh(aFunc, 0.0, M_PI);

        Assert.True(aResult.IsDone);
        Assert.Equal(2.0, aResult.Value!.Value, 1.0e-6);
    }

    [Fact]
    public void TanhSinh_SqrtSingularity()
    {
        SqrtSingularityFunc aFunc = new();

        // Integral of 1/sqrt(x) from 0 to 1 = 2; singularity at x=0
        global::Kerf.Engine.Numerics.Integration.MathInteg.DoubleExpConfig aConfig = new();
        aConfig.NbLevels = 8;

        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.TanhSinh(aFunc, 0.0, 1.0, aConfig);

        Assert.True(aResult.IsDone);
        Assert.Equal(2.0, aResult.Value!.Value, 1.0e-4);
    }

    [Fact]
    public void TanhSinh_LogSingularity()
    {
        LogSingularityFunc aFunc = new();

        // Integral of -log(x) from 0 to 1 = 1; log singularity at x=0
        global::Kerf.Engine.Numerics.Integration.MathInteg.DoubleExpConfig aConfig = new();
        aConfig.NbLevels = 8;

        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.TanhSinh(aFunc, 0.0, 1.0, aConfig);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Value!.Value, 1.0e-4);
    }

    [Fact]
    public void TanhSinhSingular()
    {
        SqrtSingularityFunc aFunc = new();

        // Optimized for endpoint singularities
        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.TanhSinhSingular(aFunc, 0.0, 1.0, 1.0e-6);

        Assert.True(aResult.IsDone);
        Assert.Equal(2.0, aResult.Value!.Value, 1.0e-3);
    }

    [Fact]
    public void ExpSinh_SemiInfinite()
    {
        ExponentialDecayFunc aFunc = new();

        // Integral of exp(-x) from 0 to infinity = 1
        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.ExpSinh(aFunc, 0.0);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Value!.Value, 1.0e-4);
    }

    [Fact]
    public void SinhSinh_Infinite()
    {
        InfiniteGaussianFunc aFunc = new();

        // Integral of exp(-x^2) from -inf to +inf = sqrt(pi)
        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.SinhSinh(aFunc);

        Assert.True(aResult.IsDone);
        Assert.Equal(Math.Sqrt(M_PI), aResult.Value!.Value, 1.0e-4);
    }

    [Fact]
    public void DoubleExponential_Auto()
    {
        SineFunc aFunc = new();

        // Finite interval
        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.DoubleExponential(aFunc, 0.0, M_PI);

        Assert.True(aResult.IsDone);
        Assert.Equal(2.0, aResult.Value!.Value, 1.0e-6);
    }

    [Fact]
    public void WithSingularity()
    {
        SingularFunc aFunc = new();

        // Split at singularity
        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.TanhSinhWithSingularity(aFunc, 0.0, 1.0, 0.5);

        Assert.True(aResult.IsDone);
        // The integral exists (both halves are sqrt singularities)
        Assert.True(aResult.Value!.Value > 0.0);
    }
}

public class MathInteg_NewTest
{
    [Fact]
    public void KronrodVsGauss_Polynomial()
    {
        PolynomialFunc aFunc = new();

        var aKronrodResult = global::Kerf.Engine.Numerics.Integration.MathInteg.KronrodRule(aFunc, 0.0, 1.0, 7);
        var aGaussResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, 1.0, 15);

        Assert.True(aKronrodResult.IsDone);
        Assert.True(aGaussResult.IsDone);

        Assert.Equal(aGaussResult.Value!.Value, aKronrodResult.Value!.Value, 1.0e-10);
    }

    [Fact]
    public void KronrodVsGauss_Sine()
    {
        SineFunc aFunc = new();

        var aKronrodResult = global::Kerf.Engine.Numerics.Integration.MathInteg.KronrodRule(aFunc, 0.0, M_PI, 7);
        var aGaussResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aFunc, 0.0, M_PI, 15);

        Assert.True(aKronrodResult.IsDone);
        Assert.True(aGaussResult.IsDone);

        Assert.Equal(aGaussResult.Value!.Value, aKronrodResult.Value!.Value, 1.0e-10);
    }

    [Fact]
    public void TanhSinhVsKronrod_Polynomial()
    {
        PolynomialFunc aFunc = new();

        var aTanhSinhResult = global::Kerf.Engine.Numerics.Integration.MathInteg.TanhSinh(aFunc, 0.0, 1.0);
        var aKronrodResult = global::Kerf.Engine.Numerics.Integration.MathInteg.KronrodRule(aFunc, 0.0, 1.0, 7);

        Assert.True(aTanhSinhResult.IsDone);
        Assert.True(aKronrodResult.IsDone);

        Assert.Equal(aKronrodResult.Value!.Value, aTanhSinhResult.Value!.Value, 1.0e-6);
    }

    [Fact]
    public void CompareWithOldAPI_Kronrod()
    {
        SineFuncOld anOldFunc = new();
        SineFunc aNewFunc = new();

        // Old API - constructor takes: func, lower, upper, nbPoints, tolerance, maxIterations
        KronrodSingleIntegration anOldInteg = new(anOldFunc, 0.0, M_PI, 15, 1.0e-10, 100);

        // New API
        global::Kerf.Engine.Numerics.Integration.MathInteg.KronrodConfig aConfig = new();
        aConfig.NbGaussPoints = 7;
        aConfig.Tolerance = 1.0e-10;
        aConfig.Adaptive = true;
        var aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Kronrod(aNewFunc, 0.0, M_PI, aConfig);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        // Both should give sin integral = 2
        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), 1.0e-8);
        Assert.Equal(2.0, aNewResult.Value!.Value, 1.0e-8);
    }

    [Fact]
    public void CompareWithOldAPI_Gauss()
    {
        PolynomialFuncOld anOldFunc = new();
        PolynomialFunc aNewFunc = new();

        // Old API
        GaussSingleIntegration anOldInteg = new(anOldFunc, 0.0, 1.0, 15);

        // New API
        var aNewResult = global::Kerf.Engine.Numerics.Integration.MathInteg.Gauss(aNewFunc, 0.0, 1.0, 15);

        Assert.True(anOldInteg.IsDone);
        Assert.True(aNewResult.IsDone);

        Assert.Equal(aNewResult.Value!.Value, anOldInteg.Value(), 1.0e-10);
    }

    [Fact]
    public void ErrorEstimation_Kronrod()
    {
        SineFunc aFunc = new();

        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.KronrodRule(aFunc, 0.0, M_PI, 7);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.AbsoluteError.HasValue);
        Assert.True(aResult.RelativeError.HasValue);

        // Error estimate should be small for smooth function
        Assert.True(aResult.AbsoluteError!.Value < 1.0e-10);
    }

    [Fact]
    public void ErrorEstimation_TanhSinh()
    {
        SineFunc aFunc = new();

        var aResult = global::Kerf.Engine.Numerics.Integration.MathInteg.TanhSinh(aFunc, 0.0, M_PI);

        Assert.True(aResult.IsDone);
        // TanhSinh provides error estimates when converged
    }
}
