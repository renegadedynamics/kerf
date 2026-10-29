// Port of: src/FoundationClasses/TKMath/GTests/MathUtils_Functor_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

internal static class MathUtils_Functor_Helpers
{
    internal const double THE_TOLERANCE = 1e-10;
    internal const double THE_PI = 3.14159265358979323846;
}

public class MathUtils_Functor_Scalar
{
    private const double THE_TOLERANCE = MathUtils_Functor_Helpers.THE_TOLERANCE;
    private const double THE_PI = MathUtils_Functor_Helpers.THE_PI;

    [Fact]
    public void ScalarLambda_Value()
    {
        var aFunc = MathUtils.MakeScalar((x, out y) =>
        {
            y = x * x - 2.0;
            return true;
        });

        double aValue;
        Assert.True(aFunc.Value(2.0, out aValue));
        Assert.Equal(2.0, aValue, THE_TOLERANCE);

        Assert.True(aFunc.Value(Math.Sqrt(2.0), out aValue));
        Assert.Equal(0.0, aValue, THE_TOLERANCE);
    }

    [Fact]
    public void ScalarLambdaWithDerivative_Values()
    {
        var aFunc = MathUtils.MakeScalarWithDerivative((x, out y, out dy) =>
        {
            y = x * x - 2.0;
            dy = 2.0 * x;
            return true;
        });

        Assert.True(aFunc.Values(3.0, out double aValue, out double aDeriv));
        Assert.Equal(7.0, aValue, THE_TOLERANCE);
        Assert.Equal(6.0, aDeriv, THE_TOLERANCE);
    }

    [Fact]
    public void Polynomial_Value()
    {
        // p(x) = 1 + 2x + 3x^2
        MathUtils.Polynomial aPoly = new([1.0, 2.0, 3.0]);

        double aValue;
        Assert.True(aPoly.Value(0.0, out aValue));
        Assert.Equal(1.0, aValue, THE_TOLERANCE);

        Assert.True(aPoly.Value(1.0, out aValue));
        Assert.Equal(6.0, aValue, THE_TOLERANCE); // 1 + 2 + 3

        Assert.True(aPoly.Value(2.0, out aValue));
        Assert.Equal(17.0, aValue, THE_TOLERANCE); // 1 + 4 + 12
    }

    [Fact]
    public void Polynomial_Values()
    {
        // p(x) = x^2 - 2, p'(x) = 2x
        MathUtils.Polynomial aPoly = new([-2.0, 0.0, 1.0]);

        Assert.True(aPoly.Values(3.0, out double aValue, out double aDeriv));
        Assert.Equal(7.0, aValue, THE_TOLERANCE);
        Assert.Equal(6.0, aDeriv, THE_TOLERANCE);
    }

    [Fact]
    public void Polynomial_WithRoots()
    {
        // Using Brent's method with polynomial
        // p(x) = x^2 - 2, roots at +/-sqrt(2)
        MathUtils.Polynomial aPoly = new([-2.0, 0.0, 1.0]);

        var aResult = MathRoot.Brent(aPoly, 0.0, 2.0);
        Assert.True(aResult.IsDone());
        Assert.True(aResult.Root.HasValue);
        Assert.Equal(Math.Sqrt(2.0), aResult.Root!.Value, 1e-9);
    }

    [Fact]
    public void Polynomial_WithNewton()
    {
        // Using Newton's method with polynomial (has derivative)
        // p(x) = x^3 - 2x - 5
        MathUtils.Polynomial aPoly = new([-5.0, -2.0, 0.0, 1.0]);

        var aResult = MathRoot.Newton(aPoly, 2.0);
        Assert.True(aResult.IsDone());
        Assert.True(aResult.Root.HasValue);
        // Root is approximately 2.0945514815
        Assert.Equal(2.0945514815, aResult.Root!.Value, 1e-8);
    }

    [Fact]
    public void Rational_Value()
    {
        // f(x) = (x + 1) / (x^2 + 1)
        MathUtils.Rational aRat = new([1.0, 1.0], [1.0, 0.0, 1.0]);

        double aValue;
        Assert.True(aRat.Value(0.0, out aValue));
        Assert.Equal(1.0, aValue, THE_TOLERANCE);

        Assert.True(aRat.Value(1.0, out aValue));
        Assert.Equal(1.0, aValue, THE_TOLERANCE); // 2/2 = 1

        Assert.True(aRat.Value(2.0, out aValue));
        Assert.Equal(0.6, aValue, THE_TOLERANCE); // 3/5 = 0.6
    }

    [Fact]
    public void Rational_DivisionByZero()
    {
        // f(x) = x / (x - 1), undefined at x = 1
        MathUtils.Rational aRat = new([0.0, 1.0], [-1.0, 1.0]);

        Assert.False(aRat.Value(1.0, out _));
    }

    [Fact]
    public void Composite_Value()
    {
        // f(g(x)) where f(x) = x^2, g(x) = x + 1
        // Result: (x+1)^2
        MathUtils.Polynomial aOuter = new([0.0, 0.0, 1.0]); // x^2
        MathUtils.Polynomial aInner = new([1.0, 1.0]);      // x + 1
        var aComposite = MathUtils.MakeComposite(aOuter, aInner);

        double aValue;
        Assert.True(aComposite.Value(0.0, out aValue));
        Assert.Equal(1.0, aValue, THE_TOLERANCE); // (0+1)^2 = 1

        Assert.True(aComposite.Value(2.0, out aValue));
        Assert.Equal(9.0, aValue, THE_TOLERANCE); // (2+1)^2 = 9
    }

    [Fact]
    public void Sum_Value()
    {
        // f(x) + g(x) = (x^2) + (2x + 1) = x^2 + 2x + 1
        MathUtils.Polynomial aF = new([0.0, 0.0, 1.0]); // x^2
        MathUtils.Polynomial aG = new([1.0, 2.0]);      // 2x + 1
        var aSum = MathUtils.MakeSum(aF, aG);

        double aValue;
        Assert.True(aSum.Value(1.0, out aValue));
        Assert.Equal(4.0, aValue, THE_TOLERANCE); // 1 + 3 = 4

        Assert.True(aSum.Value(2.0, out aValue));
        Assert.Equal(9.0, aValue, THE_TOLERANCE); // 4 + 5 = 9
    }

    [Fact]
    public void Difference_Value()
    {
        // f(x) - g(x) = (x^2) - (x) = x^2 - x
        MathUtils.Polynomial aF = new([0.0, 0.0, 1.0]); // x^2
        MathUtils.Polynomial aG = new([0.0, 1.0]);      // x
        var aDiff = MathUtils.MakeDifference(aF, aG);

        Assert.True(aDiff.Value(3.0, out double aValue));
        Assert.Equal(6.0, aValue, THE_TOLERANCE); // 9 - 3 = 6
    }

    [Fact]
    public void Product_Value()
    {
        // f(x) * g(x) = (x) * (x + 1) = x^2 + x
        MathUtils.Polynomial aF = new([0.0, 1.0]); // x
        MathUtils.Polynomial aG = new([1.0, 1.0]); // x + 1
        var aProd = MathUtils.MakeProduct(aF, aG);

        Assert.True(aProd.Value(2.0, out double aValue));
        Assert.Equal(6.0, aValue, THE_TOLERANCE); // 2 * 3 = 6
    }

    [Fact]
    public void Quotient_Value()
    {
        // f(x) / g(x) = (x^2) / (x) = x
        MathUtils.Polynomial aF = new([0.0, 0.0, 1.0]); // x^2
        MathUtils.Polynomial aG = new([0.0, 1.0]);      // x
        var aQuot = MathUtils.MakeQuotient(aF, aG);

        Assert.True(aQuot.Value(5.0, out double aValue));
        Assert.Equal(5.0, aValue, THE_TOLERANCE);
    }

    [Fact]
    public void Scaled_Value()
    {
        // 3 * f(x) = 3 * x^2
        MathUtils.Polynomial aF = new([0.0, 0.0, 1.0]); // x^2
        var aScaled = MathUtils.MakeScaled(aF, 3.0);

        Assert.True(aScaled.Value(2.0, out double aValue));
        Assert.Equal(12.0, aValue, THE_TOLERANCE); // 3 * 4 = 12
    }

    [Fact]
    public void Shifted_Value()
    {
        // f(x) + 5 = x^2 + 5
        MathUtils.Polynomial aF = new([0.0, 0.0, 1.0]); // x^2
        var aShifted = MathUtils.MakeShifted(aF, 5.0);

        Assert.True(aShifted.Value(2.0, out double aValue));
        Assert.Equal(9.0, aValue, THE_TOLERANCE); // 4 + 5 = 9
    }

    [Fact]
    public void Negated_Value()
    {
        // -f(x) = -x^2
        MathUtils.Polynomial aF = new([0.0, 0.0, 1.0]); // x^2
        var aNegated = MathUtils.MakeNegated(aF);

        Assert.True(aNegated.Value(3.0, out double aValue));
        Assert.Equal(-9.0, aValue, THE_TOLERANCE);
    }

    [Fact]
    public void Constant_Value()
    {
        MathUtils.Constant aConst = new(42.0);

        double aValue;
        Assert.True(aConst.Value(123.456, out aValue));
        Assert.Equal(42.0, aValue, THE_TOLERANCE);

        Assert.True(aConst.Values(789.0, out aValue, out double aDeriv));
        Assert.Equal(42.0, aValue, THE_TOLERANCE);
        Assert.Equal(0.0, aDeriv, THE_TOLERANCE);
    }

    [Fact]
    public void Linear_Value()
    {
        // f(x) = 2x + 3
        MathUtils.Linear aLinear = new(2.0, 3.0);

        double aValue;
        Assert.True(aLinear.Value(5.0, out aValue));
        Assert.Equal(13.0, aValue, THE_TOLERANCE);

        Assert.True(aLinear.Values(5.0, out aValue, out double aDeriv));
        Assert.Equal(13.0, aValue, THE_TOLERANCE);
        Assert.Equal(2.0, aDeriv, THE_TOLERANCE);
    }

    [Fact]
    public void Sine_Value()
    {
        MathUtils.Sine aSine = new(1.0, 1.0, 0.0, 0.0);

        double aValue;
        Assert.True(aSine.Value(0.0, out aValue));
        Assert.Equal(0.0, aValue, THE_TOLERANCE);

        Assert.True(aSine.Value(THE_PI / 2.0, out aValue));
        Assert.Equal(1.0, aValue, THE_TOLERANCE);
    }

    [Fact]
    public void Sine_Values()
    {
        MathUtils.Sine aSine = new(2.0, 1.0, 0.0, 0.0); // 2*sin(x)

        Assert.True(aSine.Values(0.0, out double aValue, out double aDeriv));
        Assert.Equal(0.0, aValue, THE_TOLERANCE);
        Assert.Equal(2.0, aDeriv, THE_TOLERANCE); // 2*cos(0) = 2
    }

    [Fact]
    public void Cosine_Value()
    {
        MathUtils.Cosine aCosine = new(1.0, 1.0, 0.0, 0.0);

        double aValue;
        Assert.True(aCosine.Value(0.0, out aValue));
        Assert.Equal(1.0, aValue, THE_TOLERANCE);

        Assert.True(aCosine.Value(THE_PI, out aValue));
        Assert.Equal(-1.0, aValue, THE_TOLERANCE);
    }

    [Fact]
    public void Exponential_Value()
    {
        MathUtils.Exponential aExp = new(1.0, 1.0, 0.0); // exp(x)

        double aValue;
        Assert.True(aExp.Value(0.0, out aValue));
        Assert.Equal(1.0, aValue, THE_TOLERANCE);

        Assert.True(aExp.Value(1.0, out aValue));
        Assert.Equal(Math.Exp(1.0), aValue, THE_TOLERANCE);
    }

    [Fact]
    public void Power_Value()
    {
        MathUtils.Power aPower = new(2.0, 1.0, 0.0); // x^2

        Assert.True(aPower.Value(3.0, out double aValue));
        Assert.Equal(9.0, aValue, THE_TOLERANCE);
    }

    [Fact]
    public void Power_NonInteger()
    {
        MathUtils.Power aPower = new(0.5, 1.0, 0.0); // sqrt(x)

        double aValue;
        Assert.True(aPower.Value(4.0, out aValue));
        Assert.Equal(2.0, aValue, THE_TOLERANCE);

        // Negative x with non-integer exponent should fail
        Assert.False(aPower.Value(-4.0, out aValue));
    }

    [Fact]
    public void Gaussian_Value()
    {
        MathUtils.Gaussian aGauss = new(1.0, 0.0, 1.0); // Standard normal

        double aValue;
        Assert.True(aGauss.Value(0.0, out aValue));
        Assert.Equal(1.0, aValue, THE_TOLERANCE); // Peak at mean

        Assert.True(aGauss.Value(1.0, out aValue));
        Assert.Equal(Math.Exp(-0.5), aValue, THE_TOLERANCE);
    }
}

public class MathUtils_Functor_Vector
{
    private const double THE_TOLERANCE = MathUtils_Functor_Helpers.THE_TOLERANCE;

    [Fact]
    public void VectorLambda_Value()
    {
        var aFunc = MathUtils.MakeVector((x, out y) =>
        {
            y = x[1] * x[1] + x[2] * x[2]; // Sphere
            return true;
        });

        math_Vector aX = new(1, 2);
        aX[1] = 3.0;
        aX[2] = 4.0;

        Assert.True(aFunc.Value(aX, out double aValue));
        Assert.Equal(25.0, aValue, THE_TOLERANCE); // 9 + 16 = 25
    }

    [Fact]
    public void VectorLambdaWithGradient_Values()
    {
        var aFunc = MathUtils.MakeVectorWithGradient(
          (x, out y) =>
          {
              y = x[1] * x[1] + x[2] * x[2];
              return true;
          },
          (x, g) =>
          {
              g[1] = 2.0 * x[1];
              g[2] = 2.0 * x[2];
              return true;
          });

        math_Vector aX = new(1, 2);
        aX[1] = 3.0;
        aX[2] = 4.0;

        math_Vector aGrad = new(1, 2);
        Assert.True(aFunc.Values(aX, out double aValue, aGrad));
        Assert.Equal(25.0, aValue, THE_TOLERANCE);
        Assert.Equal(6.0, aGrad[1], THE_TOLERANCE);
        Assert.Equal(8.0, aGrad[2], THE_TOLERANCE);
    }

    [Fact]
    public void Rosenbrock_Value()
    {
        MathUtils.Rosenbrock aRosen = new(); // Default a=1, b=100

        math_Vector aX = new(1, 2);
        aX[1] = 1.0;
        aX[2] = 1.0;

        Assert.True(aRosen.Value(aX, out double aValue));
        Assert.Equal(0.0, aValue, THE_TOLERANCE); // Minimum at (1, 1)
    }

    [Fact]
    public void Rosenbrock_AwayFromMinimum()
    {
        MathUtils.Rosenbrock aRosen = new();

        math_Vector aX = new(1, 2);
        aX[1] = 0.0;
        aX[2] = 0.0;

        Assert.True(aRosen.Value(aX, out double aValue));
        Assert.Equal(1.0, aValue, THE_TOLERANCE); // (1-0)^2 + 100*(0-0)^2 = 1
    }

    [Fact]
    public void Rosenbrock_Gradient()
    {
        MathUtils.Rosenbrock aRosen = new();

        math_Vector aX = new(1, 2);
        aX[1] = 1.0;
        aX[2] = 1.0;

        math_Vector aGrad = new(1, 2);
        Assert.True(aRosen.Gradient(aX, aGrad));
        Assert.Equal(0.0, aGrad[1], THE_TOLERANCE); // Gradient is zero at minimum
        Assert.Equal(0.0, aGrad[2], THE_TOLERANCE);
    }

    [Fact]
    public void Sphere_Value()
    {
        MathUtils.Sphere aSphere = new();

        math_Vector aX = new(1, 3);
        aX[1] = 1.0;
        aX[2] = 2.0;
        aX[3] = 3.0;

        Assert.True(aSphere.Value(aX, out double aValue));
        Assert.Equal(14.0, aValue, THE_TOLERANCE); // 1 + 4 + 9 = 14
    }

    [Fact]
    public void Sphere_Gradient()
    {
        MathUtils.Sphere aSphere = new();

        math_Vector aX = new(1, 3);
        aX[1] = 1.0;
        aX[2] = 2.0;
        aX[3] = 3.0;

        math_Vector aGrad = new(1, 3);
        Assert.True(aSphere.Gradient(aX, aGrad));
        Assert.Equal(2.0, aGrad[1], THE_TOLERANCE);
        Assert.Equal(4.0, aGrad[2], THE_TOLERANCE);
        Assert.Equal(6.0, aGrad[3], THE_TOLERANCE);
    }

    [Fact]
    public void Booth_Value()
    {
        MathUtils.Booth aBooth = new();

        math_Vector aX = new(1, 2);
        aX[1] = 1.0;
        aX[2] = 3.0;

        Assert.True(aBooth.Value(aX, out double aValue));
        Assert.Equal(0.0, aValue, THE_TOLERANCE); // Minimum at (1, 3)
    }

    [Fact]
    public void Beale_Value()
    {
        MathUtils.Beale aBeale = new();

        math_Vector aX = new(1, 2);
        aX[1] = 3.0;
        aX[2] = 0.5;

        Assert.True(aBeale.Value(aX, out double aValue));
        Assert.Equal(0.0, aValue, THE_TOLERANCE); // Minimum at (3, 0.5)
    }

    [Fact]
    public void Himmelblau_Value()
    {
        MathUtils.Himmelblau aHimmel = new();

        // Test one of the four minima: (3, 2)
        math_Vector aX = new(1, 2);
        aX[1] = 3.0;
        aX[2] = 2.0;

        Assert.True(aHimmel.Value(aX, out double aValue));
        Assert.Equal(0.0, aValue, THE_TOLERANCE);
    }

    [Fact]
    public void Rastrigin_Value()
    {
        MathUtils.Rastrigin aRast = new();

        math_Vector aX = new(1, 2);
        aX[1] = 0.0;
        aX[2] = 0.0;

        Assert.True(aRast.Value(aX, out double aValue));
        Assert.Equal(0.0, aValue, THE_TOLERANCE); // Minimum at origin
    }

    [Fact]
    public void Ackley_Value()
    {
        MathUtils.Ackley aAckley = new();

        math_Vector aX = new(1, 2);
        aX[1] = 0.0;
        aX[2] = 0.0;

        Assert.True(aAckley.Value(aX, out double aValue));
        Assert.Equal(0.0, aValue, THE_TOLERANCE); // Minimum at origin
    }

    [Fact]
    public void QuadraticForm_Value()
    {
        // f(x) = x^T * I * x = ||x||^2 (identity matrix)
        math_Matrix aA = new(1, 2, 1, 2, 0.0);
        aA[1, 1] = 1.0;
        aA[2, 2] = 1.0;

        math_Vector aB = new(1, 2, 0.0);
        double aC = 0.0;

        MathUtils.QuadraticForm aQuad = new(aA, aB, aC);

        math_Vector aX = new(1, 2);
        aX[1] = 3.0;
        aX[2] = 4.0;

        Assert.True(aQuad.Value(aX, out double aValue));
        Assert.Equal(25.0, aValue, THE_TOLERANCE); // 9 + 16 = 25
    }

    [Fact]
    public void QuadraticForm_Gradient()
    {
        // f(x) = x^T * I * x, gradient = 2x
        math_Matrix aA = new(1, 2, 1, 2, 0.0);
        aA[1, 1] = 1.0;
        aA[2, 2] = 1.0;

        math_Vector aB = new(1, 2, 0.0);
        double aC = 0.0;

        MathUtils.QuadraticForm aQuad = new(aA, aB, aC);

        math_Vector aX = new(1, 2);
        aX[1] = 3.0;
        aX[2] = 4.0;

        math_Vector aGrad = new(1, 2);
        Assert.True(aQuad.Gradient(aX, aGrad));
        Assert.Equal(6.0, aGrad[1], THE_TOLERANCE);
        Assert.Equal(8.0, aGrad[2], THE_TOLERANCE);
    }

    [Fact]
    public void LinearResidual_Value()
    {
        // Solve: [1 0; 0 1] * x = [3; 4]
        // Residual at x = [3, 4] should be 0
        math_Matrix aA = new(1, 2, 1, 2, 0.0);
        aA[1, 1] = 1.0;
        aA[2, 2] = 1.0;

        math_Vector aB = new(1, 2);
        aB[1] = 3.0;
        aB[2] = 4.0;

        MathUtils.LinearResidual aRes = new(aA, aB);

        math_Vector aX = new(1, 2);
        aX[1] = 3.0;
        aX[2] = 4.0;

        Assert.True(aRes.Value(aX, out double aValue));
        Assert.Equal(0.0, aValue, THE_TOLERANCE);
    }

    [Fact]
    public void LinearResidual_NonZero()
    {
        math_Matrix aA = new(1, 2, 1, 2, 0.0);
        aA[1, 1] = 1.0;
        aA[2, 2] = 1.0;

        math_Vector aB = new(1, 2);
        aB[1] = 3.0;
        aB[2] = 4.0;

        MathUtils.LinearResidual aRes = new(aA, aB);

        math_Vector aX = new(1, 2);
        aX[1] = 0.0;
        aX[2] = 0.0;

        Assert.True(aRes.Value(aX, out double aValue));
        Assert.Equal(25.0, aValue, THE_TOLERANCE); // ||[3,4]||^2 = 25
    }
}

public class MathUtils_Functor_Integration
{
    private const double THE_PI = MathUtils_Functor_Helpers.THE_PI;

    [Fact]
    public void Polynomial_WithBrent()
    {
        // Find root of x^3 - x - 2 = 0 (root near 1.52)
        MathUtils.Polynomial aPoly = new([-2.0, -1.0, 0.0, 1.0]);

        var aResult = MathRoot.Brent(aPoly, 1.0, 2.0);
        Assert.True(aResult.IsDone());
        Assert.True(aResult.Root.HasValue);

        // Verify the root
        aPoly.Value(aResult.Root!.Value, out double aValue);
        Assert.Equal(0.0, aValue, 1e-10);
    }

    [Fact]
    public void Sine_RootFinding()
    {
        // Find root of sin(x) in [2, 4] (root at pi)
        MathUtils.Sine aSine = new();

        var aResult = MathRoot.Brent(aSine, 2.0, 4.0);
        Assert.True(aResult.IsDone());
        Assert.True(aResult.Root.HasValue);
        Assert.Equal(THE_PI, aResult.Root!.Value, 1e-10);
    }

    [Fact]
    public void Sphere_WithPowell()
    {
        // Minimize sphere function - should find origin
        MathUtils.Sphere aSphere = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 5.0;
        aStart[2] = 5.0;

        var aResult = MathOpt.Powell(aSphere, aStart);
        Assert.True(aResult.IsDone());
        Assert.NotNull(aResult.Solution);
        Assert.Equal(0.0, aResult.Solution![1], 1e-5);
        Assert.Equal(0.0, aResult.Solution![2], 1e-5);
    }

    [Fact]
    public void Booth_WithBFGS()
    {
        // Minimize Booth function - should find (1, 3)
        MathUtils.Booth aBooth = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 0.0;
        aStart[2] = 0.0;

        var aResult = MathOpt.BFGS(aBooth, aStart);
        Assert.True(aResult.IsDone());
        Assert.NotNull(aResult.Solution);
        Assert.Equal(1.0, aResult.Solution![1], 1e-5);
        Assert.Equal(3.0, aResult.Solution![2], 1e-5);
    }

    [Fact]
    public void Rosenbrock_WithBFGS()
    {
        // Minimize Rosenbrock - more challenging
        MathUtils.Rosenbrock aRosen = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = -1.0;
        aStart[2] = 1.0;

        MathUtils.NDimConfig aConfig = new();
        aConfig.MaxIterations = 500;
        aConfig.FTolerance = 1e-10;
        aConfig.XTolerance = 1e-10;

        var aResult = MathOpt.BFGS(aRosen, aStart, aConfig);
        Assert.True(aResult.IsDone());
        Assert.NotNull(aResult.Solution);
        Assert.Equal(1.0, aResult.Solution![1], 1e-4);
        Assert.Equal(1.0, aResult.Solution![2], 1e-4);
    }

    [Fact]
    public void Quadratic_WithNewton()
    {
        // Minimize quadratic form with Newton's method
        // f(x) = x^T * A * x + b^T * x + c
        // where A = diag(2, 4), b = [-4, -8], c = 10
        // Minimum at x = A^(-1) * (-b/2) = [1, 1] with f = 6
        math_Matrix aA = new(1, 2, 1, 2, 0.0);
        aA[1, 1] = 2.0;
        aA[2, 2] = 4.0;

        math_Vector aB = new(1, 2);
        aB[1] = -4.0;
        aB[2] = -8.0;

        MathUtils.QuadraticForm aQuad = new(aA, aB, 10.0);

        math_Vector aStart = new(1, 2);
        aStart[1] = 0.0;
        aStart[2] = 0.0;

        var aResult = MathOpt.BFGS(aQuad, aStart);
        Assert.True(aResult.IsDone());
        Assert.NotNull(aResult.Solution);
        Assert.Equal(1.0, aResult.Solution![1], 1e-5);
        Assert.Equal(1.0, aResult.Solution![2], 1e-5);
    }
}
