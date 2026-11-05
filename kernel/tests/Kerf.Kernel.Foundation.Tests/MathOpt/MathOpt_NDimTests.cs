// Port of: src/FoundationClasses/TKMath/GTests/MathOpt_NDim_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Kernel.Foundation.Tests;

internal static class MathOpt_NDim_TestFunctions
{
    public static math_Vector Vec(params double[] theValues)
    {
        math_Vector aVec = new(1, theValues.Length);
        for (int i = 0; i < theValues.Length; ++i)
        {
            aVec[i + 1] = theValues[i];
        }
        return aVec;
    }

    // Rosenbrock function: f(x,y) = (1-x)^2 + 100(y-x^2)^2, minimum at (1, 1) with f = 0
    public sealed class RosenbrockFunction : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theF)
        {
            double aX = theX[1];
            double aY = theX[2];
            theF = (1.0 - aX) * (1.0 - aX) + 100.0 * (aY - aX * aX) * (aY - aX * aX);
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            double aX = theX[1];
            double aY = theX[2];
            theGrad[1] = -2.0 * (1.0 - aX) - 400.0 * aX * (aY - aX * aX);
            theGrad[2] = 200.0 * (aY - aX * aX);
            return true;
        }
    }

    // Simple quadratic: f(x,y) = x^2 + y^2, minimum at (0, 0) with f = 0
    public sealed class SimpleQuadratic : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theF)
        {
            theF = 0.0;
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theF += theX[i] * theX[i];
            }
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theGrad[i] = 2.0 * theX[i];
            }
            return true;
        }
    }

    // Sphere function in N dimensions: f(x) = sum(x_i^2), minimum at origin with f = 0
    public sealed class SphereFunction : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theF)
        {
            theF = 0.0;
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theF += theX[i] * theX[i];
            }
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theGrad[i] = 2.0 * theX[i];
            }
            return true;
        }
    }

    // Booth function: f(x,y) = (x + 2y - 7)^2 + (2x + y - 5)^2, minimum at (1, 3) with f = 0
    public sealed class BoothFunction : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theF)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = aX + 2.0 * aY - 7.0;
            double aT2 = 2.0 * aX + aY - 5.0;
            theF = aT1 * aT1 + aT2 * aT2;
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = aX + 2.0 * aY - 7.0;
            double aT2 = 2.0 * aX + aY - 5.0;
            theGrad[1] = 2.0 * aT1 + 4.0 * aT2;
            theGrad[2] = 4.0 * aT1 + 2.0 * aT2;
            return true;
        }
    }

    // Beale's function, minimum at (3, 0.5) with f = 0
    public sealed class BealeFunction : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theF)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = 1.5 - aX + aX * aY;
            double aT2 = 2.25 - aX + aX * aY * aY;
            double aT3 = 2.625 - aX + aX * aY * aY * aY;
            theF = aT1 * aT1 + aT2 * aT2 + aT3 * aT3;
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aY2 = aY * aY;
            double aY3 = aY2 * aY;
            double aT1 = 1.5 - aX + aX * aY;
            double aT2 = 2.25 - aX + aX * aY2;
            double aT3 = 2.625 - aX + aX * aY3;
            theGrad[1] = 2.0 * aT1 * (-1.0 + aY) + 2.0 * aT2 * (-1.0 + aY2) + 2.0 * aT3 * (-1.0 + aY3);
            theGrad[2] = 2.0 * aT1 * aX + 2.0 * aT2 * (2.0 * aX * aY) + 2.0 * aT3 * (3.0 * aX * aY2);
            return true;
        }
    }
}

public class MathOpt_PowellTest
{
    [Fact]
    public void SimpleQuadratic()
    {
        MathOpt_NDim_TestFunctions.SimpleQuadratic aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(5.0, 5.0);

        Config aConfig = new();
        aConfig.MaxIterations = 200;
        aConfig.XTolerance = 1.0e-8;
        aConfig.FTolerance = 1.0e-10;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Powell(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone);
        Assert.NotNull(aResult.Solution);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-6);
        Assert.Equal(0.0, aResult.Solution[1], 1.0e-4);
        Assert.Equal(0.0, aResult.Solution[2], 1.0e-4);
    }

    [Fact]
    public void BoothFunction()
    {
        MathOpt_NDim_TestFunctions.BoothFunction aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(0.0, 0.0);

        Config aConfig = new();
        aConfig.MaxIterations = 200;
        aConfig.XTolerance = 1.0e-8;
        aConfig.FTolerance = 1.0e-10;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Powell(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone);
        Assert.NotNull(aResult.Solution);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-6);
        Assert.Equal(1.0, aResult.Solution[1], 1.0e-4);
        Assert.Equal(3.0, aResult.Solution[2], 1.0e-4);
    }

    [Fact]
    public void SphereFunction3D()
    {
        MathOpt_NDim_TestFunctions.SphereFunction aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(3.0, -2.0, 4.0);

        Config aConfig = new();
        aConfig.MaxIterations = 200;
        aConfig.XTolerance = 1.0e-8;
        aConfig.FTolerance = 1.0e-10;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Powell(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone);
        Assert.NotNull(aResult.Solution);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-6);
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(0.0, aResult.Solution[i], 1.0e-4);
        }
    }
}

public class MathOpt_BFGSTest
{
    [Fact]
    public void SimpleQuadratic()
    {
        MathOpt_NDim_TestFunctions.SimpleQuadratic aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(5.0, 5.0);

        Config aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.XTolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-12;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone);
        Assert.NotNull(aResult.Solution);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-8);
        Assert.Equal(0.0, aResult.Solution[1], 1.0e-6);
        Assert.Equal(0.0, aResult.Solution[2], 1.0e-6);
        Assert.True(aResult.NbIterations < 20); // BFGS should converge quickly on quadratics
    }

    [Fact]
    public void BoothFunction()
    {
        MathOpt_NDim_TestFunctions.BoothFunction aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(0.0, 0.0);

        Config aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.XTolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-12;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone);
        Assert.NotNull(aResult.Solution);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-8);
        Assert.Equal(1.0, aResult.Solution[1], 1.0e-6);
        Assert.Equal(3.0, aResult.Solution[2], 1.0e-6);
    }

    [Fact]
    public void RosenbrockFunction()
    {
        MathOpt_NDim_TestFunctions.RosenbrockFunction aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(-1.0, 1.0);

        Config aConfig = new();
        aConfig.MaxIterations = 500;
        aConfig.XTolerance = 1.0e-8;
        aConfig.FTolerance = 1.0e-10;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone);
        Assert.NotNull(aResult.Solution);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-4);
        Assert.Equal(1.0, aResult.Solution[1], 1.0e-3);
        Assert.Equal(1.0, aResult.Solution[2], 1.0e-3);
    }

    [Fact]
    public void BealeFunction()
    {
        MathOpt_NDim_TestFunctions.BealeFunction aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(0.0, 0.0);

        Config aConfig = new();
        aConfig.MaxIterations = 200;
        aConfig.XTolerance = 1.0e-8;
        aConfig.FTolerance = 1.0e-10;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone);
        Assert.NotNull(aResult.Solution);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-4);
        Assert.Equal(3.0, aResult.Solution[1], 1.0e-3);
        Assert.Equal(0.5, aResult.Solution[2], 1.0e-3);
    }

    [Fact]
    public void SphereFunction5D()
    {
        MathOpt_NDim_TestFunctions.SphereFunction aFunc = new();
        math_Vector aStart = new(1, 5);
        for (int i = 1; i <= 5; ++i)
        {
            aStart[i] = (double)i;
        }

        Config aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.XTolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-12;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone);
        Assert.NotNull(aResult.Solution);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-8);
        for (int i = 1; i <= 5; ++i)
        {
            Assert.Equal(0.0, aResult.Solution[i], 1.0e-6);
        }
        Assert.True(aResult.NbIterations < 20); // Should converge quickly on quadratics
    }
}

public class MathOpt_BFGSNumericalTest
{
    // Same test but using numerical gradient
    private sealed class QuadraticNoGrad : IVectorFunction
    {
        public bool Value(math_Vector theX, out double theF)
        {
            theF = 0.0;
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theF += theX[i] * theX[i];
            }
            return true;
        }
    }

    [Fact]
    public void SimpleQuadratic()
    {
        QuadraticNoGrad aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(5.0, 5.0);

        Config aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.XTolerance = 1.0e-8;
        aConfig.FTolerance = 1.0e-10;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGSNumerical(aFunc, aStart, 1.0e-7, aConfig);

        Assert.True(aResult.IsDone);
        Assert.NotNull(aResult.Solution);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-6);
        Assert.Equal(0.0, aResult.Solution[1], 1.0e-4);
        Assert.Equal(0.0, aResult.Solution[2], 1.0e-4);
    }
}

public class MathOpt_LBFGSTest
{
    [Fact]
    public void SimpleQuadratic()
    {
        MathOpt_NDim_TestFunctions.SimpleQuadratic aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(5.0, 5.0);

        Config aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.XTolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-12;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.LBFGS(aFunc, aStart, 10, aConfig);

        Assert.True(aResult.IsDone);
        Assert.NotNull(aResult.Solution);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-8);
        Assert.Equal(0.0, aResult.Solution[1], 1.0e-6);
        Assert.Equal(0.0, aResult.Solution[2], 1.0e-6);
    }

    [Fact]
    public void BoothFunction()
    {
        MathOpt_NDim_TestFunctions.BoothFunction aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(0.0, 0.0);

        Config aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.XTolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-12;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.LBFGS(aFunc, aStart, 10, aConfig);

        Assert.True(aResult.IsDone);
        Assert.NotNull(aResult.Solution);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-8);
        Assert.Equal(1.0, aResult.Solution[1], 1.0e-6);
        Assert.Equal(3.0, aResult.Solution[2], 1.0e-6);
    }

    [Fact]
    public void SphereFunction10D()
    {
        MathOpt_NDim_TestFunctions.SphereFunction aFunc = new();
        math_Vector aStart = new(1, 10);
        for (int i = 1; i <= 10; ++i)
        {
            aStart[i] = (double)i;
        }

        Config aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.XTolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-12;

        // Use small memory size for L-BFGS
        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.LBFGS(aFunc, aStart, 5, aConfig);

        Assert.True(aResult.IsDone);
        Assert.NotNull(aResult.Solution);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-8);
        for (int i = 1; i <= 10; ++i)
        {
            Assert.Equal(0.0, aResult.Solution[i], 1.0e-6);
        }
    }
}

public class MathOpt_ComparisonTest
{
    [Fact]
    public void BFGSFasterThanPowellOnQuadratic()
    {
        MathOpt_NDim_TestFunctions.SimpleQuadratic aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(10.0, 10.0);

        Config aConfig = new();
        aConfig.MaxIterations = 500;
        aConfig.XTolerance = 1.0e-8;
        aConfig.FTolerance = 1.0e-10;

        VectorResult aPowellResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Powell(aFunc, aStart, aConfig);
        VectorResult aBFGSResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aFunc, aStart, aConfig);

        Assert.True(aPowellResult.IsDone);
        Assert.True(aBFGSResult.IsDone);

        // Both should find the same minimum
        Assert.Equal(aBFGSResult.Value!.Value, aPowellResult.Value!.Value, 1.0e-4);

        // BFGS should use fewer iterations on quadratic functions
        Assert.True(aBFGSResult.NbIterations < aPowellResult.NbIterations);
    }
}
