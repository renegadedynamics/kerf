// Port of: src/FoundationClasses/TKMath/GTests/MathOpt_Newton_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace OcctNet.Foundation.Tests;

public class MathOpt_NewtonTest
{
    private const double THE_TOLERANCE = 1.0e-6;

    // Test function structs for new API (with Hessian)

    // Quadratic function: f(x,y) = (x-1)^2 + (y-2)^2, minimum at (1, 2) with f = 0
    private struct QuadraticFunc : IVectorFunctionWithHessian
    {
        public bool Value(math_Vector theX, out double theF)
        {
            double aDx = theX[1] - 1.0;
            double aDy = theX[2] - 2.0;
            theF = aDx * aDx + aDy * aDy;
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            theGrad[1] = 2.0 * (theX[1] - 1.0);
            theGrad[2] = 2.0 * (theX[2] - 2.0);
            return true;
        }

        public bool Hessian(math_Vector theX, math_Matrix theHess)
        {
            theHess[1, 1] = 2.0;
            theHess[1, 2] = 0.0;
            theHess[2, 1] = 0.0;
            theHess[2, 2] = 2.0;
            return true;
        }
    }

    // Rosenbrock function: f(x,y) = 100*(y-x^2)^2 + (1-x)^2, minimum at (1, 1) with f = 0
    private struct RosenbrockFunc : IVectorFunctionWithHessian
    {
        public bool Value(math_Vector theX, out double theF)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = aY - aX * aX;
            double aT2 = 1.0 - aX;
            theF = 100.0 * aT1 * aT1 + aT2 * aT2;
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            double aX = theX[1];
            double aY = theX[2];
            theGrad[1] = -400.0 * aX * (aY - aX * aX) - 2.0 * (1.0 - aX);
            theGrad[2] = 200.0 * (aY - aX * aX);
            return true;
        }

        public bool Hessian(math_Vector theX, math_Matrix theHess)
        {
            double aX = theX[1];
            double aY = theX[2];
            theHess[1, 1] = 1200.0 * aX * aX - 400.0 * aY + 2.0;
            theHess[1, 2] = -400.0 * aX;
            theHess[2, 1] = -400.0 * aX;
            theHess[2, 2] = 200.0;
            return true;
        }
    }

    // Booth function: f(x,y) = (x + 2y - 7)^2 + (2x + y - 5)^2, minimum at (1, 3) with f = 0
    private struct BoothFunc : IVectorFunctionWithHessian
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

        public bool Hessian(math_Vector theX, math_Matrix theHess)
        {
            theHess[1, 1] = 10.0; // 2 + 8
            theHess[1, 2] = 8.0;  // 4 + 4
            theHess[2, 1] = 8.0;
            theHess[2, 2] = 10.0; // 8 + 2
            return true;
        }
    }

    // Sphere function in N dimensions: f(x) = sum(x_i^2), minimum at origin with f = 0
    private struct SphereFunc : IVectorFunctionWithHessian
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

        public bool Hessian(math_Vector theX, math_Matrix theHess)
        {
            int aLower = theX.Lower();
            int aUpper = theX.Upper();
            for (int i = aLower; i <= aUpper; ++i)
            {
                for (int j = aLower; j <= aUpper; ++j)
                {
                    theHess[i, j] = (i == j) ? 2.0 : 0.0;
                }
            }
            return true;
        }
    }

    // Old API adapter classes

    private sealed class QuadraticFuncOld : math_MultipleVarFunctionWithHessian
    {
        public override int NbVariables() => 2;

        public override bool Value(math_Vector theX, out double theF)
        {
            double aDx = theX[1] - 1.0;
            double aDy = theX[2] - 2.0;
            theF = aDx * aDx + aDy * aDy;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            theG[1] = 2.0 * (theX[1] - 1.0);
            theG[2] = 2.0 * (theX[2] - 2.0);
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            return Value(theX, out theF) && Gradient(theX, theG);
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG, math_Matrix theH)
        {
            if (!Value(theX, out theF))
            {
                return false;
            }
            if (!Gradient(theX, theG))
            {
                return false;
            }
            theH[1, 1] = 2.0;
            theH[1, 2] = 0.0;
            theH[2, 1] = 0.0;
            theH[2, 2] = 2.0;
            return true;
        }
    }

    private sealed class BoothFuncOld : math_MultipleVarFunctionWithHessian
    {
        public override int NbVariables() => 2;

        public override bool Value(math_Vector theX, out double theF)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = aX + 2.0 * aY - 7.0;
            double aT2 = 2.0 * aX + aY - 5.0;
            theF = aT1 * aT1 + aT2 * aT2;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = aX + 2.0 * aY - 7.0;
            double aT2 = 2.0 * aX + aY - 5.0;
            theG[1] = 2.0 * aT1 + 4.0 * aT2;
            theG[2] = 4.0 * aT1 + 2.0 * aT2;
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            return Value(theX, out theF) && Gradient(theX, theG);
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG, math_Matrix theH)
        {
            if (!Value(theX, out theF))
            {
                return false;
            }
            if (!Gradient(theX, theG))
            {
                return false;
            }
            theH[1, 1] = 10.0;
            theH[1, 2] = 8.0;
            theH[2, 1] = 8.0;
            theH[2, 2] = 10.0;
            return true;
        }
    }

    // Function with gradient only
    private struct QuadraticGradOnly : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theF)
        {
            double aDx = theX[1] - 1.0;
            double aDy = theX[2] - 2.0;
            theF = aDx * aDx + aDy * aDy;
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            theGrad[1] = 2.0 * (theX[1] - 1.0);
            theGrad[2] = 2.0 * (theX[2] - 2.0);
            return true;
        }
    }

    // Function with value only
    private struct QuadraticValueOnly : IVectorFunction
    {
        public bool Value(math_Vector theX, out double theF)
        {
            double aDx = theX[1] - 1.0;
            double aDy = theX[2] - 2.0;
            theF = aDx * aDx + aDy * aDy;
            return true;
        }
    }

    private static MathOpt.NewtonConfig MakeConfig(int theMaxIter, double theTolerance)
    {
        MathOpt.NewtonConfig aConfig = new();
        aConfig.MaxIterations = theMaxIter;
        aConfig.Tolerance = theTolerance;
        return aConfig;
    }

    // Basic Newton minimization tests

    [Fact]
    public void Quadratic()
    {
        QuadraticFunc aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(5.0, 7.0);

        VectorResult aResult = MathOpt.Newton(aFunc, aStart, MakeConfig(50, 1.0e-10));

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);

        // Newton should converge in very few iterations for quadratic
        Assert.True(aResult.NbIterations <= 5);
    }

    [Fact]
    public void Booth()
    {
        BoothFunc aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(0.0, 0.0);

        VectorResult aResult = MathOpt.Newton(aFunc, aStart, MakeConfig(50, 1.0e-10));

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(3.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Rosenbrock()
    {
        RosenbrockFunc aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(0.5, 0.5); // Start closer to solution for Newton

        VectorResult aResult = MathOpt.Newton(aFunc, aStart, MakeConfig(100, 1.0e-8));

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], 1.0e-3);
        Assert.Equal(1.0, aResult.Solution![2], 1.0e-3);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-4);
    }

    [Fact]
    public void Sphere3D()
    {
        SphereFunc aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(3.0, -2.0, 4.0);

        VectorResult aResult = MathOpt.Newton(aFunc, aStart, MakeConfig(50, 1.0e-10));

        Assert.True(aResult.IsDone());
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(0.0, aResult.Solution![i], THE_TOLERANCE);
        }
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);

        // Newton should converge in very few iterations for quadratic
        Assert.True(aResult.NbIterations <= 5);
    }

    // Modified Newton tests

    [Fact]
    public void ModifiedNewton_Quadratic()
    {
        QuadraticFunc aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(5.0, 7.0);

        VectorResult aResult = MathOpt.NewtonModified(aFunc, aStart, MakeConfig(50, 1.0e-10));

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void ModifiedNewton_Rosenbrock()
    {
        RosenbrockFunc aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(-1.0, 1.0);

        MathOpt.NewtonConfig aConfig = MakeConfig(200, 1.0e-6);
        aConfig.Regularization = 1.0e-4; // Higher regularization for Rosenbrock from far start

        VectorResult aResult = MathOpt.NewtonModified(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], 1.0e-3);
        Assert.Equal(1.0, aResult.Solution![2], 1.0e-3);
    }

    // Numerical Hessian tests

    [Fact]
    public void NumericalHessian_Quadratic()
    {
        QuadraticGradOnly aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(5.0, 7.0);

        VectorResult aResult = MathOpt.NewtonNumericalHessian(aFunc, aStart, 1.0e-6, MakeConfig(50, 1.0e-8));

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], 1.0e-4);
        Assert.Equal(2.0, aResult.Solution![2], 1.0e-4);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-6);
    }

    [Fact]
    public void FullNumerical_Quadratic()
    {
        QuadraticValueOnly aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(5.0, 7.0);

        VectorResult aResult = MathOpt.NewtonNumerical(aFunc, aStart, 1.0e-8, 1.0e-6, MakeConfig(50, 1.0e-6));

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], 1.0e-3);
        Assert.Equal(2.0, aResult.Solution![2], 1.0e-3);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-4);
    }

    // Comparison with old API

    [Fact]
    public void CompareWithOldAPI_Quadratic()
    {
        QuadraticFuncOld anOldFunc = new();
        QuadraticFunc aNewFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(5.0, 7.0);

        // Old API
        math_NewtonMinimum anOldSolver = new(anOldFunc, 1.0e-8, 50, 1.0e-10);
        anOldSolver.Perform(anOldFunc, aStart);

        // New API
        VectorResult aNewResult = MathOpt.Newton(aNewFunc, aStart, MakeConfig(50, 1.0e-10));

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        // Both should find the same minimum
        Assert.Equal(aNewResult.Solution![1], anOldSolver.Location()[1], THE_TOLERANCE);
        Assert.Equal(aNewResult.Solution![2], anOldSolver.Location()[2], THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);
    }

    [Fact]
    public void CompareWithOldAPI_Booth()
    {
        BoothFuncOld anOldFunc = new();
        BoothFunc aNewFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(0.0, 0.0);

        // Old API
        math_NewtonMinimum anOldSolver = new(anOldFunc, 1.0e-8, 50, 1.0e-10);
        anOldSolver.Perform(anOldFunc, aStart);

        // New API
        VectorResult aNewResult = MathOpt.Newton(aNewFunc, aStart, MakeConfig(50, 1.0e-10));

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        // Both should find the same minimum
        Assert.Equal(aNewResult.Solution![1], anOldSolver.Location()[1], THE_TOLERANCE);
        Assert.Equal(aNewResult.Solution![2], anOldSolver.Location()[2], THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);
    }

    // Comparison with BFGS

    [Fact]
    public void CompareWithBFGS()
    {
        QuadraticFunc aQuadFunc = new();

        // Function that works with both Newton and BFGS
        QuadraticGradOnly aBFGSFunc = new();

        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(5.0, 7.0);

        Config aBFGSConfig = new();
        aBFGSConfig.MaxIterations = 100;
        aBFGSConfig.XTolerance = 1.0e-10;
        aBFGSConfig.FTolerance = 1.0e-12;

        VectorResult aNewtonResult = MathOpt.Newton(aQuadFunc, aStart, MakeConfig(50, 1.0e-10));
        VectorResult aBFGSResult = MathOpt.BFGS(aBFGSFunc, aStart, aBFGSConfig);

        Assert.True(aNewtonResult.IsDone());
        Assert.True(aBFGSResult.IsDone());

        // Both should find the same minimum
        Assert.Equal(aBFGSResult.Value!.Value, aNewtonResult.Value!.Value, THE_TOLERANCE);

        // Newton should use fewer iterations for quadratic functions
        Assert.True(aNewtonResult.NbIterations <= aBFGSResult.NbIterations);
    }

    // Convergence rate test

    [Fact]
    public void QuadraticConvergence()
    {
        QuadraticFunc aFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(100.0, 100.0);

        VectorResult aResult = MathOpt.Newton(aFunc, aStart, MakeConfig(10, 1.0e-12));

        Assert.True(aResult.IsDone());

        // Pure Newton should converge in exactly 1 iteration for a quadratic
        // (or 2 with line search)
        Assert.True(aResult.NbIterations <= 2);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-10);
    }
}
