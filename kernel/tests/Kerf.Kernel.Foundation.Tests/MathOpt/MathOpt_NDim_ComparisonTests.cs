// Port of: src/FoundationClasses/TKMath/GTests/MathOpt_NDim_Comparison_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Kernel.Foundation.Tests;

public class MathOpt_NDim_ComparisonTest
{
    private const double THE_TOLERANCE = 1.0e-6;

    // Adapter classes for old API (inherit from math_MultipleVarFunction)

    // Quadratic function: f(x,y) = (x-1)^2 + (y-2)^2, minimum at (1, 2) with f = 0
    private sealed class QuadraticFuncOld : MultipleVarFunction
    {
        public override int VariableCount => 2;

        public override bool Value(math_Vector theX, out double theF)
        {
            double aDx = theX[1] - 1.0;
            double aDy = theX[2] - 2.0;
            theF = aDx * aDx + aDy * aDy;
            return true;
        }
    }

    // Rosenbrock function: f(x,y) = 100*(y-x^2)^2 + (1-x)^2, minimum at (1, 1) with f = 0
    private sealed class RosenbrockFuncOld : MultipleVarFunction
    {
        public override int VariableCount => 2;

        public override bool Value(math_Vector theX, out double theF)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = aY - aX * aX;
            double aT2 = 1.0 - aX;
            theF = 100.0 * aT1 * aT1 + aT2 * aT2;
            return true;
        }
    }

    // Booth function: f(x,y) = (x + 2y - 7)^2 + (2x + y - 5)^2, minimum at (1, 3) with f = 0
    private sealed class BoothFuncOld : MultipleVarFunction
    {
        public override int VariableCount => 2;

        public override bool Value(math_Vector theX, out double theF)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = aX + 2.0 * aY - 7.0;
            double aT2 = 2.0 * aX + aY - 5.0;
            theF = aT1 * aT1 + aT2 * aT2;
            return true;
        }
    }

    // Sphere function in N dimensions: f(x) = sum(x_i^2), minimum at origin with f = 0
    private sealed class SphereFuncOld : MultipleVarFunction
    {
        private readonly int myN;

        public SphereFuncOld(int theN)
        {
            myN = theN;
        }

        public override int VariableCount => myN;

        public override bool Value(math_Vector theX, out double theF)
        {
            theF = 0.0;
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theF += theX[i] * theX[i];
            }
            return true;
        }
    }

    // Adapter classes for old BFGS API (inherit from math_MultipleVarFunctionWithGradient)

    private sealed class QuadraticFuncGradOld : MultipleVarFunctionWithGradient
    {
        public override int VariableCount => 2;

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
    }

    private sealed class RosenbrockFuncGradOld : MultipleVarFunctionWithGradient
    {
        public override int VariableCount => 2;

        public override bool Value(math_Vector theX, out double theF)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = aY - aX * aX;
            double aT2 = 1.0 - aX;
            theF = 100.0 * aT1 * aT1 + aT2 * aT2;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            double aX = theX[1];
            double aY = theX[2];
            theG[1] = -400.0 * aX * (aY - aX * aX) - 2.0 * (1.0 - aX);
            theG[2] = 200.0 * (aY - aX * aX);
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            return Value(theX, out theF) && Gradient(theX, theG);
        }
    }

    private sealed class BoothFuncGradOld : MultipleVarFunctionWithGradient
    {
        public override int VariableCount => 2;

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
    }

    private sealed class SphereFuncGradOld : MultipleVarFunctionWithGradient
    {
        private readonly int myN;

        public SphereFuncGradOld(int theN)
        {
            myN = theN;
        }

        public override int VariableCount => myN;

        public override bool Value(math_Vector theX, out double theF)
        {
            theF = 0.0;
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theF += theX[i] * theX[i];
            }
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theG[i] = 2.0 * theX[i];
            }
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            return Value(theX, out theF) && Gradient(theX, theG);
        }
    }

    // Function structs for new API

    private struct QuadraticFuncNew : IVectorFunctionWithGradient
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

    private struct RosenbrockFuncNew : IVectorFunctionWithGradient
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
    }

    private struct BoothFuncNew : IVectorFunctionWithGradient
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

    private struct SphereFuncNew : IVectorFunctionWithGradient
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

    // Create identity matrix for Powell directions.
    private static Matrix CreateIdentityMatrix(int theN)
    {
        Matrix aMat = new(1, theN, 1, theN, 0.0);
        for (int i = 1; i <= theN; ++i)
        {
            aMat[i, i] = 1.0;
        }
        return aMat;
    }

    private static Config MakeConfig(int theMaxIter, double theXTol, double theFTol)
    {
        Config aConfig = new();
        aConfig.MaxIterations = theMaxIter;
        aConfig.XTolerance = theXTol;
        aConfig.FTolerance = theFTol;
        return aConfig;
    }

    // math_Powell vs MathOpt::Powell comparison tests

    [Fact]
    public void Powell_Quadratic()
    {
        QuadraticFuncOld anOldFunc = new();
        QuadraticFuncNew aNewFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(5.0, 7.0);

        // Old API
        Powell anOldSolver = new(anOldFunc, 1.0e-8, 200);
        Matrix aDirections = CreateIdentityMatrix(2);
        anOldSolver.Perform(anOldFunc, aStart, aDirections);

        // New API
        VectorResult aNewResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Powell(aNewFunc, aStart, MakeConfig(200, 1.0e-8, 1.0e-10));

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Both should find the same minimum
        Assert.Equal(aNewResult.Solution![1], anOldSolver.Location()[1], THE_TOLERANCE);
        Assert.Equal(aNewResult.Solution![2], anOldSolver.Location()[2], THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);

        // Verify correctness
        Assert.Equal(1.0, aNewResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(2.0, aNewResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Powell_Booth()
    {
        BoothFuncOld anOldFunc = new();
        BoothFuncNew aNewFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(0.0, 0.0);

        // Old API
        Powell anOldSolver = new(anOldFunc, 1.0e-8, 200);
        Matrix aDirections = CreateIdentityMatrix(2);
        anOldSolver.Perform(anOldFunc, aStart, aDirections);

        // New API
        VectorResult aNewResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Powell(aNewFunc, aStart, MakeConfig(200, 1.0e-8, 1.0e-10));

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Both should find the same minimum
        Assert.Equal(aNewResult.Solution![1], anOldSolver.Location()[1], THE_TOLERANCE);
        Assert.Equal(aNewResult.Solution![2], anOldSolver.Location()[2], THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);

        // Verify correctness
        Assert.Equal(1.0, aNewResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(3.0, aNewResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Powell_Rosenbrock()
    {
        RosenbrockFuncOld anOldFunc = new();
        RosenbrockFuncNew aNewFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(-1.0, 1.0);

        // Old API - needs more iterations for Rosenbrock
        Powell anOldSolver = new(anOldFunc, 1.0e-6, 1000);
        Matrix aDirections = CreateIdentityMatrix(2);
        anOldSolver.Perform(anOldFunc, aStart, aDirections);

        // New API
        VectorResult aNewResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Powell(aNewFunc, aStart, MakeConfig(1000, 1.0e-6, 1.0e-8));

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Both should find the minimum (relaxed tolerance for Rosenbrock)
        Assert.Equal(aNewResult.Solution![1], anOldSolver.Location()[1], 1.0e-3);
        Assert.Equal(aNewResult.Solution![2], anOldSolver.Location()[2], 1.0e-3);

        // Verify correctness (Rosenbrock minimum at (1, 1))
        Assert.Equal(1.0, aNewResult.Solution![1], 1.0e-3);
        Assert.Equal(1.0, aNewResult.Solution![2], 1.0e-3);
    }

    [Fact]
    public void Powell_Sphere3D()
    {
        SphereFuncOld anOldFunc = new(3);
        SphereFuncNew aNewFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(3.0, -2.0, 4.0);

        // Old API
        Powell anOldSolver = new(anOldFunc, 1.0e-8, 200);
        Matrix aDirections = CreateIdentityMatrix(3);
        anOldSolver.Perform(anOldFunc, aStart, aDirections);

        // New API
        VectorResult aNewResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Powell(aNewFunc, aStart, MakeConfig(200, 1.0e-8, 1.0e-10));

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Both should find the origin
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(aNewResult.Solution![i], anOldSolver.Location()[i], THE_TOLERANCE);
            Assert.Equal(0.0, aNewResult.Solution![i], THE_TOLERANCE);
        }
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);
        Assert.Equal(0.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    // math_BFGS vs MathOpt::BFGS comparison tests

    [Fact]
    public void BFGS_Quadratic()
    {
        QuadraticFuncGradOld anOldFunc = new();
        QuadraticFuncNew aNewFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(5.0, 7.0);

        // Old API
        Bfgs anOldSolver = new(2, 1.0e-8, 100);
        anOldSolver.Perform(anOldFunc, aStart);

        // New API
        VectorResult aNewResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aNewFunc, aStart, MakeConfig(100, 1.0e-8, 1.0e-10));

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Both should find the same minimum
        Assert.Equal(aNewResult.Solution![1], anOldSolver.Location()[1], THE_TOLERANCE);
        Assert.Equal(aNewResult.Solution![2], anOldSolver.Location()[2], THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);

        // Verify correctness
        Assert.Equal(1.0, aNewResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(2.0, aNewResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void BFGS_Booth()
    {
        BoothFuncGradOld anOldFunc = new();
        BoothFuncNew aNewFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(0.0, 0.0);

        // Old API
        Bfgs anOldSolver = new(2, 1.0e-8, 100);
        anOldSolver.Perform(anOldFunc, aStart);

        // New API
        VectorResult aNewResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aNewFunc, aStart, MakeConfig(100, 1.0e-8, 1.0e-10));

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Both should find the same minimum
        Assert.Equal(aNewResult.Solution![1], anOldSolver.Location()[1], THE_TOLERANCE);
        Assert.Equal(aNewResult.Solution![2], anOldSolver.Location()[2], THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);

        // Verify correctness
        Assert.Equal(1.0, aNewResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(3.0, aNewResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void BFGS_Rosenbrock()
    {
        RosenbrockFuncGradOld anOldFunc = new();
        RosenbrockFuncNew aNewFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(-1.0, 1.0);

        // Old API - needs more iterations for Rosenbrock
        Bfgs anOldSolver = new(2, 1.0e-6, 1000);
        anOldSolver.Perform(anOldFunc, aStart);

        // New API
        VectorResult aNewResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aNewFunc, aStart, MakeConfig(1000, 1.0e-6, 1.0e-8));

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Both should find the minimum (relaxed tolerance for Rosenbrock)
        Assert.Equal(aNewResult.Solution![1], anOldSolver.Location()[1], 1.0e-3);
        Assert.Equal(aNewResult.Solution![2], anOldSolver.Location()[2], 1.0e-3);

        // Verify correctness (Rosenbrock minimum at (1, 1))
        Assert.Equal(1.0, aNewResult.Solution![1], 1.0e-3);
        Assert.Equal(1.0, aNewResult.Solution![2], 1.0e-3);
    }

    [Fact]
    public void BFGS_Sphere3D()
    {
        SphereFuncGradOld anOldFunc = new(3);
        SphereFuncNew aNewFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(3.0, -2.0, 4.0);

        // Old API
        Bfgs anOldSolver = new(3, 1.0e-8, 100);
        anOldSolver.Perform(anOldFunc, aStart);

        // New API
        VectorResult aNewResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aNewFunc, aStart, MakeConfig(100, 1.0e-8, 1.0e-10));

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Both should find the origin
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(aNewResult.Solution![i], anOldSolver.Location()[i], THE_TOLERANCE);
            Assert.Equal(0.0, aNewResult.Solution![i], THE_TOLERANCE);
        }
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);
        Assert.Equal(0.0, aNewResult.Value!.Value, THE_TOLERANCE);
    }

    // Iteration count comparison tests

    [Fact]
    public void IterationCount_BFGSVsPowell()
    {
        QuadraticFuncNew aNewFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(10.0, 10.0);

        Config aConfig = MakeConfig(500, 1.0e-8, 1.0e-10);

        VectorResult aPowellResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.Powell(aNewFunc, aStart, aConfig);
        VectorResult aBFGSResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aNewFunc, aStart, aConfig);

        Assert.True(aPowellResult.IsDone);
        Assert.True(aBFGSResult.IsDone);

        // Both should find the same minimum
        Assert.Equal(aBFGSResult.Value!.Value, aPowellResult.Value!.Value, THE_TOLERANCE);

        // BFGS should use fewer iterations on quadratic functions (due to superlinear convergence)
        Assert.True(aBFGSResult.NbIterations < aPowellResult.NbIterations);
    }

    [Fact]
    public void IterationCount_LBFGSVsBFGS()
    {
        SphereFuncNew aNewFunc = new();
        math_Vector aStart = new(1, 5);
        for (int i = 1; i <= 5; ++i)
        {
            aStart[i] = (double)i;
        }

        Config aConfig = MakeConfig(100, 1.0e-8, 1.0e-10);

        VectorResult aBFGSResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aNewFunc, aStart, aConfig);
        VectorResult aLBFGSResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.LBFGS(aNewFunc, aStart, 10, aConfig);

        Assert.True(aBFGSResult.IsDone);
        Assert.True(aLBFGSResult.IsDone);

        // Both should find the same minimum
        Assert.Equal(aLBFGSResult.Value!.Value, aBFGSResult.Value!.Value, THE_TOLERANCE);

        // Both should find the origin
        for (int i = 1; i <= 5; ++i)
        {
            Assert.Equal(0.0, aBFGSResult.Solution![i], THE_TOLERANCE);
            Assert.Equal(0.0, aLBFGSResult.Solution![i], THE_TOLERANCE);
        }
    }

    // Accuracy comparison tests

    [Fact]
    public void Accuracy_BFGSVsOld()
    {
        QuadraticFuncGradOld anOldFunc = new();
        QuadraticFuncNew aNewFunc = new();
        math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(5.0, 7.0);

        // Tighter tolerance
        const double aTightTol = 1.0e-12;

        // Old API
        Bfgs anOldSolver = new(2, aTightTol, 200);
        anOldSolver.Perform(anOldFunc, aStart);

        // New API
        VectorResult aNewResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aNewFunc, aStart, MakeConfig(200, aTightTol, 1.0e-14));

        Assert.True(anOldSolver.IsDone);
        Assert.True(aNewResult.IsDone);

        // Both should achieve good accuracy
        Assert.Equal(1.0, anOldSolver.Location()[1], 1.0e-8);
        Assert.Equal(2.0, anOldSolver.Location()[2], 1.0e-8);
        Assert.Equal(1.0, aNewResult.Solution![1], 1.0e-8);
        Assert.Equal(2.0, aNewResult.Solution![2], 1.0e-8);
    }

    // Different starting points comparison tests

    [Fact]
    public void DifferentStartingPoints()
    {
        QuadraticFuncGradOld anOldFunc = new();
        QuadraticFuncNew aNewFunc = new();

        // Test multiple starting points
        (double, double)[] aStartPoints = [(0.0, 0.0), (10.0, 10.0), (-5.0, 5.0), (100.0, -50.0)];

        foreach ((double aFirst, double aSecond) in aStartPoints)
        {
            math_Vector aStart = MathOpt_NDim_TestFunctions.Vec(aFirst, aSecond);

            // Old API
            Bfgs anOldSolver = new(2, 1.0e-8, 200);
            anOldSolver.Perform(anOldFunc, aStart);

            // New API
            VectorResult aNewResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.BFGS(aNewFunc, aStart, MakeConfig(200, 1.0e-8, 1.0e-10));

            Assert.True(anOldSolver.IsDone, $"Old API failed for start point ({aFirst}, {aSecond})");
            Assert.True(aNewResult.IsDone, $"New API failed for start point ({aFirst}, {aSecond})");

            // Both should find the same minimum
            Assert.Equal(aNewResult.Solution![1], anOldSolver.Location()[1], THE_TOLERANCE);
            Assert.Equal(aNewResult.Solution![2], anOldSolver.Location()[2], THE_TOLERANCE);
            Assert.Equal(1.0, aNewResult.Solution![1], THE_TOLERANCE);
            Assert.Equal(2.0, aNewResult.Solution![2], THE_TOLERANCE);
        }
    }
}
