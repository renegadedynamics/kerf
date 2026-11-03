// Port of: src/FoundationClasses/TKMath/GTests/MathOpt_GlobOpt_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Kernel.Foundation.Tests;

public class MathOpt_GlobOptTest
{
    private const double THE_TOLERANCE = 1.0e-6;
    private const double THE_STOCHASTIC_TOLERANCE = 1.0e-2;

    // Test function structs

    // Sphere function: f(x) = sum(x_i^2)
    private struct SphereFunc : IVectorFunction
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

    // Rastrigin function - highly multimodal
    private struct RastriginFunc : IVectorFunction
    {
        public bool Value(math_Vector theX, out double theF)
        {
            int aN = theX.Length();
            double aPI = Math.PI;
            theF = 10.0 * aN;
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theF += theX[i] * theX[i] - 10.0 * Math.Cos(2.0 * aPI * theX[i]);
            }
            return true;
        }
    }

    // Rosenbrock function
    private struct RosenbrockFunc : IVectorFunction
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
    }

    // Booth function
    private struct BoothFunc : IVectorFunction
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
    }

    // Himmelblau function - has 4 identical local minima: f(x,y) = (x^2 + y - 11)^2 + (x + y^2 - 7)^2
    private struct HimmelblauFunc : IVectorFunction
    {
        public bool Value(math_Vector theX, out double theF)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = aX * aX + aY - 11.0;
            double aT2 = aX + aY * aY - 7.0;
            theF = aT1 * aT1 + aT2 * aT2;
            return true;
        }
    }

    // Goldstein-Price function - complex multimodal
    private struct GoldsteinPriceFunc : IVectorFunction
    {
        public bool Value(math_Vector theX, out double theF)
        {
            double aX = theX[1];
            double aY = theX[2];

            double aA =
              1.0
              + (aX + aY + 1.0) * (aX + aY + 1.0)
                  * (19.0 - 14.0 * aX + 3.0 * aX * aX - 14.0 * aY + 6.0 * aX * aY + 3.0 * aY * aY);

            double aB =
              30.0
              + (2.0 * aX - 3.0 * aY) * (2.0 * aX - 3.0 * aY)
                  * (18.0 - 32.0 * aX + 12.0 * aX * aX + 48.0 * aY - 36.0 * aX * aY + 27.0 * aY * aY);

            theF = aA * aB;
            return true;
        }
    }

    // Old API adapter
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

    // GlobalMinimum unified interface tests

    [Fact]
    public void GlobalMinimum_PSO_Sphere()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfig = new(global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalStrategy.PSO, 100);
        aConfig.NbPopulation = 40;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalMinimum(aFunc, aLower, aUpper, aConfig);

        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void GlobalMinimum_MultiStart_Sphere()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfig = new(global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalStrategy.MultiStart, 100);
        aConfig.NbStarts = 20;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalMinimum(aFunc, aLower, aUpper, aConfig);

        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void GlobalMinimum_PSOHybrid_Booth()
    {
        BoothFunc aFunc = new();
        math_Vector aLower = new(1, 2, -10.0);
        math_Vector aUpper = new(1, 2, 10.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfig = new(global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalStrategy.PSOHybrid, 100);
        aConfig.NbPopulation = 40;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalMinimum(aFunc, aLower, aUpper, aConfig);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(3.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void GlobalMinimum_DE_Sphere()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfig = new(global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalStrategy.DifferentialEvolution, 100);
        aConfig.NbPopulation = 40;
        aConfig.MutationScale = 0.8;
        aConfig.CrossoverProb = 0.9;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalMinimum(aFunc, aLower, aUpper, aConfig);

        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    // Differential Evolution tests

    private static global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig MakeDEConfig(int theMaxIter, int thePop, double theMut, double theCross)
    {
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfig = new();
        aConfig.MaxIterations = theMaxIter;
        aConfig.NbPopulation = thePop;
        aConfig.MutationScale = theMut;
        aConfig.CrossoverProb = theCross;
        return aConfig;
    }

    [Fact]
    public void DifferentialEvolution_Sphere()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.DifferentialEvolution(aFunc, aLower, aUpper, MakeDEConfig(100, 40, 0.8, 0.9));

        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void DifferentialEvolution_Rosenbrock()
    {
        RosenbrockFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.DifferentialEvolution(aFunc, aLower, aUpper, MakeDEConfig(200, 50, 0.8, 0.9));

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Solution![1], 0.1);
        Assert.Equal(1.0, aResult.Solution![2], 0.1);
        Assert.True(aResult.Value!.Value < 0.1);
    }

    [Fact]
    public void DifferentialEvolution_Rastrigin()
    {
        RastriginFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.12);
        math_Vector aUpper = new(1, 2, 5.12);

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.DifferentialEvolution(aFunc, aLower, aUpper, MakeDEConfig(200, 60, 0.5, 0.9));

        Assert.True(aResult.IsDone);
        // Should find global minimum at origin
        Assert.Equal(0.0, aResult.Solution![1], 0.5);
        Assert.Equal(0.0, aResult.Solution![2], 0.5);
        Assert.True(aResult.Value!.Value < 1.0);
    }

    // MultiStart tests

    [Fact]
    public void MultiStart_Sphere()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfig = new();
        aConfig.NbStarts = 20;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.MultiStart(aFunc, aLower, aUpper, aConfig);

        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void MultiStart_Booth()
    {
        BoothFunc aFunc = new();
        math_Vector aLower = new(1, 2, -10.0);
        math_Vector aUpper = new(1, 2, 10.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfig = new();
        aConfig.NbStarts = 30;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.MultiStart(aFunc, aLower, aUpper, aConfig);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Solution![1], 0.5);
        Assert.Equal(3.0, aResult.Solution![2], 0.5);
        Assert.True(aResult.Value!.Value < 1.0);
    }

    // Himmelblau function (multiple global minima)

    [Fact]
    public void Himmelblau_FindsOneMinimum()
    {
        HimmelblauFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfig = new(global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalStrategy.DifferentialEvolution, 200);
        aConfig.NbPopulation = 50;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalMinimum(aFunc, aLower, aUpper, aConfig);

        Assert.True(aResult.IsDone);

        // Himmelblau has 4 global minima, all with f = 0
        // The algorithm should find one of them
        Assert.Equal(0.0, aResult.Value!.Value, THE_STOCHASTIC_TOLERANCE);
    }

    // Goldstein-Price function

    [Fact]
    public void GoldsteinPrice()
    {
        GoldsteinPriceFunc aFunc = new();
        math_Vector aLower = new(1, 2, -2.0);
        math_Vector aUpper = new(1, 2, 2.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfig = new(global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalStrategy.DifferentialEvolution, 200);
        aConfig.NbPopulation = 60;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalMinimum(aFunc, aLower, aUpper, aConfig);

        Assert.True(aResult.IsDone);

        // Global minimum at (0, -1) with f = 3
        Assert.Equal(0.0, aResult.Solution![1], 0.1);
        Assert.Equal(-1.0, aResult.Solution![2], 0.1);
        Assert.Equal(3.0, aResult.Value!.Value, 0.5);
    }

    // Higher dimensional tests

    [Fact]
    public void DifferentialEvolution_Sphere5D()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 5, -5.0);
        math_Vector aUpper = new(1, 5, 5.0);

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.DifferentialEvolution(aFunc, aLower, aUpper, MakeDEConfig(300, 80, 0.7, 0.9));

        Assert.True(aResult.IsDone);
        for (int i = 1; i <= 5; ++i)
        {
            Assert.Equal(0.0, aResult.Solution![i], 0.1);
        }
        Assert.True(aResult.Value!.Value < 0.1);
    }

    // Strategy comparison tests

    [Fact]
    public void StrategyComparison_Rosenbrock()
    {
        RosenbrockFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        // Test PSO
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfigPSO = new(global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalStrategy.PSO, 200);
        aConfigPSO.NbPopulation = 50;

        VectorResult aResultPSO = global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalMinimum(aFunc, aLower, aUpper, aConfigPSO);

        // Test DE
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfigDE = new(global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalStrategy.DifferentialEvolution, 200);
        aConfigDE.NbPopulation = 50;

        VectorResult aResultDE = global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalMinimum(aFunc, aLower, aUpper, aConfigDE);

        Assert.True(aResultPSO.IsDone);
        Assert.True(aResultDE.IsDone);

        // Both should find reasonable solutions
        Assert.True(aResultPSO.Value!.Value < 1.0);
        Assert.True(aResultDE.Value!.Value < 1.0);
    }

    // Reproducibility test

    [Fact]
    public void Reproducibility_DE()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.NbPopulation = 40;

        VectorResult aResult1 = global::Kerf.Engine.Numerics.Optimisation.MathOpt.DifferentialEvolution(aFunc, aLower, aUpper, aConfig);
        VectorResult aResult2 = global::Kerf.Engine.Numerics.Optimisation.MathOpt.DifferentialEvolution(aFunc, aLower, aUpper, aConfig);

        Assert.True(aResult1.IsDone);
        Assert.True(aResult2.IsDone);

        // With same seed, results should be identical
        Assert.Equal(aResult2.Value!.Value, aResult1.Value!.Value);
    }

    // Comparison with old API

    [Fact]
    public void CompareWithOldAPI_Sphere()
    {
        SphereFuncOld anOldFunc = new(2);
        SphereFunc aNewFunc = new();

        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        // Old API
        GlobOptMin anOldSolver = new(anOldFunc, aLower, aUpper, 0.01);
        anOldSolver.Perform();
        math_Vector anOldSol = new(1, 2);
        anOldSolver.Points(1, anOldSol);

        // New API
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfig = new(global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalStrategy.PSOHybrid, 100);
        aConfig.NbPopulation = 40;

        VectorResult aNewResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalMinimum(aNewFunc, aLower, aUpper, aConfig);

        Assert.True(aNewResult.IsDone);

        // Both should find a good solution
        Assert.True(aNewResult.Value!.Value < 0.1);
    }

    // New GlobalMinimum overload tests

    [Fact]
    public void GlobalMinimum_PSO_CustomPSOConfig()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aGlobConfig = new(global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalStrategy.PSO, 200);
        aGlobConfig.NbPopulation = 40;

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aPSOConfig = new();
        aPSOConfig.NbParticles = 60;
        aPSOConfig.MaxIterations = 150;
        aPSOConfig.Tolerance = 1.0e-6;

        aPSOConfig.InertiaSchedule = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOInertiaSchedule.LinearDecay;
        aPSOConfig.OmegaMin = 0.3;

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOStats aStats = new();
        VectorResult aResult =
          global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalMinimum(aFunc, aLower, aUpper, aGlobConfig, aPSOConfig, null, aStats);

        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
        Assert.True(aStats.NbFunctionEvals > 0);
        Assert.True(aStats.NbIterations > 0);
    }

    [Fact]
    public void GlobalMinimum_PSOHybrid_WithSeeds()
    {
        BoothFunc aFunc = new();
        math_Vector aLower = new(1, 2, -10.0);
        math_Vector aUpper = new(1, 2, 10.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aGlobConfig = new(global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalStrategy.PSOHybrid, 100);
        aGlobConfig.NbPopulation = 40;

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aPSOConfig = new();
        aPSOConfig.NbParticles = 40;
        aPSOConfig.MaxIterations = 100;
        aPSOConfig.Tolerance = 1.0e-6;

        aPSOConfig.InitMode = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOInitMode.SeededPlusRandom;

        // Seed near the minimum (1, 3)
        math_Vector aSeedPos = MathOpt_NDim_TestFunctions.Vec(1.5, 2.5);
        List<global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOSeedParticle> aSeeds = new();
        aSeeds.Add(new global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOSeedParticle(aSeedPos));

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOStats aStats = new();
        VectorResult aResult =
          global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalMinimum(aFunc, aLower, aUpper, aGlobConfig, aPSOConfig, aSeeds, aStats);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(3.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
        Assert.True(aStats.NbFunctionEvals > 0);
    }

    [Fact]
    public void GlobalMinimum_NonPSOStrategies_UnchangedWithPSOOptions()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        // MultiStart - PSO params should be ignored
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfigMS = new(global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalStrategy.MultiStart, 100);
        aConfigMS.NbStarts = 20;

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aPSOConfig = new();
        aPSOConfig.NbParticles = 100;

        VectorResult aResultMS = global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalMinimum(aFunc, aLower, aUpper, aConfigMS, aPSOConfig);
        Assert.True(aResultMS.IsDone);
        Assert.Equal(0.0, aResultMS.Value!.Value, THE_TOLERANCE);

        // DE - PSO params should be ignored
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalConfig aConfigDE = new(global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalStrategy.DifferentialEvolution, 100);
        aConfigDE.NbPopulation = 40;

        VectorResult aResultDE = global::Kerf.Engine.Numerics.Optimisation.MathOpt.GlobalMinimum(aFunc, aLower, aUpper, aConfigDE, aPSOConfig);
        Assert.True(aResultDE.IsDone);
        Assert.Equal(0.0, aResultDE.Value!.Value, THE_TOLERANCE);
    }
}
