// Port of: src/FoundationClasses/TKMath/GTests/MathOpt_PSO_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Kernel.Foundation.Tests;

public class MathOpt_PSOTest
{
    private const double THE_TOLERANCE = 1.0e-6;
    private const double THE_STOCHASTIC_TOLERANCE = 1.0e-3;

    // Test function structs for new API

    // Sphere function: f(x) = sum(x_i^2), global minimum at origin with f = 0
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

    // Rastrigin function: f(x) = 10n + sum(x_i^2 - 10*cos(2*pi*x_i)), global minimum at origin
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

    // Ackley function - multimodal with global minimum at origin
    private struct AckleyFunc : IVectorFunction
    {
        public bool Value(math_Vector theX, out double theF)
        {
            int aN = theX.Length();
            double aPI = Math.PI;
            double aSum1 = 0.0;
            double aSum2 = 0.0;

            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                aSum1 += theX[i] * theX[i];
                aSum2 += Math.Cos(2.0 * aPI * theX[i]);
            }

            theF = -20.0 * Math.Exp(-0.2 * Math.Sqrt(aSum1 / aN)) - Math.Exp(aSum2 / aN) + 20.0 + Math.E;
            return true;
        }
    }

    // Rosenbrock function: f(x,y) = 100*(y-x^2)^2 + (1-x)^2, minimum at (1, 1) with f = 0
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

    // Booth function: f(x,y) = (x + 2y - 7)^2 + (2x + y - 5)^2, minimum at (1, 3) with f = 0
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

    private static global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig MakeConfig(int theNbParticles, int theMaxIter, double theTolerance)
    {
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfig = new();
        aConfig.NbParticles = theNbParticles;
        aConfig.MaxIterations = theMaxIter;
        aConfig.Tolerance = theTolerance;
        return aConfig;
    }

    // Basic PSO tests

    [Fact]
    public void Sphere2D()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, MakeConfig(50, 200, 1.0e-6));

        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Sphere3D()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 3, -5.0);
        math_Vector aUpper = new(1, 3, 5.0);

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, MakeConfig(40, 100, 1.0e-6));

        Assert.True(aResult.IsDone);
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(0.0, aResult.Solution![i], THE_TOLERANCE);
        }
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Booth()
    {
        BoothFunc aFunc = new();
        math_Vector aLower = new(1, 2, -10.0);
        math_Vector aUpper = new(1, 2, 10.0);

        // PSO with larger search space needs more exploration
        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, MakeConfig(60, 200, 1.0e-6));

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Solution![1], THE_STOCHASTIC_TOLERANCE);
        Assert.Equal(3.0, aResult.Solution![2], THE_STOCHASTIC_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_STOCHASTIC_TOLERANCE);
    }

    [Fact]
    public void Rosenbrock()
    {
        RosenbrockFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, MakeConfig(50, 200, 1.0e-6));

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Solution![1], 0.1);
        Assert.Equal(1.0, aResult.Solution![2], 0.1);
        Assert.True(aResult.Value!.Value < 0.1);
    }

    // Multimodal function tests

    [Fact]
    public void Rastrigin2D()
    {
        RastriginFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.12);
        math_Vector aUpper = new(1, 2, 5.12);

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, MakeConfig(60, 200, 1.0e-6));

        Assert.True(aResult.IsDone);
        // PSO should find the global minimum at origin
        Assert.Equal(0.0, aResult.Solution![1], 0.5);
        Assert.Equal(0.0, aResult.Solution![2], 0.5);
        Assert.True(aResult.Value!.Value < 1.0);
    }

    [Fact]
    public void Ackley2D()
    {
        AckleyFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, MakeConfig(60, 200, 1.0e-6));

        Assert.True(aResult.IsDone);
        // Global minimum at origin with f = 0
        Assert.Equal(0.0, aResult.Solution![1], 0.5);
        Assert.Equal(0.0, aResult.Solution![2], 0.5);
        Assert.True(aResult.Value!.Value < 1.0);
    }

    // Parameter sensitivity tests

    [Fact]
    public void ParameterSensitivity_Omega()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        // Test different inertia weights
        double[] aOmegas = [0.4, 0.7, 0.9];

        foreach (double aOmega in aOmegas)
        {
            global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfig = MakeConfig(30, 100, 1.0e-6);
            aConfig.Omega = aOmega;

            VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfig);

            Assert.True(aResult.IsDone, $"Failed with omega = {aOmega}");
            Assert.True(aResult.Value!.Value < 0.01, $"Failed with omega = {aOmega}");
        }
    }

    [Fact]
    public void ParameterSensitivity_SwarmSize()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        // Test different swarm sizes
        int[] aSizes = [10, 30, 50, 100];

        foreach (int aSize in aSizes)
        {
            VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, MakeConfig(aSize, 100, 1.0e-6));

            Assert.True(aResult.IsDone, $"Failed with swarm size = {aSize}");
            Assert.True(aResult.Value!.Value < 0.01, $"Failed with swarm size = {aSize}");
        }
    }

    // Reproducibility test

    [Fact]
    public void Reproducibility()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfig = MakeConfig(30, 50, 1.0e-6);

        VectorResult aResult1 = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfig);
        VectorResult aResult2 = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfig);

        Assert.True(aResult1.IsDone);
        Assert.True(aResult2.IsDone);

        // With same seed, results should be identical
        Assert.Equal(aResult2.Value!.Value, aResult1.Value!.Value);
        Assert.Equal(aResult2.Solution![1], aResult1.Solution![1]);
        Assert.Equal(aResult2.Solution![2], aResult1.Solution![2]);
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
        math_Vector aStep = new(1, 2, 0.5);
        Pso anOldSolver = new(anOldFunc, aLower, aUpper, aStep, 30, 100);
        math_Vector anOldOutPnt = new(1, 2);
        anOldSolver.Perform(aStep, out double anOldValue, anOldOutPnt);
        _ = anOldValue;

        // New API
        VectorResult aNewResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aNewFunc, aLower, aUpper, MakeConfig(30, 100, 1.0e-8));

        Assert.True(aNewResult.IsDone);

        // Both should find a good solution
        Assert.True(aNewResult.Value!.Value < 0.01);
    }

    // Higher dimensional test

    [Fact]
    public void Sphere5D()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 5, -5.0);
        math_Vector aUpper = new(1, 5, 5.0);

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, MakeConfig(60, 200, 1.0e-6));

        Assert.True(aResult.IsDone);
        for (int i = 1; i <= 5; ++i)
        {
            Assert.Equal(0.0, aResult.Solution![i], 0.1);
        }
        Assert.True(aResult.Value!.Value < 0.1);
    }

    // Asymmetric bounds test

    [Fact]
    public void AsymmetricBounds()
    {
        BoothFunc aFunc = new();

        // Asymmetric bounds that still contain the minimum (1, 3)
        math_Vector aLower = MathOpt_NDim_TestFunctions.Vec(-5.0, 0.0);
        math_Vector aUpper = MathOpt_NDim_TestFunctions.Vec(5.0, 10.0);

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, MakeConfig(40, 100, 1.0e-6));

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Solution![1], THE_STOCHASTIC_TOLERANCE);
        Assert.Equal(3.0, aResult.Solution![2], THE_STOCHASTIC_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_STOCHASTIC_TOLERANCE);
    }

    // Seeded initialization tests

    [Fact]
    public void SeededInit_Sphere2D_ImprovesEvalCount()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        // Unseeded run
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfig = MakeConfig(40, 200, 1.0e-6);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOStats aStatsUnseeded = new();
        VectorResult aResultUnseeded = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfig, null, aStatsUnseeded);
        Assert.True(aResultUnseeded.IsDone);

        // Seeded run - seed near the origin
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfigSeeded = new(aConfig);
        aConfigSeeded.InitMode = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOInitMode.SeededPlusRandom;

        math_Vector aSeedPos = new(1, 2, 0.0);
        aSeedPos[1] = 0.1;
        aSeedPos[2] = -0.1;
        List<global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOSeedParticle> aSeeds = new();
        aSeeds.Add(new global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOSeedParticle(aSeedPos));

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOStats aStatsSeeded = new();
        VectorResult aResultSeeded = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfigSeeded, aSeeds, aStatsSeeded);
        Assert.True(aResultSeeded.IsDone);

        // Seeded should converge at least as well
        Assert.True(aResultSeeded.Value!.Value <= aResultUnseeded.Value!.Value + 0.01);
        // Stats should be populated
        Assert.True(aStatsSeeded.NbFunctionEvals > 0);
        Assert.True(aStatsSeeded.NbIterations > 0);
    }

    [Fact]
    public void SeededInit_Rastrigin2D_StableQuality()
    {
        RastriginFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.12);
        math_Vector aUpper = new(1, 2, 5.12);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfig = MakeConfig(60, 200, 1.0e-6);
        aConfig.InitMode = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOInitMode.SeededPlusRandom;

        // Seed near global minimum
        math_Vector aSeedPos = new(1, 2, 0.0);
        List<global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOSeedParticle> aSeeds = new();
        aSeeds.Add(new global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOSeedParticle(aSeedPos, 0.0));

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfig, aSeeds);

        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Solution![1], 0.5);
        Assert.Equal(0.0, aResult.Solution![2], 0.5);
        Assert.True(aResult.Value!.Value < 1.0);
    }

    // Boundary mode tests

    [Fact]
    public void BoundaryMode_Reflect_NearBoundMinimum()
    {
        BoothFunc aFunc = new();

        // Tight bounds around minimum (1, 3)
        math_Vector aLower = MathOpt_NDim_TestFunctions.Vec(0.0, 2.0);
        math_Vector aUpper = MathOpt_NDim_TestFunctions.Vec(2.0, 4.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfig = MakeConfig(40, 200, 1.0e-6);
        aConfig.BoundaryMode = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOBoundaryMode.Reflect;

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOStats aStats = new();
        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfig, null, aStats);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.Solution![1], 0.1);
        Assert.Equal(3.0, aResult.Solution![2], 0.1);
        Assert.True(aResult.Value!.Value < 0.1);
        // Tight bounds should trigger boundary corrections
        Assert.True(aStats.NbBoundaryCorrections > 0);
    }

    // Restart tests

    [Fact]
    public void Restart_Ackley_RecoversAfterStagnation()
    {
        AckleyFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfig = MakeConfig(30, 300, 1.0e-8);
        aConfig.RestartFraction = 0.5;
        aConfig.NoImproveIters = 5;

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOStats aStats = new();
        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfig, null, aStats);

        Assert.True(aResult.IsDone);
        Assert.True(aResult.Value!.Value < 1.0);
        // With tight convergence and restart enabled, stats should show activity
        Assert.True(aStats.NbFunctionEvals > 0);
        Assert.True(aStats.NbRestarts > 0);
        Assert.True(aStats.NbStagnationEvents > 0);
    }

    // Target value early stop

    [Fact]
    public void TargetValue_EarlyStop()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        // Run without target value
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfigFull = MakeConfig(40, 200, 1.0e-10); // very tight - force many iterations

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOStats aStatsFull = new();
        VectorResult aResultFull = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfigFull, null, aStatsFull);
        Assert.True(aResultFull.IsDone);

        // Run with target value - should stop early
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfigTarget = new(aConfigFull);
        aConfigTarget.TargetValue = 0.1;

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOStats aStatsTarget = new();
        VectorResult aResultTarget = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfigTarget, null, aStatsTarget);
        Assert.True(aResultTarget.IsDone);

        // Target run should use fewer iterations
        Assert.True(aStatsTarget.NbIterations <= aStatsFull.NbIterations);
        Assert.True(aResultTarget.Value!.Value <= 0.1);
    }

    // Inertia schedule tests

    [Fact]
    public void InertiaSchedule_LinearDecay_NonRegression()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfig = MakeConfig(40, 200, 1.0e-6);
        aConfig.InertiaSchedule = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOInertiaSchedule.LinearDecay;
        aConfig.OmegaMin = 0.3;

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfig);

        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    // Stats reproducibility

    [Fact]
    public void Stats_Reproducibility_FixedSeed()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfig = MakeConfig(30, 50, 1.0e-6);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOStats aStats1 = new();
        VectorResult aResult1 = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfig, null, aStats1);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOStats aStats2 = new();
        VectorResult aResult2 = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfig, null, aStats2);

        Assert.True(aResult1.IsDone);
        Assert.True(aResult2.IsDone);

        // With same seed, stats should be identical
        Assert.Equal(aStats2.NbFunctionEvals, aStats1.NbFunctionEvals);
        Assert.Equal(aStats2.NbIterations, aStats1.NbIterations);
        Assert.Equal(aStats2.NbBoundaryCorrections, aStats1.NbBoundaryCorrections);
        Assert.Equal(aStats2.InitialBest, aStats1.InitialBest);
        Assert.Equal(aStats2.FinalBest, aStats1.FinalBest);
        Assert.Equal(aResult2.Value!.Value, aResult1.Value!.Value);
    }

    // Polishing precision tests

    [Fact]
    public void Polishing_Sphere2D_ReachesHighPrecision()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, MakeConfig(30, 50, 1.0e-10));

        Assert.True(aResult.IsDone);
        // After polishing, component precision should be better than 1e-8
        Assert.Equal(0.0, aResult.Solution![1], 1.0e-8);
        Assert.Equal(0.0, aResult.Solution![2], 1.0e-8);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-14);
    }

    // SeededOnly validation test

    [Fact]
    public void SeededOnly_NoSeeds_ReturnsInvalidInput()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfig = new();
        aConfig.NbParticles = 20;
        aConfig.MaxIterations = 50;
        aConfig.InitMode = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOInitMode.SeededOnly;

        // No seeds provided
        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfig);

        Assert.False(aResult.IsDone);
        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput, aResult.Status);
    }

    // Clamped seed re-evaluation test

    [Fact]
    public void ClampedSeed_ReEvaluates_StaleValue()
    {
        SphereFunc aFunc = new();
        math_Vector aLower = new(1, 2, -5.0);
        math_Vector aUpper = new(1, 2, 5.0);

        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOConfig aConfig = MakeConfig(10, 100, 1.0e-8);
        aConfig.InitMode = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOInitMode.SeededPlusRandom;

        // Seed at out-of-bounds position with incorrect stale value
        math_Vector aSeedPos = new(1, 2);
        aSeedPos[1] = 10.0;  // out of bounds (will be clamped to 5.0)
        aSeedPos[2] = -10.0; // out of bounds (will be clamped to -5.0)

        // Provide a stale value of 0.0 which is wrong for (5, -5)
        List<global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOSeedParticle> aSeeds = new();
        global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSOSeedParticle aSeed = new(aSeedPos, 0.0);
        aSeeds.Add(aSeed);

        VectorResult aResult = global::Kerf.Engine.Numerics.Optimisation.MathOpt.PSO(aFunc, aLower, aUpper, aConfig, aSeeds);

        Assert.True(aResult.IsDone);
        // The result should be correct (near origin), not misled by the stale value
        Assert.Equal(0.0, aResult.Solution![1], THE_STOCHASTIC_TOLERANCE);
        Assert.Equal(0.0, aResult.Solution![2], THE_STOCHASTIC_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_STOCHASTIC_TOLERANCE);
    }
}
