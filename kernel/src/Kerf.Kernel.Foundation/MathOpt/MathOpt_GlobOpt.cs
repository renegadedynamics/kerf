// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_GlobOpt.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Optimisation;

public static partial class MathOpt
{
    /// <summary>Global optimization strategy selection.</summary>
    public enum GlobalStrategy
    {
        /// <summary>Particle Swarm Optimization only.</summary>
        PSO,
        /// <summary>Multiple local optimizations from random starts.</summary>
        MultiStart,
        /// <summary>PSO followed by local refinement.</summary>
        PSOHybrid,
        /// <summary>Differential Evolution algorithm.</summary>
        DifferentialEvolution,
    }

    /// <summary>Configuration for global optimization.</summary>
    public class GlobalConfig : NDimConfig
    {
        /// <summary>Algorithm to use.</summary>
        public GlobalStrategy Strategy = GlobalStrategy.PSOHybrid;
        /// <summary>Population/swarm size.</summary>
        public int NbPopulation = 40;
        /// <summary>Number of random starts (for MultiStart).</summary>
        public int NbStarts = 10;
        /// <summary>Mutation scale (for DE).</summary>
        public double MutationScale = 0.8;
        /// <summary>Crossover probability (for DE).</summary>
        public double CrossoverProb = 0.9;
        /// <summary>Random seed.</summary>
        public uint Seed = 6;
        /// <summary>Max polishing evals per dimension (0 = no polishing).</summary>
        public int PolishBudgetPerDim = 50;

        public GlobalConfig()
            : base(1.0e-8, 200, true)
        {
        }

        public GlobalConfig(GlobalStrategy strategy, int maxIter = 200)
            : base(1.0e-8, maxIter, true)
        {
            Strategy = strategy;
        }

        public GlobalConfig(GlobalConfig other)
            : base(other)
        {
            Strategy = other.Strategy;
            NbPopulation = other.NbPopulation;
            NbStarts = other.NbStarts;
            MutationScale = other.MutationScale;
            CrossoverProb = other.CrossoverProb;
            Seed = other.Seed;
            PolishBudgetPerDim = other.PolishBudgetPerDim;
        }
    }

    /// <summary>Differential Evolution (DE/rand/1 with binomial crossover) for global minimization within bounds.</summary>
    public static VectorResult DifferentialEvolution<Function>(Function func,
                                                               math_Vector lowerBounds,
                                                               math_Vector upperBounds,
                                                               GlobalConfig? config = null)
        where Function : IVectorFunction
    {
        config ??= new GlobalConfig();
        VectorResult result = new();

        int lower = lowerBounds.Lower();
        int upper = lowerBounds.Upper();
        int nbDims = upper - lower + 1;

        if (upperBounds.Length() != nbDims)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        int nbPop = config.NbPopulation;
        if (nbPop < 4)
        {
            // DE requires at least 4 population members for mutation (3 distinct + target)
            result.Status = Status.InvalidInput;
            return result;
        }

        double mutScale = config.MutationScale;
        double crossProb = config.CrossoverProb;

        // Random number generator
        RandomGenerator rng = new(config.Seed);

        // Population: vector of candidate solutions
        NCollection_DynamicArray<math_Vector> population = new();
        math_Vector fitness = new(0, nbPop - 1);

        // Initialize population
        for (int memberIdx = 0; memberIdx < nbPop; ++memberIdx)
        {
            population.Append(new math_Vector(lower, upper));
            for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
            {
                double randVal = rng.NextReal();
                population.ChangeValue(memberIdx)[dimIdx] =
                  lowerBounds[dimIdx] + randVal * (upperBounds[dimIdx] - lowerBounds[dimIdx]);
            }

            if (!func.Value(population.Value(memberIdx), out double fitVal))
            {
                fitVal = double.MaxValue;
            }
            fitness[memberIdx] = fitVal;
        }

        // Find best
        int bestIdx = 0;
        double bestValue = fitness[0];
        for (int memberIdx = 1; memberIdx < nbPop; ++memberIdx)
        {
            if (fitness[memberIdx] < bestValue)
            {
                bestValue = fitness[memberIdx];
                bestIdx = memberIdx;
            }
        }

        // Trial vector
        math_Vector trial = new(lower, upper);

        // Evolution loop
        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            for (int memberIdx = 0; memberIdx < nbPop; ++memberIdx)
            {
                // Select 3 distinct random indices different from current member
                int idxA, idxB, idxC;
                do
                {
                    idxA = (int)(rng.NextReal() * nbPop);
                } while (idxA == memberIdx);
                do
                {
                    idxB = (int)(rng.NextReal() * nbPop);
                } while (idxB == memberIdx || idxB == idxA);
                do
                {
                    idxC = (int)(rng.NextReal() * nbPop);
                } while (idxC == memberIdx || idxC == idxA || idxC == idxB);

                // Mutation and crossover
                int jRand = lower + (int)(rng.NextReal() * nbDims);

                for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
                {
                    if (rng.NextReal() < crossProb || dimIdx == jRand)
                    {
                        // Mutation: DE/rand/1
                        double mutVal =
                          population.Value(idxA)[dimIdx]
                          + mutScale * (population.Value(idxB)[dimIdx] - population.Value(idxC)[dimIdx]);
                        // Clamp to bounds
                        trial[dimIdx] =
                          global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(mutVal, lowerBounds[dimIdx], upperBounds[dimIdx]);
                    }
                    else
                    {
                        trial[dimIdx] = population.Value(memberIdx)[dimIdx];
                    }
                }

                // Selection
                if (!func.Value(trial, out double trialFitness))
                {
                    trialFitness = double.MaxValue;
                }

                if (trialFitness <= fitness[memberIdx])
                {
                    population.ChangeValue(memberIdx).Initialized(trial);
                    fitness[memberIdx] = trialFitness;

                    if (trialFitness < bestValue)
                    {
                        bestValue = trialFitness;
                        bestIdx = memberIdx;
                    }
                }
            }

            // Check convergence
            double maxDiff = 0.0;
            for (int memberIdx = 0; memberIdx < nbPop; ++memberIdx)
            {
                maxDiff = Max(maxDiff, Math.Abs(fitness[memberIdx] - bestValue));
            }

            if (maxDiff < config.Tolerance)
            {
                break;
            }
        }

        // Polish the best solution using coordinate-wise Brent's method
        math_Vector polished = new(population.Value(bestIdx));
        double polishedValue = bestValue;
        if (config.PolishBudgetPerDim > 0)
        {
            PolishCoordinateWise(func,
                                 polished,
                                 ref polishedValue,
                                 lowerBounds,
                                 upperBounds,
                                 config.Tolerance,
                                 config.PolishBudgetPerDim * nbDims,
                                 out int polishEvals);
        }

        result.Status = Status.OK;
        result.Solution = polished;
        result.Value = polishedValue;
        return result;
    }

    /// <summary>
    /// Multi-start local optimization: Powell's method from NbStarts random points within the bounds; the best
    /// (clamped) result is returned.
    /// </summary>
    public static VectorResult MultiStart<Function>(Function func,
                                                    math_Vector lowerBounds,
                                                    math_Vector upperBounds,
                                                    GlobalConfig? config = null)
        where Function : IVectorFunction
    {
        config ??= new GlobalConfig();
        VectorResult result = new();

        int lower = lowerBounds.Lower();
        int upper = lowerBounds.Upper();
        int nbDims = upper - lower + 1;

        if (upperBounds.Length() != nbDims)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        RandomGenerator rng = new(config.Seed);

        math_Vector bestSolution = new(lower, upper);
        double bestValue = double.MaxValue;
        bool found = false;

        // Configure Powell optimization for local refinement
        Config powellConfig = new();
        powellConfig.Tolerance = config.Tolerance;
        powellConfig.XTolerance = config.Tolerance;
        powellConfig.FTolerance = config.Tolerance;
        // OCCT: divides by NbStarts unchecked (undefined in C++ when it is 0; DivideByZeroException here)
        powellConfig.MaxIterations = config.MaxIterations / config.NbStarts;
        if (powellConfig.MaxIterations < 10)
        {
            powellConfig.MaxIterations = 10;
        }

        for (int startIdx = 0; startIdx < config.NbStarts; ++startIdx)
        {
            // Random starting point within bounds
            math_Vector start = new(lower, upper);
            for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
            {
                double randVal = rng.NextReal();
                start[dimIdx] =
                  lowerBounds[dimIdx] + randVal * (upperBounds[dimIdx] - lowerBounds[dimIdx]);
            }

            // Run local optimization from this starting point
            VectorResult localResult = Powell(func, start, powellConfig);

            if (localResult.IsDone && localResult.Value.HasValue)
            {
                // Clamp solution to bounds
                math_Vector sol = new(localResult.Solution!);
                for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
                {
                    sol[dimIdx] =
                      global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(sol[dimIdx], lowerBounds[dimIdx], upperBounds[dimIdx]);
                }

                // Re-evaluate at clamped point
                if (func.Value(sol, out double value) && value < bestValue)
                {
                    bestValue = value;
                    bestSolution.Initialized(sol);
                    found = true;
                }
            }
            else
            {
                // Powell failed, just evaluate at starting point
                if (func.Value(start, out double value) && value < bestValue)
                {
                    bestValue = value;
                    bestSolution.Initialized(start);
                    found = true;
                }
            }
        }

        if (!found)
        {
            result.Status = Status.NumericalError;
            return result;
        }

        result.Status = Status.OK;
        result.Solution = bestSolution;
        result.Value = bestValue;
        return result;
    }

    /// <summary>Unified global optimization interface: runs the strategy chosen in the configuration.</summary>
    public static VectorResult GlobalMinimum<Function>(Function func,
                                                       math_Vector lowerBounds,
                                                       math_Vector upperBounds,
                                                       GlobalConfig? config = null)
        where Function : IVectorFunction
    {
        config ??= new GlobalConfig();
        return GlobalMinimum(func,
                             lowerBounds,
                             upperBounds,
                             config,
                             null,
                             null,
                             null);
    }

    /// <summary>
    /// Unified global optimization interface with PSO-specific options: for the PSO and PSOHybrid strategies,
    /// thePSOConfig (if not null) replaces the PSO configuration derived from theConfig, and theSeeds and
    /// theStats are passed to PSO. Other strategies ignore them.
    /// </summary>
    public static VectorResult GlobalMinimum<Function>(Function func,
                                                       math_Vector lowerBounds,
                                                       math_Vector upperBounds,
                                                       GlobalConfig config,
                                                       PSOConfig? psoConfig,
                                                       NCollection_DynamicArray<PSOSeedParticle>? seeds = null,
                                                       PSOStats? stats = null)
        where Function : IVectorFunction
    {
        switch (config.Strategy)
        {
            case GlobalStrategy.PSO:
            {
                PSOConfig aPSOConfig = new();
                if (psoConfig is not null)
                {
                    aPSOConfig = new PSOConfig(psoConfig);
                }
                else
                {
                    aPSOConfig.NbParticles = config.NbPopulation;
                    aPSOConfig.MaxIterations = config.MaxIterations;
                    aPSOConfig.Tolerance = config.Tolerance;
                    aPSOConfig.Seed = config.Seed;
                    aPSOConfig.PolishBudgetPerDim = config.PolishBudgetPerDim;
                }
                return PSO(func, lowerBounds, upperBounds, aPSOConfig, seeds, stats);
            }

            case GlobalStrategy.MultiStart:
                return MultiStart(func, lowerBounds, upperBounds, config);

            case GlobalStrategy.PSOHybrid:
            {
                // Run PSO first for global exploration
                PSOConfig aPSOConfig = new();
                if (psoConfig is not null)
                {
                    // Honor caller's PSO configuration fully
                    aPSOConfig = new PSOConfig(psoConfig);
                }
                else
                {
                    // Auto-generate PSO config from GlobalConfig:
                    // use half iterations and relaxed tolerance for the PSO phase
                    aPSOConfig.NbParticles = config.NbPopulation;
                    aPSOConfig.MaxIterations = config.MaxIterations / 2;
                    aPSOConfig.Tolerance = config.Tolerance * 10.0;
                    aPSOConfig.Seed = config.Seed;
                    aPSOConfig.PolishBudgetPerDim = config.PolishBudgetPerDim;
                }

                VectorResult psoResult =
                  PSO(func, lowerBounds, upperBounds, aPSOConfig, seeds, stats);

                if (!psoResult.IsDone || psoResult.Solution is null)
                {
                    return psoResult;
                }

                // Local refinement using Powell (gradient-free)
                Config powellConfig = new();
                powellConfig.Tolerance = config.Tolerance;
                powellConfig.XTolerance = config.Tolerance;
                powellConfig.FTolerance = config.Tolerance;
                powellConfig.MaxIterations = config.MaxIterations / 2;

                VectorResult localResult = Powell(func, psoResult.Solution, powellConfig);

                if (localResult.IsDone && localResult.Value.HasValue && psoResult.Value.HasValue
                    && localResult.Value.Value < psoResult.Value.Value)
                {
                    // Clamp to bounds
                    int lower = lowerBounds.Lower();
                    int upper = lowerBounds.Upper();
                    math_Vector sol = new(localResult.Solution!);
                    for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
                    {
                        sol[dimIdx] =
                          global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(sol[dimIdx], lowerBounds[dimIdx], upperBounds[dimIdx]);
                    }
                    localResult.Solution = sol;

                    // Re-evaluate at clamped point
                    if (func.Value(sol, out double value))
                    {
                        localResult.Value = value;
                    }
                    return localResult;
                }

                return psoResult;
            }

            case GlobalStrategy.DifferentialEvolution:
                return DifferentialEvolution(func, lowerBounds, upperBounds, config);

            default:
                VectorResult result = new();
                result.Status = Status.InvalidInput;
                return result;
        }
    }
}
