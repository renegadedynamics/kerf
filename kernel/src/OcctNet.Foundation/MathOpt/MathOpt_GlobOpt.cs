// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_GlobOpt.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

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

        public GlobalConfig(GlobalStrategy theStrategy, int theMaxIter = 200)
            : base(1.0e-8, theMaxIter, true)
        {
            Strategy = theStrategy;
        }

        public GlobalConfig(GlobalConfig theOther)
            : base(theOther)
        {
            Strategy = theOther.Strategy;
            NbPopulation = theOther.NbPopulation;
            NbStarts = theOther.NbStarts;
            MutationScale = theOther.MutationScale;
            CrossoverProb = theOther.CrossoverProb;
            Seed = theOther.Seed;
            PolishBudgetPerDim = theOther.PolishBudgetPerDim;
        }
    }

    /// <summary>Differential Evolution (DE/rand/1 with binomial crossover) for global minimization within bounds.</summary>
    public static VectorResult DifferentialEvolution<Function>(Function theFunc,
                                                               math_Vector theLowerBounds,
                                                               math_Vector theUpperBounds,
                                                               GlobalConfig? theConfig = null)
        where Function : IVectorFunction
    {
        theConfig ??= new GlobalConfig();
        VectorResult aResult = new();

        int aLower = theLowerBounds.Lower();
        int aUpper = theLowerBounds.Upper();
        int aNbDims = aUpper - aLower + 1;

        if (theUpperBounds.Length() != aNbDims)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        int aNbPop = theConfig.NbPopulation;
        if (aNbPop < 4)
        {
            // DE requires at least 4 population members for mutation (3 distinct + target)
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        double aMutScale = theConfig.MutationScale;
        double aCrossProb = theConfig.CrossoverProb;

        // Random number generator
        RandomGenerator aRNG = new(theConfig.Seed);

        // Population: vector of candidate solutions
        NCollection_DynamicArray<math_Vector> aPopulation = new();
        math_Vector aFitness = new(0, aNbPop - 1);

        // Initialize population
        for (int aMemberIdx = 0; aMemberIdx < aNbPop; ++aMemberIdx)
        {
            aPopulation.Append(new math_Vector(aLower, aUpper));
            for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
            {
                double aRandVal = aRNG.NextReal();
                aPopulation.ChangeValue(aMemberIdx)[aDimIdx] =
                  theLowerBounds[aDimIdx] + aRandVal * (theUpperBounds[aDimIdx] - theLowerBounds[aDimIdx]);
            }

            if (!theFunc.Value(aPopulation.Value(aMemberIdx), out double aFitVal))
            {
                aFitVal = double.MaxValue;
            }
            aFitness[aMemberIdx] = aFitVal;
        }

        // Find best
        int aBestIdx = 0;
        double aBestValue = aFitness[0];
        for (int aMemberIdx = 1; aMemberIdx < aNbPop; ++aMemberIdx)
        {
            if (aFitness[aMemberIdx] < aBestValue)
            {
                aBestValue = aFitness[aMemberIdx];
                aBestIdx = aMemberIdx;
            }
        }

        // Trial vector
        math_Vector aTrial = new(aLower, aUpper);

        // Evolution loop
        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            for (int aMemberIdx = 0; aMemberIdx < aNbPop; ++aMemberIdx)
            {
                // Select 3 distinct random indices different from current member
                int anIdxA, anIdxB, anIdxC;
                do
                {
                    anIdxA = (int)(aRNG.NextReal() * aNbPop);
                } while (anIdxA == aMemberIdx);
                do
                {
                    anIdxB = (int)(aRNG.NextReal() * aNbPop);
                } while (anIdxB == aMemberIdx || anIdxB == anIdxA);
                do
                {
                    anIdxC = (int)(aRNG.NextReal() * aNbPop);
                } while (anIdxC == aMemberIdx || anIdxC == anIdxA || anIdxC == anIdxB);

                // Mutation and crossover
                int aJRand = aLower + (int)(aRNG.NextReal() * aNbDims);

                for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
                {
                    if (aRNG.NextReal() < aCrossProb || aDimIdx == aJRand)
                    {
                        // Mutation: DE/rand/1
                        double aMutVal =
                          aPopulation.Value(anIdxA)[aDimIdx]
                          + aMutScale * (aPopulation.Value(anIdxB)[aDimIdx] - aPopulation.Value(anIdxC)[aDimIdx]);
                        // Clamp to bounds
                        aTrial[aDimIdx] =
                          MathUtils.Clamp(aMutVal, theLowerBounds[aDimIdx], theUpperBounds[aDimIdx]);
                    }
                    else
                    {
                        aTrial[aDimIdx] = aPopulation.Value(aMemberIdx)[aDimIdx];
                    }
                }

                // Selection
                if (!theFunc.Value(aTrial, out double aTrialFitness))
                {
                    aTrialFitness = double.MaxValue;
                }

                if (aTrialFitness <= aFitness[aMemberIdx])
                {
                    aPopulation.ChangeValue(aMemberIdx).Initialized(aTrial);
                    aFitness[aMemberIdx] = aTrialFitness;

                    if (aTrialFitness < aBestValue)
                    {
                        aBestValue = aTrialFitness;
                        aBestIdx = aMemberIdx;
                    }
                }
            }

            // Check convergence
            double aMaxDiff = 0.0;
            for (int aMemberIdx = 0; aMemberIdx < aNbPop; ++aMemberIdx)
            {
                aMaxDiff = Max(aMaxDiff, Math.Abs(aFitness[aMemberIdx] - aBestValue));
            }

            if (aMaxDiff < theConfig.Tolerance)
            {
                break;
            }
        }

        // Polish the best solution using coordinate-wise Brent's method
        math_Vector aPolished = new(aPopulation.Value(aBestIdx));
        double aPolishedValue = aBestValue;
        if (theConfig.PolishBudgetPerDim > 0)
        {
            PolishCoordinateWise(theFunc,
                                 aPolished,
                                 ref aPolishedValue,
                                 theLowerBounds,
                                 theUpperBounds,
                                 theConfig.Tolerance,
                                 theConfig.PolishBudgetPerDim * aNbDims,
                                 out int aPolishEvals);
        }

        aResult.Status = Status.OK;
        aResult.Solution = aPolished;
        aResult.Value = aPolishedValue;
        return aResult;
    }

    /// <summary>
    /// Multi-start local optimization: Powell's method from NbStarts random points within the bounds; the best
    /// (clamped) result is returned.
    /// </summary>
    public static VectorResult MultiStart<Function>(Function theFunc,
                                                    math_Vector theLowerBounds,
                                                    math_Vector theUpperBounds,
                                                    GlobalConfig? theConfig = null)
        where Function : IVectorFunction
    {
        theConfig ??= new GlobalConfig();
        VectorResult aResult = new();

        int aLower = theLowerBounds.Lower();
        int aUpper = theLowerBounds.Upper();
        int aNbDims = aUpper - aLower + 1;

        if (theUpperBounds.Length() != aNbDims)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        RandomGenerator aRNG = new(theConfig.Seed);

        math_Vector aBestSolution = new(aLower, aUpper);
        double aBestValue = double.MaxValue;
        bool aFound = false;

        // Configure Powell optimization for local refinement
        Config aPowellConfig = new();
        aPowellConfig.Tolerance = theConfig.Tolerance;
        aPowellConfig.XTolerance = theConfig.Tolerance;
        aPowellConfig.FTolerance = theConfig.Tolerance;
        // OCCT: divides by NbStarts unchecked (undefined in C++ when it is 0; DivideByZeroException here)
        aPowellConfig.MaxIterations = theConfig.MaxIterations / theConfig.NbStarts;
        if (aPowellConfig.MaxIterations < 10)
        {
            aPowellConfig.MaxIterations = 10;
        }

        for (int aStartIdx = 0; aStartIdx < theConfig.NbStarts; ++aStartIdx)
        {
            // Random starting point within bounds
            math_Vector aStart = new(aLower, aUpper);
            for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
            {
                double aRandVal = aRNG.NextReal();
                aStart[aDimIdx] =
                  theLowerBounds[aDimIdx] + aRandVal * (theUpperBounds[aDimIdx] - theLowerBounds[aDimIdx]);
            }

            // Run local optimization from this starting point
            VectorResult aLocalResult = Powell(theFunc, aStart, aPowellConfig);

            if (aLocalResult.IsDone() && aLocalResult.Value.HasValue)
            {
                // Clamp solution to bounds
                math_Vector aSol = new(aLocalResult.Solution!);
                for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
                {
                    aSol[aDimIdx] =
                      MathUtils.Clamp(aSol[aDimIdx], theLowerBounds[aDimIdx], theUpperBounds[aDimIdx]);
                }

                // Re-evaluate at clamped point
                if (theFunc.Value(aSol, out double aValue) && aValue < aBestValue)
                {
                    aBestValue = aValue;
                    aBestSolution.Initialized(aSol);
                    aFound = true;
                }
            }
            else
            {
                // Powell failed, just evaluate at starting point
                if (theFunc.Value(aStart, out double aValue) && aValue < aBestValue)
                {
                    aBestValue = aValue;
                    aBestSolution.Initialized(aStart);
                    aFound = true;
                }
            }
        }

        if (!aFound)
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }

        aResult.Status = Status.OK;
        aResult.Solution = aBestSolution;
        aResult.Value = aBestValue;
        return aResult;
    }

    /// <summary>Unified global optimization interface: runs the strategy chosen in the configuration.</summary>
    public static VectorResult GlobalMinimum<Function>(Function theFunc,
                                                       math_Vector theLowerBounds,
                                                       math_Vector theUpperBounds,
                                                       GlobalConfig? theConfig = null)
        where Function : IVectorFunction
    {
        theConfig ??= new GlobalConfig();
        return GlobalMinimum(theFunc,
                             theLowerBounds,
                             theUpperBounds,
                             theConfig,
                             null,
                             null,
                             null);
    }

    /// <summary>
    /// Unified global optimization interface with PSO-specific options: for the PSO and PSOHybrid strategies,
    /// thePSOConfig (if not null) replaces the PSO configuration derived from theConfig, and theSeeds and
    /// theStats are passed to PSO. Other strategies ignore them.
    /// </summary>
    public static VectorResult GlobalMinimum<Function>(Function theFunc,
                                                       math_Vector theLowerBounds,
                                                       math_Vector theUpperBounds,
                                                       GlobalConfig theConfig,
                                                       PSOConfig? thePSOConfig,
                                                       NCollection_DynamicArray<PSOSeedParticle>? theSeeds = null,
                                                       PSOStats? theStats = null)
        where Function : IVectorFunction
    {
        switch (theConfig.Strategy)
        {
            case GlobalStrategy.PSO:
            {
                PSOConfig aPSOConfig = new();
                if (thePSOConfig is not null)
                {
                    aPSOConfig = new PSOConfig(thePSOConfig);
                }
                else
                {
                    aPSOConfig.NbParticles = theConfig.NbPopulation;
                    aPSOConfig.MaxIterations = theConfig.MaxIterations;
                    aPSOConfig.Tolerance = theConfig.Tolerance;
                    aPSOConfig.Seed = theConfig.Seed;
                    aPSOConfig.PolishBudgetPerDim = theConfig.PolishBudgetPerDim;
                }
                return PSO(theFunc, theLowerBounds, theUpperBounds, aPSOConfig, theSeeds, theStats);
            }

            case GlobalStrategy.MultiStart:
                return MultiStart(theFunc, theLowerBounds, theUpperBounds, theConfig);

            case GlobalStrategy.PSOHybrid:
            {
                // Run PSO first for global exploration
                PSOConfig aPSOConfig = new();
                if (thePSOConfig is not null)
                {
                    // Honor caller's PSO configuration fully
                    aPSOConfig = new PSOConfig(thePSOConfig);
                }
                else
                {
                    // Auto-generate PSO config from GlobalConfig:
                    // use half iterations and relaxed tolerance for the PSO phase
                    aPSOConfig.NbParticles = theConfig.NbPopulation;
                    aPSOConfig.MaxIterations = theConfig.MaxIterations / 2;
                    aPSOConfig.Tolerance = theConfig.Tolerance * 10.0;
                    aPSOConfig.Seed = theConfig.Seed;
                    aPSOConfig.PolishBudgetPerDim = theConfig.PolishBudgetPerDim;
                }

                VectorResult aPSOResult =
                  PSO(theFunc, theLowerBounds, theUpperBounds, aPSOConfig, theSeeds, theStats);

                if (!aPSOResult.IsDone() || aPSOResult.Solution is null)
                {
                    return aPSOResult;
                }

                // Local refinement using Powell (gradient-free)
                Config aPowellConfig = new();
                aPowellConfig.Tolerance = theConfig.Tolerance;
                aPowellConfig.XTolerance = theConfig.Tolerance;
                aPowellConfig.FTolerance = theConfig.Tolerance;
                aPowellConfig.MaxIterations = theConfig.MaxIterations / 2;

                VectorResult aLocalResult = Powell(theFunc, aPSOResult.Solution, aPowellConfig);

                if (aLocalResult.IsDone() && aLocalResult.Value.HasValue && aPSOResult.Value.HasValue
                    && aLocalResult.Value.Value < aPSOResult.Value.Value)
                {
                    // Clamp to bounds
                    int aLower = theLowerBounds.Lower();
                    int aUpper = theLowerBounds.Upper();
                    math_Vector aSol = new(aLocalResult.Solution!);
                    for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
                    {
                        aSol[aDimIdx] =
                          MathUtils.Clamp(aSol[aDimIdx], theLowerBounds[aDimIdx], theUpperBounds[aDimIdx]);
                    }
                    aLocalResult.Solution = aSol;

                    // Re-evaluate at clamped point
                    if (theFunc.Value(aSol, out double aValue))
                    {
                        aLocalResult.Value = aValue;
                    }
                    return aLocalResult;
                }

                return aPSOResult;
            }

            case GlobalStrategy.DifferentialEvolution:
                return DifferentialEvolution(theFunc, theLowerBounds, theUpperBounds, theConfig);

            default:
                VectorResult aResult = new();
                aResult.Status = Status.InvalidInput;
                return aResult;
        }
    }
}
