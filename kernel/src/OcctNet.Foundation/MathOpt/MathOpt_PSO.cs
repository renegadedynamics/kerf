// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_PSO.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

public static partial class MathOpt
{
    /// <summary>Initialization mode for PSO particles.</summary>
    public enum PSOInitMode
    {
        /// <summary>All particles randomly initialized (default).</summary>
        RandomOnly,
        /// <summary>Initialize from seed particles only.</summary>
        SeededOnly,
        /// <summary>Seed particles + remaining random.</summary>
        SeededPlusRandom,
    }

    /// <summary>Boundary handling mode for particles leaving the search space.</summary>
    public enum PSOBoundaryMode
    {
        /// <summary>Clamp to bound, reverse velocity with -0.5 damping (default).</summary>
        Clamp,
        /// <summary>Reflect coordinate, damp velocity by -0.5.</summary>
        Reflect,
        /// <summary>Periodic wrap into bounds.</summary>
        Wrap,
    }

    /// <summary>Inertia weight schedule.</summary>
    public enum PSOInertiaSchedule
    {
        /// <summary>Fixed omega (default).</summary>
        Constant,
        /// <summary>Linear decay from Omega to OmegaMin over iterations.</summary>
        LinearDecay,
    }

    /// <summary>Seed particle for PSO initialization.</summary>
    /// <remarks>A C++ aggregate kept as a struct with public fields. Its constructors copy the position, as C++
    /// does; copying the struct itself shares the vectors (C++ copies them).</remarks>
    public struct PSOSeedParticle
    {
        /// <summary>Initial position (will be clamped to bounds).</summary>
        public math_Vector Position;
        /// <summary>Known function value (null = will be evaluated).</summary>
        public double? Value;
        /// <summary>Initial velocity (null = generate bounded random).</summary>
        public math_Vector? Velocity;

        public PSOSeedParticle(math_Vector thePos)
        {
            Position = new math_Vector(thePos);
        }

        public PSOSeedParticle(math_Vector thePos, double theValue)
        {
            Position = new math_Vector(thePos);
            Value = theValue;
        }
    }

    /// <summary>Statistics collected during PSO execution.</summary>
    /// <remarks>A class: PSO fills the one the caller passes, where C++ writes through an optional pointer.</remarks>
    public sealed class PSOStats
    {
        /// <summary>Total function evaluations.</summary>
        public int NbFunctionEvals = 0;
        /// <summary>Iterations performed.</summary>
        public int NbIterations = 0;
        /// <summary>Boundary corrections applied.</summary>
        public int NbBoundaryCorrections = 0;
        /// <summary>Times stagnation was detected.</summary>
        public int NbStagnationEvents = 0;
        /// <summary>Restarts performed.</summary>
        public int NbRestarts = 0;
        /// <summary>Best value after initialization.</summary>
        public double InitialBest = double.MaxValue;
        /// <summary>Best value at termination.</summary>
        public double FinalBest = double.MaxValue;

        public PSOStats()
        {
        }

        public PSOStats(PSOStats theOther)
        {
            Assign(theOther);
        }

        /// <summary>C++'s assignment: copies all the values.</summary>
        public void Assign(PSOStats theOther)
        {
            NbFunctionEvals = theOther.NbFunctionEvals;
            NbIterations = theOther.NbIterations;
            NbBoundaryCorrections = theOther.NbBoundaryCorrections;
            NbStagnationEvents = theOther.NbStagnationEvents;
            NbRestarts = theOther.NbRestarts;
            InitialBest = theOther.InitialBest;
            FinalBest = theOther.FinalBest;
        }
    }

    /// <summary>Configuration for Particle Swarm Optimization.</summary>
    public class PSOConfig : NDimConfig
    {
        /// <summary>Number of particles in the swarm.</summary>
        public int NbParticles = 40;
        /// <summary>Inertia weight (velocity decay).</summary>
        public double Omega = 0.7;
        /// <summary>Personal best attraction coefficient.</summary>
        public double PhiPersonal = 1.5;
        /// <summary>Global best attraction coefficient.</summary>
        public double PhiGlobal = 1.5;
        /// <summary>Max velocity as fraction of search space.</summary>
        public double VelocityClamp = 0.5;
        /// <summary>Random seed for reproducibility.</summary>
        public uint Seed = 6;

        public PSOInitMode InitMode = PSOInitMode.RandomOnly;
        public PSOBoundaryMode BoundaryMode = PSOBoundaryMode.Clamp;
        public PSOInertiaSchedule InertiaSchedule = PSOInertiaSchedule.Constant;
        /// <summary>Min inertia for LinearDecay.</summary>
        public double OmegaMin = 0.4;
        /// <summary>Minimum iterations before convergence.</summary>
        public int MinIterations = 0;
        /// <summary>Early stop if best &lt;= target (null = disabled).</summary>
        public double? TargetValue;
        /// <summary>Stagnation tolerance (0 = use Tolerance).</summary>
        public double NoImproveTol = 0.0;
        /// <summary>Stagnation iteration threshold.</summary>
        public int NoImproveIters = 10;
        /// <summary>Fraction of particles to reinitialize (0 = no restarts).</summary>
        public double RestartFraction = 0.0;
        /// <summary>Maximum restart count (0 = unlimited when fraction &gt; 0).</summary>
        public int MaxRestarts = 0;
        /// <summary>Max polishing evals per dimension (0 = no polishing).</summary>
        public int PolishBudgetPerDim = 50;

        public PSOConfig()
            : base(1.0e-8, 100, true)
        {
        }

        public PSOConfig(int theNbParticles, int theMaxIter = 100, double theTolerance = 1.0e-8)
            : base(theTolerance, theMaxIter, true)
        {
            NbParticles = theNbParticles;
        }

        public PSOConfig(PSOConfig theOther)
            : base(theOther)
        {
            NbParticles = theOther.NbParticles;
            Omega = theOther.Omega;
            PhiPersonal = theOther.PhiPersonal;
            PhiGlobal = theOther.PhiGlobal;
            VelocityClamp = theOther.VelocityClamp;
            Seed = theOther.Seed;
            InitMode = theOther.InitMode;
            BoundaryMode = theOther.BoundaryMode;
            InertiaSchedule = theOther.InertiaSchedule;
            OmegaMin = theOther.OmegaMin;
            MinIterations = theOther.MinIterations;
            TargetValue = theOther.TargetValue;
            NoImproveTol = theOther.NoImproveTol;
            NoImproveIters = theOther.NoImproveIters;
            RestartFraction = theOther.RestartFraction;
            MaxRestarts = theOther.MaxRestarts;
            PolishBudgetPerDim = theOther.PolishBudgetPerDim;
        }
    }

    /// <summary>
    /// Coordinate-wise polishing of thePosition/theValue (updated in place) by Brent's 1D minimization along each
    /// coordinate within the bounds, using at most theMaxPolishEvals evaluations (counted in theEvalCount).
    /// </summary>
    public static void PolishCoordinateWise<Function>(Function theFunc,
                                                      math_Vector thePosition,
                                                      ref double theValue,
                                                      math_Vector theLowerBounds,
                                                      math_Vector theUpperBounds,
                                                      double theTolerance,
                                                      int theMaxPolishEvals,
                                                      out int theEvalCount)
        where Function : IVectorFunction
    {
        theEvalCount = 0;

        int aLower = thePosition.Lower();
        int aUpper = thePosition.Upper();
        int aNbDims = aUpper - aLower + 1;

        // Fair per-coordinate cap: total budget split across dimensions and passes
        int aNbPasses = (aNbDims > 1) ? 2 : 1;
        int aPerCoordLimit = Max(1, theMaxPolishEvals / (aNbDims * aNbPasses));

        for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
        {
            if (theEvalCount >= theMaxPolishEvals)
            {
                break;
            }

            int aMaxIter = Min(aPerCoordLimit, theMaxPolishEvals - theEvalCount);
            BrentAlongCoordinate(theFunc,
                                 thePosition,
                                 aDimIdx,
                                 theLowerBounds[aDimIdx],
                                 theUpperBounds[aDimIdx],
                                 ref theValue,
                                 theTolerance,
                                 aMaxIter,
                                 ref theEvalCount);
        }

        // Second pass for non-separable functions - dimensions may interact
        if (theEvalCount < theMaxPolishEvals && aNbDims > 1)
        {
            for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
            {
                if (theEvalCount >= theMaxPolishEvals)
                {
                    break;
                }

                int aMaxIter = Min(aPerCoordLimit, theMaxPolishEvals - theEvalCount);
                BrentAlongCoordinate(theFunc,
                                     thePosition,
                                     aDimIdx,
                                     theLowerBounds[aDimIdx],
                                     theUpperBounds[aDimIdx],
                                     ref theValue,
                                     theTolerance,
                                     aMaxIter,
                                     ref theEvalCount);
            }
        }
    }

    /// <summary>A particle of the swarm (C++'s struct local to PSO, held by reference in the swarm array).</summary>
    private sealed class PSO_Particle
    {
        public math_Vector Position;
        public math_Vector Velocity;
        public math_Vector BestPosition;
        public double BestValue;
        public double CurrentValue;

        public PSO_Particle(int theLower, int theUpper)
        {
            Position = new math_Vector(theLower, theUpper);
            Velocity = new math_Vector(theLower, theUpper);
            BestPosition = new math_Vector(theLower, theUpper);
            BestValue = double.MaxValue;
            CurrentValue = double.MaxValue;
        }
    }

    /// <summary>
    /// Particle Swarm Optimization for global minimization within box bounds, optionally initialized from seed
    /// particles (theSeeds, may be null) and reporting statistics into theStats (may be null). Stochastic, but
    /// reproducible for a given PSOConfig.Seed. The best point is polished coordinate-wise at the end.
    /// </summary>
    public static VectorResult PSO<Function>(Function theFunc,
                                             math_Vector theLowerBounds,
                                             math_Vector theUpperBounds,
                                             PSOConfig theConfig,
                                             NCollection_DynamicArray<PSOSeedParticle>? theSeeds,
                                             PSOStats? theStats = null)
        where Function : IVectorFunction
    {
        VectorResult aResult = new();

        int aLower = theLowerBounds.Lower();
        int aUpper = theLowerBounds.Upper();
        int aNbDims = aUpper - aLower + 1;

        // Check dimensions
        // OCCT: only the length of the upper bounds is checked; they are then indexed with the lower bounds' indexes.
        if (theUpperBounds.Length() != aNbDims)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        // Check bounds
        for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
        {
            if (theLowerBounds[aDimIdx] >= theUpperBounds[aDimIdx])
            {
                aResult.Status = Status.InvalidInput;
                return aResult;
            }
        }

        int aNbParticles = theConfig.NbParticles;
        if (aNbParticles <= 0)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        // Initialize stats
        PSOStats aLocalStats = new();

        NCollection_DynamicArray<PSO_Particle> aSwarm = new();
        for (int aPartIdx = 0; aPartIdx < aNbParticles; ++aPartIdx)
        {
            aSwarm.Append(new PSO_Particle(aLower, aUpper));
        }

        // Random number generator
        RandomGenerator aRNG = new(theConfig.Seed);

        // Compute velocity limits
        math_Vector aVelMax = new(aLower, aUpper);
        math_Vector aRange = new(aLower, aUpper);
        for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
        {
            aRange[aDimIdx] = theUpperBounds[aDimIdx] - theLowerBounds[aDimIdx];
            aVelMax[aDimIdx] = theConfig.VelocityClamp * aRange[aDimIdx];
        }

        // Initialize particles
        math_Vector aGlobalBest = new(aLower, aUpper);
        double aGlobalBestValue = double.MaxValue;

        // Determine how many particles are seeded
        int aNbSeeds = (theSeeds is not null) ? theSeeds.Length() : 0;
        int aSeeded = 0;

        // Validate seed dimensions and index ranges
        if (aNbSeeds > 0)
        {
            for (int aSeedIdx = 0; aSeedIdx < aNbSeeds; ++aSeedIdx)
            {
                PSOSeedParticle aSeed = theSeeds!.Value(aSeedIdx);
                if (aSeed.Position.Lower() != aLower || aSeed.Position.Upper() != aUpper)
                {
                    aResult.Status = Status.InvalidInput;
                    return aResult;
                }
                if (aSeed.Velocity is not null
                    && (aSeed.Velocity.Lower() != aLower || aSeed.Velocity.Upper() != aUpper))
                {
                    aResult.Status = Status.InvalidInput;
                    return aResult;
                }
            }
        }

        // Validate SeededOnly requires at least one seed
        if (theConfig.InitMode == PSOInitMode.SeededOnly && aNbSeeds == 0)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        if (aNbSeeds > 0
            && (theConfig.InitMode == PSOInitMode.SeededPlusRandom
                || theConfig.InitMode == PSOInitMode.SeededOnly))
        {
            // Initialize from seeds
            int aNbDirect = Min(aNbSeeds, aNbParticles);
            for (int aSeedIdx = 0; aSeedIdx < aNbDirect; ++aSeedIdx)
            {
                PSO_Particle aParticle = aSwarm.ChangeValue(aSeedIdx);
                PSOSeedParticle aSeed = theSeeds!.Value(aSeedIdx);

                // Clamp seed position to bounds, track if any coordinate was clamped
                bool aWasClamped = false;
                for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
                {
                    double aClamped = MathUtils.Clamp(aSeed.Position[aDimIdx],
                                                      theLowerBounds[aDimIdx],
                                                      theUpperBounds[aDimIdx]);
                    if (aClamped != aSeed.Position[aDimIdx])
                    {
                        aWasClamped = true;
                    }
                    aParticle.Position[aDimIdx] = aClamped;
                }

                // Use seed velocity or generate random
                if (aSeed.Velocity is not null)
                {
                    math_Vector aSeedVel = aSeed.Velocity;
                    for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
                    {
                        aParticle.Velocity[aDimIdx] =
                          MathUtils.Clamp(aSeedVel[aDimIdx], -aVelMax[aDimIdx], aVelMax[aDimIdx]);
                    }
                }
                else
                {
                    for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
                    {
                        aParticle.Velocity[aDimIdx] = (2.0 * aRNG.NextReal() - 1.0) * aVelMax[aDimIdx];
                    }
                }

                // Use known value only if position was not clamped; otherwise re-evaluate
                if (aSeed.Value.HasValue && !aWasClamped)
                {
                    aParticle.CurrentValue = aSeed.Value.Value;
                }
                else
                {
                    if (!theFunc.Value(aParticle.Position, out aParticle.CurrentValue))
                    {
                        aParticle.CurrentValue = double.MaxValue;
                    }
                    ++aLocalStats.NbFunctionEvals;
                }

                aParticle.BestPosition.Initialized(aParticle.Position);
                aParticle.BestValue = aParticle.CurrentValue;

                if (aParticle.BestValue < aGlobalBestValue)
                {
                    aGlobalBestValue = aParticle.BestValue;
                    aGlobalBest.Initialized(aParticle.BestPosition);
                }
            }
            aSeeded = aNbDirect;

            // For SeededOnly: if seeds < NbParticles, jitter best seeds to fill
            if (theConfig.InitMode == PSOInitMode.SeededOnly && aSeeded < aNbParticles)
            {
                for (int aPartIdx = aSeeded; aPartIdx < aNbParticles; ++aPartIdx)
                {
                    PSO_Particle aParticle = aSwarm.ChangeValue(aPartIdx);
                    // Pick a seed to jitter (round-robin)
                    int aSrcIdx = aPartIdx % aSeeded;
                    PSOSeedParticle aSrc = theSeeds!.Value(aSrcIdx);

                    for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
                    {
                        double aJitter = (2.0 * aRNG.NextReal() - 1.0) * 0.1 * aRange[aDimIdx];
                        aParticle.Position[aDimIdx] = MathUtils.Clamp(aSrc.Position[aDimIdx] + aJitter,
                                                                      theLowerBounds[aDimIdx],
                                                                      theUpperBounds[aDimIdx]);
                        aParticle.Velocity[aDimIdx] = (2.0 * aRNG.NextReal() - 1.0) * aVelMax[aDimIdx];
                    }

                    if (!theFunc.Value(aParticle.Position, out aParticle.CurrentValue))
                    {
                        aParticle.CurrentValue = double.MaxValue;
                    }
                    ++aLocalStats.NbFunctionEvals;

                    aParticle.BestPosition.Initialized(aParticle.Position);
                    aParticle.BestValue = aParticle.CurrentValue;

                    if (aParticle.BestValue < aGlobalBestValue)
                    {
                        aGlobalBestValue = aParticle.BestValue;
                        aGlobalBest.Initialized(aParticle.BestPosition);
                    }
                }
                aSeeded = aNbParticles;
            }
        }

        // Fill remaining particles randomly
        for (int aPartIdx = aSeeded; aPartIdx < aNbParticles; ++aPartIdx)
        {
            PSO_Particle aParticle = aSwarm.ChangeValue(aPartIdx);
            for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
            {
                aParticle.Position[aDimIdx] = theLowerBounds[aDimIdx] + aRNG.NextReal() * aRange[aDimIdx];
                aParticle.Velocity[aDimIdx] = (2.0 * aRNG.NextReal() - 1.0) * aVelMax[aDimIdx];
            }

            if (!theFunc.Value(aParticle.Position, out aParticle.CurrentValue))
            {
                aParticle.CurrentValue = double.MaxValue;
            }
            ++aLocalStats.NbFunctionEvals;

            aParticle.BestPosition.Initialized(aParticle.Position);
            aParticle.BestValue = aParticle.CurrentValue;

            if (aParticle.BestValue < aGlobalBestValue)
            {
                aGlobalBestValue = aParticle.BestValue;
                aGlobalBest.Initialized(aParticle.BestPosition);
            }
        }

        aLocalStats.InitialBest = aGlobalBestValue;

        // Main PSO loop
        double aPrevBest = aGlobalBestValue;
        int aStagnationCount = 0;
        double aStagnTol =
          (theConfig.NoImproveTol > 0.0) ? theConfig.NoImproveTol : theConfig.Tolerance;

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            // Compute inertia weight
            double anOmega = theConfig.Omega;
            if (theConfig.InertiaSchedule == PSOInertiaSchedule.LinearDecay)
            {
                anOmega = theConfig.Omega
                          - (theConfig.Omega - theConfig.OmegaMin) * (double)anIter
                              / (double)theConfig.MaxIterations;
            }

            // Update each particle
            for (int aPartIdx = 0; aPartIdx < aNbParticles; ++aPartIdx)
            {
                PSO_Particle aParticle = aSwarm.ChangeValue(aPartIdx);

                // Update velocity
                for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
                {
                    double aRand1 = aRNG.NextReal();
                    double aRand2 = aRNG.NextReal();

                    double aVnew =
                      anOmega * aParticle.Velocity[aDimIdx]
                      + theConfig.PhiPersonal * aRand1
                          * (aParticle.BestPosition[aDimIdx] - aParticle.Position[aDimIdx])
                      + theConfig.PhiGlobal * aRand2 * (aGlobalBest[aDimIdx] - aParticle.Position[aDimIdx]);

                    // Clamp velocity
                    aVnew = MathUtils.Clamp(aVnew, -aVelMax[aDimIdx], aVelMax[aDimIdx]);
                    aParticle.Velocity[aDimIdx] = aVnew;
                }

                // Update position with boundary handling
                for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
                {
                    double aXnew = aParticle.Position[aDimIdx] + aParticle.Velocity[aDimIdx];

                    if (aXnew < theLowerBounds[aDimIdx] || aXnew > theUpperBounds[aDimIdx])
                    {
                        ++aLocalStats.NbBoundaryCorrections;

                        switch (theConfig.BoundaryMode)
                        {
                            case PSOBoundaryMode.Clamp:
                            {
                                aXnew = MathUtils.Clamp(aXnew, theLowerBounds[aDimIdx], theUpperBounds[aDimIdx]);
                                aParticle.Velocity[aDimIdx] = -0.5 * aParticle.Velocity[aDimIdx];
                                break;
                            }
                            case PSOBoundaryMode.Reflect:
                            {
                                if (aXnew < theLowerBounds[aDimIdx])
                                {
                                    aXnew = 2.0 * theLowerBounds[aDimIdx] - aXnew;
                                }
                                else
                                {
                                    aXnew = 2.0 * theUpperBounds[aDimIdx] - aXnew;
                                }
                                // Re-clamp in case reflection overshoots the other bound
                                aXnew = MathUtils.Clamp(aXnew, theLowerBounds[aDimIdx], theUpperBounds[aDimIdx]);
                                aParticle.Velocity[aDimIdx] = -0.5 * aParticle.Velocity[aDimIdx];
                                break;
                            }
                            case PSOBoundaryMode.Wrap:
                            {
                                double aRangeSize = aRange[aDimIdx];
                                aXnew = aXnew - theLowerBounds[aDimIdx];
                                aXnew = aXnew % aRangeSize; // C#'s % on doubles is std::fmod
                                if (aXnew < 0.0)
                                {
                                    aXnew += aRangeSize;
                                }
                                aXnew += theLowerBounds[aDimIdx];
                                break;
                            }
                        }
                    }

                    aParticle.Position[aDimIdx] = aXnew;
                }

                // Evaluate fitness
                if (!theFunc.Value(aParticle.Position, out aParticle.CurrentValue))
                {
                    aParticle.CurrentValue = double.MaxValue;
                }
                ++aLocalStats.NbFunctionEvals;

                // Update personal best
                if (aParticle.CurrentValue < aParticle.BestValue)
                {
                    aParticle.BestValue = aParticle.CurrentValue;
                    aParticle.BestPosition.Initialized(aParticle.Position);
                }

                // Update global best
                if (aParticle.BestValue < aGlobalBestValue)
                {
                    aGlobalBestValue = aParticle.BestValue;
                    aGlobalBest.Initialized(aParticle.BestPosition);
                }
            }

            // Check for target value early stop
            if (theConfig.TargetValue.HasValue && aGlobalBestValue <= theConfig.TargetValue.Value)
            {
                break;
            }

            // Check for convergence (stagnation) after minimum iterations
            if (anIter >= theConfig.MinIterations)
            {
                if (Math.Abs(aGlobalBestValue - aPrevBest) < aStagnTol * (1.0 + Math.Abs(aGlobalBestValue)))
                {
                    ++aStagnationCount;
                    if (aStagnationCount >= theConfig.NoImproveIters)
                    {
                        ++aLocalStats.NbStagnationEvents;

                        // Restart logic
                        if (theConfig.RestartFraction > 0.0
                            && (theConfig.MaxRestarts == 0 || aLocalStats.NbRestarts < theConfig.MaxRestarts))
                        {
                            ++aLocalStats.NbRestarts;
                            aStagnationCount = 0;

                            // Reinitialize worst particles
                            int aNbRestart = (int)Math.Ceiling(theConfig.RestartFraction * aNbParticles);
                            aNbRestart = Min(aNbRestart, aNbParticles - 1); // keep at least the best

                            // Find best particle index
                            int aBestIdx = 0;
                            for (int aFindIdx = 1; aFindIdx < aNbParticles; ++aFindIdx)
                            {
                                if (aSwarm.Value(aFindIdx).BestValue < aSwarm.Value(aBestIdx).BestValue)
                                {
                                    aBestIdx = aFindIdx;
                                }
                            }

                            // Sort by fitness descending (simple selection of worst)
                            // Reinitialize aNbRestart worst particles
                            NCollection_DynamicArray<int> aWorstIndices = new();
                            for (int aCollIdx = 0; aCollIdx < aNbParticles; ++aCollIdx)
                            {
                                if (aCollIdx != aBestIdx)
                                {
                                    aWorstIndices.Append(aCollIdx);
                                }
                            }
                            // Simple approach: sort by BestValue descending, take first aNbRestart
                            for (int aSortOuter = 0; aSortOuter < aWorstIndices.Length() - 1; ++aSortOuter)
                            {
                                for (int aSortInner = aSortOuter + 1; aSortInner < aWorstIndices.Length();
                                     ++aSortInner)
                                {
                                    if (aSwarm.Value(aWorstIndices.Value(aSortOuter)).BestValue
                                        < aSwarm.Value(aWorstIndices.Value(aSortInner)).BestValue)
                                    {
                                        int aTmp = aWorstIndices.Value(aSortOuter);
                                        aWorstIndices.ChangeValue(aSortOuter) = aWorstIndices.Value(aSortInner);
                                        aWorstIndices.ChangeValue(aSortInner) = aTmp;
                                    }
                                }
                            }

                            int aNbToRestart = Min(aNbRestart, aWorstIndices.Length());
                            for (int aRestIdx = 0; aRestIdx < aNbToRestart; ++aRestIdx)
                            {
                                PSO_Particle aRestartPart = aSwarm.ChangeValue(aWorstIndices.Value(aRestIdx));
                                for (int aDimIdx = aLower; aDimIdx <= aUpper; ++aDimIdx)
                                {
                                    aRestartPart.Position[aDimIdx] =
                                      theLowerBounds[aDimIdx] + aRNG.NextReal() * aRange[aDimIdx];
                                    aRestartPart.Velocity[aDimIdx] = (2.0 * aRNG.NextReal() - 1.0) * aVelMax[aDimIdx];
                                }

                                if (!theFunc.Value(aRestartPart.Position, out aRestartPart.CurrentValue))
                                {
                                    aRestartPart.CurrentValue = double.MaxValue;
                                }
                                ++aLocalStats.NbFunctionEvals;

                                aRestartPart.BestPosition.Initialized(aRestartPart.Position);
                                aRestartPart.BestValue = aRestartPart.CurrentValue;

                                if (aRestartPart.BestValue < aGlobalBestValue)
                                {
                                    aGlobalBestValue = aRestartPart.BestValue;
                                    aGlobalBest.Initialized(aRestartPart.BestPosition);
                                }
                            }
                        }
                        else
                        {
                            // No restart budget - converged
                            break;
                        }
                    }
                }
                else
                {
                    aStagnationCount = 0;
                }
            }
            aPrevBest = aGlobalBestValue;
        }

        // Polish the global best using coordinate-wise Brent's method
        int aPolishEvals = 0;
        if (theConfig.PolishBudgetPerDim > 0)
        {
            PolishCoordinateWise(theFunc,
                                 aGlobalBest,
                                 ref aGlobalBestValue,
                                 theLowerBounds,
                                 theUpperBounds,
                                 theConfig.Tolerance,
                                 theConfig.PolishBudgetPerDim * aNbDims,
                                 out aPolishEvals);
            aLocalStats.NbFunctionEvals += aPolishEvals;
        }

        aLocalStats.NbIterations = aResult.NbIterations;
        aLocalStats.FinalBest = aGlobalBestValue;

        theStats?.Assign(aLocalStats);

        aResult.Status = Status.OK;
        aResult.Solution = aGlobalBest;
        aResult.Value = aGlobalBestValue;
        return aResult;
    }

    /// <summary>Particle Swarm Optimization for global minimization (basic overload).</summary>
    public static VectorResult PSO<Function>(Function theFunc,
                                             math_Vector theLowerBounds,
                                             math_Vector theUpperBounds,
                                             PSOConfig? theConfig = null)
        where Function : IVectorFunction
    {
        theConfig ??= new PSOConfig();
        return PSO(theFunc, theLowerBounds, theUpperBounds, theConfig, null, null);
    }
}
