// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_PSO.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Optimisation;

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

        public PSOSeedParticle(math_Vector pos)
        {
            Position = new math_Vector(pos);
        }

        public PSOSeedParticle(math_Vector pos, double value)
        {
            Position = new math_Vector(pos);
            Value = value;
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

        public PSOStats(PSOStats other)
        {
            Assign(other);
        }

        /// <summary>C++'s assignment: copies all the values.</summary>
        public void Assign(PSOStats other)
        {
            NbFunctionEvals = other.NbFunctionEvals;
            NbIterations = other.NbIterations;
            NbBoundaryCorrections = other.NbBoundaryCorrections;
            NbStagnationEvents = other.NbStagnationEvents;
            NbRestarts = other.NbRestarts;
            InitialBest = other.InitialBest;
            FinalBest = other.FinalBest;
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

        public PSOConfig(int nbParticles, int maxIter = 100, double tolerance = 1.0e-8)
            : base(tolerance, maxIter, true)
        {
            NbParticles = nbParticles;
        }

        public PSOConfig(PSOConfig other)
            : base(other)
        {
            NbParticles = other.NbParticles;
            Omega = other.Omega;
            PhiPersonal = other.PhiPersonal;
            PhiGlobal = other.PhiGlobal;
            VelocityClamp = other.VelocityClamp;
            Seed = other.Seed;
            InitMode = other.InitMode;
            BoundaryMode = other.BoundaryMode;
            InertiaSchedule = other.InertiaSchedule;
            OmegaMin = other.OmegaMin;
            MinIterations = other.MinIterations;
            TargetValue = other.TargetValue;
            NoImproveTol = other.NoImproveTol;
            NoImproveIters = other.NoImproveIters;
            RestartFraction = other.RestartFraction;
            MaxRestarts = other.MaxRestarts;
            PolishBudgetPerDim = other.PolishBudgetPerDim;
        }
    }

    /// <summary>
    /// Coordinate-wise polishing of thePosition/theValue (updated in place) by Brent's 1D minimization along each
    /// coordinate within the bounds, using at most theMaxPolishEvals evaluations (counted in theEvalCount).
    /// </summary>
    public static void PolishCoordinateWise<Function>(Function func,
                                                      math_Vector position,
                                                      ref double value,
                                                      math_Vector lowerBounds,
                                                      math_Vector upperBounds,
                                                      double tolerance,
                                                      int maxPolishEvals,
                                                      out int evalCount)
        where Function : IVectorFunction
    {
        evalCount = 0;

        int lower = position.Lower();
        int upper = position.Upper();
        int nbDims = upper - lower + 1;

        // Fair per-coordinate cap: total budget split across dimensions and passes
        int nbPasses = (nbDims > 1) ? 2 : 1;
        int perCoordLimit = Max(1, maxPolishEvals / (nbDims * nbPasses));

        for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
        {
            if (evalCount >= maxPolishEvals)
            {
                break;
            }

            int maxIter = Min(perCoordLimit, maxPolishEvals - evalCount);
            BrentAlongCoordinate(func,
                                 position,
                                 dimIdx,
                                 lowerBounds[dimIdx],
                                 upperBounds[dimIdx],
                                 ref value,
                                 tolerance,
                                 maxIter,
                                 ref evalCount);
        }

        // Second pass for non-separable functions - dimensions may interact
        if (evalCount < maxPolishEvals && nbDims > 1)
        {
            for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
            {
                if (evalCount >= maxPolishEvals)
                {
                    break;
                }

                int maxIter = Min(perCoordLimit, maxPolishEvals - evalCount);
                BrentAlongCoordinate(func,
                                     position,
                                     dimIdx,
                                     lowerBounds[dimIdx],
                                     upperBounds[dimIdx],
                                     ref value,
                                     tolerance,
                                     maxIter,
                                     ref evalCount);
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

        public PSO_Particle(int lower, int upper)
        {
            Position = new math_Vector(lower, upper);
            Velocity = new math_Vector(lower, upper);
            BestPosition = new math_Vector(lower, upper);
            BestValue = double.MaxValue;
            CurrentValue = double.MaxValue;
        }
    }

    /// <summary>
    /// Particle Swarm Optimization for global minimization within box bounds, optionally initialized from seed
    /// particles (theSeeds, may be null) and reporting statistics into theStats (may be null). Stochastic, but
    /// reproducible for a given PSOConfig.Seed. The best point is polished coordinate-wise at the end.
    /// </summary>
    public static VectorResult PSO<Function>(Function func,
                                             math_Vector lowerBounds,
                                             math_Vector upperBounds,
                                             PSOConfig config,
                                             NCollection_DynamicArray<PSOSeedParticle>? seeds,
                                             PSOStats? stats = null)
        where Function : IVectorFunction
    {
        VectorResult result = new();

        int lower = lowerBounds.Lower();
        int upper = lowerBounds.Upper();
        int nbDims = upper - lower + 1;

        // Check dimensions
        // OCCT: only the length of the upper bounds is checked; they are then indexed with the lower bounds' indexes.
        if (upperBounds.Length() != nbDims)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        // Check bounds
        for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
        {
            if (lowerBounds[dimIdx] >= upperBounds[dimIdx])
            {
                result.Status = Status.InvalidInput;
                return result;
            }
        }

        int nbParticles = config.NbParticles;
        if (nbParticles <= 0)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        // Initialize stats
        PSOStats localStats = new();

        NCollection_DynamicArray<PSO_Particle> swarm = new();
        for (int partIdx = 0; partIdx < nbParticles; ++partIdx)
        {
            swarm.Append(new PSO_Particle(lower, upper));
        }

        // Random number generator
        RandomGenerator rng = new(config.Seed);

        // Compute velocity limits
        math_Vector velMax = new(lower, upper);
        math_Vector range = new(lower, upper);
        for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
        {
            range[dimIdx] = upperBounds[dimIdx] - lowerBounds[dimIdx];
            velMax[dimIdx] = config.VelocityClamp * range[dimIdx];
        }

        // Initialize particles
        math_Vector globalBest = new(lower, upper);
        double globalBestValue = double.MaxValue;

        // Determine how many particles are seeded
        int nbSeeds = (seeds is not null) ? seeds.Length() : 0;
        int seeded = 0;

        // Validate seed dimensions and index ranges
        if (nbSeeds > 0)
        {
            for (int seedIdx = 0; seedIdx < nbSeeds; ++seedIdx)
            {
                PSOSeedParticle seed = seeds!.Value(seedIdx);
                if (seed.Position.Lower() != lower || seed.Position.Upper() != upper)
                {
                    result.Status = Status.InvalidInput;
                    return result;
                }
                if (seed.Velocity is not null
                    && (seed.Velocity.Lower() != lower || seed.Velocity.Upper() != upper))
                {
                    result.Status = Status.InvalidInput;
                    return result;
                }
            }
        }

        // Validate SeededOnly requires at least one seed
        if (config.InitMode == PSOInitMode.SeededOnly && nbSeeds == 0)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        if (nbSeeds > 0
            && (config.InitMode == PSOInitMode.SeededPlusRandom
                || config.InitMode == PSOInitMode.SeededOnly))
        {
            // Initialize from seeds
            int nbDirect = Min(nbSeeds, nbParticles);
            for (int seedIdx = 0; seedIdx < nbDirect; ++seedIdx)
            {
                PSO_Particle particle = swarm.ChangeValue(seedIdx);
                PSOSeedParticle seed = seeds!.Value(seedIdx);

                // Clamp seed position to bounds, track if any coordinate was clamped
                bool wasClamped = false;
                for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
                {
                    double clamped = global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(seed.Position[dimIdx],
                                                      lowerBounds[dimIdx],
                                                      upperBounds[dimIdx]);
                    if (clamped != seed.Position[dimIdx])
                    {
                        wasClamped = true;
                    }
                    particle.Position[dimIdx] = clamped;
                }

                // Use seed velocity or generate random
                if (seed.Velocity is not null)
                {
                    math_Vector seedVel = seed.Velocity;
                    for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
                    {
                        particle.Velocity[dimIdx] =
                          global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(seedVel[dimIdx], -velMax[dimIdx], velMax[dimIdx]);
                    }
                }
                else
                {
                    for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
                    {
                        particle.Velocity[dimIdx] = (2.0 * rng.NextReal() - 1.0) * velMax[dimIdx];
                    }
                }

                // Use known value only if position was not clamped; otherwise re-evaluate
                if (seed.Value.HasValue && !wasClamped)
                {
                    particle.CurrentValue = seed.Value.Value;
                }
                else
                {
                    if (!func.Value(particle.Position, out particle.CurrentValue))
                    {
                        particle.CurrentValue = double.MaxValue;
                    }
                    ++localStats.NbFunctionEvals;
                }

                particle.BestPosition.Initialized(particle.Position);
                particle.BestValue = particle.CurrentValue;

                if (particle.BestValue < globalBestValue)
                {
                    globalBestValue = particle.BestValue;
                    globalBest.Initialized(particle.BestPosition);
                }
            }
            seeded = nbDirect;

            // For SeededOnly: if seeds < NbParticles, jitter best seeds to fill
            if (config.InitMode == PSOInitMode.SeededOnly && seeded < nbParticles)
            {
                for (int partIdx = seeded; partIdx < nbParticles; ++partIdx)
                {
                    PSO_Particle particle = swarm.ChangeValue(partIdx);
                    // Pick a seed to jitter (round-robin)
                    int srcIdx = partIdx % seeded;
                    PSOSeedParticle src = seeds!.Value(srcIdx);

                    for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
                    {
                        double jitter = (2.0 * rng.NextReal() - 1.0) * 0.1 * range[dimIdx];
                        particle.Position[dimIdx] = global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(src.Position[dimIdx] + jitter,
                                                                      lowerBounds[dimIdx],
                                                                      upperBounds[dimIdx]);
                        particle.Velocity[dimIdx] = (2.0 * rng.NextReal() - 1.0) * velMax[dimIdx];
                    }

                    if (!func.Value(particle.Position, out particle.CurrentValue))
                    {
                        particle.CurrentValue = double.MaxValue;
                    }
                    ++localStats.NbFunctionEvals;

                    particle.BestPosition.Initialized(particle.Position);
                    particle.BestValue = particle.CurrentValue;

                    if (particle.BestValue < globalBestValue)
                    {
                        globalBestValue = particle.BestValue;
                        globalBest.Initialized(particle.BestPosition);
                    }
                }
                seeded = nbParticles;
            }
        }

        // Fill remaining particles randomly
        for (int partIdx = seeded; partIdx < nbParticles; ++partIdx)
        {
            PSO_Particle particle = swarm.ChangeValue(partIdx);
            for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
            {
                particle.Position[dimIdx] = lowerBounds[dimIdx] + rng.NextReal() * range[dimIdx];
                particle.Velocity[dimIdx] = (2.0 * rng.NextReal() - 1.0) * velMax[dimIdx];
            }

            if (!func.Value(particle.Position, out particle.CurrentValue))
            {
                particle.CurrentValue = double.MaxValue;
            }
            ++localStats.NbFunctionEvals;

            particle.BestPosition.Initialized(particle.Position);
            particle.BestValue = particle.CurrentValue;

            if (particle.BestValue < globalBestValue)
            {
                globalBestValue = particle.BestValue;
                globalBest.Initialized(particle.BestPosition);
            }
        }

        localStats.InitialBest = globalBestValue;

        // Main PSO loop
        double prevBest = globalBestValue;
        int stagnationCount = 0;
        double stagnTol =
          (config.NoImproveTol > 0.0) ? config.NoImproveTol : config.Tolerance;

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            // Compute inertia weight
            double omega = config.Omega;
            if (config.InertiaSchedule == PSOInertiaSchedule.LinearDecay)
            {
                omega = config.Omega
                          - (config.Omega - config.OmegaMin) * (double)iter
                              / (double)config.MaxIterations;
            }

            // Update each particle
            for (int partIdx = 0; partIdx < nbParticles; ++partIdx)
            {
                PSO_Particle particle = swarm.ChangeValue(partIdx);

                // Update velocity
                for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
                {
                    double rand1 = rng.NextReal();
                    double rand2 = rng.NextReal();

                    double vnew =
                      omega * particle.Velocity[dimIdx]
                      + config.PhiPersonal * rand1
                          * (particle.BestPosition[dimIdx] - particle.Position[dimIdx])
                      + config.PhiGlobal * rand2 * (globalBest[dimIdx] - particle.Position[dimIdx]);

                    // Clamp velocity
                    vnew = global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(vnew, -velMax[dimIdx], velMax[dimIdx]);
                    particle.Velocity[dimIdx] = vnew;
                }

                // Update position with boundary handling
                for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
                {
                    double xnew = particle.Position[dimIdx] + particle.Velocity[dimIdx];

                    if (xnew < lowerBounds[dimIdx] || xnew > upperBounds[dimIdx])
                    {
                        ++localStats.NbBoundaryCorrections;

                        switch (config.BoundaryMode)
                        {
                            case PSOBoundaryMode.Clamp:
                            {
                                xnew = global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(xnew, lowerBounds[dimIdx], upperBounds[dimIdx]);
                                particle.Velocity[dimIdx] = -0.5 * particle.Velocity[dimIdx];
                                break;
                            }
                            case PSOBoundaryMode.Reflect:
                            {
                                if (xnew < lowerBounds[dimIdx])
                                {
                                    xnew = 2.0 * lowerBounds[dimIdx] - xnew;
                                }
                                else
                                {
                                    xnew = 2.0 * upperBounds[dimIdx] - xnew;
                                }
                                // Re-clamp in case reflection overshoots the other bound
                                xnew = global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(xnew, lowerBounds[dimIdx], upperBounds[dimIdx]);
                                particle.Velocity[dimIdx] = -0.5 * particle.Velocity[dimIdx];
                                break;
                            }
                            case PSOBoundaryMode.Wrap:
                            {
                                double rangeSize = range[dimIdx];
                                xnew = xnew - lowerBounds[dimIdx];
                                xnew = xnew % rangeSize; // C#'s % on doubles is std::fmod
                                if (xnew < 0.0)
                                {
                                    xnew += rangeSize;
                                }
                                xnew += lowerBounds[dimIdx];
                                break;
                            }
                        }
                    }

                    particle.Position[dimIdx] = xnew;
                }

                // Evaluate fitness
                if (!func.Value(particle.Position, out particle.CurrentValue))
                {
                    particle.CurrentValue = double.MaxValue;
                }
                ++localStats.NbFunctionEvals;

                // Update personal best
                if (particle.CurrentValue < particle.BestValue)
                {
                    particle.BestValue = particle.CurrentValue;
                    particle.BestPosition.Initialized(particle.Position);
                }

                // Update global best
                if (particle.BestValue < globalBestValue)
                {
                    globalBestValue = particle.BestValue;
                    globalBest.Initialized(particle.BestPosition);
                }
            }

            // Check for target value early stop
            if (config.TargetValue.HasValue && globalBestValue <= config.TargetValue.Value)
            {
                break;
            }

            // Check for convergence (stagnation) after minimum iterations
            if (iter >= config.MinIterations)
            {
                if (Math.Abs(globalBestValue - prevBest) < stagnTol * (1.0 + Math.Abs(globalBestValue)))
                {
                    ++stagnationCount;
                    if (stagnationCount >= config.NoImproveIters)
                    {
                        ++localStats.NbStagnationEvents;

                        // Restart logic
                        if (config.RestartFraction > 0.0
                            && (config.MaxRestarts == 0 || localStats.NbRestarts < config.MaxRestarts))
                        {
                            ++localStats.NbRestarts;
                            stagnationCount = 0;

                            // Reinitialize worst particles
                            int nbRestart = (int)Math.Ceiling(config.RestartFraction * nbParticles);
                            nbRestart = Min(nbRestart, nbParticles - 1); // keep at least the best

                            // Find best particle index
                            int bestIdx = 0;
                            for (int findIdx = 1; findIdx < nbParticles; ++findIdx)
                            {
                                if (swarm.Value(findIdx).BestValue < swarm.Value(bestIdx).BestValue)
                                {
                                    bestIdx = findIdx;
                                }
                            }

                            // Sort by fitness descending (simple selection of worst)
                            // Reinitialize aNbRestart worst particles
                            NCollection_DynamicArray<int> worstIndices = new();
                            for (int collIdx = 0; collIdx < nbParticles; ++collIdx)
                            {
                                if (collIdx != bestIdx)
                                {
                                    worstIndices.Append(collIdx);
                                }
                            }
                            // Simple approach: sort by BestValue descending, take first aNbRestart
                            for (int sortOuter = 0; sortOuter < worstIndices.Length() - 1; ++sortOuter)
                            {
                                for (int sortInner = sortOuter + 1; sortInner < worstIndices.Length();
                                     ++sortInner)
                                {
                                    if (swarm.Value(worstIndices.Value(sortOuter)).BestValue
                                        < swarm.Value(worstIndices.Value(sortInner)).BestValue)
                                    {
                                        int tmp = worstIndices.Value(sortOuter);
                                        worstIndices.ChangeValue(sortOuter) = worstIndices.Value(sortInner);
                                        worstIndices.ChangeValue(sortInner) = tmp;
                                    }
                                }
                            }

                            int nbToRestart = Min(nbRestart, worstIndices.Length());
                            for (int restIdx = 0; restIdx < nbToRestart; ++restIdx)
                            {
                                PSO_Particle restartPart = swarm.ChangeValue(worstIndices.Value(restIdx));
                                for (int dimIdx = lower; dimIdx <= upper; ++dimIdx)
                                {
                                    restartPart.Position[dimIdx] =
                                      lowerBounds[dimIdx] + rng.NextReal() * range[dimIdx];
                                    restartPart.Velocity[dimIdx] = (2.0 * rng.NextReal() - 1.0) * velMax[dimIdx];
                                }

                                if (!func.Value(restartPart.Position, out restartPart.CurrentValue))
                                {
                                    restartPart.CurrentValue = double.MaxValue;
                                }
                                ++localStats.NbFunctionEvals;

                                restartPart.BestPosition.Initialized(restartPart.Position);
                                restartPart.BestValue = restartPart.CurrentValue;

                                if (restartPart.BestValue < globalBestValue)
                                {
                                    globalBestValue = restartPart.BestValue;
                                    globalBest.Initialized(restartPart.BestPosition);
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
                    stagnationCount = 0;
                }
            }
            prevBest = globalBestValue;
        }

        // Polish the global best using coordinate-wise Brent's method
        int polishEvals = 0;
        if (config.PolishBudgetPerDim > 0)
        {
            PolishCoordinateWise(func,
                                 globalBest,
                                 ref globalBestValue,
                                 lowerBounds,
                                 upperBounds,
                                 config.Tolerance,
                                 config.PolishBudgetPerDim * nbDims,
                                 out polishEvals);
            localStats.NbFunctionEvals += polishEvals;
        }

        localStats.NbIterations = result.NbIterations;
        localStats.FinalBest = globalBestValue;

        stats?.Assign(localStats);

        result.Status = Status.OK;
        result.Solution = globalBest;
        result.Value = globalBestValue;
        return result;
    }

    /// <summary>Particle Swarm Optimization for global minimization (basic overload).</summary>
    public static VectorResult PSO<Function>(Function func,
                                             math_Vector lowerBounds,
                                             math_Vector upperBounds,
                                             PSOConfig? config = null)
        where Function : IVectorFunction
    {
        config ??= new PSOConfig();
        return PSO(func, lowerBounds, upperBounds, config, null, null);
    }
}
