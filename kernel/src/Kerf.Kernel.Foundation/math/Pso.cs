// Port of: src/FoundationClasses/TKMath/math/math_PSO.hxx
// Port of: src/FoundationClasses/TKMath/math/math_PSO.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics;

/// <summary>
/// A variation of the Particle Swarm Optimization (PSO) method (A. Ismael F. Vaz, L. N. Vicente, "A particle swarm
/// pattern search method for bound constrained global optimization").
/// </summary>
/// <remarks>
/// Particles placed in the search space move in cycles, each new position depending on the particle's velocity, its
/// own best position and the best position of the pool; the velocities decrease so that it converges. It finds the
/// surroundings of an optimum, not a strict optimum: run a local optimization from its output. The random number
/// generator has a fixed seed, so results are reproducible.
/// </remarks>
internal class Pso
{
    private const double ABorderDivisor = 1.0e+4;

    private MultipleVarFunction _func;
    private math_Vector _lowBorder; // Lower border.
    private math_Vector _uppBorder; // Upper border.
    private math_Vector _steps; // steps used in PSO algorithm.
    private int _n; // Dimension count.
    private int _nbParticles; // Particles number.
    private int _nbIter;

    /// <summary>
    /// Prepares the algorithm. theFunc must live as long as this object; theSteps are the steps of the regular grid
    /// used to generate the particles, which also define the stop condition (terminal velocity).
    /// </summary>
    public Pso(MultipleVarFunction func, math_Vector lowBorder, math_Vector uppBorder, math_Vector steps,
                    int nbParticles = 32, int nbIter = 100)
    {
        _lowBorder = new math_Vector(1, func.VariableCount);
        _uppBorder = new math_Vector(1, func.VariableCount);
        _steps = new math_Vector(1, func.VariableCount);

        _n = func.VariableCount;
        _nbParticles = nbParticles;
        _nbIter = nbIter;
        _func = func;

        _lowBorder.Initialized(lowBorder);
        _uppBorder.Initialized(uppBorder);
        _steps.Initialized(steps);
    }

    /// <summary>Performs the computation with the given particles.</summary>
    public void Perform(PsoParticlesPool particles, int nbParticles, out double value, math_Vector outPnt, int nbIter = 100)
    {
        PerformPSOWithGivenParticles(particles, nbParticles, out value, outPnt, nbIter);
    }

    /// <summary>Performs the computation, generating the particles on a regular grid of steps <paramref name="steps"/>.</summary>
    public void Perform(math_Vector steps, out double value, math_Vector outPnt, int nbIter = 100)
    {
        // Initialization.
        math_Vector minUV = new(1, _n), maxUV = new(1, _n);
        minUV.Initialized(_lowBorder + (_uppBorder - _lowBorder) / ABorderDivisor);
        maxUV.Initialized(_uppBorder - (_uppBorder - _lowBorder) / ABorderDivisor);
        _nbIter = nbIter;
        _steps.Initialized(steps);

        // To generate initial distribution it is necessary to have grid steps.
        PsoParticlesPool pool = new(_nbParticles, _n);

        // Generate initial particles distribution.
        bool isRegularGridFinished = false;
        double currValue;
        math_Vector currPoint = new(1, _n);

        Particle particle = pool.GetWorstParticle();
        currPoint.Initialized(minUV);
        do
        {
            _func.Value(currPoint, out currValue);

            if (currValue < particle.Distance)
            {
                int dimIdx;
                for (dimIdx = 0; dimIdx < _n; ++dimIdx)
                {
                    particle.Position[dimIdx] = currPoint[dimIdx + 1];
                    particle.BestPosition[dimIdx] = currPoint[dimIdx + 1];
                }

                particle.Distance = currValue;
                particle.BestDistance = currValue;

                particle = pool.GetWorstParticle();
            }

            // Step.
            currPoint[1] += Max(_steps[1], 1.0e-15); // Avoid too small step
            for (int dimIdx = 1; dimIdx < _n; ++dimIdx)
            {
                if (currPoint[dimIdx] > maxUV[dimIdx])
                {
                    currPoint[dimIdx] = minUV[dimIdx];
                    currPoint[dimIdx + 1] += _steps[dimIdx + 1];
                }
                else
                {
                    break;
                }
            }

            // Stop criteria.
            if (currPoint[_n] > maxUV[_n])
            {
                isRegularGridFinished = true;
            }
        } while (!isRegularGridFinished);

        PerformPSOWithGivenParticles(pool, _nbParticles, out value, outPnt, nbIter);
    }

    private void PerformPSOWithGivenParticles(PsoParticlesPool particles, int nbParticles, out double value,
                                              math_Vector outPnt, int nbIter = 100)
    {
        math_Vector minUV = new(1, _n), maxUV = new(1, _n);
        minUV.Initialized(_lowBorder + (_uppBorder - _lowBorder) / ABorderDivisor);
        maxUV.Initialized(_uppBorder - (_uppBorder - _lowBorder) / ABorderDivisor);
        _nbIter = nbIter;
        _nbParticles = nbParticles;
        PsoParticlesPool aParticles = particles;

        // Current particle data.
        math_Vector currPoint = new(1, _n);
        math_Vector bestGlobalPosition = new(1, _n);
        Particle particle;

        // Generate initial particle velocities.
        BullardGenerator random = new();
        for (int partIdx = 1; partIdx <= _nbParticles; ++partIdx)
        {
            particle = aParticles.GetParticle(partIdx);
            for (int dimIdx = 1; dimIdx <= _n; ++dimIdx)
            {
                double ksi = random.NextReal();
                particle.Velocity[dimIdx - 1] = _steps[dimIdx] * (ksi - 0.5) * 2.0;
            }
        }

        particle = aParticles.GetBestParticle();
        for (int dimIdx = 0; dimIdx < _n; ++dimIdx)
        {
            bestGlobalPosition[dimIdx + 1] = particle.Position[dimIdx];
        }

        double bestGlobalDistance = particle.Distance;

        // This velocity is used for detecting stagnation state.
        math_Vector terminationVelocity = new(1, _n);
        terminationVelocity.Initialized(_steps / 2048.0);
        // This velocity shows minimal velocity on current step.
        math_Vector minimalVelocity = new(1, _n);

        // Run PSO iterations
        for (int step = 1; step < _nbIter; ++step)
        {
            minimalVelocity.Init(RealLast());

            for (int partIdx = 1; partIdx <= _nbParticles; ++partIdx)
            {
                double ksi1 = random.NextReal();
                double ksi2 = random.NextReal();

                particle = aParticles.GetParticle(partIdx);

                const double retentWeight = 0.72900;
                const double personWeight = 1.49445;
                const double socialWeight = 1.49445;

                for (int dimIdx = 0; dimIdx < _n; ++dimIdx)
                {
                    particle.Velocity[dimIdx] =
                        particle.Velocity[dimIdx] * retentWeight
                        + (particle.BestPosition[dimIdx] - particle.Position[dimIdx]) * (personWeight * ksi1)
                        + (bestGlobalPosition[dimIdx + 1] - particle.Position[dimIdx]) * (socialWeight * ksi2);

                    particle.Position[dimIdx] += particle.Velocity[dimIdx];
                    particle.Position[dimIdx] = Min(Max(particle.Position[dimIdx], minUV[dimIdx + 1]), maxUV[dimIdx + 1]);
                    currPoint[dimIdx + 1] = particle.Position[dimIdx];

                    minimalVelocity[dimIdx + 1] = Min(Math.Abs(particle.Velocity[dimIdx]), minimalVelocity[dimIdx + 1]);
                }

                _func.Value(currPoint, out particle.Distance);
                if (particle.Distance < particle.BestDistance)
                {
                    particle.BestDistance = particle.Distance;
                    for (int dimIdx = 0; dimIdx < _n; ++dimIdx)
                    {
                        particle.BestPosition[dimIdx] = particle.Position[dimIdx];
                    }

                    if (particle.Distance < bestGlobalDistance)
                    {
                        bestGlobalDistance = particle.Distance;
                        for (int dimIdx = 0; dimIdx < _n; ++dimIdx)
                        {
                            bestGlobalPosition[dimIdx + 1] = particle.Position[dimIdx];
                        }
                    }
                }
            }

            bool isTerminalVelocityReached = true;
            for (int dimIdx = 1; dimIdx <= _n; ++dimIdx)
            {
                if (minimalVelocity[dimIdx] > terminationVelocity[dimIdx])
                {
                    isTerminalVelocityReached = false;
                    break;
                }
            }

            if (isTerminalVelocityReached)
            {
                // Minimum number of steps
                const int minSteps = 16;

                if (step > minSteps)
                {
                    break;
                }

                for (int partIdx = 1; partIdx <= _nbParticles; ++partIdx)
                {
                    double ksi = random.NextReal();

                    particle = aParticles.GetParticle(partIdx);

                    for (int dimIdx = 0; dimIdx < _n; ++dimIdx)
                    {
                        if (particle.Position[dimIdx] == minUV[dimIdx + 1] || particle.Position[dimIdx] == maxUV[dimIdx + 1])
                        {
                            particle.Velocity[dimIdx] = particle.Position[dimIdx] == minUV[dimIdx + 1]
                                                              ? _steps[dimIdx + 1] * ksi
                                                              : -_steps[dimIdx + 1] * ksi;
                        }
                        else
                        {
                            particle.Velocity[dimIdx] = _steps[dimIdx + 1] * (ksi - 0.5) * 2.0;
                        }
                    }
                }
            }
        }

        value = bestGlobalDistance;
        outPnt.Initialized(bestGlobalPosition);
    }
}
