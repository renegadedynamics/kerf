// Port of: src/FoundationClasses/TKMath/math/math_PSO.hxx
// Port of: src/FoundationClasses/TKMath/math/math_PSO.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public class math_PSO
{
    private const double aBorderDivisor = 1.0e+4;

    private math_MultipleVarFunction myFunc;
    private math_Vector myLowBorder; // Lower border.
    private math_Vector myUppBorder; // Upper border.
    private math_Vector mySteps; // steps used in PSO algorithm.
    private int myN; // Dimension count.
    private int myNbParticles; // Particles number.
    private int myNbIter;

    /// <summary>
    /// Prepares the algorithm. theFunc must live as long as this object; theSteps are the steps of the regular grid
    /// used to generate the particles, which also define the stop condition (terminal velocity).
    /// </summary>
    public math_PSO(math_MultipleVarFunction theFunc, math_Vector theLowBorder, math_Vector theUppBorder, math_Vector theSteps,
                    int theNbParticles = 32, int theNbIter = 100)
    {
        myLowBorder = new math_Vector(1, theFunc.NbVariables());
        myUppBorder = new math_Vector(1, theFunc.NbVariables());
        mySteps = new math_Vector(1, theFunc.NbVariables());

        myN = theFunc.NbVariables();
        myNbParticles = theNbParticles;
        myNbIter = theNbIter;
        myFunc = theFunc;

        myLowBorder.Initialized(theLowBorder);
        myUppBorder.Initialized(theUppBorder);
        mySteps.Initialized(theSteps);
    }

    /// <summary>Performs the computation with the given particles.</summary>
    public void Perform(math_PSOParticlesPool theParticles, int theNbParticles, out double theValue, math_Vector theOutPnt, int theNbIter = 100)
    {
        performPSOWithGivenParticles(theParticles, theNbParticles, out theValue, theOutPnt, theNbIter);
    }

    /// <summary>Performs the computation, generating the particles on a regular grid of steps <paramref name="theSteps"/>.</summary>
    public void Perform(math_Vector theSteps, out double theValue, math_Vector theOutPnt, int theNbIter = 100)
    {
        // Initialization.
        math_Vector aMinUV = new(1, myN), aMaxUV = new(1, myN);
        aMinUV.Initialized(myLowBorder + (myUppBorder - myLowBorder) / aBorderDivisor);
        aMaxUV.Initialized(myUppBorder - (myUppBorder - myLowBorder) / aBorderDivisor);
        myNbIter = theNbIter;
        mySteps.Initialized(theSteps);

        // To generate initial distribution it is necessary to have grid steps.
        math_PSOParticlesPool aPool = new(myNbParticles, myN);

        // Generate initial particles distribution.
        bool isRegularGridFinished = false;
        double aCurrValue;
        math_Vector aCurrPoint = new(1, myN);

        PSO_Particle aParticle = aPool.GetWorstParticle();
        aCurrPoint.Initialized(aMinUV);
        do
        {
            myFunc.Value(aCurrPoint, out aCurrValue);

            if (aCurrValue < aParticle.Distance)
            {
                int aDimIdx;
                for (aDimIdx = 0; aDimIdx < myN; ++aDimIdx)
                {
                    aParticle.Position[aDimIdx] = aCurrPoint[aDimIdx + 1];
                    aParticle.BestPosition[aDimIdx] = aCurrPoint[aDimIdx + 1];
                }

                aParticle.Distance = aCurrValue;
                aParticle.BestDistance = aCurrValue;

                aParticle = aPool.GetWorstParticle();
            }

            // Step.
            aCurrPoint[1] += Max(mySteps[1], 1.0e-15); // Avoid too small step
            for (int aDimIdx = 1; aDimIdx < myN; ++aDimIdx)
            {
                if (aCurrPoint[aDimIdx] > aMaxUV[aDimIdx])
                {
                    aCurrPoint[aDimIdx] = aMinUV[aDimIdx];
                    aCurrPoint[aDimIdx + 1] += mySteps[aDimIdx + 1];
                }
                else
                {
                    break;
                }
            }

            // Stop criteria.
            if (aCurrPoint[myN] > aMaxUV[myN])
            {
                isRegularGridFinished = true;
            }
        } while (!isRegularGridFinished);

        performPSOWithGivenParticles(aPool, myNbParticles, out theValue, theOutPnt, theNbIter);
    }

    private void performPSOWithGivenParticles(math_PSOParticlesPool theParticles, int theNbParticles, out double theValue,
                                              math_Vector theOutPnt, int theNbIter = 100)
    {
        math_Vector aMinUV = new(1, myN), aMaxUV = new(1, myN);
        aMinUV.Initialized(myLowBorder + (myUppBorder - myLowBorder) / aBorderDivisor);
        aMaxUV.Initialized(myUppBorder - (myUppBorder - myLowBorder) / aBorderDivisor);
        myNbIter = theNbIter;
        myNbParticles = theNbParticles;
        math_PSOParticlesPool aParticles = theParticles;

        // Current particle data.
        math_Vector aCurrPoint = new(1, myN);
        math_Vector aBestGlobalPosition = new(1, myN);
        PSO_Particle aParticle;

        // Generate initial particle velocities.
        math_BullardGenerator aRandom = new();
        for (int aPartIdx = 1; aPartIdx <= myNbParticles; ++aPartIdx)
        {
            aParticle = aParticles.GetParticle(aPartIdx);
            for (int aDimIdx = 1; aDimIdx <= myN; ++aDimIdx)
            {
                double aKsi = aRandom.NextReal();
                aParticle.Velocity[aDimIdx - 1] = mySteps[aDimIdx] * (aKsi - 0.5) * 2.0;
            }
        }

        aParticle = aParticles.GetBestParticle();
        for (int aDimIdx = 0; aDimIdx < myN; ++aDimIdx)
        {
            aBestGlobalPosition[aDimIdx + 1] = aParticle.Position[aDimIdx];
        }

        double aBestGlobalDistance = aParticle.Distance;

        // This velocity is used for detecting stagnation state.
        math_Vector aTerminationVelocity = new(1, myN);
        aTerminationVelocity.Initialized(mySteps / 2048.0);
        // This velocity shows minimal velocity on current step.
        math_Vector aMinimalVelocity = new(1, myN);

        // Run PSO iterations
        for (int aStep = 1; aStep < myNbIter; ++aStep)
        {
            aMinimalVelocity.Init(RealLast());

            for (int aPartIdx = 1; aPartIdx <= myNbParticles; ++aPartIdx)
            {
                double aKsi1 = aRandom.NextReal();
                double aKsi2 = aRandom.NextReal();

                aParticle = aParticles.GetParticle(aPartIdx);

                const double aRetentWeight = 0.72900;
                const double aPersonWeight = 1.49445;
                const double aSocialWeight = 1.49445;

                for (int aDimIdx = 0; aDimIdx < myN; ++aDimIdx)
                {
                    aParticle.Velocity[aDimIdx] =
                        aParticle.Velocity[aDimIdx] * aRetentWeight
                        + (aParticle.BestPosition[aDimIdx] - aParticle.Position[aDimIdx]) * (aPersonWeight * aKsi1)
                        + (aBestGlobalPosition[aDimIdx + 1] - aParticle.Position[aDimIdx]) * (aSocialWeight * aKsi2);

                    aParticle.Position[aDimIdx] += aParticle.Velocity[aDimIdx];
                    aParticle.Position[aDimIdx] = Min(Max(aParticle.Position[aDimIdx], aMinUV[aDimIdx + 1]), aMaxUV[aDimIdx + 1]);
                    aCurrPoint[aDimIdx + 1] = aParticle.Position[aDimIdx];

                    aMinimalVelocity[aDimIdx + 1] = Min(Math.Abs(aParticle.Velocity[aDimIdx]), aMinimalVelocity[aDimIdx + 1]);
                }

                myFunc.Value(aCurrPoint, out aParticle.Distance);
                if (aParticle.Distance < aParticle.BestDistance)
                {
                    aParticle.BestDistance = aParticle.Distance;
                    for (int aDimIdx = 0; aDimIdx < myN; ++aDimIdx)
                    {
                        aParticle.BestPosition[aDimIdx] = aParticle.Position[aDimIdx];
                    }

                    if (aParticle.Distance < aBestGlobalDistance)
                    {
                        aBestGlobalDistance = aParticle.Distance;
                        for (int aDimIdx = 0; aDimIdx < myN; ++aDimIdx)
                        {
                            aBestGlobalPosition[aDimIdx + 1] = aParticle.Position[aDimIdx];
                        }
                    }
                }
            }

            bool isTerminalVelocityReached = true;
            for (int aDimIdx = 1; aDimIdx <= myN; ++aDimIdx)
            {
                if (aMinimalVelocity[aDimIdx] > aTerminationVelocity[aDimIdx])
                {
                    isTerminalVelocityReached = false;
                    break;
                }
            }

            if (isTerminalVelocityReached)
            {
                // Minimum number of steps
                const int aMinSteps = 16;

                if (aStep > aMinSteps)
                {
                    break;
                }

                for (int aPartIdx = 1; aPartIdx <= myNbParticles; ++aPartIdx)
                {
                    double aKsi = aRandom.NextReal();

                    aParticle = aParticles.GetParticle(aPartIdx);

                    for (int aDimIdx = 0; aDimIdx < myN; ++aDimIdx)
                    {
                        if (aParticle.Position[aDimIdx] == aMinUV[aDimIdx + 1] || aParticle.Position[aDimIdx] == aMaxUV[aDimIdx + 1])
                        {
                            aParticle.Velocity[aDimIdx] = aParticle.Position[aDimIdx] == aMinUV[aDimIdx + 1]
                                                              ? mySteps[aDimIdx + 1] * aKsi
                                                              : -mySteps[aDimIdx + 1] * aKsi;
                        }
                        else
                        {
                            aParticle.Velocity[aDimIdx] = mySteps[aDimIdx + 1] * (aKsi - 0.5) * 2.0;
                        }
                    }
                }
            }
        }

        theValue = aBestGlobalDistance;
        theOutPnt.Initialized(aBestGlobalPosition);
    }
}
