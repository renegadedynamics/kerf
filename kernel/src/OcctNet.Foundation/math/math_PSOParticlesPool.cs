// Port of: src/FoundationClasses/TKMath/math/math_PSOParticlesPool.hxx
// Port of: src/FoundationClasses/TKMath/math/math_PSOParticlesPool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A particle of <see cref="math_PSOParticlesPool"/>. Coordinate indexes are 0..dimension-1.</summary>
/// <remarks>
/// A struct in OCCT, but only ever used through pointers into the pool, so a class here. Position, Velocity and
/// BestPosition are views (lower bound 0) over the pool's shared storage, as OCCT's pointers are.
/// </remarks>
public class PSO_Particle
{
    public NCollection_Array1<double> Position; // Data for pointers allocated within PSOParticlesPool instance.
    public NCollection_Array1<double> Velocity; // Not need to delete it manually.
    public NCollection_Array1<double> BestPosition;
    public double Distance;
    public double BestDistance;

    public PSO_Particle()
    {
        Distance = RealLast();
        BestDistance = RealLast();
        Position = null!;
        Velocity = null!;
        BestPosition = null!;
    }

    /// <summary>OCCT's <c>operator&lt;</c>: compares the particles by their distances.</summary>
    public static bool operator <(PSO_Particle theLeft, PSO_Particle thePnt) => theLeft.Distance < thePnt.Distance;

    public static bool operator >(PSO_Particle theLeft, PSO_Particle thePnt) => thePnt.Distance < theLeft.Distance;
}

/// <summary>The particle pool of the PSO algorithm (<see cref="math_PSO"/>). Particle indexes are 1..count.</summary>
public class math_PSOParticlesPool
{
    private NCollection_Array1<PSO_Particle> myParticlesPool;
    private NCollection_Array1<double> myMemory; // Stores particles vector data.
    private int myParticlesCount;
    private int myDimensionCount;

    public math_PSOParticlesPool(int theParticlesCount, int theDimensionCount)
    {
        myParticlesPool = new NCollection_Array1<PSO_Particle>(1, theParticlesCount);
        myMemory = new NCollection_Array1<double>(0, theParticlesCount * (theDimensionCount // Position
                                                                         + theDimensionCount // Velocity
                                                                         + theDimensionCount) // BestPosition
                                                         - 1);
        myParticlesCount = theParticlesCount;
        myDimensionCount = theDimensionCount;

        myMemory.Init(0.0);

        // Pointers adjusting.
        int aParIdx, aShiftIdx;
        for (aParIdx = 1; aParIdx <= myParticlesCount; ++aParIdx)
        {
            // C++ default-constructs the elements; a .NET array of classes holds nulls.
            myParticlesPool[aParIdx] = new PSO_Particle();

            aShiftIdx = (theDimensionCount * 3) * (aParIdx - 1);
            myParticlesPool[aParIdx].Position = new NCollection_Array1<double>(myMemory, aShiftIdx, 0, theDimensionCount - 1);
            myParticlesPool[aParIdx].Velocity = new NCollection_Array1<double>(myMemory, aShiftIdx + theDimensionCount, 0, theDimensionCount - 1);
            myParticlesPool[aParIdx].BestPosition = new NCollection_Array1<double>(myMemory, aShiftIdx + 2 * theDimensionCount, 0, theDimensionCount - 1);
        }
    }

    public PSO_Particle GetParticle(int theIdx)
    {
        return myParticlesPool[theIdx];
    }

    /// <summary>The particle of least distance (the first one if several are equal, as <c>std::min_element</c>).</summary>
    public PSO_Particle GetBestParticle()
    {
        PSO_Particle aBest = myParticlesPool[myParticlesPool.Lower()];
        for (int i = myParticlesPool.Lower() + 1; i <= myParticlesPool.Upper(); i++)
        {
            if (myParticlesPool[i] < aBest)
            {
                aBest = myParticlesPool[i];
            }
        }

        return aBest;
    }

    /// <summary>The particle of greatest distance (the first one if several are equal, as <c>std::max_element</c>).</summary>
    public PSO_Particle GetWorstParticle()
    {
        PSO_Particle aWorst = myParticlesPool[myParticlesPool.Lower()];
        for (int i = myParticlesPool.Lower() + 1; i <= myParticlesPool.Upper(); i++)
        {
            if (aWorst < myParticlesPool[i])
            {
                aWorst = myParticlesPool[i];
            }
        }

        return aWorst;
    }
}
