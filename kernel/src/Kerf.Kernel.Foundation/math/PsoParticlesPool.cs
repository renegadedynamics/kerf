// Port of: src/FoundationClasses/TKMath/math/math_PSOParticlesPool.hxx
// Port of: src/FoundationClasses/TKMath/math/math_PSOParticlesPool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics;

/// <summary>A particle of <see cref="PsoParticlesPool"/>. Coordinate indexes are 0..dimension-1.</summary>
/// <remarks>
/// A struct in OCCT, but only ever used through pointers into the pool, so a class here. Position, Velocity and
/// BestPosition are views (lower bound 0) over the pool's shared storage, as OCCT's pointers are.
/// </remarks>
public class Particle
{
    public NCollection_Array1<double> Position; // Data for pointers allocated within PSOParticlesPool instance.
    public NCollection_Array1<double> Velocity; // Not need to delete it manually.
    public NCollection_Array1<double> BestPosition;
    public double Distance;
    public double BestDistance;

    public Particle()
    {
        Distance = RealLast();
        BestDistance = RealLast();
        Position = null!;
        Velocity = null!;
        BestPosition = null!;
    }

    /// <summary>OCCT's <c>operator&lt;</c>: compares the particles by their distances.</summary>
    public static bool operator <(Particle left, Particle pnt) => left.Distance < pnt.Distance;

    public static bool operator >(Particle left, Particle pnt) => pnt.Distance < left.Distance;
}

/// <summary>The particle pool of the PSO algorithm (<see cref="Pso"/>). Particle indexes are 1..count.</summary>
public class PsoParticlesPool
{
    private NCollection_Array1<Particle> _particlesPool;
    private NCollection_Array1<double> _memory; // Stores particles vector data.
    private int _particlesCount;
    private int _dimensionCount;

    public PsoParticlesPool(int particlesCount, int dimensionCount)
    {
        _particlesPool = new NCollection_Array1<Particle>(1, particlesCount);
        _memory = new NCollection_Array1<double>(0, particlesCount * (dimensionCount // Position
                                                                         + dimensionCount // Velocity
                                                                         + dimensionCount) // BestPosition
                                                         - 1);
        _particlesCount = particlesCount;
        _dimensionCount = dimensionCount;

        _memory.Init(0.0);

        // Pointers adjusting.
        int parIdx, shiftIdx;
        for (parIdx = 1; parIdx <= _particlesCount; ++parIdx)
        {
            // C++ default-constructs the elements; a .NET array of classes holds nulls.
            _particlesPool[parIdx] = new Particle();

            shiftIdx = (dimensionCount * 3) * (parIdx - 1);
            _particlesPool[parIdx].Position = new NCollection_Array1<double>(_memory, shiftIdx, 0, dimensionCount - 1);
            _particlesPool[parIdx].Velocity = new NCollection_Array1<double>(_memory, shiftIdx + dimensionCount, 0, dimensionCount - 1);
            _particlesPool[parIdx].BestPosition = new NCollection_Array1<double>(_memory, shiftIdx + 2 * dimensionCount, 0, dimensionCount - 1);
        }
    }

    public Particle GetParticle(int idx)
    {
        return _particlesPool[idx];
    }

    /// <summary>The particle of least distance (the first one if several are equal, as <c>std::min_element</c>).</summary>
    public Particle GetBestParticle()
    {
        Particle best = _particlesPool[_particlesPool.Lower()];
        for (int i = _particlesPool.Lower() + 1; i <= _particlesPool.Upper(); i++)
        {
            if (_particlesPool[i] < best)
            {
                best = _particlesPool[i];
            }
        }

        return best;
    }

    /// <summary>The particle of greatest distance (the first one if several are equal, as <c>std::max_element</c>).</summary>
    public Particle GetWorstParticle()
    {
        Particle worst = _particlesPool[_particlesPool.Lower()];
        for (int i = _particlesPool.Lower() + 1; i <= _particlesPool.Upper(); i++)
        {
            if (worst < _particlesPool[i])
            {
                worst = _particlesPool[i];
            }
        }

        return worst;
    }
}
