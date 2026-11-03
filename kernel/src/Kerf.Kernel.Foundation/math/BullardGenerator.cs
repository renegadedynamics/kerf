// Port of: src/FoundationClasses/TKMath/math/math_BullardGenerator.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics;

/// <summary>A fast random number generator (the algorithm proposed by Ian C. Bullard).</summary>
public class BullardGenerator
{
    private uint _stateHi;
    private uint _stateLo;

    public BullardGenerator(uint seed = 1)
    {
        _stateHi = seed;
        SetSeed(seed);
    }

    /// <summary>Sets a new seed, resetting the state.</summary>
    public void SetSeed(uint seed = 1)
    {
        _stateHi = seed;
        _stateLo = seed ^ 0x49616E42;
    }

    /// <summary>The next integer value.</summary>
    public uint NextInt()
    {
        unchecked
        {
            _stateHi = (_stateHi >> 2) + (_stateHi << 2);
            _stateHi += _stateLo;
            _stateLo += _stateHi;
        }

        return _stateHi;
    }

    /// <summary>The next real value, in [0, 1].</summary>
    public double NextReal() => NextInt() / (double)0xFFFFFFFFu;
}
