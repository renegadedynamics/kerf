// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// C++'s std::mt19937 (the 32-bit Mersenne Twister, same sequence) for the ExtremaPC random tests. Their
/// std::uniform_real_distribution is library-specific, so it is replaced by a simple mapping: the random points
/// differ from OCCT's (decision 60); no check depends on the values.
/// </summary>
internal sealed class Mt19937
{
    private readonly uint[] myState = new uint[624];
    private int myIndex;

    public Mt19937(uint theSeed)
    {
        myState[0] = theSeed;
        for (int i = 1; i < 624; ++i)
        {
            myState[i] = unchecked(1812433253u * (myState[i - 1] ^ (myState[i - 1] >> 30)) + (uint)i);
        }
        myIndex = 624;
    }

    public uint Next()
    {
        if (myIndex >= 624)
        {
            for (int i = 0; i < 624; ++i)
            {
                uint y = (myState[i] & 0x80000000u) | (myState[(i + 1) % 624] & 0x7fffffffu);
                myState[i] = myState[(i + 397) % 624] ^ (y >> 1) ^ ((y & 1u) != 0 ? 0x9908b0dfu : 0u);
            }
            myIndex = 0;
        }
        uint z = myState[myIndex++];
        z ^= z >> 11;
        z ^= (z << 7) & 0x9d2c5680u;
        z ^= (z << 15) & 0xefc60000u;
        z ^= z >> 18;
        return z;
    }

    /// <summary>A real in [theMin, theMax).</summary>
    public double NextReal(double theMin, double theMax) => theMin + (theMax - theMin) * (Next() / 4294967296.0);
}
