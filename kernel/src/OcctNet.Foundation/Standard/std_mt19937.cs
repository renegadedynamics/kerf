// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// C++'s <c>std::mt19937</c>, the 32-bit Mersenne Twister (MT19937), giving the same sequence as every standard
/// library for the same seed. OCCT uses it where its results depend on the sequence (NCollection_UBTreeFiller's
/// shuffle, which shapes the tree), so the port's trees are OCCT's.
/// </summary>
/// <remarks>A class: its only purpose is its changing state (as <c>MathUtils.RandomGenerator</c>, decision 58).</remarks>
public sealed class std_mt19937
{
    private const int N = 624;
    private const int M = 397;

    /// <summary><c>std::mt19937::default_seed</c>.</summary>
    public const uint default_seed = 5489u;

    private readonly uint[] myState = new uint[N];
    private int myIndex;

    /// <summary>A generator seeded with <paramref name="theSeed"/>.</summary>
    public std_mt19937(uint theSeed = default_seed)
    {
        myState[0] = theSeed;
        for (int i = 1; i < N; ++i)
        {
            myState[i] = unchecked(1812433253u * (myState[i - 1] ^ (myState[i - 1] >> 30)) + (uint)i);
        }
        myIndex = N;
    }

    /// <summary>The next value of the sequence (C++'s <c>operator()</c>).</summary>
    public uint Invoke()
    {
        if (myIndex >= N)
        {
            for (int i = 0; i < N; ++i)
            {
                uint y = (myState[i] & 0x80000000u) | (myState[(i + 1) % N] & 0x7fffffffu);
                myState[i] = myState[(i + M) % N] ^ (y >> 1) ^ ((y & 1u) != 0 ? 0x9908b0dfu : 0u);
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
}
