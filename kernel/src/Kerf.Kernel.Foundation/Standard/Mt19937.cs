// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Foundation.Std;

/// <summary>
/// C++'s <c>std::mt19937</c>, the 32-bit Mersenne Twister (MT19937), giving the same sequence as every standard
/// library for the same seed. OCCT uses it where its results depend on the sequence (NCollection_UBTreeFiller's
/// shuffle, which shapes the tree), so the port's trees are OCCT's.
/// </summary>
/// <remarks>A class: its only purpose is its changing state (as <c>MathUtils.RandomGenerator</c>, decision 58).</remarks>
internal sealed class Mt19937
{
    private const int N = 624;
    private const int M = 397;

    /// <summary><c>std::mt19937::default_seed</c>.</summary>
    public const uint default_seed = 5489u;

    private readonly uint[] _state = new uint[N];
    private int _index;

    /// <summary>A generator seeded with <paramref name="seed"/>.</summary>
    public Mt19937(uint seed = default_seed)
    {
        _state[0] = seed;
        for (int i = 1; i < N; ++i)
        {
            _state[i] = unchecked(1812433253u * (_state[i - 1] ^ (_state[i - 1] >> 30)) + (uint)i);
        }
        _index = N;
    }

    /// <summary>The next value of the sequence (C++'s <c>operator()</c>).</summary>
    public uint Invoke()
    {
        if (_index >= N)
        {
            for (int i = 0; i < N; ++i)
            {
                uint y = (_state[i] & 0x80000000u) | (_state[(i + 1) % N] & 0x7fffffffu);
                _state[i] = _state[(i + M) % N] ^ (y >> 1) ^ ((y & 1u) != 0 ? 0x9908b0dfu : 0u);
            }
            _index = 0;
        }
        uint z = _state[_index++];
        z ^= z >> 11;
        z ^= (z << 7) & 0x9d2c5680u;
        z ^= (z << 15) & 0xefc60000u;
        z ^= z >> 18;
        return z;
    }
}
