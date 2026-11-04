// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Random.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Utilities;

internal static partial class MathUtils
{
    /// <summary>Pseudo-random number generator based on xoshiro256**, seeded through SplitMix64.</summary>
    /// <remarks>A class, not a struct: its whole purpose is its changing state, which a struct would lose to
    /// defensive copies.</remarks>
    public sealed class RandomGenerator
    {
        private readonly ulong[] _state = new ulong[4]; // 256-bit generator state

        /// <summary>Initialize with a seed value.</summary>
        public RandomGenerator(ulong seed = 1)
        {
            SetSeed(seed);
        }

        /// <summary>Re-seed the generator.</summary>
        public void SetSeed(ulong seed)
        {
            // Use SplitMix64 to initialize the state from a single seed.
            ulong seedState = seed;
            _state[0] = Splitmix64(ref seedState);
            _state[1] = Splitmix64(ref seedState);
            _state[2] = Splitmix64(ref seedState);
            _state[3] = Splitmix64(ref seedState);
        }

        /// <summary>Generate next 64-bit unsigned integer.</summary>
        public ulong NextInt()
        {
            ulong result = Rotl(_state[1] * 5, 7) * 9;
            ulong t = _state[1] << 17;

            _state[2] ^= _state[0];
            _state[3] ^= _state[1];
            _state[1] ^= _state[2];
            _state[0] ^= _state[3];

            _state[2] ^= t;
            _state[3] = Rotl(_state[3], 45);

            return result;
        }

        /// <summary>Generate next double in [0, 1) with 53 bits of randomness.</summary>
        public double NextReal() => (double)(NextInt() >> 11) * (1.0 / 9007199254740992.0);

        private static ulong Rotl(ulong x, int k)
        {
            return (x << k) | (x >> (64 - k));
        }

        private static ulong Splitmix64(ref ulong state)
        {
            unchecked
            {
                state += 0x9e3779b97f4a7c15UL;
                ulong z = state;
                z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9UL;
                z = (z ^ (z >> 27)) * 0x94d049bb133111ebUL;
                return z ^ (z >> 31);
            }
        }
    }
}
