// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Random.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

public static partial class MathUtils
{
    /// <summary>Pseudo-random number generator based on xoshiro256**, seeded through SplitMix64.</summary>
    /// <remarks>A class, not a struct: its whole purpose is its changing state, which a struct would lose to
    /// defensive copies.</remarks>
    public sealed class RandomGenerator
    {
        private readonly ulong[] myState = new ulong[4]; // 256-bit generator state

        /// <summary>Initialize with a seed value.</summary>
        public RandomGenerator(ulong theSeed = 1)
        {
            SetSeed(theSeed);
        }

        /// <summary>Re-seed the generator.</summary>
        public void SetSeed(ulong theSeed)
        {
            // Use SplitMix64 to initialize the state from a single seed.
            ulong aSeedState = theSeed;
            myState[0] = splitmix64(ref aSeedState);
            myState[1] = splitmix64(ref aSeedState);
            myState[2] = splitmix64(ref aSeedState);
            myState[3] = splitmix64(ref aSeedState);
        }

        /// <summary>Generate next 64-bit unsigned integer.</summary>
        public ulong NextInt()
        {
            ulong aResult = rotl(myState[1] * 5, 7) * 9;
            ulong aT = myState[1] << 17;

            myState[2] ^= myState[0];
            myState[3] ^= myState[1];
            myState[1] ^= myState[2];
            myState[0] ^= myState[3];

            myState[2] ^= aT;
            myState[3] = rotl(myState[3], 45);

            return aResult;
        }

        /// <summary>Generate next double in [0, 1) with 53 bits of randomness.</summary>
        public double NextReal() => (double)(NextInt() >> 11) * (1.0 / 9007199254740992.0);

        private static ulong rotl(ulong theX, int theK)
        {
            return (theX << theK) | (theX >> (64 - theK));
        }

        private static ulong splitmix64(ref ulong theState)
        {
            unchecked
            {
                theState += 0x9e3779b97f4a7c15UL;
                ulong aZ = theState;
                aZ = (aZ ^ (aZ >> 30)) * 0xbf58476d1ce4e5b9UL;
                aZ = (aZ ^ (aZ >> 27)) * 0x94d049bb133111ebUL;
                return aZ ^ (aZ >> 31);
            }
        }
    }
}
