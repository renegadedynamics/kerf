// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Primes.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Primes.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Collections;

/// <summary>
/// The bucket counts of OCCT's hash maps. The port's general maps use .NET's dictionaries and don't need them;
/// containers whose iteration or lookup order is part of OCCT's results (<see cref="NCollection_PackedMapOfInteger"/>,
/// Poly_MergeNodesTool's node map) reproduce OCCT's buckets with them.
/// </summary>
internal static class Primes
{
    private static readonly int[] THE_PRIME_VECTOR =
    [
        101, 1009, 2003, 5003, 10007, 20011, 37003, 57037,
        65003, 100019, 209953, 472393, 995329, 2359297, 4478977, 9437185,
        17915905, 35831809, 71663617, 150994945, 301989889, 573308929, 1019215873, 2038431745,
    ];

    /// <summary>The smallest bucket count in OCCT's table greater than <paramref name="n"/>, or theN + 1 beyond it.</summary>
    public static long NextPrimeForMap(long n)
    {
        // std::lower_bound(..., theN + 1)
        foreach (int prime in THE_PRIME_VECTOR)
        {
            if (prime >= n + 1)
            {
                return prime;
            }
        }

        // Return theN + 1 if requested size exceeds the largest available prime.
        return n + 1;
    }
}
