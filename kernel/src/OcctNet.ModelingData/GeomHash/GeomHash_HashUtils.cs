// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The hashing primitives the GeomHash and Geom2dHash hashers use in place of OCCT's <c>opencascade::hash</c> and
/// <c>opencascade::hashBytes</c> (Standard_HashUtils): an integer hashes to itself, and an array of hashes combines
/// with 64-bit FNV-1a over its bytes. Deterministic across runs and platforms; the values are not OCCT's.
/// </summary>
internal static class GeomHash_HashUtils
{
    private const ulong FnvOffsetBasis = 14695981039346656037UL;
    private const ulong FnvPrime = 1099511628211UL;

    /// <summary><c>Precision::Computational()</c> (RealEpsilon()), the default tolerances of the mesh hashers.</summary>
    public const double Computational = 2.220446049250313E-16;

    /// <summary>The mesh hashers' <c>quantizedHash</c>: the hash of the value rounded to a multiple of 1 / theFactor.</summary>
    public static ulong QuantizedHash(double theValue, double theFactor) => Hash((long)Round(theValue * theFactor));

    /// <summary><c>std::round</c>: halfway cases away from zero.</summary>
    public static double Round(double theValue) => Math.Round(theValue, MidpointRounding.AwayFromZero);

    /// <summary><c>opencascade::hash</c> of an integer.</summary>
    public static ulong Hash(long theValue) => unchecked((ulong)theValue);

    /// <summary><c>opencascade::hash</c> of an integer.</summary>
    public static ulong Hash(int theValue) => unchecked((ulong)(long)theValue);

    /// <summary>A string's hash (OCCT's <c>Standard_CStringHasher</c> of a type name): FNV-1a over its UTF-16 code units.</summary>
    public static ulong Hash(string theValue)
    {
        ulong aHash = FnvOffsetBasis;
        foreach (char aChar in theValue)
        {
            aHash = (aHash ^ aChar) * FnvPrime;
        }

        return aHash;
    }

    /// <summary><c>opencascade::hashBytes</c> of an array of hashes: FNV-1a over the bytes of the values.</summary>
    public static ulong HashBytes(ReadOnlySpan<ulong> theHashes)
    {
        ulong aHash = FnvOffsetBasis;
        foreach (ulong aValue in theHashes)
        {
            for (int aShift = 0; aShift < 64; aShift += 8)
            {
                aHash = (aHash ^ ((aValue >> aShift) & 0xFF)) * FnvPrime;
            }
        }

        return aHash;
    }

    /// <summary>Folds a 64-bit hash into a .NET hash code; 0 stays 0.</summary>
    public static int ToInt32(ulong theHash) => unchecked((int)(theHash ^ (theHash >> 32)));
}
