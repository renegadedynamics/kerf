// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_PolygonOnTriHasher.hxx
// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_PolygonOnTriHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

namespace Occt;

/// <summary>
/// The key of <see cref="GeomHash_PolygonOnTriHasher"/>: a polygon on a triangulation and an identifier of that
/// triangulation, since the polygon's node indices only mean something with it.
/// </summary>
/// <remarks>
/// OCCT's aggregate struct with public fields; its own equality (as a record struct) compares the polygon by
/// reference, the hasher compares it by value.
/// </remarks>
public readonly record struct PolygonOnTriHashKey(PolygonOnTriangulation? Poly, uint TriRepId);

/// <summary>
/// Hasher of <see cref="PolygonOnTriHashKey"/>, used for mesh deduplication: the hash reads the triangulation
/// identifier, the node count, the presence of parameters, the deflection, the end node indices and the end
/// parameters (rounded to <see cref="HashTolerance"/>), the comparison every node index exactly and every parameter
/// within <see cref="CompTolerance"/>.
/// </summary>
/// <remarks>
/// OCCT's functor is an <see cref="IEqualityComparer{T}"/>: <c>aHasher(x)</c> is
/// <see cref="GetHashCode(PolygonOnTriHashKey)"/> and <c>aHasher(a, b)</c> is
/// <see cref="Equals(PolygonOnTriHashKey, PolygonOnTriHashKey)"/>. The hash values are not OCCT's.
/// </remarks>
public sealed class GeomHash_PolygonOnTriHasher : IEqualityComparer<PolygonOnTriHashKey>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Computational()).</summary>
    public GeomHash_PolygonOnTriHasher(double theCompTolerance = GeomHash_HashUtils.Computational,
                                       double theHashTolerance = GeomHash_HashUtils.Computational)
    {
        CompTolerance = theCompTolerance;
        HashTolerance = theHashTolerance;
    }

    /// <summary>Hashes the key.</summary>
    public int GetHashCode(PolygonOnTriHashKey theKey) => GeomHash_HashUtils.ToInt32(Hash(theKey));

    // OCCT's size_t operator()(const PolygonOnTriHashKey&)
    internal ulong Hash(in PolygonOnTriHashKey theKey)
    {
        PolygonOnTriangulation? aPoly = theKey.Poly;
        Span<ulong> aCombined = stackalloc ulong[9];
        aCombined.Clear();
        aCombined[0] = GeomHash_HashUtils.Hash((long)theKey.TriRepId);
        if (aPoly is null)
        {
            aCombined[1] = GeomHash_HashUtils.Hash(1);
            return GeomHash_HashUtils.HashBytes(aCombined);
        }

        double aFactor = 1.0 / HashTolerance;
        aCombined[2] = GeomHash_HashUtils.Hash(aPoly.NodeCount);
        aCombined[3] = GeomHash_HashUtils.Hash(aPoly.HasParameters ? 1 : 0);
        aCombined[4] = GeomHash_HashUtils.QuantizedHash(aPoly.Deflection(), aFactor);
        if (aPoly.NodeCount > 0)
        {
            aCombined[5] = GeomHash_HashUtils.Hash(aPoly.Node(1));
            aCombined[6] = GeomHash_HashUtils.Hash(aPoly.Node(aPoly.NodeCount));
            if (aPoly.HasParameters)
            {
                aCombined[7] = GeomHash_HashUtils.QuantizedHash(aPoly.Parameter(1), aFactor);
                aCombined[8] = GeomHash_HashUtils.QuantizedHash(aPoly.Parameter(aPoly.NodeCount), aFactor);
            }
        }

        return GeomHash_HashUtils.HashBytes(aCombined);
    }

    /// <summary>Compares two keys: the same triangulation identifier and equal polygons (null equals only null).</summary>
    public bool Equals(PolygonOnTriHashKey theKey1, PolygonOnTriHashKey theKey2)
    {
        PolygonOnTriangulation? aPoly1 = theKey1.Poly;
        PolygonOnTriangulation? aPoly2 = theKey2.Poly;
        if (theKey1.TriRepId != theKey2.TriRepId)
        {
            return false;
        }

        if (aPoly1 is null || aPoly2 is null)
        {
            return aPoly1 is null && aPoly2 is null;
        }

        if (aPoly1.NodeCount != aPoly2.NodeCount)
        {
            return false;
        }

        if (aPoly1.HasParameters != aPoly2.HasParameters)
        {
            return false;
        }

        if (Math.Abs(aPoly1.Deflection() - aPoly2.Deflection()) > CompTolerance)
        {
            return false;
        }

        for (int aIdx = 1; aIdx <= aPoly1.NodeCount; ++aIdx)
        {
            if (aPoly1.Node(aIdx) != aPoly2.Node(aIdx))
            {
                return false;
            }
        }

        if (aPoly1.HasParameters)
        {
            for (int aIdx = 1; aIdx <= aPoly1.NodeCount; ++aIdx)
            {
                if (Math.Abs(aPoly1.Parameter(aIdx) - aPoly2.Parameter(aIdx)) > CompTolerance)
                {
                    return false;
                }
            }
        }

        return true;
    }
}
