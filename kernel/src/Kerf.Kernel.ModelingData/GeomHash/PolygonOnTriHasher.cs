// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_PolygonOnTriHasher.hxx
// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_PolygonOnTriHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>
/// The key of <see cref="PolygonOnTriHasher"/>: a polygon on a triangulation and an identifier of that
/// triangulation, since the polygon's node indices only mean something with it.
/// </summary>
/// <remarks>
/// OCCT's aggregate struct with public fields; its own equality (as a record struct) compares the polygon by
/// reference, the hasher compares it by value.
/// </remarks>
internal readonly record struct PolygonOnTriHashKey(PolygonOnTriangulation? Poly, uint TriRepId);

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
internal sealed class PolygonOnTriHasher : IEqualityComparer<global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHashKey>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Computational()).</summary>
    public PolygonOnTriHasher(double compTolerance = HashUtils.Computational,
                                       double hashTolerance = HashUtils.Computational)
    {
        CompTolerance = compTolerance;
        HashTolerance = hashTolerance;
    }

    /// <summary>Hashes the key.</summary>
    public int GetHashCode(global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHashKey key) => HashUtils.ToInt32(Hash(key));

    // OCCT's size_t operator()(const PolygonOnTriHashKey&)
    internal ulong Hash(in global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHashKey key)
    {
        PolygonOnTriangulation? poly = key.Poly;
        Span<ulong> combined = stackalloc ulong[9];
        combined.Clear();
        combined[0] = HashUtils.Hash((long)key.TriRepId);
        if (poly is null)
        {
            combined[1] = HashUtils.Hash(1);
            return HashUtils.HashBytes(combined);
        }

        double factor = 1.0 / HashTolerance;
        combined[2] = HashUtils.Hash(poly.NodeCount);
        combined[3] = HashUtils.Hash(poly.HasParameters ? 1 : 0);
        combined[4] = HashUtils.QuantizedHash(poly.Deflection(), factor);
        if (poly.NodeCount > 0)
        {
            combined[5] = HashUtils.Hash(poly.Node(1));
            combined[6] = HashUtils.Hash(poly.Node(poly.NodeCount));
            if (poly.HasParameters)
            {
                combined[7] = HashUtils.QuantizedHash(poly.Parameter(1), factor);
                combined[8] = HashUtils.QuantizedHash(poly.Parameter(poly.NodeCount), factor);
            }
        }

        return HashUtils.HashBytes(combined);
    }

    /// <summary>Compares two keys: the same triangulation identifier and equal polygons (null equals only null).</summary>
    public bool Equals(global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHashKey key1, global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHashKey key2)
    {
        PolygonOnTriangulation? poly1 = key1.Poly;
        PolygonOnTriangulation? poly2 = key2.Poly;
        if (key1.TriRepId != key2.TriRepId)
        {
            return false;
        }

        if (poly1 is null || poly2 is null)
        {
            return poly1 is null && poly2 is null;
        }

        if (poly1.NodeCount != poly2.NodeCount)
        {
            return false;
        }

        if (poly1.HasParameters != poly2.HasParameters)
        {
            return false;
        }

        if (Math.Abs(poly1.Deflection() - poly2.Deflection()) > CompTolerance)
        {
            return false;
        }

        for (int idx = 1; idx <= poly1.NodeCount; ++idx)
        {
            if (poly1.Node(idx) != poly2.Node(idx))
            {
                return false;
            }
        }

        if (poly1.HasParameters)
        {
            for (int idx = 1; idx <= poly1.NodeCount; ++idx)
            {
                if (Math.Abs(poly1.Parameter(idx) - poly2.Parameter(idx)) > CompTolerance)
                {
                    return false;
                }
            }
        }

        return true;
    }
}
