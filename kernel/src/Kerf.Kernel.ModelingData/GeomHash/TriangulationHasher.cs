// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_TriangulationHasher.hxx
// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_TriangulationHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>
/// Hasher of Poly_Triangulation, used for mesh deduplication: the hash reads the counts, the presence of UV nodes,
/// the deflection, the end nodes and UV nodes (rounded to <see cref="HashTolerance"/>) and the end triangles, the
/// comparison every node and UV node within <see cref="CompTolerance"/> and every triangle exactly.
/// </summary>
/// <remarks>
/// OCCT's functor is an <see cref="IEqualityComparer{T}"/>: <c>aHasher(x)</c> is
/// <see cref="GetHashCode(Triangulation)"/> and <c>aHasher(a, b)</c> is
/// <see cref="Equals(Triangulation, Triangulation)"/>. The hash values are not OCCT's.
/// </remarks>
internal sealed class TriangulationHasher : IEqualityComparer<Triangulation?>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Computational()).</summary>
    public TriangulationHasher(double compTolerance = HashUtils.Computational,
                                        double hashTolerance = HashUtils.Computational)
    {
        CompTolerance = compTolerance;
        HashTolerance = hashTolerance;
    }

    /// <summary>Hashes the triangulation; 0 for null.</summary>
    public int GetHashCode(Triangulation? tri) => HashUtils.ToInt32(Hash(tri));

    // OCCT's size_t operator()(const handle&)
    internal ulong Hash(Triangulation? tri)
    {
        if (tri is null)
        {
            return 0;
        }

        double factor = 1.0 / HashTolerance;
        Span<ulong> combined = stackalloc ulong[20];
        combined.Clear();
        combined[0] = HashUtils.Hash(tri.NodeCount);
        combined[1] = HashUtils.Hash(tri.TriangleCount);
        combined[2] = HashUtils.Hash(tri.HasUVNodes ? 1 : 0);
        combined[3] = HashUtils.QuantizedHash(tri.Deflection(), factor);
        if (tri.NodeCount > 0)
        {
            Point3D firstNode = tri.Node(1);
            Point3D lastNode = tri.Node(tri.NodeCount);
            combined[4] = HashUtils.QuantizedHash(firstNode.X, factor);
            combined[5] = HashUtils.QuantizedHash(firstNode.Y, factor);
            combined[6] = HashUtils.QuantizedHash(firstNode.Z, factor);
            combined[7] = HashUtils.QuantizedHash(lastNode.X, factor);
            combined[8] = HashUtils.QuantizedHash(lastNode.Y, factor);
            combined[9] = HashUtils.QuantizedHash(lastNode.Z, factor);
        }

        if (tri.TriangleCount > 0)
        {
            tri.Triangle(1).Get(out int n1, out int n2, out int n3);
            combined[10] = HashUtils.Hash(n1);
            combined[11] = HashUtils.Hash(n2);
            combined[12] = HashUtils.Hash(n3);
            tri.Triangle(tri.TriangleCount).Get(out n1, out n2, out n3);
            combined[13] = HashUtils.Hash(n1);
            combined[14] = HashUtils.Hash(n2);
            combined[15] = HashUtils.Hash(n3);
        }

        if (tri.HasUVNodes && tri.NodeCount > 0)
        {
            Point2D firstUV = tri.UVNode(1);
            Point2D lastUV = tri.UVNode(tri.NodeCount);
            combined[16] = HashUtils.QuantizedHash(firstUV.X, factor);
            combined[17] = HashUtils.QuantizedHash(firstUV.Y, factor);
            combined[18] = HashUtils.QuantizedHash(lastUV.X, factor);
            combined[19] = HashUtils.QuantizedHash(lastUV.Y, factor);
        }

        return HashUtils.HashBytes(combined);
    }

    /// <summary>Compares two triangulations node by node and triangle by triangle; null equals only null.</summary>
    public bool Equals(Triangulation? tri1, Triangulation? tri2)
    {
        if (tri1 is null || tri2 is null)
        {
            return tri1 is null && tri2 is null;
        }

        if (tri1.NodeCount != tri2.NodeCount)
        {
            return false;
        }

        if (tri1.TriangleCount != tri2.TriangleCount)
        {
            return false;
        }

        if (tri1.HasUVNodes != tri2.HasUVNodes)
        {
            return false;
        }

        if (Math.Abs(tri1.Deflection() - tri2.Deflection()) > CompTolerance)
        {
            return false;
        }

        for (int idx = 1; idx <= tri1.NodeCount; ++idx)
        {
            Point3D p1 = tri1.Node(idx);
            Point3D p2 = tri2.Node(idx);
            if (Math.Abs(p1.X - p2.X) > CompTolerance || Math.Abs(p1.Y - p2.Y) > CompTolerance
                || Math.Abs(p1.Z - p2.Z) > CompTolerance)
            {
                return false;
            }
        }

        for (int idx = 1; idx <= tri1.TriangleCount; ++idx)
        {
            tri1.Triangle(idx).Get(out int n11, out int n12, out int n13);
            tri2.Triangle(idx).Get(out int n21, out int n22, out int n23);
            if (n11 != n21 || n12 != n22 || n13 != n23)
            {
                return false;
            }
        }

        if (tri1.HasUVNodes)
        {
            for (int idx = 1; idx <= tri1.NodeCount; ++idx)
            {
                Point2D u1 = tri1.UVNode(idx);
                Point2D u2 = tri2.UVNode(idx);
                if (Math.Abs(u1.X - u2.X) > CompTolerance || Math.Abs(u1.Y - u2.Y) > CompTolerance)
                {
                    return false;
                }
            }
        }

        return true;
    }
}
