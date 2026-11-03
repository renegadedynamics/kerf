// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_Polygon2DHasher.hxx
// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_Polygon2DHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>
/// Hasher of Poly_Polygon2D, used for mesh deduplication: the hash reads the node count, the deflection and the end
/// nodes (rounded to <see cref="HashTolerance"/>), the comparison every node within <see cref="CompTolerance"/>.
/// </summary>
/// <remarks>
/// OCCT's functor is an <see cref="IEqualityComparer{T}"/>: <c>aHasher(x)</c> is <see cref="GetHashCode(Polygon2D)"/>
/// and <c>aHasher(a, b)</c> is <see cref="Equals(Polygon2D, Polygon2D)"/>. The hash values are not OCCT's.
/// </remarks>
public sealed class Polygon2DHasher : IEqualityComparer<Polygon2D?>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Computational()).</summary>
    public Polygon2DHasher(double compTolerance = HashUtils.Computational,
                                    double hashTolerance = HashUtils.Computational)
    {
        CompTolerance = compTolerance;
        HashTolerance = hashTolerance;
    }

    /// <summary>Hashes the polygon; 0 for null.</summary>
    public int GetHashCode(Polygon2D? poly) => HashUtils.ToInt32(Hash(poly));

    // OCCT's size_t operator()(const handle&)
    internal ulong Hash(Polygon2D? poly)
    {
        if (poly is null)
        {
            return 0;
        }

        double factor = 1.0 / HashTolerance;
        Span<ulong> combined = stackalloc ulong[6];
        combined.Clear();
        combined[0] = HashUtils.Hash(poly.NodeCount);
        combined[1] = HashUtils.QuantizedHash(poly.Deflection(), factor);
        if (poly.NodeCount > 0)
        {
            NCollection_Array1<Point2D> nodes = poly.Nodes();
            combined[2] = HashUtils.QuantizedHash(nodes.First().X, factor);
            combined[3] = HashUtils.QuantizedHash(nodes.First().Y, factor);
            combined[4] = HashUtils.QuantizedHash(nodes.Last().X, factor);
            combined[5] = HashUtils.QuantizedHash(nodes.Last().Y, factor);
        }

        return HashUtils.HashBytes(combined);
    }

    /// <summary>Compares two polygons node by node; null equals only null.</summary>
    public bool Equals(Polygon2D? poly1, Polygon2D? poly2)
    {
        if (poly1 is null || poly2 is null)
        {
            return poly1 is null && poly2 is null;
        }

        if (poly1.NodeCount != poly2.NodeCount)
        {
            return false;
        }

        if (Math.Abs(poly1.Deflection() - poly2.Deflection()) > CompTolerance)
        {
            return false;
        }

        NCollection_Array1<Point2D> nodes1 = poly1.Nodes();
        NCollection_Array1<Point2D> nodes2 = poly2.Nodes();
        for (int idx = nodes1.Lower(); idx <= nodes1.Upper(); ++idx)
        {
            Point2D p1 = nodes1.Value(idx);
            Point2D p2 = nodes2.Value(idx);
            if (Math.Abs(p1.X - p2.X) > CompTolerance || Math.Abs(p1.Y - p2.Y) > CompTolerance)
            {
                return false;
            }
        }

        return true;
    }
}
