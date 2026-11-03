// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_Polygon3DHasher.hxx
// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_Polygon3DHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>
/// Hasher of Poly_Polygon3D, used for mesh deduplication: the hash reads the node count, the presence of parameters,
/// the deflection, the end nodes and the end parameters (rounded to <see cref="HashTolerance"/>), the comparison
/// every node and parameter within <see cref="CompTolerance"/>.
/// </summary>
/// <remarks>
/// OCCT's functor is an <see cref="IEqualityComparer{T}"/>: <c>aHasher(x)</c> is <see cref="GetHashCode(Polygon3D)"/>
/// and <c>aHasher(a, b)</c> is <see cref="Equals(Polygon3D, Polygon3D)"/>. The hash values are not OCCT's.
/// </remarks>
internal sealed class Polygon3DHasher : IEqualityComparer<Polygon3D?>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Computational()).</summary>
    public Polygon3DHasher(double compTolerance = HashUtils.Computational,
                                    double hashTolerance = HashUtils.Computational)
    {
        CompTolerance = compTolerance;
        HashTolerance = hashTolerance;
    }

    /// <summary>Hashes the polygon; 0 for null.</summary>
    public int GetHashCode(Polygon3D? poly) => HashUtils.ToInt32(Hash(poly));

    // OCCT's size_t operator()(const handle&)
    internal ulong Hash(Polygon3D? poly)
    {
        if (poly is null)
        {
            return 0;
        }

        double factor = 1.0 / HashTolerance;
        Span<ulong> combined = stackalloc ulong[11];
        combined.Clear();
        combined[0] = HashUtils.Hash(poly.NodeCount);
        combined[1] = HashUtils.Hash(poly.HasParameters ? 1 : 0);
        combined[2] = HashUtils.QuantizedHash(poly.Deflection(), factor);
        if (poly.NodeCount > 0)
        {
            NCollection_Array1<Point3D> nodes = poly.Nodes();
            combined[3] = HashUtils.QuantizedHash(nodes.First().X, factor);
            combined[4] = HashUtils.QuantizedHash(nodes.First().Y, factor);
            combined[5] = HashUtils.QuantizedHash(nodes.First().Z, factor);
            combined[6] = HashUtils.QuantizedHash(nodes.Last().X, factor);
            combined[7] = HashUtils.QuantizedHash(nodes.Last().Y, factor);
            combined[8] = HashUtils.QuantizedHash(nodes.Last().Z, factor);
            if (poly.HasParameters)
            {
                NCollection_Array1<double> aParams = poly.Parameters();
                combined[9] = HashUtils.QuantizedHash(aParams.First(), factor);
                combined[10] = HashUtils.QuantizedHash(aParams.Last(), factor);
            }
        }

        return HashUtils.HashBytes(combined);
    }

    /// <summary>Compares two polygons node by node and parameter by parameter; null equals only null.</summary>
    public bool Equals(Polygon3D? poly1, Polygon3D? poly2)
    {
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

        NCollection_Array1<Point3D> nodes1 = poly1.Nodes();
        NCollection_Array1<Point3D> nodes2 = poly2.Nodes();
        for (int idx = nodes1.Lower(); idx <= nodes1.Upper(); ++idx)
        {
            Point3D p1 = nodes1.Value(idx);
            Point3D p2 = nodes2.Value(idx);
            if (Math.Abs(p1.X - p2.X) > CompTolerance || Math.Abs(p1.Y - p2.Y) > CompTolerance
                || Math.Abs(p1.Z - p2.Z) > CompTolerance)
            {
                return false;
            }
        }

        if (poly1.HasParameters)
        {
            NCollection_Array1<double> params1 = poly1.Parameters();
            NCollection_Array1<double> params2 = poly2.Parameters();
            for (int idx = params1.Lower(); idx <= params1.Upper(); ++idx)
            {
                if (Math.Abs(params1.Value(idx) - params2.Value(idx)) > CompTolerance)
                {
                    return false;
                }
            }
        }

        return true;
    }
}
