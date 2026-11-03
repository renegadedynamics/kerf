// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_Polygon3DHasher.hxx
// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_Polygon3DHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Hasher of Poly_Polygon3D, used for mesh deduplication: the hash reads the node count, the presence of parameters,
/// the deflection, the end nodes and the end parameters (rounded to <see cref="HashTolerance"/>), the comparison
/// every node and parameter within <see cref="CompTolerance"/>.
/// </summary>
/// <remarks>
/// OCCT's functor is an <see cref="IEqualityComparer{T}"/>: <c>aHasher(x)</c> is <see cref="GetHashCode(Polygon3D)"/>
/// and <c>aHasher(a, b)</c> is <see cref="Equals(Polygon3D, Polygon3D)"/>. The hash values are not OCCT's.
/// </remarks>
public sealed class GeomHash_Polygon3DHasher : IEqualityComparer<Polygon3D?>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Computational()).</summary>
    public GeomHash_Polygon3DHasher(double theCompTolerance = GeomHash_HashUtils.Computational,
                                    double theHashTolerance = GeomHash_HashUtils.Computational)
    {
        CompTolerance = theCompTolerance;
        HashTolerance = theHashTolerance;
    }

    /// <summary>Hashes the polygon; 0 for null.</summary>
    public int GetHashCode(Polygon3D? thePoly) => GeomHash_HashUtils.ToInt32(Hash(thePoly));

    // OCCT's size_t operator()(const handle&)
    internal ulong Hash(Polygon3D? thePoly)
    {
        if (thePoly is null)
        {
            return 0;
        }

        double aFactor = 1.0 / HashTolerance;
        Span<ulong> aCombined = stackalloc ulong[11];
        aCombined.Clear();
        aCombined[0] = GeomHash_HashUtils.Hash(thePoly.NodeCount);
        aCombined[1] = GeomHash_HashUtils.Hash(thePoly.HasParameters ? 1 : 0);
        aCombined[2] = GeomHash_HashUtils.QuantizedHash(thePoly.Deflection(), aFactor);
        if (thePoly.NodeCount > 0)
        {
            NCollection_Array1<Point3D> aNodes = thePoly.Nodes();
            aCombined[3] = GeomHash_HashUtils.QuantizedHash(aNodes.First().X(), aFactor);
            aCombined[4] = GeomHash_HashUtils.QuantizedHash(aNodes.First().Y(), aFactor);
            aCombined[5] = GeomHash_HashUtils.QuantizedHash(aNodes.First().Z(), aFactor);
            aCombined[6] = GeomHash_HashUtils.QuantizedHash(aNodes.Last().X(), aFactor);
            aCombined[7] = GeomHash_HashUtils.QuantizedHash(aNodes.Last().Y(), aFactor);
            aCombined[8] = GeomHash_HashUtils.QuantizedHash(aNodes.Last().Z(), aFactor);
            if (thePoly.HasParameters)
            {
                NCollection_Array1<double> aParams = thePoly.Parameters();
                aCombined[9] = GeomHash_HashUtils.QuantizedHash(aParams.First(), aFactor);
                aCombined[10] = GeomHash_HashUtils.QuantizedHash(aParams.Last(), aFactor);
            }
        }

        return GeomHash_HashUtils.HashBytes(aCombined);
    }

    /// <summary>Compares two polygons node by node and parameter by parameter; null equals only null.</summary>
    public bool Equals(Polygon3D? thePoly1, Polygon3D? thePoly2)
    {
        if (thePoly1 is null || thePoly2 is null)
        {
            return thePoly1 is null && thePoly2 is null;
        }

        if (thePoly1.NodeCount != thePoly2.NodeCount)
        {
            return false;
        }

        if (thePoly1.HasParameters != thePoly2.HasParameters)
        {
            return false;
        }

        if (Math.Abs(thePoly1.Deflection() - thePoly2.Deflection()) > CompTolerance)
        {
            return false;
        }

        NCollection_Array1<Point3D> aNodes1 = thePoly1.Nodes();
        NCollection_Array1<Point3D> aNodes2 = thePoly2.Nodes();
        for (int aIdx = aNodes1.Lower(); aIdx <= aNodes1.Upper(); ++aIdx)
        {
            Point3D aP1 = aNodes1.Value(aIdx);
            Point3D aP2 = aNodes2.Value(aIdx);
            if (Math.Abs(aP1.X() - aP2.X()) > CompTolerance || Math.Abs(aP1.Y() - aP2.Y()) > CompTolerance
                || Math.Abs(aP1.Z() - aP2.Z()) > CompTolerance)
            {
                return false;
            }
        }

        if (thePoly1.HasParameters)
        {
            NCollection_Array1<double> aParams1 = thePoly1.Parameters();
            NCollection_Array1<double> aParams2 = thePoly2.Parameters();
            for (int aIdx = aParams1.Lower(); aIdx <= aParams1.Upper(); ++aIdx)
            {
                if (Math.Abs(aParams1.Value(aIdx) - aParams2.Value(aIdx)) > CompTolerance)
                {
                    return false;
                }
            }
        }

        return true;
    }
}
