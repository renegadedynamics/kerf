// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_Polygon2DHasher.hxx
// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_Polygon2DHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Hasher of Poly_Polygon2D, used for mesh deduplication: the hash reads the node count, the deflection and the end
/// nodes (rounded to <see cref="HashTolerance"/>), the comparison every node within <see cref="CompTolerance"/>.
/// </summary>
/// <remarks>
/// OCCT's functor is an <see cref="IEqualityComparer{T}"/>: <c>aHasher(x)</c> is <see cref="GetHashCode(Poly_Polygon2D)"/>
/// and <c>aHasher(a, b)</c> is <see cref="Equals(Poly_Polygon2D, Poly_Polygon2D)"/>. The hash values are not OCCT's.
/// </remarks>
public sealed class GeomHash_Polygon2DHasher : IEqualityComparer<Poly_Polygon2D?>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Computational()).</summary>
    public GeomHash_Polygon2DHasher(double theCompTolerance = GeomHash_HashUtils.Computational,
                                    double theHashTolerance = GeomHash_HashUtils.Computational)
    {
        CompTolerance = theCompTolerance;
        HashTolerance = theHashTolerance;
    }

    /// <summary>Hashes the polygon; 0 for null.</summary>
    public int GetHashCode(Poly_Polygon2D? thePoly) => GeomHash_HashUtils.ToInt32(Hash(thePoly));

    // OCCT's size_t operator()(const handle&)
    internal ulong Hash(Poly_Polygon2D? thePoly)
    {
        if (thePoly is null)
        {
            return 0;
        }

        double aFactor = 1.0 / HashTolerance;
        Span<ulong> aCombined = stackalloc ulong[6];
        aCombined.Clear();
        aCombined[0] = GeomHash_HashUtils.Hash(thePoly.NbNodes());
        aCombined[1] = GeomHash_HashUtils.QuantizedHash(thePoly.Deflection(), aFactor);
        if (thePoly.NbNodes() > 0)
        {
            NCollection_Array1<gp_Pnt2d> aNodes = thePoly.Nodes();
            aCombined[2] = GeomHash_HashUtils.QuantizedHash(aNodes.First().X(), aFactor);
            aCombined[3] = GeomHash_HashUtils.QuantizedHash(aNodes.First().Y(), aFactor);
            aCombined[4] = GeomHash_HashUtils.QuantizedHash(aNodes.Last().X(), aFactor);
            aCombined[5] = GeomHash_HashUtils.QuantizedHash(aNodes.Last().Y(), aFactor);
        }

        return GeomHash_HashUtils.HashBytes(aCombined);
    }

    /// <summary>Compares two polygons node by node; null equals only null.</summary>
    public bool Equals(Poly_Polygon2D? thePoly1, Poly_Polygon2D? thePoly2)
    {
        if (thePoly1 is null || thePoly2 is null)
        {
            return thePoly1 is null && thePoly2 is null;
        }

        if (thePoly1.NbNodes() != thePoly2.NbNodes())
        {
            return false;
        }

        if (Math.Abs(thePoly1.Deflection() - thePoly2.Deflection()) > CompTolerance)
        {
            return false;
        }

        NCollection_Array1<gp_Pnt2d> aNodes1 = thePoly1.Nodes();
        NCollection_Array1<gp_Pnt2d> aNodes2 = thePoly2.Nodes();
        for (int aIdx = aNodes1.Lower(); aIdx <= aNodes1.Upper(); ++aIdx)
        {
            gp_Pnt2d aP1 = aNodes1.Value(aIdx);
            gp_Pnt2d aP2 = aNodes2.Value(aIdx);
            if (Math.Abs(aP1.X() - aP2.X()) > CompTolerance || Math.Abs(aP1.Y() - aP2.Y()) > CompTolerance)
            {
                return false;
            }
        }

        return true;
    }
}
