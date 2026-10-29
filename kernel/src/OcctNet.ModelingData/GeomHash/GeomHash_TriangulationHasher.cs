// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_TriangulationHasher.hxx
// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_TriangulationHasher.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Hasher of Poly_Triangulation, used for mesh deduplication: the hash reads the counts, the presence of UV nodes,
/// the deflection, the end nodes and UV nodes (rounded to <see cref="HashTolerance"/>) and the end triangles, the
/// comparison every node and UV node within <see cref="CompTolerance"/> and every triangle exactly.
/// </summary>
/// <remarks>
/// OCCT's functor is an <see cref="IEqualityComparer{T}"/>: <c>aHasher(x)</c> is
/// <see cref="GetHashCode(Poly_Triangulation)"/> and <c>aHasher(a, b)</c> is
/// <see cref="Equals(Poly_Triangulation, Poly_Triangulation)"/>. The hash values are not OCCT's.
/// </remarks>
public sealed class GeomHash_TriangulationHasher : IEqualityComparer<Poly_Triangulation?>
{
    /// <summary>The tolerance of the comparison of numeric fields.</summary>
    public readonly double CompTolerance;

    /// <summary>The step to which numeric fields are rounded before hashing.</summary>
    public readonly double HashTolerance;

    /// <summary>A hasher with the given tolerances (OCCT's defaults: Precision::Computational()).</summary>
    public GeomHash_TriangulationHasher(double theCompTolerance = GeomHash_HashUtils.Computational,
                                        double theHashTolerance = GeomHash_HashUtils.Computational)
    {
        CompTolerance = theCompTolerance;
        HashTolerance = theHashTolerance;
    }

    /// <summary>Hashes the triangulation; 0 for null.</summary>
    public int GetHashCode(Poly_Triangulation? theTri) => GeomHash_HashUtils.ToInt32(Hash(theTri));

    // OCCT's size_t operator()(const handle&)
    internal ulong Hash(Poly_Triangulation? theTri)
    {
        if (theTri is null)
        {
            return 0;
        }

        double aFactor = 1.0 / HashTolerance;
        Span<ulong> aCombined = stackalloc ulong[20];
        aCombined.Clear();
        aCombined[0] = GeomHash_HashUtils.Hash(theTri.NbNodes());
        aCombined[1] = GeomHash_HashUtils.Hash(theTri.NbTriangles());
        aCombined[2] = GeomHash_HashUtils.Hash(theTri.HasUVNodes() ? 1 : 0);
        aCombined[3] = GeomHash_HashUtils.QuantizedHash(theTri.Deflection(), aFactor);
        if (theTri.NbNodes() > 0)
        {
            gp_Pnt aFirstNode = theTri.Node(1);
            gp_Pnt aLastNode = theTri.Node(theTri.NbNodes());
            aCombined[4] = GeomHash_HashUtils.QuantizedHash(aFirstNode.X(), aFactor);
            aCombined[5] = GeomHash_HashUtils.QuantizedHash(aFirstNode.Y(), aFactor);
            aCombined[6] = GeomHash_HashUtils.QuantizedHash(aFirstNode.Z(), aFactor);
            aCombined[7] = GeomHash_HashUtils.QuantizedHash(aLastNode.X(), aFactor);
            aCombined[8] = GeomHash_HashUtils.QuantizedHash(aLastNode.Y(), aFactor);
            aCombined[9] = GeomHash_HashUtils.QuantizedHash(aLastNode.Z(), aFactor);
        }

        if (theTri.NbTriangles() > 0)
        {
            theTri.Triangle(1).Get(out int aN1, out int aN2, out int aN3);
            aCombined[10] = GeomHash_HashUtils.Hash(aN1);
            aCombined[11] = GeomHash_HashUtils.Hash(aN2);
            aCombined[12] = GeomHash_HashUtils.Hash(aN3);
            theTri.Triangle(theTri.NbTriangles()).Get(out aN1, out aN2, out aN3);
            aCombined[13] = GeomHash_HashUtils.Hash(aN1);
            aCombined[14] = GeomHash_HashUtils.Hash(aN2);
            aCombined[15] = GeomHash_HashUtils.Hash(aN3);
        }

        if (theTri.HasUVNodes() && theTri.NbNodes() > 0)
        {
            gp_Pnt2d aFirstUV = theTri.UVNode(1);
            gp_Pnt2d aLastUV = theTri.UVNode(theTri.NbNodes());
            aCombined[16] = GeomHash_HashUtils.QuantizedHash(aFirstUV.X(), aFactor);
            aCombined[17] = GeomHash_HashUtils.QuantizedHash(aFirstUV.Y(), aFactor);
            aCombined[18] = GeomHash_HashUtils.QuantizedHash(aLastUV.X(), aFactor);
            aCombined[19] = GeomHash_HashUtils.QuantizedHash(aLastUV.Y(), aFactor);
        }

        return GeomHash_HashUtils.HashBytes(aCombined);
    }

    /// <summary>Compares two triangulations node by node and triangle by triangle; null equals only null.</summary>
    public bool Equals(Poly_Triangulation? theTri1, Poly_Triangulation? theTri2)
    {
        if (theTri1 is null || theTri2 is null)
        {
            return theTri1 is null && theTri2 is null;
        }

        if (theTri1.NbNodes() != theTri2.NbNodes())
        {
            return false;
        }

        if (theTri1.NbTriangles() != theTri2.NbTriangles())
        {
            return false;
        }

        if (theTri1.HasUVNodes() != theTri2.HasUVNodes())
        {
            return false;
        }

        if (Math.Abs(theTri1.Deflection() - theTri2.Deflection()) > CompTolerance)
        {
            return false;
        }

        for (int aIdx = 1; aIdx <= theTri1.NbNodes(); ++aIdx)
        {
            gp_Pnt aP1 = theTri1.Node(aIdx);
            gp_Pnt aP2 = theTri2.Node(aIdx);
            if (Math.Abs(aP1.X() - aP2.X()) > CompTolerance || Math.Abs(aP1.Y() - aP2.Y()) > CompTolerance
                || Math.Abs(aP1.Z() - aP2.Z()) > CompTolerance)
            {
                return false;
            }
        }

        for (int aIdx = 1; aIdx <= theTri1.NbTriangles(); ++aIdx)
        {
            theTri1.Triangle(aIdx).Get(out int aN11, out int aN12, out int aN13);
            theTri2.Triangle(aIdx).Get(out int aN21, out int aN22, out int aN23);
            if (aN11 != aN21 || aN12 != aN22 || aN13 != aN23)
            {
                return false;
            }
        }

        if (theTri1.HasUVNodes())
        {
            for (int aIdx = 1; aIdx <= theTri1.NbNodes(); ++aIdx)
            {
                gp_Pnt2d aU1 = theTri1.UVNode(aIdx);
                gp_Pnt2d aU2 = theTri2.UVNode(aIdx);
                if (Math.Abs(aU1.X() - aU2.X()) > CompTolerance || Math.Abs(aU1.Y() - aU2.Y()) > CompTolerance)
                {
                    return false;
                }
            }
        }

        return true;
    }
}
