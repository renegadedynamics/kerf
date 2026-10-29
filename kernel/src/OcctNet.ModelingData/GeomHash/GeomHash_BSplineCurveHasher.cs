// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_BSplineCurveHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom_BSplineCurve: the hash reads the metadata only, the comparison the knots, poles and weights.</summary>
internal readonly struct GeomHash_BSplineCurveHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the B-spline curve metadata only.
    public ulong Hash(Geom_BSplineCurve theCurve)
    {
        ReadOnlySpan<ulong> aHashes =
        [
            GeomHash_HashUtils.Hash(theCurve.Degree()),
            GeomHash_HashUtils.Hash(theCurve.NbPoles()),
            GeomHash_HashUtils.Hash(theCurve.NbKnots()),
            GeomHash_HashUtils.Hash(theCurve.IsRational() ? 1 : 0)
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two B-spline curves by full geometric data.
    public bool IsEqual(Geom_BSplineCurve theCurve1, Geom_BSplineCurve theCurve2)
    {
        // Compare degrees
        if (theCurve1.Degree() != theCurve2.Degree())
        {
            return false;
        }

        // Compare knot counts
        if (theCurve1.NbKnots() != theCurve2.NbKnots())
        {
            return false;
        }

        // Compare knots and multiplicities
        for (int i = 1; i <= theCurve1.NbKnots(); ++i)
        {
            if (Math.Abs(theCurve1.Knot(i) - theCurve2.Knot(i)) > CompTolerance
                || theCurve1.Multiplicity(i) != theCurve2.Multiplicity(i))
            {
                return false;
            }
        }

        // Compare rationality
        if (theCurve1.IsRational() != theCurve2.IsRational())
        {
            return false;
        }

        // OCCT compares neither the periodicity nor the pole counts: a periodic curve has more poles than a
        // non-periodic one with the same knots, and the loops below then read past the second curve's poles
        // (OCCT's release build reads out of its array; Pole throws Standard_OutOfRange here)
        var aPointHasher = new GeomHash_PointHasher(CompTolerance, HashTolerance);

        // Compare poles
        for (int i = 1; i <= theCurve1.NbPoles(); ++i)
        {
            if (!aPointHasher.IsEqual(theCurve1.Pole(i), theCurve2.Pole(i)))
            {
                return false;
            }
        }

        // Compare weights if rational
        if (theCurve1.IsRational())
        {
            for (int i = 1; i <= theCurve1.NbPoles(); ++i)
            {
                if (Math.Abs(theCurve1.Weight(i) - theCurve2.Weight(i)) > CompTolerance)
                {
                    return false;
                }
            }
        }

        return true;
    }
}
