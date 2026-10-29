// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_BSplineSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom_BSplineSurface: the hash reads the metadata only, the comparison the knots, poles and weights.</summary>
internal readonly struct GeomHash_BSplineSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the B-spline surface metadata only.
    public ulong Hash(Geom_BSplineSurface theSurface)
    {
        ReadOnlySpan<ulong> aHashes =
        [
            GeomHash_HashUtils.Hash(theSurface.UDegree()),
            GeomHash_HashUtils.Hash(theSurface.VDegree()),
            GeomHash_HashUtils.Hash(theSurface.NbUPoles()),
            GeomHash_HashUtils.Hash(theSurface.NbVPoles()),
            GeomHash_HashUtils.Hash(theSurface.NbUKnots()),
            GeomHash_HashUtils.Hash(theSurface.NbVKnots()),
            GeomHash_HashUtils.Hash((theSurface.IsURational() ? 1 : 0) | ((theSurface.IsVRational() ? 1 : 0) << 1))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two B-spline surfaces by full geometric data.
    public bool IsEqual(Geom_BSplineSurface theSurface1, Geom_BSplineSurface theSurface2)
    {
        // Compare degrees
        if (theSurface1.UDegree() != theSurface2.UDegree() || theSurface1.VDegree() != theSurface2.VDegree())
        {
            return false;
        }

        // Compare knot counts
        if (theSurface1.NbUKnots() != theSurface2.NbUKnots() || theSurface1.NbVKnots() != theSurface2.NbVKnots())
        {
            return false;
        }

        // Compare U knots and multiplicities
        for (int i = 1; i <= theSurface1.NbUKnots(); ++i)
        {
            if (Math.Abs(theSurface1.UKnot(i) - theSurface2.UKnot(i)) > CompTolerance
                || theSurface1.UMultiplicity(i) != theSurface2.UMultiplicity(i))
            {
                return false;
            }
        }

        // Compare V knots and multiplicities
        for (int i = 1; i <= theSurface1.NbVKnots(); ++i)
        {
            if (Math.Abs(theSurface1.VKnot(i) - theSurface2.VKnot(i)) > CompTolerance
                || theSurface1.VMultiplicity(i) != theSurface2.VMultiplicity(i))
            {
                return false;
            }
        }

        // Compare rationality
        if (theSurface1.IsURational() != theSurface2.IsURational()
            || theSurface1.IsVRational() != theSurface2.IsVRational())
        {
            return false;
        }

        // OCCT compares neither the periodicities nor the pole counts: with the same knots, a periodic direction
        // has more poles than a non-periodic one, and the loops below then read past the second surface's poles
        // (OCCT's release build reads out of its array; Pole throws Standard_OutOfRange here)
        var aPointHasher = new GeomHash_PointHasher(CompTolerance, HashTolerance);

        // Compare poles
        for (int i = 1; i <= theSurface1.NbUPoles(); ++i)
        {
            for (int j = 1; j <= theSurface1.NbVPoles(); ++j)
            {
                if (!aPointHasher.IsEqual(theSurface1.Pole(i, j), theSurface2.Pole(i, j)))
                {
                    return false;
                }
            }
        }

        // Compare weights if rational
        if (theSurface1.IsURational() || theSurface1.IsVRational())
        {
            for (int i = 1; i <= theSurface1.NbUPoles(); ++i)
            {
                for (int j = 1; j <= theSurface1.NbVPoles(); ++j)
                {
                    if (Math.Abs(theSurface1.Weight(i, j) - theSurface2.Weight(i, j)) > CompTolerance)
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }
}
