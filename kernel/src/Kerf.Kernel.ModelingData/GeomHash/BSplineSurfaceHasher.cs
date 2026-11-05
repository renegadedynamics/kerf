// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_BSplineSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Engine;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_BSplineSurface: the hash reads the metadata only, the comparison the knots, poles and weights.</summary>
internal readonly struct BSplineSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the B-spline surface metadata only.
    public ulong Hash(BSplineSurface surface)
    {
        ReadOnlySpan<ulong> hashes =
        [
            HashUtils.Hash(surface.UDegree),
            HashUtils.Hash(surface.VDegree),
            HashUtils.Hash(surface.UPoleCount),
            HashUtils.Hash(surface.VPoleCount),
            HashUtils.Hash(surface.UKnotCount),
            HashUtils.Hash(surface.VKnotCount),
            HashUtils.Hash((surface.IsURational ? 1 : 0) | ((surface.IsVRational ? 1 : 0) << 1))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two B-spline surfaces by full geometric data.
    public bool IsEqual(BSplineSurface surface1, BSplineSurface surface2)
    {
        // Compare degrees
        if (surface1.UDegree != surface2.UDegree || surface1.VDegree != surface2.VDegree)
        {
            return false;
        }

        // Compare knot counts
        if (surface1.UKnotCount != surface2.UKnotCount || surface1.VKnotCount != surface2.VKnotCount)
        {
            return false;
        }

        // Compare U knots and multiplicities
        for (int i = 1; i <= surface1.UKnotCount; ++i)
        {
            if (Math.Abs(surface1.UKnot(i) - surface2.UKnot(i)) > CompTolerance
                || surface1.UMultiplicity(i) != surface2.UMultiplicity(i))
            {
                return false;
            }
        }

        // Compare V knots and multiplicities
        for (int i = 1; i <= surface1.VKnotCount; ++i)
        {
            if (Math.Abs(surface1.VKnot(i) - surface2.VKnot(i)) > CompTolerance
                || surface1.VMultiplicity(i) != surface2.VMultiplicity(i))
            {
                return false;
            }
        }

        // Compare rationality
        if (surface1.IsURational != surface2.IsURational
            || surface1.IsVRational != surface2.IsVRational)
        {
            return false;
        }

        // OCCT compares neither the periodicities nor the pole counts: with the same knots, a periodic direction
        // has more poles than a non-periodic one, and the loops below then read past the second surface's poles
        // (OCCT's release build reads out of its array; Pole throws Standard_OutOfRange here)
        var pointHasher = new PointHasher(CompTolerance, HashTolerance);

        // Compare poles
        for (int i = 1; i <= surface1.UPoleCount; ++i)
        {
            for (int j = 1; j <= surface1.VPoleCount; ++j)
            {
                if (!pointHasher.IsEqual(surface1.Pole(i, j), surface2.Pole(i, j)))
                {
                    return false;
                }
            }
        }

        // Compare weights if rational
        if (surface1.IsURational || surface1.IsVRational)
        {
            for (int i = 1; i <= surface1.UPoleCount; ++i)
            {
                for (int j = 1; j <= surface1.VPoleCount; ++j)
                {
                    if (Math.Abs(surface1.Weight(i, j) - surface2.Weight(i, j)) > CompTolerance)
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }
}
