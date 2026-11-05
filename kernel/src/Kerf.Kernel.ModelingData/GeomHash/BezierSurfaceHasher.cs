// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_BezierSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_BezierSurface: the hash reads the metadata only, the comparison the poles and weights.</summary>
internal readonly struct BezierSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the Bezier surface metadata only.
    public ulong Hash(BezierSurface surface)
    {
        ReadOnlySpan<ulong> hashes =
        [
            HashUtils.Hash(surface.UDegree),
            HashUtils.Hash(surface.VDegree),
            HashUtils.Hash(surface.UPoleCount),
            HashUtils.Hash(surface.VPoleCount),
            HashUtils.Hash((surface.IsURational ? 1 : 0) | ((surface.IsVRational ? 1 : 0) << 1))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two Bezier surfaces by full geometric data.
    public bool IsEqual(BezierSurface surface1, BezierSurface surface2)
    {
        // Compare degrees
        if (surface1.UDegree != surface2.UDegree || surface1.VDegree != surface2.VDegree)
        {
            return false;
        }

        // Compare rationality
        if (surface1.IsURational != surface2.IsURational
            || surface1.IsVRational != surface2.IsVRational)
        {
            return false;
        }

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
