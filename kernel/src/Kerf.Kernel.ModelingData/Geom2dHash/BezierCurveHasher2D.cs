// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_BezierCurveHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom2d_BezierCurve: the hash reads the metadata only, the comparison the poles and weights.</summary>
internal readonly struct BezierCurveHasher2D(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the Bezier curve metadata only.
    public ulong Hash(BezierCurve2D curve)
    {
        ReadOnlySpan<ulong> hashes =
        [
            HashUtils.Hash(curve.Degree),
            HashUtils.Hash(curve.PoleCount),
            HashUtils.Hash(curve.IsRational ? 1 : 0)
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two Bezier curves by full geometric data.
    public bool IsEqual(BezierCurve2D curve1, BezierCurve2D curve2)
    {
        // Compare degrees
        if (curve1.Degree != curve2.Degree)
        {
            return false;
        }

        // Compare rationality
        if (curve1.IsRational != curve2.IsRational)
        {
            return false;
        }

        var pointHasher = new PointHasher2D(CompTolerance, HashTolerance);

        // Compare poles
        for (int i = 1; i <= curve1.PoleCount; ++i)
        {
            if (!pointHasher.IsEqual(curve1.Pole(i), curve2.Pole(i)))
            {
                return false;
            }
        }

        // Compare weights if rational
        if (curve1.IsRational)
        {
            for (int i = 1; i <= curve1.PoleCount; ++i)
            {
                if (Math.Abs(curve1.Weight(i) - curve2.Weight(i)) > CompTolerance)
                {
                    return false;
                }
            }
        }

        return true;
    }
}
