// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_BSplineCurveHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_BSplineCurve: the hash reads the metadata only, the comparison the knots, poles and weights.</summary>
internal readonly struct BSplineCurveHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the B-spline curve metadata only.
    public ulong Hash(BSplineCurve3D curve)
    {
        ReadOnlySpan<ulong> hashes =
        [
            HashUtils.Hash(curve.Degree),
            HashUtils.Hash(curve.PoleCount),
            HashUtils.Hash(curve.KnotCount),
            HashUtils.Hash(curve.IsRational ? 1 : 0)
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two B-spline curves by full geometric data.
    public bool IsEqual(BSplineCurve3D curve1, BSplineCurve3D curve2)
    {
        // Compare degrees
        if (curve1.Degree != curve2.Degree)
        {
            return false;
        }

        // Compare knot counts
        if (curve1.KnotCount != curve2.KnotCount)
        {
            return false;
        }

        // Compare knots and multiplicities
        for (int i = 1; i <= curve1.KnotCount; ++i)
        {
            if (Math.Abs(curve1.Knot(i) - curve2.Knot(i)) > CompTolerance
                || curve1.Multiplicity(i) != curve2.Multiplicity(i))
            {
                return false;
            }
        }

        // Compare rationality
        if (curve1.IsRational != curve2.IsRational)
        {
            return false;
        }

        // OCCT compares neither the periodicity nor the pole counts: a periodic curve has more poles than a
        // non-periodic one with the same knots, and the loops below then read past the second curve's poles
        // (OCCT's release build reads out of its array; Pole throws Standard_OutOfRange here)
        var pointHasher = new PointHasher(CompTolerance, HashTolerance);

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
