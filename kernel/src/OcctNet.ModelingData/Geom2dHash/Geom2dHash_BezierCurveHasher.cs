// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_BezierCurveHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom2d_BezierCurve: the hash reads the metadata only, the comparison the poles and weights.</summary>
internal readonly struct Geom2dHash_BezierCurveHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the Bezier curve metadata only.
    public ulong Hash(Geom2d_BezierCurve theCurve)
    {
        ReadOnlySpan<ulong> aHashes =
        [
            GeomHash_HashUtils.Hash(theCurve.Degree()),
            GeomHash_HashUtils.Hash(theCurve.NbPoles()),
            GeomHash_HashUtils.Hash(theCurve.IsRational() ? 1 : 0)
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two Bezier curves by full geometric data.
    public bool IsEqual(Geom2d_BezierCurve theCurve1, Geom2d_BezierCurve theCurve2)
    {
        // Compare degrees
        if (theCurve1.Degree() != theCurve2.Degree())
        {
            return false;
        }

        // Compare rationality
        if (theCurve1.IsRational() != theCurve2.IsRational())
        {
            return false;
        }

        var aPointHasher = new Geom2dHash_PointHasher(CompTolerance, HashTolerance);

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
