// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_OffsetCurveHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom2d_OffsetCurve.</summary>
internal readonly struct OffsetCurveHasher2D(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the offset curve by its offset distance and basis curve.
    public ulong Hash(OffsetCurve2D curve)
    {
        double factor = 1.0 / HashTolerance;
        var curveHasher = new CurveHasher2D(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            curveHasher.Hash(curve.BasisCurve),
            HashUtils.Hash((long)HashUtils.Round(curve.Offset * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two offset curves.
    public bool IsEqual(OffsetCurve2D curve1, OffsetCurve2D curve2)
    {
        var curveHasher = new CurveHasher2D(CompTolerance, HashTolerance);
        return curveHasher.Equals(curve1.BasisCurve, curve2.BasisCurve)
               && Math.Abs(curve1.Offset - curve2.Offset) <= CompTolerance;
    }
}
