// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_TrimmedCurveHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom2d_TrimmedCurve.</summary>
internal readonly struct TrimmedCurveHasher2D(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the trimmed curve by its parameters and basis curve.
    public ulong Hash(TrimmedCurve2D curve)
    {
        double factor = 1.0 / HashTolerance;
        var curveHasher = new CurveHasher2D(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            curveHasher.Hash(curve.BasisCurve),
            HashUtils.Hash((long)HashUtils.Round(curve.FirstParameter * factor)),
            HashUtils.Hash((long)HashUtils.Round(curve.LastParameter * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two trimmed curves.
    public bool IsEqual(TrimmedCurve2D curve1, TrimmedCurve2D curve2)
    {
        var curveHasher = new CurveHasher2D(CompTolerance, HashTolerance);
        return curveHasher.Equals(curve1.BasisCurve, curve2.BasisCurve)
               && Math.Abs(curve1.FirstParameter - curve2.FirstParameter) <= CompTolerance
               && Math.Abs(curve1.LastParameter - curve2.LastParameter) <= CompTolerance;
    }
}
