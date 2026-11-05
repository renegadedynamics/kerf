// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_OffsetCurveHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_OffsetCurve.</summary>
internal readonly struct OffsetCurveHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the offset curve by its offset distance, direction, and basis curve.
    public ulong Hash(OffsetCurve3D curve)
    {
        double factor = 1.0 / HashTolerance;
        var dirHasher = new DirectionHasher(CompTolerance, HashTolerance);
        var curveHasher = new CurveHasher(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            curveHasher.Hash(curve.BasisCurve),
            HashUtils.Hash((long)HashUtils.Round(curve.Offset * factor)),
            dirHasher.Hash(curve.Direction)
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two offset curves.
    public bool IsEqual(OffsetCurve3D curve1, OffsetCurve3D curve2)
    {
        var dirHasher = new DirectionHasher(CompTolerance, HashTolerance);
        var curveHasher = new CurveHasher(CompTolerance, HashTolerance);
        return curveHasher.Equals(curve1.BasisCurve, curve2.BasisCurve)
               && Math.Abs(curve1.Offset - curve2.Offset) <= CompTolerance
               && dirHasher.IsEqual(curve1.Direction, curve2.Direction);
    }
}
