// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_TrimmedCurveHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom2d_TrimmedCurve.</summary>
internal readonly struct Geom2dHash_TrimmedCurveHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the trimmed curve by its parameters and basis curve.
    public ulong Hash(Geom2d_TrimmedCurve theCurve)
    {
        double aFactor = 1.0 / HashTolerance;
        var aCurveHasher = new Geom2dHash_CurveHasher(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            aCurveHasher.Hash(theCurve.BasisCurve()),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theCurve.FirstParameter() * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theCurve.LastParameter() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two trimmed curves.
    public bool IsEqual(Geom2d_TrimmedCurve theCurve1, Geom2d_TrimmedCurve theCurve2)
    {
        var aCurveHasher = new Geom2dHash_CurveHasher(CompTolerance, HashTolerance);
        return aCurveHasher.Equals(theCurve1.BasisCurve(), theCurve2.BasisCurve())
               && Math.Abs(theCurve1.FirstParameter() - theCurve2.FirstParameter()) <= CompTolerance
               && Math.Abs(theCurve1.LastParameter() - theCurve2.LastParameter()) <= CompTolerance;
    }
}
