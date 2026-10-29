// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_CircleHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom2d_Circle.</summary>
internal readonly struct Geom2dHash_CircleHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the circle by its position and radius.
    public ulong Hash(Geom2d_Circle theCircle)
    {
        double aFactor = 1.0 / HashTolerance;
        var anAxisHasher = new Geom2dHash_AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            anAxisHasher.Hash(theCircle.Position()),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theCircle.Radius() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two circles.
    public bool IsEqual(Geom2d_Circle theCircle1, Geom2d_Circle theCircle2)
    {
        var anAxisHasher = new Geom2dHash_AxisPlacement(CompTolerance, HashTolerance);
        return anAxisHasher.IsEqual(theCircle1.Position(), theCircle2.Position())
               && Math.Abs(theCircle1.Radius() - theCircle2.Radius()) <= CompTolerance;
    }
}
