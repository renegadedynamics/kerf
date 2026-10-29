// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_HyperbolaHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom2d_Hyperbola.</summary>
internal readonly struct Geom2dHash_HyperbolaHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the hyperbola by its position, major radius, and minor radius.
    public ulong Hash(Geom2d_Hyperbola theHyperbola)
    {
        double aFactor = 1.0 / HashTolerance;
        var anAxisHasher = new Geom2dHash_AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            anAxisHasher.Hash(theHyperbola.Position()),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theHyperbola.MajorRadius() * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theHyperbola.MinorRadius() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two hyperbolas.
    public bool IsEqual(Geom2d_Hyperbola theHyperbola1, Geom2d_Hyperbola theHyperbola2)
    {
        var anAxisHasher = new Geom2dHash_AxisPlacement(CompTolerance, HashTolerance);
        return anAxisHasher.IsEqual(theHyperbola1.Position(), theHyperbola2.Position())
               && Math.Abs(theHyperbola1.MajorRadius() - theHyperbola2.MajorRadius()) <= CompTolerance
               && Math.Abs(theHyperbola1.MinorRadius() - theHyperbola2.MinorRadius()) <= CompTolerance;
    }
}
