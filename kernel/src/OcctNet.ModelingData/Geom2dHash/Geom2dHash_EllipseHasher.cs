// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_EllipseHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom2d_Ellipse.</summary>
internal readonly struct Geom2dHash_EllipseHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the ellipse by its position, major radius, and minor radius.
    public ulong Hash(Geom2d_Ellipse theEllipse)
    {
        double aFactor = 1.0 / HashTolerance;
        var anAxisHasher = new Geom2dHash_AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            anAxisHasher.Hash(theEllipse.Position()),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theEllipse.MajorRadius() * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theEllipse.MinorRadius() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two ellipses.
    public bool IsEqual(Geom2d_Ellipse theEllipse1, Geom2d_Ellipse theEllipse2)
    {
        var anAxisHasher = new Geom2dHash_AxisPlacement(CompTolerance, HashTolerance);
        return anAxisHasher.IsEqual(theEllipse1.Position(), theEllipse2.Position())
               && Math.Abs(theEllipse1.MajorRadius() - theEllipse2.MajorRadius()) <= CompTolerance
               && Math.Abs(theEllipse1.MinorRadius() - theEllipse2.MinorRadius()) <= CompTolerance;
    }
}
