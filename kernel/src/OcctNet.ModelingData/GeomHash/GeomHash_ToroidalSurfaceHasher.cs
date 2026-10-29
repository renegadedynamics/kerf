// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_ToroidalSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom_ToroidalSurface.</summary>
internal readonly struct GeomHash_ToroidalSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the torus by its position, major radius, and minor radius.
    public ulong Hash(Geom_ToroidalSurface theTorus)
    {
        double aFactor = 1.0 / HashTolerance;
        var anAxisHasher = new GeomHash_AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            anAxisHasher.Hash(theTorus.Position().Ax2()),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theTorus.MajorRadius() * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theTorus.MinorRadius() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two toruss.
    public bool IsEqual(Geom_ToroidalSurface theTorus1, Geom_ToroidalSurface theTorus2)
    {
        var anAxisHasher = new GeomHash_AxisPlacement(CompTolerance, HashTolerance);
        return anAxisHasher.IsEqual(theTorus1.Position().Ax2(), theTorus2.Position().Ax2())
               && Math.Abs(theTorus1.MajorRadius() - theTorus2.MajorRadius()) <= CompTolerance
               && Math.Abs(theTorus1.MinorRadius() - theTorus2.MinorRadius()) <= CompTolerance;
    }
}
