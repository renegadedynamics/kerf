// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_RectangularTrimmedSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom_RectangularTrimmedSurface.</summary>
internal readonly struct GeomHash_RectangularTrimmedSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the trimmed surface by its basis surface and trim bounds.
    public ulong Hash(Geom_RectangularTrimmedSurface theSurface)
    {
        double aFactor = 1.0 / HashTolerance;
        var aSurfaceHasher = new GeomHash_SurfaceHasher(CompTolerance, HashTolerance);
        theSurface.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        ReadOnlySpan<ulong> aHashes =
        [
            aSurfaceHasher.Hash(theSurface.BasisSurface()),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(aU1 * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(aU2 * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(aV1 * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(aV2 * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two trimmed surfaces by their trim bounds and basis surfaces.
    public bool IsEqual(Geom_RectangularTrimmedSurface theSurface1, Geom_RectangularTrimmedSurface theSurface2)
    {
        var aSurfaceHasher = new GeomHash_SurfaceHasher(CompTolerance, HashTolerance);

        // Compare basis surfaces
        if (!aSurfaceHasher.Equals(theSurface1.BasisSurface(), theSurface2.BasisSurface()))
        {
            return false;
        }

        // Compare trim bounds
        theSurface1.Bounds(out double aU1_1, out double aU2_1, out double aV1_1, out double aV2_1);
        theSurface2.Bounds(out double aU1_2, out double aU2_2, out double aV1_2, out double aV2_2);
        return Math.Abs(aU1_1 - aU1_2) <= CompTolerance && Math.Abs(aU2_1 - aU2_2) <= CompTolerance
               && Math.Abs(aV1_1 - aV1_2) <= CompTolerance && Math.Abs(aV2_1 - aV2_2) <= CompTolerance;
    }
}
