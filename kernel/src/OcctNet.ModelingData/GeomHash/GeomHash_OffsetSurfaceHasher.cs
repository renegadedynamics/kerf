// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_OffsetSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom_OffsetSurface.</summary>
internal readonly struct GeomHash_OffsetSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the offset surface by its basis surface and offset distance.
    public ulong Hash(Geom_OffsetSurface theSurface)
    {
        double aFactor = 1.0 / HashTolerance;
        var aSurfaceHasher = new GeomHash_SurfaceHasher(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            aSurfaceHasher.Hash(theSurface.BasisSurface()),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theSurface.Offset() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two offset surfaces by their offset distances and basis surfaces.
    public bool IsEqual(Geom_OffsetSurface theSurface1, Geom_OffsetSurface theSurface2)
    {
        var aSurfaceHasher = new GeomHash_SurfaceHasher(CompTolerance, HashTolerance);
        return aSurfaceHasher.Equals(theSurface1.BasisSurface(), theSurface2.BasisSurface())
               && Math.Abs(theSurface1.Offset() - theSurface2.Offset()) <= CompTolerance;
    }
}
