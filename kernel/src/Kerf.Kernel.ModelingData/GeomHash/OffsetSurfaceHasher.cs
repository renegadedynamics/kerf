// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_OffsetSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_OffsetSurface.</summary>
internal readonly struct OffsetSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the offset surface by its basis surface and offset distance.
    public ulong Hash(OffsetSurface surface)
    {
        double factor = 1.0 / HashTolerance;
        var surfaceHasher = new SurfaceHasher(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            surfaceHasher.Hash(surface.BasisSurface),
            HashUtils.Hash((long)HashUtils.Round(surface.Offset * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two offset surfaces by their offset distances and basis surfaces.
    public bool IsEqual(OffsetSurface surface1, OffsetSurface surface2)
    {
        var surfaceHasher = new SurfaceHasher(CompTolerance, HashTolerance);
        return surfaceHasher.Equals(surface1.BasisSurface, surface2.BasisSurface)
               && Math.Abs(surface1.Offset - surface2.Offset) <= CompTolerance;
    }
}
