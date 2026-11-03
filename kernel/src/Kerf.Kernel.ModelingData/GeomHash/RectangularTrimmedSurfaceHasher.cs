// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_RectangularTrimmedSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_RectangularTrimmedSurface.</summary>
internal readonly struct RectangularTrimmedSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the trimmed surface by its basis surface and trim bounds.
    public ulong Hash(TrimmedSurface surface)
    {
        double factor = 1.0 / HashTolerance;
        var surfaceHasher = new SurfaceHasher(CompTolerance, HashTolerance);
        surface.Bounds(out double u1, out double u2, out double v1, out double v2);
        ReadOnlySpan<ulong> hashes =
        [
            surfaceHasher.Hash(surface.BasisSurface()),
            HashUtils.Hash((long)HashUtils.Round(u1 * factor)),
            HashUtils.Hash((long)HashUtils.Round(u2 * factor)),
            HashUtils.Hash((long)HashUtils.Round(v1 * factor)),
            HashUtils.Hash((long)HashUtils.Round(v2 * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two trimmed surfaces by their trim bounds and basis surfaces.
    public bool IsEqual(TrimmedSurface surface1, TrimmedSurface surface2)
    {
        var surfaceHasher = new SurfaceHasher(CompTolerance, HashTolerance);

        // Compare basis surfaces
        if (!surfaceHasher.Equals(surface1.BasisSurface(), surface2.BasisSurface()))
        {
            return false;
        }

        // Compare trim bounds
        surface1.Bounds(out double u1_1, out double u2_1, out double v1_1, out double v2_1);
        surface2.Bounds(out double u1_2, out double u2_2, out double v1_2, out double v2_2);
        return Math.Abs(u1_1 - u1_2) <= CompTolerance && Math.Abs(u2_1 - u2_2) <= CompTolerance
               && Math.Abs(v1_1 - v1_2) <= CompTolerance && Math.Abs(v2_1 - v2_2) <= CompTolerance;
    }
}
