// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_SurfaceOfLinearExtrusionHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_SurfaceOfLinearExtrusion.</summary>
internal readonly struct SurfaceOfLinearExtrusionHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the extrusion surface by its basis curve and direction.
    public ulong Hash(ExtrusionSurface surface)
    {
        var dirHasher = new DirectionHasher(CompTolerance, HashTolerance);
        var curveHasher = new CurveHasher(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            curveHasher.Hash(surface.BasisCurve),
            dirHasher.Hash(surface.Direction)
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two extrusion surfaces.
    public bool IsEqual(ExtrusionSurface surface1, ExtrusionSurface surface2)
    {
        var dirHasher = new DirectionHasher(CompTolerance, HashTolerance);
        var curveHasher = new CurveHasher(CompTolerance, HashTolerance);
        return curveHasher.Equals(surface1.BasisCurve, surface2.BasisCurve)
               && dirHasher.IsEqual(surface1.Direction, surface2.Direction);
    }
}
