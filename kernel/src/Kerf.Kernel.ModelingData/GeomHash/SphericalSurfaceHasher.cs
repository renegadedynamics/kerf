// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_SphericalSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Engine;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_SphericalSurface.</summary>
internal readonly struct SphericalSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the sphere by its position and radius.
    public ulong Hash(SphericalSurface sphere)
    {
        double factor = 1.0 / HashTolerance;
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            axisHasher.Hash(sphere.Position.Frame),
            HashUtils.Hash((long)HashUtils.Round(sphere.Radius * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two spheres.
    public bool IsEqual(SphericalSurface sphere1, SphericalSurface sphere2)
    {
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        return axisHasher.IsEqual(sphere1.Position.Frame, sphere2.Position.Frame)
               && Math.Abs(sphere1.Radius - sphere2.Radius) <= CompTolerance;
    }
}
