// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_ToroidalSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Engine;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_ToroidalSurface.</summary>
internal readonly struct ToroidalSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the torus by its position, major radius, and minor radius.
    public ulong Hash(ToroidalSurface torus)
    {
        double factor = 1.0 / HashTolerance;
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            axisHasher.Hash(torus.Position.Frame),
            HashUtils.Hash((long)HashUtils.Round(torus.MajorRadius * factor)),
            HashUtils.Hash((long)HashUtils.Round(torus.MinorRadius * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two toruss.
    public bool IsEqual(ToroidalSurface torus1, ToroidalSurface torus2)
    {
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        return axisHasher.IsEqual(torus1.Position.Frame, torus2.Position.Frame)
               && Math.Abs(torus1.MajorRadius - torus2.MajorRadius) <= CompTolerance
               && Math.Abs(torus1.MinorRadius - torus2.MinorRadius) <= CompTolerance;
    }
}
