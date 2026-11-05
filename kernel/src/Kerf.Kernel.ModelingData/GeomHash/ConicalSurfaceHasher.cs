// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_ConicalSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Engine;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_ConicalSurface.</summary>
internal readonly struct ConicalSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the cone by its position, reference radius, and semi-angle.
    public ulong Hash(ConicalSurface cone)
    {
        double factor = 1.0 / HashTolerance;
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            axisHasher.Hash(cone.Position.Frame),
            HashUtils.Hash((long)HashUtils.Round(cone.RefRadius * factor)),
            HashUtils.Hash((long)HashUtils.Round(cone.SemiAngle * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two cones.
    public bool IsEqual(ConicalSurface cone1, ConicalSurface cone2)
    {
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        return axisHasher.IsEqual(cone1.Position.Frame, cone2.Position.Frame)
               && Math.Abs(cone1.RefRadius - cone2.RefRadius) <= CompTolerance
               && Math.Abs(cone1.SemiAngle - cone2.SemiAngle) <= CompTolerance;
    }
}
