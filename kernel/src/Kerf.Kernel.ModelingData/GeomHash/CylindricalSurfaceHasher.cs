// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_CylindricalSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_CylindricalSurface.</summary>
internal readonly struct CylindricalSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the cylinder by its position and radius.
    public ulong Hash(CylindricalSurface cylinder)
    {
        double factor = 1.0 / HashTolerance;
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            axisHasher.Hash(cylinder.Position.Frame),
            HashUtils.Hash((long)HashUtils.Round(cylinder.Radius * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two cylinders.
    public bool IsEqual(CylindricalSurface cylinder1, CylindricalSurface cylinder2)
    {
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        return axisHasher.IsEqual(cylinder1.Position.Frame, cylinder2.Position.Frame)
               && Math.Abs(cylinder1.Radius - cylinder2.Radius) <= CompTolerance;
    }
}
