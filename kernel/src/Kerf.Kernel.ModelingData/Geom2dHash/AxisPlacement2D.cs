// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_AxisPlacement.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of gp_Ax22d (axis placements), composed of the point and direction hashers.</summary>
internal readonly struct AxisPlacement2D(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the axis placement by location, X direction, and Y direction.
    public ulong Hash(in Frame2D axisPlacement)
    {
        var pointHasher = new PointHasher2D(CompTolerance, HashTolerance);
        var directionHasher = new DirectionHasher2D(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            pointHasher.Hash(axisPlacement.Location),
            directionHasher.Hash(axisPlacement.XDirection),
            directionHasher.Hash(axisPlacement.YDirection)
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two axis placements.
    public bool IsEqual(in Frame2D axisPlacement1, in Frame2D axisPlacement2)
    {
        var pointHasher = new PointHasher2D(CompTolerance, HashTolerance);
        var directionHasher = new DirectionHasher2D(CompTolerance, HashTolerance);
        return pointHasher.IsEqual(axisPlacement1.Location, axisPlacement2.Location)
               && directionHasher.IsEqual(axisPlacement1.XDirection, axisPlacement2.XDirection)
               && directionHasher.IsEqual(axisPlacement1.YDirection, axisPlacement2.YDirection);
    }
}
