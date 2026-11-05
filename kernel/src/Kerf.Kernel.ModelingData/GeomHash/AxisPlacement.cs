// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_AxisPlacement.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of gp_Ax2 (axis placements), composed of the point and direction hashers.</summary>
internal readonly struct AxisPlacement(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the axis placement by location, axis direction, and X direction.
    public ulong Hash(in Frame3D axisPlacement)
    {
        var pointHasher = new PointHasher(CompTolerance, HashTolerance);
        var directionHasher = new DirectionHasher(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            pointHasher.Hash(axisPlacement.Location),
            directionHasher.Hash(axisPlacement.Direction),
            directionHasher.Hash(axisPlacement.XDirection)
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two axis placements.
    public bool IsEqual(in Frame3D axisPlacement1, in Frame3D axisPlacement2)
    {
        var pointHasher = new PointHasher(CompTolerance, HashTolerance);
        var directionHasher = new DirectionHasher(CompTolerance, HashTolerance);
        return pointHasher.IsEqual(axisPlacement1.Location, axisPlacement2.Location)
               && directionHasher.IsEqual(axisPlacement1.Direction, axisPlacement2.Direction)
               && directionHasher.IsEqual(axisPlacement1.XDirection, axisPlacement2.XDirection);
    }
}
