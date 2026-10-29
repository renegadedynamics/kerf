// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_AxisPlacement.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of gp_Ax22d (axis placements), composed of the point and direction hashers.</summary>
internal readonly struct Geom2dHash_AxisPlacement(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the axis placement by location, X direction, and Y direction.
    public ulong Hash(in gp_Ax22d theAxisPlacement)
    {
        var aPointHasher = new Geom2dHash_PointHasher(CompTolerance, HashTolerance);
        var aDirectionHasher = new Geom2dHash_DirectionHasher(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            aPointHasher.Hash(theAxisPlacement.Location()),
            aDirectionHasher.Hash(theAxisPlacement.XDirection()),
            aDirectionHasher.Hash(theAxisPlacement.YDirection())
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two axis placements.
    public bool IsEqual(in gp_Ax22d theAxisPlacement1, in gp_Ax22d theAxisPlacement2)
    {
        var aPointHasher = new Geom2dHash_PointHasher(CompTolerance, HashTolerance);
        var aDirectionHasher = new Geom2dHash_DirectionHasher(CompTolerance, HashTolerance);
        return aPointHasher.IsEqual(theAxisPlacement1.Location(), theAxisPlacement2.Location())
               && aDirectionHasher.IsEqual(theAxisPlacement1.XDirection(), theAxisPlacement2.XDirection())
               && aDirectionHasher.IsEqual(theAxisPlacement1.YDirection(), theAxisPlacement2.YDirection());
    }
}
