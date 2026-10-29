// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_AxisPlacement.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of gp_Ax2 (axis placements), composed of the point and direction hashers.</summary>
internal readonly struct GeomHash_AxisPlacement(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the axis placement by location, axis direction, and X direction.
    public ulong Hash(in gp_Ax2 theAxisPlacement)
    {
        var aPointHasher = new GeomHash_PointHasher(CompTolerance, HashTolerance);
        var aDirectionHasher = new GeomHash_DirectionHasher(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            aPointHasher.Hash(theAxisPlacement.Location()),
            aDirectionHasher.Hash(theAxisPlacement.Direction()),
            aDirectionHasher.Hash(theAxisPlacement.XDirection())
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two axis placements.
    public bool IsEqual(in gp_Ax2 theAxisPlacement1, in gp_Ax2 theAxisPlacement2)
    {
        var aPointHasher = new GeomHash_PointHasher(CompTolerance, HashTolerance);
        var aDirectionHasher = new GeomHash_DirectionHasher(CompTolerance, HashTolerance);
        return aPointHasher.IsEqual(theAxisPlacement1.Location(), theAxisPlacement2.Location())
               && aDirectionHasher.IsEqual(theAxisPlacement1.Direction(), theAxisPlacement2.Direction())
               && aDirectionHasher.IsEqual(theAxisPlacement1.XDirection(), theAxisPlacement2.XDirection());
    }
}
