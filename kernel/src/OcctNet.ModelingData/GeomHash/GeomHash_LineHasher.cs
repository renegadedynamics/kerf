// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_LineHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom_Line.</summary>
internal readonly struct GeomHash_LineHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the line by its location and direction.
    public ulong Hash(Geom_Line theLine)
    {
        var aPointHasher = new GeomHash_PointHasher(CompTolerance, HashTolerance);
        var aDirHasher = new GeomHash_DirectionHasher(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            aPointHasher.Hash(theLine.Position().Location()),
            aDirHasher.Hash(theLine.Position().Direction())
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two lines by their positions.
    public bool IsEqual(Geom_Line theLine1, Geom_Line theLine2)
    {
        var aPointHasher = new GeomHash_PointHasher(CompTolerance, HashTolerance);
        var aDirHasher = new GeomHash_DirectionHasher(CompTolerance, HashTolerance);
        return aPointHasher.IsEqual(theLine1.Position().Location(), theLine2.Position().Location())
               && aDirHasher.IsEqual(theLine1.Position().Direction(), theLine2.Position().Direction());
    }
}
