// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_DirectionHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of gp_Dir2d (2D directions), used for geometry deduplication.</summary>
internal readonly struct Geom2dHash_DirectionHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the 2D direction by its components.
    public ulong Hash(in gp_Dir2d theDirection)
    {
        double aFactor = 1.0 / HashTolerance;

        // Round each component to tolerance precision before hashing
        ReadOnlySpan<ulong> aHashes =
        [
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theDirection.X() * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theDirection.Y() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two 2D directions with fixed tolerance.
    public bool IsEqual(in gp_Dir2d theDirection1, in gp_Dir2d theDirection2)
    {
        return Math.Abs(theDirection1.X() - theDirection2.X()) <= CompTolerance
               && Math.Abs(theDirection1.Y() - theDirection2.Y()) <= CompTolerance;
    }
}
