// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_DirectionHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>OCCT's hasher of gp_Dir (3D directions), used for geometry deduplication.</summary>
internal readonly struct GeomHash_DirectionHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the 3D direction by its components.
    public ulong Hash(in Direction3D theDirection)
    {
        double aFactor = 1.0 / HashTolerance;

        // Round each component to tolerance precision before hashing
        ReadOnlySpan<ulong> aHashes =
        [
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theDirection.X() * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theDirection.Y() * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theDirection.Z() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two 3D directions with fixed tolerance.
    public bool IsEqual(in Direction3D theDirection1, in Direction3D theDirection2)
    {
        return Math.Abs(theDirection1.X() - theDirection2.X()) <= CompTolerance
               && Math.Abs(theDirection1.Y() - theDirection2.Y()) <= CompTolerance
               && Math.Abs(theDirection1.Z() - theDirection2.Z()) <= CompTolerance;
    }
}
