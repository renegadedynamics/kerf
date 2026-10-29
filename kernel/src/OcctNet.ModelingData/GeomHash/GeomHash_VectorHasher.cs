// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_VectorHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of gp_Vec (3D vectors), used for geometry deduplication.</summary>
internal readonly struct GeomHash_VectorHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the 3D vector by its components.
    public ulong Hash(in gp_Vec theVector)
    {
        double aFactor = 1.0 / HashTolerance;

        // Round each component to tolerance precision before hashing
        ReadOnlySpan<ulong> aHashes =
        [
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theVector.X() * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theVector.Y() * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theVector.Z() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two 3D vectors with fixed tolerance.
    public bool IsEqual(in gp_Vec theVector1, in gp_Vec theVector2)
    {
        return Math.Abs(theVector1.X() - theVector2.X()) <= CompTolerance
               && Math.Abs(theVector1.Y() - theVector2.Y()) <= CompTolerance
               && Math.Abs(theVector1.Z() - theVector2.Z()) <= CompTolerance;
    }
}
