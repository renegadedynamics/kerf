// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_VectorHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of gp_Vec (3D vectors), used for geometry deduplication.</summary>
internal readonly struct VectorHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the 3D vector by its components.
    public ulong Hash(in Vector3D vector)
    {
        double factor = 1.0 / HashTolerance;

        // Round each component to tolerance precision before hashing
        ReadOnlySpan<ulong> hashes =
        [
            HashUtils.Hash((long)HashUtils.Round(vector.X() * factor)),
            HashUtils.Hash((long)HashUtils.Round(vector.Y() * factor)),
            HashUtils.Hash((long)HashUtils.Round(vector.Z() * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two 3D vectors with fixed tolerance.
    public bool IsEqual(in Vector3D vector1, in Vector3D vector2)
    {
        return Math.Abs(vector1.X() - vector2.X()) <= CompTolerance
               && Math.Abs(vector1.Y() - vector2.Y()) <= CompTolerance
               && Math.Abs(vector1.Z() - vector2.Z()) <= CompTolerance;
    }
}
