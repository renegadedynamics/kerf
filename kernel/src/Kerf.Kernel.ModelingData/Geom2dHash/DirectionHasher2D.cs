// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_DirectionHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of gp_Dir2d (2D directions), used for geometry deduplication.</summary>
internal readonly struct DirectionHasher2D(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the 2D direction by its components.
    public ulong Hash(in Direction2D direction)
    {
        double factor = 1.0 / HashTolerance;

        // Round each component to tolerance precision before hashing
        ReadOnlySpan<ulong> hashes =
        [
            HashUtils.Hash((long)HashUtils.Round(direction.X * factor)),
            HashUtils.Hash((long)HashUtils.Round(direction.Y * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two 2D directions with fixed tolerance.
    public bool IsEqual(in Direction2D direction1, in Direction2D direction2)
    {
        return Math.Abs(direction1.X - direction2.X) <= CompTolerance
               && Math.Abs(direction1.Y - direction2.Y) <= CompTolerance;
    }
}
