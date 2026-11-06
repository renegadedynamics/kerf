// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_PointHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of gp_Pnt (3D points), used for geometry deduplication.</summary>
internal readonly struct PointHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the 3D point by its components.
    public ulong Hash(in Point3D point)
    {
        double factor = 1.0 / HashTolerance;

        // Round each component to tolerance precision before hashing
        ReadOnlySpan<ulong> hashes =
        [
            HashUtils.Hash((long)HashUtils.Round(point.X * factor)),
            HashUtils.Hash((long)HashUtils.Round(point.Y * factor)),
            HashUtils.Hash((long)HashUtils.Round(point.Z * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two 3D points with fixed tolerance.
    public bool IsEqual(in Point3D point1, in Point3D point2)
    {
        return Math.Abs(point1.X - point2.X) <= CompTolerance
               && Math.Abs(point1.Y - point2.Y) <= CompTolerance
               && Math.Abs(point1.Z - point2.Z) <= CompTolerance;
    }
}
