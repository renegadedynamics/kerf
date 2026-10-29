// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_PointHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of gp_Pnt (3D points), used for geometry deduplication.</summary>
internal readonly struct GeomHash_PointHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the 3D point by its components.
    public ulong Hash(in gp_Pnt thePoint)
    {
        double aFactor = 1.0 / HashTolerance;

        // Round each component to tolerance precision before hashing
        ReadOnlySpan<ulong> aHashes =
        [
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(thePoint.X() * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(thePoint.Y() * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(thePoint.Z() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two 3D points with fixed tolerance.
    public bool IsEqual(in gp_Pnt thePoint1, in gp_Pnt thePoint2)
    {
        return Math.Abs(thePoint1.X() - thePoint2.X()) <= CompTolerance
               && Math.Abs(thePoint1.Y() - thePoint2.Y()) <= CompTolerance
               && Math.Abs(thePoint1.Z() - thePoint2.Z()) <= CompTolerance;
    }
}
