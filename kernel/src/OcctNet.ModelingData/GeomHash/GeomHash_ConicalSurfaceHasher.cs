// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_ConicalSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom_ConicalSurface.</summary>
internal readonly struct GeomHash_ConicalSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the cone by its position, reference radius, and semi-angle.
    public ulong Hash(Geom_ConicalSurface theCone)
    {
        double aFactor = 1.0 / HashTolerance;
        var anAxisHasher = new GeomHash_AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            anAxisHasher.Hash(theCone.Position().Ax2()),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theCone.RefRadius() * aFactor)),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theCone.SemiAngle() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two cones.
    public bool IsEqual(Geom_ConicalSurface theCone1, Geom_ConicalSurface theCone2)
    {
        var anAxisHasher = new GeomHash_AxisPlacement(CompTolerance, HashTolerance);
        return anAxisHasher.IsEqual(theCone1.Position().Ax2(), theCone2.Position().Ax2())
               && Math.Abs(theCone1.RefRadius() - theCone2.RefRadius()) <= CompTolerance
               && Math.Abs(theCone1.SemiAngle() - theCone2.SemiAngle()) <= CompTolerance;
    }
}
