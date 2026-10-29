// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_CylindricalSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom_CylindricalSurface.</summary>
internal readonly struct GeomHash_CylindricalSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the cylinder by its position and radius.
    public ulong Hash(Geom_CylindricalSurface theCylinder)
    {
        double aFactor = 1.0 / HashTolerance;
        var anAxisHasher = new GeomHash_AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            anAxisHasher.Hash(theCylinder.Position().Ax2()),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theCylinder.Radius() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two cylinders.
    public bool IsEqual(Geom_CylindricalSurface theCylinder1, Geom_CylindricalSurface theCylinder2)
    {
        var anAxisHasher = new GeomHash_AxisPlacement(CompTolerance, HashTolerance);
        return anAxisHasher.IsEqual(theCylinder1.Position().Ax2(), theCylinder2.Position().Ax2())
               && Math.Abs(theCylinder1.Radius() - theCylinder2.Radius()) <= CompTolerance;
    }
}
