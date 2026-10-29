// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_SphericalSurfaceHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom_SphericalSurface.</summary>
internal readonly struct GeomHash_SphericalSurfaceHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the sphere by its position and radius.
    public ulong Hash(Geom_SphericalSurface theSphere)
    {
        double aFactor = 1.0 / HashTolerance;
        var anAxisHasher = new GeomHash_AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            anAxisHasher.Hash(theSphere.Position().Ax2()),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theSphere.Radius() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two spheres.
    public bool IsEqual(Geom_SphericalSurface theSphere1, Geom_SphericalSurface theSphere2)
    {
        var anAxisHasher = new GeomHash_AxisPlacement(CompTolerance, HashTolerance);
        return anAxisHasher.IsEqual(theSphere1.Position().Ax2(), theSphere2.Position().Ax2())
               && Math.Abs(theSphere1.Radius() - theSphere2.Radius()) <= CompTolerance;
    }
}
