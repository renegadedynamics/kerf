// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_SurfaceOfLinearExtrusionHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom_SurfaceOfLinearExtrusion.</summary>
internal readonly struct GeomHash_SurfaceOfLinearExtrusionHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the extrusion surface by its basis curve and direction.
    public ulong Hash(Geom_SurfaceOfLinearExtrusion theSurface)
    {
        var aDirHasher = new GeomHash_DirectionHasher(CompTolerance, HashTolerance);
        var aCurveHasher = new GeomHash_CurveHasher(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            aCurveHasher.Hash(theSurface.BasisCurve()),
            aDirHasher.Hash(theSurface.Direction())
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two extrusion surfaces.
    public bool IsEqual(Geom_SurfaceOfLinearExtrusion theSurface1, Geom_SurfaceOfLinearExtrusion theSurface2)
    {
        var aDirHasher = new GeomHash_DirectionHasher(CompTolerance, HashTolerance);
        var aCurveHasher = new GeomHash_CurveHasher(CompTolerance, HashTolerance);
        return aCurveHasher.Equals(theSurface1.BasisCurve(), theSurface2.BasisCurve())
               && aDirHasher.IsEqual(theSurface1.Direction(), theSurface2.Direction());
    }
}
