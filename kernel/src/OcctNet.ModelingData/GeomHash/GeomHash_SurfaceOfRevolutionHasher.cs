// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_SurfaceOfRevolutionHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom_SurfaceOfRevolution.</summary>
internal readonly struct GeomHash_SurfaceOfRevolutionHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the revolution surface by its basis curve and axis.
    public ulong Hash(Geom_SurfaceOfRevolution theSurface)
    {
        var aPointHasher = new GeomHash_PointHasher(CompTolerance, HashTolerance);
        var aDirHasher = new GeomHash_DirectionHasher(CompTolerance, HashTolerance);
        var aCurveHasher = new GeomHash_CurveHasher(CompTolerance, HashTolerance);
        gp_Ax1 anAxis = theSurface.Axis();
        ReadOnlySpan<ulong> aHashes =
        [
            aCurveHasher.Hash(theSurface.BasisCurve()),
            aPointHasher.Hash(anAxis.Location()),
            aDirHasher.Hash(anAxis.Direction())
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two revolution surfaces.
    public bool IsEqual(Geom_SurfaceOfRevolution theSurface1, Geom_SurfaceOfRevolution theSurface2)
    {
        var aPointHasher = new GeomHash_PointHasher(CompTolerance, HashTolerance);
        var aDirHasher = new GeomHash_DirectionHasher(CompTolerance, HashTolerance);
        var aCurveHasher = new GeomHash_CurveHasher(CompTolerance, HashTolerance);
        gp_Ax1 anAxis1 = theSurface1.Axis();
        gp_Ax1 anAxis2 = theSurface2.Axis();
        return aCurveHasher.Equals(theSurface1.BasisCurve(), theSurface2.BasisCurve())
               && aPointHasher.IsEqual(anAxis1.Location(), anAxis2.Location())
               && aDirHasher.IsEqual(anAxis1.Direction(), anAxis2.Direction());
    }
}
