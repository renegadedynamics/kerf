// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_PlaneHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom_Plane.</summary>
internal readonly struct GeomHash_PlaneHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the plane by its position.
    public ulong Hash(Geom_Plane thePlane)
    {
        var anAxisHasher = new GeomHash_AxisPlacement(CompTolerance, HashTolerance);
        return anAxisHasher.Hash(thePlane.Position().Ax2());
    }

    // Compares two planes.
    public bool IsEqual(Geom_Plane thePlane1, Geom_Plane thePlane2)
    {
        var anAxisHasher = new GeomHash_AxisPlacement(CompTolerance, HashTolerance);
        return anAxisHasher.IsEqual(thePlane1.Position().Ax2(), thePlane2.Position().Ax2());
    }
}
