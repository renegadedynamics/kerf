// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_PlaneHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_Plane.</summary>
internal readonly struct PlaneHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the plane by its position.
    public ulong Hash(PlaneSurface plane)
    {
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        return axisHasher.Hash(plane.Position().Ax2());
    }

    // Compares two planes.
    public bool IsEqual(PlaneSurface plane1, PlaneSurface plane2)
    {
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        return axisHasher.IsEqual(plane1.Position().Ax2(), plane2.Position().Ax2());
    }
}
