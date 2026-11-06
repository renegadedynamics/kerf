// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_SurfaceOfRevolutionHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_SurfaceOfRevolution.</summary>
internal readonly struct SurfaceOfRevolutionHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the revolution surface by its basis curve and axis.
    public ulong Hash(RevolutionSurface surface)
    {
        var pointHasher = new PointHasher(CompTolerance, HashTolerance);
        var dirHasher = new DirectionHasher(CompTolerance, HashTolerance);
        var curveHasher = new CurveHasher(CompTolerance, HashTolerance);
        Axis3D axis = surface.Axis;
        ReadOnlySpan<ulong> hashes =
        [
            curveHasher.Hash(surface.BasisCurve),
            pointHasher.Hash(axis.Location),
            dirHasher.Hash(axis.Direction)
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two revolution surfaces.
    public bool IsEqual(RevolutionSurface surface1, RevolutionSurface surface2)
    {
        var pointHasher = new PointHasher(CompTolerance, HashTolerance);
        var dirHasher = new DirectionHasher(CompTolerance, HashTolerance);
        var curveHasher = new CurveHasher(CompTolerance, HashTolerance);
        Axis3D axis1 = surface1.Axis;
        Axis3D axis2 = surface2.Axis;
        return curveHasher.Equals(surface1.BasisCurve, surface2.BasisCurve)
               && pointHasher.IsEqual(axis1.Location, axis2.Location)
               && dirHasher.IsEqual(axis1.Direction, axis2.Direction);
    }
}
