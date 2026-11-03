// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_CircleHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_Circle.</summary>
internal readonly struct CircleHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the circle by its position and radius.
    public ulong Hash(CircleCurve3D circle)
    {
        double factor = 1.0 / HashTolerance;
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            axisHasher.Hash(circle.Position()),
            HashUtils.Hash((long)HashUtils.Round(circle.Radius() * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two circles.
    public bool IsEqual(CircleCurve3D circle1, CircleCurve3D circle2)
    {
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        return axisHasher.IsEqual(circle1.Position(), circle2.Position())
               && Math.Abs(circle1.Radius() - circle2.Radius()) <= CompTolerance;
    }
}
