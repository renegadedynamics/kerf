// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_CircleHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom2d_Circle.</summary>
internal readonly struct CircleHasher2D(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the circle by its position and radius.
    public ulong Hash(CircleCurve2D circle)
    {
        double factor = 1.0 / HashTolerance;
        var axisHasher = new AxisPlacement2D(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            axisHasher.Hash(circle.Position),
            HashUtils.Hash((long)HashUtils.Round(circle.Radius * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two circles.
    public bool IsEqual(CircleCurve2D circle1, CircleCurve2D circle2)
    {
        var axisHasher = new AxisPlacement2D(CompTolerance, HashTolerance);
        return axisHasher.IsEqual(circle1.Position, circle2.Position)
               && Math.Abs(circle1.Radius - circle2.Radius) <= CompTolerance;
    }
}
