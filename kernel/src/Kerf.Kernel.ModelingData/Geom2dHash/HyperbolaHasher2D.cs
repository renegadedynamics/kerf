// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_HyperbolaHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Engine;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom2d_Hyperbola.</summary>
internal readonly struct HyperbolaHasher2D(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the hyperbola by its position, major radius, and minor radius.
    public ulong Hash(HyperbolaCurve2D hyperbola)
    {
        double factor = 1.0 / HashTolerance;
        var axisHasher = new AxisPlacement2D(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            axisHasher.Hash(hyperbola.Position),
            HashUtils.Hash((long)HashUtils.Round(hyperbola.MajorRadius * factor)),
            HashUtils.Hash((long)HashUtils.Round(hyperbola.MinorRadius * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two hyperbolas.
    public bool IsEqual(HyperbolaCurve2D hyperbola1, HyperbolaCurve2D hyperbola2)
    {
        var axisHasher = new AxisPlacement2D(CompTolerance, HashTolerance);
        return axisHasher.IsEqual(hyperbola1.Position, hyperbola2.Position)
               && Math.Abs(hyperbola1.MajorRadius - hyperbola2.MajorRadius) <= CompTolerance
               && Math.Abs(hyperbola1.MinorRadius - hyperbola2.MinorRadius) <= CompTolerance;
    }
}
