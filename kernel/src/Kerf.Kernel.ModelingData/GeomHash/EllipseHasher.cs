// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_EllipseHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_Ellipse.</summary>
internal readonly struct EllipseHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the ellipse by its position, major radius, and minor radius.
    public ulong Hash(EllipseCurve3D ellipse)
    {
        double factor = 1.0 / HashTolerance;
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            axisHasher.Hash(ellipse.Position),
            HashUtils.Hash((long)HashUtils.Round(ellipse.MajorRadius * factor)),
            HashUtils.Hash((long)HashUtils.Round(ellipse.MinorRadius * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two ellipses.
    public bool IsEqual(EllipseCurve3D ellipse1, EllipseCurve3D ellipse2)
    {
        var axisHasher = new AxisPlacement(CompTolerance, HashTolerance);
        return axisHasher.IsEqual(ellipse1.Position, ellipse2.Position)
               && Math.Abs(ellipse1.MajorRadius - ellipse2.MajorRadius) <= CompTolerance
               && Math.Abs(ellipse1.MinorRadius - ellipse2.MinorRadius) <= CompTolerance;
    }
}
