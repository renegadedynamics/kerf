// Port of: src/ModelingData/TKG2d/Geom2dHash/Geom2dHash_ParabolaHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom2d_Parabola.</summary>
internal readonly struct ParabolaHasher2D(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the parabola by its position and focal length.
    public ulong Hash(ParabolaCurve2D parabola)
    {
        double factor = 1.0 / HashTolerance;
        var axisHasher = new AxisPlacement2D(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            axisHasher.Hash(parabola.Position),
            HashUtils.Hash((long)HashUtils.Round(parabola.Focal * factor))
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two parabolas.
    public bool IsEqual(ParabolaCurve2D parabola1, ParabolaCurve2D parabola2)
    {
        var axisHasher = new AxisPlacement2D(CompTolerance, HashTolerance);
        return axisHasher.IsEqual(parabola1.Position, parabola2.Position)
               && Math.Abs(parabola1.Focal - parabola2.Focal) <= CompTolerance;
    }
}
