// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_LineHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Engine;

namespace Kerf.Engine.Geometry.Hashing;

/// <summary>OCCT's hasher of Geom_Line.</summary>
internal readonly struct LineHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the line by its location and direction.
    public ulong Hash(LineCurve3D line)
    {
        var pointHasher = new PointHasher(CompTolerance, HashTolerance);
        var dirHasher = new DirectionHasher(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> hashes =
        [
            pointHasher.Hash(line.Position.Location),
            dirHasher.Hash(line.Position.Direction)
        ];
        return HashUtils.HashBytes(hashes);
    }

    // Compares two lines by their positions.
    public bool IsEqual(LineCurve3D line1, LineCurve3D line2)
    {
        var pointHasher = new PointHasher(CompTolerance, HashTolerance);
        var dirHasher = new DirectionHasher(CompTolerance, HashTolerance);
        return pointHasher.IsEqual(line1.Position.Location, line2.Position.Location)
               && dirHasher.IsEqual(line1.Position.Direction, line2.Position.Direction);
    }
}
