// Port of: src/ModelingData/TKG3d/GeomHash/GeomHash_ParabolaHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>OCCT's hasher of Geom_Parabola.</summary>
internal readonly struct GeomHash_ParabolaHasher(double theCompTolerance, double theHashTolerance)
{
    public readonly double CompTolerance = theCompTolerance;
    public readonly double HashTolerance = theHashTolerance;

    // Hashes the parabola by its position and focal length.
    public ulong Hash(Geom_Parabola theParabola)
    {
        double aFactor = 1.0 / HashTolerance;
        var anAxisHasher = new GeomHash_AxisPlacement(CompTolerance, HashTolerance);
        ReadOnlySpan<ulong> aHashes =
        [
            anAxisHasher.Hash(theParabola.Position()),
            GeomHash_HashUtils.Hash((long)GeomHash_HashUtils.Round(theParabola.Focal() * aFactor))
        ];
        return GeomHash_HashUtils.HashBytes(aHashes);
    }

    // Compares two parabolas.
    public bool IsEqual(Geom_Parabola theParabola1, Geom_Parabola theParabola2)
    {
        var anAxisHasher = new GeomHash_AxisPlacement(CompTolerance, HashTolerance);
        return anAxisHasher.IsEqual(theParabola1.Position(), theParabola2.Position())
               && Math.Abs(theParabola1.Focal() - theParabola2.Focal()) <= CompTolerance;
    }
}
