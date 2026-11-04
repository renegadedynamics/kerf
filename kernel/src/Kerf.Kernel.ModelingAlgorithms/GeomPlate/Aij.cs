// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_Aij.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_Aij.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// A pair of indices of points and the normalised vector between them, which GeomPlate_BuildAveragePlane uses to
/// find the lines through a set of points.
/// </summary>
/// <remarks>
/// A value type (rules, section 3); its fields are internal, as OCCT's friend GeomPlate_BuildAveragePlane reads and
/// writes them (HalfSpace re-indexes the pairs it keeps).
/// </remarks>
internal struct Aij : IEquatable<Aij>
{
    internal int _ind1;
    internal int _ind2;
    internal Vector3D _vec;

    public Aij()
    {
        _ind1 = 0;
        _ind2 = 0;
        _vec = new Vector3D();
    }

    public Aij(int anInd1, int anInd2, in Vector3D aVec)
    {
        _ind1 = anInd1;
        _ind2 = anInd2;
        _vec = aVec;
    }

    public readonly bool Equals(Aij other) => _ind1 == other._ind1 && _ind2 == other._ind2 && _vec.Equals(other._vec);

    public override readonly bool Equals(object? obj) => obj is Aij other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_ind1, _ind2, _vec);

    public static bool operator ==(in Aij left, in Aij right) => left.Equals(right);

    public static bool operator !=(in Aij left, in Aij right) => !left.Equals(right);

    public override readonly string ToString() => $"GeomPlate_Aij({_ind1}, {_ind2}, {_vec})";
}
