// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_Aij.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomPlate/GeomPlate_Aij.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A pair of indices of points and the normalised vector between them, which GeomPlate_BuildAveragePlane uses to
/// find the lines through a set of points.
/// </summary>
/// <remarks>
/// A value type (rules, section 3); its fields are internal, as OCCT's friend GeomPlate_BuildAveragePlane reads and
/// writes them (HalfSpace re-indexes the pairs it keeps).
/// </remarks>
public struct GeomPlate_Aij : IEquatable<GeomPlate_Aij>
{
    internal int Ind1;
    internal int Ind2;
    internal Vector3D Vec;

    public GeomPlate_Aij()
    {
        Ind1 = 0;
        Ind2 = 0;
        Vec = new Vector3D();
    }

    public GeomPlate_Aij(int anInd1, int anInd2, in Vector3D aVec)
    {
        Ind1 = anInd1;
        Ind2 = anInd2;
        Vec = aVec;
    }

    public readonly bool Equals(GeomPlate_Aij other) => Ind1 == other.Ind1 && Ind2 == other.Ind2 && Vec.Equals(other.Vec);

    public override readonly bool Equals(object? obj) => obj is GeomPlate_Aij other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(Ind1, Ind2, Vec);

    public static bool operator ==(in GeomPlate_Aij theLeft, in GeomPlate_Aij theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in GeomPlate_Aij theLeft, in GeomPlate_Aij theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"GeomPlate_Aij({Ind1}, {Ind2}, {Vec})";
}
