// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Vertex.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using static Occt.BRepMesh_DegreeOfFreedom;

namespace Occt;

/// <summary>
/// A vertex of the mesh in the parametric space, possibly associated with a 3D point stored elsewhere (by its index).
/// </summary>
/// <remarks>
/// The default vertex is the origin, with no 3D point and <c>BRepMesh_Free</c>, as OCCT's; <c>default(T)</c> is the
/// same. <see cref="Equals(BRepMesh_Vertex)"/> and <c>==</c> compare the fields exactly (decision 4402); OCCT's
/// <c>operator==</c>, which compares within <c>Precision::PConfusion()</c> and never matches a deleted vertex, is
/// <see cref="IsEqual"/>, and maps in ported code take <see cref="Hasher"/>, OCCT's hash and equality.
/// </remarks>
public partial struct BRepMesh_Vertex : IEquatable<BRepMesh_Vertex>
{
    private gp_XY myUV;
    private int myLocation3d;
    private BRepMesh_DegreeOfFreedom myMovability;

    /// <summary>A vertex associated with the 3D point of index <paramref name="theLocation3d"/>.</summary>
    public BRepMesh_Vertex(in gp_XY theUV, int theLocation3d, BRepMesh_DegreeOfFreedom theMovability)
    {
        Initialize(theUV, theLocation3d, theMovability);
    }

    /// <summary>A vertex without a 3D point.</summary>
    public BRepMesh_Vertex(double theU, double theV, BRepMesh_DegreeOfFreedom theMovability)
    {
        myUV = new gp_XY(theU, theV);
        myLocation3d = 0;
        myMovability = theMovability;
    }

    /// <summary>Sets the position, the index of the 3D point and the movability.</summary>
    internal void Initialize(in gp_XY theUV, int theLocation3d, BRepMesh_DegreeOfFreedom theMovability)
    {
        myUV = theUV;
        myLocation3d = theLocation3d;
        myMovability = theMovability;
    }

    /// <summary>The position in the parametric space.</summary>
    public readonly gp_XY Coord() => myUV;

    /// <summary>The position in the parametric space, for writing.</summary>
    [UnscopedRef]
    internal ref gp_XY ChangeCoord() => ref myUV;

    /// <summary>The index of the associated 3D point.</summary>
    public readonly int Location3d() => myLocation3d;

    /// <summary>The movability.</summary>
    public readonly BRepMesh_DegreeOfFreedom Movability() => myMovability;

    /// <summary>Sets the movability.</summary>
    internal void SetMovability(BRepMesh_DegreeOfFreedom theMovability) => myMovability = theMovability;

    /// <summary>This vertex with another movability (the twin of <c>SetMovability</c>).</summary>
    public readonly BRepMesh_Vertex WithMovability(BRepMesh_DegreeOfFreedom theMovability)
    {
        BRepMesh_Vertex aCopy = this;
        aCopy.SetMovability(theMovability);
        return aCopy;
    }

    /// <summary>This vertex at another position (the twin of writing through <c>ChangeCoord</c>).</summary>
    public readonly BRepMesh_Vertex WithCoord(in gp_XY theUV)
    {
        BRepMesh_Vertex aCopy = this;
        aCopy.myUV = theUV;
        return aCopy;
    }

    /// <summary>
    /// OCCT's equality: false if either vertex is deleted, else whether the positions are equal within
    /// <c>Precision::PConfusion()</c>.
    /// </summary>
    public readonly bool IsEqual(in BRepMesh_Vertex theOther)
    {
        if (myMovability == BRepMesh_Deleted || theOther.myMovability == BRepMesh_Deleted)
        {
            return false;
        }

        return myUV.IsEqual(theOther.myUV, Precision.PConfusion());
    }

    /// <summary>OCCT's hasher: <c>std::hash&lt;BRepMesh_Vertex&gt;</c> and <c>operator==</c> (<see cref="IsEqual"/>).</summary>
    public sealed class Hasher : IEqualityComparer<BRepMesh_Vertex>
    {
        public static readonly Hasher Instance = new();

        public bool Equals(BRepMesh_Vertex theV1, BRepMesh_Vertex theV2) => theV1.IsEqual(theV2);

        public int GetHashCode(BRepMesh_Vertex theVertex)
            => (Math.Floor(1e5 * theVertex.Coord().X()) * Math.Floor(1e5 * theVertex.Coord().Y())).GetHashCode();
    }

    public readonly bool Equals(BRepMesh_Vertex other)
        => myUV.Equals(other.myUV) && myLocation3d == other.myLocation3d && myMovability == other.myMovability;

    public override readonly bool Equals(object? obj) => obj is BRepMesh_Vertex other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myUV, myLocation3d, myMovability);

    public static bool operator ==(in BRepMesh_Vertex theLeft, in BRepMesh_Vertex theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in BRepMesh_Vertex theLeft, in BRepMesh_Vertex theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"BRepMesh_Vertex({myUV}, {myLocation3d}, {myMovability})";
}
