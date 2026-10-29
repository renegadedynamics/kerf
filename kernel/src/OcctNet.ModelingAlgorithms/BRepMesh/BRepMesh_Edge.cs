// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Edge.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepMesh_DegreeOfFreedom;

namespace Occt;

/// <summary>A link of the mesh, with its movability.</summary>
/// <remarks>
/// OCCT derives it from <see cref="BRepMesh_OrientedEdge"/>; a struct can't, so it holds the nodes itself and converts
/// to an oriented edge implicitly, as C++ slices it. The default link joins nodes -1 and -1 and is deleted, as OCCT's;
/// <c>default(T)</c> joins 0 and 0 and is free, so ported code writes <c>new BRepMesh_Edge()</c>.
/// <see cref="Equals(BRepMesh_Edge)"/> and <c>==</c> compare the fields exactly (decision 539); OCCT's
/// <c>operator==</c> (the same nodes in either order, never a deleted link) is <see cref="IsEqual"/>, and maps in
/// ported code take <see cref="Hasher"/>.
/// </remarks>
public partial struct BRepMesh_Edge : IEquatable<BRepMesh_Edge>
{
    private int myFirstNode;
    private int myLastNode;
    private BRepMesh_DegreeOfFreedom myMovability;

    /// <summary>The deleted link of nodes -1 and -1.</summary>
    public BRepMesh_Edge()
    {
        myFirstNode = -1;
        myLastNode = -1;
        myMovability = BRepMesh_Deleted;
    }

    /// <summary>A link from <paramref name="theFirstNode"/> to <paramref name="theLastNode"/>.</summary>
    public BRepMesh_Edge(int theFirstNode, int theLastNode, BRepMesh_DegreeOfFreedom theMovability)
    {
        myFirstNode = theFirstNode;
        myLastNode = theLastNode;
        myMovability = theMovability;
    }

    /// <summary>The index of the first node.</summary>
    public readonly int FirstNode() => myFirstNode;

    /// <summary>The index of the last node.</summary>
    public readonly int LastNode() => myLastNode;

    /// <summary>The movability.</summary>
    public readonly BRepMesh_DegreeOfFreedom Movability() => myMovability;

    /// <summary>Sets the movability.</summary>
    internal void SetMovability(BRepMesh_DegreeOfFreedom theMovability) => myMovability = theMovability;

    /// <summary>This link with another movability (the twin of <c>SetMovability</c>).</summary>
    public readonly BRepMesh_Edge WithMovability(BRepMesh_DegreeOfFreedom theMovability)
        => new(myFirstNode, myLastNode, theMovability);

    /// <summary>The nodes as an oriented edge (C++'s slicing to the base class).</summary>
    public static implicit operator BRepMesh_OrientedEdge(in BRepMesh_Edge theEdge)
        => new(theEdge.myFirstNode, theEdge.myLastNode);

    /// <summary>Whether both links have the same nodes in the same order.</summary>
    public readonly bool IsSameOrientation(in BRepMesh_Edge theOther)
        => myFirstNode == theOther.myFirstNode && myLastNode == theOther.myLastNode;

    /// <summary>OCCT's equality: false if either link is deleted, else whether they join the same nodes.</summary>
    public readonly bool IsEqual(in BRepMesh_Edge theOther)
    {
        if (myMovability == BRepMesh_Deleted || theOther.myMovability == BRepMesh_Deleted)
        {
            return false;
        }

        return IsSameOrientation(theOther)
               || (FirstNode() == theOther.LastNode() && LastNode() == theOther.FirstNode());
    }

    /// <summary>OCCT's hash of a link: the low 16 bits of the smaller node, then of the larger.</summary>
    internal readonly uint OcctHash()
    {
        ushort a0 = (ushort)myFirstNode;
        ushort a1 = (ushort)myLastNode;
        if (a0 > a1)
        {
            (a0, a1) = (a1, a0);
        }

        return a0 | ((uint)a1 << 16);
    }

    /// <summary>OCCT's hasher: <c>std::hash&lt;BRepMesh_Edge&gt;</c> and <c>operator==</c> (<see cref="IsEqual"/>).</summary>
    public sealed class Hasher : IEqualityComparer<BRepMesh_Edge>
    {
        public static readonly Hasher Instance = new();

        public bool Equals(BRepMesh_Edge theE1, BRepMesh_Edge theE2) => theE1.IsEqual(theE2);

        public int GetHashCode(BRepMesh_Edge theEdge) => (int)theEdge.OcctHash();
    }

    public readonly bool Equals(BRepMesh_Edge other)
        => myFirstNode == other.myFirstNode && myLastNode == other.myLastNode && myMovability == other.myMovability;

    public override readonly bool Equals(object? obj) => obj is BRepMesh_Edge other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myFirstNode, myLastNode, myMovability);

    public static bool operator ==(in BRepMesh_Edge theLeft, in BRepMesh_Edge theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in BRepMesh_Edge theLeft, in BRepMesh_Edge theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"BRepMesh_Edge({myFirstNode}, {myLastNode}, {myMovability})";
}
