// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_OrientedEdge.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A link of the mesh between two nodes, from the first to the last.</summary>
/// <remarks>
/// The default link joins nodes -1 and -1, as OCCT's; <c>default(T)</c> joins 0 and 0, so ported code writes
/// <c>new BRepMesh_OrientedEdge()</c>. OCCT's equality is exact, so <see cref="Equals(BRepMesh_OrientedEdge)"/> is
/// OCCT's <c>IsEqual</c>, and <see cref="GetHashCode"/> is OCCT's hash (the two indices as 16-bit halves).
/// </remarks>
public readonly partial struct BRepMesh_OrientedEdge : IEquatable<BRepMesh_OrientedEdge>
{
    private readonly int myFirstNode;
    private readonly int myLastNode;

    /// <summary>The link of nodes -1 and -1.</summary>
    public BRepMesh_OrientedEdge()
    {
        myFirstNode = -1;
        myLastNode = -1;
    }

    /// <summary>A link from <paramref name="theFirstNode"/> to <paramref name="theLastNode"/>.</summary>
    public BRepMesh_OrientedEdge(int theFirstNode, int theLastNode)
    {
        myFirstNode = theFirstNode;
        myLastNode = theLastNode;
    }

    /// <summary>The index of the first node.</summary>
    public int FirstNode() => myFirstNode;

    /// <summary>The index of the last node.</summary>
    public int LastNode() => myLastNode;

    /// <summary>Whether both links have the same nodes in the same order.</summary>
    public bool IsEqual(in BRepMesh_OrientedEdge theOther)
        => myFirstNode == theOther.myFirstNode && myLastNode == theOther.myLastNode;

    public bool Equals(BRepMesh_OrientedEdge other) => IsEqual(other);

    public override bool Equals(object? obj) => obj is BRepMesh_OrientedEdge other && Equals(other);

    /// <summary>OCCT's hash: the low 16 bits of the first node, then of the last.</summary>
    public override int GetHashCode() => (int)((uint)(ushort)myFirstNode | ((uint)(ushort)myLastNode << 16));

    public static bool operator ==(in BRepMesh_OrientedEdge theLeft, in BRepMesh_OrientedEdge theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in BRepMesh_OrientedEdge theLeft, in BRepMesh_OrientedEdge theRight) => !theLeft.Equals(theRight);

    public override string ToString() => $"BRepMesh_OrientedEdge({myFirstNode}, {myLastNode})";
}
