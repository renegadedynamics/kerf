// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_OrientedEdge.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Meshing;

/// <summary>A link of the mesh between two nodes, from the first to the last.</summary>
/// <remarks>
/// The default link joins nodes -1 and -1, as OCCT's; <c>default(T)</c> joins 0 and 0, so ported code writes
/// <c>new BRepMesh_OrientedEdge()</c>. OCCT's equality is exact, so <see cref="Equals(OrientedEdge)"/> is
/// OCCT's <c>IsEqual</c>, and <see cref="GetHashCode"/> is OCCT's hash (the two indices as 16-bit halves).
/// </remarks>
internal readonly partial struct OrientedEdge : IEquatable<OrientedEdge>
{
    private readonly int _firstNode;
    private readonly int _lastNode;

    /// <summary>The link of nodes -1 and -1.</summary>
    public OrientedEdge()
    {
        _firstNode = -1;
        _lastNode = -1;
    }

    /// <summary>A link from <paramref name="firstNode"/> to <paramref name="lastNode"/>.</summary>
    public OrientedEdge(int firstNode, int lastNode)
    {
        _firstNode = firstNode;
        _lastNode = lastNode;
    }

    /// <summary>The index of the first node.</summary>
    public int FirstNode() => _firstNode;

    /// <summary>The index of the last node.</summary>
    public int LastNode() => _lastNode;

    /// <summary>Whether both links have the same nodes in the same order.</summary>
    public bool IsEqual(in OrientedEdge other)
        => _firstNode == other._firstNode && _lastNode == other._lastNode;

    public bool Equals(OrientedEdge other) => IsEqual(other);

    public override bool Equals(object? obj) => obj is OrientedEdge other && Equals(other);

    /// <summary>OCCT's hash: the low 16 bits of the first node, then of the last.</summary>
    public override int GetHashCode() => (int)((uint)(ushort)_firstNode | ((uint)(ushort)_lastNode << 16));

    public static bool operator ==(in OrientedEdge left, in OrientedEdge right) => left.Equals(right);

    public static bool operator !=(in OrientedEdge left, in OrientedEdge right) => !left.Equals(right);

    public override string ToString() => $"BRepMesh_OrientedEdge({_firstNode}, {_lastNode})";
}
