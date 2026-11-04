// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Edge.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Meshing.DegreeOfFreedom;

namespace Kerf.Engine.Meshing;

/// <summary>A link of the mesh, with its movability.</summary>
/// <remarks>
/// OCCT derives it from <see cref="OrientedEdge"/>; a struct can't, so it holds the nodes itself and converts
/// to an oriented edge implicitly, as C++ slices it. The default link joins nodes -1 and -1 and is deleted, as OCCT's;
/// <c>default(T)</c> joins 0 and 0 and is free, so ported code writes <c>new BRepMesh_Edge()</c>.
/// <see cref="Equals(MeshEdge)"/> and <c>==</c> compare the fields exactly (decision 539); OCCT's
/// <c>operator==</c> (the same nodes in either order, never a deleted link) is <see cref="IsEqual"/>, and maps in
/// ported code take <see cref="Hasher"/>.
/// </remarks>
internal partial struct MeshEdge : IEquatable<MeshEdge>
{
    private int _firstNode;
    private int _lastNode;
    private DegreeOfFreedom _movability;

    /// <summary>The deleted link of nodes -1 and -1.</summary>
    public MeshEdge()
    {
        _firstNode = -1;
        _lastNode = -1;
        _movability = DegreeOfFreedom.Deleted;
    }

    /// <summary>A link from <paramref name="firstNode"/> to <paramref name="lastNode"/>.</summary>
    public MeshEdge(int firstNode, int lastNode, DegreeOfFreedom movability)
    {
        _firstNode = firstNode;
        _lastNode = lastNode;
        _movability = movability;
    }

    /// <summary>The index of the first node.</summary>
    public readonly int FirstNode() => _firstNode;

    /// <summary>The index of the last node.</summary>
    public readonly int LastNode() => _lastNode;

    /// <summary>The movability.</summary>
    public readonly DegreeOfFreedom Movability() => _movability;

    /// <summary>Sets the movability.</summary>
    internal void SetMovability(DegreeOfFreedom movability) => _movability = movability;

    /// <summary>This link with another movability (the twin of <c>SetMovability</c>).</summary>
    public readonly MeshEdge WithMovability(DegreeOfFreedom movability)
        => new(_firstNode, _lastNode, movability);

    /// <summary>The nodes as an oriented edge (C++'s slicing to the base class).</summary>
    public static implicit operator OrientedEdge(in MeshEdge edge)
        => new(edge._firstNode, edge._lastNode);

    /// <summary>Whether both links have the same nodes in the same order.</summary>
    public readonly bool IsSameOrientation(in MeshEdge other)
        => _firstNode == other._firstNode && _lastNode == other._lastNode;

    /// <summary>OCCT's equality: false if either link is deleted, else whether they join the same nodes.</summary>
    public readonly bool IsEqual(in MeshEdge other)
    {
        if (_movability == DegreeOfFreedom.Deleted || other._movability == DegreeOfFreedom.Deleted)
        {
            return false;
        }

        return IsSameOrientation(other)
               || (FirstNode() == other.LastNode() && LastNode() == other.FirstNode());
    }

    /// <summary>OCCT's hash of a link: the low 16 bits of the smaller node, then of the larger.</summary>
    internal readonly uint OcctHash()
    {
        ushort a0 = (ushort)_firstNode;
        ushort a1 = (ushort)_lastNode;
        if (a0 > a1)
        {
            (a0, a1) = (a1, a0);
        }

        return a0 | ((uint)a1 << 16);
    }

    /// <summary>OCCT's hasher: <c>std::hash&lt;BRepMesh_Edge&gt;</c> and <c>operator==</c> (<see cref="IsEqual"/>).</summary>
    public sealed class Hasher : IEqualityComparer<MeshEdge>
    {
        public static readonly Hasher Instance = new();

        public bool Equals(MeshEdge e1, MeshEdge e2) => e1.IsEqual(e2);

        public int GetHashCode(MeshEdge edge) => (int)edge.OcctHash();
    }

    public readonly bool Equals(MeshEdge other)
        => _firstNode == other._firstNode && _lastNode == other._lastNode && _movability == other._movability;

    public override readonly bool Equals(object? obj) => obj is MeshEdge other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_firstNode, _lastNode, _movability);

    public static bool operator ==(in MeshEdge left, in MeshEdge right) => left.Equals(right);

    public static bool operator !=(in MeshEdge left, in MeshEdge right) => !left.Equals(right);

    public override readonly string ToString() => $"BRepMesh_Edge({_firstNode}, {_lastNode}, {_movability})";
}
