// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Triangle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Runtime.CompilerServices;
using static Kerf.Engine.Meshing.DegreeOfFreedom;

namespace Kerf.Engine.Meshing;

/// <summary>A triangle of the mesh: three links (by index) with their orientations, and a movability.</summary>
/// <remarks>
/// OCCT's public arrays <c>myEdges[3]</c> and <c>myOrientations[3]</c> are internal inline arrays here, read by ported
/// code as in C++ (<c>var e = aElement.myEdges; e[i]</c>); callers use <see cref="Edges"/>. The default triangle has
/// links 0, 0, 0, orientations false and is free, as OCCT's; <c>default(T)</c> is the same.
/// <see cref="Equals(Triangle)"/> and <c>==</c> compare the fields exactly (decision 539); OCCT's
/// <c>operator==</c> (the same links in the same cyclic order, never a deleted triangle) is <see cref="IsEqual"/>,
/// and maps in ported code take <see cref="Hasher"/>.
/// </remarks>
public partial struct Triangle : IEquatable<Triangle>
{
    /// <summary>Three link indices (C++'s <c>int[3]</c>).</summary>
    [InlineArray(3)]
    internal struct Int3
    {
        private int _element0;
    }

    /// <summary>Three orientations (C++'s <c>bool[3]</c>).</summary>
    [InlineArray(3)]
    internal struct Bool3
    {
        private bool _element0;
    }

    internal Int3 _edges;
    internal Bool3 _orientations;
    internal DegreeOfFreedom _movability;

    /// <summary>A triangle of the given three links (by index) and orientations.</summary>
    public Triangle(ReadOnlySpan<int> edges, ReadOnlySpan<bool> orientations, DegreeOfFreedom movability)
    {
        Initialize(edges, orientations, movability);
    }

    /// <summary>Sets the links, the orientations and the movability.</summary>
    internal void Initialize(ReadOnlySpan<int> edges, ReadOnlySpan<bool> orientations, DegreeOfFreedom movability)
    {
        edges[..3].CopyTo(_edges);
        orientations[..3].CopyTo(_orientations);
        _movability = movability;
    }

    /// <summary>The links and their orientations, each into a span of three.</summary>
    public readonly void Edges(Span<int> edges, Span<bool> orientations)
    {
        ((ReadOnlySpan<int>)_edges).CopyTo(edges);
        ((ReadOnlySpan<bool>)_orientations).CopyTo(orientations);
    }

    /// <summary>The index of the link at position 0, 1 or 2.</summary>
    public readonly int Edge(int index) => _edges[index];

    /// <summary>The orientation of the link at position 0, 1 or 2.</summary>
    public readonly bool Orientation(int index) => _orientations[index];

    /// <summary>The movability.</summary>
    public readonly DegreeOfFreedom Movability() => _movability;

    /// <summary>Sets the movability.</summary>
    internal void SetMovability(DegreeOfFreedom movability) => _movability = movability;

    /// <summary>This triangle with another movability (the twin of <c>SetMovability</c>).</summary>
    public readonly Triangle WithMovability(DegreeOfFreedom movability)
    {
        Triangle copy = this;
        copy.SetMovability(movability);
        return copy;
    }

    /// <summary>OCCT's equality: false if either triangle is deleted, else whether they have the same links in the
    /// same cyclic order.</summary>
    public readonly bool IsEqual(in Triangle other)
    {
        if (_movability == DegreeOfFreedom.Deleted || other._movability == DegreeOfFreedom.Deleted)
        {
            return false;
        }

        if (_edges[0] == other._edges[0] && _edges[1] == other._edges[1]
            && _edges[2] == other._edges[2])
        {
            return true;
        }

        if (_edges[0] == other._edges[1] && _edges[1] == other._edges[2]
            && _edges[2] == other._edges[0])
        {
            return true;
        }

        if (_edges[0] == other._edges[2] && _edges[1] == other._edges[0]
            && _edges[2] == other._edges[1])
        {
            return true;
        }

        return false;
    }

    /// <summary>
    /// OCCT's hasher: <c>operator==</c> (<see cref="IsEqual"/>) and a hash of the sorted link indices (OCCT hashes their
    /// bytes; the value differs, which only changes the order of a non-indexed map).
    /// </summary>
    public sealed class Hasher : IEqualityComparer<Triangle>
    {
        public static readonly Hasher Instance = new();

        public bool Equals(Triangle t1, Triangle t2) => t1.IsEqual(t2);

        public int GetHashCode(Triangle triangle)
        {
            Span<int> combination = [triangle._edges[0], triangle._edges[1], triangle._edges[2]];
            combination.Sort();
            return HashCode.Combine(combination[0], combination[1], combination[2]);
        }
    }

    public readonly bool Equals(Triangle other)
        => _edges[0] == other._edges[0] && _edges[1] == other._edges[1] && _edges[2] == other._edges[2]
           && _orientations[0] == other._orientations[0] && _orientations[1] == other._orientations[1]
           && _orientations[2] == other._orientations[2] && _movability == other._movability;

    public override readonly bool Equals(object? obj) => obj is Triangle other && Equals(other);

    public override readonly int GetHashCode()
        => HashCode.Combine(_edges[0], _edges[1], _edges[2], _orientations[0], _orientations[1], _orientations[2], _movability);

    public static bool operator ==(in Triangle left, in Triangle right) => left.Equals(right);

    public static bool operator !=(in Triangle left, in Triangle right) => !left.Equals(right);

    public override readonly string ToString()
        => $"BRepMesh_Triangle({(_orientations[0] ? "" : "-")}{_edges[0]}, {(_orientations[1] ? "" : "-")}{_edges[1]}, {(_orientations[2] ? "" : "-")}{_edges[2]}, {_movability})";
}
