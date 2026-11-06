// Port of: src/FoundationClasses/TKMath/Poly/Poly_Triangle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;

namespace Kerf.Engine.Meshing;

/// <summary>
/// A triangle of a triangulation (<see cref="Triangulation"/>): three node indices, each in
/// [1, Poly_Triangulation.NbNodes()].
/// </summary>
/// <remarks>
/// Three <c>int</c>s and nothing else, so an array of triangles reads as an array of 3 × n indices
/// (<see cref="Triangulation.TriangleIndices"/>). The default triangle has all three indices 0, as OCCT's.
/// </remarks>
internal partial struct Triangle1 : IEquatable<Triangle1>
{
    private NCollection_Buffer3<int> _nodes;

    /// <summary>A triangle with the three node indices.</summary>
    public Triangle1(int n1, int n2, int n3)
    {
        _nodes[0] = n1;
        _nodes[1] = n2;
        _nodes[2] = n3;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the three node indices.</summary>
    internal void Set(int n1, int n2, int n3)
    {
        _nodes[0] = n1;
        _nodes[1] = n2;
        _nodes[2] = n3;
    }

    /// <summary>Sets the node index at position 1, 2 or 3; throws <see cref="OutOfRangeException"/> otherwise.</summary>
    internal void Set(int index, int node)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > 3, "Poly_Triangle::Set(), invalid index");
        _nodes[index - 1] = node;
    }

    /// <summary>The node index at position 1, 2 or 3, for writing.</summary>
    [UnscopedRef]
    internal ref int ChangeValue(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > 3, "Poly_Triangle::ChangeValue(), invalid index");
        return ref _nodes[index - 1];
    }

    // Non-mutating twins

    /// <summary>This triangle with the node index at position 1, 2 or 3 replaced (the twin of <c>Set(index, node)</c>).</summary>
    public readonly Triangle1 WithNode(int index, int node)
    {
        Triangle1 copy = this;
        copy.Set(index, node);
        return copy;
    }

    /// <summary>The three node indices.</summary>
    public readonly void Get(out int n1, out int n2, out int n3)
    {
        n1 = _nodes[0];
        n2 = _nodes[1];
        n3 = _nodes[2];
    }

    /// <summary>The node index at position 1, 2 or 3; throws <see cref="OutOfRangeException"/> otherwise.</summary>
    public readonly int Value(int index)
    {
        Standard_OutOfRange_Raise_if(index < 1 || index > 3, "Poly_Triangle::Value(), invalid index");
        return _nodes[index - 1];
    }

    /// <summary>The node index at position 1, 2 or 3 (OCCT's <c>operator()</c>).</summary>
    public readonly int this[int index] => Value(index);

    // Equality and formatting (Kerf additions)

    public readonly bool Equals(Triangle1 other) => _nodes[0] == other._nodes[0] && _nodes[1] == other._nodes[1] && _nodes[2] == other._nodes[2];

    public override readonly bool Equals(object? obj) => obj is Triangle1 other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_nodes[0], _nodes[1], _nodes[2]);

    public static bool operator ==(in Triangle1 left, in Triangle1 right) => left.Equals(right);

    public static bool operator !=(in Triangle1 left, in Triangle1 right) => !left.Equals(right);

    public override readonly string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Poly_Triangle({_nodes[0]}, {_nodes[1]}, {_nodes[2]})");
}
