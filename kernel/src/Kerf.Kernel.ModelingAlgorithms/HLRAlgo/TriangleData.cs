// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_TriangleData.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>A triangle of the polygonal hidden-line removal: its three nodes and its flags.</summary>
/// <remarks>A plain C++ struct, ported with OCCT's public fields (plain data, changed in place in arrays).</remarks>
internal struct TriangleData : IEquatable<TriangleData>
{
    public int Node1;
    public int Node2;
    public int Node3;
    public int Flags;

    public readonly bool Equals(TriangleData other)
        => Node1 == other.Node1 && Node2 == other.Node2 && Node3 == other.Node3 && Flags == other.Flags;

    public override readonly bool Equals(object? obj) => obj is TriangleData other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(Node1, Node2, Node3, Flags);

    public static bool operator ==(in TriangleData left, in TriangleData right)
        => left.Equals(right);

    public static bool operator !=(in TriangleData left, in TriangleData right)
        => !left.Equals(right);

    public override readonly string ToString() => $"HLRAlgo_TriangleData({Node1}, {Node2}, {Node3}, {Flags})";
}
