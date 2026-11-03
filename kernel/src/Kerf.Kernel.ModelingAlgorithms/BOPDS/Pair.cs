// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Pair.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Booleans.Data;

/// <summary>The pair of indices of two interfering shapes.</summary>
/// <remarks>
/// Equality is OCCT's: unordered, so (1, 2) equals (2, 1), and the hash is computed on the ordered pair as OCCT's
/// <c>std::hash&lt;BOPDS_Pair&gt;</c> does. The order (C++'s <c>operator&lt;</c>) is by the first index, then the
/// second; <see cref="LessThan"/> gives it to <see cref="Sorting"/>.
/// </remarks>
public partial struct Pair : IEquatable<Pair>, IComparable<Pair>
{
    private int _index1;
    private int _index2;

    /// <summary>The pair (-1, -1).</summary>
    public Pair()
    {
        _index1 = -1;
        _index2 = -1;
    }

    public Pair(int index1, int index2)
    {
        _index1 = index1;
        _index2 = index2;
    }

    internal void SetIndices(int index1, int index2)
    {
        _index1 = index1;
        _index2 = index2;
    }

    /// <summary>The indices.</summary>
    public readonly void Indices(out int index1, out int index2)
    {
        index1 = _index1;
        index2 = _index2;
    }

    /// <summary>C++'s <c>operator&lt;</c>: by the first index, then the second.</summary>
    public static bool operator <(Pair left, Pair other) =>
        (left._index1 != other._index1) ? (left._index1 < other._index1) : (left._index2 < other._index2);

    public static bool operator >(Pair left, Pair other) => other < left;

    /// <summary>True if the pairs hold the same indices, in either order.</summary>
    public readonly bool IsEqual(in Pair other) =>
        (_index1 == other._index1 && _index2 == other._index2)
        || (_index1 == other._index2 && _index2 == other._index1);

    // ---- OCCT.NET additions

    /// <summary>C++'s <c>operator&lt;</c> for <see cref="Sorting"/>.</summary>
    public readonly struct LessThan : Heap.ILess<Pair>
    {
        public bool Less(Pair left, Pair right) => left < right;
    }

    public readonly int CompareTo(Pair other) => this < other ? -1 : (other < this ? 1 : 0);

    public readonly bool Equals(Pair other) => IsEqual(other);

    public override readonly bool Equals(object? obj) => obj is Pair other && IsEqual(other);

    public override readonly int GetHashCode() =>
        _index1 > _index2 ? HashCode.Combine(_index2, _index1) : HashCode.Combine(_index1, _index2);

    public static bool operator ==(Pair left, Pair right) => left.IsEqual(right);

    public static bool operator !=(Pair left, Pair right) => !left.IsEqual(right);

    public override readonly string ToString() => Constants.Format("BOPDS_Pair", _index1, _index2);
}
