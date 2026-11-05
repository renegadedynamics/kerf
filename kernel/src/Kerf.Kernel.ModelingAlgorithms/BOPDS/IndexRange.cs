// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_IndexRange.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_IndexRange.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_IndexRange.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Booleans.Data;

/// <summary>The range of indices [First, Last] of an argument and its sub-shapes in the data structure.</summary>
internal partial struct IndexRange : IEquatable<IndexRange>
{
    private int _first;
    private int _last;

    /// <summary>The range [0, 0].</summary>
    public IndexRange()
    {
        _first = 0;
        _last = 0;
    }

    public IndexRange(int i1, int i2)
    {
        _first = i1;
        _last = i2;
    }

    internal void SetFirst(int aFirst) => _first = aFirst;

    public readonly int First() => _first;

    internal void SetLast(int aLast) => _last = aLast;

    public readonly int Last() => _last;

    internal void SetIndices(int i1, int i2)
    {
        _first = i1;
        _last = i2;
    }

    public readonly void Indices(out int i1, out int i2)
    {
        i1 = _first;
        i2 = _last;
    }

    /// <summary>True if the range contains the index.</summary>
    public readonly bool Contains(int aIndex) => aIndex >= _first && aIndex <= _last;

    /// <summary>Writes the range as OCCT's <c>Dump</c> prints it.</summary>
    public readonly void Dump(TextWriter os) => os.Write(" range: " + _first + " " + _last);

    // ---- Kerf additions

    public readonly IndexRange WithIndices(int i1, int i2) => new(i1, i2);

    public readonly bool Equals(IndexRange other) => _first == other._first && _last == other._last;

    public override readonly bool Equals(object? obj) => obj is IndexRange other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_first, _last);

    public static bool operator ==(IndexRange left, IndexRange right) => left.Equals(right);

    public static bool operator !=(IndexRange left, IndexRange right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("IndexRange", _first, _last);
}
