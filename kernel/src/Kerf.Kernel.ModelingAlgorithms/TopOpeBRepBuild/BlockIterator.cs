// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_BlockIterator.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_BlockIterator.lxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepBuild/TopOpeBRepBuild_BlockIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans.Legacy.Build;

/// <summary>An iterator on the elements of a block of a <see cref="BlockBuilder"/>: an index range.</summary>
/// <remarks>A value, as in OCCT. <c>new()</c> is OCCT's default (an empty range starting at 1); <c>default</c> starts at
/// 0. The iteration methods are internal.</remarks>
public partial struct BlockIterator : IEquatable<BlockIterator>
{
    private int _lower;
    private int _upper;
    private int _value;

    /// <summary>An empty iterator.</summary>
    public BlockIterator()
    {
        _lower = 0;
        _upper = 0;
        _value = 1;
    }

    /// <summary>An iterator on the indices <paramref name="lower"/> to <paramref name="upper"/>.</summary>
    public BlockIterator(int lower, int upper)
    {
        _lower = lower;
        _upper = upper;
        _value = lower;
    }

    internal void Initialize()
    {
        _value = _lower;
    }

    /// <summary>Whether the current index is in the range.</summary>
    public readonly bool More()
    {
        bool b = _value <= _upper;
        return b;
    }

    internal void Next()
    {
        _value++;
    }

    /// <summary>The current index.</summary>
    public readonly int Value() => _value;

    /// <summary>The number of indices (0 for an empty iterator, whose lower bound is 0).</summary>
    public readonly int Extent()
    {
        if (_lower != 0)
        {
            int n = _upper - _lower + 1;
            return n;
        }
        return 0;
    }

    // OCCT.NET: value equality

    public readonly bool Equals(BlockIterator other)
        => _lower == other._lower && _upper == other._upper && _value == other._value;

    public override readonly bool Equals(object? obj) => obj is BlockIterator other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_lower, _upper, _value);

    public static bool operator ==(in BlockIterator left, in BlockIterator right)
        => left.Equals(right);

    public static bool operator !=(in BlockIterator left, in BlockIterator right)
        => !left.Equals(right);
}
