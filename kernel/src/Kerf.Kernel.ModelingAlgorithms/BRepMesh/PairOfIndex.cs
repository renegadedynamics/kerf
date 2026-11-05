// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_PairOfIndex.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2009-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Meshing;

/// <summary>Up to two indices: the triangles connected to a link. Storing a third throws.</summary>
/// <remarks>
/// The default pair is empty (both indices -1), as OCCT's; <c>default(T)</c> holds 0 and 0, so ported code writes
/// <c>new BRepMesh_PairOfIndex()</c>.
/// </remarks>
internal partial struct PairOfIndex : IEquatable<PairOfIndex>
{
    private int _index0;
    private int _index1;

    /// <summary>An empty pair.</summary>
    public PairOfIndex() => Clear();

    /// <summary>Empties the pair.</summary>
    internal void Clear() => _index0 = _index1 = -1;

    /// <summary>Appends an index; throws <see cref="OutOfRangeException"/> if the pair is full.</summary>
    internal void Append(int index)
    {
        if (_index0 < 0)
        {
            _index0 = index;
        }
        else
        {
            if (_index1 >= 0)
            {
                throw new OutOfRangeException("BRepMesh_PairOfIndex::Append, more than two index to store");
            }

            _index1 = index;
        }
    }

    /// <summary>Prepends an index; throws <see cref="OutOfRangeException"/> if the pair is full.</summary>
    internal void Prepend(int index)
    {
        if (_index1 >= 0)
        {
            throw new OutOfRangeException("BRepMesh_PairOfIndex::Prepend, more than two index to store");
        }

        _index1 = _index0;
        _index0 = index;
    }

    /// <summary>Whether the pair is empty (only the first index is checked: the second can't be set alone).</summary>
    public readonly bool IsEmpty => _index0 < 0;

    /// <summary>The number of indices stored.</summary>
    public readonly int Extent() => _index0 < 0 ? 0 : (_index1 < 0 ? 1 : 2);

    /// <summary>The first index.</summary>
    public readonly int FirstIndex() => _index0;

    /// <summary>The last index stored.</summary>
    public readonly int LastIndex() => _index1 < 0 ? _index0 : _index1;

    /// <summary>The index at position 1 or 2; throws <see cref="OutOfRangeException"/> otherwise.</summary>
    public readonly int Index(int pairPos)
    {
        if (pairPos != 1 && pairPos != 2)
        {
            throw new OutOfRangeException("BRepMesh_PairOfIndex::Index, requested index is out of range");
        }

        return pairPos == 1 ? _index0 : _index1;
    }

    /// <summary>Sets the index at position 1 or 2; throws <see cref="OutOfRangeException"/> otherwise.</summary>
    internal void SetIndex(int pairPos, int index)
    {
        if (pairPos != 1 && pairPos != 2)
        {
            throw new OutOfRangeException("BRepMesh_PairOfIndex::SetIndex, requested index is out of range");
        }

        if (pairPos == 1)
        {
            _index0 = index;
        }
        else
        {
            _index1 = index;
        }
    }

    /// <summary>Removes the index at position 1 or 2; throws <see cref="OutOfRangeException"/> otherwise.</summary>
    internal void RemoveIndex(int pairPos)
    {
        if (pairPos != 1 && pairPos != 2)
        {
            throw new OutOfRangeException("BRepMesh_PairOfIndex::RemoveIndex, requested index is out of range");
        }

        if (pairPos == 1)
        {
            _index0 = _index1;
        }

        _index1 = -1;
    }

    // Non-mutating twins

    /// <summary>This pair with an index appended (the twin of <c>Append</c>).</summary>
    public readonly PairOfIndex Appended(int index)
    {
        PairOfIndex copy = this;
        copy.Append(index);
        return copy;
    }

    /// <summary>This pair with an index prepended (the twin of <c>Prepend</c>).</summary>
    public readonly PairOfIndex Prepended(int index)
    {
        PairOfIndex copy = this;
        copy.Prepend(index);
        return copy;
    }

    /// <summary>This pair with the index at a position replaced (the twin of <c>SetIndex</c>).</summary>
    public readonly PairOfIndex WithIndex(int pairPos, int index)
    {
        PairOfIndex copy = this;
        copy.SetIndex(pairPos, index);
        return copy;
    }

    /// <summary>This pair with the index at a position removed (the twin of <c>RemoveIndex</c>).</summary>
    public readonly PairOfIndex IndexRemoved(int pairPos)
    {
        PairOfIndex copy = this;
        copy.RemoveIndex(pairPos);
        return copy;
    }

    public readonly bool Equals(PairOfIndex other) => _index0 == other._index0 && _index1 == other._index1;

    public override readonly bool Equals(object? obj) => obj is PairOfIndex other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_index0, _index1);

    public static bool operator ==(in PairOfIndex left, in PairOfIndex right) => left.Equals(right);

    public static bool operator !=(in PairOfIndex left, in PairOfIndex right) => !left.Equals(right);

    public override readonly string ToString() => $"BRepMesh_PairOfIndex({_index0}, {_index1})";
}
