// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_PairOfIndex.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2009-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Up to two indices: the triangles connected to a link. Storing a third throws.</summary>
/// <remarks>
/// The default pair is empty (both indices -1), as OCCT's; <c>default(T)</c> holds 0 and 0, so ported code writes
/// <c>new BRepMesh_PairOfIndex()</c>.
/// </remarks>
public partial struct BRepMesh_PairOfIndex : IEquatable<BRepMesh_PairOfIndex>
{
    private int myIndex0;
    private int myIndex1;

    /// <summary>An empty pair.</summary>
    public BRepMesh_PairOfIndex() => Clear();

    /// <summary>Empties the pair.</summary>
    internal void Clear() => myIndex0 = myIndex1 = -1;

    /// <summary>Appends an index; throws <see cref="Standard_OutOfRange"/> if the pair is full.</summary>
    internal void Append(int theIndex)
    {
        if (myIndex0 < 0)
        {
            myIndex0 = theIndex;
        }
        else
        {
            if (myIndex1 >= 0)
            {
                throw new Standard_OutOfRange("BRepMesh_PairOfIndex::Append, more than two index to store");
            }

            myIndex1 = theIndex;
        }
    }

    /// <summary>Prepends an index; throws <see cref="Standard_OutOfRange"/> if the pair is full.</summary>
    internal void Prepend(int theIndex)
    {
        if (myIndex1 >= 0)
        {
            throw new Standard_OutOfRange("BRepMesh_PairOfIndex::Prepend, more than two index to store");
        }

        myIndex1 = myIndex0;
        myIndex0 = theIndex;
    }

    /// <summary>Whether the pair is empty (only the first index is checked: the second can't be set alone).</summary>
    public readonly bool IsEmpty() => myIndex0 < 0;

    /// <summary>The number of indices stored.</summary>
    public readonly int Extent() => myIndex0 < 0 ? 0 : (myIndex1 < 0 ? 1 : 2);

    /// <summary>The first index.</summary>
    public readonly int FirstIndex() => myIndex0;

    /// <summary>The last index stored.</summary>
    public readonly int LastIndex() => myIndex1 < 0 ? myIndex0 : myIndex1;

    /// <summary>The index at position 1 or 2; throws <see cref="Standard_OutOfRange"/> otherwise.</summary>
    public readonly int Index(int thePairPos)
    {
        if (thePairPos != 1 && thePairPos != 2)
        {
            throw new Standard_OutOfRange("BRepMesh_PairOfIndex::Index, requested index is out of range");
        }

        return thePairPos == 1 ? myIndex0 : myIndex1;
    }

    /// <summary>Sets the index at position 1 or 2; throws <see cref="Standard_OutOfRange"/> otherwise.</summary>
    internal void SetIndex(int thePairPos, int theIndex)
    {
        if (thePairPos != 1 && thePairPos != 2)
        {
            throw new Standard_OutOfRange("BRepMesh_PairOfIndex::SetIndex, requested index is out of range");
        }

        if (thePairPos == 1)
        {
            myIndex0 = theIndex;
        }
        else
        {
            myIndex1 = theIndex;
        }
    }

    /// <summary>Removes the index at position 1 or 2; throws <see cref="Standard_OutOfRange"/> otherwise.</summary>
    internal void RemoveIndex(int thePairPos)
    {
        if (thePairPos != 1 && thePairPos != 2)
        {
            throw new Standard_OutOfRange("BRepMesh_PairOfIndex::RemoveIndex, requested index is out of range");
        }

        if (thePairPos == 1)
        {
            myIndex0 = myIndex1;
        }

        myIndex1 = -1;
    }

    // Non-mutating twins

    /// <summary>This pair with an index appended (the twin of <c>Append</c>).</summary>
    public readonly BRepMesh_PairOfIndex Appended(int theIndex)
    {
        BRepMesh_PairOfIndex aCopy = this;
        aCopy.Append(theIndex);
        return aCopy;
    }

    /// <summary>This pair with an index prepended (the twin of <c>Prepend</c>).</summary>
    public readonly BRepMesh_PairOfIndex Prepended(int theIndex)
    {
        BRepMesh_PairOfIndex aCopy = this;
        aCopy.Prepend(theIndex);
        return aCopy;
    }

    /// <summary>This pair with the index at a position replaced (the twin of <c>SetIndex</c>).</summary>
    public readonly BRepMesh_PairOfIndex WithIndex(int thePairPos, int theIndex)
    {
        BRepMesh_PairOfIndex aCopy = this;
        aCopy.SetIndex(thePairPos, theIndex);
        return aCopy;
    }

    /// <summary>This pair with the index at a position removed (the twin of <c>RemoveIndex</c>).</summary>
    public readonly BRepMesh_PairOfIndex IndexRemoved(int thePairPos)
    {
        BRepMesh_PairOfIndex aCopy = this;
        aCopy.RemoveIndex(thePairPos);
        return aCopy;
    }

    public readonly bool Equals(BRepMesh_PairOfIndex other) => myIndex0 == other.myIndex0 && myIndex1 == other.myIndex1;

    public override readonly bool Equals(object? obj) => obj is BRepMesh_PairOfIndex other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myIndex0, myIndex1);

    public static bool operator ==(in BRepMesh_PairOfIndex theLeft, in BRepMesh_PairOfIndex theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in BRepMesh_PairOfIndex theLeft, in BRepMesh_PairOfIndex theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"BRepMesh_PairOfIndex({myIndex0}, {myIndex1})";
}
