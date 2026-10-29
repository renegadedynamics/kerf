// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Array.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_ArrayOfEdges.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_ArrayOfPointNormal.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_ArrayOfPoints.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_ArrayOfSectionLines.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_ArrayOfTangentZones.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_ArrayOfTriangles.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_PMaillageAffinage.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// IntPolyh's dynamic array, indexed from 0: a number of "allocated" (constructed) items and a smaller number of stored
/// items, which <see cref="IncrementNbItems"/> grows, allocating <c>aIncrement</c> more items when it reaches the end.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's typedefs are written out: <c>IntPolyh_ArrayOfPoints</c> is <c>IntPolyh_Array&lt;IntPolyh_Point&gt;</c>, and
/// likewise ArrayOfEdges, ArrayOfTriangles, ArrayOfTangentZones (of <see cref="IntPolyh_StartPoint"/>),
/// ArrayOfSectionLines and ArrayOfPointNormal. <c>IntPolyh_PMaillageAffinage</c> (a raw pointer) is a reference to
/// <see cref="IntPolyh_MaillageAffinage"/>.
/// </para>
/// <para>
/// As OCCT's <c>NCollection_DynamicArray</c>, the items live in blocks that never move, so a <c>ref</c> to an item
/// stays valid while the array grows: <c>IntPolyh_Triangle::MiddleRefinement</c> appends triangles to the array that
/// holds the triangle being refined. Items are constructed with <c>new T()</c> (OCCT's default constructors), and
/// <see cref="Init"/> on an array that already holds items keeps them, as OCCT's does (decision 5501).
/// </para>
/// <para>
/// A working object of the intersection (decision 21): its readers are public, its mutators internal.
/// </para>
/// </remarks>
public sealed class IntPolyh_Array<T> where T : new()
{
    // OCCT's NCollection_DynamicArray default block size
    private const int THE_BLOCK_SIZE = 256;

    // Value types are constructed once and copied; IntPolyh_SectionLine (a class) is constructed for each item.
    private static readonly T? s_prototype = typeof(T).IsValueType ? new T() : default;

    private readonly List<T[]> myBlocks = [];
    private int myLength; // the items constructed (NCollection_DynamicArray::Length)

    private int myNbAllocated;
    private int myNbItems;
    private readonly int myIncrement;

    /// <summary>An empty array, growing by <paramref name="aIncrement"/> items; call <see cref="Init"/> first.</summary>
    public IntPolyh_Array(int aIncrement = 256)
    {
        myNbAllocated = 0;
        myNbItems = 0;
        myIncrement = aIncrement;
    }

    /// <summary>An array with <paramref name="aN"/> allocated items, growing by <paramref name="aIncrement"/>.</summary>
    public IntPolyh_Array(int aN, int aIncrement)
    {
        myNbItems = 0;
        myIncrement = aIncrement;
        Init(aN);
    }

    /// <summary>Copies the other array (its items, the stored and allocated numbers); C++'s <c>operator=</c>.</summary>
    internal IntPolyh_Array<T> Copy(IntPolyh_Array<T> aOther)
    {
        if (ReferenceEquals(this, aOther))
        {
            return this;
        }

        Clear();
        Init(aOther.myNbAllocated);
        // myVectorOfType = aOther.myVectorOfType: the other's items, as many as it has
        Clear();
        for (int i = 0; i < aOther.myLength; ++i)
        {
            T anItem = aOther.ItemAt(i);
            if (!typeof(T).IsValueType && anItem is IntPolyh_SectionLine aSL)
            {
                anItem = (T)(object)new IntPolyh_SectionLine(aSL);
            }

            AppendItem(anItem);
        }

        myNbItems = aOther.myNbItems;
        return this;
    }

    /// <summary>Allocates (constructs) items up to index <paramref name="aN"/>; the item at <paramref name="aN"/> is
    /// reset to a default one, the others are kept.</summary>
    internal void Init(int aN)
    {
        SetValue(aN, NewItem());
        myNbAllocated = aN;
    }

    /// <summary>Increments the number of stored items, allocating more when needed.</summary>
    internal void IncrementNbItems()
    {
        myNbItems++;
        if (myNbItems >= myNbAllocated)
        {
            int aN = myNbAllocated + myIncrement;
            Init(aN);
        }
    }

    /// <summary>The number of allocated items.</summary>
    public int GetN() => myNbAllocated;

    /// <summary>The number of stored items.</summary>
    public int NbItems() => myNbItems;

    /// <summary>Sets the number of stored items.</summary>
    internal void SetNbItems(int aNb) => myNbItems = aNb;

    /// <summary>The item at <paramref name="aIndex"/> (any allocated item, stored or not).</summary>
    public ref readonly T Value(int aIndex) => ref ChangeValue(aIndex);

    /// <summary>The item at <paramref name="aIndex"/>, for writing; the reference stays valid while the array grows.</summary>
    internal ref T ChangeValue(int aIndex)
    {
        if (aIndex < 0 || aIndex >= myLength)
        {
            throw new Standard_OutOfRange("NCollection_DynamicArray::ChangeValue");
        }

        return ref myBlocks[aIndex / THE_BLOCK_SIZE][aIndex % THE_BLOCK_SIZE];
    }

    /// <summary>C++'s <c>operator[]</c>.</summary>
    internal ref T this[int aIndex] => ref ChangeValue(aIndex);

    /// <summary>Writes the stored items that have a <c>Dump()</c>, as OCCT's <c>Dump()</c> prints them.</summary>
    public void Dump(TextWriter theStream)
    {
        theStream.Write($"\n ArrayOfSectionLines 0-> {IntPolyh_Format.D(myNbItems - 1)}");
        for (int i = 0; i < myNbItems; i++)
        {
            if (Value(i) is IntPolyh_IDump anItem)
            {
                anItem.Dump(theStream);
            }
        }

        theStream.Write("\n");
    }

    private static T NewItem() => typeof(T).IsValueType ? s_prototype! : new T();

    private T ItemAt(int aIndex) => myBlocks[aIndex / THE_BLOCK_SIZE][aIndex % THE_BLOCK_SIZE];

    private void Clear()
    {
        myBlocks.Clear();
        myLength = 0;
    }

    private void AppendItem(T theItem)
    {
        if (myLength == myBlocks.Count * THE_BLOCK_SIZE)
        {
            myBlocks.Add(new T[THE_BLOCK_SIZE]);
        }

        myBlocks[myLength / THE_BLOCK_SIZE][myLength % THE_BLOCK_SIZE] = theItem;
        myLength++;
    }

    // NCollection_DynamicArray::SetValue: grows with default items up to the index, then sets it
    private void SetValue(int aIndex, T theValue)
    {
        while (myLength <= aIndex)
        {
            AppendItem(NewItem());
        }

        ChangeValue(aIndex) = theValue;
    }
}

/// <summary>A point of a surface with the surface's normal there (OCCT's struct of IntPolyh_ArrayOfPointNormal).</summary>
/// <remarks>Both are zero by default, as C++ default-constructs <c>gp_Pnt</c> and <c>gp_Vec</c>.</remarks>
public struct IntPolyh_PointNormal : IEquatable<IntPolyh_PointNormal>
{
    public gp_Pnt Point;
    public gp_Vec Normal;

    public readonly bool Equals(IntPolyh_PointNormal other) => Point.Equals(other.Point) && Normal.Equals(other.Normal);

    public override readonly bool Equals(object? obj) => obj is IntPolyh_PointNormal other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(Point, Normal);

    public static bool operator ==(in IntPolyh_PointNormal theLeft, in IntPolyh_PointNormal theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntPolyh_PointNormal theLeft, in IntPolyh_PointNormal theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"IntPolyh_PointNormal({Point}, {Normal})";
}
