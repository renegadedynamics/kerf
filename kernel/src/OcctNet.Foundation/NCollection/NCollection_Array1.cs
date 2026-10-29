// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Array1.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_HArray1.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections;
using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// OCCT's array with arbitrary bounds (1-based by convention). A mutable container for ported algorithms and
/// for the inputs and outputs of low-level APIs; immutable objects return <see cref="PackedList{T}"/> instead.
/// </summary>
/// <remarks>
/// Not OCCT's implementation: a .NET array with OCCT's API. Two differences matter when porting:
/// <list type="bullet">
/// <item>It is a class, so C++ copy construction and assignment (<c>NCollection_Array1&lt;T&gt; a = b;</c>,
/// <c>a = b;</c>) must be written <c>new(b)</c> and <c>a.Assign(b)</c>. Plain C# assignment shares the array.</item>
/// <item>Elements are default-constructed as in C++: a type's parameterless constructor runs (so a
/// <c>gp_Dir</c> element is (1, 0, 0), not zero).</item>
/// </list>
/// An array made over another array's storage (OCCT's "buffer" constructors) shares it, as in OCCT.
/// </remarks>
public class NCollection_Array1<T> : IEnumerable<T>
{
    private T[] myData;
    private int myOffset;
    private int mySize;
    private int myLowerBound;
    private bool myIsOwner;

    public NCollection_Array1()
    {
        myData = [];
        myLowerBound = 1;
        myIsOwner = true;
    }

    public NCollection_Array1(int theLower, int theUpper)
    {
        Standard_RangeError_Raise_if(theUpper < theLower - 1, "NCollection_Array1::Create");
        myLowerBound = theLower;
        mySize = theUpper - theLower + 1;
        myData = NCollection_Default<T>.NewArray(mySize);
        myIsOwner = true;
    }

    /// <summary>A 0-based array of the given size.</summary>
    public NCollection_Array1(int theSize) : this(0, theSize - 1) { }

    /// <summary>A copy (OCCT's copy constructor).</summary>
    public NCollection_Array1(NCollection_Array1<T> theOther)
    {
        myLowerBound = theOther.myLowerBound;
        mySize = theOther.mySize;
        myData = theOther.AsSpan().ToArray();
        myIsOwner = true;
    }

    /// <summary>
    /// An array over another array's storage, starting at its element <paramref name="theOtherIndex"/>, with bounds
    /// <paramref name="theLower"/>..<paramref name="theUpper"/>: OCCT's <c>NCollection_Array1(theOther(i), lower, upper)</c>.
    /// Writes go to <paramref name="theOther"/>.
    /// </summary>
    public NCollection_Array1(NCollection_Array1<T> theOther, int theOtherIndex, int theLower, int theUpper)
    {
        int aSize = theUpper - theLower + 1;
        int aStart = theOther.myOffset + (theOtherIndex - theOther.myLowerBound);
        Standard_RangeError_Raise_if(aSize < 0 || aStart < theOther.myOffset || aStart + aSize > theOther.myOffset + theOther.mySize,
                                     "NCollection_Array1::Create");
        myData = theOther.myData;
        myOffset = aStart;
        mySize = aSize;
        myLowerBound = theLower;
        myIsOwner = false;
    }

    /// <summary>An array over a .NET array's storage, without copying. Writes go to <paramref name="theData"/>.</summary>
    public NCollection_Array1(T[] theData, int theOffset, int theLower, int theUpper)
    {
        myData = theData;
        myOffset = theOffset;
        mySize = theUpper - theLower + 1;
        myLowerBound = theLower;
        myIsOwner = false;
    }

    /// <summary>An owning array holding a copy of the values, with the given lower bound.</summary>
    public static NCollection_Array1<T> From(ReadOnlySpan<T> theValues, int theLower = 1)
        => new(theValues.ToArray(), 0, theLower, theLower + theValues.Length - 1) { myIsOwner = true };

    /// <summary>Sets every element to <paramref name="theValue"/>.</summary>
    public void Init(T theValue) => AsSpan().Fill(theValue);

    public int Size() => mySize;

    public int Length() => mySize;

    public bool IsEmpty() => mySize == 0;

    public int Lower() => myLowerBound;

    public int Upper() => myLowerBound + mySize - 1;

    public bool IsDeletable() => myIsOwner;

    public T First() => myData[myOffset];

    public ref T ChangeFirst() => ref myData[myOffset];

    public T Last() => myData[myOffset + mySize - 1];

    public ref T ChangeLast() => ref myData[myOffset + mySize - 1];

    public T Value(int theIndex) => myData[Position(theIndex)];

    public ref T ChangeValue(int theIndex) => ref myData[Position(theIndex)];

    /// <summary>The element at an index within the bounds: OCCT's <c>operator()</c> and <c>operator[]</c>, for reading and writing.</summary>
    public ref T this[int theIndex] => ref myData[Position(theIndex)];

    public void SetValue(int theIndex, T theItem) => myData[Position(theIndex)] = theItem;

    /// <summary>The element at a 0-based position, whatever the bounds.</summary>
    public T At(int theIndex) => myData[myOffset + CheckAt(theIndex)];

    public ref T ChangeAt(int theIndex) => ref myData[myOffset + CheckAt(theIndex)];

    public void UpdateLowerBound(int theLower) => myLowerBound = theLower;

    public void UpdateUpperBound(int theUpper) => myLowerBound = myLowerBound - Upper() + theUpper;

    /// <summary>Copies the values of an array of the same length (OCCT's <c>operator=</c> and <c>Assign</c>).</summary>
    public NCollection_Array1<T> Assign(NCollection_Array1<T> theOther)
    {
        if (ReferenceEquals(this, theOther))
        {
            return this;
        }

        if (myIsOwner)
        {
            if (mySize != theOther.mySize)
            {
                myData = new T[theOther.mySize];
                myOffset = 0;
                mySize = theOther.mySize;
            }

            myLowerBound = theOther.myLowerBound;
        }
        else
        {
            Standard_DimensionMismatch_Raise_if(mySize != theOther.mySize, "NCollection_Array1::operator=");
        }

        theOther.AsSpan().CopyTo(AsSpan());
        return this;
    }

    /// <summary>
    /// Copies the values of an array of the same length, keeping this array's bounds (OCCT's <c>CopyValues</c>).
    /// Throws <see cref="Standard_DimensionMismatch"/> if the lengths differ.
    /// </summary>
    public NCollection_Array1<T> CopyValues(NCollection_Array1<T> theOther)
    {
        if (ReferenceEquals(this, theOther))
        {
            return this;
        }

        Standard_DimensionMismatch_Raise_if(mySize != theOther.mySize, "NCollection_Array1::CopyValues");
        theOther.AsSpan().CopyTo(AsSpan());
        return this;
    }

    /// <summary>Takes over another array's storage, leaving it empty (OCCT's <c>Move</c>).</summary>
    public NCollection_Array1<T> Move(NCollection_Array1<T> theOther)
    {
        if (ReferenceEquals(this, theOther))
        {
            return this;
        }

        myData = theOther.myData;
        myOffset = theOther.myOffset;
        mySize = theOther.mySize;
        myLowerBound = theOther.myLowerBound;
        myIsOwner = theOther.myIsOwner;
        theOther.myData = [];
        theOther.myOffset = 0;
        theOther.mySize = 0;
        theOther.myIsOwner = true;
        return this;
    }

    /// <summary>Resizes to new bounds; as in OCCT, the range may not be empty.</summary>
    public void Resize(int theLower, int theUpper, bool theToCopyData)
    {
        Standard_RangeError_Raise_if(theUpper < theLower, "NCollection_Array1::Resize");
        ResizeImpl(theUpper - theLower + 1, theLower, theToCopyData);
    }

    /// <summary>Resizes to a number of elements, keeping the lower bound; as in OCCT, zero is allowed.</summary>
    public void Resize(int theSize, bool theToCopyData)
    {
        Standard_RangeError_Raise_if(theSize < 0, "NCollection_Array1::Resize");
        ResizeImpl(theSize, myLowerBound, theToCopyData);
    }

    private void ResizeImpl(int aNewSize, int theLower, bool theToCopyData)
    {
        myLowerBound = theLower;
        if (aNewSize == mySize)
        {
            return;
        }

        T[] aNewData = NCollection_Default<T>.NewArray(aNewSize);
        if (theToCopyData)
        {
            AsSpan()[..Math.Min(mySize, aNewSize)].CopyTo(aNewData);
        }

        myData = aNewData;
        myOffset = 0;
        mySize = aNewSize;
        myIsOwner = true;
    }

    /// <summary>The elements, without copying; index 0 is <see cref="Lower"/>.</summary>
    public Span<T> AsSpan() => new(myData, myOffset, mySize);

    /// <summary>The elements from index <paramref name="theFrom"/> to the upper bound, without copying.</summary>
    public Span<T> AsSpan(int theFrom) => new(myData, Position(theFrom), Upper() - theFrom + 1);

    /// <summary>The backing .NET array and the position of <see cref="Lower"/> in it, for views over this array.</summary>
    internal (T[] Data, int Offset) Storage() => (myData, myOffset);

    /// <summary>An immutable copy, for returning from immutable objects.</summary>
    public PackedList<T> ToPackedList() => PackedList.Create<T>(AsSpan());

    public Span<T>.Enumerator GetEnumerator() => AsSpan().GetEnumerator();

    IEnumerator<T> IEnumerable<T>.GetEnumerator()
    {
        for (int i = 0; i < mySize; i++)
        {
            yield return myData[myOffset + i];
        }
    }

    IEnumerator IEnumerable.GetEnumerator() => ((IEnumerable<T>)this).GetEnumerator();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int Position(int theIndex)
    {
        int aRelative = theIndex - myLowerBound;
        if ((uint)aRelative >= (uint)mySize)
        {
            throw new Standard_OutOfRange("NCollection_Array1::Value");
        }

        return myOffset + aRelative;
    }

    private int CheckAt(int theIndex)
        => (uint)theIndex < (uint)mySize ? theIndex : throw new Standard_OutOfRange("NCollection_Array1::At");
}

/// <summary>
/// OCCT's handle-able array (<c>occ::handle&lt;NCollection_HArray1&lt;T&gt;&gt;</c>). Arrays are classes here
/// already, so this only adds OCCT's accessors.
/// </summary>
public class NCollection_HArray1<T> : NCollection_Array1<T>
{
    public NCollection_HArray1() { }

    public NCollection_HArray1(int theLower, int theUpper) : base(theLower, theUpper) { }

    public NCollection_HArray1(int theLower, int theUpper, T theValue) : base(theLower, theUpper) => Init(theValue);

    public NCollection_HArray1(NCollection_Array1<T> theOther) : base(theOther) { }

    public NCollection_Array1<T> Array1() => this;

    public NCollection_Array1<T> ChangeArray1() => this;
}
