// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_LinearVector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// OCCT's contiguous growable array indexed from 0 (like <c>std::vector</c>), over a .NET array. Reimplemented, not
/// ported line for line (docs/decisions.md 15). It is a class: a C++ copy must be written <c>new(other)</c>.
/// </summary>
/// <remarks>
/// New elements are default-constructed as in C++ (<see cref="NCollection_Default{T}"/>). Indices are checked
/// against the size, where OCCT checks only in <c>Value</c>/<c>ChangeValue</c>. Left out: the move operations,
/// <c>EmplaceAppend</c> (use <see cref="Append"/>), <c>MaxSize</c> and the C++ iterators (use <see cref="AsSpan"/>).
/// </remarks>
public class NCollection_LinearVector<T>
{
    private T[] myData = [];
    private int mySize;

    public NCollection_LinearVector()
    {
    }

    /// <summary>An empty vector with the given capacity.</summary>
    public NCollection_LinearVector(int theCapacity) => Reserve(theCapacity);

    /// <summary>A vector of <paramref name="theSize"/> copies of <paramref name="theValue"/>.</summary>
    public NCollection_LinearVector(int theSize, T theValue) => Resize(theSize, theValue);

    /// <summary>A copy (C++'s copy constructor).</summary>
    public NCollection_LinearVector(NCollection_LinearVector<T> theOther)
    {
        mySize = theOther.mySize;
        myData = theOther.AsSpan().ToArray();
    }

    /// <summary>Copies the other vector's elements (C++'s copy assignment).</summary>
    public NCollection_LinearVector<T> Assign(NCollection_LinearVector<T> theOther)
    {
        if (!ReferenceEquals(this, theOther))
        {
            Reserve(theOther.mySize);
            theOther.AsSpan().CopyTo(myData);
            if (theOther.mySize < mySize)
            {
                Array.Clear(myData, theOther.mySize, mySize - theOther.mySize);
            }

            mySize = theOther.mySize;
        }

        return this;
    }

    public bool HasData() => myData.Length != 0;

    public bool Empty() => IsEmpty();

    public int Size() => mySize;

    public bool IsEmpty() => mySize == 0;

    public int Capacity() => myData.Length;

    public void Reserve(int theCapacity)
    {
        if (theCapacity > myData.Length)
        {
            Array.Resize(ref myData, theCapacity);
        }
    }

    /// <summary>Resizes, default-constructing new elements.</summary>
    public void Resize(int theSize) => Resize(theSize, NCollection_Default<T>.Value);

    public void Resize(int theSize, T theValue)
    {
        if (theSize > mySize)
        {
            if (theSize > myData.Length)
            {
                grow(theSize);
            }

            Array.Fill(myData, theValue, mySize, theSize - mySize);
        }
        else if (theSize < mySize)
        {
            Array.Clear(myData, theSize, mySize - theSize);
        }

        mySize = theSize;
    }

    public T Value(int theIndex) => myData[Check(theIndex)];

    public ref T ChangeValue(int theIndex) => ref myData[Check(theIndex)];

    /// <summary>The element at the index, for reading and writing (C++'s <c>operator[]</c> and <c>operator()</c>).</summary>
    public ref T this[int theIndex] => ref myData[Check(theIndex)];

    public T First() => Value(0);

    public ref T ChangeFirst() => ref ChangeValue(0);

    public T Last() => Value(mySize - 1);

    public ref T ChangeLast() => ref ChangeValue(mySize - 1);

    public ref T Append(T theValue)
    {
        if (mySize == myData.Length)
        {
            grow(mySize + 1);
        }

        myData[mySize] = theValue;
        return ref myData[mySize++];
    }

    /// <summary>Appends a default-constructed element and returns it for writing.</summary>
    public ref T Appended() => ref Append(NCollection_Default<T>.Value);

    /// <summary>Sets an element, first growing the vector with default elements if the index is beyond the end.</summary>
    public ref T SetValue(int theIndex, T theValue)
    {
        if (theIndex >= mySize)
        {
            Resize(theIndex + 1);
        }

        myData[theIndex] = theValue;
        return ref myData[theIndex];
    }

    public void InsertBefore(int theIndex, T theValue)
    {
        if (mySize == myData.Length)
        {
            grow(mySize + 1);
        }

        if (theIndex < mySize)
        {
            Array.Copy(myData, theIndex, myData, theIndex + 1, mySize - theIndex);
        }

        myData[theIndex] = theValue;
        ++mySize;
    }

    public void InsertAfter(int theIndex, T theValue) => InsertBefore(theIndex + 1, theValue);

    public void EraseLast()
    {
        if (mySize > 0)
        {
            myData[--mySize] = default!;
        }
    }

    public void Erase(int theIndex) => Erase(theIndex, theIndex);

    /// <summary>Removes the elements from <paramref name="theFrom"/> to <paramref name="theTo"/> inclusive.</summary>
    public void Erase(int theFrom, int theTo)
    {
        if (theTo < mySize)
        {
            int aCount = theTo - theFrom + 1;
            Array.Copy(myData, theTo + 1, myData, theFrom, mySize - theTo - 1);
            Array.Clear(myData, mySize - aCount, aCount);
            mySize -= aCount;
        }
    }

    /// <summary>Removes all elements, releasing the storage if asked.</summary>
    public void Clear(bool theReleaseMemory = false)
    {
        if (theReleaseMemory)
        {
            myData = [];
        }
        else
        {
            Array.Clear(myData, 0, mySize);
        }

        mySize = 0;
    }

    public NCollection_Array1<T> ToArray1()
    {
        var anArray = new NCollection_Array1<T>(0, mySize - 1);
        AsSpan().CopyTo(anArray.AsSpan());
        return anArray;
    }

    /// <summary>The elements, without copying (C++'s <c>Data()</c>, <c>begin()</c>, <c>end()</c>). Valid until the vector next grows.</summary>
    public Span<T> AsSpan() => new(myData, 0, mySize);

    private int Check(int theIndex)
    {
        if ((uint)theIndex >= (uint)mySize)
        {
            throw new Standard_OutOfRange("NCollection_LinearVector: index out of range");
        }

        return theIndex;
    }

    private void grow(int theMinCapacity)
    {
        int aNewCap = myData.Length > 0 ? myData.Length * 2 : 2;
        if (aNewCap < theMinCapacity)
        {
            aNewCap = theMinCapacity;
        }

        Array.Resize(ref myData, aNewCap);
    }
}
