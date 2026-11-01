// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_Sequence.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_HSequence.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_List.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_DynamicArray.hxx
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_LocalArray.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections;
using System.Runtime.InteropServices;

namespace Occt;

/// <summary>
/// OCCT's sequence: a list indexed from 1. Not OCCT's linked implementation but a .NET list with OCCT's API;
/// allocator arguments are dropped. A class: copy with <c>new(other)</c> or <c>Assign</c>.
/// </summary>
public class NCollection_Sequence<T> : IEnumerable<T>
{
    private readonly List<T> myItems;

    public NCollection_Sequence() => myItems = [];

    public NCollection_Sequence(NCollection_Sequence<T> theOther) => myItems = [.. theOther.myItems];

    public static int Lower() => 1;

    public int Upper() => myItems.Count;

    public int Size() => myItems.Count;

    public int Length() => myItems.Count;

    public bool IsEmpty() => myItems.Count == 0;

    public void Clear() => myItems.Clear();

    public NCollection_Sequence<T> Assign(NCollection_Sequence<T> theOther)
    {
        if (!ReferenceEquals(this, theOther))
        {
            myItems.Clear();
            myItems.AddRange(theOther.myItems);
        }

        return this;
    }

    public void Reverse() => myItems.Reverse();

    public void Exchange(int I, int J)
    {
        Check(I);
        Check(J);
        (myItems[I - 1], myItems[J - 1]) = (myItems[J - 1], myItems[I - 1]);
    }

    public void Append(T theItem) => myItems.Add(theItem);

    /// <summary>Moves the items of <paramref name="theSeq"/> to the end of this sequence, leaving it empty.</summary>
    public void Append(NCollection_Sequence<T> theSeq)
    {
        myItems.AddRange(theSeq.myItems);
        theSeq.myItems.Clear();
    }

    public void Prepend(T theItem) => myItems.Insert(0, theItem);

    public void Prepend(NCollection_Sequence<T> theSeq)
    {
        myItems.InsertRange(0, theSeq.myItems);
        theSeq.myItems.Clear();
    }

    public void InsertBefore(int theIndex, T theItem)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > myItems.Count + 1, "NCollection_Sequence::InsertBefore");
        myItems.Insert(theIndex - 1, theItem);
    }

    public void InsertBefore(int theIndex, NCollection_Sequence<T> theSeq)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > myItems.Count + 1, "NCollection_Sequence::InsertBefore");
        myItems.InsertRange(theIndex - 1, theSeq.myItems);
        theSeq.myItems.Clear();
    }

    public void InsertAfter(int theIndex, T theItem)
    {
        Standard_OutOfRange_Raise_if(theIndex < 0 || theIndex > myItems.Count, "NCollection_Sequence::InsertAfter");
        myItems.Insert(theIndex, theItem);
    }

    public void InsertAfter(int theIndex, NCollection_Sequence<T> theSeq)
    {
        Standard_OutOfRange_Raise_if(theIndex < 0 || theIndex > myItems.Count, "NCollection_Sequence::InsertAfter");
        myItems.InsertRange(theIndex, theSeq.myItems);
        theSeq.myItems.Clear();
    }

    public void Remove(int theIndex)
    {
        Check(theIndex);
        myItems.RemoveAt(theIndex - 1);
    }

    public void Remove(int theFromIndex, int theToIndex)
    {
        Standard_OutOfRange_Raise_if(theFromIndex < 1 || theToIndex > myItems.Count || theFromIndex > theToIndex,
                                     "NCollection_Sequence::Remove");
        myItems.RemoveRange(theFromIndex - 1, theToIndex - theFromIndex + 1);
    }

    /// <summary>Moves the items from <paramref name="theIndex"/> on into <paramref name="theSeq"/>, replacing its content.</summary>
    public void Split(int theIndex, NCollection_Sequence<T> theSeq)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > myItems.Count + 1, "NCollection_Sequence::Split");
        theSeq.myItems.Clear();
        theSeq.myItems.AddRange(myItems.GetRange(theIndex - 1, myItems.Count - theIndex + 1));
        myItems.RemoveRange(theIndex - 1, myItems.Count - theIndex + 1);
    }

    public T First() => Value(1);

    public ref T ChangeFirst() => ref ChangeValue(1);

    public T Last() => Value(myItems.Count);

    public ref T ChangeLast() => ref ChangeValue(myItems.Count);

    public T Value(int theIndex)
    {
        Check(theIndex);
        return myItems[theIndex - 1];
    }

    public ref T ChangeValue(int theIndex)
    {
        Check(theIndex);
        return ref CollectionsMarshal.AsSpan(myItems)[theIndex - 1];
    }

    /// <summary>OCCT's <c>operator()</c>, for reading and writing.</summary>
    public ref T this[int theIndex] => ref ChangeValue(theIndex);

    public void SetValue(int theIndex, T theItem) => ChangeValue(theIndex) = theItem;

    public List<T>.Enumerator GetEnumerator() => myItems.GetEnumerator();

    IEnumerator<T> IEnumerable<T>.GetEnumerator() => myItems.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => myItems.GetEnumerator();

    private void Check(int theIndex)
        => Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > myItems.Count, "NCollection_Sequence::Value");
}

/// <summary>OCCT's handle-able sequence.</summary>
public class NCollection_HSequence<T> : NCollection_Sequence<T>
{
    public NCollection_HSequence() { }

    public NCollection_HSequence(NCollection_Sequence<T> theOther) : base(theOther) { }

    public NCollection_Sequence<T> Sequence() => this;

    public NCollection_Sequence<T> ChangeSequence() => this;
}

/// <summary>
/// OCCT's singly linked list, with its <see cref="Iterator"/> (More/Next/Value). A .NET list underneath;
/// allocator arguments are dropped. A class: copy with <c>new(other)</c> or <c>Assign</c>.
/// </summary>
public class NCollection_List<T> : IEnumerable<T>
{
    private readonly List<T> myItems;

    public NCollection_List() => myItems = [];

    public NCollection_List(NCollection_List<T> theOther) => myItems = [.. theOther.myItems];

    public NCollection_List(IEnumerable<T> theItems) => myItems = [.. theItems];

    public int Extent() => myItems.Count;

    public int Size() => myItems.Count;

    public bool IsEmpty() => myItems.Count == 0;

    public void Clear() => myItems.Clear();

    public NCollection_List<T> Assign(NCollection_List<T> theOther)
    {
        if (!ReferenceEquals(this, theOther))
        {
            myItems.Clear();
            myItems.AddRange(theOther.myItems);
        }

        return this;
    }

    public T First() => myItems.Count > 0 ? myItems[0] : throw new Standard_NoSuchObject("NCollection_List::First");

    public ref T ChangeFirst() => ref ItemAt(0, "NCollection_List::First");

    public T Last() => myItems.Count > 0 ? myItems[^1] : throw new Standard_NoSuchObject("NCollection_List::Last");

    public ref T ChangeLast() => ref ItemAt(myItems.Count - 1, "NCollection_List::Last");

    public ref T Append(T theItem)
    {
        myItems.Add(theItem);
        return ref CollectionsMarshal.AsSpan(myItems)[^1];
    }

    /// <summary>Appends, and sets <paramref name="theIter"/> to the new item.</summary>
    public void Append(T theItem, Iterator theIter)
    {
        myItems.Add(theItem);
        theIter.Reset(this, myItems.Count - 1);
    }

    /// <summary>Moves the items of <paramref name="theOther"/> to the end of this list, leaving it empty.</summary>
    public void Append(NCollection_List<T> theOther)
    {
        if (ReferenceEquals(this, theOther))
        {
            return;
        }

        myItems.AddRange(theOther.myItems);
        theOther.myItems.Clear();
    }

    public ref T Prepend(T theItem)
    {
        myItems.Insert(0, theItem);
        return ref CollectionsMarshal.AsSpan(myItems)[0];
    }

    public void Prepend(NCollection_List<T> theOther)
    {
        if (ReferenceEquals(this, theOther))
        {
            return;
        }

        myItems.InsertRange(0, theOther.myItems);
        theOther.myItems.Clear();
    }

    public void RemoveFirst()
    {
        if (myItems.Count > 0)
        {
            myItems.RemoveAt(0);
        }
    }

    /// <summary>Removes the iterator's item; the iterator moves to the next one.</summary>
    public void Remove(Iterator theIter)
    {
        myItems.RemoveAt(theIter.Index);
    }

    /// <summary>Removes the first item equal to <paramref name="theObject"/>.</summary>
    public bool Remove(T theObject) => myItems.Remove(theObject);

    public ref T InsertBefore(T theItem, Iterator theIter)
    {
        myItems.Insert(theIter.Index, theItem);
        int anIndex = theIter.Index;
        theIter.Reset(this, anIndex + 1);
        return ref CollectionsMarshal.AsSpan(myItems)[anIndex];
    }

    public ref T InsertAfter(T theItem, Iterator theIter)
    {
        myItems.Insert(theIter.Index + 1, theItem);
        return ref CollectionsMarshal.AsSpan(myItems)[theIter.Index + 1];
    }

    public void Reverse() => myItems.Reverse();

    public void Exchange(NCollection_List<T> theOther)
    {
        var aTemp = new List<T>(myItems);
        myItems.Clear();
        myItems.AddRange(theOther.myItems);
        theOther.myItems.Clear();
        theOther.myItems.AddRange(aTemp);
    }

    public bool Contains(T theObject) => myItems.Contains(theObject);

    public List<T>.Enumerator GetEnumerator() => myItems.GetEnumerator();

    IEnumerator<T> IEnumerable<T>.GetEnumerator() => myItems.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => myItems.GetEnumerator();

    private ref T ItemAt(int theIndex, string theWhere)
    {
        if (theIndex < 0 || theIndex >= myItems.Count)
        {
            throw new Standard_NoSuchObject(theWhere);
        }

        return ref CollectionsMarshal.AsSpan(myItems)[theIndex];
    }

    /// <summary>OCCT's list iterator: <c>for (It it(list); it.More(); it.Next()) it.Value();</c></summary>
    public sealed class Iterator
    {
        private NCollection_List<T>? myList;

        public Iterator() { }

        public Iterator(NCollection_List<T> theList) => Initialize(theList);

        /// <summary>OCCT's copy: an iterator on the same list at the same item.</summary>
        public Iterator(Iterator theOther)
        {
            myList = theOther.myList;
            Index = theOther.Index;
        }

        internal int Index { get; private set; }

        public void Initialize(NCollection_List<T> theList) => Reset(theList, 0);

        internal void Reset(NCollection_List<T> theList, int theIndex)
        {
            myList = theList;
            Index = theIndex;
        }

        public bool More() => myList is not null && Index < myList.myItems.Count;

        public void Next() => Index++;

        public T Value() => myList!.myItems[Index];

        public ref T ChangeValue() => ref CollectionsMarshal.AsSpan(myList!.myItems)[Index];
    }
}

/// <summary>
/// OCCT's growable array indexed from 0 (<c>NCollection_DynamicArray</c>, also known as <c>NCollection_Vector</c>).
/// Setting an element beyond the end grows the array, as in OCCT.
/// </summary>
public class NCollection_DynamicArray<T> : IEnumerable<T>
{
    private readonly List<T> myItems;

    /// <summary>The increment is an allocation hint in OCCT and is ignored.</summary>
    public NCollection_DynamicArray(int theIncrement = 256) => myItems = new List<T>(Math.Max(0, Math.Min(theIncrement, 256)));

    public NCollection_DynamicArray(NCollection_DynamicArray<T> theOther) => myItems = [.. theOther.myItems];

    public int Size() => myItems.Count;

    public int Length() => myItems.Count;

    public int Lower() => 0;

    public int Upper() => myItems.Count - 1;

    public bool IsEmpty() => myItems.Count == 0;

    public void Clear() => myItems.Clear();

    public NCollection_DynamicArray<T> Assign(NCollection_DynamicArray<T> theOther)
    {
        if (!ReferenceEquals(this, theOther))
        {
            myItems.Clear();
            myItems.AddRange(theOther.myItems);
        }

        return this;
    }

    public ref T Append(T theValue)
    {
        myItems.Add(theValue);
        return ref CollectionsMarshal.AsSpan(myItems)[^1];
    }

    /// <summary>Appends a default-constructed element and returns it for writing.</summary>
    public ref T Appended() => ref Append(NCollection_Default<T>.Value);

    public ref T InsertBefore(int theIndex, T theValue)
    {
        myItems.Insert(theIndex, theValue);
        return ref CollectionsMarshal.AsSpan(myItems)[theIndex];
    }

    public ref T InsertAfter(int theIndex, T theValue) => ref InsertBefore(theIndex + 1, theValue);

    public void EraseLast()
    {
        if (myItems.Count > 0)
        {
            myItems.RemoveAt(myItems.Count - 1);
        }
    }

    public T Value(int theIndex) => myItems[theIndex];

    public ref T ChangeValue(int theIndex) => ref CollectionsMarshal.AsSpan(myItems)[theIndex];

    public ref T this[int theIndex] => ref ChangeValue(theIndex);

    public T First() => myItems[0];

    public ref T ChangeFirst() => ref ChangeValue(0);

    public T Last() => myItems[^1];

    public ref T ChangeLast() => ref ChangeValue(myItems.Count - 1);

    /// <summary>Sets an element, first growing the array with default elements if the index is beyond the end.</summary>
    public ref T SetValue(int theIndex, T theValue)
    {
        while (myItems.Count <= theIndex)
        {
            myItems.Add(NCollection_Default<T>.Value);
        }

        myItems[theIndex] = theValue;
        return ref ChangeValue(theIndex);
    }

    /// <summary>The elements, without copying. Valid until the array next grows.</summary>
    public Span<T> AsSpan() => CollectionsMarshal.AsSpan(myItems);

    public List<T>.Enumerator GetEnumerator() => myItems.GetEnumerator();

    IEnumerator<T> IEnumerable<T>.GetEnumerator() => myItems.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => myItems.GetEnumerator();
}

/// <summary>OCCT's NCollection_Vector, which OCCT 8 makes the same container as NCollection_DynamicArray.</summary>
public class NCollection_Vector<T> : NCollection_DynamicArray<T>
{
    public NCollection_Vector(int theIncrement = 256) : base(theIncrement) { }

    public NCollection_Vector(NCollection_Vector<T> theOther) : base(theOther) { }
}

/// <summary>
/// OCCT's scratch array, which lives on the stack when small. Here a .NET array, indexed from 0. Ported code that
/// only needs a short-lived buffer can use <c>stackalloc</c> and a span instead.
/// </summary>
public sealed class NCollection_LocalArray<T>
{
    private T[] myData;

    public NCollection_LocalArray() => myData = [];

    public NCollection_LocalArray(int theSize) => myData = new T[theSize];

    public void Allocate(int theSize) => myData = new T[theSize];

    public void Reallocate(int theNewSize, bool theToCopy = true)
    {
        if (theToCopy)
        {
            Array.Resize(ref myData, theNewSize);
        }
        else
        {
            myData = new T[theNewSize];
        }
    }

    public int Size() => myData.Length;

    public ref T this[int theIndex] => ref myData[theIndex];

    public Span<T> AsSpan() => myData;

    public NCollection_Array1<T> ToArray1() => NCollection_Array1<T>.From(myData, 0);
}
