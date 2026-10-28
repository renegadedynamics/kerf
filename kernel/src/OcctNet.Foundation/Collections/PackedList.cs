// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections;
using Bento.Collections;

namespace Occt;

/// <summary>Creates <see cref="PackedList{T}"/>s.</summary>
public static class PackedList
{
    /// <summary>A list holding a copy of the values.</summary>
    public static PackedList<T> Create<T>(params ReadOnlySpan<T> values) => values.IsEmpty ? PackedList<T>.Empty : new(values.ToArray());

    /// <summary>
    /// A list that takes ownership of <paramref name="values"/> without copying. The caller must not change the
    /// array afterwards. For ported code that has just filled an array.
    /// </summary>
    internal static PackedList<T> Own<T>(T[] values) => values.Length == 0 ? PackedList<T>.Empty : new(values);
}

/// <summary>
/// An immutable list held in one contiguous array, for the numeric results of geometry: poles, knots, weights,
/// mesh nodes. Reading it as a span (<see cref="AsSpan"/>) costs nothing, which Bento's tree-shaped lists can't
/// offer. Edits return ordinary Bento lists, which make further edits cheap.
/// </summary>
/// <typeparam name="T">The type of the elements, usually a gp struct or a number.</typeparam>
public sealed class PackedList<T> : IBentoList<T>
{
    private readonly T[] myItems;
    private readonly int myOffset;
    private readonly int myCount;

    /// <summary>The empty list.</summary>
    public static PackedList<T> Empty { get; } = new([]);

    internal PackedList(T[] theItems) : this(theItems, 0, theItems.Length) { }

    private PackedList(T[] theItems, int theOffset, int theCount)
    {
        myItems = theItems;
        myOffset = theOffset;
        myCount = theCount;
    }

    /// <summary>The elements, without copying.</summary>
    public ReadOnlySpan<T> AsSpan() => new(myItems, myOffset, myCount);

    /// <summary>The elements, without copying.</summary>
    public ReadOnlyMemory<T> AsMemory() => new(myItems, myOffset, myCount);

    public int Count => myCount;

    public bool IsEmpty => myCount == 0;

    /// <summary>1 for a non-empty list (a single leaf, in Bento's terms), 0 for an empty one.</summary>
    public int Depth => myCount == 0 ? 0 : 1;

    public T this[int index]
    {
        get
        {
            if ((uint)index >= (uint)myCount)
            {
                throw new ArgumentOutOfRangeException(nameof(index));
            }

            return myItems[myOffset + index];
        }
    }

    public T Peek => myCount > 0 ? myItems[myOffset + myCount - 1] : throw new InvalidOperationException("The list is empty.");

    public T PeekFront => myCount > 0 ? myItems[myOffset] : throw new InvalidOperationException("The list is empty.");

    /// <summary>Shares this list's array: O(1).</summary>
    public IBentoList<T> Subset(int start, int count)
    {
        if (start < 0 || count < 0 || start > myCount - count)
        {
            throw new ArgumentOutOfRangeException(nameof(count));
        }

        if (start == 0 && count == myCount)
        {
            return this;
        }

        return count == 0 ? Empty : new PackedList<T>(myItems, myOffset + start, count);
    }

    public IBentoList<T> Reverse()
    {
        if (myCount < 2)
        {
            return this;
        }

        T[] aReversed = AsSpan().ToArray();
        Array.Reverse(aReversed);
        return new PackedList<T>(aReversed);
    }

    /// <summary>Already as compact as a list can be.</summary>
    public IBentoList<T> Compact() => this;

    // Edits hand over to Bento's own lists, which are built for them

    public IBentoList<T> SetItem(int index, T value) => ToBentoList().SetItem(index, value);

    public IBentoList<T> Push(T value) => ToBentoList().Push(value);

    public IBentoList<T> Pop() => myCount > 0 ? Subset(0, myCount - 1) : throw new InvalidOperationException("The list is empty.");

    public IBentoList<T> PushFront(T value) => ToBentoList().PushFront(value);

    public IBentoList<T> PopFront() => myCount > 0 ? Subset(1, myCount - 1) : throw new InvalidOperationException("The list is empty.");

    public IBentoList<T> Insert(int index, T item) => ToBentoList().Insert(index, item);

    public IBentoList<T> RemoveAt(int index) => ToBentoList().RemoveAt(index);

    private IBentoList<T> ToBentoList() => BentoList.Create(AsSpan());

    public Enumerator GetEnumerator() => new(this);

    IEnumerator<T> IEnumerable<T>.GetEnumerator() => GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    /// <summary>Enumerates the elements without allocating.</summary>
    public struct Enumerator : IEnumerator<T>
    {
        private readonly PackedList<T> myList;
        private int myIndex;

        internal Enumerator(PackedList<T> theList)
        {
            myList = theList;
            myIndex = -1;
        }

        public readonly T Current => myList.myItems[myList.myOffset + myIndex];

        readonly object? IEnumerator.Current => Current;

        public bool MoveNext() => ++myIndex < myList.myCount;

        public void Reset() => myIndex = -1;

        public readonly void Dispose() { }
    }
}
