// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Occt;

// AdvApp2Var's numeric core is f2c output: Fortran translated to C, where every array argument is a pointer that the
// function shifts before indexing it with Fortran's 1-based (or multi-dimensional) subscripts:
//
//     --point1;                         // point1[1] is now the first element
//     crvold -= crvold_offset;          // crvold[i + j * crvold_dim1], i and j from 1
//     f(&crvold[crvold_offset]);        // pass the array on from its first element
//
// F2cPtr and F2cConstPtr are those pointers over a span (decision 111). They keep a bias, so `--p`, `p -= offset`
// and `p + k` compile as written and every subscript is the C one, operation for operation. Out-of-range accesses
// throw (IndexOutOfRangeException) where C would read or write outside the array.

/// <summary>An f2c array pointer: a span and the index, in the span, of the element <c>p[0]</c> (decision 111).</summary>
/// <remarks>The bias may be negative or beyond the span (a shifted pointer); only the elements accessed must exist.</remarks>
internal readonly ref struct F2cPtr<T>
{
    private readonly Span<T> mySpan;
    private readonly int myBias;

    public F2cPtr(Span<T> theSpan)
    {
        mySpan = theSpan;
        myBias = 0;
    }

    private F2cPtr(Span<T> theSpan, int theBias)
    {
        mySpan = theSpan;
        myBias = theBias;
    }

    /// <summary><c>p[i]</c>.</summary>
    public ref T this[int theIndex] => ref mySpan[myBias + theIndex];

    /// <summary>True for OCCT's <c>nullptr</c> (a default pointer over no memory).</summary>
    public bool IsNull => mySpan.IsEmpty;

    /// <summary>The elements from <c>p[0]</c> to the end of the underlying span.</summary>
    public Span<T> AsSpan() => mySpan.Slice(myBias);

    /// <summary>The <paramref name="theLength"/> elements from <c>p[0]</c>.</summary>
    public Span<T> AsSpan(int theLength) => mySpan.Slice(myBias, theLength);

    /// <summary>The start of the underlying span (its element 0, whatever the bias).</summary>
    internal ref T UnderlyingReference => ref MemoryMarshal.GetReference(mySpan);

    public static implicit operator F2cPtr<T>(Span<T> theSpan) => new(theSpan);

    public static implicit operator F2cPtr<T>(T[] theArray) => new(theArray);

    public static implicit operator F2cConstPtr<T>(F2cPtr<T> thePtr) => new(thePtr.mySpan, thePtr.myBias);

    public static F2cPtr<T> operator +(F2cPtr<T> thePtr, int theShift) => new(thePtr.mySpan, thePtr.myBias + theShift);

    public static F2cPtr<T> operator -(F2cPtr<T> thePtr, int theShift) => new(thePtr.mySpan, thePtr.myBias - theShift);

    public static F2cPtr<T> operator ++(F2cPtr<T> thePtr) => new(thePtr.mySpan, thePtr.myBias + 1);

    public static F2cPtr<T> operator --(F2cPtr<T> thePtr) => new(thePtr.mySpan, thePtr.myBias - 1);
}

/// <summary>A read-only f2c array pointer (an input array, or one of AdvApp2Var's constant tables).</summary>
internal readonly ref struct F2cConstPtr<T>
{
    private readonly ReadOnlySpan<T> mySpan;
    private readonly int myBias;

    public F2cConstPtr(ReadOnlySpan<T> theSpan)
    {
        mySpan = theSpan;
        myBias = 0;
    }

    internal F2cConstPtr(ReadOnlySpan<T> theSpan, int theBias)
    {
        mySpan = theSpan;
        myBias = theBias;
    }

    /// <summary><c>p[i]</c>.</summary>
    public ref readonly T this[int theIndex] => ref mySpan[myBias + theIndex];

    /// <summary>True for OCCT's <c>nullptr</c>.</summary>
    public bool IsNull => mySpan.IsEmpty;

    /// <summary>The elements from <c>p[0]</c> to the end of the underlying span.</summary>
    public ReadOnlySpan<T> AsSpan() => mySpan.Slice(myBias);

    /// <summary>The <paramref name="theLength"/> elements from <c>p[0]</c>.</summary>
    public ReadOnlySpan<T> AsSpan(int theLength) => mySpan.Slice(myBias, theLength);

    public static implicit operator F2cConstPtr<T>(ReadOnlySpan<T> theSpan) => new(theSpan);

    public static implicit operator F2cConstPtr<T>(Span<T> theSpan) => new(theSpan);

    public static implicit operator F2cConstPtr<T>(T[] theArray) => new(theArray);

    public static F2cConstPtr<T> operator +(F2cConstPtr<T> thePtr, int theShift) =>
        new(thePtr.mySpan, thePtr.myBias + theShift);

    public static F2cConstPtr<T> operator -(F2cConstPtr<T> thePtr, int theShift) =>
        new(thePtr.mySpan, thePtr.myBias - theShift);

    public static F2cConstPtr<T> operator ++(F2cConstPtr<T> thePtr) => new(thePtr.mySpan, thePtr.myBias + 1);

    public static F2cConstPtr<T> operator --(F2cConstPtr<T> thePtr) => new(thePtr.mySpan, thePtr.myBias - 1);
}

/// <summary>Helpers for f2c idioms that C# has no syntax for.</summary>
internal static class F2c
{
    /// <summary><c>&amp;x</c> passed where a function takes an array: a pointer to the single variable.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static F2cPtr<T> Ref<T>(ref T theVariable) => new(MemoryMarshal.CreateSpan(ref theVariable, 1));

    /// <summary><c>&amp;x</c> passed where a function takes a read-only array.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static F2cConstPtr<T> In<T>(ref readonly T theVariable) =>
        new(MemoryMarshal.CreateReadOnlySpan(in theVariable, 1));
}
