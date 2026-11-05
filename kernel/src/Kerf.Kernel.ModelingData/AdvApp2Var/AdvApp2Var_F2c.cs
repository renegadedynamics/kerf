// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Kerf.Engine.Approximation.Advanced;

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
    private readonly Span<T> _span;
    private readonly int _bias;

    public F2cPtr(Span<T> span)
    {
        _span = span;
        _bias = 0;
    }

    private F2cPtr(Span<T> span, int bias)
    {
        _span = span;
        _bias = bias;
    }

    /// <summary><c>p[i]</c>.</summary>
    public ref T this[int index] => ref _span[_bias + index];

    /// <summary>True for OCCT's <c>nullptr</c> (a default pointer over no memory).</summary>
    public bool IsNull => _span.IsEmpty;

    /// <summary>The elements from <c>p[0]</c> to the end of the underlying span.</summary>
    public Span<T> AsSpan() => _span.Slice(_bias);

    /// <summary>The <paramref name="length"/> elements from <c>p[0]</c>.</summary>
    public Span<T> AsSpan(int length) => _span.Slice(_bias, length);

    /// <summary>The start of the underlying span (its element 0, whatever the bias).</summary>
    internal ref T UnderlyingReference => ref MemoryMarshal.GetReference(_span);

    public static implicit operator global::Kerf.Engine.Approximation.Advanced.F2cPtr<T>(Span<T> span) => new(span);

    public static implicit operator global::Kerf.Engine.Approximation.Advanced.F2cPtr<T>(T[] array) => new(array);

    public static implicit operator global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T>(global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> ptr) => new(ptr._span, ptr._bias);

    public static global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> operator +(global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> ptr, int shift) => new(ptr._span, ptr._bias + shift);

    public static global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> operator -(global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> ptr, int shift) => new(ptr._span, ptr._bias - shift);

    public static global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> operator ++(global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> ptr) => new(ptr._span, ptr._bias + 1);

    public static global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> operator --(global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> ptr) => new(ptr._span, ptr._bias - 1);
}

/// <summary>A read-only f2c array pointer (an input array, or one of AdvApp2Var's constant tables).</summary>
internal readonly ref struct F2cConstPtr<T>
{
    private readonly ReadOnlySpan<T> _span;
    private readonly int _bias;

    public F2cConstPtr(ReadOnlySpan<T> span)
    {
        _span = span;
        _bias = 0;
    }

    internal F2cConstPtr(ReadOnlySpan<T> span, int bias)
    {
        _span = span;
        _bias = bias;
    }

    /// <summary><c>p[i]</c>.</summary>
    public ref readonly T this[int index] => ref _span[_bias + index];

    /// <summary>True for OCCT's <c>nullptr</c>.</summary>
    public bool IsNull => _span.IsEmpty;

    /// <summary>The elements from <c>p[0]</c> to the end of the underlying span.</summary>
    public ReadOnlySpan<T> AsSpan() => _span.Slice(_bias);

    /// <summary>The <paramref name="length"/> elements from <c>p[0]</c>.</summary>
    public ReadOnlySpan<T> AsSpan(int length) => _span.Slice(_bias, length);

    public static implicit operator global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T>(ReadOnlySpan<T> span) => new(span);

    public static implicit operator global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T>(Span<T> span) => new(span);

    public static implicit operator global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T>(T[] array) => new(array);

    public static global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T> operator +(global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T> ptr, int shift) =>
        new(ptr._span, ptr._bias + shift);

    public static global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T> operator -(global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T> ptr, int shift) =>
        new(ptr._span, ptr._bias - shift);

    public static global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T> operator ++(global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T> ptr) => new(ptr._span, ptr._bias + 1);

    public static global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T> operator --(global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T> ptr) => new(ptr._span, ptr._bias - 1);
}

/// <summary>Helpers for f2c idioms that C# has no syntax for.</summary>
internal static class F2c
{
    /// <summary><c>&amp;x</c> passed where a function takes an array: a pointer to the single variable.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static global::Kerf.Engine.Approximation.Advanced.F2cPtr<T> Ref<T>(ref T variable) => new(MemoryMarshal.CreateSpan(ref variable, 1));

    /// <summary><c>&amp;x</c> passed where a function takes a read-only array.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static global::Kerf.Engine.Approximation.Advanced.F2cConstPtr<T> In<T>(ref readonly T variable) =>
        new(MemoryMarshal.CreateReadOnlySpan(in variable, 1));
}
