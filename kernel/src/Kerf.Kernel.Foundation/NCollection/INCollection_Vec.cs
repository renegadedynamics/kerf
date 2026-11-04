// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;
using System.Numerics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Text;

namespace Occt;

/// <summary>
/// What NCollection_Vec2, NCollection_Vec3 and NCollection_Vec4 have in common, so that code OCCT templates over
/// the dimension (<c>BVH_Box&lt;T, N&gt;</c> and the rest of BVH) ports as generics over the vector type.
/// </summary>
/// <remarks>
/// Not in OCCT, where <c>BVH::VectorType&lt;T, N&gt;::Type</c> selects the vector type. Generic code writes a
/// component through <c>NCollection_VecTools.Change</c>, which is internal, so a vector stays immutable outside
/// OCCT.NET.
/// </remarks>
/// <typeparam name="T">The element type.</typeparam>
/// <typeparam name="TSelf">The vector type itself.</typeparam>
internal interface INCollection_Vec<T, TSelf> : IEquatable<TSelf>
    where T : unmanaged, INumber<T>
    where TSelf : unmanaged, INCollection_Vec<T, TSelf>
{
    /// <summary>The number of components.</summary>
    static abstract int Length();

    /// <summary>A vector with every component set to <paramref name="value"/>, as OCCT's one-value constructor.</summary>
    static abstract TSelf Create(T value);

    /// <summary>The component of the given (0-based) index.</summary>
    T this[int index] { get; }

    TSelf CwiseMin(in TSelf vec);

    TSelf CwiseMax(in TSelf vec);

    TSelf CwiseAbs();

    T MaxComp();

    T MinComp();

    T Dot(in TSelf other);

    T SquareModulus();

    static abstract TSelf operator +(in TSelf left, in TSelf right);

    static abstract TSelf operator -(in TSelf left, in TSelf right);

    static abstract TSelf operator -(in TSelf vec);

    static abstract TSelf operator *(in TSelf left, in TSelf right);

    static abstract TSelf operator *(in TSelf vec, T factor);

    static abstract TSelf operator /(in TSelf vec, T invFactor);

    static abstract TSelf operator /(in TSelf left, in TSelf right);
}

/// <summary>Helpers for the NCollection vector and matrix types.</summary>
internal static class NCollection_VecTools
{
    /// <summary>
    /// A reference to a component of a vector of any dimension, as OCCT's <c>ChangeData()[theIndex]</c>. The vector
    /// types are their components laid out contiguously, so this reinterprets the vector as its first component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static ref T Change<T, TVec>(ref TVec vec, int index)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        if ((uint)index >= (uint)TVec.Length())
        {
            throw new OutOfRangeException("NCollection_Vec: index out of range");
        }

        return ref Unsafe.Add(ref Unsafe.As<TVec, T>(ref vec), index);
    }

    /// <summary>The components of a vector of any dimension, for writing (OCCT's <c>ChangeData()</c>).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Span<T> ChangeData<T, TVec>(ref TVec vec)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
        => MemoryMarshal.CreateSpan(ref Unsafe.As<TVec, T>(ref vec), TVec.Length());

    /// <summary>True for <c>float</c> and <c>double</c>, for which OCCT specialises vector division by a scalar.</summary>
    internal static bool IsReal<T>() => typeof(T) == typeof(float) || typeof(T) == typeof(double);

    /// <summary><c>std::sqrt</c> on the element type (for an integer type, the square root truncated back).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static T Sqrt<T>(T value) where T : unmanaged, INumber<T>
    {
        if (typeof(T) == typeof(float))
        {
            float aValue = MathF.Sqrt((float)(object)value);
            return (T)(object)aValue;
        }

        return T.CreateTruncating(Math.Sqrt(double.CreateTruncating(value)));
    }

    internal static string Format<T>(string name, ReadOnlySpan<T> values) where T : INumber<T>
    {
        var builder = new StringBuilder(name).Append('(');
        for (int i = 0; i < values.Length; i++)
        {
            if (i > 0)
            {
                builder.Append(", ");
            }

            builder.Append(values[i].ToString(null, CultureInfo.InvariantCulture));
        }

        return builder.Append(')').ToString();
    }
}

[InlineArray(2)]
internal struct NCollection_Buffer2<T>
{
    private T _element0;
}

[InlineArray(3)]
internal struct NCollection_Buffer3<T>
{
    private T _element0;
}

[InlineArray(4)]
internal struct NCollection_Buffer4<T>
{
    private T _element0;
}

[InlineArray(9)]
internal struct NCollection_Buffer9<T>
{
    private T _element0;
}

[InlineArray(16)]
internal struct NCollection_Buffer16<T>
{
    private T _element0;
}
