// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
public interface INCollection_Vec<T, TSelf> : IEquatable<TSelf>
    where T : unmanaged, INumber<T>
    where TSelf : unmanaged, INCollection_Vec<T, TSelf>
{
    /// <summary>The number of components.</summary>
    static abstract int Length();

    /// <summary>A vector with every component set to <paramref name="theValue"/>, as OCCT's one-value constructor.</summary>
    static abstract TSelf Create(T theValue);

    /// <summary>The component of the given (0-based) index.</summary>
    T this[int theIndex] { get; }

    TSelf cwiseMin(in TSelf theVec);

    TSelf cwiseMax(in TSelf theVec);

    TSelf cwiseAbs();

    T maxComp();

    T minComp();

    T Dot(in TSelf theOther);

    T SquareModulus();

    static abstract TSelf operator +(in TSelf theLeft, in TSelf theRight);

    static abstract TSelf operator -(in TSelf theLeft, in TSelf theRight);

    static abstract TSelf operator -(in TSelf theVec);

    static abstract TSelf operator *(in TSelf theLeft, in TSelf theRight);

    static abstract TSelf operator *(in TSelf theVec, T theFactor);

    static abstract TSelf operator /(in TSelf theVec, T theInvFactor);

    static abstract TSelf operator /(in TSelf theLeft, in TSelf theRight);
}

/// <summary>Helpers for the NCollection vector and matrix types.</summary>
internal static class NCollection_VecTools
{
    /// <summary>
    /// A reference to a component of a vector of any dimension, as OCCT's <c>ChangeData()[theIndex]</c>. The vector
    /// types are their components laid out contiguously, so this reinterprets the vector as its first component.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static ref T Change<T, TVec>(ref TVec theVec, int theIndex)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
    {
        if ((uint)theIndex >= (uint)TVec.Length())
        {
            throw new Standard_OutOfRange("NCollection_Vec: index out of range");
        }

        return ref Unsafe.Add(ref Unsafe.As<TVec, T>(ref theVec), theIndex);
    }

    /// <summary>The components of a vector of any dimension, for writing (OCCT's <c>ChangeData()</c>).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Span<T> ChangeData<T, TVec>(ref TVec theVec)
        where T : unmanaged, INumber<T>
        where TVec : unmanaged, INCollection_Vec<T, TVec>
        => MemoryMarshal.CreateSpan(ref Unsafe.As<TVec, T>(ref theVec), TVec.Length());

    /// <summary>True for <c>float</c> and <c>double</c>, for which OCCT specialises vector division by a scalar.</summary>
    internal static bool IsReal<T>() => typeof(T) == typeof(float) || typeof(T) == typeof(double);

    /// <summary><c>std::sqrt</c> on the element type (for an integer type, the square root truncated back).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static T Sqrt<T>(T theValue) where T : unmanaged, INumber<T>
    {
        if (typeof(T) == typeof(float))
        {
            float aValue = MathF.Sqrt((float)(object)theValue);
            return (T)(object)aValue;
        }

        return T.CreateTruncating(Math.Sqrt(double.CreateTruncating(theValue)));
    }

    internal static string Format<T>(string theName, ReadOnlySpan<T> theValues) where T : INumber<T>
    {
        var aBuilder = new StringBuilder(theName).Append('(');
        for (int i = 0; i < theValues.Length; i++)
        {
            if (i > 0)
            {
                aBuilder.Append(", ");
            }

            aBuilder.Append(theValues[i].ToString(null, CultureInfo.InvariantCulture));
        }

        return aBuilder.Append(')').ToString();
    }
}

[InlineArray(2)]
internal struct NCollection_Buffer2<T>
{
    private T myElement0;
}

[InlineArray(3)]
internal struct NCollection_Buffer3<T>
{
    private T myElement0;
}

[InlineArray(4)]
internal struct NCollection_Buffer4<T>
{
    private T myElement0;
}

[InlineArray(9)]
internal struct NCollection_Buffer9<T>
{
    private T myElement0;
}

[InlineArray(16)]
internal struct NCollection_Buffer16<T>
{
    private T myElement0;
}
