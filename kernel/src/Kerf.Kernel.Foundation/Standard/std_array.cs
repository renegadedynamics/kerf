// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Foundation.Std;

// C++'s std::array<T, N>, which OCCT 8's newer packages (MathUtils, MathSys, ...) use for small fixed-size
// values inside result structs. These are inline arrays: values, copied as a whole like std::array, indexed
// from 0 with a range check. `a[i][j]` works on a nested array held in a variable, as in C++.

/// <summary>C++'s <c>std::array&lt;T, 2&gt;</c>: two values held inline, indexed from 0.</summary>
[InlineArray(2)]
internal struct Array2<T>
{
    private T _element0;

    /// <summary>Builds the array from its values, as C++'s brace initialisation <c>{a, b}</c>.</summary>
    public static Array2<T> Of(T a, T b)
    {
        Array2<T> array = default;
        array[0] = a;
        array[1] = b;
        return array;
    }

    public readonly int Length => 2;
}

/// <summary>C++'s <c>std::array&lt;T, 3&gt;</c>: three values held inline, indexed from 0.</summary>
[InlineArray(3)]
internal struct Array3<T>
{
    private T _element0;

    /// <summary>Builds the array from its values, as C++'s brace initialisation <c>{a, b, c}</c>.</summary>
    public static Array3<T> Of(T a, T b, T c)
    {
        Array3<T> array = default;
        array[0] = a;
        array[1] = b;
        array[2] = c;
        return array;
    }

    public readonly int Length => 3;
}

/// <summary>C++'s <c>std::array&lt;T, 4&gt;</c>: four values held inline, indexed from 0.</summary>
[InlineArray(4)]
internal struct Array4<T>
{
    private T _element0;

    /// <summary>Builds the array from its values, as C++'s brace initialisation <c>{a, b, c, d}</c>.</summary>
    public static Array4<T> Of(T a, T b, T c, T d)
    {
        Array4<T> array = default;
        array[0] = a;
        array[1] = b;
        array[2] = c;
        array[3] = d;
        return array;
    }

    public readonly int Length => 4;
}
