// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

// C++'s std::array<T, N>, which OCCT 8's newer packages (MathUtils, MathSys, ...) use for small fixed-size
// values inside result structs. These are inline arrays: values, copied as a whole like std::array, indexed
// from 0 with a range check. `a[i][j]` works on a nested array held in a variable, as in C++.

/// <summary>C++'s <c>std::array&lt;T, 2&gt;</c>: two values held inline, indexed from 0.</summary>
[InlineArray(2)]
public struct std_array2<T>
{
    private T myElement0;

    /// <summary>Builds the array from its values, as C++'s brace initialisation <c>{a, b}</c>.</summary>
    public static std_array2<T> Of(T theA, T theB)
    {
        std_array2<T> anArray = default;
        anArray[0] = theA;
        anArray[1] = theB;
        return anArray;
    }

    public readonly int Length => 2;
}

/// <summary>C++'s <c>std::array&lt;T, 3&gt;</c>: three values held inline, indexed from 0.</summary>
[InlineArray(3)]
public struct std_array3<T>
{
    private T myElement0;

    /// <summary>Builds the array from its values, as C++'s brace initialisation <c>{a, b, c}</c>.</summary>
    public static std_array3<T> Of(T theA, T theB, T theC)
    {
        std_array3<T> anArray = default;
        anArray[0] = theA;
        anArray[1] = theB;
        anArray[2] = theC;
        return anArray;
    }

    public readonly int Length => 3;
}

/// <summary>C++'s <c>std::array&lt;T, 4&gt;</c>: four values held inline, indexed from 0.</summary>
[InlineArray(4)]
public struct std_array4<T>
{
    private T myElement0;

    /// <summary>Builds the array from its values, as C++'s brace initialisation <c>{a, b, c, d}</c>.</summary>
    public static std_array4<T> Of(T theA, T theB, T theC, T theD)
    {
        std_array4<T> anArray = default;
        anArray[0] = theA;
        anArray[1] = theB;
        anArray[2] = theC;
        anArray[3] = theD;
        return anArray;
    }

    public readonly int Length => 4;
}
