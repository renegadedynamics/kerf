// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Collections;

/// <summary>What OCCT's growable arrays had and <see cref="List{T}"/> lacks.</summary>
internal static class ListExtensions
{
    /// <summary>
    /// Sets the element at the index, first growing the list with default elements if the index is beyond its end
    /// (OCCT's <c>NCollection_DynamicArray::SetValue</c>). New elements are default-constructed as in C++
    /// (<see cref="Default{T}"/>).
    /// </summary>
    public static void SetAt<T>(this List<T> list, int index, T value)
    {
        while (list.Count <= index)
        {
            list.Add(Default<T>.Value);
        }

        list[index] = value;
    }
}
