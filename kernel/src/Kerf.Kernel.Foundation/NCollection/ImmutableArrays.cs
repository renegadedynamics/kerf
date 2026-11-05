// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using System.Runtime.InteropServices;

namespace Kerf.Engine;

/// <summary>Makes the <see cref="ImmutableArray{T}"/>s that the kernel returns as <see cref="IReadOnlyList{T}"/>s.</summary>
internal static class ImmutableArrays
{
    /// <summary>An array holding a copy of the values.</summary>
    internal static ImmutableArray<T> Copy<T>(params ReadOnlySpan<T> values) => ImmutableArray.Create(values);

    /// <summary>
    /// An array that takes ownership of <paramref name="values"/> without copying. The caller must not change the
    /// array afterwards. For ported code that has just filled an array.
    /// </summary>
    internal static ImmutableArray<T> Own<T>(T[] values) => ImmutableCollectionsMarshal.AsImmutableArray(values);
}
