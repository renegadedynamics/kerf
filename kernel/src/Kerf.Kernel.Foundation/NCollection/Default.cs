// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;

namespace Kerf.Engine.Collections;

/// <summary>
/// The value C++ would default-construct for <typeparamref name="T"/>. For structs with a parameterless constructor
/// (gp_Dir, gp_Trsf, gp_Ax2, ...) that is not all zeros, so new arrays are filled with it, as OCCT's containers
/// construct their elements.
/// </summary>
internal static class Default<T>
{
    private static readonly T? S_value = Create();
    private static readonly bool S_isZero = !typeof(T).IsValueType || EqualityComparer<T>.Default.Equals(S_value, default);

    public static T Value => S_value!;

    public static T[] NewArray(int size)
    {
        var array = size == 0 ? [] : new T[size];
        if (!S_isZero)
        {
            Array.Fill(array, S_value!);
        }

        return array;
    }

    // The value types with parameterless constructors are Kerf's own, which construct them directly elsewhere,
    // so trimming keeps the constructors. Recorded in docs/decisions.md.
    [UnconditionalSuppressMessage("Trimming", "IL2091", Justification = "Kerf's value types keep their constructors")]
    private static T? Create() => typeof(T).IsValueType ? Activator.CreateInstance<T>() : default;
}
