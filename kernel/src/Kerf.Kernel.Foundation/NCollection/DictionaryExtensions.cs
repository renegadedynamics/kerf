// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Kerf.Engine.Collections;

/// <summary>
/// What OCCT's <c>NCollection_DataMap</c> had and <see cref="Dictionary{TKey, TValue}"/> lacks:
/// <list type="bullet">
/// <item>a lookup that fails as OCCT's does, with <see cref="NoSuchObjectException"/> (a
/// <see cref="FailureException"/>) rather than <see cref="KeyNotFoundException"/>: the algorithms catch OCCT's
/// failures, and the recorded results name the exception;</item>
/// <item>access by reference to the values (<c>ChangeFind</c>, <c>Bound</c>, <c>TryBound</c>): the ported code writes
/// into struct values in place, or keeps a reference to the stored value. The reference is valid until the dictionary
/// is next changed.</item>
/// </list>
/// </summary>
internal static class DictionaryExtensions
{
    /// <summary>The key's value (OCCT's <c>Find</c>); throws <see cref="NoSuchObjectException"/> if the key isn't there.</summary>
    public static TValue GetValue<TKey, TValue>(this Dictionary<TKey, TValue> dictionary, TKey key) where TKey : notnull
        => dictionary.TryGetValue(key, out TValue? value) ? value : throw new NoSuchObjectException("NCollection_DataMap::Find");

    /// <summary>The key's value, for writing (OCCT's <c>ChangeFind</c>); throws if the key isn't there.</summary>
    public static ref TValue GetValueRef<TKey, TValue>(this Dictionary<TKey, TValue> dictionary, TKey key) where TKey : notnull
    {
        ref TValue value = ref CollectionsMarshal.GetValueRefOrNullRef(dictionary, key);
        if (Unsafe.IsNullRef(ref value))
        {
            throw new NoSuchObjectException("NCollection_DataMap::ChangeFind");
        }

        return ref value;
    }

    /// <summary>Sets the key's value, adding the key if new, and returns the value for writing (OCCT's <c>Bound</c>).</summary>
    public static ref TValue SetValueRef<TKey, TValue>(this Dictionary<TKey, TValue> dictionary, TKey key, TValue value) where TKey : notnull
    {
        ref TValue? stored = ref CollectionsMarshal.GetValueRefOrAddDefault(dictionary, key, out _);
        stored = value;
        return ref Unsafe.As<TValue?, TValue>(ref stored);
    }

    /// <summary>
    /// Adds the key with the value if the key isn't there, and returns the key's value (the existing one or the new
    /// one) for writing (OCCT's <c>TryBound</c>).
    /// </summary>
    public static ref TValue GetOrAddValueRef<TKey, TValue>(this Dictionary<TKey, TValue> dictionary, TKey key, TValue value) where TKey : notnull
    {
        ref TValue? stored = ref CollectionsMarshal.GetValueRefOrAddDefault(dictionary, key, out bool exists);
        if (!exists)
        {
            stored = value;
        }

        return ref Unsafe.As<TValue?, TValue>(ref stored);
    }
}
