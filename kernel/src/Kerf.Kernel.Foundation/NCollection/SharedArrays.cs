// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.InteropServices;

namespace Kerf.Engine.Foundation;

/// <summary>
/// Small arrays that frozen objects can share, as most B-splines have the same knots and multiplicities: a STEP import
/// of 2,086 B-spline surfaces held 24,873 arrays of knots with 459 different contents, 38% of all it kept
/// (performance.md, "The sixth slice").
/// </summary>
/// <remarks>
/// A frozen object's array never changes, so one array can serve any number of them, on any thread, and it reads the same
/// as the array it replaces: it's compared bit for bit, with its bounds. Only an object being frozen gives or takes one.
/// The table is one for the process, a slot for each hash holding the last array seen there, so what it can keep alive is
/// fixed: 1,024 arrays of up to 32 values a type, about 300 kB of doubles at most. A slot is one reference, read and
/// written whole, so threads racing on it can only lose an entry; the array was complete before it was written there.
/// </remarks>
internal static class SharedArrays<T> where T : unmanaged
{
    private const int LongestShared = 32;
    private const int Slots = 1024;

    private static readonly NCollection_Array1<T>?[] s_slots = new NCollection_Array1<T>?[Slots];

    /// <summary>The array to hold in place of <paramref name="array"/>, which belongs to an object being frozen: itself, or an equal one already shared.</summary>
    /// <param name="array">The array.</param>
    /// <param name="isFixed">
    /// That its storage can't change even if it's a view of another array's, as with the shared unit weights. Any other
    /// view isn't this array's to give away: it can take a shared one, but not offer itself.
    /// </param>
    public static NCollection_Array1<T> Shared(NCollection_Array1<T> array, bool isFixed = false)
    {
        int length = array.Length();
        if (length == 0 || length > LongestShared)
        {
            return array;
        }

        ReadOnlySpan<T> values = array.AsSpan();
        ref NCollection_Array1<T>? slot = ref s_slots[HashOf(values, array.Lower()) & (Slots - 1)];
        NCollection_Array1<T>? known = Volatile.Read(ref slot);
        if (known is not null && !ReferenceEquals(known, array) && known.Lower() == array.Lower()
            && MemoryMarshal.AsBytes(known.AsSpan()).SequenceEqual(MemoryMarshal.AsBytes(values)))
        {
            return known;
        }

        if (isFixed || array.IsDeletable())
        {
            Volatile.Write(ref slot, array);
        }

        return array;
    }

    private static int HashOf(ReadOnlySpan<T> values, int lower)
    {
        var hash = new HashCode();
        hash.Add(lower);
        hash.AddBytes(MemoryMarshal.AsBytes(values));
        return hash.ToHashCode();
    }
}
