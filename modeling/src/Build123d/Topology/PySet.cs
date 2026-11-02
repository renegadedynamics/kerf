namespace Build123d;

/// <summary>
/// CPython's <c>list(set(points))</c> for vectors: the distinct points in the order a Python set iterates them, which
/// depends on their hashes. build123d picks results by that order in places (the plane of
/// <see cref="Shape1D.CommonPlane(IEnumerable{Shape1D?}, double)"/>), so the port reproduces it.
/// </summary>
internal static class PySet
{
    private const ulong Modulus = (1UL << 61) - 1;   // _PyHASH_MODULUS
    private const int LinearProbes = 9, PerturbShift = 5, MinSize = 8;

    /// <summary>Python's <c>hash(float)</c> (finite values).</summary>
    internal static long HashDouble(double v)
    {
        if (double.IsInfinity(v))
            return v > 0 ? 314159 : -314159;
        var m = Frexp(v, out var e);
        long sign = 1;
        if (m < 0)
        {
            sign = -1;
            m = -m;
        }
        ulong x = 0;
        while (m != 0)
        {
            x = ((x << 28) & Modulus) | x >> (61 - 28);
            m *= 268435456.0;   // 2**28
            e -= 28;
            var y = (ulong)m;
            m -= y;
            x += y;
            if (x >= Modulus)
                x -= Modulus;
        }
        e = e >= 0 ? e % 61 : 61 - 1 - ((-1 - e) % 61);
        x = ((x << e) & Modulus) | x >> (61 - e);
        var h = (long)x * sign;
        return h == -1 ? -2 : h;
    }

    /// <summary>C's frexp: <paramref name="v"/> as m·2^e with 0.5 ≤ |m| &lt; 1 (m = 0 for 0).</summary>
    private static double Frexp(double v, out int e)
    {
        if (v == 0)
        {
            e = 0;
            return v;
        }
        e = Math.ILogB(v) + 1;
        return Math.ScaleB(v, -e);
    }

    /// <summary>Python's <c>hash(tuple)</c> of three floats (the xxHash-based tuple hash of CPython 3.8 on).</summary>
    internal static long HashTuple(double a, double b, double c)
    {
        const ulong prime1 = 11400714785074694791UL, prime2 = 14029467366897019727UL, prime5 = 2870177450012600261UL;
        var acc = prime5;
        foreach (var item in (ReadOnlySpan<double>)[a, b, c])
        {
            acc += unchecked((ulong)HashDouble(item) * prime2);
            acc = (acc << 31) | (acc >> 33);
            acc = unchecked(acc * prime1);
        }
        acc += 3 ^ (prime5 ^ 3527539UL);
        return acc == ulong.MaxValue ? 1546275796 : (long)acc;
    }

    /// <summary>The distinct points (by <see cref="Vector"/> equality) in the order <c>list(set(points))</c> gives them.</summary>
    internal static List<Vector> Distinct(IEnumerable<Vector> points)
    {
        var table = new (Vector Key, long Hash, bool Used)[MinSize];
        int fill = 0;
        foreach (var point in points)
        {
            var key = point.Key;
            var hash = HashTuple(key.Item1, key.Item2, key.Item3);
            if (!Add(table, point, hash))
                continue;
            fill++;
            var mask = table.Length - 1;
            if (fill * 5 >= mask * 3)
                table = Resize(table, fill > 50000 ? fill * 2 : fill * 4);
        }
        return [.. table.Where(s => s.Used).Select(s => s.Key)];
    }

    /// <summary>set_add_entry: false if an equal point is already there.</summary>
    private static bool Add((Vector Key, long Hash, bool Used)[] table, Vector point, long hash)
    {
        var mask = (ulong)table.Length - 1;
        var perturb = (ulong)hash;
        var i = (ulong)hash & mask;
        while (true)
        {
            var probes = i + LinearProbes <= mask ? LinearProbes : 0;
            var j = i;
            do
            {
                if (!table[j].Used)
                {
                    table[j] = (point, hash, true);
                    return true;
                }
                if (table[j].Hash == hash && table[j].Key == point)
                    return false;
                j++;
            } while (probes-- > 0);
            perturb >>= PerturbShift;
            i = (i * 5 + 1 + perturb) & mask;
        }
    }

    /// <summary>set_table_resize: the entries reinserted, in table order, into a table larger than <paramref name="minUsed"/>.</summary>
    private static (Vector, long, bool)[] Resize((Vector Key, long Hash, bool Used)[] old, int minUsed)
    {
        var size = MinSize;
        while (size <= minUsed)
            size <<= 1;
        var table = new (Vector Key, long Hash, bool Used)[size];
        var mask = (ulong)size - 1;
        foreach (var entry in old)
        {
            if (!entry.Used)
                continue;
            // set_insert_clean
            var perturb = (ulong)entry.Hash;
            var i = (ulong)entry.Hash & mask;
            ulong? slot = null;
            while (slot is null)
            {
                if (!table[i].Used)
                    slot = i;
                else if (i + LinearProbes <= mask)
                    for (var j = i + 1; j <= i + LinearProbes && slot is null; j++)
                        if (!table[j].Used)
                            slot = j;
                perturb >>= PerturbShift;
                i = (i * 5 + 1 + perturb) & mask;
            }
            table[slot.Value] = entry;
        }
        return table;
    }
}
