// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;

namespace Kerf.Engine.Fonts;

/// <summary>
/// A font's pair kerning, in font units: GPOS's pair adjustments (lookup type 2, formats 1 and 2, also inside extension
/// lookups) of the lookups its <c>kern</c> features name, else the <c>kern</c> table's format 0 pairs, which is all
/// FreeType's <c>FT_Get_Kerning</c> reads. GPOS is preferred where a font has both, as text shapers prefer it: it is the
/// newer table, and fonts that carry both (Liberation Sans) give the same pairs in each, with more in GPOS. Only the first
/// glyph's advance is adjusted (the value record's <c>XAdvance</c>). Every structure is checked when the font is read.
/// </summary>
internal sealed class PairKerning
{
    public static readonly PairKerning None = new([], [], null);

    // The kern table's pairs, sorted by (left << 16 | right), with the value of each subtable added (or overriding)
    private readonly uint[] _keys;
    private readonly short[] _values;

    // GPOS: each lookup's pair-adjustment subtables
    private readonly PairLookup[]? _lookups;

    private PairKerning(uint[] keys, short[] values, PairLookup[]? lookups)
    {
        _keys = keys;
        _values = values;
        _lookups = lookups;
    }

    public bool HasPairs => _keys.Length > 0 || _lookups is { Length: > 0 };

    public static PairKerning Read(FontData? gpos, FontData? kern)
    {
        if (gpos is { } g)
        {
            PairLookup[] lookups = ReadGpos(g);
            if (lookups.Length > 0)
            {
                return new PairKerning([], [], lookups);
            }
        }

        return kern is { } k ? ReadKern(k) : None;
    }

    /// <summary>The adjustment to <paramref name="left"/>'s advance when <paramref name="right"/> follows it.</summary>
    public int Value(int left, int right)
    {
        if (_lookups is { } lookups)
        {
            int sum = 0;
            foreach (PairLookup lookup in lookups)
            {
                foreach (PairSubtable sub in lookup.Subtables)
                {
                    if (sub.TryGet(left, right, out int value))
                    {
                        sum += value;
                        break;
                    }
                }
            }

            return sum;
        }

        int at = Array.BinarySearch(_keys, (uint)left << 16 | (uint)right);
        return at >= 0 ? _values[at] : 0;
    }

    private static PairKerning ReadKern(FontData kern)
    {
        var pairs = new Dictionary<uint, int>();
        int version = kern.U16(0);
        int tables, at;
        bool apple = version == 1;
        if (apple)
        {
            tables = (int)Math.Min(kern.U32(4), int.MaxValue);
            at = 8;
        }
        else
        {
            tables = kern.U16(2);
            at = 4;
        }

        for (int t = 0; t < tables && at < kern.Length; t++)
        {
            int length, format, coverage, header;
            if (apple)
            {
                length = kern.Offset32(at, "a kern subtable");
                coverage = kern.U16(at + 4);
                format = coverage & 0xFF;
                header = 8;
                // Vertical, cross-stream or variation subtables aren't horizontal kerning
                if ((coverage & 0xE000) != 0)
                {
                    format = -1;
                }
            }
            else
            {
                length = kern.U16(at + 2);
                coverage = kern.U16(at + 4);
                format = coverage >> 8;
                header = 6;
                if ((coverage & 0x0005) != 0x0001) // horizontal, not cross-stream
                {
                    format = -1;
                }
            }

            if (format == 0)
            {
                bool overrides = !apple && (coverage & 0x0008) != 0;
                int n = kern.U16(at + header);
                kern.CheckArray(at + header + 8, n, 6, "the kern pairs");
                for (int i = 0; i < n; i++)
                {
                    int p = at + header + 8 + 6 * i;
                    uint key = (uint)kern.U16(p) << 16 | kern.U16(p + 2);
                    int value = kern.I16(p + 4);
                    pairs[key] = overrides || !pairs.TryGetValue(key, out int sum) ? value : sum + value;
                }
            }

            // A version 0 subtable's 16-bit length is often wrong for a large format 0 subtable (FreeType reads the pairs'
            // count instead); the last subtable ends the table either way
            if (!apple && format == 0)
            {
                at += header + 8 + 6 * kern.U16(at + header);
            }
            else
            {
                if (length <= 0)
                {
                    break;
                }

                at += length;
            }
        }

        uint[] keys = [.. pairs.Keys];
        Array.Sort(keys);
        var values = new short[keys.Length];
        for (int i = 0; i < keys.Length; i++)
        {
            values[i] = (short)Math.Clamp(pairs[keys[i]], short.MinValue, short.MaxValue);
        }

        return new PairKerning(keys, values, null);
    }

    private static PairLookup[] ReadGpos(FontData gpos)
    {
        int features = gpos.U16(6), lookups = gpos.U16(8);
        FontData featureList = gpos.From(features, "GPOS's feature list");
        FontData lookupList = gpos.From(lookups, "GPOS's lookup list");
        int featureCount = featureList.U16(0);
        featureList.CheckArray(2, featureCount, 6, "GPOS's features");
        int lookupCount = lookupList.U16(0);
        lookupList.CheckArray(2, lookupCount, 2, "GPOS's lookups");

        var indices = new SortedSet<int>();
        for (int i = 0; i < featureCount; i++)
        {
            if (featureList.U32(2 + 6 * i) != SfntFile.Tag("kern"))
            {
                continue;
            }

            FontData feature = featureList.From(featureList.U16(6 + 6 * i), "a GPOS feature");
            int n = feature.U16(2);
            feature.CheckArray(4, n, 2, "a GPOS feature's lookups");
            for (int j = 0; j < n; j++)
            {
                int index = feature.U16(4 + 2 * j);
                if (index >= lookupCount)
                {
                    throw FontData.Malformed($"a GPOS feature names lookup {index} of {lookupCount}");
                }

                indices.Add(index);
            }
        }

        var result = new List<PairLookup>();
        foreach (int index in indices)
        {
            FontData lookup = lookupList.From(lookupList.U16(2 + 2 * index), "a GPOS lookup");
            int type = lookup.U16(0);
            int count = lookup.U16(4);
            lookup.CheckArray(6, count, 2, "a GPOS lookup's subtables");
            var subtables = new List<PairSubtable>();
            for (int s = 0; s < count; s++)
            {
                FontData sub = lookup.From(lookup.U16(6 + 2 * s), "a GPOS subtable");
                int subType = type;
                if (type == 9)
                {
                    subType = sub.U16(2);
                    if (subType == 9)
                    {
                        throw FontData.Malformed("a GPOS extension lookup points to another extension");
                    }

                    sub = sub.From(sub.Offset32(4, "a GPOS extension"), "a GPOS extension's subtable");
                }

                if (subType == 2)
                {
                    subtables.Add(PairSubtable.Read(sub));
                }
            }

            if (subtables.Count > 0)
            {
                result.Add(new PairLookup([.. subtables]));
            }
        }

        return [.. result];
    }

    private sealed record PairLookup(PairSubtable[] Subtables);

    /// <summary>One PairPos subtable, its structure checked when read.</summary>
    private sealed class PairSubtable
    {
        private readonly FontData _data;
        private readonly int _format;
        private readonly Coverage _coverage;
        private readonly int _size1, _size2, _xAdvance1;
        private readonly ClassDefinition? _class1, _class2;
        private readonly int _class2Count;
        private readonly int _class1Count;

        private PairSubtable(FontData data, int format, Coverage coverage, int valueFormat1, int valueFormat2,
            ClassDefinition? class1, ClassDefinition? class2, int class1Count, int class2Count)
        {
            _class1Count = class1Count;
            _data = data;
            _format = format;
            _coverage = coverage;
            _size1 = 2 * BitOperations.PopCount((uint)valueFormat1 & 0xFF);
            _size2 = 2 * BitOperations.PopCount((uint)valueFormat2 & 0xFF);
            // XAdvance's place in the first value record: after XPlacement and YPlacement where they are present
            _xAdvance1 = (valueFormat1 & 0x0004) == 0 ? -1 : 2 * BitOperations.PopCount((uint)valueFormat1 & 0x0003);
            _class1 = class1;
            _class2 = class2;
            _class2Count = class2Count;
        }

        public static PairSubtable Read(FontData sub)
        {
            int format = sub.U16(0);
            var coverage = Coverage.Read(sub.From(sub.U16(2), "a GPOS coverage"));
            int vf1 = sub.U16(4), vf2 = sub.U16(6);
            int size = 2 * (BitOperations.PopCount((uint)vf1 & 0xFF) + BitOperations.PopCount((uint)vf2 & 0xFF));
            if (format == 1)
            {
                int sets = sub.U16(8);
                sub.CheckArray(10, sets, 2, "GPOS's pair sets");
                for (int i = 0; i < sets; i++)
                {
                    FontData set = sub.From(sub.U16(10 + 2 * i), "a GPOS pair set");
                    set.CheckArray(2, set.U16(0), 2 + size, "a GPOS pair set's records");
                }

                return new PairSubtable(sub, 1, coverage, vf1, vf2, null, null, 0, 0);
            }

            if (format == 2)
            {
                var class1 = ClassDefinition.Read(sub.From(sub.U16(8), "a GPOS class definition"));
                var class2 = ClassDefinition.Read(sub.From(sub.U16(10), "a GPOS class definition"));
                int count1 = sub.U16(12), count2 = sub.U16(14);
                sub.CheckArray(16, (long)count1 * count2, size, "a GPOS class pair table");
                // A class past the table (HarfBuzz reads it as no adjustment) is checked at each lookup
                return new PairSubtable(sub, 2, coverage, vf1, vf2, class1, class2, count1, count2);
            }

            throw FontData.Malformed($"a GPOS pair subtable has format {format}");
        }

        public bool TryGet(int left, int right, out int value)
        {
            value = 0;
            int index = _coverage.IndexOf(left);
            if (index < 0)
            {
                return false;
            }

            int record = 2 + _size1 + _size2;
            if (_format == 1)
            {
                if (index >= _data.U16(8))
                {
                    return false;
                }

                FontData set = _data.From(_data.U16(10 + 2 * index), "a GPOS pair set");
                int lo = 0, hi = set.U16(0) - 1;
                while (lo <= hi)
                {
                    int mid = (lo + hi) >>> 1;
                    int second = set.U16(2 + record * mid);
                    if (second < right)
                    {
                        lo = mid + 1;
                    }
                    else if (second > right)
                    {
                        hi = mid - 1;
                    }
                    else
                    {
                        value = _xAdvance1 < 0 ? 0 : set.I16(2 + record * mid + 2 + _xAdvance1);
                        return true;
                    }
                }

                return false;
            }

            int c1 = _class1!.ClassOf(left), c2 = _class2!.ClassOf(right);
            if (c1 >= _class1Count || c2 >= _class2Count)
            {
                return true;
            }

            int at = 16 + (c1 * _class2Count + c2) * (_size1 + _size2);
            value = _xAdvance1 < 0 ? 0 : _data.I16(at + _xAdvance1);
            return true;
        }
    }

    /// <summary>An OpenType coverage table: the glyphs a subtable applies to, and each one's index.</summary>
    private sealed class Coverage
    {
        private readonly FontData _data;
        private readonly int _format, _count;

        private Coverage(FontData data, int format, int count)
        {
            _data = data;
            _format = format;
            _count = count;
        }

        /// <summary>The number of glyphs covered.</summary>
        public int Count { get; private init; }

        public static Coverage Read(FontData data)
        {
            int format = data.U16(0), n = data.U16(2);
            switch (format)
            {
                case 1:
                    data.CheckArray(4, n, 2, "a coverage table");
                    return new Coverage(data, 1, n) { Count = n };
                case 2:
                {
                    data.CheckArray(4, n, 6, "a coverage table's ranges");
                    int total = 0;
                    for (int i = 0; i < n; i++)
                    {
                        int start = data.U16(4 + 6 * i), end = data.U16(6 + 6 * i);
                        total = Math.Max(total, data.U16(8 + 6 * i) + Math.Max(0, end - start + 1));
                    }

                    return new Coverage(data, 2, n) { Count = total };
                }
                default:
                    throw FontData.Malformed($"a coverage table has format {format}");
            }
        }

        public int IndexOf(int glyph)
        {
            int lo = 0, hi = _count - 1;
            while (lo <= hi)
            {
                int mid = (lo + hi) >>> 1;
                if (_format == 1)
                {
                    int g = _data.U16(4 + 2 * mid);
                    if (g == glyph)
                    {
                        return mid;
                    }

                    (lo, hi) = g < glyph ? (mid + 1, hi) : (lo, mid - 1);
                }
                else
                {
                    int at = 4 + 6 * mid;
                    int start = _data.U16(at), end = _data.U16(at + 2);
                    if (glyph < start)
                    {
                        hi = mid - 1;
                    }
                    else if (glyph > end)
                    {
                        lo = mid + 1;
                    }
                    else
                    {
                        return _data.U16(at + 4) + glyph - start;
                    }
                }
            }

            return -1;
        }
    }

    /// <summary>An OpenType class definition: each glyph's class, 0 for any not listed.</summary>
    private sealed class ClassDefinition
    {
        private readonly FontData _data;
        private readonly int _format, _count, _start;

        private ClassDefinition(FontData data, int format, int count, int start, int maxClass)
        {
            _data = data;
            _format = format;
            _count = count;
            _start = start;
            MaxClass = maxClass;
        }

        public int MaxClass { get; }

        public static ClassDefinition Read(FontData data)
        {
            int format = data.U16(0);
            int max = 0;
            if (format == 1)
            {
                int start = data.U16(2), n = data.U16(4);
                data.CheckArray(6, n, 2, "a class definition");
                for (int i = 0; i < n; i++)
                {
                    max = Math.Max(max, data.U16(6 + 2 * i));
                }

                return new ClassDefinition(data, 1, n, start, max);
            }

            if (format == 2)
            {
                int n = data.U16(2);
                data.CheckArray(4, n, 6, "a class definition's ranges");
                for (int i = 0; i < n; i++)
                {
                    max = Math.Max(max, data.U16(8 + 6 * i));
                }

                return new ClassDefinition(data, 2, n, 0, max);
            }

            throw FontData.Malformed($"a class definition has format {format}");
        }

        public int ClassOf(int glyph)
        {
            if (_format == 1)
            {
                return glyph >= _start && glyph - _start < _count ? _data.U16(6 + 2 * (glyph - _start)) : 0;
            }

            int lo = 0, hi = _count - 1;
            while (lo <= hi)
            {
                int mid = (lo + hi) >>> 1;
                int at = 4 + 6 * mid;
                if (glyph < _data.U16(at))
                {
                    hi = mid - 1;
                }
                else if (glyph > _data.U16(at + 2))
                {
                    lo = mid + 1;
                }
                else
                {
                    return _data.U16(at + 4);
                }
            }

            return 0;
        }
    }
}
