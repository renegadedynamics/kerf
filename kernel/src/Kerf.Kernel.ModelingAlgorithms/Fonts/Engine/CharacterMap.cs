// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Fonts;

/// <summary>
/// The <c>cmap</c> table's Unicode map, chosen as FreeType's <c>FT_Select_Charmap</c> chooses it for Unicode: a full
/// Unicode subtable (platform 3 encoding 10, or platform 0 encodings 4 and 6) before a BMP one (3, 1, or platform 0), and
/// a symbol font's map (3, 0) last, where a character is also looked up at U+F000 above its code, as Windows does.
/// Formats 0, 4, 6, 12 and 13. Every lookup is a binary search over the table's own bytes, bounds-checked.
/// </summary>
internal sealed class CharacterMap
{
    private readonly FontData _table;
    private readonly int _format;
    private readonly bool _symbol;
    private readonly int _count;   // segments (format 4), groups (12, 13), entries (6)
    private readonly int _first;   // format 6's first code

    private CharacterMap(FontData table, int format, bool symbol, int count, int first)
    {
        _table = table;
        _format = format;
        _symbol = symbol;
        _count = count;
        _first = first;
    }

    public static CharacterMap Read(FontData cmap)
    {
        int count = cmap.U16(2);
        cmap.CheckArray(4, count, 8, "the cmap's encoding records");
        int best = -1, bestRank = int.MaxValue;
        bool bestSymbol = false;
        for (int i = 0; i < count; i++)
        {
            int platform = cmap.U16(4 + 8 * i);
            int encoding = cmap.U16(6 + 8 * i);
            int offset = cmap.Offset32(8 + 8 * i, "a cmap subtable");
            if (offset >= cmap.Length - 2)
            {
                continue;
            }

            int format = cmap.U16(offset);
            if (format is not (0 or 4 or 6 or 12 or 13))
            {
                continue;
            }

            int rank = (platform, encoding) switch
            {
                (3, 10) => 0,
                (0, 4) or (0, 6) => 1,
                (3, 1) => 2,
                (0, _) => 3,
                (3, 0) => 4,
                _ => int.MaxValue,
            };
            if (rank < bestRank)
            {
                (best, bestRank, bestSymbol) = (offset, rank, rank == 4);
            }
        }

        if (best < 0)
        {
            throw new FontUnsupportedException("it has no Unicode character map");
        }

        FontData table = cmap.From(best, "the cmap subtable");
        int f = table.U16(0);
        switch (f)
        {
            case 0:
                table.CheckArray(6, 256, 1, "the cmap's format 0 subtable");
                return new CharacterMap(table, 0, bestSymbol, 256, 0);
            case 4:
            {
                int segments = table.U16(6) / 2;
                table.CheckArray(14, segments, 8, "the cmap's format 4 segments");
                return new CharacterMap(table, 4, bestSymbol, segments, 0);
            }
            case 6:
            {
                int first = table.U16(6);
                int entries = table.U16(8);
                table.CheckArray(10, entries, 2, "the cmap's format 6 subtable");
                return new CharacterMap(table, 6, bestSymbol, entries, first);
            }
            default: // 12, 13
            {
                uint groups = table.U32(12);
                table.CheckArray(16, groups, 12, $"the cmap's format {f} groups");
                return new CharacterMap(table, f, bestSymbol, (int)groups, 0);
            }
        }
    }

    /// <summary>The glyph of <paramref name="codePoint"/>, or 0 where the map has none.</summary>
    public int GlyphOf(int codePoint)
    {
        int glyph = Lookup(codePoint);
        if (glyph == 0 && _symbol && codePoint is >= 0 and <= 0xFF)
        {
            glyph = Lookup(0xF000 | codePoint);
        }

        return glyph;
    }

    private int Lookup(int c)
    {
        if (c < 0)
        {
            return 0;
        }

        switch (_format)
        {
            case 0:
                return c < 256 ? _table.U8(6 + c) : 0;
            case 6:
                return c >= _first && c - _first < _count ? _table.U16(10 + 2 * (c - _first)) : 0;
            case 4:
                return Format4(c);
            default:
                return Groups(c);
        }
    }

    private int Format4(int c)
    {
        if (c > 0xFFFF)
        {
            return 0;
        }

        int segments = _count;
        int ends = 14, starts = 16 + 2 * segments, deltas = starts + 2 * segments, ranges = deltas + 2 * segments;
        int lo = 0, hi = segments - 1;
        while (lo < hi)
        {
            int mid = (lo + hi) >>> 1;
            if (_table.U16(ends + 2 * mid) < c)
            {
                lo = mid + 1;
            }
            else
            {
                hi = mid;
            }
        }

        if (segments == 0 || _table.U16(ends + 2 * lo) < c)
        {
            return 0;
        }

        int start = _table.U16(starts + 2 * lo);
        if (c < start)
        {
            return 0;
        }

        int delta = _table.U16(deltas + 2 * lo);
        int rangeAt = ranges + 2 * lo;
        int range = _table.U16(rangeAt);
        if (range == 0)
        {
            return (c + delta) & 0xFFFF;
        }

        int at = rangeAt + range + 2 * (c - start);
        if (at > _table.Length - 2)
        {
            return 0; // a range past the table, which FreeType also maps to nothing
        }

        int glyph = _table.U16(at);
        return glyph == 0 ? 0 : (glyph + delta) & 0xFFFF;
    }

    private int Groups(int c)
    {
        int lo = 0, hi = _count - 1;
        while (lo <= hi)
        {
            int mid = (lo + hi) >>> 1;
            int at = 16 + 12 * mid;
            uint start = _table.U32(at), end = _table.U32(at + 4);
            if ((uint)c < start)
            {
                hi = mid - 1;
            }
            else if ((uint)c > end)
            {
                lo = mid + 1;
            }
            else
            {
                uint first = _table.U32(at + 8);
                long glyph = _format == 12 ? first + ((uint)c - start) : first;
                return glyph is > 0 and <= 0xFFFF ? (int)glyph : 0;
            }
        }

        return 0;
    }
}
