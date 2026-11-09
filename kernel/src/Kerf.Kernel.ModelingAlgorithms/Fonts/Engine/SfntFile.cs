// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Fonts;

namespace Kerf.Engine.Fonts;

/// <summary>
/// The table directory of one face of an OpenType file (the "sfnt" structure TrueType and OpenType share), or of each face
/// of a collection (<c>.ttc</c>): the tables by tag, each a range checked to lie inside the file.
/// </summary>
internal sealed class SfntFile
{
    public const uint TagTrueType = 0x00010000;
    public const uint TagOtto = 0x4F54544F;      // 'OTTO': CFF outlines
    public const uint TagTrue = 0x74727565;      // 'true': Apple's TrueType
    public const uint TagCollection = 0x74746366; // 'ttcf'

    // At most this many tables in one face: the format's 16-bit count allows 65,535, and real fonts have under 50
    private const int MaxTables = 4096;

    private readonly Dictionary<uint, FontData> _tables;

    private SfntFile(uint version, Dictionary<uint, FontData> tables)
    {
        Version = version;
        _tables = tables;
    }

    /// <summary>The face's sfnt version: <see cref="TagTrueType"/>, <see cref="TagTrue"/> or <see cref="TagOtto"/>.</summary>
    public uint Version { get; }

    public static uint Tag(string tag) => (uint)(tag[0] << 24 | tag[1] << 16 | tag[2] << 8 | tag[3]);

    public bool TryGetTable(string tag, out FontData table) => _tables.TryGetValue(Tag(tag), out table);

    public FontData Table(string tag) =>
        _tables.TryGetValue(Tag(tag), out FontData table) ? table : throw FontData.Malformed($"it has no '{tag.TrimEnd()}' table");

    /// <summary>The offsets of the faces' table directories: one for a single font, each of a collection's.</summary>
    public static int[] FaceOffsets(FontData file, FontReadLimits limits)
    {
        if (file.Length < 12)
        {
            throw FontData.Malformed($"it is {file.Length} bytes long, too short for a font");
        }

        uint tag = file.U32(0);
        if (tag == TagCollection)
        {
            uint count = file.U32(8);
            if (count == 0)
            {
                throw FontData.Malformed("its collection holds no fonts");
            }

            if (count > (uint)limits.MaxFaces)
            {
                throw new FontLimitException($"its collection holds {count} fonts, more than {nameof(FontReadLimits.MaxFaces)} ({limits.MaxFaces})");
            }

            file.CheckArray(12, count, 4, "the collection's offsets");
            var offsets = new int[count];
            for (int i = 0; i < offsets.Length; i++)
            {
                offsets[i] = file.Offset32(12 + 4 * i, "a collection's font");
            }

            return offsets;
        }

        CheckVersion(tag);
        return [0];
    }

    /// <summary>The face whose table directory is at <paramref name="offset"/>.</summary>
    public static SfntFile Read(FontData file, int offset)
    {
        uint version = file.U32(offset);
        CheckVersion(version);
        int count = file.U16(offset + 4);
        if (count > MaxTables)
        {
            throw new FontLimitException($"it has {count} tables, more than a font has ({MaxTables})");
        }

        file.CheckArray(offset + 12, count, 16, "the table directory");
        var tables = new Dictionary<uint, FontData>(count);
        for (int i = 0; i < count; i++)
        {
            int record = offset + 12 + 16 * i;
            uint tag = file.U32(record);
            int start = file.Offset32(record + 8, "a table");
            int length = file.Offset32(record + 12, "a table");
            // The first of a repeated tag counts, as FreeType reads it
            if (!tables.ContainsKey(tag))
            {
                tables[tag] = file.Slice(start, length, $"the '{TagName(tag)}' table");
            }
        }

        return new SfntFile(version, tables);
    }

    private static void CheckVersion(uint tag)
    {
        switch (tag)
        {
            case TagTrueType or TagTrue or TagOtto:
                return;
            case 0x774F4646: // 'wOFF'
            case 0x774F4632: // 'wOF2'
                throw new FontUnsupportedException("it is a WOFF web font, which isn't read: use its TrueType or OpenType file");
            case 0x74797031: // 'typ1'
                throw new FontUnsupportedException("it is a Type 1 font in an sfnt wrapper, which isn't read");
            default:
                throw new FontUnsupportedException($"it isn't a TrueType or OpenType font (its first four bytes are 0x{tag:X8})");
        }
    }

    public static string TagName(uint tag) =>
        string.Create(4, tag, static (s, t) =>
        {
            for (int i = 0; i < 4; i++)
            {
                char c = (char)((t >> (24 - 8 * i)) & 0xFF);
                s[i] = c is >= ' ' and <= '~' ? c : '?';
            }
        });
}
