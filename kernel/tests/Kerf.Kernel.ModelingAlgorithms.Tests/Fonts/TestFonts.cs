// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Buffers.Binary;
using System.Globalization;
using System.IO.Compression;
using Kerf.Fonts;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.Fonts;

/// <summary>
/// The fonts the tests read (Fonts/Data: three small fonts made by maketestfonts.py, and the shipped Liberation Sans) and
/// FreeType's outlines of them (ftdump.py: FreeType 2.13.2, unhinted, at 65,535 pixels to the em), with helpers to damage a
/// font's bytes.
/// </summary>
internal static class TestFonts
{
    public static byte[] Bytes(string name)
    {
        using Stream stream = typeof(TestFonts).Assembly.GetManifestResourceStream("Fonts." + name)
            ?? throw new InvalidOperationException($"No test resource {name}");
        var memory = new MemoryStream();
        if (name.EndsWith(".gz", StringComparison.Ordinal))
        {
            using var unzip = new GZipStream(stream, CompressionMode.Decompress);
            unzip.CopyTo(memory);
        }
        else
        {
            stream.CopyTo(memory);
        }

        return memory.ToArray();
    }

    public static Font Load(string name) => Font.Read(new MemoryStream(Bytes(name))).Value;

    public static string Text(string name) => System.Text.Encoding.UTF8.GetString(Bytes(name));

    /// <summary>FreeType's outlines from a dump: each glyph's contours of (kind, coordinates in ems).</summary>
    public static Dictionary<int, List<List<(char Kind, double[] Points)>>> Reference(string name)
    {
        var glyphs = new Dictionary<int, List<List<(char, double[])>>>();
        List<List<(char, double[])>>? current = null;
        foreach (string line in Text(name).Split(['\r', '\n'], StringSplitOptions.RemoveEmptyEntries))
        {
            string[] t = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            switch (t[0])
            {
                case "font":
                    break;
                case "glyph":
                    glyphs[int.Parse(t[1], CultureInfo.InvariantCulture)] = current = [];
                    break;
                case "contour":
                    current!.Add([]);
                    break;
                default:
                    current![^1].Add((t[0][0], t.Skip(1).Select(v => double.Parse(v, CultureInfo.InvariantCulture)).ToArray()));
                    break;
            }
        }

        return glyphs;
    }

    /// <summary>The offset and length of a table of a single font.</summary>
    public static (int Offset, int Length) Table(byte[] font, string tag)
    {
        int count = BinaryPrimitives.ReadUInt16BigEndian(font.AsSpan(4));
        for (int i = 0; i < count; i++)
        {
            int record = 12 + 16 * i;
            if (System.Text.Encoding.ASCII.GetString(font, record, 4) == tag)
            {
                return ((int)BinaryPrimitives.ReadUInt32BigEndian(font.AsSpan(record + 8)), (int)BinaryPrimitives.ReadUInt32BigEndian(font.AsSpan(record + 12)));
            }
        }

        throw new InvalidOperationException($"No table {tag}");
    }

    /// <summary>The offset of a glyph's data in a TrueType font's glyf table.</summary>
    public static int GlyphOffset(byte[] font, int glyph)
    {
        (int head, _) = Table(font, "head");
        bool longOffsets = BinaryPrimitives.ReadInt16BigEndian(font.AsSpan(head + 50)) == 1;
        (int loca, _) = Table(font, "loca");
        (int glyf, _) = Table(font, "glyf");
        int at = longOffsets
            ? (int)BinaryPrimitives.ReadUInt32BigEndian(font.AsSpan(loca + 4 * glyph))
            : 2 * BinaryPrimitives.ReadUInt16BigEndian(font.AsSpan(loca + 2 * glyph));
        return glyf + at;
    }

    public static void PutU16(byte[] font, int at, int value) => BinaryPrimitives.WriteUInt16BigEndian(font.AsSpan(at), (ushort)value);

    public static void PutU32(byte[] font, int at, uint value) => BinaryPrimitives.WriteUInt32BigEndian(font.AsSpan(at), value);

    /// <summary>A collection (<c>.ttc</c>) of whole single fonts, each one's tables copied after the directories.</summary>
    public static byte[] Collection(params byte[][] fonts)
    {
        var output = new MemoryStream();
        int header = 12 + 4 * fonts.Length;
        int[] directorySizes = [.. fonts.Select(f => 12 + 16 * BinaryPrimitives.ReadUInt16BigEndian(f.AsSpan(4)))];
        int tablesStart = header + directorySizes.Sum();
        var directories = new List<byte[]>();
        var tables = new MemoryStream();
        foreach (byte[] font in fonts)
        {
            int count = BinaryPrimitives.ReadUInt16BigEndian(font.AsSpan(4));
            byte[] directory = font[..(12 + 16 * count)];
            for (int i = 0; i < count; i++)
            {
                int record = 12 + 16 * i;
                int offset = (int)BinaryPrimitives.ReadUInt32BigEndian(font.AsSpan(record + 8));
                int length = (int)BinaryPrimitives.ReadUInt32BigEndian(font.AsSpan(record + 12));
                while (tables.Length % 4 != 0)
                {
                    tables.WriteByte(0);
                }

                BinaryPrimitives.WriteUInt32BigEndian(directory.AsSpan(record + 8), (uint)(tablesStart + tables.Length));
                tables.Write(font, offset, length);
            }

            directories.Add(directory);
        }

        byte[] head = new byte[header];
        "ttcf"u8.CopyTo(head);
        PutU32(head, 4, 0x00010000);
        PutU32(head, 8, (uint)fonts.Length);
        int at = header;
        for (int i = 0; i < fonts.Length; i++)
        {
            PutU32(head, 12 + 4 * i, (uint)at);
            at += directorySizes[i];
        }

        output.Write(head);
        foreach (byte[] directory in directories)
        {
            output.Write(directory);
        }

        tables.Position = 0;
        tables.CopyTo(output);
        return output.ToArray();
    }
}
