// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Fonts;

namespace Kerf.Engine.Fonts;

/// <summary>
/// The names and style of each font in a file, read from its table directories and its <c>name</c>, <c>OS/2</c> and
/// <c>head</c> tables alone, for a library to list fonts without reading their outlines (OCCT's <c>Font_FontMgr</c> asks
/// FreeType for each face's family and style the same way).
/// </summary>
internal static class FontHeaders
{
    public readonly record struct Face(int Index, string Family, string StyleName, string TypographicFamily, string TypographicStyleName, FontStyle Style);

    // A face's few tables: anything larger isn't a font's name table
    private const int MaxTableBytes = 4 << 20;

    public static List<Face> Read(Stream file, FontReadLimits limits)
    {
        var faces = new List<Face>();
        FontData head = Chunk(file, 0, 12);
        int[] offsets;
        if (head.U32(0) == SfntFile.TagCollection)
        {
            uint count = head.U32(8);
            if (count == 0 || count > (uint)limits.MaxFaces)
            {
                return faces;
            }

            FontData table = Chunk(file, 12, 4 * (int)count);
            offsets = new int[count];
            for (int i = 0; i < offsets.Length; i++)
            {
                offsets[i] = table.Offset32(4 * i, "a collection's font");
            }
        }
        else
        {
            offsets = [0];
        }

        for (int i = 0; i < offsets.Length; i++)
        {
            FontData directory = Chunk(file, offsets[i], 12);
            uint version = directory.U32(0);
            if (version is not (SfntFile.TagTrueType or SfntFile.TagTrue or SfntFile.TagOtto))
            {
                continue;
            }

            int tables = directory.U16(4);
            FontData records = Chunk(file, offsets[i] + 12L, 16 * tables);
            FontData? name = null, os2 = null, headTable = null;
            for (int t = 0; t < tables; t++)
            {
                uint tag = records.U32(16 * t);
                int at = records.Offset32(16 * t + 8, "a table");
                int length = records.Offset32(16 * t + 12, "a table");
                if (tag == SfntFile.Tag("name"))
                {
                    name = Chunk(file, at, Math.Min(length, MaxTableBytes));
                }
                else if (tag == SfntFile.Tag("OS/2"))
                {
                    os2 = Chunk(file, at, Math.Min(length, 128));
                }
                else if (tag == SfntFile.Tag("head"))
                {
                    headTable = Chunk(file, at, Math.Min(length, 54));
                }
            }

            FontNames names = FontNames.Read(name);
            if (names.Family.Length == 0)
            {
                continue;
            }

            bool bold, italic;
            if (os2 is { Length: >= 64 } o)
            {
                int selection = o.U16(62);
                (bold, italic) = ((selection & 0x20) != 0, (selection & 0x201) != 0);
            }
            else if (headTable is { Length: >= 46 } h)
            {
                int macStyle = h.U16(44);
                (bold, italic) = ((macStyle & 1) != 0, (macStyle & 2) != 0);
            }
            else
            {
                (bold, italic) = (false, false);
            }

            FontStyle style = (bold, italic) switch
            {
                (true, true) => FontStyle.BoldItalic,
                (true, false) => FontStyle.Bold,
                (false, true) => FontStyle.Italic,
                _ => FontStyle.Regular,
            };
            faces.Add(new Face(i, names.Family, names.Subfamily,
                names.TypographicFamily.Length > 0 ? names.TypographicFamily : names.Family,
                names.TypographicSubfamily.Length > 0 ? names.TypographicSubfamily : names.Subfamily, style));
        }

        return faces;
    }

    private static FontData Chunk(Stream file, long at, int length)
    {
        if (at < 0 || length < 0 || at > file.Length || length > file.Length - at)
        {
            throw FontData.Malformed("a table runs past the end of the file");
        }

        var bytes = new byte[length];
        file.Position = at;
        file.ReadExactly(bytes);
        return new FontData(bytes);
    }
}
