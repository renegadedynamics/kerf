// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;

namespace Kerf.Engine.Fonts;

/// <summary>
/// The names a font's <c>name</c> table gives: family, subfamily (style), full name, PostScript name and the typographic
/// family and subfamily (IDs 1, 2, 4, 6, 16 and 17). Windows' Unicode records (platform 3, US English first, then any
/// language) before Unicode's (platform 0) and the Macintosh's Roman ones (platform 1, read as Latin-1 for their ASCII).
/// </summary>
internal sealed class FontNames
{
    public string Family { get; private init; } = "";

    public string Subfamily { get; private init; } = "";

    public string FullName { get; private init; } = "";

    public string PostScriptName { get; private init; } = "";

    public string TypographicFamily { get; private init; } = "";

    public string TypographicSubfamily { get; private init; } = "";

    public static FontNames Read(FontData? name)
    {
        if (name is not { } table)
        {
            return new FontNames();
        }

        int count = table.U16(2);
        int storage = table.U16(4);
        table.CheckArray(6, count, 12, "the name records");
        var found = new Dictionary<int, (int Rank, string Text)>();
        for (int i = 0; i < count; i++)
        {
            int at = 6 + 12 * i;
            int platform = table.U16(at), encoding = table.U16(at + 2), language = table.U16(at + 4), id = table.U16(at + 6);
            if (id is not (1 or 2 or 4 or 6 or 16 or 17))
            {
                continue;
            }

            int rank = (platform, encoding) switch
            {
                (3, 1 or 10) => language == 0x0409 ? 0 : 1,
                (3, 0) => language == 0x0409 ? 2 : 3,
                (0, _) => 4,
                (1, 0) => language == 0 ? 5 : 6,
                _ => int.MaxValue,
            };
            if (rank == int.MaxValue || (found.TryGetValue(id, out var have) && have.Rank <= rank))
            {
                continue;
            }

            int length = table.U16(at + 8), offset = table.U16(at + 10);
            FontData text = table.Slice((long)storage + offset, length, "a name");
            string value = platform == 1
                ? Encoding.Latin1.GetString(text.Span)
                : Encoding.BigEndianUnicode.GetString(text.Span[..(text.Length & ~1)]);
            value = value.Replace("\0", "", StringComparison.Ordinal).Trim();
            if (value.Length > 0)
            {
                found[id] = (rank, value);
            }
        }

        string Get(int id) => found.TryGetValue(id, out var v) ? v.Text : "";
        return new FontNames
        {
            Family = Get(1),
            Subfamily = Get(2),
            FullName = Get(4),
            PostScriptName = Get(6),
            TypographicFamily = Get(16),
            TypographicSubfamily = Get(17),
        };
    }
}
