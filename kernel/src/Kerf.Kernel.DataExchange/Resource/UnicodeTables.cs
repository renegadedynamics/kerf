// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.IO.Compression;
using System.Text;

namespace Kerf.Engine.Foundation.Resources;

/// <summary>
/// Kerf's stand-in for Resource_Unicode's code tables (Resource_CodePages.pxx, Resource_Shiftjis.pxx,
/// Resource_GB2312.pxx, Resource_GBK.pxx, Resource_Big5.pxx): each lookup asks .NET's encodings
/// (<see cref="CodePagesEncodingProvider"/>), then applies the corrections recorded where .NET differs from OCCT's
/// table (the embedded <c>Resource_UnicodeTables.diff.gz</c>, made by tools/stepdatarec's table dump; decision 1191).
/// </summary>
/// <remarks>Built once per process, on first use, and read-only after (thread-safe).</remarks>
internal static class UnicodeTables
{
    /// <summary>The tables, numbered as the corrections file numbers them.</summary>
    internal enum Table
    {
        CodePages = 0, // 19 pages of 128 entries (CP1250 ... CP850), index = page * 128 + (byte & 0x7F)
        Sjis = 1,      // sjisuni[65536], indexed by the SJIS code (lead << 8 | trail)
        Gb = 2,        // gbuni[65536], indexed by the GB2312 code without its high bits ((lead & 0x7F) << 8 | (trail & 0x7F))
        Gbk = 3,       // gbkuni[23940], indexed by the GBK pointer ((lead - 0x81) * 190 + trail - offset)
        Big5 = 4,      // big5uni[19782], indexed by the Big5 pointer ((lead - 0x81) * 157 + trail - offset)
    }

    private const int NbCodePages = (int)FormatType.FormatType_CP850 - (int)FormatType.FormatType_CP1250 + 1;

    private static readonly Lazy<Dictionary<int, uint>[]> Corrections = new(LoadCorrections);
    private static readonly Lazy<char[][]> S_codePages = new(BuildCodePages);
    private static readonly Lazy<uint[]> S_sjis = new(() => BuildTable(Table.Sjis, 65536, 932, SjisBytes));
    private static readonly Lazy<uint[]> S_gb = new(() => BuildTable(Table.Gb, 65536, 936, GbBytes));
    private static readonly Lazy<uint[]> S_gbk = new(() => BuildTable(Table.Gbk, 23940, 936, GbkBytes));
    private static readonly Lazy<uint[]> S_big5 = new(() => BuildTable(Table.Big5, 19782, 950, Big5Bytes));

    /// <summary>The 128 upper characters of an 8-bit code page (THE_CODEPAGES_ANSI[page]); 0 where undefined.</summary>
    internal static char[] CodePage(FormatType format) =>
        S_codePages.Value[(int)format - (int)FormatType.FormatType_CP1250];

    /// <summary>sjisuni[theSjis].</summary>
    internal static uint SjisUni(int sjis) => S_sjis.Value[sjis];

    /// <summary>gbuni[theGb].</summary>
    internal static uint GbUni(int gb) => S_gb.Value[gb];

    /// <summary>gbkuni[thePointer].</summary>
    internal static uint GbkUni(int pointer) => S_gbk.Value[pointer];

    /// <summary>big5uni[thePointer].</summary>
    internal static uint Big5Uni(int pointer) => S_big5.Value[pointer];

    /// <summary>The .NET code page of an 8-bit format, in OCCT's order of THE_CODEPAGES_ANSI.</summary>
    internal static int WindowsCodePage(int page) => page switch
    {
        < 9 => 1250 + page,          // cp1250 ... cp1258
        < 18 => 28591 + (page - 9),  // iso8859-1 ... iso8859-9
        _ => 850,                       // cp850
    };

    /// <summary>The table as .NET's encodings give it, before the corrections (for the comparison with OCCT's).</summary>
    internal static uint[] NetTable(Table table) => table switch
    {
        Table.CodePages => BuildNetCodePages(),
        Table.Sjis => BuildNet(65536, 932, SjisBytes),
        Table.Gb => BuildNet(65536, 936, GbBytes),
        Table.Gbk => BuildNet(23940, 936, GbkBytes),
        _ => BuildNet(19782, 950, Big5Bytes),
    };

    // The bytes .NET decodes for an index of each table (null: no such byte sequence, the entry is 0)
    private static byte[]? SjisBytes(int index) => index < 0x100 ? null : [(byte)(index >> 8), (byte)index];

    private static byte[]? GbBytes(int index)
    {
        int high = index >> 8, low = index & 0xFF;
        return high >= 0x80 || low >= 0x80 ? null : [(byte)(high | 0x80), (byte)(low | 0x80)];
    }

    private static byte[]? GbkBytes(int pointer)
    {
        int trail = pointer % 190;
        return [(byte)(pointer / 190 + 0x81), (byte)(trail + (trail < 0x3F ? 0x40 : 0x41))];
    }

    private static byte[]? Big5Bytes(int pointer)
    {
        int trail = pointer % 157;
        return [(byte)(pointer / 157 + 0x81), (byte)(trail + (trail < 0x3F ? 0x40 : 0x62))];
    }

    private static Encoding Strict(int codePage) =>
        CodePagesEncodingProvider.Instance.GetEncoding(codePage, EncoderFallback.ExceptionFallback, DecoderFallback.ExceptionFallback)
        ?? Encoding.GetEncoding(codePage, EncoderFallback.ExceptionFallback, DecoderFallback.ExceptionFallback);

    // One code point from a byte sequence, 0 if .NET does not decode it to exactly one code point
    private static uint Decode(Encoding encoding, byte[]? bytes)
    {
        if (bytes is null)
        {
            return 0;
        }
        string text;
        try
        {
            text = encoding.GetString(bytes);
        }
        catch (DecoderFallbackException)
        {
            return 0;
        }
        if (text.Length == 1)
        {
            return text[0];
        }
        if (text.Length == 2 && char.IsSurrogatePair(text[0], text[1]))
        {
            return (uint)char.ConvertToUtf32(text[0], text[1]);
        }
        return 0;
    }

    private static uint[] BuildNet(int size, int codePage, Func<int, byte[]?> bytes)
    {
        Encoding encoding = Strict(codePage);
        var table = new uint[size];
        for (int i = 0; i < size; i++)
        {
            table[i] = Decode(encoding, bytes(i));
        }
        return table;
    }

    private static uint[] BuildNetCodePages()
    {
        var table = new uint[NbCodePages * 128];
        for (int page = 0; page < NbCodePages; page++)
        {
            Encoding encoding = Strict(WindowsCodePage(page));
            for (int i = 0; i < 128; i++)
            {
                table[page * 128 + i] = Decode(encoding, [(byte)(0x80 | i)]);
            }
        }
        return table;
    }

    private static uint[] BuildTable(Table table, int size, int codePage, Func<int, byte[]?> bytes)
    {
        uint[] aTable = BuildNet(size, codePage, bytes);
        foreach (KeyValuePair<int, uint> fix in Corrections.Value[(int)table])
        {
            aTable[fix.Key] = fix.Value;
        }
        return aTable;
    }

    private static char[][] BuildCodePages()
    {
        uint[] table = BuildNetCodePages();
        foreach (KeyValuePair<int, uint> fix in Corrections.Value[(int)Table.CodePages])
        {
            table[fix.Key] = fix.Value;
        }
        var pages = new char[NbCodePages][];
        for (int page = 0; page < NbCodePages; page++)
        {
            pages[page] = new char[128];
            for (int i = 0; i < 128; i++)
            {
                pages[page][i] = (char)table[page * 128 + i];
            }
        }
        return pages;
    }

    // The corrections file: for each table in order, a count then (index, value) pairs, all little-endian int32
    private static Dictionary<int, uint>[] LoadCorrections()
    {
        var result = new Dictionary<int, uint>[5];
        for (int i = 0; i < result.Length; i++)
        {
            result[i] = [];
        }
        using Stream? stream = typeof(UnicodeTables).Assembly.GetManifestResourceStream("Occt.Resource.Resource_UnicodeTables.diff.gz");
        if (stream is null)
        {
            return result;
        }
        using var gzip = new GZipStream(stream, CompressionMode.Decompress);
        using var reader = new BinaryReader(gzip);
        for (int table = 0; table < result.Length; table++)
        {
            int count = reader.ReadInt32();
            for (int i = 0; i < count; i++)
            {
                int index = reader.ReadInt32();
                result[table][index] = reader.ReadUInt32();
            }
        }
        return result;
    }
}
