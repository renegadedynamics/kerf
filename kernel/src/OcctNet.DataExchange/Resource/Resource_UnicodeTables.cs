// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.IO.Compression;
using System.Text;

namespace Occt;

/// <summary>
/// OCCT.NET's stand-in for Resource_Unicode's code tables (Resource_CodePages.pxx, Resource_Shiftjis.pxx,
/// Resource_GB2312.pxx, Resource_GBK.pxx, Resource_Big5.pxx): each lookup asks .NET's encodings
/// (<see cref="CodePagesEncodingProvider"/>), then applies the corrections recorded where .NET differs from OCCT's
/// table (the embedded <c>Resource_UnicodeTables.diff.gz</c>, made by tools/stepdatarec's table dump; decision 1191).
/// </summary>
/// <remarks>Built once per process, on first use, and read-only after (thread-safe).</remarks>
internal static class Resource_UnicodeTables
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

    private const int NbCodePages = (int)Resource_FormatType.Resource_FormatType_CP850 - (int)Resource_FormatType.Resource_FormatType_CP1250 + 1;

    private static readonly Lazy<Dictionary<int, uint>[]> s_corrections = new(LoadCorrections);
    private static readonly Lazy<char[][]> s_codePages = new(BuildCodePages);
    private static readonly Lazy<uint[]> s_sjis = new(() => BuildTable(Table.Sjis, 65536, 932, SjisBytes));
    private static readonly Lazy<uint[]> s_gb = new(() => BuildTable(Table.Gb, 65536, 936, GbBytes));
    private static readonly Lazy<uint[]> s_gbk = new(() => BuildTable(Table.Gbk, 23940, 936, GbkBytes));
    private static readonly Lazy<uint[]> s_big5 = new(() => BuildTable(Table.Big5, 19782, 950, Big5Bytes));

    /// <summary>The 128 upper characters of an 8-bit code page (THE_CODEPAGES_ANSI[page]); 0 where undefined.</summary>
    internal static char[] CodePage(Resource_FormatType theFormat) =>
        s_codePages.Value[(int)theFormat - (int)Resource_FormatType.Resource_FormatType_CP1250];

    /// <summary>sjisuni[theSjis].</summary>
    internal static uint SjisUni(int theSjis) => s_sjis.Value[theSjis];

    /// <summary>gbuni[theGb].</summary>
    internal static uint GbUni(int theGb) => s_gb.Value[theGb];

    /// <summary>gbkuni[thePointer].</summary>
    internal static uint GbkUni(int thePointer) => s_gbk.Value[thePointer];

    /// <summary>big5uni[thePointer].</summary>
    internal static uint Big5Uni(int thePointer) => s_big5.Value[thePointer];

    /// <summary>The .NET code page of an 8-bit format, in OCCT's order of THE_CODEPAGES_ANSI.</summary>
    internal static int WindowsCodePage(int thePage) => thePage switch
    {
        < 9 => 1250 + thePage,          // cp1250 ... cp1258
        < 18 => 28591 + (thePage - 9),  // iso8859-1 ... iso8859-9
        _ => 850,                       // cp850
    };

    /// <summary>The table as .NET's encodings give it, before the corrections (for the comparison with OCCT's).</summary>
    internal static uint[] NetTable(Table theTable) => theTable switch
    {
        Table.CodePages => BuildNetCodePages(),
        Table.Sjis => BuildNet(65536, 932, SjisBytes),
        Table.Gb => BuildNet(65536, 936, GbBytes),
        Table.Gbk => BuildNet(23940, 936, GbkBytes),
        _ => BuildNet(19782, 950, Big5Bytes),
    };

    // The bytes .NET decodes for an index of each table (null: no such byte sequence, the entry is 0)
    private static byte[]? SjisBytes(int theIndex) => theIndex < 0x100 ? null : [(byte)(theIndex >> 8), (byte)theIndex];

    private static byte[]? GbBytes(int theIndex)
    {
        int aHigh = theIndex >> 8, aLow = theIndex & 0xFF;
        return aHigh >= 0x80 || aLow >= 0x80 ? null : [(byte)(aHigh | 0x80), (byte)(aLow | 0x80)];
    }

    private static byte[]? GbkBytes(int thePointer)
    {
        int aTrail = thePointer % 190;
        return [(byte)(thePointer / 190 + 0x81), (byte)(aTrail + (aTrail < 0x3F ? 0x40 : 0x41))];
    }

    private static byte[]? Big5Bytes(int thePointer)
    {
        int aTrail = thePointer % 157;
        return [(byte)(thePointer / 157 + 0x81), (byte)(aTrail + (aTrail < 0x3F ? 0x40 : 0x62))];
    }

    private static Encoding Strict(int theCodePage) =>
        CodePagesEncodingProvider.Instance.GetEncoding(theCodePage, EncoderFallback.ExceptionFallback, DecoderFallback.ExceptionFallback)
        ?? Encoding.GetEncoding(theCodePage, EncoderFallback.ExceptionFallback, DecoderFallback.ExceptionFallback);

    // One code point from a byte sequence, 0 if .NET does not decode it to exactly one code point
    private static uint Decode(Encoding theEncoding, byte[]? theBytes)
    {
        if (theBytes is null)
        {
            return 0;
        }
        string aText;
        try
        {
            aText = theEncoding.GetString(theBytes);
        }
        catch (DecoderFallbackException)
        {
            return 0;
        }
        if (aText.Length == 1)
        {
            return aText[0];
        }
        if (aText.Length == 2 && char.IsSurrogatePair(aText[0], aText[1]))
        {
            return (uint)char.ConvertToUtf32(aText[0], aText[1]);
        }
        return 0;
    }

    private static uint[] BuildNet(int theSize, int theCodePage, Func<int, byte[]?> theBytes)
    {
        Encoding anEncoding = Strict(theCodePage);
        var aTable = new uint[theSize];
        for (int i = 0; i < theSize; i++)
        {
            aTable[i] = Decode(anEncoding, theBytes(i));
        }
        return aTable;
    }

    private static uint[] BuildNetCodePages()
    {
        var aTable = new uint[NbCodePages * 128];
        for (int aPage = 0; aPage < NbCodePages; aPage++)
        {
            Encoding anEncoding = Strict(WindowsCodePage(aPage));
            for (int i = 0; i < 128; i++)
            {
                aTable[aPage * 128 + i] = Decode(anEncoding, [(byte)(0x80 | i)]);
            }
        }
        return aTable;
    }

    private static uint[] BuildTable(Table theTable, int theSize, int theCodePage, Func<int, byte[]?> theBytes)
    {
        uint[] aTable = BuildNet(theSize, theCodePage, theBytes);
        foreach (KeyValuePair<int, uint> aFix in s_corrections.Value[(int)theTable])
        {
            aTable[aFix.Key] = aFix.Value;
        }
        return aTable;
    }

    private static char[][] BuildCodePages()
    {
        uint[] aTable = BuildNetCodePages();
        foreach (KeyValuePair<int, uint> aFix in s_corrections.Value[(int)Table.CodePages])
        {
            aTable[aFix.Key] = aFix.Value;
        }
        var aPages = new char[NbCodePages][];
        for (int aPage = 0; aPage < NbCodePages; aPage++)
        {
            aPages[aPage] = new char[128];
            for (int i = 0; i < 128; i++)
            {
                aPages[aPage][i] = (char)aTable[aPage * 128 + i];
            }
        }
        return aPages;
    }

    // The corrections file: for each table in order, a count then (index, value) pairs, all little-endian int32
    private static Dictionary<int, uint>[] LoadCorrections()
    {
        var aResult = new Dictionary<int, uint>[5];
        for (int i = 0; i < aResult.Length; i++)
        {
            aResult[i] = [];
        }
        using Stream? aStream = typeof(Resource_UnicodeTables).Assembly.GetManifestResourceStream("Occt.Resource.Resource_UnicodeTables.diff.gz");
        if (aStream is null)
        {
            return aResult;
        }
        using var aGzip = new GZipStream(aStream, CompressionMode.Decompress);
        using var aReader = new BinaryReader(aGzip);
        for (int aTable = 0; aTable < aResult.Length; aTable++)
        {
            int aCount = aReader.ReadInt32();
            for (int i = 0; i < aCount; i++)
            {
                int anIndex = aReader.ReadInt32();
                aResult[aTable][anIndex] = aReader.ReadUInt32();
            }
        }
        return aResult;
    }
}
