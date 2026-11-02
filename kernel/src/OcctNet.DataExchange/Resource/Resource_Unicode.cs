// Port of: src/FoundationClasses/TKernel/Resource/Resource_Unicode.hxx
// Port of: src/FoundationClasses/TKernel/Resource/Resource_Unicode.cxx
// Port of: src/FoundationClasses/TKernel/Resource/Resource_ConvertUnicode.c
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_UtfIterator.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;

namespace Occt;

/// <summary>
/// The conversions of Resource_Unicode that STEP's reader uses: a text of a given format (code page) to Unicode.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's logic is ported as is (how bytes are grouped into characters, its fallbacks and quirks), but OCCT's code
/// tables (Resource_CodePages, Resource_Shiftjis, Resource_GB2312, Resource_GBK, Resource_Big5: 25,700 lines) are not:
/// each table lookup asks .NET's encodings (<see cref="CodePagesEncodingProvider"/>) instead, corrected where they
/// differ from OCCT's table by <see cref="Resource_UnicodeTables"/> (decision 1166, 12501).
/// </para>
/// <para>
/// Texts are C strings of bytes, given as .NET strings of one char per byte (decision 1174); the result is UTF-16, as
/// OCCT's <c>TCollection_ExtendedString</c>.
/// </para>
/// </remarks>
public static class Resource_Unicode
{
    /// <summary>Converts a non-ASCII C string (one char per byte) of the given format to Unicode.</summary>
    public static string ConvertFormatToUnicode(Resource_FormatType theFormat, string theFromStr)
    {
        var aToStr = new StringBuilder();
        ConvertFormatToUnicode(theFormat, theFromStr, aToStr);
        return aToStr.ToString();
    }

    /// <summary>Converts a non-ASCII C string (one char per byte) of the given format to Unicode, into <paramref name="theToStr"/> (cleared first).</summary>
    public static void ConvertFormatToUnicode(Resource_FormatType theFormat, string theFromStr, StringBuilder theToStr)
    {
        byte[] aFrom = CBytes(theFromStr);
        switch (theFormat)
        {
            case Resource_FormatType.Resource_FormatType_SJIS:
                ConvertSJISToUnicode(aFrom, theToStr);
                break;
            case Resource_FormatType.Resource_FormatType_EUC:
                ConvertEUCToUnicode(aFrom, theToStr);
                break;
            case Resource_FormatType.Resource_FormatType_GB:
                ConvertGBToUnicode(aFrom, theToStr);
                break;
            case Resource_FormatType.Resource_FormatType_ANSI:
                theToStr.Clear();
                AppendBytes(aFrom, 0, CLength(aFrom, 0), theToStr); // TCollection_ExtendedString(theFromStr, false)
                break;
            case Resource_FormatType.Resource_FormatType_CP1250:
            case Resource_FormatType.Resource_FormatType_CP1251:
            case Resource_FormatType.Resource_FormatType_CP1252:
            case Resource_FormatType.Resource_FormatType_CP1253:
            case Resource_FormatType.Resource_FormatType_CP1254:
            case Resource_FormatType.Resource_FormatType_CP1255:
            case Resource_FormatType.Resource_FormatType_CP1256:
            case Resource_FormatType.Resource_FormatType_CP1257:
            case Resource_FormatType.Resource_FormatType_CP1258:
            case Resource_FormatType.Resource_FormatType_iso8859_1:
            case Resource_FormatType.Resource_FormatType_iso8859_2:
            case Resource_FormatType.Resource_FormatType_iso8859_3:
            case Resource_FormatType.Resource_FormatType_iso8859_4:
            case Resource_FormatType.Resource_FormatType_iso8859_5:
            case Resource_FormatType.Resource_FormatType_iso8859_6:
            case Resource_FormatType.Resource_FormatType_iso8859_7:
            case Resource_FormatType.Resource_FormatType_iso8859_8:
            case Resource_FormatType.Resource_FormatType_iso8859_9:
            case Resource_FormatType.Resource_FormatType_CP850:
            {
                char[] aCodePage = Resource_UnicodeTables.CodePage(theFormat);
                theToStr.Clear();
                for (int anInputPntr = 0; anInputPntr < aFrom.Length && aFrom[anInputPntr] != 0; ++anInputPntr)
                {
                    byte anInputChar = aFrom[anInputPntr];
                    char aRes = (anInputChar & 0x80) != 0 ? aCodePage[0x7f & anInputChar] : (char)anInputChar;
                    if (aRes == 0)
                    {
                        aRes = '?';
                    }
                    theToStr.Append(aRes);
                }
                break;
            }
            case Resource_FormatType.Resource_FormatType_Big5:
                ConvertBig5ToUnicode(aFrom, theToStr);
                break;
            case Resource_FormatType.Resource_FormatType_GBK:
                ConvertGBKToUnicode(aFrom, theToStr);
                break;
            case Resource_FormatType.Resource_FormatType_UTF8:
                theToStr.Clear();
                ExtendedStringFromMultiByte(aFrom, theToStr); // TCollection_ExtendedString(theFromStr, true)
                break;
            case Resource_FormatType.Resource_FormatType_SystemLocale:
                // OCCT.NET: OCCT converts from the process's C locale (NCollection_UtfString::FromLocale); the port has
                // no such locale and reads the text as UTF-8 (decision 12501)
                theToStr.Clear();
                ExtendedStringFromMultiByte(aFrom, theToStr);
                break;
        }
    }

    // The bytes of a C string given one char per byte (chars above 0xFF, which no file text holds, keep their low byte)
    private static byte[] CBytes(string theStr)
    {
        var aBytes = new byte[theStr.Length];
        for (int i = 0; i < theStr.Length; i++)
        {
            aBytes[i] = unchecked((byte)theStr[i]);
        }
        return aBytes;
    }

    // C's strlen from an index: up to the first zero byte or the end
    private static int CLength(byte[] theBytes, int theFrom)
    {
        int anEnd = Array.IndexOf(theBytes, (byte)0, theFrom);
        return (anEnd < 0 ? theBytes.Length : anEnd) - theFrom;
    }

    // C's *p on a C string: the byte, or 0 at and past its end (OCCT reads past the terminator there)
    private static byte At(byte[] theBytes, int theIndex) => theIndex < theBytes.Length ? theBytes[theIndex] : (byte)0;

    // TCollection_ExtendedString(const char*, false), and its fallback: each byte to the char of the same code
    private static void AppendBytes(byte[] theBytes, int theFrom, int theLength, StringBuilder theToStr)
    {
        for (int i = theFrom; i < theFrom + theLength; i++)
        {
            theToStr.Append((char)theBytes[i]);
        }
    }

    #region UTF-8 (NCollection_UtfIterator<char>, TCollection_ExtendedString::ConvertToUnicode)

    // ConvertUTF.c's tables, as NCollection_UtfIterator uses them
    private static readonly byte[] UTF8_BYTES_MINUS_ONE = BuildBytesMinusOne();
    private static readonly uint[] offsetsFromUTF8 = [0x00000000u, 0x00003080u, 0x000E2080u, 0x03C82080u, 0xFA082080u, 0x82082080u];
    private const uint UTF32_MAX_LEGAL = 0x0010FFFF;
    private const uint UTF32_MAX_BMP = 0x0000FFFF;
    private const uint UTF16_SURROGATE_HIGH_START = 0xD800;
    private const uint UTF16_SURROGATE_LOW_END = 0xDFFF;
    private const uint UTF16_SURROGATE_LOW_BASE = 0x0010000;
    private const int UTF16_SURROGATE_HIGH_SHIFT = 10;
    private const uint UTF16_SURROGATE_LOW_MASK = 0x3FF;
    private const uint UTF16_SURROGATE_LOW_START = 0xDC00;

    private static byte[] BuildBytesMinusOne()
    {
        var aTable = new byte[256];
        for (int i = 0; i < 256; i++)
        {
            aTable[i] = (byte)(i < 0xC0 ? 0 : i < 0xE0 ? 1 : i < 0xF0 ? 2 : i < 0xF8 ? 3 : i < 0xFC ? 4 : 5);
        }
        return aTable;
    }

    /// <summary>
    /// NCollection_UtfIterator&lt;char&gt;::readUTF8: one code point from a lead byte and as many following bytes as it
    /// announces, whatever they are ("not protected against invalid UTF-8").
    /// </summary>
    private static uint ReadUTF8(byte[] theBytes, ref int thePos)
    {
        byte aBytesToRead = UTF8_BYTES_MINUS_ONE[At(theBytes, thePos)];
        uint aChar = 0;
        for (int i = aBytesToRead; i >= 0; i--)
        {
            aChar += At(theBytes, thePos++);
            if (i > 0)
            {
                aChar <<= 6;
            }
        }
        return unchecked(aChar - offsetsFromUTF8[aBytesToRead]);
    }

    /// <summary>
    /// TCollection_ExtendedString(theString, true): the UTF-8 C string as UTF-16 (code points in the surrogate range
    /// dropped, as GetUtf16 does); if one code point is above U+10FFFF, each byte as a char instead.
    /// </summary>
    /// <remarks>
    /// OCCT.NET: a lead byte near the end makes OCCT read its terminating zero as a continuation byte, then go on past
    /// it (beyond the string); the port reads zeros there and stops at the end.
    /// </remarks>
    private static void ExtendedStringFromMultiByte(byte[] theBytes, StringBuilder theToStr)
    {
        int aStart = theToStr.Length;
        int aPos = 0;
        while (aPos < theBytes.Length)
        {
            uint aChar = ReadUTF8(theBytes, ref aPos);
            if (aChar == 0)
            {
                break;
            }
            if (aChar > UTF32_MAX_LEGAL)
            {
                // !IsValid(): falls back to one char per byte
                theToStr.Length = aStart;
                AppendBytes(theBytes, 0, CLength(theBytes, 0), theToStr);
                return;
            }
            AppendUtf16(aChar, theToStr);
        }
    }

    // NCollection_UtfIterator::GetUtf16
    internal static void AppendUtf16(uint theChar, StringBuilder theToStr)
    {
        if (theChar <= UTF32_MAX_BMP)
        {
            if (theChar >= UTF16_SURROGATE_HIGH_START && theChar <= UTF16_SURROGATE_LOW_END)
            {
                return;
            }
            theToStr.Append((char)theChar);
        }
        else if (theChar > UTF32_MAX_LEGAL)
        {
            // illegal
        }
        else
        {
            uint aChar = theChar - UTF16_SURROGATE_LOW_BASE;
            theToStr.Append((char)((aChar >> UTF16_SURROGATE_HIGH_SHIFT) + UTF16_SURROGATE_HIGH_START));
            theToStr.Append((char)((aChar & UTF16_SURROGATE_LOW_MASK) + UTF16_SURROGATE_LOW_START));
        }
    }

    /// <summary>
    /// TCollection_AsciiString(const TCollection_ExtendedString&amp;, 0) read back as Unicode: the UTF-16 text whose
    /// UTF-8 form OCCT writes. Its UTF-8 conversion skips unpaired surrogates (code points without a UTF-8 form).
    /// </summary>
    internal static string ToUtf8Equivalent(StringBuilder theExtString)
    {
        var aResult = new StringBuilder(theExtString.Length);
        for (int i = 0; i < theExtString.Length; i++)
        {
            char aChar = theExtString[i];
            if (char.IsHighSurrogate(aChar) && i + 1 < theExtString.Length && char.IsLowSurrogate(theExtString[i + 1]))
            {
                aResult.Append(aChar).Append(theExtString[i + 1]);
                i++;
            }
            else if (!char.IsSurrogate(aChar))
            {
                aResult.Append(aChar);
            }
        }
        return aResult.ToString();
    }

    #endregion

    #region Multi-byte code pages

    private static bool issjis1(uint c) => (c >= 0x81 && c <= 0x9f) || (c >= 0xe0 && c <= 0xef);

    private static bool issjis2(uint c) => c >= 0x40 && c <= 0xfc && c != 0x7f;

    private static bool iseuc(uint c) => c >= 0xa1 && c <= 0xfe;

    private static bool isshift(uint c) => c >= 0x80 && c <= 0xff;

    // TCollection_ExtendedString(char): the char of the byte's code
    private static char ToExtCharacter(byte theChar) => (char)theChar;

    /// <summary>Converts a SJIS C string (one char per byte) to Unicode.</summary>
    public static void ConvertSJISToUnicode(byte[] fromstr, StringBuilder tostr)
    {
        tostr.Clear();
        int currentstr = 0;
        uint pl, ph;
        // BIG INDIAN USED HERE
        while (At(fromstr, currentstr) != 0)
        {
            if (issjis1(At(fromstr, currentstr)))
            {
                ph = At(fromstr, currentstr);
                // Be Careful with first and second !!
                currentstr++;
                pl = At(fromstr, currentstr);
                currentstr++;
                Resource_sjis_to_unicode(ref ph, ref pl);
                char curcar = (char)((ph << 8) | pl);
                tostr.Append(curcar);
            }
            else
            {
                tostr.Append(ToExtCharacter(At(fromstr, currentstr)));
                currentstr++;
            }
        }
    }

    /// <summary>Converts an EUC C string (one char per byte) to Unicode.</summary>
    public static void ConvertEUCToUnicode(byte[] fromstr, StringBuilder tostr)
    {
        tostr.Clear();
        int currentstr = 0;
        uint pl, ph;
        // BIG INDIAN USED HERE
        while (At(fromstr, currentstr) != 0)
        {
            if (iseuc(At(fromstr, currentstr)))
            {
                ph = At(fromstr, currentstr);
                // Be Careful with first and second !!
                currentstr++;
                pl = At(fromstr, currentstr);
                currentstr++;
                Resource_euc_to_unicode(ref ph, ref pl);
                char curcar = (char)((ph << 8) | pl);
                tostr.Append(curcar);
            }
            else
            {
                tostr.Append(ToExtCharacter(At(fromstr, currentstr)));
                currentstr++;
            }
        }
    }

    /// <summary>Converts a GB C string (one char per byte) to Unicode.</summary>
    public static void ConvertGBToUnicode(byte[] fromstr, StringBuilder tostr)
    {
        tostr.Clear();
        int currentstr = 0;
        uint pl, ph;
        // BIG INDIAN USED HERE
        while (At(fromstr, currentstr) != 0)
        {
            if (isshift(At(fromstr, currentstr)))
            {
                ph = At(fromstr, currentstr);
                // Be Careful with first and second !!
                currentstr++;
                pl = At(fromstr, currentstr);
                currentstr++;
                Resource_gb_to_unicode(ref ph, ref pl);
                char curcar = (char)((ph << 8) | pl);
                tostr.Append(curcar);
            }
            else
            {
                tostr.Append(ToExtCharacter(At(fromstr, currentstr)));
                currentstr++;
            }
        }
    }

    /// <summary>Converts a GBK C string (one char per byte) to Unicode. Returns false if the text is not GBK.</summary>
    public static bool ConvertGBKToUnicode(byte[] fromstr, StringBuilder tostr)
    {
        tostr.Clear();
        int currentch = 0;
        uint gb1 = 0x00, gb2 = 0x00, gb3 = 0x00;
        while (At(fromstr, currentch) != 0)
        {
            uint aCur = At(fromstr, currentch);
            if (gb3 != 0x00)
            {
                if (aCur < 0x30 || aCur > 0x39)
                {
                    throw InsertAtZero(); // OCCT: TCollection_ExtendedString::Insert(0, ...) throws
                }
                uint codepnt = ((gb1 - 0x81) * (10 * 126 * 10)) + ((gb2 - 0x30) * (10 * 126)) + ((gb3 - 0x81) * 10) + aCur - 0x30;
                if (codepnt < 23940)
                {
                    char curcar = (char)Resource_UnicodeTables.GbkUni((int)codepnt);
                    tostr.Append(curcar);
                    currentch++;
                    continue; // OCCT: gb1, gb2 and gb3 stay set, so each following byte is read as a 4th byte
                }
                return false;
            }
            else if (gb2 != 0x00)
            {
                if (aCur >= 0x81 && aCur <= 0xFE)
                {
                    gb3 = aCur;
                    currentch++;
                    continue;
                }
                throw InsertAtZero(); // OCCT: TCollection_ExtendedString::Insert(0, ...) throws
            }
            else if (gb1 != 0x00)
            {
                if (aCur >= 0x30 && aCur <= 0x39)
                {
                    gb2 = aCur;
                    currentch++;
                    continue;
                }
                uint lead = gb1;
                uint pointer = 0;
                gb1 = 0x00;
                uint offset = aCur < 0x7F ? 0x40u : 0x41u;
                if ((aCur >= 0x40 && aCur <= 0x7E) || (aCur >= 0x80 && aCur <= 0xFE))
                {
                    pointer = (lead - 0x81) * 190 + (aCur - offset);
                    if (pointer < 23940)
                    {
                        char curcar = (char)Resource_UnicodeTables.GbkUni((int)pointer);
                        tostr.Append(curcar);
                        currentch++;
                        continue;
                    }
                }
                if (aCur <= 0x7F)
                {
                    // ASCII symbol
                    currentch++;
                    throw InsertAtZero(); // OCCT: TCollection_ExtendedString::Insert(0, ...) throws
                }
                return false;
            }
            else
            {
                if (aCur <= 0x7F)
                {
                    // ASCII symbol
                    currentch++;
                    tostr.Append((char)aCur);
                }
                else if (aCur == 0x80)
                {
                    // Special symbol
                    tostr.Append((char)((0x20 << 8) | 0xAC));
                    currentch++;
                }
                else if (aCur >= 0x81 && aCur <= 0xFE)
                {
                    // Chinese symbol
                    gb1 = aCur;
                    currentch++;
                }
                else
                {
                    return false;
                }
            }
        }
        return true;
    }

    /// <summary>Converts a Big5 C string (one char per byte) to Unicode. Returns false if the text is not Big5.</summary>
    public static bool ConvertBig5ToUnicode(byte[] fromstr, StringBuilder tostr)
    {
        tostr.Clear();
        int currentch = 0;
        uint big5lead = 0x00;
        while (At(fromstr, currentch) != 0)
        {
            uint aCur = At(fromstr, currentch);
            if (big5lead != 0x00)
            {
                uint lead = big5lead;
                uint pointer = 0;
                big5lead = 0x00;
                uint offset = aCur < 0x7F ? 0x40u : 0x62u;
                if ((aCur >= 0x40 && aCur <= 0x7E) || (aCur >= 0xA1 && aCur <= 0xFE))
                {
                    pointer = (lead - 0x81) * 157 + (aCur - offset);
                    switch (pointer)
                    {
                        case 1133:
                            tostr.Append((char)0x00CA).Append((char)0x0304);
                            currentch++;
                            continue;
                        case 1135:
                            tostr.Append((char)0x00CA).Append((char)0x030C);
                            currentch++;
                            continue;
                        case 1164:
                            tostr.Append((char)0x00EA).Append((char)0x0304);
                            currentch++;
                            continue;
                        case 1166:
                            tostr.Append((char)0x00EA).Append((char)0x030C);
                            currentch++;
                            continue;
                        default:
                            if (pointer < 19782)
                            {
                                uint uni = Resource_UnicodeTables.Big5Uni((int)pointer);
                                if (uni <= 0xFFFF)
                                {
                                    tostr.Append((char)uni);
                                }
                                else
                                {
                                    // NCollection_UtfString<char32_t>::ToUtf16: a surrogate pair
                                    if (uni > UTF32_MAX_LEGAL)
                                    {
                                        return false; // not a surrogate pair
                                    }
                                    AppendUtf16(uni, tostr);
                                }
                                currentch++;
                                continue;
                            }
                            break;
                    }
                }
                if (aCur <= 0x7F)
                {
                    // ASCII symbol
                    currentch++;
                    throw InsertAtZero(); // OCCT: TCollection_ExtendedString::Insert(0, ...) throws
                }
                return false;
            }
            else
            {
                if (aCur <= 0x7F)
                {
                    // ASCII symbol
                    currentch++;
                    tostr.Append((char)aCur);
                }
                else if (aCur >= 0x81 && aCur <= 0xFE)
                {
                    // Chinese symbol
                    big5lead = aCur;
                    currentch++;
                }
                else
                {
                    return false;
                }
            }
        }
        return true;
    }

    // TCollection_ExtendedString::Insert(0, ...): OCCT inserts the bytes read so far "at 0", which is out of range
    private static Standard_OutOfRange InsertAtZero() =>
        new("TCollection_ExtendedString::Insert : Parameter theWhere is out of range");

    // Resource_ConvertUnicode.c

    private static void jis_to_sjis(ref uint ph, ref uint pl)
    {
        if ((ph & 1) != 0)
        {
            if (pl < 0x60)
            {
                pl += 0x1f;
            }
            else
            {
                pl += 0x20;
            }
        }
        else
        {
            pl += 0x7e;
        }
        if (ph < 0x5f)
        {
            ph = (ph + 0xe1) >> 1;
        }
        else
        {
            ph = (ph + 0x161) >> 1;
        }
    }

    private static void euc_to_sjis(ref uint ph, ref uint pl)
    {
        if ((ph & 0xFFFFFF00) != 0 || (pl & 0xFFFFFF00) != 0)
        {
            ph = 0;
            pl = 0;
            return;
        }
        if (!iseuc(ph) || !iseuc(pl))
        {
            return;
        }
        ph &= 0x7F;
        pl &= 0x7F;
        jis_to_sjis(ref ph, ref pl);
    }

    internal static void Resource_sjis_to_unicode(ref uint ph, ref uint pl)
    {
        if ((ph & 0xFFFFFF00) != 0 || (pl & 0xFFFFFF00) != 0)
        {
            ph = 0;
            pl = 0;
            return;
        }
        if (!issjis1(ph) || !issjis2(pl))
        {
            return;
        }
        int sjis = (int)((ph << 8) | pl);
        uint uni = Resource_UnicodeTables.SjisUni(sjis);
        ph = uni >> 8;
        pl = uni & 0xFF;
    }

    internal static void Resource_euc_to_unicode(ref uint ph, ref uint pl)
    {
        if (!iseuc(ph) || !iseuc(pl))
        {
            return;
        }
        if (ph == 0 && pl == 0)
        {
            return;
        }
        euc_to_sjis(ref ph, ref pl);
        Resource_sjis_to_unicode(ref ph, ref pl);
    }

    internal static void Resource_gb_to_unicode(ref uint ph, ref uint pl)
    {
        if ((ph & 0xFFFFFF00) != 0 || (pl & 0xFFFFFF00) != 0)
        {
            ph = 0;
            pl = 0;
            return;
        }
        if (!isshift(ph) || !isshift(pl))
        {
            return;
        }
        ph = ph & 0x7f;
        pl = pl & 0x7f;
        int gb = (int)((ph << 8) | pl);
        uint uni = Resource_UnicodeTables.GbUni(gb);
        ph = uni >> 8;
        pl = uni & 0xFF;
    }

    #endregion
}
