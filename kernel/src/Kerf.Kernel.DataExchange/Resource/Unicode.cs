// Port of: src/FoundationClasses/TKernel/Resource/Resource_Unicode.hxx
// Port of: src/FoundationClasses/TKernel/Resource/Resource_Unicode.cxx
// Port of: src/FoundationClasses/TKernel/Resource/Resource_ConvertUnicode.c
// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_UtfIterator.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Text;

namespace Kerf.Engine.Foundation.Resources;

/// <summary>
/// The conversions of Resource_Unicode that STEP's reader uses: a text of a given format (code page) to Unicode.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's logic is ported as is (how bytes are grouped into characters, its fallbacks and quirks), but OCCT's code
/// tables (Resource_CodePages, Resource_Shiftjis, Resource_GB2312, Resource_GBK, Resource_Big5: 25,700 lines) are not:
/// each table lookup asks .NET's encodings (<see cref="CodePagesEncodingProvider"/>) instead, corrected where they
/// differ from OCCT's table by <see cref="UnicodeTables"/> (decision 1166, 1191).
/// </para>
/// <para>
/// Texts are C strings of bytes, given as .NET strings of one char per byte (decision 1174); the result is UTF-16, as
/// OCCT's <c>TCollection_ExtendedString</c>.
/// </para>
/// </remarks>
internal static class Unicode
{
    /// <summary>Converts a non-ASCII C string (one char per byte) of the given format to Unicode.</summary>
    public static string ConvertFormatToUnicode(FormatType format, string fromStr)
    {
        var toStr = new StringBuilder();
        ConvertFormatToUnicode(format, fromStr, toStr);
        return toStr.ToString();
    }

    /// <summary>Converts a non-ASCII C string (one char per byte) of the given format to Unicode, into <paramref name="toStr"/> (cleared first).</summary>
    public static void ConvertFormatToUnicode(FormatType format, string fromStr, StringBuilder toStr)
    {
        byte[] aFrom = CBytes(fromStr);
        switch (format)
        {
            case FormatType.FormatType_SJIS:
                ConvertSJISToUnicode(aFrom, toStr);
                break;
            case FormatType.FormatType_EUC:
                ConvertEUCToUnicode(aFrom, toStr);
                break;
            case FormatType.FormatType_GB:
                ConvertGBToUnicode(aFrom, toStr);
                break;
            case FormatType.FormatType_ANSI:
                toStr.Clear();
                AppendBytes(aFrom, 0, CLength(aFrom, 0), toStr); // TCollection_ExtendedString(theFromStr, false)
                break;
            case FormatType.FormatType_CP1250:
            case FormatType.FormatType_CP1251:
            case FormatType.FormatType_CP1252:
            case FormatType.FormatType_CP1253:
            case FormatType.FormatType_CP1254:
            case FormatType.FormatType_CP1255:
            case FormatType.FormatType_CP1256:
            case FormatType.FormatType_CP1257:
            case FormatType.FormatType_CP1258:
            case FormatType.FormatType_iso8859_1:
            case FormatType.FormatType_iso8859_2:
            case FormatType.FormatType_iso8859_3:
            case FormatType.FormatType_iso8859_4:
            case FormatType.FormatType_iso8859_5:
            case FormatType.FormatType_iso8859_6:
            case FormatType.FormatType_iso8859_7:
            case FormatType.FormatType_iso8859_8:
            case FormatType.FormatType_iso8859_9:
            case FormatType.FormatType_CP850:
            {
                char[] codePage = UnicodeTables.CodePage(format);
                toStr.Clear();
                for (int inputPntr = 0; inputPntr < aFrom.Length && aFrom[inputPntr] != 0; ++inputPntr)
                {
                    byte inputChar = aFrom[inputPntr];
                    char res = (inputChar & 0x80) != 0 ? codePage[0x7f & inputChar] : (char)inputChar;
                    if (res == 0)
                    {
                        res = '?';
                    }
                    toStr.Append(res);
                }
                break;
            }
            case FormatType.FormatType_Big5:
                ConvertBig5ToUnicode(aFrom, toStr);
                break;
            case FormatType.FormatType_GBK:
                ConvertGBKToUnicode(aFrom, toStr);
                break;
            case FormatType.FormatType_UTF8:
                toStr.Clear();
                ExtendedStringFromMultiByte(aFrom, toStr); // TCollection_ExtendedString(theFromStr, true)
                break;
            case FormatType.FormatType_SystemLocale:
                // Kerf: OCCT converts from the process's C locale (NCollection_UtfString::FromLocale); the port has
                // no such locale and reads the text as UTF-8 (decision 1191)
                toStr.Clear();
                ExtendedStringFromMultiByte(aFrom, toStr);
                break;
        }
    }

    // The bytes of a C string given one char per byte (chars above 0xFF, which no file text holds, keep their low byte)
    private static byte[] CBytes(string str)
    {
        var bytes = new byte[str.Length];
        for (int i = 0; i < str.Length; i++)
        {
            bytes[i] = unchecked((byte)str[i]);
        }
        return bytes;
    }

    // C's strlen from an index: up to the first zero byte or the end
    private static int CLength(byte[] bytes, int theFrom)
    {
        int end = Array.IndexOf(bytes, (byte)0, theFrom);
        return (end < 0 ? bytes.Length : end) - theFrom;
    }

    // C's *p on a C string: the byte, or 0 at and past its end (OCCT reads past the terminator there)
    private static byte At(byte[] bytes, int index) => index < bytes.Length ? bytes[index] : (byte)0;

    // TCollection_ExtendedString(const char*, false), and its fallback: each byte to the char of the same code
    private static void AppendBytes(byte[] bytes, int theFrom, int length, StringBuilder toStr)
    {
        for (int i = theFrom; i < theFrom + length; i++)
        {
            toStr.Append((char)bytes[i]);
        }
    }

    #region UTF-8 (NCollection_UtfIterator<char>, TCollection_ExtendedString::ConvertToUnicode)

    // ConvertUTF.c's tables, as NCollection_UtfIterator uses them
    private static readonly byte[] UTF8_BYTES_MINUS_ONE = BuildBytesMinusOne();
    private static readonly uint[] OffsetsFromUTF8 = [0x00000000u, 0x00003080u, 0x000E2080u, 0x03C82080u, 0xFA082080u, 0x82082080u];
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
        var table = new byte[256];
        for (int i = 0; i < 256; i++)
        {
            table[i] = (byte)(i < 0xC0 ? 0 : i < 0xE0 ? 1 : i < 0xF0 ? 2 : i < 0xF8 ? 3 : i < 0xFC ? 4 : 5);
        }
        return table;
    }

    /// <summary>
    /// NCollection_UtfIterator&lt;char&gt;::readUTF8: one code point from a lead byte and as many following bytes as it
    /// announces, whatever they are ("not protected against invalid UTF-8").
    /// </summary>
    private static uint ReadUTF8(byte[] bytes, ref int pos)
    {
        byte bytesToRead = UTF8_BYTES_MINUS_ONE[At(bytes, pos)];
        uint aChar = 0;
        for (int i = bytesToRead; i >= 0; i--)
        {
            aChar += At(bytes, pos++);
            if (i > 0)
            {
                aChar <<= 6;
            }
        }
        return unchecked(aChar - OffsetsFromUTF8[bytesToRead]);
    }

    /// <summary>
    /// TCollection_ExtendedString(theString, true): the UTF-8 C string as UTF-16 (code points in the surrogate range
    /// dropped, as GetUtf16 does); if one code point is above U+10FFFF, each byte as a char instead.
    /// </summary>
    /// <remarks>
    /// Kerf: a lead byte near the end makes OCCT read its terminating zero as a continuation byte, then go on past
    /// it (beyond the string); the port reads zeros there and stops at the end.
    /// </remarks>
    private static void ExtendedStringFromMultiByte(byte[] bytes, StringBuilder toStr)
    {
        int start = toStr.Length;
        int pos = 0;
        while (pos < bytes.Length)
        {
            uint aChar = ReadUTF8(bytes, ref pos);
            if (aChar == 0)
            {
                break;
            }
            if (aChar > UTF32_MAX_LEGAL)
            {
                // !IsValid(): falls back to one char per byte
                toStr.Length = start;
                AppendBytes(bytes, 0, CLength(bytes, 0), toStr);
                return;
            }
            AppendUtf16(aChar, toStr);
        }
    }

    // NCollection_UtfIterator::GetUtf16
    internal static void AppendUtf16(uint theChar, StringBuilder toStr)
    {
        if (theChar <= UTF32_MAX_BMP)
        {
            if (theChar >= UTF16_SURROGATE_HIGH_START && theChar <= UTF16_SURROGATE_LOW_END)
            {
                return;
            }
            toStr.Append((char)theChar);
        }
        else if (theChar > UTF32_MAX_LEGAL)
        {
            // illegal
        }
        else
        {
            uint aChar = theChar - UTF16_SURROGATE_LOW_BASE;
            toStr.Append((char)((aChar >> UTF16_SURROGATE_HIGH_SHIFT) + UTF16_SURROGATE_HIGH_START));
            toStr.Append((char)((aChar & UTF16_SURROGATE_LOW_MASK) + UTF16_SURROGATE_LOW_START));
        }
    }

    /// <summary>
    /// TCollection_AsciiString(const TCollection_ExtendedString&amp;, 0) read back as Unicode: the UTF-16 text whose
    /// UTF-8 form OCCT writes. Its UTF-8 conversion skips unpaired surrogates (code points without a UTF-8 form).
    /// </summary>
    internal static string ToUtf8Equivalent(StringBuilder extString)
    {
        var result = new StringBuilder(extString.Length);
        for (int i = 0; i < extString.Length && extString[i] != '\0'; i++) // a C string: up to its first zero
        {
            char aChar = extString[i];
            if (char.IsHighSurrogate(aChar) && i + 1 < extString.Length && char.IsLowSurrogate(extString[i + 1]))
            {
                result.Append(aChar).Append(extString[i + 1]);
                i++;
            }
            else if (!char.IsSurrogate(aChar))
            {
                result.Append(aChar);
            }
        }
        return result.ToString();
    }

    #endregion

    #region Multi-byte code pages

    private static bool Issjis1(uint c) => (c >= 0x81 && c <= 0x9f) || (c >= 0xe0 && c <= 0xef);

    private static bool Issjis2(uint c) => c >= 0x40 && c <= 0xfc && c != 0x7f;

    private static bool Iseuc(uint c) => c >= 0xa1 && c <= 0xfe;

    private static bool Isshift(uint c) => c >= 0x80 && c <= 0xff;

    // tostr.AssignCat(TCollection_ExtendedString(curcar)): a zero char makes an empty string, so nothing is added
    private static void AssignCat(StringBuilder str, char theChar)
    {
        if (theChar != '\0')
        {
            str.Append(theChar);
        }
    }

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
            if (Issjis1(At(fromstr, currentstr)))
            {
                ph = At(fromstr, currentstr);
                // Be Careful with first and second !!
                currentstr++;
                pl = At(fromstr, currentstr);
                currentstr++;
                Resource_sjis_to_unicode(ref ph, ref pl);
                char curcar = (char)((ph << 8) | pl);
                AssignCat(tostr, curcar);
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
            if (Iseuc(At(fromstr, currentstr)))
            {
                ph = At(fromstr, currentstr);
                // Be Careful with first and second !!
                currentstr++;
                pl = At(fromstr, currentstr);
                currentstr++;
                Resource_euc_to_unicode(ref ph, ref pl);
                char curcar = (char)((ph << 8) | pl);
                AssignCat(tostr, curcar);
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
            if (Isshift(At(fromstr, currentstr)))
            {
                ph = At(fromstr, currentstr);
                // Be Careful with first and second !!
                currentstr++;
                pl = At(fromstr, currentstr);
                currentstr++;
                Resource_gb_to_unicode(ref ph, ref pl);
                char curcar = (char)((ph << 8) | pl);
                AssignCat(tostr, curcar);
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
            uint cur = At(fromstr, currentch);
            if (gb3 != 0x00)
            {
                if (cur < 0x30 || cur > 0x39)
                {
                    throw InsertAtZero(); // OCCT: TCollection_ExtendedString::Insert(0, ...) throws
                }
                uint codepnt = ((gb1 - 0x81) * (10 * 126 * 10)) + ((gb2 - 0x30) * (10 * 126)) + ((gb3 - 0x81) * 10) + cur - 0x30;
                if (codepnt < 23940)
                {
                    char curcar = (char)UnicodeTables.GbkUni((int)codepnt);
                    AssignCat(tostr, curcar);
                    currentch++;
                    continue; // OCCT: gb1, gb2 and gb3 stay set, so each following byte is read as a 4th byte
                }
                return false;
            }
            else if (gb2 != 0x00)
            {
                if (cur >= 0x81 && cur <= 0xFE)
                {
                    gb3 = cur;
                    currentch++;
                    continue;
                }
                throw InsertAtZero(); // OCCT: TCollection_ExtendedString::Insert(0, ...) throws
            }
            else if (gb1 != 0x00)
            {
                if (cur >= 0x30 && cur <= 0x39)
                {
                    gb2 = cur;
                    currentch++;
                    continue;
                }
                uint lead = gb1;
                uint pointer = 0;
                gb1 = 0x00;
                uint offset = cur < 0x7F ? 0x40u : 0x41u;
                if ((cur >= 0x40 && cur <= 0x7E) || (cur >= 0x80 && cur <= 0xFE))
                {
                    pointer = (lead - 0x81) * 190 + (cur - offset);
                    if (pointer < 23940)
                    {
                        char curcar = (char)UnicodeTables.GbkUni((int)pointer);
                        AssignCat(tostr, curcar);
                        currentch++;
                        continue;
                    }
                }
                if (cur <= 0x7F)
                {
                    // ASCII symbol
                    currentch++;
                    throw InsertAtZero(); // OCCT: TCollection_ExtendedString::Insert(0, ...) throws
                }
                return false;
            }
            else
            {
                if (cur <= 0x7F)
                {
                    // ASCII symbol
                    currentch++;
                    tostr.Append((char)cur);
                }
                else if (cur == 0x80)
                {
                    // Special symbol
                    tostr.Append((char)((0x20 << 8) | 0xAC));
                    currentch++;
                }
                else if (cur >= 0x81 && cur <= 0xFE)
                {
                    // Chinese symbol
                    gb1 = cur;
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
            uint cur = At(fromstr, currentch);
            if (big5lead != 0x00)
            {
                uint lead = big5lead;
                uint pointer = 0;
                big5lead = 0x00;
                uint offset = cur < 0x7F ? 0x40u : 0x62u;
                if ((cur >= 0x40 && cur <= 0x7E) || (cur >= 0xA1 && cur <= 0xFE))
                {
                    pointer = (lead - 0x81) * 157 + (cur - offset);
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
                                uint uni = UnicodeTables.Big5Uni((int)pointer);
                                if (uni <= 0xFFFF)
                                {
                                    tostr.Append((char)uni); // TCollection_ExtendedString::Insert: a zero is inserted too
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
                if (cur <= 0x7F)
                {
                    // ASCII symbol
                    currentch++;
                    throw InsertAtZero(); // OCCT: TCollection_ExtendedString::Insert(0, ...) throws
                }
                return false;
            }
            else
            {
                if (cur <= 0x7F)
                {
                    // ASCII symbol
                    currentch++;
                    tostr.Append((char)cur);
                }
                else if (cur >= 0x81 && cur <= 0xFE)
                {
                    // Chinese symbol
                    big5lead = cur;
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
    private static OutOfRangeException InsertAtZero() =>
        new("TCollection_ExtendedString::Insert : Parameter theWhere is out of range");

    // Resource_ConvertUnicode.c

    private static void JisToSjis(ref uint ph, ref uint pl)
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

    private static void EucToSjis(ref uint ph, ref uint pl)
    {
        if ((ph & 0xFFFFFF00) != 0 || (pl & 0xFFFFFF00) != 0)
        {
            ph = 0;
            pl = 0;
            return;
        }
        if (!Iseuc(ph) || !Iseuc(pl))
        {
            return;
        }
        ph &= 0x7F;
        pl &= 0x7F;
        JisToSjis(ref ph, ref pl);
    }

    internal static void Resource_sjis_to_unicode(ref uint ph, ref uint pl)
    {
        if ((ph & 0xFFFFFF00) != 0 || (pl & 0xFFFFFF00) != 0)
        {
            ph = 0;
            pl = 0;
            return;
        }
        if (!Issjis1(ph) || !Issjis2(pl))
        {
            return;
        }
        int sjis = (int)((ph << 8) | pl);
        uint uni = UnicodeTables.SjisUni(sjis);
        ph = uni >> 8;
        pl = uni & 0xFF;
    }

    internal static void Resource_euc_to_unicode(ref uint ph, ref uint pl)
    {
        if (!Iseuc(ph) || !Iseuc(pl))
        {
            return;
        }
        if (ph == 0 && pl == 0)
        {
            return;
        }
        EucToSjis(ref ph, ref pl);
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
        if (!Isshift(ph) || !Isshift(pl))
        {
            return;
        }
        ph = ph & 0x7f;
        pl = pl & 0x7f;
        int gb = (int)((ph << 8) | pl);
        uint uni = UnicodeTables.GbUni(gb);
        ph = uni >> 8;
        pl = uni & 0xFF;
    }

    #endregion
}
