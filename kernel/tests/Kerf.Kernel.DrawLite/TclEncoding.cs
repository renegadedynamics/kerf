// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// Tcl's encodings (the <c>encoding</c> command and the channels' <c>-encoding</c>), over .NET's encodings.
/// </summary>
/// <remarks>
/// As in Tcl, a byte string is a string of one character per byte (a character above 0xFF gives its low byte); the
/// file overlay keeps files that way (<see cref="TclFileSystem"/>). The system encoding is utf-8; utf-8 is decoded as
/// Tcl 8.6 decodes it: a byte that doesn't begin a valid sequence stands for itself (the ISO 8859-1 character).
/// Characters an encoding can't represent are written as '?', as Tcl writes them.
/// </remarks>
internal static class TclEncoding
{
    /// <summary>Tcl's system encoding in Draw-lite.</summary>
    public const string System = "utf-8";

    // Tcl's names, mapped to .NET's code pages (0: handled here)
    private static readonly Dictionary<string, int> s_codePages = BuildNames();

    private static Dictionary<string, int> BuildNames()
    {
        var aNames = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase)
        {
            ["utf-8"] = 0, ["unicode"] = 0, ["binary"] = 0, ["identity"] = 0, ["iso8859-1"] = 28591, ["ascii"] = 20127,
            ["cp437"] = 437, ["cp737"] = 737, ["cp775"] = 775, ["cp850"] = 850, ["cp852"] = 852, ["cp855"] = 855,
            ["cp857"] = 857, ["cp860"] = 860, ["cp861"] = 861, ["cp862"] = 862, ["cp863"] = 863, ["cp864"] = 864,
            ["cp865"] = 865, ["cp866"] = 866, ["cp869"] = 869, ["cp874"] = 874, ["cp932"] = 932, ["cp936"] = 936,
            ["cp949"] = 949, ["cp950"] = 950, ["shiftjis"] = 932, ["gb2312"] = 936, ["euc-cn"] = 936, ["big5"] = 950,
            ["euc-kr"] = 51949, ["euc-jp"] = 20932, ["koi8-r"] = 20866, ["koi8-u"] = 21866, ["macRoman"] = 10000,
            ["macCyrillic"] = 10007, ["macGreek"] = 10006, ["macCentEuro"] = 10029, ["macTurkish"] = 10081,
            ["macIceland"] = 10079, ["macCroatian"] = 10082, ["macRomania"] = 10010, ["macUkraine"] = 10017,
            ["macThai"] = 10021, ["macJapan"] = 10001,
        };
        for (int i = 2; i <= 16; i++)
        {
            if (i != 12)
            {
                aNames[$"iso8859-{i}"] = 28590 + i;
            }
        }

        for (int i = 1250; i <= 1258; i++)
        {
            aNames[$"cp{i}"] = i;
        }

        return aNames;
    }

    /// <summary>The names <c>encoding names</c> lists.</summary>
    public static string Names => string.Join(' ', s_codePages.Keys.Order(StringComparer.Ordinal));

    /// <summary>Checks a name, as Tcl's "unknown encoding" error.</summary>
    public static string Check(string theName)
        => s_codePages.ContainsKey(theName) ? theName : throw new DrawException($"unknown encoding \"{theName}\"");

    private static Encoding Get(string theName)
    {
        int aCodePage = s_codePages.TryGetValue(theName, out int aPage) ? aPage : throw new DrawException($"unknown encoding \"{theName}\"");
        if (theName.Equals("unicode", StringComparison.OrdinalIgnoreCase))
        {
            return Encoding.Unicode; // Tcl's "unicode": UTF-16 in the machine's order (little-endian here)
        }

        if (aCodePage == 0)
        {
            return Encoding.UTF8;
        }

        return CodePagesEncodingProvider.Instance.GetEncoding(aCodePage, new EncoderReplacementFallback("?"), new DecoderReplacementFallback("?"))
               ?? Encoding.GetEncoding(aCodePage, new EncoderReplacementFallback("?"), new DecoderReplacementFallback("?"));
    }

    private static bool IsBinary(string theName) => theName.Equals("binary", StringComparison.OrdinalIgnoreCase)
                                                     || theName.Equals("identity", StringComparison.OrdinalIgnoreCase);

    /// <summary>The bytes of a byte string (a character per byte, its low byte).</summary>
    public static byte[] Bytes(string theByteString)
    {
        var aBytes = new byte[theByteString.Length];
        for (int i = 0; i < theByteString.Length; i++)
        {
            aBytes[i] = unchecked((byte)theByteString[i]);
        }

        return aBytes;
    }

    /// <summary>A byte string of bytes.</summary>
    public static string ByteString(ReadOnlySpan<byte> theBytes) => Encoding.Latin1.GetString(theBytes);

    /// <summary><c>encoding convertto</c>: a string's bytes in an encoding, as a byte string.</summary>
    public static string ConvertTo(string theName, string theText)
        => IsBinary(Check(theName)) ? ByteString(Bytes(theText)) : ByteString(Get(theName).GetBytes(theText));

    /// <summary><c>encoding convertfrom</c>: a byte string decoded from an encoding.</summary>
    public static string ConvertFrom(string theName, string theByteString)
    {
        Check(theName);
        if (IsBinary(theName))
        {
            return ByteString(Bytes(theByteString));
        }

        byte[] aBytes = Bytes(theByteString);
        return theName.Equals("utf-8", StringComparison.OrdinalIgnoreCase) ? DecodeUtf8(aBytes) : Get(theName).GetString(aBytes);
    }

    /// <summary>UTF-8 decoded as Tcl 8.6 decodes it: a byte that doesn't begin a valid sequence stands for itself.</summary>
    public static string DecodeUtf8(ReadOnlySpan<byte> theBytes)
    {
        var aText = new StringBuilder(theBytes.Length);
        int i = 0;
        while (i < theBytes.Length)
        {
            if (Rune.DecodeFromUtf8(theBytes[i..], out Rune aRune, out int aLength) == global::System.Buffers.OperationStatus.Done)
            {
                aText.Append(aRune.ToString());
                i += aLength;
            }
            else
            {
                aText.Append((char)theBytes[i]); // not a valid sequence: the byte stands for itself
                i++;
            }
        }

        return aText.ToString();
    }

    /// <summary>A channel's text written in its encoding, as a byte string.</summary>
    public static string Encode(string theName, string theText)
        => theName.Equals("utf-8", StringComparison.OrdinalIgnoreCase) && IsAscii(theText) ? theText : ConvertTo(theName, theText);

    /// <summary>A channel's bytes (a byte string) read in its encoding.</summary>
    public static string Decode(string theName, string theByteString)
        => theName.Equals("utf-8", StringComparison.OrdinalIgnoreCase) && IsAscii(theByteString) ? theByteString : ConvertFrom(theName, theByteString);

    private static bool IsAscii(string theText)
    {
        foreach (char aChar in theText)
        {
            if (aChar >= 0x80)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// <c>binary format formatString ?arg ...?</c>: a byte string of the values (a A for strings; c s S t i I n w W m for
    /// integers; f r R d q Q for floats; x for a zero byte), each with a count or <c>*</c> (a list's elements); the
    /// other field types and <c>binary scan</c> aren't supported.
    /// </summary>
    public static string BinaryCommand(IReadOnlyList<string> a)
    {
        if (a.Count < 3 || a[1] != "format")
        {
            throw new DrawException(a.Count >= 2 && a[1] is "scan" or "encode" or "decode"
                                        ? $"binary {a[1]}: not supported by Draw-lite"
                                        : "wrong # args: should be \"binary format formatString ?arg ...?\"");
        }

        string aFormat = a[2];
        int anArg = 3;
        var aBytes = new List<byte>();
        int i = 0;
        while (i < aFormat.Length)
        {
            char aType = aFormat[i++];
            if (char.IsWhiteSpace(aType))
            {
                continue;
            }

            int aCount = -1; // no count given
            bool isStar = false;
            if (i < aFormat.Length && aFormat[i] == '*')
            {
                isStar = true;
                i++;
            }
            else
            {
                int aStart = i;
                while (i < aFormat.Length && char.IsAsciiDigit(aFormat[i]))
                {
                    i++;
                }

                if (i > aStart)
                {
                    aCount = int.Parse(aFormat.AsSpan(aStart, i - aStart), CultureInfo.InvariantCulture);
                }
            }

            if (aType == 'x')
            {
                aBytes.AddRange(new byte[aCount < 0 ? 1 : aCount]);
                continue;
            }

            if (anArg >= a.Count)
            {
                throw new DrawException("not enough arguments for all format specifiers");
            }

            string aValue = a[anArg++];
            if (aType is 'a' or 'A')
            {
                byte[] aText = Bytes(aValue);
                int aLength = isStar ? aText.Length : aCount < 0 ? 1 : aCount;
                for (int k = 0; k < aLength; k++)
                {
                    aBytes.Add(k < aText.Length ? aText[k] : (byte)(aType == 'a' ? 0 : ' '));
                }

                continue;
            }

            // a count (or *) takes the elements of a list, no count a single value
            IReadOnlyList<string> aValues = aCount < 0 && !isStar ? [aValue] : TclList.Parse(aValue);
            int aNb = isStar ? aValues.Count : aCount < 0 ? 1 : aCount;
            if (aNb > aValues.Count)
            {
                throw new DrawException("number of elements in list does not match count");
            }

            for (int k = 0; k < aNb; k++)
            {
                string anElement = aValues[k];
                switch (aType)
                {
                    case 'c':
                        aBytes.Add(unchecked((byte)TclNumber.GetInt(anElement)));
                        break;
                    case 's' or 'S' or 't':
                    {
                        var aShort = unchecked((short)TclNumber.GetInt(anElement));
                        byte[] aRaw = BitConverter.GetBytes(aShort);
                        aBytes.AddRange(aType == 'S' ? aRaw.Reverse() : aRaw);
                        break;
                    }

                    case 'i' or 'I' or 'n':
                    {
                        byte[] aRaw = BitConverter.GetBytes(unchecked((int)TclNumber.GetInt(anElement)));
                        aBytes.AddRange(aType == 'I' ? aRaw.Reverse() : aRaw);
                        break;
                    }

                    case 'w' or 'W' or 'm':
                    {
                        byte[] aRaw = BitConverter.GetBytes(TclNumber.GetInt(anElement));
                        aBytes.AddRange(aType == 'W' ? aRaw.Reverse() : aRaw);
                        break;
                    }

                    case 'f' or 'r' or 'R':
                    {
                        byte[] aRaw = BitConverter.GetBytes((float)TclNumber.GetDouble(anElement));
                        aBytes.AddRange(aType == 'R' ? aRaw.Reverse() : aRaw);
                        break;
                    }

                    case 'd' or 'q' or 'Q':
                    {
                        byte[] aRaw = BitConverter.GetBytes(TclNumber.GetDouble(anElement));
                        aBytes.AddRange(aType == 'Q' ? aRaw.Reverse() : aRaw);
                        break;
                    }

                    default:
                        throw new DrawException($"bad field specifier \"{aType}\"");
                }
            }
        }

        return ByteString(aBytes.ToArray());
    }

    /// <summary>The <c>encoding</c> command: convertfrom, convertto, names, system.</summary>
    public static string Command(IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            throw TclControlCommands.Usage("encoding option ?arg ...?");
        }

        switch (a[1])
        {
            case "system":
                if (a.Count > 2)
                {
                    Check(a[2]); // the system encoding stays utf-8
                }

                return a.Count == 2 ? System : "";
            case "names":
                return Names;
            case "convertto" or "convertfrom":
            {
                if (a.Count is < 3 or > 4)
                {
                    throw TclControlCommands.Usage($"encoding {a[1]} ?encoding? data");
                }

                string anEncoding = a.Count == 4 ? a[2] : System;
                return a[1] == "convertto" ? ConvertTo(anEncoding, a[^1]) : ConvertFrom(anEncoding, a[^1]);
            }

            case "dirs":
                return "";
            default:
                throw new DrawException(string.Create(CultureInfo.InvariantCulture,
                                                      $"bad option \"{a[1]}\": must be convertfrom, convertto, dirs, names, or system"));
        }
    }
}
