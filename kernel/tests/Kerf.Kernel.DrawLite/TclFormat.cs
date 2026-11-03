// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;

namespace Kerf.Kernel.DrawLite;

/// <summary>Tcl's format and scan: C's printf and scanf conversions over Tcl values.</summary>
public static class TclFormat
{
    private static readonly CultureInfo Invariant = CultureInfo.InvariantCulture;

    /// <summary>
    /// format: %d %i %u %o %x %X %b %c %s %f %e %E %g %G %a and %%, with XPG positions (%2$s), the flags
    /// - + space 0 #, width and precision (or *), and the size modifiers h l ll (integers are 64-bit, as Tcl on
    /// 64-bit Linux).
    /// </summary>
    public static string Format(string theFormat, IReadOnlyList<string> theArgs)
    {
        var aBuilder = new StringBuilder();
        int anArg = 0;
        int i = 0;
        while (i < theFormat.Length)
        {
            char c = theFormat[i++];
            if (c != '%')
            {
                aBuilder.Append(c);
                continue;
            }

            if (i < theFormat.Length && theFormat[i] == '%')
            {
                aBuilder.Append('%');
                i++;
                continue;
            }

            // XPG position: %n$
            int aSave = i;
            int aPosition = ReadDigits(theFormat, ref i);
            if (aPosition > 0 && i < theFormat.Length && theFormat[i] == '$')
            {
                anArg = aPosition - 1;
                i++;
            }
            else
            {
                i = aSave;
            }

            bool aLeft = false, aPlus = false, aSpace = false, aZero = false, anAlternate = false;
            for (; i < theFormat.Length; i++)
            {
                switch (theFormat[i])
                {
                    case '-':
                        aLeft = true;
                        continue;
                    case '+':
                        aPlus = true;
                        continue;
                    case ' ':
                        aSpace = true;
                        continue;
                    case '0':
                        aZero = true;
                        continue;
                    case '#':
                        anAlternate = true;
                        continue;
                }

                break;
            }

            int aWidth = 0;
            if (i < theFormat.Length && theFormat[i] == '*')
            {
                aWidth = (int)TclNumber.GetInt(NextArg(theArgs, ref anArg));
                if (aWidth < 0)
                {
                    aLeft = true;
                    aWidth = -aWidth;
                }

                i++;
            }
            else
            {
                aWidth = ReadDigits(theFormat, ref i);
            }

            int aPrecision = -1;
            if (i < theFormat.Length && theFormat[i] == '.')
            {
                i++;
                if (i < theFormat.Length && theFormat[i] == '*')
                {
                    aPrecision = (int)TclNumber.GetInt(NextArg(theArgs, ref anArg));
                    i++;
                }
                else
                {
                    aPrecision = ReadDigits(theFormat, ref i);
                }
            }

            int aSize = 0;
            while (i < theFormat.Length && theFormat[i] is 'h' or 'l' or 'L' or 'q' or 'j' or 'z' or 't')
            {
                aSize = theFormat[i] == 'h' ? -1 : aSize + 1;
                i++;
            }

            if (i >= theFormat.Length)
            {
                throw new DrawException("format string ended in middle of field specifier");
            }

            char aConversion = theFormat[i++];
            string aBody;
            string aSign = "";
            string aPrefix = "";
            bool aNumeric = true;
            switch (aConversion)
            {
                case 'd' or 'i':
                {
                    long v = Truncate(GetInteger(NextArg(theArgs, ref anArg)), aSize);
                    aBody = v == long.MinValue ? "9223372036854775808" : Math.Abs(v).ToString(Invariant);
                    aSign = v < 0 ? "-" : aPlus ? "+" : aSpace ? " " : "";
                    if (aPrecision >= 0)
                    {
                        aBody = aBody.PadLeft(aPrecision, '0');
                        aZero = false;
                    }

                    break;
                }

                case 'u' or 'o' or 'x' or 'X' or 'b':
                {
                    ulong v = Unsigned(Truncate(GetInteger(NextArg(theArgs, ref anArg)), aSize), aSize);
                    aBody = aConversion switch
                    {
                        'u' => v.ToString(Invariant),
                        'o' => Convert.ToString((long)v, 8),
                        'x' => v.ToString("x", Invariant),
                        'X' => v.ToString("X", Invariant),
                        _ => Convert.ToString((long)v, 2),
                    };
                    if (aPrecision >= 0)
                    {
                        aBody = aBody.PadLeft(aPrecision, '0');
                        aZero = false;
                    }

                    if (anAlternate && v != 0)
                    {
                        aPrefix = aConversion switch
                        {
                            'o' => "0",
                            'x' => "0x",
                            'X' => "0X",
                            'b' => "0b",
                            _ => "",
                        };
                    }

                    break;
                }

                case 'c':
                    aBody = char.ConvertFromUtf32((int)GetInteger(NextArg(theArgs, ref anArg)));
                    aNumeric = false;
                    break;
                case 's':
                    aBody = NextArg(theArgs, ref anArg);
                    if (aPrecision >= 0 && aPrecision < aBody.Length)
                    {
                        aBody = aBody[..aPrecision];
                    }

                    aNumeric = false;
                    break;
                case 'f' or 'F' or 'e' or 'E' or 'g' or 'G' or 'a' or 'A':
                {
                    double v = TclNumber.GetDouble(NextArg(theArgs, ref anArg));
                    aSign = v < 0 || (v == 0 && double.IsNegative(v)) ? "-" : aPlus ? "+" : aSpace ? " " : "";
                    aBody = FormatFloat(Math.Abs(v), aConversion, aPrecision < 0 ? 6 : aPrecision, anAlternate);
                    if (double.IsNaN(v) || double.IsInfinity(v))
                    {
                        aZero = false;
                    }

                    break;
                }

                default:
                    throw new DrawException($"bad field specifier \"{aConversion}\"");
            }

            int aLength = aSign.Length + aPrefix.Length + aBody.Length;
            if (aLength >= aWidth)
            {
                aBuilder.Append(aSign).Append(aPrefix).Append(aBody);
            }
            else if (aLeft)
            {
                aBuilder.Append(aSign).Append(aPrefix).Append(aBody).Append(' ', aWidth - aLength);
            }
            else if (aZero && aNumeric)
            {
                aBuilder.Append(aSign).Append(aPrefix).Append('0', aWidth - aLength).Append(aBody);
            }
            else
            {
                aBuilder.Append(' ', aWidth - aLength).Append(aSign).Append(aPrefix).Append(aBody);
            }
        }

        return aBuilder.ToString();
    }

    private static string NextArg(IReadOnlyList<string> theArgs, ref int theIndex)
        => theIndex < theArgs.Count ? theArgs[theIndex++] : throw new DrawException("not enough arguments for all format specifiers");

    private static int ReadDigits(string s, ref int i)
    {
        int aValue = 0;
        while (i < s.Length && char.IsAsciiDigit(s[i]))
        {
            aValue = (aValue * 10) + (s[i++] - '0');
        }

        return aValue;
    }

    /// <summary>An integer argument; like Tcl, a double (even 2.0) is an error.</summary>
    private static long GetInteger(string theText) => TclNumber.GetInt(theText);

    private static long Truncate(long theValue, int theSize) => theSize < 0 ? (short)theValue : theValue;

    private static ulong Unsigned(long theValue, int theSize) => theSize < 0 ? (ushort)theValue : unchecked((ulong)theValue);

    /// <summary>A non-negative double in C's %f, %e or %g notation.</summary>
    internal static string FormatFloat(double v, char theConversion, int thePrecision, bool theAlternate)
    {
        if (double.IsNaN(v))
        {
            return "NaN";
        }

        if (double.IsInfinity(v))
        {
            return "Inf";
        }

        switch (theConversion)
        {
            case 'f' or 'F':
            {
                string s = v.ToString("F" + thePrecision.ToString(Invariant), Invariant);
                return theAlternate && thePrecision == 0 ? s + "." : s;
            }

            case 'e' or 'E':
                return Exponential(v, thePrecision, theConversion, theAlternate);
            case 'a' or 'A':
                return Exponential(v, thePrecision, theConversion == 'a' ? 'e' : 'E', theAlternate);
            default:
            {
                int p = thePrecision == 0 ? 1 : thePrecision;
                int anExponent = v == 0 ? 0 : DecimalExponent(v, p);
                string s;
                if (anExponent < p && anExponent >= -4)
                {
                    s = v.ToString("F" + (p - 1 - anExponent).ToString(Invariant), Invariant);
                    if (!theAlternate && s.Contains('.'))
                    {
                        s = s.TrimEnd('0').TrimEnd('.');
                    }
                    else if (theAlternate && !s.Contains('.'))
                    {
                        s += ".";
                    }
                }
                else
                {
                    s = Exponential(v, p - 1, theConversion == 'g' ? 'e' : 'E', theAlternate);
                    if (!theAlternate)
                    {
                        int anE = s.IndexOfAny(['e', 'E']);
                        string aMantissa = s[..anE];
                        if (aMantissa.Contains('.'))
                        {
                            aMantissa = aMantissa.TrimEnd('0').TrimEnd('.');
                        }

                        s = aMantissa + s[anE..];
                    }
                }

                return s;
            }
        }
    }

    /// <summary>The exponent of v when written with <paramref name="theDigits"/> significant digits (rounding may carry).</summary>
    private static int DecimalExponent(double v, int theDigits)
    {
        string s = v.ToString("E" + (theDigits - 1).ToString(Invariant), Invariant);
        return int.Parse(s.AsSpan(s.IndexOf('E') + 1), NumberStyles.AllowLeadingSign, Invariant);
    }

    private static string Exponential(double v, int thePrecision, char theLetter, bool theAlternate)
    {
        string s = v.ToString("E" + thePrecision.ToString(Invariant), Invariant);
        int anE = s.IndexOf('E');
        string aMantissa = s[..anE];
        int anExponent = int.Parse(s.AsSpan(anE + 1), NumberStyles.AllowLeadingSign, Invariant);
        if (theAlternate && !aMantissa.Contains('.'))
        {
            aMantissa += ".";
        }

        return $"{aMantissa}{theLetter}{(anExponent < 0 ? '-' : '+')}{Math.Abs(anExponent).ToString("00", Invariant)}";
    }

    /// <summary>
    /// scan: the values converted (null for fields not reached), how many were converted, and whether the input
    /// ended before the first conversion. Conversions: %d %i %o %x %u %c %s %f %e %g %[chars] %n, with %*
    /// suppression, widths and XPG positions.
    /// </summary>
    public static (List<string?> Values, int Converted, bool InputEnded) Scan(string theInput, string theFormat)
    {
        var aValues = new List<string?>();
        int aConverted = 0;
        int aNext = 0;
        int p = 0;
        int s = 0;
        bool anInputEnded = false;
        while (p < theFormat.Length)
        {
            char f = theFormat[p];
            if (char.IsWhiteSpace(f))
            {
                while (s < theInput.Length && char.IsWhiteSpace(theInput[s]))
                {
                    s++;
                }

                p++;
                continue;
            }

            if (f != '%' || (p + 1 < theFormat.Length && theFormat[p + 1] == '%'))
            {
                int aLiteral = f == '%' ? 2 : 1;
                if (s >= theInput.Length)
                {
                    anInputEnded = true;
                    break;
                }

                if (theInput[s] != f)
                {
                    break;
                }

                s++;
                p += aLiteral;
                continue;
            }

            p++;
            bool aSuppress = false;
            if (p < theFormat.Length && theFormat[p] == '*')
            {
                aSuppress = true;
                p++;
            }

            int aSave = p;
            int aPosition = ReadDigits(theFormat, ref p);
            int aSlot = aNext;
            if (aPosition > 0 && p < theFormat.Length && theFormat[p] == '$')
            {
                aSlot = aPosition - 1;
                p++;
            }
            else
            {
                p = aSave;
            }

            int aWidth = ReadDigits(theFormat, ref p);
            while (p < theFormat.Length && theFormat[p] is 'h' or 'l' or 'L')
            {
                p++;
            }

            if (p >= theFormat.Length)
            {
                throw new DrawException("unfinished format specifier");
            }

            char aConversion = theFormat[p++];
            string? aSet = null;
            if (aConversion == '[')
            {
                int anEnd = theFormat.IndexOf(']', p + (theFormat[p] == ']' ? 1 : theFormat[p] == '^' && p + 1 < theFormat.Length && theFormat[p + 1] == ']' ? 2 : 0));
                if (anEnd < 0)
                {
                    throw new DrawException("unmatched [ in format string");
                }

                aSet = theFormat[p..anEnd];
                p = anEnd + 1;
            }

            if (aConversion is not ('c' or '[' or 'n'))
            {
                while (s < theInput.Length && char.IsWhiteSpace(theInput[s]))
                {
                    s++;
                }
            }

            int aLimit = aWidth > 0 ? Math.Min(theInput.Length, s + aWidth) : theInput.Length;
            string? aValue;
            if (aConversion == 'n')
            {
                aValue = s.ToString(Invariant);
            }
            else if (s >= theInput.Length)
            {
                anInputEnded = true;
                if (!aSuppress)
                {
                    SetSlot(aValues, aSlot, null);
                    aNext = aSlot + 1;
                }

                break;
            }
            else
            {
                aValue = aConversion switch
                {
                    'd' or 'u' => ScanInteger(theInput, ref s, aLimit, 10),
                    'i' => ScanInteger(theInput, ref s, aLimit, 0),
                    'o' => ScanInteger(theInput, ref s, aLimit, 8),
                    'x' or 'X' => ScanInteger(theInput, ref s, aLimit, 16),
                    'b' => ScanInteger(theInput, ref s, aLimit, 2),
                    'c' => ((int)theInput[s++]).ToString(Invariant),
                    's' => ScanWhile(theInput, ref s, aLimit, static c => !char.IsWhiteSpace(c)),
                    'f' or 'e' or 'g' or 'E' or 'G' => ScanDouble(theInput, ref s, aLimit),
                    '[' => ScanWhile(theInput, ref s, aLimit, InSet(aSet!)),
                    _ => throw new DrawException($"bad scan conversion character \"{aConversion}\""),
                };
            }

            if (aValue is null)
            {
                break;
            }

            if (!aSuppress)
            {
                SetSlot(aValues, aSlot, aValue);
                aNext = aSlot + 1;
                if (aConversion != 'n')
                {
                    aConverted++;
                }
            }
        }

        // Fields not reached still count as fields (they are empty in the inline result)
        int aFields = CountFields(theFormat);
        while (aValues.Count < aFields)
        {
            aValues.Add(null);
        }

        return (aValues, aConverted, anInputEnded && aConverted == 0);
    }

    private static int CountFields(string theFormat)
    {
        int aCount = 0;
        for (int i = 0; i < theFormat.Length; i++)
        {
            if (theFormat[i] != '%')
            {
                continue;
            }

            if (i + 1 < theFormat.Length && theFormat[i + 1] == '%')
            {
                i++;
                continue;
            }

            if (i + 1 < theFormat.Length && theFormat[i + 1] == '*')
            {
                continue;
            }

            aCount++;
        }

        return aCount;
    }

    private static void SetSlot(List<string?> theValues, int theSlot, string? theValue)
    {
        while (theValues.Count <= theSlot)
        {
            theValues.Add(null);
        }

        theValues[theSlot] = theValue;
    }

    private static Func<char, bool> InSet(string theSet)
    {
        bool aNegate = theSet.StartsWith('^');
        string aSet = aNegate ? theSet[1..] : theSet;
        return c =>
        {
            bool aIn = false;
            for (int i = 0; i < aSet.Length; i++)
            {
                if (i + 2 < aSet.Length && aSet[i + 1] == '-')
                {
                    if (c >= aSet[i] && c <= aSet[i + 2])
                    {
                        aIn = true;
                    }

                    i += 2;
                }
                else if (aSet[i] == c)
                {
                    aIn = true;
                }
            }

            return aIn != aNegate;
        };
    }

    private static string? ScanWhile(string s, ref int i, int theLimit, Func<char, bool> thePredicate)
    {
        int aStart = i;
        while (i < theLimit && thePredicate(s[i]))
        {
            i++;
        }

        return i == aStart ? null : s[aStart..i];
    }

    private static string? ScanInteger(string s, ref int i, int theLimit, int theBase)
    {
        int aStart = i;
        bool aNegative = false;
        if (i < theLimit && s[i] is '+' or '-')
        {
            aNegative = s[i] == '-';
            i++;
        }

        if (theBase == 0)
        {
            theBase = 10;
            if (i + 1 < theLimit && s[i] == '0' && s[i + 1] is 'x' or 'X')
            {
                theBase = 16;
                i += 2;
            }
            else if (i < theLimit && s[i] == '0')
            {
                theBase = 8;
            }
        }
        else if (theBase == 16 && i + 1 < theLimit && s[i] == '0' && s[i + 1] is 'x' or 'X')
        {
            i += 2;
        }

        int aDigitsStart = i;
        long aValue = 0;
        while (i < theLimit)
        {
            int aDigit = char.IsAsciiDigit(s[i]) ? s[i] - '0' : char.IsAsciiLetter(s[i]) ? char.ToLowerInvariant(s[i]) - 'a' + 10 : 99;
            if (aDigit >= theBase)
            {
                break;
            }

            aValue = unchecked((aValue * theBase) + aDigit);
            i++;
        }

        if (i == aDigitsStart)
        {
            i = aStart;
            return null;
        }

        return (aNegative ? -aValue : aValue).ToString(Invariant);
    }

    private static string? ScanDouble(string s, ref int i, int theLimit)
    {
        int aStart = i;
        if (i < theLimit && s[i] is '+' or '-')
        {
            i++;
        }

        int aDigits = 0;
        while (i < theLimit && char.IsAsciiDigit(s[i]))
        {
            i++;
            aDigits++;
        }

        if (i < theLimit && s[i] == '.')
        {
            i++;
            while (i < theLimit && char.IsAsciiDigit(s[i]))
            {
                i++;
                aDigits++;
            }
        }

        if (aDigits == 0)
        {
            i = aStart;
            return null;
        }

        if (i < theLimit && s[i] is 'e' or 'E')
        {
            int aSave = i++;
            if (i < theLimit && s[i] is '+' or '-')
            {
                i++;
            }

            if (i < theLimit && char.IsAsciiDigit(s[i]))
            {
                while (i < theLimit && char.IsAsciiDigit(s[i]))
                {
                    i++;
                }
            }
            else
            {
                i = aSave;
            }
        }

        return TclNumber.FormatDouble(double.Parse(s.AsSpan(aStart, i - aStart), NumberStyles.Float, Invariant));
    }
}
