// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Numerics;
using System.Text;

namespace OcctNet.DrawLite;

/// <summary>Tcl's rules for reading numbers from strings and writing them back.</summary>
public static class TclNumber
{
    private static readonly CultureInfo Invariant = CultureInfo.InvariantCulture;

    /// <summary>
    /// Reads an integer as Tcl 8.6 does: blanks around it, a sign, and decimal, 0x hex, 0o or leading-0 octal,
    /// or 0b binary digits. Integers too large for 64 bits are not integers here (Tcl would make a bignum).
    /// </summary>
    public static bool TryParseInt(string theText, out long theValue) => TryParseInt(theText.AsSpan(), out theValue);

    public static bool TryParseInt(ReadOnlySpan<char> theText, out long theValue)
    {
        theValue = 0;
        ReadOnlySpan<char> s = theText.Trim(" \t\n\r\v\f");
        if (s.IsEmpty)
        {
            return false;
        }

        bool aNegative = false;
        if (s[0] is '+' or '-')
        {
            aNegative = s[0] == '-';
            s = s[1..];
        }

        if (s.IsEmpty)
        {
            return false;
        }

        int aBase = 10;
        if (s.Length > 1 && s[0] == '0')
        {
            switch (s[1])
            {
                case 'x' or 'X':
                    aBase = 16;
                    s = s[2..];
                    break;
                case 'o' or 'O':
                    aBase = 8;
                    s = s[2..];
                    break;
                case 'b' or 'B':
                    aBase = 2;
                    s = s[2..];
                    break;
                default:
                    aBase = 8;
                    s = s[1..];
                    break;
            }

            if (s.IsEmpty)
            {
                return false;
            }
        }

        ulong aValue = 0;
        foreach (char c in s)
        {
            uint aDigit = c switch
            {
                >= '0' and <= '9' => (uint)(c - '0'),
                >= 'a' and <= 'f' => (uint)(c - 'a' + 10),
                >= 'A' and <= 'F' => (uint)(c - 'A' + 10),
                _ => 99,
            };
            if (aDigit >= aBase || aValue > (ulong.MaxValue - aDigit) / (uint)aBase)
            {
                return false;
            }

            aValue = (aValue * (uint)aBase) + aDigit;
        }

        if (aNegative)
        {
            if (aValue > 1UL << 63)
            {
                return false;
            }

            theValue = aValue == 1UL << 63 ? long.MinValue : -(long)aValue;
            return true;
        }

        if (aValue > long.MaxValue)
        {
            return false;
        }

        theValue = (long)aValue;
        return true;
    }

    /// <summary>
    /// Reads a floating-point number (or an integer, as a double) as Tcl does: blanks around it, Inf and NaN.
    /// A leading-zero integer with an 8 or 9 in it is an invalid octal number, so not a number at all.
    /// </summary>
    public static bool TryParseDouble(string theText, out double theValue)
    {
        if (TryParseInt(theText, out long anInt))
        {
            theValue = anInt;
            return true;
        }

        ReadOnlySpan<char> s = theText.AsSpan().Trim(" \t\n\r\v\f");
        theValue = 0;
        if (s.IsEmpty || IsBadOctal(s))
        {
            return false;
        }

        // Only Tcl's own syntax: digits, '.', exponent; .NET would also accept "1,0" with some styles
        foreach (char c in s)
        {
            if (!(char.IsAsciiDigit(c) || c is '.' or 'e' or 'E' or '+' or '-' || char.IsAsciiLetter(c)))
            {
                return false;
            }
        }

        if (double.TryParse(s, NumberStyles.Float, Invariant, out theValue))
        {
            return true;
        }

        string aLower = s.ToString().ToLowerInvariant().TrimStart('+');
        switch (aLower)
        {
            case "inf" or "infinity":
                theValue = double.PositiveInfinity;
                return true;
            case "-inf" or "-infinity":
                theValue = double.NegativeInfinity;
                return true;
            case "nan" or "-nan":
                theValue = double.NaN;
                return true;
        }

        // An integer too large for 64 bits
        if (BigInteger.TryParse(s, NumberStyles.AllowLeadingSign, Invariant, out BigInteger aBig))
        {
            theValue = (double)aBig;
            return true;
        }

        return false;
    }

    private static bool IsBadOctal(ReadOnlySpan<char> s)
    {
        if (s[0] is '+' or '-')
        {
            s = s[1..];
        }

        if (s.Length < 2 || s[0] != '0')
        {
            return false;
        }

        foreach (char c in s)
        {
            if (!char.IsAsciiDigit(c))
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>True when Tcl would read the text as a number (integer or floating-point).</summary>
    public static bool IsNumber(string theText) => TryParseDouble(theText, out _);

    /// <summary>
    /// Writes a double as Tcl 8.6 does with tcl_precision 0: the shortest string that reads back to the same
    /// value, in fixed notation with at least one fractional digit for exponents -4..16, else as 1.5e+17.
    /// </summary>
    public static string FormatDouble(double theValue)
    {
        if (double.IsNaN(theValue))
        {
            return "NaN";
        }

        if (double.IsInfinity(theValue))
        {
            return theValue > 0 ? "Inf" : "-Inf";
        }

        string aRoundTrip = theValue.ToString("R", Invariant);
        bool aNegative = aRoundTrip.StartsWith('-');
        if (aNegative)
        {
            aRoundTrip = aRoundTrip[1..];
        }

        int anE = aRoundTrip.IndexOfAny(['E', 'e']);
        int anExponent10 = anE < 0 ? 0 : int.Parse(aRoundTrip.AsSpan(anE + 1), NumberStyles.AllowLeadingSign, Invariant);
        string aMantissa = anE < 0 ? aRoundTrip : aRoundTrip[..anE];
        int aDot = aMantissa.IndexOf('.');
        int anIntDigits = aDot < 0 ? aMantissa.Length : aDot;
        string aDigits = aMantissa.Replace(".", "");
        int anExponent = anIntDigits - 1 + anExponent10;
        int aLead = 0;
        while (aLead < aDigits.Length - 1 && aDigits[aLead] == '0')
        {
            aLead++;
            anExponent--;
        }

        aDigits = aDigits[aLead..].TrimEnd('0');
        if (aDigits.Length == 0)
        {
            aDigits = "0";
            anExponent = 0;
        }

        var aBuilder = new StringBuilder(32);
        if (aNegative)
        {
            aBuilder.Append('-');
        }

        if (anExponent < -4 || anExponent > 16)
        {
            aBuilder.Append(aDigits[0]);
            if (aDigits.Length > 1)
            {
                aBuilder.Append('.').Append(aDigits, 1, aDigits.Length - 1);
            }

            aBuilder.Append('e').Append(anExponent < 0 ? '-' : '+').Append(Math.Abs(anExponent).ToString(Invariant));
            return aBuilder.ToString();
        }

        if (anExponent < 0)
        {
            aBuilder.Append("0.").Append('0', -anExponent - 1).Append(aDigits);
            return aBuilder.ToString();
        }

        if (aDigits.Length <= anExponent + 1)
        {
            aBuilder.Append(aDigits).Append('0', anExponent + 1 - aDigits.Length).Append(".0");
        }
        else
        {
            aBuilder.Append(aDigits, 0, anExponent + 1).Append('.').Append(aDigits, anExponent + 1, aDigits.Length - anExponent - 1);
        }

        return aBuilder.ToString();
    }

    public static string FormatInt(long theValue) => theValue.ToString(Invariant);

    /// <summary>Reads a Tcl boolean: a number (non-zero is true) or yes/no, true/false, on/off and their prefixes.</summary>
    public static bool TryParseBoolean(string theText, out bool theValue)
    {
        theValue = false;
        if (TryParseInt(theText, out long anInt))
        {
            theValue = anInt != 0;
            return true;
        }

        if (TryParseDouble(theText, out double aDouble))
        {
            theValue = aDouble != 0;
            return true;
        }

        string s = theText.Trim().ToLowerInvariant();
        if (s.Length == 0)
        {
            return false;
        }

        foreach ((string aWord, bool aMeaning, int aMinLength) in s_booleans)
        {
            if (s.Length >= aMinLength && aWord.StartsWith(s, StringComparison.Ordinal))
            {
                theValue = aMeaning;
                return true;
            }
        }

        return false;
    }

    private static readonly (string, bool, int)[] s_booleans =
        [("true", true, 1), ("false", false, 1), ("yes", true, 1), ("no", false, 1), ("on", true, 2), ("off", false, 2)];

    /// <summary>A boolean, or Tcl's error for a value that isn't one.</summary>
    public static bool GetBoolean(string theText)
        => TryParseBoolean(theText, out bool aValue) ? aValue : throw new DrawException($"expected boolean value but got \"{theText}\"");

    /// <summary>An integer, or Tcl's error for a value that isn't one.</summary>
    public static long GetInt(string theText)
        => TryParseInt(theText, out long aValue) ? aValue : throw new DrawException($"expected integer but got \"{theText}\"");

    /// <summary>A double, or Tcl's error for a value that isn't one.</summary>
    public static double GetDouble(string theText)
        => TryParseDouble(theText, out double aValue) ? aValue : throw new DrawException($"expected floating-point number but got \"{theText}\"");
}
