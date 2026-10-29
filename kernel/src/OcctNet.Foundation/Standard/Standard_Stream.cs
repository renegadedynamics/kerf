// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Strtod follows OCCT's Strtod (src/FoundationClasses/TKernel/Standard/Standard_CString.hxx), the C library's strtod
// in the C locale. Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.

using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text;

namespace Occt;

/// <summary>
/// The behaviour of OCCT's C++ streams (<c>Standard_OStream</c>, <c>Standard_IStream</c>) on .NET's
/// <see cref="TextWriter"/> and <see cref="TextReader"/>, for the ports that write and read OCCT's text formats
/// (GeomTools, and BRepTools later).
/// </summary>
/// <remarks>
/// <para>
/// A C++ stream carries state that a .NET writer or reader doesn't: the output precision (<c>OS.precision(17)</c>),
/// and the input's fail and end-of-file flags. This class keeps that state per writer and per reader, in a weak
/// table, so ported code keeps OCCT's signatures (a <see cref="TextWriter"/> where OCCT takes a
/// <c>Standard_OStream&amp;</c>) and several calls on one stream share its state, as in C++.
/// </para>
/// <para>
/// Output: <c>OS &lt;&lt; double</c> is <c>printf("%.*g", precision, value)</c> as Microsoft's C runtime writes it
/// (precision 6 by default; <c>inf</c>, <c>-inf</c>, <c>nan</c>, <c>-nan(ind)</c>; at least two exponent digits),
/// invariant culture. Input: extraction skips white space, a failed extraction sets the fail flag, and then every
/// later extraction fails, until <see cref="Clear(TextReader)"/>.
/// </para>
/// </remarks>
public static class Standard_Stream
{
    // ---------------------------------------------------------------------------------------------------------
    // Output

    private sealed class OutputState
    {
        public int Precision = 6;
    }

    private static readonly ConditionalWeakTable<TextWriter, OutputState> theOutputStates = new();

    /// <summary>The writer's precision for reals, as C++'s <c>OS.precision()</c>: 6 unless set.</summary>
    public static int Precision(TextWriter OS) => theOutputStates.TryGetValue(OS, out OutputState? aState) ? aState.Precision : 6;

    /// <summary>Sets the writer's precision for reals and returns the previous one, as C++'s <c>OS.precision(n)</c>.</summary>
    public static int Precision(TextWriter OS, int thePrecision)
    {
        OutputState aState = theOutputStates.GetOrCreateValue(OS);
        int anOld = aState.Precision;
        aState.Precision = thePrecision;
        return anOld;
    }

    /// <summary>Writes a real at the writer's precision, as C++'s <c>OS &lt;&lt; theValue</c>.</summary>
    public static void Write(TextWriter OS, double theValue) => OS.Write(FormatReal(theValue, Precision(OS)));

    /// <summary>Writes an integer, as C++'s <c>OS &lt;&lt; theValue</c>.</summary>
    public static void Write(TextWriter OS, int theValue) => OS.Write(theValue.ToString(CultureInfo.InvariantCulture));

    /// <summary>
    /// A real as <c>printf("%.*g", thePrecision, theValue)</c> writes it with Microsoft's C runtime: the shortest of
    /// fixed and exponent notation at that many significant digits, trailing zeros removed, at least two exponent
    /// digits. A precision of 0 is 1, as in C; a negative one is 6.
    /// </summary>
    /// <remarks>
    /// .NET's "G" format gives the same digits: exact, with ties rounded to even, as the C runtime's standard rounding
    /// does (the default of programs built with a Windows 10 2004 or later SDK, which native OCCT is; the legacy
    /// rounding of older programs rounded ties away from zero).
    /// </remarks>
    public static string FormatReal(double theValue, int thePrecision)
    {
        if (double.IsNaN(theValue))
        {
            return double.IsNegative(theValue) ? "-nan(ind)" : "nan";
        }

        if (double.IsInfinity(theValue))
        {
            return theValue > 0 ? "inf" : "-inf";
        }

        int aPrecision = thePrecision < 0 ? 6 : thePrecision == 0 ? 1 : thePrecision;

        // .NET's "G" format switches to exponent notation where %g does (exponent below -4 or at least the precision)
        // and removes trailing zeros; it writes "E+05" where C writes "e+05"
        return theValue.ToString("G" + aPrecision.ToString(CultureInfo.InvariantCulture), CultureInfo.InvariantCulture)
                       .Replace('E', 'e');
    }

    /// <summary>The text right-aligned in a field of the width, as C++'s <c>std::setw</c> followed by an output.</summary>
    public static string Setw(int theWidth, string theText) => theText.PadLeft(theWidth);

    // ---------------------------------------------------------------------------------------------------------
    // Input

    private sealed class InputState
    {
        public bool Fail;
        public bool Eof;
    }

    private static readonly ConditionalWeakTable<TextReader, InputState> theInputStates = new();

    private static InputState State(TextReader IS) => theInputStates.GetOrCreateValue(IS);

    /// <summary>True if an extraction failed, as C++'s <c>IS.fail()</c>.</summary>
    public static bool Fail(TextReader IS) => theInputStates.TryGetValue(IS, out InputState? aState) && aState.Fail;

    /// <summary>True if an extraction reached the end of the text, as C++'s <c>IS.eof()</c>.</summary>
    public static bool Eof(TextReader IS) => theInputStates.TryGetValue(IS, out InputState? aState) && aState.Eof;

    /// <summary>True if neither flag is set, as C++'s <c>IS.good()</c>.</summary>
    public static bool Good(TextReader IS) => !Fail(IS) && !Eof(IS);

    /// <summary>Clears the fail and end-of-file flags, as C++'s <c>IS.clear()</c>.</summary>
    public static void Clear(TextReader IS)
    {
        if (theInputStates.TryGetValue(IS, out InputState? aState))
        {
            aState.Fail = false;
            aState.Eof = false;
        }
    }

    // The sentry of a formatted extraction: fails on a stream that isn't good, then skips white space; reaching the
    // end sets both flags.
    private static bool Sentry(TextReader IS, InputState theState)
    {
        if (theState.Fail || theState.Eof)
        {
            theState.Fail = true;
            return false;
        }

        int aChar = IS.Peek();
        while (aChar >= 0 && IsSpace(aChar))
        {
            IS.Read();
            aChar = IS.Peek();
        }

        if (aChar < 0)
        {
            theState.Eof = true;
            theState.Fail = true;
            return false;
        }

        return true;
    }

    // isspace() in the C locale
    private static bool IsSpace(int theChar) => theChar is ' ' or '\t' or '\n' or '\v' or '\f' or '\r';

    /// <summary>
    /// Extracts a word, as C++'s <c>IS &gt;&gt; aBuffer</c> into a <c>char</c> array: white space is skipped, then
    /// characters are read up to the next white space, or up to <c>theWidth - 1</c> of them when a width is set
    /// (<c>IS.width(theWidth)</c>). Returns null, setting the fail flag, if there is no word (OCCT then leaves its
    /// buffer as it was).
    /// </summary>
    public static string? ReadWord(TextReader IS, int theWidth = 0)
    {
        InputState aState = State(IS);
        if (!Sentry(IS, aState))
        {
            return null;
        }

        int aMax = theWidth > 0 ? theWidth - 1 : int.MaxValue;
        var aBuilder = new StringBuilder();
        while (aBuilder.Length < aMax)
        {
            int aChar = IS.Peek();
            if (aChar < 0)
            {
                aState.Eof = true;
                break;
            }

            if (IsSpace(aChar))
            {
                break;
            }

            aBuilder.Append((char)IS.Read());
        }

        return aBuilder.ToString();
    }

    /// <summary>
    /// Extracts an integer, as C++'s <c>IS &gt;&gt; theValue</c>: an optional sign and decimal digits, stopping at
    /// the first other character. If the stream had failed, the value is left as it is; if there are no digits it is
    /// 0, and on overflow the nearest limit, with the fail flag set.
    /// </summary>
    public static void Read(TextReader IS, ref int theValue)
    {
        InputState aState = State(IS);
        if (!Sentry(IS, aState))
        {
            return;
        }

        if (ReadLong(IS, aState, out long aValue))
        {
            if (aValue > int.MaxValue)
            {
                theValue = int.MaxValue;
                aState.Fail = true;
            }
            else if (aValue < int.MinValue)
            {
                theValue = int.MinValue;
                aState.Fail = true;
            }
            else
            {
                theValue = (int)aValue;
            }
        }
        else
        {
            theValue = 0;
            aState.Fail = true;
        }
    }

    /// <summary>
    /// Extracts a boolean, as C++'s <c>IS &gt;&gt; theValue</c> without <c>boolalpha</c>: an integer, 0 for false and
    /// 1 for true; any other number gives true and sets the fail flag, no number gives false and sets it.
    /// </summary>
    public static void Read(TextReader IS, ref bool theValue)
    {
        InputState aState = State(IS);
        if (!Sentry(IS, aState))
        {
            return;
        }

        if (!ReadLong(IS, aState, out long aValue))
        {
            theValue = false;
            aState.Fail = true;
        }
        else if (aValue is 0 or 1)
        {
            theValue = aValue == 1;
        }
        else
        {
            theValue = true;
            aState.Fail = true;
        }
    }

    // num_get's integer field: [sign] digits. Returns false (value 0) if there is no digit; clamps on overflow.
    private static bool ReadLong(TextReader IS, InputState theState, out long theValue)
    {
        theValue = 0;
        bool isNegative = false;
        int aChar = IS.Peek();
        if (aChar is '+' or '-')
        {
            isNegative = aChar == '-';
            IS.Read();
            aChar = IS.Peek();
        }

        bool hasDigit = false;
        bool isOverflow = false;
        long aValue = 0;
        while (aChar is >= '0' and <= '9')
        {
            hasDigit = true;
            IS.Read();
            if (!isOverflow)
            {
                long aNext = aValue * 10 + (aChar - '0');
                if (aNext < aValue || aValue > long.MaxValue / 10)
                {
                    isOverflow = true;
                }
                else
                {
                    aValue = aNext;
                }
            }

            aChar = IS.Peek();
        }

        if (aChar < 0)
        {
            theState.Eof = true;
        }

        if (!hasDigit)
        {
            return false;
        }

        theValue = isOverflow ? (isNegative ? long.MinValue : long.MaxValue) : isNegative ? -aValue : aValue;
        return true;
    }

    // ---------------------------------------------------------------------------------------------------------
    // Strtod

    /// <summary>
    /// OCCT's <c>Strtod</c>: C's <c>strtod</c> in the C locale. Converts the longest prefix of the text (after white
    /// space) that is a decimal real (sign, digits, decimal point, exponent), <c>inf</c>, <c>infinity</c> or
    /// <c>nan</c> (with an optional <c>(...)</c>), case-insensitive; 0 if there is none. Out of range values give
    /// ±infinity or a rounded denormal or zero, as strtod does.
    /// </summary>
    /// <remarks>Hexadecimal reals (<c>0x1.8p3</c>), which OCCT never writes, are read as 0, the "0" prefix.</remarks>
    public static double Strtod(string theText) => Strtod(theText, out _);

    /// <summary>
    /// <see cref="Strtod(string)"/>, with the number of characters converted (strtod's end pointer), 0 if none.
    /// </summary>
    public static double Strtod(string theText, out int theEnd)
    {
        theEnd = 0;
        int i = 0;
        int aLength = theText.Length;
        while (i < aLength && IsSpace(theText[i]))
        {
            i++;
        }

        int aStart = i;
        bool isNegative = false;
        if (i < aLength && theText[i] is '+' or '-')
        {
            isNegative = theText[i] == '-';
            i++;
        }

        // inf, infinity, nan, nan(chars)
        if (MatchesIgnoreCase(theText, i, "inf"))
        {
            theEnd = MatchesIgnoreCase(theText, i, "infinity") ? i + 8 : i + 3;
            return isNegative ? double.NegativeInfinity : double.PositiveInfinity;
        }

        if (MatchesIgnoreCase(theText, i, "nan"))
        {
            int anEnd = i + 3;
            if (anEnd < aLength && theText[anEnd] == '(')
            {
                int j = anEnd + 1;
                while (j < aLength && (char.IsAsciiLetterOrDigit(theText[j]) || theText[j] == '_'))
                {
                    j++;
                }

                if (j < aLength && theText[j] == ')')
                {
                    anEnd = j + 1;
                }
            }

            theEnd = anEnd;
            // the quiet NaN, with the sign given (.NET's double.NaN has the sign bit set)
            return BitConverter.Int64BitsToDouble(isNegative ? unchecked((long)0xFFF8000000000000UL) : 0x7FF8000000000000L);
        }

        int aDigits = 0;
        while (i < aLength && char.IsAsciiDigit(theText[i]))
        {
            i++;
            aDigits++;
        }

        if (i < aLength && theText[i] == '.')
        {
            i++;
            while (i < aLength && char.IsAsciiDigit(theText[i]))
            {
                i++;
                aDigits++;
            }
        }

        if (aDigits == 0)
        {
            return 0.0;
        }

        if (i < aLength && theText[i] is 'e' or 'E')
        {
            int j = i + 1;
            if (j < aLength && theText[j] is '+' or '-')
            {
                j++;
            }

            if (j < aLength && char.IsAsciiDigit(theText[j]))
            {
                while (j < aLength && char.IsAsciiDigit(theText[j]))
                {
                    j++;
                }

                i = j;
            }
        }

        theEnd = i;
        // .NET's parsing rounds exactly, as the C runtime's strtod does, and overflows to infinity
        return double.Parse(theText.AsSpan(aStart, i - aStart), NumberStyles.Float, CultureInfo.InvariantCulture);
    }

    private static bool MatchesIgnoreCase(string theText, int theStart, string theWord)
        => theStart + theWord.Length <= theText.Length
           && string.Compare(theText, theStart, theWord, 0, theWord.Length, StringComparison.OrdinalIgnoreCase) == 0;
}
