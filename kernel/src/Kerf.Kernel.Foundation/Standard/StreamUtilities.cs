// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Strtod follows OCCT's Strtod (src/FoundationClasses/TKernel/Standard/Standard_CString.hxx), the C library's strtod
// in the C locale. Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.

using Kerf.Engine;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text;

namespace Kerf.Engine.Foundation;

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
internal static class StreamUtilities
{
    // ---------------------------------------------------------------------------------------------------------
    // Output

    private sealed class OutputState
    {
        public int Precision = 6;
    }

    private static readonly ConditionalWeakTable<TextWriter, OutputState> OutputStates = new();

    /// <summary>The writer's precision for reals, as C++'s <c>OS.precision()</c>: 6 unless set.</summary>
    public static int Precision(TextWriter os) => OutputStates.TryGetValue(os, out OutputState? state) ? state.Precision : 6;

    /// <summary>Sets the writer's precision for reals and returns the previous one, as C++'s <c>OS.precision(n)</c>.</summary>
    public static int Precision(TextWriter os, int precision)
    {
        OutputState state = OutputStates.GetOrCreateValue(os);
        int old = state.Precision;
        state.Precision = precision;
        return old;
    }

    /// <summary>Writes a real at the writer's precision, as C++'s <c>OS &lt;&lt; theValue</c>.</summary>
    public static void Write(TextWriter os, double value) => os.Write(FormatReal(value, Precision(os)));

    /// <summary>Writes an integer, as C++'s <c>OS &lt;&lt; theValue</c>.</summary>
    public static void Write(TextWriter os, int value) => os.Write(value.ToString(CultureInfo.InvariantCulture));

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
    public static string FormatReal(double value, int precision)
    {
        if (double.IsNaN(value))
        {
            return double.IsNegative(value) ? "-nan(ind)" : "nan";
        }

        if (double.IsInfinity(value))
        {
            return value > 0 ? "inf" : "-inf";
        }

        int aPrecision = precision < 0 ? 6 : precision == 0 ? 1 : precision;

        // .NET's "G" format switches to exponent notation where %g does (exponent below -4 or at least the precision)
        // and removes trailing zeros; it writes "E+05" where C writes "e+05"
        return value.ToString("G" + aPrecision.ToString(CultureInfo.InvariantCulture), CultureInfo.InvariantCulture)
                       .Replace('E', 'e');
    }

    /// <summary>
    /// A real as <c>printf("%.*e", thePrecision, theValue)</c> writes it with Microsoft's C runtime (C++'s
    /// <c>std::scientific</c> with <c>std::setprecision</c>): one digit, the point, <paramref name="precision"/>
    /// digits, and an exponent of at least two digits (<c>1.500000000000e+02</c>); <c>inf</c>, <c>-inf</c>, <c>nan</c>,
    /// <c>-nan(ind)</c> as <see cref="FormatReal"/>.
    /// </summary>
    /// <remarks>The digits are exact, ties rounded to even, as the C runtime's (see <see cref="FormatReal"/>).</remarks>
    public static string FormatScientific(double value, int precision)
    {
        if (double.IsNaN(value) || double.IsInfinity(value))
        {
            return FormatReal(value, precision);
        }

        int aPrecision = precision < 0 ? 6 : precision;
        // .NET writes "1.500000000000E+002": at least three exponent digits, where C writes at least two
        string text = value.ToString("E" + aPrecision.ToString(CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);
        int e = text.IndexOf('E');
        int digits = e + 2;
        while (digits < text.Length - 2 && text[digits] == '0')
        {
            digits++;
        }
        return string.Concat(text.AsSpan(0, e), "e", text.AsSpan(e + 1, 1), text.AsSpan(digits));
    }

    /// <summary>The text right-aligned in a field of the width, as C++'s <c>std::setw</c> followed by an output.</summary>
    public static string Setw(int width, string text) => text.PadLeft(width);

    // ---------------------------------------------------------------------------------------------------------
    // Input

    private sealed class InputState
    {
        public bool Fail;
        public bool Eof;
        public long Length = -1;
    }

    private static readonly ConditionalWeakTable<TextReader, InputState> InputStates = new();

    private static InputState State(TextReader IS) => InputStates.GetOrCreateValue(IS);

    /// <summary>True if an extraction failed, as C++'s <c>IS.fail()</c>.</summary>
    public static bool Fail(TextReader IS) => InputStates.TryGetValue(IS, out InputState? state) && state.Fail;

    /// <summary>True if an extraction reached the end of the text, as C++'s <c>IS.eof()</c>.</summary>
    public static bool Eof(TextReader IS) => InputStates.TryGetValue(IS, out InputState? state) && state.Eof;

    /// <summary>True if neither flag is set, as C++'s <c>IS.good()</c>.</summary>
    public static bool Good(TextReader IS) => !Fail(IS) && !Eof(IS);

    /// <summary>Clears the fail and end-of-file flags, as C++'s <c>IS.clear()</c>.</summary>
    public static void Clear(TextReader IS)
    {
        if (InputStates.TryGetValue(IS, out InputState? state))
        {
            state.Fail = false;
            state.Eof = false;
        }
    }

    // The sentry of a formatted extraction: fails on a stream that isn't good, then skips white space; reaching the
    // end sets both flags.
    private static bool Sentry(TextReader IS, InputState state)
    {
        if (state.Fail || state.Eof)
        {
            state.Fail = true;
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
            state.Eof = true;
            state.Fail = true;
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
    public static string? ReadWord(TextReader IS, int width = 0)
    {
        InputState state = State(IS);
        if (!Sentry(IS, state))
        {
            return null;
        }

        int max = width > 0 ? width - 1 : int.MaxValue;
        var builder = new StringBuilder();
        while (builder.Length < max)
        {
            int aChar = IS.Peek();
            if (aChar < 0)
            {
                state.Eof = true;
                break;
            }

            if (IsSpace(aChar))
            {
                break;
            }

            builder.Append((char)IS.Read());
        }

        return builder.ToString();
    }

    /// <summary>
    /// Extracts an integer, as C++'s <c>IS &gt;&gt; theValue</c>: an optional sign and decimal digits, stopping at
    /// the first other character. If the stream had failed, the value is left as it is; if there are no digits it is
    /// 0, and on overflow the nearest limit, with the fail flag set.
    /// </summary>
    public static void Read(TextReader IS, ref int value)
    {
        InputState state = State(IS);
        if (!Sentry(IS, state))
        {
            return;
        }

        if (ReadLong(IS, state, out long aValue))
        {
            if (aValue > int.MaxValue)
            {
                value = int.MaxValue;
                state.Fail = true;
            }
            else if (aValue < int.MinValue)
            {
                value = int.MinValue;
                state.Fail = true;
            }
            else
            {
                value = (int)aValue;
            }
        }
        else
        {
            value = 0;
            state.Fail = true;
        }
    }

    /// <summary>
    /// Extracts a boolean, as C++'s <c>IS &gt;&gt; theValue</c> without <c>boolalpha</c>: an integer, 0 for false and
    /// 1 for true; any other number gives true and sets the fail flag, no number gives false and sets it.
    /// </summary>
    public static void Read(TextReader IS, ref bool value)
    {
        InputState state = State(IS);
        if (!Sentry(IS, state))
        {
            return;
        }

        if (!ReadLong(IS, state, out long aValue))
        {
            value = false;
            state.Fail = true;
        }
        else if (aValue is 0 or 1)
        {
            value = aValue == 1;
        }
        else
        {
            value = true;
            state.Fail = true;
        }
    }

    // num_get's integer field: [sign] digits. Returns false (value 0) if there is no digit; clamps on overflow.
    private static bool ReadLong(TextReader IS, InputState state, out long value)
    {
        value = 0;
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
                long next = aValue * 10 + (aChar - '0');
                if (next < aValue || aValue > long.MaxValue / 10)
                {
                    isOverflow = true;
                }
                else
                {
                    aValue = next;
                }
            }

            aChar = IS.Peek();
        }

        if (aChar < 0)
        {
            state.Eof = true;
        }

        if (!hasDigit)
        {
            return false;
        }

        value = isOverflow ? (isNegative ? long.MinValue : long.MaxValue) : isNegative ? -aValue : aValue;
        return true;
    }

    // ---------------------------------------------------------------------------------------------------------
    // C strings and lines

    /// <summary>
    /// Extracts a word into a C buffer, as C++'s <c>IS &gt;&gt; theBuffer</c> into a <c>char[N]</c>: the word and a
    /// terminating <c>'\0'</c> are stored at the start of the buffer, and the rest of the buffer is left as it was.
    /// Returns false, leaving the buffer as it was and setting the fail flag, if there is no word.
    /// </summary>
    /// <exception cref="StreamFormatErrorException">The word doesn't fit in the buffer with its terminating null (OCCT
    /// writes past the end of its buffer).</exception>
    public static bool ReadWord(TextReader IS, char[] buffer)
    {
        // Kerf: untrusted input: at most the buffer's length is read, and a word that long doesn't fit
        string? word = ReadWord(IS, buffer.Length + 1);
        if (word is null)
        {
            return false;
        }

        if (word.Length >= buffer.Length)
        {
            throw new StreamFormatErrorException("A word longer than " + (buffer.Length - 1) + " characters");
        }

        word.CopyTo(0, buffer, 0, word.Length);
        buffer[word.Length] = '\0';
        return true;
    }

    /// <summary>
    /// C++'s <c>IS.getline(theBuffer, theCount, theDelim)</c>: extracts characters without skipping white space, up to
    /// and including the delimiter (which isn't stored), the end of the text (setting the end-of-file flag), or
    /// <c>theCount - 1</c> stored characters followed by another one (setting the fail flag, without extracting it);
    /// then stores a terminating <c>'\0'</c>. Extracting nothing sets the fail flag, as does a stream that isn't good
    /// (which stores only the <c>'\0'</c>). Returns true if the fail flag isn't set.
    /// </summary>
    public static bool GetLine(TextReader IS, char[] buffer, int count, char delim)
    {
        if (count > buffer.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(count));
        }

        InputState state = State(IS);
        int stored = 0;
        int extracted = 0;
        if (!state.Fail && !state.Eof && count > 0)
        {
            while (true)
            {
                int aChar = IS.Peek();
                if (aChar < 0)
                {
                    state.Eof = true;
                    break;
                }

                if (aChar == delim)
                {
                    IS.Read();
                    extracted++;
                    break;
                }

                if (stored >= count - 1)
                {
                    state.Fail = true;
                    break;
                }

                buffer[stored++] = (char)IS.Read();
                extracted++;
            }
        }

        if (count > 0)
        {
            buffer[stored] = '\0';
        }

        if (extracted == 0)
        {
            state.Fail = true;
        }

        return !state.Fail;
    }

    /// <summary>The length of a C string in the buffer: the characters before the first <c>'\0'</c>.</summary>
    public static int Strlen(ReadOnlySpan<char> buffer)
    {
        int length = buffer.IndexOf('\0');
        return length < 0 ? buffer.Length : length;
    }

    /// <summary>The C string in the buffer, up to its first <c>'\0'</c>.</summary>
    public static string CString(ReadOnlySpan<char> buffer) => new(buffer[..Strlen(buffer)]);

    /// <summary>
    /// C's <c>atoi</c> on the C string in the buffer: white space skipped, an optional sign and decimal digits; 0 if
    /// there are none; clamped to the range of an <c>int</c> on overflow, as Microsoft's C runtime does.
    /// </summary>
    public static int Atoi(ReadOnlySpan<char> buffer)
    {
        int length = Strlen(buffer);
        int i = 0;
        while (i < length && IsSpace(buffer[i]))
        {
            i++;
        }

        bool isNegative = false;
        if (i < length && buffer[i] is '+' or '-')
        {
            isNegative = buffer[i] == '-';
            i++;
        }

        long value = 0;
        for (; i < length && buffer[i] is >= '0' and <= '9'; i++)
        {
            value = Math.Min(value * 10 + (buffer[i] - '0'), (long)int.MaxValue + 1);
        }

        value = isNegative ? -value : value;
        return (int)Math.Clamp(value, int.MinValue, int.MaxValue);
    }

    // ---------------------------------------------------------------------------------------------------------
    // Untrusted input

    /// <summary>
    /// The length assumed for a reader whose length isn't known (2^28 characters): one that isn't a
    /// <see cref="StreamReader"/> over a seekable stream and wasn't given a length with
    /// <see cref="SetInputLength"/>.
    /// </summary>
    public const long UnknownInputLength = 1L << 28;

    /// <summary>
    /// Declares how many characters the reader holds at most, so that the readers of OCCT's formats can reject
    /// counts that the input couldn't hold before allocating for them. A <see cref="StreamReader"/> over a seekable
    /// stream needs no declaration; for a <see cref="StringReader"/>, pass the string's length.
    /// </summary>
    public static void SetInputLength(TextReader IS, long length)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(length);
        State(IS).Length = length;
    }

    /// <summary>
    /// The number of characters the reader holds at most: the length given to <see cref="SetInputLength"/>, the
    /// length in bytes of a <see cref="StreamReader"/>'s seekable stream, or <see cref="UnknownInputLength"/>.
    /// </summary>
    public static long InputLength(TextReader IS)
    {
        if (InputStates.TryGetValue(IS, out InputState? state) && state.Length >= 0)
        {
            return state.Length;
        }

        try
        {
            if (IS is StreamReader { BaseStream: { CanSeek: true } stream })
            {
                // a character takes at least one byte in every encoding a StreamReader reads
                return stream.Length;
            }
        }
        catch (ObjectDisposedException)
        {
        }

        return UnknownInputLength;
    }

    /// <summary>
    /// Checks a count read from the reader before it is used to allocate or to loop: it must not be negative, and
    /// the items, taking at least <paramref name="minChars"/> characters each, must fit in the reader's
    /// <see cref="InputLength"/>.
    /// </summary>
    /// <exception cref="StreamFormatErrorException">The count is negative or larger than the input could hold.</exception>
    public static void CheckCount(TextReader IS, long count, int minChars, string what)
    {
        if (count < 0 || count > InputLength(IS) / Math.Max(minChars, 1))
        {
            throw new StreamFormatErrorException("A count of " + what + " (" + count
                                                + ") that the input can't hold");
        }
    }

    /// <summary>
    /// Throws <see cref="StreamFormatErrorException"/> if an extraction from the reader has failed: for the places
    /// where OCCT's readers would loop for ever, or for as long as a count says, on a stream that has failed.
    /// </summary>
    public static void CheckNotFailed(TextReader IS, string theWhere)
    {
        if (Fail(IS))
        {
            throw new StreamFormatErrorException(theWhere + ": unexpected end or format of the input");
        }
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
    public static double Strtod(string text) => Strtod(text, out _);

    /// <summary>
    /// <see cref="Strtod(string)"/>, with the number of characters converted (strtod's end pointer), 0 if none.
    /// </summary>
    public static double Strtod(string text, out int end) => Strtod(text.AsSpan(), out end);

    /// <summary>
    /// <see cref="Strtod(string)"/> on a span of characters, with the number of characters converted, 0 if none.
    /// </summary>
    public static double Strtod(ReadOnlySpan<char> text, out int end)
    {
        end = 0;
        int i = 0;
        int length = text.Length;
        while (i < length && IsSpace(text[i]))
        {
            i++;
        }

        int start = i;
        bool isNegative = false;
        if (i < length && text[i] is '+' or '-')
        {
            isNegative = text[i] == '-';
            i++;
        }

        // inf, infinity, nan, nan(chars)
        if (MatchesIgnoreCase(text, i, "inf"))
        {
            end = MatchesIgnoreCase(text, i, "infinity") ? i + 8 : i + 3;
            return isNegative ? double.NegativeInfinity : double.PositiveInfinity;
        }

        if (MatchesIgnoreCase(text, i, "nan"))
        {
            int anEnd = i + 3;
            if (anEnd < length && text[anEnd] == '(')
            {
                int j = anEnd + 1;
                while (j < length && (char.IsAsciiLetterOrDigit(text[j]) || text[j] == '_'))
                {
                    j++;
                }

                if (j < length && text[j] == ')')
                {
                    anEnd = j + 1;
                }
            }

            end = anEnd;
            // the quiet NaN, with the sign given (.NET's double.NaN has the sign bit set)
            return BitConverter.Int64BitsToDouble(isNegative ? unchecked((long)0xFFF8000000000000UL) : 0x7FF8000000000000L);
        }

        int digits = 0;
        while (i < length && char.IsAsciiDigit(text[i]))
        {
            i++;
            digits++;
        }

        if (i < length && text[i] == '.')
        {
            i++;
            while (i < length && char.IsAsciiDigit(text[i]))
            {
                i++;
                digits++;
            }
        }

        if (digits == 0)
        {
            return 0.0;
        }

        if (i < length && text[i] is 'e' or 'E')
        {
            int j = i + 1;
            if (j < length && text[j] is '+' or '-')
            {
                j++;
            }

            if (j < length && char.IsAsciiDigit(text[j]))
            {
                while (j < length && char.IsAsciiDigit(text[j]))
                {
                    j++;
                }

                i = j;
            }
        }

        end = i;
        // .NET's parsing rounds exactly, as the C runtime's strtod does, and overflows to infinity
        return double.Parse(text.Slice(start, i - start), NumberStyles.Float, CultureInfo.InvariantCulture);
    }

    private static bool MatchesIgnoreCase(ReadOnlySpan<char> text, int start, string word)
        => start + word.Length <= text.Length
           && text.Slice(start, word.Length).Equals(word, StringComparison.OrdinalIgnoreCase);
}
