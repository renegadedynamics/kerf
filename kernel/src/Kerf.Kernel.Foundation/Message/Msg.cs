// Port of: src/FoundationClasses/TKernel/Message/Message_Msg.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Msg.lxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Msg.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;

namespace Kerf.Engine.Foundation;

/// <summary>
/// A message with placeholders (<c>%s</c>, <c>%d</c>, <c>%f</c> and the other C conversions) filled one by one with
/// <see cref="Arg(string)"/>, made from a keyword of <see cref="MsgFile"/> or from a text. <see cref="Get"/>
/// returns the text with every placeholder still unfilled replaced by <c>UNKNOWN</c>.
/// </summary>
/// <remarks>
/// A working object, as OCCT's (which is copied by value): <see cref="Msg(Msg)"/> is its copy
/// constructor. TCollection's strings are <see cref="string"/>, so OCCT's overloads by string type merge: the
/// <c>const char*</c> and AsciiString forms of <c>Arg</c> are <see cref="Arg(string)"/> (formatted with the
/// placeholder's flags, width and precision, as <c>Sprintf</c>), and the ExtendedString form, which inserts the text
/// as it is, is <see cref="ArgExtended"/>. The <c>operator&lt;&lt;</c> forms are the <c>Arg</c> methods.
/// </remarks>
internal sealed class Msg
{
    // OCCT's FormatType
    private const int Msg_IntegerType = 0;
    private const int Msg_RealType = 1;
    private const int Msg_StringType = 2;

    private string _original = "";
    private StringBuilder _messageBody = new();
    private readonly NCollection_Sequence<int> _seqOfFormats = new();

    /// <summary>An empty message.</summary>
    public Msg()
    {
    }

    /// <summary>A copy of <paramref name="msg"/>, with its filled and unfilled placeholders.</summary>
    public Msg(Msg msg)
    {
        _messageBody = new StringBuilder(msg._messageBody.ToString());
        _original = msg._original;
        for (int i = 1, n = msg._seqOfFormats.Length(); i <= n; i++)
        {
            _seqOfFormats.Append(msg._seqOfFormats.Value(i));
        }
    }

    /// <summary>The message of <see cref="MsgFile"/> for the keyword.</summary>
    public Msg(string msgCode)
    {
        Set(MsgFile.Msg(msgCode));
    }

    private char CharAt(int index) => index < _messageBody.Length ? _messageBody[index] : '\0';

    /// <summary>Sets the message text, as an alternative to a keyword of the message file.</summary>
    public void Set(string msg)
    {
        _messageBody = new StringBuilder(msg);

        // OCCT reads the characters through a pointer into the body, which Remove shifts in place: charAt reads the
        // current body the same way.
        int msgLength = _messageBody.Length;
        for (int i = 0; i < msgLength; i++)
        {
            //  Search for '%' character starting a format specification
            if (CharAt(i) == '%')
            {
                int start = i++;
                char aChar = CharAt(i);
                //        Check for format '%%'
                if (aChar == '%')
                {
                    // OCCT: the character after "%%" is then read as the start of a format specification, so a literal
                    // "%%" followed by a conversion ("100%% done" reads "% d") becomes a placeholder.
                    _messageBody.Remove(i, 1);
                    if (i >= --msgLength)
                    {
                        break;
                    }
                    aChar = CharAt(i);
                }
                //        Skip flags, field width and precision
                while (i < msgLength)
                {
                    if (aChar == '-' || aChar == '+' || aChar == ' ' || aChar == '#' || (aChar >= '0' && aChar <= '9')
                        || aChar == '.')
                    {
                        i++;
                    }
                    else
                    {
                        break;
                    }
                    aChar = CharAt(i);
                }
                if (i >= msgLength)
                {
                    break;
                }

                int formatType;
                if (aChar == 'h' || aChar == 'l')
                {
                    aChar = CharAt(++i);
                }
                switch (aChar) // detect the type of format spec
                {
                    case 'd':
                    case 'i':
                    case 'o':
                    case 'u':
                    case 'x':
                    case 'X':
                        formatType = Msg_IntegerType;
                        break;
                    case 'f':
                    case 'e':
                    case 'E':
                    case 'g':
                    case 'G':
                        formatType = Msg_RealType;
                        break;
                    case 's':
                        formatType = Msg_StringType;
                        break;
                    default:
                        continue;
                }
                _seqOfFormats.Append(formatType); // type
                _seqOfFormats.Append(start); // beginning pos
                _seqOfFormats.Append(i + 1 - start); // length
            }
        }
        _original = _messageBody.ToString();
    }

    /// <summary>Fills the next <c>%s</c> placeholder with the string, formatted by the placeholder (C's <c>sprintf</c>).</summary>
    public Msg Arg(string theString)
    {
        // get location and format
        int first = GetFormat(Msg_StringType, out string format);
        if (first == 0)
        {
            return this;
        }

        // print string according to format
        string str = Sprintf(format, theString);

        // replace the format placeholder by the actual string
        ReplaceText(first, format.Length, str);

        return this;
    }

    /// <summary>
    /// Fills the next <c>%s</c> placeholder with the string as it is (OCCT's <c>Arg(const TCollection_ExtendedString&amp;)</c>,
    /// which ignores the placeholder's width and precision).
    /// </summary>
    public Msg ArgExtended(string theString)
    {
        // get location and format
        int first = GetFormat(Msg_StringType, out string format);
        if (first == 0)
        {
            return this;
        }

        // replace the format placeholder by the actual string
        ReplaceText(first, format.Length, theString);

        return this;
    }

    /// <summary>Fills the next integer placeholder (<c>%d</c>, <c>%i</c>, <c>%o</c>, <c>%u</c>, <c>%x</c>, <c>%X</c>).</summary>
    public Msg Arg(int value)
    {
        // get location and format
        int first = GetFormat(Msg_IntegerType, out string format);
        if (first == 0)
        {
            return this;
        }

        // print string according to format
        string str = Sprintf(format, value);

        // replace the format placeholder by the actual string
        ReplaceText(first, format.Length, str);

        return this;
    }

    /// <summary>Fills the next real placeholder (<c>%f</c>, <c>%e</c>, <c>%E</c>, <c>%g</c>, <c>%G</c>).</summary>
    public Msg Arg(double value)
    {
        // get location and format
        int first = GetFormat(Msg_RealType, out string format);
        if (first == 0)
        {
            return this;
        }

        // print string according to format
        string str = Sprintf(format, value);

        // replace the format placeholder by the actual string
        ReplaceText(first, format.Length, str);

        return this;
    }

    /// <summary>The original message text.</summary>
    public string Original() => _original;

    /// <summary>The message text with the placeholders filled so far.</summary>
    public string Value() => _messageBody.ToString();

    /// <summary>True if <see cref="Value"/> differs from <see cref="Original"/>.</summary>
    public bool IsEdited => !string.Equals(_original, _messageBody.ToString(), StringComparison.Ordinal);

    /// <summary>
    /// The message with every placeholder not yet filled replaced by <c>UNKNOWN</c>. The replacement is made in the
    /// message itself.
    /// </summary>
    /// <remarks>
    /// OCCT: the placeholders replaced are not forgotten, so a second call replaces again at their old positions and
    /// garbles the text ("a %s b" gives "a UNKNOWN b", then "a UNKNOWNKNOWN b"); reproduced.
    /// </remarks>
    public string Get()
    {
        // remove all non-initialised format specifications
        int i, increment = 0;
        const string unknown = "UNKNOWN";
        for (i = 1; i < _seqOfFormats.Length(); i += 3)
        {
            string body = _messageBody.ToString();
            int split = _seqOfFormats[i + 1] + increment;
            string rightPart = body.Substring(split);
            rightPart = rightPart.Remove(0, _seqOfFormats[i + 2]);
            _messageBody = new StringBuilder(body, 0, split, body.Length + unknown.Length);
            _messageBody.Append(unknown);
            _messageBody.Append(rightPart);
            increment += unknown.Length - _seqOfFormats[i + 2];
        }
        return _messageBody.ToString();
    }

    public override string ToString() => _messageBody.ToString();

    // Finds the placeholder where to put the next value of the type; returns its 1-based position and its format,
    // and forgets it. 0 if there is none.
    private int GetFormat(int type, out string format)
    {
        for (int i = 1; i <= _seqOfFormats.Length(); i += 3)
        {
            if (_seqOfFormats[i] == type)
            {
                // Extract format
                int first = _seqOfFormats[i + 1];
                int len = _seqOfFormats[i + 2];
                char[] aFormat = new string(' ', len).ToCharArray();
                for (int j = 1; j <= len; j++)
                {
                    char aChar = _messageBody[first + j - 1];
                    if (aChar < 128)
                    {
                        aFormat[j - 1] = aChar;
                    }
                }
                format = new string(aFormat);
                // delete information on this placeholder
                _seqOfFormats.Remove(i, i + 2);
                // return start position
                return first + 1;
            }
        }
        format = "";
        return 0;
    }

    // Replaces theNb characters of the body from the 1-based position theFirst by theStr.
    private void ReplaceText(int first, int nb, string str)
    {
        _messageBody.Remove(first - 1, nb);
        _messageBody.Insert(first - 1, str);

        // update information on remaining format placeholders
        int increment = str.Length - nb;
        if (increment == 0)
        {
            return;
        }
        for (int i = 1; i <= _seqOfFormats.Length(); i += 3)
        {
            if (_seqOfFormats[i + 1] > first)
            {
                _seqOfFormats[i + 1] += increment;
            }
        }
    }

    /// <summary>
    /// C's <c>sprintf</c> of one conversion (<c>%[flags][width][.precision][h|l]conversion</c>) as Microsoft's C runtime
    /// writes it, invariant culture: the formatting <see cref="Msg"/>'s <c>Arg</c> methods need.
    /// </summary>
    internal static string Sprintf(string format, object value)
    {
        int pos = 0;
        if (pos < format.Length && format[pos] == '%')
        {
            pos++;
        }
        bool isLeft = false, isPlus = false, isSpace = false, isAlt = false, isZero = false;
        for (; pos < format.Length; pos++)
        {
            char aChar = format[pos];
            if (aChar == '-') isLeft = true;
            else if (aChar == '+') isPlus = true;
            else if (aChar == ' ') isSpace = true;
            else if (aChar == '#') isAlt = true;
            else if (aChar == '0') isZero = true;
            else break;
        }
        int width = 0;
        for (; pos < format.Length && char.IsAsciiDigit(format[pos]); pos++)
        {
            width = width * 10 + (format[pos] - '0');
        }
        int precision = -1;
        if (pos < format.Length && format[pos] == '.')
        {
            precision = 0;
            for (pos++; pos < format.Length && char.IsAsciiDigit(format[pos]); pos++)
            {
                precision = precision * 10 + (format[pos] - '0');
            }
        }
        while (pos < format.Length && (format[pos] == 'h' || format[pos] == 'l'))
        {
            pos++;
        }
        char conv = pos < format.Length ? format[pos] : 's';

        string sign = "";
        string body;
        bool isNumeric = true;
        switch (conv)
        {
            case 's':
            {
                isNumeric = false;
                body = Convert.ToString(value, CultureInfo.InvariantCulture) ?? "";
                if (precision >= 0 && body.Length > precision)
                {
                    body = body.Substring(0, precision);
                }
                break;
            }
            case 'd':
            case 'i':
            {
                long aValue = Convert.ToInt64(value, CultureInfo.InvariantCulture);
                sign = aValue < 0 ? "-" : isPlus ? "+" : isSpace ? " " : "";
                body = Math.Abs(aValue).ToString(CultureInfo.InvariantCulture);
                body = IntegerPrecision(body, precision, aValue == 0);
                break;
            }
            case 'u':
            case 'o':
            case 'x':
            case 'X':
            {
                uint aValue = unchecked((uint)Convert.ToInt64(value, CultureInfo.InvariantCulture));
                body = conv switch
                {
                    'u' => aValue.ToString(CultureInfo.InvariantCulture),
                    'o' => Convert.ToString(aValue, 8),
                    'x' => aValue.ToString("x", CultureInfo.InvariantCulture),
                    _ => aValue.ToString("X", CultureInfo.InvariantCulture),
                };
                body = IntegerPrecision(body, precision, aValue == 0);
                if (isAlt && aValue != 0)
                {
                    if (conv == 'o' && !body.StartsWith('0'))
                    {
                        body = "0" + body;
                    }
                    else if (conv == 'x')
                    {
                        sign = "0x";
                    }
                    else if (conv == 'X')
                    {
                        sign = "0X";
                    }
                }
                break;
            }
            default:
            {
                double aValue = Convert.ToDouble(value, CultureInfo.InvariantCulture);
                bool isNegative = double.IsNegative(aValue) && !double.IsNaN(aValue);
                sign = isNegative ? "-" : isPlus ? "+" : isSpace ? " " : "";
                double abs = Math.Abs(aValue);
                int prec = precision < 0 ? 6 : precision;
                if (double.IsNaN(aValue))
                {
                    body = "nan";
                    isZero = false;
                }
                else if (double.IsInfinity(aValue))
                {
                    body = "inf";
                    isZero = false;
                }
                else if (conv == 'f' || conv == 'F')
                {
                    body = abs.ToString("F" + prec.ToString(CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);
                    if (isAlt && prec == 0)
                    {
                        body += ".";
                    }
                }
                else if (conv == 'e' || conv == 'E')
                {
                    body = Exponent(abs, prec);
                }
                else
                {
                    body = StreamUtilities.FormatReal(abs, prec == 0 ? 1 : prec);
                }
                if (char.IsUpper(conv))
                {
                    body = body.ToUpperInvariant();
                }
                break;
            }
        }

        int pad = width - sign.Length - body.Length;
        if (pad <= 0)
        {
            return sign + body;
        }
        if (isLeft)
        {
            return sign + body + new string(' ', pad);
        }
        if (isNumeric && isZero && (precision < 0 || conv is 'f' or 'F' or 'e' or 'E' or 'g' or 'G'))
        {
            return sign + new string('0', pad) + body;
        }
        return new string(' ', pad) + sign + body;
    }

    private static string IntegerPrecision(string digits, int precision, bool isZero)
    {
        if (precision == 0 && isZero)
        {
            return "";
        }
        return precision > digits.Length ? new string('0', precision - digits.Length) + digits : digits;
    }

    // "%.*e" of a non-negative finite value: the mantissa and at least two exponent digits.
    private static string Exponent(double value, int precision)
    {
        string text = value.ToString("e" + precision.ToString(CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);
        int e = text.IndexOf('e');
        int exp = int.Parse(text.AsSpan(e + 1), NumberStyles.AllowLeadingSign, CultureInfo.InvariantCulture);
        string expText = Math.Abs(exp).ToString("00", CultureInfo.InvariantCulture);
        return text.Substring(0, e) + (exp < 0 ? "e-" : "e+") + expText;
    }
}
