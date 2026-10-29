// Port of: src/FoundationClasses/TKernel/Message/Message_Msg.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Msg.lxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Msg.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;

namespace Occt;

/// <summary>
/// A message with placeholders (<c>%s</c>, <c>%d</c>, <c>%f</c> and the other C conversions) filled one by one with
/// <see cref="Arg(string)"/>, made from a keyword of <see cref="Message_MsgFile"/> or from a text. <see cref="Get"/>
/// returns the text with every placeholder still unfilled replaced by <c>UNKNOWN</c>.
/// </summary>
/// <remarks>
/// A working object, as OCCT's (which is copied by value): <see cref="Message_Msg(Message_Msg)"/> is its copy
/// constructor. TCollection's strings are <see cref="string"/>, so OCCT's overloads by string type merge: the
/// <c>const char*</c> and AsciiString forms of <c>Arg</c> are <see cref="Arg(string)"/> (formatted with the
/// placeholder's flags, width and precision, as <c>Sprintf</c>), and the ExtendedString form, which inserts the text
/// as it is, is <see cref="ArgExtended"/>. The <c>operator&lt;&lt;</c> forms are the <c>Arg</c> methods.
/// </remarks>
public sealed class Message_Msg
{
    // OCCT's FormatType
    private const int Msg_IntegerType = 0;
    private const int Msg_RealType = 1;
    private const int Msg_StringType = 2;

    private string myOriginal = "";
    private StringBuilder myMessageBody = new();
    private readonly NCollection_Sequence<int> mySeqOfFormats = new();

    /// <summary>An empty message.</summary>
    public Message_Msg()
    {
    }

    /// <summary>A copy of <paramref name="theMsg"/>, with its filled and unfilled placeholders.</summary>
    public Message_Msg(Message_Msg theMsg)
    {
        myMessageBody = new StringBuilder(theMsg.myMessageBody.ToString());
        myOriginal = theMsg.myOriginal;
        for (int i = 1, n = theMsg.mySeqOfFormats.Length(); i <= n; i++)
        {
            mySeqOfFormats.Append(theMsg.mySeqOfFormats.Value(i));
        }
    }

    /// <summary>The message of <see cref="Message_MsgFile"/> for the keyword.</summary>
    public Message_Msg(string theMsgCode)
    {
        Set(Message_MsgFile.Msg(theMsgCode));
    }

    private char charAt(int theIndex) => theIndex < myMessageBody.Length ? myMessageBody[theIndex] : '\0';

    /// <summary>Sets the message text, as an alternative to a keyword of the message file.</summary>
    public void Set(string theMsg)
    {
        myMessageBody = new StringBuilder(theMsg);

        // OCCT reads the characters through a pointer into the body, which Remove shifts in place: charAt reads the
        // current body the same way.
        int anMsgLength = myMessageBody.Length;
        for (int i = 0; i < anMsgLength; i++)
        {
            //  Search for '%' character starting a format specification
            if (charAt(i) == '%')
            {
                int aStart = i++;
                char aChar = charAt(i);
                //        Check for format '%%'
                if (aChar == '%')
                {
                    // OCCT: the character after "%%" is then read as the start of a format specification, so a literal
                    // "%%" followed by a conversion ("100%% done" reads "% d") becomes a placeholder.
                    myMessageBody.Remove(i, 1);
                    if (i >= --anMsgLength)
                    {
                        break;
                    }
                    aChar = charAt(i);
                }
                //        Skip flags, field width and precision
                while (i < anMsgLength)
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
                    aChar = charAt(i);
                }
                if (i >= anMsgLength)
                {
                    break;
                }

                int aFormatType;
                if (aChar == 'h' || aChar == 'l')
                {
                    aChar = charAt(++i);
                }
                switch (aChar) // detect the type of format spec
                {
                    case 'd':
                    case 'i':
                    case 'o':
                    case 'u':
                    case 'x':
                    case 'X':
                        aFormatType = Msg_IntegerType;
                        break;
                    case 'f':
                    case 'e':
                    case 'E':
                    case 'g':
                    case 'G':
                        aFormatType = Msg_RealType;
                        break;
                    case 's':
                        aFormatType = Msg_StringType;
                        break;
                    default:
                        continue;
                }
                mySeqOfFormats.Append(aFormatType); // type
                mySeqOfFormats.Append(aStart); // beginning pos
                mySeqOfFormats.Append(i + 1 - aStart); // length
            }
        }
        myOriginal = myMessageBody.ToString();
    }

    /// <summary>Fills the next <c>%s</c> placeholder with the string, formatted by the placeholder (C's <c>sprintf</c>).</summary>
    public Message_Msg Arg(string theString)
    {
        // get location and format
        int aFirst = getFormat(Msg_StringType, out string aFormat);
        if (aFirst == 0)
        {
            return this;
        }

        // print string according to format
        string aStr = Sprintf(aFormat, theString);

        // replace the format placeholder by the actual string
        replaceText(aFirst, aFormat.Length, aStr);

        return this;
    }

    /// <summary>
    /// Fills the next <c>%s</c> placeholder with the string as it is (OCCT's <c>Arg(const TCollection_ExtendedString&amp;)</c>,
    /// which ignores the placeholder's width and precision).
    /// </summary>
    public Message_Msg ArgExtended(string theString)
    {
        // get location and format
        int aFirst = getFormat(Msg_StringType, out string aFormat);
        if (aFirst == 0)
        {
            return this;
        }

        // replace the format placeholder by the actual string
        replaceText(aFirst, aFormat.Length, theString);

        return this;
    }

    /// <summary>Fills the next integer placeholder (<c>%d</c>, <c>%i</c>, <c>%o</c>, <c>%u</c>, <c>%x</c>, <c>%X</c>).</summary>
    public Message_Msg Arg(int theValue)
    {
        // get location and format
        int aFirst = getFormat(Msg_IntegerType, out string aFormat);
        if (aFirst == 0)
        {
            return this;
        }

        // print string according to format
        string aStr = Sprintf(aFormat, theValue);

        // replace the format placeholder by the actual string
        replaceText(aFirst, aFormat.Length, aStr);

        return this;
    }

    /// <summary>Fills the next real placeholder (<c>%f</c>, <c>%e</c>, <c>%E</c>, <c>%g</c>, <c>%G</c>).</summary>
    public Message_Msg Arg(double theValue)
    {
        // get location and format
        int aFirst = getFormat(Msg_RealType, out string aFormat);
        if (aFirst == 0)
        {
            return this;
        }

        // print string according to format
        string aStr = Sprintf(aFormat, theValue);

        // replace the format placeholder by the actual string
        replaceText(aFirst, aFormat.Length, aStr);

        return this;
    }

    /// <summary>The original message text.</summary>
    public string Original() => myOriginal;

    /// <summary>The message text with the placeholders filled so far.</summary>
    public string Value() => myMessageBody.ToString();

    /// <summary>True if <see cref="Value"/> differs from <see cref="Original"/>.</summary>
    public bool IsEdited() => !string.Equals(myOriginal, myMessageBody.ToString(), StringComparison.Ordinal);

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
        int i, anIncrement = 0;
        const string anUnknown = "UNKNOWN";
        for (i = 1; i < mySeqOfFormats.Length(); i += 3)
        {
            string aBody = myMessageBody.ToString();
            int aSplit = mySeqOfFormats[i + 1] + anIncrement;
            string aRightPart = aBody.Substring(aSplit);
            aRightPart = aRightPart.Remove(0, mySeqOfFormats[i + 2]);
            myMessageBody = new StringBuilder(aBody, 0, aSplit, aBody.Length + anUnknown.Length);
            myMessageBody.Append(anUnknown);
            myMessageBody.Append(aRightPart);
            anIncrement += anUnknown.Length - mySeqOfFormats[i + 2];
        }
        return myMessageBody.ToString();
    }

    public override string ToString() => myMessageBody.ToString();

    // Finds the placeholder where to put the next value of the type; returns its 1-based position and its format,
    // and forgets it. 0 if there is none.
    private int getFormat(int theType, out string theFormat)
    {
        for (int i = 1; i <= mySeqOfFormats.Length(); i += 3)
        {
            if (mySeqOfFormats[i] == theType)
            {
                // Extract format
                int aFirst = mySeqOfFormats[i + 1];
                int aLen = mySeqOfFormats[i + 2];
                char[] aFormat = new string(' ', aLen).ToCharArray();
                for (int j = 1; j <= aLen; j++)
                {
                    char aChar = myMessageBody[aFirst + j - 1];
                    if (aChar < 128)
                    {
                        aFormat[j - 1] = aChar;
                    }
                }
                theFormat = new string(aFormat);
                // delete information on this placeholder
                mySeqOfFormats.Remove(i, i + 2);
                // return start position
                return aFirst + 1;
            }
        }
        theFormat = "";
        return 0;
    }

    // Replaces theNb characters of the body from the 1-based position theFirst by theStr.
    private void replaceText(int theFirst, int theNb, string theStr)
    {
        myMessageBody.Remove(theFirst - 1, theNb);
        myMessageBody.Insert(theFirst - 1, theStr);

        // update information on remaining format placeholders
        int anIncrement = theStr.Length - theNb;
        if (anIncrement == 0)
        {
            return;
        }
        for (int i = 1; i <= mySeqOfFormats.Length(); i += 3)
        {
            if (mySeqOfFormats[i + 1] > theFirst)
            {
                mySeqOfFormats[i + 1] += anIncrement;
            }
        }
    }

    /// <summary>
    /// C's <c>sprintf</c> of one conversion (<c>%[flags][width][.precision][h|l]conversion</c>) as Microsoft's C runtime
    /// writes it, invariant culture: the formatting <see cref="Message_Msg"/>'s <c>Arg</c> methods need.
    /// </summary>
    internal static string Sprintf(string theFormat, object theValue)
    {
        int aPos = 0;
        if (aPos < theFormat.Length && theFormat[aPos] == '%')
        {
            aPos++;
        }
        bool isLeft = false, isPlus = false, isSpace = false, isAlt = false, isZero = false;
        for (; aPos < theFormat.Length; aPos++)
        {
            char aChar = theFormat[aPos];
            if (aChar == '-') isLeft = true;
            else if (aChar == '+') isPlus = true;
            else if (aChar == ' ') isSpace = true;
            else if (aChar == '#') isAlt = true;
            else if (aChar == '0') isZero = true;
            else break;
        }
        int aWidth = 0;
        for (; aPos < theFormat.Length && char.IsAsciiDigit(theFormat[aPos]); aPos++)
        {
            aWidth = aWidth * 10 + (theFormat[aPos] - '0');
        }
        int aPrecision = -1;
        if (aPos < theFormat.Length && theFormat[aPos] == '.')
        {
            aPrecision = 0;
            for (aPos++; aPos < theFormat.Length && char.IsAsciiDigit(theFormat[aPos]); aPos++)
            {
                aPrecision = aPrecision * 10 + (theFormat[aPos] - '0');
            }
        }
        while (aPos < theFormat.Length && (theFormat[aPos] == 'h' || theFormat[aPos] == 'l'))
        {
            aPos++;
        }
        char aConv = aPos < theFormat.Length ? theFormat[aPos] : 's';

        string aSign = "";
        string aBody;
        bool isNumeric = true;
        switch (aConv)
        {
            case 's':
            {
                isNumeric = false;
                aBody = Convert.ToString(theValue, CultureInfo.InvariantCulture) ?? "";
                if (aPrecision >= 0 && aBody.Length > aPrecision)
                {
                    aBody = aBody.Substring(0, aPrecision);
                }
                break;
            }
            case 'd':
            case 'i':
            {
                long aValue = Convert.ToInt64(theValue, CultureInfo.InvariantCulture);
                aSign = aValue < 0 ? "-" : isPlus ? "+" : isSpace ? " " : "";
                aBody = Math.Abs(aValue).ToString(CultureInfo.InvariantCulture);
                aBody = integerPrecision(aBody, aPrecision, aValue == 0);
                break;
            }
            case 'u':
            case 'o':
            case 'x':
            case 'X':
            {
                uint aValue = unchecked((uint)Convert.ToInt64(theValue, CultureInfo.InvariantCulture));
                aBody = aConv switch
                {
                    'u' => aValue.ToString(CultureInfo.InvariantCulture),
                    'o' => Convert.ToString(aValue, 8),
                    'x' => aValue.ToString("x", CultureInfo.InvariantCulture),
                    _ => aValue.ToString("X", CultureInfo.InvariantCulture),
                };
                aBody = integerPrecision(aBody, aPrecision, aValue == 0);
                if (isAlt && aValue != 0)
                {
                    if (aConv == 'o' && !aBody.StartsWith('0'))
                    {
                        aBody = "0" + aBody;
                    }
                    else if (aConv == 'x')
                    {
                        aSign = "0x";
                    }
                    else if (aConv == 'X')
                    {
                        aSign = "0X";
                    }
                }
                break;
            }
            default:
            {
                double aValue = Convert.ToDouble(theValue, CultureInfo.InvariantCulture);
                bool isNegative = double.IsNegative(aValue) && !double.IsNaN(aValue);
                aSign = isNegative ? "-" : isPlus ? "+" : isSpace ? " " : "";
                double anAbs = Math.Abs(aValue);
                int aPrec = aPrecision < 0 ? 6 : aPrecision;
                if (double.IsNaN(aValue))
                {
                    aBody = "nan";
                    isZero = false;
                }
                else if (double.IsInfinity(aValue))
                {
                    aBody = "inf";
                    isZero = false;
                }
                else if (aConv == 'f' || aConv == 'F')
                {
                    aBody = anAbs.ToString("F" + aPrec.ToString(CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);
                    if (isAlt && aPrec == 0)
                    {
                        aBody += ".";
                    }
                }
                else if (aConv == 'e' || aConv == 'E')
                {
                    aBody = exponent(anAbs, aPrec);
                }
                else
                {
                    aBody = Standard_Stream.FormatReal(anAbs, aPrec == 0 ? 1 : aPrec);
                }
                if (char.IsUpper(aConv))
                {
                    aBody = aBody.ToUpperInvariant();
                }
                break;
            }
        }

        int aPad = aWidth - aSign.Length - aBody.Length;
        if (aPad <= 0)
        {
            return aSign + aBody;
        }
        if (isLeft)
        {
            return aSign + aBody + new string(' ', aPad);
        }
        if (isNumeric && isZero && (aPrecision < 0 || aConv is 'f' or 'F' or 'e' or 'E' or 'g' or 'G'))
        {
            return aSign + new string('0', aPad) + aBody;
        }
        return new string(' ', aPad) + aSign + aBody;
    }

    private static string integerPrecision(string theDigits, int thePrecision, bool theIsZero)
    {
        if (thePrecision == 0 && theIsZero)
        {
            return "";
        }
        return thePrecision > theDigits.Length ? new string('0', thePrecision - theDigits.Length) + theDigits : theDigits;
    }

    // "%.*e" of a non-negative finite value: the mantissa and at least two exponent digits.
    private static string exponent(double theValue, int thePrecision)
    {
        string aText = theValue.ToString("e" + thePrecision.ToString(CultureInfo.InvariantCulture), CultureInfo.InvariantCulture);
        int anE = aText.IndexOf('e');
        int anExp = int.Parse(aText.AsSpan(anE + 1), NumberStyles.AllowLeadingSign, CultureInfo.InvariantCulture);
        string anExpText = Math.Abs(anExp).ToString("00", CultureInfo.InvariantCulture);
        return aText.Substring(0, anE) + (anExp < 0 ? "e-" : "e+") + anExpText;
    }
}
