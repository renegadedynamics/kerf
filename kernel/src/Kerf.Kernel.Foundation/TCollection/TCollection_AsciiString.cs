// Port of: src/FoundationClasses/TKernel/TCollection/TCollection_AsciiString.cxx (IntegerValue, IsIntegerValue,
//          IsRealValue, RealValue, Token)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Engine;

/// <summary>
/// The parts of OCCT's <c>TCollection_AsciiString</c> that ported code reads values with, on .NET strings (translation
/// rules: <c>TCollection_AsciiString</c> is <see cref="string"/>): the C library's <c>strtol</c> and <c>strtod</c>
/// conversions OCCT's <c>IntegerValue</c> and <c>RealValue</c> make, and <c>Token</c>.
/// </summary>
internal static class TCollection_AsciiString
{
    /// <summary>
    /// C's <c>strtol(text, &amp;end, 10)</c> with a 32-bit <c>long</c> (MSVC's): leading white space, a sign, decimal
    /// digits; out-of-range values clamp to the <c>long</c> range. <paramref name="end"/> is the number of characters
    /// converted, 0 if none.
    /// </summary>
    internal static int Strtol(string text, out int end)
    {
        end = 0;
        int i = 0;
        int length = text.Length;
        while (i < length && IsSpace(text[i]))
        {
            i++;
        }

        bool isNegative = false;
        if (i < length && text[i] is '+' or '-')
        {
            isNegative = text[i] == '-';
            i++;
        }

        int firstDigit = i;
        long value = 0;
        while (i < length && text[i] is >= '0' and <= '9')
        {
            if (value <= (long)int.MaxValue + 1)
            {
                value = value * 10 + (text[i] - '0');
            }
            i++;
        }

        if (i == firstDigit)
        {
            return 0;
        }

        end = i;
        if (isNegative)
        {
            value = -value;
        }
        return (int)Math.Clamp(value, int.MinValue, int.MaxValue);
    }

    private static bool IsSpace(char theChar) => theChar is ' ' or '\t' or '\n' or '\v' or '\f' or '\r';

    /// <summary>
    /// OCCT's <c>IntegerValue</c>: the integer the text starts with (strtol); Standard_NumericError if it starts with
    /// none.
    /// </summary>
    internal static int IntegerValue(string theString)
    {
        int value = Strtol(theString, out int ptr);
        if (ptr != 0)
        {
            return value;
        }

        throw new NumericErrorException("TCollection_AsciiString::IntegerValue");
    }

    /// <summary>
    /// OCCT's <c>IsIntegerValue</c>: true if the text starts with an integer (strtol) and no '.' follows it anywhere.
    /// </summary>
    internal static bool IsIntegerValue(string theString)
    {
        Strtol(theString, out int ptr);

        if (ptr != 0)
        {
            for (int i = ptr; i < theString.Length; i++)
            {
                if (theString[i] == '.')
                {
                    return false; // what about 'e','x',etc ???
                }
            }
            return true;
        }
        return false;
    }

    /// <summary>
    /// OCCT's <c>IsRealValue</c>: true if the text starts with a real (strtod); with <paramref name="toCheckFull"/>,
    /// if the whole text is one.
    /// </summary>
    internal static bool IsRealValue(string theString, bool toCheckFull = false)
    {
        StreamUtilities.Strtod(theString, out int ptr);
        if (toCheckFull)
        {
            return ptr == theString.Length;
        }
        else
        {
            return ptr != 0;
        }
    }

    /// <summary>
    /// OCCT's <c>RealValue</c>: the real the text starts with (strtod); Standard_NumericError if it starts with none.
    /// </summary>
    internal static double RealValue(string theString)
    {
        double value = StreamUtilities.Strtod(theString, out int ptr);
        if (ptr != 0)
        {
            return value;
        }

        throw new NumericErrorException("TCollection_AsciiString::RealValue");
    }

    /// <summary>
    /// OCCT's <c>Token</c>: the <paramref name="whichOne"/>-th (1-based) run of characters not in
    /// <paramref name="separators"/>, or "" if there are fewer.
    /// </summary>
    internal static string Token(string theString, string separators, int whichOne = 1)
    {
        int myLength = theString.Length;
        int index;
        int stringIndex = 0;
        int beginIndex = 0;
        int endIndex = 0;

        for (index = 0; index < whichOne; index++)
        {
            beginIndex = 0;
            endIndex = 0;
            if (stringIndex == myLength)
            {
                break;
            }
            for (; stringIndex < myLength && endIndex == 0; stringIndex++)
            {
                if (separators.Contains(theString[stringIndex], StringComparison.Ordinal))
                { // We have a Separator
                    if (beginIndex != 0 && endIndex == 0)
                    {
                        endIndex = stringIndex;
                        break;
                    }
                }
                else if (beginIndex == 0)
                { // We have not a Separator
                    beginIndex = stringIndex + 1;
                }
            }
        }
        if (beginIndex == 0)
        {
            return "";
        }
        if (endIndex == 0)
        {
            endIndex = myLength;
        }
        return theString.Substring(beginIndex - 1, endIndex - beginIndex + 1);
    }
}
