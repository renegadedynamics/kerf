// Port of: src/FoundationClasses/TKernel/TCollection/TCollection_AsciiString.cxx (IntegerValue, IsIntegerValue,
//          IsRealValue, RealValue, Token)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The parts of OCCT's <c>TCollection_AsciiString</c> that ported code reads values with, on .NET strings (translation
/// rules: <c>TCollection_AsciiString</c> is <see cref="string"/>): the C library's <c>strtol</c> and <c>strtod</c>
/// conversions OCCT's <c>IntegerValue</c> and <c>RealValue</c> make, and <c>Token</c>.
/// </summary>
internal static class TCollection_AsciiString
{
    /// <summary>
    /// C's <c>strtol(text, &amp;end, 10)</c> with a 32-bit <c>long</c> (MSVC's): leading white space, a sign, decimal
    /// digits; out-of-range values clamp to the <c>long</c> range. <paramref name="theEnd"/> is the number of characters
    /// converted, 0 if none.
    /// </summary>
    internal static int Strtol(string theText, out int theEnd)
    {
        theEnd = 0;
        int i = 0;
        int aLength = theText.Length;
        while (i < aLength && IsSpace(theText[i]))
        {
            i++;
        }

        bool isNegative = false;
        if (i < aLength && theText[i] is '+' or '-')
        {
            isNegative = theText[i] == '-';
            i++;
        }

        int aFirstDigit = i;
        long aValue = 0;
        while (i < aLength && theText[i] is >= '0' and <= '9')
        {
            if (aValue <= (long)int.MaxValue + 1)
            {
                aValue = aValue * 10 + (theText[i] - '0');
            }
            i++;
        }

        if (i == aFirstDigit)
        {
            return 0;
        }

        theEnd = i;
        if (isNegative)
        {
            aValue = -aValue;
        }
        return (int)Math.Clamp(aValue, int.MinValue, int.MaxValue);
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

        throw new Standard_NumericError("TCollection_AsciiString::IntegerValue");
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
    /// OCCT's <c>IsRealValue</c>: true if the text starts with a real (strtod); with <paramref name="theToCheckFull"/>,
    /// if the whole text is one.
    /// </summary>
    internal static bool IsRealValue(string theString, bool theToCheckFull = false)
    {
        Standard_Stream.Strtod(theString, out int ptr);
        if (theToCheckFull)
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
        double value = Standard_Stream.Strtod(theString, out int ptr);
        if (ptr != 0)
        {
            return value;
        }

        throw new Standard_NumericError("TCollection_AsciiString::RealValue");
    }

    /// <summary>
    /// OCCT's <c>Token</c>: the <paramref name="theWhichOne"/>-th (1-based) run of characters not in
    /// <paramref name="theSeparators"/>, or "" if there are fewer.
    /// </summary>
    internal static string Token(string theString, string theSeparators, int theWhichOne = 1)
    {
        int myLength = theString.Length;
        int anIndex;
        int aStringIndex = 0;
        int aBeginIndex = 0;
        int anEndIndex = 0;

        for (anIndex = 0; anIndex < theWhichOne; anIndex++)
        {
            aBeginIndex = 0;
            anEndIndex = 0;
            if (aStringIndex == myLength)
            {
                break;
            }
            for (; aStringIndex < myLength && anEndIndex == 0; aStringIndex++)
            {
                if (theSeparators.Contains(theString[aStringIndex], StringComparison.Ordinal))
                { // We have a Separator
                    if (aBeginIndex != 0 && anEndIndex == 0)
                    {
                        anEndIndex = aStringIndex;
                        break;
                    }
                }
                else if (aBeginIndex == 0)
                { // We have not a Separator
                    aBeginIndex = aStringIndex + 1;
                }
            }
        }
        if (aBeginIndex == 0)
        {
            return "";
        }
        if (anEndIndex == 0)
        {
            anEndIndex = myLength;
        }
        return theString.Substring(aBeginIndex - 1, anEndIndex - aBeginIndex + 1);
    }
}
