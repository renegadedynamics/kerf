// Port of: src/FoundationClasses/TKernel/TCollection/TCollection_HAsciiString.hxx
// Port of: src/FoundationClasses/TKernel/TCollection/TCollection_HAsciiString.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;

namespace Occt;

/// <summary>
/// A string held by handle (<c>occ::handle&lt;TCollection_HAsciiString&gt;</c>), which several objects can share:
/// data exchange keeps entity names, labels and parameter values in it.
/// </summary>
/// <remarks>
/// <para>
/// Unlike <c>TCollection_AsciiString</c>, which the port writes as <see cref="string"/>, this is a class: it has
/// identity and OCCT changes it in place (a <c>MoniTool_TypedValue</c>'s value is such a shared string). Its
/// mutators are internal (decision 21's rule for working objects); callers read it with <see cref="ToCString"/>,
/// <see cref="String"/> or <see cref="ToString"/>.
/// </para>
/// <para>
/// Characters are .NET's UTF-16 code units where OCCT counts bytes: the two agree on ASCII text, which is what the
/// exchange formats write. Indices are 1-based, as OCCT's. Only the members data exchange uses are ported.
/// </para>
/// </remarks>
public sealed class TCollection_HAsciiString : Standard_Transient
{
    private readonly StringBuilder myString;

    /// <summary>An empty string.</summary>
    public TCollection_HAsciiString() => myString = new StringBuilder();

    /// <summary>A copy of the text (a null text is empty, as OCCT's null <c>const char*</c> is refused earlier).</summary>
    public TCollection_HAsciiString(string? message) => myString = new StringBuilder(message ?? "");

    /// <summary>A one-character string (empty for the null character).</summary>
    public TCollection_HAsciiString(char aChar) => myString = new StringBuilder(aChar == '\0' ? "" : aChar.ToString());

    /// <summary>A string of <paramref name="length"/> copies of <paramref name="filler"/>.</summary>
    public TCollection_HAsciiString(int length, char filler) => myString = new StringBuilder(new string(filler, length));

    /// <summary>The integer in decimal (C's <c>%d</c>).</summary>
    public TCollection_HAsciiString(int value) =>
        myString = new StringBuilder(value.ToString(System.Globalization.CultureInfo.InvariantCulture));

    /// <summary>The real as C's <c>%g</c> writes it (six significant digits).</summary>
    public TCollection_HAsciiString(double value) => myString = new StringBuilder(Standard_Stream.FormatReal(value, 6));

    /// <summary>A copy of another handled string.</summary>
    public TCollection_HAsciiString(TCollection_HAsciiString aString) => myString = new StringBuilder(aString.ToCString());

    /// <summary>The text, as OCCT's <c>ToCString</c>.</summary>
    public string ToCString() => myString.ToString();

    /// <summary>The text, as OCCT's <c>String</c> (a <c>TCollection_AsciiString</c>, a string in the port).</summary>
    public string String() => myString.ToString();

    /// <summary>The text.</summary>
    public override string ToString() => myString.ToString();

    /// <summary>The number of characters.</summary>
    public int Length() => myString.Length;

    /// <summary>True if the string has no characters.</summary>
    public bool IsEmpty() => myString.Length == 0;

    /// <summary>The character at <paramref name="where"/> (1-based).</summary>
    /// <exception cref="Standard_OutOfRange">Out of 1 to Length.</exception>
    public char Value(int where)
    {
        if (where > 0 && where <= myString.Length)
        {
            return myString[where - 1];
        }
        throw new Standard_OutOfRange("TCollection_AsciiString::Value : parameter theWhere");
    }

    /// <summary>Appends a text.</summary>
    internal void AssignCat(string other) => myString.Append(other);

    /// <summary>Appends another string.</summary>
    internal void AssignCat(TCollection_HAsciiString other) => myString.Append(other.ToCString());

    /// <summary>A new string: this one followed by <paramref name="other"/>.</summary>
    public TCollection_HAsciiString Cat(string other) => new(ToCString() + other);

    /// <summary>A new string: this one followed by <paramref name="other"/>.</summary>
    public TCollection_HAsciiString Cat(TCollection_HAsciiString other) => new(ToCString() + other.ToCString());

    /// <summary>Empties the string.</summary>
    internal void Clear() => myString.Clear();

    /// <summary>Replaces the character at <paramref name="where"/> (1-based).</summary>
    internal void SetValue(int where, char what)
    {
        if (where <= 0 || where > myString.Length)
        {
            throw new Standard_OutOfRange("TCollection_AsciiString::SetValue(): out of range location");
        }
        myString[where - 1] = what;
    }

    /// <summary>Inserts a character at <paramref name="where"/> (1-based; Length + 1 appends).</summary>
    internal void Insert(int where, char what)
    {
        if (where > myString.Length + 1 || where < 1)
        {
            throw new Standard_OutOfRange("TCollection_AsciiString::Insert : Parameter where is out of range");
        }
        myString.Insert(where - 1, what);
    }

    /// <summary>Inserts a text at <paramref name="where"/> (1-based; Length + 1 appends).</summary>
    internal void Insert(int where, string what)
    {
        if (where > myString.Length + 1 || where < 1)
        {
            throw new Standard_OutOfRange("TCollection_AsciiString::Insert : Parameter where is out of range");
        }
        myString.Insert(where - 1, what);
    }

    /// <summary>Puts another string in front of this one.</summary>
    internal void Prepend(TCollection_HAsciiString other) => myString.Insert(0, other.ToCString());

    /// <summary>Removes <paramref name="ahowmany"/> characters from <paramref name="where"/> (1-based).</summary>
    internal void Remove(int where, int ahowmany = 1)
    {
        if (where + ahowmany <= myString.Length + 1)
        {
            myString.Remove(where - 1, ahowmany);
        }
        else
        {
            throw new Standard_OutOfRange("TCollection_AsciiString::Remove: Too many characters to erase or invalid starting value.");
        }
    }

    /// <summary>Removes every occurrence of the character.</summary>
    internal void RemoveAll(char what)
    {
        string aText = myString.ToString().Replace(what.ToString(), "", StringComparison.Ordinal);
        myString.Clear().Append(aText);
    }

    /// <summary>Keeps the first <paramref name="ahowmany"/> characters.</summary>
    internal void Trunc(int ahowmany)
    {
        if (ahowmany < 0 || ahowmany > myString.Length)
        {
            throw new Standard_OutOfRange("TCollection_AsciiString::Trunc : parameter 'theHowMany'");
        }
        myString.Length = ahowmany;
    }

    /// <summary>Removes the leading white space (C's <c>isspace</c>).</summary>
    internal void LeftAdjust()
    {
        int i = 0;
        while (i < myString.Length && IsSpace(myString[i]))
        {
            i++;
        }
        if (i > 0)
        {
            myString.Remove(0, i);
        }
    }

    /// <summary>Removes the trailing white space (C's <c>isspace</c>).</summary>
    internal void RightAdjust()
    {
        int i = myString.Length - 1;
        while (i >= 0 && IsSpace(myString[i]))
        {
            i--;
        }
        myString.Length = i + 1;
    }

    /// <summary>Converts to upper case (ASCII letters only, as C's <c>toupper</c> in the C locale).</summary>
    internal void UpperCase()
    {
        for (int i = 0; i < myString.Length; i++)
        {
            if (myString[i] is >= 'a' and <= 'z')
            {
                myString[i] = (char)(myString[i] - 32);
            }
        }
    }

    /// <summary>Converts to lower case (ASCII letters only, as C's <c>tolower</c> in the C locale).</summary>
    internal void LowerCase()
    {
        for (int i = 0; i < myString.Length; i++)
        {
            if (myString[i] is >= 'A' and <= 'Z')
            {
                myString[i] = (char)(myString[i] + 32);
            }
        }
    }

    /// <summary>True if both have the same characters (case-sensitive).</summary>
    /// <exception cref="Standard_NullObject"><paramref name="S"/> is null.</exception>
    public bool IsSameString(TCollection_HAsciiString? S)
    {
        if (S is null)
        {
            throw new Standard_NullObject("TCollection_HAsciiString::IsSameString");
        }
        return string.Equals(ToCString(), S.ToCString(), StringComparison.Ordinal);
    }

    /// <summary>True if both have the same characters, ignoring ASCII case if <paramref name="CaseSensitive"/> is false.</summary>
    /// <exception cref="Standard_NullObject"><paramref name="S"/> is null.</exception>
    public bool IsSameString(TCollection_HAsciiString? S, bool CaseSensitive)
    {
        if (S is null)
        {
            throw new Standard_NullObject("TCollection_HAsciiString::IsSameString");
        }
        return CaseSensitive
            ? string.Equals(ToCString(), S.ToCString(), StringComparison.Ordinal)
            : string.Equals(ToCString(), S.ToCString(), StringComparison.OrdinalIgnoreCase);
    }

    /// <summary>True if the strings differ.</summary>
    /// <exception cref="Standard_NullObject"><paramref name="S"/> is null.</exception>
    public bool IsDifferent(TCollection_HAsciiString? S)
    {
        if (S is null)
        {
            throw new Standard_NullObject("TCollection_HAsciiString::IsDifferent");
        }
        return !string.Equals(ToCString(), S.ToCString(), StringComparison.Ordinal);
    }

    /// <summary>True if this string is before <paramref name="other"/> in byte order.</summary>
    public bool IsLess(TCollection_HAsciiString other) => string.CompareOrdinal(ToCString(), other.ToCString()) < 0;

    /// <summary>True if this string is after <paramref name="other"/> in byte order.</summary>
    public bool IsGreater(TCollection_HAsciiString other) => string.CompareOrdinal(ToCString(), other.ToCString()) > 0;

    /// <summary>The integer the string starts with (C's <c>strtol</c>), 0 if none.</summary>
    public int IntegerValue() => TCollection_AsciiString.IntegerValue(ToCString());

    /// <summary>True if the string starts with an integer and holds no '.'.</summary>
    public bool IsIntegerValue() => TCollection_AsciiString.IsIntegerValue(ToCString());

    /// <summary>True if the string starts with a real (C's <c>strtod</c>).</summary>
    public bool IsRealValue() => TCollection_AsciiString.IsRealValue(ToCString());

    /// <summary>The real the string starts with (C's <c>strtod</c>).</summary>
    public double RealValue() => TCollection_AsciiString.RealValue(ToCString());

    /// <summary>The 1-based position of the first occurrence of <paramref name="what"/>, -1 if none (or if empty).</summary>
    public int Search(string what)
    {
        if (what.Length == 0)
        {
            return -1;
        }
        int anIndex = ToCString().IndexOf(what, StringComparison.Ordinal);
        return anIndex < 0 ? -1 : anIndex + 1;
    }

    /// <summary>The 1-based position of the first occurrence of <paramref name="what"/>, -1 if none.</summary>
    public int Search(TCollection_HAsciiString what) => Search(what.ToCString());

    /// <summary>The 1-based position of the last occurrence of <paramref name="what"/>, -1 if none (or if empty).</summary>
    public int SearchFromEnd(string what)
    {
        if (what.Length == 0)
        {
            return -1;
        }
        int anIndex = ToCString().LastIndexOf(what, StringComparison.Ordinal);
        return anIndex < 0 ? -1 : anIndex + 1;
    }

    /// <summary>The 1-based position of the last occurrence of <paramref name="what"/>, -1 if none.</summary>
    public int SearchFromEnd(TCollection_HAsciiString what) => SearchFromEnd(what.ToCString());

    /// <summary>
    /// The 1-based position of the first occurrence of <paramref name="S"/> starting between
    /// <paramref name="FromIndex"/> and <paramref name="ToIndex"/>, 0 if none (or if either string is empty).
    /// </summary>
    /// <exception cref="Standard_OutOfRange">The range isn't in the string.</exception>
    public int Location(TCollection_HAsciiString S, int FromIndex, int ToIndex)
    {
        if (Length() == 0 || S.Length() == 0)
        {
            return 0;
        }
        if (ToIndex > Length() || FromIndex <= 0 || FromIndex > ToIndex)
        {
            throw new Standard_OutOfRange();
        }
        string aText = ToCString();
        string aWhat = S.ToCString();
        // TCollection_AsciiString::Location
        int i = FromIndex - 1;
        int k = 1;
        int l = FromIndex - 2;
        bool aFind = false;
        while (!aFind && i < ToIndex)
        {
            if (aText[i] == aWhat[k - 1])
            {
                k++;
                if (k > aWhat.Length)
                {
                    aFind = true;
                }
            }
            else
            {
                if (k > 1)
                {
                    i--; // si on est en cours de recherche
                }
                k = 1;
                l = i;
            }
            i++;
        }
        return aFind ? l + 2 : 0;
    }

    /// <summary>A new string with the characters <paramref name="FromIndex"/> to <paramref name="ToIndex"/> (1-based, inclusive).</summary>
    /// <exception cref="Standard_OutOfRange">The range isn't in the string.</exception>
    public TCollection_HAsciiString SubString(int FromIndex, int ToIndex)
    {
        if (ToIndex > myString.Length || FromIndex <= 0 || FromIndex > ToIndex)
        {
            throw new Standard_OutOfRange("TCollection_AsciiString::SubString : parameters out of range");
        }
        return new TCollection_HAsciiString(myString.ToString(FromIndex - 1, ToIndex - FromIndex + 1));
    }

    /// <summary>The <paramref name="whichone"/>-th token between the separators (empty if none).</summary>
    public TCollection_HAsciiString Token(string separators = " \t", int whichone = 1) =>
        new(TCollection_AsciiString.Token(ToCString(), separators, whichone));

    /// <summary>The length without trailing non-graphic characters (non-ASCII characters count as graphic).</summary>
    public int UsefullLength()
    {
        int aLast = 0;
        for (int i = 0; i < myString.Length; i++)
        {
            char aChar = myString[i];
            if (aChar > 0x7F || (aChar > ' ' && aChar < 0x7F))
            {
                aLast = i + 1;
            }
        }
        return aLast;
    }

    /// <summary>True if both have the same length and characters (OCCT's <c>IsSameState</c>).</summary>
    public bool IsSameState(TCollection_HAsciiString other) =>
        string.Equals(ToCString(), other.ToCString(), StringComparison.Ordinal);

    /// <summary>Writes the text.</summary>
    public void Print(TextWriter astream) => astream.Write(ToCString());

    private static bool IsSpace(char theChar) => theChar is ' ' or '\t' or '\n' or '\v' or '\f' or '\r';
}
