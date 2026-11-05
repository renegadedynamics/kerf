// Port of: src/FoundationClasses/TKernel/TCollection/TCollection_HAsciiString.hxx
// Port of: src/FoundationClasses/TKernel/TCollection/TCollection_HAsciiString.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Text;

namespace Kerf.Engine;

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
internal sealed class TCollection_HAsciiString : Transient
{
    private readonly StringBuilder _string;

    /// <summary>An empty string.</summary>
    public TCollection_HAsciiString() => _string = new StringBuilder();

    /// <summary>A copy of the text (a null text is empty, as OCCT's null <c>const char*</c> is refused earlier).</summary>
    public TCollection_HAsciiString(string? message) => _string = new StringBuilder(message ?? "");

    /// <summary>A one-character string (empty for the null character).</summary>
    public TCollection_HAsciiString(char aChar) => _string = new StringBuilder(aChar == '\0' ? "" : aChar.ToString());

    /// <summary>A string of <paramref name="length"/> copies of <paramref name="filler"/>.</summary>
    public TCollection_HAsciiString(int length, char filler) => _string = new StringBuilder(new string(filler, length));

    /// <summary>The integer in decimal (C's <c>%d</c>).</summary>
    public TCollection_HAsciiString(int value) =>
        _string = new StringBuilder(value.ToString(System.Globalization.CultureInfo.InvariantCulture));

    /// <summary>The real as C's <c>%g</c> writes it (six significant digits).</summary>
    public TCollection_HAsciiString(double value) => _string = new StringBuilder(StreamUtilities.FormatReal(value, 6));

    /// <summary>A copy of another handled string.</summary>
    public TCollection_HAsciiString(TCollection_HAsciiString aString) => _string = new StringBuilder(aString.ToCString());

    /// <summary>The text, as OCCT's <c>ToCString</c>.</summary>
    public string ToCString() => _string.ToString();

    /// <summary>The text, as OCCT's <c>String</c> (a <c>TCollection_AsciiString</c>, a string in the port).</summary>
    public string String() => _string.ToString();

    /// <summary>The text.</summary>
    public override string ToString() => _string.ToString();

    /// <summary>The number of characters.</summary>
    public int Length() => _string.Length;

    /// <summary>True if the string has no characters.</summary>
    public bool IsEmpty() => _string.Length == 0;

    /// <summary>The character at <paramref name="where"/> (1-based).</summary>
    /// <exception cref="OutOfRangeException">Out of 1 to Length.</exception>
    public char Value(int where)
    {
        if (where > 0 && where <= _string.Length)
        {
            return _string[where - 1];
        }
        throw new OutOfRangeException("TCollection_AsciiString::Value : parameter theWhere");
    }

    /// <summary>Appends a text.</summary>
    internal void AssignCat(string other) => _string.Append(other);

    /// <summary>Appends another string.</summary>
    internal void AssignCat(TCollection_HAsciiString other) => _string.Append(other.ToCString());

    /// <summary>A new string: this one followed by <paramref name="other"/>.</summary>
    public TCollection_HAsciiString Cat(string other) => new(ToCString() + other);

    /// <summary>A new string: this one followed by <paramref name="other"/>.</summary>
    public TCollection_HAsciiString Cat(TCollection_HAsciiString other) => new(ToCString() + other.ToCString());

    /// <summary>Empties the string.</summary>
    internal void Clear() => _string.Clear();

    /// <summary>Replaces the character at <paramref name="where"/> (1-based).</summary>
    internal void SetValue(int where, char what)
    {
        if (where <= 0 || where > _string.Length)
        {
            throw new OutOfRangeException("TCollection_AsciiString::SetValue(): out of range location");
        }
        _string[where - 1] = what;
    }

    /// <summary>Inserts a character at <paramref name="where"/> (1-based; Length + 1 appends).</summary>
    internal void Insert(int where, char what)
    {
        if (where > _string.Length + 1 || where < 1)
        {
            throw new OutOfRangeException("TCollection_AsciiString::Insert : Parameter where is out of range");
        }
        _string.Insert(where - 1, what);
    }

    /// <summary>Inserts a text at <paramref name="where"/> (1-based; Length + 1 appends).</summary>
    internal void Insert(int where, string what)
    {
        if (where > _string.Length + 1 || where < 1)
        {
            throw new OutOfRangeException("TCollection_AsciiString::Insert : Parameter where is out of range");
        }
        _string.Insert(where - 1, what);
    }

    /// <summary>Puts another string in front of this one.</summary>
    internal void Prepend(TCollection_HAsciiString other) => _string.Insert(0, other.ToCString());

    /// <summary>Removes <paramref name="ahowmany"/> characters from <paramref name="where"/> (1-based).</summary>
    internal void Remove(int where, int ahowmany = 1)
    {
        if (where + ahowmany <= _string.Length + 1)
        {
            _string.Remove(where - 1, ahowmany);
        }
        else
        {
            throw new OutOfRangeException("TCollection_AsciiString::Remove: Too many characters to erase or invalid starting value.");
        }
    }

    /// <summary>Removes every occurrence of the character.</summary>
    internal void RemoveAll(char what)
    {
        string text = _string.ToString().Replace(what.ToString(), "", StringComparison.Ordinal);
        _string.Clear().Append(text);
    }

    /// <summary>Keeps the first <paramref name="ahowmany"/> characters.</summary>
    internal void Trunc(int ahowmany)
    {
        if (ahowmany < 0 || ahowmany > _string.Length)
        {
            throw new OutOfRangeException("TCollection_AsciiString::Trunc : parameter 'theHowMany'");
        }
        _string.Length = ahowmany;
    }

    /// <summary>Removes the leading white space (C's <c>isspace</c>).</summary>
    internal void LeftAdjust()
    {
        int i = 0;
        while (i < _string.Length && IsSpace(_string[i]))
        {
            i++;
        }
        if (i > 0)
        {
            _string.Remove(0, i);
        }
    }

    /// <summary>Removes the trailing white space (C's <c>isspace</c>).</summary>
    internal void RightAdjust()
    {
        int i = _string.Length - 1;
        while (i >= 0 && IsSpace(_string[i]))
        {
            i--;
        }
        _string.Length = i + 1;
    }

    /// <summary>Converts to upper case (ASCII letters only, as C's <c>toupper</c> in the C locale).</summary>
    internal void UpperCase()
    {
        for (int i = 0; i < _string.Length; i++)
        {
            if (_string[i] is >= 'a' and <= 'z')
            {
                _string[i] = (char)(_string[i] - 32);
            }
        }
    }

    /// <summary>Converts to lower case (ASCII letters only, as C's <c>tolower</c> in the C locale).</summary>
    internal void LowerCase()
    {
        for (int i = 0; i < _string.Length; i++)
        {
            if (_string[i] is >= 'A' and <= 'Z')
            {
                _string[i] = (char)(_string[i] + 32);
            }
        }
    }

    /// <summary>True if both have the same characters (case-sensitive).</summary>
    /// <exception cref="NullObjectException"><paramref name="s"/> is null.</exception>
    public bool IsSameString(TCollection_HAsciiString? s)
    {
        if (s is null)
        {
            throw new NullObjectException("TCollection_HAsciiString::IsSameString");
        }
        return string.Equals(ToCString(), s.ToCString(), StringComparison.Ordinal);
    }

    /// <summary>True if both have the same characters, ignoring ASCII case if <paramref name="caseSensitive"/> is false.</summary>
    /// <exception cref="NullObjectException"><paramref name="s"/> is null.</exception>
    public bool IsSameString(TCollection_HAsciiString? s, bool caseSensitive)
    {
        if (s is null)
        {
            throw new NullObjectException("TCollection_HAsciiString::IsSameString");
        }
        return caseSensitive
            ? string.Equals(ToCString(), s.ToCString(), StringComparison.Ordinal)
            : string.Equals(ToCString(), s.ToCString(), StringComparison.OrdinalIgnoreCase);
    }

    /// <summary>True if the strings differ.</summary>
    /// <exception cref="NullObjectException"><paramref name="s"/> is null.</exception>
    public bool IsDifferent(TCollection_HAsciiString? s)
    {
        if (s is null)
        {
            throw new NullObjectException("TCollection_HAsciiString::IsDifferent");
        }
        return !string.Equals(ToCString(), s.ToCString(), StringComparison.Ordinal);
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
        int index = ToCString().IndexOf(what, StringComparison.Ordinal);
        return index < 0 ? -1 : index + 1;
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
        int index = ToCString().LastIndexOf(what, StringComparison.Ordinal);
        return index < 0 ? -1 : index + 1;
    }

    /// <summary>The 1-based position of the last occurrence of <paramref name="what"/>, -1 if none.</summary>
    public int SearchFromEnd(TCollection_HAsciiString what) => SearchFromEnd(what.ToCString());

    /// <summary>
    /// The 1-based position of the first occurrence of <paramref name="s"/> starting between
    /// <paramref name="fromIndex"/> and <paramref name="toIndex"/>, 0 if none (or if either string is empty).
    /// </summary>
    /// <exception cref="OutOfRangeException">The range isn't in the string.</exception>
    public int Location(TCollection_HAsciiString s, int fromIndex, int toIndex)
    {
        if (Length() == 0 || s.Length() == 0)
        {
            return 0;
        }
        if (toIndex > Length() || fromIndex <= 0 || fromIndex > toIndex)
        {
            throw new OutOfRangeException();
        }
        string text = ToCString();
        string what = s.ToCString();
        // TCollection_AsciiString::Location
        int i = fromIndex - 1;
        int k = 1;
        int l = fromIndex - 2;
        bool find = false;
        while (!find && i < toIndex)
        {
            if (text[i] == what[k - 1])
            {
                k++;
                if (k > what.Length)
                {
                    find = true;
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
        return find ? l + 2 : 0;
    }

    /// <summary>A new string with the characters <paramref name="fromIndex"/> to <paramref name="toIndex"/> (1-based, inclusive).</summary>
    /// <exception cref="OutOfRangeException">The range isn't in the string.</exception>
    public TCollection_HAsciiString SubString(int fromIndex, int toIndex)
    {
        if (toIndex > _string.Length || fromIndex <= 0 || fromIndex > toIndex)
        {
            throw new OutOfRangeException("TCollection_AsciiString::SubString : parameters out of range");
        }
        return new TCollection_HAsciiString(_string.ToString(fromIndex - 1, toIndex - fromIndex + 1));
    }

    /// <summary>The <paramref name="whichone"/>-th token between the separators (empty if none).</summary>
    public TCollection_HAsciiString Token(string separators = " \t", int whichone = 1) =>
        new(TCollection_AsciiString.Token(ToCString(), separators, whichone));

    /// <summary>The length without trailing non-graphic characters (non-ASCII characters count as graphic).</summary>
    public int UsefullLength()
    {
        int last = 0;
        for (int i = 0; i < _string.Length; i++)
        {
            char aChar = _string[i];
            if (aChar > 0x7F || (aChar > ' ' && aChar < 0x7F))
            {
                last = i + 1;
            }
        }
        return last;
    }

    /// <summary>True if both have the same length and characters (OCCT's <c>IsSameState</c>).</summary>
    public bool IsSameState(TCollection_HAsciiString other) =>
        string.Equals(ToCString(), other.ToCString(), StringComparison.Ordinal);

    /// <summary>Writes the text.</summary>
    public void Print(TextWriter astream) => astream.Write(ToCString());

    private static bool IsSpace(char theChar) => theChar is ' ' or '\t' or '\n' or '\v' or '\f' or '\r';
}
