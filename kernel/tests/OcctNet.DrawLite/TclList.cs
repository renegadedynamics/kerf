// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using System.Text;

namespace OcctNet.DrawLite;

/// <summary>Tcl lists: reading a string as a list, writing a list in canonical form, and list indices.</summary>
public static class TclList
{
    // Values flow through variables unchanged, so the same string instance is often read as a list many times
    private static readonly ConditionalWeakTable<string, string[]> s_parsed = new();

    /// <summary>The elements of a list, as Tcl reads them (braces, quotes and backslashes).</summary>
    public static IReadOnlyList<string> Parse(string theList)
    {
        if (theList.Length == 0)
        {
            return [];
        }

        if (s_parsed.TryGetValue(theList, out string[]? aCached))
        {
            return aCached;
        }

        string[] anElements = ParseUncached(theList);
        if (theList.Length > 16)
        {
            s_parsed.AddOrUpdate(theList, anElements);
        }

        return anElements;
    }

    /// <summary>True when a string reads as a list (string is list).</summary>
    public static bool IsList(string theText)
    {
        try
        {
            Parse(theText);
            return true;
        }
        catch (DrawException)
        {
            return false;
        }
    }

    private static string[] ParseUncached(string s)
    {
        var anElements = new List<string>();
        int aPos = 0;
        while (true)
        {
            while (aPos < s.Length && IsSpace(s[aPos]))
            {
                aPos++;
            }

            if (aPos >= s.Length)
            {
                break;
            }

            char c = s[aPos];
            if (c == '{')
            {
                int aDepth = 1;
                int aStart = ++aPos;
                while (aPos < s.Length && aDepth > 0)
                {
                    char d = s[aPos];
                    if (d == '\\')
                    {
                        aPos++;
                    }
                    else if (d == '{')
                    {
                        aDepth++;
                    }
                    else if (d == '}')
                    {
                        aDepth--;
                    }

                    aPos++;
                }

                if (aDepth > 0)
                {
                    throw new DrawException("unmatched open brace in list");
                }

                anElements.Add(s[aStart..(aPos - 1)]);
                if (aPos < s.Length && !IsSpace(s[aPos]))
                {
                    throw new DrawException($"list element in braces followed by \"{Tail(s, aPos)}\" instead of space");
                }
            }
            else if (c == '"')
            {
                aPos++;
                var aBuilder = new StringBuilder();
                int aLine = 0;
                while (aPos < s.Length && s[aPos] != '"')
                {
                    if (s[aPos] == '\\')
                    {
                        aBuilder.Append(TclParser.ReadEscape(s, ref aPos, ref aLine));
                    }
                    else
                    {
                        aBuilder.Append(s[aPos++]);
                    }
                }

                if (aPos >= s.Length)
                {
                    throw new DrawException("unmatched open quote in list");
                }

                aPos++;
                anElements.Add(aBuilder.ToString());
                if (aPos < s.Length && !IsSpace(s[aPos]))
                {
                    throw new DrawException($"list element in quotes followed by \"{Tail(s, aPos)}\" instead of space");
                }
            }
            else
            {
                int aStart = aPos;
                bool anEscaped = false;
                while (aPos < s.Length && !IsSpace(s[aPos]))
                {
                    if (s[aPos] == '\\')
                    {
                        anEscaped = true;
                        aPos++;
                    }

                    aPos++;
                }

                aPos = Math.Min(aPos, s.Length);
                if (!anEscaped)
                {
                    anElements.Add(s[aStart..aPos]);
                }
                else
                {
                    var aBuilder = new StringBuilder();
                    int aLine = 0;
                    int i = aStart;
                    while (i < aPos)
                    {
                        if (s[i] == '\\')
                        {
                            aBuilder.Append(TclParser.ReadEscape(s, ref i, ref aLine));
                        }
                        else
                        {
                            aBuilder.Append(s[i++]);
                        }
                    }

                    anElements.Add(aBuilder.ToString());
                }
            }
        }

        return [.. anElements];
    }

    private static string Tail(string s, int thePos) => s.Substring(thePos, Math.Min(1, s.Length - thePos));

    private static bool IsSpace(char c) => c is ' ' or '\t' or '\n' or '\r' or '\v' or '\f';

    /// <summary>A list's canonical string: elements separated by spaces, each quoted as Tcl's list command does.</summary>
    public static string Format(IEnumerable<string> theElements)
    {
        var aBuilder = new StringBuilder();
        bool aFirst = true;
        foreach (string anElement in theElements)
        {
            if (!aFirst)
            {
                aBuilder.Append(' ');
            }

            AppendElement(aBuilder, anElement, aFirst);
            aFirst = false;
        }

        return aBuilder.ToString();
    }

    /// <summary>A list's canonical string, remembered as the parse of that string (the elements are not copied).</summary>
    public static string Make(string[] theElements)
    {
        string aList = Format(theElements);
        if (aList.Length > 16)
        {
            s_parsed.AddOrUpdate(aList, theElements);
        }

        return aList;
    }

    /// <summary>Appends elements to a list (lappend), giving the canonical string of the longer list.</summary>
    public static string Append(string theList, IReadOnlyList<string> theElements)
    {
        IReadOnlyList<string> anOld = Parse(theList);
        var aNew = new string[anOld.Count + theElements.Count];
        for (int i = 0; i < anOld.Count; i++)
        {
            aNew[i] = anOld[i];
        }

        for (int i = 0; i < theElements.Count; i++)
        {
            aNew[anOld.Count + i] = theElements[i];
        }

        return Make(aNew);
    }

    /// <summary>One element quoted so that it reads back as itself.</summary>
    public static string Quote(string theElement, bool theIsFirst = true)
    {
        var aBuilder = new StringBuilder();
        AppendElement(aBuilder, theElement, theIsFirst);
        return aBuilder.ToString();
    }

    private static void AppendElement(StringBuilder theBuilder, string s, bool theIsFirst)
    {
        if (s.Length == 0)
        {
            theBuilder.Append("{}");
            return;
        }

        bool aNeedsQuoting = s[0] is '{' or '"' || (theIsFirst && s[0] == '#');
        bool aCanBrace = true;
        int aDepth = 0;
        for (int i = 0; i < s.Length; i++)
        {
            switch (s[i])
            {
                case '{':
                    aDepth++;
                    aNeedsQuoting = true;
                    break;
                case '}':
                    aDepth--;
                    aNeedsQuoting = true;
                    if (aDepth < 0)
                    {
                        aCanBrace = false;
                    }

                    break;
                case '\\':
                    aNeedsQuoting = true;
                    if (i + 1 >= s.Length || s[i + 1] == '\n')
                    {
                        aCanBrace = false;
                    }

                    i++;
                    break;
                case '[' or ']' or '$' or ';' or '"' or ' ' or '\t' or '\n' or '\r' or '\v' or '\f':
                    aNeedsQuoting = true;
                    break;
            }
        }

        if (aDepth != 0)
        {
            aCanBrace = false;
        }

        if (!aNeedsQuoting)
        {
            theBuilder.Append(s);
        }
        else if (aCanBrace)
        {
            theBuilder.Append('{').Append(s).Append('}');
        }
        else
        {
            for (int i = 0; i < s.Length; i++)
            {
                char c = s[i];
                switch (c)
                {
                    case '{' or '}' or '[' or ']' or '$' or ';' or '"' or '\\' or ' ':
                        theBuilder.Append('\\').Append(c);
                        break;
                    case '#' when i == 0 && theIsFirst:
                        theBuilder.Append("\\#");
                        break;
                    case '\n':
                        theBuilder.Append("\\n");
                        break;
                    case '\t':
                        theBuilder.Append("\\t");
                        break;
                    case '\r':
                        theBuilder.Append("\\r");
                        break;
                    case '\v':
                        theBuilder.Append("\\v");
                        break;
                    case '\f':
                        theBuilder.Append("\\f");
                        break;
                    default:
                        theBuilder.Append(c);
                        break;
                }
            }
        }
    }

    /// <summary>
    /// A list index: an integer, end, end-N, end+N, or M+N / M-N, resolved against a list of
    /// <paramref name="theCount"/> elements. The result may be out of range; callers decide what that means.
    /// </summary>
    public static long ParseIndex(string theIndex, int theCount)
    {
        string s = theIndex.Trim();
        if (TclNumber.TryParseInt(s, out long aValue))
        {
            return aValue;
        }

        long aBase = 0;
        string aRest = s;
        if (s.StartsWith("end", StringComparison.Ordinal))
        {
            aBase = theCount - 1;
            aRest = s[3..];
            if (aRest.Length == 0)
            {
                return aBase;
            }
        }
        else
        {
            int anOp = s.IndexOfAny(['+', '-'], 1);
            if (anOp > 0 && TclNumber.TryParseInt(s[..anOp], out long aLeft))
            {
                aBase = aLeft;
                aRest = s[anOp..];
            }
            else
            {
                throw BadIndex(theIndex);
            }
        }

        if (aRest.Length > 1 && aRest[0] is '+' or '-' && aRest[1] is not ('+' or '-') && TclNumber.TryParseInt(aRest[1..], out long anOffset))
        {
            return aRest[0] == '+' ? aBase + anOffset : aBase - anOffset;
        }

        throw BadIndex(theIndex);
    }

    private static DrawException BadIndex(string theIndex)
        => new($"bad index \"{theIndex}\": must be integer?[+-]integer? or end?[+-]integer?");
}
