// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;

namespace OcctNet.DrawLite;

/// <summary>One piece of a Tcl word: literal text, a variable, or a bracketed command.</summary>
public abstract record WordPart
{
    public sealed record Text(string Value) : WordPart;

    public sealed record Variable(string Name) : WordPart;

    public sealed record Command(string Script) : WordPart;
}

/// <summary>A word of a command. A braced word is one literal part and is never substituted.</summary>
public sealed record Word(IReadOnlyList<WordPart> Parts, bool IsBraced)
{
    /// <summary>The word's text when it has no substitutions, otherwise null.</summary>
    public string? Literal => Parts switch
    {
        [] => "",
        [WordPart.Text aText] => aText.Value,
        _ => null,
    };
}

/// <summary>A command: its words, and the line it starts on (for messages).</summary>
public sealed record Command(IReadOnlyList<Word> Words, int Line);

/// <summary>
/// Parses the subset of Tcl that OCCT's Draw test scripts use: words, "quotes", {braces}, [commands],
/// $variables, ${variables}, $array(index), backslash escapes and continuations, comments and semicolons.
/// </summary>
public static class TclParser
{
    public static IReadOnlyList<Command> Parse(string theScript)
    {
        var aCommands = new List<Command>();
        int aPos = 0;
        int aLine = 1;
        while (aPos < theScript.Length)
        {
            SkipCommandSeparators(theScript, ref aPos, ref aLine);
            if (aPos >= theScript.Length)
            {
                break;
            }

            if (theScript[aPos] == '#')
            {
                SkipComment(theScript, ref aPos, ref aLine);
                continue;
            }

            int aStartLine = aLine;
            var aWords = new List<Word>();
            while (aPos < theScript.Length)
            {
                SkipBlanks(theScript, ref aPos, ref aLine);
                if (aPos >= theScript.Length || theScript[aPos] is '\n' or ';')
                {
                    break;
                }

                aWords.Add(ParseWord(theScript, ref aPos, ref aLine));
            }

            if (aWords.Count > 0)
            {
                aCommands.Add(new Command(aWords, aStartLine));
            }
        }

        return aCommands;
    }

    private static Word ParseWord(string s, ref int aPos, ref int aLine)
    {
        char aFirst = s[aPos];
        if (aFirst == '{')
        {
            return new Word([new WordPart.Text(ReadBraced(s, ref aPos, ref aLine))], IsBraced: true);
        }

        bool aQuoted = aFirst == '"';
        if (aQuoted)
        {
            aPos++;
        }

        var aParts = new List<WordPart>();
        var aText = new StringBuilder();
        while (aPos < s.Length)
        {
            char c = s[aPos];
            if (aQuoted ? c == '"' : c is ' ' or '\t' or '\r' or '\n' or ';')
            {
                if (aQuoted)
                {
                    aPos++;
                }

                break;
            }

            switch (c)
            {
                case '\\':
                    aText.Append(ReadEscape(s, ref aPos, ref aLine));
                    break;
                case '$':
                    string? aName = ReadVariableName(s, ref aPos);
                    if (aName is null)
                    {
                        // A lone $ is literal
                        aText.Append('$');
                        aPos++;
                    }
                    else
                    {
                        Flush(aParts, aText);
                        aParts.Add(new WordPart.Variable(aName));
                    }

                    break;
                case '[':
                    Flush(aParts, aText);
                    aParts.Add(new WordPart.Command(ReadBracketed(s, ref aPos, ref aLine)));
                    break;
                default:
                    if (c == '\n')
                    {
                        aLine++;
                    }

                    aText.Append(c);
                    aPos++;
                    break;
            }
        }

        Flush(aParts, aText);
        return new Word(aParts, IsBraced: false);
    }

    private static void Flush(List<WordPart> theParts, StringBuilder theText)
    {
        if (theText.Length > 0)
        {
            theParts.Add(new WordPart.Text(theText.ToString()));
            theText.Clear();
        }
    }

    /// <summary>The text between matching braces, without them; nested braces and escaped braces are kept.</summary>
    private static string ReadBraced(string s, ref int aPos, ref int aLine)
    {
        int aDepth = 0;
        int aStart = aPos + 1;
        for (; aPos < s.Length; aPos++)
        {
            char c = s[aPos];
            if (c == '\\')
            {
                aPos++;
            }
            else if (c == '\n')
            {
                aLine++;
            }
            else if (c == '{')
            {
                aDepth++;
            }
            else if (c == '}' && --aDepth == 0)
            {
                return s[aStart..aPos++];
            }
        }

        throw new DrawException("missing close-brace", aLine);
    }

    /// <summary>The script between matching brackets, without them.</summary>
    private static string ReadBracketed(string s, ref int aPos, ref int aLine)
    {
        int aDepth = 0;
        int aStart = aPos + 1;
        for (; aPos < s.Length; aPos++)
        {
            char c = s[aPos];
            switch (c)
            {
                case '\\':
                    aPos++;
                    break;
                case '\n':
                    aLine++;
                    break;
                case '{':
                    ReadBraced(s, ref aPos, ref aLine);
                    aPos--;
                    break;
                case '[':
                    aDepth++;
                    break;
                case ']':
                    if (--aDepth == 0)
                    {
                        return s[aStart..aPos++];
                    }

                    break;
            }
        }

        throw new DrawException("missing close-bracket", aLine);
    }

    private static string? ReadVariableName(string s, ref int aPos)
    {
        int aStart = aPos + 1;
        if (aStart < s.Length && s[aStart] == '{')
        {
            int anEnd = s.IndexOf('}', aStart);
            if (anEnd < 0)
            {
                return null;
            }

            aPos = anEnd + 1;
            return s[(aStart + 1)..anEnd];
        }

        int i = aStart;
        while (i < s.Length && (char.IsLetterOrDigit(s[i]) || s[i] == '_' || (s[i] == ':' && i + 1 < s.Length && s[i + 1] == ':')))
        {
            i += s[i] == ':' ? 2 : 1;
        }

        if (i == aStart)
        {
            return null;
        }

        // An array element: $name(index), whose index is kept literally
        if (i < s.Length && s[i] == '(')
        {
            int anEnd = s.IndexOf(')', i);
            if (anEnd > 0)
            {
                aPos = anEnd + 1;
                return s[aStart..aPos];
            }
        }

        aPos = i;
        return s[aStart..i];
    }

    private static string ReadEscape(string s, ref int aPos, ref int aLine)
    {
        aPos++;
        if (aPos >= s.Length)
        {
            return "\\";
        }

        char c = s[aPos++];
        switch (c)
        {
            case 'n':
                return "\n";
            case 't':
                return "\t";
            case '\n':
                // A continuation: the newline and the next line's leading blanks become one space
                aLine++;
                while (aPos < s.Length && s[aPos] is ' ' or '\t')
                {
                    aPos++;
                }

                return " ";
            default:
                return c.ToString();
        }
    }

    private static void SkipBlanks(string s, ref int aPos, ref int aLine)
    {
        while (aPos < s.Length)
        {
            if (s[aPos] is ' ' or '\t' or '\r')
            {
                aPos++;
            }
            else if (s[aPos] == '\\' && aPos + 1 < s.Length && s[aPos + 1] == '\n')
            {
                aPos += 2;
                aLine++;
            }
            else
            {
                break;
            }
        }
    }

    private static void SkipCommandSeparators(string s, ref int aPos, ref int aLine)
    {
        while (aPos < s.Length && s[aPos] is ' ' or '\t' or '\r' or '\n' or ';')
        {
            if (s[aPos] == '\n')
            {
                aLine++;
            }

            aPos++;
        }
    }

    private static void SkipComment(string s, ref int aPos, ref int aLine)
    {
        while (aPos < s.Length && s[aPos] != '\n')
        {
            // A comment continues onto the next line after a backslash
            if (s[aPos] == '\\' && aPos + 1 < s.Length && s[aPos + 1] == '\n')
            {
                aPos++;
                aLine++;
            }

            aPos++;
        }
    }
}
