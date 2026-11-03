// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;

namespace Kerf.Kernel.DrawLite;

/// <summary>One piece of a Tcl word: literal text, a variable, or a bracketed command.</summary>
public abstract record WordPart
{
    public sealed record Text(string Value) : WordPart;

    /// <summary>
    /// <c>$name</c>, <c>${name}</c> or <c>$name(index)</c>. The index of an array element is itself substituted,
    /// so it is kept as parts; it is null for a scalar.
    /// </summary>
    public sealed record Variable(string Name, IReadOnlyList<WordPart>? Index = null) : WordPart;

    public sealed record Command(string Script) : WordPart;
}

/// <summary>
/// A word of a command. A braced word is one literal part and is never substituted. An expanded word
/// (<c>{*}$list</c>) contributes each element of its value as a separate word.
/// </summary>
public sealed record Word(IReadOnlyList<WordPart> Parts, bool IsBraced, bool IsExpanded = false)
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
/// Parses Tcl scripts by the rules of Tcl's man page (the "dodekalogue"): words, "quotes", {braces}, [commands],
/// $variables, ${variables}, $array(index), {*} expansion, backslash escapes and continuations, comments and
/// semicolons. Line ends may be CRLF.
/// </summary>
public static class TclParser
{
    public static IReadOnlyList<Command> Parse(string theScript)
    {
        string s = theScript.Contains('\r') ? theScript.Replace("\r\n", "\n") : theScript;
        int aPos = 0;
        int aLine = 1;
        return ParseScript(s, ref aPos, ref aLine, theNested: false);
    }

    /// <summary>
    /// The parts of a string after <c>subst</c>'s substitutions, as if it were a quoted word with no closing quote.
    /// </summary>
    public static IReadOnlyList<WordPart> ParseSubst(string theText, bool theBackslashes = true, bool theVariables = true, bool theCommands = true)
    {
        var aParts = new List<WordPart>();
        var aText = new StringBuilder();
        int aPos = 0;
        int aLine = 1;
        string s = theText;
        while (aPos < s.Length)
        {
            char c = s[aPos];
            if (c == '\\' && theBackslashes)
            {
                aText.Append(ReadEscape(s, ref aPos, ref aLine));
            }
            else if (c == '$' && theVariables && ReadVariable(s, ref aPos, ref aLine) is { } aVariable)
            {
                Flush(aParts, aText);
                aParts.Add(aVariable);
            }
            else if (c == '[' && theCommands)
            {
                Flush(aParts, aText);
                aParts.Add(new WordPart.Command(ReadBracketed(s, ref aPos, ref aLine)));
            }
            else
            {
                aText.Append(c);
                aPos++;
            }
        }

        Flush(aParts, aText);
        return aParts;
    }

    /// <summary>True when a script is complete: its braces, quotes and brackets are closed.</summary>
    public static bool IsComplete(string theScript)
    {
        try
        {
            Parse(theScript);
            return true;
        }
        catch (DrawException)
        {
            return false;
        }
    }

    private static List<Command> ParseScript(string s, ref int aPos, ref int aLine, bool theNested)
    {
        var aCommands = new List<Command>();
        while (aPos < s.Length)
        {
            SkipCommandSeparators(s, ref aPos, ref aLine);
            if (aPos >= s.Length || (theNested && s[aPos] == ']'))
            {
                break;
            }

            if (s[aPos] == '#')
            {
                SkipComment(s, ref aPos, ref aLine);
                continue;
            }

            int aStartLine = aLine;
            var aWords = new List<Word>();
            while (aPos < s.Length)
            {
                SkipBlanks(s, ref aPos, ref aLine);
                if (aPos >= s.Length || s[aPos] is '\n' or ';' || (theNested && s[aPos] == ']'))
                {
                    break;
                }

                aWords.Add(ParseWord(s, ref aPos, ref aLine, theNested));
            }

            if (aWords.Count > 0)
            {
                aCommands.Add(new Command(aWords, aStartLine));
            }

            if (theNested && aPos < s.Length && s[aPos] == ']')
            {
                break;
            }
        }

        return aCommands;
    }

    private static Word ParseWord(string s, ref int aPos, ref int aLine, bool theNested)
    {
        char aFirst = s[aPos];
        if (aFirst == '{')
        {
            // {*} followed by a word expands that word's list into separate words
            if (aPos + 3 < s.Length && s[aPos + 1] == '*' && s[aPos + 2] == '}' && !IsWordEnd(s[aPos + 3], theNested))
            {
                aPos += 3;
                Word anInner = ParseWord(s, ref aPos, ref aLine, theNested);
                return anInner with { IsExpanded = true };
            }

            var aBraced = new Word([new WordPart.Text(ReadBraced(s, ref aPos, ref aLine))], IsBraced: true);
            if (aPos < s.Length && !IsWordEnd(s[aPos], theNested))
            {
                throw new DrawException("extra characters after close-brace", aLine);
            }

            return aBraced;
        }

        bool aQuoted = aFirst == '"';
        if (aQuoted)
        {
            aPos++;
        }

        var aParts = new List<WordPart>();
        var aText = new StringBuilder();
        bool aClosed = !aQuoted;
        while (aPos < s.Length)
        {
            char c = s[aPos];
            if (aQuoted ? c == '"' : IsWordEnd(c, theNested))
            {
                if (aQuoted)
                {
                    aPos++;
                    aClosed = true;
                }

                break;
            }

            // A backslash-newline ends a bare word: Tcl replaces it (and the blanks after it) by a space before it
            // splits the command into words (unit 7G: draft/angle/H6's "15 0 0 1\" rows read as one word "1 0").
            if (!aQuoted && c == '\\' && aPos + 1 < s.Length && s[aPos + 1] == '\n')
            {
                break;
            }

            switch (c)
            {
                case '\\':
                    aText.Append(ReadEscape(s, ref aPos, ref aLine));
                    break;
                case '$':
                    if (ReadVariable(s, ref aPos, ref aLine) is { } aVariable)
                    {
                        Flush(aParts, aText);
                        aParts.Add(aVariable);
                    }
                    else
                    {
                        // A lone $ is literal
                        aText.Append('$');
                        aPos++;
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

        if (!aClosed)
        {
            throw new DrawException("missing \"", aLine);
        }

        if (aQuoted && aPos < s.Length && !IsWordEnd(s[aPos], theNested))
        {
            throw new DrawException("extra characters after close-quote", aLine);
        }

        Flush(aParts, aText);
        return new Word(aParts, IsBraced: false);
    }

    private static bool IsWordEnd(char c, bool theNested) => c is ' ' or '\t' or '\r' or '\n' or ';' || (theNested && c == ']');

    private static void Flush(List<WordPart> theParts, StringBuilder theText)
    {
        if (theText.Length > 0)
        {
            theParts.Add(new WordPart.Text(theText.ToString()));
            theText.Clear();
        }
    }

    /// <summary>
    /// The text between matching braces, without them; nested braces and escaped braces are kept. As in Tcl, a
    /// backslash-newline (and the blanks after it) becomes one space even inside braces.
    /// </summary>
    internal static string ReadBraced(string s, ref int aPos, ref int aLine)
    {
        int aDepth = 0;
        int aStart = aPos + 1;
        StringBuilder? aBuilder = null;
        int aCopied = aStart;
        for (; aPos < s.Length; aPos++)
        {
            char c = s[aPos];
            if (c == '\\')
            {
                if (aPos + 1 < s.Length && s[aPos + 1] == '\n')
                {
                    aBuilder ??= new StringBuilder();
                    aBuilder.Append(s, aCopied, aPos - aCopied).Append(' ');
                    aPos += 2;
                    aLine++;
                    while (aPos < s.Length && s[aPos] is ' ' or '\t')
                    {
                        aPos++;
                    }

                    aCopied = aPos;
                    aPos--;
                }
                else
                {
                    aPos++;
                }
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
                string aText = aBuilder is null ? s[aStart..aPos] : aBuilder.Append(s, aCopied, aPos - aCopied).ToString();
                aPos++;
                return aText;
            }
        }

        throw new DrawException("missing close-brace", aLine);
    }

    /// <summary>The script between matching brackets, without them. The script is parsed to find its end.</summary>
    internal static string ReadBracketed(string s, ref int aPos, ref int aLine)
    {
        int aStart = aPos + 1;
        aPos = aStart;
        ParseScript(s, ref aPos, ref aLine, theNested: true);
        if (aPos >= s.Length || s[aPos] != ']')
        {
            throw new DrawException("missing close-bracket", aLine);
        }

        return s[aStart..aPos++];
    }

    /// <summary>A variable reference at <paramref name="aPos"/> (on its $), or null when the $ is literal.</summary>
    internal static WordPart.Variable? ReadVariable(string s, ref int aPos, ref int aLine)
    {
        int aStart = aPos + 1;
        if (aStart < s.Length && s[aStart] == '{')
        {
            int anEnd = s.IndexOf('}', aStart);
            if (anEnd < 0)
            {
                throw new DrawException("missing close-brace for variable name", aLine);
            }

            aPos = anEnd + 1;
            return new WordPart.Variable(s[(aStart + 1)..anEnd]);
        }

        int i = aStart;
        while (i < s.Length)
        {
            if (char.IsLetterOrDigit(s[i]) || s[i] == '_')
            {
                i++;
            }
            else if (s[i] == ':' && i + 1 < s.Length && s[i + 1] == ':')
            {
                i += 2;
                while (i < s.Length && s[i] == ':')
                {
                    i++;
                }
            }
            else
            {
                break;
            }
        }

        bool anArray = i < s.Length && s[i] == '(';
        if (i == aStart && !anArray)
        {
            return null;
        }

        string aName = s[aStart..i];
        if (!anArray)
        {
            aPos = i;
            return new WordPart.Variable(aName);
        }

        // An array element: the index runs to the first unescaped ")" and is substituted
        int aPosIndex = i + 1;
        var aParts = new List<WordPart>();
        var aText = new StringBuilder();
        while (true)
        {
            if (aPosIndex >= s.Length)
            {
                throw new DrawException("missing )", aLine);
            }

            char c = s[aPosIndex];
            if (c == ')')
            {
                aPosIndex++;
                break;
            }

            switch (c)
            {
                case '\\':
                    aText.Append(ReadEscape(s, ref aPosIndex, ref aLine));
                    break;
                case '$' when ReadVariable(s, ref aPosIndex, ref aLine) is { } aVariable:
                    Flush(aParts, aText);
                    aParts.Add(aVariable);
                    break;
                case '[':
                    Flush(aParts, aText);
                    aParts.Add(new WordPart.Command(ReadBracketed(s, ref aPosIndex, ref aLine)));
                    break;
                default:
                    if (c == '\n')
                    {
                        aLine++;
                    }

                    aText.Append(c);
                    aPosIndex++;
                    break;
            }
        }

        Flush(aParts, aText);
        aPos = aPosIndex;
        return new WordPart.Variable(aName, aParts);
    }

    /// <summary>A backslash sequence at <paramref name="aPos"/> (on its backslash), substituted.</summary>
    internal static string ReadEscape(string s, ref int aPos, ref int aLine)
    {
        aPos++;
        if (aPos >= s.Length)
        {
            return "\\";
        }

        char c = s[aPos++];
        switch (c)
        {
            case 'a':
                return "\a";
            case 'b':
                return "\b";
            case 'f':
                return "\f";
            case 'n':
                return "\n";
            case 'r':
                return "\r";
            case 't':
                return "\t";
            case 'v':
                return "\v";
            case 'x':
                return ReadHex(s, ref aPos, 2, "x");
            case 'u':
                return ReadHex(s, ref aPos, 4, "u");
            case 'U':
                return ReadHex(s, ref aPos, 8, "U");
            case >= '0' and <= '7':
            {
                int aValue = c - '0';
                for (int k = 0; k < 2 && aPos < s.Length && s[aPos] is >= '0' and <= '7'; k++)
                {
                    aValue = (aValue * 8) + (s[aPos++] - '0');
                }

                return ((char)(aValue & 0xFF)).ToString();
            }

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

    private static string ReadHex(string s, ref int aPos, int theMaxDigits, string theLetter)
    {
        int aStart = aPos;
        while (aPos < s.Length && aPos - aStart < theMaxDigits && char.IsAsciiHexDigit(s[aPos]))
        {
            aPos++;
        }

        if (aPos == aStart)
        {
            return theLetter;
        }

        int aValue = int.Parse(s.AsSpan(aStart, aPos - aStart), NumberStyles.HexNumber, CultureInfo.InvariantCulture);
        if (theLetter == "x")
        {
            aValue &= 0xFF;
        }

        return aValue is >= 0 and <= 0x10FFFF and not (>= 0xD800 and <= 0xDFFF) ? char.ConvertFromUtf32(aValue) : "�";
    }

    private static void SkipBlanks(string s, ref int aPos, ref int aLine)
    {
        while (aPos < s.Length)
        {
            if (s[aPos] is ' ' or '\t' or '\r' or '\f' or '\v')
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
        while (aPos < s.Length)
        {
            if (s[aPos] is ' ' or '\t' or '\r' or '\n' or ';' or '\f' or '\v')
            {
                if (s[aPos] == '\n')
                {
                    aLine++;
                }

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

    private static void SkipComment(string s, ref int aPos, ref int aLine)
    {
        while (aPos < s.Length && s[aPos] != '\n')
        {
            // A comment continues onto the next line after a backslash
            if (s[aPos] == '\\' && aPos + 1 < s.Length)
            {
                if (s[aPos + 1] == '\n')
                {
                    aLine++;
                }

                aPos++;
            }

            aPos++;
        }
    }
}
