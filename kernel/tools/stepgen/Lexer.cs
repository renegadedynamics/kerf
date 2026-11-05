// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// The C++ lexer of tools/stepgen: tokens with their comments, and the preprocessor reduced to what OCCT's STEP
// packages use (include guards, #include, and #ifdef OCCT_DEBUG blocks, which are dropped).

using System.Text;

namespace StepGen;

internal enum TokKind { Ident, Number, String, Char, Punct, Comment, Dropped }

internal sealed class Token
{
    public TokKind Kind;
    public string Text = "";
    public int Line;
    /// <summary>A newline separates this token from the previous one.</summary>
    public bool NewlineBefore;
    /// <summary>At least one blank line separates this token from the previous one.</summary>
    public bool BlankBefore;
    /// <summary>No space separates this token from the previous one (">" ">" is a shift).</summary>
    public bool Adjacent;

    public bool Is(string theText) => Text == theText && Kind != TokKind.Comment && Kind != TokKind.String && Kind != TokKind.Char;
    public bool IsIdent => Kind == TokKind.Ident;
    public bool IsComment => Kind == TokKind.Comment || Kind == TokKind.Dropped;
    public override string ToString() => Text;
}

internal static class Lexer
{
    private static readonly string[] Puncts3 = ["...", "<<=", "->*"];
    private static readonly string[] Puncts2 =
        ["::", "->", "<<", "<=", ">=", "==", "!=", "&&", "||", "++", "--", "+=", "-=", "*=", "/=", "%=", "&=", "|=", "^=", "[[", "]]"];

    [ThreadStatic] private static int myLastEnd;

    public static List<Token> Lex(string theText)
    {
        theText = theText.Replace("\r\n", "\n");
        var aTokens = new List<Token>();
        int i = 0, aLine = 1;
        myLastEnd = -1;
        int aNewlines = 1;
        // the preprocessor's stack of conditions: true while the lines are kept
        var aConds = new Stack<(bool Active, bool ParentActive, bool Debug)>();
        bool Active() => aConds.Count == 0 || aConds.Peek().Active;
        bool aDroppedNoted = false;

        while (i < theText.Length)
        {
            char c = theText[i];
            if (c == '\n')
            {
                aNewlines++;
                aLine++;
                i++;
                continue;
            }

            if (c == ' ' || c == '\t' || c == '\f' || c == '\v')
            {
                i++;
                continue;
            }

            // preprocessor lines (at the start of a line)
            if (c == '#' && aNewlines > 0)
            {
                int aStart = i;
                var aSb = new StringBuilder();
                while (i < theText.Length && theText[i] != '\n')
                {
                    if (theText[i] == '\\' && i + 1 < theText.Length && theText[i + 1] == '\n')
                    {
                        i += 2;
                        aLine++;
                        continue;
                    }

                    aSb.Append(theText[i]);
                    i++;
                }

                string aDir = aSb.ToString().Trim();
                int aCommentAt = aDir.IndexOf("//", StringComparison.Ordinal);
                if (aCommentAt >= 0)
                {
                    aDir = aDir[..aCommentAt].Trim();
                }

                string[] aParts = aDir[1..].Trim().Split([' ', '\t'], 2, StringSplitOptions.RemoveEmptyEntries);
                string aWord = aParts.Length > 0 ? aParts[0] : "";
                string anArg = aParts.Length > 1 ? aParts[1].Trim() : "";
                switch (aWord)
                {
                    case "ifdef":
                    case "ifndef":
                    case "if":
                    {
                        bool aParent = Active();
                        bool isDebug = anArg.Contains("OCCT_DEBUG", StringComparison.Ordinal);
                        bool aValue;
                        if (aWord == "ifndef")
                        {
                            aValue = true; // include guards, and "not OCCT_DEBUG"
                        }
                        else if (aWord == "ifdef")
                        {
                            aValue = false; // OCCT_DEBUG and the odd macros (Status, SetForm) are not defined
                        }
                        else
                        {
                            aValue = anArg.Trim() == "1";
                        }

                        aConds.Push((aParent && aValue, aParent, isDebug));
                        if (aParent && !aValue && !aDroppedNoted && isDebug)
                        {
                            aTokens.Add(new Token { Kind = TokKind.Dropped, Text = "// OCCT_DEBUG block left out", Line = aLine, NewlineBefore = true });
                            aDroppedNoted = true;
                        }

                        break;
                    }
                    case "else":
                    case "elif":
                        if (aConds.Count > 0)
                        {
                            var aTop = aConds.Pop();
                            aConds.Push((aTop.ParentActive && !aTop.Active && aWord == "else", aTop.ParentActive, aTop.Debug));
                        }

                        break;
                    case "endif":
                        if (aConds.Count > 0)
                        {
                            aConds.Pop();
                        }

                        aDroppedNoted = false;
                        break;
                }

                _ = aStart;
                continue;
            }

            if (!Active())
            {
                // skip the rest of the line
                while (i < theText.Length && theText[i] != '\n')
                {
                    i++;
                }

                continue;
            }

            var aTok = new Token { Line = aLine, NewlineBefore = aNewlines > 0 || aTokens.Count == 0, BlankBefore = aNewlines > 1, Adjacent = i == myLastEnd };
            aNewlines = 0;

            if (c == '/' && i + 1 < theText.Length && theText[i + 1] == '/')
            {
                int j = theText.IndexOf('\n', i);
                if (j < 0)
                {
                    j = theText.Length;
                }

                aTok.Kind = TokKind.Comment;
                aTok.Text = theText[i..j].TrimEnd();
                i = j;
                aTokens.Add(aTok);
                continue;
            }

            if (c == '/' && i + 1 < theText.Length && theText[i + 1] == '*')
            {
                int j = theText.IndexOf("*/", i + 2, StringComparison.Ordinal);
                j = j < 0 ? theText.Length : j + 2;
                aTok.Kind = TokKind.Comment;
                aTok.Text = theText[i..j];
                aLine += aTok.Text.Count(ch => ch == '\n');
                i = j;
                aTokens.Add(aTok);
                continue;
            }

            if (char.IsLetter(c) || c == '_')
            {
                int j = i;
                while (j < theText.Length && (char.IsLetterOrDigit(theText[j]) || theText[j] == '_'))
                {
                    j++;
                }

                aTok.Kind = TokKind.Ident;
                aTok.Text = theText[i..j];
                i = j;
                aTokens.Add(aTok);
                continue;
            }

            if (char.IsDigit(c) || (c == '.' && i + 1 < theText.Length && char.IsDigit(theText[i + 1])))
            {
                int j = i;
                while (j < theText.Length)
                {
                    char d = theText[j];
                    if (char.IsLetterOrDigit(d) || d == '.' || d == '_')
                    {
                        j++;
                    }
                    else if ((d == '+' || d == '-') && (theText[j - 1] == 'e' || theText[j - 1] == 'E') && !theText[i..j].StartsWith("0x", StringComparison.OrdinalIgnoreCase))
                    {
                        j++;
                    }
                    else
                    {
                        break;
                    }
                }

                aTok.Kind = TokKind.Number;
                aTok.Text = theText[i..j];
                i = j;
                aTokens.Add(aTok);
                continue;
            }

            if (c == '"' || c == '\'')
            {
                int j = i + 1;
                while (j < theText.Length && theText[j] != c)
                {
                    if (theText[j] == '\\')
                    {
                        j++;
                    }

                    j++;
                }

                j++;
                aTok.Kind = c == '"' ? TokKind.String : TokKind.Char;
                aTok.Text = theText[i..j];
                i = j;
                aTokens.Add(aTok);
                continue;
            }

            aTok.Kind = TokKind.Punct;
            string? aP = null;
            foreach (string p in Puncts3)
            {
                if (string.CompareOrdinal(theText, i, p, 0, 3) == 0)
                {
                    aP = p;
                    break;
                }
            }

            if (aP == null)
            {
                foreach (string p in Puncts2)
                {
                    if (string.CompareOrdinal(theText, i, p, 0, 2) == 0)
                    {
                        aP = p;
                        break;
                    }
                }
            }

            aP ??= c.ToString();
            aTok.Text = aP;
            i += aP.Length;
            myLastEnd = i;
            aTokens.Add(aTok);
        }

        // adjacent string literals ("a" "b") are one literal in C++
        var aResult = new List<Token>(aTokens.Count);
        foreach (var t in aTokens)
        {
            if (t.Kind == TokKind.String && aResult.Count > 0 && aResult[^1].Kind == TokKind.String)
            {
                aResult[^1].Text = aResult[^1].Text[..^1] + t.Text[1..];
                continue;
            }

            aResult.Add(t);
        }

        return aResult;
    }
}
