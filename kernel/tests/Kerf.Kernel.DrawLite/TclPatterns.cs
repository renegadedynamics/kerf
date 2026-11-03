// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using System.Text;
using System.Text.RegularExpressions;

namespace Kerf.Kernel.DrawLite;

/// <summary>Tcl's glob patterns (string match): *, ?, [chars] with ranges, and \x for a literal x.</summary>
public static class TclGlob
{
    public static bool Match(string thePattern, string theText, bool theNoCase = false)
    {
        if (theNoCase)
        {
            thePattern = thePattern.ToLowerInvariant();
            theText = theText.ToLowerInvariant();
        }

        return Match(thePattern, 0, theText, 0);
    }

    private static bool Match(string p, int pi, string s, int si)
    {
        while (pi < p.Length)
        {
            char c = p[pi];
            switch (c)
            {
                case '*':
                {
                    while (pi < p.Length && p[pi] == '*')
                    {
                        pi++;
                    }

                    if (pi == p.Length)
                    {
                        return true;
                    }

                    for (int k = si; k <= s.Length; k++)
                    {
                        if (Match(p, pi, s, k))
                        {
                            return true;
                        }
                    }

                    return false;
                }

                case '?':
                    if (si >= s.Length)
                    {
                        return false;
                    }

                    pi++;
                    si++;
                    break;
                case '[':
                {
                    if (si >= s.Length)
                    {
                        return false;
                    }

                    char aChar = s[si];
                    pi++;
                    bool aMatched = false;
                    while (pi < p.Length && p[pi] != ']')
                    {
                        char aLow = p[pi];
                        if (aLow == '\\' && pi + 1 < p.Length)
                        {
                            aLow = p[++pi];
                        }

                        char aHigh = aLow;
                        if (pi + 2 < p.Length && p[pi + 1] == '-' && p[pi + 2] != ']')
                        {
                            aHigh = p[pi + 2];
                            pi += 2;
                        }

                        if ((aChar >= aLow && aChar <= aHigh) || (aChar >= aHigh && aChar <= aLow))
                        {
                            aMatched = true;
                        }

                        pi++;
                    }

                    if (!aMatched)
                    {
                        return false;
                    }

                    pi++;
                    si++;
                    break;
                }

                case '\\' when pi + 1 < p.Length:
                    if (si >= s.Length || s[si] != p[pi + 1])
                    {
                        return false;
                    }

                    pi += 2;
                    si++;
                    break;
                default:
                    if (si >= s.Length || s[si] != c)
                    {
                        return false;
                    }

                    pi++;
                    si++;
                    break;
            }
        }

        return si == s.Length;
    }
}

/// <summary>
/// Tcl's regular expressions (ARE) on .NET's engine. The syntax is close; the differences handled here are
/// POSIX classes ([[:alpha:]]), Tcl's word anchors (\m \M \y \Y), $ matching only at the very end, and "."
/// matching newlines unless -line is given. Tcl prefers the longest of several alternatives; .NET takes the
/// first that matches, which differs only for patterns like a|ab.
/// </summary>
public static class TclRegex
{
    private static readonly ConcurrentDictionary<(string, bool, bool, bool), Regex> s_cache = new();

    public static Regex Create(string thePattern, bool theNoCase = false, bool theLine = false, bool theExpanded = false)
        => s_cache.GetOrAdd((thePattern, theNoCase, theLine, theExpanded), static k =>
        {
            (string aPattern, bool aNoCase, bool aLine, bool anExpanded) = k;
            RegexOptions anOptions = RegexOptions.CultureInvariant
                                   | (aNoCase ? RegexOptions.IgnoreCase : 0)
                                   | (aLine ? RegexOptions.Multiline : RegexOptions.Singleline)
                                   | (anExpanded ? RegexOptions.IgnorePatternWhitespace : 0);
            try
            {
                return new Regex(Translate(aPattern, aLine), anOptions);
            }
            catch (ArgumentException anError)
            {
                throw new DrawException($"couldn't compile regular expression pattern: {anError.Message}");
            }
        });

    /// <summary>A Tcl ARE pattern rewritten for .NET.</summary>
    public static string Translate(string thePattern, bool theLine = false)
    {
        if (thePattern.StartsWith("***=", StringComparison.Ordinal))
        {
            return Regex.Escape(thePattern[4..]);
        }

        var aBuilder = new StringBuilder(thePattern.Length + 8);
        int i = 0;
        while (i < thePattern.Length)
        {
            char c = thePattern[i];
            if (c == '\\' && i + 1 < thePattern.Length)
            {
                char d = thePattern[i + 1];
                aBuilder.Append(d switch
                {
                    'm' => @"\b(?=\w)",
                    'M' => @"\b(?<=\w)",
                    'y' => @"\b",
                    'Y' => @"\B",
                    'Z' => @"\z",
                    _ => $"\\{d}",
                });
                i += 2;
            }
            else if (c == '[')
            {
                i = TranslateBracket(thePattern, i, aBuilder);
            }
            else if (c == '$' && !theLine)
            {
                aBuilder.Append(@"\z");
                i++;
            }
            else
            {
                aBuilder.Append(c);
                i++;
            }
        }

        return aBuilder.ToString();
    }

    private static int TranslateBracket(string p, int i, StringBuilder theBuilder)
    {
        theBuilder.Append('[');
        i++;
        if (i < p.Length && p[i] == '^')
        {
            theBuilder.Append('^');
            i++;
        }

        if (i < p.Length && p[i] == ']')
        {
            theBuilder.Append(@"\]");
            i++;
        }

        while (i < p.Length && p[i] != ']')
        {
            if (p[i] == '[' && i + 1 < p.Length && p[i + 1] == ':')
            {
                int anEnd = p.IndexOf(":]", i + 2, StringComparison.Ordinal);
                if (anEnd > 0)
                {
                    theBuilder.Append(p[(i + 2)..anEnd] switch
                    {
                        "alpha" => @"\p{L}",
                        "digit" => "0-9",
                        "alnum" => @"\p{L}\p{Nd}",
                        "upper" => @"\p{Lu}",
                        "lower" => @"\p{Ll}",
                        "space" => @"\s",
                        "blank" => @" \t",
                        "punct" => @"\p{P}\p{S}",
                        "xdigit" => "0-9A-Fa-f",
                        "word" => @"\w",
                        "cntrl" => @"\p{Cc}",
                        "print" or "graph" => @"\P{Cc}",
                        _ => throw new DrawException($"couldn't compile regular expression pattern: invalid character class"),
                    });
                    i = anEnd + 2;
                    continue;
                }
            }

            if (p[i] == '\\' && i + 1 < p.Length)
            {
                theBuilder.Append(p, i, 2);
                i += 2;
                continue;
            }

            // A '[' or "-[" inside a .NET class would start a subtraction; Tcl means the character
            theBuilder.Append(p[i] == '[' ? @"\[" : p[i].ToString());
            i++;
        }

        theBuilder.Append(']');
        return i + 1;
    }
}
