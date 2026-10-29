// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using System.Text.RegularExpressions;

namespace OcctNet.DrawLite;

/// <summary>Tcl's string commands: string, regexp, regsub, subst, split, format and scan.</summary>
internal static class TclStringCommands
{
    private const string Whitespace = " \t\n\r\v\f";

    public static void Register(DrawInterpreter theInterpreter)
    {
        theInterpreter.Register("string", String);
        theInterpreter.Register("regexp", Regexp);
        theInterpreter.Register("regsub", Regsub);
        theInterpreter.Register("subst", Subst);
        theInterpreter.Register("split", Split);
        theInterpreter.Register("format", static (_, a) => a.Count >= 2 ? TclFormat.Format(a[1], a.Skip(2).ToList()) : throw TclControlCommands.Usage("format formatString ?arg ...?"));
        theInterpreter.Register("scan", Scan);
    }

    private static DrawException Usage(string theUsage) => TclControlCommands.Usage(theUsage);

    private static int Index(string theIndex, int theLength) => (int)Math.Clamp(TclList.ParseIndex(theIndex, theLength), -1, (long)theLength + 1);

    private static string String(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            throw Usage("string subcommand ?arg ...?");
        }

        switch (TclControlCommands.Resolve(a[1], "subcommand", ["length", "bytelength", "equal", "compare", "first", "last", "index", "range", "tolower", "toupper", "totitle", "trim", "trimleft", "trimright", "match", "map", "is", "repeat", "replace", "cat", "reverse", "wordstart", "wordend"]))
        {
            case "length" when a.Count == 3:
                return a[2].Length.ToString(CultureInfo.InvariantCulture);
            case "bytelength" when a.Count == 3:
                return Encoding.UTF8.GetByteCount(a[2]).ToString(CultureInfo.InvariantCulture);
            case "equal" or "compare":
            {
                bool aNoCase = false;
                long aLength = -1;
                int i = 2;
                for (; i < a.Count - 2; i++)
                {
                    if (a[i] == "-nocase")
                    {
                        aNoCase = true;
                    }
                    else if (a[i] == "-length" && i + 1 < a.Count - 2)
                    {
                        aLength = TclNumber.GetInt(a[++i]);
                    }
                    else
                    {
                        throw new DrawException($"bad option \"{a[i]}\": must be -nocase or -length");
                    }
                }

                if (a.Count - i != 2)
                {
                    throw Usage($"string {a[1]} ?-nocase? ?-length int? string1 string2");
                }

                string x = a[i];
                string y = a[i + 1];
                if (aLength >= 0)
                {
                    x = x[..(int)Math.Min(aLength, x.Length)];
                    y = y[..(int)Math.Min(aLength, y.Length)];
                }

                int aCompare = aNoCase ? string.Compare(x.ToLowerInvariant(), y.ToLowerInvariant(), StringComparison.Ordinal) : string.CompareOrdinal(x, y);
                return a[1] == "equal" ? (aCompare == 0 ? "1" : "0") : Math.Sign(aCompare).ToString(CultureInfo.InvariantCulture);
            }

            case "first" when a.Count is 4 or 5:
            {
                int aStart = a.Count == 5 ? Math.Max(0, Index(a[4], a[3].Length)) : 0;
                if (a[2].Length == 0 || aStart > a[3].Length)
                {
                    return "-1";
                }

                return a[3].IndexOf(a[2], aStart, StringComparison.Ordinal).ToString(CultureInfo.InvariantCulture);
            }

            case "last" when a.Count is 4 or 5:
            {
                string aHaystack = a[3];
                int aLast = a.Count == 5 ? Index(a[4], aHaystack.Length) : aHaystack.Length - 1;
                if (a[2].Length == 0 || aLast < 0)
                {
                    return "-1";
                }

                string aRegion = aHaystack[..Math.Min(aHaystack.Length, aLast + a[2].Length)];
                int aFound = aRegion.LastIndexOf(a[2], StringComparison.Ordinal);
                return (aFound > aLast ? -1 : aFound).ToString(CultureInfo.InvariantCulture);
            }

            case "index" when a.Count == 4:
            {
                int k = Index(a[3], a[2].Length);
                return k >= 0 && k < a[2].Length ? a[2][k].ToString() : "";
            }

            case "range" when a.Count == 5:
            {
                int aFirst = Math.Max(0, Index(a[3], a[2].Length));
                int aLast = Math.Min(a[2].Length - 1, Index(a[4], a[2].Length));
                return aFirst > aLast ? "" : a[2][aFirst..(aLast + 1)];
            }

            case "tolower" or "toupper" or "totitle" when a.Count is >= 3 and <= 5:
            {
                string s = a[2];
                int aFirst = a.Count > 3 ? Math.Max(0, Index(a[3], s.Length)) : 0;
                int aLast = a.Count > 4 ? Math.Min(s.Length - 1, Index(a[4], s.Length)) : a.Count > 3 ? aFirst : s.Length - 1;
                if (aFirst > aLast || aFirst >= s.Length)
                {
                    return s;
                }

                string aPart = s[aFirst..(aLast + 1)];
                aPart = a[1] switch
                {
                    "tolower" => aPart.ToLowerInvariant(),
                    "toupper" => aPart.ToUpperInvariant(),
                    _ => aPart.Length == 0 ? aPart : char.ToUpperInvariant(aPart[0]) + aPart[1..].ToLowerInvariant(),
                };
                return s[..aFirst] + aPart + s[(aLast + 1)..];
            }

            case "trim" or "trimleft" or "trimright" when a.Count is 3 or 4:
            {
                char[] aChars = (a.Count == 4 ? a[3] : Whitespace + "\0").ToCharArray();
                return a[1] switch
                {
                    "trim" => a[2].Trim(aChars),
                    "trimleft" => a[2].TrimStart(aChars),
                    _ => a[2].TrimEnd(aChars),
                };
            }

            case "match":
            {
                bool aNoCase = a.Count == 5 && a[2] == "-nocase";
                if (a.Count != (aNoCase ? 5 : 4))
                {
                    throw Usage("string match ?-nocase? pattern string");
                }

                return TclGlob.Match(a[^2], a[^1], aNoCase) ? "1" : "0";
            }

            case "map":
            {
                bool aNoCase = a.Count == 5 && a[2] == "-nocase";
                if (a.Count != (aNoCase ? 5 : 4))
                {
                    throw Usage("string map ?-nocase? charMap string");
                }

                return Map(TclList.Parse(a[^2]), a[^1], aNoCase);
            }

            case "is" when a.Count >= 4:
                return Is(theInterpreter, a) ? "1" : "0";
            case "repeat" when a.Count == 4:
            {
                long aCount = TclNumber.GetInt(a[3]);
                var aBuilder = new StringBuilder();
                for (long i = 0; i < aCount; i++)
                {
                    aBuilder.Append(a[2]);
                }

                return aBuilder.ToString();
            }

            case "replace" when a.Count is 5 or 6:
            {
                string s = a[2];
                long aFirstIndex = TclList.ParseIndex(a[3], s.Length);
                long aLastIndex = TclList.ParseIndex(a[4], s.Length);
                if (aLastIndex < 0 || aFirstIndex >= s.Length || aFirstIndex > aLastIndex)
                {
                    return s;
                }

                int aFirst = (int)Math.Max(0, aFirstIndex);
                int aLast = (int)Math.Min(s.Length - 1, aLastIndex);
                return s[..aFirst] + (a.Count == 6 ? a[5] : "") + s[(aLast + 1)..];
            }

            case "cat":
                return string.Concat(a.Skip(2));
            case "reverse" when a.Count == 3:
            {
                char[] aChars = a[2].ToCharArray();
                Array.Reverse(aChars);
                return new string(aChars);
            }

            case "wordstart" when a.Count == 4:
            {
                int k = Math.Clamp(Index(a[3], a[2].Length), 0, Math.Max(0, a[2].Length - 1));
                while (k > 0 && IsWordChar(a[2][k]) && IsWordChar(a[2][k - 1]))
                {
                    k--;
                }

                return k.ToString(CultureInfo.InvariantCulture);
            }

            case "wordend" when a.Count == 4:
            {
                int k = Math.Clamp(Index(a[3], a[2].Length), 0, a[2].Length);
                if (k < a[2].Length && !IsWordChar(a[2][k]))
                {
                    return (k + 1).ToString(CultureInfo.InvariantCulture);
                }

                while (k < a[2].Length && IsWordChar(a[2][k]))
                {
                    k++;
                }

                return k.ToString(CultureInfo.InvariantCulture);
            }

            default:
                throw new DrawException($"string {a[1]}: unknown subcommand or wrong # args");
        }
    }

    private static bool IsWordChar(char c) => char.IsLetterOrDigit(c) || c == '_' || char.GetUnicodeCategory(c) == UnicodeCategory.ConnectorPunctuation;

    private static string Map(IReadOnlyList<string> theMap, string theText, bool theNoCase)
    {
        if (theMap.Count % 2 != 0)
        {
            throw new DrawException("char map list unbalanced");
        }

        StringComparison aComparison = theNoCase ? StringComparison.OrdinalIgnoreCase : StringComparison.Ordinal;
        var aBuilder = new StringBuilder();
        int i = 0;
        while (i < theText.Length)
        {
            bool aReplaced = false;
            for (int k = 0; k < theMap.Count; k += 2)
            {
                string aKey = theMap[k];
                if (aKey.Length > 0 && string.Compare(theText, i, aKey, 0, aKey.Length, aComparison) == 0 && i + aKey.Length <= theText.Length)
                {
                    aBuilder.Append(theMap[k + 1]);
                    i += aKey.Length;
                    aReplaced = true;
                    break;
                }
            }

            if (!aReplaced)
            {
                aBuilder.Append(theText[i++]);
            }
        }

        return aBuilder.ToString();
    }

    private static bool Is(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        string aClass = TclControlCommands.Resolve(a[2], "class", ["alnum", "alpha", "ascii", "boolean", "control", "digit", "double", "entier", "false", "graph", "integer", "list", "lower", "print", "punct", "space", "true", "upper", "wideinteger", "wordchar", "xdigit"]);
        bool aStrict = false;
        string? aFailVar = null;
        int i = 3;
        for (; i < a.Count - 1; i++)
        {
            if (a[i] == "-strict")
            {
                aStrict = true;
            }
            else if (a[i] == "-failindex" && i + 1 < a.Count - 1)
            {
                aFailVar = a[++i];
            }
            else
            {
                throw new DrawException($"bad option \"{a[i]}\": must be -strict or -failindex");
            }
        }

        string s = a[^1];
        if (s.Length == 0)
        {
            return !aStrict;
        }

        Func<char, bool>? aCharClass = aClass switch
        {
            "alnum" => char.IsLetterOrDigit,
            "alpha" => char.IsLetter,
            "ascii" => static c => c < 128,
            "control" => char.IsControl,
            "digit" => char.IsDigit,
            "graph" => static c => !char.IsControl(c) && !char.IsWhiteSpace(c),
            "lower" => char.IsLower,
            "print" => static c => !char.IsControl(c),
            "punct" => static c => char.IsPunctuation(c) || char.IsSymbol(c),
            "space" => char.IsWhiteSpace,
            "upper" => char.IsUpper,
            "wordchar" => IsWordChar,
            "xdigit" => char.IsAsciiHexDigit,
            _ => null,
        };
        if (aCharClass is not null)
        {
            for (int k = 0; k < s.Length; k++)
            {
                if (!aCharClass(s[k]))
                {
                    if (aFailVar is not null)
                    {
                        theInterpreter.SetVariable(aFailVar, k.ToString(CultureInfo.InvariantCulture));
                    }

                    return false;
                }
            }

            return true;
        }

        bool aResult = aClass switch
        {
            "integer" => TclNumber.TryParseInt(s, out long anInt) && anInt is >= int.MinValue and <= int.MaxValue,
            "wideinteger" or "entier" => TclNumber.TryParseInt(s, out _),
            "double" => TclNumber.TryParseDouble(s, out _),
            "boolean" => IsBoolean(s, out _),
            "true" => IsBoolean(s, out bool aTrue) && aTrue,
            "false" => IsBoolean(s, out bool aFalse) && !aFalse,
            "list" => TclList.IsList(s),
            _ => throw new DrawException($"bad class \"{aClass}\": must be alnum, alpha, ascii, control, boolean, digit, double, entier, false, graph, integer, list, lower, print, punct, space, true, upper, wideinteger, wordchar, or xdigit"),
        };
        if (!aResult && aFailVar is not null)
        {
            theInterpreter.SetVariable(aFailVar, "0");
        }

        return aResult;
    }

    /// <summary>string is boolean: the words, or the integers 0 and 1 (not other numbers).</summary>
    private static bool IsBoolean(string s, out bool theValue)
    {
        if (TclNumber.TryParseInt(s, out long anInt))
        {
            theValue = anInt != 0;
            return anInt is 0 or 1;
        }

        theValue = false;
        return !TclNumber.IsNumber(s) && TclNumber.TryParseBoolean(s, out theValue);
    }

    // -------------------------------------------------------------------------------------------------------------
    // regexp, regsub

    private sealed class RegexOptionsParsed
    {
        public bool NoCase;
        public bool All;
        public bool Inline;
        public bool Indices;
        public bool Line;
        public bool Expanded;
        public int Start;
    }

    private static int ParseRegexOptions(IReadOnlyList<string> a, RegexOptionsParsed theOptions, bool theIsSub)
    {
        int i = 1;
        while (i < a.Count && a[i].StartsWith('-'))
        {
            string anOption = a[i++];
            switch (anOption)
            {
                case "--":
                    return i;
                case "-nocase":
                    theOptions.NoCase = true;
                    break;
                case "-all":
                    theOptions.All = true;
                    break;
                case "-inline" when !theIsSub:
                    theOptions.Inline = true;
                    break;
                case "-indices" when !theIsSub:
                    theOptions.Indices = true;
                    break;
                case "-line" or "-lineanchor" or "-linestop":
                    theOptions.Line = true;
                    break;
                case "-expanded":
                    theOptions.Expanded = true;
                    break;
                case "-about":
                    break;
                case "-start" when i < a.Count:
                    theOptions.Start = (int)Math.Max(0, TclList.ParseIndex(a[i++], int.MaxValue));
                    break;
                default:
                    i--;
                    return i;
            }
        }

        return i;
    }

    private static string Regexp(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        var anOptions = new RegexOptionsParsed();
        int i = ParseRegexOptions(a, anOptions, theIsSub: false);
        if (a.Count - i < 2)
        {
            throw Usage("regexp ?-option ...? exp string ?matchVar? ?subMatchVar ...?");
        }

        Regex aRegex = TclRegex.Create(a[i], anOptions.NoCase, anOptions.Line, anOptions.Expanded);
        string aText = a[i + 1];
        var aVars = a.Skip(i + 2).ToList();
        if (anOptions.Inline && aVars.Count > 0)
        {
            throw new DrawException("regexp match variables not allowed when using -inline");
        }

        int aStart = Math.Min(anOptions.Start, aText.Length);
        var anInline = new List<string>();
        int aCount = 0;
        Match? aLast = null;
        Match aMatch = aRegex.Match(aText, aStart);
        while (aMatch.Success)
        {
            aCount++;
            aLast = aMatch;
            if (anOptions.Inline)
            {
                for (int g = 0; g < aMatch.Groups.Count; g++)
                {
                    anInline.Add(GroupValue(aMatch.Groups[g], anOptions.Indices));
                }
            }

            if (!anOptions.All)
            {
                break;
            }

            aMatch = aMatch.Length == 0
                         ? (aMatch.Index + 1 <= aText.Length ? aRegex.Match(aText, aMatch.Index + 1) : Match.Empty)
                         : aMatch.NextMatch();
        }

        if (anOptions.Inline)
        {
            return TclList.Format(anInline);
        }

        if (aLast is not null)
        {
            for (int v = 0; v < aVars.Count; v++)
            {
                string aValue = v < aLast.Groups.Count ? GroupValue(aLast.Groups[v], anOptions.Indices) : anOptions.Indices ? "-1 -1" : "";
                theInterpreter.SetVariable(aVars[v], aValue);
            }
        }

        return aCount.ToString(CultureInfo.InvariantCulture);
    }

    private static string GroupValue(Group theGroup, bool theIndices)
    {
        if (!theIndices)
        {
            return theGroup.Success ? theGroup.Value : "";
        }

        return theGroup.Success
                   ? $"{theGroup.Index} {theGroup.Index + theGroup.Length - 1}"
                   : "-1 -1";
    }

    private static string Regsub(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        var anOptions = new RegexOptionsParsed();
        int i = ParseRegexOptions(a, anOptions, theIsSub: true);
        if (a.Count - i is < 3 or > 4)
        {
            throw Usage("regsub ?-option ...? exp string subSpec ?varName?");
        }

        Regex aRegex = TclRegex.Create(a[i], anOptions.NoCase, anOptions.Line, anOptions.Expanded);
        string aText = a[i + 1];
        string aSpec = a[i + 2];
        var aBuilder = new StringBuilder();
        int aCount = 0;
        int aCopied = 0;
        int aStart = Math.Min(anOptions.Start, aText.Length);
        Match aMatch = aRegex.Match(aText, aStart);
        while (aMatch.Success)
        {
            aCount++;
            aBuilder.Append(aText, aCopied, aMatch.Index - aCopied);
            AppendSubstitution(aBuilder, aSpec, aMatch);
            aCopied = aMatch.Index + aMatch.Length;
            if (!anOptions.All)
            {
                break;
            }

            if (aMatch.Length == 0)
            {
                if (aMatch.Index < aText.Length)
                {
                    aBuilder.Append(aText[aMatch.Index]);
                }

                aCopied = aMatch.Index + 1;
                aMatch = aMatch.Index + 1 <= aText.Length ? aRegex.Match(aText, aMatch.Index + 1) : Match.Empty;
            }
            else
            {
                aMatch = aMatch.NextMatch();
            }
        }

        if (aCopied < aText.Length)
        {
            aBuilder.Append(aText, aCopied, aText.Length - aCopied);
        }

        string aResult = aBuilder.ToString();
        if (a.Count - i == 4)
        {
            theInterpreter.SetVariable(a[i + 3], aResult);
            return aCount.ToString(CultureInfo.InvariantCulture);
        }

        return aResult;
    }

    private static void AppendSubstitution(StringBuilder theBuilder, string theSpec, Match theMatch)
    {
        for (int k = 0; k < theSpec.Length; k++)
        {
            char c = theSpec[k];
            if (c == '&')
            {
                theBuilder.Append(theMatch.Value);
            }
            else if (c == '\\' && k + 1 < theSpec.Length)
            {
                char d = theSpec[k + 1];
                if (char.IsAsciiDigit(d))
                {
                    int aGroup = d - '0';
                    if (aGroup < theMatch.Groups.Count && theMatch.Groups[aGroup].Success)
                    {
                        theBuilder.Append(theMatch.Groups[aGroup].Value);
                    }

                    k++;
                }
                else if (d is '&' or '\\')
                {
                    theBuilder.Append(d);
                    k++;
                }
                else
                {
                    theBuilder.Append(c);
                }
            }
            else
            {
                theBuilder.Append(c);
            }
        }
    }

    // -------------------------------------------------------------------------------------------------------------
    // subst, split, scan

    private static string Subst(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        bool aBackslashes = true;
        bool aVariables = true;
        bool aCommands = true;
        for (int i = 1; i < a.Count - 1; i++)
        {
            switch (a[i])
            {
                case "-nobackslashes":
                    aBackslashes = false;
                    break;
                case "-novariables":
                    aVariables = false;
                    break;
                case "-nocommands":
                    aCommands = false;
                    break;
                default:
                    throw new DrawException($"bad option \"{a[i]}\": must be -nobackslashes, -nocommands, or -novariables");
            }
        }

        if (a.Count < 2)
        {
            throw Usage("subst ?-nobackslashes? ?-nocommands? ?-novariables? string");
        }

        return theInterpreter.SubstituteParts(TclParser.ParseSubst(a[^1], aBackslashes, aVariables, aCommands));
    }

    private static string Split(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count is < 2 or > 3)
        {
            throw Usage("split string ?splitChars?");
        }

        string s = a[1];
        string aChars = a.Count == 3 ? a[2] : " \t\n\r";
        var aParts = new List<string>();
        if (aChars.Length == 0)
        {
            foreach (char c in s)
            {
                aParts.Add(c.ToString());
            }
        }
        else if (s.Length > 0)
        {
            int aStart = 0;
            for (int i = 0; i < s.Length; i++)
            {
                if (aChars.Contains(s[i]))
                {
                    aParts.Add(s[aStart..i]);
                    aStart = i + 1;
                }
            }

            aParts.Add(s[aStart..]);
        }

        return TclList.Make([.. aParts]);
    }

    private static string Scan(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            throw Usage("scan string format ?varName ...?");
        }

        (List<string?> aValues, int aConverted, bool anInputEnded) = TclFormat.Scan(a[1], a[2]);
        if (a.Count == 3)
        {
            return TclList.Format(aValues.Select(v => v ?? ""));
        }

        if (a.Count - 3 != aValues.Count)
        {
            throw new DrawException("different numbers of variable names and field specifiers");
        }

        for (int i = 0; i < aValues.Count; i++)
        {
            if (aValues[i] is { } aValue)
            {
                theInterpreter.SetVariable(a[i + 3], aValue);
            }
        }

        return (aConverted == 0 && anInputEnded ? -1 : aConverted).ToString(CultureInfo.InvariantCulture);
    }
}
