// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Runtime.ExceptionServices;
using System.Text;

namespace Kerf.Kernel.DrawLite;

/// <summary>Tcl's list commands.</summary>
internal static class TclListCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        theInterpreter.Register("list", static (_, a) => TclList.Make(a.Skip(1).ToArray()));
        theInterpreter.Register("llength", static (_, a) => a.Count == 2 ? TclList.Parse(a[1]).Count.ToString(CultureInfo.InvariantCulture) : throw Usage("llength list"));
        theInterpreter.Register("lindex", Lindex);
        theInterpreter.Register("lrange", Lrange);
        theInterpreter.Register("lappend", Lappend);
        theInterpreter.Register("linsert", Linsert);
        theInterpreter.Register("lreplace", Lreplace);
        theInterpreter.Register("lassign", Lassign);
        theInterpreter.Register("lreverse", static (_, a) => a.Count == 2 ? TclList.Make(TclList.Parse(a[1]).Reverse().ToArray()) : throw Usage("lreverse list"));
        theInterpreter.Register("lrepeat", Lrepeat);
        theInterpreter.Register("lset", Lset);
        theInterpreter.Register("lsearch", Lsearch);
        theInterpreter.Register("lsort", Lsort);
        theInterpreter.Register("concat", static (_, a) => a.Count == 1 ? "" : ConcatAll(a));
        theInterpreter.Register("join", static (_, a) => a.Count is 2 or 3 ? string.Join(a.Count == 3 ? a[2] : " ", TclList.Parse(a[1])) : throw Usage("join list ?joinString?"));
    }

    private static DrawException Usage(string theUsage) => TclControlCommands.Usage(theUsage);

    private static string ConcatAll(IReadOnlyList<string> theArgs)
    {
        var aBuilder = new StringBuilder();
        for (int i = 1; i < theArgs.Count; i++)
        {
            string aTrimmed = theArgs[i].Trim(' ', '\t', '\n', '\r', '\v', '\f');
            if (aTrimmed.Length == 0)
            {
                continue;
            }

            if (aBuilder.Length > 0)
            {
                aBuilder.Append(' ');
            }

            aBuilder.Append(aTrimmed);
        }

        return aBuilder.ToString();
    }

    /// <summary>The index arguments of lindex/lset: several words, or one word that is a list of indices.</summary>
    private static IReadOnlyList<string> Indices(IReadOnlyList<string> theArgs, int theFrom, int theTo)
    {
        if (theTo - theFrom == 1)
        {
            IReadOnlyList<string> aList = TclList.Parse(theArgs[theFrom]);
            return aList.Count == 1 ? [theArgs[theFrom]] : aList;
        }

        return theArgs.Skip(theFrom).Take(theTo - theFrom).ToList();
    }

    private static string Lindex(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            throw Usage("lindex list ?index ...?");
        }

        string aValue = a[1];
        foreach (string anIndex in Indices(a, 2, a.Count))
        {
            IReadOnlyList<string> aList = TclList.Parse(aValue);
            long k = TclList.ParseIndex(anIndex, aList.Count);
            if (k < 0 || k >= aList.Count)
            {
                return "";
            }

            aValue = aList[(int)k];
        }

        return aValue;
    }

    private static string Lrange(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count != 4)
        {
            throw Usage("lrange list first last");
        }

        IReadOnlyList<string> aList = TclList.Parse(a[1]);
        long aFirst = Math.Max(0, TclList.ParseIndex(a[2], aList.Count));
        long aLast = Math.Min(aList.Count - 1, TclList.ParseIndex(a[3], aList.Count));
        if (aFirst > aLast)
        {
            return "";
        }

        var aRange = new string[aLast - aFirst + 1];
        for (long i = aFirst; i <= aLast; i++)
        {
            aRange[i - aFirst] = aList[(int)i];
        }

        return TclList.Make(aRange);
    }

    private static string Lappend(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            throw Usage("lappend varName ?value ...?");
        }

        // a list lappend made, extended where nothing reads the result: in place, so that a list built element by element
        // (OCCT's _check_log builds its HTML log of every line so) costs its length, not its length squared (decision 2084)
        if (theInterpreter.ResultUnused && a.Count > 2 && theInterpreter.FindVar(a[1], false) is { IsMadeList: true, Array: null, Drawable: null } aVar)
        {
            aVar.AppendListElements(a.Skip(2));
            return "";
        }

        theInterpreter.TryGetVariable(a[1], out string anOld);
        string aNew = a.Count == 2 ? TclList.Make([.. TclList.Parse(anOld)]) : TclList.Append(anOld, a.Skip(2).ToList());
        if (a.Count == 2 && theInterpreter.TryGetVariable(a[1], out _))
        {
            // lappend with no values leaves the value unchanged
            return anOld;
        }

        theInterpreter.SetVariable(a[1], aNew);
        if (a.Count > 2 && theInterpreter.FindVar(a[1], false) is { Array: null } aSet)
        {
            aSet.IsMadeList = true;
        }

        return aNew;
    }

    private static string Linsert(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            throw Usage("linsert list index ?element ...?");
        }

        var aList = TclList.Parse(a[1]).ToList();
        int aPosition = (int)Math.Clamp(TclList.ParseIndex(a[2], aList.Count + 1), 0, aList.Count);
        aList.InsertRange(aPosition, a.Skip(3));
        return TclList.Make([.. aList]);
    }

    private static string Lreplace(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 4)
        {
            throw Usage("lreplace list first last ?element ...?");
        }

        var aList = TclList.Parse(a[1]).ToList();
        int aFirst = (int)Math.Clamp(TclList.ParseIndex(a[2], aList.Count), 0, aList.Count);
        long aLast = Math.Min(aList.Count - 1, TclList.ParseIndex(a[3], aList.Count));
        if (aLast >= aFirst)
        {
            aList.RemoveRange(aFirst, (int)(aLast - aFirst + 1));
        }

        aList.InsertRange(aFirst, a.Skip(4));
        return TclList.Make([.. aList]);
    }

    private static string Lassign(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            throw Usage("lassign list ?varName ...?");
        }

        IReadOnlyList<string> aList = TclList.Parse(a[1]);
        for (int i = 2; i < a.Count; i++)
        {
            theInterpreter.SetVariable(a[i], i - 2 < aList.Count ? aList[i - 2] : "");
        }

        return TclList.Make(aList.Skip(a.Count - 2).ToArray());
    }

    private static string Lrepeat(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            throw Usage("lrepeat count ?value ...?");
        }

        long aCount = TclNumber.GetInt(a[1]);
        if (aCount < 0)
        {
            throw new DrawException($"bad count \"{a[1]}\": must be integer >= 0");
        }

        var aList = new List<string>();
        for (long i = 0; i < aCount; i++)
        {
            aList.AddRange(a.Skip(2));
        }

        return TclList.Make([.. aList]);
    }

    private static string Lset(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            throw Usage("lset listVar ?index? ?index ...? value");
        }

        string anOld = theInterpreter.GetVariable(a[1]);
        IReadOnlyList<string> anIndices = a.Count == 3 ? [] : Indices(a, 2, a.Count - 1);
        string aNew = SetIn(anOld, anIndices, 0, a[^1]);
        theInterpreter.SetVariable(a[1], aNew);
        return aNew;
    }

    private static string SetIn(string theList, IReadOnlyList<string> theIndices, int theFrom, string theValue)
    {
        if (theFrom == theIndices.Count)
        {
            return theValue;
        }

        var aList = TclList.Parse(theList).ToList();
        long k = TclList.ParseIndex(theIndices[theFrom], aList.Count);
        if (k < 0 || k > aList.Count)
        {
            throw new DrawException("list index out of range");
        }

        if (k == aList.Count)
        {
            aList.Add(SetIn("", theIndices, theFrom + 1, theValue));
        }
        else
        {
            aList[(int)k] = SetIn(aList[(int)k], theIndices, theFrom + 1, theValue);
        }

        return TclList.Make([.. aList]);
    }

    private static string Lsearch(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        string aMode = "-glob";
        string aType = "-ascii";
        bool anAll = false, anInline = false, aNot = false, aNoCase = false;
        int aStart = 0;
        string? anIndex = null;
        int i = 1;
        for (; i < a.Count - 2; i++)
        {
            switch (a[i].StartsWith('-') ? TclControlCommands.Resolve(a[i], "option", ["-exact", "-glob", "-regexp", "-all", "-inline", "-not", "-nocase", "-integer", "-real", "-ascii", "-dictionary", "-sorted", "-increasing", "-decreasing", "-bisect", "-subindices", "-start", "-index"]) : a[i])
            {
                case "-exact" or "-glob" or "-regexp":
                    aMode = a[i];
                    break;
                case "-all":
                    anAll = true;
                    break;
                case "-inline":
                    anInline = true;
                    break;
                case "-not":
                    aNot = true;
                    break;
                case "-nocase":
                    aNoCase = true;
                    break;
                case "-integer" or "-real" or "-ascii" or "-dictionary":
                    aType = a[i];
                    if (aMode == "-glob" && aType is "-integer" or "-real")
                    {
                        aMode = "-exact";
                    }

                    break;
                case "-sorted":
                    aMode = "-exact";
                    break;
                case "-increasing" or "-decreasing" or "-bisect" or "-subindices":
                    break;
                case "-start":
                    aStart = (int)Math.Max(0, TclList.ParseIndex(a[++i], int.MaxValue));
                    break;
                case "-index":
                    anIndex = a[++i];
                    break;
                default:
                    throw new DrawException($"bad option \"{a[i]}\"");
            }
        }

        if (a.Count - i != 2)
        {
            throw Usage("lsearch ?-option value ...? list pattern");
        }

        IReadOnlyList<string> aList = TclList.Parse(a[i]);
        string aPattern = a[i + 1];
        var aMatches = new List<int>();
        for (int k = aStart; k < aList.Count; k++)
        {
            string anElement = anIndex is null ? aList[k] : Lindex(theInterpreter, ["lindex", aList[k], anIndex]);
            bool aMatch = aMode switch
            {
                "-exact" => aType switch
                {
                    "-integer" => TclNumber.TryParseInt(anElement, out long x) && x == TclNumber.GetInt(aPattern),
                    "-real" => TclNumber.TryParseDouble(anElement, out double y) && y == TclNumber.GetDouble(aPattern),
                    _ => string.Equals(anElement, aPattern, aNoCase ? StringComparison.OrdinalIgnoreCase : StringComparison.Ordinal),
                },
                "-regexp" => TclRegex.Create(aPattern, aNoCase).IsMatch(anElement),
                _ => TclGlob.Match(aPattern, anElement, aNoCase),
            };
            if (aMatch != aNot)
            {
                aMatches.Add(k);
                if (!anAll)
                {
                    break;
                }
            }
        }

        if (anInline)
        {
            return anAll ? TclList.Make(aMatches.Select(k => aList[k]).ToArray()) : aMatches.Count > 0 ? aList[aMatches[0]] : "";
        }

        return anAll
                   ? string.Join(' ', aMatches.Select(k => k.ToString(CultureInfo.InvariantCulture)))
                   : (aMatches.Count > 0 ? aMatches[0] : -1).ToString(CultureInfo.InvariantCulture);
    }

    private static string Lsort(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        string aType = "-ascii";
        bool aDecreasing = false, aUnique = false, aNoCase = false, anIndices = false;
        string? anIndex = null;
        string? aCommand = null;
        int i = 1;
        for (; i < a.Count - 1; i++)
        {
            switch (a[i].StartsWith('-') ? TclControlCommands.Resolve(a[i], "option", ["-ascii", "-dictionary", "-integer", "-real", "-increasing", "-decreasing", "-unique", "-nocase", "-indices", "-index", "-command"]) : a[i])
            {
                case "-ascii" or "-dictionary" or "-integer" or "-real":
                    aType = a[i];
                    break;
                case "-increasing":
                    aDecreasing = false;
                    break;
                case "-decreasing":
                    aDecreasing = true;
                    break;
                case "-unique":
                    aUnique = true;
                    break;
                case "-nocase":
                    aNoCase = true;
                    break;
                case "-indices":
                    anIndices = true;
                    break;
                case "-index":
                    anIndex = a[++i];
                    break;
                case "-command":
                    aCommand = a[++i];
                    break;
                default:
                    throw new DrawException($"bad option \"{a[i]}\": must be -ascii, -command, -decreasing, -dictionary, -increasing, -index, -indices, -integer, -nocase, -real, or -unique");
            }
        }

        if (i != a.Count - 1)
        {
            throw Usage("lsort ?-option value ...? list");
        }

        IReadOnlyList<string> aList = TclList.Parse(a[^1]);
        string[] aKeys = anIndex is null ? [.. aList] : aList.Select(e => Lindex(theInterpreter, ["lindex", e, anIndex])).ToArray();
        Comparison<string> aCompare = aCommand is not null
                                          ? (x, y) => (int)TclNumber.GetInt(theInterpreter.EvalScript($"{aCommand} {TclList.Quote(x)} {TclList.Quote(y)}"))
                                          : aType switch
                                          {
                                              "-integer" => static (x, y) => TclNumber.GetInt(x).CompareTo(TclNumber.GetInt(y)),
                                              "-real" => static (x, y) => TclNumber.GetDouble(x).CompareTo(TclNumber.GetDouble(y)),
                                              "-dictionary" => DictionaryCompare,
                                              _ when aNoCase => static (x, y) => string.CompareOrdinal(x.ToLowerInvariant(), y.ToLowerInvariant()),
                                              _ => string.CompareOrdinal,
                                          };
        // Numeric keys are checked first: an exception inside the sort would be wrapped by .NET
        foreach (string aKey in aCommand is null ? aKeys : [])
        {
            _ = aType switch
            {
                "-integer" => TclNumber.GetInt(aKey),
                "-real" => TclNumber.GetDouble(aKey),
                _ => 0,
            };
        }

        Comparer<int> aComparer = Comparer<int>.Create((x, y) => aCompare(aKeys[x], aKeys[y]));
        IEnumerable<int> anOrder = Enumerable.Range(0, aList.Count);
        List<int> aSorted;
        try
        {
            aSorted = (aDecreasing ? anOrder.OrderByDescending(k => k, aComparer) : anOrder.OrderBy(k => k, aComparer)).ToList();
        }
        catch (InvalidOperationException anError) when (anError.InnerException is DrawException or TclFlowException)
        {
            ExceptionDispatchInfo.Throw(anError.InnerException);
            throw;
        }
        if (aUnique)
        {
            // Of equal elements, the last is kept
            var aKept = new List<int>();
            for (int k = 0; k < aSorted.Count; k++)
            {
                if (k + 1 < aSorted.Count && aCompare(aKeys[aSorted[k]], aKeys[aSorted[k + 1]]) == 0)
                {
                    continue;
                }

                aKept.Add(aSorted[k]);
            }

            aSorted = aKept;
        }

        return anIndices
                   ? string.Join(' ', aSorted.Select(k => k.ToString(CultureInfo.InvariantCulture)))
                   : TclList.Make(aSorted.Select(k => aList[k]).ToArray());
    }

    /// <summary>lsort -dictionary: case-insensitive, with embedded numbers compared as integers.</summary>
    internal static int DictionaryCompare(string a, string b)
    {
        int i = 0, j = 0, aTieBreak = 0;
        while (i < a.Length && j < b.Length)
        {
            if (char.IsAsciiDigit(a[i]) && char.IsAsciiDigit(b[j]))
            {
                int aZerosA = 0, aZerosB = 0;
                while (i < a.Length && a[i] == '0' && i + 1 < a.Length && char.IsAsciiDigit(a[i + 1]))
                {
                    i++;
                    aZerosA++;
                }

                while (j < b.Length && b[j] == '0' && j + 1 < b.Length && char.IsAsciiDigit(b[j + 1]))
                {
                    j++;
                    aZerosB++;
                }

                int anEndA = i, anEndB = j;
                while (anEndA < a.Length && char.IsAsciiDigit(a[anEndA]))
                {
                    anEndA++;
                }

                while (anEndB < b.Length && char.IsAsciiDigit(b[anEndB]))
                {
                    anEndB++;
                }

                int aLengthDiff = (anEndA - i) - (anEndB - j);
                if (aLengthDiff != 0)
                {
                    return aLengthDiff;
                }

                int aDigits = string.CompareOrdinal(a, i, b, j, anEndA - i);
                if (aDigits != 0)
                {
                    return aDigits;
                }

                if (aTieBreak == 0)
                {
                    aTieBreak = aZerosB - aZerosA;
                }

                i = anEndA;
                j = anEndB;
                continue;
            }

            char ca = char.ToLowerInvariant(a[i]);
            char cb = char.ToLowerInvariant(b[j]);
            if (ca != cb)
            {
                return ca - cb;
            }

            if (aTieBreak == 0 && a[i] != b[j])
            {
                aTieBreak = char.IsUpper(a[i]) ? -1 : 1;
            }

            i++;
            j++;
        }

        int aRemaining = (a.Length - i) - (b.Length - j);
        return aRemaining != 0 ? aRemaining : aTieBreak;
    }
}
