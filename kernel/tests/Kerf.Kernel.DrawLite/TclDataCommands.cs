// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Kerf.Kernel.DrawLite;

/// <summary>Tcl's array and dict commands.</summary>
internal static class TclDataCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        theInterpreter.Register("array", Array);
        theInterpreter.Register("dict", Dict);
    }

    private static DrawException Usage(string theUsage) => TclControlCommands.Usage(theUsage);

    // -------------------------------------------------------------------------------------------------------------
    // array

    private static string Array(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            throw Usage("array subcommand ?arg ...?");
        }

        string aName = a[2];
        Dictionary<string, string>? anArray = theInterpreter.FindVar(aName, false)?.Array;
        switch (TclControlCommands.Resolve(a[1], "subcommand", ["exists", "size", "names", "get", "set", "unset", "anymore", "donesearch", "nextelement", "startsearch", "statistics"]))
        {
            case "exists":
                return anArray is null ? "0" : "1";
            case "size":
                return (anArray?.Count ?? 0).ToString(CultureInfo.InvariantCulture);
            case "names":
            {
                if (anArray is null)
                {
                    return "";
                }

                string aMode = a.Count == 5 ? a[3] : "-glob";
                string? aPattern = a.Count >= 4 ? a[^1] : null;
                return TclList.Make(anArray.Keys.Where(k => aPattern is null || Matches(aMode, aPattern, k)).ToArray());
            }

            case "get":
            {
                if (anArray is null)
                {
                    return "";
                }

                string? aPattern = a.Count >= 4 ? a[3] : null;
                var aPairs = new List<string>();
                foreach ((string aKey, string aValue) in anArray)
                {
                    if (aPattern is null || TclGlob.Match(aPattern, aKey))
                    {
                        aPairs.Add(aKey);
                        aPairs.Add(aValue);
                    }
                }

                return TclList.Make([.. aPairs]);
            }

            case "set" when a.Count == 4:
            {
                IReadOnlyList<string> aPairs = TclList.Parse(a[3]);
                if (aPairs.Count % 2 != 0)
                {
                    throw new DrawException("list must have an even number of elements");
                }

                TclVar aVar = theInterpreter.FindVar(aName, true)!;
                if (aVar.Value is not null)
                {
                    throw new DrawException($"can't set \"{aName}(...)\": variable isn't array");
                }

                aVar.Array ??= new Dictionary<string, string>(StringComparer.Ordinal);
                for (int i = 0; i < aPairs.Count; i += 2)
                {
                    aVar.Array[aPairs[i]] = aPairs[i + 1];
                    aVar.ElementDrawables?.Remove(aPairs[i]);
                }

                return "";
            }

            case "unset":
            {
                if (anArray is null)
                {
                    return "";
                }

                if (a.Count == 3)
                {
                    theInterpreter.UnsetVariable(aName);
                }
                else
                {
                    foreach (string aKey in anArray.Keys.Where(k => TclGlob.Match(a[3], k)).ToList())
                    {
                        anArray.Remove(aKey);
                        theInterpreter.FindVar(aName, false)?.ElementDrawables?.Remove(aKey);
                    }
                }

                return "";
            }

            default:
                throw new DrawException($"bad option \"{a[1]}\": must be exists, get, names, set, size, or unset");
        }
    }

    private static bool Matches(string theMode, string thePattern, string theText) => theMode switch
    {
        "-exact" => thePattern == theText,
        "-regexp" => TclRegex.Create(thePattern).IsMatch(theText),
        _ => TclGlob.Match(thePattern, theText),
    };

    // -------------------------------------------------------------------------------------------------------------
    // dict

    /// <summary>A dict's key/value pairs, in order; a repeated key keeps its first position and last value.</summary>
    internal static List<KeyValuePair<string, string>> ParseDict(string theDict)
    {
        IReadOnlyList<string> aList = TclList.Parse(theDict);
        if (aList.Count % 2 != 0)
        {
            throw new DrawException("missing value to go with key");
        }

        var aPairs = new List<KeyValuePair<string, string>>(aList.Count / 2);
        for (int i = 0; i < aList.Count; i += 2)
        {
            Put(aPairs, aList[i], aList[i + 1]);
        }

        return aPairs;
    }

    private static void Put(List<KeyValuePair<string, string>> thePairs, string theKey, string theValue)
    {
        int anIndex = thePairs.FindIndex(p => p.Key == theKey);
        if (anIndex >= 0)
        {
            thePairs[anIndex] = new(theKey, theValue);
        }
        else
        {
            thePairs.Add(new(theKey, theValue));
        }
    }

    private static string? Find(List<KeyValuePair<string, string>> thePairs, string theKey)
    {
        int anIndex = thePairs.FindIndex(p => p.Key == theKey);
        return anIndex >= 0 ? thePairs[anIndex].Value : null;
    }

    internal static string FormatDict(IEnumerable<KeyValuePair<string, string>> thePairs)
        => TclList.Make(thePairs.SelectMany(p => new[] { p.Key, p.Value }).ToArray());

    private static string GetPath(string theDict, IReadOnlyList<string> theKeys, int theFrom, int theTo)
    {
        string aValue = theDict;
        for (int i = theFrom; i < theTo; i++)
        {
            aValue = Find(ParseDict(aValue), theKeys[i]) ?? throw new DrawException($"key \"{theKeys[i]}\" not known in dictionary");
        }

        return aValue;
    }

    private static string SetPath(string theDict, IReadOnlyList<string> theKeys, int theFrom, int theTo, string? theValue)
    {
        List<KeyValuePair<string, string>> aPairs = ParseDict(theDict);
        string aKey = theKeys[theFrom];
        if (theFrom == theTo - 1)
        {
            if (theValue is null)
            {
                aPairs.RemoveAll(p => p.Key == aKey);
            }
            else
            {
                Put(aPairs, aKey, theValue);
            }
        }
        else
        {
            string? anInner = Find(aPairs, aKey);
            if (anInner is null && theValue is null)
            {
                return FormatDict(aPairs);
            }

            Put(aPairs, aKey, SetPath(anInner ?? "", theKeys, theFrom + 1, theTo, theValue));
        }

        return FormatDict(aPairs);
    }

    private static string Dict(DrawInterpreter theInterpreter, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            throw Usage("dict subcommand ?arg ...?");
        }

        switch (TclControlCommands.Resolve(a[1], "subcommand", ["create", "get", "exists", "size", "keys", "values", "set", "unset", "remove", "replace", "merge", "append", "lappend", "incr", "for", "filter", "info", "map", "update", "with"]))
        {
            case "create":
            {
                if (a.Count % 2 != 0)
                {
                    throw Usage("dict create ?key value ...?");
                }

                var aPairs = new List<KeyValuePair<string, string>>();
                for (int i = 2; i < a.Count; i += 2)
                {
                    Put(aPairs, a[i], a[i + 1]);
                }

                return FormatDict(aPairs);
            }

            case "get" when a.Count >= 3:
                return a.Count == 3 ? FormatDict(ParseDict(a[2])) : GetPath(a[2], a, 3, a.Count);
            case "exists" when a.Count >= 4:
            {
                try
                {
                    GetPath(a[2], a, 3, a.Count);
                    return "1";
                }
                catch (DrawException)
                {
                    return "0";
                }
            }

            case "size" when a.Count == 3:
                return ParseDict(a[2]).Count.ToString(CultureInfo.InvariantCulture);
            case "keys" or "values" when a.Count is 3 or 4:
            {
                bool aKeys = a[1] == "keys";
                return TclList.Make(ParseDict(a[2]).Where(p => a.Count == 3 || TclGlob.Match(a[3], aKeys ? p.Key : p.Value))
                                                   .Select(p => aKeys ? p.Key : p.Value).ToArray());
            }

            case "set" when a.Count >= 5:
            {
                theInterpreter.TryGetVariable(a[2], out string aDict);
                string aNew = SetPath(aDict, a, 3, a.Count - 1, a[^1]);
                theInterpreter.SetVariable(a[2], aNew);
                return aNew;
            }

            case "unset" when a.Count >= 4:
            {
                theInterpreter.TryGetVariable(a[2], out string aDict);
                string aNew = SetPath(aDict, a, 3, a.Count, null);
                theInterpreter.SetVariable(a[2], aNew);
                return aNew;
            }

            case "remove" when a.Count >= 3:
            {
                List<KeyValuePair<string, string>> aPairs = ParseDict(a[2]);
                foreach (string aKey in a.Skip(3))
                {
                    aPairs.RemoveAll(p => p.Key == aKey);
                }

                return FormatDict(aPairs);
            }

            case "replace" when a.Count >= 3 && a.Count % 2 == 1:
            {
                List<KeyValuePair<string, string>> aPairs = ParseDict(a[2]);
                for (int i = 3; i < a.Count; i += 2)
                {
                    Put(aPairs, a[i], a[i + 1]);
                }

                return FormatDict(aPairs);
            }

            case "merge":
            {
                var aPairs = new List<KeyValuePair<string, string>>();
                foreach (string aDict in a.Skip(2))
                {
                    foreach (KeyValuePair<string, string> aPair in ParseDict(aDict))
                    {
                        Put(aPairs, aPair.Key, aPair.Value);
                    }
                }

                return FormatDict(aPairs);
            }

            case "append" or "lappend" when a.Count >= 4:
            {
                theInterpreter.TryGetVariable(a[2], out string aDict);
                List<KeyValuePair<string, string>> aPairs = ParseDict(aDict);
                string anOld = Find(aPairs, a[3]) ?? "";
                string aValue = a[1] == "append" ? anOld + string.Concat(a.Skip(4)) : TclList.Append(anOld, a.Skip(4).ToList());
                Put(aPairs, a[3], aValue);
                string aNew = FormatDict(aPairs);
                theInterpreter.SetVariable(a[2], aNew);
                return aNew;
            }

            case "incr" when a.Count is 4 or 5:
            {
                theInterpreter.TryGetVariable(a[2], out string aDict);
                List<KeyValuePair<string, string>> aPairs = ParseDict(aDict);
                long anOld = Find(aPairs, a[3]) is { } aText ? TclNumber.GetInt(aText) : 0;
                long anIncrement = a.Count == 5 ? TclNumber.GetInt(a[4]) : 1;
                Put(aPairs, a[3], TclNumber.FormatInt(anOld + anIncrement));
                string aNew = FormatDict(aPairs);
                theInterpreter.SetVariable(a[2], aNew);
                return aNew;
            }

            case "for" when a.Count == 5:
            {
                IReadOnlyList<string> aVars = TclList.Parse(a[2]);
                if (aVars.Count != 2)
                {
                    throw new DrawException("must have exactly two variable names");
                }

                foreach (KeyValuePair<string, string> aPair in ParseDict(a[3]))
                {
                    theInterpreter.SetVariable(aVars[0], aPair.Key);
                    theInterpreter.SetVariable(aVars[1], aPair.Value);
                    try
                    {
                        theInterpreter.EvalScript(a[4]);
                    }
                    catch (TclBreakException)
                    {
                        break;
                    }
                    catch (TclContinueException)
                    {
                    }
                }

                return "";
            }

            case "filter" when a.Count >= 4:
            {
                List<KeyValuePair<string, string>> aPairs = ParseDict(a[2]);
                Func<KeyValuePair<string, string>, bool> aKeep = a[3] switch
                {
                    "key" => p => a.Skip(4).Any(g => TclGlob.Match(g, p.Key)),
                    "value" => p => a.Skip(4).Any(g => TclGlob.Match(g, p.Value)),
                    _ => throw new DrawException($"dict filter {a[3]}: not supported by Draw-lite"),
                };
                return FormatDict(aPairs.Where(aKeep));
            }

            case "info":
                return "";
            default:
                throw new DrawException($"dict {a[1]}: unknown subcommand or wrong # args");
        }
    }
}
