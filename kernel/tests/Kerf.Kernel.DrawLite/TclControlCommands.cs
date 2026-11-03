// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Globalization;
using System.Text;

namespace Kerf.Kernel.DrawLite;

/// <summary>Tcl's variables, control flow, procedures and evaluation commands.</summary>
internal static class TclControlCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        theInterpreter.Register("set", Set);
        theInterpreter.Register("unset", Unset);
        theInterpreter.Register("incr", Incr);
        theInterpreter.Register("append", Append);
        theInterpreter.Register("if", If);
        theInterpreter.Register("while", While);
        theInterpreter.Register("for", For);
        theInterpreter.Register("foreach", (i, a) => Foreach(i, a, theCollect: false));
        theInterpreter.Register("lmap", (i, a) => Foreach(i, a, theCollect: true));
        theInterpreter.Register("break", static (_, a) => a.Count == 1 ? throw TclBreakException.Instance : throw Usage("break"));
        theInterpreter.Register("continue", static (_, a) => a.Count == 1 ? throw TclContinueException.Instance : throw Usage("continue"));
        theInterpreter.Register("return", Return);
        theInterpreter.Register("catch", Catch);
        theInterpreter.Register("try", Try);
        theInterpreter.Register("error", Error);
        theInterpreter.Register("throw", static (_, a) => a.Count == 3 ? throw new DrawException(a[2]) { ErrorCode = a[1] } : throw Usage("throw type message"));
        theInterpreter.Register("eval", static (i, a) => a.Count >= 2 ? i.EvalScript(Concat(a, 1)) : throw Usage("eval arg ?arg ...?"));
        theInterpreter.Register("uplevel", Uplevel);
        theInterpreter.Register("upvar", Upvar);
        theInterpreter.Register("global", Global);
        theInterpreter.Register("variable", Variable);
        theInterpreter.Register("proc", Proc);
        theInterpreter.Register("rename", static (i, a) =>
        {
            if (a.Count != 3)
            {
                throw Usage("rename oldName newName");
            }

            i.RenameCommand(a[1], a[2]);
            return "";
        });
        theInterpreter.Register("switch", Switch);
        theInterpreter.Register("expr", static (i, a) => a.Count >= 2 ? TclExpr.Evaluate(i, Concat(a, 1)) : throw Usage("expr arg ?arg ...?"));
        theInterpreter.Register("time", Time);
        theInterpreter.Register("exit", static (_, a) => throw new DrawExitException(a.Count > 1 ? (int)TclNumber.GetInt(a[1]) : 0));
        theInterpreter.Register("namespace", Namespace);
    }

    internal static DrawException Usage(string theUsage) => new($"wrong # args: should be \"{theUsage}\"");

    /// <summary>
    /// A subcommand or option name: the name itself or, as Tcl allows, a unique prefix of it.
    /// </summary>
    internal static string Resolve(string theWord, string theWhat, IReadOnlyList<string> theNames)
    {
        if (theNames.Contains(theWord))
        {
            return theWord;
        }

        var aMatches = theNames.Where(n => theWord.Length > 0 && n.StartsWith(theWord, StringComparison.Ordinal)).ToList();
        if (aMatches.Count == 1)
        {
            return aMatches[0];
        }

        var aSorted = theNames.Order(StringComparer.Ordinal).ToList();
        string aList = aSorted.Count == 1 ? aSorted[0] : string.Join(", ", aSorted.Take(aSorted.Count - 1)) + ", or " + aSorted[^1];
        throw new DrawException($"{(aMatches.Count > 1 ? "ambiguous" : "unknown or ambiguous")} {theWhat} \"{theWord}\": must be {aList}");
    }

    /// <summary>Tcl's concat: the arguments trimmed of blanks and joined by single spaces, empty ones dropped.</summary>
    internal static string Concat(IReadOnlyList<string> theArgs, int theFrom)
    {
        if (theArgs.Count - theFrom == 1)
        {
            return theArgs[theFrom];
        }

        var aBuilder = new StringBuilder();
        for (int i = theFrom; i < theArgs.Count; i++)
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

    private static string Set(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        return theArgs.Count switch
        {
            2 => theInterpreter.GetVariable(theArgs[1]),
            3 => SetValue(theInterpreter, theArgs[1], theArgs[2]),
            _ => throw Usage("set varName ?newValue?"),
        };
    }

    private static string SetValue(DrawInterpreter theInterpreter, string theName, string theValue)
    {
        (string aName, string? anIndex) = DrawInterpreter.SplitName(theName);
        return theInterpreter.SetVariable(aName, anIndex, theValue);
    }

    private static string Unset(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        int i = 1;
        bool aNoComplain = false;
        if (i < theArgs.Count && theArgs[i] == "-nocomplain")
        {
            aNoComplain = true;
            i++;
        }

        if (i < theArgs.Count && theArgs[i] == "--")
        {
            i++;
        }

        for (; i < theArgs.Count; i++)
        {
            if (!theInterpreter.UnsetVariable(theArgs[i]) && !aNoComplain)
            {
                (string aName, string? anIndex) = DrawInterpreter.SplitName(theArgs[i]);
                bool anArray = anIndex is not null && theInterpreter.FindVar(aName, false)?.Array is not null;
                throw new DrawException($"can't unset \"{theArgs[i]}\": {(anArray ? "no such element in array" : "no such variable")}");
            }
        }

        return "";
    }

    private static string Incr(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        if (theArgs.Count is < 2 or > 3)
        {
            throw Usage("incr varName ?increment?");
        }

        long anIncrement = theArgs.Count == 3 ? TclNumber.GetInt(theArgs[2]) : 1;
        long aValue = theInterpreter.TryGetVariable(theArgs[1], out string anOld) ? TclNumber.GetInt(anOld) : 0;
        return SetValue(theInterpreter, theArgs[1], TclNumber.FormatInt(unchecked(aValue + anIncrement)));
    }

    private static string Append(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        if (theArgs.Count < 2)
        {
            throw Usage("append varName ?value value ...?");
        }

        theInterpreter.TryGetVariable(theArgs[1], out string anOld);
        if (theArgs.Count == 2)
        {
            return theInterpreter.TryGetVariable(theArgs[1], out _) ? anOld : SetValue(theInterpreter, theArgs[1], "");
        }

        var aBuilder = new StringBuilder(anOld);
        for (int i = 2; i < theArgs.Count; i++)
        {
            aBuilder.Append(theArgs[i]);
        }

        return SetValue(theInterpreter, theArgs[1], aBuilder.ToString());
    }

    private static string If(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        int i = 1;
        while (true)
        {
            if (i >= theArgs.Count)
            {
                throw new DrawException($"wrong # args: no expression after \"{theArgs[i - 1]}\" argument");
            }

            string aCondition = theArgs[i++];
            if (i < theArgs.Count && theArgs[i] == "then")
            {
                i++;
            }

            if (i >= theArgs.Count)
            {
                throw new DrawException($"wrong # args: no script following \"{theArgs[i - 1]}\" argument");
            }

            string aBody = theArgs[i++];
            if (TclExpr.EvalBoolean(theInterpreter, aCondition))
            {
                return theInterpreter.EvalScript(aBody);
            }

            if (i >= theArgs.Count)
            {
                return "";
            }

            if (theArgs[i] == "elseif")
            {
                i++;
                continue;
            }

            if (theArgs[i] == "else")
            {
                i++;
                if (i >= theArgs.Count)
                {
                    throw new DrawException("wrong # args: no script following \"else\" argument");
                }
            }

            if (i != theArgs.Count - 1)
            {
                throw new DrawException("wrong # args: extra words after \"else\" clause in \"if\" command");
            }

            return theInterpreter.EvalScript(theArgs[i]);
        }
    }

    /// <summary>Runs a loop body: false when the loop should stop (break).</summary>
    private static bool RunBody(DrawInterpreter theInterpreter, string theBody, List<string>? theResults = null)
    {
        try
        {
            string aResult = theInterpreter.EvalScript(theBody);
            theResults?.Add(aResult);
            return true;
        }
        catch (TclBreakException)
        {
            return false;
        }
        catch (TclContinueException)
        {
            return true;
        }
    }

    private static string While(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        if (theArgs.Count != 3)
        {
            throw Usage("while test command");
        }

        while (TclExpr.EvalBoolean(theInterpreter, theArgs[1]))
        {
            if (!RunBody(theInterpreter, theArgs[2]))
            {
                break;
            }
        }

        return "";
    }

    private static string For(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        if (theArgs.Count != 5)
        {
            throw Usage("for start test next command");
        }

        theInterpreter.EvalScript(theArgs[1]);
        while (TclExpr.EvalBoolean(theInterpreter, theArgs[2]))
        {
            if (!RunBody(theInterpreter, theArgs[4]))
            {
                break;
            }

            try
            {
                theInterpreter.EvalScript(theArgs[3]);
            }
            catch (TclBreakException)
            {
                break;
            }
        }

        return "";
    }

    private static string Foreach(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs, bool theCollect)
    {
        string aName = theCollect ? "lmap" : "foreach";
        if (theArgs.Count < 4 || theArgs.Count % 2 != 0)
        {
            throw Usage($"{aName} varList list ?varList list ...? command");
        }

        var aVariables = new List<IReadOnlyList<string>>();
        var aLists = new List<IReadOnlyList<string>>();
        int anIterations = 0;
        for (int i = 1; i < theArgs.Count - 1; i += 2)
        {
            IReadOnlyList<string> aNames = TclList.Parse(theArgs[i]);
            if (aNames.Count == 0)
            {
                throw new DrawException($"{aName} varlist is empty");
            }

            IReadOnlyList<string> aValues = TclList.Parse(theArgs[i + 1]);
            aVariables.Add(aNames);
            aLists.Add(aValues);
            anIterations = Math.Max(anIterations, (aValues.Count + aNames.Count - 1) / aNames.Count);
        }

        string aBody = theArgs[^1];
        List<string>? aResults = theCollect ? [] : null;
        for (int anIteration = 0; anIteration < anIterations; anIteration++)
        {
            for (int k = 0; k < aVariables.Count; k++)
            {
                IReadOnlyList<string> aNames = aVariables[k];
                for (int j = 0; j < aNames.Count; j++)
                {
                    int anIndex = (anIteration * aNames.Count) + j;
                    SetValue(theInterpreter, aNames[j], anIndex < aLists[k].Count ? aLists[k][anIndex] : "");
                }
            }

            if (!RunBody(theInterpreter, aBody, aResults))
            {
                break;
            }
        }

        return aResults is null ? "" : TclList.Make([.. aResults]);
    }

    internal static int ParseCode(string theCode) => theCode switch
    {
        "ok" => 0,
        "error" => 1,
        "return" => 2,
        "break" => 3,
        "continue" => 4,
        _ => TclNumber.TryParseInt(theCode, out long aCode)
                 ? (int)aCode
                 : throw new DrawException($"bad completion code \"{theCode}\": must be ok, error, return, break, continue, or an integer"),
    };

    private static string Return(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        int aCode = 0;
        int aLevel = 1;
        string anErrorCode = "NONE";
        int anOptionsEnd = theArgs.Count % 2 == 0 ? theArgs.Count - 1 : theArgs.Count;
        string aValue = theArgs.Count % 2 == 0 ? theArgs[^1] : "";
        for (int i = 1; i < anOptionsEnd; i += 2)
        {
            switch (theArgs[i])
            {
                case "-code":
                    aCode = ParseCode(theArgs[i + 1]);
                    break;
                case "-level":
                    aLevel = (int)TclNumber.GetInt(theArgs[i + 1]);
                    break;
                case "-errorcode":
                    anErrorCode = theArgs[i + 1];
                    break;
                case "-options":
                {
                    IReadOnlyList<string> anOptions = TclList.Parse(theArgs[i + 1]);
                    for (int k = 0; k + 1 < anOptions.Count; k += 2)
                    {
                        if (anOptions[k] == "-code")
                        {
                            aCode = ParseCode(anOptions[k + 1]);
                        }
                        else if (anOptions[k] == "-level")
                        {
                            aLevel = (int)TclNumber.GetInt(anOptions[k + 1]);
                        }
                    }

                    break;
                }
            }
        }

        if (aLevel == 0)
        {
            return aCode switch
            {
                0 => aValue,
                1 => throw new DrawException(aValue) { ErrorCode = anErrorCode },
                2 => throw new TclReturnException(0, aValue),
                3 => throw TclBreakException.Instance,
                4 => throw TclContinueException.Instance,
                _ => aValue,
            };
        }

        throw new TclReturnException(aCode, aValue) { ErrorCode = anErrorCode };
    }

    /// <summary>Runs a script and reports how it ended: Tcl's return code, result, and error code.</summary>
    internal static (int Code, string Result, string ErrorCode) Run(DrawInterpreter theInterpreter, string theScript)
    {
        try
        {
            return (0, theInterpreter.EvalScript(theScript), "");
        }
        catch (DrawException anError)
        {
            theInterpreter.SetVariable("::errorInfo", null, anError.Message);
            theInterpreter.SetVariable("::errorCode", null, anError.ErrorCode);
            return (1, anError.Reason, anError.ErrorCode);
        }
        catch (TclReturnException aReturn)
        {
            return (2, aReturn.Value, aReturn.ErrorCode);
        }
        catch (TclBreakException)
        {
            return (3, "", "");
        }
        catch (TclContinueException)
        {
            return (4, "", "");
        }
    }

    private static string Catch(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        if (theArgs.Count is < 2 or > 4)
        {
            throw Usage("catch script ?resultVarName? ?optionVarName?");
        }

        (int aCode, string aResult, string anErrorCode) = Run(theInterpreter, theArgs[1]);
        if (theArgs.Count > 2)
        {
            SetValue(theInterpreter, theArgs[2], aResult);
        }

        if (theArgs.Count > 3)
        {
            var anOptions = new List<string> { "-code", aCode.ToString(CultureInfo.InvariantCulture), "-level", aCode == 2 ? "1" : "0" };
            if (aCode == 1)
            {
                anOptions.AddRange(["-errorcode", anErrorCode, "-errorinfo", aResult]);
            }

            SetValue(theInterpreter, theArgs[3], TclList.Format(anOptions));
        }

        return aCode.ToString(CultureInfo.InvariantCulture);
    }

    private static string Try(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        if (theArgs.Count < 2)
        {
            throw Usage("try body ?handler ...? ?finally script?");
        }

        (int aCode, string aResult, string anErrorCode) = Run(theInterpreter, theArgs[1]);
        string? aFinally = null;
        int aHandlersEnd = theArgs.Count;
        if (theArgs.Count >= 4 && theArgs[^2] == "finally")
        {
            aFinally = theArgs[^1];
            aHandlersEnd -= 2;
        }

        try
        {
            for (int i = 2; i < aHandlersEnd; i += 4)
            {
                if (i + 3 >= aHandlersEnd)
                {
                    throw new DrawException("wrong # args to try handler");
                }

                bool aMatches = theArgs[i] switch
                {
                    "on" => ParseCode(theArgs[i + 1]) == aCode,
                    "trap" => aCode == 1 && TclList.Parse(anErrorCode).Take(TclList.Parse(theArgs[i + 1]).Count).SequenceEqual(TclList.Parse(theArgs[i + 1])),
                    _ => throw new DrawException($"bad handler \"{theArgs[i]}\": must be on or trap"),
                };
                if (!aMatches)
                {
                    continue;
                }

                IReadOnlyList<string> aVars = TclList.Parse(theArgs[i + 2]);
                if (aVars.Count > 0)
                {
                    SetValue(theInterpreter, aVars[0], aResult);
                }

                if (aVars.Count > 1)
                {
                    SetValue(theInterpreter, aVars[1], TclList.Format(["-code", aCode.ToString(CultureInfo.InvariantCulture), "-errorcode", anErrorCode]));
                }

                return theInterpreter.EvalScript(theArgs[i + 3]);
            }

            return Rethrow(aCode, aResult, anErrorCode);
        }
        finally
        {
            if (aFinally is not null)
            {
                theInterpreter.EvalScript(aFinally);
            }
        }
    }

    private static string Rethrow(int theCode, string theResult, string theErrorCode) => theCode switch
    {
        0 => theResult,
        1 => throw new DrawException(theResult) { ErrorCode = theErrorCode },
        2 => throw new TclReturnException(0, theResult),
        3 => throw TclBreakException.Instance,
        4 => throw TclContinueException.Instance,
        _ => theResult,
    };

    private static string Error(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        if (theArgs.Count is < 2 or > 4)
        {
            throw Usage("error message ?errorInfo? ?errorCode?");
        }

        throw new DrawException(theArgs[1]) { ErrorCode = theArgs.Count > 3 ? theArgs[3] : "NONE" };
    }

    private static string Uplevel(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        if (theArgs.Count < 2)
        {
            throw Usage("uplevel ?level? command ?arg ...?");
        }

        string aLevel = "1";
        int aFrom = 1;
        if (theArgs.Count > 2 && DrawInterpreter.IsLevel(theArgs[1]))
        {
            aLevel = theArgs[1];
            aFrom = 2;
        }

        return theInterpreter.EvalInFrame(theInterpreter.FrameAt(aLevel), Concat(theArgs, aFrom));
    }

    private static string Upvar(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        string aLevel = "1";
        int aFrom = 1;
        if (theArgs.Count % 2 == 0)
        {
            aLevel = theArgs[1];
            aFrom = 2;
        }

        if (theArgs.Count - aFrom < 2)
        {
            throw Usage("upvar ?level? otherVar localVar ?otherVar localVar ...?");
        }

        TclFrame aFrame = theInterpreter.FrameAt(aLevel);
        for (int i = aFrom; i + 1 < theArgs.Count; i += 2)
        {
            theInterpreter.LinkVariable(aFrame, theArgs[i], theArgs[i + 1]);
        }

        return "";
    }

    private static string Tail(string theName)
    {
        int aColons = theName.LastIndexOf("::", StringComparison.Ordinal);
        return aColons < 0 ? theName : theName[(aColons + 2)..];
    }

    private static string Global(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        if (theInterpreter.Level == 0)
        {
            return "";
        }

        foreach (string aName in theArgs.Skip(1))
        {
            theInterpreter.LinkVariable(theInterpreter.GlobalFrame, aName.TrimStart(':'), Tail(aName));
        }

        return "";
    }

    private static string Variable(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        // Draw-lite has only the global namespace, so namespace variables are global variables
        for (int i = 1; i < theArgs.Count; i += 2)
        {
            string aName = theArgs[i].TrimStart(':');
            if (theInterpreter.Level > 0)
            {
                theInterpreter.LinkVariable(theInterpreter.GlobalFrame, aName, Tail(aName));
            }

            if (i + 1 < theArgs.Count)
            {
                theInterpreter.SetVariable("::" + aName, null, theArgs[i + 1]);
            }
        }

        return "";
    }

    private static string Proc(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        if (theArgs.Count != 4)
        {
            throw Usage("proc name args body");
        }

        var aParams = new List<(string, string?)>();
        foreach (string aParam in TclList.Parse(theArgs[2]))
        {
            IReadOnlyList<string> aParts = TclList.Parse(aParam);
            switch (aParts.Count)
            {
                case 1:
                    aParams.Add((aParts[0], null));
                    break;
                case 2:
                    aParams.Add((aParts[0], aParts[1]));
                    break;
                case 0:
                    throw new DrawException($"argument with no name");
                default:
                    throw new DrawException($"too many fields in argument specifier \"{aParam}\"");
            }
        }

        theInterpreter.DefineProc(new TclProc(theArgs[1], aParams, theArgs[3]));
        return "";
    }

    private static string Switch(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        const string theUsage = "switch ?-option ...? string ?pattern body ...? ?default body?";
        string aMode = "-exact";
        bool aNoCase = false;
        int i = 1;
        for (; i < theArgs.Count && theArgs[i].StartsWith('-'); i++)
        {
            string anOption = theArgs[i];
            if (anOption == "--")
            {
                i++;
                break;
            }

            switch (anOption)
            {
                case "-exact" or "-glob" or "-regexp":
                    aMode = anOption;
                    break;
                case "-nocase":
                    aNoCase = true;
                    break;
                default:
                    // Not an option: the string to match
                    goto EndOptions;
            }
        }

    EndOptions:
        if (i >= theArgs.Count)
        {
            throw Usage(theUsage);
        }

        string aString = theArgs[i++];
        IReadOnlyList<string> aPairs = theArgs.Count - i == 1 ? TclList.Parse(theArgs[i]) : theArgs.Skip(i).ToList();
        if (aPairs.Count == 0)
        {
            throw Usage(theUsage);
        }

        if (aPairs.Count % 2 != 0)
        {
            throw new DrawException("extra switch pattern with no body");
        }

        for (int k = 0; k < aPairs.Count; k += 2)
        {
            string aPattern = aPairs[k];
            bool aMatch = (k == aPairs.Count - 2 && aPattern == "default") || aMode switch
            {
                "-glob" => TclGlob.Match(aPattern, aString, aNoCase),
                "-regexp" => TclRegex.Create(aPattern, aNoCase).IsMatch(aString),
                _ => aNoCase ? string.Equals(aPattern, aString, StringComparison.OrdinalIgnoreCase) : aPattern == aString,
            };
            if (!aMatch)
            {
                continue;
            }

            int aBody = k + 1;
            while (aBody < aPairs.Count && aPairs[aBody] == "-")
            {
                aBody += 2;
            }

            if (aBody >= aPairs.Count)
            {
                throw new DrawException($"no body specified for pattern \"{aPattern}\"");
            }

            return theInterpreter.EvalScript(aPairs[aBody]);
        }

        return "";
    }

    private static string Time(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        if (theArgs.Count is < 2 or > 3)
        {
            throw Usage("time command ?count?");
        }

        long aCount = theArgs.Count == 3 ? TclNumber.GetInt(theArgs[2]) : 1;
        var aWatch = Stopwatch.StartNew();
        for (long i = 0; i < aCount; i++)
        {
            theInterpreter.EvalScript(theArgs[1]);
        }

        // Tcl_TimeObjCmd: one iteration (or none) gives an integer ("time is not fractional", Tcl bug 1202178), more
        // the mean as a double. OCCT's tests divide it by 10^6 with expr, so the type matters (bugs/mesh/bug25503_1).
        if (aCount <= 1)
        {
            long aWhole = aCount <= 0 ? 0 : (long)aWatch.Elapsed.TotalMicroseconds;
            return $"{aWhole} microseconds per iteration";
        }

        double aMicroseconds = aWatch.Elapsed.TotalMicroseconds / aCount;
        return $"{TclNumber.FormatDouble(Math.Round(aMicroseconds, 3))} microseconds per iteration";
    }

    private static string Namespace(DrawInterpreter theInterpreter, IReadOnlyList<string> theArgs)
    {
        // Draw-lite has only the global namespace
        if (theArgs.Count < 2)
        {
            throw Usage("namespace subcommand ?arg ...?");
        }

        switch (Resolve(theArgs[1], "subcommand", ["eval", "current", "tail", "qualifiers", "exists", "which", "export", "import", "forget", "ensemble", "path", "unknown", "children", "code", "delete", "origin", "parent", "upvar"]))
        {
            case "eval" when theArgs.Count >= 4:
                return theInterpreter.EvalScript(Concat(theArgs, 3));
            case "current":
                return "::";
            case "tail" when theArgs.Count == 3:
                return Tail(theArgs[2]);
            case "qualifiers" when theArgs.Count == 3:
            {
                int aColons = theArgs[2].LastIndexOf("::", StringComparison.Ordinal);
                return aColons < 0 ? "" : theArgs[2][..aColons];
            }

            case "exists" when theArgs.Count == 3:
                return theArgs[2] is "::" or "" ? "1" : "0";
            case "which":
            {
                string aName = theArgs[^1];
                if (theArgs.Count > 3 && theArgs[2] == "-variable")
                {
                    return theInterpreter.TryGetVariable(aName, out _) ? "::" + aName.TrimStart(':') : "";
                }

                return theInterpreter.HasCommand(aName) ? "::" + aName.TrimStart(':') : "";
            }

            case "export" or "import" or "forget" or "ensemble" or "path" or "unknown":
                return "";
            default:
                throw new DrawException($"namespace {theArgs[1]}: not supported by Draw-lite");
        }
    }
}
