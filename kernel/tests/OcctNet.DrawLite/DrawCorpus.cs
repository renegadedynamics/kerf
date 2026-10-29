// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;

namespace OcctNet.DrawLite;

/// <summary>One test case of OCCT's corpus: tests/&lt;group&gt;/&lt;grid&gt;/&lt;case&gt;.</summary>
public sealed record DrawTestCase(string Group, string Grid, string Name, string Path)
{
    public string Id => $"{Group}/{Grid}/{Name}";
}

/// <summary>What a test case needs: the commands it calls that the interpreter doesn't have yet.</summary>
public sealed record DrawCaseScan(DrawTestCase Case, IReadOnlySet<string> Commands, IReadOnlySet<string> Missing)
{
    public bool IsRunnable => Missing.Count == 0;
}

/// <summary>A case run, with the status OCCT's _check_log gives its log (OK, BAD, IMPROVEMENT, SKIPPED, FAILED ...).</summary>
public sealed record DrawCaseRun(DrawTestCase Case, string Status, string Log)
{
    /// <summary>OK, or BAD (known problem): OCCT's harness counts both as passing.</summary>
    public bool Passed => Status.StartsWith("OK", StringComparison.Ordinal) || Status.StartsWith("BAD", StringComparison.Ordinal);
}

/// <summary>
/// OCCT's Draw test corpus (tests/ in the OCCT source). A case runs as group/begin, grid/begin, the case,
/// grid/end, group/end, which is how OCCT's own test harness assembles it.
/// </summary>
public sealed class DrawCorpus(string theTestsDirectory)
{
    /// <summary>
    /// Commands that procedures of OCCT's DrawResources call through a variable (eval ${CommandName} ...), so a
    /// scan of their bodies can't see them.
    /// </summary>
    public static IReadOnlyDictionary<string, string[]> IndirectCalls { get; } = new Dictionary<string, string[]>(StringComparer.Ordinal)
    {
        ["checkprops"] = ["lprops", "sprops", "vprops"],
        ["checklength"] = ["length"],
    };

    private readonly ConcurrentDictionary<string, IReadOnlySet<string>> myProcCalls = new(StringComparer.Ordinal);

    public string TestsDirectory { get; } = theTestsDirectory;

    public IEnumerable<DrawTestCase> Cases()
    {
        foreach (string aGroup in Directory.EnumerateDirectories(TestsDirectory).Order(StringComparer.Ordinal))
        {
            foreach (string aGrid in Directory.EnumerateDirectories(aGroup).Order(StringComparer.Ordinal))
            {
                foreach (string aCase in Directory.EnumerateFiles(aGrid).Order(StringComparer.Ordinal))
                {
                    string aName = Path.GetFileName(aCase);
                    if (aName is "begin" or "end" or "parse.rules" or "cases.list" || aName.Contains('.'))
                    {
                        continue;
                    }

                    yield return new DrawTestCase(Path.GetFileName(aGroup), Path.GetFileName(aGrid), aName, aCase);
                }
            }
        }
    }

    /// <summary>The whole script of a case, with its group's and grid's begin and end scripts around it.</summary>
    public string Script(DrawTestCase theCase)
    {
        string aGroupDir = Path.Combine(TestsDirectory, theCase.Group);
        string aGridDir = Path.Combine(aGroupDir, theCase.Grid);
        return string.Join('\n',
                           ReadIfExists(Path.Combine(aGroupDir, "begin")),
                           ReadIfExists(Path.Combine(aGridDir, "begin")),
                           File.ReadAllText(theCase.Path),
                           ReadIfExists(Path.Combine(aGridDir, "end")),
                           ReadIfExists(Path.Combine(aGroupDir, "end")));
    }

    /// <summary>
    /// Runs a case as OCCT's harness (_run_test in TestCommands.tcl) does: it sets casename, groupname, gridname,
    /// dirname, imagedir and test_image, then sources group/begin, grid/begin, the case, grid/end and group/end.
    /// An error stops the run and is thrown. The interpreter should have loaded the DrawResources.
    /// </summary>
    public void Run(DrawTestCase theCase, DrawInterpreter theInterpreter, string? theImageDirectory = null)
    {
        string aTests = TclFileSystem.Slashes(TestsDirectory);
        string aGroupDir = $"{aTests}/{theCase.Group}";
        string aGridDir = $"{aGroupDir}/{theCase.Grid}";
        var aScript = new System.Text.StringBuilder();
        aScript.Append($"set casename {TclList.Quote(theCase.Name)}\n")
               .Append($"set groupname {TclList.Quote(theCase.Group)}\n")
               .Append($"set gridname {TclList.Quote(theCase.Grid)}\n")
               .Append($"set dirname {TclList.Quote(aTests)}\n")
               .Append($"set imagedir {TclList.Quote(theImageDirectory ?? TclFileSystem.Slashes(Path.GetTempPath()).TrimEnd('/'))}\n")
               .Append($"set test_image {TclList.Quote(theCase.Name)}\n");
        foreach (string aFile in new[] { $"{aGroupDir}/begin", $"{aGridDir}/begin", TclFileSystem.Slashes(theCase.Path), $"{aGridDir}/end", $"{aGroupDir}/end" })
        {
            if (File.Exists(aFile))
            {
                aScript.Append($"source -encoding utf-8 {TclList.Quote(aFile)}\n");
            }
        }

        theInterpreter.Eval(aScript.ToString());
    }

    /// <summary>
    /// Runs a case in a new interpreter with the DrawResources loaded, as OCCT's harness does (an error ends the
    /// script with "Tcl Exception: ..." in the log), then judges its log with the harness's own <c>_check_log</c>
    /// (TestCommands.tcl: parse.rules, TODO and REQUIRED statements, "TEST COMPLETED"). The status is _check_log's:
    /// OK, BAD (known problem), IMPROVEMENT, SKIPPED or FAILED, with its reason.
    /// </summary>
    /// <param name="theCase">The case.</param>
    /// <param name="theResourcesDirectory">OCCT's resources/DrawResources.</param>
    /// <param name="theDataPath">CSF_TestDataPath: directories of data files, separated by ';'.</param>
    /// <param name="theTimeLimit">The time after which the case stops ("Process killed by elapsed limit").</param>
    /// <param name="theSetup">Runs on the interpreter before the case (to remove commands, for one).</param>
    public DrawCaseRun RunAndCheck(DrawTestCase theCase, string theResourcesDirectory, string? theDataPath, TimeSpan theTimeLimit,
                                   Action<DrawInterpreter>? theSetup = null)
    {
        var aLog = new StringWriter { NewLine = "\n" };
        var anInterpreter = new DrawInterpreter(aLog);
        anInterpreter.LoadDrawResources(theResourcesDirectory);
        if (theDataPath is not null)
        {
            anInterpreter.SetVariable("env(CSF_TestDataPath)", theDataPath);
        }

        // the environment DRAWEXE has: OCCT's root (the parent of tests/), its resources and data
        string aRoot = TclFileSystem.Slashes(Path.GetDirectoryName(Path.GetFullPath(TestsDirectory).TrimEnd('\\', '/'))!);
        anInterpreter.SetVariable("env(CASROOT)", aRoot);
        anInterpreter.SetVariable("env(CSF_OCCTResourcePath)", aRoot + "/resources");
        anInterpreter.SetVariable("env(CSF_OCCTDataPath)", aRoot + "/data");

        // an empty, writable image directory, as the harness makes for each case: in the overlay, as every write
        string anImages = TclFileSystem.Slashes(Path.Combine(Path.GetTempPath(), "drawlite-images", theCase.Group, theCase.Grid, theCase.Name));
        anInterpreter.Files.MakeDirectory(anImages);
        theSetup?.Invoke(anInterpreter);
        anInterpreter.Deadline = DateTime.UtcNow + theTimeLimit;
        try
        {
            Run(theCase, anInterpreter, anImages);
        }
        catch (DrawException anError)
        {
            aLog.WriteLine("Tcl Exception: " + anError.Reason);
        }
        catch (DrawTimeoutException)
        {
            aLog.WriteLine("Process killed by elapsed limit");
        }
        catch (Exception anError) when (anError is not OutOfMemoryException)
        {
            // an exception from the port that a command didn't turn into a Tcl error
            aLog.WriteLine("Tcl Exception: " + anError.GetType().Name + ": " + anError.Message);
        }

        string aText = aLog.ToString();
        var aChecker = new DrawInterpreter();
        aChecker.LoadDrawResources(theResourcesDirectory);
        aChecker.SetVariable("log", aText);
        string aStatus;
        try
        {
            aChecker.Eval($"_check_log {TclList.Quote(TclFileSystem.Slashes(TestsDirectory))} {TclList.Quote(theCase.Group)} "
                          + $"{TclList.Quote(theCase.Grid)} {TclList.Quote(theCase.Name)} 0 $log summary html_log");
            string aSummary = aChecker.Eval("set summary");
            int aColon = aSummary.IndexOf(": ", StringComparison.Ordinal);
            aStatus = aColon >= 0 ? aSummary[(aColon + 2)..].Trim() : aSummary;
        }
        catch (DrawException anError)
        {
            aStatus = "FAILED (_check_log: " + anError.Reason + ")";
        }

        return new DrawCaseRun(theCase, aStatus, aText);
    }

    /// <summary>
    /// The commands a case calls, found without running it, and those of them the interpreter lacks. Commands
    /// are the first words of commands, including those in script arguments (if, foreach, proc, catch, ...) and
    /// brackets, also inside expressions. A call to a procedure (defined by the case's begin files or by the
    /// DrawResources the interpreter loaded) counts the commands its body calls, so the missing commands are the
    /// primitives still to port. Words that are themselves substituted are skipped.
    /// </summary>
    public DrawCaseScan Scan(DrawTestCase theCase, DrawInterpreter theInterpreter)
    {
        var aCommands = new HashSet<string>(StringComparer.Ordinal);
        var aDefined = new HashSet<string>(StringComparer.Ordinal);
        try
        {
            Collect(Script(theCase), aCommands, aDefined, theDepth: 0);
        }
        catch (DrawException)
        {
            aCommands.Add("<unparsable>");
        }

        // Follow calls into the interpreter's procedures (the case's own procedures were collected in place)
        var aPending = new Stack<string>(aCommands.Where(c => !aDefined.Contains(c)));
        var aVisited = new HashSet<string>(StringComparer.Ordinal);
        while (aPending.TryPop(out string? aName))
        {
            if (!aVisited.Add(aName) || ProcCalls(theInterpreter, aName) is not { } aCalls)
            {
                continue;
            }

            foreach (string aCall in aCalls)
            {
                aCommands.Add(aCall);
                aPending.Push(aCall);
            }
        }

        var aMissing = aCommands.Where(c => !theInterpreter.HasCommand(c) && !aDefined.Contains(c)).ToHashSet(StringComparer.Ordinal);
        return new DrawCaseScan(theCase, aCommands, aMissing);
    }

    /// <summary>The commands a procedure of the interpreter calls, or null when the name isn't a procedure.</summary>
    public IReadOnlySet<string>? ProcCalls(DrawInterpreter theInterpreter, string theName)
    {
        string? aBody = theInterpreter.ProcBody(theName);
        if (aBody is null)
        {
            return null;
        }

        return myProcCalls.GetOrAdd(theName + "\0" + aBody, _ =>
        {
            var aCalls = new HashSet<string>(StringComparer.Ordinal);
            try
            {
                Collect(aBody, aCalls, new HashSet<string>(StringComparer.Ordinal), theDepth: 0);
            }
            catch (DrawException)
            {
            }

            if (IndirectCalls.TryGetValue(theName, out string[]? anIndirect))
            {
                aCalls.UnionWith(anIndirect);
            }

            return aCalls;
        });
    }

    /// <summary>The commands a script calls (see <see cref="Scan"/>), without following procedures.</summary>
    public static IReadOnlySet<string> CommandsCalledBy(string theScript)
    {
        var aCommands = new HashSet<string>(StringComparer.Ordinal);
        Collect(theScript, aCommands, new HashSet<string>(StringComparer.Ordinal), theDepth: 0);
        return aCommands;
    }

    // Commands whose arguments are data, never scripts: their braced words are not scanned
    private static readonly HashSet<string> s_dataCommands = new(StringComparer.Ordinal)
    {
        "set", "puts", "list", "lappend", "append", "regexp", "regsub", "help", "string", "format", "lsearch", "lsort",
        "lindex", "llength", "lrange", "return", "error", "dict", "array", "split", "join", "concat", "scan", "subst",
        "dputs", "lassign", "linsert", "lreplace", "global", "upvar", "variable", "unset", "info", "file", "source",
    };

    private static void Collect(string theScript, HashSet<string> theCommands, HashSet<string> theDefined, int theDepth)
    {
        if (theDepth > 20)
        {
            return;
        }

        IReadOnlyList<Command> aScript;
        try
        {
            aScript = TclParser.Parse(theScript);
        }
        catch (DrawException) when (theDepth > 0)
        {
            // A braced body that doesn't parse only fails if it runs (OCCT has a few, in branches never taken)
            return;
        }

        foreach (Command aCommand in aScript)
        {
            IReadOnlyList<Word> aWords = aCommand.Words;
            string? aName = aWords[0].Literal;
            if (aName is not null)
            {
                theCommands.Add(aName.StartsWith("::", StringComparison.Ordinal) ? aName.TrimStart(':') : aName);
                if (aName == "proc" && aWords.Count > 1 && aWords[1].Literal is { } aProc)
                {
                    theDefined.Add(aProc.TrimStart(':'));
                }
            }

            for (int i = 0; i < aWords.Count; i++)
            {
                Word aWord = aWords[i];

                // Bracketed parts always run
                foreach (WordPart aPart in aWord.Parts)
                {
                    CollectSubstitutions(aPart, theCommands, theDefined, theDepth);
                }

                if (i == 0 || aWord.Literal is not { } aText)
                {
                    continue;
                }

                switch (Role(aName, aWords, i))
                {
                    case ArgumentRole.Script:
                        Collect(aText, theCommands, theDefined, theDepth + 1);
                        break;
                    case ArgumentRole.Expression:
                        foreach (WordPart aPart in TclParser.ParseSubst(aText, theBackslashes: false))
                        {
                            CollectSubstitutions(aPart, theCommands, theDefined, theDepth);
                        }

                        break;
                    case ArgumentRole.SwitchBody:
                    {
                        IReadOnlyList<string> aPairs = TclList.IsList(aText) ? TclList.Parse(aText) : [];
                        for (int k = 1; k < aPairs.Count; k += 2)
                        {
                            Collect(aPairs[k], theCommands, theDefined, theDepth + 1);
                        }

                        break;
                    }

                    case ArgumentRole.Unknown when aWord.IsBraced && aText.Contains('\n'):
                        // A braced multi-line argument of an unknown command is most likely a script
                        Collect(aText, theCommands, theDefined, theDepth + 1);
                        break;
                }
            }
        }
    }

    private static void CollectSubstitutions(WordPart thePart, HashSet<string> theCommands, HashSet<string> theDefined, int theDepth)
    {
        switch (thePart)
        {
            case WordPart.Command aBracketed:
                Collect(aBracketed.Script, theCommands, theDefined, theDepth + 1);
                break;
            case WordPart.Variable { Index: { } anIndex }:
                foreach (WordPart anIndexPart in anIndex)
                {
                    CollectSubstitutions(anIndexPart, theCommands, theDefined, theDepth);
                }

                break;
        }
    }

    private enum ArgumentRole
    {
        Unknown,
        Data,
        Script,
        Expression,
        SwitchBody,
    }

    /// <summary>What the argument at <paramref name="theIndex"/> of a command is: a script, an expression, or data.</summary>
    private static ArgumentRole Role(string? theCommand, IReadOnlyList<Word> theWords, int theIndex)
    {
        int aCount = theWords.Count;
        string? Literal(int k) => k < aCount ? theWords[k].Literal : null;
        switch (theCommand)
        {
            case "if":
            {
                // if expr ?then? body ?elseif expr ?then? body? ... ?else? ?body?
                string? aWord = Literal(theIndex);
                if (aWord is "then" or "else" or "elseif")
                {
                    return ArgumentRole.Data;
                }

                return theIndex == 1 || Literal(theIndex - 1) == "elseif" ? ArgumentRole.Expression : ArgumentRole.Script;
            }

            case "while":
                return theIndex == 1 ? ArgumentRole.Expression : ArgumentRole.Script;
            case "for":
                return theIndex == 2 ? ArgumentRole.Expression : ArgumentRole.Script;
            case "foreach" or "lmap":
                return theIndex == aCount - 1 ? ArgumentRole.Script : ArgumentRole.Data;
            case "proc":
                return theIndex == 3 ? ArgumentRole.Script : ArgumentRole.Data;
            case "catch" or "time":
                return theIndex == 1 ? ArgumentRole.Script : ArgumentRole.Data;
            case "eval":
                // The words are concatenated into one script; the first is (usually) its command
                return theIndex == 1 ? ArgumentRole.Script : ArgumentRole.Data;
            case "uplevel":
            {
                int aScript = aCount > 2 && Literal(1) is { } aLevel && DrawInterpreter.IsLevel(aLevel) ? 2 : 1;
                return theIndex == aScript ? ArgumentRole.Script : ArgumentRole.Data;
            }

            case "try":
                return theIndex == 1 || Literal(theIndex - 1) == "finally" || (theIndex >= 4 && Literal(theIndex - 3) is "on" or "trap")
                           ? ArgumentRole.Script
                           : ArgumentRole.Data;
            case "expr":
                return ArgumentRole.Expression;
            case "switch":
                return theIndex == aCount - 1 && theWords[theIndex].IsBraced && theIndex >= 2 ? ArgumentRole.SwitchBody : ArgumentRole.Data;
            case "namespace":
                return Literal(1) == "eval" && theIndex >= 3 ? ArgumentRole.Script : ArgumentRole.Data;
            case "after":
                return theIndex >= 2 ? ArgumentRole.Script : ArgumentRole.Data;
            case "dict":
                return Literal(1) == "for" && theIndex == 4 ? ArgumentRole.Script : ArgumentRole.Data;
            case not null when s_dataCommands.Contains(theCommand):
                return ArgumentRole.Data;
            default:
                return ArgumentRole.Unknown;
        }
    }

    private static string ReadIfExists(string thePath) => File.Exists(thePath) ? File.ReadAllText(thePath) : "";
}
