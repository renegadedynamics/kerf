// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using System.Text.RegularExpressions;

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
    /// Commands that procedures of OCCT's DrawResources, or of a grid's begin script, call through a variable
    /// (eval ${CommandName} ...), so a scan of their bodies can't see them.
    /// </summary>
    public static IReadOnlyDictionary<string, string[]> IndirectCalls { get; } = new Dictionary<string, string[]>(StringComparer.Ordinal)
    {
        ["checkprops"] = ["lprops", "sprops", "vprops"],
        ["checklength"] = ["length"],

        // procedures of a grid's begin script: chamfer/begin's compute_chamf runs `catch "$chamf_str"`
        ["compute_chamf"] = ["chamf"],
    };

    /// <summary>
    /// Commands a group's scripts call only in a branch on the case's data file (<c>TheFileName</c>), and the data
    /// files that don't take that branch: the mesh group's end script reads a .brep or .rle file with brestore and
    /// any other (STEP) file with <c>stepread</c> and <c>tpcompound</c>. A case whose data file matches the pattern,
    /// or that sets none, doesn't need the commands.
    /// </summary>
    public static IReadOnlyList<(string Group, string[] Commands, Regex SkippedFor)> DataFileBranches { get; } =
    [
        ("mesh", ["stepread", "tpcompound"], new Regex(@"\.(brep|rle)", RegexOptions.CultureInvariant)),
    ];

    /// <summary>
    /// Commands a group's scripts call only when the case sets a variable (unit 5R): the fillet2d group's end script
    /// runs <c>CheckTriArea</c>, which nothing in that group defines, only <c>if {[info exists rel_tol]}</c>. A case
    /// that never sets the variable doesn't need the commands.
    /// </summary>
    public static IReadOnlyList<(string Group, string[] Commands, string Variable)> VariableBranches { get; } =
    [
        ("fillet2d", ["CheckTriArea"], "rel_tol"),
    ];

    /// <summary>
    /// Commands a group's scripts call only for some values of a variable the grid or the case sets (unit 5O2): the
    /// offset group's end script checks a <c>mkoffset</c> result in one branch (<c>if {$command == "mkoffset"}</c>), a
    /// <c>withintersect</c> one in another, and the solid offset of every other grid in the last, which its begin
    /// script's <c>OFFSETSHAPE</c> procedure builds. Each entry: the commands called only where the variable has one of
    /// the values (null: where it is unset or has another value than those of the group's other entries). A case
    /// whose own script calls a command still needs it.
    /// </summary>
    public static IReadOnlyList<(string Group, string Variable, string?[] Values, string[] Commands)> ValueBranches { get; } =
    [
        ("offset", "command", ["mkoffset"], ["mkoffset", "renamevar", "tolerance", "checksection", "mkplane", "whatis"]),
        ("offset", "command", ["withintersect", null], ["offsetparameter", "offsetload", "offsetperform", "vprops", "sprops", "bsection"]),
        // unit 8G1: the xcaf group's end script writes and reads the document back by the case's TypeOfFile (OCAF
        // storage for "", IGES, STEP; BRep files otherwise), and rescales it only in the mode RESCALE_ASSEMBLY
        ("xcaf", "TypeOfFile", [""], ["SaveAs", "Open"]),
        ("xcaf", "TypeOfFile", ["igs"], ["WriteIges", "ReadIges"]),
        ("xcaf", "TypeOfFile", ["stp"], ["WriteStep", "ReadStep"]),
        ("xcaf", "CompareDocumentsMode", ["RESCALE_ASSEMBLY"], ["XRescaleGeometry"]),
    ];

    /// <summary>Whether the scan knows the branches of <see cref="ValueBranches"/> (unit 5O2).</summary>
    public bool KnowsValueBranches { get; init; } = true;

    private static readonly Regex s_theFileName = new(@"^\s*set\s+TheFileName\s+(\S+)", RegexOptions.Multiline | RegexOptions.CultureInvariant);

    private readonly ConcurrentDictionary<string, IReadOnlySet<string>> myProcCalls = new(StringComparer.Ordinal);

    /// <summary>
    /// Whether the scan knows the branches of <see cref="DataFileBranches"/> (unit 4L2); false counts the commands of
    /// every branch, as before.
    /// </summary>
    public bool KnowsDataFileBranches { get; init; } = true;

    public string TestsDirectory { get; } = theTestsDirectory;

    /// <summary>
    /// Whether grids whose cases live in a shared directory named by their <c>cases.list</c> (mesh, heal's split and
    /// surface-to-revolution grids, chamfer, opengl, xml) are read, as OCCT's harness reads them. Before unit 4L1 the
    /// scan skipped them; false gives that former set of cases.
    /// </summary>
    public bool ReadCaseLists { get; init; } = true;

    /// <summary>
    /// Whether a procedure the case's scripts define counts its body's commands only when something calls it (unit 5W).
    /// The bugs group's modalg grids define <c>OFFSETSHAPE</c> in their begin script, which calls phase 7's offset
    /// commands, and only a few of their cases call it. The default since phase 7's plan (open question 29); false counts
    /// every defined procedure's body, as before.
    /// </summary>
    public bool FollowsCalledProcsOnly { get; init; } = true;

    /// <summary>
    /// The directory a grid's cases are in: the grid's own, or the one the first line of its <c>cases.list</c> names
    /// relative to it (TestCommands.tcl's <c>test</c>: "check if this grid is aliased to another one"); null if the
    /// grid has a cases.list and <see cref="ReadCaseLists"/> is off.
    /// </summary>
    public string? CaseDirectory(string theGridDirectory)
    {
        string aList = Path.Combine(theGridDirectory, "cases.list");
        if (!File.Exists(aList))
        {
            return theGridDirectory;
        }

        if (!ReadCaseLists)
        {
            return null;
        }

        string? aFirst = File.ReadLines(aList).FirstOrDefault();
        return aFirst is null ? theGridDirectory : Path.GetFullPath(Path.Combine(theGridDirectory, aFirst.Trim()));
    }

    /// <summary>Whether a case is one of a grid that has a cases.list.</summary>
    public bool IsListed(DrawTestCase theCase)
        => File.Exists(Path.Combine(TestsDirectory, theCase.Group, theCase.Grid, "cases.list"));

    public IEnumerable<DrawTestCase> Cases()
    {
        foreach (string aGroup in Directory.EnumerateDirectories(TestsDirectory).Order(StringComparer.Ordinal))
        {
            foreach (string aGrid in Directory.EnumerateDirectories(aGroup).Order(StringComparer.Ordinal))
            {
                if (CaseDirectory(aGrid) is not { } aCaseDirectory || !Directory.Exists(aCaseDirectory))
                {
                    continue;
                }

                foreach (string aCase in Directory.EnumerateFiles(aCaseDirectory).Order(StringComparer.Ordinal))
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
        // _run_test logs the commands and their results (dlog on)
        anInterpreter.DoLog = true;
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
        finally
        {
            // OCCT's harness runs each case in a fresh DRAWEXE process; the mesher's process-wide defaults (mpparallel,
            // mpsetdefaultname) don't carry over to the next case
            Occt.BRepMesh_IncrementalMesh.SetParallelDefault(false);
            Occt.BRepMesh_DiscretFactory.Get().SetDefaultName("FastDiscret");
            // nor ShapeProcess's cached resource file (unit 7C; its operator registry is the same in every DRAWEXE)
            Occt.ShapeProcess_Context.ClearResourceCache();
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
        Dictionary<string, string>? aBodies = FollowsCalledProcsOnly ? new(StringComparer.Ordinal) : null;
        try
        {
            Collect(Script(theCase), aCommands, aDefined, theDepth: 0, aBodies);
            // the case's own procedures, once called (FollowsCalledProcsOnly)
            var aCalled = new Stack<string>(aCommands);
            var aDone = new HashSet<string>(StringComparer.Ordinal);
            while (aBodies is not null && aCalled.TryPop(out string? aCall))
            {
                if (!aDone.Add(aCall) || !aBodies.TryGetValue(aCall, out string? aBody))
                {
                    continue;
                }

                var aBodyCommands = new HashSet<string>(StringComparer.Ordinal);
                Collect(aBody, aBodyCommands, aDefined, theDepth: 1, aBodies);
                foreach (string aNew in aBodyCommands.Where(aCommands.Add))
                {
                    aCalled.Push(aNew);
                }
            }
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
        if (KnowsDataFileBranches)
        {
            foreach ((string aGroup, string[] aBranchCommands, Regex aSkippedFor) in DataFileBranches)
            {
                if (aGroup != theCase.Group)
                {
                    continue;
                }

                // the case's data file, as the case sets it (the last set wins, as when it runs)
                Match? aSet = s_theFileName.Matches(File.ReadAllText(theCase.Path)).LastOrDefault();
                if (aSet is null || aSet.Groups[1].Value is "\"\"" or "{}" || aSkippedFor.IsMatch(aSet.Groups[1].Value))
                {
                    aMissing.ExceptWith(aBranchCommands);
                }
            }
        }

        foreach ((string aGroup, string[] aBranchCommands, string aVariable) in VariableBranches)
        {
            if (aGroup == theCase.Group
                && !Regex.IsMatch(File.ReadAllText(theCase.Path), @"\bset\s+" + Regex.Escape(aVariable) + @"\b", RegexOptions.CultureInvariant))
            {
                aMissing.ExceptWith(aBranchCommands);
            }
        }

        if (KnowsValueBranches)
        {
            foreach (IGrouping<(string Group, string Variable), (string Group, string Variable, string?[] Values, string[] Commands)> aBranches
                     in ValueBranches.Where(b => b.Group == theCase.Group).GroupBy(b => (b.Group, b.Variable)))
            {
                // the value the grid's begin script or the case sets (the last set wins)
                string aGridBegin = ReadIfExists(Path.Combine(TestsDirectory, theCase.Group, theCase.Grid, "begin"));
                string aCaseText = File.ReadAllText(theCase.Path);
                // a quoted value whole (unit 8G1: the xcaf group's "SHAPES VOLUME"), else the word
                var aSetRegex = new Regex(@"^\s*set\s+" + Regex.Escape(aBranches.Key.Variable) + @"\s+(?:""([^""]*)""|(\S+))", RegexOptions.Multiline | RegexOptions.CultureInvariant);
                string aText = aGridBegin + "\n" + aCaseText;
                Match? aSet = aSetRegex.Matches(aText).LastOrDefault();
                string? aValue = aSet is null ? null : aSet.Groups[1].Success ? aSet.Groups[1].Value : aSet.Groups[2].Value;
                bool isKnown = aBranches.Any(b => b.Values.Contains(aValue));
                IReadOnlySet<string> anOwn = CommandsCalledBy(aText);
                foreach ((string _, string _, string?[] aValues, string[] aBranchCommands) in aBranches)
                {
                    bool isTaken = aValues.Contains(aValue) || (!isKnown && aValues.Contains(null));
                    if (!isTaken)
                    {
                        aMissing.ExceptWith(aBranchCommands.Where(c => !anOwn.Contains(c)));
                    }
                }
            }
        }

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

    private static void Collect(string theScript, HashSet<string> theCommands, HashSet<string> theDefined, int theDepth,
                                Dictionary<string, string>? theProcBodies = null)
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
                    if (IndirectCalls.TryGetValue(aProc.TrimStart(':'), out string[]? anIndirect))
                    {
                        theCommands.UnionWith(anIndirect);
                    }
                }
            }

            for (int i = 0; i < aWords.Count; i++)
            {
                Word aWord = aWords[i];

                // Bracketed parts always run
                foreach (WordPart aPart in aWord.Parts)
                {
                    CollectSubstitutions(aPart, theCommands, theDefined, theDepth, theProcBodies);
                }

                if (i == 0 || aWord.Literal is not { } aText)
                {
                    continue;
                }

                // a procedure's body, kept until something calls it (FollowsCalledProcsOnly)
                if (theProcBodies is not null && aName == "proc" && i == 3 && aWords[1].Literal is { } aProcName)
                {
                    theProcBodies[aProcName.TrimStart(':')] = aText;
                    continue;
                }

                switch (Role(aName, aWords, i))
                {
                    case ArgumentRole.Script:
                        Collect(aText, theCommands, theDefined, theDepth + 1, theProcBodies);
                        break;
                    case ArgumentRole.Expression:
                        foreach (WordPart aPart in TclParser.ParseSubst(aText, theBackslashes: false))
                        {
                            CollectSubstitutions(aPart, theCommands, theDefined, theDepth, theProcBodies);
                        }

                        break;
                    case ArgumentRole.SwitchBody:
                    {
                        IReadOnlyList<string> aPairs = TclList.IsList(aText) ? TclList.Parse(aText) : [];
                        for (int k = 1; k < aPairs.Count; k += 2)
                        {
                            Collect(aPairs[k], theCommands, theDefined, theDepth + 1, theProcBodies);
                        }

                        break;
                    }

                    case ArgumentRole.Unknown when aWord.IsBraced && aText.Contains('\n'):
                        // A braced multi-line argument of an unknown command is most likely a script
                        Collect(aText, theCommands, theDefined, theDepth + 1, theProcBodies);
                        break;
                }
            }
        }
    }

    private static void CollectSubstitutions(WordPart thePart, HashSet<string> theCommands, HashSet<string> theDefined, int theDepth,
                                             Dictionary<string, string>? theProcBodies = null)
    {
        switch (thePart)
        {
            case WordPart.Command aBracketed:
                Collect(aBracketed.Script, theCommands, theDefined, theDepth + 1, theProcBodies);
                break;
            case WordPart.Variable { Index: { } anIndex }:
                foreach (WordPart anIndexPart in anIndex)
                {
                    CollectSubstitutions(anIndexPart, theCommands, theDefined, theDepth, theProcBodies);
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

            // StandardCommands.tcl's procedures that run a script (unit 7W): `repeat n script`, `do var start end ?incr? body`;
            // their scripts are often one line, which the guess for unknown commands (a braced multi-line word) misses
            case "repeat":
                return theIndex == 2 ? ArgumentRole.Script : ArgumentRole.Data;
            case "do":
                return theIndex == aCount - 1 && theIndex >= 4 ? ArgumentRole.Script : ArgumentRole.Data;
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
