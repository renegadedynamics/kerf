// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>Scans OCCT's own test corpus to show how much of it the port could run, and what to implement next.</summary>
public class DrawCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    [Fact]
    public void Every_case_parses_and_the_report_lists_missing_commands()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");

        var aCorpus = new DrawCorpus(TestsDirectory);
        var anInterpreter = new DrawInterpreter();
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        var aScans = aCorpus.Cases().AsParallel().AsOrdered().Select(c => aCorpus.Scan(c, anInterpreter)).ToList();

        var aMissing = aScans.SelectMany(s => s.Missing)
                             .GroupBy(c => c)
                             .Select(g => (Command: g.Key, Cases: g.Count()))
                             .OrderByDescending(x => x.Cases)
                             .ThenBy(x => x.Command, StringComparer.Ordinal)
                             .ToList();

        output.WriteLine($"{aScans.Count} cases, {aScans.Count(s => s.IsRunnable)} runnable with the commands implemented so far");
        output.WriteLine($"{aScans.Count(s => s.Missing.Count == 1)} cases miss exactly one command; {aMissing.Count} distinct commands are missing");
        output.WriteLine("Most-needed commands (cases that use each):");
        foreach (var (aCommand, aCases) in aMissing.Take(60))
        {
            output.WriteLine($"  {aCommand,-24} {aCases}");
        }

        Assert.True(aScans.Count > 10000, "OCCT V8_0_1 has over 10,000 test cases");
        var anUnparsable = aScans.Where(s => s.Commands.Contains("<unparsable>")).Select(s => s.Case.Id).ToList();
        output.WriteLine($"{anUnparsable.Count} unparsable: {string.Join(", ", anUnparsable.Take(10))}");
        Assert.True(anUnparsable.Count < aScans.Count / 100, "the parser should read at least 99% of the corpus");

        // Tcl's own commands are all implemented, so what is missing is Draw's (geometry and viewer) commands
        string[] aTclCore = ["if", "expr", "proc", "lindex", "uplevel", "upvar", "string", "regexp", "foreach", "dict", "format", "catch", "global", "open"];
        Assert.DoesNotContain(aMissing, m => aTclCore.Contains(m.Command));
    }

    /// <summary>The Draw commands unit 3L1 added (dset and protect were no-ops before it).</summary>
    internal static readonly string[] Unit3L1Commands =
    [
        "bsave", "brestore", "isdraw", "isprot", "dtyp", "dval", "dname", "dump", "copy", "renamevar", "unprotect",
        "autodisplay", "cos", "sin", "tan", "acos", "asin", "atan2", "sqrt", "isos", "hlr", "vori", "vconn", "triangles",
        "polygons", "discretisation", "compound", "add", "explode", "nexplode", "exwire", "emptycopy", "orientation",
        "treverse", "complement", "invert", "nbshapes", "numshapes", "countshapes", "setflags", "purgemmgt",
        "writebrep", "binsave", "readbrep", "binrestore", "reset", "tmove", "ttranslate", "trotate", "tmirror",
        "tscale", "bmove", "fmove", "btranslate", "brotate", "bmirror", "fmirror", "bscale", "fscale", "tcopy",
        "maxtolerance", "purgeloc", "checkloc", "lprops", "sprops", "vprops", "vpropsgk", "tolerance",
    ];

    private static readonly System.Text.RegularExpressions.Regex s_failure =
        new(@"\b(?:[Ee]xception|[Ee][Rr][Rr][Oo][Rr]|[Ff][Aa][Ii][Ll](?:[0-9]|[Ee][Dd]|[Uu][Rr][Ee]))\b|could not be found");

    internal static string DataPath
        => string.Join(';', Path.Combine(DrawResourcesTests.OcctSourceDirectory, "data"), DrawResourcesTests.OcctTestDataDirectory)
                 .Replace('\\', '/');

    /// <summary>
    /// Runs every case whose commands are all implemented, as OCCT's harness does, and judges each log with OCCT's
    /// own _check_log; reports the cases runnable before and after unit 3L1, the statuses, and what blocks the rest.
    /// </summary>
    [Fact]
    public void Runnable_cases_run_and_OCCTs_log_check_judges_them()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        var anInterpreter = new DrawInterpreter();
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        var aBefore = new DrawInterpreter();
        aBefore.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        foreach (string aCommand in Unit3L1Commands)
        {
            aBefore.Eval($"rename {aCommand} {{}}");
        }

        List<DrawTestCase> aCases = aCorpus.Cases().ToList();
        var aScans = aCases.AsParallel().AsOrdered().Select(c => aCorpus.Scan(c, anInterpreter)).ToList();
        List<DrawTestCase> aRunnableBefore = aCases.AsParallel().AsOrdered().Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        output.WriteLine($"{aCases.Count} cases; runnable before 3L1: {aRunnableBefore.Count}, now: {aRunnable.Count}");

        List<DrawCaseRun> Run(List<DrawTestCase> theCases, Action<DrawInterpreter>? theSetup)
            => theCases.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                       .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DataPath, TimeSpan.FromSeconds(120), theSetup))
                       .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                       .ToList();

        void Statuses(string theWhat, List<DrawCaseRun> theRuns)
        {
            output.WriteLine($"{theWhat}: " + string.Join(", ", theRuns.GroupBy(r => r.Status.Split(' ')[0])
                                                                        .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                                        .Select(g => $"{g.Key} {g.Count()}")));
        }

        Statuses("before 3L1", Run(aRunnableBefore, i =>
        {
            foreach (string aCommand in Unit3L1Commands)
            {
                i.Eval($"rename {aCommand} {{}}");
            }
        }));
        List<DrawCaseRun> aRuns = Run(aRunnable, null);
        Statuses("now", aRuns);

        foreach (DrawCaseRun aRun in aRuns)
        {
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}");
            if (!aRun.Passed)
            {
                // the first line that made it fail
                string? aLine = aRun.Log.Split('\n').FirstOrDefault(l => s_failure.IsMatch(l));
                output.WriteLine($"      {aLine ?? aRun.Log.Split('\n').Reverse().Skip(1).FirstOrDefault()}");
            }
        }

        // the cases that miss one command: what to implement next
        var aNext = aScans.Where(s => s.Missing.Count == 1)
                          .GroupBy(s => s.Missing.Single())
                          .OrderByDescending(g => g.Count())
                          .Take(15);
        output.WriteLine("Cases missing exactly one command, by command:");
        foreach (var aGroup in aNext)
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        Assert.True(aRunnable.Count > aRunnableBefore.Count);
        Assert.Contains(aRuns, r => r.Status == "OK");
    }
}
