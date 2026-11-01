// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 7W's corpus test: the cases of OCCT's corpus that the cheap gaps of phase7-plan.md section 1.6 make runnable
/// (Draw's core commands, the geometry and topology commands over ported packages), run as OCCT's harness runs them and
/// judged by OCCT's log check.
/// </summary>
/// <remarks>
/// The cases that measure memory (<c>meminfo</c>) run one at a time after the others: Draw-lite runs cases side by side
/// in one process, and the managed heap is the process's, where DRAWEXE runs each case in its own process.
/// </remarks>
[Collection(MeshDefaultsCollection.Name)]
public class DrawCheapGapsCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 7W added.</summary>
    internal static readonly string[] Unit7WCommands =
    [
        "lastrep", "dversion", "meminfo", "dmode", "discr", "defle", "XProgress", "removeinternals", "repaint", "proj", "to2d",
        "to3d", "project", "projonplane", "crvpoints", "crvtpoints", "2dproj", "2dextrema", "2dintersect", "2dintanalytical",
        "intconcon", "incdeg", "insertpole", "cmovepoint", "convert", "segsur", "setweight", "validrange", "shape", "subshape",
        "BRepIntCS", "xbounds", "xclassify", "concatC0wire",
    ];

    /// <summary>Draw-lite before unit 7W: without its commands.</summary>
    internal static void Before7W(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit7WCommands)
        {
            theInterpreter.Eval($"rename {aCommand} {{}}");
        }
    }

    private static DrawInterpreter NewInterpreter()
    {
        var anInterpreter = new DrawInterpreter();
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    /// <summary>The corpus cases that fail for an attributed reason (decisions of unit 7W, docs/occt-issues.md).</summary>
    internal static readonly Dictionary<string, string> KnownFailures = new(StringComparer.Ordinal)
    {
        // the progress indicator's output, which Draw-lite's commands don't make (decision 954)
        ["bugs/modalg_7/bug21264"] = "progress",
        ["bugs/modalg_7/bug30788"] = "progress",
        ["lowalgos/progress/A1"] = "progress",
        ["lowalgos/progress/A2"] = "progress",
        ["lowalgos/progress/A3"] = "progress",
        ["lowalgos/progress/A4"] = "progress",
        // the triangulation DRAWEXE's `checkview -display` makes (the viewer); Draw-lite's checkview is a no-op
        ["bugs/mesh/bug32241"] = "viewer",
        // BRepOffsetAPI_MakeEvolved's volume mode (BRepFill_AdvancedEvolved, phase 7)
        ["evolved/voluved/AGT001"] = "phase 7",
        ["evolved/voluved/AGT002"] = "phase 7",
        ["evolved/voluved/AGT003"] = "phase 7",
        ["evolved/voluved/AGT004"] = "phase 7",
        ["evolved/voluved/AGT005"] = "phase 7",
        ["evolved/voluved/AGT006"] = "phase 7",
        ["evolved/voluved/AGT007"] = "phase 7",
        ["evolved/voluved/AGT008"] = "phase 7",
        ["evolved/voluved/AGT009"] = "phase 7",
        ["evolved/voluved/bug26470_1"] = "phase 7",
        ["evolved/voluved/bug26470_2"] = "phase 7",
        ["evolved/voluved/bug30781"] = "phase 7",
    };

    /// <summary>
    /// The cases that measure memory (<c>meminfo</c>): they pass only run one at a time, so this test alone runs them,
    /// and the other corpus tests leave them out.
    /// </summary>
    internal static bool MeasuresMemory(DrawCaseScan theScan) => theScan.Commands.Contains("meminfo");

    [Fact]
    public void Corpus_cases_the_cheap_gaps_make_runnable_run_and_pass()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before7W(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => Unit7WCommands.Any(s.Commands.Contains))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = aRunnable.Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        List<DrawTestCase> aNew = aRunnable.Except(aRunnableBefore).ToList();
        output.WriteLine($"{aScans.Count} cases use 7W's commands; runnable {aRunnable.Count}, of which {aNew.Count} only with 7W's commands");

        bool UsesMemory(DrawTestCase theCase) => MeasuresMemory(aScans.First(s => s.Case == theCase));
        DrawCaseRun RunOne(DrawTestCase theCase)
            => aCorpus.RunAndCheck(theCase, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath, TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null);
        List<DrawCaseRun> aRuns = [.. aNew.Where(c => !UsesMemory(c)).AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                                          .Select(RunOne),
                                   .. aNew.Where(UsesMemory).Select(RunOne)];
        aRuns.Sort((x, y) => StringComparer.Ordinal.Compare(x.Case.Id, y.Case.Id));
        output.WriteLine("after 7W: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
                                                               .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                               .Select(g => $"{g.Key} {g.Count()}")));
        foreach (DrawCaseRun aRun in aRuns.Where(r => r.Status != "OK"))
        {
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}");
            if (!aRun.Passed)
            {
                string[] aLines = aRun.Log.Split('\n');
                string? aLine = aLines.FirstOrDefault(l => l.Contains("rror", StringComparison.Ordinal)
                                                           || l.Contains("xception", StringComparison.Ordinal));
                output.WriteLine($"      {aLine ?? aLines.Reverse().Skip(1).FirstOrDefault()}");
            }
        }

        output.WriteLine("Commands the other cases using 7W's commands miss (cases):");
        foreach (var aGroup in aScans.Where(s => !s.IsRunnable).SelectMany(s => s.Missing).GroupBy(c => c)
                                     .OrderByDescending(g => g.Count()).Take(20))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        // measured (decision 988): 512 newly runnable; OK 337, BAD 18 (OCCT's TODOs), SKIPPED 138 (data not public),
        // FAILED 19, all attributed (KnownFailures)
        Assert.True(aNew.Count >= 512, $"{aNew.Count}");
        Assert.True(aRuns.Count(r => r.Status == "OK") >= 337, $"{aRuns.Count(r => r.Status == "OK")}");
        Assert.All(aRuns, r => Assert.True(r.Passed || r.Status.StartsWith("SKIPPED", StringComparison.Ordinal) || KnownFailures.ContainsKey(r.Case.Id),
                                           $"{r.Case.Id}: {r.Status}"));
    }
}
