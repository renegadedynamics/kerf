// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 7W's corpus test: the cases of OCCT's corpus that the cheap gaps of phase7-plan.md section 1.6 make runnable
/// (Draw's core commands, the geometry and topology commands over ported packages), run as OCCT's harness runs them and
/// judged by OCCT's log check.
/// </summary>
public class DrawCheapGapsCorpusTests(ITestOutputHelper output) : ICorpusTest
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
        ["bugs/modalg_7/bug32330"] = "progress", // runnable since unit 7G's offset commands
        ["de_mesh/shape_write_stl/B1"] = "progress", // runnable since unit 8H1's writestl
        ["lowalgos/progress/A1"] = "progress",
        ["lowalgos/progress/A2"] = "progress",
        ["lowalgos/progress/A3"] = "progress",
        ["lowalgos/progress/A4"] = "progress",
        // the triangulation DRAWEXE's `checkview -display` makes (the viewer); Draw-lite's checkview is a no-op
        ["bugs/mesh/bug32241"] = "viewer",
        // OCCT's script (decision 1309): `extrema s1 s1` and `extrema s2 s2` measure each plane against itself
        // (distance 0, native OCCT too) where the test expects 10, the distance between s1 and s2
        ["lowalgos/extss/bug29712_44"] = "OCCT's script: a plane's extrema with itself",
        // Draw's plugin loader (decision 1311): Draw-lite's commands are built in and its pload loads nothing
        ["bugs/fclasses/bug1281"] = "pload's plugins",
    };

    /// <summary>
    /// The cases that measure memory (<c>meminfo</c>), each in a process of its own (DrawCaseProcess, decision 1699):
    /// this test and DrawCaseProcessTests run them, so the corpus tests with an overlapping list leave them out.
    /// </summary>
    internal static bool MeasuresMemory(DrawCaseScan theScan) => theScan.Commands.Contains("meminfo");

    private static IEnumerable<DrawCaseScan> Using7W => SharedCorpus.Scans.Where(s => Unit7WCommands.Any(s.Commands.Contains));

    /// <summary>The cases the corpus test runs, those only 7W made runnable, a row each in DrawCheapGapsCorpusShard*Tests (decision 27801).</summary>
    public static CorpusPlan Corpus { get; } = new(9, () => SharedCorpus.NewlyRunnable(Using7W, Before7W), Check);

    // a case passes, or has no data, or fails for an attributed reason
    private static void Check(DrawCaseRun theRun)
        => Assert.True(theRun.Passed || theRun.Status.StartsWith("SKIPPED", StringComparison.Ordinal) || KnownFailures.ContainsKey(theRun.Case.Id)
                       || DrawFilletCorpusTests.KnownFailures.ContainsKey(theRun.Case.Id),
                       $"{theRun.Case.Id}: {theRun.Status}");

    [Fact]
    public void Corpus_cases_the_cheap_gaps_make_runnable_run_and_pass()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        DrawInterpreter aBefore = NewInterpreter();
        Before7W(aBefore);

        List<DrawCaseScan> aScans = [.. Using7W];
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = SharedCorpus.RunnableWith(aRunnable, aBefore);
        List<DrawTestCase> aNew = aRunnable.Except(aRunnableBefore).ToList();
        output.WriteLine($"{aScans.Count} cases use 7W's commands; runnable {aRunnable.Count}, of which {aNew.Count} only with 7W's commands");

        Assert.Equal(aNew, Corpus.Cases);
        List<DrawCaseRun> aRuns = [.. SharedCorpus.RunAll(aNew)];
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
        // FAILED 19, all attributed (KnownFailures); since unit 7G (MakeEvolved's volume mode) the 12 evolved/voluved
        // cases among them pass; with 7G's commands 596 are newly runnable (OK 354, BAD 18, SKIPPED 216, FAILED 8
        // attributed); with 7N3's fillet commands 603 (OK 359, SKIPPED 218, FAILED 8, the fillets' failures attributed in
        // DrawFilletCorpusTests.KnownFailures); 627 now, with de_mesh/shape_write_stl/B1 (8H1's writestl; FAILED, progress)
        Assert.True(aNew.Count >= 512, $"{aNew.Count}");
        Assert.True(aRuns.Count(r => r.Status == "OK") >= 337, $"{aRuns.Count(r => r.Status == "OK")}");
        Assert.All(aRuns, Check);
    }
}
