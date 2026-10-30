// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 5S1's <c>pipe</c> (BRepTest_SweepCommands.cxx, BRepOffsetAPI_MakePipe) and the corpus's cases that use it.
/// BRepFill_Sweep is ported by unit 5S2: until then every pipe throws Standard_NotImplemented, which these tests pin
/// as the expected failure (unit 5S2 turns them into passing runs).
/// </summary>
public class DrawPipeMakerCommandsTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    private const string NotPorted = "BRepFill_Sweep is ported by unit 5S2";

    /// <summary>Draw-lite before unit 5S1: without <c>pipe</c>.</summary>
    internal static void Before5S1(DrawInterpreter theInterpreter) => theInterpreter.Eval("rename pipe {}");

    private static DrawInterpreter NewInterpreter()
    {
        var anInterpreter = new DrawInterpreter();
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    [Fact]
    public void Pipe_prints_its_help_checks_its_arguments_and_waits_for_5S2()
    {
        var i = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        Assert.StartsWith("pipe result Wire_spine Profile [Mode [Approx]]\n", i.Eval("pipe"), StringComparison.Ordinal);
        Assert.ThrowsAny<DrawException>(() => i.Eval("pipe r"));
        Assert.ThrowsAny<DrawException>(() => i.Eval("pipe r nospine noprofile"));

        i.Eval("polyline sp 0 0 0 0 0 10 3 0 15");
        i.Eval("circle c 0 0 0 1");
        i.Eval("mkedge e c");
        i.Eval("wire w e");
        foreach (string aMode in new[] { "", " 0", " 1", " 2", " 2 1" })
        {
            var anError = Assert.ThrowsAny<DrawException>(() => i.Eval("pipe r sp w" + aMode));
            Assert.Contains(NotPorted, anError.Message, StringComparison.Ordinal);
        }
    }

    [Fact]
    public void Corpus_cases_using_pipe_run_before_and_after_5S1()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before5S1(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => s.Commands.Contains("pipe"))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use pipe; runnable before 5S1: {aRunnableBefore.Count}, after: {aRunnable.Count}");

        var aRuns = aRunnable.Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                              TimeSpan.FromSeconds(120), null))
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 5S1: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
                                                                .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                                .Select(g => $"{g.Key} {g.Count()}")));
        foreach (DrawCaseRun aRun in aRuns)
        {
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}"
                             + (aRun.Log.Contains(NotPorted, StringComparison.Ordinal) ? " (waits for 5S2's BRepFill_Sweep)" : ""));
        }
        output.WriteLine("Commands the other cases using pipe miss (cases):");
        foreach (var aGroup in aScans.Where(s => !s.IsRunnable).SelectMany(s => s.Missing).GroupBy(c => c)
                                     .OrderByDescending(g => g.Count()).Take(15))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        Assert.Empty(aRunnableBefore);
        Assert.NotEmpty(aRunnable);
        // The expected failure until unit 5S2: every runnable case reaches the placeholder sweep.
        // (cases the corpus skips, for missing data, never reach it)
        Assert.All(aRuns.Where(r => !r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)),
                   r => Assert.Contains(NotPorted, r.Log, StringComparison.Ordinal));
    }
}
