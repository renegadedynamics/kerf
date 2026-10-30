// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 5M's bisecting-locus commands (BRepTest_MatCommands.cxx: <c>topoload</c>, <c>drawcont</c>, <c>side</c>,
/// <c>mat</c>, <c>result</c>, <c>zone</c>) on a hand-made face and on OCCT's corpus: the cases that use them, runnable
/// before and after the unit (none: they need the offset commands too).
/// </summary>
public class DrawMatCommandsTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 5M's MAT part added (2dfit joined the ignored viewer commands).</summary>
    internal static readonly string[] Unit5MCommands = ["topoload", "drawcont", "side", "mat", "result", "zone", "2dfit"];

    /// <summary>Draw-lite before unit 5M: without its commands.</summary>
    internal static void Before5M(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit5MCommands)
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

    [Fact]
    public void Mat_runs_on_both_sides_of_a_face()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("polyline w 0 0 0 10 0 0 10 4 0 4 4 0 4 10 0 0 10 0 0 0 0");
        aDI.Eval("mkplane f w");
        Assert.Equal("", aDI.Eval("topoload f"));
        Assert.Equal("", aDI.Eval("drawcont"));
        Assert.Equal("", aDI.Eval("side left"));
        Assert.Equal("", aDI.Eval("mat"));
        Assert.Equal("", aDI.Eval("result"));
        aDI.Eval("explode w e");
        Assert.Equal("", aDI.Eval("zone w_1"));
        Assert.Equal("", aDI.Eval("side right"));
        Assert.Equal("", aDI.Eval("mat i"));
        Assert.Equal("", aDI.Eval("result"));
        Assert.Equal("", aDI.Eval("mat i o"));
        // a shape that isn't a face
        Assert.ThrowsAny<Exception>(() => aDI.Eval("topoload w"));
        Assert.ThrowsAny<Exception>(() => aDI.Eval("topoload f -approx"));
    }

    [Fact]
    public void Corpus_cases_using_mat_run_before_and_after_5M()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before5M(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => s.Commands.Contains("topoload"))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use topoload; runnable before 5M: {aRunnableBefore.Count}, after: {aRunnable.Count}");
        foreach (DrawCaseScan aScan in aScans.Where(s => !s.IsRunnable))
        {
            output.WriteLine($"  {aScan.Case.Id} misses {string.Join(", ", aScan.Missing)}");
        }

        var aRuns = aRunnable.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                              TimeSpan.FromSeconds(300), null))
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 5M: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
                                                                .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                                .Select(g => $"{g.Key} {g.Count()}")));
        foreach (DrawCaseRun aRun in aRuns)
        {
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}");
            if (!aRun.Passed)
            {
                string[] aLines = aRun.Log.Split('\n');
                string? aLine = aLines.FirstOrDefault(l => l.Contains("rror", StringComparison.Ordinal)
                                                           || l.Contains("xception", StringComparison.Ordinal)
                                                           || l.Contains("Faulty", StringComparison.Ordinal));
                output.WriteLine($"      {aLine ?? aLines.Reverse().Skip(1).FirstOrDefault()}");
            }
        }

        // The 8 cases (bugs/modalg_1 buc60830_*, bugs/modalg_4 bug745_*) also run the offset commands (phase 7's
        // offsetparameter/offsetload/offsetperform, 5O2's mkoffset) and the viewer's coordload/axo: none of 5M's
        // commands is missing any more, the cases become runnable with the 2D offset.
        Assert.Equal(8, aScans.Count);
        Assert.Empty(aRunnableBefore);
        Assert.All(aScans, s => Assert.Empty(s.Missing.Intersect(Unit5MCommands)));
        Assert.All(aScans, s => Assert.Contains("offsetperform", s.Missing));
        Assert.DoesNotContain(aRuns, r => r.Status.StartsWith("FAILED", StringComparison.Ordinal));
    }
}
