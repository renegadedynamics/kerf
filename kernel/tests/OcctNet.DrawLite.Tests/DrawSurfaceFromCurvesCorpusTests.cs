// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 5L3's <c>appsurf</c> and <c>fillcurves</c> (GeometryTest_SurfaceCommands.cxx) on hand-made curves and on OCCT's
/// corpus: the cases that use them, runnable before and after the commands, run as OCCT's harness runs them and judged
/// by OCCT's log check.
/// </summary>
public class DrawSurfaceFromCurvesCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 5L3 added.</summary>
    internal static readonly string[] Unit5L3Commands = ["appsurf", "fillcurves"];

    /// <summary>Draw-lite before unit 5L3: without its commands.</summary>
    internal static void Before5L3(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit5L3Commands)
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
    public void Appsurf_and_fillcurves_make_bspline_surfaces()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("circle c1 0 0 0 0 0 1 2");
        aDI.Eval("circle c2 0 0 5 0 0 1 3");
        aDI.Eval("circle c3 0 0 10 0 0 1 2");
        aDI.Eval("trim t1 c1 0 3");
        aDI.Eval("trim t2 c2 0 3");
        aDI.Eval("trim t3 c3 0 3");
        Assert.Equal("", aDI.Eval("appsurf s t1 t2 t3"));
        Assert.Contains("BSplineSurface", aDI.Eval("dump s"), StringComparison.Ordinal);

        aDI.Eval("line l1 0 0 0 1 0 0");
        aDI.Eval("trim e1 l1 0 10");
        aDI.Eval("line l2 10 0 0 0 1 0");
        aDI.Eval("trim e2 l2 0 10");
        aDI.Eval("line l3 10 10 0 -1 0 0");
        aDI.Eval("trim e3 l3 0 10");
        aDI.Eval("line l4 0 10 0 0 -1 0");
        aDI.Eval("trim e4 l4 0 10");
        foreach (int aStyle in new[] { 1, 3 })
        {
            Assert.Equal("", aDI.Eval($"fillcurves f{aStyle} e1 e2 e3 e4 {aStyle}"));
            Assert.Contains("BSplineSurface", aDI.Eval($"dump f{aStyle}"), StringComparison.Ordinal);
        }
        // Coons (the default style) needs four poles a side, as in OCCT
        Assert.Contains("invalid filling style",
                        Assert.ThrowsAny<Exception>(() => aDI.Eval("fillcurves f2 e1 e2 e3 e4")).Message, StringComparison.Ordinal);

        // wrong arguments
        Assert.ThrowsAny<Exception>(() => aDI.Eval("appsurf s t1"));
        Assert.ThrowsAny<Exception>(() => aDI.Eval("fillcurves f e1 e2 e3"));
        Assert.ThrowsAny<Exception>(() => aDI.Eval("appsurf s t1 nothing"));
    }

    [Fact]
    public void Corpus_cases_using_appsurf_or_fillcurves_run_before_and_after_5L3()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before5L3(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => Unit5L3Commands.Any(s.Commands.Contains))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use appsurf or fillcurves; runnable before 5L3: {aRunnableBefore.Count}, "
                         + $"after: {aRunnable.Count}");

        var aRuns = aRunnable.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                              TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 5L3: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
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

        output.WriteLine("Commands the other cases miss (cases):");
        foreach (var aGroup in aScans.Where(s => !s.IsRunnable).SelectMany(s => s.Missing).GroupBy(c => c)
                                     .OrderByDescending(g => g.Count()).Take(15))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        Assert.Empty(aRunnableBefore);
        Assert.DoesNotContain(aRuns, r => r.Status.StartsWith("FAILED", StringComparison.Ordinal));
    }
}
