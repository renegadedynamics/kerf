// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 5G's <c>intersect</c> (GeometryTest_CurveCommands.cxx) on hand-made surfaces and on OCCT's corpus: the cases
/// that use it, runnable before and after the command, run as OCCT's harness runs them and judged by OCCT's log check.
/// </summary>
public class DrawIntersectCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 5G added.</summary>
    internal static readonly string[] Unit5GCommands = ["intersect", "xdistcs", "checkoverlapedges"];

    /// <summary>Draw-lite before unit 5G: without its commands.</summary>
    internal static void Before5G(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit5GCommands)
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
    public void Intersect_names_its_lines_and_points_as_Draw()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("plane p 0 0 0 0 0 1");
        aDI.Eval("cylinder c 0 0 0 1 0 0 2");
        Assert.Equal("r_1 r_2 ", aDI.Eval("intersect r p c"));
        Assert.Contains("Line", aDI.Eval("dump r_1"), StringComparison.Ordinal);

        aDI.Eval("sphere s 0 0 0 1");
        Assert.Equal("q ", aDI.Eval("intersect q p s 1e-7"));
        Assert.Contains("Circle", aDI.Eval("dump q"), StringComparison.Ordinal);

        // no intersection: an empty result
        aDI.Eval("plane p2 0 0 5 0 0 1");
        Assert.Equal("", aDI.Eval("intersect e p p2"));

        // a curve and a surface: GeomAPI_IntCS's points
        aDI.Eval("line l 0 0 -3 0 0 1");
        Assert.Equal("x_1 x_2 ", aDI.Eval("intersect x l s"));

        // wrong arguments
        Assert.ThrowsAny<Exception>(() => aDI.Eval("intersect r p"));
        Assert.ThrowsAny<Exception>(() => aDI.Eval("intersect r p c 0 0 0"));
    }

    [Fact]
    public void Corpus_cases_using_intersect_run_before_and_after_5G()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before5G(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => s.Commands.Contains("intersect"))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use intersect; runnable before 5G: {aRunnableBefore.Count}, after: {aRunnable.Count}");

        var aRuns = aRunnable.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                              TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 5G: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
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

        var aNext = aScans.Where(s => s.Missing.Count == 1)
                          .GroupBy(s => s.Missing.Single())
                          .OrderByDescending(g => g.Count())
                          .Take(10);
        output.WriteLine("Commands the other cases using intersect miss (cases):");
        foreach (var aGroup in aScans.SelectMany(s => s.Missing).GroupBy(c => c).OrderByDescending(g => g.Count()).Take(15))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }
        List<DrawCaseScan> anIntss = aScans.Where(s => s.Case.Group == "lowalgos" && s.Case.Grid == "intss").ToList();
        output.WriteLine($"lowalgos/intss: {anIntss.Count} cases use intersect, {anIntss.Count(s => s.IsRunnable)} runnable; they miss (cases):");
        foreach (var aGroup in anIntss.SelectMany(s => s.Missing).GroupBy(c => c).OrderByDescending(g => g.Count()).Take(15))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }
        output.WriteLine("Cases using intersect that miss exactly one command, by command:");
        foreach (var aGroup in aNext)
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        Assert.Empty(aRunnableBefore);
        Assert.NotEmpty(aRunnable);
        Assert.Contains(aRuns, r => r.Status == "OK");
    }
}
