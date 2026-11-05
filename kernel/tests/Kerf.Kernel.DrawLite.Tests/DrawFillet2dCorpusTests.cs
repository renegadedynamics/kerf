// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 5R's <c>chfi2d</c>, <c>fillet2d</c> and <c>chamfer2d</c> (BRepTest_Fillet2DCommands.cxx, over
/// BRepFilletAPI_MakeFillet2d, ChFi2d_FilletAPI and ChFi2d_ChamferAPI) on hand-made shapes and on OCCT's corpus: the
/// cases that use them, runnable before and after the commands, run as OCCT's harness runs them and judged by OCCT's
/// log check.
/// </summary>
public class DrawFillet2dCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 5R added.</summary>
    internal static readonly string[] Unit5RCommands = ["chfi2d", "fillet2d", "chamfer2d"];

    /// <summary>Draw-lite before unit 5R: without its commands.</summary>
    internal static void Before5R(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit5RCommands)
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
    public void The_commands_fillet_and_chamfer_as_Draw()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("polyline w 0 0 0 10 0 0 10 10 0 0 10 0 0 0 0");
        aDI.Eval("mkplane f w");
        aDI.Eval("explode f e");
        // chfi2d: a fillet on f_1/f_2's corner, a chamfer by distances on f_2/f_3's, by distance and angle on f_3/f_4's
        Assert.Equal("", aDI.Eval("chfi2d r f f_1 f_2 F 2 f_2 f_3 CDD 1 2 f_3 f_4 CDA 1 45"));
        Assert.Contains("FACE", aDI.Eval("nbshapes r"), StringComparison.Ordinal);
        Assert.Contains("EDGE      : 7", aDI.Eval("nbshapes r"), StringComparison.Ordinal);
        Assert.Contains("chfi2d : not enough args", Assert.ThrowsAny<Exception>(() => aDI.Eval("chfi2d r")).Message, StringComparison.Ordinal);
        Assert.Contains("does not share a vertex", Assert.ThrowsAny<Exception>(() => aDI.Eval("chfi2d r f f_1 f_3 F 1")).Message,
                        StringComparison.Ordinal);

        // fillet2d and chamfer2d on two edges: the fillet or chamfer, and the edges trimmed in place
        aDI.Eval("line l1 0 0 0 1 0 0");
        aDI.Eval("mkedge e1 l1 0 100");
        aDI.Eval("line l2 0 0 0 0 1 0");
        aDI.Eval("mkedge e2 l2 0 100");
        Assert.Equal("", aDI.Eval("fillet2d a e1 e2 10"));
        Assert.Matches(@"Mass :\s+15\.708", aDI.Eval("lprops a"));
        aDI.Eval("mkedge e3 l1 0 100");
        aDI.Eval("mkedge e4 l2 0 100");
        Assert.Equal("", aDI.Eval("chamfer2d c e3 e4 3 4"));
        Assert.Matches(@"Mass :\s+5\n", aDI.Eval("lprops c"));
        Assert.Contains("Usage : fillet2d", Assert.ThrowsAny<Exception>(() => aDI.Eval("fillet2d a")).Message, StringComparison.Ordinal);
    }

    [Fact]
    public void Corpus_cases_using_the_2d_fillet_commands_run_before_and_after_5R()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before5R(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => Unit5RCommands.Any(s.Commands.Contains))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use chfi2d, fillet2d or chamfer2d; runnable before 5R: {aRunnableBefore.Count}, after: {aRunnable.Count}");

        var aRuns = aRunnable.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                              TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 5R: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
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

        foreach (DrawCaseScan aScan in aScans.Where(s => !s.IsRunnable))
        {
            output.WriteLine($"  not runnable: {aScan.Case.Id} misses {string.Join(" ", aScan.Missing)}");
        }
        output.WriteLine("Commands the other cases miss (cases):");
        foreach (var aGroup in aScans.Where(s => !s.IsRunnable).SelectMany(s => s.Missing).GroupBy(c => c).OrderByDescending(g => g.Count()).Take(15))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        Assert.Empty(aRunnableBefore);
        Assert.NotEmpty(aRunnable);
        // bugs/moddata_1/bug20823 reads OCC20823.brep, which isn't public: SKIPPED, as OCCT's harness reports it
        Assert.All(aRuns, r => Assert.True(r.Passed || r.Status.StartsWith("SKIPPED (data file is missing)", StringComparison.Ordinal),
                                           $"{r.Case.Id}: {r.Status}"));
    }
}
