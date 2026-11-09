// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 5P's <c>plate</c>, <c>gplate</c>, <c>approxplate</c>, <c>filling</c> and <c>fillingparam</c>
/// (BRepTest_FillingCommands.cxx) on hand-made shapes and on OCCT's corpus: the cases that use them, runnable before and
/// after the commands, run as OCCT's harness runs them and judged by OCCT's log check.
/// </summary>
public class DrawFillingCorpusTests(ITestOutputHelper output) : ICorpusTest
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 5P added.</summary>
    internal static readonly string[] Unit5PCommands = ["plate", "gplate", "approxplate", "filling", "fillingparam"];

    /// <summary>Draw-lite before unit 5P: without its commands.</summary>
    internal static void Before5P(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit5PCommands)
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
    public void Filling_commands_fill_a_box_hole_as_Draw()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("box b 10 10 10");
        aDI.Eval("explode b f");
        // b_6 is the top face (z = 10); its four edges bound the hole, each tangent to a side face
        aDI.Eval("explode b_6 e");
        aDI.Eval("explode b_1 e");

        // filling: four G0 bounds
        string aFill = aDI.Eval("filling r 4 0 0 b_6_1 0 b_6_2 0 b_6_3 0 b_6_4 0");
        Assert.StartsWith(" dist. max = ", aFill, StringComparison.Ordinal);
        Assert.Contains(" ; diffcurv max = ", aFill, StringComparison.Ordinal);
        Assert.Contains("FACE", aDI.Eval("whatis r"), StringComparison.Ordinal);
        // with a point
        aDI.Eval("point p 5 5 12");
        aDI.Eval("filling r2 4 0 1 b_6_1 0 b_6_2 0 b_6_3 0 b_6_4 0 p");
        // gplate: the same four edges, then with an initial surface
        aDI.Eval("gplate g 4 0 b_6_1 0 b_6_2 0 b_6_3 0 b_6_4 0");
        Assert.Contains("FACE", aDI.Eval("whatis g"), StringComparison.Ordinal);
        aDI.Eval("gplate g2 4 1 b_6 b_6_1 0 b_6_2 0 b_6_3 0 b_6_4 0 p");
        // fillingparam: list, set, reset
        Assert.Contains("NbPtsOnCur = 10", aDI.Eval("fillingparam -l"), StringComparison.Ordinal);
        aDI.Eval("fillingparam -r 3 15 2 0");
        Assert.Contains("NbPtsOnCur = 15", aDI.Eval("fillingparam -l"), StringComparison.Ordinal);
        aDI.Eval("fillingparam -i");
        Assert.Contains("NbPtsOnCur = 10", aDI.Eval("fillingparam -l"), StringComparison.Ordinal);
        Assert.Contains("Wrong parameters", Assert.ThrowsAny<Exception>(() => aDI.Eval("fillingparam -x")).Message, StringComparison.Ordinal);
        Assert.StartsWith("fillingparam : options are", aDI.Eval("fillingparam"), StringComparison.Ordinal);
    }

    /// <summary>The cases the corpus test runs, a row each in DrawFillingCorpusShard*Tests (decision 27801).</summary>
    public static CorpusPlan Corpus { get; } = new(1, () => SharedCorpus.Scans.Where(s => Unit5PCommands.Any(s.Commands.Contains)).Where(s => s.IsRunnable).Select(s => s.Case));

    [Fact]
    public void Corpus_cases_using_the_filling_commands_run_before_and_after_5P()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        DrawInterpreter aBefore = NewInterpreter();
        Before5P(aBefore);

        List<DrawCaseScan> aScans = [.. SharedCorpus.Scans.Where(s => Unit5PCommands.Any(s.Commands.Contains))];
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = SharedCorpus.RunnableWith(aScans.Select(s => s.Case), aBefore);
        output.WriteLine($"{aScans.Count} cases use the filling commands; runnable before 5P: {aRunnableBefore.Count}, after: {aRunnable.Count}");

        Assert.Equal(aRunnable, Corpus.Cases);
        var aRuns = SharedCorpus.RunAll(aRunnable)
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 5P: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
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
                if (Environment.GetEnvironmentVariable("KERF_FILL_LOG") is not null)
                {
                    output.WriteLine(aRun.Log);
                }
            }
        }

        output.WriteLine("Commands the other cases using the filling commands miss (cases):");
        foreach (var aGroup in aScans.SelectMany(s => s.Missing).GroupBy(c => c).OrderByDescending(g => g.Count()).Take(15))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        Assert.Empty(aRunnableBefore);
        Assert.NotEmpty(aRunnable);
        Assert.Contains(aRuns, r => r.Status == "OK");
    }
}
