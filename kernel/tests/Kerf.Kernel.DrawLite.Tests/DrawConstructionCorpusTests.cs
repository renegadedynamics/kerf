// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 5W's corpus milestone: the cases of OCCT's corpus that use the construction commands it adds (lofts, pipe
/// shells, projections, continuity, qualified circles, ...), runnable before and after the unit, run as OCCT's harness
/// runs them and judged by OCCT's log check.
/// </summary>
public class DrawConstructionCorpusTests(ITestOutputHelper output) : ICorpusTest
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 5W added.</summary>
    internal static readonly string[] Unit5WCommands =
    [
        "pruled", "gener", "thrusections", "mksweep", "setsweep", "addsweep", "deletesweep", "buildsweep", "errorsweep",
        "simulsweep", "geompipe", "middlepath", "prj", "cprj", "ruled", "partuyau", "surfaceCcontinuity",
        "surfaceGcontinuity", "curveCcontinuity", "curveGcontinuity", "getcurvcontinuity", "getsurfcontinuity", "qcircle",
        "qline", "circ2d3Tan", "interpol", "tanginterpol",
    ];

    /// <summary>Draw-lite before unit 5W: without its commands.</summary>
    internal static void Before5W(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit5WCommands)
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

    private static IEnumerable<DrawCaseScan> Using5W => SharedCorpus.Scans.Where(s => Unit5WCommands.Any(s.Commands.Contains));

    // OCCTNET_5W_FILTER: only the cases whose id contains it
    private static List<DrawTestCase> ToRun(IEnumerable<DrawTestCase> theRunnable, string? theFilter)
        => [.. theRunnable.Where(c => theFilter is null || c.Id.Contains(theFilter, StringComparison.Ordinal))];

    // unit 7W's dversion made bugs/mesh/bug32241 runnable: it reads the triangulation DRAWEXE's checkview -display makes
    // (an attributed failure of DrawCheapGapsCorpusTests)
    private static void Check(DrawCaseRun theRun)
        => Assert.True(theRun.Passed || theRun.Status.StartsWith("SKIPPED", StringComparison.Ordinal)
                       || DrawCheapGapsCorpusTests.KnownFailures.ContainsKey(theRun.Case.Id), $"{theRun.Case.Id}: {theRun.Status}");

    /// <summary>The cases the corpus test runs, a row each in DrawConstructionCorpusShard*Tests (decision 2082).</summary>
    public static CorpusPlan Corpus { get; } =
        new(15, () => ToRun(Using5W.Where(s => s.IsRunnable).Select(s => s.Case), Environment.GetEnvironmentVariable("OCCTNET_5W_FILTER")), Check);

    [Fact]
    public void Corpus_cases_using_the_construction_commands_run_before_and_after_5W()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        // procedures count only once called: the bugs group's modalg grids define OFFSETSHAPE (phase 7's offset) for a few
        // of their cases
        var aFormerCorpus = new DrawCorpus(TestsDirectory) { FollowsCalledProcsOnly = false };
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before5W(aBefore);

        List<DrawCaseScan> aScans = [.. Using5W];
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = SharedCorpus.RunnableWith(aScans.Select(s => s.Case), aBefore);
        output.WriteLine($"{aScans.Count} cases use 5W's commands; runnable before 5W: {aRunnableBefore.Count}, after: {aRunnable.Count} "
                         + $"({SharedCorpus.RunnableWith(aRunnable, anInterpreter, aFormerCorpus).Count} with every defined procedure counted)");
        foreach (var aGroup in aRunnable.GroupBy(c => c.Group + "/" + c.Grid).OrderBy(g => g.Key, StringComparer.Ordinal))
        {
            output.WriteLine($"  {aGroup.Key,-32} {aGroup.Count()}");
        }

        // OCCTNET_5W_FILTER runs only the cases whose id contains it, and lists what the others matching it miss
        string? aFilter = Environment.GetEnvironmentVariable("OCCTNET_5W_FILTER");
        foreach (DrawCaseScan aScan in aScans.Where(s => aFilter is not null && !s.IsRunnable && s.Case.Id.Contains(aFilter, StringComparison.Ordinal)))
        {
            output.WriteLine($"  {aScan.Case.Id} misses {string.Join(' ', aScan.Missing)}");
        }

        List<DrawTestCase> aToRun = ToRun(aRunnable, aFilter);
        Assert.Equal(aToRun, Corpus.Cases);
        var aRuns = SharedCorpus.RunAll(aToRun)
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 5W: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
                                                               .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                               .Select(g => $"{g.Key} {g.Count()}")));
        foreach (DrawCaseRun aRun in aRuns)
        {
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}");
            if (!aRun.Passed || aRun.Status.StartsWith("BAD", StringComparison.Ordinal))
            {
                string[] aLines = aRun.Log.Split('\n');
                string? aLine = aLines.FirstOrDefault(l => l.Contains("rror", StringComparison.Ordinal)
                                                           || l.Contains("xception", StringComparison.Ordinal)
                                                           || l.Contains("Faulty", StringComparison.Ordinal));
                output.WriteLine($"      {aLine ?? aLines.Reverse().Skip(1).FirstOrDefault()}");
                if (Environment.GetEnvironmentVariable("OCCTNET_5W_LOG") is not null)
                {
                    output.WriteLine(aRun.Log);
                }
            }
        }

        output.WriteLine("Commands the other cases using 5W's commands miss (cases):");
        foreach (var aGroup in aScans.Where(s => !s.IsRunnable).SelectMany(s => s.Missing).GroupBy(c => c)
                                     .OrderByDescending(g => g.Count()).Take(25))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        // phase 5's construction commands in all (the plan's scan, section 1.5: 861 newly runnable, 601 with data)
        string[] aPhase5 =
        [
            .. Unit5WCommands, .. DrawConstraintCorpusTests.Unit5K2Commands, "gcarc", .. DrawPipeCommandsTests.Unit5L2Commands,
            .. DrawSurfaceFromCurvesCorpusTests.Unit5L3Commands, .. DrawFillingCorpusTests.Unit5PCommands,
            .. DrawFillet2dCorpusTests.Unit5RCommands, "pipe", "evolved", "mkoffset", "openoffset", "battencurve",
            "minvarcurve", "bisec",
        ];
        int aPhase5Runnable = SharedCorpus.Scans.Count(s => s.IsRunnable && aPhase5.Any(s.Commands.Contains));
        output.WriteLine($"cases using phase 5's construction commands runnable now: {aPhase5Runnable}");

        // unit 5W: 1,085 cases use its commands; runnable before 0, after 789. OK 499, BAD 40 (OCCT's TODOs: OCC7166's
        // loft areas, OCC24909 and OCC25110's sweeps), SKIPPED 250 (data files missing); none failing.
        Assert.Empty(aRunnableBefore);
        Assert.True(aRunnable.Count >= 789, $"{aRunnable.Count}");
        // unit 7W's dversion made bugs/mesh/bug32241 runnable: it reads the triangulation DRAWEXE's checkview -display makes
        // (an attributed failure of DrawCheapGapsCorpusTests)
        Assert.All(aRuns, Check);
        Assert.True(aFilter is not null || aRuns.Count(r => r.Status == "OK") >= 499);
    }
}
