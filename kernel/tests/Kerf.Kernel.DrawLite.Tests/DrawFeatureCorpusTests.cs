// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 7I's feature commands (BRepTest_FeatureCommands.cxx, over BRepFeat): on hand-made shapes and on OCCT's corpus:
/// the cases that use them, runnable after the commands and not before, run as OCCT's harness runs them and judged by
/// OCCT's log check.
/// </summary>
public class DrawFeatureCorpusTests(ITestOutputHelper output) : ICorpusTest
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 7I added.</summary>
    internal static readonly string[] Unit7ICommands =
    [
        "localope", "hole", "firsthole", "holend", "blindhole", "holecontrol", "wprism", "fprism", "wrotate", "frotate",
        "splitshape", "glue", "featprism", "featrevol", "featpipe", "featdprism", "featlf", "featrf", "addslide",
        "featperform", "featperformval", "endedges",
    ];

    /// <summary>Draw-lite before unit 7I: without its commands.</summary>
    internal static void Before7I(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit7ICommands)
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
    public void The_commands_build_features_as_Draw()
    {
        DrawInterpreter aDI = NewInterpreter();
        // feat/featlf/A1
        aDI.Eval("box b 1 1 1");
        aDI.Eval("profile pp c 0.3 180 ww");
        aDI.Eval("trotate pp 0 0.15 0 0 1 0 -90");
        aDI.Eval("ttranslate pp 0.3 0.3 0.9");
        aDI.Eval("plane pl 0.3 0.6 1 -1 0 0");
        aDI.Eval("featlf b pp pl 0.3 0 0 -0.2 0 0 1 1");
        aDI.Eval("featperform lf result");
        Assert.Matches(@"Mass :\s+6\.2514", aDI.Eval("sprops result"));

        // a prism defined per interpreter: another interpreter has none
        DrawInterpreter anOther = NewInterpreter();
        Assert.Contains("prism not defined", Assert.ThrowsAny<Exception>(() => anOther.Eval("featperform prism r")).Message,
                        StringComparison.Ordinal);

        // holecontrol
        Assert.Equal("Mode sans controle", aDI.Eval("holecontrol 0"));
        Assert.Equal("Mode avec controle", aDI.Eval("holecontrol 1"));
        aDI.Eval("hole h b 0.5 0.5 -1 0 0 1 0.1");
        Assert.Matches(@"Mass :\s+0\.96858", aDI.Eval("vprops h"));
    }

    /// <summary>The cases the corpus test runs, a row each in DrawFeatureCorpusShard*Tests (decision 2082).</summary>
    public static CorpusPlan Corpus { get; } = new(3, () => SharedCorpus.Scans.Where(s => Unit7ICommands.Any(s.Commands.Contains)).Where(s => s.IsRunnable).Select(s => s.Case));

    [Fact]
    public void Corpus_cases_using_the_feature_commands_run_after_7I()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        DrawInterpreter aBefore = NewInterpreter();
        Before7I(aBefore);

        List<DrawCaseScan> aScans = [.. SharedCorpus.Scans.Where(s => Unit7ICommands.Any(s.Commands.Contains))];
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = SharedCorpus.RunnableWith(aScans.Select(s => s.Case), aBefore);
        output.WriteLine($"{aScans.Count} cases use the feature commands; runnable before 7I: {aRunnableBefore.Count}, after: {aRunnable.Count}");

        // OCCTNET_7I_FILTER: only the cases whose id contains it (for debugging); OCCTNET_7I_LOG: the failures' logs
        string? aFilter = Environment.GetEnvironmentVariable("OCCTNET_7I_FILTER");
        bool isFiltered = !string.IsNullOrEmpty(aFilter);
        if (isFiltered)
        {
            aRunnable = aRunnable.Where(c => c.Id.Contains(aFilter!, StringComparison.Ordinal)).ToList();
        }

        Assert.Equal(aRunnable, Corpus.Cases);
        var aRuns = SharedCorpus.RunAll(aRunnable)
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 7I: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
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
                if (Environment.GetEnvironmentVariable("OCCTNET_7I_LOG") is not null)
                {
                    output.WriteLine(aRun.Log);
                }
            }
        }

        foreach (DrawCaseScan aScan in aScans.Where(s => !s.IsRunnable))
        {
            output.WriteLine($"  not runnable: {aScan.Case.Id} misses {string.Join(" ", aScan.Missing)}");
        }

        if (isFiltered)
        {
            return;
        }
        // unit 7I: 356 cases use the commands, 328 runnable after 7I (none before): OK 280, BAD 2 (OCCT's TODOs: feat/featlf/A4,
        // feat/featrevol/H9), SKIPPED 46 (data not public); the 28 others wait for depouille and blend (7G, 7N3), offsetshape
        // (7G), the viewer, STEP and QA commands. With 7G's offset and draft commands merged: 337 runnable, OK 289. With
        // 7N3's fillet commands (blend, fillet, bossage): 349 runnable, OK 301. With 8F's STEP commands: 350
        // runnable, SKIPPED 47
        Assert.Empty(aRunnableBefore);
        Assert.Equal(356, aScans.Count);
        Assert.Equal(350, aRunnable.Count);
        Assert.Equal(301, aRuns.Count(r => r.Status.StartsWith("OK", StringComparison.Ordinal)));
        Assert.Equal(47, aRuns.Count(r => r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)));
        Assert.All(aRuns, r => Assert.True(r.Status.StartsWith("OK", StringComparison.Ordinal) || r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)
                                           || r.Status.StartsWith("BAD", StringComparison.Ordinal) || KnownFailures.Contains(r.Case.Id),
                                           $"{r.Case.Id}: {r.Status}"));
    }

    /// <summary>Cases that fail in Draw-lite, each attributed (docs/occt-issues.md, unit 7I).</summary>
    private static readonly HashSet<string> KnownFailures = new(StringComparer.Ordinal)
    {
    };
}
