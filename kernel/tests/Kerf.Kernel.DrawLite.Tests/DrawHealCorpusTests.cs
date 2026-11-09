// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 6L's healing commands (sewing, SWDRAW's ShapeAnalysis, ShapeFix, ShapeTool and ShapeUpgrade commands) on OCCT's
/// corpus: the cases that use them, runnable before and after the commands, run as OCCT's harness runs them and judged
/// by OCCT's log check.
/// </summary>
/// <remarks>
/// About 900 cases are runnable (700 of them the sewing grids'); they run in under half a minute, so all of them run.
/// </remarks>
public class DrawHealCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands of unit 6L's healing half.</summary>
    internal static readonly string[] Unit6LHealCommands =
    [
        "sewing", "fastsewing",
        "projface", "projcurve", "projpcurve", "statshape", "comptol", "freebounds", "fbprops", "fbclose", "getareacontour",
        "checkselfintersection", "checkedge", "getanasurf", "getanacurve",
        "edgesameparam", "settolerance", "reface", "fixshape", "fixwgaps", "fixsmall", "fixsmalledges", "fixsmallfaces",
        "checkfclass2d", "connectedges",
        "anaedges", "expwire", "ssolid", "samerange", "sortcompound",
        "removeloc", "copytranslate", "reshape",
    ];

    /// <summary>Draw-lite before unit 6L's healing half: without its commands.</summary>
    internal static void Before6LHeal(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit6LHealCommands)
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
    public void Corpus_cases_using_the_healing_commands_run_before_and_after_6L()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before6LHeal(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => Unit6LHealCommands.Any(s.Commands.Contains))
                                           // the de group's STEP cases read with ReadStep: unit 8G2's class runs them
                                           .Where(s => !DrawStepCafCorpusTests.IsDeStepCase(s.Case))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use the healing commands; runnable before 6L: {aRunnableBefore.Count}, after: {aRunnable.Count}");

        var aRuns = aRunnable.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                          .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                           TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                          .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                          .ToList();
        output.WriteLine("after 6L: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
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
                if (Environment.GetEnvironmentVariable("KERF_HEAL_LOG") is not null)
                {
                    output.WriteLine(aRun.Log);
                }
            }
        }

        output.WriteLine("Commands the other cases using the healing commands miss (cases):");
        foreach (var aGroup in aScans.SelectMany(s => s.Missing).GroupBy(c => c).OrderByDescending(g => g.Count()).Take(15))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        Assert.True(aRunnable.Count > aRunnableBefore.Count);
        Assert.Contains(aRuns, r => r.Status == "OK");
        // a case failing is a port error to fix or a difference to attribute (docs/occt-issues.md); none is expected
        // the Boolean cases' attributed failures (bugs/modalg_7/bug26034's bopcheck, bugs/modalg_5/bug23881's sewn shell)
        Assert.DoesNotContain(aRuns, r => r.Status.StartsWith("FAILED", StringComparison.Ordinal) && !KnownFailures.Contains(r.Case.Id)
                                          && !DrawBooleanCorpusTests.KnownFailures.Contains(r.Case.Id)
                                          && !DrawStepCorpusTests.KnownFailures.Contains(r.Case.Id)
                                          && !DrawStepCafCorpusTests.KnownFailures.Contains(r.Case.Id));
    }

    /// <summary>The corpus cases that fail for an attributed reason (see docs/occt-issues.md, unit 6L).</summary>
    private static readonly HashSet<string> KnownFailures = new(StringComparer.Ordinal)
    {
    };
}
