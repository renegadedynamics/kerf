// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 7C's healing commands (SWDRAW's ShapeUpgrade divide and convert commands, its ShapeCustom commands, and
/// ShapeProcess's <c>DT_ApplySeq</c> and <c>SPApply</c>) on OCCT's corpus: the cases that use them, runnable after the
/// unit and not before, run as OCCT's harness runs them and judged by OCCT's log check.
/// </summary>
[Collection(MeshDefaultsCollection.Name)]
public class DrawHealDivideCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands of unit 7C.</summary>
    internal static readonly string[] Unit7CCommands =
    [
        "DT_ShapeDivide", "DT_SplitAngle", "DT_ShapeConvert", "DT_ShapeConvertRev", "DT_SplitCurve2d", "DT_SplitCurve",
        "DT_SplitSurface", "offset2dcurve", "offsetcurve", "splitface", "DT_ToBspl", "DT_ClosedSplit", "DT_SplitByArea",
        "DT_SplitByNumber", "RemoveIntWires",
        "directfaces", "expshape", "scaleshape", "bsplres", "convtorevol",
        "DT_ApplySeq", "SPApply",
    ];

    /// <summary>Draw-lite before unit 7C: without its commands.</summary>
    internal static void Before7C(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit7CCommands)
        {
            if (theInterpreter.Eval($"info commands {aCommand}").Length > 0)
            {
                theInterpreter.Eval($"rename {aCommand} {{}}");
            }
        }
    }

    private static DrawInterpreter NewInterpreter()
    {
        var anInterpreter = new DrawInterpreter();
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    [Fact]
    public void Corpus_cases_using_the_healing_commands_of_7C_run()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before7C(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => Unit7CCommands.Any(s.Commands.Contains))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        var aRunnableBefore = new HashSet<string>(aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable)
                                                        .Select(c => c.Id), StringComparer.Ordinal);
        List<DrawTestCase> aNew = aRunnable.Where(c => !aRunnableBefore.Contains(c.Id)).ToList();
        output.WriteLine($"{aScans.Count} cases use the commands of 7C; runnable before 7C: {aRunnableBefore.Count}, after: {aRunnable.Count}, newly: {aNew.Count}");

        // OCCTNET_7C_FILTER: only the cases whose id contains it (and their logs with OCCTNET_7C_LOG)
        string? aFilter = Environment.GetEnvironmentVariable("OCCTNET_7C_FILTER");
        bool isFiltered = !string.IsNullOrEmpty(aFilter);
        if (isFiltered)
        {
            aNew = aNew.Where(c => c.Id.Contains(aFilter!, StringComparison.Ordinal)).ToList();
        }

        var aRuns = aNew.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                        .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                         TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                        .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                        .ToList();
        output.WriteLine("after 7C: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
                                                              .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                              .Select(g => $"{g.Key} {g.Count()}")));
        foreach (DrawCaseRun aRun in aRuns)
        {
            if (aRun.Status == "OK" || aRun.Status.StartsWith("SKIPPED", StringComparison.Ordinal))
            {
                continue;
            }
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}");
            if (!aRun.Passed)
            {
                string[] aLines = aRun.Log.Split('\n');
                string? aLine = aLines.FirstOrDefault(l => l.Contains("rror", StringComparison.Ordinal)
                                                           || l.Contains("xception", StringComparison.Ordinal)
                                                           || l.Contains("Faulty", StringComparison.Ordinal));
                output.WriteLine($"      {aLine ?? aLines.Reverse().Skip(1).FirstOrDefault()}");
                if (Environment.GetEnvironmentVariable("OCCTNET_7C_LOG") is not null)
                {
                    output.WriteLine(aRun.Log);
                }
            }
        }

        output.WriteLine("Commands the other cases using 7C's commands miss (cases):");
        foreach (var aGroup in aScans.Where(s => !s.IsRunnable).SelectMany(s => s.Missing).GroupBy(c => c)
                                     .OrderByDescending(g => g.Count()).Take(15))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        if (isFiltered)
        {
            return;
        }

        // unit 7C: 2,607 newly runnable (OK 1,584, BAD 41: OCCT's TODOs, SKIPPED 982: data not public); 11 cases using
        // the commands still miss others (testreadiges, the offset commands, vhlr, igesbrep)
        Assert.Equal(2607, aNew.Count);
        Assert.Equal(1584, aRuns.Count(r => r.Status == "OK"));
        Assert.All(aRuns, r => Assert.True(!r.Status.StartsWith("FAILED", StringComparison.Ordinal) || KnownFailures.Contains(r.Case.Id)
                                           || DrawBooleanCorpusTests.KnownFailures.Contains(r.Case.Id), $"{r.Case.Id}: {r.Status}"));
    }

    /// <summary>The corpus cases that fail for an attributed reason (see docs/occt-issues.md, unit 7C).</summary>
    internal static readonly HashSet<string> KnownFailures = new(StringComparer.Ordinal)
    {
    };
}
