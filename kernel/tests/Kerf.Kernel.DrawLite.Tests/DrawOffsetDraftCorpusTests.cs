// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 7G's offset and draft commands (BRepTest_FeatureCommands' 3D offset commands, BRepTest_DraftAngleCommands,
/// the rolling ball and <c>nproject</c>) on OCCT's corpus: the cases that use them, runnable after the unit and not
/// before, run as OCCT's harness runs them and judged by OCCT's log check.
/// </summary>
[Collection(MeshDefaultsCollection.Name)]
public class DrawOffsetDraftCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands of unit 7G.</summary>
    internal static readonly string[] Unit7GCommands =
    [
        "thickshell", "mkoffsetshape", "offsetshape", "offsetcompshape", "offsetparameter", "offsetload", "offsetonface",
        "offsetperform", "offsetshapesimple", "depouille", "ndepouille", "draft", "rollingball", "brollingball",
        "trollingball", "nproject",
    ];

    /// <summary>Draw-lite before unit 7G: without its commands.</summary>
    internal static void Before7G(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit7GCommands)
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
    public void Corpus_cases_using_the_offset_and_draft_commands_of_7G_run()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before7G(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => Unit7GCommands.Any(s.Commands.Contains))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        var aRunnableBefore = new HashSet<string>(aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable)
                                                        .Select(c => c.Id), StringComparer.Ordinal);
        List<DrawTestCase> aNew = aRunnable.Where(c => !aRunnableBefore.Contains(c.Id)).ToList();
        output.WriteLine($"{aScans.Count} cases use the commands of 7G; runnable before 7G: {aRunnableBefore.Count}, after: {aRunnable.Count}, newly: {aNew.Count}");

        // OCCTNET_7G_FILTER: only the cases whose id contains it (and their logs with OCCTNET_7G_LOG)
        string? aFilter = Environment.GetEnvironmentVariable("OCCTNET_7G_FILTER");
        bool isFiltered = !string.IsNullOrEmpty(aFilter);
        if (isFiltered)
        {
            aNew = aNew.Where(c => c.Id.Contains(aFilter!, StringComparison.Ordinal)).ToList();
        }

        DrawCaseRun RunOne(DrawTestCase theCase)
            => aCorpus.RunAndCheck(theCase, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                   TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null);
        List<DrawCaseRun> aRuns = [.. aNew.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2)).Select(RunOne)];
        aRuns.Sort((x, y) => StringComparer.Ordinal.Compare(x.Case.Id, y.Case.Id));
        output.WriteLine("after 7G: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
                                                              .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                              .Select(g => $"{g.Key} {g.Count()}")));
        output.WriteLine("with data: " + aRuns.Count(r => !r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)));
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
                if (Environment.GetEnvironmentVariable("OCCTNET_7G_LOG") is not null)
                {
                    output.WriteLine(aRun.Log);
                }
            }
        }

        output.WriteLine("Commands the other cases using 7G's commands miss (cases):");
        foreach (var aGroup in aScans.Where(s => !s.IsRunnable).SelectMany(s => s.Missing).GroupBy(c => c)
                                     .OrderByDescending(g => g.Count()).Take(15))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        if (isFiltered)
        {
            return;
        }

        // unit 7G: 1,687 newly runnable (471 with data): OK 405, BAD 63 (OCCT's TODOs), SKIPPED 1,216 (data not public),
        // FAILED 3, attributed (KnownFailures); 67 cases using the commands still miss others (the viewer, IGES, the
        // feature commands of 7I, blend). With 7I's feature commands merged: 1,696 newly runnable, OK 414, FAILED 3 as before.
        // Unit 7Y: bug5805_3 and _4 pass (decision 1101): OK 416, FAILED 1. Unit 7Q2's view commands: 2 more, SKIPPED
        // (data not public). With 7N3's fillet commands (blend): 1,713 newly runnable, OK 425, FAILED 7 (six draft/angle
        // cases drafting a reconstructed fillet, attributed in DrawFilletCorpusTests.KnownFailures). With unit 8W's exec:
        // 1,714 (bugs/modalg_2/bug22864_1, SKIPPED: data not public)
        Assert.Equal(1714, aNew.Count);
        Assert.Equal(431, aRuns.Count(r => r.Status == "OK"));
        Assert.All(aRuns, r => Assert.True(!r.Status.StartsWith("FAILED", StringComparison.Ordinal) || KnownFailures.ContainsKey(r.Case.Id)
                                           || DrawBooleanCorpusTests.KnownFailures.Contains(r.Case.Id)
                                           || DrawFilletCorpusTests.KnownFailures.ContainsKey(r.Case.Id), $"{r.Case.Id}: {r.Status}"));
    }

    /// <summary>The corpus cases that fail for an attributed reason (see docs/occt-issues.md, unit 7G).</summary>
    internal static readonly Dictionary<string, string> KnownFailures = new(StringComparer.Ordinal)
    {
        // the progress indicator's output (XProgress), which Draw-lite's commands don't make (decision 954)
        ["bugs/modalg_7/bug32330"] = "progress",
    };
}
