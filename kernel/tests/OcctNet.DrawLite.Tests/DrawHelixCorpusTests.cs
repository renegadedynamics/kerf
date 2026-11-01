// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 7D's helix commands (BRepTest_HelixCommands.cxx, over HelixBRep_BuilderHelix): <c>setaxis</c>,
/// <c>comphelix</c>, <c>helix</c>, <c>spiral</c>, <c>comphelix2</c>, <c>helix2</c> and <c>spiral2</c>, on hand-made
/// parameters and on OCCT's corpus: the cases that use them, runnable after the commands and not before, run as OCCT's
/// harness runs them and judged by OCCT's log check.
/// </summary>
public class DrawHelixCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 7D added.</summary>
    internal static readonly string[] Unit7DCommands = ["setaxis", "comphelix", "helix", "spiral", "comphelix2", "helix2", "spiral2"];

    /// <summary>Draw-lite before unit 7D: without its commands.</summary>
    internal static void Before7D(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit7DCommands)
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
    public void The_commands_build_helix_wires_as_Draw()
    {
        DrawInterpreter aDI = NewInterpreter();
        // helix: 5 turns of diameter 100 over a height of 100 (P1 is a number of turns: PF1 = 0)
        string aResult = aDI.Eval("helix h 1 100 100 5 0");
        Assert.Matches(@"^WarningStatus = 0\nToleranceReached = [0-9.e+-]+\n$", aResult);
        Assert.Contains("WIRE      : 1", aDI.Eval("nbshapes h"), StringComparison.Ordinal);
        Assert.Contains("EDGE      : 5", aDI.Eval("nbshapes h"), StringComparison.Ordinal);
        // 5 * sqrt((100 pi)^2 + 20^2) = 1573.98
        Assert.Matches(@"Mass :\s+1573\.98\n", aDI.Eval("lprops h"));

        // the other forms
        Assert.StartsWith("WarningStatus", aDI.Eval("comphelix c 2 100 80 60 50 50 10 10 1 1"), StringComparison.Ordinal);
        Assert.StartsWith("WarningStatus", aDI.Eval("spiral s 1 100 20 100 10 1"), StringComparison.Ordinal);
        Assert.StartsWith("WarningStatus", aDI.Eval("comphelix2 c2 2 100 80 60 10 10 2 3"), StringComparison.Ordinal);
        Assert.StartsWith("WarningStatus", aDI.Eval("helix2 h2 1 100 20 5"), StringComparison.Ordinal);
        Assert.StartsWith("WarningStatus", aDI.Eval("spiral2 s2 1 100 20 10 10"), StringComparison.Ordinal);
        Assert.Equal(aDI.Eval("lprops h"), aDI.Eval("lprops h2"));

        // an error status is the result, and no shape is bound
        Assert.Equal("ErrorStatus = 11\n", aDI.Eval("helix e 1 100 100 0 1"));
        Assert.Equal("0", aDI.Eval("isdraw e"));
        // wrong argument counts are usage errors
        Assert.Contains("Usage : helix", Assert.ThrowsAny<Exception>(() => aDI.Eval("helix e 2 100 100 5 0")).Message,
                        StringComparison.Ordinal);
        Assert.Contains("Usage : setaxis", Assert.ThrowsAny<Exception>(() => aDI.Eval("setaxis 0 0 0")).Message,
                        StringComparison.Ordinal);

        // setaxis moves later helixes: about X, the centre of mass is on X
        Assert.Equal("", aDI.Eval("setaxis 10 0 0 1 0 0 0 1 0"));
        aDI.Eval("helix hx 1 100 100 5 0");
        Assert.Matches(@"X =\s+60\n", aDI.Eval("lprops hx"));
        // another interpreter keeps OCCT's default axis
        DrawInterpreter anOther = NewInterpreter();
        anOther.Eval("helix h 1 100 100 5 0");
        Assert.Equal(aDI.Eval("lprops h"), anOther.Eval("lprops h"));
    }

    [Fact]
    public void Corpus_cases_using_the_helix_commands_run_after_7D()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before7D(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => Unit7DCommands.Any(s.Commands.Contains))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use the helix commands; runnable before 7D: {aRunnableBefore.Count}, after: {aRunnable.Count}");

        var aRuns = aRunnable.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                              TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 7D: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
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

        Assert.Empty(aRunnableBefore);
        Assert.Equal(56, aScans.Count);
        Assert.Equal(56, aRunnable.Count);
        Assert.All(aRuns, r => Assert.True(r.Passed || KnownFailures.Contains(r.Case.Id), $"{r.Case.Id}: {r.Status}"));
    }

    /// <summary>Cases that fail in Draw-lite, each attributed (docs/occt-issues.md, unit 7D).</summary>
    private static readonly HashSet<string> KnownFailures = new(StringComparer.Ordinal)
    {
    };
}
