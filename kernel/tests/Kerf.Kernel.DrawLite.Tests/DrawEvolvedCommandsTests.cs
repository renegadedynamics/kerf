// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 5O2's <c>evolved</c> (BRepTest_SweepCommands.cxx) on hand-made spines and profiles, and on OCCT's corpus: the
/// cases that use it, runnable before and after the unit, and how they end (slanted profile edges are swept by
/// BRepFill_Pipe since unit 5S2; <c>-v</c> needs BRepFill_AdvancedEvolved, phase 7).
/// </summary>
public class DrawEvolvedCommandsTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>Draw-lite before unit 5O2's evolved.</summary>
    internal static void Before5O2Evolved(DrawInterpreter theInterpreter) => theInterpreter.Eval("rename evolved {}");

    private static DrawInterpreter NewInterpreter()
    {
        var anInterpreter = new DrawInterpreter();
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    // sprops' or vprops' mass (checkprops needs a value to check against with each option)
    private static double Mass(DrawInterpreter theDI, string theCommand)
        => double.Parse(System.Text.RegularExpressions.Regex.Match(theDI.Eval(theCommand), @"Mass :\s+(\S+)").Groups[1].Value,
                        System.Globalization.CultureInfo.InvariantCulture);

    [Fact]
    public void Evolved_builds_walls_and_solids()
    {
        DrawInterpreter aDI = NewInterpreter();
        Assert.Contains("Make evolved profile on spine.", aDI.Eval("evolved"));
        aDI.Eval("polyline s 0 0 0 10 0 0 10 10 0 0 10 0 0 0 0");
        aDI.Eval("polyline p 0 0 0 0 0 5");
        Assert.Equal("", aDI.Eval("evolved r -s s -p p"));
        Assert.Equal(200.0, Mass(aDI, "sprops r"), 1e-9);
        Assert.Equal("", aDI.Eval("evolved r -s s -p p -solid"));
        Assert.Equal(500.0, Mass(aDI, "vprops r"), 1e-9);
        aDI.Eval("mkplane f s");
        aDI.Eval("polyline q 5 0 0 5 0 4");
        Assert.Equal("", aDI.Eval("evolved r -s f -p q -a"));
        Assert.Equal(160.0, Mass(aDI, "sprops r"), 1e-9);
        Assert.Contains("This shape seems to be valid", aDI.Eval("checkshape r"));
        // a wrong option and a missing profile
        Assert.ThrowsAny<Exception>(() => aDI.Eval("evolved r s p"));
        Assert.ThrowsAny<Exception>(() => aDI.Eval("evolved r -s s"));
        // the volume mode (BRepFill_AdvancedEvolved, unit 7G): the square's 10 x 10 x 5 slab
        Assert.Equal("", aDI.Eval("evolved r -s s -p p -solid -v"));
        Assert.Equal(500.0, Mass(aDI, "vprops r"), 1e-6);
        // a slanted profile, swept by BRepFill_Pipe since unit 5S2 (native value from tools/evorec)
        aDI.Eval("polyline t 0 0 0 0 1 1");
        Assert.Equal("", aDI.Eval("evolved r -s s -p t"));
        Assert.Equal(50.911688245431428, Mass(aDI, "sprops r"), 1e-4); // sprops prints 6 digits
        Assert.Contains("This shape seems to be valid", aDI.Eval("checkshape r"));
    }

    [Fact]
    public void Corpus_cases_using_evolved_before_and_after_5O2()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before5O2Evolved(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => s.Commands.Contains("evolved"))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use evolved; runnable before 5O2: {aRunnableBefore.Count}, after: {aRunnable.Count}");
        foreach (DrawCaseScan aScan in aScans.Where(s => !s.IsRunnable))
        {
            output.WriteLine($"  {aScan.Case.Id} misses {string.Join(", ", aScan.Missing)}");
        }

        var aRuns = aRunnable.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                              TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 5O2: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
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
                                                           || l.Contains("Faulty", StringComparison.Ordinal)
                                                           || l.Contains("NotImplemented", StringComparison.Ordinal));
                output.WriteLine($"      {aLine ?? aLines.Reverse().Skip(1).FirstOrDefault()}");
            }
        }

        Assert.Empty(aRunnableBefore);
        Assert.All(aScans, s => Assert.DoesNotContain("evolved", s.Missing));
    }
}
