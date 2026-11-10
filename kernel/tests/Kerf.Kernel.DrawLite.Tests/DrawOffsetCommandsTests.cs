// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 5O2's <c>mkoffset</c> and <c>openoffset</c> (BRepTest_CurveCommands.cxx, BRepOffsetAPI_MakeOffset) on
/// hand-made wires and faces, and on OCCT's corpus: the cases that use them (the offset group's wire grids through its
/// end script's <c>command</c> branch, and bug cases), runnable before and after the unit, and how they end.
/// </summary>
public class DrawOffsetCommandsTests(ITestOutputHelper output) : ICorpusTest
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    private static readonly string[] Unit5O2Commands = ["mkoffset", "openoffset"];

    /// <summary>Draw-lite before unit 5O2's offset commands.</summary>
    internal static void Before5O2Offset(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit5O2Commands)
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

    // An offset group case that takes its end script's mkoffset branch (the other branches don't call mkoffset)
    private static bool IsMkoffsetCase(DrawTestCase theCase)
    {
        string aBegin = Path.Combine(TestsDirectory, theCase.Group, theCase.Grid, "begin");
        string aText = (File.Exists(aBegin) ? File.ReadAllText(aBegin) : "") + "\n" + File.ReadAllText(theCase.Path);
        return aText.Contains("set command \"mkoffset\"", StringComparison.Ordinal)
               || DrawCorpus.CommandsCalledBy(File.ReadAllText(theCase.Path)).Overlaps(Unit5O2Commands);
    }

    // lprops' length
    private static double Length(DrawInterpreter theDI, string theName)
        => double.Parse(System.Text.RegularExpressions.Regex.Match(theDI.Eval($"lprops {theName}"), @"Mass :\s+(\S+)").Groups[1].Value,
                        System.Globalization.CultureInfo.InvariantCulture);

    [Fact]
    public void Mkoffset_and_openoffset_offset_wires_and_faces()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("polyline w 0 0 0 10 0 0 10 10 0 0 10 0 0 0 0");
        aDI.Eval("mkplane f w");
        // three outward parallels of a square with arcs at the corners: lengths 40 + 2 pi d
        Assert.Equal("", aDI.Eval("mkoffset r f 3 1"));
        Assert.Equal(40 + 2 * Math.PI, Length(aDI, "r_1"), 1e-3);
        Assert.Equal(40 + 4 * Math.PI, Length(aDI, "r_2"), 1e-3);
        Assert.Equal(40 + 6 * Math.PI, Length(aDI, "r_3"), 1e-3);
        // intersection joins: squares
        Assert.Equal("", aDI.Eval("mkoffset r f 1 1 i"));
        Assert.Equal(48, Length(aDI, "r_1"), 1e-6);
        // inward, from the wire, at an altitude
        Assert.Equal("", aDI.Eval("mkoffset r w 1 -2 a 5"));
        Assert.Equal(24, Length(aDI, "r_1"), 1e-6);
        Assert.Matches(@"Center of gravity :\s+X =\s+5\s+Y =\s+5\s+Z =\s+5\s", aDI.Eval("lprops r_1"));
        // -approx
        Assert.Equal("", aDI.Eval("mkoffset r f 1 1 -approx"));
        Assert.Equal(40 + 2 * Math.PI, Length(aDI, "r_1"), 1e-3);
        // an open wire: closed around it, or open on both sides
        aDI.Eval("polyline o 0 0 0 10 0 0 10 10 0");
        Assert.Equal("", aDI.Eval("mkoffset r o 1 1"));
        Assert.Contains("1", aDI.Eval("nbshapes r_1"));
        Assert.Equal("", aDI.Eval("openoffset r o 1 1"));
        Assert.Contains("This shape seems to be valid", aDI.Eval("checkshape r_1"));
        // too far inside: done, an empty compound (as natively)
        Assert.Equal("", aDI.Eval("mkoffset r f 1 -6"));
        Assert.Equal("", aDI.Eval("explode r_1 e"));
        // too few arguments, a missing shape
        Assert.ThrowsAny<Exception>(() => aDI.Eval("mkoffset r f 1"));
        Assert.ThrowsAny<Exception>(() => aDI.Eval("openoffset r nothing 1 1"));
    }

    private static IEnumerable<DrawCaseScan> UsingOffset
        => SharedCorpus.Scans.Where(s => s.Commands.Overlaps(Unit5O2Commands) && (s.Case.Group != "offset" || IsMkoffsetCase(s.Case)));

    // none fails: OK or BAD, or skipped for missing data files
    private static void Check(DrawCaseRun theRun)
        => Assert.True(theRun.Passed || theRun.Status.StartsWith("SKIPPED", StringComparison.Ordinal), $"{theRun.Case.Id}: {theRun.Status}");

    /// <summary>The cases the corpus test runs: those using the commands, runnable after 5O2; a row each in DrawOffsetCommandsShard*Tests.</summary>
    public static CorpusPlan Corpus { get; } = new(14, () => UsingOffset.Where(s => s.IsRunnable).Select(s => s.Case), Check);

    [Fact]
    public void Corpus_cases_using_mkoffset_or_openoffset_before_and_after_5O2()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpusBefore = new DrawCorpus(TestsDirectory) { KnowsValueBranches = false };
        DrawInterpreter aBefore = NewInterpreter();
        Before5O2Offset(aBefore);

        List<DrawCaseScan> aScans = [.. UsingOffset];
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = SharedCorpus.RunnableWith(aScans.Select(s => s.Case), aBefore, aCorpusBefore);
        int anOffsetGroup = aScans.Count(s => s.Case.Group == "offset");
        int anOffsetRunnable = aRunnable.Count(c => c.Group == "offset");
        output.WriteLine($"{aScans.Count} cases use mkoffset or openoffset ({anOffsetGroup} of the offset group); "
                         + $"runnable before 5O2: {aRunnableBefore.Count}, after: {aRunnable.Count} ({anOffsetRunnable} of the offset group)");
        foreach (IGrouping<string, string> aMissing in aScans.Where(s => !s.IsRunnable).SelectMany(s => s.Missing)
                                                         .GroupBy(m => m).OrderByDescending(g => g.Count()))
        {
            output.WriteLine($"  missing {aMissing.Key}: {aMissing.Count()} cases");
        }

        Assert.Equal(aRunnable, Corpus.Cases);
        var aRuns = SharedCorpus.RunAll(aRunnable)
                                .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                                .ToList();
        output.WriteLine("after 5O2: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
                                                                 .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                                 .Select(g => $"{g.Key} {g.Count()}")));
        foreach (DrawCaseRun aRun in aRuns.Where(r => !r.Passed))
        {
            string[] aLines = aRun.Log.Split('\n');
            string? aLine = aLines.FirstOrDefault(l => l.Contains("rror", StringComparison.Ordinal)
                                                       || l.Contains("xception", StringComparison.Ordinal)
                                                       || l.Contains("Faulty", StringComparison.Ordinal)
                                                       || l.Contains("NotImplemented", StringComparison.Ordinal));
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}");
            output.WriteLine($"      {aLine ?? aLines.Reverse().Skip(1).FirstOrDefault()}");
        }

        Assert.All(aScans, s => Assert.DoesNotContain("mkoffset", s.Missing));
        Assert.All(aScans, s => Assert.DoesNotContain("openoffset", s.Missing));
        // unit 5O2: 0 runnable before, 626 after (621 of the offset group's mkoffset grids); none fails (OK or BAD, the
        // rest skipped for missing data files)
        Assert.Empty(aRunnableBefore);
        Assert.True(anOffsetRunnable >= 600, $"{anOffsetRunnable}");
        Assert.All(aRuns, Check);
    }
}
