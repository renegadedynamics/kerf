// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 7Q2's hidden-line removal commands: HLRTest's (HLRTest.cxx: <c>hprj</c>, <c>houtl</c>, <c>hfill</c>,
/// <c>hsin</c>, <c>hsout</c>, <c>hload</c>, <c>hremove</c>, <c>hsetprj</c>, <c>hupdate</c>, <c>hhide</c>,
/// <c>hshowall</c>, <c>hdebug</c>, <c>hnullify</c>, <c>hres2d</c>, <c>reflectlines</c>, <c>hlrin3d</c>,
/// <c>hlrin2d</c>), BRepTest's <c>build3d</c>, ViewerTest's <c>vcomputehlr</c> and view projections (<c>vfront</c>,
/// <c>vback</c>, <c>vbottom</c>, <c>vleft</c>, <c>vright</c>; <c>vtop</c> and <c>vaxo</c> were no-ops before), and
/// Draw's view commands as no-ops (<c>top</c>, <c>axo</c>, <c>front</c>, <c>v2d</c>), on hand-made shapes and on
/// OCCT's corpus: the cases that use them, runnable after the commands and not before, run as OCCT's harness runs
/// them and judged by OCCT's log check.
/// </summary>
public class DrawHlrCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 7Q2 added.</summary>
    internal static readonly string[] Unit7Q2Commands =
    [
        "hprj", "houtl", "hfill", "hsin", "hsout", "hload", "hremove", "hsetprj", "hupdate", "hhide", "hshowall", "hdebug",
        "hnullify", "hres2d", "reflectlines", "hlrin3d", "hlrin2d", "build3d", "vcomputehlr", "vfront", "vback",
        "vbottom", "vleft", "vright", "top", "axo", "front", "v2d",
    ];

    /// <summary>Draw-lite before unit 7Q2: without its commands.</summary>
    internal static void Before7Q2(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit7Q2Commands)
        {
            theInterpreter.Eval($"rename {aCommand} {{}}");
        }
    }

    private static DrawInterpreter NewInterpreter(TextWriter? theOutput = null)
    {
        var anInterpreter = new DrawInterpreter(theOutput);
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    [Fact]
    public void The_HLRTest_commands_hide_as_Draw()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("box b 10 20 30");
        aDI.Eval("psphere s 8");
        aDI.Eval("ttranslate s 5 10 40");
        Assert.Equal("", aDI.Eval("hprj p 11 0 0 0 1 -1 1 -1 1 2"));
        Assert.Contains("projector", aDI.Eval("whatis p"), StringComparison.Ordinal);
        aDI.Eval("houtl ob b");
        aDI.Eval("houtl os s");
        Assert.Contains("outliner", aDI.Eval("whatis ob"), StringComparison.Ordinal);
        aDI.Eval("hload ob");
        aDI.Eval("hload os");
        aDI.Eval("hsetprj p");
        aDI.Eval("hupdate");
        aDI.Eval("hhide");
        aDI.Eval("hres2d");
        // the box's twelve edges: nine visible, three hidden (in part by the sphere)
        Assert.Contains("EDGE", aDI.Eval("nbshapes vl"), StringComparison.Ordinal);
        Assert.Contains("EDGE", aDI.Eval("nbshapes hl"), StringComparison.Ordinal);
        Assert.Contains("EDGE", aDI.Eval("nbshapes vol"), StringComparison.Ordinal); // the sphere's outline
        // one shape's results
        aDI.Eval("hres2d b");
        Assert.Equal("ob removed\n", aDI.Eval("hremove ob"));
        Assert.Equal(" all shapes removed\n", aDI.Eval("hremove"));
        // hfill/hsout: the outlined shape, frozen, and the original unchanged
        aDI.Eval("hfill os p 2");
        aDI.Eval("hsout so os");
        aDI.Eval("hsin si os");
        Assert.Contains("FACE", aDI.Eval("nbshapes so"), StringComparison.Ordinal);
        Assert.Equal(aDI.Eval("nbshapes s").Split((char)10)[1..], aDI.Eval("nbshapes si").Split((char)10)[1..]);
        Assert.Equal("debug\n", aDI.Eval("hdebug"));
        Assert.Equal("no debug\n", aDI.Eval("hdebug"));
        // errors as Draw's
        Assert.Contains("is not an OutLiner", Assert.ThrowsAny<Exception>(() => aDI.Eval("hload b")).Message, StringComparison.Ordinal);
        Assert.Contains("is not a projector", Assert.ThrowsAny<Exception>(() => aDI.Eval("hsetprj b")).Message, StringComparison.Ordinal);
    }

    [Fact]
    public void The_reflect_lines_and_vcomputehlr_as_Draw()
    {
        StringWriter anOutput = new();
        DrawInterpreter aDI = NewInterpreter(anOutput);
        aDI.Eval("pcylinder c 10 30");
        aDI.Eval("reflectlines r c 1 -1 1");
        aDI.Eval("hlrin3d r3 c 1 -1 1");
        aDI.Eval("hlrin2d r2 c 1 -1 1 1 1 0");
        Assert.Contains("EDGE", aDI.Eval("nbshapes r"), StringComparison.Ordinal);
        Assert.Contains("EDGE", aDI.Eval("nbshapes r3"), StringComparison.Ordinal);
        Assert.Contains("EDGE", aDI.Eval("nbshapes r2"), StringComparison.Ordinal);

        // without a viewer, vcomputehlr needs a view direction
        Assert.ThrowsAny<Exception>(() => aDI.Eval("vcomputehlr c h -algoType algo"));
        Assert.Contains("Syntax error: wrong number of arguments", anOutput.ToString(), StringComparison.Ordinal);
        aDI.Eval("vcomputehlr c h1 0 0 0 0 0 1 0 1 0 -algoType algo");
        aDI.Eval("vinit View1");
        aDI.Eval("vdisplay c");
        aDI.Eval("vtop");
        aDI.Eval("vcomputehlr c h2 -algoType algo");
        // the same view: the same edges (the projector's location differs, along the view direction only)
        Assert.Equal(aDI.Eval("nbshapes h1").Split((char)10)[1..], aDI.Eval("nbshapes h2").Split((char)10)[1..]);
        aDI.Eval("build3d h2");
        aDI.Eval("vright");
        aDI.Eval("vcomputehlr c h3 -algoType polyAlgo -showHiddenEdges");
        Assert.Contains("EDGE", aDI.Eval("nbshapes h3"), StringComparison.Ordinal);
        // the top view of the cylinder is its circle (2 pi 10)
        Assert.Matches(@"Mass :\s+62\.83", aDI.Eval("lprops h2"));
    }

    [Fact]
    public void Corpus_cases_using_the_HLR_commands_run_after_7Q2()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before7Q2(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => Unit7Q2Commands.Any(s.Commands.Contains))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        string? aFilter = Environment.GetEnvironmentVariable("OCCTNET_7Q2_FILTER");
        string? aLogDir = Environment.GetEnvironmentVariable("OCCTNET_7Q2_LOG");
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use the HLR commands; runnable before 7Q2: {aRunnableBefore.Count}, after: {aRunnable.Count}");

        var aRuns = aRunnable.Where(c => aFilter is null || c.Id.Contains(aFilter, StringComparison.Ordinal)).AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                              TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 7Q2: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
                                                                .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                                .Select(g => $"{g.Key} {g.Count()}")));
        foreach (DrawCaseRun aRun in aRuns)
        {
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}");
            if (!aRun.Passed)
            {
                if (aLogDir is not null)
                {
                    File.WriteAllText(Path.Combine(aLogDir, aRun.Case.Id.Replace('/', '_') + ".log"), aRun.Log);
                }
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

        if (aFilter is not null)
        {
            return;
        }
        Assert.Empty(aRunnableBefore);
        Assert.Equal(332, aScans.Count);
        Assert.Equal(188, aRunnable.Count);
        // OK 44, BAD 5 (OCCT's TODOs), FAILED 2 (attributed), SKIPPED 137 (data not public); the 144 others wait for the
        // viewer's other commands, STEP and QA commands
        Assert.Equal(44, aRuns.Count(r => r.Status == "OK"));
        Assert.Equal(137, aRuns.Count(r => r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)));
        Assert.All(aRuns, r => Assert.True(r.Status == "OK" || r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)
                                           || r.Status.StartsWith("BAD", StringComparison.Ordinal) || KnownFailures.Contains(r.Case.Id),
                                           $"{r.Case.Id}: {r.Status}"));
    }

    /// <summary>Cases that fail in Draw-lite, each attributed (docs/occt-issues.md, unit 7Q2).</summary>
    private static readonly HashSet<string> KnownFailures = new(StringComparer.Ordinal)
    {
        // The polygonal HLR of the default view: OCCT's TODOs give Windows 404.004 and 301.999, Linux 404.386 and 302,
        // the port 404.007 and 301.996. The result moves with the last bits of the view's projector, which DRAWEXE
        // derives from the camera's matrices (V3d_View::ConvertWithProj) and Draw-lite from its axis: with the
        // same frame given explicitly, the torus gives Windows' 301.999 (decision 1131)
        "hlr/poly_hlr/Plate",
        "hlr/poly_hlr/bug25813_3",
    };
}
