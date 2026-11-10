// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 6G's periodicity and connection commands (BOPTest_PeriodicityCommands.cxx, BOPTest_MkConnectedCommands.cxx),
/// and the corpus cases they make runnable; the capture of those cases for tools/borec.
/// </summary>
public class DrawPeriodicityCommandsTests(ITestOutputHelper output) : ICorpusTest
{
    /// <summary>The commands of the two files.</summary>
    internal static readonly string[] Commands =
    [
        "makeperiodic", "periodictwins", "repeatshape", "clearrepetitions",
        "makeconnected", "cmaterialson", "cmakeperiodic", "crepeatshape", "cperiodictwins", "cclearrepetitions",
    ];

    private static DrawInterpreter Load()
    {
        var anInterpreter = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    /// <summary>Draw-lite before these commands.</summary>
    internal static void Before(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Commands)
        {
            theInterpreter.Eval($"rename {aCommand} {{}}");
        }
    }

    private static string NbShapes(DrawInterpreter i, string theName) => i.Eval($"nbshapes {theName} -t");

    [Fact]
    public void Makeperiodic_repeatshape_and_twins_work_with_the_history()
    {
        DrawInterpreter i = Load();
        i.Eval("box b 10 10 10");
        i.Eval("makeperiodic bp b -x 5 -trim 2 -y 8 -trim 1 -z 12 -trim -1");
        Assert.Contains("SOLID     : 1", NbShapes(i, "bp"), StringComparison.Ordinal);
        Assert.Contains("FACE      : 6", NbShapes(i, "bp"), StringComparison.Ordinal);
        i.Eval("checkshape bp");

        i.Eval("repeatshape result -x 5 -y 5 -z 5");
        Assert.Contains("SOLID     : 216", NbShapes(i, "result"), StringComparison.Ordinal);
        i.Eval("clearrepetitions r");
        i.Eval("explode bp f");
        i.Eval("periodictwins t bp_2");
        Assert.False(i.GetShape("t").IsNull);
        i.Eval("savehistory h");
        Assert.Equal("Not deleted.", i.Eval("isdeleted h b"));

        Assert.Contains("Invalid key", Assert.ThrowsAny<DrawException>(() => i.Eval("makeperiodic r b -w 5 -x 4")).Reason, StringComparison.Ordinal);
        Assert.Contains("Period for x direction is not set", Assert.ThrowsAny<DrawException>(() => i.Eval("makeperiodic r b -y 4 -x")).Reason, StringComparison.Ordinal);
        Assert.Contains("Number of repetitions for y", Assert.ThrowsAny<DrawException>(() => i.Eval("repeatshape r -y 0")).Reason, StringComparison.Ordinal);
    }

    [Fact]
    public void Makeconnected_materials_and_periodicity_work()
    {
        DrawInterpreter i = Load();
        i.Eval("box b1 10 10 10");
        i.Eval("box b2 10 0 0 10 10 10");
        Assert.Contains("Make the shapes connected first", Assert.ThrowsAny<DrawException>(() => i.Eval("cmakeperiodic r -x 20")).Reason, StringComparison.Ordinal);
        i.Eval("makeconnected c b1 b2");
        Assert.Contains("FACE      : 11", NbShapes(i, "c"), StringComparison.Ordinal);
        i.Eval("explode b1 f");
        // the face of b1 at x = 10 is shared: b1 on one side, b2 on the other
        i.Eval("savehistory h0");
        i.Eval("if {[regexp \"not been modified\" [modified f h0 b1_2]]} {copy b1_2 f}");
        i.Eval("cmaterialson mp + f");
        i.Eval("cmaterialson mn - f");
        Assert.False(i.GetShape("mp").IsNull);
        Assert.False(i.GetShape("mn").IsNull);
        i.Eval("cmakeperiodic cp -x 20 -y 10");
        i.Eval("crepeatshape cr -x 1");
        Assert.Contains("SOLID     : 4", NbShapes(i, "cr"), StringComparison.Ordinal);
        i.Eval("cclearrepetitions cc");
        Assert.Contains("SOLID     : 2", NbShapes(i, "cc"), StringComparison.Ordinal);
        i.Eval("savehistory h");
        i.Eval("modified m h b1");
        Assert.False(i.GetShape("m").IsNull);
    }

    /// <summary>The cases the corpus test runs, a row each in DrawPeriodicityCommandsShard*Tests (decision 27801).</summary>
    public static CorpusPlan Corpus { get; } = new(2, () => SharedCorpus.Scans.Where(s => s.Commands.Overlaps(Commands)).Where(s => s.IsRunnable).Select(s => s.Case));

    /// <summary>
    /// The corpus cases using these commands: how many are runnable before and after them, and how the runnable ones
    /// end under OCCT's _check_log.
    /// </summary>
    [Fact]
    public void Periodicity_and_connection_cases_of_the_corpus_run()
    {
        string aTests = Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");
        DrawInterpreter aBefore = Load();
        Before(aBefore);

        List<DrawCaseScan> aScans = [.. SharedCorpus.Scans.Where(s => s.Commands.Overlaps(Commands))];
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        int aRunnableBefore = SharedCorpus.RunnableWith(aScans.Select(s => s.Case), aBefore).Count;
        output.WriteLine($"{aScans.Count} cases use the commands: {aRunnableBefore} runnable before, {aRunnable.Count} after");
        foreach (var aGroup in aScans.Where(s => !s.IsRunnable).SelectMany(s => s.Missing).GroupBy(c => c)
                                     .OrderByDescending(g => g.Count()))
        {
            output.WriteLine($"  still missing {aGroup.Key,-20} {aGroup.Count()} cases");
        }

        Assert.Equal(aRunnable, Corpus.Cases);
        List<DrawCaseRun> aRuns = SharedCorpus.RunAll(aRunnable)
                                           .OrderBy(r => r.Case.Id, StringComparer.Ordinal).ToList();
        output.WriteLine("statuses: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0]).OrderBy(g => g.Key, StringComparer.Ordinal)
                                                                  .Select(g => $"{g.Key} {g.Count()}")));
        foreach (DrawCaseRun aRun in aRuns)
        {
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}");
            if (!aRun.Passed)
            {
                output.WriteLine(aRun.Log);
            }
        }

        Assert.True(aRunnable.Count > aRunnableBefore);
        Assert.DoesNotContain(aRuns, r => r.Status.StartsWith("FAILED", StringComparison.Ordinal));
    }

    /// <summary>
    /// Writes the capture of the periodicity and mkconnected grids for tools/borec (connected area) when
    /// KERF_BOREC_CAPTURE names a directory; otherwise does nothing.
    /// </summary>
    [Fact]
    public void Capture_for_borec()
    {
        string? aDir = Environment.GetEnvironmentVariable("KERF_BOREC_CAPTURE");
        if (string.IsNullOrEmpty(aDir))
        {
            return;
        }
        Directory.CreateDirectory(aDir);
        int aNb = DrawCapture.CaptureGrids(Path.Combine(aDir, "connected.capture"), Commands, "periodicity", "mkconnected");
        output.WriteLine($"{aNb} calls captured");
        Assert.True(aNb > 0);
    }
}
