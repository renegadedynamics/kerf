// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 6G's <c>mkvolume</c> (BOPAlgo_MakerVolume) and cells commands (<c>bcbuild</c>, <c>bcadd</c>, ...,
/// BOPAlgo_CellsBuilder), and the corpus cases they make runnable, before and after them. With the environment
/// variable OCCTNET_BOREC_CAPTURE naming a directory, the corpus calls are captured there for tools/borec
/// (volume.capture.txt).
/// </summary>
public class DrawVolumeCellsCommandsTests(ITestOutputHelper output)
{
    private static readonly string[] theCommands =
        ["mkvolume", "bcbuild", "bcaddall", "bcremoveall", "bcadd", "bcremove", "bcremoveint", "bcmakecontainers"];

    private static DrawInterpreter Load()
    {
        var anInterpreter = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    [Fact]
    public void Mkvolume_makes_solids_of_faces_with_its_options()
    {
        DrawInterpreter i = Load();
        i.Eval("box b1 10 10 10; box b2 5 5 5 10 10 10; explode b1 f; explode b2 f");
        i.Eval(("mkvolume r b1_1 b1_2 b1_3 b1_4 b1_5 b1_6 b2_1 b2_2 b2_3 b2_4 b2_5 b2_6"));
        Assert.Equal(3, Count(i.GetShape("r"), ShapeType.Solid));
        i.Eval("compound b1_1 b1_2 b1_3 b1_4 b1_5 b1_6 c1; mkvolume r c1 -c");
        Assert.Equal(1, Count(i.GetShape("r"), ShapeType.Solid));
        i.Eval("mkvolume r b1 -ni");
        Assert.Equal(1, Count(i.GetShape("r"), ShapeType.Solid));
        i.Eval("savehistory h");
        Assert.Equal("history", i.Eval("whatis h").Split(' ').Last().Trim());
        Assert.Contains("Usage: mkvolume", Assert.ThrowsAny<DrawException>(() => i.Eval("mkvolume r")).Reason);
    }

    [Fact]
    public void Cells_commands_build_add_remove_and_unify()
    {
        DrawInterpreter i = Load();
        i.Eval("box b1 10 10 10; box b2 5 5 5 10 10 10");
        i.Eval("bclearobjects; bcleartools; baddobjects b1 b2; bfillds; bcbuild r");
        Assert.Equal(3, Count(i.GetShape("r"), ShapeType.Solid));
        i.Eval("bcadd res b1 1 b2 0");
        Assert.Equal(1, Count(i.GetShape("res"), ShapeType.Solid));
        i.Eval("bcadd res b2 1 -m 1");
        Assert.Equal(3, Count(i.GetShape("res"), ShapeType.Solid));
        i.Eval("bcremove res b1 1 b2 0");
        Assert.Equal(2, Count(i.GetShape("res"), ShapeType.Solid));
        i.Eval("bcremoveall; bcaddall res -m 1 -u");
        Assert.Equal(1, Count(i.GetShape("res"), ShapeType.Solid));
        i.Eval("bcremoveall; bcaddall res; bcmakecontainers res");
        Assert.Equal(1, Count(i.GetShape("res"), ShapeType.CompSolid));
        i.Eval("savehistory h; modified m h b1");
        Assert.Equal(2, Count(i.GetShape("m"), ShapeType.Solid));
    }

    /// <summary>Draw-lite before unit 6G's volume and cells commands.</summary>
    internal static void Before(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in theCommands)
        {
            theInterpreter.Eval($"rename {aCommand} {{}}");
        }
    }

    /// <summary>
    /// The corpus cases that use mkvolume or the bc* commands: how many are runnable before and after, and how the
    /// newly runnable ones end under OCCT's _check_log.
    /// </summary>
    [Fact]
    public void Volume_and_cells_cases_of_the_corpus_run()
    {
        string aTests = Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");
        var aCorpus = new DrawCorpus(aTests);
        DrawInterpreter anAfter = Load();
        DrawInterpreter aBefore = Load();
        Before(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered().Select(c => aCorpus.Scan(c, anAfter))
                                           .Where(s => theCommands.Any(s.Commands.Contains)).ToList();
        // The cases that measure memory run one at a time, in DrawCheapGapsCorpusTests only
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable && !DrawCheapGapsCorpusTests.MeasuresMemory(s)).Select(s => s.Case).ToList();
        int aRunnableBefore = aScans.Count(s => aCorpus.Scan(s.Case, aBefore).IsRunnable);
        output.WriteLine($"{aScans.Count} cases use mkvolume or bc*: {aRunnableBefore} runnable before, {aRunnable.Count} after");
        // the cases that also need unit 6L's Boolean commands (the boolean grids' mkvolume cases) are run by
        // DrawBooleanCorpusTests and DrawCorpusTests; this test keeps to those 6G's commands made runnable
        DrawInterpreter aWithout6L = Load();
        DrawBooleanCorpusTests.Before6LBop(aWithout6L);
        int aWith6L = aRunnable.Count;
        aRunnable = aRunnable.Where(c => aCorpus.Scan(c, aWithout6L).IsRunnable).ToList();
        output.WriteLine($"  of which {aWith6L - aRunnable.Count} need 6L's Boolean commands too; running the other {aRunnable.Count}");
        foreach (var aGroup in aScans.Where(s => !s.IsRunnable).SelectMany(s => s.Missing).GroupBy(c => c)
                                     .OrderByDescending(g => g.Count()).Take(12))
        {
            output.WriteLine($"  still missing {aGroup.Key,-20} {aGroup.Count()} cases");
        }

        List<DrawCaseRun> aRuns = aRunnable.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                                           .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory,
                                                                            DrawCorpusTests.DataPath, TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                                           .OrderBy(r => r.Case.Id, StringComparer.Ordinal).ToList();
        output.WriteLine("statuses: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0]).OrderBy(g => g.Key, StringComparer.Ordinal)
                                                                  .Select(g => $"{g.Key} {g.Count()}")));
        foreach (DrawCaseRun aRun in aRuns)
        {
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}");
        }

        Assert.True(aRunnable.Count > aRunnableBefore);
        Assert.DoesNotContain(aRuns, r => r.Status.StartsWith("FAILED", StringComparison.Ordinal) && !KnownFailures.Contains(r.Case.Id));
    }

    /// <summary>Cases that fail with an attributed cause (see the unit's decisions).</summary>
    private static readonly HashSet<string> KnownFailures = new(StringComparer.Ordinal);

    /// <summary>Writes the capture of the corpus calls for tools/borec when OCCTNET_BOREC_CAPTURE is set.</summary>
    [Fact]
    public void Capture_for_borec()
    {
        string? aDir = Environment.GetEnvironmentVariable("OCCTNET_BOREC_CAPTURE");
        if (string.IsNullOrEmpty(aDir))
        {
            return;
        }
        int aNb = DrawCapture.CaptureGrids(Path.Combine(aDir, "volume.capture.txt"),
                                           [.. theCommands, "baddobjects", "baddtools", "bclearobjects", "bcleartools", "bfillds",
                                            "baddcompound", "baddctools", "bclear", "bfuzzyvalue", "bnondestructive", "bglue", "brunparallel"],
                                           "volumemaker", "cells_test");
        output.WriteLine($"{aNb} calls captured");
        Assert.True(aNb > 0);
    }
}
