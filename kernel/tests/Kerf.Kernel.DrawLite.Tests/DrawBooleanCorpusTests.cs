// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 6L's Boolean commands (BOPTest's BOPCommands, PartitionCommands, APICommands, OptionCommands' <c>bsimplify</c>,
/// UtilityCommands, TolerCommands and DebugCommands) on OCCT's corpus: the cases that use them, runnable before and
/// after the commands, run as OCCT's harness runs them and judged by OCCT's log check.
/// </summary>
/// <remarks>
/// The corpus has about 4,400 runnable cases using these commands (3.9 minutes in a Debug build on 12 workers), and
/// <see cref="DrawCorpusTests"/> already runs every runnable case, so the default run here takes every
/// <see cref="SampleStep"/>-th of them (by case id, so the sample is the same on every machine) and asserts that none
/// fails unattributed; <c>KERF_CORPUS_FULL=1</c> runs them all.
/// <c>KERF_CORPUS_LOGS=dir</c> writes the log of every case that doesn't pass into that directory.
/// </remarks>
[Collection(MeshDefaultsCollection.Name)]
public class DrawBooleanCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The share of the cases the default run takes.</summary>
    private const int SampleStep = 4;

    /// <summary>The Draw commands of BOPTest that unit 6L added.</summary>
    internal static readonly string[] Unit6LBopCommands =
    [
        "bop", "bopcommon", "bopfuse", "bopcut", "boptuc", "bopsection", "bcommon", "bfuse", "bcut", "btuc", "bsection",
        "bopcurves", "bbuild", "bbop", "bsplit", "buildbop", "bapibuild", "bapibop", "bapisplit", "bsimplify",
        "attachpcurve", "edgestowire", "edgestofaces", "buildpcurvesonplane", "breducetolerance", "btolx", "bopaddpcs",
        "bopds", "bopiterator", "bopinterf", "bopnews", "bopwho", "bopindex", "bopsd", "bopsc", "boppb", "bopcb", "bopsp",
        "bopfon", "bopfin", "bopfsc", "bopfav", "bopimage", "boporigin", "bopfsd", "bopbsolid", "bopbface",
    ];

    /// <summary>Draw-lite before unit 6L's Boolean commands.</summary>
    internal static void Before6LBop(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit6LBopCommands)
        {
            if (theInterpreter.HasCommand(aCommand))
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
    public void Corpus_cases_using_the_Boolean_commands_run_before_and_after_6L()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before6LBop(aBefore);
        DrawHealCorpusTests.Before6LHeal(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => Unit6LBopCommands.Any(s.Commands.Contains))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use the Boolean commands; runnable before 6L: {aRunnableBefore.Count}, after: {aRunnable.Count}");

        bool isFull = Environment.GetEnvironmentVariable("KERF_CORPUS_FULL") == "1";
        List<DrawTestCase> aToRun = isFull
            ? aRunnable
            : aRunnable.OrderBy(c => c.Id, StringComparer.Ordinal).Where((_, i) => i % SampleStep == 0).ToList();
        // The cases that measure memory run one at a time, in DrawCheapGapsCorpusTests only (left out after sampling,
        // so the sample stays the same)
        aToRun = aToRun.Where(c => !DrawCheapGapsCorpusTests.MeasuresMemory(aScans.First(s => s.Case == c))).ToList();
        output.WriteLine(isFull ? "running all of them" : $"running every {SampleStep}th: {aToRun.Count} (KERF_CORPUS_FULL=1 runs all)");

        var aRuns = aToRun.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                          .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                           TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                          .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                          .ToList();
        output.WriteLine("after 6L: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
                                                              .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                              .Select(g => $"{g.Key} {g.Count()}")));
        string? aLogs = Environment.GetEnvironmentVariable("KERF_CORPUS_LOGS");
        foreach (DrawCaseRun aRun in aRuns)
        {
            if (aRun.Status == "OK")
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
                if (aLogs is not null)
                {
                    Directory.CreateDirectory(aLogs);
                    File.WriteAllText(Path.Combine(aLogs, aRun.Case.Id.Replace('/', '_') + ".log"), aRun.Log);
                }
            }
        }

        output.WriteLine("Commands the other cases using the Boolean commands miss (cases):");
        foreach (var aGroup in aScans.SelectMany(s => s.Missing).GroupBy(c => c).OrderByDescending(g => g.Count()).Take(15))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        Assert.True(aRunnable.Count > aRunnableBefore.Count + 3000);
        Assert.Contains(aRuns, r => r.Status == "OK");
        // and unit 7W's attributed failures among the cases using the Boolean commands (bugs/modalg_7/bug21264: progress)
        Assert.DoesNotContain(aRuns, r => r.Status.StartsWith("FAILED", StringComparison.Ordinal) && !KnownFailures.Contains(r.Case.Id)
                                          && !DrawCheapGapsCorpusTests.KnownFailures.ContainsKey(r.Case.Id)
                                          && !DrawFilletCorpusTests.KnownFailures.ContainsKey(r.Case.Id));
    }

    /// <summary>The corpus cases that fail for an attributed reason (see docs/occt-issues.md, unit 6L).</summary>
    internal static readonly HashSet<string> KnownFailures = new(StringComparer.Ordinal)
    {
        // DRAWEXE's Booleans are destructive by default; Draw's variables hold frozen shapes, so the port's are not
        // (decision 577): mkvolume copies the argument face part_2 instead of using it, the script's `compare part_2`
        // finds no solid holding it, and the cut runs on the wrong solid
        "boolean/bcut_complex/F2",
        // native OCCT (Kerf.Modeling's build) gives the same 6 vertices on the port's bcommon result and on its own
        "bugs/heal/bug29382_3",

        // The cases FollowsCalledProcsOnly made runnable (decision 972), attributed by unit 7W:
        // decision 955 again: the section of the sewn shell by the plane y = 1 is the shell's common edge itself in
        // DRAWEXE's destructive mode; non-destructively native OCCT (Kerf.Modeling's build) gives the port's result, a
        // copy of the edge (8 vertices and 8 edges in the compound of the section and the shell where the script wants
        // 6 and 7)
        "bugs/modalg_5/bug23881",
        // native OCCT (Kerf.Modeling's build) gives the same warning on the translated shape: IntPatch_ImpImpIntersection
        // of the sphere and a cone throws Standard_NumericError in math_FunctionAllRoots, which bopcheck reports as
        // "Intersection of pair of shapes has failed"; OCCT's default Release build compiles that check out (No_Exception),
        // the port and Kerf.Modeling's build keep it (decision 9)
        "bugs/modalg_7/bug26034",
        // decision 955 again: the general fuse of s1_sp and s_2 is non-destructive in the port, and native OCCT
        // non-destructively gives the same counts (22 vertices, 41 edges, 5 solids in the compound of the result and
        // s1_sp); DRAWEXE's destructive default keeps s1_sp's sub-shapes, as the script expects (16, 28, 3)
        "bugs/modalg_7/bug29333_2",
        // the same for its twin on two faces (8 vertices, 10 edges, 3 faces destructively; 11, 15, 5 non-destructively,
        // natively as in the port); in the default run's sample since unit 7W's commands made more cases runnable
        "bugs/modalg_7/bug29333_1",

        // Attributed by unit 7W's follow-up (decisions 990 on):
        // decision 955 again: the general fuse of the wire with the vertices v0 and v1 as tools; native OCCT
        // (Kerf.Modeling's BOPAlgo_Builder on the case's files) reports each vertex modified once non-destructively, as
        // the port does, and not modified destructively, which the script expects ("The shape has not been modified.")
        "bugs/modalg_6/bug26393",
        // speed: the script wants each section within 0.1 s. The port takes 0.34 s for each on a cold start (the JIT) and
        // 0.02 s and 0.20 s warm in Release; native (Kerf.Modeling's BRepAlgoAPI_Section, the same options) takes 0.03 s
        // for the second: a performance gap of about 6x on these B-spline sheets (occt-issues.md, unit 7W follow-up)
        "bugs/modalg_7/bug83",
    };
}
