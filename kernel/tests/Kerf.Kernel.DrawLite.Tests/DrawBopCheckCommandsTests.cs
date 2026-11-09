// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 6G's check commands (BOPTest_CheckCommands.cxx: <c>bopcheck</c>, <c>bopargcheck</c>, <c>bopapicheck</c>,
/// <c>xdistef</c>, <c>checkcurveonsurf</c>), and the corpus cases they make runnable.
/// </summary>
public class DrawBopCheckCommandsTests(ITestOutputHelper output) : ICorpusTest
{
    internal static readonly string[] Commands = ["bopcheck", "bopargcheck", "bopapicheck", "xdistef", "checkcurveonsurf"];

    private static DrawInterpreter Load()
    {
        var anInterpreter = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    [Fact]
    public void Bopcheck_lists_the_interferences_of_a_compound()
    {
        DrawInterpreter i = Load();
        i.Eval("box b1 10 10 10; box b2 5 5 5 10 10 10; compound b1 b2 c");
        Assert.Equal(" This shape seems to be OK.\n", i.Eval("bopcheck b1"));
        string aText = i.Eval("bopcheck c");
        Assert.Contains("F/F: ", aText, StringComparison.Ordinal);
        Assert.Contains("E/F: ", aText, StringComparison.Ordinal);
        Assert.DoesNotContain("seems to be OK", aText, StringComparison.Ordinal);
        Assert.StartsWith("Info:\nThe level of check is set to V/V, i.e. intersection(s)\nV/E, E/E, V/F, E/F, F/F, V/Z, E/Z, F/Z, Z/Z will not be checked.\n\n",
                          i.Eval("bopcheck c 0"), StringComparison.Ordinal);
        Assert.Contains("Invalid level", Assert.ThrowsAny<DrawException>(() => i.Eval("bopcheck c 12")).Reason);
    }

    [Fact]
    public void Bopargcheck_and_bopapicheck_report_the_faults()
    {
        DrawInterpreter i = Load();
        i.Eval("box b1 10 10 10; box b2 5 5 5 10 10 10; compound b1 b2 c; vertex v 0 0 0");
        Assert.Equal("Shape(s) seem(s) to be valid for BOP.\n", i.Eval("bopargcheck b1"));
        Assert.Equal("Shape(s) seem(s) to be valid for BOP.\n", i.Eval("bopargcheck b1 b2 -F"));
        Assert.Equal("Faulties, that can not be treated by BOP, are detected.\n", i.Eval("bopargcheck c"));
        string aFull = i.Eval("bopargcheck c #F");
        Assert.StartsWith("Made faulty shape: s1si_1\n", aFull, StringComparison.Ordinal);
        Assert.Contains("Faulties for FIRST  shape found : ", aFull, StringComparison.Ordinal);
        Assert.Contains("Self-Intersections              : YES  Cases(", aFull, StringComparison.Ordinal);
        Assert.Contains("Faulties for SECOND  shape found : 0\n", aFull, StringComparison.Ordinal);
        Assert.Equal("Faulties, that can not be treated by BOP, are detected.\n", i.Eval("bopargcheck b1 v -F"));

        Assert.Equal("The shape seems to be valid\n", i.Eval("bopapicheck b1"));
        Assert.Equal("The shapes seem to be valid\n", i.Eval("bopapicheck b1 b2 -op fuse"));
        Assert.Equal("The shape is invalid\n", i.Eval("bopapicheck c"));
        Assert.Equal("The shapes are not valid for Boolean operation\n", i.Eval("bopapicheck b1 v -op fuse"));
        Assert.Equal("Invalid key: -x. Skipped.\nThe shape seems to be valid\n", i.Eval("bopapicheck b1 -x"));
    }

    [Fact]
    public void Xdistef_and_checkcurveonsurf_measure_curves_on_surfaces()
    {
        DrawInterpreter i = Load();
        i.Eval("box b 10 10 10; explode b f; explode b_1 e");
        Assert.StartsWith("Max Distance = ", i.Eval("xdistef b_1_1 b_1"), StringComparison.Ordinal);
        Assert.Equal("This shape seems to be OK.\n", i.Eval("checkcurveonsurf b"));
        Assert.Contains("type mismatch", Assert.ThrowsAny<DrawException>(() => i.Eval("xdistef b_1 b_1")).Reason);
    }

    /// <summary>Draw-lite before unit 6G's check commands.</summary>
    internal static void Before6G(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Commands)
        {
            theInterpreter.Eval($"rename {aCommand} {{}}");
        }
    }

    private static IEnumerable<DrawCaseScan> UsingCheck => SharedCorpus.Scans.Where(s => Commands.Any(s.Commands.Contains));

    // the runnable cases; those that measure memory are DrawCheapGapsCorpusTests'
    private static List<DrawTestCase> ToRun(IEnumerable<DrawCaseScan> theScans)
        => theScans.Where(s => s.IsRunnable && !DrawCheapGapsCorpusTests.MeasuresMemory(s)).Select(s => s.Case).ToList();

    /// <summary>The cases the corpus test runs, a row each in DrawBopCheckCommandsShard*Tests (decision 27801).</summary>
    public static CorpusPlan Corpus { get; } = new(8, () => ToRun(UsingCheck));

    /// <summary>
    /// The corpus cases that use the check commands: how many are runnable before and after them, and how the newly
    /// runnable ones end under OCCT's _check_log.
    /// </summary>
    [Fact]
    public void Check_command_cases_of_the_corpus_run()
    {
        string aTests = Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");
        Assert.True(Directory.Exists(aTests), $"No OCCT source at {aTests}: run tools/get-occt.ps1");
        DrawInterpreter aBefore = Load();
        Before6G(aBefore);

        List<DrawCaseScan> aScans = [.. UsingCheck];
        List<DrawTestCase> aRunnable = ToRun(aScans);
        int aRunnableBefore = SharedCorpus.RunnableWith(aScans.Select(s => s.Case), aBefore).Count;
        output.WriteLine($"{aScans.Count} cases use the check commands: {aRunnableBefore} runnable before, {aRunnable.Count} after");
        foreach (var aGroup in aScans.Where(s => !s.IsRunnable).SelectMany(s => s.Missing).GroupBy(c => c)
                                     .OrderByDescending(g => g.Count()).Take(15))
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
        }

        Assert.True(aRunnable.Count > aRunnableBefore);
        // the Boolean cases' attributed failures (bugs/modalg_7/bug26034's bopcheck, bugs/modalg_5/bug23881's sewn shell),
        // and unit 7W's (bugs/modalg_7/bug21264, made runnable by XProgress, reads the progress indicator's output)
        Assert.DoesNotContain(aRuns, r => r.Status.StartsWith("FAILED", StringComparison.Ordinal) && !KnownFailures.Contains(r.Case.Id)
                                          && !DrawBooleanCorpusTests.KnownFailures.Contains(r.Case.Id)
                                          && !DrawCheapGapsCorpusTests.KnownFailures.ContainsKey(r.Case.Id));
    }

    /// <summary>Cases that fail with an attributed cause (see the unit's decisions).</summary>
    internal static readonly HashSet<string> KnownFailures = new(StringComparer.Ordinal);

    /// <summary>
    /// Captures the check commands' calls over the corpus cases that use them (runnable or not: a case's calls up to
    /// its first missing command are captured), for tools/borec (unit 6G), when
    /// KERF_BOREC_CAPTURE names a directory: writes check.capture.txt there.
    /// </summary>
    [Fact]
    public void Capture_for_borec()
    {
        string? aDir = Environment.GetEnvironmentVariable("KERF_BOREC_CAPTURE");
        if (string.IsNullOrEmpty(aDir))
        {
            return;
        }
        var aCorpus = new DrawCorpus(Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests"));
        DrawInterpreter anAfter = Load();
        List<DrawTestCase> aCases = aCorpus.Cases().AsParallel().AsOrdered().Select(c => aCorpus.Scan(c, anAfter))
                                           .Where(s => Commands.Take(3).Any(s.Commands.Contains)).Select(s => s.Case).ToList();
        var aResults = aCases.AsParallel().AsOrdered().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => (Case: c, Calls: DrawCapture.Capture(aCorpus, c, ["bopcheck", "bopargcheck", "bopapicheck", "bfuzzyvalue"], TimeSpan.FromSeconds(300))))
                             .ToList();
        using var aWriter = new StreamWriter(Path.Combine(aDir, "check.capture.txt"), false, new System.Text.UTF8Encoding(false)) { NewLine = "\n" };
        foreach ((DrawTestCase aCase, List<DrawCapture.Call> aCalls) in aResults)
        {
            if (aCalls.Count > 0)
            {
                DrawCapture.Write(aWriter, $"{aCase.Group}/{aCase.Grid}/{aCase.Name}", aCalls);
            }
        }
    }
}
