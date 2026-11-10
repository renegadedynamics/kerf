// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 8F's commands (XSDRAWSTEP's commands without a document, XSDRAW's session with the IFSelect and XSControl
/// commands) on hand-made shapes and on OCCT's corpus: the cases that use them, runnable after the commands and not
/// before, run as OCCT's harness runs them and judged by OCCT's log check.
/// </summary>
public class DrawStepCorpusTests(ITestOutputHelper output) : ICorpusTest
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 8F added.</summary>
    internal static readonly string[] Unit8FCommands =
    [
        "stepread", "stepwrite", "testreadstep", "testwritestep", "stepfileunits", "steptrans",
        "param", "data", "entity", "xload", "xread", "writeall", "writeent", "writent", "listtypes", "newmodel",
        "tpstat", "tpdraw", "tpcompound",
    ];

    /// <summary>Draw-lite before unit 8F: without its commands.</summary>
    internal static void Before8F(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit8FCommands)
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
    public void Stepwrite_stepread_and_the_session_commands_round_trip_a_box()
    {
        var anOutput = new StringWriter();
        DrawInterpreter aDI = NewInterpreter(anOutput);
        aDI.Eval("box b 10 20 30");
        string aWrite = aDI.Eval("stepwrite a b /tmp/p8f/b.stp");
        Assert.Contains("Translation: OK", aWrite, StringComparison.Ordinal);
        Assert.Contains("File /tmp/p8f/b.stp written", aWrite, StringComparison.Ordinal);
        Assert.Equal("1", aDI.Eval("file exists /tmp/p8f/b.stp"));
        Assert.False(File.Exists("/tmp/p8f/b.stp")); // nothing written to disk

        string aRead = aDI.Eval("stepread /tmp/p8f/b.stp r *");
        Assert.Contains("Transferrable Roots : Nb entities selected : 1", aRead, StringComparison.Ordinal);
        Assert.Contains("Transfer entity n0 ", aRead, StringComparison.Ordinal);
        Assert.Contains("-> DRAW Shape: r_1", aRead, StringComparison.Ordinal);
        Assert.Contains("SOLID     : 1", aDI.Eval("nbshapes r_1"), StringComparison.Ordinal);

        // the session's model and transfer
        string aData = aDI.Eval("data g");
        output.WriteLine(aData);
        Assert.Matches(@"Model : \d+ Entities, of which \d+ Root\(s\)", aData);
        Assert.Contains("TransferRead :", aDI.Eval("tpstat c"), StringComparison.Ordinal);
        Assert.Contains("1 Shape(s) listed", aDI.Eval("tpcompound c"), StringComparison.Ordinal);
        Assert.Contains("SOLID     : 1", aDI.Eval("nbshapes c"), StringComparison.Ordinal);
        Assert.Contains("MANIFOLD_SOLID_BREP", aDI.Eval("listtypes"), StringComparison.Ordinal);
        Assert.Contains("DUMP  Entity n0 1", aDI.Eval("entity 1"), StringComparison.Ordinal);

        // the interpreter's parameters
        Assert.Contains("Value : AP214IS", aDI.Eval("param write.step.schema AP203"), StringComparison.Ordinal);
        Assert.Contains("AP203", aDI.Eval("param write.step.schema"), StringComparison.Ordinal);
        aDI.Eval("newmodel");
        aDI.Eval("stepwrite m b /tmp/p8f/c.stp");
        Assert.Contains("CONFIG_CONTROL_DESIGN", aDI.Eval("read [open /tmp/p8f/c.stp r]"), StringComparison.Ordinal);
        Assert.ThrowsAny<Exception>(() => aDI.Eval("param no.such.param 1"));

        // another interpreter's parameters are its own
        DrawInterpreter aDI2 = NewInterpreter();
        Assert.Contains("AP214IS", aDI2.Eval("param write.step.schema"), StringComparison.Ordinal);

        // testreadstep and testwritestep
        aDI.Eval("param write.step.schema AP214IS");
        Assert.Equal("File(s) are Written", aDI.Eval("testwritestep /tmp/p8f/t.stp b"));
        Assert.Equal("Count of shapes produced : 1\n", aDI.Eval("testreadstep /tmp/p8f/t.stp t"));
        Assert.Equal("Count of shapes produced : 1\n", aDI.Eval("testreadstep /tmp/p8f/t.stp ts -stream"));
        Assert.Contains("SOLID     : 1", aDI.Eval("nbshapes ts"), StringComparison.Ordinal);
        Assert.Contains("LENGTH Unit\nmillimetre\n", aDI.Eval("stepfileunits /tmp/p8f/t.stp"), StringComparison.Ordinal);
    }

    private static IEnumerable<DrawCaseScan> Using8F
        => SharedCorpus.Scans.Where(s => Unit8FCommands.Any(s.Commands.Contains))
                       // the de group's STEP cases read with ReadStep: unit 8G2's class runs them
                       .Where(s => !DrawStepCafCorpusTests.IsDeStepCase(s.Case))
                       // the cases that measure memory are DrawCheapGapsCorpusTests'
                       .Where(s => !DrawCheapGapsCorpusTests.MeasuresMemory(s));

    /// <summary>The cases the corpus test runs, those 8F made runnable, a row each in DrawStepCorpusShard*Tests (decision 2082).</summary>
    public static CorpusPlan Corpus { get; } = new(6, () => SharedCorpus.NewlyRunnable(Using8F, Before8F, "OCCTNET_8F_FILTER"));

    [Fact]
    public void Corpus_cases_using_the_STEP_commands_run_after_8F()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        DrawInterpreter aBefore = NewInterpreter();
        Before8F(aBefore);

        List<DrawCaseScan> aScans = [.. Using8F];
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        string? aFilter = Environment.GetEnvironmentVariable("OCCTNET_8F_FILTER");
        // the commands' names are common words ("data", "param", "entity"): cases that name them in places the scan
        // counts as commands but don't run them were runnable before; the cases 8F makes runnable are the others
        var aRunnableBefore = new HashSet<string>(SharedCorpus.RunnableWith(aScans.Select(s => s.Case), aBefore).Select(c => c.Id),
                                                  StringComparer.Ordinal);
        List<DrawTestCase> aGained = aRunnable.Where(c => !aRunnableBefore.Contains(c.Id)).ToList();
        output.WriteLine($"{aScans.Count} cases name the STEP commands; runnable before 8F: {aRunnableBefore.Count}, "
                         + $"after: {aRunnable.Count}, made runnable by 8F: {aGained.Count}");

        List<DrawTestCase> aToRun = [.. aGained.Where(c => aFilter is null || c.Id.Contains(aFilter, StringComparison.Ordinal))];
        Assert.Equal(aToRun, Corpus.Cases);
        var aRuns = SharedCorpus.RunAll(aToRun)
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 8F: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
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

        var aMissing = new SortedDictionary<string, int>(StringComparer.Ordinal);
        foreach (DrawCaseScan aScan in aScans.Where(s => !s.IsRunnable))
        {
            foreach (string aCommand in aScan.Missing)
            {
                aMissing[aCommand] = aMissing.GetValueOrDefault(aCommand) + 1;
            }
        }
        output.WriteLine("not runnable, missing: " + string.Join(", ", aMissing.OrderByDescending(m => m.Value).Select(m => $"{m.Key} {m.Value}")));

        if (aFilter is not null)
        {
            return;
        }
        Assert.Equal(GainedCount, aGained.Count);
        Assert.Equal(OkCount, aRuns.Count(r => r.Status == "OK"));
        Assert.All(aRuns, r => Assert.True(r.Status == "OK" || r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)
                                           || r.Status.StartsWith("BAD", StringComparison.Ordinal) || KnownFailures.Contains(r.Case.Id),
                                           $"{r.Case.Id}: {r.Status}"));
    }

    // the corpus's counts after unit 8F (with unit 8G1's documents and unit 8G2's ReadStep and WriteStep; the de
    // group's STEP cases, which read with ReadStep, are DrawStepCafCorpusTests'): 305 gained, OK 54; with unit 8W's
    // commands (exec, ReadFile, WriteFile, xnorm) and Tcl's encodings 318, OK 65, BAD 5, SKIPPED 248, no failure
    private const int GainedCount = 318;
    private const int OkCount = 65;

    /// <summary>Cases that fail in Draw-lite, each attributed.</summary>
    internal static readonly HashSet<string> KnownFailures = new(StringComparer.Ordinal)
    {
        // set the environment variable CSF_STEPDefaults to a resource file of their own (`dsetenv`), whose
        // parameters Draw-lite's session doesn't load (resource files: decision 1181)
        "bugs/modalg_7/bug30273",
        "bugs/xde/bug6283",
    };
}
