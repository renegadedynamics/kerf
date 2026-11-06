// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text.RegularExpressions;
using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// The cases that read their process's own figures run in a process of their own (decision 1699), and what that has to
/// show: OCCT's memory and time checks give the same answer whatever else is running, and still fail when they should.
/// The scripts that should fail are small ones in a corpus of their own, using OCCT's <c>checktrend</c> as the real
/// cases do.
/// </summary>
public sealed class DrawCaseProcessTests : IDisposable
{
    private readonly string myTests = Directory.CreateTempSubdirectory("kerf-case-process").FullName;

    public void Dispose() => Directory.Delete(myTests, true);

    private DrawTestCase Case(string theName, string theScript)
    {
        string aGrid = Path.Combine(myTests, "bugs", "fclasses");
        Directory.CreateDirectory(aGrid);
        File.Copy(Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests", "parse.rules"), Path.Combine(myTests, "parse.rules"), true);
        string aPath = Path.Combine(aGrid, theName);
        File.WriteAllText(aPath, theScript + "\nputs \"TEST COMPLETED\"\n");
        return new DrawTestCase("bugs", "fclasses", theName, aPath);
    }

    private DrawCaseRun Run(DrawTestCase theCase)
        => new DrawCorpus(myTests).RunAndCheck(theCase, DrawResourcesTests.ResourcesDirectory, null, TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds));

    private static string Trend(string theCycle, int theTolerance = 256, int theCycles = 9)
        => $$"""
             set listmem {}
             box b 10 10 10
             box b1 10 10 10 100 100 100
             box b2 50 50 50
             for {set i 1} {$i <= {{theCycles}}} {incr i} {
                 {{theCycle}}
                 lappend listmem [meminfo h]
                 checktrend $listmem 0 {{theTolerance}} "Memory leak detected"
             }
             """;

    [Theory]
    [InlineData("meminfo h", "Heap")]
    [InlineData("lappend listmem [meminfo   heap]", "Heap")]
    [InlineData("set p [meminfo wsetpeak]", "Process")]
    [InlineData("puts \"Error: Section of simple BSpline surfaces_2 is performed too slow\"", "Process")]
    [InlineData("regexp {Tps: +([0-9.]+)} [bbuild r -t] f t\nif {$t > 2} { puts \"Error: low performance\" }", "BuildTime")]
    [InlineData("bbuild r -t", "None")]
    [InlineData("puts \"Error: low performance\"", "None")]
    [InlineData("bop b1 b2", "None")]
    internal void What_a_script_reads_decides_where_it_runs(string theScript, string theReading)
        => Assert.Equal(theReading, DrawCaseProcess.ReadingOf(theScript).ToString());

    [Fact]
    public void A_cycle_that_lets_go_of_what_it_made_passes_at_any_tolerance()
    {
        // what bug24863_1 does and bug31075's tolerance, with a reading a cycle for longer than bug31075's first failures
        DrawCaseRun aRun = Run(Case("let_go", Trend("box b 10 10 10", theTolerance: 0, theCycles: 40)));
        Assert.True(aRun.Status == "OK", aRun.Status + "\n" + aRun.Log);
    }

    [Theory]
    [InlineData("box b$i 10 10 10", 256)] // a shape, what bug24863_1 asks about; about 15 kB a cycle
    [InlineData("bop b1 b2; bopcut r$i", 100)] // the result of a Boolean, what bug7287 asks about; about 15 kB a cycle
    // about the same again, as a plain string. Smaller ones are seen too (decision 1701), but less surely
    [InlineData("box b 10 10 10; lappend keep [string repeat x 8000]", 256)]
    [InlineData("box b 10 10 10; lappend keep [string repeat x 8000]", 0)]
    public void A_cycle_that_holds_on_to_what_it_made_fails(string theCycle, int theTolerance)
    {
        // thirty cycles, where the real cases have nine or ten: one passing step in a short run can widen sigma past the mean
        DrawCaseRun aRun = Run(Case("keeps", Trend(theCycle, theTolerance, theCycles: 30)));
        Assert.True(aRun.Status.StartsWith("FAILED", StringComparison.Ordinal) && aRun.Log.Contains("Memory leak detected", StringComparison.Ordinal),
                    aRun.Status + "\n" + aRun.Log);
    }

    [Fact]
    public void A_reading_is_the_cases_own_and_not_the_tests_process()
    {
        // kept here, out of the case's reach: a reading in the same process would be above 200 MB
        byte[] aKept = new byte[200 * 1024 * 1024];
        DrawCaseRun aRun = Run(Case("own_heap", "puts \"Reading: [meminfo h]\""));
        GC.KeepAlive(aKept);

        double aReading = double.Parse(Regex.Match(aRun.Log, @"Reading: (\d+)").Groups[1].Value, CultureInfo.InvariantCulture);
        Assert.InRange(aReading, 1024, 100 * 1024 * 1024);
        Assert.Equal(0, aReading % 1024);
    }

    [Fact]
    public void A_process_that_doesnt_finish_is_a_failure_and_says_why()
    {
        DrawTestCase aCase = Case("exits", "meminfo h\nwhile 1 {}");
        DrawCaseRun aRun = new DrawCorpus(myTests).RunAndCheck(aCase, DrawResourcesTests.ResourcesDirectory, null, TimeSpan.FromSeconds(0.2));
        Assert.False(aRun.Passed, aRun.Status);
        Assert.Contains("Process killed by elapsed limit", aRun.Log, StringComparison.Ordinal);
    }

    [Fact]
    public void The_build_time_is_the_processes_processor_time_and_still_fails_above_the_figure()
    {
        // bug24157's shape, on a small build whose time isn't zero: a figure of nothing fails and a generous one passes
        string Script(string theFigure)
            => $$"""
                 box b1 10 10 10
                 box b2 5 5 5 10 10 10
                 bclearobjects
                 bcleartools
                 baddobjects b1
                 baddtools b2
                 bfillds
                 regexp { +Tps: +([-0-9.+eE]+)} [bbuild result -t] full tps_time
                 if { $tps_time > {{theFigure}} } {
                   puts "Error: low performance"
                 } else {
                   puts "OK: high performance"
                 }
                 """;

        Assert.StartsWith("FAILED", Run(Case("slow", Script("-1"))).Status, StringComparison.Ordinal);
        Assert.Equal("OK", Run(Case("quick", Script("60"))).Status);
    }

    /// <summary>
    /// What bugs/modalg_7/bug31120 asks, asked so that it can fail: the script compares a text, <c>(a-{b})/c</c>, with 0.05
    /// as strings, which is false whatever the figures (occt-issues.md, unit 11P).
    /// </summary>
    [Fact]
    public void Repeating_an_offset_keeps_nothing()
    {
        DrawTestCase aCase = Case("offset", """
            restore [locate_data_file bug31120_wire.brep] w
            mkoffset r w 1 0.5
            set listmem {}
            for {set i 0} {$i < 20} {incr i} {
              mkoffset r w 1 0.5
              lappend listmem [meminfo h]
              checktrend $listmem 0 256 "Memory leak detected"
            }
            """);
        DrawCaseRun aRun = new DrawCorpus(myTests).RunAndCheck(aCase, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath, TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds));
        Assert.True(aRun.Status == "OK", aRun.Status + "\n" + aRun.Log);
    }

    /// <summary>bug83's time check fails on a cold start: in a new process each time it doesn't depend on what ran before.</summary>
    [Fact]
    public void The_section_timed_against_a_figure_gives_one_answer()
    {
        string aTests = Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");
        var aCorpus = new DrawCorpus(aTests);
        DrawTestCase aCase = aCorpus.Cases().Single(c => c.Id == "bugs/modalg_7/bug83");
        var aStatuses = Enumerable.Range(0, 3)
                                  .Select(_ => aCorpus.RunAndCheck(aCase, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath, TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds)).Status)
                                  .Distinct().ToList();
        Assert.StartsWith("FAILED", Assert.Single(aStatuses), StringComparison.Ordinal);
    }

    /// <summary>The corpus's memory trends, all that run, repeated beside each other: each gives one answer.</summary>
    [Fact]
    public void The_corpus_memory_trends_pass_each_time_side_by_side()
    {
        string aTests = Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");
        Assert.True(Directory.Exists(aTests), $"No OCCT source at {aTests}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(aTests);
        string[] anIds =
        [
            "bugs/fclasses/bug7287_1", "bugs/fclasses/bug7287_2", "bugs/fclasses/bug7287_3", "bugs/fclasses/bug7287_4", "bugs/fclasses/bug7287_5",
            "bugs/fclasses/bug7287_6", "bugs/fclasses/bug24863_1", "bugs/caf/bug31075", "bugs/modalg_7/bug31120",
        ];
        List<DrawTestCase> aCases = aCorpus.Cases().Where(c => anIds.Contains(c.Id)).ToList();
        Assert.Equal(anIds.Length, aCases.Count);
        List<DrawCaseRun> aRuns = Enumerable.Repeat(aCases, 3).SelectMany(c => c).AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                                            .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath, TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds)))
                                            .ToList();
        Assert.All(aRuns, r => Assert.True(r.Status == "OK", $"{r.Case.Id}: {r.Status}\n{r.Log}"));
    }
}
