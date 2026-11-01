// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 5K2's <c>cirtang</c> and <c>lintan</c> (GeometryTest_ConstraintCommands.cxx, over Geom2dGcc) on hand-made
/// curves and on OCCT's corpus: the cases that use them, runnable before and after the commands, run as OCCT's harness
/// runs them and judged by OCCT's log check.
/// </summary>
public class DrawConstraintCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 5K2 added (gcarc, in the same file, is GC's and came with unit 3L2b).</summary>
    internal static readonly string[] Unit5K2Commands = ["cirtang", "lintan"];

    /// <summary>Draw-lite before unit 5K2: without its commands.</summary>
    internal static void Before5K2(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit5K2Commands)
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
    public void Cirtang_and_lintan_name_their_solutions_as_Draw()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("circle c1 0 0 1");
        aDI.Eval("circle c2 5 0 1");
        aDI.Eval("line l 0 -3 1 0");
        // three curves: the eight circles tangent to two circles and a line, as many as GccAna finds
        string aCCC = aDI.Eval("cirtang r -c c1 -c c2 -c l");
        Assert.StartsWith("Solution of type C-C-C is: r_1 r_2 ", aCCC, StringComparison.Ordinal);
        Assert.Contains("Circle", aDI.Eval("dump r_1"), StringComparison.Ordinal);
        // two curves and a radius
        Assert.StartsWith("Solution of type C-C-R is: q_1 ", aDI.Eval("cirtang q -c c1 -c c2 -r 3"), StringComparison.Ordinal);
        // a curve and a centre
        aDI.Eval("point p 3 0");
        Assert.StartsWith("Solution of type C-P is: k_1 ", aDI.Eval("cirtang k -c c1 -p p"), StringComparison.Ordinal);
        // three points
        aDI.Eval("point p2 0 3");
        aDI.Eval("point p3 -3 0");
        Assert.Equal("Solution of type P-P-P is: t_1 ", aDI.Eval("cirtang t -p p -p p2 -p p3"));

        // the usage, unknown options, wrong names and unsupported sets are Tcl errors with OCCT's messages
        Assert.Contains("Use: cirtangresult", Assert.ThrowsAny<Exception>(() => aDI.Eval("cirtang r")).Message, StringComparison.Ordinal);
        Assert.Contains("Unknown option -x", Assert.ThrowsAny<Exception>(() => aDI.Eval("cirtang r -x 1")).Message, StringComparison.Ordinal);
        Assert.Contains("Error: nothing is not a curve", Assert.ThrowsAny<Exception>(() => aDI.Eval("cirtang r -c nothing")).Message,
                        StringComparison.Ordinal);
        Assert.Contains("Unsupported set of input data", Assert.ThrowsAny<Exception>(() => aDI.Eval("cirtang r -c c1 -c c2")).Message,
                        StringComparison.Ordinal);

        // lines tangent to two circles (GccAna: the four common tangents), and at an angle to a line
        Assert.Equal("m_1 m_2 m_3 m_4 ", aDI.Eval("lintan m c1 c2"));
        Assert.Contains("Line", aDI.Eval("dump m_1"), StringComparison.Ordinal);
        Assert.Equal("o_1 o_2 ", aDI.Eval("lintan o c1 l 30"));
        Assert.EndsWith("Second argument must be a line", Assert.ThrowsAny<Exception>(() => aDI.Eval("lintan o c1 c2 30")).Message,
                       StringComparison.Ordinal);
    }

    [Fact]
    public void Corpus_cases_using_cirtang_or_lintan_run_before_and_after_5K2()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before5K2(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => s.Commands.Contains("cirtang") || s.Commands.Contains("lintan"))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use cirtang or lintan; runnable before 5K2: {aRunnableBefore.Count}, after: {aRunnable.Count}");

        var aRuns = aRunnable.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                              TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 5K2: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
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

        output.WriteLine("Commands the other cases using cirtang or lintan miss (cases):");
        foreach (var aGroup in aScans.SelectMany(s => s.Missing).GroupBy(c => c).OrderByDescending(g => g.Count()).Take(15))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        Assert.Empty(aRunnableBefore);
        Assert.NotEmpty(aRunnable);
        Assert.Contains(aRuns, r => r.Status == "OK");
    }
}
