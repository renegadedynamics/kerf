// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 8H1's STL commands (XSDRAWSTL.cxx: <c>readstl</c>, <c>writestl</c>) on hand-made shapes and on OCCT's corpus:
/// the cases that use them, runnable after the commands and not before, run as OCCT's harness runs them and judged by
/// OCCT's log check.
/// </summary>
[Collection(MeshDefaultsCollection.Name)]
public class DrawStlCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 8H1 added.</summary>
    internal static readonly string[] Unit8H1Commands = ["readstl", "writestl"];

    /// <summary>Draw-lite before unit 8H1: without its commands.</summary>
    internal static void Before8H1(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit8H1Commands)
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
    public void Writestl_and_readstl_round_trip_a_meshed_box()
    {
        var anOutput = new StringWriter();
        DrawInterpreter aDI = NewInterpreter(anOutput);
        aDI.Eval("box b 10 20 30");
        aDI.Eval("incmesh b 0.1");
        // binary by default, ASCII with 0
        Assert.Equal("", aDI.Eval("writestl b /tmp/p8h1/b.stl"));
        Assert.Equal("", aDI.Eval("writestl b /tmp/p8h1/a.stl 0"));
        Assert.Equal("1", aDI.Eval("file exists /tmp/p8h1/b.stl"));
        aDI.Eval("readstl r /tmp/p8h1/b.stl");
        aDI.Eval("readstl ra /tmp/p8h1/a.stl");
        // one face holding the triangulation: 8 nodes (merged) and 12 triangles
        Assert.Contains("12 triangles", aDI.Eval("trinfo r"), StringComparison.Ordinal);
        Assert.Contains("8 nodes", aDI.Eval("trinfo r"), StringComparison.Ordinal);
        Assert.Contains("8 nodes", aDI.Eval("trinfo ra"), StringComparison.Ordinal);
        Assert.Contains("FACE      : 1", aDI.Eval("nbshapes r"), StringComparison.Ordinal);
        // -mergeAngle 0: no merging, three nodes per triangle; -brep: a face per triangle
        aDI.Eval("readstl r0 /tmp/p8h1/b.stl -mergeAngle 0");
        Assert.Contains("36 nodes", aDI.Eval("trinfo r0"), StringComparison.Ordinal);
        aDI.Eval("readstl rb /tmp/p8h1/a.stl -brep");
        Assert.Contains("FACE      : 12", aDI.Eval("nbshapes rb"), StringComparison.Ordinal);
        Assert.Contains("VERTEX    : 8", aDI.Eval("nbshapes rb"), StringComparison.Ordinal);
        // -multi on a single domain: one face
        aDI.Eval("readstl rm /tmp/p8h1/a.stl -multi");
        Assert.Contains("FACE      : 1", aDI.Eval("nbshapes rm"), StringComparison.Ordinal);
        // nothing was written to disk
        Assert.False(File.Exists("/tmp/p8h1/b.stl"));

        // errors as Draw's
        Assert.Contains("Use: writestl", aDI.Eval("writestl b"), StringComparison.Ordinal);
        Assert.Contains("Mesh writing has been failed", aDI.Eval("box c 1 1 1; writestl c /tmp/p8h1/c.stl"), StringComparison.Ordinal);
        Assert.ThrowsAny<Exception>(() => aDI.Eval("readstl x"));
        Assert.Contains("not enough arguments", anOutput.ToString(), StringComparison.Ordinal);
        Assert.ThrowsAny<Exception>(() => aDI.Eval("readstl x /tmp/p8h1/a.stl -mergeAngle 100"));
        Assert.ThrowsAny<Exception>(() => aDI.Eval("readstl x /tmp/p8h1/a.stl -bad"));
        Assert.Contains("unknown argument '-bad'", anOutput.ToString(), StringComparison.Ordinal);
        aDI.Eval("readstl x /tmp/p8h1/none.stl");
        Assert.Contains("Error: file '/tmp/p8h1/none.stl' is not found", anOutput.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void Corpus_cases_using_the_STL_commands_run_after_8H1()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before8H1(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => Unit8H1Commands.Any(s.Commands.Contains))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        string? aFilter = Environment.GetEnvironmentVariable("OCCTNET_8H1_FILTER");
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use the STL commands; runnable before 8H1: {aRunnableBefore.Count}, after: {aRunnable.Count}");

        var aRuns = aRunnable.Where(c => aFilter is null || c.Id.Contains(aFilter, StringComparison.Ordinal))
                             .AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                              TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 8H1: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
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

        foreach (DrawCaseScan aScan in aScans.Where(s => !s.IsRunnable))
        {
            output.WriteLine($"  not runnable: {aScan.Case.Id} misses {string.Join(" ", aScan.Missing)}");
        }

        if (aFilter is not null)
        {
            return;
        }
        Assert.Empty(aRunnableBefore);
        Assert.Equal(88, aScans.Count);
        Assert.Equal(67, aRunnable.Count);
        // OK 7, SKIPPED 59 (data not public: de_mesh's models), FAILED 1 (attributed); the 21 others wait for XDE,
        // STEP, IGES, VRML, the viewer's and the MeshVS commands, and exec
        Assert.Equal(7, aRuns.Count(r => r.Status == "OK"));
        Assert.Equal(59, aRuns.Count(r => r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)));
        Assert.All(aRuns, r => Assert.True(r.Status == "OK" || r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)
                                           || r.Status.StartsWith("BAD", StringComparison.Ordinal) || KnownFailures.Contains(r.Case.Id),
                                           $"{r.Case.Id}: {r.Status}"));
    }

    /// <summary>Cases that fail in Draw-lite, each attributed.</summary>
    private static readonly HashSet<string> KnownFailures = new(StringComparer.Ordinal)
    {
        // checks the progress indicator's output (XProgress +t), which Draw-lite's commands don't make (decision 954)
        "de_mesh/shape_write_stl/B1",
    };
}
