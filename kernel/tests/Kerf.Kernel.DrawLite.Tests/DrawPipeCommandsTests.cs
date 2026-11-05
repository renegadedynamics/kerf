// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 5L2's <c>sweep</c> and <c>tuyau</c> (GeometryTest_SurfaceCommands.cxx, GeomFill_Pipe) on hand-made curves and
/// on OCCT's corpus: the cases that use them, runnable before and after the commands, run as OCCT's harness runs them
/// and judged by OCCT's log check.
/// </summary>
public class DrawPipeCommandsTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 5L2 added.</summary>
    internal static readonly string[] Unit5L2Commands = ["sweep", "tuyau"];

    /// <summary>Draw-lite before unit 5L2: without its commands.</summary>
    internal static void Before5L2(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit5L2Commands)
        {
            theInterpreter.Eval($"rename {aCommand} {{}}");
        }
    }

    private static DrawInterpreter New() => new(new StringWriter { NewLine = "\n" });

    private static T Geometry<T>(DrawInterpreter theDI, string theName)
        => Assert.IsAssignableFrom<T>(Assert.IsType<DrawGeometry>(theDI.GetDrawable(theName)).Geometry);

    [Fact]
    public void Tuyau_builds_each_kind_of_pipe()
    {
        DrawInterpreter i = New();
        i.Eval("beziercurve path 4 0 0 0 10 0 0 10 10 0 20 10 5");
        // a constant radius
        Assert.StartsWith("Accuracy of approximation = ", i.Eval("tuyau r1 path 1.5"), StringComparison.Ordinal);
        Assert.IsType<BSplineSurface>(Geometry<Surface>(i, "r1"));

        // a constant section: a straight path and a circle make a cylinder
        i.Eval("line l 0 0 0 0 0 1; trim l l 0 10; circle c 0 0 0 0 0 1 2");
        Assert.Equal("Accuracy of approximation = 0\n", i.Eval("tuyau r2 l c"));
        var aCyl = Geometry<TrimmedSurface>(i, "r2");
        Assert.IsType<CylindricalSurface>(aCyl.BasisSurface);

        // two sections on a planar convex path
        i.Eval("circle arc 0 20 0 20; trim arc arc -1.5707963267948966 -0.5");
        i.Eval("circle s1 0 0 0 1 0 0 1; circle s2 0 0 0 1 0 0 2");
        Assert.StartsWith("Accuracy of approximation = ", i.Eval("tuyau r3 arc s1 s2"), StringComparison.Ordinal);
        Assert.IsType<BSplineSurface>(Geometry<Surface>(i, "r3"));

        // a pipe on an edge between two guide lines: the parallel lines of a cylinder
        i.Eval("line g1 2 0 0 0 0 1; trim g1 g1 0 10; line g2 0 2 0 0 0 1; trim g2 g2 0 10");
        Assert.Equal("", i.Eval("tuyau r4 l g1 g2 2"));
        Assert.IsType<CylindricalSurface>(Geometry<TrimmedSurface>(i, "r4").BasisSurface);

        // wrong arguments: a C++ command returning 1
        Assert.ThrowsAny<DrawException>(() => i.Eval("tuyau r5 path"));
        Assert.ThrowsAny<DrawException>(() => i.Eval("tuyau r5 nopath c"));
    }

    [Fact]
    public void Sweep_builds_with_each_trihedron()
    {
        DrawInterpreter i = New();
        i.Eval("beziercurve path 4 0 0 0 10 0 0 10 10 0 20 10 5");
        i.Eval("circle c 0 0 0 1 0 0 1");
        foreach (string anOption in new[] { "", "-FX ", "-FR ", "-CF " })
        {
            Assert.StartsWith("Accuracy of approximation = ", i.Eval($"sweep s {anOption}path c"), StringComparison.Ordinal);
            Assert.IsType<BSplineSurface>(Geometry<Surface>(i, "s"));
        }
        // a constant normal: sweep result -CN dx dy dz path section
        Assert.StartsWith("Accuracy of approximation = ", i.Eval("sweep s -CN 0 0 1 path c"), StringComparison.Ordinal);
        // Darboux: a 2d path on a support surface
        i.Eval("plane p 0 0 0 0 0 1; 2dbeziercurve p2 3 0 0 5 1 10 0");
        i.Eval("circle cd 0 0 0 1 0 0 0.5");
        Assert.StartsWith("Accuracy of approximation = ", i.Eval("sweep s -DX p2 p cd"), StringComparison.Ordinal);
        Assert.ThrowsAny<DrawException>(() => i.Eval("sweep s path"));
    }

    [Fact]
    public void Corpus_cases_using_sweep_and_tuyau_run_before_and_after_5L2()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before5L2(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => s.Commands.Contains("sweep") || s.Commands.Contains("tuyau"))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use sweep or tuyau; runnable before 5L2: {aRunnableBefore.Count}, after: {aRunnable.Count}");

        var aRuns = aRunnable.Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                              TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 5L2: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
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
        output.WriteLine("Commands the other cases using sweep or tuyau miss (cases):");
        foreach (var aGroup in aScans.Where(s => !s.IsRunnable).SelectMany(s => s.Missing).GroupBy(c => c)
                                     .OrderByDescending(g => g.Count()).Take(15))
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        Assert.Empty(aRunnableBefore);
        Assert.NotEmpty(aRunnable);
        Assert.DoesNotContain(aRuns, r => r.Status.StartsWith("FAILED", StringComparison.Ordinal));
    }

    private static DrawInterpreter NewInterpreter()
    {
        var anInterpreter = new DrawInterpreter();
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }
}
