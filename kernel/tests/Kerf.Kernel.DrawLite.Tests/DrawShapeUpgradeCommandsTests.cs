// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 6U's command, <c>unifysamedom</c> (SWDRAW_ShapeUpgrade.cxx), with its options, and the corpus cases it makes
/// runnable: the cases that use it, before and after it.
/// </summary>
public class DrawShapeUpgradeCommandsTests(ITestOutputHelper output)
{
    private static DrawInterpreter Load()
    {
        var anInterpreter = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    // Two coplanar rectangles sewn into a shell: [0, 5] x [0, 4] and [5, 10] x [0, 4]
    private static Shape TwoCoplanarFaces(out Shape theSharedEdge)
    {
        Face Rectangle(double x0, double x1)
            => new MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), x0, x1, 0, 4).Face();
        var aSewing = new Sewing(1.0e-6);
        aSewing.Add(Rectangle(0, 5));
        aSewing.Add(Rectangle(5, 10));
        aSewing.Perform();
        Shape aShell = aSewing.SewedShape();
        var aMap = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aShell, ShapeType.Edge, ShapeType.Face, aMap);
        theSharedEdge = aMap.Keys.Single(e => aMap.GetValue(e).Extent() == 2);
        return aShell;
    }

    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    [Fact]
    public void Unifysamedom_unifies_with_its_options_and_keeps_the_given_shapes()
    {
        DrawInterpreter i = Load();
        i.SetShape("s", TwoCoplanarFaces(out Shape aShared));
        i.SetShape("e", aShared);

        i.Eval("unifysamedom r s");
        Assert.Equal(1, Count(i.GetShape("r"), ShapeType.Face));
        Assert.Equal(4, Count(i.GetShape("r"), ShapeType.Edge));

        i.Eval("unifysamedom r s -e");
        Assert.Equal(1, Count(i.GetShape("r"), ShapeType.Face));
        Assert.Equal(6, Count(i.GetShape("r"), ShapeType.Edge));

        i.Eval("unifysamedom r s -f");
        Assert.Equal(2, Count(i.GetShape("r"), ShapeType.Face));

        i.Eval("unifysamedom r s e");
        Assert.Equal(2, Count(i.GetShape("r"), ShapeType.Face));

        i.Eval("unifysamedom r s +b +i -nosafe -t 1e-5 -a 1");
        Assert.Equal(1, Count(i.GetShape("r"), ShapeType.Face));

        Assert.Equal(2, Count(i.GetShape("s"), ShapeType.Face));
        Assert.Contains("value expected after -t", Assert.ThrowsAny<DrawException>(() => i.Eval("unifysamedom r s -t")).Reason);
        Assert.Contains("Use unifysamedom result shape", Assert.ThrowsAny<DrawException>(() => i.Eval("unifysamedom r")).Reason);
    }

    /// <summary>Draw-lite before unit 6U: without unifysamedom.</summary>
    internal static void Before6U(DrawInterpreter theInterpreter)
    {
        theInterpreter.Eval("rename unifysamedom {}");
    }

    /// <summary>
    /// The corpus cases that use unifysamedom: how many are runnable before and after unit 6U, and how the newly
    /// runnable ones end under OCCT's _check_log.
    /// </summary>
    [Fact]
    public void Unifysamedom_cases_of_the_corpus_run()
    {
        string aTests = Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");
        Assert.True(Directory.Exists(aTests), $"No OCCT source at {aTests}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(aTests);
        DrawInterpreter anAfter = Load();
        DrawInterpreter aBefore = Load();
        Before6U(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered().Select(c => aCorpus.Scan(c, anAfter))
                                           .Where(s => s.Commands.Contains("unifysamedom")).ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        int aRunnableBefore = aScans.Count(s => aCorpus.Scan(s.Case, aBefore).IsRunnable);
        output.WriteLine($"{aScans.Count} cases use unifysamedom: {aRunnableBefore} runnable before 6U, {aRunnable.Count} after");
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
        // after 6L, cases that also use its Boolean commands run too; their attributed failures are 6L's
        Assert.DoesNotContain(aRuns, r => r.Status.StartsWith("FAILED", StringComparison.Ordinal)
                                          && !DrawBooleanCorpusTests.KnownFailures.Contains(r.Case.Id));
    }
}
