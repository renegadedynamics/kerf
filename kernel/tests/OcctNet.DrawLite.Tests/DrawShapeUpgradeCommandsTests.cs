// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

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
    private static TopoDS_Shape TwoCoplanarFaces(out TopoDS_Shape theSharedEdge)
    {
        TopoDS_Face Rectangle(double x0, double x1)
            => new BRepBuilderAPI_MakeFace(new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), x0, x1, 0, 4).Face();
        var aSewing = new BRepBuilderAPI_Sewing(1.0e-6);
        aSewing.Add(Rectangle(0, 5));
        aSewing.Add(Rectangle(5, 10));
        aSewing.Perform();
        TopoDS_Shape aShell = aSewing.SewedShape();
        var aMap = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aShell, TopAbs_ShapeEnum.TopAbs_EDGE, TopAbs_ShapeEnum.TopAbs_FACE, aMap);
        theSharedEdge = Enumerable.Range(1, aMap.Extent()).Select(aMap.FindKey).Single(e => aMap.FindFromKey(e).Extent() == 2);
        return aShell;
    }

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    [Fact]
    public void Unifysamedom_unifies_with_its_options_and_keeps_the_given_shapes()
    {
        DrawInterpreter i = Load();
        i.SetShape("s", TwoCoplanarFaces(out TopoDS_Shape aShared));
        i.SetShape("e", aShared);

        i.Eval("unifysamedom r s");
        Assert.Equal(1, Count(i.GetShape("r"), TopAbs_ShapeEnum.TopAbs_FACE));
        Assert.Equal(4, Count(i.GetShape("r"), TopAbs_ShapeEnum.TopAbs_EDGE));

        i.Eval("unifysamedom r s -e");
        Assert.Equal(1, Count(i.GetShape("r"), TopAbs_ShapeEnum.TopAbs_FACE));
        Assert.Equal(6, Count(i.GetShape("r"), TopAbs_ShapeEnum.TopAbs_EDGE));

        i.Eval("unifysamedom r s -f");
        Assert.Equal(2, Count(i.GetShape("r"), TopAbs_ShapeEnum.TopAbs_FACE));

        i.Eval("unifysamedom r s e");
        Assert.Equal(2, Count(i.GetShape("r"), TopAbs_ShapeEnum.TopAbs_FACE));

        i.Eval("unifysamedom r s +b +i -nosafe -t 1e-5 -a 1");
        Assert.Equal(1, Count(i.GetShape("r"), TopAbs_ShapeEnum.TopAbs_FACE));

        Assert.Equal(2, Count(i.GetShape("s"), TopAbs_ShapeEnum.TopAbs_FACE));
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
                                                                            DrawCorpusTests.DataPath, TimeSpan.FromSeconds(120), null))
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
