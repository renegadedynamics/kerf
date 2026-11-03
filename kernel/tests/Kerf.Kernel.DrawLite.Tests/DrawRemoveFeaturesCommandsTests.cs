// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 6G's <c>removefeatures</c> (BOPTest_RemoveFeaturesCommands.cxx), its history through <c>savehistory</c>, and
/// the corpus cases it makes runnable: the cases that use it, before and after it. The captures tools/borec records
/// from are written when OCCTNET_BOREC_CAPTURE names a directory.
/// </summary>
public class DrawRemoveFeaturesCommandsTests(ITestOutputHelper output)
{
    private static DrawInterpreter Load()
    {
        var anInterpreter = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    [Fact]
    public void Removefeatures_on_a_hand_made_hole()
    {
        DrawInterpreter i = Load();
        Shape aTool = new ApiMakeCylinder(new Frame3D(new Point3D(5, 5, -1), new Direction3D(0, 0, 1)), 2.0, 12.0).Shape();
        Shape aShape = new Cut(new MakeBox(10.0, 10.0, 10.0).Shape(), aTool).Shape();
        i.SetShape("s", aShape);
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aShape, ShapeType.Face, aMap);
        Shape aHole = aMap
                                       .First(f => Tool.Surface(ShapeCasts.Face(f)) is CylindricalSurface);
        i.SetShape("f", aHole);
        i.Eval("removefeatures r s f nothere -parallel");
        Assert.Contains("1000", i.Eval("vprops r"), StringComparison.Ordinal);
        i.Eval("savehistory h");
        Assert.Equal("Deleted.", i.Eval("isdeleted h f"));

        // a face not in the shape: the error is reported and the result isn't set
        i.Eval("box b2 1 1 1; explode b2 f");
        Assert.Contains("Error: ", i.Eval("removefeatures r2 s b2_1"), StringComparison.Ordinal);
        Assert.Null(i.GetDrawable("r2"));
        Assert.Contains("is a null shape", Assert.ThrowsAny<DrawException>(() => i.Eval("removefeatures r2 nothere f")).Reason);
    }

    /// <summary>Draw-lite before unit 6G's removefeatures: without the command.</summary>
    internal static void Before6G(DrawInterpreter theInterpreter)
    {
        theInterpreter.Eval("rename removefeatures {}");
    }

    /// <summary>The corpus cases that use removefeatures: runnable before and after, and how they end.</summary>
    [Fact]
    public void Removefeatures_cases_of_the_corpus_run()
    {
        string aTests = Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");
        var aCorpus = new DrawCorpus(aTests);
        DrawInterpreter anAfter = Load();
        DrawInterpreter aBefore = Load();
        Before6G(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered().Select(c => aCorpus.Scan(c, anAfter))
                                           .Where(s => s.Commands.Contains("removefeatures")).ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        int aRunnableBefore = aScans.Count(s => aCorpus.Scan(s.Case, aBefore).IsRunnable);
        output.WriteLine($"{aScans.Count} cases use removefeatures: {aRunnableBefore} runnable before, {aRunnable.Count} after");
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
        // unit 7W's dversion and XProgress made bugs/modalg_7/bug30788 runnable: it reads the progress indicator's output,
        // which Draw-lite's commands don't make (an attributed failure of DrawCheapGapsCorpusTests)
        Assert.DoesNotContain(aRuns, r => r.Status.StartsWith("FAILED", StringComparison.Ordinal)
                                          && !DrawCheapGapsCorpusTests.KnownFailures.ContainsKey(r.Case.Id));
    }

    /// <summary>
    /// Writes the capture of the corpus's removefeatures calls for tools/borec (when OCCTNET_BOREC_CAPTURE names a
    /// directory): each call's words, the input's BRep text and, as the extra word "#faces i j ...", the indices of the
    /// faces to remove in the input's TopExp.MapShapes(FACE) numbering (shapes read from separate texts share no
    /// TShape, so native finds the faces by these indices).
    /// </summary>
    [Fact]
    public void Capture_for_borec()
    {
        string? aDir = Environment.GetEnvironmentVariable("OCCTNET_BOREC_CAPTURE");
        if (string.IsNullOrEmpty(aDir))
        {
            return;
        }
        var aCorpus = new DrawCorpus(Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests"));
        List<DrawTestCase> aCases = [.. aCorpus.Cases().Where(c => c is { Group: "boolean", Grid: "removefeatures" })];
        var aResults = aCases.AsParallel().AsOrdered().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => (Case: c, Calls: CaptureCase(aCorpus, c))).ToList();
        using var aWriter = new StreamWriter(Path.Combine(aDir, "defeature-capture.txt"), false, new System.Text.UTF8Encoding(false)) { NewLine = "\n" };
        int aNb = 0;
        foreach ((DrawTestCase aCase, List<DrawCapture.Call> aCalls) in aResults)
        {
            if (aCalls.Count > 0)
            {
                DrawCapture.Write(aWriter, $"{aCase.Group}/{aCase.Grid}/{aCase.Name}", aCalls);
                aNb += aCalls.Count;
            }
        }
        output.WriteLine($"{aNb} removefeatures calls captured");
        Assert.True(aNb > 0);
    }

    private static List<DrawCapture.Call> CaptureCase(DrawCorpus theCorpus, DrawTestCase theCase)
    {
        List<DrawCapture.Call> aCalls = [];
        theCorpus.RunAndCheck(theCase, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath, TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), i =>
        {
            i.Eval("rename removefeatures __capture_removefeatures");
            i.Register("removefeatures", (theDI, a) =>
            {
                if (a.Count >= 4 && theDI.GetShape(a[2]) is { } anInput && !anInput.IsNull)
                {
                    var aFaces = new IndexedSet<Shape>(ShapeHasher.Instance);
                    Exploration.MapShapes(anInput, ShapeType.Face, aFaces);
                    List<int> anIndices = [];
                    for (int k = 3; k < a.Count; ++k)
                    {
                        Shape aFeature = theDI.GetShape(a[k]);
                        if (aFeature.IsNull)
                        {
                            continue;
                        }
                        for (Explorer anExp = new(aFeature, ShapeType.Face); anExp.More(); anExp.Next())
                        {
                            anIndices.Add(aFaces.IndexOf(anExp.Current()) + 1);
                        }
                    }
                    using var aText = new StringWriter(System.Globalization.CultureInfo.InvariantCulture) { NewLine = "\n" };
                    BrepTools.Write(anInput, aText);
                    aCalls.Add(new DrawCapture.Call([.. a, "#faces " + string.Join(' ', anIndices)],
                                                    new Dictionary<string, string>(StringComparer.Ordinal) { [a[2]] = aText.ToString() }));
                }
                return theDI.Invoke(["__capture_removefeatures", .. a.Skip(1)]);
            });
        });
        return aCalls;
    }
}
