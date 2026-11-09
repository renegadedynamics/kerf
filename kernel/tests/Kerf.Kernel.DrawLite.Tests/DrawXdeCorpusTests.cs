// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// Unit 8G1's XDE commands over Draw-lite's documents (XDEDRAW, DDocStd's document commands, the glTF and OBJ
/// commands) on OCCT's corpus: the cases that use them, runnable after the commands and not before, run as OCCT's
/// harness runs them and judged by OCCT's log check.
/// </summary>
public class DrawXdeCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 8G1 added.</summary>
    internal static readonly string[] Unit8G1Commands =
    [
        "NewDocument", "Close", "ForgetAll", "UndoLimit", "NewCommand", "OpenCommand", "AbortCommand", "CommitCommand", "SetName", "GetName",
        "XNewDoc", "Xdump", "XStat", "XSetLengthUnit", "XGetLengthUnit", "XDumpAssemblyTree", "XDumpAssemblyGraph",
        "XDumpNomenclature", "XDisplay", "XAddShape", "XNewShape", "XSetShape", "XGetShape", "XRemoveShape", "XFindShape",
        "XFindSubShape", "XFindMainShape", "XAddSubShape", "XLabelInfo", "XGetUsers", "XNbComponents", "XAddComponent",
        "XRemoveComponent", "XGetReferredShape", "XGetTopLevelShapes", "XGetFreeShapes", "XGetOneShape", "XDumpLocation",
        "XUpdateAssemblies", "XAutoNaming",
        "XSetColor", "XGetColor", "XGetShapeColor", "XGetAllColors", "XAddColor", "XRemoveColor", "XFindColor", "XUnsetColor",
        "XSetObjVisibility", "XGetObjVisibility", "XSetLayer", "XGetLayers", "XGetOneLayer", "XAddLayer", "XSetLinkLayer",
        "XGetAllLayers", "XUnSetLayer", "XUnSetAllLayers", "XRemoveLayer", "XRemoveAllLayers", "XFindLayer", "XGetLayerLabels",
        "XSetVisibility", "XIsVisible", "XSetProps", "XSetVolume", "XGetVolume", "XSetArea", "XGetArea", "XSetCentroid",
        "XGetCentroid",
        "ReadGltf", "readgltf", "WriteGltf", "writegltf", "WriteObj", "writeobj",
        "vcaps", "vlight",
    ];

    /// <summary>Draw-lite before unit 8G1: without its commands, and without the scan's knowledge of the xcaf branches.</summary>
    internal static void Before8G1(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit8G1Commands)
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
    public void Corpus_cases_using_the_XDE_commands_run_after_8G1()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        Before8G1(aBefore);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => Unit8G1Commands.Any(s.Commands.Contains))
                                           // the de group's STEP cases read with ReadStep: unit 8G2's class runs them
                                           .Where(s => !DrawStepCafCorpusTests.IsDeStepCase(s.Case))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        string? aFilter = Environment.GetEnvironmentVariable("OCCTNET_8G1_FILTER");
        // the cases that measure memory are DrawCheapGapsCorpusTests'
        HashSet<DrawTestCase> aMemoryCases = aScans.Where(DrawCheapGapsCorpusTests.MeasuresMemory).Select(s => s.Case).ToHashSet();
        List<DrawTestCase> aRunnableBefore = aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).ToList();
        output.WriteLine($"{aScans.Count} cases use the XDE commands; runnable before 8G1: {aRunnableBefore.Count}, after: {aRunnable.Count}");

        var aRuns = aRunnable.Where(c => !aMemoryCases.Contains(c))
                             .Where(c => aFilter is null or "*" || c.Id.Contains(aFilter, StringComparison.Ordinal))
                             .AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                             .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                              TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                             .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                             .ToList();
        output.WriteLine("after 8G1: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
                                                                .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                                .Select(g => $"{g.Key} {g.Count()}")));
        foreach (DrawCaseRun aRun in aRuns)
        {
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}");
            if (!aRun.Passed)
            {
                string[] aLines = aRun.Log.Split('\n');
                foreach (string aLine in aLines.Where(l => l.Contains("rror", StringComparison.Ordinal)
                                                           || l.Contains("xception", StringComparison.Ordinal)
                                                           || l.Contains("Faulty", StringComparison.Ordinal)).Take(4))
                {
                    output.WriteLine($"      {aLine}");
                }
                if (Environment.GetEnvironmentVariable("OCCTNET_8G1_LOG") == "1")
                {
                    output.WriteLine(aRun.Log);
                }
            }
        }

        foreach (IGrouping<string, string> aMissing in aScans.Where(s => !s.IsRunnable)
                                                             .SelectMany(s => s.Missing)
                                                             .GroupBy(m => m)
                                                             .OrderByDescending(g => g.Count())
                                                             .Take(40))
        {
            output.WriteLine($"  missing {aMissing.Key}: {aMissing.Count()} cases");
        }

        if (aFilter is not null)
        {
            return;
        }
        Assert.Empty(aRunnableBefore);
        Assert.Equal(2028, aScans.Count); // 2858 less the de group's STEP cases (unit 8G2's class)
        // 109 with 8G1 alone; 120 with 8F's commands; 268 with 8G2's ReadStep; 284 with 8W's commands (exec, ReadFile,
        // WriteFile, xnorm), two of which measure memory and are left to DrawCheapGapsCorpusTests (bugs/caf/bug31075)
        Assert.Equal(284, aRunnable.Count);
        Assert.Equal(2, aMemoryCases.Count(aRunnable.Contains));
        // OK 63 (xcaf's BRep grids 44, bugs/xde 11, de_mesh's glTF 5, three others); SKIPPED 37 (data not public);
        // FAILED 9, attributed. Most of de, xcaf and bugs/xde wait for STEP (unit 8G2), IGES, OCAF storage, the
        // parameter commands (unit 8F), GD&T, notes and views.
        // with unit 8G2's ReadStep and WriteStep: OK 141, SKIPPED 112; with unit 8W's commands and Tcl's encodings:
        // OK 153, SKIPPED 119, FAILED 10 (attributed), the two cases measuring memory left out
        Assert.Equal(153, aRuns.Count(r => r.Status == "OK"));
        Assert.Equal(119, aRuns.Count(r => r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)));
        Assert.All(aRuns, r => Assert.True(r.Status == "OK" || r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)
                                           || r.Status.StartsWith("BAD", StringComparison.Ordinal) || KnownFailures.Contains(r.Case.Id),
                                           $"{r.Case.Id}: {r.Status}"));
    }

    /// <summary>Cases that fail in Draw-lite, each attributed.</summary>
    internal static readonly HashSet<string> KnownFailures = new(StringComparer.Ordinal)
    {
        // unit 8T's glTF reader (decision 1236) turns glTF's Y-up into Z-up in the coordinates, where OCCT's places the
        // scene's nodes at a rotation, so OCCT's document has a reference (INSTANCE) per root node where the tree's has
        // a part; boxinterleaved's unnamed root node holding a rotation is also folded into its child
        "de_mesh/gltf_read/boxinterleaved",
        "de_mesh/gltf_read/cubeemb",
        // runnable with unit 8G2's ReadStep: `incmesh` meshes the shape XGetOneShape gives, which in OCCT meshes the
        // document's shapes in place; the document's shapes are frozen (design 4.1), so WriteGltf finds no triangles
        "de_mesh/gltf_write/as1",
        // OCCT's glTF writer leaves out nodes without triangles (a part not meshed, an empty compound) and its reader
        // names an unnamed root after the file and unnamed instances by their shape type; unit 8T's writer and reader don't
        "de_mesh/gltf_write/empty",
        // the sample script it sources (samples/tcl/vis_pbr_spheres.tcl) calls XFindComponent and the material
        // commands, which Draw-lite's documents don't have (decision 1259); the scan doesn't follow source
        "de_mesh/gltf_write/spheres",
        // runnable since vcaps and vlight are ignored (viewer commands): the sample scripts they source call viewer
        // commands Draw-lite hasn't (vbackground, text2brep), which the scan doesn't see
        "demo/samples/dimensions",
        "demo/samples/dimensionsglsl",
        "demo/samples/dimensionspbr",
        "demo/samples/pencil",
        "v3d/materials/raytrace1",
    };
}
