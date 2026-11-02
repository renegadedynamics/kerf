// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>
/// Unit 8G2's commands (ReadStep and WriteStep over Draw-lite's documents, through STEPCAFControl) on hand-made
/// documents and on OCCT's corpus: the cases that use them, runnable after the commands and not before, run as OCCT's
/// harness runs them and judged by OCCT's log check.
/// </summary>
[Collection(MeshDefaultsCollection.Name)]
public class DrawStepCafCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>The Draw commands unit 8G2 added.</summary>
    internal static readonly string[] Unit8G2Commands = ["ReadStep", "WriteStep"];

    /// <summary>A case of the de group's STEP grids, whose end script reads and writes with ReadStep and WriteStep.</summary>
    internal static bool IsDeStepCase(DrawTestCase theCase) => theCase.Group == "de" && theCase.Grid.StartsWith("step_", StringComparison.Ordinal);

    private static DrawInterpreter NewInterpreter(TextWriter? theOutput = null)
    {
        var anInterpreter = new DrawInterpreter(theOutput);
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    [Fact]
    public void WriteStep_and_ReadStep_round_trip_a_document()
    {
        DrawInterpreter aDI = NewInterpreter();
        aDI.Eval("box b 10 20 30");
        aDI.Eval("XNewDoc D");
        aDI.Eval("XAddShape D b");
        aDI.Eval("SetName D 0:1:1:1 mybox");
        aDI.Eval("XSetColor D b 1 0 0");
        aDI.Eval("XSetLayer D b layer1");
        string aWrite = aDI.Eval("WriteStep D /tmp/p8g2/d.stp");
        output.WriteLine(aWrite);
        Assert.Contains("Translating document D to STEP", aWrite, StringComparison.Ordinal);
        Assert.Contains("File /tmp/p8g2/d.stp written", aWrite, StringComparison.Ordinal);
        Assert.False(File.Exists("/tmp/p8g2/d.stp")); // nothing written to disk

        string aRead = aDI.Eval("ReadStep R /tmp/p8g2/d.stp");
        Assert.Contains("Document saved with name R", aRead, StringComparison.Ordinal);
        string aStat = aDI.Eval("XStat R");
        output.WriteLine(aStat);
        Assert.Contains("Number of labels with name = ", aStat, StringComparison.Ordinal);
        Assert.Equal("mybox", aDI.Eval("GetName R 0:1:1:1").Trim());
        Assert.Contains("RED", aDI.Eval("XGetAllColors R"), StringComparison.Ordinal);
        Assert.Contains("layer1", aDI.Eval("XGetAllLayers R"), StringComparison.Ordinal);
        Assert.Contains("SOLID     : 1", aDI.Eval("XGetOneShape s R; nbshapes s"), StringComparison.Ordinal);
        Assert.Contains("'b' is not a document", aDI.Eval("catch {WriteStep b /tmp/p8g2/x.stp} e; set e"), StringComparison.Ordinal);
    }

    [Fact]
    public void Corpus_cases_using_the_document_STEP_commands_run_after_8G2()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        DrawInterpreter aBefore = NewInterpreter();
        foreach (string aCommand in Unit8G2Commands)
        {
            aBefore.Eval($"rename {aCommand} {{}}");
        }

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(s => Unit8G2Commands.Any(s.Commands.Contains))
                                           .ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        string? aFilter = Environment.GetEnvironmentVariable("OCCTNET_8G2_FILTER");
        var aRunnableBefore = new HashSet<string>(aScans.Select(s => s.Case).Where(c => aCorpus.Scan(c, aBefore).IsRunnable).Select(c => c.Id),
                                                  StringComparer.Ordinal);
        // (the de group's STEP cases read and write through its end script's $ReadCommand and $WriteCommand:
        // DrawCorpus.DataFileCommands)
        List<DrawTestCase> aGained = aRunnable.Where(c => !aRunnableBefore.Contains(c.Id)).ToList();
        output.WriteLine($"{aScans.Count} cases name ReadStep or WriteStep; runnable before 8G2: {aRunnableBefore.Count}, "
                         + $"after: {aRunnable.Count}, made runnable by 8G2: {aGained.Count}");

        var aRuns = aGained.Where(c => aFilter is null || c.Id.Contains(aFilter, StringComparison.Ordinal))
                           .AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                           .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                            TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                           .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                           .ToList();
        output.WriteLine("after 8G2: " + string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0])
                                                                .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                                .Select(g => $"{g.Key} {g.Count()}")));
        foreach (DrawCaseRun aRun in aRuns)
        {
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}");
            if (!aRun.Passed)
            {
                string[] aLines = aRun.Log.Split('\n');
                List<string> aFaulty = aLines.Where(l => !l.StartsWith("TODO", StringComparison.Ordinal)
                                                         && (l.Contains("rror", StringComparison.Ordinal)
                                                             || l.Contains("xception", StringComparison.Ordinal)
                                                             || l.Contains("Faulty", StringComparison.Ordinal)))
                                             .Take(6).ToList();
                if (aFilter is not null)
                {
                    aFaulty = [.. aLines]; // a filtered run shows the whole log
                }
                foreach (string aLine in aFaulty.Count > 0 ? aFaulty : [aLines.Reverse().Skip(1).FirstOrDefault() ?? ""])
                {
                    output.WriteLine($"      {aLine}");
                }
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

    // the corpus's counts after unit 8G2: OK 255, BAD 76 (OCCT's TODOs reproduced), SKIPPED 660 (data not public),
    // FAILED 8, attributed
    private const int GainedCount = 999;
    private const int OkCount = 255;

    /// <summary>Cases that fail in Draw-lite, each attributed.</summary>
    internal static readonly HashSet<string> KnownFailures = new(StringComparer.Ordinal)
    {
        // they write their STEP files with Tcl's encodings (`encoding convertfrom`, `fconfigure -encoding`), which
        // Draw-lite's Tcl has as stubs: the names they compare are not the ones written (unit 8G2)
        "bugs/step/bug28454_1",
        "bugs/step/bug30694",
        "bugs/step/bug31670",
        "bugs/step/bug31670_1",
        "bugs/step/bug31923",
        // GD&T (not read: decision 13504): OCCT's GD&T reading adds sub-shape labels for the faces its tolerances
        // refer to, which the end script counts (LABELS)
        "de/step_1/ZQ2",
        "de/step_5/B8",
        // `incmesh` meshes the shape XGetOneShape gives, which in OCCT meshes the document's shapes in place; the
        // document's shapes are frozen (design 4.1), so WriteGltf finds no triangles
        "de_mesh/gltf_write/as1",
    };
}
