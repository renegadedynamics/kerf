// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections;
using System.Reflection;
using System.Text.RegularExpressions;
using Kerf.Kernel.DrawLite;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>Phase 8's third milestone: the data-exchange cases (unit 8W).</summary>
public partial class DrawDataExchangeCorpusTests
{
    /// <summary>The groups whose cases are all data exchange.</summary>
    internal static readonly string[] DataExchangeGroups = ["de", "de_mesh", "de_wrapper", "xcaf", "gdt", "metadata"];

    /// <summary>Why a data-exchange case isn't runnable: the first of these its missing commands fall in.</summary>
    internal enum OutOfScope
    {
        OcafStorage,      // OCAF's storage, undo and attributes (DDocStd, DDataStd, DNaming..., XOpen, XSave)
        Iges,             // TKXSDRAWIGES
        Vrml,             // TKXSDRAWVRML
        GdtNotesViewsMaterials, // XDEDRAW_GDTs, _Notes, _Views, the material and metadata commands, the gdt and
                                // metadata groups
        ObjPlyReading,    // ReadObj, readobj (no OBJ reader is ported), TKXSDRAWPLY
        Viewer,           // the viewer's, AIS's and MeshVS's commands, XShow
        Other,
    }

    private static readonly Regex s_registered = new(@"(?:\.Add|AddFunc)\(\s*""([A-Za-z0-9_]+)""", RegexOptions.Compiled);

    // the commands registered in the files under a directory of OCCT's source
    private static HashSet<string> Registered(string theDirectory, string thePattern = "*.cxx")
    {
        var aCommands = new HashSet<string>(StringComparer.Ordinal);
        string aPath = Path.Combine(DrawResourcesTests.OcctSourceDirectory, "src", theDirectory);
        if (!Directory.Exists(aPath))
        {
            return aCommands;
        }
        foreach (string aFile in Directory.EnumerateFiles(aPath, thePattern, SearchOption.AllDirectories))
        {
            foreach (Match aMatch in s_registered.Matches(File.ReadAllText(aFile)))
            {
                aCommands.Add(aMatch.Groups[1].Value);
            }
        }
        return aCommands;
    }

    /// <summary>The data-exchange commands (XSDRAW's plugins, XDEDRAW, XSBase's functions) and the out-of-scope sets.</summary>
    internal sealed class DataExchangeCommands
    {
        public HashSet<string> All { get; } = new(StringComparer.Ordinal);
        public HashSet<string> Ocaf { get; }
        public HashSet<string> Iges { get; }
        public HashSet<string> Vrml { get; }
        public HashSet<string> GdtNotesViews { get; }
        public HashSet<string> ObjPly { get; }

        public DataExchangeCommands()
        {
            foreach (string aToolkit in new[] { "TKXSDRAW", "TKXSDRAWDE", "TKXSDRAWGLTF", "TKXSDRAWIGES", "TKXSDRAWOBJ", "TKXSDRAWPLY",
                                                "TKXSDRAWSTEP", "TKXSDRAWSTL", "TKXSDRAWVRML", "TKXDEDRAW" })
            {
                All.UnionWith(Registered(Path.Combine("Draw", aToolkit)));
            }
            All.UnionWith(Registered(Path.Combine("DataExchange", "TKXSBase")));
            Ocaf = Registered(Path.Combine("Draw", "TKDCAF"));
            Ocaf.UnionWith(["XOpen", "XSave", "Open", "Save", "SaveAs", "Undo", "Redo"]);
            Iges = Registered(Path.Combine("Draw", "TKXSDRAWIGES"));
            Vrml = Registered(Path.Combine("Draw", "TKXSDRAWVRML"));
            GdtNotesViews = Registered(Path.Combine("Draw", "TKXDEDRAW"), "XDEDRAW_GDTs.cxx");
            GdtNotesViews.UnionWith(Registered(Path.Combine("Draw", "TKXDEDRAW"), "XDEDRAW_Notes.cxx"));
            GdtNotesViews.UnionWith(Registered(Path.Combine("Draw", "TKXDEDRAW"), "XDEDRAW_Views.cxx"));
            GdtNotesViews.UnionWith(All.Where(c => c.Contains("Material", StringComparison.Ordinal)));
            GdtNotesViews.Add("XGetProperties");
            ObjPly = Registered(Path.Combine("Draw", "TKXSDRAWPLY"));
            ObjPly.UnionWith(["ReadObj", "readobj"]);
        }

        public bool IsDataExchange(DrawCaseScan theScan)
            => DataExchangeGroups.Contains(theScan.Case.Group) || theScan.Commands.Any(All.Contains);

        public OutOfScope Classify(DrawCaseScan theScan)
        {
            IReadOnlySet<string> aMissing = theScan.Missing;
            if (aMissing.Any(Ocaf.Contains))
            {
                return OutOfScope.OcafStorage;
            }
            if (aMissing.Any(Iges.Contains))
            {
                return OutOfScope.Iges;
            }
            if (aMissing.Any(Vrml.Contains))
            {
                return OutOfScope.Vrml;
            }
            if (aMissing.Any(GdtNotesViews.Contains) || theScan.Case.Group is "gdt" or "metadata")
            {
                return OutOfScope.GdtNotesViewsMaterials;
            }
            if (aMissing.Any(ObjPly.Contains))
            {
                return OutOfScope.ObjPlyReading;
            }
            if (aMissing.Any(c => c is "XShow" or "wavefront" || c.StartsWith("AIS", StringComparison.Ordinal) || c.StartsWith("mesh", StringComparison.Ordinal)
                                  || (c.Length > 1 && c[0] == 'v' && char.IsAsciiLetterLower(c[1]))))
            {
                return OutOfScope.Viewer;
            }
            return OutOfScope.Other;
        }
    }

    /// <summary>
    /// The attributions of the corpus tests (each class's <c>KnownFailures</c>, a set of case names or a map from them):
    /// a case failing in the milestone's run is attributed if one of them names it.
    /// </summary>
    internal static HashSet<string> AllKnownFailures()
    {
        var aKnown = new HashSet<string>(StringComparer.Ordinal);
        foreach (Type aType in typeof(DrawDataExchangeCorpusTests).Assembly.GetTypes())
        {
            FieldInfo? aField = aType.GetField("KnownFailures", BindingFlags.Static | BindingFlags.NonPublic | BindingFlags.Public);
            switch (aField?.GetValue(null))
            {
                case HashSet<string> aSet:
                    aKnown.UnionWith(aSet);
                    break;
                case IDictionary aMap:
                    aKnown.UnionWith(aMap.Keys.Cast<string>());
                    break;
            }
        }
        return aKnown;
    }

    /// <summary>
    /// The milestone: the data-exchange cases (the groups de, de_mesh, de_wrapper, xcaf, gdt and metadata, and every
    /// case naming a command of XSDRAW's plugins, XDEDRAW or XSBase's functions), how many are runnable and why the
    /// others aren't. The runs (about 2,700 cases; a few minutes) are made when asked for, with
    /// <c>OCCTNET_CORPUS_FULL=1</c> or <c>OCCTNET_DE_MILESTONE=1</c> (decision 1020): their statuses, with every failure
    /// attributed by one of the corpus tests. The cases that measure memory run one at a time after the others.
    /// </summary>
    [Fact]
    public void Data_exchange_cases_run_at_DRAWEXEs_rate()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        DrawInterpreter anInterpreter = NewInterpreter();
        var aCommands = new DataExchangeCommands();
        Assert.Contains("ReadStep", aCommands.All);
        Assert.Contains("ReadIges", aCommands.Iges);

        List<DrawCaseScan> aScans = aCorpus.Cases().AsParallel().AsOrdered()
                                           .Select(c => aCorpus.Scan(c, anInterpreter))
                                           .Where(aCommands.IsDataExchange)
                                           .ToList();
        List<DrawCaseScan> aRunnable = aScans.Where(s => s.IsRunnable).ToList();
        var aBlocked = aScans.Where(s => !s.IsRunnable).GroupBy(aCommands.Classify).ToDictionary(g => g.Key, g => g.ToList());
        output.WriteLine($"{aScans.Count} data-exchange cases ({aScans.Count(s => DataExchangeGroups.Contains(s.Case.Group))} in the "
                         + $"data-exchange groups); runnable {aRunnable.Count}; not runnable {aScans.Count - aRunnable.Count}:");
        foreach (OutOfScope aWhy in Enum.GetValues<OutOfScope>())
        {
            List<DrawCaseScan> aCases = aBlocked.GetValueOrDefault(aWhy) ?? [];
            output.WriteLine($"  {aWhy,-24} {aCases.Count}");
            if (aWhy == OutOfScope.Other)
            {
                foreach (IGrouping<string, string> aMissing in aCases.SelectMany(s => s.Missing).GroupBy(c => c)
                                                                     .OrderByDescending(g => g.Count()).ThenBy(g => g.Key, StringComparer.Ordinal).Take(25))
                {
                    output.WriteLine($"      {aMissing.Key,-24} {aMissing.Count()}");
                }
            }
        }

        // measured at unit 8W
        Assert.Equal(3366, aScans.Count);
        Assert.Equal(2174, aRunnable.Count);
        Assert.Equal(new Dictionary<OutOfScope, int>
                     {
                         [OutOfScope.OcafStorage] = 405,
                         [OutOfScope.Iges] = 536,
                         [OutOfScope.Vrml] = 26,
                         [OutOfScope.GdtNotesViewsMaterials] = 108,
                         [OutOfScope.ObjPlyReading] = 23,
                         [OutOfScope.Viewer] = 78,
                         [OutOfScope.Other] = 16, // DE_Wrapper's configuration commands, XCAFDoc_Editor's, a few others
                     },
                     Enum.GetValues<OutOfScope>().ToDictionary(w => w, w => aBlocked.GetValueOrDefault(w)?.Count ?? 0));

        if (Environment.GetEnvironmentVariable("OCCTNET_CORPUS_FULL") != "1" && Environment.GetEnvironmentVariable("OCCTNET_DE_MILESTONE") != "1")
        {
            output.WriteLine("runs skipped: OCCTNET_DE_MILESTONE=1 (or OCCTNET_CORPUS_FULL=1) runs the runnable cases");
            return;
        }

        DrawCaseRun RunOne(DrawTestCase theCase)
            => aCorpus.RunAndCheck(theCase, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                   TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null);
        List<DrawCaseRun> aRuns = [.. aRunnable.Where(s => !DrawCheapGapsCorpusTests.MeasuresMemory(s)).Select(s => s.Case)
                                               .AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2)).Select(RunOne),
                                   .. aRunnable.Where(DrawCheapGapsCorpusTests.MeasuresMemory).Select(s => DrawCheapGapsCorpusTests.RunMemoryCase(aCorpus, s.Case))];
        aRuns.Sort((x, y) => StringComparer.Ordinal.Compare(x.Case.Id, y.Case.Id));
        HashSet<string> aKnown = AllKnownFailures();
        output.WriteLine("statuses: " + Counts(aRuns));
        output.WriteLine("  in the data-exchange groups: " + Counts(aRuns.Where(r => DataExchangeGroups.Contains(r.Case.Group))));
        List<DrawCaseRun> aFailed = aRuns.Where(r => !r.Passed && !r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)).ToList();
        foreach (DrawCaseRun aRun in aFailed)
        {
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}{(aKnown.Contains(aRun.Case.Id) ? "" : "  NOT ATTRIBUTED")}");
            output.WriteLine($"      {FirstFailure(aRun)}");
        }

        // measured at unit 8W: OK 631, BAD 97 (OCCT's TODOs reproduced), SKIPPED 1436 (data not public), FAILED 10,
        // attributed; in the data-exchange groups OK 286, BAD 76, SKIPPED 713, FAILED 9
        Assert.Equal(631, aRuns.Count(r => r.Status == "OK"));
        Assert.Equal(97, aRuns.Count(r => r.Status.StartsWith("BAD", StringComparison.Ordinal)));
        Assert.Equal(1436, aRuns.Count(r => r.Status.StartsWith("SKIPPED", StringComparison.Ordinal)));
        Assert.All(aFailed, r => Assert.True(aKnown.Contains(r.Case.Id), $"{r.Case.Id}: {r.Status}"));
    }
}
