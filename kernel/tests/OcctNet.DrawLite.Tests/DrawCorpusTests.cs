// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>Scans OCCT's own test corpus to show how much of it the port could run, and what to implement next.</summary>
[Collection(MeshDefaultsCollection.Name)]
public class DrawCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    [Fact]
    public void Every_case_parses_and_the_report_lists_missing_commands()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");

        var aCorpus = new DrawCorpus(TestsDirectory);
        var anInterpreter = new DrawInterpreter();
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        var aScans = aCorpus.Cases().AsParallel().AsOrdered().Select(c => aCorpus.Scan(c, anInterpreter)).ToList();

        var aMissing = aScans.SelectMany(s => s.Missing)
                             .GroupBy(c => c)
                             .Select(g => (Command: g.Key, Cases: g.Count()))
                             .OrderByDescending(x => x.Cases)
                             .ThenBy(x => x.Command, StringComparer.Ordinal)
                             .ToList();

        output.WriteLine($"{aScans.Count} cases, {aScans.Count(s => s.IsRunnable)} runnable with the commands implemented so far");
        output.WriteLine($"{aScans.Count(s => s.Missing.Count == 1)} cases miss exactly one command; {aMissing.Count} distinct commands are missing");
        output.WriteLine("Most-needed commands (cases that use each):");
        foreach (var (aCommand, aCases) in aMissing.Take(60))
        {
            output.WriteLine($"  {aCommand,-24} {aCases}");
        }

        Assert.True(aScans.Count > 10000, "OCCT V8_0_1 has over 10,000 test cases");
        var anUnparsable = aScans.Where(s => s.Commands.Contains("<unparsable>")).Select(s => s.Case.Id).ToList();
        output.WriteLine($"{anUnparsable.Count} unparsable: {string.Join(", ", anUnparsable.Take(10))}");
        Assert.True(anUnparsable.Count < aScans.Count / 100, "the parser should read at least 99% of the corpus");

        // Tcl's own commands are all implemented, so what is missing is Draw's (geometry and viewer) commands
        string[] aTclCore = ["if", "expr", "proc", "lindex", "uplevel", "upvar", "string", "regexp", "foreach", "dict", "format", "catch", "global", "open"];
        Assert.DoesNotContain(aMissing, m => aTclCore.Contains(m.Command));
    }

    [Fact]
    public void Grids_with_a_cases_list_read_their_cases_from_the_directory_it_names()
    {
        var aCorpus = new DrawCorpus(TestsDirectory);
        List<DrawTestCase> aMesh = aCorpus.Cases().Where(c => c.Group == "mesh").ToList();

        // three grids on data/standard's 212 cases, three on data/advanced's 21
        Assert.Equal(3 * 212 + 3 * 21, aMesh.Count);
        DrawTestCase aCase = aMesh.Single(c => c.Grid == "standard_incmesh" && c.Name == "A1");
        Assert.Equal(Path.GetFullPath(Path.Combine(TestsDirectory, "mesh", "data", "standard", "A1")), aCase.Path);
        Assert.True(aCorpus.IsListed(aCase));
        Assert.False(aCorpus.IsListed(aCorpus.Cases().First(c => c.Group == "bugs")));

        // the grid's own begin and end scripts run around the case
        string aScript = aCorpus.Script(aCase);
        Assert.Contains("set command incmesh", aScript, StringComparison.Ordinal);
        Assert.Contains("set TheFileName shading_001.brep", aScript, StringComparison.Ordinal);
        Assert.Contains("tricheck res", aScript, StringComparison.Ordinal);

        // the scan before unit 4L1 didn't see them
        Assert.DoesNotContain(new DrawCorpus(TestsDirectory) { ReadCaseLists = false }.Cases(), c => c.Group == "mesh");
    }

    /// <summary>The Draw commands unit 3L1 added (dset and protect were no-ops before it).</summary>
    internal static readonly string[] Unit3L1Commands =
    [
        "bsave", "brestore", "isdraw", "isprot", "dtyp", "dval", "dname", "dump", "copy", "renamevar", "unprotect",
        "autodisplay", "cos", "sin", "tan", "acos", "asin", "atan2", "sqrt", "isos", "hlr", "vori", "vconn", "triangles",
        "polygons", "discretisation", "compound", "add", "explode", "nexplode", "exwire", "emptycopy", "orientation",
        "treverse", "complement", "invert", "nbshapes", "numshapes", "countshapes", "setflags", "purgemmgt",
        "writebrep", "binsave", "readbrep", "binrestore", "reset", "tmove", "ttranslate", "trotate", "tmirror",
        "tscale", "bmove", "fmove", "btranslate", "brotate", "bmirror", "fmirror", "bscale", "fscale", "tcopy",
        "maxtolerance", "purgeloc", "checkloc", "lprops", "sprops", "vprops", "vpropsgk", "tolerance",
    ];

    /// <summary>The Draw commands unit 3L2a added.</summary>
    internal static readonly string[] Unit3L2aCommands =
    [
        "checkshape", "checksection", "computetolerance", "bounding", "isbbinterf", "gbounding", "getcoords", "bclassify",
        "b2dclassify", "point",
    ];

    /// <summary>Commands that were no-ops before unit 3L2a and now work (dlog also turns on the command log).</summary>
    internal static readonly string[] Unit3L2aFormerNoOps = ["dlog", "dchrono", "chrono"];

    /// <summary>Draw-lite as unit 3L1 left it: without 3L2a's commands, its former no-ops back, and no command log.</summary>
    internal static void Before3L2a(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit3L2aCommands)
        {
            theInterpreter.Eval($"rename {aCommand} {{}}");
        }

        foreach (string aCommand in Unit3L2aFormerNoOps)
        {
            theInterpreter.Register(aCommand, static (_, _) => "");
        }

        theInterpreter.DoLog = false;
    }

    /// <summary>The Draw commands unit 3M added (the primitives and sweeps; bounding's -shape now makes its box).</summary>
    internal static readonly string[] Unit3MCommands =
    [
        "box", "wedge", "pcylinder", "pcone", "psphere", "ptorus", "halfspace", "prism", "revol",
    ];

    /// <summary>Draw-lite before unit 3M: without its commands.</summary>
    internal static void Before3M(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit3MCommands)
        {
            theInterpreter.Eval($"rename {aCommand} {{}}");
        }
    }

    /// <summary>The Draw commands unit 3L2b added: geometry, construction, distances and modifications.</summary>
    internal static readonly string[] Unit3L2bCommands =
    [
        // GeomliteTest's curves and surfaces
        "line", "circle", "ellipse", "parabola", "hyperbola", "beziercurve", "bsplinecurve", "pbsplinecurve",
        "2dbeziercurve", "2dbsplinecurve", "2dpbsplinecurve", "reverse", "segment", "setorigin", "cvalue", "2dcvalue",
        "coord", "plane", "cone", "cylinder", "sphere", "torus", "beziersurf", "bsplinesurf", "upbsplinesurf",
        "vpbsplinesurf", "uvpbsplinesurf", "extsurf", "revsurf", "offset", "trim", "trimu", "trimv", "ureverse",
        "vreverse", "uiso", "viso", "svalue", "sderivative", "parameters", "bounds", "length",
        // GeometryTest's gcarc and extrema
        "gcarc", "extrema",
        // DrawTrSurf's transformations
        "translate", "rotate", "pmirror", "lmirror", "smirror", "pscale", "2dtranslate", "2drotate", "2dpmirror",
        "2dlmirror", "2dpscale",
        // BRepTest_ExtremaCommands
        "dist", "distmini", "proximity", "selfintersect",
        // BRepTest_CurveCommands
        "vertex", "edge", "mkedge", "wire", "polyline", "polyvertex", "mkcurve", "mkoricurve", "mkpoint", "mk2dcurve",
        "uisoedge", "visoedge", "profile",
        // BRepTest_SurfaceCommands
        "mkface", "mkshell", "mksurface", "mkplane", "pcurve", "encoderegularity", "getedgeregularity", "projponf",
        // BRepTest_BasicCommands
        "nurbsconvert", "deform", "scalexyz", "sameparameter", "fsameparameter", "updatetolerance", "solidorientation",
        "mkedgecurve", "findplane", "compare", "issubshape", "wexplo",
    ];

    /// <summary>Draw-lite as unit 3L2a left it: without 3L2b's commands and the sources getsourcefile reads.</summary>
    internal static void Before3L2b(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit3L2bCommands)
        {
            theInterpreter.Eval($"rename {aCommand} {{}}");
        }

        theInterpreter.Eval("unset -nocomplain Draw_Files");
    }

    /// <summary>The Draw commands unit 4L1 added: the readers of the meshes shapes carry.</summary>
    internal static readonly string[] Unit4L1Commands =
    [
        "tclean", "trinfo", "triepoints", "trmergenodes", "correctnormals", "mpparallel", "triarea", "tricheck",
    ];

    /// <summary>Draw-lite as phase 3 left it: without 4L1's commands.</summary>
    internal static void Before4L1(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit4L1Commands)
        {
            theInterpreter.Eval($"rename {aCommand} {{}}");
        }
    }

    /// <summary>
    /// The Draw commands unit 4L2 added: the mesher's commands, and the viewer commands that mesh what they display
    /// (vinit, vdisplay, vsetdispmode and vclose were no-ops before it, and vglinfo, an ignored viewer query, is new).
    /// </summary>
    internal static readonly string[] Unit4L2Commands =
    [
        "incmesh", "tessellate", "mpnames", "mpsetdefaultname", "mpgetdefaultname", "mpincmesh", "vdisplay",
        "vdisplayall", "vsetdispmode", "vunsetdispmode", "vdefaults", "verase", "vremove", "vclear", "vglinfo",
    ];

    /// <summary>The viewer commands that were no-ops before unit 4L2.</summary>
    internal static readonly string[] Unit4L2FormerNoOps = ["vinit", "vdisplay", "vclear", "vsetdispmode", "vclose"];

    /// <summary>
    /// Draw-lite as unit 4L1 left it: without 4L2's commands, its former no-ops back. (mpparallel now sets the
    /// mesher's process-wide default; the meshes don't depend on it, decision 490.)
    /// </summary>
    internal static void Before4L2(DrawInterpreter theInterpreter)
    {
        foreach (string aCommand in Unit4L2Commands)
        {
            theInterpreter.Eval($"rename {aCommand} {{}}");
        }

        foreach (string aCommand in Unit4L2FormerNoOps)
        {
            theInterpreter.Register(aCommand, static (_, _) => "");
        }
    }

    private static readonly System.Text.RegularExpressions.Regex s_failure =
        new(@"\b(?:[Ee]xception|[Ee][Rr][Rr][Oo][Rr]|[Ff][Aa][Ii][Ll](?:[0-9]|[Ee][Dd]|[Uu][Rr][Ee]))\b|could not be found");

    /// <summary>
    /// The elapsed-time limit of each case in the large corpus tests. A wall-clock limit, where DRAWEXE's cpulimit
    /// counts CPU time: in the full suite those tests run side by side, each with half the processors, and cases that take
    /// a minute alone have passed 300 s (bugs/modalg_5/bug25232_9 after unit 7W added its corpus test).
    /// </summary>
    internal const int CorpusTimeLimitSeconds = 900;

    internal static string DataPath
        => string.Join(';', Path.Combine(DrawResourcesTests.OcctSourceDirectory, "data"), DrawResourcesTests.OcctTestDataDirectory)
                 .Replace('\\', '/');

    /// <summary>
    /// Runs every case whose commands are all implemented, as OCCT's harness does, and judges each log with OCCT's
    /// own _check_log; reports the cases runnable before and after unit 4L2 (its commands, and the scan's knowledge of
    /// the mesh group's data-file branch), the statuses, and what blocks the rest. The totals are given for the grids
    /// without a cases.list, for those with one (mesh, heal, chamfer, opengl, xml), and for the mesh group apart.
    /// </summary>
    [Fact]
    public void Runnable_cases_run_and_OCCTs_log_check_judges_them()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");
        var aCorpus = new DrawCorpus(TestsDirectory);
        var anInterpreter = new DrawInterpreter();
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        var aBefore = new DrawInterpreter();
        aBefore.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        Before4L2(aBefore);
        var aCorpusBefore = new DrawCorpus(TestsDirectory) { KnowsDataFileBranches = false };

        List<DrawTestCase> aCases = aCorpus.Cases().ToList();
        HashSet<string> aListed = aCases.Where(aCorpus.IsListed).Select(c => c.Id).ToHashSet(StringComparer.Ordinal);
        var aScans = aCases.AsParallel().AsOrdered().Select(c => aCorpus.Scan(c, anInterpreter)).ToList();
        List<DrawTestCase> aRunnableBefore = aCases.AsParallel().AsOrdered().Where(c => aCorpusBefore.Scan(c, aBefore).IsRunnable).ToList();
        List<DrawTestCase> aRunnable = aScans.Where(s => s.IsRunnable).Select(s => s.Case).ToList();
        output.WriteLine($"{aCases.Count} cases ({aCases.Count - aListed.Count} in the former grids, {aListed.Count} in the cases.list grids)");
        output.WriteLine($"mesh group: {aCases.Count(c => c.Group == "mesh")} cases");
        output.WriteLine($"runnable before 4L2: {aRunnableBefore.Count} ({aRunnableBefore.Count(c => aListed.Contains(c.Id))} in the cases.list grids, "
                         + $"{aRunnableBefore.Count(c => c.Group == "mesh")} in mesh), "
                         + $"now: {aRunnable.Count} ({aRunnable.Count(c => aListed.Contains(c.Id))} in the cases.list grids, "
                         + $"{aRunnable.Count(c => c.Group == "mesh")} in mesh)");

        List<DrawCaseRun> Run(List<DrawTestCase> theCases, Action<DrawInterpreter>? theSetup)
            => theCases.AsParallel().WithDegreeOfParallelism(Math.Max(1, Environment.ProcessorCount / 2))
                       .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DataPath, TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), theSetup))
                       .OrderBy(r => r.Case.Id, StringComparer.Ordinal)
                       .ToList();

        void Statuses(string theWhat, List<DrawCaseRun> theRuns)
        {
            string Counts(IEnumerable<DrawCaseRun> theSome)
            {
                string aCounts = string.Join(", ", theSome.GroupBy(r => r.Status.Split(' ')[0])
                                                          .OrderBy(g => g.Key, StringComparer.Ordinal)
                                                          .Select(g => $"{g.Key} {g.Count()}"));
                return aCounts.Length > 0 ? aCounts : "none";
            }

            output.WriteLine($"{theWhat}: {Counts(theRuns)}");
            output.WriteLine($"  former grids: {Counts(theRuns.Where(r => !aListed.Contains(r.Case.Id)))}");
            output.WriteLine($"  cases.list grids: {Counts(theRuns.Where(r => aListed.Contains(r.Case.Id)))}");
            output.WriteLine($"  mesh group: {Counts(theRuns.Where(r => r.Case.Group == "mesh"))}");
        }

        Statuses("before 4L2", Run(aRunnableBefore, Before4L2));
        List<DrawCaseRun> aRuns = Run(aRunnable, null);
        Statuses("now", aRuns);

        foreach (DrawCaseRun aRun in aRuns)
        {
            output.WriteLine($"  {aRun.Case.Id}: {aRun.Status}");
            if (!aRun.Passed)
            {
                // the first line that made it fail
                string? aLine = aRun.Log.Split('\n').FirstOrDefault(l => s_failure.IsMatch(l));
                output.WriteLine($"      {aLine ?? aRun.Log.Split('\n').Reverse().Skip(1).FirstOrDefault()}");
            }
        }

        // the cases that miss one command: what to implement next
        var aNext = aScans.Where(s => s.Missing.Count == 1)
                          .GroupBy(s => s.Missing.Single())
                          .OrderByDescending(g => g.Count())
                          .Take(15);
        output.WriteLine("Cases missing exactly one command, by command:");
        foreach (var aGroup in aNext)
        {
            output.WriteLine($"  {aGroup.Key,-20} {aGroup.Count()}");
        }

        Assert.True(aRunnable.Count > aRunnableBefore.Count);
        Assert.Contains(aRuns, r => r.Status == "OK");
    }
}
