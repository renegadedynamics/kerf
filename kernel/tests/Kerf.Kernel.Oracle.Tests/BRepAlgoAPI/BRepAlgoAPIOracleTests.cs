// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Globalization;
using System.Reflection;
using System.Text;
using Kerf.Kernel.DrawLite;
using Kerf.Kernel.TestHarness;
using Xunit.Abstractions;
using Op = Kerf.Kernel.Oracle.Tests.BooleanOracleTests.Op;
using Run = Kerf.Kernel.Oracle.Tests.BooleanOracleTests.Run;
using Tally = Kerf.Kernel.Oracle.Tests.BooleanOracleTests.Tally;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// The Boolean milestone (unit 6P, plan section 5, milestones 2 and 3): native BRepAlgoAPI against the port on the plan's
/// inputs with what unit 6F's oracle (<see cref="BooleanOracleTests"/>) left out: <c>SimplifyResult</c> (UnifySameDomain
/// on the result, unit 6U), OCCT's destructive mode (native's arguments written, the port's frozen ones never), the
/// inverted-solid check off, and the corpus's <c>bfuzzyvalue</c> and <c>bglue</c> cases; then the determinism test
/// (serial = parallel at several degrees, repeatedly, with native's own repeatability measured).
/// </summary>
/// <remarks>
/// Every comparison goes through <see cref="BooleanOracleTests.Compare"/>: BRep text with the Free column masked, then
/// canonically; histories (Modified, Generated, IsDeleted) of every argument sub-shape through the indices of the
/// results' texts, with the in-place entries of decision 643 identified exactly; the port's arguments checked
/// unchanged, its result frozen, its parallel runs (degrees 2, 3 and the processor count) equal to its serial run.
/// </remarks>
public class BRepAlgoAPIOracleTests(ITestOutputHelper output)
{
    private static string Metadata(string theKey) =>
        typeof(BRepAlgoAPIOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    [Fact]
    public void SimplifyResult_on_random_primitives_as_native()
    {
        var aHarness = new BooleanOracleTests(output);
        var anOracle = new Oracle(output, 20261110);
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "bop");
        foreach (Run aCase in Tape.Budget(BooleanOracleTests.RandomRuns(anOracle, aRun, 240)))
        {
            aHarness.Compare(aRun, aTally, aCase with { Simplify = true });
        }
        aHarness.Report(aTally, "SimplifyResult on random primitives");
    }

    [Fact]
    public void SimplifyResult_on_the_boolean_group_and_data_occ_as_native()
    {
        var aHarness = new BooleanOracleTests(output);
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "bop");
        int k = 0;
        foreach (Run aCase in Tape.Budget(BooleanOracleTests.RecordedRuns(), isSpread: true))
        {
            // every third case's own operation, simplified
            if (k++ % 3 == 0)
            {
                aHarness.Compare(aRun, aTally, aCase with { Simplify = true });
            }
        }
        aHarness.Report(aTally, "SimplifyResult on the boolean group's and data/occ's arguments");
    }

    [Fact]
    public void Destructive_mode_and_options_as_native()
    {
        var aHarness = new BooleanOracleTests(output);
        var anOracle = new Oracle(output, 20261111);
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "bop");
        int k = 0;
        foreach (Run aCase in Tape.Budget(BooleanOracleTests.RandomRuns(anOracle, aRun, 160)))
        {
            // native writes into its arguments; the port's are frozen, so its operation stays non-destructive (decision
            // 577); every fourth also simplified (the unifier's unsafe input mode) and every fifth without the check of
            // inverted solids
            aHarness.Compare(aRun, aTally, aCase with { NonDestructive = false, Simplify = k % 4 == 0, CheckInverted = k % 5 != 0 });
            k++;
        }
        aHarness.Report(aTally, "the destructive mode, simplified or not, and the inverted-solid check off");
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The corpus's bfuzzyvalue and bglue cases

    /// <summary>
    /// The corpus cases that set <c>bfuzzyvalue</c> or <c>bglue</c> (48 scripts in tests/bugs and tests/perf): each script
    /// runs in Draw-lite with recording stand-ins for BOPTest's commands (<c>baddobjects</c>, <c>bfillds</c>, <c>bbop</c>,
    /// <c>bbuild</c>, <c>bsplit</c>, <c>bop</c>/<c>bopfuse</c>..., <c>bfuse</c>/<c>bcut</c>/<c>bcommon</c>/<c>bsection</c>,
    /// and the options <c>bfuzzyvalue</c>, <c>bglue</c>, <c>bnondestructive</c>, <c>buseobb</c>, <c>bcheckinverted</c>):
    /// each operation is recorded with its arguments (as BRep text) and options and compared, and the port's result is
    /// bound so that the script goes on. Draw-lite's own Boolean commands are unit 6L's.
    /// </summary>
    [Fact]
    public void Corpus_fuzzy_and_glue_cases_as_native()
    {
        string aSource = Metadata("OcctSourceDir");
        string aData = Metadata("OcctTestDataDir");
        string aTests = Path.Combine(aSource, "tests");
        var aScripts = Directory.EnumerateFiles(aTests, "*", SearchOption.AllDirectories)
                                .Where(f => Path.GetExtension(f).Length == 0
                                            && File.ReadLines(f).Any(l => l.TrimStart().StartsWith("bfuzzyvalue ", StringComparison.Ordinal)
                                                                          || l.TrimStart().StartsWith("bglue ", StringComparison.Ordinal)))
                                .Order(StringComparer.Ordinal).ToList();
        var aHarness = new BooleanOracleTests(output);
        var aTally = new Tally();
        using var aRun = new ShapeFixOracleRun(output, "bop");
        int aRecorded = 0, aScriptsWithRuns = 0;
        var aStops = new List<string>();
        foreach (string aScript in Tape.Budget(aScripts, isSpread: true))
        {
            string aRel = Path.GetRelativePath(aTests, aScript).Replace('\\', '/');
            string[] aParts = aRel.Split('/');
            if (aParts.Length != 3)
            {
                continue;
            }
            List<Run> aRuns = RecordCase(new DrawTestCase(aParts[0], aParts[1], aParts[2], aScript), aTests, aSource, aData, out string? aStop);
            if (aStop is not null)
            {
                aStops.Add($"{aRel}: {aStop}");
            }
            if (aRuns.Count > 0)
            {
                aScriptsWithRuns++;
            }
            foreach (Run aCase in aRuns)
            {
                aRecorded++;
                aHarness.Compare(aRun, aTally, aCase);
            }
        }
        output.WriteLine($"{aScripts.Count} scripts set bfuzzyvalue or bglue; {aScriptsWithRuns} gave {aRecorded} operations");
        foreach (string aStop in aStops)
        {
            output.WriteLine("  stopped early: " + aStop);
        }
        aHarness.Report(aTally, "the corpus's bfuzzyvalue and bglue cases");
        // most cases restore data files the public dataset lacks (OCCT's private test data)
        Assert.True(aRecorded >= 5, "too few corpus operations recorded");
    }

    // The recording state of one script: BOPTest's objects, tools and options.
    private sealed class Recorder
    {
        public readonly List<Shape> Objects = [], Tools = [];
        public List<Shape> FilledObjects = [], FilledTools = [];
        public Shape BopA, BopB;
        public double Fuzzy;
        public GlueMode Glue = GlueMode.GlueOff;
        public bool NonDestructive, UseOBB, CheckInverted = true;
        public readonly List<Run> Runs = [];
        public string Name = "";
    }

    private static List<Run> RecordCase(DrawTestCase theCase, string theTests, string theSource, string theData, out string? theStop)
    {
        var aLog = new StringWriter();
        var anInterp = new DrawInterpreter(aLog);
        anInterp.LoadDrawResources(Path.Combine(theSource, "resources", "DrawResources"));
        anInterp.SetVariable("env(CSF_TestDataPath)", theData.Replace('\\', '/'));
        string aRoot = theSource.Replace('\\', '/').TrimEnd('/');
        anInterp.SetVariable("env(CASROOT)", aRoot);
        anInterp.SetVariable("env(CSF_OCCTResourcePath)", aRoot + "/resources");
        anInterp.SetVariable("env(CSF_OCCTDataPath)", aRoot + "/data");
        var aRec = new Recorder { Name = $"{theCase.Group}/{theCase.Grid}/{theCase.Name}" };
        Register(anInterp, aRec);
        anInterp.Deadline = DateTime.UtcNow + TimeSpan.FromMinutes(5);
        theStop = null;
        try
        {
            new DrawCorpus(theTests).Run(theCase, anInterp);
        }
        catch (Exception e) when (e is not OutOfMemoryException)
        {
            theStop = e.Message.Split('\n')[0];
        }
        return aRec.Runs;
    }

    private static double Real(string theWord) => double.Parse(theWord, NumberStyles.Float, CultureInfo.InvariantCulture);

    private static void Register(DrawInterpreter theInterp, Recorder theRec)
    {
        Shape Get(DrawInterpreter i, string n)
        {
            Shape aS = i.GetShape(n);
            if (aS.IsNull)
            {
                throw new DrawException($"{n} is not a shape");
            }
            return aS;
        }
        void Add(string theName, Func<DrawInterpreter, IReadOnlyList<string>, string> theCommand) => theInterp.Register(theName, (i, a) => theCommand(i, a));

        Add("bclear", (_, _) => { theRec.Objects.Clear(); theRec.Tools.Clear(); return ""; });
        Add("bclearobjects", (_, _) => { theRec.Objects.Clear(); return ""; });
        Add("bcleartools", (_, _) => { theRec.Tools.Clear(); return ""; });
        Add("baddobjects", (i, a) => { theRec.Objects.AddRange(a.Skip(1).Select(n => Get(i, n))); return ""; });
        Add("baddtools", (i, a) => { theRec.Tools.AddRange(a.Skip(1).Select(n => Get(i, n))); return ""; });
        Add("baddcompound", (i, a) => { AddCompound(Get(i, a[1]), theRec.Objects); return ""; });
        Add("baddctools", (i, a) => { AddCompound(Get(i, a[1]), theRec.Tools); return ""; });
        Add("bfuzzyvalue", (_, a) => { theRec.Fuzzy = a.Count > 1 ? Real(a[1]) : theRec.Fuzzy; return ""; });
        Add("bglue", (_, a) => { theRec.Glue = a.Count > 1 ? (GlueMode)int.Parse(a[1], CultureInfo.InvariantCulture) : theRec.Glue; return ""; });
        Add("bnondestructive", (_, a) => { theRec.NonDestructive = a.Count > 1 && a[1] != "0"; return ""; });
        Add("buseobb", (_, a) => { theRec.UseOBB = a.Count > 1 && a[1] != "0"; return ""; });
        Add("bcheckinverted", (_, a) => { theRec.CheckInverted = a.Count > 1 && a[1] != "0"; return ""; });
        Add("brunparallel", (_, _) => "");
        Add("boptions", (_, _) => "");
        Add("bdrawwarnshapes", (_, _) => "");
        // the history is always filled (Compare checks it); the self-interference check is unit 6G's
        Add("setfillhistory", (_, _) => "");
        Add("bopcheck", (_, _) => "");
        Add("bfillds", (_, _) =>
        {
            theRec.FilledObjects = [.. theRec.Objects];
            theRec.FilledTools = [.. theRec.Tools];
            return "";
        });
        Add("bbop", (i, a) =>
        {
            Op anOp = int.Parse(a[2], CultureInfo.InvariantCulture) switch
            {
                0 => Op.Common, 1 => Op.Fuse, 2 => Op.Cut, 3 => Op.Cut21, _ => Op.Section,
            };
            return Record(i, theRec, a[1], theRec.FilledObjects, theRec.FilledTools, anOp);
        });
        Add("bbuild", (i, a) => Record(i, theRec, a[1], [.. theRec.FilledObjects, .. theRec.FilledTools], [], Op.GeneralFuse));
        Add("bsplit", (i, a) => Record(i, theRec, a[1], theRec.FilledObjects, theRec.FilledTools, Op.Splitter));
        Add("bapibop", (i, a) =>
        {
            Op anOp = int.Parse(a[2], CultureInfo.InvariantCulture) switch
            {
                0 => Op.Common, 1 => Op.Fuse, 2 => Op.Cut, 3 => Op.Cut21, _ => Op.Section,
            };
            return Record(i, theRec, a[1], theRec.Objects, theRec.Tools, anOp);
        });
        Add("bapibuild", (i, a) => Record(i, theRec, a[1], [.. theRec.Objects, .. theRec.Tools], [], Op.GeneralFuse));
        Add("bop", (i, a) => { theRec.BopA = Get(i, a[1]); theRec.BopB = Get(i, a[2]); return ""; });
        foreach ((string aName, Op anOp) in new[] { ("bopfuse", Op.Fuse), ("bopcut", Op.Cut), ("boptuc", Op.Cut21), ("bopcommon", Op.Common), ("bopsection", Op.Section) })
        {
            Add(aName, (i, a) => Record(i, theRec, a[1], [theRec.BopA], [theRec.BopB], anOp));
        }
        foreach ((string aName, Op anOp) in new[] { ("bfuse", Op.Fuse), ("bcut", Op.Cut), ("btuc", Op.Cut21), ("bcommon", Op.Common), ("bsection", Op.Section) })
        {
            Add(aName, (i, a) => Record(i, theRec, a[1], [Get(i, a[2])], [Get(i, a[3])], anOp));
        }
    }

    private static void AddCompound(in Shape theC, List<Shape> theList)
    {
        for (Iterator anIt = new(theC); anIt.More(); anIt.Next())
        {
            theList.Add(anIt.Value());
        }
    }

    // Records one operation and binds the port's result (so the script can go on).
    private static string Record(DrawInterpreter theInterp, Recorder theRec, string theResult, List<Shape> theObjects,
                                 List<Shape> theTools, Op theOp)
    {
        if (theObjects.Count == 0 || theObjects.Count + theTools.Count < 2)
        {
            throw new DrawException("not enough arguments");
        }
        Compound aC = Builder.MakeCompound(theObjects.Concat(theTools));
        var aText = new StringWriter();
        BrepTools.Write(aC, aText);
        var aCase = new Run($"{theRec.Name} #{theRec.Runs.Count + 1}", aText.ToString(), theObjects.Count, theOp, theRec.Fuzzy,
                            theRec.Glue, theRec.UseOBB, false, theRec.NonDestructive, theRec.CheckInverted);
        theRec.Runs.Add(aCase);

        // the port's result, bound for the rest of the script
        var anArgs = new List<Shape>(theObjects.Concat(theTools));
        (Shape? aRes, _, string? anErr) = BooleanOracleTests.RunPort(aCase, anArgs, false);
        if (aRes is Shape aShape && anErr is null)
        {
            theInterp.SetShape(theResult, aShape);
        }
        return "";
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Determinism

    /// <summary>
    /// The determinism test (plan 2.4, milestone 3): random primitive Booleans of every operation (some simplified, with
    /// fuzzy values, glue and OBB), each run by the port serially and in parallel at degrees 2, 3, 4, 8 and the processor
    /// count, three times each, requiring identical BRep text and identical histories; native's repeatability measured on
    /// the same cases (five serial and five parallel runs in this process).
    /// </summary>
    [Fact]
    public void Determinism_serial_equals_parallel_at_every_degree_and_repeat()
    {
        var anOracle = new Oracle(output, 20261112);
        using var aRun = new ShapeFixOracleRun(output, "bop");
        int aCases = 0, aPortRuns = 0, aPortDiffer = 0, aNativeVaries = 0, aErrors = 0;
        var aDifferences = new List<string>();
        int[] aDegrees = [.. new[] { 2, 3, 4, 8, Environment.ProcessorCount }.Distinct()];
        int k = 0;
        foreach (Run aCase0 in Tape.Budget(BooleanOracleTests.RandomRuns(anOracle, aRun, 96)))
        {
            Run aCase = aCase0 with { Simplify = k++ % 3 == 0 };
            if (BooleanOracleTests.ReadArgs(aRun, aCase.ArgsText) is not (List<N.TopoDS_Shape> aNArgs, List<Shape> aPArgs))
            {
                continue;
            }
            aCases++;
            string Record((Shape?, History?, string?) r) =>
                (r.Item1 is Shape s ? aRun.PortText(s) : r.Item3 ?? "")
                + DeterminismHarness.HistoryText(aPArgs, r.Item1 ?? new Shape(), r.Item2);
            string aSerial = Record(BooleanOracleTests.RunPort(aCase, aPArgs, false));
            if (aSerial.StartsWith("errors", StringComparison.Ordinal) || aSerial.StartsWith("throws", StringComparison.Ordinal))
            {
                aErrors++;
            }
            for (int aRepeat = 0; aRepeat < 3; aRepeat++)
            {
                aPortRuns++;
                if (Record(BooleanOracleTests.RunPort(aCase, aPArgs, false)) != aSerial)
                {
                    aPortDiffer++;
                    aDifferences.Add($"{aCase.Name}: serial repeat {aRepeat} differs");
                }
                foreach (int aDegree in aDegrees)
                {
                    using (OsdParallel.MaxDegreeOfParallelismScope(aDegree))
                    {
                        aPortRuns++;
                        string aPar = Record(BooleanOracleTests.RunPort(aCase, aPArgs, true));
                        if (aPar != aSerial)
                        {
                            aPortDiffer++;
                            aDifferences.Add($"{aCase.Name} [{aCase.Op}]: degree {aDegree} repeat {aRepeat}: {DeterminismHarness.FirstDifference(aSerial, aPar)}");
                        }
                    }
                }
            }

            // native's own repeatability
            var aNTexts = new HashSet<string>(StringComparer.Ordinal);
            for (int r = 0; r < 10; r++)
            {
                (N.TopoDS_Shape? aNRes, _, string? aNErr) = BooleanOracleTests.RunNative(aCase, aNArgs, r >= 5);
                aNTexts.Add(aNRes is null ? aNErr ?? "" : aRun.NativeText(aNRes));
            }
            if (aNTexts.Count > 1)
            {
                aNativeVaries++;
            }
        }
        output.WriteLine($"determinism: {aCases} cases ({aErrors} errors on both serial runs), {aPortRuns} repeated or parallel port runs "
                         + $"at degrees {string.Join(", ", aDegrees)}: {aPortDiffer} differ from the serial run; native varies over 10 runs on {aNativeVaries}");
        foreach (string aD in aDifferences.Take(30))
        {
            output.WriteLine("  " + aD);
        }
        Assert.Equal(0, aPortDiffer);
    }
}
