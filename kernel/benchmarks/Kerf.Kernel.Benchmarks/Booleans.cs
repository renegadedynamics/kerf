// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Diagnostics;
using System.Globalization;
using System.IO.Compression;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using Kerf.Engine;

namespace Kerf.Kernel.Benchmarks;

/// <summary>One Boolean to time: its arguments and tools as BRep text (a compound, the arguments first).</summary>
public sealed record BooleanCase(string Group, string Name, string ArgsText, int NbObjects, string Op, double Fuzzy, bool Glue, bool Clean);

/// <summary>
/// The Booleans benchmark (plan 2.9, unit 6M): the Booleans, serially and in parallel, with the clean Kerf.Modeling makes
/// after each (<c>UnifySameDomain(shape, true, true, true)</c> without internal edges), and their allocation. Two groups of
/// inputs:
/// <list type="bullet">
/// <item><c>simple</c>: the corpus's <c>bopfuse_simple</c> grid (tools/pfrec's recording of its arguments, decision 785);</item>
/// <item><c>complex</c>: the corpus's <c>bfuse_complex</c>, <c>bcut_complex</c> and <c>bcommon_complex</c> cases that restore
/// two shapes from the public test data and run one Boolean on them.</item>
/// <item><c>many</c>, only when the filter asks for it, so the rig's rows stay comparable: many tools at once.</item>
/// </list>
/// The arguments are frozen, as a caller's are. Each timing is a median after a warm-up. Run in Release:
/// <c>-- booleans [filter]</c> (a filter on the group or the name), or <c>-- booleans-profile [filter]</c> for the time
/// split between the pave filler and the builder.
/// </summary>
public static class Booleans
{
    private static string Metadata(string theKey)
        => typeof(Booleans).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    // ---------------------------------------------------------------------------------------------------------------
    // The cases

    public static IEnumerable<BooleanCase> All()
    {
        foreach (BooleanCase aCase in SimpleCases())
        {
            yield return aCase;
        }
        foreach (BooleanCase aCase in ComplexCases())
        {
            yield return aCase;
        }
    }


    /// <summary>
    /// Many tools in one Boolean: a plate with 100 holes, and the corpus's bug25614_fuse, a fuse of 121 boxes with 100 cylinders.
    /// </summary>
    private static IEnumerable<BooleanCase> ManyCases()
    {
        static string Text(IEnumerable<Shape> theShapes)
        {
            var aWriter = new StringWriter();
            BrepTools.Write(Builder.MakeCompound(theShapes), aWriter);
            return aWriter.ToString();
        }
        var aPlate = new List<Shape> { new MakeBox(new Point3D(0, 0, -2), 110, 110, 4).Shape() };
        var aBoxes = new List<Shape>();
        var aCylinders = new List<Shape>();
        for (int i = 0; i < 11; i++)
        {
            for (int j = 0; j < 11; j++)
            {
                aBoxes.Add(new MakeBox(new Point3D(i * 10, j * 10, -2), 10, 10, 4).Shape());
            }
        }
        for (int i = 0; i < 10; i++)
        {
            for (int j = 0; j < 10; j++)
            {
                var aFrame = new Frame3D(new Point3D(10 + i * 10, 10 + j * 10, -5), new Direction3D(0, 0, 1));
                aCylinders.Add(new ApiMakeCylinder(aFrame, 6, 10).Shape());
                aPlate.Add(new ApiMakeCylinder(aFrame, 3, 10).Shape());
            }
        }
        yield return new BooleanCase("many", "plate with 100 holes", Text(aPlate), 1, "Cut", 0, false, false);
        yield return new BooleanCase("many", "121 boxes and 100 cylinders", Text(aBoxes.Concat(aCylinders)), 121, "Fuse", 0, false, false);
    }

    /// <summary>The corpus's bopfuse_simple grid, from tools/pfrec's recording (every case's arguments as one compound).</summary>
    private static IEnumerable<BooleanCase> SimpleCases()
    {
        using Stream aRes = typeof(Booleans).Assembly.GetManifestResourceStream("BOPAlgo.PaveFillerRecordings.txt.gz")!;
        using GZipStream aZip = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aZip, Encoding.Latin1);
        string aText = aReader.ReadToEnd();
        int aPos = 0;
        string NextLine()
        {
            int anEnd = aText.IndexOf('\n', aPos);
            string aLine = aText[aPos..anEnd];
            aPos = anEnd + 1;
            return aLine;
        }
        var aCases = new List<BooleanCase>();
        while (aPos < aText.Length)
        {
            string[] aHead = NextLine().Split(' ');
            string aShapesLine = NextLine();
            int aLength = int.Parse(aShapesLine[7..], CultureInfo.InvariantCulture);
            string aShapes = aText.Substring(aPos, aLength);
            aPos += aLength + 1;
            double aFuzzy = 0.0;
            if (aText.AsSpan(aPos).StartsWith("fuzzy "))
            {
                aFuzzy = BitConverter.Int64BitsToDouble(long.Parse(NextLine()[6..], NumberStyles.HexNumber, CultureInfo.InvariantCulture));
            }
            for (int m = 0; m < 2; ++m)
            {
                NextLine();
                while (NextLine() != "end")
                {
                }
            }
            if (aHead[2].StartsWith("bopfuse_simple/", StringComparison.Ordinal))
            {
                // the recorded compound holds the arguments of bop a b: the first is the object
                aCases.Add(new BooleanCase("simple", aHead[2], aShapes, 1, "Fuse", aFuzzy, false, false));
            }
        }
        return aCases;
    }

    private static readonly Regex s_restore = new(@"^restore \[locate_data_file (\S+)\] (\S+)\s*$");
    private static readonly Regex s_operation = new(@"^b(fuse|cut|common) (\S+) (\S+) (\S+)\s*$");
    private static readonly Regex s_ignored = new(@"^\s*$|^\s*#|^(checkprops|checkview|checknbshapes|checkshape|puts|set|cpulimit) ");

    /// <summary>
    /// The corpus's complex grids' cases that restore two shapes from the public test data and run one Boolean on them:
    /// both shapes read natively and written as one compound.
    /// </summary>
    private static IEnumerable<BooleanCase> ComplexCases()
    {
        string aTests = Path.Combine(Metadata("OcctSourceDir"), "tests", "boolean");
        string aData = Metadata("OcctTestDataDir");
        if (!Directory.Exists(aTests) || !Directory.Exists(aData))
        {
            yield break;
        }
        var aFiles = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        foreach (string aFile in Directory.EnumerateFiles(aData, "*", SearchOption.AllDirectories))
        {
            aFiles.TryAdd(Path.GetFileName(aFile), aFile);
        }
        foreach (string aGrid in new[] { "bfuse_complex", "bcut_complex", "bcommon_complex" })
        {
            foreach (string aScript in Directory.EnumerateFiles(Path.Combine(aTests, aGrid)).Order(StringComparer.Ordinal))
            {
                var aRestored = new Dictionary<string, string>(StringComparer.Ordinal);
                Match? anOperation = null;
                bool isPlain = true;
                foreach (string aLine in File.ReadLines(aScript))
                {
                    Match aRestore = s_restore.Match(aLine);
                    Match anOp = s_operation.Match(aLine);
                    if (aRestore.Success && anOperation is null)
                    {
                        string aName = aRestore.Groups[1].Value;
                        aRestored[aRestore.Groups[2].Value] = aFiles.GetValueOrDefault(aName) ?? aFiles.GetValueOrDefault(aName + ".brep")
                                                              ?? aFiles.GetValueOrDefault(aName + ".rle") ?? "";
                    }
                    else if (anOp.Success && anOperation is null)
                    {
                        anOperation = anOp;
                    }
                    else if (!s_ignored.IsMatch(aLine))
                    {
                        isPlain = false;
                    }
                }
                if (!isPlain || anOperation is null || aRestored.Count != 2
                    || !aRestored.TryGetValue(anOperation.Groups[3].Value, out string? anA) || anA.Length == 0
                    || !aRestored.TryGetValue(anOperation.Groups[4].Value, out string? aB) || aB.Length == 0)
                {
                    continue;
                }
                string? aText = CompoundText(anA, aB);
                if (aText is not null)
                {
                    string anOp = anOperation.Groups[1].Value;
                    yield return new BooleanCase("complex", $"{aGrid}/{Path.GetFileName(aScript)}", aText, 1,
                                                 anOp == "fuse" ? "Fuse" : anOp == "cut" ? "Cut" : "Common", 0, false, false);
                }
            }
        }
    }

    // The two files' shapes as one compound, written by the port (both sides read it)
    private static string? CompoundText(string theA, string theB)
    {
        if (!BrepTools.Read(out Shape aA, theA) || !BrepTools.Read(out Shape aB, theB) || aA.IsNull || aB.IsNull)
        {
            return null;
        }
        var aWriter = new StringWriter();
        BrepTools.Write(Builder.MakeCompound([aA, aB]), aWriter);
        return aWriter.ToString();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The two sides

    private static (List<Shape> Objects, List<Shape> Tools) PortArgs(BooleanCase theCase)
    {
        BrepTools.Read(out Shape aC, new StringReader(theCase.ArgsText));
        // read shapes are frozen (TopTools_ShapeSet), as a caller's
        var anObjects = new List<Shape>();
        var aTools = new List<Shape>();
        int i = 0;
        for (var anIt = new Iterator(aC); anIt.More(); anIt.Next())
        {
            (i++ < theCase.NbObjects ? anObjects : aTools).Add(anIt.Value());
        }
        return (anObjects, aTools);
    }

    private static BuilderAlgo PortOperation(BooleanCase theCase, List<Shape> theObjects, List<Shape> theTools,
                                                         bool theParallel)
    {
        BuilderAlgo anOp = theCase.Op switch
        {
            "Fuse" => new Fuse(),
            "Cut" => new Cut(),
            "Common" => new Common(),
            "Section" => new SectionOperation(),
            _ => new SplitterOperation(),
        };
        var anArgs = new NCollection_List<Shape>();
        var aTools = new NCollection_List<Shape>();
        theObjects.ForEach(s => anArgs.Append(s));
        theTools.ForEach(s => aTools.Append(s));
        anOp.SetArguments(anArgs);
        if (anOp is BooleanOperation aBop)
        {
            aBop.SetTools(aTools);
        }
        else
        {
            ((SplitterOperation)anOp).SetTools(aTools);
        }
        anOp.SetRunParallel(theParallel);
        anOp.SetNonDestructive(true);
        anOp.SetFuzzyValue(theCase.Fuzzy);
        anOp.SetGlue(theCase.Glue ? GlueMode.Shift : GlueMode.Off);
        return anOp;
    }

    private static Shape PortRun(BooleanCase theCase, List<Shape> theObjects, List<Shape> theTools, bool theParallel)
    {
        BuilderAlgo anOp = PortOperation(theCase, theObjects, theTools, theParallel);
        anOp.Build();
        return anOp.HasErrors ? new Shape() : anOp.Shape();
    }

    private static Shape PortClean(in Shape theShape)
    {
        var aU = new UnifySameDomain(theShape, true, true, true);
        aU.AllowInternalEdges(false);
        aU.Build();
        return aU.Shape();
    }


    // ---------------------------------------------------------------------------------------------------------------
    // The timings

    private static double Median(List<double> theTimes)
    {
        theTimes.Sort();
        int n = theTimes.Count;
        return n % 2 == 1 ? theTimes[n / 2] : 0.5 * (theTimes[n / 2 - 1] + theTimes[n / 2]);
    }

    private static double Ms(long theTicks) => theTicks * 1000.0 / Stopwatch.Frequency;

    /// <summary>The median of <paramref name="theRun"/>'s milliseconds after a warm-up: at least 3 runs, up to 0.5 s or 15 runs.</summary>
    private static double Measure(Action theRun)
    {
        theRun();
        var aTimes = new List<double>();
        var aClock = Stopwatch.StartNew();
        for (int i = 0; i < 15 && (i < 3 || aClock.Elapsed.TotalSeconds < 0.5); i++)
        {
            long t0 = Stopwatch.GetTimestamp();
            theRun();
            aTimes.Add(Ms(Stopwatch.GetTimestamp() - t0));
        }
        return Median(aTimes);
    }

    private sealed class Totals
    {
        public int Cases;
        public double PortSerial, PortParallel, PortClean, AllocatedMB, CleanAllocatedMB;
        public long Faces;
        public readonly List<(string Name, double Ms)> Slowest = [];
    }

    public static void Run(string? theFilter)
    {
        Console.WriteLine($".NET {Environment.Version}, {Environment.ProcessorCount} logical processors, {(Debugger.IsAttached ? "debugger" : "no debugger")}");
        Console.WriteLine("Medians in ms of the Boolean (its arguments frozen) and of Kerf.Modeling's clean of the result. "
                          + "The MB are allocated by one serial Boolean (and one clean).");
        var aTotals = new SortedDictionary<string, Totals>(StringComparer.Ordinal);
        foreach (BooleanCase aCase in theFilter == "many" ? ManyCases() : All())
        {
            if (theFilter is not null && !aCase.Group.Contains(theFilter, StringComparison.OrdinalIgnoreCase)
                && !aCase.Name.Contains(theFilter, StringComparison.OrdinalIgnoreCase))
            {
                continue;
            }
            (List<Shape> aPObjects, List<Shape> aPTools) = PortArgs(aCase);
            Shape aPResult = PortRun(aCase, aPObjects, aPTools, false);
            long aBytes = GC.GetTotalAllocatedBytes(true);
            PortRun(aCase, aPObjects, aPTools, false);
            double anAllocated = (GC.GetTotalAllocatedBytes(true) - aBytes) / 1048576.0;
            double aPortSerial = Measure(() => PortRun(aCase, aPObjects, aPTools, false));
            double aPortParallel = Measure(() => PortRun(aCase, aPObjects, aPTools, true));
            double aPortClean = 0, aCleanAllocated = 0;
            if (!aPResult.IsNull)
            {
                aBytes = GC.GetTotalAllocatedBytes(true);
                PortClean(aPResult);
                aCleanAllocated = (GC.GetTotalAllocatedBytes(true) - aBytes) / 1048576.0;
                aPortClean = Measure(() => PortClean(aPResult));
            }
            if (!aTotals.TryGetValue(aCase.Group, out Totals? aT))
            {
                aTotals[aCase.Group] = aT = new Totals();
            }
            aT.Cases++;
            aT.PortSerial += aPortSerial;
            aT.PortParallel += aPortParallel;
            aT.PortClean += aPortClean;
            aT.AllocatedMB += anAllocated;
            aT.CleanAllocatedMB += aCleanAllocated;
            aT.Slowest.Add((aCase.Name, aPortSerial));
            if (!aPResult.IsNull)
            {
                for (Explorer anExplorer = new(aPResult, ShapeType.Face); anExplorer.More(); anExplorer.Next())
                {
                    aT.Faces++;
                }
            }
        }

        Console.WriteLine();
        Console.WriteLine("| group | cases | serial | parallel | speed-up | clean | MB per Boolean | MB per clean |");
        Console.WriteLine("|---|---:|---:|---:|---:|---:|---:|---:|");
        foreach ((string aGroup, Totals aT) in aTotals)
        {
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"| {aGroup} | {aT.Cases} | {aT.PortSerial:F1} | {aT.PortParallel:F1} | {aT.PortSerial / aT.PortParallel:F2} | {aT.PortClean:F1} | "
                + $"{aT.AllocatedMB / aT.Cases:F2} | {aT.CleanAllocatedMB / aT.Cases:F2} |"));
            string aRow = $"{aGroup}, {aT.Cases} cases";
            BenchRows.Add("booleans", aRow, "serial ms", aT.PortSerial);
            BenchRows.Add("booleans", aRow, "parallel ms", aT.PortParallel);
            BenchRows.Add("booleans", aRow, "speed-up x", aT.PortSerial / aT.PortParallel);
            BenchRows.Add("booleans", aRow, "clean ms", aT.PortClean);
            BenchRows.Add("booleans", aRow, "alloc MB", aT.AllocatedMB);
            BenchRows.Add("booleans", aRow, "clean alloc MB", aT.CleanAllocatedMB);
            BenchRows.Add("booleans", aRow, "result", $"{aT.Faces} faces");
        }
        Console.WriteLine();
        Console.WriteLine("The slowest cases, serially:");
        foreach ((string aGroup, Totals aT) in aTotals)
        {
            foreach ((string aName, double aMs) in aT.Slowest.OrderByDescending(w => w.Ms).Take(5))
            {
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {aGroup} {aName}: {aMs:F2} ms"));
            }
        }
    }

    /// <summary>The serial time split between the pave filler (the intersections) and the builder.</summary>
    public static void Profile(string? theFilter)
    {
        Console.WriteLine("| group | cases | pave filler | builder | total |");
        Console.WriteLine("|---|---:|---:|---:|---:|");
        var aTotals = new SortedDictionary<string, double[]>(StringComparer.Ordinal);
        foreach (BooleanCase aCase in All())
        {
            if ((theFilter is not null && !aCase.Group.Contains(theFilter, StringComparison.OrdinalIgnoreCase)
                 && !aCase.Name.Contains(theFilter, StringComparison.OrdinalIgnoreCase)) || aCase.Op is "Split")
            {
                continue;
            }
            (List<Shape> aPObjects, List<Shape> aPTools) = PortArgs(aCase);
            var anAll = new NCollection_List<Shape>();
            aPObjects.Concat(aPTools).ToList().ForEach(s => anAll.Append(s));
            var aFillers = new List<double>();
            var aBuilders = new List<double>();
            double aTotal = Measure(() =>
            {
                long t0 = Stopwatch.GetTimestamp();
                var aPF = new PaveFiller();
                aPF.SetArguments(anAll);
                aPF.SetNonDestructive(true);
                aPF.SetFuzzyValue(aCase.Fuzzy);
                aPF.SetGlue(aCase.Glue ? GlueMode.Shift : GlueMode.Off);
                aPF.Perform();
                long t1 = Stopwatch.GetTimestamp();
                var aBOP = new Bop();
                aPObjects.ForEach(s => aBOP.AddArgument(s));
                aPTools.ForEach(s => aBOP.AddTool(s));
                aBOP.SetOperation(aCase.Op switch
                {
                    "Fuse" => Operation.Fuse,
                    "Cut" => Operation.Cut,
                    "Common" => Operation.Common,
                    _ => Operation.Section,
                });
                aBOP.PerformWithFiller(aPF);
                long t2 = Stopwatch.GetTimestamp();
                aFillers.Add(Ms(t1 - t0));
                aBuilders.Add(Ms(t2 - t1));
            });
            double aFiller = Median(aFillers), aBuilder = Median(aBuilders);
            if (!aTotals.TryGetValue(aCase.Group, out double[]? aT))
            {
                aTotals[aCase.Group] = aT = new double[4];
            }
            aT[0]++;
            aT[1] += aFiller;
            aT[2] += aBuilder;
            aT[3] += aTotal;
        }
        foreach ((string aGroup, double[] aT) in aTotals)
        {
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"| {aGroup} | {aT[0]} | {aT[1]:F1} | {aT[2]:F1} | {aT[3]:F1} |"));
            string aRow = $"{aGroup}, {aT[0]} cases";
            BenchRows.Add("booleans", aRow, "pave filler ms", aT[1]);
            BenchRows.Add("booleans", aRow, "builder ms", aT[2]);
        }
    }
}
