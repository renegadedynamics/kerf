// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Globalization;
using System.IO.Compression;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using Occt;
#if HAS_NATIVE
using N = Build123d.Occt;
#endif

namespace OcctNet.Benchmarks;

/// <summary>One Boolean to time: its arguments and tools as BRep text (a compound, the arguments first).</summary>
public sealed record BooleanCase(string Group, string Name, string ArgsText, int NbObjects, string Op, double Fuzzy, bool Glue, bool Clean);

/// <summary>
/// The Booleans benchmark (plan 2.9, unit 6M): the port's BRepAlgoAPI against native OCCT's, serially and in parallel,
/// with the clean build123d.NET makes after each (<c>ShapeUpgrade_UnifySameDomain(shape, true, true, true)</c> without
/// internal edges), and the port's allocation. Three groups of inputs:
/// <list type="bullet">
/// <item><c>build123d</c>: every Boolean build123d.NET's own tests make (the milestone's capture, decision 7501), with its
/// options (non-destructive, glue, fuzzy value);</item>
/// <item><c>simple</c>: the corpus's <c>bopfuse_simple</c> grid (tools/pfrec's recording of its arguments, decision 785);</item>
/// <item><c>complex</c>: the corpus's <c>bfuse_complex</c>, <c>bcut_complex</c> and <c>bcommon_complex</c> cases that restore
/// two shapes from the public test data and run one Boolean on them.</item>
/// </list>
/// Both sides read the same text; the port's arguments are frozen (a caller's), native's operation non-destructive.
/// Each timing is a median after a warm-up. Run in Release: <c>-- booleans [filter]</c> (a filter on the group or the
/// name), or <c>-- booleans-profile [filter]</c> for the port's time split between the pave filler and the builder.
/// </summary>
public static class Booleans
{
    private static string Metadata(string theKey)
        => typeof(Booleans).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    // ---------------------------------------------------------------------------------------------------------------
    // The cases

    public static IEnumerable<BooleanCase> All()
    {
#if HAS_NATIVE
        foreach (BooleanCase aCase in Build123dCases())
        {
            yield return aCase;
        }
#endif
        foreach (BooleanCase aCase in SimpleCases())
        {
            yield return aCase;
        }
        foreach (BooleanCase aCase in ComplexCases())
        {
            yield return aCase;
        }
    }

#if HAS_NATIVE
    /// <summary>build123d.NET's tests' Booleans, as the oracle's capture records them.</summary>
    private static IEnumerable<BooleanCase> Build123dCases()
    {
        var aCapture = OcctNet.Oracle.Tests.Build123dBooleanCapture.Capture(f => Console.Error.WriteLine("scenario failed: " + f));
        foreach (OcctNet.Oracle.Tests.Build123dBooleanCapture.Call aCall in aCapture.Calls)
        {
            if (aCall.Kind == OcctNet.Oracle.Tests.Build123dBooleanCapture.Kind.Clean)
            {
                continue;
            }
            using var aBuilder = new N.BRep_Builder();
            var aCompound = new N.TopoDS_Compound();
            aBuilder.MakeCompound(aCompound);
            foreach (N.TopoDS_Shape aShape in aCall.Arguments.Concat(aCall.Tools))
            {
                aBuilder.Add(aCompound, aShape);
            }
            string aFile = Path.Combine(Path.GetTempPath(), $"b3dbench-{Guid.NewGuid():N}.brep");
            N.BRepTools.Write(aCompound, aFile, true, false, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3);
            string aText = File.ReadAllText(aFile);
            File.Delete(aFile);
            yield return new BooleanCase("build123d", aCall.Name, aText, aCall.Arguments.Length, aCall.Kind.ToString(), aCall.Fuzzy,
                                         aCall.Glue, aCall.Clean);
        }
    }
#endif

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
        if (!BRepTools.Read(out TopoDS_Shape aA, theA) || !BRepTools.Read(out TopoDS_Shape aB, theB) || aA.IsNull() || aB.IsNull())
        {
            return null;
        }
        var aWriter = new StringWriter();
        BRepTools.Write(TopoDS_Builder.MakeCompound([aA, aB]), aWriter);
        return aWriter.ToString();
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The two sides

    private static (List<TopoDS_Shape> Objects, List<TopoDS_Shape> Tools) PortArgs(BooleanCase theCase)
    {
        BRepTools.Read(out TopoDS_Shape aC, new StringReader(theCase.ArgsText));
        // read shapes are frozen (TopTools_ShapeSet), as a caller's
        var anObjects = new List<TopoDS_Shape>();
        var aTools = new List<TopoDS_Shape>();
        int i = 0;
        for (var anIt = new TopoDS_Iterator(aC); anIt.More(); anIt.Next())
        {
            (i++ < theCase.NbObjects ? anObjects : aTools).Add(anIt.Value());
        }
        return (anObjects, aTools);
    }

    private static BRepAlgoAPI_BuilderAlgo PortOperation(BooleanCase theCase, List<TopoDS_Shape> theObjects, List<TopoDS_Shape> theTools,
                                                         bool theParallel)
    {
        BRepAlgoAPI_BuilderAlgo anOp = theCase.Op switch
        {
            "Fuse" => new BRepAlgoAPI_Fuse(),
            "Cut" => new BRepAlgoAPI_Cut(),
            "Common" => new BRepAlgoAPI_Common(),
            "Section" => new BRepAlgoAPI_Section(),
            _ => new BRepAlgoAPI_Splitter(),
        };
        var anArgs = new NCollection_List<TopoDS_Shape>();
        var aTools = new NCollection_List<TopoDS_Shape>();
        theObjects.ForEach(s => anArgs.Append(s));
        theTools.ForEach(s => aTools.Append(s));
        anOp.SetArguments(anArgs);
        if (anOp is BRepAlgoAPI_BooleanOperation aBop)
        {
            aBop.SetTools(aTools);
        }
        else
        {
            ((BRepAlgoAPI_Splitter)anOp).SetTools(aTools);
        }
        anOp.SetRunParallel(theParallel);
        anOp.SetNonDestructive(true);
        anOp.SetFuzzyValue(theCase.Fuzzy);
        anOp.SetGlue(theCase.Glue ? BOPAlgo_GlueEnum.BOPAlgo_GlueShift : BOPAlgo_GlueEnum.BOPAlgo_GlueOff);
        return anOp;
    }

    private static TopoDS_Shape PortRun(BooleanCase theCase, List<TopoDS_Shape> theObjects, List<TopoDS_Shape> theTools, bool theParallel)
    {
        BRepAlgoAPI_BuilderAlgo anOp = PortOperation(theCase, theObjects, theTools, theParallel);
        anOp.Build();
        return anOp.HasErrors() ? new TopoDS_Shape() : anOp.Shape();
    }

    private static TopoDS_Shape PortClean(in TopoDS_Shape theShape)
    {
        var aU = new ShapeUpgrade_UnifySameDomain(theShape, true, true, true);
        aU.AllowInternalEdges(false);
        aU.Build();
        return aU.Shape();
    }

#if HAS_NATIVE
    private static (List<N.TopoDS_Shape> Objects, List<N.TopoDS_Shape> Tools) NativeArgs(BooleanCase theCase)
    {
        string aFile = Path.Combine(Path.GetTempPath(), $"b3dbench-{Guid.NewGuid():N}.brep");
        File.WriteAllText(aFile, theCase.ArgsText);
        N.TopoDS_Shape aC = Cases.NativeRead(aFile);
        File.Delete(aFile);
        var anObjects = new List<N.TopoDS_Shape>();
        var aTools = new List<N.TopoDS_Shape>();
        int i = 0;
        for (var anIt = new N.TopoDS_Iterator(aC); anIt.More(); anIt.Next())
        {
            (i++ < theCase.NbObjects ? anObjects : aTools).Add(anIt.Value());
        }
        return (anObjects, aTools);
    }

    private static N.TopoDS_Shape NativeRun(BooleanCase theCase, List<N.TopoDS_Shape> theObjects, List<N.TopoDS_Shape> theTools, bool theParallel)
    {
        using N.BRepAlgoAPI_BuilderAlgo anOp = theCase.Op switch
        {
            "Fuse" => new N.BRepAlgoAPI_Fuse(),
            "Cut" => new N.BRepAlgoAPI_Cut(),
            "Common" => new N.BRepAlgoAPI_Common(),
            "Section" => new N.BRepAlgoAPI_Section(),
            _ => new N.BRepAlgoAPI_Splitter(),
        };
        using var anArgs = new N.TopTools_ListOfShape();
        using var aTools = new N.TopTools_ListOfShape();
        theObjects.ForEach(anArgs.Append);
        theTools.ForEach(aTools.Append);
        anOp.SetArguments(anArgs);
        if (anOp is N.BRepAlgoAPI_BooleanOperation aBop)
        {
            aBop.SetTools(aTools);
        }
        else
        {
            ((N.BRepAlgoAPI_Splitter)anOp).SetTools(aTools);
        }
        anOp.SetRunParallel(theParallel);
        anOp.SetNonDestructive(true);
        anOp.SetFuzzyValue(theCase.Fuzzy);
        anOp.SetGlue(theCase.Glue ? N.BOPAlgo_GlueEnum.BOPAlgo_GlueShift : N.BOPAlgo_GlueEnum.BOPAlgo_GlueOff);
        anOp.Build();
        return anOp.HasErrors() ? new N.TopoDS_Shape() : anOp.Shape();
    }

    private static void NativeClean(N.TopoDS_Shape theShape)
    {
        using var aU = new N.ShapeUpgrade_UnifySameDomain(theShape, true, true, true);
        aU.AllowInternalEdges(false);
        aU.Build();
        aU.Shape().Dispose();
    }
#endif

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
        public double PortSerial, PortParallel, NativeSerial, NativeParallel, PortClean, NativeClean, LogRatio, AllocatedMB, CleanAllocatedMB;
        public readonly List<(string Name, double Port, double Native)> Worst = [];
    }

    public static void Run(string? theFilter)
    {
        Console.WriteLine($".NET {Environment.Version}, {Environment.ProcessorCount} logical processors, {(Debugger.IsAttached ? "debugger" : "no debugger")}");
        Console.WriteLine("Medians in ms of BRepAlgoAPI's Build (the port's arguments frozen, native's operation non-destructive) and of "
                          + "build123d.NET's clean of the result; native: build123d.NET's binding of OCCT 8.0.1 (MSVC). "
                          + "The port's MB are allocated by one serial Boolean (and one clean).");
        var aTotals = new SortedDictionary<string, Totals>(StringComparer.Ordinal);
        foreach (BooleanCase aCase in All())
        {
            if (theFilter is not null && !aCase.Group.Contains(theFilter, StringComparison.OrdinalIgnoreCase)
                && !aCase.Name.Contains(theFilter, StringComparison.OrdinalIgnoreCase))
            {
                continue;
            }
            (List<TopoDS_Shape> aPObjects, List<TopoDS_Shape> aPTools) = PortArgs(aCase);
            TopoDS_Shape aPResult = PortRun(aCase, aPObjects, aPTools, false);
            long aBytes = GC.GetTotalAllocatedBytes(true);
            PortRun(aCase, aPObjects, aPTools, false);
            double anAllocated = (GC.GetTotalAllocatedBytes(true) - aBytes) / 1048576.0;
            double aPortSerial = Measure(() => PortRun(aCase, aPObjects, aPTools, false));
            double aPortParallel = Measure(() => PortRun(aCase, aPObjects, aPTools, true));
            double aPortClean = 0, aCleanAllocated = 0;
            if (!aPResult.IsNull())
            {
                aBytes = GC.GetTotalAllocatedBytes(true);
                PortClean(aPResult);
                aCleanAllocated = (GC.GetTotalAllocatedBytes(true) - aBytes) / 1048576.0;
                aPortClean = Measure(() => PortClean(aPResult));
            }
            double aNativeSerial = aPortSerial, aNativeParallel = aPortParallel, aNativeClean = aPortClean;
#if HAS_NATIVE
            (List<N.TopoDS_Shape> aNObjects, List<N.TopoDS_Shape> aNTools) = NativeArgs(aCase);
            aNativeSerial = Measure(() => NativeRun(aCase, aNObjects, aNTools, false).Dispose());
            aNativeParallel = Measure(() => NativeRun(aCase, aNObjects, aNTools, true).Dispose());
            N.TopoDS_Shape aNResult = NativeRun(aCase, aNObjects, aNTools, false);
            aNativeClean = aNResult.IsNull() ? 0 : Measure(() => NativeClean(aNResult));
#endif
            if (!aTotals.TryGetValue(aCase.Group, out Totals? aT))
            {
                aTotals[aCase.Group] = aT = new Totals();
            }
            aT.Cases++;
            aT.PortSerial += aPortSerial;
            aT.PortParallel += aPortParallel;
            aT.NativeSerial += aNativeSerial;
            aT.NativeParallel += aNativeParallel;
            aT.PortClean += aPortClean;
            aT.NativeClean += aNativeClean;
            aT.LogRatio += Math.Log(aPortSerial / aNativeSerial);
            aT.AllocatedMB += anAllocated;
            aT.CleanAllocatedMB += aCleanAllocated;
            aT.Worst.Add((aCase.Name, aPortSerial, aNativeSerial));
        }

        Console.WriteLine();
        Console.WriteLine("| group | cases | port serial | native serial | port/native serial | geometric mean | port parallel | native parallel "
                          + "| port/native parallel | port speed-up | native speed-up | port clean | native clean | port/native clean "
                          + "| port MB per Boolean | port MB per clean |");
        Console.WriteLine("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|");
        foreach ((string aGroup, Totals aT) in aTotals)
        {
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"| {aGroup} | {aT.Cases} | {aT.PortSerial:F1} | {aT.NativeSerial:F1} | {aT.PortSerial / aT.NativeSerial:F2}× | "
                + $"{Math.Exp(aT.LogRatio / aT.Cases):F2}× | {aT.PortParallel:F1} | {aT.NativeParallel:F1} | {aT.PortParallel / aT.NativeParallel:F2}× | "
                + $"{aT.PortSerial / aT.PortParallel:F2} | {aT.NativeSerial / aT.NativeParallel:F2} | {aT.PortClean:F1} | {aT.NativeClean:F1} | "
                + $"{aT.PortClean / Math.Max(aT.NativeClean, 1e-9):F2}× | {aT.AllocatedMB / aT.Cases:F2} | {aT.CleanAllocatedMB / aT.Cases:F2} |"));
        }
        Console.WriteLine();
        Console.WriteLine("The slowest port/native serial ratios (cases over 1 ms natively):");
        foreach ((string aGroup, Totals aT) in aTotals)
        {
            foreach ((string aName, double aPort, double aNative) in aT.Worst.Where(w => w.Native > 1).OrderByDescending(w => w.Port / w.Native).Take(5))
            {
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {aGroup} {aName}: port {aPort:F2} ms, native {aNative:F2} ms, {aPort / aNative:F2}×"));
            }
        }
    }

    /// <summary>The port's serial time split between the pave filler (the intersections) and the builder, against native's total.</summary>
    public static void Profile(string? theFilter)
    {
        Console.WriteLine("| group | cases | port pave filler | port builder | port total | native total |");
        Console.WriteLine("|---|---:|---:|---:|---:|---:|");
        var aTotals = new SortedDictionary<string, double[]>(StringComparer.Ordinal);
        foreach (BooleanCase aCase in All())
        {
            if ((theFilter is not null && !aCase.Group.Contains(theFilter, StringComparison.OrdinalIgnoreCase)
                 && !aCase.Name.Contains(theFilter, StringComparison.OrdinalIgnoreCase)) || aCase.Op is "Split")
            {
                continue;
            }
            (List<TopoDS_Shape> aPObjects, List<TopoDS_Shape> aPTools) = PortArgs(aCase);
            var anAll = new NCollection_List<TopoDS_Shape>();
            aPObjects.Concat(aPTools).ToList().ForEach(s => anAll.Append(s));
            var aFillers = new List<double>();
            var aBuilders = new List<double>();
            double aTotal = Measure(() =>
            {
                long t0 = Stopwatch.GetTimestamp();
                var aPF = new BOPAlgo_PaveFiller();
                aPF.SetArguments(anAll);
                aPF.SetNonDestructive(true);
                aPF.SetFuzzyValue(aCase.Fuzzy);
                aPF.SetGlue(aCase.Glue ? BOPAlgo_GlueEnum.BOPAlgo_GlueShift : BOPAlgo_GlueEnum.BOPAlgo_GlueOff);
                aPF.Perform();
                long t1 = Stopwatch.GetTimestamp();
                var aBOP = new BOPAlgo_BOP();
                aPObjects.ForEach(s => aBOP.AddArgument(s));
                aPTools.ForEach(s => aBOP.AddTool(s));
                aBOP.SetOperation(aCase.Op switch
                {
                    "Fuse" => BOPAlgo_Operation.BOPAlgo_FUSE,
                    "Cut" => BOPAlgo_Operation.BOPAlgo_CUT,
                    "Common" => BOPAlgo_Operation.BOPAlgo_COMMON,
                    _ => BOPAlgo_Operation.BOPAlgo_SECTION,
                });
                aBOP.PerformWithFiller(aPF);
                long t2 = Stopwatch.GetTimestamp();
                aFillers.Add(Ms(t1 - t0));
                aBuilders.Add(Ms(t2 - t1));
            });
            double aFiller = Median(aFillers), aBuilder = Median(aBuilders);
            double aNative = aTotal;
#if HAS_NATIVE
            (List<N.TopoDS_Shape> aNObjects, List<N.TopoDS_Shape> aNTools) = NativeArgs(aCase);
            aNative = Measure(() => NativeRun(aCase, aNObjects, aNTools, false).Dispose());
#endif
            if (!aTotals.TryGetValue(aCase.Group, out double[]? aT))
            {
                aTotals[aCase.Group] = aT = new double[5];
            }
            aT[0]++;
            aT[1] += aFiller;
            aT[2] += aBuilder;
            aT[3] += aTotal;
            aT[4] += aNative;
        }
        foreach ((string aGroup, double[] aT) in aTotals)
        {
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"| {aGroup} | {aT[0]} | {aT[1]:F1} | {aT[2]:F1} | {aT[3]:F1} | {aT[4]:F1} |"));
        }
    }
}
