// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Phase 4's benchmarks (phase4-plan 2.5). In Release:
//   dotnet run -c Release --project benchmarks/OcctNet.Benchmarks -- quick [filter]
//     a Stopwatch table: per shape, serial and parallel, the median of the port's mesh and extraction against native
//     OCCT's (through build123d.NET), and the port's allocations;
//   dotnet run -c Release --project benchmarks/OcctNet.Benchmarks -- bdn [BenchmarkDotNet arguments]
//     the same mesh-and-extract under BenchmarkDotNet (in process, with its memory diagnoser);
//   ... -- alloc [filter]: the types the port allocates;
//   ... -- booleans [filter]: the Booleans suite (unit 6M, phase5-6-plan 2.9): BRepAlgoAPI and build123d.NET's clean, the
//     port against native, serial and parallel, with allocation, over build123d.NET's tests' Booleans and the corpus's
//     simple and complex grids (Booleans.cs); ... -- booleans-profile [filter]: the port's pave filler and builder apart;
//   ... -- wasm-compare report.json data-folder: the WebAssembly demo's report against win-x64 (run-headless.ps1).
// Pass the same -p:OcctSourceDir and -p:Build123dNetRoot as for the tests.

using System.Diagnostics;
using System.Globalization;
using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Configs;
using BenchmarkDotNet.Jobs;
using BenchmarkDotNet.Running;
using BenchmarkDotNet.Toolchains.InProcess.Emit;
using Occt;
using OcctNet.Benchmarks;
#if HAS_NATIVE
using N = Build123d.Occt;
#endif

if (args.Length > 0 && args[0] == "bdn")
{
    IConfig aConfig = DefaultConfig.Instance.AddJob(Job.ShortRun.WithToolchain(InProcessEmitToolchain.Instance));
    BenchmarkSwitcher.FromAssembly(typeof(MeshBenchmarks).Assembly).Run(args[1..], aConfig);
    return 0;
}

string? aFilter = args.Length > 1 ? args[1] : null;
if (args.Length > 2 && args[0] == "wasm-compare")
{
    WasmCompare.Run(args[1], args[2]);
    return 0;
}
if (args.Length > 1 && args[0] == "demo")
{
    // The WebAssembly demo's work on win-x64, for one file: its timings, counts and memory
    foreach (string aFile in args[1..])
    {
        OcctNet.WasmDemo.DemoResult aRun = OcctNet.WasmDemo.DemoCore.Run(File.ReadAllBytes(aFile));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"{Path.GetFileName(aFile)}: read {aRun.ReadMs:F1} ms, mesh {aRun.MeshMs:F1} ms, extract {aRun.ExtractMs:F1} ms; {aRun.Faces} faces, "
            + $"{aRun.Nodes} nodes, {aRun.Triangles} triangles; GC heap peak {GC.GetGCMemoryInfo().HeapSizeBytes / 1048576.0:F0} MB, "
            + $"working set peak {Environment.WorkingSet / 1048576.0:F0} MB, allocated {GC.GetTotalAllocatedBytes() / 1048576.0:F0} MB"));
#if HAS_NATIVE
        N.TopoDS_Shape aNative = Cases.NativeRead(aFile);
        N.BRepTools.Clean(aNative);
        var aClock = Stopwatch.StartNew();
        new N.BRepMesh_IncrementalMesh(aNative, aRun.Deflection, false, OcctNet.WasmDemo.DemoCore.Angle, true).Dispose();
        double aMeshMs = aClock.Elapsed.TotalMilliseconds;
        N.BRepTools.Clean(aNative);
        aClock.Restart();
        new N.BRepMesh_IncrementalMesh(aNative, aRun.Deflection, false, OcctNet.WasmDemo.DemoCore.Angle, false).Dispose();
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  native mesh {aMeshMs:F1} ms parallel, {aClock.Elapsed.TotalMilliseconds:F1} ms serial"));
        aNative.Dispose();
#endif
    }
    return 0;
}
if (args.Length > 0 && args[0] == "booleans")
{
    Booleans.Run(aFilter);
    return 0;
}
if (args.Length > 0 && args[0] == "booleans-profile")
{
    Booleans.Profile(aFilter);
    return 0;
}
if (args.Length > 0 && args[0] == "alloc")
{
    Allocations.Run(aFilter);
    return 0;
}
Quick.Run(aFilter);
return 0;

namespace OcctNet.Benchmarks
{
    /// <summary>The Stopwatch table.</summary>
    internal static class Quick
    {
        private static double Median(List<double> theTimes)
        {
            theTimes.Sort();
            int n = theTimes.Count;
            return n % 2 == 1 ? theTimes[n / 2] : 0.5 * (theTimes[n / 2 - 1] + theTimes[n / 2]);
        }

        /// <summary>Times <paramref name="theRun"/> (which returns its parts' milliseconds) after a warm-up: at least 3 runs, up to 2 s or 30 runs.</summary>
        private static double[] Measure(Func<double[]> theRun)
        {
            theRun();
            var aParts = new List<List<double>>();
            var aClock = Stopwatch.StartNew();
            for (int i = 0; i < 30 && (i < 3 || aClock.Elapsed.TotalSeconds < 2); i++)
            {
                double[] aTimes = theRun();
                while (aParts.Count < aTimes.Length)
                {
                    aParts.Add([]);
                }
                for (int k = 0; k < aTimes.Length; k++)
                {
                    aParts[k].Add(aTimes[k]);
                }
            }
            return aParts.Select(Median).ToArray();
        }

        private static double Ms(long theTicks) => theTicks * 1000.0 / Stopwatch.Frequency;

        public static void Run(string? theFilter)
        {
            Console.WriteLine($".NET {Environment.Version}, {Environment.ProcessorCount} logical processors, {(Debugger.IsAttached ? "debugger" : "no debugger")}");
            Console.WriteLine("Medians in ms; mesh = BRepMesh_IncrementalMesh (the port's includes copying the frozen input), "
                              + "extract = the viewer's buffers. Native: build123d.NET's binding of OCCT 8.0.1 (MSVC).");
            Console.WriteLine();
            Console.WriteLine("| shape | faces | mode | port mesh | port extract | port total | native mesh | native extract | native total | port/native | port MB allocated |");
            Console.WriteLine("|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|");
            foreach (Case aCase in Cases.All())
            {
                if (theFilter is not null && !aCase.Name.Contains(theFilter, StringComparison.OrdinalIgnoreCase))
                {
                    continue;
                }
                TopoDS_Shape aPortInput = Cases.PortRead(aCase.F1);
#if HAS_NATIVE
                N.TopoDS_Shape aNativeInput = Cases.NativeRead(aCase.F1);
#endif
                foreach (bool isParallel in new[] { false, true })
                {
                    long anAllocated = 0;
                    double[] aPort = Measure(() =>
                    {
                        long aBytes = GC.GetTotalAllocatedBytes(true);
                        long t0 = Stopwatch.GetTimestamp();
                        TopoDS_Shape aMeshed = new BRepMesh_IncrementalMesh(aPortInput, aCase.Deflection, false, Cases.Angle, isParallel).Shape();
                        long t1 = Stopwatch.GetTimestamp();
                        MeshExtraction.Extract(aMeshed, aCase.Deflection, Cases.Angle);
                        long t2 = Stopwatch.GetTimestamp();
                        anAllocated = GC.GetTotalAllocatedBytes(true) - aBytes;
                        return [Ms(t1 - t0), Ms(t2 - t1), Ms(t2 - t0)];
                    });
                    string aNativeColumns = "| | | | ";
                    string aRatio = "";
#if HAS_NATIVE
                    double[] aNative = Measure(() =>
                    {
                        N.BRepTools.Clean(aNativeInput);
                        long t0 = Stopwatch.GetTimestamp();
                        new N.BRepMesh_IncrementalMesh(aNativeInput, aCase.Deflection, false, Cases.Angle, isParallel).Dispose();
                        long t1 = Stopwatch.GetTimestamp();
                        Cases.NativeExtract(aNativeInput, aCase.Deflection);
                        long t2 = Stopwatch.GetTimestamp();
                        return [Ms(t1 - t0), Ms(t2 - t1), Ms(t2 - t0)];
                    });
                    aNativeColumns = string.Create(CultureInfo.InvariantCulture, $"| {aNative[0]:F2} | {aNative[1]:F2} | {aNative[2]:F2} ");
                    aRatio = string.Create(CultureInfo.InvariantCulture, $"{aPort[2] / aNative[2]:F2}×");
#endif
                    Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                        $"| {aCase.Name} | {aCase.Faces} | {(isParallel ? "parallel" : "serial")} | {aPort[0]:F2} | {aPort[1]:F2} | {aPort[2]:F2} "
                        + $"{aNativeColumns}| {aRatio} | {anAllocated / 1048576.0:F1} |"));
                }
#if HAS_NATIVE
                aNativeInput.Dispose();
#endif
            }
        }
    }

    /// <summary>Mesh and extract, the port against native, under BenchmarkDotNet.</summary>
    [MemoryDiagnoser]
    public class MeshBenchmarks
    {
        private Case myCase = null!;
        private TopoDS_Shape myPortInput;
#if HAS_NATIVE
        private N.TopoDS_Shape myNativeInput = null!;
#endif

        public static IEnumerable<string> Names() => ["Pump_Nut.brep", "bottle.brep", "CrankArm.brep", "filleted block with holes", "sphere"];

        [ParamsSource(nameof(Names))]
        public string Shape { get; set; } = "";

        [Params(false, true)]
        public bool Parallel { get; set; }

        [GlobalSetup]
        public void Setup()
        {
            myCase = Cases.All().Single(c => c.Name == Shape);
            myPortInput = Cases.PortRead(myCase.F1);
#if HAS_NATIVE
            myNativeInput = Cases.NativeRead(myCase.F1);
#endif
        }

        [Benchmark(Baseline = true)]
        public object Port() => Cases.PortMeshAndExtract(myPortInput, myCase.Deflection, Parallel);

#if HAS_NATIVE
        /// <summary>build123d.NET's ShapeMesh path: copy, mesh, extract.</summary>
        [Benchmark]
        public object Native() => Cases.NativeMeshAndExtract(myNativeInput, myCase.Deflection, Parallel);
#endif
    }
}
