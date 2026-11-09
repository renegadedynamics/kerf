// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Runtime.Intrinsics;
using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Configs;
using BenchmarkDotNet.Jobs;
using BenchmarkDotNet.Running;
using BenchmarkDotNet.Toolchains.InProcess.Emit;
using Kerf.Drawing;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Kernel.Benchmarks;

/// <summary>
/// Unit 14E's micro-benchmarks: each loop given vector instructions against its scalar form, in one process, on one
/// processor (KERF_BENCH_CORE, 2 by default), under BenchmarkDotNet (the renderer's; Class2d's prototype, taken out, is
/// in the history at 7662090d3, with its tests):
///   dotnet run -c Release --project benchmarks/Kerf.Kernel.Benchmarks -- vectors [BenchmarkDotNet arguments, e.g. --filter *Render*]
/// </summary>
internal static class Vectors
{
    public static void Run(string[] args)
    {
        int core = int.TryParse(Environment.GetEnvironmentVariable("KERF_BENCH_CORE"), out int c) ? c : 2;
        if (OperatingSystem.IsWindows() || OperatingSystem.IsLinux())
        {
            Process.GetCurrentProcess().ProcessorAffinity = (IntPtr)(1L << core);
        }

        Console.WriteLine($"Core {core}; Vector256 {Vector256.IsHardwareAccelerated}, Vector512 {Vector512.IsHardwareAccelerated}");
        IConfig config = DefaultConfig.Instance.AddJob(Job.Default.WithToolchain(InProcessEmitToolchain.Instance))
                                               .WithOptions(ConfigOptions.DisableOptimizationsValidator);
        BenchmarkSwitcher.FromTypes([typeof(RenderBenchmarks)]).Run(args, config);
    }
}

/// <summary>
/// A picture rendered on one thread, with the rasteriser's loops over pixels by vectors or one at a time: a data/occ
/// shape, isometric, as the viewer's default (800 by 600, smooth) or as the blade bumper's publishing (1800 by 1100,
/// flat, feature edges at 25 degrees), supersampled twice.
/// </summary>
public class RenderBenchmarks
{
    private Scene _scene = null!;
    private RenderOptions _options = null!;

    [Params("CrankArm.brep", "Motor-c.brep")]
    public string Shape { get; set; } = "";

    [Params("viewer", "publish")]
    public string Options { get; set; } = "";

    [GlobalSetup]
    public void Setup()
    {
        Shape shape = Cases.PortRead(Path.Combine(Cases.DataOcc, Shape));
        ShapeMesh mesh = shape.Mesh(Renderer.DefaultMeshOptions(shape)).Value;
        _scene = new Scene(new SceneItem(mesh));
        _options = Options == "viewer"
            ? RenderOptions.Default with { Parallel = false }
            : new RenderOptions { Width = 1800, Height = 1100, Shading = Shading.Flat, Edges = EdgeDisplay.Feature, FeatureAngle = 25, Supersampling = 2, Parallel = false };
    }

    [Benchmark(Baseline = true)]
    public RgbaImage Scalar() => Renderer.Render(_scene, Camera.Isometric, _options, vectors: false, CancellationToken.None).Value;

    [Benchmark]
    public RgbaImage Vector() => Renderer.Render(_scene, Camera.Isometric, _options, vectors: true, CancellationToken.None).Value;
}
