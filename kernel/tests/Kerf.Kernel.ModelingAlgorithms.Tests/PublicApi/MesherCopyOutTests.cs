// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using Kerf.Meshing;
using Kerf.Topology;
using Xunit.Abstractions;
using EngineSphere = Kerf.Engine.Construction.Api.MakeSphere;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.PublicApi;

/// <summary>
/// What the public mesher costs beyond the engine's meshing (unit 10L3): the copy-out that unit 10H6 measured at 2.7 MB
/// for 2.4 MB of buffers, now through the mesher's overlay.
/// </summary>
public class MesherCopyOutTests(ITestOutputHelper output)
{
    [Fact]
    public void A_big_mesh_comes_out_with_little_more_than_its_buffers()
    {
        // A sphere meshed once and published: the mesher reuses its triangulations, so what is measured is the overlay,
        // the mesher's check that they are fine enough, and the copy-out
        Shape sphere = new EngineSphere(100).Shape();
        var options = new MeshOptions { LinearDeflection = 0.01, AngularDeflection = 0.2 };
        Shape meshed = Kerf.Engine.Meshing.IncrementalMesh.Meshed(sphere, options.ToEngine());
        ShapeMesh mesh = null!;
        long allocated = long.MaxValue;
        var best = TimeSpan.MaxValue;
        for (int run = 0; run < 5; run++)
        {
            long before = GC.GetAllocatedBytesForCurrentThread();
            var clock = Stopwatch.StartNew();
            mesh = meshed.Mesh(options with { Parallel = false }).Value;
            clock.Stop();
            allocated = Math.Min(allocated, GC.GetAllocatedBytesForCurrentThread() - before);
            best = clock.Elapsed < best ? clock.Elapsed : best;
        }

        long bufferBytes = 4L * (mesh.Positions.Length + mesh.Normals.Length + mesh.Triangles.Length);
        output.WriteLine($"{mesh.TriangleCount:N0} triangles: {allocated:N0} bytes allocated for {bufferBytes:N0} of buffers, best {best.TotalMilliseconds:F1} ms");
        Assert.True(mesh.TriangleCount > 100_000, $"{mesh.TriangleCount} triangles");
        Assert.True(allocated < 2 * bufferBytes, $"{allocated:N0} bytes allocated for {bufferBytes:N0} of buffers");
    }
}
