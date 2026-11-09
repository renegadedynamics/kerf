// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using Kerf.Construction;
using Kerf.Exchange;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>A number damaged in a file: a failure, or something that can be stopped, and never a hang (decisions 1914 and 1915).</summary>
public class DamagedFileTests
{
    private static string BrepOf(Shape shape)
    {
        var text = new MemoryStream();
        Assert.True(Brep.Write(shape, text).IsSuccess);
        return Encoding.ASCII.GetString(text.ToArray());
    }

    private static Result<Shape> Read(string brep) => Brep.Read(new MemoryStream(Encoding.ASCII.GetBytes(brep)));

    [Theory]
    [InlineData("nan")]
    [InlineData("inf")]
    [InlineData("-Infinity")]
    public void A_brep_with_a_number_that_is_not_finite_is_malformed(string number)
    {
        string brep = BrepOf(Solid.Box(1, 1, 1).Value.Shape);
        int at = brep.IndexOf('\n', brep.IndexOf("\nVe", StringComparison.Ordinal) + 1) + 1; // the vertex's tolerance
        string damaged = brep[..at] + number + brep[brep.IndexOf('\n', at)..];

        Result<Shape> read = Read(damaged);

        Assert.False(read.IsSuccess);
        Assert.Equal("exchange.malformed", read.Error!.Code.Value);
    }

    // The six files of the damaged-file sweep (decision 1917) whose mesh ran out of a 1.5 GB heap: a cone's seam damaged
    // on its curve or a pcurve, and a loft's surface, each to 1e17 or 2^31 (decision 1942)
    [Theory]
    [InlineData("1430-cone.brep")]
    [InlineData("5462-cone.brep")]
    [InlineData("20630-cone.brep")]
    [InlineData("20030-loft.brep")]
    [InlineData("20958-loft.brep")]
    [InlineData("5086-loft.brep")]
    public void A_mesh_of_a_face_damaged_out_of_proportion_fails_carrying_the_other_faces(string file)
    {
        using Stream resource = typeof(DamagedFileTests).Assembly.GetManifestResourceStream("DamagedFiles." + file)!;
        Result<Shape> read = Brep.Read(resource);
        Assert.True(read.IsSuccess);
        long before = GC.GetAllocatedBytesForCurrentThread();

        Result<ShapeMesh> mesh = read.Value.Mesh(new MeshOptions { Parallel = false });

        // Before, 1 GB and more, then out of memory; now under 600 MB, most of it the circles in cells up to their floor
        Assert.InRange(GC.GetAllocatedBytesForCurrentThread() - before, 0, 800_000_000);
        Assert.False(mesh.IsSuccess);
        Assert.Equal(Mesher.FacesRefused, mesh.Error!.Code);
        Assert.True(mesh.TryGetInvalid(out ShapeMesh? rest));
        Assert.True(rest.TriangleCount > 0);
    }

    // Lofts whose surface has a pole damaged to 1e300 beside an edge that isn't (decision 2060): the edges between that
    // face and the next were split 10 levels down on every segment, to 44,000 points, as the surface never came near
    // them. The damaged face walked its frontier for 25 s before it was refused, and the next face meshed those points
    // for 270 s, 1 GB allocated (a loaded machine); now the edges find the surface out of proportion to them, and the
    // face is refused before it's meshed: under a second, 23 MB
    [Theory]
    [InlineData("846-loft.brep")]
    [InlineData("1822-loft.brep")]
    [InlineData("78-loft.brep")]
    public void A_mesh_of_a_loft_whose_surface_is_damaged_to_1e300_beside_its_edges_is_refused_at_once(string file)
    {
        using Stream resource = typeof(DamagedFileTests).Assembly.GetManifestResourceStream("DamagedFiles." + file)!;
        Result<Shape> read = Brep.Read(resource);
        Assert.True(read.IsSuccess);
        long before = GC.GetAllocatedBytesForCurrentThread();

        Result<ShapeMesh> mesh = read.Value.Mesh(new MeshOptions { Parallel = false });

        Assert.InRange(GC.GetAllocatedBytesForCurrentThread() - before, 0, 100_000_000);
        Assert.False(mesh.IsSuccess);
        Assert.Equal(Mesher.FacesRefused, mesh.Error!.Code);
        Assert.True(mesh.TryGetInvalid(out ShapeMesh? rest));
        Assert.True(rest.TriangleCount > 0);
    }

    // A loft damaged to 1e300: its frontier's walks went on for hours and couldn't be cancelled (decision 1944)
    [Fact]
    public async Task A_mesh_of_a_loft_damaged_to_1e300_can_be_stopped()
    {
        using Stream resource = typeof(DamagedFileTests).Assembly.GetManifestResourceStream("DamagedFiles.78-loft.brep")!;
        Result<Shape> read = Brep.Read(resource);
        Assert.True(read.IsSuccess);
        using var source = new CancellationTokenSource(TimeSpan.FromSeconds(2));
        Task task = Task.Run(() =>
        {
            try
            {
                _ = read.Value.Mesh(new MeshOptions { Parallel = false }, cancellationToken: source.Token);
            }
            catch (OperationCanceledException)
            {
            }
        });
        await task.WaitAsync(TimeSpan.FromSeconds(60));
    }

    [Fact]
    public async Task A_mesh_of_a_range_damaged_to_1e300_can_be_stopped()
    {
        string brep = BrepOf(Solid.Cylinder(5, 10).Value.Shape);
        string damaged = brep.Replace("6.28318530717959", "1e300", StringComparison.Ordinal);
        Assert.NotEqual(brep, damaged);
        Result<Shape> read = Read(damaged);
        if (!read.IsSuccess)
        {
            return;
        }

        using var source = new CancellationTokenSource(TimeSpan.FromSeconds(2));
        Task task = Task.Run(() =>
        {
            try
            {
                _ = read.Value.Mesh(new MeshOptions { Parallel = false }, cancellationToken: source.Token);
            }
            catch (OperationCanceledException)
            {
            }
        });
        await task.WaitAsync(TimeSpan.FromSeconds(60));
    }
}
