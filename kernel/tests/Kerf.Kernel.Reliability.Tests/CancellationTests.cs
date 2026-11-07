// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Kernel.Reliability.Tests;

/// <summary>Stopping what's running: <see cref="Cancellation"/> and the checks in long loops (decision 1855).</summary>
public class CancellationTests
{
    private static Shape Box(double x = 0) => Solid.Box(2, 2, 2, new Frame3D(new Point3D(x, 0, 0), Direction3D.UnitZ)).Value.Shape;

    // A plate with many holes: a fillet on every edge takes a good while, in many pieces
    private static Shape Plate(int n)
    {
        Shape block = Solid.Box(n * 4, n * 4, 4).Value.Shape;
        var holes = new List<Shape>();
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                holes.Add(Solid.Cylinder(1, 8, new Frame3D(new Point3D(i * 4 + 2, j * 4 + 2, -2), Direction3D.UnitZ)).Value.Shape);
            }
        }

        return BooleanOperations.Cut([block], holes).Value.Shape;
    }

    private static CancellationToken Cancelled()
    {
        using var source = new CancellationTokenSource();
        CancellationToken token = source.Token;
        source.Cancel();
        return token;
    }

    [Fact]
    public void An_operation_without_a_token_obeys_the_one_in_force()
    {
        Shape a = Box(), b = Box(1);
        using (Cancellation.Use(Cancelled()))
        {
            Assert.ThrowsAny<OperationCanceledException>(() => a.Fuse(b));
            Assert.ThrowsAny<OperationCanceledException>(() => a.Fillet(a.AllEdges(), 0.1));
            Assert.ThrowsAny<OperationCanceledException>(() => a.Mesh());
            Assert.ThrowsAny<OperationCanceledException>(() => Box());
        }

        Assert.True(Box().Fuse(Box(1)).IsSuccess);
    }

    [Fact]
    public void An_operation_given_a_token_obeys_that_one()
    {
        Shape a = Box(), b = Box(1);
        using (Cancellation.Use(Cancelled()))
        {
            using var live = new CancellationTokenSource();
            Assert.True(a.Fuse(b, cancellationToken: live.Token).IsSuccess);
        }

        Assert.ThrowsAny<OperationCanceledException>(() => Box().Fuse(Box(1), cancellationToken: Cancelled()));
    }

    [Fact]
    public void The_token_in_force_flows_into_parallel_loops_and_comes_off_at_the_end()
    {
        using var source = new CancellationTokenSource();
        Assert.False(Cancellation.Current.CanBeCanceled);
        using (Cancellation.Use(source.Token))
        {
            CancellationToken seen = default;
            Parallel.For(0, 64, _ => seen = Cancellation.Current);
            Assert.Equal(source.Token, seen);
            using (Cancellation.Use(default))
            {
                Assert.False(Cancellation.Current.CanBeCanceled);
            }

            Assert.Equal(source.Token, Cancellation.Current);
        }

        Assert.False(Cancellation.Current.CanBeCanceled);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void A_fillet_stops_part_way_and_then_gives_the_same_result(bool byScope)
    {
        Shape plate = Plate(12);
        Edge[] edges = [.. plate.AllEdges()];
        var clock = Stopwatch.StartNew();
        double whole = plate.Fillet(edges, 0.1).Value.Shape.IntegrateVolume();
        TimeSpan took = clock.Elapsed;

        using var source = new CancellationTokenSource();
        source.CancelAfter(took / 4);
        Assert.ThrowsAny<OperationCanceledException>(() =>
        {
            if (byScope)
            {
                using (Cancellation.Use(source.Token))
                {
                    plate.Fillet(edges, 0.1);
                }
            }
            else
            {
                plate.Fillet(edges, 0.1, cancellationToken: source.Token);
            }
        });
        Assert.Equal(whole, plate.Fillet(edges, 0.1).Value.Shape.IntegrateVolume());
    }

    [Fact]
    public void A_mesh_too_large_to_be_meant_fails_at_once()
    {
        Assert.Equal("meshing.too-large", ChildProcess.Run("mesh-too-fine", 60));
        Assert.Equal("ok", ChildProcess.Run("mesh-sensible"));
    }

    [Fact]
    public void The_limit_on_triangles_is_an_option()
    {
        Shape sphere = Solid.Sphere(10).Value.Shape;
        ShapeMesh mesh = sphere.Mesh(new MeshOptions { LinearDeflection = 0.01 }).Value;
        int triangles = mesh.Faces.Sum(f => f.Triangles.Length);
        Result<ShapeMesh> refused = sphere.Mesh(new MeshOptions { LinearDeflection = 0.01, MaxTriangles = triangles / 4 });
        Assert.Equal(Mesher.TooLarge, refused.Error!.Code);
        ShapeMesh allowed = sphere.Mesh(new MeshOptions { LinearDeflection = 0.01, MaxTriangles = triangles * 4L }).Value;
        Assert.Equal(triangles, allowed.Faces.Sum(f => f.Triangles.Length));
    }
}
