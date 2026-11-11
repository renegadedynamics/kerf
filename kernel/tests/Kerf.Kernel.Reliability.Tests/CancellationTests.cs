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

    // The fillet reports no progress, so the cancel comes on a timer: after a quarter of a first run's time, and if the run finished
    // first (a first run slowed by a loaded machine, or a second one quicker), again after half that, down to a few milliseconds,
    // each finished run the same as the first. Only a cancel that fell while the fillet ran ends it (decision 2206)
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void A_fillet_stops_part_way_and_then_gives_the_same_result(bool byScope)
    {
        Shape plate = Plate(12);
        Edge[] edges = [.. plate.AllEdges()];
        var clock = Stopwatch.StartNew();
        double whole = plate.Fillet(edges, 0.1).Value.Shape.IntegrateVolume();
        TimeSpan delay = clock.Elapsed / 4;

        bool stopped = false;
        for (int attempt = 0; !stopped && attempt < 12; attempt++, delay /= 2)
        {
            using var source = new CancellationTokenSource(delay);
            try
            {
                Result<Built<Shape>> done;
                if (byScope)
                {
                    using (Cancellation.Use(source.Token))
                    {
                        done = plate.Fillet(edges, 0.1);
                    }
                }
                else
                {
                    done = plate.Fillet(edges, 0.1, cancellationToken: source.Token);
                }
                Assert.Equal(whole, done.Value.Shape.IntegrateVolume());
            }
            catch (OperationCanceledException)
            {
                stopped = true;
            }
        }

        Assert.True(stopped, "no cancel fell while the fillet ran");
        Assert.Equal(whole, plate.Fillet(edges, 0.1).Value.Shape.IntegrateVolume());
    }

    [Fact]
    public void A_fillet_of_thousands_of_edges_stops_soon_after_the_cancel_in_its_checks_before_the_algorithm()
    {
        // Bento's case: every edge of a plate of 900 holes, cancelled while the limits are read. 0.4.0 spent 2 s there on a quiet
        // machine before its first check, asking of each hole's far rim whether it was tangent to the plate's top, of 904 edges, and
        // returned 1.8 to 2.4 s after the cancel; now within 100 ms, and 1 s is the bound so that a loaded machine passes. The holes
        // are as deep as the plate, as Bento's: on the plate above, its holes past both faces, 0.4.0 read the rims in 0.3 s
        Shape block = Solid.Box(200, 200, 10).Value.Shape;
        var holes = new List<Shape>();
        for (int i = 0; i < 30; i++)
        {
            for (int j = 0; j < 30; j++)
            {
                holes.Add(Solid.Cylinder(1, 10, new Frame3D(new Point3D(27.5 + i * 5, 27.5 + j * 5, 0), Direction3D.UnitZ)).Value.Shape);
            }
        }
        Shape plate = BooleanOperations.Cut([block], holes).Value.Shape;
        Edge[] edges = [.. plate.AllEdges()];
        using var source = new CancellationTokenSource();
        long cancelled = 0;
        var clock = Stopwatch.StartNew();
        using var timer = new Timer(_ =>
        {
            Interlocked.Exchange(ref cancelled, clock.ElapsedTicks);
            source.Cancel();
        }, null, 200, Timeout.Infinite);
        Assert.ThrowsAny<OperationCanceledException>(() => plate.Fillet(edges, 0.2, cancellationToken: source.Token));
        TimeSpan late = TimeSpan.FromTicks((long)((clock.ElapsedTicks - Interlocked.Read(ref cancelled)) * (double)TimeSpan.TicksPerSecond / Stopwatch.Frequency));
        Assert.True(late < TimeSpan.FromSeconds(1), $"returned {late.TotalMilliseconds:F0} ms after the cancel");
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
