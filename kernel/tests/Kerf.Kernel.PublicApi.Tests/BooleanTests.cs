// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>The Boolean operations (docs/api/booleans.md; unit 10L2).</summary>
public class BooleanTests
{
    private static Solid Box(double x, double size = 2) =>
        Solid.Box(size, size, size, new Frame3D(new Point3D(x, 0, 0), Direction3D.UnitZ)).Value;

    [Fact]
    public void A_fuse_is_simplified_by_default()
    {
        Shape fused = Box(0).Fuse(Box(1)).Value.Shape;
        Assert.Single(fused.AllSolids());
        Assert.Equal(6, fused.AllFaces().Length);
        Shape raw = BooleanOperations.Fuse(Box(0), Box(1), new BooleanOptions { Simplify = false }).Value.Shape;
        Assert.True(raw.AllFaces().Length > 6);
        Assert.Single(BooleanOperations.FuseAll([Box(0), Box(1), Box(2)]).Value.Shape.AllSolids());
    }

    [Fact]
    public void A_cut_carries_the_tools_faces_in_its_history()
    {
        Solid block = Box(0, 10);
        Solid hole = Solid.Cylinder(2, 20, new Frame3D(new Point3D(5, 5, -5), Direction3D.UnitZ)).Value;
        Built<Shape> cut = block.Cut(hole).Value;
        Assert.Equal(7, cut.Shape.AllFaces().Length);
        Face wall = hole.AllFaces().First(f => f.TryGetSurface(out Surface? s) && s is CylindricalSurface);
        Assert.NotEmpty(cut.History.Modified(wall));
        Assert.True(cut.History.IsRemoved(hole.AllFaces().First(f => !f.Equals(wall))));
    }

    [Fact]
    public void A_common_of_apart_shapes_is_empty_with_a_warning()
    {
        var common = BooleanOperations.Common(Box(0), Box(5));
        Assert.True(common.IsSuccess);
        Assert.Contains(common.Diagnostics, d => d.Code == BooleanOperations.EmptyResult && !d.IsError);
        Assert.Single(Box(0).Common(Box(1)).Value.Shape.AllSolids());
    }

    [Fact]
    public void A_fuse_of_a_solid_with_an_edge_fails_with_the_engines_alert()
    {
        Edge edge = Edge.Line(new(0, 0, 5), new(1, 0, 5)).Value;
        var fused = Box(0).Fuse(edge);
        Assert.False(fused.IsSuccess);
        Assert.StartsWith("boolean.", fused.Error!.Code.Value, StringComparison.Ordinal);
    }

    [Fact]
    public void Section_split_and_partition()
    {
        Face plane = Face.FromPlane(new Plane3D(new Point3D(0, 0, 1), Direction3D.UnitZ)).Value;
        Shape section = Box(0).Section(plane).Value.Shape;
        Assert.Equal(4, section.AllEdges().Length);
        Face cutter = Face.FromWire(Wire.Polygon([new(-1, -1, 1), new(3, -1, 1), new(3, 3, 1), new(-1, 3, 1)], closed: true).Value).Value;
        Assert.Equal(2, BooleanOperations.Split([Box(0)], [cutter]).Value.Shape.AllSolids().Length);
        Assert.Equal(3, BooleanOperations.Partition([Box(0), Box(1)], new BooleanOptions { Simplify = false }).Value.Shape.AllSolids().Length);
        KernelAssert.Throws<ArgumentException>(() => BooleanOperations.Partition([]));
    }

    [Fact]
    public void A_boolean_is_cancelled_by_its_token()
    {
        using var source = new CancellationTokenSource();
        source.Cancel();
        Assert.Throws<OperationCanceledException>(() => Box(0).Fuse(Box(1), cancellationToken: source.Token));
        var fractions = new List<double>();
        Assert.True(Box(0).Cut(Box(1), progress: new Progress(fractions)).IsSuccess);
    }

    private sealed class Progress(List<double> fractions) : IProgress<double>
    {
        public void Report(double value) => fractions.Add(value);
    }
}
