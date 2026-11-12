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
    public void A_partition_whose_crossing_stops_short_is_made_again_and_is_right()
    {
        // OCCT #905: a turned disc and two sheets that leave its axis and pass out through its rim, which cut a sector from
        // it. One crossing of a sheet and the disc ends 3.1e-5 short of the next, with vertices of 5e-6, so neither sheet
        // is split and the disc stays one solid: a success, before decision 2300
        IReadOnlyList<Shape> parts = [.. Fixtures.Named("sheets-through-disc").Children];
        Assert.Equal(3, parts.Count);

        Result<Built<Shape>> partition = BooleanOperations.Partition(parts);
        Assert.True(partition.IsSuccess);
        Assert.Contains(partition.Diagnostics, d => d.Code == BooleanOperations.BuiltWithFuzzyValue);
        var solids = partition.Value.Shape.AllSolids();
        Assert.Equal(2, solids.Length);
        Assert.True(Kerf.Healing.HealingOperations.Validate(partition.Value.Shape).IsValid);
        double[] volumes = [.. solids.Select(s => Kerf.Measurement.MeasureOperations.IntegrateVolume(s)).Order()];
        Assert.Equal(319.32425, volumes[0], 1e-4);
        Assert.Equal(3173.54846, volumes[1], 1e-4);
        Assert.Equal(Kerf.Measurement.MeasureOperations.IntegrateVolume(parts[2]), volumes.Sum(), 1e-4);

        // unchecked, it's as OCCT has it
        Assert.Single(BooleanOperations.Partition(parts, new BooleanOptions { CheckResult = false }).Value.Shape.AllSolids());
    }

    [Theory]
    [InlineData(7.539994956368779, -67.98510139717416)]
    [InlineData(31.408469470395012, -29.006276572539274)]
    [InlineData(31.261914556282857, -48.49488543234123)]
    [InlineData(-11.749784258838382, 25.638988751484614)]
    [InlineData(27.747314915394483, -17.810983395051622)]
    public void A_section_with_a_piece_missing_is_not_a_success(double x, double y)
    {
        // OCCT #1124: a disc of two cones and a torus, and a cylinder of radius 6 through it at each of the issue's five
        // places. Approximated, each section is the whole curve round the cylinder. Not approximated, or without its curves
        // on the faces, some keep a fraction of it, each a success before decision 2301
        Shape disc = Fixtures.Named("section-disc");
        Shape cylinder = Primitives.Cylinder(6, 100, new Frame3D(new Point3D(x, y, -50), Direction3D.UnitZ)).Value.Shape;
        double whole = Kerf.Measurement.MeasureOperations.IntegrateLength(disc.Section(cylinder).Value.Shape);
        Assert.InRange(whole, 38, 43);

        int failures = 0;
        foreach (BooleanOptions options in new BooleanOptions[]
                 {
                     new() { ApproximateSection = false },
                     new() { ApproximateSection = false, SectionCurvesOnFaces = false },
                     new() { SectionCurvesOnFaces = false },
                 })
        {
            Result<Built<Shape>> section = disc.Section(cylinder, options);
            if (section.IsSuccess)
            {
                Assert.Equal(whole, Kerf.Measurement.MeasureOperations.IntegrateLength(section.Value.Shape), 2e-3);
            }
            else
            {
                failures++;
                Assert.Contains(section.Diagnostics, d => d.Code == DiagnosticCode.InvalidResult);
                Assert.True(section.TryGetInvalid(out _));
            }
        }

        // each place loses a piece one way or another: the fifth has 0.004 of its 40.53 not approximated, the others
        // 0.96 to 16.8 without their curves on the faces
        Assert.True(failures > 0);
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
