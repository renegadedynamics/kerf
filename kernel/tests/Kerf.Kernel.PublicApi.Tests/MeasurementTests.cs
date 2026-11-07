// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Features;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Kerf.Measurement (docs/api/measurement.md): mass properties, boxes, distances, classification, line hits.</summary>
public class MeasurementTests
{
    private const double Tol = 1e-6;

    [Fact]
    public void A_box_has_its_volume_area_and_length()
    {
        Solid box = Fixtures.Box();
        Assert.Equal(6000, box.IntegrateVolume(), Tol);
        Assert.Equal(2200, box.IntegrateArea(), Tol);
        // Each edge once per face it bounds, as OCCT counts; once with SkipShared
        Assert.Equal(480, box.IntegrateLength(), Tol);
        Assert.Equal(240, box.IntegrateLength(new MassOptions { SkipShared = true }), Tol);
        Assert.Equal(6000, MeasureOperations.IntegrateVolume(box), Tol);
        Assert.Equal(Math.PI * 25 * 10, Fixtures.Cylinder().IntegrateVolume(), 1e-6);
    }

    [Fact]
    public void Mass_properties_have_the_centre_inertia_and_moments()
    {
        MassProperties props = Fixtures.Box().MassProperties(Dimension.Volume);
        Assert.Equal(Dimension.Volume, props.Dimension);
        Assert.Equal(6000, props.Mass, Tol);
        Assert.True(props.CentreOfMass.IsEqual(new Point3D(5, 10, 15), Tol));
        Assert.True(props.StaticMoments.IsEqual(new Vector3D(30000, 60000, 90000), Tol, Tol));
        // About the vertical axis through the centre: m (a² + b²) / 12
        var vertical = new Axis3D(new Point3D(5, 10, 15), Direction3D.UnitZ);
        Assert.Equal(6000 * (100 + 400) / 12.0, props.MomentOfInertia(vertical), 1e-3);
        Assert.Equal(Math.Sqrt((100 + 400) / 12.0), props.RadiusOfGyration(vertical), 1e-9);
        // Moved off the centre by 5 along X: plus m d²
        var moved = new Axis3D(new Point3D(10, 10, 0), Direction3D.UnitZ);
        Assert.Equal(6000 * (100 + 400) / 12.0 + 6000 * 25, props.MomentOfInertia(moved), 1e-3);
        double sum = props.PrincipalMoments.I1 + props.PrincipalMoments.I2 + props.PrincipalMoments.I3;
        Assert.Equal(6000 * (2 * (100 + 400 + 900)) / 12.0, sum, 1e-3);
        Assert.Equal(0, props.PrincipalAxes.A1.Dot(props.PrincipalAxes.A2), 1e-9);
    }

    [Fact]
    public void Area_and_length_properties_and_the_options()
    {
        Solid box = Fixtures.Box();
        Assert.Equal(2200, box.MassProperties(Dimension.Area).Mass, Tol);
        Assert.Equal(480, box.MassProperties(Dimension.Length).Mass, Tol);
        Assert.Equal(6000, box.IntegrateVolume(new MassOptions { Tolerance = 1e-9 }), 1e-4);
        Compound twice = Compound.Create([box, box]);
        Assert.Equal(12000, twice.IntegrateVolume(), 1e-6);
        Assert.Equal(6000, twice.IntegrateVolume(new MassOptions { SkipShared = true }), 1e-6);
        Assert.Throws<ArgumentOutOfRangeException>(() => new MassOptions { Tolerance = -1 });
        Assert.Throws<ArgumentOutOfRangeException>(() => box.MassProperties((Dimension)7));
    }

    [Fact]
    public void Bounding_boxes_are_loose_or_tight()
    {
        Solid box = Fixtures.Box();
        BoundingBox loose = box.BoundingBox();
        BoundingBox tight = box.BoundingBox(optimal: true);
        Assert.True(loose.Contains(new Point3D(0, 0, 0)) && loose.Contains(new Point3D(10, 20, 30)));
        Assert.True(tight.Min.IsEqual(new Point3D(0, 0, 0), 1e-6) && tight.Max.IsEqual(new Point3D(10, 20, 30), 1e-6));
        Assert.True(loose.Size.X >= tight.Size.X);
    }

    [Fact]
    public void An_oriented_box_fits_the_shape()
    {
        OrientedBox box = Fixtures.Box().OrientedBox(optimal: true);
        Assert.False(box.IsEmpty);
        Assert.Equal(6000, box.Volume, 1e-3);
        Assert.True(box.Contains(new Point3D(5, 10, 15)));
        Assert.False(box.Contains(new Point3D(50, 10, 15)));
        Assert.Equal(8, box.Corners().Length);
        Assert.True(box.ToBoundingBox().Max.IsEqual(new Point3D(10, 20, 30), 1e-3));
        Assert.True(box.Intersects(OrientedBox.FromBoundingBox(new BoundingBox(new Point3D(9, 0, 0), new Point3D(12, 1, 1)))));
        Assert.True(default(OrientedBox).IsEmpty);
        Assert.Throws<InvalidOperationException>(() => default(OrientedBox).Centre);
        Assert.Throws<ArgumentException>(() => new OrientedBox(Point3D.Origin, Direction3D.UnitX, Direction3D.UnitX, Direction3D.UnitZ, Vector3D.Zero));
    }

    [Fact]
    public void A_distance_has_its_closest_points_and_supports()
    {
        Solid box = Fixtures.Box();
        Shape other = box.Moved(Transform3D.Translation(new Vector3D(20, 0, 0)));
        Result<DistanceResult> gap = box.Distance(other);
        Assert.True(gap.IsSuccess);
        Assert.Equal(10, gap.Value.Value, Tol);
        Assert.False(gap.Value.IsInnerSolution);
        Assert.NotEmpty(gap.Value.Solutions);
        DistanceSolution first = gap.Value.Solutions[0];
        Assert.Equal(10, first.OnA.X, Tol);
        Assert.Equal(20, first.OnB.X, Tol);
        Assert.False(first.SupportShapeA.IsNull);
        Assert.Equal(0, MeasureOperations.Distance(box, box).Value.Value, Tol);
    }

    [Fact]
    public void A_point_is_classified_in_out_or_on()
    {
        Solid box = Fixtures.Box();
        Assert.Equal(Containment.In, box.Classify(new Point3D(5, 10, 15)));
        Assert.Equal(Containment.Out, box.Classify(new Point3D(50, 10, 15)));
        Assert.Equal(Containment.On, MeasureOperations.Classify(box, new Point3D(0, 10, 15)));
    }

    [Fact]
    public void A_line_meets_the_faces_in_order()
    {
        Solid box = Fixtures.Box();
        var line = new Line3D(new Point3D(-5, 10, 15), Direction3D.UnitX);
        var hits = box.Intersect(line);
        Assert.Equal(2, hits.Length);
        Assert.Equal(5, hits[0].LineParameter, Tol);
        Assert.Equal(15, hits[1].LineParameter, Tol);
        Assert.Equal(0, hits[0].Point.X, Tol);
        Assert.Equal(Containment.In, hits[0].State);
        Assert.Throws<ArgumentException>(() => box.Intersect(default(Line3D)));
    }

    // A cut by a crossing cylinder of equal radius has faces bounded by walked curves. The fixed points gave 139.90 here for 118.73, 18% out
    [Fact]
    public void The_default_volume_is_the_precise_one_and_fixed_points_are_asked_for()
    {
        Shape cylinder = Solid.Cylinder(5, 10).Value.Shape.Transformed(Transform3D.Translation(new Vector3D(0, 0, -5))).Value.Shape;
        var turn = Transform3D.Rotation(new Axis3D(new Point3D(0, 0, 0), Direction3D.UnitX), -Math.PI / 2);
        Shape tool = Solid.Cylinder(5, 20).Value.Shape.Transformed(turn).Value.Shape
            .Transformed(Transform3D.Translation(new Vector3D(1.5e-5, -10, 0))).Value.Shape;
        Shape cut = cylinder.Cut(tool).Value.Shape;
        double precise = cut.IntegrateVolume(new MassOptions { Tolerance = 1e-9 });
        Assert.Equal((Math.PI * 25 * 10) - (16.0 / 3 * 125), precise, 1e-3);
        Assert.Equal(precise, cut.IntegrateVolume(), precise * 2e-6);
        Assert.True(Math.Abs(cut.IntegrateVolume(new MassOptions { Tolerance = 0 }) - precise) > 1);
        Assert.Equal(precise, cut.IntegrateVolume(MassOptions.Default), precise * 2e-6);
        Assert.Equal(1e-6, MassOptions.Default.Tolerance);
        Assert.Equal(0, new MassOptions { Tolerance = 0 }.Tolerance);
        Assert.Equal(Math.PI * 25 * 10, Fixtures.Cylinder().IntegrateVolume(new MassOptions { Tolerance = 0 }), 1e-6);
    }

    // Each of the first two never returned: decision 21506
    [Theory]
    [InlineData(double.NaN, 1)]
    [InlineData(double.PositiveInfinity, 1)]
    [InlineData(double.NegativeInfinity, 1)]
    [InlineData(1, double.NaN)]
    [InlineData(1e300, 1)]
    public async Task A_parameter_no_face_can_have_is_out_and_comes_back(double u, double v)
    {
        Face side = Fixtures.Cylinder().AllFaces().First(f => f.AsShape().AllEdges().Length >= 3);
        Containment asked = await Task.Run(() => side.Classify(new Point2D(u, v))).WaitAsync(TimeSpan.FromSeconds(30));
        Assert.Equal(Containment.Out, asked);
    }

    [Fact]
    public void A_null_shape_is_misuse()
    {
        Assert.Throws<ArgumentException>(() => default(Shape).IntegrateVolume());
        Assert.Throws<ArgumentException>(() => default(Shape).BoundingBox());
        Assert.Throws<ArgumentException>(() => Fixtures.Box().Distance(default(Shape)));
    }
}
