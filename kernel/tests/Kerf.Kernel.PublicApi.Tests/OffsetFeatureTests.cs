// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Features;
using Kerf.Geometry;
using Kerf.Offsets;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Offsets and features (docs/api/offsets-features.md; unit 10L2).</summary>
public class OffsetFeatureTests
{
    private static Face Top(Solid box) => box.AllFaces().MaxBy(f => f.AllVertices().Min(v => v.Point.Z));

    private static double MaxX(Shape shape) => shape.AllVertices().Max(v => v.Point.X);

    [Fact]
    public void A_box_offset_outwards_grows_with_its_history()
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        Built<Shape> grown = box.Offset(1, new OffsetOptions { Join = JoinKind.Intersection }).Value;
        Assert.Equal(11, MaxX(grown.Shape), 6);
        Assert.Equal(6, grown.Shape.AllFaces().Length);
        Assert.Single(grown.History.Generated(box.AllFaces()[0]));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => box.Offset(0));
    }

    [Fact]
    public void A_hollow_box_has_an_opening_and_walls()
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        Built<Solid> shelled = box.Hollow([Top(box)], -1).Value;
        Assert.Equal(11, shelled.Shape.AllFaces().Length);
        Assert.NotEmpty(shelled.History.Generated(box.AllFaces()[0]));
        KernelAssert.Throws<ArgumentException>(() => box.Hollow([Face.FromWire(ConstructionTests.Square(1, 50)).Value], -1));
    }

    [Fact]
    public void An_offset_in_the_plane_of_a_skew_wire_fails()
    {
        Wire skew = Wire.Polygon([new(0, 0, 0), new(1, 0, 0), new(1, 1, 1), new(0, 1, 0)], closed: true).Value;
        var outline = skew.Offset2D(0.1);
        Assert.False(outline.IsSuccess);
        Assert.StartsWith("offset.", outline.Error!.Code.Value, StringComparison.Ordinal);
    }

    [Fact]
    public void A_face_thickens_into_a_solid_and_a_wire_offsets_in_its_plane()
    {
        Face square = Face.FromWire(ConstructionTests.Square(10)).Value;
        Built<Solid> slab = square.Thicken(2).Value;
        Assert.Equal(6, slab.Shape.AllFaces().Length);
        Shape outline = square.Offset2D(1, new Offset2DOptions { Join = JoinKind.Intersection }).Value.Shape;
        Assert.Equal(11, MaxX(outline), 6);
        KernelAssert.Throws<ArgumentException>(() => Solid.Box(1, 1, 1).Value.Shape.Thicken(1));
    }

    [Fact]
    public void Offsets_are_cancelled_by_their_token()
    {
        using var source = new CancellationTokenSource();
        source.Cancel();
        Solid box = Solid.Box(10, 10, 10).Value;
        Assert.Throws<OperationCanceledException>(() => box.Offset(1, cancellationToken: source.Token));
    }

    [Fact]
    public void A_draft_tilts_side_faces_and_names_the_face_it_cant()
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        Face side = box.AllFaces().First(f => f.AllVertices().All(v => Math.Abs(v.Point.X) < 1e-9));
        var basePlane = new Plane3D(Point3D.Origin, Direction3D.UnitZ);
        Built<Shape> drafted = box.Draft([side], Direction3D.UnitZ, 5 * Math.PI / 180, basePlane).Value;
        Assert.Equal(6, drafted.Shape.AllFaces().Length);
        Assert.Single(drafted.History.Generated(side));
        Assert.Contains(drafted.Shape.AllVertices(), v => Math.Abs(Math.Abs(v.Point.X) - 10 * Math.Tan(5 * Math.PI / 180)) < 1e-6);

        Face top = Top(box);
        var cant = box.Draft([top], Direction3D.UnitZ, 5 * Math.PI / 180, basePlane);
        Assert.False(cant.IsSuccess);
        Assert.Equal(FeatureOperations.DraftFaceFailed, cant.Error!.Code);
    }

    [Fact]
    public void A_boss_extruded_up_to_a_height_and_a_split_face()
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        Face top = Top(box);
        Face profile = Face.FromWire(Wire.Polygon([new(3, 3, 10), new(6, 3, 10), new(6, 6, 10), new(3, 6, 10)], closed: true).Value).Value;
        Built<Shape> boss = box.ExtrudeUntil(profile, top, 0, new ExtrudeUntilOptions { Until = ExtrudeLimit.Height, Height = 5 }).Value;
        Assert.Equal(15, boss.Shape.AllVertices().Max(v => v.Point.Z), 6);
        KernelAssert.Throws<ArgumentException>(() => box.ExtrudeUntil(profile, top, 0, new ExtrudeUntilOptions { Until = ExtrudeLimit.Face }));

        Edge across = Edge.Line(new(5, 0, 10), new(5, 10, 10)).Value;
        Built<Shape> split = box.SplitFaces([(top, (Shape)across)]).Value;
        Assert.Equal(7, split.Shape.AllFaces().Length);
        Assert.Equal(2, split.History.Modified(top).Count);
    }
}
