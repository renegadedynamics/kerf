// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Fillets and chamfers, 3D and 2D (docs/api/fillets.md; unit 10L2).</summary>
public class FilletTests
{
    private static bool IsVertical(Edge edge)
    {
        Vertex[] ends = [.. edge.AllVertices()];
        return ends.Length == 2 && Math.Abs(ends[0].Point.Z - ends[1].Point.Z) > 1e-9;
    }

    [Fact]
    public void A_fillet_rounds_its_edges_and_its_history_names_the_new_faces()
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        Edge[] vertical = [.. box.AllEdges().Where(IsVertical)];
        Built<Shape> rounded = box.Fillet(vertical, 1).Value;
        Assert.Equal(10, rounded.Shape.AllFaces().Length);
        Assert.Single(rounded.History.Generated(vertical[0]));
        Assert.Equal(ShapeType.Face, rounded.History.Generated(vertical[0])[0].Type);
        Built<Shape> varying = FilletOperations.Fillet(box, [new FilletSpec(vertical[0], 1, EndRadius: 2)]).Value;
        Assert.Equal(7, varying.Shape.AllFaces().Length);
    }

    [Fact]
    public void A_fillet_too_large_fails_and_names_its_edges()
    {
        Solid box = Solid.Box(2, 2, 2).Value;
        Edge edge = box.AllEdges()[0];
        var rounded = box.Fillet([edge], 5);
        Assert.False(rounded.IsSuccess);
        Assert.StartsWith("fillet.", rounded.Error!.Code.Value, StringComparison.Ordinal);
    }

    [Fact]
    public void A_radius_equal_to_a_faces_width_is_built_and_one_above_says_what_fits()
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        Edge edge = box.AllEdges()[0];
        Assert.Equal(10, box.FilletLimit([edge]));
        Built<Shape> rounded = box.Fillet([edge], 10).Value;
        Assert.Equal(5, rounded.Shape.AllFaces().Length);
        Assert.Single(rounded.History.Generated(edge));
        Assert.True(rounded.History.IsRemoved(edge));

        var tooLarge = box.Fillet([edge], 11);
        Assert.Equal(FilletOperations.TooLarge, tooLarge.Error!.Code);
        Assert.Contains("the most that fits is 10", tooLarge.Error.Message, StringComparison.Ordinal);
        Assert.Equal(FilletOperations.TooLarge, box.Chamfer([edge], 11).Error!.Code);
        Assert.Equal(5, box.Chamfer([edge], 10).Value.Shape.AllFaces().Length);
        Assert.Contains("10 wide", box.Chamfer([new ChamferSpec(edge, 11, Distance2: 2)]).Error!.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void A_fillet_limit_on_a_rim_is_what_its_faces_allow_and_on_other_curves_nothing()
    {
        Solid cylinder = Solid.Cylinder(5, 20).Value;
        Assert.Equal(5, cylinder.FilletLimit(cylinder.AllEdges().Where(e => e.AllVertices().Length == 1))!.Value, 1e-9);
        Solid sphere = Solid.Sphere(5).Value;
        Assert.Null(sphere.FilletLimit(sphere.AllEdges()));
    }


    [Fact]
    public void A_fillets_misuse_throws()
    {
        Solid box = Solid.Box(2, 2, 2).Value;
        Edge stranger = Edge.Line(new(0, 0, 9), new(1, 0, 9)).Value;
        Assert.Throws<ArgumentException>(() => box.Fillet([stranger], 1));
        Assert.Throws<ArgumentException>(() => box.Fillet(Array.Empty<Edge>(), 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => box.Fillet([box.AllEdges()[0]], 0));
    }

    [Fact]
    public void Chamfers_by_distance_two_distances_or_an_angle()
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        Edge edge = box.AllEdges()[0];
        Assert.Equal(7, box.Chamfer([edge], 1).Value.Shape.AllFaces().Length);
        Assert.Equal(7, box.Chamfer([new ChamferSpec(edge, 1, Distance2: 2)]).Value.Shape.AllFaces().Length);
        Assert.Equal(7, box.Chamfer([new ChamferSpec(edge, 1, Angle: Math.PI / 6)]).Value.Shape.AllFaces().Length);
        Assert.Throws<ArgumentException>(() => box.Chamfer([new ChamferSpec(edge, 1, 2, Math.PI / 6)]));
    }

    [Fact]
    public void Fillets_are_cancelled_by_their_token()
    {
        using var source = new CancellationTokenSource();
        source.Cancel();
        Solid box = Solid.Box(10, 10, 10).Value;
        Assert.Throws<OperationCanceledException>(() => box.Fillet([box.AllEdges()[0]], 1, cancellationToken: source.Token));
    }

    [Fact]
    public void A_planar_face_has_its_corners_rounded_and_chamfered()
    {
        Face square = Face.FromWire(ConstructionTests.Square(10)).Value;
        Built<Face> rounded = square.Fillet2D(square.AllVertices(), 2).Value;
        Assert.Equal(8, rounded.Shape.AllEdges().Length);
        Assert.Single(rounded.History.Generated(square.AllVertices()[0]));
        Assert.NotEmpty(rounded.History.Modified(square.AllEdges()[0]));
        Assert.Equal(5, square.Chamfer2D([square.AllVertices()[0]], 1).Value.Shape.AllEdges().Length);

        var tooLarge = square.Fillet2D([square.AllVertices()[0]], 50);
        Assert.False(tooLarge.IsSuccess);
        Assert.StartsWith("fillet.", tooLarge.Error!.Code.Value, StringComparison.Ordinal);
        Assert.Throws<ArgumentException>(() => square.Fillet2D([Vertex.At(new Point3D(50, 50, 0))], 1));
    }

    [Fact]
    public void A_chamfer_that_fails_says_which_edges_and_why()
    {
        // OCCT #1632: a chamfer had no way to say which contours failed
        Shape cut = Solid.Sphere(5).Value.Shape.Cut(Solid.Box(5, 5, 5).Value.Shape).Value.Shape;
        Edge[] edges = [.. cut.AllEdges().Where(static e => !e.IsDegenerated)];
        Result<Built<Shape>> chamfer = cut.Chamfer(edges, 2);
        Assert.True(chamfer.IsFailure);
        Assert.Equal(new DiagnosticCode("fillet.failed"), chamfer.Error!.Code);
        Assert.InRange(chamfer.Error.Shapes.Length, 1, edges.Length);
        Assert.All(chamfer.Error.Shapes, shape => Assert.Contains(edges, edge => edge.IsSame(shape.As<Edge>())));

        // Past the cut faces' quarter circles of 5 it's refused before ChFi3d, naming those three (decision 1985)
        Result<Built<Shape>> past = cut.Chamfer(edges, 6);
        Assert.Equal(new DiagnosticCode("fillet.too-large"), past.Error!.Code);
        Assert.Contains("the most that fits is 5,", past.Error.Message, StringComparison.Ordinal);
        Assert.Equal(3, past.Error.Shapes.Length);
    }

    [Fact]
    public void A_corner_chamfered_twice_is_a_plain_failure()
    {
        // OCCT #1431: a crash there, and an internal fault here
        Wire outline = Wire.Polygon([new(0, 0, 0), new(10, 0, 0), new(10, 10, 0), new(0, 10, 0)], closed: true).Value;
        Face face = Face.FromWire(outline).Value;
        Edge[] edges = [.. face.AllEdges()];
        Edge bottom = edges.First(static e => e.BoundingBox().Max.Y < 1e-3), right = edges.First(static e => e.BoundingBox().Min.X > 9.99);
        Assert.Equal(100 - 0.5, face.Chamfer2D([(bottom, right)], 1).Value.Shape.IntegrateArea(), 1e-9);
        Result<Built<Face>> twice = face.Chamfer2D([(bottom, right), (bottom, right)], 1);
        Assert.True(twice.IsFailure);
        Assert.Equal(new DiagnosticCode("fillet.connexion-error"), twice.Error!.Code);
    }
}
