// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Fillets;
using Kerf.Geometry;
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
}
