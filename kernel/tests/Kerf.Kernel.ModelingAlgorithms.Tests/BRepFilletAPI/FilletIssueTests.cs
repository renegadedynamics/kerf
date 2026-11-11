// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Fillets;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>Fillets that OCCT's issues report wrong, and Kerf now makes right.</summary>
public class FilletIssueTests
{
    private static Shape Read(string name)
    {
        using Stream stream = typeof(FilletIssueTests).Assembly.GetManifestResourceStream($"BRepFilletAPI.Issues.{name}.brep")
                              ?? throw new InvalidOperationException(name);
        BrepTools.Read(out Shape shape, new StreamReader(stream));
        return shape;
    }

    // OCCT #1639's solid, in the draft's order of faces and reversed: a prism of a quarter disc beside a strip, 10 high,
    // its sides drafted 5 degrees. Filleted at 2 along the top's arc and the straight edge that meets it at its end of
    // greater y, the corner's curve between the two fillets crosses the straight fillet's seam. In the draft's order its
    // walking line started on the seam's far side, and the approximation threw it 190 off the surfaces (decision 2119).
    [Theory]
    [InlineData("1639-as-built")]
    [InlineData("1639-reversed")]
    public void Issue_1639_a_fillet_is_the_same_whatever_the_order_of_the_shells_faces(string name)
    {
        Shape solid = Read(name);
        Face top = solid.AllFaces().Single(f => f.AllVertices().All(v => Math.Abs(v.Point.Z - 10) < 1e-6));
        Vertex corner = top.AllVertices().MaxBy(v => v.Point.Y)!;
        Edge[] edges = [.. top.AllEdges().Where(e => e.AllVertices().Any(v => v.IsSame(corner)))];
        Assert.Equal(2, edges.Length);

        Shape filleted = solid.Fillet(edges, 2.0).Value.Shape;
        Assert.Equal(29799.8675, filleted.IntegrateVolume(new MassOptions { Tolerance = 1e-6 }), 3);
        Assert.All(filleted.AllEdges(), static e => Assert.True(Tool.Tolerance(e) < 1e-5));
    }

    // OCCT #1570's part (a FreeCAD report), its edges 1 and 8 filleted together. At 5.5 the fillet is valid by the shape's
    // own check with two pairs of its edges running along each other, the sides of slivers where the fillets' ends meet
    // (bopcheck: E/E x33 x34, E/E x37 x38); at 5 it's clean (decision 2120). On the ball (decision 2160) one of the slivers
    // is gone and the other's edge is made with a tolerance of 2.4e10, so the fillet at 5.5 still fails, for that.
    [Fact]
    public void Issue_1570_a_fillet_whose_edges_cross_fails_naming_them()
    {
        Shape solid = Read("1570");
        ImmutableArray<Edge> edges = solid.AllEdges();
        Result<Built<Shape>> crossed = solid.Fillet([edges[0], edges[7]], 5.5);
        Assert.False(crossed.IsSuccess);
        Assert.Equal(DiagnosticCode.InvalidResult, crossed.Error!.Code);
        Assert.Contains("isn't reliable: it has a tolerance of", crossed.Error.Message, StringComparison.Ordinal);
        Assert.True(crossed.TryGetInvalid(out Built<Shape>? invalid));
        Assert.True(invalid!.Shape.Validate().IsValid);

        Assert.True(solid.Fillet([edges[0], edges[7]], 5.0).IsSuccess);
        Assert.True(solid.Fillet([edges[0], edges[7]], 5.5, new FilletOptions { CheckResult = false }).IsSuccess);
    }

    // OCCT #1430's part, its valid case: four edges at 1, which no longer intersects itself (decision 2120)
    [Fact]
    public void Issue_1430_a_fillet_that_no_longer_intersects_itself_is_a_success()
    {
        Shape solid = Read("1430");
        ImmutableArray<Edge> edges = solid.AllEdges();
        Assert.True(solid.Fillet([edges[91], edges[94], edges[97], edges[98]], 1.0).IsSuccess);
    }
}
