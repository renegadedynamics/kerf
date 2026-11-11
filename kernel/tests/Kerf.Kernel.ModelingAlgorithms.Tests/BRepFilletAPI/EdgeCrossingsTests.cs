// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>
/// The check that a fillet's or chamfer's edges don't cross (decision 2123; OCCT #1570), and what it costs: the pairs it
/// asks of the intersector are counted (<c>EdgeCrossings.Counting</c>), on the test's own thread, so the counts are its own.
/// </summary>
public class EdgeCrossingsTests
{
    private static Shape Read(string name)
    {
        using Stream stream = typeof(EdgeCrossingsTests).Assembly.GetManifestResourceStream($"BRepFilletAPI.Issues.{name}.brep")
                              ?? throw new InvalidOperationException(name);
        BrepTools.Read(out Shape shape, new StreamReader(stream));
        return shape;
    }

    // What the checks made by act asked, counted on this thread alone, so the counts are the test's own however the tests are run
    private static (int Checks, int Pairs, int Settled, int Intersections) Counting(Action act)
    {
        EdgeCrossings.Counts counts = EdgeCrossings.Counting(act);
        return (counts.Checks, counts.BoxPairs, counts.Settled, counts.Intersections);
    }

    [Fact]
    public void Issue_1570s_fillet_has_two_pairs_of_edges_that_run_along_each_other_and_its_input_none()
    {
        Shape solid = Read("1570");
        ImmutableArray<Edge> edges = solid.AllEdges();
        Shape result = solid.Fillet([edges[0], edges[7]], 5.5, new FilletOptions { CheckResult = false }).Value.Shape;
        IReadOnlyList<EdgeCrossings.Crossing> crossings = EdgeCrossings.All(solid, result);
        // one pair on the ball (decision 2160), where the off-ball fillet had two
        Assert.Single(crossings);
        Assert.All(crossings, c => Assert.True(c.Along));
        Assert.All(crossings, c => Assert.False(c.Edge1.IsSame(c.Edge2)));
        // bopcheck's E/E x33 x34 and x37 x38: each pair is the two sides of a sliver where the fillets' ends meet, 4.26 long
        Assert.All(crossings, c => Assert.Equal(4.26, c.Edge1.AllVertices().Select(v => v.Point).Max(p => p.DistanceTo(c.Point)), 1));

        // The same result, asked about itself, has nothing new to check
        Assert.Null(EdgeCrossings.Problem(result, result, "The fillet's result", FilletOperations.SelfIntersecting));
        Assert.Empty(EdgeCrossings.All(solid, solid));
    }

    [Fact]
    public void A_box_with_every_edge_rounded_is_settled_by_boxes_alone()
    {
        Shape box = new MakeBox(20, 20, 20).Shape();
        Shape rounded = box.Fillet(box.AllEdges(), 2.0, new FilletOptions { CheckResult = false }).Value.Shape;
        (int checks, int pairs, int settled, int intersections) = Counting(() =>
            Assert.Null(EdgeCrossings.Problem(box, rounded, "The fillet's result", FilletOperations.SelfIntersecting)));
        Assert.Equal(1, checks);
        // 56 edges, all of faces the fillet made or trimmed; the pairs whose boxes meet all share a vertex, where the
        // fillets' lines meet the corners' arcs tangentially or the arcs meet at right angles: cut back and boxed apart
        Assert.True(pairs > 100, $"{pairs} pairs");
        Assert.True(settled > pairs / 2, $"{settled} of {pairs} pairs settled");
        Assert.Equal(0, intersections);

        Shape chamfered = box.Chamfer(box.AllEdges(), 2.0, new ChamferOptions { CheckResult = false }).Value.Shape;
        (_, _, _, intersections) = Counting(() =>
            Assert.Null(EdgeCrossings.Problem(box, chamfered, "The chamfer's result", FilletOperations.SelfIntersecting)));
        Assert.Equal(0, intersections);
    }

    [Fact]
    public void Only_the_edges_of_faces_the_fillet_touched_are_checked()
    {
        // OCCT #1430's part, 35 faces: a fillet of four edges at 1 leaves most faces as they were, and their edges are
        // asked about only against the new ones' boxes
        Shape solid = Read("1430");
        ImmutableArray<Edge> edges = solid.AllEdges();
        Shape result = solid.Fillet([edges[91], edges[94], edges[97], edges[98]], 1.0, new FilletOptions { CheckResult = false }).Value.Shape;
        (int checks, int pairs, _, int intersections) = Counting(() =>
            Assert.Null(EdgeCrossings.Problem(solid, result, "The fillet's result", FilletOperations.SelfIntersecting)));
        Assert.Equal(1, checks);
        Assert.True(pairs < 100, $"{pairs} pairs of {result.AllEdges().Length} edges");
        Assert.Equal(0, intersections);
    }

    [Fact]
    public void A_null_or_untouched_result_is_no_problem()
    {
        Shape box = new MakeBox(10, 10, 10).Shape();
        Assert.Null(EdgeCrossings.Problem(box, new Shape(), "x", FilletOperations.SelfIntersecting));
        Assert.Null(EdgeCrossings.Problem(new Shape(), box, "x", FilletOperations.SelfIntersecting));
        Assert.Null(EdgeCrossings.Problem(box, box, "x", FilletOperations.SelfIntersecting));
    }
}
