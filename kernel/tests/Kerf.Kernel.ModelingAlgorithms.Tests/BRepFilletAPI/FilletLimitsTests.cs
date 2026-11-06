// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Engine.Topology.Brep;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>What a fillet at a limit keeps of what ChFi3d's would have (decision 21201).</summary>
public class FilletLimitsTests
{
    private static int TangentEdges(Shape shape)
    {
        Face[] faces = [.. shape.AllFaces()];
        int count = 0;
        foreach (Edge edge in shape.AllEdges())
        {
            Face[] on = [.. faces.Where(f => f.AllEdges().Any(e => e.IsSame(edge)))];
            if (on.Length == 2 && Tool.Continuity(edge, on[0], on[1]) >= Continuity.G1)
            {
                count++;
            }
        }
        return count;
    }

    private static Edge[] Vertical(Shape shape) =>
        [.. shape.AllEdges().Where(e => Math.Abs(e.AllVertices()[0].Point.Z - e.AllVertices()[1].Point.Z) > 1e-9)];

    [Fact]
    public void Its_edges_are_coded_tangent_as_ChFi3d_codes_them()
    {
        // Across a 10 by 20 box, radius 10 uses up the 10 wide face and leaves the fillet tangent to what's left of the other
        Solid box = Solid.Box(10, 20, 30).Value;
        Edge edge = Vertical(box)[0];
        Assert.Equal(2, TangentEdges(box.Fillet([edge], 9).Value.Shape));
        Assert.Equal(1, TangentEdges(box.Fillet([edge], 10).Value.Shape));
    }

    [Fact]
    public void Two_fillets_that_meet_are_coded_tangent_to_each_other()
    {
        Solid box = Solid.Box(10, 10, 30).Value;
        Edge[] vertical = Vertical(box);
        Edge first = vertical[0];
        Edge opposite = vertical.First(e => !e.IsSame(first)
            && (Math.Abs(e.AllVertices()[0].Point.X - first.AllVertices()[0].Point.X) < 1e-9
                || Math.Abs(e.AllVertices()[0].Point.Y - first.AllVertices()[0].Point.Y) < 1e-9));
        // With each other, and each with what's left of the face it's also cut back
        Assert.Equal(3, TangentEdges(box.Fillet([first, opposite], 5).Value.Shape));
    }

    [Fact]
    public void The_shape_given_is_not_written()
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        double before = box.AsShape().AllFaces().Length;
        _ = box.Fillet([box.AllEdges()[0]], 10).Value;
        Assert.Equal(before, box.AsShape().AllFaces().Length);
    }
}
