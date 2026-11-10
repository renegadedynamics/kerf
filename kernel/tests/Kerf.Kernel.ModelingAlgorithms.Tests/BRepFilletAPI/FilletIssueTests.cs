// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Fillets;
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
}
