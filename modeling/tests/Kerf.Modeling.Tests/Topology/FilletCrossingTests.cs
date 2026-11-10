using Kerf.Fillets;
using Kerf.Modeling;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Topology;

/// <summary>
/// A fillet whose edges cross fails, carrying the result (kernel decision 2123; OCCT #1570): the kernel's error passes
/// through, with its code <c>fillet.self-intersecting</c> and the edges, as a chamfer's does.
/// </summary>
public class FilletCrossingTests
{
    // OCCT #1570's part, as the kernel's FilletIssueTests hold it
    private static Part Issue1570()
    {
        using Stream stream = typeof(FilletCrossingTests).Assembly.GetManifestResourceStream("Topology.Issues.1570.brep")!;
        return new Part([.. Importers.ImportBrep(stream).Value.Solids()]);
    }

    [Fact]
    public void A_fillet_whose_edges_cross_fails_with_the_kernels_code_carrying_the_result()
    {
        Part part = Issue1570();
        var edges = part.Edges();
        Result<Part> crossed = Fillet(part, [edges[0], edges[7]], 5.5);
        Assert.True(crossed.IsFailure);
        Assert.Equal(FilletOperations.SelfIntersecting, crossed.Error!.Code);
        Assert.Equal("The fillet's result isn't reliable: two of its edges run along each other. Try a smaller radius, or MaxFillet to find the largest that works.",
                     crossed.Error.Message);
        Assert.Equal(2, crossed.Error.Shapes.Length);
        Assert.True(crossed.TryGetInvalid(out Part? invalid));
        Assert.True(invalid!.IsValid);
        Assert.Equal(10, invalid.Faces().Count);

        // The shorthand throws the same, and the result carries the shape
        var thrown = Assert.Throws<ModelingException>(() => part.Fillet(5.5, [edges[0], edges[7]]));
        Assert.Equal(FilletOperations.SelfIntersecting, thrown.Diagnostics[0].Code);
        Assert.True(ModelingDiagnostics.TryGetInvalid(thrown, out _));

        // Below it the fillet is clean
        Assert.True(Fillet(part, [edges[0], edges[7]], 5).IsSuccess);
    }

    [Fact]
    public void MaxFillet_stops_below_the_radius_whose_edges_cross()
    {
        Part part = Issue1570();
        var edges = part.Edges();
        double largest = part.MaxFillet([edges[0], edges[7]], tolerance: 0.5);
        Assert.InRange(largest, 4.6, 5.3);
        Assert.True(Fillet(part, [edges[0], edges[7]], largest).IsSuccess);
    }
}
