using Kerf.Fillets;
using Kerf.Modeling;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Topology;

/// <summary>
/// A fillet whose edges cross fails, carrying the result (kernel decision 2123; OCCT #1570): the kernel's error passes
/// through, with its code <c>fillet.self-intersecting</c> and the edges, as a chamfer's does; OCCT #1570's fillet, on the
/// ball since kernel decision 30013, fails for an edge's tolerance instead.
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
        // On the ball (kernel decision 30013) one of the two slivers where the fillets' ends meet is gone and the other's
        // edge is made with a tolerance of 2.4e10: the kernel's FilletIssueTests hold the same
        Assert.Equal(DiagnosticCode.InvalidResult, crossed.Error!.Code);
        Assert.True(crossed.TryGetInvalid(out Part? invalid));
        Assert.True(invalid!.IsValid);

        // The shorthand throws the same, and the result carries the shape
        var thrown = Assert.Throws<ModelingException>(() => part.Fillet(5.5, [edges[0], edges[7]]));
        Assert.Equal(DiagnosticCode.InvalidResult, thrown.Diagnostics[0].Code);
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

    [Fact]
    public void A_fillet_whose_face_narrows_to_a_point_is_built()
    {
        // The corpus's blend/simple/X4 (kernel decision 2131): a block with a half round notch of 30 in its front, the front's
        // top edge beside the notch filleted at 30, where the fillet's line on the top just reaches the notch's deepest point. The
        // cylinder's seam is turned away from the notch's edges, as the corpus's file has it
        Part block = new Box(120, 100, 50) - new Pos(0, -50, 0) * new Rotation(z: -90) * new Cylinder(30, 50);
        var edge = block.Edges().Where(e => e.GeomType == GeomType.Line && Math.Abs(e.Center().Y + 50) < 1e-9
                                            && Math.Abs(e.Center().Z - 25) < 1e-9 && e.Center().X > 30).Single();
        Result<Part> filleted = Fillet(block, [edge], 30);
        Assert.True(filleted.IsSuccess);
        Part part = filleted.Value;
        Assert.True(part.IsValid);
        Assert.Equal(10, part.Faces().Count);

        // The block less the fillet's corner all along it, less what the notch had already taken, by the section
        double expected = 120.0 * 100 * 50 - Math.PI * 900 / 2 * 50 - 120 * (1 - Math.PI / 4) * 900;
        const int n = 200_000;
        for (int i = 0; i < n; i++)
        {
            double u = 30.0 * (i + 0.5) / n;
            expected += 2 * Math.Sqrt(900 - u * u) * (30 - Math.Sqrt(900 - (30 - u) * (30 - u))) * (30.0 / n);
        }
        Assert.Equal(expected, part.Volume, 0.01);
    }
}
