using Kerf.Geometry;
using Kerf.Topology;
using Ops = Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Errors;

using Kerf.Modeling;

/// <summary>
/// A modelling call can be stopped, though it takes no token, and a request too large to be meant fails rather than taking
/// the machine's memory (kernel docs/reliability.md; decisions 1855 and 1856).
/// </summary>
public class ReliabilityTests
{
    private static CancellationToken Cancelled()
    {
        using var source = new CancellationTokenSource();
        CancellationToken token = source.Token;
        source.Cancel();
        return token;
    }

    [Fact]
    public void TheTokenInForceStopsAModellingCall()
    {
        Part box = new Box(10, 10, 10);
        Part cylinder = new Cylinder(2, 20);
        Kerf.Topology.Shape shape = box;
        var edges = shape.Edges();
        using (Cancellation.Use(Cancelled()))
        {
            Assert.ThrowsAny<OperationCanceledException>(() => box - cylinder);
            Assert.ThrowsAny<OperationCanceledException>(() => box + cylinder);
            Assert.ThrowsAny<OperationCanceledException>(() => Ops.Fillet(shape, edges, 1));
            Assert.ThrowsAny<OperationCanceledException>(() => Ops.Chamfer(shape, edges, 1));
            Assert.ThrowsAny<OperationCanceledException>(() => Ops.Offset(shape, 1));
            Assert.ThrowsAny<OperationCanceledException>(() => shape.Tessellate(0.1));
            Assert.ThrowsAny<OperationCanceledException>(() => Exporters3D.ExportStep(shape, new MemoryStream()));
        }

        Assert.Equal(1000 - Math.PI * 4 * 10, (box - cylinder).Volume, 3);
    }

    // reliability.md's island 3: a loft from a square to a circle, every edge chamfered. OCCT fails at its top corners, where
    // the side faces turn tangent; Kerf's own construction builds it, proved, within its 20 s limit (kernel decision 30121)
    [Fact]
    public void A_lofts_chamfer_that_OCCT_cant_build_is_built()
    {
        Part loft = Ops.Loft([(Kerf.Topology.Shape)(Plane.XY * new Rectangle(10, 10)), (Kerf.Topology.Shape)(Plane.XY.Offset(10) * new Circle(4))]).Value;
        Kerf.Topology.Shape shape = loft;
        var made = Ops.Chamfer(shape, shape.Edges(), 0.05);
        Assert.True(made.IsSuccess, made.Error?.ToString());
        Kerf.Topology.Shape chamfered = (Kerf.Topology.Shape)made.Value!;
        Assert.True(Kerf.Healing.HealingOperations.Validate(chamfered).IsValid);
        Assert.Equal(740.8526, chamfered.Volume, 3);
    }

    [Theory]
    [InlineData(2_000_000_000)]
    [InlineData(LocationList.MaxCount + 1)]
    public void APatternTooLargeToBeMeantIsRefused(int count) =>
        Assert.Throws<ArgumentException>(() => new PolarLocations(10, count));

    [Fact]
    public void APatternOfManyRowsIsRefusedBeforeItsCountOverflows()
    {
        Assert.Throws<ArgumentException>(() => new GridLocations(1, 1, 100_000, 100_000));
        Assert.Throws<ArgumentException>(() => new HexLocations(1, 100_000, 100_000));
        Assert.Equal(100, new GridLocations(1, 1, 10, 10).Count);
    }

    // A spline's pipe at 0.72 failed after 320,000 sections for each end, 5 to 12 s. It should still fail, and the small one still work (decision 1884)
    [Fact]
    public void A_fillet_that_cannot_fit_a_pipe_still_fails_and_a_small_one_works()
    {
        var path = new Spline(new Point3D(0, 0, 0), new Point3D(10, 0, 10), new Point3D(20, 10, 20), new Point3D(20, 20, 30));
        Shape pipe = Ops.Sweep(Plane.XY * new Circle(1), (Shape)path).Value;
        Assert.False(Ops.Fillet(pipe, pipe.Edges(), 0.72).IsSuccess);
        Assert.True(Ops.Fillet(pipe, pipe.Edges(), 0.1).IsSuccess);
    }
}
