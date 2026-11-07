using Kerf.Geometry;
using Kerf.Topology;
using Ops = Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Errors;

using Kerf.Modeling;

/// <summary>
/// A modelling call can be stopped, though it takes no token, and a request too large to be meant fails rather than taking
/// the machine's memory (kernel docs/reliability.md; decisions 21103 and 21104).
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
}
