using Occt;
using Ops = Build123d.Operations;

namespace Build123d.Tests.Errors;

/// <summary>
/// A shape's memory estimate (the native shim's, which the native backend told the GC about as memory pressure, unit
/// 10B retiring the pressure with it): shared geometry counted once, detail counted.
/// </summary>
public class MemoryPressureTests
{
    [Fact]
    public void SmallShapesArentWorthTelling()
    {
        var line = Edge.MakeLine((0, 0, 0), (1, 0, 0)).Wrapped;
        Assert.InRange(Extras.EstimateMemory(line), 1, 4095);
    }

    [Fact]
    public void TheEstimateCountsSharedGeometryOnce()
    {
        var box = new Box(1, 1, 1);
        var one = Extras.EstimateMemory(box.TopoShape);
        var twice = Extras.EstimateMemory(new Compound([box, box]).TopoShape);
        var apart = Extras.EstimateMemory(new Compound([box, new Box(1, 1, 1)]).TopoShape);
        Assert.InRange(twice - one, 0, 200);   // the compound itself
        Assert.True(apart > 2 * one - 200);
    }

    [Fact]
    public void TheEstimateGrowsWithDetail()
    {
        var plain = Extras.EstimateMemory(new Box(10, 10, 10).TopoShape);
        var filleted = Extras.EstimateMemory(Ops.Fillet(new Box(10, 10, 10).Edges(), 1).TopoShape);
        var sphere = new Sphere(10);
        var meshed = Extras.EstimateMemory(sphere.MeshedCopy(0.001, 0.1, relative: false));
        Assert.True(filleted > 3 * plain, $"{filleted} vs {plain}");
        Assert.True(meshed > 1_000_000, $"a fine mesh is {meshed} bytes");
    }
}
