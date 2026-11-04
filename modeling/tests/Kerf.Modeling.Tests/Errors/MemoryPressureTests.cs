using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Errors;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>
/// A shape's memory estimate (the native shim's, which the native backend told the GC about as memory pressure, unit
/// 10B retiring the pressure with it): shared geometry counted once, detail counted.
/// </summary>
public class MemoryPressureTests
{
    [Fact]
    public void SmallShapesArentWorthTelling()
    {
        var line = Edge.MakeLine(new Vector(0, 0, 0), new Vector(1, 0, 0)).Wrapped;
        Assert.InRange(EngineShapes.EstimateMemory(line), 1, 4095);
    }

    [Fact]
    public void TheEstimateCountsSharedGeometryOnce()
    {
        var box = new Box(1, 1, 1);
        var one = EngineShapes.EstimateMemory(box.TopoShape);
        var twice = EngineShapes.EstimateMemory(new Compound([box, box]).TopoShape);
        var apart = EngineShapes.EstimateMemory(new Compound([box, new Box(1, 1, 1)]).TopoShape);
        Assert.InRange(twice - one, 0, 200);   // the compound itself
        Assert.True(apart > 2 * one - 200);
    }

    [Fact]
    public void TheEstimateGrowsWithDetail()
    {
        var plain = EngineShapes.EstimateMemory(new Box(10, 10, 10).TopoShape);
        var filleted = EngineShapes.EstimateMemory(Ops.Fillet(new Box(10, 10, 10).Edges(), 1).TopoShape);
        var sphere = new Sphere(10);
        var meshed = EngineShapes.EstimateMemory(sphere.MeshedCopy(0.001, 0.1, relative: false));
        Assert.True(filleted > 3 * plain, $"{filleted} vs {plain}");
        Assert.True(meshed > 1_000_000, $"a fine mesh is {meshed} bytes");
    }
}
