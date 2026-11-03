using Build123d.Occt;
using Ops = Build123d.Operations;

namespace Build123d.Tests.Errors;

/// <summary>
/// New results tell the GC about the native memory they hold, so a spreadsheet that drops results without disposing
/// them is collected before native memory balloons; sub-shapes taken out of a result aren't counted again.
/// </summary>
public class MemoryPressureTests
{
    private static long Pressure(Shape shape) => Extras.MemoryPressureOf(shape.TopoShape);

    [Fact]
    public void SubShapesAndCopiesDontCountAgain()
    {
        var part = Ops.Fillet(new Box(10, 10, 10).Edges(), 1);
        Assert.All(part.Faces(), f => Assert.Equal(0, Pressure(f)));
        // A relocated or relabelled copy shares the result's geometry and its wrapper's pressure
        Assert.Equal(0, Pressure(part.Moved(new Location((1.0, 0.0, 0.0)))));
        Assert.Same(part.TopoShape, part.WithLabel("x").TopoShape);
    }

    [Fact]
    public void SmallShapesArentWorthTelling()
    {
        var line = Edge.MakeLine((0, 0, 0), (1, 0, 0)).Wrapped;
        Assert.InRange(Extras.EstimateMemory(line), 1, 4095);
        Extras.AddMemoryPressure(line);
        Assert.Equal(0, Extras.MemoryPressureOf(line));
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
