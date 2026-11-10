namespace Kerf.Modeling.Tests.Immutability;

using System.Reflection;
using Kerf.Modeling;

/// <summary>
/// Volumes integrated on several threads as a caller asks (decision 2092): by the <c>parallelVolumes</c> argument of
/// <c>Cut</c>, <c>Fuse</c>, <c>Intersect</c> and <c>ComputeVolume</c>, or by the scope <see cref="ParallelVolumes"/> around code
/// that passes none, operators and <c>Volume</c> included. On or off, the same parts and figures to the bit.
/// </summary>
public class ParallelVolumesTests
{
    // How many volumes this thread has integrated on several threads (the kernel's count, for the tests)
    private static readonly FieldInfo Count = typeof(Kerf.Measurement.MassOptions).Assembly
        .GetType("Kerf.Engine.Measurement.MassPropertiesBuilder")!
        .GetField("ParallelIntegrationsOnThisThread", BindingFlags.Static | BindingFlags.NonPublic)!;

    private static int Integrations => (int)Count.GetValue(null)!;

    // A plate with 20 holes: the cut's check integrates 20 cylindrical faces
    private static Part Plate() => new Box(100, 40, 6);

    private static IReadOnlyList<Part> Holes() =>
        [.. Enumerable.Range(0, 20).Select(i => (Part)(new Pos(-45 + 9 * (i % 10), -10 + 20 * (i / 10), 0) * new Cylinder(1.5 + 0.05 * i, 10)))];

    private static byte[] BrepOf(Shape shape)
    {
        using var stream = new MemoryStream();
        Assert.True(Exporters3D.ExportBrep(shape, stream).IsSuccess);
        return stream.ToArray();
    }

    private static long Bits(double d) => BitConverter.DoubleToInt64Bits(d);

    [Fact]
    public void A_cut_is_the_same_on_and_off_and_parallel_only_when_asked()
    {
        Part plate = Plate();
        IReadOnlyList<Part> holes = Holes();

        int before = Integrations;
        Shape serial = plate.Cut(holes, parallelVolumes: false);
        Shape byDefault = plate.Cut(holes);
        Shape byOperator = plate - holes.Select(h => (Shape)h);
        Assert.Equal(before, Integrations);

        Shape parallel = plate.Cut(holes, parallelVolumes: true);
        Assert.True(Integrations > before);
        Assert.Equal(BrepOf(serial), BrepOf(parallel));
        Assert.Equal(BrepOf(serial), BrepOf(byDefault));

        // The scope, for operators; an argument overrides it
        before = Integrations;
        using (ParallelVolumes.Use())
        {
            Shape inScope = plate - holes.Select(h => (Shape)h);
            int after = Integrations;
            Assert.True(after > before);
            Assert.Equal(BrepOf(byOperator), BrepOf(inScope));
            Assert.Equal(BrepOf(serial), BrepOf(plate.Cut(holes, parallelVolumes: false)));
            Assert.Equal(after, Integrations);
        }
    }

    [Fact]
    public void A_fuse_and_an_intersect_are_the_same_on_and_off()
    {
        Part plate = Plate();
        IReadOnlyList<Part> pins = [.. Holes().Select(h => (Part)(new Pos(0, 0, 4) * h))];

        Shape fused = plate.Fuse(pins, parallelVolumes: false);
        int before = Integrations;
        Assert.Equal(BrepOf(fused), BrepOf(plate.Fuse(pins, parallelVolumes: true)));
        Assert.True(Integrations > before);

        Shape stubs = new Box(100, 40, 6).Fuse(pins);
        ShapeList<Shape> common = stubs.Intersect([new Box(100, 40, 4).AsShape()], parallelVolumes: false);
        ShapeList<Shape> commonOn = stubs.Intersect([new Box(100, 40, 4).AsShape()], parallelVolumes: true);
        Assert.Equal(common.Count, commonOn.Count);
        for (int i = 0; i < common.Count; i++)
            Assert.Equal(BrepOf(common[i]), BrepOf(commonOn[i]));
    }

    [Fact]
    public void Volume_and_the_mass_properties_are_the_same_bits_on_and_off()
    {
        Shape part = Plate() - Holes().Select(h => (Shape)h);
        double volume = part.Volume;
        var inertia = part.MatrixOfInertia;
        var moments = part.StaticMoments;

        int before = Integrations;
        Assert.Equal(Bits(volume), Bits(part.ComputeVolume(parallelVolumes: false)));
        Assert.Equal(before, Integrations);
        Assert.Equal(Bits(volume), Bits(part.ComputeVolume(parallelVolumes: true)));
        Assert.Equal(before + 1, Integrations);

        using (ParallelVolumes.Use())
        {
            Assert.Equal(Bits(volume), Bits(part.Volume));
            Assert.Equal(inertia.SelectMany(r => r).Select(Bits), part.MatrixOfInertia.SelectMany(r => r).Select(Bits));
            Assert.Equal((Bits(moments.Mx), Bits(moments.My), Bits(moments.Mz)),
                         (Bits(part.StaticMoments.Mx), Bits(part.StaticMoments.My), Bits(part.StaticMoments.Mz)));
            Assert.True(Integrations >= before + 3);
        }
    }
}
