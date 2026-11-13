// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Exchange;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests.PublicApi;

/// <summary>
/// Placements with a scale where they enter from data (decision 2452): the kernel's operations don't hold under one (a
/// solid classified a point inside it OUT, distances came out up to 20% off), so a scale is made part of the geometry
/// where the data places a whole shape with it, and refused where it can't be.
/// </summary>
public class ScaledPlacementTests
{
    private static readonly Transform3D Scale = Transform3D.Scale(new Point3D(2, -3, 1), 1.7);

    private static Shape Box() => Primitives.Box(3, 4, 5).Value.Shape;

    // The box, the scale as part of its geometry: valid, its volume times the scale's cube, a point by its corner inside
    private static void AssertScaledBox(Shape shape)
    {
        Assert.True(shape.Validate().IsValid);
        Assert.All(shape.AllFaces(), f => Assert.Equal(1.0, f.Placement.Transformation.ScaleFactor));
        Assert.Equal(60 * 1.7 * 1.7 * 1.7, shape.IntegrateVolume(), 6);
        Assert.Equal(Containment.In, shape.ClassifyPoint(Scale * new Point3D(2.9, 3.9, 4.9)));
        Assert.Equal(Containment.Out, shape.ClassifyPoint(Scale * new Point3D(3.1, 3.9, 4.9)));
    }

    [Fact]
    public void An_assembly_placed_with_a_scale_gives_its_parts_the_scale_in_their_geometry()
    {
        ModelNode model = ModelNode.Assembly([ModelNode.Part(Box(), "box")], "scaled", new Placement(Scale));
        AssertScaledBox(model.ToShape());
    }

    [Fact]
    public void A_brep_whose_shape_is_placed_with_a_scale_reads_with_the_scale_in_its_geometry()
    {
        Shape scaled = Box();
        scaled.Move(new Placement(Scale));
        Shape read = Brep.Read(new MemoryStream(Written(scaled))).Value;
        AssertScaledBox(read);
    }

    [Fact]
    public void A_brep_with_a_shape_inside_it_placed_with_a_scale_is_refused()
    {
        Shape scaled = Box();
        scaled.Move(new Placement(Scale));
        Result<Shape> read = Brep.Read(new MemoryStream(Written(Compound.Create([scaled, Box()]))));
        Assert.False(read.IsSuccess);
        Assert.Contains("with a scale", read.Error!.Message, StringComparison.Ordinal);
    }

    private static byte[] Written(Shape shape)
    {
        var stream = new MemoryStream();
        Assert.True(Brep.Write(shape, stream).IsSuccess);
        return stream.ToArray();
    }
}
