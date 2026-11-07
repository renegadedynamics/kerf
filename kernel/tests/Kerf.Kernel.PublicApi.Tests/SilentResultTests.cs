// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Offsets;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Results every check called valid and that were wrong, which the corpus's BAD cases led to (decisions 1918 and 1919).</summary>
public class SilentResultTests
{
    private static readonly MassOptions Measured = new() { Tolerance = 1e-6 };

    [Theory]
    [InlineData(-0.1)]
    [InlineData(-1)]
    [InlineData(-2)]
    public void A_cone_with_its_apex_offset_inwards_fails_and_holds_what_it_made(double distance)
    {
        Result<Built<Shape>> offset = Solid.Cone(5, 0, 12).Value.Shape.Offset(distance);

        Assert.False(offset.IsSuccess);
        Assert.Equal("result.invalid", offset.Error!.Code.Value);
        Assert.True(offset.TryGetInvalid(out _));
    }

    [Theory]
    [InlineData(-1, 107.757)] // by similar triangles: the cone's inscribed sphere has a radius of 10/3
    [InlineData(-2, 20.106)]
    public void The_same_cone_with_mitres_is_right(double distance, double volume)
    {
        Shape result = Solid.Cone(5, 0, 12).Value.Shape.Offset(distance, new OffsetOptions { Join = JoinKind.Intersection }).Value.Shape;

        Assert.Equal(volume, result.IntegrateVolume(Measured), 2);
    }

    [Theory]
    [InlineData(-1)]
    [InlineData(-0.1)]
    [InlineData(0.1)]
    [InlineData(1)]
    public void Offsets_of_other_shapes_pass(double distance)
    {
        Shape[] shapes =
        [
            Solid.Box(10, 12, 7).Value.Shape,
            Solid.Cylinder(5, 12).Value.Shape,
            Solid.Sphere(6).Value.Shape,
            Solid.Torus(9, 3).Value.Shape,
            Solid.Cone(5, 2, 12).Value.Shape,
        ];
        foreach (Shape shape in shapes)
        {
            Result<Built<Shape>> offset = shape.Offset(distance);
            Assert.True(offset.IsSuccess, offset.Error?.Message);
            double before = shape.IntegrateVolume(Measured), after = offset.Value.Shape.IntegrateVolume(Measured);
            Assert.True(distance < 0 ? after < before : after > before);
        }
    }

    [Fact]
    public void A_face_with_bounds_off_its_surface_fails()
    {
        var curve = new BezierCurve3D([new Point3D(0, 0, 0), new Point3D(2, 2, 2), new Point3D(4, 5, 2)]);
        var surface = new ExtrusionSurface(curve, Direction3D.UnitZ);

        Result<Face> off = Face.FromSurface(surface, (new Interval(0, 3), new Interval(0, 30)));
        Assert.False(off.IsSuccess);
        Assert.Equal("result.invalid", off.Error!.Code.Value);

        Assert.True(Face.FromSurface(surface, (new Interval(0, 1), new Interval(0, 30))).IsSuccess);
        Assert.True(Face.FromSurface(surface, (new Interval(0, 3), new Interval(0, 30)), checkResult: false).IsSuccess);
    }
}
