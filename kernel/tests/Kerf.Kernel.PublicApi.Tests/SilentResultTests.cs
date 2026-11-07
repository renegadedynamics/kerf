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

    // A cone of radius 5 and height 12 has an inscribed sphere of radius 10/3 (its section's area 60 over half its perimeter
    // 18). Moved inwards by d, every face of it, its apex too, makes the same cone scaled by (10/3 - d) / (10/3) about that
    // sphere's centre: of volume 100 pi (1 + 0.3 d) cubed, whichever join. Round joins failed (decision 1918), so the offset is
    // made again with mitres and says so (decision 1938).
    [Theory]
    [InlineData(5, 0, -0.1, 286.7247)]
    [InlineData(5, 0, -0.5, 192.9331)]
    [InlineData(5, 0, -1, 107.7566)]
    [InlineData(5, 0, -2, 20.1062)]
    public void A_cone_with_its_apex_offset_inwards_is_made_with_mitres_and_says_so(double radius1, double radius2, double distance, double volume)
    {
        Result<Built<Shape>> offset = Solid.Cone(radius1, radius2, 12).Value.Shape.Offset(distance);

        Assert.True(offset.IsSuccess, offset.Error?.Message);
        Assert.Equal(volume, offset.Value.Shape.IntegrateVolume(Measured), 0.001);
        Assert.Equal(volume, 100 * Math.PI * Math.Pow(1 + 0.3 * distance, 3), 0.001);
        Assert.Contains(offset.Diagnostics, d => d.Code == OffsetOperations.JoinChanged && d.Severity == Severity.Warning);
    }

    // Outwards, round joins are right as they are, by Steiner's formula: V + S d + M d^2 + 4/3 pi d^3, where S = 90 pi and
    // M is the lateral face's 12 pi plus the rim's 5 pi times its exterior angle, pi - atan(12/5). No retry, no warning.
    [Theory]
    [InlineData(0.5, 473.198)]
    [InlineData(1, 669.666)]
    [InlineData(2, 1187.454)]
    public void A_cone_offset_outwards_keeps_its_round_joins(double distance, double volume)
    {
        double m = 12 * Math.PI + 5 * Math.PI * (Math.PI - Math.Atan(12.0 / 5));
        double steiner = 100 * Math.PI + 90 * Math.PI * distance + m * distance * distance + 4.0 / 3 * Math.PI * Math.Pow(distance, 3);
        foreach (Shape cone in new[] { Solid.Cone(5, 0, 12).Value.Shape, Solid.Cone(0, 5, 12).Value.Shape })
        {
            Result<Built<Shape>> offset = cone.Offset(distance);

            Assert.True(offset.IsSuccess, offset.Error?.Message);
            Assert.Equal(steiner, offset.Value.Shape.IntegrateVolume(Measured), 0.01);
            Assert.Equal(volume, steiner, 0.01);
            Assert.DoesNotContain(offset.Diagnostics, d => d.Code == OffsetOperations.JoinChanged);
        }
    }

    // The inverted cone, apex at the origin, fails inwards with either join, and outwards with mitres: the retry can't save
    // it, so it stays a failure holding the shape (decision 1918's rule)
    [Theory]
    [InlineData(-0.1, JoinKind.Arc)]
    [InlineData(-1, JoinKind.Arc)]
    [InlineData(-1, JoinKind.Intersection)]
    [InlineData(1, JoinKind.Intersection)]
    public void An_inverted_cone_still_fails_where_neither_join_helps(double distance, JoinKind join)
    {
        Result<Built<Shape>> offset = Solid.Cone(0, 5, 12).Value.Shape.Offset(distance, new OffsetOptions { Join = join });

        Assert.False(offset.IsSuccess);
        Assert.Equal("result.invalid", offset.Error!.Code.Value);
        Assert.True(offset.TryGetInvalid(out _));
    }

    // Hollowed through its base, inwards by t, the cone's cavity is a cone with its apex lowered by t / sin(half-angle) =
    // 13 t / 5: round joins failed in the engine; with mitres it is 100 pi - pi (5/12)^2 h^3 / 3, h = 12 - 13 t / 5
    [Theory]
    [InlineData(-0.5, 91.4402)]
    [InlineData(-1, 163.1548)]
    public void A_cone_hollowed_inwards_is_made_with_mitres_and_says_so(double thickness, double volume)
    {
        Solid cone = Solid.Cone(5, 0, 12).Value.Shape;
        Face bottom = cone.AllFaces().First(f => f.TryGetSurface(out Surface? s) && s is PlaneSurface);

        Result<Built<Solid>> hollow = cone.Hollow([bottom], thickness);

        double h = 12 + 13 * thickness / 5;
        Assert.True(hollow.IsSuccess, hollow.Error?.Message);
        Assert.Equal(volume, 100 * Math.PI - Math.PI * 25.0 / 144 * h * h * h / 3, 0.001);
        Assert.Equal(volume, hollow.Value.Shape.AsShape().IntegrateVolume(Measured), 0.001);
        Assert.Contains(hollow.Diagnostics, d => d.Code == OffsetOperations.JoinChanged);
    }

    // The inverted cone hollowed inwards with mitres gave a "valid" solid holding all of its input: now a failure
    // (decision 1939)
    [Fact]
    public void A_hollow_that_takes_nothing_out_fails()
    {
        Solid cone = Solid.Cone(0, 5, 12).Value.Shape;
        Face top = cone.AllFaces().First(f => f.TryGetSurface(out Surface? s) && s is PlaneSurface);

        Result<Built<Solid>> hollow = cone.Hollow([top], -1, new OffsetOptions { Join = JoinKind.Intersection });

        Assert.False(hollow.IsSuccess);
        Assert.Equal("result.invalid", hollow.Error!.Code.Value);
    }

    // ThickSolid, which Kerf.Modeling's Offset3D and Hollow call, had neither check nor retry: with no openings, the cone inwards
    // with round joins was a success holding a compound with no solid in it
    [Theory]
    [InlineData(-0.5, 192.9331)]
    [InlineData(-1, 107.7566)]
    public void A_thick_solid_of_a_cone_inwards_is_made_with_mitres_and_says_so(double thickness, double volume)
    {
        Result<Built<Shape>> thick = Solid.Cone(5, 0, 12).Value.Shape.ThickSolid([], thickness);

        Assert.True(thick.IsSuccess, thick.Error?.Message);
        Assert.Equal(volume, thick.Value.Shape.IntegrateVolume(Measured), 0.001);
        Assert.Contains(thick.Diagnostics, d => d.Code == OffsetOperations.JoinChanged);
    }

    // Outwards with no openings OCCT's thick solid is the offset turned inside out, so its volume is negative, and right
    [Fact]
    public void A_thick_solid_outwards_is_checked_the_right_way_out()
    {
        Result<Built<Shape>> thick = Solid.Cone(5, 0, 12).Value.Shape.ThickSolid([], 1);

        Assert.True(thick.IsSuccess, thick.Error?.Message);
        Assert.Equal(-669.666, thick.Value.Shape.IntegrateVolume(Measured), 0.01);
        Assert.DoesNotContain(thick.Diagnostics, d => d.Code == OffsetOperations.JoinChanged);
    }

    [Fact]
    public void A_thick_solid_of_the_inverted_cone_inwards_fails()
    {
        Solid cone = Solid.Cone(0, 5, 12).Value.Shape;
        Face top = cone.AllFaces().First(f => f.TryGetSurface(out Surface? s) && s is PlaneSurface);

        foreach (Result<Built<Shape>> thick in new[] { cone.ThickSolid([], -1), cone.ThickSolid([top], -1) })
        {
            Assert.False(thick.IsSuccess);
            Assert.Equal("result.invalid", thick.Error!.Code.Value);
            Assert.True(thick.TryGetInvalid(out _));
        }
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
