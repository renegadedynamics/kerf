using Kerf.Measurement;
using Kerf.Offsets;
using Kerf.Topology;

namespace Kerf.Modeling.Tests.Topology;

using Kerf.Modeling;

/// <summary>
/// The kernel's check of an offset's size and its retry with mitres (decisions 1918 and 1938) through the modelling layer's
/// <c>Offset3D</c> and <c>Hollow</c>, which go through the kernel's thick solid, as Bento does.
/// </summary>
public class OffsetCheckTests
{
    private static readonly MassOptions Measured = new() { Tolerance = 1e-6 };

    // A cone of radius 5 and height 12 has an inscribed sphere of radius 10/3. Moved inwards by d, every face of it makes the
    // same cone scaled about that sphere's centre by (10/3 - d) / (10/3): of volume 100 pi (1 + 0.3 d) cubed. Round joins gave
    // a compound with no solid in it, which went through as a shape of no volume; now it's made with mitres, and says so.
    [Theory]
    [InlineData(-0.1, 286.7247)]
    [InlineData(-0.5, 192.9331)]
    [InlineData(-1, 107.7566)]
    [InlineData(-2, 20.1062)]
    public void A_cone_offset_inwards_is_made_with_mitres_and_says_so(double distance, double volume)
    {
        var cone = Solid.MakeCone(5, 0, 12);

        using var warnings = Warnings.Capture();
        var offset = cone.Offset3D(null, distance);

        Assert.Equal(volume, 100 * Math.PI * Math.Pow(1 + 0.3 * distance, 3), 0.001);
        Assert.Single(offset.Solids());
        Assert.Equal(volume, offset.AsShape().IntegrateVolume(Measured), 0.001);
        Assert.Contains(warnings.Messages, m => m.StartsWith("Made with mitred joins", StringComparison.Ordinal));
    }

    // Operations.Offset, Bento's way in, carries the kernel's warning with its code
    [Fact]
    public void The_operation_carries_the_warning()
    {
        var offset = Operations.Offset((Shape)Solid.MakeCone(5, 0, 12), -1);

        Assert.True(offset.IsSuccess, offset.ToString());
        Assert.Equal(107.7566, offset.Value.AsShape().IntegrateVolume(Measured), 0.001);
        var warning = Assert.Single(offset.Diagnostics);
        Assert.Equal(OffsetOperations.JoinChanged, warning.Code);
    }

    // Hollowed through its base, inwards by t, the cone's cavity is a cone with its apex lowered by 13 t / 5: round joins threw;
    // with mitres it is 100 pi - pi (5/12)^2 h^3 / 3, h = 12 - 13 t / 5
    [Theory]
    [InlineData(-0.5, 91.4402)]
    [InlineData(-1, 163.1548)]
    public void A_cone_hollowed_inwards_is_made_with_mitres(double thickness, double volume)
    {
        var cone = Solid.MakeCone(5, 0, 12);
        double h = 12 + 13 * thickness / 5;
        Assert.Equal(volume, 100 * Math.PI - Math.PI * 25.0 / 144 * h * h * h / 3, 0.001);

        foreach (var made in new[] { cone.Hollow(cone.Faces().Where(GeomType.Plane), thickness),
                                     cone.Offset3D(cone.Faces().Where(GeomType.Plane), thickness) })
            Assert.Equal(volume, made.AsShape().IntegrateVolume(Measured), 0.001);
    }

    // The inverted cone fails inwards with either join: before, a shape of no volume, or one of -2e100, called a success
    [Theory]
    [InlineData(-1, Kind.Arc)]
    [InlineData(-1, Kind.Intersection)]
    [InlineData(-0.1, Kind.Arc)]
    public void An_inverted_cone_offset_inwards_fails_holding_the_shape(double distance, Kind kind)
    {
        var cone = Solid.MakeCone(0, 5, 12);
        var top = cone.Faces().Where(GeomType.Plane);

        foreach (Func<Shape> offset in new Func<Shape>[] { () => cone.Offset3D(null, distance, kind: kind), () => cone.Hollow(top, distance, kind: kind) })
        {
            var failure = Assert.Throws<ModelingException>(() => offset());
            Assert.StartsWith("The thick solid is wrong", failure.Message, StringComparison.Ordinal);
            Assert.True(ModelingDiagnostics.TryGetInvalid(failure, out _));
        }
    }

    // Hollowed outwards with mitres, the inverted cone's thick solid isn't valid, and ShapeFix made it the cone itself, 314.159,
    // with no wall: the repair is now checked as the kernel checks what it makes, and fails, so the kernel's failure stands
    [Theory]
    [InlineData(0.5)]
    [InlineData(1)]
    public void A_repair_that_fills_the_wall_fails(double thickness)
    {
        var cone = Solid.MakeCone(0, 5, 12);
        var top = cone.Faces().Where(GeomType.Plane);

        Assert.Throws<ModelingException>(() => cone.Hollow(top, thickness, kind: Kind.Intersection));
        Assert.Throws<ModelingException>(() => cone.Offset3D(top, thickness, kind: Kind.Intersection));
    }

    // Outwards, round joins are right as they are, by Steiner's formula, for both cones and both ways of asking: no warning
    [Theory]
    [InlineData(0.5, 473.198)]
    [InlineData(1, 669.666)]
    [InlineData(2, 1187.454)]
    public void A_cone_offset_outwards_keeps_its_round_joins(double distance, double volume)
    {
        foreach (var cone in new[] { Solid.MakeCone(5, 0, 12), Solid.MakeCone(0, 5, 12) })
        {
            using var warnings = Warnings.Capture();
            var offset = cone.Offset3D(null, distance);

            Assert.Equal(volume, offset.AsShape().IntegrateVolume(Measured), 0.01);
            Assert.Equal(volume - 100 * Math.PI, cone.Hollow([], distance).AsShape().IntegrateVolume(Measured), 0.01);
            Assert.Empty(warnings.Messages);
        }
    }
}
