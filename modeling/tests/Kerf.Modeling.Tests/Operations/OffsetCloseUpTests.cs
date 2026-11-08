// Unit 12S: a 2D offset inwards that closes up or splits at a waist, through Offset as Bento's CAD_SK_OFFSET calls it. It
// threw an InvalidOperationException ("Unexpected result type") out of an operation that returns a result, a face that
// closed up was an empty success, and a hole whose offset failed for any reason was left out as if it had closed up.
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests;

using Kerf.Modeling;

public class OffsetCloseUpTests
{
    private static Point3D P(double x, double y) => new(x, y, 0);

    [Theory]
    [InlineData(-5.0)]
    [InlineData(-6.0)]
    [InlineData(-50.0)]
    public void A_circle_offset_inwards_past_its_radius_is_a_failure_not_an_exception(double amount)
    {
        var made = Offset(new Circle(5), amount);
        Assert.False(made.IsSuccess);
        Assert.Equal(ModelingDiagnostics.OffsetFailed, made.Error!.Code);
        Assert.Contains("leaves nothing", made.Error.Message);
    }

    [Fact]
    public void A_rectangle_offset_inwards_past_half_its_width_is_a_failure_with_either_join()
    {
        foreach (var kind in new[] { Kind.Arc, Kind.Intersection })
        {
            var made = Offset(new Rectangle(20, 10), -6, new OffsetOptions { Kind = kind });
            Assert.False(made.IsSuccess);
            Assert.Equal(ModelingDiagnostics.OffsetFailed, made.Error!.Code);
        }
    }

    [Fact]
    public void A_closed_curve_that_closes_up_is_a_failure()
    {
        var square = new Polyline([P(-5, -5), P(5, -5), P(5, 5), P(-5, 5)], close: true);
        var made = Offset(square, -7);
        Assert.False(made.IsSuccess);
        Assert.Contains("leaves nothing", made.Error!.Message);
    }

    [Fact]
    public void A_washer_offset_inwards_past_half_its_width_is_a_failure_not_an_empty_sketch()
    {
        var washer = new Circle(10) - new Circle(5);
        Assert.Equal(Math.PI * (9 * 9 - 6 * 6), Offset(washer, -1).Value.Area, 6);
        var made = Offset(washer, -3);
        Assert.False(made.IsSuccess);
        Assert.Contains("leaves nothing", made.Error!.Message);
    }

    [Fact]
    public void A_dumbbell_offset_inwards_past_its_waist_is_the_two_pieces()
    {
        Sketch dumbbell = new Circle(5).Moved(new Location(P(-10, 0))) + new Circle(5).Moved(new Location(P(10, 0))) + new Rectangle(20, 2);
        var made = Offset(dumbbell, -1.5);
        Assert.True(made.IsSuccess, made.Error?.Message);
        var faces = made.Value.Faces();
        Assert.Equal(2, faces.Count);
        Assert.True(made.Value.IsValid);
        // two discs of radius 3.5, slightly more where the waist met them (an independent grid estimate gives 77.21)
        Assert.InRange(made.Value.Area, 2 * Math.PI * 3.5 * 3.5, 77.4);
        foreach (var face in faces)
            Assert.Equal(7, face.Bounds().Size.Y, 3);
    }

    [Fact]
    public void A_face_of_several_that_closes_up_is_left_out_with_a_warning()
    {
        Sketch two = new Circle(5) + new Circle(0.5).Moved(new Location(P(20, 0)));
        var made = Offset(two, -1);
        Assert.True(made.IsSuccess, made.Error?.Message);
        Assert.Single(made.Value.Faces());
        Assert.Equal(Math.PI * 16, made.Value.Area, 6);
        Assert.Contains(made.Diagnostics, d => d.Code == ModelingDiagnostics.OffsetFailed && d.Message.Contains("1 of the 2 faces"));
    }

    [Fact]
    public void An_offset_kind_other_than_arc_or_intersection_is_refused()
    {
        // Kind.Tangent has gone (decision 23603): in 2D it made nothing at a triangle's corners outwards (unit 12S)
        Assert.Equal([Kind.Arc, Kind.Intersection], Enum.GetValues<Kind>());
        var e = Assert.Throws<ArgumentOutOfRangeException>(() => new RegularPolygon(10, 3).Faces()[0].OuterWire().Offset2D(1, (Kind)2));
        Assert.Contains("Kind.Arc or Kind.Intersection", e.Message);
        Assert.True(Offset(new RegularPolygon(10, 3), 1, new OffsetOptions { Kind = Kind.Intersection }).IsSuccess);
    }

    [Fact]
    public void A_hole_that_closes_up_goes_and_one_that_grows_is_cut_away()
    {
        var plate = new Rectangle(40, 20) - new Circle(4);
        // outwards: the hole shrinks, and past its radius it closes up
        Assert.Equal(800 + 120 * 1 + Math.PI - Math.PI * 9, Offset(plate, 1).Value.Area, 6);
        Assert.Equal(800 + 120 * 5 + Math.PI * 25, Offset(plate, 5).Value.Area, 6);
        // inwards: it grows
        Assert.Equal(38 * 18 - Math.PI * 25, Offset(plate, -1).Value.Area, 6);
    }
}
