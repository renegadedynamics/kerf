// Unit 12T: a 2D offset inwards past a curve's sharpest bend, where the kernel stands a polyline in for the curve
// (CheckBadEdges). OCCT's deflection of 0.01 of the distance put the offset up to 1% of it too far in, silently; 0.001
// keeps it within 0.1% (decision 23601). And a closed curve whose offset came out open is a failure, not an exception.
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests;

using Kerf.Modeling;

public class OffsetAccuracyTests
{
    private static Point3D P(double x, double y) => new(x, y, 0);

    // The distance from a point to the ellipse x = 10 cos t, y = 5 sin t, by dense sampling
    private static double ToEllipse(Point3D q)
    {
        double best = double.MaxValue;
        for (int j = 0; j < 20000; j++)
        {
            double t = 2 * Math.PI * j / 20000;
            double dx = 10 * Math.Cos(t) - q.X, dy = 5 * Math.Sin(t) - q.Y;
            best = Math.Min(best, dx * dx + dy * dy);
        }
        return Math.Sqrt(best);
    }

    [Theory]
    [InlineData(2.5)]
    [InlineData(3.0)]
    [InlineData(4.5)]
    public void An_ellipse_offset_inwards_past_its_sharpest_bend_is_within_a_thousandth_of_the_distance(double d)
    {
        // the ellipse's sharpest radius is 2.5: from there on the kernel offsets a polyline standing in for it
        var made = Offset(new Ellipse(10, 5), -d);
        Assert.True(made.IsSuccess, made.Error?.Message);
        double worst = 0;
        foreach (var e in made.Value.Edges())
            for (int i = 0; i <= 10; i++)
                worst = Math.Max(worst, Math.Abs(ToEllipse(e.PositionAt(i / 10.0)) - d));
        // the polyline's chords lie inside the curve: 1% of d with OCCT's deflection (0.029 at d = 3)
        Assert.InRange(worst, 0, 0.001 * d + 1e-6);
    }

    [Fact]
    public void An_ellipse_offset_inwards_by_its_sharpest_radius_has_nearly_the_area_of_its_inner_parallel()
    {
        // Steiner's formula holds up to the sharpest radius: A - P d + pi d^2
        var ellipse = new Ellipse(10, 5);
        double p = ellipse.Edges().Sum(e => e.Length);
        double exact = ellipse.Area - p * 2.5 + Math.PI * 2.5 * 2.5;
        var made = Offset(ellipse, -2.5);
        Assert.True(made.IsSuccess, made.Error?.Message);
        // 55.26 of 55.61 with OCCT's deflection; now 55.57
        Assert.InRange(made.Value.Area, exact * (1 - 0.001), exact);
    }

    [Fact]
    public void A_closed_curve_whose_offset_comes_out_open_is_a_failure_not_an_exception()
    {
        // At -9, where this blob closes up, the finer polyline gave an open wire flagged as closed, on which Face.Make threw;
        // the kernel now falls back to OCCT's polyline there, and Offset2D fails such a wire if one still comes
        var blob = MakeFace(new Spline([P(0, 0), P(10, -3), P(20, 2), P(15, 12), P(4, 10)], periodic: true));
        var made = Offset(blob, -9);
        Assert.False(made.IsSuccess);
        Assert.Equal(ModelingDiagnostics.OffsetFailed, made.Error!.Code);
    }
}
