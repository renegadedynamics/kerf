using Kerf.Modeling;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Topology;

/// <summary>
/// Chamfers past tight corners through <see cref="Operations.Chamfer(IEnumerable{Edge}, double, double?, double?, Face?, ChamferOptions?)"/>
/// (the kernel's docs/chamfer-corners.md): the user's prism and a housing, against closed forms worked out here.
/// </summary>
public class ChamferTightCornersTests
{
    private static readonly ChamferOptions Collapse = new() { TightCorners = TightCorners.Collapse };

    private static Part Rounded(double w, double h, double height, double r)
    {
        var box = new Box(w, h, height);
        return Fillet(box, box.Edges().ParallelTo(Axis.Z), r).Value;
    }

    // As the kernel's tests: the hull of the outline at depth b and the contact on the face, by Simpson's rule on its sections
    private static double Removed(double w, double h, double r, double a, double b)
    {
        double loss = 4 - Math.PI;
        double outline = w * h - loss * r * r;
        double top = (w - 2 * a) * (h - 2 * a) - (a <= r ? loss * (r - a) * (r - a) : 0);
        double half = a <= r ? r - a / 2 : r / 2;
        double middle = (w - a) * (h - a) - loss * half * half;
        return b * outline - b / 6 * (outline + 4 * middle + top);
    }

    private static double Prism(double w, double h, double height, double r) => (w * h - (4 - Math.PI) * r * r) * height;

    [Theory]
    [InlineData(1.71)]
    [InlineData(2)]
    [InlineData(3)]
    public void The_users_prism_chamfered_past_its_17_corners(double length)
    {
        Part prism = Rounded(40, 30, 30, 1.7);
        var top = prism.Edges().GroupBy(Axis.Z).Last();
        Assert.Equal(8, top.Count);
        Part made = Chamfer(prism, top, length, options: Collapse).Value;
        Assert.True(made.IsValid);
        double expected = Prism(40, 30, 30, 1.7) - Removed(40, 30, 1.7, length, length);
        Assert.Equal(expected, made.Volume, 1e-6 * expected);
        Assert.Equal(18, made.Faces().Count);
    }

    [Fact]
    public void Without_the_option_it_is_refused_as_before()
    {
        Part prism = Rounded(40, 30, 30, 1.7);
        var top = prism.Edges().GroupBy(Axis.Z).Last();
        Result<Part> refused = Chamfer(prism, top, 1.71);
        Assert.False(refused.IsSuccess); // invalid, as the user found; unit 12V refuses it before building

        // And below the corners, the same with it or without
        Assert.Equal(Chamfer(prism, top, 1.69).Value.Volume, Chamfer(prism, top, 1.69, options: Collapse).Value.Volume);
        Assert.Equal(35734.872, Chamfer(prism, top, 1.69).Value.Volume, 1e-3);
    }

    [Fact]
    public void A_housing_with_15_corners_at_3()
    {
        Part housing = Rounded(80, 50, 25, 1.5) - new Box(75, 45, 22.5).Moved(new Location(new Vector3D(0, 0, -1.25)));
        var top = housing.Edges().GroupBy(Axis.Z).Last();
        Part made = Chamfer(housing, top, 3, options: Collapse).Value;
        Assert.True(made.IsValid);
        double expected = Prism(80, 50, 25, 1.5) - 75 * 45 * 22.5 - Removed(80, 50, 1.5, 3, 3);
        Assert.Equal(expected, made.Volume, 1e-6 * expected);
    }

    [Fact]
    public void Two_lengths_and_an_angle_collapse_by_the_length_on_the_face()
    {
        Part prism = Rounded(40, 30, 30, 1.7);
        var top = prism.Edges().GroupBy(Axis.Z).Last();
        Face face = prism.Faces().OrderBy(Axis.Z).Last();
        double expected = Prism(40, 30, 30, 1.7) - Removed(40, 30, 1.7, 3, 1);
        Assert.Equal(expected, Chamfer(prism, top, 3, 1, reference: face, options: Collapse).Value.Volume, 1e-6 * expected);
        Assert.Equal(expected, Chamfer(prism, top, 3, angle: Math.Atan(1.0 / 3) * 180 / Math.PI, reference: face, options: Collapse).Value.Volume, 1e-6 * expected);
    }

    [Fact]
    public void Too_narrow_says_so()
    {
        Part prism = Rounded(10, 6, 10, 1.7);
        var top = prism.Edges().GroupBy(Axis.Z).Last();
        Result<Part> refused = Chamfer(prism, top, 3.5, options: Collapse);
        Assert.False(refused.IsSuccess);
        Assert.Contains("too narrow", refused.Error!.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void The_shape_member_takes_the_option_too()
    {
        Part prism = Rounded(40, 30, 30, 1.7);
        var top = prism.Edges().GroupBy(Axis.Z).Last();
        Shape made = prism.Chamfer(2, null, top, options: Collapse);
        Assert.Equal(Prism(40, 30, 30, 1.7) - Removed(40, 30, 1.7, 2, 2), made.Volume, 1e-3);
    }
}
