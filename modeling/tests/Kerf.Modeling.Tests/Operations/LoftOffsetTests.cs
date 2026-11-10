// Unit 15O: a loft from a square to a circle offset outwards failed at every distance, "Bad orientation of subshape", as
// it does in OCCT 8.0.1: at each top corner the side faces are tangent, and OCCT made a sphere of no area there. The
// kernel leaves those spheres out and sews the tubes; the result is checked against Steiner's formula for a convex body.
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests;

using Kerf.Modeling;

public class LoftOffsetTests
{
    private static Part SquareToCircle() =>
        Loft([(Kerf.Topology.Shape)(Plane.XY * new Rectangle(10, 10)), (Kerf.Topology.Shape)(Plane.XY.Offset(10) * new Circle(4))]).Value;

    [Theory]
    [InlineData(0.02)]
    [InlineData(0.5)]
    [InlineData(1.0)]
    [InlineData(5.0)]
    public void A_loft_offset_outwards_is_made_and_has_Steiners_volume(double d)
    {
        Part loft = SquareToCircle();
        var made = Offset((Kerf.Topology.Shape)loft, d);
        Assert.True(made.IsSuccess, made.Error?.Message);
        Part offset = (Part)made.Value!;
        Assert.True(offset.IsValid);
        // V + A d + M d^2 + 4/3 pi d^3, the loft being convex; M, its mean curvature integrated, from a fine mesh of it
        double expected = loft.Volume + loft.Area * d + 84.4535 * d * d + 4.0 / 3.0 * Math.PI * d * d * d;
        Assert.Equal(expected, offset.Volume, 1.0e-4 * d * d + 1.0e-6 * expected);
    }
}
