namespace Kerf.Modeling.Tests.Objects;

using Kerf.Modeling;

public class PartialTorusTests
{
    [Fact]
    public void Part_of_the_minor_circle_is_a_slice_of_the_tube()
    {
        // The outer half, by Pappus: half the section's area, its centroid at 5 + 4 / 3 pi
        double expected = 2 * Math.PI * (5 + 4 / (3 * Math.PI)) * Math.PI / 2;
        Part torus = new Torus(5, 1, minorStartAngle: -90, minorEndAngle: 90);
        Assert.Equal(expected, torus.Volume, 6);
        Assert.True(torus.IsValid);
        Assert.Equal(expected, Solid.MakeTorus(5, 1, startAngle: -90, endAngle: 90).Volume, 6);
        // And a quarter of the way round
        Assert.Equal(expected / 4, Solid.MakeTorus(5, 1, startAngle: -90, endAngle: 90, majorAngle: 90).Volume, 6);
    }

    [Fact]
    public void The_whole_tube_is_unchanged() =>
        Assert.Equal(2 * Math.PI * Math.PI * 5, new Torus(5, 1).Volume, 6);
}
