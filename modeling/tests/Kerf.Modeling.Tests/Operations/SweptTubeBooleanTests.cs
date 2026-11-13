using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Operations;

using Kerf.Modeling;

/// <summary>
/// OCCT #1496's tubes: two annular tubes swept along a corner filleted at different radii, overlapping along their straight
/// runs. Their common was a success larger than the smaller tube (kernel decision 2429): the intersection of one's straight
/// run with the other's bend ended its line a quarter turn round their tangent circle, and its curve, 17 off both surfaces,
/// gave the result a tolerance that excused it.
/// </summary>
public class SweptTubeBooleanTests
{
    private static Part Tube(double bend) =>
        Sweep(Plane.YZ * (new Circle(12) - new Circle(9)),
              new FilletPolyline([new Point3D(0, 0, 0), new Point3D(60, 0, 0), new Point3D(60, 0, 60)], bend).AsShape()).Value;

    [Fact]
    public void The_common_and_cuts_of_tubes_bent_at_24_and_30_agree()
    {
        Part a = Tube(24), b = Tube(30);
        double common = (a & b).Volume, aLessB = (a - b).Volume, bLessA = (b - a).Volume;
        Assert.True(common < Math.Min(a.Volume, b.Volume), $"common {common}");
        Assert.Equal(a.Volume, common + aLessB, 2);
        Assert.Equal(b.Volume, common + bLessA, 2);
        Assert.Equal(18541.24, common, 2);
    }

    // Tubes bent at 24 and 24.5: each cut removed nothing, a success, as the result's vertex of 0.25 took the check's depth to
    // 2.5 in walls 3 thick (kernel decision 2433). The cuts are still wrong after the retries, so they fail, saying so
    [Fact]
    public void Cuts_of_tubes_bent_at_24_and_24_5_that_remove_nothing_fail()
    {
        Part a = Tube(24), b = Tube(24.5);
        Assert.Contains("removed nothing", Assert.Throws<ModelingException>(() => a - b).Message);
        Assert.Contains("removed nothing", Assert.Throws<ModelingException>(() => b - a).Message);
    }
}
