using Kerf.Measurement;

namespace Kerf.Modeling.Tests.Errors;

using Kerf.Modeling;

/// <summary>
/// Where the kernel can't confirm an area, volume or centre to its precision (decision 2460), the layer says so as it says any
/// failure: a property throws a <see cref="ModelingException"/> with <c>measurement.unconfirmed</c>, naming the face and why,
/// and an operation whose check needs the measure fails with it. Every confirmation is made to fail on the test's thread.
/// </summary>
public class UnconfirmedMeasureTests
{
    private static void Unconfirmable(Action action)
    {
        MeasureOperations.s_confirmNothing = true;
        try
        {
            action();
        }
        finally
        {
            MeasureOperations.s_confirmNothing = false;
        }
    }

    [Fact]
    public void A_property_that_cant_be_confirmed_throws_with_the_code()
    {
        var box = new Box(10, 10, 10);
        Assert.Equal(1000, box.Volume, 1e-9);
        foreach (Action read in new Action[] { () => _ = box.Volume, () => _ = box.Area, () => _ = box.Center(CenterOf.Mass), () => _ = box.StaticMoments,
                                               () => _ = box.MatrixOfInertia, () => _ = box.PrincipalProperties })
        {
            ModelingException e = null!;
            Unconfirmable(() => e = Assert.Throws<ModelingException>(read));
            Assert.Equal(MeasureOperations.Unconfirmed, e.Error.Code);
            Assert.Contains("couldn't be confirmed to", e.Message, StringComparison.Ordinal);
        }
    }

    [Fact]
    public void A_Boolean_whose_check_cant_be_confirmed_fails_with_the_code()
    {
        var box = new Box(10, 10, 10);
        var cylinder = new Cylinder(3, 20);
        ModelingException e = null!;
        Unconfirmable(() => e = Assert.Throws<ModelingException>(() => box - cylinder));
        Assert.Equal(MeasureOperations.Unconfirmed, e.Error.Code);
    }
}
