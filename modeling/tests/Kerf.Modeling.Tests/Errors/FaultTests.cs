namespace Build123d.Tests.Errors;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>OCCT's calls leave the thread's floating point as it was: no traps.</summary>
public class FaultTests
{
    [Fact]
    public void FloatingPointStaysUntrappedAfterOcctCalls()
    {
        _ = (new Box(1, 1, 1) - new Sphere(0.7)).Volume;
        double zero = 0, huge = double.MaxValue;
        Assert.Equal(double.PositiveInfinity, huge * 10);
        Assert.Equal(double.NegativeInfinity, -1 / zero);
    }
}
