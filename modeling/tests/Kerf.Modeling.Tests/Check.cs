using Xunit.Sdk;

namespace Kerf.Modeling.Tests;

/// <summary>Assertions matching the ones build123d's Python tests use.</summary>
internal static class Check
{
    /// <summary>
    /// As unittest's assertAlmostEqual(a, b, places): true when round(|a - b|, places) is zero. For vectors
    /// |a - b| is the length of their difference.
    /// </summary>
    public static void AlmostEqual(double expected, double actual, int places = 7)
    {
        if (expected == actual)
            return;
        if (Math.Round(Math.Abs(expected - actual), places) != 0)
            throw new XunitException($"Expected {expected} but got {actual} (to {places} places)");
    }

    /// <inheritdoc cref="AlmostEqual(double, double, int)"/>
    public static void AlmostEqual(Vector expected, Vector actual, int places = 7)
    {
        if (Math.Round((expected - actual).Length, places) != 0)
            throw new XunitException($"Expected {expected} but got {actual} (to {places} places)");
    }

    /// <summary>Each component almost equal, as build123d's assertTupleAlmostEquals.</summary>
    public static void TupleAlmostEqual(IReadOnlyList<double> expected, IReadOnlyList<double> actual, int places = 7)
    {
        Assert.Equal(expected.Count, actual.Count);
        for (int i = 0; i < expected.Count; i++)
            if (Math.Round(Math.Abs(expected[i] - actual[i]), places) != 0)
                throw new XunitException(
                    $"Expected ({string.Join(", ", expected)}) but got ({string.Join(", ", actual)}) (to {places} places)");
    }

    public static double[] Of(Vector v) => [v.X, v.Y, v.Z];

    public static double[] Of(Color c) => [c.R, c.G, c.B, c.A];
}
