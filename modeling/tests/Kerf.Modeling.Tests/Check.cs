using Xunit.Sdk;

namespace Build123d.Tests;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

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

    /// <summary>The kernel's shape held is a <typeparamref name="T"/>: where a build123d test pins the class of a result. An empty shape counts as any type, as it holds nothing.</summary>
    public static T Is<T>(IShape shape) where T : struct, IShape<T>
    {
        var held = shape.AsShape();
        if (!shape.IsEmpty)
            Assert.Equal(T.Type, held.Type);
        else if (T.Type == ShapeType.Compound)
            return default;
        return shape.IsEmpty ? default : held.As<T>();
    }

    /// <summary>The kernel's shape held is an edge or wire.</summary>
    public static void IsShape1D(IShape shape) => Assert.Equal(1, shape.IsEmpty ? 1 : shape.AsShape().Type is ShapeType.Edge or ShapeType.Wire ? 1 : 0);

    /// <summary>The kernel's shape held is a face or shell.</summary>
    public static void IsShape2D(IShape shape) => Assert.Equal(2, shape.IsEmpty ? 2 : shape.AsShape().Type is ShapeType.Face or ShapeType.Shell ? 2 : 0);

    /// <summary>The kernel's shape held is a solid or compound.</summary>
    public static void IsShape3D(IShape shape) => Assert.Equal(3, shape.IsEmpty ? 3 : shape.AsShape().Type is ShapeType.Solid or ShapeType.Compound or ShapeType.CompSolid ? 3 : 0);

    /// <summary>It is a shape, or holds one.</summary>
    public static void IsShape(IShape shape) => Assert.NotNull(shape);

    /// <summary>The same shape: the same data at the same place, either orientation, as the layer's shapes compared before unit 10Q.</summary>
    public static void Same<TA, TB>(TA expected, TB actual) where TA : IShape where TB : IShape =>
        Assert.True(expected.SameAs(actual), $"Expected {expected.AsShape().Describe()} but got {actual.AsShape().Describe()}");

    /// <summary>Not the same shape.</summary>
    public static void NotSame<TA, TB>(TA expected, TB actual) where TA : IShape where TB : IShape =>
        Assert.False(expected.SameAs(actual), $"Both are {actual.AsShape().Describe()}");

    public static double[] Of(Vector v) => [v.X, v.Y, v.Z];

    public static double[] Of(Color c) => [c.R, c.G, c.B, c.A];
}
