// Ported from build123d tests/test_direct_api/test_group_by.py. Omitted:
// - test_str, test_pp: pretty-printing (repr, as ToString, is ported).
namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class GroupByTests
{
    readonly ShapeGroups<Vertex, double> _v = Solid.MakeBox(1, 1, 1).Vertices().GroupBy(Axis.Z);

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void FirstLast(bool reverse)
    {
        var groups = Solid.MakeBox(1, 1, 2).Vertices().GroupBy(Axis.Z);
        IReadOnlyList<ShapeList<Vertex>> grouped = reverse ? [.. groups.Reverse()] : groups;
        Assert.Same(grouped[0], grouped.First());
        Assert.Same(grouped[^1], grouped.Last());
        Assert.Equal(reverse ? 2 : 0, grouped.First().OrderBy(Axis.X).First().Z);
        Assert.Equal(reverse ? 0 : 2, grouped.Last().OrderBy(Axis.X).Last().Z);
    }

    [Fact]
    public void FirstLastSingleGroup()
    {
        var grouped = Solid.MakeBox(1, 1, 1).Edges().GroupByValue(e => e.Length);
        Assert.Same(grouped.First(), grouped.Last());
        Assert.Equal(12, grouped.First().Count);
    }

    [Fact]
    public void FirstLastEmpty()
    {
        var grouped = ShapeList<Shape>.Empty.GroupBy(Axis.Z);
        Assert.Throws<InvalidOperationException>(() => grouped.First());
        Assert.Throws<InvalidOperationException>(() => grouped.Last());
    }

    [Fact]
    public void Repr() =>
        Assert.Equal(
            "[[Vertex(0, 0, 0), Vertex(0, 1, 0), Vertex(1, 0, 0), Vertex(1, 1, 0)], "
            + "[Vertex(0, 0, 1), Vertex(0, 1, 1), Vertex(1, 0, 1), Vertex(1, 1, 1)]]",
            _v.ToString());

    [Fact]
    public void NonRoundableProperty()
    {
        var cyl = Solid.MakeCylinder(1, 2, new Plane((0, 0, -1)));
        var grouped = cyl.Faces().GroupByValue(f => f.MatrixOfInertia);
        Assert.Equal(2, grouped.Count);
    }

    private static ShapeList<Edge> TangentLines() =>
        Edge.MakeConstrainedLines(new CenterArc((-5, 0), 2, 0, 360), new CenterArc((5, 0), 4, 0, 360));

    [Fact]
    public void Properties()
    {
        var longestLines = TangentLines().GroupByValue(e => e.Length).Last();
        Assert.Equal(2, longestLines.Count);
    }

    [Fact]
    public void Callable()
    {
        static double EdgeLength(Edge e) => e.Length;
        var longestLines = TangentLines().GroupByValue(EdgeLength).Last();
        Assert.Equal(2, longestLines.Count);
    }
}
