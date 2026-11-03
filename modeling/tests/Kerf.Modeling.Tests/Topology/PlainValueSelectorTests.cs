using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>
/// Every selector takes plain values, as a spreadsheet formula passes them: build123d's <c>property</c> arguments by
/// name, and enums by name where a value is compared. Each gives what the lambda overload gives.
/// </summary>
public class PlainValueSelectorTests
{
    // A plate with a hole, filleted around the top: planes, a cylinder, tori and circles of several radii
    private static readonly Shape Part =
        Ops.Fillet((new Box(20, 10, 4) - new Cylinder(2, 10)).Edges().GroupBy(Axis.Z).Last.FilterBy(GeomType.Line), 1);

    [Theory]
    [InlineData("IsClosed")]
    [InlineData("isClosed")]
    [InlineData("is_closed")]
    public void FilterByABooleanPropertyByName(string name)
    {
        var edges = Part.Edges();
        Assert.Equal(edges.FilterBy(e => e.IsClosed), edges.FilterBy(name));
        Assert.Equal(edges.FilterBy(e => e.IsClosed, reverse: true), edges.FilterBy(name, reverse: true));
        Assert.NotEmpty(edges.FilterBy(name));
    }

    [Fact]
    public void FilterByAPropertyThatIsntBooleanSaysSo()
    {
        var e = Assert.Throws<ArgumentException>(() => Part.Edges().FilterBy("Length"));
        Assert.Contains("FilterByValue", e.Message);
    }

    [Fact]
    public void AnUnknownNameListsTheKnownOnes()
    {
        var e = Assert.Throws<ArgumentException>(() => Part.Faces().SortBy("Lenght"));
        Assert.Contains("no property 'Lenght'", e.Message);
        Assert.Contains("Area", e.Message);
        Assert.Contains("Center()", e.Message);
    }

    [Fact]
    public void FilterByValueByName()
    {
        var edges = Part.Edges();
        var circles = edges.FilterBy(GeomType.Circle);
        Assert.Equal(edges.FilterByValue(o => o.GeomType, GeomType.Circle), edges.FilterByValue("GeomType", GeomType.Circle));
        // An enum by its name, as build123d spells it too
        Assert.Equal(circles, edges.FilterByValue("GeomType", "Circle"));
        Assert.Equal(circles, edges.FilterByValue("geom_type", "CIRCLE"));
        // A number, within the tolerance, whatever its type
        var radius2 = circles.FilterByValue(o => o.Radius, 2.0);
        Assert.NotEmpty(radius2);
        Assert.Equal(radius2, circles.FilterByValue("Radius", 2));
        Assert.Equal(radius2, circles.FilterByValue("radius", 2.0000001, tolerance: 1e-6));
    }

    [Fact]
    public void AMissingMeasureNeverMatches()
    {
        // A plane has no radius: never matched, so kept by reverse
        var faces = Part.Faces();
        var cylinders = faces.FilterByValue("Radius", 2.0);
        Assert.All(cylinders, f => Assert.Equal(GeomType.Cylinder, f.GeomType));
        Assert.Contains(faces.FilterByValue("Radius", 2.0, reverse: true), f => f.GeomType == GeomType.Plane);
    }

    [Fact]
    public void FilterByValueOfAMeasure()
    {
        var edges = Part.Edges();
        Assert.Equal(edges.FilterByValue(o => o.Length, 20.0), edges.FilterByValue(SortBy.Length, 20));
        var faces = Part.Faces();
        Assert.Equal(faces.FilterByValue(o => o.Radius, 2.0), faces.FilterByValue(SortBy.Radius, 2));
        Assert.Equal(faces.FilterByValue(o => o.Area, faces[0].Area), faces.FilterByValue(SortBy.Area, faces[0].Area));
    }

    [Fact]
    public void SortByName()
    {
        var edges = Part.Edges();
        Assert.Equal(edges.SortBy(e => e.Length).ToList(), edges.SortBy("Length").ToList());
        Assert.Equal(edges.SortBy(e => e.Length, reverse: true).ToList(), edges.SortBy("length", reverse: true).ToList());
        // A method that takes no arguments
        var faces = Part.Faces();
        Assert.Equal(faces.SortBy(f => f.Area).ToList(), faces.SortBy("Area").ToList());
    }

    [Fact]
    public void GroupByName()
    {
        var faces = Part.Faces();
        var byLambda = faces.GroupBy(f => f.Area);
        var byName = faces.GroupBy("Area");
        Assert.Equal(byLambda.Count, byName.Count);
        for (int i = 0; i < byLambda.Count; i++)
            Assert.Equal(byLambda[i].ToList(), byName[i].ToList());
        Assert.Equal(byLambda.Keys.Cast<object?>(), byName.Keys);
        // Not a number: grouped as it is
        var byType = faces.GroupBy("GeomType");
        Assert.Equal(faces.GroupBy(f => f.GeomType).Count, byType.Count);
        Assert.Equal(faces.FilterBy(GeomType.Plane).ToList(), byType.Group(GeomType.Plane).ToList());
    }

    [Fact]
    public void NamesResolvePerClass()
    {
        // A mixed list: each shape's own class answers
        var mixed = new ShapeList<Shape>([.. Part.Faces().Take(2), .. Part.Edges().Take(2)]);
        Assert.Equal(4, mixed.SortBy("Area").Count);
        Assert.Equal(mixed.Where(s => s.GeomType == GeomType.Line).ToList(), mixed.FilterByValue("GeomType", "Line").ToList());
    }

    [Fact]
    public void ManyThreadsResolveNamesAtOnce()
    {
        var edges = Part.Edges();
        var expected = edges.SortBy(e => e.Length).ToList();
        Parallel.For(0, 32, _ => Assert.Equal(expected, edges.SortBy("Length").ToList()));
    }
}
