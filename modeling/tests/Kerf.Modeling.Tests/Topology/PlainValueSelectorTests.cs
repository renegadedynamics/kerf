using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>
/// Every selector takes plain values, as a spreadsheet formula passes them: a geometry type, an axis, a
/// <see cref="ShapeKey"/>, a number. Each gives what the lambda form gives. (Until unit 10M2 they were also taken by
/// name, <c>FilterBy("IsClosed")</c>; these are the same tests on the typed forms that replaced the names.)
/// </summary>
public class PlainValueSelectorTests
{
    // A plate with a hole, filleted around the top: planes, a cylinder, tori and circles of several radii
    private static readonly Shape Part =
        Ops.Fillet((new Box(20, 10, 4) - new Cylinder(2, 10)).Edges().GroupBy(Axis.Z).Last().Where(GeomType.Line), 1);

    [Theory]
    [InlineData(GeomType.Line)]
    [InlineData(GeomType.Circle)]
    [InlineData(GeomType.Ellipse)]
    public void FilterByAGeomType(GeomType type)
    {
        var edges = Part.Edges();
        Assert.Equal(edges.Where(e => e.GeomType == type).ToShapeList(), edges.Where(type));
        Assert.Equal(edges.Where(e => e.GeomType != type).ToShapeList(), edges.WhereNot(type));
        Assert.Equal(edges.Count, edges.Where(type).Count + edges.WhereNot(type).Count);
    }

    [Fact]
    public void AMeasureAShapeLacksSaysSo()
    {
        var e = Assert.Throws<InvalidOperationException>(() => Part.Vertices().OrderBy(ShapeKey.Length));
        Assert.Contains("has no length", e.Message);
    }

    [Fact]
    public void AnUnknownKeyIsRefused() =>
        Assert.Throws<ArgumentOutOfRangeException>(() => Part.Faces().OrderBy((ShapeKey)99));

    [Fact]
    public void FilterByValueOfAnEnum()
    {
        var edges = Part.Edges();
        var circles = edges.Where(GeomType.Circle);
        Assert.Equal(circles, edges.WithValue(o => o.GeomType, GeomType.Circle));
        Assert.Equal(edges.WithValue(o => o.GeomType, GeomType.Line), edges.Where(GeomType.Line));
        Assert.Equal(edges.WithoutValue(o => o.GeomType, GeomType.Circle), edges.WhereNot(GeomType.Circle));
        // A number, within the tolerance
        var radius2 = circles.WithValue(o => o.Radius, 2.0);
        Assert.NotEmpty(radius2);
        Assert.Equal(radius2, circles.WithValue(ShapeKey.Radius, 2));
        Assert.Equal(radius2, circles.WithValue(ShapeKey.Radius, 2.0000001, tolerance: 1e-6));
    }

    [Fact]
    public void AMissingMeasureNeverMatches()
    {
        // A plane has no radius: never matched, so kept by WithoutValue
        var faces = Part.Faces();
        var cylinders = faces.WithValue(ShapeKey.Radius, 2.0);
        Assert.All(cylinders, f => Assert.Equal(GeomType.Cylinder, f.GeomType));
        Assert.Contains(faces.WithoutValue(ShapeKey.Radius, 2.0), f => f.GeomType == GeomType.Plane);
    }

    [Fact]
    public void FilterByValueOfAMeasure()
    {
        var edges = Part.Edges();
        Assert.Equal(edges.WithValue(o => o.Length, 20.0), edges.WithValue(ShapeKey.Length, 20));
        var faces = Part.Faces();
        Assert.Equal(faces.WithValue(o => o.Radius, 2.0), faces.WithValue(ShapeKey.Radius, 2));
        Assert.Equal(faces.WithValue(o => o.Area, faces[0].Area), faces.WithValue(ShapeKey.Area, faces[0].Area));
    }

    [Fact]
    public void SortByAMeasure()
    {
        var edges = Part.Edges();
        Assert.Equal(edges.OrderBy(e => e.Length).ToList(), edges.OrderBy(ShapeKey.Length).ToList());
        Assert.Equal(edges.OrderByDescending(e => e.Length).ToList(), edges.OrderByDescending(ShapeKey.Length).ToList());
        var faces = Part.Faces();
        Assert.Equal(faces.OrderBy(f => f.Area).ToList(), faces.OrderBy(ShapeKey.Area).ToList());
    }

    [Fact]
    public void GroupByAMeasure()
    {
        var faces = Part.Faces();
        var byLambda = faces.GroupByValue(f => f.Area);
        var byKey = faces.GroupBy(ShapeKey.Area);
        Assert.Equal(byLambda.Count, byKey.Count);
        for (int i = 0; i < byLambda.Count; i++)
            Assert.Equal(byLambda[i].ToList(), byKey[i].ToList());
        Assert.Equal(byLambda.Keys, byKey.Keys);
        // Not a number: grouped as it is
        var byType = faces.GroupByValue(f => f.GeomType);
        Assert.Equal(faces.Select(f => f.GeomType).Distinct().Count(), byType.Count);
        Assert.Equal(faces.Where(GeomType.Plane).ToList(), byType.Group(GeomType.Plane).ToList());
    }

    [Fact]
    public void AMixedListAnswersPerClass()
    {
        // A mixed list: each shape's own class answers
        var mixed = new ShapeList<Shape>([.. Part.Faces().Take(2), .. Part.Edges().Take(2)]);
        Assert.Equal(4, mixed.OrderBy(ShapeKey.Area).Count);
        Assert.Equal(mixed.Where(s => s.GeomType == GeomType.Line).ToList(), mixed.WithValue(s => s.GeomType, GeomType.Line).ToList());
    }

    [Fact]
    public void ManyThreadsSelectAtOnce()
    {
        var edges = Part.Edges();
        var expected = edges.OrderBy(e => e.Length).ToList();
        Parallel.For(0, 32, _ => Assert.Equal(expected, edges.OrderBy(ShapeKey.Length).ToList()));
    }
}
