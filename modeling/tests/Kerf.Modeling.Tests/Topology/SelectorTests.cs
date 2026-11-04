using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>Selectors written as plain values, and found for a face, edge or vertex picked in a viewer.</summary>
public class SelectorTests
{
    private static readonly Shape Part =
        Ops.Fillet((new Box(20, 10, 4) - new Cylinder(2, 10)).Edges().GroupBy(Axis.Z).Last().Where(GeomType.Line), 1);

    [Fact]
    public void ASelectorIsTheSelectionItDescribes()
    {
        var selector = new Selector(ShapeType.Face, [new Selector.OfGeomType(GeomType.Plane), new Selector.GroupByAxis(Axis.Z, -1)]);
        Assert.Equal(Part.Faces().Where(GeomType.Plane).GroupBy(Axis.Z).Last().ToList<Shape>(), selector.Apply(Part).ToList());
        Assert.Equal("Faces().Where(GeomType.Plane).GroupBy(Axis.Z).Last()", selector.ToString());
        Assert.Equal(Part.Faces().Where(GeomType.Plane).GroupBy(Axis.Z)[^1].ToList<Shape>(), selector.Apply(Part).ToList());
    }

    [Fact]
    public void GroupsAreIndexedFromEitherEnd()
    {
        var byX = Part.Faces().GroupBy(Axis.X);
        Selector Group(int index) => new(ShapeType.Face, [new Selector.GroupByAxis(Axis.X, index)]);
        Assert.Equal(byX[2].ToList<Shape>(), Group(2).Apply(Part).ToList());
        Assert.Equal(byX[byX.Count - 2].ToList<Shape>(), Group(-2).Apply(Part).ToList());
        Assert.Equal("Faces().GroupBy(Axis.X)[2]", Group(2).ToString());
        Assert.Equal("Faces().GroupBy(Axis.X)[^2]", Group(-2).ToString());
        Assert.Equal("Faces().GroupBy(Axis.X).First()", Group(0).ToString());
        // Past the groups of a changed model: nothing
        Assert.Empty(Group(99).Apply(Part));
    }

    [Fact]
    public void AnInnerHoleOfAGridIsPickedByItsPlace()
    {
        var holes = Enumerable.Range(0, 16).Select(k => (Shape)new Cylinder(1, 20).Moved(new Location((-15.0 + 10 * (k % 4), -15.0 + 10 * (k / 4), 0.0))));
        var plate = new Box(40, 40, 2).Cut(holes);
        var inner = plate.Faces().Where(GeomType.Cylinder).GroupBy(Axis.X)[1].GroupBy(Axis.Y)[2].Single();
        var found = Selector.For(plate, inner);
        Assert.NotEmpty(found);
        // Among all the faces, the sides and the top and bottom also group along X, ahead of the second column
        Assert.Equal("Faces().GroupBy(Axis.X)[2].GroupBy(Axis.Y)[^2]", found[0].ToString());
    }

    [Fact]
    public void NoSelectorHoldsASimplerOne()
    {
        var box = new Box(3, 2, 1);
        var top = box.Faces().GroupBy(Axis.Z).Last().Single();
        var found = Selector.For(box, top, count: 10);
        Assert.Single(found, s => s.Steps.Count == 1 && s.Steps[0] is Selector.GroupByAxis);
        Assert.DoesNotContain(found, s => s.Steps.Count > 1 && s.Steps[^1].Equals(new Selector.GroupByAxis(Axis.Z, -1)));
    }

    [Fact]
    public void SelectorsAreValues()
    {
        Selector Make() => new(ShapeType.Edge, [new Selector.ParallelTo(Axis.Z), new Selector.GroupByKey(ShapeKey.Length, 0)]);
        Assert.Equal(Make(), Make());
        Assert.Equal(Make().GetHashCode(), Make().GetHashCode());
        Assert.NotEqual(Make(), new Selector(ShapeType.Edge, [new Selector.ParallelTo(Axis.Z)]));
        Assert.Equal("Edges().ParallelTo(Axis.Z).GroupBy(ShapeKey.Length).First()", Make().ToString());
        Assert.Equal(Part.Edges().ParallelTo(Axis.Z).GroupBy(ShapeKey.Length).First().ToList<Shape>(), Make().Apply(Part).ToList());
        Assert.Throws<ArgumentException>(() => new Selector(ShapeType.Solid, []));
    }

    [Fact]
    public void EveryFaceOfAPartHasASelector()
    {
        var faces = Part.Faces();
        foreach (var face in faces)
        {
            var found = Selector.For(Part, face);
            Assert.NotEmpty(found);
            foreach (var selector in found)
            {
                var picked = Assert.Single(selector.Apply(Part));
                Assert.True(picked.TopoShape.IsSame(face.TopoShape), $"{selector} picks another face");
            }
        }
    }

    [Fact]
    public void EveryEdgeAndVertexOfABoxHasASelector()
    {
        var box = new Box(3, 2, 1);
        foreach (var edge in box.Edges())
            Assert.True(Assert.Single(Selector.For(box, edge, count: 1)[0].Apply(box)).TopoShape.IsSame(edge.TopoShape));
        foreach (var vertex in box.Vertices())
            Assert.True(Assert.Single(Selector.For(box, vertex, count: 1)[0].Apply(box)).TopoShape.IsSame(vertex.TopoShape));
    }

    [Fact]
    public void TheSimplestComesFirst()
    {
        var box = new Box(3, 2, 1);
        var top = box.Faces().GroupBy(Axis.Z).Last().Single();
        Assert.Equal("Faces().GroupBy(Axis.Z).Last()", Selector.For(box, top)[0].ToString());
        // A top edge along X: two steps, along the model's axes
        var edge = box.Edges().GroupBy(Axis.Z).Last().GroupBy(Axis.Y).Last().Single();
        var simplest = Selector.For(box, edge)[0];
        Assert.Equal(2, simplest.Steps.Count);
        Assert.All(simplest.Steps, s => Assert.IsType<Selector.GroupByAxis>(s));
    }

    [Fact]
    public void AHoleIsPickedByWhatSetsItApart()
    {
        // The hole spans the part in Z, among the fillets' cylinders: told apart by a measure or a place
        var hole = Part.Faces().Where(GeomType.Cylinder).WithValue(ShapeKey.Radius, 2).Single();
        var found = Selector.For(Part, hole);
        Assert.NotEmpty(found);
        Assert.All(found, s => Assert.True(Assert.Single(s.Apply(Part)).TopoShape.IsSame(hole.TopoShape)));
    }

    [Fact]
    public void ASelectorSurvivesAChangeToTheModel()
    {
        // Found on one model, applied to a bigger one, it picks the corresponding face
        var top = Part.Faces().Where(GeomType.Plane).GroupBy(Axis.Z).Last().Single();
        var selector = Selector.For(Part, top)[0];
        var bigger = Ops.Fillet((new Box(30, 15, 6) - new Cylinder(3, 10)).Edges().GroupBy(Axis.Z).Last().Where(GeomType.Line), 1);
        var picked = Assert.Single(selector.Apply(bigger));
        Assert.Equal(3, picked.Center().Z, 6);
    }

    [Fact]
    public void ThePickMustBelongToTheShape()
    {
        Assert.Throws<ArgumentException>(() => Selector.For(Part, new Box(1, 1, 1).Faces()[0]));
        Assert.Throws<ArgumentException>(() => Selector.For(Part, Part));
    }

    [Fact]
    public void IndistinguishableFacesHaveNoSelector()
    {
        // Two coincident boxes side by side in a compound: either top face can't be told from the other by place alone
        var box = new Box(1, 1, 1);
        var twins = new Compound([box, box.Moved(new Location((0.0, 0.0, 0.0)))]);
        Assert.Empty(Selector.For(twins, twins.Faces().GroupBy(Axis.Z).Last()[0]));
    }
}
