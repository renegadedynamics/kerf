// A file that imports the kernel's and the modelling layer's namespaces together, at the top as a user's would: question 56,
// one set of shapes. Everything here has to bind without ambiguity.
using Kerf;
using Kerf.Modeling;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

namespace Build123d.Tests.Topology;

public class OneSetOfShapesTests
{
    [Fact]
    public void SelectionsAreTheKernelsShapes()
    {
        Part box = new Box(10, 10, 10);
        Edge edge = box.Edges().ParallelTo(Axis.Z)[0];
        Face face = box.Faces().OrderBy(Axis.Z).Last();
        Solid solid = box.SingleSolid();

        // The kernel's own members and the layer's conveniences, on the one value
        Assert.Equal(ShapeType.Edge, edge.Type);
        Assert.Equal(10, edge.Length, 9);
        Assert.Equal(100, face.Area, 9);
        Assert.Equal(GeomType.Plane, face.GeomType);
        Assert.Equal(1000, solid.Volume, 6);
        Assert.Equal(12, solid.SelectEdges().Count);
    }

    [Fact]
    public void AnObjectGoesWhereAShapeGoes()
    {
        Part box = new Box(1, 2, 3);
        Shape shape = box;
        Assert.True(shape.IsSame(box.Shape));
        Assert.Equal(6, shape.Volume, 6);
        Assert.Equal(6, box.Volume, 6);
        Assert.Equal(6, ((Solid)box.Moved(new Pos(1, 0, 0)).SingleSolid()).Volume, 6);
        Assert.IsType<Box>(box.Moved(new Pos(1, 0, 0)));
    }

    [Fact]
    public void ATypedShapeKeepsItsType()
    {
        Edge edge = Edge.MakeLine(new Vector(0, 0), new Vector(1, 0));
        Edge moved = edge.Moved(new Pos(0, 0, 5));
        Edge viaOperator = new Pos(0, 0, 5) * edge;
        Edge reversed = edge.Reversed();
        Assert.Equal(5, moved.Center().Z, 9);
        Assert.Equal(5, viaOperator.Center().Z, 9);
        Assert.True(reversed.IsSame(edge));
    }

    [Fact]
    public void EmptyIsTheDefaultOrAnEmptyCompound()
    {
        Assert.True(default(Solid).IsEmpty);
        Assert.True(Compound.Make(Array.Empty<Shape>()).IsEmpty);
        Assert.True(new Part().IsEmpty);
        Assert.False(new Box(1, 1, 1).IsEmpty);
    }

    [Fact]
    public void AListRemembersWhatItWasSelectedFrom()
    {
        Part box = new Box(10, 10, 10);
        var edges = box.Edges();
        Assert.Same(box, edges.Origin);

        // Every selection keeps it
        Assert.Same(box, edges.ParallelTo(Axis.Z).Origin);
        Assert.Same(box, edges.Where(GeomType.Line).Origin);
        Assert.Same(box, edges.Where(e => e.Length > 1).Origin);
        Assert.Same(box, edges.OrderBy(Axis.Z).Origin);
        Assert.Same(box, edges.OrderByDescending(ShapeKey.Length).Origin);
        Assert.Same(box, edges.GroupBy(Axis.Z)[0].Origin);
        Assert.Same(box, edges[1..3].Origin);
        Assert.Same(box, edges.Except(edges[..1]).Origin);

        // LINQ's forms leave the list, and the origin with it
        Assert.Null(edges.Select(e => e).ToShapeList().Origin);

        // A selection through a sub-shape has that sub-shape
        Face face = box.Faces()[0];
        Assert.True(face.SelectEdges().Origin!.AsShape().IsSame(face));
    }

    [Fact]
    public void AnOperationTakesItsTargetFromTheListOrTheCaller()
    {
        Part box = new Box(10, 10, 10);
        var edges = box.Edges().ParallelTo(Axis.Z);
        Part fromList = Fillet(edges, 1).Value;
        Part fromCaller = Fillet(box, edges.ToList(), 1).Value;
        Assert.Equal(fromList.Volume, fromCaller.Volume, 9);

        // A single edge has no origin: it names its target
        Part one = Fillet(box, edges[0], 1).Value;
        Assert.True(one.Volume < 1000);
        Assert.Throws<ArgumentException>(() => Fillet(edges.ToList(), 1));
    }

    [Fact]
    public void ConvexityIsAskedOfTheParent()
    {
        Part box = new Box(10, 10, 10);
        Assert.All(box.Edges(), e => Assert.Equal(Convexity.Convex, box.ConvexityOf(e)));
        Assert.Equal(12, box.Edges().Where(Convexity.Convex).Count);
        Assert.All(box.Vertices(), v => Assert.Equal(Convexity.Convex, box.ConvexityOf(v)));

        // Through a face, a vertex is a corner of that face
        Face face = box.Faces()[0];
        Assert.Equal(Convexity.Convex, box.ConvexityOf(face.SelectVertices()[0], through: face));
    }

    [Fact]
    public void AKernelShapeBecomesAnObjectToCarryAttributes()
    {
        var solid = Solid.MakeBox(1, 1, 1).WithLabel("brick").WithColor(new Color(1, 0, 0));
        var part = Assert.IsType<Part>(solid);
        Assert.Equal("brick", part.Label);
        Assert.Equal(new Color(1, 0, 0), part.Color);
        Assert.IsType<Sketch>(Face.MakeRect(1, 1).WithLabel("plate"));
        Assert.IsType<Curve>(Edge.MakeLine(new Vector(0, 0), new Vector(1, 0)).WithLabel("line"));
        Assert.IsType<Assembly>(Vertex.Make(0, 0, 0).WithLabel("point"));
    }

    [Fact]
    public void AnAssemblyHoldsObjects()
    {
        var bolt = new Cylinder(1, 10).WithLabel("bolt");
        var assembly = new Assembly([bolt, Solid.MakeBox(1, 1, 1)], "kit");
        Assert.Equal("kit", assembly.Label);
        Assert.Same(bolt, assembly.Children[0]);
        Assert.IsType<Part>(assembly.Children[1]);
        Assert.Equal(2, assembly.Shape.ChildCount);
    }

    [Fact]
    public void ALineIsACurveAndConvertsToItsEdge()
    {
        var line = new Line(new Vector(0, 0), new Vector(3, 4));
        Edge edge = line;
        Assert.Equal(5, line.Length, 9);
        Assert.Equal(5, edge.Length, 9);
        Curve joined = line + new Line(new Vector(3, 4), new Vector(3, 0));
        Assert.Equal(ShapeType.Wire, joined.Shape.Type);
    }

    [Fact]
    public void ProjectingFacesOntoACurvedSurfaceKeepsTheirFaces()
    {
        // Unit 10P3 found an InvalidCastException here: a projection onto a curved surface can be a shell, not a face
        var cylinder = Solid.MakeCylinder(20, 40, new Plane(new Vector(0, 0, -20)));
        var path = Edge.MakeLine(new Vector(-20, -50, 0), new Vector(-20, 50, 0));
        var faces = new[] { Face.MakeRect(4, 4) };
        var projected = cylinder.ProjectFaces(faces, path, 0.5);
        Assert.NotEmpty(projected);
        Assert.All(projected, f => Assert.Equal(ShapeType.Face, f.Type));
    }
}
