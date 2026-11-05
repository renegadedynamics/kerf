// A file that imports the kernel's and the modelling layer's namespaces together, at the top as a user's would: question 56,
// one set of shapes. Everything here has to bind without ambiguity.
using Kerf;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Modeling;
using Kerf.Topology;
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Topology;

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
        Assert.Equal(12, solid.Edges().Count);
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
        Edge edge = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
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
        Assert.True(face.Edges().Origin!.AsShape().IsSame(face));
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
        Assert.Equal(Convexity.Convex, box.ConvexityOf(face.Vertices()[0], through: face));
    }

    [Fact]
    public void AKernelShapeBecomesAnObjectToCarryAttributes()
    {
        var solid = Solid.MakeBox(1, 1, 1).WithLabel("brick").WithColor(new Color(1, 0, 0));
        var part = Assert.IsType<Part>(solid);
        Assert.Equal("brick", part.Label);
        Assert.Equal(new Color(1, 0, 0), part.Color);
        Assert.IsType<Sketch>(Face.MakeRect(1, 1).WithLabel("plate"));
        Assert.IsType<Curve>(Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).WithLabel("line"));
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
        var line = new Line(new Point3D(0, 0, 0), new Point3D(3, 4, 0));
        Edge edge = line;
        Assert.Equal(5, line.Length, 9);
        Assert.Equal(5, edge.Length, 9);
        Curve joined = line + new Line(new Point3D(3, 4, 0), new Point3D(3, 0, 0));
        Assert.Equal(ShapeType.Wire, joined.Shape.Type);
    }

    [Fact]
    public void ProjectingFacesOntoACurvedSurfaceKeepsTheirFaces()
    {
        // Unit 10P3 found an InvalidCastException here: a projection onto a curved surface can be a shell, not a face
        var cylinder = Solid.MakeCylinder(20, 40, new Plane(new Point3D(0, 0, -20)));
        var path = Edge.MakeLine(new Point3D(-20, -50, 0), new Point3D(-20, 50, 0));
        var faces = new[] { Face.MakeRect(4, 4) };
        var projected = cylinder.ProjectFaces(faces, path, 0.5);
        Assert.NotEmpty(projected);
        Assert.All(projected, f => Assert.Equal(ShapeType.Face, f.Type));
    }

    [Fact]
    public void AFileWithEveryNamespaceBindsTheCommonNames()
    {
        // Kerf.Geometry beside Kerf.Modeling: the objects keep the plain names, the analytic values say 3D
        Part cylinder = new Cylinder(1, 2);
        Part sphere = new Sphere(1);
        Part cone = new Cone(1, 0.5, 2);
        Part torus = new Torus(2, 0.5);
        var surface = new Cylinder3D(new CoordinateSystem(), 1);
        Assert.Equal(1, surface.Radius);
        Assert.Equal(Math.PI * 2, cylinder.Volume, 6);
        Assert.Equal(4 * Math.PI / 3, sphere.Volume, 6);
        Assert.True(cone.Volume > 0 && torus.Volume > 0);
        Assert.Equal(new Sphere3D(new CoordinateSystem(), 2).Radius, new Torus3D(new CoordinateSystem(), 2, 1).MajorRadius);
        Assert.True(new Cone3D(new CoordinateSystem(), 0.5, 1).RefRadius == 1);
        Point3D point = new(1, 2, 3);
        Tangency tangency = Tangency.Enclosing;
        Assert.Equal(1, (int)tangency);
        Assert.Equal(1, point.X);

        // Kerf.Measurement beside Kerf.Modeling: the plain names are properties
        Solid box = Solid.MakeBox(1, 2, 3);
        Assert.Equal(6, box.Volume, 6);
        Assert.Equal(22, box.Area, 6);
        Assert.Equal(Math.PI * 2, Edge.MakeCircle(1).Length, 6);
        Assert.Equal(6, box.IntegrateVolume(), 6);
    }

    [Fact]
    public void VolumeCountsSolidsWhereIntegratingCountsWhateverTheFacesBound()
    {
        // A face has no volume; integrating gives what its surface bounds
        Face sphereFace = Solid.MakeSphere(1).Faces()[0];
        Assert.Equal(0, sphereFace.Volume);
        Assert.Equal(4 * Math.PI / 3, sphereFace.IntegrateVolume(), 6);

        // A compound's counts the solids in it, and its loose faces not at all
        Compound mixed = Compound.Make([Solid.MakeBox(1, 1, 1), sphereFace]);
        Assert.Equal(1, mixed.Volume, 6);
        Assert.Equal(1 + 4 * Math.PI / 3, mixed.IntegrateVolume(), 6);

        // The other two differ only on nothing: zero, where the kernel refuses
        Assert.Equal(0, default(Face).Area);
        Assert.Equal(0, default(Edge).Length);
        Assert.Throws<ArgumentException>(() => default(Face).IntegrateArea());
        Assert.Throws<ArgumentException>(() => default(Edge).IntegrateLength());
        Assert.Equal(4 * Math.PI, sphereFace.Area, 6);
        Assert.Equal(sphereFace.Area, sphereFace.IntegrateArea(), 12);
    }

    [Fact]
    public void IsClosedLooks()
    {
        // The trap of decision 1622: a full circle's edge, which no builder flags as closed
        Part ring = new Sphere(2) - new Box(10, 10, 1);
        Edge circle = Edge.MakeCircle(1);
        Assert.True(circle.IsClosed);
        Assert.False(Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).IsClosed);
        Assert.Contains(ring.Edges(), e => e.IsClosed);
        Part cylinder = new Cylinder(1, 2);
        Assert.Equal(2, cylinder.Edges().Where(e => e.IsClosed && e.GeomType == GeomType.Circle).Count);

        Assert.True(Wire.MakeCircle(1).IsClosed);
        Assert.False(Wire.Make([Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0))]).IsClosed);
        Assert.False(Face.MakeRect(1, 1).IsClosed);
        Assert.True(Solid.MakeSphere(1).Faces()[0].IsClosed);
        Assert.True(Solid.MakeBox(1, 1, 1).Shells()[0].IsClosed);
        Assert.True(Solid.MakeBox(1, 1, 1).IsClosed);
        Assert.True(Compound.Make([Solid.MakeBox(1, 1, 1), Edge.MakeCircle(1)]).IsClosed);
        Assert.False(Compound.Make([Solid.MakeBox(1, 1, 1), Face.MakeRect(1, 1)]).IsClosed);
        Assert.False(Vertex.Make(0, 0, 0).IsClosed);
        Assert.True(new Curve((Shape)circle).IsClosed);
        Assert.Throws<InvalidOperationException>(() => default(Edge).IsClosed);
    }

    [Fact]
    public void AStructAndAnObjectSelectAlike()
    {
        Solid solid = Solid.MakeSphere(1);
        Part part = new Part(solid);

        // The plain names leave out a sphere's degenerate edges and return a selection, on either
        ShapeList<Edge> ofStruct = solid.Edges(), ofObject = part.Edges();
        Assert.Equal(ofObject.Count, ofStruct.Count);
        Assert.True(solid.AllEdges().Length > ofStruct.Count);
        Assert.Contains(solid.AllEdges(), e => e.IsDegenerated);
        Assert.DoesNotContain(ofStruct, e => e.IsDegenerated);
        Assert.True(ofStruct.Origin is not null);
        Assert.Equal(part.Faces().Count, solid.Faces().Count);
        Assert.Equal(part.Vertices().Count, solid.Vertices().Count);
        Assert.Equal(part.Wires().Count, solid.Wires().Count);
        Assert.Equal(part.Shells().Count, solid.Shells().Count);
        Assert.Equal(part.Solids().Count, solid.Solids().Count);

        // A shape's Location is Kerf's; OrientationAngles reads the same on both, and the kernel's Orientation is the enum
        Assert.Equal(part.OrientationAngles, solid.OrientationAngles);
        Assert.Equal(Orientation.Forward, solid.Orientation);
        Assert.False(Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).IsInfinite);
        Assert.True(Edge.FromAxis(new Axis(Point3D.Origin, new Vector3D(1, 0, 0))).IsInfinite);
    }
}
