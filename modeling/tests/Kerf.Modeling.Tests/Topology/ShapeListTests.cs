// Ported from build123d tests/test_direct_api/test_shape_list.py.
// Box and Cylinder (centred Parts in build123d) are made with the direct API; where the Part class matters
// (compounds) the solid is wrapped in a Part. Omitted:
// - sort_by_invalid, filter_by_value_rejects_sort_by, group_by("AREA"), filter_by("True"), add_invalid_type: run-time
//   type checks the compiler rules out.
// - group_by_str_repr: str/repr with memory addresses and IPython pretty-printing.
// - The topo_distance_to([1]) and key(1) lines of the topological_distance error checks: type checks.
// - TestShapeListEmptyAxis: needs an Axis with its OCP object cleared, which can't be made.
// - add_vector: a ShapeList holds only shapes.
// - Part of filter_by_value_tolerance (2**53 + 1 compared exactly): C# measures are doubles, not Python ints.
// - Part of group_by_convexity (sort_by(Edge.convexity) raising TypeError): C# enums are ordered.
// - Part of test_equal (equality with AlwaysEqual): Python's __eq__ cooperation.
// - iadd_*: the in-place identity check, as ShapeList is immutable (+= rebinds to a new list).
// test_vertex, test_edge, … test_compound are Vertex_, Edge_, … Compound_, as a member can't share a type's name.
// filter_by_value_radius_over_mixed_faces: the fillet() operation on a Part is translated to IShape3D.Fillet.
// group_by's BuildPart with GridLocations/PolarLocations is replaced by the same boxes placed with Pos.
namespace Kerf.Modeling.Tests.Topology;

using Kerf.Modeling;

public class ShapeListTests
{
    static Solid Box(double l, double w, double h) => Solid.MakeBox(l, w, h, new Plane(new Point3D(-l / 2, -w / 2, -h / 2)));

    static Solid Cylinder(double r, double h) => Solid.MakeCylinder(r, h, new Plane(new Point3D(0, 0, -h / 2)));

    static Shape Pocket() => Box(20, 20, 10) - new Pos(z: 5) * Box(10, 10, 10);

    [Fact]
    public void SortBy()
    {
        var faces = Solid.MakeBox(1, 2, 3).Faces().OrderByDescending(ShapeKey.Area);
        Check.AlmostEqual(2, faces[^1].Area, 5);
    }

    [Fact]
    public void SortByLambda()
    {
        var c = Solid.MakeCone(2, 1, 2);
        var flatFaces = c.Faces().Where(GeomType.Plane);
        var sortedFlatFaces = flatFaces.OrderBy(f => f.Area).ToShapeList();
        Check.AlmostEqual(Math.PI * 1, sortedFlatFaces[0].Area, 5);
        Check.AlmostEqual(Math.PI * 4, sortedFlatFaces[^1].Area, 5);
    }

    [Fact]
    public void SortByProperty()
    {
        var box1 = Box(1, 1, 1);
        var box2 = Box(2, 2, 2);
        var box3 = Box(3, 3, 3);
        var unsortedBoxes = new ShapeList<Solid>([box2, box3, box1]);
        Assert.Equal(new[] { box1, box2, box3 }, unsortedBoxes.OrderBy(s => s.Volume).ToShapeList());
        Assert.Equal(new[] { box3, box2, box1 }, unsortedBoxes.OrderByDescending(s => s.Volume).ToShapeList());
    }

    [Fact]
    public void FilterByGeomtype()
    {
        var nonPlanarFaces = Solid.MakeCylinder(1, 1).Faces().WhereNot(GeomType.Plane);
        Assert.Single(nonPlanarFaces);
        Check.AlmostEqual(2 * Math.PI, nonPlanarFaces[0].Area, 5);
    }

    [Fact]
    public void FilterByAxis()
    {
        var box = Solid.MakeBox(1, 1, 1);
        Assert.Equal(2, box.Faces().ParallelTo(Axis.X).Count);
        Assert.Equal(4, box.Edges().ParallelTo(Axis.X).Count);
        Assert.Empty(box.Vertices().ParallelTo(Axis.X));
    }

    [Fact]
    public void FilterByPlane()
    {
        var c1 = Cylinder(1, 3);
        var c2 = new Rotation(90, 0, 0) * Cylinder(1, 3);

        Assert.Equal(2, c1.Faces().ParallelTo(Plane.XY).Count);
        Assert.Equal(2, c1.Edges().ParallelTo(Plane.XY).Count);
        Assert.Equal(2, c2.Faces().ParallelTo(Plane.XZ).Count);
        Assert.Equal(2, c2.Edges().ParallelTo(Plane.XZ).Count);
        Assert.Equal(2, c1.Wires().ParallelTo(Plane.XY).Count);
        Assert.Equal(2, c2.Wires().ParallelTo(Plane.XZ).Count);
    }

    [Fact]
    public void FilterByCallablePredicate()
    {
        var shapelist = new ShapeList<ModelShape>(
        [
            Solid.MakeBox(1, 1, 1).WithLabel("A"),
            Solid.MakeBox(1, 1, 1).WithLabel("A"),
            Solid.MakeBox(1, 1, 1).WithLabel("B"),
        ]);
        Assert.Equal(2, shapelist.Where(s => s.Label == "A").ToShapeList().Count);
        Assert.Single(shapelist.Where(s => s.Label == "B").ToShapeList());
    }

    [Fact]
    public void FilterByProperty()
    {
        var box1 = Box(2, 2, 2);
        var box2 = Box(2, 2, 2).Translate(new Vector3D(1, 1, 1));
        Assert.Equal(6, (box1 + box2).Edges().Where(Convexity.Concave).Count);
        Assert.Equal(3, (box1 - box2).Edges().Where(Convexity.Concave).Count);
    }

    [Fact]
    public void FilterByValueExactGeometry()
    {
        var box = Box(10, 20, 30);
        var edges = box.Edges().WithValue(e => e.Length, 30);
        Assert.IsType<ShapeList<Edge>>(edges);
        Assert.Equal(4, edges.Count);
        Assert.All(edges, e => Assert.True(Math.Abs(e.Length - 30) < 1e-9));
        Assert.Equal(2, box.Faces().WithValue(f => f.Area, 600).Count);
        Assert.Empty(box.Edges().WithValue(e => e.Length, 25));
        var rest = box.Edges().WithoutValue(e => e.Length, 30);
        Assert.Equal([10, 10, 10, 10, 20, 20, 20, 20], rest.Select(e => Math.Round(e.Length)).Order());
    }

    [Fact]
    public void FilterByValueRoundingNeighbours()
    {
        // Both are within 1e-6 of 30 but round to different GroupBy keys
        var below = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(29.9999994, 0, 0));
        var above = Edge.MakeLine(new Point3D(0, 1, 0), new Point3D(30.0000004, 1, 0));
        var other = Edge.MakeLine(new Point3D(0, 2, 0), new Point3D(31, 2, 0));
        var shapes = new ShapeList<Edge>([below, above, other]);
        Assert.Equal(3, shapes.GroupByValue(e => e.Length).Count);
        Assert.Equal(new[] { below, above }, shapes.WithValue(e => e.Length, 30));
    }

    [Fact]
    public void FilterByValueTolerance()
    {
        var edges = Box(10, 20, 30.001).Edges();
        Assert.Empty(edges.WithValue(e => e.Length, 30));
        Assert.Empty(edges.WithValue(e => e.Length, 30, tolerance: 1e-4));
        Assert.Equal(4, edges.WithValue(e => e.Length, 30, tolerance: 1e-2).Count);
        var edge = edges[..1];
        // Inclusive
        Assert.Equal(edge, edge.WithValue(_ => 1.125, 1, 0.125));
    }

    [Fact]
    public void FilterByValueCallable()
    {
        var top = Box(10, 20, 30).Faces().WithValue(f => f.Center().Z, 15);
        Assert.Single(top);
        Check.AlmostEqual(15, top[0].Center().Z, 5);
        Check.AlmostEqual(200, top[0].Area, 5);
    }

    [Fact]
    public void FilterByValueNonNumeric()
    {
        var faces = Cylinder(1, 3).Faces();
        Assert.Equal(2, faces.WithValue(f => f.GeomType, GeomType.Plane).Count);
        Assert.Single(faces.WithValue(f => f.GeomType, GeomType.Cylinder));
        var pocket = Pocket();
        var concave = pocket.Edges().WithValue(e => pocket.ConvexityOf(e), Convexity.Concave);
        Assert.Equal(8, concave.Count);
    }

    // build123d's fillet() operation on a Part is the Part's fillet, IShape3D.Fillet here
    [Fact]
    public void FilterByValueRadiusOverMixedFaces()
    {
        var box = Box(20, 20, 10);
        var part = box.Fillet(2, box.Edges().ParallelTo(Axis.Z)) - Cylinder(3, 10);
        var faces = part.Faces();
        var fillets = faces.WithValue(f => f.Radius, 2);
        Assert.Equal(4, fillets.Count);
        Assert.All(fillets, f => Assert.Equal(GeomType.Cylinder, f.GeomType));
        Assert.Single(faces.WithValue(f => f.Radius, 3));
        // A planar face has no radius, so it is never a match and always in the rest
        var rest = faces.WithoutValue(f => f.Radius, 2);
        Assert.Equal(faces.Count - 4, rest.Count);
        Assert.Equal(rest.Count - 1, rest.Where(GeomType.Plane).Count);
        Assert.Empty(faces.WithValue(f => f.Radius, (double?)null));
        Assert.Equal(faces, faces.WithoutValue(f => f.Radius, (double?)null));
    }

    [Fact]
    public void FilterByValueEmptyList() => Assert.Empty(ShapeList<Edge>.Empty.WithValue(e => e.Length, 30));

    [Fact]
    public void FirstLast()
    {
        var vertices = Solid.MakeBox(1, 1, 1).Vertices().OrderBy(new Axis(new Point3D(0, 0, 0), new Vector3D(1, 1, 1)));
        Check.AlmostEqual(new Point3D(1, 1, 1), vertices.Last().Point, 5);
        Check.AlmostEqual(new Point3D(0, 0, 0), vertices.First().Point, 5);
    }

    [Fact]
    public void GroupBy()
    {
        var vertices = Solid.MakeBox(1, 1, 1).Vertices().GroupBy(Axis.Z);
        Assert.Equal(4, vertices[0].Count);

        var edges = Solid.MakeBox(1, 1, 1).Edges().GroupBy(ShapeKey.Length);
        Assert.Equal(12, edges[0].Count);

        var circles = Solid.MakeCone(2, 1, 2).Edges().Where(GeomType.Circle).GroupBy(ShapeKey.Radius);
        Assert.Single(circles[0]);

        var smallest = Solid.MakeCone(2, 1, 2).Edges().Where(GeomType.Circle).GroupBy(ShapeKey.Radius).First();
        Check.AlmostEqual(2 * Math.PI, smallest[0].Length, 5);

        var byDistance = Solid.MakeBox(1, 1, 1).Vertices().GroupBy(ShapeKey.Distance);
        Check.AlmostEqual(new Point3D(1, 1, 1), (byDistance[^1][0].Point), 5);

        var box = Solid.MakeBox(1, 1, 2);
        Assert.Equal(2, box.Faces().GroupBy(ShapeKey.Area)[0].Count);
        Assert.Equal(4, box.Faces().GroupBy(ShapeKey.Area)[1].Count);

        var line = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 1, 2));
        var verticesByLine = box.Vertices().GroupBy(line);
        Assert.Single(verticesByLine[0]);
        Assert.Equal(2, verticesByLine[1].Count);
        Assert.Single(verticesByLine[2]);
        Assert.Single(verticesByLine[3]);
        Assert.Equal(2, verticesByLine[4].Count);
        Assert.Single(verticesByLine[5]);
        Check.AlmostEqual(new Point3D(0, 0, 0), verticesByLine[0][0].Point, 5);
        Check.AlmostEqual(new Point3D(1, 1, 2), (verticesByLine[^1][0].Point), 5);

        // GridLocations(10, 10, 3, 3) of 1x1x1 boxes and PolarLocations(100, 10) of 1x1x2 boxes
        var solids = new List<Solid>();
        for (int i = -1; i <= 1; i++)
            for (int j = -1; j <= 1; j++)
                solids.Add(new Pos(10 * i, 10 * j) * Box(1, 1, 1));
        for (int k = 0; k < 10; k++)
        {
            var angle = 36.0 * k * Math.PI / 180;
            solids.Add(new Location(new Point3D(100 * Math.Cos(angle), 100 * Math.Sin(angle), 0), 36.0 * k) * Box(1, 1, 2));
        }
        var boxes = (solids[0] + solids.Skip(1)).Solids();
        Assert.Equal(10, boxes.GroupBy(ShapeKey.Volume)[^1].Count);
        Assert.Equal(9, boxes.GroupBy(ShapeKey.Volume).First().Count);
    }

    static ShapeList<ModelShape> LabelledBoxes(out ModelShape a0, out ModelShape b0)
    {
        var boxesA = Enumerable.Range(0, 3).Select(_ => Solid.MakeBox(1, 1, 1).WithLabel("A")).ToList();
        var boxesB = Enumerable.Range(0, 2).Select(_ => Solid.MakeBox(1, 1, 1).WithLabel("B")).ToList();
        var boxNoLabel = Solid.MakeBox(1, 1, 1);
        (a0, b0) = (boxesA[0], boxesB[0]);
        return new ShapeList<ModelShape>([.. boxesA, .. boxesB, boxNoLabel]);
    }

    [Fact]
    public void GroupByCallablePredicate()
    {
        var result = LabelledBoxes(out _, out _).GroupByValue(shape => shape.Label);
        Assert.Equal([1, 3, 2], result.Select(g => g.Count));
    }

    [Fact]
    public void GroupByProperty()
    {
        var box1 = Box(2, 2, 2);
        var box2 = Box(2, 2, 2).Translate(new Vector3D(1, 1, 1));
        var fused = box1 + box2;
        var g1 = fused.Edges().GroupByValue(e => fused.ConvexityOf(e) == Convexity.Concave);
        Assert.Equal(6, g1.Group(true).Count);
        Assert.Equal(24, g1.Group(false).Count);

        var cut = box1 - box2;
        var g2 = cut.Edges().GroupByValue(e => cut.ConvexityOf(e) == Convexity.Concave);
        Assert.Equal(3, g2.Group(true).Count);
        Assert.Equal(18, g2.Group(false).Count);
    }

    [Fact]
    public void GroupByRetrieveGroups()
    {
        var result = LabelledBoxes(out var a0, out var b0).GroupByValue(shape => shape.Label);
        Assert.Single(result.Group(""));
        Assert.Equal(3, result.Group("A").Count);
        Assert.Equal(2, result.Group("B").Count);
        Assert.True(result.Group("") == result[0]);
        Assert.True(result.Group("A") == result[1]);
        Assert.True(result.Group("B") == result[2]);
        Assert.True(result.GroupFor(a0) == result.GroupFor(a0));
        Assert.True(result.GroupFor(a0) != result.GroupFor(b0));
        Assert.Throws<KeyNotFoundException>(() => result.Group("C"));
    }

    [Fact]
    public void Distance()
    {
        var box = Box(1, 2, 3);
        var obj = new Point3D(-0.2, 0.1, 0.5);
        var edges = box.Edges().OrderByDistanceTo(obj);
        var distances = edges.Select(e => Vertex.Make(obj).DistanceTo(e)).ToList();
        for (int i = 1; i < edges.Count; i++)
            Assert.True(distances[i] >= distances[i - 1]);
    }

    [Fact]
    public void DistanceReverse()
    {
        var box = Box(1, 2, 3);
        var obj = new Point3D(-0.2, 0.1, 0.5);
        var edges = box.Edges().OrderByDescendingDistanceTo(obj);
        var distances = edges.Select(e => Vertex.Make(obj).DistanceTo(e)).ToList();
        for (int i = 1; i < edges.Count; i++)
            Assert.True(distances[i] <= distances[i - 1]);
    }

    [Fact]
    public void DistanceEqual() => Assert.Equal(12, Box(1, 1, 1).Edges().OrderByDistanceTo(new Point3D(0, 0, 0)).Count);

    [Fact]
    public void TopologicalDistanceFaces()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var faces = box.Faces();
        var topFace = faces.OrderBy(Axis.Z)[^1];
        var bottomFace = faces.OrderBy(Axis.Z)[0];

        var faceGroups = faces.GroupByValue(TopologyUtils.TopoDistanceTo(box, topFace));

        Assert.Equal([1, 4, 1], faceGroups.Select(g => g.Count));
        Assert.Contains(topFace, faceGroups[0]);
        Assert.Contains(bottomFace, faceGroups[2]);
    }

    [Fact]
    public void TopologicalDistanceEdges()
    {
        var wire = Face.MakeRect(2, 1).OuterWire();
        var edges = wire.Edges();
        var topEdge = edges.OrderBy(Axis.Y)[^1];
        var bottomEdge = edges.OrderBy(Axis.Y)[0];

        var edgeGroups = edges.GroupByValue(TopologyUtils.TopoDistanceTo(wire, topEdge));

        Assert.Equal([1, 2, 1], edgeGroups.Select(g => g.Count));
        Assert.Contains(topEdge, edgeGroups[0]);
        Assert.Contains(bottomEdge, edgeGroups[2]);
    }

    [Fact]
    public void TopologicalDistanceVertices()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var vertices = box.Vertices();
        var referenceVertex = vertices.OrderBy(Axis.Z)[^1];
        var oppositeVertex = vertices.OrderByDescendingDistanceTo(referenceVertex)[0];

        var vertexGroups = vertices.GroupByValue(TopologyUtils.TopoDistanceTo(box, referenceVertex));

        Assert.Equal([1, 3, 3, 1], vertexGroups.Select(g => g.Count));
        Assert.Contains(referenceVertex, vertexGroups[0]);
        Assert.Contains(oppositeVertex, vertexGroups[3]);
    }

    [Fact]
    public void TopologicalDistanceWires()
    {
        var face = Face.MakeRect(1, 1) - Face.Make(Wire.MakeCircle(0.2));
        var wires = face.Wires();

        var wireGroups = wires.GroupByValue(TopologyUtils.TopoDistanceTo(face, wires[0]));

        Assert.Equal([1, 1], wireGroups.Select(g => g.Count));
        Assert.Equal(0, TopologyUtils.TopoDistanceTo(face, wires[0])(wires[0]));
        Assert.Equal(double.PositiveInfinity, TopologyUtils.TopoDistanceTo(face, wires[0])(wires[1]));
    }

    [Fact]
    public void TopologicalDistanceMultipleSources()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var faces = box.Faces();
        var topFace = faces.OrderBy(Axis.Z)[^1];
        var bottomFace = faces.OrderBy(Axis.Z)[0];

        var faceGroups = faces.GroupByValue(TopologyUtils.TopoDistanceTo(box, [topFace, bottomFace]));

        Assert.Equal([2, 4], faceGroups.Select(g => g.Count));
    }

    [Fact]
    public void TopologicalDistanceMovedShape()
    {
        // A moved copy keeps the unmoved container as topo_parent; peers are matched by TShape so the graph is still
        // reachable
        var movedBox = new Pos(x: 10) * new Part([Box(1, 1, 1)]).SingleSolid();
        var edges = movedBox.Edges();
        var edgeGroups = edges.GroupByValue(TopologyUtils.TopoDistanceTo(movedBox, edges[0]));

        Assert.Equal([1, 4, 6, 1], edgeGroups.Select(g => g.Count));
    }

    [Fact]
    public void TopologicalDistanceRequiresTopoParent()
    {
        var faces = TwoRects();
        Assert.Throws<ArgumentException>(() => TopologyUtils.TopoDistanceTo(default(Compound), faces[0]));
    }

    [Fact]
    public void TopologicalDistanceRejectsMixedParentAndOrphanSources()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var orphan = Face.MakeRect(1, 1);
        Assert.Throws<ArgumentException>(() => TopologyUtils.TopoDistanceTo(box, [box.Faces()[0], orphan]));
    }

    [Fact]
    public void TopologicalDistanceRequiresUniformShapeType()
    {
        var box = Solid.MakeBox(1, 1, 1);
        ShapeList<Shape> mixed = [box.Faces()[0], box.Edges()[0]];
        Assert.Throws<ArgumentException>(() => TopologyUtils.TopoDistanceTo(box, mixed));
    }

    [Fact]
    public void TopologicalDistanceSourceErrorChecking()
    {
        var box1 = Solid.MakeBox(1, 1, 1);
        var box2 = Solid.MakeBox(1, 1, 1, new Plane(new Point3D(2, 0, 0)));

        var e = Assert.Throws<ArgumentException>(() => TopologyUtils.TopoDistanceTo(box1, Array.Empty<Face>()));
        Assert.Contains("empty object", e.Message);

        e = Assert.Throws<ArgumentException>(() => TopologyUtils.TopoDistanceTo(box1, Compound.Make([box1, box2])));
        Assert.Contains("not supported", e.Message);

        e = Assert.Throws<ArgumentException>(() => TopologyUtils.TopoDistanceTo(box1, [box1.Faces()[0], box2.Faces()[0]]));
        Assert.Contains("the parent shape", e.Message);
    }

    [Fact]
    public void TopologicalDistanceKeyErrorChecking()
    {
        var box1 = Solid.MakeBox(1, 1, 1);
        var box2 = Solid.MakeBox(1, 1, 1, new Plane(new Point3D(2, 0, 0)));
        var key = TopologyUtils.TopoDistanceTo(box1, [(Shape)box1.Faces()[0]]);

        var e = Assert.Throws<ArgumentException>(() => key(box1.Edges()[0]));
        Assert.Contains("same type", e.Message);

        e = Assert.Throws<ArgumentException>(() => key(Face.MakeRect(1, 1)));
        Assert.Contains("the parent shape", e.Message);

        e = Assert.Throws<ArgumentException>(() => key(box2.Faces()[0]));
        Assert.Contains("the parent shape", e.Message);
    }

    [Fact]
    public void FilterByConvexity()
    {
        var box = Box(1, 1, 1);
        Assert.Equal(12, box.Edges().Where(Convexity.Convex).Count);
        Assert.Empty(box.Edges().WhereNot(Convexity.Convex));
        var pocket = Pocket();
        Assert.Equal(8, pocket.Edges().Where(Convexity.Concave).Count);
        Assert.Equal(11, pocket.Faces().Where(Convexity.Smooth).Count);
        Assert.Equal(4, pocket.Vertices().Where(Convexity.Saddle).Count);
    }

    [Fact]
    public void FilterByConvexityNeedsAnElement()
    {
        var e = Assert.Throws<InvalidOperationException>(() => new ShapeList<Shape>([new Part([Box(1, 1, 1)])]).Where(Convexity.Convex));
        Assert.Contains("not taken from a shape", e.Message);
    }

    [Fact]
    public void GroupByConvexity()
    {
        var groups = Pocket().Edges().GroupByConvexity();
        Assert.Equal([16, 8], groups.Select(g => g.Count));
        Assert.Equal(8, groups.Group(Convexity.Concave).Count);
        Assert.Equal(16, groups.Group(Convexity.Convex).Count);
    }

    static ShapeList<Face> TwoRects() => [Face.MakeRect(1, 1), Face.MakeRect(1, 1, new Plane(new Point3D(4, 4, 0)))];

    [Fact]
    public void Vertices() => Assert.Equal(8, TwoRects().Vertices().Count);

    [Fact]
    public void Vertex_()
    {
        ShapeList<Edge> sl = [Edge.MakeCircle(1)];
        Check.AlmostEqual(new Point3D(1, 0, 0), sl.SingleVertex().Point, 5);
        var e = Assert.Throws<InvalidOperationException>(() => TwoRects().SingleVertex());
        Assert.Contains("Expected exactly one vertex", e.Message);
        Assert.Empty(default(Edge).Vertices());
    }

    [Fact]
    public void Edges()
    {
        Assert.Equal(8, TwoRects().Edges().Count);
        Assert.Empty(default(Edge).Edges());
    }

    [Fact]
    public void Edge_()
    {
        ShapeList<Edge> sl = [Edge.MakeCircle(1)];
        Check.AlmostEqual(2 * Math.PI, sl.SingleEdge().Length, 5);
        var e = Assert.Throws<InvalidOperationException>(() => TwoRects().SingleEdge());
        Assert.Contains("Expected exactly one edge", e.Message);
    }

    [Fact]
    public void Wires()
    {
        Assert.Equal(2, TwoRects().Wires().Count);
        Assert.Empty(default(Wire).Wires());
    }

    [Fact]
    public void Wire_()
    {
        ShapeList<Wire> sl = [Wire.MakeCircle(1)];
        Check.AlmostEqual(2 * Math.PI, sl.SingleWire().Length, 5);
        var e = Assert.Throws<InvalidOperationException>(() => TwoRects().SingleWire());
        Assert.Contains("Expected exactly one wire", e.Message);
    }

    static ShapeList<Solid> BoxAndCylinder() => [Solid.MakeBox(1, 1, 1), Solid.MakeCylinder(1, 1)];

    [Fact]
    public void Faces()
    {
        Assert.Equal(9, BoxAndCylinder().Faces().Count);
        Assert.Empty(default(Face).Faces());
    }

    [Fact]
    public void Face_()
    {
        ShapeList<Shape> sl = [Vertex.Make(1, 1, 1), Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 1, 0)), Face.MakeRect(2, 1)];
        Check.AlmostEqual(2, sl.SingleFace().Area, 5);
        var e = Assert.Throws<InvalidOperationException>(() => BoxAndCylinder().SingleFace());
        Assert.Contains("Expected exactly one face", e.Message);
    }

    [Fact]
    public void Shells()
    {
        Assert.Equal(2, BoxAndCylinder().Shells().Count);
        Assert.Empty(default(Shell).Shells());
    }

    [Fact]
    public void Shell_()
    {
        ShapeList<Shape> sl = [Vertex.Make(1, 1, 1), Solid.MakeBox(1, 1, 1)];
        Check.AlmostEqual(6, sl.SingleShell().Area, 5);
        var e = Assert.Throws<InvalidOperationException>(() => BoxAndCylinder().SingleShell());
        Assert.Contains("Expected exactly one shell", e.Message);
    }

    [Fact]
    public void Solids()
    {
        Assert.Equal(2, BoxAndCylinder().Solids().Count);
        Assert.Empty(default(Solid).Solids());
    }

    [Fact]
    public void Solid_()
    {
        var e = Assert.Throws<InvalidOperationException>(() => BoxAndCylinder().SingleSolid());
        Assert.Contains("Expected exactly one solid", e.Message);
        ShapeList<Shape> sl = [Solid.MakeBox(1, 2, 3), Vertex.Make(1, 1, 1)];
        Check.AlmostEqual(6, sl.SingleSolid().Volume, 5);
    }

    // Box and Cylinder are Parts in build123d
    static ShapeList<Part> BoxAndCylinderParts() => [new Part([Box(1, 1, 1)]), new Part([Cylinder(1, 1)])];

    [Fact]
    public void Compounds()
    {
        Assert.Equal(2, BoxAndCylinderParts().Compounds().Count);
        Assert.Empty(default(Compound).Compounds());
    }

    [Fact]
    public void Compound_()
    {
        var e = Assert.Throws<InvalidOperationException>(() => BoxAndCylinderParts().SingleCompound());
        Assert.Contains("Expected exactly one compound", e.Message);
        ShapeList<Shape> sl = [new Part([Box(1, 2, 3)]), Vertex.Make(1, 1, 1)];
        Check.AlmostEqual(6, sl.SingleCompound().Volume, 5);
    }

    [Fact]
    public void Equal()
    {
        var box = new Part([Box(1, 1, 1)]);
        var cyl = new Part([Cylinder(1, 1)]);
        ShapeList<Part> sl = [box, cyl];
        ShapeList<Part> same = [cyl, box];
        Assert.True(sl == same);
        Assert.True(sl.Equals(same));
    }

    [Fact]
    public void NotEqual()
    {
        ShapeList<Part> sl = [new Part([Box(1, 1, 1)]), new Part([Cylinder(1, 1)])];
        ShapeList<Part> diff = [new Part([Box(1, 1, 1)]), new Part([Box(1, 2, 3)])];
        Assert.True(sl != diff);
        Assert.False(sl.Equals(new object()));
    }

    [Fact]
    public void Center()
    {
        var empty = ShapeList<Shape>.Empty.Center();
        Assert.Equal((0.0, 0.0, 0.0), (empty.X, empty.Y, empty.Z));
        var center = new ShapeList<Vertex>(Enumerable.Range(0, 3).Select(i => Vertex.Make(i, 0, 0))).Center();
        Assert.Equal((1.0, 0.0, 0.0), (center.X, center.Y, center.Z));
    }
}

public class ShapeListAdditionTests
{
    readonly Face _face1, _face2, _face3;

    public ShapeListAdditionTests()
    {
        static Solid Box() => Solid.MakeBox(1, 1, 1, new Plane(new Point3D(-0.5, -0.5, -0.5)));
        _face1 = Box().Faces().OrderBy(Axis.Z)[0];    // bottom face
        _face2 = Box().Faces().OrderBy(Axis.Z)[^1];   // top face
        _face3 = Box().Faces().OrderBy(Axis.X)[0];    // side face
    }

    [Fact]
    public void AddSingleShape()
    {
        ShapeList<Face> sl = [_face1];
        var result = sl.Add(_face2);
        Assert.IsType<ShapeList<Face>>(result);
        Assert.Equal(2, result.Count);
        Assert.Contains(_face1, result);
        Assert.Contains(_face2, result);
    }

    [Fact]
    public void AddShapeList()
    {
        ShapeList<Face> sl1 = [_face1];
        ShapeList<Face> sl2 = [_face2, _face3];
        var result = sl1.AddRange(sl2);
        Assert.IsType<ShapeList<Face>>(result);
        Assert.Equal(3, result.Count);
        Assert.Equal(new[] { _face1, _face2, _face3 }, result);
    }

    [Fact]
    public void IaddSingleShape()
    {
        ShapeList<Face> sl = [_face1];
        sl = sl.Add(_face2);
        Assert.Equal(2, sl.Count);
        Assert.Equal(new[] { _face1, _face2 }, sl);
    }

    [Fact]
    public void IaddShapeList()
    {
        ShapeList<Face> sl = [_face1];
        sl = sl.AddRange(new ShapeList<Face>([_face2, _face3]));
        Assert.Equal(3, sl.Count);
        Assert.Equal(new[] { _face1, _face2, _face3 }, sl);
    }

    [Fact]
    public void SetOperatorsPreserveOrder()
    {
        // Intersect and Except keep the order of the left operand (issue #1477)
        var edges = Solid.MakeBox(10, 20, 30, new Plane(new Point3D(-5, -10, -15))).Edges();
        Assert.Equal(12, edges.Count);

        var vertical = edges.ParallelTo(Axis.Z);
        var intersection = edges.Intersect(vertical);
        var verticalSet = vertical.ToHashSet();
        var expectedAnd = edges.Where(verticalSet.Contains).ToList();
        Assert.Equal(expectedAnd.Count, intersection.Count);
        Assert.All(intersection.Zip(expectedAnd), p => Check.Same(p.Second, p.First));

        ShapeList<Edge> removed = [edges[1], edges[4], edges[9]];
        var difference = edges.Except(removed);
        var removedSet = removed.ToHashSet();
        var expectedSub = edges.Where(e => !removedSet.Contains(e)).ToList();
        Assert.Equal(expectedSub.Count, difference.Count);
        Assert.All(difference.Zip(expectedSub), p => Check.Same(p.Second, p.First));
    }
}
