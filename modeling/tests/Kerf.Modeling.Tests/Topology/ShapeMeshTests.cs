using System.Diagnostics;
using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class ShapeMeshTests
{
    private static Vector Node(ReadOnlyMemory<float> xyz, int i) => new(xyz.Span[3 * i], xyz.Span[3 * i + 1], xyz.Span[3 * i + 2]);

    private static Vector Centroid(FaceMesh face) =>
        Enumerable.Range(0, face.NodeCount).Aggregate(new Vector(0, 0, 0), (sum, i) => sum + Node(face.Positions, i)) * (1.0 / face.NodeCount);

    [Fact]
    public void ABoxHasAMeshPerFaceEdgeAndVertex()
    {
        var box = new Box(10, 20, 30);
        var mesh = box.Triangulate(0.1);
        Assert.Equal(6, mesh.Faces.Count);
        Assert.Equal(12, mesh.Edges.Count);
        Assert.Equal(8, mesh.Vertices.Count);
        Assert.All(mesh.Faces, f => Assert.Equal((4, 2, GeomType.Plane), (f.NodeCount, f.TriangleCount, f.Type)));
        Assert.All(mesh.Edges, e => Assert.Equal((2, GeomType.Line), (e.PointCount, e.Type)));
        Assert.Equal(12, mesh.TriangleCount);
    }

    [Fact]
    public void TheOrderIsTheShapes()
    {
        var part = Ops.Fillet(new Box(10, 10, 10).Edges().GroupBy(Axis.Z).Last, 2) - new Cylinder(2, 20);
        var mesh = part.Triangulate(0.01);
        var faces = part.Faces();
        Assert.Equal(faces.Count, mesh.Faces.Count);
        for (int i = 0; i < faces.Count; i++)
        {
            // Every node of face i lies on face i
            var face = faces[i];
            Assert.Equal(face.GeomType, mesh.Faces[i].Type);
            for (int n = 0; n < mesh.Faces[i].NodeCount; n += 7)
                Assert.True(face.DistanceTo(Node(mesh.Faces[i].Positions, n)) < 1e-3, $"face {i} node {n}");
        }
        var edges = part.Edges();
        Assert.Equal(edges.Count, mesh.Edges.Count);
        for (int i = 0; i < edges.Count; i++)
        {
            // Polylines run the way their edges do
            var points = mesh.Edges[i];
            Assert.Equal(edges[i].GeomType, points.Type);
            Check.AlmostEqual(edges[i].StartPoint(), Node(points.Points, 0), 4);
            Check.AlmostEqual(edges[i].EndPoint(), Node(points.Points, points.PointCount - 1), 4);
        }
        Assert.Equal(part.Vertices().Select(v => v.ToVector()), mesh.Vertices);
    }

    [Fact]
    public void NormalsAndWindingFaceOut()
    {
        var part = new Box(10, 10, 10) - new Sphere(4).Moved(new Location((5.0, 5.0, 5.0)));
        var center = new Vector(0, 0, 0);
        foreach (var face in part.Triangulate(0.05).Faces)
        {
            var tri = face.Triangles.Span;
            for (int t = 0; t < face.TriangleCount; t++)
            {
                Vector a = Node(face.Positions, tri[3 * t]), b = Node(face.Positions, tri[3 * t + 1]), c = Node(face.Positions, tri[3 * t + 2]);
                var wound = (b - a).Cross(c - a);
                if (wound.Length < 1e-9)
                    continue;   // a degenerate triangle, at a pole, has no winding
                var normal = Node(face.Normals, tri[3 * t]) + Node(face.Normals, tri[3 * t + 1]) + Node(face.Normals, tri[3 * t + 2]);
                Assert.True(wound.Dot(normal) > 0, "the winding agrees with the normals");
            }
            // Out of the solid: away from its centre on the box's sides, into the spherical pocket at the corner
            var outward = face.Type == GeomType.Plane ? Centroid(face) - center : new Vector(5, 5, 5) - Centroid(face);
            Assert.True(Node(face.Normals, 0).Dot(outward) > 0, $"{face.Type} faces out");
        }
    }

    [Fact]
    public void AReversedFaceIsFlipped()
    {
        var face = Face.MakeRect(4, 4);
        Check.AlmostEqual((0, 0, 1), Node(face.Triangulate(0.1).Faces[0].Normals, 0), 6);
        Check.AlmostEqual((0, 0, -1), Node((-face).Triangulate(0.1).Faces[0].Normals, 0), 6);
    }

    [Fact]
    public void NormalsAreTheSurfacesSoShadingIsSmooth()
    {
        var mesh = new Sphere(3).Triangulate(0.01);
        var face = Assert.Single(mesh.Faces);
        Assert.Equal(GeomType.Sphere, face.Type);
        for (int i = 0; i < face.NodeCount; i++)
            Check.AlmostEqual(Node(face.Positions, i).Normalized(), Node(face.Normals, i), 5);
    }

    [Fact]
    public void EdgesLieOnTheFaceMeshes()
    {
        var mesh = new Cylinder(5, 10).Triangulate(0.01);
        var nodes = mesh.Faces.SelectMany(f => Enumerable.Range(0, f.NodeCount).Select(i => Node(f.Positions, i))).ToHashSet();
        foreach (var edge in mesh.Edges)
            for (int i = 0; i < edge.PointCount; i++)
                Assert.Contains(Node(edge.Points, i), nodes);
    }

    [Fact]
    public void FreeEdgesHavePolylines()
    {
        var mesh = Edge.MakeCircle(5).Triangulate(0.01);
        Assert.Empty(mesh.Faces);
        var circle = Assert.Single(mesh.Edges);
        Assert.Equal(GeomType.Circle, circle.Type);
        Assert.True(circle.PointCount > 20);
        for (int i = 0; i < circle.PointCount; i++)
            Assert.Equal(5, Node(circle.Points, i).Length, 4);
    }

    [Fact]
    public void AnEmptyShapeHasAnEmptyMesh()
    {
        Assert.Same(ShapeMesh.Empty, new Compound([]).Triangulate(0.1));
        Assert.Equal(0, new Part().Triangulate().TriangleCount);
    }

    [Fact]
    public void PositionsAreLocated()
    {
        var box = new Box(2, 2, 2).Moved(new Location((10.0, 20.0, 30.0), (0.0, 0.0, 45.0)));
        var mesh = box.Triangulate(0.1);
        var points = mesh.Faces.SelectMany(f => Enumerable.Range(0, f.NodeCount).Select(i => Node(f.Positions, i))).ToList();
        var bounds = box.BoundingBox();
        Assert.All(points, p => Assert.True(p.X >= bounds.Min.X - 1e-4 && p.X <= bounds.Max.X + 1e-4 && p.Y >= bounds.Min.Y - 1e-4 && p.Y <= bounds.Max.Y + 1e-4 && p.Z >= bounds.Min.Z - 1e-4 && p.Z <= bounds.Max.Z + 1e-4));
        Check.AlmostEqual(box.Center(), points.Aggregate(new Vector(0, 0, 0), (s, p) => s + p) * (1.0 / points.Count), 4);
    }

    [Fact]
    public void TheSameShapeAndTolerancesGiveTheSameMesh()
    {
        var part = Ops.Fillet(new Box(10, 10, 10).Edges(), 1);
        var a = part.Triangulate(0.01);
        var b = part.Triangulate(0.01);
        Assert.Equal(a.Faces.Count, b.Faces.Count);
        for (int i = 0; i < a.Faces.Count; i++)
        {
            Assert.True(a.Faces[i].Positions.Span.SequenceEqual(b.Faces[i].Positions.Span));
            Assert.True(a.Faces[i].Triangles.Span.SequenceEqual(b.Faces[i].Triangles.Span));
        }
    }

    [Fact]
    public void ShapesSharingGeometryTriangulateConcurrently()
    {
        var part = Ops.Fillet(new Box(10, 10, 10).Edges(), 1);
        var copies = Enumerable.Range(0, 16).Select(i => part.Moved(new Location((i * 20.0, 0.0, 0.0)))).ToList();
        var expected = part.Triangulate(0.01).TriangleCount;
        var counts = new int[copies.Count];
        Parallel.For(0, copies.Count, i => counts[i] = copies[i].Triangulate(0.01).TriangleCount);
        Assert.All(counts, c => Assert.Equal(expected, c));
    }

    [Fact]
    public void DefaultTolerancesFollowTheShapesSize()
    {
        var small = MeshTolerances.For(new Box(1, 1, 1));
        var large = MeshTolerances.For(new Box(1000, 1000, 1000));
        Assert.Equal(1000, large.Linear / small.Linear, 6);
        Assert.Equal(Math.Sqrt(3) * 0.1 / 100, small.Linear, 9);
        var mesh = new Sphere(50).Triangulate();
        Assert.Equal(MeshTolerances.For(new Sphere(50)), mesh.Tolerances);
        Assert.True(mesh.TriangleCount > 100);
    }

    [Fact]
    public void AWireTriangulates()
    {
        // It threw until unit 10B: the binding wrapped the meshed copy as a plain shape, which a wire's edges couldn't cast
        var wire = Wire.MakeRect(2, 4);
        var mesh = wire.Triangulate();
        Assert.Empty(mesh.Faces);
        Assert.Equal(4, mesh.Edges.Count);
        Assert.Equal(4, wire.TriangulateTree().Mesh!.Edges.Count);
    }

    [Fact]
    public void ABigMeshComesOutQuickly()
    {
        // Meshing itself is OCCT's: what's timed is copying the mesh out, the part that is the library's
        var sphere = new Sphere(100);
        var meshed = sphere.Rewrapped(sphere.MeshedCopy(0.01, 0.2, relative: false));
        var faces = meshed.Faces().Select(f => f.TopoShape).ToList();
        var edges = meshed.Edges().Select(e => e.TopoShape).ToList();
        var clock = Stopwatch.StartNew();
        var buffers = Kerf.Engine.Meshing.MeshExtraction.Extract(meshed.TopoShape, faces, edges, 0.01, 0.2);
        clock.Stop();
        var triangles = buffers.Triangles.Length / 3;
        Assert.True(triangles > 100_000, $"{triangles} triangles");
        Assert.True(clock.Elapsed < TimeSpan.FromMilliseconds(100), $"{triangles} triangles took {clock.Elapsed.TotalMilliseconds:F0} ms to copy out");
    }

    [Fact]
    public void AnAssemblyTreeSharesTheMeshOfInstances()
    {
        var bolt = new Cylinder(1, 10).WithLabel("bolt").WithColor(new Color(0.5, 0.5, 0.5, 0.8));
        var plate = new Box(20, 20, 2).WithLabel("plate");
        var assembly = Compound.Assembly(
            [plate, bolt.Moved(new Location((-5.0, 0.0, 0.0))), bolt.Moved(new Location((5.0, 0.0, 0.0)))], label: "joint");

        var tree = assembly.TriangulateTree();
        Assert.Equal("joint", tree.Label);
        Assert.Null(tree.Mesh);
        Assert.Equal(3, tree.Children.Count);
        var (first, second) = (tree.Children[1], tree.Children[2]);
        Assert.Equal(("bolt", 0.8), (first.Label, first.Color!.Value.A));
        Assert.Same(first.Mesh, second.Mesh);
        Assert.NotSame(tree.Children[0].Mesh, first.Mesh);
        Check.AlmostEqual((-5, 0, 0), first.Location.Position, 6);
        Check.AlmostEqual((5, 0, 0), second.Location.Position, 6);
        // An instance's mesh is in its own coordinates
        var cylinder = first.Mesh!.Faces.Single(f => f.Type == GeomType.Cylinder);
        var nodes = Enumerable.Range(0, cylinder.NodeCount).Select(i => Node(cylinder.Positions, i)).ToList();
        Assert.All(nodes, n => Assert.Equal(1, Math.Sqrt(n.X * n.X + n.Y * n.Y), 4));   // about its own axis, not at x = ±5
    }

    [Fact]
    public void APartIsALeaf()
    {
        var tree = new Box(1, 1, 1).Moved(new Location((1.0, 2.0, 3.0))).TriangulateTree();
        Assert.Empty(tree.Children);
        Assert.Equal(6, tree.Mesh!.Faces.Count);
        Check.AlmostEqual((1, 2, 3), tree.Location.Position, 6);
    }
}
