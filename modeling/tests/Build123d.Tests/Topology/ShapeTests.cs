// Ported from build123d tests/test_direct_api/test_shape.py. Box(l, w, h) and Cylinder(r, h) are the centred direct-API
// solids (a Solid, where build123d's objects are a Part). Omitted:
// - test_compute_mass, and compute_mass in test_empty_shape: materials and mass are phase 7.
// - test_max_fillet_invalid_shape_raises_error, test_tessellate_missing_triangulation, TestMakeComposite: they patch
//   library internals with mocks.
// - test_copy_vertex: copy.copy and copy.deepcopy have no counterpart (shapes are immutable).
// - test_equal's AlwaysEqual and test_not_equal's object(): Python's __eq__ protocol; only shape equality is ported.
// - test_boolean_zero_cut_multi_args, test_boolean_zero_fuse_multi_args, test_boolean_zero_fuse_multi_tools: _bool_op
//   is private.
// - test_project_to_viewport: hidden-line removal is phase 5.
// - test_compound, and the error case of test_shell: they need text (phase 7) and extrude().
// - test_copy_attributes_to: joints, children and topo_parent setters (joints are phase 7; children hold no parent).
// - test_empty_shape's locate/move "is" checks: those are mutators; the copies are checked empty instead.
// - test_rmul_iterable_non_location, test_rmul_non_iterable_non_location: run-time type checks the compiler rules out.
// - TestGlobalLocation: needs parent links and global_location, which the port's assemblies don't have.
// - TestShapeCast.test_unregistered_type_is_reported, .test_registry_covers_every_wrapped_type and
//   TestAsShape.test_registry_covers_the_geometry_types: Shape.Cast and AsShape are switches, not registries.
using Occt;

namespace Build123d.Tests.Topology;

file static class Objects
{
    public static Solid Box(double l, double w, double h) => Solid.MakeBox(l, w, h, new Plane((-l / 2, -w / 2, -h / 2)));

    public static Solid Cylinder(double r, double h) => Solid.MakeCylinder(r, h, new Plane((0, 0, -h / 2)));
}

public class ShapeTests
{
    [Fact]
    public void Mirror()
    {
        var boxBb = Solid.MakeBox(1, 1, 1).Mirror(Plane.XZ).BoundingBox();
        Check.AlmostEqual(0, boxBb.Min.X, 5);
        Check.AlmostEqual(1, boxBb.Max.X, 5);
        Check.AlmostEqual(-1, boxBb.Min.Y, 5);
        Check.AlmostEqual(0, boxBb.Max.Y, 5);

        boxBb = Solid.MakeBox(1, 1, 1).Mirror().BoundingBox();
        Check.AlmostEqual(-1, boxBb.Min.Z, 5);
        Check.AlmostEqual(0, boxBb.Max.Z, 5);
    }

    [Fact]
    public void CombinedCenter()
    {
        var objs = new[] { -2, 1 }.Select(x => Solid.MakeBox(1, 1, 1, new Plane((x, 0, 0)))).ToList();
        Check.AlmostEqual((0, 0.5, 0.5), Shape.CombinedCenter(objs, CenterOf.Mass), 5);

        objs = new[] { -2, 1 }.Select(x => Solid.MakeSphere(1, new Plane((x, 0, 0)))).ToList();
        Check.AlmostEqual((-0.5, 0, 0), Shape.CombinedCenter(objs, CenterOf.BoundingBox), 5);
        Assert.Throws<ArgumentException>(() => Shape.CombinedCenter(objs, CenterOf.Geometry));
    }

    [Fact]
    public void ShapeType() => Assert.Equal(Build123d.ShapeType.Vertex, new Vertex().ShapeType);

    [Fact]
    public void Scale()
    {
        Check.AlmostEqual(Math.Pow(2, 3), Solid.MakeBox(1, 1, 1).Scale(2).Volume, 5);

        var locatedBox = Solid.MakeBox(1, 1, 1).Located(new Location((10, 0, 0)));
        var scaledBox = locatedBox.Scale(2);
        Check.AlmostEqual(Math.Pow(2, 3), scaledBox.Volume, 5);
        Check.AlmostEqual(11, scaledBox.Center().X, 5);

        var nonUniformBox = locatedBox.Scale((2, 3, 4));
        Check.AlmostEqual(2 * 3 * 4, nonUniformBox.Volume, 5);
        Check.AlmostEqual(11, nonUniformBox.Center().X, 5);

        var originScaledBox = locatedBox.Scale(2, about: (0, 0, 0));
        Check.AlmostEqual(21, originScaledBox.Center().X, 5);
    }

    [Fact]
    public void Fuse()
    {
        var box1 = Solid.MakeBox(1, 1, 1);
        var box2 = Solid.MakeBox(1, 1, 1, new Plane((1, 0, 0)));
        var combined = box1.Fuse([box2], glue: true);
        Assert.True(combined.IsValid);
        Check.AlmostEqual(2, combined.Volume, 5);
        var fuzzy = box1.Fuse([box2], tolerance: 1e-6);
        Assert.True(fuzzy.IsValid);
        Check.AlmostEqual(2, fuzzy.Volume, 5);
    }

    [Fact]
    public void BooleanZeroCut()
    {
        var box = Solid.MakeBox(1, 2, 3);
        var zero = new Solid();

        var result = box.Cut(zero);
        Assert.IsType<Solid>(result);
        Assert.True(result.IsValid);
        Check.AlmostEqual(box.Volume, result.Volume, 5);

        result = box - zero;
        Assert.IsType<Solid>(result);
        Assert.True(result.IsValid);
        Check.AlmostEqual(box.Volume, result.Volume, 5);
    }

    [Fact]
    public void BooleanZeroFuse()
    {
        var box = Solid.MakeBox(1, 2, 3);
        var zero = new Solid();

        foreach (var result in new[] { box.Fuse(zero), zero.Fuse(box), box + zero, zero + box })
        {
            Assert.IsType<Solid>(result);
            Assert.True(result.IsValid);
            Check.AlmostEqual(box.Volume, result.Volume, 5);
        }
    }

    [Fact]
    public void FacesIntersectedByAxis()
    {
        var box = Solid.MakeBox(1, 1, 1, new Plane((0, 0, 1)));
        var intersectedFaces = box.FacesIntersectedByAxis(Axis.Z);
        Assert.Contains(box.Faces().SortBy(Axis.Z)[0], intersectedFaces);
        Assert.Contains(box.Faces().SortBy(Axis.Z).Last, intersectedFaces);
    }

    [Fact]
    public void Split()
    {
        var shape = Objects.Box(1, 1, 1) - new Pos(0, 0, -0.25) * Objects.Box(1, 0.5, 0.5);
        var splitShape = shape.Split(Plane.XY, Keep.Bottom);
        Assert.Equal(2, splitShape.Count);
        Check.AlmostEqual(0.25, splitShape[0].Volume + splitShape[1].Volume, 5);
        // build123d returns the single piece itself; here it is a list of one
        splitShape = shape.Split(Plane.XY, Keep.Top);
        Assert.Single(splitShape.Solids());
        Assert.IsType<Solid>(Assert.Single(splitShape));
        Check.AlmostEqual(0.5, splitShape[0].Volume, 5);

        var s = Solid.MakeCone(1, 0.5, 2, Plane.YZ.Offset(10));
        var tool = Solid.MakeSphere(11).Rotate(Axis.Z, 90).SingleFace();
        var s2 = s.Split(tool, Keep.Top).Sum(p => p.Volume);
        Assert.True(s2 < s.Volume);
        Assert.True(s2 > 0.0);
    }

    [Fact]
    public void SplitCurvedFacesByPlane()
    {
        // Issue #1035: a small piece of a curved face lies on one side of the plane while the middle of its surface
        // lies on the other
        var spine = Edge.MakeLine((0, -2, 0), (0, 2, 0));
        double[] sizes = [1, 2, 2, 1], twists = [10, 0, 0, -10], positions = [0, 0.33, 0.66, 1];
        var sections = Enumerable.Range(0, 4)
            .Select(i => new Plane(spine ^ positions[i]) * new Rotation(0, 0, twists[i]) * Wire.MakeRect(sizes[i], sizes[i]))
            .ToList();
        var solid = Solid.MakeLoft(sections);
        var edge = solid.Edges().GroupBy(e => e.Length, tolDigits: 6).Last.SortBy(Axis.X)[^2..].SortBy(Axis.Z).Last;
        var faces = solid.Faces().Where(f => f.Edges().Any(e => edge == e)).ToList();
        Assert.Equal(2, faces.Count);

        foreach (var uValue in new[] { 0.0, 0.4 })
        {
            var plane = new Plane(edge ^ uValue);
            foreach (var face in faces)
            {
                var (tops, bottoms) = face.SplitBoth(plane);
                var top = Assert.IsType<Face>(Assert.Single(tops));
                var bottom = Assert.IsType<Face>(Assert.Single(bottoms));
                Check.AlmostEqual(face.Area, top.Area + bottom.Area, 5);
                IEnumerable<double> Heights(Shape piece) => piece.Vertices().Select(v => plane.ToLocalCoords(new Vector(v)).Z);
                Assert.True(Heights(top).Min() > -1e-6);
                Assert.True(Heights(bottom).Max() < 1e-6);
            }
        }
    }

    [Fact]
    public void SplitByNonPlanarFace()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var tool = Wire.MakeCircle(1);
        var toolShell = Shell.Extrude(tool, new Vector(0, 0, 1));
        var (top, bottom) = box.SplitBoth(toolShell);

        Assert.NotEmpty(top);
        Assert.NotEmpty(bottom);
        Assert.True(top.Sum(s => s.Volume) > bottom.Sum(s => s.Volume));
    }

    [Fact]
    public void SplitByShell()
    {
        var box = Solid.MakeBox(5, 5, 1);
        var tool = Wire.MakeRect(4, 4);
        var toolShell = Shell.Extrude(tool, new Vector(0, 0, 1));
        var split = box.Split(toolShell, Keep.Top);
        const double innerVol = 2 * 2;
        const double outerVol = 5 * 5;
        Check.AlmostEqual(outerVol - innerVol, split.Sum(s => s.Volume));
    }

    [Fact]
    public void SplitKeepAll()
    {
        var shape = Objects.Box(1, 1, 1);
        var splitShape = shape.Split(Plane.XY, Keep.All);
        Assert.Equal(2, splitShape.Count);
    }

    [Fact]
    public void SplitEdgeByShell()
    {
        var edge = Edge.MakeLine((-5, 0, 0), (5, 0, 0));
        var tool = Wire.MakeRect(4, 4);
        var toolShell = Shell.Extrude(tool, new Vector(0, 0, 1));
        var top = edge.Split(toolShell, Keep.Top);
        Assert.Equal(2, top.Count);
        Check.AlmostEqual(3, ((Edge)top[0]).Length, 5);
    }

    [Fact]
    public void SplitInvalidKeep()
    {
        Assert.Throws<ArgumentException>(() => Objects.Box(1, 1, 1).Split(Plane.XY, Keep.Inside));
        Assert.Throws<ArgumentException>(() => Objects.Box(1, 1, 1).Split(Plane.XY, Keep.Outside));
    }

    [Fact]
    public void Distance()
    {
        var sphere1 = Solid.MakeSphere(1, new Plane((-5, 0, 0)));
        var sphere2 = Solid.MakeSphere(1, new Plane((5, 0, 0)));
        Check.AlmostEqual(8, sphere1.Distance(sphere2), 5);
    }

    [Fact]
    public void Distances()
    {
        var sphere1 = Solid.MakeSphere(1, new Plane((-5, 0, 0)));
        var sphere2 = Solid.MakeSphere(1, new Plane((5, 0, 0)));
        var sphere3 = Solid.MakeSphere(1, new Plane((-5, 0, 5)));
        double[] distances = [8, 3];
        var found = sphere1.Distances(sphere2, sphere3).ToList();
        Assert.Equal(2, found.Count);
        for (int i = 0; i < found.Count; i++)
            Check.AlmostEqual(distances[i], found[i], 5);
    }

    [Fact]
    public void MaxFillet()
    {
        Solid[] testSolids = [Solid.MakeBox(10, 8, 2), Solid.MakeCone(5, 3, 8)];
        double[] maxValues = [0.96, 3.84];
        for (int i = 0; i < testSolids.Length; i++)
        {
            var max = testSolids[i].MaxFillet(testSolids[i].Edges());
            Check.AlmostEqual(maxValues[i], max, 2);
        }
        Assert.Throws<InvalidOperationException>(
            () => testSolids[0].MaxFillet(testSolids[0].Edges(), tolerance: 1e-6, maxIterations: 1));
        Assert.Throws<ArgumentException>(() =>
        {
            var box = Solid.MakeBox(1, 1, 1);
            box.Fillet(0.75, box.Edges());
        });
    }

    [Fact]
    public void LocateBb()
    {
        var boundingBox = Solid.MakeCone(1, 2, 1).BoundingBox();
        var relocatedBoundingBox = Plane.XZ.FromLocalCoords(boundingBox);
        Check.AlmostEqual(-2, relocatedBoundingBox.Min.X, 5);
        Check.AlmostEqual(2, relocatedBoundingBox.Max.X, 5);
        Check.AlmostEqual(0, relocatedBoundingBox.Min.Y, 5);
        Check.AlmostEqual(-1, relocatedBoundingBox.Max.Y, 5);
        Check.AlmostEqual(-2, relocatedBoundingBox.Min.Z, 5);
        Check.AlmostEqual(2, relocatedBoundingBox.Max.Z, 5);
    }

    [Fact]
    public void LocatedVertex()
    {
        // Located() replaces a vertex's location rather than adding to it
        var moved = new Vertex(0, 0, 0).Moved(new Pos(-2.5, 0, 0));
        Check.AlmostEqual(1, moved.Located(new Pos(1, 0, 0)).X, 5);

        var extracted = new Compound([moved]).Vertices()[0];
        Check.AlmostEqual(1, extracted.Located(new Pos(1, 0, 0)).X, 5);

        var box = Solid.MakeBox(1, 1, 1).Moved(new Pos(-2.5, 0, 0));
        Check.AlmostEqual(1, box.Located(new Pos(1, 0, 0)).BoundingBox().Min.X, 5);
    }

    [Fact]
    public void IsEqual()
    {
        var box = Solid.MakeBox(1, 1, 1);
        Assert.True(box.IsEqual(box));
    }

    [Fact]
    public void Equal()
    {
        var box = Solid.MakeBox(1, 1, 1);
        Assert.Equal(box, box);
    }

    [Fact]
    public void NotEqual()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var diff = Solid.MakeBox(1, 2, 3);
        Assert.NotEqual(box, diff);
    }

    [Fact]
    public void Tessellate()
    {
        var box123 = Solid.MakeBox(1, 2, 3);
        var (verts, triangles) = box123.Tessellate(1e-6);
        Assert.Equal(24, verts.Count);
        Assert.Equal(12, triangles.Count);
    }

    [Fact]
    public void Transformed()
    {
        // Validate that transformed works the same as changing location
        var random = new Random(1);
        double Uniform(double a, double b) => a + (b - a) * random.NextDouble();
        var rotation = new Vector(Uniform(0, 360), Uniform(0, 360), Uniform(0, 360));
        var offset = new Vector(Uniform(0, 50), Uniform(0, 50), Uniform(0, 50));
        var shape = Solid.MakeBox(1, 1, 1).Transformed(rotation, offset);
        var predictedLocation = new Location(offset) * new Rotation(rotation);
        var locatedShape = Solid.MakeBox(1, 1, 1).Located(predictedLocation);
        var intersect = shape.Intersect(locatedShape);
        var volume = intersect.Solids().Sum(s => s.Volume);
        Check.AlmostEqual(1, volume, 5);
    }

    [Fact]
    public void PositionAndOrientation()
    {
        var box = Solid.MakeBox(1, 1, 1).Located(new Location((1, 2, 3), (10, 20, 30)));
        Check.AlmostEqual((1, 2, 3), box.Position, 5);
        Check.AlmostEqual((10, 20, 30), box.Orientation, 5);
    }

    [Fact]
    public void DistanceToWithClosestPoints()
    {
        var s0 = Solid.MakeSphere(1).Located(new Location((0, 2.1, 0)));
        var s1 = Solid.MakeSphere(1);
        var (distance, pnt0, pnt1) = s0.DistanceToWithClosestPoints(s1);
        Check.AlmostEqual(0.1, distance, 5);
        Check.AlmostEqual((0, 1.1, 0), pnt0, 5);
        Check.AlmostEqual((0, 1, 0), pnt1, 5);
    }

    [Fact]
    public void ClosestPoints()
    {
        var c0 = Edge.MakeCircle(1).Located(new Location((0, 2.1, 0)));
        var c1 = Edge.MakeCircle(1);
        var closest = c0.ClosestPoints(c1);
        Check.AlmostEqual(c0.PositionAt(0.75), closest.OnThis, 5);
        Check.AlmostEqual(c1.PositionAt(0.25), closest.OnOther, 5);
    }

    [Fact]
    public void DistanceTo()
    {
        var c0 = Edge.MakeCircle(1).Located(new Location((0, 2.1, 0)));
        var c1 = Edge.MakeCircle(1);
        Check.AlmostEqual(0.1, c0.DistanceTo(c1), 5);
    }

    [Fact]
    public void Intersection()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var intersections = box.Intersect(new Axis((0.5, 0.5, 4), (0, 0, -1))).Vertices().SortBy(Axis.Z);
        Check.AlmostEqual((0.5, 0.5, 0), new Vector(intersections[0]), 5);
        Check.AlmostEqual((0.5, 0.5, 1), new Vector(intersections[1]), 5);
    }

    [Fact]
    public void BooleanZeroIntersection()
    {
        var box = Solid.MakeBox(1, 2, 3);
        var zero = new Solid();

        Assert.Empty(box.Intersect(zero));
        Assert.Empty(zero.Intersect(box));
    }

    [Fact]
    public void CleanError()
    {
        // Here to alert build123d to changes in bad OCCT clean behaviour with spheres or hemispheres. The extra edge
        // in a sphere seems to be the cause of this.
        var sphere = Solid.MakeSphere(1);
        var divider = Solid.MakeBox(0.1, 3, 3, new Plane((-0.05, -1.5, -1.5)));
        var halves = sphere.Cut(divider).Solids().Select(s => s.Clean()).ToList();
        Assert.Equal(2, halves.Count);
        Assert.True(Math.Abs(halves[0].Volume - halves[1].Volume) > 0);
    }

    [Fact]
    public void CleanEmpty()
    {
        var obj = new Solid();
        Assert.Same(obj, obj.Clean());
    }

    [Fact]
    public void Vertex_()
    {
        var v = Edge.MakeCircle(1).SingleVertex();
        Assert.IsType<Vertex>(v);
        var e = Assert.Throws<InvalidOperationException>(() => Wire.MakeRect(1, 1).SingleVertex());
        Assert.Contains("Expected exactly one vertex", e.Message);
    }

    [Fact]
    public void Edge_()
    {
        var edge = Edge.MakeCircle(1).SingleEdge();
        Assert.IsType<Edge>(edge);
        var e = Assert.Throws<InvalidOperationException>(() => Wire.MakeRect(1, 1).SingleEdge());
        Assert.Contains("Expected exactly one edge", e.Message);
    }

    [Fact]
    public void Wire_()
    {
        var w = Wire.MakeCircle(1).SingleWire();
        Assert.IsType<Wire>(w);
        var e = Assert.Throws<InvalidOperationException>(() => Solid.MakeBox(1, 1, 1).SingleWire());
        Assert.Contains("Expected exactly one wire", e.Message);
    }

    [Fact]
    public void Face_()
    {
        var f = Face.MakeRect(1, 1);
        Assert.IsType<Face>(f);
        var e = Assert.Throws<InvalidOperationException>(() => Solid.MakeBox(1, 1, 1).SingleFace());
        Assert.Contains("Expected exactly one face", e.Message);
    }

    [Fact]
    public void Shell_()
    {
        // The error case extrudes text, which isn't ported yet
        var s = Solid.MakeSphere(1).SingleShell();
        Assert.IsType<Shell>(s);
    }

    [Fact]
    public void Solid_()
    {
        var s = Solid.MakeSphere(1).SingleSolid();
        Assert.IsType<Solid>(s);
        var (top, bottom) = Solid.MakeSphere(1).SplitBoth(Plane.XY);
        var e = Assert.Throws<InvalidOperationException>(() => new Compound([.. top, .. bottom]).SingleSolid());
        Assert.Contains("Expected exactly one solid", e.Message);
    }

    [Fact]
    public void Manifold()
    {
        Assert.True(Solid.MakeBox(1, 1, 1).IsManifold);
        Assert.True(Solid.MakeBox(1, 1, 1).SingleShell().IsManifold);
        Assert.False(Solid.MakeBox(1, 1, 1).SingleShell().Cut(Solid.MakeBox(0.5, 0.5, 0.5)).IsManifold);
        Assert.True(Compound.Assembly([Solid.MakeBox(1, 1, 1), Solid.MakeCylinder(1, 1)]).IsManifold);
    }

    [Fact]
    public void ManifoldDegenerateEdges()
    {
        // The poles of a sphere and the apex of a cone are edges of one face
        var box = Solid.MakeBox(4, 4, 4);
        Assert.True(Solid.MakeSphere(1).IsManifold);
        Assert.True(Solid.MakeCone(2, 0, 3).IsManifold);
        Assert.True(box.Fillet(0.5, box.Edges()).IsManifold);
        Check.AlmostEqual(Solid.MakeSphere(1).Volume, Solid.MakeSphere(1).SingleShell().Volume, 5);
    }

    [Fact]
    public void ManifoldInconsistentOrientation()
    {
        // Issue #855: a sweep that folds over itself has two faces on every edge, but they are not consistently
        // oriented
        var path = Wire.MakePolygon([(0, 20, 0), (20, 20, 0), (20, 0, 0)], close: false);
        var start = new Plane((0, 20, 0), zDir: (1, 0, 0));
        Shape folded;
        IReadOnlyList<string> warnings;
        using (var scope = Warnings.Capture())
        {
            folded = Solid.Sweep(start * Face.MakeRect(40, 40), path, transition: Transition.Round);
            warnings = scope.Messages;
        }
        Assert.Contains(warnings, w => w.Contains("invalid solid"));
        Assert.False(folded.IsValid);
        Assert.False(folded.IsManifold);
        var unfolded = Solid.Sweep(start * Face.MakeRect(30, 30), path, transition: Transition.Round);
        Assert.True(unfolded.IsValid);
        Assert.True(unfolded.IsManifold);
    }

    [Fact]
    public void InheritColor()
    {
        // Create some objects and assign colours to them
        var b = Objects.Box(1, 1, 1).Located(new Pos(2, 2, 0)).WithColor(new Color("blue"));
        var c = Objects.Cylinder(1, 1).Located(new Pos(-2, 2, 0)).WithColor(new Color("red"));
        var a = Compound.Assembly([b, c]).WithColor(new Color(0, 1, 0));
        // Check that assigned colours stay
        Check.TupleAlmostEqual([0, 1, 0, 1], Check.Of(a.Color!.Value), 5);
        Check.TupleAlmostEqual([0, 0, 1, 1], Check.Of(b.Color!.Value), 5);
        Check.TupleAlmostEqual([1, 0, 0, 1], Check.Of(c.Color!.Value), 5);
    }

    [Fact]
    public void OcpSection()
    {
        // Vertex
        var (verts, edges) = new Vertex(1, 2, 0).OcpSection(new Vertex(1, 2, 0));
        Assert.Single(verts);
        Assert.Empty(edges);
        Check.AlmostEqual((1, 2, 0), new Vector(verts[0]), 5);

        (verts, edges) = new Vertex(1, 2, 0).OcpSection(Edge.MakeLine((0, 0), (2, 4)));
        Assert.Single(verts);
        Assert.Empty(edges);
        Check.AlmostEqual((1, 2, 0), new Vector(verts[0]), 5);

        (verts, edges) = new Vertex(1, 2, 0).OcpSection(Face.MakeRect(5, 5));
        Check.AlmostEqual((1, 2, 0), new Vector(verts[0]), 5);
        Assert.Empty(edges);

        (verts, edges) = new Vertex(1, 2, 0).OcpSection(Face.FromPlane(Plane.XY));
        Check.AlmostEqual((1, 2, 0), new Vector(verts[0]), 5);
        Assert.Empty(edges);

        var cylinder = Face.Extrude(Edge.MakeCircle(5, Plane.XY.Offset(-10)), (0, 0, 20));
        var cylinder2 = Face.Extrude(Edge.MakeCircle(5, Plane.YZ.Offset(-10)), (20, 0, 0));
        var pln = Plane.XY;

        var vEdge = Edge.MakeLine((-5, 0, -20), (-5, 0, 20));
        var (vertices1, edges1) = cylinder.OcpSection(vEdge);
        vertices1 = vertices1.SortBy(Axis.Z);
        Assert.Equal(2, vertices1.Count);

        Check.AlmostEqual((-5, 0, -10), new Vector(vertices1[0]), 5);
        Check.AlmostEqual((-5, 0, 10), new Vector(vertices1[1]), 5);
        Assert.Single(edges1);
        Check.AlmostEqual(20, edges1[0].Length, 5);

        var (vertices2, edges2) = cylinder.OcpSection(Face.FromPlane(pln));
        Assert.Single(vertices2);
        Assert.Single(edges2);
        Check.AlmostEqual((5, 0, 0), new Vector(vertices2[0]), 5);
        Assert.Equal(GeomType.Circle, edges2[0].GeomType);
        Check.AlmostEqual(5, edges2[0].Radius, 5);

        var (_, edges4) = cylinder2.OcpSection(cylinder);
        Assert.True(edges4.Count >= 2);
        Assert.All(edges4, e => Assert.Equal(GeomType.Ellipse, e.GeomType));

        var cylinder3 = Objects.Cylinder(5, 20).SingleSolid();
        var cylinder4 = new Rotation(0, 90, 0) * cylinder3;

        var (_, edges5) = cylinder3.OcpSection(cylinder4);
        Assert.True(edges5.Count >= 2);
        Assert.All(edges5, e => Assert.Equal(GeomType.Ellipse, e.GeomType));
    }

    [Fact]
    public void EmptyShape()
    {
        var empty = new Solid();
        var box = Solid.MakeBox(1, 1, 1);
        Assert.Throws<InvalidOperationException>(() => empty.Location);
        Assert.Throws<InvalidOperationException>(() => empty.Position);
        Assert.Throws<InvalidOperationException>(() => empty.Orientation);
        Assert.False(empty.IsManifold);
        Assert.Throws<InvalidOperationException>(() => empty.GeomType);
        Assert.Same(empty, empty.Fix());
        Assert.Equal(new Solid().GetHashCode(), empty.GetHashCode());
        Assert.False(empty.IsSame(new Solid()));
        Assert.False(empty.IsEqual(new Solid()));
        Assert.True(empty.IsValid);
        var emptyBbox = empty.BoundingBox();
        Assert.Equal((0.0, 0.0, 0.0), (emptyBbox.Size.X, emptyBbox.Size.Y, emptyBbox.Size.Z));
        Assert.Same(empty, empty.Mirror(Plane.XY));
        Check.AlmostEqual(0, empty.ComputeVolume(), 5);
        Assert.Empty(empty.Entities(Build123d.ShapeType.Face));
        Assert.Equal(0, empty.Area);
        Assert.Same(empty, empty.Rotate(Axis.Z, 90));
        var translateMatrix = new Matrix(new double[,]
        {
            { 1.0, 0.0, 0.0, 1.0 },
            { 0.0, 1.0, 0.0, 2.0 },
            { 0.0, 0.0, 1.0, 3.0 },
            { 0.0, 0.0, 0.0, 1.0 },
        });
        Assert.Same(empty, empty.TransformShape(translateMatrix));
        Assert.Same(empty, empty.TransformGeometry(translateMatrix));
        // Nothing is nothing wherever it is put
        Assert.True(empty.Located(new Location((1, 2, 3))).IsEmpty);
        Assert.True(empty.Moved(new Location((1, 2, 3))).IsEmpty);
        Assert.Throws<InvalidOperationException>(() => empty.DistanceTo(new Vector(1, 1, 1)));
        Assert.Throws<InvalidOperationException>(() => empty.DistanceToWithClosestPoints(new Vector(1, 1, 1)));
        var (sectionVertices, sectionEdges) = empty.OcpSection(new Vertex(1, 1, 1));
        Assert.Empty(sectionVertices);
        Assert.Empty(sectionEdges);
        Assert.Empty(empty.FacesIntersectedByAxis(Axis.Z));
        Assert.Throws<InvalidOperationException>(() => empty.Distance(new Vertex(1, 1, 1)));
        Assert.Throws<InvalidOperationException>(() => empty.Distances(new Vertex(0, 0, 0), new Vertex(1, 1, 1)).ToList());
        Assert.Throws<InvalidOperationException>(() => box.Distances(empty, new Vertex(1, 1, 1)).ToList());
        Assert.Throws<InvalidOperationException>(() => empty.Meshed(0.001));
        Assert.Throws<InvalidOperationException>(() => empty.Tessellate(0.001));
        Assert.Throws<InvalidOperationException>(() => empty.ToSplines());
        var emptyWire = new Wire();
        Assert.Throws<ArgumentException>(() => box.Vertices().GroupBy(emptyWire));
        Assert.Throws<ArgumentException>(() => box.Vertices().SortBy(emptyWire));
    }

    [Fact]
    public void EmptySelectors()
    {
        Assert.Empty(new Vertex(1, 1, 1).Edges());
        Assert.Empty(new Vertex(1, 1, 1).Wires());
        Assert.Empty(new Vertex(1, 1, 1).Faces());
        Assert.Empty(new Vertex(1, 1, 1).Shells());
        Assert.Empty(new Vertex(1, 1, 1).Solids());
        Assert.Empty(new Vertex(1, 1, 1).Compounds());
        (Func<Shape> Single, string Name)[] cases =
        [
            (() => new Vertex(1, 1, 1).SingleEdge(), "edge"),
            (() => new Vertex(1, 1, 1).SingleWire(), "wire"),
            (() => new Vertex(1, 1, 1).SingleFace(), "face"),
            (() => new Vertex(1, 1, 1).SingleShell(), "shell"),
            (() => new Vertex(1, 1, 1).SingleSolid(), "solid"),
            (() => new Vertex(1, 1, 1).SingleCompound(), "compound"),
        ];
        foreach (var (single, name) in cases)
        {
            var e = Assert.Throws<InvalidOperationException>(() => single());
            Assert.Contains($"Expected exactly one {name}", e.Message);
        }
    }

    [Fact]
    public void Rotate()
    {
        var line = Edge.MakeLine((0, 0), (1, 0));
        var rotatedLine = line.Rotate(new Axis((1, 0, 0), (0, 0, 1)), 45);
        var root2O2 = Math.Sqrt(2) / 2;
        Check.AlmostEqual((1 - root2O2, -root2O2, 0), rotatedLine.PositionAt(0));
        Check.AlmostEqual((1, 0, 0), rotatedLine.PositionAt(1));
        Assert.True(line.Wrapped.IsPartner(rotatedLine.Wrapped));

        rotatedLine = line.Rotate(new Axis((1, 0, 0), (0, 0, 1)), 45, transform: true);
        Check.AlmostEqual((1 - root2O2, -root2O2, 0), rotatedLine.PositionAt(0));
        Check.AlmostEqual((1, 0, 0), rotatedLine.PositionAt(1));
        Assert.False(line.Wrapped.IsPartner(rotatedLine.Wrapped));

        var rotatedEmpty = new Edge().Rotate(new Axis((1, 0, 0), (0, 0, 1)), 45);
        Assert.True(rotatedEmpty.IsEmpty);
    }

    [Fact]
    public void Translate()
    {
        var line = Edge.MakeLine((0, 0), (1, 0));
        var translatedLine = line.Translate((0, 1, 0));
        Check.AlmostEqual((0, 1, 0), translatedLine.PositionAt(0));
        Check.AlmostEqual((1, 1, 0), translatedLine.PositionAt(1));
        Assert.True(line.Wrapped.IsPartner(translatedLine.Wrapped));

        translatedLine = line.Translate((0, 1, 0), transform: true);
        Check.AlmostEqual((0, 1, 0), translatedLine.PositionAt(0));
        Check.AlmostEqual((1, 1, 0), translatedLine.PositionAt(1));
        Assert.False(line.Wrapped.IsPartner(translatedLine.Wrapped));

        var translatedEmpty = new Edge().Translate((0, 1, 0));
        Assert.True(translatedEmpty.IsEmpty);
    }
}

public class ShapeCastTests
{
    // build123d's Box is a Part; here the compound sample is a Compound around the box solid
    private readonly Solid _box = Objects.Box(1, 1, 1);

    [Fact]
    public void EveryTopologyTypeRoundTrips()
    {
        (Type Expected, Shape Shape)[] samples =
        [
            (typeof(Vertex), _box.Vertices()[0]),
            (typeof(Edge), _box.Edges()[0]),
            (typeof(Face), _box.Faces()[0]),
            (typeof(Shell), _box.Shells()[0]),
            (typeof(Solid), _box.Solids()[0]),
            (typeof(Compound), new Compound([_box])),
        ];
        foreach (var (expected, shape) in samples)
            Assert.IsType(expected, Shape.Cast(shape.Wrapped));
    }

    [Fact]
    public void CastIsTheSameFromAnyClass()
    {
        // Cast is static on Shape, so it is the same through every class
        var solid = _box.Solids()[0].Wrapped;
        Assert.IsType<Solid>(Vertex.Cast(solid));
        Assert.IsType<Solid>(Edge.Cast(solid));
        Assert.IsType<Solid>(Face.Cast(solid));
        Assert.IsType<Solid>(Shell.Cast(solid));
        Assert.IsType<Solid>(Solid.Cast(solid));
        Assert.IsType<Solid>(Compound.Cast(solid));
        Assert.IsType<Solid>(Shape.Cast(solid));
    }
}

public class TransformShapeTests
{
    // TransformShape is the type-preserving counterpart to TransformGeometry

    [Fact]
    public void RigidTransformKeepsTypeAndSize()
    {
        var box = Solid.MakeBox(1, 2, 3);
        var matrix = new Matrix().Rotated(Axis.Z, 90);
        var rotated = box.TransformShape(matrix);

        Assert.IsType<Solid>(rotated);
        Check.AlmostEqual(box.Volume, rotated.Volume, 6);
        Check.AlmostEqual(2, rotated.BoundingBox().Size.X, 6);
        Check.AlmostEqual(1, rotated.BoundingBox().Size.Y, 6);
    }

    [Fact]
    public void OriginalIsLeftAlone()
    {
        var box = Solid.MakeBox(1, 2, 3);
        var matrix = new Matrix().Rotated(Axis.Z, 90);
        var rotated = box.TransformShape(matrix);

        Assert.NotSame(box, rotated);
        Check.AlmostEqual(1, box.BoundingBox().Size.X, 6);
    }
}

public class ShapeAlgebraEdgesTests
{
    [Fact]
    public void AddNoneReturnsSelf()
    {
        // Compound overrides Add, so exercise the Shape implementation with a shape that does not: a bare Solid
        var solid = Solid.MakeBox(1, 1, 1);
        Assert.Same(solid, solid + (Shape?)null);
    }

    [Fact]
    public void IntersectWithNoArguments() => Assert.Empty(Objects.Box(1, 1, 1).Intersect());

    [Fact]
    public void IntersectYieldingDisjointPieces()
    {
        // A bar crossing two separated blocks intersects in two solids, which come back as a single composite
        // rather than a list
        var bar = Objects.Box(20, 1, 1);
        var blocks = new Pos(-6, 0, 0) * Objects.Box(2, 2, 2) + new Pos(6, 0, 0) * Objects.Box(2, 2, 2);
        var result = bar & blocks;
        Assert.Equal(2, result.Solids().Count);
        Check.AlmostEqual(4, result.Volume, 5);
    }
}

public class AsShapeTests
{
    // Geometry objects convert to their shape equivalents

    [Fact]
    public void GeometryTypesConvert()
    {
        Assert.IsType<Vertex>(Shape.AsShape(new Vector(1, 2, 3)));
        Assert.IsType<Vertex>(Shape.AsShape(new Location((1, 2, 3))));
        Assert.IsType<Edge>(Shape.AsShape(Axis.X));
        Assert.IsType<Face>(Shape.AsShape(Plane.XY));
    }

    [Fact]
    public void LocationSubclassesConvertLikeLocation()
    {
        // Pos and Rotation derive from Location, so an exact-type lookup would miss them
        Assert.IsType<Vertex>(Shape.AsShape(new Pos(1, 2, 3)));
        Assert.IsType<Vertex>(Shape.AsShape(new Rotation(0, 0, 45)));
        var center = Shape.AsShape(new Pos(1, 2, 3)).Center();
        Assert.Equal((1.0, 2.0, 3.0), (center.X, center.Y, center.Z));
    }

    [Fact]
    public void ShapesPassThroughUnchanged()
    {
        var edge = Edge.MakeLine((0, 0, 0), (1, 0, 0));
        Assert.Same(edge, Shape.AsShape(edge));
    }
}
