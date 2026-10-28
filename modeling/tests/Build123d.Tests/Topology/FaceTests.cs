// Ported from build123d tests/test_direct_api/test_face.py.
// Omitted:
// - test_project_to_face_with_compound_boolean_result: patches an internal OCCT boolean (mock).
// - test_fillet_2d_mixed_profile_case, _regression, test_fillet_2d_holes_regression,
//   test_fillet_geom2dgcc_circ2d2tanrad_algorithm, TestFillet2DNoVertices: Face.fillet_2d is not yet ported
//   (and they use Sketch/Ellipse/BuildSketch).
// - test_length_width_of_a_cylinder, _do_not_depend_on_orientation, test_length_follows_a_curved_trim: length and
//   width of a cylindrical face are not yet ported.
// - test_uv_face, test_wrap, test_wrap_across_the_seam, test_wrap_faces: uv_face and wrap are not yet ported.
// - test_make_gordon_surface, test_make_gordon_surface_input_types: make_gordon_surface is not yet ported.
// - test_import_stl: STL import/export is not ported.
// - test_is_convex_concave: needs BuildPart, offset() and fillet().
// - test_constructor (Face(bob="fred")) and test_surface_from_curves_type_checks: argument-type checks the
//   compiler rules out; likewise the bad-argument calls normal_at(0), location_at(0), normal_at(center=...),
//   location_at(center=...) and make_surface(<a single Edge>).
// - The first of the two test_location_at methods: Python shadows it with the second, so it never runs (and it
//   fails there: the bottom face's orientation is (180, 0, 0), not (-180, 0, -180)).
// - TestAxesOfSysmmetrySplitNone, test_make_surface_rejects_an_invalid_result,
//   test_surface_from_points_reports_a_failed_approximation, test_sew_faces_reports_an_unexpected_result,
//   TestSurfaceHoles: monkey-patch or mock OCCT/build123d internals.
// - test_a_face_has_no_mass_or_volume: mass() is not yet ported; only the volume is checked.
// Translated: builder-made objects become direct-API shapes (Box, Cylinder, Cone, Sphere, Torus, Rectangle, Circle,
// Ellipse, Triangle, RegularPolygon, Polyline, Spline, JernArc, PolarLocations, BuildSketch/make_face in
// test_make_holes and test_axes_of_symmetry, the RegularPolygon prism in test_geometry). test_axes_of_symmetry checks
// the number of axes only, as the Python's other two assertions are on generators and always pass.
// test_wire is SingleWire (the C# name of wire(); a method can't be named after the Wire type it uses).
// test_zero_offset_returns_a_copy doesn't check identity: shapes are immutable, so returning the same face is fine.
using Build123d.Occt;

namespace Build123d.Tests.Topology;

/// <summary>Direct-API stand-ins for build123d's centred objects.</summary>
file static class Obj
{
    public static Solid Box(double l, double w, double h) => Solid.MakeBox(l, w, h, new Plane((-l / 2, -w / 2, -h / 2)));
    public static Solid Cylinder(double r, double h) => Solid.MakeCylinder(r, h, new Plane((0, 0, -h / 2)));
    public static Solid Cone(double bottom, double top, double h) => Solid.MakeCone(bottom, top, h, new Plane((0, 0, -h / 2)));
    public static Face Circle(double r) => new(Wire.MakeCircle(r));
    public static Face Rectangle(double w, double h) => Face.MakeRect(w, h);

    /// <summary>PolarLocations(radius, count): evenly spaced round a circle, each rotated to face outward.</summary>
    public static IEnumerable<Location> PolarLocations(double radius, int count) =>
        Enumerable.Range(0, count).Select(i =>
        {
            var angle = i * 360.0 / count;
            var a = angle * Math.PI / 180;
            return new Location((radius * Math.Cos(a), radius * Math.Sin(a), 0), (0, 0, angle));
        });
}

public class FaceTests
{
    [Fact]
    public void MakeSurfaceFromCurves()
    {
        var bottomEdge = Edge.MakeCircle(1, endAngle: 90);
        var topEdge = Edge.MakeCircle(1, new Plane((0, 0, 1)), endAngle: 90);
        var curved = Face.MakeSurfaceFromCurves(bottomEdge, topEdge);
        Assert.True(curved.IsValid);
        Check.AlmostEqual(Math.PI / 2, curved.Area, 5);
        Check.AlmostEqual((Math.Sqrt(2) / 2, Math.Sqrt(2) / 2, 0), curved.NormalAt(), 5);

        var bottomWire = Wire.MakeCircle(1);
        var topWire = Wire.MakeCircle(1, new Plane((0, 0, 1)));
        var curvedShape = Face.MakeSurfaceFromCurves(bottomWire, topWire);
        Assert.True(curvedShape.IsValid);
        Check.AlmostEqual(2 * Math.PI, curvedShape.Area, 5);
    }

    [Fact]
    public void Center()
    {
        var testFace = new Face(Wire.MakePolygon([(0, 0), (1, 0), (1, 1), (0, 0)]));
        Check.AlmostEqual((2.0 / 3, 1.0 / 3, 0), testFace.Center(CenterOf.Mass), 1);
        Check.AlmostEqual((0.5, 0.5, 0), testFace.Center(CenterOf.BoundingBox), 5);
    }

    [Fact]
    public void FaceVolume() => Check.AlmostEqual(0, Face.MakeRect(1, 1).Volume, 5);

    static double AreaOf(IEnumerable<Face> faces) => faces.Sum(f => f.Area);

    [Fact]
    public void SplitByPerimeter()
    {
        // Test 0 - extract a spherical cap from a Face
        var sphere = Solid.MakeSphere(10).Rotate(Axis.Z, 90);
        var target0 = sphere.Faces()[0];
        var circle = Plane.YZ.Offset(15) * Obj.Circle(5);
        var circleProjected = circle.ProjectToShape(sphere, (-1, 0, 0))[0];
        var circleOuterwire = circleProjected.SingleEdge();
        var (inside0, outside0) = target0.SplitByPerimeter(circleOuterwire);
        Assert.True(AreaOf(inside0) < AreaOf(outside0));

        // Test 1 - extract ring of a sphere from a Face
        var ring = new Pos(z: 15) * (Obj.Circle(5) - Obj.Circle(3)).SingleFace();
        var ringProjected = (Face)ring.ProjectToShape(sphere, (0, 0, -1))[0];
        var ringOuterwire = ringProjected.OuterWire();
        var (inside1, outside1) = target0.SplitByPerimeter(ringOuterwire);
        Assert.True(AreaOf(inside1) < AreaOf(outside1));
        Assert.Equal(2, outside1.Count);

        // Test 2 - extract multiple faces from a Shell
        var target2 = Obj.Box(1, 10, 10).SingleShell();
        var square = Face.MakeRect(3, 3, new Plane((12, 0, 0), zDir: (1, 0, 0)));
        var squareProjected = (Face)square.ProjectToShape(target2, (-1, 0, 0))[0];
        var outside2 = target2.SplitByPerimeter(squareProjected.OuterWire(), Keep.Outside);
        Assert.IsType<Shell>(outside2);
        var inside2 = target2.SplitByPerimeter(squareProjected.OuterWire(), Keep.Inside);
        Assert.IsType<Face>(inside2);

        // Test 3 - split a spherical face with a single edge crossing the seam
        sphere = Solid.MakeSphere(10);
        var target3 = sphere.SingleFace();
        circle = Plane.YZ.Offset(15) * Obj.Circle(5);
        var projectedWire = new Wire(circle.ProjectToShape(sphere, (-1, 0, 0))[0].Edges().GroupBy(Axis.X)[0]);
        var perimeter = Edge.MakeCircle(projectedWire.Edges()[0].Radius,
                                        new Plane(projectedWire.Edges()[0].ArcCenter, zDir: (1, 0, 0)));

        Assert.True(target3.Seams[0].DistanceTo(perimeter) < 1e-5);
        var (inside3, outside3) = target3.SplitByPerimeter(perimeter);
        Assert.NotEmpty(inside3);
        Assert.NotEmpty(outside3);
        Assert.True(AreaOf(inside3) < AreaOf(outside3));
        Check.AlmostEqual(target3.Area, AreaOf(inside3) + AreaOf(outside3), 3);

        // Test 4 - invalid inputs
        Assert.Throws<ArgumentException>(() => target2.SplitByPerimeter(Edge.MakeLine((0, 0), (1, 0))));
        Assert.Throws<ArgumentException>(() => target2.SplitByPerimeter(Edge.MakeCircle(1), Keep.Top));
    }

    [Fact]
    public void SplitByPerimeterStandaloneSphericalFaceWithoutSeamCrossing()
    {
        var sphere = Solid.MakeSphere(10).Rotate(Axis.Z, 90);
        var sphericalFace = sphere.Faces()[0];
        var circle = Plane.YZ.Offset(15) * Obj.Circle(5);
        var perimeter = circle.ProjectToShape(sphere, (-1, 0, 0))[0].SingleEdge();

        Assert.True(sphericalFace.Seams[0].DistanceTo(perimeter) > 1);
        var (inside, outside) = sphericalFace.SplitByPerimeter(perimeter);

        Assert.NotEmpty(inside);
        Assert.NotEmpty(outside);
        Assert.True(AreaOf(inside) < AreaOf(outside));
        Check.AlmostEqual(sphericalFace.Area, AreaOf(inside) + AreaOf(outside), 5);
    }

    [Fact]
    public void Chamfer2D()
    {
        var testFace = Face.MakeRect(10, 10);
        testFace = testFace.Chamfer2D(1, 2, testFace.Vertices());
        Check.AlmostEqual(100 - 4 * 0.5 * 1 * 2, testFace.Area);
    }

    [Fact]
    public void Chamfer2DReference()
    {
        var testFace = Face.MakeRect(10, 10);
        var edge = testFace.Edges().SortBy(Axis.Y)[0];
        var vertex = edge.Vertices().SortBy(Axis.X)[0];
        testFace = testFace.Chamfer2D(1, 2, [vertex], edge);
        Check.AlmostEqual(100 - 0.5 * 1 * 2, testFace.Area);
        Check.AlmostEqual(9, testFace.Edges().SortBy(Axis.Y)[0].Length);
        Check.AlmostEqual(8, testFace.Edges().SortBy(Axis.X)[0].Length);
    }

    [Fact]
    public void Chamfer2DReferenceInverted()
    {
        var testFace = Face.MakeRect(10, 10);
        var edge = testFace.Edges().SortBy(Axis.Y)[0];
        var vertex = edge.Vertices().SortBy(Axis.X)[0];
        testFace = testFace.Chamfer2D(2, 1, [vertex], edge);
        Check.AlmostEqual(100 - 0.5 * 1 * 2, testFace.Area);
        Check.AlmostEqual(8, testFace.Edges().SortBy(Axis.Y)[0].Length);
        Check.AlmostEqual(9, testFace.Edges().SortBy(Axis.X)[0].Length);
    }

    [Fact]
    public void Chamfer2DErrorChecking()
    {
        var testFace = Face.MakeRect(10, 10);
        var edge = testFace.Edges().SortBy(Axis.Y)[0];
        var vertex = edge.Vertices().SortBy(Axis.X)[0];
        var otherEdge = testFace.Edges().SortBy(Axis.Y).Last;
        Assert.Throws<ArgumentException>(() => testFace.Chamfer2D(1, 2, [vertex], otherEdge));
    }

    static Face HoleFace() => (Face.MakeRect(20, 20) - Face.MakeRect(5, 5)).SingleFace();

    [Fact]
    public void Chamfer2DWithHole()
    {
        var holeFace = HoleFace();
        var corner = holeFace.Vertices().GroupBy(Axis.Y).Last.SortBy(Axis.X)[0];
        var chamfered = holeFace.Chamfer2D(2, 2, [corner]);
        Assert.True(chamfered.IsValid);
        Assert.Single(chamfered.InnerWires());
        Check.AlmostEqual(375 - 2, chamfered.Area, 5);
    }

    [Fact]
    public void Chamfer2DHoleVertices()
    {
        var holeFace = HoleFace();
        var holeVertices = holeFace.InnerWires()[0].Vertices();
        var chamfered = holeFace.Chamfer2D(1, 1, holeVertices);
        Assert.True(chamfered.IsValid);
        Assert.Single(chamfered.InnerWires());
        Check.AlmostEqual(375 + 4 * 0.5, chamfered.Area, 5);
    }

    [Fact]
    public void Chamfer2DNoVertices()
    {
        var face = Face.MakeRect(20, 20);
        Assert.Same(face, face.Chamfer2D(2, 2, []));
    }

    [Fact]
    public void Chamfer2DReversedFaceKeepsNormal()
    {
        var holeFace = (Face)(-HoleFace());
        var corner = holeFace.Vertices().GroupBy(Axis.Y).Last.SortBy(Axis.X)[0];
        var chamfered = holeFace.Chamfer2D(2, 2, [corner]);
        Check.AlmostEqual(holeFace.NormalAt(), chamfered.NormalAt(), 5);
        Check.AlmostEqual(375 - 2, chamfered.Area, 5);
    }

    [Fact]
    public void PlaneAsFace() => Check.AlmostEqual((0, 0, 1), Face.FromPlane(Plane.XY).NormalAt(), 5);

    [Fact]
    public void LengthWidth()
    {
        var testFace = Face.MakeRect(8, 10, Plane.XZ);
        Check.AlmostEqual(8, testFace.Length!.Value, 5);
        Check.AlmostEqual(10, testFace.Width!.Value, 5);
    }

    [Fact]
    public void LengthIsNoneForOtherSurfaces()
    {
        var sphere = Solid.MakeSphere(5).Faces()[0];
        Assert.Null(sphere.Length);
        Assert.Null(sphere.Width);
    }

    [Fact]
    public void Geometry()
    {
        var box = Solid.MakeBox(1, 1, 2);
        Assert.Equal("SQUARE", box.Faces().SortBy(Axis.Z).Last.Geometry);
        Assert.Equal("RECTANGLE", box.Faces().SortBy(Axis.Y).Last.Geometry);
        // RegularPolygon(1, 3) extruded by 1
        var triangle = Enumerable.Range(0, 3)
            .Select(i => new Vector(Math.Cos(i * 2 * Math.PI / 3) - 0.25, Math.Sin(i * 2 * Math.PI / 3)));
        var prism = Solid.Extrude(new Face(Wire.MakePolygon(triangle)), (0, 0, 1));
        Assert.Equal("POLYGON", prism.Faces().SortBy(Axis.Z).Last.Geometry);
    }

    [Fact]
    public void IsPlanar()
    {
        Assert.NotNull(Face.MakeRect(1, 1).IsPlanar);
        Assert.Null(Solid.MakeCylinder(1, 1).Faces().FilterBy(GeomType.Cylinder)[0].IsPlanar);
        // Some of these faces have geom_type BSPLINE but are planar
        var mount = Solid.MakeLoft(
        [
            Wire.MakeRect(1 + 16 + 4, 20, new Plane((10.5, 0, 0))),
            new Pos(1, 0, 4) * Wire.MakeRect(16, 20, new Plane((8, 0, 0))),
        ]);
        Assert.All(mount.Faces(), f => Assert.NotNull(f.IsPlanar));
    }

    [Fact]
    public void Negate()
    {
        var square = Face.MakeRect(1, 1);
        Check.AlmostEqual((0, 0, 1), square.NormalAt(), 5);
        var flippedSquare = (Face)(-square);
        Check.AlmostEqual((0, 0, -1), flippedSquare.NormalAt(), 5);

        // Ensure the topo_parent is cleared when a face is negated
        // (otherwise the original Rectangle would be the topo_parent)
        var flipped = (Face)(-Obj.Rectangle(34, 10));
        var leftEdge = flipped.Edges().SortBy(Axis.X)[0];
        var parentFace = (Face)leftEdge.TopoParent!;
        Check.AlmostEqual(parentFace.NormalAt(), flipped.NormalAt(), 5);
    }

    [Fact]
    public void Offset()
    {
        var bbox = Face.MakeRect(2, 2, Plane.XY).Offset(5).BoundingBox();
        Check.AlmostEqual((-1, -1, 5), bbox.Min, 5);
        Check.AlmostEqual((1, 1, 5), bbox.Max, 5);
    }

    [Fact]
    public void MakeFromWires()
    {
        var outer = Wire.MakeCircle(10);
        Wire[] inners =
        [
            Wire.MakeCircle(1).Located(new Location((-2, 2, 0))),
            Wire.MakeCircle(1).Located(new Location((2, 2, 0))),
        ];
        var happy = new Face(outer, inners);
        Check.AlmostEqual(Math.PI * (Math.Pow(10, 2) - 2), happy.Area, 5);

        outer = new Wire(Edge.MakeCircle(10, endAngle: 180));
        Assert.Throws<ArgumentException>(() => new Face(outer, inners));
        Assert.Throws<ArgumentException>(() => new Face(Wire.MakeCircle(10, Plane.XZ), inners));

        outer = Wire.MakeCircle(10);
        inners =
        [
            Wire.MakeCircle(1).Located(new Location((-2, 2, 0))),
            new Wire(Edge.MakeCircle(1, endAngle: 180)).Located(new Location((2, 2, 0))),
        ];
        Assert.Throws<ArgumentException>(() => new Face(outer, inners));
    }

    [Fact]
    public void SewFaces()
    {
        var patches = (from x in Enumerable.Range(0, 2)
                       from y in Enumerable.Range(0, 2)
                       from z in Enumerable.Range(0, 3)
                       select Face.MakeRect(1, 1, new Plane((x, y, z)))).ToArray();
        new Random(1).Shuffle(patches);
        var sheets = Face.SewFaces(patches);
        Assert.Equal(3, sheets.Count);
        Assert.Equal(4, sheets[0].Count);
        Assert.IsType<Face>(sheets[0][0]);
    }

    static Vector[][] WavePoints() =>
        [.. Enumerable.Range(0, 11).Select(y => Enumerable.Range(0, 11)
            .Select(x => new Vector(x, y, Math.Cos(Math.PI * x / 10) + Math.Sin(Math.PI * y / 10))).ToArray())];

    [Fact]
    public void SurfaceFromArrayOfPoints()
    {
        var surface = Face.MakeSurfaceFromArrayOfPoints(WavePoints());
        var bbox = surface.BoundingBox();
        Check.AlmostEqual((0, 0, -1), bbox.Min, 3);
        Check.AlmostEqual((10, 10, 2), bbox.Max, 2);
    }

    [Fact]
    public void BezierSurface()
    {
        Vector[][] Grid(int from, int to) =>
            [.. Enumerable.Range(from, to - from).Select(y => Enumerable.Range(from, to - from)
                .Select(x => new Vector(x, y, x == 0 && y == 0 ? 2 : x == 0 || y == 0 ? 1 : 0)).ToArray())];
        var points = Grid(-1, 2);
        var surface = Face.MakeBezierSurface(points);
        var bbox = surface.BoundingBox();
        Check.AlmostEqual((-1, -1, 0), bbox.Min, 3);
        Check.AlmostEqual((1, 1, 1), bbox.Max, 1);
        Assert.True(bbox.Max.Z < 1.0);

        double[][] weights =
        [.. Enumerable.Range(-1, 3).Select(y => Enumerable.Range(-1, 3).Select(x => x == 0 || y == 0 ? 2.0 : 1.0).ToArray())];
        surface = Face.MakeBezierSurface(points, weights);
        bbox = surface.BoundingBox();
        Check.AlmostEqual((-1, -1, 0), bbox.Min, 3);
        Assert.True(bbox.Max.Z > 1.0);

        var tooManyPoints = Grid(-1, 27);
        Assert.Throws<ArgumentException>(() => Face.MakeBezierSurface([[new Vector(0, 0)]]));
        Assert.Throws<ArgumentException>(() => Face.MakeBezierSurface(points, [[1.0, 1.0], [1.0, 1.0]]));
        Assert.Throws<ArgumentException>(() => Face.MakeBezierSurface(tooManyPoints));
    }

    [Fact]
    public void Thicken()
    {
        var surface = Face.MakeSurfaceFromArrayOfPoints(WavePoints());
        var solid = Solid.Thicken(surface, 1);
        Check.AlmostEqual(101.59, solid.Volume, 2);

        var square = Face.MakeRect(10, 10);
        var bbox = Solid.Thicken(square, 1, normalOverride: (0, 0, -1)).BoundingBox();
        Check.AlmostEqual((-5, -5, -1), bbox.Min, 5);
        Check.AlmostEqual((5, 5, 0), bbox.Max, 5);
    }

    [Fact]
    public void FaceOnSurface()
    {
        // a band of a cylinder between two circles: the seam comes from the fix
        var wall = Solid.MakeCylinder(2, 10).Faces().FilterBy(GeomType.Cylinder)[0];
        var top = new Wire(Edge.MakeCircle(2, Plane.XY.Offset(7)));
        var bottom = new Wire(Edge.MakeCircle(2, Plane.XY.Offset(3)));
        var band = new Face(top, [bottom], surface: wall);
        Assert.True(band.IsValid);
        Assert.Equal(GeomType.Cylinder, band.GeomType);
        Check.AlmostEqual(2 * Math.PI * 2 * 4, band.Area, 5);

        // a frustum of a cone, on a cone face of a different extent
        var cone = Solid.MakeCone(4, 0, 4).Faces().FilterBy(GeomType.Cone)[0];
        var outer = new Wire(Edge.MakeCircle(3, Plane.XY.Offset(1)));
        var inner = new Wire(Edge.MakeCircle(1, Plane.XY.Offset(3)));
        var frustum = new Face(outer, [inner], surface: cone);
        Assert.True(frustum.IsValid);
        Assert.Equal(GeomType.Cone, frustum.GeomType);
        Check.AlmostEqual(Math.PI * (3 + 1) * Math.Sqrt(8), frustum.Area, 5);

        // a planar surface gives the planar face
        var square = Obj.Rectangle(4, 4).OuterWire();
        var onPlane = new Face(square, surface: Face.MakeRect(1, 1));
        Check.AlmostEqual(new Face(square).Area, onPlane.Area, 5);

        // the surface's placement is applied
        var shift = new Pos(10, 0, 0);
        var shifted = new Face(top.Moved(shift), [bottom.Moved(shift)], surface: wall.Moved(shift));
        Check.AlmostEqual(10, shifted.BoundingBox().Center().X, 5);
        Check.AlmostEqual(band.Area, shifted.Area, 5);

        Assert.Throws<ArgumentException>(() => new Face(top, [bottom], surface: new Face()));
        Assert.Throws<ArgumentException>(() => new Face(new Wire(Edge.MakeLine((0, 0, 7), (2, 0, 7))), surface: wall));
        Assert.Throws<InvalidOperationException>(() => new Face(new Wire(Edge.MakeCircle(3, Plane.XY.Offset(7))), surface: wall));
    }

    [Fact]
    public void MakeHoles()
    {
        double radius = 10;
        var circumference = 2 * Math.PI * radius;
        var hexDiagonal = 4 * (circumference / 10) / 3;
        var cylinder = Solid.MakeCylinder(radius, hexDiagonal * 5);
        var cylinderWall = cylinder.Faces().FilterBy(GeomType.Plane, reverse: true)[0];
        // BuildSketch(Plane.XZ.offset(radius)) with Locations((0, hex_diagonal)): RegularPolygon(hex_diagonal * 0.4, 6)
        var r = hexDiagonal * 0.4;
        var hex = Wire.MakePolygon(Enumerable.Range(0, 6)
            .Select(i => new Vector(r * Math.Cos(i * Math.PI / 3), hexDiagonal + r * Math.Sin(i * Math.PI / 3))));
        var hexWireVertical = (Plane.XZ.Offset(radius) * new Face(hex)).OuterWire();

        var projectedWire = hexWireVertical.ProjectToShape(cylinder, center: (0, 0, hexWireVertical.Center().Z))[0];
        var projectedWires =
            from i in Enumerable.Range(0, 5)
            from j in Enumerable.Range(0, 4 - i % 2)
            select projectedWire.Rotate(Axis.Z, -90 + i * 360.0 / 10).Translate((0, 0, (j + (i % 2) / 2.0) * hexDiagonal));
        var cylinderWallsWithHoles = cylinderWall.MakeHoles(projectedWires);
        Assert.True(cylinderWallsWithHoles.IsValid);
        Assert.True(cylinderWallsWithHoles.Area < cylinderWall.Area);
    }

    [Fact]
    public void IsInside()
    {
        var square = Face.MakeRect(10, 10);
        Assert.True(square.IsInside((1, 1)));
        Assert.False(square.IsInside((20, 1)));
    }

    [Fact]
    public void IsCoplanar()
    {
        var square = Face.MakeRect(1, 1, Plane.XZ);
        Assert.True(square.IsCoplanar(Plane.XZ));
        Assert.True(((Face)(-square)).IsCoplanar(Plane.XZ));
        Assert.False(square.IsCoplanar(Plane.XY));
        var surface = Solid.MakeSphere(1).Faces()[0];
        Assert.False(surface.IsCoplanar(Plane.XY));
    }

    [Fact]
    public void CenterLocation()
    {
        var square = Face.MakeRect(1, 1, Plane.XZ);
        var cl = square.CenterLocation;
        Check.AlmostEqual((0, 0, 0), cl.Position, 5);
        Check.AlmostEqual(Plane.XZ.ZDir, new Plane(cl).ZDir, 5);
    }

    [Fact]
    public void PositionAt()
    {
        var square = Face.MakeRect(2, 2, Plane.XZ.Offset(1));
        Check.AlmostEqual((-0.5, -1.0, 0.5), square.PositionAt(0.25, 0.75), 5);
    }

    [Fact]
    public void ParamAtPoint()
    {
        // normalized parameters invert position_at, on a moved face too
        var cylinder = Obj.Cylinder(5, 10).Faces().FilterBy(GeomType.Cylinder)[0];
        var torus = Solid.MakeTorus(10, 2).SingleFace();
        var patch = new Pos(1, 2, 3) * new Rotation(30, 40, 50) * Face.MakeRect(4, 2);
        foreach (var face in new[] { cylinder, torus, patch, new Pos(1, 2, 3) * new Rotation(25) * torus })
            foreach (var (u, v) in new[] { (0.1, 0.2), (0.5, 0.5), (0.9, 0.7) })
            {
                var uv = face.ParamAtPoint(face.PositionAt(u, v));
                Check.AlmostEqual(u, uv.U, 6);
                Check.AlmostEqual(v, uv.V, 6);
            }

        // the kernel's own parameters: angle about the axis, distance along it
        var (ku, kv) = cylinder.ParamAtPoint(cylinder.PositionAt(0.25, 0.5), false);
        Check.AlmostEqual(Math.PI / 2, ku, 6);
        Check.AlmostEqual(5, kv, 6);

        // a face across both seams of a torus: periodic parameters land in the face's own range, in u and in v
        using var ax3 = new gp_Ax3();
        using var toroidal = new Geom_ToroidalSurface(ax3, 10, 2);
        using var maker = new BRepBuilderAPI_MakeFace(toroidal, -Math.PI / 2, Math.PI / 2, -Math.PI / 3, Math.PI / 3, 1e-6);
        var across = new Face(maker.Face());
        foreach (var (u, v) in new[] { (0.1, 0.1), (0.9, 0.9), (0.1, 0.9) })
        {
            var uv = across.ParamAtPoint(across.PositionAt(u, v));
            Check.AlmostEqual(u, uv.U, 6);
            Check.AlmostEqual(v, uv.V, 6);
        }

        // on the surface but beyond the face's boundary
        Check.AlmostEqual(1.5, patch.ParamAtPoint(patch.PositionAt(1.5, 0.5)).U, 6);

        Assert.Contains("from the face's surface", Assert.Throws<ArgumentException>(() => cylinder.ParamAtPoint((0, 0, 5))).Message);
        Assert.Contains("empty face", Assert.Throws<InvalidOperationException>(() => new Face().ParamAtPoint((0, 0, 0))).Message);
    }

    [Fact]
    public void DerivativeAt()
    {
        // normalized derivatives are those of position_at, by central differences
        var cylinder = Obj.Cylinder(5, 10).Faces().FilterBy(GeomType.Cylinder)[0];
        var torus = new Pos(1, 2, 3) * new Rotation(25) * Solid.MakeTorus(10, 2).SingleFace();
        const double h = 1e-4;
        foreach (var face in new[] { cylinder, torus })
        {
            Vector At(double du, double dv) => face.PositionAt(0.3 + du, 0.6 + dv);
            var expected = new Dictionary<(int, int), Vector>
            {
                [(1, 0)] = (At(h, 0) - At(-h, 0)) / (2 * h),
                [(0, 1)] = (At(0, h) - At(0, -h)) / (2 * h),
                [(2, 0)] = (At(h, 0) - At(0, 0) * 2 + At(-h, 0)) / (h * h),
                [(0, 2)] = (At(0, h) - At(0, 0) * 2 + At(0, -h)) / (h * h),
                [(1, 1)] = (At(h, h) - At(h, -h) - At(-h, h) + At(-h, -h)) / (4 * h * h),
            };
            foreach (var ((uOrder, vOrder), value) in expected)
                Check.AlmostEqual(value, face.DerivativeAt(0.3, 0.6, uOrder, vOrder), 3);
        }

        // the kernel's own: a radius per radian about the axis, 1 along it
        var (u, v) = cylinder.ParamAtPoint(cylinder.PositionAt(0.25, 0.5), false);
        Check.AlmostEqual((-5, 0, 0), cylinder.DerivativeAt(u, v, 1, 0, normalize: false), 6);
        Check.AlmostEqual((0, 0, 1), cylinder.DerivativeAt(u, v, 0, 1, normalize: false), 6);

        Assert.Contains("orders", Assert.Throws<ArgumentException>(() => cylinder.DerivativeAt(0.5, 0.5, 0, 0)).Message);
        Assert.Contains("orders", Assert.Throws<ArgumentException>(() => cylinder.DerivativeAt(0.5, 0.5, -1, 2)).Message);
        Assert.Contains("empty face", Assert.Throws<InvalidOperationException>(() => new Face().DerivativeAt(0.5, 0.5, 1, 0)).Message);
    }

    [Fact]
    public void MakeSurface()
    {
        Vector[] corners = [.. from x in new[] { -50.5, 50.5 } from y in new[] { -24.5, 24.5 } select new Vector(x, y)];
        var netExterior = new Wire(
        [
            Edge.MakeLine(corners[3], corners[1]),
            Edge.MakeLine(corners[1], corners[0]),
            Edge.MakeLine(corners[0], corners[2]),
            Edge.MakeThreePointArc(corners[2], (corners[2] + corners[3]) / 2 - new Vector(0, 0, 3), corners[3]),
        ]);
        var surface = Face.MakeSurface(netExterior, surfacePoints: [new Vector(0, 0, -5)]);
        var holeFlat = Wire.MakeCircle(10);
        var hole = holeFlat.ProjectToShape(surface, (0, 0, -1))[0];
        surface = Face.MakeSurface(netExterior, surfacePoints: [new Vector(0, 0, -5)], interiorWires: [hole]);
        Assert.True(surface.IsValid);
        Assert.Equal(GeomType.BSpline, surface.GeomType);
        var bbox = surface.BoundingBox();
        Check.AlmostEqual((-50.5, -24.5, -5.113393280136395), bbox.Min, 5);
        Check.AlmostEqual((50.5, 24.5, 0), bbox.Max, 5);

        // With no surface point
        surface = Face.MakeSurface(netExterior);
        bbox = surface.BoundingBox();
        Check.AlmostEqual((-50.5, -24.5, -3), bbox.Min, 5);
        Check.AlmostEqual((50.5, 24.5, 0), bbox.Max, 5);

        // Exterior Edge
        surface = Face.MakeSurface([Edge.MakeCircle(50)], surfacePoints: [(0, 0, -5)]);
        bbox = surface.BoundingBox();
        Check.AlmostEqual((-50, -50, -5), bbox.Min, 5);
        Check.AlmostEqual((50, 50, 0), bbox.Max, 5);
    }

    [Fact]
    public void MakeSurfaceErrorChecking()
    {
        Assert.Throws<InvalidOperationException>(() => Face.MakeSurface([Edge.MakeLine((0, 0), (1, 0))]));
        Assert.Throws<InvalidOperationException>(() =>
            Face.MakeSurface([Edge.MakeCircle(50)], surfacePoints: [(0, 0, -50), (0, 0, 50)]));
        Assert.Throws<InvalidOperationException>(() =>
            Face.MakeSurface([Edge.MakeCircle(50)], interiorWires: [Wire.MakeCircle(5, Plane.XZ)]));
    }

    [Fact]
    public void Sweep()
    {
        var edge = Edge.MakeLine((1, 0), (2, 0));
        var path = Wire.MakeCircle(1);
        var circleWithHole = Face.Sweep(edge, path);
        Assert.IsType<Face>(circleWithHole);
        Check.AlmostEqual(Math.PI * (Math.Pow(2, 2) - 1), circleWithHole.Area, 5);
        var polyline = new Wire([Edge.MakeLine((0, 0), (0.1, 0)), Edge.MakeLine((0.1, 0), (0.2, 0.1))]);
        Assert.Throws<ArgumentException>(() => Face.Sweep(edge, polyline));
    }

    [Fact]
    public void MakeSurfacePatch()
    {
        var m1 = Edge.MakeSpline([(0, 0), (1, 0), (10, 0, -10)]);
        var m2 = Edge.MakeSpline([(0, 0), (0, 1), (0, 10, -10)]);
        var m3 = Edge.MakeSpline([m1.PositionAt(1), (7, 7, -10), m2.PositionAt(1)]);

        var patch = Face.MakeSurfacePatch(edgeConstraints: [m1, m2, m3]);
        Check.AlmostEqual(157.186, patch.Area, 3);

        var f1 = Face.Extrude(m1, (0, -1, 0));
        var f2 = Face.Extrude(m2, (-1, 0, 0));
        var f3 = Face.Extrude(m3, (0, 0, -1));

        var patch2 = Face.MakeSurfacePatch(edgeFaceConstraints:
        [
            (m1, f1, ContinuityLevel.C1),
            (m2, f2, ContinuityLevel.C1),
            (m3, f3, ContinuityLevel.C1),
        ]);
        Check.AlmostEqual(152.670, patch2.Area, 3);

        // Mix both perimeter constraint styles: two edges with support faces, the third as a plain edge
        var patch3 = Face.MakeSurfacePatch(
            edgeFaceConstraints: [(m1, f1, ContinuityLevel.C1), (m2, f2, ContinuityLevel.C1)],
            edgeConstraints: [m3]);
        Check.AlmostEqual(151.672, patch3.Area, 3);

        var point = patch.PositionAt(0.5, 0.5) + new Vector(0.5, 0.5);
        var patch4 = -Face.MakeSurfacePatch(edgeConstraints: [m1, m2, m3], pointConstraints: [point]);
        Check.AlmostEqual(164.618, patch4.Area, 3);
    }

    [Fact]
    public void MakeSurfacePatchErrorChecking()
    {
        Assert.Throws<InvalidOperationException>(() => Face.MakeSurfacePatch(edgeConstraints: [Edge.MakeLine((0, 0), (1, 0))]));
        Assert.Throws<InvalidOperationException>(() => Face.MakeSurfacePatch(edgeConstraints: []));
        Assert.Throws<InvalidOperationException>(() =>
            Face.MakeSurfacePatch(edgeConstraints: [Edge.MakeLine((0, 0), (1, 0)), Edge.MakeLine((0, 0), (0, 1))]));
    }

    static Face Frame() => (Face.MakeRect(1, 1) - Face.MakeRect(0.5, 0.5)).SingleFace();

    [Fact]
    public void OuterWire() => Check.AlmostEqual(4, Frame().OuterWire().Length, 5);

    [Fact]
    public void SingleWire()
    {
        // a face with holes has more than one wire, so the singular accessor raises as it does on every other shape
        var face = Frame();
        var e = Assert.Throws<InvalidOperationException>(() => face.SingleWire());
        Assert.Contains("Expected exactly one wire, found 2", e.Message);
        Check.AlmostEqual(4, Face.MakeRect(1, 1).SingleWire().Length, 5);
    }

    [Fact]
    public void NormalAt()
    {
        var face = Face.MakeRect(1, 1);
        Check.AlmostEqual((0, 0, 1), face.NormalAt(0, 0), 5);
        Check.AlmostEqual((0, 0, 1), face.NormalAt(face.PositionAt(0, 0)), 5);
        face = Obj.Cylinder(1, 1).Faces().FilterBy(GeomType.Cylinder)[0];
        Check.AlmostEqual((1, 0, 0), face.NormalAt(0, 1), 5);
    }

    [Fact]
    public void LocationAt()
    {
        var face = Face.MakeRect(1, 1);

        // Default center (u=0, v=0)
        var loc = face.LocationAt(0, 0);
        Check.AlmostEqual((-0.5, -0.5, 0), loc.Position, 5);
        Check.AlmostEqual((0, 0, 1), loc.ZAxis.Direction, 5);

        // Using surface_point instead of u,v
        var point = face.PositionAt(0, 0);
        var loc2 = face.LocationAt(point);
        Check.AlmostEqual((-0.5, -0.5, 0), loc2.Position, 5);
        Check.AlmostEqual((0, 0, 1), loc2.ZAxis.Direction, 5);

        // Curved surface: verify z-direction is outward normal
        face = Obj.Cylinder(1, 1).Faces().FilterBy(GeomType.Cylinder)[0];
        var loc3 = face.LocationAt(0, 1);
        Check.AlmostEqual((1, 0, 0), loc3.ZAxis.Direction, 5);

        // Curved surface: verify center
        var loc4 = face.LocationAt();
        Check.AlmostEqual((-1, 0, 0), loc4.Position, 5);
        Check.AlmostEqual((-1, 0, 0), loc4.ZAxis.Direction, 5);

        // Reversed face: z-direction must follow the orientation flag (#1007)
        var rect = Face.MakeRect(34, 10);
        var rectFlipped = (Face)(-rect);
        Check.AlmostEqual((0, 0, 1), rect.LocationAt().ZAxis.Direction, 5);
        Check.AlmostEqual((0, 0, -1), rectFlipped.LocationAt().ZAxis.Direction, 5);
    }

    [Fact]
    public void WithoutHoles()
    {
        // Planar test
        var frame = Frame();
        var filled = frame.WithoutHoles();
        Assert.Single(frame.InnerWires());
        Assert.Empty(filled.InnerWires());
        Check.AlmostEqual(0.75, frame.Area, 5);
        Check.AlmostEqual(1.0, filled.Area, 5);

        // Errors
        Assert.Throws<InvalidOperationException>(() => new Face().WithoutHoles());

        // No holes
        var rect = Face.MakeRect(1, 1);
        Assert.Equal(rect, rect.WithoutHoles());

        // Non-planar test
        var cylFace = (Obj.Cylinder(1, 3) - new Rotation(90, 0, 0) * Obj.Cylinder(0.5, 3))
            .Faces().SortBy(f => f.Area).Last;
        filled = cylFace.WithoutHoles();
        Assert.Equal(2, cylFace.InnerWires().Count);
        Assert.Empty(filled.InnerWires());
        Assert.True(cylFace.Area < filled.Area);
        Check.AlmostEqual(filled.Area, cylFace.AreaWithoutHoles, 5);
    }

    [Fact]
    public void AreaWithoutHoles() => Check.AlmostEqual(0.0, new Face().AreaWithoutHoles, 5);

    [Fact]
    public void AxesOfSymmetry()
    {
        // Empty shape
        Assert.Throws<InvalidOperationException>(() => new Face().AxesOfSymmetry);

        // Non planar
        var cylinder = Solid.MakeCylinder(1, 2).Faces().FilterBy(GeomType.Cylinder)[0];
        Assert.Throws<InvalidOperationException>(() => cylinder.AxesOfSymmetry);

        // Test a variety of shapes
        var rectMin = Face.MakeRect(1, 2, new Plane((0.5, 1)));
        var rectMinWithHole = (rectMin - new Pos(0.5, 0.75) * Obj.Circle(0.2)).SingleFace();
        IEnumerable<Shape> Circles(Location at) => Obj.PolarLocations(1, 3).Select(l => (Shape)(at * l * Obj.Circle(0.3)));
        (Face Face, int Axes)[] shapes =
        [
            (Obj.Rectangle(1, 1), 4),
            (rectMin, 2),
            (new Rotation(0, 0, 10) * Obj.Rectangle(1, 2), 2),
            (rectMinWithHole, 1),
            (rectMinWithHole.Rotate(Axis.Z, 10), 1),
            // Triangle(a=1, b=0.5, C=90)
            (new Face(Wire.MakePolygon([(-2.0 / 3, -1.0 / 6), (1.0 / 3, -1.0 / 6), (1.0 / 3, 1.0 / 3)])), 0),
            ((Obj.Circle(2) - new Pos(0.1) * Obj.Rectangle(0.5, 0.5)).SingleFace(), 1),
            ((Obj.Circle(2) - new Pos(0.1, 0.1) * Obj.Rectangle(0.5, 0.5)).SingleFace(), 1),
            ((Obj.Circle(2) - Circles(new Pos(0.1, 0.1))).SingleFace(), 0),
            ((Obj.Circle(2) - Circles(new Pos(0.5))).SingleFace(), 1),
            ((Obj.Circle(2) - Circles(new Location())).SingleFace(), 3),
            // Ellipse(1, 2, rotation=10)
            (new Rotation(0, 0, 10) * new Face(Wire.MakeEllipse(1, 2)), 2),
        ];
        for (int i = 0; i < shapes.Length; i++)
        {
            var (testFace, count) = shapes[i];
            Assert.True(count == testFace.AxesOfSymmetry.Count, $"shape {i}: expected {count} axes, got {testFace.AxesOfSymmetry.Count}");
        }

        // Fast abort code paths
        var s1 = Edge.MakeSpline(
        [
            (0.0293923441471, 1.9478225275438),
            (0.0293923441471, 1.2810839877038),
            (0, -0.0521774724562),
            (0.0293923441471, -1.3158620329962),
            (0.0293923441471, -1.9478180575162),
        ]);
        var l1 = Edge.MakeLine(s1.PositionAt(1), s1.PositionAt(0));
        Assert.Empty(new Face(new Wire([s1, l1])).AxesOfSymmetry);

        // BuildLine + make_face
        static Edge L(double x1, double y1, double x2, double y2) => Edge.MakeLine((x1, y1), (x2, y2));
        static Edge A(double x1, double y1, double x2, double y2, double x3, double y3) =>
            Edge.MakeThreePointArc((x1, y1), (x2, y2), (x3, y3));
        Edge[] edges =
        [
            L(-13.186467340991, 2.3737403364651, -5.1864673409911, 2.3737403364651),
            L(-13.186467340991, 2.3737403364651, -13.186467340991, -2.4506956262169),
            A(-13.186467340991, -2.4506956262169, -13.479360559805, -3.1578024074034, -14.186467340991, -3.4506956262169),
            L(-17.186467340991, -3.4506956262169, -14.186467340991, -3.4506956262169),
            A(-17.186467340991, -3.4506956262169, -17.893574122178, -3.1578024074034, -18.186467340991, -2.4506956262169),
            L(-18.186467340991, 7.6644400497781, -18.186467340991, -2.4506956262169),
            L(-51.186467340991, 7.6644400497781, -18.186467340991, 7.6644400497781),
            L(-51.186467340991, 7.6644400497781, -51.186467340991, -5.5182296356389),
            L(-51.186467340991, -5.5182296356389, -33.186467340991, -5.5182296356389),
            L(-33.186467340991, -5.5182296356389, -33.186467340991, -5.3055423052429),
            L(-33.186467340991, -5.3055423052429, 53.813532659009, -5.3055423052429),
            L(53.813532659009, -5.3055423052429, 53.813532659009, -5.7806956262169),
            L(66.813532659009, -5.7806956262169, 53.813532659009, -5.7806956262169),
            L(66.813532659009, -2.7217530775369, 66.813532659009, -5.7806956262169),
            L(54.813532659009, -2.7217530775369, 66.813532659009, -2.7217530775369),
            L(54.813532659009, 7.6644400497781, 54.813532659009, -2.7217530775369),
            L(38.813532659009, 7.6644400497781, 54.813532659009, 7.6644400497781),
            L(38.813532659009, 7.6644400497781, 38.813532659009, -2.4506956262169),
            A(38.813532659009, -2.4506956262169, 38.520639440195, -3.1578024074034, 37.813532659009, -3.4506956262169),
            L(37.813532659009, -3.4506956262169, 34.813532659009, -3.4506956262169),
            A(34.813532659009, -3.4506956262169, 34.106425877822, -3.1578024074034, 33.813532659009, -2.4506956262169),
            L(33.813532659009, 2.3737403364651, 33.813532659009, -2.4506956262169),
            L(25.813532659009, 2.3737403364651, 33.813532659009, 2.3737403364651),
            L(25.813532659009, 2.3737403364651, 25.813532659009, -2.4506956262169),
            A(25.813532659009, -2.4506956262169, 25.520639440195, -3.1578024074034, 24.813532659009, -3.4506956262169),
            L(24.813532659009, -3.4506956262169, 21.813532659009, -3.4506956262169),
            A(21.813532659009, -3.4506956262169, 21.106425877822, -3.1578024074034, 20.813532659009, -2.4506956262169),
            L(20.813532659009, 2.3737403364651, 20.813532659009, -2.4506956262169),
            L(12.813532659009, 2.3737403364651, 20.813532659009, 2.3737403364651),
            L(12.813532659009, 2.3737403364651, 12.813532659009, -2.4506956262169),
            A(12.813532659009, -2.4506956262169, 12.520639440195, -3.1578024074034, 11.813532659009, -3.4506956262169),
            L(8.8135326590089, -3.4506956262169, 11.813532659009, -3.4506956262169),
            A(8.8135326590089, -3.4506956262169, 8.1064258778223, -3.1578024074034, 7.8135326590089, -2.4506956262169),
            L(7.8135326590089, 2.3737403364651, 7.8135326590089, -2.4506956262169),
            L(-0.1864673409911, 2.3737403364651, 7.8135326590089, 2.3737403364651),
            L(-0.1864673409911, 2.3737403364651, -0.1864673409911, -2.4506956262169),
            A(-0.1864673409911, -2.4506956262169, -0.4793605598046, -3.1578024074034, -1.1864673409911, -3.4506956262169),
            L(-4.1864673409911, -3.4506956262169, -1.1864673409911, -3.4506956262169),
            A(-4.1864673409911, -3.4506956262169, -4.8935741221777, -3.1578024074034, -5.1864673409911, -2.4506956262169),
            L(-5.1864673409911, 2.3737403364651, -5.1864673409911, -2.4506956262169),
        ];
        var outline = Assert.Single(Wire.Combine(edges));
        Assert.Empty(new Face(outline).AxesOfSymmetry);
    }

    [Fact]
    public void RadiusProperty()
    {
        var c = Obj.Cylinder(1.5, 2).Faces().FilterBy(GeomType.Cylinder)[0];
        var s = Solid.MakeSphere(3).Faces().FilterBy(GeomType.Sphere)[0];
        var b = Obj.Box(1, 1, 1).Faces()[0];
        Check.AlmostEqual(1.5, c.Radius!.Value, 5);
        Check.AlmostEqual(3, s.Radius!.Value, 5);
        Assert.Null(b.Radius);
    }

    [Fact]
    public void AxisOfRotationProperty()
    {
        // CONE
        var cone = Solid.MakeCone(2, 1, 2).Faces().FilterBy(GeomType.Cone)[0];
        Check.AlmostEqual((0, 0, 1), cone.AxisOfRotation!.Value.Direction, 5);
        Check.AlmostEqual((0, 0, 0), cone.AxisOfRotation!.Value.Position, 5);

        // CYLINDER
        var cyl = (new Rotation(90, 0, 0) * Obj.Cylinder(1.5, 2)).Faces().FilterBy(GeomType.Cylinder)[0];
        Check.AlmostEqual((0, -1, 0), cyl.AxisOfRotation!.Value.Direction, 5);
        Check.AlmostEqual((0, 1, 0), cyl.AxisOfRotation!.Value.Position, 5);

        // REVOLUTION
        var r = Face.Revolve(Edge.MakeSpline([(0, 0, 0), (1, 0, 0.5), (1, 0, 2)]), 90, Axis.Z);
        Check.AlmostEqual((0, 0, 1), r.AxisOfRotation!.Value.Direction, 5);
        Check.AlmostEqual((0, 0, 0), r.AxisOfRotation!.Value.Position, 5);

        // SPHERE
        var s = Solid.MakeSphere(3).Faces().FilterBy(GeomType.Sphere)[0];
        Check.AlmostEqual((0, 0, 1), s.AxisOfRotation!.Value.Direction, 5);
        Check.AlmostEqual((0, 0, 0), s.AxisOfRotation!.Value.Position, 5);

        // TORUS
        var t = (new Rotation(90, 0, 0) * Solid.MakeTorus(4, 1)).Faces().FilterBy(GeomType.Torus)[0];
        Check.AlmostEqual((0, -1, 0), t.AxisOfRotation!.Value.Direction, 5);
        Check.AlmostEqual((0, 0, 0), t.AxisOfRotation!.Value.Position, 5);

        // Geom_RectangularTrimmedSurface
        using var origin = new gp_Pnt(0, 0, 0);
        using var up = new gp_Dir(0, 0, 1);
        using var ax3 = new gp_Ax3(origin, up);
        using var cylSurf = new Geom_CylindricalSurface(ax3, 2.0);
        using var trimSurf = new Geom_RectangularTrimmedSurface(cylSurf, 0.0, 1.0, 0.5, 1.5);
        using var trimMaker = new BRepBuilderAPI_MakeFace(trimSurf, 1e-6);
        var trimFace = new Face(trimMaker.Face());
        Check.AlmostEqual((0, 0, 1), trimFace.AxisOfRotation!.Value.Direction, 5);
        Check.AlmostEqual((0, 0, 0), trimFace.AxisOfRotation!.Value.Position, 5);
        Check.AlmostEqual(2.0, trimFace.Radius!.Value, 5);

        // Geom_OffsetSurface
        using var cylOffSurf = new Geom_OffsetSurface(cylSurf, 0.5);
        using var offMaker = new BRepBuilderAPI_MakeFace(cylOffSurf, 1e-6);
        var offFace = new Face(offMaker.Face());
        Check.AlmostEqual((0, 0, 1), offFace.AxisOfRotation!.Value.Direction, 5);
        Check.AlmostEqual((0, 0, 0), offFace.AxisOfRotation!.Value.Position, 5);

        // Invalid
        Assert.Null(Face.MakeRect(1, 1).AxisOfRotation);
    }

    [Fact]
    public void ConvexityBySurface()
    {
        Assert.All(Obj.Box(1, 1, 1).Faces(), f => Assert.Equal(Convexity.Smooth, f.Convexity));
        var boss = Obj.Cylinder(3, 5).Faces().FilterBy(GeomType.Cylinder)[0];
        Assert.Equal(Convexity.Convex, boss.Convexity);
        var hole = (Obj.Box(10, 10, 5) - Obj.Cylinder(2, 10)).Faces().FilterBy(GeomType.Cylinder)[0];
        Assert.Equal(Convexity.Concave, hole.Convexity);
        Assert.Equal(Convexity.Convex, Solid.MakeSphere(5).Faces()[0].Convexity);
        var cavity = (Obj.Box(20, 20, 20) - Solid.MakeSphere(5)).Faces().FilterBy(GeomType.Sphere)[0];
        Assert.Equal(Convexity.Concave, cavity.Convexity);
        // the inner half of a torus bends the other way to the outer half
        Assert.Equal(Convexity.Saddle, Solid.MakeTorus(10, 3).Faces()[0].Convexity);
        Assert.Contains("empty face", Assert.Throws<InvalidOperationException>(() => new Face().Convexity).Message);
    }

    [Fact]
    public void Radii()
    {
        var t = Solid.MakeTorus(5, 1).SingleFace();
        var radii = t.Radii!.Value;
        Check.AlmostEqual(5, radii.Major, 5);
        Check.AlmostEqual(1, radii.Minor, 5);
        Assert.Null(Solid.MakeSphere(1).SingleFace().Radii);
    }

    [Fact]
    public void Revolve()
    {
        var l1 = Edge.MakeLine((3, 0), (3, 2));
        var revolved = Face.Revolve(l1, 360, Axis.Y);
        Assert.IsType<Face>(revolved);
        Check.AlmostEqual(2 * Math.PI * 3 * 2, revolved.Area, 5);

        // JernArc(l1 @ 1, l1 % 1, 1, 90): a quarter circle about (2, 2) from (3, 2) to (2, 3)
        var l2 = Edge.MakeThreePointArc(l1.PositionAt(1), (2 + Math.Sqrt(0.5), 2 + Math.Sqrt(0.5)), (2, 3));
        var w1 = new Wire([l1, l2]);
        var revolvedShell = Shell.Revolve(w1, 180, Axis.Y);
        Assert.IsType<Shell>(revolvedShell);
        Check.AlmostEqual(2, revolvedShell.Edges().SortBy(Axis.Y).Last.Radius, 5);
    }

    [Fact]
    public void OpenWire()
    {
        var perimeter = new Wire(
        [
            Edge.MakeLine((0, 0), (1, 0)),
            Edge.MakeLine((1, 0), (1, 1)),
            Edge.MakeLine((1, 1), (0, 1)),
        ]);
        Assert.Throws<ArgumentException>(() => new Face(perimeter));
        Assert.Throws<ArgumentException>(() => new Face(Wire.MakeCircle(5), [perimeter]));
    }

    [Fact]
    public void Seams()
    {
        Assert.Empty(Face.MakeRect(1, 1).Seams);
        Assert.Single(Solid.MakeSphere(1).SingleFace().Seams);
        Assert.Equal(2, Solid.MakeTorus(4, 1).SingleFace().Seams.Count);
    }
}

/// <summary>Mixin2D.offset moves each point along its own normal.</summary>
public class SurfaceOffsetTests
{
    static Face QuarterCylinder(double radius = 5.0) =>
        Face.Extrude(Edge.MakeCircle(radius, Plane.XY, 0, 90), (0, 0, 10));

    [Fact]
    public void PlanarFaceTranslates()
    {
        // For a plane, offsetting is a translation, as it always was
        var face = Face.MakeRect(10, 10);
        foreach (var amount in new[] { 2.0, -2.0 })
        {
            var moved = face.Offset(amount);
            Assert.IsType<Face>(moved);
            Check.AlmostEqual(amount, moved.Center().Z, 6);
            Check.AlmostEqual(face.Area, moved.Area, 6);
        }
    }

    [Fact]
    public void CylindricalFaceChangesRadius()
    {
        // A translation would leave the radius alone
        var face = QuarterCylinder();
        var outward = face.Offset(-1);
        var inward = face.Offset(1);
        Check.AlmostEqual(6.0, outward.Radius!.Value, 6);
        Check.AlmostEqual(4.0, inward.Radius!.Value, 6);
        Check.AlmostEqual(face.Area * 6 / 5, outward.Area, 6);
        Check.AlmostEqual(face.Area * 4 / 5, inward.Area, 6);
    }

    [Fact]
    public void PositiveFollowsTheNormal()
    {
        // A sphere's normal points outward, so a positive offset grows it
        var sphere = Solid.MakeSphere(5).Faces()[0];
        Check.AlmostEqual(4 * Math.PI * 36, sphere.Offset(1).Area, 4);
        Check.AlmostEqual(4 * Math.PI * 16, sphere.Offset(-1).Area, 4);
    }

    [Fact]
    public void ShellStaysAShell()
    {
        var boxShell = Solid.MakeBox(10, 10, 10).Shells()[0];
        var bigger = boxShell.Offset(0.5);
        Assert.IsType<Shell>(bigger);
        Assert.Equal(6, bigger.Faces().Count);
        Check.AlmostEqual(6 * 11 * 11, bigger.Area, 6);
    }

    [Fact]
    public void ZeroOffsetReturnsACopy()
    {
        var face = Face.MakeRect(10, 10);
        var same = face.Offset(0);
        Assert.IsType<Face>(same);
        Check.AlmostEqual(face.Area, same.Area, 6);
    }

    [Fact]
    public void CollapseIsRejected()
    {
        // Offsetting a cylinder by its own radius leaves nothing
        var e = Assert.Throws<InvalidOperationException>(() => QuarterCylinder().Offset(5.0));
        Assert.Contains("collapsed", e.Message);
    }

    [Fact]
    public void AttributesAreCarriedOver()
    {
        var face = Face.MakeRect(10, 10).WithLabel("plate");
        Assert.Equal("plate", face.Offset(1).Label);
    }
}

/// <summary>Rejection paths of Face construction and surface building.</summary>
public class FaceValidationTests
{
    [Fact]
    public void ExtrudeRejectsAnEmptyObject() =>
        Assert.Contains("Can't extrude empty object",
                        Assert.Throws<ArgumentException>(() => Face.Extrude(new Edge(), (0, 0, 1))).Message);

    [Fact]
    public void SurfaceExteriorRejectsEmptyEdges() =>
        Assert.Contains("exterior contains empty edges",
                        Assert.Throws<ArgumentException>(() => Face.MakeSurface([Edge.MakeLine((0, 0), (1, 0)), new Edge()])).Message);

    [Fact]
    public void SurfaceInteriorRejectsAnEmptyWire()
    {
        var exterior = Wire.MakeRect(10, 10);
        Assert.Contains("empty wire",
                        Assert.Throws<ArgumentException>(() => Face.MakeSurface(exterior, interiorWires: [new Wire()])).Message);
    }

    [Fact]
    public void ProjectToAVertexIsUnsupported()
    {
        var face = Face.MakeRect(2, 2);
        Assert.Contains("projection to a vertex",
                        Assert.Throws<ArgumentException>(() => face.ProjectToShape(new Vertex(0, 0, 5), (0, 0, -1))).Message);
    }
}

public class SortWiresByBuildOrderTests
{
    [Fact]
    public void OuterWireFirstThenHoles()
    {
        var outer = Wire.MakeRect(10, 10);
        Wire[] holes = [Wire.MakeCircle(1, new Plane((-2, 0))), Wire.MakeCircle(1, new Plane((2, 0)))];
        var groups = TwoD.SortWiresByBuildOrder([outer, .. holes]);

        var group = Assert.Single(groups);
        Assert.Equal(3, group.Count);
        Check.AlmostEqual(40, group[0].Length, 5);
        foreach (var inner in group.Skip(1))
            Check.AlmostEqual(2 * Math.PI, inner.Length, 5);
    }

    [Fact]
    public void ASingleWireIsReturnedAsIs()
    {
        var outer = Wire.MakeRect(10, 10);
        var group = Assert.Single(TwoD.SortWiresByBuildOrder([outer]));
        Assert.Equal(outer, Assert.Single(group));
    }
}

public class FacePropertiesTests
{
    [Theory]
    [InlineData(5, 0, 5)]
    [InlineData(5, 0, 10)]
    [InlineData(5, 2, 5)]
    public void SemiAngleOfACone(double bottom, double top, double height)
    {
        // The magnitude is atan(radius change / height); OCCT signs it against the cone's axis, so a narrowing cone
        // reads negative.
        var lateral = Obj.Cone(bottom, top, height).Faces().FilterBy(GeomType.Cone)[0];
        var expected = Math.Atan((bottom - top) / height) * 180 / Math.PI;
        Check.AlmostEqual(expected, Math.Abs(lateral.SemiAngle!.Value), 5);
    }

    [Fact]
    public void SemiAngleOfATrimmedCone()
    {
        // A cone that arrived as a trimmed surface is still a cone
        var lateral = Obj.Cone(5, 0, 5).Faces().FilterBy(GeomType.Cone)[0];
        using var surface = BRep_Tool.Surface(TopoDS.Face(lateral.Wrapped));
        using var trimmed = new Geom_RectangularTrimmedSurface(surface, 0.0, 1.0, 0.0, 2.0);
        using var maker = new BRepBuilderAPI_MakeFace(trimmed, 1e-6);
        var face = new Face(maker.Face());
        Assert.Equal(GeomType.Cone, face.GeomType);
        Check.AlmostEqual(45, Math.Abs(face.SemiAngle!.Value), 5);
    }

    [Fact]
    public void SemiAngleOfOtherSurfaces()
    {
        Assert.Null(Obj.Rectangle(1, 1).SemiAngle);
        var cylinder = Obj.Cylinder(1, 2).Faces().FilterBy(GeomType.Cylinder)[0];
        Assert.Null(cylinder.SemiAngle);
    }

    [Fact]
    public void AFaceHasNoMassOrVolume() => Assert.Equal(0.0, Obj.Rectangle(2, 3).Volume);
}

public class SurfaceFromArrayOfPointsTests
{
    static Vector[][] Grid() =>
        [.. Enumerable.Range(0, 6).Select(y => Enumerable.Range(0, 6)
            .Select(x => new Vector(x, y, Math.Sin(x / 3.0) * Math.Cos(y / 3.0))).ToArray())];

    [Theory]
    [InlineData(1.0, 1.0, 1.0)]
    [InlineData(1.0, 5.0, 10.0)]
    [InlineData(0.1, 1.0, 10.0)]
    public void VariationalSmoothing(double w1, double w2, double w3)
    {
        // Smoothing needs degree 6 for the C2 continuity OCCT asks of it; the default max_deg of 3 is raised to suit
        // rather than failing.
        var grid = Grid();
        var plain = Face.MakeSurfaceFromArrayOfPoints(grid);
        var smoothed = Face.MakeSurfaceFromArrayOfPoints(grid, smoothing: (w1, w2, w3));
        Check.AlmostEqual(plain.Area, smoothed.Area, 1);
    }

    [Fact]
    public void ExplicitMaxDegIsNotLowered()
    {
        var surface = Face.MakeSurfaceFromArrayOfPoints(Grid(), smoothing: (1.0, 1.0, 1.0), maxDeg: 8);
        Assert.True(surface.Area > 0);
    }
}

public class ShellValidationTests
{
    [Fact]
    public void RejectsAnEmptyFace() =>
        Assert.Contains("Can't create a Shell from empty Face",
                        Assert.Throws<ArgumentException>(() => new Shell(new Face())).Message);

    [Fact]
    public void NegateRejectsAnEmptyFace() =>
        Assert.Contains("Invalid Shape", Assert.Throws<InvalidOperationException>(() => -new Face()).Message);
}
