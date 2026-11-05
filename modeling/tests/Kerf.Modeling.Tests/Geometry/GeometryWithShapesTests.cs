// Ported from build123d tests/test_direct_api/test_axis.py, test_plane.py, test_bound_box.py,
// test_oriented_bound_box.py, test_vector.py, test_location.py, test_rotation.py and test_matrix.py: the cases
// that need topology, which the Geometry/*Tests.cs ports left out. One class per source file. Omitted:
// - test_axis: the bad-edge ValueError and Axis(gp_ax1=edge) TypeError in test_axis_init: the compiler rules them
//   out (an Edge can't wrap a vertex; the other constructors take no edge).
// - test_plane: Plane(edge), to_local_coords("box"), shift_origin(edge) TypeErrors: the compiler rules them out.
// - test_plane: copy.deepcopy of a plane: planes are immutable.
// - test_plane_from_face_with_origin_warns: Plane(face) takes no origin in the port.
// - test_shift_origin_axis/_vertex/_vector: the BuildPart (insert, a BuildSketch on the plane, a subtracting extrude)
//   is written in algebra form.
// - test_local_coords_of_an_unknown_shape_type: a Solid can't be made to wrap a TopoDS_CompSolid.
// - test_bound_box test_constructor_errors: argument-type validation is the compiler's job.
// - test_bound_box find_outside_box_2d (in test_basic_bounding_box): deprecated, not ported.
// - test_combined_center_of_boundbox: empty in Python.
// - test_oriented_bound_box test_requires_a_bnd_obb_or_a_shape: argument-type validation.
// (test_location test_intersection's e1.intersect("line") is kept as the ArgumentException the port's object
// overload throws.)
// - test_rotation.py and test_matrix.py: no cases use topology.
using Ops = Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Geometry;

using Kerf.Modeling;

file static class Objects
{
    /// <summary>build123d's Box(l, w, h): centred on the origin.</summary>
    public static Solid Box(double l, double w, double h) => Solid.MakeBox(l, w, h, new Plane(new Point3D(-l / 2, -w / 2, -h / 2)));
}

public class AxisWithShapesTests
{
    [Fact]
    public void AxisInit()
    {
        var axis = new Axis(Edge.MakeLine(new Point3D(1, 2, 3), new Point3D(1, 2, 4)));
        Check.AlmostEqual(new Point3D(1, 2, 3), axis.Position, 5);
        Check.AlmostEqual(new Vector3D(0, 0, 1), axis.Direction, 5);

        axis = new Axis(edge: Edge.MakeLine(new Point3D(1, 2, 3), new Point3D(1, 2, 4)));
        Check.AlmostEqual(new Point3D(1, 2, 3), axis.Position, 5);
        Check.AlmostEqual(new Vector3D(0, 0, 1), axis.Direction, 5);
    }

    [Fact]
    public void AxisAsEdge()
    {
        var edge = Edge.FromAxis(Axis.X);
        Check.Is<Edge>(edge);
        var common = (edge & Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0))).SingleEdge();
        Check.AlmostEqual(1, common.Length, 5);
    }

    [Fact]
    public void AxisIntersect()
    {
        var common = Axis.X.Intersect(Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0))).SingleEdge();
        Check.AlmostEqual(1, common.Length, 5);
        common = (Axis.X.Intersect(Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0)))).SingleEdge();
        Check.AlmostEqual(1, common.Length, 5);

        var arc = Edge.MakeCircle(20, startAngle: 0, endAngle: 180);
        var ax0 = new Axis(new Point3D(-20, 30, 0), new Vector3D(4, -3, 0));
        var intersections = arc.Intersect(ax0).Vertices().OrderBy(Axis.X);
        Check.TupleAlmostEqual([-5.6, 19.2, 0], Check.Of(intersections[0].Point), 4);
        Check.TupleAlmostEqual([20, 0, 0], Check.Of(intersections[1].Point), 4);

        intersections = ax0.Intersect(arc).Vertices().OrderBy(Axis.X);
        Check.TupleAlmostEqual([-5.6, 19.2, 0], Check.Of(intersections[0].Point), 4);
        Check.TupleAlmostEqual([20, 0, 0], Check.Of(intersections[1].Point), 4);

        var l = (Edge.MakeLine(new Point3D(0, 0, 1), new Point3D(0, 0, 2))).LocationAt(1);
        var i = Assert.IsType<Location>(Axis.Z.Intersect(l));
        Check.AlmostEqual(l.Position, i.Position, 5);
        Check.AlmostEqual(l.Orientation, i.Orientation, 5);

        Assert.Null(Axis.Z.Intersect(Edge.MakeLine(new Point3D(0, 0, 1), new Point3D(1, 0, 0)).LocationAt(1)));
        Assert.Null(Axis.Z.Intersect(Edge.MakeLine(new Point3D(1, 0, 1), new Point3D(1, 0, 2)).LocationAt(1)));
    }
}

public class PlaneWithShapesTests
{
    static readonly double R2 = Math.Sqrt(2) / 2;

    [Fact]
    public void PlaneInit()
    {
        // From a face
        var f = Face.MakeRect(1, 2).Located(new Location(new Point3D(1, 2, 3), new Vector3D(45, 0, 45)));
        var pFromFace = new Plane(f);
        var pln = pFromFace.ToPlane3D();
        foreach (var p in new[] { pFromFace, new Plane(face: f), new Plane(pln) })
        {
            Check.AlmostEqual(new Point3D(1, 2, 3), p.Origin, 6);
            Check.AlmostEqual(new Vector3D(R2, 0.5, 0.5), p.XDir, 6);
            Check.AlmostEqual(new Vector3D(-R2, 0.5, 0.5), p.YDir, 6);
            Check.AlmostEqual(new Vector3D(0, -R2, R2), p.ZDir, 6);
            Check.AlmostEqual(Check.LocationOf(f).Position, p.Location.Position, 6);
            Check.AlmostEqual(Check.LocationOf(f).Orientation, p.Location.Orientation, 6);
        }

        // From a face with an x direction
        f = Face.MakeRect(1, 2);
        Vector3D x = new Vector3D(1, 1, 0), y = new Vector3D(-1, 1, 0);
        foreach (var p in new[] { new Plane(f, x), new Plane(f, xDir: x), new Plane(face: f, xDir: x) })
        {
            Check.AlmostEqual(new Point3D(0, 0, 0), p.Origin, 6);
            Check.AlmostEqual(x.Normalized(), p.XDir, 6);
            Check.AlmostEqual(y.Normalized(), p.YDir, 6);
            Check.AlmostEqual(new Vector3D(0, 0, 1), p.ZDir, 6);
        }

        // From planar faces of surface types other than Geom_Plane: this loft's trapezoid faces are B-spline surfaces
        var loftedSolid = Solid.MakeLoft([Face.MakeRect(3, 1).SingleWire(), new Pos(0, 0, 1) * Face.MakeRect(1, 1).SingleWire()]);   // Rectangle(...).wire()
        (Direction3D X, Direction3D Y, Direction3D Z)[] expected =
        [
            (Axis.X.Direction, Axis.Z.Direction, -Axis.Y.Direction),   // trapezoid face, negative y
            (-Axis.X.Direction, Axis.Z.Direction, Axis.Y.Direction),   // trapezoid face, positive y
        ];
        var faces = loftedSolid.Faces().ParallelTo(Plane.XZ).OrderBy(Axis.Y);
        Assert.Equal(2, faces.Count);
        for (int i = 0; i < faces.Count; i++)
        {
            var p = new Plane(faces[i]);
            var face = faces[i];
            var centre = global::Kerf.Measurement.MeasureOperations.MassProperties(face, global::Kerf.Measurement.Dimension.Area).CentreOfMass;
            Check.AlmostEqual(centre, p.Origin, 6);
            Check.AlmostEqual(expected[i].X.ToVector(), p.XDir, 6);
            Check.AlmostEqual(expected[i].Y.ToVector(), p.YDir, 6);
            Check.AlmostEqual(expected[i].Z.ToVector(), p.ZDir, 6);
        }
    }

    [Fact]
    public void PlaneMulShape()
    {
        var p = Plane.XY.Offset(2);
        Assert.Equal(Objects.Box(1, 2, 3).Translate(new Vector3D(0, 0, 2)).BoundBox().Describe(),
                     (p * Objects.Box(1, 2, 3)).BoundBox().Describe());
    }

    [Fact]
    public void PlaneMethods()
    {
        var p = new Plane(new Point3D(0, 0, 0), xDir: new Vector3D(1, 0, 0), zDir: new Vector3D(0, 1, 0));
        var localBox = p.ToLocalCoords(Solid.MakeBox(1, 1, 1));
        var vertices = localBox.Vertices();
        double[][] targets =
        [
            [0, -1, 0], [0, 0, 0], [0, -1, 1], [0, 0, 1],
            [1, -1, 0], [1, 0, 0], [1, -1, 1], [1, 0, 1],
        ];
        Assert.Equal(targets.Length, vertices.Count);
        for (int i = 0; i < targets.Length; i++)
            Check.TupleAlmostEqual(targets[i], [vertices[i].X, vertices[i].Y, vertices[i].Z], 7);
    }

    [Fact]
    public void LocalizeVertex()
    {
        var random = new Random();
        double x = random.NextDouble(), y = random.NextDouble(), z = random.NextDouble();
        var vertex = Vertex.Make(x, y, z);
        Check.AlmostEqual(new Point3D(y, z, x), Plane.YZ.ToLocalCoords(vertex).Point, 5);
        Check.AlmostEqual(new Point3D(y, z, x), Plane.YZ.ToLocalCoords(vertex).Point, 5);
    }

    [Fact]
    public void FromNonPlanarFace()
    {
        Assert.IsType<Plane>(new Plane(Face.MakeRect(1, 1)));
        var cyl = Solid.MakeCylinder(1, 4).Faces().WhereNot(GeomType.Plane)[0];
        Assert.Throws<ArgumentException>(() => new Plane(cyl));
    }

    [Fact]
    public void ShiftOriginAxis()
    {
        var cyl = new Cylinder(1, 2, align: Align.Min);
        var top = cyl.Faces().OrderBy(Axis.Z)[^1];
        var pln = new Plane(top).ShiftOrigin(Axis.Z);
        var p = cyl - Ops.Extrude(pln * (new Pos(1, 1) * new Circle(0.5)), -2).Value;
        Check.AlmostEqual(Math.PI * (1 - 0.5 * 0.5) * 2, p.Volume, 5);
    }

    [Fact]
    public void ShiftOriginVertex()
    {
        var box = new Box(1, 1, 1, align: Align.Min);
        var front = box.Faces().OrderBy(Axis.X)[^1];
        var pln = new Plane(front).ShiftOrigin(front.Vertices().GroupBy(Axis.Z)[^1].OrderBy(Axis.Y)[^1]);
        var p = box - Ops.Extrude(pln * (new Pos(-0.5, 0.5) * new Circle(0.5)), -1).Value;
        Check.AlmostEqual(1 - Math.PI * 0.5 * 0.5 * 1, p.Volume, 5);
    }

    [Fact]
    public void ShiftOriginVector()
    {
        var b = Ops.Fillet(new Box(4, 4, 2).Edges().ParallelTo(Axis.Z), 0.5).Value;
        var top = b.Faces().OrderBy(Axis.Z)[^1];
        var reference = top.Edges().Where(GeomType.Circle).GroupBy(Axis.X)[^1].OrderBy(Axis.Y)[0].ArcCenter;
        var pln = new Plane(top, xDir: new Vector3D(0, 1, 0)).ShiftOrigin(reference);
        var p = b - Ops.Extrude(pln * (new Pos(0.5, 0.5) * new Rectangle(2, 2, align: Align.Min)), -1).Value;
        Check.AlmostEqual(b.Volume - 2 * 2 * 1, p.Volume, 5);
    }

    [Fact]
    public void ShiftOriginError() => Assert.Throws<ArgumentException>(() => Plane.XY.ShiftOrigin(Vertex.Make(1, 1, 1)));

    [Fact]
    public void PlaneIntersect()
    {
        var section = Plane.XY.Intersect(Solid.MakeBox(1, 2, 3, Plane.XY.Offset(-1.5)));
        Assert.Empty(section.Solids());
        Assert.Single(section.Faces());
        Check.AlmostEqual(2, section.SingleFace().Area);

        section = Plane.XY.Intersect(Solid.MakeBox(1, 2, 3, Plane.XY.Offset(-1.5)));
        Assert.Empty(section.Solids());
        Assert.Single(section.Faces());
        Check.AlmostEqual(2, section.SingleFace().Area);
    }
}

public class BoundBoxWithShapesTests
{
    static BoundingBox Box(Point3D min, Point3D max)
    {
        var box = new global::Kerf.Geometry.BoundingBox(new global::Kerf.Geometry.Point3D(min.X, min.Y, min.Z),
                                                       new global::Kerf.Geometry.Point3D(max.X, max.Y, max.Z));
        return box;
    }

    [Fact]
    public void ShapeConstructor()
    {
        var shape = Solid.MakeBox(1, 2, 3);
        var expected = shape.BoundBox();
        var bbox = shape.BoundBox();
        Check.AlmostEqual(expected.Min, bbox.Min, 7);
        Check.AlmostEqual(expected.Max, bbox.Max, 7);

        var expectedNonoptimal = shape.BoundBox(optimal: false);
        var bboxNonoptimal = shape.BoundBox(optimal: false);
        Check.AlmostEqual(expectedNonoptimal.Min, bboxNonoptimal.Min, 7);
        Check.AlmostEqual(expectedNonoptimal.Max, bboxNonoptimal.Max, 7);
    }

    [Fact]
    public void EmptyShapeConstructor()
    {
        var emptyShape = default(Solid);
        var expected = emptyShape.BoundBox();
        var bbox = emptyShape.BoundBox();
        Assert.True(bbox.IsEmpty);
        Assert.Equal(expected, bbox);   // the kernel's box has no corners when empty, where the layer's had the origin
        Assert.Throws<InvalidOperationException>(() => bbox.Min);
        Assert.Equal(expected.Size, bbox.Size);
    }

    [Fact]
    public void EmptyBoxPredicates()
    {
        var empty = default(Solid).BoundBox();
        var box = Box(new Point3D(0, 0, 0), new Point3D(1, 1, 1));
        Assert.False(empty.Covers(box));
        Assert.False(empty.CoveredBy(box));
        Assert.False(empty.Contains(box));
        Assert.False(empty.ContainsProperly(box));
        Assert.False(empty.Within(box));
        Assert.False(empty.IntersectsNear(box));
        Assert.True(empty.Disjoint(box));
        Assert.False(empty.Touches(box));
        Assert.False(empty.Overlaps(box));
    }

    [Fact]
    public void BasicBoundingBox()
    {
        var v = Vertex.Make(1, 1, 1);
        var v2 = Vertex.Make(2, 2, 2);
        Assert.IsType<BoundingBox>(v.BoundBox());
        Assert.IsType<BoundingBox>(v2.BoundBox());

        var bb1 = v.BoundBox().Union(v2.BoundBox());
        // OCC uses some approximations
        Check.AlmostEqual(1.0, bb1.Size.X, 1);
        Check.AlmostEqual(1.0, bb1.Measure, 5);

        // Adding to an existing bounding box
        var v0 = Vertex.Make(0, 0, 0);
        var bb2 = v0.BoundBox().Union(v.BoundBox());
        var bb3 = bb1.Union(bb2);
        Check.AlmostEqual(new Vector3D(2, 2, 2), bb3.Size, 7);
        Check.AlmostEqual(8, bb3.Measure, 5);

        bb3 = bb2.Including(new Point3D(3, 3, 3));
        Check.AlmostEqual(new Vector3D(3, 3, 3), bb3.Size, 7);
        bb3 = bb2.Including(new Point3D(3, 3, 3));
        Check.AlmostEqual(new Vector3D(3, 3, 3), bb3.Size, 7);

        // 2D bounding boxes
        bb2 = Vertex.Make(0, 0, 0).BoundBox().Union(Vertex.Make(3, 3, 0).BoundBox());
        Check.AlmostEqual(9, bb2.Measure, 5);

        // A box from a shape: low accuracy, as the box is a little larger than the shape
        var cylinder = Solid.MakeCylinder(1, 1);
        bb1 = cylinder.BoundBox(optimal: false);
        Check.AlmostEqual(new Vector3D(2, 2, 1), bb1.Size, 1);

        var small = Solid.MakeCylinder(0.5, 0.5).Translate(new Vector3D(0, 0, 0.1));
        bb2 = small.BoundBox(optimal: false);
        Assert.True(bb2.Within(bb1));
    }

    [Fact]
    public void BoundingBoxRepr() =>
        Assert.Equal("bbox: 0 <= x <= 1, 0 <= y <= 1, 0 <= z <= 1", Solid.MakeBox(1, 1, 1).BoundBox().Describe());

    [Fact]
    public void CenterOfBoundbox() => Check.AlmostEqual(new Point3D(0.5, 0.5, 0.5), Solid.MakeBox(1, 1, 1).BoundBox().Center(), 5);

    [Fact]
    public void CleanBoundbox()
    {
        var s = Solid.MakeSphere(3);
        Check.AlmostEqual(new Vector3D(6, 6, 6), s.BoundBox().Size, 5);
        // A mesh no longer changes the shape (the kernel meshes in an overlay): meshing leaves the box clean
        s.Triangulate(1e-3);
        Check.AlmostEqual(new Vector3D(6, 6, 6), s.BoundBox().Size, 5);
    }
}

public class OrientedBoundBoxWithShapesTests
{
    [Fact]
    public void SizeAndDiagonal()
    {
        var obb = Solid.MakeBox(1, 1, 1).OrientedBoundingBox();
        var size = obb.Size;
        Check.AlmostEqual(1.0, size.X, 6);
        Check.AlmostEqual(1.0, size.Y, 6);
        Check.AlmostEqual(1.0, size.Z, 6);
        Check.AlmostEqual(Math.Sqrt(3), obb.Diagonal, 6);
    }

    [Fact]
    public void Center()
    {
        var center = Solid.MakeBox(1, 1, 1).OrientedBoundingBox().Center();
        Check.AlmostEqual(0.5, center.X, 6);
        Check.AlmostEqual(0.5, center.Y, 6);
        Check.AlmostEqual(0.5, center.Z, 6);
    }

    [Fact]
    public void DirectionsAreUnitVectors()
    {
        var obb = (new Rotation(45, 45, 0) * Solid.MakeBox(1, 1, 1)).OrientedBoundingBox();
        foreach (var direction in new[] { obb.XDirection, obb.YDirection, obb.ZDirection })
            Check.AlmostEqual(1.0, direction.ToVector().Magnitude, 6);
    }

    [Fact]
    public void IsOutside()
    {
        var obb = Solid.MakeBox(1, 1, 1).OrientedBoundingBox();
        Assert.False(obb.IsOutside(obb.Center()));
        Assert.True(obb.IsOutside(new Point3D(10, 10, 10)));
    }

    [Fact]
    public void IsCompletelyInside()
    {
        var largeObb = Solid.MakeBox(2, 2, 2).OrientedBoundingBox();
        var smallObb = (new Pos(0.5, 0.5, 0.5) * Solid.MakeBox(1, 1, 1)).OrientedBoundingBox();
        Assert.True(largeObb.IsCompletelyInside(smallObb));
        Assert.False(smallObb.IsCompletelyInside(largeObb));
    }

    [Fact]
    public void InitFromBndObb()
    {
        var obb1 = Solid.MakeBox(1, 1, 1).OrientedBoundingBox();
        var obb2 = OrientedBoundBox.FromOrientedBox(obb1.Wrapped);
        Check.AlmostEqual(obb1.Diagonal, obb2.Diagonal, 6);
        Check.TupleAlmostEqual(Check.Of(obb1.Size), Check.Of(obb2.Size), 6);
        Check.TupleAlmostEqual(Check.Of(obb1.Center()), Check.Of(obb2.Center()), 6);
    }

    [Fact]
    public void Plane()
    {
        // The plane's orientation may not be consistent across platforms
        var rect = new Rotation(0, 0, 10) * Face.MakeRect(1, 2);
        var pln = rect.OrientedBoundingBox().Plane;
        Check.AlmostEqual(1.0, Math.Abs(pln.ZDir.Dot(new Vector3D(0, 0, 1))), 6);
        Assert.Contains(new[] { pln.XDir, pln.YDir, pln.ZDir },
                        d => Math.Abs(d.Dot(new Vector3D(1, 0, 0).Rotate(Axis.Z, 10))) > 0.999);
    }

    [Fact]
    public void Repr()
    {
        var rep = Solid.MakeBox(1, 1, 1).OrientedBoundingBox().ToString();
        Assert.Contains("OrientedBoundBox(center=(", rep);
        Assert.Contains("size=(", rep);
        Assert.Contains("plane=((", rep);

        const string n = @"([-\d\.]+)";
        var pattern = $@"OrientedBoundBox\(center=\({n}, {n}, {n}\), size=\({n}, {n}, {n}\), "
                      + $@"plane=\(\({n}, {n}, {n}\), \({n}, {n}, {n}\), \({n}, {n}, {n}\)\)\)";
        var m = System.Text.RegularExpressions.Regex.Match(rep, "^" + pattern);
        Assert.True(m.Success, "The ToString() didn't match the expected format.");
        double G(int i) => double.Parse(m.Groups[i].Value, System.Globalization.CultureInfo.InvariantCulture);
        Check.AlmostEqual(0.5, G(1), 6);
        Check.AlmostEqual(0.5, G(2), 6);
        Check.AlmostEqual(0.5, G(3), 6);
        Check.AlmostEqual(1.0, G(4), 6);
        Check.AlmostEqual(1.0, G(5), 6);
        Check.AlmostEqual(1.0, G(6), 6);
    }

    [Fact]
    public void RotatedCubeCorners()
    {
        var obb = (new Rotation(45, 45, 45) * Objects.Box(2, 2, 2)).OrientedBoundingBox();
        var corners = obb.Corners;
        Assert.Equal(8, corners.Count);
        var center = obb.Center();
        foreach (var corner in corners)
            Check.AlmostEqual(Math.Sqrt(3), (corner - center).Magnitude, 6);
    }

    [Fact]
    public void PlanarFaceCorners()
    {
        var corners = Face.MakeRect(2, 2).OrientedBoundingBox().Corners;
        var uniquePoints = corners.Select(pt => (Math.Round(pt.X, 6), Math.Round(pt.Y, 6), Math.Round(pt.Z, 6))).ToHashSet();
        Assert.Equal(4, uniquePoints.Count);

        // Orientation
        foreach (var pln in new[] { Kerf.Modeling.Plane.XY, Kerf.Modeling.Plane.XZ, Kerf.Modeling.Plane.YZ })
        {
            var rect = Face.MakeRect(1, 2, pln);
            var poly = Face.Make(Wire.MakePolygon(rect.OrientedBoundingBox().Corners));   // Polygon(*corners, align=None)
            var area = rect.Intersect(poly).Faces().Sum(f => f.Area);
            Check.AlmostEqual(rect.Area, area, 5);
        }

        foreach (var face in Objects.Box(1, 2, 3).Faces())
        {
            var poly = Face.Make(Wire.MakePolygon(face.OrientedBoundingBox().Corners));
            var area = face.Intersect(poly).Faces().Sum(f => f.Area);
            Check.AlmostEqual(face.Area, area, 5);
        }
    }

    [Fact]
    public void LineCorners()
    {
        var corners = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).OrientedBoundingBox().Corners;
        var uniquePoints = corners.Select(pt => (Math.Round(pt.X, 6), Math.Round(pt.Y, 6), Math.Round(pt.Z, 6))).ToHashSet();
        Assert.Equal(2, uniquePoints.Count);

        // Orientation
        foreach (var end in new Point3D[] { new Point3D(1, 0, 0), new Point3D(0, 1, 0), new Point3D(0, 0, 1) })
        {
            corners = Edge.MakeLine(new Point3D(0, 0, 0), end).OrientedBoundingBox().Corners;
            Assert.Equal(2, corners.Count);
            Assert.Contains(end, corners, PointComparer.Near);
        }
    }

    [Fact]
    public void Location()
    {
        var obb = Solid.MakeBox(1, 1, 1).OrientedBoundingBox();
        var loc = obb.Location;
        Assert.IsType<Kerf.Modeling.Location>(loc);

        var center = obb.Center();
        Check.AlmostEqual(center.X, loc.Position.X, 6);
        Check.AlmostEqual(center.Y, loc.Position.Y, 6);
        Check.AlmostEqual(center.Z, loc.Position.Z, 6);

        var plane = obb.Plane;
        Check.TupleAlmostEqual(Check.Of(plane.XDir), Check.Of(loc.XAxis.Direction), 6);
        Check.TupleAlmostEqual(Check.Of(plane.ZDir), Check.Of(loc.ZAxis.Direction), 6);
    }
}

public class VectorWithShapesTests
{
    [Fact]
    public void VectorConstructors()
    {
        var vertex = Vertex.Make(0, 0, 0).Moved(new Pos(0, 0, 10));
        Check.AlmostEqual(new Point3D(0, 0, 10), vertex.Point, 4);
    }

    [Fact]
    public void Intersect()
    {
        var v1 = new Point3D(1, 2, 3);
        Check.AlmostEqual(new Point3D(1, 2, 3), ((v1.Intersect(Solid.MakeBox(2, 4, 5))).SingleVertex().Point), 5);
        Assert.Empty(v1.Intersect(Solid.MakeBox(0.5, 0.5, 0.5)));
        Assert.Empty(Vertex.Make(-10, -10, -10).Intersect(Solid.MakeBox(0.5, 0.5, 0.5)));
    }
}

public class LocationWithShapesTests
{
    [Fact]
    public void MultIterable()
    {
        var locs = new Location(new Point3D(1, 2, 0)) * new GridLocations(4, 4, 2, 1);
        Check.AlmostEqual(new Point3D(-1, 2, 0), locs[0].Position, 5);
        Check.AlmostEqual(new Point3D(3, 2, 0), locs[1].Position, 5);
    }

    [Fact]
    public void Intersection()
    {
        var e = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 1, 1));
        var l0 = e.LocationAt(0);
        var l1 = e.LocationAt(1);
        Assert.Null(l0.Intersect(l1));
        Assert.Equal(l1, l1.Intersect(l1));

        Check.AlmostEqual(new Point3D(1, 1, 1), Assert.IsType<Point3D>(l1.Intersect(new Point3D(1, 1, 1))), 5);
        Assert.Equal(l1, Assert.IsType<Location>(l1.Intersect(new Axis(new Point3D(0.5, 0.5, 0.5), new Vector3D(1, 1, 1)))));

        var p = Plane.XY.Rotated(new Vector3D(45, 0, 0)).ShiftOrigin(new Point3D(1, 0, 0));
        var l = new Location(new Point3D(1, 0, 0), new Vector3D(1, 0, 0), 45);
        var i = Assert.IsType<Location>(l.Intersect(p));
        Check.AlmostEqual(new Point3D(1, 0, 0), i.Position, 5);
        Check.AlmostEqual(l.Orientation, i.Orientation, 5);

        var b = Solid.MakeBox(1, 1, 1);
        l = new Location(new Point3D(0.5, 0.5, 0.5), new Vector3D(1, 0, 0), 45);
        var vertex = Check.Is<Vertex>((l.Intersect(b)).SingleVertex());
        Check.AlmostEqual(new Point3D(0.5, 0.5, 0.5), vertex.Point, 5);

        var e1 = Edge.MakeLine(new Point3D(0, -1, 0), new Point3D(2, 1, 0));
        var e2 = Edge.MakeLine(new Point3D(0, 1, 0), new Point3D(2, -1, 0));
        var e3 = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(2, 0, 0));
        var found = e1.Intersect(e2, e3);
        Check.AlmostEqual(new Point3D(1, 0, 0), (Check.Is<Vertex>(found[0]).Point), 5);

        var e4 = Edge.MakeLine(new Point3D(1, -1, 0), new Point3D(1, 1, 0));
        var e5 = Edge.MakeLine(new Point3D(2, -1, 0), new Point3D(2, 1, 0));
        Assert.Empty(e3.Intersect(e4, e5));

        Assert.Empty(b.Intersect(b.Moved(new Pos(10))));

        // Common vertices (endpoint-endpoint contacts are touches, not intersections)
        e1 = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        e2 = Edge.MakeLine(new Point3D(1, 0, 0), new Point3D(1, 1, 0));
        e3 = Edge.MakeLine(new Point3D(1, 0, 0), new Point3D(2, 0, 0));
        found = e1.Intersect([e2], includeTouched: true);
        Assert.Single(found.Vertices());
        var v = found.SingleVertex();
        Assert.Equal((1.0, 0.0, 0.0), (v.X, v.Y, v.Z));
        found = e1.Intersect([e3], includeTouched: true);
        Assert.Single(found.Vertices());
        v = found.SingleVertex();
        Assert.Equal((1.0, 0.0, 0.0), (v.X, v.Y, v.Z));

        // With a plane
        e1 = Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(2, 0, 0));
        var p1 = Plane.YZ.Offset(1);
        found = e1.Intersect(p1);
        Assert.Single(found.Vertices());
        v = found.SingleVertex();
        Assert.Equal((1.0, 0.0, 0.0), (v.X, v.Y, v.Z));

        e2 = Edge.MakeLine(p1.Origin, p1.Origin + 2 * p1.XDir);
        found = e2.Intersect(p1);
        Assert.Equal(2, found.Vertices().Count);
        Assert.Single(found.Edges());
        Check.AlmostEqual(2, found.SingleEdge().Length, 5);

        Assert.Throws<ArgumentException>(() => e1.Intersect("line"));
    }
}
