// Ported from the fillet_2d tests of build123d: tests/test_direct_api/test_wire.py (TestWire, TestWireFilletHelpers,
// TestWireValidation), test_face.py, test_fillet_2d_regressions.py, the rounded rectangle of test_build_sketch.py,
// test_algebra.py's test_fillet_2d, and the filleted profiles of test_mixin1_d.py's test_offset_2d and test_solid.py's
// test_extrude_taper_draft_falls_back_to_loft. The fillet operation in a BuildSketch is written as Face.Fillet2D of
// each face with the vertices of it that are given, as fillet() does in Python, followed by the clean the builder does,
// and make_face as a face of the combined edges.
// Omitted:
// - TestWireFilletHelpers.test_analyze_wire_fillet_corner_vertex_not_degree_two,
//   test_fillet_wire_corner_failure_when_solver_fails, test_fillet_wire_corner_failure_when_closed_wire_becomes_open:
//   mocks.
// - test_extrude_taper_draft_falls_back_to_loft, test_offset_2d: only the filleted profiles are checked here (the
//   rest belongs to the Solid and Wire tests).
using Build123d.Occt;

namespace Build123d.Tests.Objects;

public class Fillet2DTests
{
    /// <summary>
    /// build123d's fillet(vertices, radius) in a BuildSketch: each face filleted at those of the vertices on it, and the
    /// sketch cleaned as the builder does when it replaces its sketch with the result.
    /// </summary>
    private static Sketch Fillet(Shape sketch, IEnumerable<Vertex> vertices, double radius)
    {
        var list = vertices.ToList();
        return new Sketch(sketch.Faces().Select(f =>
        {
            var onFace = list.Where(v => f.Vertices().Contains(v)).ToList();
            return (Shape)(onFace.Count > 0 ? f.Fillet2D(radius, onFace) : f);
        })).Clean();
    }

    /// <summary>build123d's make_face of the edges.</summary>
    private static Face MakeFace(params Shape[] curves) => new(Wire.Combine(curves.SelectMany(c => c.Edges()))[0]);

    // ---- test_wire.py ----

    [Fact]
    public void WireFillet2D()
    {
        var square = Wire.MakeRect(1, 1);
        var squaroid = square.Fillet2D(0.1, square.Vertices());
        Check.AlmostEqual(4 * (1 - 2 * 0.1) + 2 * Math.PI * 0.1, squaroid.Length, 5);

        var straightWire = new Wire([Edge.MakeLine((0, 0), (1, 0)), Edge.MakeLine((1, 0), (2, 0))]);
        var straightVertex = straightWire.Vertices().SortByDistance((1, 0, 0))[0];
        var unmodifiedWire = straightWire.Fillet2D(0.1, [straightVertex]);
        Check.AlmostEqual(straightWire.Length, unmodifiedWire.Length, 5);
        Assert.Equal(2, unmodifiedWire.Edges().Count);

        Assert.Throws<InvalidOperationException>(() => new Wire().Fillet2D(0.1, square.Vertices()));
    }

    [Fact]
    public void FilletNonPlanar()
    {
        var wire = new Wire([Edge.MakeLine((0, 0, 0), (1, 0, 0)), Edge.MakeLine((1, 0, 0), (1, 1, 0)), Edge.MakeLine((1, 1, 0), (0, 0, 1))]);
        Assert.Throws<ArgumentException>(() => wire.Fillet2D(0.1, [wire.Vertices()[1]]));
    }

    [Fact]
    public void BadVertex()
    {
        var rect = Face.MakeRect(2, 2).SingleWire();
        Assert.Throws<ArgumentException>(() => rect.Fillet2D(0.2, [new Vertex(0, 0, 0)]));
    }

    [Fact]
    public void FilletNonPlaneXY()
    {
        var wireLoc = (Plane.XZ * new Pos(1, 2) * new Rectangle(2, 2)).SingleWire();
        var filletedLoc = wireLoc.Fillet2D(0.2, wireLoc.Vertices());
        Assert.Equal(4, filletedLoc.Edges().FilterBy(GeomType.Circle).Count);

        var wireFace = Face.MakeRect(2, 2, Plane.XZ).SingleWire();
        var filletedFace = wireFace.Fillet2D(0.2, wireFace.Vertices());
        Assert.Equal(4, filletedFace.Edges().FilterBy(GeomType.Circle).Count);
    }

    [Fact]
    public void Fillet2DVertexNotOnWire()
    {
        var wire = Wire.MakeRect(10, 10);
        var e = Assert.Throws<ArgumentException>(() => wire.Fillet2D(1, [new Vertex(100, 100, 0)]));
        Assert.Contains("Could not find fillet vertex", e.Message);
    }

    [Fact]
    public void AnalyzeWireFilletCornerMissingVertex()
    {
        var e = Assert.Throws<ArgumentException>(() => WireFillet.Analyze(Wire.MakeRect(1, 1), new Vertex(10, 10, 0)));
        Assert.Contains("Could not find shared vertex on wire", e.Message);
    }

    [Fact]
    public void SolveWireFilletCornerNoSolution()
    {
        var wire = Wire.MakeRect(1, 1);
        var corner = WireFillet.Analyze(wire, wire.Vertices()[0]);
        Assert.Null(WireFillet.Solve(corner, 10));
    }

    [Fact]
    public void WireFilletCornerIsTangentContinuousStraightWire()
    {
        var wire = new Wire([Edge.MakeLine((0, 0), (1, 0)), Edge.MakeLine((1, 0), (2, 0))]);
        var corner = WireFillet.Analyze(wire, wire.Vertices().SortByDistance((1, 0, 0))[0]);
        Assert.True(WireFillet.IsTangentContinuous(corner));
    }

    [Fact]
    public void WireFilletCornerIsNotTangentContinuousOnRoundedCutTips()
    {
        Shape sketch = new RectangleRounded(20, 10, 2);
        sketch -= sketch.Edges().FilterBy(GeomType.Circle).Select(e => (Shape?)(new Location(e.ArcCenter) * new Circle(2)));
        var wire = sketch.SingleWire();
        foreach (Vector point in new Vector[] { (-10, -3, 0), (-10, 3, 0), (-8, 5, 0) })
        {
            var corner = WireFillet.Analyze(wire, wire.Vertices().SortByDistance(point)[0]);
            Assert.False(WireFillet.IsTangentContinuous(corner));
        }
    }

    // ---- test_face.py ----

    [Fact]
    public void Fillet2DMixedProfileCase()
    {
        var sketch = new Sketch() + new Rectangle(10, 20) + new Ellipse(20, 5);
        var vertex = sketch.Vertices().GroupBy(Axis.X)[0].SortBy(Axis.Y)[0];
        var filleted = sketch.Faces()[0].Fillet2D(1.0, [vertex]);
        Assert.True(filleted.IsValid);
        Assert.True(filleted.Area > 0);
        Assert.True(filleted.Edges().FilterBy(GeomType.Circle).Count >= 1);
    }

    [Fact]
    public void Fillet2DMixedProfileRegression()
    {
        var sketch = new Sketch() + new Rectangle(10, 20) + new Ellipse(20, 5);
        var vertex = sketch.Vertices().GroupBy(Axis.X)[1].SortBy(Axis.Y)[1];
        var filleted = sketch.Faces()[0].Fillet2D(1.0, [vertex]);
        Assert.True(filleted.IsValid);
        Assert.True(filleted.Area > 0);
        Assert.True(filleted.Edges().FilterBy(GeomType.Circle).Count >= 1);
    }

    [Fact]
    public void Fillet2DHolesRegression()
    {
        // Ellipse(x_radius=74 / 2, y_radius=54 / 2) with GridLocations(49, 32, 2, 2) of Circle(12 / 2) subtracted
        Location[] grid = [new Pos(-24.5, -16), new Pos(-24.5, 16), new Pos(24.5, -16), new Pos(24.5, 16)];
        var sketch = new Ellipse(74 / 2.0, 54 / 2.0) - grid.Select(l => (Shape?)(l * new Circle(12 / 2.0)));
        var vertex = sketch.Vertices().SortByDistance((30, 20))[0];
        var original = sketch.SingleFace();
        var filleted = original.Fillet2D(2.0, [vertex]);
        Assert.True(filleted.IsValid);
        Assert.True(filleted.Area > 0);
        Assert.True(filleted.Area < original.Area);
    }

    [Fact]
    public void FilletGeom2dGccCirc2d2TanRadAlgorithm()
    {
        var r = new Rectangle(6, 6) - new Pos(1, 1) * new Circle(2) - new Pos(3, 3) * new Rectangle(4, 4);
        var filleted = r.SingleFace().Fillet2D(0.2, r.Vertices());
        Assert.Equal(6, filleted.Edges().FilterBy(GeomType.Circle).Count);
    }

    [Fact]
    public void Fillet2DNoVerticesReturnsSelf()
    {
        var face = new Rectangle(10, 10).SingleFace();
        Assert.Same(face, face.Fillet2D(1, []));
    }

    // ---- test_build_sketch.py, test_algebra.py ----

    [Fact]
    public void RectangleRounded()
    {
        var r = new RectangleRounded(20, 10, 1);
        Assert.Equal(20, r.Width);
        Assert.Equal(10, r.RectangleHeight);
        Assert.Equal(0, r.Rotation);
        Assert.Equal(1, r.Radius);
        Assert.Equal((Alignment)(Align.Center, Align.Center), r.Align);
        Assert.Equal(Mode.Add, r.Mode);
        Check.AlmostEqual(20 * 10 - 4 * 1 + Math.PI * 1, r.Area, 5);
        Assert.Equal(new Vector(0, 0, 1), r.Faces()[0].NormalAt());

        Assert.Throws<ArgumentException>(() => new RectangleRounded(20, 10, 5));
        Assert.Throws<ArgumentException>(() => new RectangleRounded(10, 20, 5));
    }

    [Fact]
    public void AlgebraFillet2D()
    {
        var r = new Rectangle(1, 2);
        var c = Fillet(r, r.Vertices(), 0.2);
        Check.AlmostEqual(2.0, r.Area, 6);
        Check.AlmostEqual(1.965663, c.Area, 4);
    }

    // ---- the filleted profiles of test_mixin1_d.py and test_solid.py ----

    [Fact]
    public void OffsetBaseWire()
    {
        var baseWire = Wire.MakePolygon([(0, 0), (1, 0), (1, 1)], close: false);
        var corner = baseWire.Vertices().GroupBy(Axis.Y)[0].SortBy(Axis.X)[^1];
        baseWire = baseWire.Fillet2D(0.4, [corner]);
        Assert.Equal(3, baseWire.Edges().Count);
        Assert.Single(baseWire.Edges().FilterBy(GeomType.Circle));
        var offsetWire = (Wire)baseWire.Offset2D(0.1, side: Side.Left);
        Assert.True(offsetWire.IsClosed);
        Assert.Equal(6, offsetWire.Edges().FilterBy(GeomType.Line).Count);
        Assert.Equal(2, offsetWire.Edges().FilterBy(GeomType.Circle).Count);
    }

    [Fact]
    public void CrossRecess()
    {
        // A cross with rounded inner corners, as cut for a Phillips recess
        var cross = (new Rectangle(5.3, 5.3 / 6) + new Rectangle(5.3 / 6, 5.3)).SingleFace();
        var inner = new ShapeList<Vertex>(cross.Vertices()).SortByDistance((0, 0, 0)).Take(4);
        var recess = cross.Fillet2D(5.3 / 3, inner);
        Assert.True(recess.IsValid);
        Assert.Equal(4, recess.Edges().FilterBy(GeomType.Circle).Count);
    }

    // ---- test_fillet_2d_regressions.py ----

    private const double Radius = 5;   // fillet radius
    private const double EdgeRadius = 5.001;   // arc/spline/ellipse 'radius'

    /// <summary>The two-arc profile of the regression tests, with the top edge made by <paramref name="top"/>.</summary>
    private static Face Profile(Func<JernArc, JernArc, Edge> top)
    {
        var ln1 = new JernArc((0, 0), (1, 0), 20, 90);
        var ln2 = new JernArc((0, 10), (1, 0), 10, 90);
        return MakeFace(ln1, ln2, top(ln1, ln2), new Line(ln1.PositionAt(0), ln2.PositionAt(0)));
    }

    private static Face LineProfile() => Profile((ln1, ln2) => new Line(ln1.PositionAt(1), ln2.PositionAt(1)));

    [Fact]
    public void FilletLineFullEdgeConsumed()
    {
        // A fillet consuming a whole line between two circular arcs (a ChFi2d_FilletAlgo failure made an open wire)
        foreach (var allVerts in new[] { false, true })
        {
            var sk = LineProfile();
            var vrts = allVerts ? sk.Vertices() : sk.Vertices().SortBy(Axis.Y).TakeLast(2);
            Assert.Equal(4, Fillet(sk, vrts, Radius).Edges().Count);
        }
    }

    [Fact]
    public void FilletLineAllEdgesConsumed()
    {
        // All the edges of the sketch consumed ("Face can only be created with closed wires")
        var cr1 = new CenterArc((14.142135623730951, 14.999999999999996, 0.0), Radius, 0, 360);
        var cn1 = new ConstrainedLines(cr1.SingleEdge(), (0, 20), selector: lines => [lines[1]]).SingleEdge();
        var cut = new Plane(cn1.PositionAt(0), xDir: cn1 % 1) * new Rectangle(20, 20, align: (Align.Center, Align.Min));
        var sk = LineProfile() - cut;
        Assert.Single(Fillet(sk, sk.Vertices(), Radius).Edges());
    }

    [Fact]
    public void FilletArcCutoutAllEdgesConsumed()
    {
        foreach (var allVerts in new[] { false, true })
        {
            var ln1 = new JernArc((0, 0), (1, 0), 20, 90);
            var ln2 = new JernArc((0, 10), (1, 0), 10, 90);
            var ln3 = new RadiusArc(ln1.PositionAt(1), ln2.PositionAt(1), EdgeRadius);
            var profile = MakeFace(ln1, ln2, ln3, new Line(ln1.PositionAt(0), ln2.PositionAt(0)));
            var arc3tan = new ConstrainedArcs(ln1, ln2, ln3, sagitta: Sagitta.Both, selector: arcs => [arcs.SortBy(Axis.Y)[0]],
                                              mode: Mode.Private);
            var circle = new ConstrainedArcs(arc3tan.SingleEdge(), center: ln1.PositionAt(0));
            var sk = profile - MakeFace(circle);
            var vrts = allVerts ? sk.Vertices() : sk.Vertices().SortBy(Axis.Y).TakeLast(2);
            Assert.Equal(allVerts ? 1 : 4, Fillet(sk, vrts, Radius).Edges().Count);
        }
    }

    [Fact]
    public void FilletSplineFullEdgeConsumed()
    {
        // A fillet consuming a whole spline between two circular arcs
        var sk = Profile((ln1, ln2) =>
        {
            var cln3 = new RadiusArc(ln1.PositionAt(1), ln2.PositionAt(1), EdgeRadius, mode: Mode.Private);
            return new Spline(cln3.PositionAt(0), cln3.PositionAt(0.5), cln3.PositionAt(1));
        });
        Assert.Equal(4, Fillet(sk, sk.Vertices(), Radius).Edges().Count);
    }

    [Fact]
    public void FilletEllipseCutout()
    {
        // An ellipse cutout once left a short dangling edge and an open wire
        var sk = LineProfile() - new Pos(15, 20) * new Ellipse(10, 5);
        var vrts = sk.Vertices().SortBy(Axis.Y).TakeLast(2);
        Assert.Equal(4, Fillet(sk, vrts, Radius).Edges().Count);
    }

    // ---- the record, as Python build123d keeps it ----

    /// <summary>
    /// For each vertex then edge of <paramref name="original"/>: how many shapes it was modified into and generated, and
    /// whether it was removed; and for each vertex then edge of the result, whether it is new.
    /// </summary>
    private static string Record(Shape original, Shape result)
    {
        var h = result.History!;
        var inputs = original.Vertices().Cast<Shape>().Concat(original.Edges()).Select(s =>
            $"{h.Modified(s.TopoShape).Length}{h.Generated(s.TopoShape).Length}{(h.IsRemoved(s.TopoShape) ? 1 : 0)}");
        var news = result.Vertices().Cast<Shape>().Concat(result.Edges()).Select(s => h.IsNew(s.TopoShape) ? "1" : "0");
        return $"{string.Join(" ", inputs)} new {string.Concat(news)}";
    }

    [KnownOcctNetDifferenceFact("the wire's first vertex has two modified shapes where native gives one (docs/design.md)")]
    public void FilletRecord()
    {
        // The filleted vertex is removed (its arc counts as new), the edges beside it are modified and the rest untouched
        var w = Wire.MakePolygon([(0, 0), (3, 0), (4, 2), (1, 5), (-1, 2)], close: true);
        Assert.Equal("100 001 100 100 100 100 100 100 100 100 new 011000010000", Record(w, w.Fillet2D(0.7, [w.Vertices()[1]])));
        Assert.Equal("001 001 001 001 001 100 100 100 100 100 new 11111111110101010101", Record(w, w.Fillet2D(0.7, w.Vertices())));

        // A whole edge consumed: it is removed
        var ln1 = new JernArc((0, 0), (1, 0), 20, 90);
        var ln2 = new JernArc((0, 10), (1, 0), 10, 90);
        var w2 = Wire.Combine([ln1, ln2, new Line(ln1.PositionAt(1), ln2.PositionAt(1)), new Line(ln1.PositionAt(0), ln2.PositionAt(0))])[0];
        Assert.Equal("100 001 001 100 100 001 100 100 new 0111001100", Record(w2, w2.Fillet2D(5, w2.Vertices().SortBy(Axis.Y).TakeLast(2))));

        var face = Face.MakeRect(4, 3);
        Assert.Equal("001 001 001 001 100 100 100 100 new 1111111111111111", Record(face, face.Fillet2D(0.5, face.Vertices())));

        // The record of a single corner keeps what the vertex generated as well as its removal
        var vertex = w.Vertices()[1];
        var corner = WireFillet.FilletCorner(w, vertex, 0.7).History!;
        Assert.Equal(3, corner.Generated(vertex.TopoShape).Length);
        Assert.True(corner.IsRemoved(vertex.TopoShape));
    }
}
