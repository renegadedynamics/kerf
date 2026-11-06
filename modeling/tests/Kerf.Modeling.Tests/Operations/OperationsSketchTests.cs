// Ported from build123d tests/test_build_sketch.py, the operation tests rewritten in algebra form, with curve objects
// (Line, Polyline, CenterArc, ThreePointArc) made with the direct API. Omitted:
// - The builder and object tests (TestAlign, TestBaseSketchObjectFirewall, TestBuildSketch, TestBuildOnPlanes,
//   TestBuildSketchExceptions, TestBuildSketchObjects other than test_offset's first half, test_invalid_slots).
// - TestUpSideDown.test_flip_face's second half: insert into a builder.
// - test_make_face and test_make_hull's builder cases, test_trace's pending-edge case.
// - test_full_round's full_round(ShapeList): a type error the compiler rules out.
using static Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Operations;

using Kerf.Modeling;

public class OperationsSketchTests
{
    [Fact]
    public void FilletedFaceIsUpsideDown()
    {
        var f1 = Face.Make(Wire.MakePolygon([new Point3D(1, 0, 0), new Point3D(1.5, 0.5, 0), new Point3D(1, 2, 0), new Point3D(3, 1, 0), new Point3D(2, 0, 0), new Point3D(1, 0, 0)]));
        var f = ((Sketch)Fillet(f1.Vertices()[2..4], 0.2).Value - new Pos(1.8, 1.2, 0) * new Rectangle(0.2, 0.4)
                 - new Pos(2, 0.5, 0) * new Circle(0.2)).Faces()[0];
        Assert.True(f.NormalAt().Z < 0);   // Up-side-down, as the face it came from
    }

    [Fact]
    public void MakeHullFlipped()
    {
        var basePlan = new Circle(55.0 / 2) + new Pos(0, 125) * new Circle(30.0 / 2);
        var baseHull = MakeHull(basePlan.Edges());
        foreach (var face in baseHull.Faces())
            Assert.True(face.NormalAt().Z > 0);
    }

    [Fact]
    public void MakeFaceFlipped()
    {
        var wire = Wire.MakePolygon([new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)]);
        var sketch = MakeFace(wire.Edges());
        Assert.True(sketch.Faces()[0].NormalAt().Z > 0);
    }

    [Fact]
    public void OffsetSketchFace()
    {
        var test = Offset(new Circle(10).Faces()[0], 1).Value;
        Check.AlmostEqual(11, test.Edges()[0].Radius);
    }

    [Fact]
    public void MakeFaceErrors()
    {
        Assert.Throws<ArgumentException>(() => MakeFace((IEnumerable<Shape>)null!));
        Assert.True(MakeFace(Array.Empty<Shape>()).IsEmpty);
    }

    [Fact]
    public void MakeFaceAcceptsCurve()
    {
        double length = 80.0, width = 60.0;
        var lines = new Curve([
            Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(length, 0, 0)),
            Edge.MakeLine(new Point3D(length, 0, 0), new Point3D(length, width, 0)),
            Edge.MakeThreePointArc(new Point3D(length, width, 0), new Point3D(width, width * 1.5, 0), new Point3D(0.0, width, 0)),
            Edge.MakeLine(new Point3D(0.0, width, 0), new Point3D(0.0, 0.0, 0)),
        ]);
        var sketch = MakeFace(lines);
        Assert.IsType<Sketch>(sketch);
        Assert.Single(sketch.Faces());
    }

    [Fact]
    public void MakeHullOfEdges()
    {
        Edge[] edges =
        [
            Edge.MakeCircle(1), Edge.MakeCircle(0.5, new Plane(new Point3D(1, 1.5, 0))), Edge.MakeLine(new Point3D(0.0, 2, 0), new Point3D(-1, 3.0, 0)),
        ];
        Check.AlmostEqual(7.2582, MakeHull(edges).Area, 4);

        var circles = new Pos(-10, 0) * new Circle(10) + new Pos(10, 0) * new Circle(7);
        Check.AlmostEqual(577.8808, MakeHull(circles.Edges()).Area, 4);

        Assert.Throws<ArgumentException>(() => MakeHull((IEnumerable<Edge>)null!));
    }

    [Fact]
    public void TraceLines()
    {
        var test = Trace(Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(10, 0, 0)), lineWidth: 1);
        Assert.Single(test.Faces());
        Check.AlmostEqual(10, test.Area, 5);

        Assert.Throws<ArgumentException>(() => Trace((IEnumerable<Shape>)null!));

        var line = Wire.MakePolygon([new Point3D(0, 0, 0), new Point3D(10, 10, 0), new Point3D(20, 10, 0)], close: false);
        Assert.Single(Trace(line, 4).Faces());
    }

    [Fact]
    public void FullRoundEdges()
    {
        var trap = new Trapezoid(0.5, 1, 90 - 8);
        var test = FullRound(trap.Edges().OrderBy(Axis.Y)[^1], trap);
        Assert.True(test.SingleFace().Area < trap.SingleFace().Area);

        Assert.Throws<ArgumentException>(() => FullRound(trap.Edges().OrderBy(Axis.X)[^1], trap));
        Assert.Throws<ArgumentException>(() => FullRound(Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0)), Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(1, 0, 0))));

        var l1 = Edge.MakeSpline([new Point3D(-1, 0, 0), new Point3D(1, 0, 0)], tangents: [new Vector3D(0, -8, 0), new Vector3D(0, 8, 0)], scale: true);
        var l2 = Edge.MakeLine(l1.PositionAt(0), l1.PositionAt(1));
        var face = Face.Make(Wire.Make([l1, l2]));
        Assert.Throws<ArgumentException>(() => FullRound(face.Edges()[0], face));

        var positive = FullRound(trap.Edges().OrderBy(ShapeKey.Length)[0], trap);
        var negative = FullRound(trap.Edges().OrderBy(ShapeKey.Length)[0], trap, invert: true);
        Assert.True(negative.SingleFace().Area < positive.SingleFace().Area);
        // build123d's (scipy's Voronoi) results
        Check.AlmostEqual(0.35458548403442275, positive.Area, 5);
        Check.AlmostEqual(0.3052168041379631, negative.Area, 5);

        var rect = new Rectangle(34, 10);
        var convexRect = FullRound(rect.Edges().GroupBy(Axis.X).First()[0], rect);
        var concaveRect = FullRound(rect.Edges().GroupBy(Axis.X).First()[0], rect, invert: true);
        Assert.True(convexRect.Area < rect.Area);
        Assert.True(concaveRect.Area < convexRect.Area);
        Check.AlmostEqual(329.2704811215115, convexRect.Area, 3);
        Check.AlmostEqual(251.36350580659322, concaveRect.Area, 3);

        var tri = new Triangle(sideA: 10, sideB: 10, sideC: 10);
        var triRound = FullRound(tri.Edges().OrderBy(Axis.X)[0], tri);
        Assert.True(triRound.Area < tri.Area);
        Check.AlmostEqual(31.887519392726578, triRound.Area, 3);

        // Flipping the face
        var flipped = (Face)(-Face.MakeRect(34, 10));
        var rounded = FullRound(flipped.Edges().GroupBy(Axis.X).First()[0], flipped).SingleFace();
        Assert.Equal(flipped.NormalAt(), rounded.NormalAt());
    }

    [Fact]
    public void VoronoiVerticesOfASquare()
    {
        // The four corners are cocircular: one Voronoi vertex, at the centre
        var vertices = Kerf.Modeling.Operations.VoronoiVertices([(0, 0), (1, 0), (1, 1), (0, 1)]);
        Assert.Single(vertices);
        Check.AlmostEqual(0.5, vertices[0].X);
        Check.AlmostEqual(0.5, vertices[0].Y);
    }
}
