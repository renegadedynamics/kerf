// Ported from build123d tests/test_direct_api/test_solid.py. Box, Torus, Rectangle, Circle and Spline are made with the
// direct API. Omitted:
// - Part of test_extrude (Solid.extrude of a Solid) and test_constructor (Solid(foo="bar")): type errors the compiler
//   rules out.
// - TestSolidThicken (both cases), TestSolidDraft.test_adddone_failure_raises_draftangleerror and
//   test_build_failure_raises_draftangleerror: mock the OCP builders.
using Kerf.Topology;

namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class SolidTests
{
    [Fact]
    public void MakeSolid()
    {
        var boxFaces = Solid.MakeBox(1, 1, 1).Faces();
        var boxShell = new Shell(boxFaces);
        var box = new Solid(boxShell);
        Check.AlmostEqual(6, box.Area, 5);
        Check.AlmostEqual(1, box.Volume, 5);
        Assert.True(box.IsValid);
    }

    [Fact]
    public void Extrude()
    {
        var v = Edge.Extrude(new Vertex(1, 1, 1), (0, 0, 1));
        Check.AlmostEqual(1, v.Length, 5);

        var e = Face.Extrude(Edge.MakeLine((2, 1), (2, 0)), (0, 0, 1));
        Check.AlmostEqual(1, e.Area, 5);

        var w = Shell.Extrude(new Wire([Edge.MakeLine((1, 1), (0, 2)), Edge.MakeLine((1, 1), (1, 0))]), (0, 0, 1));
        Check.AlmostEqual(1 + Math.Sqrt(2), w.Area, 5);

        var f = Solid.Extrude(Face.MakeRect(1, 1), (0, 0, 1));
        Check.AlmostEqual(1, f.Volume, 5);

        var s = Compound.Extrude(
            new Shell(Solid.MakeBox(1, 1, 1).Located(new Location((-2, 1, 0))).Faces().SortBy(new Axis((0, 0, 0), (1, 1, 1)))[^2..]),
            (0.1, 0.1, 0.1));
        Check.AlmostEqual(0.2, s.Volume, 5);
    }

    public static TheoryData<double, double, bool> TaperCases()
    {
        var data = new TheoryData<double, double, bool>();
        foreach (var z in new[] { 2.0, -2.0 })
            foreach (var taper in new[] { 10.0, -10.0 })
                foreach (var flipped in new[] { false, true })
                    data.Add(z, taper, flipped);
        return data;
    }

    [Theory]
    [MemberData(nameof(TaperCases))]
    public void ExtrudeTaper(double directionZ, double taper, bool flipped)
    {
        const double a = 1;
        var rect = Face.MakeRect(a, a);
        var face = flipped ? (Face)(-rect) : rect;
        var direction = new Vector(0, 0, directionZ);
        var offsetAmt = -direction.Length * Math.Tan(taper * Math.PI / 180);
        var taperSolid = Solid.ExtrudeTaper(face, direction, taper);
        // V = 1/3 × h × (a² + b² + ab)
        var h = direction.Length;
        var b = a + 2 * offsetAmt;
        var v = h * (a * a + b * b + a * b) / 3;
        Check.AlmostEqual(v, taperSolid.Volume, 5);
        var bbox = taperSolid.BoundingBox();
        var size = Math.Max(1, b) / 2;
        if (direction.Z > 0)
        {
            Check.AlmostEqual((-size, -size, 0), bbox.Min, 1);
            Check.AlmostEqual((size, size, h), bbox.Max, 1);
        }
        else
        {
            Check.AlmostEqual((-size, -size, -h), bbox.Min, 1);
            Check.AlmostEqual((size, size, 0), bbox.Max, 1);
        }
    }

    [Fact]
    public void ExtrudeTaperDraftFallsBackToLoft()
    {
        // A cross with rounded inner corners, as cut for a Phillips recess, cannot be drafted at this angle but can be
        // lofted
        var cross = (new Rectangle(5.3, 5.3 / 6) + new Rectangle(5.3 / 6, 5.3)).SingleFace();
        var inner = new ShapeList<Vertex>(cross.Vertices()).SortByDistance((0, 0, 0))[..4];
        var recess = cross.Fillet2D(5.3 / 3, inner);
        var tapered = Solid.ExtrudeTaper(recess, (0, 0, -3.27), 20);
        Assert.True(tapered.IsValid);
        Check.AlmostEqual(-3.27, tapered.BoundingBox().Min.Z, 5);
    }

    [Fact]
    public void ExtrudeTaperWithHole()
    {
        var rectHole = Face.MakeRect(1, 1).MakeHoles([Wire.MakeCircle(0.25)]);
        var direction = new Vector(0, 0, 0.5);
        const double taper = 10;
        var taperSolid = Solid.ExtrudeTaper(rectHole, direction, taper);
        var offsetAmt = -direction.Length * Math.Tan(taper * Math.PI / 180);
        var hole = taperSolid.Edges().FilterBy(GeomType.Circle).SortBy(Axis.Z)[^1];
        Check.AlmostEqual(0.25 - offsetAmt, hole.Radius, 5);
    }

    [Fact]
    public void ExtrudeTaperWithHoleFlipped()
    {
        var rectHole = Face.MakeRect(1, 1).MakeHoles([Wire.MakeCircle(0.25)]);
        var direction = new Vector(0, 0, 1);
        const double taper = 10;
        var taperSolidT = Solid.ExtrudeTaper(rectHole, direction, taper, true);
        var taperSolidF = Solid.ExtrudeTaper(rectHole, direction, taper, false);
        var holeT = taperSolidT.Edges().FilterBy(GeomType.Circle).SortBy(Axis.Z)[^1];
        var holeF = taperSolidF.Edges().FilterBy(GeomType.Circle).SortBy(Axis.Z)[^1];
        Assert.True(holeT.Radius > holeF.Radius);
    }

    [Fact]
    public void ExtrudeTaperHoleThroughWall()
    {
        // The hole widens as the walls close in and breaks through them part way up, which the draft builds as an
        // invalid solid
        var rectHole = Face.MakeRect(20, 12).MakeHoles([Wire.MakeCircle(3)]);
        var taperSolid = Solid.ExtrudeTaper(rectHole, (0, 0, 5), 20);
        Assert.True(taperSolid.IsValid);
        Check.AlmostEqual(687.89, taperSolid.Volume, 1);
    }

    [Fact]
    public void ExtrudeTaperOblique()
    {
        var rect = Face.MakeRect(2, 1);
        var rectHole = rect.MakeHoles([Wire.MakeCircle(0.25)]);
        var oRectHole = rectHole.Moved(new Location((0, 0, 0), (1, 0.1, 0), 77));
        var taper0 = Solid.ExtrudeTaper(rectHole, (0, 0, 1), 5);
        var taper1 = Solid.ExtrudeTaper(oRectHole, oRectHole.NormalAt(), 5);
        Check.AlmostEqual(taper0.Volume, taper1.Volume, 5);
    }

    [Fact]
    public void ExtrudeTaperFollowsDirection()
    {
        // Issues #553, #987: the winding of the profile must not set the direction
        Vector[] points = [(0, 0), (0, -20), (-10, -21), (-11, 0)];
        foreach (var outline in new[] { points, points.Reverse().ToArray() })
        {
            var profile = new Face(Wire.MakePolygon(outline));
            foreach (var direction in new Vector[] { (0, 0, 5), (0, 0, -5) })
            {
                var box = Solid.ExtrudeTaper(profile, direction, 1).BoundingBox();
                Check.AlmostEqual(5, box.Max.Z - box.Min.Z, 5);
                Check.AlmostEqual(direction.Z / 2, box.Center().Z, 5);
            }
        }
    }

    [Fact]
    public void ExtrudeTaperKeepsPlanarSides()
    {
        var tapered = Solid.ExtrudeTaper(Face.MakeRect(10, 10), (0, 0, 5), -10);
        Assert.All(tapered.Faces(), f => Assert.Equal(GeomType.Plane, f.GeomType));
    }

    [Fact]
    public void ExtrudeTaperStar()
    {
        // Issue #568: three thin bars crossing at the origin, widening taper
        var bar = Face.MakeRect(100, 1);
        var star = (bar + bar.Rotate(Axis.Z, 60) + bar.Rotate(Axis.Z, 120)).SingleFace();
        var tapered = Solid.ExtrudeTaper(star, (0, 0, -2), -15);
        Assert.True(tapered.IsValid);
        Assert.True(tapered.Volume > star.Area * 2);
    }

    [Fact]
    public void ExtrudeTaperCurvedProfile()
    {
        // The sides of an ellipse cannot be drafted, so it is lofted
        var ellipse = new Face(new Wire(Edge.MakeEllipse(8, 4)));
        var tapered = Solid.ExtrudeTaper(ellipse, (0, 0, 5), 10);
        Assert.True(tapered.IsValid);
        Assert.True(tapered.Volume < ellipse.Area * 5);
        Check.AlmostEqual(5, tapered.BoundingBox().Max.Z, 5);

        var holed = ellipse.MakeHoles([Wire.MakeCircle(1)]);
        var taperedHoled = Solid.ExtrudeTaper(holed, (0, 0, 5), 10);
        Assert.True(taperedHoled.IsValid);
        Assert.True(taperedHoled.Volume < tapered.Volume);
    }

    [Fact]
    public void ExtrudeTaperCollapsingProfile()
    {
        // Issue #567: a 15 degree taper over 2 closes up the 1 wide arms
        var cross = (Face.MakeRect(10, 1) + Face.MakeRect(1, 10)).SingleFace();
        Assert.True(Solid.ExtrudeTaper(cross, (0, 0, 2), 10).IsValid);
        var e = Assert.Throws<InvalidOperationException>(() => Solid.ExtrudeTaper(cross, (0, 0, 2), 15));
        Assert.Contains("may collapse the profile", e.Message);
    }

    [Fact]
    public void ExtrudeLinearWithRotation()
    {
        static void CheckTwist(Solid twist)
        {
            Check.AlmostEqual(1, twist.Volume, 5);
            var top = twist.Faces().SortBy(Axis.Z)[^1].Rotate(Axis.Z, 45);
            var bottom = twist.Faces().SortBy(Axis.Z)[0];
            var intersect = top.Translate((0, 0, -1)).Intersect(bottom);
            var area = intersect.SelectMany(s => s.Faces()).Sum(f => f.Area);
            Check.AlmostEqual(1, area, 5);
        }
        // Face
        CheckTwist(Solid.ExtrudeLinearWithRotation(Face.MakeRect(1, 1), center: (0, 0, 0), normal: (0, 0, 1), angle: 45));
        // Wire
        CheckTwist(Solid.ExtrudeLinearWithRotation(Wire.MakeRect(1, 1), center: (0, 0, 0), normal: (0, 0, 1), angle: 45));
    }

    [Fact]
    public void MakeLoft()
    {
        var loft = Solid.MakeLoft([Wire.MakeRect(2, 2), Wire.MakeCircle(1, new Plane((0, 0, 1)))]);
        Check.AlmostEqual((4 + Math.PI) / 2, loft.Volume, 1);

        Assert.Throws<ArgumentException>(() => Solid.MakeLoft([Wire.MakeRect(1, 1)]));
    }

    [Fact]
    public void MakeLoftWithVertices()
    {
        var loft = Solid.MakeLoft([new Vertex(0, 0, -1), Wire.MakeRect(1, 1.5), new Vertex(0, 0, 1)], true);
        Check.AlmostEqual(1, loft.Volume, 5);

        Assert.Throws<ArgumentException>(() => Solid.MakeLoft([Wire.MakeRect(1, 1), new Vertex(0, 0, 1), Wire.MakeRect(1, 1)]));
        Assert.Throws<ArgumentException>(() => Solid.MakeLoft([new Vertex(0, 0, 1), new Vertex(0, 0, 2)]));
        Assert.Throws<ArgumentException>(() =>
            Solid.MakeLoft([new Vertex(0, 0, 1), Wire.MakeRect(1, 1), new Vertex(0, 0, 2), new Vertex(0, 0, 3)]));

        // Two vertices, but not one at each end of the list
        var e = Assert.Throws<ArgumentException>(() =>
            Solid.MakeLoft([new Vertex(0, 0, 1), new Vertex(0, 0, 2), Wire.MakeRect(1, 1)]));
        Assert.Contains("beginning and end of the list", e.Message);
    }

    [Fact]
    public void ExtrudeUntil()
    {
        var square = Face.MakeRect(1, 1);
        var box = Solid.MakeBox(4, 4, 1, new Plane((-2, -2, 3)));
        var extrusion = Solid.ExtrudeUntil(square, box, (0, 0, 1), Until.Last);
        Check.AlmostEqual(4, extrusion.Volume, 5);

        box = Solid.MakeBox(4, 4, 1, new Plane((-2, -2, -3)));
        extrusion = Solid.ExtrudeUntil(square, box, (0, 0, 1), Until.Previous);
        Check.AlmostEqual(2, extrusion.Volume, 5);
    }

    [Fact]
    public void Sweep()
    {
        var path = Edge.MakeSpline([(0, 0), (3, 5), (7, -2)]);
        var section = Wire.MakeCircle(1, new Plane(path.PositionAt(0), zDir: path % 0));
        var area = new Face(section).Area;
        var swept = Solid.Sweep(section, path);
        Check.AlmostEqual(path.Length * area, swept.Volume, 0);
    }

    [Fact]
    public void HollowSweep()
    {
        var path = Edge.MakeLine((0, 0, 0), (0, 0, 5));
        var section = (Face.MakeRect(1, 1) - Face.MakeRect(0.1, 0.1)).Faces()[0];
        var swept = Solid.Sweep(section, path);
        Check.AlmostEqual(5 * (1 - 0.1 * 0.1), swept.Volume, 5);
    }

    // Spline(points, tangents=...) is Edge.make_spline with the tangents normalized and scale=True
    static Edge Spline(Vector start, Vector end, Vector tangent0, Vector tangent1) =>
        Edge.MakeSpline([start, end], [tangent0.Normalized(), tangent1.Normalized()], scale: true);

    [Fact]
    public void SweepMulti()
    {
        var f0 = Face.MakeRect(1, 1);
        var f1 = new Pos(x: 10) * new Face(Wire.MakeCircle(1));
        var path = Spline((0, 0), (10, 0), (0, 0, 1), (0, 0, -1));
        var binormal = Edge.MakeLine((0, 1), (10, 1));
        var swept = Solid.SweepMulti([f0, f1], path, isFrenet: true, binormal: binormal);
        Check.AlmostEqual(23.782, swept.Volume, 2);

        path = Spline((0, 0), (10, 0), (0, 0, 1), (1, 0, 0));
        swept = Solid.SweepMulti([f0, f1], path, isFrenet: true, binormal: new Vector(5, 0, 1));
        Check.AlmostEqual(20.75, swept.Volume, 2);
    }

    [Fact]
    public void SweepWarnsWhenInvalid()
    {
        var warnings = Warnings.Capture();
        // The kernel reports success for a sweep that folds over itself
        var path = Wire.MakePolygon([(0, 20), (20, 20), (20, 0)], close: false);
        var section = new Plane((0, 20, 0), zDir: (1, 0, 0)) * Face.MakeRect(10, 10);
        var folded = Solid.Sweep(section, path);
        Assert.Contains(warnings.Messages, w => w.Contains("sweep created an invalid solid"));
        Assert.False(folded.IsValid);

        var arc = Edge.MakeCircle(5, startAngle: 0, endAngle: 180);
        var profiles = new[] { 0.0, 1.0 }.Select(u => new Plane(arc.PositionAt(u), zDir: arc % u) * Face.MakeRect(30, 30));
        Solid.SweepMulti(profiles, arc);
        Assert.Contains(warnings.Messages, w => w.Contains("sweep_multi created an invalid"));
    }

    [Fact]
    public void Offset3D() =>
        Assert.Throws<ArgumentException>(() => Solid.MakeBox(1, 1, 1).Offset3D(null, 0.1, kind: Kind.Tangent));

    [Fact]
    public void Revolve()
    {
        var r = Solid.Revolve(Face.MakeRect(1, 1, new Plane((10, 0, 0))).SingleWire(), 180, axis: Axis.Y);
        Assert.Equal(6, r.Faces().Count);
    }

    [Fact]
    public void FromBoundingBox()
    {
        var cyl = Solid.MakeCylinder(0.001, 10).Located(new Location(Plane.Isometric));
        var cyl2 = Solid.MakeCylinder(1, 10).Located(new Location(Plane.Isometric));

        var rbb = Solid.FromBoundingBox(cyl.BoundingBox());
        var obb = Solid.FromBoundingBox(cyl.OrientedBoundingBox());
        var obb2 = Solid.FromBoundingBox(cyl2.OrientedBoundingBox());

        Check.AlmostEqual(Math.Pow(10, 3) * Math.Sqrt(3) / 9, rbb.Volume, 0);
        Assert.True(rbb.Volume > obb.Volume);
        Check.AlmostEqual(40, obb2.Volume, 4);
    }
}

/// <summary>An outward offset with openings comes back as a forward solid.</summary>
/// <remarks>
/// OCCT returns it with inward faces under a REVERSED flag, which volume and validity checks accept but the fillet
/// builder does not.
/// </remarks>
public class OffsetOrientationTests
{
    [Fact]
    public void OffsetCupCanBeFilleted()
    {
        var cone = Solid.MakeCone(22.5, 33.75, 40);
        var cup = (Shape3D)cone.Offset3D(cone.Faces().SortBy(Axis.Z)[^1..], 3, kind: Kind.Intersection);
        Assert.Equal(Orientation.Forward, cup.Wrapped.Orientation);
        Assert.True(cup.Volume > 0);
        var rim = cup.Edges().FilterBy(GeomType.Circle).SortBy(Axis.Z)[0];
        var rounded = cup.Fillet(1, [rim]);
        Assert.True(rounded.IsValid);
        Assert.InRange(rounded.Volume, cup.Volume - cup.Volume / 100, cup.Volume + cup.Volume / 100);
        Assert.True(rounded.Volume < cup.Volume);
    }

    [Theory]
    [InlineData(2, Kind.Arc)]
    [InlineData(2, Kind.Intersection)]
    [InlineData(-2, Kind.Arc)]
    [InlineData(-2, Kind.Intersection)]
    public void OffsetBoxBothWaysAndKinds(double amount, Kind kind)
    {
        var box = Solid.MakeBox(30, 20, 10);
        var top = box.Faces().SortBy(Axis.Z)[^1..];
        var shelled = (Shape3D)box.Offset3D(top, amount, kind: kind);
        Assert.Equal(Orientation.Forward, shelled.Wrapped.Orientation);
        Assert.True(shelled.Volume > 0);
        // The sharp edges around the opening fillet on every variant
        var rim = shelled.Edges().FilterBy(GeomType.Line).GroupBy(Axis.Z)[^1];
        var rounded = shelled.Fillet(0.5, rim);
        Assert.True(rounded.IsValid);
        Assert.True(rounded.Volume < shelled.Volume);
    }
}

public class SolidDraftTests
{
    readonly Solid _box;
    readonly ShapeList<Face> _sides;
    readonly Face _bottomFace;
    readonly Plane _neutralPlane;

    public SolidDraftTests()
    {
        // A simple box to test draft
        _box = Solid.MakeBox(10, 10, 10, new Plane((-5, -5, -5)));
        _sides = _box.Faces().FilterBy(Axis.Z, reverse: true);
        _bottomFace = _box.Faces().SortBy(Axis.Z)[0];
        _neutralPlane = new Plane(_bottomFace);
    }

    [Fact]
    public void SuccessfulDraft()
    {
        // A draft operation completes successfully on planar faces
        var drafted = _box.Draft(_sides, _neutralPlane, 5);
        Assert.IsType<Solid>(drafted);
        Assert.NotEqual(_box.Volume, drafted.Volume);
    }

    [Fact]
    public void UnsupportedGeometry()
    {
        // Unsupported face geometry is rejected: a toroidal face
        var torus = Solid.MakeTorus(5, 1);
        var e = Assert.Throws<ArgumentException>(() => torus.Draft([torus.Faces()[0]], _neutralPlane, 5));
        Assert.Contains("unsupported geometry type", e.Message);
    }

    [Fact]
    public void DraftAngleErrorContents()
    {
        // DraftAngleError stores the face and the problematic shape
        var face = _sides[0];
        var shape = _box.Wrapped;
        var err = new DraftAngleError("msg", face: face, problematicShape: shape);
        Assert.Equal("msg", err.Message);
        Assert.Same(face, err.Face);
        Assert.True(err.ProblematicShape is { } problematic && problematic.IsEqual(shape));
    }
}
