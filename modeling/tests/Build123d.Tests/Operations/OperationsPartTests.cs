// Ported from build123d tests/test_build_part.py, the operation tests rewritten in algebra form. Omitted:
// - TestAlign, TestBasePartObjectFirewall, TestBuildPart, TestBuildPartExceptions, TestCounterBoreHole,
//   TestCounterSinkHole, TestCylinder, TestHole, TestTorus, TestWedge, TestConvexPolyhedron: builder mode or objects,
//   not operations.
// - TestMakeBrakeFormed: make_brake_formed isn't ported (deprecated in favour of BuildSheet).
// - TestPartOperationDraft.test_draftangleerror_from_solid_draft, TestExtrude.test_extrude_until_invalid_sewn_shape,
//   test_extrude_until_invalid_split, TestLoft.test_loft_invalid_recovery, test_loft_recovery_remains_invalid,
//   TestProjectWorkplane.test_x_dir_perpendicular_to_projection_dir: mock OCP or build123d internals.
// - TestProjectWorkplane.test_invalid_context, TestRevolve's and TestSection's builder-default cases
//   (test_no_profiles in a builder, test_default_plane_from_context), TestSection.test_no_section_planes (an explicit
//   None): builder mode.
// - TestLoft.test_loft_with_two_holes, test_loft_with_inconsistent_holes: need Text (phase 7); the same checks are made
//   with faces holding two circular holes.
// The BuildLine/BuildSketch profiles of TestRevolve are made with the curve objects and MakeFace.
using static Build123d.Operations;

namespace Build123d.Tests.Operations;

public class OperationsPartTests
{
    // ---- TestPartOperationDraft ----

    private readonly Solid _box = new Box(10, 10, 10).SingleSolid();

    private ShapeList<Face> Sides => _box.Faces().FilterBy(Axis.Z, reverse: true);

    private Plane NeutralPlane => new(_box.Faces().SortBy(Axis.Z)[0]);

    [Fact]
    public void SuccessfulDraft()
    {
        var result = Draft(Sides, NeutralPlane, 5);
        Assert.IsType<Part>(result);
        Assert.True(_box.Volume < result.Volume);
        Assert.NotNull(result.History);

        var box = new Box(10, 10, 10);
        var drafted = Draft(box.Faces().FilterBy(Axis.Z, reverse: true), Plane.XY.Offset(-5), 5);
        Assert.True(drafted.Volume < 1000);
    }

    [Fact]
    public void DraftInvalidFaceType()
    {
        var torus = new Torus(5, 1).SingleSolid();
        var e = Assert.Throws<ArgumentException>(() => Draft([torus.Faces()[0]], NeutralPlane, 5));
        Assert.Equal("Draft not supported on face(s) with geometry: TORUS", e.Message);
    }

    [Fact]
    public void DraftFacesFromMultipleSolids()
    {
        var box2 = new Box(5, 5, 5).SingleSolid();
        var e = Assert.Throws<ArgumentException>(() => Draft([Sides[0], box2.Faces()[0]], NeutralPlane, 5));
        Assert.Contains("same topological parent", e.Message);
    }

    [Fact]
    public void DraftFacesFromMultipleParts()
    {
        var box2 = new Box(5, 5, 5).SingleSolid();
        var part = new Part().Add(_box, new Pos(10) * box2);
        Face[] mixed = [part.Faces().SortBy(Axis.X)[0], part.Faces().SortBy(Axis.X)[^1]];
        Assert.Throws<ArgumentException>(() => Draft(mixed, NeutralPlane, 5));
    }

    [Fact]
    public void BadDraftFaces()
    {
        Assert.Throws<DraftAngleError>(() => Draft(_box.Faces().SortBy(Axis.Z)[0], NeutralPlane, 10));
    }

    // ---- TestExtrude ----

    [Fact]
    public void ExtrudeWithFaceInput()
    {
        var test = Extrude(new Rectangle(5, 5).Faces(), amount: 2.5, both: true);
        Check.AlmostEqual(125, test.Volume, 5);
    }

    [Fact]
    public void ExtrudeBoth()
    {
        Check.AlmostEqual(125, Extrude(new Rectangle(5, 5), amount: 2.5, both: true).Volume, 5);
    }

    [Fact]
    public void ExtrudeTapered()
    {
        var test = Extrude(new Rectangle(2, 2), amount: 1, taper: 10);
        Assert.True(test.Volume < 4);
        Assert.True(test.Volume > 0);
    }

    [Fact]
    public void ExtrudeUntil()
    {
        Shape box = new Box(10, 10, 10, align: (Align.Center, Align.Center, Align.Min));
        var target = box - Scale(box, by: (0.8, 0.8, 0.8));
        var rib = Extrude(new Rectangle(1, 1), until: Until.Next, target: target);
        Check.AlmostEqual(10 * 10 * 10 - 8 * 8 * 8 + 1 * 1 * 8, (target + rib).Volume, 5);
    }

    [Fact]
    public void ExtrudeUntil2()
    {
        var target = new Box(10, 5, 5) - new Pos(2.5) * new Cylinder(0.5, 5);
        var pln = new Plane((7, 0, 7), zDir: (-1, 0, -1));
        var profile = (pln * new Circle(1)).SingleFace();
        var extrusion = Extrude(profile, dir: pln.ZDir, until: Until.Next, target: target);
        Assert.True(extrusion.BoundingBox().Min.Z < 2.5);
    }

    [Fact]
    public void ExtrudeUntil3()
    {
        var sketch = Plane.XZ * (new Rectangle(8, 8, align: Align.Min) - new Pos(1, 1) * new Rectangle(7, 7, align: Align.Min));
        var p = Extrude(sketch, amount: 2, both: true);
        var profile = new Plane((-2, 0, -2), (0, 1, 0), (1, 0, 1));
        var rib = Extrude(profile * new Rectangle(4, 1), until: Until.Next, target: p);
        Check.AlmostEqual(72.313, (p + rib).Volume, 2);
    }

    [Fact]
    public void ExtrudeUntilFromAFaceOfTheTarget()
    {
        // A sketch drawn on a face of the target extrudes into it: LAST reaches the far side of the material, a cavity's
        // underside here. NEXT reaches the face the sketch lies in, which cuts nothing
        var section = new Rectangle(20, 10, align: (Align.Center, Align.Min))
                      - new Pos(0, 1) * new Circle(6, align: (Align.Center, Align.Min));
        var body = Extrude(Plane.YZ.Offset(-10) * section, amount: 20);
        var cross = new RectangleRounded(10, 3, 1, rotation: 0) + new RectangleRounded(10, 3, 1, rotation: 90);
        var rib = Extrude(cross, until: Until.Last, target: body);
        Assert.True(rib.IsValid);
        Check.AlmostEqual(0, rib.BoundingBox().Min.Z, 6);
        Check.AlmostEqual(7 - Math.Sqrt(36 - 25), rib.BoundingBox().Max.Z, 4);
        Check.AlmostEqual(72.302, rib.Volume, 2);
        var e = Assert.Throws<ArgumentException>(() => Extrude(cross, until: Until.Next, target: body));
        Assert.Contains("does not cut the extrusion", e.Message);
    }

    [Fact]
    public void ExtrudeUntilErrors()
    {
        Assert.Throws<ArgumentException>(() =>
            Extrude(new Rectangle(1, 1), until: Until.Next, dir: (0, 0, 1), target: new Pos(0, 0, -10) * new Box(1, 1, 1)));
    }

    [Fact]
    public void ExtrudeFace()
    {
        var square = Plane.XZ * new Rectangle(10, 10, align: (Align.Center, Align.Min));
        Check.AlmostEqual(10 * 10 * 10, Extrude(square, amount: 10).Volume, 5);
    }

    [Fact]
    public void ExtrudeNonPlanarFace()
    {
        var cyl = new Cylinder(1, 2);
        var npf = cyl.Split(Plane.XZ).Faces().FilterBy(GeomType.Plane, reverse: true)[0];
        var testSolid = Extrude(npf, amount: 3, dir: (0, 1, 0));
        Check.AlmostEqual(2 * 2 * 3, testSolid.Volume, 5);
    }

    [Fact]
    public void NonPlanarFaceWithoutDir()
    {
        var side = new Cylinder(1, 2).Faces().FilterBy(GeomType.Cylinder)[0];
        var e = Assert.Throws<ArgumentException>(() => Extrude(side, amount: 0.5));
        Assert.Contains("dir must be provided", e.Message);
    }

    [Fact]
    public void NoAmountOrUntil()
    {
        var e = Assert.Throws<ArgumentException>(() => Extrude(new Rectangle(1, 1).SingleFace()));
        Assert.Contains("Either amount or until", e.Message);
    }

    [Fact]
    public void UntilWithoutTarget()
    {
        var e = Assert.Throws<ArgumentException>(() => Extrude(new Rectangle(1, 1).SingleFace(), until: Until.Next));
        Assert.Contains("A target object must be provided", e.Message);
    }

    [Fact]
    public void UntilWithEmptyTarget()
    {
        var e = Assert.Throws<ArgumentException>(() => Extrude(new Rectangle(1, 1), until: Until.Next, target: new Part()));
        Assert.Contains("No target object provided", e.Message);
    }

    // ---- TestLoft ----

    [Fact]
    public void LoftReturnIsCleaned()
    {
        // Issue #451: a section edge drawn as two collinear lines lofts into two coplanar faces that clean merges
        Shape[] sections =
        [
            new Face(Wire.MakePolygon([(0, 0), (1, 0), (2, 0), (2, 1), (0, 1)])),
            new Pos(0, 0, 1) * Face.MakeRect(2, 1, new Plane((1, 0.5, 0))),
        ];
        Assert.Equal(6, Loft(sections).Faces().Count);
        Assert.Equal(7, Loft(sections, clean: false).Faces().Count);
    }

    [Fact]
    public void SimpleLoft()
    {
        const int sliceCount = 10;
        var sections = Enumerable.Range(0, sliceCount + 1).Select(i =>
            (Shape)new Face(new Wire([Edge.MakeCircle(10 * Math.Sin(i * Math.PI / sliceCount) + 5).Moved(new Location((0, 0, i * 3)))])));
        var test = Loft(sections);
        Assert.True(test.Volume < 225 * Math.PI * 30);
        Assert.True(test.Volume > 25 * Math.PI * 30);
        Assert.NotNull(test.History);
    }

    [Fact]
    public void LoftVertex()
    {
        Check.AlmostEqual(1, Loft([new Rectangle(1, 1), new Vertex(0, 0, 3)], ruled: true).Volume, 5);
        Check.AlmostEqual(1, Loft([new Rectangle(1, 1).SingleFace(), new Vertex(0, 0, 3)], ruled: true).Volume, 5);
    }

    [Fact]
    public void LoftVertices()
    {
        var test = Loft([new Vertex(0, 0, -3), new Rectangle(1, 1), new Vertex(0, 0, 3)], ruled: true);
        Check.AlmostEqual(2, test.Volume, 5);
    }

    [Fact]
    public void LoftInvalidVertex()
    {
        var lowerSection = Face.MakeRect(10, 10) - Face.MakeRect(8, 8);
        var upperSection = new Pos(0, 0, 5) * lowerSection;
        Assert.Throws<ArgumentException>(() => Loft([lowerSection, new Vertex(0, 0, 2.5), upperSection]));
    }

    [Fact]
    public void LoftNoSections()
    {
        Assert.Throws<ArgumentException>(() => Loft([null!]));
        Assert.Throws<ArgumentException>(() => Loft([]));
    }

    [Fact]
    public void LoftNothing()
    {
        Assert.True(Loft([new Face()]).IsEmpty);
    }

    [Fact]
    public void LoftVertexMiddle()
    {
        Assert.Throws<ArgumentException>(() => Loft([new Vertex(0, 0, -1), new Vertex(0, 0, 2), new Circle(1)]));
    }

    [Fact]
    public void LoftWithHole()
    {
        var lowerSection = Face.MakeRect(10, 10) - Face.MakeRect(8, 8);
        var upperSection = new Pos(0, 0, 5) * lowerSection;
        Check.AlmostEqual(10 * 10 * 5 - 8 * 8 * 5, Loft([lowerSection, upperSection]).Volume, 5);
    }

    [Fact]
    public void LoftWithTwoHoles()
    {
        var lowerSection = new Rectangle(10, 10) - new Pos(-2.5) * new Circle(1) - new Pos(2.5) * new Circle(1.5);
        // The holes swap places in the order of the upper section's inner wires, which matching undoes
        var upperSection = new Pos(0, 0, 5) * (new Rectangle(10, 10) - new Pos(2.5) * new Circle(1.5) - new Pos(-2.5) * new Circle(1));
        var test = Loft([lowerSection, upperSection]);
        Assert.True(test.IsValid);
        Check.AlmostEqual(5 * (100 - Math.PI * (1 + 1.5 * 1.5)), test.Volume, 3);
    }

    [Fact]
    public void LoftWithInconsistentHoles()
    {
        var lowerSection = new Rectangle(10, 10) - new Pos(-2.5) * new Circle(1) - new Pos(2.5) * new Circle(1.5);
        var upperSection = new Pos(0, 0, 5) * Face.MakeRect(10, 10);
        Assert.Throws<ArgumentException>(() => Loft([lowerSection, upperSection]));
    }

    // ---- TestProjectWorkplane ----

    [Fact]
    public void ProjectWorkplaneBasic()
    {
        var workplane = ProjectWorkplane((0, 0, 0), (1, 0, 0), (0, 0, 1), 10);
        Check.AlmostEqual(new Vector(0, 0, 10), workplane.Origin, 5);
        Check.AlmostEqual(new Vector(1, 0, 0), workplane.XDir, 5);
        Check.AlmostEqual(new Vector(0, 0, 1), workplane.ZDir, 5);
    }

    // ---- TestRevolve ----

    [Fact]
    public void RevolveReturnIsCleaned()
    {
        // Issue #451: a profile edge drawn as two collinear lines revolves into two faces that clean merges
        var profile = new Pos(5, 0, 0) * new Face(Wire.MakePolygon([(0, 0), (1, 0), (2, 0), (2, 1), (0, 1)]));
        Assert.Equal(4, Revolve(profile, Axis.Y).Faces().Count);
        Assert.Equal(5, Revolve(profile, Axis.Y, clean: false).Faces().Count);
    }

    [Fact]
    public void RevolveSize()
    {
        // The revolution matches revolution_arc's size and direction
        var ax = Axis.X;
        var profile = new RegularPolygon(10, 4, align: (Align.Center, Align.Min));
        var fullVolume = Revolve(profile, ax, 360).Volume;
        int[] positive = [30, 90, 150, 180, 200, 360, 500, 720, 750];
        var sizes = positive.Reverse().Select(x => -x).Append(0).Concat(positive);
        foreach (var size in sizes)
        {
            var solid = Revolve(profile, axis: ax, revolutionArc: size);

            // A rotation edge, and the start tangent normal to the profile
            var edge = Edge.MakeCircle(1, Plane.YZ, 0, PyMod(size, 360),
                                       size > 0 ? AngularDirection.CounterClockwise : AngularDirection.Clockwise);
            var sign = edge.TangentAt(0).Z;

            var expected = PyMod(size, sign * 360);
            expected = expected == 0 ? sign * 360 : expected;
            var result = edge.Length / edge.Radius / Math.PI * 180 * sign;

            Check.AlmostEqual(fullVolume * Math.Abs(expected) / 360, solid.Volume);
            Check.AlmostEqual(expected, result);
        }
    }

    private static double PyMod(double a, double b)
    {
        var r = a % b;
        return r != 0 && (r < 0) != (b < 0) ? r + b : r;
    }

    [Fact]
    public void SimpleRevolve()
    {
        var l1 = new Line((0, 0), (12, 0));
        var l2 = new RadiusArc(l1.PositionAt(1), (15, 20), 50);
        var l3 = new Spline([l2.PositionAt(1), (22, 40), (20, 50)], tangents: [l2 % 1, (-0.75, 1)]);
        var l4 = new RadiusArc(l3.PositionAt(1), l3.PositionAt(1) + new Vector(0, 5), 5);
        var l5 = new Spline([l4.PositionAt(1), l4.PositionAt(1) + new Vector(2.5, 2.5), l4.PositionAt(1) + new Vector(0, 5)],
                            tangents: [l4 % 1, (-1, 0)]);
        var l6 = new Polyline(l5.PositionAt(1), l5.PositionAt(1) + new Vector(0, 1), (0, l5.PositionAt(1).Y + 1), l1.PositionAt(0));
        var test = Revolve(MakeFace([l1, l2, l3, l4, l5, l6]), axis: Axis.Y);
        Assert.True(test.Volume < 22 * 22 * Math.PI * 50);
        Assert.True(test.Volume > 144 * Math.PI * 50);
    }

    [Fact]
    public void RevolveWithAxis()
    {
        var l1 = new Line((0, 0), (0, 12));
        var l2 = new RadiusArc(l1.PositionAt(1), (20, 10), 50);
        var l3 = new Line(l2.PositionAt(1), (20, 0));
        var l4 = new Line(l3.PositionAt(1), l1.PositionAt(0));
        var test = Revolve(MakeFace([l1, l2, l3, l4]), axis: Axis.X);
        Assert.True(test.Volume < 244 * Math.PI * 20);
        Assert.True(test.Volume > 100 * Math.PI * 20);
    }

    [Fact]
    public void RevolveNoProfiles()
    {
        var e = Assert.Throws<ArgumentException>(() => Revolve(Array.Empty<Shape>()));
        Assert.Contains("No profiles provided", e.Message);
    }

    // ---- TestSection ----

    [Fact]
    public void SectionCircle()
    {
        Check.AlmostEqual(100 * Math.PI, Section(new Sphere(10)).Area, 5);
        Check.AlmostEqual(100 * Math.PI, Section(new Sphere(10), Plane.XZ).Area, 5);
    }

    [Fact]
    public void SectionHeight()
    {
        // A sphere's section 6 above its centre is a circle of radius 8
        Check.AlmostEqual(64 * Math.PI, Section(new Sphere(10), Plane.XY, height: 6).Area, 5);
    }

    [Fact]
    public void SectionManyPlanes()
    {
        var sec = Section(new Sphere(10), [Plane.XY, Plane.YZ]);
        Assert.Equal(2, sec.Faces().Count);
    }

    [Fact]
    public void SectionMovedObject()
    {
        var sec = Section(new Pos(-100, 100) * new Sphere(10), Plane.XY);
        Assert.Single(sec.Faces());
        Check.AlmostEqual(10, sec.SingleFace().SingleEdge().Radius, 5);
        Check.AlmostEqual(new Vector(-100, 100, 0), sec.SingleFace().Center(), 5);
    }

    [Fact]
    public void SectionNoObject()
    {
        var e = Assert.Throws<ArgumentException>(() => Section(null!));
        Assert.Contains("No object to section", e.Message);
    }

    // ---- TestSplit ----

    [Fact]
    public void SplitSphere()
    {
        Check.AlmostEqual(2.0 / 3 * 1000 * Math.PI, Split(new Sphere(10), keep: Keep.Top).Volume, 5);
        Assert.Equal(2, Split(new Sphere(10), keep: Keep.Both).Solids().Count);
        Check.AlmostEqual(2.0 / 3 * 1000 * Math.PI, Split(new Sphere(10), bisectBy: Plane.YZ, keep: Keep.Top).Volume, 5);
    }

    [Fact]
    public void SplitWrappedObject()
    {
        var obj = new Box(1, 1, 1);
        var filleted = Fillet(obj.Edges().GroupBy(Axis.Z).Last, 0.1);
        var right = Split(filleted, bisectBy: Plane.YZ, keep: Keep.Top);
        Assert.True(right.Volume < filleted.Volume);
    }

    [Fact]
    public void SplitNoObjects()
    {
        var e = Assert.Throws<ArgumentException>(() => Split((IEnumerable<Shape>)null!));
        Assert.Contains("objects must be provided", e.Message);
    }

    // ---- TestThicken ----

    [Fact]
    public void ThickenFaces()
    {
        var part = Thicken(new RectangleRounded(10, 10, 1), amount: 1);
        Check.AlmostEqual(1, part.BoundingBox().Max.Z, 5);

        var nonPlanar = new Sphere(1).Faces()[0];
        var outerSphere = Thicken(nonPlanar, amount: 0.1);
        Check.AlmostEqual(4.0 / 3 * Math.PI * (Math.Pow(1.1, 3) - 1), outerSphere.Volume, 5);
    }

    [Fact]
    public void ThickenSweptSurface()
    {
        var wire = (Shape1D)(new JernArc((0, -2), (-1, 0), 1, -180) + new JernArc((0, 0), (1, 0), 2, -90));

        var surface = Sweep((wire ^ 0) * new RadiusArc((0, 0), (0, -1), 1), wire);
        var part = Thicken(surface, 0.4);
        Check.AlmostEqual(2.241583787221904, part.Volume, 5);

        part = Thicken(Sweep((wire ^ 0) * new RadiusArc((0, 0), (0, -1), 1), wire), 0.4, both: true);
        Check.AlmostEqual(4.711747154435256, part.Volume, 5);
    }

    [Fact]
    public void ThickenBoth()
    {
        var part = Thicken(Face.MakeRect(2, 2), amount: 1, both: true);
        Check.AlmostEqual(8, part.Volume, 5);
        Check.AlmostEqual(2, part.BoundingBox().Size.Z, 5);
    }

    [Fact]
    public void ThickenNoAmount()
    {
        var e = Assert.Throws<ArgumentException>(() => Thicken(new Rectangle(1, 1).SingleFace()));
        Assert.Contains("An amount must be provided", e.Message);
    }

    [Fact]
    public void ThickenNoFace()
    {
        var e = Assert.Throws<ArgumentException>(() => Thicken((Shape)null!, amount: 1));
        Assert.Contains("A face or sketch must be provided", e.Message);
    }
}
