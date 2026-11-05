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
using static Kerf.Modeling.Operations;

namespace Build123d.Tests.Operations;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class OperationsPartTests
{
    // ---- TestPartOperationDraft ----

    private readonly Solid _box = new Box(10, 10, 10).SingleSolid();

    private ShapeList<Face> Sides => _box.SelectFaces().NotParallelTo(Axis.Z);

    private Plane NeutralPlane => new(_box.SelectFaces().OrderBy(Axis.Z)[0]);

    [Fact]
    public void SuccessfulDraft()
    {
        var result = Draft(Sides, NeutralPlane, 5).Value;
        Assert.IsType<Part>(result);
        Assert.True(_box.Volume < result.Volume);
        Assert.NotNull(result.History);

        var box = new Box(10, 10, 10);
        var drafted = Draft(box.Faces().NotParallelTo(Axis.Z), Plane.XY.Offset(-5), 5).Value;
        Assert.True(drafted.Volume < 1000);
    }

    [Fact]
    public void DraftInvalidFaceType()
    {
        var torus = new Torus(5, 1).SingleSolid();
        var e = Assert.Throws<ArgumentException>(() => Draft([torus.SelectFaces()[0]], NeutralPlane, 5));
        Assert.Equal("Draft not supported on face(s) with geometry: TORUS", e.Message);
    }

    [Fact]
    public void DraftFacesFromMultipleSolids()
    {
        var box2 = new Box(5, 5, 5).SingleSolid();
        var e = Assert.Throws<ArgumentException>(() => Draft([Sides[0], box2.SelectFaces()[0]], NeutralPlane, 5));
        Assert.Contains("same topological parent", e.Message);
    }

    [Fact]
    public void DraftFacesFromMultipleParts()
    {
        var box2 = new Box(5, 5, 5).SingleSolid();
        var part = new Part().Add(_box, new Pos(10) * box2).Value;
        Face[] mixed = [part.Faces().OrderBy(Axis.X)[0], part.Faces().OrderBy(Axis.X)[^1]];
        Assert.Throws<ArgumentException>(() => Draft(mixed, NeutralPlane, 5));
    }

    [Fact]
    public void BadDraftFaces()
    {
        var drafted = Draft(_box.SelectFaces().OrderBy(Axis.Z)[0], NeutralPlane, 10);
        Assert.Equal(ModelingDiagnostics.DraftFailed, drafted.Error!.Code);
        // The same failure from the faces' form
        Assert.Equal(ModelingDiagnostics.DraftFailed, Draft([_box.SelectFaces().OrderBy(Axis.Z)[0]], NeutralPlane, 10).Error!.Code);
    }

    // ---- TestExtrude ----

    [Fact]
    public void ExtrudeWithFaceInput()
    {
        var test = Extrude(new Rectangle(5, 5).Faces(), 2.5, new ExtrudeOptions { Both = true }).Value;
        Check.AlmostEqual(125, test.Volume, 5);
    }

    [Fact]
    public void ExtrudeBoth()
    {
        Check.AlmostEqual(125, Extrude(new Rectangle(5, 5), 2.5, new ExtrudeOptions { Both = true }).Value.Volume, 5);
    }

    [Fact]
    public void ExtrudeTapered()
    {
        var test = Extrude(new Rectangle(2, 2), 1, new ExtrudeOptions { Taper = 10 }).Value;
        Assert.True(test.Volume < 4);
        Assert.True(test.Volume > 0);
    }

    [Fact]
    public void ExtrudeUntil()
    {
        Shape box = new Box(10, 10, 10, align: new Alignment(Align.Center, Align.Center, Align.Min));
        var target = box - Scale(box, by: new Vector(0.8, 0.8, 0.8));
        var rib = Extrude(new Rectangle(1, 1), Until.Next, target).Value;
        Check.AlmostEqual(10 * 10 * 10 - 8 * 8 * 8 + 1 * 1 * 8, (target + rib).Volume, 5);
    }

    [Fact]
    public void ExtrudeUntil2()
    {
        var target = new Box(10, 5, 5) - new Pos(2.5) * new Cylinder(0.5, 5);
        var pln = new Plane(new Vector(7, 0, 7), zDir: new Vector(-1, 0, -1));
        var profile = (pln * new Circle(1)).SingleFace();
        var extrusion = Extrude(profile, Until.Next, target, new ExtrudeOptions { Direction = pln.ZDir }).Value;
        Assert.True(extrusion.BoundingBox().Min.Z < 2.5);
    }

    [Fact]
    public void ExtrudeUntil3()
    {
        var sketch = Plane.XZ * (new Rectangle(8, 8, align: Align.Min) - new Pos(1, 1) * new Rectangle(7, 7, align: Align.Min));
        var p = Extrude(sketch, 2, new ExtrudeOptions { Both = true }).Value;
        var profile = new Plane(new Vector(-2, 0, -2), new Vector(0, 1, 0), new Vector(1, 0, 1));
        var rib = Extrude(profile * new Rectangle(4, 1), Until.Next, p).Value;
        Check.AlmostEqual(72.313, (p + rib).Volume, 2);
    }

    [Fact]
    public void ExtrudeUntilFromAFaceOfTheTarget()
    {
        // A sketch drawn on a face of the target extrudes into it: LAST reaches the far side of the material, a cavity's
        // underside here. NEXT reaches the face the sketch lies in, which cuts nothing
        var section = new Rectangle(20, 10, align: new Alignment(Align.Center, Align.Min))
                      - new Pos(0, 1) * new Circle(6, align: new Alignment(Align.Center, Align.Min));
        var body = Extrude(Plane.YZ.Offset(-10) * section, 20).Value;
        var cross = new RectangleRounded(10, 3, 1, rotation: 0) + new RectangleRounded(10, 3, 1, rotation: 90);
        var rib = Extrude(cross, Until.Last, body).Value;
        Assert.True(rib.IsValid);
        Check.AlmostEqual(0, rib.BoundingBox().Min.Z, 6);
        Check.AlmostEqual(7 - Math.Sqrt(36 - 25), rib.BoundingBox().Max.Z, 4);
        Check.AlmostEqual(72.302, rib.Volume, 2);
        var e = Assert.Throws<ArgumentException>(() => Extrude(cross, Until.Next, body).Value);
        Assert.Contains("does not cut the extrusion", e.Message);
    }

    [Fact]
    public void ExtrudeUntilErrors()
    {
        Assert.Throws<ArgumentException>(() =>
            Extrude(new Rectangle(1, 1), Until.Next, new Pos(0, 0, -10) * new Box(1, 1, 1), new ExtrudeOptions { Direction = new Vector(0, 0, 1) }));
    }

    [Fact]
    public void ExtrudeFace()
    {
        var square = Plane.XZ * new Rectangle(10, 10, align: new Alignment(Align.Center, Align.Min));
        Check.AlmostEqual(10 * 10 * 10, Extrude(square, 10).Value.Volume, 5);
    }

    [Fact]
    public void ExtrudeNonPlanarFace()
    {
        var cyl = new Cylinder(1, 2);
        var npf = cyl.Split(Plane.XZ).Faces().WhereNot(GeomType.Plane)[0];
        var testSolid = Extrude(npf, 3, new ExtrudeOptions { Direction = new Vector(0, 1, 0) }).Value;
        Check.AlmostEqual(2 * 2 * 3, testSolid.Volume, 5);
    }

    [Fact]
    public void NonPlanarFaceWithoutDir()
    {
        var side = new Cylinder(1, 2).Faces().Where(GeomType.Cylinder)[0];
        var e = Assert.Throws<ArgumentException>(() => Extrude(side, 0.5));
        Assert.Contains("A direction is needed", e.Message);
    }

    [Fact]
    public void NoAmountOrUntil()
    {
        // An extrusion takes an amount or an Until and a target by its overload (unit 10M2); nothing to extrude is refused
        var e = Assert.Throws<ArgumentNullException>(() => Extrude((Shape)null!, 1));
        Assert.Equal("toExtrude", e.ParamName);
    }

    [Fact]
    public void UntilWithoutTarget()
    {
        var e = Assert.Throws<ArgumentNullException>(() => Extrude(new Rectangle(1, 1).SingleFace(), Until.Next, null!));
        Assert.Equal("target", e.ParamName);
    }

    [Fact]
    public void UntilWithEmptyTarget()
    {
        var e = Assert.Throws<ArgumentException>(() => Extrude(new Rectangle(1, 1), Until.Next, new Part()));
        Assert.Contains("The target to extrude up to is empty", e.Message);
    }

    // ---- TestLoft ----

    [Fact]
    public void LoftReturnIsCleaned()
    {
        // Issue #451: a section edge drawn as two collinear lines lofts into two coplanar faces that clean merges
        Shape[] sections =
        [
            Face.Make(Wire.MakePolygon([new Vector(0, 0), new Vector(1, 0), new Vector(2, 0), new Vector(2, 1), new Vector(0, 1)])),
            new Pos(0, 0, 1) * Face.MakeRect(2, 1, new Plane(new Vector(1, 0.5, 0))),
        ];
        Assert.Equal(6, Loft(sections).Value.Faces().Count);
        Assert.Equal(7, Loft(sections, clean: false).Value.Faces().Count);
    }

    [Fact]
    public void SimpleLoft()
    {
        const int sliceCount = 10;
        var sections = Enumerable.Range(0, sliceCount + 1).Select(i =>
            (Shape)Face.Make(Wire.Make([Edge.MakeCircle(10 * Math.Sin(i * Math.PI / sliceCount) + 5).Moved(new Location(new Vector(0, 0, i * 3)))])));
        var test = Loft(sections).Value;
        Assert.True(test.Volume < 225 * Math.PI * 30);
        Assert.True(test.Volume > 25 * Math.PI * 30);
        Assert.NotNull(test.History);
    }

    [Fact]
    public void LoftVertex()
    {
        Check.AlmostEqual(1, Loft([new Rectangle(1, 1), Vertex.Make(0, 0, 3)], ruled: true).Value.Volume, 5);
        Check.AlmostEqual(1, Loft([new Rectangle(1, 1).SingleFace(), Vertex.Make(0, 0, 3)], ruled: true).Value.Volume, 5);
    }

    [Fact]
    public void LoftVertices()
    {
        var test = Loft([Vertex.Make(0, 0, -3), new Rectangle(1, 1), Vertex.Make(0, 0, 3)], ruled: true).Value;
        Check.AlmostEqual(2, test.Volume, 5);
    }

    [Fact]
    public void LoftInvalidVertex()
    {
        var lowerSection = Face.MakeRect(10, 10) - Face.MakeRect(8, 8);
        var upperSection = new Pos(0, 0, 5) * lowerSection;
        Assert.Throws<ArgumentException>(() => Loft([lowerSection, Vertex.Make(0, 0, 2.5), upperSection]));
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
        Assert.True(Loft([default(Face)]).Value.IsEmpty);
    }

    [Fact]
    public void LoftVertexMiddle()
    {
        Assert.Throws<ArgumentException>(() => Loft([Vertex.Make(0, 0, -1), Vertex.Make(0, 0, 2), new Circle(1)]));
    }

    [Fact]
    public void LoftWithHole()
    {
        var lowerSection = Face.MakeRect(10, 10) - Face.MakeRect(8, 8);
        var upperSection = new Pos(0, 0, 5) * lowerSection;
        Check.AlmostEqual(10 * 10 * 5 - 8 * 8 * 5, Loft([lowerSection, upperSection]).Value.Volume, 5);
    }

    [Fact]
    public void LoftWithTwoHoles()
    {
        var lowerSection = new Rectangle(10, 10) - new Pos(-2.5) * new Circle(1) - new Pos(2.5) * new Circle(1.5);
        // The holes swap places in the order of the upper section's inner wires, which matching undoes
        var upperSection = new Pos(0, 0, 5) * (new Rectangle(10, 10) - new Pos(2.5) * new Circle(1.5) - new Pos(-2.5) * new Circle(1));
        var test = Loft([lowerSection, upperSection]).Value;
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
        var workplane = ProjectWorkplane(new Vector(0, 0, 0), new Vector(1, 0, 0), new Vector(0, 0, 1), 10);
        Check.AlmostEqual(new Vector(0, 0, 10), workplane.Origin, 5);
        Check.AlmostEqual(new Vector(1, 0, 0), workplane.XDir, 5);
        Check.AlmostEqual(new Vector(0, 0, 1), workplane.ZDir, 5);
    }

    // ---- TestRevolve ----

    [Fact]
    public void RevolveReturnIsCleaned()
    {
        // Issue #451: a profile edge drawn as two collinear lines revolves into two faces that clean merges
        var profile = new Pos(5, 0, 0) * Face.Make(Wire.MakePolygon([new Vector(0, 0), new Vector(1, 0), new Vector(2, 0), new Vector(2, 1), new Vector(0, 1)]));
        Assert.Equal(4, Revolve(profile, Axis.Y).Value.Faces().Count);
        Assert.Equal(5, Revolve(profile, Axis.Y, clean: false).Value.Faces().Count);
    }

    [Fact]
    public void RevolveSize()
    {
        // The revolution matches revolution_arc's size and direction
        var ax = Axis.X;
        var profile = new RegularPolygon(10, 4, align: new Alignment(Align.Center, Align.Min));
        var fullVolume = Revolve(profile, ax, 360).Value.Volume;
        int[] positive = [30, 90, 150, 180, 200, 360, 500, 720, 750];
        var sizes = positive.Reverse().Select(x => -x).Append(0).Concat(positive);
        foreach (var size in sizes)
        {
            var solid = Revolve(profile, axis: ax, revolutionArc: size).Value;

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
        var l1 = new Line(new Vector(0, 0), new Vector(12, 0));
        var l2 = new RadiusArc(l1.PositionAt(1), new Vector(15, 20), 50);
        var l3 = new Spline([l2.PositionAt(1), new Vector(22, 40), new Vector(20, 50)], tangents: [l2.TangentAt(1), new Vector(-0.75, 1)]);
        var l4 = new RadiusArc(l3.PositionAt(1), l3.PositionAt(1) + new Vector(0, 5), 5);
        var l5 = new Spline([l4.PositionAt(1), l4.PositionAt(1) + new Vector(2.5, 2.5), l4.PositionAt(1) + new Vector(0, 5)],
                            tangents: [l4.TangentAt(1), new Vector(-1, 0)]);
        var l6 = new Polyline(l5.PositionAt(1), l5.PositionAt(1) + new Vector(0, 1), new Vector(0, l5.PositionAt(1).Y + 1), l1.PositionAt(0));
        var test = Revolve(MakeFace([l1, l2, l3, l4, l5, l6]), axis: Axis.Y).Value;
        Assert.True(test.Volume < 22 * 22 * Math.PI * 50);
        Assert.True(test.Volume > 144 * Math.PI * 50);
    }

    [Fact]
    public void RevolveWithAxis()
    {
        var l1 = new Line(new Vector(0, 0), new Vector(0, 12));
        var l2 = new RadiusArc(l1.PositionAt(1), new Vector(20, 10), 50);
        var l3 = new Line(l2.PositionAt(1), new Vector(20, 0));
        var l4 = new Line(l3.PositionAt(1), l1.PositionAt(0));
        var test = Revolve(MakeFace([l1, l2, l3, l4]), axis: Axis.X).Value;
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
        var filleted = Fillet(obj.Edges().GroupBy(Axis.Z).Last(), 0.1).Value;
        var right = Split(filleted, bisectBy: Plane.YZ, keep: Keep.Top);
        Assert.True(right.Volume < filleted.Volume);
    }

    [Fact]
    public void SplitNoObjects()
    {
        var e = Assert.Throws<ArgumentNullException>(() => Split((IEnumerable<Shape>)null!));
        Assert.Equal("objects", e.ParamName);
    }

    // ---- TestThicken ----

    [Fact]
    public void ThickenFaces()
    {
        var part = Thicken(new RectangleRounded(10, 10, 1), 1).Value;
        Check.AlmostEqual(1, part.BoundingBox().Max.Z, 5);

        var nonPlanar = new Sphere(1).Faces()[0];
        var outerSphere = Thicken(nonPlanar, 0.1).Value;
        Check.AlmostEqual(4.0 / 3 * Math.PI * (Math.Pow(1.1, 3) - 1), outerSphere.Volume, 5);
    }

    [Fact]
    public void ThickenSweptSurface()
    {
        var wire = (IShape1D)(new JernArc(new Vector(0, -2), new Vector(-1, 0), 1, -180) + new JernArc(new Vector(0, 0), new Vector(1, 0), 2, -90));

        var surface = Sweep((wire.LocationAt(0)) * new RadiusArc(new Vector(0, 0), new Vector(0, -1), 1), wire).Value;
        var part = Thicken(surface, 0.4).Value;
        Check.AlmostEqual(2.241583787221904, part.Volume, 5);

        part = Thicken(Sweep((wire.LocationAt(0)) * new RadiusArc(new Vector(0, 0), new Vector(0, -1), 1), wire).Value, 0.4, new ThickenOptions { Both = true }).Value;
        Check.AlmostEqual(4.711747154435256, part.Volume, 5);
    }

    [Fact]
    public void ThickenBoth()
    {
        var part = Thicken(Face.MakeRect(2, 2), 1, new ThickenOptions { Both = true }).Value;
        Check.AlmostEqual(8, part.Volume, 5);
        Check.AlmostEqual(2, part.BoundingBox().Size.Z, 5);
    }

    [Fact]
    public void ThickenNoAmount()
    {
        // The amount is required by the signature (unit 10M2); no faces to thicken is refused
        var e = Assert.Throws<ArgumentNullException>(() => Thicken((IEnumerable<Face>)null!, 1));
        Assert.Equal("toThicken", e.ParamName);
    }

    [Fact]
    public void ThickenNoFace()
    {
        var e = Assert.Throws<ArgumentNullException>(() => Thicken((Shape)null!, 1));
        Assert.Equal("toThicken", e.ParamName);
    }
}
