// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Booleans.Legacy.Build;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// No copy at entry in the Booleans (Kerf unit 10H2, phase10-plan.md 2.4): after each kind of Boolean operation the
/// arguments are untouched, TShape for TShape and value for value (<see cref="ShapeSnapshot"/>), and the result holds
/// the arguments' own sub-shapes wherever the operation didn't change them.
/// </summary>
public class BooleanInputsUnchangedTests
{
    // [0, 10]^3 and [5, 15]^3: they overlap in [5, 10]^3; A's faces x = 0, y = 0 and z = 0 are away from B
    private static Shape BoxA() => new MakeBox(10.0, 10.0, 10.0).Shape();

    private static Shape BoxB() => new MakeBox(new Point3D(5, 5, 5), 10.0, 10.0, 10.0).Shape();

    // [10, 20] x [0, 10]^2: touches A along the face x = 10
    private static Shape BoxC() => new MakeBox(new Point3D(10, 0, 0), 10.0, 10.0, 10.0).Shape();

    private static List<Shape> SubShapes(in Shape shape, ShapeType type)
    {
        var map = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(shape, type, map);
        var list = new List<Shape>(map.Count);
        for (int i = 0; i < map.Count; i++)
        {
            list.Add(map[i]);
        }
        return list;
    }

    private static bool Holds(in Shape shape, in Shape sub)
    {
        TShape? tShape = sub.TShape();
        return SubShapes(shape, sub.Type).Exists(s => ReferenceEquals(s.TShape(), tShape));
    }

    // The face of the shape whose plane is at coordinate theValue on the axis (0: x, 1: y, 2: z)
    private static Shape PlanarFace(in Shape shape, int axis, double value)
    {
        foreach (Shape f in SubShapes(shape, ShapeType.Face))
        {
            if (Tool.Surface(ShapeCasts.Face(f)) is PlaneSurface p)
            {
                Point3D o = p.Plane.Location;
                Direction3D d = p.Plane.Axis.Direction;
                double n = axis == 0 ? d.X : axis == 1 ? d.Y : d.Z;
                double c = axis == 0 ? o.X : axis == 1 ? o.Y : o.Z;
                if (Math.Abs(Math.Abs(n) - 1.0) < 1e-12 && Math.Abs(c - value) < 1e-9)
                {
                    return f;
                }
            }
        }
        return new Shape();
    }

    private static NCollection_List<Shape> List(params Shape[] shapes)
    {
        var list = new NCollection_List<Shape>();
        foreach (Shape s in shapes)
        {
            list.Append(s);
        }
        return list;
    }

    // Runs the operation on frozen A and B, checks both untouched, and returns the result
    private static Shape OnAAndB(Func<Shape, Shape, Shape> operation, out Shape a, out Shape b)
    {
        a = BoxA();
        b = BoxB();
        ShapeSnapshot snapshotA = ShapeSnapshot.Of(a), snapshotB = ShapeSnapshot.Of(b);
        Shape result = operation(a, b);
        snapshotA.AssertUnchanged(a);
        snapshotB.AssertUnchanged(b);
        Assert.False(result.IsNull);
        return result;
    }

    private static void AssertAwayFacesShared(in Shape result, in Shape a)
    {
        for (int axis = 0; axis < 3; axis++)
        {
            Shape face = PlanarFace(a, axis, 0.0);
            Assert.False(face.IsNull);
            Assert.True(Holds(result, face), $"A's face at {axis} = 0 isn't shared with the result");
        }
    }

    [Theory]
    [InlineData("fuse")]
    [InlineData("cut")]
    [InlineData("splitter")]
    [InlineData("general fuse")]
    public void The_arguments_are_untouched_and_their_unchanged_faces_shared(string kind)
    {
        Shape result = OnAAndB((a, b) => kind switch
        {
            "fuse" => new Fuse(a, b).Shape(),
            "cut" => new Cut(a, b).Shape(),
            "splitter" => Split(a, b),
            _ => GeneralFuse(a, b),
        }, out Shape aA, out _);
        AssertAwayFacesShared(result, aA);
    }

    private static Shape Split(in Shape a, in Shape b)
    {
        SplitterOperation splitter = new();
        splitter.SetArguments(List(a));
        splitter.SetTools(List(b));
        splitter.Build();
        Assert.False(splitter.HasErrors);
        return splitter.Shape();
    }

    private static Shape GeneralFuse(in Shape a, in Shape b)
    {
        BooleanBuilder builder = new();
        builder.SetArguments(List(a, b));
        builder.Perform();
        Assert.False(builder.HasErrors);
        return builder.Shape();
    }

    [Fact]
    public void A_common_and_a_section_leave_the_arguments_untouched_and_share_their_corners()
    {
        Shape common = OnAAndB((a, b) => new Common(a, b).Shape(), out Shape aA, out Shape aB);
        // the common is [5, 10]^3: A's corner (10, 10, 10) and B's (5, 5, 5) are its own
        Assert.Contains(SubShapes(aA, ShapeType.Vertex), v => Holds(common, v));
        Assert.Contains(SubShapes(aB, ShapeType.Vertex), v => Holds(common, v));

        Shape section = OnAAndB((a, b) => new SectionOperation(a, b).Shape(), out _, out _);
        Assert.NotEmpty(SubShapes(section, ShapeType.Edge));
    }

    [Fact]
    public void The_cells_builder_never_changes_a_published_result()
    {
        Shape a = BoxA(), b = BoxB();
        ShapeSnapshot snapshotA = ShapeSnapshot.Of(a), snapshotB = ShapeSnapshot.Of(b);
        CellsBuilder cells = new();
        cells.SetArguments(List(a, b));
        cells.Perform();
        Assert.False(cells.HasErrors);
        cells.AddToResult(List(a), List(b));
        Shape first = cells.Shape(); // published: frozen
        ShapeSnapshot snapshotFirst = ShapeSnapshot.Of(first);
        cells.AddToResult(List(b), List(a));
        Shape second = cells.Shape();

        snapshotFirst.AssertUnchanged(first);
        snapshotA.AssertUnchanged(a);
        snapshotB.AssertUnchanged(b);
        Assert.Equal(first.ChildCount + 1, second.ChildCount);
        // the second result holds the first one's part itself
        Assert.True(Holds(second, SubShapes(first, ShapeType.Solid)[0]));
        AssertAwayFacesShared(second, a);
    }

    [Fact]
    public void The_checks_leave_the_argument_untouched()
    {
        Shape fused = new Fuse(BoxA(), BoxB()).Shape();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(fused);

        CheckerSI checker = new();
        checker.SetArguments(List(fused));
        checker.Perform();
        Assert.False(checker.HasErrors);
        snapshot.AssertUnchanged(fused);

        Check check = new(fused);
        Assert.True(check.IsValid);
        snapshot.AssertUnchanged(fused);
    }

    [Fact]
    public void Defeaturing_leaves_the_input_untouched_and_shares_its_unchanged_faces()
    {
        Shape shape = BOPAlgo_RemoveFeaturesTests.BoxWithHole();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(shape);
        Defeaturing defeaturing = new();
        defeaturing.SetShape(shape);
        foreach (Shape f in SubShapes(shape, ShapeType.Face))
        {
            if (Tool.Surface(ShapeCasts.Face(f)) is CylindricalSurface)
            {
                defeaturing.AddFaceToRemove(f);
            }
        }
        defeaturing.Build();
        Assert.True(defeaturing.IsDone);
        Shape result = defeaturing.Shape();
        snapshot.AssertUnchanged(shape);
        // a side face is in the result on its own surface (the face itself is a copy where UnifySameDomain, healing's,
        // adds p-curves to its edges: decisions 643 and 766, unit 10H4)
        Shape side = PlanarFace(shape, 0, 0.0);
        Surface? sideSurface = Tool.Surface(ShapeCasts.Face(side));
        Assert.Contains(SubShapes(result, ShapeType.Face), f => ReferenceEquals(Tool.Surface(ShapeCasts.Face(f)), sideSurface));
    }

    [Fact]
    public void Connected_and_periodic_shapes_leave_the_arguments_untouched_and_share_what_they_dont_change()
    {
        Shape a = BoxA(), c = BoxC();
        ShapeSnapshot snapshotA = ShapeSnapshot.Of(a), snapshotC = ShapeSnapshot.Of(c);
        MakeConnectedAlgorithm connected = new();
        connected.AddArgument(a);
        connected.AddArgument(c);
        connected.Perform();
        Assert.False(connected.HasErrors);
        Shape glued = connected.Shape(); // published: frozen
        ShapeSnapshot snapshotGlued = ShapeSnapshot.Of(glued);
        Assert.True(Holds(glued, PlanarFace(a, 0, 0.0)));

        MakePeriodicAlgorithm.PeriodicityParams parameters = new();
        parameters.myPeriodic[0] = true;
        parameters.myPeriod[0] = 20.0;
        parameters.myIsTrimmed[0] = true;
        connected.MakePeriodic(parameters);
        Assert.False(connected.HasErrors);
        connected.RepeatShape(0, 2);
        Shape repeated = connected.PeriodicShape();
        Assert.False(repeated.IsNull);

        snapshotA.AssertUnchanged(a);
        snapshotC.AssertUnchanged(c);
        snapshotGlued.AssertUnchanged(glued);
        // what the periodicity and the gluing of the repetitions don't change is the connected shape's own
        Assert.Contains(SubShapes(glued, ShapeType.Vertex), v => Holds(repeated, v));
    }

    [Fact]
    public void A_trimmed_periodic_shape_leaves_its_input_untouched()
    {
        Shape box = BoxA();
        ShapeSnapshot snapshot = ShapeSnapshot.Of(box);
        MakePeriodicAlgorithm maker = new();
        maker.SetShape(box);
        maker.MakePeriodic(0, true, 5);
        maker.SetTrimmed(0, false, 2);
        maker.Perform();
        Assert.False(maker.HasErrors);
        Shape periodic = maker.Shape();
        Shape repeated = maker.RepeatShape(0, 3);
        Assert.False(maker.HasWarnings);
        Assert.Equal(4, SubShapes(repeated, ShapeType.Solid).Count);
        snapshot.AssertUnchanged(box);
        // the repetition glues copies of the published periodic shape, which never changes
        Assert.False(periodic.IsNull);
    }

    // A closed square of frozen edges in the plane z = 0
    private static Wire Square(double x0, double y0, double size)
        => new MakePolygon(new Point3D(x0, y0, 0), new Point3D(x0 + size, y0, 0), new Point3D(x0 + size, y0 + size, 0),
                           new Point3D(x0, y0 + size, 0), true).Wire();

    [Fact]
    public void The_wire_tools_leave_the_wires_untouched_and_share_their_vertices()
    {
        Wire square = Square(0, 0, 10);
        ShapeSnapshot snapshot = ShapeSnapshot.Of(square);

        Shape faces = new();
        Assert.True(BuilderTools.WiresToFaces(square, ref faces));
        snapshot.AssertUnchanged(square);
        Assert.Contains(SubShapes(square, ShapeType.Vertex), v => Holds(faces, v));

        Wire corner = new MakePolygon(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0)).Wire();
        ShapeSnapshot snapshotCorner = ShapeSnapshot.Of(corner);
        Edge edge = BRepAlgo.ConcatenateWireC0(corner);
        snapshotCorner.AssertUnchanged(corner);
        Assert.False(edge.IsNull);
        Assert.Contains(SubShapes(corner, ShapeType.Vertex), v => Holds(edge, v));
    }

    [Fact]
    public void The_face_restrictor_and_wire_to_face_leave_their_inputs_untouched()
    {
        Face face = ShapeCasts.Face(PlanarFace(BoxA(), 2, 0.0));
        Wire inner = Square(2, 2, 6);
        ShapeSnapshot snapshotFace = ShapeSnapshot.Of(face), snapshotWire = ShapeSnapshot.Of(inner);

        FaceRestrictor restrictor = new();
        restrictor.Init(face);
        restrictor.Add(inner);
        restrictor.Perform();
        Assert.True(restrictor.IsDone);
        Assert.True(restrictor.More());
        Face restricted = restrictor.Current();
        snapshotFace.AssertUnchanged(face);
        snapshotWire.AssertUnchanged(inner);
        Assert.Contains(SubShapes(inner, ShapeType.Edge), e => Holds(restricted, e));

        WireToFace wireToFace = new();
        wireToFace.AddWire(inner);
        var faces = new NCollection_List<Shape>();
        wireToFace.MakeFaces(face, faces);
        Assert.False(faces.IsEmpty());
        snapshotFace.AssertUnchanged(face);
        snapshotWire.AssertUnchanged(inner);
        Assert.Contains(SubShapes(inner, ShapeType.Edge), e => Holds(faces.First(), e));
    }
}
