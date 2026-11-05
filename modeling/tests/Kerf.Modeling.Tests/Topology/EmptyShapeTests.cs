// Ported from build123d tests/test_direct_api/test_empty_shapes.py. build123d's Box(1, 1, 1) is a Part and
// Rectangle(1, 1) a Sketch, so they are written here as a Part around a centred box solid and a Sketch around
// Face.MakeRect. Omitted:
// - test_setting_wrapped_to_none_empties: shapes are immutable.
// - test_is_null_is_deprecated: is_null is IsEmpty, without a deprecated alias.
// - TestEmptyShapeCopying: copy.copy, copy.deepcopy and pickle have no counterpart (shapes are immutable).
// - test_moving_keeps_the_zero's move/locate "is" checks: those are mutators; the copies are checked instead.
// - test_face_without_a_surface's uv_face: phase 7.
// - test_a_boolean_that_leaves_nothing_returns_a_typed_zero's _bool_op_list checks: the method is private.
// - TestEmptyShapeBuilders: builders aren't ported yet.
// - test_split_sides_that_have_nothing's last check (Box.split is a Part): the split here is always a list.
// - TestEmptyShapeExport: the exporters aren't ported yet.
// - test_a_kernel_empty_is_the_zero_of_any_class's Face(empty): the Face constructor takes a TopoDS_Face, and
//   Shape.Cast is static, so Solid.cast's class-specific result is checked through the internal CastLike.
using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Topology;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

file static class Zeros
{
    // Each class with its zero and its dimension
    public static readonly (Type Class, Func<Shape> Zero, int? Dim)[] Classes =
    [
        (typeof(Vertex), () => default(Vertex), 0),
        (typeof(Edge), () => default(Edge), 1),
        (typeof(Wire), () => default(Wire), 1),
        (typeof(Face), () => default(Face), 2),
        (typeof(Shell), () => default(Shell), 2),
        (typeof(Solid), () => default(Solid), 3),
        (typeof(Compound), () => default(Compound), null),
        (typeof(Part), () => new Part(), 3),
        (typeof(Sketch), () => new Sketch(), 2),
        (typeof(Curve), () => new Curve(), 1),
    ];

    public static Solid Box(double l, double w, double h) => Solid.MakeBox(l, w, h, new Plane(new Vector(-l / 2, -w / 2, -h / 2)));

    public static Part BoxPart(double l, double w, double h) => new([Box(l, w, h)]);

    public static Sketch Rectangle(double w, double h) => new([Face.MakeRect(w, h)]);
}

public class EmptyShapeDefinitionTests
{
    // The no-argument constructor of every class is its zero

    [Fact]
    public void WhatAZeroIs()
    {
        foreach (var (cls, make, dim) in Zeros.Classes)
        {
            var zero = make();
            Assert.IsType(cls, zero);
            Assert.True(zero.IsEmpty);
            Assert.Equal(ShapeType.Compound, zero.Type);
            Assert.Equal(dim, zero.Dim);
        }
    }

    [Fact]
    public void ShapeTypeIsTheClass()
    {
        Assert.Equal(ShapeType.Solid, default(Solid).ShapeType);
        Assert.Equal(ShapeType.Face, default(Face).ShapeType);
        Assert.Equal(ShapeType.Vertex, default(Vertex).ShapeType);
        Assert.Equal(ShapeType.Compound, new Part().ShapeType);
    }

    [Fact]
    public void VertexZeroIsNotTheOrigin()
    {
        Assert.True(default(Vertex).IsEmpty);
        Assert.False(Vertex.Make(0, 0, 0).IsEmpty);
        Assert.Equal(0, Vertex.Make(0, 0, 0).X);
    }

    [Fact]
    public void EveryZeroHasARepr()
    {
        Assert.Equal("Vertex()", default(Vertex).ToString());
        foreach (var (_, make, _) in Zeros.Classes)
            Assert.NotNull(make().ToString());
    }

    [Fact]
    public void AKernelEmptyIsTheZeroOfAnyClass()
    {
        var empty = (Solid.MakeBox(1, 1, 1) - Solid.MakeBox(1, 1, 1));
        Assert.True((empty).IsEmpty);
        Check.Is<Solid>((empty));
        Assert.True((empty).IsEmpty);
        Check.Is<Compound>(Shape.Cast(empty));
    }
}

public class EmptyShapeEqualityTests
{
    [Fact]
    public void ZerosOfOneClassAreEqual()
    {
        foreach (var (_, make, _) in Zeros.Classes)
        {
            Check.Same(make(), make());
            Assert.Equal(make().GetHashCode(), make().GetHashCode());
        }
    }

    [Fact]
    public void ZerosOfDifferentClassesAreNot()
    {
        Check.NotSame(default(Solid), default(Face));
        Check.NotSame(new Part(), default(Compound));
    }

    [Fact]
    public void AZeroIsNotAShape()
    {
        var box = Solid.MakeBox(1, 1, 1);
        Check.NotSame(default(Solid), box);
        Check.NotSame(box, default(Solid));
    }
}

public class EmptyShapePlacementTests
{
    // Nothing is nothing wherever it is put

    [Fact]
    public void MovingKeepsTheZero()
    {
        var loc = new Location(new Vector(1, 2, 3), new Vector(10, 20, 30));
        foreach (var (cls, make, _) in Zeros.Classes)
        {
            var zero = make();
            Shape[] placed =
            [
                zero.Moved(loc),
                zero.Located(loc),
                loc * zero,
                Plane.XZ * zero,
                zero.Rotate(Axis.Z, 30),
                zero.Translate(new Vector(1, 2, 3)),
                zero.Mirror(Plane.XY),
                zero.Scale(2),
            ];
            foreach (var p in placed)
            {
                Assert.True(p.IsEmpty);
                Assert.IsType(cls, p);
            }
        }
    }

    [Fact]
    public void AZeroHasNoPlace()
    {
        foreach (var (_, make, _) in Zeros.Classes)
        {
            Assert.Contains("no location", Assert.Throws<InvalidOperationException>(() => make().Location).Message);
            Assert.Contains("no position", Assert.Throws<InvalidOperationException>(() => make().Position).Message);
            Assert.Contains("no orientation", Assert.Throws<InvalidOperationException>(() => make().Orientation).Message);
            Assert.Contains("no center", Assert.Throws<InvalidOperationException>(() => make().Center()).Message);
        }
    }
}

public class EmptyShapeQueriesTests
{
    [Fact]
    public void SelectorsFindNothing()
    {
        foreach (var (_, make, _) in Zeros.Classes)
        {
            var zero = make();
            Assert.Empty(zero.SelectVertices());
            Assert.Empty(zero.SelectEdges());
            Assert.Empty(zero.SelectWires());
            Assert.Empty(zero.SelectFaces());
            Assert.Empty(zero.SelectShells());
            Assert.Empty(zero.SelectSolids());
            Assert.Empty(zero.GetTopLevelShapes());
        }
    }

    [Fact]
    public void AmountsAreZero()
    {
        foreach (var (_, make, _) in Zeros.Classes)
        {
            var zero = make();
            Assert.Equal(0, zero.Volume);
            Assert.Equal(0, zero.Area);
            Assert.Equal((0.0, 0.0, 0.0), zero.StaticMoments);
            Assert.Equal([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]], zero.MatrixOfInertia);
        }
    }

    [Fact]
    public void DirectionsRaise()
    {
        Assert.Contains("no principal axes",
                        Assert.Throws<InvalidOperationException>(() => default(Solid).PrincipalProperties).Message);
        Assert.Contains("no radius of gyration",
                        Assert.Throws<InvalidOperationException>(() => default(Solid).RadiusOfGyration(Axis.Z)).Message);
    }

    [Fact]
    public void BoundingBoxIsEmpty()
    {
        var box = Solid.MakeBox(1, 1, 1).BoundingBox();
        static (double, double, double) Size(BoundBox b) => (b.Size.X, b.Size.Y, b.Size.Z);
        foreach (var (_, make, _) in Zeros.Classes)
        {
            var empty = make().BoundingBox();
            Assert.True(empty.IsEmpty);
            Assert.Equal((0.0, 0.0, 0.0), Size(empty));
            // The bounding box of nothing adds nothing
            Assert.Equal((1.0, 1.0, 1.0), Size(box.Add(empty)));
            Assert.Equal((1.0, 1.0, 1.0), Size(empty.Add(box)));
        }
        Assert.False(box.IsEmpty);
    }

    [Fact]
    public void Validity()
    {
        foreach (var (_, make, _) in Zeros.Classes)
        {
            Assert.True(make().IsValid);
            Assert.True(make().Clean().IsEmpty);
            Assert.True(make().Fix().IsEmpty);
        }
    }
}

public class EmptyShapeAlgebraTests
{
    // The zero is the identity for + and absorbs - and &

    private static (Type Class, Func<Shape> Zero, Shape Other)[] Others() =>
    [
        (typeof(Vertex), () => default(Vertex), Vertex.Make(1, 2, 3)),
        (typeof(Edge), () => default(Edge), Edge.MakeLine(new Vector(0, 0, 0), new Vector(1, 0, 0))),
        (typeof(Wire), () => default(Wire), Wire.MakeRect(1, 1)),
        (typeof(Face), () => default(Face), Face.MakeRect(1, 1)),
        (typeof(Solid), () => default(Solid), Solid.MakeBox(1, 1, 1)),
        (typeof(Part), () => new Part(), Zeros.BoxPart(1, 1, 1)),
        (typeof(Sketch), () => new Sketch(), Zeros.Rectangle(1, 1)),
        (typeof(Curve), () => new Curve(), new Curve() + Edge.MakeLine(new Vector(0, 0, 0), new Vector(1, 0, 0))),
    ];

    private static (double, double, double) Size(Shape shape) =>
        (shape.Volume, shape.Area, shape.SelectEdges().Sum(e => e.Length));

    [Fact]
    public void IdentityForAddition()
    {
        foreach (var (cls, zero, other) in Others())
        {
            if (cls == typeof(Vertex))
                continue;   // Vertex + is vector arithmetic
            // Adding nothing gives the shape itself back
            Check.Same(other, other + zero());
            Check.Same(other, other - zero());
            // Nothing plus the shape is the shape's geometry, in a shape of the same dimension
            var total = zero() + other;
            Assert.Equal(zero().Dim, total.Dim);
            Assert.Equal(Size(other), Size(total));
            Assert.True((zero() + zero()).IsEmpty);
        }
    }

    [Fact]
    public void Absorption()
    {
        foreach (var (cls, zero, other) in Others())
        {
            if (cls == typeof(Vertex))
                continue;   // Vertex - and & are not boolean operations
            foreach (var result in new[] { zero() - other, zero() & other, other & zero() })
            {
                Assert.True(result.IsEmpty);
                Assert.IsType(cls, result);
            }
        }
    }

    [Fact]
    public void AnUntypedEmptyCompoundIsTheIdentityForAnything()
    {
        var box = Zeros.BoxPart(1, 1, 1);
        Assert.Equal(Size(box), Size(default(Compound) + box));
        Assert.Equal(Size(Zeros.Rectangle(1, 1)), Size(default(Compound) + Zeros.Rectangle(1, 1)));
        Assert.True((default(Compound) - box).IsEmpty);
        Assert.True((default(Compound) & box).IsEmpty);
    }

    [Fact]
    public void ABooleanThatLeavesNothingReturnsATypedZero()
    {
        var box = Zeros.BoxPart(1, 1, 1);
        var apart = new Pos(5, 0, 0) * Zeros.BoxPart(1, 1, 1);
        (Shape Result, Type Class, int Dim)[] cases =
        [
            (box - box, typeof(Part), 3),
            (box & apart, typeof(Part), 3),
            (Solid.MakeBox(1, 1, 1) - Solid.MakeBox(1, 1, 1), typeof(Part), 3),
            (Zeros.Rectangle(1, 1) - Zeros.Rectangle(1, 1), typeof(Sketch), 2),
            (Face.MakeRect(1, 1) & new Pos(5, 0, 0) * Face.MakeRect(1, 1), typeof(Face), 2),
        ];
        foreach (var (result, cls, dim) in cases)
        {
            Assert.True(result.IsEmpty);
            Assert.IsType(cls, result);
            Assert.Equal(dim, result.Dim);
        }
        // And the zero carries on through the next operation
        Assert.True(((box - box) - apart).IsEmpty);
        Assert.True(((box - box) & apart).IsEmpty);
        Check.AlmostEqual(1, ((box - box) + apart).Volume, 5);
    }

    [Fact]
    public void IntersectFindsNothingAsAnEmptyList()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var apart = new Pos(5, 0, 0) * Solid.MakeBox(1, 1, 1);
        Assert.Empty(box.Intersect(apart));
        Assert.Empty(box.Intersect(default(Solid)));
        Assert.Empty(default(Solid).Intersect(box));
        Assert.Empty(box.Intersect());
        Assert.Single(box.Intersect(box));
    }

    [Fact]
    public void FuseCutAndIntersectMethods()
    {
        var box = Solid.MakeBox(1, 1, 1);
        Check.Same(box, box.Fuse(default(Solid)));
        Check.Same(box, box.Cut(default(Solid)));
        Check.Same(box, default(Solid).Fuse(box));
        Assert.True(default(Solid).Cut(box).IsEmpty);
    }
}

public class EmptyShapeGeometryQueriesTests
{
    // What needs real geometry says so; what asks about its kind says null

    [Fact]
    public void VertexHasNoCoordinates()
    {
        Func<double>[] coordinates = [() => default(Vertex).X, () => default(Vertex).Y, () => default(Vertex).Z];
        foreach (var coordinate in coordinates)
            Assert.Contains("empty Vertex has no position",
                            Assert.Throws<InvalidOperationException>(() => coordinate()).Message);
        Assert.Contains("empty Vertex has no position",
                        Assert.Throws<InvalidOperationException>(() => default(Vertex) + new Vector(1, 2, 3)).Message);
    }

    [Fact]
    public void FaceWithoutASurface()
    {
        Action[] asked = [() => default(Face).GeomAdaptor(), () => default(Face).OuterWire(), () => default(Face).NormalAt()];
        foreach (var ask in asked)
            Assert.Contains("An empty Face has no", Assert.Throws<InvalidOperationException>(ask).Message);
        Assert.Contains("An empty Face has no center",
                        Assert.Throws<InvalidOperationException>(() => default(Face).CenterLocation).Message);
        Assert.Empty(default(Face).InnerWires());
    }

    [Fact]
    public void ProbesOfAKindAnswerNone()
    {
        var empty = default(Face);
        Assert.Null(empty.Geometry);
        Assert.Null(empty.IsPlanar);
        Assert.Null(empty.Length);
        Assert.Null(empty.Width);
        Assert.Null(empty.Radius);
        Assert.Null(empty.AxisOfRotation);
        Assert.Null(default(Edge).CommonPlane());
        Assert.Null(default(Wire).CommonPlane());
    }

    [Fact]
    public void NothingMeetsNothing()
    {
        var line = Edge.MakeLine(new Vector(0, 0, 0), new Vector(1, 0, 0));
        foreach (var zero in new Shape[] { default(Edge), default(Wire), default(Face), default(Shell) })
        {
            Assert.Empty(zero.Intersect(line));
            Assert.Empty(line.Intersect(zero));
        }
    }

    [Fact]
    public void OrientedBoundingBoxOfNothing()
    {
        var box = default(Solid).OrientedBoundingBox();
        Assert.True(box.IsEmpty);
        Assert.Equal((0.0, 0.0, 0.0), (box.Size.X, box.Size.Y, box.Size.Z));
        Assert.Equal(Plane.XY, box.Plane);
        Assert.False(Solid.MakeBox(1, 1, 1).OrientedBoundingBox().IsEmpty);
    }
}

public class EmptyShapeOperationsTests
{
    // An operation on nothing gives nothing, of the dimension it would make

    [Fact]
    public void OperationsOnAZero()
    {
        var line = Edge.MakeLine(new Vector(0, 0, 0), new Vector(1, 0, 0));
        (string Name, Func<Shape> Operation, Type Class)[] cases =
        [
            ("extrude", () => Ops.Extrude(new Sketch(), 1).Value, typeof(Part)),
            ("revolve", () => Ops.Revolve(new Sketch(), Axis.X).Value, typeof(Part)),
            ("offset part", () => Ops.Offset(new Part(), 1).Value, typeof(Part)),
            ("offset sketch", () => Ops.Offset(new Sketch(), 1).Value, typeof(Sketch)),
            ("mirror", () => Ops.Mirror(new Part(), Plane.XY), typeof(Part)),
            ("scale", () => Ops.Scale(new Part(), 2), typeof(Part)),
            ("split", () => Ops.Split(new Part(), Plane.XY), typeof(Part)),
            ("project", () => Ops.Project(default(Face), Plane.XY), typeof(Sketch)),
            ("section", () => Ops.Section(new Part(), Plane.XY), typeof(Sketch)),
            ("thicken", () => Ops.Thicken(new Sketch(), 1).Value, typeof(Part)),
            ("sweep", () => Ops.Sweep(new Sketch(), line).Value, typeof(Part)),
            ("loft", () => Ops.Loft([default(Face), default(Face)]).Value, typeof(Part)),
            ("make_face", () => Ops.MakeFace(new Curve()), typeof(Sketch)),
            ("make_hull", () => Ops.MakeHull(new Curve().Edges()), typeof(Sketch)),
            ("trace", () => Ops.Trace(new Curve(), 1), typeof(Sketch)),
            ("bounding_box", () => Ops.BoundingBox(new Part()), typeof(Part)),
        ];
        foreach (var (name, operation, cls) in cases)
        {
            var result = operation();
            Assert.True(cls == result.GetType(), $"{name} gave a {result.GetType().Name}");
            Assert.True(result.IsEmpty, $"{name} gave something");
        }
    }

    [Fact]
    public void AnEmptySectionIsNoSection()
    {
        // a loft through one real section and one empty one has one section
        var e = Assert.Throws<ArgumentException>(() => Ops.Loft([new Rectangle(1, 1), default(Face)]));
        Assert.Contains("More than one", e.Message);
    }

    [Fact]
    public void OperationsThatNeedSomething()
    {
        Assert.Contains("Nothing to fillet", Assert.Throws<ArgumentException>(() => Ops.Fillet(ShapeList<Edge>.Empty, 1)).Message);
        Assert.Contains("Nothing to chamfer", Assert.Throws<ArgumentException>(() => Ops.Chamfer(ShapeList<Edge>.Empty, 1)).Message);
        Assert.Contains("empty path", Assert.Throws<ArgumentException>(() => Ops.Sweep(new Rectangle(1, 1), new Curve())).Message);
        Assert.Contains("empty tool",
                        Assert.Throws<ArgumentException>(() => Solid.MakeBox(1, 1, 1).Split(default(Face))).Message);
    }

    [Fact]
    public void SplitSidesThatHaveNothing()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var beyond = Plane.XY.Offset(5);
        Assert.Empty(box.Split(beyond));
        var (top, bottom) = box.SplitBoth(beyond);
        Assert.Empty(top);
        Check.AlmostEqual(1, bottom.Sum(s => s.Volume), 5);
        Assert.Empty(default(Solid).Split(beyond));
        Assert.Empty(default(Solid).Split(beyond, Keep.All));
    }

    [Fact]
    public void SplitByPerimeterOutsideTheFace()
    {
        var face = Face.MakeRect(4, 4);
        var elsewhere = new Pos(10, 0, 0) * Wire.MakeRect(1, 1);
        Assert.True(face.SplitByPerimeter(elsewhere, Keep.Inside).IsEmpty);
        Assert.True(default(Face).SplitByPerimeter(Wire.MakeRect(1, 1), Keep.Inside).IsEmpty);
    }

    [Fact]
    public void TrimAndCommonVertexFindNothing()
    {
        var line = Edge.MakeLine(new Vector(0, 0, 0), new Vector(1, 0, 0));
        var apart = Edge.MakeLine(new Vector(0, 5, 0), new Vector(1, 5, 0));
        Assert.True(line.TrimToOther(apart).IsEmpty);
        Assert.True(default(Edge).TrimToOther(line).IsEmpty);
        Assert.True(Vertex.CommonVertex(line, apart).IsEmpty);
    }
}

public class EmptyShapeContainersTests
{
    [Fact]
    public void ACompoundOfNothingIsNothing()
    {
        Assert.True(Compound.Make([]).IsEmpty);
        Assert.True(Compound.Make([default(Solid), default(Face)]).IsEmpty);
        Assert.True(Compound.Assembly([]).IsEmpty);
    }

    [Fact]
    public void ZerosAddNothingToACompound()
    {
        var box = Solid.MakeBox(1, 1, 1);
        var mixed = Compound.Make([default(Solid), box, default(Face)]);
        Assert.False(mixed.IsEmpty);
        Assert.Single(mixed.SelectSolids());
        Check.AlmostEqual(1, mixed.Volume, 5);
    }
}
