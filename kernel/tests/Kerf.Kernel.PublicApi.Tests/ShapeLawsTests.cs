// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// What every kind of shape keeps (question 49): its orientation, its placement, sameness and equality, and its
/// sub-shapes, for each of the eight types through their own members.
/// </summary>
public class ShapeLawsTests
{
    private static Solid Box() => Solid.Box(10, 10, 10).Value.Shape;

    private static Placement Moved => new(Transform3D.Translation(new Vector3D(1, 0, 0)));

    private static CompSolid MakeCompSolid()
    {
        Solid a = Box();
        Solid b = (Solid)a.Moved(new Placement(Transform3D.Translation(new Vector3D(20, 0, 0))));
        return CompSolid.Create([a, b]);
    }

    public static IEnumerable<object[]> Shapes =>
    [
        [(Shape)Box(), ShapeType.Solid],
        [(Shape)Box().Shells()[0], ShapeType.Shell],
        [(Shape)Box().Faces()[0], ShapeType.Face],
        [(Shape)Box().Wires()[0], ShapeType.Wire],
        [(Shape)Box().Edges()[0], ShapeType.Edge],
        [(Shape)Box().Vertices()[0], ShapeType.Vertex],
        [(Shape)Compound.Create([Box()]), ShapeType.Compound],
        [(Shape)MakeCompSolid(), ShapeType.CompSolid],
    ];

    [Theory]
    [MemberData(nameof(Shapes))]
    public void A_shape_keeps_its_laws(Shape general, ShapeType type)
    {
        general = general.Oriented(Orientation.Forward);
        object shape = type switch
        {
            ShapeType.Solid => (Solid)general,
            ShapeType.Shell => (Shell)general,
            ShapeType.Face => (Face)general,
            ShapeType.Wire => (Wire)general,
            ShapeType.Edge => (Edge)general,
            ShapeType.Vertex => (Vertex)general,
            ShapeType.Compound => (Compound)general,
            _ => (CompSolid)general,
        };
        Assert.Equal(type, (ShapeType)Get(shape, "Type"));
        Assert.Equal(Orientation.Forward, (Orientation)Get(shape, "Orientation"));
        Assert.True(((Placement)Get(shape, "Placement")).IsIdentity);
        Assert.False((bool)Get(shape, "IsInfinite"));
        Assert.Equal(general.IsClosed, (bool)Get(shape, "IsClosed"));
        Assert.Equal(general.IsOrientable, (bool)Get(shape, "IsOrientable"));
        Assert.Equal(general.ChildCount, (int)Get(shape, "ChildCount"));

        // orientation
        var reversed = (Shape)Call(shape, "Reversed");
        Assert.Equal(Orientation.Reversed, reversed.Orientation);
        Assert.Equal(Orientation.Reversed, ((Shape)Call(shape, "Complemented")).Orientation);
        Assert.Equal(Orientation.Reversed, ((Shape)Call(shape, "Composed", Orientation.Reversed)).Orientation);
        Assert.Equal(Orientation.Reversed, general.Composed(Orientation.Reversed).Orientation);
        Assert.Equal(Orientation.Internal, ((Shape)Call(shape, "Oriented", Orientation.Internal)).Orientation);

        // the same TShape: same, partner, not equal
        Assert.True((bool)Call(shape, "IsSame", general));
        Assert.True((bool)Call(shape, "IsPartner", reversed));
        Assert.False((bool)Call(shape, "IsEqual", reversed));
        var moved = (Shape)Call(shape, "Moved", Moved, true);
        Assert.True((bool)Call(shape, "IsPartner", moved));
        Assert.False((bool)Call(shape, "IsSame", moved));
        Assert.Equal(Moved, ((Shape)Call(shape, "Located", Moved, true)).Placement);

        // equality, on its own type
        Assert.True((bool)Call(shape, "Equals", shape));
        Assert.True(shape.Equals(shape));
        Assert.False(shape.Equals("something else"));
        Assert.Equal(shape.GetHashCode(), general.GetHashCode());
        Assert.False((bool)Call(null, shape.GetType(), "op_Inequality", shape, shape));
        Assert.Contains(type.ToString(), shape.ToString(), StringComparison.OrdinalIgnoreCase);
    }

    private static object Get(object target, string property) => target.GetType().GetProperty(property)!.GetValue(target)!;

    private static object Call(object target, string name, params object[] arguments) => Call(target, target.GetType(), name, arguments);

    private static object Call(object? target, Type type, string name, params object[] arguments)
    {
        var method = type.GetMethods().Single(m => m.Name == name && m.GetParameters()
            .Select(p => p.ParameterType.IsByRef ? p.ParameterType.GetElementType()! : p.ParameterType)
            .SequenceEqual(arguments.Select(a => a.GetType())));
        return method.Invoke(target, arguments)!;
    }

    [Fact]
    public void Each_type_explores_its_sub_shapes()
    {
        Solid box = Box();
        Assert.Equal(1, Count(box.Explore<Shell>()));
        Assert.Equal(6, Count(box.Shells()[0].Explore<Face>()));
        Assert.Equal(4, Count(box.Wires()[0].Explore<Edge>()));
        Assert.Equal(0, Count(box.Vertices()[0].Explore<Edge>()));
        Assert.Equal(8, Count(box.Faces()[0].Explore<Vertex>()));
        Assert.Equal(1, Count(Compound.Create([box]).Explore<Solid>()));
        Assert.Equal(2, Count(MakeCompSolid().Explore<Solid>()));
        // avoiding wires, no edges are reached
        Assert.Equal(0, Count(box.Faces()[0].Explore<Edge>(ShapeType.Wire)));
    }

    private static int Count<T>(SubShapes<T> shapes) where T : struct, IShape<T>
    {
        int n = 0;
        foreach (T _ in shapes)
        {
            n++;
        }
        return n;
    }

    [Fact]
    public void A_vertex_edge_and_face_read_back()
    {
        Solid box = Box();
        Assert.Equal(1, box.Faces()[0].ChildCount);
        Assert.Equal(1e-7, box.Faces()[0].Tolerance, 12);
        Assert.False(box.IsInfinite);
        Assert.Equal(0, box.Vertices()[0].ChildCount);
        Assert.Equal(2, box.Edges()[0].ChildCount);
        Assert.Throws<InvalidCastException>(() => (CompSolid)(Shape)box);
        Assert.Equal(ShapeType.CompSolid, ((CompSolid)(Shape)MakeCompSolid()).Type);
        Assert.Equal(1000, box.Oriented(Orientation.Forward).Volume(), 6);
    }

    [Fact]
    public void A_placement_compares_and_prints()
    {
        Placement here = new(), there = Moved;
        Assert.True(here != there);
        Assert.False(here.Equals((object)there));
        Assert.True(there.Equals((object)Moved));
        Assert.False(string.IsNullOrEmpty(there.ToString()));
    }

    [Fact]
    public void TheDimensionInterfacesSayWhatEachShapeIs()
    {
        Solid box = Box();
        Face face = box.Faces()[0];
        Edge edge = box.Edges()[0];

        Assert.True(((object)edge) is IShape1D);
        Assert.True(((object)Wire.Create([edge])) is IShape1D);
        Assert.True(((object)face) is IShape2D);
        Assert.True(((object)box.Shells()[0]) is IShape2D);
        Assert.True(((object)box) is IShape3D);
        Assert.True(((object)MakeCompSolid()) is IShape3D);
        Assert.True(((object)Compound.Create([box])) is IShape3D);
        Assert.False(((object)box.Vertices()[0]) is IShape1D or IShape2D or IShape3D);
        Assert.False(((object)(Shape)edge) is IShape1D);
        // Non-generic, as modelling objects implement it
        Assert.True(AsShapeOf<IShape>(face).IsSame(face));
        Assert.True(AsShapeOf(edge).IsSame(edge));
    }

    private static Shape AsShapeOf<T>(T shape) where T : IShape => shape.AsShape();

    [Fact]
    public void ATypedComparerComparesAsShapeComparerDoes()
    {
        Edge edge = Box().Edges()[0];
        Edge reversed = (Edge)edge.Reversed();

        Assert.True(ShapeComparer<Edge>.Same.Equals(edge, reversed));
        Assert.False(ShapeComparer<Edge>.Equal.Equals(edge, reversed));
        Assert.Equal(ShapeComparer<Edge>.Same.GetHashCode(edge), ShapeComparer<Edge>.Same.GetHashCode(reversed));
        Assert.Single(new[] { edge, reversed }.Distinct(ShapeComparer<Edge>.Same));
        Assert.Equal(2, new[] { edge, reversed }.Distinct(ShapeComparer<Edge>.Equal).Count());
        Assert.True(ShapeComparer<IShape>.Same.Equals(null, null));
        Assert.False(ShapeComparer<IShape>.Same.Equals(edge, null));
    }
}
