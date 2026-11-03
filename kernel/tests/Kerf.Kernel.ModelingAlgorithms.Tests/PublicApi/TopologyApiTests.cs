// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using EngineBox = Kerf.Engine.Construction.Api.MakeBox;
using EngineCylinder = Kerf.Engine.Construction.Api.ApiMakeCylinder;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.PublicApi;

/// <summary>
/// Topology's public API (docs/api/topology.md), as a user calls it. Only the fixtures come from the engine: the
/// construction area's public primitives are unit 10L2's. Everything asserted goes through Kerf.Topology.
/// </summary>
public class TopologyApiTests
{
    private static Solid Box() => (Solid)new EngineBox(1, 2, 3).Shape();

    private static Solid Cylinder() => (Solid)new EngineCylinder(1, 2).Shape();

    [Fact]
    public void A_shape_has_its_type_placement_and_orientation_as_properties()
    {
        Solid box = Box();
        Shape shape = box;
        Assert.Equal(ShapeType.Solid, shape.Type);
        Assert.Equal(Orientation.Forward, shape.Orientation);
        Assert.True(shape.Placement.IsIdentity);
        Assert.False(shape.IsNull);
        Assert.True(default(Shape).IsNull);
        Assert.Equal(1, shape.ChildCount);
    }

    [Fact]
    public void Casts_check_the_type()
    {
        Shape shape = Box();
        Assert.True(shape.TryAs(out Solid solid));
        Assert.Equal(shape, solid.AsShape());
        Assert.False(shape.TryAs(out Face _));
        Assert.Throws<InvalidCastException>(() => (Face)shape);
        Assert.Throws<InvalidCastException>(() => shape.As<Edge>());
        Solid same = (Solid)shape;
        Assert.True(same.IsSame(shape));
        Assert.True(shape.TryAs(out Shape itself) && itself == shape);
    }

    [Fact]
    public void Traversal_finds_each_sub_shape_once()
    {
        Solid box = Box();
        Assert.Equal(6, box.Faces().Length);
        Assert.Equal(12, box.Edges().Length);
        Assert.Equal(8, box.Vertices().Length);
        Assert.Equal(6, box.Wires().Length);
        Assert.Single(box.Shells());
        Assert.Single(box.Solids());
        Assert.Equal(24, box.Explore<Edge>().Count());
        Assert.Equal(6, box.Explore<Face>().Count());
        Assert.Empty(box.Explore<Edge>(avoid: ShapeType.Face));
        Assert.Single(box.Children);
        Assert.Equal(ShapeType.Shell, box.Children.Single().Type);
        Assert.Throws<ArgumentException>(() => box.Explore<Shape>());
    }

    [Fact]
    public void Comparers_tell_same_from_equal()
    {
        Solid box = Box();
        Face face = box.Faces()[0];
        Shape reversed = face.Reversed();
        Assert.True(ShapeComparer.Same.Equals(face, reversed));
        Assert.False(ShapeComparer.Equal.Equals(face, reversed));
        Assert.Equal(ShapeComparer.Same.GetHashCode(face), ShapeComparer.Same.GetHashCode(reversed));
        var set = new HashSet<Shape>(ShapeComparer.Same);
        foreach (Edge edge in box.Explore<Edge>())
        {
            set.Add(edge);
        }

        Assert.Equal(12, set.Count);
    }

    [Fact]
    public void Raw_assembly_makes_shapes_from_parts()
    {
        Solid box = Box();
        Solid cylinder = Cylinder();
        Compound both = Compound.Create([box, cylinder.Moved(Transform3D.Translation(new Vector3D(5, 0, 0)))]);
        Assert.Equal(2, both.ChildCount);
        Assert.Equal(2, both.Solids().Length);
        Assert.Equal(2, both.TopLevelShapes().Length);

        Shell shell = Shell.Create(box.Faces());
        Assert.Equal(6, shell.Faces().Length);
        Wire wire = Wire.Create(box.Faces()[0].Edges());
        Assert.Equal(4, wire.Edges().Length);
        Solid solid = Solid.Create(box.Shells());
        Assert.Equal(6, solid.Faces().Length);
        CompSolid compSolid = CompSolid.Create([box]);
        Assert.Single(compSolid.Solids());
        Assert.Throws<ArgumentException>(() => Compound.Create([default(Shape)]));
    }

    [Fact]
    public void A_shape_is_rebuilt_with_other_children_and_the_original_is_unchanged()
    {
        Shell shell = Box().Shells()[0];
        Face first = shell.Faces()[0];
        Shape fewer = shell.WithoutChild(first);
        Assert.Equal(5, ((Shell)fewer).Faces().Length);
        Assert.Equal(6, shell.Faces().Length);
        Shape again = fewer.WithChildren(shell.Faces().Select(f => (Shape)f));
        Assert.Equal(6, ((Shell)again).Faces().Length);
        Assert.Throws<ArgumentException>(() => shell.WithoutChild(Cylinder()));
    }

    [Fact]
    public void Moving_composes_the_placement_and_shares_the_data()
    {
        Solid box = Box();
        Shape moved = box.Moved(Transform3D.Translation(new Vector3D(10, 0, 0)));
        Assert.True(moved.IsPartner(box));
        Assert.False(moved.IsSame(box));
        Assert.Equal(new Vector3D(10, 0, 0), moved.Placement.Transformation.TranslationPart);
        Assert.Throws<ArgumentException>(() => box.Moved(Transform3D.Scale(Point3D.Origin, 2)));
    }

    [Fact]
    public void Placements_compare_by_value_in_public()
    {
        var a = new Placement(Transform3D.Translation(new Vector3D(1, 0, 0)));
        var b = new Placement(Transform3D.Translation(new Vector3D(1, 0, 0)));
        Assert.Equal(a, b);
        Assert.True(a == b);
        Assert.Equal(a.GetHashCode(), b.GetHashCode());
        Assert.NotEqual(a, new Placement());
        Assert.True(new Placement().IsIdentity);
    }

    [Fact]
    public void Vertices_and_edges_read_their_geometry()
    {
        Solid box = Box();
        Vertex vertex = box.Vertices()[0];
        Assert.True(BoundingBox.Of(Point3D.Origin, new Point3D(1, 2, 3)).Contains(vertex.Point));
        Assert.True(vertex.Tolerance > 0);

        Edge edge = box.Edges()[0];
        Assert.True(edge.TryGetCurve(out Curve3D? curve, out Interval range));
        Assert.Equal(edge.Range, range);
        Assert.True(curve!.Evaluate(range.Min).IsEqual(edge.FirstVertex.Point, 1e-7)
                    || curve.Evaluate(range.Min).IsEqual(edge.LastVertex.Point, 1e-7));
        Assert.False(edge.IsDegenerated);
        Assert.True(edge.Tolerance > 0);
        Assert.Equal(range.Min, edge.ParameterOf(edge.Orientation == Orientation.Forward ? edge.FirstVertex : edge.LastVertex), 9);
        Assert.Throws<InvalidOperationException>(() => default(Edge).Range);
    }

    [Fact]
    public void Faces_read_their_surface_boundary_and_curves_on_them()
    {
        Solid cylinder = Cylinder();
        Face side = cylinder.Faces().First(f => f.ToSurface().Kind == SurfaceKind.Cylinder);
        Assert.True(side.TryGetSurface(out Surface? surface));
        Assert.Equal(SurfaceKind.Cylinder, surface!.Kind);
        Assert.False(side.OuterWire.IsNull);
        (Interval u, Interval v) = side.UVBounds;
        Assert.Equal(2 * Math.PI, u.Length, 9);
        Assert.Equal(2, v.Length, 9);
        Edge seam = side.Edges().First(e => e.IsSeam(side));
        Assert.True(seam.TryGetCurveOnFace(side, out Curve2D? pcurve, out _));
        Assert.Equal(CurveKind.Line, pcurve!.Kind);
        Assert.Equal(4, side.OuterWire.OrderedEdges().Length);
    }

    [Fact]
    public void Evaluators_answer_in_world_coordinates()
    {
        Solid cylinder = Cylinder();
        Face side = cylinder.Faces().First(f => f.ToSurface().Kind == SurfaceKind.Cylinder);
        FaceSurface surface = side.ToSurface();
        Assert.True(surface.TryGetCylinder(out Kerf.Geometry.Cylinder analytic));
        Assert.Equal(1, analytic.Radius);
        Assert.False(surface.TryGetPlane(out _));
        Direction3D normal = surface.Normal(0, 1);
        Assert.True(normal.IsEqual(Direction3D.UnitX, 1e-9));
        Assert.True(surface.Evaluate(0, 1).IsEqual(new Point3D(1, 0, 1), 1e-9));
        Assert.Equal(1, surface.Derivatives(0, 1, 1).Order);

        Edge circle = cylinder.Edges().First(e => e.ToCurve().Kind == CurveKind.Circle);
        EdgeCurve curve = circle.ToCurve();
        Assert.True(curve.TryGetCircle(out Circle3D c));
        Assert.Equal(1, c.Radius);
        Assert.True(curve.IsPeriodic);
        Assert.Equal(2 * Math.PI, curve.Range.Length, 9);
        Assert.Equal(1, curve.Tangent(0).Magnitude, 9);
        Assert.False(curve.TryGetLine(out _));
        Assert.Equal(circle, curve.Edge);

        Face top = Box().Faces().First(f => f.ToSurface().TryGetPlane(out Plane3D p) && p.Location.Z > 2.5);
        Assert.True(top.ToSurface().Normal(0, 0).IsEqual(Direction3D.UnitZ, 1e-9));
    }

    [Fact]
    public void A_graph_answers_ancestors()
    {
        Solid box = Box();
        var graph = ShapeGraph.Build(box);
        Edge edge = box.Edges()[0];
        Assert.Equal(2, graph.Ancestors(edge, ShapeType.Face).Length);
        Assert.Single(graph.Ancestors(edge, ShapeType.Solid));
        Assert.Equal(0, graph.IndexOf(box));
        Assert.Equal(-1, graph.IndexOf(Cylinder()));
        Assert.Equal(12, graph.All(ShapeType.Edge).Length);
        Assert.Throws<ArgumentException>(() => graph.Ancestors(edge, ShapeType.Vertex));
    }

    [Fact]
    public void A_history_reads_and_composes()
    {
        Solid box = Box();
        Face a = box.Faces()[0];
        Face b = box.Faces()[1];
        Face c = box.Faces()[2];
        ShapeHistory first = ShapeHistory.Create([new ShapeHistoryEntry(a, ShapeHistoryKind.Modified, [b])]);
        ShapeHistory second = ShapeHistory.Create([new ShapeHistoryEntry(b, ShapeHistoryKind.Modified, [c])]);
        Assert.Equal([b], first.Modified(a));
        Assert.True(first.HasModified);
        Assert.Empty(first.Generated(a));
        Assert.False(first.IsRemoved(a));
        ShapeHistoryEntry entry = Assert.Single(first.Entries);
        Assert.Equal(ShapeHistoryKind.Modified, entry.Kind);
        Assert.Equal([c], first.Then(second).Modified(a));
        Assert.Equal([c], ShapeHistory.Compose(first, second).Modified(a));
        Assert.Empty(ShapeHistory.Empty.Entries);
        ShapeHistory removed = ShapeHistory.Create([new ShapeHistoryEntry(a, ShapeHistoryKind.Removed, [])]);
        Assert.True(removed.IsRemoved(a));
        Assert.True(removed.HasRemoved);
    }

    [Fact]
    public void A_built_shape_converts_to_its_shape()
    {
        Solid box = Box();
        var built = new Kerf.Built<Solid>(box, ShapeHistory.Empty);
        Solid shape = built;
        Assert.Equal(box, shape);
        Assert.Same(ShapeHistory.Empty, built.History);
    }
}
