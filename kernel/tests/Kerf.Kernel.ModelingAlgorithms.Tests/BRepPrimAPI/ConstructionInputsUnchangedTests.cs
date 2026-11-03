// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// No copy at entry in construction (Kerf unit 10H3, phase10-plan.md 2.4): after each kind of construction on frozen
/// inputs, the inputs are untouched, TShape for TShape and value for value (<see cref="ShapeSnapshot"/>), and the result
/// holds the inputs' own sub-shapes wherever OCCT's algorithm leaves them unchanged: the profile face of an extrusion
/// (decisions 468 and 1102), the edges of a face's wire, the sections of a ruled loft.
/// </summary>
public class ConstructionInputsUnchangedTests
{
    private static Point3D P(double x, double y, double z) => new(x, y, z);

    private static Wire Polygon(bool close, params Point3D[] points)
    {
        MakePolygon maker = new();
        foreach (Point3D point in points)
        {
            maker.Add(point);
        }
        if (close)
        {
            maker.Close();
        }
        return maker.Wire();
    }

    private static Wire Square(double z, double size = 10) =>
        Polygon(true, P(0, 0, z), P(size, 0, z), P(size, size, z), P(0, size, z));

    private static Wire Circle(double z, double r) =>
        new MakeWire(new MakeEdge(new Circle3D(new Frame3D(P(0, 0, z), new Direction3D(0, 0, 1)), r)).Edge()).Wire();

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

    private static void AssertHoldsAll(in Shape result, in Shape input, ShapeType type)
    {
        foreach (Shape sub in SubShapes(input, type))
        {
            Assert.True(Holds(result, sub), $"the result doesn't hold the input's {type}");
        }
    }

    // Runs the construction on frozen inputs, checks them untouched, and returns the result
    private static Shape Untouched(Func<Shape> construction, params Shape[] inputs)
    {
        foreach (Shape input in inputs)
        {
            Assert.True(input.TShape()!.IsFrozen);
        }
        ShapeSnapshot[] snapshots = [.. inputs.Select(s => ShapeSnapshot.Of(s))];
        Shape result = construction();
        for (int i = 0; i < inputs.Length; i++)
        {
            snapshots[i].AssertUnchanged(inputs[i]);
        }
        Assert.False(result.IsNull);
        return result;
    }

    // ---------------------------------------------------------------------------------------------------- sweeps

    [Fact]
    public void A_prism_of_a_polygon_face_shares_the_face_its_edges_and_vertices()
    {
        Face face = new MakeFace(Square(0)).Face();
        ApiMakePrism prism = null!;
        Shape result = Untouched(() => (prism = new ApiMakePrism(face, new Vector3D(0, 0, 5))).Shape(), face);
        Assert.True(Holds(result, face));
        AssertHoldsAll(result, face, ShapeType.Edge);
        AssertHoldsAll(result, face, ShapeType.Vertex);
        Assert.Same(face.TShape(), prism.FirstShape().TShape());
        foreach (Shape edge in SubShapes(face, ShapeType.Edge))
        {
            Assert.Single(prism.Generated(edge));
            Assert.Same(edge.TShape(), prism.FirstShape(edge).TShape());
        }
    }

    [Fact]
    public void A_prism_of_a_circle_face_shares_its_vertex_and_answers_for_the_caller_s_edge()
    {
        // the cylinder's pcurve is written into the circle: the edge and the face are new, the vertex is the caller's
        Face face = new MakeFace(Circle(0, 3)).Face();
        ApiMakePrism prism = null!;
        Shape result = Untouched(() => (prism = new ApiMakePrism(face, new Vector3D(0, 0, 5))).Shape(), face);
        AssertHoldsAll(result, face, ShapeType.Vertex);
        Edge circle = ShapeCasts.Edge(SubShapes(face, ShapeType.Edge)[0]);
        Assert.False(Holds(result, circle));
        Shape bottom = prism.FirstShape(circle);
        Assert.True(Holds(result, bottom));
        Assert.True(Holds(prism.FirstShape(), bottom));
        Assert.Single(prism.Generated(circle));
        Assert.True(Holds(result, prism.Generated(circle)[0]));
    }

    [Theory]
    [InlineData(Math.PI)]
    [InlineData(2 * Math.PI)]
    public void A_revolution_of_a_polygon_face_shares_what_it_leaves_unchanged(double angle)
    {
        Face face = new MakeFace(Polygon(true, P(5, 0, 0), P(10, 0, 0), P(10, 0, 4), P(5, 0, 4))).Face();
        ApiMakeRevol revol = null!;
        Shape result = Untouched(() => (revol = new ApiMakeRevol(face, new Axis3D(P(0, 0, 0), new Direction3D(0, 0, 1)), angle)).Shape(), face);
        // the new faces' pcurves go into the face's edges, its vertices stay as they are
        AssertHoldsAll(result, face, ShapeType.Vertex);
        foreach (Shape edge in SubShapes(face, ShapeType.Edge))
        {
            // (a whole turn deletes the edges at right angles to the axis, as OCCT's history says)
            foreach (Shape generated in revol.Generated(edge))
            {
                Assert.True(Holds(result, generated));
            }
            if (angle < 2 * Math.PI)
            {
                Assert.NotEmpty(revol.Generated(edge));
                Assert.True(Holds(result, revol.FirstShape(edge)));
            }
        }
    }

    [Fact]
    public void A_pipe_of_a_face_leaves_the_profile_and_the_spine_untouched()
    {
        Face face = new MakeFace(Square(0, 2)).Face();
        Wire spine = Polygon(false, P(1, 1, 0), P(1, 1, 10), P(1, 11, 20));
        ApiMakePipe pipe = null!;
        Shape result = Untouched(() => (pipe = new ApiMakePipe(spine, face)).Shape(), face, spine);
        Assert.True(Holds(result, pipe.FirstShape()));
        foreach (Shape edge in SubShapes(face, ShapeType.Edge))
        {
            Assert.NotEmpty(pipe.Generated(edge));
        }
    }

    // ---------------------------------------------------------------------------------------------------- lofts

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void A_loft_leaves_its_sections_untouched(bool isRuled)
    {
        Wire s1 = Circle(0, 5), s2 = Circle(10, 8), s3 = Circle(20, 4);
        ThruSections loft = new(true, isRuled);
        Shape result = Untouched(() =>
        {
            loft.AddWire(s1);
            loft.AddWire(s2);
            loft.AddWire(s3);
            loft.Build();
            return loft.Shape();
        }, s1, s2, s3);
        foreach (Wire section in new[] { s1, s2, s3 })
        {
            foreach (Shape edge in SubShapes(section, ShapeType.Edge))
            {
                Assert.NotEmpty(loft.Generated(edge));
            }
        }
        AssertHoldsAll(result, loft.FirstShape(), ShapeType.Face);
    }

    [Fact]
    public void A_loft_between_points_leaves_them_and_its_sections_untouched()
    {
        // (bugs/modalg_6/bug26150_5: the point section handed out by Wires() holds the caller's vertex)
        Vertex v1 = new MakeVertex(P(5, 5, -30)).Vertex(), v2 = new MakeVertex(P(5, 5, 40)).Vertex();
        Wire s1 = Square(0), s2 = Circle(10, 8);
        ThruSections loft = new(true, true);
        Untouched(() =>
        {
            loft.AddVertex(v1);
            loft.AddWire(s1);
            loft.AddWire(s2);
            loft.AddVertex(v2);
            loft.Build();
            return loft.Shape();
        }, v1, v2, s1, s2);
        Assert.NotEmpty(loft.Generated(v1));
        Assert.Contains(loft.Wires(), w => Holds(w, v1));
    }

    [Fact]
    public void The_generator_leaves_its_sections_untouched()
    {
        Wire s1 = Square(0), s2 = Square(5);
        Generator generator = new();
        Shape result = Untouched(() =>
        {
            generator.AddWire(s1);
            generator.AddWire(s2);
            generator.Perform();
            return generator.Shell();
        }, s1, s2);
        // the sides' pcurves go into the sections' edges: the shell holds what stands for them
        foreach (Shape edge in SubShapes(s1, ShapeType.Edge))
        {
            Assert.True(Holds(result, generator.ResultShape(edge)));
        }
    }

    [Fact]
    public void Ruled_faces_and_shells_leave_their_edges_and_wires_untouched()
    {
        Edge e1 = new MakeEdge(P(0, 0, 0), P(10, 0, 0)).Edge();
        Edge e2 = new MakeEdge(P(0, 5, 3), P(10, 5, 3)).Edge();
        Shape face = Untouched(() => Fill.Face(e1, e2), e1, e2);
        Assert.Single(SubShapes(face, ShapeType.Face));

        Wire w1 = Polygon(false, P(0, 0, 0), P(5, 0, 0), P(10, 0, 0));
        Wire w2 = Polygon(false, P(0, 5, 3), P(5, 5, 3), P(10, 5, 3));
        Shape shell = Untouched(() => Fill.Shell(w1, w2), w1, w2);
        Assert.Equal(2, SubShapes(shell, ShapeType.Face).Count);
    }

    // ------------------------------------------------------------------------------------- evolved and projections

    [Fact]
    public void Evolved_shapes_leave_the_spine_and_the_profile_untouched()
    {
        Wire spine = Square(0), profile = Polygon(false, P(0, 0, 0), P(0, 0, 2));
        Untouched(() => new MakeEvolved(spine, profile).Shape(), spine, profile);
        Wire volumeProfile = Polygon(true, P(0, 0, 0), P(1, 0, 0), P(1, 0, 1), P(0, 0, 1));
        Untouched(() => new MakeEvolved(spine, volumeProfile, isVolume: true).Shape(), spine, volumeProfile);
    }

    [Fact]
    public void A_projection_leaves_the_wire_and_the_target_untouched()
    {
        Wire wire = Polygon(true, P(-3, -3, 40), P(3, -3, 40), P(3, 3, 40), P(-3, 3, 40));
        Shape cylinder = new ApiMakeCylinder(20, 30).Shape();
        Untouched(() => new Projection(wire, cylinder, new Direction3D(0, 0, 1)).Shape(), wire, cylinder);
        // a compound target is gathered into a new compound: its frozen faces are still sectioned without writes
        BrepBuilder b = new();
        Compound compound = new();
        b.MakeCompound(ref compound);
        foreach (Shape f in SubShapes(cylinder, ShapeType.Face))
        {
            b.Add(compound, f);
        }
        Untouched(() => new Projection(wire, compound, new Direction3D(0, 0, 1)).Shape(), cylinder);
    }

    [Fact]
    public void A_half_space_shares_the_caller_s_face()
    {
        Face face = new MakeFace(Square(0)).Face();
        Shape result = Untouched(() => new MakeHalfSpace(face, P(5, 5, 5)).Solid(), face);
        Assert.True(Holds(result, face));
    }

    // ---------------------------------------------------------------------------------------------------- makers

    [Fact]
    public void A_face_of_a_frozen_wire_shares_its_edges_and_vertices()
    {
        Wire wire = Square(0);
        Shape face = Untouched(() => new MakeFace(wire).Face(), wire);
        AssertHoldsAll(face, wire, ShapeType.Edge);
        AssertHoldsAll(face, wire, ShapeType.Vertex);
        Wire circle = Circle(0, 4);
        Shape disc = Untouched(() => new MakeFace(circle).Face(), circle);
        AssertHoldsAll(disc, circle, ShapeType.Edge);
    }

    [Fact]
    public void A_wire_of_frozen_edges_shares_them_and_merges_on_new_vertices()
    {
        Edge e1 = new MakeEdge(P(0, 0, 0), P(10, 0, 0)).Edge();
        Edge e2 = new MakeEdge(Exploration.LastVertex(e1), new MakeVertex(P(10, 10, 0)).Vertex()).Edge();
        Shape wire = Untouched(() => new MakeWire(e1, e2).Wire(), e1, e2);
        Assert.True(Holds(wire, e1) && Holds(wire, e2));

        // an edge whose vertex is near the wire's but not the same: OCCT moves and enlarges the wire's vertex
        Edge e3 = new MakeEdge(P(10, 10.00000005, 0), P(0, 10, 0)).Edge();
        MakeWire maker = new(e1, e2);
        Shape first = maker.Wire();
        ShapeSnapshot snapshotFirst = ShapeSnapshot.Of(first);
        Shape merged = Untouched(() =>
        {
            maker.Add(e3);
            return maker.Wire();
        }, e1, e2, e3);
        snapshotFirst.AssertUnchanged(first);
        Assert.Equal(3, SubShapes(merged, ShapeType.Edge).Count);
        Assert.True(Holds(merged, e1));
    }

    [Fact]
    public void Adding_to_a_published_result_leaves_it_as_it_was()
    {
        MakePolygon polygon = new();
        polygon.Add(P(0, 0, 0));
        polygon.Add(P(10, 0, 0));
        polygon.Add(P(10, 10, 0));
        Wire open = polygon.Wire();
        ShapeSnapshot snapshotOpen = ShapeSnapshot.Of(open);
        polygon.Add(P(0, 10, 0));
        polygon.Close();
        Wire closed = polygon.Wire();
        snapshotOpen.AssertUnchanged(open);
        Assert.Equal(4, SubShapes(closed, ShapeType.Edge).Count);
        AssertHoldsAll(closed, open, ShapeType.Edge);

        MakeFace faceMaker = new(new MakeFace(closed).Face());
        Face face = faceMaker.Face();
        ShapeSnapshot snapshotFace = ShapeSnapshot.Of(face);
        Wire hole = Polygon(true, P(2, 2, 0), P(2, 4, 0), P(4, 4, 0), P(4, 2, 0));
        faceMaker.Add(hole);
        Face withHole = faceMaker.Face();
        snapshotFace.AssertUnchanged(face);
        Assert.Equal(2, SubShapes(withHole, ShapeType.Wire).Count);
        Assert.True(Holds(withHole, hole));
    }

    // ---------------------------------------------------------------------------------------------------- BRepLib

    [Fact]
    public void BRepLib_s_twins_leave_their_input_untouched_and_share_what_they_leave()
    {
        Shape box = new MakeBox(10, 10, 10).Shape();
        // UpdateTolerances raises every vertex by its rounding guard: they are all new
        Shape updated = Untouched(() => BrepLibrary.TolerancesUpdated(box, out _), box);
        Untouched(() => BrepLibrary.SameParametered(box, out _), box);
        // EncodeRegularity writes the edges' continuities: the vertices are the box's
        List<Shape> edges = SubShapes(box, ShapeType.Edge);
        Shape encoded = Untouched(() => BrepLibrary.RegularityEncoded(box, edges.Take(2), out _), box);
        Assert.True(encoded.TShape()!.IsFrozen && updated.TShape()!.IsFrozen);
        AssertHoldsAll(encoded, box, ShapeType.Vertex);
        // a functional twin that changes nothing gives the input back
        Shape face = new MakeFace(Square(0)).Face();
        Assert.Same(face.TShape(), BrepLibrary.Curves3dBuilt(face, out bool isDone).TShape());
        Assert.True(isDone);
    }
}
