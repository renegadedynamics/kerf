// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Meshing;
using Kerf.Offsets;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// Unit 11J: the kernel's shapes and evaluators by name, where <see cref="ShapeLawsTests"/> reaches them by reflection:
/// identity under orientation and placement, what each evaluator says of the curve or surface it holds, meshes and offsets.
/// </summary>
public class TopologyDirectTests
{
    private static readonly Placement Away = new(Transform3D.Translation(new Vector3D(20, 0, 0)));

    private static Solid Box() => Solid.Box(2, 2, 2).Value;

    private static T Same<T>(T value) => value;

    // ---- Identity: partner ignores orientation and place, same ignores orientation, equal ignores neither ----

    [Fact]
    public void A_vertex_s_and_an_edge_s_identity_under_orientation_and_placement()
    {
        Vertex v = Box().AllVertices()[0];
        Assert.True(v.IsEqual(v));
        Assert.True(v.IsPartner((Vertex)v.Moved(Away)));
        Assert.False(v.IsEqual((Vertex)v.Moved(Away)));
        Assert.True(v == Same(v));
        Assert.False(v != Same(v));
        Assert.True(v.Equals(v));
        Assert.Equal(v.Orientation == Orientation.Forward ? Orientation.Reversed : Orientation.Forward, v.Composed(Orientation.Reversed).Orientation);
        Assert.NotEqual(v.Orientation, v.Complemented().Orientation);
        Assert.True(((Vertex)v.Moved(Away)).Placement == Away);
        Assert.Equal(ShapeType.Vertex, v.Type);

        Edge e = Box().AllEdges()[0];
        Assert.True(e == Same(e));
        Assert.False(e != Same(e));
        Assert.True(e.Equals(e));
        Assert.NotEqual(e.Orientation, e.Complemented().Orientation);
        Assert.True(e.Complemented().IsSame(e));
        Assert.False(e.Complemented().IsEqual(e));
    }

    [Fact]
    public void Wires_shells_solids_and_faces_keep_their_identity_laws()
    {
        Solid s = Box();
        Shell shell = s.AllShells()[0];
        Face face = s.AllFaces()[0];
        Wire wire = face.AllWires()[0];

        Assert.NotEqual(wire.Orientation, wire.Complemented().Orientation);
        Assert.Equal(wire.Orientation == Orientation.Forward ? Orientation.Reversed : Orientation.Forward, wire.Composed(Orientation.Reversed).Orientation);
        Assert.True(wire.Equals(wire));
        Assert.True(wire.IsEqual(wire));
        Assert.True(wire.IsPartner(wire.Located(Away)));
        Assert.Equal(Orientation.Internal, wire.Oriented(Orientation.Internal).Orientation);
        Assert.True(wire.Located(Away).Placement == Away);
        Assert.False(wire != Same(wire));

        Assert.NotEqual(shell.Orientation, shell.Complemented().Orientation);
        Assert.Equal(shell.Orientation == Orientation.Forward ? Orientation.Reversed : Orientation.Forward, shell.Composed(Orientation.Reversed).Orientation);
        Assert.True(shell.Equals(shell));
        Assert.True(shell.IsEqual(shell));
        Assert.True(shell.IsPartner(shell.Located(Away)));
        Assert.Equal(Orientation.Internal, shell.Oriented(Orientation.Internal).Orientation);
        Assert.True(shell.Located(Away).Placement == Away);
        Assert.False(shell != Same(shell));

        Assert.NotEqual(s.Orientation, s.Complemented().Orientation);
        Assert.Equal(s.Orientation == Orientation.Forward ? Orientation.Reversed : Orientation.Forward, s.Composed(Orientation.Reversed).Orientation);
        Assert.True(s.Equals(s));
        Assert.True(s.IsPartner(s.Located(Away)));
        Assert.True(s.Located(Away).Placement == Away);
        Assert.Equal(8, s.Located(Away).IntegrateVolume(), 7);
        Assert.False(s != Same(s));

        Assert.NotEqual(face.Orientation, face.Complemented().Orientation);
        Assert.Equal(face.Orientation == Orientation.Forward ? Orientation.Reversed : Orientation.Forward, face.Composed(Orientation.Reversed).Orientation);
        Assert.True(face.IsPartner((Face)face.Moved(Away)));
        Assert.False(face != Same(face));
    }

    [Fact]
    public void Compounds_and_compsolids_keep_their_identity_laws()
    {
        Compound c = Compound.Create([Box()]);
        Assert.NotEqual(c.Orientation, c.Complemented().Orientation);
        Assert.Equal(c.Orientation == Orientation.Forward ? Orientation.Reversed : Orientation.Forward, c.Composed(Orientation.Reversed).Orientation);
        Assert.True(c.Equals(c));
        Assert.True(c.IsEqual(c));
        Assert.True(c.IsSame(c.Reversed()));
        Assert.False(c.IsEqual(c.Reversed()));
        Assert.True(c.IsPartner(c.Located(Away)));
        Assert.Equal(Orientation.Internal, c.Oriented(Orientation.Internal).Orientation);
        Assert.True(c.Located(Away).Placement == Away);
        Assert.False(c != Same(c));

        Solid a = Box();
        CompSolid cs = CompSolid.Create([a, (Solid)a.Located(Away)]);
        Assert.NotEqual(cs.Orientation, cs.Complemented().Orientation);
        Assert.Equal(cs.Orientation == Orientation.Forward ? Orientation.Reversed : Orientation.Forward, cs.Composed(Orientation.Reversed).Orientation);
        Assert.True(cs.Equals(cs));
        Assert.True(cs.IsEqual(cs));
        Assert.True(cs.IsSame(cs.Reversed()));
        Assert.True(cs.IsPartner(cs.Located(Away)));
        Assert.Equal(Orientation.Internal, cs.Oriented(Orientation.Internal).Orientation);
        Assert.True(cs.Located(Away).Placement == Away);
        Assert.Equal(16, cs.Moved(Away).IntegrateVolume(), 7);
        Assert.False(cs != Same(cs));
    }

    [Fact]
    public void Placements_compose_and_divide_back()
    {
        var a = new Placement(Transform3D.Translation(new Vector3D(1, 2, 3)));
        var b = new Placement(Transform3D.Rotation(Axis3D.ZAxis, Math.PI / 2));
        Placement both = a.Multiplied(b);
        Point3D moved = new Point3D(1, 0, 0).Transformed(both.Transformation);
        Assert.Equal(1, moved.X, 9);
        Assert.Equal(3, moved.Y, 9);
        Assert.Equal(3, moved.Z, 9);
        Assert.True(both.Divided(b) == a);
    }

    [Fact]
    public void Exploring_a_box_counts_its_kinds()
    {
        Solid box = Box();
        Shape shape = box;
        Assert.Equal(6, shape.Explore<Face>().Count());
        // each edge is met from both its faces
        Assert.Equal(24, shape.Explore<Edge>().Count());
        // what sits under an avoided kind isn't reached
        Assert.Empty(shape.Explore<Vertex>(ShapeType.Edge));
        Assert.Throws<ArgumentException>(() => shape.Explore<Shape>());
        Edge edge = box.AllEdges()[0];
        Assert.Equal(2, edge.Explore<Vertex>().Count());
        Assert.Empty(edge.Explore<Face>());
    }

    // ---- What evaluators say of what they hold ----

    [Fact]
    public void An_edge_s_curve_says_what_it_is_and_only_that()
    {
        var ellipse = new Ellipse3D(Frame3D.World, 3, 2);
        EdgeCurve onEllipse = Edge.FromCurve(new EllipseCurve3D(ellipse)).Value.ToCurve();
        Assert.True(onEllipse.TryGetEllipse(out Ellipse3D gotEllipse));
        Assert.Equal(3, gotEllipse.MajorRadius, 12);
        Assert.False(onEllipse.TryGetHyperbola(out _));
        Assert.False(onEllipse.TryGetParabola(out _));
        Assert.False(onEllipse.TryGetBezier(out _));
        Assert.False(onEllipse.TryGetBSpline(out _));

        var circle = Edge.Circle(new Circle3D(Frame3D.World, 1)).Value.ToCurve();
        Assert.Equal(2 * Math.PI, circle.Period, 12);
        var line = Edge.Line(Point3D.Origin, new Point3D(1, 0, 0)).Value.ToCurve();
        Assert.Throws<InvalidOperationException>(() => line.Period);
    }

    [Fact]
    public void A_curve_s_length_between_parameters_runs_either_way()
    {
        var line = Edge.Line(Point3D.Origin, new Point3D(4, 0, 0)).Value.ToCurve();
        Assert.Equal(4, line.LengthBetween(line.Range.Min, line.Range.Max), 9);
        // a length has no sign, whichever parameter comes first
        Assert.Equal(4, line.LengthBetween(line.Range.Max, line.Range.Min), 9);
    }

    [Fact]
    public void A_face_s_surface_says_what_it_is_and_its_curvature()
    {
        Solid sphere = Solid.Sphere(2).Value;
        var onSphere = sphere.AllFaces()[0].ToSurface();
        Assert.Equal(sphere.AllFaces()[0], onSphere.Face);
        Assert.True(onSphere.TryGetSphere(out Sphere3D gotSphere));
        Assert.Equal(2, gotSphere.Radius, 12);
        Assert.False(onSphere.TryGetCone(out _));
        Assert.False(onSphere.TryGetTorus(out _));
        Assert.True(onSphere.TryGetCurvature(1, 0.5, 1e-7, out SurfaceCurvature curvature));
        Assert.Equal(0.5, Math.Abs(curvature.Min), 7);
        Assert.Equal(0.5, Math.Abs(curvature.Max), 7);
        Assert.Equal(1, curvature.Normal.ToVector().Magnitude, 9);

        Solid cone = Solid.Cone(2, 1, 3).Value;
        Face side = cone.AllFaces().First(f => f.ToSurface().TryGetCone(out _));
        Assert.True(side.ToSurface().TryGetCone(out Cone3D gotCone));
        Assert.Equal(2, gotCone.RefRadius, 9);

        Solid torus = Solid.Torus(5, 1).Value;
        Assert.True(torus.AllFaces()[0].ToSurface().TryGetTorus(out Torus3D gotTorus));
        Assert.Equal(5, gotTorus.MajorRadius, 12);
        Assert.Equal(1, gotTorus.MinorRadius, 12);

        // a cylinder bends one way only, by one over its radius
        Solid cylinder = Solid.Cylinder(2, 3).Value;
        Face round = cylinder.AllFaces().First(f => f.ToSurface().TryGetCylinder(out _));
        Assert.True(round.ToSurface().TryGetCurvature(1, 1, 1e-7, out SurfaceCurvature bent));
        Assert.Equal(0, Math.Min(Math.Abs(bent.Min), Math.Abs(bent.Max)), 7);
        Assert.Equal(0.5, Math.Max(Math.Abs(bent.Min), Math.Abs(bent.Max)), 7);
    }

    // ---- Surfaces bounded by uneven curves ----

    [Fact]
    public void A_prism_on_an_uneven_ellipse_measures_by_its_formulas()
    {
        Wire outline = Wire.FromEdges([Edge.FromCurve(new EllipseCurve3D(new Ellipse3D(Frame3D.World, 10, 1))).Value]).Value;
        Face floor = Face.FromWire(outline).Value;
        Assert.Equal(Math.PI * 10, floor.IntegrateArea(), 5);
        // the true perimeter, by a fine polygon, is the edge's length
        const int pieces = 200_000;
        double perimeter = 0;
        for (int i = 0; i < pieces; i++)
        {
            double t = (i + 0.5) / pieces * 2 * Math.PI;
            perimeter += Math.Sqrt(100 * Math.Sin(t) * Math.Sin(t) + Math.Cos(t) * Math.Cos(t)) * 2 * Math.PI / pieces;
        }
        Assert.Equal(perimeter, outline.IntegrateLength(), 6);
    }

    // ---- Meshes ----

    [Fact]
    public void Mesh_options_default_as_documented_and_a_mesh_s_arrays_agree()
    {
        MeshOptions d = MeshOptions.Default;
        Assert.False(d.Relative);
        Assert.True(d.Parallel);
        Assert.True(d.InternalVertices);
        Assert.Null(d.EdgeDeflection);
        Assert.True(d.LinearDeflection > 0);
        Assert.True(d.AngularDeflection > 0);
        Assert.True(d.MinSize >= 0);
        Assert.Equal(0.01, new MeshOptions { EdgeDeflection = 0.01 }.EdgeDeflection);

        ShapeMesh mesh = Box().Mesh().Value;
        Assert.Equal(6, mesh.Faces.Count);
        int nodes = 0;
        foreach (FaceMesh face in mesh.Faces)
        {
            // each face's slices are of its own nodes, on its plane, its normals unit and square to it
            Assert.Equal(3 * face.NodeCount, face.Positions.Length);
            Assert.Equal(3 * face.NodeCount, face.Normals.Length);
            nodes += face.NodeCount;
            ReadOnlySpan<float> normals = face.Normals.Span;
            ReadOnlySpan<float> positions = face.Positions.Span;
            for (int i = 0; i < face.NodeCount; i++)
            {
                Assert.Equal(1, Math.Abs(normals[3 * i]) + Math.Abs(normals[3 * i + 1]) + Math.Abs(normals[3 * i + 2]), 5);
                Assert.Contains(positions[3 * i], new[] { 0f, 2f });
            }
        }
        Assert.Equal(mesh.NodeCount, nodes);
        Assert.Equal(0, mesh.EdgePoints.Length % 3);
        Assert.NotEqual(0, mesh.EdgePoints.Length);
        Assert.Equal(12, mesh.Edges.Count);
        foreach (EdgePolyline edge in mesh.Edges)
        {
            Assert.Equal(CurveKind.Line, edge.Kind);
            Assert.Equal(0, edge.Points.Length % 3);
            Assert.True(edge.Points.Length >= 6);
        }
    }

    // ---- Offsets ----

    [Fact]
    public void Offset_options_default_as_documented()
    {
        OffsetOptions d = OffsetOptions.Default;
        Assert.Equal(1e-4, d.Tolerance);
        Assert.Equal(OffsetMode.Skin, d.Mode);
        Assert.Equal(JoinKind.Arc, d.Join);
        Assert.False(d.Intersection);
        Assert.False(d.SelfIntersection);
        Assert.False(d.RemoveInternalEdges);

        Offset2DOptions flat = Offset2DOptions.Default;
        Assert.Equal(JoinKind.Arc, flat.Join);
        Assert.False(flat.Open);
        Assert.Equal("offset.failed", OffsetOperations.Failed.Value);
    }

    [Fact]
    public void An_offset_of_a_box_grows_it_by_its_formula_and_its_inward_one_shrinks_it()
    {
        Solid box = Solid.Box(4, 4, 4).Value;
        var sharp = new OffsetOptions { Join = JoinKind.Intersection };
        Assert.Equal(6 * 6 * 6, box.Offset(1, sharp).Value.Shape.IntegrateVolume(), 5);
        Assert.Equal(2 * 2 * 2, box.Offset(-1, sharp).Value.Shape.IntegrateVolume(), 5);
        // rounded corners: the box, slabs, quarter cylinders and an eighth of a sphere at each corner
        double rounded = 64 + 6 * 16 + 12 * 4 * Math.PI / 4 + 4.0 / 3 * Math.PI;
        Assert.Equal(rounded, box.Offset(1).Value.Shape.IntegrateVolume(), 3);
    }

    // ---- Fill ----

    [Fact]
    public void Fill_options_default_as_documented_and_a_boundary_keeps_its_values()
    {
        FillOptions d = FillOptions.Default;
        Assert.Empty(d.Points);
        Assert.Null(d.InitialSurface);
        Assert.Equal(3, d.Degree);
        Assert.Equal(15, d.PointsOnCurve);
        Assert.Equal(2, d.Iterations);
        Assert.False(d.Anisotropy);
        Assert.Equal(0.00001, d.Tolerance2D);
        Assert.Equal(0.0001, d.Tolerance3D);
        Assert.Equal(0.01, d.AngularTolerance);
        Assert.Equal(0.1, d.CurvatureTolerance);
        Assert.Equal(8, d.MaxDegree);
        Assert.Equal(9, d.MaxSegments);

        Edge edge = Edge.Line(Point3D.Origin, new Point3D(1, 0, 0)).Value;
        var boundary = new FillBoundary(edge, Continuity.C0);
        Assert.Equal(edge, boundary.Edge);
        Assert.Equal(Continuity.C0, boundary.Continuity);
        Assert.Null(boundary.Support);
        Assert.True(boundary.IsBoundary);
    }

    // ---- Projections ----

    [Fact]
    public void A_wire_projected_from_an_eye_lands_on_the_target_and_grows_by_similar_triangles()
    {
        Wire square = Wire.Polygon([new Point3D(-1, -1, 5), new Point3D(1, -1, 5), new Point3D(1, 1, 5), new Point3D(-1, 1, 5)], closed: true).Value;
        Face floor = Face.FromWire(Wire.Polygon([new Point3D(-20, -20, 0), new Point3D(20, -20, 0), new Point3D(20, 20, 0), new Point3D(-20, 20, 0)], closed: true).Value).Value;
        // from z = 10 through z = 5 to z = 0: twice the size
        var projected = ShapeProjections.Project(square, floor, new Point3D(0, 0, 10)).Value;
        Assert.NotEmpty(projected);
        Assert.Equal(4 * 4, projected.Sum(w => w.IntegrateLength()), 6);
    }
}
