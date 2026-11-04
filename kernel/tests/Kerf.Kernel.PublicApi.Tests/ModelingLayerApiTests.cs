// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Features;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Offsets;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// The members unit 10M1 added to the public API for the modelling layer, which no longer reaches the engine (decisions
/// 1565 onwards): each checked against what the OCCT call it stands for does.
/// </summary>
public class ModelingLayerApiTests
{
    private const double Tol = 1e-7;

    private static Wire Square(double size = 10, double z = 0) =>
        Wire.Polygon([new(0, 0, z), new(size, 0, z), new(size, size, z), new(0, size, z)], closed: true).Value;

    private static Face SquareFace(double size = 10) => Face.FromWires(Square(size), []).Value;

    private static Solid Box(double size = 10) => Solid.Box(size, size, size).Value.Shape;

    // ---------------------------------------------------------------------------------------------- foundation

    [Fact]
    public void Colours_are_found_by_name_and_named_by_the_nearest()
    {
        Assert.True(Color.TryFromName("red", out Color red));
        Assert.Equal(1.0, red.ToLinear().R, 6);
        Assert.Equal("RED", red.NearestName(), ignoreCase: true);
        Assert.False(Color.TryFromName("no such colour", out _));
    }

    [Fact]
    public void An_oriented_box_contains_a_box_inside_it()
    {
        OrientedBox big = Box().OrientedBox();
        OrientedBox small = Solid.Box(new Point3D(2, 2, 2), new Point3D(3, 3, 3)).Value.Shape.OrientedBox();
        Assert.True(big.Contains(small));
        Assert.False(small.Contains(big));
        Assert.Equal(Math.Sqrt(300), big.Diagonal, 6);
    }

    [Fact]
    public void An_engine_exception_is_a_kernel_failure_with_a_diagnostic()
    {
        Exception e = Record.Exception(() => new Direction3D(0, 0, 0))!;
        Assert.True(Diagnostic.IsKernelFailure(e));
        Diagnostic diagnostic = Diagnostic.FromException(e);
        Assert.True(diagnostic.IsError);
        Assert.StartsWith("engine.", diagnostic.Code.Value);
        Assert.False(Diagnostic.IsKernelFailure(new InvalidOperationException()));
    }

    [Fact]
    public void Transforms_give_their_quaternion_and_their_rotation()
    {
        Transform3D turn = Transform3D.Rotation(Axis3D.ZAxis, Math.PI / 2);
        (double x, double y, double z, double w) = turn.GetQuaternion();
        Assert.Equal((0.0, 0.0, Math.Round(Math.Sqrt(0.5), 12), Math.Round(Math.Sqrt(0.5), 12)), (Math.Round(x, 12), Math.Round(y, 12), Math.Round(z, 12), Math.Round(w, 12)));
        Transform3D moved = Transform3D.Translation(new Vector3D(1, 2, 3)) * turn;
        Transform3D rotation = moved.ToRotation();
        Assert.Equal(0, rotation.TranslationPart.Magnitude, 12);
        Assert.True((rotation * new Point3D(1, 0, 0)).DistanceTo(new Point3D(0, 1, 0)) < Tol);
        var stretch = GeneralTransform3D.Affinity(Axis3D.ZAxis, 2);
        Assert.True((stretch * new Point3D(1, 0, 5)).DistanceTo(new Point3D(2, 0, 5)) < Tol);
    }

    // ---------------------------------------------------------------------------------------------- topology

    [Fact]
    public void A_closed_shell_checks_closed_and_an_open_one_doesnt()
    {
        Assert.True(Shell.Create(Box().Faces()).CheckClosed());
        Assert.False(Shell.Create(Box().Faces().Take(5)).CheckClosed());
    }

    [Fact]
    public void Edges_give_their_curves_local_on_a_plane_and_as_a_wire()
    {
        Face face = SquareFace();
        Vertex corner = face.Vertices()[0];
        Point2D uv = corner.ParametersOn(face);
        Assert.True(face.ToSurface().Evaluate(uv.X, uv.Y).DistanceTo(corner.Point) < Tol);

        Edge edge = Edge.Line(new(0, 0, 0), new(4, 0, 0)).Value;
        Assert.True(edge.TryGetCurveOnPlane(new Plane3D(), out Curve2D? flat, out Interval range));
        Assert.True(flat.Evaluate(range.Max).DistanceTo(new Point2D(4, 0)) < Tol);
        Edge moved = (Edge)edge.Moved(new Placement(Transform3D.Translation(new Vector3D(0, 0, 5))));
        Assert.True(moved.TryGetLocalCurve(out Curve3D? local, out Interval localRange, out Placement placement));
        Assert.True(local.Evaluate(localRange.Max).DistanceTo(new Point3D(4, 0, 0)) < Tol);
        Assert.Equal(5, placement.Transformation.TranslationPart.Z, 12);

        WireCurve wire = Square().ToCurve();
        Assert.True(wire.IsClosed);
        Assert.Equal(40, wire.Length, 9);
        Assert.Equal(10, wire.LengthBetween(wire.Range.Min, wire.Range.Min + 1), 9);
        CurvePoint nearest = wire.Project(new Point3D(5, -3, 0))[0];
        Assert.Equal(3, nearest.Distance, 9);
        Assert.True(wire.Evaluate(wire.ParameterAtLength(15, wire.Range.Min)).DistanceTo(new Point3D(10, 5, 0)) < 1e-6);
        Assert.Equal(5, wire.Discretize(0.1).Length);   // the corners, the first again at the end
    }

    [Fact]
    public void An_edge_curve_measures_and_compares_its_continuity()
    {
        Edge arc = Edge.Arc(new Circle3D(Frame3D.World, 2), 0, Math.PI).Value;
        EdgeCurve curve = arc.ToCurve();
        Assert.Equal(2 * Math.PI, curve.Length, 9);
        Assert.Equal(Math.PI, curve.LengthBetween(0, Math.PI / 2), 9);
        Assert.Equal(Math.PI / 4, curve.ParameterAtLength(Math.PI / 2, 0), 9);
        Assert.Equal(curve.Derivatives(1, 2).D2.X, curve.DerivativeN(1, 2).X, 12);
        ImmutableArray<double> points = curve.Discretize(0.01);
        Assert.True(points.Length > 10);
        Assert.False(curve.TryGetOffsetCurve(out _));

        Edge next = Edge.Arc(new Circle3D(Frame3D.World, 2), Math.PI, 2 * Math.PI).Value;
        Assert.Equal(Continuity.C2, curve.ContinuityWith(next.ToCurve(), Math.PI, Math.PI));
    }

    [Fact]
    public void A_face_surface_gives_derivatives_and_curvature()
    {
        Face sphere = Solid.Sphere(2).Value.Shape.Faces()[0];
        FaceSurface surface = sphere.ToSurface();
        Assert.True(surface.TryGetCurvature(0.3, 0.2, 1e-9, out SurfaceCurvature curvature));
        Assert.Equal(0.25, curvature.Gaussian, 9);
        Assert.Equal(0.5, Math.Abs(curvature.Mean), 9);
        Assert.Equal(surface.Derivatives(0.3, 0.2, 1).DU.X, surface.DerivativeN(0.3, 0.2, 1, 0).X, 12);
    }

    // ---------------------------------------------------------------------------------------------- geometry

    [Fact]
    public void Curves_measure_lengths_project_and_test_planarity()
    {
        var line = new LineCurve3D(Point3D.Origin, Direction3D.UnitX);
        Assert.Equal(3, line.LengthBetween(1, 4), 12);
        Assert.Equal(6, line.ParameterAtLength(5, 1), 12);
        var circle = new CircleCurve3D(new Circle3D(Frame3D.World, 1));
        Assert.True(circle.IsPlanar());
        Assert.True(circle.IsPlanar(Direction3D.UnitZ));
        Assert.False(circle.IsPlanar(Direction3D.UnitX));

        var flat = new CircleCurve2D(new Axis2D(new Point2D(0, 0), Direction2D.UnitX), 1);
        ImmutableArray<CurvePoint2D> onCircle = flat.Project(new Point2D(3, 0));
        Assert.Equal(2, onCircle[0].Distance, 12);
        Assert.Equal(4, onCircle[^1].Distance, 12);
    }

    [Fact]
    public void Fitting_takes_periodic_parameters_free_tangents_and_smoothing()
    {
        Point3D[] points = [new(1, 0, 0), new(0, 1, 0), new(-1, 0, 0), new(0, -1, 0)];
        BSplineCurve3D periodic = BSplineCurve3D.Interpolate(points, new InterpolationOptions
        {
            Periodic = true,
            Parameters = [0, 1, 2, 3, 4],
        }).Value;
        Assert.True(periodic.IsPeriodic);

        Point3D[] row = [new(0, 0, 0), new(1, 1, 0), new(2, 0, 0)];
        BSplineCurve3D free = BSplineCurve3D.Interpolate(row, new InterpolationOptions
        {
            Tangents = [new Vector3D(1, 0, 0), new Vector3D(0, 0, 0), new Vector3D(1, 0, 0)],
        }).Value;
        Assert.True(free.Evaluate(free.LastParameter).DistanceTo(row[2]) < Tol);

        Point3D[] many = [.. Enumerable.Range(0, 20).Select(i => new Point3D(i, Math.Sin(i / 3.0), 0))];
        BSplineCurve3D smooth = BSplineCurve3D.Approximate(many, new ApproximationOptions { Smoothing = (1, 0, 0) }).Value;
        Assert.True(smooth.Evaluate(smooth.FirstParameter).DistanceTo(many[0]) < 0.5);

        BSplineCurve3D forced = new TrimmedCurve3D(new LineCurve3D(Point3D.Origin, Direction3D.UnitX), 0, 1)
            .ToBSpline(new ConversionOptions { ForceApproximation = true }).Value;
        Assert.True(forced.Evaluate(forced.LastParameter).DistanceTo(new Point3D(1, 0, 0)) < 1e-4);
    }

    [Fact]
    public void A_figure_eight_crosses_itself_and_frames_follow_a_curve()
    {
        Point3D[] loop = [new(0, 0, 0), new(1, 1, 0), new(2, 0, 0), new(1, -1, 0), new(0, 0.2, 0), new(-1, 1, 0), new(-2, 0, 0), new(-1, -1, 0), new(0, 0.4, 0)];
        Curve2D eight = BSplineCurve3D.Interpolate(loop).Value.ToCurve2D(new Plane3D()).Value;
        Assert.NotEmpty(eight.SelfIntersections().Value);

        EdgeCurve helixLike = Edge.Arc(new Circle3D(Frame3D.World, 1), 0, Math.PI).Value.ToCurve();
        ImmutableArray<CurveFrame> frames = helixLike.Frames([0.1, 1.0, 2.0], FrameLaw.CorrectedFrenet);
        Assert.Equal(3, frames.Length);
        foreach (CurveFrame frame in frames)
        {
            Assert.Equal(0, frame.Tangent.Dot(frame.Normal), 9);
            Assert.Equal(1, frame.Binormal.Magnitude, 9);
        }

        Assert.Equal(2, Square().ToCurve().Frames([0.5, 1.5]).Length);
    }

    [Fact]
    public void Tangents2D_solves_circles_centred_on_a_locus_and_lines_at_an_angle()
    {
        var circle = new CircleCurve2D(new Axis2D(new Point2D(0, 0), Direction2D.UnitX), 1);
        var axis = new LineCurve2D(new Point2D(0, 0), Direction2D.UnitX);
        // Tangent to the unit circle, through (3, 0)... centred on the x axis: the circles touching it on the axis
        ImmutableArray<TangentCircle2D> onAxis = Tangents2D.CirclesCentredOn(circle, new Point2D(3, 0), axis).Value;
        Assert.NotEmpty(onAxis);
        Assert.All(onAxis, c => Assert.Equal(0, c.Circle.Location.Y, 9));

        // A line is solved whole, whatever its range: the circles of radius 0.5 touching the circle, centred on the axis
        ImmutableArray<TangentCircle2D> ofRadius = Tangents2D.CirclesCentredOn(circle, axis, 0.5, new Interval(0, 10)).Value;
        Assert.Equal(4, ofRadius.Length);
        Assert.All(ofRadius, c => Assert.Equal(0.5, c.Circle.Radius, 12));
        Assert.All(ofRadius, c => Assert.Equal(0, c.Circle.Location.Y, 9));

        ImmutableArray<TangentLine2D> sloped = Tangents2D.LinesAtAngle(circle, new Line2D(new Point2D(0, -5), Direction2D.UnitX), Math.PI / 4).Value;
        Assert.Equal(2, sloped.Length);
        Assert.All(sloped, l => Assert.Equal(-5, l.Contacts[1].Point.Y, 9));

        TangentTarget2D bounded = TangentTarget2D.Tangent(circle, new Interval(0, Math.PI));
        Assert.Equal(new Interval(0, Math.PI), bounded.Range);
        Assert.Throws<ArgumentException>(() => TangentTarget2D.Tangent(circle, new Interval(1, 1)));
    }

    // ---------------------------------------------------------------------------------------------- construction

    [Fact]
    public void Wires_and_faces_are_built_from_parts()
    {
        Edge[] edges = [.. Square().Edges()];
        Wire connected = Wire.Connect(edges).Value;
        Assert.Equal(4, connected.Edges().Length);

        Wire hole = Wire.Polygon([new(4, 4, 0), new(6, 4, 0), new(6, 6, 0), new(4, 6, 0)], closed: true).Value;
        Face holed = Face.FromWires(Square(), [(Wire)hole.Reversed()]).Value;
        Assert.Equal(96, holed.Area(), 9);
        Assert.Equal(96, SquareFace().WithHoles([(Wire)hole.Reversed()]).Value.Area(), 9);
        Face onSurface = Face.FromSurface(new PlaneSurface(new Plane3D()), boundary: Square(), holes: [(Wire)hole.Reversed()]).Value;
        Assert.Equal(96, onSurface.Area(), 9);
    }

    [Fact]
    public void An_edge_on_a_surface_gets_its_3D_curve_when_asked()
    {
        var line = new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 1));
        var cylinder = new CylindricalSurface(new CoordinateSystem(), 1);
        Edge bare = Edge.FromCurve(line, cylinder, new Interval(0, 3), computeCurve3D: false).Value;
        Assert.False(bare.TryGetCurve(out _, out _));
        Shape built = bare.WithCurves3D(1e-9, Continuity.C1, 14, 2000).Value.Shape;
        Assert.True(((Edge)built).TryGetCurve(out _, out _));
        Assert.True(Edge.FromCurve(line, cylinder, new Interval(0, 3)).Value.TryGetCurve(out _, out _));
    }

    [Fact]
    public void Transforms_copy_geometry_when_asked_and_revolve_copies_its_profile()
    {
        Solid box = Box();
        Shape copy = box.Transformed(Transform3D.Translation(new Vector3D(1, 0, 0)), copyGeometry: true).Value.Shape;
        Assert.Equal(1000, copy.Volume(), 9);
        Shape stretched = box.Transformed(GeneralTransform3D.Affinity(Axis3D.ZAxis, 2), copyGeometry: true).Value.Shape;
        Assert.Equal(4000, stretched.Volume(), 6);

        Face profile = (Face)SquareFace(1).Moved(new Placement(Transform3D.Translation(new Vector3D(5, 0, 0))));
        Built<Shape> revolved = profile.Revolve(Axis3D.YAxis, Math.PI, copyProfile: true).Value;
        Assert.Equal(Math.PI * 5.5, revolved.Shape.Volume(), 6);
        Assert.DoesNotContain(revolved.Shape.Faces(), f => f.IsSame(profile));
    }

    [Fact]
    public void A_half_space_of_a_shell_and_edges_projected_onto_a_shape()
    {
        Shell top = Shell.Create([Box().Faces().First(f => f.ToSurface().Evaluate(0, 0).Z > 9)]);
        Solid half = Solid.HalfSpace(top, new Point3D(5, 5, 0)).Value.Shape;
        Assert.Equal(Containment.In, half.ClassifyPoint(new Point3D(5, 5, 0)));

        Edge across = Edge.Line(new(-5, 5, 20), new(15, 5, 20)).Value;
        ImmutableArray<Wire> projected = ShapeProjections.Project(across, Box(), -Direction3D.UnitZ).Value;
        Assert.NotEmpty(projected);
        Assert.NotEmpty(ShapeProjections.Project(across, Box(), new Point3D(5, 5, 100)).Value);
    }

    [Fact]
    public void Sections_can_leave_out_their_curves_on_the_faces()
    {
        // A plane's curves are computed on demand, so a sphere shows what was stored
        Solid ball = Solid.Sphere(5).Value.Shape;
        Face sphere = ball.Faces()[0];
        Face plane = Face.FromPlane(new Plane3D(new Point3D(0, 0, 1), Direction3D.UnitZ)).Value;
        var without = BooleanOptions.Default with { SectionCurvesOnFaces = false };
        Shape bare = BooleanOperations.Section([sphere], [plane], without).Value.Shape;
        Assert.NotEmpty(bare.Edges());
        Assert.All(bare.Edges(), e => Assert.False(e.TryGetCurveOnFace(sphere, out _, out _)));
        Shape full = BooleanOperations.Section([sphere], [plane]).Value.Shape;
        Assert.Contains(full.Edges(), e => e.TryGetCurveOnFace(sphere, out _, out _));
    }

    // ---------------------------------------------------------------------------------------------- healing

    [Fact]
    public void Healing_fixes_faces_wireframes_and_approximates()
    {
        Face face = SquareFace();
        Assert.Equal(100, face.FixFace().Value.Shape.Area(), 9);
        Assert.Empty(Box().MisorientedEdges());
        Shape wire = Square().FixWireframe(1e-3, new WireframeFixOptions { SmallEdgesFirst = true, MaxTolerance = 1e-2 }).Value.Shape;
        Assert.Equal(4, wire.Edges().Length);
        Shape splines = Solid.Cylinder(1, 2).Value.Shape.ApproximateBSplines(BSplineApproximationOptions.Default).Value.Shape;
        Assert.Equal(3, splines.Faces().Length);
        Assert.Equal(Math.PI * 2, splines.Volume(), 4);
        Assert.True(Box().Fix(new FixOptions { CheckResult = false }).IsSuccess);
    }

    [Fact]
    public void Sewing_records_the_edges_it_merges()
    {
        Face left = SquareFace(4);
        Face right = (Face)SquareFace(4).Moved(new Placement(Transform3D.Translation(new Vector3D(4, 0, 0))));
        Built<Shape> sewn = HealingOperations.Sew([left, right]).Value;
        Edge leftEdge = left.Edges().First(e => e.ToCurve().Evaluate(e.ToCurve().Range.Min).X == 4 && e.ToCurve().Evaluate(e.ToCurve().Range.Max).X == 4);
        Assert.NotEmpty(sewn.History.Modified(leftEdge));
    }

    // ---------------------------------------------------------------------------------------------- measurement

    [Fact]
    public void Points_are_classified_on_faces_and_their_parameters_found()
    {
        Face face = SquareFace();
        Assert.Equal(Containment.In, face.Classify(face.ParametersOf(new Point3D(5, 5, 0)).Parameters));
        Assert.Equal(Containment.Out, face.Classify(face.ParametersOf(new Point3D(15, 5, 0)).Parameters));
        (Point2D uv, double gap) = face.ParametersOf(new Point3D(3, 4, 1));
        Assert.Equal(1, gap, 9);
        Assert.True(face.ToSurface().Evaluate(uv.X, uv.Y).DistanceTo(new Point3D(3, 4, 0)) < Tol);
        Assert.True(Box().IsOnFace(new Point3D(5, 5, 10)));
        Assert.False(Box().IsOnFace(new Point3D(5, 5, 5)));
        Assert.Equal(Containment.In, Box().ClassifyPoint(new Point3D(5, 5, 5)));
    }

    // ---------------------------------------------------------------------------------------------- features, fillets, offsets

    [Fact]
    public void Faces_are_chamfered_by_corner_drafted_by_angle_and_split_along_edges()
    {
        Face face = SquareFace();
        Edge[] edges = [.. face.Edges()];
        Face chamfered = face.Chamfer2D([(edges[0], edges[1])], 1).Value.Shape;
        Assert.Equal(99.5, chamfered.Area(), 9);

        Solid box = Box();
        Face[] sides = [.. box.Faces().Where(f => Math.Abs(f.ToSurface().Normal(0.5, 0.5).Z) < 0.5)];
        Shape drafted = box.Draft(sides.Select(f => (f, 0.05)), Direction3D.UnitZ, new Plane3D()).Value.Shape;
        Assert.True(drafted.Volume() < 1000);

        Edge cut = Edge.Line(new(5, 0, 0), new(5, 10, 0)).Value;
        SplitFacesResult split = FeatureOperations.SplitAlong(face, [cut]).Value;
        Assert.Equal(2, split.Shape.Faces().Length);
        Assert.Single(split.Left);
        Assert.Single(split.Right);
        Assert.NotEmpty(split.History.Modified(face));
    }

    [Fact]
    public void Offsets_hollow_any_shape_and_offset_a_wire_in_a_face()
    {
        Solid box = Box();
        Face top = box.Faces().First(f => f.ToSurface().Evaluate(0, 0).Z > 9);
        Shape hollow = ((Shape)box).ThickSolid([top], -1).Value.Shape;
        Assert.Equal(1000 - 8 * 8 * 9, hollow.Volume(), 6);
        Shape grown = Square().Offset2D(1, SquareFace()).Value.Shape;
        Assert.True(grown.Edges().Length >= 4);
    }

    [Fact]
    public void An_extrusion_up_to_a_face_can_leave_its_sketch_face_alone()
    {
        Assert.True(new ExtrudeUntilOptions().ModifySketchFace);
        Assert.False((new ExtrudeUntilOptions() with { ModifySketchFace = false }).ModifySketchFace);
        Assert.False(new SweepOptions().CurvilinearEquivalence);
    }

    // ---------------------------------------------------------------------------------------------- meshing

    [Fact]
    public void A_mesh_read_keeps_the_shape_read_and_a_shape_mesh_has_none()
    {
        var stl = new MemoryStream();
        Assert.True(Kerf.Exchange.Stl.Write(Kerf.Meshing.Mesher.Mesh(Box()).Value, stl).IsSuccess);
        stl.Position = 0;
        Kerf.Meshing.ShapeMesh read = Kerf.Exchange.Stl.Read(stl).Value;
        Assert.Equal(ShapeType.Face, read.TriangulatedShape.Type);
        Assert.True(Kerf.Meshing.Mesher.Mesh(Box()).Value.TriangulatedShape.IsNull);
    }
}
