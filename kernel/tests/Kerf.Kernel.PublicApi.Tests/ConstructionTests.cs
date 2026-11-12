// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Construction (docs/api/construction.md; unit 10L2): primitives, sweeps, lofts, shapes from geometry,
/// transforms and projections.</summary>
public class ConstructionTests
{
    internal static Wire Square(double size, double z = 0) =>
        Wire.Polygon([new(0, 0, z), new(size, 0, z), new(size, size, z), new(0, size, z)], closed: true).Value;

    private static double MaxZ(Shape shape) => shape.AllVertices().Max(v => v.Point.Z);

    [Fact]
    public void Primitives_are_solids_of_their_sizes()
    {
        Solid box = Solid.Box(80, 60, 10).Value;
        Assert.Equal(6, box.AllFaces().Length);
        Assert.Equal(8, box.AllVertices().Length);
        Assert.Equal(10, MaxZ(box), 12);
        Assert.Equal(3, Solid.Cylinder(5, 30).Value.Shape.AllFaces().Length);
        Assert.Equal(40, MaxZ(Solid.Cylinder(5, 30, new Frame3D(new Point3D(0, 0, 10), Direction3D.UnitZ)).Value.Shape), 12);
        Assert.Equal(2, Solid.Cone(5, 0, 10).Value.Shape.AllFaces().Length);
        Assert.Single(Solid.Sphere(3).Value.Shape.AllFaces());
        Assert.Single(Solid.Torus(10, 2).Value.Shape.AllFaces());
        Assert.Equal(6, Solid.Wedge(10, 10, 10, new WedgeOptions { TopLength = 5 }).Value.Shape.AllFaces().Length);
        Assert.Equal(6, Primitives.Box(new Point3D(1, 1, 1), new Point3D(0, 0, 0)).Value.Shape.AllFaces().Length);
        Assert.Equal(ShapeType.Solid, Solid.HalfSpace(box.AllFaces()[0], new Point3D(40, 30, 5)).Value.Shape.Type);
    }

    [Fact]
    public void A_primitives_misuse_throws()
    {
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => Solid.Box(0, 1, 1));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => Solid.Cylinder(-1, 1));
        KernelAssert.Throws<ArgumentException>(() => Solid.Box(1, 1, 1, default(Frame3D)));
        KernelAssert.Throws<ArgumentException>(() => Solid.Cone(2, 2, 1));
        KernelAssert.Throws<ArgumentException>(() => Solid.Wedge(1, 1, 1, new WedgeOptions()));
    }

    [Fact]
    public void Shapes_are_made_from_geometry_and_their_failures_carry_the_builders_status()
    {
        Edge line = Edge.Line(new(0, 0, 0), new(1, 0, 0)).Value;
        Assert.Equal(2, line.AllVertices().Length);
        Assert.Equal("construction.line-through-identical-points", Edge.Line(Point3D.Origin, Point3D.Origin).Error!.Code.Value);
        Assert.Single(Edge.Circle(new Circle3D(Frame3D.World, 1)).Value.AllVertices());
        Assert.Equal(2, Edge.Arc(new Circle3D(Frame3D.World, 1), 0, Math.PI).Value.AllVertices().Length);
        Assert.Equal(new Point3D(2, 0, 0), Vertex.At(new Point3D(2, 0, 0)).Point);

        Edge far = Edge.Line(new(5, 5, 0), new(6, 5, 0)).Value;
        var broken = Wire.FromEdges([line, far]);
        Assert.False(broken.IsSuccess);
        Assert.Equal("construction.disconnected-wire", broken.Error!.Code.Value);

        Wire square = Square(10);
        Face face = Face.FromWire(square).Value;
        Assert.True(square.TryGetPlane(out Plane3D plane));
        Assert.True(plane.Axis.Direction.IsParallel(Direction3D.UnitZ, 1e-9));
        Assert.True(face.TryFindSurface(-1, out Surface? surface) && surface is PlaneSurface);

        Wire hole = Wire.Polygon([new(2, 2, 0), new(2, 4, 0), new(4, 4, 0), new(4, 2, 0)], closed: true).Value;
        Assert.Equal(2, Face.FromWires(square, [hole]).Value.AllWires().Length);

        Wire skew = Wire.Polygon([new(0, 0, 0), new(1, 0, 0), new(1, 1, 1), new(0, 1, 0)], closed: true).Value;
        Assert.Equal("construction.not-planar", Face.FromWire(skew, onlyPlane: true).Error!.Code.Value);

        var sphere = new SphericalSurface(new Sphere3D(CoordinateSystem.World, 1));
        Assert.Equal(ShapeType.Face, Face.FromSurface(sphere).Value.Type);
        Assert.Equal(ShapeType.Shell, Shell.FromSurface(sphere).Value.Type);
        Face onPlane = Face.FromPlane(new Plane3D(Point3D.Origin, Direction3D.UnitZ), square).Value;
        Assert.Equal(4, onPlane.AllEdges().Length);
        KernelAssert.Throws<ArgumentException>(() => Wire.Polygon([Point3D.Origin]));
        KernelAssert.Throws<ArgumentException>(() => Solid.FromShells([]));
    }

    [Fact]
    public void A_shell_of_polygons_shares_its_corners_and_edges()
    {
        // A unit cube's six squares, each anticlockwise about its outward normal (decision 1995)
        Point3D[] corners = [new(0, 0, 0), new(1, 0, 0), new(1, 1, 0), new(0, 1, 0), new(0, 0, 1), new(1, 0, 1), new(1, 1, 1), new(0, 1, 1)];
        (Plane3D, int[]) Side(Direction3D normal, double offset, params int[] loop) =>
            (new Plane3D(new Point3D(0, 0, 0) + new Vector3D(normal) * offset, normal), loop);
        var sides = new[]
        {
            Side(new Direction3D(0, 0, -1), 0, 0, 3, 2, 1), Side(Direction3D.UnitZ, 1, 4, 5, 6, 7),
            Side(new Direction3D(0, -1, 0), 0, 0, 1, 5, 4), Side(Direction3D.UnitY, 1, 2, 3, 7, 6),
            Side(new Direction3D(-1, 0, 0), 0, 0, 4, 7, 3), Side(Direction3D.UnitX, 1, 1, 2, 6, 5),
        };
        Shell shell = Shell.FromPolygons(corners, sides).Value;
        Assert.Equal(6, shell.AllFaces().Length);
        Assert.Equal(12, shell.AllEdges().Length);
        Assert.Equal(8, shell.AllVertices().Length);
        Solid solid = Solid.FromShells([shell]).Value;
        Assert.Equal(1, solid.IntegrateVolume(), 12);

        // Each face is as given: its normal the plane's, nothing turned round
        Assert.All(shell.AllFaces(), f => Assert.Equal(Orientation.Forward, f.Orientation));
        // A polygon crossing itself is refused
        var wrong = sides.ToArray();
        wrong[0] = Side(new Direction3D(0, 0, -1), 0, 0, 2, 3, 1);
        Assert.False(Shell.FromPolygons(corners, wrong).IsSuccess);
        KernelAssert.Throws<ArgumentException>(() => Shell.FromPolygons(corners, [(sides[0].Item1, new[] { 0, 1 })]));
        KernelAssert.Throws<ArgumentException>(() => Shell.FromPolygons(corners, [(sides[0].Item1, new[] { 0, 1, 8 })]));
    }

    [Fact]
    public void An_edge_on_a_surface_gets_its_3D_curve()
    {
        var plane = new PlaneSurface(new Plane3D(Point3D.Origin, Direction3D.UnitZ));
        var segment = new LineCurve2D(new Line2D(Point2D.Origin, Direction2D.UnitX));
        Edge edge = Edge.FromCurve(segment, plane, new Interval(0, 2)).Value;
        Assert.True(edge.TryGetCurve(out _, out _));
        Assert.Equal(2, edge.AllVertices().Max(v => v.Point.X), 9);
    }

    [Fact]
    public void An_extrusion_carries_its_history()
    {
        Face face = Face.FromWire(Square(10)).Value;
        Built<Shape> prism = face.Extrude(new Vector3D(0, 0, 15)).Value;
        Assert.Equal(ShapeType.Solid, prism.Shape.Type);
        Assert.Equal(15, MaxZ(prism.Shape), 12);
        Edge bottomEdge = face.AllEdges()[0];
        // the side face it swept, then its top edge (the caps, decision 1560)
        Assert.Equal(2, prism.History.Generated(bottomEdge).Count);
        Assert.Equal(ShapeType.Face, prism.History.Generated(bottomEdge)[0].Type);
        KernelAssert.Throws<ArgumentException>(() => face.Extrude(Vector3D.Zero));
    }

    [Fact]
    public void An_extrusions_and_a_revolutions_caps_are_in_their_histories()
    {
        Face face = Face.FromWire(Square(10)).Value;
        Built<Shape> prism = face.Extrude(new Vector3D(0, 0, 15)).Value;
        // the top cap is generated beside the solid; the bottom cap is the profile, or what stands for it
        IReadOnlyList<Shape> generated = prism.History.Generated(face);
        Assert.Equal([ShapeType.Solid, ShapeType.Face], generated.Select(g => g.Type));
        Assert.Equal(15, generated[1].AllVertices().Min(v => v.Point.Z), 9);
        Shape bottom = prism.History.Modified(face) is [var modified] ? modified : face;
        Assert.Contains(prism.Shape.AllFaces(), f => f.IsSame(bottom));
        Assert.Equal(0, bottom.AllVertices().Max(v => v.Point.Z), 9);
        Vertex corner = face.AllVertices()[0];
        Assert.Contains(prism.History.Generated(corner), g => g.Type == ShapeType.Vertex && Math.Abs(((Vertex)g).Point.Z - 15) < 1e-9);

        Face section = Face.FromWire(Wire.Polygon([new(5, 0, 0), new(6, 0, 0), new(6, 0, 1), new(5, 0, 1)], closed: true).Value).Value;
        Built<Shape> quarter = section.Revolve(Axis3D.ZAxis, Math.PI / 2).Value;
        Shape end = Assert.Single(quarter.History.Generated(section), g => g.Type == ShapeType.Face);
        Assert.Equal(5, end.AllVertices().Min(v => v.Point.Y), 9);
        // a whole turn ends where it starts: no cap
        Built<Shape> ring = section.Revolve(Axis3D.ZAxis).Value;
        Assert.DoesNotContain(ring.History.Generated(section), g => g.Type == ShapeType.Face);
    }

    [Fact]
    public void A_revolution_of_a_face_is_a_solid()
    {
        Face face = Face.FromWire(Wire.Polygon([new(5, 0, 0), new(6, 0, 0), new(6, 0, 1), new(5, 0, 1)], closed: true).Value).Value;
        Built<Shape> ring = face.Revolve(Axis3D.ZAxis).Value;
        Assert.Equal(ShapeType.Solid, ring.Shape.Type);
        Assert.Single(ring.History.Generated(face.AllEdges()[0]));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => face.Revolve(Axis3D.ZAxis, 0));
    }

    [Fact]
    public void A_revolution_shares_the_profile_and_answers_for_every_edge()
    {
        Face face = Face.FromWire(Wire.Polygon([new(5, 0, 0), new(6, 0, 0), new(6, 0, 1), new(5, 0, 1)], closed: true).Value).Value;
        foreach (double angle in new[] { Math.PI / 2, 2 * Math.PI })
        {
            Built<Shape> revolved = face.Revolve(Axis3D.ZAxis, angle).Value;
            // no copy of the profile: its vertices are the result's own
            Assert.All(face.AllVertices(), v => Assert.Contains(revolved.Shape.AllVertices(), r => r.IsSame(v)));
            // every edge swept a face of the result, even those a whole turn leaves out (decision 1558)
            foreach (Edge edge in face.AllEdges())
            {
                Shape swept = Assert.Single(revolved.History.Generated(edge), g => g.Type == ShapeType.Face);
                Assert.Contains(revolved.Shape.AllFaces(), f => f.IsSame(swept));
            }
        }
    }

    [Fact]
    public void A_filling_spans_its_boundary_and_passes_through_its_points()
    {
        Wire square = Square(10);
        FillBoundary[] boundary = [.. square.AllEdges().Select(e => new FillBoundary(e))];
        Built<Face> flat = Surfaces.Fill(boundary).Value;
        Assert.Equal(4, flat.Shape.AllEdges().Length);
        Assert.Equal(0, flat.Shape.AllVertices().Max(v => Math.Abs(v.Point.Z)), 6);
        Assert.Single(flat.History.Generated(square.AllEdges()[0]));

        Built<Face> domed = Surfaces.Fill(boundary, new FillOptions { Points = [new Point3D(5, 5, 2)] }).Value;
        Assert.True(domed.Shape.TryFindSurface(-1, out Surface? surface));
        Assert.NotNull(surface);
        Assert.Equal(2, MaxZOfSurface(surface!), 1);

        KernelAssert.Throws<ArgumentException>(() => Surfaces.Fill([]));
        KernelAssert.Throws<ArgumentException>(() => Surfaces.Fill([new FillBoundary(square.AllEdges()[0], Continuity.C2)]));
    }

    private static double MaxZOfSurface(Surface surface)
    {
        double max = double.MinValue;
        for (int i = 0; i <= 10; i++)
        {
            for (int j = 0; j <= 10; j++)
            {
                (Interval u, Interval v) = surface.Bounds;
                max = Math.Max(max, surface.Evaluate(u.Min + (u.Max - u.Min) * i / 10, v.Min + (v.Max - v.Min) * j / 10).Z);
            }
        }
        return max;
    }

    [Fact]
    public void A_helix_rises_by_its_pitch()
    {
        Wire helix = Sweeps.Helix(5, 2, 10).Value;
        Point3D[] ends = [.. helix.AllVertices().Select(v => v.Point)];
        Assert.Contains(ends, p => p.DistanceTo(new Point3D(5, 0, 0)) < 1e-6);
        Assert.Contains(ends, p => p.DistanceTo(new Point3D(5, 0, 10)) < 1e-6);

        Wire cone = Sweeps.Helix(new Frame3D(new Point3D(0, 0, 1), Direction3D.UnitZ), 5, 2, 10, new HelixOptions { EndRadius = 2 }).Value;
        Assert.Contains(cone.AllVertices(), v => v.Point.DistanceTo(new Point3D(2, 0, 11)) < 1e-6);

        KernelAssert.Throws<ArgumentOutOfRangeException>(() => Sweeps.Helix(5, 0, 10));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => Sweeps.Helix(5, 2, 10, new HelixOptions { EndRadius = -1 }));
    }

    // Upstream 3f3d32d9f3 (decision 34002): a taper under 1e-4 radians was taken for none, so the helix ended at its
    // start radius; every point of a conical helix is at the radius its height gives
    [Theory]
    [InlineData(10.0, 10.005)]
    [InlineData(10.0, 10.0 - 0.002)]
    [InlineData(5.0, 2.0)]
    [InlineData(2.0, 6.0)]
    public void A_conical_helix_is_at_the_radius_its_height_gives(double radius, double endRadius)
    {
        const double height = 100, pitch = 10;
        Wire helix = Sweeps.Helix(radius, pitch, height, new HelixOptions { EndRadius = endRadius }).Value;
        Assert.Contains(helix.AllVertices(), v => v.Point.DistanceTo(new Point3D(endRadius, 0, height)) < 1e-6);
        double worst = 0;
        foreach (Edge edge in helix.AllEdges())
        {
            EdgeCurve curve = edge.ToCurve();
            for (int i = 0; i <= 50; i++)
            {
                Point3D p = curve.Evaluate(curve.Range.Min + (curve.Range.Max - curve.Range.Min) * i / 50);
                double expected = radius + (endRadius - radius) * p.Z / height;
                worst = Math.Max(worst, Math.Abs(Math.Sqrt(p.X * p.X + p.Y * p.Y) - expected));
            }
        }
        Assert.True(worst < 1e-4, $"{worst} from the cone");
    }

    [Fact]
    public void A_sweep_follows_its_spine()
    {
        Wire profile = Wire.FromEdges([Edge.Circle(new Circle3D(Frame3D.World, 1)).Value]).Value;
        Wire spine = Wire.FromEdges([Edge.Line(Point3D.Origin, new Point3D(0, 0, 20)).Value]).Value;
        Built<Shape> tube = Sweeps.Sweep(profile, spine).Value;
        Assert.Equal(ShapeType.Solid, tube.Shape.Type);
        Assert.Equal(20, MaxZ(tube.Shape), 9);
        Built<Shape> open = Sweeps.Sweep(profile, spine, new SweepOptions { MakeSolid = false, Frame = SweepFrame.Frenet }).Value;
        Assert.NotEqual(ShapeType.Solid, open.Shape.Type);
        KernelAssert.Throws<ArgumentException>(() => Sweeps.Sweep(profile, spine, new SweepOptions { Frame = SweepFrame.Binormal }));
    }

    [Fact]
    public void A_loft_through_sections_and_its_failure()
    {
        Built<Shape> loft = Sweeps.Loft([Square(10), Square(10, 5), Square(10, 12)], new LoftOptions { Ruled = true }).Value;
        Assert.Equal(ShapeType.Solid, loft.Shape.Type);
        Assert.Equal(12, MaxZ(loft.Shape), 9);

        Wire triangle = Wire.Polygon([new(0, 0, 5), new(1, 0, 5), new(0, 1, 5)], closed: true).Value;
        var apexInside = Sweeps.Loft([Square(10), Vertex.At(new Point3D(5, 5, 5)), Square(10, 12)]);
        Assert.False(apexInside.IsSuccess);
        Assert.Equal("construction.loft-wrong-usage", apexInside.Error!.Code.Value);
        Assert.True(Sweeps.Loft([Square(10), triangle]).IsSuccess);
        KernelAssert.Throws<ArgumentException>(() => Sweeps.Loft([Square(1)]));
    }

    [Fact]
    public void Ruled_surfaces_join_edges_and_wires()
    {
        Edge a = Edge.Line(new(0, 0, 0), new(1, 0, 0)).Value;
        Edge b = Edge.Line(new(0, 1, 1), new(1, 1, 1)).Value;
        Assert.Equal(ShapeType.Face, Sweeps.Ruled(a, b).Value.Shape.Type);
        Assert.Equal(4, Sweeps.Ruled(Square(1), Square(1, 2)).Value.Shape.AllFaces().Length);
    }

    [Fact]
    public void Sweeps_are_cancelled_by_their_token()
    {
        using var source = new CancellationTokenSource();
        source.Cancel();
        Face face = Face.FromWire(Square(10)).Value;
        Assert.Throws<OperationCanceledException>(() => face.Extrude(new Vector3D(0, 0, 1), cancellationToken: source.Token));
        Assert.Throws<OperationCanceledException>(() => Sweeps.Loft([Square(10), Square(10, 5)], cancellationToken: source.Token));
    }

    [Fact]
    public void Transforms_make_new_shapes_with_their_history()
    {
        Solid box = Solid.Box(1, 1, 1).Value;
        Built<Shape> moved = box.Transformed(Transform3D.Translation(new Vector3D(0, 0, 5))).Value;
        Assert.Equal(6, MaxZ(moved.Shape), 12);
        Built<Shape> scaled = box.Scaled(Point3D.Origin, 3).Value;
        Assert.Equal(3, MaxZ(scaled.Shape), 12);
        Assert.Single(scaled.History.Modified(box.AllFaces()[0]));
        Built<Shape> mirrored = box.Mirrored(new Frame3D(Point3D.Origin, Direction3D.UnitZ)).Value;
        Assert.Equal(0, MaxZ(mirrored.Shape), 12);
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => box.Scaled(Point3D.Origin, 0));
    }

    [Fact]
    public void A_wire_projects_onto_a_shape_or_says_it_misses()
    {
        Solid box = Solid.Box(10, 10, 10).Value;
        Wire small = Wire.Polygon([new(2, 2, 20), new(4, 2, 20), new(4, 4, 20), new(2, 4, 20)], closed: true).Value;
        var onto = ShapeProjections.Project(small, box, Direction3D.UnitZ.Reversed());
        Assert.True(onto.IsSuccess);
        Assert.NotEmpty(onto.Value);
        Wire away = Wire.Polygon([new(50, 50, 20), new(52, 50, 20), new(52, 52, 20)], closed: true).Value;
        Assert.Equal(ShapeProjections.ProjectionEmpty, ShapeProjections.Project(away, box, Direction3D.UnitZ).Error!.Code);
    }

    [Fact]
    public void A_solid_of_no_volume_is_a_failure_carrying_it()
    {
        Wire square = Wire.Polygon([new(0, 0, 0), new(10, 0, 0), new(10, 10, 0), new(0, 10, 0)], closed: true).Value;
        Wire inner = Wire.Polygon([new(2, 2, 0), new(8, 2, 0), new(8, 8, 0), new(2, 8, 0)], closed: true).Value;
        Wire path = Wire.Polygon([new(0, 0, 0), new(20, 0, 0)], closed: false).Value;
        Face face = Face.FromWire(square).Value;

        void Empty(Result<Built<Shape>> made)
        {
            Assert.True(made.IsFailure);
            Assert.True(made.TryGetInvalid(out Built<Shape>? shape));
            Assert.Equal(0, shape.Shape.IntegrateVolume(), 1e-6);
        }

        // The direction in the profile's plane
        Result<Built<Shape>> flat = face.Extrude(new Vector3D(1, 0, 0));
        Empty(flat);
        Assert.Equal(Sweeps.NoVolume, flat.Error!.Code);
        // Sections in one plane
        Result<Built<Shape>> loft = Sweeps.Loft([square, inner], new LoftOptions { MakeSolid = true });
        Empty(loft);
        Assert.Equal(Sweeps.NoVolume, loft.Error!.Code);
        // A path in the section's plane
        Empty(Sweeps.Sweep(square, path, new SweepOptions { MakeSolid = true }));
        Empty(Sweeps.Sweep(square, path, new SweepOptions { MakeSolid = true, Frame = SweepFrame.Fixed, FixedFrame = Frame3D.World, TryOtherFrames = false }));

        // What's left is its volume, however thin: base times height
        Assert.Equal(100 * 1e-6, face.Extrude(new Vector3D(1, 0, 1e-6)).Value.Shape.IntegrateVolume(), 1e-9);
        Assert.Equal(300, face.Extrude(new Vector3D(0, 0, 3)).Value.Shape.IntegrateVolume(), 1e-9);
        // Asked not to check, it comes back as it was
        Assert.True(face.Extrude(new Vector3D(1, 0, 0), new ExtrudeOptions { CheckResult = false }).IsSuccess);
        // The profile turned to the path has its volume: area times length
        Assert.Equal(100 * 20, Sweeps.Sweep(square, path, new SweepOptions { MakeSolid = true, WithCorrection = true, TryOtherFrames = false }).Value.Shape.IntegrateVolume(), 1e-6);
    }
}
