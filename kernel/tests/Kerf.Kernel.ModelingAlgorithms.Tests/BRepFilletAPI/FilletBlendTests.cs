// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>
/// Fillet stage 4a (unit 15N): a rolling ball between two cylinders whose faces turn tangent at the ends of the edge, built where
/// ChFi3d's walk fails, as on reliability.md's equal tee. Each result is checked here apart from how it was built: the region the
/// fillet adds by slicing the tee and asking of each point whether a ball of the radius clear of both cylinders reaches it, and the
/// ball put back on the new faces.
/// </summary>
public class FilletBlendTests
{
    private static readonly MassOptions Precise = new() { Tolerance = 1e-12 };

    // reliability.md's equal tee: a cylinder of radius 5 along z, 20 long, and one of the same radius along y through it, 30 long,
    // both seams at +x, through the points where they're tangent, as Kerf.Modeling's are
    private static Shape Tee(double branch = 5) =>
        Solid.Cylinder(5, 20, new Frame3D(new Point3D(0, 0, -10), Direction3D.D.Z, Direction3D.D.X)).Value.Shape
             .Fuse(Solid.Cylinder(branch, 30, new Frame3D(new Point3D(0, -15, 0), Direction3D.D.Y, Direction3D.D.X)).Value.Shape).Value.Shape;

    // The edges where they meet: on both cylinders, not a seam
    private static Edge[] Meeting(Shape tee, double branch = 5) =>
        [.. tee.AllEdges().Where(e =>
        {
            var curve = new EdgeCurveAdaptor(e);
            Point3D p = curve.Value(0.37 * curve.FirstParameter() + 0.63 * curve.LastParameter());
            return Math.Abs(Math.Sqrt(p.X * p.X + p.Y * p.Y) - 5) < 1e-6 && Math.Abs(Math.Sqrt(p.X * p.X + p.Z * p.Z) - branch) < 1e-6;
        })];

    [Theory]
    [InlineData(0.02)]
    [InlineData(0.022502230342965425)]
    public void The_equal_tee_builds_where_ChFi3d_fails(double radius)
    {
        Shape tee = Tee();
        Edge[] edges = Meeting(tee);
        Assert.Equal(6, edges.Length);

        // ChFi3d alone fails here
        var chfi3d = new MakeFillet(tee);
        foreach (Edge edge in edges)
        {
            chfi3d.Add(radius, edge);
        }
        chfi3d.Build();
        Assert.False(chfi3d.IsDone);

        Built<Shape> made = tee.Fillet(edges, radius).Value;
        Shape shape = made.Shape;
        Assert.True(shape.Validate().IsValid);
        Assert.Empty(EdgeCrossings.All(tee, shape));
        Assert.Equal(12, shape.AllFaces().Length);
        AddsTheRegion(tee, shape, radius);
        BallTouchesBoth(shape, radius);

        // Each edge's history gives its fillet, and the edges are gone
        foreach (Edge edge in edges)
        {
            Assert.Single(made.History.Generated(edge), s => s.Type == ShapeType.Face);
            Assert.True(made.History.IsRemoved(edge));
        }
    }

    // The tee mirrored in z = 0, the same solid, holds its cylinders' faces reversed in its shell. The new faces go in beside the
    // chain's first face as the shell holds it; before decision 2208 they came out turned the wrong way there, and the result was
    // refused (decision 2170)
    [Theory]
    [InlineData(0.02)]
    [InlineData(0.5)]
    public void Beside_a_face_reversed_in_its_shell_the_new_faces_are_turned_as_it_is(double radius)
    {
        Shape tee = new Transform(Tee(), new Transform3D().WithMirror(new Frame3D(new Point3D(0, 0, 0), Direction3D.D.Z)), true).Shape();
        Assert.True(tee.Validate().IsValid);
        Edge[] edges = Meeting(tee);
        Assert.Equal(6, edges.Length);
        Assert.Contains(tee.AllFaces(), f => f.Orientation == Orientation.Reversed && new FaceSurfaceAdaptor(f).GetType() == SurfaceKind.Cylinder);

        EngineOutcome? outcome = FilletLimits.AfterAlgorithm(tee, [.. edges.Select(e => new FilletSpec(e, radius))], FilletOptions.Default);
        Assert.NotNull(outcome);
        Shape shape = outcome.Value.Shape();
        Assert.True(shape.Validate().IsValid);
        Assert.Empty(EdgeCrossings.All(tee, shape));
        Assert.Equal(12, shape.AllFaces().Length);
        AddsTheRegion(tee, shape, radius);
        BallTouchesBoth(shape, radius);
    }

    [Theory]
    [InlineData(0.3)]
    [InlineData(1)]
    [InlineData(2.5)]
    public void Where_ChFi3d_succeeds_the_path_isnt_asked_but_agrees_with_the_slices(double radius)
    {
        // Asked directly, the path gives the region the slices give; ChFi3d's own result is left as it is (its fillet is off the
        // ball by 3e-5 at 0.3 and 1.6e-2 at 2.5: occt-issues.md)
        Shape tee = Tee();
        Edge[] edges = Meeting(tee);
        EngineOutcome? outcome = FilletLimits.AfterAlgorithm(tee, [.. edges.Select(e => new FilletSpec(e, radius))], FilletOptions.Default);
        Assert.NotNull(outcome);
        Shape shape = outcome.Value.Shape();
        Assert.True(shape.Validate().IsValid);
        Assert.Empty(EdgeCrossings.All(tee, shape));
        AddsTheRegion(tee, shape, radius);
        BallTouchesBoth(shape, radius);
    }

    [Theory]
    [InlineData(0.05)]
    [InlineData(0.5)]
    public void A_convex_blend_between_cylinders_tangent_at_its_ends_is_built_with_the_ball_inside(double radius)
    {
        // Two equal cylinders' common, a Steinmetz solid: its edges are the same half ellipses, convex, and the ball runs inside
        Shape common = Solid.Cylinder(5, 20, new Frame3D(new Point3D(0, 0, -10), Direction3D.D.Z, Direction3D.D.X)).Value.Shape
                            .Common(Solid.Cylinder(5, 30, new Frame3D(new Point3D(0, -15, 0), Direction3D.D.Y, Direction3D.D.X)).Value.Shape).Value.Shape;
        Edge[] edges = Meeting(common);
        Assert.NotEmpty(edges);
        EngineOutcome? outcome = FilletLimits.AfterAlgorithm(common, [.. edges.Select(e => new FilletSpec(e, radius))], FilletOptions.Default);
        Assert.NotNull(outcome);
        Shape shape = outcome.Value.Shape();
        Assert.True(shape.Validate().IsValid);
        Assert.Empty(EdgeCrossings.All(common, shape));
        Assert.True(shape.IntegrateVolume(Precise) < common.IntegrateVolume(Precise));
        BallTouchesBoth(shape, radius, -1);
    }

    [Fact]
    public void Edges_whose_faces_arent_tangent_at_their_ends_are_left_to_ChFi3d()
    {
        // A branch of radius 2 meets the cylinder at an angle all round: no end where the fillet vanishes, so this path doesn't build it
        Shape tee = Tee(2);
        Edge[] edges = Meeting(tee, 2);
        Assert.NotEmpty(edges);
        Assert.Null(FilletLimits.AfterAlgorithm(tee, [.. edges.Select(e => new FilletSpec(e, 0.3))], FilletOptions.Default));
    }

    [Fact]
    public void Radii_that_differ_are_left_to_ChFi3d()
    {
        Shape tee = Tee();
        Edge[] edges = Meeting(tee);
        Assert.Null(FilletLimits.AfterAlgorithm(tee, [.. edges.Select((e, i) => new FilletSpec(e, i == 0 ? 0.03 : 0.02))], FilletOptions.Default));
    }

    // ChFi3d's own fillets on the tee are on the ball (decision 2156). 15R found them off it, as OCCT's are (decision 2141;
    // occt-issues.md): BlendFunc_ConstRad put each section's circle in the plane square to the edge, tangent there to that plane's
    // sections of the two cylinders, where the rolling ball's circle lies in the plane square to the path of its centre. Kerf's
    // ConstRad takes the surfaces' own normals, so the walk's sections are the ball's, and what's left is the approximation's own
    // error; the volume the fillet adds is the slices' region to what that moves the faces by over their area. They were off by
    // 3.9e-5, 1.30e-3 and 1.71e-2, and added 0.4853598, 5.3579340 and 33.2356606.
    [Theory]
    [InlineData(0.3)]
    [InlineData(1)]
    [InlineData(2.5)]
    public void ChFi3ds_tee_fillet_is_on_the_ball(double radius)
    {
        Shape tee = Tee();
        var chfi3d = new MakeFillet(tee);
        foreach (Edge edge in Meeting(tee))
        {
            chfi3d.Add(radius, edge);
        }
        chfi3d.Build();
        Assert.True(chfi3d.IsDone);
        Assert.True(BallMiss(chfi3d, radius, static p => Math.Abs(Math.Sqrt(p.X * p.X + p.Y * p.Y) - 5),
                             static p => Math.Abs(Math.Sqrt(p.X * p.X + p.Z * p.Z) - 5)) < Approximation);
        Shape shape = chfi3d.Shape();
        double added = shape.IntegrateVolume(Precise) - tee.IntegrateVolume(Precise);
        double area = shape.AllFaces().Where(f => new FaceSurfaceAdaptor(f).GetType() is not (SurfaceKind.Plane or SurfaceKind.Cylinder))
                           .Sum(f => ((Shape)f).IntegrateArea(Precise));
        Assert.Equal(TeeRegion(radius), added, Approximation * area);
    }

    // What ChFi3d's approximation of a walked fillet is asked for (its tolapp3d): the ball on its faces is within this, where the
    // sections are the ball's
    private const double Approximation = 1e-4;

    private delegate double Distance(Point3D p);

    private static double Rho(Point3D p, Point3D o, Direction3D d, out double s)
    {
        double vx = p.X - o.X, vy = p.Y - o.Y, vz = p.Z - o.Z;
        s = vx * d.X + vy * d.Y + vz * d.Z;
        double rx = vx - s * d.X, ry = vy - s * d.Y, rz = vz - s * d.Z;
        return Math.Sqrt(rx * rx + ry * ry + rz * rz);
    }

    private static Shape Cylinder(double r, double h, Point3D o, Direction3D d) => Solid.Cylinder(r, h, new Frame3D(o, d)).Value.Shape;

    // 15R's fourteen pairs of faces (decision 2141), each a solid and the distances to the two surfaces its edges lie on
    private static (Func<Shape> Make, Distance First, Distance Second) Pair(string name)
    {
        Direction3D z = new(0, 0, 1), y = new(0, 1, 0);
        Point3D origin = new(0, 0, 0);
        Shape Main() => Cylinder(5, 20, new Point3D(0, 0, -10), z);
        Distance Cyl(Point3D o, Direction3D d, double r) => p => Math.Abs(Rho(p, o, d, out _) - r);
        Distance main = Cyl(origin, z, 5);
        Distance Cone(Point3D o, Direction3D d, double r1, double r2, double h) => p =>
        {
            double rho = Rho(p, o, d, out double s), k = (r2 - r1) / h;
            return Math.Abs(rho - r1 - k * s) / Math.Sqrt(1 + k * k);
        };
        Direction3D At(double degrees) => new(0, Math.Sin(degrees * Math.PI / 180), Math.Cos(degrees * Math.PI / 180));
        Shape Plate() => Solid.Box(20, 20, 5, new Frame3D(new Point3D(-10, -10, -5), z, new Direction3D(1, 0, 0))).Value.Shape;
        return name switch
        {
            "tee 5/5" or "tee 5/4" or "tee 5/3" or "tee 5/2" => (() => Main().Fuse(Cylinder(name[^1] - '0', 30, new Point3D(0, -15, 0), y)).Value.Shape,
                                                                   main, Cyl(origin, y, name[^1] - '0')),
            "branch 3 at 60" or "branch 3 at 45" => (() => Main().Fuse(Cylinder(3, 9, origin, At(name[^2] == '6' ? 60 : 45))).Value.Shape,
                                                       main, Cyl(origin, At(name[^2] == '6' ? 60 : 45), 3)),
            "branch 2 offset by 2" => (() => Main().Fuse(Cylinder(2, 15, new Point3D(2, 0, 0), y)).Value.Shape, main, Cyl(new Point3D(2, 0, 0), y, 2)),
            "cone into a cylinder" => (() => Main().Fuse(Solid.Cone(3, 2, 12, new Frame3D(origin, y)).Value.Shape).Value.Shape,
                                       main, Cone(origin, y, 3, 2, 12)),
            "cylinder through a sphere's centre" => (() => Solid.Sphere(6).Value.Shape.Fuse(Cylinder(2, 10, origin, z)).Value.Shape,
                                                     p => Math.Abs(p.DistanceTo(origin) - 6), Cyl(origin, z, 2)),
            "cylinder through a sphere off its centre" => (() => Solid.Sphere(6).Value.Shape.Fuse(Cylinder(2, 10, new Point3D(2, 0, 0), z)).Value.Shape,
                                                           p => Math.Abs(p.DistanceTo(origin) - 6), Cyl(new Point3D(2, 0, 0), z, 2)),
            "cylinder through a torus's tube" => (() => Solid.Torus(6, 2).Value.Shape.Fuse(Cylinder(1, 8, new Point3D(6, 0, -4), z)).Value.Shape,
                                                  p =>
                                                  {
                                                      double rho = Rho(p, origin, z, out double s);
                                                      return Math.Abs(Math.Sqrt((rho - 6) * (rho - 6) + s * s) - 2);
                                                  }, Cyl(new Point3D(6, 0, 0), z, 1)),
            "cylinder tilted through a plane" => (() => Plate().Fuse(Cylinder(2, 8, new Point3D(0, 0, -2), At(30))).Value.Shape,
                                                  p => Math.Abs(p.Z), Cyl(new Point3D(0, 0, -2), At(30), 2)),
            "upright cylinder on a plane" => (() => Plate().Fuse(Cylinder(2, 8, new Point3D(0, 0, -2), z)).Value.Shape,
                                              p => Math.Abs(p.Z), Cyl(new Point3D(0, 0, -2), z, 2)),
            "cone on a cylinder's axis" => (() => Main().Fuse(Solid.Cone(7, 3, 6, new Frame3D(origin, z)).Value.Shape).Value.Shape,
                                            main, Cone(origin, z, 7, 3, 6)),
            // Not one of the fourteen: a box's edge along z, between the planes x = 0 and y = 0
            "box's edge" or "short box's edge" => (() => Solid.Box(20, 20, name[0] == 's' ? 4 : 20,
                                                                   new Frame3D(origin, z, new Direction3D(1, 0, 0))).Value.Shape,
                                                   p => Math.Abs(p.X), p => Math.Abs(p.Y)),
            _ => throw new ArgumentException(name),
        };
    }

    // Each pair at two radii, and the branch at 60 degrees at 1, whose fillet runs onto the branch's end, where ChFi3d blends a
    // surface and that end's edge with BlendFunc_SurfRstConstRad, on the ball since decision 2158. Left out: the branch at 45
    // degrees at 0.3, where ChFi3d's approximation fails as it does at 0.05 to 0.2 without the change.
    public static TheoryData<string, double> Pairs() => new()
    {
        { "tee 5/5", 0.3 }, { "tee 5/5", 1 }, { "tee 5/4", 0.3 }, { "tee 5/4", 1 }, { "tee 5/3", 0.3 }, { "tee 5/3", 1 },
        { "tee 5/2", 0.3 }, { "tee 5/2", 1 }, { "branch 3 at 60", 0.3 }, { "branch 3 at 60", 0.6 }, { "branch 3 at 60", 1 },
        { "branch 3 at 45", 0.6 },
        { "branch 3 at 45", 1 }, { "branch 2 offset by 2", 0.3 }, { "branch 2 offset by 2", 1 }, { "cone into a cylinder", 0.3 },
        { "cone into a cylinder", 1 }, { "cylinder through a sphere's centre", 0.3 }, { "cylinder through a sphere's centre", 1 },
        { "cylinder through a sphere off its centre", 0.3 }, { "cylinder through a sphere off its centre", 1 },
        { "cylinder through a torus's tube", 0.3 }, { "cylinder through a torus's tube", 1 }, { "cylinder tilted through a plane", 0.3 },
        { "cylinder tilted through a plane", 1 }, { "upright cylinder on a plane", 0.3 }, { "upright cylinder on a plane", 1 },
        { "cone on a cylinder's axis", 0.3 }, { "cone on a cylinder's axis", 1 },
    };

    // 15N's ball on ChFi3d's walked fillets between 15R's pairs (decision 2156): within the approximation's tolerance everywhere.
    // They were off by up to 2.4e-2 at 1 (decision 2141), growing as the radius cubed.
    [Theory]
    [MemberData(nameof(Pairs))]
    public void ChFi3ds_walked_fillets_are_on_the_ball(string name, double radius)
    {
        (Func<Shape> make, Distance first, Distance second) = Pair(name);
        Shape shape = make();
        Edge[] edges = [.. shape.AllEdges().Where(e =>
        {
            var curve = new EdgeCurveAdaptor(e);
            Point3D p = curve.Value(0.37 * curve.FirstParameter() + 0.63 * curve.LastParameter());
            return first(p) < 1e-6 && second(p) < 1e-6;
        })];
        Assert.NotEmpty(edges);
        var chfi3d = new MakeFillet(shape);
        foreach (Edge edge in edges)
        {
            chfi3d.Add(radius, edge);
        }
        chfi3d.Build();
        Assert.True(chfi3d.IsDone);
        Assert.True(chfi3d.Shape().Validate().IsValid);
        // Where the branch at 60 degrees ends, the ball rolls on the end's edge, a circle of radius 3 at 9 along its axis
        // (BlendFunc_SurfRstConstRad, decision 2158): there it's at the radius from that circle
        Distance side = name == "branch 3 at 60" ? p => Nearer(second(p), EndCircle(p, 60), radius) : second;
        double miss = BallMiss(chfi3d, radius, p => first(p), p => side(p));
        Assert.True(miss < Approximation, $"{name} at {radius}: the ball misses by {miss}");
    }

    // An evolving radius, 0.4 at the contour's ends and 1 at its middle, a steep law (BlendFunc_EvolRad; unit 15AK). A ball of
    // changing radius sweeps the envelope of its positions, which meets each ball in a circle square to the path of its centre:
    // so at every point of the new faces the surface's normal goes through the centre of a ball that touches both surfaces, of
    // a radius the law takes, to the approximation's tolerance. The new faces meet each surface along a boundary, and the ball
    // through a point of it touches the surface there, so they're tangent along the contacts. Before, each section was a great
    // circle of its ball (decision 2215), and this missed by up to 6.5e-3, 1.4e-2 and 1.8e-2 (decision 2218); OCCT's sections
    // were 2D fillets in the plane square to the edge, further off.
    // With "unlike laws" every other edge of the contour rises to 0.7 only, so the law's second derivative jumps where two
    // edges meet (its first is nothing there, as ChFi3d's laws' always is at an edge's end): the sections are continuous
    // there and their derivative along the walk isn't, and the approximation is cut there.
    [Theory]
    [InlineData("cylinder tilted through a plane", false)]
    [InlineData("cylinder through a sphere off its centre", false)]
    [InlineData("tee 5/3", false)]
    [InlineData("box's edge", false)]
    [InlineData("short box's edge", false)]
    [InlineData("tee 5/3", true)]
    [InlineData("tee 5/4", true)]
    public void ChFi3ds_evolving_fillets_are_the_envelope_of_their_balls(string name, bool unlikeLaws)
    {
        (Func<Shape> make, Distance first, Distance second) = Pair(name);
        Shape shape = make();
        Edge[] edges = [.. shape.AllEdges().Where(e =>
        {
            var curve = new EdgeCurveAdaptor(e);
            Point3D p = curve.Value(0.37 * curve.FirstParameter() + 0.63 * curve.LastParameter());
            return first(p) < 1e-6 && second(p) < 1e-6;
        })];
        Assert.NotEmpty(edges);
        var chfi3d = new MakeFillet(shape);
        NCollection_Array1<Point2D> law = new(1, 3);
        law.SetValue(1, new Point2D(0.0, 0.4));
        law.SetValue(2, new Point2D(0.5, 1.0));
        law.SetValue(3, new Point2D(1.0, 0.4));
        NCollection_Array1<Point2D> lower = new(1, 3);
        lower.SetValue(1, new Point2D(0.0, 0.4));
        lower.SetValue(2, new Point2D(0.5, 0.7));
        lower.SetValue(3, new Point2D(1.0, 0.4));
        Assert.True(!unlikeLaws || edges.Length > 1);
        for (int i = 0; i < edges.Length; i++)
        {
            chfi3d.Add(unlikeLaws && i % 2 == 1 ? lower : law, edges[i]);
        }
        chfi3d.Build();
        Assert.True(chfi3d.IsDone);
        Assert.True(chfi3d.Shape().Validate().IsValid);
        (double miss, double other, double offContact, double least, double greatest) = EnvelopeMiss(chfi3d, first, second, 0.2, 1.5);
        Assert.True(miss < Approximation, $"{name}: the envelope's balls miss by {miss}");
        Assert.True(other < 10 * Approximation, $"{name}: the balls that touch one surface miss the other by {other}");
        Assert.True(offContact < Approximation, $"{name}: the contacts are off their surfaces by {offContact}");
        // A box's surface runs a little past its edge's ends, where the law's cubic goes on below 0.4 (to 0.39967, and to
        // 0.39185 on the short box, whose edge is 4 long: a law that rises 0.6 over 2)
        Assert.InRange(least, 0.39, 0.45);
        Assert.InRange(greatest, 0.95, 1 + Approximation);
    }

    // A radius that runs down to nothing at an edge's end (the corpus's blend/tolblend_buildvol: 50 to 100 to 0 along a box's
    // edge): the envelope's circle shrinks to a point there with the ball, and is made from the directions to the contacts, not
    // the contacts themselves, which meet. Built, valid, on the envelope where it has any width.
    [Fact]
    public void An_evolving_fillet_whose_radius_runs_down_to_nothing_is_built()
    {
        Shape box = Solid.Box(500, 500, 500, new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))).Value.Shape;
        Edge edge = box.AllEdges().Single(e =>
        {
            var curve = new EdgeCurveAdaptor(e);
            Point3D p = curve.Value(0.37 * curve.FirstParameter() + 0.63 * curve.LastParameter());
            return Math.Abs(p.X) < 1e-6 && Math.Abs(p.Y) < 1e-6;
        });
        var chfi3d = new MakeFillet(box);
        NCollection_Array1<Point2D> law = new(1, 3);
        law.SetValue(1, new Point2D(0.0, 50));
        law.SetValue(2, new Point2D(0.5, 100));
        law.SetValue(3, new Point2D(1.0, 0));
        chfi3d.Add(law, edge);
        chfi3d.Build();
        Assert.True(chfi3d.IsDone);
        Assert.True(chfi3d.Shape().Validate().IsValid);
        (double miss, _, double offContact, _, double greatest) = EnvelopeMiss(chfi3d, p => Math.Abs(p.X), p => Math.Abs(p.Y), 1e-9, 150);
        // 1.2e-4 at radii up to 100
        Assert.True(miss < 5 * Approximation, $"the envelope's balls miss by {miss}");
        Assert.True(offContact < Approximation, $"the contacts are off their surfaces by {offContact}");
        // The law's cubic, through unlike rises, goes a little past 100 beside its middle point (to 101.6)
        Assert.InRange(greatest, 95, 105);
    }

    // Unit 15AK (decision 32306): where the ball passes from one face to the next while its radius is changing and the faces'
    // curvatures differ (a plane to the cylinder tangent to it, round a contour's arc), the path of its centre has a corner:
    // the centre's speed along the guide changes with the contacts' own. The surface the balls sweep has a piece of the ball
    // there, between the last circle of one face's fillet and the first of the next's, which the walked faces, each carried on
    // until they meet, leave by about the square of the gap between those circles over twice the radius. Within the
    // approximation's tolerance of that the fillet is built, and is on the boundary of the union of its balls to that
    // tolerance (2.3e-6, 2.6e-5 and 9.1e-5 as the radius's end goes from 1.5 to 3); past it, it fails, saying why, where it
    // built 7.2e-4 off before (at 4).
    [Theory]
    [InlineData(1.5, true)]
    [InlineData(2.5, true)]
    [InlineData(3.0, true)]
    [InlineData(3.5, false)]
    [InlineData(4.0, false)]
    public void An_evolving_fillet_round_a_contours_arc_is_built_only_within_tolerance_of_its_balls(double end, bool built)
    {
        MakeFillet chfi3d = RoundedBoxTop(end, out Distance side);
        chfi3d.Build();
        Assert.Equal(built, chfi3d.IsDone);
        if (!built)
        {
            Assert.Equal(Kerf.Engine.Fillets.Data.ErrorStatus.EnvelopeCorner, chfi3d.StripeStatus(1));
            Assert.Contains("fillet.radius-changes-at-a-corner", Kerf.Engine.Foundation.StatusDiagnostics.ToDiagnostic(chfi3d.StripeStatus(1))!.Code.ToString());
            return;
        }
        Assert.True(chfi3d.Shape().Validate().IsValid);
        (double worst, double inner, _) = UnionMiss(chfi3d, side);
        Assert.True(worst < Approximation, $"to {end}: off the union of the balls by {worst}");
        Assert.True(inner < 0.1 * Approximation, $"to {end}: off the union of the balls by {inner} away from the arc's ends");
    }

    // A radius that bends hard enough along the edge (0.2 to 2 and back along a box's edge of 4) that the circles of
    // neighbouring balls cross between the contacts: the swept surface folds back on itself, the larger balls covering the
    // smaller ones' circles, and the fillet fails, saying why. The same law to 1 folds nowhere and is built, on the envelope.
    [Theory]
    [InlineData(1.0, true)]
    [InlineData(2.0, false)]
    public void An_evolving_fillet_whose_swept_surface_would_fold_fails(double top, bool built)
    {
        (Func<Shape> make, Distance first, Distance second) = Pair("short box's edge");
        Shape shape = make();
        Edge edge = shape.AllEdges().Single(e =>
        {
            var curve = new EdgeCurveAdaptor(e);
            Point3D p = curve.Value(0.37 * curve.FirstParameter() + 0.63 * curve.LastParameter());
            return first(p) < 1e-6 && second(p) < 1e-6;
        });
        var chfi3d = new MakeFillet(shape);
        NCollection_Array1<Point2D> law = new(1, 3);
        law.SetValue(1, new Point2D(0.0, 0.2));
        law.SetValue(2, new Point2D(0.5, top));
        law.SetValue(3, new Point2D(1.0, 0.2));
        chfi3d.Add(law, edge);
        chfi3d.Build();
        Assert.Equal(built, chfi3d.IsDone);
        if (!built)
        {
            Assert.Equal(Kerf.Engine.Fillets.Data.ErrorStatus.EnvelopeCorner, chfi3d.StripeStatus(1));
            return;
        }
        Assert.True(chfi3d.Shape().Validate().IsValid);
        (double miss, _, _, _, _) = EnvelopeMiss(chfi3d, first, second, 0.05, 2);
        Assert.True(miss < Approximation, $"the envelope's balls miss by {miss}");
    }

    // A box of 20 by 20 by 10 with its upright edge at the origin rounded at 5, and the fillet of the top's edge along x = 0,
    // which runs on, tangent, round the arc and along y = 0: a guide whose curvature jumps where the line meets the arc. The
    // radius goes evenly from 1 to the end's along the three edges, so it is changing at both jumps.
    private static MakeFillet RoundedBoxTop(double end, out Distance side)
    {
        Shape box = Solid.Box(20, 20, 10, new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))).Value.Shape;
        Edge upright = box.AllEdges().Single(e =>
        {
            var curve = new EdgeCurveAdaptor(e);
            Point3D p = curve.Value(0.37 * curve.FirstParameter() + 0.63 * curve.LastParameter());
            return Math.Abs(p.X) < 1e-6 && Math.Abs(p.Y) < 1e-6;
        });
        var round = new MakeFillet(box);
        round.Add(5.0, upright);
        round.Build();
        Shape rounded = round.Shape();
        Edge top = rounded.AllEdges().Single(e =>
        {
            var curve = new EdgeCurveAdaptor(e);
            Point3D p = curve.Value(0.37 * curve.FirstParameter() + 0.63 * curve.LastParameter());
            return Math.Abs(p.X) < 1e-6 && Math.Abs(p.Z - 10) < 1e-6;
        });
        side = p => p.X < 5 && p.Y < 5 ? Math.Abs(Math.Sqrt((p.X - 5) * (p.X - 5) + (p.Y - 5) * (p.Y - 5)) - 5)
                    : p.Y >= 5 && p.X < 5 ? Math.Abs(p.X) : p.X >= 5 && p.Y < 5 ? Math.Abs(p.Y) : Math.Min(Math.Abs(p.X), Math.Abs(p.Y));
        var chfi3d = new MakeFillet(rounded);
        chfi3d.Add(top);
        Kerf.Engine.Construction.Laws.Linear law = new();
        law.Set(0.0, 1.0, chfi3d.Length(1), end);
        chfi3d.SetRadius(law, 1, 1);
        return chfi3d;
    }

    // The fillet of RoundedBoxTop against the union of its balls, apart from the law and the guide: each point of a face's
    // contact with the top (z = 10) is above the centre of the ball that touches there, whose radius is that point's distance
    // from the side (30,001 of them along each face); a point of the fillet is on the surface the balls sweep if it is on the
    // boundary of their union. The most a point of the new faces is inside the union or outside it (47 by 47 points of each,
    // crowded towards its ends), and the most among those more than 1 from both jumps.
    private static (double Worst, double Inner, Point3D At) UnionMiss(MakeFillet fillet, Distance side)
    {
        List<(Point3D C, double R)> balls = [];
        List<(FaceSurfaceAdaptor Surface, Face Face)> faces = [];
        for (int i = 1; i <= fillet.SurfaceCount; i++)
        {
            foreach (Shape face in fillet.NewFaces(i))
            {
                var surface = new FaceSurfaceAdaptor((Face)face);
                faces.Add((surface, (Face)face));
                double u0 = surface.FirstUParameter(), u1 = surface.LastUParameter(), v0 = surface.FirstVParameter(), v1 = surface.LastVParameter();
                Point3D mid = surface.Value((u0 + u1) / 2, (v0 + v1) / 2);
                int region = mid.X < 5 && mid.Y < 5 ? 1 : mid.Y >= 5 ? 0 : 2;
                foreach ((bool alongV, double fix) in new[] { (true, u0), (true, u1), (false, v0), (false, v1) })
                {
                    Point3D On(double t) => alongV ? surface.Value(fix, v0 + (v1 - v0) * t) : surface.Value(u0 + (u1 - u0) * t, fix);
                    if (Enumerable.Range(0, 21).Any(k => Math.Abs(On(k / 20.0).Z - 10) > 1e-4))
                    {
                        continue;
                    }
                    for (int k = 0; k <= 30000; k++)
                    {
                        Point3D q = On(k / 30000.0);
                        int at = q.X < 5 && q.Y < 5 ? 1 : q.Y >= 5 ? 0 : 2;
                        if (at != region || q.X > 19.5 || q.Y > 19.5)
                        {
                            continue;
                        }
                        double r = side(q);
                        balls.Add((new Point3D(q.X, q.Y, 10 - r), r));
                    }
                }
            }
        }
        double worst = 0, inner = 0;
        Point3D where = default;
        double[] near = [1e-5, 3e-5, 1e-4, 3e-4, 1e-3, 2e-3, 4e-3, 7e-3, 0.012, 0.02, 0.03];
        double[] ts = [.. Enumerable.Range(0, 25).Select(k => k / 24.0), .. near, .. near.Select(x => 1 - x)];
        foreach ((FaceSurfaceAdaptor surface, Face face) in faces)
        {
            double u0 = surface.FirstUParameter(), u1 = surface.LastUParameter(), v0 = surface.FirstVParameter(), v1 = surface.LastVParameter();
            foreach (double ta in ts)
            {
                foreach (double tb in ts)
                {
                    double u = u0 + (u1 - u0) * ta, v = v0 + (v1 - v0) * tb;
                    if (new Kerf.Engine.Classification.FaceClassifier(face, new Point2D(u, v), 1e-7).State() != Containment.In)
                    {
                        continue;
                    }
                    Point3D p = surface.Value(u, v);
                    if (p.X > 19 || p.Y > 19)
                    {
                        continue;
                    }
                    double depth = double.MinValue;
                    foreach ((Point3D c, double r) in balls)
                    {
                        double dx = p.X - c.X, dy = p.Y - c.Y, dz = p.Z - c.Z;
                        depth = Math.Max(depth, r - Math.Sqrt(dx * dx + dy * dy + dz * dz));
                    }
                    depth = Math.Abs(depth);
                    if (depth > worst)
                    {
                        worst = depth;
                        where = p;
                    }
                    bool nearJump = (Math.Abs(p.Y - 5) < 1 && p.X < 5) || (Math.Abs(p.X - 5) < 1 && p.Y < 5);
                    if (!nearJump)
                    {
                        inner = Math.Max(inner, depth);
                    }
                }
            }
        }
        return (worst, inner, where);
    }

    // The envelope's test, the ball test's kin for a radius that changes. At 39 by 9 points of each new face, its boundaries
    // among them, the ball centred on the surface's normal that best touches both surfaces (its radius between the bounds given):
    // the most such a ball misses either surface by, and the least and greatest of the radii. And the most the points of the
    // faces' two boundaries along the contacts are off the surface each lies on: with the ball's test there, tangency. That
    // ball shares what it misses by between the two surfaces, and a surface whose normals are a little out still finds one
    // nearly touching both; so also, at the points inside, the ball on the normal that touches one surface exactly: the most
    // it misses the other by, which is the normal's error over the little its centre's distance from the first changes along
    // the normal near that contact (some thirty times, an eighth of the way across).
    private static (double Miss, double Other, double OffContact, double Least, double Greatest) EnvelopeMiss(
        MakeFillet fillet, Distance first, Distance second, double lower, double upper)
    {
        double worst = 0, other = 0, offContact = 0, least = double.MaxValue, greatest = 0;
        for (int i = 1; i <= fillet.SurfaceCount; i++)
        {
            foreach (Shape face in fillet.NewFaces(i))
            {
                var surface = new FaceSurfaceAdaptor((Face)face);
                double u0 = surface.FirstUParameter(), u1 = surface.LastUParameter();
                double v0 = surface.FirstVParameter(), v1 = surface.LastVParameter();
                // The contacts are the boundaries in whichever direction lies on the two surfaces
                double Off(bool acrossU) => Enumerable.Range(0, 41).Max(k =>
                {
                    Point3D a = acrossU ? surface.Value(u0, v0 + (v1 - v0) * k / 40) : surface.Value(u0 + (u1 - u0) * k / 40, v0);
                    Point3D b = acrossU ? surface.Value(u1, v0 + (v1 - v0) * k / 40) : surface.Value(u0 + (u1 - u0) * k / 40, v1);
                    return Math.Min(Math.Max(first(a), second(b)), Math.Max(second(a), first(b)));
                });
                bool acrossU = Off(true) <= Off(false);
                offContact = Math.Max(offContact, Off(acrossU));
                for (int a = 0; a <= 40; a++)
                {
                    (double Miss, double Radius, double Other) Ball(int b, double from, double to)
                    {
                        double across = b / 8.0, along = a / 40.0;
                        double u = acrossU ? u0 + (u1 - u0) * across : u0 + (u1 - u0) * along;
                        double v = acrossU ? v0 + (v1 - v0) * along : v0 + (v1 - v0) * across;
                        surface.D1(u, v, out Point3D p, out Vector3D du, out Vector3D dv);
                        Vector3D n = du.Crossed(dv);
                        n = n.Divided(n.Magnitude);
                        return new[] { 1.0, -1.0 }.Select(side =>
                        {
                            (double m, double r) = BallOnNormal(p, side * n, first, second, from, to);
                            return (m, r, Math.Max(TouchingMiss(p, side * n, first, second, lower, upper),
                                                   TouchingMiss(p, side * n, second, first, lower, upper)));
                        }).MinBy(x => x.m);
                    }
                    var inside = Enumerable.Range(1, 7).Select(b => Ball(b, lower, upper)).ToArray();
                    other = Math.Max(other, inside.Max(x => x.Other));
                    // On a contact the normal is the surface's own, and balls of other radii along it touch both surfaces
                    // elsewhere: the one meant is the one beside it
                    (double Miss, double Radius, double Other)[] balls = [.. inside, Ball(0, 0.98 * inside[0].Radius, 1.02 * inside[0].Radius),
                                                            Ball(8, 0.98 * inside[6].Radius, 1.02 * inside[6].Radius)];
                    worst = Math.Max(worst, balls.Max(x => x.Miss));
                    least = Math.Min(least, balls.Min(x => x.Radius));
                    greatest = Math.Max(greatest, balls.Max(x => x.Radius));
                }
            }
        }
        return (worst, other, offContact, least, greatest);
    }

    // The ball centred on the line from p along n that touches the first surface, its radius between the bounds (its centre's
    // distance from that surface less the radius only falls as the radius grows): what it misses the second by
    private static double TouchingMiss(Point3D p, Vector3D n, Distance touched, Distance other, double lower, double upper)
    {
        Point3D At(double rho) => new(p.X + rho * n.X, p.Y + rho * n.Y, p.Z + rho * n.Z);
        double lo = lower, hi = upper;
        if (touched(At(lo)) - lo < 0 || touched(At(hi)) - hi > 0)
        {
            return double.MaxValue;
        }
        for (int k = 0; k < 60; k++)
        {
            double mid = (lo + hi) / 2;
            if (touched(At(mid)) - mid > 0)
            {
                lo = mid;
            }
            else
            {
                hi = mid;
            }
        }
        return Math.Abs(other(At(lo)) - lo);
    }

    // The ball centred on the line from p along n that best touches both surfaces: its radius rho, between the bounds, the one
    // that makes the centre's distances from the two surfaces nearest to rho (the sum of the squares of what they miss by falls
    // and then rises with rho, as each distance grows more slowly than rho does), and the most either misses by
    private static (double Miss, double Radius) BallOnNormal(Point3D p, Vector3D n, Distance first, Distance second, double lower, double upper)
    {
        (double A, double B) At(double rho)
        {
            Point3D c = new(p.X + rho * n.X, p.Y + rho * n.Y, p.Z + rho * n.Z);
            return (first(c) - rho, second(c) - rho);
        }
        double Cost(double rho)
        {
            (double a, double b) = At(rho);
            return a * a + b * b;
        }
        const double Golden = 0.6180339887498949;
        double lo = lower, hi = upper, x1 = hi - Golden * (hi - lo), x2 = lo + Golden * (hi - lo), f1 = Cost(x1), f2 = Cost(x2);
        for (int k = 0; k < 80; k++)
        {
            if (f1 < f2)
            {
                (hi, x2, f2) = (x2, x1, f1);
                x1 = hi - Golden * (hi - lo);
                f1 = Cost(x1);
            }
            else
            {
                (lo, x1, f1) = (x1, x2, f2);
                x2 = lo + Golden * (hi - lo);
                f2 = Cost(x2);
            }
        }
        double rho = (lo + hi) / 2;
        (double a, double b) = At(rho);
        return (Math.Max(Math.Abs(a), Math.Abs(b)), rho);
    }

    private static double Nearer(double a, double b, double radius) => Math.Abs(a - radius) <= Math.Abs(b - radius) ? a : b;

    // The distance from the circle that ends the branch at the angle: radius 3, at 9 along its axis
    private static double EndCircle(Point3D p, double degrees)
    {
        Direction3D d = new(0, Math.Sin(degrees * Math.PI / 180), Math.Cos(degrees * Math.PI / 180));
        double rho = Rho(p, new Point3D(0, 0, 0), d, out double s);
        return Math.Sqrt((rho - 3) * (rho - 3) + (s - 9) * (s - 9));
    }

    // The volume added is the slices' region, to what the path's tolerance (decision 1898's confusion) moves its faces by over
    // their area, and the slices' own error
    private static void AddsTheRegion(Shape tee, Shape shape, double radius)
    {
        double added = shape.IntegrateVolume(Precise) - tee.IntegrateVolume(Precise);
        double area = shape.AllFaces().Where(f => new FaceSurfaceAdaptor(f).GetType() is not (SurfaceKind.Plane or SurfaceKind.Cylinder))
                           .Sum(f => ((Shape)f).IntegrateArea(Precise));
        double expected = TeeRegion(radius);
        Assert.Equal(expected, added, 2e-7 * area + 1e-5 * expected);
    }

    // The most a ball of the radius on either side of a fillet's faces misses either surface by, at 39 by 7 points of each
    private static double BallMiss(MakeFillet fillet, double radius, Func<Point3D, double> first, Func<Point3D, double> second)
    {
        double worst = 0;
        for (int i = 1; i <= fillet.SurfaceCount; i++)
        {
            foreach (Shape face in fillet.NewFaces(i))
            {
                var surface = new FaceSurfaceAdaptor((Face)face);
                for (int a = 1; a < 40; a++)
                {
                    for (int b = 1; b < 8; b++)
                    {
                        double u = surface.FirstUParameter() + (surface.LastUParameter() - surface.FirstUParameter()) * a / 40;
                        double v = surface.FirstVParameter() + (surface.LastVParameter() - surface.FirstVParameter()) * b / 8;
                        surface.D1(u, v, out Point3D p, out Vector3D du, out Vector3D dv);
                        Vector3D n = du.Crossed(dv);
                        n = n.Divided(n.Magnitude).Multiplied(radius);
                        Point3D outside = new(p.X + n.X, p.Y + n.Y, p.Z + n.Z), inside = new(p.X - n.X, p.Y - n.Y, p.Z - n.Z);
                        worst = Math.Max(worst, Math.Min(Math.Max(Math.Abs(first(outside) - radius), Math.Abs(second(outside) - radius)),
                                                         Math.Max(Math.Abs(first(inside) - radius), Math.Abs(second(inside) - radius))));
                    }
                }
            }
        }
        return worst;
    }

    // On every face that isn't a plane or a cylinder: a ball of the radius on the face's outward side where the edge is concave, or
    // inward where it's convex (side -1), is at the radius from both cylinders' surfaces
    private static void BallTouchesBoth(Shape shape, double radius, double side = 1)
    {
        int faces = 0;
        foreach (Face face in shape.AllFaces())
        {
            var surface = new FaceSurfaceAdaptor(face);
            if (surface.GetType() is SurfaceKind.Plane or SurfaceKind.Cylinder)
            {
                continue;
            }
            faces++;
            double sign = face.Orientation == Orientation.Reversed ? -1 : 1;
            for (int i = 1; i < 40; i++)
            {
                for (int j = 1; j < 8; j++)
                {
                    double u = surface.FirstUParameter() + (surface.LastUParameter() - surface.FirstUParameter()) * i / 40;
                    double v = surface.FirstVParameter() + (surface.LastVParameter() - surface.FirstVParameter()) * j / 8;
                    surface.D1(u, v, out Point3D p, out Vector3D du, out Vector3D dv);
                    Vector3D n = du.Crossed(dv);
                    n = n.Divided(n.Magnitude).Multiplied(sign * side * radius);
                    double x = p.X + n.X, y = p.Y + n.Y, z = p.Z + n.Z;
                    Assert.Equal(5 + side * radius, Math.Sqrt(x * x + y * y), 1e-6);
                    Assert.Equal(5 + side * radius, Math.Sqrt(x * x + z * z), 1e-6);
                }
            }
        }
        Assert.Equal(4, faces);
    }

    // The volume the fillet adds, apart from how it's built: in each slice x = 5 cos phi the two cylinders are strips |y|, |z| < a,
    // a = 5 sin phi, meeting at four corners; a point outside both in a corner is added if every ball of the radius through it
    // reaches into a cylinder, that is, if it's further than the radius from every centre clear of both (outside both cylinders of
    // 5 + r, at the nearer of their surfaces' exposed parts and the ellipse where they cross). Gauss in phi and across the corner,
    // bisection for the region's edge
    private static double TeeRegion(double r)
    {
        const double Rho = 5;
        double big = Rho + r;
        bool Added(double x, double y, double z)
        {
            if (x * x + y * y >= big * big && x * x + z * z >= big * big)
            {
                return false;
            }
            double best = double.MaxValue;
            double rz = Math.Sqrt(x * x + y * y), ry = Math.Sqrt(x * x + z * z);
            if (rz < big && (big * x / rz) * (big * x / rz) + z * z >= big * big)
            {
                best = Math.Min(best, big - rz);
            }
            if (ry < big && (big * x / ry) * (big * x / ry) + y * y >= big * big)
            {
                best = Math.Min(best, big - ry);
            }
            double F(double t)
            {
                double dx = x - big * Math.Cos(t), dy = y - big * Math.Sin(t), dz = z - big * Math.Sin(t);
                return dx * dx + dy * dy + dz * dz;
            }
            double t0 = Math.Atan2((y + z) / Math.Sqrt(2), x);
            double lo = Math.Max(0, t0 - 0.6), hi = Math.Min(Math.PI, t0 + 0.6), g = (Math.Sqrt(5) - 1) / 2;
            for (int i = 0; i < 80; i++)
            {
                double m1 = hi - g * (hi - lo), m2 = lo + g * (hi - lo);
                (lo, hi) = F(m1) < F(m2) ? (lo, m2) : (m1, hi);
            }
            return Math.Min(best, Math.Sqrt(F(0.5 * (lo + hi)))) > r;
        }
        double Top(double x, double y, double a)
        {
            if (!Added(x, y, a + 1e-14 * (1 + a)))
            {
                return a;
            }
            double lo = a, hi = a + 4 * r;
            for (int i = 0; i < 60; i++)
            {
                double m = 0.5 * (lo + hi);
                (lo, hi) = Added(x, y, m) ? (m, hi) : (lo, m);
            }
            return 0.5 * (lo + hi);
        }
        (double[] nodes, double[] weights) = Gauss(96);
        double total = 0;
        for (int i = 0; i < nodes.Length; i++)
        {
            double phi = 0.5 * Math.PI * (nodes[i] + 1), x = Rho * Math.Cos(phi), a = Rho * Math.Sin(phi);
            double lo = a, hi = a + 4 * r;
            for (int k = 0; k < 60; k++)
            {
                double m = 0.5 * (lo + hi);
                (lo, hi) = Top(x, m, a) > a ? (m, hi) : (lo, m);
            }
            double end = 0.5 * (lo + hi), area = 0;
            for (int j = 0; j < nodes.Length; j++)
            {
                double y = a + (end - a) * 0.5 * (nodes[j] + 1);
                area += weights[j] * 0.5 * (end - a) * (Top(x, y, a) - a);
            }
            total += weights[i] * 0.5 * Math.PI * a * area;
        }
        return 4 * total;
    }

    // Gauss-Legendre's points and weights on [-1, 1]
    private static (double[] Nodes, double[] Weights) Gauss(int n)
    {
        var x = new double[n];
        var w = new double[n];
        for (int i = 0; i < n; i++)
        {
            double z = Math.Cos(Math.PI * (i + 0.75) / (n + 0.5)), dp = 0;
            for (int iteration = 0; iteration < 100; iteration++)
            {
                double p0 = 1, p1 = z;
                for (int k = 2; k <= n; k++)
                {
                    (p0, p1) = (p1, ((2 * k - 1) * z * p1 - (k - 1) * p0) / k);
                }
                dp = n * (z * p1 - p0) / (z * z - 1);
                double step = p1 / dp;
                z -= step;
                if (Math.Abs(step) < 1e-16)
                {
                    break;
                }
            }
            x[i] = z;
            w[i] = 2 / ((1 - z * z) * dp * dp);
        }
        return (x, w);
    }
}
