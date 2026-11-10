// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
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

    // ChFi3d's own fillets on the tee are on the ball (decision 30001). 15R found them off it, as OCCT's are (decision 2141;
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
            _ => throw new ArgumentException(name),
        };
    }

    // Each pair at two radii. Left out, for the next slice of unit 15W: the branch at 60 degrees at 1, whose fillet runs onto the
    // branch's end, where ChFi3d blends a surface and that end's edge with BlendFunc_SurfRstConstRad, which has the same plane; and
    // the branch at 45 degrees at 0.3, where ChFi3d's approximation fails as it does at 0.05 to 0.2 without the change.
    public static TheoryData<string, double> Pairs() => new()
    {
        { "tee 5/5", 0.3 }, { "tee 5/5", 1 }, { "tee 5/4", 0.3 }, { "tee 5/4", 1 }, { "tee 5/3", 0.3 }, { "tee 5/3", 1 },
        { "tee 5/2", 0.3 }, { "tee 5/2", 1 }, { "branch 3 at 60", 0.3 }, { "branch 3 at 60", 0.6 }, { "branch 3 at 45", 0.6 },
        { "branch 3 at 45", 1 }, { "branch 2 offset by 2", 0.3 }, { "branch 2 offset by 2", 1 }, { "cone into a cylinder", 0.3 },
        { "cone into a cylinder", 1 }, { "cylinder through a sphere's centre", 0.3 }, { "cylinder through a sphere's centre", 1 },
        { "cylinder through a sphere off its centre", 0.3 }, { "cylinder through a sphere off its centre", 1 },
        { "cylinder through a torus's tube", 0.3 }, { "cylinder through a torus's tube", 1 }, { "cylinder tilted through a plane", 0.3 },
        { "cylinder tilted through a plane", 1 }, { "upright cylinder on a plane", 0.3 }, { "upright cylinder on a plane", 1 },
        { "cone on a cylinder's axis", 0.3 }, { "cone on a cylinder's axis", 1 },
    };

    // 15N's ball on ChFi3d's walked fillets between 15R's pairs (decision 30001): within the approximation's tolerance everywhere.
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
        double miss = BallMiss(chfi3d, radius, p => first(p), p => second(p));
        Assert.True(miss < Approximation, $"{name} at {radius}: the ball misses by {miss}");
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
