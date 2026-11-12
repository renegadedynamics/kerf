// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Classification;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>
/// Fillet stage 4b (unit 15T2): reliability.md's crossing cylinders, every edge rounded, where ChFi3d's one step fails at the four
/// corners where the top rim's convex arc meets the branch's two concave edges. Each result is checked here apart from how it was
/// built: the ball on every new face against the two cylinders' cores, tangency across every edge of those faces, and the volume
/// against the shape the convention defines (decision 2139: convex first, then concave), integrated along rays from the branch's axis.
/// </summary>
public class FilletConvexFirstTests
{
    private static readonly MassOptions Precise = new() { Tolerance = 1e-12 };

    // What ChFi3d's approximation is asked for (its tolapp3d): its walked faces are within this of the ball
    private const double Approximation = 1e-4;

    private const double Main = 5, Half = 5, Length = 10, Ball = 0.3;

    // A cylinder of radius 5 from z = -5 to 5, and through it one of radius rad, 20 long along y, its axis in the top plane; both seams
    // at +x, as Kerf.Modeling's are
    private static Shape Crossing(double rad) =>
        Solid.Cylinder(Main, 2 * Half, new Frame3D(new Point3D(0, 0, -Half), Direction3D.D.Z, Direction3D.D.X)).Value.Shape
             .Fuse(Solid.Cylinder(rad, 2 * Length, new Frame3D(new Point3D(0, -Length, Half), Direction3D.D.Y, Direction3D.D.X)).Value.Shape).Value.Shape;

    [Theory]
    [InlineData(0.5)]
    [InlineData(1)]
    [InlineData(3)]
    [InlineData(4.9)]
    public void The_crossing_cylinders_build_where_ChFi3d_fails(double rad)
    {
        Shape input = Crossing(rad);
        Edge[] edges = [.. input.AllEdges()];

        // ChFi3d alone fails here, or makes something invalid
        var chfi3d = new MakeFillet(input);
        foreach (Edge edge in edges)
        {
            chfi3d.Add(Ball, edge);
        }
        chfi3d.Build();
        Assert.False(chfi3d.IsDone && chfi3d.Shape().Validate().IsValid);

        long before = FilletLimits.s_convexFirstBuilt;
        Built<Shape> made = input.Fillet(edges, Ball).Value;
        Assert.True(FilletLimits.s_convexFirstBuilt > before);
        Shape shape = made.Shape;
        Assert.True(shape.Validate().IsValid);
        Assert.Empty(EdgeCrossings.All(input, shape));

        Face[] blends = [.. shape.AllFaces().Where(f => new FaceSurfaceAdaptor(f).GetType() is not (SurfaceKind.Plane or SurfaceKind.Cylinder))];
        double miss = BallMiss(blends, rad);
        Assert.True(miss < Approximation, $"the ball misses by {miss}");
        double turn = WorstTurn(shape, blends);
        Assert.True(turn < 1e-3, $"a new face turns by {turn} across an edge");

        // The volume, to what the approximation moves the new faces by over their area
        double area = blends.Sum(f => ((Shape)f).IntegrateArea(Precise));
        Assert.Equal(Expected(rad), shape.IntegrateVolume(Precise), Approximation * area);

        // And against unit 15T4's integral of the same convention, along rays from the ball's centres, which has no grid to miss the
        // region's thin runs: to a tenth of that
        Assert.Equal(AlongTheCentres(input, rad), shape.IntegrateVolume(Precise), 0.1 * Approximation * area);

        // Each of the top rim's arcs gave the rim's torus, and is gone
        foreach (Edge arc in edges.Where(e => IsTopArc(e)))
        {
            Assert.Contains(made.History.Generated(arc), s => s.Type == ShapeType.Face && new FaceSurfaceAdaptor((Face)s).GetType() == SurfaceKind.Torus);
            Assert.True(made.History.IsRemoved(arc));
        }
    }

    [Fact]
    public void The_arcs_without_the_concave_edges_are_left_to_ChFi3d()
    {
        // Rounding the rim alone asks for something else: what the rim's blend does under the branch isn't the convention's
        Shape input = Crossing(1);
        FilletSpec[] arcs = [.. input.AllEdges().Where(e => IsTopArc(e)).Select(e => new FilletSpec(e, Ball))];
        Assert.NotEmpty(arcs);
        long refused = FilletLimits.s_convexFirstRefused;
        Assert.Null(FilletLimits.AfterAlgorithm(input, arcs, FilletOptions.Default));
        Assert.Equal(refused, FilletLimits.s_convexFirstRefused);
    }

    [Fact]
    public void A_wrong_shape_for_the_second_step_fails_the_ball()
    {
        // The ball test asked of the result against the input, where the concave blend runs over the rim's torus, which the input
        // doesn't have: it must fail, or it proves nothing
        Shape input = Crossing(1);
        Shape shape = input.Fillet(input.AllEdges(), Ball).Value.Shape;
        Face[] blends = [.. shape.AllFaces().Where(f => new FaceSurfaceAdaptor(f).GetType() == SurfaceKind.BSplineSurface)];
        Assert.NotEmpty(blends);
        Assert.False(FilletLimits.BallOnFacesForTests(input, shape, [.. blends], Ball));
    }

    private static bool IsTopArc(Edge edge)
    {
        var curve = new EdgeCurveAdaptor(edge);
        return curve.GetType() == CurveKind.Circle && Math.Abs(curve.Circle().Location.Z - Half) < 1e-9 && Math.Abs(curve.Circle().Radius - Main) < 1e-9
               && edge.AllVertices().Length == 2;
    }

    // The cores: each cylinder less the ball's radius all round. The rounded cylinder is the points within the radius of its core, and
    // a concave blend's ball is twice the radius from both cores. Signed distances: negative inside
    private static double FromMainCore(double x, double y, double z) => FromCore(Math.Sqrt(x * x + y * y), z, Main - Ball, Half - Ball);

    private static double FromBranchCore(double x, double y, double z, double rad) => FromCore(Math.Sqrt(x * x + (z - Half) * (z - Half)), y, rad - Ball, Length - Ball);

    private static double FromCore(double rho, double along, double radius, double half)
    {
        double dr = rho - radius, dz = Math.Abs(along) - half;
        return dr <= 0 && dz <= 0 ? Math.Max(dr, dz) : Math.Sqrt(Math.Max(dr, 0) * Math.Max(dr, 0) + Math.Max(dz, 0) * Math.Max(dz, 0));
    }

    // The most a ball on either side of a new face misses where it should be, at points inside each: a rim's ball has its centre on a
    // core's surface, a concave blend's at twice the radius from both cores
    private static double BallMiss(Face[] faces, double rad)
    {
        double worst = 0;
        foreach (Face face in faces)
        {
            var surface = new FaceSurfaceAdaptor(face);
            BrepTools.UVBounds(face, out double u0, out double u1, out double v0, out double v1);
            int tried = 0;
            for (int i = 1; i < 20; i++)
            {
                for (int j = 1; j < 8; j++)
                {
                    double u = u0 + (u1 - u0) * i / 20, v = v0 + (v1 - v0) * j / 8;
                    if (new FaceClassifier(face, new Point2D(u, v), 1e-9).State() != Containment.In)
                    {
                        continue;
                    }
                    surface.D1(u, v, out Point3D p, out Vector3D du, out Vector3D dv);
                    Vector3D n = du.Crossed(dv);
                    n = n.Divided(n.Magnitude).Multiplied(Ball);
                    double best = double.MaxValue;
                    foreach (double sign in (ReadOnlySpan<double>)[1, -1])
                    {
                        double x = p.X + sign * n.X, y = p.Y + sign * n.Y, z = p.Z + sign * n.Z;
                        double m = FromMainCore(x, y, z), b = FromBranchCore(x, y, z, rad);
                        best = Math.Min(best, Math.Min(Math.Min(Math.Abs(m), Math.Abs(b)), Math.Max(Math.Abs(m - 2 * Ball), Math.Abs(b - 2 * Ball))));
                    }
                    worst = Math.Max(worst, best);
                    tried++;
                }
            }
            Assert.True(tried > 0);
        }
        return worst;
    }

    // The most any new face turns from the face beside it, across each of its edges
    private static double WorstTurn(Shape shape, Face[] faces)
    {
        var facesOfEdge = new Dictionary<Shape, List<Face>>(ShapeHasher.Instance);
        foreach (Face face in shape.AllFaces())
        {
            foreach (Edge edge in face.AllEdges())
            {
                if (!facesOfEdge.TryGetValue(edge, out List<Face>? list))
                {
                    facesOfEdge[edge] = list = [];
                }
                list.Add(face);
            }
        }
        double worst = 0;
        foreach (Face face in faces)
        {
            foreach (Edge edge in face.AllEdges())
            {
                List<Face> two = facesOfEdge[edge];
                if (two.Count != 2 || two[0].IsSame(two[1]) || Tool.Degenerated(edge))
                {
                    continue;
                }
                Tool.Range(edge, out double a, out double b);
                for (int k = 1; k < 10; k++)
                {
                    double t = a + (b - a) * k / 10;
                    Vector3D n1 = Normal(two[0], edge, t), n2 = Normal(two[1], edge, t);
                    double c = n1.Dot(n2) / (n1.Magnitude * n2.Magnitude);
                    worst = Math.Max(worst, Math.Acos(Math.Clamp(c, -1, 1)));
                }
            }
        }
        return worst;
    }

    private static Vector3D Normal(Face face, Edge edge, double t)
    {
        Point2D uv = Tool.CurveOnSurface(edge, face, out _, out _)!.Value(t);
        new FaceSurfaceAdaptor(face).D1(uv.X, uv.Y, out _, out Vector3D du, out Vector3D dv);
        Vector3D n = du.Crossed(dv);
        return face.Orientation == Orientation.Reversed ? n.Reversed() : n;
    }

    // The volume of the shape the convention defines: each cylinder rounded (the points within the radius of its core), then the
    // concave blend's region where they meet (a point is added if every ball of the radius through it reaches one of them). The
    // rounded cylinders by Pappus, less their common part, and the added region, both integrated along rays from the branch's axis
    private static double Expected(double rad)
    {
        double spandrel = Ball * Ball * (1 - Math.PI / 4), centroid = Ball * (10 - 3 * Math.PI) / (3 * (4 - Math.PI));
        double main = Math.PI * Main * Main * 2 * Half - 2 * 2 * Math.PI * spandrel * (Main - centroid);
        double branch = Math.PI * rad * rad * 2 * Length - 2 * 2 * Math.PI * spandrel * (rad - centroid);
        return main + branch - Common(rad) + Added(rad);
    }

    // The same volume by FilletJunctionIntegral: the input's, less each rim's ring (the top's only where it's clear of the branch;
    // the bottom's and the branch's two ends' whole, by Pappus), with what the concave blend adds round the branch
    private static double AlongTheCentres(Shape input, double rad)
    {
        double spandrel = Ball * Ball * (1 - Math.PI / 4), centroid = Ball * (10 - 3 * Math.PI) / (3 * (4 - Math.PI));
        double Branch(FilletJunctionIntegral.V p) => Math.Sqrt(p.X * p.X + (p.Z - Half) * (p.Z - Half)) - rad;
        return input.IntegrateVolume(Precise) - 2 * Math.PI * spandrel * (Main - centroid) - 2 * 2 * Math.PI * spandrel * (rad - centroid)
               - FilletJunctionIntegral.Ring(Main, Half, Ball, Branch)
               + FilletJunctionIntegral.Added(p => FilletJunctionIntegral.FromCylinder(p, Main - Ball, Half - Ball), Branch, Ball,
                                              new FilletJunctionIntegral.V(rad + Ball, 0, Half + Ball), Ball / 40);
    }

    // Gauss-Legendre on [lo, hi]
    private static readonly double[] Nodes = [-0.9602898564975363, -0.7966664774136267, -0.5255324099163290, -0.1834346424956498,
                                              0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363];
    private static readonly double[] Weights = [0.1012285362903763, 0.2223810344533745, 0.3137066458778873, 0.3626837833783620,
                                                0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763];

    private static double Gauss(Func<double, double> f, double lo, double hi, int panels)
    {
        double sum = 0;
        for (int k = 0; k < panels; k++)
        {
            double a = lo + (hi - lo) * k / panels, b = lo + (hi - lo) * (k + 1) / panels;
            for (int i = 0; i < Nodes.Length; i++)
            {
                sum += Weights[i] * 0.5 * (b - a) * f(0.5 * (a + b) + 0.5 * (b - a) * Nodes[i]);
            }
        }
        return sum;
    }

    // Gauss-Kronrod's 7 and 15 points on [lo, hi], halved where they differ by more than the tolerance
    private static double Adaptive(Func<double, double> f, double lo, double hi, double tolerance, int depth = 0)
    {
        ReadOnlySpan<double> xk = [0.991455371120813, 0.949107912342759, 0.864864423359769, 0.741531185599394, 0.586087235467691,
                                   0.405845151377397, 0.207784955007898, 0];
        ReadOnlySpan<double> wk = [0.022935322010529, 0.063092092629979, 0.104790010322250, 0.140653259715525, 0.169004726639267,
                                   0.190350578064785, 0.204432940075298, 0.209482141084728];
        ReadOnlySpan<double> wg = [0.129484966168870, 0.279705391489277, 0.381830050505119, 0.417959183673469];
        double c = 0.5 * (lo + hi), h = 0.5 * (hi - lo), kronrod = 0, gauss = 0;
        for (int i = 0; i < 8; i++)
        {
            double sum = i == 7 ? f(c) : f(c - h * xk[i]) + f(c + h * xk[i]);
            kronrod += wk[i] * sum;
            if (i % 2 == 1)
            {
                gauss += wg[i / 2] * sum;
            }
        }
        kronrod *= h;
        gauss *= h;
        if (Math.Abs(kronrod - gauss) <= tolerance || depth >= 8)
        {
            return kronrod;
        }
        return Adaptive(f, lo, c, tolerance / 2, depth + 1) + Adaptive(f, c, hi, tolerance / 2, depth + 1);
    }

    // A point at angle theta about the branch's axis, s along it and rho from it
    private static (double X, double Y, double Z) At(double theta, double s, double rho) => (rho * Math.Cos(theta), s, Half + rho * Math.Sin(theta));

    private static double Bisect(Func<double, bool> inside, double from, double to)
    {
        for (int i = 0; i < 60; i++)
        {
            double m = 0.5 * (from + to);
            if (inside(m))
            {
                from = m;
            }
            else
            {
                to = m;
            }
        }
        return 0.5 * (from + to);
    }

    // The rounded cylinders' common part: on a ray going down from the branch's axis, the rounded main is an interval of rho, as it's
    // convex, and the branch is rho up to rad. By symmetry a quarter: x below 0, y above
    private static double Common(double rad)
    {
        bool InMain(double theta, double s, double rho)
        {
            (double x, double y, double z) = At(theta, s, rho);
            return FromMainCore(x, y, z) <= Ball;
        }
        double Ray(double theta, double s)
        {
            // Where the main's distance along the ray is least, then its interval within the branch
            double lo = 0, hi = rad, g = (Math.Sqrt(5) - 1) / 2;
            double F(double rho)
            {
                (double x, double y, double z) = At(theta, s, rho);
                return FromMainCore(x, y, z);
            }
            for (int i = 0; i < 80; i++)
            {
                double m1 = hi - g * (hi - lo), m2 = lo + g * (hi - lo);
                if (F(m1) < F(m2))
                {
                    hi = m2;
                }
                else
                {
                    lo = m1;
                }
            }
            double least = 0.5 * (lo + hi);
            if (F(least) > Ball)
            {
                return 0;
            }
            double a = F(0) <= Ball ? 0 : Bisect(rho => !InMain(theta, s, rho), 0, least);
            double b = F(rad) <= Ball ? rad : Bisect(rho => InMain(theta, s, rho), least, rad);
            return 0.5 * (b * b - a * a);
        }

        // In s, split where the ray's interval stops reaching the branch's surface and where the axis leaves the rounded main
        double Slice(double theta)
        {
            double stop = Half - Ball;
            double side = Bisect(s => InMain(theta, s, rad), 0, Main + Ball);
            double[] cuts = [.. new[] { 0, Math.Min(side, stop), Math.Max(side, stop), Main }.Where(c => c >= 0 && c <= Main).Order()];
            double sum = 0;
            for (int k = 0; k + 1 < cuts.Length; k++)
            {
                if (cuts[k + 1] > cuts[k])
                {
                    sum += Gauss(s => Ray(theta, s), cuts[k], cuts[k + 1], 8);
                }
            }
            return sum;
        }
        return 4 * Gauss(Slice, Math.PI, 1.5 * Math.PI, 24);
    }

    // The region the concave blend adds: points outside both rounded cylinders, further than the radius from every centre clear of
    // both, which is the set where both cores are at least twice the radius away. Its nearest point to a point between them is the
    // nearest on one core's offset if that's clear of the other's, otherwise on the curve where the offsets cross
    private static bool InAdded(double x, double y, double z, double rad)
    {
        double m = FromMainCore(x, y, z), b = FromBranchCore(x, y, z, rad), twice = 2 * Ball;
        if (m <= Ball || b <= Ball || (m >= twice && b >= twice))
        {
            return false;
        }
        if (m < twice)
        {
            (double qx, double qy, double qz) = Offset(x, y, z, Main - Ball, Half - Ball, main: true);
            if (FromBranchCore(qx, qy, qz, rad) >= twice)
            {
                return false;
            }
        }
        if (b < twice)
        {
            (double qx, double qy, double qz) = Offset(x, y, z, rad - Ball, Length - Ball, main: false);
            if (FromMainCore(qx, qy, qz) >= twice)
            {
                return false;
            }
        }
        return Ridge(x, y, z, rad) > Ball;
    }

    // The nearest point to (x, y, z) on a core's offset by twice the radius, from outside the core: along the line from its nearest
    // point on the core
    private static (double, double, double) Offset(double x, double y, double z, double radius, double half, bool main)
    {
        // In the cylinder's own frame: rho about its axis, w along it
        double rx, ry, w;
        if (main)
        {
            (rx, ry, w) = (x, y, z);
        }
        else
        {
            (rx, ry, w) = (x, z - Half, y);
        }
        double rho = Math.Sqrt(rx * rx + ry * ry);
        double pr = Math.Min(rho, radius), pw = Math.Clamp(w, -half, half);
        double fx = rho > 0 ? rx * pr / rho : 0, fy = rho > 0 ? ry * pr / rho : 0;
        double dx = rx - fx, dy = ry - fy, dw = w - pw, d = Math.Sqrt(dx * dx + dy * dy + dw * dw);
        double k = 2 * Ball / d;
        double ox = fx + dx * k, oy = fy + dy * k, ow = pw + dw * k;
        return main ? (ox, oy, ow) : (ox, ow, oy + Half);
    }

    // The distance to the curve where the two offsets cross, the nearest point on it found by Newton's method on the two conditions,
    // each step the least move from the point that meets them as linearised; from the nearest point on the branch's offset
    private static double Ridge(double x, double y, double z, double rad)
    {
        double twice = 2 * Ball;
        (double cx, double cy, double cz) = Offset(x, y, z, rad - Ball, Length - Ball, main: false);
        for (int i = 0; i < 60; i++)
        {
            if (Gradient(cx, cy, cz, Main - Ball, Half - Ball, main: true) is not { } a
                || Gradient(cx, cy, cz, rad - Ball, Length - Ball, main: false) is not { } b)
            {
                return double.MaxValue;
            }
            double g1 = FromMainCore(cx, cy, cz) - twice, g2 = FromBranchCore(cx, cy, cz, rad) - twice;
            double dx = cx - x, dy = cy - y, dz = cz - z;
            double a11 = a.X * a.X + a.Y * a.Y + a.Z * a.Z, a12 = a.X * b.X + a.Y * b.Y + a.Z * b.Z, a22 = b.X * b.X + b.Y * b.Y + b.Z * b.Z;
            double r1 = a.X * dx + a.Y * dy + a.Z * dz - g1, r2 = b.X * dx + b.Y * dy + b.Z * dz - g2;
            double det = a11 * a22 - a12 * a12;
            if (Math.Abs(det) < 1e-14)
            {
                return double.MaxValue;
            }
            double m1 = (r1 * a22 - r2 * a12) / det, m2 = (a11 * r2 - a12 * r1) / det;
            double nx = x + m1 * a.X + m2 * b.X, ny = y + m1 * a.Y + m2 * b.Y, nz = z + m1 * a.Z + m2 * b.Z;
            double step = Math.Sqrt((nx - cx) * (nx - cx) + (ny - cy) * (ny - cy) + (nz - cz) * (nz - cz));
            (cx, cy, cz) = (nx, ny, nz);
            if (step < 1e-13)
            {
                break;
            }
        }
        if (Math.Abs(FromMainCore(cx, cy, cz) - twice) > 1e-9 || Math.Abs(FromBranchCore(cx, cy, cz, rad) - twice) > 1e-9)
        {
            return double.MaxValue;
        }
        return Math.Sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y) + (cz - z) * (cz - z));
    }

    // The gradient of the distance to a core, outside it: the unit vector from its nearest point; null inside
    private static (double X, double Y, double Z)? Gradient(double x, double y, double z, double radius, double half, bool main)
    {
        (double rx, double ry, double w) = main ? (x, y, z) : (x, z - Half, y);
        double rho = Math.Sqrt(rx * rx + ry * ry);
        double pr = Math.Min(rho, radius), pw = Math.Clamp(w, -half, half);
        double dx = rho > 0 ? rx - rx * pr / rho : 0, dy = rho > 0 ? ry - ry * pr / rho : 0, dw = w - pw;
        double d = Math.Sqrt(dx * dx + dy * dy + dw * dw);
        if (d < 1e-12)
        {
            return null;
        }
        return main ? (dx / d, dy / d, dw / d) : (dx / d, dw / d, dy / d);
    }

    // The added region along rays from the branch's axis, by symmetry a quarter, x below 0 and y above. On a ray it's runs of rho, not
    // always from the branch's surface (below the rim's torus it lies past a stretch of the rounded main), each found on a grid and
    // its ends by bisection. In each slice of angle the rays that meet it are found on a grid in s, and integrated over each run of
    // them and a step either side
    private static double Added(double rad)
    {
        const int Samples = 360;
        double reach = 12 * Ball;
        bool Inside(double theta, double s, double rho)
        {
            (double x, double y, double z) = At(theta, s, rho);
            return InAdded(x, y, z, rad);
        }
        double Out(double theta, double s)
        {
            double sum = 0, step = reach / Samples;
            bool was = Inside(theta, s, rad * (1 + 1e-12) + 1e-12);
            double start = rad;
            for (int k = 1; k <= Samples; k++)
            {
                double rho = rad + step * k;
                bool now = Inside(theta, s, rho);
                if (now != was)
                {
                    double edge = now ? Bisect(r => !Inside(theta, s, r), rho - step, rho) : Bisect(r => Inside(theta, s, r), rho - step, rho);
                    if (now)
                    {
                        start = edge;
                    }
                    else
                    {
                        sum += 0.5 * (edge * edge - start * start);
                    }
                    was = now;
                }
            }
            return was ? sum + 0.5 * ((rad + reach) * (rad + reach) - start * start) : sum;
        }
        bool Meets(double theta, double s)
        {
            for (int k = 0; k <= Samples; k++)
            {
                if (Inside(theta, s, rad * (1 + 1e-12) + 1e-12 + reach * k / Samples))
                {
                    return true;
                }
            }
            return false;
        }
        double Slice(double theta)
        {
            const int Steps = 600;
            const double Far = Main + 2 * Ball;
            double sum = 0;
            int k = 0;
            while (k <= Steps)
            {
                if (!Meets(theta, Far * k / Steps))
                {
                    k++;
                    continue;
                }
                int first = k;
                while (k <= Steps && Meets(theta, Far * k / Steps))
                {
                    k++;
                }

                // The run's ends where the rays stop meeting it at all, found on the rays themselves
                double lo = first == 0 ? 0 : Bisect(s => Out(theta, s) == 0, Far * (first - 1) / Steps, Far * first / Steps);
                double hi = k > Steps ? Far : Bisect(s => Out(theta, s) > 0, Far * (k - 1) / Steps, Far * k / Steps);
                sum += Adaptive(s => Out(theta, s), lo, hi, 1e-7);
            }
            return sum;
        }
        double top = Math.Asin(Math.Min(1, Ball / (rad + Ball)));
        return 4 * (Gauss(Slice, Math.PI - top - 0.05, Math.PI - 0.02, 12) + Gauss(Slice, Math.PI - 0.02, Math.PI, 16)
                    + Gauss(Slice, Math.PI, 1.5 * Math.PI, 16));
    }
}
