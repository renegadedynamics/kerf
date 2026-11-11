// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>
/// Fillet stage 4c (unit 15X): where a contour crosses a cylinder of exactly the fillet's radius with the ball inside it, the ball
/// can't move, and the fillet there is a piece of a sphere. Where the cylinder's axis is square to the face the contour runs along,
/// ChFi3d builds that exactly (an analytic sphere); where it isn't, ChFi3d fails (decision 2159) and this stage builds it. Each
/// result is checked apart from how it was built: its volume against a closed form or a separate integral (the straight stretches'
/// spandrels, and at each pin the region between the ball and the faces, along the rays from its centre), and the ball put back at
/// points of every new face inside the input, touching it.
/// </summary>
public class FilletSphereTests
{
    private static readonly MassOptions Precise = new() { Tolerance = 1e-12 };

    private static Shape Rounded(Shape shape, IEnumerable<Edge> edges, double radius) => shape.Fillet(edges, radius).Value.Shape;

    private static bool Upright(Edge e) => Math.Abs(e.AllVertices()[0].Point.Z - e.AllVertices()[^1].Point.Z) > 1e-9;

    private static bool At(Edge e, Func<Point3D, bool> where) => e.AllVertices().All(v => where(v.Point));

    private static bool Straight(Edge e) => new EdgeCurveAdaptor(e).GetType() == CurveKind.Line;

    // The spandrel's area beside a straight stretch between planes at an angle inside of theta, and what a pin takes where the
    // cylinder's axis is square to the face: a quarter column of the radius, a radius high, less an eighth of the ball
    private static double Spandrel(double r, double theta = Math.PI / 2) => r * r * (1 / Math.Tan(theta / 2) - (Math.PI - theta) / 2);

    private static double Corner(double r) => Math.PI * r * r * r / 4 - Math.PI * r * r * r / 6;

    // What this stage makes, asked directly as after ChFi3d's failure
    private static Shape Staged(Shape shape, Edge edge, double radius)
    {
        EngineOutcome? outcome = FilletLimits.AfterAlgorithm(shape, [new FilletSpec(edge, radius)], FilletOptions.Default);
        Assert.NotNull(outcome);
        return outcome.Value.Shape();
    }

    // At points of each face this stage made, the ball put back along its normal touches the input's faces at the radius: inside
    // the input on a convex edge (side -1), outside it on a concave one. Faces a contour ends square on are left out, as the ball
    // runs through them at its end
    private static void OnTheBall(Shape input, Shape made, double radius, double side, Func<Point3D, bool>? endsOn = null)
    {
        var builder = new BrepBuilder();
        var boundary = new Compound();
        builder.MakeCompound(ref boundary);
        foreach (Face f in input.AllFaces().Where(f => endsOn is null || !f.AllVertices().All(v => endsOn(v.Point))))
        {
            builder.Add(boundary, f);
        }
        Face[] faces = [.. made.AllFaces().Where(f => !input.AllFaces().Any(g => g.IsSame(f)) && Tool.Surface(f) is BSplineSurface)];
        Assert.NotEmpty(faces);
        foreach (Face face in faces)
        {
            var surface = new FaceSurfaceAdaptor(face);
            (double u0, double u1, double v0, double v1) = (surface.FirstUParameter(), surface.LastUParameter(), surface.FirstVParameter(), surface.LastVParameter());
            foreach (double a in (ReadOnlySpan<double>)[0.2, 0.5, 0.8])
            {
                foreach (double b in (ReadOnlySpan<double>)[0.25, 0.5, 0.75])
                {
                    surface.D1(u0 + (u1 - u0) * a, v0 + (v1 - v0) * b, out Point3D p, out Vector3D du, out Vector3D dv);
                    Vector3D n = du.Crossed(dv);
                    n *= (face.Orientation == Orientation.Reversed ? -1 : 1) / n.Magnitude;
                    var vertex = new Vertex();
                    builder.MakeVertex(ref vertex, new Point3D(p.X + side * n.X * radius, p.Y + side * n.Y * radius, p.Z + side * n.Z * radius), 1e-7);
                    var distance = new DistanceShapeShape(vertex, boundary);
                    Assert.True(distance.IsDone);
                    Assert.Equal(radius, distance.Value(), 1e-6 * Math.Max(1, radius));
                }
            }
        }
    }

    private static void Sound(Shape input, Shape made)
    {
        Assert.True(made.Validate().IsValid);
        Assert.Empty(EdgeCrossings.All(input, made));
    }

    [Theory]
    [InlineData(40, 30, 20, 3)]
    [InlineData(4, 3, 2, 0.3)]
    [InlineData(4000, 3000, 2000, 300)]
    public void A_blocks_foot_rounded_at_its_corners_radius_has_spheres_at_the_corners(double w, double d, double h, double r)
    {
        // A block with its upright edges rounded at r, its foot then rounded at r all round: at each corner the ball stands still
        Shape block = Solid.Box(w, d, h).Value.Shape;
        Shape rounded = Rounded(block, block.AllEdges().Where(Upright), r);
        Edge foot = rounded.AllEdges().First(e => !Upright(e) && At(e, p => Math.Abs(p.Z) < 1e-9) && Straight(e));
        double removed = Spandrel(r) * 2 * (w - 2 * r + d - 2 * r) + 4 * Corner(r);
        double expected = rounded.IntegrateVolume(Precise) - removed;

        // ChFi3d's analytic sphere, and this stage's
        Shape chfi3d = rounded.Fillet([foot], r).Value.Shape;
        Assert.Equal(expected, chfi3d.IntegrateVolume(Precise), 1e-9 * removed);
        Shape made = Staged(rounded, foot, r);
        Sound(rounded, made);
        Assert.Equal(18, made.AllFaces().Length);
        Assert.Equal(expected, made.IntegrateVolume(Precise), 1e-6 * removed);
        OnTheBall(rounded, made, r, -1);
    }

    [Fact]
    public void A_pockets_floor_rounded_at_its_corners_radius_has_spheres_in_the_corners()
    {
        // The concave case: a pocket whose upright edges are rounded at r, its floor then rounded at r; the ball sits in each corner
        const double r = 2;
        Shape pocket = Solid.Box(40, 30, 20).Value.Shape
                            .Cut(Solid.Box(20, 10, 20, new Frame3D(new Point3D(10, 10, 5), Direction3D.D.Z, Direction3D.D.X)).Value.Shape).Value.Shape;
        Edge[] corners = [.. pocket.AllEdges().Where(e => Upright(e) && At(e, p => p.X > 5 && p.X < 35 && p.Y > 5 && p.Y < 25))];
        Assert.Equal(4, corners.Length);
        Shape rounded = Rounded(pocket, corners, r);
        Edge floor = rounded.AllEdges().First(e => !Upright(e) && At(e, p => Math.Abs(p.Z - 5) < 1e-9) && Straight(e));
        double added = Spandrel(r) * 2 * (20 - 2 * r + 10 - 2 * r) + 4 * Corner(r);
        double expected = rounded.IntegrateVolume(Precise) + added;

        Assert.Equal(expected, rounded.Fillet([floor], r).Value.Shape.IntegrateVolume(Precise), 1e-9 * added);
        Shape made = Staged(rounded, floor, r);
        Sound(rounded, made);
        Assert.Equal(expected, made.IntegrateVolume(Precise), 1e-6 * added);
        OnTheBall(rounded, made, r, 1);
    }

    [Fact]
    public void An_end_face_round_a_rounded_edge_of_its_radius_ends_square()
    {
        // A bar with one long edge rounded at r; its end face's edges beside that edge rounded at r, out to the top and the far side,
        // where the faces are square to the contour
        const double r = 1.5, l = 30, w = 8, h = 6;
        Shape bar = Solid.Box(l, w, h).Value.Shape;
        Shape rounded = Rounded(bar, bar.AllEdges().Where(e => At(e, p => Math.Abs(p.Y) < 1e-9 && Math.Abs(p.Z) < 1e-9)), r);
        Edge side = rounded.AllEdges().First(e => Upright(e) && At(e, p => Math.Abs(p.X - l) < 1e-9 && Math.Abs(p.Y) < 1e-9));
        double removed = Spandrel(r) * (h - r + w - r) + Corner(r);
        double expected = rounded.IntegrateVolume(Precise) - removed;

        Assert.Equal(expected, rounded.Fillet([side], r).Value.Shape.IntegrateVolume(Precise), 1e-9 * removed);
        Shape made = Staged(rounded, side, r);
        Sound(rounded, made);
        Assert.Equal(expected, made.IntegrateVolume(Precise), 1e-6 * removed);
        OnTheBall(rounded, made, r, -1, p => Math.Abs(p.Z - h) < 1e-9 || Math.Abs(p.Y - w) < 1e-9);
    }

    [Theory]
    [InlineData(30)]
    [InlineData(55)]
    public void An_end_face_round_a_rounded_edge_of_its_radius_ends_on_a_slanted_face_beyond_a_concave_corner(double degrees)
    {
        // As above, but the end face's foot runs to a concave corner with a face slanted to it: the fillet is cut by that face's
        // plane, which is carried on past the corner, as blend/complex/C1's is. The slanted end takes the spandrel up to the plane
        const double r = 1.5, l = 30, w = 8, h = 6, reach = 10;
        double phi = degrees * Math.PI / 180, tan = Math.Tan(phi);
        Shape block = Solid.Box(l + reach * Math.Cos(phi), w + reach * Math.Sin(phi), h).Value.Shape;
        Shape notch = Solid.Box(reach * 2, w + 1, h + 2, new Frame3D(new Point3D(l, -1, -1), Direction3D.D.Z, Direction3D.D.X)).Value.Shape;
        Shape wedge = Solid.Box(reach * 2, reach * 2, h + 2, new Frame3D(new Point3D(l, w, -1), new Direction3D(0, 0, 1), new Direction3D(Math.Cos(phi), Math.Sin(phi), 0))).Value.Shape
                           .Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(Math.Sin(phi) * reach * 2, -Math.Cos(phi) * reach * 2, 0))));
        Shape bar = block.Cut(notch).Value.Shape.Cut(wedge).Value.Shape;
        Shape rounded = Rounded(bar, bar.AllEdges().Where(e => At(e, p => Math.Abs(p.Y) < 1e-9 && Math.Abs(p.Z) < 1e-9)), r);
        Edge side = rounded.AllEdges().First(e => Upright(e) && At(e, p => Math.Abs(p.X - l) < 1e-9 && Math.Abs(p.Y) < 1e-9));
        double spandrel = Spandrel(r), mean = r * (10 - 3 * Math.PI) / (3 * (4 - Math.PI));
        double removed = spandrel * (h - r) + Corner(r) + spandrel * (w - r) - tan * spandrel * mean;
        double expected = rounded.IntegrateVolume(Precise) - removed;

        Shape made = Staged(rounded, side, r);
        Sound(rounded, made);
        Assert.Equal(expected, made.IntegrateVolume(Precise), 1e-6 * removed);
        OnTheBall(rounded, made, r, -1, p => Math.Abs(p.Z - h) < 1e-9 || Math.Abs((p.X - l) * Math.Sin(phi) - (p.Y - w) * Math.Cos(phi)) < 1e-9);
    }

    [Theory]
    [InlineData(5)]
    [InlineData(12)]
    public void A_tilted_foot_round_corners_of_its_radius_is_built_where_ChFi3d_fails(double degrees)
    {
        // The block rounded at its upright edges, its foot cut on a slope across y: the corners' axes aren't square to it, so
        // ChFi3d fails, and the volume is checked against the spandrels and, at each corner, the rays from the ball's centre
        const double r = 3, w = 40, d = 30, h = 20;
        double beta = degrees * Math.PI / 180;
        Shape block = Solid.Box(w, d, h).Value.Shape;
        Shape rounded = Rounded(block, block.AllEdges().Where(Upright), r);
        var up = new Vector3D(0, -Math.Sin(beta), Math.Cos(beta));
        var along = new Vector3D(0, Math.Cos(beta), Math.Sin(beta));
        var through = new Point3D(w / 2, d / 2, 4);
        Point3D origin = through.Translated(up * -50 + new Vector3D(-w, 0, 0) + along * -d * 2);
        Shape tool = Solid.Box(3 * w, 4 * d, 50, new Frame3D(origin, new Direction3D(up.X, up.Y, up.Z), new Direction3D(1, 0, 0))).Value.Shape;
        Shape tilted = rounded.Cut(tool).Value.Shape;
        Edge foot = tilted.AllEdges().First(e => At(e, p => Math.Abs(p.X) < 1e-9) && Straight(e)
                                               && At(e, p => Math.Abs((p.X - through.X) * up.X + (p.Y - through.Y) * up.Y + (p.Z - through.Z) * up.Z) < 1e-9));

        var chfi3d = new MakeFillet(tilted);
        chfi3d.Add(r, foot);
        chfi3d.Build();
        Assert.False(chfi3d.IsDone);
        Built<Shape> made = tilted.Fillet([foot], r).Value;
        Sound(tilted, made.Shape);
        Assert.True(made.History.IsRemoved(foot));
        Assert.NotEmpty(made.History.Generated(foot));

        // The balls' centres on the corners' axes, at r above the foot; the stretches between them, at the angle each wall makes
        // with the foot; the corners' regions by rays
        Point3D Centre(double x, double y) => new(x, y, through.Z + (r + Math.Sin(beta) * (y - through.Y)) / Math.Cos(beta));
        Point3D[] centres = [Centre(r, r), Centre(w - r, r), Centre(w - r, d - r), Centre(r, d - r)];
        double removed = Spandrel(r) * 2 * (d - 2 * r) / Math.Cos(beta) + (Spandrel(r, Math.PI / 2 - beta) + Spandrel(r, Math.PI / 2 + beta)) * (w - 2 * r);
        double[] azimuths = [Math.PI, 1.5 * Math.PI, 0, 0.5 * Math.PI];
        for (int i = 0; i < 4; i++)
        {
            removed += Rays(centres[i], up, azimuths[i], azimuths[i] + Math.PI / 2, r);
        }
        Assert.Equal(tilted.IntegrateVolume(Precise) - removed, made.Shape.IntegrateVolume(Precise), 1e-5 * removed);
        OnTheBall(tilted, made.Shape, r, -1);
    }

    // The region a pin's ball leaves of a corner whose cylinder is upright about its centre: over the directions from the foot's
    // point straight down the foot's normal to the circle round the axis, from the ball out to the foot or the cylinder, (rho^3 -
    // r^3) / 3 of solid angle
    private static double Rays(Point3D c, Vector3D up, double from, double to, double r)
    {
        ReadOnlySpan<double> nodes = [-0.9602898564975363, -0.7966664774136267, -0.5255324099163290, -0.1834346424956498,
                                      0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363];
        ReadOnlySpan<double> weights = [0.1012285362903763, 0.2223810344533745, 0.3137066458778873, 0.3626837833783620,
                                        0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763];
        Vector3D down = up * -1;
        Vector3D Direction(double azimuth, double w)
        {
            var g = new Vector3D(Math.Cos(azimuth), Math.Sin(azimuth), 0);
            double angle = Math.Acos(Math.Clamp(down.Dot(g), -1, 1));
            return (down * Math.Sin((1 - w) * angle) + g * Math.Sin(w * angle)) / Math.Sin(angle);
        }
        double Rho(Vector3D u)
        {
            double cylinder = r / Math.Sqrt(u.X * u.X + u.Y * u.Y);
            double toward = -u.Dot(up);
            return toward > 0 ? Math.Min(cylinder, r / toward) : cylinder;
        }
        const int panels = 48;
        double sum = 0, h = 1e-6;
        for (int i = 0; i < panels; i++)
        {
            for (int a = 0; a < nodes.Length; a++)
            {
                double s = (i + 0.5 + 0.5 * nodes[a]) / panels;
                double azimuth = from + (to - from) * s;
                for (int j = 0; j < panels; j++)
                {
                    for (int b = 0; b < nodes.Length; b++)
                    {
                        double w = (j + 0.5 + 0.5 * nodes[b]) / panels;
                        Vector3D u = Direction(azimuth, w);
                        Vector3D du = (Direction(azimuth + h, w) - Direction(azimuth - h, w)) / (2 * h);
                        Vector3D dw = (Direction(azimuth, w + h) - Direction(azimuth, w - h)) / (2 * h);
                        double rho = Rho(u);
                        sum += weights[a] * weights[b] * 0.25 / (panels * panels) * (to - from) * du.Crossed(dw).Magnitude * (rho * rho * rho - r * r * r) / 3;
                    }
                }
            }
        }
        return sum;
    }

    [Fact]
    public void A_radius_other_than_the_cylinders_isnt_built_here()
    {
        // At 3.001 against corners of 3 the ball isn't pinned: this stage leaves it, whatever ChFi3d makes of it
        Shape block = Solid.Box(40, 30, 20).Value.Shape;
        Shape rounded = Rounded(block, block.AllEdges().Where(Upright), 3);
        Edge foot = rounded.AllEdges().First(e => !Upright(e) && At(e, p => Math.Abs(p.Z) < 1e-9) && Straight(e));
        Assert.Null(FilletLimits.AfterAlgorithm(rounded, [new FilletSpec(foot, 3.001)], FilletOptions.Default));
        Assert.Null(FilletLimits.AfterAlgorithm(rounded, [new FilletSpec(foot, 2.999)], FilletOptions.Default));
    }
}
