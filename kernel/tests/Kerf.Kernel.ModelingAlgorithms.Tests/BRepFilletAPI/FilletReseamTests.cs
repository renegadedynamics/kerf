// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
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
/// Fillet stage 4b, slice 2 (unit 15T3): ChFi3d fails where an edge it's asked for ends at a vertex on a cylinder's seam, as on
/// the corpus's bug26655 (OCCT #26655), a tube of radii 10 and 20 with a window through its wall whose corners lie on both
/// cylinders' seams. The seam is moved round and ChFi3d asked again. Each result is checked here apart from how it was built: the
/// ball on every new face against the tube's faces in closed form, tangency across every edge of those faces, and the volume
/// against ChFi3d's own on the same solid made with its seams away from the window.
/// </summary>
public class FilletReseamTests
{
    private static readonly MassOptions Precise = new() { Tolerance = 1e-12 };

    // What ChFi3d's approximation is asked for (its tolapp3d): its walked faces are within this of the ball
    private const double Approximation = 1e-4;

    // bug26655's tube: radii 10 and 20, 30 high, a window through its wall where x > 0, y < 0 and 10 < z < 20; the cylinders'
    // seams at the angle given (bug26655's at 0, through two of the window's sides)
    private static Shape Tube(double seam)
    {
        var x = new Direction3D(Math.Cos(seam), Math.Sin(seam), 0);
        var frame = new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), x);
        return Solid.Cylinder(20, 30, frame).Value.Shape.Cut(Solid.Cylinder(10, 30, frame).Value.Shape).Value.Shape
                    .Cut(Solid.Box(new Point3D(0, -25, 10), new Point3D(25, 0, 20)).Value.Shape).Value.Shape;
    }

    // The window's twelve edges: its four concave corners, and its rims on both cylinders
    private static Edge[] Window(Shape tube) =>
        [.. tube.AllEdges().Where(e =>
        {
            var curve = new EdgeCurveAdaptor(e);
            Point3D p = curve.Value(0.5 * (curve.FirstParameter() + curve.LastParameter()));
            bool onLevel = Math.Abs(p.Z - 10) < 1e-6 || Math.Abs(p.Z - 20) < 1e-6, onSide = Math.Abs(p.X) < 1e-6 || Math.Abs(p.Y) < 1e-6;
            return p.Z > 10 - 1e-6 && p.Z < 20 + 1e-6 && p.X > -1e-6 && p.Y < 1e-6 && (onLevel || onSide);
        })];

    private static Shape? Corpus(string file)
    {
        string path = Path.Combine(typeof(FilletReseamTests).Assembly.GetCustomAttributes(typeof(AssemblyMetadataAttribute), false)
                                       .Cast<AssemblyMetadataAttribute>().Single(a => a.Key == "OcctTestDataDir").Value!, "brep", file);
        if (!File.Exists(path))
        {
            return null;
        }
        return BrepTools.Read(out Shape shape, path) ? shape : null;
    }

    [Fact]
    public void Bug26655_builds_where_ChFi3d_fails()
    {
        if (Corpus("bug26655_OCC50.brep") is not { } input)
        {
            return;
        }
        Edge[] edges = Window(input);
        Assert.Equal(12, edges.Length);
        Proved(input, edges, 1);
    }

    [Theory]
    [InlineData(0.5)]
    [InlineData(1)]
    [InlineData(2)]
    public void The_tube_with_its_seams_through_the_window_builds_where_ChFi3d_fails(double radius)
    {
        Shape input = Tube(0);
        Edge[] edges = Window(input);
        Assert.Equal(12, edges.Length);
        Proved(input, edges, radius);
    }

    [Fact]
    public void Where_no_seam_ends_at_the_edges_the_stage_isnt_asked()
    {
        Shape input = Tube(3 * Math.PI / 4);
        Assert.Null(FilletLimits.ReseamedForTests(input, [.. Window(input).Select(e => new FilletSpec(e, 1))]));
    }

    [Fact]
    public void The_tube_remade_is_the_same_solid()
    {
        Shape input = Tube(0);
        Shape remade = FilletLimits.ReseamedForTests(input, [.. Window(input).Select(e => new FilletSpec(e, 1))])!.Value;
        Assert.True(remade.Validate().IsValid);
        Assert.Equal(input.IntegrateVolume(Precise), remade.IntegrateVolume(Precise), 1e-9);
        Assert.Equal(input.AllFaces().Length, remade.AllFaces().Length);
        foreach (Face face in remade.AllFaces().Where(f => new FaceSurfaceAdaptor(f).GetType() == SurfaceKind.Cylinder))
        {
            // Each cylinder's seam is now clear of the window: no vertex of a seam is one of the window's
            foreach (Edge seam in face.AllEdges().Where(e => Tool.IsClosed(e, face)))
            {
                Assert.All(seam.AllVertices(), v => Assert.True(Tool.Pnt(v).X < 0, "the seam ends at the window"));
            }
        }
    }

    // Unit 15T4's bodies, each with its seams at the angle given, through the cut where it's 0: a hollow cone to its apexes with a
    // window through its wall; a hollow sphere with one; and bug26655's tube with a quarter cut out of its top end
    private static Shape Other(string kind, double seam)
    {
        var frame = new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(Math.Cos(seam), Math.Sin(seam), 0));
        return kind switch
        {
            "cone" => Solid.Cone(10, 0, 20, frame).Value.Shape.Cut(Solid.Cone(7, 0, 14, frame).Value.Shape).Value.Shape
                           .Cut(Solid.Box(new Point3D(0, -25, 5), new Point3D(25, 0, 10)).Value.Shape).Value.Shape,
            "sphere" => Solid.Sphere(10, frame).Value.Shape.Cut(Solid.Sphere(7, frame).Value.Shape).Value.Shape
                             .Cut(Solid.Box(new Point3D(0, -25, -3), new Point3D(25, 0, 3)).Value.Shape).Value.Shape,
            _ => Solid.Cylinder(20, 30, frame).Value.Shape.Cut(Solid.Cylinder(10, 30, frame).Value.Shape).Value.Shape
                      .Cut(Solid.Box(new Point3D(0, -25, 20), new Point3D(25, 0, 31)).Value.Shape).Value.Shape,
        };
    }

    private static Point3D Middle(Edge e)
    {
        var curve = new EdgeCurveAdaptor(e);
        return curve.Value(0.5 * (curve.FirstParameter() + curve.LastParameter()));
    }

    // The edges asked for: the window's twelve, or every edge of the cut end from its floor up (the cut's eight and the top's four);
    // "ends" is the cut's eight alone, four of which end on the top
    private static Edge[] Asked(string kind, Shape shape)
    {
        (double low, double high) = kind == "cone" ? (5, 10) : (-3, 3);
        return [.. shape.AllEdges().Distinct().Where(e =>
        {
            Point3D p = Middle(e);
            if (kind is "slot" or "ends")
            {
                return p.Z > 20 - 1e-6 && (kind == "slot" || p.Z < 30 - 1e-6) && !Tool.Degenerated(e);
            }
            bool onLevel = Math.Abs(p.Z - low) < 1e-6 || Math.Abs(p.Z - high) < 1e-6, onSide = Math.Abs(p.X) < 1e-6 || Math.Abs(p.Y) < 1e-6;
            return p.Z > low - 1e-6 && p.Z < high + 1e-6 && p.X > -1e-6 && p.Y < 1e-6 && (onLevel || onSide);
        })];
    }

    // How far a ball's centre is from touching each of a body's faces, as surfaces, and its concave edges' blends
    private static IEnumerable<double> OtherMisses(string kind, double x, double y, double z, double radius)
    {
        double rho = Math.Sqrt(x * x + y * y);
        double[] levels;
        switch (kind)
        {
            case "cone":
                double cos = 1 / Math.Sqrt(1.25);
                yield return Math.Abs(Math.Abs(rho - (10 - z / 2)) * cos - radius);
                yield return Math.Abs(Math.Abs(rho - (7 - z / 2)) * cos - radius);
                yield return Math.Abs(Math.Abs(z - 5) - radius);
                yield return Math.Abs(Math.Abs(z - 10) - radius);
                levels = [5 + radius, 10 - radius];
                break;
            case "sphere":
                double r = Math.Sqrt(x * x + y * y + z * z);
                yield return Math.Abs(Math.Abs(r - 10) - radius);
                yield return Math.Abs(Math.Abs(r - 7) - radius);
                yield return Math.Abs(Math.Abs(z + 3) - radius);
                yield return Math.Abs(Math.Abs(z - 3) - radius);
                levels = [-3 + radius, 3 - radius];
                break;
            default:
                yield return Math.Abs(Math.Abs(rho - 10) - radius);
                yield return Math.Abs(Math.Abs(rho - 20) - radius);
                yield return Math.Abs(Math.Abs(z - 20) - radius);
                yield return Math.Abs(Math.Abs(z - 30) - radius);
                levels = [20 + radius];
                break;
        }
        yield return Math.Abs(Math.Abs(x) - radius);
        yield return Math.Abs(Math.Abs(y) - radius);
        foreach (double level in levels)
        {
            yield return Math.Abs(Math.Sqrt((x - radius) * (x - radius) + (z - level) * (z - level)) - 2 * radius);
            yield return Math.Abs(Math.Sqrt((y + radius) * (y + radius) + (z - level) * (z - level)) - 2 * radius);
        }
    }

    // Unit 15T4: a seam on a face that isn't a band between two whole circles. To a cone's apex and on a sphere the seam ends at a
    // point where the surface closes; on the cut end of a tube it ends on an arc, which the new seam cuts in two
    [Theory]
    [InlineData("cone", 0.5)]
    [InlineData("sphere", 0.5)]
    [InlineData("slot", 1)]
    public void A_seam_to_an_apex_on_a_sphere_or_to_a_cut_end_is_moved(string kind, double radius)
    {
        Shape input = Other(kind, 0);
        Edge[] edges = Asked(kind, input);
        Assert.Equal(12, edges.Length);

        // The same solid, remade
        Shape remade = FilletLimits.ReseamedForTests(input, [.. edges.Select(e => new FilletSpec(e, radius))])!.Value;
        Assert.True(remade.Validate().IsValid);
        Assert.Equal(input.IntegrateVolume(Precise), remade.IntegrateVolume(Precise), 1e-9);
        Assert.Equal(input.AllFaces().Length, remade.AllFaces().Length);

        // ChFi3d alone fails here
        var chfi3d = new MakeFillet(input);
        foreach (Edge edge in edges)
        {
            chfi3d.Add(radius, edge);
        }
        chfi3d.Build();
        Assert.False(chfi3d.IsDone && chfi3d.Shape().Validate().IsValid);

        long before = FilletLimits.s_reseamBuilt;
        Built<Shape> made = input.Fillet(edges, radius).Value;
        Assert.True(FilletLimits.s_reseamBuilt > before);
        Shape shape = made.Shape;
        Assert.True(shape.Validate().IsValid);
        Assert.Empty(EdgeCrossings.All(input, shape));

        // The new faces: every face but the planes and the body's own cylinders, cones or spheres
        Face[] blends = [.. shape.AllFaces().Where(f =>
        {
            var surface = new FaceSurfaceAdaptor(f);
            return surface.GetType() switch
            {
                SurfaceKind.Plane or SurfaceKind.Cone => false,
                SurfaceKind.Cylinder => Math.Abs(surface.Cylinder().Radius - radius) < 1e-9,
                SurfaceKind.Sphere => Math.Abs(surface.Sphere().Radius - radius) < 1e-9,
                _ => true,
            };
        })];
        Assert.True(blends.Length >= 20);
        double miss = BallMiss(blends, radius, (x, y, z) => OtherMisses(kind, x, y, z, radius));
        Assert.True(miss < Approximation, $"the ball misses by {miss}");
        double turn = WorstTurn(shape, blends);
        Assert.True(turn < 1e-3, $"a new face turns by {turn} across an edge");

        // The volume: ChFi3d's own on the same solid made with its seams clear of the edges
        Shape clear = Other(kind, 3 * Math.PI / 4);
        var reference = new MakeFillet(clear);
        foreach (Edge edge in Asked(kind, clear))
        {
            reference.Add(radius, edge);
        }
        reference.Build();
        Assert.True(reference.IsDone);
        double area = blends.Sum(f => ((Shape)f).IntegrateArea(Precise));
        Assert.Equal(reference.Shape().IntegrateVolume(Precise), shape.IntegrateVolume(Precise), 0.1 * Approximation * area);
        foreach (Edge edge in edges)
        {
            Assert.True(made.History.IsRemoved(edge));
        }
    }

    [Fact]
    public void Where_the_result_with_the_seam_moved_cant_be_proved_the_failure_says_why()
    {
        // The cut end's eight edges alone: four of them end on the top, where a fillet's end isn't tangent to the face it ends on,
        // which the stage's proof doesn't allow for. So it's refused, and the failure names the seam
        Shape input = Other("ends", 0);
        Edge[] edges = Asked("ends", input);
        Assert.Equal(8, edges.Length);
        long refused = FilletLimits.s_reseamRefused;
        Result<Built<Shape>> made = input.Fillet(edges, 1);
        Assert.False(made.IsSuccess);
        Assert.True(FilletLimits.s_reseamRefused > refused);
        Assert.Contains("seam", made.Error!.Message);

        // With the seam elsewhere ChFi3d builds it
        Shape clear = Other("ends", 3 * Math.PI / 4);
        Assert.True(clear.Fillet(Asked("ends", clear), 1).IsSuccess);
    }

    private static void Proved(Shape input, Edge[] edges, double radius)
    {
        // ChFi3d alone fails here
        var chfi3d = new MakeFillet(input);
        foreach (Edge edge in edges)
        {
            chfi3d.Add(radius, edge);
        }
        chfi3d.Build();
        Assert.False(chfi3d.IsDone && chfi3d.Shape().Validate().IsValid);

        long before = FilletLimits.s_reseamBuilt;
        Built<Shape> made = input.Fillet(edges, radius).Value;
        Assert.True(FilletLimits.s_reseamBuilt > before);
        Shape shape = made.Shape;
        Assert.True(shape.Validate().IsValid);
        Assert.Empty(EdgeCrossings.All(input, shape));

        // The new faces: every face but the planes and the tube's two cylinders, twelve along the edges and one at each corner
        Face[] blends = [.. shape.AllFaces().Where(f =>
        {
            var surface = new FaceSurfaceAdaptor(f);
            return surface.GetType() switch
            {
                SurfaceKind.Plane => false,
                SurfaceKind.Cylinder => Math.Abs(surface.Cylinder().Radius - radius) < 1e-9,
                _ => true,
            };
        })];
        Assert.Equal(20, blends.Length);

        // The twelve along the edges, cylinders and tori, are on the ball; the eight at the corners are ChFi3d's corner fillings,
        // B-splines tangent to the blends round them but not the ball's (decision 2210), as in ChFi3d's one step on the same solid
        Face[] along = blends;
        double miss = BallMiss(along, radius, (x, y, z) => Misses(x, y, z, radius));
        Assert.True(miss < Approximation, $"the ball misses by {miss}");
        double turn = WorstTurn(shape, blends);
        Assert.True(turn < 1e-3, $"a new face turns by {turn} across an edge");

        // The volume: ChFi3d's own on the same solid made with its seams clear of the window, to what the approximation moves the
        // new faces by over their area
        Shape clear = Tube(3 * Math.PI / 4);
        var reference = new MakeFillet(clear);
        foreach (Edge edge in Window(clear))
        {
            reference.Add(radius, edge);
        }
        reference.Build();
        Assert.True(reference.IsDone);
        double area = blends.Sum(f => ((Shape)f).IntegrateArea(Precise));
        Assert.Equal(reference.Shape().IntegrateVolume(Precise), shape.IntegrateVolume(Precise), Approximation * area);

        // And the convention's own volume, which owes nothing to ChFi3d (unit 15T4): to a tenth of that
        Assert.Equal(Expected(radius), shape.IntegrateVolume(Precise), 0.1 * Approximation * area);
        foreach (Edge edge in edges)
        {
            Assert.True(made.History.IsRemoved(edge));
        }
    }

    // The volume of the shape the convention defines at these corners, where two convex rims meet one concave edge: the concave
    // edges rounded first, then the rims over them (the tube closed by the ball, then opened; decision 2210). In closed form the
    // tube less its window, 8250 pi; with each concave edge's spandrel through the wall, one integral; less what rounding each
    // opening's rim takes away, the rim then a smooth loop round a window whose corners are rounded, by FilletJunctionIntegral with
    // the material and the space outside it changed over: the ball's centre a radius inside the wall's cylinder and a radius from
    // the rounded window, which is its box shrunk by the radius and grown again
    private static double Expected(double radius)
    {
        double spandrel = FilletJunctionIntegral.Gauss(
            x => (Math.Sqrt(400 - x * x) - Math.Sqrt(100 - x * x)) * (radius - Math.Sqrt(radius * radius - (x - radius) * (x - radius))), 0, radius, 64);
        double FromShrunk(FilletJunctionIntegral.V p)
        {
            double dx = Math.Max(radius - p.X, 0), dy = Math.Max(p.Y + radius, 0), dz = Math.Max(Math.Max(10 + radius - p.Z, p.Z - (20 - radius)), 0);
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }
        double rims = 0;
        foreach ((double wall, double sign) in new[] { (10.0, 1.0), (20.0, -1.0) })
        {
            double rho = wall + sign * radius, half = Math.Sqrt(0.5);
            rims += FilletJunctionIntegral.Added(FromShrunk, p => sign * (Math.Sqrt(p.X * p.X + p.Y * p.Y) - wall), radius,
                                                 new FilletJunctionIntegral.V(rho * half, -rho * half, 10 - radius), radius / 40);
        }
        return 8250 * Math.PI + 4 * spandrel - rims;
    }

    // How far a ball's centre is from touching each of the tube's faces where the ball can touch them, as surfaces: the two cylinders, the window's
    // four planes, and the concave corners' blends, cylinders of the radius about lines a radius in from both planes (a ball touching
    // one from outside it is at twice the radius from its line)
    private static IEnumerable<double> Misses(double x, double y, double z, double radius)
    {
        double rho = Math.Sqrt(x * x + y * y);
        yield return Math.Abs(Math.Abs(rho - 10) - radius);
        yield return Math.Abs(Math.Abs(rho - 20) - radius);
        yield return Math.Abs(Math.Abs(z - 10) - radius);
        yield return Math.Abs(Math.Abs(z - 20) - radius);
        yield return Math.Abs(Math.Abs(x) - radius);
        yield return Math.Abs(Math.Abs(y) - radius);
        foreach (double level in new[] { 10 + radius, 20 - radius })
        {
            yield return Math.Abs(Math.Sqrt((x - radius) * (x - radius) + (z - level) * (z - level)) - 2 * radius);
            yield return Math.Abs(Math.Sqrt((y + radius) * (y + radius) + (z - level) * (z - level)) - 2 * radius);
        }
    }

    // The most a ball on either side of a new face misses touching two of the tube's faces, at points inside each: a concave corner's
    // ball touches its two planes, a rim's the cylinder and a plane, or the cylinder and a concave corner's blend
    private static double BallMiss(Face[] faces, double radius, Func<double, double, double, IEnumerable<double>> all)
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
                    n = n.Divided(n.Magnitude).Multiplied(radius);
                    double best = double.MaxValue;
                    foreach (double sign in (ReadOnlySpan<double>)[1, -1])
                    {
                        double[] misses = [.. all(p.X + sign * n.X, p.Y + sign * n.Y, p.Z + sign * n.Z).Order()];
                        best = Math.Min(best, misses[1]);
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
}
