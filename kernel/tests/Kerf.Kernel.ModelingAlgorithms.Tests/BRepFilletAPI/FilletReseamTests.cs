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
        // B-splines tangent to the blends round them but not the ball's (decision 31303), as in ChFi3d's one step on the same solid
        Face[] along = blends;
        double miss = BallMiss(along, radius);
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
        foreach (Edge edge in edges)
        {
            Assert.True(made.History.IsRemoved(edge));
        }
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
    private static double BallMiss(Face[] faces, double radius)
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
                        double[] misses = [.. Misses(p.X + sign * n.X, p.Y + sign * n.Y, p.Z + sign * n.Z, radius).Order()];
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
