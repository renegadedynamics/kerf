// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Classification;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;
using V = Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI.FilletJunctionIntegral.V;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>
/// Fillet stage 4b's first step where a rim's convex arc ends on a plane, on a cylinder that holds the material outside it (a hole)
/// or on a cone (unit 15T4; decisions 2196, 2212), with arcs of less and of more than half a turn. The corpus has no such case, so
/// they're made here: a cylinder of radius 5 whose top a rib, a pipe or a cone crosses, half sunk in it. Each result is checked apart
/// from how it was built: the ball on every new face against the bodies in closed form, tangency across every edge of those faces,
/// and the volume against the convention's shape (decision 2139), by <see cref="FilletJunctionIntegral"/>.
/// </summary>
public class FilletConvexFirstEndsTests
{
    private static readonly MassOptions Precise = new() { Tolerance = 1e-12 };

    // What ChFi3d's approximation is asked for (its tolapp3d): its walked faces are within this of the ball
    private const double Approximation = 1e-4;

    private const double Main = 5, Top = 5, Ball = 0.3;

    private static Shape Body() => Solid.Cylinder(Main, 2 * Top, new Frame3D(new Point3D(0, 0, -Top), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))).Value.Shape;

    // Along y, 20 long from y = -10, its surface's seam towards +z, inside whatever it's joined to, or towards +x, along the line
    // where it meets the top
    private static Frame3D AlongY(double x, double z, bool seamOnJunction, double y = -10) =>
        new(new Point3D(x, y, z), new Direction3D(0, 1, 0), seamOnJunction ? new Direction3D(1, 0, 0) : new Direction3D(0, 0, 1));

    // A rib 2w wide whose flat sides run down to z = 4, a half cylinder below them
    private static Shape Rib(double x0, double w) =>
        Body().Fuse(Solid.Box(new Point3D(x0 - w, -10, 4), new Point3D(x0 + w, 10, 7)).Value.Shape
                         .Fuse(Solid.Cylinder(w, 20, AlongY(x0, 4, false)).Value.Shape).Value.Shape).Value.Shape;

    // A pipe whose axis lies in the top
    private static Shape Pipe(double xc, double inner, double outer, bool seamOnJunction) =>
        Body().Fuse(Solid.Cylinder(outer, 20, AlongY(xc, Top, seamOnJunction)).Value.Shape
                         .Cut(Solid.Cylinder(inner, 24, AlongY(xc, Top, seamOnJunction, -12)).Value.Shape).Value.Shape).Value.Shape;

    // A cone whose axis lies in the top, of radius r1 at y = -10 and r2 at y = 10
    private static Shape Cone(double x0, double r1, double r2, bool seamOnJunction) =>
        Body().Fuse(Solid.Cone(r1, r2, 20, AlongY(x0, Top, seamOnJunction)).Value.Shape).Value.Shape;

    private static double Across(V p, double x0, double z0) => Math.Sqrt((p.X - x0) * (p.X - x0) + (p.Z - z0) * (p.Z - z0));

    // A made case: the shape, the edges asked for, and for each closed junction the distance to the body that crosses the top there
    // (negative inside it) with a point near a ball's centre on it
    private sealed record Case(Shape Input, Edge[] Edges, (Func<V, double> Distance, V Near)[] Junctions);

    private static Case Make(string kind, bool seamOnJunction)
    {
        double above = Top + Ball;
        switch (kind)
        {
            case "rib":
            case "rib aside":
            {
                // Aside, the rib leaves one arc of 227 degrees and one of 74
                double x0 = kind == "rib" ? 0 : 3, w = 1;
                Shape input = Rib(x0, w);
                return new Case(input, Junction(input), [(p => p.Z >= 4 ? Math.Abs(p.X - x0) - w : Across(p, x0, 4) - w, new V(x0 + w + Ball, 0, above))]);
            }
            case "bore":
            case "bore aside":
            {
                // Only the arcs inside the bore and the edges round it; aside, the bore holds one arc of 254 degrees
                (double xc, double inner, double outer) = kind == "bore" ? (0, 3, 4) : (-2, 5, 6.5);
                Shape input = Pipe(xc, inner, outer, seamOnJunction);
                Edge[] edges = [.. Junction(input, f => IsCylinder(f, inner)).Where(e => !IsRimArc(e) || Math.Abs(Middle(e).X - xc) < inner)];
                return new Case(input, edges, [(p => inner - Across(p, xc, Top), new V(xc + inner - Ball, 0, above))]);
            }
            case "pipe":
            {
                // The arcs inside the bore and outside the pipe, the edges round both
                Shape input = Pipe(0, 3, 4, seamOnJunction);
                return new Case(input, Junction(input), [(p => 3 - Across(p, 0, Top), new V(3 - Ball, 0, above)), (p => Across(p, 0, Top) - 4, new V(4 + Ball, 0, above))]);
            }
            default:
            {
                // Aside, the cone leaves one arc of about 227 degrees and one of about 74
                (double x0, double r1, double r2) = kind == "cone" ? (0, 1, 2.5) : (3, 0.6, 1.4);
                Shape input = Cone(x0, r1, r2, seamOnJunction);
                double slope = (r2 - r1) / 20, cos = 1 / Math.Sqrt(1 + slope * slope);
                return new Case(input, Junction(input), [(p => (Across(p, x0, Top) - (r1 + (p.Y + 10) * slope)) * cos, new V(x0 + 0.5 * (r1 + r2) + Ball, 0, above))]);
            }
        }
    }

    // How each case comes to the stage. "kept": ChFi3d's one step builds it, on the ball, and the operation keeps that; the stage is
    // asked alone. "instead": ChFi3d's one step is valid, but its faces at a corner aren't tangent to one another, and the operation
    // takes the stage's. "fails": with the crossing body's seam along the junction ChFi3d fails, or makes something invalid, and the
    // operation comes to the stage
    [Theory]
    [InlineData("rib", "kept")]
    [InlineData("rib aside", "instead")]
    [InlineData("bore", "kept")]
    [InlineData("bore", "fails")]
    [InlineData("bore aside", "kept")]
    [InlineData("bore aside", "fails")]
    [InlineData("pipe", "instead")]
    [InlineData("pipe", "fails")]
    [InlineData("cone", "kept")]
    [InlineData("cone", "fails")]
    [InlineData("cone aside", "instead")]
    [InlineData("cone aside", "fails")]
    public void An_arc_ending_on_a_plane_a_hole_or_a_cone_is_built_on_the_ball(string kind, string how)
    {
        Case made = Make(kind, seamOnJunction: how == "fails");
        Shape input = made.Input;
        Assert.True(input.Validate().IsValid);
        Assert.Contains(made.Edges, IsRimArc);
        double expected = Expected(made);

        var chfi3d = new MakeFillet(input);
        foreach (Edge edge in made.Edges)
        {
            chfi3d.Add(Ball, edge);
        }
        chfi3d.Build();
        bool alone = chfi3d.IsDone && chfi3d.Shape().Validate().IsValid;
        long built = FilletLimits.s_convexFirstBuilt, instead = FilletLimits.s_convexFirstInstead;
        Shape shape = input.Fillet(made.Edges, Ball).Value.Shape;
        switch (how)
        {
            case "fails":
                Assert.False(alone);
                Assert.True(FilletLimits.s_convexFirstBuilt > built);
                break;
            case "instead":
                // ChFi3d's own is valid and without crossings, and wrong
                Assert.True(alone);
                Assert.Empty(EdgeCrossings.All(input, chfi3d.Shape()));
                Assert.True(WorstTurn(chfi3d.Shape(), NewFaces(input, chfi3d.Shape(), made.Junctions), betweenNew: true) > 0.1);
                Assert.True(Math.Abs(chfi3d.Shape().IntegrateVolume(Precise) - expected) > 5e-4);
                Assert.True(FilletLimits.s_convexFirstInstead > instead);
                break;
            default:
                // ChFi3d's own is kept, and is right; then the stage asked alone
                Assert.True(alone);
                Assert.Equal(instead, FilletLimits.s_convexFirstInstead);
                Assert.Equal(chfi3d.Shape().IntegrateVolume(Precise), shape.IntegrateVolume(Precise), 1e-9);
                Proved(made, shape, expected);
                built = FilletLimits.s_convexFirstBuilt;
                shape = FilletLimits.AfterAlgorithm(input, [.. made.Edges.Select(e => new FilletSpec(e, Ball))], FilletOptions.Default)!.Value.Shape();
                Assert.True(FilletLimits.s_convexFirstBuilt > built);
                break;
        }
        Proved(made, shape, expected);
    }

    // The volume of the convention's shape: the input's, less the rim's ring clear of the body across it, with what the concave blend
    // adds at each junction
    private static double Expected(Case made)
    {
        double Outside(V p) => made.Junctions.Max(j => j.Distance(p));
        return made.Input.IntegrateVolume(Precise) - FilletJunctionIntegral.Ring(Main, Top, Ball, Outside)
               + made.Junctions.Sum(j => FilletJunctionIntegral.Added(Core, j.Distance, Ball, j.Near, Ball / 40));
    }

    private static void Proved(Case made, Shape shape, double expected)
    {
        Assert.True(shape.Validate().IsValid);
        Assert.Empty(EdgeCrossings.All(made.Input, shape));

        // No arc asked for is left, and every new face is on the ball and tangent to its neighbours
        Assert.DoesNotContain(shape.AllEdges(), e => IsRimArc(e) && made.Edges.Any(a => IsRimArc(a) && Middle(a).DistanceTo(Middle(e)) < 1e-6));
        Face[] blends = NewFaces(made.Input, shape, made.Junctions);
        Assert.True(blends.Length >= 2);
        double miss = BallMiss(blends, [.. made.Junctions.Select(j => j.Distance)]);
        Assert.True(miss < Approximation, $"the ball misses by {miss}");
        double turn = WorstTurn(shape, blends);
        Assert.True(turn < 1e-3, $"a new face turns by {turn} across an edge");

        // The volume, to a tenth of what the approximation moves the new faces by over their area
        double area = blends.Sum(f => ((Shape)f).IntegrateArea(Precise));
        Assert.Equal(expected, shape.IntegrateVolume(Precise), 0.1 * Approximation * area);
    }

    [Fact]
    public void The_ring_is_Pappus_where_nothing_crosses_it()
    {
        double spandrel = Ball * Ball * (1 - Math.PI / 4), centroid = Ball * (10 - 3 * Math.PI) / (3 * (4 - Math.PI));
        Assert.Equal(2 * Math.PI * spandrel * (Main - centroid), FilletJunctionIntegral.Ring(Main, Top, Ball, _ => 1, 4), 1e-12);
    }

    // The rounded cylinder's core: the cylinder less the ball's radius, its foot left far below
    private static double Core(V p) => FilletJunctionIntegral.FromCylinder(p, Main - Ball, Top - Ball);

    private static bool IsTop(Face f)
    {
        var s = new FaceSurfaceAdaptor(f);
        return s.GetType() == SurfaceKind.Plane && Math.Abs(s.Plane().Location.Z - Top) < 1e-9 && Math.Abs(Math.Abs(s.Plane().Axis.Direction.Z) - 1) < 1e-9;
    }

    private static bool IsSide(Face f) => IsCylinder(f, Main) && Math.Abs(Math.Abs(new FaceSurfaceAdaptor(f).Cylinder().Axis.Direction.Z) - 1) < 1e-9;

    private static bool IsCylinder(Face f, double radius)
    {
        var s = new FaceSurfaceAdaptor(f);
        return s.GetType() == SurfaceKind.Cylinder && Math.Abs(s.Cylinder().Radius - radius) < 1e-9;
    }

    private static Point3D Middle(Edge e)
    {
        var c = new EdgeCurveAdaptor(e);
        return c.Value(0.5 * (c.FirstParameter() + c.LastParameter()));
    }

    private static bool IsRimArc(Edge e)
    {
        var c = new EdgeCurveAdaptor(e);
        return c.GetType() == CurveKind.Circle && Math.Abs(c.Circle().Radius - Main) < 1e-9 && Math.Abs(c.Circle().Location.Z - Top) < 1e-9
               && c.Circle().Location.DistanceTo(new Point3D(0, 0, Top)) < 1e-9;
    }

    // The top rim's arcs, and the edges where the cylinder's top and side meet a face of what crosses them, those the filter keeps
    private static Edge[] Junction(Shape shape, Func<Face, bool>? crossing = null)
    {
        var faces = FacesOfEdges(shape);
        var result = new List<Edge>();
        foreach ((Shape edge, List<Face> two) in faces)
        {
            if (two.Count != 2)
            {
                continue;
            }
            bool first = IsTop(two[0]) || IsSide(two[0]), second = IsTop(two[1]) || IsSide(two[1]);
            if (first && second ? IsRimArc((Edge)edge) : first != second && Middle((Edge)edge).Z > -Top + 1e-6 && (crossing is null || crossing(first ? two[1] : two[0])))
            {
                result.Add((Edge)edge);
            }
        }
        return [.. result];
    }

    private static Dictionary<Shape, List<Face>> FacesOfEdges(Shape shape)
    {
        var faces = new Dictionary<Shape, List<Face>>(ShapeHasher.Instance);
        foreach (Face face in shape.AllFaces())
        {
            foreach (Edge edge in face.AllEdges())
            {
                if (!faces.TryGetValue(edge, out List<Face>? list))
                {
                    faces[edge] = list = [];
                }
                if (!list.Any(f => f.IsSame(face)))
                {
                    list.Add(face);
                }
            }
        }
        return faces;
    }

    // A point inside a face, in its parameters
    private static Point2D Inside(Face face)
    {
        BrepTools.UVBounds(face, out double u0, out double u1, out double v0, out double v1);
        foreach (double s in (ReadOnlySpan<double>)[0.5, 0.3, 0.7, 0.1, 0.9])
        {
            foreach (double t in (ReadOnlySpan<double>)[0.5, 0.3, 0.7, 0.1, 0.9])
            {
                var uv = new Point2D(u0 + s * (u1 - u0), v0 + t * (v1 - v0));
                if (new FaceClassifier(face, uv, 1e-9).State() == Containment.In)
                {
                    return uv;
                }
            }
        }
        throw new InvalidOperationException("no point inside the face");
    }

    // The result's faces that aren't on the input: a point inside each is off it, and off the surface of the body across the top,
    // more of which shows where the rim was rounded (a face of its own where a seam cuts it off)
    private static Face[] NewFaces(Shape input, Shape result, (Func<V, double> Distance, V Near)[] junctions)
    {
        var builder = new BrepBuilder();
        return [.. result.AllFaces().Where(face =>
        {
            Point2D uv = Inside(face);
            Point3D p = new FaceSurfaceAdaptor(face).Value(uv.X, uv.Y);
            if (junctions.Any(j => Math.Abs(j.Distance(new V(p.X, p.Y, p.Z))) < 1e-7))
            {
                return false;
            }
            var vertex = new Vertex();
            builder.MakeVertex(ref vertex, p, 1e-7);
            var distance = new DistanceShapeShape(vertex, input);
            return distance.IsDone && distance.Value() > 1e-6;
        })];
    }

    // The most a ball on either side of a new face misses where it should be, at points inside each: the rim's ball has its centre
    // on the core's surface, a concave blend's at twice the radius from the core and at the radius from the body across the top
    private static double BallMiss(Face[] faces, Func<V, double>[] others)
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
                        var c = new V(p.X + sign * n.X, p.Y + sign * n.Y, p.Z + sign * n.Z);
                        double core = Core(c);
                        best = Math.Min(best, Math.Abs(core));
                        foreach (Func<V, double> other in others)
                        {
                            best = Math.Min(best, Math.Max(Math.Abs(core - 2 * Ball), Math.Abs(other(c) - Ball)));
                        }
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
    private static double WorstTurn(Shape shape, Face[] faces, bool betweenNew = false)
    {
        Dictionary<Shape, List<Face>> facesOfEdge = FacesOfEdges(shape);
        double worst = 0;
        foreach (Face face in faces)
        {
            foreach (Edge edge in face.AllEdges())
            {
                List<Face> two = facesOfEdge[edge];
                if (two.Count != 2 || Tool.Degenerated(edge) || (betweenNew && !two.All(f => faces.Any(n => n.IsSame(f)))))
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
