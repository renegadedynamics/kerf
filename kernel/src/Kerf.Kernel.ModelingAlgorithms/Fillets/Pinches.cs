// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

/// <summary>
/// A fillet face that narrows to nothing at a point (unit 15Q; decision 2131): its two edges touch there, tangent, and part
/// again, so the face is two faces that meet at a vertex, as where a fillet's contact line just reaches a far edge, or where the
/// two faces of the edge are tangent at a point. ChFi3d builds the surfaces right but at some sizes leaves one face with its two
/// edges touching where it has no vertex, which <see cref="EdgeCrossings"/> rightly fails. This puts the vertex in, splits the
/// two edges at it and the face into its two parts, and moves no point of any surface or curve. Where the face's own surface
/// narrows to the point, its two edges reach it at different parameters, and a degenerate edge, all at the vertex, joins them
/// in each part, as at a cone's apex.
/// </summary>
/// <remarks>
/// It's asked only of a result that failed for its edges meeting, and only where every meeting is such a touch: two edges of
/// one face the operation made, each with one other face, not the same, within their tolerances of each other, with the face
/// between them on both sides of the point, out to where they're ten times that apart. Edges that cross (each passing to the
/// other's side), or run along each other, or touch with the face outside them, aren't changed, and the failure stands. What
/// it makes is checked as any result is, the edge crossings included. A part's tolerance is raised to what its curves on
/// their faces really are from its curve, where that's more. Everything is local to the call.
/// </remarks>
internal static class Pinches
{
    // The most touches resolved in one result
    private const int MostTouches = 8;

    // How far apart, in their tolerances, two edges have to be where the side of each is read
    private const double WellApart = 10;

    /// <summary>
    /// The result with each place where a face narrows to a point made a vertex, and its history from <paramref name="result"/>;
    /// null unless every meeting <see cref="EdgeCrossings"/> finds is such a touch.
    /// </summary>
    public static (Shape Shape, History History)? Resolve(Shape input, Shape result)
    {
        try
        {
            return Resolved(input, result);
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            return null;
        }
    }

    private static (Shape Shape, History History)? Resolved(Shape input, Shape result)
    {
        Shape current = result;
        History? history = null;
        for (int round = 0; round < MostTouches; round++)
        {
            IReadOnlyList<EdgeCrossings.Crossing> found = EdgeCrossings.All(input, current);
            if (found.Count == 0)
            {
                return history is null ? null : (current, history);
            }
            if (Split(current, found[0]) is not { } split)
            {
                return null;
            }
            current = split.Shape;
            if (history is null)
            {
                history = split.History;
            }
            else
            {
                history.Merge(split.History);
            }
        }
        return null;
    }

    // The shape with the face between the touching edges split at a new vertex, or null if it isn't such a touch
    private static (Shape Shape, History History)? Split(Shape shape, EdgeCrossings.Crossing touch)
    {
        if (touch.Along || double.IsNaN(touch.Parameter1) || double.IsNaN(touch.Parameter2))
        {
            return null;
        }
        Edge e1 = ShapeCasts.Edge(touch.Edge1.Oriented(Orientation.Forward));
        Edge e2 = ShapeCasts.Edge(touch.Edge2.Oriented(Orientation.Forward));
        if (e1.IsSame(e2) || Tool.Degenerated(e1) || Tool.Degenerated(e2))
        {
            return null;
        }

        // One face has both edges, and each has one other face, not the same
        List<Face> on1 = FacesOf(shape, e1), on2 = FacesOf(shape, e2);
        Face[] common = [.. on1.Where(f => on2.Any(g => g.IsSame(f)))];
        if (on1.Count != 2 || on2.Count != 2 || common.Length != 1)
        {
            return null;
        }
        Face face = ShapeCasts.Face(common[0].Oriented(Orientation.Forward));
        Face other1 = on1.First(f => !f.IsSame(face)), other2 = on2.First(f => !f.IsSame(face));
        if (other1.IsSame(other2) || face.Children.Count() != 1)
        {
            return null;
        }

        // Within their tolerances at the point, and the face between them on either side of it, out to where they're well apart
        Edge in1 = OnFace(face, e1), in2 = OnFace(face, e2);
        double t1 = touch.Parameter1, t2 = touch.Parameter2;
        var curve1 = new EdgeCurveAdaptor(e1);
        var curve2 = new EdgeCurveAdaptor(e2);
        double tolerance = Tool.Tolerance(e1) + Tool.Tolerance(e2);
        if (Distance(curve1.Value(t1), curve2.Value(t2)) > tolerance)
        {
            return null;
        }
        (t1, t2) = Closest(curve1, curve2, t1, t2, tolerance);
        if (!Between(face, in1, in2, curve1, curve2, t1, t2, tolerance, +1)
            || !Between(face, in1, in2, curve1, curve2, t1, t2, tolerance, -1))
        {
            return null;
        }

        // The vertex, the edges split at it, and the face's loop cut into two there
        AlgoTools.MakeNewVertex(e1, t1, e2, t2, out Vertex pinch);
        (Edge Before, Edge After) halves1 = Halves(e1, t1, pinch), halves2 = Halves(e2, t2, pinch);
        Edge? across = Across(face, in1, t1, in2, t2, pinch);
        if (Lobes(face, e1, halves1, e2, halves2, pinch, across) is not { } lobes)
        {
            return null;
        }

        var reshape = new ReShape();
        reshape.Replace(e1, Both(halves1.Before, halves1.After));
        reshape.Replace(e2, Both(halves2.Before, halves2.After));
        reshape.Replace(face, Both(lobes.First, lobes.Second));
        Shape made = reshape.Apply(shape);
        if (made.IsNull)
        {
            return null;
        }

        // The substitutions' history gives each of the three the compound it was replaced by: each part instead
        History history = reshape.History();
        foreach ((Shape whole, Shape first, Shape second) in (ReadOnlySpan<(Shape, Shape, Shape)>)[(e1, halves1.Before, halves1.After), (e2, halves2.Before, halves2.After), (face, lobes.First, lobes.Second)])
        {
            history.ReplaceModified(whole, first);
            history.AddModified(whole, second);
        }
        return (made, history);
    }

    private static List<Face> FacesOf(Shape shape, Edge edge)
    {
        var faces = new List<Face>();
        for (var exp = new Explorer(shape, ShapeType.Face); exp.More(); exp.Next())
        {
            Face face = ShapeCasts.Face(exp.Current());
            if (faces.Any(f => f.IsSame(face)))
            {
                continue;
            }
            for (var edges = new Explorer(face, ShapeType.Edge); edges.More(); edges.Next())
            {
                if (edges.Current().IsSame(edge))
                {
                    faces.Add(face);
                    break;
                }
            }
        }
        return faces;
    }

    // The edge as the forward face holds it, with its orientation there
    private static Edge OnFace(Face face, Edge edge)
    {
        for (var exp = new Explorer(face, ShapeType.Edge); exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(edge))
            {
                return ShapeCasts.Edge(exp.Current());
            }
        }
        return edge;
    }

    private static double Distance(Point3D a, Point3D b) => new Vector3D(a, b).Magnitude;

    // Whether, on one side of the touch, out to where the edges are well apart, the face lies on each edge's side towards the other:
    // so it narrows between them to the point, and they don't cross there
    private static bool Between(Face face, Edge in1, Edge in2, EdgeCurveAdaptor curve1, EdgeCurveAdaptor curve2,
                                double t1, double t2, double tolerance, int side)
    {
        double first = curve1.FirstParameter(), last = curve1.LastParameter();
        double speed = curve1.DN(t1, 1).Magnitude;
        if (speed < Kerf.Engine.Geometry.Constants.Resolution())
        {
            return false;
        }

        // Out along the first edge from the touch, doubling, until the two are well apart
        double step = tolerance / speed;
        double u1 = t1, u2 = t2;
        bool apart = false;
        for (int i = 0; i < 60; i++)
        {
            u1 = t1 + side * step;
            if (u1 <= first || u1 >= last)
            {
                return false;
            }
            Point3D p = curve1.Value(u1);
            u2 = Nearest(curve2, p, u2);
            if (Distance(p, curve2.Value(u2)) > WellApart * tolerance)
            {
                apart = true;
                break;
            }
            step *= 2;
        }
        if (!apart || u2 <= curve2.FirstParameter() || u2 >= curve2.LastParameter())
        {
            return false;
        }

        // In the face's parameters the face is to the left of each edge as the face holds it
        var on1 = new EdgeCurve2DAdaptor(in1, face);
        var on2 = new EdgeCurve2DAdaptor(in2, face);
        on1.D1(u1, out Point2D a, out Vector2D da);
        on2.D1(u2, out Point2D b, out Vector2D db);
        var towards = new Vector2D(b.X - a.X, b.Y - a.Y);
        return Inwards(da, in1.Orientation).Dot(towards) > 0 && Inwards(db, in2.Orientation).Dot(towards.Reversed()) > 0;
    }

    // Where the two come closest near the touch, as the crossing's point is only somewhere in the stretch where they're within
    // their tolerances: the first edge's parameter by golden section, out to where they're twice their tolerances apart each way
    private static (double T1, double T2) Closest(EdgeCurveAdaptor curve1, EdgeCurveAdaptor curve2, double t1, double t2, double tolerance)
    {
        double first = curve1.FirstParameter(), last = curve1.LastParameter();
        double speed = curve1.DN(t1, 1).Magnitude;
        if (speed < Kerf.Engine.Geometry.Constants.Resolution())
        {
            return (t1, t2);
        }
        double Gap(double t, ref double u)
        {
            Point3D p = curve1.Value(t);
            u = Nearest(curve2, p, u);
            return Distance(p, curve2.Value(u));
        }
        double Out(int side)
        {
            double step = tolerance / speed, u = t2;
            for (int i = 0; i < 60; i++)
            {
                double t = Math.Clamp(t1 + side * step, first, last);
                if (t == first || t == last || Gap(t, ref u) > 2 * tolerance)
                {
                    return t;
                }
                step *= 2;
            }
            return t1;
        }
        double lo = Out(-1), hi = Out(+1);
        const double golden = 0.6180339887498949;
        double uLo = t2, uHi = t2;
        double a = hi - golden * (hi - lo), b = lo + golden * (hi - lo);
        double ga = Gap(a, ref uLo), gb = Gap(b, ref uHi);
        for (int i = 0; i < 80 && hi - lo > 1e-15 * Math.Max(1, Math.Abs(t1)); i++)
        {
            if (ga < gb)
            {
                hi = b;
                b = a;
                gb = ga;
                uHi = uLo;
                a = hi - golden * (hi - lo);
                ga = Gap(a, ref uLo);
            }
            else
            {
                lo = a;
                a = b;
                ga = gb;
                uLo = uHi;
                b = lo + golden * (hi - lo);
                gb = Gap(b, ref uHi);
            }
        }
        double best = (lo + hi) / 2, near = t2;
        double gap = Gap(best, ref near);
        return gap <= Distance(curve1.Value(t1), curve2.Value(t2)) ? (best, near) : (t1, t2);
    }

    // The face's side of an edge, as the face holds it: to the left of its direction
    private static Vector2D Inwards(Vector2D tangent, Orientation orientation) =>
        orientation == Orientation.Reversed ? new Vector2D(tangent.Y, -tangent.X) : new Vector2D(-tangent.Y, tangent.X);

    // The parameter of the curve's point nearest p, by Newton's method from a guess, within the curve's range
    private static double Nearest(EdgeCurveAdaptor curve, Point3D p, double guess)
    {
        double first = curve.FirstParameter(), last = curve.LastParameter();
        double u = guess;
        for (int i = 0; i < 50; i++)
        {
            curve.D2(u, out Point3D at, out Vector3D d1, out Vector3D d2);
            var off = new Vector3D(p, at);
            double slope = d2.Dot(off) + d1.Dot(d1);
            if (Math.Abs(slope) < 1e-300)
            {
                break;
            }
            double next = Math.Clamp(u - d1.Dot(off) / slope, first, last);
            if (Math.Abs(next - u) <= 1e-14 * Math.Max(1, Math.Abs(u)))
            {
                u = next;
                break;
            }
            u = next;
        }
        return u;
    }

    // The edge split at the vertex, each part with the edge's curves, forward
    private static (Edge Before, Edge After) Halves(Edge edge, double t, Vertex at)
    {
        Tool.Range(edge, out double first, out double last);
        Exploration.Vertices(edge, out Vertex start, out Vertex end);
        AlgoTools.MakeSplitEdge(edge, start, first, at, t, out Edge before);
        AlgoTools.MakeSplitEdge(edge, at, t, end, last, out Edge after);
        Tightened(before, at);
        Tightened(after, at);
        return (before, after);
    }

    // The part's tolerance as far as each of its curves on a surface really is from its curve, where that's more than the edge
    // had: ChFi3d's curves can be further apart than its tolerance says, between the points the validity check samples, and a
    // part samples other points. The new vertex is raised with it; the edge's own vertices already have the edge's tolerance
    private static void Tightened(Edge part, Vertex at)
    {
        double tolerance = Tool.Tolerance(part);
        double worst = tolerance;
        Curve3D? curve = Tool.Curve(part, out Placement placement, out double first, out double last);
        if (curve is null)
        {
            return;
        }
        if (!placement.IsIdentity)
        {
            Transform3D move = placement.Transformation;
            curve = (Curve3D)curve.Transformed(move);
            first = curve.TransformedParameter(first, move);
            last = curve.TransformedParameter(last, move);
        }
        var along = new CurveAdaptor(curve, first, last);
        for (int i = 1; ; i++)
        {
            Curve2D? onSurface = null;
            Surface? surface = null;
            Placement where = new();
            double f = 0, l = 0;
            Tool.CurveOnSurface(part, ref onSurface, ref surface, ref where, ref f, ref l, i);
            if (onSurface is null)
            {
                break;
            }
            var check = new ValidateEdge(along, new CurveOnSurface(new CurveAdaptor2D(onSurface, f, l),
                                                                   new SurfaceAdaptor((Surface)surface!.Transformed(where.Transformation))), true);
            check.SetExactMethod(true);
            check.Process();
            if (check.IsDone)
            {
                check.UpdateTolerance(ref worst);
            }
        }
        if (worst > tolerance)
        {
            var builder = new BrepBuilder();
            builder.UpdateEdge(part, worst);
            builder.UpdateVertex(at, worst);
        }
    }

    // Where the face's surface itself narrows to the point (its section there of no length, as a fillet's between faces that are
    // tangent there), the two edges reach it at different parameters of the face: the section between them, an edge of the face
    // that is all at the vertex (degenerate, as at a cone's apex), from the first edge's point to the second's. Null where they
    // reach the same parameters, as where the face's edge runs into another face's edge (a fillet reaching the face's far edge)
    private static Edge? Across(Face face, Edge in1, double t1, Edge in2, double t2, Vertex pinch)
    {
        Point2D a = new EdgeCurve2DAdaptor(in1, face).Value(t1), b = new EdgeCurve2DAdaptor(in2, face).Value(t2);
        var surface = new FaceSurfaceAdaptor(face);
        double tolerance = Tool.Tolerance(pinch);
        double du = b.X - a.X, dv = b.Y - a.Y, length = Math.Sqrt(du * du + dv * dv);
        if (Math.Abs(du) <= surface.UResolution(tolerance) && Math.Abs(dv) <= surface.VResolution(tolerance))
        {
            return null;
        }
        var builder = new BrepBuilder();
        var across = new Edge();
        builder.MakeEdge(ref across);
        builder.UpdateEdge(across, new LineCurve2D(a, new Direction2D(du, dv)), face, tolerance);
        builder.Range(across, face, 0, length);
        builder.Add(across, pinch.Oriented(Orientation.Forward));
        builder.Add(across, pinch.Oriented(Orientation.Reversed));
        builder.Degenerated(across, true);
        return across;
    }

    // The face's one loop, with the two edges split, cut at the vertex into two loops, each a face on the face's surface, with the
    // section across between them where there is one
    private static (Face First, Face Second)? Lobes(Face face, Edge original1, (Edge Before, Edge After) halves1, Edge original2,
                                                    (Edge Before, Edge After) halves2, Vertex pinch, Edge? across)
    {
        var pieces = new List<Edge>();
        for (var exp = new Explorer(face, ShapeType.Edge); exp.More(); exp.Next())
        {
            Edge edge = ShapeCasts.Edge(exp.Current());
            (Edge Before, Edge After)? halves = edge.IsSame(original1) ? halves1 : edge.IsSame(original2) ? halves2 : null;
            if (halves is { } h)
            {
                pieces.Add(ShapeCasts.Edge(h.Before.Oriented(edge.Orientation)));
                pieces.Add(ShapeCasts.Edge(h.After.Oriented(edge.Orientation)));
            }
            else
            {
                pieces.Add(edge);
            }
        }

        // Each piece's start, as the loop runs; at the pinch, what came in along one edge goes out along the other
        bool IsOf(Edge piece, (Edge Before, Edge After) halves) => piece.IsSame(halves.Before) || piece.IsSame(halves.After);
        Vertex Start(Edge piece) => Exploration.FirstVertex(piece, true);
        Vertex End(Edge piece) => Exploration.LastVertex(piece, true);
        Edge? Next(Edge piece)
        {
            Vertex end = End(piece);
            Edge[] from = [.. pieces.Where(p => Start(p).IsSame(end) && !p.IsSame(piece))];
            if (end.IsSame(pinch))
            {
                (Edge, Edge) otherHalves = IsOf(piece, halves1) ? halves2 : halves1;
                from = [.. from.Where(p => IsOf(p, otherHalves))];
            }
            return from.Length == 1 ? from[0] : null;
        }

        var used = new HashSet<int>();
        var loops = new List<List<Edge>>();
        for (int i = 0; i < pieces.Count; i++)
        {
            if (used.Contains(i))
            {
                continue;
            }
            var run = new List<Edge>();
            Edge piece = pieces[i];
            int at = i;
            while (true)
            {
                if (!used.Add(at))
                {
                    return null;
                }
                run.Add(piece);
                if (across is { } section && End(piece).IsSame(pinch))
                {
                    run.Add(ShapeCasts.Edge(IsOf(piece, halves1) ? section : section.Reversed()));
                }
                if (Next(piece) is not { } next)
                {
                    return null;
                }
                at = pieces.IndexOf(next);
                if (at == i)
                {
                    break;
                }
                piece = next;
            }
            loops.Add(run);
        }
        if (loops.Count != 2)
        {
            return null;
        }

        var builder = new BrepBuilder();
        Face Made(List<Edge> run)
        {
            var wire = new Wire();
            builder.MakeWire(ref wire);
            foreach (Edge edge in run)
            {
                builder.Add(wire, edge);
            }
            wire.Closed(true);
            Face lobe = (Face)face.EmptyCopied();
            builder.Add(lobe, wire);
            return lobe;
        }
        return (Made(loops[0]), Made(loops[1]));
    }

    private static Compound Both(Shape first, Shape second)
    {
        var builder = new BrepBuilder();
        var compound = new Compound();
        builder.MakeCompound(ref compound);
        builder.Add(compound, first);
        builder.Add(compound, second);
        return compound;
    }
}
