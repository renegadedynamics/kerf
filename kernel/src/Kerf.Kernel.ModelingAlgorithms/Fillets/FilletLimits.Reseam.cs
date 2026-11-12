// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

// Stage 4b, slice 2 (unit 15T3; fillet-design.md, section 16): ChFi3d fails where an edge it's asked for ends at a vertex on a
// cylinder's or a cone's seam (OCCT #26655: the corpus's bug26655, a tube with a window whose corners lie on both cylinders' seams).
// The seam is only where the face's parameters wrap, not a feature of the shape, so, where ChFi3d has failed, each such face is
// remade with its seam moved round to where no edge or vertex of the face lies, and ChFi3d is asked again on the shape so remade,
// which is the same solid. Its result is proved by the ball as stage 4b's first slice is.
internal static partial class FilletLimits
{
    // Counted for the tests: fillets built this way and returned, and those the ball test refused
    internal static long s_reseamBuilt;
    internal static long s_reseamRefused;

    private static EngineOutcome? BuildReseamed(Shape shape, FilletSpec[] specs, FilletOptions options)
    {
        if (shape.AllSolids().Length != 1 || shape.Type is not (ShapeType.Solid or ShapeType.Compound) || specs.Length == 0)
        {
            return null;
        }
        double radius = specs[0].Radius;
        if (specs.Any(s => s.Radius != radius) || Reseam(shape, specs) is not { } reseamed)
        {
            return null;
        }
        (Shape remade, ReShape reshape, History history) = reseamed;
        Cancellation.ThrowIfRequested();

        // ChFi3d, on the edges as the shape remade holds them
        var fillet = new MakeFillet(remade, options.Shape);
        FilletTolerances t = options.Tolerances;
        fillet.SetParams(t.Angular, t.Spine, t.Parametric, t.Approximation3D, t.Approximation2D, t.Deflection);
        fillet.SetContinuity(options.InternalContinuity, options.AngularTolerance);
        var asked = new List<Edge>();
        foreach (FilletSpec spec in specs)
        {
            if (Images(reshape, spec.Edge) is not { } images)
            {
                return null;
            }
            foreach (Edge edge in images)
            {
                if (!asked.Any(e => e.IsSame(edge)))
                {
                    asked.Add(edge);
                    fillet.Add(radius, edge);
                }
            }
        }
        EngineOutcome outcome = MakerOutcome.Of(fillet, default, [remade]);
        if (!outcome.IsDone)
        {
            return null;
        }
        Shape result = outcome.Shape();
        if (result.AllSolids().Length != 1)
        {
            return null;
        }
        var blends = new List<Face>();
        foreach (Edge edge in asked)
        {
            foreach (Shape face in fillet.Generated(edge))
            {
                if (!blends.Any(b => b.IsSame(face)))
                {
                    blends.Add(ShapeCasts.Face(face));
                }
            }
        }
        if (Validity.Problem(result, "The fillet") is not null || blends.Count == 0 || !BallOnFaces(remade, result, blends, radius, t))
        {
            Interlocked.Increment(ref s_reseamRefused);
            return null;
        }
        // The history: the remaking's, then ChFi3d's; each edge asked for generated what ChFi3d made from its image, and is gone
        history.Merge(outcome.History);
        foreach (FilletSpec spec in specs)
        {
            foreach (Edge image in Images(reshape, spec.Edge)!)
            {
                foreach (Shape face in fillet.Generated(image))
                {
                    history.AddGenerated(spec.Edge, face);
                }
            }
            history.Remove(spec.Edge);
        }
        Interlocked.Increment(ref s_reseamBuilt);
        return new EngineOutcome(true, () => result, history, Valid: true);
    }

    // An edge as the shape remade holds it: itself, its copy on a face remade, or the two it became where a new seam cut it
    private static List<Edge>? Images(ReShape reshape, Edge edge)
    {
        Shape image = reshape.Value(edge);
        if (image.IsNull || image.Type is not (ShapeType.Edge or ShapeType.Wire))
        {
            return null;
        }
        return [.. image.AllEdges().Select(static e => ShapeCasts.Edge(e.Oriented(Orientation.Forward)))];
    }

    /// <summary>Whether an edge asked for ends on a face's seam, where the face's surface closes on itself: where ChFi3d fails (OCCT
    /// #26655), and what the failure says if the seam couldn't be moved.</summary>
    public static bool EndsOnSeam(Shape shape, FilletSpec[] specs)
    {
        var ends = new HashSet<Shape>(ShapeHasher.Instance);
        foreach (FilletSpec spec in specs)
        {
            foreach (Vertex vertex in spec.Edge.AllVertices())
            {
                ends.Add(vertex);
            }
        }
        foreach (Face face in shape.AllFaces())
        {
            Face forward = ShapeCasts.Face(face.Oriented(Orientation.Forward));
            foreach (Edge edge in forward.AllEdges())
            {
                if (!Tool.Degenerated(edge) && Tool.IsClosed(edge, forward) && edge.AllVertices().Any(v => ends.Contains(v)))
                {
                    return true;
                }
            }
        }
        return false;
    }

    // For the tests: the shape with its seams moved, as the stage would ask ChFi3d of it
    internal static Shape? ReseamedForTests(Shape shape, FilletSpec[] specs) => Reseam(shape, specs)?.Remade;

    // The shape with each cylinder's, cone's or sphere's seam that ends at a vertex of an edge asked for moved away; the same solid
    private static (Shape Remade, ReShape Reshape, History History)? Reseam(Shape shape, FilletSpec[] specs)
    {
        // The faces whose seam ends at a vertex of an edge asked for
        var ends = new HashSet<Shape>(ShapeHasher.Instance);
        var asked = new HashSet<Shape>(ShapeHasher.Instance);
        foreach (FilletSpec spec in specs)
        {
            asked.Add(spec.Edge);
            foreach (Vertex vertex in spec.Edge.AllVertices())
            {
                ends.Add(vertex);
            }
        }
        var neighbours = new Neighbours(shape);
        var reshape = new ReShape();
        var moved = new List<(Face Old, Face New)>();
        var seen = new HashSet<Shape>(ShapeHasher.Instance);
        foreach (Face face in shape.AllFaces())
        {
            if (!seen.Add(face) || new FaceSurfaceAdaptor(face).GetType() is not (SurfaceKind.Cylinder or SurfaceKind.Cone or SurfaceKind.Sphere))
            {
                continue;
            }
            Face forward = ShapeCasts.Face(face.Oriented(Orientation.Forward));
            if (!forward.AllEdges().Any(e => !Tool.Degenerated(e) && Tool.IsClosed(e, forward) && e.AllVertices().Any(v => ends.Contains(v))))
            {
                continue;
            }
            Cancellation.ThrowIfRequested();
            if (Reseated(neighbours, forward, reshape, asked) is not { } copy)
            {
                return null;
            }
            moved.Add((forward, copy));
        }
        if (moved.Count == 0)
        {
            return null;
        }
        Shape remade = reshape.Apply(shape);
        if (remade.IsNull || remade.AllSolids().Length != 1 || Validity.Problem(remade, "The fillet") is not null)
        {
            return null;
        }
        History history = reshape.History();
        foreach ((Face old, Face copy) in moved)
        {
            history.ReplaceModified(old, copy);
        }
        return (remade, reshape, history);
    }

    // A face on a cylinder, a cone or a sphere, closed round its axis, remade with its seam moved: on the surface turned about its
    // axis to put its seam there, and every edge given its curve on the new face. Between two whole circles whose vertices are the
    // seam's alone (unit 15T3) the seam goes to the middle of the widest run of angle where no other edge or vertex of the face
    // lies, and the circles' vertices with it. Otherwise (unit 15T4) to the middle of the widest run where the face is bounded,
    // below and above, by a whole circle, by a point where the surface closes (a cone's apex, a sphere's pole) or by an arc about
    // the axis, which the new seam cuts in two at a new vertex (an arc asked for only if there's no other). The replacements are recorded in reshape; null
    // if the face isn't of that kind
    private static Face? Reseated(Neighbours neighbours, Face face, ReShape reshape, HashSet<Shape> asked)
    {
        const double turn = 2 * Math.PI;
        var builder = new BrepBuilder();
        var occurrences = new List<Edge>();
        var closed = new List<Edge>();
        var points = new List<Edge>();
        var spans = new List<(double From, double To)>();
        var open = new List<(Edge Edge, Curve2D Curve, double Low, double High)>();

        // A seam is an edge the face uses twice; an edge with two curves on it used once (a boundary along the seam's line, as
        // Booleans leave) is an edge like any other, with the curve of the side it's used on
        var times = new Dictionary<Shape, int>(ShapeHasher.Instance);
        for (var all = new Explorer(face, ShapeType.Edge); all.More(); all.Next())
        {
            times[all.Current()] = times.GetValueOrDefault(all.Current()) + 1;
        }
        bool IsSeam(Edge edge) => times.GetValueOrDefault(edge) == 2 && Tool.IsClosed(edge, face);
        static bool RoundTheAxis(Curve2D curve) => curve is LineCurve2D line && Math.Abs(Math.Abs(line.Direction.X) - 1) <= 1e-12;
        for (var wires = new Explorer(face, ShapeType.Wire); wires.More(); wires.Next())
        {
            for (var edges = new Explorer(wires.Current(), ShapeType.Edge); edges.More(); edges.Next())
            {
                Edge edge = ShapeCasts.Edge(edges.Current());
                if (IsSeam(edge))
                {
                    continue; // the seam, which goes
                }
                if (Tool.CurveOnSurface(edge, face, out double first, out double last) is not { } curve)
                {
                    return null;
                }
                occurrences.Add(edge);
                if (Tool.Degenerated(edge))
                {
                    // Where the surface closes to a point, all the way round
                    if (!RoundTheAxis(curve) || Math.Abs(last - first - turn) > 1e-9 || points.Any(c => c.IsSame(edge)))
                    {
                        return null;
                    }
                    points.Add(edge);
                    continue;
                }
                double low = double.MaxValue, high = double.MinValue;
                for (int i = 0; i <= 16; i++)
                {
                    double u = curve.Value(first + (last - first) * i / 16).X;
                    low = Math.Min(low, u);
                    high = Math.Max(high, u);
                }
                if (high - low > turn - 1e-9)
                {
                    // A whole circle, iso in v, whose one vertex is the seam's alone
                    Exploration.Vertices(edge, out Vertex a, out Vertex b);
                    if (!a.IsSame(b) || !RoundTheAxis(curve)
                        || neighbours.Edges(a).Any(e => !e.IsSame(edge) && !IsSeam(e)) || closed.Any(c => c.IsSame(edge)))
                    {
                        return null;
                    }
                    closed.Add(edge);
                }
                else
                {
                    spans.Add((low, high));
                    open.Add((edge, curve, low, high));
                }
            }
        }
        double at = Math.PI;
        var cut = new List<Edge>();
        if (closed.Count == 2 && points.Count == 0)
        {
            // The widest run of angle clear of every other edge
            var covered = new List<(double From, double To)>();
            foreach ((double from, double to) in spans.Select(s => (From: Wrap(s.From), To: Wrap(s.From) + s.To - s.From)).OrderBy(s => s.From))
            {
                if (covered.Count > 0 && from <= covered[^1].To)
                {
                    covered[^1] = (covered[^1].From, Math.Max(covered[^1].To, to));
                }
                else
                {
                    covered.Add((from, to));
                }
            }
            while (covered.Count > 1 && covered[^1].To - turn >= covered[0].From)
            {
                covered[0] = (covered[^1].From - turn, Math.Max(covered[0].To, covered[^1].To - turn));
                covered.RemoveAt(covered.Count - 1);
            }
            double widest = turn;
            if (covered.Count > 0)
            {
                widest = 0;
                for (int i = 0; i < covered.Count; i++)
                {
                    double next = i + 1 < covered.Count ? covered[i + 1].From : covered[0].From + turn;
                    if (next - covered[i].To > widest)
                    {
                        widest = next - covered[i].To;
                        at = Wrap(covered[i].To + widest / 2);
                    }
                }
            }
            if (widest < 1e-3)
            {
                return null;
            }
        }
        else
        {
            // The widest run of angle between the other edges' ends where just two edges bound the face, each a whole circle, a
            // point or an arc about the axis; an arc that's asked for only if there's no run without one
            if (closed.Count + points.Count > 2)
            {
                return null;
            }
            double[] breaks = [.. open.SelectMany(o => (IEnumerable<double>)[Wrap(o.Low), Wrap(o.High)]).Order()];
            double widest = 0;
            foreach (bool cutAsked in (ReadOnlySpan<bool>)[false, true])
            {
                if (widest > 0)
                {
                    break; // an arc asked for is cut only if there's no other
                }
                for (int i = 0; i < Math.Max(breaks.Length, 1); i++)
                {
                    double from = breaks.Length == 0 ? 0 : breaks[i], width = breaks.Length == 0 ? turn : (i + 1 < breaks.Length ? breaks[i + 1] : breaks[0] + turn) - from;
                    if (width <= Math.Max(widest, 1e-3))
                    {
                        continue;
                    }
                    double middle = Wrap(from + width / 2);
                    Edge[] over = [.. open.Where(o => Wrap(middle - Wrap(o.Low)) < o.High - o.Low).Select(o => o.Edge)];
                    if (over.Length + closed.Count + points.Count != 2
                        || over.Any(e => (!cutAsked && asked.Contains(e)) || reshape.IsRecorded(e.Oriented(Orientation.Forward)) || !RoundTheAxis(open.First(o => o.Edge.IsSame(e)).Curve)))
                    {
                        continue;
                    }
                    (widest, at) = (width, middle);
                    cut = [.. over];
                }
            }
            if (widest == 0)
            {
                return null;
            }
        }

        // The face on its surface turned about its axis so that its parameters wrap at at, which is the same surface
        if (Tool.Surface(face, out Placement location) is not ElementarySurface surface)
        {
            return null;
        }
        var turned = (Surface)surface.Rotated(surface.Axis, surface.Position.IsDirect ? at : -at);
        var copy = new Face();
        builder.MakeFace(ref copy, turned, location, Tool.Tolerance(face));

        // The two ends of the new seam: each circle's new vertex, where the seam now crosses it; each point's own; each arc's new
        // one, the arc cut in two there. Each edge goes from a turn to nothing or from nothing to a turn on the face, whichever
        // way it runs
        var ends = new List<(Edge Edge, Edge[] New, Vertex Vertex, Point3D Point, double V)>();
        foreach (Edge whole in closed.Concat(points).Concat(cut))
        {
            Edge edge = ShapeCasts.Edge(whole.Oriented(Orientation.Forward));
            var line = (LineCurve2D)Tool.CurveOnSurface(edge, face, out double first, out double last)!;
            double du = line.Direction.X;
            double u0 = line.Location.X;
            if (points.Any(c => c.IsSame(edge)))
            {
                Exploration.Vertices(edge, out Vertex own, out _);
                Edge again = ShapeCasts.Edge(edge.EmptyCopied());
                builder.Add(again, own.Oriented(Orientation.Forward));
                builder.Add(again, own.Oriented(Orientation.Reversed));
                builder.UpdateEdge(again, (Curve2D)line.Translated(new Vector2D((du > 0 ? 0 : turn) - (u0 + du * first), 0)), copy, Tool.Tolerance(edge));
                builder.Range(again, first, last);
                builder.Degenerated(again, true);
                reshape.Replace(edge, again);
                ends.Add((edge, [again], own, Tool.Pnt(own), line.Location.Y));
                continue;
            }
            double t = first + Wrap((at - (u0 + du * first)) * du);
            var curve = new EdgeCurveAdaptor(edge);
            Point3D p = curve.Value(t);
            var vertex = new Vertex();
            builder.MakeVertex(ref vertex, p, Tool.Tolerance(edge));
            double ut = u0 + du * t;
            if (closed.Any(c => c.IsSame(edge)))
            {
                Edge remade = ShapeCasts.Edge(edge.EmptyCopied());
                builder.Add(remade, vertex.Oriented(Orientation.Forward));
                builder.Add(remade, vertex.Oriented(Orientation.Reversed));

                // On the face, from 0 to a turn, whichever way the circle runs
                double target = du > 0 ? 0 : turn;
                builder.UpdateEdge(remade, (Curve2D)line.Translated(new Vector2D(target - ut, 0)), copy, Tool.Tolerance(edge));
                builder.Range(remade, t, t + turn);
                reshape.Replace(edge, remade);
                ends.Add((edge, [remade], vertex, p, line.Location.Y));
                continue;
            }
            if (t <= first + 1e-9 || t >= last - 1e-9)
            {
                return null;
            }
            Exploration.Vertices(edge, out Vertex a, out Vertex b);
            Edge before = ShapeCasts.Edge(edge.EmptyCopied()), after = ShapeCasts.Edge(edge.EmptyCopied());
            builder.Add(before, a.Oriented(Orientation.Forward));
            builder.Add(before, vertex.Oriented(Orientation.Reversed));
            builder.UpdateEdge(before, (Curve2D)line.Translated(new Vector2D((du > 0 ? turn : 0) - ut, 0)), copy, Tool.Tolerance(edge));
            builder.Range(before, first, t);
            builder.Add(after, vertex.Oriented(Orientation.Forward));
            builder.Add(after, b.Oriented(Orientation.Reversed));
            builder.UpdateEdge(after, (Curve2D)line.Translated(new Vector2D((du > 0 ? 0 : turn) - ut, 0)), copy, Tool.Tolerance(edge));
            builder.Range(after, t, last);
            var both = new Wire();
            builder.MakeWire(ref both);
            builder.Add(both, before);
            builder.Add(both, after);
            reshape.Replace(edge, both);
            ends.Add((edge, [before, after], vertex, p, line.Location.Y));
        }
        if (ends.Count != 2)
        {
            return null;
        }

        // The seam: along the surface between them, at 0 and a turn; a line on a cylinder or a cone, a meridian on a sphere
        double v0 = ends[0].V, v1 = ends[1].V, length = Math.Abs(v1 - v0);
        Curve3D along3d;
        if (surface is SphericalSurface sphere)
        {
            Point3D centre = sphere.Location;
            Point3D on = surface.Value(at, v0);
            Vector3D radial = new(centre, surface.Value(at, 0)), up = new(centre, surface.Value(at, Math.PI / 2));
            Vector3D onwards = (radial * -Math.Sin(v0) + up * Math.Cos(v0)) * Math.Sign(v1 - v0);
            var from = new Direction3D(new Vector3D(centre, on));
            along3d = new CircleCurve3D(new Frame3D(centre, from.Crossed(new Direction3D(onwards)), from), sphere.Radius);
        }
        else
        {
            if (Math.Abs(ends[0].Point.DistanceTo(ends[1].Point) - length) > Tolerance)
            {
                return null;
            }
            along3d = new LineCurve3D(ends[0].Point, new Direction3D(new Vector3D(ends[0].Point, ends[1].Point)));
        }
        if (length <= Tolerance)
        {
            return null;
        }
        var seam = new Edge();
        builder.MakeEdge(ref seam, along3d, Tolerance);
        builder.Add(seam, ends[0].Vertex.Oriented(Orientation.Forward));
        builder.Add(seam, ends[1].Vertex.Oriented(Orientation.Reversed));
        var along = new Direction2D(0, Math.Sign(v1 - v0));

        // The first end's edge as the face holds it ends where the seam's forward use starts
        Edge held = occurrences.First(e => e.IsSame(ends[0].Edge));
        double endU = double.NaN;
        foreach (Edge piece in ends[0].New)
        {
            Edge use = ShapeCasts.Edge(piece.Oriented(held.Orientation));
            if (Exploration.LastVertex(use, true).IsSame(ends[0].Vertex))
            {
                double u = End(use, copy, false).X;
                if (Math.Abs(u) < 1e-9 || Math.Abs(u - turn) < 1e-9)
                {
                    endU = u;
                }
            }
        }
        if (double.IsNaN(endU))
        {
            return null;
        }
        double other = endU < Math.PI ? turn : 0;
        builder.UpdateEdge(seam, new LineCurve2D(new Point2D(endU, v0), along), new LineCurve2D(new Point2D(other, v0), along), copy, Tolerance);
        builder.Range(seam, 0, length);

        // The other edges, their curves on the face moved back by at, and on a turn where they lay before the seam
        var placed = new List<Edge>();
        foreach (Edge use in occurrences)
        {
            int index = ends.FindIndex(p => p.Edge.IsSame(use));
            if (index >= 0)
            {
                placed.AddRange(ends[index].New.Select(e => ShapeCasts.Edge(e.Oriented(use.Orientation))));
                continue;
            }
            Edge edge = ShapeCasts.Edge(use.Oriented(Orientation.Forward));
            Curve2D curve = Tool.CurveOnSurface(use, face, out double first, out double last)!;
            double middle = curve.Value(0.5 * (first + last)).X;
            double shift = turn * Math.Floor((at + turn - middle) / turn) - at;
            if (reshape.IsRecorded(edge))
            {
                return null; // an edge twice on the face, or on two faces moved
            }
            Edge remade = ShapeCasts.Edge(edge.EmptyCopied());
            Exploration.Vertices(edge, out Vertex a, out Vertex b);
            builder.Add(remade, a.Oriented(Orientation.Forward));
            builder.Add(remade, b.Oriented(Orientation.Reversed));
            builder.UpdateEdge(remade, (Curve2D)curve.Translated(new Vector2D(shift, 0)), copy, Tool.Tolerance(edge));
            builder.Range(remade, first, last);
            reshape.Replace(edge, remade);
            placed.Add(ShapeCasts.Edge(remade.Oriented(use.Orientation)));
        }
        placed.Add(seam);
        placed.Add(ShapeCasts.Edge(seam.Reversed()));

        // The wires, each use followed by the one that starts where it ends, on the face and at the same vertex
        var pending = placed.Select(u => (Use: u, Start: End(u, copy, true), End: End(u, copy, false))).ToList();
        while (pending.Count > 0)
        {
            var wire = new Wire();
            builder.MakeWire(ref wire);
            var current = pending[0];
            pending.RemoveAt(0);
            Vertex origin = Exploration.FirstVertex(current.Use, true);
            Point2D start = current.Start;
            builder.Add(wire, current.Use);
            while (true)
            {
                Vertex end = Exploration.LastVertex(current.Use, true);
                if (end.IsSame(origin) && current.End.DistanceTo(start) < 1e-6)
                {
                    break;
                }
                int best = -1;
                double nearest = 1e-6;
                for (int i = 0; i < pending.Count; i++)
                {
                    double gap = pending[i].Start.DistanceTo(current.End);
                    if (gap < nearest && Exploration.FirstVertex(pending[i].Use, true).IsSame(end))
                    {
                        (best, nearest) = (i, gap);
                    }
                }
                if (best < 0)
                {
                    return null;
                }
                current = pending[best];
                pending.RemoveAt(best);
                builder.Add(wire, current.Use);
            }
            wire.Closed(true);
            builder.Add(copy, wire);
        }
        reshape.Replace(face, copy);
        return copy;

        static double Wrap(double u) => u - turn * Math.Floor(u / turn);
    }

    // Where an edge's use on a face starts or ends, on the face
    private static Point2D End(Edge use, Face face, bool start)
    {
        Curve2D curve = Tool.CurveOnSurface(use, face, out double first, out double last)!;
        bool forward = use.Orientation != Orientation.Reversed;
        return curve.Value(start == forward ? first : last);
    }
}
