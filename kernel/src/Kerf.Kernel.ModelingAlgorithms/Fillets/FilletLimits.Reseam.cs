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
            Shape image = reshape.Value(spec.Edge);
            if (image.IsNull || image.Type != ShapeType.Edge)
            {
                return null;
            }
            Edge edge = ShapeCasts.Edge(image.Oriented(Orientation.Forward));
            if (!asked.Any(e => e.IsSame(edge)))
            {
                asked.Add(edge);
                fillet.Add(radius, edge);
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
            Edge image = ShapeCasts.Edge(reshape.Value(spec.Edge).Oriented(Orientation.Forward));
            foreach (Shape face in fillet.Generated(image))
            {
                history.AddGenerated(spec.Edge, face);
            }
            history.Remove(spec.Edge);
        }
        Interlocked.Increment(ref s_reseamBuilt);
        return new EngineOutcome(true, () => result, history, Valid: true);
    }

    // For the tests: the shape with its seams moved, as the stage would ask ChFi3d of it
    internal static Shape? ReseamedForTests(Shape shape, FilletSpec[] specs) => Reseam(shape, specs)?.Remade;

    // The shape with each cylinder's or cone's seam that ends at a vertex of an edge asked for moved away; the same solid
    private static (Shape Remade, ReShape Reshape, History History)? Reseam(Shape shape, FilletSpec[] specs)
    {
        // The faces whose seam ends at a vertex of an edge asked for
        var ends = new HashSet<Shape>(ShapeHasher.Instance);
        foreach (FilletSpec spec in specs)
        {
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
            if (!seen.Add(face) || new FaceSurfaceAdaptor(face).GetType() is not (SurfaceKind.Cylinder or SurfaceKind.Cone))
            {
                continue;
            }
            Face forward = ShapeCasts.Face(face.Oriented(Orientation.Forward));
            if (!forward.AllEdges().Any(e => Tool.IsClosed(e, forward) && e.AllVertices().Any(v => ends.Contains(v))))
            {
                continue;
            }
            Cancellation.ThrowIfRequested();
            if (Reseated(neighbours, forward, reshape) is not { } copy)
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

    // A face on a cylinder or a cone, bounded by two whole circles joined by its seam, remade with the seam at the middle of the
    // widest run of angle where no other edge or vertex of the face lies: on the surface turned about its axis to put its seam
    // there, the circles' vertices moved there and every edge given its curve on the new face. The replacements are recorded in
    // reshape; null if the face isn't of that kind
    private static Face? Reseated(Neighbours neighbours, Face face, ReShape reshape)
    {
        const double turn = 2 * Math.PI;
        var builder = new BrepBuilder();
        var occurrences = new List<Edge>();
        var closed = new List<Edge>();
        var spans = new List<(double From, double To)>();

        // A seam is an edge the face uses twice; an edge with two curves on it used once (a boundary along the seam's line, as
        // Booleans leave) is an edge like any other, with the curve of the side it's used on
        var times = new Dictionary<Shape, int>(ShapeHasher.Instance);
        for (var all = new Explorer(face, ShapeType.Edge); all.More(); all.Next())
        {
            times[all.Current()] = times.GetValueOrDefault(all.Current()) + 1;
        }
        bool IsSeam(Edge edge) => times.GetValueOrDefault(edge) == 2 && Tool.IsClosed(edge, face);
        for (var wires = new Explorer(face, ShapeType.Wire); wires.More(); wires.Next())
        {
            for (var edges = new Explorer(wires.Current(), ShapeType.Edge); edges.More(); edges.Next())
            {
                Edge edge = ShapeCasts.Edge(edges.Current());
                if (IsSeam(edge))
                {
                    continue; // the seam, which goes
                }
                if (Tool.Degenerated(edge) || Tool.CurveOnSurface(edge, face, out double first, out double last) is not { } curve)
                {
                    return null;
                }
                occurrences.Add(edge);
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
                    if (!a.IsSame(b) || curve is not LineCurve2D line || Math.Abs(Math.Abs(line.Direction.X) - 1) > 1e-12
                        || neighbours.Edges(a).Any(e => !e.IsSame(edge) && !IsSeam(e)) || closed.Any(c => c.IsSame(edge)))
                    {
                        return null;
                    }
                    closed.Add(edge);
                }
                else
                {
                    spans.Add((low, high));
                }
            }
        }
        if (closed.Count != 2)
        {
            return null;
        }

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
        double at = Math.PI, widest = turn;
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

        // The face on its surface turned about its axis so that its parameters wrap at at, which is the same surface
        if (Tool.Surface(face, out Placement location) is not ElementarySurface surface)
        {
            return null;
        }
        var turned = (Surface)surface.Rotated(surface.Axis, surface.Position.IsDirect ? at : -at);
        var copy = new Face();
        builder.MakeFace(ref copy, turned, location, Tool.Tolerance(face));

        // The circles' new vertices, where the seam now crosses them
        var points = new (Edge Edge, Edge New, Point3D Point, double V)[2];
        for (int k = 0; k < 2; k++)
        {
            Edge edge = ShapeCasts.Edge(closed[k].Oriented(Orientation.Forward));
            var line = (LineCurve2D)Tool.CurveOnSurface(edge, face, out double first, out _)!;
            double du = line.Direction.X;
            double u0 = line.Location.X;
            double t = first + Wrap((at - (u0 + du * first)) * du);
            var curve = new EdgeCurveAdaptor(edge);
            Point3D p = curve.Value(t);
            var vertex = new Vertex();
            builder.MakeVertex(ref vertex, p, Tool.Tolerance(edge));
            Edge remade = ShapeCasts.Edge(edge.EmptyCopied());
            builder.Add(remade, vertex.Oriented(Orientation.Forward));
            builder.Add(remade, vertex.Oriented(Orientation.Reversed));

            // On the face, from 0 to a turn, whichever way the circle runs
            double ut = u0 + du * t;
            double target = du > 0 ? 0 : turn;
            builder.UpdateEdge(remade, (Curve2D)line.Translated(new Vector2D(target - ut, 0)), copy, Tool.Tolerance(edge));
            builder.Range(remade, t, t + turn);
            reshape.Replace(edge, remade);
            points[k] = (edge, remade, p, line.Location.Y);
        }

        // The seam: the line along the surface between them, at 0 and a turn
        double length = points[0].Point.DistanceTo(points[1].Point);
        double v0 = points[0].V, v1 = points[1].V;
        if (Math.Abs(length - Math.Abs(v1 - v0)) > Tolerance || length <= Tolerance)
        {
            return null;
        }
        var seam = new Edge();
        builder.MakeEdge(ref seam, new LineCurve3D(points[0].Point, new Direction3D(new Vector3D(points[0].Point, points[1].Point))), Tolerance);
        Exploration.Vertices(points[0].New, out Vertex w0, out _);
        Exploration.Vertices(points[1].New, out Vertex w1, out _);
        builder.Add(seam, w0.Oriented(Orientation.Forward));
        builder.Add(seam, w1.Oriented(Orientation.Reversed));
        var along = new Direction2D(0, Math.Sign(v1 - v0));

        // The first circle as the face holds it ends where the seam's forward use starts
        Edge held = occurrences.First(e => e.IsSame(points[0].Edge));
        Curve2D onFirst = Tool.CurveOnSurface(ShapeCasts.Edge(points[0].New.Oriented(held.Orientation)), copy, out double f0, out double l0)!;
        double endU = (held.Orientation == Orientation.Forward ? onFirst.Value(l0) : onFirst.Value(f0)).X;
        double other = endU < Math.PI ? turn : 0;
        builder.UpdateEdge(seam, new LineCurve2D(new Point2D(endU, v0), along), new LineCurve2D(new Point2D(other, v0), along), copy, Tolerance);
        builder.Range(seam, 0, length);

        // The other edges, their curves on the face moved back by at, and on a turn where they lay before the seam
        var placed = new List<Edge>();
        foreach (Edge use in occurrences)
        {
            int index = Array.FindIndex(points, p => p.Edge.IsSame(use));
            if (index >= 0)
            {
                placed.Add(ShapeCasts.Edge(points[index].New.Oriented(use.Orientation)));
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
