// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Splines = Kerf.Engine.Geometry.Splines.BSplCLib;

namespace Kerf.Fillets;

// Stage 4c's faces, and its proof (FilletLimits.Spheres.cs)
internal static partial class FilletLimits
{
    // The new faces put in: each stretch's and pin's patch, the faces beside them with the contact lines in place of the contour's
    // edges, and the edges cut where the ball touches them. With the exact contacts of the new edges, for the volume
    private static (Shape Shape, History History, Dictionary<Shape, Func<double, V>> Exact, List<(Face Old, Face Copy)> Replaced)? AssembleSpheres(
        Shape shape, Neighbours neighbours, List<Rolling> balls, double radius)
    {
        var builder = new BrepBuilder();
        var exact = new Dictionary<Shape, Func<double, V>>(ShapeHasher.Instance);

        // For each face, each old edge's replacement, as the old edge's forward direction runs
        var replacing = new Dictionary<Shape, Dictionary<Shape, List<Edge>>>(ShapeHasher.Instance);
        void Replace(Face face, Edge old, List<Edge> by)
        {
            if (!replacing.TryGetValue(face, out Dictionary<Shape, List<Edge>>? map))
            {
                replacing[face] = map = new Dictionary<Shape, List<Edge>>(ShapeHasher.Instance);
            }
            map[old] = by;
        }
        var cuts = new Dictionary<Shape, List<(Vertex Old, Vertex New, double T)>>(ShapeHasher.Instance);
        var inserts = new List<(Square Square, Edge Arc)>();
        var made = new List<(Face Face, Edge[] From, Support Home)>();
        Edge Along(Edge edge, bool forward) => forward ? edge : ShapeCasts.Edge(edge.Reversed());
        Vertex Point(V p, double tolerance)
        {
            var vertex = new Vertex();
            builder.MakeVertex(ref vertex, p.Point, tolerance);
            return vertex;
        }

        foreach (Rolling ball in balls)
        {
            Support common = ball.Common;
            double worst = Math.Max(ball.Stretches.Max(s => s.Patch.Error), ball.Pins.Count == 0 ? 0 : ball.Pins.Max(p => p.Patch.Error));
            double tolerance = Math.Max(Tolerance, worst);
            var faces = new Dictionary<Patch, Face>();
            foreach (Patch patch in ball.Stretches.Select(s => s.Patch).Concat(ball.Pins.Select(p => p.Patch)))
            {
                var face = new Face();
                builder.MakeFace(ref face, patch.Fit.Surface, tolerance);
                faces[patch] = face;
            }

            // The vertices where the ball stands still and at the ends, and the arcs there
            var pinned = new Dictionary<Pin, (Vertex Foot, Vertex In, Vertex Out, Edge InArc, Edge OutArc)>();
            foreach ((Pin pin, Patch patch) in ball.Pins)
            {
                double inGap = (V.Of(new EdgeCurveAdaptor(pin.SeamIn).Value(pin.SeamInT)) - pin.In).Length;
                double outGap = (V.Of(new EdgeCurveAdaptor(pin.SeamOut).Value(pin.SeamOutT)) - pin.Out).Length;
                Vertex foot = Point(pin.Foot, tolerance), into = Point(pin.In, Math.Max(tolerance, 1.01 * inGap)), outOf = Point(pin.Out, Math.Max(tolerance, 1.01 * outGap));
                pinned[pin] = (foot, into, outOf, ArcEdge(builder, patch.Sections[0], foot, into, tolerance), ArcEdge(builder, patch.Sections[^1], foot, outOf, tolerance));
                Add(cuts, pin.SeamIn, (pin.Run.Start, into, pin.SeamInT));
                Add(cuts, pin.SeamOut, (pin.Run.End, outOf, pin.SeamOutT));
            }
            var squared = new Dictionary<Square, (Vertex OnCommon, Vertex OnOther)>();
            foreach ((Stretch stretch, _) in ball.Stretches)
            {
                foreach (Square? square in (ReadOnlySpan<Square?>)[stretch.FromSquare, stretch.ToSquare])
                {
                    if (square is null)
                    {
                        continue;
                    }
                    double commonGap = (V.Of(new EdgeCurveAdaptor(square.OnCommon).Value(square.CommonT)) - square.Common.One).Length;
                    double otherGap = (V.Of(new EdgeCurveAdaptor(square.OnOther).Value(square.OtherT)) - square.Other.Two).Length;
                    Vertex onCommon = Point(square.Common.One, Math.Max(tolerance, 1.01 * commonGap));
                    Vertex onOther = Point(square.Other.Two, Math.Max(tolerance, 1.01 * otherGap));
                    squared[square] = (onCommon, onOther);
                    Add(cuts, square.OnCommon, (square.At, onCommon, square.CommonT));
                    Add(cuts, square.OnOther, (square.At, onOther, square.OtherT));
                }
            }

            // Each stretch's face: its contact edges, and its ends' arcs, or at a slanted end where the plane cuts it
            foreach ((Stretch stretch, Patch patch) in ball.Stretches)
            {
                Face face = faces[patch];
                Run run = stretch.Run;
                int n = patch.Taus.Length, step = (n - 1) / stretch.Segments.Length;
                double commonFrom = stretch.CommonStartsLate ? patch.Taus[step] : patch.First;
                double otherFrom = stretch.FromSquare is { IsSquare: false } && !stretch.CommonStartsLate ? patch.Taus[step] : patch.First;
                double commonTo = stretch.CommonEndsEarly ? patch.Taus[n - 1 - step] : patch.Last;
                double otherTo = stretch.ToSquare is { IsSquare: false } && !stretch.CommonEndsEarly ? patch.Taus[n - 1 - step] : patch.Last;
                Edge? Stop(Square square, double tauOne, double tauTwo)
                {
                    (Vertex onCommon, Vertex onOther) = squared[square];
                    Edge? stop;
                    if (square.IsSquare)
                    {
                        stop = ArcEdge(builder, square.Common, onCommon, onOther, tolerance);
                        IsoOn(builder, stop.Value, face, tauOne, tolerance);

                        // Its curve on the end's plane: the arc's poles in the plane's parameters, as the map is affine
                        double w = Weight(square.Common, out V middle);
                        Point2D On(V p) => new(square.Plane.Parameters(p).U, square.Plane.Parameters(p).V);
                        builder.UpdateEdge(stop.Value, new BSplineCurve2D([On(square.Common.One), On(middle), On(square.Common.Two)], [0.0, 1.0], [3, 3], 2,
                                                                          weights: [1.0, w, 1.0]), square.Plane.Face, tolerance);
                    }
                    else if ((stop = CutEdge(builder, patch, face, square.Plane, tauOne, tauTwo, onCommon, onOther, tolerance)) is { } cut)
                    {
                        exact[cut] = s => Cut(stretch, square, common, ball.Side, radius, s);
                    }
                    if (stop is { } made)
                    {
                        inserts.Add((square, made));
                    }
                    return stop;
                }
                (Vertex startOne, Vertex startTwo, Edge? startArc) = stretch.FromPin is { } fp
                    ? (pinned[fp].Foot, pinned[fp].Out, pinned[fp].OutArc)
                    : (squared[stretch.FromSquare!].OnCommon, squared[stretch.FromSquare!].OnOther, Stop(stretch.FromSquare!, commonFrom, otherFrom));
                (Vertex endOne, Vertex endTwo, Edge? endArc) = stretch.ToPin is { } tp
                    ? (pinned[tp].Foot, pinned[tp].In, pinned[tp].InArc)
                    : (squared[stretch.ToSquare!].OnCommon, squared[stretch.ToSquare!].OnOther, Stop(stretch.ToSquare!, commonTo, otherTo));
                if (startArc is not { } startEdge || endArc is not { } endEdge)
                {
                    return null;
                }
                Edge one = ContactEdge(builder, patch.Fit.Curve1, patch.Fit.OnOne, common.Face, face, 0, startOne, endOne, commonFrom, commonTo, tolerance);
                Edge two = ContactEdge(builder, patch.Fit.Curve2, patch.Fit.OnTwo, run.Other.Face, face, 1, startTwo, endTwo, otherFrom, otherTo, tolerance);
                if (stretch.FromPin is not null)
                {
                    IsoOn(builder, startEdge, face, patch.First, tolerance);
                }
                if (stretch.ToPin is not null)
                {
                    IsoOn(builder, endEdge, face, patch.Last, tolerance);
                }
                Wired(builder, face, [one, endEdge, ShapeCasts.Edge(two.Reversed()), ShapeCasts.Edge(startEdge.Reversed())]);
                var commonPath = stretch.Segments[(stretch.CommonStartsLate ? 1 : 0)..(stretch.Segments.Length - (stretch.CommonEndsEarly ? 1 : 0))];
                int otherStart = stretch.FromSquare is { IsSquare: false } && !stretch.CommonStartsLate ? 1 : 0;
                int otherEnd = stretch.ToSquare is { IsSquare: false } && !stretch.CommonEndsEarly ? 1 : 0;
                var otherPath = stretch.Segments[otherStart..(stretch.Segments.Length - otherEnd)];
                Section commonFirst = stretch.CommonStartsLate ? stretch.FromSquare!.Common : stretch.First;
                Section commonLast = stretch.CommonEndsEarly ? stretch.ToSquare!.Common : stretch.Last;
                Section otherFirst = otherStart == 1 ? stretch.FromSquare!.Other : stretch.First;
                Section otherLast = otherEnd == 1 ? stretch.ToSquare!.Other : stretch.Last;
                exact[one] = s => Contact(stretch, commonPath, commonFirst, commonLast, common, ball.Side, radius, s).One;
                exact[two] = s => Contact(stretch, otherPath, otherFirst, otherLast, common, ball.Side, radius, s).Two;
                for (int i = 0; i < run.Edges.Length; i++)
                {
                    Replace(common.Face, run.Edges[i], i == 0 ? [Along(one, run.Forward[0])] : []);
                    Replace(run.Other.Face, run.Edges[i], i == 0 ? [Along(two, run.Forward[0])] : []);
                }
                made.Add((Oriented(face, patch, ball.Side), run.Edges, common));
            }

            // Each pin's piece of the sphere: a point on the common face, the arcs either side, the circle on the cylinder
            foreach ((Pin pin, Patch patch) in ball.Pins)
            {
                Face face = faces[patch];
                Run run = pin.Run;
                (Vertex foot, Vertex into, Vertex outOf, Edge inArc, Edge outArc) = pinned[pin];
                var point = new Edge();
                builder.MakeEdge(ref point);
                builder.UpdateEdge(point, new LineCurve2D(new Point2D(patch.First, 0), new Direction2D(1, 0)), face, tolerance);
                builder.Range(point, face, patch.First, patch.Last);
                builder.Add(point, foot.Oriented(Orientation.Forward));
                builder.Add(point, foot.Oriented(Orientation.Reversed));
                builder.Degenerated(point, true);
                Edge circle = ContactEdge(builder, patch.Fit.Curve2, patch.Fit.OnTwo, run.Other.Face, face, 1, into, outOf, patch.First, patch.Last, tolerance);
                IsoOn(builder, inArc, face, patch.First, tolerance);
                IsoOn(builder, outArc, face, patch.Last, tolerance);
                Wired(builder, face, [point, outArc, ShapeCasts.Edge(circle.Reversed()), ShapeCasts.Edge(inArc.Reversed())]);
                Support k = run.Other;
                exact[circle] = s =>
                {
                    double angle = pin.From + (pin.To - pin.From) * s;
                    return s <= 0 ? pin.In : s >= 1 ? pin.Out : pin.Centre + (k.X * Math.Cos(angle) + k.Y * Math.Sin(angle)) * radius;
                };
                for (int i = 0; i < run.Edges.Length; i++)
                {
                    Replace(common.Face, run.Edges[i], []);
                    Replace(k.Face, run.Edges[i], i == 0 ? [Along(circle, run.Forward[0])] : []);
                }
                made.Add((Oriented(face, patch, ball.Side), run.Edges, common));
            }
        }

        // The edges cut where the ball touches them, in every face they bound
        var trimmed = new Dictionary<Shape, Edge>(ShapeHasher.Instance);
        foreach ((Shape key, List<(Vertex Old, Vertex New, double T)> those) in cuts)
        {
            Edge edge = ShapeCasts.Edge(key.Oriented(Orientation.Forward));
            Exploration.Vertices(edge, out Vertex a, out Vertex b);
            Tool.Range(edge, out double first, out double last);
            foreach ((Vertex old, Vertex by, double t) in those)
            {
                if (old.IsSame(a) && !old.IsSame(b))
                {
                    (a, first) = (by, t);
                }
                else if (old.IsSame(b) && !old.IsSame(a))
                {
                    (b, last) = (by, t);
                }
                else
                {
                    return null;
                }
            }
            if (first >= last)
            {
                return null;
            }
            Edge cut = ShapeCasts.Edge(edge.EmptyCopied());
            builder.Add(cut, a.Oriented(Orientation.Forward));
            builder.Add(cut, b.Oriented(Orientation.Reversed));
            builder.Range(cut, first, last);
            trimmed[edge] = cut;
            foreach (Face face in neighbours.Faces(edge))
            {
                Replace(face, edge, [cut]);
            }
        }

        // At an end, the arc or the cut goes into the end face between its two cut edges; an edge can have one at each end
        foreach (IGrouping<Shape, (Square Square, Edge Arc)> group in inserts.GroupBy(i => (Shape)i.Square.OnCommon, ShapeHasher.Instance))
        {
            Square square = group.First().Square;
            Exploration.Vertices(square.OnCommon, out Vertex a, out _);
            var by = new List<Edge> { trimmed[square.OnCommon] };
            foreach ((Square at, Edge arc) in group)
            {
                if (a.IsSame(at.At))
                {
                    by.Insert(0, ShapeCasts.Edge(arc.Reversed()));
                }
                else
                {
                    by.Add(arc);
                }
            }
            Replace(square.Plane.Face, square.OnCommon, by);
        }

        // The faces beside, rebuilt with their edges replaced
        var reshape = new ReShape();
        var replaced = new List<(Face Old, Face Copy)>();

        // The new faces go in beside a face replaced as the shell holds it, so they're turned as it is
        var held = new Dictionary<Shape, Orientation>(ShapeHasher.Instance);
        for (var explorer = new Explorer(shape, ShapeType.Face); explorer.More(); explorer.Next())
        {
            held.TryAdd(explorer.Current(), explorer.Current().Orientation);
        }
        var homes = new Dictionary<Shape, List<Face>>(ShapeHasher.Instance);
        foreach ((Face face, _, Support home) in made)
        {
            Add(homes, home.Face, face);
        }
        foreach ((Shape key, Dictionary<Shape, List<Edge>> map) in replacing)
        {
            Face face = ShapeCasts.Face(key.Oriented(Orientation.Forward));
            Face copy = (Face)face.EmptyCopied();
            for (var wires = new Explorer(face, ShapeType.Wire); wires.More(); wires.Next())
            {
                var wire = new Wire();
                builder.MakeWire(ref wire);
                for (var edges = new Explorer(wires.Current(), ShapeType.Edge); edges.More(); edges.Next())
                {
                    Edge edge = ShapeCasts.Edge(edges.Current());
                    if (!map.TryGetValue(edge, out List<Edge>? by))
                    {
                        builder.Add(wire, edge);
                        continue;
                    }
                    bool forward = edge.Orientation == Orientation.Forward;
                    for (int i = 0; i < by.Count; i++)
                    {
                        builder.Add(wire, forward ? by[i] : ShapeCasts.Edge(by[by.Count - 1 - i].Reversed()));
                    }
                }
                wire.Closed(true);
                builder.Add(copy, wire);
            }
            replaced.Add((face, copy));
            if (homes.TryGetValue(key, out List<Face>? added))
            {
                var compound = new Compound();
                builder.MakeCompound(ref compound);
                builder.Add(compound, copy);
                bool turned = held.GetValueOrDefault(key, Orientation.Forward) == Orientation.Reversed;
                foreach (Face f in added)
                {
                    builder.Add(compound, turned ? f.Reversed() : f);
                }
                reshape.Replace(face, compound);
            }
            else
            {
                reshape.Replace(face, copy);
            }
        }
        Shape result = reshape.Apply(shape);
        if (result.IsNull)
        {
            return null;
        }
        History history = reshape.History();
        foreach ((Face old, Face copy) in replaced)
        {
            history.ReplaceModified(old, copy);
        }
        foreach ((Shape old, Edge cut) in trimmed)
        {
            history.AddModified(old, cut);
        }
        var removed = new HashSet<Shape>(ShapeHasher.Instance);
        foreach ((Face face, Edge[] from, _) in made)
        {
            foreach (Edge edge in from)
            {
                history.AddGenerated(edge, face);
                if (removed.Add(edge))
                {
                    history.Remove(edge);
                }
            }
        }

        // A fillet's edges are coded tangent, as ChFi3d's are (decision 1719)
        Shape[] newEdges = [.. made.SelectMany(m => m.Face.AllEdges()).Select(static e => (Shape)e)];
        result = BrepLibrary.RegularityEncoded(result, newEdges, out History coded);
        history.Merge(coded);
        return (result, history, exact, replaced);
    }

    // A section's arc as an edge from its first contact to its second, on [0, 1]
    private static Edge ArcEdge(BrepBuilder builder, Section section, Vertex one, Vertex two, double tolerance)
    {
        double w = Weight(section, out V middle);
        var curve = new BSplineCurve3D([section.One.Point, middle.Point, section.Two.Point], [0.0, 1.0], [3, 3], 2, weights: [1.0, w, 1.0]);
        var edge = new Edge();
        builder.MakeEdge(ref edge, curve, tolerance);
        builder.Add(edge, one.Oriented(Orientation.Forward));
        builder.Add(edge, two.Oriented(Orientation.Reversed));
        builder.Range(edge, 0, 1);
        return edge;
    }

    // An arc's curve on a patch, the line across it at tau
    private static void IsoOn(BrepBuilder builder, Edge arc, Face face, double tau, double tolerance)
    {
        builder.UpdateEdge(arc, new LineCurve2D(new Point2D(tau, 0), new Direction2D(0, 1)), face, tolerance);
        builder.Range(arc, face, 0, 1);
    }

    // A contact edge: the patch's curve from one tau to another, its curve on the support's face, and the line along the patch at v
    private static Edge ContactEdge(BrepBuilder builder, BSplineCurve3D curve, BSplineCurve2D on, Face support, Face face, double v,
                                    Vertex start, Vertex end, double from, double to, double tolerance)
    {
        var edge = new Edge();
        builder.MakeEdge(ref edge, curve, tolerance);
        builder.Add(edge, start.Oriented(Orientation.Forward));
        builder.Add(edge, end.Oriented(Orientation.Reversed));
        builder.UpdateEdge(edge, on, support, tolerance);
        builder.UpdateEdge(edge, new LineCurve2D(new Point2D(0, v), new Direction2D(1, 0)), face, tolerance);
        builder.Range(edge, from, to);
        return edge;
    }

    // Where a plane cuts a patch between its contacts' meetings with it, at (one, 0) and (two, 1): through points of the patch's
    // surface on the plane, its curve, its curve on the patch and its curve on the plane, cubics in v, the edge's tolerance what
    // they're apart between those points
    private static Edge? CutEdge(BrepBuilder builder, Patch patch, Face face, Support plane, double one, double two, Vertex start, Vertex end,
                                 double tolerance)
    {
        const int count = 32;
        var surface = new SurfaceAdaptor(patch.Fit.Surface);
        V At(double tau, double v)
        {
            surface.D0(tau, v, out Point3D p);
            return V.Of(p);
        }
        var ws = new double[count + 1];
        var values = new double[(count + 1) * 7];
        double lo = Math.Min(one, two), hi = Math.Max(one, two);
        for (int k = 0; k <= count; k++)
        {
            double w = 0.5 * (1 - Math.Cos(Math.PI * k / count));
            double tau = k == 0 ? one : k == count ? two : double.NaN;
            if (double.IsNaN(tau))
            {
                double a = lo, b = hi, ga = plane.Distance(At(a, w)), gb = plane.Distance(At(b, w));
                if (ga * gb > 0)
                {
                    return null;
                }
                for (int i = 0; i < 80 && b - a > 1e-15 * Math.Max(1, Math.Abs(b)); i++)
                {
                    double m = 0.5 * (a + b), gm = plane.Distance(At(m, w));
                    (a, ga, b, gb) = gm * ga <= 0 ? (a, ga, m, gm) : (m, gm, b, gb);
                }
                tau = 0.5 * (a + b);
            }
            V p = At(tau, w);
            (double u, double v) = plane.Parameters(p);
            ws[k] = w;
            new[] { p.X, p.Y, p.Z, tau, w, u, v }.CopyTo(values, k * 7);
        }
        if (Cubic(values, 7, ws) is not var (knots, multiplicities, poles))
        {
            return null;
        }
        Point3D[] points = [.. Enumerable.Range(0, count + 1).Select(i => new Point3D(poles[i * 7], poles[i * 7 + 1], poles[i * 7 + 2]))];
        Point2D[] onPatch = [.. Enumerable.Range(0, count + 1).Select(i => new Point2D(poles[i * 7 + 3], poles[i * 7 + 4]))];
        Point2D[] onPlane = [.. Enumerable.Range(0, count + 1).Select(i => new Point2D(poles[i * 7 + 5], poles[i * 7 + 6]))];
        var curve = new BSplineCurve3D(points, knots, multiplicities, 3);
        var pcurve = new BSplineCurve2D(onPatch, knots, multiplicities, 3);
        var planar = new BSplineCurve2D(onPlane, knots, multiplicities, 3);
        double gap = 0;
        for (int k = 0; k < count; k++)
        {
            double w = 0.5 * (ws[k] + ws[k + 1]);
            V c = V.Of(curve.Value(w));
            Point2D q = pcurve.Value(w), r = planar.Value(w);
            gap = Math.Max(gap, Math.Max((c - At(q.X, q.Y)).Length, Math.Max(Math.Abs(plane.Distance(c)), (c - V.Of(FilletLimits.At(plane, r))).Length)));
        }
        double own = Math.Max(tolerance, 1.5 * gap);
        var edge = new Edge();
        builder.MakeEdge(ref edge, curve, own);
        builder.Add(edge, start.Oriented(Orientation.Forward));
        builder.Add(edge, end.Oriented(Orientation.Reversed));
        builder.UpdateEdge(edge, pcurve, face, own);
        builder.UpdateEdge(edge, planar, plane.Face, own);
        builder.Range(edge, 0, 1);
        builder.UpdateVertex(start, own);
        builder.UpdateVertex(end, own);
        return edge;
    }

    // A cubic through values of the given dimension at the parameters, not-a-knot, as Interpolate's
    private static (double[] Knots, int[] Multiplicities, double[] Poles)? Cubic(double[] values, int dimension, double[] parameters)
    {
        int n = parameters.Length;
        var flat = new NCollection_Array1<double>(1, n + 4);
        for (int i = 1; i <= 4; i++)
        {
            flat.SetValue(i, parameters[0]);
            flat.SetValue(n + i, parameters[n - 1]);
        }
        for (int i = 2; i <= n - 3; i++)
        {
            flat.SetValue(i + 3, parameters[i]);
        }
        var at = new NCollection_Array1<double>(1, n);
        var orders = new NCollection_Array1<int>(1, n);
        for (int i = 0; i < n; i++)
        {
            at.SetValue(i + 1, parameters[i]);
            orders.SetValue(i + 1, 0);
        }
        double[] poles = [.. values];
        Splines.Interpolate(3, flat, at, orders, dimension, poles, out int problem);
        if (problem != 0)
        {
            return null;
        }
        return ([parameters[0], .. parameters[2..(n - 2)], parameters[n - 1]], [4, .. Enumerable.Repeat(1, n - 4), 4], poles);
    }

    private static void Wired(BrepBuilder builder, Face face, Edge[] edges)
    {
        var wire = new Wire();
        builder.MakeWire(ref wire);
        foreach (Edge edge in edges)
        {
            builder.Add(wire, edge);
        }
        wire.Closed(true);
        builder.Add(face, wire);
    }

    // Its outward side away from the ball's centre on a convex edge, towards it on a concave one
    private static Face Oriented(Face face, Patch patch, double side)
    {
        var adaptor = new SurfaceAdaptor(patch.Fit.Surface);
        double tau = patch.Taus[patch.Taus.Length / 2];
        V centre = patch.Sections[patch.Taus.Length / 2].Centre;
        adaptor.D1(tau, 0.5, out Point3D p, out Vector3D du, out Vector3D dv);
        V away = V.Of(p) - centre;
        return V.Of(du).Cross(V.Of(dv)).Dot(away) * -side < 0 ? ShapeCasts.Face(face.Reversed()) : face;
    }

    // The exact section of a stretch at s along some of its segments, from the first section given to the last, the segments
    // taking equal shares
    private static Section Contact(Stretch stretch, (Edge Edge, double From, double To)[] path, Section first, Section last, Support common,
                                   double side, double radius, double s)
    {
        if (s <= 0)
        {
            return first;
        }
        if (s >= 1)
        {
            return last;
        }
        int n = path.Length;
        int k = Math.Min((int)(s * n), n - 1);
        (Edge edge, double a, double b) = path[k];
        double t = a + (b - a) * (s * n - k);
        V nowhere = new(double.NaN, double.NaN, double.NaN); // fails the volume, so the result
        return StretchSection(stretch, edge, t, common, side, radius) ?? new Section(nowhere, nowhere, nowhere, nowhere);
    }

    // At a slanted end, the segment where the plane cuts the fillet: whether the contact on the common face is the one still short
    // of the plane there, and that segment's index
    private static (bool CommonInside, int Index)? Slanted(Stretch stretch, Square square) =>
        square.IsSquare ? null
        : ReferenceEquals(square, stretch.FromSquare) ? (!stretch.CommonStartsLate, 0)
        : (!stretch.CommonEndsEarly, stretch.Segments.Length - 1);

    // The part of a section's arc on the fillet's side of a slanted end's plane, as a range of w
    private static (double Lo, double Hi) Kept(Section section, Support plane, bool commonInside)
    {
        double side = plane.Distance(commonInside ? section.One : section.Two);
        double lo = 0, hi = 1;
        for (int i = 0; i < 60; i++)
        {
            double m = 0.5 * (lo + hi);
            bool inside = plane.Distance(ArcAt(section, m, out _)) * side > 0;
            (lo, hi) = inside == commonInside ? (m, hi) : (lo, m);
        }
        double cut = 0.5 * (lo + hi);
        return commonInside ? (0, cut) : (cut, 1);
    }

    // The exact cut at a slanted end, at w = s across the sections: the section in its segment whose arc there is on the plane
    private static V Cut(Stretch stretch, Square square, Support common, double side, double radius, double s)
    {
        if (s <= 0)
        {
            return square.Common.One;
        }
        if (s >= 1)
        {
            return square.Other.Two;
        }
        (Edge edge, double a, double b) = stretch.Segments[Slanted(stretch, square)!.Value.Index];
        V Point(double t) => StretchSection(stretch, edge, t, common, side, radius) is { } section ? ArcAt(section, s, out _) : new V(double.NaN, double.NaN, double.NaN);
        double ga = square.Plane.Distance(Point(a));
        for (int i = 0; i < 60; i++)
        {
            double m = 0.5 * (a + b), gm = square.Plane.Distance(Point(m));
            (a, b, ga) = gm * ga <= 0 ? (a, m, ga) : (m, b, gm);
        }
        return Point(0.5 * (a + b));
    }

    private static Section? StretchSection(Stretch stretch, Edge edge, double t, Support common, double side, double radius)
    {
        new EdgeCurveAdaptor(edge).D1(t, out Point3D q, out Vector3D d1);
        V tangent = V.Of(d1);
        return tangent.Length < 1e-14 ? null : Solve(common, stretch.Run.Other, side, radius, V.Of(q), tangent.Unit);
    }

    // The ball test, as stage 4a's: at points of each new face between its sections the ball put back touches both supports, along
    // its edges the face lies on them and is tangent to them, at its middle the ball reaches the result's faces at its radius and no
    // nearer, and the volume is the input's with the regions between the supports and the balls, from the exact sections and contacts
    private static bool SpheresHold(Shape input, Shape result, List<Rolling> balls, Dictionary<Shape, Func<double, V>> exact,
                                    List<(Face Old, Face Copy)> replaced, double radius)
    {
        Face[] faces = [.. result.AllFaces()];

        // The ball's distance to the rest: the faces it touches are the supports, and the patches are measured on the ball already,
        // all of a patch at the radius from its centre, which the distance's search takes long over
        var patches = new HashSet<object>(ReferenceEqualityComparer.Instance);
        foreach (Rolling ball in balls)
        {
            patches.UnionWith(ball.Stretches.Select(s => (object)s.Patch.Fit.Surface));
            patches.UnionWith(ball.Pins.Select(p => (object)p.Patch.Fit.Surface));
        }
        var boundary = new Compound();
        var builder = new BrepBuilder();
        builder.MakeCompound(ref boundary);
        foreach (Face f in faces.Where(f => Tool.Surface(f) is not { } s || !patches.Contains(s)))
        {
            builder.Add(boundary, f);
        }
        double allowed = 2 * Target(radius);
        bool Holds(Patch patch, double side, V centre)
        {
            if (faces.FirstOrDefault(f => ReferenceEquals(Tool.Surface(f), patch.Fit.Surface)) is not { IsNull: false } face)
            {
                return false;
            }
            bool reversed = face.Orientation == Orientation.Reversed;
            var surface = new FaceSurfaceAdaptor(face);
            for (int i = 0; i + 1 < patch.Taus.Length; i++)
            {
                double tau = 0.5 * (patch.Taus[i] + patch.Taus[i + 1]);
                foreach (double s in (ReadOnlySpan<double>)[0.25, 0.5, 0.75])
                {
                    if (BallError(surface, tau, s, patch.One, patch.Two, side, radius, reversed) is { } error && error > allowed)
                    {
                        return false;
                    }
                }
                if (!Tangent(surface, tau, 0, patch.One, reversed, allowed) || !Tangent(surface, tau, 1, patch.Two, reversed, allowed))
                {
                    return false;
                }
            }
            var vertex = new Vertex();
            builder.MakeVertex(ref vertex, centre.Point, Tolerance);
            var distance = new DistanceShapeShape(vertex, boundary);
            return distance.IsDone && Math.Abs(distance.Value() - radius) <= allowed + Tolerance;
        }

        V origin = balls[0].Pins[0].Pin.Centre;
        double flux = 0;
        foreach (Rolling ball in balls)
        {
            foreach ((Stretch stretch, Patch patch) in ball.Stretches)
            {
                if (!Holds(patch, ball.Side, patch.Sections[patch.Sections.Length / 2].Centre))
                {
                    return false;
                }
                for (int k = 0; k < stretch.Segments.Length; k++)
                {
                    (Edge edge, double a, double b) = stretch.Segments[k];
                    Func<Section, (double, double)>? keep = null;
                    foreach (Square? square in (ReadOnlySpan<Square?>)[stretch.FromSquare, stretch.ToSquare])
                    {
                        if (square is not null && Slanted(stretch, square) is var (commonInside, index) && index == k)
                        {
                            keep = section => Kept(section, square.Plane, commonInside);
                        }
                    }
                    flux += Swept(a, b, t => StretchSection(stretch, edge, t, ball.Common, ball.Side, radius), ball.Side, radius, origin, keep);
                }
            }
            foreach ((Pin pin, Patch patch) in ball.Pins)
            {
                if (!Holds(patch, ball.Side, pin.Centre))
                {
                    return false;
                }
                Support k = pin.Run.Other;
                flux += Swept(pin.From, pin.To, angle =>
                {
                    V on = pin.Centre + (k.X * Math.Cos(angle) + k.Y * Math.Sin(angle)) * radius;
                    return new Section(on, pin.Centre, pin.Foot, on);
                }, ball.Side, radius, origin);
            }
        }

        // The faces beside: what their boundaries lost and gained, each as the solid holds it
        var held = new Dictionary<Shape, Orientation>(ShapeHasher.Instance);
        for (var explorer = new Explorer(input, ShapeType.Face); explorer.More(); explorer.Next())
        {
            held.TryAdd(explorer.Current(), explorer.Current().Orientation);
        }
        foreach ((Face old, Face copy) in replaced)
        {
            if (Support.Of(old, true) is not { } support || !held.TryGetValue(old, out Orientation orientation))
            {
                return false;
            }
            double change = Loop(copy, support, exact, origin, old) - Loop(old, support, exact, origin, copy);
            flux += orientation == Orientation.Reversed ? -change : change;
        }
        double expected = Measured(input) + flux / 3;

        double area = Integral.Area(result);
        return Math.Abs(Measured(result) - expected) <= Allowed(expected, true) + Tolerance * area;
    }

    // Gauss's points on [-1, 1]
    private static ReadOnlySpan<double> Nodes => [-0.9602898564975363, -0.7966664774136267, -0.5255324099163290, -0.1834346424956498,
                                                  0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363];
    private static ReadOnlySpan<double> Weights => [0.1012285362903763, 0.2223810344533745, 0.3137066458778873, 0.3626837833783620,
                                                    0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763];

    // (X - O) . n over the arcs of sections from a to b, n the outward normal of what's left: away from the ball's centre on a convex
    // edge, towards it on a concave one
    private static double Swept(double a, double b, Func<double, Section?> sectionAt, double side, double radius, V origin,
                                Func<Section, (double Lo, double Hi)>? keep = null)
    {
        // A segment a slanted end's plane cuts more finely, as the cut's ends move fast across the sections
        int panels = keep is null ? 16 : 64;
        double h = 1e-5 * (b - a);
        double sum = 0;
        for (int panel = 0; panel < panels; panel++)
        {
            double lo = a + (b - a) * panel / panels, hi = a + (b - a) * (panel + 1) / panels;
            for (int i = 0; i < Nodes.Length; i++)
            {
                double t = 0.5 * (lo + hi) + 0.5 * (hi - lo) * Nodes[i];
                if (sectionAt(t - h) is not { } before || sectionAt(t) is not { } here || sectionAt(t + h) is not { } after)
                {
                    return double.NaN;
                }
                double across = 0;
                (double from, double to) = keep?.Invoke(here) ?? (0, 1);
                for (int j = 0; j < Nodes.Length; j++)
                {
                    double w = from + (to - from) * (0.5 + 0.5 * Nodes[j]);
                    V x = ArcAt(here, w, out V dw);
                    V dt = (ArcAt(after, w, out _) - ArcAt(before, w, out _)) / (2 * h);
                    V outward = (x - here.Centre) * (-side / radius);
                    across += Weights[j] * 0.5 * (to - from) * (x - origin).Dot(outward) * dt.Cross(dw).Length;
                }
                sum += Weights[i] * 0.5 * (hi - lo) * across;
            }
        }
        return sum * Math.Sign(b - a);
    }

    // The point of a section's arc at w from its first contact, and its derivative in w
    private static V ArcAt(Section s, double w, out V dw)
    {
        V a = s.One - s.Centre, b = s.Two - s.Centre;
        double r = a.Length;
        double angle = Math.Acos(Math.Clamp(a.Dot(b) / (r * b.Length), -1, 1));
        V e = a / r, f = (b - e * b.Dot(e)).Unit;
        dw = (f * Math.Cos(w * angle) - e * Math.Sin(w * angle)) * (r * angle);
        return s.Centre + (e * Math.Cos(w * angle) + f * Math.Sin(w * angle)) * r;
    }

    // The flux of X - O through a face of a support, as Green's theorem gives it from the face's boundary: the support is c(u) + v d,
    // so the integrand (S - O) . (S_u x S_v) is f(u) alone, and the integral is that of -v f(u) du round the boundary. Edges also in
    // the other face are left out, as they cancel. New edges are taken along their exact contacts, the rest along their curves on
    // the face
    private static double Loop(Face face, Support support, Dictionary<Shape, Func<double, V>> exact, V origin, Face other)
    {
        var skip = new HashSet<Shape>(ShapeHasher.Instance);
        foreach (Edge edge in other.AllEdges())
        {
            skip.Add(edge);
        }
        double Integrand(double u)
        {
            V s = support.At(u, 0, out V du);
            return (s - origin).Dot(du.Cross(support.Along));
        }
        double total = 0;
        for (var edges = new Explorer(face.Oriented(Orientation.Forward), ShapeType.Edge); edges.More(); edges.Next())
        {
            Edge edge = ShapeCasts.Edge(edges.Current());
            if (skip.Contains(edge))
            {
                continue;
            }
            var curve = new EdgeCurve2DAdaptor(edge, face);
            Tool.Range(edge, out double start, out double end);
            Func<double, (double U, double V)> uv;
            double first = start, last = end;
            if (exact.TryGetValue(edge, out Func<double, V>? path))
            {
                uv = s => Near(support, path(s), curve.Value(start + (end - start) * s).X);
                (first, last) = (0, 1);
            }
            else
            {
                uv = t =>
                {
                    Point2D p = curve.Value(t);
                    return (p.X, p.Y);
                };
            }
            const int panels = 16;
            double h = 1e-6 * (last - first);
            double sum = 0;
            for (int panel = 0; panel < panels; panel++)
            {
                double lo = first + (last - first) * panel / panels, hi = first + (last - first) * (panel + 1) / panels;
                for (int i = 0; i < Nodes.Length; i++)
                {
                    double t = 0.5 * (lo + hi) + 0.5 * (hi - lo) * Nodes[i];
                    (double u, double v) = uv(t);
                    double du = (uv(t + h).U - uv(t - h).U) / (2 * h);
                    sum += Weights[i] * 0.5 * (hi - lo) * -v * Integrand(u) * du;
                }
            }
            total += edge.Orientation == Orientation.Reversed ? -sum : sum;
        }
        return total;
    }
}
