// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

// Stage 4c (unit 15X; fillet-design.md, section 14): where a contour crosses a cylinder of exactly the fillet's radius, with the
// ball on its inside, the ball's centre has to be on that cylinder's axis and at the radius from the face the contour runs along,
// one point, so over that stretch the ball can't move: the fillet there is a piece of a sphere, touching the common face at one
// point and the cylinder round a circle. ChFi3d's walk can't pass such a point (decision 2159). Here a contour along one common
// face is taken as runs, one for each face beside it: on each pinned run a piece of the sphere, and between them, and out to an
// end square to the edge, stage 4a's sections, each solved where it lies, from the ball at one end to the ball at the other. The
// faces beside are given the contact lines in place of the contour's edges, and the edges where they meet the pinned cylinder,
// or the end face, are cut where the ball touches them. What it makes is proved as 4a's is, apart from how.
internal static partial class FilletLimits
{
    // Counted for the tests: the pieces of spheres built and returned, and those the ball test refused
    internal static long s_spheresBuilt;
    internal static long s_spheresRefused;

    // A run of the contour beside one face: its edges in order, whether each runs along the contour, and the face's support
    private sealed record Run(Edge[] Edges, bool[] Forward, Support Other, bool Pinned)
    {
        public Vertex Start => Ends(0).Start;
        public Vertex End => Ends(Edges.Length - 1).End;

        public (Vertex Start, Vertex End) Ends(int i)
        {
            Exploration.Vertices(Edges[i], out Vertex a, out Vertex b);
            return Forward[i] ? (a, b) : (b, a);
        }
    }

    // Where the ball stands still: its centre, its contact on the common face, its contacts on the runs' faces either side, the
    // angles of those round the cylinder's axis, and the edges between the cylinder and those faces, with where they're cut
    private sealed record Pin(Run Run, V Centre, V Foot, V In, V Out, double From, double To, Edge SeamIn, double SeamInT, Edge SeamOut,
                              double SeamOutT);

    // An end square to the contour: its vertex, the plane face there, its edges with the common face and the other, where the
    // section cuts them, and the section
    private sealed record Square(Vertex At, Support Plane, Edge OnCommon, double CommonT, Edge OnOther, double OtherT, Section Section);

    // A run between two ends, with the stretches of its edges' parameters the sections are taken over, and the end sections
    private sealed record Stretch(Run Run, Pin? FromPin, Square? FromSquare, Pin? ToPin, Square? ToSquare, (Edge Edge, double From, double To)[] Segments,
                                  Section First, Section Last);

    // The surface fitted through sections, with its curves and the sections, and the supports it touches
    private sealed record Patch(Fitted Fit, double[] Taus, Section[] Sections, double Error, Support One, Support Two)
    {
        public double First => Taus[0];
        public double Last => Taus[^1];
    }

    // A contour as built: its common face, the side the ball is on, its stretches and pins and their patches
    private sealed record Rolling(Support Common, double Side, List<(Stretch Stretch, Patch Patch)> Stretches, List<(Pin Pin, Patch Patch)> Pins);

    private static EngineOutcome? BuildSpheres(Shape shape, FilletSpec[] specs)
    {
        if (shape.AllSolids().Length != 1 || shape.Type is not (ShapeType.Solid or ShapeType.Compound) || specs.Length == 0)
        {
            return null;
        }
        double radius = specs[0].Radius;
        if (specs.Any(s => s.Radius != radius))
        {
            return null;
        }

        // The contours as ChFi3d makes them, along tangent edges
        var maker = new MakeFillet(shape);
        foreach (FilletSpec spec in specs)
        {
            maker.Add(radius, spec.Edge);
        }
        var neighbours = new Neighbours(shape);
        var balls = new List<Rolling>();
        for (int ic = 1; ic <= maker.ContourCount; ic++)
        {
            Cancellation.ThrowIfRequested();
            Edge[] edges = [.. Enumerable.Range(1, maker.NbEdges(ic)).Select(j => ShapeCasts.Edge(maker.Edge(ic, j).Oriented(Orientation.Forward)))];
            if (Contour(neighbours, edges, maker.Closed(ic), radius) is not { } ball)
            {
                return null;
            }
            balls.Add(ball);
        }
        if (balls.Count == 0 || AssembleSpheres(shape, neighbours, balls, radius) is not { } assembled)
        {
            return null;
        }
        (Shape made, History history, Dictionary<Shape, Func<double, V>> exact, List<(Face Old, Face Copy)> replaced) = assembled;
        if (Validity.Problem(made, "The fillet") is not null || !SpheresHold(shape, made, balls, exact, replaced, radius))
        {
            Interlocked.Increment(ref s_spheresRefused);
            return null;
        }
        Interlocked.Increment(ref s_spheresBuilt);
        Shape built = made;
        return new EngineOutcome(true, () => built, history, Valid: true);
    }

    // A contour along one face, in runs beside the others, at least one pinned, the pinned ones between stretches, an open one's
    // ends square: its stretches and pins fitted, or null if it's anything else
    private static Rolling? Contour(Neighbours neighbours, Edge[] edges, bool closed, double radius)
    {
        if (edges.Length < 2)
        {
            return null;
        }
        var faces = new (Face, Face)[edges.Length];
        for (int i = 0; i < edges.Length; i++)
        {
            if (Tool.Degenerated(edges[i]) || neighbours.Faces(edges[i]) is not [var a, var b] || a.IsSame(b))
            {
                return null;
            }
            faces[i] = (a, b);
        }

        // The face common to every edge
        Face common = faces[0].Item1;
        if (!faces.All(f => f.Item1.IsSame(common) || f.Item2.IsSame(common)))
        {
            common = faces[0].Item2;
            if (!faces.All(f => f.Item1.IsSame(common) || f.Item2.IsSame(common)))
            {
                return null;
            }
        }
        Face[] others = [.. faces.Select(f => f.Item1.IsSame(common) ? f.Item2 : f.Item1)];
        if (others.Any(o => o.IsSame(common)) || Support.Of(common, true) is not { } one)
        {
            return null;
        }
        if (!Signed(one, edges[0], common))
        {
            return null;
        }
        TypeOfConcavity kind = global::Kerf.Engine.Fillets.ChFi3d.DefineConnectType(edges[0], common, others[0], 1.0e-5, false);
        if (kind is not (TypeOfConcavity.Convex or TypeOfConcavity.Concave))
        {
            return null;
        }
        double side = kind == TypeOfConcavity.Convex ? -1 : 1;

        // Each edge's direction along the contour, from the vertices it shares with the next
        var forward = new bool[edges.Length];
        for (int i = 0; i < edges.Length; i++)
        {
            Exploration.Vertices(edges[i], out Vertex a, out Vertex b);
            Edge next = edges[(i + 1) % edges.Length];
            Edge previous = edges[(i + edges.Length - 1) % edges.Length];
            bool bTouchesNext = next.AllVertices().Any(v => v.IsSame(b)), aTouchesNext = next.AllVertices().Any(v => v.IsSame(a));
            if (i + 1 < edges.Length || closed)
            {
                if (aTouchesNext == bTouchesNext)
                {
                    return null;
                }
                forward[i] = bTouchesNext;
            }
            else
            {
                forward[i] = previous.AllVertices().Any(v => v.IsSame(a));
            }
        }

        // The runs, beside one face each; a closed contour from the start of a stretch after a pinned run
        var runs = new List<Run>();
        int start = 0;
        if (closed)
        {
            while (start < edges.Length && others[start].IsSame(others[(start + edges.Length - 1) % edges.Length]))
            {
                start++;
            }
            if (start == edges.Length)
            {
                return null;
            }
        }
        var supports = new Dictionary<Shape, Support>(ShapeHasher.Instance);
        for (int k = 0; k < edges.Length;)
        {
            int i = (start + k) % edges.Length;
            var group = new List<int> { i };
            for (k++; k < edges.Length && others[(start + k) % edges.Length].IsSame(others[i]); k++)
            {
                group.Add((start + k) % edges.Length);
            }
            Face other = others[i];
            if (supports.ContainsKey(other) || Support.Of(other, true) is not { } support || !Signed(support, edges[i], other))
            {
                return null; // a face beside the contour twice isn't a run
            }
            supports[other] = support;
            bool pinned = support.IsCylinder && Math.Abs(support.Rho - radius) <= Math.Max(Tolerance, Tool.Tolerance(other)) && support.Sign == -side;
            runs.Add(new Run([.. group.Select(g => edges[g])], [.. group.Select(g => forward[g])], support, pinned));
        }
        if (closed && runs[0].Pinned)
        {
            runs.Add(runs[0]);
            runs.RemoveAt(0);
        }
        int count = runs.Count;
        if (!runs.Any(r => r.Pinned) || (!closed && (runs[0].Pinned || runs[^1].Pinned)))
        {
            return null;
        }
        for (int i = 0; i < count; i++)
        {
            if (!closed && i + 1 == count)
            {
                break;
            }
            if (runs[i].Pinned == runs[(i + 1) % count].Pinned)
            {
                return null; // two stretches, or two pins, side by side
            }
        }

        // The pins
        var pins = new Dictionary<int, Pin>();
        for (int i = 0; i < count; i++)
        {
            if (runs[i].Pinned)
            {
                if (PinOf(neighbours, one, side, runs[(i + count - 1) % count], runs[i], runs[(i + 1) % count], radius) is not { } pin)
                {
                    return null;
                }
                pins[i] = pin;
            }
        }

        // The stretches between them, or to an end
        var stretches = new List<(Stretch, Patch)>();
        for (int i = 0; i < count; i++)
        {
            if (runs[i].Pinned)
            {
                continue;
            }
            Run run = runs[i];
            Pin? from = pins.GetValueOrDefault((i + count - 1) % count), to = pins.GetValueOrDefault((i + 1) % count);
            Square? fromSquare = null, toSquare = null;
            if (!closed && i == 0)
            {
                from = null;
                if (SquareAt(neighbours, one, side, run, true, radius) is not { } s)
                {
                    return null;
                }
                fromSquare = s;
            }
            if (!closed && i + 1 == count)
            {
                to = null;
                if (SquareAt(neighbours, one, side, run, false, radius) is not { } s)
                {
                    return null;
                }
                toSquare = s;
            }
            if (StretchOf(run, from, fromSquare, to, toSquare) is not { } stretch
                || FitStretch(stretch, one, side, radius) is not { } patch)
            {
                return null;
            }
            stretches.Add((stretch, patch));
        }
        var fitted = new List<(Pin, Patch)>();
        foreach (Pin pin in pins.Values)
        {
            if (FitPin(pin, one, side, radius) is not { } patch)
            {
                return null;
            }
            fitted.Add((pin, patch));
        }
        return new Rolling(one, side, stretches, fitted);
    }

    // The ball on a pinned run: its centre on the cylinder's axis at the radius from the common face, which must be at the radius
    // from the faces before and after too, and where those faces meet the cylinder, cut where the ball touches them
    private static Pin? PinOf(Neighbours neighbours, Support common, double side, Run before, Run run, Run after, double radius)
    {
        Support k = run.Other;
        V axis = k.Axis;
        Tool.Range(run.Edges[0], out double a0, out double b0);
        V near = V.Of(new EdgeCurveAdaptor(run.Edges[0]).Value(0.5 * (a0 + b0)));
        double s = (near - k.Origin).Dot(axis);
        double target = side * radius;
        double scale = Math.Max(1, radius);
        for (int iteration = 0; ; iteration++)
        {
            V p = k.Origin + axis * s;
            double f = common.Distance(p) - target;
            if (Math.Abs(f) <= 1e-14 * scale)
            {
                break;
            }
            double h = 1e-6 * scale;
            double df = (common.Distance(p + axis * h) - common.Distance(p - axis * h)) / (2 * h);
            if (Math.Abs(df) < 1e-6 || iteration == 50)
            {
                return null; // the axis runs along the common face, or no root
            }
            s -= f / df;
        }
        V centre = k.Origin + axis * s;
        double allowed = Math.Max(Tolerance, Math.Max(Tool.Tolerance(before.Other.Face), Tool.Tolerance(after.Other.Face)));
        if (Math.Abs(before.Other.Distance(centre) - target) > allowed || Math.Abs(after.Other.Distance(centre) - target) > allowed
            || (centre - near).Length > 10 * radius)
        {
            return null;
        }
        V foot = common.Foot(centre), into = before.Other.Foot(centre), outOf = after.Other.Foot(centre);

        // Round the axis from where the ball meets the face before to where it meets the face after, through the run
        double Angle(V p) => Math.Atan2((p - k.Origin).Dot(k.Y), (p - k.Origin).Dot(k.X));
        double from = Angle(into), to = Angle(outOf), middle = Angle(near);
        double Wrap(double x) => x - 2 * Math.PI * Math.Floor(x / (2 * Math.PI));
        double through = Wrap(middle - from), sweep = Wrap(to - from);
        if (sweep <= through)
        {
            through = Wrap(from - middle);
            sweep = Wrap(from - to);
            if (sweep <= through)
            {
                return null;
            }
            to = from - sweep;
        }
        else
        {
            to = from + sweep;
        }

        // The edges between the cylinder and the faces either side, at the run's ends, and where the ball touches them
        if (Seam(neighbours, before.End, run.Edges[0], before.Edges[^1], into) is not var (seamIn, inT)
            || Seam(neighbours, run.End, run.Edges[^1], after.Edges[0], outOf) is not var (seamOut, outT))
        {
            return null;
        }
        return new Pin(run, centre, foot, into, outOf, from, to, seamIn, inT, seamOut, outT);
    }

    // The third edge at a vertex of only three, and the parameter on it of a point that's on it, strictly inside its range
    private static (Edge Edge, double T)? Seam(Neighbours neighbours, Vertex at, Edge one, Edge two, V point)
    {
        Edge[] there = [.. neighbours.Edges(at).Where(e => !Tool.Degenerated(e)).Select(e => ShapeCasts.Edge(e.Oriented(Orientation.Forward)))];
        Edge[] rest = [.. there.Where(e => !e.IsSame(one) && !e.IsSame(two))];
        if (there.Length != 3 || rest.Length != 1)
        {
            return null;
        }
        return OnEdge(rest[0], at, point) is { } t ? (rest[0], t) : null;
    }

    // The parameter of a point on an edge, by Newton's method from the end at the given vertex, if the point is on it within
    // tolerance and strictly inside it
    private static double? OnEdge(Edge edge, Vertex from, V point)
    {
        var curve = new EdgeCurveAdaptor(edge);
        Tool.Range(edge, out double first, out double last);
        Exploration.Vertices(edge, out Vertex a, out _);
        double t = a.IsSame(from) ? first : last;
        for (int iteration = 0; iteration < 50; iteration++)
        {
            curve.D2(t, out Point3D p, out Vector3D d1, out Vector3D d2);
            V gap = V.Of(p) - point, t1 = V.Of(d1);
            double g = gap.Dot(t1), dg = t1.Dot(t1) + gap.Dot(V.Of(d2));
            if (dg <= 0)
            {
                return null;
            }
            double step = g / dg;
            t -= step;
            if (Math.Abs(step) <= 1e-14 * Math.Max(1, Math.Abs(t)))
            {
                break;
            }
        }
        double allowed = Math.Max(Tolerance, Tool.Tolerance(edge));
        double margin = 1e-9 * (last - first);
        if ((V.Of(curve.Value(t)) - point).Length > allowed || t <= first + margin || t >= last - margin)
        {
            return null;
        }
        return t;
    }

    // An open contour's end: three edges at its vertex, the third face a plane square to the contour there, holding the ball's
    // section, which cuts its edges with the common face and the other
    private static Square? SquareAt(Neighbours neighbours, Support common, double side, Run run, bool atStart, double radius)
    {
        Edge edge = atStart ? run.Edges[0] : run.Edges[^1];
        Vertex at = atStart ? run.Start : run.End;
        Tool.Range(edge, out double first, out double last);
        Exploration.Vertices(edge, out Vertex a, out _);
        double t = a.IsSame(at) ? first : last;
        var curve = new EdgeCurveAdaptor(edge);
        curve.D1(t, out Point3D q, out Vector3D d1);
        V tangent = V.Of(d1);
        if (tangent.Length < 1e-14 || Solve(common, run.Other, side, radius, V.Of(q), tangent.Unit) is not { } section)
        {
            return null;
        }
        Edge[] there = [.. neighbours.Edges(at).Where(e => !Tool.Degenerated(e)).Select(e => ShapeCasts.Edge(e.Oriented(Orientation.Forward)))];
        if (there.Length != 3)
        {
            return null;
        }
        Edge? withCommon = null, withOther = null;
        foreach (Edge e in there.Where(e => !e.IsSame(edge)))
        {
            IReadOnlyList<Face> faces = neighbours.Faces(e);
            if (faces.Any(f => f.IsSame(common.Face)))
            {
                withCommon = e;
            }
            else if (faces.Any(f => f.IsSame(run.Other.Face)))
            {
                withOther = e;
            }
        }
        if (withCommon is not { } onCommon || withOther is not { } onOther)
        {
            return null;
        }
        Face[] ends = [.. neighbours.Faces(onCommon).Where(f => !f.IsSame(common.Face))];
        if (ends.Length != 1 || !neighbours.Faces(onOther).Any(f => f.IsSame(ends[0])) || Support.Of(ends[0]) is not { IsCylinder: false } plane)
        {
            return null;
        }
        double allowed = Math.Max(Tolerance, Tool.Tolerance(ends[0]));
        if (Math.Abs(plane.Distance(section.Centre)) > allowed || Math.Abs(plane.Distance(section.One)) > allowed
            || Math.Abs(plane.Distance(section.Two)) > allowed || Math.Abs(Math.Abs(plane.Axis.Dot(tangent.Unit)) - 1) > 1e-9)
        {
            return null;
        }
        if (OnEdge(onCommon, at, section.One) is not { } commonT || OnEdge(onOther, at, section.Two) is not { } otherT)
        {
            return null;
        }
        return new Square(at, plane, onCommon, commonT, onOther, otherT, section);
    }

    // The stretch's sections run over its edges' parameters, from where the plane square to the edge holds the ball at one end to
    // where it holds it at the other: at a pin that's past or short of the edge's vertex, as the edge's curve goes on
    private static Stretch? StretchOf(Run run, Pin? from, Square? fromSquare, Pin? to, Square? toSquare)
    {
        int n = run.Edges.Length;
        var segments = new (Edge Edge, double From, double To)[n];
        for (int i = 0; i < n; i++)
        {
            Tool.Range(run.Edges[i], out double a, out double b);
            segments[i] = run.Forward[i] ? (run.Edges[i], a, b) : (run.Edges[i], b, a);
        }
        Section first, last;
        if (from is not null)
        {
            if (SquareTo(run.Edges[0], segments[0].From, from.Centre) is not { } t)
            {
                return null;
            }
            segments[0] = (segments[0].Edge, t, segments[0].To);
            first = new Section(V.Of(new EdgeCurveAdaptor(run.Edges[0]).Value(t)), from.Centre, from.Foot, from.Out);
        }
        else
        {
            first = fromSquare!.Section;
        }
        if (to is not null)
        {
            if (SquareTo(run.Edges[^1], segments[^1].To, to.Centre) is not { } t)
            {
                return null;
            }
            segments[^1] = (segments[^1].Edge, segments[^1].From, t);
            last = new Section(V.Of(new EdgeCurveAdaptor(run.Edges[^1]).Value(t)), to.Centre, to.Foot, to.In);
        }
        else
        {
            last = toSquare!.Section;
        }

        // Each segment must still run the way the contour does
        foreach ((Edge _, double a, double b) in segments)
        {
            if (a == b)
            {
                return null;
            }
        }
        if (segments.Select((s, i) => Math.Sign(s.To - s.From) == (run.Forward[i] ? 1 : -1)).Any(ok => !ok))
        {
            return null;
        }
        return new Stretch(run, from, fromSquare, to, toSquare, segments, first, last);
    }

    // The parameter, near a start, of the point of an edge's curve (going on past the edge if need be) where the plane square to it
    // holds the point given
    private static double? SquareTo(Edge edge, double start, V point)
    {
        var curve = new EdgeCurveAdaptor(edge);
        Tool.Range(edge, out double first, out double last);
        double t = start;
        for (int iteration = 0; iteration < 50; iteration++)
        {
            curve.D2(t, out Point3D p, out Vector3D d1, out Vector3D d2);
            V gap = point - V.Of(p), t1 = V.Of(d1);
            double h = gap.Dot(t1), dh = -t1.Dot(t1) + gap.Dot(V.Of(d2));
            if (dh >= 0)
            {
                return null;
            }
            double step = h / dh;
            t -= step;
            if (Math.Abs(t - start) > 0.5 * (last - first))
            {
                return null;
            }
            if (Math.Abs(step) <= 1e-14 * Math.Max(1, Math.Abs(t)))
            {
                return t;
            }
        }
        return null;
    }

    // The stretch's sections at count steps of each segment, closer at its ends, and the surface through them, more until it's
    // within the target
    private static Patch? FitStretch(Stretch stretch, Support common, double side, double radius)
    {
        double target = Target(radius);
        for (int count = FirstSections; count <= MostSections;)
        {
            Cancellation.ThrowIfRequested();
            if (StretchThrough(stretch, common, side, radius, count) is not { } patch)
            {
                return null;
            }
            if (patch.Error <= target)
            {
                return patch;
            }
            count = Math.Max(count + 1, (int)Math.Ceiling(count * 1.15 * Math.Pow(patch.Error / target, 0.25)));
        }
        return null;
    }

    private static Patch? StretchThrough(Stretch stretch, Support common, double side, double radius, int count)
    {
        Run run = stretch.Run;
        var sections = new List<Section>();
        var values = new List<double>();
        int n = stretch.Segments.Length;
        for (int k = 0; k < n; k++)
        {
            (Edge edge, double a, double b) = stretch.Segments[k];
            var curve = new EdgeCurveAdaptor(edge);
            var one = new EdgeCurve2DAdaptor(edge, common.Face);
            var two = new EdgeCurve2DAdaptor(edge, run.Other.Face);
            for (int j = k == 0 ? 0 : 1; j <= count; j++)
            {
                double t = a + (b - a) * 0.5 * (1 - Math.Cos(Math.PI * j / count));
                Section section;
                if (k == 0 && j == 0)
                {
                    section = stretch.First;
                }
                else if (k == n - 1 && j == count)
                {
                    section = stretch.Last;
                }
                else
                {
                    curve.D1(t, out Point3D q, out Vector3D d1);
                    V tangent = V.Of(d1);
                    if (tangent.Length < 1e-14 || Solve(common, run.Other, side, radius, V.Of(q), tangent.Unit) is not { } solved)
                    {
                        return null;
                    }
                    section = solved;
                }
                if (!Add(section, Near(common, section.One, one.Value(t).X), Near(run.Other, section.Two, two.Value(t).X)))
                {
                    return null;
                }
            }
        }
        return FitPatch(sections, values, common, run.Other, side, radius);

        bool Add(Section section, (double U, double V) on1, (double U, double V) on2)
        {
            sections.Add(section);
            double w = Weight(section, out V middle);
            if (w < 0.2)
            {
                return false;
            }
            V m = middle * w;
            values.AddRange([section.One.X, section.One.Y, section.One.Z, m.X, m.Y, m.Z, w, section.Two.X, section.Two.Y, section.Two.Z,
                             on1.U, on1.V, on2.U, on2.V]);
            return true;
        }
    }

    // A piece of the sphere: its sections the great arcs from the contact on the common face to points of the circle where it
    // touches the cylinder, from the face before to the face after
    private static Patch? FitPin(Pin pin, Support common, double side, double radius)
    {
        double target = Target(radius);
        for (int count = FirstSections; count <= MostSections;)
        {
            Cancellation.ThrowIfRequested();
            if (PinThrough(pin, common, side, radius, count) is not { } patch)
            {
                return null;
            }

            if (patch.Error <= target)
            {
                return patch;
            }
            count = Math.Max(count + 1, (int)Math.Ceiling(count * 1.15 * Math.Pow(patch.Error / target, 0.25)));
        }
        return null;
    }

    private static Patch? PinThrough(Pin pin, Support common, double side, double radius, int count)
    {
        Support k = pin.Run.Other;
        var sections = new List<Section>();
        var values = new List<double>();
        (double U, double V) foot = common.Parameters(pin.Foot);
        double reference = new EdgeCurve2DAdaptor(pin.SeamIn, k.Face).Value(pin.SeamInT).X;
        for (int j = 0; j <= count; j++)
        {
            double angle = pin.From + (pin.To - pin.From) * 0.5 * (1 - Math.Cos(Math.PI * j / count));
            V on = j == 0 ? pin.In : j == count ? pin.Out : pin.Centre + (k.X * Math.Cos(angle) + k.Y * Math.Sin(angle)) * radius;
            var section = new Section(on, pin.Centre, pin.Foot, on);
            sections.Add(section);
            double w = Weight(section, out V middle);
            if (w < 0.2)
            {
                return null;
            }
            (double u2, double v2) = Near(k, on, reference);
            reference = u2;
            V m = middle * w;
            values.AddRange([pin.Foot.X, pin.Foot.Y, pin.Foot.Z, m.X, m.Y, m.Z, w, on.X, on.Y, on.Z, foot.U, foot.V, u2, v2]);
        }
        return FitPatch(sections, values, common, k, side, radius, true);
    }

    private static Patch? FitPatch(List<Section> sections, List<double> values, Support one, Support two, double side, double radius, bool point = false)
    {
        int n = sections.Count;
        var taus = new double[n];
        for (int i = 1; i < n; i++)
        {
            double step = (sections[i].Edge - sections[i - 1].Edge).Length;
            if (step <= 0)
            {
                return null;
            }
            taus[i] = taus[i - 1] + step;
        }
        if (Interpolate([.. values], taus, point) is not { } fit)
        {
            return null;
        }
        double error = DeviationOf(fit.Surface, fit.Curve1, fit.Curve2, fit.OnOne, fit.OnTwo, taus, one, two, side, radius);
        return new Patch(fit, taus, [.. sections], error, one, two);
    }
}
