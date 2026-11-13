// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Splines = Kerf.Engine.Geometry.Splines.BSplCLib;

namespace Kerf.Fillets;

// Stage 4a's geometry: the sections, the surface through them, the faces, and the ball test (FilletLimits.Blends.cs)
internal static partial class FilletLimits
{
    // A ball's centre and its two contacts, for a point of the edge
    private readonly record struct Section(V Edge, V Centre, V One, V Two);

    // The values interpolated at each section: the contacts, the arc's middle pole times its weight, the weight, and the contacts'
    // parameters on their surfaces
    private const int Dimension = 14;

    private sealed class Blend
    {
        public required Chain Chain { get; init; }
        public required double Radius { get; init; }
        public required BSplineSurface Surface { get; init; }
        public required BSplineCurve3D Curve1 { get; init; }
        public required BSplineCurve3D Curve2 { get; init; }
        public required BSplineCurve2D OnOne { get; init; }
        public required BSplineCurve2D OnTwo { get; init; }
        public required double[] Taus { get; init; }
        public required Section[] Sections { get; init; }
        public required double Error { get; init; }

        public double First => Taus[0];
        public double Last => Taus[^1];

        public static Blend? Make(Chain chain, double radius)
        {
            double target = Target(radius);
            for (int count = FirstSections; count <= MostSections;)
            {
                Cancellation.ThrowIfRequested();
                if (Through(chain, radius, count) is not { } blend)
                {
                    return null;
                }
                if (blend.Error <= target)
                {
                    return blend;
                }

                // The error goes as the fourth power of the step: as many more as that says, and a little over
                count = Math.Max(count + 1, (int)Math.Ceiling(count * 1.15 * Math.Pow(blend.Error / target, 0.25)));
            }
            return null;
        }

        // The sections at count steps of each edge's parameter, closer at its ends, and the surface and curves through them
        private static Blend? Through(Chain chain, double radius, int count)
        {
            var sections = new List<Section>();
            var values = new List<double>();
            for (int k = 0; k < chain.Edges.Length; k++)
            {
                Edge edge = chain.Edges[k];
                Tool.Range(edge, out double a, out double b);
                var one = new EdgeCurve2DAdaptor(edge, chain.One.Face);
                var two = new EdgeCurve2DAdaptor(edge, chain.Two.Face);
                for (int j = k == 0 ? 0 : 1; j <= count; j++)
                {
                    double s = 0.5 * (1 - Math.Cos(Math.PI * j / count));
                    double t = chain.Forward[k] ? a + (b - a) * s : b - (b - a) * s;
                    bool end = (k == 0 && j == 0) || (k == chain.Edges.Length - 1 && j == count);
                    if (SectionAt(chain, radius, edge, t, end) is not { } section)
                    {
                        return null;
                    }
                    sections.Add(section);
                    double w = Weight(section, out V middle);
                    if (w < 0.2)
                    {
                        return null;
                    }
                    (double u1, double v1) = Near(chain.One, section.One, one.Value(t).X);
                    (double u2, double v2) = Near(chain.Two, section.Two, two.Value(t).X);
                    V m = middle * w;
                    values.AddRange([section.One.X, section.One.Y, section.One.Z, m.X, m.Y, m.Z, w, section.Two.X, section.Two.Y,
                                     section.Two.Z, u1, v1, u2, v2]);
                }
            }
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
            if (Interpolate([.. values], taus) is not { } fit)
            {
                return null;
            }
            var blend = new Blend
            {
                Chain = chain, Radius = radius, Surface = fit.Surface, Taus = taus, Sections = [.. sections], Error = 0,
                Curve1 = fit.Curve1, Curve2 = fit.Curve2, OnOne = fit.OnOne, OnTwo = fit.OnTwo,
            };
            return blend.With(blend.Deviation());
        }

        private Blend With(double error) => new()
        {
            Chain = Chain, Radius = Radius, Surface = Surface, Curve1 = Curve1, Curve2 = Curve2, OnOne = OnOne, OnTwo = OnTwo, Taus = Taus,
            Sections = Sections, Error = error,
        };

        // How far, between the sections, the surface is from a ball's and the curves from their surfaces and each other
        private double Deviation() => DeviationOf(Surface, Curve1, Curve2, OnOne, OnTwo, Taus, Chain.One, Chain.Two, Chain.Side, Radius);
    }

    // The surface through the sections' arcs, its two contact curves and their curves on the supports, all in tau
    private sealed record Fitted(BSplineSurface Surface, BSplineCurve3D Curve1, BSplineCurve3D Curve2, BSplineCurve2D OnOne, BSplineCurve2D OnTwo);

    // The values at each section (Dimension of them) through a cubic in tau
    private static Fitted? Interpolate(double[] poles, double[] taus, bool fixedOne = false)
    {
        int n = taus.Length;
        double[] firstValues = poles[..Dimension];

        // A cubic through every section, not-a-knot: the knots at the sections but the second and the second last
        var flat = new NCollection_Array1<double>(1, n + 4);
        for (int i = 1; i <= 4; i++)
        {
            flat.SetValue(i, taus[0]);
            flat.SetValue(n + i, taus[n - 1]);
        }
        for (int i = 2; i <= n - 3; i++)
        {
            flat.SetValue(i + 3, taus[i]);
        }
        var parameters = new NCollection_Array1<double>(1, n);
        var orders = new NCollection_Array1<int>(1, n);
        for (int i = 0; i < n; i++)
        {
            parameters.SetValue(i + 1, taus[i]);
            orders.SetValue(i + 1, 0);
        }
        Splines.Interpolate(3, flat, parameters, orders, Dimension, poles, out int problem);
        if (problem != 0)
        {
            return null;
        }

        // A first contact that's one point (a pin's, unit 15X) is that point at every pole, not that within rounding
        if (fixedOne)
        {
            for (int i = 0; i < n; i++)
            {
                poles[i * Dimension] = firstValues[0];
                poles[i * Dimension + 1] = firstValues[1];
                poles[i * Dimension + 2] = firstValues[2];
                poles[i * Dimension + 10] = firstValues[10];
                poles[i * Dimension + 11] = firstValues[11];
            }
        }

        double[] knots = [taus[0], .. taus[2..(n - 2)], taus[n - 1]];
        int[] multiplicities = [4, .. Enumerable.Repeat(1, n - 4), 4];
        var grid = new Point3D[n * 3];
        var weights = new double[n * 3];
        var row1 = new Point3D[n];
        var row2 = new Point3D[n];
        var uv1 = new Point2D[n];
        var uv2 = new Point2D[n];
        for (int i = 0; i < n; i++)
        {
            ReadOnlySpan<double> p = poles.AsSpan(i * Dimension, Dimension);
            double w = p[6];
            if (w <= 0)
            {
                return null;
            }
            grid[3 * i] = row1[i] = new Point3D(p[0], p[1], p[2]);
            grid[3 * i + 1] = new Point3D(p[3] / w, p[4] / w, p[5] / w);
            grid[3 * i + 2] = row2[i] = new Point3D(p[7], p[8], p[9]);
            (weights[3 * i], weights[3 * i + 1], weights[3 * i + 2]) = (1, w, 1);
            uv1[i] = new Point2D(p[10], p[11]);
            uv2[i] = new Point2D(p[12], p[13]);
        }
        var surface = new BSplineSurface(new Grid<Point3D>(grid, n, 3), knots, [0.0, 1.0], multiplicities, [3, 3], 3, 2,
                                         weights: new Grid<double>(weights, n, 3));
        return new Fitted(surface, new BSplineCurve3D(row1, knots, multiplicities, 3), new BSplineCurve3D(row2, knots, multiplicities, 3),
                       new BSplineCurve2D(uv1, knots, multiplicities, 3), new BSplineCurve2D(uv2, knots, multiplicities, 3));
    }

    // How far, between the sections, the surface is from a ball's and the curves from their surfaces and each other
    private static double DeviationOf(BSplineSurface surface, BSplineCurve3D curve1, BSplineCurve3D curve2, BSplineCurve2D onOne, BSplineCurve2D onTwo,
                                      double[] taus, Support one, Support two, double side, double radius)
    {
        var adaptor = new SurfaceAdaptor(surface);
        double worst = 0;
        for (int i = 0; i + 1 < taus.Length; i++)
        {
            double tau = 0.5 * (taus[i] + taus[i + 1]);
            V c1 = V.Of(curve1.Value(tau)), c2 = V.Of(curve2.Value(tau));
            worst = Math.Max(worst, Math.Abs(one.Distance(c1)));
            worst = Math.Max(worst, Math.Abs(two.Distance(c2)));
            worst = Math.Max(worst, (c1 - V.Of(At(one, onOne.Value(tau)))).Length);
            worst = Math.Max(worst, (c2 - V.Of(At(two, onTwo.Value(tau)))).Length);
            if (BallError(adaptor, tau, 0.5, one, two, side, radius, null) is { } error)
            {
                worst = Math.Max(worst, error);
            }
            worst = Math.Max(worst, Math.Max(Gap(adaptor, tau, 0, one), Gap(adaptor, tau, 1, two)));
        }
        return worst;
    }

    // The most a fillet built here may be from the true one, in the model's units: the kernel's confusion, or a part in a billion
    // of the radius on a large one, where that's more
    private static double Target(double radius) => Math.Max(Tolerance, 1e-9 * radius);

    // A point of a support from its parameters
    private static Point3D At(Support support, Point2D uv)
    {
        if (support.IsExtrusion)
        {
            return support.At(uv.X, uv.Y, out _).Point;
        }
        V p = support.IsCylinder
            ? support.Origin + (support.X * Math.Cos(uv.X) + support.Y * Math.Sin(uv.X)) * support.Rho + support.Axis * uv.Y
            : support.Origin + support.X * uv.X + support.Y * uv.Y;
        return p.Point;
    }

    // A point's parameters on a support, a cylinder's angle in the turn nearest the edge's
    private static (double U, double V) Near(Support support, V p, double reference)
    {
        (double u, double v) = support.Parameters(p);
        if (support.IsCylinder)
        {
            u += 2 * Math.PI * Math.Round((reference - u) / (2 * Math.PI));
        }
        return (u, v);
    }

    // The arc's middle pole and weight, as a rational quadratic
    private static double Weight(Section section, out V middle)
    {
        V a = section.One - section.Centre, b = section.Two - section.Centre;
        double cos = Math.Clamp(a.Dot(b) / (a.Length * b.Length), -1, 1);
        middle = section.Centre + (a + b) / (1 + cos);
        return Math.Sqrt(0.5 * (1 + cos));
    }

    // The ball at the radius from both surfaces, its centre in the plane square to the edge at t; where the faces are tangent,
    // at the end of the chain, both contacts are the edge's point
    private static Section? SectionAt(Chain chain, double radius, Edge edge, double t, bool end)
    {
        var curve = new EdgeCurveAdaptor(edge);
        curve.D1(t, out Point3D point, out Vector3D d1);
        V q = V.Of(point);
        double side = chain.Side * radius;
        V n1 = chain.One.Normal(q), n2 = chain.Two.Normal(q);
        if (end)
        {
            return new Section(q, q + n1 * side, q, q);
        }
        V tangent = V.Of(d1);
        if (tangent.Length < 1e-14)
        {
            return null;
        }
        return Solve(chain.One, chain.Two, chain.Side, radius, q, tangent.Unit);
    }

    // The ball at the radius from both supports, its centre in the plane through q square to the tangent
    private static Section? Solve(Support one, Support two, double sideOf, double radius, V q, V tangent)
    {
        double side = sideOf * radius;
        V n1 = one.Normal(q), n2 = two.Normal(q);
        V e1 = (n1 - tangent * n1.Dot(tangent)).Unit;
        V e2 = tangent.Cross(e1);
        V c = q + (n1 + n2) * (side / (1 + n1.Dot(n2)));
        double scale = Math.Max(1, radius);
        for (int iteration = 0; iteration < 60; iteration++)
        {
            double f1 = one.Distance(c) - side, f2 = two.Distance(c) - side;
            if (Math.Abs(f1) + Math.Abs(f2) < 1e-14 * scale)
            {
                break;
            }
            V g1 = one.Normal(c), g2 = two.Normal(c);
            double a11 = g1.Dot(e1), a12 = g1.Dot(e2), a21 = g2.Dot(e1), a22 = g2.Dot(e2);
            double det = a11 * a22 - a12 * a21;
            if (Math.Abs(det) < 1e-300)
            {
                return null;
            }
            double da = (f1 * a22 - f2 * a12) / det, db = (a11 * f2 - a21 * f1) / det;
            c -= e1 * da + e2 * db;
            if (iteration == 59)
            {
                return null;
            }
        }
        if ((c - q).Length > 10 * radius)
        {
            return null;
        }
        return new Section(q, c, one.Foot(c), two.Foot(c));
    }

    // How far the ball put back at a point of the fillet's surface is from touching both supports, or, on a rail, how far the point
    // is from its support and the surface from tangent to it, as a length at the radius. Null where the surface's normal isn't
    // defined, as where it narrows to nothing
    private static double? BallError(SurfaceAdaptorBase surface, double tau, double s, Chain chain, double radius, bool? reversed) =>
        BallError(surface, tau, s, chain.One, chain.Two, chain.Side, radius, reversed);

    private static double? BallError(SurfaceAdaptorBase surface, double tau, double s, Support one, Support two, double sideOf, double radius, bool? reversed)
    {
        surface.D1(tau, s, out Point3D point, out Vector3D du, out Vector3D dv);
        V normal = V.Of(du).Cross(V.Of(dv));
        if (normal.Length <= 1e-9 * V.Of(du).Length * V.Of(dv).Length || V.Of(dv).Length < 1e-9 * radius)
        {
            return null;
        }
        V p = V.Of(point);

        // The centre is inside the solid for a convex edge, outside for a concave one, along the face's outward normal
        double side = sideOf * radius;
        double Off(bool flipped)
        {
            V outward = normal.Unit * (flipped ? -1 : 1);
            V centre = p + (sideOf < 0 ? -outward : outward) * radius;
            return Math.Max(Math.Abs(one.Distance(centre) - side), Math.Abs(two.Distance(centre) - side));
        }

        // Where the face's orientation isn't settled yet, the side the ball is nearer
        return reversed is { } r ? Off(r) : Math.Min(Off(false), Off(true));
    }

    // The new faces put in: each chain's fillet, and its two faces with the contact lines in place of its edges
    private static (Shape Shape, History History)? Assemble(Shape shape, List<Blend> blends)
    {
        var builder = new BrepBuilder();
        var contacts = new Dictionary<Shape, List<(Chain Chain, Edge Contact)>>(ShapeHasher.Instance);
        var fillets = new Dictionary<Shape, List<Face>>(ShapeHasher.Instance);
        var made = new List<(Chain Chain, Face Fillet)>();
        foreach (Blend blend in blends)
        {
            Chain chain = blend.Chain;
            double tolerance = Math.Max(Tolerance, blend.Error);
            var fillet = new Face();
            builder.MakeFace(ref fillet, blend.Surface, tolerance);
            Edge Contact(BSplineCurve3D curve, BSplineCurve2D on, Face face, double v)
            {
                var edge = new Edge();
                builder.MakeEdge(ref edge, curve, tolerance);
                builder.Add(edge, chain.Start.Oriented(Orientation.Forward));
                builder.Add(edge, chain.End.Oriented(Orientation.Reversed));
                builder.UpdateEdge(edge, on, face, tolerance);
                builder.UpdateEdge(edge, new LineCurve2D(new Point2D(0, v), new Direction2D(1, 0)), fillet, tolerance);
                builder.Range(edge, blend.First, blend.Last);
                return edge;
            }
            Edge Degenerate(Vertex at, double tau, double from, double to)
            {
                var edge = new Edge();
                builder.MakeEdge(ref edge);
                builder.UpdateEdge(edge, new LineCurve2D(new Point2D(tau, from), new Direction2D(0, to - from)), fillet, tolerance);
                builder.Range(edge, fillet, 0, 1);
                builder.Add(edge, at.Oriented(Orientation.Forward));
                builder.Add(edge, at.Oriented(Orientation.Reversed));
                builder.Degenerated(edge, true);
                return edge;
            }
            Edge one = Contact(blend.Curve1, blend.OnOne, chain.One.Face, 0);
            Edge two = Contact(blend.Curve2, blend.OnTwo, chain.Two.Face, 1);
            var wire = new Wire();
            builder.MakeWire(ref wire);
            builder.Add(wire, one);
            builder.Add(wire, Degenerate(chain.End, blend.Last, 0, 1));
            builder.Add(wire, two.Reversed());
            builder.Add(wire, Degenerate(chain.Start, blend.First, 1, 0));
            wire.Closed(true);
            builder.Add(fillet, wire);

            // Its outward side away from the ball's centre on a convex edge, towards it on a concave one
            Section middle = blend.Sections[blend.Sections.Length / 2];
            var adaptor = new SurfaceAdaptor(blend.Surface);
            adaptor.D1(blend.Taus[blend.Sections.Length / 2], 0.5, out Point3D p, out Vector3D du, out Vector3D dv);
            V away = V.Of(p) - middle.Centre;
            if (V.Of(du).Cross(V.Of(dv)).Dot(away) * -chain.Side < 0)
            {
                fillet = ShapeCasts.Face(fillet.Reversed());
            }
            Add(contacts, chain.One.Face, (chain, one));
            Add(contacts, chain.Two.Face, (chain, two));
            Add(fillets, chain.One.Face, fillet);
            made.Add((chain, fillet));
        }

        var reshape = new ReShape();
        var replaced = new List<(Face Old, Face New)>();

        // The new faces go in beside a face replaced as the shell holds it, so they're turned as it is (decision 2170)
        var held = new Dictionary<Shape, Orientation>(ShapeHasher.Instance);
        for (var explorer = new Explorer(shape, ShapeType.Face); explorer.More(); explorer.Next())
        {
            held.TryAdd(explorer.Current(), explorer.Current().Orientation);
        }
        foreach ((Shape key, List<(Chain Chain, Edge Contact)> those) in contacts)
        {
            Face face = ShapeCasts.Face(key.Oriented(Orientation.Forward));
            Face copy = (Face)face.EmptyCopied();
            for (var wires = new Explorer(face, ShapeType.Wire); wires.More(); wires.Next())
            {
                var wire = new Wire();
                builder.MakeWire(ref wire);
                var done = new HashSet<Chain>();
                for (var edges = new Explorer(wires.Current(), ShapeType.Edge); edges.More(); edges.Next())
                {
                    Edge edge = ShapeCasts.Edge(edges.Current());
                    (Chain Chain, Edge Contact)? on = null;
                    int index = -1;
                    foreach ((Chain chain, Edge contact) in those)
                    {
                        index = Array.FindIndex(chain.Edges, e => e.IsSame(edge));
                        if (index >= 0)
                        {
                            on = (chain, contact);
                            break;
                        }
                    }
                    if (on is not { } found)
                    {
                        builder.Add(wire, edge);
                        continue;
                    }
                    if (!done.Add(found.Chain))
                    {
                        continue;
                    }
                    bool along = (edge.Orientation == Orientation.Forward) == found.Chain.Forward[index];
                    builder.Add(wire, along ? found.Contact : found.Contact.Reversed());
                }
                wire.Closed(true);
                builder.Add(copy, wire);
            }
            replaced.Add((face, copy));
            if (fillets.TryGetValue(key, out List<Face>? added))
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
        foreach ((Chain chain, Face fillet) in made)
        {
            foreach (Edge edge in chain.Edges)
            {
                history.AddGenerated(edge, fillet);
                history.Remove(edge);
            }
        }

        // A fillet's edges are coded tangent, as ChFi3d's are (decision 1719)
        Shape[] newEdges = [.. made.SelectMany(m => m.Fillet.AllEdges()).Select(static e => (Shape)e)];
        result = BrepLibrary.RegularityEncoded(result, newEdges, out History coded);
        history.Merge(coded);
        return (result, history);
    }

    private static void Add<T>(Dictionary<Shape, List<T>> map, Shape key, T value)
    {
        if (!map.TryGetValue(key, out List<T>? list))
        {
            map[key] = list = [];
        }
        list.Add(value);
    }

    // The ball test, apart from how the fillet was built: at points of each new face between its sections the ball put back touches
    // both supports, along its edges the face lies on them and is tangent to them, at its middle the ball reaches the result's
    // faces at its radius and no nearer, and the volume is the input's with the regions between the supports and the balls
    private static bool BallHolds(Shape input, Shape result, List<Blend> blends, double radius)
    {
        Face[] faces = [.. result.AllFaces()];
        var boundary = new Compound();
        var builder = new BrepBuilder();
        builder.MakeCompound(ref boundary);
        foreach (Face f in faces)
        {
            builder.Add(boundary, f);
        }
        double change = 0;
        foreach (Blend blend in blends)
        {
            Chain chain = blend.Chain;
            if (faces.FirstOrDefault(f => ReferenceEquals(Tool.Surface(f), blend.Surface)) is not { IsNull: false } face)
            {
                return false;
            }
            double allowed = 2 * Target(radius);
            bool reversed = face.Orientation == Orientation.Reversed;
            var surface = new FaceSurfaceAdaptor(face);
            for (int i = 0; i + 1 < blend.Taus.Length; i++)
            {
                double tau = 0.5 * (blend.Taus[i] + blend.Taus[i + 1]);
                foreach (double s in (ReadOnlySpan<double>)[0.25, 0.5, 0.75])
                {
                    if (BallError(surface, tau, s, chain, radius, reversed) is { } error && error > allowed)
                    {
                        return false;
                    }
                }
                if (!Tangent(surface, tau, 0, chain.One, reversed, allowed) || !Tangent(surface, tau, 1, chain.Two, reversed, allowed))
                {
                    return false;
                }
            }
            {
                Section section = blend.Sections[blend.Sections.Length / 2];
                var vertex = new Vertex();
                builder.MakeVertex(ref vertex, section.Centre.Point, Tolerance);
                var distance = new DistanceShapeShape(vertex, boundary);
                if (!distance.IsDone || Math.Abs(distance.Value() - radius) > allowed + Tolerance)
                {
                    return false;
                }
            }
            change += chain.Side * RegionVolume(chain, radius);
        }
        double expected = Measured(input) + change;
        double area = Integral.Area(result);
        // NaN, a volume that couldn't be confirmed, passes no comparison: the proof fails (decision 34806)
        return Math.Abs(Measured(result) - expected) <= Allowed(expected, true) + Tolerance * area;
    }

    // A volume to a part in a hundred million, as fixed points aren't exact on the fillet's B-spline; NaN where it couldn't be confirmed
    private static double Measured(Shape shape) => Kerf.Measurement.MeasureOperations.VolumeOrNaN(shape, Precise);

    private static readonly Kerf.Measurement.MassOptions Precise = new() { Tolerance = 1e-8 };

    // How far from tangent to its support the surface is on a rail, as the gap it opens across the section: either way round, as
    // the face's orientation isn't settled when it's built
    private static double Gap(SurfaceAdaptorBase surface, double tau, double s, Support support)
    {
        surface.D1(tau, s, out Point3D point, out Vector3D du, out Vector3D dv);
        V normal = V.Of(du).Cross(V.Of(dv));
        if (normal.Length <= 1e-9 * V.Of(du).Length * V.Of(dv).Length)
        {
            return 0;
        }
        V on = support.Normal(V.Of(point));
        return Math.Min((normal.Unit - on).Length, (normal.Unit + on).Length) * V.Of(dv).Length;
    }

    // On a rail: the point on its support, and the face's normal its support's, to the angle that opens no more than the allowed
    // distance across the section's width, as near a pinch the section has next to none
    private static bool Tangent(SurfaceAdaptorBase surface, double tau, double s, Support support, bool reversed, double allowed)
    {
        surface.D1(tau, s, out Point3D point, out Vector3D du, out Vector3D dv);
        V p = V.Of(point);
        if (Math.Abs(support.Distance(p)) > allowed)
        {
            return false;
        }
        V normal = V.Of(du).Cross(V.Of(dv));
        if (normal.Length <= 1e-9 * V.Of(du).Length * V.Of(dv).Length)
        {
            return true;
        }
        V outward = normal.Unit * (reversed ? -1 : 1);
        return (outward - support.Normal(p)).Length * V.Of(dv).Length <= allowed;
    }

    // The region between the two supports and the ball, by the divergence theorem over its three sides, each parametrised by the
    // edge's parameter and a straight line across the support's parameters, or the arc: Gauss's points on the exact sections
    private static double RegionVolume(Chain chain, double radius)
    {
        ReadOnlySpan<double> nodes = [-0.9602898564975363, -0.7966664774136267, -0.5255324099163290, -0.1834346424956498,
                                      0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363];
        ReadOnlySpan<double> weights = [0.1012285362903763, 0.2223810344533745, 0.3137066458778873, 0.3626837833783620,
                                        0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763];
        const int panels = 32;
        V origin = V.Of(Tool.Pnt(chain.Start));
        double total = 0;
        for (int k = 0; k < chain.Edges.Length; k++)
        {
            Edge edge = chain.Edges[k];
            Tool.Range(edge, out double a, out double b);
            double h = 1e-5 * (b - a);
            double sum = 0;
            for (int panel = 0; panel < panels; panel++)
            {
                double lo = a + (b - a) * panel / panels, hi = a + (b - a) * (panel + 1) / panels;
                for (int i = 0; i < nodes.Length; i++)
                {
                    double t = 0.5 * (lo + hi) + 0.5 * (hi - lo) * nodes[i];
                    if (SectionAt(chain, radius, edge, t - h, false) is not { } before || SectionAt(chain, radius, edge, t, false) is not { } here
                        || SectionAt(chain, radius, edge, t + h, false) is not { } after)
                    {
                        return double.NaN;
                    }
                    double flux = 0;
                    for (int j = 0; j < nodes.Length; j++)
                    {
                        double w = 0.5 + 0.5 * nodes[j];
                        flux += weights[j] * 0.5 * (Across(chain.One, here.Edge, here.One, w, before, after, true, h, origin)
                                                     + Arc(here, before, after, w, h, origin)
                                                     + Across(chain.Two, here.Two, here.Edge, w, before, after, false, h, origin));
                    }
                    sum += weights[i] * 0.5 * (hi - lo) * flux;
                }
            }
            total += chain.Forward[k] ? sum : -sum;
        }
        return Math.Abs(total) / 3;
    }

    // (X - O) . (X_t x X_w) on a support, along the straight line in its parameters from the first point to the second
    private static double Across(Support support, V from, V to, double w, Section before, Section after, bool fromEdge, double h, V origin)
    {
        V At(V p, V q, out V dw)
        {
            (double u0, double v0) = support.Parameters(p);
            (double u1, double v1) = support.Parameters(q);
            if (support.IsCylinder)
            {
                u1 += 2 * Math.PI * Math.Round((u0 - u1) / (2 * Math.PI));
            }
            double u = u0 + w * (u1 - u0), v = v0 + w * (v1 - v0);
            if (support.IsCylinder)
            {
                V radial = support.X * Math.Cos(u) + support.Y * Math.Sin(u);
                dw = (support.Y * Math.Cos(u) - support.X * Math.Sin(u)) * (support.Rho * (u1 - u0)) + support.Axis * (v1 - v0);
                return support.Origin + radial * support.Rho + support.Axis * v;
            }
            dw = support.X * (u1 - u0) + support.Y * (v1 - v0);
            return support.Origin + support.X * u + support.Y * v;
        }
        V x = At(from, to, out V dw);
        V xb = fromEdge ? At(before.Edge, before.One, out _) : At(before.Two, before.Edge, out _);
        V xa = fromEdge ? At(after.Edge, after.One, out _) : At(after.Two, after.Edge, out _);
        V dt = (xa - xb) / (2 * h);
        return (x - origin).Dot(dt.Cross(dw));
    }

    // The same on the ball's arc from the first contact to the second
    private static double Arc(Section here, Section before, Section after, double w, double h, V origin)
    {
        V Point(Section s, out V dw)
        {
            V a = s.One - s.Centre, b = s.Two - s.Centre;
            double r = a.Length;
            double angle = Math.Acos(Math.Clamp(a.Dot(b) / (r * b.Length), -1, 1));
            if (angle < 1e-12)
            {
                dw = default;
                return s.One;
            }
            V e = a / r, f = (b - e * b.Dot(e)).Unit;
            dw = (f * Math.Cos(w * angle) - e * Math.Sin(w * angle)) * (r * angle);
            return s.Centre + (e * Math.Cos(w * angle) + f * Math.Sin(w * angle)) * r;
        }
        V x = Point(here, out V dw);
        V dt = (Point(after, out _) - Point(before, out _)) / (2 * h);
        return (x - origin).Dot(dt.Cross(dw));
    }

    // A small vector of doubles for the arithmetic above
    private readonly record struct V(double X, double Y, double Z)
    {
        public static V Of(Point3D p) => new(p.X, p.Y, p.Z);
        public static V Of(Vector3D v) => new(v.X, v.Y, v.Z);
        public static V Of(Direction3D d) => new(d.X, d.Y, d.Z);
        public Point3D Point => new(X, Y, Z);
        public double Length => Math.Sqrt(X * X + Y * Y + Z * Z);
        public V Unit => this / Length;
        public double Dot(V o) => X * o.X + Y * o.Y + Z * o.Z;
        public V Cross(V o) => new(Y * o.Z - Z * o.Y, Z * o.X - X * o.Z, X * o.Y - Y * o.X);
        public static V operator +(V a, V b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static V operator -(V a, V b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static V operator -(V a) => new(-a.X, -a.Y, -a.Z);
        public static V operator *(V a, double s) => new(a.X * s, a.Y * s, a.Z * s);
        public static V operator /(V a, double s) => new(a.X / s, a.Y / s, a.Z / s);
    }
}
