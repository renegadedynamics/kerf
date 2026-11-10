// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Fillets;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

/// <summary>
/// A chamfer whose corner ChFi3d couldn't build, built with its chamfer faces carried on until they meet (unit 15U; decisions
/// 2137, 2144): each stripe ChFi3d computed before its corner failed, its surface extended past the corner and across its
/// contacts, a general fuse of those faces with the input's, and the pieces kept that bound the region the chamfered shape is.
/// </summary>
/// <remarks>
/// <para>
/// <b>The region.</b> Beside an edge a chamfer is the input with the side of its face away from the material taken off (a
/// convex edge) or added (a concave one). At a corner where chamfers meet, the edges there that aren't chamfered say how they
/// combine: where those are convex, the material is on the material side of all the chamfers (convex chamfers cut in turn;
/// concave ones wrapping a convex corner fill only where they all do); where they're concave, of any. Near a corner every
/// chamfer that ends there counts, its extended surface whole; elsewhere a chamfer counts only beside its own edge. The input's
/// material is the side of its nearest face behind its normal, as the faces are oriented in a shell.
/// </para>
/// <para>
/// <b>The pieces.</b> A piece of the fuse is kept where the region is on one side of it and not the other, turned to face
/// out of it. The fuse leaves the cuts that end inside a face (where an extended surface stops) as internal edges; they're
/// taken out.
/// </para>
/// <para>
/// <b>What's built:</b> open shells and faces, with every corner of one kind (its other edges all convex or all concave),
/// the chamfers there all convex or all concave, a corner with one chamfer only where it and the corner are convex; every
/// chamfer face one of ChFi3d's B-spline stripes, its contacts its u-isos. A solid isn't: its proof would need its volume
/// against an independent integral, which isn't built. What doesn't fit gives null, and ChFi3d's failure stands.
/// </para>
/// <para>
/// <b>The proof</b> (<see cref="Proved"/>), of the result alone and the chamfer's specs, not of how it was made: every free
/// edge runs along one of the input's, or is a chamfer face's end between two; and where a chamfer face meets one of its edge's
/// faces beside the edge, the contact is at the chamfer's distance on that face, and on the reference face at the chamfer's
/// angle (ChFi3d's: in the plane square to the edge). The operation then checks validity, tolerances and the crossings
/// (decision 2125) as for any result. Everything is local to the call, on frozen shapes.
/// </para>
/// </remarks>
internal static class ChamferMeet
{
    // How far past a corner a chamfer's surface is carried, and how far across its contacts, in the widest chamfer's widths
    private const double Past = 2;

    private const double Across = 1;

    // Within this many widths of a corner, the corner's rule holds; a chamfer counts beside its edge out to this many widths
    private const double Near = 2;

    // The fewest contact points checked on each chamfer
    private const int FewestChecked = 5;

    private sealed class Band
    {
        public required BSplineSurface Original { get; init; }

        public required double UMax { get; init; }      // the contacts: u = 0 on S1, u = UMax on S2

        public required Point2D A1 { get; init; }       // the contacts' ends on the surface: A at the first end, B at the last

        public required Point2D A2 { get; init; }

        public required Point2D B1 { get; init; }

        public required Point2D B2 { get; init; }

        public required double Width { get; init; }

        public required bool Convex { get; init; }

        public required Vertex FirstCorner { get; init; }   // the corner at each end, or null where the end is the edge's

        public required Vertex LastCorner { get; init; }

        public required Point3D EdgePoint { get; init; }    // a point of the chamfered edge, beside the band's middle

        public required Edge[] Edges { get; init; }

        public BSplineSurface Surface { get; set; } = null!; // extended

        public double EU0 { get; set; }

        public double EU1 { get; set; }

        public double EV0 { get; set; }

        public double EV1 { get; set; }

        public double EdgeSign { get; set; }

        public Face Face { get; set; }

        public double VLo => Math.Min(A1.Y, A2.Y);

        public double VHi => Math.Max(B1.Y, B2.Y);

        // The face's v range at u: past a corner the extended surface's; at the edge's end the line through the contacts' ends
        public double Lo(double u) => !FirstCorner.IsNull ? EV0 : A1.Y + ((A2.Y - A1.Y) * u / UMax);

        public double Hi(double u) => !LastCorner.IsNull ? EV1 : B1.Y + ((B2.Y - B1.Y) * u / UMax);

        public bool EndsAt(Vertex corner) => FirstCorner.IsSame(corner) || LastCorner.IsSame(corner);
    }

    private readonly record struct Corner(Vertex Vertex, Point3D Point, bool All, Band[] Bands);

    /// <summary>
    /// The chamfer built with its faces carried on until they meet, where ChFi3d's <paramref name="chamfer"/> has computed its
    /// stripes and failed at a corner, and proved; null where this doesn't apply or what it builds isn't proved.
    /// </summary>
    public static EngineOutcome? Build(Shape shape, MakeChamfer chamfer, ChamferSpec[] specs, Face[] referenceFaces, ChamferOptions options)
    {
        if (options.Mode != ChamferMode.Classic || shape.AllSolids().Length != 0 || shape.AllFaces().Length == 0)
        {
            return null;
        }
        try
        {
            return Built(shape, chamfer, specs, referenceFaces);
        }
        catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
        {
            return null;
        }
    }

    private static EngineOutcome? Built(Shape shape, MakeChamfer chamfer, ChamferSpec[] specs, Face[] referenceFaces)
    {
        ChBuilder builder = chamfer.InternalBuilder();
        if (builder._listStripe.IsEmpty() || builder._badstripes.Extent() != 0)
        {
            return null;
        }

        // The corners: where ChFi3d failed, and every end of a stripe that isn't at the edge of the shape
        var corners = new List<Vertex>();
        foreach (Shape v in builder._badvertices)
        {
            corners.Add(ShapeCasts.Vertex(v));
        }
        foreach (Stripe stripe in builder._listStripe)
        {
            Spine spine = stripe.Spine()!;
            if (spine.FirstStatus() != State.FreeBoundary && !corners.Any(v => v.IsSame(spine.FirstVertex())))
            {
                corners.Add(spine.FirstVertex());
            }
            if (spine.LastStatus() != State.FreeBoundary && !corners.Any(v => v.IsSame(spine.LastVertex())))
            {
                corners.Add(spine.LastVertex());
            }
        }
        if (corners.Count == 0)
        {
            return null;
        }

        Face[] faces = shape.AllFaces().ToArray();
        var chamfered = new List<Edge>();
        var bands = new List<Band>();
        foreach (Stripe stripe in builder._listStripe)
        {
            Cancellation.ThrowIfRequested();
            if (Bands(shape, faces, builder, stripe, corners) is not { } own)
            {
                return null;
            }
            bands.AddRange(own);
            Spine spine = stripe.Spine()!;
            for (int j = 1; j <= spine.EdgeCount; j++)
            {
                chamfered.Add(spine.Edges(j));
            }
        }

        // How the chamfers combine at each corner
        var cornerRules = new List<Corner>();
        Edge[] edges = shape.AllEdges().ToArray();
        foreach (Vertex corner in corners)
        {
            Edge[] others = [.. edges.Where(e => !chamfered.Any(c => c.IsSame(e)) && e.AllVertices().Any(v => v.IsSame(corner)))];
            bool?[] kinds = [.. others.Select(e => IsConvex(faces, e)).Distinct()];
            Band[] at = [.. bands.Where(b => b.EndsAt(corner))];
            if (kinds.Length != 1 || kinds[0] is not { } all || at.Length == 0 || at.Any(b => b.Convex != at[0].Convex)
                || (at.Length == 1 && !(at[0].Convex && all)))
            {
                return null;
            }
            cornerRules.Add(new Corner(corner, corner.Point, all, at));
        }

        // Each surface carried past its corners and across its contacts, its parameters kept, and made a face
        double width = bands.Max(static b => b.Width);
        foreach (Band band in bands)
        {
            BoundedSurface extended = band.Original;
            if (!band.LastCorner.IsNull)
            {
                GeomLib.ExtendSurfByLength(ref extended, Past * width, 1, false, true);
            }
            if (!band.FirstCorner.IsNull)
            {
                GeomLib.ExtendSurfByLength(ref extended, Past * width, 1, false, false);
            }
            GeomLib.ExtendSurfByLength(ref extended, Across * width, 1, true, true);
            GeomLib.ExtendSurfByLength(ref extended, Across * width, 1, true, false);
            if (extended is not BSplineSurface surface)
            {
                return null;
            }
            surface.ParameterBounds(out double u0, out double u1, out double v0, out double v1);
            Point3D middle = band.Original.EvalD0(band.UMax / 2, (band.VLo + band.VHi) / 2);
            if (surface.EvalD0(band.UMax / 2, (band.VLo + band.VHi) / 2).DistanceTo(middle) > Precision.Confusion())
            {
                return null; // the extension moved the parameters
            }
            band.Surface = surface;
            (band.EU0, band.EU1, band.EV0, band.EV1) = (u0, u1, v0, v1);
            band.Face = Polygon(surface, [new(u0, band.Lo(u0)), new(u1, band.Lo(u1)), new(u1, band.Hi(u1)), new(u0, band.Hi(u0))]);
            if (Project(band, band.EdgePoint) is not { } foot)
            {
                return null;
            }
            band.EdgeSign = Math.Sign(new Vector3D(foot.Point, band.EdgePoint).Dot(foot.Normal));
            if (band.EdgeSign == 0)
            {
                return null;
            }
        }

        // The input's material: behind its faces as one shell orients them, the side the bands' middles say
        Face[] oriented = Oriented(shape, faces);
        var near = new NearestFace(oriented);
        int[] signs = [.. bands.Select(b => (b.Convex ? 1 : -1) * Math.Sign(near.Side(b.Original.EvalD0(b.UMax / 2, (b.VLo + b.VHi) / 2)))).Distinct()];
        if (signs.Length != 1 || signs[0] == 0)
        {
            return null;
        }
        int material = signs[0];
        bool Material(Point3D x) => Math.Sign(near.Side(x)) == material;

        // The band's side of a point: true on the chamfered shape's material side; null where it says nothing
        bool? SideOf(Band band, Point3D x, bool nearCorner)
        {
            if (Project(band, x) is not { } foot)
            {
                return null;
            }
            double along = new Vector3D(foot.Point, x).Dot(foot.Normal);
            if (!nearCorner)
            {
                double lo = band.FirstCorner.IsNull ? band.Lo(foot.U) : band.VLo, hi = band.LastCorner.IsNull ? band.Hi(foot.U) : band.VHi;
                double distance = x.DistanceTo(foot.Point);
                if (foot.V < lo || foot.V > hi || distance > Near * width || Math.Abs(along) < 0.99 * distance)
                {
                    return null;
                }
            }
            bool edgeSide = Math.Sign(along) == band.EdgeSign;
            return band.Convex ? !edgeSide : edgeSide;
        }
        bool Region(Point3D x)
        {
            bool inside = Material(x);
            Corner[] close = [.. cornerRules.Where(c => c.Point.DistanceTo(x) < Near * width).OrderBy(c => c.Point.DistanceTo(x))];
            if (close.Length > 0)
            {
                Corner c = close[0];
                bool[] sides = [.. c.Bands.Select(b => SideOf(b, x, true)).OfType<bool>()];
                if (sides.Length == 0)
                {
                    return inside;
                }
                bool held = c.All ? sides.All(static s => s) : sides.Any(static s => s);
                return c.Bands[0].Convex ? inside && held : inside || held;
            }
            foreach (Band band in bands)
            {
                if (SideOf(band, x, false) is { } side && side != band.Convex)
                {
                    inside = !band.Convex;
                }
            }
            return inside;
        }

        // The fuse, and the pieces that bound the region
        var arguments = new NCollection_List<Shape>();
        foreach (Face face in faces)
        {
            arguments.Append(face);
        }
        foreach (Band band in bands)
        {
            arguments.Append(band.Face);
        }
        var fuse = new BuilderAlgo();
        fuse.SetArguments(arguments);
        fuse.SetToFillHistory(true);
        fuse.SetRunParallel(false);
        fuse.Build();
        if (fuse.HasErrors || !fuse.IsDone || fuse.History() is not { } made)
        {
            return null;
        }
        var context = new IntersectionContext();
        double step = 1e-4 * width;
        var kept = new Dictionary<Shape, Face>(ShapeHasher.Instance);
        var result = new BrepBuilder();
        Shell shell = default;
        result.MakeShell(ref shell);
        foreach (Face piece in fuse.Shape().AllFaces())
        {
            Cancellation.ThrowIfRequested();
            Point3D p = default;
            Point2D uv = default;
            if (AlgoTools3D.PointInFace(piece, ref p, ref uv, context) != 0 || Tool.Surface(piece) is not { } surface)
            {
                return null;
            }
            Vector3D n = Normal(surface, uv.X, uv.Y);
            if (piece.Orientation == Orientation.Reversed)
            {
                n = -n;
            }
            bool behind = Region(p - (step * n)), before = Region(p + (step * n));
            if (behind != before)
            {
                Face face = WithoutInternal(behind ? piece : ShapeCasts.Face(piece.Reversed()));
                kept[piece] = face;
                result.Add(shell, face);
            }
        }
        if (kept.Count == 0)
        {
            return null;
        }

        // A face the extended surfaces cut in parts that were all kept is one face again
        var unifier = new UnifySameDomain(shell, true, true, true);
        unifier.SetSafeInputMode(true);
        unifier.AllowInternalEdges(false);
        unifier.Build();
        Shape built = unifier.InternalShape();
        if (built.Type != ShapeType.Shell || Proved(shape, built, specs, referenceFaces, chamfer, builder._tolapp3d) is not true)
        {
            return null;
        }
        History unified = unifier.InternalHistory()!;

        // The history: each input face to its pieces kept, or removed; each chamfered edge to its chamfer's faces
        var history = new History();
        foreach (Face face in faces)
        {
            IReadOnlyList<Shape> pieces = made.Modified(face);
            Face[] own = pieces.Count == 0 ? (kept.TryGetValue(face, out Face same) ? [same] : []) : [.. pieces.Where(kept.ContainsKey).Select(p => kept[p])];
            if (own.Length == 0)
            {
                history.Remove(face);
            }
            foreach (Face piece in own.Where(p => !p.IsSame(face)))
            {
                history.AddModified(face, piece);
            }
        }
        foreach (Band band in bands)
        {
            foreach (Shape piece in made.Modified(band.Face).Where(kept.ContainsKey))
            {
                foreach (Edge edge in band.Edges)
                {
                    history.AddGenerated(edge, kept[piece]);
                }
            }
        }
        history.Merge(unified);
        return new EngineOutcome(true, () => built, history);
    }

    // The bands of one stripe, one for each of its surfaces; null where a surface isn't a B-spline with its contacts its u-isos,
    // or the stripe's faces aren't its edges'
    private static List<Band>? Bands(Shape shape, Face[] faces, ChBuilder builder, Stripe stripe, List<Vertex> corners)
    {
        NCollection_HSequence<SurfData>? set = stripe.SetOfSurfData();
        Spine spine = stripe.Spine()!;
        if (set is null || set.Length() == 0)
        {
            return null;
        }
        Edge[] own = [.. Enumerable.Range(1, spine.EdgeCount).Select(spine.Edges)];
        if (own.Select(e => IsConvex(faces, e)).Distinct().ToArray() is not [{ } convex])
        {
            return null;
        }
        Vertex first = corners.FirstOrDefault(v => v.IsSame(spine.FirstVertex())), last = corners.FirstOrDefault(v => v.IsSame(spine.LastVertex()));
        var bands = new List<Band>();
        for (int k = 1; k <= set.Length(); k++)
        {
            SurfData data = set.Value(k);
            if (builder._ds.Surface(data.Surf()).Surface() is not BSplineSurface surface)
            {
                return null;
            }
            Shape s1 = builder._ds.Shape(data.IndexOfS1()), s2 = builder._ds.Shape(data.IndexOfS2());
            if (!own.Any(e => OnFace(e, s1) && OnFace(e, s2)))
            {
                return null;
            }
            FaceInterference on1 = data.InterferenceOnS1(), on2 = data.InterferenceOnS2();
            if (on1.PCurveOnSurf() is not { } c1 || on2.PCurveOnSurf() is not { } c2)
            {
                return null;
            }
            Point2D a1 = c1.EvalD0(on1.FirstParameter()), b1 = c1.EvalD0(on1.LastParameter());
            Point2D a2 = c2.EvalD0(on2.FirstParameter()), b2 = c2.EvalD0(on2.LastParameter());
            surface.ParameterBounds(out double u0, out double umax, out _, out _);
            if (Math.Abs(u0) > Precision.PConfusion() || Math.Abs(a1.X) > Precision.PConfusion() || Math.Abs(b1.X) > Precision.PConfusion()
                || Math.Abs(a2.X - umax) > Precision.PConfusion() || Math.Abs(b2.X - umax) > Precision.PConfusion())
            {
                return null;
            }
            Point3D middle = surface.EvalD0(umax / 2, (Math.Min(a1.Y, a2.Y) + Math.Max(b1.Y, b2.Y)) / 2);
            Point3D? onEdge = null;
            double nearest = double.MaxValue;
            foreach (Edge e in own)
            {
                if (Tool.Curve(e, out double f, out double l) is not { } curve)
                {
                    continue;
                }
                var projection = new ProjectPointOnCurve(middle, curve, f, l);
                if (projection.NbPoints() > 0 && projection.LowerDistance() < nearest)
                {
                    nearest = projection.LowerDistance();
                    onEdge = projection.NearestPoint();
                }
            }
            if (onEdge is not { } edgePoint)
            {
                return null;
            }
            bands.Add(new Band
            {
                Original = surface,
                UMax = umax,
                A1 = a1,
                A2 = a2,
                B1 = b1,
                B2 = b2,
                Width = surface.EvalD0(0, (a1.Y + b1.Y) / 2).DistanceTo(surface.EvalD0(umax, (a2.Y + b2.Y) / 2)),
                Convex = convex,
                FirstCorner = k == 1 ? first : default,
                LastCorner = k == set.Length() ? last : default,
                EdgePoint = edgePoint,
                Edges = own,
            });
        }
        _ = shape;
        return bands;
    }

    private static bool OnFace(Edge edge, Shape face) => face.AllEdges().Any(e => e.IsSame(edge));

    // Convex (true) or concave (false) at the edge's middle, by its faces as they lie in the shape; null if it hasn't two
    // faces or they're tangent there
    private static bool? IsConvex(Face[] faces, Edge edge)
    {
        Face[] two = [.. faces.Where(f => OnFace(edge, f))];
        if (two.Length != 2 || Tool.Curve(edge, out double first, out double last) is not { } curve)
        {
            return null;
        }
        double t = (first + last) / 2;
        Point3D at = curve.EvalD0(t);
        Edge inFirst = two[0].AllEdges().First(e => e.IsSame(edge)), inSecond = two[1].AllEdges().First(e => e.IsSame(edge));
        AlgoTools3D.GetNormalToFaceOnEdge(inSecond, two[1], t, out Direction3D normal, null);
        Point2D uv = default;
        Point3D into = default;
        if (AlgoTools3D.PointNearEdge(inFirst, two[0], t, 1e-3, ref uv, ref into, new IntersectionContext()) != 0)
        {
            return null;
        }
        var inward = new Vector3D(at, into);
        double s = new Vector3D(normal.X, normal.Y, normal.Z).Dot(inward) / inward.Magnitude;
        return s < -1e-3 ? true : s > 1e-3 ? false : null;
    }

    // The faces as one shell orients them, where they aren't one shell already
    private static Face[] Oriented(Shape shape, Face[] faces)
    {
        if (shape.Type == ShapeType.Shell || shape.AllShells().Length == 1 && shape.AllShells()[0].AllFaces().Length == faces.Length)
        {
            return faces;
        }
        var builder = new BrepBuilder();
        Shell shell = default;
        builder.MakeShell(ref shell);
        foreach (Face face in faces)
        {
            builder.Add(shell, face);
        }
        var fixer = new ShellFixer();
        fixer.FixFaceOrientation(shell);
        return fixer.Shell().AllFaces().ToArray();
    }

    private readonly record struct Foot(double U, double V, Point3D Point, Vector3D Normal);

    // The nearest point of the band's extended surface, with its unit normal
    private static Foot? Project(Band band, Point3D p)
    {
        var projection = new ProjectPointOnSurface(p, band.Surface, band.EU0, band.EU1, band.EV0, band.EV1);
        if (projection.NbPoints() == 0)
        {
            return null;
        }
        int best = 1;
        for (int i = 2; i <= projection.NbPoints(); i++)
        {
            if (projection.Distance(i) < projection.Distance(best))
            {
                best = i;
            }
        }
        projection.Parameters(best, out double u, out double v);
        Vector3D n = Normal(band.Surface, u, v);
        return new Foot(u, v, band.Surface.EvalD0(u, v), n);
    }

    private static Vector3D Normal(Surface surface, double u, double v)
    {
        surface.D1(u, v, out _, out Vector3D du, out Vector3D dv);
        Vector3D n = du.Crossed(dv);
        double length = n.Magnitude;
        return length > 0 ? n / length : n;
    }

    // The side of a point's nearest face: its distance along the face's normal as the face lies, so negative behind it
    private sealed class NearestFace(Face[] faces)
    {
        private readonly Box[] _boxes = [.. faces.Select(static f =>
        {
            var box = new Box();
            Bounds.Add(f, ref box);
            return box;
        })];

        public double Side(Point3D p)
        {
            var vertex = new LibraryMakeVertex(p).InternalVertex();
            var point = new Box();
            point.Add(p);
            double best = double.MaxValue, side = 0;
            foreach (int k in Enumerable.Range(0, faces.Length).OrderBy(k => _boxes[k].Distance(point)))
            {
                if (_boxes[k].Distance(point) > best)
                {
                    break;
                }
                var extrema = new ExtPF(vertex, faces[k]);
                if (!extrema.IsDone)
                {
                    continue;
                }
                for (int n = 1; n <= extrema.ExtCount; n++)
                {
                    double d = Math.Sqrt(extrema.SquareDistance(n));
                    if (d >= best)
                    {
                        continue;
                    }
                    extrema.Parameter(n, out double u, out double v);
                    Vector3D normal = Normal(Tool.Surface(faces[k])!, u, v);
                    if (faces[k].Orientation == Orientation.Reversed)
                    {
                        normal = -normal;
                    }
                    best = d;
                    side = new Vector3D(extrema.Point(n), p).Dot(normal);
                }
            }
            return side;
        }
    }

    // The face of the surface bounded by the polygon of uv
    private static Face Polygon(Surface surface, Point2D[] uv)
    {
        var builder = new BrepBuilder();
        Vertex[] vertices = [.. uv.Select(q => new LibraryMakeVertex(surface.EvalD0(q.X, q.Y)).InternalVertex())];
        var wire = new LibraryMakeWire();
        for (int i = 0; i < uv.Length; i++)
        {
            Point2D a = uv[i], b = uv[(i + 1) % uv.Length];
            var line = new LineCurve2D(a, new Direction2D(b.X - a.X, b.Y - a.Y));
            wire.Add(new LibraryMakeEdge(line, surface, vertices[i], vertices[(i + 1) % uv.Length], 0, a.DistanceTo(b)).InternalEdge());
        }
        Face face = default;
        builder.MakeFace(ref face, surface, Precision.Confusion());
        builder.Add(face, wire.InternalWire());
        BrepLibrary.BuildCurves3d(face);
        return face;
    }

    // The face without the cuts that end inside it, which the fuse leaves as internal edges
    private static Face WithoutInternal(Face face)
    {
        if (!face.AllEdges().Any(static e => e.Orientation is Orientation.Internal or Orientation.External)
            && !face.Explore<Edge>(ShapeType.Edge).Any(static e => e.Orientation is Orientation.Internal or Orientation.External))
        {
            return face;
        }
        var builder = new BrepBuilder();
        Face forward = ShapeCasts.Face(face.Oriented(Orientation.Forward));
        Face copy = ShapeCasts.Face(forward.EmptyCopied());
        foreach (Wire wire in forward.Explore<Wire>(ShapeType.Wire))
        {
            Wire kept = ShapeCasts.Wire(wire.EmptyCopied());
            int count = 0;
            foreach (Edge edge in wire.Explore<Edge>(ShapeType.Edge))
            {
                if (edge.Orientation is not (Orientation.Internal or Orientation.External))
                {
                    builder.Add(kept, edge);
                    count++;
                }
            }
            if (count > 0)
            {
                kept.Closed(true);
                builder.Add(copy, kept);
            }
        }
        return ShapeCasts.Face(copy.Oriented(face.Orientation));
    }

    /// <summary>
    /// True if <paramref name="result"/> is the chamfer the specs ask for, read from it alone: its free edges are the input's
    /// or a chamfer face's ends between them, and its chamfer faces' contacts beside their edges are at the specs' distances and
    /// angles, within <paramref name="tolerance"/> (ChFi3d's approximation) and the edges' own.
    /// </summary>
    internal static bool Proved(Shape input, Shape result, ChamferSpec[] specs, Face[] referenceFaces, MakeChamfer contours, double tolerance)
    {
        Face[] inputFaces = input.AllFaces().ToArray();
        Face[] faces = result.AllFaces().ToArray();
        var context = new IntersectionContext();

        // Each face of the result: the input face it lies on, or none (a chamfer's)
        var origin = new Dictionary<Shape, int>(ShapeHasher.Instance);
        foreach (Face face in faces)
        {
            Point3D p = default;
            Point2D uv = default;
            if (AlgoTools3D.PointInFace(face, ref p, ref uv, context) != 0)
            {
                return false;
            }
            Vertex vertex = new LibraryMakeVertex(p).InternalVertex();
            origin[face] = Array.FindIndex(inputFaces, f =>
            {
                var extrema = new ExtPF(vertex, f);
                return extrema.IsDone && Enumerable.Range(1, extrema.ExtCount).Any(n => extrema.SquareDistance(n) < Precision.SquareConfusion());
            });
        }
        if (!Closed(input, result, faces, f => origin[f] >= 0, tolerance))
        {
            return false;
        }

        // The contours, each with its spec and reference face
        var lines = new List<(Edge[] Edges, ChamferSpec Spec, Face Reference)>();
        for (int ic = 1; ic <= contours.ContourCount; ic++)
        {
            Edge[] own = [.. Enumerable.Range(1, contours.NbEdges(ic)).Select(j => contours.Edge(ic, j))];
            int at = Array.FindIndex(specs, s => own.Any(e => e.IsSame(s.Edge)));
            if (at < 0)
            {
                return false;
            }
            lines.Add((own, specs[at], referenceFaces[at]));
        }

        // Each chamfer face's contacts: first which contour it's of (most of its contact points nearest that contour's edges
        // inside them), then each such point at its distance and angle
        var checkedOn = new int[lines.Count];
        foreach (Face chamferFace in faces.Where(f => origin[f] < 0))
        {
            var contacts = new List<(Point3D Point, Face Face)>();
            foreach (Edge edge in chamferFace.AllEdges())
            {
                Face other = faces.FirstOrDefault(f => !f.IsSame(chamferFace) && f.AllEdges().Any(e => e.IsSame(edge)));
                if (other.IsNull || origin[other] < 0 || Tool.Curve(edge, out double a, out double b) is not { } curve)
                {
                    continue;
                }
                for (int k = 1; k < 20; k++)
                {
                    contacts.Add((curve.EvalD0(a + ((b - a) * k / 20)), inputFaces[origin[other]]));
                }
            }
            var votes = new int[lines.Count];
            foreach ((Point3D x, Face face) in contacts)
            {
                if (OnEdge(x, face, lines, -1) is { } foot)
                {
                    votes[foot.Line]++;
                }
            }
            if (votes.Max() == 0)
            {
                return false;
            }
            int line = Array.IndexOf(votes, votes.Max());
            ChamferSpec spec = lines[line].Spec;
            foreach ((Point3D x, Face face) in contacts)
            {
                if (OnEdge(x, face, lines, line) is not { } foot)
                {
                    continue;
                }
                bool onReference = lines[line].Reference.IsSame(face);
                double allowed = tolerance + foot.Tolerance;
                double? distance = spec.Angle is not null ? (onReference ? spec.Distance : null) : onReference ? spec.Distance : spec.Distance2 ?? spec.Distance;
                if (distance is { } d && Math.Abs(x.DistanceTo(foot.Point) - d) > allowed)
                {
                    return false;
                }
                if (spec.Angle is { } angle && onReference)
                {
                    Vector3D along = foot.Tangent;
                    Vector3D onFace = NormalAt(face, x).Crossed(along), onChamfer = NormalAt(chamferFace, x).Crossed(along);
                    double between = Math.Acos(Math.Min(1, Math.Abs(onFace.Dot(onChamfer)) / (onFace.Magnitude * onChamfer.Magnitude)));
                    if (Math.Abs(between - angle) > allowed / spec.Distance)
                    {
                        return false;
                    }
                }
                checkedOn[line]++;
            }
        }
        return checkedOn.All(static n => n >= FewestChecked);
    }

    private readonly record struct EdgeFoot(int Line, Point3D Point, Vector3D Tangent, double Tolerance);

    // The nearest point of a chamfered edge of the face, inside the edge, on the contour line (or any, at -1)
    private static EdgeFoot? OnEdge(Point3D x, Face face, List<(Edge[] Edges, ChamferSpec Spec, Face Reference)> lines, int line)
    {
        EdgeFoot? found = null;
        double best = double.MaxValue;
        for (int i = 0; i < lines.Count; i++)
        {
            if (line >= 0 && i != line)
            {
                continue;
            }
            foreach (Edge edge in lines[i].Edges)
            {
                if (!OnFace(edge, face) || Tool.Curve(edge, out double first, out double last) is not { } curve)
                {
                    continue;
                }
                var projection = new ProjectPointOnCurve(x, curve, first, last);
                if (projection.NbPoints() == 0 || projection.LowerDistance() >= best)
                {
                    continue;
                }
                best = projection.LowerDistance();
                double t = projection.LowerDistanceParameter();
                bool inside = t - first > 1e-3 * (last - first) && last - t > 1e-3 * (last - first);
                curve.D1(t, out _, out Vector3D tangent);
                found = inside ? new EdgeFoot(i, projection.NearestPoint(), tangent / tangent.Magnitude, Tool.Tolerance(edge)) : null;
            }
        }
        return found;
    }

    private static Vector3D NormalAt(Face face, Point3D x)
    {
        Surface surface = Tool.Surface(face)!;
        var projection = new ProjectPointOnSurface(x, surface);
        projection.LowerDistanceParameters(out double u, out double v);
        return Normal(surface, u, v);
    }

    // Every free edge of the result runs along a free edge of the input, or is a chamfer face's end between two of them
    private static bool Closed(Shape input, Shape result, Face[] faces, Func<Face, bool> isInputPiece, double tolerance)
    {
        Edge[] inputFree = FreeEdges(input, input.AllFaces().ToArray(), out _);
        Edge[] free = FreeEdges(result, faces, out Face[] owners);
        for (int i = 0; i < free.Length; i++)
        {
            if (Tool.Curve(free[i], out double a, out double b) is not { } curve)
            {
                return false;
            }
            double allowed = tolerance + Tool.Tolerance(free[i]);
            if (new[] { 0.25, 0.5, 0.75 }.All(t => DistanceTo(curve.EvalD0(a + (t * (b - a))), inputFree) <= allowed))
            {
                continue;
            }
            if (!isInputPiece(owners[i]) && DistanceTo(curve.EvalD0(a), inputFree) <= allowed && DistanceTo(curve.EvalD0(b), inputFree) <= allowed)
            {
                continue;
            }
            return false;
        }
        _ = result;
        return true;
    }

    private static Edge[] FreeEdges(Shape shape, Face[] faces, out Face[] owners)
    {
        var free = new List<Edge>();
        var own = new List<Face>();
        foreach (Edge edge in shape.AllEdges())
        {
            if (Tool.Degenerated(edge))
            {
                continue;
            }
            Face[] of = [.. faces.Where(f => f.AllEdges().Any(e => e.IsSame(edge)))];
            if (of.Length == 1 && of[0].Explore<Edge>(ShapeType.Edge).Count(e => e.IsSame(edge)) == 1)
            {
                free.Add(edge);
                own.Add(of[0]);
            }
        }
        owners = [.. own];
        return [.. free];
    }

    private static double DistanceTo(Point3D p, Edge[] edges)
    {
        double best = double.MaxValue;
        foreach (Edge edge in edges)
        {
            if (Tool.Curve(edge, out double a, out double b) is not { } curve)
            {
                continue;
            }
            var projection = new ProjectPointOnCurve(p, curve, a, b);
            if (projection.NbPoints() > 0)
            {
                best = Math.Min(best, projection.LowerDistance());
            }
            best = Math.Min(best, Math.Min(p.DistanceTo(curve.EvalD0(a)), p.DistanceTo(curve.EvalD0(b))));
        }
        return best;
    }
}
