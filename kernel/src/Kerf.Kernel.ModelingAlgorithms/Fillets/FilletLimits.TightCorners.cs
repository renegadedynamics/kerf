// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using EngineCut = Kerf.Engine.Booleans.Api.Cut;

namespace Kerf.Fillets;

/// <summary>What a chamfer does at a convex arc of its face's outline whose radius is no more than the chamfer's length on that face,
/// where the outline can't be stepped in by the length (docs/chamfer-corners.md).</summary>
public enum TightCorners
{
    /// <summary>Refuse, saying what fits: as OCCT, which has nothing else.</summary>
    Refuse,

    /// <summary>Let the corner collapse to a point: the straight insets beside it meet, and the chamfer over the arc is the cone from that
    /// point to the arc's contact on the other face. A Kerf addition.</summary>
    Collapse,
}

/// <summary>
/// Chamfers past tight corners (decision 1988; docs/chamfer-corners.md). A whole loop of lines and arcs on a planar face, its other faces
/// square to it, is chamfered by one tool made of the chamfer's exact faces, and the answer checked against the volume and area its outline
/// gives. Only reached with <see cref="TightCorners.Collapse"/>, and only where a corner is tight: ChFi3d does the rest, as before.
/// </summary>
internal static partial class FilletLimits
{
    // Below this angle a joint of the outline is tangent
    private const double TangentJoint = 1e-7;

    // Gauss-Legendre, 10 points on [-1, 1]
    private static readonly double[] GaussNodes = [0.1488743389816312, 0.4333953941292472, 0.6794095682990244, 0.8650633666889845, 0.9739065285171717];
    private static readonly double[] GaussWeights = [0.2955242247147529, 0.2692667193099963, 0.2190863625159820, 0.1494513491505806, 0.0666713443086881];

    /// <summary>A point of the face's plane, in its frame.</summary>
    private readonly record struct Flat(double X, double Y)
    {
        public static Flat operator +(Flat a, Flat b) => new(a.X + b.X, a.Y + b.Y);

        public static Flat operator -(Flat a, Flat b) => new(a.X - b.X, a.Y - b.Y);

        public static Flat operator *(Flat a, double k) => new(a.X * k, a.Y * k);

        public double Dot(Flat b) => X * b.X + Y * b.Y;

        public double Cross(Flat b) => X * b.Y - Y * b.X;

        public double Length => Math.Sqrt(X * X + Y * Y);

        public Flat Left => new(-Y, X);

        public static Flat At(double angle) => new(Math.Cos(angle), Math.Sin(angle));

        public static Flat Mix(Flat a, Flat b, double t) => a + (b - a) * t;
    }

    /// <summary>The face's plane: its origin, axes, and outward normal; depth is measured down from it, into the material.</summary>
    private readonly record struct FacePlane(Point3D Origin, Direction3D X, Direction3D Y, Direction3D Up)
    {
        public Flat Flatten(in Point3D p)
        {
            var v = new Vector3D(Origin, p);
            return new Flat(v.Dot(new Vector3D(X)), v.Dot(new Vector3D(Y)));
        }

        public double Depth(in Point3D p) => -new Vector3D(Origin, p).Dot(new Vector3D(Up));

        public Point3D Lift(Flat p, double depth) =>
            Origin.Translated(new Vector3D(X) * p.X + new Vector3D(Y) * p.Y + new Vector3D(Up) * -depth);
    }

    /// <summary>One edge of the loop, as F holds it: a line from Start to End, or an arc about Centre of Radius turning Sweep (positive
    /// anticlockwise about F's normal) from the angle From. Its inward normal points into F.</summary>
    private sealed record Piece(Edge Edge, Face Side, Flat Start, Flat End, bool IsArc, Flat Centre, double Radius, double From, double Sweep)
    {
        public int Turn => Math.Sign(Sweep);

        /// <summary>An arc whose centre is on the material's side.</summary>
        public bool IsConvex => IsArc && Sweep > 0;

        public Flat StartTangent => IsArc ? Flat.At(From).Left * Turn : Direction;

        public Flat EndTangent => IsArc ? Flat.At(From + Sweep).Left * Turn : Direction;

        public Flat Direction => (End - Start) * (1 / (End - Start).Length);

        public Flat InwardAtStart => IsArc ? Flat.At(From) * -Turn : Direction.Left;

        public Flat InwardAtEnd => IsArc ? Flat.At(From + Sweep) * -Turn : Direction.Left;

        public double Length => IsArc ? Radius * Math.Abs(Sweep) : (End - Start).Length;

        // Set once the loop is read: where its contact on F runs, and P for a tight arc
        public Flat TopStart { get; set; }

        public Flat TopEnd { get; set; }

        public bool Tight { get; set; }

        public Flat Apex { get; set; }
    }

    /// <summary>One loop to chamfer, a on F and b down the sides.</summary>
    private sealed record TightLoop(Piece[] Pieces, double A, double B);

    /// <summary>The chamfer with tight corners collapsed, or a refusal saying what isn't built; null where no corner is tight, for
    /// ChFi3d to build as it always has.</summary>
    public static EngineOutcome? CollapseTightCorners(Shape shape, ChamferSpec[] specs, Face[] referenceFaces, ChamferOptions options) =>
        CollapseTightCorners(shape, specs, referenceFaces, options, assemble: true);

    /// <summary>As above; <paramref name="assemble"/> false always builds by the Boolean, for the tests to compare the two.</summary>
    internal static EngineOutcome? CollapseTightCorners(Shape shape, ChamferSpec[] specs, Face[] referenceFaces, ChamferOptions options, bool assemble)
    {
        if (options.TightCorners != TightCorners.Collapse || options.Mode != ChamferMode.Classic || shape.AllSolids().Length != 1)
        {
            return null;
        }
        var neighbours = new Neighbours(shape);
        (Edge Edge, Face Face, double Radius, double Length)? tight = null;
        try
        {
            tight = FirstTight(neighbours, specs, referenceFaces);
        }
        catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
        {
            return null;
        }
        if (tight is not { } first)
        {
            return null;
        }
        Edge[] edges = [.. specs.Select(static s => s.Edge)];
        string asked = $"The chamfer with a length of {first.Length:G6} on its face is longer than the radius {first.Radius:G6} of a convex arc of {Edges(edges)}";
        try
        {
            if (ReadLoops(neighbours, first.Face, specs, referenceFaces, out string? why) is not { } loops)
            {
                return Refused(FilletOperations.TooLarge, $"{asked}, and collapsing that corner isn't built here: {why}.", edges);
            }
            if (BuildCollapsed(shape, neighbours, first.Face, loops, assemble) is { } built)
            {
                return built;
            }
        }
        catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
        {
        }
        return Refused(FilletOperations.Failed, $"{asked}, and the chamfer with that corner collapsed couldn't be built.", edges);
    }

    private static EngineOutcome Refused(DiagnosticCode code, string message, Edge[] edges)
    {
        Diagnostic refused = Diagnostic.Error(code, message).WithShapes(edges.Select(static e => (Shape)e));
        return new EngineOutcome(false, static () => default, Failure: refused, MoreFailures: [refused]);
    }

    // The distances of a spec on the face given and on the other: two lengths, or a length and the angle from the reference face,
    // its other length the reference's times the tangent with the faces square
    private static (double OnFace, double OnOther) Lengths(in ChamferSpec spec, Face face, Face reference)
    {
        bool isReference = face.IsSame(reference);
        double onReference = spec.Distance;
        double onOther = spec.Angle is { } angle ? spec.Distance * Math.Tan(angle) : spec.Distance2 ?? spec.Distance;
        return isReference ? (onReference, onOther) : (onOther, onReference);
    }

    // A convex arc of a planar face, chamfered on it by its radius or more
    private static (Edge Edge, Face Face, double Radius, double Length)? FirstTight(Neighbours neighbours, ChamferSpec[] specs, Face[] referenceFaces)
    {
        for (int i = 0; i < specs.Length; i++)
        {
            Cancellation.ThrowIfRequested();
            Edge edge = specs[i].Edge;
            var curve = new EdgeCurveAdaptor(edge);
            if (curve.GetType() != CurveKind.Circle)
            {
                continue;
            }
            Circle3D circle = curve.Circle();
            foreach (Face face in neighbours.Faces(edge))
            {
                if (neighbours.Normal(face) is not { } up)
                {
                    continue;
                }
                double length = Lengths(specs[i], face, referenceFaces[i]).OnFace;
                if (length < circle.Radius - TightTolerance(edge, face))
                {
                    continue;
                }

                // Convex on this face: its centre is on the material's side
                Edge oriented = neighbours.Oriented(face, edge);
                double middle = (curve.FirstParameter() + curve.LastParameter()) / 2;
                curve.D1(middle, out Point3D at, out Vector3D tangent);
                if (oriented.Orientation == Orientation.Reversed)
                {
                    tangent *= -1;
                }
                Vector3D inward = new Vector3D(up).Crossed(tangent);
                if (inward.Dot(new Vector3D(at, circle.Location)) > 0)
                {
                    return (edge, face, circle.Radius, length);
                }
            }
        }
        return null;
    }

    private static double TightTolerance(in Edge edge, in Face face) => Math.Max(Tolerance, Math.Max(Tool.Tolerance(edge), Tool.Tolerance(face)));

    // The loops, each a whole wire of the face, every edge of them selected and nothing else; null, saying why, for what isn't built
    private static List<TightLoop>? ReadLoops(Neighbours neighbours, Face face, ChamferSpec[] specs, Face[] referenceFaces, out string? why)
    {
        why = null;
        var selected = new Dictionary<Shape, int>(ShapeHasher.Instance);
        for (int i = 0; i < specs.Length; i++)
        {
            selected.TryAdd(specs[i].Edge, i);
        }
        if (selected.Keys.Any(e => !neighbours.Faces((Edge)e).Any(f => f.IsSame(face))))
        {
            why = "the edges aren't all on one planar face";
            return null;
        }
        Direction3D up = neighbours.Normal(face)!.Value;
        Plane3D plane = new FaceSurfaceAdaptor(face).Plane();
        Direction3D x = plane.Position.XDirection;
        var frame = new FacePlane(plane.Location, x, up.Crossed(x), up);

        var loops = new List<TightLoop>();
        int covered = 0;
        foreach (Wire wire in face.AllWires())
        {
            Edge[] own = [.. wire.AllEdges().Where(static e => !Tool.Degenerated(e))];
            int chosen = own.Count(e => selected.ContainsKey(e));
            if (chosen == 0)
            {
                continue;
            }
            if (chosen != own.Length)
            {
                why = "the edges are part of a loop of the face, and only whole loops are built";
                return null;
            }
            covered += chosen;

            // One pair of lengths all round
            (double a, double b) = Lengths(specs[selected[own[0]]], face, referenceFaces[selected[own[0]]]);
            foreach (Edge edge in own)
            {
                (double ai, double bi) = Lengths(specs[selected[edge]], face, referenceFaces[selected[edge]]);
                if (Math.Abs(ai - a) > Tolerance || Math.Abs(bi - b) > Tolerance)
                {
                    why = "the lengths differ round the loop";
                    return null;
                }
            }
            if (ReadLoop(neighbours, face, frame, own, a, b, out why) is not { } loop)
            {
                return null;
            }
            loops.Add(loop);
        }
        if (covered != selected.Count)
        {
            why = "the edges aren't all on one planar face";
            return null;
        }
        return loops;
    }

    private static TightLoop? ReadLoop(Neighbours neighbours, Face face, FacePlane frame, Edge[] edges, double a, double b, out string? why)
    {
        why = null;
        var pieces = new List<Piece>();
        foreach (Edge edge in edges)
        {
            if (ReadPiece(neighbours, face, frame, edge, out why) is not { } piece)
            {
                return null;
            }
            pieces.Add(piece);
        }

        // In order round the loop, each starting where the last ended
        double join = edges.Max(static e => Tool.Tolerance(e)) * 2 + Tolerance;
        var ordered = new List<Piece> { pieces[0] };
        pieces.RemoveAt(0);
        while (pieces.Count > 0)
        {
            Flat end = ordered[^1].End;
            int next = pieces.FindIndex(p => (p.Start - end).Length <= join);
            if (next < 0)
            {
                why = "the loop's edges don't run end to end";
                return null;
            }
            ordered.Add(pieces[next]);
            pieces.RemoveAt(next);
        }
        Piece[] loop = [.. ordered];
        if ((loop[0].Start - loop[^1].End).Length > join || loop.Length < 2)
        {
            why = "the loop isn't closed by two edges or more";
            return null;
        }

        // How deep the sides go: the chamfer mustn't use one up
        foreach (Piece piece in loop)
        {
            double deepest = piece.Side.AllVertices().Max(v => frame.Depth(Tool.Pnt(v)));
            if (piece.Side.AllVertices().Any(v => frame.Depth(Tool.Pnt(v)) < -join))
            {
                why = "a face beside the loop rises above it";
                return null;
            }
            if (deepest <= b + join)
            {
                why = $"the chamfer's {b:G6} down the side uses up a face beside the loop, {deepest:G6} deep";
                return null;
            }
        }

        // Tight arcs, between straight neighbours tangent to them; sharp corners only between lines
        int n = loop.Length;
        for (int i = 0; i < n; i++)
        {
            Piece piece = loop[i], before = loop[(i + n - 1) % n], after = loop[(i + 1) % n];
            piece.Tight = piece.IsConvex && piece.Radius <= a + TightTolerance(piece.Edge, face);
            bool sharpAfter = Math.Abs(piece.EndTangent.Cross(after.StartTangent)) > TangentJoint || piece.EndTangent.Dot(after.StartTangent) < 0;
            if (sharpAfter && (piece.IsArc || after.IsArc))
            {
                why = "a corner beside an arc is sharp";
                return null;
            }
        }
        if (loop.All(static p => p.Tight))
        {
            why = "every edge of the loop is a tight arc, with nothing beside them for the corners to collapse onto";
            return null;
        }

        // The contact on F: each line's inset, from corner to corner; an arc's inset, or its corner's point. Each run of tight arcs
        // collapses to the point P where the insets of the two pieces beside it meet; an arc beside it is stepped in up to its foot under
        // P, and the stretch from there to the run is swallowed too, its rulings running to P as the run's do (docs/chamfer-corners.md)
        var cutStart = new double[n];
        var cutEnd = new double[n];
        for (int i = 0; i < n; i++)
        {
            if (!loop[i].Tight || loop[(i + n - 1) % n].Tight)
            {
                continue;
            }
            int last = i;
            while (loop[(last + 1) % n].Tight)
            {
                last = (last + 1) % n;
            }
            int beforeAt = (i + n - 1) % n, afterAt = (last + 1) % n;
            Piece before = loop[beforeAt], after = loop[afterAt];
            if ((before.IsArc && !before.IsConvex) || (after.IsArc && !after.IsConvex))
            {
                why = "a tight arc is beside a concave arc, where the stretch it swallows turns the other way, so its rulings to the corner's point "
                      + "don't join points of the same direction";
                return null;
            }
            if (Collapsed(before, after, a, join) is not { } corner)
            {
                why = before.IsArc || after.IsArc
                    ? "the face is too narrow: the insets beside a tight arc don't meet within the edges beside it"
                    : "the face is too narrow: the lines beside a tight arc are parallel, and their insets never meet";
                return null;
            }
            for (int k = i; ; k = (k + 1) % n)
            {
                loop[k].Apex = corner.Apex;
                loop[k].TopStart = loop[k].TopEnd = corner.Apex;
                if (k == last)
                {
                    break;
                }
            }
            cutEnd[beforeAt] = corner.BeforeSwallowed;
            cutStart[afterAt] = corner.AfterSwallowed;
        }
        for (int i = 0; i < n; i++)
        {
            Piece piece = loop[i];
            if (piece.IsArc && !piece.Tight)
            {
                piece.TopStart = piece.Start + piece.InwardAtStart * a;
                piece.TopEnd = piece.End + piece.InwardAtEnd * a;
            }
        }

        // The arcs beside a run, split where their stretch swallowed starts: the stretch is a tight piece of its own, its cone from P
        if (cutStart.Any(static c => c > 0) || cutEnd.Any(static c => c > 0))
        {
            var split = new List<Piece>();
            for (int i = 0; i < n; i++)
            {
                Piece piece = loop[i];
                if (!piece.IsArc || (cutStart[i] <= 0 && cutEnd[i] <= 0))
                {
                    split.Add(piece);
                    continue;
                }
                double sweep = Math.Abs(piece.Sweep), kept = sweep - cutStart[i] - cutEnd[i];
                if (kept * piece.Radius <= join)
                {
                    why = "the face is too narrow: the inset of an arc beside a tight arc is used up";
                    return null;
                }
                Flat apexBefore = loop[(i + n - 1) % n].Apex, apexAfter = loop[(i + 1) % n].Apex;
                if (cutStart[i] > 0)
                {
                    split.Add(SubArc(piece, 0, cutStart[i], apexBefore));
                }
                Piece middle = SubArc(piece, cutStart[i], sweep - cutEnd[i], null);
                middle.TopStart = cutStart[i] > 0 ? apexBefore : piece.TopStart;
                middle.TopEnd = cutEnd[i] > 0 ? apexAfter : piece.TopEnd;
                split.Add(middle);
                if (cutEnd[i] > 0)
                {
                    split.Add(SubArc(piece, sweep - cutEnd[i], sweep, apexAfter));
                }
            }
            loop = [.. split];
            n = loop.Length;
        }
        for (int i = 0; i < n; i++)
        {
            Piece piece = loop[i];
            if (piece.IsArc)
            {
                continue;
            }
            Piece before = loop[(i + n - 1) % n], after = loop[(i + 1) % n];
            Flat? start = before.IsArc ? before.TopEnd : Math.Abs(before.Direction.Cross(piece.Direction)) > TangentJoint ? Meet(before, a, piece) : Inset(piece, a);
            Flat? end = after.IsArc ? after.TopStart : Math.Abs(piece.Direction.Cross(after.Direction)) > TangentJoint ? Meet(piece, a, after) : piece.End + piece.Direction.Left * a;
            if (start is not { } s || end is not { } e)
            {
                why = "a corner's insets are parallel";
                return null;
            }
            piece.TopStart = s;
            piece.TopEnd = e;
            if ((e - s).Dot(piece.Direction) <= join)
            {
                why = "the face is too narrow: the inset of a straight edge is used up";
                return null;
            }
        }

        // Turned inside out, or crossing itself: the face is too narrow
        double outline = SectionArea(loop, a, 0), top = SectionArea(loop, a, 1);
        if (Math.Sign(top) != Math.Sign(outline) || outline - top <= 0) // Material on the left: an outline shrinks, a hole grows
        {
            why = "the face is too narrow for the inset";
            return null;
        }
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 2; j < n; j++)
            {
                if ((i == 0 && j == n - 1) || loop[i].IsArc || loop[j].IsArc)
                {
                    continue;
                }
                if (Crosses(loop[i].TopStart, loop[i].TopEnd, loop[j].TopStart, loop[j].TopEnd))
                {
                    why = "the face is too narrow: the insets cross";
                    return null;
                }
            }
        }
        return new TightLoop(loop, a, b);
    }

    // A line's inset by a, at its start
    private static Flat Inset(Piece line, double a) => line.Start + line.Direction.Left * a;

    // Where two lines' insets meet; null if they're parallel
    private static Flat? Meet(Piece one, double a, Piece two)
    {
        Flat p = Inset(one, a), q = Inset(two, a), u = one.Direction, v = two.Direction;
        double cross = u.Cross(v);
        if (Math.Abs(cross) < 1e-12)
        {
            return null;
        }
        return p + u * ((q - p).Cross(v) / cross);
    }

    /// <summary>Where a run of tight arcs collapses: the point P where the insets of the pieces before and after it meet, each a line's
    /// inset or a convex arc's, stepped in to radius r - a; and for an arc, the angle of it next to the run whose inset lies past P, swallowed
    /// with the run. Of the two points where a line or arc meets an arc, the one whose feet lie on both pieces, swallowing least. Null
    /// where none does.</summary>
    private static (Flat Apex, double BeforeSwallowed, double AfterSwallowed)? Collapsed(Piece before, Piece after, double a, double join)
    {
        if (!before.IsArc && !after.IsArc)
        {
            return Meet(before, a, after) is { } p ? (p, 0, 0) : null;
        }
        (Flat Apex, double Before, double After)? best = null;
        double least = double.MaxValue;
        foreach (Flat p in Crossings(before, after, a))
        {
            // How much of each piece lies between its foot under P and the run, as a length: none may be past the run's end of it
            double onBefore, onAfter, angleBefore = 0, angleAfter = 0;
            if (before.IsArc)
            {
                angleBefore = Turned(Math.Atan2(p.Y - before.Centre.Y, p.X - before.Centre.X), before.From + before.Sweep);
                onBefore = angleBefore * before.Radius;
            }
            else
            {
                onBefore = (before.End - p).Dot(before.Direction);
            }
            if (after.IsArc)
            {
                angleAfter = Turned(after.From, Math.Atan2(p.Y - after.Centre.Y, p.X - after.Centre.X));
                onAfter = angleAfter * after.Radius;
            }
            else
            {
                onAfter = (p - after.Start).Dot(after.Direction);
            }
            if (onBefore < -join || onAfter < -join || (before.IsArc && angleBefore >= before.Sweep) || (after.IsArc && angleAfter >= after.Sweep))
            {
                continue;
            }
            if (onBefore + onAfter < least)
            {
                least = onBefore + onAfter;
                // A stretch no longer than the shape can tell is none: at r = a, P is the arc's centre, on the neighbour's inset at the joint
                best = (p, onBefore > join ? angleBefore : 0, onAfter > join ? angleAfter : 0);
            }
        }
        return best;
    }

    // The angle turned anticlockwise from one direction to another, in [-π, π): small negatives are a foot just short of the joint
    private static double Turned(double from, double to)
    {
        double turned = Math.IEEERemainder(to - from, 2 * Math.PI);
        return turned >= Math.PI ? turned - 2 * Math.PI : turned;
    }

    // The points where the insets of a line or convex arc and a convex arc cross: a line's inset is a line, an arc's the circle of r - a
    private static List<Flat> Crossings(Piece one, Piece two, double a)
    {
        var found = new List<Flat>();
        if (!one.IsArc || !two.IsArc)
        {
            Piece line = one.IsArc ? two : one, arc = one.IsArc ? one : two;
            Flat p = Inset(line, a), u = line.Direction, d = p - arc.Centre;
            double rho = arc.Radius - a, half = u.Dot(d), c = d.Dot(d) - rho * rho, discriminant = half * half - c;
            if (discriminant < 0)
            {
                return found;
            }
            double root = Math.Sqrt(discriminant);
            found.Add(p + u * (-half - root));
            found.Add(p + u * (-half + root));
            return found;
        }
        Flat between = two.Centre - one.Centre;
        double distance = between.Length, r1 = one.Radius - a, r2 = two.Radius - a;
        if (distance < 1e-12 || distance > r1 + r2 || distance < Math.Abs(r1 - r2))
        {
            return found;
        }
        double along = (distance * distance + r1 * r1 - r2 * r2) / (2 * distance), across = Math.Sqrt(Math.Max(0, r1 * r1 - along * along));
        Flat axis = between * (1 / distance), mid = one.Centre + axis * along;
        found.Add(mid + axis.Left * across);
        found.Add(mid - axis.Left * across);
        return found;
    }

    // The stretch of an arc from one angle turned along it to another; with a point, a tight piece whose cone runs from there
    private static Piece SubArc(Piece arc, double from, double to, Flat? apex)
    {
        double start = arc.From + arc.Turn * from, sweep = arc.Turn * (to - from);
        var piece = arc with
        {
            From = start,
            Sweep = sweep,
            Start = arc.Centre + Flat.At(start) * arc.Radius,
            End = arc.Centre + Flat.At(start + sweep) * arc.Radius,
        };
        if (apex is { } p)
        {
            piece.Tight = true;
            piece.Apex = p;
            piece.TopStart = piece.TopEnd = p;
        }
        else
        {
            piece.Tight = false;
        }
        return piece;
    }

    private static bool Crosses(Flat p, Flat q, Flat r, Flat s)
    {
        double d1 = (q - p).Cross(r - p), d2 = (q - p).Cross(s - p), d3 = (s - r).Cross(p - r), d4 = (s - r).Cross(q - r);
        return d1 * d2 < 0 && d3 * d4 < 0;
    }

    private static Piece? ReadPiece(Neighbours neighbours, Face face, in FacePlane frame, Edge edge, out string? why)
    {
        why = null;
        Face[] others = [.. neighbours.Faces(edge).Where(f => !f.IsSame(face))];
        if (others.Length != 1)
        {
            why = "an edge of the loop isn't between two faces";
            return null;
        }
        Face side = others[0];
        Edge oriented = neighbours.Oriented(face, edge);
        var curve = new EdgeCurveAdaptor(edge);
        double first = curve.FirstParameter(), last = curve.LastParameter();
        bool reversed = oriented.Orientation == Orientation.Reversed;
        Point3D start = curve.Value(reversed ? last : first), end = curve.Value(reversed ? first : last);
        var sideSurface = new FaceSurfaceAdaptor(side);
        double square = 1e-9;
        switch (curve.GetType())
        {
            case CurveKind.Line:
            {
                if (sideSurface.GetType() != SurfaceKind.Plane || Math.Abs(sideSurface.Plane().Axis.Direction.Dot(frame.Up)) > square)
                {
                    why = "a face beside a straight edge isn't a plane square to the face";
                    return null;
                }
                return new Piece(edge, side, frame.Flatten(start), frame.Flatten(end), false, default, 0, 0, 0);
            }
            case CurveKind.Circle:
            {
                Circle3D circle = curve.Circle();
                if (sideSurface.GetType() != SurfaceKind.Cylinder)
                {
                    why = "a face beside an arc isn't a cylinder square to the face";
                    return null;
                }
                Cylinder3D cylinder = sideSurface.Cylinder();
                if (Math.Abs(Math.Abs(cylinder.Axis.Direction.Dot(frame.Up)) - 1) > square || Math.Abs(cylinder.Radius - circle.Radius) > TightTolerance(edge, face)
                    || Math.Abs(Math.Abs(circle.Axis.Direction.Dot(frame.Up)) - 1) > square)
                {
                    why = "a face beside an arc isn't a cylinder square to the face";
                    return null;
                }
                curve.D1(reversed ? last : first, out _, out Vector3D tangent);
                if (reversed)
                {
                    tangent *= -1;
                }
                Flat centre = frame.Flatten(circle.Location), from = frame.Flatten(start) - centre;
                Flat along = new(tangent.Dot(new Vector3D(frame.X)), tangent.Dot(new Vector3D(frame.Y)));
                int turn = from.Cross(along) > 0 ? 1 : -1;
                double sweep = turn * (last - first);
                return new Piece(edge, side, frame.Flatten(start), frame.Flatten(end), true, centre, circle.Radius, Math.Atan2(from.Y, from.X), sweep);
            }
            default:
                why = "an edge of the loop isn't a line or a circular arc";
                return null;
        }
    }

    /// <summary>The signed area inside the section at λ (0 the outline at depth b, 1 the contact on F), anticlockwise positive, by
    /// Green's theorem: its lines and arcs in closed form. Quadratic in λ, so Simpson's rule on three of them is exact.</summary>
    private static double SectionArea(Piece[] loop, double a, double lambda)
    {
        double twice = 0;
        foreach (Piece piece in loop)
        {
            if (!piece.IsArc)
            {
                Flat p = Flat.Mix(piece.Start, piece.TopStart, lambda), q = Flat.Mix(piece.End, piece.TopEnd, lambda);
                twice += p.Cross(q);
                continue;
            }
            (Flat centre, double radius) = SectionArc(piece, a, lambda);
            double t0 = piece.From, t1 = piece.From + piece.Sweep;
            twice += radius * centre.X * (Math.Sin(t1) - Math.Sin(t0)) - radius * centre.Y * (Math.Cos(t1) - Math.Cos(t0)) + radius * radius * piece.Sweep;
        }
        return twice / 2;
    }

    // An arc's section at λ: about its centre, stepped in, or for a tight one about the point between its centre and P
    private static (Flat Centre, double Radius) SectionArc(Piece piece, double a, double lambda) =>
        piece.Tight ? (Flat.Mix(piece.Centre, piece.Apex, lambda), (1 - lambda) * piece.Radius) : (piece.Centre, piece.Radius - piece.Turn * lambda * a);

    /// <summary>What a loop's chamfer removes: Simpson's rule on the sections' areas, exact for a quadratic.</summary>
    private static double RemovedVolume(TightLoop loop)
    {
        double outline = SectionArea(loop.Pieces, loop.A, 0), middle = SectionArea(loop.Pieces, loop.A, 0.5), top = SectionArea(loop.Pieces, loop.A, 1);
        return loop.B / 6 * (4 * (outline - middle) + (outline - top));
    }

    /// <summary>What a loop's chamfer does to the area: F loses the band to the contact, each side a band b deep, and the chamfer's faces
    /// are added.</summary>
    private static double AreaChange(TightLoop loop)
    {
        double a = loop.A, b = loop.B;
        double change = -(SectionArea(loop.Pieces, a, 0) - SectionArea(loop.Pieces, a, 1)) - b * loop.Pieces.Sum(static p => p.Length);
        foreach (Piece piece in loop.Pieces)
        {
            if (!piece.IsArc)
            {
                // A trapezium: half the cross of its diagonals
                Vector3D d1 = new(piece.TopEnd.X - piece.Start.X, piece.TopEnd.Y - piece.Start.Y, b);
                Vector3D d2 = new(piece.TopStart.X - piece.End.X, piece.TopStart.Y - piece.End.Y, b);
                change += d1.Crossed(d2).Magnitude / 2;
            }
            else if (!piece.Tight)
            {
                double top = piece.Radius - piece.Turn * a;
                change += Math.Abs(piece.Sweep) * Math.Sqrt(a * a + b * b) * (piece.Radius + top) / 2;
            }
            else
            {
                // The oblique cone: half r times the integral of the root of b² + ((c - P).e + r)² over the arc's angle
                Flat offset = piece.Centre - piece.Apex;
                double r = piece.Radius;
                change += r / 2 * Integrate(t => Math.Sqrt(b * b + Math.Pow(offset.Dot(Flat.At(t)) + r, 2)), piece.From, piece.From + piece.Sweep);
            }
        }
        return change;
    }

    // Composite Gauss-Legendre, 8 panels of 10 points: a smooth integrand, to far past one part in a million
    private static double Integrate(Func<double, double> f, double from, double to)
    {
        const int panels = 8;
        double width = (to - from) / panels, sum = 0;
        for (int panel = 0; panel < panels; panel++)
        {
            double middle = from + (panel + 0.5) * width, half = width / 2;
            for (int i = 0; i < GaussNodes.Length; i++)
            {
                sum += GaussWeights[i] * half * (f(middle + GaussNodes[i] * half) + f(middle - GaussNodes[i] * half));
            }
        }
        return Math.Abs(sum);
    }

    // Built from its faces where that's known to be what the Boolean would make, and otherwise by the Boolean; checked either way
    private static EngineOutcome? BuildCollapsed(Shape shape, Neighbours neighbours, Face face, List<TightLoop> loops, bool assemble)
    {
        Plane3D plane = new FaceSurfaceAdaptor(face).Plane();
        Direction3D up = neighbours.Normal(face)!.Value;
        Direction3D x = plane.Position.XDirection;
        var frame = new FacePlane(plane.Location, x, up.Crossed(x), up);
        double size = shape.AllVertices().Select(v => Tool.Pnt(v)).Max(p => Math.Max(Math.Abs(p.X), Math.Max(Math.Abs(p.Y), Math.Abs(p.Z))));
        double sewing = Math.Max(1e-6, 1e-9 * size);

        var made = new List<(Shape Source, Func<Face, bool> IsNew)>();
        var samples = new Dictionary<Shape, (bool Planar, Point3D[] Points)>(ShapeHasher.Instance);
        double volume = 0, area = 0;
        foreach (TightLoop loop in loops)
        {
            volume -= RemovedVolume(loop);
            area += AreaChange(loop);
            foreach (Piece piece in loop.Pieces)
            {
                FacePlane at = frame;
                TightLoop of = loop;
                made.Add((piece.Edge, f => IsChamferFace(Sampled(samples, f), at, of, piece, sewing * 10)));
            }
        }
        if (assemble && Assembled(shape, neighbours, face, frame, loops) is { } direct
            && Checked(shape, direct.Solid, direct.History, volume, area, [.. made], ownSize: true, direct.Faces) is { } assembled)
        {
            return assembled;
        }

        var tools = new List<Shape>();
        foreach (TightLoop loop in loops)
        {
            double deepest = loop.Pieces.Min(p => p.Side.AllVertices().Max(v => frame.Depth(Tool.Pnt(v))));
            double below = 0.5 * Math.Min(loop.B, deepest - loop.B);
            if (CollapseTool(frame, loop, below, sewing) is not { } tool)
            {
                return null;
            }
            tools.Add(tool);
        }
        if (Boolean(new EngineCut(), shape, tools, out History? cutting) is not { } result || result.AllSolids().Length != 1)
        {
            return null;
        }
        return Checked(shape, result.AllSolids()[0], cutting ?? new History(), volume, area, [.. made], ownSize: true);
    }

    // Three points of a face of the answer, and whether it's planar: taken once a face, as every piece asks of every face
    private static (bool Planar, Point3D[] Points) Sampled(Dictionary<Shape, (bool Planar, Point3D[] Points)> samples, Face face)
    {
        if (!samples.TryGetValue(face, out (bool Planar, Point3D[] Points) sampled))
        {
            var surface = new FaceSurfaceAdaptor(face);
            BrepTools.UVBounds(face, out double u0, out double u1, out double v0, out double v1);
            Point3D At(double s, double t) => surface.Value(u0 + s * (u1 - u0), v0 + t * (v1 - v0));
            sampled = (surface.GetType() == SurfaceKind.Plane, [At(0.5, 0.5), At(0.25, 0.75), At(0.75, 0.25)]);
            samples[face] = sampled;
        }
        return sampled;
    }

    /// <summary>Whether a face of the answer lies on a piece's chamfer: a plane on the strip's, or a surface whose points lie on the cone.</summary>
    private static bool IsChamferFace((bool Planar, Point3D[] Points) face, in FacePlane frame, TightLoop loop, Piece piece, double tolerance)
    {
        if (piece.IsArc == face.Planar)
        {
            return false;
        }
        foreach (Point3D p in face.Points)
        {
            if (OffChamfer(p, frame, loop, piece) > tolerance)
            {
                return false;
            }
        }
        return true;
    }

    // How far a point is from a piece's chamfer, carried on past its contacts
    private static double OffChamfer(in Point3D p, in FacePlane frame, TightLoop loop, Piece piece)
    {
        double depth = frame.Depth(p), lambda = 1 - depth / loop.B;
        Flat q = frame.Flatten(p);
        if (!piece.IsArc)
        {
            Flat from = Flat.Mix(piece.Start, piece.TopStart, lambda), to = Flat.Mix(piece.End, piece.TopEnd, lambda);
            Flat along = (to - from) * (1 / (to - from).Length);
            return Math.Abs(along.Cross(q - from));
        }
        (Flat centre, double radius) = SectionArc(piece, loop.A, lambda);
        return Math.Abs((q - centre).Length - radius);
    }

    /// <summary>What a loop's chamfer cuts away: each piece's chamfer face carried below the side's contact, a wall straight up from there
    /// to F's plane, and its top in that plane; sewn and made a solid.</summary>
    private static Shape? CollapseTool(in FacePlane frame, TightLoop loop, double below, double sewingTolerance)
    {
        double a = loop.A, b = loop.B, deep = b + below, k = deep / b;
        var sewing = new Sewing(sewingTolerance);
        foreach (Piece piece in loop.Pieces)
        {
            if (!piece.IsArc)
            {
                Flat bs = Flat.Mix(piece.TopStart, piece.Start, k), be = Flat.Mix(piece.TopEnd, piece.End, k);
                Point3D t0 = frame.Lift(piece.TopStart, 0), t1 = frame.Lift(piece.TopEnd, 0);
                Point3D b0 = frame.Lift(bs, deep), b1 = frame.Lift(be, deep), w0 = frame.Lift(bs, 0), w1 = frame.Lift(be, 0);
                sewing.Add(Polygon(t0, t1, b1, b0));
                sewing.Add(Polygon(b0, b1, w1, w0));
                sewing.Add(Polygon(t0, t1, w1, w0));
                continue;
            }

            // The section's arc at depth b + below, by λ = 1 - k; the wall over it a cylinder, and a right cone as OCCT's
            (Flat centre, double radius) = SectionArc(piece, a, 1 - k);
            if (piece.Tight)
            {
                (BSplineSurface chamfer, double last) = Ruled(frame, piece.Apex, 0, 0, centre, radius, deep, piece.From, piece.Sweep);
                sewing.Add(new MakeFace(chamfer, 0, last, 0, 1, Tolerance).Face());
            }
            else
            {
                sewing.Add(RightCone(frame, piece, a, b, deep));
            }
            sewing.Add(Wall(frame, centre, radius, deep, piece.From, piece.Sweep));

            // The top: from the contact on F (an arc, or P) out to the wall's top
            Flat ws = centre + Flat.At(piece.From) * radius, we = centre + Flat.At(piece.From + piece.Sweep) * radius;
            var wire = new MakeWire();
            if (!piece.Tight)
            {
                wire.Add(Arc(frame, piece.Centre, piece.Radius - piece.Turn * a, piece.From, piece.Sweep));
            }
            wire.Add(new MakeEdge(frame.Lift(piece.TopEnd, 0), frame.Lift(we, 0)).Edge());
            wire.Add(Arc(frame, centre, radius, piece.From, piece.Sweep));
            wire.Add(new MakeEdge(frame.Lift(ws, 0), frame.Lift(piece.TopStart, 0)).Edge());
            sewing.Add(new MakeFace(wire.Wire(), true).Face());
        }
        sewing.Perform();
        Shape sewn = sewing.SewedShape();
        var shells = sewn.AllShells();
        if (shells.Length != 1 || sewing.FreeEdgeCount != 0)
        {
            return null;
        }
        Solid solid = BrepLibrary.ClosedSolidOriented(new MakeSolid(shells[0]).Solid(), out bool oriented);
        return oriented ? solid : null;
    }

    // An arc's chamfer short of its corner's radius, carried down to depth deep: a right cone about the arc's axis, its angle F's
    private static Face RightCone(in FacePlane frame, Piece piece, double a, double b, double deep)
    {
        double angle = Math.Atan(-piece.Turn * a / b);
        var cone = new Cone3D(new CoordinateSystem(frame.Lift(piece.Centre, 0), frame.Up, frame.X), angle, piece.Radius - piece.Turn * a);
        return new MakeFace(cone, Math.Min(piece.From, piece.From + piece.Sweep), Math.Max(piece.From, piece.From + piece.Sweep), -deep / Math.Cos(angle), 0).Face();
    }

    // The wall over an arc of the chamfer's lower edge, straight up to F's plane: a cylinder
    private static Face Wall(in FacePlane frame, Flat centre, double radius, double deep, double from, double sweep)
    {
        var cylinder = new Cylinder3D(new CoordinateSystem(frame.Lift(centre, deep), frame.Up, frame.X), radius);
        return new MakeFace(cylinder, Math.Min(from, from + sweep), Math.Max(from, from + sweep), 0, deep).Face();
    }

    private static Face Polygon(params Point3D[] corners)
    {
        var wire = new MakeWire();
        for (int i = 0; i < corners.Length; i++)
        {
            wire.Add(new MakeEdge(corners[i], corners[(i + 1) % corners.Length]).Edge());
        }
        return new MakeFace(wire.Wire(), true).Face();
    }

    private static Edge Arc(in FacePlane frame, Flat centre, double radius, double from, double sweep)
    {
        var circle = new Circle3D(new Frame3D(frame.Lift(centre, 0), frame.Up, frame.X), radius);
        Point3D p = frame.Lift(centre + Flat.At(from) * radius, 0), q = frame.Lift(centre + Flat.At(from + sweep) * radius, 0);
        return sweep > 0 ? new MakeEdge(circle, p, q).Edge() : new MakeEdge(circle, q, p).Edge();
    }

    /// <summary>The ruled surface between two arcs of one angle, rows of the same rational poles and weights: exact, whether the arcs are
    /// concentric (a right cone or a cylinder) or one is a point (an oblique cone). U runs along the arcs, V from the first to the second.</summary>
    private static (BSplineSurface Surface, double Last) Ruled(in FacePlane frame, Flat centre1, double radius1, double depth1, Flat centre2, double radius2, double depth2, double from, double sweep)
    {
        int spans = Math.Max(1, (int)Math.Ceiling(Math.Abs(sweep) / (Math.PI / 2) - 1e-9));
        double step = sweep / spans, w = Math.Cos(step / 2);
        int count = 2 * spans + 1;
        var poles = new Point3D[count * 2];
        var weights = new double[count * 2];
        for (int i = 0; i < count; i++)
        {
            double angle = from + i * step / 2;
            double reach = i % 2 == 0 ? 1 : 1 / w;
            poles[i * 2] = frame.Lift(centre1 + Flat.At(angle) * (radius1 * reach), depth1);
            poles[i * 2 + 1] = frame.Lift(centre2 + Flat.At(angle) * (radius2 * reach), depth2);
            weights[i * 2] = weights[i * 2 + 1] = i % 2 == 0 ? 1 : w;
        }
        double[] uKnots = [.. Enumerable.Range(0, spans + 1).Select(static i => (double)i)];
        int[] uMults = [.. Enumerable.Range(0, spans + 1).Select(i => i == 0 || i == spans ? 3 : 2)];
        return (new BSplineSurface(new Grid<Point3D>(poles, count, 2), uKnots, [0.0, 1.0], uMults, [2, 2], 2, 1,
                                   weights: new Grid<double>(weights, count, 2)), spans);
    }
}
