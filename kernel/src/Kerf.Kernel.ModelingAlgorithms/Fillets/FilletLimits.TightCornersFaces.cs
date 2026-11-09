// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

/// <summary>
/// The chamfer past tight corners built from its faces, without a Boolean (unit 13B; docs/chamfer-corners.md, section 2). Where nothing
/// of the shape but the face and the faces beside the loops comes within the chamfer's depth, and seen from above no piece's chamfer
/// meets another's but its neighbours', what the Boolean would find is known: the face loses the band outside the contact, each side
/// loses its top down to the chamfer's depth, and the chamfer's faces fill the gap. So they're made and joined directly, with exact
/// edges, and the answer is checked as the Boolean's is. Anything else is left to the Boolean.
/// </summary>
internal static partial class FilletLimits
{
    /// <summary>Where a loop's edges meet: the vertex on F, the edge running down from it between the two sides, and what's made of
    /// them.</summary>
    private sealed class Joint(Vertex top, Edge down, Vertex foot)
    {
        public Vertex Top { get; } = top;

        public Edge Down { get; } = down;

        public Vertex Foot { get; } = foot;

        // Made: the vertex at the chamfer's depth, the edge down cut short there, the vertex of the contact on F, and the ruling between
        public Vertex Low { get; set; }

        public Edge Shortened { get; set; }

        public Vertex Contact { get; set; }

        public Edge Ruling { get; set; }

        public double RulingLength { get; set; }
    }

    /// <summary>The solid and its history, or null where it isn't known to be what the Boolean would make: the Boolean then builds
    /// it.</summary>
    private static (Shape Solid, History History, FacesChanged Faces)? Assembled(Shape shape, Neighbours neighbours, Face face, in FacePlane frame, List<TightLoop> loops)
    {
        // Every loop of the face is chamfered, so no other face starts on it
        if (face.AllWires().Length != loops.Count || shape.AllSolids() is not [Solid solid])
        {
            return null;
        }
        var sides = new HashSet<Shape>(ShapeHasher.Instance) { face };
        var joints = new List<Joint[]>();
        foreach (TightLoop loop in loops)
        {
            Piece[] pieces = loop.Pieces;
            int n = pieces.Length;
            double near = pieces.Max(static p => Tool.Tolerance(p.Edge)) * 2 + Tolerance;

            // One piece to a side, no two tight pieces together, and the sides square to the face: an arc split beside a run, two tight
            // arcs sharing a point, or a draft, whose pieces are the contact on the sides rather than their edges, is the Boolean's
            if (loop.Draft is not null || pieces.Any(static p => p.IsCurve) || Enumerable.Range(0, n).Any(i => pieces[i].Tight && pieces[(i + 1) % n].Tight))
            {
                return null;
            }
            foreach (Piece piece in pieces)
            {
                if (!sides.Add(piece.Side))
                {
                    return null;
                }
            }

            // At each corner of the loop, one straight edge runs square to the face down between the two sides, past the chamfer
            var atLoop = new Joint[n];
            for (int i = 0; i < n; i++)
            {
                Piece before = pieces[(i + n - 1) % n], piece = pieces[i];
                Vertex top = Exploration.FirstVertex(neighbours.Oriented(face, piece.Edge), true);
                Edge[] others = [.. neighbours.Edges(top).Where(e => !e.IsSame(piece.Edge) && !e.IsSame(before.Edge))];
                if (others.Length != 1 || neighbours.Edges(top).Count != 3 || Tool.Degenerated(others[0]))
                {
                    return null;
                }
                Edge down = others[0];
                var line = new EdgeCurveAdaptor(down);
                if (line.GetType() != CurveKind.Line || Math.Abs(Math.Abs(line.Line().Direction.Dot(frame.Up)) - 1) > 1e-9)
                {
                    return null;
                }
                IReadOnlyList<Face> beside = neighbours.Faces(down);
                if (beside.Count != 2 || !beside.Any(f => f.IsSame(before.Side)) || !beside.Any(f => f.IsSame(piece.Side)))
                {
                    return null;
                }
                Exploration.Vertices(down, out Vertex first, out Vertex last);
                Vertex foot = first.IsSame(top) ? last : first;
                if (foot.IsSame(top) || frame.Depth(Tool.Pnt(foot)) <= loop.B + near || Math.Abs(frame.Depth(Tool.Pnt(top))) > near)
                {
                    return null;
                }
                atLoop[i] = new Joint(top, down, foot);
            }
            joints.Add(atLoop);

            // The sides' other edges and the shape's other faces all lie deeper than the chamfer
            for (int i = 0; i < n; i++)
            {
                Edge start = atLoop[i].Down, end = atLoop[(i + 1) % n].Down, own = pieces[i].Edge;
                foreach (Edge edge in pieces[i].Side.AllEdges())
                {
                    if (!edge.IsSame(own) && !edge.IsSame(start) && !edge.IsSame(end) && Shallowest(edge, frame) <= loop.B + near)
                    {
                        return null;
                    }
                }
            }
        }
        double deepest = loops.Max(static l => l.B) + loops.Max(l => l.Pieces.Max(static p => Tool.Tolerance(p.Edge)) * 2 + Tolerance);
        foreach (Face other in shape.AllFaces())
        {
            Cancellation.ThrowIfRequested();
            if (!sides.Contains(other) && Shallowest(other, frame) <= deepest)
            {
                return null;
            }
        }
        if (!BandsApart(loops))
        {
            return null;
        }
        return Build(solid, neighbours, face, frame, loops, joints);
    }

    // The least depth of the shape's box: the box is larger than the shape, so it's no deeper than its shallowest point
    private static double Shallowest(in Shape shape, in FacePlane frame)
    {
        Box box = Bounds.Box(shape);
        box.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
        double least = double.MaxValue;
        foreach (double x in (ReadOnlySpan<double>)[x0, x1])
        {
            foreach (double y in (ReadOnlySpan<double>)[y0, y1])
            {
                foreach (double z in (ReadOnlySpan<double>)[z0, z1])
                {
                    least = Math.Min(least, frame.Depth(new Point3D(x, y, z)));
                }
            }
        }
        return least;
    }

    /// <summary>
    /// Seen from above, each piece's chamfer lies over the band between its edge and its stretch of the contact. Where no band meets
    /// another but its neighbours, along the ruling they share, and the two beside a collapsed corner only at its point, no face of
    /// the chamfer crosses another or the face. Each band is taken as the convex hull of its corners and its arcs' control points,
    /// which holds it: larger, so this can refuse what's sound, never pass what isn't.
    /// </summary>
    private static bool BandsApart(List<TightLoop> loops)
    {
        var bands = new List<(Flat[] Hull, int Loop, int Index, double Near)>();
        for (int l = 0; l < loops.Count; l++)
        {
            TightLoop loop = loops[l];
            double near = loop.Pieces.Max(static p => Tool.Tolerance(p.Edge)) * 2 + Tolerance;
            for (int i = 0; i < loop.Pieces.Length; i++)
            {
                bands.Add((Hull(BandCorners(loop.Pieces[i], loop.A)), l, i, near));
            }
        }
        for (int x = 0; x < bands.Count; x++)
        {
            for (int y = x + 1; y < bands.Count; y++)
            {
                (Flat[] one, int loopOne, int i, double near) = bands[x];
                (Flat[] two, int loopTwo, int j, _) = bands[y];
                double overlap = Overlap(one, two);
                bool touches = false;
                if (loopOne == loopTwo)
                {
                    Piece[] pieces = loops[loopOne].Pieces;
                    int n = pieces.Length;
                    bool besideApex = (pieces[(i + 1) % n].Tight && (i + 2) % n == j) || (pieces[(j + 1) % n].Tight && (j + 2) % n == i);
                    touches = (i + 1) % n == j || (j + 1) % n == i || besideApex;
                }
                if (touches ? overlap > near : overlap > -near)
                {
                    return false;
                }
            }
        }
        return true;
    }

    // A piece's band: its edge, its contact on F, and the arcs' control points, which their arcs lie within
    private static List<Flat> BandCorners(Piece piece, double a)
    {
        if (!piece.IsArc)
        {
            return [piece.Start, piece.End, piece.TopEnd, piece.TopStart];
        }
        var corners = new List<Flat>();
        ArcControl(corners, piece.Centre, piece.Radius, piece.From, piece.Sweep);
        if (piece.Tight)
        {
            corners.Add(piece.Apex);
        }
        else
        {
            ArcControl(corners, piece.Centre, piece.Radius - piece.Turn * a, piece.From, piece.Sweep);
        }
        return corners;
    }

    private static void ArcControl(List<Flat> into, Flat centre, double radius, double from, double sweep)
    {
        int spans = Math.Max(1, (int)Math.Ceiling(Math.Abs(sweep) / (Math.PI / 2) - 1e-9));
        double step = sweep / spans;
        for (int i = 0; i <= 2 * spans; i++)
        {
            double reach = i % 2 == 0 ? radius : radius / Math.Cos(step / 2);
            into.Add(centre + Flat.At(from + i * step / 2) * reach);
        }
    }

    // The convex hull, anticlockwise (Andrew's monotone chain)
    private static Flat[] Hull(List<Flat> points)
    {
        points.Sort(static (p, q) => p.X != q.X ? p.X.CompareTo(q.X) : p.Y.CompareTo(q.Y));
        var hull = new List<Flat>();
        for (int pass = 0; pass < 2; pass++)
        {
            int floor = hull.Count;
            for (int k = 0; k < points.Count; k++)
            {
                Flat p = points[pass == 0 ? k : points.Count - 1 - k];
                while (hull.Count >= floor + 2 && (hull[^1] - hull[^2]).Cross(p - hull[^2]) <= 0)
                {
                    hull.RemoveAt(hull.Count - 1);
                }
                hull.Add(p);
            }
            hull.RemoveAt(hull.Count - 1);
        }
        return [.. hull];
    }

    /// <summary>How far two convex polygons overlap across the axis that parts them most: negative, they're that far apart.</summary>
    private static double Overlap(Flat[] one, Flat[] two)
    {
        double least = double.MaxValue;
        foreach (Flat[] polygon in (ReadOnlySpan<Flat[]>)[one, two])
        {
            for (int i = 0; i < polygon.Length; i++)
            {
                Flat side = polygon[(i + 1) % polygon.Length] - polygon[i];
                double length = side.Length;
                if (length == 0)
                {
                    continue;
                }
                Flat axis = side.Left * (1 / length);
                (double min1, double max1) = Spread(one, axis);
                (double min2, double max2) = Spread(two, axis);
                least = Math.Min(least, Math.Min(max1, max2) - Math.Max(min1, min2));
            }
        }
        return least;
    }

    private static (double Min, double Max) Spread(Flat[] polygon, Flat axis)
    {
        double min = double.MaxValue, max = double.MinValue;
        foreach (Flat p in polygon)
        {
            double at = p.Dot(axis);
            min = Math.Min(min, at);
            max = Math.Max(max, at);
        }
        return (min, max);
    }

    /// <summary>The solid with the face's band, the sides' tops and the chamfer's faces put in, and the history the Boolean would
    /// give for the shape's own parts.</summary>
    private static (Shape Solid, History History, FacesChanged Faces)? Build(Solid solid, Neighbours neighbours, Face face, in FacePlane frame, List<TightLoop> loops, List<Joint[]> joints)
    {
        var builder = new BrepBuilder();
        var history = new History();
        var replaced = new Dictionary<Shape, Face>(ShapeHasher.Instance);
        var chamfers = new List<Face>();
        var contactWires = new List<Wire>();
        Vector3D down = new Vector3D(frame.Up) * -1;
        for (int l = 0; l < loops.Count; l++)
        {
            TightLoop loop = loops[l];
            Piece[] pieces = loop.Pieces;
            Joint[] at = joints[l];
            int n = pieces.Length;
            double b = loop.B;

            // The vertices at the chamfer's depth and on the contact, the edges down cut short, and the rulings
            for (int i = 0; i < n; i++)
            {
                Joint joint = at[i];
                Vertex low = default;
                builder.MakeVertex(ref low, Tool.Pnt(joint.Top).Translated(down * b), Tolerance);
                joint.Low = low;
                joint.Shortened = Shortened(builder, joint, low, frame, b);
                history.AddModified(joint.Down, joint.Shortened);
                history.AddGenerated(joint.Down, low);
                history.Remove(joint.Top);
            }
            for (int i = 0; i < n; i++)
            {
                // A collapsed corner's two joints share its point
                if (pieces[i].Tight)
                {
                    Vertex apex = default;
                    builder.MakeVertex(ref apex, frame.Lift(pieces[i].Apex, 0), Tolerance);
                    at[i].Contact = at[(i + 1) % n].Contact = apex;
                }
            }
            for (int i = 0; i < n; i++)
            {
                if (at[i].Contact.IsNull)
                {
                    Vertex point = default;
                    builder.MakeVertex(ref point, frame.Lift(pieces[i].TopStart, 0), Tolerance);
                    at[i].Contact = point;
                }
                Point3D from = Tool.Pnt(at[i].Low), to = Tool.Pnt(at[i].Contact);
                at[i].RulingLength = from.DistanceTo(to);
                at[i].Ruling = NewEdge(builder, new LineCurve3D(from, new Direction3D(new Vector3D(from, to))), at[i].Low, 0, at[i].Contact, at[i].RulingLength);
            }

            // Each piece: its edge at the chamfer's depth on its side, its stretch of the contact, and its face of the chamfer
            var contact = new List<Edge>();
            var lowers = new Edge[n];
            for (int i = 0; i < n; i++)
            {
                Piece piece = pieces[i];
                Joint start = at[i], end = at[(i + 1) % n];
                Edge onFace = neighbours.Oriented(face, piece.Edge);
                bool forward = onFace.Orientation == Orientation.Forward;
                Joint naturalFirst = forward ? start : end, naturalLast = forward ? end : start;
                Curve3D curve = (Curve3D)Tool.Curve(piece.Edge, out Placement placement, out double first, out double last)!.Transformed(placement.Transformation);
                Edge lower = NewEdge(builder, (Curve3D)curve.Translated(down * b), naturalFirst.Low, first, naturalLast.Low, last);
                OnSide(builder, piece, lower, frame, b);
                lowers[i] = lower;
                history.AddGenerated(piece.Side, lower);
                history.Remove(piece.Edge);

                Face chamfer;
                if (!piece.IsArc)
                {
                    Edge top = NewEdge(builder, new LineCurve3D(Tool.Pnt(start.Contact), new Direction3D(new Vector3D(Tool.Pnt(start.Contact), Tool.Pnt(end.Contact)))),
                                       start.Contact, 0, end.Contact, Tool.Pnt(start.Contact).DistanceTo(Tool.Pnt(end.Contact)));
                    contact.Add(top);
                    chamfer = Strip(builder, lower.Oriented(onFace.Orientation), start, end, top);
                }
                else if (!piece.Tight)
                {
                    chamfer = Frustum(builder, piece, onFace, lower, naturalFirst, naturalLast, frame, loop, curve, first, last, out Edge top);
                    contact.Add((Edge)top.Oriented(onFace.Orientation));
                }
                else
                {
                    chamfer = Oblique(builder, piece, onFace, lower, start, end, frame, loop, first, last);
                }
                chamfers.Add(chamfer);
            }
            var contactWire = new Wire();
            builder.MakeWire(ref contactWire);
            foreach (Edge edge in contact)
            {
                builder.Add(contactWire, edge);
            }
            contactWire.Closed(true);
            contactWires.Add(contactWire);

            // The sides, cut down to the chamfer
            for (int i = 0; i < n; i++)
            {
                Face side = pieces[i].Side;
                Face cut = (Face)side.EmptyCopied();
                var swap = new Dictionary<Shape, Edge>(ShapeHasher.Instance)
                {
                    [at[i].Down] = at[i].Shortened,
                    [at[(i + 1) % n].Down] = at[(i + 1) % n].Shortened,
                };
                swap[pieces[i].Edge] = lowers[i];
                Rebuilt(builder, side, cut, swap);
                replaced[side] = cut;
                history.AddModified(side, cut);
            }
        }

        // The face, its loops now the contacts
        Face topFace = (Face)face.EmptyCopied();
        foreach (Wire wire in contactWires)
        {
            builder.Add(topFace, wire);
        }
        replaced[face] = topFace;
        history.AddModified(face, topFace);
        foreach (Shape edge in topFace.AllEdges())
        {
            BrepLibrary.BuildPCurveForEdgeOnPlane((Edge)edge, topFace);
        }

        // The shells, each face as the shape had it or as replaced, and the chamfer with the face's shell
        var rebuilt = new Solid();
        builder.MakeSolid(ref rebuilt);
        var changed = new FacesChanged([], [], [.. chamfers]);
        foreach (Shape child in solid.Children)
        {
            if (child.Type != ShapeType.Shell)
            {
                return null;
            }
            var shell = new Shell();
            builder.MakeShell(ref shell);
            bool withFace = false;
            foreach (Shape member in child.Children)
            {
                if (replaced.TryGetValue(member, out Face by))
                {
                    builder.Add(shell, by.Oriented(member.Orientation));
                    withFace |= member.IsSame(face);
                    changed.Gone.Add((Face)member);
                    changed.Added.Add((Face)by.Oriented(member.Orientation));
                }
                else
                {
                    builder.Add(shell, member);
                    changed.Kept.Add((Face)member);
                }
            }
            if (withFace)
            {
                foreach (Face chamfer in chamfers)
                {
                    builder.Add(shell, chamfer);
                }
            }
            shell.Closed(true);
            builder.Add(rebuilt, shell);
        }
        // The faces are as the solid's own orientation had them, so the new one is forward
        Shape result = rebuilt;
        result.TShape()?.FreezeGraph();
        history.AddModified(solid, result);
        return (result, history, changed);
    }

    // An edge on the curve from one vertex to another, its range the parameters given
    private static Edge NewEdge(BrepBuilder builder, Curve3D curve, Vertex from, double first, Vertex to, double last)
    {
        var edge = new Edge();
        builder.MakeEdge(ref edge, curve, Tolerance);
        builder.Add(edge, from.Oriented(Orientation.Forward));
        builder.Add(edge, to.Oriented(Orientation.Reversed));
        builder.Range(edge, first, last);
        builder.UpdateVertex(from, first, edge, Tolerance);
        builder.UpdateVertex(to, last, edge, Tolerance);
        return edge;
    }

    // The edge down from a corner, from the chamfer's depth: its line and curves on the two sides, over less of their range
    private static Edge Shortened(BrepBuilder builder, Joint joint, Vertex low, in FacePlane frame, double b)
    {
        Edge down = (Edge)joint.Down.Oriented(Orientation.Forward);
        double atTop = Tool.Parameter(joint.Top, down), atFoot = Tool.Parameter(joint.Foot, down);
        double depth = frame.Depth(Tool.Pnt(joint.Foot)) - frame.Depth(Tool.Pnt(joint.Top));
        double atLow = atTop + (atFoot - atTop) * b / depth;
        Edge shortened = (Edge)down.EmptyCopied();
        bool upward = atTop < atFoot;
        builder.Add(shortened, low.Oriented(upward ? Orientation.Forward : Orientation.Reversed));
        builder.Add(shortened, joint.Foot.Oriented(upward ? Orientation.Reversed : Orientation.Forward));
        builder.Range(shortened, Math.Min(atLow, atFoot), Math.Max(atLow, atFoot));
        builder.UpdateVertex(low, atLow, shortened, Tolerance);
        return shortened;
    }

    // The new edge along the side at the chamfer's depth: the edge's own curve on the side, moved down the side by b
    private static void OnSide(BrepBuilder builder, Piece piece, Edge lower, in FacePlane frame, double b)
    {
        Curve2D? onSide = Tool.CurveOnSurface(piece.Edge, piece.Side, out _, out _);
        if (onSide is null)
        {
            return;
        }
        var surface = new FaceSurfaceAdaptor(piece.Side);
        Vector3D move = new Vector3D(frame.Up) * -b;
        Vector2D shift = surface.GetType() == SurfaceKind.Plane
            ? new Vector2D(move.Dot(new Vector3D(surface.Plane().Position.XDirection)), move.Dot(new Vector3D(surface.Plane().Position.YDirection)))
            : new Vector2D(0, move.Dot(new Vector3D(surface.Cylinder().Axis.Direction)));
        builder.UpdateEdge(lower, (Curve2D)onSide.Translated(shift), piece.Side, Tolerance);
    }

    // The face as it was, with some of its edges swapped for others in their places, oriented as they were
    private static void Rebuilt(BrepBuilder builder, Face from, Face into, Dictionary<Shape, Edge> swap)
    {
        foreach (Shape child in from.Children)
        {
            var wire = new Wire();
            builder.MakeWire(ref wire);
            foreach (Shape edge in child.Children)
            {
                builder.Add(wire, swap.TryGetValue(edge, out Edge by) ? by.Oriented(edge.Orientation) : edge);
            }
            wire.Closed(child.ClosedFlag);
            builder.Add(into, wire);
        }
    }

    // A straight edge's face of the chamfer: the plane from its edge at depth b to its stretch of the contact, facing out
    private static Face Strip(BrepBuilder builder, Shape lower, Joint start, Joint end, Edge top)
    {
        Point3D low0 = Tool.Pnt(start.Low), low1 = Tool.Pnt(end.Low), top0 = Tool.Pnt(start.Contact);
        var normal = new Direction3D(new Vector3D(low0, low1).Crossed(new Vector3D(low0, top0)));
        var face = new Face();
        builder.MakeFace(ref face, new PlaneSurface(new Plane3D(low0, normal)), new Placement(), Tolerance);
        var wire = new Wire();
        builder.MakeWire(ref wire);
        builder.Add(wire, lower);
        builder.Add(wire, end.Ruling);
        builder.Add(wire, top.Reversed());
        builder.Add(wire, start.Ruling.Reversed());
        wire.Closed(true);
        builder.Add(face, wire);
        foreach (Shape edge in face.AllEdges())
        {
            BrepLibrary.BuildPCurveForEdgeOnPlane((Edge)edge, face);
        }
        return face;
    }

    /// <summary>An arc's face short of its corner's radius: the right cone between the arc at depth b and its contact on F, in the
    /// arc's own frame, so that its angle is the arc's parameter and every curve on it is exact.</summary>
    private static Face Frustum(BrepBuilder builder, Piece piece, Edge onFace, Edge lower, Joint naturalFirst, Joint naturalLast, in FacePlane frame,
                                TightLoop loop, Curve3D curve, double first, double last, out Edge top)
    {
        Circle3D circle = new EdgeCurveAdaptor(piece.Edge).Circle();
        double a = loop.A, b = loop.B, s = circle.Axis.Direction.Dot(frame.Up) > 0 ? 1 : -1;
        double radius = piece.Radius - piece.Turn * a, angle = Math.Atan(-piece.Turn * a * s / b), atLow = -s * b / Math.Cos(angle);
        var surface = new ConicalSurface(new CoordinateSystem(circle.Location, circle.Axis.Direction, circle.XAxis.Direction), angle, radius);
        top = NewEdge(builder, new CircleCurve3D(new Circle3D(circle.Position, radius)), naturalFirst.Contact, first, naturalLast.Contact, last);
        builder.UpdateEdge(top, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), surface, new Placement(), Tolerance);
        builder.UpdateEdge(lower, new LineCurve2D(new Point2D(0, atLow), new Direction2D(1, 0)), surface, new Placement(), Tolerance);
        foreach ((Joint joint, double u) in new[] { (naturalFirst, first), (naturalLast, last) })
        {
            builder.UpdateEdge(joint.Ruling, new LineCurve2D(new Point2D(u, atLow), new Direction2D(0, -Math.Sign(atLow))), surface, new Placement(), Tolerance);
        }

        // Anticlockwise in (u, v): the row at the lower v, the last column, the upper row, the first column
        bool lowerFirst = atLow < 0;
        var face = new Face();
        builder.MakeFace(ref face, surface, new Placement(), Tolerance);
        var wire = new Wire();
        builder.MakeWire(ref wire);
        builder.Add(wire, lowerFirst ? lower : top);
        builder.Add(wire, lowerFirst ? naturalLast.Ruling : naturalLast.Ruling.Reversed());
        builder.Add(wire, lowerFirst ? top.Reversed() : lower.Reversed());
        builder.Add(wire, lowerFirst ? naturalFirst.Ruling.Reversed() : naturalFirst.Ruling);
        wire.Closed(true);
        builder.Add(face, wire);
        return FacingOut(face, surface, (first + last) / 2, atLow / 2, frame);
    }

    /// <summary>A collapsed corner's face: the oblique cone from P to the arc at depth b, exact as a rational B-spline whose rows are
    /// P and the arc. Its edge on the side is the side's arc, with the angle for its parameter, so its curve on the cone is fitted to
    /// it within the tolerance it then has.</summary>
    private static Face Oblique(BrepBuilder builder, Piece piece, Edge onFace, Edge lower, Joint start, Joint end, in FacePlane frame, TightLoop loop,
                                double first, double last)
    {
        (BSplineSurface surface, double spans) = Ruled(frame, piece.Apex, 0, 0, piece.Centre, piece.Radius, loop.B, piece.From, piece.Sweep);

        // u runs from the start of the piece to its end; v from P (0) to the arc (1)
        bool forward = onFace.Orientation == Orientation.Forward;
        builder.UpdateEdge(lower, Linear(new Point2D(forward ? 0 : spans, 1), new Point2D(forward ? spans : 0, 1), first, last), surface, new Placement(), Tolerance);
        builder.SameParameter(lower, false);
        foreach ((Joint joint, double u) in new[] { (start, 0.0), (end, spans) })
        {
            builder.UpdateEdge(joint.Ruling, Linear(new Point2D(u, 1), new Point2D(u, 0), 0, joint.RulingLength), surface, new Placement(), Tolerance);
        }
        var apex = new Edge();
        builder.MakeEdge(ref apex);
        builder.Add(apex, start.Contact.Oriented(Orientation.Forward));
        builder.Add(apex, start.Contact.Oriented(Orientation.Reversed));
        builder.UpdateEdge(apex, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), surface, new Placement(), Tolerance);
        builder.Range(apex, 0, spans);
        builder.Degenerated(apex, true);
        BrepLibrary.SameParameter(lower, Tolerance);

        // Anticlockwise in (u, v): P, the ruling at the end down, the arc back, the ruling at the start up
        var face = new Face();
        builder.MakeFace(ref face, surface, new Placement(), Tolerance);
        var wire = new Wire();
        builder.MakeWire(ref wire);
        builder.Add(wire, apex);
        builder.Add(wire, end.Ruling.Reversed());
        builder.Add(wire, lower.Oriented(forward ? Orientation.Reversed : Orientation.Forward));
        builder.Add(wire, start.Ruling);
        wire.Closed(true);
        builder.Add(face, wire);
        return FacingOut(face, surface, spans / 2, 0.5, frame);
    }

    // A straight line from one point of (u, v) to another over a range of parameters: a B-spline of degree 1
    private static BSplineCurve2D Linear(Point2D from, Point2D to, double first, double last)
    {
        var poles = new NCollection_Array1<Point2D>(1, 2);
        poles.SetValue(1, from);
        poles.SetValue(2, to);
        var knots = new NCollection_Array1<double>(1, 2);
        knots.SetValue(1, first);
        knots.SetValue(2, last);
        var multiplicities = new NCollection_Array1<int>(1, 2);
        multiplicities.SetValue(1, 2);
        multiplicities.SetValue(2, 2);
        return new BSplineCurve2D(poles, knots, multiplicities, 1);
    }

    // The face reversed if its surface's normal points into the material: a chamfer's face always looks up, out of F
    private static Face FacingOut(Face face, Surface surface, double u, double v, in FacePlane frame)
    {
        surface.D1(u, v, out _, out Vector3D du, out Vector3D dv);
        return du.Crossed(dv).Dot(new Vector3D(frame.Up)) < 0 ? (Face)face.Reversed() : face;
    }
}
