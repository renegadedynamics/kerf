// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using EngineCommon = Kerf.Engine.Booleans.Api.Common;
using EngineCut = Kerf.Engine.Booleans.Api.Cut;
using EngineFuse = Kerf.Engine.Booleans.Api.Fuse;

namespace Kerf.Fillets;

// Stage 4b (unit 15T2; fillet-design.md, section 15): a rim's convex arc that ends where two concave edges of one cylinder meet it,
// as where a branch's half crosses a cylinder's top. The right corner there is ChFi3d's own on planes: the concave blend runs on over
// the convex edge's blend (decision 2139). ChFi3d builds that in two steps, the convex rim first and then the concave loop, and since
// 15W its walk there is on the ball. So, where its one step has failed: the rim's whole circle is rounded by stage 3's turned tool,
// less the part of it inside the branch's cylinder, so the branch is left whole; then ChFi3d is asked for the rest, the concave edges
// now one loop round the branch over the plane, the rim's torus and the side. Its result is proved by the ball, apart from how it's
// built: at points of each face it made, the ball put back reaches the faces it was built on at the radius and no nearer, and along
// each of that face's edges the face is tangent to the next.
// Unit 15T4 (section 17): the face an arc ends on may be a plane, or a cylinder or a cone with the material on either side of it, and
// the rim's tool is the whole turn less what fills each gap between its arcs; and where ChFi3d's one step has built such a corner
// with faces that aren't tangent to one another, this stage's result is taken in its place.
internal static partial class FilletLimits
{
    // Counted for the tests and the sweeps: fillets built this way and returned, and those the ball test refused
    internal static long s_convexFirstBuilt;
    internal static long s_convexFirstRefused;

    // Counted for the tests: results of ChFi3d's own set aside for this stage's
    internal static long s_convexFirstInstead;

    /// <summary>What's built in place of ChFi3d's own result (unit 15T4): where a rim's arc ends on two concave edges, ChFi3d's one step
    /// can be valid, without crossings, and wrong, its faces at the corner not tangent to one another and off the ball. So where the
    /// edges asked for hold such an arc and two of the faces ChFi3d made meet at an angle, this stage's result, which is proved, is
    /// taken if there is one; if there's none, ChFi3d's stands as before.</summary>
    public static EngineOutcome? InsteadOfAlgorithm(Shape shape, FilletSpec[] specs, FilletOptions options, MakeFillet fillet, Func<Shape> result)
    {
        if (options.Shape != FilletShape.Rational || specs.Any(static s => s.EndRadius is not null)
            || !specs.Any(static s => s.Edge.AllVertices().Length == 2 && new EdgeCurveAdaptor(s.Edge).GetType() == CurveKind.Circle))
        {
            return null;
        }
        EngineOutcome? instead = Safely(() => BuildConvexFirst(shape, specs, options, unless: () => Smooth(fillet, specs, result(), options.Tolerances.Angular)));
        if (instead is not null)
        {
            Interlocked.Increment(ref s_convexFirstInstead);
        }
        return instead;
    }

    // Whether the faces ChFi3d made from the edges asked for and their ends are tangent to one another wherever two of them meet
    private static bool Smooth(MakeFillet fillet, FilletSpec[] specs, Shape result, double angular)
    {
        var made = new HashSet<Shape>(ShapeHasher.Instance);
        foreach (FilletSpec spec in specs)
        {
            foreach (Shape from in (IEnumerable<Shape>)[spec.Edge, .. spec.Edge.AllVertices()])
            {
                foreach (Shape face in fillet.Generated(from))
                {
                    if (face.Type == ShapeType.Face)
                    {
                        made.Add(face);
                    }
                }
            }
        }
        var neighbours = new Neighbours(result);
        var seen = new HashSet<Shape>(ShapeHasher.Instance);
        ReadOnlySpan<double> at = [0.2, 0.5, 0.8];
        foreach (Shape face in made)
        {
            foreach (Edge edge in neighbours.Edges(ShapeCasts.Face(face)))
            {
                IReadOnlyList<Face> faces = neighbours.Faces(edge);
                if (!seen.Add(edge) || faces.Count != 2 || faces[0].IsSame(faces[1]) || Tool.Degenerated(edge) || !made.Contains(faces[0]) || !made.Contains(faces[1]))
                {
                    continue;
                }
                Tool.Range(edge, out double a, out double b);
                foreach (double s in at)
                {
                    double tm = a + s * (b - a);
                    Vector3D n1 = OutwardAt(faces[0], edge, tm), n2 = OutwardAt(faces[1], edge, tm);
                    if (n1.Magnitude > 1e-12 && n2.Magnitude > 1e-12 && n1.Dot(n2) < Math.Cos(angular) * n1.Magnitude * n2.Magnitude)
                    {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    private static EngineOutcome? BuildConvexFirst(Shape shape, FilletSpec[] specs, FilletOptions options, Func<bool>? unless = null)
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
        var neighbours = new Neighbours(shape);
        var wanted = new HashSet<Shape>(ShapeHasher.Instance);
        var rest = new List<Edge>();
        foreach (FilletSpec spec in specs)
        {
            IReadOnlyList<Face> faces = neighbours.Faces(spec.Edge);
            if (faces.Count >= 1 && faces.All(f => f.IsSame(faces[0])))
            {
                continue; // a seam, which ChFi3d passes over
            }
            if (faces.Count != 2 || !wanted.Add(spec.Edge))
            {
                continue;
            }
            rest.Add(spec.Edge);
        }

        // The arcs, convex, on a circle between a plane and a cylinder about its axis, each end where two concave edges of a third
        // face meet the arc's faces: a plane, or a cylinder or a cone with the material on either side of it
        var arcs = new List<Edge>();
        var rims = new List<(Cutter Cutter, List<ArcEnd> Ends)>();
        var thirds = new List<Face>();
        foreach (Edge edge in rest)
        {
            if (ReadRim(neighbours, edge, arc: true) is not { IsConvex: true } corner)
            {
                continue;
            }
            Face plane = neighbours.Normal(corner.One.Face) is not null ? corner.One.Face : corner.Two.Face;
            Face side = plane.IsSame(corner.One.Face) ? corner.Two.Face : corner.One.Face;
            if (neighbours.Normal(plane) is null || new FaceSurfaceAdaptor(side).GetType() != SurfaceKind.Cylinder)
            {
                continue;
            }
            var curve = new EdgeCurveAdaptor(edge);
            Circle3D circle = curve.Circle();
            int rim = rims.FindIndex(c => c.Cutter.Rim!.Value.Location.DistanceTo(circle.Location) <= Tolerance && Math.Abs(c.Cutter.Rim!.Value.Radius - circle.Radius) <= Tolerance);
            if (rim < 0)
            {
                double on = radius * corner.ContactPerRadius;
                rims.Add((new Cutter(corner, on, on, radius, circle), []));
                rim = rims.Count - 1;
            }
            Circle3D reference = rims[rim].Cutter.Rim!.Value;
            double halfway = AngleOn(reference, curve.Value(0.5 * (curve.FirstParameter() + curve.LastParameter())));
            foreach (Vertex vertex in edge.AllVertices())
            {
                Face[] others = [.. neighbours.Faces(vertex).Where(f => !f.IsSame(plane) && !f.IsSame(side))];
                if (others.Length == 0)
                {
                    continue; // where the arc meets the next arc of its circle, as at a seam
                }
                if (others.Length != 1 || Third.Read(neighbours, others[0]) is not { } third
                    || !neighbours.Edges(vertex).Where(e => !e.IsSame(edge) && !IsSeam(neighbours, e)).All(e => wanted.Contains(e) && IsConcave(neighbours, e)))
                {
                    return null;
                }
                double angle = AngleOn(reference, Tool.Pnt(vertex));
                rims[rim].Ends.Add(new ArcEnd(angle, Math.Sin(halfway - angle) > 0, third));
                if (!thirds.Any(b => b.IsSame(others[0])))
                {
                    thirds.Add(others[0]);
                }
            }
            arcs.Add(edge);
        }
        if (arcs.Count == 0 || thirds.Count == 0 || unless?.Invoke() == true)
        {
            return null;
        }
        Cancellation.ThrowIfRequested();

        // The rims' tools. Each gap in a rim, between one arc's end and the next one's start, is filled by what the faces at those two
        // ends hold: the part common to their two solids (a rib between two planes), or failing that their union, whichever holds the
        // rim across the gap and none of an arc asked for; an arc of the circle that isn't asked for has to lie in a gap. The tool is
        // the whole turn less the gaps' solids, so rounding the rim doesn't notch what runs over it, whatever the arcs span
        Extent box = Extent.Of(shape);
        var tools = new List<Shape>();
        foreach ((Cutter cutter, List<ArcEnd> ends) in rims)
        {
            Circle3D circle = cutter.Rim!.Value;
            ends.Sort(static (a, b) => a.Angle.CompareTo(b.Angle));
            var onArcs = new List<Point3D>();
            var offArcs = new List<Point3D>();
            var seenEdges = new HashSet<Shape>(ShapeHasher.Instance);
            foreach (Edge edge in shape.AllEdges())
            {
                if (!seenEdges.Add(edge) || Tool.Degenerated(edge))
                {
                    continue;
                }
                var curve = new EdgeCurveAdaptor(edge);
                if (curve.GetType() != CurveKind.Circle || curve.Circle().Location.DistanceTo(circle.Location) > Tolerance
                    || Math.Abs(curve.Circle().Radius - circle.Radius) > Tolerance || Math.Abs(curve.Circle().Axis.Direction.Dot(circle.Axis.Direction)) < 1 - 1e-9)
                {
                    continue;
                }
                List<Point3D> points = arcs.Any(a => a.IsSame(edge)) ? onArcs : offArcs;
                for (int i = 1; i < 10; i++)
                {
                    points.Add(curve.Value(curve.FirstParameter() + (curve.LastParameter() - curve.FirstParameter()) * i / 10));
                }
            }
            var gaps = new List<RimGap>();
            for (int i = 0; i < ends.Count; i++)
            {
                ArcEnd from = ends[i], to = ends[(i + 1) % ends.Count];
                if (from.ArcAbove == to.ArcAbove)
                {
                    return null;
                }
                if (from.ArcAbove)
                {
                    continue; // an arc runs between these two
                }
                double width = to.Angle - from.Angle + (i + 1 < ends.Count ? 0 : 2 * Math.PI);
                Point3D[] across = [.. Enumerable.Range(1, 7).Select(k => PointOn(circle, from.Angle + width * k / 8))];
                bool same = from.Third.Equals(to.Third);
                RimGap? found = null;
                foreach (bool both in same ? [true] : (ReadOnlySpan<bool>)[true, false])
                {
                    var gap = new RimGap(from.Third, to.Third, same, both);
                    if (All(across, p => gap.Depth(p) >= -Tolerance) && onArcs.All(p => gap.Depth(p) <= Tolerance))
                    {
                        found = gap;
                        break;
                    }
                }
                if (found is not { } held)
                {
                    return null;
                }
                if (!gaps.Contains(held))
                {
                    gaps.Add(held);
                }
            }
            if (gaps.Count == 0 || offArcs.Any(p => gaps.All(g => g.Depth(p) < -Tolerance)))
            {
                return null;
            }
            Shape tool = cutter.Tool();
            foreach (RimGap gap in gaps)
            {
                if (gap.Solid(box) is not { } solid || Boolean(new EngineCut(), tool, [solid], out _) is not { } less || less.AllSolids().Length == 0)
                {
                    return null;
                }
                tool = less;
            }
            tools.Add(tool);
        }
        if (Boolean(new EngineCut(), shape, tools, out History? rounded, simplify: true) is not { } first || first.AllSolids().Length != 1
            || Validity.Problem(first.AllSolids()[0], "The fillet") is not null)
        {
            return null;
        }
        Shape middle = first.AllSolids()[0];
        History history = rounded ?? new History();

        // The rims' new faces, and the rest of the edges as the first step left them, with the edges where the branches now meet those
        // faces: one loop round each branch
        var before = new HashSet<Shape>(ShapeHasher.Instance);
        foreach (Face face in shape.AllFaces())
        {
            before.Add(face);
        }
        var middleNeighbours = new Neighbours(middle);
        Face[] made = [.. middle.AllFaces().Where(f => !before.Contains(f) && new FaceSurfaceAdaptor(f).GetType() == SurfaceKind.Torus)];
        if (made.Length == 0)
        {
            return null;
        }
        var onBranch = new HashSet<Shape>(ShapeHasher.Instance);
        foreach (Face branch in thirds)
        {
            onBranch.Add(branch);
            foreach (Shape image in history.Modified(branch))
            {
                onBranch.Add(image);
            }
        }
        var second = new List<Edge>();
        var seen = new HashSet<Shape>(ShapeHasher.Instance);
        foreach (Edge edge in rest)
        {
            if (arcs.Any(a => a.IsSame(edge)))
            {
                continue;
            }
            IReadOnlyList<Shape> images = history.Modified(edge);
            if (images.Count == 0 && history.IsRemoved(edge))
            {
                continue; // a concave edge the rim has taken all of: its place is the branch's new edge with the rim
            }
            foreach (Shape image in images.Count > 0 ? images : [edge])
            {
                Edge e = ShapeCasts.Edge(image);
                if (middleNeighbours.Faces(e).Count == 2 && seen.Add(e))
                {
                    second.Add(e);
                }
            }
        }
        foreach (Face torus in made)
        {
            foreach (Edge edge in middleNeighbours.Edges(torus))
            {
                IReadOnlyList<Face> faces = middleNeighbours.Faces(edge);
                if (faces.Count == 2 && faces.Any(f => onBranch.Contains(f)) && seen.Add(edge))
                {
                    second.Add(ShapeCasts.Edge(edge.Oriented(Orientation.Forward)));
                }
            }
        }
        Cancellation.ThrowIfRequested();

        // ChFi3d, on the rest
        var fillet = new MakeFillet(middle, options.Shape);
        FilletTolerances t = options.Tolerances;
        fillet.SetParams(t.Angular, t.Spine, t.Parametric, t.Approximation3D, t.Approximation2D, t.Deflection);
        fillet.SetContinuity(options.InternalContinuity, options.AngularTolerance);
        foreach (Edge edge in second)
        {
            fillet.Add(radius, edge);
        }
        EngineOutcome outcome = MakerOutcome.Of(fillet, default, [middle]);
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
        foreach (Edge edge in second)
        {
            foreach (Shape face in fillet.Generated(edge))
            {
                if (!blends.Any(b => b.IsSame(face)))
                {
                    blends.Add(ShapeCasts.Face(face));
                }
            }
        }
        if (Validity.Problem(result, "The fillet") is not null || blends.Count == 0 || !BallOnFaces(middle, result, blends, radius, t))
        {
            Interlocked.Increment(ref s_convexFirstRefused);
            return null;
        }

        // The history: the first step's, then ChFi3d's; each arc generated the rim's faces as they are now
        history.Merge(outcome.History);
        var tori = new List<Shape>();
        foreach (Face torus in made)
        {
            IReadOnlyList<Shape> images = outcome.History?.Modified(torus) ?? [];
            tori.AddRange(images.Count > 0 ? images : [torus]);
        }
        foreach (Edge arc in arcs)
        {
            foreach (Shape torus in tori)
            {
                history.AddGenerated(arc, torus);
            }
            history.Remove(arc);
        }
        Interlocked.Increment(ref s_convexFirstBuilt);
        return new EngineOutcome(true, () => result, history, Valid: true);
    }

    private static bool IsSeam(Neighbours neighbours, Edge edge)
    {
        IReadOnlyList<Face> faces = neighbours.Faces(edge);
        return faces.Count >= 1 && faces.All(f => f.IsSame(faces[0]));
    }

    private static bool All(Point3D[] points, Func<Point3D, bool> holds) => Array.TrueForAll(points, p => holds(p));

    // Where a point of a circle's plane lies round it, and the point of it at an angle
    private static double AngleOn(in Circle3D circle, in Point3D point)
    {
        Vector3D from = new(circle.Location, point);
        double angle = Math.Atan2(from.Dot(new Vector3D(circle.Position.YDirection)), from.Dot(new Vector3D(circle.Position.XDirection)));
        return angle < 0 ? angle + 2 * Math.PI : angle;
    }

    private static Point3D PointOn(in Circle3D circle, double angle) =>
        circle.Location.Translated(new Vector3D(circle.Position.XDirection) * (circle.Radius * Math.Cos(angle))
                                   + new Vector3D(circle.Position.YDirection) * (circle.Radius * Math.Sin(angle)));

    // An arc's end on a third face: where it is round the rim, and whether its arc runs on from it as the angle rises
    private readonly record struct ArcEnd(double Angle, bool ArcAbove, Third Third);

    // The face an arc ends on, as the solid its surface bounds: behind a plane, or inside or outside a cylinder or a cone. The frame's
    // direction is the plane's outward normal, or the axis; a cone's radius is its surface's at the frame's origin
    private readonly record struct Third(SurfaceKind Kind, Frame3D Frame, double Radius, double SemiAngle, bool Inside)
    {
        public static Third? Read(Neighbours neighbours, Face face)
        {
            var surface = new FaceSurfaceAdaptor(face);
            CoordinateSystem position;
            double radius, semiAngle = 0;
            switch (surface.GetType())
            {
                case SurfaceKind.Plane:
                    Plane3D plane = surface.Plane();
                    return new Third(SurfaceKind.Plane, new Frame3D(plane.Location, neighbours.Normal(face)!.Value), 0, 0, true);
                case SurfaceKind.Cylinder:
                    position = surface.Cylinder().Position;
                    radius = surface.Cylinder().Radius;
                    break;
                case SurfaceKind.Cone:
                    position = surface.Cone().Position;
                    radius = surface.Cone().RefRadius;
                    semiAngle = surface.Cone().SemiAngle;
                    break;
                default:
                    return null;
            }

            // The material is inside if the face's outward normal points away from the axis
            BrepTools.UVBounds(face, out double u0, out double u1, out double v0, out double v1);
            surface.D1(0.5 * (u0 + u1), 0.5 * (v0 + v1), out Point3D point, out Vector3D du, out Vector3D dv);
            Vector3D normal = du.Crossed(dv);
            if (face.Orientation == Orientation.Reversed)
            {
                normal = normal.Reversed();
            }
            Vector3D fromAxis = new(position.Location, point);
            fromAxis -= new Vector3D(position.Direction) * fromAxis.Dot(new Vector3D(position.Direction));
            return new Third(surface.GetType(), new Frame3D(position.Location, position.Direction, position.XDirection), radius, semiAngle, normal.Dot(fromAxis) > 0);
        }

        // How far inside the solid a point is; negative outside
        public double Depth(in Point3D point)
        {
            Vector3D from = new(Frame.Location, point);
            double along = from.Dot(new Vector3D(Frame.Direction));
            if (Kind == SurfaceKind.Plane)
            {
                return -along;
            }
            double across = (from - new Vector3D(Frame.Direction) * along).Magnitude;
            double depth = (Radius + along * Math.Tan(SemiAngle) - across) * Math.Cos(SemiAngle);
            return Inside ? depth : -depth;
        }

        // The solid, large enough to hold all of the shape that it should
        public Shape? Solid(Extent box)
        {
            Vector3D axis = new(Frame.Direction);
            double reach = box.Diagonal + 1;
            double centre = new Vector3D(Frame.Location, box.Centre).Dot(axis);
            if (Kind == SurfaceKind.Plane)
            {
                Point3D foot = box.Centre.Translated(axis * -centre);
                Point3D corner = foot.Translated((new Vector3D(Frame.XDirection) + new Vector3D(Frame.YDirection)) * -reach + axis * (-2 * reach));
                return new MakeBox(new Frame3D(corner, Frame.Direction, Frame.XDirection), 2 * reach, 2 * reach, 2 * reach).Shape();
            }
            Shape inside;
            if (Kind == SurfaceKind.Cylinder)
            {
                inside = BranchSolid((Frame, Radius), box);
            }
            else
            {
                // The cone's own nappe, from its apex if that's within reach
                double tangent = Math.Tan(SemiAngle), from = centre - reach, to = centre + reach;
                if (Math.Abs(tangent) < 1e-12)
                {
                    return null;
                }
                double apex = -Radius / tangent;
                (from, to) = tangent > 0 ? (Math.Max(from, apex), to) : (from, Math.Min(to, apex));
                if (to - from <= Tolerance)
                {
                    return null;
                }
                var frame = new Frame3D(Frame.Location.Translated(axis * from), Frame.Direction, Frame.XDirection);
                inside = new ApiMakeCone(frame, Math.Max(0, Radius + from * tangent), Math.Max(0, Radius + to * tangent), to - from).Shape();
            }
            if (Inside)
            {
                return inside;
            }
            double half = box.Diagonal / 2 + 1;
            Shape all = new MakeBox(box.Centre.Translated(new Vector3D(-half, -half, -half)), 2 * half, 2 * half, 2 * half).Shape();
            return Boolean(new EngineCut(), all, [inside], out _);
        }
    }

    // What fills a rim's gap between two arcs: the part common to the solids of the faces at its two ends, or their union
    private readonly record struct RimGap(Third From, Third To, bool Same, bool Both)
    {
        public double Depth(in Point3D point) =>
            Same ? From.Depth(point) : Both ? Math.Min(From.Depth(point), To.Depth(point)) : Math.Max(From.Depth(point), To.Depth(point));

        public Shape? Solid(Extent box)
        {
            if (From.Solid(box) is not { } from)
            {
                return null;
            }
            if (Same)
            {
                return from;
            }
            return To.Solid(box) is { } to ? Boolean(Both ? new EngineCommon() : new EngineFuse(), from, [to], out _) : null;
        }
    }

    // A cylinder as a solid, long enough to pass through the whole shape
    private static Shape BranchSolid((Frame3D Frame, double Radius) branch, Extent box)
    {
        Vector3D along = new(branch.Frame.Direction);
        double reach = box.Diagonal + 1;
        double centre = new Vector3D(branch.Frame.Location, box.Centre).Dot(along);
        Point3D start = branch.Frame.Location.Translated(along * (centre - reach));
        return new ApiMakeCylinder(new Frame3D(start, branch.Frame.Direction, branch.Frame.XDirection), branch.Radius, 2 * reach).Shape();
    }

    private readonly record struct Extent(Point3D Centre, double Diagonal)
    {
        public static Extent Of(Shape shape)
        {
            var box = new Box();
            Kerf.Engine.Measurement.Bounds.Add(shape, ref box, false);
            box.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
            return new Extent(new Point3D((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), Math.Sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0)));
        }
    }

    // Whether an edge between two faces is concave: the second face's surface turns towards the first's outward side
    private static bool IsConcave(Neighbours neighbours, Edge edge)
    {
        IReadOnlyList<Face> faces = neighbours.Faces(edge);
        if (faces.Count != 2)
        {
            return false;
        }
        Tool.Range(edge, out double a, out double b);
        var curve = new EdgeCurveAdaptor(edge);
        double tm = 0.5 * (a + b);
        curve.D1(tm, out Point3D at, out Vector3D tangent);
        Vector3D n1 = OutwardAt(faces[0], edge, tm), n2 = OutwardAt(faces[1], edge, tm);

        // Into the first face, square to the edge: the edge as the face holds it runs with the face on its left
        Edge held = neighbours.Oriented(faces[0], edge);
        Vector3D t = held.Orientation == Orientation.Reversed ? tangent.Reversed() : tangent;
        Vector3D inward = n1.Crossed(t);
        return inward.Dot(n2) > 1e-9 * inward.Magnitude * n2.Magnitude;
    }

    // A face's outward normal at a point of one of its edges
    private static Vector3D OutwardAt(Face face, Edge edge, double t)
    {
        Curve2D? pcurve = Tool.CurveOnSurface(edge, face, out _, out _);
        if (pcurve is null)
        {
            return default;
        }
        Point2D uv = pcurve.Value(t);
        var surface = new FaceSurfaceAdaptor(face);
        surface.D1(uv.X, uv.Y, out _, out Vector3D du, out Vector3D dv);
        Vector3D normal = du.Crossed(dv);
        return face.Orientation == Orientation.Reversed ? normal.Reversed() : normal;
    }

    // For the tests: the ball test, asked of any shape the faces are said to be built on
    internal static bool BallOnFacesForTests(Shape middle, Shape result, Face[] blends, double radius) =>
        BallOnFaces(middle, result, [.. blends], radius, FilletOptions.Default.Tolerances);

    // The ball test on what ChFi3d made in the second step, apart from how: at points inside each new face the ball put back, on
    // whichever side, reaches the faces of the shape it was built on at the radius, to ChFi3d's approximation, and so touches them
    // and enters none; and along each of the face's edges the face is tangent to the next, to ChFi3d's angle
    private static bool BallOnFaces(Shape middle, Shape result, List<Face> blends, double radius, FilletTolerances t)
    {
        var builder = new BrepBuilder();
        var boundary = new Compound();
        builder.MakeCompound(ref boundary);
        foreach (Face face in middle.AllFaces())
        {
            builder.Add(boundary, face);
        }
        double allowed = Math.Max(t.Approximation3D, 2 * Target(radius));
        var resultNeighbours = new Neighbours(result);
        ReadOnlySpan<double> at = [0.2, 0.5, 0.8];
        foreach (Face face in blends)
        {
            Cancellation.ThrowIfRequested();
            BrepTools.UVBounds(face, out double u0, out double u1, out double v0, out double v1);
            var surface = new FaceSurfaceAdaptor(face);
            int tried = 0;
            foreach (double su in at)
            {
                foreach (double sv in at)
                {
                    double u = u0 + su * (u1 - u0), v = v0 + sv * (v1 - v0);
                    if (new FaceClassifier(face, new Point2D(u, v), Tolerance).State() != Containment.In)
                    {
                        continue;
                    }
                    surface.D1(u, v, out Point3D point, out Vector3D du, out Vector3D dv);
                    Vector3D normal = du.Crossed(dv);
                    if (normal.Magnitude <= 1e-12)
                    {
                        continue;
                    }
                    normal *= radius / normal.Magnitude;
                    double error = double.MaxValue;
                    foreach (Vector3D offset in (ReadOnlySpan<Vector3D>)[normal, normal.Reversed()])
                    {
                        var vertex = new Vertex();
                        builder.MakeVertex(ref vertex, point.Translated(offset), Tolerance);
                        var distance = new DistanceShapeShape(vertex, boundary);
                        if (distance.IsDone)
                        {
                            error = Math.Min(error, Math.Abs(distance.Value() - radius));
                        }
                    }
                    if (error > allowed)
                    {
                        return false;
                    }
                    tried++;
                }
            }
            if (tried == 0)
            {
                return false;
            }
            foreach (Edge edge in resultNeighbours.Edges(face))
            {
                IReadOnlyList<Face> faces = resultNeighbours.Faces(edge);
                if (faces.Count != 2 || faces[0].IsSame(faces[1]) || Tool.Degenerated(edge))
                {
                    continue;
                }
                Tool.Range(edge, out double a, out double b);
                foreach (double s in at)
                {
                    double tm = a + s * (b - a);
                    Vector3D n1 = OutwardAt(faces[0], edge, tm), n2 = OutwardAt(faces[1], edge, tm);
                    if (n1.Magnitude <= 1e-12 || n2.Magnitude <= 1e-12)
                    {
                        continue;
                    }
                    if (n1.Dot(n2) < Math.Cos(t.Angular) * n1.Magnitude * n2.Magnitude)
                    {
                        return false;
                    }
                }
            }
        }
        return true;
    }
}
