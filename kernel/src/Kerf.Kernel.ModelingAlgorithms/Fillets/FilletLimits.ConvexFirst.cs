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
using EngineCut = Kerf.Engine.Booleans.Api.Cut;

namespace Kerf.Fillets;

// Stage 4b (unit 15T2; fillet-design.md, section 15): a rim's convex arc that ends where two concave edges of one cylinder meet it,
// as where a branch's half crosses a cylinder's top. The right corner there is ChFi3d's own on planes: the concave blend runs on over
// the convex edge's blend (decision 2139). ChFi3d builds that in two steps, the convex rim first and then the concave loop, and since
// 15W its walk there is on the ball. So, where its one step has failed: the rim's whole circle is rounded by stage 3's turned tool,
// less the part of it inside the branch's cylinder, so the branch is left whole; then ChFi3d is asked for the rest, the concave edges
// now one loop round the branch over the plane, the rim's torus and the side. Its result is proved by the ball, apart from how it's
// built: at points of each face it made, the ball put back reaches the faces it was built on at the radius and no nearer, and along
// each of that face's edges the face is tangent to the next.
internal static partial class FilletLimits
{
    // Counted for the tests and the sweeps: fillets built this way and returned, and those the ball test refused
    internal static long s_convexFirstBuilt;
    internal static long s_convexFirstRefused;

    private static EngineOutcome? BuildConvexFirst(Shape shape, FilletSpec[] specs, FilletOptions options)
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

        // The arcs, convex, on a circle between a plane and a cylinder about its axis, each end where two concave edges of a
        // cylinder holding the material inside it meet the arc's faces
        var arcs = new List<Edge>();
        var rims = new List<Cutter>();
        var branches = new List<Face>();
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
            bool ends = true;
            foreach (Vertex vertex in edge.AllVertices())
            {
                Face[] others = [.. neighbours.Faces(vertex).Where(f => !f.IsSame(plane) && !f.IsSame(side))];
                if (others.Length == 0)
                {
                    continue; // where the arc meets the next arc of its circle, as at a seam
                }
                if (others.Length != 1 || Branch(others[0]) is null
                    || !neighbours.Edges(vertex).Where(e => !e.IsSame(edge) && !IsSeam(neighbours, e)).All(e => wanted.Contains(e) && IsConcave(neighbours, e)))
                {
                    ends = false;
                    break;
                }
                if (!branches.Any(b => b.IsSame(others[0])))
                {
                    branches.Add(others[0]);
                }
            }
            if (!ends)
            {
                return null;
            }
            Circle3D circle = new EdgeCurveAdaptor(edge).Circle();
            arcs.Add(edge);
            if (!rims.Any(c => c.Rim!.Value.Location.DistanceTo(circle.Location) <= Tolerance && Math.Abs(c.Rim!.Value.Radius - circle.Radius) <= Tolerance))
            {
                double on = radius * corner.ContactPerRadius;
                rims.Add(new Cutter(corner, on, on, radius, circle));
            }
        }
        if (arcs.Count == 0 || branches.Count == 0)
        {
            return null;
        }
        Cancellation.ThrowIfRequested();

        // The rims' tools, each less the branches' cylinders, so that rounding the rim doesn't notch a branch that runs over it
        Extent box = Extent.Of(shape);
        var tools = new List<Shape>();
        foreach (Cutter rim in rims)
        {
            Shape tool = rim.Tool();
            foreach (Face branch in branches)
            {
                if (Boolean(new EngineCut(), tool, [BranchSolid(Branch(branch)!.Value, box)], out _) is not { } less || less.AllSolids().Length == 0)
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
        foreach (Face branch in branches)
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

    // A cylinder face whose material is inside it: the branch, as its frame and radius
    private static (Frame3D Frame, double Radius)? Branch(Face face)
    {
        var surface = new FaceSurfaceAdaptor(face);
        if (surface.GetType() != SurfaceKind.Cylinder)
        {
            return null;
        }
        Cylinder3D cylinder = surface.Cylinder();
        CoordinateSystem position = cylinder.Position;
        BrepTools.UVBounds(face, out double u0, out double u1, out double v0, out double v1);
        surface.D1(0.5 * (u0 + u1), 0.5 * (v0 + v1), out Point3D point, out Vector3D du, out Vector3D dv);
        Vector3D normal = du.Crossed(dv);
        if (face.Orientation == Orientation.Reversed)
        {
            normal = normal.Reversed();
        }
        Vector3D fromAxis = new(position.Location, point);
        fromAxis -= new Vector3D(position.Direction) * fromAxis.Dot(new Vector3D(position.Direction));
        return normal.Dot(fromAxis) > 0 ? (new Frame3D(position.Location, position.Direction, position.XDirection), cylinder.Radius) : null;
    }

    // The branch's cylinder as a solid, long enough to pass through the whole shape
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
