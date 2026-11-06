// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using EngineCut = Kerf.Engine.Booleans.Api.Cut;
using EngineFuse = Kerf.Engine.Booleans.Api.Fuse;

namespace Kerf.Fillets;

/// <summary>
/// Fillets and chamfers on straight edges between planes where a face just disappears (decision 21201; docs/fillet-design.md):
/// ChFi3d's analytic case refuses a contact line that lies exactly on a face's boundary, and it has no way to remove a face,
/// though the answer is exact. Asked only after ChFi3d has failed, so nothing that works changes. Its result is checked
/// against the volume and area the geometry gives, and if they differ, ChFi3d's failure stands.
/// </summary>
/// <remarks>Also the arithmetic of how large such a fillet can be, for the messages and <c>MaxFillet</c>.</remarks>
internal static class FilletLimits
{
    private const double Tolerance = Tolerances.Confusion;

    /// <summary>One face beside a straight edge.</summary>
    /// <param name="Face">The face.</param>
    /// <param name="Normal">Its outward normal.</param>
    /// <param name="Inward">The direction in the face, square to the edge, pointing into it.</param>
    /// <param name="Width">The distance to the opposite edge of a rectangular face that a tangent contact can't pass, or null when
    /// the face isn't like that, as nothing is then known.</param>
    /// <param name="Opposite">That opposite edge, which a second fillet may share the width with.</param>
    private readonly record struct Side(Face Face, Direction3D Normal, Direction3D Inward, double? Width, Edge Opposite);

    /// <summary>A straight edge between two planes.</summary>
    private readonly record struct Corner(Edge Edge, Point3D Start, Point3D End, Side One, Side Two)
    {
        public Direction3D Along => new(new Vector3D(Start, End));

        public double Length => Start.DistanceTo(End);

        /// <summary>The angle between the faces, on the side away from the material when the corner is convex.</summary>
        public double Angle => One.Inward.AngleTo(Two.Inward);

        /// <summary>The material is the wedge between the faces: a fillet here cuts, rather than adds.</summary>
        public bool IsConvex => One.Normal.Dot(Two.Inward) < 0;

        /// <summary>How far along each face a fillet's contact lies, for each unit of radius.</summary>
        public double ContactPerRadius => 1 / Math.Tan(Angle / 2);
    }

    /// <summary>The upper limit on one radius for all these edges, from the straight ones between planes beside a rectangular
    /// face: no tangent fillet fits above it. Null if none of the edges gives a limit.</summary>
    public static double? LargestRadius(Shape shape, IEnumerable<Edge> edges)
    {
        Corner[] corners = [.. Corners(shape, edges)];
        double? largest = null;
        foreach ((Corner corner, Side side) in Sides(corners))
        {
            if (side.Width is { } width)
            {
                double demand = corner.ContactPerRadius + corners.Where(c => c.Edge.IsSame(side.Opposite)).Sum(c => c.ContactPerRadius);
                largest = Math.Min(largest ?? double.MaxValue, width / demand);
            }
        }

        // Without the last bits of the tangent's rounding: well within the tolerance, and 10 reads as 10
        return largest is { } limit ? Math.Round(limit, 9) : null;
    }

    /// <summary>The error saying that a failed fillet's radii don't fit, or that they only just do, if the geometry says so.</summary>
    public static Diagnostic? TooLarge(Shape shape, FilletSpec[] specs)
    {
        if (specs.Any(s => s.EndRadius is not null) || specs.Any(s => s.Radius != specs[0].Radius))
        {
            return null;
        }
        Edge[] edges = [.. specs.Select(s => s.Edge)];
        return LargestRadius(shape, edges) is { } largest ? Message(shape, edges, "fillet", "radius", specs[0].Radius, largest) : null;
    }

    /// <summary>As <see cref="TooLarge(Shape, FilletSpec[])"/>, for a chamfer with one length: a chamfer's length has the same limit
    /// as a contact's distance.</summary>
    public static Diagnostic? TooLarge(Shape shape, ChamferSpec[] specs, Face[] referenceFaces)
    {
        if (specs.Any(s => s.Angle is not null))
        {
            return null;
        }
        if (specs.Length == 1 && specs[0].Distance2 is { } second && second != specs[0].Distance)
        {
            return Read(shape, specs[0].Edge) is { } corner ? TwoLengths(corner, specs[0].Distance, second, referenceFaces[0]) : null;
        }
        if (specs.Any(s => s.Distance2 is { } d2 && d2 != s.Distance) || specs.Any(s => s.Distance != specs[0].Distance))
        {
            return null;
        }
        Edge[] edges = [.. specs.Select(s => s.Edge)];
        Corner[] corners = [.. Corners(shape, edges)];
        double? largest = null;
        foreach ((Corner corner, Side side) in Sides(corners))
        {
            if (side.Width is { } width)
            {
                largest = Math.Min(largest ?? double.MaxValue, width / (1 + corners.Count(c => c.Edge.IsSame(side.Opposite))));
            }
        }
        return largest is { } limit ? Message(shape, edges, "chamfer", "length", specs[0].Distance, limit) : null;
    }

    private static Diagnostic? TwoLengths(in Corner corner, double onReference, double onOther, Face reference)
    {
        bool oneIsReference = corner.One.Face.IsSame(reference);
        double on1 = oneIsReference ? onReference : onOther, on2 = oneIsReference ? onOther : onReference;
        foreach ((Side side, double length) in new[] { (corner.One, on1), (corner.Two, on2) })
        {
            if (side.Width is { } width && length > width + Tolerance)
            {
                Edge edge = corner.Edge;
                return Diagnostic.Error(FilletOperations.TooLarge,
                    $"A chamfer length of {length:G6} is too large for {Edges([edge])}: the face it's measured on is {width:G6} wide.").WithShapes([edge]);
            }
        }
        return null;
    }

    private static Diagnostic? Message(Shape shape, Edge[] edges, string what, string size, double value, double largest)
    {
        if (value > largest + Tolerance)
        {
            return Diagnostic.Error(FilletOperations.TooLarge,
                $"A {what} {size} of {value:G6} is too large for {Edges(edges)}: the most that fits is {largest:G6}, where it uses up the whole width of a face beside the edge.")
                .WithShapes(edges.Select(e => (Shape)e));
        }
        if (value >= largest - Tolerance)
        {
            return Diagnostic.Error(FilletOperations.TooLarge,
                $"A {what} {size} of {value:G6} uses up the whole width of a face beside {Edges(edges)}, and this case can't be built yet: try a slightly smaller {size}.")
                .WithShapes(edges.Select(e => (Shape)e));
        }
        return null;
    }

    private static string Edges(Edge[] edges)
    {
        if (edges.Length != 1)
        {
            return $"these {edges.Length} edges";
        }
        Vertex[] ends = [.. edges[0].AllVertices()];
        if (ends.Length != 2)
        {
            return "this edge";
        }
        Point3D a = Tool.Pnt(ends[0]), b = Tool.Pnt(ends[1]);
        return $"the edge at ({(a.X + b.X) / 2:G6}, {(a.Y + b.Y) / 2:G6}, {(a.Z + b.Z) / 2:G6})";
    }

    /// <summary>The fillet, if each edge is a straight one between planes, the edges share no vertex, and a face just disappears.</summary>
    public static EngineOutcome? Fillet(Shape shape, FilletSpec[] specs, FilletOptions options)
    {
        if (options.Shape != FilletShape.Rational || specs.Any(s => s.EndRadius is not null))
        {
            return null;
        }
        return Build(shape, specs.Select(s => (s.Edge, (s.Radius, s.Radius), (double?)s.Radius)));
    }

    /// <summary>The chamfer, under the same conditions as <see cref="Fillet"/>; a chamfer by an angle isn't handled.</summary>
    public static EngineOutcome? Chamfer(Shape shape, ChamferSpec[] specs, Face[] referenceFaces, ChamferOptions options)
    {
        if (options.Mode != ChamferMode.Classic || specs.Any(s => s.Angle is not null))
        {
            return null;
        }
        return Build(shape, specs.Select((s, i) => (s.Edge, (s.Distance, s.Distance2 ?? s.Distance), (double?)null)), referenceFaces);
    }

    // sizes: a chamfer's distances on the reference face and the other; a fillet's radius twice, and also as radius
    private static EngineOutcome? Build(Shape shape, IEnumerable<(Edge Edge, (double A, double B) Sizes, double? Radius)> items, Face[]? referenceFaces = null)
    {
        var parts = items.ToArray();
        if (shape.AllSolids().Length != 1 || shape.Type is not (ShapeType.Solid or ShapeType.Compound))
        {
            return null;
        }
        var cuts = new Cutter[parts.Length];
        for (int i = 0; i < parts.Length; i++)
        {
            if (Read(shape, parts[i].Edge) is not { } corner || !EndsSquare(shape, corner))
            {
                return null;
            }
            double on1, on2;
            if (parts[i].Radius is { } r)
            {
                on1 = on2 = r * corner.ContactPerRadius;
            }
            else
            {
                bool referenceIsOne = referenceFaces is null || corner.One.Face.IsSame(referenceFaces[i]);
                on1 = referenceIsOne ? parts[i].Sizes.A : parts[i].Sizes.B;
                on2 = referenceIsOne ? parts[i].Sizes.B : parts[i].Sizes.A;
            }
            cuts[i] = new Cutter(corner, on1, on2, parts[i].Radius);
        }

        // The edges have to be apart, with one convexity, as one Boolean makes them all
        for (int i = 0; i < cuts.Length; i++)
        {
            if (cuts[i].Corner.IsConvex != cuts[0].Corner.IsConvex)
            {
                return null;
            }
            for (int j = i + 1; j < cuts.Length; j++)
            {
                if (cuts[i].Corner.Edge.IsSame(cuts[j].Corner.Edge) || cuts[i].Corner.Edge.AllVertices().Any(v => cuts[j].Corner.Edge.AllVertices().Any(w => w.IsSame(v))))
                {
                    return null;
                }
            }
        }

        // Only at a limit: a face used up. A hair past one (or short of it) is the limit; further past, nothing fits.
        bool atLimit = false;
        foreach (Cutter cut in cuts)
        {
            foreach (Side side in new[] { cut.Corner.One, cut.Corner.Two })
            {
                if (side.Width is { } width)
                {
                    double demand = cuts.Sum(c => c.ContactOn(side.Face));
                    if (demand > width + Tolerance)
                    {
                        return null;
                    }
                    if (demand >= width - Tolerance)
                    {
                        atLimit = true;
                        double scale = width / demand;
                        for (int k = 0; k < cuts.Length; k++)
                        {
                            if (cuts[k].ContactOn(side.Face) > 0)
                            {
                                cuts[k] = cuts[k].ScaledOn(side.Face, scale);
                            }
                        }
                    }
                }
            }
        }
        if (!atLimit)
        {
            return null;
        }

        BooleanOperation operation = cuts[0].Corner.IsConvex ? new EngineCut() : new EngineFuse();
        var tools = new NCollection_List<Shape>();
        foreach (Cutter cut in cuts)
        {
            tools.Append(cut.Tool());
        }
        var arguments = new NCollection_List<Shape>();
        arguments.Append(shape);
        operation.SetArguments(arguments);
        operation.SetTools(tools);
        operation.SetNonDestructive(true);
        operation.SetToFillHistory(true);
        operation.SetRunParallel(false);
        operation.Build(default);
        if (!operation.IsDone || operation.HasErrors)
        {
            return null;
        }
        if (!cuts[0].Corner.IsConvex)
        {
            operation.SimplifyResult();
        }
        Shape result = operation.Shape();
        if (result.AllSolids().Length != 1)
        {
            return null;
        }
        Shape solid = result.AllSolids()[0];
        double sign = cuts[0].Corner.IsConvex ? -1 : 1;
        double volume = Integral.Volume(shape) + sign * cuts.Sum(c => c.Corner.Length * c.Section);
        double area = Integral.Area(shape) + cuts.Sum(c => c.Corner.Length * (c.NewWidth - c.On1 - c.On2) + sign * 2 * c.Section);
        if (Math.Abs(Integral.Volume(solid) - volume) > 1e-6 * Math.Max(1, volume) || Math.Abs(Integral.Area(solid) - area) > 1e-6 * Math.Max(1, area)
            || Validity.Problem(solid, "The fillet") is not null)
        {
            return null;
        }

        // A fillet's edges are coded tangent, as ChFi3d's are (decision 1719): a draft reads that
        Shape[] newEdges = [.. solid.AllFaces().Where(f => cuts.Any(c => c.IsNewFace(f))).SelectMany(f => f.AllEdges()).Select(e => (Shape)e)];
        solid = BrepLibrary.RegularityEncoded(solid, newEdges, out History coded);

        // The new faces, and the edges they came from; the Boolean knows the rest
        History history = operation.History() is { } made ? new History(made) : new History();
        history.Merge(coded);
        foreach (Cutter cut in cuts)
        {
            Face[] faces = [.. solid.AllFaces().Where(cut.IsNewFace)];
            if (faces.Length == 0)
            {
                return null;
            }
            foreach (Face face in faces)
            {
                history.AddGenerated(cut.Corner.Edge, face);
            }
            history.Remove(cut.Corner.Edge);
        }
        Shape built = solid;
        return new EngineOutcome(true, () => built, history);
    }

    /// <summary>What's cut from one corner, and how.</summary>
    private readonly record struct Cutter(Corner Corner, double On1, double On2, double? Radius)
    {
        public double ContactOn(Face face) =>
            (Corner.One.Face.IsSame(face) ? On1 : 0) + (Corner.Two.Face.IsSame(face) ? On2 : 0);

        public Cutter ScaledOn(Face face, double scale)
        {
            if (Radius is { } r)
            {
                return this with { On1 = On1 * scale, On2 = On2 * scale, Radius = r * scale };
            }
            return this with { On1 = Corner.One.Face.IsSame(face) ? On1 * scale : On1, On2 = Corner.Two.Face.IsSame(face) ? On2 * scale : On2 };
        }

        /// <summary>The area of the section taken away, or added.</summary>
        public double Section
        {
            get
            {
                if (Radius is { } r)
                {
                    return r * On1 - r * r * (Math.PI - Corner.Angle) / 2;
                }
                return On1 * On2 * Math.Sin(Corner.Angle) / 2;
            }
        }

        /// <summary>The width round the new face: its arc, or a chamfer's straight line.</summary>
        public double NewWidth => Radius is { } r
            ? r * (Math.PI - Corner.Angle)
            : Contact1(Corner.Start).DistanceTo(Contact2(Corner.Start));

        private Point3D Contact1(in Point3D at) => at.Translated(new Vector3D(Corner.One.Inward) * On1);

        private Point3D Contact2(in Point3D at) => at.Translated(new Vector3D(Corner.Two.Inward) * On2);

        private Point3D Centre(in Point3D at)
        {
            Direction3D middle = new(new Vector3D(Corner.One.Inward) + new Vector3D(Corner.Two.Inward));
            return at.Translated(new Vector3D(middle) * (Radius!.Value / Math.Sin(Corner.Angle / 2)));
        }

        /// <summary>The solid to cut or add. One to cut stands out past the faces and the edge's ends, so that it shares no face
        /// with the shape; one to add has to be flush with both.</summary>
        public Shape Tool()
        {
            Direction3D along = Corner.Along;
            double margin = Corner.IsConvex ? Math.Max(1, Math.Max(Corner.Length, Math.Max(On1, On2))) : 0;
            Point3D at = Corner.Start.Translated(new Vector3D(along) * -margin);
            Point3D p1 = Contact1(at), p2 = Contact2(at);
            var wire = new MakeWire();
            wire.Add(Curved(at, p1, p2));
            if (Corner.IsConvex)
            {
                Vector3D n1 = new(Corner.One.Normal), n2 = new(Corner.Two.Normal);
                Point3D a = p2.Translated(n2 * margin);
                Point3D corner = at.Translated((n1 + n2) * (margin / (1 + Corner.One.Normal.Dot(Corner.Two.Normal))));
                Point3D b = p1.Translated(n1 * margin);
                wire.Add(new MakeEdge(p2, a).Edge());
                wire.Add(new MakeEdge(a, corner).Edge());
                wire.Add(new MakeEdge(corner, b).Edge());
                wire.Add(new MakeEdge(b, p1).Edge());
            }
            else
            {
                wire.Add(new MakeEdge(p2, at).Edge());
                wire.Add(new MakeEdge(at, p1).Edge());
            }
            Direction3D normal = along;
            Face section = new MakeFace(new Plane3D(at, normal), wire.Wire()).Face();
            return new ApiMakePrism(section, new Vector3D(along) * (Corner.Length + 2 * margin)).Shape();
        }

        private Edge Curved(in Point3D at, in Point3D p1, in Point3D p2)
        {
            if (Radius is not { } r)
            {
                return new MakeEdge(p1, p2).Edge();
            }
            Point3D centre = Centre(at);
            Direction3D axis = new(new Vector3D(centre, p1).Crossed(new Vector3D(centre, p2)));
            var circle = new Circle3D(new Frame3D(centre, axis, new Direction3D(new Vector3D(centre, p1))), r);
            return new MakeEdge(circle, p1, p2).Edge();
        }

        /// <summary>Whether a face of the result lies on the fillet's cylinder, or the chamfer's plane.</summary>
        public bool IsNewFace(Face face)
        {
            var surface = new FaceSurfaceAdaptor(face);
            if (Radius is { } r)
            {
                if (surface.GetType() != SurfaceKind.Cylinder)
                {
                    return false;
                }
                Cylinder3D c = surface.Cylinder();
                return Math.Abs(c.Radius - r) <= Tolerance && Math.Abs(c.Axis.Direction.Dot(Corner.Along)) >= 1 - 1e-9
                       && new Line3D(c.Axis).DistanceTo(Centre(Corner.Start)) <= Tolerance;
            }
            if (surface.GetType() != SurfaceKind.Plane)
            {
                return false;
            }
            Plane3D plane = surface.Plane();
            return plane.DistanceTo(Contact1(Corner.Start)) <= Tolerance && plane.DistanceTo(Contact2(Corner.Start)) <= Tolerance
                   && plane.DistanceTo(Contact1(Corner.End)) <= Tolerance;
        }
    }

    // The corners among the edges
    private static IEnumerable<Corner> Corners(Shape shape, IEnumerable<Edge> edges)
    {
        foreach (Edge edge in edges)
        {
            if (Read(shape, edge) is { } corner)
            {
                yield return corner;
            }
        }
    }

    private static IEnumerable<(Corner Corner, Side Side)> Sides(Corner[] corners)
    {
        foreach (Corner corner in corners)
        {
            yield return (corner, corner.One);
            yield return (corner, corner.Two);
        }
    }

    private static Corner? Read(Shape shape, Edge edge)
    {
        var curve = new EdgeCurveAdaptor(edge);
        if (curve.GetType() != CurveKind.Line)
        {
            return null;
        }
        Face[] faces = [.. shape.AllFaces().Where(f => f.AllEdges().Any(e => e.IsSame(edge)))];
        if (faces.Length != 2 || faces[0].IsSame(faces[1]))
        {
            return null;
        }
        Vertex[] ends = [.. edge.AllVertices().Where(static v => !v.IsNull)];
        if (ends.Length != 2)
        {
            return null;
        }
        Point3D start = curve.Value(curve.FirstParameter()), end = curve.Value(curve.LastParameter());
        if (ReadSide(shape, faces[0], edge, start, end) is not { } one || ReadSide(shape, faces[1], edge, start, end) is not { } two)
        {
            return null;
        }

        // Two faces in one plane aren't a corner
        double angle = one.Inward.AngleTo(two.Inward);
        if (angle < 1e-6 || angle > Math.PI - 1e-6)
        {
            return null;
        }
        return new Corner(edge, start, end, one, two);
    }

    private static Side? ReadSide(Shape shape, Face face, Edge edge, in Point3D start, in Point3D end)
    {
        var surface = new FaceSurfaceAdaptor(face);
        if (surface.GetType() != SurfaceKind.Plane)
        {
            return null;
        }
        Direction3D normal = OutwardNormal(face, surface.Plane());

        // The face lies to the left of its oriented edge, seen from outside; the edge's first as the edge runs
        Edge oriented = face.AllEdges().First(e => e.IsSame(edge));
        Direction3D tangent = new(new Vector3D(start, end));
        if (oriented.Orientation == Orientation.Reversed)
        {
            tangent = tangent.Reversed();
        }
        Direction3D inward = normal.Crossed(tangent);
        double? width = Width(shape, face, edge, normal, inward, start, out Edge opposite);
        return new Side(face, normal, inward, width, opposite);
    }

    private static Direction3D OutwardNormal(Face face, in Plane3D plane)
    {
        Direction3D normal = plane.Position.XDirection.Crossed(plane.Position.YDirection);
        return face.Orientation == Orientation.Reversed ? normal.Reversed() : normal;
    }

    // The face's width from the edge, if it's a four-sided face of straight edges whose far side is parallel to the edge and
    // ends the plane: past it the next face turns away, so a contact line can't carry on along it
    private static double? Width(Shape shape, Face face, Edge edge, Direction3D normal, Direction3D inward, in Point3D start, out Edge opposite)
    {
        opposite = default;
        Edge[] edges = [.. face.AllEdges()];
        if (edges.Length != 4 || edges.Any(e => new EdgeCurveAdaptor(e).GetType() != CurveKind.Line))
        {
            return null;
        }
        Vertex[] ends = [.. edge.AllVertices()];
        Vertex[] far = [.. face.AllVertices().Where(v => !ends.Any(w => w.IsSame(v)))];
        if (far.Length != 2)
        {
            return null;
        }
        Point3D from = start;
        Vector3D toFirst = new(from, Tool.Pnt(far[0])), toSecond = new(from, Tool.Pnt(far[1]));
        double w1 = toFirst.Dot(new Vector3D(inward)), w2 = toSecond.Dot(new Vector3D(inward));
        if (w1 <= Tolerance || Math.Abs(w1 - w2) > Tolerance)
        {
            return null;
        }
        Edge[] across = [.. edges.Where(e => far.All(v => e.AllVertices().Any(w => w.IsSame(v))))];
        if (across.Length != 1)
        {
            return null;
        }
        Edge farEdge = across[0];
        Face[] beyond = [.. shape.AllFaces().Where(f => !f.IsSame(face) && f.AllEdges().Any(e => e.IsSame(farEdge)))];
        if (beyond.Length != 1)
        {
            return null;
        }
        var next = new FaceSurfaceAdaptor(beyond[0]);
        if (next.GetType() != SurfaceKind.Plane)
        {
            return null;
        }
        if (OutwardNormal(beyond[0], next.Plane()).Dot(normal) >= 1 - 1e-9)
        {
            return null;
        }
        opposite = farEdge;
        return w1;
    }

    // Each end of the edge is a corner of three faces, the third square to the edge: so a section is cut all along it
    private static bool EndsSquare(Shape shape, in Corner corner)
    {
        Edge edge = corner.Edge;
        Face one = corner.One.Face, two = corner.Two.Face;
        Direction3D along = corner.Along;
        foreach (Vertex end in edge.AllVertices())
        {
            int edgesAtEnd = shape.AllEdges().Count(e => !e.IsSame(edge) && e.AllVertices().Any(v => v.IsSame(end)));
            Face[] third = [.. shape.AllFaces().Where(f => !f.IsSame(one) && !f.IsSame(two) && f.AllVertices().Any(v => v.IsSame(end)))];
            if (edgesAtEnd != 2 || third.Length != 1)
            {
                return false;
            }
            var surface = new FaceSurfaceAdaptor(third[0]);
            if (surface.GetType() != SurfaceKind.Plane || Math.Abs(OutwardNormal(third[0], surface.Plane()).Dot(along)) < 1 - 1e-9)
            {
                return false;
            }
        }
        return true;
    }

    // Kept apart so that the file reads as the geometry
    private static class Integral
    {
        public static double Volume(in Shape shape) => Kerf.Measurement.MeasureOperations.IntegrateVolume(shape);

        public static double Area(in Shape shape) => Kerf.Measurement.MeasureOperations.IntegrateArea(shape);
    }
}
