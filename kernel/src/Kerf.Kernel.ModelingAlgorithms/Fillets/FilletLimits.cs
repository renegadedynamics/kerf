// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Kerf.Engine;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using EngineCut = Kerf.Engine.Booleans.Api.Cut;
using EngineFuse = Kerf.Engine.Booleans.Api.Fuse;

namespace Kerf.Fillets;

/// <summary>
/// Fillets and chamfers on straight edges between planes where a face just disappears (decision 1847; docs/fillet-design.md):
/// ChFi3d's analytic case refuses a contact line that lies exactly on a face's boundary, and it has no way to remove a face,
/// though the answer is exact. Its result is checked against the volume and area the geometry gives, and if they differ,
/// it isn't used.
/// </summary>
/// <remarks>Also the arithmetic of how large such a fillet can be (<see cref="Fit"/>), which is asked before ChFi3d: a size
/// within tolerance of the limit is the limit, and one past it is refused at once (decision 1898).</remarks>
internal static class FilletLimits
{
    private const double Tolerance = Tolerances.Confusion;

    // The cosine of 0.2 radians, twice the angle ChFi3d's test of tangency allows
    private const double WellApart = 0.98;

    /// <summary>Which faces and edges meet where, so that a large shape's edges aren't each a search of the whole shape. Every fillet
    /// asks this now, so only which faces an edge is on is found for the whole shape, and the rest when a case at a limit needs it.</summary>
    private sealed class Neighbours
    {
        private readonly Shape _shape;
        private readonly Dictionary<Shape, List<Face>> _facesOfEdge = new(ShapeHasher.Instance);
        private readonly Dictionary<Shape, Dictionary<Shape, Edge>> _edgesOfFace = new(ShapeHasher.Instance);
        private readonly Dictionary<Shape, Direction3D?> _normals = new(ShapeHasher.Instance);
        private Dictionary<Shape, List<Face>>? _facesOfVertex;
        private Dictionary<Shape, List<Edge>>? _edgesOfVertex;

        public Neighbours(Shape shape)
        {
            _shape = shape;
            foreach (Face face in shape.AllFaces())
            {
                foreach (Edge edge in face.AllEdges())
                {
                    Add(_facesOfEdge, edge, face);
                }
            }
        }

        private Dictionary<Shape, Edge> OwnEdges(Face face)
        {
            if (!_edgesOfFace.TryGetValue(face, out Dictionary<Shape, Edge>? own))
            {
                own = new Dictionary<Shape, Edge>(ShapeHasher.Instance);
                foreach (Edge edge in face.AllEdges())
                {
                    own.TryAdd(edge, edge);
                }
                _edgesOfFace[face] = own;
            }
            return own;
        }

        private void FindVertices()
        {
            _facesOfVertex = new(ShapeHasher.Instance);
            _edgesOfVertex = new(ShapeHasher.Instance);
            foreach (Face face in _shape.AllFaces())
            {
                foreach (Vertex vertex in face.AllVertices())
                {
                    Add(_facesOfVertex, vertex, face);
                }
            }
            foreach (Edge edge in _shape.AllEdges())
            {
                foreach (Vertex vertex in edge.AllVertices())
                {
                    Add(_edgesOfVertex, vertex, edge);
                }
            }
        }

        /// <summary>The outward normal of a planar face, or null for any other. Asked once a face, as a surface's adaptor
        /// on a face of many edges isn't cheap.</summary>
        public Direction3D? Normal(Face face)
        {
            if (!_normals.TryGetValue(face, out Direction3D? normal))
            {
                var surface = new FaceSurfaceAdaptor(face);
                normal = surface.GetType() == SurfaceKind.Plane ? OutwardNormal(face, surface.Plane()) : null;
                _normals[face] = normal;
            }
            return normal;
        }

        /// <summary>The face's edges, each as the face holds it.</summary>
        public IReadOnlyCollection<Edge> Edges(Face face) => OwnEdges(face).Values;

        /// <summary>The edge as the face holds it, with its orientation there.</summary>
        public Edge Oriented(Face face, Edge edge) => OwnEdges(face)[edge];

        public IReadOnlyList<Face> Faces(Edge edge) => _facesOfEdge.TryGetValue(edge, out List<Face>? faces) ? faces : [];

        public IReadOnlyList<Face> Faces(Vertex vertex)
        {
            if (_facesOfVertex is null)
            {
                FindVertices();
            }
            return _facesOfVertex!.TryGetValue(vertex, out List<Face>? faces) ? faces : [];
        }

        public IReadOnlyList<Edge> Edges(Vertex vertex)
        {
            if (_edgesOfVertex is null)
            {
                FindVertices();
            }
            return _edgesOfVertex!.TryGetValue(vertex, out List<Edge>? edges) ? edges : [];
        }

        private static void Add<T>(Dictionary<Shape, List<T>> map, Shape key, T value)
            where T : struct, IShape<T>
        {
            if (!map.TryGetValue(key, out List<T>? list))
            {
                map[key] = list = [];
            }
            if (!list.Any(v => v.AsShape().IsSame(value.AsShape())))
            {
                list.Add(value);
            }
        }
    }

    /// <summary>One face beside a straight edge.</summary>
    /// <param name="Face">The face.</param>
    /// <param name="Normal">Its outward normal.</param>
    /// <param name="Inward">The direction in the face, square to the edge, pointing into it.</param>
    /// <param name="Width">The distance to the opposite edge of a rectangular face that a tangent contact can't pass, or null when
    /// the face isn't like that, as nothing is then known.</param>
    /// <param name="Opposite">That opposite edge, which a second fillet may share the width with.</param>
    /// <param name="Tolerance">How closely the shape knows that width: see <see cref="WidthTolerance"/>.</param>
    private readonly record struct Side(Face Face, Direction3D Normal, Direction3D Inward, double? Width, Edge Opposite, double Tolerance);

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

    /// <summary>How one size stands against the faces beside some edges.</summary>
    /// <param name="Limit">The most that fits: above it, a contact line would lie off its face.</param>
    /// <param name="Above">Past the limit by more than the shape's tolerance: proved not to fit.</param>
    /// <param name="AtLimit">Within the shape's tolerance of the limit, either way: it is the limit.</param>
    /// <param name="Over">The edges that don't fit, when it's above.</param>
    public readonly record struct Fit(double Limit, bool Above, bool AtLimit, IReadOnlyList<Edge> Over);

    /// <summary>The upper limit on one radius for all these edges, from the straight ones between planes beside a rectangular
    /// face: no tangent fillet fits above it. Null if none of the edges gives a limit.</summary>
    public static double? LargestRadius(Shape shape, IEnumerable<Edge> edges) =>
        Quietly(() => Judge(Corners(shape, edges), 0, static corner => corner.ContactPerRadius))?.Limit;

    /// <summary>How one radius on all these edges stands, or null if the specs don't give one radius or the edges give no limit.</summary>
    public static Fit? Judge(Shape shape, FilletSpec[] specs) =>
        specs.Any(s => s.EndRadius is not null) || specs.Any(s => s.Radius != specs[0].Radius)
            ? null
            : Quietly(() => Judge(Corners(shape, specs.Select(s => s.Edge)), specs[0].Radius, static corner => corner.ContactPerRadius));

    /// <summary>As for a fillet, for a chamfer of one length on both faces, measured from the edge: a chamfer's length has the same
    /// limit as a contact's distance.</summary>
    public static Fit? Judge(Shape shape, ChamferSpec[] specs, ChamferOptions options) =>
        options.Mode != ChamferMode.Classic || specs.Any(s => s.Angle is not null || (s.Distance2 is { } d2 && d2 != s.Distance) || s.Distance != specs[0].Distance)
            ? null
            : Quietly(() => Judge(Corners(shape, specs.Select(s => s.Edge)), specs[0].Distance, static _ => 1));

    // A shape this can't read has no limit it knows of: the algorithm's turn, which this mustn't stop
    private static Fit? Quietly(Func<Fit?> judge)
    {
        try
        {
            return judge();
        }
        catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
        {
            return null;
        }
    }

    private static Fit? Judge(Dictionary<Shape, Corner> corners, double size, Func<Corner, double> contactPerSize)
    {
        double? largest = null;
        bool atLimit = false;
        var over = new List<Edge>();
        foreach ((Corner corner, Side side) in Sides(corners.Values))
        {
            if (side.Width is { } width)
            {
                double demand = contactPerSize(corner) + (corners.TryGetValue(side.Opposite, out Corner opposite) ? contactPerSize(opposite) : 0);
                largest = Math.Min(largest ?? double.MaxValue, width / demand);
                double past = size * demand - width;
                if (past > side.Tolerance && !over.Any(e => e.IsSame(corner.Edge)))
                {
                    over.Add(corner.Edge);
                }
                atLimit |= Math.Abs(past) <= side.Tolerance;
            }
        }
        if (largest is not { } limit)
        {
            return null;
        }

        // Without the last bits of the tangent's rounding: well within the tolerance, and 10 reads as 10
        double rounded = Math.Round(limit, 9);
        return new Fit(rounded > 0 ? rounded : limit, over.Count > 0, atLimit && over.Count == 0, over);
    }

    /// <summary>The error for a size that doesn't fit, or that only just does and couldn't be built; null for one below the limit.</summary>
    public static Diagnostic? TooLarge(Fit fit, FilletSpec[] specs) =>
        Message([.. specs.Select(s => s.Edge)], "fillet", "radius", specs[0].Radius, fit);

    /// <inheritdoc cref="TooLarge(Fit, FilletSpec[])"/>
    public static Diagnostic? TooLarge(Fit fit, ChamferSpec[] specs) =>
        Message([.. specs.Select(s => s.Edge)], "chamfer", "length", specs[0].Distance, fit);

    /// <summary>The error for a chamfer of two lengths on one edge that doesn't fit, or null.</summary>
    public static Diagnostic? TooLarge(Shape shape, ChamferSpec[] specs, Face[] referenceFaces, ChamferOptions options)
    {
        if (options.Mode == ChamferMode.Classic && specs.Length == 1 && specs[0].Angle is null && specs[0].Distance2 is { } second && second != specs[0].Distance)
        {
            return Read(new Neighbours(shape), specs[0].Edge) is { } corner ? TwoLengths(corner, specs[0].Distance, second, referenceFaces[0]) : null;
        }
        return null;
    }

    private static Diagnostic? TwoLengths(in Corner corner, double onReference, double onOther, Face reference)
    {
        bool oneIsReference = corner.One.Face.IsSame(reference);
        double on1 = oneIsReference ? onReference : onOther, on2 = oneIsReference ? onOther : onReference;
        foreach ((Side side, double length) in new[] { (corner.One, on1), (corner.Two, on2) })
        {
            if (side.Width is { } width && length > width + side.Tolerance)
            {
                Edge edge = corner.Edge;
                return Diagnostic.Error(FilletOperations.TooLarge,
                    $"The chamfer with a length of {length:G6} is too large for {Edges([edge])}: the face it's measured on is {width:G6} wide.").WithShapes([edge]);
            }
        }
        return null;
    }

    private static Diagnostic? Message(Edge[] edges, string what, string size, double value, Fit fit)
    {
        if (fit.Above)
        {
            // To twelve figures: given back as the size, what this says fits is within tolerance of the limit, at any size of shape.
            // The size asked for gets as many as it takes to differ from that
            string most = fit.Limit.ToString("G12", CultureInfo.CurrentCulture);
            string asked = value.ToString("G6", CultureInfo.CurrentCulture);
            asked = asked == fit.Limit.ToString("G6", CultureInfo.CurrentCulture) ? value.ToString("G15", CultureInfo.CurrentCulture) : asked;
            return Diagnostic.Error(FilletOperations.TooLarge,
                $"The {what} with a {size} of {asked} is too large for {Edges(edges)}: the most that fits is {most}, where it uses up the whole width of a face beside the edge.")
                .WithShapes(fit.Over.Select(e => (Shape)e));
        }
        if (fit.AtLimit)
        {
            return Diagnostic.Error(FilletOperations.TooLarge,
                $"The {what} with a {size} of {fit.Limit:G6} uses up the whole width of a face beside {Edges(edges)}, and this case can't be built yet: try a slightly smaller {size}.")
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
        return $"the edge at ({Math.Round((a.X + b.X) / 2, 6):G6}, {Math.Round((a.Y + b.Y) / 2, 6):G6}, {Math.Round((a.Z + b.Z) / 2, 6):G6})";
    }

    /// <summary>The fillet, if each edge is a straight one between planes, the edges share no vertex, and a face just disappears.</summary>
    public static EngineOutcome? Fillet(Shape shape, FilletSpec[] specs, FilletOptions options)
    {
        if (options.Shape != FilletShape.Rational || specs.Any(s => s.EndRadius is not null))
        {
            return null;
        }
        return Safely(() => Build(shape, specs.Select(s => (s.Edge, (s.Radius, s.Radius), (double?)s.Radius))));
    }

    /// <summary>The chamfer, under the same conditions as <see cref="Fillet"/>; a chamfer by an angle isn't handled.</summary>
    public static EngineOutcome? Chamfer(Shape shape, ChamferSpec[] specs, Face[] referenceFaces, ChamferOptions options)
    {
        if (options.Mode != ChamferMode.Classic || specs.Any(s => s.Angle is not null))
        {
            return null;
        }
        return Safely(() => Build(shape, specs.Select(s => (s.Edge, (s.Distance, s.Distance2 ?? s.Distance), (double?)null)), referenceFaces));
    }

    // Nothing here may mask ChFi3d's failure, which has already happened
    private static EngineOutcome? Safely(Func<EngineOutcome?> build)
    {
        try
        {
            return build();
        }
        catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
        {
            return null;
        }
    }

    // sizes: a chamfer's distances on the reference face and the other; a fillet's radius twice, and also as radius
    private static EngineOutcome? Build(Shape shape, IEnumerable<(Edge Edge, (double A, double B) Sizes, double? Radius)> items, Face[]? referenceFaces = null)
    {
        var parts = items.ToArray();
        if (shape.AllSolids().Length != 1 || shape.Type is not (ShapeType.Solid or ShapeType.Compound))
        {
            return null;
        }
        var neighbours = new Neighbours(shape);
        var cuts = new Cutter[parts.Length];
        for (int i = 0; i < parts.Length; i++)
        {
            if (Read(neighbours, parts[i].Edge) is not { } corner || !EndsSquare(neighbours, corner))
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
                    if (demand > width + side.Tolerance)
                    {
                        return null;
                    }
                    if (demand >= width - side.Tolerance)
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
    private static Dictionary<Shape, Corner> Corners(Shape shape, IEnumerable<Edge> edges)
    {
        var neighbours = new Neighbours(shape);
        var corners = new Dictionary<Shape, Corner>(ShapeHasher.Instance);
        foreach (Edge edge in edges)
        {
            if (Read(neighbours, edge) is { } corner)
            {
                corners[edge] = corner;
            }
        }
        return corners;
    }

    private static IEnumerable<(Corner Corner, Side Side)> Sides(IEnumerable<Corner> corners)
    {
        foreach (Corner corner in corners)
        {
            yield return (corner, corner.One);
            yield return (corner, corner.Two);
        }
    }

    private static Corner? Read(Neighbours neighbours, Edge edge)
    {
        if (Tool.Degenerated(edge))
        {
            return null;
        }
        var curve = new EdgeCurveAdaptor(edge);
        if (curve.GetType() != CurveKind.Line)
        {
            return null;
        }
        IReadOnlyList<Face> faces = neighbours.Faces(edge);
        if (faces.Count != 2)
        {
            return null;
        }
        Vertex[] ends = [.. edge.AllVertices().Where(static v => !v.IsNull)];
        if (ends.Length != 2)
        {
            return null;
        }
        Point3D start = curve.Value(curve.FirstParameter()), end = curve.Value(curve.LastParameter());
        if (ReadSide(neighbours, faces[0], edge, start, end) is not { } one || ReadSide(neighbours, faces[1], edge, start, end) is not { } two)
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

    private static Side? ReadSide(Neighbours neighbours, Face face, Edge edge, in Point3D start, in Point3D end)
    {
        if (neighbours.Normal(face) is not { } normal)
        {
            return null;
        }

        // The face lies to the left of its oriented edge, seen from outside; the edge's first as the edge runs
        Edge oriented = neighbours.Oriented(face, edge);
        Direction3D tangent = new(new Vector3D(start, end));
        if (oriented.Orientation == Orientation.Reversed)
        {
            tangent = tangent.Reversed();
        }
        Direction3D inward = normal.Crossed(tangent);
        double? width = Width(neighbours, face, edge, normal, inward, start, out Edge opposite, out double tolerance);
        return new Side(face, normal, inward, width, opposite, tolerance);
    }

    private static Direction3D OutwardNormal(Face face, in Plane3D plane)
    {
        Direction3D normal = plane.Position.XDirection.Crossed(plane.Position.YDirection);
        return face.Orientation == Orientation.Reversed ? normal.Reversed() : normal;
    }

    // The face's width from the edge, if it's a four-sided face of straight edges whose far side is parallel to the edge and
    // ends the plane: past it the next face turns away, by more than the fillet's algorithm calls tangent, so a contact line can't
    // carry on along it
    private static double? Width(Neighbours neighbours, Face face, Edge edge, Direction3D normal, Direction3D inward, in Point3D start, out Edge opposite, out double tolerance)
    {
        opposite = default;
        tolerance = Tolerance;
        IReadOnlyCollection<Edge> edges = neighbours.Edges(face);
        if (edges.Count != 4 || edges.Any(e => new EdgeCurveAdaptor(e).GetType() != CurveKind.Line))
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
        Edge[] across = [.. edges.Where(e => far.All(v => e.AllVertices().Any(w => w.IsSame(v))))];
        if (across.Length != 1)
        {
            return null;
        }
        Edge farEdge = across[0];
        tolerance = WidthTolerance(face, edge, farEdge);
        if (w1 <= tolerance || Math.Abs(w1 - w2) > tolerance)
        {
            return null;
        }
        Face[] beyond = [.. neighbours.Faces(farEdge).Where(f => !f.IsSame(face))];
        if (beyond.Length != 1)
        {
            return null;
        }
        if (neighbours.Normal(beyond[0]) is not { } next || next.Dot(normal) >= 1 - 1e-9)
        {
            return null;
        }

        // An edge ChFi3d takes as tangent, as up to 0.1 radians between planes, is no limit: the fillet goes on over it, and it's
        // right, a ball touching the next face instead (decision 1899; docs/fillet-design.md)
        // It samples the edge, which every fillet would pay for: planes well apart, not coded tangent, don't need that
        bool wellApart = next.Dot(normal) < WellApart && Tool.Continuity(farEdge, face, beyond[0]) == Continuity.C0;
        if (!wellApart && Kerf.Engine.Fillets.ChFi3d.IsTangentFaces(farEdge, face, beyond[0]))
        {
            return null;
        }
        opposite = farEdge;
        return w1;
    }

    // How closely the shape knows a width: the largest tolerance of the face and of the two edges it's measured between, with
    // their vertices. It's in length, whatever the shape's size: a contact line this near the far edge is on it, as the shape can't
    // tell them apart, and a face left narrower than this has no width. Past it, the contact line is off the face whichever way the
    // tolerances lie, which is what makes refusing a proof.
    private static double WidthTolerance(in Face face, in Edge edge, in Edge far)
    {
        double tolerance = Math.Max(Tolerance, Tool.Tolerance(face));
        foreach (Edge e in new[] { edge, far })
        {
            tolerance = Math.Max(tolerance, Tool.Tolerance(e));
            foreach (Vertex vertex in e.AllVertices())
            {
                tolerance = Math.Max(tolerance, Tool.Tolerance(vertex));
            }
        }
        return tolerance;
    }

    // Each end of the edge is a corner of three faces, the third square to the edge: so a section is cut all along it
    private static bool EndsSquare(Neighbours neighbours, in Corner corner)
    {
        Edge edge = corner.Edge;
        Face one = corner.One.Face, two = corner.Two.Face;
        Direction3D along = corner.Along;
        foreach (Vertex end in edge.AllVertices())
        {
            int edgesAtEnd = neighbours.Edges(end).Count(e => !e.IsSame(edge));
            Face[] third = [.. neighbours.Faces(end).Where(f => !f.IsSame(one) && !f.IsSame(two))];
            if (edgesAtEnd != 2 || third.Length != 1)
            {
                return false;
            }
            if (neighbours.Normal(third[0]) is not { } normal || Math.Abs(normal.Dot(along)) < 1 - 1e-9)
            {
                return false;
            }
        }
        return true;
    }

    // Kept apart so that the file reads as the geometry
    private static class Integral
    {
        // Fixed points: planes and cylinders are exact by them, and these are held to one part in a million (decision 1879)
        private static readonly Kerf.Measurement.MassOptions Fixed = new() { Tolerance = 0 };

        public static double Volume(in Shape shape) => Kerf.Measurement.MeasureOperations.IntegrateVolume(shape, Fixed);

        public static double Area(in Shape shape) => Kerf.Measurement.MeasureOperations.IntegrateArea(shape, Fixed);
    }
}
