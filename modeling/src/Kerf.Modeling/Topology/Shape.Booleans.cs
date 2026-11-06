using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Healing;
using Kerf.Measurement;

namespace Kerf.Modeling;

/// <summary>
/// Skips the clean that booleans normally do, for operator-driven code where passing <c>clean: false</c> isn't
/// possible: <c>using (new SkipClean()) { … }</c>. Scopes nest, and apply to the current async flow only.
/// </summary>
public sealed class SkipClean : IDisposable
{
    private static readonly AsyncLocal<int> Depth = new();
    private bool _disposed;

    /// <summary>Starts skipping the clean until disposed.</summary>
    public SkipClean() => Depth.Value++;

    /// <summary>Whether booleans clean their results: true outside every <see cref="SkipClean"/> scope.</summary>
    public static bool Clean => Depth.Value == 0;

    /// <summary>Ends the scope.</summary>
    public void Dispose()
    {
        if (!_disposed)
        {
            _disposed = true;
            Depth.Value--;
        }
    }
}

public static partial class ShapeExtensions
{
    /// <summary>A boolean operation, its result one shape or, of a kind of shape less than a compound, a compound of its pieces.</summary>
    internal static Shape BoolOp(IEnumerable<Shape> args, IEnumerable<Shape> tools, BooleanKind operation, BooleanOptions? options = null)
    {
        var argList = args.ToList();
        var toolList = tools.ToList();
        var highest = argList.Concat(toolList).Select(OrderOf).Max();

        var arg = argList.Where(o => !TopoUtil.IsNothing(o)).ToList();
        var tool = toolList.Where(o => !TopoUtil.IsNothing(o)).ToList();

        // Operations with zero shapes
        Shape? topoResult = null;
        switch (operation)
        {
            case BooleanKind.Cut:
                if (arg.Count == 0)   // nothing less anything is still nothing
                    return default;
                if (tool.Count == 0)
                    topoResult = arg.Count == 1 ? arg[0] : TopoUtil.MakeCompound(arg);
                break;
            case BooleanKind.Fuse:
                if (tool.Count == 0)
                    topoResult = arg.Count == 1 ? arg[0] : TopoUtil.MakeCompound(arg);
                else if (arg.Count == 0)
                    topoResult = tool.Count == 1 ? tool[0] : TopoUtil.MakeCompound(tool);
                break;
            case BooleanKind.Common:
                if (tool.Count == 0 || arg.Count == 0)
                    return default;
                break;
        }

        // The arguments were there before and the tools are brought in. An empty record means every input
        // sub-shape came through untouched, which is what the shortcuts above produce
        var history = new ModelHistory(arg, tool);
        if (topoResult is not { } result)
        {
            // Non-destructive: where an input needs a larger tolerance, OCCT copies it rather than changing it
            var built = Failures.Get(TopoUtil.Boolean(operation, arg, tool, options),
                                     () => $"{TopoUtil.BooleanName(operation)} of {Names(argList)} with {Names(toolList)}");
            result = TopoUtil.Downcast(built.Shape);
            history = ModelHistory.FromHistory(built.History, arg, tool);
        }

        // A shortcut above made nothing, so has nothing new to check
        bool made = topoResult is null;
        string what = $"The {TopoUtil.BooleanName(operation)}'s result";
        if (SkipClean.Clean)
        {
            if (result.Unify(CleanOptions).TryGetValue(out var unified))
            {
                var cleaned = TopoUtil.Downcast(unified.Shape);
                var report = made ? cleaned.Validate() : null;
                // Cleaning is only tidying, so a clean that spoils a good result is left out (unit 11R: 8 of the 600 cases
                // of the Boolean suites)
                if (report is { IsValid: false } && result.Validate().IsValid)
                    Warnings.Warn("Boolean operation unable to clean: that would make its result invalid");
                else
                {
                    if (report is { IsValid: false })
                        throw Failures.Invalid(cleaned, report, what);
                    result = cleaned;
                    history = history.Merge(ModelHistory.FromHistory(unified.History));
                }
            }
            else
            {
                Warnings.Warn("Boolean operation unable to clean");
                if (made)
                    result = Failures.Valid(result, what);
            }
        }
        else if (made)
            result = Failures.Valid(result, what);

        // Remove an unnecessary compound around a single shape
        if (result.IsCompound())
            result = TopoUtil.UnwrapCompound((Compound)result);

        if (result.IsCompound() && highest != 4)
        {
            var pieces = TopoUtil.TopLevelShapes(result);
            foreach (var piece in pieces)
                piece.MadeBy(history);
            return ((Shape)TopoUtil.MakeCompound(pieces)).MadeBy(history);
        }
        return result.MadeBy(history);
    }

    /// <summary>A boolean operation whose pieces are returned as a list, empty for nothing.</summary>
    internal static ShapeList<Shape> BoolOpList(IEnumerable<Shape> args, IEnumerable<Shape> tools, BooleanKind operation)
    {
        var result = BoolOp(args, tools, operation);
        if (TopoUtil.IsNothing(result))
            return [];
        if (result.IsCompound())
        {
            // The same operation made them all
            var record = result.History;
            var pieces = result.GetTopLevelShapes();
            foreach (var piece in pieces)
                piece.MadeBy(record);
            return pieces;
        }
        return [result];
    }

    extension<TSelf>(TSelf self) where TSelf : IShape
    {
        /// <summary>This shape with <paramref name="toCut"/> removed. The result may be of another type.</summary>
        public Shape Cut(params IEnumerable<Shape> toCut) => BoolOp([self.AsShape()], toCut, BooleanKind.Cut);

        /// <summary>As the other forms of <c>Cut</c>.</summary>
        public Shape Cut<T>(IEnumerable<T> toCut) where T : IShape => BoolOp([self.AsShape()], toCut.Select(s => s.AsShape()), BooleanKind.Cut);

        /// <summary>As the other forms of <c>Fuse</c>.</summary>
        public Shape Fuse<T>(IEnumerable<T> toFuse, bool glue = false, double? tolerance = null) where T : IShape =>
            self.Fuse(toFuse.Select(s => s.AsShape()), glue, tolerance);

        /// <summary>
        /// This shape fused with <paramref name="toFuse"/>. <paramref name="glue"/> speeds up shapes that only touch;
        /// <paramref name="tolerance"/> sets a fuzzy value. The result may be of another type.
        /// </summary>
        public Shape Fuse(IEnumerable<Shape> toFuse, bool glue = false, double? tolerance = null)
        {
            var options = new BooleanOptions
            {
                Glue = glue ? GlueMode.Shift : GlueMode.Off,
                FuzzyValue = tolerance is { } tol && tol != 0 ? tol : 0,
            };
            return BoolOp([self.AsShape()], toFuse, BooleanKind.Fuse, options);
        }

        /// <summary>As the other forms of <c>Fuse</c>.</summary>
        public Shape Fuse(params Shape[] toFuse) => self.Fuse((IEnumerable<Shape>)toFuse);

        /// <summary>
        /// Where this shape's body meets (overlaps or crosses) <paramref name="toIntersect"/>: shapes or geometry objects
        /// (<see cref="Point3D"/>, <see cref="Location"/>, <see cref="Axis"/>, <see cref="Plane"/>), each of which converts to a <see cref="CurveLimit"/>.
        /// </summary>
        /// <remarks>
        /// Several arguments are chained: <c>c.Intersect(s1, s2)</c> is c ∩ s1 ∩ s2. A compound argument distributes:
        /// <c>c.Intersect(Compound([s1, s2]))</c> is (c ∩ s1) ∪ (c ∩ s2). <paramref name="includeTouched"/> adds contacts at the
        /// boundary without interior overlap (only where solids are involved).
        /// </remarks>
        public ShapeList<Shape> Intersect(IEnumerable<CurveLimit> toIntersect, double tolerance = 1e-6, bool includeTouched = false)
        {
            var list = toIntersect.Select(o => o.Target).ToList();
            if (list.Count == 0 || self.IsEmpty)
                return [];
            if (list.Any(o => o is IShape { IsEmpty: true }))
                return [];

            var commonSet = new List<Shape> { self.AsShape() };
            foreach (var other in list)
            {
                var nextSet = new List<Shape>();
                foreach (var obj in commonSet)
                    if (IntersectOne(obj, other, tolerance, includeTouched) is { Count: > 0 } result)
                        nextSet.AddRange(result.Expand());
                if (nextSet.Count == 0)
                    return [];   // chained: if any step finds nothing, there is nothing
                commonSet = nextSet.Distinct(ShapeComparer.Same).ToList();
            }
            return new ShapeList<Shape>(commonSet);
        }

        /// <summary>As the other forms of <c>Intersect</c>.</summary>
        public ShapeList<Shape> Intersect(params CurveLimit[] toIntersect) => self.Intersect((IEnumerable<CurveLimit>)toIntersect);

        /// <summary>As the other forms of <c>Intersect</c>, with shapes alone.</summary>
        public ShapeList<Shape> Intersect(IEnumerable<Shape> toIntersect, double tolerance = 1e-6, bool includeTouched = false) =>
            self.Intersect(toIntersect.Select(s => (CurveLimit)s), tolerance, includeTouched);

        /// <summary>
        /// The vertices and edges where this shape and <paramref name="other"/> cross (<c>BRepAlgoAPI_Section</c>).
        /// </summary>
        internal (ShapeList<Vertex> Vertices, ShapeList<Edge> Edges) OcpSection(Shape other)
        {
            if (self.IsEmpty || other.IsEmpty)
                return ([], []);
            // Non-destructive, so the inputs' tolerances are left alone; approximated, with the edges' curves on the faces
            var result = Failures.Get(TopoUtil.Boolean(BooleanKind.Section, [self.AsShape()], [other], ShapeExtensions.ApproximatedSection),
                                      () => "section").Shape;

            var vertices = new List<Vertex>();
            foreach (var vertex in TopoUtil.Explore(result, ShapeType.Vertex))
                vertices.Add((Vertex)(vertex));
            var edges = new List<Edge>();
            foreach (var edge in TopoUtil.Explore(result, ShapeType.Edge))
                edges.Add((Edge)(edge));
            return (new ShapeList<Vertex>(vertices.Distinct(ShapeComparer<Vertex>.Same)), new ShapeList<Edge>(edges));
        }

        /// <summary>Splits this shape with a plane, keeping the pieces <paramref name="keep"/> says (not Both: see <c>SplitBoth</c>).</summary>
        public ShapeList<Shape> Split(Plane tool, Keep keep = Keep.Top) => self.SplitCore(tool, keep).Kept;

        /// <summary>Splits this shape with a face or shell, keeping the pieces <paramref name="keep"/> says.</summary>
        public ShapeList<Shape> Split(Shape tool, Keep keep = Keep.Top) => self.SplitCore(tool, keep).Kept;

        /// <summary>Splits this shape, returning the pieces on top of the tool and those beneath it.</summary>
        public (ShapeList<Shape> Top, ShapeList<Shape> Bottom) SplitBoth(Plane tool)
        {
            var (_, top, bottom) = self.SplitCore(tool, Keep.Both);
            return (top, bottom);
        }

        /// <summary>As the other forms of <c>SplitBoth</c>.</summary>
        public (ShapeList<Shape> Top, ShapeList<Shape> Bottom) SplitBoth(Shape tool)
        {
            var (_, top, bottom) = self.SplitCore(tool, Keep.Both);
            return (top, bottom);
        }

        /// <summary>
        /// The split: the pieces kept (all of them for <see cref="Keep.All"/>), and those on top of and beneath the tool.
        /// build123d returns one shape, a list or the zero of the class; here a side with nothing on it is an empty list.
        /// </summary>
        private (ShapeList<Shape> Kept, ShapeList<Shape> Top, ShapeList<Shape> Bottom) SplitCore(object tool, Keep keep)
        {
            if (tool is IShape { IsEmpty: true })
                throw new ArgumentException("Can't split with an empty tool");
            if (keep is Keep.Inside or Keep.Outside)
                throw new ArgumentException($"{keep} is invalid");
            if (self.IsEmpty)   // nothing to split; nothing on either side
                return ([], [], []);

            var trimTool = tool switch
            {
                Plane p => ShapeExtensions.MakePlaneFace(p),
                IShape s => s.AsShape(),
                _ => throw new ArgumentException($"Can't split with a {tool.GetType().Name}"),
            };
            var splitResult = TopoUtil.BoolOp([self.AsShape()], [trimTool], BooleanKind.Split);
            var pieces = TopoUtil.TopLevelShapes(splitResult);

            if (keep == Keep.All)
                return (new ShapeList<Shape>(pieces), [], []);

            Shape? toolSolid = null;
            if (tool is not Plane)
            {
                // A reference point off the +ve side of the tool decides which side is up
                Face toolFace = trimTool.IsShell()
                    ? trimTool.Explore<Face>().First()
                    : (Face)trimTool;
                var (us, vs) = toolFace.UVBounds;
                double u = (us.Min + us.Max) / 2, v = (vs.Min + vs.Max) / 2;
                var surfacePnt = toolFace.ToSurface().Evaluate(u, v);
                var surfaceNormal = TopoUtil.FaceNormalVector(toolFace, u, v);
                var refPoint = surfacePnt + surfaceNormal.Normalized();
                var refPnt = refPoint;
                var halfSpace = trimTool.IsShell()
                    ? Primitives.HalfSpace((Shell)trimTool, refPnt)
                    : Primitives.HalfSpace((Face)trimTool, refPnt);
                toolSolid = Failures.Get(halfSpace, () => "half-space").Shape;
            }

            var tops = new List<Shape>();
            var bottoms = new List<Shape>();
            foreach (var part in pieces)
            {
                var subShape = (part);
                bool isUp;
                if (tool is Plane plane)
                {
                    // A piece lies wholly on one side of the plane and so does its centre of mass. Its geometric centre
                    // need not: that of a curved face is the middle of its surface, trimmed away or not.
                    isUp = plane.ToLocalCoords(ShapeExtensions.CentreOfMass(part)).Z >= 0;
                }
                else
                {
                    // Intersect the piece and the thickened tool
                    var isUpObj = TopoUtil.BoolOp([part], [toolSolid!.Value], BooleanKind.Common);
                    // Mass is the total length for linear properties
                    isUp = isUpObj.IntegrateLength() >= Constants.Tolerance;
                }
                (isUp ? tops : bottoms).Add(subShape);
            }
            var top = new ShapeList<Shape>(tops);
            var bottom = new ShapeList<Shape>(bottoms);
            return keep switch
            {
                Keep.Top => (top, top, bottom),
                Keep.Bottom => (bottom, top, bottom),
                _ => ([.. top, .. bottom], top, bottom),
            };
        }
    }

    /// <summary>Flattens boolean operands into their top-level shapes, dropping nulls.</summary>
    internal static List<Shape> Operands(IEnumerable<Shape>? others) =>
        others is null ? [] : others.OfType<Shape>().SelectMany(o => o.GetTopLevelShapes()).ToList();

    /// <summary>A boolean operation, its result cast to the highest-ranked class among the inputs.</summary>
    /// <summary>The inputs of a boolean as its error message names them: "a Solid", "2 Faces and a Shell".</summary>
    private static string Names(IEnumerable<Shape> shapes)
    {
        var groups = shapes.GroupBy(s => s.GetType().Name).Select(g => g.Count() == 1 ? $"a {g.Key}" : $"{g.Count()} {g.Key}s").ToList();
        return groups.Count switch
        {
            0 => "nothing",
            1 => groups[0],
            _ => string.Join(", ", groups[..^1]) + " and " + groups[^1],
        };
    }

    /// <summary>A face on the plane, unbounded as far as OCCT's defaults go.</summary>
    internal static Face MakePlaneFace(Plane plane)
    {
        var pln = plane.ToPlane3D();
        return Failures.Get(Face.FromPlane(pln), () => "plane face");
    }

    // OCCT's BRepAlgoAPI_Section as build123d sets it up for a section between two shapes
    private static readonly BooleanOptions ApproximatedSection = new() { ApproximateSection = true, SectionCurvesOnFaces = true };

    /// <summary>The clean after a Boolean: UnifySameDomain on edges and faces, B-splines concatenated, no internal edges.</summary>
    internal static readonly UnifyOptions CleanOptions = new() { Edges = true, Faces = true, ConcatBSplines = true, AllowInternalEdges = false, CheckResult = false };

    /// <summary>The centre of mass of a shape by its dimension: its length, area or volume.</summary>
    internal static Point3D CentreOfMass(Shape shape) =>
        shape.MassProperties(TopoUtil.Dim(shape) switch
        {
            1 => Dimension.Length,
            2 => Dimension.Area,
            _ => Dimension.Volume,
        }).CentreOfMass;
}
