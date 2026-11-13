using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Healing;
using Kerf.Measurement;

namespace Kerf.Modeling;

/// <summary>
/// Skips the clean that booleans normally do, for operator-driven code where passing <c>clean: false</c> isn't
/// possible: <c>using (new SkipClean()) { … }</c>. Scopes nest, and apply to the current async flow only.
/// </summary>
/// <remarks>
/// A scope is in force from its construction until it is disposed, on the flow that opened it and in the tasks and
/// parallel loops started from it, and nowhere after: disposing it ends it everywhere, on whichever thread or task the
/// <c>Dispose</c> runs, and a task that outlives it cleans again (unless an enclosing scope is still open). Other
/// callers, on other threads, never see it.
/// </remarks>
public sealed class SkipClean : IDisposable
{
    // The innermost scope opened on this flow; an ended one is passed over to its parent
    private static readonly AsyncLocal<SkipClean?> Innermost = new();
    private readonly SkipClean? _parent;
    private volatile bool _ended;

    /// <summary>Starts skipping the clean until disposed.</summary>
    public SkipClean()
    {
        _parent = Live(Innermost.Value);
        Innermost.Value = this;
    }

    /// <summary>Whether booleans clean their results: true outside every <see cref="SkipClean"/> scope still open.</summary>
    public static bool Clean => Live(Innermost.Value) is null;

    /// <summary>Ends the scope, wherever it is disposed: on the flow that opened it, and in the tasks started from it.</summary>
    public void Dispose()
    {
        _ended = true;
        if (ReferenceEquals(Innermost.Value, this))
            Innermost.Value = Live(_parent);
    }

    // The innermost scope from this one out that hasn't ended
    private static SkipClean? Live(SkipClean? scope)
    {
        while (scope is { _ended: true })
            scope = scope._parent;
        return scope;
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
        Diagnostic? wrong = null;
        ModelingException? invalid = null;
        try
        {
            (result, history, wrong) = Finish(operation, arg, tool, options, result, history, made, proved: false);
        }
        catch (ModelingException failure) when (made && RuleOf(operation) is not null)
        {
            // An invalid result is made again as a wrong one is, and fails as it did if that doesn't mend it (decision 2366)
            invalid = failure;
        }
        if (wrong is not null || invalid is not null)
        {
            // Made again as the kernel's checked Booleans are, the other way round, then with a fuzzy value, then with its
            // crossings held apart (decisions 1937, 2103 and 2364), its warning passed on; what that makes is cleaned and
            // checked as the first was
            var retried = BooleanOperations.Retry(RuleOf(operation)!.Value, arg, tool, (options ?? BooleanOptions.Default) with { Simplify = false, CheckResult = true });
            if (!retried.TryGetValue(out var again))
                throw invalid ?? Failures.Wrong(result, wrong!);
            foreach (var diagnostic in retried.Diagnostics)
                if (!diagnostic.IsError)
                    Warnings.Warn(diagnostic);
            (result, history, _) = Finish(operation, arg, tool, options, TopoUtil.Downcast(again.Shape),
                                          ModelHistory.FromHistory(again.History, arg, tool), made, proved: true);
        }

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

    /// <summary>
    /// A Boolean's result cleaned and checked: the clean left out where it would spoil a good result, and the error if its volume
    /// proves it wrong. <paramref name="proved"/> says the kernel has already proved it right before the clean, so a clean that
    /// changes its volume is left out without checking it again.
    /// </summary>
    private static (Shape Result, ModelHistory History, Diagnostic? Wrong) Finish(BooleanKind operation, List<Shape> arg, List<Shape> tool,
        BooleanOptions? options, Shape result, ModelHistory history, bool made, bool proved)
    {
        string what = $"The {TopoUtil.BooleanName(operation)}'s result";
        var (uncleaned, uncleanedHistory) = (result, history);
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

        return Checked(operation, arg, tool, options, result, history, uncleaned, uncleanedHistory, made, proved);
    }

    /// <summary>
    /// The check of a Boolean's result after its clean: <paramref name="result"/> as cleaned and <paramref name="uncleaned"/> as
    /// made (the same shape where the clean was left out).
    /// </summary>
    internal static (Shape Result, ModelHistory History, Diagnostic? Wrong) Checked(BooleanKind operation, List<Shape> arg, List<Shape> tool,
        BooleanOptions? options, Shape result, ModelHistory history, Shape uncleaned, ModelHistory uncleanedHistory, bool made, bool proved)
    {
        // A valid shape can still be the wrong one, and its volume can prove it (decision 1860). As above, a clean that spoils
        // a good result is left out. One the kernel proved right isn't checked again unless the clean changed it
        if (made && !(proved && result.IsSame(uncleaned)) && RuleOf(operation) is { } rule
            && BooleanVolumes.Check(rule, arg, tool, result, options?.FuzzyValue ?? 0) is { } wrong)
        {
            if (result.IsSame(uncleaned) || !proved && BooleanVolumes.Check(rule, arg, tool, uncleaned, options?.FuzzyValue ?? 0) is not null)
                return (result, history, wrong);
            Warnings.Warn("Boolean operation unable to clean: that would change its result's volume");
            return (uncleaned, uncleanedHistory, null);
        }

        // The check measures against the result's own tolerance, so a clean that raises it can pass what the result as made
        // wouldn't: equal cylinders of radius 50000, 2e-8 short of crossing, whose cut removed nothing and was wrong by the
        // check, cleaned into the whole cylinder with a vertex of tolerance 236, which passed, 74% out (decision 2277). A clean
        // is only tidying and can't make a wrong result right, so where it at least doubles the tolerance the result as made is
        // what's checked; and one the kernel proved right keeps its clean only if its volume is the same
        if (made && !result.IsSame(uncleaned) && RuleOf(operation) is { } asMade && LargestTolerance(result) > 2 * LargestTolerance(uncleaned))
        {
            if (!proved)
            {
                if (BooleanVolumes.Check(asMade, arg, tool, uncleaned, options?.FuzzyValue ?? 0) is { } wrongAsMade)
                    return (uncleaned, uncleanedHistory, wrongAsMade);
            }
            else
            {
                // A volume the kernel couldn't confirm is NaN here, which keeps the result as made (decision 34806)
                double cleanedVolume = Unconfirmed(() => result.IntegrateVolume()), asMadeVolume = Unconfirmed(() => uncleaned.IntegrateVolume());
                if (!(Math.Abs(cleanedVolume - asMadeVolume) <= 1e-6 * Math.Max(Math.Abs(cleanedVolume), Math.Abs(asMadeVolume))))
                {
                    Warnings.Warn("Boolean operation unable to clean: that would change its result's volume");
                    return (uncleaned, uncleanedHistory, null);
                }
            }
        }
        return (result, history, null);
    }

    private static double LargestTolerance(Shape shape)
    {
        double largest = 0;
        foreach (var vertex in shape.AllVertices())
            largest = Math.Max(largest, vertex.Tolerance);
        return largest;
    }

    private static BooleanVolumeRule? RuleOf(BooleanKind operation) => operation switch
    {
        BooleanKind.Common => BooleanVolumeRule.Common,
        BooleanKind.Cut => BooleanVolumeRule.Cut,
        BooleanKind.Fuse => BooleanVolumeRule.Fuse,
        _ => null,
    };

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

        /// <summary>As the other forms of <c>Cut</c>. <paramref name="parallelVolumes"/> integrates the volumes its check measures on several threads (true) or one (false), the
        /// result the same either way; null follows <see cref="ParallelVolumes"/>, off unless a scope turns it on (decision 2092).</summary>
        public Shape Cut<T>(IEnumerable<T> toCut, bool? parallelVolumes = null) where T : IShape
        {
            using ParallelVolumes.Scope parallel = TopoUtil.ParallelVolumesFor(parallelVolumes);
            return BoolOp([self.AsShape()], toCut.Select(s => s.AsShape()), BooleanKind.Cut);
        }

        /// <summary>As the other forms of <c>Fuse</c>.</summary>
        public Shape Fuse<T>(IEnumerable<T> toFuse, bool glue = false, double? tolerance = null, bool? parallelVolumes = null) where T : IShape =>
            self.Fuse(toFuse.Select(s => s.AsShape()), glue, tolerance, parallelVolumes);

        /// <summary>
        /// This shape fused with <paramref name="toFuse"/>. <paramref name="glue"/> speeds up shapes that only touch;
        /// <paramref name="tolerance"/> sets a fuzzy value. The result may be of another type.
        /// <paramref name="parallelVolumes"/> integrates the volumes its check measures on several threads (true) or one (false), the
        /// result the same either way; null follows <see cref="ParallelVolumes"/>, off unless a scope turns it on (decision 2092).
        /// </summary>
        public Shape Fuse(IEnumerable<Shape> toFuse, bool glue = false, double? tolerance = null, bool? parallelVolumes = null)
        {
            using ParallelVolumes.Scope parallel = TopoUtil.ParallelVolumesFor(parallelVolumes);
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
        /// boundary without interior overlap (only where solids are involved). <paramref name="parallelVolumes"/> integrates the volumes its check measures on several threads (true) or one (false), the
        /// result the same either way; null follows <see cref="ParallelVolumes"/>, off unless a scope turns it on (decision 2092).
        /// </remarks>
        public ShapeList<Shape> Intersect(IEnumerable<CurveLimit> toIntersect, double tolerance = 1e-6, bool includeTouched = false,
                                          bool? parallelVolumes = null)
        {
            using ParallelVolumes.Scope parallel = TopoUtil.ParallelVolumesFor(parallelVolumes);
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
        public ShapeList<Shape> Intersect(IEnumerable<Shape> toIntersect, double tolerance = 1e-6, bool includeTouched = false,
                                          bool? parallelVolumes = null) =>
            self.Intersect(toIntersect.Select(s => (CurveLimit)s), tolerance, includeTouched, parallelVolumes);

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
        TopoUtil.Measuring("centre", () => shape.MassProperties(TopoUtil.Dim(shape) switch
        {
            1 => Dimension.Length,
            2 => Dimension.Area,
            _ => Dimension.Volume,
        }).CentreOfMass);

    // A measure, or NaN where the kernel couldn't confirm it (decision 34806)
    private static double Unconfirmed(Func<double> measure)
    {
        try
        {
            return measure();
        }
        catch (ArithmeticException e) when (Diagnostic.FromException(e).Code == MeasureOperations.Unconfirmed)
        {
            return double.NaN;
        }
    }
}
