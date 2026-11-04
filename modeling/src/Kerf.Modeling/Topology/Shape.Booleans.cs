using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

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

public abstract partial class Shape
{
    /// <summary>Flattens boolean operands into their top-level shapes, dropping nulls.</summary>
    internal static List<Shape> Operands(IEnumerable<Shape?>? others) =>
        others is null ? [] : others.OfType<Shape>().SelectMany(o => o.GetTopLevelShapes()).ToList();

    /// <summary>
    /// This shape fused with <paramref name="other"/> (build123d's <c>+</c>). Adding nothing returns this shape;
    /// only shapes of the same dimension can be added.
    /// </summary>
    internal virtual Shape AddCore(IEnumerable<Shape?> other)
    {
        var summands = Operands(other);
        if (summands.Count == 0)
            return this;
        // An empty shape with no dimension of its own, Compound(), adds nothing to anything
        var addendDim = IsEmpty ? summands[0].Dim : Dim;
        if (addendDim is null)
            throw new ArgumentException("Dimensions of objects to add to are inconsistent");
        if (summands.Any(s => s.Dim != addendDim))
            throw new ArgumentException("Only shapes with the same dimension can be added");
        if (IsEmpty)
            return summands.Count == 1 ? summands[0] : summands[0].Fuse(summands.Skip(1));
        return Fuse(summands);
    }

    /// <summary>
    /// This shape with <paramref name="other"/> cut away (build123d's <c>-</c>). Only shapes of an equal or greater
    /// dimension can be subtracted.
    /// </summary>
    internal virtual Shape SubtractCore(IEnumerable<Shape?> other)
    {
        var subtrahends = Operands(other);
        if (subtrahends.Count == 0)
            return this;
        if (IsEmpty)   // nothing less anything is still nothing
            return ZeroOf([this, .. subtrahends]);
        var minuendDim = Dim;
        if (minuendDim is null || subtrahends.Any(s => s.Dim is null))
            throw new ArgumentException("Dimensions of objects to subtract from are inconsistent");
        var dims = subtrahends.Select(s => s.Dim!.Value).ToList();
        if (dims.Any(d => d < minuendDim))
            throw new ArgumentException(
                "Only shapes with equal or greater dimension can be subtracted: "
                + $"not {GetType().Name} ({minuendDim}D) and {other.First()!.GetType().Name} ({dims.Min()}D)");
        return Cut(subtrahends);
    }

    /// <summary>
    /// What this shape has in common with <paramref name="others"/> (build123d's <c>&amp;</c>): one shape, a composite
    /// of several, or the zero of the operands' class if nothing.
    /// </summary>
    internal virtual Shape CommonCore(IEnumerable<object> others)
    {
        var list = others.ToList();
        // The shapes that decide the class of an empty result, as for a cut
        var shapes = list.OfType<Shape>().Where(o => o.IsEmpty).ToList();
        shapes.AddRange(Operands(list.OfType<Shape>()));
        if (IsEmpty || shapes.Any(o => o.IsEmpty))
            return ZeroOf([this, .. shapes]);
        var found = Intersect(list);
        if (found.Count == 0)
            return ZeroOf([this, .. shapes]);
        var result = found.Count == 1 ? found[0] : MakeComposite(found);
        return SkipClean.Clean ? result.CleanCore() : result;
    }

    /// <summary>This shape with <paramref name="toCut"/> removed. The result may be of another class.</summary>
    public Shape Cut(params IEnumerable<Shape> toCut) => BoolOp([this], toCut, BooleanKind.Cut);

    /// <summary>
    /// This shape fused with <paramref name="toFuse"/>. <paramref name="glue"/> speeds up shapes that only touch;
    /// <paramref name="tolerance"/> sets a fuzzy value. The result may be of another class.
    /// </summary>
    public Shape Fuse(IEnumerable<Shape> toFuse, bool glue = false, double? tolerance = null)
    {
        var options = new BooleanOptions
        {
            Glue = glue ? GlueMode.Shift : GlueMode.Off,
            FuzzyValue = tolerance is { } tol && tol != 0 ? tol : 0,
        };
        return BoolOp([this], toFuse, BooleanKind.Fuse, options);
    }

    /// <inheritdoc cref="Fuse(IEnumerable{Shape}, bool, double?)"/>
    public Shape Fuse(params Shape[] toFuse) => Fuse((IEnumerable<Shape>)toFuse);

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

    private protected Shape BoolOp(IEnumerable<Shape> args, IEnumerable<Shape> tools, BooleanKind operation, BooleanOptions? options = null)
    {
        var argList = args.ToList();
        var toolList = tools.ToList();
        // The highest-ranked class of the inputs; among equals, the last to appear (as build123d's stable sort has it)
        var classes = new List<Shape>();
        foreach (var s in argList.Prepend(this).Concat(toolList))
            if (!classes.Any(c => c.GetType() == s.GetType()))
                classes.Add(s);
        var highest = classes.Aggregate((best, c) => c.Order >= best.Order ? c : best);

        var baseShape = argList[0];
        var arg = argList.Where(o => !o.IsEmpty).Select(o => o._wrapped).ToList();
        var tool = toolList.Where(o => !o.IsEmpty).Select(o => o._wrapped).ToList();

        // Operations with zero shapes
        global::Kerf.Topology.Shape? topoResult = null;
        switch (operation)
        {
            case BooleanKind.Cut:
                if (arg.Count == 0)   // nothing less anything is still nothing
                    return ZeroOf([this, .. Operands(argList), .. Operands(toolList)]);
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
                    return ZeroOf([this, .. Operands(argList), .. Operands(toolList)]);
                break;
        }

        // The arguments were there before and the tools are brought in. An empty record means every input
        // sub-shape came through untouched, which is what the shortcuts above produce
        var history = new ShapeHistory(arg, tool);
        if (topoResult is not { } result)
        {
            // Non-destructive: where an input needs a larger tolerance, OCCT copies it rather than changing it
            var built = Failures.Get(TopoUtil.Boolean(operation, arg, tool, options),
                                     () => $"{TopoUtil.BooleanName(operation)} of {Names(argList)} with {Names(toolList)}");
            result = TopoUtil.Downcast(built.Shape);
            history = ShapeHistory.FromHistory(built.History, arg, tool);
        }

        if (SkipClean.Clean)
        {
            if (result.Unify(CleanOptions).TryGetValue(out var unified))
            {
                result = TopoUtil.Downcast(unified.Shape);
                history = history.Merge(ShapeHistory.FromHistory(unified.History));
            }
            else
                Warnings.Warn("Boolean operation unable to clean");
        }

        // Remove an unnecessary compound around a single shape
        if (result.IsCompound())
            result = TopoUtil.UnwrapCompound((global::Kerf.Topology.Compound)result);

        if (result.IsCompound() && highest.Order != 4)
        {
            var results = TopoUtil.TopLevelShapes(result)
                .Select(s => baseShape.CopyAttributesTo(highest.CastLike(s)).MadeBy(history)).ToList();
            var composite = baseShape.CopyAttributesTo(MakeComposite(results, highest.Order));
            return composite.MadeBy(history);
        }
        return baseShape.CopyAttributesTo(highest.CastLike(result)).MadeBy(history);
    }

    /// <summary>A boolean operation whose pieces are returned as a list, empty for nothing.</summary>
    private protected ShapeList<Shape> BoolOpList(IEnumerable<Shape> args, IEnumerable<Shape> tools, BooleanKind operation)
    {
        var result = BoolOp(args, tools, operation);
        if (result.IsEmpty)
            return [];
        if (result._wrapped.IsCompound())
            // The same operation made them all
            return new ShapeList<Shape>(result.GetTopLevelShapes().Select(p => p.MadeBy(result.History)));
        return [result];
    }

    /// <summary>
    /// Where this shape's body meets (overlaps or crosses) <paramref name="toIntersect"/>: shapes or geometry objects
    /// (<see cref="Vector"/>, <see cref="Location"/>, <see cref="Axis"/>, <see cref="Plane"/>).
    /// </summary>
    /// <remarks>
    /// Several arguments are chained: <c>c.Intersect(s1, s2)</c> is c ∩ s1 ∩ s2. A compound argument distributes:
    /// <c>c.Intersect(Compound([s1, s2]))</c> is (c ∩ s1) ∪ (c ∩ s2). <paramref name="includeTouched"/> adds contacts at the
    /// boundary without interior overlap (only where solids are involved).
    /// </remarks>
    public ShapeList<Shape> Intersect(IEnumerable<object> toIntersect, double tolerance = 1e-6, bool includeTouched = false)
    {
        var list = toIntersect.ToList();
        if (list.Count == 0 || IsEmpty)
            return [];
        if (list.Any(o => o is Shape { IsEmpty: true }))
            return [];
        foreach (var obj in list)
            if (obj is not (Shape or Vector or Kerf.Modeling.Location or Axis or Plane))
                throw new ArgumentException($"Unsupported type for intersect: {obj.GetType().Name}");

        var commonSet = new List<Shape> { this };
        foreach (var other in list)
        {
            var nextSet = new List<Shape>();
            foreach (var obj in commonSet)
                if (obj.IntersectCore(other, tolerance, includeTouched) is { Count: > 0 } result)
                    nextSet.AddRange(result.Expand());
            if (nextSet.Count == 0)
                return [];   // chained: if any step finds nothing, there is nothing
            commonSet = nextSet.Distinct().ToList();
        }
        return new ShapeList<Shape>(commonSet);
    }

    /// <inheritdoc cref="Intersect(IEnumerable{object}, double, bool)"/>
    public ShapeList<Shape> Intersect(params object[] toIntersect) => Intersect((IEnumerable<object>)toIntersect);

    /// <summary>
    /// The intersection of this shape with one shape or geometry object, or null if there is none. Overridden by each
    /// kind of shape.
    /// </summary>
    internal virtual ShapeList<Shape>? IntersectCore(object other, double tolerance, bool includeTouched) => null;

    /// <summary>Where this shape touches <paramref name="other"/> at its boundary. Overridden for faces, solids and compounds.</summary>
    public virtual ShapeList<Shape> Touch(Shape other, double tolerance = 1e-6) => [];

    /// <summary>
    /// The vertices and edges where this shape and <paramref name="other"/> cross (<c>BRepAlgoAPI_Section</c>).
    /// </summary>
    internal (ShapeList<Vertex> Vertices, ShapeList<Edge> Edges) OcpSection(Shape other)
    {
        if (IsEmpty || other.IsEmpty)
            return ([], []);
        // Non-destructive, so the inputs' tolerances are left alone; approximated, with the edges' curves on the faces
        var result = Failures.Get(TopoUtil.Boolean(BooleanKind.Section, [_wrapped], [other._wrapped], ApproximatedSection),
                                  () => "section").Shape;

        var vertices = new List<Vertex>();
        foreach (var vertex in TopoUtil.Explore(result, global::Kerf.Topology.ShapeType.Vertex))
            vertices.Add((Vertex)CastLike(vertex));
        var edges = new List<Edge>();
        foreach (var edge in TopoUtil.Explore(result, global::Kerf.Topology.ShapeType.Edge))
            edges.Add((Edge)CastLike(edge));
        return (new ShapeList<Vertex>(vertices.Distinct()), new ShapeList<Edge>(edges));
    }

    /// <summary>Splits this shape with a plane, keeping the pieces <paramref name="keep"/> says (not Both: see <see cref="SplitBoth(Plane)"/>).</summary>
    public ShapeList<Shape> Split(Plane tool, Keep keep = Keep.Top) => SplitCore(tool, keep).Kept;

    /// <summary>Splits this shape with a face or shell, keeping the pieces <paramref name="keep"/> says.</summary>
    public ShapeList<Shape> Split(Shape tool, Keep keep = Keep.Top) => SplitCore(tool, keep).Kept;

    /// <summary>Splits this shape, returning the pieces on top of the tool and those beneath it.</summary>
    public (ShapeList<Shape> Top, ShapeList<Shape> Bottom) SplitBoth(Plane tool)
    {
        var (_, top, bottom) = SplitCore(tool, Keep.Both);
        return (top, bottom);
    }

    /// <inheritdoc cref="SplitBoth(Plane)"/>
    public (ShapeList<Shape> Top, ShapeList<Shape> Bottom) SplitBoth(Shape tool)
    {
        var (_, top, bottom) = SplitCore(tool, Keep.Both);
        return (top, bottom);
    }

    /// <summary>
    /// The split: the pieces kept (all of them for <see cref="Keep.All"/>), and those on top of and beneath the tool.
    /// build123d returns one shape, a list or the zero of the class; here a side with nothing on it is an empty list.
    /// </summary>
    private (ShapeList<Shape> Kept, ShapeList<Shape> Top, ShapeList<Shape> Bottom) SplitCore(object tool, Keep keep)
    {
        if (tool is Shape { IsEmpty: true })
            throw new ArgumentException("Can't split with an empty tool");
        if (keep is Keep.Inside or Keep.Outside)
            throw new ArgumentException($"{keep} is invalid");
        if (IsEmpty)   // nothing to split; nothing on either side
            return ([], [], []);

        var trimTool = tool switch
        {
            Plane p => MakePlaneFace(p),
            Shape s => s.Wrapped,
            _ => throw new ArgumentException($"Can't split with a {tool.GetType().Name}"),
        };
        var splitResult = TopoUtil.BoolOp([_wrapped], [trimTool], BooleanKind.Split);
        var pieces = TopoUtil.TopLevelShapes(splitResult);

        if (keep == Keep.All)
            return (new ShapeList<Shape>(pieces.Select(CastLike)), [], []);

        global::Kerf.Topology.Shape? toolSolid = null;
        if (tool is not Plane)
        {
            // A reference point off the +ve side of the tool decides which side is up
            global::Kerf.Topology.Face toolFace = trimTool.IsShell()
                ? trimTool.Explore<global::Kerf.Topology.Face>().First()
                : (global::Kerf.Topology.Face)trimTool;
            var (us, vs) = toolFace.UVBounds;
            double u = (us.Min + us.Max) / 2, v = (vs.Min + vs.Max) / 2;
            var surfacePnt = toolFace.ToSurface().Evaluate(u, v);
            var surfaceNormal = TopoUtil.FaceNormalVector(toolFace, u, v);
            var refPoint = new Vector(surfacePnt) + surfaceNormal.Normalized();
            var refPnt = refPoint.ToGpPnt();
            var halfSpace = trimTool.IsShell()
                ? Primitives.HalfSpace((global::Kerf.Topology.Shell)trimTool, refPnt)
                : Primitives.HalfSpace((global::Kerf.Topology.Face)trimTool, refPnt);
            toolSolid = Failures.Get(halfSpace, () => "half-space").Shape;
        }

        var tops = new List<Shape>();
        var bottoms = new List<Shape>();
        foreach (var part in pieces)
        {
            var subShape = CastLike(part);
            bool isUp;
            if (tool is Plane plane)
            {
                // A piece lies wholly on one side of the plane and so does its centre of mass. Its geometric centre
                // need not: that of a curved face is the middle of its surface, trimmed away or not.
                isUp = plane.ToLocalCoords(new Vector(CentreOfMass(part))).Z >= 0;
            }
            else
            {
                // Intersect the piece and the thickened tool
                var isUpObj = TopoUtil.BoolOp([part], [toolSolid!.Value], BooleanKind.Common);
                // Mass is the total length for linear properties
                isUp = isUpObj.Length() >= Constants.Tolerance;
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

    /// <summary>A face on the plane, unbounded as far as OCCT's defaults go.</summary>
    internal static global::Kerf.Topology.Face MakePlaneFace(Plane plane)
    {
        var pln = plane.ToGpPln();
        return Failures.Get(global::Kerf.Topology.Face.FromPlane(pln), () => "plane face");
    }

    // OCCT's BRepAlgoAPI_Section as build123d sets it up for a section between two shapes
    private static readonly BooleanOptions ApproximatedSection = new() { ApproximateSection = true, SectionCurvesOnFaces = true };

    /// <summary>The clean after a Boolean: UnifySameDomain on edges and faces, B-splines concatenated, no internal edges.</summary>
    internal static readonly UnifyOptions CleanOptions = new() { Edges = true, Faces = true, ConcatBSplines = true, AllowInternalEdges = false };

    /// <summary>The centre of mass of a shape by its dimension: its length, area or volume.</summary>
    internal static Point3D CentreOfMass(global::Kerf.Topology.Shape shape) =>
        shape.MassProperties(TopoUtil.Dim(shape) switch
        {
            1 => Dimension.Length,
            2 => Dimension.Area,
            _ => Dimension.Volume,
        }).CentreOfMass;
}
