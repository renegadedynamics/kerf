using Kerf.Booleans;
using Kerf.Healing;
using Kerf.Measurement;

namespace Kerf.Modeling;

/// <summary>Helpers over the kernel's shapes, shared by the topology classes (shape_core's module functions).</summary>
internal static partial class TopoUtil
{
    /// <summary>Whether a shape has nothing in it: a null shape, or a compound without children.</summary>
    internal static bool IsNothing(Shape shape) => shape.IsNull || IsEmptyCompound(shape);

    /// <summary>A compound of the shapes that aren't empty.</summary>
    internal static Compound MakeCompoundOf(IEnumerable<Shape> shapes) => MakeCompound(shapes.Where(s => !IsNothing(s)));

    /// <summary>The name of a shape's kind, for messages.</summary>
    internal static string KindName(Shape shape) => shape.IsNull ? "Shape" : shape.Type.ToString();

    /// <summary>A result of the type of <paramref name="original"/>: an object keeps its class and attributes, without a history.</summary>
    internal static T Rewrap<T>(T original, Shape result) where T : IShape
    {
        if (original is ModelShape model)
            return (T)(object)model.Rewrapped(result);
        if (typeof(T) == typeof(Shape))
            return (T)(object)result;
        return (T)Typed(result);
    }

    /// <summary>An edge or wire as a wire.</summary>
    internal static Wire AsWire(IShape1D curve)
    {
        var shape = curve.AsShape();
        return shape.IsWire() ? (Wire)shape : shape.IsEdge() ? Wire.Make((Edge)shape) : Wire.Make(shape.Edges());
    }

    /// <summary>The shape as its one-dimensional type.</summary>
    internal static IShape1D As1D(Shape shape) => shape.IsWire() ? (Wire)shape : (Edge)shape;

    /// <summary>The shape as its two-dimensional type.</summary>
    internal static IShape2D As2D(Shape shape) => shape.IsShell() ? (Shell)shape : (Face)shape;

    /// <summary>The shape as a solid or compound.</summary>
    internal static IShape3D As3D(Shape shape) => shape.IsSolid() ? (Solid)shape : shape.IsCompound() ? (Compound)shape : (CompSolid)shape;

    /// <summary>The shape boxed as its own type, for code that matches on it.</summary>
    internal static object Typed(Shape shape)
    {
        if (shape.IsNull)
            return shape;
        // Not a switch expression: its natural type would be Shape, and every arm would box as one
        switch (shape.Type)
        {
            case ShapeType.Vertex: return (Vertex)shape;
            case ShapeType.Edge: return (Edge)shape;
            case ShapeType.Wire: return (Wire)shape;
            case ShapeType.Face: return (Face)shape;
            case ShapeType.Shell: return (Shell)shape;
            case ShapeType.Solid: return (Solid)shape;
            case ShapeType.Compound: return (Compound)shape;
            case ShapeType.CompSolid: return (CompSolid)shape;
            default: return shape;
        }
    }

    /// <summary>The OCCT shape type of a Shapes name ("Edge" → TopAbs_EDGE).</summary>
    internal static ShapeType ToTopAbs(ShapeType type) => type switch
    {
        ShapeType.Vertex => ShapeType.Vertex,
        ShapeType.Edge => ShapeType.Edge,
        ShapeType.Wire => ShapeType.Wire,
        ShapeType.Face => ShapeType.Face,
        ShapeType.Shell => ShapeType.Shell,
        ShapeType.Solid => ShapeType.Solid,
        ShapeType.CompSolid => ShapeType.CompSolid,
        ShapeType.Compound => ShapeType.Compound,
        _ => throw new ArgumentOutOfRangeException(nameof(type)),
    };

    internal static ShapeType FromTopAbs(ShapeType type) => type switch
    {
        ShapeType.Vertex => ShapeType.Vertex,
        ShapeType.Edge => ShapeType.Edge,
        ShapeType.Wire => ShapeType.Wire,
        ShapeType.Face => ShapeType.Face,
        ShapeType.Shell => ShapeType.Shell,
        ShapeType.Solid => ShapeType.Solid,
        ShapeType.CompSolid => ShapeType.CompSolid,
        ShapeType.Compound => ShapeType.Compound,
        _ => throw new ArgumentException($"Unable to cast {type}"),
    };

    /// <summary>The shape's TopAbs type; a null shape has none.</summary>
    internal static ShapeType ShapeTypeOf(Shape obj)
    {
        if (obj.IsNull)
            throw new ArgumentException("The shape is null");
        return obj.Type;
    }

    /// <summary>What a message calls a shape: its type, or <c>null shape</c>.</summary>
    internal static string TypeName(Shape obj) => obj.IsNull ? "null shape" : obj.Type.ToString();

    /// <summary>
    /// The shape itself, checked to be one: the native binding's wrapper of its own type once (<c>TopoDS_Edge</c> for an
    /// edge), which the kernel's shape structs don't need.
    /// </summary>
    internal static Shape Downcast(Shape obj)
    {
        _ = ShapeTypeOf(obj);
        return obj;
    }

    /// <summary>The sub-shapes of a type, in exploration order, with repeats (OCCT reaches a shared sub-shape more than once).</summary>
    internal static List<Shape> Explore(Shape shape, ShapeType type,
                                                              ShapeType? avoid = null) => type switch
    {
        ShapeType.Vertex => [.. shape.Explore<Vertex>(avoid).Select(s => s.AsShape())],
        ShapeType.Edge => [.. shape.Explore<Edge>(avoid).Select(s => s.AsShape())],
        ShapeType.Wire => [.. shape.Explore<Wire>(avoid).Select(s => s.AsShape())],
        ShapeType.Face => [.. shape.Explore<Face>(avoid).Select(s => s.AsShape())],
        ShapeType.Shell => [.. shape.Explore<Shell>(avoid).Select(s => s.AsShape())],
        ShapeType.Solid => [.. shape.Explore<Solid>(avoid).Select(s => s.AsShape())],
        ShapeType.CompSolid => [.. shape.Explore<CompSolid>(avoid).Select(s => s.AsShape())],
        ShapeType.Compound => [.. shape.Explore<Compound>(avoid).Select(s => s.AsShape())],
        _ => throw new ArgumentOutOfRangeException(nameof(type)),
    };

    /// <summary>The distinct sub-shapes of a type (by <c>IsSame</c>), in the order exploration first reaches them.</summary>
    internal static List<Shape> Unique(Shape shape, ShapeType type) => type switch
    {
        ShapeType.Vertex => [.. shape.AllVertices().Select(s => s.AsShape())],
        ShapeType.Edge => [.. shape.AllEdges().Select(s => s.AsShape())],
        ShapeType.Wire => [.. shape.AllWires().Select(s => s.AsShape())],
        ShapeType.Face => [.. shape.AllFaces().Select(s => s.AsShape())],
        ShapeType.Shell => [.. shape.AllShells().Select(s => s.AsShape())],
        ShapeType.Solid => [.. shape.AllSolids().Select(s => s.AsShape())],
        _ => [.. Explore(shape, type).Distinct(ShapeComparer.Same)],
    };

    /// <summary>The shape fixed by ShapeFix_Shape, which leaves the shape it's given untouched.</summary>
    internal static Shape Fix(Shape obj) =>
        // ShapeFix edits in a working overlay (the kernel's unit 10H4): the result shares what it didn't change
        Failures.Get(obj.Fix(UncheckedFix), () => "fix").Shape;

    /// <summary>ShapeFix_Shape's defaults, without the check of the result the kernel adds (its warning isn't read here).</summary>
    internal static readonly FixOptions UncheckedFix = new() { CheckResult = false };

    /// <summary>
    /// A kernel result that failed because what it made isn't valid, with that fixed if fixing makes it valid and
    /// <paramref name="wrong"/> finds nothing wrong with the fix: the offsets, which ShapeFix can often put right, and whose
    /// size the kernel checks (decision 1918), as the fix must pass too. Any other result is left as it is.
    /// </summary>
    internal static Result<Built<Shape>> Repaired(Result<Built<Shape>> made, Func<Shape, Diagnostic?> wrong)
    {
        if (!made.TryGetInvalid(out var invalid))
            return made;
        var repaired = Fix(invalid.Shape);
        return repaired.IsValid && wrong(repaired) is null ? Result<Built<Shape>>.Success(new Built<Shape>(repaired, invalid.History)) : made;
    }

    /// <summary>A compound of the given shapes (the kernel's empty compound for none), frozen.</summary>
    internal static Compound MakeCompound(IEnumerable<Shape> shapes) =>
        Compound.Create(shapes);

    internal static Compound EmptyCompound() => MakeCompound([]);

    /// <summary>True for a compound with nothing in it: the zero of every shape class.</summary>
    internal static bool IsEmptyCompound(Shape shape) => shape.IsCompound() && shape.ChildCount == 0;

    /// <summary>
    /// The non-compound shapes at the first level of a shape: a compound's children, looking through nested
    /// compounds; any other shape is its own first level.
    /// </summary>
    internal static List<Shape> TopLevelShapes(Shape shape)
    {
        var firstLevel = new List<Shape>();
        var stack = new Stack<Shape>([shape]);
        while (stack.Count > 0)
        {
            var current = stack.Pop();
            if (current.IsCompound())
            {
                foreach (var child in current.Children)
                {
                    if (child.IsCompound())
                        stack.Push(child);
                    else
                        firstLevel.Add(child);
                }
            }
            else
                firstLevel.Add(current);
        }
        return firstLevel;
    }

    /// <summary>The dimension of a shape: 0 to 3, or null for a compound of mixed dimension.</summary>
    internal static int? Dim(Shape shape) => shape.IsNull ? null : shape.Type switch
    {
        ShapeType.Vertex => 0,
        ShapeType.Edge or ShapeType.Wire => 1,
        ShapeType.Face or ShapeType.Shell => 2,
        ShapeType.Solid => 3,
        ShapeType.Compound => TopLevelShapes(shape).Select(Dim).Distinct().ToList() is [var d] ? d : null,
        _ => null,
    };

    /// <summary>Strips compounds holding a single shape; with <paramref name="fully"/> false one compound is kept.</summary>
    internal static Shape UnwrapCompound(Compound compound, bool fully = true)
    {
        if (compound.ChildCount == 1)
        {
            var single = compound.Children.First();
            if (single.IsCompound())
                return UnwrapCompound((Compound)single, fully);
            return fully ? single : compound;
        }
        return compound;
    }

    /// <summary>The distinct sub-shapes of a type, in exploration order (OCCT reaches a shared sub-shape more than once).</summary>
    internal static List<Shape> Entities(Shape shape, ShapeType type)
    {
        // Keyed by hash, as build123d is, so orientations of one sub-shape count once; the last one found is kept
        var index = new Dictionary<ulong, int>();
        var found = new List<Shape>();
        foreach (var item in Explore(shape, type))
        {
            var hash = item.StdHash();
            if (index.TryGetValue(hash, out var i))
                found[i] = item;
            else
            {
                index[hash] = found.Count;
                found.Add(item);
            }
        }
        return found;
    }

    /// <summary>The surface area of a shape's faces.</summary>
    internal static double Area(Shape shape) => shape.IntegrateArea(Measured);

    /// <summary>
    /// How areas and volumes are integrated: adaptively, to a millionth of each face's contribution. OCCT's default of a
    /// fixed number of points can be several per cent out on swept and B-spline faces (docs/occt-defects.md). The precision
    /// is relative, so it suits a part of any size; 1e-7 is no closer on the parts measured, and up to three times the cost
    /// (decision 1802).
    /// </summary>
    internal static readonly MassOptions Measured = new() { Tolerance = 1e-6 };

    /// <summary>
    /// How centres of mass and moments of inertia are integrated. The precision applies to each face's mass, and a moment
    /// follows it less closely: on a cylinder, 4e-6 out at 1e-6 and at 1e-7, and exact from 1e-8; a symmetric part's centre
    /// 1.6e-6 off its plane of symmetry at the same two.
    /// </summary>
    internal static readonly MassOptions MeasuredMoments = new() { Tolerance = 1e-9 };

    /// <summary>The faces sewn into a shell where they meet within <paramref name="tolerance"/>.</summary>
    internal static Shape SewFaces(IEnumerable<Shape> faces, double tolerance = Constants.Tolerance) =>
        Failures.Get(HealingOperations.Sew(faces, new SewOptions { Tolerance = tolerance }), () => "sew").Shape;

    /// <summary>A boolean (or split) of raw shapes, with any compound around a single result removed.</summary>
    internal static Shape BoolOp(IEnumerable<Shape> args, IEnumerable<Shape> tools,
                                                       BooleanKind operation, BooleanOptions? options = null)
    {
        var built = Failures.Get(Boolean(operation, [.. args], [.. tools], options), () => BooleanName(operation));
        var result = Downcast(built.Shape);
        return result.IsCompound() ? UnwrapCompound((Compound)result) : result;
    }

    /// <summary>The scope of a modelling operation given a setting of <see cref="ParallelVolumes"/> of its own (decision 2092):
    /// nothing when it has none, or the one in force.</summary>
    internal static ParallelVolumes.Scope ParallelVolumesFor(bool? own) =>
        own is { } on && on != ParallelVolumes.Current ? ParallelVolumes.Use(on) : default;

    /// <summary>
    /// The kernel's Boolean of a kind, as build123d runs OCCT's: in parallel, non-destructive (where an input needs a larger
    /// tolerance, OCCT works on a copy rather than changing it), and not simplified (the modelling layer's own clean does that).
    /// </summary>
    /// <remarks>A section is OCCT's plain one unless <paramref name="options"/> say otherwise: not approximated, its edges
    /// without curves on the faces.</remarks>
    internal static Result<Built<Shape>> Boolean(BooleanKind operation, IReadOnlyList<Shape> args,
                                                                      IReadOnlyList<Shape> tools, BooleanOptions? options = null)
    {
        // BoolOp checks what comes out of the clean, once
        var o = options is null ? Unsimplified : options with { Simplify = false, CheckResult = false };
        return operation switch
        {
            BooleanKind.Fuse => BooleanOperations.Fuse(args, tools, o),
            BooleanKind.Cut => BooleanOperations.Cut(args, tools, o),
            BooleanKind.Common => BooleanOperations.Common(args, tools, o),
            BooleanKind.Section => BooleanOperations.Section(args, tools, o),
            BooleanKind.Split => BooleanOperations.Split(args, tools, o),
            _ => throw new ArgumentOutOfRangeException(nameof(operation)),
        };
    }

    private static readonly BooleanOptions Unsimplified = new() { Simplify = false, SectionCurvesOnFaces = false, CheckResult = false };

    /// <summary>What a boolean is called in an error message.</summary>
    internal static string BooleanName(BooleanKind operation) => operation switch
    {
        BooleanKind.Cut => "cut",
        BooleanKind.Fuse => "fuse",
        BooleanKind.Common => "intersect",
        BooleanKind.Section => "section",
        BooleanKind.Split => "split",
        _ => "boolean",
    };

    /// <summary>The unit normal of a face at the point of it nearest <paramref name="surfacePoint"/>.</summary>
    internal static Vector3D FaceNormalAt(Face face, Point3D surfacePoint)
    {
        face.TryGetSurface(out var surface);
        if (surface is null || !surface.TryProjectNearest(surfacePoint, out var nearest))
            throw new InvalidOperationException("The point can't be projected onto the face's surface");
        return FaceNormalVector(face, nearest.U, nearest.V).Normalized();
    }

    /// <summary>
    /// The normal of a face at (u, v), not normalised, as <c>BRepGProp_Face::Normal</c> gives it: the cross product of the
    /// surface's derivatives there, reversed for a reversed face.
    /// </summary>
    internal static Vector3D FaceNormalVector(Face face, double u, double v)
    {
        var d = face.ToSurface().Derivatives(u, v, 1);
        var normal = d.DU.Crossed(d.DV);
        if (face.Orientation == Orientation.Reversed)
            normal = normal.Reversed();
        return normal;
    }
}

/// <summary>The kinds of the kernel's Booleans the modelling layer runs.</summary>
internal enum BooleanKind
{
    Fuse,
    Cut,
    Common,
    Section,
    Split,
}

internal static partial class TopoUtil
{
    /// <summary>
    /// The candidate that is <paramref name="query"/>, allowing for a relocation: an exact IsSame match is preferred,
    /// failing that the first IsPartner one (the same TShape at another Location).
    /// </summary>
    internal static Shape? FindSame(Shape query, IEnumerable<Shape> candidates)
    {
        var list = candidates.ToList();
        int i = list.FindIndex(c => c.IsSame(query));
        if (i < 0)
            i = list.FindIndex(c => c.IsPartner(query));
        return i < 0 ? null : list[i];
    }

    // A shape with no sub-shapes, to compare placements as the kernel compares shapes' (by identity, decision 1518)
    private static readonly Compound PlacementProbe = EmptyCompound();

    /// <summary>
    /// Whether two placements are the same one, as OCCT compares shapes' locations (by identity: <c>TopLoc_Location::IsEqual</c>),
    /// where <c>==</c> compares their transformations.
    /// </summary>
    internal static bool IsSamePlacement(Placement a, Placement b) => PlacementProbe.Located(a).IsSame(PlacementProbe.Located(b));

    /// <summary>The Location that carries <paramref name="source"/>'s frame onto <paramref name="target"/>'s.</summary>
    internal static Placement RelocationBetween(Shape source, Shape target)
    {
        var s = source.Placement;
        var t = target.Placement;
        var inverse = s.Inverted();
        return t.Multiplied(inverse);
    }
}
