using Kerf.Booleans;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Modeling;

/// <summary>Helpers over raw OCCT shapes, shared by the topology classes (shape_core's module functions).</summary>
internal static partial class TopoUtil
{
    /// <summary>The OCCT shape type of a Shapes name ("Edge" → TopAbs_EDGE).</summary>
    internal static global::Kerf.Topology.ShapeType ToTopAbs(ShapeType type) => type switch
    {
        Kerf.Modeling.ShapeType.Vertex => global::Kerf.Topology.ShapeType.Vertex,
        Kerf.Modeling.ShapeType.Edge => global::Kerf.Topology.ShapeType.Edge,
        Kerf.Modeling.ShapeType.Wire => global::Kerf.Topology.ShapeType.Wire,
        Kerf.Modeling.ShapeType.Face => global::Kerf.Topology.ShapeType.Face,
        Kerf.Modeling.ShapeType.Shell => global::Kerf.Topology.ShapeType.Shell,
        Kerf.Modeling.ShapeType.Solid => global::Kerf.Topology.ShapeType.Solid,
        Kerf.Modeling.ShapeType.CompSolid => global::Kerf.Topology.ShapeType.CompSolid,
        Kerf.Modeling.ShapeType.Compound => global::Kerf.Topology.ShapeType.Compound,
        _ => throw new ArgumentOutOfRangeException(nameof(type)),
    };

    internal static ShapeType FromTopAbs(global::Kerf.Topology.ShapeType type) => type switch
    {
        global::Kerf.Topology.ShapeType.Vertex => Kerf.Modeling.ShapeType.Vertex,
        global::Kerf.Topology.ShapeType.Edge => Kerf.Modeling.ShapeType.Edge,
        global::Kerf.Topology.ShapeType.Wire => Kerf.Modeling.ShapeType.Wire,
        global::Kerf.Topology.ShapeType.Face => Kerf.Modeling.ShapeType.Face,
        global::Kerf.Topology.ShapeType.Shell => Kerf.Modeling.ShapeType.Shell,
        global::Kerf.Topology.ShapeType.Solid => Kerf.Modeling.ShapeType.Solid,
        global::Kerf.Topology.ShapeType.CompSolid => Kerf.Modeling.ShapeType.CompSolid,
        global::Kerf.Topology.ShapeType.Compound => Kerf.Modeling.ShapeType.Compound,
        _ => throw new ArgumentException($"Unable to cast {type}"),
    };

    /// <summary>The shape's TopAbs type; a null shape has none.</summary>
    internal static global::Kerf.Topology.ShapeType ShapeTypeOf(global::Kerf.Topology.Shape obj)
    {
        if (obj.IsNull)
            throw new ArgumentException("Null TopoDS_Shape object");
        return obj.Type;
    }

    /// <summary>
    /// The name of the native binding's wrapper class for a shape (<c>TopoDS_Edge</c> for an edge), as messages name it.
    /// </summary>
    internal static string TypeName(global::Kerf.Topology.Shape obj) => obj.IsNull ? "TopoDS_Shape" : obj.Type switch
    {
        global::Kerf.Topology.ShapeType.Vertex => "TopoDS_Vertex",
        global::Kerf.Topology.ShapeType.Edge => "TopoDS_Edge",
        global::Kerf.Topology.ShapeType.Wire => "TopoDS_Wire",
        global::Kerf.Topology.ShapeType.Face => "TopoDS_Face",
        global::Kerf.Topology.ShapeType.Shell => "TopoDS_Shell",
        global::Kerf.Topology.ShapeType.Solid => "TopoDS_Solid",
        global::Kerf.Topology.ShapeType.CompSolid => "TopoDS_CompSolid",
        global::Kerf.Topology.ShapeType.Compound => "TopoDS_Compound",
        _ => "TopoDS_Shape",
    };

    /// <summary>
    /// The shape itself, checked to be one: the native binding's wrapper of its own type once (<c>TopoDS_Edge</c> for an
    /// edge), which the kernel's shape structs don't need.
    /// </summary>
    internal static global::Kerf.Topology.Shape Downcast(global::Kerf.Topology.Shape obj)
    {
        _ = ShapeTypeOf(obj);
        return obj;
    }

    /// <summary>The sub-shapes of a type, in exploration order, with repeats (OCCT reaches a shared sub-shape more than once).</summary>
    internal static List<global::Kerf.Topology.Shape> Explore(global::Kerf.Topology.Shape shape, global::Kerf.Topology.ShapeType type,
                                                              global::Kerf.Topology.ShapeType? avoid = null) => type switch
    {
        global::Kerf.Topology.ShapeType.Vertex => [.. shape.Explore<global::Kerf.Topology.Vertex>(avoid).Select(s => s.AsShape())],
        global::Kerf.Topology.ShapeType.Edge => [.. shape.Explore<global::Kerf.Topology.Edge>(avoid).Select(s => s.AsShape())],
        global::Kerf.Topology.ShapeType.Wire => [.. shape.Explore<global::Kerf.Topology.Wire>(avoid).Select(s => s.AsShape())],
        global::Kerf.Topology.ShapeType.Face => [.. shape.Explore<global::Kerf.Topology.Face>(avoid).Select(s => s.AsShape())],
        global::Kerf.Topology.ShapeType.Shell => [.. shape.Explore<global::Kerf.Topology.Shell>(avoid).Select(s => s.AsShape())],
        global::Kerf.Topology.ShapeType.Solid => [.. shape.Explore<global::Kerf.Topology.Solid>(avoid).Select(s => s.AsShape())],
        global::Kerf.Topology.ShapeType.CompSolid => [.. shape.Explore<global::Kerf.Topology.CompSolid>(avoid).Select(s => s.AsShape())],
        global::Kerf.Topology.ShapeType.Compound => [.. shape.Explore<global::Kerf.Topology.Compound>(avoid).Select(s => s.AsShape())],
        _ => throw new ArgumentOutOfRangeException(nameof(type)),
    };

    /// <summary>The distinct sub-shapes of a type (by <c>IsSame</c>), in the order exploration first reaches them.</summary>
    internal static List<global::Kerf.Topology.Shape> Unique(global::Kerf.Topology.Shape shape, global::Kerf.Topology.ShapeType type) => type switch
    {
        global::Kerf.Topology.ShapeType.Vertex => [.. shape.Vertices().Select(s => s.AsShape())],
        global::Kerf.Topology.ShapeType.Edge => [.. shape.Edges().Select(s => s.AsShape())],
        global::Kerf.Topology.ShapeType.Wire => [.. shape.Wires().Select(s => s.AsShape())],
        global::Kerf.Topology.ShapeType.Face => [.. shape.Faces().Select(s => s.AsShape())],
        global::Kerf.Topology.ShapeType.Shell => [.. shape.Shells().Select(s => s.AsShape())],
        global::Kerf.Topology.ShapeType.Solid => [.. shape.Solids().Select(s => s.AsShape())],
        _ => [.. Explore(shape, type).Distinct(ShapeComparer.Same)],
    };

    /// <summary>The shape fixed by ShapeFix_Shape, which leaves the shape it's given untouched.</summary>
    internal static global::Kerf.Topology.Shape Fix(global::Kerf.Topology.Shape obj) =>
        // ShapeFix edits in a working overlay (the kernel's unit 10H4): the result shares what it didn't change
        Failures.Get(obj.Fix(UncheckedFix), () => "fix").Shape;

    /// <summary>ShapeFix_Shape's defaults, without the check of the result the kernel adds (its warning isn't read here).</summary>
    internal static readonly FixOptions UncheckedFix = new() { CheckResult = false };

    /// <summary>A compound of the given shapes (the kernel's empty compound for none), frozen.</summary>
    internal static global::Kerf.Topology.Compound MakeCompound(IEnumerable<global::Kerf.Topology.Shape> shapes) =>
        global::Kerf.Topology.Compound.Create(shapes);

    internal static global::Kerf.Topology.Compound EmptyCompound() => MakeCompound([]);

    /// <summary>True for a compound with nothing in it: the zero of every shape class.</summary>
    internal static bool IsEmptyCompound(global::Kerf.Topology.Shape shape) => shape.IsCompound() && shape.ChildCount == 0;

    /// <summary>
    /// The non-compound shapes at the first level of a shape: a compound's children, looking through nested
    /// compounds; any other shape is its own first level.
    /// </summary>
    internal static List<global::Kerf.Topology.Shape> TopLevelShapes(global::Kerf.Topology.Shape shape)
    {
        var firstLevel = new List<global::Kerf.Topology.Shape>();
        var stack = new Stack<global::Kerf.Topology.Shape>([shape]);
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
    internal static int? Dim(global::Kerf.Topology.Shape shape) => shape.IsNull ? null : shape.Type switch
    {
        global::Kerf.Topology.ShapeType.Vertex => 0,
        global::Kerf.Topology.ShapeType.Edge or global::Kerf.Topology.ShapeType.Wire => 1,
        global::Kerf.Topology.ShapeType.Face or global::Kerf.Topology.ShapeType.Shell => 2,
        global::Kerf.Topology.ShapeType.Solid => 3,
        global::Kerf.Topology.ShapeType.Compound => TopLevelShapes(shape).Select(Dim).Distinct().ToList() is [var d] ? d : null,
        _ => null,
    };

    /// <summary>Strips compounds holding a single shape; with <paramref name="fully"/> false one compound is kept.</summary>
    internal static global::Kerf.Topology.Shape UnwrapCompound(global::Kerf.Topology.Compound compound, bool fully = true)
    {
        if (compound.ChildCount == 1)
        {
            var single = compound.Children.First();
            if (single.IsCompound())
                return UnwrapCompound((global::Kerf.Topology.Compound)single, fully);
            return fully ? single : compound;
        }
        return compound;
    }

    /// <summary>The distinct sub-shapes of a type, in exploration order (OCCT reaches a shared sub-shape more than once).</summary>
    internal static List<global::Kerf.Topology.Shape> Entities(global::Kerf.Topology.Shape shape, global::Kerf.Topology.ShapeType type)
    {
        // Keyed by hash, as build123d is, so orientations of one sub-shape count once; the last one found is kept
        var index = new Dictionary<ulong, int>();
        var found = new List<global::Kerf.Topology.Shape>();
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
    internal static double Area(global::Kerf.Topology.Shape shape) => shape.Area();

    /// <summary>The faces sewn into a shell where they meet within <paramref name="tolerance"/>.</summary>
    internal static global::Kerf.Topology.Shape SewFaces(IEnumerable<global::Kerf.Topology.Shape> faces, double tolerance = Constants.Tolerance) =>
        Failures.Get(HealingOperations.Sew(faces, new SewOptions { Tolerance = tolerance }), () => "sew").Shape;

    /// <summary>A boolean (or split) of raw shapes, with any compound around a single result removed.</summary>
    internal static global::Kerf.Topology.Shape BoolOp(IEnumerable<global::Kerf.Topology.Shape> args, IEnumerable<global::Kerf.Topology.Shape> tools,
                                                       BooleanKind operation, BooleanOptions? options = null)
    {
        var built = Failures.Get(Boolean(operation, [.. args], [.. tools], options), () => BooleanName(operation));
        var result = Downcast(built.Shape);
        return result.IsCompound() ? UnwrapCompound((global::Kerf.Topology.Compound)result) : result;
    }

    /// <summary>
    /// The kernel's Boolean of a kind, as build123d runs OCCT's: in parallel, non-destructive (where an input needs a larger
    /// tolerance, OCCT works on a copy rather than changing it), and not simplified (the modelling layer's own clean does that).
    /// </summary>
    /// <remarks>A section is OCCT's plain one unless <paramref name="options"/> say otherwise: not approximated, its edges
    /// without curves on the faces.</remarks>
    internal static Result<Built<global::Kerf.Topology.Shape>> Boolean(BooleanKind operation, IReadOnlyList<global::Kerf.Topology.Shape> args,
                                                                      IReadOnlyList<global::Kerf.Topology.Shape> tools, BooleanOptions? options = null)
    {
        var o = options is null ? Unsimplified : options with { Simplify = false };
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

    private static readonly BooleanOptions Unsimplified = new() { Simplify = false, SectionCurvesOnFaces = false };

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
    internal static Vector FaceNormalAt(global::Kerf.Topology.Face face, Point3D surfacePoint)
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
    internal static Vector FaceNormalVector(global::Kerf.Topology.Face face, double u, double v)
    {
        var d = face.ToSurface().Derivatives(u, v, 1);
        var normal = d.DU.Crossed(d.DV);
        if (face.Orientation == Orientation.Reversed)
            normal = normal.Reversed();
        return new Vector(normal);
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
    internal static global::Kerf.Topology.Shape? FindSame(global::Kerf.Topology.Shape query, IEnumerable<global::Kerf.Topology.Shape> candidates)
    {
        var list = candidates.ToList();
        int i = list.FindIndex(c => c.IsSame(query));
        if (i < 0)
            i = list.FindIndex(c => c.IsPartner(query));
        return i < 0 ? null : list[i];
    }

    // A shape with no sub-shapes, to compare placements as the kernel compares shapes' (by identity, decision 1518)
    private static readonly global::Kerf.Topology.Compound PlacementProbe = EmptyCompound();

    /// <summary>
    /// Whether two placements are the same one, as OCCT compares shapes' locations (by identity: <c>TopLoc_Location::IsEqual</c>),
    /// where <c>==</c> compares their transformations.
    /// </summary>
    internal static bool IsSamePlacement(Placement a, Placement b) => PlacementProbe.Located(a).IsSame(PlacementProbe.Located(b));

    /// <summary>The Location that carries <paramref name="source"/>'s frame onto <paramref name="target"/>'s.</summary>
    internal static Placement RelocationBetween(global::Kerf.Topology.Shape source, global::Kerf.Topology.Shape target)
    {
        var s = source.Placement;
        var t = target.Placement;
        var inverse = s.Inverted();
        return t.Multiplied(inverse);
    }
}
