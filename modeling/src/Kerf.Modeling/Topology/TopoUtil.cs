using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

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
        return obj.ShapeType();
    }

    /// <summary>
    /// The name of the native binding's wrapper class for a shape (<c>TopoDS_Edge</c> for an edge), as messages name it.
    /// </summary>
    internal static string TypeName(global::Kerf.Topology.Shape obj) => obj.IsNull ? "TopoDS_Shape" : obj.ShapeType() switch
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

    /// <summary>The shape as the wrapper of its own type (TopoDS_Edge for an edge). Always a new wrapper.</summary>
    internal static global::Kerf.Topology.Shape Downcast(global::Kerf.Topology.Shape obj) => ShapeTypeOf(obj) switch
    {
        global::Kerf.Topology.ShapeType.Vertex => ShapeCasts.Vertex(obj),
        global::Kerf.Topology.ShapeType.Edge => ShapeCasts.Edge(obj),
        global::Kerf.Topology.ShapeType.Wire => ShapeCasts.Wire(obj),
        global::Kerf.Topology.ShapeType.Face => ShapeCasts.Face(obj),
        global::Kerf.Topology.ShapeType.Shell => ShapeCasts.Shell(obj),
        global::Kerf.Topology.ShapeType.Solid => ShapeCasts.Solid(obj),
        global::Kerf.Topology.ShapeType.CompSolid => ShapeCasts.CompSolid(obj),
        global::Kerf.Topology.ShapeType.Compound => ShapeCasts.Compound(obj),
        var t => throw new ArgumentException($"Unable to cast {t}"),
    };

    /// <summary>
    /// A copy of a shape that shares nothing with it but (unless <paramref name="copyGeometry"/>) its curves and surfaces,
    /// which OCCT doesn't change: for the algorithms that write into the topology they are given (adding pcurves to its
    /// edges, raising its tolerances, storing a mesh), which would otherwise change every shape sharing it.
    /// </summary>
    internal static global::Kerf.Topology.Shape PrivateCopy(global::Kerf.Topology.Shape shape, bool copyGeometry = false)
    {
        var copier = new Copy(shape, copyGeometry, false);
        return copier.Shape();
    }

    /// <summary>The shape fixed by ShapeFix_Shape, which leaves the shape it's given untouched.</summary>
    internal static global::Kerf.Topology.Shape Fix(global::Kerf.Topology.Shape obj)
    {
        // ShapeFix edits in a working overlay (the kernel's unit 10H4): the result shares what it didn't change
        var shapeFix = new ShapeFixer(obj);
        shapeFix.Perform();
        return Downcast(shapeFix.Shape());
    }

    /// <summary>A compound of the given shapes (the kernel's empty compound for none), frozen.</summary>
    internal static global::Kerf.Topology.Compound MakeCompound(IEnumerable<global::Kerf.Topology.Shape> shapes) => Builder.MakeCompound(shapes);

    internal static global::Kerf.Topology.Compound EmptyCompound() => MakeCompound([]);

    /// <summary>True for a compound with nothing in it: the zero of every shape class.</summary>
    internal static bool IsEmptyCompound(global::Kerf.Topology.Shape shape)
    {
        if (!shape.IsCompound())
            return false;
        var it = new Iterator(shape);
        return !it.More();
    }

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
                var it = new Iterator();
                it.Initialize(current);
                for (; it.More(); it.Next())
                {
                    var child = Downcast(it.Value());
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
    internal static int? Dim(global::Kerf.Topology.Shape shape) => shape.IsNull ? null : shape.ShapeType() switch
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
            var it = new Iterator(compound);
            var single = Downcast(it.Value());
            if (single.IsCompound())
                return UnwrapCompound(ShapeCasts.Compound(single), fully);
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
        var explorer = new Explorer(shape, type);
        for (; explorer.More(); explorer.Next())
        {
            var item = explorer.Current();
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
    internal static double Area(global::Kerf.Topology.Shape shape)
    {
        var props = new Properties();
        MassPropertiesBuilder.SurfaceProperties(shape, props);
        return props.Mass();
    }

    /// <summary>The faces sewn into a shell where they meet within <paramref name="tolerance"/>.</summary>
    internal static global::Kerf.Topology.Shape SewFaces(IEnumerable<global::Kerf.Topology.Shape> faces, double tolerance = Constants.Tolerance)
    {
        var builder = new Sewing(tolerance);
        foreach (var face in faces)
            builder.Add(face);
        builder.Perform();
        return Downcast(builder.SewedShape());
    }

    internal static NCollection_List<global::Kerf.Topology.Shape> ListOf(IEnumerable<global::Kerf.Topology.Shape> shapes)
    {
        var list = new NCollection_List<global::Kerf.Topology.Shape>();
        foreach (var s in shapes)
            list.Append(s);
        return list;
    }

    /// <summary>A boolean (or split) of raw shapes, with any compound around a single result removed.</summary>
    internal static global::Kerf.Topology.Shape BoolOp(IEnumerable<global::Kerf.Topology.Shape> args, IEnumerable<global::Kerf.Topology.Shape> tools, BuilderAlgo operation)
    {
        var arg = ListOf(args);
        var tool = ListOf(tools);
        operation.SetArguments(arg);
        SetTools(operation, tool);
        operation.SetRunParallel(true);
        // Non-destructive: where an input needs a larger tolerance, OCCT copies it rather than changing it
        operation.SetNonDestructive(true);
        Failures.Doing(() => BooleanName(operation), () =>
        {
            operation.Build();
            ThrowIfFailed(operation);
        });
        var result = Downcast(operation.Shape());
        return result.IsCompound() ? UnwrapCompound(ShapeCasts.Compound(result)) : result;
    }

    /// <summary>What a boolean is called in an error message.</summary>
    internal static string BooleanName(BuilderAlgo operation) => operation switch
    {
        Cut => "cut",
        Fuse => "fuse",
        Common => "intersect",
        SectionOperation => "section",
        SplitterOperation => "split",
        _ => "boolean",
    };

    /// <summary>
    /// Throws if a built boolean failed, with OCCT's reasons as diagnostics ("boolean.self-interfering-shape", say) where
    /// a bare <c>Shape()</c> would say only StdFail_NotDone.
    /// </summary>
    internal static void ThrowIfFailed(Algo operation)
    {
        if (operation.HasErrors)
            throw Failures.Boolean(operation);
    }

    internal static void SetTools(BuilderAlgo operation, NCollection_List<global::Kerf.Topology.Shape> tools)
    {
        switch (operation)
        {
            case BooleanOperation b: b.SetTools(tools); break;
            case SplitterOperation s: s.SetTools(tools); break;
            default: throw new ArgumentException($"{operation.GetType().Name} takes no tools");
        }
    }

    /// <summary>The unit normal of a face at the point of it nearest <paramref name="surfacePoint"/>.</summary>
    internal static Vector FaceNormalAt(global::Kerf.Topology.Face face, Point3D surfacePoint)
    {
        var surface = Tool.Surface(face);
        var projector = new ProjectPointOnSurface(surfacePoint, surface!);
        double u = 0, v = 0;
        projector.LowerDistanceParameters(out u, out v);
        var pnt = new Point3D();
        var normal = new Vector3D();
        var props = new FaceIntegrand(face);
        props.Normal(u, v, out pnt, out normal);
        return new Vector(normal).Normalized();
    }

    /// <summary>The shapes in a list, in order.</summary>
    internal static global::Kerf.Topology.Shape[] Items(NCollection_List<global::Kerf.Topology.Shape> list) => list.ToArray();
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

    /// <summary>The Location that carries <paramref name="source"/>'s frame onto <paramref name="target"/>'s.</summary>
    internal static Placement RelocationBetween(global::Kerf.Topology.Shape source, global::Kerf.Topology.Shape target)
    {
        var s = source.Location();
        var t = target.Location();
        var inverse = s.Inverted();
        return t.Multiplied(inverse);
    }
}
