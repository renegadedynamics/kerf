using Occt;

namespace Build123d;

/// <summary>Helpers over raw OCCT shapes, shared by the topology classes (shape_core's module functions).</summary>
internal static partial class TopoUtil
{
    /// <summary>The OCCT shape type of a Shapes name ("Edge" → TopAbs_EDGE).</summary>
    internal static TopAbs_ShapeEnum ToTopAbs(ShapeType type) => type switch
    {
        Build123d.ShapeType.Vertex => TopAbs_ShapeEnum.TopAbs_VERTEX,
        Build123d.ShapeType.Edge => TopAbs_ShapeEnum.TopAbs_EDGE,
        Build123d.ShapeType.Wire => TopAbs_ShapeEnum.TopAbs_WIRE,
        Build123d.ShapeType.Face => TopAbs_ShapeEnum.TopAbs_FACE,
        Build123d.ShapeType.Shell => TopAbs_ShapeEnum.TopAbs_SHELL,
        Build123d.ShapeType.Solid => TopAbs_ShapeEnum.TopAbs_SOLID,
        Build123d.ShapeType.CompSolid => TopAbs_ShapeEnum.TopAbs_COMPSOLID,
        Build123d.ShapeType.Compound => TopAbs_ShapeEnum.TopAbs_COMPOUND,
        _ => throw new ArgumentOutOfRangeException(nameof(type)),
    };

    internal static ShapeType FromTopAbs(TopAbs_ShapeEnum type) => type switch
    {
        TopAbs_ShapeEnum.TopAbs_VERTEX => Build123d.ShapeType.Vertex,
        TopAbs_ShapeEnum.TopAbs_EDGE => Build123d.ShapeType.Edge,
        TopAbs_ShapeEnum.TopAbs_WIRE => Build123d.ShapeType.Wire,
        TopAbs_ShapeEnum.TopAbs_FACE => Build123d.ShapeType.Face,
        TopAbs_ShapeEnum.TopAbs_SHELL => Build123d.ShapeType.Shell,
        TopAbs_ShapeEnum.TopAbs_SOLID => Build123d.ShapeType.Solid,
        TopAbs_ShapeEnum.TopAbs_COMPSOLID => Build123d.ShapeType.CompSolid,
        TopAbs_ShapeEnum.TopAbs_COMPOUND => Build123d.ShapeType.Compound,
        _ => throw new ArgumentException($"Unable to cast {type}"),
    };

    /// <summary>The shape's TopAbs type; a null shape has none.</summary>
    internal static TopAbs_ShapeEnum ShapeTypeOf(TopoDS_Shape obj)
    {
        if (obj.IsNull())
            throw new ArgumentException("Null TopoDS_Shape object");
        return obj.ShapeType();
    }

    /// <summary>
    /// The name of the native binding's wrapper class for a shape (<c>TopoDS_Edge</c> for an edge), as messages name it.
    /// </summary>
    internal static string TypeName(TopoDS_Shape obj) => obj.IsNull() ? "TopoDS_Shape" : obj.ShapeType() switch
    {
        TopAbs_ShapeEnum.TopAbs_VERTEX => "TopoDS_Vertex",
        TopAbs_ShapeEnum.TopAbs_EDGE => "TopoDS_Edge",
        TopAbs_ShapeEnum.TopAbs_WIRE => "TopoDS_Wire",
        TopAbs_ShapeEnum.TopAbs_FACE => "TopoDS_Face",
        TopAbs_ShapeEnum.TopAbs_SHELL => "TopoDS_Shell",
        TopAbs_ShapeEnum.TopAbs_SOLID => "TopoDS_Solid",
        TopAbs_ShapeEnum.TopAbs_COMPSOLID => "TopoDS_CompSolid",
        TopAbs_ShapeEnum.TopAbs_COMPOUND => "TopoDS_Compound",
        _ => "TopoDS_Shape",
    };

    /// <summary>The shape as the wrapper of its own type (TopoDS_Edge for an edge). Always a new wrapper.</summary>
    internal static TopoDS_Shape Downcast(TopoDS_Shape obj) => ShapeTypeOf(obj) switch
    {
        TopAbs_ShapeEnum.TopAbs_VERTEX => TopoDS.Vertex(obj),
        TopAbs_ShapeEnum.TopAbs_EDGE => TopoDS.Edge(obj),
        TopAbs_ShapeEnum.TopAbs_WIRE => TopoDS.Wire(obj),
        TopAbs_ShapeEnum.TopAbs_FACE => TopoDS.Face(obj),
        TopAbs_ShapeEnum.TopAbs_SHELL => TopoDS.Shell(obj),
        TopAbs_ShapeEnum.TopAbs_SOLID => TopoDS.Solid(obj),
        TopAbs_ShapeEnum.TopAbs_COMPSOLID => TopoDS.CompSolid(obj),
        TopAbs_ShapeEnum.TopAbs_COMPOUND => TopoDS.Compound(obj),
        var t => throw new ArgumentException($"Unable to cast {t}"),
    };

    /// <summary>
    /// A copy of a shape that shares nothing with it but (unless <paramref name="copyGeometry"/>) its curves and surfaces,
    /// which OCCT doesn't change: for the algorithms that write into the topology they are given (adding pcurves to its
    /// edges, raising its tolerances, storing a mesh), which would otherwise change every shape sharing it.
    /// </summary>
    internal static TopoDS_Shape PrivateCopy(TopoDS_Shape shape, bool copyGeometry = false)
    {
        var copier = new BRepBuilderAPI_Copy(shape, copyGeometry, false);
        return copier.Shape();
    }

    /// <summary>The shape fixed by ShapeFix_Shape, which works on a private copy.</summary>
    internal static TopoDS_Shape Fix(TopoDS_Shape obj)
    {
        // ShapeFix edits the shape it's given, pcurves and parameters included, so it gets its own geometry too
        var shapeFix = new ShapeFix_Shape(PrivateCopy(obj, copyGeometry: true));
        shapeFix.Perform();
        return Downcast(shapeFix.Shape());
    }

    /// <summary>A compound of the given shapes (the kernel's empty compound for none), frozen.</summary>
    internal static TopoDS_Compound MakeCompound(IEnumerable<TopoDS_Shape> shapes) => TopoDS_Builder.MakeCompound(shapes);

    internal static TopoDS_Compound EmptyCompound() => MakeCompound([]);

    /// <summary>True for a compound with nothing in it: the zero of every shape class.</summary>
    internal static bool IsEmptyCompound(TopoDS_Shape shape)
    {
        if (!shape.IsCompound())
            return false;
        var it = new TopoDS_Iterator(shape);
        return !it.More();
    }

    /// <summary>
    /// The non-compound shapes at the first level of a shape: a compound's children, looking through nested
    /// compounds; any other shape is its own first level.
    /// </summary>
    internal static List<TopoDS_Shape> TopLevelShapes(TopoDS_Shape shape)
    {
        var firstLevel = new List<TopoDS_Shape>();
        var stack = new Stack<TopoDS_Shape>([shape]);
        while (stack.Count > 0)
        {
            var current = stack.Pop();
            if (current.IsCompound())
            {
                var it = new TopoDS_Iterator();
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
    internal static int? Dim(TopoDS_Shape shape) => shape.IsNull() ? null : shape.ShapeType() switch
    {
        TopAbs_ShapeEnum.TopAbs_VERTEX => 0,
        TopAbs_ShapeEnum.TopAbs_EDGE or TopAbs_ShapeEnum.TopAbs_WIRE => 1,
        TopAbs_ShapeEnum.TopAbs_FACE or TopAbs_ShapeEnum.TopAbs_SHELL => 2,
        TopAbs_ShapeEnum.TopAbs_SOLID => 3,
        TopAbs_ShapeEnum.TopAbs_COMPOUND => TopLevelShapes(shape).Select(Dim).Distinct().ToList() is [var d] ? d : null,
        _ => null,
    };

    /// <summary>Strips compounds holding a single shape; with <paramref name="fully"/> false one compound is kept.</summary>
    internal static TopoDS_Shape UnwrapCompound(TopoDS_Compound compound, bool fully = true)
    {
        if (compound.NbChildren() == 1)
        {
            var it = new TopoDS_Iterator(compound);
            var single = Downcast(it.Value());
            if (single.IsCompound())
                return UnwrapCompound(TopoDS.Compound(single), fully);
            return fully ? single : compound;
        }
        return compound;
    }

    /// <summary>The distinct sub-shapes of a type, in exploration order (OCCT reaches a shared sub-shape more than once).</summary>
    internal static List<TopoDS_Shape> Entities(TopoDS_Shape shape, TopAbs_ShapeEnum type)
    {
        // Keyed by hash, as build123d is, so orientations of one sub-shape count once; the last one found is kept
        var index = new Dictionary<ulong, int>();
        var found = new List<TopoDS_Shape>();
        var explorer = new TopExp_Explorer(shape, type);
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
    internal static double Area(TopoDS_Shape shape)
    {
        var props = new GProp_GProps();
        BRepGProp.SurfaceProperties(shape, props);
        return props.Mass();
    }

    /// <summary>The faces sewn into a shell where they meet within <paramref name="tolerance"/>.</summary>
    internal static TopoDS_Shape SewFaces(IEnumerable<TopoDS_Shape> faces, double tolerance = Constants.Tolerance)
    {
        var builder = new BRepBuilderAPI_Sewing(tolerance);
        foreach (var face in faces)
            builder.Add(face);
        builder.Perform();
        return Downcast(builder.SewedShape());
    }

    internal static NCollection_List<TopoDS_Shape> ListOf(IEnumerable<TopoDS_Shape> shapes)
    {
        var list = new NCollection_List<TopoDS_Shape>();
        foreach (var s in shapes)
            list.Append(s);
        return list;
    }

    /// <summary>A boolean (or split) of raw shapes, with any compound around a single result removed.</summary>
    internal static TopoDS_Shape BoolOp(IEnumerable<TopoDS_Shape> args, IEnumerable<TopoDS_Shape> tools, BRepAlgoAPI_BuilderAlgo operation)
    {
        var arg = ListOf(args);
        var tool = ListOf(tools);
        operation.SetArguments(arg);
        SetTools(operation, tool);
        operation.SetRunParallel(true);
        // Non-destructive: where an input needs a larger tolerance, OCCT copies it rather than changing it
        operation.SetNonDestructive(true);
        operation.Build();
        ThrowIfFailed(operation);
        var result = Downcast(operation.Shape());
        return result.IsCompound() ? UnwrapCompound(TopoDS.Compound(result)) : result;
    }

    /// <summary>What a boolean is called in an error message.</summary>
    internal static string BooleanName(BRepAlgoAPI_BuilderAlgo operation) => operation switch
    {
        BRepAlgoAPI_Cut => "cut",
        BRepAlgoAPI_Fuse => "fuse",
        BRepAlgoAPI_Common => "intersect",
        BRepAlgoAPI_Section => "section",
        BRepAlgoAPI_Splitter => "split",
        _ => "boolean",
    };

    /// <summary>
    /// Throws if a built boolean failed, naming OCCT's reasons ("BOPAlgo_AlertSelfInterferingShape", say) where a bare
    /// <c>Shape()</c> would say only StdFail_NotDone.
    /// </summary>
    internal static void ThrowIfFailed(BRepAlgoAPI_Algo operation)
    {
        if (operation.HasErrors())
            throw new StdFail_NotDone(Extras.AlgoErrors(operation) ?? "");
    }

    internal static void SetTools(BRepAlgoAPI_BuilderAlgo operation, NCollection_List<TopoDS_Shape> tools)
    {
        switch (operation)
        {
            case BRepAlgoAPI_BooleanOperation b: b.SetTools(tools); break;
            case BRepAlgoAPI_Splitter s: s.SetTools(tools); break;
            default: throw new ArgumentException($"{operation.GetType().Name} takes no tools");
        }
    }

    /// <summary>The unit normal of a face at the point of it nearest <paramref name="surfacePoint"/>.</summary>
    internal static Vector FaceNormalAt(TopoDS_Face face, gp_Pnt surfacePoint)
    {
        var surface = BRep_Tool.Surface(face);
        var projector = new GeomAPI_ProjectPointOnSurf(surfacePoint, surface!);
        double u = 0, v = 0;
        projector.LowerDistanceParameters(out u, out v);
        var pnt = new gp_Pnt();
        var normal = new gp_Vec();
        var props = new BRepGProp_Face(face);
        props.Normal(u, v, out pnt, out normal);
        return new Vector(normal).Normalized();
    }

    /// <summary>The shapes in a list, in order.</summary>
    internal static TopoDS_Shape[] Items(NCollection_List<TopoDS_Shape> list) => list.ToArray();
}

internal static partial class TopoUtil
{
    /// <summary>
    /// The candidate that is <paramref name="query"/>, allowing for a relocation: an exact IsSame match is preferred,
    /// failing that the first IsPartner one (the same TShape at another Location).
    /// </summary>
    internal static TopoDS_Shape? FindSame(TopoDS_Shape query, IEnumerable<TopoDS_Shape> candidates)
    {
        var list = candidates.ToList();
        int i = list.FindIndex(c => c.IsSame(query));
        if (i < 0)
            i = list.FindIndex(c => c.IsPartner(query));
        return i < 0 ? null : list[i];
    }

    /// <summary>The Location that carries <paramref name="source"/>'s frame onto <paramref name="target"/>'s.</summary>
    internal static TopLoc_Location RelocationBetween(TopoDS_Shape source, TopoDS_Shape target)
    {
        var s = source.Location();
        var t = target.Location();
        var inverse = s.Inverted();
        return t.Multiplied(inverse);
    }
}
