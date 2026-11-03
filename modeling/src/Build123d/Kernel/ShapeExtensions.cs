using Occt;

namespace Build123d;

/// <summary>
/// What the native binding's wrapper classes gave a shape, for OCCT.NET's shape structs: its kind (the wrappers were
/// subclasses, TopoDS_Edge for an edge) and OCCT's <c>std::hash</c>.
/// </summary>
internal static class ShapeExtensions
{
    /// <summary>
    /// <c>std::hash&lt;TopoDS_Shape&gt;</c>: equal for shapes that are <c>IsSame</c> (the same TShape at the same
    /// Location), whatever their orientation.
    /// </summary>
    public static ulong StdHash(this TopoDS_Shape shape) => unchecked((uint)shape.GetHashCode());

    /// <summary>Whether the shape isn't null and is of <paramref name="type"/>.</summary>
    public static bool Is(this TopoDS_Shape shape, TopAbs_ShapeEnum type) => !shape.IsNull() && shape.ShapeType() == type;

    public static bool IsVertex(this TopoDS_Shape shape) => shape.Is(TopAbs_ShapeEnum.TopAbs_VERTEX);

    public static bool IsEdge(this TopoDS_Shape shape) => shape.Is(TopAbs_ShapeEnum.TopAbs_EDGE);

    public static bool IsWire(this TopoDS_Shape shape) => shape.Is(TopAbs_ShapeEnum.TopAbs_WIRE);

    public static bool IsFace(this TopoDS_Shape shape) => shape.Is(TopAbs_ShapeEnum.TopAbs_FACE);

    public static bool IsShell(this TopoDS_Shape shape) => shape.Is(TopAbs_ShapeEnum.TopAbs_SHELL);

    public static bool IsSolid(this TopoDS_Shape shape) => shape.Is(TopAbs_ShapeEnum.TopAbs_SOLID);

    public static bool IsCompound(this TopoDS_Shape shape) => shape.Is(TopAbs_ShapeEnum.TopAbs_COMPOUND);
}
