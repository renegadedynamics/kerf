
namespace Kerf.Modeling;

/// <summary>
/// What the native binding's wrapper classes gave a shape, for OCCT.NET's shape structs: its kind (the wrappers were
/// subclasses, TopoDS_Edge for an edge) and OCCT's <c>std::hash</c>.
/// </summary>
internal static class ShapeKinds
{
    /// <summary>
    /// <c>std::hash&lt;TopoDS_Shape&gt;</c>: equal for shapes that are <c>IsSame</c> (the same TShape at the same
    /// Location), whatever their orientation.
    /// </summary>
    public static ulong StdHash(this Shape shape) => unchecked((uint)shape.GetHashCode());

    /// <summary>Whether the shape isn't null and is of <paramref name="type"/>.</summary>
    public static bool Is(this Shape shape, ShapeType type) => !shape.IsNull && shape.Type == type;

    public static bool IsVertex(this Shape shape) => shape.Is(ShapeType.Vertex);

    public static bool IsEdge(this Shape shape) => shape.Is(ShapeType.Edge);

    public static bool IsWire(this Shape shape) => shape.Is(ShapeType.Wire);

    public static bool IsFace(this Shape shape) => shape.Is(ShapeType.Face);

    public static bool IsShell(this Shape shape) => shape.Is(ShapeType.Shell);

    public static bool IsSolid(this Shape shape) => shape.Is(ShapeType.Solid);

    /// <summary>The shapes of a type among others (LINQ's <c>OfType</c> looks at the static type, here always <see cref="Shape"/>).</summary>
    public static IEnumerable<T> OfKind<T>(this IEnumerable<Shape> shapes) where T : struct, IShape<T> =>
        shapes.Where(s => !s.IsNull && s.Type == T.Type).Select(s => s.As<T>());

    public static bool IsCompound(this Shape shape) => shape.Is(ShapeType.Compound);
}
