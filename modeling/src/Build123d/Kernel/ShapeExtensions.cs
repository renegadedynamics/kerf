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
    public static ulong StdHash(this global::Kerf.Topology.Shape shape) => unchecked((uint)shape.GetHashCode());

    /// <summary>Whether the shape isn't null and is of <paramref name="type"/>.</summary>
    public static bool Is(this global::Kerf.Topology.Shape shape, global::Kerf.Topology.ShapeType type) => !shape.IsNull && shape.ShapeType() == type;

    public static bool IsVertex(this global::Kerf.Topology.Shape shape) => shape.Is(global::Kerf.Topology.ShapeType.Vertex);

    public static bool IsEdge(this global::Kerf.Topology.Shape shape) => shape.Is(global::Kerf.Topology.ShapeType.Edge);

    public static bool IsWire(this global::Kerf.Topology.Shape shape) => shape.Is(global::Kerf.Topology.ShapeType.Wire);

    public static bool IsFace(this global::Kerf.Topology.Shape shape) => shape.Is(global::Kerf.Topology.ShapeType.Face);

    public static bool IsShell(this global::Kerf.Topology.Shape shape) => shape.Is(global::Kerf.Topology.ShapeType.Shell);

    public static bool IsSolid(this global::Kerf.Topology.Shape shape) => shape.Is(global::Kerf.Topology.ShapeType.Solid);

    public static bool IsCompound(this global::Kerf.Topology.Shape shape) => shape.Is(global::Kerf.Topology.ShapeType.Compound);
}
