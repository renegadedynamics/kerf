using Build123d.Occt;

namespace Build123d;

/// <summary>
/// The tolerances to triangulate with: the largest distance between the mesh and the surface (<see cref="Linear"/>,
/// in the model's units, or relative to each edge's size with <see cref="Relative"/>) and the largest angle between
/// neighbouring facets (<see cref="Angular"/>, in radians).
/// </summary>
public readonly record struct MeshTolerances(double Linear, double Angular, bool Relative = false)
{
    /// <summary>
    /// Tolerances in proportion to a shape: a linear deflection of <paramref name="deviation"/> percent of the
    /// bounding box's diagonal, so a shape of any size gets the same apparent quality.
    /// </summary>
    /// <param name="shape">The shape to be triangulated.</param>
    /// <param name="deviation">The deflection, as a percentage of the diagonal.</param>
    /// <param name="angular">The angular deflection, in radians.</param>
    public static MeshTolerances For(Shape shape, double deviation = 0.1, double angular = 0.2)
    {
        var diagonal = shape.IsEmpty ? 0 : shape.BoundingBox().Diagonal;
        // A point or an empty shape has no size: any small value does
        return new MeshTolerances(diagonal > Constants.Tolerance ? diagonal * deviation / 100 : 1e-3, angular);
    }
}

/// <summary>
/// A shape's triangulation, for display and picking: its faces' meshes, its edges' polylines and its vertices, each in
/// the order of the shape's <see cref="Shape.Faces"/>, <see cref="Shape.Edges"/> and <see cref="Shape.Vertices"/>, so
/// an index picked in a viewer is the index of the face, edge or vertex. Coordinates are global (locations applied).
/// </summary>
/// <remarks>The buffers are shared, read-only slices of a few large arrays; the mesh never changes.</remarks>
public sealed class ShapeMesh
{
    internal ShapeMesh(IReadOnlyList<FaceMesh> faces, IReadOnlyList<EdgeMesh> edges, IReadOnlyList<Vector> vertices, MeshTolerances tolerances)
        => (Faces, Edges, Vertices, Tolerances) = (faces, edges, vertices, tolerances);

    /// <summary>The mesh of an empty shape: no faces, edges or vertices.</summary>
    public static ShapeMesh Empty { get; } = new([], [], [], default);

    /// <summary>The faces' meshes, in the order of <see cref="Shape.Faces"/>.</summary>
    public IReadOnlyList<FaceMesh> Faces { get; }

    /// <summary>The edges' polylines, in the order of <see cref="Shape.Edges"/>: each topological edge once.</summary>
    public IReadOnlyList<EdgeMesh> Edges { get; }

    /// <summary>The vertices' positions, in the order of <see cref="Shape.Vertices"/>.</summary>
    public IReadOnlyList<Vector> Vertices { get; }

    /// <summary>The tolerances the mesh was made with.</summary>
    public MeshTolerances Tolerances { get; }

    /// <summary>The number of triangles in all the faces.</summary>
    public int TriangleCount => Faces.Sum(f => f.TriangleCount);

    /// <summary>The number of nodes in all the faces.</summary>
    public int NodeCount => Faces.Sum(f => f.NodeCount);

    /// <summary>
    /// Triangulates a shape. The shape isn't changed: a private copy is meshed (OCCT stores a mesh in the faces it
    /// meshes), so meshing is safe from any thread, on shapes that share geometry too.
    /// </summary>
    internal static ShapeMesh Of(Shape shape, MeshTolerances tolerances)
    {
        if (shape.IsEmpty)
            return Empty;
        var meshed = shape.Rewrapped(shape.MeshedCopy(tolerances.Linear, tolerances.Angular, tolerances.Relative));
        // The copy has the shape's structure, so its faces, edges and vertices come in the same order
        var faces = meshed.Faces();
        var edges = meshed.Edges();
        var deflection = tolerances.Relative ? MeshTolerances.For(shape).Linear : tolerances.Linear;
        var buffers = Extras.ExtractMesh(meshed.TopoShape, [.. faces.Select(f => f.TopoShape)], [.. edges.Select(e => e.TopoShape)],
                                         deflection, tolerances.Angular);

        var faceMeshes = new FaceMesh[faces.Count];
        for (int i = 0; i < faceMeshes.Length; i++)
        {
            int node = (int)buffers.FaceNodes[i], nodes = (int)buffers.FaceNodes[i + 1] - node;
            int tri = (int)buffers.FaceTriangles[i], tris = (int)buffers.FaceTriangles[i + 1] - tri;
            faceMeshes[i] = new FaceMesh(new ReadOnlyMemory<float>(buffers.Positions, 3 * node, 3 * nodes),
                                         new ReadOnlyMemory<float>(buffers.Normals, 3 * node, 3 * nodes),
                                         new ReadOnlyMemory<int>(buffers.Triangles, tri, tris),
                                         GeomTypeOf((GeomAbs_SurfaceType)buffers.FaceTypes[i]));
        }
        var edgeMeshes = new EdgeMesh[edges.Count];
        for (int i = 0; i < edgeMeshes.Length; i++)
        {
            int point = (int)buffers.EdgePoints[i], points = (int)buffers.EdgePoints[i + 1] - point;
            edgeMeshes[i] = new EdgeMesh(new ReadOnlyMemory<float>(buffers.Points, 3 * point, 3 * points),
                                         GeomTypeOf((GeomAbs_CurveType)buffers.EdgeTypes[i]));
        }
        return new ShapeMesh(faceMeshes, edgeMeshes, [.. shape.Vertices().Select(v => v.ToVector())], tolerances);
    }

    private static GeomType GeomTypeOf(GeomAbs_SurfaceType type) => Shape.GeomTypeOf(type);

    private static GeomType GeomTypeOf(GeomAbs_CurveType type) => Shape.GeomTypeOf(type);
}

/// <summary>
/// A face's mesh: its nodes, with the surface's normal at each (so shading is smooth), and triangles wound to face
/// out of the shape (a reversed face's are flipped).
/// </summary>
public sealed class FaceMesh
{
    internal FaceMesh(ReadOnlyMemory<float> positions, ReadOnlyMemory<float> normals, ReadOnlyMemory<int> triangles, GeomType type)
        => (Positions, Normals, Triangles, Type) = (positions, normals, triangles, type);

    /// <summary>The nodes: x, y, z each.</summary>
    public ReadOnlyMemory<float> Positions { get; }

    /// <summary>The unit normal at each node, facing out: x, y, z each.</summary>
    public ReadOnlyMemory<float> Normals { get; }

    /// <summary>The triangles: three indices each into this face's nodes, counter-clockwise seen from outside.</summary>
    public ReadOnlyMemory<int> Triangles { get; }

    /// <summary>The kind of surface: plane, cylinder, B-spline and so on.</summary>
    public GeomType Type { get; }

    /// <summary>The number of nodes.</summary>
    public int NodeCount => Positions.Length / 3;

    /// <summary>The number of triangles.</summary>
    public int TriangleCount => Triangles.Length / 3;
}

/// <summary>
/// An edge's polyline, running the way the edge does. Where the edge bounds a face it is the face mesh's own boundary,
/// so it lies exactly on the shaded surface.
/// </summary>
public sealed class EdgeMesh
{
    internal EdgeMesh(ReadOnlyMemory<float> points, GeomType type) => (Points, Type) = (points, type);

    /// <summary>The points: x, y, z each.</summary>
    public ReadOnlyMemory<float> Points { get; }

    /// <summary>The kind of curve: line, circle, B-spline and so on.</summary>
    public GeomType Type { get; }

    /// <summary>The number of points.</summary>
    public int PointCount => Points.Length / 3;
}

/// <summary>
/// A node of a shape's assembly tree, triangulated: a part with its mesh, or an assembly with its children. Locations
/// are relative to the parent. Instances of one geometry at different locations share the same
/// <see cref="ShapeMesh"/> object, made once.
/// </summary>
public sealed class MeshNode
{
    internal MeshNode(string label, Color? color, Location location, ShapeMesh? mesh, IReadOnlyList<MeshNode> children)
        => (Label, Color, Location, Mesh, Children) = (label, color, location, mesh, children);

    /// <summary>The shape's label.</summary>
    public string Label { get; }

    /// <summary>The shape's colour (with its alpha), if it has one.</summary>
    public Color? Color { get; }

    /// <summary>Where the node is placed in its parent.</summary>
    public Location Location { get; }

    /// <summary>The mesh of a part, in its own coordinates (before <see cref="Location"/>); null for an assembly.</summary>
    public ShapeMesh? Mesh { get; }

    /// <summary>An assembly's children; empty for a part.</summary>
    public IReadOnlyList<MeshNode> Children { get; }

    /// <summary>
    /// The tree of a shape: an assembly (a compound with <see cref="Compound.AssemblyChildren"/>) gives a node per
    /// child, any other shape a part.
    /// </summary>
    internal static MeshNode Of(Shape shape, MeshTolerances tolerances)
    {
        var cache = new List<(TopoDS_Shape Key, ShapeMesh Mesh)>();
        return Build(shape, tolerances, cache);
    }

    private static MeshNode Build(Shape shape, MeshTolerances tolerances, List<(TopoDS_Shape Key, ShapeMesh Mesh)> cache)
    {
        var location = shape.IsEmpty ? new Location() : shape.Location;
        if (!shape.IsEmpty && shape is Compound { AssemblyChildren.Count: > 0 } assembly)
            return new MeshNode(shape.Label, shape.Color, location, null,
                                [.. assembly.AssemblyChildren.Select(child => Build(child, tolerances, cache))]);
        return new MeshNode(shape.Label, shape.Color, location, Instance(shape, tolerances, cache), []);
    }

    /// <summary>The mesh of a part in its own coordinates, shared with any earlier part of the same geometry.</summary>
    private static ShapeMesh Instance(Shape shape, MeshTolerances tolerances, List<(TopoDS_Shape Key, ShapeMesh Mesh)> cache)
    {
        if (shape.IsEmpty)
            return ShapeMesh.Empty;
        foreach (var (key, mesh) in cache)
            if (key.IsPartner(shape.TopoShape) && key.Orientation() == shape.TopoShape.Orientation())
                return mesh;
        using var identity = new TopLoc_Location();
        var unplaced = shape.Placed(shape.TopoShape.Located(identity));
        var made = ShapeMesh.Of(unplaced, tolerances);
        cache.Add((shape.TopoShape, made));
        return made;
    }
}

public abstract partial class Shape
{
    /// <summary>
    /// The shape's triangulation for display and picking (see <see cref="ShapeMesh"/>), with a linear deflection of
    /// <paramref name="linearDeflection"/> (relative to each edge's size if <paramref name="relative"/>) and an angular
    /// one of <paramref name="angularDeflection"/> radians. An empty shape gives an empty mesh. The shape isn't
    /// changed, and the same shape and tolerances always give the same mesh.
    /// </summary>
    public ShapeMesh Triangulate(double linearDeflection, double angularDeflection = 0.2, bool relative = false) =>
        ShapeMesh.Of(this, new MeshTolerances(linearDeflection, angularDeflection, relative));

    /// <summary>The shape's triangulation with the given tolerances, by default <see cref="MeshTolerances.For"/> it.</summary>
    public ShapeMesh Triangulate(MeshTolerances? tolerances = null) => ShapeMesh.Of(this, tolerances ?? MeshTolerances.For(this));

    /// <summary>
    /// The shape's assembly tree, triangulated (see <see cref="MeshNode"/>): labels, colours and locations, with each
    /// geometry meshed once however many instances there are. By default the tolerances are
    /// <see cref="MeshTolerances.For"/> the whole shape, so every part is meshed to the same quality.
    /// </summary>
    public MeshNode TriangulateTree(MeshTolerances? tolerances = null) => MeshNode.Of(this, tolerances ?? MeshTolerances.For(this));
}
