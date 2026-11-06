namespace Kerf.Modeling;

/// <summary>
/// The tolerances to triangulate with: the distance between the mesh and the surface to aim for (<see cref="Linear"/>,
/// in the model's units, or relative to each edge's size with <see cref="Relative"/>) and the largest angle between
/// neighbouring facets (<see cref="Angular"/>, in radians).
/// </summary>
/// <remarks>
/// <see cref="Linear"/> isn't a guaranteed maximum. On elementary surfaces the mesher relies on its grid of nodes, so a long,
/// thin face such as a fillet can stray further: 3.7 times on a filleted bottle. <see cref="ControlAllSurfaces"/> checks
/// those surfaces too (there, 0.65 times, with 39% more triangles), which suits a mesh used as the shape, such as for STL.
/// It holds closer to the value, though it still isn't a guarantee.
/// </remarks>
public readonly record struct MeshTolerances(double Linear, double Angular, bool Relative = false, bool ControlAllSurfaces = false)
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
        var diagonal = shape.IsEmpty ? 0 : shape.Bounds().Diagonal;
        // A point or an empty shape has no size: any small value does
        return new MeshTolerances(diagonal > Constants.Tolerance ? diagonal * deviation / 100 : 1e-3, angular);
    }
}

/// <summary>
/// A shape's triangulation, for display and picking: its faces' meshes, its edges' polylines and its vertices, each in
/// the order of the shape's <c>Faces</c>, <c>Edges</c> and <c>Vertices</c>, so
/// an index picked in a viewer is the index of the face, edge or vertex. Coordinates are global (locations applied).
/// </summary>
/// <remarks>The buffers are shared, read-only slices of a few large arrays; the mesh never changes.</remarks>
public sealed class ShapeMesh
{
    internal ShapeMesh(IReadOnlyList<FaceMesh> faces, IReadOnlyList<EdgeMesh> edges, IReadOnlyList<Point3D> vertices, MeshTolerances tolerances)
        => (Faces, Edges, Vertices, Tolerances) = (faces, edges, vertices, tolerances);

    /// <summary>The mesh of an empty shape: no faces, edges or vertices.</summary>
    public static ShapeMesh Empty { get; } = new([], [], [], default);

    /// <summary>The faces' meshes, in the order of <c>Faces</c>.</summary>
    public IReadOnlyList<FaceMesh> Faces { get; }

    /// <summary>The edges' polylines, in the order of <c>Edges</c>: each topological edge once.</summary>
    public IReadOnlyList<EdgeMesh> Edges { get; }

    /// <summary>The vertices' positions, in the order of <c>Vertices</c>.</summary>
    public IReadOnlyList<Point3D> Vertices { get; }

    /// <summary>The tolerances the mesh was made with.</summary>
    public MeshTolerances Tolerances { get; }

    /// <summary>The number of triangles in all the faces.</summary>
    public int TriangleCount => Faces.Sum(f => f.TriangleCount);

    /// <summary>The number of nodes in all the faces.</summary>
    public int NodeCount => Faces.Sum(f => f.NodeCount);

    /// <summary>
    /// Triangulates a shape. The shape isn't changed: the kernel's mesher meshes in a working overlay and reads the
    /// meshes out, so meshing is safe from any thread, on shapes that share geometry too.
    /// </summary>
    internal static ShapeMesh Of(Shape shape, MeshTolerances tolerances) =>
        shape.IsEmpty ? Empty : Failures.Doing(() => "triangulate", () => Mesh(shape, tolerances));

    private static ShapeMesh Mesh(Shape shape, MeshTolerances tolerances)
    {
        // The kernel's mesher meshes in a working overlay and reads the meshes out (unit 10L3): the shape is never
        // written, and its faces, edges and vertices come in the order of Faces(), Edges() and Vertices()
        var options = new global::Kerf.Meshing.MeshOptions
        {
            LinearDeflection = tolerances.Linear,
            AngularDeflection = tolerances.Angular,
            Relative = tolerances.Relative,
            Parallel = true,
            EdgeDeflection = tolerances.Relative ? MeshTolerances.For(shape).Linear : null,
            ControlAllSurfaces = tolerances.ControlAllSurfaces,
        };
        var mesh = Failures.Get(global::Kerf.Meshing.Mesher.Mesh(shape, options), () => "mesh");
        var faceMeshes = mesh.Faces.Select(f => new FaceMesh(f.Positions, f.Normals, f.Triangles, GeomTypeOf(f.Kind))).ToArray();
        var edgeMeshes = mesh.Edges.Select(e => new EdgeMesh(e.Points, GeomTypeOf(e.Kind))).ToArray();
        return new ShapeMesh(faceMeshes, edgeMeshes, [.. shape.Vertices().Select(v => v.Point)], tolerances);
    }

    private static GeomType GeomTypeOf(SurfaceKind type) => ShapeExtensions.GeomTypeOf(type);

    private static GeomType GeomTypeOf(CurveKind type) => ShapeExtensions.GeomTypeOf(type);
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
    /// The tree of a shape: an assembly (a compound with <see cref="Assembly.Children"/>) gives a node per
    /// child, any other shape a part.
    /// </summary>
    internal static MeshNode Of(IShape shape, MeshTolerances tolerances)
    {
        var cache = new List<(Shape Key, ShapeMesh Mesh)>();
        return Build(shape, tolerances, cache);
    }

    private static MeshNode Build(IShape shape, MeshTolerances tolerances, List<(Shape Key, ShapeMesh Mesh)> cache)
    {
        var location = shape.IsEmpty ? new Location() : shape.Location;
        var model = shape as ModelShape;
        if (!shape.IsEmpty && model is Assembly { Children.Count: > 0 } assembly)
            return new MeshNode(assembly.Label, assembly.Color, location, null,
                                [.. assembly.Children.Select(child => Build(child, tolerances, cache))]);
        return new MeshNode(model?.Label ?? "", model?.Color, location, Instance(shape.AsShape(), tolerances, cache), []);
    }

    /// <summary>The mesh of a part in its own coordinates, shared with any earlier part of the same geometry.</summary>
    private static ShapeMesh Instance(Shape shape, MeshTolerances tolerances, List<(Shape Key, ShapeMesh Mesh)> cache)
    {
        if (shape.IsEmpty)
            return ShapeMesh.Empty;
        foreach (var (key, mesh) in cache)
            if (key.IsPartner(shape) && key.Orientation == shape.Orientation)
                return mesh;
        var unplaced = shape.Located(new Placement());
        var made = ShapeMesh.Of(unplaced, tolerances);
        cache.Add((shape, made));
        return made;
    }
}

public static partial class ShapeExtensions
{
    extension<TSelf>(TSelf self) where TSelf : IShape
    {
        /// <summary>
        /// The shape's triangulation for display and picking (see <see cref="ShapeMesh"/>), with a linear deflection of
        /// <paramref name="linearDeflection"/> (relative to each edge's size if <paramref name="relative"/>) and an angular
        /// one of <paramref name="angularDeflection"/> radians. An empty shape gives an empty mesh. The shape isn't
        /// changed, and the same shape and tolerances always give the same mesh.
        /// </summary>
        public ShapeMesh Triangulate(double linearDeflection, double angularDeflection = 0.2, bool relative = false) =>
            ShapeMesh.Of(self.AsShape(), new MeshTolerances(linearDeflection, angularDeflection, relative));

        /// <summary>The shape's triangulation with the given tolerances, by default <see cref="MeshTolerances.For"/> it.</summary>
        public ShapeMesh Triangulate(MeshTolerances? tolerances = null) => ShapeMesh.Of(self.AsShape(), tolerances ?? MeshTolerances.For(self.AsShape()));

        /// <summary>
        /// The shape's assembly tree, triangulated (see <see cref="MeshNode"/>): labels, colours and locations, with each
        /// geometry meshed once however many instances there are. By default the tolerances are
        /// <see cref="MeshTolerances.For"/> the whole shape, so every part is meshed to the same quality.
        /// </summary>
        public MeshNode TriangulateTree(MeshTolerances? tolerances = null) => MeshNode.Of(self, tolerances ?? MeshTolerances.For(self.AsShape()));
    }

}
