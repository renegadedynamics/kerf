using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using System.Text;

namespace Kerf.Modeling;

public abstract partial class Shape
{
    private sealed record DisplayNode(string Label, string Name, long Address, string Position, List<DisplayNode> Children)
    {
        public int Height => Children.Count == 0 ? 0 : 1 + Children.Max(c => c.Height);
    }

    // Containers first, as build123d orders the shape types
    private static readonly global::Kerf.Topology.ShapeType[] OrderedShapes =
    [
        global::Kerf.Topology.ShapeType.Compound, global::Kerf.Topology.ShapeType.CompSolid, global::Kerf.Topology.ShapeType.Solid,
        global::Kerf.Topology.ShapeType.Shell, global::Kerf.Topology.ShapeType.Face, global::Kerf.Topology.ShapeType.Wire,
        global::Kerf.Topology.ShapeType.Edge, global::Kerf.Topology.ShapeType.Vertex,
    ];

    /// <summary>
    /// The internal structure of an assembly, or of a shape's topology down to <paramref name="limitClass"/>, as an
    /// indented tree: each line the node's label and class, an identity, and its location (an assembly) or centre (a shape).
    /// </summary>
    /// <param name="limitClass">The kind of the leaf nodes of a shape's tree.</param>
    /// <param name="showCenter">Show centres (true) or positions; by default locations for an assembly, centres for a shape.</param>
    public string ShowTopology(ShapeType limitClass = ShapeType.Vertex, bool? showCenter = null)
    {
        if (!IsEmpty && this is Compound { AssemblyChildren.Count: > 0 })
        {
            bool center = showCenter ?? false;
            return ShowTree(AssemblyTree(this, center));
        }
        bool centers = showCenter ?? true;
        return ShowTree(BuildTree(TopoShape, TopoUtil.ToTopAbs(limitClass), centers, centers ? "Center" : "Position"));
    }

    private static DisplayNode AssemblyTree(Shape node, bool showCenter) => new(
        node.Label, node.GetType().Name.PadRight(9), RuntimeHelpers.GetHashCode(node),
        showCenter ? "Center" + node.Center().ToString(".6g") : node.Location.Repr(),
        node is Compound c ? [.. c.AssemblyChildren.Select(ch => AssemblyTree(ch, showCenter))] : []);

    private static DisplayNode BuildTree(global::Kerf.Topology.Shape shape, global::Kerf.Topology.ShapeType limit, bool showCenter, string caption)
    {
        var label = TopoUtil.FromTopAbs(shape.Type).ToString();
        var position = caption + (showCenter
            ? BoundBox.FromTopoDS(shape).Center().ToString(".6g")
            : new Location(shape.Placement).Position.ToString(".6g"));
        var children = new List<DisplayNode>();
        foreach (var child in shape.Children)
        {
            // As build123d's _build_tree, a child carries its centre whatever the root shows
            if (Array.IndexOf(OrderedShapes, child.Type) <= Array.IndexOf(OrderedShapes, limit))
                children.Add(BuildTree(child, limit, true, caption));
        }
        return new DisplayNode(label, "", (long)(shape.StdHash() & 0xffffffffffff), position, children);
    }

    /// <summary>The tree drawn as anytree's RenderTree does, its labels padded to a common width.</summary>
    private static string ShowTree(DisplayNode root)
    {
        // The width of each level's labels
        var sizes = new List<(int Height, int Length)>();
        void Collect(DisplayNode n)
        {
            sizes.Add((n.Height, n.Label.Length));
            n.Children.ForEach(Collect);
        }
        Collect(root);
        var maxPerLevel = Enumerable.Range(0, root.Height + 1)
            .Select(h => Math.Max(4, sizes.Where(s => s.Height == h).Select(s => s.Length).DefaultIfEmpty(0).Max())).ToList();
        maxPerLevel.Reverse();
        var width = maxPerLevel.Select((l, i) => l + i * 4).Max() + 1;

        var sb = new StringBuilder();
        void Render(DisplayNode n, string prefix, string childPrefix)
        {
            sb.Append((prefix + n.Label).PadRight(width)).Append(n.Name).Append($"at 0x{n.Address:x}, {n.Position}\n");
            for (int i = 0; i < n.Children.Count; i++)
            {
                bool last = i == n.Children.Count - 1;
                Render(n.Children[i], childPrefix + (last ? "└── " : "├── "), childPrefix + (last ? "    " : "│   "));
            }
        }
        Render(root, "", "");
        return sb.ToString();
    }

    /// <summary>
    /// Faces projected onto this shape along a path on it: each face is placed normal to the surface at its position
    /// along the path (from <paramref name="start"/>, by its centre's x) and projected onto the surface, so faces may
    /// distort where the surface curves.
    /// </summary>
    public ShapeList<Face> ProjectFaces(IEnumerable<Face> faces, Shape1D path, double start = 0)
    {
        var faceList = faces.ToList();
        var pathLength = path.Length;
        var shapeCenter = Center();
        var firstMinX = faceList[0].BoundingBox().Min.X;
        var projected = new List<Face>();
        foreach (var face in faceList)
        {
            var box = face.BoundingBox();
            var faceCenterX = (box.Min.X + box.Max.X) / 2;
            var relative = start + (faceCenterX - firstMinX) / pathLength;
            var pathPosition = path.PositionAt(relative);
            var pathTangent = path.TangentAt(relative);
            var projectionAxis = new Axis(pathPosition, shapeCenter - pathPosition);
            var (surfacePoint, surfaceNormal) = SurfaceIntersection.FindPoints(this, projectionAxis, Constants.Tolerance)[0];
            var normalPlane = new Plane(surfacePoint, pathTangent, surfaceNormal);
            var placed = normalPlane.FromLocalCoords(face.Moved(new Location((-faceCenterX, 0, 0))));
            projected.Add((Face)placed.ProjectToShape(this, surfaceNormal * -1)[0]);
        }
        return new ShapeList<Face>(projected);
    }

    /// <inheritdoc cref="ProjectFaces(IEnumerable{Face}, Shape1D, double)"/>
    public ShapeList<Face> ProjectFaces(Compound faces, Shape1D path, double start = 0) => ProjectFaces(faces.Faces(), path, start);
}

public abstract partial class Shape3D
{
    /// <summary>The points where an axis meets the shape's surface, nearest the axis's start first, each with the normal there.</summary>
    public IReadOnlyList<(Vector Point, Vector Normal)> FindIntersectionPoints(Axis other, double tolerance = Constants.Tolerance) =>
        SurfaceIntersection.FindPoints(this, other, tolerance);
}

/// <summary>The axis-surface intersection shared by faces, shells, solids and compounds (Mixin3D borrows Mixin2D's).</summary>
internal static class SurfaceIntersection
{
    internal static IReadOnlyList<(Vector Point, Vector Normal)> FindPoints(Shape shape, Axis axis, double tolerance)
    {
        if (shape.IsEmpty)
            return [];
        var ax1 = axis.ToGpAx1();
        var line = new Line3D(ax1);
        var plane = new Plane(axis);
        var found = new List<(global::Kerf.Topology.Face Face, Vector Point, double Distance)>();
        foreach (var hit in shape.TopoShape.Intersect(line, tolerance))
        {
            var point = new Vector(hit.Point);
            found.Add((hit.Face, point, plane.ToLocalCoords(point).Z));
        }
        return [.. found.OrderBy(f => f.Distance).Select(f =>
        {
            var pnt = f.Point.ToGpPnt();
            return (f.Point, TopoUtil.FaceNormalAt(f.Face, pnt));
        })];
    }
}
