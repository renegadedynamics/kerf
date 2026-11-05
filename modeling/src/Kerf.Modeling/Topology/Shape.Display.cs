using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;
using System.Runtime.CompilerServices;
using System.Text;

namespace Kerf.Modeling;

public static partial class ShapeExtensions
{
    private sealed record DisplayNode(string Label, string Name, long Address, string Position, List<DisplayNode> Children)
    {
        public int Height => Children.Count == 0 ? 0 : 1 + Children.Max(c => c.Height);
    }

    extension<TSelf>(TSelf self) where TSelf : IShape
    {
        /// <summary>
        /// The internal structure of an assembly, or of a shape's topology down to <paramref name="limitClass"/>, as an
        /// indented tree: each line the node's label and class, an identity, and its location (an assembly) or centre (a shape).
        /// </summary>
        /// <param name="limitClass">The kind of the leaf nodes of a shape's tree.</param>
        /// <param name="showCenter">Show centres (true) or positions; by default locations for an assembly, centres for a shape.</param>
        public string ShowTopology(ShapeType limitClass = ShapeType.Vertex, bool? showCenter = null)
        {
            if (!self.IsEmpty && self is Assembly { Children.Count: > 0 } assembly)
            {
                bool center = showCenter ?? false;
                return ShapeExtensions.ShowTree(ShapeExtensions.AssemblyTree(assembly, center));
            }
            bool centers = showCenter ?? true;
            return ShapeExtensions.ShowTree(ShapeExtensions.BuildTree(self.AsShape(), (limitClass), centers, centers ? "Center" : "Position"));
        }

        /// <summary>
        /// Faces projected onto this shape along a path on it: each face is placed normal to the surface at its position
        /// along the path (from <paramref name="start"/>, by its centre's x) and projected onto the surface, so faces may
        /// distort where the surface curves.
        /// </summary>
        public ShapeList<Face> ProjectFaces(IEnumerable<Face> faces, IShape1D path, double start = 0)
        {
            var faceList = faces.ToList();
            var pathLength = path.Length;
            var shapeCenter = self.Center();
            var firstMinX = faceList[0].BoundBox().Min.X;
            var projected = new List<Face>();
            foreach (var face in faceList)
            {
                var box = face.BoundBox();
                var faceCenterX = (box.Min.X + box.Max.X) / 2;
                var relative = start + (faceCenterX - firstMinX) / pathLength;
                var pathPosition = path.PositionAt(relative);
                var pathTangent = path.TangentAt(relative);
                var projectionAxis = new Axis(pathPosition, shapeCenter - pathPosition);
                var (surfacePoint, surfaceNormal) = SurfaceIntersection.FindPoints(self.AsShape(), projectionAxis, Constants.Tolerance)[0];
                var normalPlane = new Plane(surfacePoint, pathTangent, surfaceNormal);
                var placed = normalPlane.FromLocalCoords(face.Moved(new Location(new Vector(-faceCenterX, 0, 0))));
                // Onto a curved surface the projection may come back as a shell of several faces
                projected.AddRange(placed.ProjectToShape(self.AsShape(), surfaceNormal * -1)[0].SelectFaces());
            }
            return new ShapeList<Face>(projected);
        }

        /// <summary>As the other forms of <c>ProjectFaces</c>.</summary>
        public ShapeList<Face> ProjectFaces(Compound faces, IShape1D path, double start = 0) => self.ProjectFaces(faces.SelectFaces(), path, start);
    }


    // Containers first, as build123d orders the shape types
    private static readonly ShapeType[] OrderedShapes =
    [
        ShapeType.Compound, ShapeType.CompSolid, ShapeType.Solid,
        ShapeType.Shell, ShapeType.Face, ShapeType.Wire,
        ShapeType.Edge, ShapeType.Vertex,
    ];

    private static DisplayNode AssemblyTree(ModelShape node, bool showCenter) => new(
        node.Label, node.GetType().Name.PadRight(9), RuntimeHelpers.GetHashCode(node),
        showCenter ? "Center" + node.Center().ToString("G6") : $"Location({node.Location.Position.ToString("G6")}, {node.Location.Orientation.ToString("G6")})",
        node is Assembly assembly ? [.. assembly.Children.Select(ch => AssemblyTree(ch, showCenter))] : []);

    private static DisplayNode BuildTree(Shape shape, ShapeType limit, bool showCenter, string caption)
    {
        var label = (shape.Type).ToString();
        var position = caption + (showCenter
            ? Kerf.Modeling.BoundBox.FromTopoDS(shape).Center().ToString("G6")
            : new Location(shape.Placement).Position.ToString("G6"));
        var children = new List<DisplayNode>();
        foreach (var child in shape.Children)
        {
            // As build123d's _build_tree, a child carries its centre whatever the root shows
            if (Array.IndexOf(ShapeExtensions.OrderedShapes, child.Type) <= Array.IndexOf(ShapeExtensions.OrderedShapes, limit))
                children.Add(ShapeExtensions.BuildTree(child, limit, true, caption));
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
}

public static partial class Shape3DExtensions
{
    extension<TSelf>(TSelf self) where TSelf : IShape3D
    {
        /// <summary>The points where an axis meets the shape's surface, nearest the axis's start first, each with the normal there.</summary>
        public IReadOnlyList<(Vector Point, Vector Normal)> FindIntersectionPoints(Axis other, double tolerance = Constants.Tolerance) =>
            SurfaceIntersection.FindPoints(self.AsShape(), other, tolerance);
    }

}

/// <summary>The axis-surface intersection shared by faces, shells, solids and compounds (Mixin3D borrows Mixin2D's).</summary>
internal static class SurfaceIntersection
{
    internal static IReadOnlyList<(Vector Point, Vector Normal)> FindPoints(Shape shape, Axis axis, double tolerance)
    {
        if (shape.IsEmpty)
            return [];
        var ax1 = axis.ToAxis3D();
        var line = new Line3D(ax1);
        var plane = new Plane(axis);
        var found = new List<(Face Face, Vector Point, double Distance)>();
        foreach (var hit in MeasureOperations.Intersect(shape, in line, tolerance))
        {
            var point = new Vector(hit.Point);
            found.Add((hit.Face, point, plane.ToLocalCoords(point).Z));
        }
        return [.. found.OrderBy(f => f.Distance).Select(f =>
        {
            var pnt = f.Point.ToPoint3D();
            return (f.Point, TopoUtil.FaceNormalAt(f.Face, pnt));
        })];
    }
}
