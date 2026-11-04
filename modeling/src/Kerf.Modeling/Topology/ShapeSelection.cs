using Kerf.Geometry;

namespace Kerf.Modeling;

/// <summary>
/// The selections suited to CAD, on any sequence of shapes: by geometry type, direction, position or measure, sorted
/// along an axis, and grouped. They sit beside LINQ's own and chain with them, each returning a
/// <see cref="ShapeList{T}"/>: <c>part.Edges().Where(e =&gt; e.IsClosed).OrderBy(Axis.Z).Last()</c>.
/// </summary>
/// <remarks>
/// They are build123d's <c>filter_by</c>, <c>filter_by_position</c>, <c>filter_by_value</c>, <c>sort_by</c>,
/// <c>sort_by_distance</c> and <c>group_by</c>, with typed arguments in place of its operators and its
/// <c>reverse</c> flag: <c>OrderByDescending</c>, <c>NotParallelTo</c> and <c>WhereNot</c> for the reverse. Every one
/// takes plain values (an axis, an enum, a number), so a caller that can't pass a delegate, such as a spreadsheet
/// formula, can still select. A sort is stable: shapes with equal keys keep their order.
/// </remarks>
public static class ShapeSelection
{
    extension<T>(IEnumerable<T> shapes) where T : Shape
    {
        /// <summary>The shapes as a <see cref="ShapeList{T}"/>, in order.</summary>
        public ShapeList<T> ToShapeList() => shapes as ShapeList<T> ?? new ShapeList<T>(shapes);

        // ---- Filtering ----

        /// <summary>The shapes of a geometry type: <c>faces.Where(GeomType.Plane)</c>.</summary>
        public ShapeList<T> Where(GeomType geomType) => Keep(shapes, s => s.GeomType == geomType);

        /// <summary>The shapes not of a geometry type.</summary>
        public ShapeList<T> WhereNot(GeomType geomType) => Keep(shapes, s => s.GeomType != geomType);

        /// <summary>The shapes of a convexity relative to the shape each was selected from.</summary>
        public ShapeList<T> Where(Convexity convexity) => Keep(shapes, s => s.Convexity == convexity);

        /// <summary>The shapes not of a convexity relative to the shape each was selected from.</summary>
        public ShapeList<T> WhereNot(Convexity convexity) => Keep(shapes, s => s.Convexity != convexity);

        /// <summary>
        /// The planar faces whose normal, and the linear edges whose direction, is parallel to <paramref name="axis"/>
        /// within <paramref name="tolerance"/> radians.
        /// </summary>
        public ShapeList<T> ParallelTo(Axis axis, double tolerance = 1e-5) => Keep(shapes, s => IsParallelTo(s, axis, tolerance));

        /// <summary>The shapes that <c>ParallelTo(axis)</c> leaves out.</summary>
        public ShapeList<T> NotParallelTo(Axis axis, double tolerance = 1e-5) => Keep(shapes, s => !IsParallelTo(s, axis, tolerance));

        /// <summary>The faces, wires and edges parallel to <paramref name="plane"/> within <paramref name="tolerance"/>.</summary>
        public ShapeList<T> ParallelTo(Plane plane, double tolerance = 1e-5)
        {
            var planeAxis = new Axis(plane.Origin, plane.ZDir);
            return Keep(shapes, s => IsParallelTo(s, plane, planeAxis, tolerance));
        }

        /// <summary>The shapes that <c>ParallelTo(plane)</c> leaves out.</summary>
        public ShapeList<T> NotParallelTo(Plane plane, double tolerance = 1e-5)
        {
            var planeAxis = new Axis(plane.Origin, plane.ZDir);
            return Keep(shapes, s => !IsParallelTo(s, plane, planeAxis, tolerance));
        }

        /// <summary>
        /// The shapes whose centre lies between <paramref name="minimum"/> and <paramref name="maximum"/> along
        /// <paramref name="axis"/>, sorted along it (build123d's <c>filter_by_position</c>).
        /// </summary>
        public ShapeList<T> Between(Axis axis, double minimum, double maximum, bool includeMinimum = true, bool includeMaximum = true)
        {
            var plane = new Plane(axis);
            return Keep(shapes, o =>
            {
                var z = plane.ToLocalCoords(o).Center().Z;
                return (includeMinimum ? minimum <= z : minimum < z) && (includeMaximum ? z <= maximum : z < maximum);
            }).OrderBy(axis);
        }

        /// <summary>
        /// The shapes whose <paramref name="key"/> is within <paramref name="tolerance"/> of <paramref name="value"/>: the
        /// edges of a length, the faces of a radius (build123d's <c>filter_by_value</c>). A shape without the measure (the
        /// radius of a planar face) doesn't match.
        /// </summary>
        public ShapeList<T> WithValue(ShapeKey key, double value, double tolerance = Constants.Tolerance) =>
            Keep(shapes, o => HasValue(o, key, value, tolerance));

        /// <summary>
        /// The shapes whose <paramref name="measure"/> is <paramref name="value"/>: within <paramref name="tolerance"/>
        /// for numbers, equal otherwise (an enum). A shape whose measure is null never matches.
        /// </summary>
        public ShapeList<T> WithValue<TValue>(Func<T, TValue> measure, TValue value, double tolerance = Constants.Tolerance) =>
            Keep(shapes, o => HasValue(measure(o), value, tolerance));

        /// <summary>The shapes that <c>WithValue(key, value)</c> leaves out, those without the measure among them.</summary>
        public ShapeList<T> WithoutValue(ShapeKey key, double value, double tolerance = Constants.Tolerance) =>
            Keep(shapes, o => !HasValue(o, key, value, tolerance));

        /// <summary>The shapes that <c>WithValue(measure, value)</c> leaves out, those whose measure is null among them.</summary>
        public ShapeList<T> WithoutValue<TValue>(Func<T, TValue> measure, TValue value, double tolerance = Constants.Tolerance) =>
            Keep(shapes, o => !HasValue(measure(o), value, tolerance));

        // ---- Sorting ----

        /// <summary>The shapes sorted by the position of their centres along <paramref name="axis"/>.</summary>
        public ShapeList<T> OrderBy(Axis axis) => Sorted<T>(shapes, AlongAxis(axis), descending: false);

        /// <summary>The shapes sorted along <paramref name="axis"/>, the furthest first.</summary>
        public ShapeList<T> OrderByDescending(Axis axis) => Sorted<T>(shapes, AlongAxis(axis), descending: true);

        /// <summary>
        /// The shapes sorted by a measure, the smallest first. Shapes without it (a vertex has no radius) are left out of
        /// a sort by radius.
        /// </summary>
        public ShapeList<T> OrderBy(ShapeKey key) => SortedByKey(shapes, key, descending: false);

        /// <summary>The shapes sorted by a measure, the largest first.</summary>
        public ShapeList<T> OrderByDescending(ShapeKey key) => SortedByKey(shapes, key, descending: true);

        /// <summary>The shapes sorted by where along an edge or wire the point closest to each one's centre lies.</summary>
        /// <exception cref="ArgumentException"><paramref name="path"/> is empty.</exception>
        public ShapeList<T> OrderBy(Shape1D path) => Sorted<T>(shapes, AlongPath(path, "sort"), descending: false);

        /// <summary>The shapes sorted along <paramref name="path"/>, the furthest along first.</summary>
        public ShapeList<T> OrderByDescending(Shape1D path) => Sorted<T>(shapes, AlongPath(path, "sort"), descending: true);

        /// <summary>The shapes sorted by their minimal distance to <paramref name="other"/>, the nearest first.</summary>
        public ShapeList<T> OrderByDistanceTo(Shape other) => Sorted(shapes, o => o.DistanceTo(other), descending: false);

        /// <summary>The shapes sorted by their minimal distance to a point, the nearest first.</summary>
        public ShapeList<T> OrderByDistanceTo(Vector point) => Sorted(shapes, o => o.DistanceTo(point), descending: false);

        /// <summary>The shapes sorted by their minimal distance to <paramref name="other"/>, the furthest first.</summary>
        public ShapeList<T> OrderByDescendingDistanceTo(Shape other) => Sorted(shapes, o => o.DistanceTo(other), descending: true);

        /// <summary>The shapes sorted by their minimal distance to a point, the furthest first.</summary>
        public ShapeList<T> OrderByDescendingDistanceTo(Vector point) => Sorted(shapes, o => o.DistanceTo(point), descending: true);

        // ---- Grouping ----

        /// <summary>
        /// The shapes grouped by the position of their centres along <paramref name="axis"/>, rounded to
        /// <paramref name="digits"/> decimal places, the groups in order along it: <c>edges.GroupBy(Axis.Z).Last()</c> is
        /// the topmost group.
        /// </summary>
        public ShapeGroups<T, double> GroupBy(Axis axis, int digits = 6)
        {
            var along = AlongAxis(axis);
            return new(o => Math.Round(along(o), digits), shapes);
        }

        /// <summary>The shapes grouped by a measure rounded to <paramref name="digits"/> decimal places, the smallest first.</summary>
        public ShapeGroups<T, double> GroupBy(ShapeKey key, int digits = 6) => key switch
        {
            ShapeKey.Length or ShapeKey.Radius or ShapeKey.Distance or ShapeKey.Area or ShapeKey.Volume =>
                new(o => Math.Round(Measures.Of(o, key), digits), shapes),
            _ => throw new ArgumentOutOfRangeException(nameof(key)),
        };

        /// <summary>The shapes grouped by where along an edge or wire the point closest to each one's centre lies.</summary>
        /// <exception cref="ArgumentException"><paramref name="path"/> is empty.</exception>
        public ShapeGroups<T, double> GroupBy(Shape1D path, int digits = 6)
        {
            var along = AlongPath(path, "group");
            return new(o => Math.Round(along(o), digits), shapes);
        }

        /// <summary>The shapes grouped by their convexity, in the enum's order.</summary>
        public ShapeGroups<T, Convexity> GroupByConvexity() => new(o => o.Convexity, shapes);

        /// <summary>
        /// The shapes grouped by a numeric key rounded to <paramref name="digits"/> decimal places, the groups in key
        /// order (where LINQ's <c>GroupBy</c> keeps them in the order met and compares keys exactly).
        /// </summary>
        public ShapeGroups<T, double> GroupByValue(Func<T, double> key, int digits = 6) => new(o => Math.Round(key(o), digits), shapes);

        /// <summary>
        /// The shapes grouped by a key, the groups in the order of <paramref name="comparer"/>, by default a key's own
        /// order, null first, and a sequence (an inertia matrix, say) element by element.
        /// </summary>
        public ShapeGroups<T, TKey> GroupByValue<TKey>(Func<T, TKey> key, IComparer<TKey>? comparer = null) => new(key, shapes, comparer);
    }

    private static ShapeList<T> Keep<T>(IEnumerable<T> shapes, Func<T, bool> keep) where T : Shape =>
        new(Enumerable.Where(shapes, keep));

    private static ShapeList<T> Sorted<T>(IEnumerable<T> shapes, Func<T, double> key, bool descending) where T : Shape =>
        new(descending ? Enumerable.OrderByDescending(shapes, key) : Enumerable.OrderBy(shapes, key));

    private static ShapeList<T> SortedByKey<T>(IEnumerable<T> shapes, ShapeKey key, bool descending) where T : Shape => key switch
    {
        ShapeKey.Radius => Sorted(Enumerable.Where(shapes, Measures.HasRadius), o => Measures.Radius(o), descending),
        ShapeKey.Length or ShapeKey.Distance or ShapeKey.Area or ShapeKey.Volume => Sorted(shapes, o => Measures.Of(o, key), descending),
        _ => throw new ArgumentOutOfRangeException(nameof(key)),
    };

    private static Func<Shape, double> AlongAxis(Axis axis)
    {
        var axisAsLocation = axis.Location.Inverse();
        return o => (axisAsLocation * new Location(o.Center())).Position.Z;
    }

    private static Func<Shape, double> AlongPath(Shape1D path, string verb)
    {
        ArgumentNullException.ThrowIfNull(path);
        if (path.IsEmpty)
            throw new ArgumentException($"Can't {verb} along an empty path.", nameof(path));
        return o => path.ParamAtPoint(path.ClosestPoints(o.Center()).OnThis);
    }

    // A radius only some shapes have; any other measure of a shape that lacks it throws, as build123d's does
    private static bool HasValue(Shape o, ShapeKey key, double value, double tolerance) =>
        key == ShapeKey.Radius
            ? Measures.RadiusOrNull(o) is { } r && Math.Abs(r - value) <= tolerance
            : Math.Abs(Measures.Of(o, key) - value) <= tolerance;

    private static bool HasValue<TValue>(TValue measured, TValue value, double tolerance) =>
        measured is not null &&
        (AsDouble(measured) is { } m && AsDouble(value) is { } v
            ? Math.Abs(m - v) <= tolerance
            : EqualityComparer<TValue>.Default.Equals(measured, value));

    private static double? AsDouble(object? x) => x switch
    {
        double d => d,
        float f => f,
        int i => i,
        long l => l,
        _ => null,
    };

    private static bool IsParallelTo(Shape shape, Axis axis, double tolerance)
    {
        Axis shapeAxis;
        if (shape is Face face && face.IsPlanarSurface())
        {
            var (u0, _, v0, _) = face.UVBounds();
            shapeAxis = new Axis(shape.Center(), face.NormalAtParameters(u0, v0));
        }
        else if (shape is Edge edge && shape.GeomType == GeomType.Line)
        {
            var curve = edge.GeomAdaptor();
            var d = curve.Derivatives(curve.Range.Min, 1);
            var dir = new Direction3D(d.D1);
            shapeAxis = new Axis(new Vector(d.Point), new Vector(dir));
        }
        else
            return false;
        return axis.IsParallel(shapeAxis, tolerance);
    }

    private static bool IsParallelTo(Shape shape, Plane plane, Axis planeAxis, double tolerance)
    {
        switch (shape)
        {
            case Face face when face.IsPlanarSurface():
            {
                var (u0, _, v0, _) = face.UVBounds();
                return planeAxis.IsParallel(new Axis(shape.Center(), face.NormalAtParameters(u0, v0)), tolerance);
            }
            case Wire wire:
                return wire.Edges().All(e => IsParallelTo(e, plane, planeAxis, tolerance));
            case Edge edge:
            {
                var local = edge.Location.Inverse() * plane;
                var normal = new Direction3D(local.ZDir.X, local.ZDir.Y, local.ZDir.Z);
                return Extras.EdgeCurve3DIsPlanar(edge.TopoEdge, normal, tolerance) ?? false;
            }
            default:
                return false;
        }
    }
}
