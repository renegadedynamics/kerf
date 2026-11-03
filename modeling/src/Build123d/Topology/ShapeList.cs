using System.Collections;
using System.Runtime.CompilerServices;
using Schematiq.Collections;
using Occt;

namespace Build123d;

/// <summary>Builds <see cref="ShapeList{T}"/> from collection expressions.</summary>
public static class ShapeList
{
    /// <summary>A list of the given shapes.</summary>
    public static ShapeList<T> Create<T>(ReadOnlySpan<T> shapes) where T : Shape => new(SchematiqList.Create(shapes));
}

/// <summary>
/// An immutable list of shapes, with the filtering, sorting and grouping suited to CAD.
/// </summary>
/// <remarks>
/// Operators follow build123d: <c>list &gt; Axis.Z</c> sorts, <c>list &lt; Axis.Z</c> sorts in reverse,
/// <c>list | Axis.Z</c> filters, <c>list &gt;&gt; Axis.Z</c> and <c>list &lt;&lt; Axis.Z</c> select the last and first
/// group, and <c>+</c>, <c>-</c> and <c>&amp;</c> combine lists. Edits return a new list.
/// </remarks>
[CollectionBuilder(typeof(ShapeList), nameof(ShapeList.Create))]
public sealed class ShapeList<T> : IReadOnlyList<T>, IEquatable<ShapeList<T>> where T : Shape
{
    private readonly ISchematiqList<T> _items;

    internal ShapeList(ISchematiqList<T> items) => _items = items;

    /// <summary>A list of the given shapes, in order.</summary>
    public ShapeList(IEnumerable<T> shapes) => _items = SchematiqList.Create(shapes);

    /// <summary>The empty list.</summary>
    public static ShapeList<T> Empty { get; } = new(SchematiqList.Empty<T>());

    /// <summary>The number of shapes.</summary>
    public int Count => _items.Count;

    /// <summary>The shape at an index.</summary>
    public T this[int index] => _items[index];

    /// <summary>A slice of the list.</summary>
    public ShapeList<T> this[Range range]
    {
        get
        {
            var (start, length) = range.GetOffsetAndLength(Count);
            return new(_items.Subset(start, length));
        }
    }

    /// <summary>The first shape.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The list is empty.</exception>
    public T First => _items[0];

    /// <summary>The last shape.</summary>
    public T Last => _items[Count - 1];

    /// <summary>Iterates the shapes in order.</summary>
    public IEnumerator<T> GetEnumerator() => _items.GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    /// <summary>This list with a shape added at the end.</summary>
    public ShapeList<T> Add(T shape) => new(_items.Push(shape));

    /// <summary>This list with shapes added at the end.</summary>
    public ShapeList<T> AddRange(IEnumerable<T> shapes) => new(_items.Concat(SchematiqList.Create(shapes)));

    /// <summary>The list as a list of a base class of its shapes.</summary>
    public ShapeList<TBase> Upcast<TBase>() where TBase : Shape => new(_items.View(s => (TBase)(Shape)s));

    /// <summary>The shapes of another class in the list.</summary>
    public ShapeList<TOther> OfType<TOther>() where TOther : Shape => new(Enumerable.OfType<TOther>(_items));

    // ---- Set-like operators ----

    /// <summary>This list with a shape added.</summary>
    public static ShapeList<T> operator +(ShapeList<T> a, T b) => a.Add(b);

    /// <summary>This list with shapes added.</summary>
    public static ShapeList<T> operator +(ShapeList<T> a, IEnumerable<T> b) => a.AddRange(b);

    /// <summary>The shapes of <paramref name="a"/> that are also in <paramref name="b"/>, once each.</summary>
    public static ShapeList<T> operator &(ShapeList<T> a, IEnumerable<T> b)
    {
        var other = b.ToHashSet();
        return new(a.Where(other.Contains).Distinct());
    }

    /// <summary>The shapes of <paramref name="a"/> that aren't in <paramref name="b"/>, once each.</summary>
    public static ShapeList<T> operator -(ShapeList<T> a, IEnumerable<T> b)
    {
        var other = b.ToHashSet();
        return new(a.Where(s => !other.Contains(s)).Distinct());
    }

    /// <summary>Whether two lists hold the same shapes, in any order and counting duplicates once.</summary>
    public bool Equals(ShapeList<T>? other) => other is not null && this.ToHashSet().SetEquals(other);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is ShapeList<T> l && Equals(l);

    /// <summary>A hash consistent with <see cref="Equals(ShapeList{T}?)"/>.</summary>
    public override int GetHashCode()
    {
        // Order-independent, to agree with Equals
        int h = 0;
        foreach (var s in this.Distinct())
            h ^= s.GetHashCode();
        return h;
    }

    /// <summary>Whether two lists hold the same shapes (see <see cref="Equals(ShapeList{T}?)"/>).</summary>
    public static bool operator ==(ShapeList<T>? a, ShapeList<T>? b) => a is null ? b is null : a.Equals(b);

    /// <summary>Whether two lists hold different shapes.</summary>
    public static bool operator !=(ShapeList<T>? a, ShapeList<T>? b) => !(a == b);

    // ---- Sorting, filtering and grouping operators ----

    /// <summary>The list sorted along an axis.</summary>
    public static ShapeList<T> operator >(ShapeList<T> list, Axis axis) => list.SortBy(axis);

    /// <summary>The list sorted along an axis, in reverse.</summary>
    public static ShapeList<T> operator <(ShapeList<T> list, Axis axis) => list.SortBy(axis, reverse: true);

    /// <summary>The list sorted by a measure.</summary>
    public static ShapeList<T> operator >(ShapeList<T> list, SortBy sortBy) => list.SortBy(sortBy);

    /// <summary>The list sorted by a measure, in reverse.</summary>
    public static ShapeList<T> operator <(ShapeList<T> list, SortBy sortBy) => list.SortBy(sortBy, reverse: true);

    /// <summary>The shapes parallel to an axis.</summary>
    public static ShapeList<T> operator |(ShapeList<T> list, Axis axis) => list.FilterBy(axis);

    /// <summary>The shapes parallel to a plane.</summary>
    public static ShapeList<T> operator |(ShapeList<T> list, Plane plane) => list.FilterBy(plane);

    /// <summary>The shapes of a geometry type.</summary>
    public static ShapeList<T> operator |(ShapeList<T> list, GeomType geomType) => list.FilterBy(geomType);

    /// <summary>The shapes of a convexity.</summary>
    public static ShapeList<T> operator |(ShapeList<T> list, Convexity convexity) => list.FilterBy(convexity);

    /// <summary>The last (largest) group.</summary>
    public static ShapeList<T> operator >>(ShapeList<T> list, Axis axis) => list.GroupBy(axis).Last;

    /// <summary>The first (smallest) group.</summary>
    public static ShapeList<T> operator <<(ShapeList<T> list, Axis axis) => list.GroupBy(axis).First;

    /// <summary>The last (largest) group by a measure.</summary>
    public static ShapeList<T> operator >>(ShapeList<T> list, SortBy sortBy) => list.GroupBy(sortBy).Last;

    /// <summary>The first (smallest) group by a measure.</summary>
    public static ShapeList<T> operator <<(ShapeList<T> list, SortBy sortBy) => list.GroupBy(sortBy).First;

    // ---- Sub-shapes ----

    /// <summary>The list with compounds dissolved into their children, wires into edges and shells into faces, and empty shapes left out.</summary>
    public ShapeList<Shape> Expand()
    {
        var expanded = new List<Shape>();
        foreach (var shape in this)
        {
            switch (shape.TopoShape.ShapeType())
            {
                case TopAbs_ShapeEnum.TopAbs_COMPOUND:
                    expanded.AddRange(new ShapeList<Shape>(shape is Compound c ? c.Children() : []).Expand());
                    break;
                case TopAbs_ShapeEnum.TopAbs_SHELL:
                    expanded.AddRange(shape.Faces());
                    break;
                case TopAbs_ShapeEnum.TopAbs_WIRE:
                    expanded.AddRange(shape.Edges());
                    break;
                default:
                    if (!shape.IsEmpty)
                        expanded.Add(shape);
                    break;
            }
        }
        return new(expanded);
    }

    /// <summary>The average of the centres of the shapes.</summary>
    public Vector Center() =>
        Count == 0 ? new Vector(0, 0, 0) : this.Aggregate(new Vector(0, 0, 0), (sum, o) => sum + o.Center()) / Count;

    /// <summary>All the vertices of the shapes.</summary>
    public ShapeList<Vertex> Vertices() => new(this.SelectMany(s => s.Vertices()));

    /// <summary>All the edges of the shapes.</summary>
    public ShapeList<Edge> Edges() => new(this.SelectMany(s => s.Edges()));

    /// <summary>All the wires of the shapes.</summary>
    public ShapeList<Wire> Wires() => new(this.SelectMany(s => s.Wires()));

    /// <summary>All the faces of the shapes.</summary>
    public ShapeList<Face> Faces() => new(this.SelectMany(s => s.Faces()));

    /// <summary>All the shells of the shapes.</summary>
    public ShapeList<Shell> Shells() => new(this.SelectMany(s => s.Shells()));

    /// <summary>All the solids of the shapes.</summary>
    public ShapeList<Solid> Solids() => new(this.SelectMany(s => s.Solids()));

    /// <summary>All the compounds of the shapes.</summary>
    public ShapeList<Compound> Compounds() => new(this.SelectMany(s => s.Compounds()));

    private static TItem Single<TItem>(ShapeList<TItem> items, string kind) where TItem : Shape =>
        items.Count == 1 ? items[0] : throw new InvalidOperationException($"Expected exactly one {kind}, found {items.Count}");

    /// <summary>The one vertex of the shapes.</summary>
    public Vertex SingleVertex() => Single(Vertices(), "vertex");

    /// <summary>The one edge of the shapes.</summary>
    public Edge SingleEdge() => Single(Edges(), "edge");

    /// <summary>The one wire of the shapes.</summary>
    public Wire SingleWire() => Single(Wires(), "wire");

    /// <summary>The one face of the shapes.</summary>
    public Face SingleFace() => Single(Faces(), "face");

    /// <summary>The one shell of the shapes.</summary>
    public Shell SingleShell() => Single(Shells(), "shell");

    /// <summary>The one solid of the shapes.</summary>
    public Solid SingleSolid() => Single(Solids(), "solid");

    /// <summary>The one compound of the shapes.</summary>
    public Compound SingleCompound() => Single(Compounds(), "compound");

    // ---- Filtering ----

    /// <summary>The shapes for which <paramref name="predicate"/> holds (or doesn't, with <paramref name="reverse"/>).</summary>
    public ShapeList<T> FilterBy(Func<T, bool> predicate, bool reverse = false) =>
        new(this.Where(s => predicate(s) != reverse));

    /// <summary>
    /// The planar faces whose normal, and the linear edges whose direction, is parallel to <paramref name="axis"/>
    /// within <paramref name="tolerance"/> radians.
    /// </summary>
    public ShapeList<T> FilterBy(Axis axis, bool reverse = false, double tolerance = 1e-5) =>
        FilterBy(s => IsParallelTo(s, axis, tolerance), reverse);

    /// <summary>The faces, wires and edges parallel to <paramref name="plane"/>.</summary>
    public ShapeList<T> FilterBy(Plane plane, bool reverse = false, double tolerance = 1e-5)
    {
        var planeAxis = new Axis(plane.Origin, plane.ZDir);
        return FilterBy(s => IsParallelTo(s, plane, planeAxis, tolerance), reverse);
    }

    /// <summary>The shapes of a geometry type.</summary>
    public ShapeList<T> FilterBy(GeomType geomType, bool reverse = false) => FilterBy(s => s.GeomType == geomType, reverse);

    /// <summary>The shapes of a convexity relative to the shape each was selected from.</summary>
    public ShapeList<T> FilterBy(Convexity convexity, bool reverse = false) => FilterBy(s => s.Convexity == convexity, reverse);

    /// <summary>
    /// The shapes for which a true-or-false property, named by a string, holds: <c>FilterBy("IsClosed")</c> for
    /// build123d's <c>filter_by(Edge.is_closed)</c> (see <see cref="ShapeProperty"/> for how names match).
    /// </summary>
    /// <exception cref="ArgumentException">A shape has no such property, or it isn't true or false.</exception>
    public ShapeList<T> FilterBy(string property, bool reverse = false) => FilterBy(s => ShapeProperty.Get(s, property) switch
    {
        bool b => b,
        var other => throw new ArgumentException(
            $"{s.GetType().Name}.{property} is {other?.GetType().Name ?? "null"}, not true or false: use FilterByValue", nameof(property)),
    }, reverse);

    private static bool IsParallelTo(Shape shape, Axis axis, double tolerance)
    {
        Axis shapeAxis;
        if (shape is Face face && face.IsPlanarSurface())
        {
            double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
            BRepTools.UVBounds(face.TopoFace, out u0, out u1, out v0, out v1);
            shapeAxis = new Axis(shape.Center(), face.NormalAtParameters(u0, v0));
        }
        else if (shape is Edge edge && shape.GeomType == GeomType.Line)
        {
            var curve = edge.GeomAdaptor();
            var pnt = new gp_Pnt();
            var d1 = new gp_Vec();
            curve.D1(curve.FirstParameter(), out pnt, out d1);
            var dir = new gp_Dir(d1);
            shapeAxis = new Axis(new Vector(pnt), new Vector(dir));
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
                double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
                BRepTools.UVBounds(face.TopoFace, out u0, out u1, out v0, out v1);
                return planeAxis.IsParallel(new Axis(shape.Center(), face.NormalAtParameters(u0, v0)), tolerance);
            }
            case Wire wire:
                return wire.Edges().All(e => IsParallelTo(e, plane, planeAxis, tolerance));
            case Edge edge:
            {
                var local = edge.Location.Inverse() * plane;
                var normal = local.ZDir.ToGpXyz();
                return Extras.EdgeCurve3DIsPlanar(edge.TopoEdge, normal, tolerance) ?? false;
            }
            default:
                return false;
        }
    }

    /// <summary>
    /// The shapes whose centre lies between <paramref name="minimum"/> and <paramref name="maximum"/> along
    /// <paramref name="axis"/>, sorted along it. <paramref name="inclusive"/> says whether each end is included.
    /// </summary>
    public ShapeList<T> FilterByPosition(Axis axis, double minimum, double maximum, (bool Minimum, bool Maximum)? inclusive = null)
    {
        var (incMin, incMax) = inclusive ?? (true, true);
        var plane = new Plane(axis);
        return FilterBy(o =>
        {
            var z = plane.ToLocalCoords(o).Center().Z;
            return (incMin ? minimum <= z : minimum < z) && (incMax ? z <= maximum : z < maximum);
        }).SortBy(axis);
    }

    /// <summary>
    /// The shapes whose <paramref name="measure"/> is within <paramref name="tolerance"/> of <paramref name="value"/>, such as
    /// the edges of a given length or faces of a given radius. When both are numbers a shape is kept if
    /// |measure − value| ≤ tolerance; otherwise (a GeomType, a Convexity) if they are equal. A shape whose measure is null
    /// (the radius of a planar face) never matches, so <paramref name="reverse"/> keeps it.
    /// </summary>
    public ShapeList<T> FilterByValue<TValue>(Func<T, TValue> measure, TValue value, double tolerance = Constants.Tolerance, bool reverse = false)
    {
        bool Matches(T o)
        {
            var measured = measure(o);
            if (measured is null)
                return false;
            if (AsDouble(measured) is { } m && AsDouble(value) is { } v)
                return Math.Abs(m - v) <= tolerance;
            return EqualityComparer<TValue>.Default.Equals(measured, value);
        }
        return new(this.Where(o => Matches(o) != reverse));
    }

    private static double? AsDouble(object? x) => x switch
    {
        double d => d,
        float f => f,
        int i => i,
        long l => l,
        _ => null,
    };

    /// <summary>
    /// The shapes whose property, named by a string, is <paramref name="value"/>: <c>FilterByValue("Radius", 5)</c>
    /// for build123d's <c>filter_by_value(Face.radius, 5)</c>. Numbers match within <paramref name="tolerance"/>; an
    /// enum matches its value or its name (<c>FilterByValue("GeomType", "Plane")</c>); a missing (null) measure never
    /// matches.
    /// </summary>
    /// <exception cref="ArgumentException">A shape has no such property.</exception>
    public ShapeList<T> FilterByValue(string measure, object? value, double tolerance = Constants.Tolerance, bool reverse = false) =>
        new(this.Where(o => ShapeProperty.Matches(ShapeProperty.Get(o, measure), value, tolerance) != reverse));

    /// <summary>The shapes whose length, radius, area, volume or distance from the origin is <paramref name="value"/>.</summary>
    public ShapeList<T> FilterByValue(SortBy measure, double value, double tolerance = Constants.Tolerance, bool reverse = false) =>
        measure == Build123d.SortBy.Radius
            ? new ShapeList<T>(this.Where(Measures.HasRadius)).FilterByValue(o => (double?)Measures.RadiusOrNull(o), value, tolerance, reverse)
            : FilterByValue(o => Measures.Of(o, measure), value, tolerance, reverse);

    // ---- Sorting ----

    /// <summary>The shapes sorted by a key.</summary>
    public ShapeList<T> SortBy<TKey>(Func<T, TKey> key, bool reverse = false) =>
        new(reverse ? this.OrderByDescending(key) : this.OrderBy(key));

    /// <summary>The shapes sorted by the position of their centres along <paramref name="axis"/>.</summary>
    public ShapeList<T> SortBy(Axis axis, bool reverse = false)
    {
        var axisAsLocation = axis.Location.Inverse();
        return SortBy(o => (axisAsLocation * new Location(o.Center())).Position.Z, reverse);
    }

    /// <summary>The shapes sorted by where along an edge or wire the point closest to their centre lies.</summary>
    public ShapeList<T> SortBy(Shape1D path, bool reverse = false)
    {
        if (path.IsEmpty)
            throw new ArgumentException("Cannot sort by an empty object");
        return SortBy(o => path.ParamAtPoint(path.ClosestPoints(o.Center()).OnThis), reverse);
    }

    /// <summary>
    /// The shapes sorted by a measure. Shapes without the measure (a vertex has no radius) are left out of a sort by
    /// radius, area or volume.
    /// </summary>
    public ShapeList<T> SortBy(SortBy sortBy, bool reverse = false) => sortBy switch
    {
        Build123d.SortBy.Length => SortBy(o => Measures.Length(o), reverse),
        Build123d.SortBy.Radius => new ShapeList<T>(this.Where(Measures.HasRadius)).SortBy(o => Measures.Radius(o), reverse),
        Build123d.SortBy.Distance => SortBy(o => o.Center().Length, reverse),
        Build123d.SortBy.Area => SortBy(o => o.Area, reverse),
        Build123d.SortBy.Volume => SortBy(o => o.Volume, reverse),
        _ => throw new ArgumentOutOfRangeException(nameof(sortBy)),
    };

    /// <summary>The shapes sorted by their minimal distance to <paramref name="other"/>.</summary>
    public ShapeList<T> SortByDistance(Shape other, bool reverse = false) => SortBy(o => o.DistanceTo(other), reverse);

    /// <summary>The shapes sorted by their minimal distance to a point.</summary>
    public ShapeList<T> SortByDistance(Vector other, bool reverse = false) => SortBy(o => o.DistanceTo(other), reverse);

    /// <summary>
    /// The shapes sorted by a property named by a string: <c>SortBy("Length")</c> for build123d's
    /// <c>sort_by(Edge.length)</c>. Values compare as Python's do; a missing (null) one sorts first.
    /// </summary>
    /// <exception cref="ArgumentException">A shape has no such property.</exception>
    public ShapeList<T> SortBy(string property, bool reverse = false)
    {
        var keyed = this.Select(o => (Key: ShapeProperty.Get(o, property), Shape: o)).ToList();
        var order = PyComparer<object?>.Instance;
        return new((reverse ? keyed.OrderByDescending(k => k.Key, order) : keyed.OrderBy(k => k.Key, order)).Select(k => k.Shape));
    }

    // ---- Grouping ----

    /// <summary>
    /// The shapes grouped by a key, the groups in key order. Keys compare as Python's do: a numeric key rounded to
    /// <paramref name="tolDigits"/> places (see the double overload), and sequences element by element.
    /// </summary>
    public GroupBy<T, TKey> GroupBy<TKey>(Func<T, TKey> key, bool reverse = false, int tolDigits = 6) => new(key, this, reverse);

    /// <summary>The shapes grouped by a numeric key rounded to <paramref name="tolDigits"/> decimal places.</summary>
    public GroupBy<T, double> GroupBy(Func<T, double> key, bool reverse = false, int tolDigits = 6) =>
        new(o => Math.Round(key(o), tolDigits), this, reverse);

    /// <summary>The shapes grouped by the position of their centres along <paramref name="axis"/>.</summary>
    public GroupBy<T, double> GroupBy(Axis axis, bool reverse = false, int tolDigits = 6)
    {
        var axisAsLocation = axis.Location.Inverse();
        return GroupBy(o => (axisAsLocation * new Location(o.Center())).Position.Z, reverse, tolDigits);
    }

    /// <summary>The shapes grouped by where along an edge or wire the point closest to their centre lies.</summary>
    public GroupBy<T, double> GroupBy(Shape1D path, bool reverse = false, int tolDigits = 6)
    {
        if (path.IsEmpty)
            throw new ArgumentException("Cannot group by an empty object");
        return GroupBy(o => path.ParamAtPoint(path.ClosestPoints(o.Center()).OnThis), reverse, tolDigits);
    }

    /// <summary>The shapes grouped by a measure.</summary>
    public GroupBy<T, double> GroupBy(SortBy groupBy, bool reverse = false, int tolDigits = 6) => groupBy switch
    {
        Build123d.SortBy.Length => GroupBy(o => Measures.Length(o), reverse, tolDigits),
        Build123d.SortBy.Radius => GroupBy(o => Measures.Radius(o), reverse, tolDigits),
        Build123d.SortBy.Distance => GroupBy(o => o.Center().Length, reverse, tolDigits),
        Build123d.SortBy.Area => GroupBy(o => o.Area, reverse, tolDigits),
        Build123d.SortBy.Volume => GroupBy(o => o.Volume, reverse, tolDigits),
        _ => throw new ArgumentOutOfRangeException(nameof(groupBy)),
    };

    /// <summary>The shapes grouped by their convexity, in the enum's order.</summary>
    public GroupBy<T, Convexity> GroupByConvexity(bool reverse = false) => new(o => o.Convexity, this, reverse);

    /// <summary>
    /// The shapes grouped by a property named by a string: <c>GroupBy("Area")</c> for build123d's
    /// <c>group_by(Face.area)</c>. A numeric key is rounded to <paramref name="tolDigits"/> places.
    /// </summary>
    /// <exception cref="ArgumentException">A shape has no such property.</exception>
    public GroupBy<T, object?> GroupBy(string property, bool reverse = false, int tolDigits = 6) =>
        new(o => ShapeProperty.Key(o, property, tolDigits), this, reverse);

    /// <inheritdoc/>
    public override string ToString() => "[" + string.Join(", ", this) + "]";
}

/// <summary>
/// The result of <see cref="ShapeList{T}.GroupBy{TKey}(Func{T, TKey}, bool, int)"/>: shapes in groups of equal key, the
/// groups sorted by key. Groups can be reached by index or by key.
/// </summary>
public sealed class GroupBy<T, TKey> : IReadOnlyList<ShapeList<T>> where T : Shape
{
    private readonly List<(TKey Key, ShapeList<T> Group)> _groups = [];
    private readonly Func<T, TKey> _key;

    internal GroupBy(Func<T, TKey> key, IEnumerable<T> shapes, bool reverse)
    {
        _key = key;
        var keyed = shapes.Select(s => (Key: key(s), Shape: s));
        var order = PyComparer<TKey>.Instance;
        var sorted = reverse ? keyed.OrderByDescending(k => k.Key, order) : keyed.OrderBy(k => k.Key, order);
        // Consecutive runs of equal keys, as itertools.groupby makes them
        var comparer = order;
        foreach (var (k, s) in sorted)
        {
            if (_groups.Count > 0 && comparer.Equals(_groups[^1].Key, k))
                _groups[^1] = (k, _groups[^1].Group.Add(s));
            else
                _groups.Add((k, [s]));
        }
    }

    /// <summary>The number of groups.</summary>
    public int Count => _groups.Count;

    /// <summary>The group at an index.</summary>
    public ShapeList<T> this[int index] => _groups[index].Group;

    /// <summary>The first group.</summary>
    public ShapeList<T> First => _groups[0].Group;

    /// <summary>The last group.</summary>
    public ShapeList<T> Last => _groups[^1].Group;

    /// <summary>The keys of the groups, in order.</summary>
    public IEnumerable<TKey> Keys => _groups.Select(g => g.Key);

    /// <summary>The group with a key.</summary>
    /// <exception cref="KeyNotFoundException">No group has the key.</exception>
    public ShapeList<T> Group(TKey key)
    {
        foreach (var (k, g) in _groups)
            if (PyComparer<TKey>.Instance.Equals(k, key))
                return g;
        throw new KeyNotFoundException(key?.ToString());
    }

    /// <summary>The group a shape would be in.</summary>
    public ShapeList<T> GroupFor(T shape) => Group(_key(shape));

    /// <summary>Iterates the groups in order.</summary>
    public IEnumerator<ShapeList<T>> GetEnumerator() => _groups.Select(g => g.Group).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    /// <inheritdoc/>
    public override string ToString() => "[" + string.Join(", ", this) + "]";
}

/// <summary>
/// Orders and compares keys as Python does: by their own ordering, and sequences (an inertia matrix, say) element by
/// element.
/// </summary>
internal sealed class PyComparer<TKey> : IComparer<TKey>, IEqualityComparer<TKey>
{
    internal static readonly PyComparer<TKey> Instance = new();

    public int Compare(TKey? x, TKey? y) => CompareObjects(x, y);

    public bool Equals(TKey? x, TKey? y) => CompareObjects(x, y) == 0;

    public int GetHashCode(TKey obj) => 0;   // only used through Equals

    private static int CompareObjects(object? x, object? y)
    {
        if (x is null || y is null)
            return x is null ? (y is null ? 0 : -1) : 1;
        if (x is not string && x is System.Collections.IEnumerable xs && y is System.Collections.IEnumerable ys)
        {
            var a = xs.GetEnumerator();
            var b = ys.GetEnumerator();
            while (true)
            {
                bool moreA = a.MoveNext(), moreB = b.MoveNext();
                if (!moreA || !moreB)
                    return moreA == moreB ? 0 : moreA ? 1 : -1;
                var c = CompareObjects(a.Current, b.Current);
                if (c != 0)
                    return c;
            }
        }
        return Comparer<object>.Default.Compare(x, y);
    }
}

/// <summary>The measures that only some kinds of shape have, for sorting and grouping by them.</summary>
internal static class Measures
{
    internal static double Length(Shape s) => s switch
    {
        Shape1D one => one.Length,
        Face face => face.Length ?? throw new InvalidOperationException($"{face} has no length"),
        _ => throw new InvalidOperationException($"a {s.GetType().Name} has no length"),
    };

    internal static bool HasRadius(Shape s) => s is Shape1D or Face;

    /// <summary>A shape's length, radius, area, volume or distance from the origin, as sorting and grouping take it.</summary>
    internal static double Of(Shape o, SortBy measure) => measure switch
    {
        SortBy.Length => Length(o),
        SortBy.Radius => Radius(o),
        SortBy.Distance => o.Center().Length,
        SortBy.Area => o.Area,
        SortBy.Volume => o.Volume,
        _ => throw new ArgumentOutOfRangeException(nameof(measure)),
    };

    /// <summary>The radius, or null where the shape hasn't one (a line, a plane).</summary>
    internal static double? RadiusOrNull(Shape s)
    {
        try
        {
            return s switch
            {
                Shape1D one => one.Radius,
                Face face => face.Radius,
                _ => null,
            };
        }
        catch (Exception e) when (e is InvalidOperationException or ArgumentException or Standard_Failure or NotImplementedException)
        {
            return null;
        }
    }

    internal static double Radius(Shape s) => s switch
    {
        Shape1D one => one.Radius,
        Face face => face.Radius ?? throw new InvalidOperationException($"{face} has no radius"),
        _ => throw new InvalidOperationException($"a {s.GetType().Name} has no radius"),
    };
}
