using Kerf.Geometry;
using System.Collections;
using System.Collections.Immutable;
using System.Runtime.CompilerServices;

namespace Kerf.Modeling;

/// <summary>Builds <see cref="ShapeList{T}"/> from collection expressions.</summary>
public static class ShapeList
{
    /// <summary>A list of the given shapes.</summary>
    public static ShapeList<T> Create<T>(ReadOnlySpan<T> shapes) where T : IShape => new(ImmutableArray.Create(shapes));
}

/// <summary>
/// An immutable list of shapes. Edits return a new list. A list selected from a shape remembers it: see <see cref="Origin"/>.
/// </summary>
/// <remarks>
/// The selections suited to CAD (by axis, geometry type or measure) are <see cref="ShapeSelection"/>'s extension
/// methods on any sequence of shapes, beside LINQ's own: <c>part.Edges().Where(e =&gt; e.IsClosed).OrderBy(Axis.Z).Last()</c>.
/// </remarks>
[CollectionBuilder(typeof(ShapeList), nameof(ShapeList.Create))]
public sealed class ShapeList<T> : IReadOnlyList<T>, IEquatable<ShapeList<T>> where T : IShape
{
    private readonly ImmutableArray<T> _items;

    internal ShapeList(ImmutableArray<T> items, IShape? origin = null)
    {
        _items = items;
        Origin = origin;
    }

    /// <summary>A list of the given shapes, in order.</summary>
    public ShapeList(IEnumerable<T> shapes) => _items = [.. shapes];

    /// <summary>A list of the given shapes, in order, selected from <paramref name="origin"/>.</summary>
    internal ShapeList(IEnumerable<T> shapes, IShape? origin)
    {
        _items = [.. shapes];
        Origin = origin;
    }

    internal static ShapeList<T> From(IShape origin) => new(ImmutableArray<T>.Empty, origin);

    /// <summary>The empty list.</summary>
    public static ShapeList<T> Empty { get; } = new(ImmutableArray<T>.Empty);

    /// <summary>
    /// The shape this list was selected from, if it was: <c>part.Edges()</c>'s is the part, and every selection on the list
    /// (<see cref="Where(Func{T, bool})"/>, <c>OrderBy</c>, <c>ParallelTo</c>…) keeps it. A selection through a shape's
    /// sub-shape (<c>faces[0].SelectEdges()</c>) has that sub-shape. What needs the parent, such as a fillet or
    /// <c>Where(Convexity)</c>, reads it here.
    /// </summary>
    public IShape? Origin { get; }

    /// <summary>The number of shapes.</summary>
    public int Count => _items.Length;

    /// <summary>The shape at an index.</summary>
    public T this[int index] => _items[index];

    /// <summary>A slice of the list.</summary>
    public ShapeList<T> this[Range range]
    {
        get
        {
            var (start, length) = range.GetOffsetAndLength(Count);
            return new(_items.Slice(start, length), Origin);
        }
    }

    /// <summary>Iterates the shapes in order.</summary>
    public IEnumerator<T> GetEnumerator() => ((IEnumerable<T>)_items).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    /// <summary>The shapes that pass a test, in order, keeping the list's <see cref="Origin"/>.</summary>
    public ShapeList<T> Where(Func<T, bool> predicate) => new(Enumerable.Where(_items, predicate), Origin);

    /// <summary>This list with a shape added at the end.</summary>
    public ShapeList<T> Add(T shape) => new(_items.Add(shape), Origin);

    /// <summary>This list with shapes added at the end.</summary>
    public ShapeList<T> AddRange(IEnumerable<T> shapes) => new(_items.AddRange(shapes), Origin);

    /// <summary>The list as a list of <see cref="Shape"/>.</summary>
    public ShapeList<Shape> Upcast() => new(_items.Select(s => s.AsShape()), Origin);

    /// <summary>The shapes of another type in the list.</summary>
    public ShapeList<TOther> OfType<TOther>() where TOther : struct, IShape<TOther> =>
        new(_items.Select(s => s.AsShape()).Where(s => !s.IsNull && s.Type == TOther.Type).Select(s => s.As<TOther>()), Origin);

    /// <summary>The list with each shape's sub-shape of the shape's own type picked out: what <paramref name="select"/> gives, in order.</summary>
    internal ShapeList<TOut> Map<TOut>(Func<T, TOut> select) where TOut : IShape => new(_items.Select(select), Origin);

    // ---- Set operations ----

    /// <summary>The shapes of this list that are also in <paramref name="other"/>, once each, in this list's order.</summary>
    public ShapeList<T> Intersect(IEnumerable<T> other)
    {
        var set = other.ToHashSet(ShapeComparer<T>.Same);
        return new(Enumerable.Where(this, set.Contains).Distinct(ShapeComparer<T>.Same), Origin);
    }

    /// <summary>The shapes of this list that aren't in <paramref name="other"/>, once each, in this list's order.</summary>
    public ShapeList<T> Except(IEnumerable<T> other)
    {
        var set = other.ToHashSet(ShapeComparer<T>.Same);
        return new(Enumerable.Where(this, s => !set.Contains(s)).Distinct(ShapeComparer<T>.Same), Origin);
    }

    /// <summary>Whether two lists hold the same shapes, in any order and counting duplicates once.</summary>
    public bool Equals(ShapeList<T>? other) => other is not null && this.ToHashSet(ShapeComparer<T>.Same).SetEquals(other);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is ShapeList<T> l && Equals(l);

    /// <summary>A hash consistent with <see cref="Equals(ShapeList{T}?)"/>.</summary>
    public override int GetHashCode()
    {
        // Order-independent, to agree with Equals
        int h = 0;
        foreach (var s in this.Distinct(ShapeComparer<T>.Same))
            h ^= ShapeComparer<T>.Same.GetHashCode(s);
        return h;
    }

    /// <summary>Whether two lists hold the same shapes (see <see cref="Equals(ShapeList{T}?)"/>).</summary>
    public static bool operator ==(ShapeList<T>? a, ShapeList<T>? b) => a is null ? b is null : a.Equals(b);

    /// <summary>Whether two lists hold different shapes.</summary>
    public static bool operator !=(ShapeList<T>? a, ShapeList<T>? b) => !(a == b);

    // ---- Sub-shapes ----

    /// <summary>The list with compounds dissolved into their children, wires into edges and shells into faces, and empty shapes left out.</summary>
    public ShapeList<Shape> Expand()
    {
        var expanded = new List<Shape>();
        foreach (var item in this)
        {
            var shape = item.AsShape();
            if (TopoUtil.IsNothing(shape))
                continue;
            switch (shape.Type)
            {
                case ShapeType.Compound:
                    expanded.AddRange(new ShapeList<Shape>(shape.ChildShapes()).Expand());
                    break;
                case ShapeType.Shell:
                    expanded.AddRange(shape.SelectFaces().Select(f => (Shape)f));
                    break;
                case ShapeType.Wire:
                    expanded.AddRange(shape.SelectEdges().Select(e => (Shape)e));
                    break;
                default:
                    expanded.Add(shape);
                    break;
            }
        }
        return new(expanded);
    }

    /// <summary>The average of the centres of the shapes.</summary>
    public Point3D Center() =>
        Count == 0 ? Point3D.Origin : (Enumerable.Aggregate(this, Vector3D.Zero, (sum, o) => sum + o.Center().ToVector()) * (1.0 / Count)).ToPoint();

    /// <summary>All the vertices of the shapes.</summary>
    public ShapeList<Vertex> Vertices() => new(this.SelectMany(s => s.SelectVertices()));

    /// <summary>All the edges of the shapes.</summary>
    public ShapeList<Edge> Edges() => new(this.SelectMany(s => s.SelectEdges()));

    /// <summary>All the wires of the shapes.</summary>
    public ShapeList<Wire> Wires() => new(this.SelectMany(s => s.SelectWires()));

    /// <summary>All the faces of the shapes.</summary>
    public ShapeList<Face> Faces() => new(this.SelectMany(s => s.SelectFaces()));

    /// <summary>All the shells of the shapes.</summary>
    public ShapeList<Shell> Shells() => new(this.SelectMany(s => s.SelectShells()));

    /// <summary>All the solids of the shapes.</summary>
    public ShapeList<Solid> Solids() => new(this.SelectMany(s => s.SelectSolids()));

    /// <summary>All the compounds of the shapes.</summary>
    public ShapeList<Compound> Compounds() => new(this.SelectMany(s => s.Compounds()));

    // The shapes' sub-shapes, a selection from the one shape if there is one, else from what this list was selected from
    private ShapeList<TSub> Through<TSub>(Func<T, ShapeList<TSub>> select) where TSub : IShape =>
        new(this.SelectMany(select), Count == 1 ? _items[0] : Origin);

    private static TItem Single<TItem>(ShapeList<TItem> items, string kind) where TItem : IShape =>
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

    /// <inheritdoc/>
    public override string ToString() => "[" + string.Join(", ", _items.Select(Text)) + "]";

    private static string Text(T shape) => shape is ModelShape ? shape.ToString()! : shape.AsShape().Describe();
}

/// <summary>
/// Shapes in groups of equal key, the groups in key order: what <see cref="ShapeSelection"/>'s <c>GroupBy</c> gives.
/// A group is reached by index (<c>groups[0]</c>, <c>groups[^1]</c>, LINQ's <c>Last()</c>) or by key.
/// </summary>
public sealed class ShapeGroups<T, TKey> : IReadOnlyList<ShapeList<T>> where T : IShape
{
    private readonly List<(TKey Key, ShapeList<T> Group)> _groups = [];
    private readonly Func<T, TKey> _key;
    private readonly IComparer<TKey> _comparer;

    internal ShapeGroups(Func<T, TKey> key, IEnumerable<T> shapes, IComparer<TKey>? comparer = null)
    {
        _key = key;
        _comparer = comparer ?? KeyComparer<TKey>.Instance;
        var origin = (shapes as ShapeList<T>)?.Origin;
        var keyed = shapes.Select(s => (Key: key(s), Shape: s));
        // Consecutive runs of equal keys after a stable sort, as build123d's sorted() then itertools.groupby make them
        var run = new List<T>();
        TKey runKey = default!;
        foreach (var (k, s) in keyed.OrderBy(k => k.Key, _comparer))
        {
            if (run.Count > 0 && _comparer.Compare(runKey, k) != 0)
            {
                _groups.Add((runKey, new ShapeList<T>(run, origin)));
                run.Clear();
            }
            run.Add(s);
            runKey = k;
        }
        if (run.Count > 0)
            _groups.Add((runKey, new ShapeList<T>(run, origin)));
    }

    /// <summary>The number of groups.</summary>
    public int Count => _groups.Count;

    /// <summary>The group at an index.</summary>
    public ShapeList<T> this[int index] => _groups[index].Group;

    /// <summary>The keys of the groups, in order.</summary>
    public IReadOnlyList<TKey> Keys => [.. _groups.Select(g => g.Key)];

    /// <summary>The group with a key.</summary>
    /// <exception cref="KeyNotFoundException">No group has the key.</exception>
    public ShapeList<T> Group(TKey key)
    {
        foreach (var (k, g) in _groups)
            if (_comparer.Compare(k, key) == 0)
                return g;
        throw new KeyNotFoundException($"No group has the key {key}.");
    }

    /// <summary>The group a shape would be in.</summary>
    public ShapeList<T> GroupFor(T shape) => Group(_key(shape));

    /// <summary>Iterates the groups in order.</summary>
    public IEnumerator<ShapeList<T>> GetEnumerator() => _groups.Select(g => g.Group).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    /// <inheritdoc/>
    public override string ToString() => "[" + string.Join(", ", _groups.Select(g => g.Group)) + "]";
}

/// <summary>
/// Orders and compares the keys of sorts and groups: by the key type's own order, a null key first, and a sequence (an
/// inertia matrix, say) element by element, a sequence that is the start of a longer one first.
/// </summary>
/// <remarks>
/// It keeps the order build123d's <c>sorted()</c> gives the same keys, Python comparing tuples and lists
/// lexicographically, so groups come out as build123d's do.
/// </remarks>
internal sealed class KeyComparer<TKey> : IComparer<TKey>, IEqualityComparer<TKey>
{
    internal static readonly KeyComparer<TKey> Instance = new();

    public int Compare(TKey? x, TKey? y) => CompareObjects(x, y);

    public bool Equals(TKey? x, TKey? y) => CompareObjects(x, y) == 0;

    public int GetHashCode(TKey obj) => 0;   // only used through Equals

    private static int CompareObjects(object? x, object? y)
    {
        if (x is null || y is null)
            return x is null ? (y is null ? 0 : -1) : 1;
        if (x is not string && x is IEnumerable xs && y is IEnumerable ys)
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
    internal static double Length(Shape s)
    {
        if (s.TryAs(out Edge edge) && s.IsEdge()) return edge.Length;
        if (s.TryAs(out Wire wire) && s.IsWire()) return wire.Length;
        if (s.TryAs(out Face face) && s.IsFace()) return face.Length ?? throw new InvalidOperationException($"{face.Describe()} has no length");
        throw new InvalidOperationException($"a {TopoUtil.KindName(s)} has no length");
    }

    internal static bool HasRadius(Shape s) => s.IsEdge() || s.IsWire() || s.IsFace();

    /// <summary>A shape's length, radius, area, volume or distance from the origin, as sorting and grouping take it.</summary>
    internal static double Of(Shape o, ShapeKey measure) => measure switch
    {
        ShapeKey.Length => Length(o),
        ShapeKey.Radius => Radius(o),
        ShapeKey.Distance => o.Center().ToVector().Magnitude,
        ShapeKey.Area => o.Area,
        ShapeKey.Volume => o.Volume,
        _ => throw new ArgumentOutOfRangeException(nameof(measure)),
    };

    /// <summary>The radius, or null where the shape hasn't one (a line, a plane).</summary>
    internal static double? RadiusOrNull(Shape s)
    {
        try
        {
            return HasRadius(s) ? RadiusCore(s) : null;
        }
        catch (Exception e) when (Failures.IsKernelFailure(e) || e is InvalidOperationException or ArgumentException or NotImplementedException)
        {
            return null;
        }
    }

    private static double? RadiusCore(Shape s) =>
        s.IsEdge() ? ((Edge)s).Radius : s.IsWire() ? ((Wire)s).Radius : ((Face)s).Radius;

    internal static double Radius(Shape s)
    {
        if (!HasRadius(s))
            throw new InvalidOperationException($"a {TopoUtil.KindName(s)} has no radius");
        return RadiusCore(s) ?? throw new InvalidOperationException($"{s.Describe()} has no radius");
    }
}
