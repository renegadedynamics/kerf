using Kerf.Geometry;
using System.Collections;
using System.Collections.Immutable;
using System.Runtime.CompilerServices;

namespace Kerf.Modeling;

/// <summary>Builds <see cref="ShapeList{T}"/> from collection expressions.</summary>
public static class ShapeList
{
    /// <summary>A list of the given shapes.</summary>
    public static ShapeList<T> Create<T>(ReadOnlySpan<T> shapes) where T : Shape => new(ImmutableArray.Create(shapes));
}

/// <summary>
/// An immutable list of shapes. Edits return a new list.
/// </summary>
/// <remarks>
/// The selections suited to CAD (by axis, geometry type or measure) are <see cref="ShapeSelection"/>'s extension
/// methods on any sequence of shapes, beside LINQ's own: <c>part.Edges().Where(e =&gt; e.IsClosed).OrderBy(Axis.Z).Last()</c>.
/// </remarks>
[CollectionBuilder(typeof(ShapeList), nameof(ShapeList.Create))]
public sealed class ShapeList<T> : IReadOnlyList<T>, IEquatable<ShapeList<T>> where T : Shape
{
    private readonly ImmutableArray<T> _items;

    internal ShapeList(ImmutableArray<T> items) => _items = items;

    /// <summary>A list of the given shapes, in order.</summary>
    public ShapeList(IEnumerable<T> shapes) => _items = [.. shapes];

    /// <summary>The empty list.</summary>
    public static ShapeList<T> Empty { get; } = new(ImmutableArray<T>.Empty);

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
            return new(_items.Slice(start, length));
        }
    }

    /// <summary>Iterates the shapes in order.</summary>
    public IEnumerator<T> GetEnumerator() => ((IEnumerable<T>)_items).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    /// <summary>This list with a shape added at the end.</summary>
    public ShapeList<T> Add(T shape) => new(_items.Add(shape));

    /// <summary>This list with shapes added at the end.</summary>
    public ShapeList<T> AddRange(IEnumerable<T> shapes) => new(_items.AddRange(shapes));

    /// <summary>The list as a list of a base class of its shapes.</summary>
    public ShapeList<TBase> Upcast<TBase>() where TBase : Shape
    {
        // Sharing the array where it is one of TBase already (arrays of classes are covariant, and this one is never written)
        ImmutableArray<TBase> shared = _items.As<TBase>();
        return new(shared.IsDefault ? [.. _items.Select(s => (TBase)(Shape)s)] : shared);
    }

    /// <summary>The shapes of another class in the list.</summary>
    public ShapeList<TOther> OfType<TOther>() where TOther : Shape => new(Enumerable.OfType<TOther>(this));

    // ---- Set operations ----

    /// <summary>The shapes of this list that are also in <paramref name="other"/>, once each, in this list's order.</summary>
    public ShapeList<T> Intersect(IEnumerable<T> other)
    {
        var set = other.ToHashSet();
        return new(Enumerable.Where(this, set.Contains).Distinct());
    }

    /// <summary>The shapes of this list that aren't in <paramref name="other"/>, once each, in this list's order.</summary>
    public ShapeList<T> Except(IEnumerable<T> other)
    {
        var set = other.ToHashSet();
        return new(Enumerable.Where(this, s => !set.Contains(s)).Distinct());
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

    // ---- Sub-shapes ----

    /// <summary>The list with compounds dissolved into their children, wires into edges and shells into faces, and empty shapes left out.</summary>
    public ShapeList<Shape> Expand()
    {
        var expanded = new List<Shape>();
        foreach (var shape in this)
        {
            switch (shape.TopoShape.Type)
            {
                case global::Kerf.Topology.ShapeType.Compound:
                    expanded.AddRange(new ShapeList<Shape>(shape is Compound c ? c.Children() : []).Expand());
                    break;
                case global::Kerf.Topology.ShapeType.Shell:
                    expanded.AddRange(shape.Faces());
                    break;
                case global::Kerf.Topology.ShapeType.Wire:
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

    /// <inheritdoc/>
    public override string ToString() => "[" + string.Join(", ", this) + "]";
}

/// <summary>
/// Shapes in groups of equal key, the groups in key order: what <see cref="ShapeSelection"/>'s <c>GroupBy</c> gives.
/// A group is reached by index (<c>groups[0]</c>, <c>groups[^1]</c>, LINQ's <c>Last()</c>) or by key.
/// </summary>
public sealed class ShapeGroups<T, TKey> : IReadOnlyList<ShapeList<T>> where T : Shape
{
    private readonly List<(TKey Key, ShapeList<T> Group)> _groups = [];
    private readonly Func<T, TKey> _key;
    private readonly IComparer<TKey> _comparer;

    internal ShapeGroups(Func<T, TKey> key, IEnumerable<T> shapes, IComparer<TKey>? comparer = null)
    {
        _key = key;
        _comparer = comparer ?? KeyComparer<TKey>.Instance;
        var keyed = shapes.Select(s => (Key: key(s), Shape: s));
        // Consecutive runs of equal keys after a stable sort, as build123d's sorted() then itertools.groupby make them
        var run = new List<T>();
        TKey runKey = default!;
        foreach (var (k, s) in keyed.OrderBy(k => k.Key, _comparer))
        {
            if (run.Count > 0 && _comparer.Compare(runKey, k) != 0)
            {
                _groups.Add((runKey, new ShapeList<T>(run)));
                run.Clear();
            }
            run.Add(s);
            runKey = k;
        }
        if (run.Count > 0)
            _groups.Add((runKey, new ShapeList<T>(run)));
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
    public override string ToString() => "[" + string.Join(", ", this) + "]";
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
    internal static double Length(Shape s) => s switch
    {
        Shape1D one => one.Length,
        Face face => face.Length ?? throw new InvalidOperationException($"{face} has no length"),
        _ => throw new InvalidOperationException($"a {s.GetType().Name} has no length"),
    };

    internal static bool HasRadius(Shape s) => s is Shape1D or Face;

    /// <summary>A shape's length, radius, area, volume or distance from the origin, as sorting and grouping take it.</summary>
    internal static double Of(Shape o, ShapeKey measure) => measure switch
    {
        ShapeKey.Length => Length(o),
        ShapeKey.Radius => Radius(o),
        ShapeKey.Distance => o.Center().Length,
        ShapeKey.Area => o.Area,
        ShapeKey.Volume => o.Volume,
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
        catch (Exception e) when (Failures.IsKernelFailure(e) || e is InvalidOperationException or ArgumentException or NotImplementedException)
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
