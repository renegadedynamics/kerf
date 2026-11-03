using Occt;

namespace Build123d;

/// <summary>
/// A selection written as plain values: the faces, edges or vertices of a shape, narrowed by a few filters and
/// groups, such as <c>Faces().FilterBy(GeomType.Plane).GroupBy(Axis.Z).Last</c>. It can be kept (in a cell, say) and
/// applied again to a changed model, where a picked face itself would be lost.
/// </summary>
/// <remarks>
/// <see cref="For"/> finds selectors for a face, edge or vertex picked in a viewer, simplest first. Each is checked
/// against the shape it was found for: applied to it, it gives exactly the picked sub-shape.
/// </remarks>
public sealed class Selector : IEquatable<Selector>
{
    /// <summary>A selector of <paramref name="of"/> (faces, edges or vertices) narrowed by <paramref name="steps"/>.</summary>
    /// <exception cref="ArgumentException"><paramref name="of"/> isn't Face, Edge or Vertex.</exception>
    public Selector(ShapeType of, IEnumerable<Step> steps)
    {
        if (of is not (ShapeType.Face or ShapeType.Edge or ShapeType.Vertex))
            throw new ArgumentException($"A selector selects faces, edges or vertices, not {of}", nameof(of));
        Of = of;
        Steps = [.. steps];
    }

    /// <summary>What is selected: faces, edges or vertices.</summary>
    public ShapeType Of { get; }

    /// <summary>The filters and groups, in order.</summary>
    public IReadOnlyList<Step> Steps { get; }

    /// <summary>The sub-shapes of <paramref name="shape"/> this selects.</summary>
    public ShapeList<Shape> Apply(Shape shape)
    {
        var selected = Of switch
        {
            ShapeType.Face => shape.Faces().Upcast<Shape>(),
            ShapeType.Edge => shape.Edges().Upcast<Shape>(),
            _ => shape.Vertices().Upcast<Shape>(),
        };
        foreach (var step in Steps)
            selected = step.Apply(selected);
        return selected;
    }

    /// <summary>The selector as C#: <c>Faces().GroupBy(Axis.Z).Last</c>.</summary>
    public override string ToString() =>
        (Of switch { ShapeType.Face => "Faces()", ShapeType.Edge => "Edges()", _ => "Vertices()" }) + string.Concat(Steps.Select(s => "." + s));

    /// <summary>The selector as build123d's Python: <c>faces().group_by(Axis.Z)[-1]</c>.</summary>
    public string ToPython() =>
        (Of switch { ShapeType.Face => "faces()", ShapeType.Edge => "edges()", _ => "vertices()" }) + string.Concat(Steps.Select(s => s.ToPython()));

    /// <inheritdoc/>
    public bool Equals(Selector? other) => other is not null && Of == other.Of && Steps.SequenceEqual(other.Steps);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Selector s && Equals(s);

    /// <inheritdoc/>
    public override int GetHashCode() => Steps.Aggregate(Of.GetHashCode(), HashCode.Combine);

    // ---- Steps ----

    /// <summary>One filter or group of a <see cref="Selector"/>.</summary>
    public abstract record Step
    {
        private protected Step() { }

        internal abstract ShapeList<Shape> Apply(ShapeList<Shape> shapes);

        internal abstract string ToPython();

        private protected static string Name(Axis axis) =>
            axis.Equals(Axis.X) ? "Axis.X" : axis.Equals(Axis.Y) ? "Axis.Y" : axis.Equals(Axis.Z) ? "Axis.Z" : axis.ToString();
    }

    /// <summary>The shapes of a geometry type: <c>FilterBy(GeomType.Plane)</c>.</summary>
    public sealed record FilterByGeomType(GeomType GeomType) : Step
    {
        internal override ShapeList<Shape> Apply(ShapeList<Shape> shapes) => shapes.FilterBy(GeomType);

        /// <inheritdoc/>
        public override string ToString() => $"FilterBy(GeomType.{GeomType})";

        internal override string ToPython() => $".filter_by(GeomType.{PyName(GeomType.ToString())})";
    }

    /// <summary>The planar faces or linear edges parallel to an axis: <c>FilterBy(Axis.Z)</c>.</summary>
    public sealed record FilterByAxis(Axis Axis) : Step
    {
        internal override ShapeList<Shape> Apply(ShapeList<Shape> shapes) => shapes.FilterBy(Axis);

        /// <inheritdoc/>
        public override string ToString() => $"FilterBy({Name(Axis)})";

        internal override string ToPython() => $".filter_by({Name(Axis)})";
    }

    /// <summary>
    /// A group along an axis, by its index from the start (0 is the first) or, if negative, from the end (-1 is the
    /// last): <c>GroupBy(Axis.Z).Last</c>, <c>GroupBy(Axis.X)[2]</c>. An index past the groups selects nothing.
    /// </summary>
    public sealed record GroupByAxis(Axis Axis, int Index) : Step
    {
        internal override ShapeList<Shape> Apply(ShapeList<Shape> shapes) => Pick(shapes.GroupBy(Axis), Index);

        /// <inheritdoc/>
        public override string ToString() => $"GroupBy({Name(Axis)}){Indexed(Index)}";

        internal override string ToPython() => $".group_by({Name(Axis)})[{Index}]";
    }

    /// <summary>
    /// A group by a measure, indexed as <see cref="GroupByAxis"/> is: <c>GroupBy(SortBy.Area).Last</c> is the group of
    /// the largest.
    /// </summary>
    public sealed record GroupByMeasure(SortBy Measure, int Index) : Step
    {
        internal override ShapeList<Shape> Apply(ShapeList<Shape> shapes) => Pick(shapes.GroupBy(Measure), Index);

        /// <inheritdoc/>
        public override string ToString() => $"GroupBy(SortBy.{Measure}){Indexed(Index)}";

        internal override string ToPython() => $".group_by(SortBy.{PyName(Measure.ToString())})[{Index}]";
    }

    private static ShapeList<Shape> Pick<TKey>(GroupBy<Shape, TKey> groups, int index)
    {
        var at = index < 0 ? groups.Count + index : index;
        return at >= 0 && at < groups.Count ? groups[at] : [];
    }

    private static string Indexed(int index) => index switch
    {
        0 => ".First",
        -1 => ".Last",
        < 0 => $"[^{-index}]",
        _ => $"[{index}]",
    };

    private static string PyName(string name) =>
        string.Concat(name.Select((c, i) => i > 0 && char.IsUpper(c) ? "_" + c : c.ToString())).ToUpperInvariant();

    // ---- Finding selectors ----

    /// <summary>
    /// Selectors that pick exactly <paramref name="picked"/>, a face, edge or vertex of <paramref name="shape"/>: up to
    /// <paramref name="count"/>, simplest and most robust first. Fewer steps come first; then groups at the ends of
    /// the model's axes, filters by type and direction, the smallest or largest of a measure, and groups further in.
    /// Empty if nothing of up to <paramref name="maxSteps"/> steps picks it alone.
    /// </summary>
    /// <exception cref="ArgumentException"><paramref name="picked"/> isn't a face, edge or vertex of the shape.</exception>
    public static IReadOnlyList<Selector> For(Shape shape, Shape picked, int count = 3, int maxSteps = 3)
    {
        var of = picked switch
        {
            Face => ShapeType.Face,
            Edge => ShapeType.Edge,
            Vertex => ShapeType.Vertex,
            _ => throw new ArgumentException($"Only a face, edge or vertex can be picked, not a {picked.GetType().Name}", nameof(picked)),
        };
        var all = new Selector(of, []).Apply(shape);
        int target = -1;
        for (int i = 0; i < all.Count && target < 0; i++)
            if (all[i].TopoShape.IsSame(picked.TopoShape))
                target = i;
        if (target < 0)
            throw new ArgumentException($"The {of.ToString().ToLowerInvariant()} isn't one of the shape's", nameof(picked));

        var found = new List<Selector>();
        foreach (var steps in new Search(all, target).Run(maxSteps))
        {
            var selector = new Selector(of, steps);
            // The search models the selectors; applying one is the proof
            var result = selector.Apply(shape);
            if (result.Count == 1 && result[0].TopoShape.IsSame(picked.TopoShape))
                found.Add(selector);
            if (found.Count == count)
                break;
        }
        return found;
    }

    /// <summary>
    /// A breadth-first search over sets of the candidates holding the target. Each step's effect is worked out from
    /// keys measured once, rather than by running the selectors themselves.
    /// </summary>
    private sealed class Search
    {
        private static readonly Axis[] Axes = [Axis.Z, Axis.X, Axis.Y];
        private static readonly SortBy[] MeasureOrder = [SortBy.Area, SortBy.Length, SortBy.Radius, SortBy.Volume, SortBy.Distance];

        private readonly int _n;
        private readonly int _target;
        // Each move narrows a set to the part holding the target, at a cost, or doesn't apply
        private readonly List<Func<int[], (Step Step, int Cost, int[] Set)?>> _moves = [];

        internal Search(ShapeList<Shape> all, int target)
        {
            _n = all.Count;
            _target = target;
            // Groups along the model's axes: the most robust as a model changes, the outermost most of all
            foreach (var axis in Axes)
            {
                var axisAsLocation = axis.Location.Inverse();
                var key = all.Select(o => (double?)Math.Round((axisAsLocation * new Location(o.Center())).Position.Z, 6)).ToArray();
                _moves.Add(set => Group(set, key, index => new GroupByAxis(axis, index), extremeCost: 0, innerCost: 3, inner: true));
            }
            // Filters by type and direction
            var types = all.Select(o => Try(() => o.GeomType)).ToArray();
            if (types[target] is { } type)
                _moves.Add(set => Filter(set, i => types[i] == type, new FilterByGeomType(type), 1));
            foreach (var axis in Axes)
            {
                var parallel = all.FilterBy(axis).ToHashSet(ReferenceEqualityComparer.Instance);
                if (parallel.Contains(all[target]))
                {
                    var flags = all.Select(o => parallel.Contains(o)).ToArray();
                    _moves.Add(set => Filter(set, i => flags[i], new FilterByAxis(axis), 1));
                }
            }
            // The smallest or largest of a measure, where every candidate has it
            foreach (var measure in MeasureOrder)
            {
                var key = all.Select(o => Try(() => Math.Round(Measures.Of(o, measure), 6))).ToArray();
                if (key[target] is null)
                    continue;
                int cost = measure == SortBy.Distance ? 5 : 2;
                _moves.Add(set => Group(set, key, index => new GroupByMeasure(measure, index), extremeCost: cost, innerCost: 0, inner: false));
            }
        }

        /// <summary>
        /// The step lists that leave only the target: fewest steps first, then cheapest, leaving out any that holds a
        /// simpler one (a filter that changes nothing for the target's sake).
        /// </summary>
        internal IEnumerable<IReadOnlyList<Step>> Run(int maxSteps)
        {
            var start = Enumerable.Range(0, _n).ToArray();
            if (start.Length == 1)
            {
                yield return [];
                yield break;
            }
            var seen = new HashSet<string> { Signature(start) };
            var level = new List<(int[] Set, List<Step> Steps, int Cost)> { (start, [], 0) };
            var given = new List<List<Step>>();
            for (int depth = 1; depth <= maxSteps && level.Count > 0; depth++)
            {
                var next = new List<(int[] Set, List<Step> Steps, int Cost)>();
                var solved = new List<(List<Step> Steps, int Cost)>();
                foreach (var (set, steps, cost) in level)
                    foreach (var move in _moves)
                    {
                        if (move(set) is not { } narrowed || narrowed.Set.Length == set.Length)
                            continue;
                        List<Step> path = [.. steps, narrowed.Step];
                        if (narrowed.Set.Length == 1)
                            solved.Add((path, cost + narrowed.Cost));
                        else if (seen.Add(Signature(narrowed.Set)))
                            next.Add((narrowed.Set, path, cost + narrowed.Cost));
                    }
                foreach (var (steps, _) in solved.OrderBy(s => s.Cost))
                    if (!given.Any(simpler => IsSubsequence(simpler, steps)))
                    {
                        given.Add(steps);
                        yield return steps;
                    }
                // Cheapest first, so a set reached two ways keeps its cheaper path
                level = [.. next.OrderBy(n => n.Cost)];
            }
        }

        private static (Step, int, int[])? Filter(int[] set, Func<int, bool> keep, Step step, int cost) =>
            (step, cost, [.. set.Where(keep)]);

        /// <summary>
        /// The target's group, by its index from the nearer end: an end group at <paramref name="extremeCost"/>, an
        /// inner one (if <paramref name="inner"/>) dearer the further in. Not applicable if any candidate lacks a key,
        /// as grouping it would throw.
        /// </summary>
        private (Step, int, int[])? Group(int[] set, double?[] key, Func<int, Step> step, int extremeCost, int innerCost, bool inner)
        {
            if (set.Any(i => key[i] is null))
                return null;
            var keys = set.Select(i => key[i]!.Value).Distinct().Order().ToList();
            var target = key[_target]!.Value;
            int at = keys.BinarySearch(target), fromEnd = keys.Count - 1 - at;
            bool extreme = at == 0 || fromEnd == 0;
            if (!extreme && !inner)
                return null;
            int index = at <= fromEnd ? at : -(fromEnd + 1);
            int cost = extreme ? extremeCost : innerCost + Math.Min(at, fromEnd);
            return (step(index), cost, [.. set.Where(i => key[i] == target)]);
        }

        private static bool IsSubsequence(List<Step> small, List<Step> large)
        {
            int k = 0;
            foreach (var step in large)
                if (k < small.Count && small[k].Equals(step))
                    k++;
            return k == small.Count;
        }

        private static string Signature(int[] set) => string.Join(",", set);

        private static TValue? Try<TValue>(Func<TValue> get) where TValue : struct
        {
            try
            {
                return get();
            }
            catch (Exception e) when (e is InvalidOperationException or ArgumentException or Standard_Failure or NotSupportedException
                                       or NotImplementedException)
            {
                return null;
            }
        }
    }
}
