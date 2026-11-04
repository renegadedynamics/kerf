using Kerf.Topology;

namespace Kerf.Modeling;

/// <summary>The kinds of topological shape (build123d's <c>Shapes</c> names).</summary>
public enum ShapeType
{
#pragma warning disable CS1591 // Self-describing
    Vertex,
    Edge,
    Wire,
    Face,
    Shell,
    Solid,
    CompSolid,
    Compound,
#pragma warning restore CS1591
}

/// <summary>
/// The record of what an operation did to its inputs' sub-shapes.
/// </summary>
/// <remarks>
/// <para>
/// Each tracked sub-shape of an input ends up in one of four states in the result: untouched (still there,
/// unchanged), modified (rebuilt into one or more sub-shapes of the same kind), generating (a sub-shape of
/// another kind was made from it, as a face is made from an edge by an extrusion) or removed. Booleans and
/// <c>ShapeUpgrade_UnifySameDomain</c> hand over a complete record (the kernel's operations return it with their result);
/// other algorithms answer per sub-shape and the modelling layer collects those answers.
/// </para>
/// <para>
/// Records chain with <see cref="Merge"/>, so that after a fuse and a clean a face of the original input
/// still maps to the face it became. A record also knows which inputs were already there (<see cref="Before"/>)
/// and which the operation brought in (<see cref="Brought"/>): for <c>a + b</c> the argument and the tool.
/// That is what lets a sub-shape of the result be classified as last or new.
/// </para>
/// <para>
/// Unlike build123d's, a record is immutable: <see cref="Merge"/> returns a new record, so the shapes that
/// share one can't see it change.
/// </para>
/// </remarks>
public sealed class ShapeHistory
{
    // The kernel's record, immutable
    private readonly global::Kerf.Topology.ShapeHistory _wrapped;
    private (ShapeTrace Before, ShapeTrace Brought)? _traces;

    /// <summary>The sub-shape types BRepTools_History records; wires, shells and compounds are containers.</summary>
    internal static readonly global::Kerf.Topology.ShapeType[] TrackedTypes =
    [
        global::Kerf.Topology.ShapeType.Vertex, global::Kerf.Topology.ShapeType.Edge, global::Kerf.Topology.ShapeType.Face, global::Kerf.Topology.ShapeType.Solid,
    ];

    /// <summary>The inputs that were already there.</summary>
    public IReadOnlyList<global::Kerf.Topology.Shape> Before { get; }

    /// <summary>The inputs the operation brought in.</summary>
    public IReadOnlyList<global::Kerf.Topology.Shape> Brought { get; }

    private ShapeHistory(global::Kerf.Topology.ShapeHistory history, IEnumerable<global::Kerf.Topology.Shape> before,
                         IEnumerable<global::Kerf.Topology.Shape> brought)
    {
        _wrapped = history;
        Before = before.ToArray();
        Brought = brought.ToArray();
    }

    /// <summary>An empty record: everything came through untouched.</summary>
    public ShapeHistory(IEnumerable<global::Kerf.Topology.Shape>? before = null, IEnumerable<global::Kerf.Topology.Shape>? brought = null)
        : this(global::Kerf.Topology.ShapeHistory.Empty, before ?? [], brought ?? []) { }

    /// <summary>A record of the kernel's history of an operation (immutable, so shared).</summary>
    public static ShapeHistory FromHistory(global::Kerf.Topology.ShapeHistory? history, IEnumerable<global::Kerf.Topology.Shape>? before = null,
                                           IEnumerable<global::Kerf.Topology.Shape>? brought = null) =>
        new(history ?? global::Kerf.Topology.ShapeHistory.Empty, before ?? [], brought ?? []);

    /// <summary>The kernel's history this record holds.</summary>
    public global::Kerf.Topology.ShapeHistory ToHistory() => _wrapped;

    // ---- Constructors ----

    /// <summary>
    /// Collects the per-sub-shape answers of an algorithm.
    /// </summary>
    /// <remarks>
    /// <c>IsDeleted</c> is not consulted: several algorithms answer it for the sub-shape kinds they track and
    /// guess for the rest, so a sub-shape is removed when it is neither still in <paramref name="result"/> nor
    /// modified into something that is. An algorithm that throws when asked about a kind it doesn't track
    /// (<c>BRepFilletAPI_MakeFillet2d</c> only answers for edges) is taken to have no answer for it.
    /// </remarks>
    internal static ShapeHistory FromAlgorithm(Func<global::Kerf.Topology.Shape, IEnumerable<global::Kerf.Topology.Shape>> modified,
                                               Func<global::Kerf.Topology.Shape, IEnumerable<global::Kerf.Topology.Shape>> generated,
                                               IEnumerable<global::Kerf.Topology.Shape> inputs, global::Kerf.Topology.Shape result)
    {
        var inputList = inputs.ToList();
        var entries = new List<ShapeHistoryEntry>();
        var present = new HashSet<global::Kerf.Topology.Shape>(ShapeComparer.Same);
        foreach (var sub in TrackedSubshapes([result]))
            present.Add(sub);
        foreach (var sub in TrackedSubshapes(inputList))
        {
            var mods = Answers(modified, sub);
            if (mods.Length > 0)
                entries.Add(new(sub, ShapeHistoryKind.Modified, [.. mods]));
            var gens = Answers(generated, sub).Where(IsSupportedType).ToArray();
            if (gens.Length > 0)
                entries.Add(new(sub, ShapeHistoryKind.Generated, [.. gens]));
            if (mods.Length == 0 && !present.Contains(sub))
                entries.Add(new(sub, ShapeHistoryKind.Removed, []));
        }
        return new ShapeHistory(global::Kerf.Topology.ShapeHistory.Create(entries), inputList, []);
    }

    /// <summary>
    /// The record of moving a shape: each sub-shape modified into its moved self. A move keeps every TShape
    /// and changes the root Location; each child is normalised by its root Location so repeated instances keep
    /// their relative placements. Sub-shapes whose Location didn't change are left untouched.
    /// </summary>
    public static ShapeHistory FromRelocation(global::Kerf.Topology.Shape before, global::Kerf.Topology.Shape after)
    {
        var entries = new List<ShapeHistoryEntry>();
        var beforeToLocal = before.Placement.Inverted();
        var afterToLocal = after.Placement.Inverted();
        var placed = new Dictionary<global::Kerf.Topology.Shape, global::Kerf.Topology.Shape>(ShapeComparer.Same);
        foreach (var sub in TrackedSubshapes([after]))
            placed[sub.Moved(afterToLocal)] = sub;
        foreach (var sub in TrackedSubshapes([before]))
        {
            var key = sub.Moved(beforeToLocal);
            if (placed.TryGetValue(key, out var moved) && !moved.IsSame(sub))
                entries.Add(new(sub, ShapeHistoryKind.Modified, [moved]));
        }
        return new ShapeHistory(global::Kerf.Topology.ShapeHistory.Create(entries), [before], []);
    }

    /// <summary>
    /// The records the given shapes carry from the operations that made them, merged into one, or null if
    /// none of them carries one.
    /// </summary>
    public static ShapeHistory? Of(IEnumerable<Shape?> shapes)
    {
        var found = shapes.Select(s => s?.History).OfType<ShapeHistory>().ToList();
        if (found.Count == 0)
            return null;
        if (found.Count == 1)
            return found[0];
        var merged = new ShapeHistory(found.SelectMany(h => h.Before), found.SelectMany(h => h.Brought));
        foreach (var history in found)
            merged = merged.Merge(history);
        return merged;
    }

    // ---- Instance methods ----

    /// <summary>
    /// This record extended with the knowledge that <paramref name="before"/> was rebuilt into
    /// <paramref name="after"/>: for what an algorithm did but doesn't say.
    /// </summary>
    public ShapeHistory AddModified(global::Kerf.Topology.Shape before, global::Kerf.Topology.Shape after)
    {
        if (!IsSupportedType(before))
            return this;
        var history = global::Kerf.Topology.ShapeHistory.Create([.. _wrapped.Entries, new(before, ShapeHistoryKind.Modified, [after])]);
        return new ShapeHistory(history, Before, Brought);
    }

    /// <summary>This record extended with the operation that came after it.</summary>
    public ShapeHistory Merge(ShapeHistory? later)
    {
        if (later is null)
            return this;
        return new ShapeHistory(_wrapped.Then(later._wrapped), Before, Brought);
    }

    /// <summary>This record, saying which inputs were already there and which were brought in.</summary>
    public ShapeHistory WithInputs(IEnumerable<global::Kerf.Topology.Shape> before, IEnumerable<global::Kerf.Topology.Shape> brought) =>
        new(_wrapped, before, brought);   // the kernel's history is immutable, so it is shared

    private (ShapeTrace Before, ShapeTrace Brought) Traces => _traces ??= (new ShapeTrace(this, Before), new ShapeTrace(this, Brought));

    /// <summary>
    /// Did the operation bring this sub-shape of the result in, or create it? A sub-shape that descends from
    /// something brought in, untouched or rebuilt, is last, including one that also descends from what was there
    /// before. One that descends only from what was there before is not. One the record can't attribute to any
    /// input is new, and so last.
    /// </summary>
    public bool IsLast(global::Kerf.Topology.Shape shape)
    {
        var (before, brought) = Traces;
        if (brought.IsUntouched(shape) || brought.IsModified(shape))
            return true;
        return !(before.IsUntouched(shape) || before.IsModified(shape));
    }

    /// <summary>Did this sub-shape of the result exist in no input, in any form?</summary>
    public bool IsNew(global::Kerf.Topology.Shape shape)
    {
        var (before, brought) = Traces;
        foreach (var trace in new[] { brought, before })
            if (trace.IsUntouched(shape) || trace.IsModified(shape))
                return false;
        return true;
    }

    /// <summary>What an input sub-shape was rebuilt into, if anything.</summary>
    public global::Kerf.Topology.Shape[] Modified(global::Kerf.Topology.Shape shape)
    {
        if (!IsSupportedType(shape))
            return [];
        return [.. _wrapped.Modified(shape)];
    }

    /// <summary>What was made from an input sub-shape, if anything.</summary>
    public global::Kerf.Topology.Shape[] Generated(global::Kerf.Topology.Shape shape)
    {
        if (!IsSupportedType(shape))
            return [];
        return [.. _wrapped.Generated(shape)];
    }

    /// <summary>Was an input sub-shape removed by the operation?</summary>
    public bool IsRemoved(global::Kerf.Topology.Shape shape) => IsSupportedType(shape) && _wrapped.IsRemoved(shape);

    /// <summary>The kinds of sub-shape a history records (OCCT's <c>BRepTools_History::IsSupportedType</c>).</summary>
    internal static bool IsSupportedType(global::Kerf.Topology.Shape shape) => !shape.IsNull && TrackedTypes.Contains(shape.Type);

    /// <summary>The record indexed from the result's side for the given inputs.</summary>
    public ShapeTrace TraceOf(IEnumerable<global::Kerf.Topology.Shape> inputs) => new(this, inputs);

    /// <summary>Every distinct vertex, edge, face and solid of the given shapes.</summary>
    internal static global::Kerf.Topology.Shape[] TrackedSubshapes(IEnumerable<global::Kerf.Topology.Shape> shapes)
    {
        var seen = new HashSet<global::Kerf.Topology.Shape>(ShapeComparer.Same);
        var found = new List<global::Kerf.Topology.Shape>();
        foreach (var shape in shapes)
        {
            if (shape.IsNull)
                continue;
            foreach (var type in TrackedTypes)
                foreach (var sub in TopoUtil.Unique(shape, type))
                    if (seen.Add(sub))
                        found.Add(sub);
        }
        return [.. found];
    }

    /// <summary>What an algorithm says about one sub-shape, or nothing if it won't say.</summary>
    private static global::Kerf.Topology.Shape[] Answers(Func<global::Kerf.Topology.Shape, IEnumerable<global::Kerf.Topology.Shape>> query, global::Kerf.Topology.Shape shape)
    {
        try
        {
            return [.. query(shape)];
        }
        catch (Exception e) when (Failures.IsKernelFailure(e))
        {
            return [];
        }
    }
}

/// <summary>
/// The record of a <see cref="ShapeHistory"/> read from the result's side. Built for a set of inputs, it answers
/// for any sub-shape of the result whether it was one of those inputs' sub-shapes left untouched, was modified
/// from one, or was generated from one.
/// </summary>
public sealed class ShapeTrace
{
    private readonly HashSet<global::Kerf.Topology.Shape> _untouched = new(ShapeComparer.Same);
    private readonly Dictionary<global::Kerf.Topology.Shape, global::Kerf.Topology.Shape> _modifiedFrom = new(ShapeComparer.Same);
    private readonly Dictionary<global::Kerf.Topology.Shape, global::Kerf.Topology.Shape> _generatedFrom = new(ShapeComparer.Same);

    internal ShapeTrace(ShapeHistory history, IEnumerable<global::Kerf.Topology.Shape> inputs)
    {
        foreach (var sub in ShapeHistory.TrackedSubshapes(inputs))
        {
            _untouched.Add(sub);
            foreach (var shape in history.Modified(sub))
                _modifiedFrom[shape] = sub;
            foreach (var shape in history.Generated(sub))
                _generatedFrom[shape] = sub;
        }
    }

    /// <summary>Was this result sub-shape an input sub-shape, unchanged?</summary>
    public bool IsUntouched(global::Kerf.Topology.Shape shape) => _untouched.Contains(shape);

    /// <summary>Was this result sub-shape rebuilt from an input sub-shape?</summary>
    public bool IsModified(global::Kerf.Topology.Shape shape) => _modifiedFrom.ContainsKey(shape);

    /// <summary>Was this result sub-shape made from an input sub-shape of another kind?</summary>
    public bool IsGenerated(global::Kerf.Topology.Shape shape) => _generatedFrom.ContainsKey(shape);

    /// <summary>Did this result sub-shape come from the inputs in any way?</summary>
    public bool IsDescendant(global::Kerf.Topology.Shape shape) => IsUntouched(shape) || IsModified(shape) || IsGenerated(shape);
}
