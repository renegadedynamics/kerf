using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;

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
/// <c>ShapeUpgrade_UnifySameDomain</c> hand over a complete record; other algorithms answer per sub-shape and
/// <see cref="FromAlgorithm(MakeShape, IEnumerable{Kerf.Topology.Shape}, Kerf.Topology.Shape)"/> collects those answers.
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
    // Never modified once the constructor returns
    private readonly History _wrapped;
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

    private ShapeHistory(History history, IEnumerable<global::Kerf.Topology.Shape> before, IEnumerable<global::Kerf.Topology.Shape> brought)
    {
        _wrapped = history;
        Before = before.ToArray();
        Brought = brought.ToArray();
    }

    /// <summary>An empty record: everything came through untouched.</summary>
    public ShapeHistory(IEnumerable<global::Kerf.Topology.Shape>? before = null, IEnumerable<global::Kerf.Topology.Shape>? brought = null)
        : this(new History(), before ?? [], brought ?? []) { }

    /// <summary>A copy of an OCCT history, which the record keeps to itself.</summary>
    public static ShapeHistory FromHistory(History? history, IEnumerable<global::Kerf.Topology.Shape>? before = null,
                                           IEnumerable<global::Kerf.Topology.Shape>? brought = null)
    {
        var own = history is null ? new History() : new History(history);
        return new ShapeHistory(own, before ?? [], brought ?? []);
    }

    // ---- Constructors ----

    /// <summary>The record a boolean operation (or splitter) kept while building.</summary>
    public static ShapeHistory FromBoolean(BuilderAlgo operation, IEnumerable<global::Kerf.Topology.Shape>? before = null,
                                           IEnumerable<global::Kerf.Topology.Shape>? brought = null)
    {
        var history = operation.History();
        return FromHistory(history, before, brought);
    }

    /// <summary>The record of a <c>Clean</c>.</summary>
    public static ShapeHistory FromUnify(UnifySameDomain upgrader)
    {
        var history = upgrader.History();
        return FromHistory(history);
    }

    /// <summary>
    /// Collects the per-sub-shape answers of a <c>BRepBuilderAPI_MakeShape</c>.
    /// </summary>
    /// <remarks>
    /// <c>IsDeleted</c> is not consulted: several algorithms answer it for the sub-shape kinds they track and
    /// guess for the rest, so a sub-shape is removed when it is neither still in <paramref name="result"/> nor
    /// modified into something that is. An algorithm that throws when asked about a kind it doesn't track
    /// (<c>BRepFilletAPI_MakeFillet2d</c> only answers for edges) is taken to have no answer for it.
    /// </remarks>
    public static ShapeHistory FromAlgorithm(MakeShape algorithm, IEnumerable<global::Kerf.Topology.Shape> inputs, global::Kerf.Topology.Shape result) =>
        FromAlgorithm(s => algorithm.Modified(s), s => algorithm.Generated(s), inputs, result);

    internal static ShapeHistory FromAlgorithm(Func<global::Kerf.Topology.Shape, IEnumerable<global::Kerf.Topology.Shape>> modified,
                                               Func<global::Kerf.Topology.Shape, IEnumerable<global::Kerf.Topology.Shape>> generated,
                                               IEnumerable<global::Kerf.Topology.Shape> inputs, global::Kerf.Topology.Shape result)
    {
        var inputList = inputs.ToList();
        var history = new History();
        var present = new HashSet<global::Kerf.Topology.Shape>(ShapeHasher.Instance);
        foreach (var sub in TrackedSubshapes([result]))
            present.Add(sub);
        foreach (var sub in TrackedSubshapes(inputList))
        {
            var mods = Answers(modified, sub);
            foreach (var shape in mods)
                history.AddModified(sub, shape);
            foreach (var shape in Answers(generated, sub))
                if (History.IsSupportedType(shape))
                    history.AddGenerated(sub, shape);
            if (mods.Length == 0 && !present.Contains(sub))
                history.Remove(sub);
        }
        return new ShapeHistory(history, inputList, []);
    }

    /// <summary>
    /// The record of sewing faces into a shell. Sewing reports faces it rebuilt through <c>Modified</c> and
    /// the edges and vertices it merged through <c>ModifiedSubShape</c>; a face it dropped as a duplicate is
    /// neither, and is recorded as removed.
    /// </summary>
    public static ShapeHistory FromSewing(Sewing sewing, IEnumerable<global::Kerf.Topology.Shape> inputs)
    {
        var inputList = inputs.ToList();
        var history = new History();
        var present = new HashSet<global::Kerf.Topology.Shape>(ShapeHasher.Instance);
        foreach (var sub in TrackedSubshapes([sewing.SewedShape()]))
            present.Add(sub);
        foreach (var sub in TrackedSubshapes(inputList))
        {
            if (sub.ShapeType() == global::Kerf.Topology.ShapeType.Face)
            {
                if (sewing.IsModified(sub))
                    history.AddModified(sub, sewing.Modified(sub));
                else if (!present.Contains(sub))
                    history.Remove(sub);
            }
            else if (sewing.IsModifiedSubShape(sub))
                history.AddModified(sub, sewing.ModifiedSubShape(sub));
            else if (!present.Contains(sub))
                history.Remove(sub);
        }
        return new ShapeHistory(history, inputList, []);
    }

    /// <summary>
    /// The record of moving a shape: each sub-shape modified into its moved self. A move keeps every TShape
    /// and changes the root Location; each child is normalised by its root Location so repeated instances keep
    /// their relative placements. Sub-shapes whose Location didn't change are left untouched.
    /// </summary>
    public static ShapeHistory FromRelocation(global::Kerf.Topology.Shape before, global::Kerf.Topology.Shape after)
    {
        var history = new History();
        var beforeToLocal = before.Location().Inverted();
        var afterToLocal = after.Location().Inverted();
        var placed = new Dictionary<global::Kerf.Topology.Shape, global::Kerf.Topology.Shape>(ShapeHasher.Instance);
        foreach (var sub in TrackedSubshapes([after]))
            placed[sub.Moved(afterToLocal)] = sub;
        foreach (var sub in TrackedSubshapes([before]))
        {
            var key = sub.Moved(beforeToLocal);
            if (placed.ContainsKey(key))
            {
                var moved = placed.GetValue(key);
                if (!moved.IsSame(sub))
                    history.AddModified(sub, moved);
            }
        }
        return new ShapeHistory(history, [before], []);
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
        if (!History.IsSupportedType(before))
            return this;
        var copy = Copy();
        copy.AddModified(before, after);
        return new ShapeHistory(copy, Before, Brought);
    }

    /// <summary>This record extended with the operation that came after it.</summary>
    public ShapeHistory Merge(ShapeHistory? later)
    {
        if (later is null)
            return this;
        var copy = Copy();
        copy.Merge(later._wrapped);
        return new ShapeHistory(copy, Before, Brought);
    }

    /// <summary>This record, saying which inputs were already there and which were brought in.</summary>
    public ShapeHistory WithInputs(IEnumerable<global::Kerf.Topology.Shape> before, IEnumerable<global::Kerf.Topology.Shape> brought) =>
        new(_wrapped, before, brought);   // the OCCT history is never modified, so it can be shared

    private History Copy()
    {
        // As build123d merges in place: Merge into an empty history isn't a copy
        return new History(_wrapped);
    }

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
        if (!History.IsSupportedType(shape))
            return [];
        var list = _wrapped.Modified(shape);
        return list.ToArray();
    }

    /// <summary>What was made from an input sub-shape, if anything.</summary>
    public global::Kerf.Topology.Shape[] Generated(global::Kerf.Topology.Shape shape)
    {
        if (!History.IsSupportedType(shape))
            return [];
        var list = _wrapped.Generated(shape);
        return list.ToArray();
    }

    /// <summary>Was an input sub-shape removed by the operation?</summary>
    public bool IsRemoved(global::Kerf.Topology.Shape shape) => History.IsSupportedType(shape) && _wrapped.IsRemoved(shape);

    /// <summary>The record indexed from the result's side for the given inputs.</summary>
    public ShapeTrace TraceOf(IEnumerable<global::Kerf.Topology.Shape> inputs) => new(this, inputs);

    /// <summary>Every distinct vertex, edge, face and solid of the given shapes.</summary>
    internal static global::Kerf.Topology.Shape[] TrackedSubshapes(IEnumerable<global::Kerf.Topology.Shape> shapes)
    {
        var found = new NCollection_IndexedMap<global::Kerf.Topology.Shape>(ShapeHasher.Instance);
        foreach (var shape in shapes)
        {
            if (shape.IsNull)
                continue;
            foreach (var type in TrackedTypes)
                Exploration.MapShapes(shape, type, found);
        }
        return found.ToArray();
    }

    /// <summary>What an algorithm says about one sub-shape, or nothing if it won't say.</summary>
    private static global::Kerf.Topology.Shape[] Answers(Func<global::Kerf.Topology.Shape, IEnumerable<global::Kerf.Topology.Shape>> query, global::Kerf.Topology.Shape shape)
    {
        try
        {
            return [.. query(shape)];
        }
        catch (FailureException)
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
    private readonly HashSet<global::Kerf.Topology.Shape> _untouched = new(ShapeHasher.Instance);
    private readonly Dictionary<global::Kerf.Topology.Shape, global::Kerf.Topology.Shape> _modifiedFrom = new(ShapeHasher.Instance);
    private readonly Dictionary<global::Kerf.Topology.Shape, global::Kerf.Topology.Shape> _generatedFrom = new(ShapeHasher.Instance);

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
