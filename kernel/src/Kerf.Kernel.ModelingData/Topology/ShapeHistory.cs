// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Topology;

namespace Kerf.Topology;

/// <summary>How a sub-shape of an operation's input came through it.</summary>
public enum ShapeHistoryKind
{
    /// <summary>Rebuilt into one or more sub-shapes of the same type (a face split by a Boolean).</summary>
    Modified,

    /// <summary>Gave rise to sub-shapes of another type (the faces an extrusion sweeps from an edge).</summary>
    Generated,

    /// <summary>Gone from the result.</summary>
    Removed,
}

/// <summary>One relation of a <see cref="ShapeHistory"/>: an input sub-shape, how it came through, and what it became.</summary>
/// <param name="Initial">The input's sub-shape: a vertex, an edge, a face or a solid.</param>
/// <param name="Kind">How it came through.</param>
/// <param name="Results">What it became: empty when it was removed.</param>
public readonly record struct ShapeHistoryEntry(Shape Initial, ShapeHistoryKind Kind, ImmutableArray<Shape> Results);

/// <summary>
/// What an operation did to its inputs' sub-shapes: which were modified, which generated new ones, which were removed.
/// Every operation that builds a shape returns one with it (<see cref="Built{TShape}"/>). Immutable.
/// </summary>
/// <remarks>
/// <para>
/// A history tracks vertices, edges, faces and solids, compared with <see cref="Shape.IsSame"/>: a sub-shape not named in
/// it came through unchanged. It is OCCT's <c>BRepTools_History</c>, copied once and never changed.
/// </para>
/// <para>
/// Histories chain: <c>first.Then(second)</c> answers for <c>first</c>'s inputs with <c>second</c>'s results, when
/// <c>second</c> ran on <c>first</c>'s result.
/// </para>
/// </remarks>
public sealed class ShapeHistory
{
    // Never changed once the constructor returns
    private readonly History _history;

    private ShapeHistory(History history) => _history = history;

    /// <summary>The history of an operation that changed nothing.</summary>
    public static ShapeHistory Empty { get; } = new(new History());

    /// <summary>A history of the given relations: raw assembly, for an operation of the caller's own.</summary>
    /// <remarks>Relations of other shape types than vertices, edges, faces and solids are ignored, as are results of a
    /// modification whose type differs from the initial's.</remarks>
    /// <exception cref="ArgumentNullException"><paramref name="entries"/> is null.</exception>
    /// <exception cref="ArgumentException">An entry's initial shape is null.</exception>
    public static ShapeHistory Create(IEnumerable<ShapeHistoryEntry> entries)
    {
        KernelArgumentNullException.ThrowIfNull(entries);
        var history = new History();
        foreach (ShapeHistoryEntry entry in entries)
        {
            if (entry.Initial.IsNull)
            {
                throw new KernelArgumentException("A history entry's initial shape is null.", nameof(entries));
            }

            switch (entry.Kind)
            {
                case ShapeHistoryKind.Removed:
                    history.Remove(entry.Initial);
                    break;
                case ShapeHistoryKind.Modified:
                    foreach (Shape result in entry.Results.IsDefault ? [] : entry.Results)
                    {
                        history.AddModified(entry.Initial, result);
                    }

                    break;
                case ShapeHistoryKind.Generated:
                    foreach (Shape result in entry.Results.IsDefault ? [] : entry.Results)
                    {
                        history.AddGenerated(entry.Initial, result);
                    }

                    break;
                default:
                    throw new KernelArgumentOutOfRangeException(nameof(entries), entry.Kind, "An unknown history kind.");
            }
        }

        return new ShapeHistory(history);
    }

    /// <summary>The public history of an engine history: a copy, which the engine may go on changing.</summary>
    internal static ShapeHistory FromEngine(History? history) => history is null ? Empty : new ShapeHistory(new History(history));

    /// <summary>A copy of the engine history, for the engine to merge into.</summary>
    internal History ToEngine() => new(_history);

    /// <summary>The sub-shapes <paramref name="initial"/> was rebuilt into; empty if it wasn't modified.</summary>
    public IReadOnlyList<Shape> Modified(Shape initial) => [.. _history.Modified(initial)];

    /// <summary>The sub-shapes <paramref name="initial"/> gave rise to; empty if it generated none.</summary>
    public IReadOnlyList<Shape> Generated(Shape initial) => [.. _history.Generated(initial)];

    /// <summary>True if <paramref name="initial"/> is gone from the result.</summary>
    public bool IsRemoved(Shape initial) => _history.IsRemoved(initial);

    /// <summary>True if some sub-shape was modified.</summary>
    public bool HasModified => _history.HasModified;

    /// <summary>True if some sub-shape generated others.</summary>
    public bool HasGenerated => _history.HasGenerated;

    /// <summary>True if some sub-shape was removed.</summary>
    public bool HasRemoved => _history.HasRemoved;

    /// <summary>Every relation: the modifications, then the generations, then the removals.</summary>
    public IEnumerable<ShapeHistoryEntry> Entries
    {
        get
        {
            foreach ((Shape initial, History.TRelationType kind, IReadOnlyList<Shape> results) in _history.Relations())
            {
                yield return new ShapeHistoryEntry(initial, kind switch
                {
                    History.TRelationType.TRelationType_Modified => ShapeHistoryKind.Modified,
                    History.TRelationType.TRelationType_Generated => ShapeHistoryKind.Generated,
                    _ => ShapeHistoryKind.Removed,
                }, [.. results]);
            }
        }
    }

    /// <summary>
    /// This history followed by <paramref name="next"/>, which ran on this one's result: it answers for this history's
    /// inputs with <paramref name="next"/>'s results.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="next"/> is null.</exception>
    public ShapeHistory Then(ShapeHistory next)
    {
        KernelArgumentNullException.ThrowIfNull(next);
        History merged = ToEngine();
        merged.Merge(next._history);
        return new ShapeHistory(merged);
    }

    /// <summary>The histories in order, each having run on the previous one's result (<see cref="Then"/>).</summary>
    /// <exception cref="ArgumentNullException">A history is null.</exception>
    public static ShapeHistory Compose(params ReadOnlySpan<ShapeHistory> histories)
    {
        if (histories.IsEmpty)
        {
            return Empty;
        }

        ShapeHistory result = histories[0] ?? throw new KernelArgumentNullException(nameof(histories));
        foreach (ShapeHistory history in histories[1..])
        {
            result = result.Then(history ?? throw new KernelArgumentNullException(nameof(histories)));
        }

        return result;
    }
}
