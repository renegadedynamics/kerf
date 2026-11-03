// Kerf: the working overlay of healing's tools (unit 10H4): an operation run in place inside another's overlay, and
// the detached overlay of a tool whose fixes run in several public calls. Not a port of an OCCT file.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Tools;

internal sealed partial class WorkingOverlay
{
    private Dictionary<TShape, TShape>? _initial; // the inverse of _final, made at the first Initial

    /// <summary>
    /// The overlay current here, if it still takes writes (not finished); else null.
    /// </summary>
    public static WorkingOverlay? Writing => Current is { _isFinished: false } current ? current : null;

    /// <summary>
    /// True if <paramref name="tShape"/> is frozen and no overlay here takes writes into it: the port's frozen-shape
    /// checks that skip or refuse a write ask this, so that inside an overlay the operation writes as OCCT's does (as it
    /// did into the unfrozen copies made at the entry).
    /// </summary>
    public static bool IsReadOnly(TShape tShape) => tShape.IsFrozen && Writing is null;

    /// <summary>
    /// Opens an overlay for an in-place operation on <paramref name="input"/>, unless one that takes writes is open
    /// here: then returns null, and the operation runs in place inside it, its writes going to that overlay, as OCCT's
    /// in-place algorithm called by another (ShapeFix_Wire by ShapeFix_Face, ShapeFix_Shape by ShapeProcess) writes
    /// into the shapes the outer one works on. The caller finishes and disposes the overlay it gets.
    /// </summary>
    public static WorkingOverlay? BeginUnlessWriting(in Shape input, ReShape? context = null) =>
        Writing is null ? Begin(input, context) : null;

    /// <summary>
    /// Makes an overlay for a tool whose fixes run in several public calls (a load, then fixes, then a result):
    /// current nowhere until <see cref="Enter"/>, so nothing is left open between the calls. It needs no disposal.
    /// </summary>
    public static WorkingOverlay Detached(in Shape input, ReShape? context = null) => new(null, input, context);

    /// <summary>
    /// The overlay a tool with several public calls works in from its first fix to its result:
    /// <see cref="Writing"/> when one is open here (the tool runs in place inside the operation that called it, and
    /// <paramref name="owns"/> is false: the tool doesn't finish it), else a <see cref="Detached"/> one it owns.
    /// </summary>
    public static WorkingOverlay ForTool(in Shape input, ReShape? context, out bool owns)
    {
        WorkingOverlay? writing = Writing;
        owns = writing is null;
        return writing ?? Detached(input, context);
    }

    /// <summary>
    /// Makes this overlay current here until the scope is disposed: nothing when it is current already or finished.
    /// </summary>
    public Scope Enter()
    {
        WorkingOverlay? previous = s_current.Value;
        if (_isFinished || ReferenceEquals(previous, this))
        {
            return default;
        }
        s_current.Value = this;
        Interlocked.Increment(ref s_openCount);
        return new Scope(previous, true);
    }

    /// <summary>The scope of <see cref="Enter"/>: its disposal restores the overlay current before.</summary>
    public readonly struct Scope : IDisposable
    {
        private readonly WorkingOverlay? _previous;
        private readonly bool _isEntered;

        internal Scope(WorkingOverlay? previous, bool isEntered)
        {
            _previous = previous;
            _isEntered = isEntered;
        }

        /// <summary>Restores the overlay current before <see cref="Enter"/>.</summary>
        public void Dispose()
        {
            if (_isEntered)
            {
                s_current.Value = _previous;
                Interlocked.Decrement(ref s_openCount);
            }
        }
    }

    /// <summary>
    /// The input of a functional operation (a twin, whose result is new and whose input is never written) with each of
    /// its unfrozen TShapes (the caller's own, which an overlay lets the operation write in place) replaced by a copy
    /// holding the same data and children, so that the writes there go to the copies; frozen TShapes are shared, and
    /// their writes go to the overlay. <paramref name="copies"/> maps each unfrozen TShape to its copy (null when there
    /// is none). The input itself when its TShape is frozen, the usual case.
    /// </summary>
    public static Shape WithUnfrozenCopied(in Shape input, out Dictionary<TShape, TShape>? copies)
    {
        copies = null;
        if (input.IsNull || input.TShape()!.IsFrozen)
        {
            return input;
        }
        copies = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
        Shape aShape = input;
        aShape.TShape(CopyMutable(input.TShape()!, copies));
        return aShape;
    }

    /// <summary>
    /// After <see cref="Finish"/> of an operation run on <see cref="WithUnfrozenCopied"/>'s input: the result with each
    /// copy that the operation left equal to its original (data and children) replaced by the original, so that an
    /// input needing nothing comes back itself, as the sub-shapes the overlay didn't change do.
    /// </summary>
    public static Shape UncopiedWhereUnchanged(in Shape result, Dictionary<TShape, TShape>? copies)
    {
        if (copies is null || result.IsNull)
        {
            return result;
        }
        var originals = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
        foreach (KeyValuePair<TShape, TShape> pair in copies)
        {
            originals[pair.Value] = pair.Key;
        }
        Shape aResult = result;
        aResult.TShape(Uncopied(result.TShape()!, originals,
                                new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance)));
        return aResult;
    }

    private static TShape Uncopied(TShape tShape, Dictionary<TShape, TShape> originals, Dictionary<TShape, TShape> done)
    {
        if (done.TryGetValue(tShape, out TShape? image))
        {
            return image;
        }
        done[tShape] = tShape; // a cycle (not expected) stops here
        if (tShape.IsFrozen)
        {
            return tShape;
        }
        System.Runtime.CompilerServices.RuntimeHelpers.EnsureSufficientExecutionStack();
        List<Shape>? children = null;
        int index = 0;
        for (var it = new NCollection_List<Shape>.Iterator(tShape._shapes); it.More(); it.Next(), index++)
        {
            Shape child = it.Value();
            TShape now = Uncopied(child.TShape()!, originals, done);
            if (!ReferenceEquals(now, child.TShape()) && children is null)
            {
                children = new List<Shape>(tShape._shapes.Extent());
                for (var it2 = new NCollection_List<Shape>.Iterator(tShape._shapes); children.Count < index; it2.Next())
                {
                    children.Add(it2.Value());
                }
            }
            if (children is not null)
            {
                child.TShape(now);
                children.Add(child);
            }
        }
        if (children is not null)
        {
            tShape._shapes.Clear();
            foreach (Shape child in children)
            {
                tShape._shapes.Append(child);
            }
        }
        if (originals.TryGetValue(tShape, out TShape? original) && IsSameChildren(original._shapes, tShape._shapes)
            && BrepTools.IsSameData(original, tShape, false))
        {
            done[tShape] = original;
            return original;
        }
        return tShape;
    }

    /// <summary>
    /// The shape on its TShape's copy in <paramref name="copies"/> (from
    /// <see cref="WithUnfrozenCopied(in Shape, out Dictionary{TShape, TShape}?)"/>), or itself.
    /// </summary>
    public static Shape CopiedIn(in Shape shape, Dictionary<TShape, TShape>? copies)
    {
        if (copies is null || shape.IsNull || !copies.TryGetValue(shape.TShape()!, out TShape? copy))
        {
            return shape;
        }
        Shape aShape = shape;
        aShape.TShape(copy);
        return aShape;
    }

    private static TShape CopyMutable(TShape tShape, Dictionary<TShape, TShape> copies)
    {
        if (tShape.IsFrozen)
        {
            return tShape;
        }
        if (copies.TryGetValue(tShape, out TShape? done))
        {
            return done;
        }
        System.Runtime.CompilerServices.RuntimeHelpers.EnsureSufficientExecutionStack();
        TShape copy = BrepTools.CopyData(tShape);
        BrepTools.CopyFlags(tShape, copy);
        copies[tShape] = copy;
        for (var it = new NCollection_List<Shape>.Iterator(tShape._shapes); it.More(); it.Next())
        {
            Shape child = it.Value();
            child.TShape(CopyMutable(child.TShape()!, copies));
            copy._shapes.Append(child);
        }
        return copy;
    }

    /// <summary>
    /// After <see cref="Finish"/>: the shape the operation worked with that <paramref name="shape"/> stands for (the
    /// inverse of <see cref="Final"/>): for a TShape that replaced a changed one, the one it replaced; else the shape
    /// itself. For a tool's queries keyed by the shapes it worked with (sewing's sections).
    /// </summary>
    public Shape Initial(in Shape shape)
    {
        CheckFinished();
        TShape? tShape = shape.TShape();
        if (tShape is null || _final is null)
        {
            return shape;
        }
        if (_initial is null)
        {
            _initial = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
            foreach (KeyValuePair<TShape, TShape> pair in _final)
            {
                if (!ReferenceEquals(pair.Key, pair.Value))
                {
                    _initial[pair.Value] = pair.Key;
                }
            }
        }
        if (!_initial.TryGetValue(tShape, out TShape? initial))
        {
            return shape;
        }
        Shape aShape = shape;
        aShape.TShape(initial);
        return aShape;
    }
}
