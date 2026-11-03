// Kerf: the per-operation working overlay (phase10-plan.md, section 2.4; unit 10H1): in-place edits of topology an
// operation didn't create are recorded as replacements of its TShapes, read back through them, and assembled into a
// result that shares everything unchanged. Not a port of an OCCT file.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using System.Collections.Concurrent;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Topology.Tools;

/// <summary>
/// The working overlay of one operation: the replacements of the frozen TShapes the operation writes into, read back
/// in their place while it runs, then assembled into its result.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's algorithms edit the topology they are given: they raise a vertex's tolerance, add a pcurve, set an edge's
/// range or flags, add and remove sub-shapes. In Kerf the shapes a caller holds are frozen. Instead of copying the input
/// at the entry (<see cref="CopyAtEntry"/>), an operation opens an overlay and runs on the input itself:
/// </para>
/// <list type="number">
/// <item><description><b>Writes.</b> The first write into a frozen TShape (through <c>BrepBuilder</c>,
/// <c>Builder.Add</c>/<c>Remove</c> or a TShape's internal setters) makes its <i>working copy</i>: a new TShape with
/// its data, flags and children, which takes this write and every later one. The frozen TShape is never
/// written.</description></item>
/// <item><description><b>Reads.</b> While the overlay is open on the thread (or in a task it started), a frozen
/// TShape's accessors (<c>BRep_Tool</c>'s reads, the TShapes' getters, <c>TopoDS_Iterator</c>,
/// <c>TopExp_Explorer</c>) answer from its working copy if it has one. Shapes keep the input's TShapes as their
/// identity: <c>IsSame</c>, maps of shapes and the operation's <see cref="ReShape"/> context are all keyed by the
/// caller's own sub-shapes.</description></item>
/// <item><description><b>The result.</b> <see cref="Finish"/> assembles the result: new TShapes for what changed (a
/// working copy whose data differs from its original's, and every TShape above a changed one), the input's own for
/// everything else. A working copy left equal to its original (a tolerance "raised" to the value it had) gives back
/// the original. <see cref="History"/> is direct: each of the input's sub-shapes that changed, modified into the TShape
/// that stands for it now.</description></item>
/// </list>
/// <para>
/// Overlays nest: an operation inside another sees the outer one's working copies, and its own writes go to its own
/// overlay (an inner operation's result is its own; it doesn't write into the outer operation's state). An overlay is
/// held in an <see cref="AsyncLocal{T}"/> (mirrored in a thread static, which the accessors read), so tasks and
/// <c>Parallel.For</c> bodies started inside the operation see it; operations on other threads, on the same frozen
/// inputs, have overlays of their own. The working copies are made under
/// a lock, so parallel bodies of one operation may write; what they write into one TShape is the algorithm's affair, as
/// in OCCT.
/// </para>
/// <para>
/// Unfrozen TShapes (made by the operation, or by ported code that owns them) are written in place, as in OCCT. A
/// representation (a curve on an edge, a point on a vertex) read through the read-only accessors (<c>Curves()</c>)
/// of a frozen TShape without a working copy is the frozen one: editing it throws, as before; writers use the
/// <c>Change…</c> accessors, which make the working copy.
/// </para>
/// <para>
/// Without an open overlay nothing changes: a write into a frozen TShape throws as before
/// (<see cref="LockedShapeException"/>, <see cref="ImmutableObjectException"/>), and the accessors read the TShape's
/// own data. The cost when no overlay is open anywhere in the process is one static read per accessor.
/// </para>
/// </remarks>
internal sealed class WorkingOverlay : IDisposable
{
    // the number of overlays open in the process: zero means no accessor needs to look further
    private static int s_openCount;
    private static readonly AsyncLocal<WorkingOverlay?> s_current = new(OnCurrentChanged);

    // The current overlay as a thread static, which the accessors read: an AsyncLocal's value costs a lookup in the
    // execution context, a thread static a field read. The AsyncLocal's change handler keeps the two equal on every
    // thread, whenever the value changes or a thread changes execution context (a task starting, a pool thread
    // returning).
    [ThreadStatic]
    private static WorkingOverlay? t_current;

    private static void OnCurrentChanged(AsyncLocalValueChangedArgs<WorkingOverlay?> args) => t_current = args.CurrentValue;

    private readonly WorkingOverlay? _parent;
    private readonly Shape _input;
    private ReShape? _context;
    // the working copies, and a bit per hash of their originals (most reads miss); both made at the first write, so an
    // overlay nothing writes into costs little (unit 10H3: the makers open one per change)
    private volatile ConcurrentDictionary<TShape, TShape>? _working;
    private readonly Lock _lock = new();
    private volatile int _workingCount;
    private long[]? _filter;
    private volatile bool _isAssembling; // Finish reads the state before this overlay's writes
    private Dictionary<TShape, TShape>? _final;
    private History? _history;
    private bool _isFinished;
    private bool _isDisposed;
    private bool _ignoreToleranceGuard;

    private WorkingOverlay(WorkingOverlay? parent, in Shape input, ReShape? context)
    {
        _parent = parent;
        _input = input;
        _context = context;
    }

    /// <summary>
    /// Opens an overlay for an operation on <paramref name="input"/>, current on this thread (and in the tasks it
    /// starts) until disposed. <paramref name="context"/> is the operation's <see cref="ReShape"/>, if it keeps one:
    /// <see cref="Finish"/> brings its replacements up to date. <paramref name="input"/> may be null for an operation on
    /// several shapes that keeps no history over them (<see cref="History"/> is then empty, and what it hands out goes
    /// through <see cref="Finish"/>'s <c>others</c> and <see cref="Final"/>).
    /// </summary>
    public static WorkingOverlay Begin(in Shape input, ReShape? context = null)
    {
        var overlay = new WorkingOverlay(s_current.Value, input, context);
        s_current.Value = overlay;
        Interlocked.Increment(ref s_openCount);
        return overlay;
    }

    /// <summary>
    /// Opens an overlay as <see cref="Begin"/> does, unless an operation's overlay is open here and unfinished: then
    /// null, and the writes go to that overlay (unit 10H3). For an algorithm that another operation runs on shapes it
    /// works on (a sweep of a profile in an evolved shape, the generator of a loft): OCCT writes in place, and the
    /// calling operation reads the writes back and assembles them into its own result.
    /// </summary>
    public static WorkingOverlay? BeginUnlessInside(in Shape input, ReShape? context = null)
        => Current is { IsFinished: false } ? null : Begin(input, context);

    /// <summary>The overlay current here, or null.</summary>
    public static WorkingOverlay? Current => Volatile.Read(ref s_openCount) == 0 ? null : t_current;

    /// <summary>The input given to <see cref="Begin"/>.</summary>
    public Shape Input() => _input;

    /// <summary>The overlay this one is nested in, or null.</summary>
    public WorkingOverlay? Parent => _parent;

    /// <summary>The number of TShapes written so far (each has a working copy).</summary>
    public int WorkingCount => _workingCount;

    /// <summary>True once <see cref="Finish"/> has run.</summary>
    public bool IsFinished => _isFinished;

    /// <summary>
    /// Gives the overlay the context an operation created after <see cref="Begin"/>, so that <see cref="Finish"/>
    /// updates it too (ignored once one is set).
    /// </summary>
    public void AttachContext(ReShape? context)
    {
        if (_context is null && !_isFinished)
        {
            _context = context;
        }
    }

    // ---- the hooks: the accessors of TShape and its BRep subclasses, BrepBuilder, Builder and Iterator ----

    /// <summary>
    /// True, with the working copy that stands for <paramref name="tShape"/> here, if it is frozen and an open overlay
    /// (this thread's, or one it is nested in) has written into it.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryView(TShape tShape, [NotNullWhen(true)] out TShape? view)
    {
        if (Volatile.Read(ref s_openCount) == 0)
        {
            view = null;
            return false;
        }
        return TryViewSlow(tShape, out view);
    }

    private static bool TryViewSlow(TShape tShape, [NotNullWhen(true)] out TShape? view)
    {
        view = null;
        WorkingOverlay? current = t_current;
        if (current is null || !tShape.IsFrozen)
        {
            return false;
        }
        for (WorkingOverlay? overlay = current; overlay is not null; overlay = overlay._parent)
        {
            if (!overlay._isAssembling && overlay._workingCount != 0 && overlay.MayHold(tShape)
                && overlay._working!.TryGetValue(tShape, out view))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>The TShape to read <paramref name="tShape"/>'s data from here: its working copy, or itself.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static TShape View(TShape tShape) => TryView(tShape, out TShape? view) ? view : tShape;

    /// <summary>
    /// True, with the working copy to write into, if <paramref name="tShape"/> is frozen and an overlay is open here
    /// (the working copy is made at the first write). False otherwise: the caller writes into the TShape itself, which
    /// throws if it is frozen.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool TryWritable(TShape tShape, [NotNullWhen(true)] out TShape? working)
    {
        if (Volatile.Read(ref s_openCount) == 0 || !tShape.IsFrozen)
        {
            working = null;
            return false;
        }
        WorkingOverlay? overlay = t_current;
        if (overlay is null || overlay._isFinished)
        {
            working = null;
            return false;
        }
        working = overlay.Writable(tShape);
        return true;
    }

    /// <summary>The TShape to write <paramref name="tShape"/>'s data into here: its working copy, or itself.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static TShape ForWrite(TShape tShape) => TryWritable(tShape, out TShape? working) ? working : tShape;

    /// <summary>
    /// True if a write into <paramref name="tShape"/> must be refused (OCCT's <c>Locked()</c> test in BRep_Builder): it
    /// is locked by its flag, or frozen with no overlay open here to take the write.
    /// </summary>
    internal static bool IsLocked(TShape tShape)
    {
        if (tShape.LockedFlag())
        {
            return true;
        }
        if (!tShape.IsFrozen)
        {
            return false;
        }
        WorkingOverlay? overlay = Current;
        return overlay is null || overlay._isFinished;
    }

    // The working copy of a frozen TShape, made at the first write from what the operation sees of it.
    private TShape Writable(TShape tShape)
    {
        TShape? working = null;
        if (_working?.TryGetValue(tShape, out working) == true)
        {
            return working!;
        }
        lock (_lock)
        {
            if (_working?.TryGetValue(tShape, out working) == true)
            {
                return working!;
            }
            _filter ??= new long[FilterWords];
            _working ??= new ConcurrentDictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
            TShape source = ParentView(tShape);
            working = BrepTools.CopyData(source);
            for (var it = new NCollection_List<Shape>.Iterator(source._shapes); it.More(); it.Next())
            {
                working._shapes.Append(it.Value());
            }
            BrepTools.CopyFlags(source, working);
            (int word, long bit) = FilterBit(tShape);
            Interlocked.Or(ref _filter![word], bit);
            _working[tShape] = working;
            _workingCount++;
            return working;
        }
    }

    private const int FilterWords = 32;

    private static (int Word, long Bit) FilterBit(TShape tShape)
    {
        int hash = RuntimeHelpers.GetHashCode(tShape) & (FilterWords * 64 - 1);
        return (hash >> 6, 1L << (hash & 63));
    }

    private bool MayHold(TShape tShape)
    {
        (int word, long bit) = FilterBit(tShape);
        return (Volatile.Read(ref _filter![word]) & bit) != 0;
    }

    // What the overlays this one is nested in make of the TShape (itself when none wrote into it).
    private TShape ParentView(TShape tShape)
    {
        for (WorkingOverlay? overlay = _parent; overlay is not null; overlay = overlay._parent)
        {
            if (overlay._workingCount != 0 && overlay._working!.TryGetValue(tShape, out TShape? view))
            {
                return view;
            }
        }
        return tShape;
    }

    // ---- the end of the operation ----

    /// <summary>
    /// Ends the operation: assembles <paramref name="result"/> and <paramref name="others"/> (other shapes it hands
    /// out), and the replacements its context records, with new TShapes for what changed and the input's own for the
    /// rest. In the context, a sub-shape of the input that changed without a request is recorded as replaced by what
    /// stands for it now (as <see cref="CopyAtEntry"/> does). Returns the assembled result, unfrozen. When nothing was
    /// written, returns <paramref name="result"/> as it is. With <paramref name="ignoreToleranceGuard"/>, a vertex whose
    /// tolerance was only raised by BRepLib's rounding guard (<c>2 * Epsilon(tol)</c>) counts as unchanged, as in
    /// <c>BrepTools.KeepUnchanged</c> (unit 10H3: the face of a planar wire, which updates every tolerance).
    /// </summary>
    public Shape Finish(in Shape result, IEnumerable<Shape>? others = null, bool ignoreToleranceGuard = false)
    {
        if (_isFinished)
        {
            throw new ProgramErrorException("WorkingOverlay::Finish called twice");
        }
        _isFinished = true;
        _ignoreToleranceGuard = ignoreToleranceGuard;
        if (_workingCount == 0)
        {
            return result;
        }

        _isAssembling = true;
        try
        {
            _final = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
            Shape aResult = Assembled(result);
            if (others is not null)
            {
                foreach (Shape other in others)
                {
                    Assembled(other);
                }
            }

            // the input's sub-shapes as the operation found them, and what stands for each now
            var subShapes = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            MapInput(subShapes);
            for (int i = 1; i <= subShapes.Extent(); i++)
            {
                Shape sub = subShapes[i];
                Shape now = Assembled(sub);
                if (!ReferenceEquals(now.TShape(), sub.TShape()) && global::Kerf.Engine.Topology.History.IsSupportedType(sub))
                {
                    (_history ??= new History()).AddModified(sub, now);
                }
            }

            if (_context is not null)
            {
                foreach (Shape replacement in _context.InternalReplacements())
                {
                    Assembled(replacement);
                }
                _context.ReplaceTShapes(NoImages, _final);
                for (int i = 1; i <= subShapes.Extent(); i++)
                {
                    Shape sub = subShapes[i];
                    Shape now = Final(sub);
                    if (!ReferenceEquals(now.TShape(), sub.TShape()) && !_context.IsRecorded(sub))
                    {
                        _context.Replace(sub, now);
                    }
                }
            }
            return aResult;
        }
        finally
        {
            _isAssembling = false;
        }
    }

    private static readonly Dictionary<TShape, TShape> NoImages = new(ReferenceEqualityComparer.Instance);

    /// <summary>
    /// After <see cref="Finish"/>: what stands for a shape the operation worked with: for a TShape it wrote into (or
    /// one above it), the new TShape; else the shape itself.
    /// </summary>
    public Shape Final(in Shape shape)
    {
        CheckFinished();
        TShape? tShape = shape.TShape();
        if (tShape is null || _final is null || !_final.TryGetValue(tShape, out TShape? image))
        {
            return shape;
        }
        Shape aShape = shape;
        aShape.TShape(image);
        return aShape;
    }

    /// <summary>
    /// After <see cref="Finish"/>: the input's vertices, edges, faces and solids that changed, each modified into what
    /// stands for it now (empty when nothing was written).
    /// </summary>
    public History History()
    {
        CheckFinished();
        return _history ??= new History();
    }

    /// <summary>
    /// After <see cref="Finish"/>: the operation's own history over the input's sub-shapes, its results mapped with
    /// <see cref="Final"/>, with each sub-shape that changed in place but isn't in it modified into what stands for it.
    /// </summary>
    public History Composed(History? history)
    {
        CheckFinished();
        if (_final is null)
        {
            return history ?? new History();
        }

        var composed = new History();
        var subShapes = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        MapInput(subShapes);
        for (int i = 1; i <= subShapes.Extent(); i++)
        {
            Shape sub = subShapes[i];
            if (!global::Kerf.Engine.Topology.History.IsSupportedType(sub))
            {
                continue;
            }
            bool isModified = false;
            if (history is not null)
            {
                if (history.IsRemoved(sub))
                {
                    composed.Remove(sub);
                    continue;
                }
                foreach (Shape modified in history.Modified(sub))
                {
                    composed.AddModified(sub, Final(modified));
                    isModified = true;
                }
                foreach (Shape generated in history.Generated(sub))
                {
                    composed.AddGenerated(sub, Final(generated));
                }
            }
            if (!isModified)
            {
                Shape now = Final(sub);
                if (!ReferenceEquals(now.TShape(), sub.TShape()))
                {
                    composed.AddModified(sub, now);
                }
            }
        }
        return composed;
    }

    /// <summary>
    /// After <see cref="Finish"/>: each vertex, edge, face and solid of <paramref name="shapes"/> (shapes the operation
    /// worked with, as placed: its result's sub-shapes mapped before <see cref="Finish"/>, translated or glued copies of
    /// the input) whose TShape was replaced, modified into what stands for it now. Merged into a history kept over those
    /// shapes, it brings the history up to date, as <see cref="Composed"/> does for one over the input's sub-shapes.
    /// </summary>
    public History Images(IEnumerable<Shape> shapes)
    {
        CheckFinished();
        var images = new History();
        if (_final is null)
        {
            return images;
        }
        foreach (Shape shape in shapes)
        {
            if (shape.IsNull || !global::Kerf.Engine.Topology.History.IsSupportedType(shape))
            {
                continue;
            }
            Shape now = Final(shape);
            if (!ReferenceEquals(now.TShape(), shape.TShape()))
            {
                images.AddModified(shape, now);
            }
        }
        return images;
    }

    /// <summary>Closes the overlay: its working copies are dropped, and the frozen TShapes read their own data again.</summary>
    public void Dispose()
    {
        if (_isDisposed)
        {
            return;
        }
        _isDisposed = true;
        if (ReferenceEquals(s_current.Value, this))
        {
            s_current.Value = _parent;
        }
        Interlocked.Decrement(ref s_openCount);
    }

    // The input's sub-shapes (none when the operation was opened without one).
    private void MapInput(NCollection_IndexedMap<Shape> subShapes)
    {
        if (!_input.IsNull)
        {
            Exploration.MapShapes(_input, subShapes);
        }
    }

    private void CheckFinished()
    {
        if (!_isFinished)
        {
            throw new ProgramErrorException("WorkingOverlay: Finish not called");
        }
    }

    private Shape Assembled(in Shape shape)
    {
        if (shape.IsNull)
        {
            return shape;
        }
        Shape aShape = shape;
        aShape.TShape(Assemble(shape.TShape()!));
        return aShape;
    }

    // What stands for the TShape in the result: the operation's own TShapes get their children replaced in place; a
    // frozen TShape is itself unless it (or something below it) changed, then a new TShape: its working copy, or a
    // copy of its data, holding the assembled children.
    private TShape Assemble(TShape tShape)
    {
        if (_final!.TryGetValue(tShape, out TShape? done))
        {
            return done;
        }
        RuntimeHelpers.EnsureSufficientExecutionStack();

        if (!tShape.IsFrozen)
        {
            _final[tShape] = tShape; // before the children: a cycle (not expected) stops here
            SubstituteChildren(tShape, tShape._shapes);
            return tShape;
        }

        TShape? working = null;
        _working?.TryGetValue(tShape, out working);
        TShape before = ParentView(tShape);
        NCollection_List<Shape> children = (working ?? before)._shapes;
        var assembled = new List<Shape>(children.Extent());
        bool isChildChanged = false;
        for (var it = new NCollection_List<Shape>.Iterator(children); it.More(); it.Next())
        {
            Shape child = it.Value();
            TShape childTShape = child.TShape()!;
            TShape now = Assemble(childTShape);
            if (!ReferenceEquals(now, childTShape))
            {
                child.TShape(now);
                isChildChanged = true;
            }
            assembled.Add(child);
        }

        bool isDataChanged = working is not null
                             && (!IsSameChildren(before._shapes, working._shapes)
                                 || !BrepTools.IsSameData(before, working, _ignoreToleranceGuard));
        if (!isDataChanged && !isChildChanged)
        {
            _final[tShape] = tShape;
            return tShape;
        }

        TShape result = working ?? BrepTools.CopyData(before);
        if (working is null)
        {
            BrepTools.CopyFlags(before, result);
        }
        result._shapes.Clear();
        foreach (Shape child in assembled)
        {
            TShape childTShape = child.TShape()!;
            if (!childTShape.IsFrozen && !ReferenceEquals(childTShape, tShape))
            {
                childTShape.Free(false); // as TopoDS_Builder::Add clears it
            }
            result._shapes.Append(child);
        }
        _final[tShape] = result;
        return result;
    }

    // Replaces, in an unfrozen TShape's children, each one whose TShape is assembled into another.
    private void SubstituteChildren(TShape tShape, NCollection_List<Shape> children)
    {
        List<Shape>? changed = null;
        int index = 0;
        for (var it = new NCollection_List<Shape>.Iterator(children); it.More(); it.Next(), index++)
        {
            Shape child = it.Value();
            TShape childTShape = child.TShape()!;
            TShape now = Assemble(childTShape);
            if (!ReferenceEquals(now, childTShape) && changed is null)
            {
                changed = new List<Shape>(children.Extent());
                int j = 0;
                for (var it2 = new NCollection_List<Shape>.Iterator(children); j < index; it2.Next(), j++)
                {
                    changed.Add(it2.Value());
                }
            }
            if (changed is not null)
            {
                if (!ReferenceEquals(now, childTShape) && !now.IsFrozen)
                {
                    now.Free(false); // as TopoDS_Builder::Add clears it (unit 10H3)
                }
                child.TShape(now);
                changed.Add(child);
            }
        }
        if (changed is not null)
        {
            tShape._shapes.Clear();
            foreach (Shape child in changed)
            {
                tShape._shapes.Append(child);
            }
        }
    }

    private static bool IsSameChildren(NCollection_List<Shape> a, NCollection_List<Shape> b)
    {
        if (a.Extent() != b.Extent())
        {
            return false;
        }
        for (NCollection_List<Shape>.Iterator itA = new(a), itB = new(b); itA.More(); itA.Next(), itB.Next())
        {
            if (!itA.Value().IsEqual(itB.Value()))
            {
                return false;
            }
        }
        return true;
    }
}
