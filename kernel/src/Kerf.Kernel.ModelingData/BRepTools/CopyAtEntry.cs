// OCCT.NET: the copy at the entry of an algorithm that edits the shape it is given in place (healing, sewing,
// clean-up: docs/phase5-6-plan.md, section 2.7; decision 583), and the composition of its histories with the copy's.
// Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Tools;

/// <summary>
/// The copy for write of a shape at the entry of an in-place algorithm, and the way back to the caller's shapes at its
/// end.
/// </summary>
/// <remarks>
/// <para>
/// Shape Healing's algorithms (ShapeFix, ShapeUpgrade), sewing and the clean-up edit the shape they are given: they
/// raise tolerances, add pcurves, reorder wires in place, and record the shapes they rebuild in a
/// <see cref="ReShape"/> context. A shape a caller holds is frozen, so such an algorithm runs on a copy:
/// </para>
/// <list type="number">
/// <item><description><c>new BRepTools_CopyAtEntry(input, context)</c> makes
/// <see cref="BrepTools.ExactCopyForWrite(in Shape, out History)"/> of the input when its graph holds a
/// frozen TShape (else the algorithm runs in place, as OCCT's, on the unfrozen shape ported code owns). Requests
/// already recorded in the context for the input's sub-shapes are moved onto their copies.</description></item>
/// <item><description>The algorithm runs on <see cref="Shape"/>. Other arguments that refer to the input's sub-shapes
/// (a face of the input given with one of its wires, the shapes added one by one to sewing) are mapped with
/// <see cref="Copied"/>.</description></item>
/// <item><description><see cref="Finish"/> takes its result: every copied sub-shape the algorithm left equal to its
/// original is replaced by that original (<see cref="BrepTools.KeepUnchangedIn"/>), in the result and in the shapes the
/// context records, so a face that needed no fix is the caller's face, and a shape that needed nothing is the input
/// itself. The context's requests are moved back onto the input's sub-shapes, and a sub-shape the algorithm changed in
/// place without recording it (a tolerance raised, a pcurve added) is recorded as replaced by its changed copy. So the
/// context's <c>Value</c>, <c>Apply</c> and <c>History</c> answer for the shapes the caller holds.</description></item>
/// <item><description>A history the algorithm keeps itself, over the copy's sub-shapes (<c>Modified</c> of sewing,
/// UnifySameDomain's <c>History()</c>), is mapped onto the input's with <see cref="Composed"/>; single shapes with
/// <see cref="Final"/>.</description></item>
/// </list>
/// <para>
/// The algorithm's public accessors then freeze what they hand out, as every publication point does. The helper itself
/// freezes nothing.
/// </para>
/// </remarks>
internal sealed class CopyAtEntry
{
    private readonly Shape _input;
    private readonly Shape _copy;
    private ReShape? _context;
    private readonly Dictionary<TShape, TShape>? _copies;    // original -> copy
    private readonly Dictionary<TShape, TShape>? _originals; // copy -> original
    private Dictionary<TShape, TShape>? _final;              // copy -> what stands for it after Finish
    private History _history = new();
    private bool _isFinished;

    /// <summary>
    /// Copies <paramref name="input"/> for write if its graph holds a frozen TShape (see
    /// <see cref="BrepTools.HoldsFrozen"/>), or always with <paramref name="toForce"/>; the requests recorded in
    /// <paramref name="context"/> for the input's sub-shapes are moved onto their copies.
    /// </summary>
    public CopyAtEntry(in Shape input, ReShape? context = null, bool toForce = false)
    {
        _input = input;
        _context = context;
        if (input.IsNull || !(toForce || BrepTools.HoldsFrozen(input)))
        {
            _copy = input;
            return;
        }

        _copy = BrepTools.ExactCopyForWrite(input, out _, out Dictionary<TShape, TShape> copies);
        _copies = copies;
        _originals = new Dictionary<TShape, TShape>(ReferenceEqualityComparer.Instance);
        foreach (KeyValuePair<TShape, TShape> pair in copies)
        {
            _originals[pair.Value] = pair.Key;
        }

        context?.ReplaceTShapes(copies, NoImages);
    }

    private static readonly Dictionary<TShape, TShape> NoImages = new(ReferenceEqualityComparer.Instance);

    /// <summary>
    /// Gives the helper the context an algorithm created after the entry (ShapeFix_Face's FixMissingSeam creates one
    /// when it has none), so that <see cref="Finish"/> re-keys it too. Ignored when a context was given at the entry:
    /// a context created later holds requests on the copy's sub-shapes only, so nothing had to be moved at the entry.
    /// </summary>
    public void AttachContext(ReShape? context)
    {
        if (_context is null && !_isFinished)
        {
            _context = context;
        }
    }

    /// <summary>The input.</summary>
    public Shape Input() => _input;

    /// <summary>The shape to run the algorithm on: the copy, or the input itself when it wasn't copied.</summary>
    public Shape Shape() => _copy;

    /// <summary>True once <see cref="Finish"/> has run.</summary>
    public bool IsFinished => _isFinished;

    /// <summary>True if the input was copied.</summary>
    public bool IsCopied => _copies is not null;

    /// <summary>
    /// The copy of a sub-shape of the input, at the same location and orientation (the shape itself when the input
    /// wasn't copied, or when it isn't a sub-shape of the input).
    /// </summary>
    public Shape Copied(in Shape shape) => WithImage(shape, _copies);

    /// <summary>
    /// The input's sub-shape a copy was made from, at the copy's location and orientation (the shape itself when the
    /// input wasn't copied, or when it isn't a copy). The inverse of <see cref="Copied"/>.
    /// </summary>
    public Shape Original(in Shape shape) => WithImage(shape, _originals);

    /// <summary>
    /// After <see cref="Finish"/>: what stands for a shape the algorithm worked with: for a copied sub-shape, its
    /// original if the algorithm left it unchanged, else the changed copy; any other shape is itself (its sub-shapes
    /// were already replaced in place by <see cref="Finish"/> where it reached them).
    /// </summary>
    public Shape Final(in Shape shape)
    {
        CheckFinished();
        return WithImage(shape, _final);
    }

    /// <summary>
    /// After <see cref="Finish"/>: the input's vertices, edges, faces and solids whose copies the algorithm changed in
    /// place, each modified into its changed copy (empty when the input wasn't copied).
    /// </summary>
    public History History()
    {
        CheckFinished();
        return _history;
    }

    /// <summary>
    /// Ends the algorithm: the copied sub-shapes it left equal to their originals are replaced by them in
    /// <paramref name="result"/> (which may be one of them) and in the context's replacements, the context's
    /// requests are moved back onto the input's sub-shapes, and the sub-shapes changed in place but not recorded are
    /// recorded as replaced by their copies. Returns the result, unfrozen. When the input wasn't copied, returns
    /// <paramref name="result"/> and changes nothing.
    /// </summary>
    /// <param name="result">The algorithm's result (null if it has none).</param>
    /// <param name="others">Other shapes the algorithm hands out (free edges, multiple edges, ...): their sub-shapes
    /// are replaced in place too, and <see cref="Final"/> maps them.</param>
    public Shape Finish(in Shape result, IEnumerable<Shape>? others = null)
    {
        if (_isFinished)
        {
            throw new ProgramErrorException("BRepTools_CopyAtEntry::Finish called twice");
        }
        _isFinished = true;
        if (_copies is null)
        {
            _final = null;
            return result;
        }

        // Every shape that may hold copies goes into one unfrozen compound, so that KeepUnchangedIn replaces the kept
        // copies in all their graphs at once (it doesn't enter frozen TShapes, which can't hold copies). The children
        // are appended directly: BRep_Builder.Add would clear their Free flags.
        var holder = new Compound();
        new BrepBuilder().MakeCompound(ref holder);
        TShape holderTShape = holder.TShape()!;
        if (!result.IsNull)
        {
            holderTShape._shapes.Append(result);
        }
        if (others is not null)
        {
            foreach (Shape other in others)
            {
                if (!other.IsNull)
                {
                    holderTShape._shapes.Append(other);
                }
            }
        }
        if (_context is not null)
        {
            foreach (Shape replacement in _context.InternalReplacements())
            {
                if (!replacement.IsNull)
                {
                    holderTShape._shapes.Append(replacement);
                }
            }
        }

        Shape held = holder;
        _final = BrepTools.KeepUnchangedIn(_input, _copy, _copies, ref held, out _history);
        Shape aResult = WithImage(result, _final);

        if (_context is not null)
        {
            // the requests back onto the input's sub-shapes, their replacements onto the final shapes
            _context.ReplaceTShapes(_originals!, _final);

            // the changes made in place without a request
            var subShapes = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(_input, subShapes);
            for (int i = 1; i <= subShapes.Extent(); i++)
            {
                Shape sub = subShapes[i];
                Shape now = WithImage(Copied(sub), _final);
                if (!ReferenceEquals(now.TShape(), sub.TShape()) && !_context.IsRecorded(sub))
                {
                    _context.Replace(sub, now);
                }
            }
        }

        return aResult;
    }

    /// <summary>
    /// After <see cref="Finish"/>: the history <paramref name="history"/> of the algorithm over the copy's
    /// sub-shapes as a history over the input's. For each vertex, edge, face and solid of the input: removed if its
    /// copy was; modified into (and generated) the final shapes of its copy's; else, if its copy was changed in place,
    /// modified into it. When the input wasn't copied, <paramref name="history"/> itself (a new empty history for
    /// null). Entries of <paramref name="history"/> for shapes that aren't copies (shapes the algorithm made) aren't
    /// carried over.
    /// </summary>
    public History Composed(History? history)
    {
        CheckFinished();
        if (_copies is null)
        {
            return history ?? new History();
        }

        var composed = new History();
        var subShapes = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(_input, subShapes);
        for (int i = 1; i <= subShapes.Extent(); i++)
        {
            Shape sub = subShapes[i];
            if (!global::Kerf.Engine.Topology.History.IsSupportedType(sub))
            {
                continue;
            }

            Shape copy = Copied(sub);
            bool isModified = false;
            if (history is not null)
            {
                if (history.IsRemoved(copy))
                {
                    composed.Remove(sub);
                    continue;
                }
                foreach (Shape modified in history.Modified(copy))
                {
                    composed.AddModified(sub, WithImage(modified, _final));
                    isModified = true;
                }
                foreach (Shape generated in history.Generated(copy))
                {
                    composed.AddGenerated(sub, WithImage(generated, _final));
                }
            }
            if (!isModified)
            {
                Shape now = WithImage(copy, _final);
                if (!ReferenceEquals(now.TShape(), sub.TShape()))
                {
                    composed.AddModified(sub, now);
                }
            }
        }
        return composed;
    }

    private void CheckFinished()
    {
        if (!_isFinished)
        {
            throw new ProgramErrorException("BRepTools_CopyAtEntry: Finish not called");
        }
    }

    // The shape with the image of its TShape, if it has one.
    private static Shape WithImage(in Shape shape, Dictionary<TShape, TShape>? images)
    {
        TShape? tShape = shape.TShape();
        if (tShape is null || images is null || !images.TryGetValue(tShape, out TShape? image))
        {
            return shape;
        }
        Shape aShape = shape;
        aShape.TShape(image);
        return aShape;
    }
}
