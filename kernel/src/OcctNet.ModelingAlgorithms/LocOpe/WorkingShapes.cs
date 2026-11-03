// OCCT.NET: the copy at the entry of LocOpe's local operations (unit 7H), which write into the shapes they are given
// (point representations added to vertices, pcurves to edges, tolerances raised, continuities set). Not a port of an
// OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;

using Kerf.Engine.Topology.Brep;

using Kerf.Engine.Topology.Tools;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Features.Local;

/// <summary>
/// The working shapes of one local operation: the copies for write of the frozen shapes it is given, shared by every
/// argument, and the way back to the caller's shapes at its end.
/// </summary>
/// <remarks>
/// <para>
/// A local operation (LocOpe_SplitShape, LocOpe_WiresOnShape with LocOpe_Spliter, LocOpe_Generator, LocOpe_Gluer) takes
/// a shape and other shapes that refer to it or lie on it (splitting edges and wires, vertices, a glued shape) and, as
/// OCCT does, writes into all of them. A shape a caller holds is frozen, so every shape entering the operation goes
/// through <see cref="Working"/>: a shape whose graph holds a frozen TShape is replaced by its exact copy for write
/// (<see cref="BrepTools.ExactCopyForWrite(in Shape, Dictionary{TShape, TShape})"/>), all copies
/// sharing one map, so a vertex of a splitting edge and the same vertex given alone, or a face given with the shape it
/// belongs to, have one copy; a shape without a frozen TShape (ported code's own) is worked on in place, as in OCCT.
/// </para>
/// <para>
/// <see cref="Finish"/> ends the operation: every copy it left equal to its original is replaced by that original in the
/// shapes it built (<see cref="BrepTools.KeepUnchangedIn"/>), so the result shares the caller's unchanged sub-shapes;
/// <see cref="Final(in Shape)"/> then maps a shape the operation worked with to what stands for it. The caller's shapes given to a
/// query are mapped to the working ones with <see cref="Copied"/>, which never copies. The operation's public accessors
/// freeze what they hand out.
/// </para>
/// </remarks>
internal sealed class WorkingShapes
{
    private readonly Dictionary<TShape, TShape> _copies = new(ReferenceEqualityComparer.Instance);    // original -> copy
    private readonly Dictionary<TShape, TShape> _originals = new(ReferenceEqualityComparer.Instance); // copy -> original
    // what ExactCopyForWrite reuses: the copies, and each copy as its own
    private readonly Dictionary<TShape, TShape> _known = new(ReferenceEqualityComparer.Instance);
    private readonly List<Shape> _originalRoots = [];
    private readonly List<Shape> _copyRoots = [];
    private Dictionary<TShape, TShape>? _final; // copy -> what stands for it after Finish
    private bool _isFinished;

    /// <summary>True when some shape was copied.</summary>
    public bool IsCopied => _copies.Count != 0;

    /// <summary>True after <see cref="Finish"/>.</summary>
    public bool IsFinished => _isFinished;

    /// <summary>
    /// The shape to work on in place of <paramref name="shape"/>: its copy if it was copied (it or a shape holding
    /// it), a new copy if its graph holds a frozen TShape, else the shape itself. After <see cref="Finish"/>, only
    /// <see cref="Copied"/>.
    /// </summary>
    public Shape Working(in Shape shape)
    {
        TShape? tShape = shape.TShape();
        if (tShape is null)
        {
            return shape;
        }
        if (_known.TryGetValue(tShape, out TShape? image))
        {
            return WithImage(shape, image);
        }
        if (_isFinished || !BrepTools.HoldsFrozen(shape))
        {
            return shape;
        }

        int nbKnown = _known.Count;
        Shape copy = BrepTools.ExactCopyForWrite(shape, _known);
        if (_known.Count != nbKnown)
        {
            // the new pairs: the TShapes of the shape's graph that weren't known
            var aNew = new List<KeyValuePair<TShape, TShape>>();
            foreach (KeyValuePair<TShape, TShape> pair in _known)
            {
                if (!ReferenceEquals(pair.Key, pair.Value) && !_copies.ContainsKey(pair.Key))
                {
                    aNew.Add(pair);
                }
            }
            foreach (KeyValuePair<TShape, TShape> pair in aNew)
            {
                _copies[pair.Key] = pair.Value;
                _originals[pair.Value] = pair.Key;
                _known[pair.Value] = pair.Value;
            }
        }
        _originalRoots.Add(shape);
        _copyRoots.Add(copy);
        return copy;
    }

    /// <summary>
    /// The working shape standing for <paramref name="shape"/> (a caller's shape given to a query): its copy if it
    /// was copied, else itself. Never copies.
    /// </summary>
    public Shape Copied(in Shape shape)
    {
        TShape? tShape = shape.TShape();
        return tShape is not null && _copies.TryGetValue(tShape, out TShape? image) ? WithImage(shape, image) : shape;
    }

    /// <summary>The caller's shape a working copy was made from (the shape itself if it isn't a copy).</summary>
    public Shape Original(in Shape shape)
    {
        TShape? tShape = shape.TShape();
        return tShape is not null && _originals.TryGetValue(tShape, out TShape? image) ? WithImage(shape, image) : shape;
    }

    /// <summary>
    /// Ends the operation: in the shapes <paramref name="shapes"/> it built or worked with (their unfrozen graphs, and
    /// each of them if it is a copy), every copy left equal to its original is replaced by the original. Once only;
    /// nothing is done when nothing was copied.
    /// </summary>
    public void Finish(IEnumerable<Shape> shapes)
    {
        if (_isFinished)
        {
            return;
        }
        _isFinished = true;
        if (_copies.Count == 0)
        {
            return;
        }

        // one unfrozen compound holds every shape, another the roots copied and a third their copies, so that
        // KeepUnchangedIn compares each copy with its original and replaces the kept ones in all the graphs at once.
        // The children are appended directly: BRep_Builder.Add would clear their Free flags.
        Shape held = Holder(shapes);
        Shape originals = Holder(_originalRoots);
        Shape copies = Holder(_copyRoots);
        _final = BrepTools.KeepUnchangedIn(originals, copies, _copies, ref held, out _);
    }

    /// <summary>
    /// After <see cref="Finish"/>, what stands for a shape the operation worked with: for a copy, its original if the
    /// operation left it unchanged, else the copy; any other shape is itself. Before, the shape itself.
    /// </summary>
    public Shape Final(in Shape shape)
    {
        TShape? tShape = shape.TShape();
        return tShape is not null && _final is not null && _final.TryGetValue(tShape, out TShape? image)
                   ? WithImage(shape, image)
                   : shape;
    }

    /// <summary>A new list of the <see cref="Final(in Shape)"/> shapes of <paramref name="list"/>.</summary>
    public NCollection_List<Shape> Final(NCollection_List<Shape> list)
    {
        var aList = new NCollection_List<Shape>();
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            aList.Append(Final(it.Value()));
        }
        return aList;
    }

    /// <summary>Freezes the shapes of a list a public accessor hands out, and returns it.</summary>
    public static NCollection_List<Shape> Published(NCollection_List<Shape> list)
    {
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            it.Value().TShape()?.FreezeGraph();
        }
        return list;
    }

    /// <summary>Freezes a shape a public accessor hands out, and returns it.</summary>
    public static Shape Published(Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    private static Shape Holder(IEnumerable<Shape> shapes)
    {
        var holder = new Compound();
        new BrepBuilder().MakeCompound(ref holder);
        TShape tShape = holder.TShape()!;
        foreach (Shape shape in shapes)
        {
            if (!shape.IsNull)
            {
                tShape._shapes.Append(shape);
            }
        }
        return holder;
    }

    private static Shape WithImage(in Shape shape, TShape image)
    {
        Shape aShape = shape;
        aShape.TShape(image);
        return aShape;
    }
}
