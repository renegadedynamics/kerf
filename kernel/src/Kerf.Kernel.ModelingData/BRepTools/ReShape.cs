// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_ReShape.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_ReShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Topology;

/// <summary>
/// Records requests to replace or remove shapes, and applies them to a shape: the parents of the replaced or removed
/// sub-shapes are rebuilt. Keeps the history of the substitutions (<see cref="History"/>).
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21): the requests are recorded through public methods. By default a shape is recorded
/// whatever its location; with <see cref="ModeConsiderLocation(bool)"/> set, the location is taken into account.
/// </para>
/// <para>
/// <see cref="Apply"/>, <see cref="Value"/>, <see cref="ValueLeaf"/>, <see cref="Status"/> and <see cref="History"/>
/// hand out shapes that may have been built here, so they freeze the graphs they return (docs/phase3-design.md,
/// section 4.2). Ported code that keeps building on them uses the internal, non-freezing forms
/// (<see cref="InternalApply"/>, <see cref="InternalValue"/>, ...), which a derived class (ShapeBuild_ReShape)
/// overrides.
/// </para>
/// </remarks>
public class ReShape : Transient
{
    /// <summary>The kinds of the replacements.</summary>
    protected enum TReplacementKind
    {
        TReplacementKind_Remove = 1,
        TReplacementKind_Modify = 2,
        TReplacementKind_Merge_Main = 4,
        TReplacementKind_Merge_Ordinary = 8
    }

    // A replacement of an initial shape.
    private readonly struct TReplacement
    {
        private readonly Shape _result; // The result of the replacement.
        private readonly TReplacementKind _kind; // The kind of the replacement.

        // The initializing constructor.
        public TReplacement(in Shape result, TReplacementKind kind)
        {
            _result = result;
            _kind = kind;
        }

        // Returns the result of the replacement.
        public Shape Result() => (_kind != TReplacementKind.TReplacementKind_Merge_Ordinary) ? _result : new Shape();

        // Returns the result of the relation.
        public Shape RelationResult() => _result;

        // OCCT.NET: the kind, for ReplaceTShapes.
        public TReplacementKind Kind() => _kind;

        // Returns the kind of the relation
        // between an initial shape and the result of the replacement.
        public global::Kerf.Engine.Topology.History.TRelationType RelationKind() => (_kind == TReplacementKind.TReplacementKind_Remove)
                                                                     ? global::Kerf.Engine.Topology.History.TRelationType.TRelationType_Removed
                                                                     : global::Kerf.Engine.Topology.History.TRelationType.TRelationType_Modified;
    }

    // Maps each shape to its replacement.
    // If a shape is not bound to the map then the shape is replaced by itself.
    private readonly NCollection_DataMap<Shape, TReplacement> _shapeToReplacement = new(ShapeHasher.Instance);

    /// <summary>The shapes recorded as replacements.</summary>
    protected readonly HashSet<Shape> _newShapes = new(ShapeHasher.Instance);

    /// <summary>The status of the last application (ShapeExtend's encoding).</summary>
    protected int _status;

    private bool _considerLocation;

    /// <summary>An empty ReShape.</summary>
    public ReShape()
    {
        _status = -1;
        _considerLocation = false;
    }

    /// <summary>Clears all substitution requests.</summary>
    public virtual void Clear()
    {
        _shapeToReplacement.Clear();
        _newShapes.Clear();
    }

    /// <summary>Records a request to remove a shape, whatever its orientation.</summary>
    public virtual void Remove(in Shape shape)
    {
        Shape nulshape = new();
        ReplaceCore(shape, nulshape, TReplacementKind.TReplacementKind_Remove);
    }

    /// <summary>Records a request to replace a shape by a new one.</summary>
    public virtual void Replace(in Shape shape, in Shape newshape)
    {
        ReplaceCore(shape, newshape, TReplacementKind.TReplacementKind_Modify);
    }

    /// <summary>
    /// Merges the parts into the single product: the first part is replaced by the product, the others are removed.
    /// The history presents the merged shapes alike.
    /// </summary>
    public void Merge(IEnumerable<Shape> parts, in Shape product)
    {
        using IEnumerator<Shape> pIt = parts.GetEnumerator();

        bool more = pIt.MoveNext();
        if (more)
        {
            ReplaceCore(pIt.Current, product, TReplacementKind.TReplacementKind_Merge_Main);

            more = pIt.MoveNext();
        }

        const TReplacementKind kind = TReplacementKind.TReplacementKind_Merge_Ordinary;
        for (; more; more = pIt.MoveNext())
        {
            ReplaceCore(pIt.Current, product, kind);
        }
    }

    /// <summary>True if a replacement or a removal is recorded for the shape.</summary>
    public virtual bool IsRecorded(in Shape ashape)
    {
        Shape shape = ashape;
        if (_considerLocation)
        {
            Placement nullLoc = new();
            shape.Location(nullLoc);
        }
        if (shape.IsNull)
        {
            return false;
        }
        return _shapeToReplacement.IsBound(shape);
    }

    /// <summary>
    /// The new value of a shape: the shape itself if not recorded, a null shape if it is to be removed, else its
    /// replacement. Frozen.
    /// </summary>
    public Shape Value(in Shape shape) => Published(InternalValue(shape));

    /// <summary>The new value of a shape, as <see cref="Value"/>, without freezing it (ported code).</summary>
    internal virtual Shape InternalValue(in Shape ashape)
    {
        Shape res = new();
        if (ashape.IsNull)
        {
            return res;
        }
        Shape shape = ashape;
        if (_considerLocation)
        {
            Placement nullLoc = new();
            shape.Location(nullLoc);
        }

        bool fromMap = false;
        if (!_shapeToReplacement.IsBound(shape))
        {
            res = shape;
        }
        else
        {
            res = _shapeToReplacement[shape].Result();
            if (shape.Orientation() == Reversed)
            {
                res.Reverse();
            }
            fromMap = true;
        }
        // for INTERNAL/EXTERNAL, since they are not fully supported, keep orientation
        if (shape.Orientation() == Internal || shape.Orientation() == External)
        {
            res.Orientation(shape.Orientation());
        }

        if (_considerLocation)
        {
            // sln 29.11.01 Bug22: Recalculate location of resulting shape in accordance with
            // whether result is from map or not
            if (fromMap)
            {
                res.Location(ashape.Location() * res.Location(), false);
            }
            else
            {
                res.Location(ashape.Location(), false);
            }
        }

        return res;
    }

    /// <summary>
    /// Follows the replacement chain of the shape to its end without rebuilding sub-shapes: the final replacement,
    /// the shape itself if not recorded, or a null shape if the chain ends in a removal. Frozen.
    /// </summary>
    public Shape ValueLeaf(in Shape shape) => Published(InternalValueLeaf(shape));

    /// <summary>The end of the replacement chain, as <see cref="ValueLeaf"/>, without freezing it (ported code).</summary>
    internal Shape InternalValueLeaf(in Shape shape)
    {
        if (shape.IsNull)
        {
            return new Shape();
        }

        // Track visited shapes by their underlying TShape. Rationale: the replacement map keys
        // entries via TopTools_ShapeMapHasher (orientation-ignoring IsSame) and, when
        // myConsiderLocation is set, strips locations on both insertion and lookup. That leaves
        // TShape as the only identity axis that's stable across the walk under every mode -
        // so keying the cycle guard on the TShape handle catches cycles that would otherwise
        // slip past orientation/location-sensitive comparisons.
        Shape current = shape;
        var visited = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        visited.Add(current.TShape()!);

        for (;;)
        {
            Shape next = InternalValue(current);
            if (next.IsNull)
            {
                return next;
            }
            if (next.IsSame(current))
            {
                return next;
            }
            if (!visited.Add(next.TShape()!))
            {
                // Cycle in replacement data - return current best to avoid looping.
                return next;
            }
            current = next;
        }
    }

    /// <summary>
    /// The substitution status of a shape: 0 if not recorded (<paramref name="newsh"/> is the shape), &lt; 0 if it is
    /// to be removed (<paramref name="newsh"/> is null), &gt; 0 if it is to be replaced (<paramref name="newsh"/> is
    /// the replacement). With <paramref name="last"/>, a replaced shape's replacement is applied recursively.
    /// <paramref name="newsh"/> is frozen.
    /// </summary>
    public int Status(in Shape shape, out Shape newsh, bool last = false)
    {
        int res = InternalStatus(shape, out newsh, last);
        newsh = Published(newsh);
        return res;
    }

    /// <summary>The substitution status, as <see cref="Status"/>, without freezing (ported code).</summary>
    internal virtual int InternalStatus(in Shape ashape, out Shape newsh, bool last = false)
    {
        int res = 0;
        newsh = new Shape();
        if (ashape.IsNull)
        {
            newsh.Nullify();
            return res;
        }

        Shape shape = ashape;
        Placement locSh = shape.Location();
        if (_considerLocation)
        {
            Placement nullLoc = new();
            shape.Location(nullLoc);
        }

        if (!_shapeToReplacement.IsBound(shape))
        {
            newsh = shape;
            res = 0;
        }
        else
        {
            newsh = _shapeToReplacement[shape].Result();
            res = 1;
        }
        if (res > 0)
        {
            if (newsh.IsNull)
            {
                res = -1;
            }
            else if (newsh.IsEqual(shape))
            {
                res = 0;
            }
            else if (last
                     && ((_considerLocation && !newsh.IsPartner(shape))
                         || (!_considerLocation && !newsh.IsSame(shape))))
            {
                //  sln 29.11.01 Bug24: Correction iteration through maps. Way of iteration used early does
                //  not correspond to way of storing information in the maps.
                newsh = InternalApply(shape, ShapeType.Shape);
                if (newsh.IsNull)
                {
                    res = -1;
                }
                if (newsh.IsEqual(shape))
                {
                    res = 0;
                }
            }
        }
        if (_considerLocation && !newsh.IsNull)
        {
            Placement resLoc =
                (res > 0 && !newsh.Location().IsIdentity ? locSh * newsh.Location() : locSh);
            newsh.Location(resLoc, false);
        }
        return res;
    }

    private static int EncodeStatus(int status)
    {
        switch (status)
        {
            case 0:
                return 0x0000; // ShapeExtend_OK
            case 1:
                return 0x0001; // ShapeExtend_DONE1
            case 2:
                return 0x0002; //....
            case 3:
                return 0x0004;
            case 4:
                return 0x0008;
            case 5:
                return 0x0010;
            case 6:
                return 0x0020;
            case 7:
                return 0x0040;
            case 8:
                return 0x0080; //....
            case 9:
                return 0x00ff; // ShapeExtend_DONE
            case 10:
                return 0x0100; // ShapeExtend_FAIL1
            case 11:
                return 0x0200; //...
            case 12:
                return 0x0400;
            case 13:
                return 0x0800;
            case 14:
                return 0x1000;
            case 15:
                return 0x2000;
            case 16:
                return 0x4000;
            case 17:
                return 0x8000; //....
            case 18:
                return 0xff00; // ShapeExtend_FAIL
        }
        return 0;
    }

    /// <summary>
    /// Applies the substitution requests to a shape, rebuilding the parents of replaced or removed sub-shapes. Sub-shapes
    /// of type <paramref name="until"/> are neither rebuilt nor explored. A sub-shape can be replaced by a shape of the
    /// same type or by a container of shapes of that type; other types are ignored (status FAIL1). The result is frozen.
    /// </summary>
    public Shape Apply(in Shape shape, ShapeType until = ShapeType.Shape) =>
        Published(InternalApply(shape, until));

    /// <summary>Applies the substitution requests, as <see cref="Apply"/>, without freezing the result (ported code).</summary>
    internal virtual Shape InternalApply(in Shape shape, ShapeType until = ShapeType.Shape)
    {
        var inFlight = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        return ApplyImpl(shape, until, inFlight);
    }

    /// <summary>
    /// The recursive worker of Apply, with a set of the TShapes being processed further up (by identity), so that a
    /// replacement containing the original shape doesn't recurse without end.
    /// </summary>
    protected Shape ApplyImpl(in Shape shape, ShapeType until, HashSet<TShape> inFlight)
    {
        _status = EncodeStatus(0); // ShapeExtend::EncodeStatus ( ShapeExtend_OK );
        if (shape.IsNull)
        {
            return shape;
        }

        // apply direct replacement
        Shape newsh = InternalValue(shape);

        // if shape removed, return NULL
        if (newsh.IsNull)
        {
            _status = EncodeStatus(2); // ShapeExtend_DONE2
            return newsh;
        }

        // DFS cycle guard: if shape is already being processed further up the call
        // stack, its replacement must be a compound that transitively contains it.
        // Return the direct replacement without descending to break the cycle.
        if (inFlight.Contains(shape.TShape()!))
        {
            return newsh;
        }

        // if shape replaced, apply modifications to the result recursively
        if ((_considerLocation && !newsh.IsPartner(shape))
            || (!_considerLocation && !newsh.IsSame(shape)))
        {
            inFlight.Add(shape.TShape()!);
            Shape res = ApplyImpl(newsh, until, inFlight);
            inFlight.Remove(shape.TShape()!);
            _status |= EncodeStatus(1); // ShapeExtend::EncodeStatus ( ShapeExtend_DONE1 );
            return res;
        }

        ShapeType st = shape.ShapeType();
        if (st > until || (st == until && until > ShapeType.Compound))
        {
            return newsh; // stopping criteria
        }
        if (st == ShapeType.Vertex || st == ShapeType.Shape)
        {
            return shape;
        }
        // define allowed types of components
        // fix for SAMTECH bug OCC322 about absent internal vertices after sewing.
        BrepBuilder B = new();

        Shape result = shape.EmptyCopied();
        Orientation orien = shape.Orientation();
        result.Orientation(Forward); // protect against INTERNAL or EXTERNAL shapes
        bool modif = false;
        int locStatus = _status;

        // apply recorded modifications to subshapes
        bool isEmpty = true;
        inFlight.Add(shape.TShape()!);
        for (var it = new Iterator(shape, false); it.More(); it.Next())
        {
            Shape sh = it.Value();
            newsh = ApplyImpl(sh, until, inFlight);
            if (newsh != sh)
            {
                if ((_status & EncodeStatus(4)) != 0)
                { // ShapeExtend::DecodeStatus ( myStatus, ShapeExtend_DONE4 ) )
                    locStatus |= EncodeStatus(4); //|= ShapeExtend::EncodeStatus ( ShapeExtend_DONE4 );
                }
                modif = true;
            }
            if (newsh.IsNull)
            {
                locStatus |= EncodeStatus(4); // ShapeExtend::EncodeStatus ( ShapeExtend_DONE4 );
                continue;
            }
            if (isEmpty)
            {
                isEmpty = false;
            }
            locStatus |= EncodeStatus(3); // ShapeExtend::EncodeStatus ( ShapeExtend_DONE3 );
            if (st == ShapeType.Compound || newsh.ShapeType() == sh.ShapeType())
            { // fix for SAMTECH bug OCC322 about absent internal vertices after sewing.
                B.Add(result, newsh);
                continue;
            }
            int nitems = 0;
            for (var subit = new Iterator(newsh); subit.More(); subit.Next(), nitems++)
            {
                Shape subsh = subit.Value();
                if (subsh.ShapeType() == sh.ShapeType())
                {
                    B.Add(result, subsh); //fix for SAMTECH bug OCC322 about absent internal vertices after sewing.
                }
                else
                {
                    locStatus |= EncodeStatus(10); //ShapeExtend::EncodeStatus ( ShapeExtend_FAIL1 );
                }
            }
            if (nitems == 0)
            {
                locStatus |= EncodeStatus(10); //ShapeExtend::EncodeStatus ( ShapeExtend_FAIL1 );
            }
        }
        inFlight.Remove(shape.TShape()!);
        if (!modif)
        {
            return shape;
        }

        // For empty topological containers (any kind of shape except vertex, edge
        // and face) we have to produce an empty result
        if (isEmpty && st != ShapeType.Vertex && st != ShapeType.Edge && st != ShapeType.Face)
        {
            result = new Shape();
        }
        else
        {
            // restore Range on edge broken by EmptyCopied()
            if (st == ShapeType.Edge)
            {
                CopyRanges(result, shape, 0, 1);
            }
            else if (st == ShapeType.Face)
            {
                Face face = ShapeCasts.Face(shape);
                if (Tool.NaturalRestriction(face))
                {
                    BrepBuilder b = new();
                    b.NaturalRestriction(ShapeCasts.Face(result), true);
                }
            }
            else if (st == ShapeType.Wire || st == ShapeType.Shell)
            {
                result.Closed(Tool.IsClosed(result));
            }

            result.Orientation(orien);
        }

        ReplaceCore(shape, result, result.IsNull ? TReplacementKind.TReplacementKind_Remove : TReplacementKind.TReplacementKind_Modify);
        _status = locStatus;

        return result;
    }

    /// <summary>True if the location of a shape is taken into account when recording and looking it up.</summary>
    public virtual bool ModeConsiderLocation() => _considerLocation;

    /// <summary>Sets whether the location of a shape is taken into account (OCCT's modifiable <c>bool&amp;</c>).</summary>
    public virtual void ModeConsiderLocation(bool mode) => _considerLocation = mode;

    /// <summary>
    /// A copy of the vertex with a new tolerance if it isn't recorded (then recorded as its replacement), or its
    /// recorded replacement changed in place. Internal: it returns an unfrozen vertex, and changes a recorded one in
    /// place (which throws <see cref="LockedShapeException"/> on a frozen one, as OCCT does on a locked one).
    /// </summary>
    internal Vertex CopyVertex(in Vertex v, double tol = -1.0)
    {
        return CopyVertex(v, Tool.Pnt(v), tol);
    }

    /// <summary>
    /// A copy of the vertex at a new position, as <see cref="CopyVertex(in Vertex, double)"/>.
    /// </summary>
    internal Vertex CopyVertex(in Vertex v, in Point3D newPos, double tol)
    {
        Vertex vertexCopy;
        bool isRecorded = IsRecorded(v);
        vertexCopy = isRecorded ? ShapeCasts.Vertex(InternalApply(v)) : ShapeCasts.Vertex(v.EmptyCopied());

        BrepBuilder b = new();
        double newTol = tol > 0.0 ? tol : Tool.Tolerance(v);
        b.UpdateVertex(vertexCopy, newPos, newTol);

        if (!isRecorded)
        {
            Replace(v, vertexCopy);
        }

        return vertexCopy;
    }

    /// <summary>True if the shape has been recorded as a replacement.</summary>
    public bool IsNewShape(in Shape shape) => _newShapes.Contains(shape);

    /// <summary>
    /// The history of the substitutions: each recorded shape (not itself a replacement) is modified into the shapes at
    /// the ends of its replacement chain, or removed. The shapes it lists are frozen.
    /// </summary>
    public History History() => HistoryCore(true);

    /// <summary>The history of the substitutions, as <see cref="History"/>, without freezing (ported code).</summary>
    internal History InternalHistory() => HistoryCore(false);

    private History HistoryCore(bool toFreeze)
    {
        History history = new();

        // Fill the history.
        foreach (KeyValuePair<Shape, TReplacement> rIt in _shapeToReplacement)
        {
            Shape shape = rIt.Key;
            if (!global::Kerf.Engine.Topology.History.IsSupportedType(shape) || _newShapes.Contains(shape))
            {
                continue;
            }

            // OCCT.NET: OCCT's NCollection_IndexedMap<TopoDS_Shape> and NCollection_Map<TopoDS_Shape> without a hasher
            // use std::hash and operator==, which are IsEqual.
            var intermediates = new NCollection_IndexedMap<Shape>(EqualityComparer<Shape>.Default);
            var modified = new HashSet<Shape>(EqualityComparer<Shape>.Default);
            intermediates.Add(shape);
            for (int i = 1; i <= intermediates.Extent(); ++i)
            {
                Shape intermediate = intermediates[i];
                if (!_shapeToReplacement.Find(intermediate, out TReplacement replacement))
                {
                    Add(modified, intermediate);
                }
                else if (replacement.RelationKind() != global::Kerf.Engine.Topology.History.TRelationType.TRelationType_Removed)
                {
                    Shape result = replacement.RelationResult();
                    if (!result.IsNull)
                    {
                        Add(intermediates, result);
                    }
                }
            }

            if (modified.Count == 0)
            {
                history.Remove(shape);
            }
            else
            {
                foreach (Shape it in modified)
                {
                    history.AddModified(shape, it);
                    if (toFreeze)
                    {
                        Published(it);
                    }
                }
            }
        }

        return history;
    }

    /// <summary>
    /// Replaces the first shape by the second after reorienting them: if the first is REVERSED both are reversed; if it
    /// is INTERNAL or EXTERNAL, the second is oriented FORWARD (REVERSED) if its orientation is (isn't) the first's,
    /// and the first is oriented FORWARD.
    /// </summary>
    protected virtual void ReplaceCore(in Shape ashape, in Shape anewshape, TReplacementKind kind)
    {
        Shape shape = ashape;
        Shape newshape = anewshape;
        if (shape.IsNull || shape == newshape)
        {
            return;
        }

        if (shape.Orientation() == Reversed)
        {
            shape.Reverse();
            newshape.Reverse();
        }
        // protect against INTERNAL or EXTERNAL shape
        else if (shape.Orientation() == Internal || shape.Orientation() == External)
        {
            newshape.Orientation((newshape.Orientation() == shape.Orientation()) ? Forward
                                                                                 : Reversed);
            shape.Orientation(Forward);
        }

        if (_considerLocation)
        {
            // sln 29.11.01 Bug22: Change location of 'newshape' in accordance with location of 'shape'
            newshape.Location(newshape.Location().Multiplied(shape.Location().Inverted()), false);
            Placement nullLoc = new();
            shape.Location(nullLoc);
        }

        // Cycle handling: cycles in the replacement map (A -> B -> A or longer) are
        // accepted at insertion time. The DFS in-flight guard inside Apply()/applyImpl()
        // breaks any cycle at traversal time, returning the immediate Value() instead
        // of recursing.
        _shapeToReplacement.Bind(shape, new TReplacement(newshape, kind));
        _newShapes.Add(newshape);
    }

    /// <summary>
    /// OCCT.NET (decision 583, <c>BRepTools_CopyAtEntry</c>): replaces, in every recorded request, the TShape of the
    /// recorded shape by its image in <paramref name="keys"/> and the TShape of the replacement (and of the
    /// recorded replacements, <see cref="IsNewShape"/>) by its image in <paramref name="results"/>, keeping their
    /// locations and orientations; TShapes without an image stay. The requests keep their order and kinds; if two
    /// recorded shapes get the same image, the later request wins. So a context filled by an algorithm working on a
    /// copy answers for the shapes the copy was made from, and the other way round.
    /// </summary>
    internal void ReplaceTShapes(IReadOnlyDictionary<TShape, TShape> keys,
                                 IReadOnlyDictionary<TShape, TShape> results)
    {
        var entries = new List<KeyValuePair<Shape, TReplacement>>(_shapeToReplacement);
        _shapeToReplacement.Clear();
        foreach (KeyValuePair<Shape, TReplacement> entry in entries)
        {
            Shape key = WithImage(entry.Key, keys);
            Shape result = WithImage(entry.Value.RelationResult(), results);
            _shapeToReplacement.Bind(key, new TReplacement(result, entry.Value.Kind()));
        }

        var newShapes = new List<Shape>(_newShapes);
        _newShapes.Clear();
        foreach (Shape aNew in newShapes)
        {
            _newShapes.Add(WithImage(aNew, results));
        }
    }

    /// <summary>
    /// OCCT.NET: the replacements recorded (the results of the requests, null for removals), unfrozen, for
    /// <c>BRepTools_CopyAtEntry</c>.
    /// </summary>
    internal IEnumerable<Shape> InternalReplacements()
    {
        foreach (KeyValuePair<Shape, TReplacement> entry in _shapeToReplacement)
        {
            yield return entry.Value.RelationResult();
        }
    }

    // The shape with the image of its TShape, if it has one.
    private static Shape WithImage(in Shape shape, IReadOnlyDictionary<TShape, TShape> images)
    {
        TShape? tShape = shape.TShape();
        if (tShape is null || !images.TryGetValue(tShape, out TShape? image))
        {
            return shape;
        }
        Shape aShape = shape;
        aShape.TShape(image);
        return aShape;
    }

    // OCCT's private isOrdinaryMerged is never called and isn't ported.

    // Freezes the graph of a shape handed to a caller (docs/phase3-design.md, section 4.2).
    private static Shape Published(in Shape shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    // Adds the shape to the map.
    // If the shape is a wire, shell or solid then
    // adds the sub-shapes of the shape instead.
    private static void Add(HashSet<Shape> map, in Shape shape)
    {
        ShapeType type = shape.ShapeType();
        if (type != ShapeType.Wire && type != ShapeType.Shell && type != ShapeType.CompSolid)
        {
            map.Add(shape);
            return;
        }

        for (var it = new Iterator(shape); it.More(); it.Next())
        {
            map.Add(it.Value());
        }
    }

    // The same, for an indexed map.
    private static void Add(NCollection_IndexedMap<Shape> map, in Shape shape)
    {
        ShapeType type = shape.ShapeType();
        if (type != ShapeType.Wire && type != ShapeType.Shell && type != ShapeType.CompSolid)
        {
            map.Add(shape);
            return;
        }

        for (var it = new Iterator(shape); it.More(); it.Next())
        {
            map.Add(it.Value());
        }
    }

    // Sets the ranges of the 3D curve and the pcurves of toedge (a new edge, EmptyCopied from fromedge) to the
    // ranges of fromedge's, scaled to [alpha, beta].
    private static void CopyRanges(in Shape toedge, in Shape fromedge, double alpha, double beta)
    {
        BrepTEdge tEdgeFrom = (BrepTEdge)fromedge.TShape()!;
        BrepTEdge tEdgeTo = (BrepTEdge)toedge.TShape()!;
        NCollection_List<CurveRepresentation> tolist = tEdgeTo.ChangeCurves();
        // OCCT.NET: frozen: OCCT iterates fromedge's ChangeCurves() only to read them; the port reads Curves(), so a
        // frozen fromedge is accepted.
        var fromitcr = new NCollection_List<CurveRepresentation>.Iterator(tEdgeFrom.Curves());
        for (; fromitcr.More(); fromitcr.Next())
        {
            if (fromitcr.Value() is not GCurve fromGC)
            {
                continue;
            }
            bool isC3d = fromGC.IsCurve3D;
            if (isC3d)
            {
                if (fromGC.Curve3D() is null)
                {
                    continue;
                }
            }
            else
            {
                if (fromGC.PCurve() is null)
                {
                    continue;
                }
            }

            if (!isC3d && !fromGC.IsCurveOnSurface())
            {
                continue; // only 3d curves and pcurves are treated
            }

            Surface? surface = null;
            Placement l = new();
            if (!isC3d)
            {
                surface = fromGC.Surface();
                l = fromGC.Location();
            }

            for (var toitcr = new NCollection_List<CurveRepresentation>.Iterator(tolist); toitcr.More(); toitcr.Next())
            {
                if (toitcr.Value() is not GCurve toGC)
                {
                    continue;
                }
                if (isC3d)
                {
                    if (!toGC.IsCurve3D)
                    {
                        continue;
                    }
                }
                else if (!toGC.IsCurveOnSurface() || !ReferenceEquals(surface, toGC.Surface()) || l != toGC.Location())
                {
                    continue;
                }
                double first = fromGC.First();
                double last = fromGC.Last();
                double len = last - first;
                toGC.SetRange(first + alpha * len, first + beta * len);
                break;
            }
        }
    }
}
