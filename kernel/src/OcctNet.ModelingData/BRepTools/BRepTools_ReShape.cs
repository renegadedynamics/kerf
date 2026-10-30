// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_ReShape.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_ReShape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

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
public class BRepTools_ReShape : Standard_Transient
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
        private readonly TopoDS_Shape myResult; // The result of the replacement.
        private readonly TReplacementKind myKind; // The kind of the replacement.

        // The initializing constructor.
        public TReplacement(in TopoDS_Shape theResult, TReplacementKind theKind)
        {
            myResult = theResult;
            myKind = theKind;
        }

        // Returns the result of the replacement.
        public TopoDS_Shape Result() => (myKind != TReplacementKind.TReplacementKind_Merge_Ordinary) ? myResult : new TopoDS_Shape();

        // Returns the result of the relation.
        public TopoDS_Shape RelationResult() => myResult;

        // OCCT.NET: the kind, for ReplaceTShapes.
        public TReplacementKind Kind() => myKind;

        // Returns the kind of the relation
        // between an initial shape and the result of the replacement.
        public BRepTools_History.TRelationType RelationKind() => (myKind == TReplacementKind.TReplacementKind_Remove)
                                                                     ? BRepTools_History.TRelationType.TRelationType_Removed
                                                                     : BRepTools_History.TRelationType.TRelationType_Modified;
    }

    // Maps each shape to its replacement.
    // If a shape is not bound to the map then the shape is replaced by itself.
    private readonly NCollection_DataMap<TopoDS_Shape, TReplacement> myShapeToReplacement = new(TopTools_ShapeMapHasher.Instance);

    /// <summary>The shapes recorded as replacements.</summary>
    protected readonly NCollection_Map<TopoDS_Shape> myNewShapes = new(TopTools_ShapeMapHasher.Instance);

    /// <summary>The status of the last application (ShapeExtend's encoding).</summary>
    protected int myStatus;

    private bool myConsiderLocation;

    /// <summary>An empty ReShape.</summary>
    public BRepTools_ReShape()
    {
        myStatus = -1;
        myConsiderLocation = false;
    }

    /// <summary>Clears all substitution requests.</summary>
    public virtual void Clear()
    {
        myShapeToReplacement.Clear();
        myNewShapes.Clear();
    }

    /// <summary>Records a request to remove a shape, whatever its orientation.</summary>
    public virtual void Remove(in TopoDS_Shape shape)
    {
        TopoDS_Shape nulshape = new();
        replace(shape, nulshape, TReplacementKind.TReplacementKind_Remove);
    }

    /// <summary>Records a request to replace a shape by a new one.</summary>
    public virtual void Replace(in TopoDS_Shape shape, in TopoDS_Shape newshape)
    {
        replace(shape, newshape, TReplacementKind.TReplacementKind_Modify);
    }

    /// <summary>
    /// Merges the parts into the single product: the first part is replaced by the product, the others are removed.
    /// The history presents the merged shapes alike.
    /// </summary>
    public void Merge(IEnumerable<TopoDS_Shape> theParts, in TopoDS_Shape theProduct)
    {
        using IEnumerator<TopoDS_Shape> aPIt = theParts.GetEnumerator();

        bool aMore = aPIt.MoveNext();
        if (aMore)
        {
            replace(aPIt.Current, theProduct, TReplacementKind.TReplacementKind_Merge_Main);

            aMore = aPIt.MoveNext();
        }

        const TReplacementKind aKind = TReplacementKind.TReplacementKind_Merge_Ordinary;
        for (; aMore; aMore = aPIt.MoveNext())
        {
            replace(aPIt.Current, theProduct, aKind);
        }
    }

    /// <summary>True if a replacement or a removal is recorded for the shape.</summary>
    public virtual bool IsRecorded(in TopoDS_Shape ashape)
    {
        TopoDS_Shape shape = ashape;
        if (myConsiderLocation)
        {
            TopLoc_Location nullLoc = new();
            shape.Location(nullLoc);
        }
        if (shape.IsNull())
        {
            return false;
        }
        return myShapeToReplacement.IsBound(shape);
    }

    /// <summary>
    /// The new value of a shape: the shape itself if not recorded, a null shape if it is to be removed, else its
    /// replacement. Frozen.
    /// </summary>
    public TopoDS_Shape Value(in TopoDS_Shape shape) => Published(InternalValue(shape));

    /// <summary>The new value of a shape, as <see cref="Value"/>, without freezing it (ported code).</summary>
    internal virtual TopoDS_Shape InternalValue(in TopoDS_Shape ashape)
    {
        TopoDS_Shape res = new();
        if (ashape.IsNull())
        {
            return res;
        }
        TopoDS_Shape shape = ashape;
        if (myConsiderLocation)
        {
            TopLoc_Location nullLoc = new();
            shape.Location(nullLoc);
        }

        bool fromMap = false;
        if (!myShapeToReplacement.IsBound(shape))
        {
            res = shape;
        }
        else
        {
            res = myShapeToReplacement[shape].Result();
            if (shape.Orientation() == TopAbs_REVERSED)
            {
                res.Reverse();
            }
            fromMap = true;
        }
        // for INTERNAL/EXTERNAL, since they are not fully supported, keep orientation
        if (shape.Orientation() == TopAbs_INTERNAL || shape.Orientation() == TopAbs_EXTERNAL)
        {
            res.Orientation(shape.Orientation());
        }

        if (myConsiderLocation)
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
    public TopoDS_Shape ValueLeaf(in TopoDS_Shape theShape) => Published(InternalValueLeaf(theShape));

    /// <summary>The end of the replacement chain, as <see cref="ValueLeaf"/>, without freezing it (ported code).</summary>
    internal TopoDS_Shape InternalValueLeaf(in TopoDS_Shape theShape)
    {
        if (theShape.IsNull())
        {
            return new TopoDS_Shape();
        }

        // Track visited shapes by their underlying TShape. Rationale: the replacement map keys
        // entries via TopTools_ShapeMapHasher (orientation-ignoring IsSame) and, when
        // myConsiderLocation is set, strips locations on both insertion and lookup. That leaves
        // TShape as the only identity axis that's stable across the walk under every mode -
        // so keying the cycle guard on the TShape handle catches cycles that would otherwise
        // slip past orientation/location-sensitive comparisons.
        TopoDS_Shape aCurrent = theShape;
        var aVisited = new NCollection_Map<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        aVisited.Add(aCurrent.TShape()!);

        for (;;)
        {
            TopoDS_Shape aNext = InternalValue(aCurrent);
            if (aNext.IsNull())
            {
                return aNext;
            }
            if (aNext.IsSame(aCurrent))
            {
                return aNext;
            }
            if (!aVisited.Add(aNext.TShape()!))
            {
                // Cycle in replacement data - return current best to avoid looping.
                return aNext;
            }
            aCurrent = aNext;
        }
    }

    /// <summary>
    /// The substitution status of a shape: 0 if not recorded (<paramref name="newsh"/> is the shape), &lt; 0 if it is
    /// to be removed (<paramref name="newsh"/> is null), &gt; 0 if it is to be replaced (<paramref name="newsh"/> is
    /// the replacement). With <paramref name="last"/>, a replaced shape's replacement is applied recursively.
    /// <paramref name="newsh"/> is frozen.
    /// </summary>
    public int Status(in TopoDS_Shape shape, out TopoDS_Shape newsh, bool last = false)
    {
        int res = InternalStatus(shape, out newsh, last);
        newsh = Published(newsh);
        return res;
    }

    /// <summary>The substitution status, as <see cref="Status"/>, without freezing (ported code).</summary>
    internal virtual int InternalStatus(in TopoDS_Shape ashape, out TopoDS_Shape newsh, bool last = false)
    {
        int res = 0;
        newsh = new TopoDS_Shape();
        if (ashape.IsNull())
        {
            newsh.Nullify();
            return res;
        }

        TopoDS_Shape shape = ashape;
        TopLoc_Location aLocSh = shape.Location();
        if (myConsiderLocation)
        {
            TopLoc_Location nullLoc = new();
            shape.Location(nullLoc);
        }

        if (!myShapeToReplacement.IsBound(shape))
        {
            newsh = shape;
            res = 0;
        }
        else
        {
            newsh = myShapeToReplacement[shape].Result();
            res = 1;
        }
        if (res > 0)
        {
            if (newsh.IsNull())
            {
                res = -1;
            }
            else if (newsh.IsEqual(shape))
            {
                res = 0;
            }
            else if (last
                     && ((myConsiderLocation && !newsh.IsPartner(shape))
                         || (!myConsiderLocation && !newsh.IsSame(shape))))
            {
                //  sln 29.11.01 Bug24: Correction iteration through maps. Way of iteration used early does
                //  not correspond to way of storing information in the maps.
                newsh = InternalApply(shape, TopAbs_SHAPE);
                if (newsh.IsNull())
                {
                    res = -1;
                }
                if (newsh.IsEqual(shape))
                {
                    res = 0;
                }
            }
        }
        if (myConsiderLocation && !newsh.IsNull())
        {
            TopLoc_Location aResLoc =
                (res > 0 && !newsh.Location().IsIdentity() ? aLocSh * newsh.Location() : aLocSh);
            newsh.Location(aResLoc, false);
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
    public TopoDS_Shape Apply(in TopoDS_Shape shape, TopAbs_ShapeEnum until = TopAbs_SHAPE) =>
        Published(InternalApply(shape, until));

    /// <summary>Applies the substitution requests, as <see cref="Apply"/>, without freezing the result (ported code).</summary>
    internal virtual TopoDS_Shape InternalApply(in TopoDS_Shape shape, TopAbs_ShapeEnum until = TopAbs_SHAPE)
    {
        var anInFlight = new NCollection_Map<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        return applyImpl(shape, until, anInFlight);
    }

    /// <summary>
    /// The recursive worker of Apply, with a set of the TShapes being processed further up (by identity), so that a
    /// replacement containing the original shape doesn't recurse without end.
    /// </summary>
    protected TopoDS_Shape applyImpl(in TopoDS_Shape shape, TopAbs_ShapeEnum until, NCollection_Map<TopoDS_TShape> theInFlight)
    {
        myStatus = EncodeStatus(0); // ShapeExtend::EncodeStatus ( ShapeExtend_OK );
        if (shape.IsNull())
        {
            return shape;
        }

        // apply direct replacement
        TopoDS_Shape newsh = InternalValue(shape);

        // if shape removed, return NULL
        if (newsh.IsNull())
        {
            myStatus = EncodeStatus(2); // ShapeExtend_DONE2
            return newsh;
        }

        // DFS cycle guard: if shape is already being processed further up the call
        // stack, its replacement must be a compound that transitively contains it.
        // Return the direct replacement without descending to break the cycle.
        if (theInFlight.Contains(shape.TShape()!))
        {
            return newsh;
        }

        // if shape replaced, apply modifications to the result recursively
        if ((myConsiderLocation && !newsh.IsPartner(shape))
            || (!myConsiderLocation && !newsh.IsSame(shape)))
        {
            theInFlight.Add(shape.TShape()!);
            TopoDS_Shape res = applyImpl(newsh, until, theInFlight);
            theInFlight.Remove(shape.TShape()!);
            myStatus |= EncodeStatus(1); // ShapeExtend::EncodeStatus ( ShapeExtend_DONE1 );
            return res;
        }

        TopAbs_ShapeEnum st = shape.ShapeType();
        if (st > until || (st == until && until > TopAbs_COMPOUND))
        {
            return newsh; // stopping criteria
        }
        if (st == TopAbs_VERTEX || st == TopAbs_SHAPE)
        {
            return shape;
        }
        // define allowed types of components
        // fix for SAMTECH bug OCC322 about absent internal vertices after sewing.
        BRep_Builder B = new();

        TopoDS_Shape result = shape.EmptyCopied();
        TopAbs_Orientation orien = shape.Orientation();
        result.Orientation(TopAbs_FORWARD); // protect against INTERNAL or EXTERNAL shapes
        bool modif = false;
        int locStatus = myStatus;

        // apply recorded modifications to subshapes
        bool isEmpty = true;
        theInFlight.Add(shape.TShape()!);
        for (var it = new TopoDS_Iterator(shape, false); it.More(); it.Next())
        {
            TopoDS_Shape sh = it.Value();
            newsh = applyImpl(sh, until, theInFlight);
            if (newsh != sh)
            {
                if ((myStatus & EncodeStatus(4)) != 0)
                { // ShapeExtend::DecodeStatus ( myStatus, ShapeExtend_DONE4 ) )
                    locStatus |= EncodeStatus(4); //|= ShapeExtend::EncodeStatus ( ShapeExtend_DONE4 );
                }
                modif = true;
            }
            if (newsh.IsNull())
            {
                locStatus |= EncodeStatus(4); // ShapeExtend::EncodeStatus ( ShapeExtend_DONE4 );
                continue;
            }
            if (isEmpty)
            {
                isEmpty = false;
            }
            locStatus |= EncodeStatus(3); // ShapeExtend::EncodeStatus ( ShapeExtend_DONE3 );
            if (st == TopAbs_COMPOUND || newsh.ShapeType() == sh.ShapeType())
            { // fix for SAMTECH bug OCC322 about absent internal vertices after sewing.
                B.Add(result, newsh);
                continue;
            }
            int nitems = 0;
            for (var subit = new TopoDS_Iterator(newsh); subit.More(); subit.Next(), nitems++)
            {
                TopoDS_Shape subsh = subit.Value();
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
        theInFlight.Remove(shape.TShape()!);
        if (!modif)
        {
            return shape;
        }

        // For empty topological containers (any kind of shape except vertex, edge
        // and face) we have to produce an empty result
        if (isEmpty && st != TopAbs_VERTEX && st != TopAbs_EDGE && st != TopAbs_FACE)
        {
            result = new TopoDS_Shape();
        }
        else
        {
            // restore Range on edge broken by EmptyCopied()
            if (st == TopAbs_EDGE)
            {
                CopyRanges(result, shape, 0, 1);
            }
            else if (st == TopAbs_FACE)
            {
                TopoDS_Face face = TopoDS.Face(shape);
                if (BRep_Tool.NaturalRestriction(face))
                {
                    BRep_Builder aB = new();
                    aB.NaturalRestriction(TopoDS.Face(result), true);
                }
            }
            else if (st == TopAbs_WIRE || st == TopAbs_SHELL)
            {
                result.Closed(BRep_Tool.IsClosed(result));
            }

            result.Orientation(orien);
        }

        replace(shape, result, result.IsNull() ? TReplacementKind.TReplacementKind_Remove : TReplacementKind.TReplacementKind_Modify);
        myStatus = locStatus;

        return result;
    }

    /// <summary>True if the location of a shape is taken into account when recording and looking it up.</summary>
    public virtual bool ModeConsiderLocation() => myConsiderLocation;

    /// <summary>Sets whether the location of a shape is taken into account (OCCT's modifiable <c>bool&amp;</c>).</summary>
    public virtual void ModeConsiderLocation(bool theMode) => myConsiderLocation = theMode;

    /// <summary>
    /// A copy of the vertex with a new tolerance if it isn't recorded (then recorded as its replacement), or its
    /// recorded replacement changed in place. Internal: it returns an unfrozen vertex, and changes a recorded one in
    /// place (which throws <see cref="TopoDS_LockedShape"/> on a frozen one, as OCCT does on a locked one).
    /// </summary>
    internal TopoDS_Vertex CopyVertex(in TopoDS_Vertex theV, double theTol = -1.0)
    {
        return CopyVertex(theV, BRep_Tool.Pnt(theV), theTol);
    }

    /// <summary>
    /// A copy of the vertex at a new position, as <see cref="CopyVertex(in TopoDS_Vertex, double)"/>.
    /// </summary>
    internal TopoDS_Vertex CopyVertex(in TopoDS_Vertex theV, in gp_Pnt theNewPos, double theTol)
    {
        TopoDS_Vertex aVertexCopy;
        bool isRecorded = IsRecorded(theV);
        aVertexCopy = isRecorded ? TopoDS.Vertex(InternalApply(theV)) : TopoDS.Vertex(theV.EmptyCopied());

        BRep_Builder B = new();
        double aNewTol = theTol > 0.0 ? theTol : BRep_Tool.Tolerance(theV);
        B.UpdateVertex(aVertexCopy, theNewPos, aNewTol);

        if (!isRecorded)
        {
            Replace(theV, aVertexCopy);
        }

        return aVertexCopy;
    }

    /// <summary>True if the shape has been recorded as a replacement.</summary>
    public bool IsNewShape(in TopoDS_Shape theShape) => myNewShapes.Contains(theShape);

    /// <summary>
    /// The history of the substitutions: each recorded shape (not itself a replacement) is modified into the shapes at
    /// the ends of its replacement chain, or removed. The shapes it lists are frozen.
    /// </summary>
    public BRepTools_History History() => history(true);

    /// <summary>The history of the substitutions, as <see cref="History"/>, without freezing (ported code).</summary>
    internal BRepTools_History InternalHistory() => history(false);

    private BRepTools_History history(bool theToFreeze)
    {
        BRepTools_History aHistory = new();

        // Fill the history.
        foreach (KeyValuePair<TopoDS_Shape, TReplacement> aRIt in myShapeToReplacement)
        {
            TopoDS_Shape aShape = aRIt.Key;
            if (!BRepTools_History.IsSupportedType(aShape) || myNewShapes.Contains(aShape))
            {
                continue;
            }

            // OCCT.NET: OCCT's NCollection_IndexedMap<TopoDS_Shape> and NCollection_Map<TopoDS_Shape> without a hasher
            // use std::hash and operator==, which are IsEqual.
            var aIntermediates = new NCollection_IndexedMap<TopoDS_Shape>(EqualityComparer<TopoDS_Shape>.Default);
            var aModified = new NCollection_Map<TopoDS_Shape>(EqualityComparer<TopoDS_Shape>.Default);
            aIntermediates.Add(aShape);
            for (int aI = 1; aI <= aIntermediates.Extent(); ++aI)
            {
                TopoDS_Shape aIntermediate = aIntermediates[aI];
                if (!myShapeToReplacement.Find(aIntermediate, out TReplacement aReplacement))
                {
                    Add(aModified, aIntermediate);
                }
                else if (aReplacement.RelationKind() != BRepTools_History.TRelationType.TRelationType_Removed)
                {
                    TopoDS_Shape aResult = aReplacement.RelationResult();
                    if (!aResult.IsNull())
                    {
                        Add(aIntermediates, aResult);
                    }
                }
            }

            if (aModified.IsEmpty())
            {
                aHistory.Remove(aShape);
            }
            else
            {
                foreach (TopoDS_Shape aIt in aModified)
                {
                    aHistory.AddModified(aShape, aIt);
                    if (theToFreeze)
                    {
                        Published(aIt);
                    }
                }
            }
        }

        return aHistory;
    }

    /// <summary>
    /// Replaces the first shape by the second after reorienting them: if the first is REVERSED both are reversed; if it
    /// is INTERNAL or EXTERNAL, the second is oriented FORWARD (REVERSED) if its orientation is (isn't) the first's,
    /// and the first is oriented FORWARD.
    /// </summary>
    protected virtual void replace(in TopoDS_Shape ashape, in TopoDS_Shape anewshape, TReplacementKind theKind)
    {
        TopoDS_Shape shape = ashape;
        TopoDS_Shape newshape = anewshape;
        if (shape.IsNull() || shape == newshape)
        {
            return;
        }

        if (shape.Orientation() == TopAbs_REVERSED)
        {
            shape.Reverse();
            newshape.Reverse();
        }
        // protect against INTERNAL or EXTERNAL shape
        else if (shape.Orientation() == TopAbs_INTERNAL || shape.Orientation() == TopAbs_EXTERNAL)
        {
            newshape.Orientation((newshape.Orientation() == shape.Orientation()) ? TopAbs_FORWARD
                                                                                 : TopAbs_REVERSED);
            shape.Orientation(TopAbs_FORWARD);
        }

        if (myConsiderLocation)
        {
            // sln 29.11.01 Bug22: Change location of 'newshape' in accordance with location of 'shape'
            newshape.Location(newshape.Location().Multiplied(shape.Location().Inverted()), false);
            TopLoc_Location nullLoc = new();
            shape.Location(nullLoc);
        }

        // Cycle handling: cycles in the replacement map (A -> B -> A or longer) are
        // accepted at insertion time. The DFS in-flight guard inside Apply()/applyImpl()
        // breaks any cycle at traversal time, returning the immediate Value() instead
        // of recursing.
        myShapeToReplacement.Bind(shape, new TReplacement(newshape, theKind));
        myNewShapes.Add(newshape);
    }

    /// <summary>
    /// OCCT.NET (decision 583, <c>BRepTools_CopyAtEntry</c>): replaces, in every recorded request, the TShape of the
    /// recorded shape by its image in <paramref name="theKeys"/> and the TShape of the replacement (and of the
    /// recorded replacements, <see cref="IsNewShape"/>) by its image in <paramref name="theResults"/>, keeping their
    /// locations and orientations; TShapes without an image stay. The requests keep their order and kinds; if two
    /// recorded shapes get the same image, the later request wins. So a context filled by an algorithm working on a
    /// copy answers for the shapes the copy was made from, and the other way round.
    /// </summary>
    internal void ReplaceTShapes(IReadOnlyDictionary<TopoDS_TShape, TopoDS_TShape> theKeys,
                                 IReadOnlyDictionary<TopoDS_TShape, TopoDS_TShape> theResults)
    {
        var anEntries = new List<KeyValuePair<TopoDS_Shape, TReplacement>>(myShapeToReplacement);
        myShapeToReplacement.Clear();
        foreach (KeyValuePair<TopoDS_Shape, TReplacement> anEntry in anEntries)
        {
            TopoDS_Shape aKey = withImage(anEntry.Key, theKeys);
            TopoDS_Shape aResult = withImage(anEntry.Value.RelationResult(), theResults);
            myShapeToReplacement.Bind(aKey, new TReplacement(aResult, anEntry.Value.Kind()));
        }

        var aNewShapes = new List<TopoDS_Shape>(myNewShapes);
        myNewShapes.Clear();
        foreach (TopoDS_Shape aNew in aNewShapes)
        {
            myNewShapes.Add(withImage(aNew, theResults));
        }
    }

    /// <summary>
    /// OCCT.NET: the replacements recorded (the results of the requests, null for removals), unfrozen, for
    /// <c>BRepTools_CopyAtEntry</c>.
    /// </summary>
    internal IEnumerable<TopoDS_Shape> InternalReplacements()
    {
        foreach (KeyValuePair<TopoDS_Shape, TReplacement> anEntry in myShapeToReplacement)
        {
            yield return anEntry.Value.RelationResult();
        }
    }

    // The shape with the image of its TShape, if it has one.
    private static TopoDS_Shape withImage(in TopoDS_Shape theShape, IReadOnlyDictionary<TopoDS_TShape, TopoDS_TShape> theImages)
    {
        TopoDS_TShape? aTShape = theShape.TShape();
        if (aTShape is null || !theImages.TryGetValue(aTShape, out TopoDS_TShape? anImage))
        {
            return theShape;
        }
        TopoDS_Shape aShape = theShape;
        aShape.TShape(anImage);
        return aShape;
    }

    // OCCT's private isOrdinaryMerged is never called and isn't ported.

    // Freezes the graph of a shape handed to a caller (docs/phase3-design.md, section 4.2).
    private static TopoDS_Shape Published(in TopoDS_Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }

    // Adds the shape to the map.
    // If the shape is a wire, shell or solid then
    // adds the sub-shapes of the shape instead.
    private static void Add(NCollection_Map<TopoDS_Shape> theMap, in TopoDS_Shape theShape)
    {
        TopAbs_ShapeEnum aType = theShape.ShapeType();
        if (aType != TopAbs_WIRE && aType != TopAbs_SHELL && aType != TopAbs_COMPSOLID)
        {
            theMap.Add(theShape);
            return;
        }

        for (var aIt = new TopoDS_Iterator(theShape); aIt.More(); aIt.Next())
        {
            theMap.Add(aIt.Value());
        }
    }

    // The same, for an indexed map.
    private static void Add(NCollection_IndexedMap<TopoDS_Shape> theMap, in TopoDS_Shape theShape)
    {
        TopAbs_ShapeEnum aType = theShape.ShapeType();
        if (aType != TopAbs_WIRE && aType != TopAbs_SHELL && aType != TopAbs_COMPSOLID)
        {
            theMap.Add(theShape);
            return;
        }

        for (var aIt = new TopoDS_Iterator(theShape); aIt.More(); aIt.Next())
        {
            theMap.Add(aIt.Value());
        }
    }

    // Sets the ranges of the 3D curve and the pcurves of toedge (a new edge, EmptyCopied from fromedge) to the
    // ranges of fromedge's, scaled to [alpha, beta].
    private static void CopyRanges(in TopoDS_Shape toedge, in TopoDS_Shape fromedge, double alpha, double beta)
    {
        BRep_TEdge aTEdgeFrom = (BRep_TEdge)fromedge.TShape()!;
        BRep_TEdge aTEdgeTo = (BRep_TEdge)toedge.TShape()!;
        NCollection_List<BRep_CurveRepresentation> tolist = aTEdgeTo.ChangeCurves();
        // OCCT.NET: frozen: OCCT iterates fromedge's ChangeCurves() only to read them; the port reads Curves(), so a
        // frozen fromedge is accepted.
        var fromitcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(aTEdgeFrom.Curves());
        for (; fromitcr.More(); fromitcr.Next())
        {
            if (fromitcr.Value() is not BRep_GCurve fromGC)
            {
                continue;
            }
            bool isC3d = fromGC.IsCurve3D();
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

            Geom_Surface? surface = null;
            TopLoc_Location L = new();
            if (!isC3d)
            {
                surface = fromGC.Surface();
                L = fromGC.Location();
            }

            for (var toitcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(tolist); toitcr.More(); toitcr.Next())
            {
                if (toitcr.Value() is not BRep_GCurve toGC)
                {
                    continue;
                }
                if (isC3d)
                {
                    if (!toGC.IsCurve3D())
                    {
                        continue;
                    }
                }
                else if (!toGC.IsCurveOnSurface() || !ReferenceEquals(surface, toGC.Surface()) || L != toGC.Location())
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
