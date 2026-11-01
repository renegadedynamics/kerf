// OCCT.NET: the copy at the entry of an algorithm that edits the shape it is given in place (healing, sewing,
// clean-up: docs/phase5-6-plan.md, section 2.7; decision 583), and the composition of its histories with the copy's.
// Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The copy for write of a shape at the entry of an in-place algorithm, and the way back to the caller's shapes at its
/// end.
/// </summary>
/// <remarks>
/// <para>
/// Shape Healing's algorithms (ShapeFix, ShapeUpgrade), sewing and the clean-up edit the shape they are given: they
/// raise tolerances, add pcurves, reorder wires in place, and record the shapes they rebuild in a
/// <see cref="BRepTools_ReShape"/> context. A shape a caller holds is frozen, so such an algorithm runs on a copy:
/// </para>
/// <list type="number">
/// <item><description><c>new BRepTools_CopyAtEntry(input, context)</c> makes
/// <see cref="BRepTools.ExactCopyForWrite(in TopoDS_Shape, out BRepTools_History)"/> of the input when its graph holds a
/// frozen TShape (else the algorithm runs in place, as OCCT's, on the unfrozen shape ported code owns). Requests
/// already recorded in the context for the input's sub-shapes are moved onto their copies.</description></item>
/// <item><description>The algorithm runs on <see cref="Shape"/>. Other arguments that refer to the input's sub-shapes
/// (a face of the input given with one of its wires, the shapes added one by one to sewing) are mapped with
/// <see cref="Copied"/>.</description></item>
/// <item><description><see cref="Finish"/> takes its result: every copied sub-shape the algorithm left equal to its
/// original is replaced by that original (<see cref="BRepTools.KeepUnchangedIn"/>), in the result and in the shapes the
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
internal sealed class BRepTools_CopyAtEntry
{
    private readonly TopoDS_Shape myInput;
    private readonly TopoDS_Shape myCopy;
    private BRepTools_ReShape? myContext;
    private readonly Dictionary<TopoDS_TShape, TopoDS_TShape>? myCopies;    // original -> copy
    private readonly Dictionary<TopoDS_TShape, TopoDS_TShape>? myOriginals; // copy -> original
    private Dictionary<TopoDS_TShape, TopoDS_TShape>? myFinal;              // copy -> what stands for it after Finish
    private BRepTools_History myHistory = new();
    private bool myIsFinished;

    /// <summary>
    /// Copies <paramref name="theInput"/> for write if its graph holds a frozen TShape (see
    /// <see cref="BRepTools.HoldsFrozen"/>), or always with <paramref name="theToForce"/>; the requests recorded in
    /// <paramref name="theContext"/> for the input's sub-shapes are moved onto their copies.
    /// </summary>
    public BRepTools_CopyAtEntry(in TopoDS_Shape theInput, BRepTools_ReShape? theContext = null, bool theToForce = false)
    {
        myInput = theInput;
        myContext = theContext;
        if (theInput.IsNull() || !(theToForce || BRepTools.HoldsFrozen(theInput)))
        {
            myCopy = theInput;
            return;
        }

        myCopy = BRepTools.ExactCopyForWrite(theInput, out _, out Dictionary<TopoDS_TShape, TopoDS_TShape> aCopies);
        myCopies = aCopies;
        myOriginals = new Dictionary<TopoDS_TShape, TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        foreach (KeyValuePair<TopoDS_TShape, TopoDS_TShape> aPair in aCopies)
        {
            myOriginals[aPair.Value] = aPair.Key;
        }

        theContext?.ReplaceTShapes(aCopies, s_noImages);
    }

    private static readonly Dictionary<TopoDS_TShape, TopoDS_TShape> s_noImages = new(ReferenceEqualityComparer.Instance);

    /// <summary>
    /// Gives the helper the context an algorithm created after the entry (ShapeFix_Face's FixMissingSeam creates one
    /// when it has none), so that <see cref="Finish"/> re-keys it too. Ignored when a context was given at the entry:
    /// a context created later holds requests on the copy's sub-shapes only, so nothing had to be moved at the entry.
    /// </summary>
    public void AttachContext(BRepTools_ReShape? theContext)
    {
        if (myContext is null && !myIsFinished)
        {
            myContext = theContext;
        }
    }

    /// <summary>The input.</summary>
    public TopoDS_Shape Input() => myInput;

    /// <summary>The shape to run the algorithm on: the copy, or the input itself when it wasn't copied.</summary>
    public TopoDS_Shape Shape() => myCopy;

    /// <summary>True once <see cref="Finish"/> has run.</summary>
    public bool IsFinished() => myIsFinished;

    /// <summary>True if the input was copied.</summary>
    public bool IsCopied() => myCopies is not null;

    /// <summary>
    /// The copy of a sub-shape of the input, at the same location and orientation (the shape itself when the input
    /// wasn't copied, or when it isn't a sub-shape of the input).
    /// </summary>
    public TopoDS_Shape Copied(in TopoDS_Shape theShape) => withImage(theShape, myCopies);

    /// <summary>
    /// The input's sub-shape a copy was made from, at the copy's location and orientation (the shape itself when the
    /// input wasn't copied, or when it isn't a copy). The inverse of <see cref="Copied"/>.
    /// </summary>
    public TopoDS_Shape Original(in TopoDS_Shape theShape) => withImage(theShape, myOriginals);

    /// <summary>
    /// After <see cref="Finish"/>: what stands for a shape the algorithm worked with: for a copied sub-shape, its
    /// original if the algorithm left it unchanged, else the changed copy; any other shape is itself (its sub-shapes
    /// were already replaced in place by <see cref="Finish"/> where it reached them).
    /// </summary>
    public TopoDS_Shape Final(in TopoDS_Shape theShape)
    {
        checkFinished();
        return withImage(theShape, myFinal);
    }

    /// <summary>
    /// After <see cref="Finish"/>: the input's vertices, edges, faces and solids whose copies the algorithm changed in
    /// place, each modified into its changed copy (empty when the input wasn't copied).
    /// </summary>
    public BRepTools_History History()
    {
        checkFinished();
        return myHistory;
    }

    /// <summary>
    /// Ends the algorithm: the copied sub-shapes it left equal to their originals are replaced by them in
    /// <paramref name="theResult"/> (which may be one of them) and in the context's replacements, the context's
    /// requests are moved back onto the input's sub-shapes, and the sub-shapes changed in place but not recorded are
    /// recorded as replaced by their copies. Returns the result, unfrozen. When the input wasn't copied, returns
    /// <paramref name="theResult"/> and changes nothing.
    /// </summary>
    /// <param name="theResult">The algorithm's result (null if it has none).</param>
    /// <param name="theOthers">Other shapes the algorithm hands out (free edges, multiple edges, ...): their sub-shapes
    /// are replaced in place too, and <see cref="Final"/> maps them.</param>
    public TopoDS_Shape Finish(in TopoDS_Shape theResult, IEnumerable<TopoDS_Shape>? theOthers = null)
    {
        if (myIsFinished)
        {
            throw new Standard_ProgramError("BRepTools_CopyAtEntry::Finish called twice");
        }
        myIsFinished = true;
        if (myCopies is null)
        {
            myFinal = null;
            return theResult;
        }

        // Every shape that may hold copies goes into one unfrozen compound, so that KeepUnchangedIn replaces the kept
        // copies in all their graphs at once (it doesn't enter frozen TShapes, which can't hold copies). The children
        // are appended directly: BRep_Builder.Add would clear their Free flags.
        var aHolder = new TopoDS_Compound();
        new BRep_Builder().MakeCompound(ref aHolder);
        TopoDS_TShape aHolderTShape = aHolder.TShape()!;
        if (!theResult.IsNull())
        {
            aHolderTShape.myShapes.Append(theResult);
        }
        if (theOthers is not null)
        {
            foreach (TopoDS_Shape anOther in theOthers)
            {
                if (!anOther.IsNull())
                {
                    aHolderTShape.myShapes.Append(anOther);
                }
            }
        }
        if (myContext is not null)
        {
            foreach (TopoDS_Shape aReplacement in myContext.InternalReplacements())
            {
                if (!aReplacement.IsNull())
                {
                    aHolderTShape.myShapes.Append(aReplacement);
                }
            }
        }

        TopoDS_Shape aHeld = aHolder;
        myFinal = BRepTools.KeepUnchangedIn(myInput, myCopy, myCopies, ref aHeld, out myHistory);
        TopoDS_Shape aResult = withImage(theResult, myFinal);

        if (myContext is not null)
        {
            // the requests back onto the input's sub-shapes, their replacements onto the final shapes
            myContext.ReplaceTShapes(myOriginals!, myFinal);

            // the changes made in place without a request
            var aSubShapes = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(myInput, aSubShapes);
            for (int i = 1; i <= aSubShapes.Extent(); i++)
            {
                TopoDS_Shape aSub = aSubShapes[i];
                TopoDS_Shape aNow = withImage(Copied(aSub), myFinal);
                if (!ReferenceEquals(aNow.TShape(), aSub.TShape()) && !myContext.IsRecorded(aSub))
                {
                    myContext.Replace(aSub, aNow);
                }
            }
        }

        return aResult;
    }

    /// <summary>
    /// After <see cref="Finish"/>: the history <paramref name="theHistory"/> of the algorithm over the copy's
    /// sub-shapes as a history over the input's. For each vertex, edge, face and solid of the input: removed if its
    /// copy was; modified into (and generated) the final shapes of its copy's; else, if its copy was changed in place,
    /// modified into it. When the input wasn't copied, <paramref name="theHistory"/> itself (a new empty history for
    /// null). Entries of <paramref name="theHistory"/> for shapes that aren't copies (shapes the algorithm made) aren't
    /// carried over.
    /// </summary>
    public BRepTools_History Composed(BRepTools_History? theHistory)
    {
        checkFinished();
        if (myCopies is null)
        {
            return theHistory ?? new BRepTools_History();
        }

        var aComposed = new BRepTools_History();
        var aSubShapes = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(myInput, aSubShapes);
        for (int i = 1; i <= aSubShapes.Extent(); i++)
        {
            TopoDS_Shape aSub = aSubShapes[i];
            if (!BRepTools_History.IsSupportedType(aSub))
            {
                continue;
            }

            TopoDS_Shape aCopy = Copied(aSub);
            bool isModified = false;
            if (theHistory is not null)
            {
                if (theHistory.IsRemoved(aCopy))
                {
                    aComposed.Remove(aSub);
                    continue;
                }
                foreach (TopoDS_Shape aModified in theHistory.Modified(aCopy))
                {
                    aComposed.AddModified(aSub, withImage(aModified, myFinal));
                    isModified = true;
                }
                foreach (TopoDS_Shape aGenerated in theHistory.Generated(aCopy))
                {
                    aComposed.AddGenerated(aSub, withImage(aGenerated, myFinal));
                }
            }
            if (!isModified)
            {
                TopoDS_Shape aNow = withImage(aCopy, myFinal);
                if (!ReferenceEquals(aNow.TShape(), aSub.TShape()))
                {
                    aComposed.AddModified(aSub, aNow);
                }
            }
        }
        return aComposed;
    }

    private void checkFinished()
    {
        if (!myIsFinished)
        {
            throw new Standard_ProgramError("BRepTools_CopyAtEntry: Finish not called");
        }
    }

    // The shape with the image of its TShape, if it has one.
    private static TopoDS_Shape withImage(in TopoDS_Shape theShape, Dictionary<TopoDS_TShape, TopoDS_TShape>? theImages)
    {
        TopoDS_TShape? aTShape = theShape.TShape();
        if (aTShape is null || theImages is null || !theImages.TryGetValue(aTShape, out TopoDS_TShape? anImage))
        {
            return theShape;
        }
        TopoDS_Shape aShape = theShape;
        aShape.TShape(anImage);
        return aShape;
    }
}
