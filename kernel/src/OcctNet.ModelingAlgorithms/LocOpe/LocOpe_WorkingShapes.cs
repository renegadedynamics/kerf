// OCCT.NET: the copy at the entry of LocOpe's local operations (unit 7H), which write into the shapes they are given
// (point representations added to vertices, pcurves to edges, tolerances raised, continuities set). Not a port of an
// OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
/// (<see cref="BRepTools.ExactCopyForWrite(in TopoDS_Shape, Dictionary{TopoDS_TShape, TopoDS_TShape})"/>), all copies
/// sharing one map, so a vertex of a splitting edge and the same vertex given alone, or a face given with the shape it
/// belongs to, have one copy; a shape without a frozen TShape (ported code's own) is worked on in place, as in OCCT.
/// </para>
/// <para>
/// <see cref="Finish"/> ends the operation: every copy it left equal to its original is replaced by that original in the
/// shapes it built (<see cref="BRepTools.KeepUnchangedIn"/>), so the result shares the caller's unchanged sub-shapes;
/// <see cref="Final(in TopoDS_Shape)"/> then maps a shape the operation worked with to what stands for it. The caller's shapes given to a
/// query are mapped to the working ones with <see cref="Copied"/>, which never copies. The operation's public accessors
/// freeze what they hand out.
/// </para>
/// </remarks>
internal sealed class LocOpe_WorkingShapes
{
    private readonly Dictionary<TopoDS_TShape, TopoDS_TShape> myCopies = new(ReferenceEqualityComparer.Instance);    // original -> copy
    private readonly Dictionary<TopoDS_TShape, TopoDS_TShape> myOriginals = new(ReferenceEqualityComparer.Instance); // copy -> original
    // what ExactCopyForWrite reuses: the copies, and each copy as its own
    private readonly Dictionary<TopoDS_TShape, TopoDS_TShape> myKnown = new(ReferenceEqualityComparer.Instance);
    private readonly List<TopoDS_Shape> myOriginalRoots = [];
    private readonly List<TopoDS_Shape> myCopyRoots = [];
    private Dictionary<TopoDS_TShape, TopoDS_TShape>? myFinal; // copy -> what stands for it after Finish
    private bool myIsFinished;

    /// <summary>True when some shape was copied.</summary>
    public bool IsCopied() => myCopies.Count != 0;

    /// <summary>True after <see cref="Finish"/>.</summary>
    public bool IsFinished() => myIsFinished;

    /// <summary>
    /// The shape to work on in place of <paramref name="theShape"/>: its copy if it was copied (it or a shape holding
    /// it), a new copy if its graph holds a frozen TShape, else the shape itself. After <see cref="Finish"/>, only
    /// <see cref="Copied"/>.
    /// </summary>
    public TopoDS_Shape Working(in TopoDS_Shape theShape)
    {
        TopoDS_TShape? aTShape = theShape.TShape();
        if (aTShape is null)
        {
            return theShape;
        }
        if (myKnown.TryGetValue(aTShape, out TopoDS_TShape? anImage))
        {
            return withImage(theShape, anImage);
        }
        if (myIsFinished || !BRepTools.HoldsFrozen(theShape))
        {
            return theShape;
        }

        int aNbKnown = myKnown.Count;
        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(theShape, myKnown);
        if (myKnown.Count != aNbKnown)
        {
            // the new pairs: the TShapes of the shape's graph that weren't known
            var aNew = new List<KeyValuePair<TopoDS_TShape, TopoDS_TShape>>();
            foreach (KeyValuePair<TopoDS_TShape, TopoDS_TShape> aPair in myKnown)
            {
                if (!ReferenceEquals(aPair.Key, aPair.Value) && !myCopies.ContainsKey(aPair.Key))
                {
                    aNew.Add(aPair);
                }
            }
            foreach (KeyValuePair<TopoDS_TShape, TopoDS_TShape> aPair in aNew)
            {
                myCopies[aPair.Key] = aPair.Value;
                myOriginals[aPair.Value] = aPair.Key;
                myKnown[aPair.Value] = aPair.Value;
            }
        }
        myOriginalRoots.Add(theShape);
        myCopyRoots.Add(aCopy);
        return aCopy;
    }

    /// <summary>
    /// The working shape standing for <paramref name="theShape"/> (a caller's shape given to a query): its copy if it
    /// was copied, else itself. Never copies.
    /// </summary>
    public TopoDS_Shape Copied(in TopoDS_Shape theShape)
    {
        TopoDS_TShape? aTShape = theShape.TShape();
        return aTShape is not null && myCopies.TryGetValue(aTShape, out TopoDS_TShape? anImage) ? withImage(theShape, anImage) : theShape;
    }

    /// <summary>The caller's shape a working copy was made from (the shape itself if it isn't a copy).</summary>
    public TopoDS_Shape Original(in TopoDS_Shape theShape)
    {
        TopoDS_TShape? aTShape = theShape.TShape();
        return aTShape is not null && myOriginals.TryGetValue(aTShape, out TopoDS_TShape? anImage) ? withImage(theShape, anImage) : theShape;
    }

    /// <summary>
    /// Ends the operation: in the shapes <paramref name="theShapes"/> it built or worked with (their unfrozen graphs, and
    /// each of them if it is a copy), every copy left equal to its original is replaced by the original. Once only;
    /// nothing is done when nothing was copied.
    /// </summary>
    public void Finish(IEnumerable<TopoDS_Shape> theShapes)
    {
        if (myIsFinished)
        {
            return;
        }
        myIsFinished = true;
        if (myCopies.Count == 0)
        {
            return;
        }

        // one unfrozen compound holds every shape, another the roots copied and a third their copies, so that
        // KeepUnchangedIn compares each copy with its original and replaces the kept ones in all the graphs at once.
        // The children are appended directly: BRep_Builder.Add would clear their Free flags.
        TopoDS_Shape aHeld = holder(theShapes);
        TopoDS_Shape anOriginals = holder(myOriginalRoots);
        TopoDS_Shape aCopies = holder(myCopyRoots);
        myFinal = BRepTools.KeepUnchangedIn(anOriginals, aCopies, myCopies, ref aHeld, out _);
    }

    /// <summary>
    /// After <see cref="Finish"/>, what stands for a shape the operation worked with: for a copy, its original if the
    /// operation left it unchanged, else the copy; any other shape is itself. Before, the shape itself.
    /// </summary>
    public TopoDS_Shape Final(in TopoDS_Shape theShape)
    {
        TopoDS_TShape? aTShape = theShape.TShape();
        return aTShape is not null && myFinal is not null && myFinal.TryGetValue(aTShape, out TopoDS_TShape? anImage)
                   ? withImage(theShape, anImage)
                   : theShape;
    }

    /// <summary>A new list of the <see cref="Final(in TopoDS_Shape)"/> shapes of <paramref name="theList"/>.</summary>
    public NCollection_List<TopoDS_Shape> Final(NCollection_List<TopoDS_Shape> theList)
    {
        var aList = new NCollection_List<TopoDS_Shape>();
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(theList); anIt.More(); anIt.Next())
        {
            aList.Append(Final(anIt.Value()));
        }
        return aList;
    }

    /// <summary>Freezes the shapes of a list a public accessor hands out, and returns it.</summary>
    public static NCollection_List<TopoDS_Shape> Published(NCollection_List<TopoDS_Shape> theList)
    {
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(theList); anIt.More(); anIt.Next())
        {
            anIt.Value().TShape()?.FreezeGraph();
        }
        return theList;
    }

    /// <summary>Freezes a shape a public accessor hands out, and returns it.</summary>
    public static TopoDS_Shape Published(TopoDS_Shape theShape)
    {
        theShape.TShape()?.FreezeGraph();
        return theShape;
    }

    private static TopoDS_Shape holder(IEnumerable<TopoDS_Shape> theShapes)
    {
        var aHolder = new TopoDS_Compound();
        new BRep_Builder().MakeCompound(ref aHolder);
        TopoDS_TShape aTShape = aHolder.TShape()!;
        foreach (TopoDS_Shape aShape in theShapes)
        {
            if (!aShape.IsNull())
            {
                aTShape.myShapes.Append(aShape);
            }
        }
        return aHolder;
    }

    private static TopoDS_Shape withImage(in TopoDS_Shape theShape, TopoDS_TShape theImage)
    {
        TopoDS_Shape aShape = theShape;
        aShape.TShape(theImage);
        return aShape;
    }
}
