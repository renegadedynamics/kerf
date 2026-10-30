// OCCT.NET: copying the frozen parts of a shape an algorithm works on in place (unit 6G). Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

public static partial class BRepTools
{
    /// <summary>
    /// Makes an unfrozen shape that an algorithm is about to write into in place (its own intermediate result) free of
    /// frozen TShapes: each frozen sub-graph is replaced, in its unfrozen parents, by an exact copy
    /// (<see cref="ExactCopyForWrite(in TopoDS_Shape, out BRepTools_History)"/>'s), and a frozen shape is replaced as a
    /// whole. The unfrozen TShapes stay the same objects, so the in-place writes still reach every shape that holds
    /// them, as in OCCT, while the frozen originals (a caller's or published shapes) are never written.
    /// </summary>
    /// <param name="theShape">The shape (it is returned as is when nothing in it is frozen).</param>
    /// <param name="theHistory">Records each vertex, edge, face and solid of the shape, as placed in it, replaced by its
    /// copy as modified into the copy.</param>
    /// <returns>The shape, or its copy when it is frozen itself.</returns>
    internal static TopoDS_Shape CopyFrozenForWrite(in TopoDS_Shape theShape, BRepTools_History theHistory)
    {
        if (theShape.IsNull() || !HoldsFrozen(theShape))
        {
            return theShape;
        }

        // the sub-shapes as placed in the shape, before the replacement
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, aMap);

        var aCopies = new Dictionary<TopoDS_TShape, TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        var aVisited = new HashSet<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
        TopoDS_Shape aResult = theShape;
        aResult.TShape(replaceFrozen(theShape.TShape()!, aCopies, aVisited));

        for (int i = 1; i <= aMap.Extent(); i++)
        {
            TopoDS_Shape aS = aMap[i];
            if (BRepTools_History.IsSupportedType(aS) && aCopies.TryGetValue(aS.TShape()!, out TopoDS_TShape? aCopy)
                && !ReferenceEquals(aCopy, aS.TShape()))
            {
                TopoDS_Shape aNew = aS;
                aNew.TShape(aCopy);
                theHistory.AddModified(aS, aNew);
            }
        }
        return aResult;
    }

    // A frozen TShape is copied; an unfrozen one gets its frozen children replaced in place (each TShape once)
    private static TopoDS_TShape replaceFrozen(TopoDS_TShape theTShape, Dictionary<TopoDS_TShape, TopoDS_TShape> theCopies,
                                               HashSet<TopoDS_TShape> theVisited)
    {
        if (theTShape.IsFrozen())
        {
            return exactCopy(theTShape, theCopies);
        }

        if (!theVisited.Add(theTShape))
        {
            return theTShape;
        }

        bool aChanged = false;
        List<TopoDS_Shape> aChildren = [];
        for (var anIt = new NCollection_List<TopoDS_Shape>.Iterator(theTShape.myShapes); anIt.More(); anIt.Next())
        {
            TopoDS_Shape aChild = anIt.Value();
            TopoDS_TShape aChildT = aChild.TShape()!;
            TopoDS_TShape aNewT = replaceFrozen(aChildT, theCopies, theVisited);
            if (!ReferenceEquals(aNewT, aChildT))
            {
                aNewT.Free(false); // as TopoDS_Builder::Add clears it
                aChild.TShape(aNewT);
                aChanged = true;
            }
            aChildren.Add(aChild);
        }

        if (aChanged)
        {
            theTShape.myShapes.Clear();
            foreach (TopoDS_Shape aChild in aChildren)
            {
                theTShape.myShapes.Append(aChild);
            }
        }
        return theTShape;
    }
}
