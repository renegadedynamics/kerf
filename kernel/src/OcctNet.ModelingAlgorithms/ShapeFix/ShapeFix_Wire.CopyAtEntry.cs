// OCCT.NET: the copy at the entry of ShapeFix_Wire (docs/phase5-6-plan.md, section 2.7; decision 583) and its public
// results. Not a port of an OCCT file.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

public partial class ShapeFix_Wire
{
    // The copy of the loaded edges, from the first fix to the public result; null when the loaded edges hold no frozen
    // shape (they are the caller's working shapes, fixed in place as in OCCT).
    private BRepTools_CopyAtEntry? myEntry;

    // True when the loaded edges must be checked (and copied if frozen) before the next fix: after Load, Init and a
    // public result.
    private bool myToEnter;

    /// <summary>
    /// Returns the resulting wire (made with a builder): the edges of <see cref="WireData"/>, frozen. Where the loaded
    /// wire was frozen, the edges and vertices the fixes left unchanged are the caller's own, and the context records
    /// the ones they changed in place.
    /// </summary>
    public TopoDS_Wire Wire()
    {
        endWork();
        return myAnalyzer.WireData()!.Wire();
    }

    /// <summary>
    /// Returns the resulting wire made with BRepBuilderAPI_MakeWire (which can merge vertices), frozen; see
    /// <see cref="Wire"/>.
    /// </summary>
    public TopoDS_Wire WireAPIMake()
    {
        endWork();
        return myAnalyzer.WireData()!.WireAPIMake();
    }

    /// <summary>
    /// The resulting wire, unfrozen, for ported code fixing a wire it owns (ShapeFix_Face, UnifySameDomain): its edges
    /// are the working ones.
    /// </summary>
    internal TopoDS_Wire InternalWire() => myAnalyzer.WireData()!.InternalWire();

    /// <summary>The resulting wire made with BRepBuilderAPI_MakeWire, unfrozen; see <see cref="InternalWire"/>.</summary>
    internal TopoDS_Wire InternalWireAPIMake() => myAnalyzer.WireData()!.InternalWireAPIMake();

    /// <summary>
    /// Called at the start of <see cref="Perform"/> and of every public fix: after a load or a public result, copies
    /// the loaded edges (and the non-manifold ones) when they hold a frozen shape, and works on the copies.
    /// </summary>
    private void enterWork()
    {
        if (!myToEnter)
        {
            return;
        }
        myToEnter = false;
        myEntry   = null;

        ShapeExtend_WireData? sbwd = myAnalyzer.WireData();
        if (sbwd is null)
        {
            return;
        }
        TopoDS_Compound anEdges = edgesCompound(sbwd);
        if (!BRepTools.HoldsFrozen(anEdges))
        {
            return;
        }

        myEntry = new BRepTools_CopyAtEntry(anEdges, Context());
        TopoDS_Shape aCopy = myEntry.Shape();
        setEdges(sbwd, aCopy);

        // The copy clears the Free flag of the edges it puts in the compound; give each copy its original's (a cache
        // bit, which KeepUnchanged ignores), so the fixes that test E.Free() (FixConnected, ...) take OCCT's branch.
        var anOriginal = new TopoDS_Iterator(anEdges, false, false);
        for (var aCopied = new TopoDS_Iterator(aCopy, false, false); aCopied.More(); aCopied.Next(), anOriginal.Next())
        {
            aCopied.Value().TShape()!.Free(anOriginal.Value().TShape()!.Free());
        }
    }

    /// <summary>
    /// Before a public result: gives back the caller's own edges and vertices wherever the fixes left the copies
    /// unchanged, in the wire data and in the context, and records in the context the copies changed in place. The
    /// next fix copies again (the published edges are frozen).
    /// </summary>
    private void endWork()
    {
        myToEnter = true;
        BRepTools_CopyAtEntry? anEntry = myEntry;
        myEntry = null;
        ShapeExtend_WireData? sbwd = myAnalyzer.WireData();
        if (anEntry is null || sbwd is null)
        {
            return;
        }

        TopoDS_Compound aWorked = edgesCompound(sbwd);
        TopoDS_Shape aFinal = anEntry.Finish(aWorked);
        setEdges(sbwd, aFinal);
    }

    // The edges, then the non-manifold edges, of the wire data, appended to a new compound as they are (no Free flag
    // changed).
    private static TopoDS_Compound edgesCompound(ShapeExtend_WireData sbwd)
    {
        var aCompound = new TopoDS_Compound();
        new BRep_Builder().MakeCompound(ref aCompound);
        TopoDS_TShape aTShape = aCompound.TShape()!;
        for (int i = 1; i <= sbwd.NbEdges(); i++)
        {
            aTShape.myShapes.Append(sbwd.Edge(i));
        }
        NCollection_HSequence<TopoDS_Shape> aNM = sbwd.InternalNonmanifoldEdges();
        for (int i = 1; i <= aNM.Length(); i++)
        {
            aTShape.myShapes.Append(aNM[i]);
        }
        return aCompound;
    }

    // Puts back into the wire data, in order, the children of a compound made by edgesCompound.
    private static void setEdges(ShapeExtend_WireData sbwd, in TopoDS_Shape theCompound)
    {
        int aNbEdges = sbwd.NbEdges();
        NCollection_HSequence<TopoDS_Shape> aNM = sbwd.InternalNonmanifoldEdges();
        int i = 1;
        for (var anIt = new TopoDS_Iterator(theCompound, false, false); anIt.More(); anIt.Next(), i++)
        {
            TopoDS_Shape anEdge = anIt.Value();
            if (i <= aNbEdges)
            {
                sbwd.InternalSetEdge(TopoDS.Edge(anEdge), i);
            }
            else
            {
                aNM.SetValue(i - aNbEdges, anEdge);
            }
        }
    }
}
