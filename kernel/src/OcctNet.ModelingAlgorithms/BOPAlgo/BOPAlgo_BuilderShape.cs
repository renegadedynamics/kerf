// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderShape.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;

namespace Occt;

/// <summary>The root of the Boolean component's algorithms that build a shape and its history.</summary>
/// <remarks>
/// <see cref="Shape"/> and <see cref="History"/> are publication points (decision 414): they freeze the result graph.
/// Ported code that goes on building reads <see cref="InternalShape"/> and <see cref="InternalHistory"/>
/// (decision 639). The history's lists are read-only lists (BRepTools_History's port).
/// </remarks>
public abstract class BOPAlgo_BuilderShape : BOPAlgo_Algo
{
    protected TopoDS_Shape myShape = new();                                                //!< Result of the operation
    protected NCollection_List<TopoDS_Shape> myHistShapes = new();                         //!< Storer for the history shapes
    protected NCollection_Map<TopoDS_Shape> myMapShape = new(TopTools_ShapeMapHasher.Instance); //!< cached map of all arguments shapes
    protected bool myFillHistory;                                                          //!< Controls the history filling
    protected BRepTools_History? myHistory;                                                //!< History tool

    protected BOPAlgo_BuilderShape()
    {
        myFillHistory = true;
    }

    /// <summary>The result of the operation (frozen).</summary>
    public TopoDS_Shape Shape()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary>The result, unfrozen, for ported code.</summary>
    internal TopoDS_Shape InternalShape() => myShape;

    /// <summary>The shapes modified from <paramref name="theS"/>.</summary>
    public IBentoList<TopoDS_Shape> Modified(in TopoDS_Shape theS)
    {
        if (myFillHistory && myHistory is not null)
        {
            return myHistory.Modified(theS);
        }
        return PackedList<TopoDS_Shape>.Empty;
    }

    /// <summary>The shapes generated from <paramref name="theS"/>.</summary>
    public IBentoList<TopoDS_Shape> Generated(in TopoDS_Shape theS)
    {
        if (myFillHistory && myHistory is not null)
        {
            return myHistory.Generated(theS);
        }
        return PackedList<TopoDS_Shape>.Empty;
    }

    /// <summary>True if <paramref name="theS"/> has been deleted: neither it nor a split of it is in the result.</summary>
    public bool IsDeleted(in TopoDS_Shape theS) => myFillHistory && myHistory is not null && myHistory.IsRemoved(theS);

    public bool HasModified() => myFillHistory && myHistory is not null && myHistory.HasModified();

    public bool HasGenerated() => myFillHistory && myHistory is not null && myHistory.HasGenerated();

    public bool HasDeleted() => myFillHistory && myHistory is not null && myHistory.HasRemoved();

    /// <summary>The history of the operation (null when not collected); the result graph is frozen first.</summary>
    public BRepTools_History? History()
    {
        myShape.TShape()?.FreezeGraph();
        return InternalHistory();
    }

    /// <summary>OCCT's <c>History()</c>, without freezing: makes an empty history if there is none.</summary>
    internal BRepTools_History? InternalHistory()
    {
        if (myFillHistory)
        {
            myHistory ??= new BRepTools_History();
            return myHistory;
        }
        return null;
    }

    /// <summary>Enables or disables the collection of the history.</summary>
    public void SetToFillHistory(bool theHistFlag) => myFillHistory = theHistFlag;

    public bool HasHistory() => myFillHistory;

    public override void Clear()
    {
        base.Clear();
        myHistory = null;
        myMapShape.Clear();
    }
}
