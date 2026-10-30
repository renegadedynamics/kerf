// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Defeaturing.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Defeaturing.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The API of the removal of features (holes, bosses, pockets, fillets, chamfers) from a solid, a compsolid or a
/// compound of solids, over <see cref="BOPAlgo_RemoveFeatures"/>.
/// </summary>
/// <remarks>
/// Set the shape and the faces to remove, then <see cref="Build"/>. The result (<see cref="BRepBuilderAPI_MakeShape.Shape"/>)
/// and the history (<see cref="History"/>, <c>Modified</c>, <c>Generated</c>, <see cref="IsDeleted"/>) are published
/// frozen; they answer for the input's sub-shapes. Features that can't be removed are reported as warnings
/// (<see cref="BOPAlgo_AlertUnableToRemoveTheFeature"/>). The input shape is never written.
/// </remarks>
public class BRepAlgoAPI_Defeaturing : BRepAlgoAPI_Algo
{
    protected TopoDS_Shape myInputShape = new();                     //!< Input shape to remove the features from
    protected NCollection_List<TopoDS_Shape> myFacesToRemove = new(); //!< Features to remove from the shape
    protected bool myFillHistory;                                     //!< Defines whether to track the history of shapes modifications or not (true by default)
    protected BOPAlgo_RemoveFeatures myFeatureRemovalTool = new();    //!< Tool for the features removal

    /// <summary>Empty constructor.</summary>
    public BRepAlgoAPI_Defeaturing()
    {
        myFillHistory = true;
    }

    /// <summary>Sets the shape for processing: a SOLID, COMPSOLID or COMPOUND of solids.</summary>
    public void SetShape(in TopoDS_Shape theShape) => myInputShape = theShape;

    /// <summary>The input shape.</summary>
    public TopoDS_Shape InputShape() => myInputShape;

    /// <summary>Adds the faces of <paramref name="theFace"/> to the features to remove.</summary>
    public void AddFaceToRemove(in TopoDS_Shape theFace) => myFacesToRemove.Append(theFace);

    /// <summary>Adds the faces of the shapes of <paramref name="theFaces"/> to the features to remove.</summary>
    public void AddFacesToRemove(NCollection_List<TopoDS_Shape> theFaces)
    {
        NCollection_List<TopoDS_Shape>.Iterator it = new(theFaces);
        for (; it.More(); it.Next())
        {
            myFacesToRemove.Append(it.Value());
        }
    }

    /// <summary>The shapes requested for removal.</summary>
    public NCollection_List<TopoDS_Shape> FacesToRemove() => myFacesToRemove;

    /// <summary>Performs the operation.</summary>
    public override void Build(Message_ProgressRange theRange = default)
    {
        // Set not done state for the operation
        NotDone();

        // Clear the tools performing the operation
        Clear();

        // Set the inputs to BOPAlgo_RemoveFeatures algorithm
        myFeatureRemovalTool.SetShape(myInputShape);
        myFeatureRemovalTool.AddFacesToRemove(myFacesToRemove);
        myFeatureRemovalTool.SetToFillHistory(myFillHistory);
        myFeatureRemovalTool.SetRunParallel(RunParallel());

        // Perform the features removal
        myFeatureRemovalTool.Perform(theRange);

        // Merge the Errors/Warnings from the features removal tool
        GetReport().Merge(myFeatureRemovalTool.GetReport());

        if (HasErrors())
        {
            return;
        }

        // Set done state
        Done();

        // Get the result shape
        myShape = myFeatureRemovalTool.InternalShape();
    }

    /// <summary>Defines whether to track the modification of the shapes or not.</summary>
    public void SetToFillHistory(bool theFlag) => myFillHistory = theFlag;

    /// <summary>Whether the history was requested.</summary>
    public bool HasHistory() => myFillHistory;

    /// <summary>The algorithm's own list behind <c>Modified</c>: the shapes modified from <paramref name="theS"/>.</summary>
    internal override NCollection_List<TopoDS_Shape> InternalModified(in TopoDS_Shape theS)
    {
        myGenerated.Clear();
        foreach (TopoDS_Shape aS in myFeatureRemovalTool.Modified(theS))
        {
            myGenerated.Append(aS);
        }
        return myGenerated;
    }

    /// <summary>The algorithm's own list behind <c>Generated</c>: the shapes generated from <paramref name="theS"/>.</summary>
    internal override NCollection_List<TopoDS_Shape> InternalGenerated(in TopoDS_Shape theS)
    {
        myGenerated.Clear();
        foreach (TopoDS_Shape aS in myFeatureRemovalTool.Generated(theS))
        {
            myGenerated.Append(aS);
        }
        return myGenerated;
    }

    /// <summary>True if <paramref name="theS"/> has no trace in the result.</summary>
    public override bool IsDeleted(in TopoDS_Shape theS) => myFeatureRemovalTool.IsDeleted(theS);

    /// <summary>True if any of the input shapes has been modified.</summary>
    public virtual bool HasModified() => myFeatureRemovalTool.HasModified();

    /// <summary>True if any of the input shapes has generated shapes.</summary>
    public virtual bool HasGenerated() => myFeatureRemovalTool.HasGenerated();

    /// <summary>True if any of the input shapes has been deleted.</summary>
    public virtual bool HasDeleted() => myFeatureRemovalTool.HasDeleted();

    /// <summary>The history of the shapes' modifications (the result is frozen first; null without history).</summary>
    public BRepTools_History? History()
    {
        BeforePublishingLists();
        return myFeatureRemovalTool.History();
    }

    /// <summary>The history without freezing the result (OCCT's <c>History()</c>), for ported code.</summary>
    internal BRepTools_History? InternalHistory() => myFeatureRemovalTool.InternalHistory();

    // The lists hold sub-shapes of the result: the whole result is frozen with them (decision 414).
    private protected override void BeforePublishingLists() => myShape.TShape()?.FreezeGraph();

    /// <summary>Clears the tools performing the operation; the options and inputs stay.</summary>
    public override void Clear()
    {
        base.Clear();
        myFeatureRemovalTool.Clear();
    }
}
