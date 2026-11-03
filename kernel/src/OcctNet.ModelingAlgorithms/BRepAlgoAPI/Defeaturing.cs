// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Defeaturing.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Defeaturing.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Api;

/// <summary>
/// The API of the removal of features (holes, bosses, pockets, fillets, chamfers) from a solid, a compsolid or a
/// compound of solids, over <see cref="RemoveFeaturesAlgorithm"/>.
/// </summary>
/// <remarks>
/// Set the shape and the faces to remove, then <see cref="Build"/>. The result (<see cref="MakeShape.Shape"/>)
/// and the history (<see cref="History"/>, <c>Modified</c>, <c>Generated</c>, <see cref="IsDeleted"/>) are published
/// frozen; they answer for the input's sub-shapes. Features that can't be removed are reported as warnings
/// (<see cref="AlertUnableToRemoveTheFeature"/>). The input shape is never written.
/// </remarks>
public class Defeaturing : Algo
{
    protected Shape _inputShape = new();                     //!< Input shape to remove the features from
    protected NCollection_List<Shape> _facesToRemove = new(); //!< Features to remove from the shape
    protected bool _fillHistory;                                     //!< Defines whether to track the history of shapes modifications or not (true by default)
    protected RemoveFeaturesAlgorithm _featureRemovalTool = new();    //!< Tool for the features removal

    /// <summary>Empty constructor.</summary>
    public Defeaturing()
    {
        _fillHistory = true;
    }

    /// <summary>Sets the shape for processing: a SOLID, COMPSOLID or COMPOUND of solids.</summary>
    public void SetShape(in Shape shape) => _inputShape = shape;

    /// <summary>The input shape.</summary>
    public Shape InputShape() => _inputShape;

    /// <summary>Adds the faces of <paramref name="face"/> to the features to remove.</summary>
    public void AddFaceToRemove(in Shape face) => _facesToRemove.Append(face);

    /// <summary>Adds the faces of the shapes of <paramref name="faces"/> to the features to remove.</summary>
    public void AddFacesToRemove(NCollection_List<Shape> faces)
    {
        NCollection_List<Shape>.Iterator it = new(faces);
        for (; it.More(); it.Next())
        {
            _facesToRemove.Append(it.Value());
        }
    }

    /// <summary>The shapes requested for removal.</summary>
    public NCollection_List<Shape> FacesToRemove() => _facesToRemove;

    /// <summary>Performs the operation.</summary>
    public override void Build(ProgressRange range = default)
    {
        // Set not done state for the operation
        NotDone();

        // Clear the tools performing the operation
        Clear();

        // Set the inputs to BOPAlgo_RemoveFeatures algorithm
        _featureRemovalTool.SetShape(_inputShape);
        _featureRemovalTool.AddFacesToRemove(_facesToRemove);
        _featureRemovalTool.SetToFillHistory(_fillHistory);
        _featureRemovalTool.SetRunParallel(RunParallel());

        // Perform the features removal
        _featureRemovalTool.Perform(range);

        // Merge the Errors/Warnings from the features removal tool
        GetReport().Merge(_featureRemovalTool.GetReport());

        if (HasErrors)
        {
            return;
        }

        // Set done state
        Done();

        // Get the result shape
        _shape = _featureRemovalTool.InternalShape();
    }

    /// <summary>Defines whether to track the modification of the shapes or not.</summary>
    public void SetToFillHistory(bool flag) => _fillHistory = flag;

    /// <summary>Whether the history was requested.</summary>
    public bool HasHistory => _fillHistory;

    /// <summary>The algorithm's own list behind <c>Modified</c>: the shapes modified from <paramref name="s"/>.</summary>
    internal override NCollection_List<Shape> InternalModified(in Shape s)
    {
        _generated.Clear();
        foreach (Shape aS in _featureRemovalTool.Modified(s))
        {
            _generated.Append(aS);
        }
        return _generated;
    }

    /// <summary>The algorithm's own list behind <c>Generated</c>: the shapes generated from <paramref name="s"/>.</summary>
    internal override NCollection_List<Shape> InternalGenerated(in Shape s)
    {
        _generated.Clear();
        foreach (Shape aS in _featureRemovalTool.Generated(s))
        {
            _generated.Append(aS);
        }
        return _generated;
    }

    /// <summary>True if <paramref name="s"/> has no trace in the result.</summary>
    public override bool IsDeleted(in Shape s) => _featureRemovalTool.IsDeleted(s);

    /// <summary>True if any of the input shapes has been modified.</summary>
    public virtual bool HasModified => _featureRemovalTool.HasModified;

    /// <summary>True if any of the input shapes has generated shapes.</summary>
    public virtual bool HasGenerated => _featureRemovalTool.HasGenerated;

    /// <summary>True if any of the input shapes has been deleted.</summary>
    public virtual bool HasDeleted => _featureRemovalTool.HasDeleted;

    /// <summary>The history of the shapes' modifications (the result is frozen first; null without history).</summary>
    public History? History()
    {
        BeforePublishingLists();
        return _featureRemovalTool.History();
    }

    /// <summary>The history without freezing the result (OCCT's <c>History()</c>), for ported code.</summary>
    internal History? InternalHistory() => _featureRemovalTool.InternalHistory();

    // The lists hold sub-shapes of the result: the whole result is frozen with them (decision 414).
    private protected override void BeforePublishingLists() => _shape.TShape()?.FreezeGraph();

    /// <summary>Clears the tools performing the operation; the options and inputs stay.</summary>
    public override void Clear()
    {
        base.Clear();
        _featureRemovalTool.Clear();
    }
}
