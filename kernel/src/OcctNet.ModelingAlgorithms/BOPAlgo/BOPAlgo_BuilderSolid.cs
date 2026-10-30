// 6F skeleton (to be replaced by the port of BOPAlgo_BuilderSolid.hxx/.cxx).

namespace Occt;

/// <summary>Builds the solids bounded by a set of faces.</summary>
public class BOPAlgo_BuilderSolid : BOPAlgo_BuilderArea
{
    private readonly NCollection_DataMap<TopoDS_Shape, Bnd_Box> myBoxes = new(TopTools_ShapeMapHasher.Instance);

    public BOPAlgo_BuilderSolid()
    {
    }

    public override void Perform(Message_ProgressRange theRange = default) => throw new NotImplementedException();

    /// <summary>The boxes of the created solids.</summary>
    public NCollection_DataMap<TopoDS_Shape, Bnd_Box> GetBoxesMap() => myBoxes;

    protected override void PerformShapesToAvoid(Message_ProgressRange theRange) => throw new NotImplementedException();

    protected override void PerformLoops(Message_ProgressRange theRange) => throw new NotImplementedException();

    protected override void PerformAreas(Message_ProgressRange theRange) => throw new NotImplementedException();

    protected override void PerformInternalShapes(Message_ProgressRange theRange) => throw new NotImplementedException();
}
