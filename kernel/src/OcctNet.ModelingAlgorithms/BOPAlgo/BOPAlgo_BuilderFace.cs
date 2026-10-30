// 6F skeleton (to be replaced by the port of BOPAlgo_BuilderFace.hxx/.cxx).

namespace Occt;

/// <summary>Builds the faces bounded by a set of edges on a surface (the split parts of a face).</summary>
public class BOPAlgo_BuilderFace : BOPAlgo_BuilderArea
{
    protected TopoDS_Face myFace = new();
    protected TopAbs_Orientation myOrientation;

    public BOPAlgo_BuilderFace()
    {
        myOrientation = TopAbs_Orientation.TopAbs_EXTERNAL;
    }

    public void SetFace(in TopoDS_Face theFace) => throw new NotImplementedException();

    public TopoDS_Face Face() => myFace;

    public override void Perform(Message_ProgressRange theRange = default) => throw new NotImplementedException();

    public TopAbs_Orientation Orientation() => myOrientation;

    protected override void PerformShapesToAvoid(Message_ProgressRange theRange) => throw new NotImplementedException();

    protected override void PerformLoops(Message_ProgressRange theRange) => throw new NotImplementedException();

    protected override void PerformAreas(Message_ProgressRange theRange) => throw new NotImplementedException();

    protected override void PerformInternalShapes(Message_ProgressRange theRange) => throw new NotImplementedException();

    protected override void CheckData() => throw new NotImplementedException();
}
