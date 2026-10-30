// 6F skeleton (to be replaced by the port of BOPAlgo_ShellSplitter.hxx/.cxx).

namespace Occt;

/// <summary>Builds the loops (shells) from a set of faces.</summary>
public class BOPAlgo_ShellSplitter : BOPAlgo_Algo
{
    protected NCollection_List<TopoDS_Shape> myStartShapes = new();
    protected NCollection_List<TopoDS_Shape> myShells = new();
    protected NCollection_List<BOPTools_ConnexityBlock> myLCB = new();

    public BOPAlgo_ShellSplitter()
    {
    }

    public void AddStartElement(in TopoDS_Shape theS) => throw new NotImplementedException();

    public NCollection_List<TopoDS_Shape> StartElements() => myStartShapes;

    public override void Perform(Message_ProgressRange theRange = default) => throw new NotImplementedException();

    public NCollection_List<TopoDS_Shape> Shells() => myShells;

    public static void SplitBlock(BOPTools_ConnexityBlock theCB) => throw new NotImplementedException();

    protected void MakeShells(Message_ProgressRange theRange) => throw new NotImplementedException();
}
