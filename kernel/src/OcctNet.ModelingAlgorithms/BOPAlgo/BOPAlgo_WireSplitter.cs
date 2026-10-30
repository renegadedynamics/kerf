// 6F skeleton (to be replaced by the port of BOPAlgo_WireSplitter.hxx/.lxx/.cxx and _1.cxx).

namespace Occt;

/// <summary>Builds the loops (wires) from a set of edges on a face.</summary>
public partial class BOPAlgo_WireSplitter : BOPAlgo_Algo
{
    protected BOPAlgo_WireEdgeSet? myWES;
    protected NCollection_List<BOPTools_ConnexityBlock> myLCB = new();
    protected IntTools_Context? myContext;

    public BOPAlgo_WireSplitter()
    {
    }

    public void SetWES(BOPAlgo_WireEdgeSet theWES) => myWES = theWES;

    public BOPAlgo_WireEdgeSet WES() => myWES!;

    public void SetContext(IntTools_Context? theContext) => myContext = theContext;

    public IntTools_Context? Context() => myContext;

    public override void Perform(Message_ProgressRange theRange = default) => throw new NotImplementedException();

    public static void MakeWire(NCollection_List<TopoDS_Shape> theLE, ref TopoDS_Wire theW) => throw new NotImplementedException();

    public static void SplitBlock(in TopoDS_Face theF, BOPTools_ConnexityBlock theCB, IntTools_Context theContext) => throw new NotImplementedException();

    protected override void CheckData() => throw new NotImplementedException();

    protected void MakeWires(Message_ProgressRange theRange) => throw new NotImplementedException();
}
