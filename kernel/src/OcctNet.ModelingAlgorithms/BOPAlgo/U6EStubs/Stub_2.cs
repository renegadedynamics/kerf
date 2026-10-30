// TEMPORARY unit 6E stub, deleted when BOPAlgo_PaveFiller_2 is ported. Not a port.
namespace Occt;

public partial class BOPAlgo_PaveFiller
{
    protected virtual void PerformVE(Message_ProgressRange theRange) => throw new NotImplementedException();
    protected void IntersectVE(NCollection_IndexedDataMap<BOPDS_PaveBlock, NCollection_List<int>> theVEPairs, Message_ProgressRange theRange, bool bAddInterfs = true) => throw new NotImplementedException();
    protected void SplitPaveBlocks(NCollection_Map<int> theMEdges, bool theAddInterfs) => throw new NotImplementedException();
    protected void AddIntersectionFailedWarning(in TopoDS_Shape theS1, in TopoDS_Shape theS2) => throw new NotImplementedException();
}
