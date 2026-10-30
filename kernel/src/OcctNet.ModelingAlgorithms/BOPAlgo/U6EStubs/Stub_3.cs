// TEMPORARY unit 6E stub, deleted when BOPAlgo_PaveFiller_3 is ported. Not a port.
namespace Occt;

public partial class BOPAlgo_PaveFiller
{
    protected virtual void PerformEE(Message_ProgressRange theRange) => throw new NotImplementedException();
    protected void PerformNewVertices(NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> theMVCPB, Message_ProgressRange theRange, bool theIsEEIntersection = true) => throw new NotImplementedException();
    protected void TreatNewVertices(NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> theMVCPB, NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theImages) => throw new NotImplementedException();
    protected void FillShrunkData(BOPDS_PaveBlock thePB) => throw new NotImplementedException();
    protected void AnalyzeShrunkData(BOPDS_PaveBlock thePB, IntTools_ShrunkRange theSR) => throw new NotImplementedException();
    protected bool ForceInterfVE(int nV, BOPDS_PaveBlock aPB, NCollection_Map<int> theMEdges) => throw new NotImplementedException();
    protected bool GetPBBox(in TopoDS_Edge theE, BOPDS_PaveBlock thePB, NCollection_DataMap<BOPDS_PaveBlock, Bnd_Box> thePBBox, ref double theFirst, ref double theLast, ref double theSFirst, ref double theSLast, ref Bnd_Box theBox) => throw new NotImplementedException();
    protected void UpdateVerticesOfCB() => throw new NotImplementedException();
    protected void ForceInterfEE(Message_ProgressRange theRange) => throw new NotImplementedException();
}
