// TEMPORARY unit 6E stub, deleted when BOPAlgo_PaveFiller_6 (first half) is ported. Not a port.
namespace Occt;

public partial class BOPAlgo_PaveFiller
{
    protected virtual void PerformFF(Message_ProgressRange theRange) => throw new NotImplementedException();
    protected void MakeBlocks(Message_ProgressRange theRange) => throw new NotImplementedException();
    protected void MakeSDVerticesFF(NCollection_DataMap<int, NCollection_List<int>> aDMVLV, NCollection_DataMap<int, int> theDMNewSD) => throw new NotImplementedException();
    protected void PostTreatFF(NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> theMSCPB, NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<BOPDS_PaveBlock>> theDMExEdges, NCollection_DataMap<int, int> theDMNewSD, NCollection_IndexedMap<BOPDS_PaveBlock> theMicroPB, NCollection_IndexedMap<TopoDS_Shape> theVertsOnRejectedPB, Message_ProgressRange theRange) => throw new NotImplementedException();
    protected void UpdateFaceInfo(NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<BOPDS_PaveBlock>> theDME, NCollection_DataMap<int, int> theDMV, NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<int>> thePBFacesMap) => throw new NotImplementedException();
    protected bool IsExistingVertex(in gp_Pnt theP, double theTol, NCollection_Map<int> theMVOn) => throw new NotImplementedException();
    protected bool IsExistingPaveBlock(BOPDS_PaveBlock thePB, BOPDS_Curve theNC, NCollection_List<int> theLSE, ref int theNEOut, ref double theTolNew) => throw new NotImplementedException();
    protected bool IsExistingPaveBlock(BOPDS_PaveBlock thePB, BOPDS_Curve theNC, double theTolR3D, NCollection_IndexedMap<BOPDS_PaveBlock> theMPB, BOPTools_BoxTree thePBTree, NCollection_Map<BOPDS_PaveBlock> theMPBCommon, ref BOPDS_PaveBlock? thePBOut, ref double theTolNew) => throw new NotImplementedException();
}
