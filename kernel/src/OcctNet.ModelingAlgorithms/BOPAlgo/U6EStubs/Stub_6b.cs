// TEMPORARY unit 6E stub, deleted when BOPAlgo_PaveFiller_6 (second half) is ported. Not a port.
namespace Occt;

public partial class BOPAlgo_PaveFiller
{
    protected void PutBoundPaveOnCurve(in TopoDS_Face theF1, in TopoDS_Face theF2, BOPDS_Curve theNC, NCollection_List<int> theLBV) => throw new NotImplementedException();
    protected void PutPavesOnCurve(NCollection_Map<int> theMVOnIn, NCollection_Map<int> theMVCommon, BOPDS_Curve theNC, NCollection_Map<int> theMI, NCollection_Map<int> theMVEF, NCollection_DataMap<int, double> theMVTol, NCollection_DataMap<int, NCollection_List<int>> theDMVLV) => throw new NotImplementedException();
    protected void FilterPavesOnCurves(NCollection_DynamicArray<BOPDS_Curve> theVNC, NCollection_DataMap<int, double> theMVTol) => throw new NotImplementedException();
    protected bool ExtendedTolerance(int nV, NCollection_Map<int> aMI, ref double aTolVExt, int aType = 0) => throw new NotImplementedException();
    protected void GetEFPnts(int nF1, int nF2, NCollection_List<IntSurf_PntOn2S> aListOfPnts) => throw new NotImplementedException();
    protected void PutEFPavesOnCurve(NCollection_DynamicArray<BOPDS_Curve> theVC, int theIndex, NCollection_Map<int> theMI, NCollection_Map<int> theMVEF, NCollection_DataMap<int, double> theMVTol, NCollection_DataMap<int, NCollection_List<int>> aDMVLV) => throw new NotImplementedException();
    protected void PutStickPavesOnCurve(in TopoDS_Face aF1, in TopoDS_Face aF2, NCollection_Map<int> theMI, NCollection_DynamicArray<BOPDS_Curve> theVC, int theIndex, NCollection_Map<int> theMVStick, NCollection_DataMap<int, double> theMVTol, NCollection_DataMap<int, NCollection_List<int>> aDMVLV) => throw new NotImplementedException();
    protected void GetStickVertices(int nF1, int nF2, NCollection_Map<int> theMVStick, NCollection_Map<int> theMVEF, NCollection_Map<int> theMI) => throw new NotImplementedException();
    protected void GetFullShapeMap(int nF, NCollection_Map<int> theMI) => throw new NotImplementedException();
    protected void RemoveUsedVertices(NCollection_DynamicArray<BOPDS_Curve> theVC, NCollection_Map<int> theMV) => throw new NotImplementedException();
    protected void PutPaveOnCurve(int nV, double theTolR3D, BOPDS_Curve theNC, NCollection_Map<int> theMI, NCollection_DataMap<int, double> theMVTol, NCollection_DataMap<int, NCollection_List<int>> aDMVLV, int aType = 0) => throw new NotImplementedException();
    protected void ProcessExistingPaveBlocks(int theInt, int theCur, int nF1, int nF2, in TopoDS_Edge theES, NCollection_IndexedMap<BOPDS_PaveBlock> theMPBOnIn, BOPTools_BoxTree thePBTree, NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> theMSCPB, NCollection_DataMap<TopoDS_Shape, int> theMVI, NCollection_List<BOPDS_PaveBlock> theLPBC, NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<int>> thePBFacesMap, NCollection_Map<BOPDS_PaveBlock> theMPB) => throw new NotImplementedException();
    protected void ProcessExistingPaveBlocks(int theInt, int nF1, int nF2, NCollection_IndexedMap<BOPDS_PaveBlock> theMPBOnIn, BOPTools_BoxTree thePBTree, NCollection_DataMap<int, NCollection_List<int>> theDMBV, NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> theMSCPB, NCollection_DataMap<TopoDS_Shape, int> theMVI, NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<int>> thePBFacesMap, NCollection_Map<BOPDS_PaveBlock> theMPB) => throw new NotImplementedException();
    protected void UpdateExistingPaveBlocks(BOPDS_PaveBlock thePB, NCollection_List<BOPDS_PaveBlock> theLPB, NCollection_DataMap<BOPDS_PaveBlock, NCollection_List<int>> thePBFacesMap) => throw new NotImplementedException();
    protected void PutClosingPaveOnCurve(BOPDS_Curve aNC) => throw new NotImplementedException();
    protected void PreparePostTreatFF(int aInt, int aCur, BOPDS_PaveBlock aPB, NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> aMSCPB, NCollection_DataMap<TopoDS_Shape, int> aMVI, NCollection_List<BOPDS_PaveBlock> aLPB) => throw new NotImplementedException();
    protected bool CheckPlanes(int nF1, int nF2) => throw new NotImplementedException();
    protected void UpdatePaveBlocks(NCollection_DataMap<int, int> aDMNewSD) => throw new NotImplementedException();
    protected void RemovePaveBlocks(NCollection_Map<int> theEdges) => throw new NotImplementedException();
    protected void UpdateBlocksWithSharedVertices() => throw new NotImplementedException();
    protected bool EstimatePaveOnCurve(int nV, BOPDS_Curve theNC, double theTolR3D) => throw new NotImplementedException();
    protected void CorrectToleranceOfSE() => throw new NotImplementedException();
    protected void PutSEInOtherFaces(Message_ProgressRange theRange) => throw new NotImplementedException();
    protected void RemoveMicroSectionEdges(NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> theMSCPB, NCollection_IndexedMap<BOPDS_PaveBlock> theMicroPB) => throw new NotImplementedException();
    protected void RemoveMicroEdges() => throw new NotImplementedException();
    private static double ToleranceFF(BRepAdaptor_Surface aBAS1, BRepAdaptor_Surface aBAS2) => throw new NotImplementedException();
}
