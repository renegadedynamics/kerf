// TEMPORARY unit 6E stub, deleted when BOPAlgo_PaveFiller_5 is ported. Not a port.
namespace Occt;

public partial class BOPAlgo_PaveFiller
{
    protected virtual void PerformEF(Message_ProgressRange theRange) => throw new NotImplementedException();
    protected static bool CheckFacePaves(int theN, NCollection_Map<int> theMIFOn, NCollection_Map<int> theMIFIn) => throw new NotImplementedException();
    protected bool CheckFacePaves(in TopoDS_Vertex theVnew, NCollection_Map<int> theMIF) => throw new NotImplementedException();
    protected bool ForceInterfVF(int nV, int nF) => throw new NotImplementedException();
    protected void ReduceIntersectionRange(int theV1, int theV2, int theE, int theF, ref double theTS1, ref double theTS2) => throw new NotImplementedException();
    protected void ForceInterfEF(Message_ProgressRange theRange) => throw new NotImplementedException();
    protected void ForceInterfEF(NCollection_IndexedMap<BOPDS_PaveBlock> theMPB, Message_ProgressRange theRange, bool theAddInterf) => throw new NotImplementedException();
}
