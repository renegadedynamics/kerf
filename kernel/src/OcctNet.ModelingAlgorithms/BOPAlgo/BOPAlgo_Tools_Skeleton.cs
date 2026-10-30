// 6F skeleton of the rest of BOPAlgo_Tools (to be moved into BOPAlgo_Tools.cs by the port).

namespace Occt;

public static partial class BOPAlgo_Tools
{
    public static int EdgesToWires(in TopoDS_Shape theEdges, ref TopoDS_Shape theWires, bool theShared = false, double theAngTol = 1.0e-8)
        => throw new NotImplementedException();

    public static bool WiresToFaces(in TopoDS_Shape theWires, ref TopoDS_Shape theFaces, double theAngTol = 1.0e-8)
        => throw new NotImplementedException();

    public static void ClassifyFaces(NCollection_List<TopoDS_Shape> theFaces,
                                     NCollection_List<TopoDS_Shape> theSolids,
                                     bool theRunParallel,
                                     IntTools_Context theContext,
                                     NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theInParts,
                                     NCollection_DataMap<TopoDS_Shape, Bnd_Box>? theShapeBoxMap = null,
                                     NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>? theSolidsIF = null,
                                     Message_ProgressRange theRange = default)
        => throw new NotImplementedException();

    public static void FillInternals(NCollection_List<TopoDS_Shape> theSolids,
                                     NCollection_List<TopoDS_Shape> theParts,
                                     NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theImages,
                                     IntTools_Context theContext)
        => throw new NotImplementedException();
}
