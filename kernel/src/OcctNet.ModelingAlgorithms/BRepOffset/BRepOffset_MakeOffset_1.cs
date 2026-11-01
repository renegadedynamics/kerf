// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_MakeOffset_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

public sealed partial class BRepOffset_MakeOffset
{
    private void BuildSplitsOfExtendedFaces(NCollection_List<TopoDS_Shape> theLF, BRepOffset_Analyse theAnalyse,
                                            BRepAlgo_AsDes theAsDes,
                                            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEdgesOrigins,
                                            NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theFacesOrigins,
                                            NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theETrimEInf,
                                            BRepAlgo_Image theImage, Message_ProgressRange theRange)
    {
        throw new NotImplementedException();
    }

    private void BuildSplitsOfTrimmedFaces(NCollection_List<TopoDS_Shape> theLF, BRepAlgo_AsDes theAsDes,
                                           BRepAlgo_Image theImage, Message_ProgressRange theRange)
    {
        throw new NotImplementedException();
    }
}
