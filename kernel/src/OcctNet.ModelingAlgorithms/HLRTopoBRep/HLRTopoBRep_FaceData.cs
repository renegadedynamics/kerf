// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_FaceData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_FaceData.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_FaceData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The internal outlines, outlines and isolines of a face.</summary>
/// <remarks>
/// OCCT's value class, stored by value in HLRTopoBRep_Data's map and reached by reference; a class here, the map
/// holding one instance per face.
/// </remarks>
public sealed class HLRTopoBRep_FaceData
{
    private readonly NCollection_List<TopoDS_Shape> myIntL = new();
    private readonly NCollection_List<TopoDS_Shape> myOutL = new();
    private readonly NCollection_List<TopoDS_Shape> myIsoL = new();

    public HLRTopoBRep_FaceData()
    {
    }

    public NCollection_List<TopoDS_Shape> FaceIntL() => myIntL;

    public NCollection_List<TopoDS_Shape> FaceOutL() => myOutL;

    public NCollection_List<TopoDS_Shape> FaceIsoL() => myIsoL;

    public NCollection_List<TopoDS_Shape> AddIntL() => myIntL;

    public NCollection_List<TopoDS_Shape> AddOutL() => myOutL;

    public NCollection_List<TopoDS_Shape> AddIsoL() => myIsoL;
}
