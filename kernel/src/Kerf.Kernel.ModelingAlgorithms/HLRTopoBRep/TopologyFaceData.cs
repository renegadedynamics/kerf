// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_FaceData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_FaceData.lxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_FaceData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.HiddenLines.Topology;

/// <summary>The internal outlines, outlines and isolines of a face.</summary>
/// <remarks>
/// OCCT's value class, stored by value in HLRTopoBRep_Data's map and reached by reference; a class here, the map
/// holding one instance per face.
/// </remarks>
internal sealed class TopologyFaceData
{
    private readonly NCollection_List<Shape> _intL = new();
    private readonly NCollection_List<Shape> _outL = new();
    private readonly NCollection_List<Shape> _isoL = new();

    public TopologyFaceData()
    {
    }

    public NCollection_List<Shape> FaceIntL() => _intL;

    public NCollection_List<Shape> FaceOutL() => _outL;

    public NCollection_List<Shape> FaceIsoL() => _isoL;

    public NCollection_List<Shape> AddIntL() => _intL;

    public NCollection_List<Shape> AddOutL() => _outL;

    public NCollection_List<Shape> AddIsoL() => _isoL;
}
