// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Face.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Face.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing.Model;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Meshing.Data;

/// <summary>The default discrete face.</summary>
internal class FaceData : FaceModel
{
    private readonly List<WireModel> _dWires;

    /// <summary>An empty discrete face of the face.</summary>
    public FaceData(in Face face) : base(face)
    {
        _dWires = new List<WireModel>();
    }

    /// <inheritdoc/>
    public override int WiresNb() => _dWires.Count;

    /// <inheritdoc/>
    public override WireModel AddWire(in Wire wire, int edgeNb = 0)
    {
        WireModel aWire = new WireData(wire, edgeNb);
        _dWires.Add(aWire);
        return GetWire(WiresNb() - 1);
    }

    /// <inheritdoc/>
    public override WireModel GetWire(int index) => _dWires[index];
}
