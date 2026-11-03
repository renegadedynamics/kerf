// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Wire.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Wire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing.Model;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Meshing.Data;

/// <summary>The default discrete wire.</summary>
public class WireData : WireModel
{
    private readonly NCollection_DynamicArray<EdgeModel> _dEdges;
    private readonly NCollection_DynamicArray<Orientation> _dEdgesOri;

    /// <summary>An empty discrete wire of the wire, expecting <paramref name="edgeNb"/> edges (256 if not positive).</summary>
    public WireData(in Wire wire, int edgeNb) : base(wire)
    {
        _dEdges = new NCollection_DynamicArray<EdgeModel>(edgeNb > 0 ? edgeNb : 256);
        _dEdgesOri = new NCollection_DynamicArray<Orientation>(edgeNb > 0 ? edgeNb : 256);
    }

    /// <inheritdoc/>
    public override int EdgesNb() => _dEdges.Length();

    /// <inheritdoc/>
    public override int AddEdge(EdgeModel dEdge, Orientation orientation)
    {
        int index = EdgesNb();

        _dEdges.Append(dEdge);
        _dEdgesOri.Append(orientation);

        return index;
    }

    /// <inheritdoc/>
    public override EdgeModel GetEdge(int index) => _dEdges.Value(index);

    /// <inheritdoc/>
    public override Orientation GetEdgeOrientation(int index) => _dEdgesOri.Value(index);
}
