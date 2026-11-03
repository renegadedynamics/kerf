// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Wire.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Wire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;

namespace Occt;

/// <summary>The default discrete wire.</summary>
public class BRepMeshData_Wire : IMeshData_Wire
{
    private readonly NCollection_DynamicArray<IMeshData_Edge> myDEdges;
    private readonly NCollection_DynamicArray<Orientation> myDEdgesOri;

    /// <summary>An empty discrete wire of the wire, expecting <paramref name="theEdgeNb"/> edges (256 if not positive).</summary>
    public BRepMeshData_Wire(in Wire theWire, int theEdgeNb) : base(theWire)
    {
        myDEdges = new NCollection_DynamicArray<IMeshData_Edge>(theEdgeNb > 0 ? theEdgeNb : 256);
        myDEdgesOri = new NCollection_DynamicArray<Orientation>(theEdgeNb > 0 ? theEdgeNb : 256);
    }

    /// <inheritdoc/>
    public override int EdgesNb() => myDEdges.Length();

    /// <inheritdoc/>
    public override int AddEdge(IMeshData_Edge theDEdge, Orientation theOrientation)
    {
        int aIndex = EdgesNb();

        myDEdges.Append(theDEdge);
        myDEdgesOri.Append(theOrientation);

        return aIndex;
    }

    /// <inheritdoc/>
    public override IMeshData_Edge GetEdge(int theIndex) => myDEdges.Value(theIndex);

    /// <inheritdoc/>
    public override Orientation GetEdgeOrientation(int theIndex) => myDEdgesOri.Value(theIndex);
}
