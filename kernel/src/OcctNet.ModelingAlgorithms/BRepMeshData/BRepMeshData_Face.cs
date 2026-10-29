// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Face.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMeshData/BRepMeshData_Face.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The default discrete face.</summary>
public class BRepMeshData_Face : IMeshData_Face
{
    private readonly NCollection_DynamicArray<IMeshData_Wire> myDWires;

    /// <summary>An empty discrete face of the face.</summary>
    public BRepMeshData_Face(in TopoDS_Face theFace) : base(theFace)
    {
        myDWires = new NCollection_DynamicArray<IMeshData_Wire>(256);
    }

    /// <inheritdoc/>
    public override int WiresNb() => myDWires.Length();

    /// <inheritdoc/>
    public override IMeshData_Wire AddWire(in TopoDS_Wire theWire, int theEdgeNb = 0)
    {
        IMeshData_Wire aWire = new BRepMeshData_Wire(theWire, theEdgeNb);
        myDWires.Append(aWire);
        return GetWire(WiresNb() - 1);
    }

    /// <inheritdoc/>
    public override IMeshData_Wire GetWire(int theIndex) => myDWires.Value(theIndex);
}
