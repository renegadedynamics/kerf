// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelaunayBaseMeshAlgo.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelaunayBaseMeshAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Builds a face's triangulation with Watson's Delaunay algorithm (BRepMesh_Delaun) from the model's raw data.
/// </summary>
/// <remarks>
/// The vertex order 1..n is sorted by BRepMesh_Delaun with MSVC's heap algorithm (decision 491), so ties are broken as
/// in native OCCT.
/// </remarks>
public class BRepMesh_DelaunayBaseMeshAlgo : BRepMesh_ConstrainedBaseMeshAlgo
{
    /// <summary>An algorithm.</summary>
    public BRepMesh_DelaunayBaseMeshAlgo()
    {
    }

    /// <summary>Generates the mesh of the contour stored in the data structure.</summary>
    protected override void generateMesh(ProgressRange theRange)
    {
        BRepMesh_DataStructureOfDelaun aStructure = getStructure();
        NCollection_DynamicArray<Point3D> aNodesMap = getNodesMap();

        NCollection_DynamicArray<int> aVerticesOrder = new(aNodesMap.Length());
        for (int i = 1; i <= aNodesMap.Length(); ++i)
        {
            aVerticesOrder.Append(i);
        }

        (int first, int second) aCellsCount = getCellsCount(aVerticesOrder.Length());
        BRepMesh_Delaun aMesher = new(aStructure, aVerticesOrder, aCellsCount.first, aCellsCount.second);
        BRepMesh_MeshTool aCleaner = new(aStructure);
        aCleaner.EraseFreeLinks();

        if (!theRange.More())
        {
            return;
        }
        postProcessMesh(aMesher, theRange);
    }
}
