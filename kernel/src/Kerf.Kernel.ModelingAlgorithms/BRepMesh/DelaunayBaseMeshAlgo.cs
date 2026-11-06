// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelaunayBaseMeshAlgo.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelaunayBaseMeshAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Builds a face's triangulation with Watson's Delaunay algorithm (BRepMesh_Delaun) from the model's raw data.
/// </summary>
/// <remarks>
/// The vertex order 1..n is sorted by BRepMesh_Delaun with MSVC's heap algorithm (decision 491), so ties are broken as
/// in native OCCT.
/// </remarks>
internal class DelaunayBaseMeshAlgo : ConstrainedBaseMeshAlgo
{
    /// <summary>An algorithm.</summary>
    public DelaunayBaseMeshAlgo()
    {
    }

    /// <summary>Generates the mesh of the contour stored in the data structure.</summary>
    protected override void GenerateMesh(ProgressRange range)
    {
        DataStructureOfDelaun structure = GetStructure();
        List<Point3D> nodesMap = GetNodesMap();

        List<int> verticesOrder = new(nodesMap.Count);
        for (int i = 1; i <= nodesMap.Count; ++i)
        {
            verticesOrder.Add(i);
        }

        (int first, int second) cellsCount = GetCellsCount(verticesOrder.Count);
        Delaun mesher = new(structure, verticesOrder, cellsCount.first, cellsCount.second);
        MeshTool cleaner = new(structure);
        cleaner.EraseFreeLinks();

        if (!range.More())
        {
            return;
        }
        PostProcessMesh(mesher, range);
    }
}
