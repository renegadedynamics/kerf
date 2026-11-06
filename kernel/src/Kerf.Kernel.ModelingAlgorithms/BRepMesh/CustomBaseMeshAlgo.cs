// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CustomBaseMeshAlgo.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CustomBaseMeshAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The base of the face algorithms building the triangulation with a custom triangulation algorithm (the base
/// triangulation), whose constraints BRepMesh_Delaun then restores.
/// </summary>
internal abstract class CustomBaseMeshAlgo : ConstrainedBaseMeshAlgo
{
    /// <summary>An algorithm.</summary>
    protected CustomBaseMeshAlgo()
    {
    }

    /// <summary>Generates the mesh of the contour stored in the data structure.</summary>
    protected override void GenerateMesh(ProgressRange range)
    {
        DataStructureOfDelaun structure = GetStructure();
        int nodesNb = structure.NodeCount;

        BuildBaseTriangulation();

        (int first, int second) cellsCount = GetCellsCount(structure.NodeCount);
        Delaun mesher = new(structure, cellsCount.first, cellsCount.second, false);

        int newNodesNb = structure.NodeCount;
        bool isRemoveAux = newNodesNb > nodesNb;
        if (isRemoveAux)
        {
            List<int> auxVertices = new(newNodesNb - nodesNb);
            for (int extNodesIt = nodesNb + 1; extNodesIt <= newNodesNb; ++extNodesIt)
            {
                auxVertices.Add(extNodesIt);
            }

            // Set aux vertices if there are some to clean up mesh correctly.
            mesher.SetAuxVertices(auxVertices);
        }

        mesher.ProcessConstraints();

        // Destruction of triangles containing aux vertices added (possibly) during base mesh computation.
        if (isRemoveAux)
        {
            mesher.RemoveAuxElements();
        }

        MeshTool cleaner = new(structure);
        cleaner.EraseFreeLinks();

        PostProcessMesh(mesher, range);
    }

    /// <summary>Builds the base triangulation with the custom triangulation algorithm.</summary>
    protected abstract void BuildBaseTriangulation();
}
