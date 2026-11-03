// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CustomBaseMeshAlgo.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CustomBaseMeshAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// The base of the face algorithms building the triangulation with a custom triangulation algorithm (the base
/// triangulation), whose constraints BRepMesh_Delaun then restores.
/// </summary>
public abstract class BRepMesh_CustomBaseMeshAlgo : BRepMesh_ConstrainedBaseMeshAlgo
{
    /// <summary>An algorithm.</summary>
    protected BRepMesh_CustomBaseMeshAlgo()
    {
    }

    /// <summary>Generates the mesh of the contour stored in the data structure.</summary>
    protected override void generateMesh(ProgressRange theRange)
    {
        BRepMesh_DataStructureOfDelaun aStructure = getStructure();
        int aNodesNb = aStructure.NbNodes();

        buildBaseTriangulation();

        (int first, int second) aCellsCount = getCellsCount(aStructure.NbNodes());
        BRepMesh_Delaun aMesher = new(aStructure, aCellsCount.first, aCellsCount.second, false);

        int aNewNodesNb = aStructure.NbNodes();
        bool isRemoveAux = aNewNodesNb > aNodesNb;
        if (isRemoveAux)
        {
            NCollection_DynamicArray<int> aAuxVertices = new(aNewNodesNb - aNodesNb);
            for (int aExtNodesIt = aNodesNb + 1; aExtNodesIt <= aNewNodesNb; ++aExtNodesIt)
            {
                aAuxVertices.Append(aExtNodesIt);
            }

            // Set aux vertices if there are some to clean up mesh correctly.
            aMesher.SetAuxVertices(aAuxVertices);
        }

        aMesher.ProcessConstraints();

        // Destruction of triangles containing aux vertices added (possibly) during base mesh computation.
        if (isRemoveAux)
        {
            aMesher.RemoveAuxElements();
        }

        BRepMesh_MeshTool aCleaner = new(aStructure);
        aCleaner.EraseFreeLinks();

        postProcessMesh(aMesher, theRange);
    }

    /// <summary>Builds the base triangulation with the custom triangulation algorithm.</summary>
    protected abstract void buildBaseTriangulation();
}
