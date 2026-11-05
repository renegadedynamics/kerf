// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_CustomDelaunayBaseMeshAlgo.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Meshing;

/// <summary>
/// OCCT's <c>BRepMesh_CustomDelaunayBaseMeshAlgo&lt;BRepMesh_DelabellaBaseMeshAlgo&gt;</c>: the Delabella base
/// algorithm, whose generated mesh can be modified afterwards (the circles of its triangles are registered first).
/// </summary>
/// <remarks>
/// OCCT's template derives from its parameter, which C# can't; OCCT instantiates it only over
/// BRepMesh_DelabellaBaseMeshAlgo, so this is that instantiation, named as OCCT names <c>.gxx</c> instantiations
/// (decision 202). It is the base of the Delabella set of the node-insertion algorithms
/// (<see cref="NodeInsertionMeshAlgoOfDelabella{RangeSplitter}"/> and the two below it).
/// </remarks>
internal class CustomDelaunayBaseMeshAlgoOfDelabella : DelabellaBaseMeshAlgo
{
    /// <summary>An algorithm.</summary>
    public CustomDelaunayBaseMeshAlgoOfDelabella()
    {
    }

    /// <summary>Processes the generated mesh: fills the mesher's circles, then the base's processing.</summary>
    protected override void PostProcessMesh(Delaun mesher, ProgressRange range)
    {
        DataStructureOfDelaun structure = GetStructure();
        (int first, int second) cellsCount = GetCellsCount(structure.NodeCount);
        mesher.InitCirclesTool(cellsCount.first, cellsCount.second);

        base.PostProcessMesh(mesher, range);
    }
}
