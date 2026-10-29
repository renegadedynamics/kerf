// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelaunayNodeInsertionMeshAlgo.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepMesh_DegreeOfFreedom;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Extends the node-insertion algorithm with the surface nodes of the range splitter, registered before the base mesh
/// is generated or inserted into it afterwards.
/// </summary>
/// <remarks>
/// The Watson instantiation of OCCT's <c>BRepMesh_DelaunayNodeInsertionMeshAlgo&lt;RangeSplitter, BaseAlgo&gt;</c>;
/// <see cref="BRepMesh_DelaunayNodeInsertionMeshAlgoOfDelabella{RangeSplitter}"/> is the Delabella one (phase4-plan
/// section 6).
/// </remarks>
public class BRepMesh_DelaunayNodeInsertionMeshAlgo<RangeSplitter> : BRepMesh_NodeInsertionMeshAlgo<RangeSplitter>
    where RangeSplitter : BRepMesh_DefaultRangeSplitter, new()
{
    private bool myIsPreProcessSurfaceNodes;

    /// <summary>An algorithm.</summary>
    public BRepMesh_DelaunayNodeInsertionMeshAlgo()
    {
        myIsPreProcessSurfaceNodes = false;
    }

    /// <summary>True if the surface nodes are registered before the base mesh is generated.</summary>
    public bool IsPreProcessSurfaceNodes() => myIsPreProcessSurfaceNodes;

    /// <summary>
    /// Registers the surface nodes before the base mesh is generated if true, or inserts them afterwards if false.
    /// </summary>
    public void SetPreProcessSurfaceNodes(bool isPreProcessSurfaceNodes)
    {
        myIsPreProcessSurfaceNodes = isPreProcessSurfaceNodes;
    }

    /// <summary>Initializes the data structure from the model's data.</summary>
    protected override bool initDataStructure()
    {
        if (!base.initDataStructure())
        {
            return false;
        }

        if (myIsPreProcessSurfaceNodes)
        {
            NCollection_List<gp_Pnt2d>? aSurfaceNodes =
                getRangeSplitter().GenerateSurfaceNodes(getParameters());

            registerSurfaceNodes(aSurfaceNodes);
        }

        return true;
    }

    /// <summary>The numbers of cells of the circles' acceleration grid along U and V.</summary>
    protected override (int first, int second) getCellsCount(int theVerticesNb)
    {
        return BRepMesh_GeomTool.CellsCount(getDFace().GetSurface(),
                                            theVerticesNb,
                                            getDFace().GetDeflection(),
                                            getRangeSplitter());
    }

    /// <summary>Processes the generated mesh: generates the surface nodes and inserts them.</summary>
    protected override void postProcessMesh(BRepMesh_Delaun theMesher, Message_ProgressRange theRange)
    {
        if (!theRange.More())
        {
            return;
        }
        base.postProcessMesh(theMesher, new Message_ProgressRange()); // shouldn't be range passed here?

        if (!myIsPreProcessSurfaceNodes)
        {
            NCollection_List<gp_Pnt2d>? aSurfaceNodes =
                getRangeSplitter().GenerateSurfaceNodes(getParameters());

            insertNodes(aSurfaceNodes, theMesher, theRange);
        }
    }

    /// <summary>Inserts the nodes inside the face into the mesh; true if any was.</summary>
    protected bool insertNodes(NCollection_List<gp_Pnt2d>? theNodes,
                               BRepMesh_Delaun theMesher,
                               Message_ProgressRange theRange)
    {
        if (theNodes is null || theNodes.IsEmpty())
        {
            return false;
        }

        NCollection_DynamicArray<int> aVertexIndexes = new(theNodes.Size());
        NCollection_List<gp_Pnt2d>.Iterator aNodesIt = new(theNodes);
        for (int aNodeIt = 1; aNodesIt.More(); aNodesIt.Next(), ++aNodeIt)
        {
            gp_Pnt2d aPnt2d = aNodesIt.Value();
            if (getClassifier().Perform(aPnt2d) == TopAbs_IN)
            {
                aVertexIndexes.Append(
                    registerNode(getRangeSplitter().Point(aPnt2d), aPnt2d, BRepMesh_Free, false));
            }
        }

        theMesher.AddVertices(aVertexIndexes, theRange);
        if (!theRange.More())
        {
            return false;
        }
        return !aVertexIndexes.IsEmpty();
    }

    // Registers surface nodes in data structure.
    private bool registerSurfaceNodes(NCollection_List<gp_Pnt2d>? theNodes)
    {
        if (theNodes is null || theNodes.IsEmpty())
        {
            return false;
        }

        bool isAdded = false;
        NCollection_List<gp_Pnt2d>.Iterator aNodesIt = new(theNodes);
        for (int aNodeIt = 1; aNodesIt.More(); aNodesIt.Next(), ++aNodeIt)
        {
            gp_Pnt2d aPnt2d = aNodesIt.Value();
            if (getClassifier().Perform(aPnt2d) == TopAbs_IN)
            {
                isAdded = true;
                registerNode(getRangeSplitter().Point(aPnt2d), aPnt2d, BRepMesh_Free, false);
            }
        }

        return isAdded;
    }
}
