// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelaunayNodeInsertionMeshAlgo.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Meshing.DegreeOfFreedom;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Extends the node-insertion algorithm with the surface nodes of the range splitter, registered before the base mesh
/// is generated or inserted into it afterwards.
/// </summary>
/// <remarks>
/// The Watson instantiation of OCCT's <c>BRepMesh_DelaunayNodeInsertionMeshAlgo&lt;RangeSplitter, BaseAlgo&gt;</c>;
/// <see cref="DelaunayNodeInsertionMeshAlgoOfDelabella{RangeSplitter}"/> is the Delabella one (phase4-plan
/// section 6).
/// </remarks>
internal class DelaunayNodeInsertionMeshAlgo<RangeSplitter> : NodeInsertionMeshAlgo<RangeSplitter>
    where RangeSplitter : DefaultRangeSplitter, new()
{
    private bool _isPreProcessSurfaceNodes;

    /// <summary>An algorithm.</summary>
    public DelaunayNodeInsertionMeshAlgo()
    {
        _isPreProcessSurfaceNodes = false;
    }

    /// <summary>True if the surface nodes are registered before the base mesh is generated.</summary>
    public bool IsPreProcessSurfaceNodes => _isPreProcessSurfaceNodes;

    /// <summary>
    /// Registers the surface nodes before the base mesh is generated if true, or inserts them afterwards if false.
    /// </summary>
    public void SetPreProcessSurfaceNodes(bool isPreProcessSurfaceNodes)
    {
        _isPreProcessSurfaceNodes = isPreProcessSurfaceNodes;
    }

    /// <summary>Initializes the data structure from the model's data.</summary>
    protected override bool InitDataStructure()
    {
        if (!base.InitDataStructure())
        {
            return false;
        }

        if (_isPreProcessSurfaceNodes)
        {
            NCollection_List<Point2D>? surfaceNodes =
                GetRangeSplitter().GenerateSurfaceNodes(GetParameters());

            RegisterSurfaceNodes(surfaceNodes);
        }

        return true;
    }

    /// <summary>The numbers of cells of the circles' acceleration grid along U and V.</summary>
    protected override (int first, int second) GetCellsCount(int verticesNb)
    {
        return GeomTool.CellsCount(GetDFace().GetSurface(),
                                            verticesNb,
                                            GetDFace().GetDeflection(),
                                            GetRangeSplitter());
    }

    /// <summary>Processes the generated mesh: generates the surface nodes and inserts them.</summary>
    protected override void PostProcessMesh(Delaun mesher, ProgressRange range)
    {
        if (!range.More())
        {
            return;
        }
        base.PostProcessMesh(mesher, new ProgressRange()); // shouldn't be range passed here?

        if (!_isPreProcessSurfaceNodes)
        {
            NCollection_List<Point2D>? surfaceNodes =
                GetRangeSplitter().GenerateSurfaceNodes(GetParameters());

            InsertNodes(surfaceNodes, mesher, range);
        }
    }

    /// <summary>Inserts the nodes inside the face into the mesh; true if any was.</summary>
    protected bool InsertNodes(NCollection_List<Point2D>? nodes,
                               Delaun mesher,
                               ProgressRange range)
    {
        if (nodes is null || nodes.IsEmpty())
        {
            return false;
        }

        List<int> vertexIndexes = new(nodes.Size());
        NCollection_List<Point2D>.Iterator nodesIt = new(nodes);
        for (int nodeIt = 1; nodesIt.More(); nodesIt.Next(), ++nodeIt)
        {
            Point2D pnt2d = nodesIt.Value();
            if (GetClassifier().Perform(pnt2d) == In)
            {
                vertexIndexes.Add(
                    RegisterNode(GetRangeSplitter().Point(pnt2d), pnt2d, DegreeOfFreedom.Free, false));
            }
        }

        mesher.AddVertices(vertexIndexes, range);
        if (!range.More())
        {
            return false;
        }
        return vertexIndexes.Count != 0;
    }

    // Registers surface nodes in data structure.
    private bool RegisterSurfaceNodes(NCollection_List<Point2D>? nodes)
    {
        if (nodes is null || nodes.IsEmpty())
        {
            return false;
        }

        bool isAdded = false;
        NCollection_List<Point2D>.Iterator nodesIt = new(nodes);
        for (int nodeIt = 1; nodesIt.More(); nodesIt.Next(), ++nodeIt)
        {
            Point2D pnt2d = nodesIt.Value();
            if (GetClassifier().Perform(pnt2d) == In)
            {
                isAdded = true;
                RegisterNode(GetRangeSplitter().Point(pnt2d), pnt2d, DegreeOfFreedom.Free, false);
            }
        }

        return isAdded;
    }
}
