// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_BaseMeshAlgo.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_BaseMeshAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Meshing;

/// <summary>
/// The base of the face meshing algorithms: registers the boundary of a discrete face in a Delaunay data structure,
/// lets a subclass generate the mesh, and stores the result in the face as its triangulation.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21), used for one face at a time. The structure, the 3D nodes and the map of used nodes
/// live only during <see cref="Perform"/>.
/// </para>
/// <para>
/// The triangulation's triangles are in the order of the structure's <c>ElementsOfDomain()</c>, which iterates as
/// OCCT's packed map does; the triangulation is attached to the face through BRepMesh_ShapeTool's non-freezing attach
/// (decision 488). A failure (<see cref="ModelAlgo.IsCaughtFailure"/>) leaves the face without a mesh, as
/// OCCT's <c>catch (Standard_Failure)</c>.
/// </para>
/// </remarks>
internal abstract class BaseMeshAlgo : MeshAlgo
{
    private FaceModel? _dFace;
    private ToolParameters _parameters = new();
    private DataStructureOfDelaun? _structure;
    private List<Point3D>? _nodesMap;
    private Dictionary<int, int>? _usedNodes;

    /// <summary>An algorithm.</summary>
    protected BaseMeshAlgo()
    {
    }

    /// <summary>Meshes the face.</summary>
    public override void Perform(FaceModel dFace,
                                 in ToolParameters parameters,
                                 ProgressRange range)
    {
        // Kerf: an edge found the face's surface out of all proportion to it, as a surface damaged to 1e300 is: refused at
        // once, as a face out of proportion to its mesh is, before its nodes are made (decision 26602)
        if (dFace.EdgesOffSurface)
        {
            dFace.SetStatus(StatusModel.Failure);
            parameters.Budget?.RefuseFace();
            return;
        }

        try
        {
            _dFace = dFace;
            _parameters = parameters;
            _structure = new DataStructureOfDelaun(storage: MeshWorkStorage.Rent()) { InProportion = true };
            if (parameters.Budget is { } budget)
            {
                _structure.MaxCircleEntries = budget.MaxCircleEntries;
            }

            _nodesMap = new List<Point3D>();
            _usedNodes = new Dictionary<int, int>();

            if (InitDataStructure())
            {
                if (!range.More())
                {
                    return;
                }
                GenerateMesh(range);
                CommitSurfaceTriangulation();
            }
        }
        catch (MeshFaceRefusedException)
        {
            // Kerf: the face, not the mesh, fails, and a public mesh says so (decision 1942)
            dFace.SetStatus(StatusModel.Failure);
            parameters.Budget?.RefuseFace();
        }
        catch (Exception exception) when (ModelAlgo.IsCaughtFailure(exception))
        {
        }
        catch (InsufficientMemoryException) when (parameters.Budget is not null)
        {
            parameters.Budget.Refuse();
        }

        _dFace = null; // Do not hold link to face.
        _structure?.ReleaseStorage();
        _structure = null;
        _nodesMap = null;
        _usedNodes = null;
    }

    /// <summary>The discrete face.</summary>
    protected FaceModel GetDFace() => _dFace!;

    /// <summary>The meshing parameters.</summary>
    protected ToolParameters GetParameters() => _parameters;

    /// <summary>The mesh structure.</summary>
    protected DataStructureOfDelaun GetStructure() => _structure!;

    /// <summary>The 3D nodes, indexed by the structure's nodes' <c>Location3d</c>.</summary>
    protected List<Point3D> GetNodesMap() => _nodesMap!;

    /// <summary>
    /// Registers the point in the 3D nodes and adds the 2D point to the mesh structure; returns the node's index in
    /// the structure.
    /// </summary>
    protected virtual int RegisterNode(in Point3D point,
                                       in Point2D point2d,
                                       DegreeOfFreedom movability,
                                       bool isForceAdd)
    {
        int nodeIndex =
            AddNodeToStructure(point2d, _nodesMap!.Count, movability, isForceAdd);

        if (nodeIndex > _nodesMap.Count)
        {
            _nodesMap.Add(point);
            // Pre-bind frontier and fixed nodes with identity mapping so that
            // pcurve indices (set in initDataStructure as raw structure indices)
            // remain valid as compact Poly_Triangulation indices. Free (internal)
            // nodes are NOT pre-bound: they enter myUsedNodes only via
            // collectTriangles() when actually referenced by a triangle, which
            // prevents unreferenced internal nodes from appearing as free nodes.
            if (movability != DegreeOfFreedom.Free)
            {
                _usedNodes![nodeIndex] = nodeIndex;
            }
        }

        return nodeIndex;
    }

    /// <summary>Adds the 2D point to the mesh structure; returns the node's index in the structure.</summary>
    protected virtual int AddNodeToStructure(in Point2D point,
                                             int location3d,
                                             DegreeOfFreedom movability,
                                             bool isForceAdd)
    {
        MeshVertex node = new(point.XY(), location3d, movability);
        return _structure!.AddNode(node, isForceAdd);
    }

    /// <summary>The 2D point of the vertex.</summary>
    protected virtual Point2D GetNodePoint2d(in MeshVertex vertex) => new(vertex.Coord());

    /// <summary>Initializes the data structure from the model's data: the wires' nodes and links.</summary>
    protected virtual bool InitDataStructure()
    {
        FaceModel dFace = _dFace!;
        for (int wireIt = 0; wireIt < dFace.WiresNb(); ++wireIt)
        {
            WireModel dWire = dFace.GetWire(wireIt);
            if (dWire.IsSet(StatusModel.SelfIntersectingWire))
            {
                // TODO: here we can add points of self-intersecting wire as fixed points
                // in order to keep consistency of nodes with adjacent faces.
                continue;
            }

            for (int edgeIt = 0; edgeIt < dWire.EdgesNb(); ++edgeIt)
            {
                EdgeModel dEdge = dWire.GetEdge(edgeIt);
                CurveModel curve = dEdge.GetCurve();
                // A wire defines one topological occurrence of an edge. Registering every
                // pcurve attached to its face would create duplicate constraints at a periodic seam.
                PCurveModel pCurve = dEdge.GetPCurve(dFace, dWire.GetEdgeOrientation(edgeIt));
                Orientation ori = FixSeamEdgeOrientation(dEdge, pCurve);

                int prevNodeIndex = -1;
                int lastPoint = pCurve.ParametersNb() - 1;
                for (int pointIndex = 0; pointIndex <= lastPoint; ++pointIndex)
                {
                    int nodeIndex = RegisterNode(curve.GetPoint(pointIndex),
                                                  pCurve.GetPoint(pointIndex),
                                                  DegreeOfFreedom.Frontier,
                                                  false);

                    pCurve.GetIndex(pointIndex) = nodeIndex;

                    if (prevNodeIndex != -1 && prevNodeIndex != nodeIndex)
                    {
                        int linksNb = _structure!.LinkCount;
                        int linkIndex = AddLinkToMesh(prevNodeIndex, nodeIndex, ori);
                        if (wireIt != 0 && linkIndex <= linksNb)
                        {
                            // Prevent holes around wire of zero area.
                            _structure.ChangeLink(linkIndex).SetMovability(DegreeOfFreedom.Fixed);
                        }
                    }

                    prevNodeIndex = nodeIndex;
                }

                // Every pcurve needs node indices for Poly_PolygonOnTriangulation, but only
                // the pcurve belonging to this wire occurrence defines a mesh constraint.
                NCollection_List<int> listOfPCurves = dEdge.GetPCurves(dFace);
                for (NCollection_List<int>.Iterator pCurveIt = new(listOfPCurves); pCurveIt.More(); pCurveIt.Next())
                {
                    PCurveModel otherPCurve = dEdge.GetPCurve(pCurveIt.Value());
                    if (ReferenceEquals(otherPCurve, pCurve))
                    {
                        continue;
                    }

                    int otherLastPoint = otherPCurve.ParametersNb() - 1;
                    for (int pointIndex = 0; pointIndex <= otherLastPoint; ++pointIndex)
                    {
                        int nodeIndex = RegisterNode(curve.GetPoint(pointIndex),
                                                      otherPCurve.GetPoint(pointIndex),
                                                      DegreeOfFreedom.Frontier,
                                                      false);
                        otherPCurve.GetIndex(pointIndex) = nodeIndex;
                    }
                }
            }
        }

        return true;
    }

    /// <summary>Generates the mesh of the contour stored in the data structure.</summary>
    protected abstract void GenerateMesh(ProgressRange range);

    // If the given edge has another pcurve for current face coinciding with specified one,
    // returns TopAbs_INTERNAL flag. Elsewhere returns orientation of specified pcurve.
    private Orientation FixSeamEdgeOrientation(EdgeModel dEdge, PCurveModel pCurve)
    {
        for (int pCurveIt = 0; pCurveIt < dEdge.PCurvesNb(); ++pCurveIt)
        {
            PCurveModel aPCurve = dEdge.GetPCurve(pCurveIt);
            if (ReferenceEquals(aPCurve.GetFace(), _dFace) && !ReferenceEquals(pCurve, aPCurve))
            {
                // Simple check that another pcurve of seam edge does not coincide with reference one.
                Point2D pnt1_1 = pCurve.GetPoint(0);
                Point2D pnt2_1 = pCurve.GetPoint(pCurve.ParametersNb() - 1);

                Point2D pnt1_2 = aPCurve.GetPoint(0);
                Point2D pnt2_2 = aPCurve.GetPoint(aPCurve.ParametersNb() - 1);

                double sqDist1 =
                    Min(pnt1_1.SquareDistanceTo(pnt1_2), pnt1_1.SquareDistanceTo(pnt2_2));
                double sqDist2 =
                    Min(pnt2_1.SquareDistanceTo(pnt1_2), pnt2_1.SquareDistanceTo(pnt2_2));
                if (sqDist1 < global::Kerf.Engine.Foundation.Precision.SquareConfusion() && sqDist2 < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    return Internal;
                }
            }
        }

        return pCurve.GetOrientation();
    }

    // Adds new link to the mesh data structure.
    // Movability of the link and order of nodes depend on orientation parameter.
    private int AddLinkToMesh(int firstNodeId, int lastNodeId, Orientation orientation)
    {
        int linkIndex;
        if (orientation == Reversed)
        {
            linkIndex =
                _structure!.AddLink(new MeshEdge(lastNodeId, firstNodeId, DegreeOfFreedom.Frontier));
        }
        else if (orientation == Internal)
        {
            linkIndex = _structure!.AddLink(new MeshEdge(firstNodeId, lastNodeId, DegreeOfFreedom.Fixed));
        }
        else
        {
            linkIndex =
                _structure!.AddLink(new MeshEdge(firstNodeId, lastNodeId, DegreeOfFreedom.Frontier));
        }

        return Math.Abs(linkIndex);
    }

    // Commits generated triangulation to TopoDS face.
    private void CommitSurfaceTriangulation()
    {
        Triangulation? triangulation = CollectTriangles();
        if (triangulation is null)
        {
            _dFace!.SetStatus(StatusModel.Failure);
            return;
        }

        CollectNodes(triangulation);

        _parameters.Budget?.Commit(triangulation.TriangleCount);
        ShapeTool.AddInFace(_dFace!, triangulation);
    }

    // Collects triangles to output data.
    private Triangulation? CollectTriangles()
    {
        NCollection_PackedMapOfInteger triangles = _structure!.ElementsOfDomain();
        if (triangles.IsEmpty())
        {
            return null;
        }

        Triangulation res = new();
        res.ResizeTriangles(triangles.Extent(), false);
        Span<int> node = stackalloc int[3];
        NCollection_PackedMapOfInteger.Iterator triIt = new(triangles);
        for (int triangeId = 1; triIt.More(); triIt.Next(), ++triangeId)
        {
            Triangle curElem = _structure.GetElement(triIt.Key());

            _structure.ElementNodes(curElem, node);

            for (int i = 0; i < 3; ++i)
            {
                if (!_usedNodes!.ContainsKey(node[i]))
                {
                    _usedNodes[node[i]] = _usedNodes.Count + 1;
                }

                node[i] = _usedNodes.GetValue(node[i]);
            }

            res.SetTriangle(triangeId, new Triangle1(node[0], node[1], node[2]));
        }
        res.ResizeNodes(_usedNodes!.Count, false);
        res.AddUVNodes();
        return res;
    }

    // Collects nodes to output data.
    private void CollectNodes(Triangulation triangulation)
    {
        for (int i = 1; i <= _nodesMap!.Count; ++i)
        {
            if (_usedNodes!.ContainsKey(i))
            {
                MeshVertex vertex = _structure!.GetNode(i);

                int nodeIndex = _usedNodes.GetValue(i);
                triangulation.SetNode(nodeIndex, _nodesMap[vertex.Location3d()]);
                triangulation.SetUVNode(nodeIndex, GetNodePoint2d(vertex));
            }
        }
    }
}
