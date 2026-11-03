// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_BaseMeshAlgo.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_BaseMeshAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using static Occt.BRepMesh_DegreeOfFreedom;
using static Occt.IMeshData_Status;
using static Occt.TopAbs_Orientation;

namespace Occt;

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
/// (decision 488). A failure (<see cref="IMeshTools_ModelAlgo.IsCaughtFailure"/>) leaves the face without a mesh, as
/// OCCT's <c>catch (Standard_Failure)</c>.
/// </para>
/// </remarks>
public abstract class BRepMesh_BaseMeshAlgo : IMeshTools_MeshAlgo
{
    private IMeshData_Face? myDFace;
    private IMeshTools_Parameters myParameters = new();
    private BRepMesh_DataStructureOfDelaun? myStructure;
    private NCollection_DynamicArray<Point3D>? myNodesMap;
    private NCollection_DataMap<int, int>? myUsedNodes;

    /// <summary>An algorithm.</summary>
    protected BRepMesh_BaseMeshAlgo()
    {
    }

    /// <summary>Meshes the face.</summary>
    public override void Perform(IMeshData_Face theDFace,
                                 in IMeshTools_Parameters theParameters,
                                 ProgressRange theRange)
    {
        try
        {
            myDFace = theDFace;
            myParameters = theParameters;
            myStructure = new BRepMesh_DataStructureOfDelaun();
            myNodesMap = new NCollection_DynamicArray<Point3D>(256);
            myUsedNodes = new NCollection_DataMap<int, int>();

            if (initDataStructure())
            {
                if (!theRange.More())
                {
                    return;
                }
                generateMesh(theRange);
                commitSurfaceTriangulation();
            }
        }
        catch (Exception anException) when (IMeshTools_ModelAlgo.IsCaughtFailure(anException))
        {
        }

        myDFace = null; // Do not hold link to face.
        myStructure = null;
        myNodesMap = null;
        myUsedNodes = null;
    }

    /// <summary>The discrete face.</summary>
    protected IMeshData_Face getDFace() => myDFace!;

    /// <summary>The meshing parameters.</summary>
    protected IMeshTools_Parameters getParameters() => myParameters;

    /// <summary>The mesh structure.</summary>
    protected BRepMesh_DataStructureOfDelaun getStructure() => myStructure!;

    /// <summary>The 3D nodes, indexed by the structure's nodes' <c>Location3d</c>.</summary>
    protected NCollection_DynamicArray<Point3D> getNodesMap() => myNodesMap!;

    /// <summary>
    /// Registers the point in the 3D nodes and adds the 2D point to the mesh structure; returns the node's index in
    /// the structure.
    /// </summary>
    protected virtual int registerNode(in Point3D thePoint,
                                       in Point2D thePoint2d,
                                       BRepMesh_DegreeOfFreedom theMovability,
                                       bool isForceAdd)
    {
        int aNodeIndex =
            addNodeToStructure(thePoint2d, myNodesMap!.Length(), theMovability, isForceAdd);

        if (aNodeIndex > myNodesMap.Length())
        {
            myNodesMap.Append(thePoint);
            // Pre-bind frontier and fixed nodes with identity mapping so that
            // pcurve indices (set in initDataStructure as raw structure indices)
            // remain valid as compact Poly_Triangulation indices. Free (internal)
            // nodes are NOT pre-bound: they enter myUsedNodes only via
            // collectTriangles() when actually referenced by a triangle, which
            // prevents unreferenced internal nodes from appearing as free nodes.
            if (theMovability != BRepMesh_Free)
            {
                myUsedNodes!.Bind(aNodeIndex, aNodeIndex);
            }
        }

        return aNodeIndex;
    }

    /// <summary>Adds the 2D point to the mesh structure; returns the node's index in the structure.</summary>
    protected virtual int addNodeToStructure(in Point2D thePoint,
                                             int theLocation3d,
                                             BRepMesh_DegreeOfFreedom theMovability,
                                             bool isForceAdd)
    {
        BRepMesh_Vertex aNode = new(thePoint.XY(), theLocation3d, theMovability);
        return myStructure!.AddNode(aNode, isForceAdd);
    }

    /// <summary>The 2D point of the vertex.</summary>
    protected virtual Point2D getNodePoint2d(in BRepMesh_Vertex theVertex) => new(theVertex.Coord());

    /// <summary>Initializes the data structure from the model's data: the wires' nodes and links.</summary>
    protected virtual bool initDataStructure()
    {
        IMeshData_Face aDFace = myDFace!;
        for (int aWireIt = 0; aWireIt < aDFace.WiresNb(); ++aWireIt)
        {
            IMeshData_Wire aDWire = aDFace.GetWire(aWireIt);
            if (aDWire.IsSet(IMeshData_SelfIntersectingWire))
            {
                // TODO: here we can add points of self-intersecting wire as fixed points
                // in order to keep consistency of nodes with adjacent faces.
                continue;
            }

            for (int aEdgeIt = 0; aEdgeIt < aDWire.EdgesNb(); ++aEdgeIt)
            {
                IMeshData_Edge aDEdge = aDWire.GetEdge(aEdgeIt);
                IMeshData_Curve aCurve = aDEdge.GetCurve();
                // A wire defines one topological occurrence of an edge. Registering every
                // pcurve attached to its face would create duplicate constraints at a periodic seam.
                IMeshData_PCurve aPCurve = aDEdge.GetPCurve(aDFace, aDWire.GetEdgeOrientation(aEdgeIt));
                TopAbs_Orientation aOri = fixSeamEdgeOrientation(aDEdge, aPCurve);

                int aPrevNodeIndex = -1;
                int aLastPoint = aPCurve.ParametersNb() - 1;
                for (int aPointIndex = 0; aPointIndex <= aLastPoint; ++aPointIndex)
                {
                    int aNodeIndex = registerNode(aCurve.GetPoint(aPointIndex),
                                                  aPCurve.GetPoint(aPointIndex),
                                                  BRepMesh_Frontier,
                                                  false);

                    aPCurve.GetIndex(aPointIndex) = aNodeIndex;

                    if (aPrevNodeIndex != -1 && aPrevNodeIndex != aNodeIndex)
                    {
                        int aLinksNb = myStructure!.NbLinks();
                        int aLinkIndex = addLinkToMesh(aPrevNodeIndex, aNodeIndex, aOri);
                        if (aWireIt != 0 && aLinkIndex <= aLinksNb)
                        {
                            // Prevent holes around wire of zero area.
                            myStructure.ChangeLink(aLinkIndex).SetMovability(BRepMesh_Fixed);
                        }
                    }

                    aPrevNodeIndex = aNodeIndex;
                }

                // Every pcurve needs node indices for Poly_PolygonOnTriangulation, but only
                // the pcurve belonging to this wire occurrence defines a mesh constraint.
                NCollection_List<int> aListOfPCurves = aDEdge.GetPCurves(aDFace);
                for (NCollection_List<int>.Iterator aPCurveIt = new(aListOfPCurves); aPCurveIt.More(); aPCurveIt.Next())
                {
                    IMeshData_PCurve anOtherPCurve = aDEdge.GetPCurve(aPCurveIt.Value());
                    if (ReferenceEquals(anOtherPCurve, aPCurve))
                    {
                        continue;
                    }

                    int anOtherLastPoint = anOtherPCurve.ParametersNb() - 1;
                    for (int aPointIndex = 0; aPointIndex <= anOtherLastPoint; ++aPointIndex)
                    {
                        int aNodeIndex = registerNode(aCurve.GetPoint(aPointIndex),
                                                      anOtherPCurve.GetPoint(aPointIndex),
                                                      BRepMesh_Frontier,
                                                      false);
                        anOtherPCurve.GetIndex(aPointIndex) = aNodeIndex;
                    }
                }
            }
        }

        return true;
    }

    /// <summary>Generates the mesh of the contour stored in the data structure.</summary>
    protected abstract void generateMesh(ProgressRange theRange);

    // If the given edge has another pcurve for current face coinciding with specified one,
    // returns TopAbs_INTERNAL flag. Elsewhere returns orientation of specified pcurve.
    private TopAbs_Orientation fixSeamEdgeOrientation(IMeshData_Edge theDEdge, IMeshData_PCurve thePCurve)
    {
        for (int aPCurveIt = 0; aPCurveIt < theDEdge.PCurvesNb(); ++aPCurveIt)
        {
            IMeshData_PCurve aPCurve = theDEdge.GetPCurve(aPCurveIt);
            if (ReferenceEquals(aPCurve.GetFace(), myDFace) && !ReferenceEquals(thePCurve, aPCurve))
            {
                // Simple check that another pcurve of seam edge does not coincide with reference one.
                Point2D aPnt1_1 = thePCurve.GetPoint(0);
                Point2D aPnt2_1 = thePCurve.GetPoint(thePCurve.ParametersNb() - 1);

                Point2D aPnt1_2 = aPCurve.GetPoint(0);
                Point2D aPnt2_2 = aPCurve.GetPoint(aPCurve.ParametersNb() - 1);

                double aSqDist1 =
                    Min(aPnt1_1.SquareDistance(aPnt1_2), aPnt1_1.SquareDistance(aPnt2_2));
                double aSqDist2 =
                    Min(aPnt2_1.SquareDistance(aPnt1_2), aPnt2_1.SquareDistance(aPnt2_2));
                if (aSqDist1 < global::Kerf.Engine.Foundation.Precision.SquareConfusion() && aSqDist2 < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    return TopAbs_INTERNAL;
                }
            }
        }

        return thePCurve.GetOrientation();
    }

    // Adds new link to the mesh data structure.
    // Movability of the link and order of nodes depend on orientation parameter.
    private int addLinkToMesh(int theFirstNodeId, int theLastNodeId, TopAbs_Orientation theOrientation)
    {
        int aLinkIndex;
        if (theOrientation == TopAbs_REVERSED)
        {
            aLinkIndex =
                myStructure!.AddLink(new BRepMesh_Edge(theLastNodeId, theFirstNodeId, BRepMesh_Frontier));
        }
        else if (theOrientation == TopAbs_INTERNAL)
        {
            aLinkIndex = myStructure!.AddLink(new BRepMesh_Edge(theFirstNodeId, theLastNodeId, BRepMesh_Fixed));
        }
        else
        {
            aLinkIndex =
                myStructure!.AddLink(new BRepMesh_Edge(theFirstNodeId, theLastNodeId, BRepMesh_Frontier));
        }

        return Math.Abs(aLinkIndex);
    }

    // Commits generated triangulation to TopoDS face.
    private void commitSurfaceTriangulation()
    {
        Triangulation? aTriangulation = collectTriangles();
        if (aTriangulation is null)
        {
            myDFace!.SetStatus(IMeshData_Failure);
            return;
        }

        collectNodes(aTriangulation);

        BRepMesh_ShapeTool.AddInFace(myDFace!.GetFace(), aTriangulation);
    }

    // Collects triangles to output data.
    private Triangulation? collectTriangles()
    {
        NCollection_PackedMapOfInteger aTriangles = myStructure!.ElementsOfDomain();
        if (aTriangles.IsEmpty())
        {
            return null;
        }

        Triangulation aRes = new();
        aRes.ResizeTriangles(aTriangles.Extent(), false);
        Span<int> aNode = stackalloc int[3];
        NCollection_PackedMapOfInteger.Iterator aTriIt = new(aTriangles);
        for (int aTriangeId = 1; aTriIt.More(); aTriIt.Next(), ++aTriangeId)
        {
            BRepMesh_Triangle aCurElem = myStructure.GetElement(aTriIt.Key());

            myStructure.ElementNodes(aCurElem, aNode);

            for (int i = 0; i < 3; ++i)
            {
                if (!myUsedNodes!.IsBound(aNode[i]))
                {
                    myUsedNodes.Bind(aNode[i], myUsedNodes.Extent() + 1);
                }

                aNode[i] = myUsedNodes.Find(aNode[i]);
            }

            aRes.SetTriangle(aTriangeId, new Triangle1(aNode[0], aNode[1], aNode[2]));
        }
        aRes.ResizeNodes(myUsedNodes!.Extent(), false);
        aRes.AddUVNodes();
        return aRes;
    }

    // Collects nodes to output data.
    private void collectNodes(Triangulation theTriangulation)
    {
        for (int i = 1; i <= myNodesMap!.Length(); ++i)
        {
            if (myUsedNodes!.IsBound(i))
            {
                BRepMesh_Vertex aVertex = myStructure!.GetNode(i);

                int aNodeIndex = myUsedNodes.Find(i);
                theTriangulation.SetNode(aNodeIndex, myNodesMap.Value(aVertex.Location3d()));
                theTriangulation.SetUVNode(aNodeIndex, getNodePoint2d(aVertex));
            }
        }
    }
}
