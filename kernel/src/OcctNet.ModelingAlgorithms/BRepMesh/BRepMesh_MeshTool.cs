// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_MeshTool.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_MeshTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepMesh_DegreeOfFreedom;

namespace Occt;

/// <summary>
/// Auxiliary operations on a <see cref="BRepMesh_DataStructureOfDelaun"/>: adding triangles and legalizing them by
/// Delaunay's criterion, and erasing triangles and free links.
/// </summary>
/// <remarks>A working object, as OCCT's: it writes into the structure it is given.</remarks>
public sealed class BRepMesh_MeshTool : Standard_Transient
{
    /// <summary>Classifies nodes against the line of a constraint link.</summary>
    public sealed class NodeClassifier
    {
        private readonly BRepMesh_DataStructureOfDelaun myStructure;
        private gp_Lin2d myConstraint;
        private readonly bool mySign;

        /// <summary>A classifier against the line of the link, oriented from its first node to its last.</summary>
        public NodeClassifier(in BRepMesh_Edge theConstraint, BRepMesh_DataStructureOfDelaun theStructure)
        {
            myStructure = theStructure;
            myConstraint = new gp_Lin2d();
            BRepMesh_Vertex aVertex1 = myStructure.GetNode(theConstraint.FirstNode());
            BRepMesh_Vertex aVertex2 = myStructure.GetNode(theConstraint.LastNode());

            myConstraint.SetLocation(new gp_Pnt2d(aVertex1.Coord()));
            myConstraint.SetDirection(new gp_Dir2d(new gp_Vec2d(new gp_Pnt2d(aVertex1.Coord()), new gp_Pnt2d(aVertex2.Coord()))));
            mySign = myConstraint.Direction().X() > 0;
        }

        /// <summary>Whether the node is strictly above the line (in the sense of its direction's X).</summary>
        public bool IsAbove(int theNodeIndex)
        {
            BRepMesh_Vertex aVertex = myStructure.GetNode(theNodeIndex);
            gp_Vec2d aNodeVec = new(myConstraint.Location(), new gp_Pnt2d(aVertex.Coord()));
            if (aNodeVec.SquareMagnitude() > gp.Resolution())
            {
                double aCross = aNodeVec.Crossed(new gp_Vec2d(myConstraint.Direction()));
                if (Math.Abs(aCross) > gp.Resolution())
                {
                    return mySign ? aCross < 0.0 : aCross > 0.0;
                }
            }

            return false;
        }
    }

    private readonly BRepMesh_DataStructureOfDelaun myStructure;

    /// <summary>A tool over the given structure.</summary>
    public BRepMesh_MeshTool(BRepMesh_DataStructureOfDelaun theStructure) => myStructure = theStructure;

    /// <summary>The structure.</summary>
    public BRepMesh_DataStructureOfDelaun GetStructure() => myStructure;

    /// <summary>
    /// Writes the triangles of the domain (only those in <paramref name="theTriangles"/>, if given) as planar faces of a
    /// compound to a BRep file.
    /// </summary>
    public void DumpTriangles(string theFileName, NCollection_PackedMapOfInteger? theTriangles)
    {
        BRep_Builder aBuilder = new();
        TopoDS_Compound aResult = new();
        aBuilder.MakeCompound(ref aResult);

        NCollection_PackedMapOfInteger aTriangles = myStructure.ElementsOfDomain();
        Span<int> aNodes = stackalloc int[3];
        for (NCollection_PackedMapOfInteger.Iterator aIt = new(aTriangles); aIt.More(); aIt.Next())
        {
            if (theTriangles is not null && !theTriangles.Contains(aIt.Key()))
            {
                continue;
            }

            BRepMesh_Triangle aTri = myStructure.GetElement(aIt.Key());
            myStructure.ElementNodes(aTri, aNodes);

            gp_XY aV1 = myStructure.GetNode(aNodes[0]).Coord();
            gp_XY aV2 = myStructure.GetNode(aNodes[1]).Coord();
            gp_XY aV3 = myStructure.GetNode(aNodes[2]).Coord();

            BRepBuilderAPI_MakePolygon aPoly = new(new gp_Pnt(aV1.X(), aV1.Y(), 0.0),
                                                   new gp_Pnt(aV2.X(), aV2.Y(), 0.0),
                                                   new gp_Pnt(aV3.X(), aV3.Y(), 0.0),
                                                   true);

            BRepBuilderAPI_MakeFace aFaceBuilder = new(new gp_Pln(new gp_Ax3(gp.XOY())), aPoly.Wire());
            aBuilder.Add(aResult, aFaceBuilder.Shape());
        }

        BRepTools.Write(aResult, theFileName);
    }

    /// <summary>Adds the triangle of the three nodes and legalizes its three links.</summary>
    public void AddAndLegalizeTriangle(int thePoint1, int thePoint2, int thePoint3)
    {
        Span<int> aEdges = stackalloc int[3];
        AddTriangle(thePoint1, thePoint2, thePoint3, aEdges);

        Legalize(aEdges[0]);
        Legalize(aEdges[1]);
        Legalize(aEdges[2]);
    }

    /// <summary>Adds the triangle of the three nodes (and its links); returns its link indices in a span of three.</summary>
    public void AddTriangle(int thePoint1, int thePoint2, int thePoint3, Span<int> theEdges)
    {
        Span<bool> aOri = stackalloc bool[3];
        AddLink(thePoint1, thePoint2, out theEdges[0], out aOri[0]);
        AddLink(thePoint2, thePoint3, out theEdges[1], out aOri[1]);
        AddLink(thePoint3, thePoint1, out theEdges[2], out aOri[2]);

        myStructure.AddElement(new BRepMesh_Triangle(theEdges, aOri, BRepMesh_Free));
    }

    /// <summary>Adds the free link of the two nodes; returns its index and whether it has this orientation.</summary>
    public void AddLink(int theFirstNode, int theLastNode, out int theLinkIndex, out bool theLinkOri)
    {
        int aLinkIt = myStructure.AddLink(new BRepMesh_Edge(theFirstNode, theLastNode, BRepMesh_Free));

        theLinkIndex = Math.Abs(aLinkIt);
        theLinkOri = (aLinkIt > 0);
    }

    /// <summary>Flips the link and the links around it, recursively, until they satisfy Delaunay's criterion.</summary>
    public void Legalize(int theLinkIndex)
    {
        Stack<int> aStack = new();
        aStack.Push(theLinkIndex);

        NCollection_PackedMapOfInteger aUsedLinks = new();
        Span<int> aNodes0 = stackalloc int[3];
        Span<int> aNodes1 = stackalloc int[3];
        while (aStack.Count != 0)
        {
            int aLinkIndex = aStack.Pop();

            aUsedLinks.Add(aLinkIndex);
            BRepMesh_Edge aLink = myStructure.GetLink(aLinkIndex);
            if (aLink.Movability() != BRepMesh_Frontier)
            {
                BRepMesh_PairOfIndex aPair = myStructure.ElementsConnectedTo(aLinkIndex);
                if (aPair.Extent() == 2)
                {
                    BRepMesh_Triangle aTriangle1 = myStructure.GetElement(aPair.FirstIndex());
                    BRepMesh_Triangle aTriangle2 = myStructure.GetElement(aPair.LastIndex());

                    myStructure.ElementNodes(aTriangle1, aNodes0);
                    myStructure.ElementNodes(aTriangle2, aNodes1);

                    int aApexIndex0 = findApexIndex(aNodes0, aLink);
                    int aApexIndex1 = findApexIndex(aNodes1, aLink);

                    if (checkCircle(aNodes0, aNodes1[aApexIndex1])
                        || checkCircle(aNodes1, aNodes0[aApexIndex0]))
                    {
                        myStructure.RemoveElement(aPair.FirstIndex());
                        myStructure.RemoveElement(aPair.LastIndex());
                        myStructure.RemoveLink(aLinkIndex);

                        addTriangleAndUpdateStack(aNodes0[aApexIndex0],
                                                  aNodes0[(aApexIndex0 + 1) % 3],
                                                  aNodes1[aApexIndex1],
                                                  aUsedLinks,
                                                  aStack);

                        addTriangleAndUpdateStack(aNodes1[aApexIndex1],
                                                  aNodes1[(aApexIndex1 + 1) % 3],
                                                  aNodes0[aApexIndex0],
                                                  aUsedLinks,
                                                  aStack);
                    }
                }
            }
        }
    }

    /// <summary>Erases the triangles around the node, the free links they leave, and the node (if free).</summary>
    public void EraseItemsConnectedTo(int theNodeIndex)
    {
        BRepMesh_SelectorOfDataStructureOfDelaun aSelector = new(myStructure);
        aSelector.NeighboursOfNode(theNodeIndex);

        NCollection_IntDataMap<bool> aLoopEdges = new(1);
        EraseTriangles(aSelector.Elements(), aLoopEdges);
        EraseFreeLinks(aLoopEdges);
        myStructure.RemoveNode(theNodeIndex);
    }

    /// <summary>
    /// Erases the triangles outside the frontier links (on their right) and the triangles reached from them through free
    /// links around the frontier's nodes, and the free links they leave.
    /// </summary>
    public void CleanFrontierLinks()
    {
        NCollection_PackedMapOfInteger aTrianglesToErase = new();
        NCollection_IntDataMap<bool> aLoopEdges = new(1);

        NCollection_PackedMapOfInteger aFrontier = GetEdgesByType(BRepMesh_Frontier);
        NCollection_PackedMapOfInteger.Iterator aFrontierIt = new(aFrontier);
        for (; aFrontierIt.More(); aFrontierIt.Next())
        {
            int aFrontierId = aFrontierIt.Key();
            BRepMesh_Edge aLink = myStructure.GetLink(aFrontierId);

            bool isTriangleFound = false;
            BRepMesh_PairOfIndex aPair = myStructure.ElementsConnectedTo(aFrontierId);
            for (int aElemIt = 1; aElemIt <= aPair.Extent() && !isTriangleFound; ++aElemIt)
            {
                int aPriorElemId = aPair.Index(aElemIt);
                BRepMesh_Triangle aElement = myStructure.GetElement(aPriorElemId);
                BRepMesh_Triangle.Int3 e = aElement.myEdges;
                BRepMesh_Triangle.Bool3 o = aElement.myOrientations;

                for (int n = 0; n < 3 && !isTriangleFound; ++n)
                {
                    if (aFrontierId == e[n] && !o[n])
                    {
                        // Destruction of external triangles on boundary edges
                        isTriangleFound = true;
                        aTrianglesToErase.Add(aPriorElemId);

                        collectTrianglesOnFreeLinksAroundNodesOf(aLink, e[(n + 1) % 3], aTrianglesToErase);
                        collectTrianglesOnFreeLinksAroundNodesOf(aLink, e[(n + 2) % 3], aTrianglesToErase);
                    }
                }
            }
        }

        EraseTriangles(aTrianglesToErase, aLoopEdges);
        EraseFreeLinks(aLoopEdges);
    }

    /// <summary>Erases the triangles; their links go into <paramref name="theLoopEdges"/> (see <see cref="EraseTriangle"/>).</summary>
    public void EraseTriangles(NCollection_PackedMapOfInteger theTriangles, NCollection_IntDataMap<bool> theLoopEdges)
    {
        NCollection_PackedMapOfInteger.Iterator aFreeTriangles = new(theTriangles);
        for (; aFreeTriangles.More(); aFreeTriangles.Next())
        {
            EraseTriangle(aFreeTriangles.Key(), theLoopEdges);
        }
    }

    /// <summary>
    /// Erases the triangle. Each of its links is added to <paramref name="theLoopEdges"/> with its orientation; a link
    /// already there (shared with another erased triangle) is taken out and removed.
    /// </summary>
    public void EraseTriangle(int theTriangleIndex, NCollection_IntDataMap<bool> theLoopEdges)
    {
        BRepMesh_Triangle aElement = myStructure.GetElement(theTriangleIndex);
        BRepMesh_Triangle.Int3 e = aElement.myEdges;
        BRepMesh_Triangle.Bool3 o = aElement.myOrientations;

        myStructure.RemoveElement(theTriangleIndex);

        for (int i = 0; i < 3; ++i)
        {
            if (!theLoopEdges.Bind(e[i], o[i]))
            {
                theLoopEdges.UnBind(e[i]);
                myStructure.RemoveLink(e[i]);
            }
        }
    }

    /// <summary>Removes every link without triangles (frontier and fixed ones included, made free first).</summary>
    public void EraseFreeLinks()
    {
        for (int i = 1; i <= myStructure.NbLinks(); i++)
        {
            if (myStructure.ElementsConnectedTo(i).IsEmpty())
            {
                ref BRepMesh_Edge anEdge = ref myStructure.ChangeLink(i);
                if (anEdge.Movability() == BRepMesh_Deleted)
                {
                    continue;
                }

                anEdge.SetMovability(BRepMesh_Free);
                myStructure.RemoveLink(i);
            }
        }
    }

    /// <summary>Removes the free links of the map that have no triangles.</summary>
    public void EraseFreeLinks(NCollection_IntDataMap<bool> theLinks)
    {
        NCollection_IntDataMap<bool>.Iterator aFreeEdges = new(theLinks);
        for (; aFreeEdges.More(); aFreeEdges.Next())
        {
            if (myStructure.ElementsConnectedTo(aFreeEdges.Key()).IsEmpty())
            {
                myStructure.RemoveLink(aFreeEdges.Key());
            }
        }
    }

    /// <summary>The links of the domain with the given movability.</summary>
    public NCollection_PackedMapOfInteger GetEdgesByType(BRepMesh_DegreeOfFreedom theEdgeType)
    {
        NCollection_PackedMapOfInteger aResult = new();
        NCollection_PackedMapOfInteger.Iterator aEdgeIt = new(myStructure.LinksOfDomain());

        for (; aEdgeIt.More(); aEdgeIt.Next())
        {
            BRepMesh_Edge aEdge = myStructure.GetLink(aEdgeIt.Key());
            if (aEdge.Movability() == theEdgeType)
            {
                aResult.Add(aEdgeIt.Key());
            }
        }

        return aResult;
    }

    // Returns index of triangle node opposite to the given link.
    private static int findApexIndex(ReadOnlySpan<int> aNodes, in BRepMesh_Edge theLink)
    {
        int i = 0;
        for (; i < 3; ++i)
        {
            if (aNodes[i] != theLink.FirstNode() && aNodes[i] != theLink.LastNode())
            {
                break;
            }
        }

        return i;
    }

    private bool checkCircle(ReadOnlySpan<int> aNodes, int thePoint)
    {
        BRepMesh_Vertex aVertex0 = myStructure.GetNode(aNodes[0]);
        BRepMesh_Vertex aVertex1 = myStructure.GetNode(aNodes[1]);
        BRepMesh_Vertex aVertex2 = myStructure.GetNode(aNodes[2]);

        bool isOk = BRepMesh_CircleTool.MakeCircle(aVertex0.Coord(),
                                                   aVertex1.Coord(),
                                                   aVertex2.Coord(),
                                                   out gp_XY aLocation,
                                                   out double aRadius);

        if (isOk)
        {
            BRepMesh_Vertex aVertex = myStructure.GetNode(thePoint);
            double aDist = (aVertex.Coord() - aLocation).SquareModulus() - (aRadius * aRadius);
            return (aDist < Precision.SquareConfusion());
        }

        return false;
    }

    private void addTriangleAndUpdateStack(int theNode0,
                                           int theNode1,
                                           int theNode2,
                                           NCollection_PackedMapOfInteger theUsedLinks,
                                           Stack<int> theStack)
    {
        Span<int> aEdges = stackalloc int[3];
        AddTriangle(theNode0, theNode1, theNode2, aEdges);

        for (int i = 0; i < 3; ++i)
        {
            if (!theUsedLinks.Contains(aEdges[i]))
            {
                theStack.Push(aEdges[i]);
            }
        }
    }

    private void collectTrianglesOnFreeLinksAroundNodesOf(in BRepMesh_Edge theConstraint,
                                                          int theStartLink,
                                                          NCollection_PackedMapOfInteger theTriangles)
    {
        NCollection_PackedMapOfInteger aUsedLinks = new();
        Stack<int> aStack = new();
        aStack.Push(theStartLink);
        aUsedLinks.Add(theStartLink);

        while (aStack.Count != 0)
        {
            int aLinkIndex = aStack.Pop();

            BRepMesh_Edge aLink = myStructure.GetLink(aLinkIndex);
            if (aLink.Movability() == BRepMesh_Free
                && (aLink.FirstNode() == theConstraint.FirstNode()
                    || aLink.LastNode() == theConstraint.FirstNode()
                    || aLink.FirstNode() == theConstraint.LastNode()
                    || aLink.LastNode() == theConstraint.LastNode()))
            {
                BRepMesh_PairOfIndex aPair = myStructure.ElementsConnectedTo(aLinkIndex);
                for (int aElemIt = 1; aElemIt <= aPair.Extent(); ++aElemIt)
                {
                    int aIndex = aPair.Index(aElemIt);
                    theTriangles.Add(aIndex);

                    BRepMesh_Triangle aElement = myStructure.GetElement(aIndex);
                    BRepMesh_Triangle.Int3 aEdges = aElement.myEdges;

                    for (int i = 0; i < 3; ++i)
                    {
                        if (aEdges[i] != aLinkIndex && !aUsedLinks.Contains(aEdges[i]))
                        {
                            aUsedLinks.Add(aEdges[i]);
                            aStack.Push(aEdges[i]);
                        }
                    }
                }
            }
        }
    }
}
