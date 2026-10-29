// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Delaun.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Delaun.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepMesh_DegreeOfFreedom;
using static Occt.BRepMesh_GeomTool.IntFlag;

namespace Occt;

/// <summary>
/// Computes the Delaunay triangulation of the nodes of a <see cref="BRepMesh_DataStructureOfDelaun"/> with Watson's
/// algorithm, then inserts its frontier and internal links (constraints) and removes the triangles outside the
/// frontier.
/// </summary>
/// <remarks>
/// A working object, as OCCT's (a class used by value there; the allocators are dropped). It writes into the data
/// structure it is given, and the constructors taking an index vector sort that vector in place, as OCCT's do, with
/// MSVC's heap sort (<see cref="std_heap"/>, decision 491), so ties come out in native OCCT's order.
/// </remarks>
public sealed class BRepMesh_Delaun
{
    private static readonly double AngDeviation1Deg = M_PI / 180.0;
    private static readonly double AngDeviation90Deg = 90 * AngDeviation1Deg;
    private static readonly double Angle2PI = 2 * M_PI;

    private static readonly double Precision = Occt.Precision.PConfusion();
    private static readonly double Precision2 = Precision * Precision;

    private enum ReplaceFlag
    {
        Replace,
        InsertAfter,
        InsertBefore,
    }

    private BRepMesh_DataStructureOfDelaun? myMeshData;
    private readonly BRepMesh_CircleTool myCircles;
    private readonly NCollection_DynamicArray<int> mySupVert;
    private bool myInitCircles;
    private BRepMesh_Triangle mySupTrian;

    /// <summary>
    /// A triangulator of an existing structure, which doesn't run: the circles' cells are filled from the structure's
    /// triangles if <paramref name="isFillCircles"/>.
    /// </summary>
    public BRepMesh_Delaun(BRepMesh_DataStructureOfDelaun theOldMesh, int theCellsCountU, int theCellsCountV, bool isFillCircles)
    {
        myMeshData = theOldMesh;
        myCircles = new BRepMesh_CircleTool();
        mySupVert = new NCollection_DynamicArray<int>(3);
        myInitCircles = false;
        if (isFillCircles)
        {
            InitCirclesTool(theCellsCountU, theCellsCountV);
        }
    }

    /// <summary>Triangulates the vertices in a new structure (none if there are fewer than three).</summary>
    public BRepMesh_Delaun(NCollection_Array1<BRepMesh_Vertex> theVertices)
    {
        myCircles = new BRepMesh_CircleTool(theVertices.Length());
        mySupVert = new NCollection_DynamicArray<int>(3);
        myInitCircles = false;
        if (theVertices.Length() > 2)
        {
            myMeshData = new BRepMesh_DataStructureOfDelaun(theVertices.Length());
            Init(theVertices);
        }
    }

    /// <summary>Adds the vertices to the structure and triangulates them (if there are at least three).</summary>
    public BRepMesh_Delaun(BRepMesh_DataStructureOfDelaun theOldMesh, NCollection_Array1<BRepMesh_Vertex> theVertices)
    {
        myMeshData = theOldMesh;
        myCircles = new BRepMesh_CircleTool(theVertices.Length());
        mySupVert = new NCollection_DynamicArray<int>(3);
        myInitCircles = false;
        if (theVertices.Length() > 2)
        {
            Init(theVertices);
        }
    }

    /// <summary>Triangulates the structure's nodes of the given indices; sorts <paramref name="theVertexIndices"/>.</summary>
    public BRepMesh_Delaun(BRepMesh_DataStructureOfDelaun theOldMesh, NCollection_DynamicArray<int> theVertexIndices)
    {
        myMeshData = theOldMesh;
        myCircles = new BRepMesh_CircleTool(theVertexIndices.Length());
        mySupVert = new NCollection_DynamicArray<int>(3);
        myInitCircles = false;
        perform(theVertexIndices);
    }

    /// <summary>
    /// Triangulates the structure's nodes of the given indices with the given numbers of cells for the circles;
    /// sorts <paramref name="theVertexIndices"/>.
    /// </summary>
    public BRepMesh_Delaun(BRepMesh_DataStructureOfDelaun theOldMesh,
                           NCollection_DynamicArray<int> theVertexIndices,
                           int theCellsCountU,
                           int theCellsCountV)
    {
        myMeshData = theOldMesh;
        myCircles = new BRepMesh_CircleTool(theVertexIndices.Length());
        mySupVert = new NCollection_DynamicArray<int>(3);
        myInitCircles = false;
        perform(theVertexIndices, theCellsCountU, theCellsCountV);
    }

    /// <summary>Adds the vertices to the structure and triangulates them.</summary>
    public void Init(NCollection_Array1<BRepMesh_Vertex> theVertices)
    {
        int aLowerIdx = theVertices.Lower();
        int anUpperIdx = theVertices.Upper();
        NCollection_DynamicArray<int> aVertexIndexes = new(theVertices.Size());

        int anIndex = aLowerIdx;
        for (; anIndex <= anUpperIdx; ++anIndex)
        {
            aVertexIndexes.Append(myMeshData!.AddNode(theVertices[anIndex]));
        }

        perform(aVertexIndexes);
    }

    /// <summary>Sets up the circles' cells from the structure's nodes and fills them with its triangles.</summary>
    public void InitCirclesTool(int theCellsCountU, int theCellsCountV)
    {
        Bnd_Box2d aBox = new();
        for (int aNodeIt = 1; aNodeIt <= myMeshData!.NbNodes(); ++aNodeIt)
        {
            aBox.Add(new gp_Pnt2d(GetVertex(aNodeIt).Coord()));
        }

        aBox.Enlarge(Precision);

        initCirclesTool(aBox, theCellsCountU, theCellsCountV);

        NCollection_PackedMapOfInteger.Iterator aTriangleIt = new(myMeshData.ElementsOfDomain());
        Span<int> aNodesIndices = stackalloc int[3];
        for (; aTriangleIt.More(); aTriangleIt.Next())
        {
            BRepMesh_Triangle aTriangle = myMeshData.GetElement(aTriangleIt.Key());
            myMeshData.ElementNodes(aTriangle, aNodesIndices);
            myCircles.Bind(aTriangleIt.Key(),
                           GetVertex(aNodesIndices[0]).Coord(),
                           GetVertex(aNodesIndices[1]).Coord(),
                           GetVertex(aNodesIndices[2]).Coord());
        }
    }

    /// <summary>Removes the node coinciding with the vertex and remeshes the hole it leaves.</summary>
    public void RemoveVertex(in BRepMesh_Vertex theVertex)
    {
        BRepMesh_SelectorOfDataStructureOfDelaun aSelector = new(myMeshData!);
        aSelector.NeighboursOf(theVertex);

        NCollection_IntDataMap<bool> aLoopEdges = new();

        // Loop on triangles to be destroyed :
        NCollection_PackedMapOfInteger.Iterator aTriangleIt = new(aSelector.Elements());
        for (; aTriangleIt.More(); aTriangleIt.Next())
        {
            deleteTriangle(aTriangleIt.Key(), aLoopEdges);
        }

        NCollection_Sequence<Bnd_B2d> aBoxes = new();
        NCollection_Sequence<int> aPolygon = new();
        int aLoopEdgesCount = aLoopEdges.Extent();
        NCollection_IntDataMap<bool>.Iterator aLoopEdgesIt = new(aLoopEdges);

        if (aLoopEdgesIt.More())
        {
            BRepMesh_Edge anEdge = GetEdge(aLoopEdgesIt.Key());
            int aFirstNode = anEdge.FirstNode();
            int aLastNode;
            int aPivotNode = anEdge.LastNode();
            int anEdgeId = aLoopEdgesIt.Key();

            bool isPositive = aLoopEdges[anEdgeId];
            if (!isPositive)
            {
                int aTmp;
                aTmp = aFirstNode;
                aFirstNode = aPivotNode;
                aPivotNode = aTmp;

                aPolygon.Append(-anEdgeId);
            }
            else
            {
                aPolygon.Append(anEdgeId);
            }

            fillBndBox(aBoxes, GetVertex(aFirstNode), GetVertex(aPivotNode));

            aLoopEdges.UnBind(anEdgeId);

            aLastNode = aFirstNode;
            while (aPivotNode != aLastNode)
            {
                BRepMesh_DataStructureOfDelaun.LinkedListOfInteger.Iterator aLinkIt = new(myMeshData!.LinksConnectedTo(aPivotNode));
                for (; aLinkIt.More(); aLinkIt.Next())
                {
                    if (aLinkIt.Value() != anEdgeId && aLoopEdges.IsBound(aLinkIt.Value()))
                    {
                        int aCurrentNode;
                        anEdgeId = aLinkIt.Value();
                        BRepMesh_Edge anEdge1 = GetEdge(anEdgeId);

                        aCurrentNode = anEdge1.LastNode();
                        if (aCurrentNode != aPivotNode)
                        {
                            aCurrentNode = anEdge1.FirstNode();
                            aPolygon.Append(-anEdgeId);
                        }
                        else
                        {
                            aPolygon.Append(anEdgeId);
                        }

                        fillBndBox(aBoxes, GetVertex(aCurrentNode), GetVertex(aPivotNode));

                        aPivotNode = aCurrentNode;
                        aLoopEdges.UnBind(anEdgeId);
                        break;
                    }
                }

                if (aLoopEdgesCount <= 0)
                {
                    break;
                }

                --aLoopEdgesCount;
            }

            meshPolygon(aPolygon, aBoxes);
        }
    }

    /// <summary>Inserts the structure's nodes of the given indices into the triangulation; sorts the vector.</summary>
    public void AddVertices(NCollection_DynamicArray<int> theVertices, Message_ProgressRange theRange = default)
    {
        ComparatorOfIndexedVertexOfDelaun aCmp = new(myMeshData!);
        std_heap.make_heap(theVertices.AsSpan(), aCmp);
        std_heap.sort_heap(theVertices.AsSpan(), aCmp);

        createTrianglesOnNewVertices(theVertices, theRange);
    }

    /// <summary>Does nothing and returns false (OCCT's body is commented out).</summary>
    public bool UseEdge(int theEdge)
    {
        _ = theEdge;
        return false;
    }

    /// <summary>The data structure.</summary>
    public BRepMesh_DataStructureOfDelaun? Result() => myMeshData;

    /// <summary>Inserts the internal links into the triangulation, then adjusts it to the frontier.</summary>
    public void ProcessConstraints()
    {
        insertInternalEdges();

        // Adjustment of meshes to boundary edges
        frontierAdjust();
    }

    /// <summary>The frontier links.</summary>
    public NCollection_PackedMapOfInteger Frontier() => getEdgesByType(BRepMesh_Frontier);

    /// <summary>The internal (fixed) links.</summary>
    public NCollection_PackedMapOfInteger InternalEdges() => getEdgesByType(BRepMesh_Fixed);

    /// <summary>The links connected to at most one triangle.</summary>
    public NCollection_PackedMapOfInteger FreeEdges() => getEdgesByType(BRepMesh_Free);

    /// <summary>The node of the given index.</summary>
    public BRepMesh_Vertex GetVertex(int theIndex) => myMeshData!.GetNode(theIndex);

    /// <summary>The link of the given index.</summary>
    public BRepMesh_Edge GetEdge(int theIndex) => myMeshData!.GetLink(theIndex);

    /// <summary>The triangle of the given index.</summary>
    public BRepMesh_Triangle GetTriangle(int theIndex) => myMeshData!.GetElement(theIndex);

    /// <summary>The circumcircles of the triangles.</summary>
    public BRepMesh_CircleTool Circles() => myCircles;

    /// <summary>
    /// Whether the triangle contains the vertex. <paramref name="theEdgeOn"/> is the link the vertex lies on (within
    /// <paramref name="theSqTolerance"/>, or near a non-free link), or 0.
    /// </summary>
    public bool Contains(int theTriangleId, in BRepMesh_Vertex theVertex, double theSqTolerance, out int theEdgeOn)
    {
        theEdgeOn = 0;

        Span<int> p = stackalloc int[3];

        BRepMesh_Triangle aElement = GetTriangle(theTriangleId);
        BRepMesh_Triangle.Int3 e = aElement.myEdges;

        Span<BRepMesh_Edge> anEdges = [GetEdge(e[0]), GetEdge(e[1]), GetEdge(e[2])];

        myMeshData!.ElementNodes(aElement, p);

        Span<gp_XY> aPoints = stackalloc gp_XY[3];
        aPoints[0] = GetVertex(p[0]).Coord();
        aPoints[1] = GetVertex(p[1]).Coord();
        aPoints[2] = GetVertex(p[2]).Coord();

        Span<gp_XY> aVEdges = stackalloc gp_XY[3];
        aVEdges[0] = aPoints[1];
        aVEdges[0].Subtract(aPoints[0]);

        aVEdges[1] = aPoints[2];
        aVEdges[1].Subtract(aPoints[1]);

        aVEdges[2] = aPoints[0];
        aVEdges[2].Subtract(aPoints[2]);

        Span<double> aDistance = stackalloc double[3];
        Span<double> aSqModulus = stackalloc double[3];

        double aSqMinDist;
        int aEdgeOnId = 0;
        aSqMinDist = calculateDist(aVEdges, aPoints, theVertex, aDistance, aSqModulus, ref aEdgeOnId);
        if (aSqMinDist < 0)
        {
            return false;
        }

        bool isNotFree = (anEdges[aEdgeOnId].Movability() != BRepMesh_Free);
        if (aSqMinDist > theSqTolerance)
        {
            if (isNotFree && aDistance[aEdgeOnId] < (aSqModulus[aEdgeOnId] / 5.0))
            {
                theEdgeOn = e[aEdgeOnId];
            }
        }
        else if (isNotFree)
        {
            return false;
        }
        else
        {
            theEdgeOn = e[aEdgeOnId];
        }

        return (aDistance[0] >= 0.0 && aDistance[1] >= 0.0 && aDistance[2] >= 0.0);
    }

    /// <summary>Sets the indices of the auxiliary vertices (the super triangle's, or a third-party algorithm's).</summary>
    public void SetAuxVertices(NCollection_DynamicArray<int> theSupVert)
    {
        mySupVert.Assign(theSupVert);
    }

    /// <summary>Removes the triangles that use an auxiliary vertex, the links they leave, and the auxiliary vertices.</summary>
    public void RemoveAuxElements()
    {
        NCollection_IntDataMap<bool> aLoopEdges = new(10);

        // Destruction of triangles containing a top of the super triangle
        BRepMesh_SelectorOfDataStructureOfDelaun aSelector = new(myMeshData!);
        for (int aSupVertId = 0; aSupVertId < mySupVert.Length(); ++aSupVertId)
        {
            aSelector.NeighboursOfNode(mySupVert[aSupVertId]);
        }

        NCollection_PackedMapOfInteger.Iterator aFreeTriangles = new(aSelector.Elements());
        for (; aFreeTriangles.More(); aFreeTriangles.Next())
        {
            deleteTriangle(aFreeTriangles.Key(), aLoopEdges);
        }

        // All edges that remain free are removed from aLoopEdges;
        // only the boundary edges of the triangulation remain there
        NCollection_IntDataMap<bool>.Iterator aFreeEdges = new(aLoopEdges);
        for (; aFreeEdges.More(); aFreeEdges.Next())
        {
            if (myMeshData!.ElementsConnectedTo(aFreeEdges.Key()).IsEmpty())
            {
                myMeshData.RemoveLink(aFreeEdges.Key());
            }
        }

        // The tops of the super triangle are destroyed
        for (int aSupVertId = 0; aSupVertId < mySupVert.Length(); ++aSupVertId)
        {
            myMeshData!.RemoveNode(mySupVert[aSupVertId]);
        }
    }

    //! Sort two points in projection on vector (1,1)
    private readonly struct ComparatorOfIndexedVertexOfDelaun(BRepMesh_DataStructureOfDelaun theDS) : std_heap.ILess<int>
    {
        private readonly BRepMesh_DataStructureOfDelaun myStructure = theDS;

        public bool Less(int theLeft, int theRight)
        {
            gp_XY aLeft = myStructure.GetNode(theLeft).Coord();
            gp_XY aRight = myStructure.GetNode(theRight).Coord();
            return aLeft.X() + aLeft.Y() < aRight.X() + aRight.Y();
        }
    }

    private static void UpdateBndBox(in gp_XY thePnt1, in gp_XY thePnt2, ref Bnd_B2d theBox)
    {
        theBox.Add(thePnt1);
        theBox.Add(thePnt2);
        theBox.Enlarge(Precision);
    }

    // A stack of frames. Each frame is a range of elements to be processed. The stack allows to process elements in
    // depth-first order meaning that when new elements are added to the stack, they will be processed before the
    // remaining elements of the current frame. Frames are processed in LIFO order while elements inside a frame are
    // processed in FIFO order.
    private sealed class StackOfFrames
    {
        // A frame is a range of elements to be processed: [FrameStart, FrameEnd).
        private struct Frame(int theFrameStart, int theFrameEnd)
        {
            private int CurrentElement = theFrameStart; // Index of the current element of the frame.
            private readonly int FrameEnd = theFrameEnd; // Index of the last element + 1 of the frame.

            // Return the index of the current element and advance to the next one.
            public int Advance() => CurrentElement++;

            // Check if all elements in the frame have been processed.
            public readonly bool IsEmpty() => CurrentElement == FrameEnd;
        }

        private readonly NCollection_LinearVector<Frame> myFrames = new(); // Container of frames.

        // Adds a new frame for the given range of elements [theFrameStart, theFrameEnd).
        public void PushFrame(int theFrameStart, int theFrameEnd) => myFrames.Append(new Frame(theFrameStart, theFrameEnd));

        // Returns the index of the current element of the top frame and advances to the next element. If all elements
        // of the top frame have been processed, the frame is removed from the stack. The stack must not be empty.
        public int PopElement()
        {
            ref Frame aFrame = ref myFrames.ChangeLast();
            int anElem = aFrame.Advance();
            if (aFrame.IsEmpty())
            {
                myFrames.EraseLast();
            }

            return anElem;
        }

        // Check if the stack is empty.
        public bool IsEmpty() => myFrames.IsEmpty();
    }

    private void initCirclesTool(in Bnd_Box2d theBox, int theCellsCountU, int theCellsCountV)
    {
        theBox.Get(out double aMinX, out double aMinY, out double aMaxX, out double aMaxY);
        double aDeltaX = aMaxX - aMinX;
        double aDeltaY = aMaxY - aMinY;

        int aScaler = 2;
        if (myMeshData!.NbNodes() > 100)
        {
            aScaler = 5;
        }
        else if (myMeshData.NbNodes() > 1000) // OCCT: unreachable (tested after > 100), so aScaler is never 7
        {
            aScaler = 7;
        }

        myCircles.SetMinMaxSize(new gp_XY(aMinX, aMinY), new gp_XY(aMaxX, aMaxY));
        myCircles.SetCellSize(aDeltaX / Math.Max(theCellsCountU, aScaler),
                              aDeltaY / Math.Max(theCellsCountV, aScaler));

        myInitCircles = true;
    }

    private void fillBndBox(NCollection_Sequence<Bnd_B2d> theBoxes, in BRepMesh_Vertex theV1, in BRepMesh_Vertex theV2)
    {
        Bnd_B2d aBox = new();
        UpdateBndBox(theV1.Coord(), theV2.Coord(), ref aBox);
        theBoxes.Append(aBox);
    }

    private NCollection_PackedMapOfInteger getEdgesByType(BRepMesh_DegreeOfFreedom theEdgeType)
    {
        NCollection_PackedMapOfInteger aResult = new();
        NCollection_PackedMapOfInteger.Iterator anEdgeIt = new(myMeshData!.LinksOfDomain());

        for (; anEdgeIt.More(); anEdgeIt.Next())
        {
            int anEdge = anEdgeIt.Key();
            bool isToAdd = (theEdgeType == BRepMesh_Free)
                               ? (myMeshData.ElementsConnectedTo(anEdge).Extent() <= 1)
                               : (GetEdge(anEdge).Movability() == theEdgeType);

            if (isToAdd)
            {
                aResult.Add(anEdge);
            }
        }

        return aResult;
    }

    private void perform(NCollection_DynamicArray<int> theVertexIndices, int theCellsCountU = -1, int theCellsCountV = -1)
    {
        if (theVertexIndices.Length() <= 2)
        {
            return;
        }

        Bnd_Box2d aBox = new();
        int anIndex = theVertexIndices.Lower();
        int anUpper = theVertexIndices.Upper();
        for (; anIndex <= anUpper; ++anIndex)
        {
            aBox.Add(new gp_Pnt2d(GetVertex(theVertexIndices[anIndex]).Coord()));
        }

        aBox.Enlarge(Precision);

        initCirclesTool(aBox, theCellsCountU, theCellsCountV);
        superMesh(aBox);

        ComparatorOfIndexedVertexOfDelaun aCmp = new(myMeshData!);
        std_heap.make_heap(theVertexIndices.AsSpan(), aCmp);
        std_heap.sort_heap(theVertexIndices.AsSpan(), aCmp);

        compute(theVertexIndices);
    }

    private void superMesh(in Bnd_Box2d theBox)
    {
        theBox.Get(out double aMinX, out double aMinY, out double aMaxX, out double aMaxY);
        double aDeltaX = aMaxX - aMinX;
        double aDeltaY = aMaxY - aMinY;

        double aDeltaMin = Min(aDeltaX, aDeltaY);
        double aDeltaMax = Max(aDeltaX, aDeltaY);
        double aDelta = aDeltaX + aDeltaY;

        mySupVert.Append(
            myMeshData!.AddNode(new BRepMesh_Vertex((aMinX + aMaxX) / 2, aMaxY + aDeltaMax, BRepMesh_Free)));

        mySupVert.Append(
            myMeshData.AddNode(new BRepMesh_Vertex(aMinX - aDelta, aMinY - aDeltaMin, BRepMesh_Free)));

        mySupVert.Append(
            myMeshData.AddNode(new BRepMesh_Vertex(aMaxX + aDelta, aMinY - aDeltaMin, BRepMesh_Free)));

        Span<int> e = stackalloc int[3];
        Span<bool> o = stackalloc bool[3];
        for (int aNodeId = 0; aNodeId < 3; ++aNodeId)
        {
            int aFirstNode = aNodeId;
            int aLastNode = (aNodeId + 1) % 3;
            int aLinkIndex = myMeshData.AddLink(
                new BRepMesh_Edge(mySupVert[aFirstNode], mySupVert[aLastNode], BRepMesh_Free));

            e[aNodeId] = Math.Abs(aLinkIndex);
            o[aNodeId] = (aLinkIndex > 0);
        }

        mySupTrian = new BRepMesh_Triangle(e, o, BRepMesh_Free);
    }

    private void deleteTriangle(int theIndex, NCollection_IntDataMap<bool> theLoopEdges)
    {
        if (myInitCircles)
        {
            myCircles.Delete(theIndex);
        }

        BRepMesh_Triangle aElement = GetTriangle(theIndex);
        BRepMesh_Triangle.Int3 e = aElement.myEdges;
        BRepMesh_Triangle.Bool3 o = aElement.myOrientations;

        myMeshData!.RemoveElement(theIndex);

        for (int i = 0; i < 3; ++i)
        {
            if (!theLoopEdges.Bind(e[i], o[i]))
            {
                theLoopEdges.UnBind(e[i]);
                myMeshData.RemoveLink(e[i]);
            }
        }
    }

    private void compute(NCollection_DynamicArray<int> theVertexIndexes)
    {
        // Insertion of edges of super triangles in the list of free edges:
        NCollection_IntDataMap<bool> aLoopEdges = new(10);
        BRepMesh_Triangle.Int3 e = mySupTrian.myEdges;

        aLoopEdges.Bind(e[0], true);
        aLoopEdges.Bind(e[1], true);
        aLoopEdges.Bind(e[2], true);

        if (theVertexIndexes.Length() > 0)
        {
            // Creation of 3 trianglers with the first node and the edges of the super triangle:
            int anVertexIdx = theVertexIndexes.Lower();
            createTriangles(theVertexIndexes[anVertexIdx], aLoopEdges);

            // Add other nodes to the mesh
            createTrianglesOnNewVertices(theVertexIndexes, default);
        }

        RemoveAuxElements();
    }

    private void createTriangles(int theVertexIndex, NCollection_IntDataMap<bool> thePoly)
    {
        NCollection_List<int> aLoopEdges = new();
        NCollection_List<int> anExternalEdges = new();
        gp_XY aVertexCoord = myMeshData!.GetNode(theVertexIndex).Coord();

        Span<int> aNodes = stackalloc int[3];
        Span<int> anEdgesInfo = stackalloc int[3];
        Span<int> anEdgeIds = stackalloc int[3];
        Span<bool> anEdgesOri = stackalloc bool[3];
        NCollection_IntDataMap<bool>.Iterator anEdges = new(thePoly);
        for (; anEdges.More(); anEdges.Next())
        {
            int anEdgeId = anEdges.Key();
            BRepMesh_Edge anEdge = GetEdge(anEdgeId);

            bool isPositive = thePoly[anEdgeId];

            if (isPositive)
            {
                aNodes[0] = anEdge.FirstNode();
                aNodes[2] = anEdge.LastNode();
            }
            else
            {
                aNodes[0] = anEdge.LastNode();
                aNodes[2] = anEdge.FirstNode();
            }

            aNodes[1] = theVertexIndex;

            BRepMesh_Vertex aFirstVertex = GetVertex(aNodes[0]);
            BRepMesh_Vertex aLastVertex = GetVertex(aNodes[2]);

            gp_XY anEdgeDir = aLastVertex.Coord() - aFirstVertex.Coord();
            double anEdgeLen = anEdgeDir.Modulus();
            if (anEdgeLen < Precision)
            {
                continue;
            }

            anEdgeDir.SetCoord(anEdgeDir.X() / anEdgeLen, anEdgeDir.Y() / anEdgeLen);

            gp_XY aFirstLinkDir = aFirstVertex.Coord() - aVertexCoord;
            gp_XY aLastLinkDir = aVertexCoord - aLastVertex.Coord();

            double aDist12 = aFirstLinkDir ^ anEdgeDir;
            double aDist23 = anEdgeDir ^ aLastLinkDir;
            if (Math.Abs(aDist12) < Precision || Math.Abs(aDist23) < Precision)
            {
                continue;
            }

            BRepMesh_Edge aFirstLink = new(aNodes[1], aNodes[0], BRepMesh_Free);
            BRepMesh_Edge aLastLink = new(aNodes[2], aNodes[1], BRepMesh_Free);

            anEdgesInfo[0] = myMeshData.AddLink(aFirstLink);
            anEdgesInfo[1] = isPositive ? anEdgeId : -anEdgeId;
            anEdgesInfo[2] = myMeshData.AddLink(aLastLink);

            bool isSensOK = (aDist12 > 0.0 && aDist23 > 0.0);
            if (isSensOK)
            {
                for (int aTriLinkIt = 0; aTriLinkIt < 3; ++aTriLinkIt)
                {
                    int anEdgeInfo = anEdgesInfo[aTriLinkIt];
                    anEdgeIds[aTriLinkIt] = Math.Abs(anEdgeInfo);
                    anEdgesOri[aTriLinkIt] = anEdgeInfo > 0;
                }

                addTriangle(anEdgeIds, anEdgesOri, aNodes);
            }
            else
            {
                if (isPositive)
                {
                    aLoopEdges.Append(anEdges.Key());
                }
                else
                {
                    aLoopEdges.Append(-anEdges.Key());
                }

                if (aFirstLinkDir.SquareModulus() > aLastLinkDir.SquareModulus())
                {
                    anExternalEdges.Append(Math.Abs(anEdgesInfo[0]));
                }
                else
                {
                    anExternalEdges.Append(Math.Abs(anEdgesInfo[2]));
                }
            }
        }

        thePoly.Clear();
        while (!anExternalEdges.IsEmpty())
        {
            ref readonly BRepMesh_PairOfIndex aPair =
                ref myMeshData.ElementsConnectedTo(Math.Abs(anExternalEdges.First()));

            if (!aPair.IsEmpty())
            {
                deleteTriangle(aPair.FirstIndex(), thePoly);
            }

            anExternalEdges.RemoveFirst();
        }

        for (anEdges.Initialize(thePoly); anEdges.More(); anEdges.Next())
        {
            if (myMeshData.ElementsConnectedTo(anEdges.Key()).IsEmpty())
            {
                myMeshData.RemoveLink(anEdges.Key());
            }
        }

        while (!aLoopEdges.IsEmpty())
        {
            BRepMesh_Edge anEdge = GetEdge(Math.Abs(aLoopEdges.First()));
            if (anEdge.Movability() != BRepMesh_Deleted)
            {
                int anEdgeIdx = aLoopEdges.First();
                meshLeftPolygonOf(Math.Abs(anEdgeIdx), (anEdgeIdx > 0));
            }

            aLoopEdges.RemoveFirst();
        }
    }

    private void createTrianglesOnNewVertices(NCollection_DynamicArray<int> theVertexIndexes, Message_ProgressRange theRange)
    {
        myMeshData!.Data().GetTolerance(out double aTolU, out double aTolV);
        double aSqTol = aTolU * aTolU + aTolV * aTolV;

        // Insertion of nodes :
        bool isModify = true;

        int anIndex = theVertexIndexes.Lower();
        int anUpper = theVertexIndexes.Upper();
        using Message_ProgressScope aPS = new(theRange, "Create triangles on new vertices", anUpper);
        for (; anIndex <= anUpper; ++anIndex, aPS.Next())
        {
            if (!aPS.More())
            {
                return;
            }

            NCollection_IntDataMap<bool> aLoopEdges = new(10);

            int aVertexIdx = theVertexIndexes[anIndex];
            BRepMesh_Vertex aVertex = GetVertex(aVertexIdx);

            // Iterator in the list of indexes of circles containing the node
            NCollection_List<int> aCirclesList = myCircles.Select(aVertex.Coord());

            int onEgdeId = 0, aTriangleId = 0;
            NCollection_List<int>.Iterator aCircleIt = new(aCirclesList);
            for (; aCircleIt.More(); aCircleIt.Next())
            {
                // To add a node in the mesh it is necessary to check conditions:
                // - the node should be within the boundaries of the mesh and so in an existing triangle
                // - all adjacent triangles should belong to a component connected with this triangle
                if (Contains(aCircleIt.Value(), aVertex, aSqTol, out onEgdeId))
                {
                    if (onEgdeId != 0 && GetEdge(onEgdeId).Movability() != BRepMesh_Free)
                    {
                        // We can skip free vertex too close to the frontier edge.
                        if (aVertex.Movability() == BRepMesh_Free)
                        {
                            continue;
                        }

                        // However, we should add vertex that have neighboring frontier edges.
                    }

                    // Remove triangle even if it contains frontier edge in order
                    // to prevent appearance of incorrect configurations like free
                    // edge glued with frontier #26407
                    aTriangleId = aCircleIt.Value();
                    aCirclesList.Remove(aCircleIt);
                    break;
                }
            }

            if (aTriangleId > 0)
            {
                deleteTriangle(aTriangleId, aLoopEdges);

                isModify = true;
                while (isModify && !aCirclesList.IsEmpty())
                {
                    isModify = false;
                    NCollection_List<int>.Iterator aCircleIt1 = new(aCirclesList);
                    for (; aCircleIt1.More(); aCircleIt1.Next())
                    {
                        BRepMesh_Triangle aElement = GetTriangle(aCircleIt1.Value());
                        BRepMesh_Triangle.Int3 e = aElement.myEdges;

                        if (aLoopEdges.IsBound(e[0]) || aLoopEdges.IsBound(e[1]) || aLoopEdges.IsBound(e[2]))
                        {
                            isModify = true;
                            deleteTriangle(aCircleIt1.Value(), aLoopEdges);
                            aCirclesList.Remove(aCircleIt1);
                            break;
                        }
                    }
                }

                // Creation of triangles with the current node and free edges
                // and removal of these edges from the list of free edges
                createTriangles(aVertexIdx, aLoopEdges);
            }
        }

        ProcessConstraints();
    }

    private void insertInternalEdges()
    {
        NCollection_PackedMapOfInteger anInternalEdges = InternalEdges();

        // Destruction of triangles intersecting internal edges
        // and their replacement by makeshift triangles
        NCollection_PackedMapOfInteger.Iterator anInernalEdgesIt = new(anInternalEdges);
        Span<bool> isGo = stackalloc bool[2];
        for (; anInernalEdgesIt.More(); anInernalEdgesIt.Next())
        {
            int aLinkIndex = anInernalEdgesIt.Key();
            BRepMesh_PairOfIndex aPair = myMeshData!.ElementsConnectedTo(aLinkIndex);

            // Check both sides of link for adjusted triangle.
            isGo[0] = true;
            isGo[1] = true;
            for (int aTriangleIt = 1; aTriangleIt <= aPair.Extent(); ++aTriangleIt)
            {
                BRepMesh_Triangle aElement = GetTriangle(aPair.Index(aTriangleIt));
                BRepMesh_Triangle.Int3 e = aElement.myEdges;
                BRepMesh_Triangle.Bool3 o = aElement.myOrientations;

                for (int i = 0; i < 3; ++i)
                {
                    if (e[i] == aLinkIndex)
                    {
                        isGo[o[i] ? 0 : 1] = false;
                        break;
                    }
                }
            }

            if (isGo[0])
            {
                meshLeftPolygonOf(aLinkIndex, true);
            }

            if (isGo[1])
            {
                meshLeftPolygonOf(aLinkIndex, false);
            }
        }
    }

    private bool isBoundToFrontier(int theRefNodeId, int theRefLinkId)
    {
        Stack<int> aLinkStack = new();
        NCollection_PackedMapOfInteger aVisitedLinks = new();

        aLinkStack.Push(theRefLinkId);
        while (aLinkStack.Count != 0)
        {
            int aCurrentLinkId = aLinkStack.Pop();

            BRepMesh_PairOfIndex aPair = myMeshData!.ElementsConnectedTo(aCurrentLinkId);
            if (aPair.IsEmpty())
            {
                return false;
            }

            int aNbElements = aPair.Extent();
            for (int anElemIt = 1; anElemIt <= aNbElements; ++anElemIt)
            {
                int aTriId = aPair.Index(anElemIt);
                if (aTriId < 0)
                {
                    continue;
                }

                BRepMesh_Triangle aElement = GetTriangle(aTriId);
                BRepMesh_Triangle.Int3 anEdges = aElement.myEdges;

                for (int anEdgeIt = 0; anEdgeIt < 3; ++anEdgeIt)
                {
                    int anEdgeId = anEdges[anEdgeIt];
                    if (anEdgeId == aCurrentLinkId)
                    {
                        continue;
                    }

                    BRepMesh_Edge anEdge = GetEdge(anEdgeId);
                    if (anEdge.FirstNode() != theRefNodeId && anEdge.LastNode() != theRefNodeId)
                    {
                        continue;
                    }

                    if (anEdge.Movability() != BRepMesh_Free)
                    {
                        return true;
                    }

                    if (aVisitedLinks.Add(anEdgeId))
                    {
                        aLinkStack.Push(anEdgeId);
                    }
                }
            }
        }

        return false;
    }

    private void cleanupMesh()
    {
        Span<int> v = stackalloc int[3];
        for (;;)
        {
            NCollection_IntDataMap<bool> aLoopEdges = new(10);
            NCollection_PackedMapOfInteger aDelTriangles = new();

            NCollection_PackedMapOfInteger aFreeEdges = FreeEdges();
            NCollection_PackedMapOfInteger.Iterator aFreeEdgesIt = new(aFreeEdges);
            for (; aFreeEdgesIt.More(); aFreeEdgesIt.Next())
            {
                int aFreeEdgeId = aFreeEdgesIt.Key();
                BRepMesh_Edge anEdge = GetEdge(aFreeEdgeId);
                if (anEdge.Movability() == BRepMesh_Frontier)
                {
                    continue;
                }

                BRepMesh_PairOfIndex aPair = myMeshData!.ElementsConnectedTo(aFreeEdgeId);
                if (aPair.IsEmpty())
                {
                    aLoopEdges.Bind(aFreeEdgeId, true);
                    continue;
                }

                int aTriId = aPair.FirstIndex();

                // Check that the connected triangle is not surrounded by another triangles
                BRepMesh_Triangle aElement = GetTriangle(aTriId);
                BRepMesh_Triangle.Int3 anEdges = aElement.myEdges;

                bool isCanNotBeRemoved = true;
                for (int aCurEdgeIdx = 0; aCurEdgeIdx < 3; ++aCurEdgeIdx)
                {
                    if (anEdges[aCurEdgeIdx] != aFreeEdgeId)
                    {
                        continue;
                    }

                    for (int anOtherEdgeIt = 1; anOtherEdgeIt <= 2 && isCanNotBeRemoved; ++anOtherEdgeIt)
                    {
                        int anOtherEdgeId = (aCurEdgeIdx + anOtherEdgeIt) % 3;
                        BRepMesh_PairOfIndex anOtherEdgePair =
                            myMeshData.ElementsConnectedTo(anEdges[anOtherEdgeId]);

                        if (anOtherEdgePair.Extent() < 2)
                        {
                            isCanNotBeRemoved = false;
                        }
                        else
                        {
                            for (int aTriIdx = 1; aTriIdx <= anOtherEdgePair.Extent() && isCanNotBeRemoved;
                                 ++aTriIdx)
                            {
                                if (anOtherEdgePair.Index(aTriIdx) == aTriId)
                                {
                                    continue;
                                }

                                BRepMesh_Triangle aCurTriangle = GetTriangle(anOtherEdgePair.Index(aTriIdx));
                                myMeshData.ElementNodes(aCurTriangle, v);
                                for (int aNodeIdx = 0; aNodeIdx < 3 && isCanNotBeRemoved; ++aNodeIdx)
                                {
                                    if (isSupVertex(v[aNodeIdx]))
                                    {
                                        isCanNotBeRemoved = false;
                                    }
                                }
                            }
                        }
                    }

                    break;
                }

                if (isCanNotBeRemoved)
                {
                    continue;
                }

                Span<bool> isConnected = [false, false];
                for (int aLinkNodeIt = 0; aLinkNodeIt < 2; ++aLinkNodeIt)
                {
                    isConnected[aLinkNodeIt] =
                        isBoundToFrontier((aLinkNodeIt == 0) ? anEdge.FirstNode() : anEdge.LastNode(),
                                          aFreeEdgeId);
                }

                if (!isConnected[0] || !isConnected[1])
                {
                    aDelTriangles.Add(aTriId);
                }
            }

            // Destruction of triangles :
            int aDeletedTrianglesNb = 0;
            NCollection_PackedMapOfInteger.Iterator aDelTrianglesIt = new(aDelTriangles);
            for (; aDelTrianglesIt.More(); aDelTrianglesIt.Next())
            {
                deleteTriangle(aDelTrianglesIt.Key(), aLoopEdges);
                aDeletedTrianglesNb++;
            }

            // Destruction of remaining hanging edges
            NCollection_IntDataMap<bool>.Iterator aLoopEdgesIt = new(aLoopEdges);
            for (; aLoopEdgesIt.More(); aLoopEdgesIt.Next())
            {
                if (myMeshData!.ElementsConnectedTo(aLoopEdgesIt.Key()).IsEmpty())
                {
                    myMeshData.RemoveLink(aLoopEdgesIt.Key());
                }
            }

            if (aDeletedTrianglesNb == 0)
            {
                break;
            }
        }
    }

    private void frontierAdjust()
    {
        NCollection_PackedMapOfInteger aFrontier = Frontier();

        NCollection_DynamicArray<int> aFailedFrontiers = new(256);
        NCollection_IntDataMap<bool> aLoopEdges = new(10);
        NCollection_PackedMapOfInteger aIntFrontierEdges = new();

        for (int aPass = 1; aPass <= 2; ++aPass)
        {
            // 1 pass): find external triangles on boundary edges;
            // 2 pass): find external triangles on boundary edges appeared
            //          during triangles replacement.

            NCollection_PackedMapOfInteger.Iterator aFrontierIt = new(aFrontier);
            for (; aFrontierIt.More(); aFrontierIt.Next())
            {
                int aFrontierId = aFrontierIt.Key();
                BRepMesh_PairOfIndex aPair = myMeshData!.ElementsConnectedTo(aFrontierId);
                int aNbElem = aPair.Extent();
                for (int aElemIt = 1; aElemIt <= aNbElem; ++aElemIt)
                {
                    int aPriorElemId = aPair.Index(aElemIt);
                    if (aPriorElemId < 0)
                    {
                        continue;
                    }

                    BRepMesh_Triangle aElement = GetTriangle(aPriorElemId);
                    BRepMesh_Triangle.Int3 e = aElement.myEdges;
                    BRepMesh_Triangle.Bool3 o = aElement.myOrientations;

                    bool isTriangleFound = false;
                    for (int n = 0; n < 3; ++n)
                    {
                        if (aFrontierId == e[n] && !o[n])
                        {
                            // Destruction  of external triangles on boundary edges
                            isTriangleFound = true;
                            deleteTriangle(aPriorElemId, aLoopEdges);
                            break;
                        }
                    }

                    if (isTriangleFound)
                    {
                        break;
                    }
                }
            }

            // destrucrion of remaining hanging edges :
            NCollection_IntDataMap<bool>.Iterator aLoopEdgesIt = new(aLoopEdges);
            for (; aLoopEdgesIt.More(); aLoopEdgesIt.Next())
            {
                int aLoopEdgeId = aLoopEdgesIt.Key();
                if (myMeshData!.ElementsConnectedTo(aLoopEdgeId).IsEmpty())
                {
                    myMeshData.RemoveLink(aLoopEdgeId);
                }
            }

            // destruction of triangles crossing the boundary edges and
            // their replacement by makeshift triangles
            for (aFrontierIt.Reset(); aFrontierIt.More(); aFrontierIt.Next())
            {
                int aFrontierId = aFrontierIt.Key();
                if (!myMeshData!.ElementsConnectedTo(aFrontierId).IsEmpty())
                {
                    continue;
                }

                bool isSuccess = meshLeftPolygonOf(aFrontierId, true, aIntFrontierEdges);

                if (aPass == 2 && !isSuccess)
                {
                    aFailedFrontiers.Append(aFrontierId);
                }
            }
        }

        cleanupMesh();

        // When the mesh has been cleaned up, try to process frontier edges
        // once again to fill the possible gaps that might be occurred in case of "saw" -
        // situation when frontier edge has a triangle at a right side, but its free
        // links cross another frontieres  and meshLeftPolygonOf itself can't collect
        // a closed polygon.
        for (int aFailedFrontiersIt = 0; aFailedFrontiersIt < aFailedFrontiers.Length(); ++aFailedFrontiersIt)
        {
            int aFrontierId = aFailedFrontiers[aFailedFrontiersIt];
            if (!myMeshData!.ElementsConnectedTo(aFrontierId).IsEmpty())
            {
                continue;
            }

            meshLeftPolygonOf(aFrontierId, true, aIntFrontierEdges);
        }
    }

    private bool meshLeftPolygonOf(int theStartEdgeId, bool isForward, NCollection_PackedMapOfInteger? theSkipped = null)
    {
        if (theSkipped is not null && theSkipped.Contains(theStartEdgeId))
        {
            return true;
        }

        BRepMesh_Edge aRefEdge = GetEdge(theStartEdgeId);

        NCollection_Sequence<int> aPolygon = new();
        int aStartNode, aPivotNode;
        if (isForward)
        {
            aPolygon.Append(theStartEdgeId);
            aStartNode = aRefEdge.FirstNode();
            aPivotNode = aRefEdge.LastNode();
        }
        else
        {
            aPolygon.Append(-theStartEdgeId);
            aStartNode = aRefEdge.LastNode();
            aPivotNode = aRefEdge.FirstNode();
        }

        BRepMesh_Vertex aStartEdgeVertexS = GetVertex(aStartNode);
        BRepMesh_Vertex aPivotVertex = GetVertex(aPivotNode);

        gp_Vec2d aRefLinkDir = new(aPivotVertex.Coord() - aStartEdgeVertexS.Coord());

        if (aRefLinkDir.SquareMagnitude() < Precision2)
        {
            return true;
        }

        // Auxiliary structures.
        // Bounding boxes of polygon links to be used for preliminary
        // analysis of intersections
        NCollection_Sequence<Bnd_B2d> aBoxes = new();
        fillBndBox(aBoxes, aStartEdgeVertexS, aPivotVertex);

        // Hanging ends
        NCollection_PackedMapOfInteger aDeadLinks = new();

        // Links are temporarily excluded from consideration
        NCollection_PackedMapOfInteger aLeprousLinks = new();
        aLeprousLinks.Add(theStartEdgeId);

        bool isSkipLeprous = true;
        int aFirstNode = aStartNode;
        while (aPivotNode != aFirstNode)
        {
            Bnd_B2d aNextLinkBndBox = new();
            gp_Vec2d aNextLinkDir = new();
            int aNextPivotNode = 0;

            int aNextLinkId = findNextPolygonLink(aFirstNode,
                                                  aPivotNode,
                                                  aPivotVertex,
                                                  aRefLinkDir,
                                                  aBoxes,
                                                  aPolygon,
                                                  theSkipped,
                                                  isSkipLeprous,
                                                  aLeprousLinks,
                                                  aDeadLinks,
                                                  ref aNextPivotNode,
                                                  ref aNextLinkDir,
                                                  ref aNextLinkBndBox);

            if (aNextLinkId != 0)
            {
                aStartNode = aPivotNode;
                aRefLinkDir = aNextLinkDir;

                aPivotNode = aNextPivotNode;
                aPivotVertex = GetVertex(aNextPivotNode);

                aBoxes.Append(aNextLinkBndBox);
                aPolygon.Append(aNextLinkId);

                isSkipLeprous = true;
            }
            else
            {
                // Nothing to do
                if (aPolygon.Length() == 1)
                {
                    return false;
                }

                // Return to the previous point
                int aDeadLinkId = Math.Abs(aPolygon.Last());
                aDeadLinks.Add(aDeadLinkId);

                aLeprousLinks.Remove(aDeadLinkId);
                aPolygon.Remove(aPolygon.Length());
                aBoxes.Remove(aBoxes.Length());

                int aPrevLinkInfo = aPolygon.Last();
                BRepMesh_Edge aPrevLink = GetEdge(Math.Abs(aPrevLinkInfo));

                if (aPrevLinkInfo > 0)
                {
                    aStartNode = aPrevLink.FirstNode();
                    aPivotNode = aPrevLink.LastNode();
                }
                else
                {
                    aStartNode = aPrevLink.LastNode();
                    aPivotNode = aPrevLink.FirstNode();
                }

                aPivotVertex = GetVertex(aPivotNode);
                aRefLinkDir = new gp_Vec2d(aPivotVertex.Coord() - GetVertex(aStartNode).Coord());

                isSkipLeprous = false;
            }
        }

        if (aPolygon.Length() < 3)
        {
            return false;
        }

        cleanupPolygon(aPolygon, aBoxes);
        meshPolygon(aPolygon, aBoxes, theSkipped);

        return true;
    }

    private int findNextPolygonLink(int theFirstNode,
                                    int thePivotNode,
                                    in BRepMesh_Vertex thePivotVertex,
                                    in gp_Vec2d theRefLinkDir,
                                    NCollection_Sequence<Bnd_B2d> theBoxes,
                                    NCollection_Sequence<int> thePolygon,
                                    NCollection_PackedMapOfInteger? theSkipped,
                                    bool isSkipLeprous,
                                    NCollection_PackedMapOfInteger theLeprousLinks,
                                    NCollection_PackedMapOfInteger theDeadLinks,
                                    ref int theNextPivotNode,
                                    ref gp_Vec2d theNextLinkDir,
                                    ref Bnd_B2d theNextLinkBndBox)
    {
        // Find the next link having the greatest angle
        // respect to a direction of a reference one
        double aMaxAngle = RealFirst();

        int aNextLinkId = 0;
        BRepMesh_DataStructureOfDelaun.LinkedListOfInteger.Iterator aLinkIt = new(myMeshData!.LinksConnectedTo(thePivotNode));
        for (; aLinkIt.More(); aLinkIt.Next())
        {
            int aNeighbourLinkInfo = aLinkIt.Value();
            int aNeighbourLinkId = Math.Abs(aNeighbourLinkInfo);

            if (theDeadLinks.Contains(aNeighbourLinkId)
                || (theSkipped is not null && theSkipped.Contains(aNeighbourLinkId)))
            {
                continue;
            }

            bool isLeprous = theLeprousLinks.Contains(aNeighbourLinkId);
            if (isSkipLeprous && isLeprous)
            {
                continue;
            }

            BRepMesh_Edge aNeighbourLink = GetEdge(aNeighbourLinkId);

            // Determine whether the link belongs to the mesh
            if (aNeighbourLink.Movability() == BRepMesh_Free
                && myMeshData.ElementsConnectedTo(aNeighbourLinkInfo).IsEmpty())
            {
                theDeadLinks.Add(aNeighbourLinkId);
                continue;
            }

            int anOtherNode = aNeighbourLink.FirstNode();
            if (anOtherNode == thePivotNode)
            {
                anOtherNode = aNeighbourLink.LastNode();
            }

            gp_Vec2d aCurLinkDir = new(GetVertex(anOtherNode).Coord() - thePivotVertex.Coord());

            if (aCurLinkDir.SquareMagnitude() < Precision2)
            {
                theDeadLinks.Add(aNeighbourLinkId);
                continue;
            }

            if (!isLeprous)
            {
                theLeprousLinks.Add(aNeighbourLinkId);
            }

            double anAngle = theRefLinkDir.Angle(aCurLinkDir);
            bool isFrontier = (aNeighbourLink.Movability() == BRepMesh_Frontier);

            bool isCheckPointOnEdge = true;
            if (isFrontier)
            {
                if (Math.Abs(Math.Abs(anAngle) - M_PI) < Occt.Precision.Angular())
                {
                    // Glued constrains - don't check intersection
                    isCheckPointOnEdge = false;
                    anAngle = Math.Abs(anAngle);
                }
            }

            if (anAngle <= aMaxAngle)
            {
                continue;
            }

            bool isCheckEndPoints = (anOtherNode != theFirstNode);

            Bnd_B2d aBox = new();
            bool isNotIntersect = checkIntersection(aNeighbourLink,
                                                    thePolygon,
                                                    theBoxes,
                                                    isCheckEndPoints,
                                                    isCheckPointOnEdge,
                                                    true,
                                                    ref aBox);

            if (isNotIntersect)
            {
                aMaxAngle = anAngle;

                theNextLinkDir = aCurLinkDir;
                theNextPivotNode = anOtherNode;
                theNextLinkBndBox = aBox;

                aNextLinkId =
                    (aNeighbourLink.FirstNode() == thePivotNode) ? aNeighbourLinkId : -aNeighbourLinkId;
            }
        }

        return aNextLinkId;
    }

    private bool checkIntersection(in BRepMesh_Edge theLink,
                                   NCollection_Sequence<int> thePolygon,
                                   NCollection_Sequence<Bnd_B2d> thePolyBoxes,
                                   bool isConsiderEndPointTouch,
                                   bool isConsiderPointOnEdge,
                                   bool isSkipLastEdge,
                                   ref Bnd_B2d theLinkBndBox)
    {
        UpdateBndBox(GetVertex(theLink.FirstNode()).Coord(),
                     GetVertex(theLink.LastNode()).Coord(),
                     ref theLinkBndBox);

        int aPolyLen = thePolygon.Length();
        // Don't check intersection with the last link
        if (isSkipLastEdge)
        {
            --aPolyLen;
        }

        bool isFrontier = (theLink.Movability() == BRepMesh_Frontier);

        for (int aPolyIt = 1; aPolyIt <= aPolyLen; ++aPolyIt)
        {
            if (!theLinkBndBox.IsOut(thePolyBoxes.Value(aPolyIt)))
            {
                // intersection is possible...
                int aPolyLinkId = Math.Abs(thePolygon[aPolyIt]);
                BRepMesh_Edge aPolyLink = GetEdge(aPolyLinkId);

                // skip intersections between frontier edges
                if (aPolyLink.Movability() == BRepMesh_Frontier && isFrontier)
                {
                    continue;
                }

                gp_Pnt2d anIntPnt = new();
                BRepMesh_GeomTool.IntFlag aIntFlag =
                    intSegSeg(theLink, aPolyLink, isConsiderEndPointTouch, isConsiderPointOnEdge, ref anIntPnt);

                if (aIntFlag != NoIntersection)
                {
                    return false;
                }
            }
        }

        // Ok, no intersection
        return true;
    }

    private void addTriangle(ReadOnlySpan<int> theEdgesId, ReadOnlySpan<bool> theEdgesOri, ReadOnlySpan<int> theNodesId)
    {
        int aNewTriangleId =
            myMeshData!.AddElement(new BRepMesh_Triangle(theEdgesId, theEdgesOri, BRepMesh_Free));

        bool isAdded = true;
        if (myInitCircles)
        {
            isAdded = myCircles.Bind(aNewTriangleId,
                                     GetVertex(theNodesId[0]).Coord(),
                                     GetVertex(theNodesId[1]).Coord(),
                                     GetVertex(theNodesId[2]).Coord());
        }

        if (!isAdded)
        {
            myMeshData.RemoveElement(aNewTriangleId);
        }
    }

    private void cleanupPolygon(NCollection_Sequence<int> thePolygon, NCollection_Sequence<Bnd_B2d> thePolyBoxes)
    {
        int aPolyLen = thePolygon.Length();
        if (aPolyLen < 3)
        {
            return;
        }

        NCollection_IntDataMap<bool> aLoopEdges = new(10);
        NCollection_PackedMapOfInteger anIgnoredEdges = new();
        NCollection_PackedMapOfInteger aPolyVerticesFindMap = new();
        NCollection_DynamicArray<int> aPolyVertices = new(256);
        // Collect boundary vertices of the polygon
        for (int aPolyIt = 1; aPolyIt <= aPolyLen; ++aPolyIt)
        {
            int aPolyEdgeInfo = thePolygon[aPolyIt];
            int aPolyEdgeId = Math.Abs(aPolyEdgeInfo);
            anIgnoredEdges.Add(aPolyEdgeId);

            bool isForward = (aPolyEdgeInfo > 0);
            BRepMesh_PairOfIndex aPair = myMeshData!.ElementsConnectedTo(aPolyEdgeId);

            int anElemIt = 1;
            for (; anElemIt <= aPair.Extent(); ++anElemIt)
            {
                int anElemId = aPair.Index(anElemIt);
                if (anElemId < 0)
                {
                    continue;
                }

                BRepMesh_Triangle aElement = GetTriangle(anElemId);
                BRepMesh_Triangle.Int3 anEdges = aElement.myEdges;
                BRepMesh_Triangle.Bool3 anEdgesOri = aElement.myOrientations;

                bool isTriangleFound = false;
                for (int anEdgeIt = 0; anEdgeIt < 3; ++anEdgeIt)
                {
                    if (anEdges[anEdgeIt] == aPolyEdgeId && anEdgesOri[anEdgeIt] == isForward)
                    {
                        isTriangleFound = true;
                        deleteTriangle(anElemId, aLoopEdges);
                        break;
                    }
                }

                if (isTriangleFound)
                {
                    break;
                }
            }

            // Skip a neighbor link to extract unique vertices each time
            if (aPolyIt % 2 != 0)
            {
                BRepMesh_Edge aPolyEdge = GetEdge(aPolyEdgeId);
                int aFirstVertex = aPolyEdge.FirstNode();
                int aLastVertex = aPolyEdge.LastNode();

                aPolyVerticesFindMap.Add(aFirstVertex);
                aPolyVerticesFindMap.Add(aLastVertex);

                if (aPolyEdgeInfo > 0)
                {
                    aPolyVertices.Append(aFirstVertex);
                    aPolyVertices.Append(aLastVertex);
                }
                else
                {
                    aPolyVertices.Append(aLastVertex);
                    aPolyVertices.Append(aFirstVertex);
                }
            }
        }

        // Make closed sequence
        if (aPolyVertices.First() != aPolyVertices.Last())
        {
            aPolyVertices.Append(aPolyVertices.First());
        }

        NCollection_PackedMapOfInteger aSurvivedLinks = new NCollection_PackedMapOfInteger().Assign(anIgnoredEdges);

        for (int aPolyVertIt = 0; aPolyVertIt < aPolyVertices.Length() - 1; ++aPolyVertIt)
        {
            StackOfFrames aStackFames = new();
            NCollection_DynamicArray<int> aStackData = new();
            for (int aCurrentVictim = aPolyVertices[aPolyVertIt]; aCurrentVictim != -1;
                 aCurrentVictim = aStackFames.IsEmpty() ? -1 : aStackData[aStackFames.PopElement()])
            {
                int aPrevStackDataSize = aStackData.Length();
                killTrianglesAroundVertex(aCurrentVictim,
                                          aPolyVertices,
                                          aPolyVerticesFindMap,
                                          thePolygon,
                                          thePolyBoxes,
                                          aSurvivedLinks,
                                          aLoopEdges,
                                          aStackData);
                int aNewStackDataSize = aStackData.Length();
                if (aNewStackDataSize > aPrevStackDataSize)
                {
                    aStackFames.PushFrame(aPrevStackDataSize, aNewStackDataSize);
                }
            }
        }

        NCollection_IntDataMap<bool>.Iterator aLoopEdgesIt = new(aLoopEdges);
        for (; aLoopEdgesIt.More(); aLoopEdgesIt.Next())
        {
            int aLoopEdgeId = aLoopEdgesIt.Key();
            if (anIgnoredEdges.Contains(aLoopEdgeId))
            {
                continue;
            }

            if (myMeshData!.ElementsConnectedTo(aLoopEdgeId).IsEmpty())
            {
                myMeshData.RemoveLink(aLoopEdgesIt.Key());
            }
        }
    }

    private void killTrianglesAroundVertex(int theZombieNodeId,
                                           NCollection_DynamicArray<int> thePolyVertices,
                                           NCollection_PackedMapOfInteger thePolyVerticesFindMap,
                                           NCollection_Sequence<int> thePolygon,
                                           NCollection_Sequence<Bnd_B2d> thePolyBoxes,
                                           NCollection_PackedMapOfInteger theSurvivedLinks,
                                           NCollection_IntDataMap<bool> theLoopEdges,
                                           NCollection_DynamicArray<int> theVictimNodes)
    {
        // The list is walked while links are removed from it (OCCT's linked-list semantics, decision 4404).
        BRepMesh_DataStructureOfDelaun.LinkedListOfInteger.Iterator aNeighborsIt = new(myMeshData!.LinksConnectedTo(theZombieNodeId));

        // Try to infect neighbor nodes
        for (; aNeighborsIt.More(); aNeighborsIt.Next())
        {
            int aNeighborLinkId = aNeighborsIt.Value();
            if (theSurvivedLinks.Contains(aNeighborLinkId))
            {
                continue;
            }

            BRepMesh_Edge aNeighborLink = GetEdge(aNeighborLinkId);
            if (aNeighborLink.Movability() == BRepMesh_Frontier)
            {
                // Though, if it lies onto the polygon boundary -
                // take its triangles
                Bnd_B2d aBox = new();
                bool isNotIntersect =
                    checkIntersection(aNeighborLink, thePolygon, thePolyBoxes, false, true, false, ref aBox);

                if (isNotIntersect)
                {
                    // Don't touch it
                    theSurvivedLinks.Add(aNeighborLinkId);
                    continue;
                }
            }
            else
            {
                int anOtherNode = aNeighborLink.FirstNode();
                if (anOtherNode == theZombieNodeId)
                {
                    anOtherNode = aNeighborLink.LastNode();
                }

                // Possible sacrifice
                if (!thePolyVerticesFindMap.Contains(anOtherNode))
                {
                    if (isVertexInsidePolygon(anOtherNode, thePolyVertices))
                    {
                        // Got you!
                        theVictimNodes.Append(anOtherNode);
                    }
                    else
                    {
                        // Lucky. But it may intersect the polygon boundary.
                        // Let's check it!
                        killTrianglesOnIntersectingLinks(aNeighborLinkId,
                                                         aNeighborLink,
                                                         anOtherNode,
                                                         thePolygon,
                                                         thePolyBoxes,
                                                         theSurvivedLinks,
                                                         theLoopEdges);

                        continue;
                    }
                }
            }

            // Add link to the survivors to avoid cycling
            theSurvivedLinks.Add(aNeighborLinkId);
            killLinkTriangles(aNeighborLinkId, theLoopEdges);
        }
    }

    private bool isVertexInsidePolygon(int theVertexId, NCollection_DynamicArray<int> thePolygonVertices)
    {
        int aPolyLen = thePolygonVertices.Length();
        if (aPolyLen < 3)
        {
            return false;
        }

        gp_XY aCenterPointXY = GetVertex(theVertexId).Coord();

        BRepMesh_Vertex aFirstVertex = GetVertex(thePolygonVertices[0]);
        gp_Vec2d aPrevVertexDir = new(aFirstVertex.Coord() - aCenterPointXY);
        if (aPrevVertexDir.SquareMagnitude() < Precision2)
        {
            return true;
        }

        double aTotalAng = 0.0;
        for (int aPolyIt = 1; aPolyIt < aPolyLen; ++aPolyIt)
        {
            BRepMesh_Vertex aPolyVertex = GetVertex(thePolygonVertices[aPolyIt]);

            gp_Vec2d aCurVertexDir = new(aPolyVertex.Coord() - aCenterPointXY);
            if (aCurVertexDir.SquareMagnitude() < Precision2)
            {
                return true;
            }

            aTotalAng += aCurVertexDir.Angle(aPrevVertexDir);
            aPrevVertexDir = aCurVertexDir;
        }

        return Math.Abs(Math.Abs(aTotalAng) - Angle2PI) <= Occt.Precision.Angular();
    }

    private void killTrianglesOnIntersectingLinks(int theLinkToCheckId,
                                                  in BRepMesh_Edge theLinkToCheck,
                                                  int theEndPoint,
                                                  NCollection_Sequence<int> thePolygon,
                                                  NCollection_Sequence<Bnd_B2d> thePolyBoxes,
                                                  NCollection_PackedMapOfInteger theSurvivedLinks,
                                                  NCollection_IntDataMap<bool> theLoopEdges)
    {
        if (theSurvivedLinks.Contains(theLinkToCheckId))
        {
            return;
        }

        Bnd_B2d aBox = new();
        bool isNotIntersect =
            checkIntersection(theLinkToCheck, thePolygon, thePolyBoxes, false, false, false, ref aBox);

        theSurvivedLinks.Add(theLinkToCheckId);

        if (isNotIntersect)
        {
            return;
        }

        killLinkTriangles(theLinkToCheckId, theLoopEdges);

        BRepMesh_DataStructureOfDelaun.LinkedListOfInteger.Iterator aNeighborsIt = new(myMeshData!.LinksConnectedTo(theEndPoint));

        for (; aNeighborsIt.More(); aNeighborsIt.Next())
        {
            int aNeighborLinkId = aNeighborsIt.Value();
            BRepMesh_Edge aNeighborLink = GetEdge(aNeighborLinkId);
            int anOtherNode = aNeighborLink.FirstNode();
            if (anOtherNode == theEndPoint)
            {
                anOtherNode = aNeighborLink.LastNode();
            }

            killTrianglesOnIntersectingLinks(aNeighborLinkId,
                                             aNeighborLink,
                                             anOtherNode,
                                             thePolygon,
                                             thePolyBoxes,
                                             theSurvivedLinks,
                                             theLoopEdges);
        }
    }

    private void killLinkTriangles(int theLinkId, NCollection_IntDataMap<bool> theLoopEdges)
    {
        // A reference: deleting a triangle removes it from the pair, so FirstIndex() moves on (as OCCT's const&).
        ref readonly BRepMesh_PairOfIndex aPair = ref myMeshData!.ElementsConnectedTo(theLinkId);

        int anElemNb = aPair.Extent();
        for (int aPairIt = 1; aPairIt <= anElemNb; ++aPairIt)
        {
            int anElemId = aPair.FirstIndex();
            if (anElemId < 0)
            {
                continue;
            }

            deleteTriangle(anElemId, theLoopEdges);
        }
    }

    private static void getOrientedNodes(in BRepMesh_Edge theEdge, bool isForward, Span<int> theNodes)
    {
        if (isForward)
        {
            theNodes[0] = theEdge.FirstNode();
            theNodes[1] = theEdge.LastNode();
        }
        else
        {
            theNodes[0] = theEdge.LastNode();
            theNodes[1] = theEdge.FirstNode();
        }
    }

    private void processLoop(int theLinkFrom,
                             int theLinkTo,
                             NCollection_Sequence<int> thePolygon,
                             NCollection_Sequence<Bnd_B2d> thePolyBoxes)
    {
        int aNbOfLinksInLoop = theLinkTo - theLinkFrom - 1;
        if (aNbOfLinksInLoop < 3)
        {
            return;
        }

        NCollection_Sequence<int> aPolygon = new();
        NCollection_Sequence<Bnd_B2d> aPolyBoxes = new();
        for (; aNbOfLinksInLoop > 0; --aNbOfLinksInLoop)
        {
            int aLoopLinkIndex = theLinkFrom + aNbOfLinksInLoop;
            aPolygon.Prepend(thePolygon[aLoopLinkIndex]);
            aPolyBoxes.Prepend(thePolyBoxes[aLoopLinkIndex]);
        }

        meshPolygon(aPolygon, aPolyBoxes);
    }

    private int createAndReplacePolygonLink(ReadOnlySpan<int> theNodes,
                                            ReadOnlySpan<gp_Pnt2d> thePnts,
                                            int theRootIndex,
                                            ReplaceFlag theReplaceFlag,
                                            NCollection_Sequence<int> thePolygon,
                                            NCollection_Sequence<Bnd_B2d> thePolyBoxes)
    {
        int aNewEdgeId = myMeshData!.AddLink(new BRepMesh_Edge(theNodes[0], theNodes[1], BRepMesh_Free));

        Bnd_B2d aNewBox = new();
        UpdateBndBox(thePnts[0].Coord(), thePnts[1].Coord(), ref aNewBox);

        switch (theReplaceFlag)
        {
            case ReplaceFlag.Replace:
                thePolygon.SetValue(theRootIndex, aNewEdgeId);
                thePolyBoxes.SetValue(theRootIndex, aNewBox);
                break;

            case ReplaceFlag.InsertAfter:
                thePolygon.InsertAfter(theRootIndex, aNewEdgeId);
                thePolyBoxes.InsertAfter(theRootIndex, aNewBox);
                break;

            case ReplaceFlag.InsertBefore:
                thePolygon.InsertBefore(theRootIndex, aNewEdgeId);
                thePolyBoxes.InsertBefore(theRootIndex, aNewBox);
                break;
        }

        return aNewEdgeId;
    }

    // OCCT's NCollection_Sequence::Remove(from, to) in a release build: its range check is compiled out, and with
    // to == from - 1 it removes nothing. The port's Remove checks, so that case is handled here.
    private static void removeRange<T>(NCollection_Sequence<T> theSequence, int theFromIndex, int theToIndex)
    {
        if (theToIndex == theFromIndex - 1)
        {
            return;
        }

        theSequence.Remove(theFromIndex, theToIndex);
    }

    private void meshPolygon(NCollection_Sequence<int> thePolygon,
                             NCollection_Sequence<Bnd_B2d> thePolyBoxes,
                             NCollection_PackedMapOfInteger? theSkipped = null)
    {
        // Check is the source polygon elementary
        if (meshElementaryPolygon(thePolygon))
        {
            return;
        }

        // Check and correct boundary edges
        int aPolyLen = thePolygon.Length();
        double aPolyArea = Math.Abs(polyArea(thePolygon, 1, aPolyLen));
        double aSmallLoopArea = 0.001 * aPolyArea;
        Span<int> aCurNodes = stackalloc int[2];
        Span<gp_Pnt2d> aCurPnts = stackalloc gp_Pnt2d[2];
        Span<int> aNextNodes = stackalloc int[2];
        Span<gp_Pnt2d> aNextPnts = stackalloc gp_Pnt2d[2];
        Span<int> aSplitLink = stackalloc int[3];
        Span<gp_Pnt2d> aSplitPnts = stackalloc gp_Pnt2d[3];
        Span<int> aSplitLinkNodes = stackalloc int[2];
        Span<gp_Pnt2d> aSplitLinkPnts = stackalloc gp_Pnt2d[2];
        for (int aPolyIt = 1; aPolyIt < aPolyLen; ++aPolyIt)
        {
            int aCurEdgeInfo = thePolygon[aPolyIt];
            int aCurEdgeId = Math.Abs(aCurEdgeInfo);
            BRepMesh_Edge aCurEdge = GetEdge(aCurEdgeId);
            if (aCurEdge.Movability() != BRepMesh_Frontier)
            {
                continue;
            }

            getOrientedNodes(aCurEdge, aCurEdgeInfo > 0, aCurNodes);

            aCurPnts[0] = new gp_Pnt2d(GetVertex(aCurNodes[0]).Coord());
            aCurPnts[1] = new gp_Pnt2d(GetVertex(aCurNodes[1]).Coord());

            gp_Vec2d aCurVec = new(aCurPnts[0], aCurPnts[1]);

            // check further links
            int aNextPolyIt = aPolyIt + 1;
            for (; aNextPolyIt <= aPolyLen; ++aNextPolyIt)
            {
                int aNextEdgeInfo = thePolygon[aNextPolyIt];
                int aNextEdgeId = Math.Abs(aNextEdgeInfo);
                BRepMesh_Edge aNextEdge = GetEdge(aNextEdgeId);
                if (aNextEdge.Movability() != BRepMesh_Frontier)
                {
                    continue;
                }

                getOrientedNodes(aNextEdge, aNextEdgeInfo > 0, aNextNodes);

                aNextPnts[0] = new gp_Pnt2d(GetVertex(aNextNodes[0]).Coord());
                aNextPnts[1] = new gp_Pnt2d(GetVertex(aNextNodes[1]).Coord());

                gp_Pnt2d anIntPnt = new();
                BRepMesh_GeomTool.IntFlag aIntFlag = intSegSeg(aCurEdge, aNextEdge, false, true, ref anIntPnt);

                if (aIntFlag == NoIntersection)
                {
                    continue;
                }

                bool isRemoveFromFirst = false;
                bool isAddReplacingEdge = true;
                int aIndexToRemoveTo = aNextPolyIt;
                if (aIntFlag == Cross)
                {
                    double aLoopArea = polyArea(thePolygon, aPolyIt + 1, aNextPolyIt);
                    gp_Vec2d aVec1 = new(anIntPnt, aCurPnts[1]);
                    gp_Vec2d aVec2 = new(anIntPnt, aNextPnts[0]);

                    aLoopArea += (aVec1 ^ aVec2) / 2.0;
                    if (Math.Abs(aLoopArea) > aSmallLoopArea)
                    {
                        aNextNodes[1] = aCurNodes[0];
                        aNextPnts[1] = aCurPnts[0];

                        aNextEdgeId = Math.Abs(createAndReplacePolygonLink(aNextNodes,
                                                                           aNextPnts,
                                                                           aNextPolyIt,
                                                                           ReplaceFlag.Replace,
                                                                           thePolygon,
                                                                           thePolyBoxes));

                        processLoop(aPolyIt, aNextPolyIt, thePolygon, thePolyBoxes);
                        return;
                    }

                    double aDist1 = anIntPnt.SquareDistance(aNextPnts[0]);
                    double aDist2 = anIntPnt.SquareDistance(aNextPnts[1]);

                    // Choose node with lower distance
                    bool isCloseToStart = (aDist1 < aDist2);
                    int aEndPointIndex = isCloseToStart ? 0 : 1;
                    aCurNodes[1] = aNextNodes[aEndPointIndex];
                    aCurPnts[1] = aNextPnts[aEndPointIndex];

                    if (isCloseToStart)
                    {
                        --aIndexToRemoveTo;
                    }

                    // In this context only intersections between frontier edges
                    // are possible. If intersection between edges of different
                    // types occurred - treat this case as invalid (i.e. result
                    // might not reflect the expectations).
                    if (theSkipped is not null)
                    {
                        int aSkippedLinkIt = aPolyIt;
                        for (; aSkippedLinkIt <= aIndexToRemoveTo; ++aSkippedLinkIt)
                        {
                            theSkipped.Add(Math.Abs(thePolygon[aSkippedLinkIt]));
                        }
                    }
                }
                else if (aIntFlag == PointOnSegment)
                {
                    // Identify chopping link
                    bool isFirstChopping = false;
                    int aCheckPointIt = 0;
                    for (; aCheckPointIt < 2; ++aCheckPointIt)
                    {
                        gp_Pnt2d aRefPoint = aCurPnts[aCheckPointIt];
                        // Check is second link touches the first one
                        gp_Vec2d aVec1 = new(aRefPoint, aNextPnts[0]);
                        gp_Vec2d aVec2 = new(aRefPoint, aNextPnts[1]);
                        if (Math.Abs(aVec1 ^ aVec2) < Precision)
                        {
                            isFirstChopping = true;
                            break;
                        }
                    }

                    if (isFirstChopping)
                    {
                        // Split second link
                        isAddReplacingEdge = false;
                        isRemoveFromFirst = (aCheckPointIt == 0);

                        aSplitLink[0] = aNextNodes[0];
                        aSplitLink[1] = aCurNodes[aCheckPointIt];
                        aSplitLink[2] = aNextNodes[1];

                        aSplitPnts[0] = aNextPnts[0];
                        aSplitPnts[1] = aCurPnts[aCheckPointIt];
                        aSplitPnts[2] = aNextPnts[1];

                        int aSplitLinkIt = 0;
                        for (; aSplitLinkIt < 2; ++aSplitLinkIt)
                        {
                            createAndReplacePolygonLink(aSplitLink[aSplitLinkIt..],
                                                        aSplitPnts[aSplitLinkIt..],
                                                        aNextPolyIt,
                                                        (aSplitLinkIt == 0) ? ReplaceFlag.Replace
                                                                            : ReplaceFlag.InsertAfter,
                                                        thePolygon,
                                                        thePolyBoxes);
                        }

                        processLoop(aPolyIt + aCheckPointIt, aIndexToRemoveTo, thePolygon, thePolyBoxes);
                    }
                    else
                    {
                        // Split first link
                        aSplitLinkNodes[0] = aNextNodes[1];
                        aSplitLinkNodes[1] = aCurNodes[1];

                        aSplitLinkPnts[0] = aNextPnts[1];
                        aSplitLinkPnts[1] = aCurPnts[1];
                        createAndReplacePolygonLink(aSplitLinkNodes,
                                                    aSplitLinkPnts,
                                                    aPolyIt,
                                                    ReplaceFlag.InsertAfter,
                                                    thePolygon,
                                                    thePolyBoxes);

                        aCurNodes[1] = aNextNodes[1];
                        aCurPnts[1] = aNextPnts[1];
                        ++aIndexToRemoveTo;

                        processLoop(aPolyIt + 1, aIndexToRemoveTo, thePolygon, thePolyBoxes);
                    }
                }
                else if (aIntFlag == Glued)
                {
                    if (aCurNodes[1] == aNextNodes[0])
                    {
                        aCurNodes[1] = aNextNodes[1];
                        aCurPnts[1] = aNextPnts[1];
                    }

                    // TODO: Non-adjacent glued links within the polygon
                }
                else if (aIntFlag == Same)
                {
                    processLoop(aPolyIt, aNextPolyIt, thePolygon, thePolyBoxes);

                    isRemoveFromFirst = true;
                    isAddReplacingEdge = false;
                }
                else
                {
                    continue; // Not supported type
                }

                if (isAddReplacingEdge)
                {
                    aCurEdgeId = Math.Abs(createAndReplacePolygonLink(aCurNodes,
                                                                      aCurPnts,
                                                                      aPolyIt,
                                                                      ReplaceFlag.Replace,
                                                                      thePolygon,
                                                                      thePolyBoxes));

                    aCurEdge = GetEdge(aCurEdgeId);
                    aCurVec = new gp_Vec2d(aCurPnts[0], aCurPnts[1]);
                }

                int aIndexToRemoveFrom = isRemoveFromFirst ? aPolyIt : aPolyIt + 1;

                removeRange(thePolygon, aIndexToRemoveFrom, aIndexToRemoveTo);
                removeRange(thePolyBoxes, aIndexToRemoveFrom, aIndexToRemoveTo);

                aPolyLen = thePolygon.Length();
                if (isRemoveFromFirst)
                {
                    --aPolyIt;
                    break;
                }

                aNextPolyIt = aPolyIt;
            }

            _ = aCurVec;
        }

        NCollection_Sequence<int> aPolygon1 = thePolygon;
        NCollection_Sequence<Bnd_B2d> aPolyBoxes1 = thePolyBoxes;

        NCollection_Sequence<int> aPolygon2 = new();
        NCollection_Sequence<Bnd_B2d> aPolyBoxes2 = new();

        NCollection_Sequence<NCollection_Sequence<int>> aPolyStack = new();
        NCollection_Sequence<NCollection_Sequence<Bnd_B2d>> aPolyBoxStack = new();
        for (;;)
        {
            decomposeSimplePolygon(aPolygon1, aPolyBoxes1, aPolygon2, aPolyBoxes2);
            if (!aPolygon2.IsEmpty())
            {
                aPolyStack.Append(aPolygon2);
                aPolyBoxStack.Append(aPolyBoxes2);

                aPolygon2 = new NCollection_Sequence<int>();
                aPolyBoxes2 = new NCollection_Sequence<Bnd_B2d>();
            }

            if (aPolygon1.IsEmpty())
            {
                if (!aPolyStack.IsEmpty() && ReferenceEquals(aPolygon1, aPolyStack.First()))
                {
                    aPolyStack.Remove(1);
                    aPolyBoxStack.Remove(1);
                }

                if (aPolyStack.IsEmpty())
                {
                    break;
                }

                aPolygon1 = aPolyStack.First();
                aPolyBoxes1 = aPolyBoxStack.First();
            }
        }
    }

    private bool meshElementaryPolygon(NCollection_Sequence<int> thePolygon)
    {
        int aPolyLen = thePolygon.Length();
        if (aPolyLen < 3)
        {
            return true;
        }
        else if (aPolyLen > 3)
        {
            return false;
        }

        // Just create a triangle
        Span<int> anEdges = stackalloc int[3];
        Span<bool> anEdgesOri = stackalloc bool[3];

        for (int anEdgeIt = 0; anEdgeIt < 3; ++anEdgeIt)
        {
            int anEdgeInfo = thePolygon[anEdgeIt + 1];
            anEdges[anEdgeIt] = Math.Abs(anEdgeInfo);
            anEdgesOri[anEdgeIt] = (anEdgeInfo > 0);
        }

        BRepMesh_Edge anEdge1 = GetEdge(anEdges[0]);
        BRepMesh_Edge anEdge2 = GetEdge(anEdges[1]);

        Span<int> aNodes = [anEdge1.FirstNode(), anEdge1.LastNode(), anEdge2.FirstNode()];
        if (aNodes[2] == aNodes[0] || aNodes[2] == aNodes[1])
        {
            aNodes[2] = anEdge2.LastNode();
        }

        addTriangle(anEdges, anEdgesOri, aNodes);
        return true;
    }

    private void decomposeSimplePolygon(NCollection_Sequence<int> thePolygon,
                                        NCollection_Sequence<Bnd_B2d> thePolyBoxes,
                                        NCollection_Sequence<int> thePolygonCut,
                                        NCollection_Sequence<Bnd_B2d> thePolyBoxesCut)
    {
        // Check is the given polygon elementary
        if (meshElementaryPolygon(thePolygon))
        {
            thePolygon.Clear();
            thePolyBoxes.Clear();
            return;
        }

        // Polygon contains more than 3 links
        int aFirstEdgeInfo = thePolygon[1];
        BRepMesh_Edge aFirstEdge = GetEdge(Math.Abs(aFirstEdgeInfo));

        Span<int> aNodes = stackalloc int[3];
        getOrientedNodes(aFirstEdge, aFirstEdgeInfo > 0, aNodes);

        Span<gp_Pnt2d> aRefVertices = stackalloc gp_Pnt2d[3];
        aRefVertices[0] = new gp_Pnt2d(GetVertex(aNodes[0]).Coord());
        aRefVertices[1] = new gp_Pnt2d(GetVertex(aNodes[1]).Coord());

        gp_Vec2d aRefEdgeDir = new(aRefVertices[0], aRefVertices[1]);

        double aRefEdgeLen = aRefEdgeDir.Magnitude();
        if (aRefEdgeLen < Precision)
        {
            thePolygon.Clear();
            thePolyBoxes.Clear();
            return;
        }

        aRefEdgeDir /= aRefEdgeLen;

        // Find a point with minimum distance respect
        // the end of reference link
        int aUsedLinkId = 0;
        double aOptAngle = 0.0;
        double aMinDist = RealLast();
        int aPivotNode = aNodes[1];
        int aPolyLen = thePolygon.Length();
        for (int aLinkIt = 3; aLinkIt <= aPolyLen; ++aLinkIt)
        {
            int aLinkInfo = thePolygon[aLinkIt];
            BRepMesh_Edge aNextEdge = GetEdge(Math.Abs(aLinkInfo));

            aPivotNode = aLinkInfo > 0 ? aNextEdge.FirstNode() : aNextEdge.LastNode();

            // We have end points touch case in the polygon - ignore it
            if (aPivotNode == aNodes[1])
            {
                continue;
            }

            gp_Pnt2d aPivotVertex = new(GetVertex(aPivotNode).Coord());
            gp_Vec2d aDistanceDir = new(aRefVertices[1], aPivotVertex);

            double aDist = aRefEdgeDir ^ aDistanceDir;
            double aAngle = Math.Abs(aRefEdgeDir.Angle(aDistanceDir));
            double anAbsDist = Math.Abs(aDist);
            if (anAbsDist < Precision || aDist < 0.0)
            {
                continue;
            }

            if ((anAbsDist >= aMinDist) && (aAngle <= aOptAngle || aAngle > AngDeviation90Deg))
            {
                continue;
            }

            // Check is the test link crosses the polygon boundaries
            bool isIntersect = false;
            for (int aRefLinkNodeIt = 0; aRefLinkNodeIt < 2; ++aRefLinkNodeIt)
            {
                int aLinkFirstNode = aNodes[aRefLinkNodeIt];
                gp_Pnt2d aLinkFirstVertex = aRefVertices[aRefLinkNodeIt];

                Bnd_B2d aBox = new();
                UpdateBndBox(aLinkFirstVertex.Coord(), aPivotVertex.Coord(), ref aBox);

                BRepMesh_Edge aCheckLink = new(aLinkFirstNode, aPivotNode, BRepMesh_Free);

                int aCheckLinkIt = 2;
                for (; aCheckLinkIt <= aPolyLen; ++aCheckLinkIt)
                {
                    if (aCheckLinkIt == aLinkIt)
                    {
                        continue;
                    }

                    if (!aBox.IsOut(thePolyBoxes.Value(aCheckLinkIt)))
                    {
                        BRepMesh_Edge aPolyLink = GetEdge(Math.Abs(thePolygon[aCheckLinkIt]));

                        if (aCheckLink.IsEqual(aPolyLink))
                        {
                            continue;
                        }

                        // intersection is possible...
                        gp_Pnt2d anIntPnt = new();
                        BRepMesh_GeomTool.IntFlag aIntFlag =
                            intSegSeg(aCheckLink, aPolyLink, false, false, ref anIntPnt);

                        if (aIntFlag != NoIntersection)
                        {
                            isIntersect = true;
                            break;
                        }
                    }
                }

                if (isIntersect)
                {
                    break;
                }
            }

            if (isIntersect)
            {
                continue;
            }

            aOptAngle = aAngle;
            aMinDist = anAbsDist;
            aNodes[2] = aPivotNode;
            aRefVertices[2] = aPivotVertex;
            aUsedLinkId = aLinkIt;
        }

        if (aUsedLinkId == 0)
        {
            thePolygon.Clear();
            thePolyBoxes.Clear();
            return;
        }

        BRepMesh_Edge aNewEdge0 = new(aNodes[1], aNodes[2], BRepMesh_Free);
        BRepMesh_Edge aNewEdge1 = new(aNodes[2], aNodes[0], BRepMesh_Free);

        Span<int> aNewEdgesInfo = stackalloc int[3];
        aNewEdgesInfo[0] = aFirstEdgeInfo;
        aNewEdgesInfo[1] = myMeshData!.AddLink(aNewEdge0);
        aNewEdgesInfo[2] = myMeshData.AddLink(aNewEdge1);

        Span<int> anEdges = stackalloc int[3];
        Span<bool> anEdgesOri = stackalloc bool[3];
        for (int aTriEdgeIt = 0; aTriEdgeIt < 3; ++aTriEdgeIt)
        {
            int anEdgeInfo = aNewEdgesInfo[aTriEdgeIt];
            anEdges[aTriEdgeIt] = Math.Abs(anEdgeInfo);
            anEdgesOri[aTriEdgeIt] = anEdgeInfo > 0;
        }

        addTriangle(anEdges, anEdgesOri, aNodes);

        if (aUsedLinkId == 3)
        {
            thePolygon.Remove(1);
            thePolyBoxes.Remove(1);

            thePolygon.SetValue(1, -aNewEdgesInfo[2]);

            Bnd_B2d aBox = new();
            UpdateBndBox(aRefVertices[0].Coord(), aRefVertices[2].Coord(), ref aBox);
            thePolyBoxes.SetValue(1, aBox);
        }
        else
        {
            // Create triangle and split the source polygon on two
            // parts (if possible) and mesh each part as independent
            // polygon.
            if (aUsedLinkId < aPolyLen)
            {
                thePolygon.Split(aUsedLinkId, thePolygonCut);
                thePolygonCut.Prepend(-aNewEdgesInfo[2]);
                thePolyBoxes.Split(aUsedLinkId, thePolyBoxesCut);

                Bnd_B2d aCutBox = new();
                UpdateBndBox(aRefVertices[0].Coord(), aRefVertices[2].Coord(), ref aCutBox);
                thePolyBoxesCut.Prepend(aCutBox);
            }
            else
            {
                thePolygon.Remove(aPolyLen);
                thePolyBoxes.Remove(aPolyLen);
            }

            thePolygon.SetValue(1, -aNewEdgesInfo[1]);

            Bnd_B2d aBox = new();
            UpdateBndBox(aRefVertices[1].Coord(), aRefVertices[2].Coord(), ref aBox);
            thePolyBoxes.SetValue(1, aBox);
        }
    }

    private double calculateDist(ReadOnlySpan<gp_XY> theVEdges,
                                 ReadOnlySpan<gp_XY> thePoints,
                                 in BRepMesh_Vertex theVertex,
                                 Span<double> theDistance,
                                 Span<double> theSqModulus,
                                 ref int theEdgeOn)
    {
        double aMinDist = RealLast();
        for (int i = 0; i < 3; ++i)
        {
            theSqModulus[i] = theVEdges[i].SquareModulus();
            if (theSqModulus[i] <= Precision2)
            {
                return -1;
            }

            theDistance[i] = theVEdges[i] ^ (theVertex.Coord() - thePoints[i]);

            double aDist = theDistance[i] * theDistance[i];
            aDist /= theSqModulus[i];

            if (aDist < aMinDist)
            {
                theEdgeOn = i;
                aMinDist = aDist;
            }
        }

        return aMinDist;
    }

    private BRepMesh_GeomTool.IntFlag intSegSeg(in BRepMesh_Edge theEdg1,
                                                in BRepMesh_Edge theEdg2,
                                                bool isConsiderEndPointTouch,
                                                bool isConsiderPointOnEdge,
                                                ref gp_Pnt2d theIntPnt)
    {
        gp_XY p1, p2, p3, p4;
        p1 = GetVertex(theEdg1.FirstNode()).Coord();
        p2 = GetVertex(theEdg1.LastNode()).Coord();
        p3 = GetVertex(theEdg2.FirstNode()).Coord();
        p4 = GetVertex(theEdg2.LastNode()).Coord();

        return BRepMesh_GeomTool.IntSegSeg(p1,
                                           p2,
                                           p3,
                                           p4,
                                           isConsiderEndPointTouch,
                                           isConsiderPointOnEdge,
                                           ref theIntPnt);
    }

    private double polyArea(NCollection_Sequence<int> thePolygon, int theStartIndex, int theEndIndex)
    {
        double aArea = 0.0;
        int aPolyLen = thePolygon.Length();
        if (theStartIndex >= theEndIndex || theStartIndex > aPolyLen)
        {
            return aArea;
        }

        int aCurEdgeInfo = thePolygon[theStartIndex];
        int aCurEdgeId = Math.Abs(aCurEdgeInfo);
        BRepMesh_Edge aCurEdge = GetEdge(aCurEdgeId);

        Span<int> aNodes = stackalloc int[2];
        getOrientedNodes(aCurEdge, aCurEdgeInfo > 0, aNodes);

        gp_Pnt2d aRefPnt = new(GetVertex(aNodes[0]).Coord());
        int aPolyIt = theStartIndex + 1;
        for (; aPolyIt <= theEndIndex; ++aPolyIt)
        {
            aCurEdgeInfo = thePolygon[aPolyIt];
            aCurEdgeId = Math.Abs(aCurEdgeInfo);
            aCurEdge = GetEdge(aCurEdgeId);

            getOrientedNodes(aCurEdge, aCurEdgeInfo > 0, aNodes);
            gp_Vec2d aVec1 = new(aRefPnt, new gp_Pnt2d(GetVertex(aNodes[0]).Coord()));
            gp_Vec2d aVec2 = new(aRefPnt, new gp_Pnt2d(GetVertex(aNodes[1]).Coord()));

            aArea += aVec1 ^ aVec2;
        }

        return aArea / 2.0;
    }

    private bool isSupVertex(int theVertexIdx)
    {
        for (int aIt = 0; aIt < mySupVert.Length(); ++aIt)
        {
            if (theVertexIdx == mySupVert[aIt])
            {
                return true;
            }
        }

        return false;
    }
}
