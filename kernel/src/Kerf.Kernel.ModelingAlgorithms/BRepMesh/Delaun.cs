// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Delaun.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Delaun.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using System.Runtime.InteropServices;
using static Kerf.Engine.Meshing.GeomTool.IntFlag;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Computes the Delaunay triangulation of the nodes of a <see cref="DataStructureOfDelaun"/> with Watson's
/// algorithm, then inserts its frontier and internal links (constraints) and removes the triangles outside the
/// frontier.
/// </summary>
/// <remarks>
/// A working object, as OCCT's (a class used by value there; the allocators are dropped). It writes into the data
/// structure it is given, and the constructors taking an index vector sort that vector in place, as OCCT's do, with
/// MSVC's heap sort (<see cref="Heap"/>, decision 491), so ties come out in native OCCT's order.
/// </remarks>
internal sealed class Delaun
{
    private static readonly double AngDeviation1Deg = M_PI / 180.0;
    private static readonly double AngDeviation90Deg = 90 * AngDeviation1Deg;
    private static readonly double Angle2PI = 2 * M_PI;

    private static readonly double Precision = Kerf.Engine.Foundation.Precision.PConfusion();
    private static readonly double Precision2 = Precision * Precision;

    private enum ReplaceFlag
    {
        Replace,
        InsertAfter,
        InsertBefore,
    }

    private DataStructureOfDelaun? _meshData;
    private readonly CircleTool _circles;
    private readonly List<int> _supVert;

    // Kept from one vertex to the next (unit 11B); see where they're used
    private NCollection_IntDataMap<bool>? _spareLoopEdges;
    private readonly NCollection_IntDataMap<bool>.Iterator _polyIterator = new();
    private Stack<int>? _frontierLinks;
    private HashSet<int>? _frontierVisited;
    private bool _initCircles;
    private Triangle _supTrian;

    /// <summary>
    /// A triangulator of an existing structure, which doesn't run: the circles' cells are filled from the structure's
    /// triangles if <paramref name="isFillCircles"/>.
    /// </summary>
    public Delaun(DataStructureOfDelaun oldMesh, int cellsCountU, int cellsCountV, bool isFillCircles)
    {
        _meshData = oldMesh;
        _circles = new CircleTool();
        _supVert = new List<int>(3);
        _initCircles = false;
        if (isFillCircles)
        {
            InitCirclesTool(cellsCountU, cellsCountV);
        }
    }

    /// <summary>Triangulates the vertices in a new structure (none if there are fewer than three).</summary>
    public Delaun(NCollection_Array1<MeshVertex> vertices)
    {
        _circles = new CircleTool(vertices.Length());
        _supVert = new List<int>(3);
        _initCircles = false;
        if (vertices.Length() > 2)
        {
            _meshData = new DataStructureOfDelaun(vertices.Length());
            Init(vertices);
        }
    }

    /// <summary>Adds the vertices to the structure and triangulates them (if there are at least three).</summary>
    public Delaun(DataStructureOfDelaun oldMesh, NCollection_Array1<MeshVertex> vertices)
    {
        _meshData = oldMesh;
        _circles = new CircleTool(vertices.Length());
        _supVert = new List<int>(3);
        _initCircles = false;
        if (vertices.Length() > 2)
        {
            Init(vertices);
        }
    }

    /// <summary>Triangulates the structure's nodes of the given indices; sorts <paramref name="vertexIndices"/>.</summary>
    public Delaun(DataStructureOfDelaun oldMesh, List<int> vertexIndices)
    {
        _meshData = oldMesh;
        _circles = new CircleTool(vertexIndices.Count, oldMesh.TakeCircles());
        _supVert = new List<int>(3);
        _initCircles = false;
        Perform(vertexIndices);
    }

    /// <summary>
    /// Triangulates the structure's nodes of the given indices with the given numbers of cells for the circles;
    /// sorts <paramref name="vertexIndices"/>.
    /// </summary>
    public Delaun(DataStructureOfDelaun oldMesh,
                           List<int> vertexIndices,
                           int cellsCountU,
                           int cellsCountV)
    {
        _meshData = oldMesh;
        _circles = new CircleTool(vertexIndices.Count, oldMesh.TakeCircles());
        _supVert = new List<int>(3);
        _initCircles = false;
        Perform(vertexIndices, cellsCountU, cellsCountV);
    }

    /// <summary>Adds the vertices to the structure and triangulates them.</summary>
    public void Init(NCollection_Array1<MeshVertex> vertices)
    {
        int lowerIdx = vertices.Lower();
        int upperIdx = vertices.Upper();
        List<int> vertexIndexes = new(vertices.Size());

        int index = lowerIdx;
        for (; index <= upperIdx; ++index)
        {
            vertexIndexes.Add(_meshData!.AddNode(vertices[index]));
        }

        Perform(vertexIndexes);
    }

    /// <summary>Sets up the circles' cells from the structure's nodes and fills them with its triangles.</summary>
    public void InitCirclesTool(int cellsCountU, int cellsCountV)
    {
        _circles.SetMaxEntries(_meshData!.MaxCircleEntries);
        Box2d box = new();
        for (int nodeIt = 1; nodeIt <= _meshData!.NodeCount; ++nodeIt)
        {
            box.Add(new Point2D(GetVertex(nodeIt).Coord()));
        }

        box.Enlarge(Precision);

        InitCirclesToolCore(box, cellsCountU, cellsCountV);

        NCollection_PackedMapOfInteger.Iterator triangleIt = new(_meshData.ElementsOfDomain());
        Span<int> nodesIndices = stackalloc int[3];
        for (; triangleIt.More(); triangleIt.Next())
        {
            Triangle triangle = _meshData.GetElement(triangleIt.Key());
            _meshData.ElementNodes(triangle, nodesIndices);
            _circles.Bind(triangleIt.Key(),
                           GetVertex(nodesIndices[0]).Coord(),
                           GetVertex(nodesIndices[1]).Coord(),
                           GetVertex(nodesIndices[2]).Coord());
        }
    }

    /// <summary>Removes the node coinciding with the vertex and remeshes the hole it leaves.</summary>
    public void RemoveVertex(in MeshVertex vertex)
    {
        SelectorOfDataStructureOfDelaun selector = new(_meshData!);
        selector.NeighboursOf(vertex);

        NCollection_IntDataMap<bool> loopEdges = new();

        // Loop on triangles to be destroyed :
        NCollection_PackedMapOfInteger.Iterator triangleIt = new(selector.Elements());
        for (; triangleIt.More(); triangleIt.Next())
        {
            DeleteTriangle(triangleIt.Key(), loopEdges);
        }

        NCollection_Sequence<B2d> boxes = new();
        NCollection_Sequence<int> polygon = new();
        int loopEdgesCount = loopEdges.Extent();
        NCollection_IntDataMap<bool>.Iterator loopEdgesIt = new(loopEdges);

        if (loopEdgesIt.More())
        {
            MeshEdge edge = GetEdge(loopEdgesIt.Key());
            int firstNode = edge.FirstNode();
            int lastNode;
            int pivotNode = edge.LastNode();
            int edgeId = loopEdgesIt.Key();

            bool isPositive = loopEdges[edgeId];
            if (!isPositive)
            {
                int tmp;
                tmp = firstNode;
                firstNode = pivotNode;
                pivotNode = tmp;

                polygon.Append(-edgeId);
            }
            else
            {
                polygon.Append(edgeId);
            }

            FillBndBox(boxes, GetVertex(firstNode), GetVertex(pivotNode));

            loopEdges.UnBind(edgeId);

            lastNode = firstNode;
            while (pivotNode != lastNode)
            {
                DataStructureOfDelaun.LinkedListOfInteger.Iterator linkIt = new(_meshData!.LinksConnectedTo(pivotNode));
                for (; linkIt.More(); linkIt.Next())
                {
                    if (linkIt.Value() != edgeId && loopEdges.IsBound(linkIt.Value()))
                    {
                        int currentNode;
                        edgeId = linkIt.Value();
                        MeshEdge edge1 = GetEdge(edgeId);

                        currentNode = edge1.LastNode();
                        if (currentNode != pivotNode)
                        {
                            currentNode = edge1.FirstNode();
                            polygon.Append(-edgeId);
                        }
                        else
                        {
                            polygon.Append(edgeId);
                        }

                        FillBndBox(boxes, GetVertex(currentNode), GetVertex(pivotNode));

                        pivotNode = currentNode;
                        loopEdges.UnBind(edgeId);
                        break;
                    }
                }

                if (loopEdgesCount <= 0)
                {
                    break;
                }

                --loopEdgesCount;
            }

            MeshPolygon(polygon, boxes);
        }
    }

    /// <summary>Inserts the structure's nodes of the given indices into the triangulation; sorts the vector.</summary>
    public void AddVertices(List<int> vertices, ProgressRange range = default)
    {
        ComparatorOfIndexedVertexOfDelaun cmp = new(_meshData!);
        Heap.make_heap(CollectionsMarshal.AsSpan(vertices), cmp);
        Heap.sort_heap(CollectionsMarshal.AsSpan(vertices), cmp);

        CreateTrianglesOnNewVertices(vertices, range);
    }

    /// <summary>Does nothing and returns false (OCCT's body is commented out).</summary>
    public bool UseEdge(int edge)
    {
        _ = edge;
        return false;
    }

    /// <summary>The data structure.</summary>
    public DataStructureOfDelaun? Result() => _meshData;

    /// <summary>Inserts the internal links into the triangulation, then adjusts it to the frontier.</summary>
    public void ProcessConstraints()
    {
        InsertInternalEdges();

        // Adjustment of meshes to boundary edges
        FrontierAdjust();
    }

    /// <summary>The frontier links.</summary>
    public NCollection_PackedMapOfInteger Frontier() => GetEdgesByType(DegreeOfFreedom.Frontier);

    /// <summary>The internal (fixed) links.</summary>
    public NCollection_PackedMapOfInteger InternalEdges() => GetEdgesByType(DegreeOfFreedom.Fixed);

    /// <summary>The links connected to at most one triangle.</summary>
    public NCollection_PackedMapOfInteger FreeEdges() => GetEdgesByType(DegreeOfFreedom.Free);

    /// <summary>The node of the given index.</summary>
    public MeshVertex GetVertex(int index) => _meshData!.GetNode(index);

    /// <summary>The link of the given index.</summary>
    public MeshEdge GetEdge(int index) => _meshData!.GetLink(index);

    /// <summary>The triangle of the given index.</summary>
    public Triangle GetTriangle(int index) => _meshData!.GetElement(index);

    /// <summary>The circumcircles of the triangles.</summary>
    public CircleTool Circles() => _circles;

    /// <summary>
    /// Whether the triangle contains the vertex. <paramref name="edgeOn"/> is the link the vertex lies on (within
    /// <paramref name="sqTolerance"/>, or near a non-free link), or 0.
    /// </summary>
    public bool Contains(int triangleId, in MeshVertex vertex, double sqTolerance, out int edgeOn)
    {
        edgeOn = 0;

        Span<int> p = stackalloc int[3];

        Triangle element = GetTriangle(triangleId);
        Triangle.Int3 e = element._edges;

        Span<MeshEdge> edges = [GetEdge(e[0]), GetEdge(e[1]), GetEdge(e[2])];

        _meshData!.ElementNodes(element, p);

        Span<Xy> points = stackalloc Xy[3];
        points[0] = GetVertex(p[0]).Coord();
        points[1] = GetVertex(p[1]).Coord();
        points[2] = GetVertex(p[2]).Coord();

        Span<Xy> vEdges = stackalloc Xy[3];
        vEdges[0] = points[1];
        vEdges[0].Subtract(points[0]);

        vEdges[1] = points[2];
        vEdges[1].Subtract(points[1]);

        vEdges[2] = points[0];
        vEdges[2].Subtract(points[2]);

        Span<double> distance = stackalloc double[3];
        Span<double> sqModulus = stackalloc double[3];

        double sqMinDist;
        int edgeOnId = 0;
        sqMinDist = CalculateDist(vEdges, points, vertex, distance, sqModulus, ref edgeOnId);
        if (sqMinDist < 0)
        {
            return false;
        }

        bool isNotFree = (edges[edgeOnId].Movability() != DegreeOfFreedom.Free);
        if (sqMinDist > sqTolerance)
        {
            if (isNotFree && distance[edgeOnId] < (sqModulus[edgeOnId] / 5.0))
            {
                edgeOn = e[edgeOnId];
            }
        }
        else if (isNotFree)
        {
            return false;
        }
        else
        {
            edgeOn = e[edgeOnId];
        }

        return (distance[0] >= 0.0 && distance[1] >= 0.0 && distance[2] >= 0.0);
    }

    /// <summary>Sets the indices of the auxiliary vertices (the super triangle's, or a third-party algorithm's).</summary>
    public void SetAuxVertices(List<int> supVert)
    {
        if (!ReferenceEquals(_supVert, supVert))
        {
            _supVert.Clear();
            _supVert.AddRange(supVert);
        }
    }

    /// <summary>Removes the triangles that use an auxiliary vertex, the links they leave, and the auxiliary vertices.</summary>
    public void RemoveAuxElements()
    {
        NCollection_IntDataMap<bool> loopEdges = new(10);

        // Destruction of triangles containing a top of the super triangle
        SelectorOfDataStructureOfDelaun selector = new(_meshData!);
        for (int supVertId = 0; supVertId < _supVert.Count; ++supVertId)
        {
            selector.NeighboursOfNode(_supVert[supVertId]);
        }

        NCollection_PackedMapOfInteger.Iterator freeTriangles = new(selector.Elements());
        for (; freeTriangles.More(); freeTriangles.Next())
        {
            DeleteTriangle(freeTriangles.Key(), loopEdges);
        }

        // All edges that remain free are removed from aLoopEdges;
        // only the boundary edges of the triangulation remain there
        NCollection_IntDataMap<bool>.Iterator freeEdges = new(loopEdges);
        for (; freeEdges.More(); freeEdges.Next())
        {
            if (_meshData!.ElementsConnectedTo(freeEdges.Key()).IsEmpty)
            {
                _meshData.RemoveLink(freeEdges.Key());
            }
        }

        // The tops of the super triangle are destroyed
        for (int supVertId = 0; supVertId < _supVert.Count; ++supVertId)
        {
            _meshData!.RemoveNode(_supVert[supVertId]);
        }
    }

    //! Sort two points in projection on vector (1,1)
    private readonly struct ComparatorOfIndexedVertexOfDelaun(DataStructureOfDelaun theDS) : Heap.ILess<int>
    {
        private readonly DataStructureOfDelaun _structure = theDS;

        public bool Less(int left, int right)
        {
            Xy aLeft = _structure.GetNode(left).Coord();
            Xy aRight = _structure.GetNode(right).Coord();
            return aLeft.X() + aLeft.Y() < aRight.X() + aRight.Y();
        }
    }

    private static void UpdateBndBox(in Xy pnt1, in Xy pnt2, ref B2d box)
    {
        box.Add(pnt1);
        box.Add(pnt2);
        box.Enlarge(Precision);
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
            private int _currentElement = theFrameStart; // Index of the current element of the frame.
            private readonly int _frameEnd = theFrameEnd; // Index of the last element + 1 of the frame.

            // Return the index of the current element and advance to the next one.
            public int Advance() => _currentElement++;

            // Check if all elements in the frame have been processed.
            public readonly bool IsEmpty => _currentElement == _frameEnd;
        }

        private readonly NCollection_LinearVector<Frame> _frames = new(); // Container of frames.

        // Adds a new frame for the given range of elements [theFrameStart, theFrameEnd).
        public void PushFrame(int frameStart, int frameEnd) => _frames.Append(new Frame(frameStart, frameEnd));

        // Returns the index of the current element of the top frame and advances to the next element. If all elements
        // of the top frame have been processed, the frame is removed from the stack. The stack must not be empty.
        public int PopElement()
        {
            ref Frame frame = ref _frames.ChangeLast();
            int elem = frame.Advance();
            if (frame.IsEmpty)
            {
                _frames.EraseLast();
            }

            return elem;
        }

        // Check if the stack is empty.
        public bool IsEmpty => _frames.IsEmpty();
    }

    private void InitCirclesToolCore(in Box2d box, int cellsCountU, int cellsCountV)
    {
        box.Get(out double minX, out double minY, out double maxX, out double maxY);
        double deltaX = maxX - minX;
        double deltaY = maxY - minY;

        int scaler = 2;
        if (_meshData!.NodeCount > 100)
        {
            scaler = 5;
        }
        else if (_meshData.NodeCount > 1000) // OCCT: unreachable (tested after > 100), so aScaler is never 7
        {
            scaler = 7;
        }

        _circles.SetMinMaxSize(new Xy(minX, minY), new Xy(maxX, maxY));
        _circles.SetCellSize(deltaX / Math.Max(cellsCountU, scaler),
                              deltaY / Math.Max(cellsCountV, scaler));

        _initCircles = true;
    }

    private void FillBndBox(NCollection_Sequence<B2d> boxes, in MeshVertex v1, in MeshVertex v2)
    {
        B2d box = new();
        UpdateBndBox(v1.Coord(), v2.Coord(), ref box);
        boxes.Append(box);
    }

    private NCollection_PackedMapOfInteger GetEdgesByType(DegreeOfFreedom edgeType)
    {
        NCollection_PackedMapOfInteger result = new();
        NCollection_PackedMapOfInteger.Iterator edgeIt = new(_meshData!.LinksOfDomain());

        for (; edgeIt.More(); edgeIt.Next())
        {
            int edge = edgeIt.Key();
            bool isToAdd = (edgeType == DegreeOfFreedom.Free)
                               ? (_meshData.ElementsConnectedTo(edge).Extent() <= 1)
                               : (GetEdge(edge).Movability() == edgeType);

            if (isToAdd)
            {
                result.Add(edge);
            }
        }

        return result;
    }

    private void Perform(List<int> vertexIndices, int cellsCountU = -1, int cellsCountV = -1)
    {
        if (vertexIndices.Count <= 2)
        {
            return;
        }

        Box2d box = new();
        int index = 0;
        int upper = vertexIndices.Count - 1;
        for (; index <= upper; ++index)
        {
            box.Add(new Point2D(GetVertex(vertexIndices[index]).Coord()));
        }

        box.Enlarge(Precision);

        InitCirclesToolCore(box, cellsCountU, cellsCountV);
        SuperMesh(box);

        ComparatorOfIndexedVertexOfDelaun cmp = new(_meshData!);
        Heap.make_heap(CollectionsMarshal.AsSpan(vertexIndices), cmp);
        Heap.sort_heap(CollectionsMarshal.AsSpan(vertexIndices), cmp);

        Compute(vertexIndices);
    }

    private void SuperMesh(in Box2d box)
    {
        box.Get(out double minX, out double minY, out double maxX, out double maxY);
        double deltaX = maxX - minX;
        double deltaY = maxY - minY;

        double deltaMin = Min(deltaX, deltaY);
        double deltaMax = Max(deltaX, deltaY);
        double delta = deltaX + deltaY;

        _supVert.Add(
            _meshData!.AddNode(new MeshVertex((minX + maxX) / 2, maxY + deltaMax, DegreeOfFreedom.Free)));

        _supVert.Add(
            _meshData.AddNode(new MeshVertex(minX - delta, minY - deltaMin, DegreeOfFreedom.Free)));

        _supVert.Add(
            _meshData.AddNode(new MeshVertex(maxX + delta, minY - deltaMin, DegreeOfFreedom.Free)));

        Span<int> e = stackalloc int[3];
        Span<bool> o = stackalloc bool[3];
        for (int nodeId = 0; nodeId < 3; ++nodeId)
        {
            int firstNode = nodeId;
            int lastNode = (nodeId + 1) % 3;
            int linkIndex = _meshData.AddLink(
                new MeshEdge(_supVert[firstNode], _supVert[lastNode], DegreeOfFreedom.Free));

            e[nodeId] = Math.Abs(linkIndex);
            o[nodeId] = (linkIndex > 0);
        }

        _supTrian = new Triangle(e, o, DegreeOfFreedom.Free);
    }

    private void DeleteTriangle(int index, NCollection_IntDataMap<bool> loopEdges)
    {
        if (_initCircles)
        {
            _circles.Delete(index);
        }

        Triangle element = GetTriangle(index);
        Triangle.Int3 e = element._edges;
        Triangle.Bool3 o = element._orientations;

        _meshData!.RemoveElement(index);

        for (int i = 0; i < 3; ++i)
        {
            if (!loopEdges.Bind(e[i], o[i]))
            {
                loopEdges.UnBind(e[i]);
                _meshData.RemoveLink(e[i]);
            }
        }
    }

    private void Compute(List<int> vertexIndexes)
    {
        // Insertion of edges of super triangles in the list of free edges:
        NCollection_IntDataMap<bool> loopEdges = new(10);
        Triangle.Int3 e = _supTrian._edges;

        loopEdges.Bind(e[0], true);
        loopEdges.Bind(e[1], true);
        loopEdges.Bind(e[2], true);

        if (vertexIndexes.Count > 0)
        {
            // Creation of 3 trianglers with the first node and the edges of the super triangle:
            int vertexIdx = 0;
            CreateTriangles(vertexIndexes[vertexIdx], loopEdges);

            // Add other nodes to the mesh
            CreateTrianglesOnNewVertices(vertexIndexes, default);
        }

        RemoveAuxElements();
    }

    private void CreateTriangles(int vertexIndex, NCollection_IntDataMap<bool> poly)
    {
        // Most triangles need neither of these (unit 11B); so nothing here is on the stack across a recursion into this method
        NCollection_List<int>? loopEdges = null;
        NCollection_List<int>? externalEdges = null;
        Xy vertexCoord = _meshData!.GetNode(vertexIndex).Coord();

        Span<int> nodes = stackalloc int[3];
        Span<int> edgesInfo = stackalloc int[3];
        Span<int> edgeIds = stackalloc int[3];
        Span<bool> edgesOri = stackalloc bool[3];
        NCollection_IntDataMap<bool>.Iterator edges = _polyIterator;
        for (edges.Initialize(poly); edges.More(); edges.Next())
        {
            int edgeId = edges.Key();
            MeshEdge edge = GetEdge(edgeId);

            bool isPositive = poly[edgeId];

            if (isPositive)
            {
                nodes[0] = edge.FirstNode();
                nodes[2] = edge.LastNode();
            }
            else
            {
                nodes[0] = edge.LastNode();
                nodes[2] = edge.FirstNode();
            }

            nodes[1] = vertexIndex;

            MeshVertex firstVertex = GetVertex(nodes[0]);
            MeshVertex lastVertex = GetVertex(nodes[2]);

            Xy edgeDir = lastVertex.Coord() - firstVertex.Coord();
            double edgeLen = edgeDir.Modulus();
            if (edgeLen < Precision)
            {
                continue;
            }

            edgeDir.SetCoord(edgeDir.X() / edgeLen, edgeDir.Y() / edgeLen);

            Xy firstLinkDir = firstVertex.Coord() - vertexCoord;
            Xy lastLinkDir = vertexCoord - lastVertex.Coord();

            double dist12 = firstLinkDir ^ edgeDir;
            double dist23 = edgeDir ^ lastLinkDir;
            if (Math.Abs(dist12) < Precision || Math.Abs(dist23) < Precision)
            {
                continue;
            }

            MeshEdge firstLink = new(nodes[1], nodes[0], DegreeOfFreedom.Free);
            MeshEdge lastLink = new(nodes[2], nodes[1], DegreeOfFreedom.Free);

            edgesInfo[0] = _meshData.AddLink(firstLink);
            edgesInfo[1] = isPositive ? edgeId : -edgeId;
            edgesInfo[2] = _meshData.AddLink(lastLink);

            bool isSensOK = (dist12 > 0.0 && dist23 > 0.0);
            if (isSensOK)
            {
                for (int triLinkIt = 0; triLinkIt < 3; ++triLinkIt)
                {
                    int edgeInfo = edgesInfo[triLinkIt];
                    edgeIds[triLinkIt] = Math.Abs(edgeInfo);
                    edgesOri[triLinkIt] = edgeInfo > 0;
                }

                AddTriangle(edgeIds, edgesOri, nodes);
            }
            else
            {
                loopEdges ??= new NCollection_List<int>();
                externalEdges ??= new NCollection_List<int>();
                if (isPositive)
                {
                    loopEdges.Append(edges.Key());
                }
                else
                {
                    loopEdges.Append(-edges.Key());
                }

                if (firstLinkDir.SquareModulus() > lastLinkDir.SquareModulus())
                {
                    externalEdges.Append(Math.Abs(edgesInfo[0]));
                }
                else
                {
                    externalEdges.Append(Math.Abs(edgesInfo[2]));
                }
            }
        }

        poly.Clear();
        while (externalEdges is not null && !externalEdges.IsEmpty())
        {
            ref readonly PairOfIndex pair =
                ref _meshData.ElementsConnectedTo(Math.Abs(externalEdges.First()));

            if (!pair.IsEmpty)
            {
                DeleteTriangle(pair.FirstIndex(), poly);
            }

            externalEdges.RemoveFirst();
        }

        for (edges.Initialize(poly); edges.More(); edges.Next())
        {
            if (_meshData.ElementsConnectedTo(edges.Key()).IsEmpty)
            {
                _meshData.RemoveLink(edges.Key());
            }
        }

        while (loopEdges is not null && !loopEdges.IsEmpty())
        {
            MeshEdge edge = GetEdge(Math.Abs(loopEdges.First()));
            if (edge.Movability() != DegreeOfFreedom.Deleted)
            {
                int edgeIdx = loopEdges.First();
                MeshLeftPolygonOf(Math.Abs(edgeIdx), (edgeIdx > 0));
            }

            loopEdges.RemoveFirst();
        }
    }

    /// <summary>Marks a circle of <see cref="CreateTrianglesOnNewVertices"/>'s list whose triangle was deleted.</summary>
    private const int RemovedCircle = int.MinValue;

    private void CreateTrianglesOnNewVertices(List<int> vertexIndexes, ProgressRange range)
    {
        _meshData!.Data().GetTolerance(out double tolU, out double tolV);
        double sqTol = tolU * tolU + tolV * tolV;

        // Insertion of nodes :

        int index = 0;
        int upper = vertexIndexes.Count - 1;
        using ProgressScope ps = new(range, "Create triangles on new vertices", upper);
        for (; index <= upper; ++index, ps.Next())
        {
            if (!ps.More())
            {
                return;
            }

            // Kerf: a long boundary comes here with no range (decision 21103)
            if ((index & 63) == 0)
            {
                Cancellation.ThrowIfRequested();
            }

            // Taken and put back, as this can be reached again from within CreateTriangles below
            NCollection_IntDataMap<bool> loopEdges = _spareLoopEdges ?? new(10);
            _spareLoopEdges = null;
            loopEdges.KeepNodes();

            int vertexIdx = vertexIndexes[index];
            MeshVertex vertex = GetVertex(vertexIdx);

            // The indexes of the circles containing the node. OCCT removes from its linked list the triangles it
            // deletes; here they are overwritten with RemovedCircle and skipped, which visits the rest in the same
            // order without moving them (unit 10J: List.RemoveAt was 7% of meshing, performance.md).
            List<int> circlesList = _circles.Select(vertex.Coord());
            int circlesLeft = circlesList.Count;

            int onEgdeId = 0, triangleId = 0;
            for (int circleIt = 0; circleIt < circlesList.Count; circleIt++)
            {
                // To add a node in the mesh it is necessary to check conditions:
                // - the node should be within the boundaries of the mesh and so in an existing triangle
                // - all adjacent triangles should belong to a component connected with this triangle
                if (Contains(circlesList[circleIt], vertex, sqTol, out onEgdeId))
                {
                    if (onEgdeId != 0 && GetEdge(onEgdeId).Movability() != DegreeOfFreedom.Free)
                    {
                        // We can skip free vertex too close to the frontier edge.
                        if (vertex.Movability() == DegreeOfFreedom.Free)
                        {
                            continue;
                        }

                        // However, we should add vertex that have neighboring frontier edges.
                    }

                    // Remove triangle even if it contains frontier edge in order
                    // to prevent appearance of incorrect configurations like free
                    // edge glued with frontier #26407
                    triangleId = circlesList[circleIt];
                    circlesList[circleIt] = RemovedCircle;
                    circlesLeft--;
                    break;
                }
            }

            if (triangleId > 0)
            {
                DeleteTriangle(triangleId, loopEdges);

                // OCCT goes back to the first circle after each deletion, and deletes the first with a free edge. No circle
                // before this cursor has one, and a deletion only changes the deleted triangle's own edges, so the only one
                // that can gain a free edge is a neighbour of that triangle: go back no further than the earliest neighbour.
                // The same circles go in the same order, without the scans (with 164 circles a node, they were most of the time).
                int cursor = 0;
                while (circlesLeft > 0 && cursor < circlesList.Count)
                {
                    int circle = circlesList[cursor];
                    if (circle == RemovedCircle)
                    {
                        cursor++;
                        continue;
                    }

                    Triangle.Int3 e = GetTriangle(circle)._edges;
                    if (!loopEdges.IsBound(e[0]) && !loopEdges.IsBound(e[1]) && !loopEdges.IsBound(e[2]))
                    {
                        cursor++;
                        continue;
                    }

                    int next = cursor + 1;
                    ReadOnlySpan<int> before = CollectionsMarshal.AsSpan(circlesList)[..cursor];
                    for (int i = 0; i < 3; ++i)
                    {
                        ref readonly PairOfIndex neighbours = ref _meshData.ElementsConnectedTo(e[i]);
                        for (int j = 1; j <= neighbours.Extent(); ++j)
                        {
                            int position = before.IndexOf(neighbours.Index(j));
                            if (position >= 0 && position < next)
                            {
                                next = position;
                            }
                        }
                    }

                    DeleteTriangle(circle, loopEdges);
                    circlesList[cursor] = RemovedCircle;
                    circlesLeft--;
                    cursor = next;
                }

                // Creation of triangles with the current node and free edges
                // and removal of these edges from the list of free edges
                CreateTriangles(vertexIdx, loopEdges);
            }

            loopEdges.Renew(10);
            _spareLoopEdges = loopEdges;
        }

        ProcessConstraints();
    }

    private void InsertInternalEdges()
    {
        NCollection_PackedMapOfInteger internalEdges = InternalEdges();

        // Destruction of triangles intersecting internal edges
        // and their replacement by makeshift triangles
        NCollection_PackedMapOfInteger.Iterator inernalEdgesIt = new(internalEdges);
        Span<bool> isGo = stackalloc bool[2];
        for (; inernalEdgesIt.More(); inernalEdgesIt.Next())
        {
            int linkIndex = inernalEdgesIt.Key();
            PairOfIndex pair = _meshData!.ElementsConnectedTo(linkIndex);

            // Check both sides of link for adjusted triangle.
            isGo[0] = true;
            isGo[1] = true;
            for (int triangleIt = 1; triangleIt <= pair.Extent(); ++triangleIt)
            {
                Triangle element = GetTriangle(pair.Index(triangleIt));
                Triangle.Int3 e = element._edges;
                Triangle.Bool3 o = element._orientations;

                for (int i = 0; i < 3; ++i)
                {
                    if (e[i] == linkIndex)
                    {
                        isGo[o[i] ? 0 : 1] = false;
                        break;
                    }
                }
            }

            if (isGo[0])
            {
                MeshLeftPolygonOf(linkIndex, true);
            }

            if (isGo[1])
            {
                MeshLeftPolygonOf(linkIndex, false);
            }
        }
    }

    private bool IsBoundToFrontier(int refNodeId, int refLinkId)
    {
        // Only ever asked whether a link is in it, so a kept HashSet in place of OCCT's map (unit 11B)
        Stack<int> linkStack = _frontierLinks ??= new Stack<int>();
        HashSet<int> visitedLinks = _frontierVisited ??= new HashSet<int>();
        linkStack.Clear();
        visitedLinks.Clear();

        linkStack.Push(refLinkId);
        while (linkStack.Count != 0)
        {
            int currentLinkId = linkStack.Pop();

            PairOfIndex pair = _meshData!.ElementsConnectedTo(currentLinkId);
            if (pair.IsEmpty)
            {
                return false;
            }

            int nbElements = pair.Extent();
            for (int elemIt = 1; elemIt <= nbElements; ++elemIt)
            {
                int triId = pair.Index(elemIt);
                if (triId < 0)
                {
                    continue;
                }

                Triangle element = GetTriangle(triId);
                Triangle.Int3 edges = element._edges;

                for (int edgeIt = 0; edgeIt < 3; ++edgeIt)
                {
                    int edgeId = edges[edgeIt];
                    if (edgeId == currentLinkId)
                    {
                        continue;
                    }

                    MeshEdge edge = GetEdge(edgeId);
                    if (edge.FirstNode() != refNodeId && edge.LastNode() != refNodeId)
                    {
                        continue;
                    }

                    if (edge.Movability() != DegreeOfFreedom.Free)
                    {
                        return true;
                    }

                    if (visitedLinks.Add(edgeId))
                    {
                        linkStack.Push(edgeId);
                    }
                }
            }
        }

        return false;
    }

    private void CleanupMesh()
    {
        Span<int> v = stackalloc int[3];
        for (;;)
        {
            NCollection_IntDataMap<bool> loopEdges = new(10);
            NCollection_PackedMapOfInteger delTriangles = new();

            NCollection_PackedMapOfInteger freeEdges = FreeEdges();
            NCollection_PackedMapOfInteger.Iterator freeEdgesIt = new(freeEdges);
            for (; freeEdgesIt.More(); freeEdgesIt.Next())
            {
                int freeEdgeId = freeEdgesIt.Key();
                MeshEdge edge = GetEdge(freeEdgeId);
                if (edge.Movability() == DegreeOfFreedom.Frontier)
                {
                    continue;
                }

                PairOfIndex pair = _meshData!.ElementsConnectedTo(freeEdgeId);
                if (pair.IsEmpty)
                {
                    loopEdges.Bind(freeEdgeId, true);
                    continue;
                }

                int triId = pair.FirstIndex();

                // Check that the connected triangle is not surrounded by another triangles
                Triangle element = GetTriangle(triId);
                Triangle.Int3 edges = element._edges;

                bool isCanNotBeRemoved = true;
                for (int curEdgeIdx = 0; curEdgeIdx < 3; ++curEdgeIdx)
                {
                    if (edges[curEdgeIdx] != freeEdgeId)
                    {
                        continue;
                    }

                    for (int otherEdgeIt = 1; otherEdgeIt <= 2 && isCanNotBeRemoved; ++otherEdgeIt)
                    {
                        int otherEdgeId = (curEdgeIdx + otherEdgeIt) % 3;
                        PairOfIndex otherEdgePair =
                            _meshData.ElementsConnectedTo(edges[otherEdgeId]);

                        if (otherEdgePair.Extent() < 2)
                        {
                            isCanNotBeRemoved = false;
                        }
                        else
                        {
                            for (int triIdx = 1; triIdx <= otherEdgePair.Extent() && isCanNotBeRemoved;
                                 ++triIdx)
                            {
                                if (otherEdgePair.Index(triIdx) == triId)
                                {
                                    continue;
                                }

                                Triangle curTriangle = GetTriangle(otherEdgePair.Index(triIdx));
                                _meshData.ElementNodes(curTriangle, v);
                                for (int nodeIdx = 0; nodeIdx < 3 && isCanNotBeRemoved; ++nodeIdx)
                                {
                                    if (IsSupVertex(v[nodeIdx]))
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
                for (int linkNodeIt = 0; linkNodeIt < 2; ++linkNodeIt)
                {
                    isConnected[linkNodeIt] =
                        IsBoundToFrontier((linkNodeIt == 0) ? edge.FirstNode() : edge.LastNode(),
                                          freeEdgeId);
                }

                if (!isConnected[0] || !isConnected[1])
                {
                    delTriangles.Add(triId);
                }
            }

            // Destruction of triangles :
            int deletedTrianglesNb = 0;
            NCollection_PackedMapOfInteger.Iterator delTrianglesIt = new(delTriangles);
            for (; delTrianglesIt.More(); delTrianglesIt.Next())
            {
                DeleteTriangle(delTrianglesIt.Key(), loopEdges);
                deletedTrianglesNb++;
            }

            // Destruction of remaining hanging edges
            NCollection_IntDataMap<bool>.Iterator loopEdgesIt = new(loopEdges);
            for (; loopEdgesIt.More(); loopEdgesIt.Next())
            {
                if (_meshData!.ElementsConnectedTo(loopEdgesIt.Key()).IsEmpty)
                {
                    _meshData.RemoveLink(loopEdgesIt.Key());
                }
            }

            if (deletedTrianglesNb == 0)
            {
                break;
            }
        }
    }

    private void FrontierAdjust()
    {
        NCollection_PackedMapOfInteger frontier = Frontier();

        List<int> failedFrontiers = new();
        NCollection_IntDataMap<bool> loopEdges = new(10);
        NCollection_PackedMapOfInteger intFrontierEdges = new();

        for (int pass = 1; pass <= 2; ++pass)
        {
            // 1 pass): find external triangles on boundary edges;
            // 2 pass): find external triangles on boundary edges appeared
            //          during triangles replacement.

            NCollection_PackedMapOfInteger.Iterator frontierIt = new(frontier);
            for (; frontierIt.More(); frontierIt.Next())
            {
                int frontierId = frontierIt.Key();
                PairOfIndex pair = _meshData!.ElementsConnectedTo(frontierId);
                int nbElem = pair.Extent();
                for (int elemIt = 1; elemIt <= nbElem; ++elemIt)
                {
                    int priorElemId = pair.Index(elemIt);
                    if (priorElemId < 0)
                    {
                        continue;
                    }

                    Triangle element = GetTriangle(priorElemId);
                    Triangle.Int3 e = element._edges;
                    Triangle.Bool3 o = element._orientations;

                    bool isTriangleFound = false;
                    for (int n = 0; n < 3; ++n)
                    {
                        if (frontierId == e[n] && !o[n])
                        {
                            // Destruction  of external triangles on boundary edges
                            isTriangleFound = true;
                            DeleteTriangle(priorElemId, loopEdges);
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
            NCollection_IntDataMap<bool>.Iterator loopEdgesIt = new(loopEdges);
            for (; loopEdgesIt.More(); loopEdgesIt.Next())
            {
                int loopEdgeId = loopEdgesIt.Key();
                if (_meshData!.ElementsConnectedTo(loopEdgeId).IsEmpty)
                {
                    _meshData.RemoveLink(loopEdgeId);
                }
            }

            // destruction of triangles crossing the boundary edges and
            // their replacement by makeshift triangles
            for (frontierIt.Reset(); frontierIt.More(); frontierIt.Next())
            {
                int frontierId = frontierIt.Key();
                if (!_meshData!.ElementsConnectedTo(frontierId).IsEmpty)
                {
                    continue;
                }

                bool isSuccess = MeshLeftPolygonOf(frontierId, true, intFrontierEdges);

                if (pass == 2 && !isSuccess)
                {
                    failedFrontiers.Add(frontierId);
                }
            }
        }

        CleanupMesh();

        // When the mesh has been cleaned up, try to process frontier edges
        // once again to fill the possible gaps that might be occurred in case of "saw" -
        // situation when frontier edge has a triangle at a right side, but its free
        // links cross another frontieres  and meshLeftPolygonOf itself can't collect
        // a closed polygon.
        for (int failedFrontiersIt = 0; failedFrontiersIt < failedFrontiers.Count; ++failedFrontiersIt)
        {
            int frontierId = failedFrontiers[failedFrontiersIt];
            if (!_meshData!.ElementsConnectedTo(frontierId).IsEmpty)
            {
                continue;
            }

            MeshLeftPolygonOf(frontierId, true, intFrontierEdges);
        }
    }

    private bool MeshLeftPolygonOf(int startEdgeId, bool isForward, NCollection_PackedMapOfInteger? skipped = null)
    {
        if (skipped is not null && skipped.Contains(startEdgeId))
        {
            return true;
        }

        MeshEdge refEdge = GetEdge(startEdgeId);

        NCollection_Sequence<int> polygon = new();
        int startNode, pivotNode;
        if (isForward)
        {
            polygon.Append(startEdgeId);
            startNode = refEdge.FirstNode();
            pivotNode = refEdge.LastNode();
        }
        else
        {
            polygon.Append(-startEdgeId);
            startNode = refEdge.LastNode();
            pivotNode = refEdge.FirstNode();
        }

        MeshVertex startEdgeVertexS = GetVertex(startNode);
        MeshVertex pivotVertex = GetVertex(pivotNode);

        Vector2D refLinkDir = new(pivotVertex.Coord() - startEdgeVertexS.Coord());

        if (refLinkDir.SquareMagnitude < Precision2)
        {
            return true;
        }

        // Auxiliary structures.
        // Bounding boxes of polygon links to be used for preliminary
        // analysis of intersections
        NCollection_Sequence<B2d> boxes = new();
        FillBndBox(boxes, startEdgeVertexS, pivotVertex);

        // Hanging ends
        NCollection_PackedMapOfInteger deadLinks = new();

        // Links are temporarily excluded from consideration
        NCollection_PackedMapOfInteger leprousLinks = new();
        leprousLinks.Add(startEdgeId);

        bool isSkipLeprous = true;
        int firstNode = startNode;
        while (pivotNode != firstNode)
        {
            B2d nextLinkBndBox = new();
            Vector2D nextLinkDir = new();
            int nextPivotNode = 0;

            int nextLinkId = FindNextPolygonLink(firstNode,
                                                  pivotNode,
                                                  pivotVertex,
                                                  refLinkDir,
                                                  boxes,
                                                  polygon,
                                                  skipped,
                                                  isSkipLeprous,
                                                  leprousLinks,
                                                  deadLinks,
                                                  ref nextPivotNode,
                                                  ref nextLinkDir,
                                                  ref nextLinkBndBox);

            if (nextLinkId != 0)
            {
                startNode = pivotNode;
                refLinkDir = nextLinkDir;

                pivotNode = nextPivotNode;
                pivotVertex = GetVertex(nextPivotNode);

                boxes.Append(nextLinkBndBox);
                polygon.Append(nextLinkId);

                isSkipLeprous = true;
            }
            else
            {
                // Nothing to do
                if (polygon.Length() == 1)
                {
                    return false;
                }

                // Return to the previous point
                int deadLinkId = Math.Abs(polygon.Last());
                deadLinks.Add(deadLinkId);

                leprousLinks.Remove(deadLinkId);
                polygon.Remove(polygon.Length());
                boxes.Remove(boxes.Length());

                int prevLinkInfo = polygon.Last();
                MeshEdge prevLink = GetEdge(Math.Abs(prevLinkInfo));

                if (prevLinkInfo > 0)
                {
                    startNode = prevLink.FirstNode();
                    pivotNode = prevLink.LastNode();
                }
                else
                {
                    startNode = prevLink.LastNode();
                    pivotNode = prevLink.FirstNode();
                }

                pivotVertex = GetVertex(pivotNode);
                refLinkDir = new Vector2D(pivotVertex.Coord() - GetVertex(startNode).Coord());

                isSkipLeprous = false;
            }
        }

        if (polygon.Length() < 3)
        {
            return false;
        }

        CleanupPolygon(polygon, boxes);
        MeshPolygon(polygon, boxes, skipped);

        return true;
    }

    private int FindNextPolygonLink(int firstNode,
                                    int pivotNode,
                                    in MeshVertex pivotVertex,
                                    in Vector2D refLinkDir,
                                    NCollection_Sequence<B2d> boxes,
                                    NCollection_Sequence<int> polygon,
                                    NCollection_PackedMapOfInteger? skipped,
                                    bool isSkipLeprous,
                                    NCollection_PackedMapOfInteger leprousLinks,
                                    NCollection_PackedMapOfInteger deadLinks,
                                    ref int nextPivotNode,
                                    ref Vector2D nextLinkDir,
                                    ref B2d nextLinkBndBox)
    {
        // Find the next link having the greatest angle
        // respect to a direction of a reference one
        double maxAngle = RealFirst();

        int nextLinkId = 0;
        DataStructureOfDelaun.LinkedListOfInteger.Iterator linkIt = new(_meshData!.LinksConnectedTo(pivotNode));
        for (; linkIt.More(); linkIt.Next())
        {
            int neighbourLinkInfo = linkIt.Value();
            int neighbourLinkId = Math.Abs(neighbourLinkInfo);

            if (deadLinks.Contains(neighbourLinkId)
                || (skipped is not null && skipped.Contains(neighbourLinkId)))
            {
                continue;
            }

            bool isLeprous = leprousLinks.Contains(neighbourLinkId);
            if (isSkipLeprous && isLeprous)
            {
                continue;
            }

            MeshEdge neighbourLink = GetEdge(neighbourLinkId);

            // Determine whether the link belongs to the mesh
            if (neighbourLink.Movability() == DegreeOfFreedom.Free
                && _meshData.ElementsConnectedTo(neighbourLinkInfo).IsEmpty)
            {
                deadLinks.Add(neighbourLinkId);
                continue;
            }

            int otherNode = neighbourLink.FirstNode();
            if (otherNode == pivotNode)
            {
                otherNode = neighbourLink.LastNode();
            }

            Vector2D curLinkDir = new(GetVertex(otherNode).Coord() - pivotVertex.Coord());

            if (curLinkDir.SquareMagnitude < Precision2)
            {
                deadLinks.Add(neighbourLinkId);
                continue;
            }

            if (!isLeprous)
            {
                leprousLinks.Add(neighbourLinkId);
            }

            double angle = refLinkDir.AngleTo(curLinkDir);
            bool isFrontier = (neighbourLink.Movability() == DegreeOfFreedom.Frontier);

            bool isCheckPointOnEdge = true;
            if (isFrontier)
            {
                if (Math.Abs(Math.Abs(angle) - M_PI) < Kerf.Engine.Foundation.Precision.Angular())
                {
                    // Glued constrains - don't check intersection
                    isCheckPointOnEdge = false;
                    angle = Math.Abs(angle);
                }
            }

            if (angle <= maxAngle)
            {
                continue;
            }

            bool isCheckEndPoints = (otherNode != firstNode);

            B2d box = new();
            bool isNotIntersect = CheckIntersection(neighbourLink,
                                                    polygon,
                                                    boxes,
                                                    isCheckEndPoints,
                                                    isCheckPointOnEdge,
                                                    true,
                                                    ref box);

            if (isNotIntersect)
            {
                maxAngle = angle;

                nextLinkDir = curLinkDir;
                nextPivotNode = otherNode;
                nextLinkBndBox = box;

                nextLinkId =
                    (neighbourLink.FirstNode() == pivotNode) ? neighbourLinkId : -neighbourLinkId;
            }
        }

        return nextLinkId;
    }

    private bool CheckIntersection(in MeshEdge link,
                                   NCollection_Sequence<int> polygon,
                                   NCollection_Sequence<B2d> polyBoxes,
                                   bool isConsiderEndPointTouch,
                                   bool isConsiderPointOnEdge,
                                   bool isSkipLastEdge,
                                   ref B2d linkBndBox)
    {
        UpdateBndBox(GetVertex(link.FirstNode()).Coord(),
                     GetVertex(link.LastNode()).Coord(),
                     ref linkBndBox);

        int polyLen = polygon.Length();
        // Don't check intersection with the last link
        if (isSkipLastEdge)
        {
            --polyLen;
        }

        bool isFrontier = (link.Movability() == DegreeOfFreedom.Frontier);

        for (int polyIt = 1; polyIt <= polyLen; ++polyIt)
        {
            if (!linkBndBox.IsOut(polyBoxes.Value(polyIt)))
            {
                // intersection is possible...
                int polyLinkId = Math.Abs(polygon[polyIt]);
                MeshEdge polyLink = GetEdge(polyLinkId);

                // skip intersections between frontier edges
                if (polyLink.Movability() == DegreeOfFreedom.Frontier && isFrontier)
                {
                    continue;
                }

                Point2D intPnt = new();
                GeomTool.IntFlag intFlag =
                    IntSegSeg(link, polyLink, isConsiderEndPointTouch, isConsiderPointOnEdge, ref intPnt);

                if (intFlag != NoIntersection)
                {
                    return false;
                }
            }
        }

        // Ok, no intersection
        return true;
    }

    private void AddTriangle(ReadOnlySpan<int> edgesId, ReadOnlySpan<bool> edgesOri, ReadOnlySpan<int> nodesId)
    {
        int newTriangleId =
            _meshData!.AddElement(new Triangle(edgesId, edgesOri, DegreeOfFreedom.Free));

        bool isAdded = true;
        if (_initCircles)
        {
            isAdded = _circles.Bind(newTriangleId,
                                     GetVertex(nodesId[0]).Coord(),
                                     GetVertex(nodesId[1]).Coord(),
                                     GetVertex(nodesId[2]).Coord());
        }

        if (!isAdded)
        {
            _meshData.RemoveElement(newTriangleId);
        }
    }

    private void CleanupPolygon(NCollection_Sequence<int> polygon, NCollection_Sequence<B2d> polyBoxes)
    {
        int polyLen = polygon.Length();
        if (polyLen < 3)
        {
            return;
        }

        NCollection_IntDataMap<bool> loopEdges = new(10);
        NCollection_PackedMapOfInteger ignoredEdges = new();
        NCollection_PackedMapOfInteger polyVerticesFindMap = new();
        List<int> polyVertices = new();
        // Collect boundary vertices of the polygon
        for (int polyIt = 1; polyIt <= polyLen; ++polyIt)
        {
            int polyEdgeInfo = polygon[polyIt];
            int polyEdgeId = Math.Abs(polyEdgeInfo);
            ignoredEdges.Add(polyEdgeId);

            bool isForward = (polyEdgeInfo > 0);
            PairOfIndex pair = _meshData!.ElementsConnectedTo(polyEdgeId);

            int elemIt = 1;
            for (; elemIt <= pair.Extent(); ++elemIt)
            {
                int elemId = pair.Index(elemIt);
                if (elemId < 0)
                {
                    continue;
                }

                Triangle element = GetTriangle(elemId);
                Triangle.Int3 edges = element._edges;
                Triangle.Bool3 edgesOri = element._orientations;

                bool isTriangleFound = false;
                for (int edgeIt = 0; edgeIt < 3; ++edgeIt)
                {
                    if (edges[edgeIt] == polyEdgeId && edgesOri[edgeIt] == isForward)
                    {
                        isTriangleFound = true;
                        DeleteTriangle(elemId, loopEdges);
                        break;
                    }
                }

                if (isTriangleFound)
                {
                    break;
                }
            }

            // Skip a neighbor link to extract unique vertices each time
            if (polyIt % 2 != 0)
            {
                MeshEdge polyEdge = GetEdge(polyEdgeId);
                int firstVertex = polyEdge.FirstNode();
                int lastVertex = polyEdge.LastNode();

                polyVerticesFindMap.Add(firstVertex);
                polyVerticesFindMap.Add(lastVertex);

                if (polyEdgeInfo > 0)
                {
                    polyVertices.Add(firstVertex);
                    polyVertices.Add(lastVertex);
                }
                else
                {
                    polyVertices.Add(lastVertex);
                    polyVertices.Add(firstVertex);
                }
            }
        }

        // Make closed sequence
        if (polyVertices[0] != polyVertices[^1])
        {
            polyVertices.Add(polyVertices[0]);
        }

        NCollection_PackedMapOfInteger survivedLinks = new NCollection_PackedMapOfInteger().Assign(ignoredEdges);

        for (int polyVertIt = 0; polyVertIt < polyVertices.Count - 1; ++polyVertIt)
        {
            StackOfFrames stackFames = new();
            List<int> stackData = new();
            for (int currentVictim = polyVertices[polyVertIt]; currentVictim != -1;
                 currentVictim = stackFames.IsEmpty ? -1 : stackData[stackFames.PopElement()])
            {
                int prevStackDataSize = stackData.Count;
                KillTrianglesAroundVertex(currentVictim,
                                          polyVertices,
                                          polyVerticesFindMap,
                                          polygon,
                                          polyBoxes,
                                          survivedLinks,
                                          loopEdges,
                                          stackData);
                int newStackDataSize = stackData.Count;
                if (newStackDataSize > prevStackDataSize)
                {
                    stackFames.PushFrame(prevStackDataSize, newStackDataSize);
                }
            }
        }

        NCollection_IntDataMap<bool>.Iterator loopEdgesIt = new(loopEdges);
        for (; loopEdgesIt.More(); loopEdgesIt.Next())
        {
            int loopEdgeId = loopEdgesIt.Key();
            if (ignoredEdges.Contains(loopEdgeId))
            {
                continue;
            }

            if (_meshData!.ElementsConnectedTo(loopEdgeId).IsEmpty)
            {
                _meshData.RemoveLink(loopEdgesIt.Key());
            }
        }
    }

    private void KillTrianglesAroundVertex(int zombieNodeId,
                                           List<int> polyVertices,
                                           NCollection_PackedMapOfInteger polyVerticesFindMap,
                                           NCollection_Sequence<int> polygon,
                                           NCollection_Sequence<B2d> polyBoxes,
                                           NCollection_PackedMapOfInteger survivedLinks,
                                           NCollection_IntDataMap<bool> loopEdges,
                                           List<int> victimNodes)
    {
        // The list is walked while links are removed from it (OCCT's linked-list semantics, decision 541).
        DataStructureOfDelaun.LinkedListOfInteger.Iterator neighborsIt = new(_meshData!.LinksConnectedTo(zombieNodeId));

        // Try to infect neighbor nodes
        for (; neighborsIt.More(); neighborsIt.Next())
        {
            int neighborLinkId = neighborsIt.Value();
            if (survivedLinks.Contains(neighborLinkId))
            {
                continue;
            }

            MeshEdge neighborLink = GetEdge(neighborLinkId);
            if (neighborLink.Movability() == DegreeOfFreedom.Frontier)
            {
                // Though, if it lies onto the polygon boundary -
                // take its triangles
                B2d box = new();
                bool isNotIntersect =
                    CheckIntersection(neighborLink, polygon, polyBoxes, false, true, false, ref box);

                if (isNotIntersect)
                {
                    // Don't touch it
                    survivedLinks.Add(neighborLinkId);
                    continue;
                }
            }
            else
            {
                int otherNode = neighborLink.FirstNode();
                if (otherNode == zombieNodeId)
                {
                    otherNode = neighborLink.LastNode();
                }

                // Possible sacrifice
                if (!polyVerticesFindMap.Contains(otherNode))
                {
                    if (IsVertexInsidePolygon(otherNode, polyVertices))
                    {
                        // Got you!
                        victimNodes.Add(otherNode);
                    }
                    else
                    {
                        // Lucky. But it may intersect the polygon boundary.
                        // Let's check it!
                        KillTrianglesOnIntersectingLinks(neighborLinkId,
                                                         neighborLink,
                                                         otherNode,
                                                         polygon,
                                                         polyBoxes,
                                                         survivedLinks,
                                                         loopEdges);

                        continue;
                    }
                }
            }

            // Add link to the survivors to avoid cycling
            survivedLinks.Add(neighborLinkId);
            KillLinkTriangles(neighborLinkId, loopEdges);
        }
    }

    private bool IsVertexInsidePolygon(int vertexId, List<int> polygonVertices)
    {
        int polyLen = polygonVertices.Count;
        if (polyLen < 3)
        {
            return false;
        }

        Xy centerPointXY = GetVertex(vertexId).Coord();

        MeshVertex firstVertex = GetVertex(polygonVertices[0]);
        Vector2D prevVertexDir = new(firstVertex.Coord() - centerPointXY);
        if (prevVertexDir.SquareMagnitude < Precision2)
        {
            return true;
        }

        double totalAng = 0.0;
        for (int polyIt = 1; polyIt < polyLen; ++polyIt)
        {
            MeshVertex polyVertex = GetVertex(polygonVertices[polyIt]);

            Vector2D curVertexDir = new(polyVertex.Coord() - centerPointXY);
            if (curVertexDir.SquareMagnitude < Precision2)
            {
                return true;
            }

            totalAng += curVertexDir.AngleTo(prevVertexDir);
            prevVertexDir = curVertexDir;
        }

        return Math.Abs(Math.Abs(totalAng) - Angle2PI) <= Kerf.Engine.Foundation.Precision.Angular();
    }

    private void KillTrianglesOnIntersectingLinks(int linkToCheckId,
                                                  in MeshEdge linkToCheck,
                                                  int endPoint,
                                                  NCollection_Sequence<int> polygon,
                                                  NCollection_Sequence<B2d> polyBoxes,
                                                  NCollection_PackedMapOfInteger survivedLinks,
                                                  NCollection_IntDataMap<bool> loopEdges)
    {
        if (survivedLinks.Contains(linkToCheckId))
        {
            return;
        }

        B2d box = new();
        bool isNotIntersect =
            CheckIntersection(linkToCheck, polygon, polyBoxes, false, false, false, ref box);

        survivedLinks.Add(linkToCheckId);

        if (isNotIntersect)
        {
            return;
        }

        KillLinkTriangles(linkToCheckId, loopEdges);

        DataStructureOfDelaun.LinkedListOfInteger.Iterator neighborsIt = new(_meshData!.LinksConnectedTo(endPoint));

        for (; neighborsIt.More(); neighborsIt.Next())
        {
            int neighborLinkId = neighborsIt.Value();
            MeshEdge neighborLink = GetEdge(neighborLinkId);
            int otherNode = neighborLink.FirstNode();
            if (otherNode == endPoint)
            {
                otherNode = neighborLink.LastNode();
            }

            KillTrianglesOnIntersectingLinks(neighborLinkId,
                                             neighborLink,
                                             otherNode,
                                             polygon,
                                             polyBoxes,
                                             survivedLinks,
                                             loopEdges);
        }
    }

    private void KillLinkTriangles(int linkId, NCollection_IntDataMap<bool> loopEdges)
    {
        // A reference: deleting a triangle removes it from the pair, so FirstIndex() moves on (as OCCT's const&).
        ref readonly PairOfIndex pair = ref _meshData!.ElementsConnectedTo(linkId);

        int elemNb = pair.Extent();
        for (int pairIt = 1; pairIt <= elemNb; ++pairIt)
        {
            int elemId = pair.FirstIndex();
            if (elemId < 0)
            {
                continue;
            }

            DeleteTriangle(elemId, loopEdges);
        }
    }

    private static void GetOrientedNodes(in MeshEdge edge, bool isForward, Span<int> nodes)
    {
        if (isForward)
        {
            nodes[0] = edge.FirstNode();
            nodes[1] = edge.LastNode();
        }
        else
        {
            nodes[0] = edge.LastNode();
            nodes[1] = edge.FirstNode();
        }
    }

    private void ProcessLoop(int linkFrom,
                             int linkTo,
                             NCollection_Sequence<int> polygon,
                             NCollection_Sequence<B2d> polyBoxes)
    {
        int nbOfLinksInLoop = linkTo - linkFrom - 1;
        if (nbOfLinksInLoop < 3)
        {
            return;
        }

        NCollection_Sequence<int> aPolygon = new();
        NCollection_Sequence<B2d> aPolyBoxes = new();
        for (; nbOfLinksInLoop > 0; --nbOfLinksInLoop)
        {
            int loopLinkIndex = linkFrom + nbOfLinksInLoop;
            aPolygon.Prepend(polygon[loopLinkIndex]);
            aPolyBoxes.Prepend(polyBoxes[loopLinkIndex]);
        }

        MeshPolygon(aPolygon, aPolyBoxes);
    }

    private int CreateAndReplacePolygonLink(ReadOnlySpan<int> nodes,
                                            ReadOnlySpan<Point2D> pnts,
                                            int rootIndex,
                                            ReplaceFlag replaceFlag,
                                            NCollection_Sequence<int> polygon,
                                            NCollection_Sequence<B2d> polyBoxes)
    {
        int newEdgeId = _meshData!.AddLink(new MeshEdge(nodes[0], nodes[1], DegreeOfFreedom.Free));

        B2d newBox = new();
        UpdateBndBox(pnts[0].Coord(), pnts[1].Coord(), ref newBox);

        switch (replaceFlag)
        {
            case ReplaceFlag.Replace:
                polygon.SetValue(rootIndex, newEdgeId);
                polyBoxes.SetValue(rootIndex, newBox);
                break;

            case ReplaceFlag.InsertAfter:
                polygon.InsertAfter(rootIndex, newEdgeId);
                polyBoxes.InsertAfter(rootIndex, newBox);
                break;

            case ReplaceFlag.InsertBefore:
                polygon.InsertBefore(rootIndex, newEdgeId);
                polyBoxes.InsertBefore(rootIndex, newBox);
                break;
        }

        return newEdgeId;
    }

    // OCCT's NCollection_Sequence::Remove(from, to) in a release build: its range check is compiled out, and with
    // to == from - 1 it removes nothing. The port's Remove checks, so that case is handled here.
    private static void RemoveRange<T>(NCollection_Sequence<T> sequence, int fromIndex, int toIndex)
    {
        if (toIndex == fromIndex - 1)
        {
            return;
        }

        sequence.Remove(fromIndex, toIndex);
    }

    private void MeshPolygon(NCollection_Sequence<int> polygon,
                             NCollection_Sequence<B2d> polyBoxes,
                             NCollection_PackedMapOfInteger? skipped = null)
    {
        // Check is the source polygon elementary
        if (MeshElementaryPolygon(polygon))
        {
            return;
        }

        // Check and correct boundary edges
        int polyLen = polygon.Length();
        double polyArea = Math.Abs(PolyArea(polygon, 1, polyLen));
        double smallLoopArea = 0.001 * polyArea;
        Span<int> curNodes = stackalloc int[2];
        Span<Point2D> curPnts = stackalloc Point2D[2];
        Span<int> nextNodes = stackalloc int[2];
        Span<Point2D> nextPnts = stackalloc Point2D[2];
        Span<int> splitLink = stackalloc int[3];
        Span<Point2D> splitPnts = stackalloc Point2D[3];
        Span<int> splitLinkNodes = stackalloc int[2];
        Span<Point2D> splitLinkPnts = stackalloc Point2D[2];
        for (int polyIt = 1; polyIt < polyLen; ++polyIt)
        {
            int curEdgeInfo = polygon[polyIt];
            int curEdgeId = Math.Abs(curEdgeInfo);
            MeshEdge curEdge = GetEdge(curEdgeId);
            if (curEdge.Movability() != DegreeOfFreedom.Frontier)
            {
                continue;
            }

            GetOrientedNodes(curEdge, curEdgeInfo > 0, curNodes);

            curPnts[0] = new Point2D(GetVertex(curNodes[0]).Coord());
            curPnts[1] = new Point2D(GetVertex(curNodes[1]).Coord());

            Vector2D curVec = new(curPnts[0], curPnts[1]);

            // check further links
            int nextPolyIt = polyIt + 1;
            for (; nextPolyIt <= polyLen; ++nextPolyIt)
            {
                int nextEdgeInfo = polygon[nextPolyIt];
                int nextEdgeId = Math.Abs(nextEdgeInfo);
                MeshEdge nextEdge = GetEdge(nextEdgeId);
                if (nextEdge.Movability() != DegreeOfFreedom.Frontier)
                {
                    continue;
                }

                GetOrientedNodes(nextEdge, nextEdgeInfo > 0, nextNodes);

                nextPnts[0] = new Point2D(GetVertex(nextNodes[0]).Coord());
                nextPnts[1] = new Point2D(GetVertex(nextNodes[1]).Coord());

                Point2D intPnt = new();
                GeomTool.IntFlag intFlag = IntSegSeg(curEdge, nextEdge, false, true, ref intPnt);

                if (intFlag == NoIntersection)
                {
                    continue;
                }

                bool isRemoveFromFirst = false;
                bool isAddReplacingEdge = true;
                int indexToRemoveTo = nextPolyIt;
                if (intFlag == Cross)
                {
                    double loopArea = PolyArea(polygon, polyIt + 1, nextPolyIt);
                    Vector2D vec1 = new(intPnt, curPnts[1]);
                    Vector2D vec2 = new(intPnt, nextPnts[0]);

                    loopArea += (vec1 ^ vec2) / 2.0;
                    if (Math.Abs(loopArea) > smallLoopArea)
                    {
                        nextNodes[1] = curNodes[0];
                        nextPnts[1] = curPnts[0];

                        nextEdgeId = Math.Abs(CreateAndReplacePolygonLink(nextNodes,
                                                                           nextPnts,
                                                                           nextPolyIt,
                                                                           ReplaceFlag.Replace,
                                                                           polygon,
                                                                           polyBoxes));

                        ProcessLoop(polyIt, nextPolyIt, polygon, polyBoxes);
                        return;
                    }

                    double dist1 = intPnt.SquareDistanceTo(nextPnts[0]);
                    double dist2 = intPnt.SquareDistanceTo(nextPnts[1]);

                    // Choose node with lower distance
                    bool isCloseToStart = (dist1 < dist2);
                    int endPointIndex = isCloseToStart ? 0 : 1;
                    curNodes[1] = nextNodes[endPointIndex];
                    curPnts[1] = nextPnts[endPointIndex];

                    if (isCloseToStart)
                    {
                        --indexToRemoveTo;
                    }

                    // In this context only intersections between frontier edges
                    // are possible. If intersection between edges of different
                    // types occurred - treat this case as invalid (i.e. result
                    // might not reflect the expectations).
                    if (skipped is not null)
                    {
                        int skippedLinkIt = polyIt;
                        for (; skippedLinkIt <= indexToRemoveTo; ++skippedLinkIt)
                        {
                            skipped.Add(Math.Abs(polygon[skippedLinkIt]));
                        }
                    }
                }
                else if (intFlag == PointOnSegment)
                {
                    // Identify chopping link
                    bool isFirstChopping = false;
                    int checkPointIt = 0;
                    for (; checkPointIt < 2; ++checkPointIt)
                    {
                        Point2D refPoint = curPnts[checkPointIt];
                        // Check is second link touches the first one
                        Vector2D vec1 = new(refPoint, nextPnts[0]);
                        Vector2D vec2 = new(refPoint, nextPnts[1]);
                        if (Math.Abs(vec1 ^ vec2) < Precision)
                        {
                            isFirstChopping = true;
                            break;
                        }
                    }

                    if (isFirstChopping)
                    {
                        // Split second link
                        isAddReplacingEdge = false;
                        isRemoveFromFirst = (checkPointIt == 0);

                        splitLink[0] = nextNodes[0];
                        splitLink[1] = curNodes[checkPointIt];
                        splitLink[2] = nextNodes[1];

                        splitPnts[0] = nextPnts[0];
                        splitPnts[1] = curPnts[checkPointIt];
                        splitPnts[2] = nextPnts[1];

                        int splitLinkIt = 0;
                        for (; splitLinkIt < 2; ++splitLinkIt)
                        {
                            CreateAndReplacePolygonLink(splitLink[splitLinkIt..],
                                                        splitPnts[splitLinkIt..],
                                                        nextPolyIt,
                                                        (splitLinkIt == 0) ? ReplaceFlag.Replace
                                                                            : ReplaceFlag.InsertAfter,
                                                        polygon,
                                                        polyBoxes);
                        }

                        ProcessLoop(polyIt + checkPointIt, indexToRemoveTo, polygon, polyBoxes);
                    }
                    else
                    {
                        // Split first link
                        splitLinkNodes[0] = nextNodes[1];
                        splitLinkNodes[1] = curNodes[1];

                        splitLinkPnts[0] = nextPnts[1];
                        splitLinkPnts[1] = curPnts[1];
                        CreateAndReplacePolygonLink(splitLinkNodes,
                                                    splitLinkPnts,
                                                    polyIt,
                                                    ReplaceFlag.InsertAfter,
                                                    polygon,
                                                    polyBoxes);

                        curNodes[1] = nextNodes[1];
                        curPnts[1] = nextPnts[1];
                        ++indexToRemoveTo;

                        ProcessLoop(polyIt + 1, indexToRemoveTo, polygon, polyBoxes);
                    }
                }
                else if (intFlag == Glued)
                {
                    if (curNodes[1] == nextNodes[0])
                    {
                        curNodes[1] = nextNodes[1];
                        curPnts[1] = nextPnts[1];
                    }

                    // TODO: Non-adjacent glued links within the polygon
                }
                else if (intFlag == Same)
                {
                    ProcessLoop(polyIt, nextPolyIt, polygon, polyBoxes);

                    isRemoveFromFirst = true;
                    isAddReplacingEdge = false;
                }
                else
                {
                    continue; // Not supported type
                }

                if (isAddReplacingEdge)
                {
                    curEdgeId = Math.Abs(CreateAndReplacePolygonLink(curNodes,
                                                                      curPnts,
                                                                      polyIt,
                                                                      ReplaceFlag.Replace,
                                                                      polygon,
                                                                      polyBoxes));

                    curEdge = GetEdge(curEdgeId);
                    curVec = new Vector2D(curPnts[0], curPnts[1]);
                }

                int indexToRemoveFrom = isRemoveFromFirst ? polyIt : polyIt + 1;

                RemoveRange(polygon, indexToRemoveFrom, indexToRemoveTo);
                RemoveRange(polyBoxes, indexToRemoveFrom, indexToRemoveTo);

                polyLen = polygon.Length();
                if (isRemoveFromFirst)
                {
                    --polyIt;
                    break;
                }

                nextPolyIt = polyIt;
            }

            _ = curVec;
        }

        NCollection_Sequence<int> polygon1 = polygon;
        NCollection_Sequence<B2d> polyBoxes1 = polyBoxes;

        NCollection_Sequence<int> polygon2 = new();
        NCollection_Sequence<B2d> polyBoxes2 = new();

        NCollection_Sequence<NCollection_Sequence<int>> polyStack = new();
        NCollection_Sequence<NCollection_Sequence<B2d>> polyBoxStack = new();
        for (;;)
        {
            DecomposeSimplePolygon(polygon1, polyBoxes1, polygon2, polyBoxes2);
            if (!polygon2.IsEmpty())
            {
                polyStack.Append(polygon2);
                polyBoxStack.Append(polyBoxes2);

                polygon2 = new NCollection_Sequence<int>();
                polyBoxes2 = new NCollection_Sequence<B2d>();
            }

            if (polygon1.IsEmpty())
            {
                if (!polyStack.IsEmpty() && ReferenceEquals(polygon1, polyStack.First()))
                {
                    polyStack.Remove(1);
                    polyBoxStack.Remove(1);
                }

                if (polyStack.IsEmpty())
                {
                    break;
                }

                polygon1 = polyStack.First();
                polyBoxes1 = polyBoxStack.First();
            }
        }
    }

    private bool MeshElementaryPolygon(NCollection_Sequence<int> polygon)
    {
        int polyLen = polygon.Length();
        if (polyLen < 3)
        {
            return true;
        }
        else if (polyLen > 3)
        {
            return false;
        }

        // Just create a triangle
        Span<int> edges = stackalloc int[3];
        Span<bool> edgesOri = stackalloc bool[3];

        for (int edgeIt = 0; edgeIt < 3; ++edgeIt)
        {
            int edgeInfo = polygon[edgeIt + 1];
            edges[edgeIt] = Math.Abs(edgeInfo);
            edgesOri[edgeIt] = (edgeInfo > 0);
        }

        MeshEdge edge1 = GetEdge(edges[0]);
        MeshEdge edge2 = GetEdge(edges[1]);

        Span<int> nodes = [edge1.FirstNode(), edge1.LastNode(), edge2.FirstNode()];
        if (nodes[2] == nodes[0] || nodes[2] == nodes[1])
        {
            nodes[2] = edge2.LastNode();
        }

        AddTriangle(edges, edgesOri, nodes);
        return true;
    }

    private void DecomposeSimplePolygon(NCollection_Sequence<int> polygon,
                                        NCollection_Sequence<B2d> polyBoxes,
                                        NCollection_Sequence<int> polygonCut,
                                        NCollection_Sequence<B2d> polyBoxesCut)
    {
        // Check is the given polygon elementary
        if (MeshElementaryPolygon(polygon))
        {
            polygon.Clear();
            polyBoxes.Clear();
            return;
        }

        // Polygon contains more than 3 links
        int firstEdgeInfo = polygon[1];
        MeshEdge firstEdge = GetEdge(Math.Abs(firstEdgeInfo));

        Span<int> nodes = stackalloc int[3];
        GetOrientedNodes(firstEdge, firstEdgeInfo > 0, nodes);

        Span<Point2D> refVertices = stackalloc Point2D[3];
        refVertices[0] = new Point2D(GetVertex(nodes[0]).Coord());
        refVertices[1] = new Point2D(GetVertex(nodes[1]).Coord());

        Vector2D refEdgeDir = new(refVertices[0], refVertices[1]);

        double refEdgeLen = refEdgeDir.Magnitude;
        if (refEdgeLen < Precision)
        {
            polygon.Clear();
            polyBoxes.Clear();
            return;
        }

        refEdgeDir /= refEdgeLen;

        // Find a point with minimum distance respect
        // the end of reference link
        int usedLinkId = 0;
        double optAngle = 0.0;
        double minDist = RealLast();
        int pivotNode = nodes[1];
        int polyLen = polygon.Length();
        for (int linkIt = 3; linkIt <= polyLen; ++linkIt)
        {
            int linkInfo = polygon[linkIt];
            MeshEdge nextEdge = GetEdge(Math.Abs(linkInfo));

            pivotNode = linkInfo > 0 ? nextEdge.FirstNode() : nextEdge.LastNode();

            // We have end points touch case in the polygon - ignore it
            if (pivotNode == nodes[1])
            {
                continue;
            }

            Point2D pivotVertex = new(GetVertex(pivotNode).Coord());
            Vector2D distanceDir = new(refVertices[1], pivotVertex);

            double dist = refEdgeDir ^ distanceDir;
            double angle = Math.Abs(refEdgeDir.AngleTo(distanceDir));
            double absDist = Math.Abs(dist);
            if (absDist < Precision || dist < 0.0)
            {
                continue;
            }

            if ((absDist >= minDist) && (angle <= optAngle || angle > AngDeviation90Deg))
            {
                continue;
            }

            // Check is the test link crosses the polygon boundaries
            bool isIntersect = false;
            for (int refLinkNodeIt = 0; refLinkNodeIt < 2; ++refLinkNodeIt)
            {
                int linkFirstNode = nodes[refLinkNodeIt];
                Point2D linkFirstVertex = refVertices[refLinkNodeIt];

                B2d box = new();
                UpdateBndBox(linkFirstVertex.Coord(), pivotVertex.Coord(), ref box);

                MeshEdge checkLink = new(linkFirstNode, pivotNode, DegreeOfFreedom.Free);

                int checkLinkIt = 2;
                for (; checkLinkIt <= polyLen; ++checkLinkIt)
                {
                    if (checkLinkIt == linkIt)
                    {
                        continue;
                    }

                    if (!box.IsOut(polyBoxes.Value(checkLinkIt)))
                    {
                        MeshEdge polyLink = GetEdge(Math.Abs(polygon[checkLinkIt]));

                        if (checkLink.IsEqual(polyLink))
                        {
                            continue;
                        }

                        // intersection is possible...
                        Point2D intPnt = new();
                        GeomTool.IntFlag intFlag =
                            IntSegSeg(checkLink, polyLink, false, false, ref intPnt);

                        if (intFlag != NoIntersection)
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

            optAngle = angle;
            minDist = absDist;
            nodes[2] = pivotNode;
            refVertices[2] = pivotVertex;
            usedLinkId = linkIt;
        }

        if (usedLinkId == 0)
        {
            polygon.Clear();
            polyBoxes.Clear();
            return;
        }

        MeshEdge newEdge0 = new(nodes[1], nodes[2], DegreeOfFreedom.Free);
        MeshEdge newEdge1 = new(nodes[2], nodes[0], DegreeOfFreedom.Free);

        Span<int> newEdgesInfo = stackalloc int[3];
        newEdgesInfo[0] = firstEdgeInfo;
        newEdgesInfo[1] = _meshData!.AddLink(newEdge0);
        newEdgesInfo[2] = _meshData.AddLink(newEdge1);

        Span<int> edges = stackalloc int[3];
        Span<bool> edgesOri = stackalloc bool[3];
        for (int triEdgeIt = 0; triEdgeIt < 3; ++triEdgeIt)
        {
            int edgeInfo = newEdgesInfo[triEdgeIt];
            edges[triEdgeIt] = Math.Abs(edgeInfo);
            edgesOri[triEdgeIt] = edgeInfo > 0;
        }

        AddTriangle(edges, edgesOri, nodes);

        if (usedLinkId == 3)
        {
            polygon.Remove(1);
            polyBoxes.Remove(1);

            polygon.SetValue(1, -newEdgesInfo[2]);

            B2d box = new();
            UpdateBndBox(refVertices[0].Coord(), refVertices[2].Coord(), ref box);
            polyBoxes.SetValue(1, box);
        }
        else
        {
            // Create triangle and split the source polygon on two
            // parts (if possible) and mesh each part as independent
            // polygon.
            if (usedLinkId < polyLen)
            {
                polygon.Split(usedLinkId, polygonCut);
                polygonCut.Prepend(-newEdgesInfo[2]);
                polyBoxes.Split(usedLinkId, polyBoxesCut);

                B2d cutBox = new();
                UpdateBndBox(refVertices[0].Coord(), refVertices[2].Coord(), ref cutBox);
                polyBoxesCut.Prepend(cutBox);
            }
            else
            {
                polygon.Remove(polyLen);
                polyBoxes.Remove(polyLen);
            }

            polygon.SetValue(1, -newEdgesInfo[1]);

            B2d box = new();
            UpdateBndBox(refVertices[1].Coord(), refVertices[2].Coord(), ref box);
            polyBoxes.SetValue(1, box);
        }
    }

    private double CalculateDist(ReadOnlySpan<Xy> vEdges,
                                 ReadOnlySpan<Xy> points,
                                 in MeshVertex vertex,
                                 Span<double> distance,
                                 Span<double> sqModulus,
                                 ref int edgeOn)
    {
        double minDist = RealLast();
        for (int i = 0; i < 3; ++i)
        {
            sqModulus[i] = vEdges[i].SquareModulus();
            if (sqModulus[i] <= Precision2)
            {
                return -1;
            }

            distance[i] = vEdges[i] ^ (vertex.Coord() - points[i]);

            double dist = distance[i] * distance[i];
            dist /= sqModulus[i];

            if (dist < minDist)
            {
                edgeOn = i;
                minDist = dist;
            }
        }

        return minDist;
    }

    private GeomTool.IntFlag IntSegSeg(in MeshEdge edg1,
                                                in MeshEdge edg2,
                                                bool isConsiderEndPointTouch,
                                                bool isConsiderPointOnEdge,
                                                ref Point2D intPnt)
    {
        Xy p1, p2, p3, p4;
        p1 = GetVertex(edg1.FirstNode()).Coord();
        p2 = GetVertex(edg1.LastNode()).Coord();
        p3 = GetVertex(edg2.FirstNode()).Coord();
        p4 = GetVertex(edg2.LastNode()).Coord();

        return GeomTool.IntSegSeg(p1,
                                           p2,
                                           p3,
                                           p4,
                                           isConsiderEndPointTouch,
                                           isConsiderPointOnEdge,
                                           ref intPnt);
    }

    private double PolyArea(NCollection_Sequence<int> polygon, int startIndex, int endIndex)
    {
        double area = 0.0;
        int polyLen = polygon.Length();
        if (startIndex >= endIndex || startIndex > polyLen)
        {
            return area;
        }

        int curEdgeInfo = polygon[startIndex];
        int curEdgeId = Math.Abs(curEdgeInfo);
        MeshEdge curEdge = GetEdge(curEdgeId);

        Span<int> nodes = stackalloc int[2];
        GetOrientedNodes(curEdge, curEdgeInfo > 0, nodes);

        Point2D refPnt = new(GetVertex(nodes[0]).Coord());
        int polyIt = startIndex + 1;
        for (; polyIt <= endIndex; ++polyIt)
        {
            curEdgeInfo = polygon[polyIt];
            curEdgeId = Math.Abs(curEdgeInfo);
            curEdge = GetEdge(curEdgeId);

            GetOrientedNodes(curEdge, curEdgeInfo > 0, nodes);
            Vector2D vec1 = new(refPnt, new Point2D(GetVertex(nodes[0]).Coord()));
            Vector2D vec2 = new(refPnt, new Point2D(GetVertex(nodes[1]).Coord()));

            area += vec1 ^ vec2;
        }

        return area / 2.0;
    }

    private bool IsSupVertex(int vertexIdx)
    {
        for (int it = 0; it < _supVert.Count; ++it)
        {
            if (vertexIdx == _supVert[it])
            {
                return true;
            }
        }

        return false;
    }
}
