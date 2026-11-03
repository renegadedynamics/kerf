// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_DS.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_DS.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_DS.cxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_PDS.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Foundation.Std;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Data;

/// <summary>
/// The data structure of the Boolean component (General Fuse, Booleans, Section, Maker Volume, Splitter, Cells
/// Builder): the arguments; the information on each argument and sub-shape and on the new shapes (type, box,
/// sub-shapes); each argument's range of indices; the pave blocks of the source edges; the state of the source faces;
/// the same-domain shapes; the interferences.
/// </summary>
/// <remarks>
/// <para>
/// A working object of the pave filler and the builders (plan section 6). OCCT's <c>BOPDS_PDS</c> (a pointer to the
/// data structure) is a reference to this class. The allocator goes (translation rules, section 9), and so do the
/// vectors' allocation increments (<c>SetIncrement</c>).
/// </para>
/// <para>
/// Maps keyed by pave blocks compare by reference and iterate in insertion order (decision 580). The maps of
/// integers are <see cref="NCollection_IntMap"/> and <see cref="NCollection_IntDataMap{TValue}"/>, which iterate in the
/// order of OCCT's hash buckets as native's do: the sub-shape lists of faces and solids built in <c>Init</c>, and
/// everything ordered by them, come out as native's (decision 784).
/// </para>
/// </remarks>
public class DS
{
    //! Initial increment for myLines container (an allocation hint in OCCT).
    private const int THE_INITIAL_LINES_INCREMENT = 500;

    private static readonly NCollection_List<PaveBlock> StaticEmptyList = new();
    private static readonly FaceInfo EmptyFaceInfo = new();

    private readonly NCollection_List<Shape> _arguments = new();
    private int _nbSourceShapes; // OCCT also keeps myNbShapes, set to 0 and never read: left out
    private readonly List<IndexRange> _ranges = new();
    private readonly List<ShapeInfo> _lines = new();
    private readonly Dictionary<Shape, int> _mapShapeIndex = new(ShapeHasher.Instance);
    private readonly List<NCollection_List<PaveBlock>> _paveBlocksPool = new();
    private readonly Dictionary<PaveBlock, CommonBlock> _mapPBCB = new();
    private readonly List<FaceInfo> _faceInfoPool = new();
    private readonly NCollection_IntDataMap<int> _shapesSD = new();
    private readonly NCollection_IntDataMap<NCollection_List<int>> _mapVE = new();
    private readonly MapOfPair _interfTB = new(100); // OCCT's buckets and hash, so that it iterates as OCCT's (BOPDS_MapOfPair)
    private readonly List<InterfVV> _interfVV = new();
    private readonly List<InterfVE> _interfVE = new();
    private readonly List<InterfVF> _interfVF = new();
    private readonly List<InterfEE> _interfEE = new();
    private readonly List<InterfEF> _interfEF = new();
    private readonly List<InterfFF> _interfFF = new();
    private readonly List<InterfVZ> _interfVZ = new();
    private readonly List<InterfEZ> _interfEZ = new();
    private readonly List<InterfFZ> _interfFZ = new();
    private readonly List<InterfZZ> _interfZZ = new();
    private readonly NCollection_IntMap _interfered = new();

    /// <summary>An empty data structure.</summary>
    public DS()
    {
        _nbSourceShapes = 0;
    }

    /// <summary>Clears the contents.</summary>
    public void Clear()
    {
        _nbSourceShapes = 0;
        //
        _arguments.Clear();
        _ranges.Clear();
        _lines.Clear();
        _mapShapeIndex.Clear();
        _paveBlocksPool.Clear();
        _faceInfoPool.Clear();
        _shapesSD.Clear();
        _mapVE.Clear();
        _mapPBCB.Clear();
        _interfTB.Clear();
        _interfVV.Clear();
        _interfVE.Clear();
        _interfVF.Clear();
        _interfEE.Clear();
        _interfEF.Clear();
        _interfFF.Clear();
        _interfVZ.Clear();
        _interfEZ.Clear();
        _interfFZ.Clear();
        _interfZZ.Clear();
        _interfered.Clear();
    }

    /// <summary>Sets the arguments of the operation (a copy of the list).</summary>
    public void SetArguments(NCollection_List<Shape> arguments) => _arguments.Assign(arguments);

    /// <summary>The arguments of the operation (the data structure's own list; read only for callers).</summary>
    public NCollection_List<Shape> Arguments() => _arguments;

    /// <summary>Initializes the data structure for the arguments.</summary>
    public void Init() => Init(global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <summary>Initializes the data structure for the arguments, with the fuzzy value.</summary>
    public void Init(double fuzz)
    {
        // 1. Append Source Shapes
        if (_arguments.IsEmpty())
        {
            return;
        }

        int i1 = 0;
        for (NCollection_List<Shape>.Iterator it = new(_arguments); it.More(); it.Next())
        {
            Shape shape = it.Value();
            if (_mapShapeIndex.ContainsKey(shape))
            {
                continue;
            }

            InitShape(Append(shape), shape);
            int i2 = ShapeCount - 1;
            _ranges.Add(new IndexRange(i1, i2));
            i1 = i2 + 1;
        }

        _nbSourceShapes = ShapeCount;

        // 2. Prepare data for shapes. Includes updating bound boxes, updating sub-shapes.
        double additionalTolerance = Max(fuzz, global::Kerf.Engine.Foundation.Precision.Confusion()) * 0.5;
        PrepareVertices(additionalTolerance);                 // Vertex.
        int edgeCount = PrepareEdges(additionalTolerance);  // Edge.
        int faceCount = PrepareFaces(additionalTolerance);   // Face.
        PrepareSolids();                                        // Solid.

        // 3. Prepare Vertex-Edge connection map.
        BuildVertexEdgeMap();

        // 4. Prepare pools (OCCT sets the allocation increments to the numbers of edges and faces).
        _ = edgeCount;
        _ = faceCount;
    }

    /// <summary>The total number of shapes stored.</summary>
    public int ShapeCount => _lines.Count;

    /// <summary>The number of source shapes (the arguments and their sub-shapes).</summary>
    public int SourceShapeCount => _nbSourceShapes;

    /// <summary>The number of index ranges (one per argument).</summary>
    public int RangeCount => _ranges.Count;

    /// <summary>The index range <paramref name="index"/>.</summary>
    public IndexRange Range(int index) => _ranges[index];

    /// <summary>The rank (the argument's range) of the shape with the index, or -1.</summary>
    public int Rank(int index)
    {
        for (int rangeIndex = 0; rangeIndex < RangeCount; ++rangeIndex)
        {
            if (Range(rangeIndex).Contains(index))
            {
                return rangeIndex;
            }
        }
        return -1;
    }

    /// <summary>True if the shape with the index is not a source shape.</summary>
    public bool IsNewShape(int index) => index >= SourceShapeCount;

    /// <summary>Appends a copy of the shape information; returns its index.</summary>
    internal int Append(ShapeInfo shapeInfo)
    {
        _lines.Add(new ShapeInfo(shapeInfo));
        int shapeIndex = _lines.Count - 1;
        _mapShapeIndex[shapeInfo.Shape()] = shapeIndex;
        return shapeIndex;
    }

    /// <summary>Appends the default information of the shape; returns its index.</summary>
    internal int Append(in Shape shape)
    {
        _lines.Add(new ShapeInfo());
        _lines[^1].SetShape(shape);
        int shapeIndex = _lines.Count - 1;
        _mapShapeIndex[shape] = shapeIndex;
        return shapeIndex;
    }

    /// <summary>The information on the shape with the index (the data structure's own object; read only for callers).</summary>
    public ShapeInfo ShapeInfo(int index) => _lines[index];

    internal ShapeInfo ChangeShapeInfo(int index) => _lines[index];

    /// <summary>The shape with the index.</summary>
    public Shape Shape(int index) => ShapeInfo(index).Shape();

    /// <summary>The index of the shape, or -1.</summary>
    public int Index(in Shape shape)
    {
        int index = -1;
        if (_mapShapeIndex.TryGetValue(shape, out int found))
        {
            index = found;
        }
        return index;
    }

    private void InitShape(int shapeInfoIndex, in Shape shape)
    {
        ShapeInfo shapeInfo = ChangeShapeInfo(shapeInfoIndex);
        shapeInfo.SetShapeType(shape.ShapeType());
        NCollection_List<int> subShapeIndices = shapeInfo.ChangeSubShapes();

        NCollection_IntMap existingSubShapeIndices = new();
        foreach (int subShapeIndex in subShapeIndices)
        {
            existingSubShapeIndices.Add(subShapeIndex);
        }

        for (Iterator shapeIter = new(shape); shapeIter.More(); shapeIter.Next())
        {
            Shape subShape = shapeIter.Value();
            int subShapeIndex = _mapShapeIndex.TryGetValue(subShape, out int existingIndex) ? existingIndex : Append(subShape);

            InitShape(subShapeIndex, subShape);
            if (existingSubShapeIndices.Add(subShapeIndex))
            {
                subShapeIndices.Append(subShapeIndex);
            }
        }
    }

    /// <summary>
    /// True if the shape <paramref name="index1"/> interferes with any (or, if not
    /// <paramref name="anyInterference"/>, every) sub-shape of the shape <paramref name="index2"/>.
    /// </summary>
    public bool HasInterfShapeSubShapes(int index1, int index2, bool anyInterference = true)
    {
        NCollection_List<int> subShapes = ShapeInfo(index2).SubShapes();
        if (anyInterference)
        {
            foreach (int subShapeIndex in subShapes)
            {
                if (HasInterf(index1, subShapeIndex))
                {
                    return true;
                }
            }
            return false;
        }
        else
        {
            foreach (int subShapeIndex in subShapes)
            {
                if (!HasInterf(index1, subShapeIndex))
                {
                    return false;
                }
            }
            return true;
        }
    }

    /// <summary>True if sub-shapes of the two shapes interfere.</summary>
    public bool HasInterfSubShapes(int index1, int index2)
    {
        foreach (int subShapeIndex in ShapeInfo(index1).SubShapes())
        {
            if (HasInterfShapeSubShapes(subShapeIndex, index2))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>The pave blocks of the source edges (the data structure's own pool; read only for callers).</summary>
    public List<NCollection_List<PaveBlock>> PaveBlocksPool() => _paveBlocksPool;

    internal List<NCollection_List<PaveBlock>> ChangePaveBlocksPool() => _paveBlocksPool;

    /// <summary>True if the shape with the index has pave blocks.</summary>
    public bool HasPaveBlocks(int index) => ShapeInfo(index).HasReference;

    /// <summary>The pave blocks of the shape with the index, or an empty list (read only for callers).</summary>
    public NCollection_List<PaveBlock> PaveBlocks(int index)
    {
        if (HasPaveBlocks(index))
        {
            return _paveBlocksPool[ShapeInfo(index).Reference()];
        }

        return StaticEmptyList;
    }

    /// <summary>The pave blocks of the edge with the index, made first if the edge has none.</summary>
    internal NCollection_List<PaveBlock> ChangePaveBlocks(int index)
    {
        ShapeInfo shapeInfo = ChangeShapeInfo(index);
        if (!shapeInfo.HasReference)
        {
            InitPaveBlocks(index);
        }
        return _paveBlocksPool[shapeInfo.Reference()];
    }

    private void InitPaveBlocks(int edgeIndex)
    {
        ShapeInfo edgeInfo = ChangeShapeInfo(edgeIndex);
        Edge edge = ShapeCasts.Edge(edgeInfo.Shape());

        NCollection_List<int> vertexIndices = edgeInfo.SubShapes();
        if (vertexIndices.IsEmpty())
        {
            return;
        }

        PaveBlock paveBlock = new();
        paveBlock.SetOriginalEdge(edgeIndex);

        if (edge.Orientation() != Internal)
        {
            foreach (int vertexIndexIt in vertexIndices)
            {
                int vertexIndex = vertexIndexIt;
                ShapeInfo vertexInfo = ShapeInfo(vertexIndex);
                Vertex vertex = ShapeCasts.Vertex(vertexInfo.Shape());
                // Important: for vertices HasFlag() means that the original edge is located on the infinite
                // curve and vertex was created during initialization of this class object.
                // Such vertices are not topologically connected to the edge so we cannot use
                // BRep_Tool::Parameter.
                double vertexParam = vertexInfo.HasFlag() ? ComputeParameter(vertex, edge)
                                                            : Tool.Parameter(vertex, edge);

                vertexIndex = GetSameDomainIndex(vertexIndex);
                Pave pave = new(vertexIndex, vertexParam);
                if (edgeInfo.HasFlag())
                {
                    // for a degenerated edge append pave unconditionally
                    paveBlock.AppendExtPave1(pave);
                }
                else
                {
                    paveBlock.AppendExtPave(pave);
                }

                // Handle closed edges (seam edges) that have a single vertex shared by both ends.
                // In this case, we need to add two paves: one for the start and one for the end
                // of the edge, even though they reference the same vertex.
                if (vertexIndices.Extent() == 1)
                {
                    vertex.Reverse();
                    paveBlock.AppendExtPave1(new Pave(vertexIndex, Tool.Parameter(vertex, edge)));
                }
            }
        }
        else
        {
            for (Iterator edgeIter = new(edge, false, true); edgeIter.More(); edgeIter.Next())
            {
                Vertex vertex = ShapeCasts.Vertex(edgeIter.Value());
                int vertexIndex = Index(vertex);
                ShapeInfo vertexInfo = ShapeInfo(vertexIndex);
                double vertexParam = vertexInfo.HasFlag() ? ComputeParameter(vertex, edge)
                                                            : Tool.Parameter(vertex, edge);
                paveBlock.AppendExtPave1(new Pave(GetSameDomainIndex(vertexIndex), vertexParam));
            }
        }

        NCollection_List<PaveBlock> newList =  new NCollection_List<PaveBlock>();
        _paveBlocksPool.Add(newList);
        paveBlock.Update(newList, false);
        edgeInfo.SetReference(_paveBlocksPool.Count - 1);
    }

    /// <summary>Splits every pave block with extra paves (see <see cref="PaveBlock.Update"/>).</summary>
    internal void UpdatePaveBlocks()
    {
        foreach (NCollection_List<PaveBlock> paveBlockList in _paveBlocksPool)
        {
            for (NCollection_List<PaveBlock>.Iterator itPB = new(paveBlockList); itPB.More();)
            {
                PaveBlock paveBlock = itPB.Value();

                if (!paveBlock.IsToUpdate)
                {
                    itPB.Next();
                    continue;
                }

                NCollection_List<PaveBlock> lpbn = new();
                paveBlock.Update(lpbn);
                paveBlockList.Remove(itPB);
                paveBlockList.Append(lpbn);
            }
        }
    }

    /// <summary>Splits the pave block at its extra paves, in its edge's list.</summary>
    internal void UpdatePaveBlock(PaveBlock paveBlock)
    {
        if (!paveBlock.IsToUpdate)
        {
            return;
        }

        ShapeInfo shapeInfo = ChangeShapeInfo(paveBlock.OriginalEdge());
        NCollection_List<PaveBlock> lpb = _paveBlocksPool[shapeInfo.Reference()];

        for (NCollection_List<PaveBlock>.Iterator itPB = new(lpb); itPB.More(); itPB.Next())
        {
            PaveBlock aPaveBlock = itPB.Value();
            if (aPaveBlock == paveBlock)
            {
                NCollection_List<PaveBlock> lpbn = new();
                aPaveBlock.Update(lpbn);
                lpb.Append(lpbn);
                lpb.Remove(itPB);
                break;
            }
        }
    }

    /// <summary>
    /// Splits the pave blocks of the common block at their extra paves and makes new common blocks of the coinciding
    /// splits.
    /// </summary>
    internal void UpdateCommonBlock(CommonBlock commonBlock, double fuzzyValue)
    {
        NCollection_List<PaveBlock> paveBlocksOfCommonBlock = commonBlock.PaveBlocks();
        if (!paveBlocksOfCommonBlock.First().IsToUpdate)
        {
            return;
        }

        NCollection_List<PaveBlock>.Iterator itPB = new();
        // OCCT iterates this map in the order of its hash buckets (keyed by pairs of vertex indices); the port in
        // insertion order. The order decides only the order in which the new common blocks are made, which nothing
        // indexes, so it isn't observable (decision 784).
        Dictionary<Pair, NCollection_List<PaveBlock>> mpklpb = new();

        foreach (PaveBlock paveBlockOfCommonBlock in paveBlocksOfCommonBlock)
        {
            int edgeIndex = paveBlockOfCommonBlock.OriginalEdge();
            NCollection_List<PaveBlock> lpb = _paveBlocksPool[ChangeShapeInfo(edgeIndex).Reference()];

            itPB.Initialize(lpb);
            for (; itPB.More(); itPB.Next())
            {
                PaveBlock pb = itPB.Value();
                if (pb == paveBlockOfCommonBlock)
                {
                    NCollection_List<PaveBlock> lpbn = new();
                    pb.Update(lpbn);
                    foreach (PaveBlock pbn in lpbn)
                    {
                        lpb.Append(pbn);

                        pbn.Indices(out int n1, out int n2);
                        Pair pk = new(n1, n2);
                        if (mpklpb.TryGetValue(pk, out NCollection_List<PaveBlock>? paveBlocks))
                        {
                            paveBlocks.Append(pbn);
                        }
                        else
                        {
                            NCollection_List<PaveBlock> newLPBx = new();
                            newLPBx.Append(pbn);
                            mpklpb[pk] = newLPBx;
                        }
                    }
                    lpb.Remove(itPB);
                    break;
                }
            }
        }

        foreach (KeyValuePair<Pair, NCollection_List<PaveBlock>> itMPKLPB in mpklpb)
        {
            NCollection_List<PaveBlock> lpBx = itMPKLPB.Value;

            while (!lpBx.IsEmpty())
            {
                NCollection_List<PaveBlock> lpBxN = new();

                itPB.Initialize(lpBx);
                for (; itPB.More();)
                {
                    PaveBlock pBx = itPB.Value();
                    if (lpBxN.Extent() != 0)
                    {
                        PaveBlock pbCx = lpBxN.First();
                        if (CheckCoincidence(pBx, pbCx, fuzzyValue))
                        {
                            lpBxN.Append(pBx);
                            lpBx.Remove(itPB);
                            continue;
                        }
                    }
                    else
                    {
                        lpBxN.Append(pBx);
                        lpBx.Remove(itPB);
                        continue;
                    }
                    itPB.Next();
                }

                CommonBlock aCommonBlock = new();
                aCommonBlock.SetPaveBlocks(lpBxN);
                aCommonBlock.SetFaces(commonBlock.Faces());

                itPB.Initialize(lpBxN);
                for (; itPB.More(); itPB.Next())
                {
                    SetCommonBlock(itPB.Value(), aCommonBlock);
                }
            }
        }
    }

    /// <summary>The first pave block of the pave block's common block, or the pave block itself.</summary>
    public PaveBlock RealPaveBlock(PaveBlock paveBlock)
    {
        CommonBlock? commonBlock = CommonBlock(paveBlock);
        return commonBlock is not null ? commonBlock.PaveBlock1() : paveBlock;
    }

    /// <summary>True if the pave block's common block has more than one pave block.</summary>
    public bool IsCommonBlockOnEdge(PaveBlock paveBlock)
    {
        CommonBlock? commonBlock = CommonBlock(paveBlock);
        return commonBlock is not null && commonBlock.PaveBlocks().Extent() > 1;
    }

    /// <summary>True if the pave block is in a common block.</summary>
    public bool IsCommonBlock(PaveBlock paveBlock) => _mapPBCB.ContainsKey(paveBlock);

    /// <summary>The pave block's common block, or null.</summary>
    public CommonBlock? CommonBlock(PaveBlock paveBlock) =>
        _mapPBCB.TryGetValue(paveBlock, out CommonBlock? commonBlock) ? commonBlock : null;

    /// <summary>Sets the pave block's common block.</summary>
    internal void SetCommonBlock(PaveBlock paveBlock, CommonBlock commonBlock) =>
        _mapPBCB[paveBlock] = commonBlock;

    /// <summary>The states of the faces (the data structure's own pool; read only for callers).</summary>
    public List<FaceInfo> FaceInfoPool() => _faceInfoPool;

    /// <summary>True if the shape with the index has face information.</summary>
    public bool HasFaceInfo(int index) => ShapeInfo(index).HasReference;

    /// <summary>The state of the face with the index, or an empty state (read only for callers).</summary>
    public FaceInfo FaceInfo(int index)
    {
        if (!HasFaceInfo(index))
        {
            return EmptyFaceInfo;
        }

        return _faceInfoPool[ShapeInfo(index).Reference()];
    }

    /// <summary>The state of the face with the index, made first if the face has none.</summary>
    internal FaceInfo ChangeFaceInfo(int index)
    {
        ShapeInfo shapeInfo = ChangeShapeInfo(index);
        if (!shapeInfo.HasReference)
        {
            InitFaceInfo(index);
        }
        return _faceInfoPool[shapeInfo.Reference()];
    }

    private void InitFaceInfo(int index)
    {
        ShapeInfo shapeInfo = ChangeShapeInfo(index);
        FaceInfo faceInfo =  new FaceInfo();
        _faceInfoPool.Add(faceInfo);
        shapeInfo.SetReference(_faceInfoPool.Count - 1);
        faceInfo.SetIndex(index);

        InitFaceInfoIn(index);
        UpdateFaceInfoOn(index);
    }

    private void InitFaceInfoIn(int index)
    {
        ShapeInfo shapeInfo = ChangeShapeInfo(index);
        if (!shapeInfo.HasReference)
        {
            return;
        }

        FaceInfo faceInfo = _faceInfoPool[shapeInfo.Reference()];
        for (Iterator shapeIter = new(Shape(index)); shapeIter.More(); shapeIter.Next())
        {
            Shape vertex = shapeIter.Value();
            if (vertex.ShapeType() == ShapeType.Vertex)
            {
                int vertexIndex = GetSameDomainIndex(Index(vertex));
                faceInfo.ChangeVerticesIn().Add(vertexIndex);
            }
        }
    }

    /// <summary>Recomputes the state In of the face with the index.</summary>
    internal void UpdateFaceInfoIn(int index)
    {
        ShapeInfo shapeInfo = ChangeShapeInfo(index);
        if (!shapeInfo.HasReference)
        {
            return;
        }

        FaceInfo faceInfo = _faceInfoPool[shapeInfo.Reference()];
        NCollection_IndexedMap<PaveBlock> paveBlocksIn = faceInfo.ChangePaveBlocksIn();
        NCollection_IntMap verticesIn = faceInfo.ChangeVerticesIn();
        paveBlocksIn.Clear();
        verticesIn.Clear();
        FaceInfoIn(index, paveBlocksIn, verticesIn);
    }

    /// <summary>Recomputes the state On of the face with the index.</summary>
    internal void UpdateFaceInfoOn(int index)
    {
        ShapeInfo shapeInfo = ChangeShapeInfo(index);
        if (!shapeInfo.HasReference)
        {
            return;
        }

        FaceInfo faceInfo = _faceInfoPool[shapeInfo.Reference()];
        NCollection_IndexedMap<PaveBlock> paveBlocksOn = faceInfo.ChangePaveBlocksOn();
        NCollection_IntMap verticesOn = faceInfo.ChangeVerticesOn();
        paveBlocksOn.Clear();
        verticesOn.Clear();
        FaceInfoOn(index, paveBlocksOn, verticesOn);
    }

    /// <summary>The state On of the face: the (real) pave blocks of its edges and the vertices.</summary>
    public void FaceInfoOn(int index, NCollection_IndexedMap<PaveBlock> mpb, NCollection_IntMap mvp)
    {
        foreach (int subShapeIndex in ShapeInfo(index).SubShapes())
        {
            if (ShapeInfo(subShapeIndex).ShapeType() == ShapeType.Edge)
            {
                foreach (PaveBlock paveBlock in PaveBlocks(subShapeIndex))
                {
                    paveBlock.Indices(out int nV1, out int nV2);
                    mvp.Add(nV1);
                    mvp.Add(nV2);
                    mpb.Add(RealPaveBlock(paveBlock));
                }
            }
            else // if (ShapeInfo(aSubShapeIndex).ShapeType() == TopAbs_VERTEX)
            {
                mvp.Add(GetSameDomainIndex(subShapeIndex));
            }
        }
    }

    /// <summary>
    /// The state In of the face: its internal vertices, the vertices of vertex/face interferences, the new vertices
    /// of edge/face interferences and the common blocks lying on the face.
    /// </summary>
    public void FaceInfoIn(int faceIndex, NCollection_IndexedMap<PaveBlock> paveBlocks, NCollection_IntMap indexMap)
    {
        // 1. Pure internal vertices on the face
        // Adding to theIndexMap all the indicies of vertices belonging to the face.
        Shape face = Shape(faceIndex);
        for (Iterator faceIter = new(face); faceIter.More(); faceIter.Next())
        {
            Shape subShape = faceIter.Value();
            if (subShape.ShapeType() == ShapeType.Vertex)
            {
                int vertexIndex = GetSameDomainIndex(Index(subShape));
                indexMap.Add(vertexIndex);
            }
        }

        // 2. Vertex-Face interferences
        // Adding to theIndexMap all the indicies of vertices having interferences with the face.
        foreach (InterfVF vertexFaceInterference in InterfVF())
        {
            if (vertexFaceInterference.Contains(faceIndex))
            {
                int vertexIndex = GetSameDomainIndex(vertexFaceInterference.OppositeIndex(faceIndex));
                indexMap.Add(vertexIndex);
            }
        }

        // 3. Edge-Face interferences
        foreach (InterfEF edgeFaceInterference in InterfEF())
        {
            if (!edgeFaceInterference.Contains(faceIndex))
            {
                continue;
            }

            if (edgeFaceInterference.GetIndexNew() is int indexNew)
            {
                indexMap.Add(GetSameDomainIndex(indexNew));
            }
            else
            {
                foreach (PaveBlock paveBlock in PaveBlocks(edgeFaceInterference.OppositeIndex(faceIndex)))
                {
                    CommonBlock? commonBlock = CommonBlock(paveBlock);
                    if (commonBlock is not null && commonBlock.Contains(faceIndex))
                    {
                        paveBlocks.Add(commonBlock.PaveBlock1());
                    }
                }
            }
        }
    }

    /// <summary>Recomputes the state In of every face of the map.</summary>
    internal void UpdateFaceInfoIn(NCollection_IntMap faces)
    {
        foreach (int faceIndex in faces)
        {
            ShapeInfo shapeInfo = ChangeShapeInfo(faceIndex);
            if (!shapeInfo.HasReference)
            {
                _faceInfoPool.Add(new FaceInfo());
                _faceInfoPool[^1].SetIndex(faceIndex);
                shapeInfo.SetReference(_faceInfoPool.Count - 1);
            }
            FaceInfo faceInfo = _faceInfoPool[shapeInfo.Reference()];
            faceInfo.ChangePaveBlocksIn().Clear();
            faceInfo.ChangeVerticesIn().Clear();

            // 1. Add pure internal vertices on the face
            InitFaceInfoIn(faceIndex);
        }

        // 2. Analyze Vertex-Face interferences
        foreach (InterfVF vertexFaceInterference in InterfVF())
        {
            int faceIndex = vertexFaceInterference.Index2();
            if (faces.Contains(faceIndex))
            {
                int sameDomainIndex = GetSameDomainIndex(vertexFaceInterference.Index1());
                _faceInfoPool[ShapeInfo(faceIndex).Reference()].ChangeVerticesIn().Add(sameDomainIndex);
            }
        }

        // 3. Analyze Edge-Face interferences
        foreach (InterfEF edgeFaceInterference in InterfEF())
        {
            int faceIndex = edgeFaceInterference.Index2();
            if (!faces.Contains(faceIndex))
            {
                continue;
            }

            FaceInfo faceInfo = _faceInfoPool[ShapeInfo(faceIndex).Reference()];
            if (edgeFaceInterference.GetIndexNew() is int indexNew)
            {
                faceInfo.ChangeVerticesIn().Add(GetSameDomainIndex(indexNew));
            }
            else
            {
                foreach (PaveBlock paveBlock in PaveBlocks(edgeFaceInterference.Index1()))
                {
                    CommonBlock? commonBlock = CommonBlock(paveBlock);
                    if (commonBlock is not null && commonBlock.Contains(faceIndex))
                    {
                        faceInfo.ChangePaveBlocksIn().Add(commonBlock.PaveBlock1());
                    }
                }
            }
        }
    }

    /// <summary>Recomputes the state On of every face of the map.</summary>
    internal void UpdateFaceInfoOn(NCollection_IntMap faces)
    {
        foreach (int shapeIndex in faces)
        {
            ShapeInfo shapeInfo = ChangeShapeInfo(shapeIndex);
            if (!shapeInfo.HasReference)
            {
                _faceInfoPool.Add(new FaceInfo());
                _faceInfoPool[^1].SetIndex(shapeIndex);
                shapeInfo.SetReference(_faceInfoPool.Count - 1);
            }
            FaceInfo faceInfo = _faceInfoPool[shapeInfo.Reference()];
            faceInfo.ChangePaveBlocksOn().Clear();
            faceInfo.ChangeVerticesOn().Clear();

            FaceInfoOn(shapeIndex, faceInfo.ChangePaveBlocksOn(), faceInfo.ChangeVerticesOn());
        }
    }

    /// <summary>Recomputes the state On of every face, keeping only the pave blocks that have edges.</summary>
    internal void RefineFaceInfoOn()
    {
        foreach (FaceInfo faceInfo in _faceInfoPool)
        {
            UpdateFaceInfoOn(faceInfo.Index());
            NCollection_IndexedMap<PaveBlock> paveBlocksOn = faceInfo.ChangePaveBlocksOn();
            for (int paveBlockIndex = paveBlocksOn.Extent(); paveBlockIndex >= 1; --paveBlockIndex)
            {
                PaveBlock paveBlock = paveBlocksOn.FindKey(paveBlockIndex);
                if (!paveBlock.HasEdge())
                {
                    paveBlocksOn.RemoveFromIndex(paveBlockIndex);
                }
            }
        }
    }

    /// <summary>Removes from the state In of every face the pave blocks that are also On.</summary>
    internal void RefineFaceInfoIn()
    {
        for (int i = 0; i < _nbSourceShapes; ++i)
        {
            ShapeInfo shapeInfo = ShapeInfo(i);
            if (shapeInfo.ShapeType() != ShapeType.Face || !shapeInfo.HasReference)
            {
                continue;
            }

            FaceInfo faceInfo = ChangeFaceInfo(i);
            NCollection_IndexedMap<PaveBlock> paveBlocksOn = faceInfo.PaveBlocksOn();
            NCollection_IndexedMap<PaveBlock> paveBlocksIn = faceInfo.ChangePaveBlocksIn();
            if (paveBlocksIn.IsEmpty() || paveBlocksOn.IsEmpty())
            {
                continue;
            }

            for (int paveBlockIndex = paveBlocksIn.Extent(); paveBlockIndex >= 1; --paveBlockIndex)
            {
                PaveBlock paveBlock = paveBlocksIn.FindKey(paveBlockIndex);
                if (paveBlocksOn.Contains(paveBlock))
                {
                    paveBlocksIn.RemoveFromIndex(paveBlockIndex);
                }
            }
        }
    }

    /// <summary>
    /// Appends to the list the vertices In and Sc of the face that are not ends of its In and Sc pave blocks
    /// (in the vertex maps' order: decision 784).
    /// </summary>
    public void AloneVertices(int faceIndex, NCollection_List<int> vertexList)
    {
        if (!HasFaceInfo(faceIndex))
        {
            return;
        }

        NCollection_IntMap mi = new(100);
        FaceInfo faceInfo = FaceInfo(faceIndex);

        foreach (NCollection_IndexedMap<PaveBlock> paveBlocks in new[] { faceInfo.PaveBlocksIn(), faceInfo.PaveBlocksSc() })
        {
            for (int paveBlockIndex = 1; paveBlockIndex <= paveBlocks.Extent(); ++paveBlockIndex)
            {
                PaveBlock paveBlock = paveBlocks.FindKey(paveBlockIndex);
                paveBlock.Indices(out int nV1, out int nV2);
                mi.Add(nV1);
                mi.Add(nV2);
            }
        }

        foreach (NCollection_IntMap vertexMap in new[] { faceInfo.VerticesIn(), faceInfo.VerticesSc() })
        {
            foreach (int vertexIndex in vertexMap)
            {
                if (vertexIndex >= 0 && mi.Add(vertexIndex))
                {
                    vertexList.Append(vertexIndex);
                }
            }
        }
    }

    /// <summary>
    /// The On and In sub-shapes of two faces: <paramref name="mvOnIn"/> the On/In vertices of both,
    /// <paramref name="mvCommon"/> the vertices common to both, <paramref name="pbOnIn"/> all their On/In pave
    /// blocks, <paramref name="commonPaveBlocks"/> the pave blocks shared by both.
    /// </summary>
    public void SubShapesOnIn(int faceIndex1,
                              int faceIndex2,
                              NCollection_IntMap mvOnIn,
                              NCollection_IntMap mvCommon,
                              NCollection_IndexedMap<PaveBlock> pbOnIn,
                              HashSet<PaveBlock> commonPaveBlocks)
    {
        FaceInfo faceInfo1 = FaceInfo(faceIndex1);
        FaceInfo faceInfo2 = FaceInfo(faceIndex2);

        NCollection_IndexedMap<PaveBlock> pbOn1 = faceInfo1.PaveBlocksOn();
        NCollection_IndexedMap<PaveBlock> pbIn1 = faceInfo1.PaveBlocksIn();
        NCollection_IndexedMap<PaveBlock> pbOn2 = faceInfo2.PaveBlocksOn();
        NCollection_IndexedMap<PaveBlock> pbIn2 = faceInfo2.PaveBlocksIn();

        // Helper to process pave blocks from a map
        void ProcessMap(NCollection_IndexedMap<PaveBlock> map)
        {
            for (int idx = 1; idx <= map.Extent(); ++idx)
            {
                PaveBlock paveBlock = map.FindKey(idx);
                pbOnIn.Add(paveBlock);
                paveBlock.Indices(out int v1, out int v2);
                mvOnIn.Add(v1);
                mvOnIn.Add(v2);
            }
        }

        // Process all four maps
        ProcessMap(pbOn1);
        ProcessMap(pbIn1);
        ProcessMap(pbOn2);
        ProcessMap(pbIn2);

        // Find common pave blocks (those in Face1 that are also in Face2)
        void FindCommon(NCollection_IndexedMap<PaveBlock> map)
        {
            for (int idx = 1; idx <= map.Extent(); ++idx)
            {
                PaveBlock paveBlock = map.FindKey(idx);
                if (pbOn2.Contains(paveBlock) || pbIn2.Contains(paveBlock))
                {
                    commonPaveBlocks.Add(paveBlock);
                    paveBlock.Indices(out int v1, out int v2);
                    mvCommon.Add(v1);
                    mvCommon.Add(v2);
                }
            }
        }

        FindCommon(pbOn1);
        FindCommon(pbIn1);

        NCollection_IntMap mvOn1 = faceInfo1.VerticesOn();
        NCollection_IntMap mvIn1 = faceInfo1.VerticesIn();
        NCollection_IntMap mvOn2 = faceInfo2.VerticesOn();
        NCollection_IntMap mvIn2 = faceInfo2.VerticesIn();

        foreach (NCollection_IntMap mv1 in new[] { mvOn1, mvIn1 })
        {
            foreach (int vertexIndex in mv1)
            {
                if (mvOn2.Contains(vertexIndex) || mvIn2.Contains(vertexIndex))
                {
                    mvOnIn.Add(vertexIndex);

                    // Vertex taken from the 1st face is in the 2nd one.
                    mvCommon.Add(vertexIndex);
                }
            }
        }
    }

    /// <summary>Appends to the list the edges shared by the two faces (the allocator of OCCT's signature goes).</summary>
    public void SharedEdges(int faceIndex1, int faceIndex2, NCollection_List<int> edgeList)
    {
        NCollection_IntMap firstFaceEdges = new(100);

        // Collect edges of the first face.
        foreach (int subShapeIndex in ShapeInfo(faceIndex1).SubShapes())
        {
            ShapeInfo subShapeInfo = ChangeShapeInfo(subShapeIndex);
            if (subShapeInfo.ShapeType() != ShapeType.Edge)
            {
                continue;
            }

            NCollection_List<PaveBlock> subShapePaveBlocks = PaveBlocks(subShapeIndex);
            if (subShapePaveBlocks.IsEmpty())
            {
                firstFaceEdges.Add(subShapeIndex);
            }
            else
            {
                foreach (PaveBlock paveBlock in subShapePaveBlocks)
                {
                    firstFaceEdges.Add(RealPaveBlock(paveBlock).Edge());
                }
            }
        }

        // Add edges of the second face if they are contained in the first one.
        foreach (int subShapeIndex in ShapeInfo(faceIndex2).SubShapes())
        {
            ShapeInfo subShapeInfo = ChangeShapeInfo(subShapeIndex);
            if (subShapeInfo.ShapeType() != ShapeType.Edge)
            {
                continue;
            }

            NCollection_List<PaveBlock> subShapePaveBlocks = PaveBlocks(subShapeIndex);
            if (subShapePaveBlocks.IsEmpty())
            {
                if (firstFaceEdges.Contains(subShapeIndex))
                {
                    edgeList.Append(subShapeIndex);
                }
            }
            else
            {
                foreach (PaveBlock paveBlock in subShapePaveBlocks)
                {
                    int edgeIndex = RealPaveBlock(paveBlock).Edge();
                    if (firstFaceEdges.Contains(edgeIndex))
                    {
                        edgeList.Append(edgeIndex);
                    }
                }
            }
        }
    }

    /// <summary>The same-domain shapes (the data structure's own map).</summary>
    internal NCollection_IntDataMap<int> ShapesSD() => _shapesSD;

    /// <summary>Records that the shape <paramref name="index"/> has the same-domain shape <paramref name="indexSD"/>.</summary>
    internal void AddShapeSD(int index, int indexSD)
    {
        if (index != indexSD)
        {
            _shapesSD.Bind(index, indexSD);
        }
    }

    /// <summary>
    /// True if the shape has a same-domain shape; the last of the chain then (otherwise
    /// <paramref name="indexSD"/> is left as it was).
    /// </summary>
    public bool HasShapeSD(int index, ref int indexSD)
    {
        bool bHasSD = false;
        int key = index;
        while (_shapesSD.Find(key, out int sd))
        {
            indexSD = sd;
            bHasSD = true;
            key = indexSD;
        }
        return bHasSD;
    }

    /// <summary>The index of the shape's same-domain shape (the end of the chain), or the index itself.</summary>
    public int GetSameDomainIndex(int index)
    {
        int result = index;
        while (_shapesSD.Find(result, out int sameDomainIndex))
        {
            result = sameDomainIndex;
        }
        return result;
    }

    internal List<InterfVV> InterfVV() => _interfVV;

    internal List<InterfVE> InterfVE() => _interfVE;

    internal List<InterfVF> InterfVF() => _interfVF;

    internal List<InterfEE> InterfEE() => _interfEE;

    internal List<InterfEF> InterfEF() => _interfEF;

    internal List<InterfFF> InterfFF() => _interfFF;

    internal List<InterfVZ> InterfVZ() => _interfVZ;

    internal List<InterfEZ> InterfEZ() => _interfEZ;

    internal List<InterfFZ> InterfFZ() => _interfFZ;

    internal List<InterfZZ> InterfZZ() => _interfZZ;

    /// <summary>The number of interference types (10).</summary>
    public static int InterfTypeCount => 10;

    /// <summary>Adds the interference between the two shapes to the table; false if it was there.</summary>
    internal bool AddInterf(int i1, int i2)
    {
        if (_interfTB.Add(new Pair(i1, i2)))
        {
            _interfered.Add(i1);
            _interfered.Add(i2);
            return true;
        }
        return false;
    }

    /// <summary>True if the shape interferes with some shape.</summary>
    public bool HasInterf(int i) => _interfered.Contains(i);

    /// <summary>True if the two shapes interfere.</summary>
    public bool HasInterf(int i1, int i2)
    {
        Pair pk = new(i1, i2);
        return _interfTB.Contains(pk);
    }

    /// <summary>The table of interferences (the data structure's own map; read only for callers).</summary>
    public MapOfPair Interferences() => _interfTB;

    /// <summary>Writes the ranges and the shapes as OCCT's <c>Dump</c> prints them.</summary>
    public void Dump(TextWriter os)
    {
        os.Write(" *** DS ***\n");
        int rangesCount = RangeCount;
        os.Write(" Ranges:" + rangesCount + "\n");
        for (int i = 0; i < rangesCount; ++i)
        {
            IndexRange range = Range(i);
            range.Dump(os);
            os.Write("\n");
        }
        //
        int sourceShapesCount = SourceShapeCount;
        os.Write(" Shapes:" + sourceShapesCount + "\n");
        int shapesCount = ShapeCount;
        for (int i = 0; i < shapesCount; ++i)
        {
            ShapeInfo shapeInfo = ShapeInfo(i);
            os.Write(" " + i + " :");
            shapeInfo.Dump(os);
            os.Write("\n");
            if (i == sourceShapesCount - 1)
            {
                os.Write(" ****** adds\n");
            }
        }
        os.Write(" ******\n");
    }

    private bool CheckCoincidence(PaveBlock paveBlock1, PaveBlock paveBlock2, double fuzzyValue)
    {
        // Find an intermediate point of the first pave block.
        // Seems like it doesn't matter where exactly this point is located on the pave block.
        paveBlock1.Range(out double paveBlock1First, out double paveBlock1Last);
        double paveBlock1Intermediate = IntersectionTools.IntermediatePoint(paveBlock1First, paveBlock1Last);
        Edge edge1 = ShapeCasts.Edge(Shape(paveBlock1.OriginalEdge()));
        AlgoTools.PointOnEdge(edge1, paveBlock1Intermediate, out Point3D paveBlock1IntermediatePoint);

        // Obtain the curve of the second edge and project the point onto it.
        paveBlock2.Range(out double paveBlock2First, out double paveBlock2Last);
        Edge edge2 = ShapeCasts.Edge(Shape(paveBlock2.OriginalEdge()));
        Curve3D edgeCurve2 = Tool.Curve(edge2, out double edge2ParamFirst, out double edge2ParamLast)!;
        ProjectPointOnCurve projector = new();
        projector.Init(edgeCurve2, edge2ParamFirst, edge2ParamLast);
        projector.Perform(paveBlock1IntermediatePoint);
        // Check the results of projection.
        if (projector.NbPoints() != 0)
        {
            double distance = projector.LowerDistance();
            double tolerance = Tool.MaxTolerance(edge1, ShapeType.Vertex)
                                + Tool.MaxTolerance(edge2, ShapeType.Vertex)
                                + Max(fuzzyValue, global::Kerf.Engine.Foundation.Precision.Confusion());
            // If the distance is less than tolerance, check that the projection
            // point lies within the parameter range of the second pave block.
            if (distance < tolerance)
            {
                double distanceParam = projector.LowerDistanceParameter();
                if (distanceParam > paveBlock2First && distanceParam < paveBlock2Last)
                {
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>True if the shape <paramref name="candidate"/> is a sub-shape of <paramref name="parent"/>.</summary>
    public bool IsSubShape(int candidate, int parent)
    {
        foreach (int subShapeIndex in ShapeInfo(parent).SubShapes())
        {
            if (subShapeIndex == candidate)
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>Appends to the list the distinct paves of the edge's pave blocks, sorted by parameter.</summary>
    /// <remarks>
    /// As OCCT, the array holds one pave more than the pave blocks; if fewer distinct paves are found (OCCT asserts
    /// in debug builds only), the default paves (-1, 99) left in it are sorted and appended too.
    /// </remarks>
    public void Paves(int edge, NCollection_List<Pave> resultPaves)
    {
        // Note: uniqueness is defined by operator == (index and parameter), while sorting is done
        // by operator <, which compares only parameters.

        NCollection_List<PaveBlock> paveBlocks = PaveBlocks(edge);
        if (paveBlocks.IsEmpty())
        {
            return;
        }

        Pave[] pPaves = new Pave[paveBlocks.Extent() + 1];
        Array.Fill(pPaves, new Pave());
        int i = 0;
        HashSet<Pave> visitedPaves = new();
        foreach (PaveBlock paveBlock in paveBlocks)
        {
            foreach (Pave pave in new[] { paveBlock.Pave1(), paveBlock.Pave2() })
            {
                if (visitedPaves.Add(pave))
                {
                    pPaves[i] = pave;
                    ++i;
                }
            }
        }

        Sorting.Sort(pPaves.AsSpan(), new Pave.LessThan());
        foreach (Pave pave in pPaves)
        {
            resultPaves.Append(pave);
        }
    }

    /// <summary>
    /// The bounding box of the solid with the index, from its faces' boxes; whole if a shell is open or (with
    /// <paramref name="checkInverted"/>) the solid is inverted.
    /// </summary>
    public void BuildBndBoxSolid(int index, ref Box box, bool checkInverted = true)
    {
        ShapeInfo shapeInfo = ShapeInfo(index);

        bool isOpenBox = false;
        foreach (int subShapeIndex in shapeInfo.SubShapes())
        {
            ShapeInfo shellInfo = ShapeInfo(subShapeIndex);
            if (shellInfo.ShapeType() != ShapeType.Shell)
            {
                continue;
            }

            foreach (int faceIndex in shellInfo.SubShapes())
            {
                ShapeInfo faceInfo = ShapeInfo(faceIndex);
                if (faceInfo.ShapeType() != ShapeType.Face)
                {
                    continue;
                }

                Box faceBoundBox = faceInfo.Box();
                box.Add(faceBoundBox);

                isOpenBox = faceBoundBox.IsOpen;
                if (isOpenBox)
                {
                    break;
                }
            }

            if (!isOpenBox)
            {
                Shell shell = ShapeCasts.Shell(shellInfo.Shape());
                isOpenBox = AlgoTools.IsOpenShell(shell);
            }

            if (isOpenBox)
            {
                break;
            }
        }

        if (isOpenBox)
        {
            box.SetWhole();
        }
        else if (checkInverted)
        {
            Solid solid = ShapeCasts.Solid(shapeInfo.Shape());
            if (AlgoTools.IsInvertedSolid(solid))
            {
                box.SetWhole();
            }
        }
    }

    /// <summary>Replaces the vertices of every pave block by their same-domain vertices.</summary>
    internal void UpdatePaveBlocksWithSDVertices()
    {
        foreach (NCollection_List<PaveBlock> paveBlockList in _paveBlocksPool)
        {
            foreach (PaveBlock paveBlock in paveBlockList)
            {
                UpdatePaveBlockWithSDVertices(paveBlock);
            }
        }
    }

    /// <summary>Replaces the vertices of the pave block by their same-domain vertices.</summary>
    internal void UpdatePaveBlockWithSDVertices(PaveBlock paveBlock)
    {
        Pave pave1 = paveBlock.Pave1();
        int index1 = GetSameDomainIndex(pave1.Index());
        pave1.SetIndex(index1);
        paveBlock.SetPave1(pave1);

        Pave pave2 = paveBlock.Pave2();
        int index2 = GetSameDomainIndex(pave2.Index());
        pave2.SetIndex(index2);
        paveBlock.SetPave2(pave2);
    }

    /// <summary>Replaces the vertices of the common block's pave blocks by their same-domain vertices.</summary>
    internal void UpdateCommonBlockWithSDVertices(CommonBlock commonBlock)
    {
        foreach (PaveBlock paveBlock in commonBlock.PaveBlocks())
        {
            UpdatePaveBlockWithSDVertices(paveBlock);
        }
    }

    /// <summary>Makes the pave blocks of the edges of the vertex.</summary>
    internal void InitPaveBlocksForVertex(int vertexIndex)
    {
        if (!_mapVE.Find(vertexIndex, out NCollection_List<int> edgeIndices))
        {
            return;
        }

        foreach (int edgeIndex in edgeIndices)
        {
            ChangePaveBlocks(edgeIndex);
        }
    }

    /// <summary>Clears the pave blocks of the untouched edges (one pave block, not common, between source vertices).</summary>
    internal void ReleasePaveBlocks()
    {
        // It is necessary to remove the reference to PaveBlocks for the untouched
        // edges to avoid creation of the same images for them.
        // Pave blocks for this reference should be cleared.
        // This will allow to differ the small edges, for which it is
        // impossible to even build a pave block from the normal edges for which the
        // pave block have been created, but stayed untouched.
        // The small edge, for which no pave blocks have been created,
        // should be avoided in the result, thus the reference to empty list
        // of pave blocks will stay to mark the edge as Deleted.

        foreach (NCollection_List<PaveBlock> paveBlockList in ChangePaveBlocksPool())
        {
            if (paveBlockList.Extent() != 1)
            {
                continue;
            }

            PaveBlock paveBlock = paveBlockList.First();
            if (IsCommonBlock(paveBlock))
            {
                continue;
            }

            paveBlock.Indices(out int shapeIndex1, out int shapeIndex2);
            if (!IsNewShape(shapeIndex1) && !IsNewShape(shapeIndex2))
            {
                // Both vertices are original, thus the PB is untouched.
                // Remove reference for the original edge
                int origEdgeIndex = paveBlock.OriginalEdge();
                if (origEdgeIndex >= 0)
                {
                    ChangeShapeInfo(origEdgeIndex).SetReference(-1);
                }
                // Clear contents of the list
                paveBlockList.Clear();
            }
        }
    }

    /// <summary>
    /// True if the pave block's shrunk data is still valid: the ends of the shrunk range stay outside the vertices'
    /// tolerances (they may not after the vertices were replaced or their tolerances grew).
    /// </summary>
    public bool IsValidShrunkData(PaveBlock paveBlock)
    {
        if (!paveBlock.HasShrunkData)
        {
            return false;
        }

        // Compare the distances from the bounds of the shrunk range to the vertices
        // with the tolerance values of vertices

        // Shrunk range
        Span<double> parameter = stackalloc double[2];
        Span<int> vertexIndex = stackalloc int[2];
        paveBlock.ShrunkData(out parameter[0], out parameter[1], out _, out _);
        paveBlock.Indices(out vertexIndex[0], out vertexIndex[1]);

        Edge edge = ShapeCasts.Edge(Shape(paveBlock.OriginalEdge()));
        EdgeCurveAdaptor curveAdaptor = new(edge);
        double epsilon = Tool.Tolerance(edge) * 0.01;

        for (int i = 0; i < 2; ++i)
        {
            Vertex vertex = ShapeCasts.Vertex(Shape(vertexIndex[i]));
            double tol = Tool.Tolerance(vertex) + global::Kerf.Engine.Foundation.Precision.Confusion();
            // Bounding point
            Point3D vertexPoint = Tool.Pnt(vertex);

            // Point on the end of shrunk range
            Point3D pointOnEdge = curveAdaptor.Value(parameter[i]);

            if (tol - vertexPoint.Distance(pointOnEdge) > epsilon)
            {
                return false;
            }
        }
        return true;
    }

    // Computes the parameter of theVertex on theEdge.
    // If theVertex is not coincident with either end of theEdge, returns 0.0.
    // Unlike BRep_Tool::Parameter, this does not require the vertex to be topologically
    // connected to the edge.
    // Note: this is generally unsafe function that is left to preserve previous behavior.
    private static double ComputeParameter(in Vertex vertex, in Edge edge)
    {
        double edgeToleranceSqr = global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(Tool.Tolerance(edge));
        Point3D vertexPoint = Tool.Pnt(vertex);

        Edge forwardEdge = edge;
        forwardEdge.Orientation(Forward);
        Curve3D curve = Tool.Curve(forwardEdge, out double curveStart, out double curveEnd)!;

        bool IsMatch(double parameter)
        {
            curve.D0(parameter, out Point3D pointOnCurve);
            return pointOnCurve.SquareDistance(vertexPoint) < edgeToleranceSqr;
        }

        // Note: only start and end parameters are checked.
        // It is unsafe to return 0. when point is not matching either end
        // but this is left to preserve previous behavior.
        return IsMatch(curveStart) ? curveStart : IsMatch(curveEnd) ? curveEnd : 0.0;
    }

    private int PrepareVertices(double additionalTolerance)
    {
        int vertexCount = 0;

        for (int vertexIndex = 0; vertexIndex < _nbSourceShapes; ++vertexIndex)
        {
            ShapeInfo vertexInfo = ChangeShapeInfo(vertexIndex);
            if (vertexInfo.ShapeType() != ShapeType.Vertex)
            {
                continue;
            }
            ++vertexCount;

            Vertex vertex = ShapeCasts.Vertex(vertexInfo.Shape());

            ref Box vertexBoundBox = ref vertexInfo.ChangeBox();
            vertexBoundBox.SetGap(Tool.Tolerance(vertex) + additionalTolerance);
            vertexBoundBox.Add(Tool.Pnt(vertex));
        }

        return vertexCount;
    }

    private int PrepareEdges(double additionalTolerance)
    {
        int edgeCount = 0;

        for (int edgeIndex = 0; edgeIndex < _nbSourceShapes; ++edgeIndex)
        {
            ShapeInfo edgeInfo = ChangeShapeInfo(edgeIndex);
            if (edgeInfo.ShapeType() != ShapeType.Edge)
            {
                continue;
            }
            ++edgeCount;

            Edge edge = ShapeCasts.Edge(edgeInfo.Shape());
            double edgeTolerance = Tool.Tolerance(edge);

            if (!Tool.Degenerated(edge))
            {
                // This is processing of special case when edge has infinite curve.
                // In this case vertices are created at the infinite positions
                // and added to the edge sub-shape list.

                NCollection_List<int> vertexIndices = edgeInfo.ChangeSubShapes();

                Edge forwardEdge = edge;
                forwardEdge.Orientation(Forward);
                Curve3D? forwardEdgeCurve = Tool.Curve(forwardEdge, out double curveStart, out double curveEnd);

                // Process infinite values of the edge curve parameters.
                // Creates vertices at the infinite positions and add them to the edge sub-shape list.
                void ProcessInfinite(double param)
                {
                    forwardEdgeCurve!.D0(param, out Point3D point);
                    Vertex vertex = new();
                    BrepBuilder builder = new();
                    builder.MakeVertex(ref vertex, point, edgeTolerance);
                    vertex.Orientation(Forward);

                    ShapeInfo vertexInfo = new();
                    vertexInfo.SetShape(vertex);
                    vertexInfo.SetShapeType(ShapeType.Vertex);
                    vertexInfo.SetFlag(1); // infinite flag

                    vertexIndices.Append(Append(vertexInfo));
                }

                if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(curveStart))
                {
                    ProcessInfinite(curveStart);
                }
                if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(curveEnd))
                {
                    ProcessInfinite(curveEnd);
                }
            }
            else
            {
                edgeInfo.SetFlag(edgeIndex);
            }

            // Update edge bounding box with its own bounding box.
            ref Box edgeBoundBox = ref edgeInfo.ChangeBox();
            Bounds.Add(edge, ref edgeBoundBox);

            // Add bounding boxes of vertices to the edge bounding box.
            foreach (int vertexIndex in edgeInfo.SubShapes())
            {
                ShapeInfo vertexInfo = ChangeShapeInfo(vertexIndex);
                edgeBoundBox.Add(vertexInfo.Box());
            }

            edgeBoundBox.SetGap(edgeBoundBox.GetGap() + additionalTolerance);
        }

        return edgeCount;
    }

    private int PrepareFaces(double additionalTolerance)
    {
        int faceCount = 0;

        for (int faceIndex = 0; faceIndex < _nbSourceShapes; ++faceIndex)
        {
            // Iterate over all faces.
            ShapeInfo faceInfo = ChangeShapeInfo(faceIndex);
            if (faceInfo.ShapeType() != ShapeType.Face)
            {
                continue;
            }
            ++faceCount;

            // Map of sub-shape indices for the face that will replace the current list of wire indices.
            NCollection_IntMap newSubShapeIndices = new(100);

            Shape face = faceInfo.Shape();

            // Update face bounding box with its own bounding box.
            ref Box faceBoundBox = ref faceInfo.ChangeBox();
            Bounds.Add(face, ref faceBoundBox);

            // Container of the face sub-shape indices. Currently contains wire indices.
            // Will be updated to contain edge and vertex indices instead.
            NCollection_List<int> faceSubShapes = faceInfo.ChangeSubShapes();

            // Iterate over edges and vertices of the face and fill the new map of sub-shape indices.
            // Also marks degenerated edges on the face.
            foreach (int wireIndex in faceSubShapes)
            {
                foreach (int edgeIndex in ChangeShapeInfo(wireIndex).SubShapes())
                {
                    // Update face bounding box with edge bounding boxes.
                    ShapeInfo edgeInfo = ChangeShapeInfo(edgeIndex);
                    faceBoundBox.Add(edgeInfo.Box());

                    // Add edge index to the map of indices.
                    newSubShapeIndices.Add(edgeIndex);

                    // Mark degenerated edges on the face (OCCT sets the flag of a degenerated edge to the face's
                    // index).
                    Edge edge = ShapeCasts.Edge(edgeInfo.Shape());
                    if (Tool.Degenerated(edge))
                    {
                        edgeInfo.SetFlag(faceIndex);
                    }

                    // Add vertices of the edge to the map of indices.
                    foreach (int vertexIndex in edgeInfo.SubShapes())
                    {
                        newSubShapeIndices.Add(vertexIndex);
                    }
                }
            }

            // Add vertices of the face to the map of indices.
            for (Iterator faceIter = new(face); faceIter.More(); faceIter.Next())
            {
                Shape subShape = faceIter.Value();
                if (subShape.ShapeType() == ShapeType.Vertex)
                {
                    newSubShapeIndices.Add(Index(subShape));
                }
            }

            // Remove wire indices and fill in edge and vertex indices instead.
            // In the order of OCCT's hash buckets, as native (decision 784).
            faceSubShapes.Clear();
            foreach (int subShapeIndex in newSubShapeIndices)
            {
                faceSubShapes.Append(subShapeIndex);
            }

            faceBoundBox.SetGap(faceBoundBox.GetGap() + additionalTolerance);
        }

        return faceCount;
    }

    private int PrepareSolids()
    {
        int solidCount = 0;

        // For the check mode we need to compute the bounding box for solid.
        // Otherwise, it will be computed on the building stage
        if (_arguments.Extent() != 1)
        {
            return 0;
        }

        for (int solidIndex = 0; solidIndex < _nbSourceShapes; ++solidIndex)
        {
            ShapeInfo solidInfo = ChangeShapeInfo(solidIndex);
            if (solidInfo.ShapeType() != ShapeType.Solid)
            {
                continue;
            }
            ++solidCount;

            // Map of sub-shape indices for the solid that will replace the current list of shell indices.
            NCollection_IntMap newSubShapeIndices = new(100);

            // Update solid bounding box with its own bounding box.
            ref Box solidBoundBox = ref solidInfo.ChangeBox();
            BuildBndBoxSolid(solidIndex, ref solidBoundBox);

            // Container of the solid sub-shape indices. Currently contains shell indices.
            // Will be updated to contain face and edge indices instead.
            NCollection_List<int> solidSubShapes = solidInfo.ChangeSubShapes();

            // Iterate over faces and edges of the solid and fill the new map of sub-shape indices.
            foreach (int shellIndex in solidSubShapes)
            {
                ShapeInfo shellInfo = ChangeShapeInfo(shellIndex);
                if (shellInfo.ShapeType() != ShapeType.Shell)
                {
                    continue;
                }

                foreach (int faceIndex in shellInfo.SubShapes())
                {
                    ShapeInfo faceInfo = ChangeShapeInfo(faceIndex);
                    if (faceInfo.ShapeType() != ShapeType.Face)
                    {
                        continue;
                    }
                    // Add face index to the map of indices.
                    newSubShapeIndices.Add(faceIndex);

                    foreach (int edgeIndex in faceInfo.SubShapes())
                    {
                        // Add edge index to the map of indices.
                        newSubShapeIndices.Add(edgeIndex);
                    }
                }
            }

            // Remove shell indices and fill in face and edge indices instead (OCCT's bucket order: decision 784).
            solidSubShapes.Clear();
            foreach (int subShapeIndex in newSubShapeIndices)
            {
                solidSubShapes.Append(subShapeIndex);
            }
        }

        return solidCount;
    }

    private void BuildVertexEdgeMap()
    {
        for (int shapeIndex = 0; shapeIndex < _nbSourceShapes; ++shapeIndex)
        {
            ShapeInfo edgeInfo = ChangeShapeInfo(shapeIndex);
            if (edgeInfo.ShapeType() != ShapeType.Edge)
            {
                continue;
            }

            foreach (int subShapeIndex in edgeInfo.SubShapes())
            {
                if (!_mapVE.Find(subShapeIndex, out NCollection_List<int> vertexIndices))
                {
                    vertexIndices = new NCollection_List<int>();
                    _mapVE.Bind(subShapeIndex, vertexIndices);
                    vertexIndices.Append(shapeIndex);
                }
                else
                {
                    // provide uniqueness of the edges in the list
                    if (!vertexIndices.Contains(shapeIndex))
                    {
                        vertexIndices.Append(shapeIndex);
                    }
                }
            }
        }
    }
}
