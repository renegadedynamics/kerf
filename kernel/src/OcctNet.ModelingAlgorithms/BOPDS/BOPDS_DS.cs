// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_DS.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_DS.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_DS.cxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_PDS.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

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
/// Maps keyed by pave blocks compare by reference and iterate in insertion order (decision 5005). The maps of
/// integers iterate in insertion order where OCCT's iterate in the order of their hash buckets: the sub-shape lists
/// of faces and solids built in <c>Init</c> come out in another order than native's (decision 6153,
/// occt-issues.md).
/// </para>
/// </remarks>
public class BOPDS_DS
{
    //! Initial increment for myLines container (an allocation hint in OCCT).
    private const int THE_INITIAL_LINES_INCREMENT = 500;

    private static readonly NCollection_List<BOPDS_PaveBlock> theStaticEmptyList = new();
    private static readonly BOPDS_FaceInfo theEmptyFaceInfo = new();

    private readonly NCollection_List<TopoDS_Shape> myArguments = new();
    private int myNbSourceShapes; // OCCT also keeps myNbShapes, set to 0 and never read: left out
    private readonly NCollection_DynamicArray<BOPDS_IndexRange> myRanges = new(0);
    private readonly NCollection_DynamicArray<BOPDS_ShapeInfo> myLines = new(0);
    private readonly NCollection_DataMap<TopoDS_Shape, int> myMapShapeIndex = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> myPaveBlocksPool = new(0);
    private readonly NCollection_DataMap<BOPDS_PaveBlock, BOPDS_CommonBlock> myMapPBCB = new();
    private readonly NCollection_DynamicArray<BOPDS_FaceInfo> myFaceInfoPool = new(0);
    private readonly NCollection_DataMap<int, int> myShapesSD = new();
    private readonly NCollection_DataMap<int, NCollection_List<int>> myMapVE = new();
    private readonly NCollection_Map<BOPDS_Pair> myInterfTB = new();
    private readonly NCollection_DynamicArray<BOPDS_InterfVV> myInterfVV = new(0);
    private readonly NCollection_DynamicArray<BOPDS_InterfVE> myInterfVE = new(0);
    private readonly NCollection_DynamicArray<BOPDS_InterfVF> myInterfVF = new(0);
    private readonly NCollection_DynamicArray<BOPDS_InterfEE> myInterfEE = new(0);
    private readonly NCollection_DynamicArray<BOPDS_InterfEF> myInterfEF = new(0);
    private readonly NCollection_DynamicArray<BOPDS_InterfFF> myInterfFF = new(0);
    private readonly NCollection_DynamicArray<BOPDS_InterfVZ> myInterfVZ = new(0);
    private readonly NCollection_DynamicArray<BOPDS_InterfEZ> myInterfEZ = new(0);
    private readonly NCollection_DynamicArray<BOPDS_InterfFZ> myInterfFZ = new(0);
    private readonly NCollection_DynamicArray<BOPDS_InterfZZ> myInterfZZ = new(0);
    private readonly NCollection_Map<int> myInterfered = new();

    /// <summary>An empty data structure.</summary>
    public BOPDS_DS()
    {
        myNbSourceShapes = 0;
    }

    /// <summary>Clears the contents.</summary>
    public void Clear()
    {
        myNbSourceShapes = 0;
        //
        myArguments.Clear();
        myRanges.Clear();
        myLines.Clear();
        myMapShapeIndex.Clear();
        myPaveBlocksPool.Clear();
        myFaceInfoPool.Clear();
        myShapesSD.Clear();
        myMapVE.Clear();
        myMapPBCB.Clear();
        myInterfTB.Clear();
        myInterfVV.Clear();
        myInterfVE.Clear();
        myInterfVF.Clear();
        myInterfEE.Clear();
        myInterfEF.Clear();
        myInterfFF.Clear();
        myInterfVZ.Clear();
        myInterfEZ.Clear();
        myInterfFZ.Clear();
        myInterfZZ.Clear();
        myInterfered.Clear();
    }

    /// <summary>Sets the arguments of the operation (a copy of the list).</summary>
    public void SetArguments(NCollection_List<TopoDS_Shape> theArguments) => myArguments.Assign(theArguments);

    /// <summary>The arguments of the operation (the data structure's own list; read only for callers).</summary>
    public NCollection_List<TopoDS_Shape> Arguments() => myArguments;

    /// <summary>Initializes the data structure for the arguments.</summary>
    public void Init() => Init(Precision.Confusion());

    /// <summary>Initializes the data structure for the arguments, with the fuzzy value.</summary>
    public void Init(double theFuzz)
    {
        // 1. Append Source Shapes
        if (myArguments.IsEmpty())
        {
            return;
        }

        int i1 = 0;
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(myArguments); anIt.More(); anIt.Next())
        {
            TopoDS_Shape aShape = anIt.Value();
            if (myMapShapeIndex.IsBound(aShape))
            {
                continue;
            }

            InitShape(Append(aShape), aShape);
            int i2 = NbShapes() - 1;
            myRanges.Append(new BOPDS_IndexRange(i1, i2));
            i1 = i2 + 1;
        }

        myNbSourceShapes = NbShapes();

        // 2. Prepare data for shapes. Includes updating bound boxes, updating sub-shapes.
        double anAdditionalTolerance = Max(theFuzz, Precision.Confusion()) * 0.5;
        prepareVertices(anAdditionalTolerance);                 // Vertex.
        int anEdgeCount = prepareEdges(anAdditionalTolerance);  // Edge.
        int aFaceCount = prepareFaces(anAdditionalTolerance);   // Face.
        prepareSolids();                                        // Solid.

        // 3. Prepare Vertex-Edge connection map.
        buildVertexEdgeMap();

        // 4. Prepare pools (OCCT sets the allocation increments to the numbers of edges and faces).
        _ = anEdgeCount;
        _ = aFaceCount;
    }

    /// <summary>The total number of shapes stored.</summary>
    public int NbShapes() => myLines.Length();

    /// <summary>The number of source shapes (the arguments and their sub-shapes).</summary>
    public int NbSourceShapes() => myNbSourceShapes;

    /// <summary>The number of index ranges (one per argument).</summary>
    public int NbRanges() => myRanges.Length();

    /// <summary>The index range <paramref name="theIndex"/>.</summary>
    public BOPDS_IndexRange Range(int theIndex) => myRanges.Value(theIndex);

    /// <summary>The rank (the argument's range) of the shape with the index, or -1.</summary>
    public int Rank(int theIndex)
    {
        for (int aRangeIndex = 0; aRangeIndex < NbRanges(); ++aRangeIndex)
        {
            if (Range(aRangeIndex).Contains(theIndex))
            {
                return aRangeIndex;
            }
        }
        return -1;
    }

    /// <summary>True if the shape with the index is not a source shape.</summary>
    public bool IsNewShape(int theIndex) => theIndex >= NbSourceShapes();

    /// <summary>Appends a copy of the shape information; returns its index.</summary>
    internal int Append(BOPDS_ShapeInfo theShapeInfo)
    {
        myLines.Append(new BOPDS_ShapeInfo(theShapeInfo));
        int aShapeIndex = myLines.Length() - 1;
        myMapShapeIndex.Bind(theShapeInfo.Shape(), aShapeIndex);
        return aShapeIndex;
    }

    /// <summary>Appends the default information of the shape; returns its index.</summary>
    internal int Append(in TopoDS_Shape theShape)
    {
        myLines.Append(new BOPDS_ShapeInfo()).SetShape(theShape);
        int aShapeIndex = myLines.Length() - 1;
        myMapShapeIndex.Bind(theShape, aShapeIndex);
        return aShapeIndex;
    }

    /// <summary>The information on the shape with the index (the data structure's own object; read only for callers).</summary>
    public BOPDS_ShapeInfo ShapeInfo(int theIndex) => myLines.Value(theIndex);

    internal BOPDS_ShapeInfo ChangeShapeInfo(int theIndex) => myLines.Value(theIndex);

    /// <summary>The shape with the index.</summary>
    public TopoDS_Shape Shape(int theIndex) => ShapeInfo(theIndex).Shape();

    /// <summary>The index of the shape, or -1.</summary>
    public int Index(in TopoDS_Shape theShape)
    {
        int anIndex = -1;
        if (myMapShapeIndex.Find(theShape, out int aFound))
        {
            anIndex = aFound;
        }
        return anIndex;
    }

    private void InitShape(int theShapeInfoIndex, in TopoDS_Shape theShape)
    {
        BOPDS_ShapeInfo aShapeInfo = ChangeShapeInfo(theShapeInfoIndex);
        aShapeInfo.SetShapeType(theShape.ShapeType());
        NCollection_List<int> aSubShapeIndices = aShapeInfo.ChangeSubShapes();

        NCollection_Map<int> anExistingSubShapeIndices = new();
        foreach (int aSubShapeIndex in aSubShapeIndices)
        {
            anExistingSubShapeIndices.Add(aSubShapeIndex);
        }

        for (TopoDS_Iterator aShapeIter = new(theShape); aShapeIter.More(); aShapeIter.Next())
        {
            TopoDS_Shape aSubShape = aShapeIter.Value();
            int aSubShapeIndex = myMapShapeIndex.Find(aSubShape, out int anExistingIndex) ? anExistingIndex : Append(aSubShape);

            InitShape(aSubShapeIndex, aSubShape);
            if (anExistingSubShapeIndices.Add(aSubShapeIndex))
            {
                aSubShapeIndices.Append(aSubShapeIndex);
            }
        }
    }

    /// <summary>
    /// True if the shape <paramref name="theIndex1"/> interferes with any (or, if not
    /// <paramref name="theAnyInterference"/>, every) sub-shape of the shape <paramref name="theIndex2"/>.
    /// </summary>
    public bool HasInterfShapeSubShapes(int theIndex1, int theIndex2, bool theAnyInterference = true)
    {
        NCollection_List<int> aSubShapes = ShapeInfo(theIndex2).SubShapes();
        if (theAnyInterference)
        {
            foreach (int aSubShapeIndex in aSubShapes)
            {
                if (HasInterf(theIndex1, aSubShapeIndex))
                {
                    return true;
                }
            }
            return false;
        }
        else
        {
            foreach (int aSubShapeIndex in aSubShapes)
            {
                if (!HasInterf(theIndex1, aSubShapeIndex))
                {
                    return false;
                }
            }
            return true;
        }
    }

    /// <summary>True if sub-shapes of the two shapes interfere.</summary>
    public bool HasInterfSubShapes(int theIndex1, int theIndex2)
    {
        foreach (int aSubShapeIndex in ShapeInfo(theIndex1).SubShapes())
        {
            if (HasInterfShapeSubShapes(aSubShapeIndex, theIndex2))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>The pave blocks of the source edges (the data structure's own pool; read only for callers).</summary>
    public NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> PaveBlocksPool() => myPaveBlocksPool;

    internal NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> ChangePaveBlocksPool() => myPaveBlocksPool;

    /// <summary>True if the shape with the index has pave blocks.</summary>
    public bool HasPaveBlocks(int theIndex) => ShapeInfo(theIndex).HasReference();

    /// <summary>The pave blocks of the shape with the index, or an empty list (read only for callers).</summary>
    public NCollection_List<BOPDS_PaveBlock> PaveBlocks(int theIndex)
    {
        if (HasPaveBlocks(theIndex))
        {
            return myPaveBlocksPool.Value(ShapeInfo(theIndex).Reference());
        }

        return theStaticEmptyList;
    }

    /// <summary>The pave blocks of the edge with the index, made first if the edge has none.</summary>
    internal NCollection_List<BOPDS_PaveBlock> ChangePaveBlocks(int theIndex)
    {
        BOPDS_ShapeInfo aShapeInfo = ChangeShapeInfo(theIndex);
        if (!aShapeInfo.HasReference())
        {
            InitPaveBlocks(theIndex);
        }
        return myPaveBlocksPool.Value(aShapeInfo.Reference());
    }

    private void InitPaveBlocks(int theEdgeIndex)
    {
        BOPDS_ShapeInfo anEdgeInfo = ChangeShapeInfo(theEdgeIndex);
        TopoDS_Edge anEdge = TopoDS.Edge(anEdgeInfo.Shape());

        NCollection_List<int> aVertexIndices = anEdgeInfo.SubShapes();
        if (aVertexIndices.IsEmpty())
        {
            return;
        }

        BOPDS_PaveBlock aPaveBlock = new();
        aPaveBlock.SetOriginalEdge(theEdgeIndex);

        if (anEdge.Orientation() != TopAbs_INTERNAL)
        {
            foreach (int aVertexIndexIt in aVertexIndices)
            {
                int aVertexIndex = aVertexIndexIt;
                BOPDS_ShapeInfo aVertexInfo = ShapeInfo(aVertexIndex);
                TopoDS_Vertex aVertex = TopoDS.Vertex(aVertexInfo.Shape());
                // Important: for vertices HasFlag() means that the original edge is located on the infinite
                // curve and vertex was created during initialization of this class object.
                // Such vertices are not topologically connected to the edge so we cannot use
                // BRep_Tool::Parameter.
                double aVertexParam = aVertexInfo.HasFlag() ? ComputeParameter(aVertex, anEdge)
                                                            : BRep_Tool.Parameter(aVertex, anEdge);

                aVertexIndex = GetSameDomainIndex(aVertexIndex);
                BOPDS_Pave aPave = new(aVertexIndex, aVertexParam);
                if (anEdgeInfo.HasFlag())
                {
                    // for a degenerated edge append pave unconditionally
                    aPaveBlock.AppendExtPave1(aPave);
                }
                else
                {
                    aPaveBlock.AppendExtPave(aPave);
                }

                // Handle closed edges (seam edges) that have a single vertex shared by both ends.
                // In this case, we need to add two paves: one for the start and one for the end
                // of the edge, even though they reference the same vertex.
                if (aVertexIndices.Extent() == 1)
                {
                    aVertex.Reverse();
                    aPaveBlock.AppendExtPave1(new BOPDS_Pave(aVertexIndex, BRep_Tool.Parameter(aVertex, anEdge)));
                }
            }
        }
        else
        {
            for (TopoDS_Iterator anEdgeIter = new(anEdge, false, true); anEdgeIter.More(); anEdgeIter.Next())
            {
                TopoDS_Vertex aVertex = TopoDS.Vertex(anEdgeIter.Value());
                int aVertexIndex = Index(aVertex);
                BOPDS_ShapeInfo aVertexInfo = ShapeInfo(aVertexIndex);
                double aVertexParam = aVertexInfo.HasFlag() ? ComputeParameter(aVertex, anEdge)
                                                            : BRep_Tool.Parameter(aVertex, anEdge);
                aPaveBlock.AppendExtPave1(new BOPDS_Pave(GetSameDomainIndex(aVertexIndex), aVertexParam));
            }
        }

        NCollection_List<BOPDS_PaveBlock> aNewList = myPaveBlocksPool.Append(new NCollection_List<BOPDS_PaveBlock>());
        aPaveBlock.Update(aNewList, false);
        anEdgeInfo.SetReference(myPaveBlocksPool.Length() - 1);
    }

    /// <summary>Splits every pave block with extra paves (see <see cref="BOPDS_PaveBlock.Update"/>).</summary>
    internal void UpdatePaveBlocks()
    {
        foreach (NCollection_List<BOPDS_PaveBlock> aPaveBlockList in myPaveBlocksPool)
        {
            for (NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new(aPaveBlockList); aItPB.More();)
            {
                BOPDS_PaveBlock aPaveBlock = aItPB.Value();

                if (!aPaveBlock.IsToUpdate())
                {
                    aItPB.Next();
                    continue;
                }

                NCollection_List<BOPDS_PaveBlock> aLPBN = new();
                aPaveBlock.Update(aLPBN);
                aPaveBlockList.Remove(aItPB);
                aPaveBlockList.Append(aLPBN);
            }
        }
    }

    /// <summary>Splits the pave block at its extra paves, in its edge's list.</summary>
    internal void UpdatePaveBlock(BOPDS_PaveBlock thePaveBlock)
    {
        if (!thePaveBlock.IsToUpdate())
        {
            return;
        }

        BOPDS_ShapeInfo aShapeInfo = ChangeShapeInfo(thePaveBlock.OriginalEdge());
        NCollection_List<BOPDS_PaveBlock> aLPB = myPaveBlocksPool.Value(aShapeInfo.Reference());

        for (NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new(aLPB); aItPB.More(); aItPB.Next())
        {
            BOPDS_PaveBlock aPaveBlock = aItPB.Value();
            if (aPaveBlock == thePaveBlock)
            {
                NCollection_List<BOPDS_PaveBlock> aLPBN = new();
                aPaveBlock.Update(aLPBN);
                aLPB.Append(aLPBN);
                aLPB.Remove(aItPB);
                break;
            }
        }
    }

    /// <summary>
    /// Splits the pave blocks of the common block at their extra paves and makes new common blocks of the coinciding
    /// splits.
    /// </summary>
    internal void UpdateCommonBlock(BOPDS_CommonBlock theCommonBlock, double theFuzzyValue)
    {
        NCollection_List<BOPDS_PaveBlock> aPaveBlocksOfCommonBlock = theCommonBlock.PaveBlocks();
        if (!aPaveBlocksOfCommonBlock.First().IsToUpdate())
        {
            return;
        }

        NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new();
        // OCCT iterates this map in the order of its hash buckets (keyed by pairs of vertex indices); the port in
        // insertion order. The order decides only the order in which the new common blocks are made.
        NCollection_DataMap<BOPDS_Pair, NCollection_List<BOPDS_PaveBlock>> aMPKLPB = new();

        foreach (BOPDS_PaveBlock aPaveBlockOfCommonBlock in aPaveBlocksOfCommonBlock)
        {
            int anEdgeIndex = aPaveBlockOfCommonBlock.OriginalEdge();
            NCollection_List<BOPDS_PaveBlock> aLPB = myPaveBlocksPool.Value(ChangeShapeInfo(anEdgeIndex).Reference());

            aItPB.Initialize(aLPB);
            for (; aItPB.More(); aItPB.Next())
            {
                BOPDS_PaveBlock aPB = aItPB.Value();
                if (aPB == aPaveBlockOfCommonBlock)
                {
                    NCollection_List<BOPDS_PaveBlock> aLPBN = new();
                    aPB.Update(aLPBN);
                    foreach (BOPDS_PaveBlock aPBN in aLPBN)
                    {
                        aLPB.Append(aPBN);

                        aPBN.Indices(out int n1, out int n2);
                        BOPDS_Pair aPK = new(n1, n2);
                        if (aMPKLPB.Find(aPK, out NCollection_List<BOPDS_PaveBlock> aPaveBlocks))
                        {
                            aPaveBlocks.Append(aPBN);
                        }
                        else
                        {
                            NCollection_List<BOPDS_PaveBlock> aNewLPBx = new();
                            aNewLPBx.Append(aPBN);
                            aMPKLPB.Bind(aPK, aNewLPBx);
                        }
                    }
                    aLPB.Remove(aItPB);
                    break;
                }
            }
        }

        foreach (KeyValuePair<BOPDS_Pair, NCollection_List<BOPDS_PaveBlock>> aItMPKLPB in aMPKLPB)
        {
            NCollection_List<BOPDS_PaveBlock> aLPBx = aItMPKLPB.Value;

            while (!aLPBx.IsEmpty())
            {
                NCollection_List<BOPDS_PaveBlock> aLPBxN = new();

                aItPB.Initialize(aLPBx);
                for (; aItPB.More();)
                {
                    BOPDS_PaveBlock aPBx = aItPB.Value();
                    if (aLPBxN.Extent() != 0)
                    {
                        BOPDS_PaveBlock aPBCx = aLPBxN.First();
                        if (CheckCoincidence(aPBx, aPBCx, theFuzzyValue))
                        {
                            aLPBxN.Append(aPBx);
                            aLPBx.Remove(aItPB);
                            continue;
                        }
                    }
                    else
                    {
                        aLPBxN.Append(aPBx);
                        aLPBx.Remove(aItPB);
                        continue;
                    }
                    aItPB.Next();
                }

                BOPDS_CommonBlock aCommonBlock = new();
                aCommonBlock.SetPaveBlocks(aLPBxN);
                aCommonBlock.SetFaces(theCommonBlock.Faces());

                aItPB.Initialize(aLPBxN);
                for (; aItPB.More(); aItPB.Next())
                {
                    SetCommonBlock(aItPB.Value(), aCommonBlock);
                }
            }
        }
    }

    /// <summary>The first pave block of the pave block's common block, or the pave block itself.</summary>
    public BOPDS_PaveBlock RealPaveBlock(BOPDS_PaveBlock thePaveBlock)
    {
        BOPDS_CommonBlock? aCommonBlock = CommonBlock(thePaveBlock);
        return aCommonBlock is not null ? aCommonBlock.PaveBlock1() : thePaveBlock;
    }

    /// <summary>True if the pave block's common block has more than one pave block.</summary>
    public bool IsCommonBlockOnEdge(BOPDS_PaveBlock thePaveBlock)
    {
        BOPDS_CommonBlock? aCommonBlock = CommonBlock(thePaveBlock);
        return aCommonBlock is not null && aCommonBlock.PaveBlocks().Extent() > 1;
    }

    /// <summary>True if the pave block is in a common block.</summary>
    public bool IsCommonBlock(BOPDS_PaveBlock thePaveBlock) => myMapPBCB.IsBound(thePaveBlock);

    /// <summary>The pave block's common block, or null.</summary>
    public BOPDS_CommonBlock? CommonBlock(BOPDS_PaveBlock thePaveBlock) =>
        myMapPBCB.Find(thePaveBlock, out BOPDS_CommonBlock aCommonBlock) ? aCommonBlock : null;

    /// <summary>Sets the pave block's common block.</summary>
    internal void SetCommonBlock(BOPDS_PaveBlock thePaveBlock, BOPDS_CommonBlock theCommonBlock) =>
        myMapPBCB.Bind(thePaveBlock, theCommonBlock);

    /// <summary>The states of the faces (the data structure's own pool; read only for callers).</summary>
    public NCollection_DynamicArray<BOPDS_FaceInfo> FaceInfoPool() => myFaceInfoPool;

    /// <summary>True if the shape with the index has face information.</summary>
    public bool HasFaceInfo(int theIndex) => ShapeInfo(theIndex).HasReference();

    /// <summary>The state of the face with the index, or an empty state (read only for callers).</summary>
    public BOPDS_FaceInfo FaceInfo(int theIndex)
    {
        if (!HasFaceInfo(theIndex))
        {
            return theEmptyFaceInfo;
        }

        return myFaceInfoPool.Value(ShapeInfo(theIndex).Reference());
    }

    /// <summary>The state of the face with the index, made first if the face has none.</summary>
    internal BOPDS_FaceInfo ChangeFaceInfo(int theIndex)
    {
        BOPDS_ShapeInfo aShapeInfo = ChangeShapeInfo(theIndex);
        if (!aShapeInfo.HasReference())
        {
            InitFaceInfo(theIndex);
        }
        return myFaceInfoPool.Value(aShapeInfo.Reference());
    }

    private void InitFaceInfo(int theIndex)
    {
        BOPDS_ShapeInfo aShapeInfo = ChangeShapeInfo(theIndex);
        BOPDS_FaceInfo aFaceInfo = myFaceInfoPool.Append(new BOPDS_FaceInfo());
        aShapeInfo.SetReference(myFaceInfoPool.Length() - 1);
        aFaceInfo.SetIndex(theIndex);

        InitFaceInfoIn(theIndex);
        UpdateFaceInfoOn(theIndex);
    }

    private void InitFaceInfoIn(int theIndex)
    {
        BOPDS_ShapeInfo aShapeInfo = ChangeShapeInfo(theIndex);
        if (!aShapeInfo.HasReference())
        {
            return;
        }

        BOPDS_FaceInfo aFaceInfo = myFaceInfoPool.Value(aShapeInfo.Reference());
        for (TopoDS_Iterator aShapeIter = new(Shape(theIndex)); aShapeIter.More(); aShapeIter.Next())
        {
            TopoDS_Shape aVertex = aShapeIter.Value();
            if (aVertex.ShapeType() == TopAbs_VERTEX)
            {
                int aVertexIndex = GetSameDomainIndex(Index(aVertex));
                aFaceInfo.ChangeVerticesIn().Add(aVertexIndex);
            }
        }
    }

    /// <summary>Recomputes the state In of the face with the index.</summary>
    internal void UpdateFaceInfoIn(int theIndex)
    {
        BOPDS_ShapeInfo aShapeInfo = ChangeShapeInfo(theIndex);
        if (!aShapeInfo.HasReference())
        {
            return;
        }

        BOPDS_FaceInfo aFaceInfo = myFaceInfoPool.Value(aShapeInfo.Reference());
        NCollection_IndexedMap<BOPDS_PaveBlock> aPaveBlocksIn = aFaceInfo.ChangePaveBlocksIn();
        NCollection_Map<int> aVerticesIn = aFaceInfo.ChangeVerticesIn();
        aPaveBlocksIn.Clear();
        aVerticesIn.Clear();
        FaceInfoIn(theIndex, aPaveBlocksIn, aVerticesIn);
    }

    /// <summary>Recomputes the state On of the face with the index.</summary>
    internal void UpdateFaceInfoOn(int theIndex)
    {
        BOPDS_ShapeInfo aShapeInfo = ChangeShapeInfo(theIndex);
        if (!aShapeInfo.HasReference())
        {
            return;
        }

        BOPDS_FaceInfo aFaceInfo = myFaceInfoPool.Value(aShapeInfo.Reference());
        NCollection_IndexedMap<BOPDS_PaveBlock> aPaveBlocksOn = aFaceInfo.ChangePaveBlocksOn();
        NCollection_Map<int> aVerticesOn = aFaceInfo.ChangeVerticesOn();
        aPaveBlocksOn.Clear();
        aVerticesOn.Clear();
        FaceInfoOn(theIndex, aPaveBlocksOn, aVerticesOn);
    }

    /// <summary>The state On of the face: the (real) pave blocks of its edges and the vertices.</summary>
    public void FaceInfoOn(int theIndex, NCollection_IndexedMap<BOPDS_PaveBlock> theMPB, NCollection_Map<int> theMVP)
    {
        foreach (int aSubShapeIndex in ShapeInfo(theIndex).SubShapes())
        {
            if (ShapeInfo(aSubShapeIndex).ShapeType() == TopAbs_EDGE)
            {
                foreach (BOPDS_PaveBlock aPaveBlock in PaveBlocks(aSubShapeIndex))
                {
                    aPaveBlock.Indices(out int nV1, out int nV2);
                    theMVP.Add(nV1);
                    theMVP.Add(nV2);
                    theMPB.Add(RealPaveBlock(aPaveBlock));
                }
            }
            else // if (ShapeInfo(aSubShapeIndex).ShapeType() == TopAbs_VERTEX)
            {
                theMVP.Add(GetSameDomainIndex(aSubShapeIndex));
            }
        }
    }

    /// <summary>
    /// The state In of the face: its internal vertices, the vertices of vertex/face interferences, the new vertices
    /// of edge/face interferences and the common blocks lying on the face.
    /// </summary>
    public void FaceInfoIn(int theFaceIndex, NCollection_IndexedMap<BOPDS_PaveBlock> thePaveBlocks, NCollection_Map<int> theIndexMap)
    {
        // 1. Pure internal vertices on the face
        // Adding to theIndexMap all the indicies of vertices belonging to the face.
        TopoDS_Shape aFace = Shape(theFaceIndex);
        for (TopoDS_Iterator aFaceIter = new(aFace); aFaceIter.More(); aFaceIter.Next())
        {
            TopoDS_Shape aSubShape = aFaceIter.Value();
            if (aSubShape.ShapeType() == TopAbs_VERTEX)
            {
                int aVertexIndex = GetSameDomainIndex(Index(aSubShape));
                theIndexMap.Add(aVertexIndex);
            }
        }

        // 2. Vertex-Face interferences
        // Adding to theIndexMap all the indicies of vertices having interferences with the face.
        foreach (BOPDS_InterfVF aVertexFaceInterference in InterfVF())
        {
            if (aVertexFaceInterference.Contains(theFaceIndex))
            {
                int aVertexIndex = GetSameDomainIndex(aVertexFaceInterference.OppositeIndex(theFaceIndex));
                theIndexMap.Add(aVertexIndex);
            }
        }

        // 3. Edge-Face interferences
        foreach (BOPDS_InterfEF anEdgeFaceInterference in InterfEF())
        {
            if (!anEdgeFaceInterference.Contains(theFaceIndex))
            {
                continue;
            }

            if (anEdgeFaceInterference.GetIndexNew() is int anIndexNew)
            {
                theIndexMap.Add(GetSameDomainIndex(anIndexNew));
            }
            else
            {
                foreach (BOPDS_PaveBlock aPaveBlock in PaveBlocks(anEdgeFaceInterference.OppositeIndex(theFaceIndex)))
                {
                    BOPDS_CommonBlock? aCommonBlock = CommonBlock(aPaveBlock);
                    if (aCommonBlock is not null && aCommonBlock.Contains(theFaceIndex))
                    {
                        thePaveBlocks.Add(aCommonBlock.PaveBlock1());
                    }
                }
            }
        }
    }

    /// <summary>Recomputes the state In of every face of the map.</summary>
    internal void UpdateFaceInfoIn(NCollection_Map<int> theFaces)
    {
        foreach (int aFaceIndex in theFaces)
        {
            BOPDS_ShapeInfo aShapeInfo = ChangeShapeInfo(aFaceIndex);
            if (!aShapeInfo.HasReference())
            {
                myFaceInfoPool.Append(new BOPDS_FaceInfo()).SetIndex(aFaceIndex);
                aShapeInfo.SetReference(myFaceInfoPool.Length() - 1);
            }
            BOPDS_FaceInfo aFaceInfo = myFaceInfoPool.Value(aShapeInfo.Reference());
            aFaceInfo.ChangePaveBlocksIn().Clear();
            aFaceInfo.ChangeVerticesIn().Clear();

            // 1. Add pure internal vertices on the face
            InitFaceInfoIn(aFaceIndex);
        }

        // 2. Analyze Vertex-Face interferences
        foreach (BOPDS_InterfVF aVertexFaceInterference in InterfVF())
        {
            int aFaceIndex = aVertexFaceInterference.Index2();
            if (theFaces.Contains(aFaceIndex))
            {
                int aSameDomainIndex = GetSameDomainIndex(aVertexFaceInterference.Index1());
                myFaceInfoPool.Value(ShapeInfo(aFaceIndex).Reference()).ChangeVerticesIn().Add(aSameDomainIndex);
            }
        }

        // 3. Analyze Edge-Face interferences
        foreach (BOPDS_InterfEF anEdgeFaceInterference in InterfEF())
        {
            int aFaceIndex = anEdgeFaceInterference.Index2();
            if (!theFaces.Contains(aFaceIndex))
            {
                continue;
            }

            BOPDS_FaceInfo aFaceInfo = myFaceInfoPool.Value(ShapeInfo(aFaceIndex).Reference());
            if (anEdgeFaceInterference.GetIndexNew() is int anIndexNew)
            {
                aFaceInfo.ChangeVerticesIn().Add(GetSameDomainIndex(anIndexNew));
            }
            else
            {
                foreach (BOPDS_PaveBlock aPaveBlock in PaveBlocks(anEdgeFaceInterference.Index1()))
                {
                    BOPDS_CommonBlock? aCommonBlock = CommonBlock(aPaveBlock);
                    if (aCommonBlock is not null && aCommonBlock.Contains(aFaceIndex))
                    {
                        aFaceInfo.ChangePaveBlocksIn().Add(aCommonBlock.PaveBlock1());
                    }
                }
            }
        }
    }

    /// <summary>Recomputes the state On of every face of the map.</summary>
    internal void UpdateFaceInfoOn(NCollection_Map<int> theFaces)
    {
        foreach (int aShapeIndex in theFaces)
        {
            BOPDS_ShapeInfo aShapeInfo = ChangeShapeInfo(aShapeIndex);
            if (!aShapeInfo.HasReference())
            {
                myFaceInfoPool.Append(new BOPDS_FaceInfo()).SetIndex(aShapeIndex);
                aShapeInfo.SetReference(myFaceInfoPool.Length() - 1);
            }
            BOPDS_FaceInfo aFaceInfo = myFaceInfoPool.Value(aShapeInfo.Reference());
            aFaceInfo.ChangePaveBlocksOn().Clear();
            aFaceInfo.ChangeVerticesOn().Clear();

            FaceInfoOn(aShapeIndex, aFaceInfo.ChangePaveBlocksOn(), aFaceInfo.ChangeVerticesOn());
        }
    }

    /// <summary>Recomputes the state On of every face, keeping only the pave blocks that have edges.</summary>
    internal void RefineFaceInfoOn()
    {
        foreach (BOPDS_FaceInfo aFaceInfo in myFaceInfoPool)
        {
            UpdateFaceInfoOn(aFaceInfo.Index());
            NCollection_IndexedMap<BOPDS_PaveBlock> aPaveBlocksOn = aFaceInfo.ChangePaveBlocksOn();
            for (int aPaveBlockIndex = aPaveBlocksOn.Extent(); aPaveBlockIndex >= 1; --aPaveBlockIndex)
            {
                BOPDS_PaveBlock aPaveBlock = aPaveBlocksOn.FindKey(aPaveBlockIndex);
                if (!aPaveBlock.HasEdge())
                {
                    aPaveBlocksOn.RemoveFromIndex(aPaveBlockIndex);
                }
            }
        }
    }

    /// <summary>Removes from the state In of every face the pave blocks that are also On.</summary>
    internal void RefineFaceInfoIn()
    {
        for (int i = 0; i < myNbSourceShapes; ++i)
        {
            BOPDS_ShapeInfo aShapeInfo = ShapeInfo(i);
            if (aShapeInfo.ShapeType() != TopAbs_FACE || !aShapeInfo.HasReference())
            {
                continue;
            }

            BOPDS_FaceInfo aFaceInfo = ChangeFaceInfo(i);
            NCollection_IndexedMap<BOPDS_PaveBlock> aPaveBlocksOn = aFaceInfo.PaveBlocksOn();
            NCollection_IndexedMap<BOPDS_PaveBlock> aPaveBlocksIn = aFaceInfo.ChangePaveBlocksIn();
            if (aPaveBlocksIn.IsEmpty() || aPaveBlocksOn.IsEmpty())
            {
                continue;
            }

            for (int aPaveBlockIndex = aPaveBlocksIn.Extent(); aPaveBlockIndex >= 1; --aPaveBlockIndex)
            {
                BOPDS_PaveBlock aPaveBlock = aPaveBlocksIn.FindKey(aPaveBlockIndex);
                if (aPaveBlocksOn.Contains(aPaveBlock))
                {
                    aPaveBlocksIn.RemoveFromIndex(aPaveBlockIndex);
                }
            }
        }
    }

    /// <summary>
    /// Appends to the list the vertices In and Sc of the face that are not ends of its In and Sc pave blocks
    /// (in the vertex maps' order: decision 6153).
    /// </summary>
    public void AloneVertices(int theFaceIndex, NCollection_List<int> theVertexList)
    {
        if (!HasFaceInfo(theFaceIndex))
        {
            return;
        }

        NCollection_Map<int> aMI = new();
        BOPDS_FaceInfo aFaceInfo = FaceInfo(theFaceIndex);

        foreach (NCollection_IndexedMap<BOPDS_PaveBlock> aPaveBlocks in new[] { aFaceInfo.PaveBlocksIn(), aFaceInfo.PaveBlocksSc() })
        {
            for (int aPaveBlockIndex = 1; aPaveBlockIndex <= aPaveBlocks.Extent(); ++aPaveBlockIndex)
            {
                BOPDS_PaveBlock aPaveBlock = aPaveBlocks.FindKey(aPaveBlockIndex);
                aPaveBlock.Indices(out int nV1, out int nV2);
                aMI.Add(nV1);
                aMI.Add(nV2);
            }
        }

        foreach (NCollection_Map<int> aVertexMap in new[] { aFaceInfo.VerticesIn(), aFaceInfo.VerticesSc() })
        {
            foreach (int aVertexIndex in aVertexMap)
            {
                if (aVertexIndex >= 0 && aMI.Add(aVertexIndex))
                {
                    theVertexList.Append(aVertexIndex);
                }
            }
        }
    }

    /// <summary>
    /// The On and In sub-shapes of two faces: <paramref name="theMVOnIn"/> the On/In vertices of both,
    /// <paramref name="theMVCommon"/> the vertices common to both, <paramref name="thePBOnIn"/> all their On/In pave
    /// blocks, <paramref name="theCommonPaveBlocks"/> the pave blocks shared by both.
    /// </summary>
    public void SubShapesOnIn(int theFaceIndex1,
                              int theFaceIndex2,
                              NCollection_Map<int> theMVOnIn,
                              NCollection_Map<int> theMVCommon,
                              NCollection_IndexedMap<BOPDS_PaveBlock> thePBOnIn,
                              NCollection_Map<BOPDS_PaveBlock> theCommonPaveBlocks)
    {
        BOPDS_FaceInfo aFaceInfo1 = FaceInfo(theFaceIndex1);
        BOPDS_FaceInfo aFaceInfo2 = FaceInfo(theFaceIndex2);

        NCollection_IndexedMap<BOPDS_PaveBlock> aPBOn1 = aFaceInfo1.PaveBlocksOn();
        NCollection_IndexedMap<BOPDS_PaveBlock> aPBIn1 = aFaceInfo1.PaveBlocksIn();
        NCollection_IndexedMap<BOPDS_PaveBlock> aPBOn2 = aFaceInfo2.PaveBlocksOn();
        NCollection_IndexedMap<BOPDS_PaveBlock> aPBIn2 = aFaceInfo2.PaveBlocksIn();

        // Helper to process pave blocks from a map
        void processMap(NCollection_IndexedMap<BOPDS_PaveBlock> theMap)
        {
            for (int anIdx = 1; anIdx <= theMap.Extent(); ++anIdx)
            {
                BOPDS_PaveBlock aPaveBlock = theMap.FindKey(anIdx);
                thePBOnIn.Add(aPaveBlock);
                aPaveBlock.Indices(out int aV1, out int aV2);
                theMVOnIn.Add(aV1);
                theMVOnIn.Add(aV2);
            }
        }

        // Process all four maps
        processMap(aPBOn1);
        processMap(aPBIn1);
        processMap(aPBOn2);
        processMap(aPBIn2);

        // Find common pave blocks (those in Face1 that are also in Face2)
        void findCommon(NCollection_IndexedMap<BOPDS_PaveBlock> theMap)
        {
            for (int anIdx = 1; anIdx <= theMap.Extent(); ++anIdx)
            {
                BOPDS_PaveBlock aPaveBlock = theMap.FindKey(anIdx);
                if (aPBOn2.Contains(aPaveBlock) || aPBIn2.Contains(aPaveBlock))
                {
                    theCommonPaveBlocks.Add(aPaveBlock);
                    aPaveBlock.Indices(out int aV1, out int aV2);
                    theMVCommon.Add(aV1);
                    theMVCommon.Add(aV2);
                }
            }
        }

        findCommon(aPBOn1);
        findCommon(aPBIn1);

        NCollection_Map<int> aMVOn1 = aFaceInfo1.VerticesOn();
        NCollection_Map<int> aMVIn1 = aFaceInfo1.VerticesIn();
        NCollection_Map<int> aMVOn2 = aFaceInfo2.VerticesOn();
        NCollection_Map<int> aMVIn2 = aFaceInfo2.VerticesIn();

        foreach (NCollection_Map<int> aMV1 in new[] { aMVOn1, aMVIn1 })
        {
            foreach (int aVertexIndex in aMV1)
            {
                if (aMVOn2.Contains(aVertexIndex) || aMVIn2.Contains(aVertexIndex))
                {
                    theMVOnIn.Add(aVertexIndex);

                    // Vertex taken from the 1st face is in the 2nd one.
                    theMVCommon.Add(aVertexIndex);
                }
            }
        }
    }

    /// <summary>Appends to the list the edges shared by the two faces (the allocator of OCCT's signature goes).</summary>
    public void SharedEdges(int theFaceIndex1, int theFaceIndex2, NCollection_List<int> theEdgeList)
    {
        NCollection_Map<int> aFirstFaceEdges = new();

        // Collect edges of the first face.
        foreach (int aSubShapeIndex in ShapeInfo(theFaceIndex1).SubShapes())
        {
            BOPDS_ShapeInfo aSubShapeInfo = ChangeShapeInfo(aSubShapeIndex);
            if (aSubShapeInfo.ShapeType() != TopAbs_EDGE)
            {
                continue;
            }

            NCollection_List<BOPDS_PaveBlock> aSubShapePaveBlocks = PaveBlocks(aSubShapeIndex);
            if (aSubShapePaveBlocks.IsEmpty())
            {
                aFirstFaceEdges.Add(aSubShapeIndex);
            }
            else
            {
                foreach (BOPDS_PaveBlock aPaveBlock in aSubShapePaveBlocks)
                {
                    aFirstFaceEdges.Add(RealPaveBlock(aPaveBlock).Edge());
                }
            }
        }

        // Add edges of the second face if they are contained in the first one.
        foreach (int aSubShapeIndex in ShapeInfo(theFaceIndex2).SubShapes())
        {
            BOPDS_ShapeInfo aSubShapeInfo = ChangeShapeInfo(aSubShapeIndex);
            if (aSubShapeInfo.ShapeType() != TopAbs_EDGE)
            {
                continue;
            }

            NCollection_List<BOPDS_PaveBlock> aSubShapePaveBlocks = PaveBlocks(aSubShapeIndex);
            if (aSubShapePaveBlocks.IsEmpty())
            {
                if (aFirstFaceEdges.Contains(aSubShapeIndex))
                {
                    theEdgeList.Append(aSubShapeIndex);
                }
            }
            else
            {
                foreach (BOPDS_PaveBlock aPaveBlock in aSubShapePaveBlocks)
                {
                    int anEdgeIndex = RealPaveBlock(aPaveBlock).Edge();
                    if (aFirstFaceEdges.Contains(anEdgeIndex))
                    {
                        theEdgeList.Append(anEdgeIndex);
                    }
                }
            }
        }
    }

    /// <summary>The same-domain shapes (the data structure's own map).</summary>
    internal NCollection_DataMap<int, int> ShapesSD() => myShapesSD;

    /// <summary>Records that the shape <paramref name="theIndex"/> has the same-domain shape <paramref name="theIndexSD"/>.</summary>
    internal void AddShapeSD(int theIndex, int theIndexSD)
    {
        if (theIndex != theIndexSD)
        {
            myShapesSD.Bind(theIndex, theIndexSD);
        }
    }

    /// <summary>
    /// True if the shape has a same-domain shape; the last of the chain then (otherwise
    /// <paramref name="theIndexSD"/> is left as it was).
    /// </summary>
    public bool HasShapeSD(int theIndex, ref int theIndexSD)
    {
        bool bHasSD = false;
        int aKey = theIndex;
        while (myShapesSD.Find(aKey, out int aSD))
        {
            theIndexSD = aSD;
            bHasSD = true;
            aKey = theIndexSD;
        }
        return bHasSD;
    }

    /// <summary>The index of the shape's same-domain shape (the end of the chain), or the index itself.</summary>
    public int GetSameDomainIndex(int theIndex)
    {
        int aResult = theIndex;
        while (myShapesSD.Find(aResult, out int aSameDomainIndex))
        {
            aResult = aSameDomainIndex;
        }
        return aResult;
    }

    internal NCollection_DynamicArray<BOPDS_InterfVV> InterfVV() => myInterfVV;

    internal NCollection_DynamicArray<BOPDS_InterfVE> InterfVE() => myInterfVE;

    internal NCollection_DynamicArray<BOPDS_InterfVF> InterfVF() => myInterfVF;

    internal NCollection_DynamicArray<BOPDS_InterfEE> InterfEE() => myInterfEE;

    internal NCollection_DynamicArray<BOPDS_InterfEF> InterfEF() => myInterfEF;

    internal NCollection_DynamicArray<BOPDS_InterfFF> InterfFF() => myInterfFF;

    internal NCollection_DynamicArray<BOPDS_InterfVZ> InterfVZ() => myInterfVZ;

    internal NCollection_DynamicArray<BOPDS_InterfEZ> InterfEZ() => myInterfEZ;

    internal NCollection_DynamicArray<BOPDS_InterfFZ> InterfFZ() => myInterfFZ;

    internal NCollection_DynamicArray<BOPDS_InterfZZ> InterfZZ() => myInterfZZ;

    /// <summary>The number of interference types (10).</summary>
    public static int NbInterfTypes() => 10;

    /// <summary>Adds the interference between the two shapes to the table; false if it was there.</summary>
    internal bool AddInterf(int theI1, int theI2)
    {
        if (myInterfTB.Add(new BOPDS_Pair(theI1, theI2)))
        {
            myInterfered.Add(theI1);
            myInterfered.Add(theI2);
            return true;
        }
        return false;
    }

    /// <summary>True if the shape interferes with some shape.</summary>
    public bool HasInterf(int theI) => myInterfered.Contains(theI);

    /// <summary>True if the two shapes interfere.</summary>
    public bool HasInterf(int theI1, int theI2)
    {
        BOPDS_Pair aPK = new(theI1, theI2);
        return myInterfTB.Contains(aPK);
    }

    /// <summary>The table of interferences (the data structure's own map; read only for callers).</summary>
    public NCollection_Map<BOPDS_Pair> Interferences() => myInterfTB;

    /// <summary>Writes the ranges and the shapes as OCCT's <c>Dump</c> prints them.</summary>
    public void Dump(TextWriter theOS)
    {
        theOS.Write(" *** DS ***\n");
        int aRangesCount = NbRanges();
        theOS.Write(" Ranges:" + aRangesCount + "\n");
        for (int i = 0; i < aRangesCount; ++i)
        {
            BOPDS_IndexRange aRange = Range(i);
            aRange.Dump(theOS);
            theOS.Write("\n");
        }
        //
        int aSourceShapesCount = NbSourceShapes();
        theOS.Write(" Shapes:" + aSourceShapesCount + "\n");
        int aShapesCount = NbShapes();
        for (int i = 0; i < aShapesCount; ++i)
        {
            BOPDS_ShapeInfo aShapeInfo = ShapeInfo(i);
            theOS.Write(" " + i + " :");
            aShapeInfo.Dump(theOS);
            theOS.Write("\n");
            if (i == aSourceShapesCount - 1)
            {
                theOS.Write(" ****** adds\n");
            }
        }
        theOS.Write(" ******\n");
    }

    private bool CheckCoincidence(BOPDS_PaveBlock thePaveBlock1, BOPDS_PaveBlock thePaveBlock2, double theFuzzyValue)
    {
        // Find an intermediate point of the first pave block.
        // Seems like it doesn't matter where exactly this point is located on the pave block.
        thePaveBlock1.Range(out double aPaveBlock1First, out double aPaveBlock1Last);
        double aPaveBlock1Intermediate = IntTools_Tools.IntermediatePoint(aPaveBlock1First, aPaveBlock1Last);
        TopoDS_Edge anEdge1 = TopoDS.Edge(Shape(thePaveBlock1.OriginalEdge()));
        BOPTools_AlgoTools.PointOnEdge(anEdge1, aPaveBlock1Intermediate, out gp_Pnt aPaveBlock1IntermediatePoint);

        // Obtain the curve of the second edge and project the point onto it.
        thePaveBlock2.Range(out double aPaveBlock2First, out double aPaveBlock2Last);
        TopoDS_Edge anEdge2 = TopoDS.Edge(Shape(thePaveBlock2.OriginalEdge()));
        Geom_Curve anEdgeCurve2 = BRep_Tool.Curve(anEdge2, out double anEdge2ParamFirst, out double anEdge2ParamLast)!;
        GeomAPI_ProjectPointOnCurve aProjector = new();
        aProjector.Init(anEdgeCurve2, anEdge2ParamFirst, anEdge2ParamLast);
        aProjector.Perform(aPaveBlock1IntermediatePoint);
        // Check the results of projection.
        if (aProjector.NbPoints() != 0)
        {
            double aDistance = aProjector.LowerDistance();
            double aTolerance = BRep_Tool.MaxTolerance(anEdge1, TopAbs_VERTEX)
                                + BRep_Tool.MaxTolerance(anEdge2, TopAbs_VERTEX)
                                + Max(theFuzzyValue, Precision.Confusion());
            // If the distance is less than tolerance, check that the projection
            // point lies within the parameter range of the second pave block.
            if (aDistance < aTolerance)
            {
                double aDistanceParam = aProjector.LowerDistanceParameter();
                if (aDistanceParam > aPaveBlock2First && aDistanceParam < aPaveBlock2Last)
                {
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>True if the shape <paramref name="theCandidate"/> is a sub-shape of <paramref name="theParent"/>.</summary>
    public bool IsSubShape(int theCandidate, int theParent)
    {
        foreach (int aSubShapeIndex in ShapeInfo(theParent).SubShapes())
        {
            if (aSubShapeIndex == theCandidate)
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
    public void Paves(int theEdge, NCollection_List<BOPDS_Pave> theResultPaves)
    {
        // Note: uniqueness is defined by operator == (index and parameter), while sorting is done
        // by operator <, which compares only parameters.

        NCollection_List<BOPDS_PaveBlock> aPaveBlocks = PaveBlocks(theEdge);
        if (aPaveBlocks.IsEmpty())
        {
            return;
        }

        BOPDS_Pave[] pPaves = new BOPDS_Pave[aPaveBlocks.Extent() + 1];
        Array.Fill(pPaves, new BOPDS_Pave());
        int i = 0;
        NCollection_Map<BOPDS_Pave> aVisitedPaves = new();
        foreach (BOPDS_PaveBlock aPaveBlock in aPaveBlocks)
        {
            foreach (BOPDS_Pave aPave in new[] { aPaveBlock.Pave1(), aPaveBlock.Pave2() })
            {
                if (aVisitedPaves.Add(aPave))
                {
                    pPaves[i] = aPave;
                    ++i;
                }
            }
        }

        std_sort.sort(pPaves.AsSpan(), new BOPDS_Pave.LessThan());
        foreach (BOPDS_Pave aPave in pPaves)
        {
            theResultPaves.Append(aPave);
        }
    }

    /// <summary>
    /// The bounding box of the solid with the index, from its faces' boxes; whole if a shell is open or (with
    /// <paramref name="theCheckInverted"/>) the solid is inverted.
    /// </summary>
    public void BuildBndBoxSolid(int theIndex, ref Bnd_Box theBox, bool theCheckInverted = true)
    {
        BOPDS_ShapeInfo aShapeInfo = ShapeInfo(theIndex);

        bool anIsOpenBox = false;
        foreach (int aSubShapeIndex in aShapeInfo.SubShapes())
        {
            BOPDS_ShapeInfo aShellInfo = ShapeInfo(aSubShapeIndex);
            if (aShellInfo.ShapeType() != TopAbs_SHELL)
            {
                continue;
            }

            foreach (int aFaceIndex in aShellInfo.SubShapes())
            {
                BOPDS_ShapeInfo aFaceInfo = ShapeInfo(aFaceIndex);
                if (aFaceInfo.ShapeType() != TopAbs_FACE)
                {
                    continue;
                }

                Bnd_Box aFaceBoundBox = aFaceInfo.Box();
                theBox.Add(aFaceBoundBox);

                anIsOpenBox = aFaceBoundBox.IsOpen();
                if (anIsOpenBox)
                {
                    break;
                }
            }

            if (!anIsOpenBox)
            {
                TopoDS_Shell aShell = TopoDS.Shell(aShellInfo.Shape());
                anIsOpenBox = BOPTools_AlgoTools.IsOpenShell(aShell);
            }

            if (anIsOpenBox)
            {
                break;
            }
        }

        if (anIsOpenBox)
        {
            theBox.SetWhole();
        }
        else if (theCheckInverted)
        {
            TopoDS_Solid aSolid = TopoDS.Solid(aShapeInfo.Shape());
            if (BOPTools_AlgoTools.IsInvertedSolid(aSolid))
            {
                theBox.SetWhole();
            }
        }
    }

    /// <summary>Replaces the vertices of every pave block by their same-domain vertices.</summary>
    internal void UpdatePaveBlocksWithSDVertices()
    {
        foreach (NCollection_List<BOPDS_PaveBlock> aPaveBlockList in myPaveBlocksPool)
        {
            foreach (BOPDS_PaveBlock aPaveBlock in aPaveBlockList)
            {
                UpdatePaveBlockWithSDVertices(aPaveBlock);
            }
        }
    }

    /// <summary>Replaces the vertices of the pave block by their same-domain vertices.</summary>
    internal void UpdatePaveBlockWithSDVertices(BOPDS_PaveBlock thePaveBlock)
    {
        BOPDS_Pave aPave1 = thePaveBlock.Pave1();
        int anIndex1 = GetSameDomainIndex(aPave1.Index());
        aPave1.SetIndex(anIndex1);
        thePaveBlock.SetPave1(aPave1);

        BOPDS_Pave aPave2 = thePaveBlock.Pave2();
        int anIndex2 = GetSameDomainIndex(aPave2.Index());
        aPave2.SetIndex(anIndex2);
        thePaveBlock.SetPave2(aPave2);
    }

    /// <summary>Replaces the vertices of the common block's pave blocks by their same-domain vertices.</summary>
    internal void UpdateCommonBlockWithSDVertices(BOPDS_CommonBlock theCommonBlock)
    {
        foreach (BOPDS_PaveBlock aPaveBlock in theCommonBlock.PaveBlocks())
        {
            UpdatePaveBlockWithSDVertices(aPaveBlock);
        }
    }

    /// <summary>Makes the pave blocks of the edges of the vertex.</summary>
    internal void InitPaveBlocksForVertex(int theVertexIndex)
    {
        if (!myMapVE.Find(theVertexIndex, out NCollection_List<int> anEdgeIndices))
        {
            return;
        }

        foreach (int anEdgeIndex in anEdgeIndices)
        {
            ChangePaveBlocks(anEdgeIndex);
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

        foreach (NCollection_List<BOPDS_PaveBlock> aPaveBlockList in ChangePaveBlocksPool())
        {
            if (aPaveBlockList.Extent() != 1)
            {
                continue;
            }

            BOPDS_PaveBlock aPaveBlock = aPaveBlockList.First();
            if (IsCommonBlock(aPaveBlock))
            {
                continue;
            }

            aPaveBlock.Indices(out int aShapeIndex1, out int aShapeIndex2);
            if (!IsNewShape(aShapeIndex1) && !IsNewShape(aShapeIndex2))
            {
                // Both vertices are original, thus the PB is untouched.
                // Remove reference for the original edge
                int anOrigEdgeIndex = aPaveBlock.OriginalEdge();
                if (anOrigEdgeIndex >= 0)
                {
                    ChangeShapeInfo(anOrigEdgeIndex).SetReference(-1);
                }
                // Clear contents of the list
                aPaveBlockList.Clear();
            }
        }
    }

    /// <summary>
    /// True if the pave block's shrunk data is still valid: the ends of the shrunk range stay outside the vertices'
    /// tolerances (they may not after the vertices were replaced or their tolerances grew).
    /// </summary>
    public bool IsValidShrunkData(BOPDS_PaveBlock thePaveBlock)
    {
        if (!thePaveBlock.HasShrunkData())
        {
            return false;
        }

        // Compare the distances from the bounds of the shrunk range to the vertices
        // with the tolerance values of vertices

        // Shrunk range
        Span<double> aParameter = stackalloc double[2];
        Span<int> aVertexIndex = stackalloc int[2];
        thePaveBlock.ShrunkData(out aParameter[0], out aParameter[1], out _, out _);
        thePaveBlock.Indices(out aVertexIndex[0], out aVertexIndex[1]);

        TopoDS_Edge anEdge = TopoDS.Edge(Shape(thePaveBlock.OriginalEdge()));
        BRepAdaptor_Curve aCurveAdaptor = new(anEdge);
        double anEpsilon = BRep_Tool.Tolerance(anEdge) * 0.01;

        for (int i = 0; i < 2; ++i)
        {
            TopoDS_Vertex aVertex = TopoDS.Vertex(Shape(aVertexIndex[i]));
            double aTol = BRep_Tool.Tolerance(aVertex) + Precision.Confusion();
            // Bounding point
            gp_Pnt aVertexPoint = BRep_Tool.Pnt(aVertex);

            // Point on the end of shrunk range
            gp_Pnt aPointOnEdge = aCurveAdaptor.Value(aParameter[i]);

            if (aTol - aVertexPoint.Distance(aPointOnEdge) > anEpsilon)
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
    private static double ComputeParameter(in TopoDS_Vertex theVertex, in TopoDS_Edge theEdge)
    {
        double anEdgeToleranceSqr = MathUtils.Sqr(BRep_Tool.Tolerance(theEdge));
        gp_Pnt aVertexPoint = BRep_Tool.Pnt(theVertex);

        TopoDS_Edge aForwardEdge = theEdge;
        aForwardEdge.Orientation(TopAbs_FORWARD);
        Geom_Curve aCurve = BRep_Tool.Curve(aForwardEdge, out double aCurveStart, out double aCurveEnd)!;

        bool isMatch(double theParameter)
        {
            aCurve.D0(theParameter, out gp_Pnt aPointOnCurve);
            return aPointOnCurve.SquareDistance(aVertexPoint) < anEdgeToleranceSqr;
        }

        // Note: only start and end parameters are checked.
        // It is unsafe to return 0. when point is not matching either end
        // but this is left to preserve previous behavior.
        return isMatch(aCurveStart) ? aCurveStart : isMatch(aCurveEnd) ? aCurveEnd : 0.0;
    }

    private int prepareVertices(double theAdditionalTolerance)
    {
        int aVertexCount = 0;

        for (int aVertexIndex = 0; aVertexIndex < myNbSourceShapes; ++aVertexIndex)
        {
            BOPDS_ShapeInfo aVertexInfo = ChangeShapeInfo(aVertexIndex);
            if (aVertexInfo.ShapeType() != TopAbs_VERTEX)
            {
                continue;
            }
            ++aVertexCount;

            TopoDS_Vertex aVertex = TopoDS.Vertex(aVertexInfo.Shape());

            ref Bnd_Box aVertexBoundBox = ref aVertexInfo.ChangeBox();
            aVertexBoundBox.SetGap(BRep_Tool.Tolerance(aVertex) + theAdditionalTolerance);
            aVertexBoundBox.Add(BRep_Tool.Pnt(aVertex));
        }

        return aVertexCount;
    }

    private int prepareEdges(double theAdditionalTolerance)
    {
        int anEdgeCount = 0;

        for (int anEdgeIndex = 0; anEdgeIndex < myNbSourceShapes; ++anEdgeIndex)
        {
            BOPDS_ShapeInfo anEdgeInfo = ChangeShapeInfo(anEdgeIndex);
            if (anEdgeInfo.ShapeType() != TopAbs_EDGE)
            {
                continue;
            }
            ++anEdgeCount;

            TopoDS_Edge anEdge = TopoDS.Edge(anEdgeInfo.Shape());
            double anEdgeTolerance = BRep_Tool.Tolerance(anEdge);

            if (!BRep_Tool.Degenerated(anEdge))
            {
                // This is processing of special case when edge has infinite curve.
                // In this case vertices are created at the infinite positions
                // and added to the edge sub-shape list.

                NCollection_List<int> aVertexIndices = anEdgeInfo.ChangeSubShapes();

                TopoDS_Edge aForwardEdge = anEdge;
                aForwardEdge.Orientation(TopAbs_FORWARD);
                Geom_Curve? aForwardEdgeCurve = BRep_Tool.Curve(aForwardEdge, out double aCurveStart, out double aCurveEnd);

                // Process infinite values of the edge curve parameters.
                // Creates vertices at the infinite positions and add them to the edge sub-shape list.
                void processInfinite(double theParam)
                {
                    aForwardEdgeCurve!.D0(theParam, out gp_Pnt aPoint);
                    TopoDS_Vertex aVertex = new();
                    BRep_Builder aBuilder = new();
                    aBuilder.MakeVertex(ref aVertex, aPoint, anEdgeTolerance);
                    aVertex.Orientation(TopAbs_FORWARD);

                    BOPDS_ShapeInfo aVertexInfo = new();
                    aVertexInfo.SetShape(aVertex);
                    aVertexInfo.SetShapeType(TopAbs_VERTEX);
                    aVertexInfo.SetFlag(1); // infinite flag

                    aVertexIndices.Append(Append(aVertexInfo));
                }

                if (Precision.IsNegativeInfinite(aCurveStart))
                {
                    processInfinite(aCurveStart);
                }
                if (Precision.IsPositiveInfinite(aCurveEnd))
                {
                    processInfinite(aCurveEnd);
                }
            }
            else
            {
                anEdgeInfo.SetFlag(anEdgeIndex);
            }

            // Update edge bounding box with its own bounding box.
            ref Bnd_Box anEdgeBoundBox = ref anEdgeInfo.ChangeBox();
            BRepBndLib.Add(anEdge, ref anEdgeBoundBox);

            // Add bounding boxes of vertices to the edge bounding box.
            foreach (int aVertexIndex in anEdgeInfo.SubShapes())
            {
                BOPDS_ShapeInfo aVertexInfo = ChangeShapeInfo(aVertexIndex);
                anEdgeBoundBox.Add(aVertexInfo.Box());
            }

            anEdgeBoundBox.SetGap(anEdgeBoundBox.GetGap() + theAdditionalTolerance);
        }

        return anEdgeCount;
    }

    private int prepareFaces(double theAdditionalTolerance)
    {
        int aFaceCount = 0;

        for (int aFaceIndex = 0; aFaceIndex < myNbSourceShapes; ++aFaceIndex)
        {
            // Iterate over all faces.
            BOPDS_ShapeInfo aFaceInfo = ChangeShapeInfo(aFaceIndex);
            if (aFaceInfo.ShapeType() != TopAbs_FACE)
            {
                continue;
            }
            ++aFaceCount;

            // Map of sub-shape indices for the face that will replace the current list of wire indices.
            NCollection_Map<int> aNewSubShapeIndices = new();

            TopoDS_Shape aFace = aFaceInfo.Shape();

            // Update face bounding box with its own bounding box.
            ref Bnd_Box aFaceBoundBox = ref aFaceInfo.ChangeBox();
            BRepBndLib.Add(aFace, ref aFaceBoundBox);

            // Container of the face sub-shape indices. Currently contains wire indices.
            // Will be updated to contain edge and vertex indices instead.
            NCollection_List<int> aFaceSubShapes = aFaceInfo.ChangeSubShapes();

            // Iterate over edges and vertices of the face and fill the new map of sub-shape indices.
            // Also marks degenerated edges on the face.
            foreach (int aWireIndex in aFaceSubShapes)
            {
                foreach (int anEdgeIndex in ChangeShapeInfo(aWireIndex).SubShapes())
                {
                    // Update face bounding box with edge bounding boxes.
                    BOPDS_ShapeInfo anEdgeInfo = ChangeShapeInfo(anEdgeIndex);
                    aFaceBoundBox.Add(anEdgeInfo.Box());

                    // Add edge index to the map of indices.
                    aNewSubShapeIndices.Add(anEdgeIndex);

                    // Mark degenerated edges on the face (OCCT sets the flag of a degenerated edge to the face's
                    // index).
                    TopoDS_Edge anEdge = TopoDS.Edge(anEdgeInfo.Shape());
                    if (BRep_Tool.Degenerated(anEdge))
                    {
                        anEdgeInfo.SetFlag(aFaceIndex);
                    }

                    // Add vertices of the edge to the map of indices.
                    foreach (int aVertexIndex in anEdgeInfo.SubShapes())
                    {
                        aNewSubShapeIndices.Add(aVertexIndex);
                    }
                }
            }

            // Add vertices of the face to the map of indices.
            for (TopoDS_Iterator aFaceIter = new(aFace); aFaceIter.More(); aFaceIter.Next())
            {
                TopoDS_Shape aSubShape = aFaceIter.Value();
                if (aSubShape.ShapeType() == TopAbs_VERTEX)
                {
                    aNewSubShapeIndices.Add(Index(aSubShape));
                }
            }

            // Remove wire indices and fill in edge and vertex indices instead.
            // The port's map iterates in insertion order, OCCT's in the order of its hash buckets (decision 6153).
            aFaceSubShapes.Clear();
            foreach (int aSubShapeIndex in aNewSubShapeIndices)
            {
                aFaceSubShapes.Append(aSubShapeIndex);
            }

            aFaceBoundBox.SetGap(aFaceBoundBox.GetGap() + theAdditionalTolerance);
        }

        return aFaceCount;
    }

    private int prepareSolids()
    {
        int aSolidCount = 0;

        // For the check mode we need to compute the bounding box for solid.
        // Otherwise, it will be computed on the building stage
        if (myArguments.Extent() != 1)
        {
            return 0;
        }

        for (int aSolidIndex = 0; aSolidIndex < myNbSourceShapes; ++aSolidIndex)
        {
            BOPDS_ShapeInfo aSolidInfo = ChangeShapeInfo(aSolidIndex);
            if (aSolidInfo.ShapeType() != TopAbs_SOLID)
            {
                continue;
            }
            ++aSolidCount;

            // Map of sub-shape indices for the solid that will replace the current list of shell indices.
            NCollection_Map<int> aNewSubShapeIndices = new();

            // Update solid bounding box with its own bounding box.
            ref Bnd_Box aSolidBoundBox = ref aSolidInfo.ChangeBox();
            BuildBndBoxSolid(aSolidIndex, ref aSolidBoundBox);

            // Container of the solid sub-shape indices. Currently contains shell indices.
            // Will be updated to contain face and edge indices instead.
            NCollection_List<int> aSolidSubShapes = aSolidInfo.ChangeSubShapes();

            // Iterate over faces and edges of the solid and fill the new map of sub-shape indices.
            foreach (int aShellIndex in aSolidSubShapes)
            {
                BOPDS_ShapeInfo aShellInfo = ChangeShapeInfo(aShellIndex);
                if (aShellInfo.ShapeType() != TopAbs_SHELL)
                {
                    continue;
                }

                foreach (int aFaceIndex in aShellInfo.SubShapes())
                {
                    BOPDS_ShapeInfo aFaceInfo = ChangeShapeInfo(aFaceIndex);
                    if (aFaceInfo.ShapeType() != TopAbs_FACE)
                    {
                        continue;
                    }
                    // Add face index to the map of indices.
                    aNewSubShapeIndices.Add(aFaceIndex);

                    foreach (int anEdgeIndex in aFaceInfo.SubShapes())
                    {
                        // Add edge index to the map of indices.
                        aNewSubShapeIndices.Add(anEdgeIndex);
                    }
                }
            }

            // Remove shell indices and fill in face and edge indices instead (insertion order: decision 6153).
            aSolidSubShapes.Clear();
            foreach (int aSubShapeIndex in aNewSubShapeIndices)
            {
                aSolidSubShapes.Append(aSubShapeIndex);
            }
        }

        return aSolidCount;
    }

    private void buildVertexEdgeMap()
    {
        for (int aShapeIndex = 0; aShapeIndex < myNbSourceShapes; ++aShapeIndex)
        {
            BOPDS_ShapeInfo anEdgeInfo = ChangeShapeInfo(aShapeIndex);
            if (anEdgeInfo.ShapeType() != TopAbs_EDGE)
            {
                continue;
            }

            foreach (int aSubShapeIndex in anEdgeInfo.SubShapes())
            {
                if (!myMapVE.Find(aSubShapeIndex, out NCollection_List<int> aVertexIndices))
                {
                    aVertexIndices = new NCollection_List<int>();
                    myMapVE.Bind(aSubShapeIndex, aVertexIndices);
                    aVertexIndices.Append(aShapeIndex);
                }
                else
                {
                    // provide uniqueness of the edges in the list
                    if (!aVertexIndices.Contains(aShapeIndex))
                    {
                        aVertexIndices.Append(aShapeIndex);
                    }
                }
            }
        }
    }
}
