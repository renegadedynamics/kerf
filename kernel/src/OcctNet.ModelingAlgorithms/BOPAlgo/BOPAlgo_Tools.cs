// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Tools.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Tools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Tools used in the intersection part of the Boolean operations.</summary>
/// <remarks>
/// Unit 6E ports the members the pave filler calls (MakeBlocks, FillMap, PerformCommonBlocks, ComputeToleranceOfCB,
/// IntersectVertices, TrsfToPoint); unit 6F adds the rest of the class (EdgesToWires, WiresToFaces, ClassifyFaces,
/// FillInternals, ...) to this partial class. OCCT's templates <c>MakeBlocks</c> and <c>FillMap</c> are generic over
/// the key; the allocator arguments go.
/// </remarks>
public static partial class BOPAlgo_Tools
{
    /// <summary>Makes the chains of the connected elements from the given connexity map.</summary>
    /// <remarks>OCCT's fence map takes the map's hasher from its type: pass it as <paramref name="theHasher"/> (null: the default).</remarks>
    public static void MakeBlocks<TKey>(NCollection_IndexedDataMap<TKey, NCollection_List<TKey>> theMILI,
                                        NCollection_List<NCollection_List<TKey>> theMBlocks,
                                        IEqualityComparer<TKey>? theHasher = null)
        where TKey : notnull
    {
        NCollection_Map<TKey> aMFence = new(theHasher);
        int i, aNb = theMILI.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TKey n = theMILI.FindKey(i);
            if (!aMFence.Add(n))
                continue;
            //
            // Start the chain
            NCollection_List<TKey> aChain = theMBlocks.Append(new NCollection_List<TKey>());
            aChain.Append(n);
            // Look for connected elements
            NCollection_List<TKey>.Iterator aItLChain = new(aChain);
            for (; aItLChain.More(); aItLChain.Next())
            {
                TKey n1 = aItLChain.Value();
                NCollection_List<TKey> aLI = theMILI.FindFromKey(n1);
                // Add connected elements into the chain
                NCollection_List<TKey>.Iterator aItLI = new(aLI);
                for (; aItLI.More(); aItLI.Next())
                {
                    TKey n2 = aItLI.Value();
                    if (aMFence.Add(n2))
                    {
                        aChain.Append(n2);
                    }
                }
            }
        }
    }

    /// <summary>Fills the map with the connected entities.</summary>
    public static void FillMap<TKey>(TKey n1, TKey n2, NCollection_IndexedDataMap<TKey, NCollection_List<TKey>> theMILI)
        where TKey : notnull
    {
        int anIndex1 = theMILI.FindIndex(n1);
        NCollection_List<TKey> pList1 = anIndex1 > 0 ? theMILI[anIndex1] : theMILI[theMILI.Add(n1, new NCollection_List<TKey>())];
        pList1.Append(n2);
        //
        int anIndex2 = theMILI.FindIndex(n2);
        NCollection_List<TKey> pList2 = anIndex2 > 0 ? theMILI[anIndex2] : theMILI[theMILI.Add(n2, new NCollection_List<TKey>())];
        pList2.Append(n1);
    }

    /// <summary>Adds the face to the pave block's list of faces.</summary>
    public static void FillMap(BOPDS_PaveBlock aPB, int nF, NCollection_IndexedDataMap<BOPDS_PaveBlock, NCollection_List<int>> aMPBLI)
    {
        int anIndex = aMPBLI.FindIndex(aPB);
        NCollection_List<int> pLI = anIndex > 0 ? aMPBLI[anIndex] : aMPBLI[aMPBLI.Add(aPB, new NCollection_List<int>())];
        pLI.Append(nF);
    }

    /// <summary>Creates the common blocks from the groups of connected pave blocks of <paramref name="aMPBLPB"/>.</summary>
    public static void PerformCommonBlocks(NCollection_IndexedDataMap<BOPDS_PaveBlock, NCollection_List<BOPDS_PaveBlock>> aMPBLPB,
                                           BOPDS_DS pDS, IntTools_Context? theContext = null)
    {
        int aNbCB;
        //
        aNbCB = aMPBLPB.Extent();
        if (aNbCB == 0)
        {
            return;
        }
        // Make Blocks of the pave blocks
        NCollection_List<NCollection_List<BOPDS_PaveBlock>> aMBlocks = new();
        MakeBlocks(aMPBLPB, aMBlocks);

        // Use temporary allocator for the local fence map
        NCollection_Map<int> aMFaces = new();

        NCollection_List<NCollection_List<BOPDS_PaveBlock>>.Iterator aItB = new(aMBlocks);
        for (; aItB.More(); aItB.Next())
        {
            NCollection_List<BOPDS_PaveBlock> aLPB = aItB.Value();
            int aNbPB = aLPB.Extent();
            if (aNbPB < 2)
            {
                continue;
            }

            aMFaces.Clear();
            // New common block
            BOPDS_CommonBlock? aCB = null;
            // Faces of the common block
            NCollection_List<int> aLFaces = new();

            NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPB);
            for (; aItLPB.More(); aItLPB.Next())
            {
                BOPDS_PaveBlock aPB = aItLPB.Value();
                if (pDS.IsCommonBlock(aPB))
                {
                    BOPDS_CommonBlock aCBx = pDS.CommonBlock(aPB)!;
                    // Move all faces to the new common block
                    NCollection_List<int>.Iterator aItLF = new(aCBx.Faces());
                    for (; aItLF.More(); aItLF.Next())
                    {
                        int nF = aItLF.Value();
                        // Append to common list avoiding duplicates
                        if (aMFaces.Add(nF))
                        {
                            aLFaces.Append(nF);
                        }
                    }
                    aCB ??= aCBx;
                }
            }

            aCB ??= new BOPDS_CommonBlock();

            aCB.SetPaveBlocks(aLPB);
            aCB.SetFaces(aLFaces);
            for (aItLPB.Initialize(aLPB); aItLPB.More(); aItLPB.Next())
            {
                pDS.SetCommonBlock(aItLPB.Value(), aCB);
            }

            // Compute tolerance for Common Block
            double aTolCB = ComputeToleranceOfCB(aCB, pDS, theContext);
            aCB.SetTolerance(aTolCB);
        }
    }

    /// <summary>Creates the common blocks on faces from the pave block → faces map <paramref name="aMPBLI"/>.</summary>
    public static void PerformCommonBlocks(NCollection_IndexedDataMap<BOPDS_PaveBlock, NCollection_List<int>> aMPBLI,
                                           BOPDS_DS pDS, IntTools_Context? theContext = null)
    {
        int nF, i, aNb;
        NCollection_List<int>.Iterator aItLI = new();
        BOPDS_PaveBlock aPB;
        BOPDS_CommonBlock aCB;
        //
        aNb = aMPBLI.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            aPB = aMPBLI.FindKey(i);
            if (pDS.IsCommonBlock(aPB))
            {
                aCB = pDS.CommonBlock(aPB)!;
            }
            else
            {
                aCB = new BOPDS_CommonBlock();
                aCB.AddPaveBlock(aPB);
            }
            //
            NCollection_List<int> aLI = aMPBLI.FindFromKey(aPB);
            NCollection_List<int> aNewFaces = new();
            NCollection_List<int> anOldFaces = aCB.Faces();
            aItLI.Initialize(aLI);
            for (; aItLI.More(); aItLI.Next())
            {
                nF = aItLI.Value();
                // the both lists aLI and anOldFaces are expected to be short,
                // so we can allow to run nested loop here
                NCollection_List<int>.Iterator it = new(anOldFaces);
                for (; it.More(); it.Next())
                {
                    if (it.Value() == nF)
                    {
                        break;
                    }
                }
                if (!it.More())
                {
                    aNewFaces.Append(nF);
                }
            }
            aCB.AppendFaces(aNewFaces);
            pDS.SetCommonBlock(aPB, aCB);
            // Compute tolerance for Common Block
            double aTolCB = ComputeToleranceOfCB(aCB, pDS, theContext);
            aCB.SetTolerance(aTolCB);
        }
    }

    /// <summary>
    /// The tolerance of a common block: the largest distance from 11 points of its real pave block's edge to the other
    /// edges and the faces of the block, plus their tolerances (at least the edge's tolerance).
    /// </summary>
    public static double ComputeToleranceOfCB(BOPDS_CommonBlock? theCB, BOPDS_DS theDS, IntTools_Context? theContext)
    {
        double aTolMax = 0.0;
        if (theCB is null)
        {
            return aTolMax;
        }
        //
        BOPDS_PaveBlock aPBR = theCB.PaveBlock1();
        int nE = aPBR.OriginalEdge();
        TopoDS_Edge aEOr = TopoDS.Edge(theDS.Shape(nE));
        aTolMax = BRep_Tool.Tolerance(aEOr);
        //
        NCollection_List<BOPDS_PaveBlock> aLPB = theCB.PaveBlocks();
        NCollection_List<int> aLFI = theCB.Faces();
        //
        if ((aLPB.Extent() < 2) && aLFI.IsEmpty())
        {
            return aTolMax;
        }
        //
        const int aNbPnt = 11;
        double aTol, aT, aT1, aT2, aDt;
        gp_Pnt aP;
        //
        Geom_Curve aC3D = BRep_Tool.Curve(aEOr, out aT1, out aT2)!;
        //
        aPBR.Range(out aT1, out aT2);
        aDt = (aT2 - aT1) / (aNbPnt + 1);
        //
        IntTools_Context aCtx = theContext ?? new IntTools_Context();

        // compute max tolerance for common blocks on edges
        if (aLPB.Extent() > 1)
        {
            // compute max distance between edges
            NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new();
            //
            aItPB.Initialize(aLPB);
            for (; aItPB.More(); aItPB.Next())
            {
                BOPDS_PaveBlock aPB = aItPB.Value();
                if (aPB == aPBR)
                {
                    continue;
                }
                //
                nE = aPB.OriginalEdge();
                TopoDS_Edge aE = TopoDS.Edge(theDS.Shape(nE));
                aTol = BRep_Tool.Tolerance(aE);
                //
                // OCCT copies the context's projector (aProjPC = aCtx->ProjPC(aE)) and projects with the copy: the
                // context's projector, used here directly, is left with the last projection, which nothing reads.
                GeomAPI_ProjectPointOnCurve aProjPC = aCtx.ProjPC(aE);
                //
                aT = aT1;
                for (int i = 1; i <= aNbPnt; i++)
                {
                    aT += aDt;
                    aC3D.D0(aT, out aP);
                    aProjPC.Perform(aP);
                    if (aProjPC.NbPoints() != 0)
                    {
                        double aTolNew = aTol + aProjPC.LowerDistance();
                        if (aTolNew > aTolMax)
                        {
                            aTolMax = aTolNew;
                        }
                    }
                }
            }
        }
        //
        // compute max tolerance for common blocks on faces
        if (aLFI.Extent() != 0)
        {
            for (NCollection_List<int>.Iterator aItLI = new(aLFI); aItLI.More(); aItLI.Next())
            {
                int nF = aItLI.Value();
                TopoDS_Face aF = TopoDS.Face(theDS.Shape(nF));
                aTol = BRep_Tool.Tolerance(aF);
                //
                GeomAPI_ProjectPointOnSurf aProjPS = aCtx.ProjPS(aF);
                //
                aT = aT1;
                for (int i = 1; i <= aNbPnt; i++)
                {
                    aT += aDt;
                    aC3D.D0(aT, out aP);
                    aProjPS.Perform(aP);
                    if (aProjPS.NbPoints() != 0)
                    {
                        double aTolNew = aTol + aProjPS.LowerDistance();
                        if (aTolNew > aTolMax)
                        {
                            aTolMax = aTolNew;
                        }
                    }
                }
            }
        }
        //
        return aTolMax;
    }

    /// <summary>Selects the pairs of vertices whose tolerance spheres (with the fuzzy value) meet.</summary>
    private sealed class BOPAlgo_PairVerticesSelector : BOPTools_PairSelector<NCollection_Vec3<double>>
    {
        private NCollection_IndexedDataMap<TopoDS_Shape, double>? myVertices;
        private double myFuzzyValue;

        public BOPAlgo_PairVerticesSelector()
        {
            myVertices = null;
            myFuzzyValue = Precision.Confusion();
        }

        //! Sets the map of vertices with tolerances
        public void SetMapOfVerticesTolerances(NCollection_IndexedDataMap<TopoDS_Shape, double> theVertices) => myVertices = theVertices;

        //! Sets the fuzzy value
        public void SetFuzzyValue(double theFuzzyValue) => myFuzzyValue = theFuzzyValue;

        //! Checks and accepts the pair of elements.
        public override bool Accept(int theID1, int theID2)
        {
            if (!RejectElement(theID1, theID2))
            {
                int anID1 = myBVHSet1!.Element(theID1);
                TopoDS_Vertex aV1 = TopoDS.Vertex(myVertices!.FindKey(anID1));
                double aTolV1 = BRep_Tool.Tolerance(aV1);
                if (aTolV1 < myVertices.FindFromIndex(anID1))
                {
                    aTolV1 = myVertices.FindFromIndex(anID1);
                }
                gp_Pnt aP1 = BRep_Tool.Pnt(aV1);

                int anID2 = myBVHSet1.Element(theID2);
                TopoDS_Vertex aV2 = TopoDS.Vertex(myVertices.FindKey(anID2));
                double aTolV2 = BRep_Tool.Tolerance(aV2);
                if (aTolV2 < myVertices.FindFromIndex(anID2))
                {
                    aTolV2 = myVertices.FindFromIndex(anID2);
                }
                gp_Pnt aP2 = BRep_Tool.Pnt(aV2);

                double aTolSum2 = aTolV1 + aTolV2 + myFuzzyValue;
                aTolSum2 *= aTolSum2;

                double aD2 = aP1.SquareDistance(aP2);
                if (aD2 < aTolSum2)
                {
                    myPairs.Append(new PairIDs(anID1, anID2));
                    return true;
                }
            }
            return false;
        }
    }

    /// <summary>
    /// Finds the chains of intersecting vertices: the vertices of <paramref name="theVertices"/> (with tolerances at
    /// least their values) connected by meeting tolerance spheres (with half the fuzzy value each).
    /// </summary>
    public static void IntersectVertices(NCollection_IndexedDataMap<TopoDS_Shape, double> theVertices,
                                         double theFuzzyValue,
                                         NCollection_List<NCollection_List<TopoDS_Shape>> theChains)
    {
        int aNbV = theVertices.Extent();
        if (aNbV <= 1)
        {
            if (aNbV == 1)
            {
                theChains.Append(new NCollection_List<TopoDS_Shape>()).Append(theVertices.FindKey(1));
            }
            return;
        }

        // Additional tolerance for intersection
        double aTolAdd = theFuzzyValue / 2.0;

        // Use BVH Tree for sorting the vertices
        BOPTools_BoxTree aBBTree = new();
        aBBTree.SetSize(aNbV);

        for (int i = 1; i <= aNbV; ++i)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(theVertices.FindKey(i));
            double aTol = BRep_Tool.Tolerance(aV);
            if (aTol < theVertices[i])
            {
                aTol = theVertices[i];
            }

            // Build bnd box for vertex
            Bnd_Box aBox = new();
            aBox.Add(BRep_Tool.Pnt(aV));
            aBox.SetGap(aTol + aTolAdd);
            aBBTree.Add(i, Bnd_Tools.Bnd2BVH(aBox));
        }

        aBBTree.Build();

        // Perform selection of the interfering vertices
        BOPAlgo_PairVerticesSelector aPairSelector = new();
        aPairSelector.SetBVHSets(aBBTree, aBBTree);
        aPairSelector.SetSame(true);
        aPairSelector.SetMapOfVerticesTolerances(theVertices);
        aPairSelector.SetFuzzyValue(theFuzzyValue);
        aPairSelector.Select();

        // Treat the selected pairs
        NCollection_LinearVector<BOPTools_PairSelector<NCollection_Vec3<double>>.PairIDs> aPairs = aPairSelector.Pairs();
        int aNbPairs = aPairs.Size();

        // Collect interfering pairs
        NCollection_IndexedDataMap<int, NCollection_List<int>> aMILI = new();

        for (int iPair = 0; iPair < aNbPairs; ++iPair)
        {
            BOPTools_PairSelector<NCollection_Vec3<double>>.PairIDs aPair = aPairs.Value(iPair);
            FillMap(aPair.ID1, aPair.ID2, aMILI);
        }

        NCollection_List<NCollection_List<int>> aBlocks = new();
        MakeBlocks(aMILI, aBlocks);

        NCollection_List<NCollection_List<int>>.Iterator itLI = new(aBlocks);
        for (; itLI.More(); itLI.Next())
        {
            NCollection_List<int> aLI = itLI.Value();
            NCollection_List<TopoDS_Shape> aChain = theChains.Append(new NCollection_List<TopoDS_Shape>());

            for (NCollection_List<int>.Iterator itI = new(aLI); itI.More(); itI.Next())
            {
                aChain.Append(theVertices.FindKey(itI.Value()));
            }
        }

        // Add not interfered vertices as a chain of 1 element
        for (int i = 1; i <= aNbV; ++i)
        {
            if (!aMILI.Contains(i))
            {
                theChains.Append(new NCollection_List<TopoDS_Shape>()).Append(theVertices.FindKey(i));
            }
        }
    }

    /// <summary>
    /// Computes the translation moving the two boxes' union to <paramref name="thePoint"/> (its minimal corner there),
    /// for more accurate computations. Returns true if the boxes are far from the point relative to their size
    /// (<paramref name="theCriteria"/>), false otherwise (the transformation is then left unchanged).
    /// </summary>
    public static bool TrsfToPoint(in Bnd_Box theBox1, in Bnd_Box theBox2, ref gp_Trsf theTrsf, in gp_Pnt thePoint,
                                   double theCriteria = 1.0e+5)
    {
        // Unify two boxes
        Bnd_Box aBox = theBox1;
        aBox.Add(theBox2);

        gp_XYZ aBCenter = (aBox.CornerMin().XYZ() + aBox.CornerMax().XYZ()) / 2.0;
        double aPBDist = (thePoint.XYZ() - aBCenter).Modulus();
        if (aPBDist < theCriteria)
        {
            return false;
        }

        double aBSize = Math.Sqrt(aBox.SquareExtent());
        if ((aBSize / aPBDist) > (1.0 / theCriteria))
        {
            return false;
        }

        theTrsf.SetTranslation(new gp_Vec(aBox.CornerMin(), thePoint));
        return true;
    }

    /// <summary><see cref="TrsfToPoint(in Bnd_Box, in Bnd_Box, ref gp_Trsf, in gp_Pnt, double)"/> to the origin.</summary>
    public static bool TrsfToPoint(in Bnd_Box theBox1, in Bnd_Box theBox2, ref gp_Trsf theTrsf) =>
        TrsfToPoint(theBox1, theBox2, ref theTrsf, new gp_Pnt(0.0, 0.0, 0.0));
}
