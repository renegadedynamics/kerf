// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Tools.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Tools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Tools used in the intersection part of the Boolean operations.</summary>
/// <remarks>
/// Unit 6E ported the members the pave filler calls, unit 6F the rest. OCCT's templates <c>MakeBlocks</c> and
/// <c>FillMap</c> are generic over the key; the allocator arguments go. <see cref="WiresToFaces"/> copies a frozen input at
/// its entry, since OCCT writes p-curves and tolerances into the input's edges.
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
        NCollection_IntMap aMFaces = new();

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

    /// <summary>
    /// Makes wires from the edges of <paramref name="theEdges"/>, grouping them by the planes they lie in. The edges are
    /// intersected first with the General Fuse unless <paramref name="theShared"/> says they already share their
    /// vertices. Returns 0 on success, 1 when there is no edge to process, 2 when the edges can't be intersected (then
    /// <paramref name="theWires"/> is left unchanged).
    /// </summary>
    public static int EdgesToWires(in TopoDS_Shape theEdges, ref TopoDS_Shape theWires, bool theShared = false, double theAngTol = 1.0e-8)
    {
        int iErr = 0;
        //
        // 1. Check the input edges
        //
        // List of edges to process
        NCollection_List<TopoDS_Shape> aLE = new();
        //
        TopExp_Explorer aExp = new(theEdges, TopAbs_EDGE);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
            if (!BRep_Tool.Degenerated(aE) && BRep_Tool.IsGeometric(aE))
            {
                aLE.Append(aExp.Current());
            }
        }
        //
        if (aLE.IsEmpty())
        {
            // no edges to process
            iErr = 1;
            return iErr;
        }
        //
        BRep_Builder aBB = new();
        TopoDS_Compound aRWires = new();
        aBB.MakeCompound(ref aRWires);
        //
        if (aLE.Extent() == 1)
        {
            TopoDS_Wire aWire = new();
            aBB.MakeWire(ref aWire);
            aBB.Add(aWire, aLE.First());
            aBB.Add(aRWires, aWire);
            theWires = aRWires;
            return iErr;
        }
        //
        // 2. Make compound of shared edges
        TopoDS_Shape aSEdges;
        //
        if (!theShared)
        {
            // intersect the edges if necessary
            BOPAlgo_Builder aGF = new();
            aGF.SetArguments(aLE);
            aGF.Perform();
            if (aGF.HasErrors())
            {
                // unable to share the edges
                iErr = 2;
                return iErr;
            }
            //
            aSEdges = aGF.InternalShape();
        }
        else
        {
            TopoDS_Compound aSEdgesC = new();
            aBB.MakeCompound(ref aSEdgesC);
            aSEdges = aSEdgesC;
            NCollection_List<TopoDS_Shape>.Iterator aItLE = new(aLE);
            for (; aItLE.More(); aItLE.Next())
            {
                aBB.Add(aSEdges, aItLE.Value());
            }
        }
        //
        // 3. Find edges located in the same planes and make wires from them.
        // If the plane cannot be found for a single edge, then it is necessary
        // to find all pairs of connected edges with the same cross product.

        // Try to compute the plane in which the edge is located
        NCollection_IndexedDataMap<TopoDS_Shape, gp_Pln> aDMEdgePln = new(TopTools_ShapeMapHasher.Instance);
        // Compute the tangent direction for the edges for which the plane is not defined
        NCollection_IndexedDataMap<TopoDS_Shape, gp_Dir> aDMEdgeTgt = new(TopTools_ShapeMapHasher.Instance);
        //
        // edges for which the plane is not found
        NCollection_Map<TopoDS_Shape> aMEdgesNoUniquePlane = new(TopTools_ShapeMapHasher.Instance);
        //
        // edges for which the plane cannot be found on a single edge
        TopoDS_Compound aLEdges = new();
        aBB.MakeCompound(ref aLEdges);
        //
        aExp.Init(aSEdges, TopAbs_EDGE);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
            BRepAdaptor_Curve aBAC = new(aE);
            //
            gp_Pln aPln = new();
            if (FindPlane(aBAC, ref aPln))
            {
                aDMEdgePln.Add(aE, aPln);
            }
            else
            {
                gp_Vec aVT = new();
                if (FindEdgeTangent(aBAC, ref aVT))
                {
                    aDMEdgeTgt.Add(aE, new gp_Dir(aVT));
                    aBB.Add(aLEdges, aE);
                    aMEdgesNoUniquePlane.Add(aE);
                }
            }
        }
        //
        // to avoid processing of the same edges in the same plane store
        // the processed planes into a list and use it as a fence map
        NCollection_List<gp_Dir> aLPFence = new();
        //
        // used edges
        NCollection_Map<TopoDS_Shape> aMEFence = new(TopTools_ShapeMapHasher.Instance);
        //
        // look for a planes on the single edges
        int i, j, aNbPlanes = aDMEdgePln.Extent(), aNbEdges = aDMEdgeTgt.Extent();
        for (i = 1; i <= aNbPlanes; ++i)
        {
            TopoDS_Shape aEI = aDMEdgePln.FindKey(i);
            if (!aMEFence.Add(aEI))
            {
                continue;
            }
            //
            gp_Pln aPlnI = aDMEdgePln[i];
            gp_Dir aDI = aPlnI.Position().Direction();
            //
            aLPFence.Append(aDI);
            //
            NCollection_IndexedMap<TopoDS_Shape> aMEPln = new(TopTools_ShapeMapHasher.Instance);
            aMEPln.Add(aEI);
            //
            NCollection_IndexedMap<TopoDS_Shape> aMV = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(aEI, TopAbs_VERTEX, aMV);
            //
            // look for other edges with the plane parallel to current one
            for (j = i + 1; j <= aNbPlanes; ++j)
            {
                gp_Dir aDJ = aDMEdgePln[j].Position().Direction();
                if (aDI.IsParallel(aDJ, theAngTol))
                {
                    TopoDS_Shape aEJ = aDMEdgePln.FindKey(j);
                    aMEPln.Add(aEJ);
                    aMEFence.Add(aEJ);
                    TopExp.MapShapes(aEJ, TopAbs_VERTEX, aMV);
                }
            }
            //
            // look for all other edges located in the plane parallel to current one
            TopoDS_Compound aCEPln = new();
            aBB.MakeCompound(ref aCEPln);
            //
            for (j = 1; j <= aNbEdges; ++j)
            {
                gp_Dir aDJ = aDMEdgeTgt[j];
                if (aDI.IsNormal(aDJ, theAngTol))
                {
                    aBB.Add(aCEPln, aDMEdgeTgt.FindKey(j));
                }
            }
            //
            // make blocks of these edges and check blocks to be connected
            // to any of the already added edges or forming a wire themselves
            NCollection_List<TopoDS_Shape> aLCBE = new();
            BOPTools_AlgoTools.MakeConnexityBlocks(aCEPln, TopAbs_VERTEX, TopAbs_EDGE, aLCBE);
            //
            // make wire from each block
            NCollection_List<TopoDS_Shape>.Iterator aItLCB = new(aLCBE);
            for (; aItLCB.More(); aItLCB.Next())
            {
                TopoDS_Shape aCBE = aItLCB.Value();
                //
                // check connectivity
                TopExp_Explorer aExpV = new(aCBE, TopAbs_VERTEX);
                for (; aExpV.More(); aExpV.Next())
                {
                    if (aMV.Contains(aExpV.Current()))
                    {
                        break;
                    }
                }
                //
                bool bAddBlock = aExpV.More();
                if (!bAddBlock)
                {
                    // check if the edges are forming a wire
                    gp_Pln aPln = new();
                    bAddBlock = FindPlane(aCBE, ref aPln, aDMEdgeTgt, aMEdgesNoUniquePlane);
                }
                //
                if (bAddBlock)
                {
                    // add edges
                    for (TopoDS_Iterator aItE = new(aCBE); aItE.More(); aItE.Next())
                    {
                        aMEPln.Add(aItE.Value());
                    }
                }
            }
            //
            MakeWires(aMEPln, aRWires, false, aDMEdgeTgt, aMEdgesNoUniquePlane);
        }
        //
        // make connection map from vertices to edges to find the connected pairs
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVE = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aLEdges, TopAbs_VERTEX, TopAbs_EDGE, aDMVE);
        //
        // find planes for connected edges
        int aNbV = aDMVE.Extent();
        for (i = 1; i <= aNbV; ++i)
        {
            NCollection_List<TopoDS_Shape> aLEI = aDMVE[i];
            if (aLEI.Extent() < 2)
            {
                continue;
            }
            //
            NCollection_List<TopoDS_Shape>.Iterator aItLEI1 = new(aLEI);
            for (; aItLEI1.More(); aItLEI1.Next())
            {
                TopoDS_Shape aEI1 = aItLEI1.Value();
                gp_Dir aDI1 = aDMEdgeTgt.FindFromKey(aEI1);
                //
                NCollection_List<TopoDS_Shape>.Iterator aItLEI2 = new(aLEI);
                for (; aItLEI2.More(); aItLEI2.Next())
                {
                    TopoDS_Shape aEI2 = aItLEI2.Value();
                    if (aEI2.IsSame(aEI1))
                    {
                        continue;
                    }
                    //
                    gp_Dir aDI2 = aDMEdgeTgt.FindFromKey(aEI2);
                    //
                    if (aDI1.IsParallel(aDI2, theAngTol))
                    {
                        continue;
                    }
                    //
                    gp_Dir aDNI = aDI1 ^ aDI2;
                    //
                    // check if this normal direction has not been checked yet
                    NCollection_List<gp_Dir>.Iterator aItLPln = new(aLPFence);
                    for (; aItLPln.More(); aItLPln.Next())
                    {
                        if (aDNI.IsParallel(aItLPln.Value(), theAngTol))
                        {
                            break;
                        }
                    }
                    if (aItLPln.More())
                    {
                        continue;
                    }
                    //
                    aLPFence.Append(aDNI);
                    //
                    // find all other edges in the plane parallel to current one
                    NCollection_IndexedMap<TopoDS_Shape> aMEPln = new(TopTools_ShapeMapHasher.Instance);
                    aMEPln.Add(aEI1);
                    aMEPln.Add(aEI2);
                    //
                    // iterate on all other edges to find all edges lying in the plane parallel to current one
                    for (j = 1; j <= aNbEdges; ++j)
                    {
                        gp_Dir aDJ = aDMEdgeTgt[j];
                        if (aDNI.IsNormal(aDJ, theAngTol))
                        {
                            aMEPln.Add(aDMEdgeTgt.FindKey(j));
                        }
                    }
                    //
                    MakeWires(aMEPln, aRWires, true, aDMEdgeTgt, aMEdgesNoUniquePlane);
                } // for (; aItLEI2.More(); aItLEI2.Next()) {
            } // for (; aItLEI1.More(); aItLEI1.Next()) {
        } // for (i = 1; i < aNb; ++i) {
        //
        // 4. Find unused edges and make wires from them
        NCollection_IndexedMap<TopoDS_Shape> aMEAlone = new(TopTools_ShapeMapHasher.Instance), aMEUsed = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aRWires, TopAbs_EDGE, aMEUsed);
        //
        for (i = 1; i <= aNbEdges; ++i)
        {
            TopoDS_Shape aE = aDMEdgeTgt.FindKey(i);
            if (!aMEUsed.Contains(aE))
            {
                aMEAlone.Add(aE);
            }
        }
        //
        MakeWires(aMEAlone, aRWires, false, aDMEdgeTgt, aMEdgesNoUniquePlane);
        //
        theWires = aRWires;
        //
        return iErr;
    }

    /// <summary>
    /// Makes planar faces from the wires of <paramref name="theWires"/>, grouping the wires lying in one plane. Returns
    /// true if any face was built.
    /// </summary>
    /// <remarks>
    /// OCCT adds p-curves on the planes to the input's edges and corrects their tolerances in place. A frozen input
    /// is copied at the entry (<see cref="BRepTools_CopyAtEntry"/>): the faces are built from the copy, whose sub-shapes
    /// left unchanged are the caller's own again in the result.
    /// </remarks>
    public static bool WiresToFaces(in TopoDS_Shape theWires, ref TopoDS_Shape theFaces, double theAngTol = 1.0e-8)
    {
        // OCCT.NET: OCCT writes into the input's edges (p-curves, tolerances): a frozen input is copied first
        BRepTools_CopyAtEntry aCopyAtEntry = new(theWires);
        TopoDS_Shape aWires = aCopyAtEntry.Shape();
        //
        BRep_Builder aBB = new();
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        TopoDS_Compound aRFaces = new();
        aBB.MakeCompound(ref aRFaces);
        //
        const double aMax = 1.0e+8;
        //
        // map to store the tangent vectors for the edges
        NCollection_IndexedDataMap<TopoDS_Shape, gp_Dir> aDMEdgeTgt = new(TopTools_ShapeMapHasher.Instance);
        // maps to store the planes found for the wires
        NCollection_IndexedDataMap<TopoDS_Shape, gp_Pln> aDMWirePln = new(TopTools_ShapeMapHasher.Instance);
        // map to store the tolerance for the wire
        NCollection_DataMap<TopoDS_Shape, double> aDMWireTol = new(TopTools_ShapeMapHasher.Instance);
        // edges for which the plane is not found
        NCollection_Map<TopoDS_Shape> aMEdgesNoUniquePlane = new(TopTools_ShapeMapHasher.Instance);
        //
        // Find planes for the wires
        TopExp_Explorer aExpW = new(aWires, TopAbs_WIRE);
        for (; aExpW.More(); aExpW.Next())
        {
            TopoDS_Wire aWire = TopoDS.Wire(aExpW.Current());
            gp_Pln aPlane = new();
            if (FindPlane(aWire, ref aPlane, aDMEdgeTgt, aMEdgesNoUniquePlane))
            {
                aDMWirePln.Add(aWire, aPlane);
                // find tolerance for the wire - max tolerance of its edges
                aDMWireTol.Bind(aWire, BRep_Tool.MaxTolerance(aWire, TopAbs_EDGE));
            }
        }
        //
        int i, j, aNb = aDMWirePln.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Shape aWireI = aDMWirePln.FindKey(i);
            if (aMFence.Contains(aWireI))
            {
                continue;
            }
            //
            gp_Pln aPlnI = aDMWirePln[i];
            //
            NCollection_List<TopoDS_Shape> aLW = new();
            aLW.Append(aWireI);
            aMFence.Add(aWireI);
            //
            double aTolI = aDMWireTol.Find(aWireI);
            //
            // Find other wires in the same plane
            for (j = i + 1; j <= aNb; ++j)
            {
                TopoDS_Shape aWireJ = aDMWirePln.FindKey(j);
                if (aMFence.Contains(aWireJ))
                {
                    continue;
                }
                //
                // check if the planes are the same
                gp_Pln aPlnJ = aDMWirePln[j];
                // check direction of the planes
                if (!aPlnI.Position().Direction().IsParallel(aPlnJ.Position().Direction(), theAngTol))
                {
                    continue;
                }
                // check distance between the planes
                double aDist = aPlnI.Distance(aPlnJ.Location());
                double aTolJ = aDMWireTol.Find(aWireJ);
                if (aDist > (aTolI + aTolJ))
                {
                    continue;
                }
                //
                aLW.Append(aWireJ);
                aMFence.Add(aWireJ);
            }
            //
            // Take the edges to build the face
            NCollection_List<TopoDS_Shape> aLE = new();
            NCollection_List<TopoDS_Shape>.Iterator aItLW = new(aLW);
            for (; aItLW.More(); aItLW.Next())
            {
                TopoDS_Iterator aItE = new(aItLW.Value());
                for (; aItE.More(); aItE.Next())
                {
                    aLE.Append(aItE.Value().Oriented(TopAbs_FORWARD));
                    aLE.Append(aItE.Value().Oriented(TopAbs_REVERSED));
                }
            }
            //
            // build planar face
            TopoDS_Face aFF = new BRepBuilderAPI_MakeFace(aPlnI, -aMax, aMax, -aMax, aMax).InternalFace();
            aFF.Orientation(TopAbs_FORWARD);
            //
            try
            {
                //
                // build pcurves for edges on this face
                BRepLib.BuildPCurveForEdgesOnPlane(aLE, aFF);
                //
                // split the face with the edges
                BOPAlgo_BuilderFace aBF = new();
                aBF.SetShapes(aLE);
                aBF.SetFace(aFF);
                aBF.Perform();
                if (aBF.HasErrors())
                {
                    continue;
                }
                //
                NCollection_List<TopoDS_Shape> aLFSp = aBF.Areas();
                NCollection_List<TopoDS_Shape>.Iterator aItLF = new(aLFSp);
                for (; aItLF.More(); aItLF.Next())
                {
                    TopoDS_Shape aFSp = aItLF.Value();
                    aBB.Add(aRFaces, aFSp);
                }
            }
            catch (Standard_Failure)
            {
                continue;
            }
        }
        //
        // fix tolerances of the resulting faces
        NCollection_IndexedMap<TopoDS_Shape> aMEmpty = new(TopTools_ShapeMapHasher.Instance);
        BOPTools_AlgoTools.CorrectTolerances(aRFaces, aMEmpty, 0.05, false);
        BOPTools_AlgoTools.CorrectShapeTolerances(aRFaces, aMEmpty, false);
        //
        theFaces = aCopyAtEntry.Finish(aRFaces);
        return theFaces.NbChildren() > 0;
    }

    // Makes wires from the separate blocks of the given edges (OCCT's file-static MakeWires).
    private static void MakeWires(NCollection_IndexedMap<TopoDS_Shape> theEdges,
                                  TopoDS_Compound theWires,
                                  bool theCheckUniquePlane,
                                  NCollection_IndexedDataMap<TopoDS_Shape, gp_Dir> theDMEdgeTgt,
                                  NCollection_Map<TopoDS_Shape> theMEdgesNoUniquePlane)
    {
        TopoDS_Compound aCE = new();
        new BRep_Builder().MakeCompound(ref aCE);
        int i, aNbE = theEdges.Extent();
        for (i = 1; i <= aNbE; ++i)
        {
            new BRep_Builder().Add(aCE, theEdges[i]);
        }
        //
        NCollection_List<TopoDS_Shape> aLCBE = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aCE, TopAbs_VERTEX, TopAbs_EDGE, aLCBE);
        //
        // make wire from each block
        NCollection_List<TopoDS_Shape>.Iterator aItLCB = new(aLCBE);
        for (; aItLCB.More(); aItLCB.Next())
        {
            TopoDS_Shape aCBE = aItLCB.Value();
            //
            if (theCheckUniquePlane)
            {
                gp_Pln aPln = new();
                if (!FindPlane(aCBE, ref aPln, theDMEdgeTgt, theMEdgesNoUniquePlane))
                {
                    continue;
                }
            }
            //
            TopoDS_Wire aWire = new();
            new BRep_Builder().MakeWire(ref aWire);
            for (TopoDS_Iterator aItE = new(aCBE); aItE.More(); aItE.Next())
            {
                new BRep_Builder().Add(aWire, aItE.Value());
            }
            //
            new BRep_Builder().Add(theWires, aWire);
        }
    }

    // Finds the tangent for the edge using the map (OCCT's file-static FindEdgeTangent); theTgt is written on success.
    private static bool FindEdgeTangent(in TopoDS_Edge theEdge,
                                        NCollection_IndexedDataMap<TopoDS_Shape, gp_Dir> theDMEdgeTgt,
                                        ref gp_Dir theTgt)
    {
        int anIndex = theDMEdgeTgt.FindIndex(theEdge);
        if (anIndex == 0)
        {
            gp_Vec aVTE = new();
            BRepAdaptor_Curve aBAC = new(theEdge);
            if (!FindEdgeTangent(aBAC, ref aVTE))
            {
                return false;
            }
            anIndex = theDMEdgeTgt.Add(theEdge, new gp_Dir(aVTE));
        }
        theTgt = theDMEdgeTgt[anIndex];
        return true;
    }

    // Finds the tangent for the edge (OCCT's file-static FindEdgeTangent).
    private static bool FindEdgeTangent(BRepAdaptor_Curve theCurve, ref gp_Vec theTangent)
    {
        if (!theCurve.Is3DCurve())
        {
            return false;
        }
        // for the line the tangent is defined by the direction
        if (theCurve.GetType() == GeomAbs_CurveType.GeomAbs_Line)
        {
            theTangent = new gp_Vec(theCurve.Line().Position().Direction());
            return true;
        }
        //
        // for other curves take D1 and check for its length
        double aT, aT1 = theCurve.FirstParameter(), aT2 = theCurve.LastParameter();
        const int aNbP = 11;
        double aDt = (aT2 - aT1) / aNbP;
        //
        for (aT = aT1 + aDt; aT <= aT2; aT += aDt)
        {
            theCurve.D1(aT, out gp_Pnt _, out theTangent);
            if (theTangent.Magnitude() > Precision.Confusion())
            {
                return true;
            }
        }
        //
        return false;
    }

    // Finds the plane in which the edge is located (OCCT's file-static FindPlane); thePlane is written on success.
    private static bool FindPlane(BRepAdaptor_Curve theCurve, ref gp_Pln thePlane)
    {
        if (!theCurve.Is3DCurve())
        {
            return false;
        }
        //
        bool bFound = true;
        gp_Vec aVN = new();
        switch (theCurve.GetType())
        {
            case GeomAbs_CurveType.GeomAbs_Line:
                return false;
            case GeomAbs_CurveType.GeomAbs_Circle:
                aVN = new gp_Vec(theCurve.Circle().Position().Direction());
                break;
            case GeomAbs_CurveType.GeomAbs_Ellipse:
                aVN = new gp_Vec(theCurve.Ellipse().Position().Direction());
                break;
            case GeomAbs_CurveType.GeomAbs_Hyperbola:
                aVN = new gp_Vec(theCurve.Hyperbola().Position().Direction());
                break;
            case GeomAbs_CurveType.GeomAbs_Parabola:
                aVN = new gp_Vec(theCurve.Parabola().Position().Direction());
                break;
            default:
            {
                // for all other types of curve compute two tangent vectors
                // on the curve and cross them
                bFound = false;
                double aT, aT1 = theCurve.FirstParameter(), aT2 = theCurve.LastParameter();
                const int aNbP = 11;
                double aDt = (aT2 - aT1) / aNbP;
                //
                aT = aT1;
                theCurve.D1(aT, out gp_Pnt _, out gp_Vec aV1);
                //
                for (aT = aT1 + aDt; aT <= aT2; aT += aDt)
                {
                    theCurve.D1(aT, out gp_Pnt _, out gp_Vec aV2);
                    //
                    aVN = aV1 ^ aV2;
                    if (aVN.Magnitude() > Precision.Confusion())
                    {
                        bFound = true;
                        break;
                    }
                }
                break;
            }
        }
        //
        if (bFound)
        {
            thePlane = new gp_Pln(theCurve.Value(theCurve.FirstParameter()), new gp_Dir(aVN));
        }
        return bFound;
    }

    // Finds the plane in which the wire is located (OCCT's file-static FindPlane); thePlane is written on success.
    private static bool FindPlane(in TopoDS_Shape theWire,
                                  ref gp_Pln thePlane,
                                  NCollection_IndexedDataMap<TopoDS_Shape, gp_Dir> theDMEdgeTgt,
                                  NCollection_Map<TopoDS_Shape> theMEdgesNoUniquePlane)
    {
        TopExp_Explorer aExpE1 = new(theWire, TopAbs_EDGE);
        if (!aExpE1.More())
        {
            return false;
        }
        //
        // try to find two not parallel edges in wire to get normal of the plane
        for (; aExpE1.More(); aExpE1.Next())
        {
            // get the first edge in the wire
            TopoDS_Edge aE1 = TopoDS.Edge(aExpE1.Current());
            //
            // find tangent for the first edge
            gp_Dir aDTE1 = new();
            if (!FindEdgeTangent(aE1, theDMEdgeTgt, ref aDTE1))
            {
                continue;
            }
            //
            // find the other edge not parallel to the first one
            TopExp_Explorer aExpE2 = new(theWire, TopAbs_EDGE);
            for (; aExpE2.More(); aExpE2.Next())
            {
                TopoDS_Edge aE2 = TopoDS.Edge(aExpE2.Current());
                if (aE1.IsSame(aE2))
                {
                    continue;
                }
                //
                // find tangent for the second edge
                gp_Dir aDTE2 = new();
                if (!FindEdgeTangent(aE2, theDMEdgeTgt, ref aDTE2))
                {
                    continue;
                }
                //
                if (aDTE1.IsParallel(aDTE2, Precision.Angular()))
                {
                    continue;
                }
                //
                gp_Dir aDN = aDTE1 ^ aDTE2;
                //
                TopoDS_Iterator aItV = new(aE1);
                thePlane = new gp_Pln(BRep_Tool.Pnt(TopoDS.Vertex(aItV.Value())), aDN);
                return true;
            }
        }
        //
        // try to compute normal on the single edge
        aExpE1.Init(theWire, TopAbs_EDGE);
        for (; aExpE1.More(); aExpE1.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(aExpE1.Current());
            if (theMEdgesNoUniquePlane.Contains(aE))
            {
                continue;
            }
            BRepAdaptor_Curve aBAC = new(aE);
            if (FindPlane(aBAC, ref thePlane))
            {
                return true;
            }
            theMEdgesNoUniquePlane.Add(aE);
        }
        return false;
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

    //=======================================================================
    // Classification of the faces relatively solids
    //=======================================================================

    /// <summary>A shape and its bounding box (OCCT's file-local BOPAlgo_ShapeBox).</summary>
    private sealed class BOPAlgo_ShapeBox
    {
        private TopoDS_Shape myShape = new();
        private Bnd_Box myBox = new();

        //! Sets the shape
        public void SetShape(in TopoDS_Shape theS) => myShape = theS;

        //! Returns the shape
        public TopoDS_Shape Shape() => myShape;

        //! Sets the bounding box
        public void SetBox(in Bnd_Box theBox) => myBox = theBox;

        //! Returns the bounding box
        public Bnd_Box Box() => myBox;
    }

    /// <summary>
    /// The classification of the selected faces relatively one solid, run in parallel (OCCT's file-local
    /// BOPAlgo_FillIn3DParts).
    /// </summary>
    private sealed class BOPAlgo_FillIn3DParts : BOPAlgo_ParallelAlgo, BOPTools_Parallel.IContextSolver<IntTools_Context>
    {
        private TopoDS_Solid mySolid = new();                                   //! Solid
        private Bnd_Box myBoxS = new();                                         // Bounding box of the solid
        private NCollection_List<TopoDS_Shape> myOwnIF = new();                 //! Own INTERNAL faces of the solid
        private readonly NCollection_List<TopoDS_Shape> myInFaces = new();      //! Faces classified as IN

        private BOPTools_BoxTree? myBBTree;                                     //! BVH tree of bounding boxes
        private NCollection_DynamicArray<BOPAlgo_ShapeBox>? myVShapeBox;        //! ShapeBoxMap

        private readonly TopoDS_Iterator myItF = new();                         //! Iterators
        private readonly TopoDS_Iterator myItW = new();

        private IntTools_Context? myContext;                                    //! Context

        public BOPAlgo_FillIn3DParts()
        {
            myBBTree = null;
            myVShapeBox = null;
        }

        //! Sets the solid
        public void SetSolid(in TopoDS_Solid theSolid) => mySolid = theSolid;

        //! Returns the solid
        public TopoDS_Solid Solid() => mySolid;

        //! Sets the box for the solid
        public void SetBoxS(in Bnd_Box theBox) => myBoxS = theBox;

        //! Returns the solid's box
        public Bnd_Box BoxS() => myBoxS;

        //! Sets own INTERNAL faces of the solid
        public void SetOwnIF(NCollection_List<TopoDS_Shape> theLIF) => myOwnIF = new NCollection_List<TopoDS_Shape>(theLIF);

        //! Returns own INTERNAL faces of the solid
        public NCollection_List<TopoDS_Shape> OwnIF() => myOwnIF;

        //! Sets the Bounding Box tree
        public void SetBBTree(BOPTools_BoxTree theBBTree) => myBBTree = theBBTree;

        //! Sets the ShapeBox structure
        public void SetShapeBoxVector(NCollection_DynamicArray<BOPAlgo_ShapeBox> theShapeBox) => myVShapeBox = theShapeBox;

        //! Sets the context
        public void SetContext(IntTools_Context theContext) => myContext = theContext;

        //! Returns the context
        public IntTools_Context? Context() => myContext;

        //! Returns the faces classified as IN for solid
        public NCollection_List<TopoDS_Shape> InFaces() => myInFaces;

        //! Performs the classification
        public override void Perform()
        {
            using var aPSOuter = new Message_ProgressScope(myProgressRange, null, 2);
            if (UserBreak(aPSOuter))
            {
                return;
            }

            myInFaces.Clear();

            // 1. Select boxes of faces that are not out of aBoxS
            BOPTools_BoxTreeSelector aSelector = new();
            aSelector.SetBox(Bnd_Tools.Bnd2BVH(myBoxS));
            aSelector.SetBVHSet(myBBTree!);
            //
            if (aSelector.Select() == 0)
            {
                return;
            }

            NCollection_List<int> aLIFP = aSelector.Indices();

            // 2. Fill maps of edges and faces of the solid

            NCollection_DynamicArray<BOPAlgo_ShapeBox> aVShapeBox = myVShapeBox!;

            NCollection_IndexedMap<TopoDS_Shape> aMSE = new(TopTools_ShapeMapHasher.Instance), aMSF = new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(mySolid, TopAbs_EDGE, aMSE);
            TopExp.MapShapes(mySolid, TopAbs_FACE, aMSF);

            // Check if the Solid contains any faces
            bool bIsEmpty = aMSF.IsEmpty();

            // Add own internal faces of the solid into aMSF
            NCollection_List<TopoDS_Shape>.Iterator aItLS = new(myOwnIF);
            for (; aItLS.More(); aItLS.Next())
            {
                aMSF.Add(aItLS.Value());
            }

            // 3. aIVec - faces to process.
            //    Filter the selected faces with faces of the solid.
            List<int> aIVec = new(256);

            NCollection_List<int>.Iterator aItLI = new(aLIFP);
            for (; aItLI.More(); aItLI.Next())
            {
                int nFP = aItLI.Value();
                TopoDS_Shape aFP0 = aVShapeBox[nFP].Shape();
                if (!aMSF.Contains(aFP0))
                {
                    aIVec.Add(nFP);
                }
            }

            // 4. Classify faces relatively solid.
            //    Store faces that are IN mySolid into <myInFaces>

            int k, aNbFP = aIVec.Count;
            // Sort indices if necessary (distinct integers: any sort gives std::sort's order)
            if (aNbFP > 1)
            {
                aIVec.Sort();
            }

            if (bIsEmpty)
            {
                // The solid is empty as it does not contain any faces.
                // It could happen when the input solid consists of INTERNAL faces only.
                // Classification of any point relatively empty solid would always give IN status.
                // Thus, we consider all selected faces as IN without real classification.
                for (k = 0; k < aNbFP; ++k)
                {
                    myInFaces.Append(aVShapeBox[aIVec[k]].Shape());
                }

                return;
            }

            // Prepare EF map of faces to process for building connexity blocks
            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMEFP = new(TopTools_ShapeMapHasher.Instance);
            if (aNbFP > 1)
            {
                for (k = 0; k < aNbFP; ++k)
                {
                    MapEdgesAndFaces(aVShapeBox[aIVec[k]].Shape(), aMEFP);
                }
            }

            aPSOuter.Next();

            // Map of Edge-Face connection, necessary for solid classification.
            // It will be filled when first classification is performed.
            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMEFDS = new(TopTools_ShapeMapHasher.Instance);

            // Fence map to avoid processing of the same faces twice
            NCollection_Map<TopoDS_Shape> aMFDone = new(TopTools_ShapeMapHasher.Instance);

            // Per-iteration list for connexity block faces
            NCollection_List<TopoDS_Shape> aLCBF = new();

            using var aPSLoop = new Message_ProgressScope(aPSOuter.Next(), null, aNbFP);
            for (k = 0; k < aNbFP; ++k, aPSLoop.Next())
            {
                if (UserBreak(aPSLoop))
                {
                    return;
                }
                aLCBF.Clear();

                int nFP = aIVec[k];
                TopoDS_Face aFP = TopoDS.Face(aVShapeBox[nFP].Shape());
                if (!aMFDone.Add(aFP))
                {
                    continue;
                }

                // Make connexity blocks of faces, avoiding passing through the
                // borders of the solid. It helps to reduce significantly the
                // number of classified faces.
                // The most appropriate face for classification
                TopoDS_Face aFaceToClassify = new();
                MakeConnexityBlock(aFP, aMSE, aMEFP, aMFDone, aLCBF, ref aFaceToClassify);

                if (!myBoxS.IsWhole())
                {
                    // First, try fast classification of the whole block by additional
                    // check on bounding boxes - check that bounding boxes of all vertices
                    // of the block interfere with the box of the solid.
                    // If not, the faces are out.
                    bool bOut = false;
                    aItLS.Initialize(aLCBF);
                    for (; aItLS.More() && !bOut; aItLS.Next())
                    {
                        TopExp_Explorer anExpV = new(aItLS.Value(), TopAbs_VERTEX);
                        for (; anExpV.More() && !bOut; anExpV.Next())
                        {
                            TopoDS_Vertex aV = TopoDS.Vertex(anExpV.Current());
                            Bnd_Box aBBV = new();
                            aBBV.Add(BRep_Tool.Pnt(aV));
                            aBBV.SetGap(BRep_Tool.Tolerance(aV));
                            bOut = myBoxS.IsOut(aBBV);
                        }
                    }
                    if (bOut)
                    {
                        continue;
                    }
                }

                if (aFaceToClassify.IsNull())
                {
                    aFaceToClassify = aFP;
                }

                if (aMEFDS.IsEmpty())
                {
                    // Fill EF map for Solid
                    TopExp.MapShapesAndAncestors(mySolid, TopAbs_EDGE, TopAbs_FACE, aMEFDS);
                }

                // All vertices are interfere with the solids box, run classification.
                bool bIsIN = BOPTools_AlgoTools.IsInternalFace(aFaceToClassify,
                                                               mySolid,
                                                               aMEFDS,
                                                               Precision.Confusion(),
                                                               myContext!);
                if (bIsIN)
                {
                    aItLS.Initialize(aLCBF);
                    for (; aItLS.More(); aItLS.Next())
                    {
                        myInFaces.Append(aItLS.Value());
                    }
                }
            }
        }

        //! Prepares Edge-Face connection map of the given shape
        private void MapEdgesAndFaces(in TopoDS_Shape theF,
                                      NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEFMap)
        {
            myItF.Initialize(theF);
            for (; myItF.More(); myItF.Next())
            {
                TopoDS_Shape aW = myItF.Value();
                if (aW.ShapeType() != TopAbs_WIRE)
                {
                    continue;
                }

                myItW.Initialize(aW);
                for (; myItW.More(); myItW.Next())
                {
                    TopoDS_Shape aE = myItW.Value();

                    int anIndex = theEFMap.FindIndex(aE);
                    NCollection_List<TopoDS_Shape> pLF = anIndex > 0
                                                         ? theEFMap[anIndex]
                                                         : theEFMap[theEFMap.Add(aE, new NCollection_List<TopoDS_Shape>())];
                    pLF.Append(theF);
                }
            }
        }

        //! Makes the connexity block of faces using the connection map
        private void MakeConnexityBlock(in TopoDS_Face theFStart,
                                        NCollection_IndexedMap<TopoDS_Shape> theMEAvoid,
                                        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theEFMap,
                                        NCollection_Map<TopoDS_Shape> theMFDone,
                                        NCollection_List<TopoDS_Shape> theLCB,
                                        ref TopoDS_Face theFaceToClassify)
        {
            // Add start element
            theLCB.Append(theFStart);
            if (theEFMap.IsEmpty())
            {
                return;
            }

            NCollection_List<TopoDS_Shape>.Iterator aItCB = new(theLCB);
            for (; aItCB.More(); aItCB.Next())
            {
                TopoDS_Shape aF = aItCB.Value();
                myItF.Initialize(aF);
                for (; myItF.More(); myItF.Next())
                {
                    TopoDS_Shape aW = myItF.Value();
                    if (aW.ShapeType() != TopAbs_WIRE)
                    {
                        continue;
                    }

                    myItW.Initialize(aW);
                    for (; myItW.More(); myItW.Next())
                    {
                        TopoDS_Edge aE = TopoDS.Edge(myItW.Value());
                        if (theMEAvoid.Contains(aE) || BRep_Tool.Degenerated(aE))
                        {
                            if (theFaceToClassify.IsNull())
                            {
                                theFaceToClassify = TopoDS.Face(aF);
                            }
                            continue;
                        }

                        if (!theEFMap.FindFromKey(aE, out NCollection_List<TopoDS_Shape> pLF))
                        {
                            continue;
                        }
                        NCollection_List<TopoDS_Shape>.Iterator aItLF = new(pLF);
                        for (; aItLF.More(); aItLF.Next())
                        {
                            TopoDS_Shape aFToAdd = aItLF.Value();
                            if (theMFDone.Add(aFToAdd))
                            {
                                theLCB.Append(aFToAdd);
                            }
                        }
                    }
                }
            }
        }
    }

    /// <summary>
    /// Classifies the faces relatively the solids: fills <paramref name="theInParts"/> with each solid and the faces
    /// IN it. The boxes of <paramref name="theShapeBoxMap"/> are used where given (null: none), and the faces of
    /// <paramref name="theSolidsIF"/> (each solid's own internal faces; null: none) are not classified relatively
    /// their solid. Runs in parallel if <paramref name="theRunParallel"/>.
    /// </summary>
    public static void ClassifyFaces(NCollection_List<TopoDS_Shape> theFaces,
                                     NCollection_List<TopoDS_Shape> theSolids,
                                     bool theRunParallel,
                                     IntTools_Context theContext,
                                     NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theInParts,
                                     NCollection_DataMap<TopoDS_Shape, Bnd_Box>? theShapeBoxMap = null,
                                     NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>? theSolidsIF = null,
                                     Message_ProgressRange theRange = default)
    {
        theShapeBoxMap ??= new NCollection_DataMap<TopoDS_Shape, Bnd_Box>(TopTools_ShapeMapHasher.Instance);
        theSolidsIF ??= new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);

        using var aPSOuter = new Message_ProgressScope(theRange, null, 10);

        // Fill the vector of shape box with faces and its bounding boxes
        NCollection_DynamicArray<BOPAlgo_ShapeBox> aVSB = new(256);

        NCollection_List<TopoDS_Shape>.Iterator aItLF = new(theFaces);
        for (; aItLF.More(); aItLF.Next())
        {
            if (!aPSOuter.More())
            {
                return;
            }

            TopoDS_Shape aF = aItLF.Value();
            // Append face to the vector of shape box
            BOPAlgo_ShapeBox aSB = aVSB.Append(new BOPAlgo_ShapeBox());
            aSB.SetShape(aF);

            // Get bounding box for the face
            if (theShapeBoxMap.Find(aF, out Bnd_Box pBox))
            {
                aSB.SetBox(pBox);
            }
            else
            {
                // Build the bounding box
                Bnd_Box aBox = new();
                BRepBndLib.Add(aF, ref aBox);
                aSB.SetBox(aBox);
            }
        }

        // Prepare BVH tree of bounding boxes of the faces to classify
        // taking the bounding boxes from the just prepared vector
        BOPTools_BoxTree aBBTree = new();
        int aNbF = aVSB.Length();
        aBBTree.SetSize(aNbF);
        for (int i = 0; i < aNbF; ++i)
        {
            aBBTree.Add(i, Bnd_Tools.Bnd2BVH(aVSB[i].Box()));
        }

        // Shake tree filler
        aBBTree.Build();

        // Prepare vector of solids to classify
        NCollection_DynamicArray<BOPAlgo_FillIn3DParts> aVFIP = new();

        NCollection_List<TopoDS_Shape>.Iterator aItLS = new(theSolids);
        for (; aItLS.More(); aItLS.Next())
        {
            TopoDS_Solid aSolid = TopoDS.Solid(aItLS.Value());
            // Append solid to the vector
            BOPAlgo_FillIn3DParts aFIP = aVFIP.Append(new BOPAlgo_FillIn3DParts());
            aFIP.SetSolid(aSolid);

            // Get bounding box for the solid
            if (theShapeBoxMap.Find(aSolid, out Bnd_Box pBox))
            {
                aFIP.SetBoxS(pBox);
            }
            else
            {
                // Build the bounding box
                Bnd_Box aBox = new();
                BRepBndLib.Add(aSolid, ref aBox);
                if (!aBox.IsWhole())
                {
                    if (BOPTools_AlgoTools.IsInvertedSolid(aSolid))
                    {
                        aBox.SetWhole();
                    }
                }
                aFIP.SetBoxS(aBox);
            }

            if (theSolidsIF.Find(aSolid, out NCollection_List<TopoDS_Shape> pLIF))
            {
                aFIP.SetOwnIF(pLIF);
            }

            aFIP.SetBBTree(aBBTree);
            aFIP.SetShapeBoxVector(aVSB);
        }

        // Close preparation task
        aPSOuter.Next();
        // Set progress range for each task to be run in parallel
        int aNbS = aVFIP.Length();
        using var aPSParallel = new Message_ProgressScope(aPSOuter.Next(9), "Classification of faces relatively solids", aNbS);
        for (int iFS = 0; iFS < aNbS; ++iFS)
        {
            aVFIP.ChangeValue(iFS).SetProgressRange(aPSParallel.Next());
        }
        // Perform classification
        //================================================================
        BOPTools_Parallel.Perform(theRunParallel, aVFIP, theContext);
        //================================================================
        // Analyze the results and fill the resulting map
        for (int i = 0; i < aNbS; ++i)
        {
            BOPAlgo_FillIn3DParts aFIP = aVFIP[i];
            TopoDS_Shape aS = aFIP.Solid();
            NCollection_List<TopoDS_Shape> aLFIn = aFIP.InFaces();
            theInParts.Add(aS, new NCollection_List<TopoDS_Shape>(aLFIn));
        }
    }

    /// <summary>
    /// Classifies the parts (vertices, edges and faces, or containers of them; replaced by their images in
    /// <paramref name="theImages"/> where they have some) relatively the solids, and adds those IN a solid into it as
    /// INTERNAL parts (the faces as internal shells). The solids are changed in place: they must be the caller's own
    /// unfrozen solids.
    /// </summary>
    public static void FillInternals(NCollection_List<TopoDS_Shape> theSolids,
                                     NCollection_List<TopoDS_Shape> theParts,
                                     NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theImages,
                                     IntTools_Context theContext)
    {
        if (theSolids.IsEmpty() || theParts.IsEmpty())
        {
            return;
        }

        // Map the solids to avoid classification of the own shapes of the solids
        NCollection_IndexedMap<TopoDS_Shape> aMSSolids = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator itLS = new(theSolids);
        for (; itLS.More(); itLS.Next())
        {
            TopoDS_Shape aSolid = itLS.Value();
            if (aSolid.ShapeType() == TopAbs_SOLID)
            {
                TopExp.MapShapes(aSolid, TopAbs_VERTEX, aMSSolids);
                TopExp.MapShapes(aSolid, TopAbs_EDGE, aMSSolids);
                TopExp.MapShapes(aSolid, TopAbs_FACE, aMSSolids);
            }
        }

        // Extract BRep elements from the given parts and
        // check them for possible splits
        NCollection_List<TopoDS_Shape> aLPartsInput = new(theParts), aLParts = new();
        NCollection_List<TopoDS_Shape>.Iterator itLP = new(aLPartsInput);
        for (; itLP.More(); itLP.Next())
        {
            TopoDS_Shape aPart = itLP.Value();
            switch (aPart.ShapeType())
            {
                case TopAbs_VERTEX:
                case TopAbs_EDGE:
                case TopAbs_FACE:
                {
                    if (theImages.Find(aPart, out NCollection_List<TopoDS_Shape> pIm))
                    {
                        NCollection_List<TopoDS_Shape>.Iterator itIm = new(pIm);
                        for (; itIm.More(); itIm.Next())
                        {
                            TopoDS_Shape aPartIm = itIm.Value();
                            if (!aMSSolids.Contains(aPartIm))
                            {
                                aLParts.Append(aPartIm);
                            }
                        }
                    }
                    else if (!aMSSolids.Contains(aPart))
                    {
                        aLParts.Append(aPart);
                    }

                    break;
                }
                default:
                {
                    for (TopoDS_Iterator it = new(aPart); it.More(); it.Next())
                    {
                        aLPartsInput.Append(it.Value());
                    }
                    break;
                }
            }
        }

        // Classify the given parts relatively solids.
        // Add edges and vertices classified as IN into solids instantly,
        // and collect faces classified as IN into a list for further shell creation

        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anINFaces = new(TopTools_ShapeMapHasher.Instance);
        itLS.Initialize(theSolids);
        for (; itLS.More(); itLS.Next())
        {
            TopoDS_Shape aSolid = itLS.Value();
            if (aSolid.ShapeType() != TopAbs_SOLID)
            {
                continue;
            }

            TopoDS_Solid aSd = TopoDS.Solid(aSolid);

            itLP.Initialize(aLParts);
            for (; itLP.More();)
            {
                TopoDS_Shape aPart = itLP.Value();
                TopAbs_State aState = BOPTools_AlgoTools.ComputeStateByOnePoint(aPart, aSd, Precision.Confusion(), theContext);
                if (aState == TopAbs_State.TopAbs_IN)
                {
                    if (aPart.ShapeType() == TopAbs_FACE)
                    {
                        if (!anINFaces.Find(aSd, out NCollection_List<TopoDS_Shape> pFaces))
                        {
                            pFaces = anINFaces.Bound(aSd, new NCollection_List<TopoDS_Shape>());
                        }
                        pFaces.Append(aPart);
                    }
                    else
                    {
                        aPart.Orientation(TopAbs_INTERNAL);
                        new BRep_Builder().Add(aSd, aPart);
                    }
                    aLParts.Remove(itLP);
                }
                else
                {
                    itLP.Next();
                }
            }
        }

        // Make shells from faces and put them into solids
        // (native iterates this map in address order; each solid gets only its own shells, so the order isn't observable)
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> itM in anINFaces)
        {
            TopoDS_Solid aSd = TopoDS.Solid(itM.Key);
            NCollection_List<TopoDS_Shape> aFaces = itM.Value;

            TopoDS_Compound aCF = new();
            new BRep_Builder().MakeCompound(ref aCF);

            NCollection_List<TopoDS_Shape>.Iterator itLF = new(aFaces);
            for (; itLF.More(); itLF.Next())
            {
                new BRep_Builder().Add(aCF, itLF.Value());
            }

            // Build blocks from the faces
            NCollection_List<TopoDS_Shape> aLCB = new();
            BOPTools_AlgoTools.MakeConnexityBlocks(aCF, TopAbs_EDGE, TopAbs_FACE, aLCB);

            // Build shell from each block
            NCollection_List<TopoDS_Shape>.Iterator itCB = new(aLCB);
            for (; itCB.More(); itCB.Next())
            {
                TopoDS_Shape aCB = itCB.Value();

                TopoDS_Shell aShell = new();
                new BRep_Builder().MakeShell(ref aShell);
                // Add faces of the block to the shell
                TopExp_Explorer expF = new(aCB, TopAbs_FACE);
                for (; expF.More(); expF.Next())
                {
                    TopoDS_Face aFInt = TopoDS.Face(expF.Current());
                    aFInt.Orientation(TopAbs_INTERNAL);
                    new BRep_Builder().Add(aShell, aFInt);
                }

                new BRep_Builder().Add(aSd, aShell);
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
