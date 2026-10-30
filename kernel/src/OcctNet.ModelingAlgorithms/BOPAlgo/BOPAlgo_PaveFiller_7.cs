// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_7.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The split of an edge by a pave block (and the tolerance of its common block), as a parallel solver.</summary>
internal sealed class BOPAlgo_SplitEdge : BOPAlgo_ParallelAlgo, BOPTools_Parallel.IContextSolver<IntTools_Context>
{
    // ->
    private TopoDS_Edge myE = new();
    private TopoDS_Vertex myV1 = new();
    private double myT1;
    private TopoDS_Vertex myV2 = new();
    private double myT2;
    // <->
    private BOPDS_PaveBlock? myPB;
    private BOPDS_CommonBlock? myCB;
    // <-
    private TopoDS_Edge myESp = new();
    private Bnd_Box myBox = new();
    private double myTol;
    //
    private BOPDS_DS? myDS;
    private IntTools_Context? myContext;

    public BOPAlgo_SplitEdge()
    {
        myT1 = 0.0;
        myT2 = 0.0;
        myTol = 0.0;
    }

    public void SetData(in TopoDS_Edge aE, in TopoDS_Vertex aV1, double aT1, in TopoDS_Vertex aV2, double aT2)
    {
        myE = aE;
        myV1 = aV1;
        myT1 = aT1;
        myV2 = aV2;
        myT2 = aT2;
        myESp = aE;
    }

    public void SetPaveBlock(BOPDS_PaveBlock aPB) => myPB = aPB;

    public BOPDS_PaveBlock PaveBlock() => myPB!;

    public void SetCommonBlock(BOPDS_CommonBlock? aCB) => myCB = aCB;

    public BOPDS_CommonBlock? CommonBlock() => myCB;

    public TopoDS_Edge SplitEdge() => myESp;

    public Bnd_Box Box() => myBox;

    public double Tolerance() => myTol;

    public void SetDS(BOPDS_DS theDS) => myDS = theDS;

    public void SetContext(IntTools_Context aContext) => myContext = aContext;

    public override void Perform()
    {
        using var aPS = new Message_ProgressScope(myProgressRange, null, 1);
        if (UserBreak(aPS))
        {
            return;
        }
        myTol = BOPAlgo_Tools.ComputeToleranceOfCB(myCB, myDS!, myContext);
        BOPTools_AlgoTools.MakeSplitEdge(myE, myV1, myT1, myV2, myT2, out myESp);
        BRepBndLib.Add(myESp, ref myBox);
        myBox.SetGap(myBox.GetGap() + Precision.Confusion());
    }
}

/// <summary>The p-curve of an edge on a face, as a parallel solver.</summary>
/// <remarks>
/// The solver builds a new p-curve only on a copy of the edge (<c>CopyEdge</c>) or adjusts an existing one; the pave
/// filler stores the new p-curve afterwards, serially. The vertex tolerances of section edges (<c>myFlag</c>) are
/// updated in place by <c>UpdateVertices</c>, in parallel, as in OCCT.
/// </remarks>
internal sealed class BOPAlgo_MPC : BOPAlgo_ParallelAlgo, BOPTools_Parallel.IContextSolver<IntTools_Context>
{
    private bool myFlag;
    private TopoDS_Edge myE = new();
    private TopoDS_Face myF = new();
    private TopoDS_Edge myEz = new();
    private TopoDS_Vertex myV1 = new();
    private double myT1;
    private TopoDS_Vertex myV2 = new();
    private double myT2;
    private Geom2d_Curve? myNewC2d;
    private double myNewTol;
    //
    private IntTools_Context? myContext;

    public BOPAlgo_MPC()
    {
        myFlag = false;
    }

    public void SetEdge(in TopoDS_Edge aE) => myE = aE;

    public TopoDS_Edge Edge() => myE;

    public void SetFace(in TopoDS_Face aF) => myF = aF;

    public TopoDS_Face Face() => myF;

    public void SetFlag(bool bFlag) => myFlag = bFlag;

    public bool Flag() => myFlag;

    public void SetData(in TopoDS_Edge aEz, in TopoDS_Vertex aV1, double aT1, in TopoDS_Vertex aV2, double aT2)
    {
        myEz = aEz;
        myV1 = aV1;
        myT1 = aT1;
        myV2 = aV2;
        myT2 = aT2;
    }

    public void SetContext(IntTools_Context aContext) => myContext = aContext;

    public IntTools_Context? Context() => myContext;

    public override void Perform()
    {
        using var aPS = new Message_ProgressScope(myProgressRange, null, 1);
        if (UserBreak(aPS))
        {
            return;
        }
        try
        {
            // Check if edge has pcurve. If no then make its copy to avoid data races,
            // and use it to build pcurve.
            TopoDS_Edge aCopyE = myE;
            double f, l;
            Geom2d_Curve? aC2d = BRep_Tool.CurveOnSurface(aCopyE, myF, out f, out l);
            if (aC2d is null)
            {
                aCopyE = BOPTools_AlgoTools.CopyEdge(aCopyE);

                int iErr = 1;
                if (!myEz.IsNull())
                {
                    // Attach pcurve from the original edge
                    BOPTools_AlgoTools.MakeSplitEdge(myEz, myV1, myT1, myV2, myT2, out TopoDS_Edge aSpz);
                    iErr = BOPTools_AlgoTools2D.AttachExistingPCurve(aSpz, aCopyE, myF, myContext);
                }
                if (iErr != 0)
                {
                    BOPTools_AlgoTools2D.BuildPCurveForEdgeOnFace(aCopyE, myF, myContext);
                }

                myNewC2d = BRep_Tool.CurveOnSurface(aCopyE, myF, out f, out l);
                if (myNewC2d is null)
                {
                    AddError(new BOPAlgo_AlertBuildingPCurveFailed(new TopoDS_Shape()));
                    return;
                }
                else
                {
                    myNewTol = BRep_Tool.Tolerance(aCopyE);
                }
            }
            else
            {
                BRepAdaptor_Surface aBAS = myContext!.SurfaceAdaptor(myF);
                if (aBAS.IsUPeriodic() || aBAS.IsVPeriodic())
                {
                    // The curve already exists. Adjust it for periodic cases.
                    BOPTools_AlgoTools2D.AdjustPCurveOnSurf(myContext.SurfaceAdaptor(myF), f, l, aC2d, out myNewC2d);
                    if (myNewC2d != aC2d)
                    {
                        myNewTol = BRep_Tool.Tolerance(aCopyE);
                    }
                    else
                    {
                        myNewC2d = null;
                    }
                }
            }

            if (myFlag)
            {
                // OCCT: updates the tolerances of the edge's vertices in place, from parallel solvers that may share
                // a vertex (unsynchronised read-compare-writes), and in non-destructive mode a section edge may have an
                // argument vertex (see BOPAlgo_PaveFiller.UpdateVertices).
                BOPAlgo_PaveFiller.UpdateVertices(aCopyE, myF);
            }
        }
        catch (Standard_Failure)
        {
            AddError(new BOPAlgo_AlertBuildingPCurveFailed(new TopoDS_Shape()));
        }
    }

    public Geom2d_Curve? GetNewPCurve() => myNewC2d;

    public double GetNewTolerance() => myNewTol;
}

/// <summary>The p-curve of an edge on a planar face (<c>BRepLib.BuildPCurveForEdgeOnPlane</c>), as a parallel solver.</summary>
internal sealed class BOPAlgo_BPC : BOPTools_Parallel.ISolver
{
    private TopoDS_Edge myE = new();
    private TopoDS_Face myF = new();
    private Geom2d_Curve? myCurve;
    private bool myToUpdate;
    private Message_ProgressRange myRange;

    public void SetFace(in TopoDS_Face aF) => myF = aF;

    public void SetEdge(in TopoDS_Edge aE) => myE = aE;

    public TopoDS_Edge GetEdge() => myE;

    public TopoDS_Face GetFace() => myF;

    public Geom2d_Curve? GetCurve2d() => myCurve;

    public bool IsToUpdate() => myToUpdate;

    public void SetRange(Message_ProgressRange theRange) => myRange = theRange;

    public void Perform()
    {
        using var aPS = new Message_ProgressScope(myRange, null, 1);
        if (!aPS.More())
        {
            return;
        }
        BRepLib.BuildPCurveForEdgeOnPlane(myE, myF, out myCurve, out myToUpdate);
    }
}

public partial class BOPAlgo_PaveFiller
{
    /// <summary>Makes the split edges of the pave blocks (one per common block).</summary>
    protected void MakeSplitEdges(Message_ProgressRange theRange)
    {
        NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> aPBP = myDS!.ChangePaveBlocksPool();
        int aNbPBP = aPBP.Length();
        using var aPSOuter = new Message_ProgressScope(theRange, null, 1);
        if (aNbPBP == 0)
        {
            return;
        }
        //
        int i, nE, nV1, nV2, nSp, aNbVBSE, k;
        double aT1, aT2;
        NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new();
        BOPDS_PaveBlock aPB;
        NCollection_Map<BOPDS_CommonBlock> aMCB = new();
        TopoDS_Vertex aV1, aV2;
        TopoDS_Edge aE;
        NCollection_DynamicArray<BOPAlgo_SplitEdge> aVBSE = new();

        //
        UpdateCommonBlocksWithSDVertices();
        //
        aNbPBP = aPBP.Length();
        //
        for (i = 0; i < aNbPBP; ++i)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            NCollection_List<BOPDS_PaveBlock> aLPB = aPBP[i];
            //
            aItPB.Initialize(aLPB);
            for (; aItPB.More(); aItPB.Next())
            {
                aPB = aItPB.Value();
                nE = aPB.OriginalEdge();
                BOPDS_ShapeInfo aSIE = myDS.ShapeInfo(nE);
                if (aSIE.HasFlag())
                {
                    // Skip degenerated edges
                    continue;
                }

                BOPDS_CommonBlock? aCB = myDS.CommonBlock(aPB);
                bool bCB = aCB is not null;
                if (bCB && !aMCB.Add(aCB!))
                {
                    continue;
                }

                aPB.Indices(out nV1, out nV2);
                // Check if it is necessary to make the split of the edge
                {
                    bool bV1 = myDS.IsNewShape(nV1);
                    bool bV2 = myDS.IsNewShape(nV2);

                    bool bToSplit = true;
                    if (!bV1 && !bV2) // no new vertices here
                    {
                        if (!myNonDestructive || !bCB)
                        {
                            if (bCB)
                            {
                                // Find the edge with these vertices
                                NCollection_List<BOPDS_PaveBlock>.Iterator it = new(aCB!.PaveBlocks());
                                for (; it.More(); it.Next())
                                {
                                    nE = it.Value().OriginalEdge();
                                    if (myDS.PaveBlocks(nE).Extent() == 1)
                                    {
                                        break;
                                    }
                                }
                                if (it.More())
                                {
                                    // The pave block is found
                                    bToSplit = false;
                                    aCB.SetRealPaveBlock(it.Value());
                                    aCB.SetEdge(nE);
                                    // Compute tolerance of the common block and update the edge
                                    double aTol = BOPAlgo_Tools.ComputeToleranceOfCB(aCB, myDS, myContext);
                                    UpdateEdgeTolerance(nE, aTol);
                                }
                            }
                            else if (aLPB.Extent() == 1)
                            {
                                bToSplit = false;
                                aPB.SetEdge(nE);
                            }
                            if (!bToSplit)
                            {
                                continue;
                            }
                        }
                    }
                }

                // Split the edge
                if (bCB)
                {
                    aPB = aCB!.PaveBlock1();
                    nE = aPB.OriginalEdge();
                    aPB.Indices(out nV1, out nV2);
                }
                aPB.Range(out aT1, out aT2);
                //
                aE = TopoDS.Edge(myDS.Shape(nE));
                aE.Orientation(TopAbs_FORWARD);
                //
                aV1 = TopoDS.Vertex(myDS.Shape(nV1));
                aV1.Orientation(TopAbs_FORWARD);
                //
                aV2 = TopoDS.Vertex(myDS.Shape(nV2));
                aV2.Orientation(TopAbs_REVERSED);
                //
                BOPAlgo_SplitEdge aBSE = aVBSE.Append(new BOPAlgo_SplitEdge());
                //
                aBSE.SetData(aE, aV1, aT1, aV2, aT2);
                aBSE.SetPaveBlock(aPB);
                if (bCB)
                {
                    aBSE.SetCommonBlock(aCB);
                }
                aBSE.SetDS(myDS);
            } // for (; aItPB.More(); aItPB.Next()) {
        } // for (i=0; i<aNbPBP; ++i) {
        //
        aNbVBSE = aVBSE.Length();
        using var aPS = new Message_ProgressScope(aPSOuter.Next(), "Splitting edges", aNbVBSE);
        for (k = 0; k < aNbVBSE; k++)
        {
            BOPAlgo_SplitEdge aBSE = aVBSE[k];
            aBSE.SetProgressRange(aPS.Next());
        }
        //======================================================
        BOPTools_Parallel.Perform(myRunParallel, aVBSE, myContext!);
        //======================================================
        if (HasErrors())
        {
            return;
        }
        //
        for (k = 0; k < aNbVBSE; ++k)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            BOPAlgo_SplitEdge aBSE = aVBSE[k];
            //
            TopoDS_Edge aSp = aBSE.SplitEdge();
            Bnd_Box aBox = aBSE.Box();
            //
            BOPDS_PaveBlock aPBk = aBSE.PaveBlock();
            BOPDS_CommonBlock? aCBk = aBSE.CommonBlock();
            //
            BOPDS_ShapeInfo aSI = new();
            aSI.SetShapeType(TopAbs_EDGE);
            aSI.SetShape(aSp);
            aSI.ChangeBox() = aBox;
            NCollection_List<int> aSubShapes = aSI.ChangeSubShapes();
            aSubShapes.Append(aPBk.Pave1().Index());
            aSubShapes.Append(aPBk.Pave2().Index());
            //
            nSp = myDS.Append(aSI);
            //
            if (aCBk is not null)
            {
                UpdateEdgeTolerance(nSp, aBSE.Tolerance());
                aCBk.SetEdge(nSp);
            }
            else
            {
                aPBk.SetEdge(nSp);
            }
        }
    }

    /// <summary>Splits the edge <paramref name="nE"/> between the two vertices and appends the split to the DS.</summary>
    protected int SplitEdge(int nE, int nV1, double aT1, int nV2, double aT2)
    {
        int nSp;
        TopoDS_Vertex aV1, aV2;
        TopoDS_Edge aE, aSp;
        BOPDS_ShapeInfo aSI = new();
        //
        aSI.SetShapeType(TopAbs_EDGE);
        //
        aE = TopoDS.Edge(myDS!.Shape(nE));
        aE.Orientation(TopAbs_FORWARD);
        //
        aV1 = TopoDS.Vertex(myDS.Shape(nV1));
        aV1.Orientation(TopAbs_FORWARD);
        //
        aV2 = TopoDS.Vertex(myDS.Shape(nV2));
        aV2.Orientation(TopAbs_REVERSED);
        //
        BOPTools_AlgoTools.MakeSplitEdge(aE, aV1, aT1, aV2, aT2, out aSp);
        //
        aSI.SetShape(aSp);
        //
        ref Bnd_Box aBox = ref aSI.ChangeBox();
        BRepBndLib.Add(aSp, ref aBox);
        aBox.SetGap(aBox.GetGap() + Precision.Confusion());
        //
        nSp = myDS.Append(aSI);
        return nSp;
    }

    /// <summary>
    /// Builds the p-curves of the edges in and on the faces, and of the section edges on their faces when the section
    /// attribute asks for them.
    /// </summary>
    /// <remarks>
    /// The p-curves are computed in parallel on copies of the edges (BOPAlgo_MPC) and stored afterwards, serially,
    /// with <c>UpdateEdge</c> into the DS edge. In non-destructive mode that edge is a new one wherever a p-curve is
    /// missing: an argument edge in or on another argument's face is in a common block, which MakeSplitEdges then
    /// always splits (decisions 577 and 719).
    /// </remarks>
    protected void MakePCurves(Message_ProgressRange theRange)
    {
        using var aPSOuter = new Message_ProgressScope(theRange, null, 1);
        if (myAvoidBuildPCurve || (!mySectionAttribute.PCurveOnS1() && !mySectionAttribute.PCurveOnS2()))
        {
            return;
        }
        bool bHasPC;
        int i, nF1, aNbC, k, nE, aNbFF, aNbFI, nEx;
        int j, aNbPBIn, aNbPBOn;
        NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new();
        TopoDS_Face aF1F;
        NCollection_DynamicArray<BOPAlgo_MPC> aVMPC = new();
        //
        // 1. Process Common Blocks
        NCollection_DynamicArray<BOPDS_FaceInfo> aFIP = myDS!.FaceInfoPool();
        //
        aNbFI = aFIP.Length();
        for (i = 0; i < aNbFI; ++i)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            BOPDS_FaceInfo aFI = aFIP[i];
            nF1 = aFI.Index();
            //
            aF1F = TopoDS.Face(myDS.Shape(nF1));
            aF1F.Orientation(TopAbs_FORWARD);
            // In
            NCollection_IndexedMap<BOPDS_PaveBlock> aMPBIn = aFI.PaveBlocksIn();
            aNbPBIn = aMPBIn.Extent();
            for (j = 1; j <= aNbPBIn; ++j)
            {
                BOPDS_PaveBlock aPB = aMPBIn[j];
                nE = aPB.Edge();
                // OCCT's Standard_ASSERT(nE >= 0, ..., continue): in release builds it only continues.
                if (nE < 0)
                {
                    continue;
                }
                TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
                //
                BOPAlgo_MPC aMPC = aVMPC.Append(new BOPAlgo_MPC());
                aMPC.SetEdge(aE);
                aMPC.SetFace(aF1F);
            }
            //
            // On
            NCollection_IndexedMap<BOPDS_PaveBlock> aMPBOn = aFI.PaveBlocksOn();
            aNbPBOn = aMPBOn.Extent();
            for (j = 1; j <= aNbPBOn; ++j)
            {
                BOPDS_PaveBlock aPB = aMPBOn[j];
                nE = aPB.Edge();
                TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
                bHasPC = BOPTools_AlgoTools2D.HasCurveOnSurface(aE, aF1F);
                if (bHasPC)
                {
                    continue;
                }
                //
                BOPAlgo_MPC aMPC = aVMPC.Append(new BOPAlgo_MPC());

                BOPDS_CommonBlock? aCB = myDS.CommonBlock(aPB);

                if (aCB is not null)
                {
                    NCollection_List<BOPDS_PaveBlock> aLPB = aCB.PaveBlocks();
                    if (aLPB.Extent() >= 2)
                    {
                        aItLPB.Initialize(aLPB);
                        for (; aItLPB.More(); aItLPB.Next())
                        {
                            BOPDS_PaveBlock aPBx = aItLPB.Value();
                            if (aPBx == aPB)
                            {
                                continue;
                            }
                            //
                            nEx = aPBx.OriginalEdge();
                            TopoDS_Edge aEx = TopoDS.Edge(myDS.Shape(nEx));
                            bHasPC = BOPTools_AlgoTools2D.HasCurveOnSurface(aEx, aF1F);
                            if (!bHasPC)
                            {
                                continue;
                            }
                            //
                            int nV1x, nV2x;
                            double aT1x, aT2x;
                            TopoDS_Vertex aV1x, aV2x;
                            TopoDS_Edge aEz;
                            //
                            aEz = aEx;
                            aEz.Orientation(TopAbs_FORWARD);
                            //
                            aPBx.Indices(out nV1x, out nV2x);
                            aPBx.Range(out aT1x, out aT2x);
                            //
                            aV1x = TopoDS.Vertex(myDS.Shape(nV1x));
                            aV1x.Orientation(TopAbs_FORWARD);
                            //
                            aV2x = TopoDS.Vertex(myDS.Shape(nV2x));
                            aV2x.Orientation(TopAbs_REVERSED);
                            //
                            aMPC.SetData(aEz, aV1x, aT1x, aV2x, aT2x);
                            //
                            break;
                        }
                    }
                }

                aMPC.SetEdge(aE);
                aMPC.SetFace(aF1F);
            }
        } // for (i=0; i<aNbFI; ++i) {
        //
        // 2. Process section edges. P-curves on them must already be computed.
        // However, we must provide the call to UpdateVertices.
        Span<bool> bPCurveOnS = stackalloc bool[2];
        bPCurveOnS[0] = mySectionAttribute.PCurveOnS1();
        bPCurveOnS[1] = mySectionAttribute.PCurveOnS2();
        //
        if (bPCurveOnS[0] || bPCurveOnS[1])
        {
            // container to remember already added edge-face pairs
            NCollection_Map<BOPDS_Pair> anEFPairs = new();
            NCollection_DynamicArray<BOPDS_InterfFF> aFFs = myDS.InterfFF();
            aNbFF = aFFs.Length();
            Span<int> nF = stackalloc int[2];
            TopoDS_Face[] aFf = new TopoDS_Face[2];
            for (i = 0; i < aNbFF; ++i)
            {
                BOPDS_InterfFF aFF = aFFs[i];
                NCollection_DynamicArray<BOPDS_Curve> aVNC = aFF.Curves();
                aNbC = aVNC.Length();
                if (aNbC == 0)
                {
                    continue;
                }
                aFF.Indices(out nF[0], out nF[1]);
                //
                aFf[0] = TopoDS.Face(myDS.Shape(nF[0]));
                aFf[0].Orientation(TopAbs_FORWARD);
                //
                aFf[1] = TopoDS.Face(myDS.Shape(nF[1]));
                aFf[1].Orientation(TopAbs_FORWARD);
                //
                for (k = 0; k < aNbC; ++k)
                {
                    BOPDS_Curve aNC = aVNC[k];
                    NCollection_List<BOPDS_PaveBlock> aLPB = aNC.PaveBlocks();
                    aItLPB.Initialize(aLPB);
                    for (; aItLPB.More(); aItLPB.Next())
                    {
                        BOPDS_PaveBlock aPB = aItLPB.Value();
                        nE = aPB.Edge();
                        TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
                        //
                        for (int m = 0; m < 2; ++m)
                        {
                            if (bPCurveOnS[m] && anEFPairs.Add(new BOPDS_Pair(nE, nF[m])))
                            {
                                BOPAlgo_MPC aMPC = aVMPC.Append(new BOPAlgo_MPC());
                                aMPC.SetEdge(aE);
                                aMPC.SetFace(aFf[m]);
                                aMPC.SetFlag(true);
                            }
                        }
                    }
                }
            } // for (i=0; i<aNbFF; ++i) {
        } // if (bPCurveOnS1 || bPCurveOnS2 ) {
        //
        using var aPS = new Message_ProgressScope(aPSOuter.Next(), "Projecting edges on faces", aVMPC.Length());
        for (i = 0; i < aVMPC.Length(); i++)
        {
            BOPAlgo_MPC aMPC = aVMPC[i];
            aMPC.SetProgressRange(aPS.Next());
        }
        //======================================================
        BOPTools_Parallel.Perform(myRunParallel, aVMPC, myContext!);
        //======================================================
        if (HasErrors())
        {
            return;
        }

        // Add warnings of the failed projections and update edges with new pcurves
        int aNb = aVMPC.Length();
        for (i = 0; i < aNb; ++i)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            BOPAlgo_MPC aMPC = aVMPC[i];
            if (aMPC.HasErrors())
            {
                TopoDS_Compound aWC = new();
                new BRep_Builder().MakeCompound(ref aWC);
                new BRep_Builder().Add(aWC, aMPC.Edge());
                new BRep_Builder().Add(aWC, aMPC.Face());
                AddWarning(new BOPAlgo_AlertBuildingPCurveFailed(aWC));
            }
            else
            {
                Geom2d_Curve? aNewPC = aMPC.GetNewPCurve();
                // if aNewPC is null we do not need to update the edge because it already contains
                // valid p-curve, and only vertices have been updated.
                if (aNewPC is not null)
                {
                    new BRep_Builder().UpdateEdge(aMPC.Edge(), aNewPC, aMPC.Face(), aMPC.GetNewTolerance());
                }
            }
        }
    }

    /// <summary>
    /// Updates the tolerances of the edge's vertices comparing the extremities of its 3D curve and of its p-curve on
    /// the face (OCCT's file-static <c>UpdateVertices</c>; writes into the vertices in place).
    /// </summary>
    internal static void UpdateVertices(in TopoDS_Edge aE, in TopoDS_Face aF)
    {
        int j;
        double aUx, aVx, aTolV2, aD2, aD;
        Span<double> aT = stackalloc double[2];
        gp_Pnt aP3D, aP3Dx;
        gp_Pnt2d aP2Dx;
        Geom_Surface aS;
        Geom_Curve aC3D;
        Geom2d_Curve aC2D;
        TopoDS_Edge aEf;
        TopoDS_Vertex[] aV = new TopoDS_Vertex[2];
        BRep_Builder aBB = new();
        //
        aEf = aE;
        aEf.Orientation(TopAbs_FORWARD);
        //
        TopExp.Vertices(aEf, out aV[0], out aV[1]);
        //
        aS = BRep_Tool.Surface(aF)!;
        aC3D = BRep_Tool.Curve(aEf, out aT[0], out aT[1])!;
        aC2D = BRep_Tool.CurveOnSurface(aEf, aF, out aT[0], out aT[1])!;
        //
        for (j = 0; j < 2; ++j)
        {
            aTolV2 = BRep_Tool.Tolerance(aV[j]);
            aTolV2 = aTolV2 * aTolV2;
            //
            aC3D.D0(aT[j], out aP3D);
            aC2D.D0(aT[j], out aP2Dx);
            aP2Dx.Coord(out aUx, out aVx);
            aS.D0(aUx, aVx, out aP3Dx);
            aD2 = aP3D.SquareDistance(aP3Dx);
            if (aD2 > aTolV2)
            {
                aD = Math.Sqrt(aD2);
                aBB.UpdateVertex(aV[j], aD + BOPTools_AlgoTools.DTolerance());
            }
        }
    }

    /// <summary>
    /// Builds the p-curves of the argument edges on the planar faces that interfere (destructive mode only: the
    /// p-curves are stored in the argument edges).
    /// </summary>
    protected void Prepare(Message_ProgressRange theRange)
    {
        if (myNonDestructive)
        {
            // do not allow storing pcurves in original edges if non-destructive mode is on
            return;
        }
        Span<TopAbs_ShapeEnum> aType = [TopAbs_VERTEX, TopAbs_EDGE, TopAbs_FACE];
        bool bIsBasedOnPlane;
        int i, aNb, n1, nF, aNbF;
        TopExp_Explorer aExp = new();
        NCollection_IndexedMap<TopoDS_Shape> aMF = new(TopTools_ShapeMapHasher.Instance);
        //
        aNb = 3;
        using var aPSOuter = new Message_ProgressScope(theRange, null, 1);
        for (i = 0; i < aNb; ++i)
        {
            myIterator!.Initialize(aType[i], aType[2]);
            for (; myIterator.More(); myIterator.Next())
            {
                myIterator.Value(out n1, out nF);
                TopoDS_Face aF = TopoDS.Face(myDS!.Shape(nF));
                //
                bIsBasedOnPlane = IsBasedOnPlane(aF);
                if (bIsBasedOnPlane)
                {
                    aMF.Add(aF);
                }
            }
        }
        //
        aNbF = aMF.Extent();
        if (aNbF == 0)
        {
            return;
        }
        //
        // Build pcurves of edges on planes; first collect pairs edge-face.
        NCollection_DynamicArray<BOPAlgo_BPC> aVBPC = new();
        //
        for (i = 1; i <= aNbF; ++i)
        {
            TopoDS_Face aF = TopoDS.Face(aMF[i]);
            aExp.Init(aF, aType[1]);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
                BOPAlgo_BPC aBPC = aVBPC.Append(new BOPAlgo_BPC());
                aBPC.SetEdge(aE);
                aBPC.SetFace(aF);
            }
        }
        //
        using var aPS = new Message_ProgressScope(aPSOuter.Next(), "Building 2d curves on planar faces", aVBPC.Length());
        for (i = 0; i < aVBPC.Length(); i++)
        {
            BOPAlgo_BPC aBPC = aVBPC[i];
            aBPC.SetRange(aPS.Next());
        }
        //======================================================
        BOPTools_Parallel.Perform(myRunParallel, aVBPC);
        //======================================================
        if (UserBreak(aPS))
        {
            return;
        }
        // pcurves are built, and now update edges
        BRep_Builder aBB = new();
        for (i = 0; i < aVBPC.Length(); i++)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            BOPAlgo_BPC aBPC = aVBPC[i];
            if (aBPC.IsToUpdate())
            {
                double aTolE = BRep_Tool.Tolerance(aBPC.GetEdge());
                aBB.UpdateEdge(aBPC.GetEdge(), aBPC.GetCurve2d(), aBPC.GetFace(), aTolE);
            }
        }
    }

    // Whether the face's surface is a plane (or a trimmed plane).
    private static bool IsBasedOnPlane(in TopoDS_Face aF)
    {
        Geom_RectangularTrimmedSurface? aGRTS;
        Geom_Plane? aGP;

        Geom_Surface? aS = BRep_Tool.Surface(aF, out TopLoc_Location _);
        aGRTS = aS as Geom_RectangularTrimmedSurface;
        if (aGRTS is not null)
        {
            aGP = aGRTS.BasisSurface() as Geom_Plane;
        }
        else
        {
            aGP = aS as Geom_Plane;
        }
        //
        return aGP is not null;
    }
}
