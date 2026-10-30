// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_5.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BOPAlgo_GlueEnum;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The intersection of a pave block of an edge with a face, as a parallel solver of the pave filler.</summary>
/// <remarks>
/// OCCT's class derives from both IntTools_EdgeFace and BOPAlgo_ParallelAlgo; here it derives from the first and holds
/// what the second gives (the progress range and a report, through a <see cref="BOPAlgo_Options"/>). Its
/// <see cref="Perform"/> hides the base's: when the shapes are far from the origin it moves the edge and the face near
/// it (<see cref="BOPAlgo_Tools.TrsfToPoint(in Bnd_Box, in Bnd_Box, ref gp_Trsf)"/>), intersects, restores them and
/// gives the common parts back the unmoved edge. A moved face is a different key of the context (decision 756).
/// </remarks>
internal sealed class BOPAlgo_EdgeFace : IntTools_EdgeFace, BOPTools_Parallel.IContextSolver<IntTools_Context>
{
    private readonly BOPAlgo_Options myOptions = new();
    private Message_ProgressRange myProgressRange;
    private int myIE;
    private int myIF;
    private IntTools_Range myNewSR = new();
    private BOPDS_PaveBlock? myPB;
    private Bnd_Box myBox1 = new();
    private Bnd_Box myBox2 = new();

    public BOPAlgo_EdgeFace()
    {
        myIE = -1;
        myIF = -1;
    }

    public void SetIndices(int nE, int nF)
    {
        myIE = nE;
        myIF = nF;
    }

    public void Indices(out int nE, out int nF)
    {
        nE = myIE;
        nF = myIF;
    }

    public void SetNewSR(in IntTools_Range aR) => myNewSR = aR;

    public IntTools_Range NewSR() => myNewSR;

    public void SetPaveBlock(BOPDS_PaveBlock aPB) => myPB = aPB;

    public BOPDS_PaveBlock PaveBlock() => myPB!;

    public void SetBoxes(in Bnd_Box theBox1, in Bnd_Box theBox2)
    {
        myBox1 = theBox1;
        myBox2 = theBox2;
    }

    /// <summary>BOPAlgo_ParallelAlgo's range for the single run.</summary>
    public void SetProgressRange(Message_ProgressRange theRange) => myProgressRange = theRange;

    /// <summary>Whether the run failed (BOPAlgo_Options' errors).</summary>
    public bool HasErrors() => myOptions.HasErrors();

    void BOPTools_Parallel.IContextSolver<IntTools_Context>.SetContext(IntTools_Context theContext) => SetContext(theContext);

    public new void Perform()
    {
        using var aPS = new Message_ProgressScope(myProgressRange, null, 1);
        if (myOptions.UserBreak(aPS))
        {
            return;
        }
        TopoDS_Face aFace = myFace;
        TopoDS_Edge anEdge = myEdge;
        bool hasTrsf = false;
        try
        {
            gp_Trsf aTrsf = new();
            if (BOPAlgo_Tools.TrsfToPoint(myBox1, myBox2, ref aTrsf))
            {
                // Shapes are located far from origin, move the shapes to the origin,
                // to increase the accuracy of intersection.
                TopLoc_Location aLoc = new(aTrsf);
                myEdge.Move(aLoc);
                myFace.Move(aLoc);
                hasTrsf = true;
            }

            base.Perform();
        }
        catch (Standard_Failure)
        {
            myOptions.AddError(new BOPAlgo_AlertIntersectionFailed());
        }
        myFace = aFace;
        myEdge = anEdge;

        if (hasTrsf)
        {
            for (int i = 1; i <= mySeqOfCommonPrts.Length(); ++i)
            {
                IntTools_CommonPrt aCPart = mySeqOfCommonPrts[i];
                aCPart.SetEdge1(myEdge);
            }
        }
    }
}

public partial class BOPAlgo_PaveFiller
{
    /// <summary>The edge/face interferences.</summary>
    protected virtual void PerformEF(Message_ProgressRange theRange)
    {
        FillShrunkData(TopAbs_EDGE, TopAbs_FACE);
        //
        myIterator!.Initialize(TopAbs_EDGE, TopAbs_FACE);
        using var aPSOuter = new Message_ProgressScope(theRange, null, 10);
        int iSize = myIterator.ExpectedLength();
        if (iSize == 0)
        {
            return;
        }
        //
        int nE, nF;
        //
        if (myGlue == BOPAlgo_GlueFull)
        {
            // there is no need to intersect edges with faces in this mode
            // just initialize FaceInfo for faces
            for (; myIterator.More(); myIterator.Next())
            {
                myIterator.Value(out nE, out nF);
                if (!myDS!.ShapeInfo(nE).HasFlag())
                {
                    myDS.ChangeFaceInfo(nF);
                }
            }
            return;
        }
        //
        Span<bool> bV = stackalloc bool[2];
        Span<bool> bIsOnPave = stackalloc bool[2];
        bool bIsPBSplittable;
        bool bV1, bV2, bExpressCompute;
        int nV1, nV2;
        int i, aNbCPrts, iX;
        Span<int> nV = stackalloc int[2];
        int aNbEdgeFace, k;
        double aTolE, aTolF, aTS1 = 0.0, aTS2 = 0.0, aT1 = 0.0, aT2 = 0.0;
        TopAbs_ShapeEnum aType;
        NCollection_List<BOPDS_PaveBlock>.Iterator aIt = new();
        NCollection_DynamicArray<BOPAlgo_EdgeFace> aVEdgeFace = new();
        //-----------------------------------------------------scope f
        //
        NCollection_IntMap aMIEFC = new(100);
        NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> aMVCPB = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedDataMap<BOPDS_PaveBlock, NCollection_List<int>> aMPBLI = new();
        NCollection_DataMap<BOPDS_PaveBlock, Bnd_Box> aDMPBBox = new();
        //
        NCollection_DynamicArray<BOPDS_InterfEF> aEFs = myDS!.InterfEF();
        //
        for (; myIterator.More(); myIterator.Next())
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            myIterator.Value(out nE, out nF);
            //
            BOPDS_ShapeInfo aSIE = myDS.ShapeInfo(nE);
            if (aSIE.HasFlag())
            { // degenerated
                continue;
            }
            //
            TopoDS_Edge aE = TopoDS.Edge(aSIE.Shape());
            TopoDS_Face aF = TopoDS.Face(myDS.Shape(nF));
            Bnd_Box aBBF = myDS.ShapeInfo(nF).Box();
            //
            BOPDS_FaceInfo aFI = myDS.ChangeFaceInfo(nF);
            NCollection_IndexedMap<BOPDS_PaveBlock> aMPBF = aFI.PaveBlocksOn();
            //
            NCollection_IntMap aMVIn = aFI.VerticesIn();
            NCollection_IntMap aMVOn = aFI.VerticesOn();
            //
            aTolE = BRep_Tool.Tolerance(aE);
            aTolF = BRep_Tool.Tolerance(aF);
            //
            NCollection_List<BOPDS_PaveBlock> aLPB = myDS.ChangePaveBlocks(nE);
            aIt.Initialize(aLPB);
            for (; aIt.More(); aIt.Next())
            {
                if (UserBreak(aPSOuter))
                {
                    return;
                }
                BOPDS_PaveBlock aPB = aIt.Value();
                //
                BOPDS_PaveBlock aPBR = myDS.RealPaveBlock(aPB);
                if (aMPBF.Contains(aPBR))
                {
                    continue;
                }
                //
                Bnd_Box aBBE = new();
                if (!GetPBBox(aE, aPB, aDMPBBox, ref aT1, ref aT2, ref aTS1, ref aTS2, ref aBBE))
                {
                    continue;
                }
                //
                if (aBBF.IsOut(aBBE))
                {
                    continue;
                }
                //
                aPBR.Indices(out nV1, out nV2);
                bV1 = aMVIn.Contains(nV1) || aMVOn.Contains(nV1);
                bV2 = aMVIn.Contains(nV2) || aMVOn.Contains(nV2);
                bExpressCompute = bV1 && bV2;
                //
                BOPAlgo_EdgeFace aEdgeFace = aVEdgeFace.Append(new BOPAlgo_EdgeFace());
                //
                aEdgeFace.SetIndices(nE, nF);
                aEdgeFace.SetPaveBlock(aPB);
                //
                aEdgeFace.SetEdge(aE);
                aEdgeFace.SetFace(aF);
                aEdgeFace.SetBoxes(myDS.ShapeInfo(nE).Box(), myDS.ShapeInfo(nF).Box());
                aEdgeFace.SetFuzzyValue(myFuzzyValue);
                aEdgeFace.UseQuickCoincidenceCheck(bExpressCompute);

                IntTools_Range aSR = new(aTS1, aTS2);
                BOPTools_AlgoTools.CorrectRange(aE, aF, aSR, out IntTools_Range anewSR);
                aEdgeFace.SetNewSR(anewSR);
                //
                IntTools_Range aPBRange = new(aT1, aT2);
                aSR = aPBRange;
                BOPTools_AlgoTools.CorrectRange(aE, aF, aSR, out aPBRange);
                aEdgeFace.SetRange(aPBRange);
                //
                // Save the pair to avoid their forced intersection
                if (!myFPBDone.Find(nF, out NCollection_Map<BOPDS_PaveBlock> pMPB))
                {
                    pMPB = new NCollection_Map<BOPDS_PaveBlock>();
                    myFPBDone.Bind(nF, pMPB);
                }
                pMPB.Add(aPB);
            } // for (; aIt.More(); aIt.Next()) {
        } // for (; myIterator->More(); myIterator->Next()) {
        //
        aNbEdgeFace = aVEdgeFace.Length();
        using var aPS = new Message_ProgressScope(aPSOuter.Next(9), "Performing Edge-Face intersection", aNbEdgeFace);
        for (int index = 0; index < aNbEdgeFace; index++)
        {
            BOPAlgo_EdgeFace aEdgeFace = aVEdgeFace[index];
            aEdgeFace.SetProgressRange(aPS.Next());
        }
        //=================================================================
        BOPTools_Parallel.Perform(myRunParallel, aVEdgeFace, myContext!);
        //=================================================================
        if (UserBreak(aPSOuter))
        {
            return;
        }
        //
        for (k = 0; k < aNbEdgeFace; ++k)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            BOPAlgo_EdgeFace aEdgeFace = aVEdgeFace[k];
            if (!aEdgeFace.IsDone() || aEdgeFace.HasErrors())
            {
                // Warn about failed intersection of sub-shapes
                AddIntersectionFailedWarning(aEdgeFace.Edge(), aEdgeFace.Face());
                continue;
            }
            //
            aEdgeFace.Indices(out nE, out nF);
            //
            TopoDS_Edge aE = aEdgeFace.Edge();
            TopoDS_Face aF = aEdgeFace.Face();
            //
            aTolE = BRep_Tool.Tolerance(aE);
            aTolF = BRep_Tool.Tolerance(aF);
            //
            NCollection_Sequence<IntTools_CommonPrt> aCPrts = aEdgeFace.CommonParts();
            aNbCPrts = aCPrts.Length();
            if (aNbCPrts == 0)
            {
                // The RealLast() test relies on MinimalDistance() staying RealLast on the paths that don't compute it
                // (decision 756).
                if (aEdgeFace.MinimalDistance() < RealLast() && aEdgeFace.MinimalDistance() > aTolE + aTolF)
                {
                    BOPDS_PaveBlock aPB0 = aEdgeFace.PaveBlock();
                    aPB0.Range(out aT1, out aT2);
                    BOPDS_Pair aPair = new(nE, nF);
                    if (!myDistances.Find(aPair, out NCollection_List<EdgeRangeDistance> pList))
                    {
                        pList = new NCollection_List<EdgeRangeDistance>();
                        myDistances.Bind(aPair, pList);
                    }
                    pList.Append(new EdgeRangeDistance(aT1, aT2, aEdgeFace.MinimalDistance()));
                }
                continue;
            }
            //
            IntTools_Range anewSR = aEdgeFace.NewSR();
            BOPDS_PaveBlock aPB = aEdgeFace.PaveBlock();
            //
            aPB.Range(out aT1, out aT2);
            aPB.Indices(out nV[0], out nV[1]);
            bIsPBSplittable = aPB.IsSplittable();
            //
            anewSR.Range(out aTS1, out aTS2);
            //
            if (aCPrts[1].Type() == TopAbs_VERTEX)
            {
                // for the intersection type VERTEX
                // extend vertices ranges using Edge/Edge intersections
                // between the edge aE and the edges of the face aF.
                // thereby the edge's intersection range is reduced
                ReduceIntersectionRange(nV[0], nV[1], nE, nF, ref aTS1, ref aTS2);
            }
            //
            IntTools_Range aR1 = new(aT1, aTS1), aR2 = new(aTS2, aT2);
            //
            BOPDS_FaceInfo aFI = myDS.ChangeFaceInfo(nF);
            NCollection_IntMap aMIFOn = aFI.VerticesOn();
            NCollection_IntMap aMIFIn = aFI.VerticesIn();
            //
            bool bLinePlane = false;
            if (aNbCPrts != 0)
            {
                BRepAdaptor_Curve aBAC = new(aE);
                bLinePlane = (aBAC.GetType() == GeomAbs_Line && myContext!.SurfaceAdaptor(aF).GetType() == GeomAbs_Plane);
            }
            //
            for (i = 1; i <= aNbCPrts; ++i)
            {
                if (UserBreak(aPSOuter))
                {
                    return;
                }
                IntTools_CommonPrt aCPart = aCPrts[i];
                aType = aCPart.Type();
                switch (aType)
                {
                    case TopAbs_VERTEX:
                    {
                        int j;
                        double aT, aTolToDecide;
                        TopoDS_Vertex aVnew;
                        //
                        IntTools_Tools.VertexParameter(aCPart, out aT);
                        BOPTools_AlgoTools.MakeNewVertex(aE, aT, aF, out aVnew);
                        //
                        IntTools_Range aR = aCPart.Range1();
                        aTolToDecide = 5.0e-8;
                        //
                        bIsOnPave[0] = IntTools_Tools.IsInRange(aR1, aR, aTolToDecide);
                        bIsOnPave[1] = IntTools_Tools.IsInRange(aR2, aR, aTolToDecide);
                        //
                        if ((bIsOnPave[0] && bIsOnPave[1]) || (bLinePlane && (bIsOnPave[0] || bIsOnPave[1])))
                        {
                            bV[0] = CheckFacePaves(nV[0], aMIFOn, aMIFIn);
                            bV[1] = CheckFacePaves(nV[1], aMIFOn, aMIFIn);
                            if (bV[0] && bV[1])
                            {
                                IntTools_CommonPrt aCP = new(aCPart);
                                aCP.SetType(TopAbs_EDGE);
                                BOPDS_InterfEF aEF0 = aEFs.Append(new BOPDS_InterfEF());
                                iX = aEFs.Length() - 1;
                                aEF0.SetIndices(nE, nF);
                                aEF0.SetCommonPart(aCP);
                                myDS.AddInterf(nE, nF);
                                //
                                aMIEFC.Add(nF);
                                //
                                BOPAlgo_Tools.FillMap(aPB, nF, aMPBLI);
                                break;
                            }
                        }
                        //
                        if (!bIsPBSplittable)
                        {
                            continue;
                        }
                        //
                        for (j = 0; j < 2; ++j)
                        {
                            if (bIsOnPave[j])
                            {
                                bV[j] = CheckFacePaves(nV[j], aMIFOn, aMIFIn);
                                if (!bV[j])
                                {
                                    bIsOnPave[j] = ForceInterfVF(nV[j], nF);
                                }
                            }
                        }

                        if (bIsOnPave[0] || bIsOnPave[1])
                        {
                            // The found intersection point is located closely to one of the pave block's
                            // bounds. So, do not create the new vertex in this point.
                            // Check if this point is a real intersection, or just a touching point.
                            // If it is a touching point, do nothing.
                            // If it is an intersection point, update the existing vertex to cover the
                            // intersection point.
                            GeomAPI_ProjectPointOnSurf aProjPS = myContext!.ProjPS(aF);
                            gp_Pnt aPnew0 = BRep_Tool.Pnt(aVnew);
                            aProjPS.Perform(aPnew0);
                            double aMinDistEF = (aProjPS.IsDone() && aProjPS.NbPoints() != 0) ? aProjPS.LowerDistance()
                                                                                             : Precision.Infinite();
                            bool hasRealIntersection = aMinDistEF < Precision.Intersection();

                            if (!hasRealIntersection)
                            {
                                // no intersection point
                                continue;
                            }

                            // Real intersection is present.
                            // Update the existing vertex to cover the intersection point.
                            for (j = 0; j < 2; ++j)
                            {
                                if (bIsOnPave[j])
                                {
                                    TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(nV[j]));
                                    gp_Pnt aP = BRep_Tool.Pnt(aV);
                                    double aDistPP = aP.Distance(aPnew0);
                                    double aTol = BRep_Tool.Tolerance(aV);
                                    double aMaxDist = 1.0e4 * aTol;
                                    if (aTol < .01)
                                    {
                                        aMaxDist = Min(aMaxDist, 0.1);
                                    }
                                    if (aDistPP < aMaxDist)
                                    {
                                        UpdateVertex(nV[j], aDistPP);
                                        myVertsToAvoidExtension.Add(nV[j]);
                                    }
                                }
                            }
                            continue;
                        }

                        if (CheckFacePaves(aVnew, aMIFOn))
                        {
                            continue;
                        }
                        //
                        double aTolVnew = BRep_Tool.Tolerance(aVnew);
                        aTolVnew = Max(aTolVnew, Max(aTolE, aTolF));
                        new BRep_Builder().UpdateVertex(aVnew, aTolVnew);
                        if (bLinePlane)
                        {
                            // increase tolerance for Line/Plane intersection, but do not update
                            // the vertex till its intersection with some other shape
                            IntTools_Range aCR = aCPart.Range1();
                            aTolVnew = Max(aTolVnew, (aCR.Last() - aCR.First()) / 2.0);
                        }
                        //
                        gp_Pnt aPnew = BRep_Tool.Pnt(aVnew);
                        //
                        if (!myContext!.IsPointInFace(aPnew, aF, aTolVnew))
                        {
                            continue;
                        }
                        //
                        aMIEFC.Add(nF);
                        // 1
                        BOPDS_InterfEF aEF = aEFs.Append(new BOPDS_InterfEF());
                        iX = aEFs.Length() - 1;
                        aEF.SetIndices(nE, nF);
                        aEF.SetCommonPart(aCPart);
                        // 2
                        myDS.AddInterf(nE, nF);
                        // 3
                        BOPDS_CoupleOfPaveBlocks aCPB = new();
                        //
                        aCPB.SetPaveBlocks(aPB, aPB);
                        aCPB.SetIndexInterf(iX);
                        aCPB.SetTolerance(aTolVnew);
                        aMVCPB.Add(aVnew, aCPB);
                    }
                    break;
                    case TopAbs_EDGE:
                    {
                        aMIEFC.Add(nF);
                        //
                        // 1
                        BOPDS_InterfEF aEF = aEFs.Append(new BOPDS_InterfEF());
                        iX = aEFs.Length() - 1;
                        aEF.SetIndices(nE, nF);
                        //
                        bV[0] = CheckFacePaves(nV[0], aMIFOn, aMIFIn);
                        bV[1] = CheckFacePaves(nV[1], aMIFOn, aMIFIn);
                        if (!bV[0] || !bV[1])
                        {
                            myDS.AddInterf(nE, nF);
                            break;
                        }
                        aEF.SetCommonPart(aCPart);
                        // 2
                        myDS.AddInterf(nE, nF);
                        // 3
                        BOPAlgo_Tools.FillMap(aPB, nF, aMPBLI);
                    }
                    break;
                    default:
                        break;
                } // switch (aType) {
            } // for (i=1; i<=aNbCPrts; ++i) {
        } // for (k=0; k < aNbEdgeEdge; ++k) {
        //
        //=========================================
        // post treatment
        //=========================================
        BOPAlgo_Tools.PerformCommonBlocks(aMPBLI, myDS, myContext);
        UpdateVerticesOfCB();
        PerformNewVertices(aMVCPB, aPSOuter.Next(1), false);
        if (HasErrors())
        {
            return;
        }
        //
        // Update FaceInfoIn for all faces having EF common parts
        myDS.UpdateFaceInfoIn(aMIEFC);

        //-----------------------------------------------------scope t
        aMIEFC.Clear();
        aMVCPB.Clear();
        aMPBLI.Clear();
    }

    /// <summary>Whether the vertex is on or in the face (its index in one of the two maps).</summary>
    protected static bool CheckFacePaves(int nVx, NCollection_IntMap aMIFOn, NCollection_IntMap aMIFIn)
    {
        return aMIFOn.Contains(nVx) || aMIFIn.Contains(nVx);
    }

    /// <summary>Whether the new vertex interferes with one of the vertices of the map.</summary>
    protected bool CheckFacePaves(in TopoDS_Vertex aVnew, NCollection_IntMap aMIF)
    {
        bool bRet;
        int iFlag;
        //
        bRet = true;
        //
        // OCCT iterates the map in its hash order (decision 6901); the result is whether any vertex interferes.
        foreach (int nV in aMIF)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
            iFlag = BOPTools_AlgoTools.ComputeVV(aVnew, aV);
            if (iFlag == 0)
            {
                return bRet;
            }
        }
        //
        return !bRet;
    }

    /// <summary>Forces the vertex/face interference, adding it if the vertex is on the face.</summary>
    protected bool ForceInterfVF(int nV, int nF)
    {
        bool bRet;
        int iFlag, nVx;
        double U = 0.0, V = 0.0, aTolVNew = 0.0;
        //
        bRet = false;
        TopoDS_Vertex aV = TopoDS.Vertex(myDS!.Shape(nV));
        TopoDS_Face aF = TopoDS.Face(myDS.Shape(nF));
        //
        iFlag = myContext!.ComputeVF(aV, aF, ref U, ref V, ref aTolVNew, myFuzzyValue);
        if (iFlag == 0 || iFlag == -2)
        {
            bRet = !bRet;
            //
            NCollection_DynamicArray<BOPDS_InterfVF> aVFs = myDS.InterfVF();
            // 1
            BOPDS_InterfVF aVF = aVFs.Append(new BOPDS_InterfVF());
            //
            aVF.SetIndices(nV, nF);
            aVF.SetUV(U, V);
            // 2
            myDS.AddInterf(nV, nF);
            //
            // 3 update vertex V/F if necessary
            nVx = UpdateVertex(nV, aTolVNew);
            // 4
            if (myDS.IsNewShape(nVx))
            {
                aVF.SetIndexNew(nVx);
            }
            //
            BOPDS_FaceInfo aFI = myDS.ChangeFaceInfo(nF);
            NCollection_IntMap aMVIn = aFI.ChangeVerticesIn();
            aMVIn.Add(nVx);
            //
            // check for self-interference
            int iRV = myDS.Rank(nV);
            if (iRV >= 0 && iRV == myDS.Rank(nF))
            {
                // add warning status
                TopoDS_Compound aWC = new();
                new BRep_Builder().MakeCompound(ref aWC);
                new BRep_Builder().Add(aWC, aV);
                new BRep_Builder().Add(aWC, aF);
                AddWarning(new BOPAlgo_AlertSelfInterferingShape(aWC));
            }
        }
        return bRet;
    }

    /// <summary>
    /// Reduces the intersection range of the edge by the edge/edge intersections of the edge with the face's edges
    /// that made its new vertices.
    /// </summary>
    protected void ReduceIntersectionRange(int theV1, int theV2, int theE, int theF, ref double theTS1, ref double theTS2)
    {
        if (!myDS!.IsNewShape(theV1) && !myDS.IsNewShape(theV2))
        {
            return;
        }
        //
        if (!myDS.HasInterfShapeSubShapes(theE, theF))
        {
            return;
        }
        //
        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = myDS.InterfEE();
        int aNbEEs = aEEs.Length();
        if (aNbEEs == 0)
        {
            return;
        }
        //
        int i, nV, nE1, nE2;
        double aTR1, aTR2;
        //
        // get face's edges to check that E/E contains the edge from the face
        NCollection_IntMap aMFE = new();
        NCollection_List<int> aLI = myDS.ShapeInfo(theF).SubShapes();
        NCollection_List<int>.Iterator aItLI = new(aLI);
        for (; aItLI.More(); aItLI.Next())
        {
            nE1 = aItLI.Value();
            if (myDS.ShapeInfo(nE1).ShapeType() == TopAbs_EDGE)
            {
                aMFE.Add(nE1);
            }
        }
        //
        for (i = 0; i < aNbEEs; ++i)
        {
            BOPDS_InterfEE aEE = aEEs[i];
            if (!aEE.HasIndexNew())
            {
                continue;
            }
            //
            // check the vertex
            nV = aEE.IndexNew();
            if (nV != theV1 && nV != theV2)
            {
                continue;
            }
            //
            // check that the intersection is between the edge
            // and one of the face's edge
            aEE.Indices(out nE1, out nE2);
            if (((theE != nE1) && (theE != nE2)) || (!aMFE.Contains(nE1) && !aMFE.Contains(nE2)))
            {
                continue;
            }
            //
            // update the intersection range
            IntTools_CommonPrt aCPart = aEE.CommonPart();
            IntTools_Range aCRange = (theE == nE1) ? aCPart.Range1() : aCPart.Ranges2().First();
            aCRange.Range(out aTR1, out aTR2);
            //
            if (nV == theV1)
            {
                if (theTS1 < aTR2)
                {
                    theTS1 = aTR2;
                }
            }
            else
            {
                if (theTS2 > aTR1)
                {
                    theTS2 = aTR1;
                }
            }
        }
    }

    /// <summary>
    /// Looks for the edge/face common blocks among the pairs of pave blocks and faces with the same vertices, after
    /// the vertices have been increased and unified.
    /// </summary>
    protected void ForceInterfEF(Message_ProgressRange theRange)
    {
        using var aPS = new Message_ProgressScope(theRange, null, 1);
        if (!myIsPrimary)
        {
            return;
        }

        // Now that we have vertices increased and unified, try to find additional
        // edge/face common blocks among the pairs of edge/face.
        // Here, we are interested in common blocks only, as all real intersections
        // should have happened already. Thus, we need to check only those pairs
        // of edge/face which have the same vertices.

        // Collect all pave blocks
        NCollection_IndexedMap<BOPDS_PaveBlock> aMPB = new();
        int aNbS = myDS!.NbSourceShapes();
        for (int nE = 0; nE < aNbS; ++nE)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(nE);
            if (aSI.ShapeType() != TopAbs_EDGE)
            {
                // Not an edge
                continue;
            }

            if (!aSI.HasReference())
            {
                // Edge has no pave blocks
                continue;
            }

            if (aSI.HasFlag())
            {
                // Degenerated edge
                continue;
            }

            if (UserBreak(aPS))
            {
                return;
            }
            NCollection_List<BOPDS_PaveBlock> aLPB = myDS.PaveBlocks(nE);
            NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPB);
            for (; aItLPB.More(); aItLPB.Next())
            {
                BOPDS_PaveBlock aPB = aItLPB.Value();
                BOPDS_PaveBlock aPBR = myDS.RealPaveBlock(aPB);
                aMPB.Add(aPBR);
            }
        }

        // Perform intersection of collected pave blocks with faces

        ForceInterfEF(aMPB, aPS.Next(), true);
    }

    /// <summary>
    /// Intersects the pave blocks with the faces containing both their vertices, looking for edge/face common blocks
    /// only; <paramref name="theAddInterf"/> adds the interferences and makes the common blocks.
    /// </summary>
    protected void ForceInterfEF(NCollection_IndexedMap<BOPDS_PaveBlock> theMPB, Message_ProgressRange theRange, bool theAddInterf)
    {
        // Split progress on preparation, intersection and post-treatment stages
        using var aPSOuter = new Message_ProgressScope(theRange, null, 10);
        if (theMPB.IsEmpty())
        {
            return;
        }
        // Fill the tree with bounding boxes of the pave blocks
        BOPTools_BoxTree aBBTree = new();

        NCollection_IndexedMap<BOPDS_PaveBlock> aPBMap = new();

        int aNbPB = theMPB.Extent();
        for (int iPB = 1; iPB <= aNbPB; ++iPB)
        {
            BOPDS_PaveBlock aPB = theMPB[iPB];
            if (!aPB.HasShrunkData() || !myDS!.IsValidShrunkData(aPB))
            {
                FillShrunkData(aPB);
                if (!aPB.HasShrunkData())
                {
                    continue;
                }
            }
            if (UserBreak(aPSOuter))
            {
                return;
            }

            aPB.ShrunkData(out double f, out double l, out Bnd_Box aPBBox, out bool isSplit);

            aBBTree.Add(aPBMap.Add(aPB), Bnd_Tools.Bnd2BVH(aPBBox));
        }

        // Shake the tree
        aBBTree.Build();

        bool bSICheckMode = (myArguments.Extent() == 1);

        // Find pairs of Face/PaveBlock containing the same vertices
        // and prepare those pairs for intersection.
        NCollection_DynamicArray<BOPAlgo_EdgeFace> aVEdgeFace = new();

        Span<double> aTS = stackalloc double[2];
        int aNbS = myDS!.NbSourceShapes();
        for (int nF = 0; nF < aNbS; ++nF)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(nF);
            if (aSI.ShapeType() != TopAbs_FACE)
            {
                // Not a face
                continue;
            }

            if (!aSI.HasReference())
            {
                // Face has no face info
                continue;
            }

            if (UserBreak(aPSOuter))
            {
                return;
            }

            Bnd_Box aBoxF = aSI.Box();
            BOPTools_BoxTreeSelector aSelector = new();
            aSelector.SetBox(Bnd_Tools.Bnd2BVH(aBoxF));
            aSelector.SetBVHSet(aBBTree);
            if (aSelector.Select() == 0)
            {
                continue;
            }

            TopoDS_Face aF = TopoDS.Face(aSI.Shape());
            BOPDS_FaceInfo aFI = myDS.FaceInfo(nF);
            // Vertices of the face
            NCollection_IntMap aMVF = new();
            NCollection_IntMap[] pMVF = [aFI.VerticesOn(), aFI.VerticesIn(), aFI.VerticesSc()];
            for (int iM = 0; iM < 3; ++iM)
            {
                foreach (int aVIndex in pMVF[iM])
                {
                    aMVF.Add(aVIndex);
                }
            }

            // Pave Blocks of the face
            NCollection_IndexedMap<BOPDS_PaveBlock>[] pMPBF = [aFI.PaveBlocksOn(), aFI.PaveBlocksIn(), aFI.PaveBlocksSc()];
            for (int iM = 0; iM < 3; ++iM)
            {
                int aNb = pMPBF[iM].Extent();
                for (int iPB = 1; iPB <= aNb; ++iPB)
                {
                    BOPDS_PaveBlock aPB = pMPBF[iM].FindKey(iPB);
                    aMVF.Add(aPB.Pave1().Index());
                    aMVF.Add(aPB.Pave2().Index());
                }
            }

            // Projection tool
            GeomAPI_ProjectPointOnSurf aProjPS = myContext!.ProjPS(aF);
            BRepAdaptor_Surface aSurfAdaptor = myContext.SurfaceAdaptor(aF);

            // Iterate on pave blocks and combine pairs containing
            // the same vertices
            NCollection_List<int> aLIPB = aSelector.Indices();
            NCollection_List<int>.Iterator itLIPB = new(aLIPB);
            for (; itLIPB.More(); itLIPB.Next())
            {
                BOPDS_PaveBlock aPB = aPBMap[itLIPB.Value()];
                if (pMPBF[0].Contains(aPB) || pMPBF[1].Contains(aPB) || pMPBF[2].Contains(aPB))
                {
                    continue;
                }

                // Check if the face contains both vertices of the pave block
                aPB.Indices(out int nV1, out int nV2);
                if (!aMVF.Contains(nV1) || !aMVF.Contains(nV2))
                {
                    // Face does not contain the vertices
                    continue;
                }

                // Get the edge
                if (!aPB.HasEdge(out int nE))
                {
                    nE = aPB.OriginalEdge();
                    if (nE < 0)
                    {
                        continue;
                    }

                    // Make sure that the edge and face came from different arguments
                    if (myDS.Rank(nF) == myDS.Rank(nE))
                    {
                        continue;
                    }
                }

                TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
                BRepAdaptor_Curve aBAC = new(aE);

                // Check directions coincidence at middle point on the edge
                // and projection of that point on the face.
                // If the angle between tangent vector to the curve and normal
                // of the face is not in the range of 65 - 115 degrees, do not use the additional
                // tolerance, as it may lead to undesired unification of edge with the face.
                bool bUseAddTol = true;

                aPB.ShrunkData(out aTS[0], out aTS[1], out Bnd_Box aPBBox, out bool isSplit);

                // Middle point
                // Tangent vector in the middle point
                aBAC.D1(BOPTools_AlgoTools2D.IntermediatePoint(aTS[0], aTS[1]), out gp_Pnt aPOnE, out gp_Vec aVETgt);
                if (aVETgt.SquareMagnitude() < gp.Resolution())
                {
                    continue;
                }

                aProjPS.Perform(aPOnE);
                if (aProjPS.NbPoints() == 0)
                {
                    continue;
                }

                // Check the distance in the middle point, using the max vertices
                // tolerance as the criteria.
                TopoDS_Vertex aV1 = TopoDS.Vertex(myDS.Shape(nV1));
                TopoDS_Vertex aV2 = TopoDS.Vertex(myDS.Shape(nV2));

                // In the Self-Interference check mode we are interested in real
                // intersections only, so use only the real tolerance of edges,
                // no need to use the extended tolerance.
                double aTolCheck = (bSICheckMode ? myFuzzyValue
                                                 : 2 * Max(BRep_Tool.Tolerance(aV1), BRep_Tool.Tolerance(aV2)));

                if (aProjPS.LowerDistance() > aTolCheck + myFuzzyValue)
                {
                    continue;
                }

                aProjPS.LowerDistanceParameters(out double U, out double V);
                if (!myContext.IsPointInFace(aF, new gp_Pnt2d(U, V)))
                {
                    continue;
                }

                if (aSurfAdaptor.GetType() != GeomAbs_Plane || aBAC.GetType() != GeomAbs_Line)
                {
                    gp_Pnt aPOnS = aProjPS.NearestPoint();
                    gp_Vec aVFNorm = new(aPOnS, aPOnE);
                    if (aVFNorm.SquareMagnitude() > gp.Resolution())
                    {
                        // Angle between vectors should be close to 90 degrees.
                        // We allow deviation of 25 degrees.
                        double aCos = aVFNorm.Normalized().Dot(aVETgt.Normalized());
                        if (Math.Abs(aCos) > 0.4226)
                        {
                            bUseAddTol = false;
                        }
                    }
                }

                // Compute an addition to Fuzzy value
                double aTolAdd = 0.0;
                if (bUseAddTol)
                {
                    // Compute the distance from the bounding points of the edge
                    // to the face and use the maximal of these distances as a
                    // fuzzy tolerance for the intersection.
                    // Use the maximal tolerance of the pave block's vertices
                    // as a max criteria for the computed distance.

                    for (int iP = 0; iP < 2; ++iP)
                    {
                        gp_Pnt aP = aBAC.Value(aTS[iP]);
                        aProjPS.Perform(aP);
                        if (aProjPS.NbPoints() != 0)
                        {
                            double aDistEF = aProjPS.LowerDistance();
                            if (aDistEF < aTolCheck && aDistEF > aTolAdd)
                            {
                                aTolAdd = aDistEF;
                            }
                        }
                    }
                    if (aTolAdd > 0.0)
                    {
                        aTolAdd -= (BRep_Tool.Tolerance(aE) + BRep_Tool.Tolerance(aF));
                        if (aTolAdd < 0.0)
                        {
                            aTolAdd = 0.0;
                        }
                    }
                }

                bool bIntersect = aTolAdd > 0;
                if (!bIntersect)
                {
                    bIntersect = !myFPBDone.Find(nF, out NCollection_Map<BOPDS_PaveBlock> pMPB) || !pMPB.Contains(aPB);
                }

                if (bIntersect)
                {
                    // Prepare pair for intersection
                    BOPAlgo_EdgeFace aEdgeFace = aVEdgeFace.Append(new BOPAlgo_EdgeFace());
                    aEdgeFace.SetIndices(nE, nF);
                    aEdgeFace.SetPaveBlock(aPB);
                    aEdgeFace.SetEdge(aE);
                    aEdgeFace.SetFace(aF);
                    aEdgeFace.SetBoxes(myDS.ShapeInfo(nE).Box(), myDS.ShapeInfo(nF).Box());
                    aEdgeFace.SetFuzzyValue(myFuzzyValue + aTolAdd);
                    aEdgeFace.UseQuickCoincidenceCheck(true);
                    aEdgeFace.SetRange(new IntTools_Range(aPB.Pave1().Parameter(), aPB.Pave2().Parameter()));
                }
            }
        }

        int aNbEFs = aVEdgeFace.Length();
        if (aNbEFs == 0)
        {
            return;
        }

        // close preparation step
        aPSOuter.Next(0.7);

        aPBMap.Clear();

        using var aPS = new Message_ProgressScope(aPSOuter.Next(9), "Checking for edges coinciding with faces", aNbEFs);
        for (int i = 0; i < aNbEFs; i++)
        {
            BOPAlgo_EdgeFace aEdgeFace = aVEdgeFace[i];
            aEdgeFace.SetProgressRange(aPS.Next());
        }
        // Perform intersection of the found pairs
        BOPTools_Parallel.Perform(myRunParallel, aVEdgeFace, myContext!);
        if (UserBreak(aPSOuter))
        {
            return;
        }

        NCollection_DynamicArray<BOPDS_InterfEF> aEFs = myDS.InterfEF();

        // Analyze the results of intersection looking for TopAbs_EDGE
        // intersection type only.

        // Collect all pairs for common block creation
        NCollection_IndexedDataMap<BOPDS_PaveBlock, NCollection_List<int>> aMPBLI = new();
        for (int i = 0; i < aNbEFs; ++i)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            BOPAlgo_EdgeFace anEdgeFace = aVEdgeFace[i];
            if (!anEdgeFace.IsDone() || anEdgeFace.HasErrors())
            {
                // Warn about failed intersection of sub-shapes
                AddIntersectionFailedWarning(anEdgeFace.Edge(), anEdgeFace.Face());
                continue;
            }

            NCollection_Sequence<IntTools_CommonPrt> aCParts = anEdgeFace.CommonParts();
            if (aCParts.Length() != 1)
            {
                continue;
            }

            IntTools_CommonPrt aCP = aCParts[1];
            if (aCP.Type() != TopAbs_EDGE)
            {
                continue;
            }

            anEdgeFace.Indices(out int nE, out int nF);
            if (theAddInterf)
            {
                // Add interference
                BOPDS_InterfEF aEF = aEFs.Append(new BOPDS_InterfEF());
                aEF.SetIndices(nE, nF);
                aEF.SetCommonPart(aCP);
                myDS.AddInterf(nE, nF);
            }

            BOPDS_PaveBlock aPB = anEdgeFace.PaveBlock();
            // Update face information with new IN pave block
            myDS.ChangeFaceInfo(nF).ChangePaveBlocksIn().Add(aPB);
            if (theAddInterf)
            {
                // Fill map for common blocks creation
                BOPAlgo_Tools.FillMap(aPB, nF, aMPBLI);
            }
        }

        if (aMPBLI.Extent() != 0)
        {
            // Create new common blocks for coinciding pairs
            BOPAlgo_Tools.PerformCommonBlocks(aMPBLI, myDS);
        }
    }
}
