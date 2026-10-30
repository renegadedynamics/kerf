// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>The edge/edge intersection solver of the pave filler's parallel loops.</summary>
/// <remarks>
/// OCCT derives it from IntTools_EdgeEdge and BOPAlgo_ParallelAlgo. C# has one base class: it derives from
/// IntTools_EdgeEdge, implements <see cref="BOPTools_Parallel.ISolver"/>, and holds what BOPAlgo_ParallelAlgo gives it
/// (the progress range, and a <see cref="BOPAlgo_Options"/> for the report) as members. <see cref="Perform"/> hides the
/// base's: it moves the edges near the origin when they are far from it (<see cref="BOPAlgo_Tools.TrsfToPoint(in Bnd_Box, in Bnd_Box, ref gp_Trsf)"/>),
/// intersects them, restores them and sets the restored edges in the common parts.
/// </remarks>
internal sealed class BOPAlgo_EdgeEdge : IntTools_EdgeEdge, BOPTools_Parallel.ISolver
{
    private readonly BOPAlgo_Options myOptions = new();
    private Message_ProgressRange myProgressRange;
    private BOPDS_PaveBlock? myPB1;
    private BOPDS_PaveBlock? myPB2;
    private Bnd_Box myBox1 = new();
    private Bnd_Box myBox2 = new();

    //
    public void SetPaveBlock1(BOPDS_PaveBlock aPB) => myPB1 = aPB;

    //
    public BOPDS_PaveBlock PaveBlock1() => myPB1!;

    //
    public void SetPaveBlock2(BOPDS_PaveBlock aPB) => myPB2 = aPB;

    //
    public BOPDS_PaveBlock PaveBlock2() => myPB2!;

    //
    public void SetBoxes(in Bnd_Box theBox1, in Bnd_Box theBox2)
    {
        myBox1 = theBox1;
        myBox2 = theBox2;
    }

    // BOPAlgo_ParallelAlgo's members
    public void SetProgressRange(Message_ProgressRange theRange) => myProgressRange = theRange;

    public bool HasErrors() => myOptions.HasErrors();

    //
    public new void Perform()
    {
        using var aPS = new Message_ProgressScope(myProgressRange, null, 1);
        if (myOptions.UserBreak(aPS))
        {
            return;
        }
        TopoDS_Edge anE1 = myEdge1, anE2 = myEdge2;
        bool hasTrsf = false;
        try
        {
            gp_Trsf aTrsf = new();
            if (BOPAlgo_Tools.TrsfToPoint(myBox1, myBox2, ref aTrsf))
            {
                // Shapes are located far from origin, move the shapes to the origin,
                // to increase the accuracy of intersection.
                TopLoc_Location aLoc = new(aTrsf);
                myEdge1.Move(aLoc);
                myEdge2.Move(aLoc);
                hasTrsf = true;
            }

            base.Perform();
        }
        catch (Standard_Failure)
        {
            myOptions.AddError(new BOPAlgo_AlertIntersectionFailed());
        }

        myEdge1 = anE1;
        myEdge2 = anE2;
        if (hasTrsf)
        {
            for (int i = 1; i <= myCommonParts.Length(); ++i)
            {
                IntTools_CommonPrt aCPart = myCommonParts[i];
                aCPart.SetEdge1(myEdge1);
                aCPart.SetEdge2(myEdge2);
            }
        }
    }
}

public partial class BOPAlgo_PaveFiller
{
    /// <summary>The edge/edge interferences: new vertices and common blocks.</summary>
    protected virtual void PerformEE(Message_ProgressRange theRange)
    {
        FillShrunkData(TopAbs_EDGE, TopAbs_EDGE);
        //
        myIterator!.Initialize(TopAbs_EDGE, TopAbs_EDGE);
        int iSize = myIterator.ExpectedLength();
        using var aPSOuter = new Message_ProgressScope(theRange, null, 10);
        if (iSize == 0)
        {
            return;
        }
        //
        bool bExpressCompute, bIsPBSplittable1, bIsPBSplittable2;
        int i, iX, nE1, nE2, aNbCPrts, k, aNbEdgeEdge;
        int nV11, nV12, nV21, nV22;
        // OCCT: uninitialised, written by GetPBBox before they are read
        double aTS11 = 0, aTS12 = 0, aTS21 = 0, aTS22 = 0, aT11 = 0, aT12 = 0, aT21 = 0, aT22 = 0;
        TopAbs_ShapeEnum aType;
        // The fixed arrays of the TopAbs_VERTEX case (bIsOnPave[4], nV[4], nVS[2])
        Span<bool> bIsOnPave = stackalloc bool[4];
        Span<int> nV = stackalloc int[4];
        Span<int> nVS = stackalloc int[2];
        NCollection_List<BOPDS_PaveBlock>.Iterator aIt1 = new(), aIt2 = new();
        NCollection_DynamicArray<BOPAlgo_EdgeEdge> aVEdgeEdge = new();
        // keep modified edges for further update
        NCollection_Map<int> aMEdges = new();
        //
        //-----------------------------------------------------scope f
        NCollection_IndexedDataMap<BOPDS_PaveBlock, NCollection_List<BOPDS_PaveBlock>> aMPBLPB = new();
        NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> aMVCPB = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<BOPDS_PaveBlock, Bnd_Box> aDMPBBox = new();
        //
        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = myDS!.InterfEE();
        //
        for (; myIterator.More(); myIterator.Next())
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            myIterator.Value(out nE1, out nE2);
            //
            BOPDS_ShapeInfo aSIE1 = myDS.ShapeInfo(nE1);
            if (aSIE1.HasFlag())
            {
                continue;
            }
            BOPDS_ShapeInfo aSIE2 = myDS.ShapeInfo(nE2);
            if (aSIE2.HasFlag())
            {
                continue;
            }
            //
            NCollection_List<BOPDS_PaveBlock> aLPB1 = myDS.ChangePaveBlocks(nE1);
            if (aLPB1.IsEmpty())
            {
                continue;
            }
            //
            NCollection_List<BOPDS_PaveBlock> aLPB2 = myDS.ChangePaveBlocks(nE2);
            if (aLPB2.IsEmpty())
            {
                continue;
            }
            //
            TopoDS_Edge aE1 = TopoDS.Edge(aSIE1.Shape());
            TopoDS_Edge aE2 = TopoDS.Edge(aSIE2.Shape());
            //
            aIt1.Initialize(aLPB1);
            for (; aIt1.More(); aIt1.Next())
            {
                if (UserBreak(aPSOuter))
                {
                    return;
                }
                Bnd_Box aBB1 = new();
                //
                BOPDS_PaveBlock aPB1 = aIt1.Value();
                //
                if (!GetPBBox(aE1, aPB1, aDMPBBox, ref aT11, ref aT12, ref aTS11, ref aTS12, ref aBB1))
                {
                    continue;
                }
                //
                aPB1.Indices(out nV11, out nV12);
                //
                aIt2.Initialize(aLPB2);
                for (; aIt2.More(); aIt2.Next())
                {
                    Bnd_Box aBB2 = new();
                    //
                    BOPDS_PaveBlock aPB2 = aIt2.Value();
                    //
                    if (!GetPBBox(aE2, aPB2, aDMPBBox, ref aT21, ref aT22, ref aTS21, ref aTS22, ref aBB2))
                    {
                        continue;
                    }
                    //
                    if (aBB1.IsOut(aBB2))
                    {
                        continue;
                    }
                    //
                    aPB2.Indices(out nV21, out nV22);
                    //
                    bExpressCompute = ((nV11 == nV21 && nV12 == nV22) || (nV12 == nV21 && nV11 == nV22));
                    //
                    BOPAlgo_EdgeEdge anEdgeEdge = aVEdgeEdge.Append(new BOPAlgo_EdgeEdge());
                    //
                    anEdgeEdge.UseQuickCoincidenceCheck(bExpressCompute);
                    //
                    anEdgeEdge.SetPaveBlock1(aPB1);
                    anEdgeEdge.SetPaveBlock2(aPB2);
                    //
                    anEdgeEdge.SetEdge1(aE1, aT11, aT12);
                    anEdgeEdge.SetEdge2(aE2, aT21, aT22);
                    anEdgeEdge.SetBoxes(aBB1, aBB2);
                    anEdgeEdge.SetFuzzyValue(myFuzzyValue);
                } // for (; aIt2.More(); aIt2.Next()) {
            } // for (; aIt1.More(); aIt1.Next()) {
        } // for (; myIterator->More(); myIterator->Next()) {
        //
        aNbEdgeEdge = aVEdgeEdge.Length();

        using (var aPS = new Message_ProgressScope(aPSOuter.Next(9), "Performing Edge-edge intersection", aNbEdgeEdge))
        {
            for (k = 0; k < aNbEdgeEdge; k++)
            {
                BOPAlgo_EdgeEdge anEdgeEdge = aVEdgeEdge.ChangeValue(k);
                anEdgeEdge.SetProgressRange(aPS.Next());
            }
            //======================================================
            BOPTools_Parallel.Perform(myRunParallel, aVEdgeEdge);
            //======================================================
        }
        if (UserBreak(aPSOuter))
        {
            return;
        }
        //
        for (k = 0; k < aNbEdgeEdge; ++k)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            Bnd_Box aBB1 = new(), aBB2 = new();
            //
            BOPAlgo_EdgeEdge anEdgeEdge = aVEdgeEdge[k];
            if (!anEdgeEdge.IsDone() || anEdgeEdge.HasErrors())
            {
                // Warn about failed intersection of sub-shapes
                TopoDS_Shape aE1f = myDS.Shape(anEdgeEdge.PaveBlock1().OriginalEdge());
                TopoDS_Shape aE2f = myDS.Shape(anEdgeEdge.PaveBlock2().OriginalEdge());
                AddIntersectionFailedWarning(aE1f, aE2f);
                continue;
            }
            //
            NCollection_Sequence<IntTools_CommonPrt> aCPrts = anEdgeEdge.CommonParts();
            aNbCPrts = aCPrts.Length();
            if (aNbCPrts == 0)
            {
                continue;
            }
            //--------------------------------------------
            BOPDS_PaveBlock aPB1 = anEdgeEdge.PaveBlock1();
            nE1 = aPB1.OriginalEdge();
            aPB1.Range(out aT11, out aT12);
            if (!aPB1.HasShrunkData())
            {
                aTS11 = aT11;
                aTS12 = aT12;
                bIsPBSplittable1 = false;
            }
            else
            {
                aPB1.ShrunkData(out aTS11, out aTS12, out aBB1, out bIsPBSplittable1);
            }
            //
            BOPDS_PaveBlock aPB2 = anEdgeEdge.PaveBlock2();
            nE2 = aPB2.OriginalEdge();
            aPB2.Range(out aT21, out aT22);
            if (!aPB2.HasShrunkData())
            {
                aTS21 = aT21;
                aTS22 = aT22;
                bIsPBSplittable2 = false;
            }
            else
            {
                aPB2.ShrunkData(out aTS21, out aTS22, out aBB2, out bIsPBSplittable2);
            }
            //
            //--------------------------------------------
            IntTools_Range aR11 = new(aT11, aTS11), aR12 = new(aTS12, aT12), aR21 = new(aT21, aTS21), aR22 = new(aTS22, aT22);
            //
            bool bAnalytical = false;
            {
                TopoDS_Edge aOE1 = TopoDS.Edge(myDS.Shape(nE1));
                TopoDS_Edge aOE2 = TopoDS.Edge(myDS.Shape(nE2));
                //
                BRepAdaptor_Curve aBAC1 = new(aOE1), aBAC2 = new(aOE2);
                //
                GeomAbs_CurveType aType1 = aBAC1.GetType();
                GeomAbs_CurveType aType2 = aBAC2.GetType();
                //
                bAnalytical = (aType1 == GeomAbs_Line && aType2 == GeomAbs_Circle)
                              || (aType1 == GeomAbs_Circle && aType2 == GeomAbs_Line);
            }
            //
            for (i = 1; i <= aNbCPrts; ++i)
            {
                if (UserBreak(aPSOuter))
                {
                    return;
                }
                IntTools_CommonPrt aCPart = aCPrts[i];
                //
                TopoDS_Edge aE1 = aCPart.Edge1();
                TopoDS_Edge aE2 = aCPart.Edge2();
                //
                aType = aCPart.Type();
                switch (aType)
                {
                    case TopAbs_VERTEX:
                    {
                        if (!bIsPBSplittable1 || !bIsPBSplittable2)
                        {
                            continue;
                        }
                        //
                        int j;
                        double aT1, aT2, aTol;
                        TopoDS_Vertex aVnew;
                        IntTools_Range aCR1, aCR2;
                        //
                        IntTools_Tools.VertexParameters(aCPart, out aT1, out aT2);
                        aTol = Precision.Confusion();
                        aCR1 = aCPart.Range1();
                        aCR2 = aCPart.Ranges2().Value(1);
                        //
                        // decide to keep the pave or not
                        bIsOnPave[0] = IntTools_Tools.IsOnPave1(aT1, aR11, aTol)
                                       || IntTools_Tools.IsOnPave1(aR11.First(), aCR1, aTol);
                        bIsOnPave[1] = IntTools_Tools.IsOnPave1(aT1, aR12, aTol)
                                       || IntTools_Tools.IsOnPave1(aR12.Last(), aCR1, aTol);
                        bIsOnPave[2] = IntTools_Tools.IsOnPave1(aT2, aR21, aTol)
                                       || IntTools_Tools.IsOnPave1(aR21.First(), aCR2, aTol);
                        bIsOnPave[3] = IntTools_Tools.IsOnPave1(aT2, aR22, aTol)
                                       || IntTools_Tools.IsOnPave1(aR22.Last(), aCR2, aTol);
                        //
                        aPB1.Indices(out nV[0], out nV[1]);
                        aPB2.Indices(out nV[2], out nV[3]);
                        //
                        if ((bIsOnPave[0] && bIsOnPave[2]) || (bIsOnPave[0] && bIsOnPave[3])
                            || (bIsOnPave[1] && bIsOnPave[2]) || (bIsOnPave[1] && bIsOnPave[3]))
                        {
                            continue;
                        }
                        //
                        bool isVExists = false;
                        for (j = 0; j < 4; ++j)
                        {
                            if (bIsOnPave[j])
                            {
                                BOPDS_PaveBlock aPB = (j < 2) ? aPB2 : aPB1;
                                bIsOnPave[j] = ForceInterfVE(nV[j], aPB, aMEdges);
                                if (bIsOnPave[j])
                                {
                                    isVExists = true;
                                }
                            }
                        }

                        BOPTools_AlgoTools.MakeNewVertex(aE1, aT1, aE2, aT2, out aVnew);
                        gp_Pnt aPnew = BRep_Tool.Pnt(aVnew);

                        if (isVExists)
                        {
                            // The found intersection point is located closely to one of the
                            // pave blocks bounds. So, do not create the new vertex in this point.
                            // Check if this point is a real intersection point or just a touching point.
                            // If it is a touching point, do nothing.
                            // If it is an intersection point, update the existing vertex to cover the
                            // intersection point.
                            gp_Pnt aPOnE1 = new BRepAdaptor_Curve(aE1).Value(aT1);
                            gp_Pnt aPOnE2 = new BRepAdaptor_Curve(aE2).Value(aT2);
                            if (aPOnE1.Distance(aPOnE2) > Precision.Intersection())
                            {
                                // No intersection point
                                continue;
                            }

                            // Real intersection is present.
                            // Update the existing vertex to cover the intersection point.
                            for (j = 0; j < 4; ++j)
                            {
                                if (bIsOnPave[j])
                                {
                                    TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(nV[j]));
                                    gp_Pnt aP = BRep_Tool.Pnt(aV);
                                    double aDistPP = aP.Distance(aPnew);
                                    // Just update the vertex
                                    UpdateVertex(nV[j], aDistPP);
                                    myVertsToAvoidExtension.Add(nV[j]);
                                }
                            }
                        }

                        double aTolVnew = BRep_Tool.Tolerance(aVnew);
                        if (bAnalytical)
                        {
                            // increase tolerance for Line/Circle intersection, but do not update
                            // the vertex till its intersection with some other shape
                            double aTolMin = (new BRepAdaptor_Curve(aE1).GetType() == GeomAbs_Line)
                                ? (aCR1.Last() - aCR1.First()) / 2.0
                                : (aCR2.Last() - aCR2.First()) / 2.0;
                            if (aTolMin > aTolVnew)
                            {
                                aTolVnew = aTolMin;
                            }
                        }
                        // <-LXBR
                        {
                            int iFound;
                            double aTolVx, aD2, aDT2;
                            NCollection_Map<int> aMV = new();
                            gp_Pnt aPx;
                            //
                            iFound = 0;
                            j = -1;
                            aMV.Add(nV[0]);
                            aMV.Add(nV[1]);
                            //
                            if (aMV.Contains(nV[2]))
                            {
                                ++j;
                                nVS[j] = nV[2];
                            }
                            if (aMV.Contains(nV[3]))
                            {
                                ++j;
                                nVS[j] = nV[3];
                            }
                            //
                            for (int k1 = 0; k1 <= j; ++k1)
                            {
                                TopoDS_Vertex aVx = TopoDS.Vertex(myDS.Shape(nVS[k1]));
                                aTolVx = BRep_Tool.Tolerance(aVx);
                                aPx = BRep_Tool.Pnt(aVx);
                                aD2 = aPnew.SquareDistance(aPx);
                                //
                                aDT2 = 100.0 * (aTolVnew + aTolVx) * (aTolVnew + aTolVx);
                                //
                                if (aD2 < aDT2)
                                {
                                    iFound = 1;
                                    break;
                                }
                            }
                            //
                            if (iFound != 0)
                            {
                                continue;
                            }
                        }
                        //
                        // 1
                        BOPDS_InterfEE aEE = aEEs.Append(new BOPDS_InterfEE());
                        iX = aEEs.Length() - 1;
                        aEE.SetIndices(nE1, nE2);
                        aEE.SetCommonPart(aCPart);
                        // 2
                        myDS.AddInterf(nE1, nE2);
                        //
                        BOPDS_CoupleOfPaveBlocks aCPB = new();
                        //
                        aCPB.SetPaveBlocks(aPB1, aPB2);
                        aCPB.SetIndexInterf(iX);
                        aCPB.SetTolerance(aTolVnew);
                        aMVCPB.Add(aVnew, aCPB);
                    } // case TopAbs_VERTEX:
                        break;
                    //
                    case TopAbs_EDGE:
                    {
                        if (aNbCPrts > 1)
                        {
                            break;
                        }
                        //
                        bool bHasSameBounds;
                        bHasSameBounds = aPB1.HasSameBounds(aPB2);
                        if (!bHasSameBounds)
                        {
                            break;
                        }
                        // 1
                        BOPDS_InterfEE aEE = aEEs.Append(new BOPDS_InterfEE());
                        iX = aEEs.Length() - 1;
                        aEE.SetIndices(nE1, nE2);
                        aEE.SetCommonPart(aCPart);
                        // 2
                        myDS.AddInterf(nE1, nE2);
                        //
                        BOPAlgo_Tools.FillMap(aPB1, aPB2, aMPBLPB);
                    } // case TopAbs_EDGE
                        break;
                    default:
                        break;
                } // switch (aType) {
            } // for (i=1; i<=aNbCPrts; i++) {
        } // for (k=0; k < aNbFdgeEdge; ++k) {
        //
        //=========================================
        // post treatment
        //=========================================
        BOPAlgo_Tools.PerformCommonBlocks(aMPBLPB, myDS, myContext);
        // Update vertices of common blocks with real CB tolerances
        UpdateVerticesOfCB();

        PerformNewVertices(aMVCPB, aPSOuter.Next());
        if (HasErrors())
        {
            return;
        }
        //
        if (aMEdges.Extent() != 0)
        {
            int aNbV = aMVCPB.Extent();
            for (i = 1; i <= aNbV; ++i)
            {
                BOPDS_CoupleOfPaveBlocks aCPB = aMVCPB.FindFromIndex(i);
                BOPDS_PaveBlock aPB1 = aCPB.PaveBlock1()!;
                BOPDS_PaveBlock aPB2 = aCPB.PaveBlock2()!;
                //
                aMEdges.Remove(aPB1.OriginalEdge());
                aMEdges.Remove(aPB2.OriginalEdge());
            }
            //
            SplitPaveBlocks(aMEdges, false);
        }
        //
        //-----------------------------------------------------scope t
        aMPBLPB.Clear();
        aMVCPB.Clear();
    }

    /// <summary>
    /// Fuses the new vertices of the edge/edge (or edge/face) interferences, adds them to the DS and splits the pave
    /// blocks by them.
    /// </summary>
    protected void PerformNewVertices(NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> theMVCPB,
                                      Message_ProgressRange theRange, bool theIsEEIntersection = true)
    {
        int aNbV = theMVCPB.Extent();
        if (aNbV == 0)
        {
            return;
        }
        //
        double aTolAdd = myFuzzyValue / 2.0;
        //
        // 1. Fuse the new vertices
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aImages = new(TopTools_ShapeMapHasher.Instance);
        TreatNewVertices(theMVCPB, aImages);
        //
        // 2. Add new vertices to myDS and connect indices to CPB structure
        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = myDS!.InterfEE();
        NCollection_DynamicArray<BOPDS_InterfEF> aEFs = myDS.InterfEF();
        //
        // 4. Compute Extra Paves and split Pave blocks by the Extra paves
        using var aPS = new Message_ProgressScope(theRange, null, 2);
        int i, aNb = aImages.Extent();
        using (var aPS1 = new Message_ProgressScope(aPS.Next(), null, aNb + aNbV))
        {
            for (i = 1; i <= aNb; ++i, aPS1.Next())
            {
                if (UserBreak(aPS))
                {
                    return;
                }
                TopoDS_Vertex aV = TopoDS.Vertex(aImages.FindKey(i));
                NCollection_List<TopoDS_Shape> aLVSD = aImages.FindFromIndex(i);
                //
                BOPDS_ShapeInfo aSI = new();
                aSI.SetShapeType(TopAbs_VERTEX);
                aSI.SetShape(aV);
                int iV = myDS.Append(aSI);
                //
                BOPDS_ShapeInfo aSIDS = myDS.ChangeShapeInfo(iV);
                ref Bnd_Box aBox = ref aSIDS.ChangeBox();
                aBox.Add(BRep_Tool.Pnt(aV));
                aBox.SetGap(BRep_Tool.Tolerance(aV) + aTolAdd);
                //
                NCollection_List<TopoDS_Shape>.Iterator aItLS = new(aLVSD);
                for (; aItLS.More(); aItLS.Next())
                {
                    TopoDS_Shape aVx = aItLS.Value();
                    ref BOPDS_CoupleOfPaveBlocks aCPB = ref theMVCPB.ChangeFromKey(aVx);
                    aCPB.SetIndex(iV);
                    // update interference
                    int iX = aCPB.IndexInterf();
                    BOPDS_Interf aInt = theIsEEIntersection ? aEEs[iX] : aEFs[iX];
                    aInt.SetIndexNew(iV);
                }
            }
            //
            // 3. Map PaveBlock/ListOfVertices to add to this PaveBlock ->aMPBLI
            NCollection_IndexedDataMap<BOPDS_PaveBlock, NCollection_List<int>> aMPBLI = new();
            for (i = 1; i <= aNbV; ++i, aPS1.Next())
            {
                if (UserBreak(aPS))
                {
                    return;
                }
                BOPDS_CoupleOfPaveBlocks aCPB = theMVCPB.FindFromIndex(i);
                int iV = aCPB.Index();
                //
                BOPDS_PaveBlock aPB1 = aCPB.PaveBlock1()!;
                BOPDS_PaveBlock aPB2 = aCPB.PaveBlock2()!;
                BOPDS_PaveBlock[] aPB = [aPB1, aPB2];
                for (int j = 0; j < 2; ++j)
                {
                    int anIndex = aMPBLI.FindIndex(aPB[j]);
                    NCollection_List<int> pLI = anIndex > 0 ? aMPBLI[anIndex] : aMPBLI[aMPBLI.Add(aPB[j], new NCollection_List<int>())];
                    pLI.Append(iV);
                    //
                    if (aPB[0] == aPB[1])
                    {
                        break;
                    }
                }
            }
            // 4. Compute Extra Paves and split Pave blocks by the Extra paves
            IntersectVE(aMPBLI, aPS.Next(), false);
        }
    }

    /// <summary>Fuses the coinciding new vertices: each chain of intersecting vertices becomes one vertex.</summary>
    protected void TreatNewVertices(NCollection_IndexedDataMap<TopoDS_Shape, BOPDS_CoupleOfPaveBlocks> theMVCPB,
                                    NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theImages)
    {
        //
        // Prepare for intersection
        NCollection_IndexedDataMap<TopoDS_Shape, double> aVerts = new(TopTools_ShapeMapHasher.Instance);
        int i, aNbV = theMVCPB.Extent();
        for (i = 1; i <= aNbV; ++i)
        {
            TopoDS_Shape aV = theMVCPB.FindKey(i);
            double aTol = theMVCPB.FindFromIndex(i).Tolerance();
            aVerts.Add(aV, aTol);
        }
        //
        // Perform intersection
        NCollection_List<NCollection_List<TopoDS_Shape>> aChains = new();
        BOPAlgo_Tools.IntersectVertices(aVerts, myFuzzyValue, aChains);
        //
        // Treat the results - make new vertices for each chain
        NCollection_List<NCollection_List<TopoDS_Shape>>.Iterator aItC = new(aChains);
        for (; aItC.More(); aItC.Next())
        {
            NCollection_List<TopoDS_Shape> aLVSD = aItC.Value();
            //
            TopoDS_Vertex aVNew = new();
            BOPTools_AlgoTools.MakeVertex(aLVSD, ref aVNew);
            theImages.Add(aVNew, aLVSD);
        }
    }

    /// <summary>Computes the shrunk data (valid range and box) of the pave block.</summary>
    protected void FillShrunkData(BOPDS_PaveBlock thePB)
    {
        // Vertices
        thePB.Indices(out int nV1, out int nV2);

        if (nV1 < 0 || nV2 < 0)
        {
            return;
        }

        TopoDS_Vertex aV1 = TopoDS.Vertex(myDS!.Shape(nV1));
        TopoDS_Vertex aV2 = TopoDS.Vertex(myDS.Shape(nV2));
        // Get the edge
        if (!thePB.HasEdge(out int nE))
        {
            nE = thePB.OriginalEdge();
            if (nE < 0)
            {
                return;
            }
        }

        TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
        // Range
        thePB.Range(out double aT1, out double aT2);
        //
        IntTools_ShrunkRange aSR = new();
        aSR.SetContext(myContext);
        aSR.SetData(aE, aT1, aT2, aV1, aV2);
        aSR.Perform();
        // Analyze the results of computations
        AnalyzeShrunkData(thePB, aSR);
    }

    /// <summary>Sets the shrunk data of the pave block from the computation, with warnings where it failed.</summary>
    protected void AnalyzeShrunkData(BOPDS_PaveBlock thePB, IntTools_ShrunkRange theSR)
    {
        // in case of error treat the warning status
        bool bWholeEdge = false;
        TopoDS_Shape aWarnShape = new();
        //
        if (!theSR.IsDone() || !theSR.IsSplittable())
        {
            BRep_Tool.Range(theSR.Edge(), out double aEFirst, out double aELast);
            thePB.Range(out double aPBFirst, out double aPBLast);
            bWholeEdge = aPBFirst <= aEFirst && aPBLast >= aELast;
            if (bWholeEdge && thePB.OriginalEdge() >= 0)
            {
                aWarnShape = theSR.Edge();
            }
            else
            {
                TopoDS_Shape aV1 = myDS!.Shape(thePB.Pave1().Index());
                TopoDS_Shape aV2 = myDS.Shape(thePB.Pave2().Index());
                TopoDS_Compound aWC = new();
                new BRep_Builder().MakeCompound(ref aWC);
                aWarnShape = aWC;
                new BRep_Builder().Add(aWarnShape, theSR.Edge());
                new BRep_Builder().Add(aWarnShape, aV1);
                new BRep_Builder().Add(aWarnShape, aV2);
            }
            //
            if (!theSR.IsDone())
            {
                if (bWholeEdge)
                {
                    AddWarning(new BOPAlgo_AlertTooSmallEdge(aWarnShape));
                }
                else
                {
                    AddWarning(new BOPAlgo_AlertBadPositioning(aWarnShape));
                }
                theSR.ShrunkRange(out double aTS1f, out double aTS2f);
                thePB.SetShrunkData(aTS1f, aTS2f, new Bnd_Box(), false);
                return;
            }
            //
            if (bWholeEdge)
            {
                AddWarning(new BOPAlgo_AlertNotSplittableEdge(aWarnShape));
            }
            else
            {
                AddWarning(new BOPAlgo_AlertBadPositioning(aWarnShape));
            }
        }
        //
        theSR.ShrunkRange(out double aTS1, out double aTS2);
        Bnd_Box aBox = theSR.BndBox();
        aBox.SetGap(aBox.GetGap() + myFuzzyValue / 2.0);
        thePB.SetShrunkData(aTS1, aTS2, aBox, theSR.IsSplittable());
    }

    /// <summary>
    /// Forces the vertex/edge interference of the vertex with the pave block's edge: on success adds the interference
    /// and the extra pave, and the edge to <paramref name="theMEdges"/>.
    /// </summary>
    protected bool ForceInterfVE(int nV, BOPDS_PaveBlock aPB, NCollection_Map<int> theMEdges)
    {
        int nE, nVx, nVSD = 0, iFlag;
        double aT = 0.0, aTolVNew = 0.0; // OCCT: uninitialised, read only on success
        //
        nE = aPB.OriginalEdge();
        //
        BOPDS_ShapeInfo aSIE = myDS!.ShapeInfo(nE);
        if (aSIE.HasSubShape(nV))
        {
            return true;
        }
        //
        if (myDS.HasInterf(nV, nE))
        {
            return true;
        }
        //
        if (myDS.HasInterfShapeSubShapes(nV, nE))
        {
            return true;
        }
        //
        if (aPB.Pave1().Index() == nV || aPB.Pave2().Index() == nV)
        {
            return true;
        }
        //
        nVx = nV;
        if (myDS.HasShapeSD(nV, ref nVSD))
        {
            nVx = nVSD;
        }
        //
        TopoDS_Vertex aV = TopoDS.Vertex(myDS.Shape(nVx));
        TopoDS_Edge aE = TopoDS.Edge(myDS.Shape(nE));
        //
        iFlag = myContext!.ComputeVE(aV, aE, ref aT, ref aTolVNew, myFuzzyValue);
        if (iFlag == 0 || iFlag == -4)
        {
            BOPDS_Pave aPave = new();
            //
            //
            NCollection_DynamicArray<BOPDS_InterfVE> aVEs = myDS.InterfVE();
            // 1
            BOPDS_InterfVE aVE = aVEs.Append(new BOPDS_InterfVE());
            aVE.SetIndices(nV, nE);
            aVE.SetParameter(aT);
            // 2
            myDS.AddInterf(nV, nE);
            //
            // 3 update vertex V/E if necessary
            nVx = UpdateVertex(nV, aTolVNew);
            // 4
            if (myDS.IsNewShape(nVx))
            {
                aVE.SetIndexNew(nVx);
            }
            // 5 append ext pave to pave block
            aPave.SetIndex(nVx);
            aPave.SetParameter(aT);
            aPB.AppendExtPave(aPave);
            //
            theMEdges.Add(nE);
            //
            // check for self-interference
            int iRV = myDS.Rank(nV);
            if (iRV >= 0 && iRV == myDS.Rank(nE))
            {
                // add warning status
                TopoDS_Compound aWC = new();
                new BRep_Builder().MakeCompound(ref aWC);
                new BRep_Builder().Add(aWC, aV);
                new BRep_Builder().Add(aWC, aE);
                AddWarning(new BOPAlgo_AlertSelfInterferingShape(aWC));
            }
            return true;
        }
        return false;
    }

    /// <summary>
    /// The range, shrunk range and box of the pave block (its shrunk box, or a box of its range kept in
    /// <paramref name="thePBBox"/>). False if the range is too small.
    /// </summary>
    protected bool GetPBBox(in TopoDS_Edge theE, BOPDS_PaveBlock thePB, NCollection_DataMap<BOPDS_PaveBlock, Bnd_Box> thePBBox,
                            ref double theFirst, ref double theLast, ref double theSFirst, ref double theSLast, ref Bnd_Box theBox)
    {
        thePB.Range(out theFirst, out theLast);
        // check the validity of PB's range
        bool bValid = theLast - theFirst > Precision.PConfusion();
        if (!bValid)
        {
            return bValid;
        }
        //
        // check shrunk data
        if (thePB.HasShrunkData())
        {
            thePB.ShrunkData(out theSFirst, out theSLast, out theBox, out bool bIsSplittable);
            return bValid;
        }
        //
        theSFirst = theFirst;
        theSLast = theLast;
        // check the map
        if (thePBBox.IsBound(thePB))
        {
            theBox = thePBBox.Find(thePB);
        }
        else
        {
            // build bounding box
            BRepAdaptor_Curve aBAC = new(theE);
            double aTol = BRep_Tool.Tolerance(theE) + Precision.Confusion();
            BndLib_Add3dCurve.Add(aBAC, theSFirst, theSLast, aTol, ref theBox);
            thePBBox.Bind(thePB, theBox);
        }
        return bValid;
    }

    /// <summary>Increases the tolerances of the common blocks' vertices to the blocks' tolerances.</summary>
    protected void UpdateVerticesOfCB()
    {
        // Fence map to avoid checking same Common block twice
        NCollection_Map<BOPDS_PaveBlock> aMPBFence = new();

        NCollection_DynamicArray<NCollection_List<BOPDS_PaveBlock>> aPBP = myDS!.ChangePaveBlocksPool();
        int aNbPBP = aPBP.Length();
        for (int i = 0; i < aNbPBP; ++i)
        {
            NCollection_List<BOPDS_PaveBlock> aLPB = aPBP[i];
            NCollection_List<BOPDS_PaveBlock>.Iterator itPB = new(aLPB);
            for (; itPB.More(); itPB.Next())
            {
                BOPDS_CommonBlock? aCB = myDS.CommonBlock(itPB.Value());
                if (aCB is null)
                {
                    continue;
                }

                BOPDS_PaveBlock aPBR = aCB.PaveBlock1();
                if (!aMPBFence.Add(aPBR))
                {
                    continue;
                }

                double aTolCB = aCB.Tolerance();
                if (aTolCB > 0.0)
                {
                    UpdateVertex(aPBR.Pave1().Index(), aTolCB);
                    UpdateVertex(aPBR.Pave2().Index(), aTolCB);
                }
            }
        }
    }

    /// <summary>
    /// Looks for more common blocks among the pave blocks with the same bounding vertices, now that the vertices have
    /// been increased and unified.
    /// </summary>
    protected void ForceInterfEE(Message_ProgressRange theRange)
    {
        // Now that we have vertices increased and unified, try to find additional
        // common blocks among the pairs of edges.
        // Since all real intersections should have already happened, here we
        // are interested in common blocks only, thus we need to check only
        // those pairs of pave blocks with the same bounding vertices.

        using var aPSOuter = new Message_ProgressScope(theRange, null, 10);
        // Initialize pave blocks for all vertices which participated in intersections
        int aNbS = myDS!.NbSourceShapes();
        for (int i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() == TopAbs_VERTEX)
            {
                if (myDS.HasInterf(i))
                {
                    myDS.InitPaveBlocksForVertex(i);
                }
            }
            if (UserBreak(aPSOuter))
            {
                return;
            }
        }
        // Fill the connection map from bounding vertices to pave blocks
        // having those bounding vertices
        NCollection_IndexedDataMap<BOPDS_Pair, NCollection_List<BOPDS_PaveBlock>> aPBMap = new();
        // Fence map of pave blocks
        NCollection_Map<BOPDS_PaveBlock> aMPBFence = new();

        for (int i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
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
            if (UserBreak(aPSOuter))
            {
                return;
            }
            NCollection_List<BOPDS_PaveBlock> aLPB = myDS.PaveBlocks(i);
            NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPB);
            for (; aItLPB.More(); aItLPB.Next())
            {
                BOPDS_PaveBlock aPB = aItLPB.Value();
                BOPDS_PaveBlock aPBR = myDS.RealPaveBlock(aPB);
                if (!aMPBFence.Add(aPBR))
                {
                    continue;
                }

                // Get indices
                aPBR.Indices(out int nV1, out int nV2);

                // Add pave block to a map
                BOPDS_Pair aPair = new(nV1, nV2);
                int anIndex = aPBMap.FindIndex(aPair);
                NCollection_List<BOPDS_PaveBlock> pList = anIndex > 0
                    ? aPBMap[anIndex]
                    : aPBMap[aPBMap.Add(aPair, new NCollection_List<BOPDS_PaveBlock>())];
                pList.Append(aPBR);
            }
        }

        int aNbPB = aPBMap.Extent();
        if (aNbPB == 0)
        {
            return;
        }

        bool bSICheckMode = (myArguments.Extent() == 1);

        // Prepare pave blocks with the same vertices for intersection.
        NCollection_DynamicArray<BOPAlgo_EdgeEdge> aVEdgeEdge = new();

        for (int i = 1; i <= aNbPB; ++i)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            NCollection_List<BOPDS_PaveBlock> aLPB = aPBMap[i];
            if (aLPB.Extent() < 2)
            {
                continue;
            }

            BOPDS_Pair aPair = aPBMap.FindKey(i);
            aPair.Indices(out int nV1, out int nV2);

            TopoDS_Vertex aV1 = TopoDS.Vertex(myDS.Shape(nV1));
            TopoDS_Vertex aV2 = TopoDS.Vertex(myDS.Shape(nV2));

            // Use the max tolerance of vertices as Fuzzy value for intersection of edges.
            // In the Self-Interference check mode we are interested in real
            // intersections only, so use only the real tolerance of edges,
            // no need to use the extended tolerance.
            double aTolAdd = (bSICheckMode ? myFuzzyValue
                                           : 2 * Max(BRep_Tool.Tolerance(aV1), BRep_Tool.Tolerance(aV2)));

            // All possible pairs combined from the list <aLPB> should be checked
            NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB1 = new(aLPB);
            for (; aItLPB1.More(); aItLPB1.Next())
            {
                BOPDS_PaveBlock aPB1 = aItLPB1.Value();
                BOPDS_CommonBlock? aCB1 = myDS.CommonBlock(aPB1);
                int nE1 = aPB1.OriginalEdge();
                int iR1 = myDS.Rank(nE1);
                TopoDS_Edge aE1 = TopoDS.Edge(myDS.Shape(nE1));
                aPB1.Range(out double aT11, out double aT12);
                BRepAdaptor_Curve aBAC1 = new(aE1);
                aBAC1.D1((aT11 + aT12) * 0.5, out gp_Pnt aPm, out gp_Vec aVTgt1);
                if (aVTgt1.SquareMagnitude() < gp.Resolution())
                {
                    continue;
                }
                aVTgt1.Normalize();

                NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB2 = new();
                aItLPB2.Reset(aLPB, aItLPB1.Index);
                for (aItLPB2.Next(); aItLPB2.More(); aItLPB2.Next())
                {
                    BOPDS_PaveBlock aPB2 = aItLPB2.Value();
                    BOPDS_CommonBlock? aCB2 = myDS.CommonBlock(aPB2);
                    int nE2 = aPB2.OriginalEdge();
                    int iR2 = myDS.Rank(nE2);

                    // Check that the edges came from different arguments
                    if (iR1 == iR2)
                    {
                        // If the sharing of the vertices is not original, but has been acquired
                        // during the operation, check the coincidence of the edges even if
                        // they came from the same argument
                        if ((!myDS.IsNewShape(nV1) && (myDS.Rank(nV1) == iR1))
                            || (!myDS.IsNewShape(nV2) && (myDS.Rank(nV2) == iR2)))
                        {
                            continue;
                        }
                    }

                    // Check that the Pave blocks do not form the Common block already
                    if (aCB1 is not null && aCB2 is not null)
                    {
                        if (aCB1 == aCB2)
                        {
                            continue;
                        }
                    }

                    TopoDS_Edge aE2 = TopoDS.Edge(myDS.Shape(nE2));
                    aPB2.Range(out double aT21, out double aT22);

                    // Check the angle between edges in the middle point.
                    // If the angle is more than 25 degrees, do not use the additional
                    // tolerance, as it may lead to undesired unification of edges
                    bool bUseAddTol = true;
                    {
                        BRepAdaptor_Curve aBAC2 = new(aE2);
                        if (aBAC1.GetType() != GeomAbs_Line || aBAC2.GetType() != GeomAbs_Line)
                        {
                            GeomAPI_ProjectPointOnCurve aProjPC = myContext!.ProjPC(aE2);
                            aProjPC.Perform(aPm);
                            if (aProjPC.NbPoints() == 0)
                            {
                                continue;
                            }

                            aBAC2.D1(aProjPC.LowerDistanceParameter(), out gp_Pnt aPm2, out gp_Vec aVTgt2);
                            if (aVTgt2.SquareMagnitude() < gp.Resolution())
                            {
                                continue;
                            }

                            // The angle should be close to zero
                            double aCos = aVTgt1.Dot(aVTgt2.Normalized());
                            if (Math.Abs(aCos) < 0.9063)
                            {
                                bUseAddTol = false;
                            }
                        }
                    }

                    // Add pair for intersection
                    BOPAlgo_EdgeEdge anEdgeEdge = aVEdgeEdge.Append(new BOPAlgo_EdgeEdge());
                    anEdgeEdge.UseQuickCoincidenceCheck(true);
                    anEdgeEdge.SetPaveBlock1(aPB1);
                    anEdgeEdge.SetPaveBlock2(aPB2);
                    anEdgeEdge.SetEdge1(aE1, aT11, aT12);
                    anEdgeEdge.SetEdge2(aE2, aT21, aT22);
                    anEdgeEdge.SetBoxes(myDS.ShapeInfo(nE1).Box(), myDS.ShapeInfo(nE2).Box());
                    if (bUseAddTol)
                    {
                        anEdgeEdge.SetFuzzyValue(myFuzzyValue + aTolAdd);
                    }
                    else
                    {
                        anEdgeEdge.SetFuzzyValue(myFuzzyValue);
                    }
                }
            }
        }

        int aNbPairs = aVEdgeEdge.Length();
        if (aNbPairs == 0)
        {
            return;
        }

        // close preparation step
        aPSOuter.Next(0.7);

        aPBMap.Clear();
        aMPBFence.Clear();

        using (var aPS = new Message_ProgressScope(aPSOuter.Next(9), "Checking for coinciding edges", aNbPairs))
        {
            for (int i = 0; i < aNbPairs; i++)
            {
                BOPAlgo_EdgeEdge anEdgeEdge = aVEdgeEdge.ChangeValue(i);
                anEdgeEdge.SetProgressRange(aPS.Next());
            }

            // Perform intersection of the found pairs
            BOPTools_Parallel.Perform(myRunParallel, aVEdgeEdge);
        }
        if (UserBreak(aPSOuter))
        {
            return;
        }
        NCollection_DynamicArray<BOPDS_InterfEE> aEEs = myDS.InterfEE();

        // Analyze the results of intersection looking for TopAbs_EDGE
        // intersection type only.

        NCollection_IndexedDataMap<BOPDS_PaveBlock, NCollection_List<BOPDS_PaveBlock>> aMPBLPB = new();
        for (int i = 0; i < aNbPairs; ++i)
        {
            if (UserBreak(aPSOuter))
            {
                return;
            }
            BOPAlgo_EdgeEdge anEdgeEdge = aVEdgeEdge[i];
            if (!anEdgeEdge.IsDone() || anEdgeEdge.HasErrors())
            {
                // Warn about failed intersection of sub-shapes
                TopoDS_Shape aE1 = myDS.Shape(anEdgeEdge.PaveBlock1().OriginalEdge());
                TopoDS_Shape aE2 = myDS.Shape(anEdgeEdge.PaveBlock2().OriginalEdge());
                AddIntersectionFailedWarning(aE1, aE2);
                continue;
            }

            NCollection_Sequence<IntTools_CommonPrt> aCParts = anEdgeEdge.CommonParts();
            if (aCParts.Length() != 1)
            {
                continue;
            }

            IntTools_CommonPrt aCP = aCParts[1];
            if (aCP.Type() != TopAbs_EDGE)
            {
                continue;
            }

            BOPDS_PaveBlock[] aPB = [anEdgeEdge.PaveBlock1(), anEdgeEdge.PaveBlock2()];
            int nE1 = aPB[0].OriginalEdge();
            int nE2 = aPB[1].OriginalEdge();

            if (myDS.Rank(nE1) == myDS.Rank(nE2))
            {
                // Add acquired self-interference warning
                TopoDS_Compound aWC = new();
                new BRep_Builder().MakeCompound(ref aWC);
                new BRep_Builder().Add(aWC, myDS.Shape(nE1));
                new BRep_Builder().Add(aWC, myDS.Shape(nE2));
                AddWarning(new BOPAlgo_AlertAcquiredSelfIntersection(aWC));
            }

            BOPDS_InterfEE aEE = aEEs.Append(new BOPDS_InterfEE());
            aEE.SetIndices(nE1, nE2);
            aEE.SetCommonPart(aCP);
            myDS.AddInterf(nE1, nE2);

            // Fill map for common blocks creation
            for (int j = 0; j < 2; ++j)
            {
                if (myDS.IsCommonBlock(aPB[j]))
                {
                    NCollection_List<BOPDS_PaveBlock> aLPBCB = myDS.CommonBlock(aPB[j])!.PaveBlocks();
                    NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aLPBCB);
                    for (; aItLPB.More(); aItLPB.Next())
                    {
                        BOPAlgo_Tools.FillMap(aPB[j], aItLPB.Value(), aMPBLPB);
                    }
                }
            }
            BOPAlgo_Tools.FillMap(aPB[0], aPB[1], aMPBLPB);
        }
        // Create new common blocks of coinciding pairs.
        BOPAlgo_Tools.PerformCommonBlocks(aMPBLPB, myDS);
    }
}
