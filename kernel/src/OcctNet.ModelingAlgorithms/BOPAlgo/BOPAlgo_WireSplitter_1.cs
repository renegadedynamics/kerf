// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireSplitter_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;

namespace Occt;

public partial class BOPAlgo_WireSplitter
{
    /// <summary>Splits an irregular connexity block of edges on the face into loops (the block's loops).</summary>
    public static void SplitBlock(in TopoDS_Face myFace, BOPTools_ConnexityBlock aCB, IntTools_Context theContext)
    {
        bool bNothingToDo, bIsClosed, bIsIN;
        int aIx, aNb, i, aCntIn, aCntOut;
        double aAngle;
        TopAbs_Orientation aOr;
        TopoDS_Iterator aItS = new();
        TopoDS_Vertex aVV;
        TopoDS_Shape aV1 = new();
        NCollection_List<TopoDS_Shape>.Iterator aIt;
        NCollection_List<BOPAlgo_EdgeInfo>.Iterator aItLEI;
        //
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<BOPAlgo_EdgeInfo>> mySmartMap = new(TopTools_ShapeMapHasher.Instance);
        NCollection_DataMap<TopoDS_Shape, bool> aVertMap = new(TopTools_ShapeMapHasher.Instance);
        //
        NCollection_List<TopoDS_Shape> myEdges = aCB.Shapes();

        NCollection_Map<TopoDS_Shape> aMS = new(TopTools_ShapeMapHasher.Instance);
        //
        // 1.Filling mySmartMap
        aIt = new NCollection_List<TopoDS_Shape>.Iterator(myEdges);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(aIt.Value());
            if (!BOPTools_AlgoTools2D.HasCurveOnSurface(aE, myFace))
            {
                continue;
            }
            //
            bIsClosed = BRep_Tool.Degenerated(aE) || BRep_Tool.IsClosed(aE, myFace);

            if (!aMS.Add(aE) && !bIsClosed)
            {
                aMS.Remove(aE);
            }

            //
            aItS.Initialize(aE);
            for (i = 0; aItS.More(); aItS.Next(), ++i)
            {
                TopoDS_Shape aV = aItS.Value();
                aIx = mySmartMap.FindIndex(aV);
                if (aIx == 0)
                {
                    NCollection_List<BOPAlgo_EdgeInfo> aLEIx = new();
                    aIx = mySmartMap.Add(aV, aLEIx);
                }
                //
                NCollection_List<BOPAlgo_EdgeInfo> aLEI = mySmartMap[aIx];
                BOPAlgo_EdgeInfo aEI = new();
                //
                aEI.SetEdge(aE);
                aOr = aV.Orientation();
                bIsIN = aOr == TopAbs_REVERSED;
                aEI.SetInFlag(bIsIN);
                aLEI.Append(aEI);
                //
                if (i == 0)
                {
                    aV1 = aV;
                }
                else
                {
                    bIsClosed = bIsClosed || aV1.IsSame(aV);
                }
                //
                if (aVertMap.IsBound(aV))
                {
                    if (bIsClosed)
                    {
                        aVertMap.ChangeFind(aV) = bIsClosed;
                    }
                }
                else
                {
                    aVertMap.Bind(aV, bIsClosed);
                }
            }
        }
        //
        aNb = mySmartMap.Extent();
        //
        bNothingToDo = true;
        for (i = 1; i <= aNb; i++)
        {
            aCntIn = 0;
            aCntOut = 0;
            NCollection_List<BOPAlgo_EdgeInfo> aLEInfo = mySmartMap[i];
            NCollection_List<BOPAlgo_EdgeInfo>.Iterator anIt = new(aLEInfo);
            for (; anIt.More(); anIt.Next())
            {
                BOPAlgo_EdgeInfo aEI = anIt.Value();
                if (aEI.IsIn())
                {
                    aCntIn++;
                }
                else
                {
                    aCntOut++;
                }
            }
            if (aCntIn != 1 || aCntOut != 1)
            {
                bNothingToDo = false;
                break;
            }
        }
        //
        // Each vertex has one edge In and one - Out. Good.
        // But it is not enough to consider that nothing to do with this.
        // We must check edges on TShape coincidence.
        // If there are such edges there is something to do with.
        if (bNothingToDo)
        {
            int aNbE, aNbMapEE;
            bool bFlag;
            //
            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMapEE = new(TopTools_ShapeMapHasher.Instance);
            //
            aIt.Initialize(myEdges);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape aE = aIt.Value();
                if (!aMapEE.Contains(aE))
                {
                    NCollection_List<TopoDS_Shape> aLEx = new();
                    aLEx.Append(aE);
                    aMapEE.Add(aE, aLEx);
                }
                else
                {
                    NCollection_List<TopoDS_Shape> aLEx = aMapEE.ChangeFromKey(aE);
                    aLEx.Append(aE);
                }
            }
            //
            bFlag = true;
            aNbMapEE = aMapEE.Extent();
            for (i = 1; i <= aNbMapEE; ++i)
            {
                NCollection_List<TopoDS_Shape> aLEx = aMapEE[i];
                aNbE = aLEx.Extent();
                if (aNbE == 1)
                { // usual case
                    continue;
                }
                else if (aNbE == 2)
                {
                    TopoDS_Shape aE1 = aLEx.First();
                    TopoDS_Shape aE2 = aLEx.Last();
                    if (aE1.IsSame(aE2))
                    {
                        bFlag = false;
                        break;
                    }
                }
                else
                {
                    bFlag = false;
                    break;
                }
            }
            bNothingToDo = bNothingToDo && bFlag;
        } // if (bNothingToDo) {
        if (bNothingToDo)
        {
            NCollection_List<TopoDS_Shape> aLECB = aCB.ChangeShapes();
            MakeWire(aLECB, out TopoDS_Wire aW);
            NCollection_List<TopoDS_Shape> aLoops = aCB.ChangeLoops();
            aLoops.Append(aW);
            //
            return;
        }
        //
        // 3. Angles in mySmartMap
        BRepAdaptor_Surface aBAS = theContext.SurfaceAdaptor(myFace);
        //
        for (i = 1; i <= aNb; i++)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(mySmartMap.FindKey(i));
            NCollection_List<BOPAlgo_EdgeInfo> aLEI = mySmartMap[i];
            aItLEI = new NCollection_List<BOPAlgo_EdgeInfo>.Iterator(aLEI);
            for (; aItLEI.More(); aItLEI.Next())
            {
                BOPAlgo_EdgeInfo aEI = aItLEI.Value();
                TopoDS_Edge aE = aEI.Edge();
                aEI.SetIsInside(!aMS.Contains(aE));
                //
                aVV = aV;
                bIsIN = aEI.IsIn();
                aOr = bIsIN ? TopAbs_REVERSED : TopAbs_FORWARD;
                aVV.Orientation(aOr);
                aAngle = Angle2D(aVV, aE, myFace, aBAS, bIsIN, theContext);
                aEI.SetAngle(aAngle);
            }
        } // for (i=1; i<=aNb; i++) {
        //
        // Theme: The treatment p-curves convergent in node.
        // The refining the angles of p-curves taking into account
        // bounding curves if exist.
        RefineAngles(myFace, mySmartMap, theContext);
        //
        // 4. Do
        //
        bool bIsOut, bIsNotPassed;
        NCollection_Sequence<TopoDS_Shape> aLS = new(), aVertVa = new();
        NCollection_Sequence<Point2D> aCoordVa = new();
        //
        for (i = 1; i <= aNb; ++i)
        {
            TopoDS_Vertex aVa = TopoDS.Vertex(mySmartMap.FindKey(i));
            NCollection_List<BOPAlgo_EdgeInfo> aLEI = mySmartMap[i];
            aItLEI = new NCollection_List<BOPAlgo_EdgeInfo>.Iterator(aLEI);
            for (; aItLEI.More(); aItLEI.Next())
            {
                BOPAlgo_EdgeInfo aEI = aItLEI.Value();
                TopoDS_Edge aEOuta = aEI.Edge();
                //
                bIsOut = !aEI.IsIn();
                bIsNotPassed = !aEI.Passed();
                if (bIsOut && bIsNotPassed)
                {
                    //
                    aLS.Clear();
                    aVertVa.Clear();
                    aCoordVa.Clear();
                    //
                    Path(aBAS, myFace, aVertMap, aVa, aEOuta, aEI, aLS, aVertVa, aCoordVa, aCB, mySmartMap);
                }
            }
        } // for (i=1; i<=aNb; ++i) {
    }

    private static void Path(BRepAdaptor_Surface aGAS,
                             in TopoDS_Face myFace,
                             NCollection_DataMap<TopoDS_Shape, bool> aVertMap,
                             in TopoDS_Vertex aVFirst,
                             in TopoDS_Edge aEFirst,
                             BOPAlgo_EdgeInfo aEIFirst,
                             NCollection_Sequence<TopoDS_Shape> aLS,
                             NCollection_Sequence<TopoDS_Shape> aVertVa,
                             NCollection_Sequence<Point2D> aCoordVa,
                             BOPTools_ConnexityBlock aCB,
                             NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<BOPAlgo_EdgeInfo>> mySmartMap)
    {
        int i, j, aNb, aNbj;
        double anAngleIn, anAngleOut, anAngle, aMinAngle;
        double aTol2D, aTol2D2, aD2, aTwoPI;
        bool anIsSameV2d, anIsSameV, anIsOut, anIsNotPassed;
        bool bIsClosed;
        TopoDS_Vertex aVa, aVb;
        TopoDS_Edge aEOuta;
        NCollection_List<BOPAlgo_EdgeInfo>.Iterator anIt;
        double eps = Epsilon(1.0);
        //
        aVa = aVFirst;
        aEOuta = aEFirst;
        BOPAlgo_EdgeInfo anEdgeInfo = aEIFirst;
        //
        aTwoPI = M_PI + M_PI;

        NCollection_Sequence<BOPAlgo_EdgeInfo> anInfoSeq = new();
        //
        // append block
        //
        for (;;)
        {
            // Do not escape through edge from which you enter
            aNb = aLS.Length();
            if (aNb == 1)
            {
                TopoDS_Shape anEPrev = aLS[aNb];
                if (anEPrev.IsSame(aEOuta))
                {
                    return;
                }
            }
            //
            anEdgeInfo.SetPassed(true);
            aLS.Append(aEOuta);
            aVertVa.Append(aVa);
            anInfoSeq.Append(anEdgeInfo);

            TopoDS_Vertex pVa = aVa;
            pVa.Orientation(TopAbs_FORWARD);
            Point2D aPa = Coord2d(pVa, aEOuta, myFace);
            aCoordVa.Append(aPa);

            GetNextVertex(pVa, aEOuta, out aVb);

            Point2D aPb = Coord2d(aVb, aEOuta, myFace);

            NCollection_List<BOPAlgo_EdgeInfo> aLEInfo = mySmartMap.FindFromKey(aVb);
            //
            aTol2D = 2.0 * Tolerance2D(aVb, aGAS);
            aTol2D2 = aTol2D * aTol2D;
            //
            bIsClosed = aVertMap.Find(aVb);
            {
                NCollection_List<TopoDS_Shape> aBuf = new();
                bool bHasEdge = false;
                //
                aNb = aLS.Length();
                for (i = aNb; i > 0; --i)
                {
                    TopoDS_Shape aVPrev = aVertVa[i];
                    Point2D aPaPrev = aCoordVa[i];
                    TopoDS_Edge aEPrev = TopoDS.Edge(aLS[i]);
                    //
                    aBuf.Append(aEPrev);
                    if (!bHasEdge)
                    {
                        bHasEdge = !BRep_Tool.Degenerated(aEPrev);
                        // do not create wire from degenerated edges only
                        if (!bHasEdge)
                        {
                            continue;
                        }
                    }
                    //
                    anIsSameV = aVPrev.IsSame(aVb);
                    anIsSameV2d = anIsSameV;
                    if (anIsSameV)
                    {
                        if (bIsClosed)
                        {
                            aD2 = aPaPrev.SquareDistance(aPb);
                            anIsSameV2d = aD2 < aTol2D2;
                            if (anIsSameV2d)
                            {
                                double udist = Math.Abs(aPaPrev.X() - aPb.X());
                                double vdist = Math.Abs(aPaPrev.Y() - aPb.Y());
                                double aTolU = 2.0 * UTolerance2D(aVb, aGAS);
                                double aTolV = 2.0 * VTolerance2D(aVb, aGAS);
                                //
                                if ((udist > aTolU) || (vdist > aTolV))
                                {
                                    anIsSameV2d = false;
                                }
                            }
                        }
                    } // if (anIsSameV) {
                    //
                    if (anIsSameV && anIsSameV2d)
                    {
                        int iPriz;
                        iPriz = 1;
                        if (aBuf.Extent() == 2)
                        {
                            if (aBuf.First().IsSame(aBuf.Last()))
                            {
                                iPriz = 0;
                            }
                        }
                        if (iPriz != 0)
                        {
                            MakeWire(aBuf, out TopoDS_Wire aW);
                            aCB.ChangeLoops().Append(aW);
                        }
                        //
                        aNbj = i - 1;
                        if (aNbj < 1)
                        {
                            //
                            aLS.Clear();
                            aVertVa.Clear();
                            aCoordVa.Clear();
                            //
                            return;
                        }
                        //
                        NCollection_Sequence<TopoDS_Shape> aLSt = new(), aVertVat = new();
                        NCollection_Sequence<Point2D> aCoordVat = new();
                        NCollection_Sequence<BOPAlgo_EdgeInfo> anInfoSeqTmp = new();
                        //
                        aVb = TopoDS.Vertex(aVertVa[i]);
                        //
                        for (j = 1; j <= aNbj; ++j)
                        {
                            aLSt.Append(aLS[j]);
                            aVertVat.Append(aVertVa[j]);
                            aCoordVat.Append(aCoordVa[j]);
                            anInfoSeqTmp.Append(anInfoSeq[j]);
                        }
                        //
                        // OCCT assigns the caller's sequences: copied into them
                        aLS.Assign(aLSt);
                        aVertVa.Assign(aVertVat);
                        aCoordVa.Assign(aCoordVat);
                        anInfoSeq.Assign(anInfoSeqTmp);

                        aEOuta = TopoDS.Edge(aLS.Last());
                        anEdgeInfo = anInfoSeq.Last();
                        //
                        break;
                    }
                }
            }
            //
            // aEOutb
            BOPAlgo_EdgeInfo? pEdgeInfo = null;
            //
            anAngleIn = AngleIn(aEOuta, aLEInfo);
            aMinAngle = 100.0;
            int iCnt = NbWaysOut(aLEInfo);

            bool isBoundary = !anEdgeInfo.IsInside();
            int aNbWaysInside = 0;
            BOPAlgo_EdgeInfo? pOnlyWayIn = null;

            anIt = new NCollection_List<BOPAlgo_EdgeInfo>.Iterator(aLEInfo);
            for (; anIt.More(); anIt.Next())
            {
                BOPAlgo_EdgeInfo anEI = anIt.Value();
                TopoDS_Edge aE = anEI.Edge();
                anIsOut = !anEI.IsIn();
                anIsNotPassed = !anEI.Passed();
                //
                if (anIsOut && anIsNotPassed)
                {
                    //
                    // Is there one way to go out of the vertex
                    // we have to use it only.
                    //
                    if (iCnt == 0)
                    {
                        // no way to go . (Error)
                        return;
                    }
                    //
                    if (iCnt == 1)
                    {
                        // the one and only way to go out .
                        pEdgeInfo = anEI;
                        break;
                    }
                    //
                    if (aE.IsSame(aEOuta))
                    {
                        anAngle = aTwoPI;
                    }
                    else
                    {
                        // check 2d distance
                        if (bIsClosed)
                        {
                            Point2D aP2Dx;
                            //
                            aP2Dx = Coord2dVf(aE, myFace);
                            //
                            aD2 = aP2Dx.SquareDistance(aPb);
                            if (aD2 > aTol2D2)
                            {
                                continue;
                            }
                        }
                        //
                        // Look for minimal angle and make the choice.
                        anAngleOut = anEI.Angle();
                        anAngle = ClockWiseAngle(anAngleIn, anAngleOut);
                    }

                    if (isBoundary && anEI.IsInside())
                    {
                        ++aNbWaysInside;
                        pOnlyWayIn = anEI;
                    }

                    if (anAngle < aMinAngle - eps)
                    {
                        aMinAngle = anAngle;
                        pEdgeInfo = anEI;
                    }
                }
            } // for (; anIt.More(); anIt.Next())
            if (aNbWaysInside == 1)
            {
                pEdgeInfo = pOnlyWayIn;
            }
            //
            if (pEdgeInfo is null)
            {
                // no way to go . (Error)
                return;
            }
            //
            aVa = aVb;
            aEOuta = pEdgeInfo.Edge();
            anEdgeInfo = pEdgeInfo;
        }
    }

    private static double ClockWiseAngle(double aAngleIn, double aAngleOut)
    {
        double aTwoPi = M_PI + M_PI;
        double dA, A1, A2, AIn, AOut;

        AIn = aAngleIn;
        AOut = aAngleOut;
        if (AIn >= aTwoPi)
        {
            AIn = AIn - aTwoPi;
        }

        if (AOut >= aTwoPi)
        {
            AOut = AOut - aTwoPi;
        }

        A1 = AIn + M_PI;

        if (A1 >= aTwoPi)
        {
            A1 = A1 - aTwoPi;
        }

        A2 = AOut;

        dA = A1 - A2;
        if (dA <= 0.0)
        {
            dA = aTwoPi + dA;
        }
        // xx
        // else if (dA <= 1.e-15) {
        else if (dA <= 1.0e-14)
        {
            dA = aTwoPi;
        }
        return dA;
    }

    private static Point2D Coord2d(in TopoDS_Vertex aV1, in TopoDS_Edge aE1, in TopoDS_Face aF)
    {
        double aT;
        Geom2d_Curve? aC2D;
        Point2D aP2D1;
        //
        aT = BRep_Tool.Parameter(aV1, aE1, aF);
        aC2D = BRep_Tool.CurveOnSurface(aE1, aF, out _, out _);
        aC2D!.D0(aT, out aP2D1);
        //
        return aP2D1;
    }

    private static Point2D Coord2dVf(in TopoDS_Edge aE, in TopoDS_Face aF)
    {
        double aCoord = 99.0;
        Point2D aP2D1 = new(aCoord, aCoord);
        TopoDS_Iterator aIt = new();
        //
        aIt.Initialize(aE);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aVx = aIt.Value();
            if (aVx.Orientation() == TopAbs_FORWARD)
            {
                TopoDS_Vertex aVxx = TopoDS.Vertex(aVx);
                aP2D1 = Coord2d(aVxx, aE, aF);
                return aP2D1;
            }
        }
        return aP2D1;
    }

    private static int NbWaysOut(NCollection_List<BOPAlgo_EdgeInfo> aLEInfo)
    {
        bool bIsOut, bIsNotPassed;
        int iCnt = 0;
        NCollection_List<BOPAlgo_EdgeInfo>.Iterator anIt = new(aLEInfo);
        //
        for (; anIt.More(); anIt.Next())
        {
            BOPAlgo_EdgeInfo anEI = anIt.Value();
            //
            bIsOut = !anEI.IsIn();
            bIsNotPassed = !anEI.Passed();
            if (bIsOut && bIsNotPassed)
            {
                iCnt++;
            }
        }
        return iCnt;
    }

    private static double AngleIn(in TopoDS_Edge aEIn, NCollection_List<BOPAlgo_EdgeInfo> aLEInfo)
    {
        double anAngleIn;
        bool anIsIn;
        NCollection_List<BOPAlgo_EdgeInfo>.Iterator anIt = new(aLEInfo);

        for (; anIt.More(); anIt.Next())
        {
            BOPAlgo_EdgeInfo anEdgeInfo = anIt.Value();
            TopoDS_Edge aE = anEdgeInfo.Edge();
            anIsIn = anEdgeInfo.IsIn();
            //
            if (anIsIn && aE == aEIn)
            {
                anAngleIn = anEdgeInfo.Angle();
                return anAngleIn;
            }
        }
        anAngleIn = 0.0;
        return anAngleIn;
    }

    private static void GetNextVertex(in TopoDS_Vertex aV, in TopoDS_Edge aE, out TopoDS_Vertex aV1)
    {
        TopoDS_Iterator aIt = new();
        //
        aIt.Initialize(aE);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aVx = aIt.Value();
            if (!aVx.IsEqual(aV))
            {
                aV1 = TopoDS.Vertex(aVx);
                return;
            }
        }
        aV1 = aV;
    }

    private static double Angle2D(in TopoDS_Vertex aV,
                                  in TopoDS_Edge anEdge,
                                  in TopoDS_Face myFace,
                                  BRepAdaptor_Surface aGAS,
                                  bool bIsIN,
                                  IntTools_Context theContext)
    {
        double aFirst, aLast, dt, aTV, aTV1, anAngle, aTX;
        Point2D aPV, aPV1;
        Vector2D aV2D;
        Geom2d_Curve? aC2D;
        //
        aTV = BRep_Tool.Parameter(aV, anEdge, myFace);
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aTV))
        {
            return 0.0;
        }
        //
        BOPTools_AlgoTools2D.CurveOnSurface(anEdge, myFace, out aC2D, out aFirst, out aLast, out _, theContext);
        double tol2d = 2.0 * Tolerance2D(aV, aGAS);
        //
        CurveKind aType;
        Geom2dAdaptor_Curve aGAC2D = new(aC2D!);
        //
        dt = Math.Max(aGAC2D.Resolution(tol2d), global::Kerf.Engine.Foundation.Precision.PConfusion());
        //
        aType = aGAC2D.GetType();
        if (aType != Line)
        {
            GeomLProp_CLProps2d LProp = new(aC2D!, aTV, 2, global::Kerf.Engine.Foundation.Precision.PConfusion());
            if (LProp.IsTangentDefined())
            {
                double R = LProp.Curvature();
                if (R > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    R = 1.0 / R;
                    double cosphi = R / (R + tol2d);
                    dt = Math.Max(dt, Math.Acos(cosphi)); // to avoid small dt for big R.
                }
            }
        }
        // for case chl/927/r9
        aTX = 0.05 * (aLast - aFirst); // aTX=0.25*(aLast - aFirst);
        if (aTX < 5.0e-5)
        {
            aTX = Math.Min(5.0e-5, (aLast - aFirst) / 2.0);
        }
        if (dt > aTX)
        {
            // to save direction of the curve as much as it possible
            // in the case of big tolerances
            dt = aTX;
        }

        if (Math.Abs(aTV - aFirst) < Math.Abs(aTV - aLast))
        {
            aTV1 = aTV + dt;
        }
        else
        {
            aTV1 = aTV - dt;
        }
        //
        aGAC2D.D0(aTV1, out aPV1);
        aGAC2D.D0(aTV, out aPV);
        //
        aV2D = bIsIN ? new Vector2D(aPV1, aPV) : new Vector2D(aPV, aPV1);
        //
        Direction2D aDir2D = new(aV2D);
        anAngle = Angle(aDir2D);
        //
        return anAngle;
    }

    private static double Angle(in Direction2D aDir2D)
    {
        Direction2D aRefDir = new(Direction2D.D.X);
        double anAngle;

        anAngle = aRefDir.Angle(aDir2D);
        if (anAngle < 0.0)
        {
            anAngle += M_PI + M_PI;
        }
        return anAngle;
    }

    private static double Tolerance2D(in TopoDS_Vertex aV, BRepAdaptor_Surface aGAS)
    {
        double aTol2D, anUr, aVr, aTolV3D;
        SurfaceKind aType;
        //
        aType = aGAS.GetType();
        aTolV3D = BRep_Tool.Tolerance(aV);

        anUr = aGAS.UResolution(aTolV3D);
        aVr = aGAS.VResolution(aTolV3D);
        aTol2D = (aVr > anUr) ? aVr : anUr;
        //
        if (aTol2D < aTolV3D)
        {
            aTol2D = aTolV3D;
        }
        if (aType == BSplineSurface)
        {
            aTol2D = 1.1 * aTol2D;
        }
        //
        return aTol2D;
    }

    private static double UTolerance2D(in TopoDS_Vertex aV, BRepAdaptor_Surface aGAS)
    {
        double aTolV3D = BRep_Tool.Tolerance(aV);
        double anUr = aGAS.UResolution(aTolV3D);
        //
        return anUr;
    }

    private static double VTolerance2D(in TopoDS_Vertex aV, BRepAdaptor_Surface aGAS)
    {
        double aTolV3D = BRep_Tool.Tolerance(aV);
        double anVr = aGAS.VResolution(aTolV3D);
        //
        return anVr;
    }

    private static void RefineAngles(in TopoDS_Face myFace,
                                     NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<BOPAlgo_EdgeInfo>> mySmartMap,
                                     IntTools_Context theContext)
    {
        int aNb = mySmartMap.Extent();
        for (int i = 1; i <= aNb; ++i)
        {
            TopoDS_Vertex aV = TopoDS.Vertex(mySmartMap.FindKey(i));
            NCollection_List<BOPAlgo_EdgeInfo> aLEI = mySmartMap[i];
            RefineAngles(aV, myFace, aLEI, theContext);
        }
    }

    private static void RefineAngles(in TopoDS_Vertex aV,
                                     in TopoDS_Face myFace,
                                     NCollection_List<BOPAlgo_EdgeInfo> aLEI,
                                     IntTools_Context theContext)
    {
        bool bIsIn, bIsBoundary, bRefined;
        int iCntBnd, iCntInt;
        double aA, aA1, aA2;
        NCollection_DataMap<TopoDS_Shape, double> aDMSR = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<BOPAlgo_EdgeInfo>.Iterator aItLEI = new(aLEI);
        //
        aA1 = 0.0; // angle of outgoing edge
        aA2 = 0.0; // angle of incoming edge
        iCntBnd = 0;
        iCntInt = 0;
        aItLEI.Initialize(aLEI);
        for (; aItLEI.More(); aItLEI.Next())
        {
            BOPAlgo_EdgeInfo aEI = aItLEI.Value();
            bIsIn = aEI.IsIn();
            aA = aEI.Angle();
            //
            if (!aEI.IsInside())
            {
                ++iCntBnd;
                if (!bIsIn)
                {
                    aA1 = aA;
                }
                else
                {
                    aA2 = aA;
                }
            }
            else
            {
                ++iCntInt;
            }
        }
        //
        if (iCntBnd != 2)
        {
            return;
        }
        //
        double aDelta = ClockWiseAngle(aA2, aA1);
        aItLEI.Initialize(aLEI);
        for (; aItLEI.More(); aItLEI.Next())
        {
            BOPAlgo_EdgeInfo aEI = aItLEI.Value();
            TopoDS_Edge aE = aEI.Edge();
            //
            bIsBoundary = !aEI.IsInside();
            bIsIn = aEI.IsIn();
            if (bIsBoundary || bIsIn)
            {
                continue;
            }
            //
            aA = aEI.Angle();
            double aDA = ClockWiseAngle(aA2, aA);
            if (aDA < aDelta)
            {
                continue; // already inside
            }
            //
            bRefined = RefineAngle2D(aV, aE, myFace, aA1, aA2, aDelta, ref aA, theContext);
            if (bRefined)
            {
                aDMSR.Bind(aE, aA);
            }
            else if (iCntInt == 2)
            {
                aA = (aA <= aA1) ? (aA1 + global::Kerf.Engine.Foundation.Precision.Angular()) : (aA2 - global::Kerf.Engine.Foundation.Precision.Angular());
                aDMSR.Bind(aE, aA);
            }
        }

        if (aDMSR.IsEmpty())
        {
            return;
        }
        //
        // update Angles
        aItLEI.Initialize(aLEI);
        for (; aItLEI.More(); aItLEI.Next())
        {
            BOPAlgo_EdgeInfo aEI = aItLEI.Value();
            TopoDS_Edge aE = aEI.Edge();
            //
            bIsIn = aEI.IsIn();
            if (!aDMSR.IsBound(aE))
            {
                continue;
            }
            //
            aA = aDMSR.Find(aE);
            if (bIsIn)
            {
                aA = aA + M_PI;
            }
            //
            aEI.SetAngle(aA);
        }
    }

    private static bool RefineAngle2D(in TopoDS_Vertex aV,
                                      in TopoDS_Edge aE,
                                      in TopoDS_Face myFace,
                                      double aA1,
                                      double aA2,
                                      double aDelta,
                                      ref double aA,
                                      IntTools_Context theContext)
    {
        bool bRet;
        int i, j, aNbP;
        double aTV, aT1, aT2, dT, aAngle, aT, aTOp;
        double aTolInt, aAi, aXi, aYi, aT1j, aT2j, aT1max, aT2max, aCf;
        Point2D aPV, aP, aP1, aP2;
        Geom2d_Curve? aC2D;
        Geom2d_Line aLi;
        Geom2dAdaptor_Curve aGAC1 = new(), aGAC2 = new();
        Geom2dInt_GInter aGInter = new();
        IntRes2d_Domain aDomain1 = new(), aDomain2 = new();
        //
        bRet = true;
        aCf = 0.01;
        aTolInt = 1.0e-10;
        //
        BOPTools_AlgoTools2D.CurveOnSurface(aE, myFace, out aC2D, out aT1, out aT2, out _, theContext);
        aGAC1.Load(aC2D!, aT1, aT2);
        //
        aTV = BRep_Tool.Parameter(aV, aE, myFace);
        aGAC1.D0(aTV, out aPV);
        //
        aTOp = (Math.Abs(aTV - aT1) < Math.Abs(aTV - aT2)) ? aT2 : aT1;
        //
        double MaxDT = 0.3 * (aT2 - aT1);
        aGAC1.D0(aT1, out aP1);
        aGAC1.D0(aT2, out aP2);
        aDomain1.SetValues(aP1, aT1, aTolInt, aP2, aT2, aTolInt);
        //
        for (i = 0; i < 2; ++i)
        {
            aAi = (i == 0) ? aA1 : (aA2 + M_PI);
            aXi = Math.Cos(aAi);
            aYi = Math.Sin(aAi);
            Direction2D aDiri = new(aXi, aYi);
            aLi = new Geom2d_Line(aPV, aDiri);
            //
            aGAC2.Load(aLi);
            //
            aGInter.Perform(aGAC1, aDomain1, aGAC2, aDomain2, aTolInt, aTolInt);
            if (!aGInter.IsDone())
            {
                continue;
            }
            //
            aNbP = aGInter.NbPoints();
            aT1max = aTV;
            aT2max = -1.0;
            for (j = 1; j <= aNbP; ++j)
            {
                IntRes2d_IntersectionPoint aIPj = aGInter.Point(j);
                aT1j = aIPj.ParamOnFirst();
                aT2j = aIPj.ParamOnSecond();
                //
                if (aT2j > aT2max && Math.Abs(aT1j - aTV) < MaxDT)
                {
                    aT2max = aT2j;
                    aT1max = aT1j;
                }
            }
            //
            if (aT2max > 0)
            {
                dT = aTOp - aT1max;
                if (Math.Abs(dT) < aTolInt)
                {
                    continue;
                }
                //
                aT = aT1max + aCf * dT;
                aGAC1.D0(aT, out aP);
                Vector2D aV2D = new(aPV, aP);
                Direction2D aDir2D = new(aV2D);
                //
                aAngle = Angle(aDir2D);
                double aDA = ClockWiseAngle(aA2, aAngle);
                if (aDA < aDelta)
                {
                    aA = aAngle;
                    return bRet;
                }
            }
        } // for (i=0; i<2; ++i) {
        return !bRet;
    }
}
