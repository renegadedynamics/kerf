// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireSplitter_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Booleans;

public partial class WireSplitter
{
    /// <summary>Splits an irregular connexity block of edges on the face into loops (the block's loops).</summary>
    public static void SplitBlock(in Face myFace, ConnexityBlock aCB, IntersectionContext context)
    {
        bool bNothingToDo, bIsClosed, bIsIN;
        int ix, nb, i, cntIn, cntOut;
        double angle;
        Orientation or;
        Iterator itS = new();
        Vertex vv;
        Shape v1 = new();
        NCollection_List<Shape>.Iterator aIt;
        NCollection_List<EdgeInfo>.Iterator itLEI;
        //
        NCollection_IndexedDataMap<Shape, NCollection_List<EdgeInfo>> mySmartMap = new(ShapeHasher.Instance);
        NCollection_DataMap<Shape, bool> vertMap = new(ShapeHasher.Instance);
        //
        NCollection_List<Shape> myEdges = aCB.Shapes();

        NCollection_Map<Shape> ms = new(ShapeHasher.Instance);
        //
        // 1.Filling mySmartMap
        aIt = new NCollection_List<Shape>.Iterator(myEdges);
        for (; aIt.More(); aIt.Next())
        {
            Edge e = ShapeCasts.Edge(aIt.Value());
            if (!AlgoTools2D.HasCurveOnSurface(e, myFace))
            {
                continue;
            }
            //
            bIsClosed = Tool.Degenerated(e) || Tool.IsClosed(e, myFace);

            if (!ms.Add(e) && !bIsClosed)
            {
                ms.Remove(e);
            }

            //
            itS.Initialize(e);
            for (i = 0; itS.More(); itS.Next(), ++i)
            {
                Shape v = itS.Value();
                ix = mySmartMap.FindIndex(v);
                if (ix == 0)
                {
                    NCollection_List<EdgeInfo> leIx = new();
                    ix = mySmartMap.Add(v, leIx);
                }
                //
                NCollection_List<EdgeInfo> lei = mySmartMap[ix];
                EdgeInfo ei = new();
                //
                ei.SetEdge(e);
                or = v.Orientation();
                bIsIN = or == Reversed;
                ei.SetInFlag(bIsIN);
                lei.Append(ei);
                //
                if (i == 0)
                {
                    v1 = v;
                }
                else
                {
                    bIsClosed = bIsClosed || v1.IsSame(v);
                }
                //
                if (vertMap.IsBound(v))
                {
                    if (bIsClosed)
                    {
                        vertMap.ChangeFind(v) = bIsClosed;
                    }
                }
                else
                {
                    vertMap.Bind(v, bIsClosed);
                }
            }
        }
        //
        nb = mySmartMap.Extent();
        //
        bNothingToDo = true;
        for (i = 1; i <= nb; i++)
        {
            cntIn = 0;
            cntOut = 0;
            NCollection_List<EdgeInfo> leInfo = mySmartMap[i];
            NCollection_List<EdgeInfo>.Iterator it = new(leInfo);
            for (; it.More(); it.Next())
            {
                EdgeInfo ei = it.Value();
                if (ei.IsIn)
                {
                    cntIn++;
                }
                else
                {
                    cntOut++;
                }
            }
            if (cntIn != 1 || cntOut != 1)
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
            int nbE, nbMapEE;
            bool bFlag;
            //
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mapEE = new(ShapeHasher.Instance);
            //
            aIt.Initialize(myEdges);
            for (; aIt.More(); aIt.Next())
            {
                Shape e = aIt.Value();
                if (!mapEE.Contains(e))
                {
                    NCollection_List<Shape> lEx = new();
                    lEx.Append(e);
                    mapEE.Add(e, lEx);
                }
                else
                {
                    NCollection_List<Shape> lEx = mapEE.ChangeFromKey(e);
                    lEx.Append(e);
                }
            }
            //
            bFlag = true;
            nbMapEE = mapEE.Extent();
            for (i = 1; i <= nbMapEE; ++i)
            {
                NCollection_List<Shape> lEx = mapEE[i];
                nbE = lEx.Extent();
                if (nbE == 1)
                { // usual case
                    continue;
                }
                else if (nbE == 2)
                {
                    Shape e1 = lEx.First();
                    Shape e2 = lEx.Last();
                    if (e1.IsSame(e2))
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
            NCollection_List<Shape> lecb = aCB.ChangeShapes();
            MakeWire(lecb, out Wire w);
            NCollection_List<Shape> loops = aCB.ChangeLoops();
            loops.Append(w);
            //
            return;
        }
        //
        // 3. Angles in mySmartMap
        FaceSurfaceAdaptor bas = context.SurfaceAdaptor(myFace);
        //
        for (i = 1; i <= nb; i++)
        {
            Vertex v = ShapeCasts.Vertex(mySmartMap.FindKey(i));
            NCollection_List<EdgeInfo> lei = mySmartMap[i];
            itLEI = new NCollection_List<EdgeInfo>.Iterator(lei);
            for (; itLEI.More(); itLEI.Next())
            {
                EdgeInfo ei = itLEI.Value();
                Edge e = ei.Edge();
                ei.SetIsInside(!ms.Contains(e));
                //
                vv = v;
                bIsIN = ei.IsIn;
                or = bIsIN ? Reversed : Forward;
                vv.Orientation(or);
                angle = Angle2D(vv, e, myFace, bas, bIsIN, context);
                ei.SetAngle(angle);
            }
        } // for (i=1; i<=aNb; i++) {
        //
        // Theme: The treatment p-curves convergent in node.
        // The refining the angles of p-curves taking into account
        // bounding curves if exist.
        RefineAngles(myFace, mySmartMap, context);
        //
        // 4. Do
        //
        bool bIsOut, bIsNotPassed;
        NCollection_Sequence<Shape> ls = new(), vertVa = new();
        NCollection_Sequence<Point2D> coordVa = new();
        //
        for (i = 1; i <= nb; ++i)
        {
            Vertex va = ShapeCasts.Vertex(mySmartMap.FindKey(i));
            NCollection_List<EdgeInfo> lei = mySmartMap[i];
            itLEI = new NCollection_List<EdgeInfo>.Iterator(lei);
            for (; itLEI.More(); itLEI.Next())
            {
                EdgeInfo ei = itLEI.Value();
                Edge eOuta = ei.Edge();
                //
                bIsOut = !ei.IsIn;
                bIsNotPassed = !ei.Passed();
                if (bIsOut && bIsNotPassed)
                {
                    //
                    ls.Clear();
                    vertVa.Clear();
                    coordVa.Clear();
                    //
                    Path(bas, myFace, vertMap, va, eOuta, ei, ls, vertVa, coordVa, aCB, mySmartMap);
                }
            }
        } // for (i=1; i<=aNb; ++i) {
    }

    private static void Path(FaceSurfaceAdaptor aGAS,
                             in Face myFace,
                             NCollection_DataMap<Shape, bool> aVertMap,
                             in Vertex aVFirst,
                             in Edge aEFirst,
                             EdgeInfo aEIFirst,
                             NCollection_Sequence<Shape> aLS,
                             NCollection_Sequence<Shape> aVertVa,
                             NCollection_Sequence<Point2D> aCoordVa,
                             ConnexityBlock aCB,
                             NCollection_IndexedDataMap<Shape, NCollection_List<EdgeInfo>> mySmartMap)
    {
        int i, j, nb, nbj;
        double angleIn, angleOut, angle, minAngle;
        double tol2D, tol2D2, d2, twoPI;
        bool isSameV2d, isSameV, isOut, isNotPassed;
        bool bIsClosed;
        Vertex va, vb;
        Edge eOuta;
        NCollection_List<EdgeInfo>.Iterator it;
        double eps = Epsilon(1.0);
        //
        va = aVFirst;
        eOuta = aEFirst;
        EdgeInfo edgeInfo = aEIFirst;
        //
        twoPI = M_PI + M_PI;

        NCollection_Sequence<EdgeInfo> infoSeq = new();
        //
        // append block
        //
        for (;;)
        {
            // Do not escape through edge from which you enter
            nb = aLS.Length();
            if (nb == 1)
            {
                Shape anEPrev = aLS[nb];
                if (anEPrev.IsSame(eOuta))
                {
                    return;
                }
            }
            //
            edgeInfo.SetPassed(true);
            aLS.Append(eOuta);
            aVertVa.Append(va);
            infoSeq.Append(edgeInfo);

            Vertex pVa = va;
            pVa.Orientation(Forward);
            Point2D pa = Coord2d(pVa, eOuta, myFace);
            aCoordVa.Append(pa);

            GetNextVertex(pVa, eOuta, out vb);

            Point2D pb = Coord2d(vb, eOuta, myFace);

            NCollection_List<EdgeInfo> leInfo = mySmartMap.FindFromKey(vb);
            //
            tol2D = 2.0 * Tolerance2D(vb, aGAS);
            tol2D2 = tol2D * tol2D;
            //
            bIsClosed = aVertMap.Find(vb);
            {
                NCollection_List<Shape> buf = new();
                bool bHasEdge = false;
                //
                nb = aLS.Length();
                for (i = nb; i > 0; --i)
                {
                    Shape vPrev = aVertVa[i];
                    Point2D paPrev = aCoordVa[i];
                    Edge ePrev = ShapeCasts.Edge(aLS[i]);
                    //
                    buf.Append(ePrev);
                    if (!bHasEdge)
                    {
                        bHasEdge = !Tool.Degenerated(ePrev);
                        // do not create wire from degenerated edges only
                        if (!bHasEdge)
                        {
                            continue;
                        }
                    }
                    //
                    isSameV = vPrev.IsSame(vb);
                    isSameV2d = isSameV;
                    if (isSameV)
                    {
                        if (bIsClosed)
                        {
                            d2 = paPrev.SquareDistance(pb);
                            isSameV2d = d2 < tol2D2;
                            if (isSameV2d)
                            {
                                double udist = Math.Abs(paPrev.X() - pb.X());
                                double vdist = Math.Abs(paPrev.Y() - pb.Y());
                                double tolU = 2.0 * UTolerance2D(vb, aGAS);
                                double tolV = 2.0 * VTolerance2D(vb, aGAS);
                                //
                                if ((udist > tolU) || (vdist > tolV))
                                {
                                    isSameV2d = false;
                                }
                            }
                        }
                    } // if (anIsSameV) {
                    //
                    if (isSameV && isSameV2d)
                    {
                        int iPriz;
                        iPriz = 1;
                        if (buf.Extent() == 2)
                        {
                            if (buf.First().IsSame(buf.Last()))
                            {
                                iPriz = 0;
                            }
                        }
                        if (iPriz != 0)
                        {
                            MakeWire(buf, out Wire w);
                            aCB.ChangeLoops().Append(w);
                        }
                        //
                        nbj = i - 1;
                        if (nbj < 1)
                        {
                            //
                            aLS.Clear();
                            aVertVa.Clear();
                            aCoordVa.Clear();
                            //
                            return;
                        }
                        //
                        NCollection_Sequence<Shape> lSt = new(), vertVat = new();
                        NCollection_Sequence<Point2D> coordVat = new();
                        NCollection_Sequence<EdgeInfo> infoSeqTmp = new();
                        //
                        vb = ShapeCasts.Vertex(aVertVa[i]);
                        //
                        for (j = 1; j <= nbj; ++j)
                        {
                            lSt.Append(aLS[j]);
                            vertVat.Append(aVertVa[j]);
                            coordVat.Append(aCoordVa[j]);
                            infoSeqTmp.Append(infoSeq[j]);
                        }
                        //
                        // OCCT assigns the caller's sequences: copied into them
                        aLS.Assign(lSt);
                        aVertVa.Assign(vertVat);
                        aCoordVa.Assign(coordVat);
                        infoSeq.Assign(infoSeqTmp);

                        eOuta = ShapeCasts.Edge(aLS.Last());
                        edgeInfo = infoSeq.Last();
                        //
                        break;
                    }
                }
            }
            //
            // aEOutb
            EdgeInfo? pEdgeInfo = null;
            //
            angleIn = AngleIn(eOuta, leInfo);
            minAngle = 100.0;
            int iCnt = NbWaysOut(leInfo);

            bool isBoundary = !edgeInfo.IsInside;
            int nbWaysInside = 0;
            EdgeInfo? pOnlyWayIn = null;

            it = new NCollection_List<EdgeInfo>.Iterator(leInfo);
            for (; it.More(); it.Next())
            {
                EdgeInfo ei = it.Value();
                Edge e = ei.Edge();
                isOut = !ei.IsIn;
                isNotPassed = !ei.Passed();
                //
                if (isOut && isNotPassed)
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
                        pEdgeInfo = ei;
                        break;
                    }
                    //
                    if (e.IsSame(eOuta))
                    {
                        angle = twoPI;
                    }
                    else
                    {
                        // check 2d distance
                        if (bIsClosed)
                        {
                            Point2D p2Dx;
                            //
                            p2Dx = Coord2dVf(e, myFace);
                            //
                            d2 = p2Dx.SquareDistance(pb);
                            if (d2 > tol2D2)
                            {
                                continue;
                            }
                        }
                        //
                        // Look for minimal angle and make the choice.
                        angleOut = ei.Angle();
                        angle = ClockWiseAngle(angleIn, angleOut);
                    }

                    if (isBoundary && ei.IsInside)
                    {
                        ++nbWaysInside;
                        pOnlyWayIn = ei;
                    }

                    if (angle < minAngle - eps)
                    {
                        minAngle = angle;
                        pEdgeInfo = ei;
                    }
                }
            } // for (; anIt.More(); anIt.Next())
            if (nbWaysInside == 1)
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
            va = vb;
            eOuta = pEdgeInfo.Edge();
            edgeInfo = pEdgeInfo;
        }
    }

    private static double ClockWiseAngle(double aAngleIn, double aAngleOut)
    {
        double twoPi = M_PI + M_PI;
        double dA, a1, a2, aIn, aOut;

        aIn = aAngleIn;
        aOut = aAngleOut;
        if (aIn >= twoPi)
        {
            aIn = aIn - twoPi;
        }

        if (aOut >= twoPi)
        {
            aOut = aOut - twoPi;
        }

        a1 = aIn + M_PI;

        if (a1 >= twoPi)
        {
            a1 = a1 - twoPi;
        }

        a2 = aOut;

        dA = a1 - a2;
        if (dA <= 0.0)
        {
            dA = twoPi + dA;
        }
        // xx
        // else if (dA <= 1.e-15) {
        else if (dA <= 1.0e-14)
        {
            dA = twoPi;
        }
        return dA;
    }

    private static Point2D Coord2d(in Vertex aV1, in Edge aE1, in Face aF)
    {
        double t;
        Curve2D? c2D;
        Point2D p2D1;
        //
        t = Tool.Parameter(aV1, aE1, aF);
        c2D = Tool.CurveOnSurface(aE1, aF, out _, out _);
        c2D!.D0(t, out p2D1);
        //
        return p2D1;
    }

    private static Point2D Coord2dVf(in Edge aE, in Face aF)
    {
        double coord = 99.0;
        Point2D p2D1 = new(coord, coord);
        Iterator it = new();
        //
        it.Initialize(aE);
        for (; it.More(); it.Next())
        {
            Shape vx = it.Value();
            if (vx.Orientation() == Forward)
            {
                Vertex vxx = ShapeCasts.Vertex(vx);
                p2D1 = Coord2d(vxx, aE, aF);
                return p2D1;
            }
        }
        return p2D1;
    }

    private static int NbWaysOut(NCollection_List<EdgeInfo> aLEInfo)
    {
        bool bIsOut, bIsNotPassed;
        int iCnt = 0;
        NCollection_List<EdgeInfo>.Iterator it = new(aLEInfo);
        //
        for (; it.More(); it.Next())
        {
            EdgeInfo ei = it.Value();
            //
            bIsOut = !ei.IsIn;
            bIsNotPassed = !ei.Passed();
            if (bIsOut && bIsNotPassed)
            {
                iCnt++;
            }
        }
        return iCnt;
    }

    private static double AngleIn(in Edge aEIn, NCollection_List<EdgeInfo> aLEInfo)
    {
        double angleIn;
        bool isIn;
        NCollection_List<EdgeInfo>.Iterator it = new(aLEInfo);

        for (; it.More(); it.Next())
        {
            EdgeInfo edgeInfo = it.Value();
            Edge e = edgeInfo.Edge();
            isIn = edgeInfo.IsIn;
            //
            if (isIn && e == aEIn)
            {
                angleIn = edgeInfo.Angle();
                return angleIn;
            }
        }
        angleIn = 0.0;
        return angleIn;
    }

    private static void GetNextVertex(in Vertex aV, in Edge aE, out Vertex aV1)
    {
        Iterator it = new();
        //
        it.Initialize(aE);
        for (; it.More(); it.Next())
        {
            Shape vx = it.Value();
            if (!vx.IsEqual(aV))
            {
                aV1 = ShapeCasts.Vertex(vx);
                return;
            }
        }
        aV1 = aV;
    }

    private static double Angle2D(in Vertex aV,
                                  in Edge anEdge,
                                  in Face myFace,
                                  FaceSurfaceAdaptor aGAS,
                                  bool bIsIN,
                                  IntersectionContext context)
    {
        double first, last, dt, tv, tv1, angle, tx;
        Point2D pv, pv1;
        Vector2D v2D;
        Curve2D? c2D;
        //
        tv = Tool.Parameter(aV, anEdge, myFace);
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(tv))
        {
            return 0.0;
        }
        //
        AlgoTools2D.CurveOnSurface(anEdge, myFace, out c2D, out first, out last, out _, context);
        double tol2d = 2.0 * Tolerance2D(aV, aGAS);
        //
        CurveKind type;
        CurveAdaptor2D gac2D = new(c2D!);
        //
        dt = Math.Max(gac2D.Resolution(tol2d), global::Kerf.Engine.Foundation.Precision.PConfusion());
        //
        type = gac2D.GetType();
        if (type != Line)
        {
            CLProps2d lProp = new(c2D!, tv, 2, global::Kerf.Engine.Foundation.Precision.PConfusion());
            if (lProp.IsTangentDefined())
            {
                double r = lProp.Curvature();
                if (r > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    r = 1.0 / r;
                    double cosphi = r / (r + tol2d);
                    dt = Math.Max(dt, Math.Acos(cosphi)); // to avoid small dt for big R.
                }
            }
        }
        // for case chl/927/r9
        tx = 0.05 * (last - first); // aTX=0.25*(aLast - aFirst);
        if (tx < 5.0e-5)
        {
            tx = Math.Min(5.0e-5, (last - first) / 2.0);
        }
        if (dt > tx)
        {
            // to save direction of the curve as much as it possible
            // in the case of big tolerances
            dt = tx;
        }

        if (Math.Abs(tv - first) < Math.Abs(tv - last))
        {
            tv1 = tv + dt;
        }
        else
        {
            tv1 = tv - dt;
        }
        //
        gac2D.D0(tv1, out pv1);
        gac2D.D0(tv, out pv);
        //
        v2D = bIsIN ? new Vector2D(pv1, pv) : new Vector2D(pv, pv1);
        //
        Direction2D dir2D = new(v2D);
        angle = Angle(dir2D);
        //
        return angle;
    }

    private static double Angle(in Direction2D aDir2D)
    {
        Direction2D refDir = new(Direction2D.D.X);
        double angle;

        angle = refDir.Angle(aDir2D);
        if (angle < 0.0)
        {
            angle += M_PI + M_PI;
        }
        return angle;
    }

    private static double Tolerance2D(in Vertex aV, FaceSurfaceAdaptor aGAS)
    {
        double tol2D, ur, vr, tolV3D;
        SurfaceKind type;
        //
        type = aGAS.GetType();
        tolV3D = Tool.Tolerance(aV);

        ur = aGAS.UResolution(tolV3D);
        vr = aGAS.VResolution(tolV3D);
        tol2D = (vr > ur) ? vr : ur;
        //
        if (tol2D < tolV3D)
        {
            tol2D = tolV3D;
        }
        if (type == SurfaceKind.BSplineSurface)
        {
            tol2D = 1.1 * tol2D;
        }
        //
        return tol2D;
    }

    private static double UTolerance2D(in Vertex aV, FaceSurfaceAdaptor aGAS)
    {
        double tolV3D = Tool.Tolerance(aV);
        double ur = aGAS.UResolution(tolV3D);
        //
        return ur;
    }

    private static double VTolerance2D(in Vertex aV, FaceSurfaceAdaptor aGAS)
    {
        double tolV3D = Tool.Tolerance(aV);
        double vr = aGAS.VResolution(tolV3D);
        //
        return vr;
    }

    private static void RefineAngles(in Face myFace,
                                     NCollection_IndexedDataMap<Shape, NCollection_List<EdgeInfo>> mySmartMap,
                                     IntersectionContext context)
    {
        int nb = mySmartMap.Extent();
        for (int i = 1; i <= nb; ++i)
        {
            Vertex v = ShapeCasts.Vertex(mySmartMap.FindKey(i));
            NCollection_List<EdgeInfo> lei = mySmartMap[i];
            RefineAngles(v, myFace, lei, context);
        }
    }

    private static void RefineAngles(in Vertex aV,
                                     in Face myFace,
                                     NCollection_List<EdgeInfo> aLEI,
                                     IntersectionContext context)
    {
        bool bIsIn, bIsBoundary, bRefined;
        int iCntBnd, iCntInt;
        double a, a1, a2;
        NCollection_DataMap<Shape, double> dmsr = new(ShapeHasher.Instance);
        NCollection_List<EdgeInfo>.Iterator itLEI = new(aLEI);
        //
        a1 = 0.0; // angle of outgoing edge
        a2 = 0.0; // angle of incoming edge
        iCntBnd = 0;
        iCntInt = 0;
        itLEI.Initialize(aLEI);
        for (; itLEI.More(); itLEI.Next())
        {
            EdgeInfo ei = itLEI.Value();
            bIsIn = ei.IsIn;
            a = ei.Angle();
            //
            if (!ei.IsInside)
            {
                ++iCntBnd;
                if (!bIsIn)
                {
                    a1 = a;
                }
                else
                {
                    a2 = a;
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
        double delta = ClockWiseAngle(a2, a1);
        itLEI.Initialize(aLEI);
        for (; itLEI.More(); itLEI.Next())
        {
            EdgeInfo ei = itLEI.Value();
            Edge e = ei.Edge();
            //
            bIsBoundary = !ei.IsInside;
            bIsIn = ei.IsIn;
            if (bIsBoundary || bIsIn)
            {
                continue;
            }
            //
            a = ei.Angle();
            double da = ClockWiseAngle(a2, a);
            if (da < delta)
            {
                continue; // already inside
            }
            //
            bRefined = RefineAngle2D(aV, e, myFace, a1, a2, delta, ref a, context);
            if (bRefined)
            {
                dmsr.Bind(e, a);
            }
            else if (iCntInt == 2)
            {
                a = (a <= a1) ? (a1 + global::Kerf.Engine.Foundation.Precision.Angular()) : (a2 - global::Kerf.Engine.Foundation.Precision.Angular());
                dmsr.Bind(e, a);
            }
        }

        if (dmsr.IsEmpty())
        {
            return;
        }
        //
        // update Angles
        itLEI.Initialize(aLEI);
        for (; itLEI.More(); itLEI.Next())
        {
            EdgeInfo ei = itLEI.Value();
            Edge e = ei.Edge();
            //
            bIsIn = ei.IsIn;
            if (!dmsr.IsBound(e))
            {
                continue;
            }
            //
            a = dmsr.Find(e);
            if (bIsIn)
            {
                a = a + M_PI;
            }
            //
            ei.SetAngle(a);
        }
    }

    private static bool RefineAngle2D(in Vertex aV,
                                      in Edge aE,
                                      in Face myFace,
                                      double aA1,
                                      double aA2,
                                      double aDelta,
                                      ref double aA,
                                      IntersectionContext context)
    {
        bool bRet;
        int i, j, nbP;
        double tv, t1, t2, dT, angle, t, tOp;
        double tolInt, ai, xi, yi, t1j, t2j, t1max, t2max, cf;
        Point2D pv, p, p1, p2;
        Curve2D? c2D;
        LineCurve2D li;
        CurveAdaptor2D gac1 = new(), gac2 = new();
        GInter gInter = new();
        TwoDDomain domain1 = new(), domain2 = new();
        //
        bRet = true;
        cf = 0.01;
        tolInt = 1.0e-10;
        //
        AlgoTools2D.CurveOnSurface(aE, myFace, out c2D, out t1, out t2, out _, context);
        gac1.Load(c2D!, t1, t2);
        //
        tv = Tool.Parameter(aV, aE, myFace);
        gac1.D0(tv, out pv);
        //
        tOp = (Math.Abs(tv - t1) < Math.Abs(tv - t2)) ? t2 : t1;
        //
        double maxDT = 0.3 * (t2 - t1);
        gac1.D0(t1, out p1);
        gac1.D0(t2, out p2);
        domain1.SetValues(p1, t1, tolInt, p2, t2, tolInt);
        //
        for (i = 0; i < 2; ++i)
        {
            ai = (i == 0) ? aA1 : (aA2 + M_PI);
            xi = Math.Cos(ai);
            yi = Math.Sin(ai);
            Direction2D diri = new(xi, yi);
            li = new LineCurve2D(pv, diri);
            //
            gac2.Load(li);
            //
            gInter.Perform(gac1, domain1, gac2, domain2, tolInt, tolInt);
            if (!gInter.IsDone)
            {
                continue;
            }
            //
            nbP = gInter.PointCount;
            t1max = tv;
            t2max = -1.0;
            for (j = 1; j <= nbP; ++j)
            {
                IntersectionPoint2D iPj = gInter.Point(j);
                t1j = iPj.ParamOnFirst();
                t2j = iPj.ParamOnSecond();
                //
                if (t2j > t2max && Math.Abs(t1j - tv) < maxDT)
                {
                    t2max = t2j;
                    t1max = t1j;
                }
            }
            //
            if (t2max > 0)
            {
                dT = tOp - t1max;
                if (Math.Abs(dT) < tolInt)
                {
                    continue;
                }
                //
                t = t1max + cf * dT;
                gac1.D0(t, out p);
                Vector2D v2D = new(pv, p);
                Direction2D dir2D = new(v2D);
                //
                angle = Angle(dir2D);
                double da = ClockWiseAngle(aA2, angle);
                if (da < aDelta)
                {
                    aA = angle;
                    return bRet;
                }
            }
        } // for (i=0; i<2; ++i) {
        return !bRet;
    }
}
