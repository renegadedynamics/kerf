// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The file-static functions of IntPatch_PrmPrmIntersection.cxx (see IntPatch_PrmPrmIntersection.cs).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Polygons;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Engine.Intersection.Polygons.PIType;

namespace Kerf.Engine.Intersection.Patches;

internal partial class PrmPrmIntersection
{
    // Replaces, for each vertex of theWLline, its point by the nearest vertex of the other walking lines when that
    // vertex is near enough in both parameter spaces (within the walking's maximal steps), removing the neighbouring
    // point where it is more than 10 points away; theWLline becomes the new line.
    private static void SeveralWlinesProcessing(SurfaceAdaptorBase surf1, SurfaceAdaptorBase surf2,
                                                NCollection_Sequence<PatchLine> sLin, double[] periodsArr,
                                                WalkingTypeTrans trans1, WalkingTypeTrans trans2,
                                                double tol, double maxStepS1, double maxStepS2,
                                                ref WLine wLline)
    {
        if (sLin.Length() == 0)
        {
            return;
        }

        double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;

        int cnbV = wLline.VertexCount;
        int ciV;
        for (ciV = 1; ciV <= cnbV; ciV++)
        {
            double pntDMin = 1.0e+100;
            int vdMin = 0;
            int wldMin = 0;
            Point3D cPV = wLline.Vertex(ciV).Value();
            wLline.Vertex(ciV).Parameters(out u1, out v1, out u2, out v2);
            Point2D pcs1 = new(u1, v1), pcs2 = new(u2, v2);
            int iL;
            for (iL = 1; iL <= sLin.Length(); iL++)
            {
                PatchLine sLine = sLin.Value(iL);
                PatchLineType type = sLine.ArcType();
                if (type != PatchLineType.Walking)
                {
                    continue;
                }
                WLine wLine = (sLine as WLine)!;
                int tnbV = wLine.VertexCount;
                int tiV;
                for (tiV = 1; tiV <= tnbV; tiV++)
                {
                    Point3D tPV = wLine.Vertex(tiV).Value();

                    wLine.Vertex(tiV).Parameters(out u1, out v1, out u2, out v2);
                    Point2D pts1 = new(u1, v1), pts2 = new(u2, v2);

                    double tDistance = cPV.DistanceTo(tPV);
                    double uRs1 = surf1.UResolution(tDistance);
                    double vRs1 = surf1.VResolution(tDistance);
                    double uRs2 = surf2.UResolution(tDistance);
                    double vRs2 = surf2.VResolution(tDistance);
                    double rmaxS1 = Max(uRs1, vRs1);
                    double rmaxS2 = Max(uRs2, vRs2);

                    if (rmaxS1 < maxStepS1 && rmaxS2 < maxStepS2)
                    {
                        if (pntDMin > tDistance && tDistance > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            double sqDist1 = pcs1.SquareDistanceTo(pts1),
                                   sqDist2 = pcs2.SquareDistanceTo(pts2);
                            if ((sqDist1 < rmaxS1 * rmaxS1) && (sqDist2 < rmaxS2 * rmaxS2))
                            {
                                pntDMin = tDistance;
                                vdMin = tiV;
                                wldMin = iL;
                            }
                        }
                    }
                }
            }

            if (vdMin != 0)
            {
                PatchLine sLine = sLin.Value(wldMin);
                WLine wLine = (sLine as WLine)!;
                int tiVpar = (int)wLine.Vertex(vdMin).ParameterOnLine();
                int ciVpar = (int)wLline.Vertex(ciV).ParameterOnLine();
                wLline.Point(ciVpar).Parameters(out _, out _, out _, out _);
                wLine.Point(tiVpar).Parameters(out double u21, out double v21, out double u22, out double v22);

                LineOn2S newL2s = new();
                PntOn2S replacePnt = wLine.Point(tiVpar);
                int cNbP = wLline.NbPnts();

                NCollection_Sequence<int> vPold = new();
                int iPo;
                for (iPo = 1; iPo <= cnbV; iPo++)
                {
                    double po = wLline.Vertex(iPo).ParameterOnLine();
                    int IPo = (int)po;
                    vPold.Append(IPo);
                }

                bool removeNext = false;
                bool removePrev = false;
                if (ciV == 1)
                {
                    int dPar = Math.Abs(vPold.Value(ciV) - vPold.Value(ciV + 1));
                    if (dPar > 10)
                    {
                        removeNext = true;
                        for (iPo = (ciV + 1); iPo <= cnbV; iPo++)
                        {
                            vPold.SetValue(iPo, vPold.Value(iPo) - 1);
                        }
                    }
                }
                else if (ciV == cnbV)
                {
                    int dPar = Math.Abs(vPold.Value(ciV) - vPold.Value(ciV - 1));
                    if (dPar > 10)
                    {
                        removePrev = true;
                        vPold.SetValue(ciV, vPold.Value(ciV) - 1);
                    }
                }
                else
                {
                    int dParMi = Math.Abs(vPold.Value(ciV) - vPold.Value(ciV - 1));
                    int dParMa = Math.Abs(vPold.Value(ciV) - vPold.Value(ciV + 1));
                    if (dParMi > 10)
                    {
                        removePrev = true;
                        vPold.SetValue(ciV, vPold.Value(ciV) - 1);
                    }

                    if (dParMa > 10)
                    {
                        removeNext = true;
                        for (iPo = (ciV + 1); iPo <= cnbV; iPo++)
                        {
                            if (dParMi > 10)
                            {
                                vPold.SetValue(iPo, vPold.Value(iPo) - 2);
                            }
                            else
                            {
                                vPold.SetValue(iPo, vPold.Value(iPo) - 1);
                            }
                        }
                    }
                    else
                    {
                        if (dParMi > 10)
                        {
                            for (iPo = (ciV + 1); iPo <= cnbV; iPo++)
                            {
                                vPold.SetValue(iPo, vPold.Value(iPo) - 1);
                            }
                        }
                    }
                }

                int pI = ciVpar;

                int iP;
                for (iP = 1; iP <= cNbP; iP++)
                {
                    if (pI == iP)
                    {
                        PntOn2S newPnt = MakeNewPoint(replacePnt, wLline.Point(iP), periodsArr);
                        newL2s.Add(newPnt);
                    }
                    else if (removeNext && iP == (pI + 1))
                    {
                        continue;
                    }
                    else if (removePrev && iP == (pI - 1))
                    {
                        continue;
                    }
                    else
                    {
                        newL2s.Add(wLline.Point(iP));
                    }
                }

                PatchPoint newVtx = new();
                Point3D pnt3dV = wLine.Vertex(vdMin).Value();
                newVtx.SetValue(pnt3dV, tol, false);
                newVtx.SetParameters(u21, v21, u22, v22);
                newVtx.SetParameter(vPold.Value(ciV));

                WLine nwLine = new(newL2s, false, trans1, trans2);
                nwLine.SetCreatingWayInfo(WLine.IntPatch_WLType.WLPrmPrm);

                int iV;
                for (iV = 1; iV <= cnbV; iV++)
                {
                    if (iV == ciV)
                    {
                        nwLine.AddVertex(newVtx);
                    }
                    else
                    {
                        PatchPoint theVtx = wLline.Vertex(iV);
                        theVtx.SetParameter(vPold.Value(iV));
                        nwLine.AddVertex(theVtx);
                    }
                }

                wLline = nwLine;
            } // if( VDMin != 0 )
        } // for( ciV = 1; ciV <= cnbV; ciV++ )
    }

    //=======================================================================
    // function : DublicateOfLinesProcessing
    // purpose  : Decides, if rejecting current line is necessary
    //=======================================================================
    private static void DublicateOfLinesProcessing(PWalking pw, int wlid,
                                                   NCollection_Sequence<PatchLine> lines,
                                                   ref bool isRejectReq)
    {
        WLine existWL = (WLine)lines.Value(wlid);
        int nbPrevPoints = existWL.NbPnts();
        int nbCurrPoints = pw.PointCount;

        if (nbPrevPoints < nbCurrPoints)
        { // Remove preview line
            lines.Remove(wlid);
            isRejectReq = false;
        }
        else if (nbPrevPoints == nbCurrPoints)
        {
            double lPrev = 0.0, lCurr = 0.0;
            for (int nbPP = 1; nbPP < nbPrevPoints; nbPP++)
            {
                Point3D p1prev = existWL.Point(nbPP).Value(), p2prev = existWL.Point(nbPP + 1).Value();
                Point3D p1curr = pw.Value(nbPP).Value(), p2curr = pw.Value(nbPP + 1).Value();

                lPrev += p1prev.DistanceTo(p2prev);
                lCurr += p1curr.DistanceTo(p2curr);
            }

            if (lPrev < lCurr)
            { // Remove preview line
                lines.Remove(wlid);
                isRejectReq = false;
            }
        }
    }

    //=================================================================================================

    // The parameters on both polyhedra of a section point: a vertex's, interpolated on an edge, or barycentric in a
    // face. The parameters of an argument of another type are left unchanged, as OCCT leaves its references.
    private static void SectionPointToParameters(in SectionPoint sp, Polyhedron poly1,
                                                 Polyhedron poly2, ref double u1, ref double v1,
                                                 ref double u2, ref double v2)
    {
        PIType typ;
        int adr1, adr2;
        double param, u, v;
        Point3D p = sp.Pnt();

        int pt1, pt2, pt3;

        sp.InfoFirst(out typ, out adr1, out adr2, out param);
        switch (typ)
        {
            case PIType.Vertex: //-- Adr1 est le numero du vertex
            {
                poly1.Parameters(adr1, out u1, out v1);
                break;
            }
            case PIType.Edge:
            {
                poly1.Parameters(adr1, out u1, out v1);
                poly1.Parameters(adr2, out u, out v);
                u1 += param * (u - u1);
                v1 += param * (v - v1);
                break;
            }
            case PIType.Face:
            {
                double ua, va, ub, vb, uc, vc, ca, cb, cc, cabc;
                poly1.Triangle(adr1, out pt1, out pt2, out pt3);
                Point3D pa = poly1.Point(pt1);
                Point3D pb = poly1.Point(pt2);
                Point3D pc = poly1.Point(pt3);
                poly1.Parameters(pt1, out ua, out va);
                poly1.Parameters(pt2, out ub, out vb);
                poly1.Parameters(pt3, out uc, out vc);
                Vector3D normale = new Vector3D(pa, pb).Crossed(new Vector3D(pa, pc));
                cc = (new Vector3D(pa, pb).Crossed(new Vector3D(pa, p))).Dot(normale);
                ca = (new Vector3D(pb, pc).Crossed(new Vector3D(pb, p))).Dot(normale);
                cb = (new Vector3D(pc, pa).Crossed(new Vector3D(pc, p))).Dot(normale);
                cabc = ca + cb + cc;

                ca /= cabc;
                cb /= cabc;
                cc /= cabc;

                u1 = ca * ua + cb * ub + cc * uc;
                v1 = ca * va + cb * vb + cc * vc;
                break;
            }
            default:
            {
                //-- std::cout<<" Default dans SectionPointToParameters "<<std::endl;
                break;
            }
        }

        sp.InfoSecond(out typ, out adr1, out adr2, out param);
        switch (typ)
        {
            case PIType.Vertex: //-- Adr1 est le numero du vertex
            {
                poly2.Parameters(adr1, out u2, out v2);
                break;
            }
            case PIType.Edge:
            {
                poly2.Parameters(adr1, out u2, out v2);
                poly2.Parameters(adr2, out u, out v);
                u2 += param * (u - u2);
                v2 += param * (v - v2);
                break;
            }
            case PIType.Face:
            {
                double ua, va, ub, vb, uc, vc, ca, cb, cc, cabc;
                poly2.Triangle(adr1, out pt1, out pt2, out pt3);
                Point3D pa = poly2.Point(pt1);
                Point3D pb = poly2.Point(pt2);
                Point3D pc = poly2.Point(pt3);
                poly2.Parameters(pt1, out ua, out va);
                poly2.Parameters(pt2, out ub, out vb);
                poly2.Parameters(pt3, out uc, out vc);
                Vector3D normale = new Vector3D(pa, pb).Crossed(new Vector3D(pa, pc));
                cc = (new Vector3D(pa, pb).Crossed(new Vector3D(pa, p))).Dot(normale);
                ca = (new Vector3D(pb, pc).Crossed(new Vector3D(pb, p))).Dot(normale);
                cb = (new Vector3D(pc, pa).Crossed(new Vector3D(pc, p))).Dot(normale);
                cabc = ca + cb + cc;

                ca /= cabc;
                cb /= cabc;
                cc /= cabc;

                u2 = ca * ua + cb * ub + cc * uc;
                v2 = ca * va + cb * vb + cc * vc;
                break;
            }
            default:
            {
                //-- std::cout<<" Default dans SectionPointToParameters "<<std::endl;
                break;
            }
        }
    }

    //=================================================================================================

    // Moves the end points of the walking lines (of 10 points or more) that lie on a period's bound (0 or the period,
    // within 1e-7) to the other bound, when the next point is more than a quarter period away.
    private static void AdjustOnPeriodic(SurfaceAdaptorBase surf1, SurfaceAdaptorBase surf2,
                                         NCollection_Sequence<PatchLine> aSLin)
    {
        bool bModified, bIsNull, bIsPeriod;
        int i, j, k, nbLines, nbPx, indx, indq;
        double eps, du;
        Span<bool> bIsPeriodic = stackalloc bool[4];
        Span<double> period = stackalloc double[4];
        Span<double> dPeriod = stackalloc double[4];
        Span<double> ux = stackalloc double[4];
        Span<double> uq = stackalloc double[4];
        //
        eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        //
        for (k = 0; k < 4; ++k)
        {
            period[k] = 0.0;
        }
        //
        bIsPeriodic[0] = surf1.IsUPeriodic;
        bIsPeriodic[1] = surf1.IsVPeriodic;
        bIsPeriodic[2] = surf2.IsUPeriodic;
        bIsPeriodic[3] = surf2.IsVPeriodic;
        //
        if (bIsPeriodic[0])
        {
            period[0] = surf1.UPeriod();
        }
        if (bIsPeriodic[1])
        {
            period[1] = surf1.VPeriod();
        }
        if (bIsPeriodic[2])
        {
            period[2] = surf2.UPeriod();
        }
        if (bIsPeriodic[3])
        {
            period[3] = surf2.VPeriod();
        }
        //
        for (k = 0; k < 4; ++k)
        {
            dPeriod[k] = 0.25 * period[k];
        }
        //
        nbLines = aSLin.Length();
        for (i = 1; i <= nbLines; ++i)
        {
            WLine il = (aSLin.Value(i) as WLine)!;
            LineOn2S l = il.Curve();

            nbPx = l.PointCount;
            if (nbPx < 10)
            {
                continue;
            }
            //
            for (j = 0; j < 2; ++j)
            {
                bModified = false;
                indx = 1;
                indq = 2;
                if (j != 0)
                {
                    indx = nbPx;
                    indq = nbPx - 1;
                }
                //
                PntOn2S pSx = l.Value(indx);
                PntOn2S pSq = l.Value(indq);
                //
                pSx.Parameters(out ux[0], out ux[1], out ux[2], out ux[3]);
                pSq.Parameters(out uq[0], out uq[1], out uq[2], out uq[3]);
                //
                for (k = 0; k < 4; ++k)
                {
                    bIsNull = false;
                    bIsPeriod = false;
                    //
                    if (!bIsPeriodic[k])
                    {
                        continue;
                    }
                    //
                    if (Math.Abs(ux[k]) < eps)
                    {
                        bModified = true;
                        bIsNull = true;
                    }
                    //
                    else if (Math.Abs(ux[k] - period[k]) < eps)
                    {
                        bModified = true;
                        bIsPeriod = true;
                    }
                    //
                    if (bModified)
                    {
                        du = Math.Abs(ux[k] - uq[k]);
                        if (du > dPeriod[k])
                        {
                            if (bIsNull)
                            {
                                ux[k] = period[k];
                            }
                            if (bIsPeriod)
                            {
                                ux[k] = 0.0;
                            }
                        }
                    }
                } // for (k=0; k<4; ++k)
                if (bModified)
                {
                    PntOn2S pntOn2S;
                    //
                    pntOn2S = pSx;
                    pntOn2S.SetValue(ux[0], ux[1], ux[2], ux[3]);
                    l.Value(indx, pntOn2S);
                }
            } // for (j=0; j<1; ++j) {
        } // for (i=1; i<=aNbLines; ++i)
    }

    //=================================================================================================

    // The point replacePnt with its parameters moved by a period where they are half a period or more from oldPnt's.
    private static PntOn2S MakeNewPoint(in PntOn2S replacePnt, in PntOn2S oldPnt,
                                                double[] periods)
    {
        PntOn2S newPoint = new();
        newPoint.SetValue(replacePnt.Value());

        Span<double> oldParams = stackalloc double[4];
        Span<double> newParams = stackalloc double[4];
        oldPnt.Parameters(out oldParams[0], out oldParams[1], out oldParams[2], out oldParams[3]);
        replacePnt.Parameters(out newParams[0], out newParams[1], out newParams[2], out newParams[3]);

        int i;
        for (i = 0; i < 4; i++)
        {
            if (periods[i] != 0.0)
            {
                if (Math.Abs(newParams[i] - oldParams[i]) >= 0.5 * periods[i])
                {
                    if (newParams[i] < oldParams[i])
                    {
                        newParams[i] += periods[i];
                    }
                    else
                    {
                        newParams[i] -= periods[i];
                    }
                }
            }
        }

        newPoint.SetValue(newParams[0], newParams[1], newParams[2], newParams[3]);
        return newPoint;
    }

    //=================================================================================================

    // True if the point is on the walking line: within Deflection of a segment it projects inside, or nearer an end of
    // a segment than the segment's length.
    private static bool IsPointOnLine(in PntOn2S pOn2S, WLine wLine, double deflection)
    {
        bool isOnLine = false;
        double deflection2 = deflection * deflection;

        pOn2S.Parameters(out double pu1, out double pv1, out double pu2, out double pv2);

        if ((!wLine.IsOutSurf1Box(new Point2D(pu1, pv1))) && (!wLine.IsOutSurf2Box(new Point2D(pu2, pv2)))
            && (!wLine.IsOutBox(pOn2S.Value())))
        {
            int nbPntOn2SOnLine = wLine.NbPnts();
            int ll;

            for (ll = 1; ll < nbPntOn2SOnLine && !isOnLine; ll++)
            {
                Point3D pa = wLine.Point(ll).Value();
                Point3D pb = wLine.Point(ll + 1).Value();
                Point3D pStart = pOn2S.Value();
                Vector3D am = new(pa, pStart);
                Vector3D mb = new(pStart, pb);
                double ammb = am.Dot(mb);

                if (ammb > 0.0)
                {
                    Direction3D abn = new(pb.X - pa.X, pb.Y - pa.Y, pb.Z - pa.Z);
                    double lan = abn.X * am.X + abn.Y * am.Y + abn.Z * am.Z;
                    Vector3D ah = new(lan * abn.X, lan * abn.Y, lan * abn.Z);
                    Vector3D hm = new(am.X - ah.X, am.Y - ah.Y, am.Z - ah.Z);
                    double d = 0.0;

                    if (hm.X < deflection)
                    {
                        d += hm.X * hm.X;

                        if (hm.Y < deflection)
                        {
                            d += hm.Y * hm.Y;

                            if (hm.Z < deflection)
                            {
                                d += hm.Z * hm.Z;
                            }
                            else
                            {
                                d = deflection2;
                            }
                        }
                        else
                        {
                            d = deflection2;
                        }
                    }
                    else
                    {
                        d = deflection2;
                    }

                    if (d < deflection2)
                    {
                        isOnLine = true;
                    }
                }
                else
                {
                    double dab = pa.SquareDistanceTo(pb);
                    double dap = pa.SquareDistanceTo(pStart);

                    if (dap < dab)
                    {
                        isOnLine = true;
                    }
                    else
                    {
                        double dbp = pb.SquareDistanceTo(pStart);

                        if (dbp < dab)
                        {
                            isOnLine = true;
                        }
                    }
                }
            }
        }

        return isOnLine;
    }

    //=================================================================================================

    // Removes from theLines the walking lines that lie on theWLine (their middle point and all their vertices), then
    // appends theWLine.
    private static void AddWLine(NCollection_Sequence<PatchLine> lines, WLine wLine,
                                 double deflection)
    {
        int i = 1;
        int nbLines = lines.Length();
        bool isToRemove;

        // Check each line of theLines if it is on theWLine.
        while (i <= nbLines)
        {
            WLine? aWLine = lines.Value(i) as WLine;

            isToRemove = false;

            if (aWLine is not null)
            {
                // Check the middle point.
                int midIndex = (aWLine.NbPnts() + 1) / 2;

                if (midIndex > 0)
                {
                    PntOn2S pnt = aWLine.Point(midIndex);

                    if (IsPointOnLine(pnt, wLine, deflection))
                    {
                        // Middle point is on theWLine. Check vertices.
                        isToRemove = true;

                        int j;
                        int nbVtx = aWLine.VertexCount;

                        for (j = 1; j <= nbVtx; j++)
                        {
                            PatchPoint point = aWLine.Vertex(j);

                            if (!IsPointOnLine(point.PntOn2S(), wLine, deflection))
                            {
                                isToRemove = false;
                                break;
                            }
                        }
                    }
                }
            }

            if (isToRemove)
            {
                lines.Remove(i);
                nbLines--;
            }
            else
            {
                i++;
            }
        }

        // Add theWLine to the sequence of lines.
        lines.Append(wLine);
    }
}
