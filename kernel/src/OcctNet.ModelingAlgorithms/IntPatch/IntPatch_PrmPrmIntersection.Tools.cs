// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The file-static functions of IntPatch_PrmPrmIntersection.cxx (see IntPatch_PrmPrmIntersection.cs).

using Kerf.Geometry;
using static Occt.IntPatch_IType;
using static Occt.Intf_PIType;

namespace Occt;

public partial class IntPatch_PrmPrmIntersection
{
    // Replaces, for each vertex of theWLline, its point by the nearest vertex of the other walking lines when that
    // vertex is near enough in both parameter spaces (within the walking's maximal steps), removing the neighbouring
    // point where it is more than 10 points away; theWLline becomes the new line.
    private static void SeveralWlinesProcessing(Adaptor3d_Surface theSurf1, Adaptor3d_Surface theSurf2,
                                                NCollection_Sequence<IntPatch_Line> theSLin, double[] thePeriodsArr,
                                                IntSurf_TypeTrans theTrans1, IntSurf_TypeTrans theTrans2,
                                                double theTol, double theMaxStepS1, double theMaxStepS2,
                                                ref IntPatch_WLine theWLline)
    {
        if (theSLin.Length() == 0)
        {
            return;
        }

        double aU1 = 0.0, aV1 = 0.0, aU2 = 0.0, aV2 = 0.0;

        int cnbV = theWLline.NbVertex();
        int ciV;
        for (ciV = 1; ciV <= cnbV; ciV++)
        {
            double pntDMin = 1.0e+100;
            int VDMin = 0;
            int WLDMin = 0;
            Point3D cPV = theWLline.Vertex(ciV).Value();
            theWLline.Vertex(ciV).Parameters(out aU1, out aV1, out aU2, out aV2);
            Point2D aPCS1 = new(aU1, aV1), aPCS2 = new(aU2, aV2);
            int iL;
            for (iL = 1; iL <= theSLin.Length(); iL++)
            {
                IntPatch_Line aSLine = theSLin.Value(iL);
                IntPatch_IType aType = aSLine.ArcType();
                if (aType != IntPatch_Walking)
                {
                    continue;
                }
                IntPatch_WLine aWLine = (aSLine as IntPatch_WLine)!;
                int tnbV = aWLine.NbVertex();
                int tiV;
                for (tiV = 1; tiV <= tnbV; tiV++)
                {
                    Point3D tPV = aWLine.Vertex(tiV).Value();

                    aWLine.Vertex(tiV).Parameters(out aU1, out aV1, out aU2, out aV2);
                    Point2D aPTS1 = new(aU1, aV1), aPTS2 = new(aU2, aV2);

                    double tDistance = cPV.Distance(tPV);
                    double uRs1 = theSurf1.UResolution(tDistance);
                    double vRs1 = theSurf1.VResolution(tDistance);
                    double uRs2 = theSurf2.UResolution(tDistance);
                    double vRs2 = theSurf2.VResolution(tDistance);
                    double RmaxS1 = Max(uRs1, vRs1);
                    double RmaxS2 = Max(uRs2, vRs2);

                    if (RmaxS1 < theMaxStepS1 && RmaxS2 < theMaxStepS2)
                    {
                        if (pntDMin > tDistance && tDistance > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            double aSqDist1 = aPCS1.SquareDistance(aPTS1),
                                   aSqDist2 = aPCS2.SquareDistance(aPTS2);
                            if ((aSqDist1 < RmaxS1 * RmaxS1) && (aSqDist2 < RmaxS2 * RmaxS2))
                            {
                                pntDMin = tDistance;
                                VDMin = tiV;
                                WLDMin = iL;
                            }
                        }
                    }
                }
            }

            if (VDMin != 0)
            {
                IntPatch_Line aSLine = theSLin.Value(WLDMin);
                IntPatch_WLine aWLine = (aSLine as IntPatch_WLine)!;
                int tiVpar = (int)aWLine.Vertex(VDMin).ParameterOnLine();
                int ciVpar = (int)theWLline.Vertex(ciV).ParameterOnLine();
                theWLline.Point(ciVpar).Parameters(out _, out _, out _, out _);
                aWLine.Point(tiVpar).Parameters(out double u21, out double v21, out double u22, out double v22);

                IntSurf_LineOn2S newL2s = new();
                IntSurf_PntOn2S replacePnt = aWLine.Point(tiVpar);
                int cNbP = theWLline.NbPnts();

                NCollection_Sequence<int> VPold = new();
                int iPo;
                for (iPo = 1; iPo <= cnbV; iPo++)
                {
                    double Po = theWLline.Vertex(iPo).ParameterOnLine();
                    int IPo = (int)Po;
                    VPold.Append(IPo);
                }

                bool removeNext = false;
                bool removePrev = false;
                if (ciV == 1)
                {
                    int dPar = Math.Abs(VPold.Value(ciV) - VPold.Value(ciV + 1));
                    if (dPar > 10)
                    {
                        removeNext = true;
                        for (iPo = (ciV + 1); iPo <= cnbV; iPo++)
                        {
                            VPold.SetValue(iPo, VPold.Value(iPo) - 1);
                        }
                    }
                }
                else if (ciV == cnbV)
                {
                    int dPar = Math.Abs(VPold.Value(ciV) - VPold.Value(ciV - 1));
                    if (dPar > 10)
                    {
                        removePrev = true;
                        VPold.SetValue(ciV, VPold.Value(ciV) - 1);
                    }
                }
                else
                {
                    int dParMi = Math.Abs(VPold.Value(ciV) - VPold.Value(ciV - 1));
                    int dParMa = Math.Abs(VPold.Value(ciV) - VPold.Value(ciV + 1));
                    if (dParMi > 10)
                    {
                        removePrev = true;
                        VPold.SetValue(ciV, VPold.Value(ciV) - 1);
                    }

                    if (dParMa > 10)
                    {
                        removeNext = true;
                        for (iPo = (ciV + 1); iPo <= cnbV; iPo++)
                        {
                            if (dParMi > 10)
                            {
                                VPold.SetValue(iPo, VPold.Value(iPo) - 2);
                            }
                            else
                            {
                                VPold.SetValue(iPo, VPold.Value(iPo) - 1);
                            }
                        }
                    }
                    else
                    {
                        if (dParMi > 10)
                        {
                            for (iPo = (ciV + 1); iPo <= cnbV; iPo++)
                            {
                                VPold.SetValue(iPo, VPold.Value(iPo) - 1);
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
                        IntSurf_PntOn2S newPnt = MakeNewPoint(replacePnt, theWLline.Point(iP), thePeriodsArr);
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
                        newL2s.Add(theWLline.Point(iP));
                    }
                }

                IntPatch_Point newVtx = new();
                Point3D Pnt3dV = aWLine.Vertex(VDMin).Value();
                newVtx.SetValue(Pnt3dV, theTol, false);
                newVtx.SetParameters(u21, v21, u22, v22);
                newVtx.SetParameter(VPold.Value(ciV));

                IntPatch_WLine NWLine = new(newL2s, false, theTrans1, theTrans2);
                NWLine.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLPrmPrm);

                int iV;
                for (iV = 1; iV <= cnbV; iV++)
                {
                    if (iV == ciV)
                    {
                        NWLine.AddVertex(newVtx);
                    }
                    else
                    {
                        IntPatch_Point theVtx = theWLline.Vertex(iV);
                        theVtx.SetParameter(VPold.Value(iV));
                        NWLine.AddVertex(theVtx);
                    }
                }

                theWLline = NWLine;
            } // if( VDMin != 0 )
        } // for( ciV = 1; ciV <= cnbV; ciV++ )
    }

    //=======================================================================
    // function : DublicateOfLinesProcessing
    // purpose  : Decides, if rejecting current line is necessary
    //=======================================================================
    private static void DublicateOfLinesProcessing(IntWalk_PWalking thePW, int theWLID,
                                                   NCollection_Sequence<IntPatch_Line> theLines,
                                                   ref bool theIsRejectReq)
    {
        IntPatch_WLine anExistWL = (IntPatch_WLine)theLines.Value(theWLID);
        int aNbPrevPoints = anExistWL.NbPnts();
        int aNbCurrPoints = thePW.NbPoints();

        if (aNbPrevPoints < aNbCurrPoints)
        { // Remove preview line
            theLines.Remove(theWLID);
            theIsRejectReq = false;
        }
        else if (aNbPrevPoints == aNbCurrPoints)
        {
            double aLPrev = 0.0, aLCurr = 0.0;
            for (int aNbPP = 1; aNbPP < aNbPrevPoints; aNbPP++)
            {
                Point3D aP1prev = anExistWL.Point(aNbPP).Value(), aP2prev = anExistWL.Point(aNbPP + 1).Value();
                Point3D aP1curr = thePW.Value(aNbPP).Value(), aP2curr = thePW.Value(aNbPP + 1).Value();

                aLPrev += aP1prev.Distance(aP2prev);
                aLCurr += aP1curr.Distance(aP2curr);
            }

            if (aLPrev < aLCurr)
            { // Remove preview line
                theLines.Remove(theWLID);
                theIsRejectReq = false;
            }
        }
    }

    //=================================================================================================

    // The parameters on both polyhedra of a section point: a vertex's, interpolated on an edge, or barycentric in a
    // face. The parameters of an argument of another type are left unchanged, as OCCT leaves its references.
    private static void SectionPointToParameters(in Intf_SectionPoint Sp, IntPatch_Polyhedron Poly1,
                                                 IntPatch_Polyhedron Poly2, ref double u1, ref double v1,
                                                 ref double u2, ref double v2)
    {
        Intf_PIType typ;
        int Adr1, Adr2;
        double Param, u, v;
        Point3D P = Sp.Pnt();

        int Pt1, Pt2, Pt3;

        Sp.InfoFirst(out typ, out Adr1, out Adr2, out Param);
        switch (typ)
        {
            case Intf_VERTEX: //-- Adr1 est le numero du vertex
            {
                Poly1.Parameters(Adr1, out u1, out v1);
                break;
            }
            case Intf_EDGE:
            {
                Poly1.Parameters(Adr1, out u1, out v1);
                Poly1.Parameters(Adr2, out u, out v);
                u1 += Param * (u - u1);
                v1 += Param * (v - v1);
                break;
            }
            case Intf_FACE:
            {
                double ua, va, ub, vb, uc, vc, ca, cb, cc, cabc;
                Poly1.Triangle(Adr1, out Pt1, out Pt2, out Pt3);
                Point3D PA = Poly1.Point(Pt1);
                Point3D PB = Poly1.Point(Pt2);
                Point3D PC = Poly1.Point(Pt3);
                Poly1.Parameters(Pt1, out ua, out va);
                Poly1.Parameters(Pt2, out ub, out vb);
                Poly1.Parameters(Pt3, out uc, out vc);
                Vector3D Normale = new Vector3D(PA, PB).Crossed(new Vector3D(PA, PC));
                cc = (new Vector3D(PA, PB).Crossed(new Vector3D(PA, P))).Dot(Normale);
                ca = (new Vector3D(PB, PC).Crossed(new Vector3D(PB, P))).Dot(Normale);
                cb = (new Vector3D(PC, PA).Crossed(new Vector3D(PC, P))).Dot(Normale);
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

        Sp.InfoSecond(out typ, out Adr1, out Adr2, out Param);
        switch (typ)
        {
            case Intf_VERTEX: //-- Adr1 est le numero du vertex
            {
                Poly2.Parameters(Adr1, out u2, out v2);
                break;
            }
            case Intf_EDGE:
            {
                Poly2.Parameters(Adr1, out u2, out v2);
                Poly2.Parameters(Adr2, out u, out v);
                u2 += Param * (u - u2);
                v2 += Param * (v - v2);
                break;
            }
            case Intf_FACE:
            {
                double ua, va, ub, vb, uc, vc, ca, cb, cc, cabc;
                Poly2.Triangle(Adr1, out Pt1, out Pt2, out Pt3);
                Point3D PA = Poly2.Point(Pt1);
                Point3D PB = Poly2.Point(Pt2);
                Point3D PC = Poly2.Point(Pt3);
                Poly2.Parameters(Pt1, out ua, out va);
                Poly2.Parameters(Pt2, out ub, out vb);
                Poly2.Parameters(Pt3, out uc, out vc);
                Vector3D Normale = new Vector3D(PA, PB).Crossed(new Vector3D(PA, PC));
                cc = (new Vector3D(PA, PB).Crossed(new Vector3D(PA, P))).Dot(Normale);
                ca = (new Vector3D(PB, PC).Crossed(new Vector3D(PB, P))).Dot(Normale);
                cb = (new Vector3D(PC, PA).Crossed(new Vector3D(PC, P))).Dot(Normale);
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
    private static void AdjustOnPeriodic(Adaptor3d_Surface Surf1, Adaptor3d_Surface Surf2,
                                         NCollection_Sequence<IntPatch_Line> aSLin)
    {
        bool bModified, bIsNull, bIsPeriod;
        int i, j, k, aNbLines, aNbPx, aIndx, aIndq;
        double aEps, du;
        Span<bool> bIsPeriodic = stackalloc bool[4];
        Span<double> aPeriod = stackalloc double[4];
        Span<double> dPeriod = stackalloc double[4];
        Span<double> ux = stackalloc double[4];
        Span<double> uq = stackalloc double[4];
        //
        aEps = global::Kerf.Engine.Foundation.Precision.Confusion();
        //
        for (k = 0; k < 4; ++k)
        {
            aPeriod[k] = 0.0;
        }
        //
        bIsPeriodic[0] = Surf1.IsUPeriodic();
        bIsPeriodic[1] = Surf1.IsVPeriodic();
        bIsPeriodic[2] = Surf2.IsUPeriodic();
        bIsPeriodic[3] = Surf2.IsVPeriodic();
        //
        if (bIsPeriodic[0])
        {
            aPeriod[0] = Surf1.UPeriod();
        }
        if (bIsPeriodic[1])
        {
            aPeriod[1] = Surf1.VPeriod();
        }
        if (bIsPeriodic[2])
        {
            aPeriod[2] = Surf2.UPeriod();
        }
        if (bIsPeriodic[3])
        {
            aPeriod[3] = Surf2.VPeriod();
        }
        //
        for (k = 0; k < 4; ++k)
        {
            dPeriod[k] = 0.25 * aPeriod[k];
        }
        //
        aNbLines = aSLin.Length();
        for (i = 1; i <= aNbLines; ++i)
        {
            IntPatch_WLine aIL = (aSLin.Value(i) as IntPatch_WLine)!;
            IntSurf_LineOn2S aL = aIL.Curve();

            aNbPx = aL.NbPoints();
            if (aNbPx < 10)
            {
                continue;
            }
            //
            for (j = 0; j < 2; ++j)
            {
                bModified = false;
                aIndx = 1;
                aIndq = 2;
                if (j != 0)
                {
                    aIndx = aNbPx;
                    aIndq = aNbPx - 1;
                }
                //
                IntSurf_PntOn2S aPSx = aL.Value(aIndx);
                IntSurf_PntOn2S aPSq = aL.Value(aIndq);
                //
                aPSx.Parameters(out ux[0], out ux[1], out ux[2], out ux[3]);
                aPSq.Parameters(out uq[0], out uq[1], out uq[2], out uq[3]);
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
                    if (Math.Abs(ux[k]) < aEps)
                    {
                        bModified = true;
                        bIsNull = true;
                    }
                    //
                    else if (Math.Abs(ux[k] - aPeriod[k]) < aEps)
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
                                ux[k] = aPeriod[k];
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
                    IntSurf_PntOn2S aPntOn2S;
                    //
                    aPntOn2S = aPSx;
                    aPntOn2S.SetValue(ux[0], ux[1], ux[2], ux[3]);
                    aL.Value(aIndx, aPntOn2S);
                }
            } // for (j=0; j<1; ++j) {
        } // for (i=1; i<=aNbLines; ++i)
    }

    //=================================================================================================

    // The point replacePnt with its parameters moved by a period where they are half a period or more from oldPnt's.
    private static IntSurf_PntOn2S MakeNewPoint(in IntSurf_PntOn2S replacePnt, in IntSurf_PntOn2S oldPnt,
                                                double[] Periods)
    {
        IntSurf_PntOn2S NewPoint = new();
        NewPoint.SetValue(replacePnt.Value());

        Span<double> OldParams = stackalloc double[4];
        Span<double> NewParams = stackalloc double[4];
        oldPnt.Parameters(out OldParams[0], out OldParams[1], out OldParams[2], out OldParams[3]);
        replacePnt.Parameters(out NewParams[0], out NewParams[1], out NewParams[2], out NewParams[3]);

        int i;
        for (i = 0; i < 4; i++)
        {
            if (Periods[i] != 0.0)
            {
                if (Math.Abs(NewParams[i] - OldParams[i]) >= 0.5 * Periods[i])
                {
                    if (NewParams[i] < OldParams[i])
                    {
                        NewParams[i] += Periods[i];
                    }
                    else
                    {
                        NewParams[i] -= Periods[i];
                    }
                }
            }
        }

        NewPoint.SetValue(NewParams[0], NewParams[1], NewParams[2], NewParams[3]);
        return NewPoint;
    }

    //=================================================================================================

    // True if the point is on the walking line: within Deflection of a segment it projects inside, or nearer an end of
    // a segment than the segment's length.
    private static bool IsPointOnLine(in IntSurf_PntOn2S thePOn2S, IntPatch_WLine theWLine, double Deflection)
    {
        bool isOnLine = false;
        double Deflection2 = Deflection * Deflection;

        thePOn2S.Parameters(out double pu1, out double pv1, out double pu2, out double pv2);

        if ((!theWLine.IsOutSurf1Box(new Point2D(pu1, pv1))) && (!theWLine.IsOutSurf2Box(new Point2D(pu2, pv2)))
            && (!theWLine.IsOutBox(thePOn2S.Value())))
        {
            int NbPntOn2SOnLine = theWLine.NbPnts();
            int ll;

            for (ll = 1; ll < NbPntOn2SOnLine && !isOnLine; ll++)
            {
                Point3D Pa = theWLine.Point(ll).Value();
                Point3D Pb = theWLine.Point(ll + 1).Value();
                Point3D PStart = thePOn2S.Value();
                Vector3D AM = new(Pa, PStart);
                Vector3D MB = new(PStart, Pb);
                double AMMB = AM.Dot(MB);

                if (AMMB > 0.0)
                {
                    Direction3D ABN = new(Pb.X() - Pa.X(), Pb.Y() - Pa.Y(), Pb.Z() - Pa.Z());
                    double lan = ABN.X() * AM.X() + ABN.Y() * AM.Y() + ABN.Z() * AM.Z();
                    Vector3D AH = new(lan * ABN.X(), lan * ABN.Y(), lan * ABN.Z());
                    Vector3D HM = new(AM.X() - AH.X(), AM.Y() - AH.Y(), AM.Z() - AH.Z());
                    double d = 0.0;

                    if (HM.X() < Deflection)
                    {
                        d += HM.X() * HM.X();

                        if (HM.Y() < Deflection)
                        {
                            d += HM.Y() * HM.Y();

                            if (HM.Z() < Deflection)
                            {
                                d += HM.Z() * HM.Z();
                            }
                            else
                            {
                                d = Deflection2;
                            }
                        }
                        else
                        {
                            d = Deflection2;
                        }
                    }
                    else
                    {
                        d = Deflection2;
                    }

                    if (d < Deflection2)
                    {
                        isOnLine = true;
                    }
                }
                else
                {
                    double dab = Pa.SquareDistance(Pb);
                    double dap = Pa.SquareDistance(PStart);

                    if (dap < dab)
                    {
                        isOnLine = true;
                    }
                    else
                    {
                        double dbp = Pb.SquareDistance(PStart);

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
    private static void AddWLine(NCollection_Sequence<IntPatch_Line> theLines, IntPatch_WLine theWLine,
                                 double Deflection)
    {
        int i = 1;
        int aNbLines = theLines.Length();
        bool isToRemove;

        // Check each line of theLines if it is on theWLine.
        while (i <= aNbLines)
        {
            IntPatch_WLine? aWLine = theLines.Value(i) as IntPatch_WLine;

            isToRemove = false;

            if (aWLine is not null)
            {
                // Check the middle point.
                int aMidIndex = (aWLine.NbPnts() + 1) / 2;

                if (aMidIndex > 0)
                {
                    IntSurf_PntOn2S aPnt = aWLine.Point(aMidIndex);

                    if (IsPointOnLine(aPnt, theWLine, Deflection))
                    {
                        // Middle point is on theWLine. Check vertices.
                        isToRemove = true;

                        int j;
                        int aNbVtx = aWLine.NbVertex();

                        for (j = 1; j <= aNbVtx; j++)
                        {
                            IntPatch_Point aPoint = aWLine.Vertex(j);

                            if (!IsPointOnLine(aPoint.PntOn2S(), theWLine, Deflection))
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
                theLines.Remove(i);
                aNbLines--;
            }
            else
            {
                i++;
            }
        }

        // Add theWLine to the sequence of lines.
        theLines.Append(theWLine);
    }
}
