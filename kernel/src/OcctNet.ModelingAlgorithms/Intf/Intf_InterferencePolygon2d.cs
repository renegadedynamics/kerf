// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_InterferencePolygon2d.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Intf/Intf_InterferencePolygon2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Intf_PIType;

namespace Occt;

/// <summary>The interference between two polygons, or the self-intersection of a polygon, in two dimensions.</summary>
public class Intf_InterferencePolygon2d : Intf_Interference
{
    // Angular precision (sinus) below that value two right segments
    // are considered as having a potential zone of tangency.
    private static readonly double PRCANG = Precision.Angular();

    private bool oClos;
    private bool tClos;
    private int nbso;

    /// <summary>Constructs an empty interference of polygons.</summary>
    public Intf_InterferencePolygon2d()
        : base(false)
    {
        oClos = false;
        tClos = false;
        nbso = 0;
    }

    /// <summary>Constructs and computes an interference between two polygons.</summary>
    public Intf_InterferencePolygon2d(Intf_Polygon2d Obje1, Intf_Polygon2d Obje2)
        : base(false)
    {
        oClos = false;
        tClos = false;
        nbso = 0;
        if (!Obje1.Bounding().IsOut(Obje2.Bounding()))
        {
            Tolerance = Obje1.DeflectionOverEstimation() + Obje2.DeflectionOverEstimation();
            if (Tolerance == 0.0)
            {
                Tolerance = Epsilon(1000.0);
            }

            nbso = Obje1.NbSegments();
            oClos = Obje1.Closed();
            tClos = Obje2.Closed();
            Interference(Obje1, Obje2);
            Clean();
        }
    }

    /// <summary>Constructs and computes the self-interference of a polygon.</summary>
    public Intf_InterferencePolygon2d(Intf_Polygon2d Obje)
        : base(true)
    {
        oClos = false;
        tClos = false;
        nbso = 0;
        Tolerance = Obje.DeflectionOverEstimation() * 2;
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        oClos = Obje.Closed();
        tClos = oClos;
        Interference(Obje);
        Clean();
    }

    /// <summary>Computes an interference between two polygons.</summary>
    public void Perform(Intf_Polygon2d Obje1, Intf_Polygon2d Obje2)
    {
        SelfInterference(false);
        if (!Obje1.Bounding().IsOut(Obje2.Bounding()))
        {
            Tolerance = Obje1.DeflectionOverEstimation() + Obje2.DeflectionOverEstimation();
            if (Tolerance == 0.0)
            {
                Tolerance = Epsilon(1000.0);
            }

            nbso = Obje1.NbSegments();
            oClos = Obje1.Closed();
            tClos = Obje2.Closed();
            Interference(Obje1, Obje2);
            Clean();
        }
    }

    /// <summary>Computes the self-interference of a polygon.</summary>
    public void Perform(Intf_Polygon2d Obje)
    {
        SelfInterference(true);
        Tolerance = Obje.DeflectionOverEstimation() * 2;
        if (Tolerance == 0.0)
        {
            Tolerance = Epsilon(1000.0);
        }

        oClos = Obje.Closed();
        tClos = oClos;
        Interference(Obje);
        Clean();
    }

    /// <summary>The 2D point of the intersection point of address <paramref name="Index"/> in the interference.</summary>
    public gp_Pnt2d Pnt2dValue(int Index)
    {
        return new gp_Pnt2d((mySPoins[Index]).Pnt().X(), (mySPoins[Index]).Pnt().Y());
    }

    private void Interference(Intf_Polygon2d Obje1, Intf_Polygon2d Obje2)
    {
        Bnd_Box2d bSO = new();
        Bnd_Box2d bST = new();

        int iObje1, iObje2, n1 = nbso, n2 = Obje2.NbSegments();
        double d1 = Obje1.DeflectionOverEstimation(), d2 = Obje2.DeflectionOverEstimation();

        gp_Pnt2d p1b, p1e, p2b, p2e;
        for (iObje1 = 1; iObje1 <= n1; iObje1++)
        {
            bSO.SetVoid();
            Obje1.Segment(iObje1, out p1b, out p1e);
            bSO.Add(p1b);
            bSO.Add(p1e);
            bSO.Enlarge(d1);
            if (!Obje2.Bounding().IsOut(bSO))
            {
                for (iObje2 = 1; iObje2 <= n2; iObje2++)
                {
                    bST.SetVoid();
                    Obje2.Segment(iObje2, out p2b, out p2e);
                    bST.Add(p2b);
                    bST.Add(p2e);
                    bST.Enlarge(d2);
                    if (!bSO.IsOut(bST))
                    {
                        Intersect(iObje1, iObje2, p1b, p1e, p2b, p2e);
                    }
                }
            }
        }
    }

    private void Interference(Intf_Polygon2d Obje)
    {
        Bnd_Box2d bSO = new();
        Bnd_Box2d bST = new();

        int iObje1, iObje2, n = Obje.NbSegments();
        double d = Obje.DeflectionOverEstimation();

        gp_Pnt2d p1b, p1e, p2b, p2e;
        for (iObje1 = 1; iObje1 <= n; iObje1++)
        {
            bSO.SetVoid();
            Obje.Segment(iObje1, out p1b, out p1e);
            bSO.Add(p1b);
            bSO.Add(p1e);
            bSO.Enlarge(d);
            if (!Obje.Bounding().IsOut(bSO))
            {
                for (iObje2 = iObje1 + 1; iObje2 <= n; iObje2++)
                {
                    bST.SetVoid();
                    Obje.Segment(iObje2, out p2b, out p2e);
                    bST.Add(p2b);
                    bST.Add(p2e);
                    bST.Enlarge(d);
                    if (!bSO.IsOut(bST))
                    {
                        Intersect(iObje1, iObje2, p1b, p1e, p2b, p2e);
                    }
                }
            }
        }
    }

    private void Clean()
    {
        // The zones of tangency that concerns only one couple of segments are
        // conserved if the angle between the segments is less than <PRCANG> and
        // if there is no real point of intersection EDGE/EDGE:
        int nbIt = myTZones.Length();
        int decal = 0;
        int addr1, addr2;
        Intf_PIType dim1, dim2;
        double par;
        int tsp, tsps;
        int lpi, ltz;
        // OCCT: set once, before the loop, so a zone keeps the value left by the previous one
        bool Only1Seg = false;

        for (ltz = 1; ltz <= nbIt; ltz++)
        {
            tsp = tsps = 0;
            double pr1mi, pr1ma, pr2mi, pr2ma, delta1, delta2;
            myTZones[ltz - decal].ParamOnFirst(out pr1mi, out pr1ma);
            delta1 = pr1ma - pr1mi;
            myTZones[ltz - decal].ParamOnSecond(out pr2mi, out pr2ma);
            delta2 = pr2ma - pr2mi;
            if (delta1 < 1.0 && delta2 < 1.0)
            {
                Only1Seg = true;
            }

            if (delta1 == 0.0 || delta2 == 0.0)
            {
                Only1Seg = true;
            }

            for (lpi = 1; lpi <= myTZones[ltz - decal].NumberOfPoints(); lpi++)
            {
                Intf_SectionPoint PI1 = myTZones[ltz - decal].GetPoint(lpi);
                if (PI1.Incidence() <= PRCANG)
                {
                    tsp = tsps = 0;
                    break;
                }

                PI1.InfoFirst(out dim1, out addr1, out par);
                PI1.InfoSecond(out dim2, out addr2, out par);
                if (dim1 == Intf_EDGE && dim2 == Intf_EDGE)
                {
                    tsps = 0;
                    if (tsp > 0)
                    {
                        tsp = 0;
                        Only1Seg = false;
                        break;
                    }

                    tsp = lpi;
                }
                else if (dim1 != Intf_EXTERNAL && dim2 != Intf_EXTERNAL)
                {
                    tsps = lpi;
                }
            }

            if (tsp > 0)
            {
                mySPoins.Append(myTZones[ltz - decal].GetPoint(tsp));
                myTZones.Remove(ltz - decal);
                decal++;
            }
            else if (Only1Seg && tsps != 0)
            {
                mySPoins.Append(myTZones[ltz - decal].GetPoint(tsps));
                myTZones.Remove(ltz - decal);
                decal++;
            }
        }

        // The points of intersection located in the tangency zone are
        // removed from the list :
        nbIt = mySPoins.Length();
        decal = 0;

        for (lpi = 1; lpi <= nbIt; lpi++)
        {
            for (ltz = 1; ltz <= myTZones.Length(); ltz++)
            {
                if (myTZones[ltz].RangeContains(mySPoins[lpi - decal]))
                {
                    mySPoins.Remove(lpi - decal);
                    decal++;
                    break;
                }
            }
        }
    }

    /// <summary>Computes the intersection between the two segments <paramref name="BegO"/> <paramref name="EndO"/> and <paramref name="BegT"/> <paramref name="EndT"/>.</summary>
    private void Intersect(int iObje1, int iObje2, in gp_Pnt2d BegO, in gp_Pnt2d EndO, in gp_Pnt2d BegT,
                           in gp_Pnt2d EndT)
    {
        if (SelfIntf)
        {
            if (Math.Abs(iObje1 - iObje2) <= 1)
            {
                return; //-- Ajout du 15 jan 98
            }
        }

        int nbpi = 0;
        Span<double> parO = stackalloc double[8];
        Span<double> parT = stackalloc double[8];
        NCollection_Sequence<Intf_SectionPoint> thePi = new();
        gp_XY segT = EndT.XY() - BegT.XY();
        gp_XY segO = EndO.XY() - BegO.XY();

        // If the length of segment is zero, nothing is done
        double lgT = Math.Sqrt(segT * segT);
        if (lgT <= 0.0)
        {
            return;
        }

        double lgO = Math.Sqrt(segO * segO);
        if (lgO <= 0.0)
        {
            return;
        }

        // Direction of parsing of segments
        double sigPS = (segO * segT) > 0.0 ? 1.0 : -1.0;

        // Precision of calculation
        double floatgap = Epsilon(lgO + lgT);

        // Angle between two straight lines and radius of interference
        double sinTeta = (segO.CrossMagnitude(segT) / lgO) / lgT;
        double rayIntf = 0.0;
        if (sinTeta > 0.0)
        {
            rayIntf = Tolerance / sinTeta;
        }

        // Interference <begO> <segT>
        double dbOT = ((BegO.XY() - BegT.XY()) ^ segT) / lgT;
        double dbObT = BegO.Distance(BegT);
        double dbOeT = BegO.Distance(EndT);
        if (Math.Abs(dbOT) <= Tolerance)
        {
            if (dbObT <= Tolerance)
            {
                nbpi++;
                parO[nbpi] = 0.0;
                parT[nbpi] = 0.0;
                thePi.Append(
                    new Intf_SectionPoint(BegO, Intf_VERTEX, iObje1, 0.0, Intf_VERTEX, iObje2, 0.0, sinTeta));
            }

            if (dbOeT <= Tolerance)
            {
                nbpi++;
                parO[nbpi] = 0.0;
                parT[nbpi] = 1.0;
                thePi.Append(
                    new Intf_SectionPoint(BegO, Intf_VERTEX, iObje1, 0.0, Intf_VERTEX, iObje2 + 1, 0.0, sinTeta));
            }

            if (dbObT > Tolerance && dbOeT > Tolerance && dbObT + dbOeT <= (lgT + Tolerance))
            {
                nbpi++;
                parO[nbpi] = 0.0;
                parT[nbpi] = dbObT / lgT;
                thePi.Append(
                    new Intf_SectionPoint(BegO, Intf_VERTEX, iObje1, 0.0, Intf_EDGE, iObje2, parT[nbpi], sinTeta));
            }
        }

        // Interference <endO> <segT>
        double deOT = ((EndO.XY() - BegT.XY()) ^ segT) / lgT;
        double deObT = EndO.Distance(BegT);
        double deOeT = EndO.Distance(EndT);
        if (Math.Abs(deOT) <= Tolerance)
        {
            if (deObT <= Tolerance)
            {
                nbpi++;
                parO[nbpi] = 1.0;
                parT[nbpi] = 0.0;
                thePi.Append(
                    new Intf_SectionPoint(EndO, Intf_VERTEX, iObje1 + 1, 0.0, Intf_VERTEX, iObje2, 0.0, sinTeta));
            }

            if (deOeT <= Tolerance)
            {
                nbpi++;
                parO[nbpi] = 1.0;
                parT[nbpi] = 1.0;
                thePi.Append(
                    new Intf_SectionPoint(EndO, Intf_VERTEX, iObje1 + 1, 0.0, Intf_VERTEX, iObje2 + 1, 0.0, sinTeta));
            }

            if (deObT > Tolerance && deOeT > Tolerance && deObT + deOeT <= (lgT + Tolerance))
            {
                nbpi++;
                parO[nbpi] = 1.0;
                parT[nbpi] = deObT / lgT;
                thePi.Append(new Intf_SectionPoint(EndO,
                                                   Intf_VERTEX,
                                                   iObje1 + 1,
                                                   0.0,
                                                   Intf_EDGE,
                                                   iObje2,
                                                   parT[nbpi],
                                                   sinTeta));
            }
        }

        // Interference <begT> <segO>
        double dbTO = ((BegT.XY() - BegO.XY()) ^ segO) / lgO;
        if (Math.Abs(dbTO) <= Tolerance)
        {
            if (dbObT > Tolerance && deObT > Tolerance && dbObT + deObT <= (lgO + Tolerance))
            {
                nbpi++;
                parO[nbpi] = dbObT / lgO;
                parT[nbpi] = 0.0;
                thePi.Append(
                    new Intf_SectionPoint(BegT, Intf_EDGE, iObje1, parO[nbpi], Intf_VERTEX, iObje2, 0.0, sinTeta));
            }
        }

        // Interference <endT> <segO>
        double deTO = ((EndT.XY() - BegO.XY()) ^ segO) / lgO;
        if (Math.Abs(deTO) <= Tolerance)
        {
            if (dbOeT > Tolerance && deOeT > Tolerance && dbOeT + deOeT <= (lgO + Tolerance))
            {
                nbpi++;
                parO[nbpi] = dbOeT / lgO;
                parT[nbpi] = 1.0;
                thePi.Append(new Intf_SectionPoint(EndT,
                                                   Intf_EDGE,
                                                   iObje1,
                                                   parO[nbpi],
                                                   Intf_VERTEX,
                                                   iObje2 + 1,
                                                   0.0,
                                                   sinTeta));
            }
        }

        bool edgeSP = false;
        double parOSP = 0, parTSP = 0;

        if (Math.Abs(dbOT - deOT) > floatgap && Math.Abs(dbTO - deTO) > floatgap)
        {
            parOSP = dbOT / (dbOT - deOT);
            parTSP = dbTO / (dbTO - deTO);
            if (dbOT * deOT <= 0.0 && dbTO * deTO <= 0.0)
            {
                edgeSP = true;
            }
            else if (nbpi == 0)
            {
                return;
            }

            // If there is no interference it is necessary to take the points segment by segment
            if (nbpi == 0 && sinTeta > PRCANG)
            {
                nbpi++;
                parO[nbpi] = parOSP;
                parT[nbpi] = parTSP;
                thePi.Append(
                    new Intf_SectionPoint(new gp_Pnt2d(BegO.X() + (segO.X() * parOSP), BegO.Y() + (segO.Y() * parOSP)),
                                          Intf_EDGE,
                                          iObje1,
                                          parOSP,
                                          Intf_EDGE,
                                          iObje2,
                                          parTSP,
                                          sinTeta));
            }

            // Otherwise it is required to check if there is no other
            else if (rayIntf >= Tolerance)
            {
                double deltaO = rayIntf / lgO;
                double deltaT = rayIntf / lgT;
                double x, y;
                double parOdeb = parOSP - deltaO;
                double parOfin = parOSP + deltaO;
                double parTdeb = parTSP - sigPS * deltaT;
                double parTfin = parTSP + sigPS * deltaT;
                if (nbpi == 0)
                {
                    parO[1] = parOdeb;
                    parO[2] = parOfin;
                    parT[1] = parTdeb;
                    parT[2] = parTfin;
                    while (nbpi < 2)
                    {
                        nbpi++;
                        x = BegO.X() + (segO.X() * parO[nbpi]);
                        y = BegO.Y() + (segO.Y() * parO[nbpi]);
                        thePi.Append(new Intf_SectionPoint(new gp_Pnt2d(x, y),
                                                           Intf_EXTERNAL,
                                                           iObje1,
                                                           parO[nbpi],
                                                           Intf_EXTERNAL,
                                                           iObje2,
                                                           parT[nbpi],
                                                           sinTeta));
                    }
                }
                else
                { // nbpi>0
                    if (nbpi == 1)
                    {
                        bool ok = true;
                        if (0.0 < parOdeb && parOdeb < 1.0 && 0.0 < parTdeb && parTdeb < 1.0)
                        {
                            parO[nbpi + 1] = parOdeb;
                            parT[nbpi + 1] = parTdeb;
                        }
                        else if (0.0 < parOfin && parOfin < 1.0 && 0.0 < parTfin && parTfin < 1.0)
                        {
                            parO[nbpi + 1] = parOfin;
                            parT[nbpi + 1] = parTfin;
                        }
                        else
                        {
                            ok = false;
                        }

                        if (ok)
                        {
                            x = BegO.X() + (segO.X() * parO[nbpi + 1]);
                            y = BegO.Y() + (segO.Y() * parO[nbpi + 1]);
                            if (thePi[1].Pnt().Distance(new gp_Pnt(x, y, 0)) >= (Tolerance / 4.0))
                            {
                                nbpi++;
                                thePi.Append(new Intf_SectionPoint(new gp_Pnt2d(x, y),
                                                                   Intf_EXTERNAL,
                                                                   iObje1,
                                                                   parO[nbpi],
                                                                   Intf_EXTERNAL,
                                                                   iObje2,
                                                                   parT[nbpi],
                                                                   sinTeta));
                            }
                        }
                    }
                    else
                    { // plus d une singularite
                        double parOmin = parO[1];
                        double parOmax = parO[1];
                        double parTmin = parT[1];
                        double parTmax = parT[1];
                        for (int i = 2; i <= nbpi; i++)
                        {
                            parOmin = Min(parOmin, parO[i]);
                            parOmax = Max(parOmax, parO[i]);
                            parTmin = Min(parTmin, parT[i]);
                            parTmax = Max(parTmax, parT[i]);
                        }

                        double delta;
                        if (parOdeb < 0.0)
                        {
                            delta = -parOdeb;
                            parOdeb = 0.0;
                            parTdeb = parTdeb + sigPS * (delta * (deltaT / deltaO));
                        }

                        if (parOfin > 1.0)
                        {
                            delta = parOfin - 1.0;
                            parOfin = 1.0;
                            parTfin = parTfin - sigPS * (delta * (deltaT / deltaO));
                        }

                        if (sigPS > 0.0)
                        {
                            if (parTdeb < 0.0)
                            {
                                delta = -parTdeb;
                                parTdeb = 0.0;
                                parOdeb = parOdeb + delta * (deltaO / deltaT);
                            }

                            if (parTfin > 1.0)
                            {
                                delta = parTfin - 1.0;
                                parTfin = 1.0;
                                parOfin = parOfin - delta * (deltaO / deltaT);
                            }
                        }
                        else
                        {
                            if (parTdeb > 1.0)
                            {
                                delta = parTdeb - 1.0;
                                parTdeb = 1.0;
                                parOdeb = parOdeb + delta * (deltaO / deltaT);
                            }

                            if (parTfin < 0.0)
                            {
                                delta = -parTfin;
                                parTfin = 0.0;
                                parOfin = parOfin - delta * (deltaO / deltaT);
                            }
                        }

                        if ((parOdeb < parOmin && parOmin > 0.0)
                            || (sigPS > 0.0 && parTdeb < parTmin && parTmin > 0.0)
                            || (sigPS < 0.0 && parTdeb > parTmax && parTmax < 1.0))
                        {
                            nbpi++;
                            parO[nbpi] = Max(0.0, Min(1.0, parOdeb));
                            parT[nbpi] = Max(0.0, Min(1.0, parTdeb));
                            x = BegO.X() + (segO.X() * parO[nbpi]);
                            y = BegO.Y() + (segO.Y() * parO[nbpi]);
                            thePi.Append(new Intf_SectionPoint(new gp_Pnt2d(x, y),
                                                               Intf_EXTERNAL,
                                                               iObje1,
                                                               parO[nbpi],
                                                               Intf_EXTERNAL,
                                                               iObje2,
                                                               parT[nbpi],
                                                               sinTeta));
                        }

                        if ((parOfin > parOmax && parOmax < 1.0)
                            || (sigPS < 0.0 && parTfin < parTmin && parTmin > 0.0)
                            || (sigPS > 0.0 && parTfin > parTmax && parTmax < 1.0))
                        {
                            nbpi++;
                            parO[nbpi] = Min(1.0, Max(0.0, parOfin));
                            parT[nbpi] = Min(1.0, Max(0.0, parTfin));
                            x = BegO.X() + (segO.X() * parO[nbpi]);
                            y = BegO.Y() + (segO.Y() * parO[nbpi]);
                            thePi.Append(new Intf_SectionPoint(new gp_Pnt2d(x, y),
                                                               Intf_EXTERNAL,
                                                               iObje1,
                                                               parO[nbpi],
                                                               Intf_EXTERNAL,
                                                               iObje2,
                                                               parT[nbpi],
                                                               sinTeta));
                        }
                    }
                }
            }
        }

        //-- lbr : The points too close to each other are suspended
        bool suppr;
        do
        {
            suppr = false;
            for (int i = 2; !suppr && i <= nbpi; i++)
            {
                gp_Pnt Pim1 = thePi[i - 1].Pnt();
                gp_Pnt Pi = thePi[i].Pnt();
                double d = Pi.Distance(Pim1);
                d *= 50.0;
                if (d < lgT && d < lgO)
                {
                    for (int j = i; j < nbpi; j++)
                    {
                        thePi[j] = thePi[j + 1];
                    }

                    nbpi--;
                    suppr = true;
                }
            }
        } while (suppr);

        if (nbpi == 1)
        {
            if (edgeSP)
            {
                thePi[1] =
                    new Intf_SectionPoint(new gp_Pnt2d(BegO.X() + (segO.X() * parOSP), BegO.Y() + (segO.Y() * parOSP)),
                                          Intf_EDGE,
                                          iObje1,
                                          parOSP,
                                          Intf_EDGE,
                                          iObje2,
                                          parTSP,
                                          sinTeta);
                parO[1] = parOSP;
                parT[1] = parTSP;
            }

            if (!SelfIntf)
            {
                bool contains = false;
                for (int i = 1; i <= mySPoins.Length(); i++)
                {
                    if (thePi[1].IsEqual(mySPoins[i]))
                    {
                        contains = true;
                        break;
                    }
                }

                if (!contains)
                {
                    mySPoins.Append(thePi[1]);
                }
            }
            else if (iObje2 - iObje1 != 1 && (!oClos || (iObje1 != 1 && iObje2 != nbso)))
            {
                mySPoins.Append(thePi[1]);
            }
        }
        else if (nbpi >= 2)
        {
            Intf_TangentZone TheTZ = new();
            if (nbpi == 2)
            {
                TheTZ.PolygonInsert(thePi[1]);
                TheTZ.PolygonInsert(thePi[2]);
            }
            else
            {
                int lpj;
                int lmin = 1;
                int lmax = 1;
                for (lpj = 2; lpj <= nbpi; lpj++)
                {
                    if (parO[lpj] < parO[lmin])
                    {
                        lmin = lpj;
                    }
                    else if (parO[lpj] > parO[lmax])
                    {
                        lmax = lpj;
                    }
                }

                TheTZ.PolygonInsert(thePi[lmin]);
                TheTZ.PolygonInsert(thePi[lmax]);

                int ltmin = 1;
                int ltmax = 1;
                for (lpj = 2; lpj <= nbpi; lpj++)
                {
                    if (parT[lpj] < parT[ltmin])
                    {
                        ltmin = lpj;
                    }
                    else if (parT[lpj] > parT[ltmax])
                    {
                        ltmax = lpj;
                    }
                }

                if (ltmin != lmin && ltmin != lmax)
                {
                    TheTZ.PolygonInsert(thePi[ltmin]);
                }

                if (ltmax != lmin && ltmax != lmax)
                {
                    TheTZ.PolygonInsert(thePi[ltmax]);
                }
            }

            if (edgeSP)
            {
                TheTZ.PolygonInsert(
                    new Intf_SectionPoint(new gp_Pnt2d(BegO.X() + (segO.X() * parOSP), BegO.Y() + (segO.Y() * parOSP)),
                                          Intf_EDGE,
                                          iObje1,
                                          parOSP,
                                          Intf_EDGE,
                                          iObje2,
                                          parTSP,
                                          sinTeta));
            }

            int nbtz = myTZones.Length();
            NCollection_List<int> LIndex = new();
            for (int ltz = 1; ltz <= nbtz; ltz++)
            {
                if (TheTZ.HasCommonRange(myTZones[ltz]))
                {
                    LIndex.Append(ltz);
                }
            }

            //------------------------------------------------------------------------
            //--   The list is parsed in ascending order by index, zone and tg
            //--
            if (LIndex.IsEmpty())
            {
                myTZones.Append(TheTZ);
            }
            else
            {
                int indexfirst = LIndex.First();
                LIndex.RemoveFirst();
                int decal = 0;
                myTZones[indexfirst].Append(TheTZ);
                while (!LIndex.IsEmpty())
                {
                    int index = LIndex.First();
                    LIndex.RemoveFirst();
                    myTZones[indexfirst].Append(myTZones[index - decal]);
                    myTZones.Remove(index - decal);
                    decal++;
                }
            }
        }
    }
}
