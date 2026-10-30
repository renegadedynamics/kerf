// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;
using static Occt.IntPatch_IType;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

// The treatment of the boundaries (OCCT 7.x's IntPatch_ImpImpIntersection_0.gxx): the points of the restrictions
// put on the intersection lines (PutPointsOnLine, MultiplePoint, PointOnSecondDom, FindLine, SingleLine), the
// restriction segments that lie on the other surface (ProcessSegments) and the restriction lines that duplicate
// another line (ProcessRLine).
public sealed partial class IntPatch_ImpImpIntersection
{
    //-- le calcul de dist est completement faux ds la routine ci dessous a revoir (lbr le 18 nov 97)
    private static bool IntersectionWithAnArc(ref gp_Pnt PSurf,
                                              IntPatch_ALine alin,
                                              ref double para,
                                              Adaptor2d_Curve2d thearc,
                                              ref double _theparameteronarc,
                                              ref gp_Pnt thepointonarc,
                                              in IntSurf_Quadric QuadSurf,
                                              double u0alin,
                                              double u1alin)
    {
        double dtheta, theta;
        //-- recherche bete du point le plus proche de thearc->Value(...)
        dtheta = (u1alin - u0alin) * 0.01;
        double du = 0.000000001;
        if (du >= dtheta)
        {
            du = dtheta / 2;
        }
        double distmin = RealLast();

        double thetamin = 0.0;

        double theparameteronarc = _theparameteronarc;
        for (double _theta = u0alin + dtheta; _theta <= u1alin - dtheta; _theta += dtheta)
        {
            gp_Pnt P = alin.Value(_theta);
            double d = P.Distance(PSurf);
            if (d < distmin)
            {
                thetamin = _theta;
                distmin = d;
            }
        }

        double bestpara = 0.0, besttheta = 0.0, bestdist = 0.0, distinit = 0.0;

        //-- Distance initiale
        {
            gp_Pnt pp0 = alin.Value(thetamin);
            double ua0 = 0.0, va0 = 0.0;
            QuadSurf.Parameters(pp0, ref ua0, ref va0);
            thearc.D1(theparameteronarc, out gp_Pnt2d p2d, out gp_Vec2d d2d);
            gp_Vec2d PaPr = new(new gp_Pnt2d(ua0, va0), p2d);
            distinit = PaPr.Magnitude();
        }
        theta = thetamin;
        //-- recherche a partir de theta et theparameteronarc
        bool cpasok = true;
        int nbiter = 0;
        double drmax = (thearc.LastParameter() - thearc.FirstParameter()) * 0.05;
        double damax = (u1alin - u0alin) * 0.05;

        bestdist = RealLast();

        do
        {
            double ua0 = 0.0, va0 = 0.0, ua1 = 0.0, va1 = 0.0;
            gp_Pnt pp0 = alin.Value(theta);
            gp_Pnt pp1 = alin.Value(theta + du);
            QuadSurf.Parameters(pp0, ref ua0, ref va0);
            QuadSurf.Parameters(pp1, ref ua1, ref va1);

            gp_Vec2d D1a = new((ua1 - ua0) / du, (va1 - va0) / du);
            thearc.D1(theparameteronarc, out gp_Pnt2d p2d, out gp_Vec2d d2d);
            gp_Vec2d PaPr = new(new gp_Pnt2d(ua0, va0), p2d);

            double pbd = PaPr.Magnitude();
            if (bestdist > pbd)
            {
                bestdist = pbd;
                bestpara = theparameteronarc;
                besttheta = theta;
            }

            D1a.SetCoord(-D1a.X(), -D1a.Y());

            double d = D1a.X() * d2d.Y() - D1a.Y() * d2d.X();

            double da = (-PaPr.X()) * d2d.Y() - (-PaPr.Y()) * d2d.X();
            double dr = D1a.X() * (-PaPr.Y()) - D1a.Y() * (-PaPr.X());
            if (Math.Abs(d) > 1e-15)
            {
                da /= d;
                dr /= d;
            }
            else
            {
                if (Math.Abs(PaPr.X()) > Math.Abs(PaPr.Y()))
                {
                    double xx = PaPr.X();
                    xx *= 0.5;
                    if (D1a.X() != 0.0)
                    {
                        da = -xx / D1a.X();
                    }
                    if (d2d.X() != 0.0)
                    {
                        dr = -xx / d2d.X();
                    }
                }
                else
                {
                    double yy = PaPr.Y();
                    yy *= 0.5;
                    if (D1a.Y() != 0.0)
                    {
                        da = -yy / D1a.Y();
                    }
                    if (d2d.Y() != 0.0)
                    {
                        dr = -yy / d2d.Y();
                    }
                }
            }

            if (da < -damax)
            {
                da = -damax;
            }
            else if (da > damax)
            {
                da = damax;
            }
            if (dr < -drmax)
            {
                dr = -drmax;
            }
            else if (dr > drmax)
            {
                dr = drmax;
            }

            if (Math.Abs(da) < 1e-10 && Math.Abs(dr) < 1e-10)
            {
                para = theta;
                PSurf = alin.Value(para);
                _theparameteronarc = theparameteronarc;
                thepointonarc = alin.Value(para);
                return (true);
            }
            else
            {
                theta += da;
                theparameteronarc += dr;
                if (theparameteronarc > thearc.LastParameter())
                {
                    theparameteronarc = thearc.LastParameter();
                }
                if (theparameteronarc < thearc.FirstParameter())
                {
                    theparameteronarc = thearc.FirstParameter();
                }
                if (theta < u0alin)
                {
                    theta = u0alin;
                }
                if (theta > u1alin - du)
                {
                    theta = u1alin - du - du;
                }
            }
            nbiter++;
        } while (cpasok && nbiter < 20);
        if (bestdist < distinit)
        {
            para = besttheta;
            PSurf = alin.Value(para);
            _theparameteronarc = bestpara;
            thepointonarc = alin.Value(para);
            return (true);
        }
        return (false);
    }

    //-- ======================================================================
    private static void Recadre(Adaptor3d_Surface myHS1,
                                Adaptor3d_Surface myHS2,
                                ref double u1,
                                ref double v1,
                                ref double u2,
                                ref double v2)
    {
        double f, l, lmf, fpls2;
        GeomAbs_SurfaceType typs1 = myHS1.GetType();
        GeomAbs_SurfaceType typs2 = myHS2.GetType();

        bool myHS1IsUPeriodic, myHS1IsVPeriodic;
        switch (typs1)
        {
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
            case GeomAbs_Sphere:
                {
                    myHS1IsUPeriodic = true;
                    myHS1IsVPeriodic = false;
                    break;
                }
            case GeomAbs_Torus:
                {
                    myHS1IsUPeriodic = myHS1IsVPeriodic = true;
                    break;
                }
            default:
                {
                    //-- Le cas de biparametrees periodiques est gere en amont
                    myHS1IsUPeriodic = myHS1IsVPeriodic = false;
                    break;
                }
        }

        bool myHS2IsUPeriodic, myHS2IsVPeriodic;
        switch (typs2)
        {
            case GeomAbs_Cylinder:
            case GeomAbs_Cone:
            case GeomAbs_Sphere:
                {
                    myHS2IsUPeriodic = true;
                    myHS2IsVPeriodic = false;
                    break;
                }
            case GeomAbs_Torus:
                {
                    myHS2IsUPeriodic = myHS2IsVPeriodic = true;
                    break;
                }
            default:
                {
                    //-- Le cas de biparametrees periodiques est gere en amont
                    myHS2IsUPeriodic = myHS2IsVPeriodic = false;
                    break;
                }
        }
        if (myHS1IsUPeriodic)
        {
            lmf = M_PI + M_PI; //-- myHS1->UPeriod();
            f = myHS1.FirstUParameter();
            l = myHS1.LastUParameter();
            fpls2 = 0.5 * (f + l);
            while ((u1 < f) && ((fpls2 - u1) > (u1 + lmf - fpls2)))
            {
                u1 += lmf;
            }
            while ((u1 > l) && ((u1 - fpls2) > (fpls2 - (u1 - lmf))))
            {
                u1 -= lmf;
            }
        }
        if (myHS1IsVPeriodic)
        {
            lmf = M_PI + M_PI; //-- myHS1->VPeriod();
            f = myHS1.FirstVParameter();
            l = myHS1.LastVParameter();
            fpls2 = 0.5 * (f + l);
            while ((v1 < f) && ((fpls2 - v1) > (v1 + lmf - fpls2)))
            {
                v1 += lmf;
            }
            while ((v1 > l) && ((v1 - fpls2) > (fpls2 - (v1 - lmf))))
            {
                v1 -= lmf;
            }
        }
        if (myHS2IsUPeriodic)
        {
            lmf = M_PI + M_PI; //-- myHS2->UPeriod();
            f = myHS2.FirstUParameter();
            l = myHS2.LastUParameter();
            fpls2 = 0.5 * (f + l);
            while ((u2 < f) && ((fpls2 - u2) > (u2 + lmf - fpls2)))
            {
                u2 += lmf;
            }
            while ((u2 > l) && ((u2 - fpls2) > (fpls2 - (u2 - lmf))))
            {
                u2 -= lmf;
            }
        }
        if (myHS2IsVPeriodic)
        {
            lmf = M_PI + M_PI; //-- myHS2->VPeriod();
            f = myHS2.FirstVParameter();
            l = myHS2.LastVParameter();
            fpls2 = 0.5 * (f + l);
            while ((v2 < f) && ((fpls2 - v2) > (v2 + lmf - fpls2)))
            {
                v2 += lmf;
            }
            while ((v2 > l) && ((v2 - fpls2) > (fpls2 - (v2 - lmf))))
            {
                v2 -= lmf;
            }
        }
    }

    // The vertex count, a vertex, and the vertex operations of a GLine or ALine (OCCT down-casts by ArcType).
    private static int NbVertexOf(IntPatch_Line theLine, IntPatch_IType theType) =>
        theType == IntPatch_Analytic ? ((IntPatch_ALine)theLine).NbVertex() : ((IntPatch_GLine)theLine).NbVertex();

    private static IntPatch_Point VertexOf(IntPatch_Line theLine, IntPatch_IType theType, int theIndex) =>
        theType == IntPatch_Analytic
            ? ((IntPatch_ALine)theLine).Vertex(theIndex)
            : ((IntPatch_GLine)theLine).Vertex(theIndex);

    private static void AddVertexTo(IntPatch_Line theLine, IntPatch_IType theType, in IntPatch_Point thePnt)
    {
        if (theType == IntPatch_Analytic)
        {
            ((IntPatch_ALine)theLine).AddVertex(thePnt);
        }
        else
        {
            ((IntPatch_GLine)theLine).AddVertex(thePnt);
        }
    }

    private static void ReplaceVertexOf(IntPatch_Line theLine, IntPatch_IType theType, int theIndex,
                                        in IntPatch_Point thePnt)
    {
        if (theType == IntPatch_Analytic)
        {
            ((IntPatch_ALine)theLine).Replace(theIndex, thePnt);
        }
        else
        {
            ((IntPatch_GLine)theLine).Replace(theIndex, thePnt);
        }
    }

    //=================================================================================================

    private static void PutPointsOnLine(Adaptor3d_Surface S1,
                                        Adaptor3d_Surface S2,
                                        NCollection_Sequence<IntPatch_ThePathPointOfTheSOnBounds> listpnt,
                                        NCollection_Sequence<IntPatch_Line> slin,
                                        bool OnFirst,
                                        Adaptor3d_TopolTool Domain,
                                        in IntSurf_Quadric QuadSurf,
                                        in IntSurf_Quadric OtherQuad,
                                        bool multpoint,
                                        double Tolarc)
    {
        // Traitement des point (de listpnt) de depart. On les replace sur
        // la ligne d intersection, en leur affectant la transition correcte sur
        // cette ligne.
        int nbpnt = listpnt.Length();
        int nblin = slin.Length();

        if (slin.Length() == 0 || nbpnt == 0)
        {
            return;
        }
        //
        int i, k;
        int linenumber;
        double currentparameter, tolerance;
        double U1, V1, U2 = 0.0, V2 = 0.0;
        bool goon;

        gp_Pnt Psurf, ptbid = new();
        gp_Vec Normale = new(), Vtgint = new(), Vtgrst = new();

        gp_Vec d1u = new(), d1v = new();
        gp_Pnt2d p2d;
        gp_Vec2d d2d;

        IntSurf_Transition Transline = new(), Transarc = new();

        Adaptor2d_Curve2d currentarc;
        Adaptor3d_HVertex? vtx = null, vtxbis;

        IntPatch_Point solpnt = new();
        IntPatch_ThePathPointOfTheSOnBounds currentpointonrst;
        IntPatch_IType TheType;

        NCollection_Array1<int> UsedLine = new(1, nblin);
        NCollection_Array1<int> Done = new(1, nbpnt);
        for (i = 1; i <= nbpnt; i++)
        {
            Done[i] = 0; //-- Initialisation a la main
        }

        for (i = 1; i <= nbpnt; i++)
        {
            if (Done[i] != 1)
            {
                currentpointonrst = listpnt.Value(i);
                Psurf = currentpointonrst.Value(); // Point dans l espace
                tolerance = currentpointonrst.Tolerance();

                // On recherche d abord si on a correspondance avec un "point multiple"
                UsedLine.Init(0);

                goon = true;
                if (multpoint)
                {
                    Normale = QuadSurf.Normale(Psurf); // Normale a la surface au point
                    currentarc = currentpointonrst.Arc();
                    currentparameter = currentpointonrst.Parameter();
                    currentarc.D1(currentparameter, out p2d, out d2d);
                    QuadSurf.D1(p2d.X(), p2d.Y(), ref ptbid, ref d1u, ref d1v);
                    Vtgrst.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
                    goon = MultiplePoint(listpnt,
                                         Domain,
                                         QuadSurf,
                                         Normale,
                                         slin,
                                         Done,
                                         UsedLine,
                                         i,
                                         OnFirst,
                                         Tolarc);
                }
                if (goon)
                {
                    bool linefound = false;

                    for (int indiceline = 1; indiceline <= slin.Length(); indiceline++)
                    {
                        if (UsedLine[indiceline] != 0)
                        {
                            continue;
                        }
                        linenumber = indiceline;

                        //-- Attention , les points peuvent etre deplaces
                        //-- il faut reprendre le point original
                        currentpointonrst = listpnt.Value(i);
                        currentarc = currentpointonrst.Arc();
                        currentparameter = currentpointonrst.Parameter();
                        Psurf = currentpointonrst.Value(); // Point dans l espace
                        tolerance = currentpointonrst.Tolerance();
                        //--

                        //  Modified by skv - Thu Jan 15 15:57:15 2004 OCC4455 Begin
                        if (!currentpointonrst.IsNew())
                        {
                            Adaptor3d_HVertex aVtx = currentpointonrst.Vertex();
                            double aVtxTol = aVtx.Resolution(currentarc);
                            double aTolAng = 0.01 * tolerance;

                            tolerance = Max(tolerance, aVtxTol);

                            gp_Vec aNorm1 = QuadSurf.Normale(Psurf);
                            gp_Vec aNorm2 = OtherQuad.Normale(Psurf);
                            //
                            if (aNorm1.Magnitude() > gp.Resolution() && aNorm2.Magnitude() > gp.Resolution())
                            {
                                if (aNorm1.IsParallel(aNorm2, aTolAng))
                                {
                                    tolerance = Math.Sqrt(tolerance);
                                }
                            } //
                        }
                        //  Modified by skv - Thu Jan 15 15:57:15 2004 OCC4455 End
                        gp_Pnt pointonarc = new();
                        Vtgint.SetCoord(0, 0, 0);
                        double aVertTol = Tolarc;
                        NCollection_List<double> aLParams = new();
                        linefound = FindLine(ref Psurf,
                                             slin,
                                             tolerance,
                                             aLParams,
                                             ref Vtgint,
                                             ref linenumber,
                                             indiceline,
                                             currentarc,
                                             ref currentparameter,
                                             ref pointonarc,
                                             QuadSurf,
                                             OtherQuad,
                                             ref aVertTol);
                        if (linefound)
                        {
                            Normale = QuadSurf.Normale(Psurf); // Normale a la surface au point
                            currentarc = currentpointonrst.Arc();
                            //-- currentparameter = currentpointonrst.Parameter();
                            currentarc.D1(currentparameter, out p2d, out d2d);
                            QuadSurf.D1(p2d.X(), p2d.Y(), ref ptbid, ref d1u, ref d1v);
                            Vtgrst.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);

                            IntPatch_Line lin = slin.Value(linenumber);
                            TheType = lin.ArcType();

                            if (!OnFirst)
                            { // on cherche la correspondance entre point sur domaine
                              // de la premiere surface et point sur domaine de la
                              // deuxieme surface

                                goon = PointOnSecondDom(listpnt,
                                                        Domain,
                                                        QuadSurf,
                                                        Normale,
                                                        Vtgint,
                                                        lin,
                                                        Done,
                                                        i,
                                                        aVertTol);
                            }

                            if (goon)
                            {
                                //-- Modification du 4 avril 97    tolerance->Tolarc
                                //-- on replace sur le vertex la tolerance d entree et
                                //-- non la tolerance qui a servi au FindLine
                                solpnt.SetValue(Psurf, aVertTol, false);

                                U1 = p2d.X();
                                V1 = p2d.Y();
                                OtherQuad.Parameters(Psurf, ref U2, ref V2);

                                if (OnFirst)
                                {
                                    Recadre(S1, S2, ref U1, ref V1, ref U2, ref V2);
                                    solpnt.SetParameters(U1, V1, U2, V2);
                                }
                                else
                                {
                                    Recadre(S1, S2, ref U2, ref V2, ref U1, ref V1);
                                    solpnt.SetParameters(U2, V2, U1, V1);
                                }

                                if (!currentpointonrst.IsNew())
                                {
                                    vtx = currentpointonrst.Vertex();
                                    solpnt.SetVertex(OnFirst, vtx);
                                }
                                else
                                {
                                    //-- goon = false; ????
                                }

                                if (Normale.SquareMagnitude() < 1e-16)
                                {
                                    Transline.SetValue(true, IntSurf_Undecided);
                                    Transarc.SetValue(true, IntSurf_Undecided);
                                }
                                else
                                {
                                    IntSurf.MakeTransition(Vtgint, Vtgrst, new gp_Dir(Normale), ref Transline, ref Transarc);
                                }
                                solpnt.SetArc(OnFirst, currentarc, currentparameter, Transline, Transarc);

                                for (var anItr = new NCollection_List<double>.Iterator(aLParams); anItr.More(); anItr.Next())
                                {
                                    solpnt.SetParameter(anItr.Value());
                                    AddVertexTo(lin, TheType, solpnt);
                                }

                                Done[i] = 1;

                                if (goon)
                                {
                                    for (k = i + 1; k <= nbpnt; k++)
                                    {
                                        if (Done[k] != 1)
                                        {
                                            currentpointonrst = listpnt.Value(k);
                                            if (!currentpointonrst.IsNew())
                                            {
                                                vtxbis = currentpointonrst.Vertex();
                                                if (vtx is null)
                                                {
                                                }
                                                else if (Domain.Identical(vtx, vtxbis))
                                                {
                                                    solpnt.SetVertex(OnFirst, vtxbis);
                                                    solpnt.SetTolerance(Tolarc);
                                                    currentarc = currentpointonrst.Arc();
                                                    currentparameter = currentpointonrst.Parameter();

                                                    currentarc.D1(currentparameter, out p2d, out d2d);
                                                    Vtgrst.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
                                                    if (Normale.SquareMagnitude() < 1e-16)
                                                    {
                                                        Transline.SetValue(true, IntSurf_Undecided);
                                                        Transarc.SetValue(true, IntSurf_Undecided);
                                                    }
                                                    else
                                                    {
                                                        IntSurf.MakeTransition(Vtgint, Vtgrst, new gp_Dir(Normale),
                                                                               ref Transline, ref Transarc);
                                                    }
                                                    solpnt.SetArc(OnFirst, currentarc, currentparameter, Transline,
                                                                  Transarc);
                                                    AddVertexTo(lin, TheType, solpnt);
                                                    Done[k] = 1;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            Done[i] = 1; // il faudra tester si IsNew ou pas
                                         // et traiter en consequence
                        }
                    }
                }
            }
        }
    }

    private static bool MultiplePoint(NCollection_Sequence<IntPatch_ThePathPointOfTheSOnBounds> listpnt,
                                      Adaptor3d_TopolTool Domain,
                                      in IntSurf_Quadric QuadSurf,
                                      in gp_Vec Normale,
                                      NCollection_Sequence<IntPatch_Line> slin,
                                      NCollection_Array1<int> Done,
                                      NCollection_Array1<int> UsedLine,
                                      int Index,
                                      bool OnFirst,
                                      double theToler)
    {
        // Traitement des points "multiples".

        int k, ii, jj, nbvtx;
        int nblin = slin.Length();
        IntPatch_IType TheType;

        IntSurf_Transition Transline = new(), Transarc = new();

        IntPatch_Point intpt;
        Adaptor2d_Curve2d currentarc;
        Adaptor3d_HVertex? vtx = null, vtxbis;

        int nbpnt = listpnt.Length();
        IntPatch_ThePathPointOfTheSOnBounds currentpointonrst = listpnt.Value(Index);
        IntPatch_ThePathPointOfTheSOnBounds otherpt;
        gp_Pnt Point = currentpointonrst.Value();
        NCollection_Array1<int> localdone = new(1, nbpnt);
        localdone.Init(0);
        for (ii = 1; ii <= nbpnt; ii++)
        {
            localdone[ii] = Done[ii];
        }

        double currentparameter;
        double Paraint = 0.0;
        gp_Vec Vtgint = new(), Vtgrst = new();
        gp_Pnt ptbid = new();

        gp_Vec d1u = new(), d1v = new();

        bool goon;

        bool Retvalue = true;

        for (ii = 1; ii <= nblin; ii++)
        {
            IntPatch_Line slinValueii = slin.Value(ii);
            TheType = slinValueii.ArcType();
            nbvtx = NbVertexOf(slinValueii, TheType);
            jj = 1;
            while (jj <= nbvtx)
            {
                intpt = VertexOf(slinValueii, TheType, jj);
                if (intpt.IsMultiple()
                    && ((OnFirst && !intpt.IsOnDomS1()) || (!OnFirst && !intpt.IsOnDomS2())))
                {
                    if (Point.Distance(intpt.Value()) <= intpt.Tolerance())
                    {
                        Retvalue = false;
                        bool foo = SingleLine(Point, slinValueii, intpt.Tolerance(), ref Paraint, ref Vtgint);
                        if (!foo)
                        {
                            return false; // ne doit pas se produire
                        }

                        if (!currentpointonrst.IsNew())
                        {
                            goon = true;
                            vtx = currentpointonrst.Vertex();
                            intpt.SetVertex(OnFirst, vtx);
                        }
                        else
                        {
                            goon = false;
                        }
                        currentarc = currentpointonrst.Arc();
                        currentparameter = currentpointonrst.Parameter();
                        currentarc.D1(currentparameter, out gp_Pnt2d p2d, out gp_Vec2d d2d);
                        QuadSurf.D1(p2d.X(), p2d.Y(), ref ptbid, ref d1u, ref d1v);
                        Vtgrst.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);

                        //-- Si la normale est nulle (apex d un cone) On simule une transition UNKNOWN
                        if (Normale.SquareMagnitude() < 1e-16)
                        {
                            Transline.SetValue(true, IntSurf_Undecided);
                            Transarc.SetValue(true, IntSurf_Undecided);
                        }
                        else
                        {
                            IntSurf.MakeTransition(Vtgint, Vtgrst, new gp_Dir(Normale), ref Transline, ref Transarc);
                        }

                        //-- Avant, on ne mettait pas ce point (17 nov 97)
                        intpt.SetValue(Point);

                        intpt.SetArc(OnFirst, currentarc, currentparameter, Transline, Transarc);
                        intpt.SetTolerance(theToler);

                        ReplaceVertexOf(slinValueii, TheType, jj, intpt);
                        localdone[Index] = 1;
                        if (goon)
                        {
                            for (k = Index + 1; k <= nbpnt; k++)
                            {
                                if (Done[k] != 1)
                                {
                                    otherpt = listpnt.Value(k);
                                    if (!otherpt.IsNew())
                                    {
                                        vtxbis = otherpt.Vertex();
                                        if (Domain.Identical(vtx!, vtxbis))
                                        {
                                            intpt.SetVertex(OnFirst, vtxbis);
                                            currentarc = otherpt.Arc();
                                            currentparameter = otherpt.Parameter();

                                            currentarc.D1(currentparameter, out p2d, out d2d);
                                            Vtgrst.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
                                            if (Normale.SquareMagnitude() < 1e-16)
                                            {
                                                Transline.SetValue(true, IntSurf_Undecided);
                                                Transarc.SetValue(true, IntSurf_Undecided);
                                            }
                                            else
                                            {
                                                IntSurf.MakeTransition(Vtgint, Vtgrst, new gp_Dir(Normale),
                                                                       ref Transline, ref Transarc);
                                            }
                                            intpt.SetArc(OnFirst, currentarc, currentparameter, Transline, Transarc);
                                            intpt.SetTolerance(theToler);
                                            AddVertexTo(slinValueii, TheType, intpt);
                                            UsedLine[ii] = 1;
                                            Retvalue = true;
                                            localdone[k] = 1;
                                        }
                                    }
                                }
                            }
                        }
                        //--           jj = nbvtx +1;
                    }
                    //--         else {
                    jj = jj + 1;
                    //--        }
                }
                else
                {
                    jj = jj + 1;
                }
            }
        }

        for (ii = 1; ii <= nbpnt; ii++)
        {
            Done[ii] = localdone[ii];
        }

        return Retvalue;
    }

    // Duplication des points sur domaine de l autre surface.
    // On sait que le vertex sous-jacent est PntRef
    private static bool PointOnSecondDom(NCollection_Sequence<IntPatch_ThePathPointOfTheSOnBounds> listpnt,
                                         Adaptor3d_TopolTool Domain,
                                         in IntSurf_Quadric QuadSurf,
                                         in gp_Vec Normale,
                                         in gp_Vec Vtgint,
                                         IntPatch_Line lin,
                                         NCollection_Array1<int> Done,
                                         int Index,
                                         double theToler)
    {
        int k, jj, nbvtx;
        IntPatch_IType TheType;

        IntSurf_Transition Transline = new(), Transarc = new();
        IntPatch_Point intpt;
        Adaptor2d_Curve2d currentarc;
        Adaptor3d_HVertex? vtx = null, vtxbis;
        gp_Pnt ptbid = new();
        gp_Vec Vtgrst = new();

        gp_Vec d1u = new(), d1v = new();

        int nbpnt = listpnt.Length();
        IntPatch_ThePathPointOfTheSOnBounds currentpointonrst = listpnt.Value(Index);
        double currentparameter;

        bool goon;
        bool Retvalue = true;

        TheType = lin.ArcType();
        nbvtx = NbVertexOf(lin, TheType);
        jj = 1;
        while (jj <= nbvtx)
        {
            intpt = VertexOf(lin, TheType, jj);
            if (!intpt.IsOnDomS2())
            {
                if (currentpointonrst.Value().Distance(intpt.Value()) <= intpt.Tolerance())
                {
                    Retvalue = false;
                    if (!currentpointonrst.IsNew())
                    {
                        goon = true;
                        vtx = currentpointonrst.Vertex();
                        intpt.SetVertex(false, vtx);
                    }
                    else
                    {
                        goon = false;
                    }
                    currentarc = currentpointonrst.Arc();
                    currentparameter = currentpointonrst.Parameter();
                    currentarc.D1(currentparameter, out gp_Pnt2d p2d, out gp_Vec2d d2d);
                    QuadSurf.D1(p2d.X(), p2d.Y(), ref ptbid, ref d1u, ref d1v);
                    Vtgrst.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
                    if (Normale.SquareMagnitude() < 1e-16)
                    {
                        Transline.SetValue(true, IntSurf_Undecided);
                        Transarc.SetValue(true, IntSurf_Undecided);
                    }
                    else
                    {
                        IntSurf.MakeTransition(Vtgint, Vtgrst, new gp_Dir(Normale), ref Transline, ref Transarc);
                    }
                    intpt.SetArc(false, currentarc, currentparameter, Transline, Transarc);
                    intpt.SetTolerance(theToler);

                    ReplaceVertexOf(lin, TheType, jj, intpt);
                    Done[Index] = 1;

                    if (goon)
                    {
                        for (k = Index + 1; k <= nbpnt; k++)
                        {
                            if (Done[k] != 1)
                            {
                                currentpointonrst = listpnt.Value(k);
                                if (!currentpointonrst.IsNew())
                                {
                                    vtxbis = currentpointonrst.Vertex();
                                    if (Domain.Identical(vtx!, vtxbis))
                                    {
                                        intpt.SetVertex(false, vtxbis);
                                        currentarc = currentpointonrst.Arc();
                                        currentparameter = currentpointonrst.Parameter();
                                        currentarc.D1(currentparameter, out p2d, out d2d);
                                        Vtgrst.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
                                        if (Normale.SquareMagnitude() < 1e-16)
                                        {
                                            Transline.SetValue(true, IntSurf_Undecided);
                                            Transarc.SetValue(true, IntSurf_Undecided);
                                        }
                                        else
                                        {
                                            IntSurf.MakeTransition(Vtgint, Vtgrst, new gp_Dir(Normale), ref Transline,
                                                                   ref Transarc);
                                        }
                                        intpt.SetArc(false, currentarc, currentparameter, Transline, Transarc);
                                        intpt.SetTolerance(theToler);
                                        AddVertexTo(lin, TheType, intpt);
                                        Done[k] = 1;
                                    }
                                }
                            }
                        }
                    }
                    //-- jj = nbvtx + 1;
                    jj++;
                }
                else
                {
                    jj = jj + 1;
                }
            }
            else
            {
                jj = jj + 1;
            }
            nbvtx = NbVertexOf(lin, TheType);
        }
        return Retvalue;
    }

    private static bool FindLine(ref gp_Pnt Psurf,
                                 NCollection_Sequence<IntPatch_Line> slin,
                                 double Tol,
                                 NCollection_List<double> theLParams,
                                 ref gp_Vec Vtgtint,
                                 ref int theLineIdx,
                                 int OnlyThisLine,
                                 Adaptor2d_Curve2d thearc,
                                 ref double theparameteronarc,
                                 ref gp_Pnt thepointonarc,
                                 in IntSurf_Quadric QuadSurf1,
                                 in IntSurf_Quadric QuadSurf2,
                                 ref double theOutputToler)
    {
        if ((QuadSurf1.Distance(Psurf) > Tol) || (QuadSurf2.Distance(Psurf) > Tol))
        {
            return false;
        }

        // Traitement du point de depart ayant pour representation Psurf
        // dans l espace. On recherche la ligne d intersection contenant ce point.
        // On a en sortie la ligne, et le parametre et sa tangente du point sur
        // la ligne d intersection.
        double aSqTol = Tol * Tol;
        double aSqDistMin = RealLast();
        double aSqDist, para = 0.0;
        double lower, upper;
        gp_Pnt pt = new();
        int i;
        IntPatch_IType typarc;

        double aParaInt = RealLast();
        int nblin = slin.Length();
        for (i = 1; i <= nblin; i++)
        {
            if (OnlyThisLine != 0)
            {
                i = OnlyThisLine;
                nblin = 0;
            }
            IntPatch_Line lin = slin.Value(i);
            typarc = lin.ArcType();
            if (typarc == IntPatch_Analytic)
            {
                lower = ((IntPatch_ALine)lin).FirstParameter(out _);
                upper = ((IntPatch_ALine)lin).LastParameter(out _);
            }
            else
            {
                if (((IntPatch_GLine)lin).HasFirstPoint())
                {
                    lower = ((IntPatch_GLine)lin).FirstPoint().ParameterOnLine();
                }
                else
                {
                    lower = RealFirst();
                }
                if (((IntPatch_GLine)lin).HasLastPoint())
                {
                    upper = ((IntPatch_GLine)lin).LastPoint().ParameterOnLine();
                }
                else
                {
                    upper = RealLast();
                }
            }

            switch (typarc)
            {
                case IntPatch_Lin:
                    {
                        para = ElCLib.Parameter(((IntPatch_GLine)lin).Line(), Psurf);
                        if (para <= upper && para >= lower)
                        {
                            pt = ElCLib.Value(para, ((IntPatch_GLine)lin).Line());
                            aSqDist = Psurf.SquareDistance(pt);
                            if ((aSqDist < aSqTol) && (aSqDist < aSqDistMin))
                            {
                                aSqDistMin = aSqDist;
                                aParaInt = para;
                                theLineIdx = i;
                            }
                        }
                    }
                    break;
                case IntPatch_Circle:
                    {
                        para = ElCLib.Parameter(((IntPatch_GLine)lin).Circle(), Psurf);
                        if ((para <= upper && para >= lower)
                            || (para + 2.0 * M_PI <= upper && para + 2.0 * M_PI >= lower)
                            || (para - 2.0 * M_PI <= upper && para - 2.0 * M_PI >= lower))
                        {
                            pt = ElCLib.Value(para, ((IntPatch_GLine)lin).Circle());
                            aSqDist = Psurf.SquareDistance(pt);
                            if ((aSqDist < aSqTol) && (aSqDist < aSqDistMin))
                            {
                                aSqDistMin = aSqDist;
                                aParaInt = para;
                                theLineIdx = i;
                            }
                        }
                    }
                    break;
                case IntPatch_Ellipse:
                    {
                        para = ElCLib.Parameter(((IntPatch_GLine)lin).Ellipse(), Psurf);
                        if ((para <= upper && para >= lower)
                            || (para + 2.0 * M_PI <= upper && para + 2.0 * M_PI >= lower)
                            || (para - 2.0 * M_PI <= upper && para - 2.0 * M_PI >= lower))
                        {
                            pt = ElCLib.Value(para, ((IntPatch_GLine)lin).Ellipse());
                            aSqDist = Psurf.SquareDistance(pt);
                            if ((aSqDist < aSqTol) && (aSqDist < aSqDistMin))
                            {
                                aSqDistMin = aSqDist;
                                aParaInt = para;
                                theLineIdx = i;
                            }
                        }
                    }
                    break;
                case IntPatch_Parabola:
                    {
                        //-- Le calcul du parametre sur une parabole est mal fait ds ElCLib. Il ne tient pas compte
                        //-- de la meilleure facon de calculer (axe X ou axe Y). Bilan : Si la parabole est tres
                        //-- pointue (focal de l'ordre de 1e-2 et si le point est a un parametre grand, ca foire. )
                        //-- On ne peut pas modifier faciolement ds ElCLib car on ne passe pas la focale. ...
                        gp_Parab Parab = ((IntPatch_GLine)lin).Parabola();
                        para = ElCLib.Parameter(Parab, Psurf);
                        if (para <= upper && para >= lower)
                        {
                            int amelioration = 0;
                            do
                            {
                                double parabis = para + 0.0000001;

                                pt = ElCLib.Value(para, Parab);
                                aSqDist = Psurf.SquareDistance(pt);

                                gp_Pnt ptbis = ElCLib.Value(parabis, Parab);
                                double distbis = Psurf.Distance(ptbis);
                                double aDist = Math.Sqrt(aSqDist);
                                double ddist = distbis - aDist;

                                if ((aSqDist < aSqTol) && (aSqDist < aSqDistMin))
                                {
                                    aSqDistMin = aSqDist;
                                    aParaInt = para;
                                    theLineIdx = i;
                                }
                                if (aSqDist < Precision.SquarePConfusion())
                                {
                                    amelioration = 100;
                                }

                                if (ddist > 1.0e-9 || ddist < -1.0e-9)
                                {
                                    para = para - aDist * (parabis - para) / ddist;
                                }
                                else
                                {
                                    amelioration = 100;
                                }
                            } while (++amelioration < 5);
                        }
                    }
                    break;
                case IntPatch_Hyperbola:
                    {
                        para = ElCLib.Parameter(((IntPatch_GLine)lin).Hyperbola(), Psurf);
                        if (para <= upper && para >= lower)
                        {
                            pt = ElCLib.Value(para, ((IntPatch_GLine)lin).Hyperbola());
                            aSqDist = Psurf.SquareDistance(pt);
                            if ((aSqDist < aSqTol) && (aSqDist < aSqDistMin))
                            {
                                aSqDistMin = aSqDist;
                                aParaInt = para;
                                theLineIdx = i;
                            }
                        }
                    }
                    break;

                case IntPatch_Analytic:
                    {
                        IntPatch_ALine alin = (IntPatch_ALine)lin;
                        NCollection_List<double> aLParams = new();
                        alin.FindParameter(Psurf, aLParams);
                        if (!aLParams.IsEmpty())
                        {
                            // All found distances are already in some internal tolerance
                            // set in alin->FindParameter(...) method.

                            aSqDist = RealLast();
                            for (var anItr = new NCollection_List<double>.Iterator(aLParams); anItr.More(); anItr.Next())
                            {
                                pt = alin.Value(anItr.Value());
                                double aSqD = Psurf.SquareDistance(pt);
                                if (aSqD < aSqDist)
                                {
                                    aSqDist = aSqD;
                                }
                            }

                            if (aSqDist < aSqDistMin)
                            {
                                aSqDistMin = aSqDist;
                                theLParams.Assign(aLParams);
                                theLineIdx = i;
                            }
                        }
                        else
                        {
                            //-- le point n a pas ete trouve par bete projection.
                            //-- on essaie l intersection avec la restriction en 2d
                            double theparamonarc = theparameteronarc;
                            gp_Pnt CopiePsurf = Psurf;
                            bool IntersectIsOk = IntersectionWithAnArc(ref CopiePsurf,
                                                                       alin,
                                                                       ref para,
                                                                       thearc,
                                                                       ref theparamonarc,
                                                                       ref thepointonarc,
                                                                       QuadSurf1,
                                                                       lower,
                                                                       upper);
                            aSqDist = CopiePsurf.SquareDistance(Psurf);
                            if (IntersectIsOk)
                            {
                                if (aSqDist < aSqTol)
                                {
                                    theparameteronarc = theparamonarc;
                                    Psurf = thepointonarc;
                                    aSqDistMin = aSqDist;
                                    theLParams.Append(para);
                                    theLineIdx = i;
                                }
                            }
                        }
                    }
                    break;

                case IntPatch_Walking: // impossible . c est pour eviter les warnings
                case IntPatch_Restriction: // impossible . c est pour eviter les warnings
                    break;
            }
        }

        if (aSqDistMin == RealLast())
        {
            return false;
        }

        theOutputToler = Max(theOutputToler, Math.Sqrt(aSqDistMin));

        typarc = slin.Value(theLineIdx).ArcType();

        // Computation of tangent vector
        switch (typarc)
        {
            case IntPatch_Lin:
                theLParams.Append(aParaInt);
                Vtgtint = new gp_Vec(((IntPatch_GLine)slin[theLineIdx]).Line().Direction());
                break;
            case IntPatch_Circle:
                theLParams.Append(aParaInt);
                Vtgtint = ElCLib.DN(aParaInt, ((IntPatch_GLine)slin[theLineIdx]).Circle(), 1);
                break;
            case IntPatch_Ellipse:
                theLParams.Append(aParaInt);
                Vtgtint = ElCLib.DN(aParaInt, ((IntPatch_GLine)slin[theLineIdx]).Ellipse(), 1);
                break;
            case IntPatch_Parabola:
                theLParams.Append(aParaInt);
                Vtgtint = ElCLib.DN(aParaInt, ((IntPatch_GLine)slin[theLineIdx]).Parabola(), 1);
                break;
            case IntPatch_Hyperbola:
                theLParams.Append(aParaInt);
                Vtgtint = ElCLib.DN(aParaInt, ((IntPatch_GLine)slin[theLineIdx]).Hyperbola(), 1);
                break;

            case IntPatch_Analytic:
                {
                    if (!((IntPatch_ALine)slin[theLineIdx]).D1(theLParams.Last(), out pt, out Vtgtint))
                    {
                        // Previously (before the fix #29807) this code tried to process case
                        // when occ::handle<IntPatch_ALine>::D1(...) method returns FALSE and
                        // computed Vtgtint input argument value. Currently, any singularities
                        // must be processed by high-level algorithms (IntPatch_SpecialPoints class).
                        // Therefore this code has been deleted as deprecated.

                        Vtgtint.SetCoord(0.0, 0.0, 0.0);
                    }
                }
                break;
            case IntPatch_Walking: // impossible . c est pour eviter les warnings
            case IntPatch_Restriction: // impossible . c est pour eviter les warnings
                break;
        }
        return true;
    }

    //=======================================================================
    // function : SingleLine
    // purpose  : Traitement du point de depart ayant pour representation Psurf
    //            dans l espace. On le replace sur la ligne d intersection; On a en sortie
    //            son parametre et sa tangente sur la ligne d intersection.
    //            La fonction renvoie False si le point projete est a une distance
    //            superieure a Tol du point a projeter.
    //=======================================================================
    private static bool SingleLine(in gp_Pnt Psurf,
                                   IntPatch_Line lin,
                                   double Tol,
                                   ref double Paraint,
                                   ref gp_Vec Vtgtint)
    {
        IntPatch_IType typarc = lin.ArcType();

        double parproj = 0.0;
        gp_Vec tgint = new();
        gp_Pnt ptproj = new();
        bool retvalue;

        switch (typarc)
        {
            case IntPatch_Lin:
                parproj = ElCLib.Parameter(((IntPatch_GLine)lin).Line(), Psurf);
                ElCLib.D1(parproj, ((IntPatch_GLine)lin).Line(), out ptproj, out tgint);
                break;
            case IntPatch_Circle:
                parproj = ElCLib.Parameter(((IntPatch_GLine)lin).Circle(), Psurf);
                ElCLib.D1(parproj, ((IntPatch_GLine)lin).Circle(), out ptproj, out tgint);
                break;
            case IntPatch_Ellipse:
                parproj = ElCLib.Parameter(((IntPatch_GLine)lin).Ellipse(), Psurf);
                ElCLib.D1(parproj, ((IntPatch_GLine)lin).Ellipse(), out ptproj, out tgint);
                break;
            case IntPatch_Parabola:
                parproj = ElCLib.Parameter(((IntPatch_GLine)lin).Parabola(), Psurf);
                ElCLib.D1(parproj, ((IntPatch_GLine)lin).Parabola(), out ptproj, out tgint);
                break;
            case IntPatch_Hyperbola:
                parproj = ElCLib.Parameter(((IntPatch_GLine)lin).Hyperbola(), Psurf);
                ElCLib.D1(parproj, ((IntPatch_GLine)lin).Hyperbola(), out ptproj, out tgint);
                break;
            case IntPatch_Analytic:
                {
                    IntPatch_ALine alin = (IntPatch_ALine)lin;
                    NCollection_List<double> aLParams = new();
                    alin.FindParameter(Psurf, aLParams);
                    if (!aLParams.IsEmpty())
                    {
                        ptproj = Psurf;
                        parproj = aLParams.Last();
                        if (!alin.D1(parproj, out _, out tgint))
                        {
                            // Previously (before the fix #29807) this code tried to process case
                            // when occ::handle<IntPatch_ALine>::D1(...) method returns FALSE and
                            // computed Vtgtint input argument value. Currently, any singularities
                            // must be processed by high-level algorithms (IntPatch_SpecialPoints class).
                            // Therefore this code has been deleted as deprecated.

                            tgint.SetCoord(0.0, 0.0, 0.0);
                        }
                    }
                    else
                    {
                        return false;
                    }
                }
                break;
            case IntPatch_Walking: // impossible . c est pour eviter les warnings
            case IntPatch_Restriction: // impossible . c est pour eviter les warnings
                break;
        }

        if (Psurf.Distance(ptproj) <= Tol)
        {
            Paraint = parproj;
            Vtgtint = tgint;
            retvalue = true;
        }
        else
        {
            retvalue = false;
        }
        return retvalue;
    }
}
