// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Patches;

// The treatment of the boundaries (OCCT 7.x's IntPatch_ImpImpIntersection_0.gxx): the points of the restrictions
// put on the intersection lines (PutPointsOnLine, MultiplePoint, PointOnSecondDom, FindLine, SingleLine), the
// restriction segments that lie on the other surface (ProcessSegments) and the restriction lines that duplicate
// another line (ProcessRLine).
public sealed partial class ImpImpIntersection
{
    //-- le calcul de dist est completement faux ds la routine ci dessous a revoir (lbr le 18 nov 97)
    private static bool IntersectionWithAnArc(ref Point3D pSurf,
                                              ALine alin,
                                              ref double para,
                                              Curve2DAdaptorBase thearc,
                                              ref double _theparameteronarc,
                                              ref Point3D thepointonarc,
                                              in SurfaceQuadric quadSurf,
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
            Point3D p = alin.Value(_theta);
            double d = p.DistanceTo(pSurf);
            if (d < distmin)
            {
                thetamin = _theta;
                distmin = d;
            }
        }

        double bestpara = 0.0, besttheta = 0.0, bestdist = 0.0, distinit = 0.0;

        //-- Distance initiale
        {
            Point3D pp0 = alin.Value(thetamin);
            double ua0 = 0.0, va0 = 0.0;
            quadSurf.Parameters(pp0, ref ua0, ref va0);
            thearc.D1(theparameteronarc, out Point2D p2d, out Vector2D d2d);
            Vector2D paPr = new(new Point2D(ua0, va0), p2d);
            distinit = paPr.Magnitude;
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
            Point3D pp0 = alin.Value(theta);
            Point3D pp1 = alin.Value(theta + du);
            quadSurf.Parameters(pp0, ref ua0, ref va0);
            quadSurf.Parameters(pp1, ref ua1, ref va1);

            Vector2D d1a = new((ua1 - ua0) / du, (va1 - va0) / du);
            thearc.D1(theparameteronarc, out Point2D p2d, out Vector2D d2d);
            Vector2D paPr = new(new Point2D(ua0, va0), p2d);

            double pbd = paPr.Magnitude;
            if (bestdist > pbd)
            {
                bestdist = pbd;
                bestpara = theparameteronarc;
                besttheta = theta;
            }

            d1a = d1a.WithCoord(-d1a.X, -d1a.Y);

            double d = d1a.X * d2d.Y - d1a.Y * d2d.X;

            double da = (-paPr.X) * d2d.Y - (-paPr.Y) * d2d.X;
            double dr = d1a.X * (-paPr.Y) - d1a.Y * (-paPr.X);
            if (Math.Abs(d) > 1e-15)
            {
                da /= d;
                dr /= d;
            }
            else
            {
                if (Math.Abs(paPr.X) > Math.Abs(paPr.Y))
                {
                    double xx = paPr.X;
                    xx *= 0.5;
                    if (d1a.X != 0.0)
                    {
                        da = -xx / d1a.X;
                    }
                    if (d2d.X != 0.0)
                    {
                        dr = -xx / d2d.X;
                    }
                }
                else
                {
                    double yy = paPr.Y;
                    yy *= 0.5;
                    if (d1a.Y != 0.0)
                    {
                        da = -yy / d1a.Y;
                    }
                    if (d2d.Y != 0.0)
                    {
                        dr = -yy / d2d.Y;
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
                pSurf = alin.Value(para);
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
            pSurf = alin.Value(para);
            _theparameteronarc = bestpara;
            thepointonarc = alin.Value(para);
            return (true);
        }
        return (false);
    }

    //-- ======================================================================
    private static void Recadre(SurfaceAdaptorBase myHS1,
                                SurfaceAdaptorBase myHS2,
                                ref double u1,
                                ref double v1,
                                ref double u2,
                                ref double v2)
    {
        double f, l, lmf, fpls2;
        SurfaceKind typs1 = myHS1.GetType();
        SurfaceKind typs2 = myHS2.GetType();

        bool myHS1IsUPeriodic, myHS1IsVPeriodic;
        switch (typs1)
        {
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
                {
                    myHS1IsUPeriodic = true;
                    myHS1IsVPeriodic = false;
                    break;
                }
            case SurfaceKind.Torus:
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
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
                {
                    myHS2IsUPeriodic = true;
                    myHS2IsVPeriodic = false;
                    break;
                }
            case SurfaceKind.Torus:
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
    private static int NbVertexOf(PatchLine line, PatchLineType type) =>
        type == PatchLineType.Analytic ? ((ALine)line).VertexCount : ((GLine)line).VertexCount;

    private static PatchPoint VertexOf(PatchLine line, PatchLineType type, int index) =>
        type == PatchLineType.Analytic
            ? ((ALine)line).Vertex(index)
            : ((GLine)line).Vertex(index);

    private static void AddVertexTo(PatchLine line, PatchLineType type, in PatchPoint pnt)
    {
        if (type == PatchLineType.Analytic)
        {
            ((ALine)line).AddVertex(pnt);
        }
        else
        {
            ((GLine)line).AddVertex(pnt);
        }
    }

    private static void ReplaceVertexOf(PatchLine line, PatchLineType type, int index,
                                        in PatchPoint pnt)
    {
        if (type == PatchLineType.Analytic)
        {
            ((ALine)line).Replace(index, pnt);
        }
        else
        {
            ((GLine)line).Replace(index, pnt);
        }
    }

    //=================================================================================================

    private static void PutPointsOnLine(SurfaceAdaptorBase s1,
                                        SurfaceAdaptorBase s2,
                                        NCollection_Sequence<ThePathPointOfTheSOnBounds> listpnt,
                                        NCollection_Sequence<PatchLine> slin,
                                        bool onFirst,
                                        TopolTool domain,
                                        in SurfaceQuadric quadSurf,
                                        in SurfaceQuadric otherQuad,
                                        bool multpoint,
                                        double tolarc)
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
        double u1, v1, u2 = 0.0, v2 = 0.0;
        bool goon;

        Point3D psurf, ptbid = new();
        Vector3D normale = new(), vtgint = new(), vtgrst = new();

        Vector3D d1u = new(), d1v = new();
        Point2D p2d;
        Vector2D d2d;

        WalkingTransition transline = new(), transarc = new();

        Curve2DAdaptorBase currentarc;
        HVertex? vtx = null, vtxbis;

        PatchPoint solpnt = new();
        ThePathPointOfTheSOnBounds currentpointonrst;
        PatchLineType theType;

        NCollection_Array1<int> usedLine = new(1, nblin);
        NCollection_Array1<int> done = new(1, nbpnt);
        for (i = 1; i <= nbpnt; i++)
        {
            done[i] = 0; //-- Initialisation a la main
        }

        for (i = 1; i <= nbpnt; i++)
        {
            if (done[i] != 1)
            {
                currentpointonrst = listpnt.Value(i);
                psurf = currentpointonrst.Value(); // Point dans l espace
                tolerance = currentpointonrst.Tolerance();

                // On recherche d abord si on a correspondance avec un "point multiple"
                usedLine.Init(0);

                goon = true;
                if (multpoint)
                {
                    normale = quadSurf.Normale(psurf); // Normale a la surface au point
                    currentarc = currentpointonrst.Arc();
                    currentparameter = currentpointonrst.Parameter();
                    currentarc.D1(currentparameter, out p2d, out d2d);
                    quadSurf.D1(p2d.X, p2d.Y, ref ptbid, ref d1u, ref d1v);
                    vtgrst = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
                    goon = MultiplePoint(listpnt,
                                         domain,
                                         quadSurf,
                                         normale,
                                         slin,
                                         done,
                                         usedLine,
                                         i,
                                         onFirst,
                                         tolarc);
                }
                if (goon)
                {
                    bool linefound = false;

                    for (int indiceline = 1; indiceline <= slin.Length(); indiceline++)
                    {
                        if (usedLine[indiceline] != 0)
                        {
                            continue;
                        }
                        linenumber = indiceline;

                        //-- Attention , les points peuvent etre deplaces
                        //-- il faut reprendre le point original
                        currentpointonrst = listpnt.Value(i);
                        currentarc = currentpointonrst.Arc();
                        currentparameter = currentpointonrst.Parameter();
                        psurf = currentpointonrst.Value(); // Point dans l espace
                        tolerance = currentpointonrst.Tolerance();
                        //--

                        //  Modified by skv - Thu Jan 15 15:57:15 2004 OCC4455 Begin
                        if (!currentpointonrst.IsNew())
                        {
                            HVertex aVtx = currentpointonrst.Vertex();
                            double vtxTol = aVtx.Resolution(currentarc);
                            double tolAng = 0.01 * tolerance;

                            tolerance = Max(tolerance, vtxTol);

                            Vector3D norm1 = quadSurf.Normale(psurf);
                            Vector3D norm2 = otherQuad.Normale(psurf);
                            //
                            if (norm1.Magnitude > Constants.Resolution() && norm2.Magnitude > Constants.Resolution())
                            {
                                if (norm1.IsParallel(norm2, tolAng))
                                {
                                    tolerance = Math.Sqrt(tolerance);
                                }
                            } //
                        }
                        //  Modified by skv - Thu Jan 15 15:57:15 2004 OCC4455 End
                        Point3D pointonarc = new();
                        vtgint = vtgint.WithCoord(0, 0, 0);
                        double vertTol = tolarc;
                        NCollection_List<double> lParams = new();
                        linefound = FindLine(ref psurf,
                                             slin,
                                             tolerance,
                                             lParams,
                                             ref vtgint,
                                             ref linenumber,
                                             indiceline,
                                             currentarc,
                                             ref currentparameter,
                                             ref pointonarc,
                                             quadSurf,
                                             otherQuad,
                                             ref vertTol);
                        if (linefound)
                        {
                            normale = quadSurf.Normale(psurf); // Normale a la surface au point
                            currentarc = currentpointonrst.Arc();
                            //-- currentparameter = currentpointonrst.Parameter();
                            currentarc.D1(currentparameter, out p2d, out d2d);
                            quadSurf.D1(p2d.X, p2d.Y, ref ptbid, ref d1u, ref d1v);
                            vtgrst = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

                            PatchLine lin = slin.Value(linenumber);
                            theType = lin.ArcType();

                            if (!onFirst)
                            { // on cherche la correspondance entre point sur domaine
                              // de la premiere surface et point sur domaine de la
                              // deuxieme surface

                                goon = PointOnSecondDom(listpnt,
                                                        domain,
                                                        quadSurf,
                                                        normale,
                                                        vtgint,
                                                        lin,
                                                        done,
                                                        i,
                                                        vertTol);
                            }

                            if (goon)
                            {
                                //-- Modification du 4 avril 97    tolerance->Tolarc
                                //-- on replace sur le vertex la tolerance d entree et
                                //-- non la tolerance qui a servi au FindLine
                                solpnt.SetValue(psurf, vertTol, false);

                                u1 = p2d.X;
                                v1 = p2d.Y;
                                otherQuad.Parameters(psurf, ref u2, ref v2);

                                if (onFirst)
                                {
                                    Recadre(s1, s2, ref u1, ref v1, ref u2, ref v2);
                                    solpnt.SetParameters(u1, v1, u2, v2);
                                }
                                else
                                {
                                    Recadre(s1, s2, ref u2, ref v2, ref u1, ref v1);
                                    solpnt.SetParameters(u2, v2, u1, v1);
                                }

                                if (!currentpointonrst.IsNew())
                                {
                                    vtx = currentpointonrst.Vertex();
                                    solpnt.SetVertex(onFirst, vtx);
                                }
                                else
                                {
                                    //-- goon = false; ????
                                }

                                if (normale.SquareMagnitude < 1e-16)
                                {
                                    transline.SetValue(true, WalkingTypeTrans.Undecided);
                                    transarc.SetValue(true, WalkingTypeTrans.Undecided);
                                }
                                else
                                {
                                    global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(vtgint, vtgrst, new Direction3D(normale), ref transline, ref transarc);
                                }
                                solpnt.SetArc(onFirst, currentarc, currentparameter, transline, transarc);

                                for (var itr = new NCollection_List<double>.Iterator(lParams); itr.More(); itr.Next())
                                {
                                    solpnt.SetParameter(itr.Value());
                                    AddVertexTo(lin, theType, solpnt);
                                }

                                done[i] = 1;

                                if (goon)
                                {
                                    for (k = i + 1; k <= nbpnt; k++)
                                    {
                                        if (done[k] != 1)
                                        {
                                            currentpointonrst = listpnt.Value(k);
                                            if (!currentpointonrst.IsNew())
                                            {
                                                vtxbis = currentpointonrst.Vertex();
                                                if (vtx is null)
                                                {
                                                }
                                                else if (domain.Identical(vtx, vtxbis))
                                                {
                                                    solpnt.SetVertex(onFirst, vtxbis);
                                                    solpnt.SetTolerance(tolarc);
                                                    currentarc = currentpointonrst.Arc();
                                                    currentparameter = currentpointonrst.Parameter();

                                                    currentarc.D1(currentparameter, out p2d, out d2d);
                                                    vtgrst = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
                                                    if (normale.SquareMagnitude < 1e-16)
                                                    {
                                                        transline.SetValue(true, WalkingTypeTrans.Undecided);
                                                        transarc.SetValue(true, WalkingTypeTrans.Undecided);
                                                    }
                                                    else
                                                    {
                                                        global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(vtgint, vtgrst, new Direction3D(normale),
                                                                               ref transline, ref transarc);
                                                    }
                                                    solpnt.SetArc(onFirst, currentarc, currentparameter, transline,
                                                                  transarc);
                                                    AddVertexTo(lin, theType, solpnt);
                                                    done[k] = 1;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        else
                        {
                            done[i] = 1; // il faudra tester si IsNew ou pas
                                         // et traiter en consequence
                        }
                    }
                }
            }
        }
    }

    private static bool MultiplePoint(NCollection_Sequence<ThePathPointOfTheSOnBounds> listpnt,
                                      TopolTool domain,
                                      in SurfaceQuadric quadSurf,
                                      in Vector3D normale,
                                      NCollection_Sequence<PatchLine> slin,
                                      NCollection_Array1<int> done,
                                      NCollection_Array1<int> usedLine,
                                      int index,
                                      bool onFirst,
                                      double toler)
    {
        // Traitement des points "multiples".

        int k, ii, jj, nbvtx;
        int nblin = slin.Length();
        PatchLineType theType;

        WalkingTransition transline = new(), transarc = new();

        PatchPoint intpt;
        Curve2DAdaptorBase currentarc;
        HVertex? vtx = null, vtxbis;

        int nbpnt = listpnt.Length();
        ThePathPointOfTheSOnBounds currentpointonrst = listpnt.Value(index);
        ThePathPointOfTheSOnBounds otherpt;
        Point3D point = currentpointonrst.Value();
        NCollection_Array1<int> localdone = new(1, nbpnt);
        localdone.Init(0);
        for (ii = 1; ii <= nbpnt; ii++)
        {
            localdone[ii] = done[ii];
        }

        double currentparameter;
        double paraint = 0.0;
        Vector3D vtgint = new(), vtgrst = new();
        Point3D ptbid = new();

        Vector3D d1u = new(), d1v = new();

        bool goon;

        bool retvalue = true;

        for (ii = 1; ii <= nblin; ii++)
        {
            PatchLine slinValueii = slin.Value(ii);
            theType = slinValueii.ArcType();
            nbvtx = NbVertexOf(slinValueii, theType);
            jj = 1;
            while (jj <= nbvtx)
            {
                intpt = VertexOf(slinValueii, theType, jj);
                if (intpt.IsMultiple
                    && ((onFirst && !intpt.IsOnDomS1) || (!onFirst && !intpt.IsOnDomS2)))
                {
                    if (point.DistanceTo(intpt.Value()) <= intpt.Tolerance())
                    {
                        retvalue = false;
                        bool foo = SingleLine(point, slinValueii, intpt.Tolerance(), ref paraint, ref vtgint);
                        if (!foo)
                        {
                            return false; // ne doit pas se produire
                        }

                        if (!currentpointonrst.IsNew())
                        {
                            goon = true;
                            vtx = currentpointonrst.Vertex();
                            intpt.SetVertex(onFirst, vtx);
                        }
                        else
                        {
                            goon = false;
                        }
                        currentarc = currentpointonrst.Arc();
                        currentparameter = currentpointonrst.Parameter();
                        currentarc.D1(currentparameter, out Point2D p2d, out Vector2D d2d);
                        quadSurf.D1(p2d.X, p2d.Y, ref ptbid, ref d1u, ref d1v);
                        vtgrst = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

                        //-- Si la normale est nulle (apex d un cone) On simule une transition UNKNOWN
                        if (normale.SquareMagnitude < 1e-16)
                        {
                            transline.SetValue(true, WalkingTypeTrans.Undecided);
                            transarc.SetValue(true, WalkingTypeTrans.Undecided);
                        }
                        else
                        {
                            global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(vtgint, vtgrst, new Direction3D(normale), ref transline, ref transarc);
                        }

                        //-- Avant, on ne mettait pas ce point (17 nov 97)
                        intpt.SetValue(point);

                        intpt.SetArc(onFirst, currentarc, currentparameter, transline, transarc);
                        intpt.SetTolerance(toler);

                        ReplaceVertexOf(slinValueii, theType, jj, intpt);
                        localdone[index] = 1;
                        if (goon)
                        {
                            for (k = index + 1; k <= nbpnt; k++)
                            {
                                if (done[k] != 1)
                                {
                                    otherpt = listpnt.Value(k);
                                    if (!otherpt.IsNew())
                                    {
                                        vtxbis = otherpt.Vertex();
                                        if (domain.Identical(vtx!, vtxbis))
                                        {
                                            intpt.SetVertex(onFirst, vtxbis);
                                            currentarc = otherpt.Arc();
                                            currentparameter = otherpt.Parameter();

                                            currentarc.D1(currentparameter, out p2d, out d2d);
                                            vtgrst = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
                                            if (normale.SquareMagnitude < 1e-16)
                                            {
                                                transline.SetValue(true, WalkingTypeTrans.Undecided);
                                                transarc.SetValue(true, WalkingTypeTrans.Undecided);
                                            }
                                            else
                                            {
                                                global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(vtgint, vtgrst, new Direction3D(normale),
                                                                       ref transline, ref transarc);
                                            }
                                            intpt.SetArc(onFirst, currentarc, currentparameter, transline, transarc);
                                            intpt.SetTolerance(toler);
                                            AddVertexTo(slinValueii, theType, intpt);
                                            usedLine[ii] = 1;
                                            retvalue = true;
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
            done[ii] = localdone[ii];
        }

        return retvalue;
    }

    // Duplication des points sur domaine de l autre surface.
    // On sait que le vertex sous-jacent est PntRef
    private static bool PointOnSecondDom(NCollection_Sequence<ThePathPointOfTheSOnBounds> listpnt,
                                         TopolTool domain,
                                         in SurfaceQuadric quadSurf,
                                         in Vector3D normale,
                                         in Vector3D vtgint,
                                         PatchLine lin,
                                         NCollection_Array1<int> done,
                                         int index,
                                         double toler)
    {
        int k, jj, nbvtx;
        PatchLineType theType;

        WalkingTransition transline = new(), transarc = new();
        PatchPoint intpt;
        Curve2DAdaptorBase currentarc;
        HVertex? vtx = null, vtxbis;
        Point3D ptbid = new();
        Vector3D vtgrst = new();

        Vector3D d1u = new(), d1v = new();

        int nbpnt = listpnt.Length();
        ThePathPointOfTheSOnBounds currentpointonrst = listpnt.Value(index);
        double currentparameter;

        bool goon;
        bool retvalue = true;

        theType = lin.ArcType();
        nbvtx = NbVertexOf(lin, theType);
        jj = 1;
        while (jj <= nbvtx)
        {
            intpt = VertexOf(lin, theType, jj);
            if (!intpt.IsOnDomS2)
            {
                if (currentpointonrst.Value().DistanceTo(intpt.Value()) <= intpt.Tolerance())
                {
                    retvalue = false;
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
                    currentarc.D1(currentparameter, out Point2D p2d, out Vector2D d2d);
                    quadSurf.D1(p2d.X, p2d.Y, ref ptbid, ref d1u, ref d1v);
                    vtgrst = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
                    if (normale.SquareMagnitude < 1e-16)
                    {
                        transline.SetValue(true, WalkingTypeTrans.Undecided);
                        transarc.SetValue(true, WalkingTypeTrans.Undecided);
                    }
                    else
                    {
                        global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(vtgint, vtgrst, new Direction3D(normale), ref transline, ref transarc);
                    }
                    intpt.SetArc(false, currentarc, currentparameter, transline, transarc);
                    intpt.SetTolerance(toler);

                    ReplaceVertexOf(lin, theType, jj, intpt);
                    done[index] = 1;

                    if (goon)
                    {
                        for (k = index + 1; k <= nbpnt; k++)
                        {
                            if (done[k] != 1)
                            {
                                currentpointonrst = listpnt.Value(k);
                                if (!currentpointonrst.IsNew())
                                {
                                    vtxbis = currentpointonrst.Vertex();
                                    if (domain.Identical(vtx!, vtxbis))
                                    {
                                        intpt.SetVertex(false, vtxbis);
                                        currentarc = currentpointonrst.Arc();
                                        currentparameter = currentpointonrst.Parameter();
                                        currentarc.D1(currentparameter, out p2d, out d2d);
                                        vtgrst = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
                                        if (normale.SquareMagnitude < 1e-16)
                                        {
                                            transline.SetValue(true, WalkingTypeTrans.Undecided);
                                            transarc.SetValue(true, WalkingTypeTrans.Undecided);
                                        }
                                        else
                                        {
                                            global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(vtgint, vtgrst, new Direction3D(normale), ref transline,
                                                                   ref transarc);
                                        }
                                        intpt.SetArc(false, currentarc, currentparameter, transline, transarc);
                                        intpt.SetTolerance(toler);
                                        AddVertexTo(lin, theType, intpt);
                                        done[k] = 1;
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
            nbvtx = NbVertexOf(lin, theType);
        }
        return retvalue;
    }

    private static bool FindLine(ref Point3D psurf,
                                 NCollection_Sequence<PatchLine> slin,
                                 double tol,
                                 NCollection_List<double> lParams,
                                 ref Vector3D vtgtint,
                                 ref int lineIdx,
                                 int onlyThisLine,
                                 Curve2DAdaptorBase thearc,
                                 ref double theparameteronarc,
                                 ref Point3D thepointonarc,
                                 in SurfaceQuadric quadSurf1,
                                 in SurfaceQuadric quadSurf2,
                                 ref double outputToler)
    {
        if ((quadSurf1.Distance(psurf) > tol) || (quadSurf2.Distance(psurf) > tol))
        {
            return false;
        }

        // Traitement du point de depart ayant pour representation Psurf
        // dans l espace. On recherche la ligne d intersection contenant ce point.
        // On a en sortie la ligne, et le parametre et sa tangente du point sur
        // la ligne d intersection.
        double sqTol = tol * tol;
        double sqDistMin = RealLast();
        double sqDist, para = 0.0;
        double lower, upper;
        Point3D pt = new();
        int i;
        PatchLineType typarc;

        double paraInt = RealLast();
        int nblin = slin.Length();
        for (i = 1; i <= nblin; i++)
        {
            if (onlyThisLine != 0)
            {
                i = onlyThisLine;
                nblin = 0;
            }
            PatchLine lin = slin.Value(i);
            typarc = lin.ArcType();
            if (typarc == PatchLineType.Analytic)
            {
                lower = ((ALine)lin).FirstParameter(out _);
                upper = ((ALine)lin).LastParameter(out _);
            }
            else
            {
                if (((GLine)lin).HasFirstPoint)
                {
                    lower = ((GLine)lin).FirstPoint().ParameterOnLine();
                }
                else
                {
                    lower = RealFirst();
                }
                if (((GLine)lin).HasLastPoint)
                {
                    upper = ((GLine)lin).LastPoint().ParameterOnLine();
                }
                else
                {
                    upper = RealLast();
                }
            }

            switch (typarc)
            {
                case PatchLineType.Lin:
                    {
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(((GLine)lin).Line(), psurf);
                        if (para <= upper && para >= lower)
                        {
                            pt = global::Kerf.Engine.Geometry.ElCLib.Value(para, ((GLine)lin).Line());
                            sqDist = psurf.SquareDistanceTo(pt);
                            if ((sqDist < sqTol) && (sqDist < sqDistMin))
                            {
                                sqDistMin = sqDist;
                                paraInt = para;
                                lineIdx = i;
                            }
                        }
                    }
                    break;
                case PatchLineType.Circle:
                    {
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(((GLine)lin).Circle(), psurf);
                        if ((para <= upper && para >= lower)
                            || (para + 2.0 * M_PI <= upper && para + 2.0 * M_PI >= lower)
                            || (para - 2.0 * M_PI <= upper && para - 2.0 * M_PI >= lower))
                        {
                            pt = global::Kerf.Engine.Geometry.ElCLib.Value(para, ((GLine)lin).Circle());
                            sqDist = psurf.SquareDistanceTo(pt);
                            if ((sqDist < sqTol) && (sqDist < sqDistMin))
                            {
                                sqDistMin = sqDist;
                                paraInt = para;
                                lineIdx = i;
                            }
                        }
                    }
                    break;
                case PatchLineType.Ellipse:
                    {
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(((GLine)lin).Ellipse(), psurf);
                        if ((para <= upper && para >= lower)
                            || (para + 2.0 * M_PI <= upper && para + 2.0 * M_PI >= lower)
                            || (para - 2.0 * M_PI <= upper && para - 2.0 * M_PI >= lower))
                        {
                            pt = global::Kerf.Engine.Geometry.ElCLib.Value(para, ((GLine)lin).Ellipse());
                            sqDist = psurf.SquareDistanceTo(pt);
                            if ((sqDist < sqTol) && (sqDist < sqDistMin))
                            {
                                sqDistMin = sqDist;
                                paraInt = para;
                                lineIdx = i;
                            }
                        }
                    }
                    break;
                case PatchLineType.Parabola:
                    {
                        //-- Le calcul du parametre sur une parabole est mal fait ds ElCLib. Il ne tient pas compte
                        //-- de la meilleure facon de calculer (axe X ou axe Y). Bilan : Si la parabole est tres
                        //-- pointue (focal de l'ordre de 1e-2 et si le point est a un parametre grand, ca foire. )
                        //-- On ne peut pas modifier faciolement ds ElCLib car on ne passe pas la focale. ...
                        Parabola3D parab = ((GLine)lin).Parabola();
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(parab, psurf);
                        if (para <= upper && para >= lower)
                        {
                            int amelioration = 0;
                            do
                            {
                                double parabis = para + 0.0000001;

                                pt = global::Kerf.Engine.Geometry.ElCLib.Value(para, parab);
                                sqDist = psurf.SquareDistanceTo(pt);

                                Point3D ptbis = global::Kerf.Engine.Geometry.ElCLib.Value(parabis, parab);
                                double distbis = psurf.DistanceTo(ptbis);
                                double dist = Math.Sqrt(sqDist);
                                double ddist = distbis - dist;

                                if ((sqDist < sqTol) && (sqDist < sqDistMin))
                                {
                                    sqDistMin = sqDist;
                                    paraInt = para;
                                    lineIdx = i;
                                }
                                if (sqDist < global::Kerf.Engine.Foundation.Precision.SquarePConfusion())
                                {
                                    amelioration = 100;
                                }

                                if (ddist > 1.0e-9 || ddist < -1.0e-9)
                                {
                                    para = para - dist * (parabis - para) / ddist;
                                }
                                else
                                {
                                    amelioration = 100;
                                }
                            } while (++amelioration < 5);
                        }
                    }
                    break;
                case PatchLineType.Hyperbola:
                    {
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(((GLine)lin).Hyperbola(), psurf);
                        if (para <= upper && para >= lower)
                        {
                            pt = global::Kerf.Engine.Geometry.ElCLib.Value(para, ((GLine)lin).Hyperbola());
                            sqDist = psurf.SquareDistanceTo(pt);
                            if ((sqDist < sqTol) && (sqDist < sqDistMin))
                            {
                                sqDistMin = sqDist;
                                paraInt = para;
                                lineIdx = i;
                            }
                        }
                    }
                    break;

                case PatchLineType.Analytic:
                    {
                        ALine alin = (ALine)lin;
                        NCollection_List<double> aLParams = new();
                        alin.FindParameter(psurf, aLParams);
                        if (!aLParams.IsEmpty())
                        {
                            // All found distances are already in some internal tolerance
                            // set in alin->FindParameter(...) method.

                            sqDist = RealLast();
                            for (var itr = new NCollection_List<double>.Iterator(aLParams); itr.More(); itr.Next())
                            {
                                pt = alin.Value(itr.Value());
                                double sqD = psurf.SquareDistanceTo(pt);
                                if (sqD < sqDist)
                                {
                                    sqDist = sqD;
                                }
                            }

                            if (sqDist < sqDistMin)
                            {
                                sqDistMin = sqDist;
                                lParams.Assign(aLParams);
                                lineIdx = i;
                            }
                        }
                        else
                        {
                            //-- le point n a pas ete trouve par bete projection.
                            //-- on essaie l intersection avec la restriction en 2d
                            double theparamonarc = theparameteronarc;
                            Point3D copiePsurf = psurf;
                            bool intersectIsOk = IntersectionWithAnArc(ref copiePsurf,
                                                                       alin,
                                                                       ref para,
                                                                       thearc,
                                                                       ref theparamonarc,
                                                                       ref thepointonarc,
                                                                       quadSurf1,
                                                                       lower,
                                                                       upper);
                            sqDist = copiePsurf.SquareDistanceTo(psurf);
                            if (intersectIsOk)
                            {
                                if (sqDist < sqTol)
                                {
                                    theparameteronarc = theparamonarc;
                                    psurf = thepointonarc;
                                    sqDistMin = sqDist;
                                    lParams.Append(para);
                                    lineIdx = i;
                                }
                            }
                        }
                    }
                    break;

                case PatchLineType.Walking: // impossible . c est pour eviter les warnings
                case PatchLineType.Restriction: // impossible . c est pour eviter les warnings
                    break;
            }
        }

        if (sqDistMin == RealLast())
        {
            return false;
        }

        outputToler = Max(outputToler, Math.Sqrt(sqDistMin));

        typarc = slin.Value(lineIdx).ArcType();

        // Computation of tangent vector
        switch (typarc)
        {
            case PatchLineType.Lin:
                lParams.Append(paraInt);
                vtgtint = new Vector3D(((GLine)slin[lineIdx]).Line().Direction);
                break;
            case PatchLineType.Circle:
                lParams.Append(paraInt);
                vtgtint = global::Kerf.Engine.Geometry.ElCLib.DN(paraInt, ((GLine)slin[lineIdx]).Circle(), 1);
                break;
            case PatchLineType.Ellipse:
                lParams.Append(paraInt);
                vtgtint = global::Kerf.Engine.Geometry.ElCLib.DN(paraInt, ((GLine)slin[lineIdx]).Ellipse(), 1);
                break;
            case PatchLineType.Parabola:
                lParams.Append(paraInt);
                vtgtint = global::Kerf.Engine.Geometry.ElCLib.DN(paraInt, ((GLine)slin[lineIdx]).Parabola(), 1);
                break;
            case PatchLineType.Hyperbola:
                lParams.Append(paraInt);
                vtgtint = global::Kerf.Engine.Geometry.ElCLib.DN(paraInt, ((GLine)slin[lineIdx]).Hyperbola(), 1);
                break;

            case PatchLineType.Analytic:
                {
                    if (!((ALine)slin[lineIdx]).D1(lParams.Last(), out pt, out vtgtint))
                    {
                        // Previously (before the fix #29807) this code tried to process case
                        // when occ::handle<IntPatch_ALine>::D1(...) method returns FALSE and
                        // computed Vtgtint input argument value. Currently, any singularities
                        // must be processed by high-level algorithms (IntPatch_SpecialPoints class).
                        // Therefore this code has been deleted as deprecated.

                        vtgtint = vtgtint.WithCoord(0.0, 0.0, 0.0);
                    }
                }
                break;
            case PatchLineType.Walking: // impossible . c est pour eviter les warnings
            case PatchLineType.Restriction: // impossible . c est pour eviter les warnings
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
    private static bool SingleLine(in Point3D psurf,
                                   PatchLine lin,
                                   double tol,
                                   ref double paraint,
                                   ref Vector3D vtgtint)
    {
        PatchLineType typarc = lin.ArcType();

        double parproj = 0.0;
        Vector3D tgint = new();
        Point3D ptproj = new();
        bool retvalue;

        switch (typarc)
        {
            case PatchLineType.Lin:
                parproj = global::Kerf.Engine.Geometry.ElCLib.Parameter(((GLine)lin).Line(), psurf);
                global::Kerf.Engine.Geometry.ElCLib.D1(parproj, ((GLine)lin).Line(), out ptproj, out tgint);
                break;
            case PatchLineType.Circle:
                parproj = global::Kerf.Engine.Geometry.ElCLib.Parameter(((GLine)lin).Circle(), psurf);
                global::Kerf.Engine.Geometry.ElCLib.D1(parproj, ((GLine)lin).Circle(), out ptproj, out tgint);
                break;
            case PatchLineType.Ellipse:
                parproj = global::Kerf.Engine.Geometry.ElCLib.Parameter(((GLine)lin).Ellipse(), psurf);
                global::Kerf.Engine.Geometry.ElCLib.D1(parproj, ((GLine)lin).Ellipse(), out ptproj, out tgint);
                break;
            case PatchLineType.Parabola:
                parproj = global::Kerf.Engine.Geometry.ElCLib.Parameter(((GLine)lin).Parabola(), psurf);
                global::Kerf.Engine.Geometry.ElCLib.D1(parproj, ((GLine)lin).Parabola(), out ptproj, out tgint);
                break;
            case PatchLineType.Hyperbola:
                parproj = global::Kerf.Engine.Geometry.ElCLib.Parameter(((GLine)lin).Hyperbola(), psurf);
                global::Kerf.Engine.Geometry.ElCLib.D1(parproj, ((GLine)lin).Hyperbola(), out ptproj, out tgint);
                break;
            case PatchLineType.Analytic:
                {
                    ALine alin = (ALine)lin;
                    NCollection_List<double> lParams = new();
                    alin.FindParameter(psurf, lParams);
                    if (!lParams.IsEmpty())
                    {
                        ptproj = psurf;
                        parproj = lParams.Last();
                        if (!alin.D1(parproj, out _, out tgint))
                        {
                            // Previously (before the fix #29807) this code tried to process case
                            // when occ::handle<IntPatch_ALine>::D1(...) method returns FALSE and
                            // computed Vtgtint input argument value. Currently, any singularities
                            // must be processed by high-level algorithms (IntPatch_SpecialPoints class).
                            // Therefore this code has been deleted as deprecated.

                            tgint = tgint.WithCoord(0.0, 0.0, 0.0);
                        }
                    }
                    else
                    {
                        return false;
                    }
                }
                break;
            case PatchLineType.Walking: // impossible . c est pour eviter les warnings
            case PatchLineType.Restriction: // impossible . c est pour eviter les warnings
                break;
        }

        if (psurf.DistanceTo(ptproj) <= tol)
        {
            paraint = parproj;
            vtgtint = tgint;
            retvalue = true;
        }
        else
        {
            retvalue = false;
        }
        return retvalue;
    }
}
