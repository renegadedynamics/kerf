// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntAna_ResultType;
using static Occt.IntPatch_IType;
using static Occt.IntSurf_Situation;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

// The quadric pairs other than plane/quadric and cylinder/cylinder: cylinder/sphere, cylinder/cone, cone/cone,
// cone/sphere, sphere/sphere and the tori (OCCT 7.x's IntPatch_ImpImpIntersection_5.gxx), and ProcessBounds, which
// puts the end points of their analytic lines.
public sealed partial class IntPatch_ImpImpIntersection
{
    //=======================================================================
    // function : ProcessBounds
    // purpose  : Puts the first and last points of the analytic line alig as vertices, sharing the vertices of
    //            the analytic lines already in slin that coincide with them.
    //=======================================================================
    private static void ProcessBounds(IntPatch_ALine alig, //-- ligne courante
                                      NCollection_Sequence<IntPatch_Line> slin,
                                      in IntSurf_Quadric Quad1,
                                      in IntSurf_Quadric Quad2,
                                      ref bool procf,
                                      in gp_Pnt ptf,   //-- Debut Ligne Courante
                                      double first,    //-- Paramf
                                      ref bool procl,
                                      in gp_Pnt ptl,   //-- Fin Ligne courante
                                      double last,     //-- Paraml
                                      ref bool Multpoint,
                                      double Tol)
    {
        int j, k;
        double U1 = 0, V1 = 0, U2 = 0, V2 = 0;
        IntPatch_Point ptsol = new();
        double d;

        if (procf && procl)
        {
            j = slin.Length() + 1;
        }
        else
        {
            j = 1;
        }

        //-- On prend les lignes deja enregistrees

        while (j <= slin.Length())
        {
            // OCCT: j advances only on analytic lines, so a line of another type in slin with procf or procl
            // still false loops forever. The callers here put only ALines in slin before calling.
            if (slin.Value(j).ArcType() == IntPatch_Analytic)
            {
                IntPatch_ALine aligold = (IntPatch_ALine)slin.Value(j);
                k = 1;

                //-- On prend les vertex des lignes deja enregistrees

                while (k <= aligold.NbVertex())
                {
                    ptsol = aligold.Vertex(k);
                    if (!procf)
                    {
                        d = ptf.Distance(ptsol.Value());
                        if (d <= Tol)
                        {
                            ptsol.SetTolerance(Tol);
                            if (!ptsol.IsMultiple())
                            {
                                //-- le point ptsol (de aligold) est declare multiple sur aligold
                                Multpoint = true;
                                ptsol.SetMultiple(true);
                                aligold.Replace(k, ptsol);
                            }
                            ptsol.SetParameter(first);
                            alig.AddVertex(ptsol);
                            alig.SetFirstPoint(alig.NbVertex());
                            procf = true;

                            //-- On restore le point avec son parametre sur aligold
                            ptsol = aligold.Vertex(k);
                        }
                    }
                    if (!procl)
                    {
                        if (ptl.Distance(ptsol.Value()) <= Tol)
                        {
                            ptsol.SetTolerance(Tol);
                            if (!ptsol.IsMultiple())
                            {
                                Multpoint = true;
                                ptsol.SetMultiple(true);
                                aligold.Replace(k, ptsol);
                            }
                            ptsol.SetParameter(last);
                            alig.AddVertex(ptsol);
                            alig.SetLastPoint(alig.NbVertex());
                            procl = true;

                            //-- On restore le point avec son parametre sur aligold
                            ptsol = aligold.Vertex(k);
                        }
                    }
                    if (procf && procl)
                    {
                        k = aligold.NbVertex() + 1;
                    }
                    else
                    {
                        k = k + 1;
                    }
                }
                if (procf && procl)
                {
                    j = slin.Length() + 1;
                }
                else
                {
                    j = j + 1;
                }
            }
        }

        ptsol.SetTolerance(Tol);
        if (!procf && !procl)
        {
            Quad1.Parameters(ptf, ref U1, ref V1);
            Quad2.Parameters(ptf, ref U2, ref V2);
            ptsol.SetValue(ptf, Tol, false);
            ptsol.SetParameters(U1, V1, U2, V2);
            ptsol.SetParameter(first);
            if (ptf.Distance(ptl) <= Tol)
            {
                ptsol.SetMultiple(true); // a voir
                Multpoint = true;        // a voir de meme
                alig.AddVertex(ptsol);
                alig.SetFirstPoint(alig.NbVertex());

                ptsol.SetParameter(last);
                alig.AddVertex(ptsol);
                alig.SetLastPoint(alig.NbVertex());
            }
            else
            {
                alig.AddVertex(ptsol);
                alig.SetFirstPoint(alig.NbVertex());
                Quad1.Parameters(ptl, ref U1, ref V1);
                Quad2.Parameters(ptl, ref U2, ref V2);
                ptsol.SetValue(ptl, Tol, false);
                ptsol.SetParameters(U1, V1, U2, V2);
                ptsol.SetParameter(last);
                alig.AddVertex(ptsol);
                alig.SetLastPoint(alig.NbVertex());
            }
        }
        else if (!procf)
        {
            Quad1.Parameters(ptf, ref U1, ref V1);
            Quad2.Parameters(ptf, ref U2, ref V2);
            ptsol.SetValue(ptf, Tol, false);
            ptsol.SetParameters(U1, V1, U2, V2);
            ptsol.SetParameter(first);
            alig.AddVertex(ptsol);
            alig.SetFirstPoint(alig.NbVertex());
        }
        else if (!procl)
        {
            Quad1.Parameters(ptl, ref U1, ref V1);
            Quad2.Parameters(ptl, ref U2, ref V2);
            ptsol.SetValue(ptl, Tol, false);
            ptsol.SetParameters(U1, V1, U2, V2);
            ptsol.SetParameter(last);
            alig.AddVertex(ptsol);
            alig.SetLastPoint(alig.NbVertex());
        }
    }

    //=================================================================================================

    private static bool IntCySp(in IntSurf_Quadric Quad1,
                                in IntSurf_Quadric Quad2,
                                double Tol,
                                bool Reversed,
                                ref bool Empty,
                                ref bool Multpoint,
                                NCollection_Sequence<IntPatch_Line> slin,
                                NCollection_Sequence<IntPatch_Point> spnt)
    {
        int i;

        IntSurf_TypeTrans trans1, trans2;
        IntAna_ResultType typint;
        IntPatch_Point ptsol = new();
        gp_Circ cirsol;

        gp_Cylinder Cy;
        gp_Sphere Sp;

        if (!Reversed)
        {
            Cy = Quad1.Cylinder();
            Sp = Quad2.Sphere();
        }
        else
        {
            Cy = Quad2.Cylinder();
            Sp = Quad1.Sphere();
        }
        IntAna_QuadQuadGeo inter = new(Cy, Sp, Tol);

        if (!inter.IsDone())
        {
            return false;
        }

        typint = inter.TypeInter();
        int NbSol = inter.NbSolutions();
        Empty = false;

        switch (typint)
        {
            case IntAna_Empty:
            {
                Empty = true;
            }
            break;

            case IntAna_Point:
            {
                gp_Pnt psol = inter.Point(1);
                double U1 = 0, V1 = 0, U2 = 0, V2 = 0;
                Quad1.Parameters(psol, ref U1, ref V1);
                Quad2.Parameters(psol, ref U2, ref V2);
                ptsol.SetValue(psol, Tol, true);
                ptsol.SetParameters(U1, V1, U2, V2);
                spnt.Append(ptsol);
            }
            break;

            case IntAna_Circle:
            {
                cirsol = inter.Circle(1);
                ElCLib.D1(0.0, cirsol, out gp_Pnt ptref, out gp_Vec Tgt);

                if (NbSol == 1)
                {
                    gp_Vec TestCurvature = new(ptref, Sp.Location());
                    gp_Vec Normsp, Normcyl;
                    if (!Reversed)
                    {
                        Normcyl = Quad1.Normale(ptref);
                        Normsp = Quad2.Normale(ptref);
                    }
                    else
                    {
                        Normcyl = Quad2.Normale(ptref);
                        Normsp = Quad1.Normale(ptref);
                    }

                    IntSurf_Situation situcyl;
                    IntSurf_Situation situsp;

                    if (Normcyl.Dot(TestCurvature) > 0.0)
                    {
                        situsp = IntSurf_Outside;
                        if (Normsp.Dot(Normcyl) > 0.0)
                        {
                            situcyl = IntSurf_Inside;
                        }
                        else
                        {
                            situcyl = IntSurf_Outside;
                        }
                    }
                    else
                    {
                        situsp = IntSurf_Inside;
                        if (Normsp.Dot(Normcyl) > 0.0)
                        {
                            situcyl = IntSurf_Outside;
                        }
                        else
                        {
                            situcyl = IntSurf_Inside;
                        }
                    }
                    IntPatch_GLine glig;
                    if (!Reversed)
                    {
                        glig = new IntPatch_GLine(cirsol, true, situcyl, situsp);
                    }
                    else
                    {
                        glig = new IntPatch_GLine(cirsol, true, situsp, situcyl);
                    }
                    slin.Append(glig);
                }
                else
                {
                    if (Tgt.DotCross(Quad2.Normale(ptref), Quad1.Normale(ptref)) > 0.0)
                    {
                        trans1 = IntSurf_Out;
                        trans2 = IntSurf_In;
                    }
                    else
                    {
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                    }
                    IntPatch_GLine glig = new(cirsol, false, trans1, trans2);
                    slin.Append(glig);

                    cirsol = inter.Circle(2);
                    ElCLib.D1(0.0, cirsol, out ptref, out Tgt);
                    double qwe = Tgt.DotCross(Quad2.Normale(ptref), Quad1.Normale(ptref));
                    if (qwe > 0.0000001)
                    {
                        trans1 = IntSurf_Out;
                        trans2 = IntSurf_In;
                    }
                    else if (qwe < -0.0000001)
                    {
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                    }
                    else
                    {
                        trans1 = trans2 = IntSurf_Undecided;
                    }
                    glig = new IntPatch_GLine(cirsol, false, trans1, trans2);
                    slin.Append(glig);
                }
            }
            break;

            case IntAna_NoGeometricSolution:
            {
                gp_Pnt psol;
                double U1 = 0, V1 = 0, U2 = 0, V2 = 0;
                IntAna_IntQuadQuad anaint = new(Cy, new IntAna_Quadric(Sp), Tol);
                if (!anaint.IsDone())
                {
                    return false;
                }

                if (anaint.NbPnt() == 0 && anaint.NbCurve() == 0)
                {
                    Empty = true;
                }
                else
                {
                    NbSol = anaint.NbPnt();
                    for (i = 1; i <= NbSol; i++)
                    {
                        psol = anaint.Point(i);
                        Quad1.Parameters(psol, ref U1, ref V1);
                        Quad2.Parameters(psol, ref U2, ref V2);
                        ptsol.SetValue(psol, Tol, true);
                        ptsol.SetParameters(U1, V1, U2, V2);
                        spnt.Append(ptsol);
                    }

                    gp_Pnt ptvalid = new(), ptf, ptl;
                    gp_Vec tgvalid = new();
                    double first, last, para;
                    IntAna_Curve curvsol;
                    bool tgfound;
                    int kount;

                    NbSol = anaint.NbCurve();
                    for (i = 1; i <= NbSol; i++)
                    {
                        curvsol = anaint.Curve(i);
                        curvsol.Domain(out first, out last);
                        ptf = curvsol.Value(first);
                        ptl = curvsol.Value(last);

                        para = last;
                        kount = 1;
                        tgfound = false;

                        while (!tgfound)
                        {
                            para = (1.123 * first + para) / 2.123;
                            tgfound = curvsol.D1u(para, out ptvalid, out tgvalid);
                            if (!tgfound)
                            {
                                kount++;
                                tgfound = kount > 5;
                            }
                        }
                        IntPatch_ALine alig;
                        if (kount <= 5)
                        {
                            double qwe = tgvalid.DotCross(Quad2.Normale(ptvalid), Quad1.Normale(ptvalid));
                            if (qwe > 0.00000001)
                            {
                                trans1 = IntSurf_Out;
                                trans2 = IntSurf_In;
                            }
                            else if (qwe < -0.00000001)
                            {
                                trans1 = IntSurf_In;
                                trans2 = IntSurf_Out;
                            }
                            else
                            {
                                trans1 = trans2 = IntSurf_Undecided;
                            }
                            alig = new IntPatch_ALine(curvsol, false, trans1, trans2);
                        }
                        else
                        {
                            alig = new IntPatch_ALine(curvsol, false);
                        }
                        bool TempFalse1a = false;
                        bool TempFalse2a = false;

                        //-- ptf et ptl : points debut et fin de alig

                        ProcessBounds(alig,
                                      slin,
                                      Quad1,
                                      Quad2,
                                      ref TempFalse1a,
                                      ptf,
                                      first,
                                      ref TempFalse2a,
                                      ptl,
                                      last,
                                      ref Multpoint,
                                      Tol);
                        slin.Append(alig);
                    } //-- boucle sur les lignes
                } //-- solution avec au moins une lihne
            }
            break;

            default:
            {
                return false;
            }
        }
        return true;
    }

    //=================================================================================================

    private static bool IntCyCo(in IntSurf_Quadric Quad1,
                                in IntSurf_Quadric Quad2,
                                double Tol,
                                bool Reversed,
                                ref bool Empty,
                                ref bool Multpoint,
                                NCollection_Sequence<IntPatch_Line> slin,
                                NCollection_Sequence<IntPatch_Point> spnt)
    {
        IntPatch_Point ptsol = new();

        int i;

        IntSurf_TypeTrans trans1, trans2;
        IntAna_ResultType typint;
        gp_Circ cirsol;

        gp_Cylinder Cy;
        gp_Cone Co;

        if (!Reversed)
        {
            Cy = Quad1.Cylinder();
            Co = Quad2.Cone();
        }
        else
        {
            Cy = Quad2.Cylinder();
            Co = Quad1.Cone();
        }
        IntAna_QuadQuadGeo inter = new(Cy, Co, Tol);

        if (!inter.IsDone())
        {
            return false;
        }

        typint = inter.TypeInter();
        int NbSol = inter.NbSolutions();
        Empty = false;

        switch (typint)
        {
            case IntAna_Empty:
            {
                Empty = true;
            }
            break;

            case IntAna_Point:
            {
                gp_Pnt psol = inter.Point(1);
                double U1 = 0, V1 = 0, U2 = 0, V2 = 0;
                Quad1.Parameters(psol, ref U1, ref V1);
                Quad1.Parameters(psol, ref U2, ref V2); // OCCT: Quad1 for the second surface's parameters
                ptsol.SetValue(psol, Tol, true);
                ptsol.SetParameters(U1, V1, U2, V2);
                spnt.Append(ptsol);
            }
            break;

            case IntAna_Circle:
            {
                gp_Vec Tgt;
                gp_Pnt ptref;
                int j;
                double qwe;
                //
                for (j = 1; j <= 2; ++j)
                {
                    cirsol = inter.Circle(j);
                    ElCLib.D1(0.0, cirsol, out ptref, out Tgt);
                    qwe = Tgt.DotCross(Quad2.Normale(ptref), Quad1.Normale(ptref));
                    if (qwe > 0.00000001)
                    {
                        trans1 = IntSurf_Out;
                        trans2 = IntSurf_In;
                    }
                    else if (qwe < -0.00000001)
                    {
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                    }
                    else
                    {
                        trans1 = trans2 = IntSurf_Undecided;
                    }
                    IntPatch_GLine glig = new(cirsol, false, trans1, trans2);
                    slin.Append(glig);
                }
            }
            break;

            case IntAna_NoGeometricSolution:
            {
                gp_Pnt psol;
                double U1 = 0, V1 = 0, U2 = 0, V2 = 0;
                IntAna_IntQuadQuad anaint = new(Cy, new IntAna_Quadric(Co), Tol);
                if (!anaint.IsDone())
                {
                    return false;
                }

                if (anaint.NbPnt() == 0 && anaint.NbCurve() == 0)
                {
                    Empty = true;
                }
                else
                {
                    NbSol = anaint.NbPnt();
                    for (i = 1; i <= NbSol; i++)
                    {
                        psol = anaint.Point(i);
                        Quad1.Parameters(psol, ref U1, ref V1);
                        Quad2.Parameters(psol, ref U2, ref V2);
                        ptsol.SetValue(psol, Tol, true);
                        ptsol.SetParameters(U1, V1, U2, V2);
                        spnt.Append(ptsol);
                    }

                    // ptf and ptl keep their values from the previous curve where the bound is open, as OCCT's.
                    // OCCT: kept across the curves; an open bound passes the previous curve's point
                    gp_Pnt ptvalid = new(), ptf = new(), ptl = new();
                    gp_Vec tgvalid = new();
                    //
                    double first, last, para;
                    bool tgfound, firstp, lastp, kept;
                    int kount;
                    //
                    //
                    // IntAna_Curve curvsol;
                    IntAna_Curve aC;
                    NCollection_List<IntAna_Curve> aLC = new();
                    NCollection_List<IntAna_Curve>.Iterator aIt = new();

                    //
                    NbSol = anaint.NbCurve();
                    for (i = 1; i <= NbSol; ++i)
                    {
                        kept = false;
                        // curvsol = anaint.Curve(i);
                        aC = anaint.Curve(i);
                        aLC.Clear();
                        ExploreCurve(Co, ref aC, 10.0 * Tol, aLC);
                        //
                        aIt.Initialize(aLC);
                        for (; aIt.More(); aIt.Next())
                        {
                            ref IntAna_Curve curvsol = ref aIt.ChangeValue();
                            //
                            curvsol.Domain(out first, out last);
                            firstp = !curvsol.IsFirstOpen();
                            lastp = !curvsol.IsLastOpen();
                            if (firstp)
                            {
                                ptf = curvsol.Value(first);
                            }
                            if (lastp)
                            {
                                ptl = curvsol.Value(last);
                            }
                            para = last;
                            kount = 1;
                            tgfound = false;

                            while (!tgfound)
                            {
                                para = (1.123 * first + para) / 2.123;
                                tgfound = curvsol.D1u(para, out ptvalid, out tgvalid);
                                if (!tgfound)
                                {
                                    kount++;
                                    tgfound = kount > 5;
                                }
                            }
                            IntPatch_ALine alig;
                            if (kount <= 5)
                            {
                                double qwe = tgvalid.DotCross(Quad2.Normale(ptvalid), Quad1.Normale(ptvalid));
                                if (qwe > 0.00000001)
                                {
                                    trans1 = IntSurf_Out;
                                    trans2 = IntSurf_In;
                                }
                                else if (qwe < -0.00000001)
                                {
                                    trans1 = IntSurf_In;
                                    trans2 = IntSurf_Out;
                                }
                                else
                                {
                                    trans1 = trans2 = IntSurf_Undecided;
                                }
                                alig = new IntPatch_ALine(curvsol, false, trans1, trans2);
                                kept = true;
                            }
                            else
                            {
                                ptvalid = curvsol.Value(para);
                                alig = new IntPatch_ALine(curvsol, false);
                                kept = true;
                                //-- std::cout << "Transition indeterminee" << std::endl;
                            }
                            if (kept)
                            {
                                bool Nfirstp = !firstp;
                                bool Nlastp = !lastp;
                                ProcessBounds(alig,
                                              slin,
                                              Quad1,
                                              Quad2,
                                              ref Nfirstp,
                                              ptf,
                                              first,
                                              ref Nlastp,
                                              ptl,
                                              last,
                                              ref Multpoint,
                                              Tol);
                                slin.Append(alig);
                            }
                        } // for (; aIt.More(); aIt.Next())
                    } // for (i = 1; i <= NbSol; ++i)
                }
            }
            break;

            default:
                return false;
        } // switch (typint)

        return true;
    }

    //=======================================================================
    // function : ExploreCurve
    // purpose  : Splits aC on several curves in the cone apex points.
    //=======================================================================
    private static bool ExploreCurve(in gp_Cone theCo,
                                     ref IntAna_Curve theCrv,
                                     double theTol,
                                     NCollection_List<IntAna_Curve> theLC)
    {
        double aSqTol = theTol * theTol;
        gp_Pnt aPapx = theCo.Apex();

        theCrv.Domain(out double aT1, out double aT2);

        theLC.Clear();
        //
        NCollection_List<double> aLParams = new();
        theCrv.FindParameter(aPapx, aLParams);
        if (aLParams.IsEmpty())
        {
            theLC.Append(theCrv);
            return false;
        }

        for (NCollection_List<double>.Iterator anItr = new(aLParams); anItr.More(); anItr.Next())
        {
            double aPrm = anItr.Value();

            if ((aPrm - aT1) < Precision.PConfusion())
            {
                continue;
            }

            bool isLast = false;
            if ((aT2 - aPrm) < Precision.PConfusion())
            {
                aPrm = aT2;
                isLast = true;
            }

            gp_Pnt aP = theCrv.Value(aPrm);
            double aSqD = aP.SquareDistance(aPapx);
            if (aSqD < aSqTol)
            {
                IntAna_Curve aC1 = theCrv;
                aC1.SetDomain(aT1, aPrm);
                aT1 = aPrm;
                theLC.Append(aC1);
            }

            if (isLast)
            {
                break;
            }
        }

        if (theLC.IsEmpty())
        {
            theLC.Append(theCrv);
            return false;
        }

        if ((aT2 - aT1) > Precision.PConfusion())
        {
            IntAna_Curve aC1 = theCrv;
            aC1.SetDomain(aT1, aT2);
            theLC.Append(aC1);
        }

        return true;
    }

    //=================================================================================================

    private static bool IntCoCo(in IntSurf_Quadric Quad1,
                                in IntSurf_Quadric Quad2,
                                double Tol,
                                ref bool Empty,
                                ref bool Same,
                                ref bool Multpoint,
                                NCollection_Sequence<IntPatch_Line> slin,
                                NCollection_Sequence<IntPatch_Point> spnt)
    {
        int i, NbSol;
        double U1 = 0, V1 = 0, U2 = 0, V2 = 0;
        IntSurf_TypeTrans trans1, trans2;
        IntAna_ResultType typint;
        //
        gp_Cone Co1 = Quad1.Cone();
        gp_Cone Co2 = Quad2.Cone();
        //
        IntAna_QuadQuadGeo inter = new(Co1, Co2, Tol);
        if (!inter.IsDone())
        {
            return false;
        }
        //
        typint = inter.TypeInter();
        NbSol = inter.NbSolutions();
        Empty = false;
        Same = false;

        switch (typint)
        {
            case IntAna_Empty:
            {
                Empty = true;
            }
            break;

            case IntAna_Same:
            {
                Same = true;
            }
            break;

            // modified by NIZNHY-PKV Wed Nov 30 12:56:06 2005f
            case IntAna_Line:
            {
                double para, aDot;
                gp_Pnt aPApex1, aPApex2, ptbid;
                gp_Lin linsol;
                gp_Vec NormC1, NormC2;
                IntPatch_Point aPtsol = new();
                IntPatch_GLine glig;
                //
                aPApex1 = Co1.Apex();
                aPApex2 = Co2.Apex();
                //
                if (NbSol == 1)
                {
                    IntSurf_Situation situC1, situC2;
                    //
                    linsol = inter.Line(1);
                    para = ElCLib.Parameter(linsol, aPApex1);
                    ptbid = ElCLib.Value(para + 5.0, linsol);
                    Quad1.Parameters(aPApex1, ref U1, ref V1);
                    Quad2.Parameters(aPApex1, ref U2, ref V2);
                    //
                    aPtsol.SetValue(aPApex1, Tol, false);
                    aPtsol.SetParameters(U1, V1, U2, V2);
                    aPtsol.SetParameter(para);
                    //
                    NormC1 = Quad1.Normale(ptbid);
                    NormC2 = Quad2.Normale(ptbid);
                    aDot = NormC1.Dot(NormC2);
                    if (aDot < 0.0)
                    {
                        situC1 = IntSurf_Outside;
                        situC2 = IntSurf_Outside;
                    }
                    else
                    {
                        double aR1, aR2;
                        gp_Lin aLAx1 = new(aPApex1, Co1.Axis().Direction());
                        gp_Lin aLAx2 = new(aPApex2, Co2.Axis().Direction());
                        //
                        aR1 = aLAx1.Distance(ptbid);
                        aR2 = aLAx2.Distance(ptbid);
                        //
                        situC1 = IntSurf_Inside;
                        situC2 = IntSurf_Outside;
                        if (aR1 > aR2)
                        {
                            // Intersection line parametrizes from Apex1 to Apex2,
                            situC1 = IntSurf_Outside; // So the distance between ptbid and aLAx1 is greater than the
                            situC2 = IntSurf_Inside;  // distance between ptbid and aLAx2 and in that case Cone2
                                                      // is inside Cone 1
                        }
                    }
                    // 1
                    glig = new IntPatch_GLine(linsol, true, situC1, situC2);
                    glig.AddVertex(aPtsol);
                    glig.SetFirstPoint(1);
                    slin.Append(glig);
                    // 2
                    linsol.SetDirection(linsol.Direction().Reversed());
                    para = ElCLib.Parameter(linsol, aPApex1);
                    aPtsol.SetParameter(para);

                    glig = new IntPatch_GLine(linsol, true, situC2, situC1);
                    glig.AddVertex(aPtsol);
                    glig.SetFirstPoint(1);
                    slin.Append(glig);
                } // if (NbSol==1) {
                //////////////////////
                else if (NbSol == 2)
                {
                    //
                    for (i = 1; i <= 2; ++i)
                    {
                        linsol = inter.Line(i);
                        para = ElCLib.Parameter(linsol, aPApex1);
                        ptbid = ElCLib.Value(para + 5.0, linsol);
                        Quad1.Parameters(aPApex1, ref U1, ref V1);
                        Quad2.Parameters(aPApex1, ref U2, ref V2);
                        //
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                        // gp_Dir::DotCross: the normals are converted to (normalised) directions, as in C++.
                        if (linsol.Direction().DotCross(new gp_Dir(Quad2.Normale(ptbid)), new gp_Dir(Quad1.Normale(ptbid))) > 0.0)
                        {
                            trans1 = IntSurf_Out;
                            trans2 = IntSurf_In;
                        }
                        //
                        Multpoint = true;
                        // 1,3
                        aPtsol.SetValue(aPApex1, Tol, false);
                        aPtsol.SetParameters(U1, V1, U2, V2);
                        aPtsol.SetParameter(para);
                        aPtsol.SetMultiple(true);

                        glig = new IntPatch_GLine(linsol, false, trans1, trans2);
                        glig.AddVertex(aPtsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);
                        // 2,4
                        linsol.SetDirection(linsol.Direction().Reversed());
                        para = ElCLib.Parameter(linsol, aPApex1);
                        aPtsol.SetParameter(para);
                        glig = new IntPatch_GLine(linsol, false, trans1, trans2);
                        glig.AddVertex(aPtsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);
                        //
                    } // for (i=1; i<=2; ++i)
                } // else if (NbSol==2)
            }
            break;
            // modified by NIZNHY-PKV Wed Nov 30 12:56:10 2005t

            case IntAna_Point:
            {
                gp_Pnt ptcontact;
                gp_Pnt apex1 = Co1.Apex();
                gp_Pnt apex2 = Co2.Apex();
                double param1, param2;
                double paramapex1 = ElCLib.LineParameter(Co1.Axis(), apex1);
                double paramapex2 = ElCLib.LineParameter(Co2.Axis(), apex2);
                for (i = 1; i <= NbSol; i++)
                {
                    ptcontact = inter.Point(i);
                    param1 = ElCLib.LineParameter(Co1.Axis(), ptcontact);
                    param2 = ElCLib.LineParameter(Co2.Axis(), ptcontact);

                    Quad1.Parameters(ptcontact, ref U1, ref V1);
                    Quad2.Parameters(ptcontact, ref U2, ref V2);

                    if (apex1.Distance(ptcontact) <= Tol && apex2.Distance(ptcontact) <= Tol)
                    {
                        IntPatch_Point ptsol = new();
                        ptsol.SetValue(ptcontact, Tol, false);
                        ptsol.SetParameters(U1, V1, U2, V2);
                        spnt.Append(ptsol);
                    }
                    else if (param1 >= paramapex1 && param2 >= paramapex2)
                    {
                        IntPatch_Point ptsol = new();
                        ptsol.SetValue(ptcontact, Tol, true);
                        ptsol.SetParameters(U1, V1, U2, V2);
                        spnt.Append(ptsol);
                    }
                }
            }
            break;

            case IntAna_Circle:
            {
                IntPatch_Point aPtsol = new();
                gp_Vec Tgt;
                gp_Pnt ptref;
                for (i = 1; i <= NbSol; i++)
                {
                    gp_Circ cirsol = inter.Circle(i);
                    ElCLib.D1(0.0, cirsol, out ptref, out Tgt);
                    double qwe = Tgt.DotCross(Quad2.Normale(ptref), Quad1.Normale(ptref));
                    if (qwe > 0.00000001)
                    {
                        trans1 = IntSurf_Out;
                        trans2 = IntSurf_In;
                    }
                    else if (qwe < -0.00000001)
                    {
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                    }
                    else
                    {
                        trans1 = trans2 = IntSurf_Undecided;
                    }
                    IntPatch_GLine glig = new(cirsol, false, trans1, trans2);
                    if (inter.HasCommonGen())
                    {
                        gp_Pnt aPChar = inter.PChar();
                        Quad1.Parameters(aPChar, ref U1, ref V1);
                        Quad2.Parameters(aPChar, ref U2, ref V2);
                        aPtsol.SetValue(aPChar, Tol, false);
                        aPtsol.SetParameters(U1, V1, U2, V2);
                        aPtsol.SetParameter(0.0);
                        glig.AddVertex(aPtsol);
                    }
                    slin.Append(glig);
                }
            }
            break;

            case IntAna_Ellipse:
            {
                IntPatch_Point aPtsol = new();
                gp_Elips elipsol = inter.Ellipse(1);

                ElCLib.D1(0.0, elipsol, out gp_Pnt ptref, out gp_Vec Tgt);

                double qwe = Tgt.DotCross(Quad2.Normale(ptref), Quad1.Normale(ptref));
                if (qwe > 0.00000001)
                {
                    trans1 = IntSurf_Out;
                    trans2 = IntSurf_In;
                }
                else if (qwe < -0.00000001)
                {
                    trans1 = IntSurf_In;
                    trans2 = IntSurf_Out;
                }
                else
                {
                    trans1 = trans2 = IntSurf_Undecided;
                }
                IntPatch_GLine glig = new(elipsol, false, trans1, trans2);
                if (inter.HasCommonGen())
                {
                    gp_Pnt aPChar = inter.PChar();
                    Quad1.Parameters(aPChar, ref U1, ref V1);
                    Quad2.Parameters(aPChar, ref U2, ref V2);
                    aPtsol.SetValue(aPChar, Tol, false);
                    aPtsol.SetParameters(U1, V1, U2, V2);
                    aPtsol.SetParameter(0.0);
                    glig.AddVertex(aPtsol);
                }
                slin.Append(glig);
            }
            break;

            case IntAna_Hyperbola:
            {
                IntPatch_Point aPtsol = new();
                gp_Vec Tgt;
                gp_Pnt ptref;
                for (i = 1; i <= 2; i++)
                {
                    gp_Hypr hyprsol = inter.Hyperbola(i);
                    ElCLib.D1(0.0, hyprsol, out ptref, out Tgt);
                    double qwe = Tgt.DotCross(Quad2.Normale(ptref), Quad1.Normale(ptref));
                    if (qwe > 0.00000001)
                    {
                        trans1 = IntSurf_Out;
                        trans2 = IntSurf_In;
                    }
                    else if (qwe < -0.00000001)
                    {
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                    }
                    else
                    {
                        trans1 = trans2 = IntSurf_Undecided;
                    }
                    IntPatch_GLine glig = new(hyprsol, false, trans1, trans2);
                    if (inter.HasCommonGen())
                    {
                        gp_Pnt aPChar = inter.PChar();
                        Quad1.Parameters(aPChar, ref U1, ref V1);
                        Quad2.Parameters(aPChar, ref U2, ref V2);
                        aPtsol.SetValue(aPChar, Tol, false);
                        aPtsol.SetParameters(U1, V1, U2, V2);
                        aPtsol.SetParameter(0.0);
                        glig.AddVertex(aPtsol);
                    }
                    slin.Append(glig);
                }
            }
            break;

            case IntAna_Parabola:
            {
                IntPatch_Point aPtsol = new();
                gp_Parab parabsol = inter.Parabola(1);

                gp_Vec Tgtorig = new(parabsol.YAxis().Direction());
                double ptran =
                    Tgtorig.DotCross(Quad2.Normale(parabsol.Location()), Quad1.Normale(parabsol.Location()));
                if (ptran > 0.00000001)
                {
                    trans1 = IntSurf_Out;
                    trans2 = IntSurf_In;
                }
                else if (ptran < -0.00000001)
                {
                    trans1 = IntSurf_In;
                    trans2 = IntSurf_Out;
                }
                else
                {
                    trans1 = trans2 = IntSurf_Undecided;
                }

                IntPatch_GLine glig = new(parabsol, false, trans1, trans2);
                if (inter.HasCommonGen())
                {
                    gp_Pnt aPChar = inter.PChar();
                    Quad1.Parameters(aPChar, ref U1, ref V1);
                    Quad2.Parameters(aPChar, ref U2, ref V2);
                    aPtsol.SetValue(aPChar, Tol, false);
                    aPtsol.SetParameters(U1, V1, U2, V2);
                    aPtsol.SetParameter(0.0);
                    glig.AddVertex(aPtsol);
                }
                slin.Append(glig);
            }
            break;

            case IntAna_NoGeometricSolution:
            {
                gp_Pnt psol;
                IntAna_IntQuadQuad anaint = new(Co1, new IntAna_Quadric(Co2), Tol);
                if (!anaint.IsDone())
                {
                    return false;
                }

                if (anaint.NbPnt() == 0 && anaint.NbCurve() == 0)
                {
                    Empty = true;
                }
                else
                {
                    NbSol = anaint.NbPnt();
                    for (i = 1; i <= NbSol; i++)
                    {
                        psol = anaint.Point(i);
                        Quad1.Parameters(psol, ref U1, ref V1);
                        Quad2.Parameters(psol, ref U2, ref V2);
                        IntPatch_Point ptsol = new();
                        ptsol.SetValue(psol, Tol, true);
                        ptsol.SetParameters(U1, V1, U2, V2);
                        spnt.Append(ptsol);
                    }

                    // ptf and ptl keep their values from the previous curve where the bound is open, as OCCT's.
                    // OCCT: kept across the curves; an open bound passes the previous curve's point
                    gp_Pnt ptvalid = new(), ptf = new(), ptl = new();
                    gp_Vec tgvalid = new();

                    double first, last, para;
                    bool tgfound, firstp, lastp, kept;
                    int kount;

                    NbSol = anaint.NbCurve();
                    for (i = 1; i <= NbSol; i++)
                    {
                        IntPatch_ALine alig;
                        kept = false;
                        IntAna_Curve curvsol = anaint.Curve(i);
                        curvsol.Domain(out first, out last);
                        firstp = !curvsol.IsFirstOpen();
                        lastp = !curvsol.IsLastOpen();
                        if (firstp)
                        {
                            ptf = curvsol.Value(first);
                        }
                        if (lastp)
                        {
                            ptl = curvsol.Value(last);
                        }
                        para = last;
                        kount = 1;
                        tgfound = false;

                        while (!tgfound)
                        {
                            para = (1.123 * first + para) / 2.123;
                            tgfound = curvsol.D1u(para, out ptvalid, out tgvalid);
                            if (tgvalid.SquareMagnitude() < 1e-14)
                            {
                                //-- on se trouve ds un cas ou les normales n'auront pas de sens
                                tgfound = false;
                            }

                            if (!tgfound)
                            {
                                kount++;
                                tgfound = kount > 5;
                            }
                        }
                        if (kount <= 5)
                        {
                            double qwe = tgvalid.DotCross(Quad2.Normale(ptvalid), Quad1.Normale(ptvalid));
                            if (qwe > 0.000000001)
                            {
                                trans1 = IntSurf_Out;
                                trans2 = IntSurf_In;
                            }
                            else if (qwe < -0.000000001)
                            {
                                trans1 = IntSurf_In;
                                trans2 = IntSurf_Out;
                            }
                            else
                            {
                                trans1 = trans2 = IntSurf_Undecided;
                            }
                            alig = new IntPatch_ALine(curvsol, false, trans1, trans2);
                            kept = true;
                        }
                        else
                        {
                            ptvalid = curvsol.Value(para);
                            alig = new IntPatch_ALine(curvsol, false);
                            kept = true;
                            //-- cout << "Transition indeterminee" << endl;
                        }
                        if (kept)
                        {
                            bool Nfirstp = !firstp;
                            bool Nlastp = !lastp;
                            ProcessBounds(alig,
                                          slin,
                                          Quad1,
                                          Quad2,
                                          ref Nfirstp,
                                          ptf,
                                          first,
                                          ref Nlastp,
                                          ptl,
                                          last,
                                          ref Multpoint,
                                          Tol);
                            slin.Append(alig);
                        }
                    }
                }
            }
            break;

            default:
            {
                return false;
            }
        }

        // When two cones have common generatrix passing through apexes
        // it is necessary to add it is solution
        if (inter.HasCommonGen())
        {
            double para;
            IntPatch_Point aPtsol = new();
            gp_Pnt aPApex1, aPApex2;
            aPApex1 = Co1.Apex();
            aPApex2 = Co2.Apex();
            // common generatrix of cones
            gce_MakeLin aMkLin = new(aPApex1, aPApex2);
            gp_Lin linsol = aMkLin.Value();
            IntPatch_GLine glig = new(linsol, true, IntSurf_Undecided, IntSurf_Undecided);

            gp_Pnt aPChar = inter.PChar();
            Quad1.Parameters(aPChar, ref U1, ref V1);
            Quad2.Parameters(aPChar, ref U2, ref V2);
            aPtsol.SetValue(aPChar, Tol, false);
            aPtsol.SetParameters(U1, V1, U2, V2);
            para = ElCLib.Parameter(linsol, aPChar);
            aPtsol.SetParameter(para);
            glig.AddVertex(aPtsol);

            slin.Append(glig);
        }

        return true;
    }

    //=================================================================================================

    private static bool IntCoSp(in IntSurf_Quadric Quad1,
                                in IntSurf_Quadric Quad2,
                                double Tol,
                                bool Reversed,
                                ref bool Empty,
                                ref bool Multpoint,
                                NCollection_Sequence<IntPatch_Line> slin,
                                NCollection_Sequence<IntPatch_Point> spnt)
    {
        int i;

        IntSurf_TypeTrans trans1, trans2;
        IntAna_ResultType typint;

        gp_Sphere Sp;
        gp_Cone Co;
        double U1 = 0, V1 = 0, U2 = 0, V2 = 0;

        if (!Reversed)
        {
            Co = Quad1.Cone();
            Sp = Quad2.Sphere();
        }
        else
        {
            Co = Quad2.Cone();
            Sp = Quad1.Sphere();
        }
        IntAna_QuadQuadGeo inter = new(Sp, Co, Tol);

        if (!inter.IsDone())
        {
            return false;
        }

        typint = inter.TypeInter();
        int NbSol = inter.NbSolutions();
        Empty = false;

        switch (typint)
        {
            case IntAna_Empty:
            {
                Empty = true;
            }
            break;

            case IntAna_Point:
            {
                gp_Pnt ptcontact;
                gp_Pnt apex = Co.Apex();
                double param;
                double paramapex = ElCLib.LineParameter(Co.Axis(), apex);
                for (i = 1; i <= NbSol; i++)
                {
                    ptcontact = inter.Point(i);
                    param = ElCLib.LineParameter(Co.Axis(), ptcontact);
                    Quad1.Parameters(ptcontact, ref U1, ref V1);
                    Quad2.Parameters(ptcontact, ref U2, ref V2);

                    if (apex.Distance(ptcontact) <= Tol)
                    {
                        IntPatch_Point ptsol = new();
                        ptsol.SetValue(ptcontact, Tol, false);
                        ptsol.SetParameters(U1, V1, U2, V2);
                        spnt.Append(ptsol);
                    }
                    else if (param >= paramapex)
                    {
                        IntPatch_Point ptsol = new();
                        ptsol.SetValue(ptcontact, Tol, true);
                        ptsol.SetParameters(U1, V1, U2, V2);
                        spnt.Append(ptsol);
                    }
                }
            }
            break;

            case IntAna_Circle:
            {
                gp_Vec Tgt;
                gp_Pnt ptref;

                for (i = 1; i <= NbSol; i++)
                {
                    gp_Circ cirsol = inter.Circle(i);
                    //-- param = ElCLib::LineParameter(Co.Axis(),
                    //-- 			      cirsol.Location());
                    //-- if (param >= paramapex) {

                    ElCLib.D1(0.0, cirsol, out ptref, out Tgt);
                    double qwe = Tgt.DotCross(Quad2.Normale(ptref), Quad1.Normale(ptref));
                    if (qwe > 0.00000001)
                    {
                        trans1 = IntSurf_Out;
                        trans2 = IntSurf_In;
                    }
                    else if (qwe < -0.00000001)
                    {
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                    }
                    else
                    {
                        trans1 = trans2 = IntSurf_Undecided;
                    }
                    IntPatch_GLine glig = new(cirsol, false, trans1, trans2);
                    slin.Append(glig);
                    //-- }
                }
            }
            break;

            case IntAna_PointAndCircle:
            {
                gp_Pnt apex = Co.Apex();
                double param;
                double paramapex = ElCLib.LineParameter(Co.Axis(), apex);

                // le point est necessairement l apex
                Quad1.Parameters(apex, ref U1, ref V1);
                Quad2.Parameters(apex, ref U2, ref V2);
                IntPatch_Point ptsol = new();
                ptsol.SetValue(apex, Tol, false);
                ptsol.SetParameters(U1, V1, U2, V2);
                spnt.Append(ptsol);

                gp_Circ cirsol = inter.Circle(1);
                param = ElCLib.LineParameter(Co.Axis(), cirsol.Location());
                ElCLib.D1(0.0, cirsol, out gp_Pnt ptref, out gp_Vec Tgt);
                double qwe = Tgt.DotCross(Quad2.Normale(ptref), Quad1.Normale(ptref));

                if (param >= paramapex)
                {
                    if (qwe > Precision.PConfusion())
                    {
                        trans1 = IntSurf_Out;
                        trans2 = IntSurf_In;
                    }
                    else if (qwe < -Precision.PConfusion())
                    {
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                    }
                    else
                    {
                        trans1 = trans2 = IntSurf_Undecided;
                    }
                }
                else
                {
                    if (qwe < -Precision.PConfusion())
                    {
                        trans1 = IntSurf_Out;
                        trans2 = IntSurf_In;
                    }
                    else if (qwe > Precision.PConfusion())
                    {
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                    }
                    else
                    {
                        trans1 = trans2 = IntSurf_Undecided;
                    }
                }
                IntPatch_GLine glig = new(cirsol, false, trans1, trans2);
                slin.Append(glig);
            }
            break;

            case IntAna_NoGeometricSolution:
            {
                gp_Pnt psol;
                IntAna_IntQuadQuad anaint = new(Co, new IntAna_Quadric(Sp), Tol);
                if (!anaint.IsDone())
                {
                    return false;
                }

                if (anaint.NbPnt() == 0 && anaint.NbCurve() == 0)
                {
                    Empty = true;
                }
                else
                {
                    NbSol = anaint.NbPnt();
                    for (i = 1; i <= NbSol; i++)
                    {
                        psol = anaint.Point(i);
                        Quad1.Parameters(psol, ref U1, ref V1);
                        Quad2.Parameters(psol, ref U2, ref V2);
                        IntPatch_Point ptsol = new();
                        ptsol.SetValue(psol, Tol, true);
                        ptsol.SetParameters(U1, V1, U2, V2);
                        spnt.Append(ptsol);
                    }

                    // ptf and ptl keep their values from the previous curve where the bound is open, as OCCT's.
                    // OCCT: kept across the curves; an open bound passes the previous curve's point
                    gp_Pnt ptvalid = new(), ptf = new(), ptl = new();
                    gp_Vec tgvalid = new();
                    double first, last, para;
                    bool tgfound, firstp, lastp, kept;
                    int kount;

                    NbSol = anaint.NbCurve();
                    for (i = 1; i <= NbSol; i++)
                    {
                        IntPatch_ALine alig;
                        kept = false;
                        IntAna_Curve curvsol = anaint.Curve(i);
                        curvsol.Domain(out first, out last);
                        firstp = !curvsol.IsFirstOpen();
                        lastp = !curvsol.IsLastOpen();
                        if (firstp)
                        {
                            ptf = curvsol.Value(first);
                        }
                        if (lastp)
                        {
                            ptl = curvsol.Value(last);
                        }
                        para = last;
                        kount = 1;
                        tgfound = false;

                        while (!tgfound)
                        {
                            para = (1.123 * first + para) / 2.123;
                            tgfound = curvsol.D1u(para, out ptvalid, out tgvalid);
                            if (!tgfound)
                            {
                                kount++;
                                tgfound = kount > 5;
                            }
                        }
                        if (kount <= 5)
                        {
                            // para is computed and never read, as in OCCT.
                            para = ElCLib.LineParameter(Co.Axis(), ptvalid);
                            double qwe = tgvalid.DotCross(Quad2.Normale(ptvalid), Quad1.Normale(ptvalid));
                            if (qwe > 0.000000001)
                            {
                                trans1 = IntSurf_Out;
                                trans2 = IntSurf_In;
                            }
                            else if (qwe < -0.000000001)
                            {
                                trans1 = IntSurf_In;
                                trans2 = IntSurf_Out;
                            }
                            else
                            {
                                trans1 = trans2 = IntSurf_Undecided;
                            }
                            alig = new IntPatch_ALine(curvsol, false, trans1, trans2);
                            kept = true;
                        }
                        else
                        {
                            ptvalid = curvsol.Value(para);
                            para = ElCLib.LineParameter(Co.Axis(), ptvalid);
                            alig = new IntPatch_ALine(curvsol, false);
                            kept = true;
                            //-- cout << "Transition indeterminee" << endl;
                        }
                        if (kept)
                        {
                            bool Nfirstp = !firstp;
                            bool Nlastp = !lastp;
                            ProcessBounds(alig,
                                          slin,
                                          Quad1,
                                          Quad2,
                                          ref Nfirstp,
                                          ptf,
                                          first,
                                          ref Nlastp,
                                          ptl,
                                          last,
                                          ref Multpoint,
                                          Tol);
                            slin.Append(alig);
                        }
                    }
                }
            }
            break;

            default:
            {
                return false;
            }
        }

        return true;
    }

    //=================================================================================================

    // Traitement du cas Sphere/Sphere
    private static bool IntSpSp(in IntSurf_Quadric Quad1,
                                in IntSurf_Quadric Quad2,
                                double Tol,
                                ref bool Empty,
                                ref bool Same,
                                NCollection_Sequence<IntPatch_Line> slin,
                                NCollection_Sequence<IntPatch_Point> spnt)
    {
        IntSurf_TypeTrans trans1, trans2;
        IntAna_ResultType typint;
        gp_Sphere sph1 = Quad1.Sphere();
        gp_Sphere sph2 = Quad2.Sphere();

        IntAna_QuadQuadGeo inter = new(sph1, sph2, Tol);
        if (!inter.IsDone())
        {
            return false;
        }

        typint = inter.TypeInter();
        Empty = false;
        Same = false;

        switch (typint)
        {
            case IntAna_Empty:
            {
                Empty = true;
            }
            break;

            case IntAna_Same:
            {
                Same = true;
            }
            break;

            case IntAna_Point:
            {
                gp_Pnt psol = inter.Point(1);
                double U1 = 0, V1 = 0, U2 = 0, V2 = 0;
                Quad1.Parameters(psol, ref U1, ref V1);
                Quad2.Parameters(psol, ref U2, ref V2);
                IntPatch_Point ptsol = new();
                ptsol.SetValue(psol, Tol, true);
                ptsol.SetParameters(U1, V1, U2, V2);
                spnt.Append(ptsol);
            }
            break;

            case IntAna_Circle:
            {
                gp_Circ cirsol = inter.Circle(1);
                ElCLib.D1(0.0, cirsol, out gp_Pnt ptref, out gp_Vec Tgt);

                double qwe = Tgt.DotCross(Quad2.Normale(ptref), Quad1.Normale(ptref));
                if (qwe > 0.00000001)
                {
                    trans1 = IntSurf_Out;
                    trans2 = IntSurf_In;
                }
                else if (qwe < -0.00000001)
                {
                    trans1 = IntSurf_In;
                    trans2 = IntSurf_Out;
                }
                else
                {
                    trans1 = trans2 = IntSurf_Undecided;
                }
                IntPatch_GLine glig = new(cirsol, false, trans1, trans2);
                slin.Append(glig);
            }
            break;

            default:
            {
                return false; // on ne doit pas passer ici
            }
        }
        return true;
    }

    //=================================================================================================

    private static bool IntCyTo(in IntSurf_Quadric theQuad1,
                                in IntSurf_Quadric theQuad2,
                                double theTolTang,
                                bool bReversed,
                                ref bool bEmpty,
                                NCollection_Sequence<IntPatch_Line> theSeqLin)
    {
        gp_Cylinder aCyl = bReversed ? theQuad2.Cylinder() : theQuad1.Cylinder();
        gp_Torus aTorus = bReversed ? theQuad1.Torus() : theQuad2.Torus();
        //
        IntAna_QuadQuadGeo anInt = new(aCyl, aTorus, theTolTang);
        bool bRet = TreatResultTorus(theQuad1, theQuad2, anInt, ref bEmpty, theSeqLin);
        //
        return bRet;
    }

    //=================================================================================================

    private static bool IntCoTo(in IntSurf_Quadric theQuad1,
                                in IntSurf_Quadric theQuad2,
                                double theTolTang,
                                bool bReversed,
                                ref bool bEmpty,
                                NCollection_Sequence<IntPatch_Line> theSeqLin)
    {
        gp_Cone aCone = bReversed ? theQuad2.Cone() : theQuad1.Cone();
        gp_Torus aTorus = bReversed ? theQuad1.Torus() : theQuad2.Torus();
        //
        IntAna_QuadQuadGeo anInt = new(aCone, aTorus, theTolTang);
        bool bRet = TreatResultTorus(theQuad1, theQuad2, anInt, ref bEmpty, theSeqLin);
        //
        return bRet;
    }

    //=================================================================================================

    private static bool IntSpTo(in IntSurf_Quadric theQuad1,
                                in IntSurf_Quadric theQuad2,
                                double theTolTang,
                                bool bReversed,
                                ref bool bEmpty,
                                NCollection_Sequence<IntPatch_Line> theSeqLin)
    {
        gp_Sphere aSphere = bReversed ? theQuad2.Sphere() : theQuad1.Sphere();
        gp_Torus aTorus = bReversed ? theQuad1.Torus() : theQuad2.Torus();
        //
        IntAna_QuadQuadGeo anInt = new(aSphere, aTorus, theTolTang);
        bool bRet = TreatResultTorus(theQuad1, theQuad2, anInt, ref bEmpty, theSeqLin);
        //
        return bRet;
    }

    //=================================================================================================

    private static bool IntToTo(in IntSurf_Quadric theQuad1,
                                in IntSurf_Quadric theQuad2,
                                double theTolTang,
                                ref bool bSameSurf,
                                ref bool bEmpty,
                                NCollection_Sequence<IntPatch_Line> theSeqLin)
    {
        gp_Torus aTorus1 = theQuad1.Torus();
        gp_Torus aTorus2 = theQuad2.Torus();
        //
        IntAna_QuadQuadGeo anInt = new(aTorus1, aTorus2, theTolTang);
        bool bRet = anInt.IsDone();
        if (bRet)
        {
            if (anInt.TypeInter() == IntAna_Same)
            {
                bEmpty = false;
                bSameSurf = true;
            }
            else
            {
                bRet = TreatResultTorus(theQuad1, theQuad2, anInt, ref bEmpty, theSeqLin);
            }
        }
        //
        return bRet;
    }

    //=================================================================================================

    private static bool TreatResultTorus(in IntSurf_Quadric theQuad1,
                                         in IntSurf_Quadric theQuad2,
                                         IntAna_QuadQuadGeo anInt,
                                         ref bool bEmpty,
                                         NCollection_Sequence<IntPatch_Line> theSeqLin)
    {
        bool bRet = anInt.IsDone();
        //
        if (!bRet)
        {
            return bRet;
        }
        //
        IntAna_ResultType typint = anInt.TypeInter();
        int NbSol = anInt.NbSolutions();
        bEmpty = false;
        //
        switch (typint)
        {
            case IntAna_Empty:
                bEmpty = true;
                break;
            //
            case IntAna_Circle:
            {
                int i;
                IntSurf_TypeTrans trans1, trans2;
                gp_Vec Tgt;
                gp_Pnt ptref;
                //
                for (i = 1; i <= NbSol; ++i)
                {
                    gp_Circ aC = anInt.Circle(i);
                    if (theQuad1.TypeQuadric() == theQuad2.TypeQuadric())
                    {
                        AdjustToSeam(theQuad1.Torus(), ref aC);
                    }
                    ElCLib.D1(0.0, aC, out ptref, out Tgt);
                    double qwe = Tgt.DotCross(theQuad2.Normale(ptref), theQuad1.Normale(ptref));
                    if (qwe > 0.00000001)
                    {
                        trans1 = IntSurf_Out;
                        trans2 = IntSurf_In;
                    }
                    else if (qwe < -0.00000001)
                    {
                        trans1 = IntSurf_In;
                        trans2 = IntSurf_Out;
                    }
                    else
                    {
                        trans1 = trans2 = IntSurf_Undecided;
                    }
                    //
                    IntPatch_GLine glig = new(aC, false, trans1, trans2);
                    theSeqLin.Append(glig);
                }
            }
            break;
            //
            case IntAna_NoGeometricSolution:
            default:
                bRet = false;
                break;
        }
        //
        return bRet;
    }
}
