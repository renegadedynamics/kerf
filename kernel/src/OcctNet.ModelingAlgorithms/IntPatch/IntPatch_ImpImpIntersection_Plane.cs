// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntAna_ResultType;
using static Occt.IntSurf_Situation;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

// The plane/quadric pairs (OCCT 7.x's IntPatch_ImpImpIntersection_3.gxx): plane/plane, plane/cylinder,
// plane/sphere, plane/cone and plane/torus, and the circles put on the seam of the quadric (AdjustToSeam).
//
// OCCT's gp_Dir::DotCross takes directions, so the normals OCCT passes to it as vectors are normalised by C++'s
// implicit conversion; the port writes the conversion (new gp_Dir(...)).
public sealed partial class IntPatch_ImpImpIntersection
{
    // modified by NIZNHY-PKV Thu Sep 15 11:09:13 2011

    //=======================================================================
    // function : IntPP
    // purpose  :
    // Traitement du cas Plan/Plan
    //=======================================================================
    private static bool IntPP(in IntSurf_Quadric Quad1,
                              in IntSurf_Quadric Quad2,
                              double Tolang,
                              double TolTang,
                              ref bool Same,
                              NCollection_Sequence<IntPatch_Line> slin)
    {
        IntSurf_TypeTrans trans1, trans2;
        IntAna_ResultType typint;
        gp_Pln pl1 = Quad1.Plane();
        gp_Pln pl2 = Quad2.Plane();

        IntAna_QuadQuadGeo inter = new(pl1, pl2, Tolang, TolTang);
        if (!inter.IsDone())
        {
            return false;
        }
        Same = false;
        typint = inter.TypeInter();
        if (typint == IntAna_Same)
        { // cas faces confondues
            Same = true;
        }
        else if (typint != IntAna_Empty)
        { // on a une ligne
            gp_Lin linsol = inter.Line(1);
            double discri = linsol.Direction().DotCross(new gp_Dir(Quad2.Normale(linsol.Location())),
                                                        new gp_Dir(Quad1.Normale(linsol.Location())));

            if (discri > 0.0)
            {
                trans1 = IntSurf_Out;
                trans2 = IntSurf_In;
            }
            else
            {
                trans1 = IntSurf_In;
                trans2 = IntSurf_Out;
            }
            IntPatch_GLine glig = new(linsol, false, trans1, trans2);
            slin.Append(glig);
        }
        return true;
    }

    //=======================================================================
    // function : IntPCy
    // purpose  :
    // Traitement du cas Plan/Cylindre et reciproquement
    //=======================================================================
    private static bool IntPCy(in IntSurf_Quadric Quad1,
                               in IntSurf_Quadric Quad2,
                               double Tolang,
                               double TolTang,
                               bool Reversed,
                               ref bool Empty,
                               NCollection_Sequence<IntPatch_Line> slin,
                               double H = 0.0)
    {
        gp_Pln Pl;
        gp_Cylinder Cy;

        IntSurf_TypeTrans trans1, trans2;
        IntAna_ResultType typint;

        IntAna_QuadQuadGeo inter = new();
        if (!Reversed)
        {
            Pl = Quad1.Plane();
            Cy = Quad2.Cylinder();
        }
        else
        {
            Pl = Quad2.Plane();
            Cy = Quad1.Cylinder();
        }
        inter.Perform(Pl, Cy, Tolang, TolTang, H);
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

            case IntAna_Line:
                {
                    gp_Lin linsol = inter.Line(1);
                    gp_Pnt orig = linsol.Location();
                    if (NbSol == 1)
                    { // ligne de tangence
                        gp_Vec TestCurvature = new(orig, Cy.Location());
                        gp_Vec Normp, Normcyl;
                        if (!Reversed)
                        {
                            Normp = Quad1.Normale(orig);
                            Normcyl = Quad2.Normale(orig);
                        }
                        else
                        {
                            Normp = Quad2.Normale(orig);
                            Normcyl = Quad1.Normale(orig);
                        }

                        IntSurf_Situation situcyl;
                        IntSurf_Situation situp;

                        if (Normp.Dot(TestCurvature) > 0.0)
                        {
                            situcyl = IntSurf_Outside;
                            if (Normp.Dot(Normcyl) > 0.0)
                            {
                                situp = IntSurf_Inside;
                            }
                            else
                            {
                                situp = IntSurf_Outside;
                            }
                        }
                        else
                        {
                            situcyl = IntSurf_Inside;
                            if (Normp.Dot(Normcyl) > 0.0)
                            {
                                situp = IntSurf_Outside;
                            }
                            else
                            {
                                situp = IntSurf_Inside;
                            }
                        }
                        IntPatch_GLine glig;
                        if (!Reversed)
                        {
                            glig = new IntPatch_GLine(linsol, true, situp, situcyl);
                        }
                        else
                        {
                            glig = new IntPatch_GLine(linsol, true, situcyl, situp);
                        }
                        slin.Append(glig);
                    }
                    else
                    {
                        // on a 2 droites. Il faut determiner les transitions
                        // de chacune.

                        if (linsol.Direction().DotCross(new gp_Dir(Quad2.Normale(orig)), new gp_Dir(Quad1.Normale(orig)))
                            > 0.0)
                        {
                            trans1 = IntSurf_Out;
                            trans2 = IntSurf_In;
                        }
                        else
                        {
                            trans1 = IntSurf_In;
                            trans2 = IntSurf_Out;
                        }
                        IntPatch_GLine glig = new(linsol, false, trans1, trans2);
                        slin.Append(glig);

                        linsol = inter.Line(2);
                        orig = linsol.Location();

                        if (linsol.Direction().DotCross(new gp_Dir(Quad2.Normale(orig)), new gp_Dir(Quad1.Normale(orig)))
                            > 0.0)
                        {
                            trans1 = IntSurf_Out;
                            trans2 = IntSurf_In;
                        }
                        else
                        {
                            trans1 = IntSurf_In;
                            trans2 = IntSurf_Out;
                        }
                        glig = new IntPatch_GLine(linsol, false, trans1, trans2);
                        slin.Append(glig);
                    }
                }
                break;
            //
            case IntAna_Circle:
                {
                    gp_Circ cirsol;
                    //
                    cirsol = inter.Circle(1);
                    // modified by NIZNHY-PKV Thu Sep 15 11:30:03 2011f
                    AdjustToSeam(Cy, ref cirsol);
                    // modified by NIZNHY-PKV Thu Sep 15 11:30:15 2011t
                    ElCLib.D1(0.0, cirsol, out gp_Pnt ptref, out gp_Vec Tgt);

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
                }
                break;
            //
            case IntAna_Ellipse:
                {
                    gp_Elips elipsol = inter.Ellipse(1);
                    ElCLib.D1(0.0, elipsol, out gp_Pnt ptref, out gp_Vec Tgt);

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
                    IntPatch_GLine glig = new(elipsol, false, trans1, trans2);
                    slin.Append(glig);
                }
                break;
            //
            default:
                {
                    return false; // on ne doit pas passer ici
                }
        }
        return true;
    }

    //=======================================================================
    // function : IntPSp
    // purpose  :
    // Traitement du cas Plan/Sphere et reciproquement
    //=======================================================================
    private static bool IntPSp(in IntSurf_Quadric Quad1,
                               in IntSurf_Quadric Quad2,
                               double Tolang,
                               double TolTang,
                               bool Reversed,
                               ref bool Empty,
                               NCollection_Sequence<IntPatch_Line> slin,
                               NCollection_Sequence<IntPatch_Point> spnt)
    {
        gp_Circ cirsol;
        gp_Pln Pl;
        gp_Sphere Sp;
        IntSurf_TypeTrans trans1, trans2;
        IntAna_ResultType typint;

        IntAna_QuadQuadGeo inter = new();
        if (!Reversed)
        {
            Pl = Quad1.Plane();
            Sp = Quad2.Sphere();
        }
        else
        {
            Pl = Quad2.Plane();
            Sp = Quad1.Sphere();
        }
        inter.Perform(Pl, Sp);

        if (!inter.IsDone())
        {
            return false;
        }

        typint = inter.TypeInter();
        Empty = false;

        switch (typint)
        {
            case IntAna_Empty:
                {
                    Empty = true;
                }
                break;
            //
            case IntAna_Point:
                {
                    gp_Pnt psol = inter.Point(1);
                    double U1 = 0.0, V1 = 0.0, U2 = 0.0, V2 = 0.0;
                    Quad1.Parameters(psol, ref U1, ref V1);
                    Quad2.Parameters(psol, ref U2, ref V2);
                    IntPatch_Point ptsol = new();
                    ptsol.SetValue(psol, TolTang, true);
                    ptsol.SetParameters(U1, V1, U2, V2);
                    spnt.Append(ptsol);
                }
                break;
            //
            case IntAna_Circle:
                {
                    cirsol = inter.Circle(1);
                    // modified by NIZNHY-PKV Thu Sep 15 11:30:03 2011f
                    AdjustToSeam(Sp, ref cirsol, Tolang);
                    // modified by NIZNHY-PKV Thu Sep 15 11:30:15 2011t
                    ElCLib.D1(0.0, cirsol, out gp_Pnt ptref, out gp_Vec Tgt);

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
                }
                break;

            default:
                {
                    return false; // on ne doit pas passer ici
                }
        }
        return true;
    }

    //=======================================================================
    // function : IntPCo
    // purpose  :
    // Traitement du cas Plan/Cone et reciproquement
    //=======================================================================
    private static bool IntPCo(in IntSurf_Quadric Quad1,
                               in IntSurf_Quadric Quad2,
                               double Tolang,
                               double TolTang,
                               bool Reversed,
                               ref bool Empty,
                               ref bool Multpoint,
                               NCollection_Sequence<IntPatch_Line> slin,
                               NCollection_Sequence<IntPatch_Point> spnt)
    {
        gp_Pnt apex;

        gp_Pln Pl;
        gp_Cone Co;

        IntSurf_TypeTrans trans1, trans2;
        IntAna_ResultType typint;

        IntAna_QuadQuadGeo inter = new();
        if (!Reversed)
        {
            Pl = Quad1.Plane();
            Co = Quad2.Cone();
            apex = Co.Apex();
        }
        else
        {
            Pl = Quad2.Plane();
            Co = Quad1.Cone();
            apex = Co.Apex();
        }

        inter.Perform(Pl, Co, Tolang, TolTang);
        if (!inter.IsDone())
        {
            return false;
        }
        //
        typint = inter.TypeInter();
        int NbSol = inter.NbSolutions();
        Empty = false;

        switch (typint)
        {
            case IntAna_Point:
                {
                    gp_Pnt psol = inter.Point(1);
                    double U1 = 0.0, V1 = 0.0, U2 = 0.0, V2 = 0.0;
                    Quad1.Parameters(psol, ref U1, ref V1);
                    Quad2.Parameters(psol, ref U2, ref V2);
                    IntPatch_Point ptsol = new();
                    ptsol.SetValue(psol, TolTang, false);
                    ptsol.SetParameters(U1, V1, U2, V2);
                    spnt.Append(ptsol);
                }
                break;

            case IntAna_Line:
                {
                    gp_Lin linsol = inter.Line(1);
                    if (linsol.Direction().Dot(Co.Axis().Direction()) < 0.0)
                    {
                        linsol.SetDirection(linsol.Direction().Reversed());
                    }
                    double para = ElCLib.Parameter(linsol, apex);
                    gp_Pnt ptbid = ElCLib.Value(para + 5.0, linsol);
                    double U1 = 0.0, V1 = 0.0, U2 = 0.0, V2 = 0.0;
                    Quad1.Parameters(apex, ref U1, ref V1);
                    Quad2.Parameters(apex, ref U2, ref V2);

                    if (NbSol == 1)
                    { // ligne de tangence
                        IntPatch_Point ptsol = new();
                        ptsol.SetValue(apex, TolTang, false);
                        ptsol.SetParameters(U1, V1, U2, V2);
                        ptsol.SetParameter(para);
                        gp_Pnt ptbid2 = new(apex.XYZ() + 5.0 * Co.Axis().Direction().XYZ());
                        gp_Vec TestCurvature = new(ptbid, ptbid2);
                        gp_Vec Normp, Normco;
                        if (!Reversed)
                        {
                            Normp = Quad1.Normale(ptbid);
                            Normco = Quad2.Normale(ptbid);
                        }
                        else
                        {
                            Normp = Quad2.Normale(ptbid);
                            Normco = Quad1.Normale(ptbid);
                        }
                        IntSurf_Situation situco, situco_otherside;
                        IntSurf_Situation situp, situp_otherside;

                        if (Normp.Dot(TestCurvature) > 0.0)
                        {
                            situco = IntSurf_Outside;
                            situco_otherside = IntSurf_Inside;
                            if (Normp.Dot(Normco) > 0.0)
                            {
                                situp = IntSurf_Inside;
                                situp_otherside = IntSurf_Outside;
                            }
                            else
                            {
                                situp = IntSurf_Outside;
                                situp_otherside = IntSurf_Inside;
                            }
                        }
                        else
                        {
                            situco = IntSurf_Inside;
                            situco_otherside = IntSurf_Outside;
                            if (Normp.Dot(Normco) > 0.0)
                            {
                                situp = IntSurf_Outside;
                                situp_otherside = IntSurf_Inside;
                            }
                            else
                            {
                                situp = IntSurf_Inside;
                                situp_otherside = IntSurf_Outside;
                            }
                        }
                        //----------------------------------------------------------
                        //--              Apex ---> Cone.Direction
                        //--
                        IntPatch_GLine glig;
                        if (!Reversed)
                        {
                            glig = new IntPatch_GLine(linsol, true, situp, situco);
                        }
                        else
                        {
                            glig = new IntPatch_GLine(linsol, true, situco, situp);
                        }
                        glig.AddVertex(ptsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);
                        //----------------------------------------------------------
                        //--   -Cone.Direction <------- Apex
                        //--
                        linsol.SetDirection(linsol.Direction().Reversed());
                        if (!Reversed)
                        {
                            glig = new IntPatch_GLine(linsol, true, situp_otherside, situco_otherside);
                        }
                        else
                        {
                            glig = new IntPatch_GLine(linsol, true, situco_otherside, situp_otherside);
                        }
                        glig.AddVertex(ptsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);
                    }
                    else
                    {
                        // on a 2 droites. Il faut determiner les transitions
                        // de chacune. On oriente chaque ligne dans le sens
                        // de l axe du cone. Les transitions de chaque ligne seront
                        // inverses l une de l autre => on ne fait le calcul que sur
                        // la premiere.
                        if (linsol.Direction().DotCross(new gp_Dir(Quad2.Normale(ptbid)), new gp_Dir(Quad1.Normale(ptbid)))
                            > 0.0)
                        {
                            trans1 = IntSurf_Out;
                            trans2 = IntSurf_In;
                        }
                        else
                        {
                            trans1 = IntSurf_In;
                            trans2 = IntSurf_Out;
                        }

                        Multpoint = true;
                        //------------------------------------------- Ligne 1 -------
                        IntPatch_Point ptsol = new();
                        ptsol.SetValue(apex, TolTang, false);
                        ptsol.SetParameters(U1, V1, U2, V2);
                        ptsol.SetParameter(para);
                        ptsol.SetMultiple(true);
                        IntPatch_GLine glig;
                        glig = new IntPatch_GLine(linsol, false, trans1, trans2);
                        glig.AddVertex(ptsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);
                        //-----------------------------------------------------------
                        //-- Other Side : Les transitions restent les memes
                        //--    linsol -> -linsol   et Quad1(2).N -> -Quad1(2).N
                        //--
                        linsol.SetDirection(linsol.Direction().Reversed());
                        glig = new IntPatch_GLine(linsol, false, trans1, trans2);
                        para = ElCLib.Parameter(linsol, apex);
                        ptsol.SetParameter(para);
                        glig.AddVertex(ptsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);

                        //------------------------------------------- Ligne 2 -------
                        linsol = inter.Line(2);
                        if (linsol.Direction().Dot(Co.Axis().Direction()) < 0.0)
                        {
                            linsol.SetDirection(linsol.Direction().Reversed());
                        }
                        para = ElCLib.Parameter(linsol, apex);
                        ptbid = ElCLib.Value(para + 5.0, linsol);
                        if (linsol.Direction().DotCross(new gp_Dir(Quad2.Normale(ptbid)), new gp_Dir(Quad1.Normale(ptbid)))
                            > 0.0)
                        {
                            trans1 = IntSurf_Out;
                            trans2 = IntSurf_In;
                        }
                        else
                        {
                            trans1 = IntSurf_In;
                            trans2 = IntSurf_Out;
                        }
                        ptsol.SetParameter(para);
                        glig = new IntPatch_GLine(linsol, false, trans1, trans2);
                        para = ElCLib.Parameter(linsol, apex);
                        ptsol.SetParameter(para);
                        glig.AddVertex(ptsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);
                        //-----------------------------------------------------------
                        //-- Other Side : Les transitions restent les memes
                        //--    linsol -> -linsol   et Quad1(2).N -> -Quad1(2).N
                        //--
                        linsol.SetDirection(linsol.Direction().Reversed());
                        glig = new IntPatch_GLine(linsol, false, trans1, trans2);
                        para = ElCLib.Parameter(linsol, apex);
                        ptsol.SetParameter(para);
                        glig.AddVertex(ptsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);
                    }
                }
                break;

            case IntAna_Circle:
                {
                    gp_Circ cirsol = inter.Circle(1);
                    // modified by NIZNHY-PKV Thu Sep 15 11:34:04 2011f
                    AdjustToSeam(Co, ref cirsol);
                    // modified by NIZNHY-PKV Thu Sep 15 11:36:08 2011t
                    ElCLib.D1(0.0, cirsol, out gp_Pnt ptref, out gp_Vec Tgt);

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
                }
                break;

            case IntAna_Ellipse:
                {
                    gp_Elips elipsol = inter.Ellipse(1);
                    ElCLib.D1(0.0, elipsol, out gp_Pnt ptref, out gp_Vec Tgt);

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
                    IntPatch_GLine glig = new(elipsol, false, trans1, trans2);
                    slin.Append(glig);
                }
                break;

            case IntAna_Parabola:
                {
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
                    slin.Append(glig);
                }
                break;

            case IntAna_Hyperbola:
                {
                    gp_Pnt tophypr;
                    gp_Vec Tgttop;

                    for (int i = 1; i <= 2; i++)
                    {
                        gp_Hypr hyprsol = inter.Hyperbola(i);
                        tophypr = ElCLib.Value(hyprsol.MajorRadius(), new gp_Lin(hyprsol.XAxis()));
                        Tgttop = new gp_Vec(hyprsol.YAxis().Direction());
                        double qwe = Tgttop.DotCross(Quad2.Normale(tophypr), Quad1.Normale(tophypr));

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
                        slin.Append(glig);
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

    private static bool IntPTo(in IntSurf_Quadric theQuad1,
                               in IntSurf_Quadric theQuad2,
                               double theTolTang,
                               bool bReversed,
                               ref bool bEmpty,
                               NCollection_Sequence<IntPatch_Line> theSeqLin)
    {
        gp_Pln aPln = bReversed ? theQuad2.Plane() : theQuad1.Plane();
        gp_Torus aTorus = bReversed ? theQuad1.Torus() : theQuad2.Torus();
        //
        IntAna_QuadQuadGeo inter = new(aPln, aTorus, theTolTang);
        bool bRet = inter.IsDone();
        //
        if (!bRet)
        {
            return bRet;
        }
        //
        IntAna_ResultType typint = inter.TypeInter();
        int NbSol = inter.NbSolutions();
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
                    //
                    for (i = 1; i <= NbSol; ++i)
                    {
                        gp_Circ aC = inter.Circle(i);
                        if (!aPln.Axis().IsNormal(aTorus.Axis(), Precision.Angular()))
                        {
                            AdjustToSeam(aTorus, ref aC);
                        }
                        ElCLib.D1(0.0, aC, out gp_Pnt ptref, out gp_Vec Tgt);
                        //
                        if (Tgt.DotCross(theQuad2.Normale(ptref), theQuad1.Normale(ptref)) > 0.0)
                        {
                            trans1 = IntSurf_Out;
                            trans2 = IntSurf_In;
                        }
                        else
                        {
                            trans1 = IntSurf_In;
                            trans2 = IntSurf_Out;
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

    //
    // modified by NIZNHY-PKV Thu Sep 15 10:53:39 2011f
    //=================================================================================================

    private static void AdjustToSeam(in gp_Cone aQuad, ref gp_Circ aCirc)
    {
        gp_Ax2 aAx2;
        //
        gp_Pnt aPLoc = aCirc.Location();
        gp_Ax3 aAx3 = aQuad.Position();
        SeamPosition(aPLoc, aAx3, out aAx2);
        aCirc.SetPosition(aAx2);
    }

    //=================================================================================================

    private static void AdjustToSeam(in gp_Sphere aQuad, ref gp_Circ aCirc, double aTolAng)
    {
        gp_Ax2 aAx2;
        //
        gp_Ax1 aAx1C = aCirc.Axis();
        gp_Ax3 aAx3 = aQuad.Position();
        gp_Ax1 aAx1Q = aAx3.Axis();
        //
        gp_Dir aDirC = aAx1C.Direction();
        gp_Dir aDirQ = aAx1Q.Direction();
        if (aDirC.IsParallel(aDirQ, aTolAng))
        {
            gp_Pnt aPLoc = aCirc.Location();
            SeamPosition(aPLoc, aAx3, out aAx2);
            aCirc.SetPosition(aAx2);
        }
    }

    //=================================================================================================

    private static void AdjustToSeam(in gp_Cylinder aQuad, ref gp_Circ aCirc)
    {
        gp_Ax2 aAx2;
        //
        gp_Pnt aPLoc = aCirc.Location();
        gp_Ax3 aAx3 = aQuad.Position();
        SeamPosition(aPLoc, aAx3, out aAx2);
        aCirc.SetPosition(aAx2);
    }

    //=================================================================================================

    private static void AdjustToSeam(in gp_Torus aQuad, ref gp_Circ aCirc)
    {
        gp_Ax2 aAx2;
        //
        gp_Pnt aPLoc = aCirc.Location();
        gp_Ax3 aAx3 = aQuad.Position();
        SeamPosition(aPLoc, aAx3, out aAx2);
        aCirc.SetPosition(aAx2);
    }

    //=================================================================================================

    private static void SeamPosition(in gp_Pnt aPLoc, in gp_Ax3 aPos, out gp_Ax2 aSeamPos)
    {
        gp_Dir aDZ = aPos.Direction();
        gp_Dir aDX = aPos.XDirection();
        gp_Ax2 aAx2 = new(aPLoc, aDZ, aDX);
        aSeamPos = aAx2;
    }

    // modified by NIZNHY-PKV Thu Sep 15 10:53:41 2011t
}
