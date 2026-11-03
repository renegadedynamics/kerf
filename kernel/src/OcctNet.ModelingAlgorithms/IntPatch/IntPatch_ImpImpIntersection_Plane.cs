// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
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
        Plane3D pl1 = Quad1.Plane();
        Plane3D pl2 = Quad2.Plane();

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
            Line3D linsol = inter.Line(1);
            double discri = linsol.Direction().DotCross(new Direction3D(Quad2.Normale(linsol.Location())),
                                                        new Direction3D(Quad1.Normale(linsol.Location())));

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
        Plane3D Pl;
        Cylinder Cy;

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
                    Line3D linsol = inter.Line(1);
                    Point3D orig = linsol.Location();
                    if (NbSol == 1)
                    { // ligne de tangence
                        Vector3D TestCurvature = new(orig, Cy.Location());
                        Vector3D Normp, Normcyl;
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

                        if (linsol.Direction().DotCross(new Direction3D(Quad2.Normale(orig)), new Direction3D(Quad1.Normale(orig)))
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

                        if (linsol.Direction().DotCross(new Direction3D(Quad2.Normale(orig)), new Direction3D(Quad1.Normale(orig)))
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
                    Circle3D cirsol;
                    //
                    cirsol = inter.Circle(1);
                    // modified by NIZNHY-PKV Thu Sep 15 11:30:03 2011f
                    AdjustToSeam(Cy, ref cirsol);
                    // modified by NIZNHY-PKV Thu Sep 15 11:30:15 2011t
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, cirsol, out Point3D ptref, out Vector3D Tgt);

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
                    Ellipse3D elipsol = inter.Ellipse(1);
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, elipsol, out Point3D ptref, out Vector3D Tgt);

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
        Circle3D cirsol;
        Plane3D Pl;
        Sphere Sp;
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
                    Point3D psol = inter.Point(1);
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
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, cirsol, out Point3D ptref, out Vector3D Tgt);

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
        Point3D apex;

        Plane3D Pl;
        Cone Co;

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
                    Point3D psol = inter.Point(1);
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
                    Line3D linsol = inter.Line(1);
                    if (linsol.Direction().Dot(Co.Axis().Direction()) < 0.0)
                    {
                        linsol.SetDirection(linsol.Direction().Reversed());
                    }
                    double para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, apex);
                    Point3D ptbid = global::Kerf.Engine.Geometry.ElCLib.Value(para + 5.0, linsol);
                    double U1 = 0.0, V1 = 0.0, U2 = 0.0, V2 = 0.0;
                    Quad1.Parameters(apex, ref U1, ref V1);
                    Quad2.Parameters(apex, ref U2, ref V2);

                    if (NbSol == 1)
                    { // ligne de tangence
                        IntPatch_Point ptsol = new();
                        ptsol.SetValue(apex, TolTang, false);
                        ptsol.SetParameters(U1, V1, U2, V2);
                        ptsol.SetParameter(para);
                        Point3D ptbid2 = new(apex.XYZ() + 5.0 * Co.Axis().Direction().XYZ());
                        Vector3D TestCurvature = new(ptbid, ptbid2);
                        Vector3D Normp, Normco;
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
                        if (linsol.Direction().DotCross(new Direction3D(Quad2.Normale(ptbid)), new Direction3D(Quad1.Normale(ptbid)))
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
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, apex);
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
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, apex);
                        ptbid = global::Kerf.Engine.Geometry.ElCLib.Value(para + 5.0, linsol);
                        if (linsol.Direction().DotCross(new Direction3D(Quad2.Normale(ptbid)), new Direction3D(Quad1.Normale(ptbid)))
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
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, apex);
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
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, apex);
                        ptsol.SetParameter(para);
                        glig.AddVertex(ptsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);
                    }
                }
                break;

            case IntAna_Circle:
                {
                    Circle3D cirsol = inter.Circle(1);
                    // modified by NIZNHY-PKV Thu Sep 15 11:34:04 2011f
                    AdjustToSeam(Co, ref cirsol);
                    // modified by NIZNHY-PKV Thu Sep 15 11:36:08 2011t
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, cirsol, out Point3D ptref, out Vector3D Tgt);

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
                    Ellipse3D elipsol = inter.Ellipse(1);
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, elipsol, out Point3D ptref, out Vector3D Tgt);

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
                    Parabola3D parabsol = inter.Parabola(1);

                    Vector3D Tgtorig = new(parabsol.YAxis().Direction());
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
                    Point3D tophypr;
                    Vector3D Tgttop;

                    for (int i = 1; i <= 2; i++)
                    {
                        Hyperbola3D hyprsol = inter.Hyperbola(i);
                        tophypr = global::Kerf.Engine.Geometry.ElCLib.Value(hyprsol.MajorRadius(), new Line3D(hyprsol.XAxis()));
                        Tgttop = new Vector3D(hyprsol.YAxis().Direction());
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
        Plane3D aPln = bReversed ? theQuad2.Plane() : theQuad1.Plane();
        Torus aTorus = bReversed ? theQuad1.Torus() : theQuad2.Torus();
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
                        Circle3D aC = inter.Circle(i);
                        if (!aPln.Axis().IsNormal(aTorus.Axis(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            AdjustToSeam(aTorus, ref aC);
                        }
                        global::Kerf.Engine.Geometry.ElCLib.D1(0.0, aC, out Point3D ptref, out Vector3D Tgt);
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

    private static void AdjustToSeam(in Cone aQuad, ref Circle3D aCirc)
    {
        Frame3D aAx2;
        //
        Point3D aPLoc = aCirc.Location();
        CoordinateSystem aAx3 = aQuad.Position();
        SeamPosition(aPLoc, aAx3, out aAx2);
        aCirc.SetPosition(aAx2);
    }

    //=================================================================================================

    private static void AdjustToSeam(in Sphere aQuad, ref Circle3D aCirc, double aTolAng)
    {
        Frame3D aAx2;
        //
        Axis3D aAx1C = aCirc.Axis();
        CoordinateSystem aAx3 = aQuad.Position();
        Axis3D aAx1Q = aAx3.Axis();
        //
        Direction3D aDirC = aAx1C.Direction();
        Direction3D aDirQ = aAx1Q.Direction();
        if (aDirC.IsParallel(aDirQ, aTolAng))
        {
            Point3D aPLoc = aCirc.Location();
            SeamPosition(aPLoc, aAx3, out aAx2);
            aCirc.SetPosition(aAx2);
        }
    }

    //=================================================================================================

    private static void AdjustToSeam(in Cylinder aQuad, ref Circle3D aCirc)
    {
        Frame3D aAx2;
        //
        Point3D aPLoc = aCirc.Location();
        CoordinateSystem aAx3 = aQuad.Position();
        SeamPosition(aPLoc, aAx3, out aAx2);
        aCirc.SetPosition(aAx2);
    }

    //=================================================================================================

    private static void AdjustToSeam(in Torus aQuad, ref Circle3D aCirc)
    {
        Frame3D aAx2;
        //
        Point3D aPLoc = aCirc.Location();
        CoordinateSystem aAx3 = aQuad.Position();
        SeamPosition(aPLoc, aAx3, out aAx2);
        aCirc.SetPosition(aAx2);
    }

    //=================================================================================================

    private static void SeamPosition(in Point3D aPLoc, in CoordinateSystem aPos, out Frame3D aSeamPos)
    {
        Direction3D aDZ = aPos.Direction();
        Direction3D aDX = aPos.XDirection();
        Frame3D aAx2 = new(aPLoc, aDZ, aDX);
        aSeamPos = aAx2;
    }

    // modified by NIZNHY-PKV Thu Sep 15 10:53:41 2011t
}
