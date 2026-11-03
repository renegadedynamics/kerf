// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.ResultType;
using static Kerf.Engine.Intersection.Walking.WalkingSituation;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Patches;

// The plane/quadric pairs (OCCT 7.x's IntPatch_ImpImpIntersection_3.gxx): plane/plane, plane/cylinder,
// plane/sphere, plane/cone and plane/torus, and the circles put on the seam of the quadric (AdjustToSeam).
//
// OCCT's gp_Dir::DotCross takes directions, so the normals OCCT passes to it as vectors are normalised by C++'s
// implicit conversion; the port writes the conversion (new gp_Dir(...)).
public sealed partial class ImpImpIntersection
{
    // modified by NIZNHY-PKV Thu Sep 15 11:09:13 2011

    //=======================================================================
    // function : IntPP
    // purpose  :
    // Traitement du cas Plan/Plan
    //=======================================================================
    private static bool IntPP(in SurfaceQuadric quad1,
                              in SurfaceQuadric quad2,
                              double tolang,
                              double tolTang,
                              ref bool same,
                              NCollection_Sequence<PatchLine> slin)
    {
        WalkingTypeTrans trans1, trans2;
        ResultType typint;
        Plane3D pl1 = quad1.Plane();
        Plane3D pl2 = quad2.Plane();

        QuadQuadGeo inter = new(pl1, pl2, tolang, tolTang);
        if (!inter.IsDone)
        {
            return false;
        }
        same = false;
        typint = inter.TypeInter();
        if (typint == global::Kerf.Engine.Intersection.ResultType.Same)
        { // cas faces confondues
            same = true;
        }
        else if (typint != Empty)
        { // on a une ligne
            Line3D linsol = inter.Line(1);
            double discri = linsol.Direction().DotCross(new Direction3D(quad2.Normale(linsol.Location())),
                                                        new Direction3D(quad1.Normale(linsol.Location())));

            if (discri > 0.0)
            {
                trans1 = WalkingTypeTrans.Out;
                trans2 = WalkingTypeTrans.In;
            }
            else
            {
                trans1 = WalkingTypeTrans.In;
                trans2 = WalkingTypeTrans.Out;
            }
            GLine glig = new(linsol, false, trans1, trans2);
            slin.Append(glig);
        }
        return true;
    }

    //=======================================================================
    // function : IntPCy
    // purpose  :
    // Traitement du cas Plan/Cylindre et reciproquement
    //=======================================================================
    private static bool IntPCy(in SurfaceQuadric quad1,
                               in SurfaceQuadric quad2,
                               double tolang,
                               double tolTang,
                               bool reversed,
                               ref bool empty,
                               NCollection_Sequence<PatchLine> slin,
                               double h = 0.0)
    {
        Plane3D pl;
        Cylinder cy;

        WalkingTypeTrans trans1, trans2;
        ResultType typint;

        QuadQuadGeo inter = new();
        if (!reversed)
        {
            pl = quad1.Plane();
            cy = quad2.Cylinder();
        }
        else
        {
            pl = quad2.Plane();
            cy = quad1.Cylinder();
        }
        inter.Perform(pl, cy, tolang, tolTang, h);
        if (!inter.IsDone)
        {
            return false;
        }
        typint = inter.TypeInter();
        int nbSol = inter.SolutionCount;
        empty = false;

        switch (typint)
        {
            case global::Kerf.Engine.Intersection.ResultType.Empty:
                {
                    empty = true;
                }
                break;

            case global::Kerf.Engine.Intersection.ResultType.Line:
                {
                    Line3D linsol = inter.Line(1);
                    Point3D orig = linsol.Location();
                    if (nbSol == 1)
                    { // ligne de tangence
                        Vector3D testCurvature = new(orig, cy.Location());
                        Vector3D normp, normcyl;
                        if (!reversed)
                        {
                            normp = quad1.Normale(orig);
                            normcyl = quad2.Normale(orig);
                        }
                        else
                        {
                            normp = quad2.Normale(orig);
                            normcyl = quad1.Normale(orig);
                        }

                        WalkingSituation situcyl;
                        WalkingSituation situp;

                        if (normp.Dot(testCurvature) > 0.0)
                        {
                            situcyl = WalkingSituation.Outside;
                            if (normp.Dot(normcyl) > 0.0)
                            {
                                situp = WalkingSituation.Inside;
                            }
                            else
                            {
                                situp = WalkingSituation.Outside;
                            }
                        }
                        else
                        {
                            situcyl = WalkingSituation.Inside;
                            if (normp.Dot(normcyl) > 0.0)
                            {
                                situp = WalkingSituation.Outside;
                            }
                            else
                            {
                                situp = WalkingSituation.Inside;
                            }
                        }
                        GLine glig;
                        if (!reversed)
                        {
                            glig = new GLine(linsol, true, situp, situcyl);
                        }
                        else
                        {
                            glig = new GLine(linsol, true, situcyl, situp);
                        }
                        slin.Append(glig);
                    }
                    else
                    {
                        // on a 2 droites. Il faut determiner les transitions
                        // de chacune.

                        if (linsol.Direction().DotCross(new Direction3D(quad2.Normale(orig)), new Direction3D(quad1.Normale(orig)))
                            > 0.0)
                        {
                            trans1 = WalkingTypeTrans.Out;
                            trans2 = WalkingTypeTrans.In;
                        }
                        else
                        {
                            trans1 = WalkingTypeTrans.In;
                            trans2 = WalkingTypeTrans.Out;
                        }
                        GLine glig = new(linsol, false, trans1, trans2);
                        slin.Append(glig);

                        linsol = inter.Line(2);
                        orig = linsol.Location();

                        if (linsol.Direction().DotCross(new Direction3D(quad2.Normale(orig)), new Direction3D(quad1.Normale(orig)))
                            > 0.0)
                        {
                            trans1 = WalkingTypeTrans.Out;
                            trans2 = WalkingTypeTrans.In;
                        }
                        else
                        {
                            trans1 = WalkingTypeTrans.In;
                            trans2 = WalkingTypeTrans.Out;
                        }
                        glig = new GLine(linsol, false, trans1, trans2);
                        slin.Append(glig);
                    }
                }
                break;
            //
            case Circle:
                {
                    Circle3D cirsol;
                    //
                    cirsol = inter.Circle(1);
                    // modified by NIZNHY-PKV Thu Sep 15 11:30:03 2011f
                    AdjustToSeam(cy, ref cirsol);
                    // modified by NIZNHY-PKV Thu Sep 15 11:30:15 2011t
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, cirsol, out Point3D ptref, out Vector3D tgt);

                    if (tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref)) > 0.0)
                    {
                        trans1 = WalkingTypeTrans.Out;
                        trans2 = WalkingTypeTrans.In;
                    }
                    else
                    {
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                    }
                    GLine glig = new(cirsol, false, trans1, trans2);
                    slin.Append(glig);
                }
                break;
            //
            case Ellipse:
                {
                    Ellipse3D elipsol = inter.Ellipse(1);
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, elipsol, out Point3D ptref, out Vector3D tgt);

                    if (tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref)) > 0.0)
                    {
                        trans1 = WalkingTypeTrans.Out;
                        trans2 = WalkingTypeTrans.In;
                    }
                    else
                    {
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                    }
                    GLine glig = new(elipsol, false, trans1, trans2);
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
    private static bool IntPSp(in SurfaceQuadric quad1,
                               in SurfaceQuadric quad2,
                               double tolang,
                               double tolTang,
                               bool reversed,
                               ref bool empty,
                               NCollection_Sequence<PatchLine> slin,
                               NCollection_Sequence<PatchPoint> spnt)
    {
        Circle3D cirsol;
        Plane3D pl;
        Sphere sp;
        WalkingTypeTrans trans1, trans2;
        ResultType typint;

        QuadQuadGeo inter = new();
        if (!reversed)
        {
            pl = quad1.Plane();
            sp = quad2.Sphere();
        }
        else
        {
            pl = quad2.Plane();
            sp = quad1.Sphere();
        }
        inter.Perform(pl, sp);

        if (!inter.IsDone)
        {
            return false;
        }

        typint = inter.TypeInter();
        empty = false;

        switch (typint)
        {
            case global::Kerf.Engine.Intersection.ResultType.Empty:
                {
                    empty = true;
                }
                break;
            //
            case global::Kerf.Engine.Intersection.ResultType.Point:
                {
                    Point3D psol = inter.Point(1);
                    double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
                    quad1.Parameters(psol, ref u1, ref v1);
                    quad2.Parameters(psol, ref u2, ref v2);
                    PatchPoint ptsol = new();
                    ptsol.SetValue(psol, tolTang, true);
                    ptsol.SetParameters(u1, v1, u2, v2);
                    spnt.Append(ptsol);
                }
                break;
            //
            case Circle:
                {
                    cirsol = inter.Circle(1);
                    // modified by NIZNHY-PKV Thu Sep 15 11:30:03 2011f
                    AdjustToSeam(sp, ref cirsol, tolang);
                    // modified by NIZNHY-PKV Thu Sep 15 11:30:15 2011t
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, cirsol, out Point3D ptref, out Vector3D tgt);

                    if (tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref)) > 0.0)
                    {
                        trans1 = WalkingTypeTrans.Out;
                        trans2 = WalkingTypeTrans.In;
                    }
                    else
                    {
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                    }
                    GLine glig = new(cirsol, false, trans1, trans2);
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
    private static bool IntPCo(in SurfaceQuadric quad1,
                               in SurfaceQuadric quad2,
                               double tolang,
                               double tolTang,
                               bool reversed,
                               ref bool empty,
                               ref bool multpoint,
                               NCollection_Sequence<PatchLine> slin,
                               NCollection_Sequence<PatchPoint> spnt)
    {
        Point3D apex;

        Plane3D pl;
        Cone co;

        WalkingTypeTrans trans1, trans2;
        ResultType typint;

        QuadQuadGeo inter = new();
        if (!reversed)
        {
            pl = quad1.Plane();
            co = quad2.Cone();
            apex = co.Apex();
        }
        else
        {
            pl = quad2.Plane();
            co = quad1.Cone();
            apex = co.Apex();
        }

        inter.Perform(pl, co, tolang, tolTang);
        if (!inter.IsDone)
        {
            return false;
        }
        //
        typint = inter.TypeInter();
        int nbSol = inter.SolutionCount;
        empty = false;

        switch (typint)
        {
            case global::Kerf.Engine.Intersection.ResultType.Point:
                {
                    Point3D psol = inter.Point(1);
                    double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
                    quad1.Parameters(psol, ref u1, ref v1);
                    quad2.Parameters(psol, ref u2, ref v2);
                    PatchPoint ptsol = new();
                    ptsol.SetValue(psol, tolTang, false);
                    ptsol.SetParameters(u1, v1, u2, v2);
                    spnt.Append(ptsol);
                }
                break;

            case global::Kerf.Engine.Intersection.ResultType.Line:
                {
                    Line3D linsol = inter.Line(1);
                    if (linsol.Direction().Dot(co.Axis().Direction()) < 0.0)
                    {
                        linsol.SetDirection(linsol.Direction().Reversed());
                    }
                    double para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, apex);
                    Point3D ptbid = global::Kerf.Engine.Geometry.ElCLib.Value(para + 5.0, linsol);
                    double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
                    quad1.Parameters(apex, ref u1, ref v1);
                    quad2.Parameters(apex, ref u2, ref v2);

                    if (nbSol == 1)
                    { // ligne de tangence
                        PatchPoint ptsol = new();
                        ptsol.SetValue(apex, tolTang, false);
                        ptsol.SetParameters(u1, v1, u2, v2);
                        ptsol.SetParameter(para);
                        Point3D ptbid2 = new(apex.XYZ() + 5.0 * co.Axis().Direction().XYZ());
                        Vector3D testCurvature = new(ptbid, ptbid2);
                        Vector3D normp, normco;
                        if (!reversed)
                        {
                            normp = quad1.Normale(ptbid);
                            normco = quad2.Normale(ptbid);
                        }
                        else
                        {
                            normp = quad2.Normale(ptbid);
                            normco = quad1.Normale(ptbid);
                        }
                        WalkingSituation situco, situco_otherside;
                        WalkingSituation situp, situp_otherside;

                        if (normp.Dot(testCurvature) > 0.0)
                        {
                            situco = WalkingSituation.Outside;
                            situco_otherside = WalkingSituation.Inside;
                            if (normp.Dot(normco) > 0.0)
                            {
                                situp = WalkingSituation.Inside;
                                situp_otherside = WalkingSituation.Outside;
                            }
                            else
                            {
                                situp = WalkingSituation.Outside;
                                situp_otherside = WalkingSituation.Inside;
                            }
                        }
                        else
                        {
                            situco = WalkingSituation.Inside;
                            situco_otherside = WalkingSituation.Outside;
                            if (normp.Dot(normco) > 0.0)
                            {
                                situp = WalkingSituation.Outside;
                                situp_otherside = WalkingSituation.Inside;
                            }
                            else
                            {
                                situp = WalkingSituation.Inside;
                                situp_otherside = WalkingSituation.Outside;
                            }
                        }
                        //----------------------------------------------------------
                        //--              Apex ---> Cone.Direction
                        //--
                        GLine glig;
                        if (!reversed)
                        {
                            glig = new GLine(linsol, true, situp, situco);
                        }
                        else
                        {
                            glig = new GLine(linsol, true, situco, situp);
                        }
                        glig.AddVertex(ptsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);
                        //----------------------------------------------------------
                        //--   -Cone.Direction <------- Apex
                        //--
                        linsol.SetDirection(linsol.Direction().Reversed());
                        if (!reversed)
                        {
                            glig = new GLine(linsol, true, situp_otherside, situco_otherside);
                        }
                        else
                        {
                            glig = new GLine(linsol, true, situco_otherside, situp_otherside);
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
                        if (linsol.Direction().DotCross(new Direction3D(quad2.Normale(ptbid)), new Direction3D(quad1.Normale(ptbid)))
                            > 0.0)
                        {
                            trans1 = WalkingTypeTrans.Out;
                            trans2 = WalkingTypeTrans.In;
                        }
                        else
                        {
                            trans1 = WalkingTypeTrans.In;
                            trans2 = WalkingTypeTrans.Out;
                        }

                        multpoint = true;
                        //------------------------------------------- Ligne 1 -------
                        PatchPoint ptsol = new();
                        ptsol.SetValue(apex, tolTang, false);
                        ptsol.SetParameters(u1, v1, u2, v2);
                        ptsol.SetParameter(para);
                        ptsol.SetMultiple(true);
                        GLine glig;
                        glig = new GLine(linsol, false, trans1, trans2);
                        glig.AddVertex(ptsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);
                        //-----------------------------------------------------------
                        //-- Other Side : Les transitions restent les memes
                        //--    linsol -> -linsol   et Quad1(2).N -> -Quad1(2).N
                        //--
                        linsol.SetDirection(linsol.Direction().Reversed());
                        glig = new GLine(linsol, false, trans1, trans2);
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, apex);
                        ptsol.SetParameter(para);
                        glig.AddVertex(ptsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);

                        //------------------------------------------- Ligne 2 -------
                        linsol = inter.Line(2);
                        if (linsol.Direction().Dot(co.Axis().Direction()) < 0.0)
                        {
                            linsol.SetDirection(linsol.Direction().Reversed());
                        }
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, apex);
                        ptbid = global::Kerf.Engine.Geometry.ElCLib.Value(para + 5.0, linsol);
                        if (linsol.Direction().DotCross(new Direction3D(quad2.Normale(ptbid)), new Direction3D(quad1.Normale(ptbid)))
                            > 0.0)
                        {
                            trans1 = WalkingTypeTrans.Out;
                            trans2 = WalkingTypeTrans.In;
                        }
                        else
                        {
                            trans1 = WalkingTypeTrans.In;
                            trans2 = WalkingTypeTrans.Out;
                        }
                        ptsol.SetParameter(para);
                        glig = new GLine(linsol, false, trans1, trans2);
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
                        glig = new GLine(linsol, false, trans1, trans2);
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, apex);
                        ptsol.SetParameter(para);
                        glig.AddVertex(ptsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);
                    }
                }
                break;

            case Circle:
                {
                    Circle3D cirsol = inter.Circle(1);
                    // modified by NIZNHY-PKV Thu Sep 15 11:34:04 2011f
                    AdjustToSeam(co, ref cirsol);
                    // modified by NIZNHY-PKV Thu Sep 15 11:36:08 2011t
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, cirsol, out Point3D ptref, out Vector3D tgt);

                    if (tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref)) > 0.0)
                    {
                        trans1 = WalkingTypeTrans.Out;
                        trans2 = WalkingTypeTrans.In;
                    }
                    else
                    {
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                    }
                    GLine glig = new(cirsol, false, trans1, trans2);
                    slin.Append(glig);
                }
                break;

            case Ellipse:
                {
                    Ellipse3D elipsol = inter.Ellipse(1);
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, elipsol, out Point3D ptref, out Vector3D tgt);

                    if (tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref)) > 0.0)
                    {
                        trans1 = WalkingTypeTrans.Out;
                        trans2 = WalkingTypeTrans.In;
                    }
                    else
                    {
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                    }
                    GLine glig = new(elipsol, false, trans1, trans2);
                    slin.Append(glig);
                }
                break;

            case Parabola:
                {
                    Parabola3D parabsol = inter.Parabola(1);

                    Vector3D tgtorig = new(parabsol.YAxis().Direction());
                    double ptran =
                        tgtorig.DotCross(quad2.Normale(parabsol.Location()), quad1.Normale(parabsol.Location()));
                    if (ptran > 0.00000001)
                    {
                        trans1 = WalkingTypeTrans.Out;
                        trans2 = WalkingTypeTrans.In;
                    }
                    else if (ptran < -0.00000001)
                    {
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                    }
                    else
                    {
                        trans1 = trans2 = WalkingTypeTrans.Undecided;
                    }
                    GLine glig = new(parabsol, false, trans1, trans2);
                    slin.Append(glig);
                }
                break;

            case Hyperbola:
                {
                    Point3D tophypr;
                    Vector3D tgttop;

                    for (int i = 1; i <= 2; i++)
                    {
                        Hyperbola3D hyprsol = inter.Hyperbola(i);
                        tophypr = global::Kerf.Engine.Geometry.ElCLib.Value(hyprsol.MajorRadius(), new Line3D(hyprsol.XAxis()));
                        tgttop = new Vector3D(hyprsol.YAxis().Direction());
                        double qwe = tgttop.DotCross(quad2.Normale(tophypr), quad1.Normale(tophypr));

                        if (qwe > 0.00000001)
                        {
                            trans1 = WalkingTypeTrans.Out;
                            trans2 = WalkingTypeTrans.In;
                        }
                        else if (qwe < -0.00000001)
                        {
                            trans1 = WalkingTypeTrans.In;
                            trans2 = WalkingTypeTrans.Out;
                        }
                        else
                        {
                            trans1 = trans2 = WalkingTypeTrans.Undecided;
                        }
                        GLine glig = new(hyprsol, false, trans1, trans2);
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

    private static bool IntPTo(in SurfaceQuadric quad1,
                               in SurfaceQuadric quad2,
                               double tolTang,
                               bool bReversed,
                               ref bool bEmpty,
                               NCollection_Sequence<PatchLine> seqLin)
    {
        Plane3D pln = bReversed ? quad2.Plane() : quad1.Plane();
        Torus torus = bReversed ? quad1.Torus() : quad2.Torus();
        //
        QuadQuadGeo inter = new(pln, torus, tolTang);
        bool bRet = inter.IsDone;
        //
        if (!bRet)
        {
            return bRet;
        }
        //
        ResultType typint = inter.TypeInter();
        int nbSol = inter.SolutionCount;
        bEmpty = false;
        //
        switch (typint)
        {
            case Empty:
                bEmpty = true;
                break;
            //
            case Circle:
                {
                    int i;
                    WalkingTypeTrans trans1, trans2;
                    //
                    for (i = 1; i <= nbSol; ++i)
                    {
                        Circle3D c = inter.Circle(i);
                        if (!pln.Axis().IsNormal(torus.Axis(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            AdjustToSeam(torus, ref c);
                        }
                        global::Kerf.Engine.Geometry.ElCLib.D1(0.0, c, out Point3D ptref, out Vector3D tgt);
                        //
                        if (tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref)) > 0.0)
                        {
                            trans1 = WalkingTypeTrans.Out;
                            trans2 = WalkingTypeTrans.In;
                        }
                        else
                        {
                            trans1 = WalkingTypeTrans.In;
                            trans2 = WalkingTypeTrans.Out;
                        }
                        //
                        GLine glig = new(c, false, trans1, trans2);
                        seqLin.Append(glig);
                    }
                }
                break;
            //
            case NoGeometricSolution:
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
        Frame3D ax2;
        //
        Point3D pLoc = aCirc.Location();
        CoordinateSystem ax3 = aQuad.Position();
        SeamPosition(pLoc, ax3, out ax2);
        aCirc.SetPosition(ax2);
    }

    //=================================================================================================

    private static void AdjustToSeam(in Sphere aQuad, ref Circle3D aCirc, double aTolAng)
    {
        Frame3D ax2;
        //
        Axis3D ax1C = aCirc.Axis();
        CoordinateSystem ax3 = aQuad.Position();
        Axis3D ax1Q = ax3.Axis();
        //
        Direction3D dirC = ax1C.Direction();
        Direction3D dirQ = ax1Q.Direction();
        if (dirC.IsParallel(dirQ, aTolAng))
        {
            Point3D pLoc = aCirc.Location();
            SeamPosition(pLoc, ax3, out ax2);
            aCirc.SetPosition(ax2);
        }
    }

    //=================================================================================================

    private static void AdjustToSeam(in Cylinder aQuad, ref Circle3D aCirc)
    {
        Frame3D ax2;
        //
        Point3D pLoc = aCirc.Location();
        CoordinateSystem ax3 = aQuad.Position();
        SeamPosition(pLoc, ax3, out ax2);
        aCirc.SetPosition(ax2);
    }

    //=================================================================================================

    private static void AdjustToSeam(in Torus aQuad, ref Circle3D aCirc)
    {
        Frame3D ax2;
        //
        Point3D pLoc = aCirc.Location();
        CoordinateSystem ax3 = aQuad.Position();
        SeamPosition(pLoc, ax3, out ax2);
        aCirc.SetPosition(ax2);
    }

    //=================================================================================================

    private static void SeamPosition(in Point3D aPLoc, in CoordinateSystem aPos, out Frame3D aSeamPos)
    {
        Direction3D dz = aPos.Direction();
        Direction3D dx = aPos.XDirection();
        Frame3D ax2 = new(aPLoc, dz, dx);
        aSeamPos = ax2;
    }

    // modified by NIZNHY-PKV Thu Sep 15 10:53:41 2011t
}
