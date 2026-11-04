// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Intersection;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.ResultType;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Engine.Intersection.Walking.WalkingSituation;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Patches;

// The quadric pairs other than plane/quadric and cylinder/cylinder: cylinder/sphere, cylinder/cone, cone/cone,
// cone/sphere, sphere/sphere and the tori (OCCT 7.x's IntPatch_ImpImpIntersection_5.gxx), and ProcessBounds, which
// puts the end points of their analytic lines.
internal sealed partial class ImpImpIntersection
{
    //=======================================================================
    // function : ProcessBounds
    // purpose  : Puts the first and last points of the analytic line alig as vertices, sharing the vertices of
    //            the analytic lines already in slin that coincide with them.
    //=======================================================================
    private static void ProcessBounds(ALine alig, //-- ligne courante
                                      NCollection_Sequence<PatchLine> slin,
                                      in SurfaceQuadric quad1,
                                      in SurfaceQuadric quad2,
                                      ref bool procf,
                                      in Point3D ptf,   //-- Debut Ligne Courante
                                      double first,    //-- Paramf
                                      ref bool procl,
                                      in Point3D ptl,   //-- Fin Ligne courante
                                      double last,     //-- Paraml
                                      ref bool multpoint,
                                      double tol)
    {
        int j, k;
        double u1 = 0, v1 = 0, u2 = 0, v2 = 0;
        PatchPoint ptsol = new();
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
            if (slin.Value(j).ArcType() == PatchLineType.Analytic)
            {
                ALine aligold = (ALine)slin.Value(j);
                k = 1;

                //-- On prend les vertex des lignes deja enregistrees

                while (k <= aligold.VertexCount)
                {
                    ptsol = aligold.Vertex(k);
                    if (!procf)
                    {
                        d = ptf.DistanceTo(ptsol.Value());
                        if (d <= tol)
                        {
                            ptsol.SetTolerance(tol);
                            if (!ptsol.IsMultiple)
                            {
                                //-- le point ptsol (de aligold) est declare multiple sur aligold
                                multpoint = true;
                                ptsol.SetMultiple(true);
                                aligold.Replace(k, ptsol);
                            }
                            ptsol.SetParameter(first);
                            alig.AddVertex(ptsol);
                            alig.SetFirstPoint(alig.VertexCount);
                            procf = true;

                            //-- On restore le point avec son parametre sur aligold
                            ptsol = aligold.Vertex(k);
                        }
                    }
                    if (!procl)
                    {
                        if (ptl.DistanceTo(ptsol.Value()) <= tol)
                        {
                            ptsol.SetTolerance(tol);
                            if (!ptsol.IsMultiple)
                            {
                                multpoint = true;
                                ptsol.SetMultiple(true);
                                aligold.Replace(k, ptsol);
                            }
                            ptsol.SetParameter(last);
                            alig.AddVertex(ptsol);
                            alig.SetLastPoint(alig.VertexCount);
                            procl = true;

                            //-- On restore le point avec son parametre sur aligold
                            ptsol = aligold.Vertex(k);
                        }
                    }
                    if (procf && procl)
                    {
                        k = aligold.VertexCount + 1;
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

        ptsol.SetTolerance(tol);
        if (!procf && !procl)
        {
            quad1.Parameters(ptf, ref u1, ref v1);
            quad2.Parameters(ptf, ref u2, ref v2);
            ptsol.SetValue(ptf, tol, false);
            ptsol.SetParameters(u1, v1, u2, v2);
            ptsol.SetParameter(first);
            if (ptf.DistanceTo(ptl) <= tol)
            {
                ptsol.SetMultiple(true); // a voir
                multpoint = true;        // a voir de meme
                alig.AddVertex(ptsol);
                alig.SetFirstPoint(alig.VertexCount);

                ptsol.SetParameter(last);
                alig.AddVertex(ptsol);
                alig.SetLastPoint(alig.VertexCount);
            }
            else
            {
                alig.AddVertex(ptsol);
                alig.SetFirstPoint(alig.VertexCount);
                quad1.Parameters(ptl, ref u1, ref v1);
                quad2.Parameters(ptl, ref u2, ref v2);
                ptsol.SetValue(ptl, tol, false);
                ptsol.SetParameters(u1, v1, u2, v2);
                ptsol.SetParameter(last);
                alig.AddVertex(ptsol);
                alig.SetLastPoint(alig.VertexCount);
            }
        }
        else if (!procf)
        {
            quad1.Parameters(ptf, ref u1, ref v1);
            quad2.Parameters(ptf, ref u2, ref v2);
            ptsol.SetValue(ptf, tol, false);
            ptsol.SetParameters(u1, v1, u2, v2);
            ptsol.SetParameter(first);
            alig.AddVertex(ptsol);
            alig.SetFirstPoint(alig.VertexCount);
        }
        else if (!procl)
        {
            quad1.Parameters(ptl, ref u1, ref v1);
            quad2.Parameters(ptl, ref u2, ref v2);
            ptsol.SetValue(ptl, tol, false);
            ptsol.SetParameters(u1, v1, u2, v2);
            ptsol.SetParameter(last);
            alig.AddVertex(ptsol);
            alig.SetLastPoint(alig.VertexCount);
        }
    }

    //=================================================================================================

    private static bool IntCySp(in SurfaceQuadric quad1,
                                in SurfaceQuadric quad2,
                                double tol,
                                bool reversed,
                                ref bool empty,
                                ref bool multpoint,
                                NCollection_Sequence<PatchLine> slin,
                                NCollection_Sequence<PatchPoint> spnt)
    {
        int i;

        WalkingTypeTrans trans1, trans2;
        ResultType typint;
        PatchPoint ptsol = new();
        Circle3D cirsol;

        Cylinder cy;
        Sphere sp;

        if (!reversed)
        {
            cy = quad1.Cylinder();
            sp = quad2.Sphere();
        }
        else
        {
            cy = quad2.Cylinder();
            sp = quad1.Sphere();
        }
        QuadQuadGeo inter = new(cy, sp, tol);

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

            case global::Kerf.Engine.Intersection.ResultType.Point:
            {
                Point3D psol = inter.Point(1);
                double u1 = 0, v1 = 0, u2 = 0, v2 = 0;
                quad1.Parameters(psol, ref u1, ref v1);
                quad2.Parameters(psol, ref u2, ref v2);
                ptsol.SetValue(psol, tol, true);
                ptsol.SetParameters(u1, v1, u2, v2);
                spnt.Append(ptsol);
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Circle:
            {
                cirsol = inter.Circle(1);
                global::Kerf.Engine.Geometry.ElCLib.D1(0.0, cirsol, out Point3D ptref, out Vector3D tgt);

                if (nbSol == 1)
                {
                    Vector3D testCurvature = new(ptref, sp.Location);
                    Vector3D normsp, normcyl;
                    if (!reversed)
                    {
                        normcyl = quad1.Normale(ptref);
                        normsp = quad2.Normale(ptref);
                    }
                    else
                    {
                        normcyl = quad2.Normale(ptref);
                        normsp = quad1.Normale(ptref);
                    }

                    WalkingSituation situcyl;
                    WalkingSituation situsp;

                    if (normcyl.Dot(testCurvature) > 0.0)
                    {
                        situsp = WalkingSituation.Outside;
                        if (normsp.Dot(normcyl) > 0.0)
                        {
                            situcyl = WalkingSituation.Inside;
                        }
                        else
                        {
                            situcyl = WalkingSituation.Outside;
                        }
                    }
                    else
                    {
                        situsp = WalkingSituation.Inside;
                        if (normsp.Dot(normcyl) > 0.0)
                        {
                            situcyl = WalkingSituation.Outside;
                        }
                        else
                        {
                            situcyl = WalkingSituation.Inside;
                        }
                    }
                    GLine glig;
                    if (!reversed)
                    {
                        glig = new GLine(cirsol, true, situcyl, situsp);
                    }
                    else
                    {
                        glig = new GLine(cirsol, true, situsp, situcyl);
                    }
                    slin.Append(glig);
                }
                else
                {
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

                    cirsol = inter.Circle(2);
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, cirsol, out ptref, out tgt);
                    double qwe = tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref));
                    if (qwe > 0.0000001)
                    {
                        trans1 = WalkingTypeTrans.Out;
                        trans2 = WalkingTypeTrans.In;
                    }
                    else if (qwe < -0.0000001)
                    {
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                    }
                    else
                    {
                        trans1 = trans2 = WalkingTypeTrans.Undecided;
                    }
                    glig = new GLine(cirsol, false, trans1, trans2);
                    slin.Append(glig);
                }
            }
            break;

            case NoGeometricSolution:
            {
                Point3D psol;
                double u1 = 0, v1 = 0, u2 = 0, v2 = 0;
                IntQuadQuad anaint = new(cy, new Quadric(sp), tol);
                if (!anaint.IsDone)
                {
                    return false;
                }

                if (anaint.PntCount == 0 && anaint.CurveCount == 0)
                {
                    empty = true;
                }
                else
                {
                    nbSol = anaint.PntCount;
                    for (i = 1; i <= nbSol; i++)
                    {
                        psol = anaint.Point(i);
                        quad1.Parameters(psol, ref u1, ref v1);
                        quad2.Parameters(psol, ref u2, ref v2);
                        ptsol.SetValue(psol, tol, true);
                        ptsol.SetParameters(u1, v1, u2, v2);
                        spnt.Append(ptsol);
                    }

                    Point3D ptvalid = new(), ptf, ptl;
                    Vector3D tgvalid = new();
                    double first, last, para;
                    Curve curvsol;
                    bool tgfound;
                    int kount;

                    nbSol = anaint.CurveCount;
                    for (i = 1; i <= nbSol; i++)
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
                        ALine alig;
                        if (kount <= 5)
                        {
                            double qwe = tgvalid.DotCross(quad2.Normale(ptvalid), quad1.Normale(ptvalid));
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
                            alig = new ALine(curvsol, false, trans1, trans2);
                        }
                        else
                        {
                            alig = new ALine(curvsol, false);
                        }
                        bool tempFalse1a = false;
                        bool tempFalse2a = false;

                        //-- ptf et ptl : points debut et fin de alig

                        ProcessBounds(alig,
                                      slin,
                                      quad1,
                                      quad2,
                                      ref tempFalse1a,
                                      ptf,
                                      first,
                                      ref tempFalse2a,
                                      ptl,
                                      last,
                                      ref multpoint,
                                      tol);
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

    private static bool IntCyCo(in SurfaceQuadric quad1,
                                in SurfaceQuadric quad2,
                                double tol,
                                bool reversed,
                                ref bool empty,
                                ref bool multpoint,
                                NCollection_Sequence<PatchLine> slin,
                                NCollection_Sequence<PatchPoint> spnt)
    {
        PatchPoint ptsol = new();

        int i;

        WalkingTypeTrans trans1, trans2;
        ResultType typint;
        Circle3D cirsol;

        Cylinder cy;
        Cone co;

        if (!reversed)
        {
            cy = quad1.Cylinder();
            co = quad2.Cone();
        }
        else
        {
            cy = quad2.Cylinder();
            co = quad1.Cone();
        }
        QuadQuadGeo inter = new(cy, co, tol);

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

            case global::Kerf.Engine.Intersection.ResultType.Point:
            {
                Point3D psol = inter.Point(1);
                double u1 = 0, v1 = 0, u2 = 0, v2 = 0;
                quad1.Parameters(psol, ref u1, ref v1);
                quad1.Parameters(psol, ref u2, ref v2); // OCCT: Quad1 for the second surface's parameters
                ptsol.SetValue(psol, tol, true);
                ptsol.SetParameters(u1, v1, u2, v2);
                spnt.Append(ptsol);
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Circle:
            {
                Vector3D tgt;
                Point3D ptref;
                int j;
                double qwe;
                //
                for (j = 1; j <= 2; ++j)
                {
                    cirsol = inter.Circle(j);
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, cirsol, out ptref, out tgt);
                    qwe = tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref));
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
                    GLine glig = new(cirsol, false, trans1, trans2);
                    slin.Append(glig);
                }
            }
            break;

            case NoGeometricSolution:
            {
                Point3D psol;
                double u1 = 0, v1 = 0, u2 = 0, v2 = 0;
                IntQuadQuad anaint = new(cy, new Quadric(co), tol);
                if (!anaint.IsDone)
                {
                    return false;
                }

                if (anaint.PntCount == 0 && anaint.CurveCount == 0)
                {
                    empty = true;
                }
                else
                {
                    nbSol = anaint.PntCount;
                    for (i = 1; i <= nbSol; i++)
                    {
                        psol = anaint.Point(i);
                        quad1.Parameters(psol, ref u1, ref v1);
                        quad2.Parameters(psol, ref u2, ref v2);
                        ptsol.SetValue(psol, tol, true);
                        ptsol.SetParameters(u1, v1, u2, v2);
                        spnt.Append(ptsol);
                    }

                    // ptf and ptl keep their values from the previous curve where the bound is open, as OCCT's.
                    // OCCT: kept across the curves; an open bound passes the previous curve's point
                    Point3D ptvalid = new(), ptf = new(), ptl = new();
                    Vector3D tgvalid = new();
                    //
                    double first, last, para;
                    bool tgfound, firstp, lastp, kept;
                    int kount;
                    //
                    //
                    // IntAna_Curve curvsol;
                    Curve c;
                    NCollection_List<Curve> lc = new();
                    NCollection_List<Curve>.Iterator it = new();

                    //
                    nbSol = anaint.CurveCount;
                    for (i = 1; i <= nbSol; ++i)
                    {
                        kept = false;
                        // curvsol = anaint.Curve(i);
                        c = anaint.Curve(i);
                        lc.Clear();
                        ExploreCurve(co, ref c, 10.0 * tol, lc);
                        //
                        it.Initialize(lc);
                        for (; it.More(); it.Next())
                        {
                            ref Curve curvsol = ref it.ChangeValue();
                            //
                            curvsol.Domain(out first, out last);
                            firstp = !curvsol.IsFirstOpen;
                            lastp = !curvsol.IsLastOpen;
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
                            ALine alig;
                            if (kount <= 5)
                            {
                                double qwe = tgvalid.DotCross(quad2.Normale(ptvalid), quad1.Normale(ptvalid));
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
                                alig = new ALine(curvsol, false, trans1, trans2);
                                kept = true;
                            }
                            else
                            {
                                ptvalid = curvsol.Value(para);
                                alig = new ALine(curvsol, false);
                                kept = true;
                                //-- std::cout << "Transition indeterminee" << std::endl;
                            }
                            if (kept)
                            {
                                bool nfirstp = !firstp;
                                bool nlastp = !lastp;
                                ProcessBounds(alig,
                                              slin,
                                              quad1,
                                              quad2,
                                              ref nfirstp,
                                              ptf,
                                              first,
                                              ref nlastp,
                                              ptl,
                                              last,
                                              ref multpoint,
                                              tol);
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
    private static bool ExploreCurve(in Cone co,
                                     ref Curve crv,
                                     double tol,
                                     NCollection_List<Curve> lc)
    {
        double sqTol = tol * tol;
        Point3D papx = co.Apex;

        crv.Domain(out double t1, out double t2);

        lc.Clear();
        //
        NCollection_List<double> lParams = new();
        crv.FindParameter(papx, lParams);
        if (lParams.IsEmpty())
        {
            lc.Append(crv);
            return false;
        }

        for (NCollection_List<double>.Iterator itr = new(lParams); itr.More(); itr.Next())
        {
            double prm = itr.Value();

            if ((prm - t1) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                continue;
            }

            bool isLast = false;
            if ((t2 - prm) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                prm = t2;
                isLast = true;
            }

            Point3D p = crv.Value(prm);
            double sqD = p.SquareDistanceTo(papx);
            if (sqD < sqTol)
            {
                Curve c1 = crv;
                c1.SetDomain(t1, prm);
                t1 = prm;
                lc.Append(c1);
            }

            if (isLast)
            {
                break;
            }
        }

        if (lc.IsEmpty())
        {
            lc.Append(crv);
            return false;
        }

        if ((t2 - t1) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            Curve c1 = crv;
            c1.SetDomain(t1, t2);
            lc.Append(c1);
        }

        return true;
    }

    //=================================================================================================

    private static bool IntCoCo(in SurfaceQuadric quad1,
                                in SurfaceQuadric quad2,
                                double tol,
                                ref bool empty,
                                ref bool same,
                                ref bool multpoint,
                                NCollection_Sequence<PatchLine> slin,
                                NCollection_Sequence<PatchPoint> spnt)
    {
        int i, nbSol;
        double u1 = 0, v1 = 0, u2 = 0, v2 = 0;
        WalkingTypeTrans trans1, trans2;
        ResultType typint;
        //
        Cone co1 = quad1.Cone();
        Cone co2 = quad2.Cone();
        //
        QuadQuadGeo inter = new(co1, co2, tol);
        if (!inter.IsDone)
        {
            return false;
        }
        //
        typint = inter.TypeInter();
        nbSol = inter.SolutionCount;
        empty = false;
        same = false;

        switch (typint)
        {
            case global::Kerf.Engine.Intersection.ResultType.Empty:
            {
                empty = true;
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Same:
            {
                same = true;
            }
            break;

            // modified by NIZNHY-PKV Wed Nov 30 12:56:06 2005f
            case global::Kerf.Engine.Intersection.ResultType.Line:
            {
                double para, dot;
                Point3D pApex1, pApex2, ptbid;
                Line3D linsol;
                Vector3D normC1, normC2;
                PatchPoint aPtsol = new();
                GLine glig;
                //
                pApex1 = co1.Apex;
                pApex2 = co2.Apex;
                //
                if (nbSol == 1)
                {
                    WalkingSituation situC1, situC2;
                    //
                    linsol = inter.Line(1);
                    para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, pApex1);
                    ptbid = global::Kerf.Engine.Geometry.ElCLib.Value(para + 5.0, linsol);
                    quad1.Parameters(pApex1, ref u1, ref v1);
                    quad2.Parameters(pApex1, ref u2, ref v2);
                    //
                    aPtsol.SetValue(pApex1, tol, false);
                    aPtsol.SetParameters(u1, v1, u2, v2);
                    aPtsol.SetParameter(para);
                    //
                    normC1 = quad1.Normale(ptbid);
                    normC2 = quad2.Normale(ptbid);
                    dot = normC1.Dot(normC2);
                    if (dot < 0.0)
                    {
                        situC1 = WalkingSituation.Outside;
                        situC2 = WalkingSituation.Outside;
                    }
                    else
                    {
                        double r1, r2;
                        Line3D lAx1 = new(pApex1, co1.Axis.Direction);
                        Line3D lAx2 = new(pApex2, co2.Axis.Direction);
                        //
                        r1 = lAx1.DistanceTo(ptbid);
                        r2 = lAx2.DistanceTo(ptbid);
                        //
                        situC1 = WalkingSituation.Inside;
                        situC2 = WalkingSituation.Outside;
                        if (r1 > r2)
                        {
                            // Intersection line parametrizes from Apex1 to Apex2,
                            situC1 = WalkingSituation.Outside; // So the distance between ptbid and aLAx1 is greater than the
                            situC2 = WalkingSituation.Inside;  // distance between ptbid and aLAx2 and in that case Cone2
                                                      // is inside Cone 1
                        }
                    }
                    // 1
                    glig = new GLine(linsol, true, situC1, situC2);
                    glig.AddVertex(aPtsol);
                    glig.SetFirstPoint(1);
                    slin.Append(glig);
                    // 2
                    linsol = linsol.WithDirection(linsol.Direction.Reversed());
                    para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, pApex1);
                    aPtsol.SetParameter(para);

                    glig = new GLine(linsol, true, situC2, situC1);
                    glig.AddVertex(aPtsol);
                    glig.SetFirstPoint(1);
                    slin.Append(glig);
                } // if (NbSol==1) {
                //////////////////////
                else if (nbSol == 2)
                {
                    //
                    for (i = 1; i <= 2; ++i)
                    {
                        linsol = inter.Line(i);
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, pApex1);
                        ptbid = global::Kerf.Engine.Geometry.ElCLib.Value(para + 5.0, linsol);
                        quad1.Parameters(pApex1, ref u1, ref v1);
                        quad2.Parameters(pApex1, ref u2, ref v2);
                        //
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                        // gp_Dir::DotCross: the normals are converted to (normalised) directions, as in C++.
                        if (linsol.Direction.DotCross(new Direction3D(quad2.Normale(ptbid)), new Direction3D(quad1.Normale(ptbid))) > 0.0)
                        {
                            trans1 = WalkingTypeTrans.Out;
                            trans2 = WalkingTypeTrans.In;
                        }
                        //
                        multpoint = true;
                        // 1,3
                        aPtsol.SetValue(pApex1, tol, false);
                        aPtsol.SetParameters(u1, v1, u2, v2);
                        aPtsol.SetParameter(para);
                        aPtsol.SetMultiple(true);

                        glig = new GLine(linsol, false, trans1, trans2);
                        glig.AddVertex(aPtsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);
                        // 2,4
                        linsol = linsol.WithDirection(linsol.Direction.Reversed());
                        para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, pApex1);
                        aPtsol.SetParameter(para);
                        glig = new GLine(linsol, false, trans1, trans2);
                        glig.AddVertex(aPtsol);
                        glig.SetFirstPoint(1);
                        slin.Append(glig);
                        //
                    } // for (i=1; i<=2; ++i)
                } // else if (NbSol==2)
            }
            break;
            // modified by NIZNHY-PKV Wed Nov 30 12:56:10 2005t

            case global::Kerf.Engine.Intersection.ResultType.Point:
            {
                Point3D ptcontact;
                Point3D apex1 = co1.Apex;
                Point3D apex2 = co2.Apex;
                double param1, param2;
                double paramapex1 = global::Kerf.Engine.Geometry.ElCLib.LineParameter(co1.Axis, apex1);
                double paramapex2 = global::Kerf.Engine.Geometry.ElCLib.LineParameter(co2.Axis, apex2);
                for (i = 1; i <= nbSol; i++)
                {
                    ptcontact = inter.Point(i);
                    param1 = global::Kerf.Engine.Geometry.ElCLib.LineParameter(co1.Axis, ptcontact);
                    param2 = global::Kerf.Engine.Geometry.ElCLib.LineParameter(co2.Axis, ptcontact);

                    quad1.Parameters(ptcontact, ref u1, ref v1);
                    quad2.Parameters(ptcontact, ref u2, ref v2);

                    if (apex1.DistanceTo(ptcontact) <= tol && apex2.DistanceTo(ptcontact) <= tol)
                    {
                        PatchPoint ptsol = new();
                        ptsol.SetValue(ptcontact, tol, false);
                        ptsol.SetParameters(u1, v1, u2, v2);
                        spnt.Append(ptsol);
                    }
                    else if (param1 >= paramapex1 && param2 >= paramapex2)
                    {
                        PatchPoint ptsol = new();
                        ptsol.SetValue(ptcontact, tol, true);
                        ptsol.SetParameters(u1, v1, u2, v2);
                        spnt.Append(ptsol);
                    }
                }
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Circle:
            {
                PatchPoint aPtsol = new();
                Vector3D tgt;
                Point3D ptref;
                for (i = 1; i <= nbSol; i++)
                {
                    Circle3D cirsol = inter.Circle(i);
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, cirsol, out ptref, out tgt);
                    double qwe = tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref));
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
                    GLine glig = new(cirsol, false, trans1, trans2);
                    if (inter.HasCommonGen)
                    {
                        Point3D pChar = inter.PChar();
                        quad1.Parameters(pChar, ref u1, ref v1);
                        quad2.Parameters(pChar, ref u2, ref v2);
                        aPtsol.SetValue(pChar, tol, false);
                        aPtsol.SetParameters(u1, v1, u2, v2);
                        aPtsol.SetParameter(0.0);
                        glig.AddVertex(aPtsol);
                    }
                    slin.Append(glig);
                }
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Ellipse:
            {
                PatchPoint aPtsol = new();
                Ellipse3D elipsol = inter.Ellipse(1);

                global::Kerf.Engine.Geometry.ElCLib.D1(0.0, elipsol, out Point3D ptref, out Vector3D tgt);

                double qwe = tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref));
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
                GLine glig = new(elipsol, false, trans1, trans2);
                if (inter.HasCommonGen)
                {
                    Point3D pChar = inter.PChar();
                    quad1.Parameters(pChar, ref u1, ref v1);
                    quad2.Parameters(pChar, ref u2, ref v2);
                    aPtsol.SetValue(pChar, tol, false);
                    aPtsol.SetParameters(u1, v1, u2, v2);
                    aPtsol.SetParameter(0.0);
                    glig.AddVertex(aPtsol);
                }
                slin.Append(glig);
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Hyperbola:
            {
                PatchPoint aPtsol = new();
                Vector3D tgt;
                Point3D ptref;
                for (i = 1; i <= 2; i++)
                {
                    Hyperbola3D hyprsol = inter.Hyperbola(i);
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, hyprsol, out ptref, out tgt);
                    double qwe = tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref));
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
                    if (inter.HasCommonGen)
                    {
                        Point3D pChar = inter.PChar();
                        quad1.Parameters(pChar, ref u1, ref v1);
                        quad2.Parameters(pChar, ref u2, ref v2);
                        aPtsol.SetValue(pChar, tol, false);
                        aPtsol.SetParameters(u1, v1, u2, v2);
                        aPtsol.SetParameter(0.0);
                        glig.AddVertex(aPtsol);
                    }
                    slin.Append(glig);
                }
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Parabola:
            {
                PatchPoint aPtsol = new();
                Parabola3D parabsol = inter.Parabola(1);

                Vector3D tgtorig = new(parabsol.YAxis.Direction);
                double ptran =
                    tgtorig.DotCross(quad2.Normale(parabsol.Location), quad1.Normale(parabsol.Location));
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
                if (inter.HasCommonGen)
                {
                    Point3D pChar = inter.PChar();
                    quad1.Parameters(pChar, ref u1, ref v1);
                    quad2.Parameters(pChar, ref u2, ref v2);
                    aPtsol.SetValue(pChar, tol, false);
                    aPtsol.SetParameters(u1, v1, u2, v2);
                    aPtsol.SetParameter(0.0);
                    glig.AddVertex(aPtsol);
                }
                slin.Append(glig);
            }
            break;

            case NoGeometricSolution:
            {
                Point3D psol;
                IntQuadQuad anaint = new(co1, new Quadric(co2), tol);
                if (!anaint.IsDone)
                {
                    return false;
                }

                if (anaint.PntCount == 0 && anaint.CurveCount == 0)
                {
                    empty = true;
                }
                else
                {
                    nbSol = anaint.PntCount;
                    for (i = 1; i <= nbSol; i++)
                    {
                        psol = anaint.Point(i);
                        quad1.Parameters(psol, ref u1, ref v1);
                        quad2.Parameters(psol, ref u2, ref v2);
                        PatchPoint ptsol = new();
                        ptsol.SetValue(psol, tol, true);
                        ptsol.SetParameters(u1, v1, u2, v2);
                        spnt.Append(ptsol);
                    }

                    // ptf and ptl keep their values from the previous curve where the bound is open, as OCCT's.
                    // OCCT: kept across the curves; an open bound passes the previous curve's point
                    Point3D ptvalid = new(), ptf = new(), ptl = new();
                    Vector3D tgvalid = new();

                    double first, last, para;
                    bool tgfound, firstp, lastp, kept;
                    int kount;

                    nbSol = anaint.CurveCount;
                    for (i = 1; i <= nbSol; i++)
                    {
                        ALine alig;
                        kept = false;
                        Curve curvsol = anaint.Curve(i);
                        curvsol.Domain(out first, out last);
                        firstp = !curvsol.IsFirstOpen;
                        lastp = !curvsol.IsLastOpen;
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
                            if (tgvalid.SquareMagnitude < 1e-14)
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
                            double qwe = tgvalid.DotCross(quad2.Normale(ptvalid), quad1.Normale(ptvalid));
                            if (qwe > 0.000000001)
                            {
                                trans1 = WalkingTypeTrans.Out;
                                trans2 = WalkingTypeTrans.In;
                            }
                            else if (qwe < -0.000000001)
                            {
                                trans1 = WalkingTypeTrans.In;
                                trans2 = WalkingTypeTrans.Out;
                            }
                            else
                            {
                                trans1 = trans2 = WalkingTypeTrans.Undecided;
                            }
                            alig = new ALine(curvsol, false, trans1, trans2);
                            kept = true;
                        }
                        else
                        {
                            ptvalid = curvsol.Value(para);
                            alig = new ALine(curvsol, false);
                            kept = true;
                            //-- cout << "Transition indeterminee" << endl;
                        }
                        if (kept)
                        {
                            bool nfirstp = !firstp;
                            bool nlastp = !lastp;
                            ProcessBounds(alig,
                                          slin,
                                          quad1,
                                          quad2,
                                          ref nfirstp,
                                          ptf,
                                          first,
                                          ref nlastp,
                                          ptl,
                                          last,
                                          ref multpoint,
                                          tol);
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
        if (inter.HasCommonGen)
        {
            double para;
            PatchPoint aPtsol = new();
            Point3D pApex1, pApex2;
            pApex1 = co1.Apex;
            pApex2 = co2.Apex;
            // common generatrix of cones
            MakeLin mkLin = new(pApex1, pApex2);
            Line3D linsol = mkLin.Value();
            GLine glig = new(linsol, true, WalkingTypeTrans.Undecided, WalkingTypeTrans.Undecided);

            Point3D pChar = inter.PChar();
            quad1.Parameters(pChar, ref u1, ref v1);
            quad2.Parameters(pChar, ref u2, ref v2);
            aPtsol.SetValue(pChar, tol, false);
            aPtsol.SetParameters(u1, v1, u2, v2);
            para = global::Kerf.Engine.Geometry.ElCLib.Parameter(linsol, pChar);
            aPtsol.SetParameter(para);
            glig.AddVertex(aPtsol);

            slin.Append(glig);
        }

        return true;
    }

    //=================================================================================================

    private static bool IntCoSp(in SurfaceQuadric quad1,
                                in SurfaceQuadric quad2,
                                double tol,
                                bool reversed,
                                ref bool empty,
                                ref bool multpoint,
                                NCollection_Sequence<PatchLine> slin,
                                NCollection_Sequence<PatchPoint> spnt)
    {
        int i;

        WalkingTypeTrans trans1, trans2;
        ResultType typint;

        Sphere sp;
        Cone co;
        double u1 = 0, v1 = 0, u2 = 0, v2 = 0;

        if (!reversed)
        {
            co = quad1.Cone();
            sp = quad2.Sphere();
        }
        else
        {
            co = quad2.Cone();
            sp = quad1.Sphere();
        }
        QuadQuadGeo inter = new(sp, co, tol);

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

            case global::Kerf.Engine.Intersection.ResultType.Point:
            {
                Point3D ptcontact;
                Point3D apex = co.Apex;
                double param;
                double paramapex = global::Kerf.Engine.Geometry.ElCLib.LineParameter(co.Axis, apex);
                for (i = 1; i <= nbSol; i++)
                {
                    ptcontact = inter.Point(i);
                    param = global::Kerf.Engine.Geometry.ElCLib.LineParameter(co.Axis, ptcontact);
                    quad1.Parameters(ptcontact, ref u1, ref v1);
                    quad2.Parameters(ptcontact, ref u2, ref v2);

                    if (apex.DistanceTo(ptcontact) <= tol)
                    {
                        PatchPoint ptsol = new();
                        ptsol.SetValue(ptcontact, tol, false);
                        ptsol.SetParameters(u1, v1, u2, v2);
                        spnt.Append(ptsol);
                    }
                    else if (param >= paramapex)
                    {
                        PatchPoint ptsol = new();
                        ptsol.SetValue(ptcontact, tol, true);
                        ptsol.SetParameters(u1, v1, u2, v2);
                        spnt.Append(ptsol);
                    }
                }
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Circle:
            {
                Vector3D tgt;
                Point3D ptref;

                for (i = 1; i <= nbSol; i++)
                {
                    Circle3D cirsol = inter.Circle(i);
                    //-- param = ElCLib::LineParameter(Co.Axis(),
                    //-- 			      cirsol.Location());
                    //-- if (param >= paramapex) {

                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, cirsol, out ptref, out tgt);
                    double qwe = tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref));
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
                    GLine glig = new(cirsol, false, trans1, trans2);
                    slin.Append(glig);
                    //-- }
                }
            }
            break;

            case PointAndCircle:
            {
                Point3D apex = co.Apex;
                double param;
                double paramapex = global::Kerf.Engine.Geometry.ElCLib.LineParameter(co.Axis, apex);

                // le point est necessairement l apex
                quad1.Parameters(apex, ref u1, ref v1);
                quad2.Parameters(apex, ref u2, ref v2);
                PatchPoint ptsol = new();
                ptsol.SetValue(apex, tol, false);
                ptsol.SetParameters(u1, v1, u2, v2);
                spnt.Append(ptsol);

                Circle3D cirsol = inter.Circle(1);
                param = global::Kerf.Engine.Geometry.ElCLib.LineParameter(co.Axis, cirsol.Location);
                global::Kerf.Engine.Geometry.ElCLib.D1(0.0, cirsol, out Point3D ptref, out Vector3D tgt);
                double qwe = tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref));

                if (param >= paramapex)
                {
                    if (qwe > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        trans1 = WalkingTypeTrans.Out;
                        trans2 = WalkingTypeTrans.In;
                    }
                    else if (qwe < -global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                    }
                    else
                    {
                        trans1 = trans2 = WalkingTypeTrans.Undecided;
                    }
                }
                else
                {
                    if (qwe < -global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        trans1 = WalkingTypeTrans.Out;
                        trans2 = WalkingTypeTrans.In;
                    }
                    else if (qwe > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        trans1 = WalkingTypeTrans.In;
                        trans2 = WalkingTypeTrans.Out;
                    }
                    else
                    {
                        trans1 = trans2 = WalkingTypeTrans.Undecided;
                    }
                }
                GLine glig = new(cirsol, false, trans1, trans2);
                slin.Append(glig);
            }
            break;

            case NoGeometricSolution:
            {
                Point3D psol;
                IntQuadQuad anaint = new(co, new Quadric(sp), tol);
                if (!anaint.IsDone)
                {
                    return false;
                }

                if (anaint.PntCount == 0 && anaint.CurveCount == 0)
                {
                    empty = true;
                }
                else
                {
                    nbSol = anaint.PntCount;
                    for (i = 1; i <= nbSol; i++)
                    {
                        psol = anaint.Point(i);
                        quad1.Parameters(psol, ref u1, ref v1);
                        quad2.Parameters(psol, ref u2, ref v2);
                        PatchPoint ptsol = new();
                        ptsol.SetValue(psol, tol, true);
                        ptsol.SetParameters(u1, v1, u2, v2);
                        spnt.Append(ptsol);
                    }

                    // ptf and ptl keep their values from the previous curve where the bound is open, as OCCT's.
                    // OCCT: kept across the curves; an open bound passes the previous curve's point
                    Point3D ptvalid = new(), ptf = new(), ptl = new();
                    Vector3D tgvalid = new();
                    double first, last, para;
                    bool tgfound, firstp, lastp, kept;
                    int kount;

                    nbSol = anaint.CurveCount;
                    for (i = 1; i <= nbSol; i++)
                    {
                        ALine alig;
                        kept = false;
                        Curve curvsol = anaint.Curve(i);
                        curvsol.Domain(out first, out last);
                        firstp = !curvsol.IsFirstOpen;
                        lastp = !curvsol.IsLastOpen;
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
                            para = global::Kerf.Engine.Geometry.ElCLib.LineParameter(co.Axis, ptvalid);
                            double qwe = tgvalid.DotCross(quad2.Normale(ptvalid), quad1.Normale(ptvalid));
                            if (qwe > 0.000000001)
                            {
                                trans1 = WalkingTypeTrans.Out;
                                trans2 = WalkingTypeTrans.In;
                            }
                            else if (qwe < -0.000000001)
                            {
                                trans1 = WalkingTypeTrans.In;
                                trans2 = WalkingTypeTrans.Out;
                            }
                            else
                            {
                                trans1 = trans2 = WalkingTypeTrans.Undecided;
                            }
                            alig = new ALine(curvsol, false, trans1, trans2);
                            kept = true;
                        }
                        else
                        {
                            ptvalid = curvsol.Value(para);
                            para = global::Kerf.Engine.Geometry.ElCLib.LineParameter(co.Axis, ptvalid);
                            alig = new ALine(curvsol, false);
                            kept = true;
                            //-- cout << "Transition indeterminee" << endl;
                        }
                        if (kept)
                        {
                            bool nfirstp = !firstp;
                            bool nlastp = !lastp;
                            ProcessBounds(alig,
                                          slin,
                                          quad1,
                                          quad2,
                                          ref nfirstp,
                                          ptf,
                                          first,
                                          ref nlastp,
                                          ptl,
                                          last,
                                          ref multpoint,
                                          tol);
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
    private static bool IntSpSp(in SurfaceQuadric quad1,
                                in SurfaceQuadric quad2,
                                double tol,
                                ref bool empty,
                                ref bool same,
                                NCollection_Sequence<PatchLine> slin,
                                NCollection_Sequence<PatchPoint> spnt)
    {
        WalkingTypeTrans trans1, trans2;
        ResultType typint;
        Sphere sph1 = quad1.Sphere();
        Sphere sph2 = quad2.Sphere();

        QuadQuadGeo inter = new(sph1, sph2, tol);
        if (!inter.IsDone)
        {
            return false;
        }

        typint = inter.TypeInter();
        empty = false;
        same = false;

        switch (typint)
        {
            case global::Kerf.Engine.Intersection.ResultType.Empty:
            {
                empty = true;
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Same:
            {
                same = true;
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Point:
            {
                Point3D psol = inter.Point(1);
                double u1 = 0, v1 = 0, u2 = 0, v2 = 0;
                quad1.Parameters(psol, ref u1, ref v1);
                quad2.Parameters(psol, ref u2, ref v2);
                PatchPoint ptsol = new();
                ptsol.SetValue(psol, tol, true);
                ptsol.SetParameters(u1, v1, u2, v2);
                spnt.Append(ptsol);
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Circle:
            {
                Circle3D cirsol = inter.Circle(1);
                global::Kerf.Engine.Geometry.ElCLib.D1(0.0, cirsol, out Point3D ptref, out Vector3D tgt);

                double qwe = tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref));
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

    //=================================================================================================

    private static bool IntCyTo(in SurfaceQuadric quad1,
                                in SurfaceQuadric quad2,
                                double tolTang,
                                bool bReversed,
                                ref bool bEmpty,
                                NCollection_Sequence<PatchLine> seqLin)
    {
        Cylinder cyl = bReversed ? quad2.Cylinder() : quad1.Cylinder();
        Torus torus = bReversed ? quad1.Torus() : quad2.Torus();
        //
        QuadQuadGeo anInt = new(cyl, torus, tolTang);
        bool bRet = TreatResultTorus(quad1, quad2, anInt, ref bEmpty, seqLin);
        //
        return bRet;
    }

    //=================================================================================================

    private static bool IntCoTo(in SurfaceQuadric quad1,
                                in SurfaceQuadric quad2,
                                double tolTang,
                                bool bReversed,
                                ref bool bEmpty,
                                NCollection_Sequence<PatchLine> seqLin)
    {
        Cone cone = bReversed ? quad2.Cone() : quad1.Cone();
        Torus torus = bReversed ? quad1.Torus() : quad2.Torus();
        //
        QuadQuadGeo anInt = new(cone, torus, tolTang);
        bool bRet = TreatResultTorus(quad1, quad2, anInt, ref bEmpty, seqLin);
        //
        return bRet;
    }

    //=================================================================================================

    private static bool IntSpTo(in SurfaceQuadric quad1,
                                in SurfaceQuadric quad2,
                                double tolTang,
                                bool bReversed,
                                ref bool bEmpty,
                                NCollection_Sequence<PatchLine> seqLin)
    {
        Sphere sphere = bReversed ? quad2.Sphere() : quad1.Sphere();
        Torus torus = bReversed ? quad1.Torus() : quad2.Torus();
        //
        QuadQuadGeo anInt = new(sphere, torus, tolTang);
        bool bRet = TreatResultTorus(quad1, quad2, anInt, ref bEmpty, seqLin);
        //
        return bRet;
    }

    //=================================================================================================

    private static bool IntToTo(in SurfaceQuadric quad1,
                                in SurfaceQuadric quad2,
                                double tolTang,
                                ref bool bSameSurf,
                                ref bool bEmpty,
                                NCollection_Sequence<PatchLine> seqLin)
    {
        Torus torus1 = quad1.Torus();
        Torus torus2 = quad2.Torus();
        //
        QuadQuadGeo anInt = new(torus1, torus2, tolTang);
        bool bRet = anInt.IsDone;
        if (bRet)
        {
            if (anInt.TypeInter() == Same)
            {
                bEmpty = false;
                bSameSurf = true;
            }
            else
            {
                bRet = TreatResultTorus(quad1, quad2, anInt, ref bEmpty, seqLin);
            }
        }
        //
        return bRet;
    }

    //=================================================================================================

    private static bool TreatResultTorus(in SurfaceQuadric quad1,
                                         in SurfaceQuadric quad2,
                                         QuadQuadGeo anInt,
                                         ref bool bEmpty,
                                         NCollection_Sequence<PatchLine> seqLin)
    {
        bool bRet = anInt.IsDone;
        //
        if (!bRet)
        {
            return bRet;
        }
        //
        ResultType typint = anInt.TypeInter();
        int nbSol = anInt.SolutionCount;
        bEmpty = false;
        //
        switch (typint)
        {
            case Empty:
                bEmpty = true;
                break;
            //
            case global::Kerf.Engine.Intersection.ResultType.Circle:
            {
                int i;
                WalkingTypeTrans trans1, trans2;
                Vector3D tgt;
                Point3D ptref;
                //
                for (i = 1; i <= nbSol; ++i)
                {
                    Circle3D c = anInt.Circle(i);
                    if (quad1.TypeQuadric() == quad2.TypeQuadric())
                    {
                        AdjustToSeam(quad1.Torus(), ref c);
                    }
                    global::Kerf.Engine.Geometry.ElCLib.D1(0.0, c, out ptref, out tgt);
                    double qwe = tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref));
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
}
