// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_PerformElSpine, ChFi3d_ApproxByC2,
//          ChFi3d_IsSmooth)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Approximation.ParametrizationType;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets;

internal static partial class Builder_0
{
    //=================================================================================================

    /// <summary>
    /// Builds the curve of the elementary spine <paramref name="hes"/>: the spine's edges over its range concatenated
    /// into a B-spline (each smoothed to C2 when it is not), reparametrized by the spine's abscissa, extended at its
    /// ends, made C2 or C3 (<paramref name="continuity"/>) and periodic with the spine.
    /// </summary>
    /// <remarks>
    /// Reads the spine's edges (the input's or the offset ones). <c>BRepLib::BuildCurve3d</c> is called on each: it
    /// writes into an edge only when the edge has no 3D curve (decision 1087). The curves made here are this
    /// function's own; the concatenation's result is taken unfrozen, as OCCT changes it in place.
    /// </remarks>
    internal static void ChFi3d_PerformElSpine(ElSpine hes, Spine spine, Continuity continuity,
                                               double tol, bool isOffset = false)
    {
        bool periodic, bof, checkdeb, cepadur, bIsSmooth;
        int iEdge, IF, il, nbed, iToApproxByC2;
        double wf, wl, wrefdeb, wreffin, nwf, nwl, period, pared = 0.0, tolpared;
        double first, last, epsV, urefdeb, tolrac;
        Continuity aContinuity;
        Point3D pDeb, pFin, bout;
        Vector3D vrefDeb, vrefFin;
        Curve3D cv;
        BoundedCurve3D tc;
        BSplineCurve3D bs, bSpline;
        Edge e, eold;
        Vertex v = new();
        //
        ElSpine es = hes;
        wf = es.FirstParameter();
        wl = es.LastParameter();
        wrefdeb = wf;
        wreffin = wl;
        nwf = wf;
        nwl = wl;
        nbed = spine.EdgeCount;
        periodic = spine.IsPeriodic;
        if (periodic)
        {
            period = spine.Period();
            nwf = global::Kerf.Engine.Geometry.ElCLib.InPeriod(wf, -tol, period - tol);
            IF = spine.Index(nwf, true);
            nwl = global::Kerf.Engine.Geometry.ElCLib.InPeriod(wl, tol, period + tol);
            il = spine.Index(nwl, false);
            if (nwl < nwf + tol)
            {
                il += nbed;
            }
        }
        else
        {
            IF = spine.Index(wf, true);
            il = spine.Index(wl, false);
            wrefdeb = Math.Max(spine.FirstParameter(IF), wf);
            wreffin = Math.Min(spine.LastParameter(il), wl);
        }
        //
        spine.D1(wf, out pDeb, out vrefDeb);
        spine.D1(wl, out pFin, out vrefFin);
        vrefDeb = vrefDeb.Normalized();
        vrefFin = vrefFin.Normalized();
        //
        // (OCCT declares ExtrapPole, ExtraCoeffs and Cont here and never uses them.)
        // Attention on segmente eventuellement la premiere et la
        // derniere arete.
        // Traitment de la premiere arete
        cepadur = false;
        e = (isOffset) ? spine.OffsetEdges(IF) : spine.Edges(IF);
        bof = BrepLibrary.BuildCurve3d(e);
        EdgeCurveAdaptor edc = spine.CurrentElementarySpine(IF);
        tolpared = edc.Resolution(tol);
        cv = Tool.Curve(e, out first, out last)!;
        // Add vertex with tangent
        if (es.IsPeriodic)
        {
            double parForElSpine = (e.Orientation == Forward) ? first : last;
            cv.D1(parForElSpine, out Point3D pntForElSpine, out Vector3D dirForElSpine);
            es.AddVertexWithTangent(new Axis3D(pntForElSpine, new Direction3D(dirForElSpine)));
        }
        /////////////////////////
        urefdeb = spine.FirstParameter(IF);
        checkdeb = (nwf > urefdeb);
        if (checkdeb)
        {
            spine.Parameter(IF, nwf, out pared, false);
        }
        //
        if (e.Orientation == Reversed)
        {
            double sov = first;
            first = cv.ReversedParameter(last);
            last = cv.ReversedParameter(sov);
            if (checkdeb)
            {
                pared = cv.ReversedParameter(pared);
            }
            else
            {
                pared = first;
            }
            if (first < pared)
            {
                first = pared;
            }
            if (il == IF)
            {
                double ureffin = spine.LastParameter(il);
                bool checkfin = (nwl < ureffin);
                if (checkfin)
                {
                    spine.Parameter(il, nwl, out pared, false);
                    pared = cv.ReversedParameter(pared);
                }
                else
                {
                    pared = last;
                }
                if (pared < last)
                {
                    last = pared;
                }
            }
            cv = cv.Reversed();
        } // if(E.Orientation() == TopAbs_REVERSED)
        else
        { // #1
            if (!checkdeb)
            {
                pared = first;
            }
            if (first < pared)
            {
                first = pared;
            }
            if (il == IF)
            {
                double ureffin = spine.LastParameter(il);
                bool checkfin = (nwl < ureffin);
                if (checkfin)
                {
                    spine.Parameter(il, nwl, out pared, false);
                }
                else
                {
                    pared = last;
                }
                if (pared < last)
                {
                    last = pared;
                }
            }
        } // else {//#1
        //
        if (Math.Abs(last - first) < tolpared)
        {
            cepadur = true;
        }
        //
        // Petite veru pour les cas ou un KPart a bouffe l arete
        // sans parvenir a terminer. On tire une droite.
        if (cepadur)
        {
            LineCurve3D l;
            Point3D ptemp;
            Vector3D vtemp;
            if (wl < spine.FirstParameter(1) + tol)
            {
                es.LastPointAndTgt(out ptemp, out vtemp);
                Direction3D d = new(vtemp);
                Xyz olinCoord = default;
                olinCoord.SetLinearForm(-wl, d.XYZ(), pFin.XYZ());
                Point3D olin = new(olinCoord);
                l = new LineCurve3D(olin, d);
                es.SetCurve(l);
            }
            else if (wf > spine.LastParameter(nbed) - tol)
            {
                es.FirstPointAndTgt(out ptemp, out vtemp);
                Direction3D d = new(vtemp);
                Xyz olinCoord = default;
                olinCoord.SetLinearForm(-wf, d.XYZ(), pDeb.XYZ());
                Point3D olin = new(olinCoord);
                l = new LineCurve3D(olin, d);
                es.SetCurve(l);
            }
            return; // =>
        }
        //
        tc = new TrimmedCurve3D(cv, first, last);
        bs = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(tc);
        CurveCleaner(bs, Math.Abs(wl - wf) * 1.0e-4, 0);
        //
        // Smoothing of the curve
        iToApproxByC2 = 0;
        aContinuity = tc.Continuity;
        bIsSmooth = ChFi3d_IsSmooth(tc);
        if (aContinuity < C2 && !bIsSmooth)
        {
            ++iToApproxByC2;
            bs = ChFi3d_ApproxByC2(tc);
            tc = bs;
        }
        //
        //  Concatenation des aretes suivantes
        CompCurveToBSplineCurve concat = new(tc, QuasiAngular);
        //
        eold = e;
        for (iEdge = IF + 1; iEdge <= il; ++iEdge)
        {
            int iloc = iEdge;
            if (periodic)
            {
                iloc = (iEdge - 1) % nbed + 1;
            }
            //
            e = (isOffset) ? spine.OffsetEdges(iloc) : spine.Edges(iloc);
            if (Tool.Degenerated(e))
            {
                continue;
            }
            //
            epsV = tol;
            bof = Exploration.CommonVertex(eold, e, ref v);
            if (bof)
            {
                epsV = Tool.Tolerance(v);
            }
            //
            bof = BrepLibrary.BuildCurve3d(e);
            if (!bof)
            {
                throw new ConstructionErrorException("PerformElSpine : BuildCurve3d error");
            }
            //
            cv = Tool.Curve(e, out first, out last)!;
            // Add vertex with tangent
            double parForElSpine = (e.Orientation == Forward) ? first : last;
            cv.D1(parForElSpine, out Point3D pntForElSpine, out Vector3D dirForElSpine);
            es.AddVertexWithTangent(new Axis3D(pntForElSpine, new Direction3D(dirForElSpine)));
            /////////////////////////
            if (iEdge == il)
            {
                double ureffin = spine.LastParameter(iloc);
                bool checkfin = (nwl < ureffin);
                if (checkfin)
                {
                    spine.Parameter(iloc, nwl, out pared, false);
                }
                else
                {
                    pared = last;
                }
                if (e.Orientation == Reversed)
                {
                    double sov = first;
                    first = cv.ReversedParameter(last);
                    last = cv.ReversedParameter(sov);
                    if (checkfin)
                    {
                        pared = cv.ReversedParameter(pared);
                    }
                    else
                    {
                        pared = last;
                    }
                    cv = cv.Reversed();
                }
                if (pared < last)
                {
                    last = pared;
                }
            }
            //
            tc = new TrimmedCurve3D(cv, first, last);
            bs = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(tc);
            CurveCleaner(bs, Math.Abs(wl - wf) * 1.0e-4, 0);
            //
            // Smoothing of the curve
            aContinuity = tc.Continuity;
            bIsSmooth = ChFi3d_IsSmooth(tc);
            if (aContinuity < C2 && !bIsSmooth)
            {
                ++iToApproxByC2;
                bs = ChFi3d_ApproxByC2(tc);
                tc = bs;
            }
            //
            tolrac = Math.Min(tol, epsV);
            bof = concat.Add(tc, 2.0 * tolrac, true);
            // si l'ajout ne s'est pas bien passe on essai d'augmenter la tolerance
            if (!bof)
            {
                bof = concat.Add(tc, 2.0 * epsV, true);
            }
            if (!bof)
            {
                bof = concat.Add(tc, 200.0 * epsV, true);
                if (!bof)
                {
                    throw new ConstructionErrorException("PerformElSpine: spine merged error");
                }
            }
            eold = e;
        } // for (IEdge=IF+1; IEdge<=IL; ++IEdge) {
        //
        // On a la portion d elspine calculee sans prolongements sur la partie
        // valide des aretes du chemin.
        // OCCT.NET: OCCT changes the concatenation's curve in place (SetKnots, Segment...) and drops the
        // concatenation; the curve is taken unfrozen (decision 1088).
        bSpline = concat.InternalBSplineCurve()!;
        // There is a reparametrisation to maximally connect the abscissas of edges.
        NCollection_Array1<double> bsNoeuds = new(bSpline.InternalKnots());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(wrefdeb, wreffin, bsNoeuds);
        bSpline.SetKnots(bsNoeuds);
        //
        // Traitement des Extremites
        int caredeb, carefin;
        double localWL, localWF, angle;
        CurveAdaptor gacurve = new();
        BSplineCurve3D? newc = null;
        //
        caredeb = 0;
        carefin = 0;
        angle = Math.PI * 0.75;
        localWL = wl;
        localWF = wf;
        if (!es.IsPeriodic && !pDeb.IsEqual(bSpline.Pole(1), tol))
        {
            // Prolongement C3 au debut
            // afin d'eviter des pts d'inflexions dans la partie utile de la
            // spine le prolongement se fait jusqu'a un point eloigne.
            if (bSpline.IsRational)
            {
                caredeb = 1;
            }
            //
            double rabdist = wrefdeb - wf;
            bout = pDeb.Translated(vrefDeb.Multiplied(-20 * rabdist));
            bool goodext = false;
            for (int icont = 3; icont >= 1 && !goodext; icont--)
            {
                BoundedCurve3D extCurve = bSpline;
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref extCurve, bout, icont, false);
                newc = (BSplineCurve3D)extCurve;
                gacurve.Load(newc);
                AbscissaPoint gcp = new(gacurve, -rabdist, wrefdeb, wf);
                if (gcp.IsDone)
                {
                    wf = gcp.Parameter();
                    goodext = GoodExt(newc, vrefDeb, wrefdeb, wf, angle);
                }
            }
            if (caredeb != 0)
            {
                caredeb = newc!.KnotCount - bSpline.KnotCount;
            }
            bSpline = newc!;
            localWF = bSpline.FirstParameter;
        }
        //
        if (!es.IsPeriodic && !pFin.IsEqual(bSpline.Pole(bSpline.PoleCount), tol))
        {
            // Prolongement C3 en fin
            if (bSpline.IsRational)
            {
                carefin = 1;
            }
            double rabdist = wl - wreffin;
            bout = pFin.Translated(vrefFin.Multiplied(20 * rabdist));
            bool goodext = false;
            for (int icont = 3; icont >= 1 && !goodext; icont--)
            {
                BoundedCurve3D extCurve = bSpline;
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref extCurve, bout, icont, true);
                newc = (BSplineCurve3D)extCurve;
                gacurve.Load(newc);
                AbscissaPoint gcp = new(gacurve, rabdist, wreffin, wl);
                if (gcp.IsDone)
                {
                    wl = gcp.Parameter();
                    goodext = GoodExt(newc, vrefFin, wreffin, wl, angle);
                }
            }
            if (carefin != 0)
            {
                carefin = newc!.KnotCount - bSpline.KnotCount;
            }
            bSpline = newc!;
            localWL = bSpline.LastParameter;
        }
        //
        // Reparametrisation et segmentation sur le domaine de la Spine.
        if (Math.Abs(bSpline.FirstParameter - wf) < tol)
        {
            wf = bSpline.FirstParameter;
        }
        if (Math.Abs(bSpline.LastParameter - wl) < tol)
        {
            wl = bSpline.LastParameter;
        }
        //
        if ((localWF < wf) || (localWL > wl))
        { // pour eviter des pb avec segment!
            bSpline.Segment(wf, wl);
            es.FirstParameter(wf);
            es.LastParameter(wl);
        }
        //
        if (bSpline.IsRational)
        {
            BSplineCurve3D c1;
            c1 = (BSplineCurve3D)bSpline.Copy();
            global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToC1BSplineCurve(ref c1, tol, 0.1);
            // Il faut s'assurer que l'origine n'a pas bouge (cts21158)
            if (c1.FirstParameter == bSpline.FirstParameter)
            {
                bSpline = c1;
            }
            else
            {
                // std::cout << "Attention : Echec de C0BSplineToC1 !" << std::endl;
            }
        }
        //
        int fk, lk, multMax, ii;
        // Deformation eventuelle pour rendre la spine C2.
        // ou C3 pour des approx C2
        if ((caredeb != 0 || carefin != 0) && bSpline.Degree < 8)
        {
            bSpline.IncreaseDegree(8);
        }
        //
        fk = 2;
        lk = bSpline.KnotCount - 1;
        if (bSpline.IsPeriodic)
        {
            fk = 1;
        }
        if (caredeb != 0)
        {
            fk += caredeb;
        }
        if (carefin != 0)
        {
            lk -= carefin;
        }
        //
        if (continuity == C3)
        {
            if (bSpline.Degree < 7)
            {
                bSpline.IncreaseDegree(7);
            }
            multMax = bSpline.Degree - 3;
        }
        else
        {
            if (bSpline.Degree < 5)
            {
                bSpline.IncreaseDegree(5);
            }
            multMax = bSpline.Degree - 2;
        }
        // correction C2 or C3 (if possible)
        CurveCleaner(bSpline, Math.Abs(wl - wf) * 1.0e-4, 1);
        CurveCleaner(bSpline, Math.Abs(wl - wf) * 1.0e-2, multMax);
        int multMin = Math.Max(bSpline.Degree - 4, 1);
        for (ii = fk; ii <= lk; ii++)
        {
            if (bSpline.Multiplicity(ii) > multMax)
            {
                bof = bSpline.RemoveKnot(ii, multMax, Math.Abs(wl - wf) / 10);
            }
            // See C4
            if (bSpline.Multiplicity(ii) > multMin)
            {
                bof = bSpline.RemoveKnot(ii, multMin, Math.Abs(wl - wf) * 1.0e-4);
            }
        }
        // elspine periodic => BSpline Periodic
        if (es.IsPeriodic)
        {
            if (!bSpline.IsPeriodic)
            {
                bSpline.SetPeriodic();
                // modified by NIZNHY-PKV Fri Dec 10 12:20:22 2010ft
                if (iToApproxByC2 != 0)
                {
                    bof = bSpline.RemoveKnot(1, multMax, Math.Abs(wl - wf) / 10);
                }
                // Bof = BSpline->RemoveKnot(1, MultMax, std::abs(WL-WF)/10);
                // modified by NIZNHY-PKV Mon Dec 13 14:12:54 2010t
            }
        }
        else
        {
            // Otherwise is it necessary to move the poles to adapt
            // them to new tangents ?
            bool adjust = false;
            Point3D p1, p2;
            Vector3D v1, v2;
            bSpline.D1(wf, out p1, out v1);
            v1 = v1.Normalized();
            es.FirstPointAndTgt(out pDeb, out vrefDeb);
            double scaldeb = vrefDeb.Dot(v1);
            double disdeb = pDeb.DistanceTo(p1);
            if ((Math.Abs(wf - localWF) < 1.0e-12) && ((scaldeb <= 0.9999999) || disdeb >= tol))
            {
                // Yes if there was no extension and the tangent is not the good one.
                adjust = true;
            }
            bSpline.D1(wl, out p2, out v2);
            v2 = v2.Normalized();
            es.LastPointAndTgt(out pFin, out vrefFin);
            double scalfin = vrefFin.Dot(v2);
            double disfin = pFin.DistanceTo(p2);
            if ((Math.Abs(wl - localWL) < 1.0e-12) && ((scalfin <= 0.9999999) || disfin >= tol))
            {
                // the same at the end
                adjust = true;
            }
            if (adjust)
            {
                BoundedCurve3D extCurve = bSpline;
                global::Kerf.Engine.Geometry.Library.GeomLib.AdjustExtremity(ref extCurve, pDeb, pFin, vrefDeb, vrefFin);
                bSpline = (BSplineCurve3D)extCurve;
            }
        }

        // Le Resultat
        es.SetCurve(bSpline);

        // Temporary
        // gp_Pnt ptgui;
        // gp_Vec d1gui;
        //( HES->Curve() ).D1(HES->FirstParameter(),ptgui,d1gui);
    }

    //=================================================================================================

    /// <summary>A C2 B-spline through 101 points of <paramref name="c"/>.</summary>
    internal static BSplineCurve3D ChFi3d_ApproxByC2(Curve3D c)
    {
        double first = c.FirstParameter, last = c.LastParameter;
        int nbPoints = 101;

        NCollection_Array1<Point3D> points = new(1, nbPoints);
        double delta = (last - first) / (nbPoints - 1);
        for (int i = 1; i <= nbPoints - 1; i++)
        {
            points[i] = c.Value(first + (i - 1) * delta);
        }
        points[nbPoints] = c.Value(last);

        PointsToBSpline approx = new(points, ChordLength, 3, 8, C2, 1.000001e-3);
        BSplineCurve3D bs = approx.Curve();
        return bs;
    }

    //=================================================================================================

    /// <summary>Whether the centre of curvature of <paramref name="c"/> moves smoothly (no turn over 60 degrees, no
    /// jump of the radius over a factor 2).</summary>
    internal static bool ChFi3d_IsSmooth(Curve3D c)
    {
        CurveAdaptor gac = new(c);

        int ii;
        int intrv, nbintv = gac.NbIntervals(CN);
        NCollection_Array1<double> ti = new(1, nbintv + 1);
        gac.Intervals(ti, CN);
        double resolution = Constants.Resolution(), curvature;
        CLProps lProp = new(c, 2, resolution);
        Point3D p1, p2;
        int discretisation = 30;

        Vector3D prevVec = new();
        bool prevVecFound = false;
        int intrvFound = 0;
        for (intrv = 1; intrv <= nbintv; intrv++)
        {
            double t = ti[intrv];
            double step = (ti[intrv + 1] - t) / discretisation;
            for (ii = 1; ii <= discretisation; ii++)
            {
                lProp.SetParameter(t);
                if (!lProp.IsTangentDefined())
                {
                    return false;
                }
                curvature = Math.Abs(lProp.Curvature());
                if (curvature > resolution)
                {
                    c.D0(t, out p1);
                    lProp.CentreOfCurvature(out p2);
                    prevVec = new Vector3D(p1, p2);
                    prevVecFound = true;
                    break;
                }
                t += step;
            }
            if (prevVecFound)
            {
                intrvFound = intrv;
                break;
            }
        }

        if (!prevVecFound)
        {
            return true;
        }

        // for (intrv = 1; intrv <= nbintv; intrv++) {
        for (intrv = intrvFound; intrv <= nbintv; intrv++)
        {
            double t = ti[intrv];
            double step = (ti[intrv + 1] - t) / discretisation;
            for (ii = 1; ii <= discretisation; ii++)
            {
                lProp.SetParameter(t);
                if (!lProp.IsTangentDefined())
                {
                    return false;
                }
                curvature = Math.Abs(lProp.Curvature());
                if (curvature > resolution)
                {
                    c.D0(t, out p1);
                    lProp.CentreOfCurvature(out p2);
                    Vector3D vec = new(p1, p2);
                    double angle = prevVec.AngleTo(vec);
                    if (angle > Math.PI / 3.0)
                    {
                        return false;
                    }
                    double ratio = vec.Magnitude / prevVec.Magnitude;
                    if (ratio < 1.0)
                    {
                        ratio = 1.0 / ratio;
                    }
                    if (ratio > 2.0 && (intrv != nbintv || ii != discretisation))
                    {
                        return false;
                    }
                    prevVec = vec;
                }
                t += step;
            }
        }
        return true;
    }
}
