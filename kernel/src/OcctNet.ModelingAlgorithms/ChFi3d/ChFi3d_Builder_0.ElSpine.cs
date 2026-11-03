// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_PerformElSpine, ChFi3d_ApproxByC2,
//          ChFi3d_IsSmooth)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Approximation.ParametrizationType;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Occt;

internal static partial class ChFi3d_Builder_0
{
    //=================================================================================================

    /// <summary>
    /// Builds the curve of the elementary spine <paramref name="HES"/>: the spine's edges over its range concatenated
    /// into a B-spline (each smoothed to C2 when it is not), reparametrized by the spine's abscissa, extended at its
    /// ends, made C2 or C3 (<paramref name="continuity"/>) and periodic with the spine.
    /// </summary>
    /// <remarks>
    /// Reads the spine's edges (the input's or the offset ones). <c>BRepLib::BuildCurve3d</c> is called on each: it
    /// writes into an edge only when the edge has no 3D curve (decision 1087). The curves made here are this
    /// function's own; the concatenation's result is taken unfrozen, as OCCT changes it in place.
    /// </remarks>
    internal static void ChFi3d_PerformElSpine(ChFiDS_ElSpine HES, ChFiDS_Spine Spine, Continuity continuity,
                                               double tol, bool IsOffset = false)
    {
        bool periodic, Bof, checkdeb, cepadur, bIsSmooth;
        int IEdge, IF, IL, nbed, iToApproxByC2;
        double WF, WL, Wrefdeb, Wreffin, nwf, nwl, period, pared = 0.0, tolpared;
        double First, Last, epsV, urefdeb, tolrac;
        Continuity aContinuity;
        Point3D PDeb, PFin, Bout;
        Vector3D VrefDeb, VrefFin;
        Curve3D Cv;
        BoundedCurve3D TC;
        BSplineCurve3D BS, BSpline;
        Edge E, Eold;
        Vertex V = new();
        //
        ChFiDS_ElSpine ES = HES;
        WF = ES.FirstParameter();
        WL = ES.LastParameter();
        Wrefdeb = WF;
        Wreffin = WL;
        nwf = WF;
        nwl = WL;
        nbed = Spine.NbEdges();
        periodic = Spine.IsPeriodic();
        if (periodic)
        {
            period = Spine.Period();
            nwf = global::Kerf.Engine.Geometry.ElCLib.InPeriod(WF, -tol, period - tol);
            IF = Spine.Index(nwf, true);
            nwl = global::Kerf.Engine.Geometry.ElCLib.InPeriod(WL, tol, period + tol);
            IL = Spine.Index(nwl, false);
            if (nwl < nwf + tol)
            {
                IL += nbed;
            }
        }
        else
        {
            IF = Spine.Index(WF, true);
            IL = Spine.Index(WL, false);
            Wrefdeb = Math.Max(Spine.FirstParameter(IF), WF);
            Wreffin = Math.Min(Spine.LastParameter(IL), WL);
        }
        //
        Spine.D1(WF, out PDeb, out VrefDeb);
        Spine.D1(WL, out PFin, out VrefFin);
        VrefDeb.Normalize();
        VrefFin.Normalize();
        //
        // (OCCT declares ExtrapPole, ExtraCoeffs and Cont here and never uses them.)
        // Attention on segmente eventuellement la premiere et la
        // derniere arete.
        // Traitment de la premiere arete
        cepadur = false;
        E = (IsOffset) ? Spine.OffsetEdges(IF) : Spine.Edges(IF);
        Bof = BRepLib.BuildCurve3d(E);
        EdgeCurveAdaptor edc = Spine.CurrentElementarySpine(IF);
        tolpared = edc.Resolution(tol);
        Cv = Tool.Curve(E, out First, out Last)!;
        // Add vertex with tangent
        if (ES.IsPeriodic)
        {
            double ParForElSpine = (E.Orientation() == Forward) ? First : Last;
            Cv.D1(ParForElSpine, out Point3D PntForElSpine, out Vector3D DirForElSpine);
            ES.AddVertexWithTangent(new Axis3D(PntForElSpine, new Direction3D(DirForElSpine)));
        }
        /////////////////////////
        urefdeb = Spine.FirstParameter(IF);
        checkdeb = (nwf > urefdeb);
        if (checkdeb)
        {
            Spine.Parameter(IF, nwf, out pared, false);
        }
        //
        if (E.Orientation() == Reversed)
        {
            double sov = First;
            First = Cv.ReversedParameter(Last);
            Last = Cv.ReversedParameter(sov);
            if (checkdeb)
            {
                pared = Cv.ReversedParameter(pared);
            }
            else
            {
                pared = First;
            }
            if (First < pared)
            {
                First = pared;
            }
            if (IL == IF)
            {
                double ureffin = Spine.LastParameter(IL);
                bool checkfin = (nwl < ureffin);
                if (checkfin)
                {
                    Spine.Parameter(IL, nwl, out pared, false);
                    pared = Cv.ReversedParameter(pared);
                }
                else
                {
                    pared = Last;
                }
                if (pared < Last)
                {
                    Last = pared;
                }
            }
            Cv = Cv.Reversed();
        } // if(E.Orientation() == TopAbs_REVERSED)
        else
        { // #1
            if (!checkdeb)
            {
                pared = First;
            }
            if (First < pared)
            {
                First = pared;
            }
            if (IL == IF)
            {
                double ureffin = Spine.LastParameter(IL);
                bool checkfin = (nwl < ureffin);
                if (checkfin)
                {
                    Spine.Parameter(IL, nwl, out pared, false);
                }
                else
                {
                    pared = Last;
                }
                if (pared < Last)
                {
                    Last = pared;
                }
            }
        } // else {//#1
        //
        if (Math.Abs(Last - First) < tolpared)
        {
            cepadur = true;
        }
        //
        // Petite veru pour les cas ou un KPart a bouffe l arete
        // sans parvenir a terminer. On tire une droite.
        if (cepadur)
        {
            LineCurve3D L;
            Point3D ptemp;
            Vector3D vtemp;
            if (WL < Spine.FirstParameter(1) + tol)
            {
                ES.LastPointAndTgt(out ptemp, out vtemp);
                Direction3D d = new(vtemp);
                Point3D olin = new();
                olin.ChangeCoord().SetLinearForm(-WL, d.XYZ(), PFin.XYZ());
                L = new LineCurve3D(olin, d);
                ES.SetCurve(L);
            }
            else if (WF > Spine.LastParameter(nbed) - tol)
            {
                ES.FirstPointAndTgt(out ptemp, out vtemp);
                Direction3D d = new(vtemp);
                Point3D olin = new();
                olin.ChangeCoord().SetLinearForm(-WF, d.XYZ(), PDeb.XYZ());
                L = new LineCurve3D(olin, d);
                ES.SetCurve(L);
            }
            return; // =>
        }
        //
        TC = new TrimmedCurve3D(Cv, First, Last);
        BS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(TC);
        CurveCleaner(BS, Math.Abs(WL - WF) * 1.0e-4, 0);
        //
        // Smoothing of the curve
        iToApproxByC2 = 0;
        aContinuity = TC.Continuity();
        bIsSmooth = ChFi3d_IsSmooth(TC);
        if (aContinuity < C2 && !bIsSmooth)
        {
            ++iToApproxByC2;
            BS = ChFi3d_ApproxByC2(TC);
            TC = BS;
        }
        //
        //  Concatenation des aretes suivantes
        CompCurveToBSplineCurve Concat = new(TC, QuasiAngular);
        //
        Eold = E;
        for (IEdge = IF + 1; IEdge <= IL; ++IEdge)
        {
            int iloc = IEdge;
            if (periodic)
            {
                iloc = (IEdge - 1) % nbed + 1;
            }
            //
            E = (IsOffset) ? Spine.OffsetEdges(iloc) : Spine.Edges(iloc);
            if (Tool.Degenerated(E))
            {
                continue;
            }
            //
            epsV = tol;
            Bof = Exploration.CommonVertex(Eold, E, ref V);
            if (Bof)
            {
                epsV = Tool.Tolerance(V);
            }
            //
            Bof = BRepLib.BuildCurve3d(E);
            if (!Bof)
            {
                throw new ConstructionErrorException("PerformElSpine : BuildCurve3d error");
            }
            //
            Cv = Tool.Curve(E, out First, out Last)!;
            // Add vertex with tangent
            double ParForElSpine = (E.Orientation() == Forward) ? First : Last;
            Cv.D1(ParForElSpine, out Point3D PntForElSpine, out Vector3D DirForElSpine);
            ES.AddVertexWithTangent(new Axis3D(PntForElSpine, new Direction3D(DirForElSpine)));
            /////////////////////////
            if (IEdge == IL)
            {
                double ureffin = Spine.LastParameter(iloc);
                bool checkfin = (nwl < ureffin);
                if (checkfin)
                {
                    Spine.Parameter(iloc, nwl, out pared, false);
                }
                else
                {
                    pared = Last;
                }
                if (E.Orientation() == Reversed)
                {
                    double sov = First;
                    First = Cv.ReversedParameter(Last);
                    Last = Cv.ReversedParameter(sov);
                    if (checkfin)
                    {
                        pared = Cv.ReversedParameter(pared);
                    }
                    else
                    {
                        pared = Last;
                    }
                    Cv = Cv.Reversed();
                }
                if (pared < Last)
                {
                    Last = pared;
                }
            }
            //
            TC = new TrimmedCurve3D(Cv, First, Last);
            BS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(TC);
            CurveCleaner(BS, Math.Abs(WL - WF) * 1.0e-4, 0);
            //
            // Smoothing of the curve
            aContinuity = TC.Continuity();
            bIsSmooth = ChFi3d_IsSmooth(TC);
            if (aContinuity < C2 && !bIsSmooth)
            {
                ++iToApproxByC2;
                BS = ChFi3d_ApproxByC2(TC);
                TC = BS;
            }
            //
            tolrac = Math.Min(tol, epsV);
            Bof = Concat.Add(TC, 2.0 * tolrac, true);
            // si l'ajout ne s'est pas bien passe on essai d'augmenter la tolerance
            if (!Bof)
            {
                Bof = Concat.Add(TC, 2.0 * epsV, true);
            }
            if (!Bof)
            {
                Bof = Concat.Add(TC, 200.0 * epsV, true);
                if (!Bof)
                {
                    throw new ConstructionErrorException("PerformElSpine: spine merged error");
                }
            }
            Eold = E;
        } // for (IEdge=IF+1; IEdge<=IL; ++IEdge) {
        //
        // On a la portion d elspine calculee sans prolongements sur la partie
        // valide des aretes du chemin.
        // OCCT.NET: OCCT changes the concatenation's curve in place (SetKnots, Segment...) and drops the
        // concatenation; the curve is taken unfrozen (decision 1088).
        BSpline = Concat.InternalBSplineCurve()!;
        // There is a reparametrisation to maximally connect the abscissas of edges.
        NCollection_Array1<double> BSNoeuds = new(BSpline.InternalKnots());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(Wrefdeb, Wreffin, BSNoeuds);
        BSpline.SetKnots(BSNoeuds);
        //
        // Traitement des Extremites
        int caredeb, carefin;
        double LocalWL, LocalWF, Angle;
        CurveAdaptor gacurve = new();
        BSplineCurve3D? newc = null;
        //
        caredeb = 0;
        carefin = 0;
        Angle = Math.PI * 0.75;
        LocalWL = WL;
        LocalWF = WF;
        if (!ES.IsPeriodic && !PDeb.IsEqual(BSpline.Pole(1), tol))
        {
            // Prolongement C3 au debut
            // afin d'eviter des pts d'inflexions dans la partie utile de la
            // spine le prolongement se fait jusqu'a un point eloigne.
            if (BSpline.IsRational)
            {
                caredeb = 1;
            }
            //
            double rabdist = Wrefdeb - WF;
            Bout = PDeb.Translated(VrefDeb.Multiplied(-20 * rabdist));
            bool goodext = false;
            for (int icont = 3; icont >= 1 && !goodext; icont--)
            {
                BoundedCurve3D anExtCurve = BSpline;
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref anExtCurve, Bout, icont, false);
                newc = (BSplineCurve3D)anExtCurve;
                gacurve.Load(newc);
                AbscissaPoint GCP = new(gacurve, -rabdist, Wrefdeb, WF);
                if (GCP.IsDone)
                {
                    WF = GCP.Parameter();
                    goodext = GoodExt(newc, VrefDeb, Wrefdeb, WF, Angle);
                }
            }
            if (caredeb != 0)
            {
                caredeb = newc!.KnotCount - BSpline.KnotCount;
            }
            BSpline = newc!;
            LocalWF = BSpline.FirstParameter();
        }
        //
        if (!ES.IsPeriodic && !PFin.IsEqual(BSpline.Pole(BSpline.PoleCount), tol))
        {
            // Prolongement C3 en fin
            if (BSpline.IsRational)
            {
                carefin = 1;
            }
            double rabdist = WL - Wreffin;
            Bout = PFin.Translated(VrefFin.Multiplied(20 * rabdist));
            bool goodext = false;
            for (int icont = 3; icont >= 1 && !goodext; icont--)
            {
                BoundedCurve3D anExtCurve = BSpline;
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref anExtCurve, Bout, icont, true);
                newc = (BSplineCurve3D)anExtCurve;
                gacurve.Load(newc);
                AbscissaPoint GCP = new(gacurve, rabdist, Wreffin, WL);
                if (GCP.IsDone)
                {
                    WL = GCP.Parameter();
                    goodext = GoodExt(newc, VrefFin, Wreffin, WL, Angle);
                }
            }
            if (carefin != 0)
            {
                carefin = newc!.KnotCount - BSpline.KnotCount;
            }
            BSpline = newc!;
            LocalWL = BSpline.LastParameter();
        }
        //
        // Reparametrisation et segmentation sur le domaine de la Spine.
        if (Math.Abs(BSpline.FirstParameter() - WF) < tol)
        {
            WF = BSpline.FirstParameter();
        }
        if (Math.Abs(BSpline.LastParameter() - WL) < tol)
        {
            WL = BSpline.LastParameter();
        }
        //
        if ((LocalWF < WF) || (LocalWL > WL))
        { // pour eviter des pb avec segment!
            BSpline.Segment(WF, WL);
            ES.FirstParameter(WF);
            ES.LastParameter(WL);
        }
        //
        if (BSpline.IsRational)
        {
            BSplineCurve3D C1;
            C1 = (BSplineCurve3D)BSpline.Copy();
            global::Kerf.Engine.Geometry.Conversion.GeomConvert.C0BSplineToC1BSplineCurve(ref C1, tol, 0.1);
            // Il faut s'assurer que l'origine n'a pas bouge (cts21158)
            if (C1.FirstParameter() == BSpline.FirstParameter())
            {
                BSpline = C1;
            }
            else
            {
                // std::cout << "Attention : Echec de C0BSplineToC1 !" << std::endl;
            }
        }
        //
        int fk, lk, MultMax, ii;
        // Deformation eventuelle pour rendre la spine C2.
        // ou C3 pour des approx C2
        if ((caredeb != 0 || carefin != 0) && BSpline.Degree() < 8)
        {
            BSpline.IncreaseDegree(8);
        }
        //
        fk = 2;
        lk = BSpline.KnotCount - 1;
        if (BSpline.IsPeriodic)
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
            if (BSpline.Degree() < 7)
            {
                BSpline.IncreaseDegree(7);
            }
            MultMax = BSpline.Degree() - 3;
        }
        else
        {
            if (BSpline.Degree() < 5)
            {
                BSpline.IncreaseDegree(5);
            }
            MultMax = BSpline.Degree() - 2;
        }
        // correction C2 or C3 (if possible)
        CurveCleaner(BSpline, Math.Abs(WL - WF) * 1.0e-4, 1);
        CurveCleaner(BSpline, Math.Abs(WL - WF) * 1.0e-2, MultMax);
        int MultMin = Math.Max(BSpline.Degree() - 4, 1);
        for (ii = fk; ii <= lk; ii++)
        {
            if (BSpline.Multiplicity(ii) > MultMax)
            {
                Bof = BSpline.RemoveKnot(ii, MultMax, Math.Abs(WL - WF) / 10);
            }
            // See C4
            if (BSpline.Multiplicity(ii) > MultMin)
            {
                Bof = BSpline.RemoveKnot(ii, MultMin, Math.Abs(WL - WF) * 1.0e-4);
            }
        }
        // elspine periodic => BSpline Periodic
        if (ES.IsPeriodic)
        {
            if (!BSpline.IsPeriodic)
            {
                BSpline.SetPeriodic();
                // modified by NIZNHY-PKV Fri Dec 10 12:20:22 2010ft
                if (iToApproxByC2 != 0)
                {
                    Bof = BSpline.RemoveKnot(1, MultMax, Math.Abs(WL - WF) / 10);
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
            Point3D P1, P2;
            Vector3D V1, V2;
            BSpline.D1(WF, out P1, out V1);
            V1.Normalize();
            ES.FirstPointAndTgt(out PDeb, out VrefDeb);
            double scaldeb = VrefDeb.Dot(V1);
            double disdeb = PDeb.Distance(P1);
            if ((Math.Abs(WF - LocalWF) < 1.0e-12) && ((scaldeb <= 0.9999999) || disdeb >= tol))
            {
                // Yes if there was no extension and the tangent is not the good one.
                adjust = true;
            }
            BSpline.D1(WL, out P2, out V2);
            V2.Normalize();
            ES.LastPointAndTgt(out PFin, out VrefFin);
            double scalfin = VrefFin.Dot(V2);
            double disfin = PFin.Distance(P2);
            if ((Math.Abs(WL - LocalWL) < 1.0e-12) && ((scalfin <= 0.9999999) || disfin >= tol))
            {
                // the same at the end
                adjust = true;
            }
            if (adjust)
            {
                BoundedCurve3D anExtCurve = BSpline;
                global::Kerf.Engine.Geometry.Library.GeomLib.AdjustExtremity(ref anExtCurve, PDeb, PFin, VrefDeb, VrefFin);
                BSpline = (BSplineCurve3D)anExtCurve;
            }
        }

        // Le Resultat
        ES.SetCurve(BSpline);

        // Temporary
        // gp_Pnt ptgui;
        // gp_Vec d1gui;
        //( HES->Curve() ).D1(HES->FirstParameter(),ptgui,d1gui);
    }

    //=================================================================================================

    /// <summary>A C2 B-spline through 101 points of <paramref name="C"/>.</summary>
    internal static BSplineCurve3D ChFi3d_ApproxByC2(Curve3D C)
    {
        double First = C.FirstParameter(), Last = C.LastParameter();
        int NbPoints = 101;

        NCollection_Array1<Point3D> Points = new(1, NbPoints);
        double delta = (Last - First) / (NbPoints - 1);
        for (int i = 1; i <= NbPoints - 1; i++)
        {
            Points[i] = C.Value(First + (i - 1) * delta);
        }
        Points[NbPoints] = C.Value(Last);

        GeomAPI_PointsToBSpline Approx = new(Points, ChordLength, 3, 8, C2, 1.000001e-3);
        BSplineCurve3D BS = Approx.Curve();
        return BS;
    }

    //=================================================================================================

    /// <summary>Whether the centre of curvature of <paramref name="C"/> moves smoothly (no turn over 60 degrees, no
    /// jump of the radius over a factor 2).</summary>
    internal static bool ChFi3d_IsSmooth(Curve3D C)
    {
        CurveAdaptor GAC = new(C);

        int ii;
        int intrv, nbintv = GAC.NbIntervals(CN);
        NCollection_Array1<double> TI = new(1, nbintv + 1);
        GAC.Intervals(TI, CN);
        double Resolution = Constants.Resolution(), Curvature;
        CLProps LProp = new(C, 2, Resolution);
        Point3D P1, P2;
        int Discretisation = 30;

        Vector3D PrevVec = new();
        bool prevVecFound = false;
        int intrvFound = 0;
        for (intrv = 1; intrv <= nbintv; intrv++)
        {
            double t = TI[intrv];
            double step = (TI[intrv + 1] - t) / Discretisation;
            for (ii = 1; ii <= Discretisation; ii++)
            {
                LProp.SetParameter(t);
                if (!LProp.IsTangentDefined())
                {
                    return false;
                }
                Curvature = Math.Abs(LProp.Curvature());
                if (Curvature > Resolution)
                {
                    C.D0(t, out P1);
                    LProp.CentreOfCurvature(out P2);
                    PrevVec = new Vector3D(P1, P2);
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
            double t = TI[intrv];
            double step = (TI[intrv + 1] - t) / Discretisation;
            for (ii = 1; ii <= Discretisation; ii++)
            {
                LProp.SetParameter(t);
                if (!LProp.IsTangentDefined())
                {
                    return false;
                }
                Curvature = Math.Abs(LProp.Curvature());
                if (Curvature > Resolution)
                {
                    C.D0(t, out P1);
                    LProp.CentreOfCurvature(out P2);
                    Vector3D Vec = new(P1, P2);
                    double Angle = PrevVec.Angle(Vec);
                    if (Angle > Math.PI / 3.0)
                    {
                        return false;
                    }
                    double Ratio = Vec.Magnitude() / PrevVec.Magnitude();
                    if (Ratio < 1.0)
                    {
                        Ratio = 1.0 / Ratio;
                    }
                    if (Ratio > 2.0 && (intrv != nbintv || ii != Discretisation))
                    {
                        return false;
                    }
                    PrevVec = Vec;
                }
                t += step;
            }
        }
        return true;
    }
}
