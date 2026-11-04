// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicConic_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// a modifier le cas de 2 points confondus ( Insert a la place d'append ? )

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.Curves.IntConicConic_Tool;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Engine.Intersection.TwoD.Situation;
using static Kerf.Engine.Intersection.TwoD.TypeTrans;
using Interval = Kerf.Engine.Intersection.Curves.IntConicConic_Tool.Interval;

namespace Kerf.Engine.Intersection.Curves;

// The geometric intersections: line/line, line/circle, circle/circle and line/ellipse. OCCT's file-level
// functions are private statics; its unused globals Affichage and AffichageGraph are left out.
internal sealed partial class IntConicConic
{
    // modified by NIZHNY-MKK  Tue Feb 15 10:53:34 2000.BEGIN
    //  #define TOLERANCE_ANGULAIRE 0.00000001
    // the reason is at least to make an accordance between transition and position computation.
    private const double TOLERANCE_ANGULAIRE = 1.0e-15;
    // modified by NIZHNY-MKK  Tue Feb 15 10:53:45 2000.END

    private static readonly double PIsur2 = 0.5 * M_PI;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    private static void ProjectOnC2AndIntersectWithC2Domain(in Circle2D circle1,
                                                            in Circle2D circle2,
                                                            ref PeriodicInterval c1DomainAndRes,
                                                            ref PeriodicInterval domainC2,
                                                            PeriodicInterval[] solutionC1,
                                                            PeriodicInterval[] solutionC2,
                                                            ref int nbSolTotal,
                                                            bool identCircles)
    {
        if (c1DomainAndRes.IsNull)
        {
            return;
        }
        //-------------------------------------------------------------------------
        //--  On cherche l intervalle correspondant sur C2
        //--  Puis on intersecte l intervalle avec le domaine de C2
        //--  Enfin, on cherche l intervalle correspondant sur C1
        //--
        double c2inf = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(circle2.Axis,
                                              global::Kerf.Engine.Geometry.ElCLib.CircleValue(c1DomainAndRes.Binf, circle1.Axis, circle1.Radius));
        double c2sup = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(circle2.Axis,
                                              global::Kerf.Engine.Geometry.ElCLib.CircleValue(c1DomainAndRes.Bsup, circle1.Axis, circle1.Radius));

        PeriodicInterval c2Inter = new(c2inf, c2sup);

        if (!identCircles)
        {
            if (c2Inter.Length() > M_PI)
            {
                c2Inter.Complement();
            }
        }
        else
        {
            if (c2sup <= c2inf)
            {
                c2sup += PIpPI;
            }
            if (c2inf >= PIpPI)
            {
                c2sup -= PIpPI;
                c2inf -= PIpPI;
            }
            c2Inter.Binf = c2inf;
            c2Inter.Bsup = c2sup; //--- Verifier la longueur de l'intervalle sur C2
            c2Inter.Bsup = c2inf + c1DomainAndRes.Bsup - c1DomainAndRes.Binf;
        }

        Span<PeriodicInterval> c2InterAndDomain = [new(), new()];

        for (int i = 0; i < 2; i++)
        {
            c2InterAndDomain[i] = (i == 0) ? domainC2.FirstIntersection(ref c2Inter)
                                           : domainC2.SecondIntersection(ref c2Inter);

            if (!c2InterAndDomain[i].IsNull)
            {
                double c1inf = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(
                    circle1.Axis, global::Kerf.Engine.Geometry.ElCLib.CircleValue(c2InterAndDomain[i].Binf, circle2.Axis, circle2.Radius));
                double c1sup = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(
                    circle1.Axis, global::Kerf.Engine.Geometry.ElCLib.CircleValue(c2InterAndDomain[i].Bsup, circle2.Axis, circle2.Radius));

                solutionC1[nbSolTotal] = new PeriodicInterval(c1inf, c1sup);
                if (!identCircles)
                {
                    if (solutionC1[nbSolTotal].Length() > M_PI)
                    {
                        solutionC1[nbSolTotal].Complement();
                    }
                }
                else
                {
                    if (solutionC1[nbSolTotal].Bsup <= solutionC1[nbSolTotal].Binf)
                    {
                        solutionC1[nbSolTotal].Bsup += PIpPI;
                    }
                    if (solutionC1[nbSolTotal].Binf >= PIpPI)
                    {
                        solutionC1[nbSolTotal].Binf -= PIpPI;
                        solutionC1[nbSolTotal].Bsup -= PIpPI;
                    }
                }
                solutionC2[nbSolTotal] = c2InterAndDomain[i];
                nbSolTotal++;
            }
        }
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    private static void CircleCircleGeometricIntersection(in Circle2D c1,
                                                          in Circle2D c2,
                                                          double tol,
                                                          double tolTang,
                                                          ref PeriodicInterval C1_Res1,
                                                          ref PeriodicInterval C1_Res2,
                                                          out int nbsol)
    {
        double C1_binf1, C1_binf2 = 0, C1_bsup1, C1_bsup2 = 0;
        double dO1O2 = (c1.Location).DistanceTo(c2.Location);
        double r1 = c1.Radius;
        double r2 = c2.Radius;
        double absR1mR2 = Math.Abs(r1 - r2);
        //----------------------------------------------------------------
        if (dO1O2 > (r1 + r2 + tol))
        {
            if (dO1O2 > (r1 + r2 + tolTang))
            {
                nbsol = 0;
                return;
            }
            else
            {
                C1_binf1 = 0.0;
                C1_bsup1 = 0.0;
                nbsol = 1;
            }
        }
        //----------------------------------------------------------------
        else if (dO1O2 <= tol && absR1mR2 <= tol)
        {
            nbsol = 3;
            return;
        }
        else
        {
            //----------------------------------------------------------------
            double r1pR2 = r1 + r2;
            double r1pTol = r1 + tol;
            double r1mTol = r1 - tol;
            //    double R1R1=R1*R1;
            double r2R2 = r2 * r2;
            double r1pTolR1pTol = r1pTol * r1pTol;
            double r1mTolR1mTol = r1mTol * r1mTol;
            double dO1O2dO1O2 = dO1O2 * dO1O2;
            double dAlpha1;
            //--------------------------------------------------------------- Cas
            //-- C2 coupe le cercle C1+ (=C(x1,y1,R1+Tol))
            //--            1 seul segment donne par Inter C2 C1+
            //--
            if (dO1O2 > r1pR2 - tol)
            {
                double dx = (r1pTolR1pTol + dO1O2dO1O2 - r2R2) / (dO1O2 + dO1O2);
                double dy = (r1pTolR1pTol - dx * dx);
                dy = (dy >= 0.0) ? Math.Sqrt(dy) : 0.0;
                dAlpha1 = Math.Atan2(dy, dx);

                C1_binf1 = -dAlpha1;
                C1_bsup1 = dAlpha1;
                nbsol = 1;
            }
            //--------------------------------------------------------------------
            //--           2 segments donnes par Inter C2 avec C1- C1 C1+
            //-- Seul le signe de dx change si dO1O2 < std::max(R1,R2)
            //--
            else if (dO1O2 > absR1mR2 - tol)
            { // -- +
                //------------------- Intersection C2 C1+ --------------------------
                double dx = (r1pTolR1pTol + dO1O2dO1O2 - r2R2) / (dO1O2 + dO1O2);
                double dy = (r1pTolR1pTol - dx * dx);
                dy = (dy >= 0.0) ? Math.Sqrt(dy) : 0.0;

                dAlpha1 = Math.Atan2(dy, dx);
                C1_binf1 = -dAlpha1;
                C1_bsup2 = dAlpha1; //--  |...?     ?...|   Sur C1

                //------------------ Intersection C2 C1- -------------------------
                dx = (r1mTolR1mTol + dO1O2dO1O2 - r2R2) / (dO1O2 + dO1O2);
                dy = (r1mTolR1mTol - dx * dx);
                dy = (dy >= 0.0) ? Math.Sqrt(dy) : 0.0;
                dAlpha1 = Math.Atan2(dy, dx);

                C1_binf2 = dAlpha1;
                C1_bsup1 = -dAlpha1; //--  |...x     x...|   Sur C1
                nbsol = 2;
                //------------------------------
                //-- Les 2 intervalles sont ils
                //-- en fait un seul inter ?
                //--
                if (dy == 0)
                { //-- Les 2 bornes internes sont identiques
                    C1_bsup1 = C1_bsup2;
                    nbsol = 1;
                }
                else
                {
                    if (C1_binf1 > C1_bsup1)
                    {
                        dAlpha1 = C1_binf1;
                        C1_binf1 = C1_bsup1;
                        C1_bsup1 = dAlpha1;
                    }
                    if (C1_binf2 > C1_bsup2)
                    {
                        dAlpha1 = C1_binf2;
                        C1_binf2 = C1_bsup2;
                        C1_bsup2 = dAlpha1;
                    }
                    if (((C1_binf1 <= C1_bsup2) && (C1_binf1 >= C1_binf2))
                        || ((C1_bsup1 <= C1_bsup2) && (C1_bsup1 >= C1_binf2)))
                    {
                        if (C1_binf1 > C1_binf2)
                        {
                            C1_binf1 = C1_binf2;
                        }
                        if (C1_binf1 > C1_bsup2)
                        {
                            C1_binf1 = C1_bsup2;
                        }
                        if (C1_bsup1 < C1_binf2)
                        {
                            C1_bsup1 = C1_binf2;
                        }
                        if (C1_bsup1 < C1_bsup2)
                        {
                            C1_bsup1 = C1_bsup2;
                        }
                        nbsol = 1;
                    }
                }
            }
            //--------------------------------------------------------------
            else
            {
                if ((dO1O2 > absR1mR2 - tolTang) && (absR1mR2 - tolTang) > 0.0)
                {
                    C1_binf1 = 0.0;
                    C1_bsup1 = 0.0;
                    nbsol = 1;
                }
                else
                {
                    nbsol = 0;
                    return;
                }
            }
        }

        //----------------------------------------------------------------
        //-- Mise en forme des resultats :
        //--    Les calculs ont ete fait dans le repere x1,y1, (O1,O2)
        //--    On se ramene au repere propre a C1

        Vector2D axe1 = new(c1.XAxis.Direction);
        Vector2D axeO1O2 = new(c1.Location, c2.Location);

        double dAngle1;
        if (axeO1O2.Magnitude <= Constants.Resolution())
        {
            dAngle1 = axe1.AngleTo(new Vector2D(c2.XAxis.Direction));
        }
        else
        {
            dAngle1 = axe1.AngleTo(axeO1O2);
        }

        if (!c1.IsDirect)
        {
            dAngle1 = -dAngle1;
        }

        C1_binf1 += dAngle1;
        C1_bsup1 += dAngle1;

        //-- par construction aucun des segments ne peut exceder PI
        //-- (permet de ne pas gerer trop de cas differents)

        C1_Res1.SetValues(C1_binf1, C1_bsup1);
        if (C1_Res1.Length() > M_PI)
        {
            C1_Res1.Complement();
        }

        if (nbsol == 2)
        {
            C1_binf2 += dAngle1;
            C1_bsup2 += dAngle1;
            C1_Res2.SetValues(C1_binf2, C1_bsup2);
            if (C1_Res2.Length() > M_PI)
            {
                C1_Res2.Complement();
            }
        }
        else
        {
            C1_Res2.SetNull();
        }
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    private static void ProjectOnLAndIntersectWithLDomain(in Circle2D circle,
                                                          in Line2D line,
                                                          ref PeriodicInterval cDomainAndRes,
                                                          ref Interval lDomain,
                                                          PeriodicInterval[] circleSolution,
                                                          Interval[] lineSolution,
                                                          ref int nbSolTotal,
                                                          in TwoDDomain refLineDomain,
                                                          in TwoDDomain _)
    {
        if (cDomainAndRes.IsNull)
        {
            return;
        }
        //-------------------------------------------------------------------------
        //--  On cherche l intervalle correspondant sur C2
        //--  Puis on intersecte l intervalle avec le domaine de C2
        //--  Enfin, on cherche l intervalle correspondant sur C1
        //--

        double linf = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, global::Kerf.Engine.Geometry.ElCLib.CircleValue(cDomainAndRes.Binf, circle.Axis, circle.Radius));
        double lsup = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, global::Kerf.Engine.Geometry.ElCLib.CircleValue(cDomainAndRes.Bsup, circle.Axis, circle.Radius));

        Interval lInter = new(linf, lsup); //-- Necessairement Borne

        Interval lInterAndDomain = lDomain.IntersectionWithBounded(lInter);

        if (!lInterAndDomain.IsNull)
        {
            double domLinf = (refLineDomain.HasFirstPoint) ? refLineDomain.FirstParameter() : -global::Kerf.Engine.Foundation.Precision.Infinite();
            double domLsup = (refLineDomain.HasLastPoint) ? refLineDomain.LastParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();

            linf = lInterAndDomain.Binf;
            lsup = lInterAndDomain.Bsup;

            if (linf < domLinf)
            {
                linf = domLinf;
            }
            if (lsup < domLinf)
            {
                lsup = domLinf;
            }

            if (linf > domLsup)
            {
                linf = domLsup;
            }
            if (lsup > domLsup)
            {
                lsup = domLsup;
            }

            lInterAndDomain.Binf = linf;
            lInterAndDomain.Bsup = lsup;

            double cinf = cDomainAndRes.Binf;
            double csup = cDomainAndRes.Bsup;
            if (cinf >= csup)
            {
                cinf = cDomainAndRes.Binf;
                csup = cDomainAndRes.Bsup;
            }
            circleSolution[nbSolTotal] = new PeriodicInterval(cinf, csup);
            if (circleSolution[nbSolTotal].Length() > M_PI)
            {
                circleSolution[nbSolTotal].Complement();
            }

            lineSolution[nbSolTotal] = lInterAndDomain;
            nbSolTotal++;
        }
    }

    //=======================================================================
    // function : LineCircleGeometricIntersection
    // purpose  :
    //~~ On cherche des segments d intersection dans le `tuyau`
    //~~   R+Tol   R-Tol  ( Tol est TolConf : Tolerance de confusion d arc)
    //~~ On Cherche un point d intersection a une distance TolTang du cercle.
    //=======================================================================
    private static void LineCircleGeometricIntersection(in Line2D line,
                                                        in Circle2D circle,
                                                        double tol,
                                                        double tolTang,
                                                        ref PeriodicInterval cInt1,
                                                        ref PeriodicInterval cInt2,
                                                        out int nbsol)
    {
        double dO1O2 = line.DistanceTo(circle.Location);
        double r = circle.Radius;
        double rmTol = r - tol;
        double binf1, binf2 = 0, bsup1, bsup2 = 0;

        //----------------------------------------------------------------
        if (dO1O2 > (r + tol))
        { //-- pas d intersection avec le 'tuyau'
            if (dO1O2 > (r + tolTang))
            {
                nbsol = 0;
                return;
            }
            else
            {
                binf1 = 0.0;
                bsup1 = 0.0;
                nbsol = 1;
            }
        }
        else
        {
            //----------------------------------------------------------------
            bool b2Sol;
            double dAlpha1;
            //---------------------------------------------------------------
            //-- Line coupe le cercle Circle+ (=C(x1,y1,R1+Tol))
            b2Sol = false;
            if (r > dO1O2 + tolTang)
            {
                double x2, tol2;
                //
                tol2 = tol * tol;
                x2 = 4.0 * (r * r - dO1O2 * dO1O2);
                if (x2 > tol2)
                {
                    b2Sol = !b2Sol;
                }
            }
            if (dO1O2 > rmTol && !b2Sol)
            {
                // if(dO1O2 > RmTol) {
                double dx = dO1O2;
                double dy = 0.0; //(RpTol*RpTol-dx*dx); //Patch !!!
                dy = (dy >= 0.0) ? Math.Sqrt(dy) : 0.0;
                dAlpha1 = Math.Atan2(dy, dx);

                binf1 = -dAlpha1;
                bsup1 = dAlpha1;
                nbsol = 1;
            }
            //--------------------------------------------------------------------
            //--           2 segments donnes par Inter Line avec Circle-  Circle+
            //--
            else
            {
                //------------------- Intersection Line Circle+ --------------------------
                double dx = dO1O2;
                double dy = r * r - dx * dx; //(RpTol*RpTol-dx*dx); //Patch !!!
                dy = (dy >= 0.0) ? Math.Sqrt(dy) : 0.0;

                dAlpha1 = Math.Atan2(dy, dx);
                binf1 = -dAlpha1;
                bsup2 = dAlpha1; //--  |...?     ?...|   Sur C1

                //------------------ Intersection Line Circle-  -------------------------
                dy = r * r - dx * dx; //(RmTol*RmTol-dx*dx); //Patch !!!
                dy = (dy >= 0.0) ? Math.Sqrt(dy) : 0.0;
                dAlpha1 = Math.Atan2(dy, dx);

                binf2 = dAlpha1;
                bsup1 = -dAlpha1; //--  |...x     x...|   Sur C1

                if ((dAlpha1 * r) < (Max(tol, tolTang)))
                {
                    bsup1 = bsup2;
                    nbsol = 1;
                }
                else
                {
                    nbsol = 2;
                }
            }
        }
        //--------------------------------------------------------------
        //-- Mise en forme des resultats :
        //--    Les calculs ont ete fait dans le repere x1,y1, (O1,O2)
        //--    On se ramene au repere propre a C1

        double dAngle1 = circle.XAxis.Direction.AngleTo(line.Direction);

        double a, b, c, d;
        line.Coefficients(out a, out b, out c);

        d = a * circle.Location.X + b * circle.Location.Y + c;

        if (d > 0.0)
        {
            dAngle1 += PIsur2;
        }
        else
        {
            dAngle1 -= PIsur2;
        }

        if (dAngle1 < 0.0)
        {
            dAngle1 += PIpPI;
        }
        else if (dAngle1 > PIpPI)
        {
            dAngle1 -= PIpPI;
        }

        binf1 += dAngle1;
        bsup1 += dAngle1;

        //-- par construction aucun des segments ne peut exceder PI
        //-- (permet de ne pas gerer trop de cas differents)

        if (!circle.IsDirect)
        {
            double t = binf1;
            binf1 = bsup1;
            bsup1 = t;
            binf1 = -binf1;
            bsup1 = -bsup1;
        }

        cInt1.SetValues(binf1, bsup1);
        if (cInt1.Length() > M_PI)
        {
            cInt1.Complement();
        }

        if (nbsol == 2)
        {
            binf2 += dAngle1;
            bsup2 += dAngle1;

            if (!circle.IsDirect)
            {
                double t = binf2;
                binf2 = bsup2;
                bsup2 = t;
                binf2 = -binf2;
                bsup2 = -bsup2;
            }

            cInt2.SetValues(binf2, bsup2);
            if (cInt2.Length() > M_PI)
            {
                cInt2.Complement();
            }
        }
        //  Modified by Sergey KHROMOV - Thu Oct 26 17:51:05 2000 Begin
        else
        {
            if (cInt1.Bsup > PIpPI && cInt1.Binf < PIpPI)
            {
                nbsol = 2;
                binf2 = cInt1.Binf;
                bsup2 = PIpPI;
                binf1 = 0.0;
                cInt1.SetValues(binf1, cInt1.Bsup - PIpPI);
                if (cInt1.Length() > M_PI)
                {
                    cInt1.Complement();
                }
                cInt2.SetValues(binf2, bsup2);
                if (cInt2.Length() > M_PI)
                {
                    cInt2.Complement();
                }
            }
        }
        //  Modified by Sergey KHROMOV - Thu Oct 26 17:51:13 2000 End
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /// <remarks>OCCT leaves the positions unwritten when the result is empty, hence <c>ref</c>.</remarks>
    private static void DomainIntersection(in TwoDDomain domain,
                                           double u1inf,
                                           double u1sup,
                                           out double res1inf,
                                           out double res1sup,
                                           ref Position2D posInf,
                                           ref Position2D posSup)
    {
        if (domain.HasFirstPoint)
        {
            if (u1sup < (domain.FirstParameter() - domain.FirstTolerance()))
            {
                res1inf = 1;
                res1sup = -1;
                return;
            }
            if (u1inf > (domain.FirstParameter() + domain.FirstTolerance()))
            {
                res1inf = u1inf;
                posInf = Position2D.Middle;
            }
            else
            {
                res1inf = domain.FirstParameter();
                posInf = Position2D.Head;
            }
        }
        else
        {
            res1inf = u1inf;
            posInf = Position2D.Middle;
        }

        if (domain.HasLastPoint)
        {
            if (u1inf > (domain.LastParameter() + domain.LastTolerance()))
            {
                res1inf = 1;
                res1sup = -1;
                return;
            }
            if (u1sup < (domain.LastParameter() - domain.LastTolerance()))
            {
                res1sup = u1sup;
                posSup = Position2D.Middle;
            }
            else
            {
                res1sup = domain.LastParameter();
                posSup = Position2D.End;
            }
        }
        else
        {
            res1sup = u1sup;
            posSup = Position2D.Middle;
        }
        //-- Si un des points est en bout ,
        //-- on s assure que les parametres sont corrects
        if (res1inf > res1sup)
        {
            if (posSup == Position2D.Middle)
            {
                res1sup = res1inf;
            }
            else
            {
                res1inf = res1sup;
            }
        }
        //--- Traitement des cas ou une intersection vraie est dans la tolerance
        //--  d un des bouts (commented out in OCCT)
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /// <remarks>OCCT leaves U1, U2 and SinDemiAngle unwritten for parallel lines, hence <c>ref</c>.</remarks>
    private static void LineLineGeometricIntersection(in Line2D l1,
                                                      in Line2D l2,
                                                      double tol,
                                                      ref double u1,
                                                      ref double u2,
                                                      ref double sinDemiAngle,
                                                      out int nbsol)
    {
        double u1x = l1.Direction.X;
        double u1y = l1.Direction.Y;
        double u2x = l2.Direction.X;
        double u2y = l2.Direction.Y;
        double uo21x = l2.Location.X - l1.Location.X;
        double uo21y = l2.Location.Y - l1.Location.Y;

        double d = u1y * u2x - u1x * u2y;

        // modified by NIZHNY-MKK  Tue Feb 15 10:54:04 2000.BEGIN
        //    if(std::abs(D)<1e-15) { //-- Droites //
        if (Math.Abs(d) < TOLERANCE_ANGULAIRE)
        {
            // modified by NIZHNY-MKK  Tue Feb 15 10:54:11 2000.END
            d = u1y * uo21x - u1x * uo21y;
            nbsol = (Math.Abs(d) <= tol) ? 2 : 0;
        }
        else
        {
            u1 = (uo21y * u2x - uo21x * u2y) / d;
            u2 = (uo21y * u1x - uo21x * u1y) / d;

            //------------------- Calcul du Sin du demi angle  entre L1 et L2
            //----
            if (d < 0.0)
            {
                d = -d;
            }
            if (d > 1.0)
            {
                d = 1.0; //-- Deja vu !
            }
            sinDemiAngle = Math.Sin(0.5 * Math.Asin(d));
            nbsol = 1;
        }
    }

    //----------------------------------------------------------------------
    /// <summary>Intersection between two circles.</summary>
    public void Perform(in Circle2D circle1, in TwoDDomain domainCirc1, in Circle2D _Circle2,
                        in TwoDDomain _DomainCirc2, double tolConf, double tol)
    {
        //-- TRES TRES MAL FAIT    A REPRENDRE UN JOUR ....   (lbr Octobre 98)
        Circle2D circle2 = _Circle2;
        TwoDDomain domainCirc2 = _DomainCirc2;
        bool indirectCircles = false;
        if (circle1.IsDirect != _Circle2.IsDirect)
        {
            indirectCircles = true;
            circle2 = _Circle2.Reversed();
            domainCirc2.SetValues(_DomainCirc2.LastPoint(),
                                  PIpPI - _DomainCirc2.LastParameter(),
                                  _DomainCirc2.LastTolerance(),
                                  _DomainCirc2.FirstPoint(),
                                  PIpPI - _DomainCirc2.FirstParameter(),
                                  _DomainCirc2.FirstTolerance());
            domainCirc2.SetEquivalentParameters(0.0, PIpPI);
        }

        this.ResetFields();
        int nbsol = 0;
        PeriodicInterval C1_Int1 = new(), C1_Int2 = new();

        //------- Intersection sans tenir compte du domaine  ----> nbsol=0,1,2,3
        CircleCircleGeometricIntersection(circle1, circle2, tolConf, tol, ref C1_Int1, ref C1_Int2, out nbsol);
        _done = true;

        if (nbsol == 0)
        { //-- Pas de solutions
            return;
        }

        PeriodicInterval c1Domain = new(domainCirc1);
        //-- On se ramene entre 0 et 2PI
        double deltat = c1Domain.Bsup - c1Domain.Binf;
        if (deltat >= PIpPI)
        {
            // make deltat not including the upper limit
            deltat = Math.BitDecrement(PIpPI);
        }

        while (c1Domain.Binf >= PIpPI)
        {
            c1Domain.Binf -= PIpPI;
        }
        while (c1Domain.Binf < 0.0)
        {
            c1Domain.Binf += PIpPI;
        }

        c1Domain.Bsup = c1Domain.Binf + deltat;

        PeriodicInterval c2Domain = new(domainCirc2);
        deltat = c2Domain.Bsup - c2Domain.Binf;
        if (deltat >= PIpPI)
        {
            deltat = Math.BitDecrement(PIpPI);
        }

        while (c2Domain.Binf >= PIpPI)
        {
            c2Domain.Binf -= PIpPI;
        }
        while (c2Domain.Binf < 0.0)
        {
            c2Domain.Binf += PIpPI;
        }

        c2Domain.Bsup = c2Domain.Binf + deltat;

        bool identCircles = false;

        if (nbsol > 2)
        {
            //-- Les 2 cercles sont confondus a Tol pres
            C1_Int1.SetValues(0, PIpPI);
            C1_Int2.SetNull();
            //---------------------------------------------------------------
            //-- Flag utilise pour specifier que les intervalles manipules
            //--   peuvent etre de longueur superieure a pi.
            //-- Pour des cercles non identiques, on a necessairement cette
            //--   condition sur les resultats de l intersection geometrique
            //--   ce qui permet de normaliser rapidement les intervalles.
            //--   ex: -1 4 -> longueur > PI
            //--        donc -1 4 devient  4 , 2*pi-1
            //---------------------------------------------------------------
            identCircles = true;
        }

        int nbSolTotal = 0;
        PeriodicInterval[] solutionC1 = [new(), new(), new(), new()];
        PeriodicInterval[] solutionC2 = [new(), new(), new(), new()];

        //----------------------------------------------------------------------
        //----------- Traitement du premier intervalle Geometrique  C1_Int1 ----
        //----------------------------------------------------------------------
        //-- NbSolTotal est incremente a chaque Intervalle solution.
        //-- On stocke les intervalles dans les tableaux : SolutionC1(C2)
        //-- Dimensionnes a 4 elements.
        //-- des Exemples faciles donnent 3 Intersections
        //-- des Problemes numeriques peuvent en donner 4 ??????
        //--
        PeriodicInterval c1DomainAndRes = c1Domain.FirstIntersection(ref C1_Int1);

        ProjectOnC2AndIntersectWithC2Domain(circle1, circle2, ref c1DomainAndRes, ref c2Domain, solutionC1, solutionC2,
                                            ref nbSolTotal, identCircles);
        //----------------------------------------------------------------------
        //-- Seconde Intersection :  Par exemple :     2*PI-1  2*PI+1
        //--                         Intersecte avec     0.5   2*PI-0.5
        //--     Donne les intervalles : 0.5,1    et  2*PI-1,2*PI-0.5
        //--
        c1DomainAndRes = c1Domain.SecondIntersection(ref C1_Int1);

        ProjectOnC2AndIntersectWithC2Domain(circle1, circle2, ref c1DomainAndRes, ref c2Domain, solutionC1, solutionC2,
                                            ref nbSolTotal, identCircles);

        //----------------------------------------------------------------------
        //----------- Traitement du second intervalle Geometrique   C1_Int2 ----
        //----------------------------------------------------------------------
        if (nbsol == 2)
        {
            c1DomainAndRes = c1Domain.FirstIntersection(ref C1_Int2);

            ProjectOnC2AndIntersectWithC2Domain(circle1, circle2, ref c1DomainAndRes, ref c2Domain, solutionC1,
                                                solutionC2, ref nbSolTotal, identCircles);
            //--------------------------------------------------------------------
            c1DomainAndRes = c1Domain.SecondIntersection(ref C1_Int2);

            ProjectOnC2AndIntersectWithC2Domain(circle1, circle2, ref c1DomainAndRes, ref c2Domain, solutionC1,
                                                solutionC2, ref nbSolTotal, identCircles);
        }
        //----------------------------------------------------------------------
        //-- Calcul de toutes les transitions et Positions.
        //--
        //----------------------------------------------------------------------
        //-- On determine si des intervalles sont reduit a des points
        //--      ( Rayon * Intervalle.Length()    <    Tol   )
        //--
        double r1 = circle1.Radius;
        double r2 = circle2.Radius;
        double tol2 = tol + tol; //---- Pour eviter de toujours retourner
                                 // des segments
        int i;
        if (tol < (1e-10))
        {
            tol2 = 1e-10;
        }

        for (i = 0; i < nbSolTotal; i++)
        {
            if (((r1 * solutionC1[i].Length()) <= tol2) && ((r2 * solutionC2[i].Length()) <= tol2))
            {
                double t = (solutionC1[i].Binf + solutionC1[i].Bsup) * 0.5;
                solutionC1[i].Binf = solutionC1[i].Bsup = t;

                t = (solutionC2[i].Binf + solutionC2[i].Bsup) * 0.5;
                solutionC2[i].Binf = solutionC2[i].Bsup = t;
            }
        }

        //----------------------------------------------------------------------
        //-- Traitement des intervalles (ou des points obtenus)
        //--
        Frame2D axis2C1 = circle1.Axis;
        Frame2D axis2C2 = circle2.Axis;
        Point2D p1a, p1b, p2a, p2b;
        Vector2D tan1, tan2, norm1, norm2;
        Transition t1a = new(), t1b = new(), t2a = new(), t2b = new();
        Position2D pos1a, pos1b, pos2a, pos2b;

        bool isOpposite = (circle1.Location.SquareDistanceTo(circle2.Location)) > (r1 * r1 + r2 * r2);

        for (i = 0; i < nbSolTotal; i++)
        {
            double c2inf = isOpposite ? solutionC2[i].Bsup : solutionC2[i].Binf;
            double c2sup = isOpposite ? solutionC2[i].Binf : solutionC2[i].Bsup;
            double c1tinf = solutionC1[i].Binf, c2tinf = c2inf;
            double c1inf = NormalizeOnCircleDomain(c1tinf, domainCirc1);
            c2inf = NormalizeOnCircleDomain(c2tinf, domainCirc2);

            bool isOutOfRange = false;
            if (c1inf < domainCirc1.FirstParameter())
            {
                if (c1tinf < domainCirc1.FirstParameter())
                {
                    c1inf = domainCirc1.FirstParameter();
                    isOutOfRange = true;
                }
                else
                {
                    c1inf = c1tinf;
                }
            }

            if (c1inf > domainCirc1.LastParameter())
            {
                if (c1tinf > domainCirc1.LastParameter())
                {
                    c1inf = domainCirc1.LastParameter();
                    isOutOfRange = true;
                }
                else
                {
                    c1inf = c1tinf;
                }
            }

            if (c2inf < domainCirc2.FirstParameter())
            {
                if (c2tinf < domainCirc2.FirstParameter())
                {
                    c2inf = domainCirc2.FirstParameter();
                    isOutOfRange = true;
                }
                else
                {
                    c2inf = c2tinf;
                }
            }

            if (c2inf > domainCirc2.LastParameter())
            {
                if (c2tinf > domainCirc2.LastParameter())
                {
                    c2inf = domainCirc2.LastParameter();
                    isOutOfRange = true;
                }
                else
                {
                    c2inf = c2tinf;
                }
            }

            if (isOutOfRange)
            {
                global::Kerf.Engine.Geometry.ElCLib.CircleD2(c1inf, axis2C1, r1, out Point2D p1, out _, out _);
                global::Kerf.Engine.Geometry.ElCLib.CircleD2(c2inf, axis2C2, r2, out Point2D p2, out _, out _);

                if (p1.SquareDistanceTo(p2) > tol2 * tol2)
                { // there are not any solutions in given parametric range.
                    continue;
                }
            }

            if (indirectCircles)
            {
                global::Kerf.Engine.Geometry.ElCLib.CircleD2(c1inf, axis2C1, r1, out p1a, out tan1, out norm1);
                global::Kerf.Engine.Geometry.ElCLib.CircleD2(c2inf, axis2C2, r2, out p2a, out tan2, out norm2);
                tan2 = tan2.Reversed();

                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1a, domainCirc1, p1a, c1inf);
                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2a, _DomainCirc2, p2a, PIpPI - c2inf);
                Determine_Transition_LC(pos1a, ref tan1, norm1, ref t1a, pos2a, ref tan2, norm2, ref t2a, tol);

                IntersectionPoint2D newPoint1 = new(p1a, c1inf, PIpPI - c2inf, t1a, t2a, false);

                if ((solutionC1[i].Length() > 0.0) || (solutionC2[i].Length() > 0.0))
                {
                    //-- On traite un intervalle non reduit a un point
                    double c1sup = NormalizeOnCircleDomain(solutionC1[i].Bsup, domainCirc1);
                    if (c1sup < c1inf)
                    {
                        c1sup += PIpPI;
                    }
                    c2sup = NormalizeOnCircleDomain(c2sup, domainCirc2);

                    global::Kerf.Engine.Geometry.ElCLib.CircleD2(c1sup, axis2C1, r1, out p1b, out tan1, out norm1);
                    global::Kerf.Engine.Geometry.ElCLib.CircleD2(c2sup, axis2C2, r2, out p2b, out tan2, out norm2);
                    tan2 = tan2.Reversed();

                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1b, domainCirc1, p1b, c1sup);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2b, _DomainCirc2, p2b, PIpPI - c2sup);
                    Determine_Transition_LC(pos1b, ref tan1, norm1, ref t1b, pos2b, ref tan2, norm2, ref t2b, tol);

                    //--------------------------------------------------

                    if (isOpposite)
                    {
                        if (nbsol != 3)
                        {
                            if (c2inf < c2sup)
                            {
                                c2inf += PIpPI;
                            }
                        }
                    }
                    else
                    {
                        if (nbsol != 3)
                        {
                            if (c2sup < c2inf)
                            {
                                c2sup += PIpPI;
                            }
                        }
                    }

                    IntersectionPoint2D newPoint2 = new(p1b, c1sup, PIpPI - c2sup, t1b, t2b, false);
                    IntersectionSegment2D newSeg = new(newPoint1, newPoint2, !isOpposite, false);
                    Append(newSeg);
                }
                else
                {
                    Append(newPoint1);
                }
            }
            else
            {
                global::Kerf.Engine.Geometry.ElCLib.CircleD2(c1inf, axis2C1, r1, out p1a, out tan1, out norm1);
                global::Kerf.Engine.Geometry.ElCLib.CircleD2(c2inf, axis2C2, r2, out p2a, out tan2, out norm2);

                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1a, domainCirc1, p1a, c1inf);
                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2a, domainCirc2, p2a, c2inf);
                Determine_Transition_LC(pos1a, ref tan1, norm1, ref t1a, pos2a, ref tan2, norm2, ref t2a, tol);

                IntersectionPoint2D newPoint1 = new(p1a, c1inf, c2inf, t1a, t2a, false);

                if ((solutionC1[i].Length() > 0.0) || (solutionC2[i].Length() > 0.0))
                {
                    //-- On traite un intervalle non reduit a un point
                    double c1sup = NormalizeOnCircleDomain(solutionC1[i].Bsup, domainCirc1);
                    if (c1sup < c1inf)
                    {
                        c1sup += PIpPI;
                    }
                    c2sup = NormalizeOnCircleDomain(c2sup, domainCirc2);

                    global::Kerf.Engine.Geometry.ElCLib.CircleD2(c1sup, axis2C1, r1, out p1b, out tan1, out norm1);
                    global::Kerf.Engine.Geometry.ElCLib.CircleD2(c2sup, axis2C2, r2, out p2b, out tan2, out norm2);

                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1b, domainCirc1, p1b, c1sup);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2b, domainCirc2, p2b, c2sup);
                    Determine_Transition_LC(pos1b, ref tan1, norm1, ref t1b, pos2b, ref tan2, norm2, ref t2b, tol);

                    //--------------------------------------------------

                    if (isOpposite)
                    {
                        if (c2inf < c2sup)
                        {
                            c2inf += PIpPI;
                        }
                    }
                    else
                    {
                        if (c2sup < c2inf)
                        {
                            c2sup += PIpPI;
                        }
                    }

                    IntersectionPoint2D newPoint2 = new(p1b, c1sup, c2sup, t1b, t2b, false);
                    IntersectionSegment2D newSeg = new(newPoint1, newPoint2, isOpposite, false);
                    Append(newSeg);
                }
                else
                {
                    Append(newPoint1);
                }
            }
        }
    }

    //----------------------------------------------------------------------
    private static Position2D FindPositionLL(ref double param, in TwoDDomain domain)
    {
        double dPar = global::Kerf.Engine.Foundation.Precision.Infinite();
        Position2D pos = Position2D.Middle;
        double resPar = param;
        if (domain.HasFirstPoint)
        {
            dPar = Math.Abs(param - domain.FirstParameter());
            if (dPar <= domain.FirstTolerance())
            {
                resPar = domain.FirstParameter();
                pos = Position2D.Head;
            }
        }
        if (domain.HasLastPoint)
        {
            double d2 = Math.Abs(param - domain.LastParameter());
            if (d2 <= domain.LastTolerance() && (pos == Position2D.Middle || d2 < dPar))
            {
                resPar = domain.LastParameter();
                pos = Position2D.End;
            }
        }
        param = resPar;
        return pos;
    }

    //--------------------------------------------------------------------
    // gka 0022833
    // Method to compute of point of intersection for case
    // when specified domain less than specified tolerance for intersection
    private static void GetDomainParametrs(in TwoDDomain domain,
                                           out double first,
                                           out double last,
                                           out double tol1,
                                           out double tol2)
    {
        first = (domain.HasFirstPoint ? domain.FirstParameter() : -global::Kerf.Engine.Foundation.Precision.Infinite());
        last = (domain.HasLastPoint ? domain.LastParameter() : global::Kerf.Engine.Foundation.Precision.Infinite());
        tol1 = (domain.HasFirstPoint ? domain.FirstTolerance() : 0.0);
        tol2 = (domain.HasLastPoint ? domain.LastTolerance() : 0.0);
    }

    //=================================================================================================

    private static bool ComputeIntPoint(in TwoDDomain curDomain,
                                        in TwoDDomain domainOther,
                                        in Line2D curLin,
                                        in Line2D otherLin,
                                        double cosT1T2,
                                        double parCur,
                                        double parOther,
                                        ref double resInf,
                                        ref double resSup,
                                        int num,
                                        TypeTrans curTrans,
                                        ref IntersectionPoint2D newPoint)
    {
        if (Math.Abs(resSup - parCur) > Math.Abs(resInf - parCur))
        {
            resSup = resInf;
        }

        double res2 = parOther + (resSup - parCur) * cosT1T2;

        double first2, last2, tol21, tol22, tol11, tol12;

        GetDomainParametrs(domainOther, out first2, out last2, out tol21, out tol22);

        if (res2 < first2 - tol21 || res2 > last2 + tol22)
        {
            return false;
        }

        //------ compute parameters of intersection point --
        Transition t1 = new(), t2 = new();
        Position2D pos1a = FindPositionLL(ref resSup, curDomain);
        Position2D pos2a = FindPositionLL(ref res2, domainOther);
        TypeTrans otherTrans =
            (curTrans == TypeTrans.Out ? TypeTrans.In
                                         : (curTrans == TypeTrans.In ? TypeTrans.Out : TypeTrans.Undecided));

        if (curTrans != TypeTrans.Undecided)
        {
            t1.SetValue(false, pos1a, curTrans);
            t2.SetValue(false, pos2a, otherTrans);
        }
        else
        {
            bool opposite = cosT1T2 < 0.0;
            t1.SetValue(false, pos1a, Situation.Unknown, opposite);
            t2.SetValue(false, pos2a, Situation.Unknown, opposite);
        }
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //--------------------------------------------------
        // gka bug 0022833
        double resU1 = parCur;
        double resU2 = parOther;

        double first1, last1;
        GetDomainParametrs(curDomain, out first1, out last1, out tol11, out tol12);

        bool isInside1 = (parCur >= first1 && parCur <= last1);
        bool isInside2 = (parOther >= first2 && parOther <= last2);

        if (!isInside1 || !isInside2)
        {
            if (isInside1)
            {
                Point2D Pt1 = global::Kerf.Engine.Geometry.ElCLib.Value(res2, otherLin);
                resU2 = res2;
                double par1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(curLin, Pt1);
                resU1 = ((par1 >= first1 && par1 <= last1) ? par1 : resSup);
            }
            else if (isInside2)
            {
                Point2D pt1 = global::Kerf.Engine.Geometry.ElCLib.Value(resSup, curLin);
                resU1 = resSup;
                double par2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(otherLin, pt1);
                resU2 = ((par2 >= first2 && par2 <= last2) ? par2 : res2);
            }
            else
            {
                // PKVf
                //  check that parameters are within range on both curves
                if (parCur < first1 - tol11 || parCur > last1 + tol12 || parOther < first2 - tol21
                    || parOther > last2 + tol22)
                {
                    return false;
                }
                // PKVt
                resU1 = resSup;
                resU2 = res2;
            }
        }
        Point2D pres = new((global::Kerf.Engine.Geometry.ElCLib.Value(resU1, curLin).XY() + global::Kerf.Engine.Geometry.ElCLib.Value(resU2, otherLin).XY()) * 0.5);
        if (num == 1)
        {
            newPoint.SetValues(pres, resU1, resU2, t1, t2, false);
        }
        else
        {
            newPoint.SetValues(pres, resU2, resU1, t2, t1, false);
        }
        return true;
    }

    //=======================================================================
    // function : CheckLLCoincidence
    // purpose  : Returns true if input are trimmed curves and they coincide
    //           within tolerance
    //=======================================================================
    private static bool CheckLLCoincidence(in Line2D l1,
                                           in Line2D l2,
                                           in TwoDDomain domain1,
                                           in TwoDDomain domain2,
                                           double tol)
    {
        bool isFirst1 = (domain1.HasFirstPoint && l2.DistanceTo(domain1.FirstPoint()) < tol);
        bool isLast1 = (domain1.HasLastPoint && l2.DistanceTo(domain1.LastPoint()) < tol);
        if (isFirst1 && isLast1)
        {
            return true;
        }
        bool isFirst2 = (domain2.HasFirstPoint && l1.DistanceTo(domain2.FirstPoint()) < tol);
        bool isLast2 = (domain2.HasLastPoint && l1.DistanceTo(domain2.LastPoint()) < tol);
        return isFirst2 && isLast2;
    }

    //----------------------------------------------------------------------
    /// <summary>Intersection between two lines.</summary>
    public void Perform(in Line2D l1, in TwoDDomain domain1, in Line2D l2, in TwoDDomain domain2,
                        double tolConf, double tolR)
    {
        this.ResetFields();

        //-- Coordonnees du point d intersection sur chacune des 2 droites
        double u1 = 0, u2 = 0;
        //-- Nombre de points solution : 1 : Intersection
        //--                             0 : Non Confondues
        //--                             2 : Confondues a la tolerance pres
        int nbsol;
        IntersectionPoint2D ptSeg1 = new(), ptSeg2 = new();
        double halfSinL1L2 = 0;
        double tol = tolR;
        if (tol < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        }

        LineLineGeometricIntersection(l1, l2, tol, ref u1, ref u2, ref halfSinL1L2, out nbsol);

        Vector2D tan1 = new(l1.Direction);
        Vector2D tan2 = new(l2.Direction);

        double cosT1T2 = tan1.Dot(tan2);
        bool isOpposite = cosT1T2 < 0.0;

        _done = true;

        if (nbsol == 1 && CheckLLCoincidence(l1, l2, domain1, domain2, tol))
        {
            nbsol = 2;
        }

        if (nbsol == 1)
        {
            //---------------------------------------------------
            //-- d: distance du point I a partir de laquelle  les
            //--  points de parametre U1+d et U2+-d sont ecartes
            //--  d une distance superieure a Tol.
            //---------------------------------------------------
            // Positions OCCT leaves uninitialised; DomainIntersection writes them unless the result is empty.
            Position2D pos1a = Position2D.Middle, pos2a = Position2D.Middle, pos1b = Position2D.Middle,
                              pos2b = Position2D.Middle;
            double d = 0.5 * tol / halfSinL1L2;
            double u1inf = u1 - d;
            double u1sup = u1 + d;
            double u1mU2 = u1 - u2;
            double u1pU2 = u1 + u2;
            double res1inf, res1sup;
            double prodVectTan;

            //---------------------------------------------------
            //-- On agrandit la zone U1inf U1sup pour tenir compte
            //-- des tolerances des points en bout
            //--
            if (domain1.HasFirstPoint)
            {
                if (l2.DistanceTo(domain1.FirstPoint()) < domain1.FirstTolerance())
                {
                    if (u1inf > domain1.FirstParameter())
                    {
                        u1inf = domain1.FirstParameter();
                    }
                    if (u1sup < domain1.FirstParameter())
                    {
                        u1sup = domain1.FirstParameter();
                    }
                }
            }
            if (domain1.HasLastPoint)
            {
                if (l2.DistanceTo(domain1.LastPoint()) < domain1.LastTolerance())
                {
                    if (u1inf > domain1.LastParameter())
                    {
                        u1inf = domain1.LastParameter();
                    }
                    if (u1sup < domain1.LastParameter())
                    {
                        u1sup = domain1.LastParameter();
                    }
                }
            }
            if (domain2.HasFirstPoint)
            {
                if (l1.DistanceTo(domain2.FirstPoint()) < domain2.FirstTolerance())
                {
                    double p = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, domain2.FirstPoint());
                    if (u1inf > p)
                    {
                        u1inf = p;
                    }
                    if (u1sup < p)
                    {
                        u1sup = p;
                    }
                }
            }
            if (domain2.HasLastPoint)
            {
                if (l1.DistanceTo(domain2.LastPoint()) < domain2.LastTolerance())
                {
                    double p = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, domain2.LastPoint());
                    if (u1inf > p)
                    {
                        u1inf = p;
                    }
                    if (u1sup < p)
                    {
                        u1sup = p;
                    }
                }
            }
            //-----------------------------------------------------------------

            DomainIntersection(domain1, u1inf, u1sup, out res1inf, out res1sup, ref pos1a, ref pos1b);

            if ((res1sup - res1inf) < 0.0)
            {
                //-- Si l intersection est vide
                //--
            }
            else
            { //-- (Domain1  INTER   Zone Intersection)    non vide

                prodVectTan = tan1.Crossed(tan2);

                // #####################################################################
                // ##  Longueur Minimale d un segment    Sur Courbe 1
                // #####################################################################

                double longMiniSeg = tol;

                if (((res1sup - res1inf) <= longMiniSeg) || ((pos1a == pos1b) && (pos1a != Position2D.Middle)))
                {
                    //-------------------------------  Un seul Point -------------------
                    //--- lorsque la longueur du segment est inferieure a ??
                    //--- ou si deux points designent le meme bout
                    // gka #0022833
                    TypeTrans curTrans =
                        (prodVectTan >= TOLERANCE_ANGULAIRE
                             ? TypeTrans.Out
                             : (prodVectTan <= -TOLERANCE_ANGULAIRE ? TypeTrans.In : TypeTrans.Undecided));

                    IntersectionPoint2D newPoint1 = new();
                    if (ComputeIntPoint(domain1, domain2, l1, l2, cosT1T2, u1, u2, ref res1inf, ref res1sup, 1,
                                        curTrans, ref newPoint1))
                    {
                        Append(newPoint1);
                    }

                    //------------------------------------------------------

                } //---------------   Fin du cas  :   1 seul point --------------------

                else
                {
                    //-- Intersection AND Domain1  --------> Segment ---------------------
                    double u2inf, u2sup;
                    double res2inf, res2sup;

                    if (isOpposite)
                    {
                        u2inf = u1pU2 - res1sup;
                        u2sup = u1pU2 - res1inf;
                    }
                    else
                    {
                        u2inf = res1inf - u1mU2;
                        u2sup = res1sup - u1mU2;
                    }

                    DomainIntersection(domain2, u2inf, u2sup, out res2inf, out res2sup, ref pos2a, ref pos2b);

                    // ####################################################################
                    // ##  Test sur la longueur minimale d un segment sur Ligne2
                    // ####################################################################
                    double Res2sup_m_Res2inf = res2sup - res2inf;
                    if (Res2sup_m_Res2inf < 0.0)
                    {
                        //-- Pas de solutions On retourne Vide
                    }
                    else if ((Res2sup_m_Res2inf > longMiniSeg) || ((pos2a == pos2b) && (pos2a != Position2D.Middle)))
                    {
                        //----------- Calcul des attributs du segment --------------
                        //-- Attention, les bornes Res1inf(sup) bougent donc il faut
                        //--  eventuellement recalculer les attributs

                        if (isOpposite)
                        {
                            res1inf = u1pU2 - res2sup;
                            res1sup = u1pU2 - res2inf;
                            double tampon = res2inf;
                            res2inf = res2sup;
                            res2sup = tampon;
                            Position2D pos = pos2a;
                            pos2a = pos2b;
                            pos2b = pos;
                        }
                        else
                        {
                            res1inf = u1mU2 + res2inf;
                            res1sup = u1mU2 + res2sup;
                        }

                        pos1a = FindPositionLL(ref res1inf, domain1);
                        pos1b = FindPositionLL(ref res1sup, domain1);

                        Transition t1a = new(), t2a = new(), t1b = new(), t2b = new();

                        if (prodVectTan >= TOLERANCE_ANGULAIRE)
                        { // &&&&&&&&&&&&&&&
                            t1a.SetValue(false, pos1a, TypeTrans.Out);
                            t2a.SetValue(false, pos2a, TypeTrans.In);
                        }
                        else if (prodVectTan <= -TOLERANCE_ANGULAIRE)
                        {
                            t1a.SetValue(false, pos1a, TypeTrans.In);
                            t2a.SetValue(false, pos2a, TypeTrans.Out);
                        }
                        else
                        {
                            t1a.SetValue(false, pos1a, Situation.Unknown, isOpposite);
                            t2a.SetValue(false, pos2a, Situation.Unknown, isOpposite);
                        }

                        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                        //~~~~~~~  C O N V E N T I O N    -    S E G M E N T     ~~~~~~~
                        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                        //~~ On Renvoie un segment dans les cas suivants :            ~~
                        //~~   (1) Extremite L1 L2   ------>    Extremite L1 L2       ~~
                        //~~   (2) Extremite L1 L2   ------>    Intersection          ~~
                        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

                        bool resultIsAPoint = false;

                        if (((res1sup - res1inf) <= longMiniSeg) || (Math.Abs(res2sup - res2inf) <= longMiniSeg))
                        {
                            //-- On force la creation d un point
                            resultIsAPoint = true;
                        }
                        else
                        {
                            //------------------------------------------------------------
                            //-- On traite les cas ou l intersection est situee du
                            //-- Mauvais cote du domaine
                            //-- Attention : Res2inf <-> Pos2a        Res2sup <-> Pos2b
                            //--  et         Res1inf <-> Pos1a        Res1sup <-> Pos1b
                            //--             avec Res1inf <= Res1sup
                            //------------------------------------------------------------
                            //-- Le point sera : Res1inf,Res2inf,T1a(Pos1a),T2a(Pos2a)
                            //------------------------------------------------------------

                            if (pos1a == Position2D.Head)
                            {
                                if (pos1b != Position2D.End && u1 < res1inf)
                                {
                                    resultIsAPoint = true;
                                    u1 = res1inf;
                                    u2 = res2inf;
                                }
                            }
                            if (pos1b == Position2D.End)
                            {
                                if (pos1a != Position2D.Head && u1 > res1sup)
                                {
                                    resultIsAPoint = true;
                                    u1 = res1sup;
                                    u2 = res2sup;
                                }
                            }

                            if (pos2a == Position2D.Head)
                            {
                                if (pos2b != Position2D.End && u2 < res2inf)
                                {
                                    resultIsAPoint = true;
                                    u2 = res2inf;
                                    u1 = res1inf;
                                }
                            }
                            else
                            {
                                if (pos2a == Position2D.End)
                                {
                                    if (pos2b != Position2D.Head && u2 > res2inf)
                                    {
                                        resultIsAPoint = true;
                                        u2 = res2inf;
                                        u1 = res1inf;
                                    }
                                }
                            }
                            if (pos2b == Position2D.Head)
                            {
                                if (pos2a != Position2D.End && u2 < res2sup)
                                {
                                    resultIsAPoint = true;
                                    u2 = res2sup;
                                    u1 = res1sup;
                                }
                            }
                            else
                            {
                                if (pos2b == Position2D.End)
                                {
                                    if (pos2a != Position2D.Head && u2 > res2sup)
                                    {
                                        resultIsAPoint = true;
                                        u2 = res2sup;
                                        u1 = res1sup;
                                    }
                                }
                            }
                        }

                        if ((!resultIsAPoint) && (pos1a != Position2D.Middle || pos2a != Position2D.Middle))
                        {
                            if (prodVectTan >= TOLERANCE_ANGULAIRE)
                            { //&&&&&&&&&&&&&&
                                t1b.SetValue(false, pos1b, TypeTrans.Out);
                                t2b.SetValue(false, pos2b, TypeTrans.In);
                            }
                            else if (prodVectTan <= -TOLERANCE_ANGULAIRE)
                            { //&&&&&&&&&&&&&&
                                t1b.SetValue(false, pos1b, TypeTrans.In);
                                t2b.SetValue(false, pos2b, TypeTrans.Out);
                            }
                            else
                            {
                                t1b.SetValue(false, pos1b, Situation.Unknown, isOpposite);
                                t2b.SetValue(false, pos2b, Situation.Unknown, isOpposite);
                            }
                            Point2D ptdebut;
                            if (pos1a == Position2D.Middle)
                            {
                                double t3;
                                if (isOpposite)
                                {
                                    t3 = (pos2a == Position2D.Head) ? res2sup : res2inf;
                                }
                                else
                                {
                                    t3 = (pos2a == Position2D.Head) ? res2inf : res2sup;
                                }
                                ptdebut = global::Kerf.Engine.Geometry.ElCLib.Value(t3, l2);
                                res1inf = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, ptdebut);
                            }
                            else
                            {
                                double t4 = (pos1a == Position2D.Head) ? res1inf : res1sup;
                                ptdebut = global::Kerf.Engine.Geometry.ElCLib.Value(t4, l1);
                                res2inf = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, ptdebut);
                            }
                            ptSeg1.SetValues(ptdebut, res1inf, res2inf, t1a, t2a, false);
                            if (pos1b != Position2D.Middle || pos2b != Position2D.Middle)
                            {
                                //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                                //~~ Ajustement des parametres et du point renvoye
                                Point2D ptfin;
                                if (pos1b == Position2D.Middle)
                                {
                                    ptfin = global::Kerf.Engine.Geometry.ElCLib.Value(res2sup, l2);
                                    res1sup = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, ptfin);
                                }
                                else
                                {
                                    ptfin = global::Kerf.Engine.Geometry.ElCLib.Value(res1sup, l1);
                                    res2sup = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, ptfin);
                                }
                                ptSeg2.SetValues(ptfin, res1sup, res2sup, t1b, t2b, false);
                                IntersectionSegment2D segment = new(ptSeg1, ptSeg2, isOpposite, false);
                                Append(segment);
                            }
                            else
                            { //-- Extremite(L1 ou L2)  ------>   Point Middle(L1 et L2)

                                pos1b = FindPositionLL(ref u1, domain1);
                                pos2b = FindPositionLL(ref u2, domain2);
                                if (prodVectTan >= TOLERANCE_ANGULAIRE)
                                {
                                    t1b.SetValue(false, pos1b, TypeTrans.Out);
                                    t2b.SetValue(false, pos2b, TypeTrans.In);
                                }
                                else if (prodVectTan <= -TOLERANCE_ANGULAIRE)
                                {
                                    t1b.SetValue(false, pos1b, TypeTrans.In);
                                    t2b.SetValue(false, pos2b, TypeTrans.Out);
                                }
                                else
                                {
                                    t1b.SetValue(false, pos1b, Situation.Unknown, isOpposite);
                                    t2b.SetValue(false, pos2b, Situation.Unknown, isOpposite);
                                }

                                ptSeg2.SetValues(global::Kerf.Engine.Geometry.ElCLib.Value(u2, l2), u1, u2, t1b, t2b, false);

                                if ((Math.Abs(res1inf - u1) > longMiniSeg) && (Math.Abs(res2inf - u2) > longMiniSeg))
                                {
                                    IntersectionSegment2D segment = new(ptSeg1, ptSeg2, isOpposite, false);
                                    Append(segment);
                                }
                                else
                                {
                                    Append(SegmentToPoint(ptSeg1, t1a, t2a, ptSeg2, t1b, t2b));
                                }
                            }

                        } //-- (Pos1a!=IntRes2d_Middle || Pos2a!=IntRes2d_Middle) --
                        else
                        { //-- Pos1a == Pos2a == Middle
                            if (pos1b == Position2D.Middle)
                            {
                                pos1b = pos1a;
                            }
                            if (pos2b == Position2D.Middle)
                            {
                                pos2b = pos2a;
                            }
                            if (resultIsAPoint)
                            {
                                //-- Middle sur le segment A
                                //--
                                if (pos1b != Position2D.Middle || pos2b != Position2D.Middle)
                                {
                                    Point2D ptfin;
                                    if (pos1b == Position2D.Middle)
                                    {
                                        double t2;
                                        if (isOpposite)
                                        {
                                            t2 = (pos2b == Position2D.Head) ? res2sup : res2inf;
                                        }
                                        else
                                        {
                                            t2 = (pos2b == Position2D.Head) ? res2inf : res2sup;
                                        }
                                        ptfin = global::Kerf.Engine.Geometry.ElCLib.Value(t2, l2);
                                        res1sup = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, ptfin);
                                        // modified by NIZHNY-MKK  Tue Feb 15 10:54:51 2000.BEGIN
                                        pos1b = FindPositionLL(ref res1sup, domain1);
                                        // modified by NIZHNY-MKK  Tue Feb 15 10:54:55 2000.END
                                    }
                                    else
                                    {
                                        double t1 = (pos1b == Position2D.Head) ? res1inf : res1sup;
                                        ptfin = global::Kerf.Engine.Geometry.ElCLib.Value(t1, l1);
                                        res2sup = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, ptfin);
                                        // modified by NIZHNY-MKK  Tue Feb 15 10:55:08 2000.BEGIN
                                        pos2b = FindPositionLL(ref res2sup, domain2);
                                        // modified by NIZHNY-MKK  Tue Feb 15 10:55:11 2000.END
                                    }
                                    if (prodVectTan >= TOLERANCE_ANGULAIRE)
                                    {
                                        t1b.SetValue(false, pos1b, TypeTrans.Out);
                                        t2b.SetValue(false, pos2b, TypeTrans.In);
                                    }
                                    else if (prodVectTan <= -TOLERANCE_ANGULAIRE)
                                    {
                                        t1b.SetValue(false, pos1b, TypeTrans.In);
                                        t2b.SetValue(false, pos2b, TypeTrans.Out);
                                    }
                                    else
                                    {
                                        t1b.SetValue(false, pos1b, Situation.Unknown, isOpposite);
                                        t2b.SetValue(false, pos2b, Situation.Unknown, isOpposite);
                                    }
                                    ptSeg2.SetValues(ptfin, res1sup, res2sup, t1b, t2b, false);
                                    Append(ptSeg2);
                                }
                                else
                                {
                                    pos1b = FindPositionLL(ref u1, domain1);
                                    pos2b = FindPositionLL(ref u2, domain2);

                                    if (prodVectTan >= TOLERANCE_ANGULAIRE)
                                    {
                                        t1b.SetValue(false, pos1b, TypeTrans.Out);
                                        t2b.SetValue(false, pos2b, TypeTrans.In);
                                    }
                                    else if (prodVectTan <= -TOLERANCE_ANGULAIRE)
                                    {
                                        t1b.SetValue(false, pos1b, TypeTrans.In);
                                        t2b.SetValue(false, pos2b, TypeTrans.Out);
                                    }
                                    else
                                    {
                                        t1b.SetValue(false, pos1b, Situation.Unknown, isOpposite);
                                        t2b.SetValue(false, pos2b, Situation.Unknown, isOpposite);
                                    }
                                    ptSeg1.SetValues(global::Kerf.Engine.Geometry.ElCLib.Value(u2, l2), u1, u2, t1b, t2b, false);
                                    Append(ptSeg1);
                                }
                            }
                            else
                            {
                                ptSeg1.SetValues(global::Kerf.Engine.Geometry.ElCLib.Value(u2, l2), u1, u2, t1a, t2a, false);

                                if ((pos1b != Position2D.Middle || pos2b != Position2D.Middle))
                                {
                                    if (prodVectTan >= TOLERANCE_ANGULAIRE)
                                    {
                                        t1b.SetValue(false, pos1b, TypeTrans.Out);
                                        t2b.SetValue(false, pos2b, TypeTrans.In);
                                    }
                                    else if (prodVectTan <= -TOLERANCE_ANGULAIRE)
                                    {
                                        t1b.SetValue(false, pos1b, TypeTrans.In);
                                        t2b.SetValue(false, pos2b, TypeTrans.Out);
                                    }
                                    else
                                    {
                                        t1b.SetValue(false, pos1b, Situation.Unknown, isOpposite);
                                        t2b.SetValue(false, pos2b, Situation.Unknown, isOpposite);
                                    }
                                    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                                    //~~ Ajustement des parametres et du point renvoye
                                    Point2D ptfin;
                                    if (pos1b == Position2D.Middle)
                                    {
                                        ptfin = global::Kerf.Engine.Geometry.ElCLib.Value(res2sup, l2);
                                        res1sup = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, ptfin);
                                    }
                                    else
                                    {
                                        ptfin = global::Kerf.Engine.Geometry.ElCLib.Value(res1sup, l1);
                                        res2sup = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, ptfin);
                                    }

                                    ptSeg2.SetValues(ptfin, res1sup, res2sup, t1b, t2b, false);

                                    if ((Math.Abs(u1 - res1sup) > longMiniSeg) || (Math.Abs(u2 - res2sup) > longMiniSeg))
                                    {
                                        //-- Modif du 1er Octobre 92 (Pour Composites)

                                        IntersectionSegment2D segment = new(ptSeg1, ptSeg2, isOpposite, false);
                                        Append(segment);
                                    }
                                    else
                                    {
                                        Append(SegmentToPoint(ptSeg1, t1a, t2a, ptSeg2, t1b, t2b));
                                    }
                                }
                                else
                                {
                                    Append(ptSeg1);
                                }
                            }
                        }
                    } //----- Fin Creation Segment ----(Res2sup-Res2inf>Tol)-------------
                    else
                    {
                        //------ (Intersection And Domain1)  AND  Domain2  --> Point ------
                        //-- Attention Res1sup peut etre  different de  U2
                        //--   Mais on a Res1sup-Res1inf < Tol

                        // gka #0022833
                        TypeTrans curTrans =
                            (prodVectTan >= TOLERANCE_ANGULAIRE
                                 ? TypeTrans.In
                                 : (prodVectTan <= -TOLERANCE_ANGULAIRE ? TypeTrans.Out : TypeTrans.Undecided));

                        IntersectionPoint2D newPoint1 = new();
                        if (ComputeIntPoint(domain2, domain1, l2, l1, cosT1T2, u2, u1, ref res2inf, ref res2sup, 2,
                                            curTrans, ref newPoint1))
                        {
                            Append(newPoint1);
                        }
                    }
                }
            }
        }
        else
        {
            if (nbsol == 2)
            { //== Droites confondues a la tolerance pres
                //--On traite ici le cas de segments resultats non neccess. bornes
                //--
                //--On prend la droite D1 comme reference ( pour le sens positif )
                //--
                int resHasFirstPoint = 0;
                int resHasLastPoint = 0;
                // ParamStart2 and ParamEnd2 are uninitialised in OCCT where they are not used.
                double paramStart = 0.0, paramStart2 = 0.0, paramEnd = 0.0, paramEnd2 = 0.0;
                double org2SurL1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, l2.Location);
                //== 3 : L1 et L2 bornent
                //== 2 :       L2 borne
                //== 1 : L1 borne
                if (domain1.HasFirstPoint)
                {
                    resHasFirstPoint = 1;
                }
                if (domain1.HasLastPoint)
                {
                    resHasLastPoint = 1;
                }
                if (isOpposite)
                {
                    if (domain2.HasLastPoint)
                    {
                        resHasFirstPoint += 2;
                    }
                    if (domain2.HasFirstPoint)
                    {
                        resHasLastPoint += 2;
                    }
                }
                else
                {
                    if (domain2.HasLastPoint)
                    {
                        resHasLastPoint += 2;
                    }
                    if (domain2.HasFirstPoint)
                    {
                        resHasFirstPoint += 2;
                    }
                }
                if (resHasFirstPoint == 0 && resHasLastPoint == 0)
                {
                    //~~~~ Creation d un segment infini avec Opposite
                    Append(new IntersectionSegment2D(isOpposite));
                }
                else
                { //-- On obtient au pire une demi-droite
                    switch (resHasFirstPoint)
                    {
                        case 1:
                            paramStart = domain1.FirstParameter();
                            paramStart2 = isOpposite ? (org2SurL1 - paramStart) : (paramStart - org2SurL1);
                            break;
                        case 2:
                            if (isOpposite)
                            {
                                paramStart2 = domain2.LastParameter();
                                paramStart = org2SurL1 - paramStart2;
                            }
                            else
                            {
                                paramStart2 = domain2.FirstParameter();
                                paramStart = org2SurL1 + paramStart2;
                            }
                            break;
                        case 3:
                            if (isOpposite)
                            {
                                paramStart2 = domain2.LastParameter();
                                paramStart = org2SurL1 - paramStart2;
                                if (paramStart < domain1.FirstParameter())
                                {
                                    paramStart = domain1.FirstParameter();
                                    paramStart2 = org2SurL1 - paramStart;
                                }
                            }
                            else
                            {
                                paramStart2 = domain2.FirstParameter();
                                paramStart = org2SurL1 + paramStart2;
                                if (paramStart < domain1.FirstParameter())
                                {
                                    paramStart = domain1.FirstParameter();
                                    paramStart2 = paramStart - org2SurL1;
                                }
                            }
                            break;
                        default: //~~~ Segment Infini a gauche
                            break;
                    }

                    switch (resHasLastPoint)
                    {
                        case 1:
                            paramEnd = domain1.LastParameter();
                            paramEnd2 = isOpposite ? (org2SurL1 - paramEnd) : (paramEnd - org2SurL1);
                            break;
                        case 2:
                            if (isOpposite)
                            {
                                paramEnd2 = domain2.FirstParameter();
                                paramEnd = org2SurL1 - paramEnd2;
                            }
                            else
                            {
                                paramEnd2 = domain2.LastParameter();
                                paramEnd = org2SurL1 + paramEnd2;
                            }
                            break;
                        case 3:
                            if (isOpposite)
                            {
                                paramEnd2 = domain2.FirstParameter();
                                paramEnd = org2SurL1 - paramEnd2;
                                if (paramEnd > domain1.LastParameter())
                                {
                                    paramEnd = domain1.LastParameter();
                                    paramEnd2 = org2SurL1 - paramEnd;
                                }
                            }
                            else
                            {
                                paramEnd2 = domain2.LastParameter();
                                paramEnd = org2SurL1 + paramEnd2;
                                if (paramEnd > domain1.LastParameter())
                                {
                                    paramEnd = domain1.LastParameter();
                                    paramEnd2 = paramEnd - org2SurL1;
                                }
                            }
                            break;
                        default: //~~~ Segment Infini a droite
                            break;
                    }

                    Transition tinf = new(), tsup = new();

                    if (resHasFirstPoint != 0)
                    {
                        if (resHasLastPoint != 0)
                        {
                            //~~~ Creation de la borne superieure
                            //~~~ L1 :     |------------->       ou          |-------------->
                            //~~~ L2 : <------------|            ou  <----|
                            if (paramEnd >= (paramStart - tol))
                            {
                                //~~~ Creation d un segment
                                Position2D pos1, pos2;
                                pos1 = FindPositionLL(ref paramStart, domain1);
                                pos2 = FindPositionLL(ref paramStart2, domain2);
                                tinf.SetValue(true, pos1, Situation.Unknown, isOpposite);
                                tsup.SetValue(true, pos2, Situation.Unknown, isOpposite);
                                IntersectionPoint2D p1 = new(global::Kerf.Engine.Geometry.ElCLib.Value(paramStart, l1), paramStart,
                                                                    paramStart2, tinf, tsup, false);
                                if (paramEnd > (paramStart + tol))
                                {
                                    //~~~ Le segment est assez long
                                    pos1 = FindPositionLL(ref paramEnd, domain1);
                                    pos2 = FindPositionLL(ref paramEnd2, domain2);
                                    tinf.SetValue(true, pos1, Situation.Unknown, isOpposite);
                                    tsup.SetValue(true, pos2, Situation.Unknown, isOpposite);

                                    IntersectionPoint2D p2 = new(global::Kerf.Engine.Geometry.ElCLib.Value(paramEnd, l1), paramEnd, paramEnd2,
                                                                        tinf, tsup, false);
                                    IntersectionSegment2D seg = new(p1, p2, isOpposite, false);
                                    Append(seg);
                                }
                                else
                                { //~~~~ le segment est de longueur inferieure a Tol
                                    Append(p1);
                                }
                            } //-- if( ParamEnd >= ...)
                        } //-- if(ResHasLastPoint)
                        else
                        {
                            //~~~ Creation de la demi droite   |----------->
                            Position2D pos1 = FindPositionLL(ref paramStart, domain1);
                            Position2D pos2 = FindPositionLL(ref paramStart2, domain2);
                            tinf.SetValue(true, pos1, Situation.Unknown, isOpposite);
                            tsup.SetValue(true, pos2, Situation.Unknown, isOpposite);

                            IntersectionPoint2D P = new(global::Kerf.Engine.Geometry.ElCLib.Value(paramStart, l1), paramStart, paramStart2,
                                                               tinf, tsup, false);
                            IntersectionSegment2D seg = new(P, true, isOpposite, false);
                            Append(seg);
                        }
                    }
                    else
                    {
                        Position2D pos1 = FindPositionLL(ref paramEnd, domain1);
                        Position2D pos2 = FindPositionLL(ref paramEnd2, domain2);
                        tinf.SetValue(true, pos1, Situation.Unknown, isOpposite);
                        tsup.SetValue(true, pos2, Situation.Unknown, isOpposite);

                        IntersectionPoint2D p2 = new(global::Kerf.Engine.Geometry.ElCLib.Value(paramEnd, l1), paramEnd, paramEnd2, tinf,
                                                            tsup, false);
                        IntersectionSegment2D seg = new(p2, false, isOpposite, false);
                        Append(seg);
                        //~~~ Creation de la demi droite   <-----------|
                    }
                }
            }
        }
    }

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    /// <summary>Intersection between a line and a circle.</summary>
    public void Perform(in Line2D line, in TwoDDomain LIG_Domain, in Circle2D circle,
                        in TwoDDomain CIRC_Domain, double tolConf, double tol)
    {
        //--  if(! CIRC_Domain.IsClosed()) {
        //--    throw Standard_ConstructionError("Domaine incorrect");
        //--  }

        bool theReversedParameters = ReversedParameters();
        this.ResetFields();
        this.SetReversedParameters(theReversedParameters);

        int nbsol = 0;
        PeriodicInterval cInt1 = new(), cInt2 = new();

        LineCircleGeometricIntersection(line, circle, tolConf, tol, ref cInt1, ref cInt2, out nbsol);

        _done = true;

        if (nbsol == 0)
        { //-- Pas de solutions
            return;
        }

        //  Modified by Sergey KHROMOV - Mon Dec 18 11:13:18 2000 Begin
        if (nbsol == 2 && cInt2.Bsup == cInt1.Binf + PIpPI)
        {
            double firstBound = CIRC_Domain.FirstParameter();
            double lastBound = CIRC_Domain.LastParameter();
            double firstTol = CIRC_Domain.FirstTolerance();
            double lastTol = CIRC_Domain.LastTolerance();
            if (cInt1.Binf == 0 && firstBound - firstTol > cInt1.Bsup)
            {
                nbsol = 1;
                cInt1.SetValues(cInt2.Binf, cInt2.Bsup);
            }
            else if (cInt2.Bsup == PIpPI && lastBound + lastTol < cInt2.Binf)
            {
                nbsol = 1;
            }
        }
        //  Modified by Sergey KHROMOV - Mon Dec 18 11:13:20 2000 End

        PeriodicInterval cDomain = new(CIRC_Domain);
        double deltat = cDomain.Bsup - cDomain.Binf;
        while (cDomain.Binf >= PIpPI)
        {
            cDomain.Binf -= PIpPI;
        }
        while (cDomain.Binf < 0.0)
        {
            cDomain.Binf += PIpPI;
        }
        cDomain.Bsup = cDomain.Binf + deltat;

        //------------------------------------------------------------
        //-- Ajout : Jeudi 28 mars 96
        //-- On agrandit artificiellement les domaines
        double binfModif = cDomain.Binf;
        double bsupModif = cDomain.Bsup;
        binfModif -= CIRC_Domain.FirstTolerance() / circle.Radius;
        bsupModif += CIRC_Domain.LastTolerance() / circle.Radius;
        deltat = bsupModif - binfModif;
        if (deltat <= PIpPI)
        {
            cDomain.Binf = binfModif;
            cDomain.Bsup = bsupModif;
        }
        else
        {
            double t = PIpPI - deltat;
            t *= 0.5;
            cDomain.Binf = binfModif + t;
            cDomain.Bsup = bsupModif - t;
        }
        deltat = cDomain.Bsup - cDomain.Binf;
        while (cDomain.Binf >= PIpPI)
        {
            cDomain.Binf -= PIpPI;
        }
        while (cDomain.Binf < 0.0)
        {
            cDomain.Binf += PIpPI;
        }
        cDomain.Bsup = cDomain.Binf + deltat;
        //-- ------------------------------------------------------------

        Interval lDomain = new(LIG_Domain);

        int nbSolTotal = 0;

        PeriodicInterval[] solutionCircle = [new(), new(), new(), new()];
        Interval[] solutionLine = [new(), new(), new(), new()];

        //----------------------------------------------------------------------
        //----------- Traitement du premier intervalle Geometrique  CInt1   ----
        //----------------------------------------------------------------------
        //-- NbSolTotal est incremente a chaque Intervalle solution.
        //-- On stocke les intervalles dans les tableaux : SolutionCircle[4]
        //--                                            et SolutionLine[4]
        //-- des Exemples faciles donnent 3 Intersections
        //-- des Problemes numeriques peuvent peut etre en donner 4 ??????
        //--
        PeriodicInterval cDomainAndRes = cDomain.FirstIntersection(ref cInt1);

        ProjectOnLAndIntersectWithLDomain(circle, line, ref cDomainAndRes, ref lDomain, solutionCircle, solutionLine,
                                          ref nbSolTotal, LIG_Domain, CIRC_Domain);

        cDomainAndRes = cDomain.SecondIntersection(ref cInt1);

        ProjectOnLAndIntersectWithLDomain(circle, line, ref cDomainAndRes, ref lDomain, solutionCircle, solutionLine,
                                          ref nbSolTotal, LIG_Domain, CIRC_Domain);

        //----------------------------------------------------------------------
        //----------- Traitement du second intervalle Geometrique   C1_Int2 ----
        //----------------------------------------------------------------------
        if (nbsol == 2)
        {
            cDomainAndRes = cDomain.FirstIntersection(ref cInt2);

            ProjectOnLAndIntersectWithLDomain(circle, line, ref cDomainAndRes, ref lDomain, solutionCircle,
                                              solutionLine, ref nbSolTotal, LIG_Domain, CIRC_Domain);

            //--------------------------------------------------------------------
            cDomainAndRes = cDomain.SecondIntersection(ref cInt2);

            ProjectOnLAndIntersectWithLDomain(circle, line, ref cDomainAndRes, ref lDomain, solutionCircle,
                                              solutionLine, ref nbSolTotal, LIG_Domain, CIRC_Domain);
        }

        //----------------------------------------------------------------------
        //-- Calcul de toutes les transitions et Positions.
        //--
        //-- On determine si des intervalles sont reduit a des points
        //--      ( Rayon * Intervalle.Length()    <    TolConf   )   ### Modif 19 Nov Tol-->TolConf
        //--
        double r = circle.Radius;
        int i;
        double maxTol = tolConf;
        if (maxTol < tol)
        {
            maxTol = tol;
        }
        if (maxTol < 1.0e-10)
        {
            maxTol = 1.0e-10;
        }

        for (i = 0; i < nbSolTotal; i++)
        {
            if ((r * solutionCircle[i].Length()) < maxTol && (solutionLine[i].Length()) < maxTol)
            {
                double t = (solutionCircle[i].Binf + solutionCircle[i].Bsup) * 0.5;
                solutionCircle[i].Binf = solutionCircle[i].Bsup = t;

                t = (solutionLine[i].Binf + solutionLine[i].Bsup) * 0.5;
                solutionLine[i].Binf = solutionLine[i].Bsup = t;
            }
        }
        //----------------------------------------------------------------------
        //-- Traitement des intervalles (ou des points obtenus)
        //--
        if (nbSolTotal != 0)
        {
            Frame2D circleAxis = circle.Axis;
            Axis2D lineAxis = line.Position;
            Point2D p1a, p2a, p1b, p2b;
            Vector2D tan1, tan2, norm1;
            Vector2D norm2 = new(0.0, 0.0);
            Transition t1a = new(), t2a = new(), t1b = new(), t2b = new();
            Position2D pos1a, pos1b, pos2a, pos2b;

            global::Kerf.Engine.Geometry.ElCLib.CircleD1(solutionCircle[0].Binf, circleAxis, r, out p1a, out tan1);
            global::Kerf.Engine.Geometry.ElCLib.LineD1(solutionLine[0].Binf, lineAxis, out p2a, out tan2);

            bool isOpposite = (tan1.Dot(tan2) < 0.0);

            for (i = 0; i < nbSolTotal; i++)
            {
                //-- 7 aout 97
                //-- On recentre Bin et Bsup de facon a avoir une portion commune avec CIRC_Domain
                double p1 = solutionCircle[i].Binf;
                double p2 = solutionCircle[i].Bsup;
                double q1 = CIRC_Domain.FirstParameter();
                double q2 = CIRC_Domain.LastParameter();
                //--          |------ CircDomain ------|   [-- Sol --]
                if (p1 > q2)
                {
                    do
                    {
                        p1 -= PIpPI;
                        p2 -= PIpPI;
                    } while ((p1 > q2));
                }
                else if (p2 < q1)
                {
                    do
                    {
                        p1 += PIpPI;
                        p2 += PIpPI;
                    } while ((p2 < q1));
                }
                if (p1 < q1 && p2 > q1)
                {
                    p1 = q1;
                }
                if (p1 < q2 && p2 > q2)
                {
                    p2 = q2;
                }

                solutionCircle[i].Binf = p1;
                solutionCircle[i].Bsup = p2;

                //-- Fin 7 aout 97

                double linf = isOpposite ? solutionLine[i].Bsup : solutionLine[i].Binf;
                double lsup = isOpposite ? solutionLine[i].Binf : solutionLine[i].Bsup;

                //---------------------------------------------------------------
                //-- Si les parametres sur le cercle sont en premier
                //-- On doit retourner ces parametres dans l ordre croissant
                //---------------------------------------------------------------
                if (linf > lsup)
                {
                    double T = solutionCircle[i].Binf;
                    solutionCircle[i].Binf = solutionCircle[i].Bsup;
                    solutionCircle[i].Bsup = T;

                    T = linf;
                    linf = lsup;
                    lsup = T;
                }

                global::Kerf.Engine.Geometry.ElCLib.CircleD2(solutionCircle[i].Binf, circleAxis, r, out p1a, out tan1, out norm1);
                global::Kerf.Engine.Geometry.ElCLib.LineD1(linf, lineAxis, out p2a, out tan2);

                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1a, CIRC_Domain, p1a, solutionCircle[i].Binf);
                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2a, LIG_Domain, p2a, linf);
                Determine_Transition_LC(pos1a, ref tan1, norm1, ref t1a, pos2a, ref tan2, norm2, ref t2a, tol);
                double cinf;
                if (pos1a == Position2D.End)
                {
                    cinf = CIRC_Domain.LastParameter();
                    p1a = CIRC_Domain.LastPoint();
                    linf = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, p1a);

                    global::Kerf.Engine.Geometry.ElCLib.CircleD2(cinf, circleAxis, r, out p1a, out tan1, out norm1);
                    global::Kerf.Engine.Geometry.ElCLib.LineD1(linf, lineAxis, out p2a, out tan2);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1a, CIRC_Domain, p1a, cinf);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2a, LIG_Domain, p2a, linf);
                    Determine_Transition_LC(pos1a, ref tan1, norm1, ref t1a, pos2a, ref tan2, norm2, ref t2a, tol);
                }
                else if (pos1a == Position2D.Head)
                {
                    cinf = CIRC_Domain.FirstParameter();
                    p1a = CIRC_Domain.FirstPoint();
                    linf = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, p1a);

                    global::Kerf.Engine.Geometry.ElCLib.CircleD2(cinf, circleAxis, r, out p1a, out tan1, out norm1);
                    global::Kerf.Engine.Geometry.ElCLib.LineD1(linf, lineAxis, out p2a, out tan2);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1a, CIRC_Domain, p1a, cinf);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2a, LIG_Domain, p2a, linf);
                    Determine_Transition_LC(pos1a, ref tan1, norm1, ref t1a, pos2a, ref tan2, norm2, ref t2a, tol);
                }
                else
                {
                    cinf = NormalizeOnCircleDomain(solutionCircle[i].Binf, CIRC_Domain);
                }

                IntersectionPoint2D newPoint1 = new(p1a, linf, cinf, t2a, t1a, ReversedParameters());

                if ((solutionLine[i].Length() + solutionCircle[i].Length()) > 0.0)
                {
                    global::Kerf.Engine.Geometry.ElCLib.CircleD2(solutionCircle[i].Bsup, circleAxis, r, out p1b, out tan1, out norm1);
                    global::Kerf.Engine.Geometry.ElCLib.LineD1(lsup, lineAxis, out p2b, out tan2);

                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1b, CIRC_Domain, p1b, solutionCircle[i].Bsup);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2b, LIG_Domain, p2b, lsup);
                    Determine_Transition_LC(pos1b, ref tan1, norm1, ref t1b, pos2b, ref tan2, norm2, ref t2b, tol);
                    double csup;
                    if (pos1b == Position2D.End)
                    {
                        csup = CIRC_Domain.LastParameter();
                        p1b = CIRC_Domain.LastPoint();
                        lsup = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, p1b);
                        global::Kerf.Engine.Geometry.ElCLib.CircleD2(csup, circleAxis, r, out p1b, out tan1, out norm1);
                        global::Kerf.Engine.Geometry.ElCLib.LineD1(lsup, lineAxis, out p2b, out tan2);

                        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1b, CIRC_Domain, p1b, csup);
                        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2b, LIG_Domain, p2b, lsup);
                        Determine_Transition_LC(pos1b, ref tan1, norm1, ref t1b, pos2b, ref tan2, norm2, ref t2b, tol);
                    }
                    else if (pos1b == Position2D.Head)
                    {
                        csup = CIRC_Domain.FirstParameter();
                        p1b = CIRC_Domain.FirstPoint();
                        lsup = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, p1b);
                        global::Kerf.Engine.Geometry.ElCLib.CircleD2(csup, circleAxis, r, out p1b, out tan1, out norm1);
                        global::Kerf.Engine.Geometry.ElCLib.LineD1(lsup, lineAxis, out p2b, out tan2);

                        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1b, CIRC_Domain, p1b, csup);
                        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2b, LIG_Domain, p2b, lsup);
                        Determine_Transition_LC(pos1b, ref tan1, norm1, ref t1b, pos2b, ref tan2, norm2, ref t2b, tol);
                    }
                    else
                    {
                        csup = NormalizeOnCircleDomain(solutionCircle[i].Bsup, CIRC_Domain);
                    }

                    IntersectionPoint2D newPoint2 = new(p1b, lsup, csup, t2b, t1b, ReversedParameters());

                    if (((Math.Abs(csup - cinf) * r > maxTol) && (Math.Abs(lsup - linf) > maxTol))
                        || (t1a.TransitionType() != t2a.TransitionType()))
                    {
                        //-- Verifier egalement les transitions

                        IntersectionSegment2D newSeg = new(newPoint1, newPoint2, isOpposite,
                                                                  ReversedParameters());
                        Append(newSeg);
                    }
                    else
                    {
                        if (pos1a != Position2D.Middle || pos2a != Position2D.Middle)
                        {
                            Insert(newPoint1);
                        }
                        if (pos1b != Position2D.Middle || pos2b != Position2D.Middle)
                        {
                            Insert(newPoint2);
                        }
                    }
                }
                else
                {
                    //--double
                    // Cmid=NormalizeOnCircleDomain(0.5*(SolutionCircle[i].Bsup+SolutionCircle[i].Binf)
                    //--					   ,CIRC_Domain);
                    //--IntRes2d_IntersectionPoint NewPoint(P2a,0.5*(Linf+Lsup)
                    //--				    ,Cmid
                    //--				    ,T2a,T1a,ReversedParameters());
                    Insert(newPoint1);
                }
            }
        }
    }

    private static IntersectionPoint2D SegmentToPoint(in IntersectionPoint2D pa,
                                                             in Transition t1a,
                                                             in Transition t2a,
                                                             in IntersectionPoint2D pb,
                                                             in Transition t1b,
                                                             in Transition t2b)
    {
        if ((t1b.PositionOnCurve() == Position2D.Middle) && (t2b.PositionOnCurve() == Position2D.Middle))
        {
            return (pa);
        }
        if ((t1a.PositionOnCurve() == Position2D.Middle) && (t2a.PositionOnCurve() == Position2D.Middle))
        {
            return (pb);
        }

        Transition t1 = t1a;
        Transition t2 = t2a;
        double u1 = pa.ParamOnFirst();
        double u2 = pa.ParamOnSecond();

        if (t1.PositionOnCurve() == Position2D.Middle)
        {
            t1.SetPosition(t1b.PositionOnCurve());
            u1 = pb.ParamOnFirst();
        }
        if (t2.PositionOnCurve() == Position2D.Middle)
        {
            t2.SetPosition(t2b.PositionOnCurve());
            u2 = pb.ParamOnSecond();
        }
        return (new IntersectionPoint2D(pa.Value(), u1, u2, t1, t2, false));
    }

    //=================================================================================================

    private static void LineEllipseGeometricIntersection(in Line2D line,
                                                         in Ellipse2D ellipse,
                                                         double tol,
                                                         double tolTang,
                                                         ref PeriodicInterval eInt1,
                                                         ref PeriodicInterval eInt2,
                                                         out int nbsol)
    {
        Frame2D elAxis = ellipse.Axis;
        Transform2D tr = new Transform2D().WithTransformation(elAxis.XAxis);
        Ellipse2D tEllipse = ellipse.Transformed(tr);
        Line2D tLine = line.Transformed(tr);
        double dy = tLine.Position.Direction.Y;
        bool isVert = Math.Abs(dy) > 1.0 - 2.0 * Epsilon(1.0);
        //
        double a = tEllipse.MajorRadius;
        double b = tEllipse.MinorRadius;
        double a2 = a * a;
        double b2 = b * b;
        //
        double eps0 = 1.0e-12;
        if (b / a < 1.0e-5)
        {
            eps0 = 1.0e-6;
        }
        //
        double anA, aB, aC;
        tLine.Coefficients(out anA, out aB, out aC);
        if (isVert)
        {
            aC += aB * tLine.Position.Location.Y;
            aB = 0.0;
        }
        //
        double x1 = 0.0, y1 = 0.0, x2 = 0.0, y2 = 0.0;
        if (Math.Abs(aB) > eps0)
        {
            double m = -anA / aB;
            double m2 = m * m;
            double c = -aC / aB;
            double c2 = c * c;
            double d = a2 * m2 + b2 - c2;
            if (d < 0.0)
            {
                ExtElC2d ext = new(tLine, tEllipse);
                int i, imin = 0;
                double dmin = RealLast();
                for (i = 1; i <= ext.ExtCount; ++i)
                {
                    if (ext.SquareDistance(i) < dmin)
                    {
                        dmin = ext.SquareDistance(i);
                        imin = i;
                    }
                }
                if (imin > 0 && dmin <= tolTang * tolTang)
                {
                    nbsol = 1;
                    ext.Points(imin, out _, out POnCurv2d aPE2);
                    double pe = aPE2.Parameter();
                    eInt1.SetValues(pe, pe);
                }
                else
                {
                    nbsol = 0;
                }
                return;
            }
            d = Math.Sqrt(d);
            double n = a2 * m2 + b2;
            double k = a * b * d / n;
            double l = -a2 * m * c / n;
            x1 = l + k;
            y1 = m * x1 + c;
            x2 = l - k;
            y2 = m * x2 + c;
            nbsol = 2;
        }
        else
        {
            x1 = -aC / anA;
            if (Math.Abs(x1) > a + tolTang)
            {
                nbsol = 0;
                return;
            }
            else if (Math.Abs(x1) >= a - Epsilon(1.0 + a))
            {
                nbsol = 1;
                y1 = 0.0;
            }
            else
            {
                y1 = b * Math.Sqrt(1.0 - x1 * x1 / a2);
                x2 = x1;
                y2 = -y1;
                nbsol = 2;
            }
        }

        Point2D p1 = new(x1, y1);
        Point2D p2 = new(x2, y2);
        double pe1 = 0.0, pe2 = 0.0;
        pe1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(tEllipse, p1);
        if (nbsol > 1)
        {
            pe2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(tEllipse, p2);
            if (pe2 < pe1)
            {
                double t = pe1;
                pe1 = pe2;
                pe2 = t;
            }
            eInt2.SetValues(pe2, pe2);
        }
        eInt1.SetValues(pe1, pe1);
    }

    //=================================================================================================

    private static void ProjectOnLAndIntersectWithLDomain(in Ellipse2D ellipse,
                                                          in Line2D line,
                                                          ref PeriodicInterval eDomainAndRes,
                                                          ref Interval lDomain,
                                                          PeriodicInterval[] ellipseSolution,
                                                          Interval[] lineSolution,
                                                          ref int nbSolTotal,
                                                          in TwoDDomain refLineDomain,
                                                          in TwoDDomain _)
    {
        if (eDomainAndRes.IsNull)
        {
            return;
        }
        //-------------------------------------------------------------------------
        //--  On cherche l intervalle correspondant sur C2
        //--  Puis on intersecte l intervalle avec le domaine de C2
        //--  Enfin, on cherche l intervalle correspondant sur C1
        //--

        double linf = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, global::Kerf.Engine.Geometry.ElCLib.Value(eDomainAndRes.Binf, ellipse));
        double lsup = global::Kerf.Engine.Geometry.ElCLib.Parameter(line, global::Kerf.Engine.Geometry.ElCLib.Value(eDomainAndRes.Bsup, ellipse));

        Interval lInter = new(linf, lsup); //-- Necessairement Borne

        Interval lInterAndDomain = lDomain.IntersectionWithBounded(lInter);

        if (!lInterAndDomain.IsNull)
        {
            double domLinf = (refLineDomain.HasFirstPoint) ? refLineDomain.FirstParameter() : -global::Kerf.Engine.Foundation.Precision.Infinite();
            double domLsup = (refLineDomain.HasLastPoint) ? refLineDomain.LastParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();

            linf = lInterAndDomain.Binf;
            lsup = lInterAndDomain.Bsup;

            if (linf < domLinf)
            {
                linf = domLinf;
            }
            if (lsup < domLinf)
            {
                lsup = domLinf;
            }

            if (linf > domLsup)
            {
                linf = domLsup;
            }
            if (lsup > domLsup)
            {
                lsup = domLsup;
            }

            lInterAndDomain.Binf = linf;
            lInterAndDomain.Bsup = lsup;

            double einf = eDomainAndRes.Binf;
            double esup = eDomainAndRes.Bsup;

            if (einf >= esup)
            {
                einf = eDomainAndRes.Binf;
                esup = eDomainAndRes.Bsup;
            }
            ellipseSolution[nbSolTotal] = new PeriodicInterval(einf, esup);
            if (ellipseSolution[nbSolTotal].Length() > M_PI)
            {
                ellipseSolution[nbSolTotal].Complement();
            }

            lineSolution[nbSolTotal] = lInterAndDomain;
            nbSolTotal++;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between a line and an ellipse.</summary>
    public void Perform(in Line2D l, in TwoDDomain dl, in Ellipse2D e, in TwoDDomain de, double tolConf,
                        double tol)
    {
        bool theReversedParameters = ReversedParameters();
        this.ResetFields();
        this.SetReversedParameters(theReversedParameters);

        int nbsol = 0;
        PeriodicInterval eInt1 = new(), eInt2 = new();

        LineEllipseGeometricIntersection(l, e, tolConf, tol, ref eInt1, ref eInt2, out nbsol);
        _done = true;
        if (nbsol == 0)
        {
            return;
        }
        //
        if (nbsol == 2 && eInt2.Bsup == eInt1.Binf + PIpPI)
        {
            double firstBound = de.FirstParameter();
            double lastBound = de.LastParameter();
            double firstTol = de.FirstTolerance();
            double lastTol = de.LastTolerance();
            if (eInt1.Binf == 0 && firstBound - firstTol > eInt1.Bsup)
            {
                nbsol = 1;
                eInt1.SetValues(eInt2.Binf, eInt2.Bsup);
            }
            else if (eInt2.Bsup == PIpPI && lastBound + lastTol < eInt2.Binf)
            {
                nbsol = 1;
            }
        }
        //
        PeriodicInterval eDomain = new(de);
        double deltat = eDomain.Bsup - eDomain.Binf;
        while (eDomain.Binf >= PIpPI)
        {
            eDomain.Binf -= PIpPI;
        }
        while (eDomain.Binf < 0.0)
        {
            eDomain.Binf += PIpPI;
        }
        eDomain.Bsup = eDomain.Binf + deltat;
        //
        double binfModif = eDomain.Binf;
        double bsupModif = eDomain.Bsup;
        binfModif -= de.FirstTolerance() / e.MinorRadius;
        bsupModif += de.LastTolerance() / e.MinorRadius;
        deltat = bsupModif - binfModif;
        if (deltat <= PIpPI)
        {
            eDomain.Binf = binfModif;
            eDomain.Bsup = bsupModif;
        }
        else
        {
            double t = PIpPI - deltat;
            t *= 0.5;
            eDomain.Binf = binfModif + t;
            eDomain.Bsup = bsupModif - t;
        }
        deltat = eDomain.Bsup - eDomain.Binf;
        while (eDomain.Binf >= PIpPI)
        {
            eDomain.Binf -= PIpPI;
        }
        while (eDomain.Binf < 0.0)
        {
            eDomain.Binf += PIpPI;
        }
        eDomain.Bsup = eDomain.Binf + deltat;
        //
        Interval lDomain = new(dl);

        int nbSolTotal = 0;

        PeriodicInterval[] solutionEllipse = [new(), new(), new(), new()];
        Interval[] solutionLine = [new(), new(), new(), new()];
        //----------------------------------------------------------------------
        //----------- Treatment of first geometric interval EInt1           ----
        //----------------------------------------------------------------------
        PeriodicInterval eDomainAndRes = eDomain.FirstIntersection(ref eInt1);

        ProjectOnLAndIntersectWithLDomain(e, l, ref eDomainAndRes, ref lDomain, solutionEllipse, solutionLine,
                                          ref nbSolTotal, dl, de);

        eDomainAndRes = eDomain.SecondIntersection(ref eInt1);

        ProjectOnLAndIntersectWithLDomain(e, l, ref eDomainAndRes, ref lDomain, solutionEllipse, solutionLine,
                                          ref nbSolTotal, dl, de);

        //----------------------------------------------------------------------
        //----------- Treatment of second geometric interval EInt2          ----
        //----------------------------------------------------------------------
        if (nbsol == 2)
        {
            eDomainAndRes = eDomain.FirstIntersection(ref eInt2);

            ProjectOnLAndIntersectWithLDomain(e, l, ref eDomainAndRes, ref lDomain, solutionEllipse, solutionLine,
                                              ref nbSolTotal, dl, de);

            eDomainAndRes = eDomain.SecondIntersection(ref eInt2);

            ProjectOnLAndIntersectWithLDomain(e, l, ref eDomainAndRes, ref lDomain, solutionEllipse, solutionLine,
                                              ref nbSolTotal, dl, de);
        }

        //----------------------------------------------------------------------
        //-- Calculation of Transitions at Positions.
        //----------------------------------------------------------------------
        double r = e.MinorRadius;
        int i;
        double maxTol = tolConf;
        if (maxTol < tol)
        {
            maxTol = tol;
        }
        if (maxTol < 1.0e-10)
        {
            maxTol = 1.0e-10;
        }

        for (i = 0; i < nbSolTotal; i++)
        {
            if ((r * solutionEllipse[i].Length()) < maxTol && (solutionLine[i].Length()) < maxTol)
            {
                double t = (solutionEllipse[i].Binf + solutionEllipse[i].Bsup) * 0.5;
                solutionEllipse[i].Binf = solutionEllipse[i].Bsup = t;

                t = (solutionLine[i].Binf + solutionLine[i].Bsup) * 0.5;
                solutionLine[i].Binf = solutionLine[i].Bsup = t;
            }
        }
        //
        if (nbSolTotal != 0)
        {
            Frame2D ellipseAxis = e.Axis;
            Axis2D lineAxis = l.Position;
            Point2D p1a, p2a, p1b, p2b;
            Vector2D tan1, tan2, norm1;
            Vector2D norm2 = new(0.0, 0.0);
            Transition t1a = new(), t2a = new(), t1b = new(), t2b = new();
            Position2D pos1a, pos1b, pos2a, pos2b;

            global::Kerf.Engine.Geometry.ElCLib.EllipseD1(solutionEllipse[0].Binf, ellipseAxis, e.MajorRadius, e.MinorRadius, out p1a,
                             out tan1);
            global::Kerf.Engine.Geometry.ElCLib.LineD1(solutionLine[0].Binf, lineAxis, out p2a, out tan2);

            bool isOpposite = (tan1.Dot(tan2) < 0.0);
            for (i = 0; i < nbSolTotal; i++)
            {
                double p1 = solutionEllipse[i].Binf;
                double p2 = solutionEllipse[i].Bsup;
                double q1 = de.FirstParameter();
                double q2 = de.LastParameter();

                if (p1 > q2)
                {
                    do
                    {
                        p1 -= PIpPI;
                        p2 -= PIpPI;
                    } while ((p1 > q2));
                }
                else if (p2 < q1)
                {
                    do
                    {
                        p1 += PIpPI;
                        p2 += PIpPI;
                    } while ((p2 < q1));
                }
                if (p1 < q1 && p2 > q1)
                {
                    p1 = q1;
                }
                if (p1 < q2 && p2 > q2)
                {
                    p2 = q2;
                }

                solutionEllipse[i].Binf = p1;
                solutionEllipse[i].Bsup = p2;

                double linf = isOpposite ? solutionLine[i].Bsup : solutionLine[i].Binf;
                double lsup = isOpposite ? solutionLine[i].Binf : solutionLine[i].Bsup;

                if (linf > lsup)
                {
                    double T = solutionEllipse[i].Binf;
                    solutionEllipse[i].Binf = solutionEllipse[i].Bsup;
                    solutionEllipse[i].Bsup = T;
                    T = linf;
                    linf = lsup;
                    lsup = T;
                }

                global::Kerf.Engine.Geometry.ElCLib.EllipseD2(solutionEllipse[i].Binf, ellipseAxis, e.MajorRadius, e.MinorRadius, out p1a,
                                 out tan1, out norm1);
                global::Kerf.Engine.Geometry.ElCLib.LineD1(linf, lineAxis, out p2a, out tan2);

                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1a, de, p1a, solutionEllipse[i].Binf);
                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2a, dl, p2a, linf);
                Determine_Transition_LC(pos1a, ref tan1, norm1, ref t1a, pos2a, ref tan2, norm2, ref t2a, tol);
                double einf;
                if (pos1a == Position2D.End)
                {
                    einf = de.LastParameter();
                    p1a = de.LastPoint();
                    linf = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, p1a);

                    global::Kerf.Engine.Geometry.ElCLib.EllipseD2(einf, ellipseAxis, e.MajorRadius, e.MinorRadius, out p1a, out tan1,
                                     out norm1);
                    global::Kerf.Engine.Geometry.ElCLib.LineD1(linf, lineAxis, out p2a, out tan2);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1a, de, p1a, einf);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2a, dl, p2a, linf);
                    Determine_Transition_LC(pos1a, ref tan1, norm1, ref t1a, pos2a, ref tan2, norm2, ref t2a, tol);
                }
                else if (pos1a == Position2D.Head)
                {
                    einf = de.FirstParameter();
                    p1a = de.FirstPoint();
                    linf = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, p1a);

                    global::Kerf.Engine.Geometry.ElCLib.EllipseD2(einf, ellipseAxis, e.MajorRadius, e.MinorRadius, out p1a, out tan1,
                                     out norm1);
                    global::Kerf.Engine.Geometry.ElCLib.LineD1(linf, lineAxis, out p2a, out tan2);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1a, de, p1a, einf);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2a, dl, p2a, linf);
                    Determine_Transition_LC(pos1a, ref tan1, norm1, ref t1a, pos2a, ref tan2, norm2, ref t2a, tol);
                }
                else
                {
                    einf = NormalizeOnCircleDomain(solutionEllipse[i].Binf, de);
                }

                IntersectionPoint2D newPoint1 = new(p1a, linf, einf, t2a, t1a, ReversedParameters());

                if ((solutionLine[i].Length() + solutionEllipse[i].Length()) > 0.0)
                {
                    // OCCT evaluates the ellipse at Binf, not Bsup, for the end of the interval (as is).
                    global::Kerf.Engine.Geometry.ElCLib.EllipseD2(solutionEllipse[i].Binf, ellipseAxis, e.MajorRadius, e.MinorRadius, out p1b,
                                     out tan1, out norm1);
                    global::Kerf.Engine.Geometry.ElCLib.LineD1(lsup, lineAxis, out p2b, out tan2);

                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1b, de, p1b, solutionEllipse[i].Bsup);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2b, dl, p2b, lsup);
                    Determine_Transition_LC(pos1b, ref tan1, norm1, ref t1b, pos2b, ref tan2, norm2, ref t2b, tol);
                    double esup;
                    if (pos1b == Position2D.End)
                    {
                        // OCCT takes the line's last parameter as the ellipse parameter here (as is).
                        esup = dl.LastParameter();
                        p1b = de.LastPoint();
                        lsup = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, p1b);
                        global::Kerf.Engine.Geometry.ElCLib.EllipseD2(esup, ellipseAxis, e.MajorRadius, e.MinorRadius, out p1b, out tan1,
                                         out norm1);
                        global::Kerf.Engine.Geometry.ElCLib.LineD1(lsup, lineAxis, out p2b, out tan2);

                        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1b, de, p1b, esup);
                        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2b, dl, p2b, lsup);
                        Determine_Transition_LC(pos1b, ref tan1, norm1, ref t1b, pos2b, ref tan2, norm2, ref t2b, tol);
                    }
                    else if (pos1b == Position2D.Head)
                    {
                        esup = de.FirstParameter();
                        p1b = de.FirstPoint();
                        lsup = global::Kerf.Engine.Geometry.ElCLib.Parameter(l, p1b);
                        global::Kerf.Engine.Geometry.ElCLib.EllipseD2(esup, ellipseAxis, e.MajorRadius, e.MinorRadius, out p1b, out tan1,
                                         out norm1);
                        global::Kerf.Engine.Geometry.ElCLib.LineD1(lsup, lineAxis, out p2b, out tan2);

                        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1b, de, p1b, esup);
                        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2b, dl, p2b, lsup);
                        Determine_Transition_LC(pos1b, ref tan1, norm1, ref t1b, pos2b, ref tan2, norm2, ref t2b, tol);
                    }
                    else
                    {
                        esup = NormalizeOnCircleDomain(solutionEllipse[i].Bsup, de);
                    }

                    IntersectionPoint2D newPoint2 = new(p1b, lsup, esup, t2b, t1b, ReversedParameters());

                    if (((Math.Abs(esup - einf) * r > maxTol) && (Math.Abs(lsup - linf) > maxTol))
                        || (t1a.TransitionType() != t2a.TransitionType()))
                    {
                        IntersectionSegment2D newSeg = new(newPoint1, newPoint2, isOpposite,
                                                                  ReversedParameters());
                        Append(newSeg);
                    }
                    else
                    {
                        if (pos1a != Position2D.Middle || pos2a != Position2D.Middle)
                        {
                            Insert(newPoint1);
                        }
                        if (pos1b != Position2D.Middle || pos2b != Position2D.Middle)
                        {
                            Insert(newPoint2);
                        }
                    }
                }
                else
                {
                    Insert(newPoint1);
                }
            }
        }
    }
}
