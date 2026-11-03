// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicConic.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicConic.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicConic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The intersection of two conics: lines, circles, ellipses, parabolas and hyperbolas, with domains.
/// </summary>
/// <remarks>
/// Circles and ellipses must have closed domains (<see cref="IntRes2d_Domain.WithEquivalentParameters"/>); where
/// they don't, most methods close them on [first, first + 2 PI]. Line/line, line/circle, circle/circle and
/// line/ellipse are solved geometrically (the second file, IntCurve_IntConicConic_1); the other pairs by
/// <see cref="IntCurve_IntImpConicParConic"/> on a parametric domain bounded around the analytic solutions.
/// </remarks>
public sealed partial class IntCurve_IntConicConic : IntRes2d_Intersection
{
    //=======================================================================
    // Perform() for
    //              Line      - Parabola
    //              Line      - Hyperbola
    //              Circle    - Parabola
    //              Circle    - Ellipse
    //              Circle    - Hyperbola
    //              Parabola  - Parabola
    //              Ellipse    - Parabola
    //              Parabola  - Hyperbola
    //              Ellipse    - Ellipse
    //              Ellipse    - Hyperbola
    //              Hyperbola - Hyperbola
    //=======================================================================
    private const double PARAM_MAX_ON_PARABOLA = 100000000.0;
    private const double PARAM_MAX_ON_HYPERBOLA = 10000.0;
    private const double TOL_EXACT_INTER = 1.0e-7;

    private readonly IntCurve_IntImpConicParConic Inter = new();

    /// <summary>Empty constructor.</summary>
    public IntCurve_IntConicConic()
    {
    }

    /// <summary>Intersection between two lines.</summary>
    public IntCurve_IntConicConic(in Line2D L1, in IntRes2d_Domain D1, in Line2D L2, in IntRes2d_Domain D2,
                                  double TolConf, double Tol)
    {
        Perform(L1, D1, L2, D2, TolConf, Tol);
    }

    /// <summary>Intersection between a line and a circle.</summary>
    public IntCurve_IntConicConic(in Line2D L1, in IntRes2d_Domain D1, in Circle2D C2, in IntRes2d_Domain D2,
                                  double TolConf, double Tol)
    {
        Perform(L1, D1, C2, D2, TolConf, Tol);
    }

    /// <summary>Intersection between a line and an ellipse.</summary>
    public IntCurve_IntConicConic(in Line2D L, in IntRes2d_Domain DL, in Ellipse2D E, in IntRes2d_Domain DE,
                                  double TolConf, double Tol)
    {
        Perform(L, DL, E, DE, TolConf, Tol);
    }

    /// <summary>Intersection between a line and a parabola.</summary>
    public IntCurve_IntConicConic(in Line2D L, in IntRes2d_Domain DL, in Parabola2D P, in IntRes2d_Domain DP,
                                  double TolConf, double Tol)
    {
        Perform(L, DL, P, DP, TolConf, Tol);
    }

    /// <summary>Intersection between a line and a hyperbola.</summary>
    public IntCurve_IntConicConic(in Line2D L, in IntRes2d_Domain DL, in Hyperbola2D H, in IntRes2d_Domain DH,
                                  double TolConf, double Tol)
    {
        Perform(L, DL, H, DH, TolConf, Tol);
    }

    /// <summary>Intersection between two circles.</summary>
    public IntCurve_IntConicConic(in Circle2D C1, in IntRes2d_Domain D1, in Circle2D C2, in IntRes2d_Domain D2,
                                  double TolConf, double Tol)
    {
        SetReversedParameters(false);
        Perform(C1, D1, C2, D2, TolConf, Tol);
    }

    /// <summary>Intersection between a circle and an ellipse.</summary>
    public IntCurve_IntConicConic(in Circle2D C, in IntRes2d_Domain DC, in Ellipse2D E, in IntRes2d_Domain DE,
                                  double TolConf, double Tol)
    {
        Perform(C, DC, E, DE, TolConf, Tol);
    }

    /// <summary>Intersection between a circle and a parabola.</summary>
    public IntCurve_IntConicConic(in Circle2D C, in IntRes2d_Domain DC, in Parabola2D P, in IntRes2d_Domain DP,
                                  double TolConf, double Tol)
    {
        Perform(C, DC, P, DP, TolConf, Tol);
    }

    /// <summary>Intersection between a circle and a hyperbola.</summary>
    public IntCurve_IntConicConic(in Circle2D C, in IntRes2d_Domain DC, in Hyperbola2D H, in IntRes2d_Domain DH,
                                  double TolConf, double Tol)
    {
        Perform(C, DC, H, DH, TolConf, Tol);
    }

    /// <summary>Intersection between two ellipses.</summary>
    public IntCurve_IntConicConic(in Ellipse2D E1, in IntRes2d_Domain D1, in Ellipse2D E2, in IntRes2d_Domain D2,
                                  double TolConf, double Tol)
    {
        Perform(E1, D1, E2, D2, TolConf, Tol);
    }

    /// <summary>Intersection between an ellipse and a parabola.</summary>
    public IntCurve_IntConicConic(in Ellipse2D E, in IntRes2d_Domain DE, in Parabola2D P, in IntRes2d_Domain DP,
                                  double TolConf, double Tol)
    {
        Perform(E, DE, P, DP, TolConf, Tol);
    }

    /// <summary>Intersection between an ellipse and a hyperbola.</summary>
    public IntCurve_IntConicConic(in Ellipse2D E, in IntRes2d_Domain DE, in Hyperbola2D H, in IntRes2d_Domain DH,
                                  double TolConf, double Tol)
    {
        Perform(E, DE, H, DH, TolConf, Tol);
    }

    /// <summary>Intersection between two parabolas.</summary>
    public IntCurve_IntConicConic(in Parabola2D P1, in IntRes2d_Domain D1, in Parabola2D P2, in IntRes2d_Domain D2,
                                  double TolConf, double Tol)
    {
        Perform(P1, D1, P2, D2, TolConf, Tol);
    }

    /// <summary>Intersection between a parabola and a hyperbola.</summary>
    public IntCurve_IntConicConic(in Parabola2D P, in IntRes2d_Domain DP, in Hyperbola2D H, in IntRes2d_Domain DH,
                                  double TolConf, double Tol)
    {
        Perform(P, DP, H, DH, TolConf, Tol);
    }

    /// <summary>Intersection between two hyperbolas.</summary>
    public IntCurve_IntConicConic(in Hyperbola2D H1, in IntRes2d_Domain DH1, in Hyperbola2D H2, in IntRes2d_Domain DH2,
                                  double TolConf, double Tol)
    {
        Perform(H1, DH1, H2, DH2, TolConf, Tol);
    }

    private static bool BOUNDED_DOMAIN(in IntRes2d_Domain domain)
    {
        return (domain.HasFirstPoint() && domain.HasLastPoint());
    }

    private static bool SET_BOUNDED_DOMAIN(in IntRes2d_Domain domain,
                                           ref double binf,
                                           ref double tolinf,
                                           ref Point2D Pntinf,
                                           ref double bsup,
                                           ref double tolsup,
                                           ref Point2D Pntsup)
    {
        if (domain.HasFirstPoint())
        {
            if (binf < domain.FirstParameter())
            {
                Pntinf = domain.FirstPoint();
                binf = domain.FirstParameter();
                tolinf = domain.FirstTolerance();
            }
        }
        if (domain.HasLastPoint())
        {
            // OCCT compares bsup with the domain's first parameter, not its last one (as is).
            if (bsup > domain.FirstParameter())
            {
                Pntsup = domain.LastPoint();
                bsup = domain.LastParameter();
                tolsup = domain.LastTolerance();
            }
        }
        bool result = bsup > binf;
        //   if(bsup<=binf) return(false);
        //   return(true);
        return result;
    }

    //=================================================================================================

    /// <summary>Intersection between a line and a parabola.</summary>
    public void Perform(in Line2D L, in IntRes2d_Domain DL, in Parabola2D P, in IntRes2d_Domain DP, double TolConf,
                        double Tol)
    {
        this.ResetFields();
        IntCurve_IConicTool ITool = new(L);
        IntCurve_PConic PCurve = new(P);

        PCurve.SetAccuracy(20);
        Inter.SetReversedParameters(ReversedParameters());

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite();
        double bsup = -global::Kerf.Engine.Foundation.Precision.Infinite();
        double maxtol = (Tol > TolConf) ? Tol : TolConf;
        if (maxtol < 1.0e-7)
        {
            maxtol = 1.0e-7;
        }
        bool wasSet = false;

        Point2D Pntinf = new(), Pntsup = new();
        AnalyticIntersection theIntAna2d = new();

        maxtol *= 100.0;
        //  if(maxtol<0.000001) maxtol = 0.000001;

        // OCCT offsets the line by (maxtol Dy, maxtol Dx), which is not normal to it (as is).
        Vector2D Offset = new(maxtol * L.Direction().Y(), maxtol * L.Direction().X());
        Line2D Lp = L.Translated(Offset);
        theIntAna2d.Perform(P, new Conic(Lp));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, P, maxtol,
                                PARAM_MAX_ON_PARABOLA);

        Offset.Reverse();
        Line2D Lm = L.Translated(Offset);
        theIntAna2d.Perform(P, new Conic(Lm));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, P, maxtol,
                                PARAM_MAX_ON_PARABOLA);

        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(DP))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(DP, ref binf, ref tolinf, ref Pntinf, ref bsup, ref tolsup, ref Pntsup))
                {
                    IntRes2d_Domain DPModif = new(Pntinf, binf, tolinf, Pntsup, bsup, tolsup);
                    Inter.Perform(ITool, DL, PCurve, DPModif, TolConf, Tol);
                }
                else
                {
                    done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < DP.FirstParameter())
                {
                    binf = DP.FirstParameter();
                    Pntinf = DP.FirstPoint();
                    ft = DP.FirstTolerance();
                    if (bsup < DP.FirstParameter())
                    {
                        done = true;
                        return;
                    }
                }
                if (bsup > DP.LastParameter())
                {
                    bsup = DP.LastParameter();
                    Pntsup = DP.LastPoint();
                    lt = DP.LastTolerance();
                    if (binf > DP.LastParameter())
                    {
                        done = true;
                        return;
                    }
                }
                IntRes2d_Domain DPModif = new(Pntinf, binf, ft, Pntsup, bsup, lt);
                //      Inter.Perform(ITool,DL,PCurve,DPModif,TolConf,Tol);
                Inter.Perform(ITool, DL, PCurve, DPModif, TOL_EXACT_INTER, TOL_EXACT_INTER);
                this.SetValues(Inter);
                wasSet = true;
                if (this.IsDone() && this.NbPoints() == 0)
                {
                    this.ResetFields();
                    Inter.Perform(ITool, DL, PCurve, DPModif, TolConf, Tol);
                    wasSet = false;
                }
            }
            if (!wasSet)
            {
                this.SetValues(Inter);
            }
        }
        else
        {
            done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between a line and a hyperbola.</summary>
    public void Perform(in Line2D L, in IntRes2d_Domain DL, in Hyperbola2D H, in IntRes2d_Domain DH, double TolConf,
                        double Tol)
    {
        this.ResetFields();
        IntCurve_IConicTool ITool = new(L);
        IntCurve_PConic PCurve = new(H);
        PCurve.SetAccuracy(20);

        Inter.SetReversedParameters(ReversedParameters());

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D Pntinf = new(), Pntsup = new();
        if (Tol > TolConf)
        {
            maxtol = Tol;
        }
        else
        {
            maxtol = TolConf;
        }
        maxtol *= 100.0;
        if (maxtol < 0.000001)
        {
            maxtol = 0.000001;
        }
        Vector2D Offset = new(maxtol * H.XAxis().Direction().X(), maxtol * H.XAxis().Direction().Y());
        Hyperbola2D Hp = H.Translated(Offset);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(Hp, new Conic(L));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, H, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        Offset.Reverse();
        Hyperbola2D Hm = H.Translated(Offset);
        theIntAna2d.Perform(Hm, new Conic(L));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, H, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(DH))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(DH, ref binf, ref tolinf, ref Pntinf, ref bsup, ref tolsup, ref Pntsup))
                {
                    IntRes2d_Domain DHModif = new(Pntinf, binf, tolinf, Pntsup, bsup, tolsup);
                    Inter.Perform(ITool, DL, PCurve, DHModif, TolConf, Tol);
                }
                else
                {
                    done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < DH.FirstParameter())
                {
                    binf = DH.FirstParameter();
                    Pntinf = DH.FirstPoint();
                    ft = DH.FirstTolerance();
                    if (bsup < DH.FirstParameter())
                    {
                        done = true;
                        return;
                    }
                }
                if (bsup > DH.LastParameter())
                {
                    bsup = DH.LastParameter();
                    Pntsup = DH.LastPoint();
                    lt = DH.LastTolerance();
                    if (binf > DH.LastParameter())
                    {
                        done = true;
                        return;
                    }
                }
                IntRes2d_Domain DHModif = new(Pntinf, binf, ft, Pntsup, bsup, lt);
                Inter.Perform(ITool, DL, PCurve, DHModif, TolConf, Tol);
            }
            this.SetValues(Inter);
        }
        else
        {
            done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between a circle and a parabola.</summary>
    public void Perform(in Circle2D C, in IntRes2d_Domain DC, in Parabola2D P, in IntRes2d_Domain DP, double TolConf,
                        double Tol)
    {
        this.ResetFields();
        IntCurve_IConicTool ITool = new(C);
        IntCurve_PConic PCurve = new(P);
        PCurve.SetAccuracy(20);
        Inter.SetReversedParameters(ReversedParameters());

        IntRes2d_Domain D = DC;
        if (!DC.IsClosed())
        {
            D.SetEquivalentParameters(DC.FirstParameter(), DC.FirstParameter() + M_PI + M_PI);
        }

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D Pntinf = new(), Pntsup = new();
        maxtol = C.Radius() / 10.0;
        Circle2D Cp = C;
        Cp.SetRadius(C.Radius() + maxtol);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(P, new Conic(Cp));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, P, maxtol,
                                PARAM_MAX_ON_PARABOLA);
        if (C.Radius() > maxtol)
        {
            Cp.SetRadius(C.Radius() - maxtol);
            theIntAna2d.Perform(P, new Conic(Cp));
            SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, P, maxtol,
                                    PARAM_MAX_ON_PARABOLA);
        }
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(DP))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(DP, ref binf, ref tolinf, ref Pntinf, ref bsup, ref tolsup, ref Pntsup))
                {
                    IntRes2d_Domain DPModif = new(Pntinf, binf, tolinf, Pntsup, bsup, tolsup);
                    Inter.Perform(ITool, D, PCurve, DPModif, TolConf, Tol);
                }
                else
                {
                    done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < DP.FirstParameter())
                {
                    binf = DP.FirstParameter();
                    Pntinf = DP.FirstPoint();
                    ft = DP.FirstTolerance();
                    if (bsup < DP.FirstParameter())
                    {
                        done = true;
                        return;
                    }
                }
                if (bsup > DP.LastParameter())
                {
                    bsup = DP.LastParameter();
                    Pntsup = DP.LastPoint();
                    lt = DP.LastTolerance();
                    if (binf > DP.LastParameter())
                    {
                        done = true;
                        return;
                    }
                }
                IntRes2d_Domain DPModif = new(Pntinf, binf, ft, Pntsup, bsup, lt);
                Inter.Perform(ITool, D, PCurve, DPModif, TolConf, Tol);
            }
            this.SetValues(Inter);
        }
        else
        {
            done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between a circle and an ellipse.</summary>
    public void Perform(in Circle2D C, in IntRes2d_Domain DC, in Ellipse2D E, in IntRes2d_Domain DE, double TolConf,
                        double Tol)
    {
        this.ResetFields();
        IntCurve_IConicTool ITool = new(C);
        IntCurve_PConic PCurve = new(E);
        PCurve.SetAccuracy(20);

        Inter.SetReversedParameters(ReversedParameters());

        if (!DC.IsClosed())
        {
            IntRes2d_Domain D1 = DC;
            D1.SetEquivalentParameters(DC.FirstParameter(), DC.FirstParameter() + M_PI + M_PI);
            if (!DE.IsClosed())
            {
                IntRes2d_Domain D2 = DE;
                D2.SetEquivalentParameters(DE.FirstParameter(), DE.FirstParameter() + M_PI + M_PI);
                Inter.Perform(ITool, D1, PCurve, D2, TolConf, Tol);
            }
            else
            {
                Inter.Perform(ITool, D1, PCurve, DE, TolConf, Tol);
            }
        }
        else
        {
            if (!DE.IsClosed())
            {
                IntRes2d_Domain D2 = DE;
                D2.SetEquivalentParameters(DE.FirstParameter(), DE.FirstParameter() + M_PI + M_PI);
                Inter.Perform(ITool, DC, PCurve, D2, TolConf, Tol);
            }
            else
            {
                Inter.Perform(ITool, DC, PCurve, DE, TolConf, Tol);
            }
        }
        this.SetValues(Inter);
    }

    //=================================================================================================

    /// <summary>Intersection between a circle and a hyperbola.</summary>
    public void Perform(in Circle2D C, in IntRes2d_Domain DC, in Hyperbola2D H, in IntRes2d_Domain DH, double TolConf,
                        double Tol)
    {
        this.ResetFields();
        IntCurve_IConicTool ITool = new(C);
        IntCurve_PConic PCurve = new(H);
        PCurve.SetAccuracy(20);
        Inter.SetReversedParameters(ReversedParameters());
        IntRes2d_Domain D = DC;

        if (!DC.IsClosed())
        {
            D.SetEquivalentParameters(DC.FirstParameter(), DC.FirstParameter() + M_PI + M_PI);
        }
        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D Pntinf = new(), Pntsup = new();
        maxtol = C.Radius() / 10.0;
        Vector2D Offset = new(maxtol * H.XAxis().Direction().X(), maxtol * H.XAxis().Direction().Y());
        Hyperbola2D Hp = H.Translated(Offset);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(Hp, new Conic(C));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, H, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        Offset.Reverse();
        Hyperbola2D Hm = H.Translated(Offset);
        theIntAna2d.Perform(Hm, new Conic(C));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, H, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(DH))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(DH, ref binf, ref tolinf, ref Pntinf, ref bsup, ref tolsup, ref Pntsup))
                {
                    IntRes2d_Domain DHModif = new(Pntinf, binf, tolinf, Pntsup, bsup, tolsup);
                    Inter.Perform(ITool, D, PCurve, DHModif, TolConf, Tol);
                }
                else
                {
                    done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < DH.FirstParameter())
                {
                    binf = DH.FirstParameter();
                    Pntinf = DH.FirstPoint();
                    ft = DH.FirstTolerance();
                    if (bsup < DH.FirstParameter())
                    {
                        done = true;
                        return;
                    }
                }
                if (bsup > DH.LastParameter())
                {
                    bsup = DH.LastParameter();
                    Pntsup = DH.LastPoint();
                    lt = DH.LastTolerance();
                    if (binf > DH.LastParameter())
                    {
                        done = true;
                        return;
                    }
                }
                IntRes2d_Domain DHModif = new(Pntinf, binf, ft, Pntsup, bsup, lt);
                Inter.Perform(ITool, D, PCurve, DHModif, TolConf, Tol);
            }
            this.SetValues(Inter);
        }
        else
        {
            done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between two parabolas.</summary>
    public void Perform(in Parabola2D P1, in IntRes2d_Domain DP1, in Parabola2D P2, in IntRes2d_Domain DP2,
                        double TolConf, double Tol)
    {
        this.ResetFields();
        IntCurve_IConicTool ITool = new(P1);
        IntCurve_PConic PCurve = new(P2);
        PCurve.SetAccuracy(20);
        Inter.SetReversedParameters(ReversedParameters());

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D Pntinf = new(), Pntsup = new();
        if (Tol > TolConf)
        {
            maxtol = Tol;
        }
        else
        {
            maxtol = TolConf;
        }
        maxtol *= 100.0;
        if (maxtol < 0.000001)
        {
            maxtol = 0.000001;
        }
        Vector2D Offset = new(maxtol * P2.MirrorAxis().Direction().X(), maxtol * P2.MirrorAxis().Direction().Y());
        Parabola2D Pp = P2.Translated(Offset);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(Pp, new Conic(P1));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, P2, maxtol,
                                PARAM_MAX_ON_PARABOLA);
        Offset.Reverse();
        Parabola2D Pm = P2.Translated(Offset);
        theIntAna2d.Perform(Pm, new Conic(P1));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, P2, maxtol,
                                PARAM_MAX_ON_PARABOLA);
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(DP2))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(DP2, ref binf, ref tolinf, ref Pntinf, ref bsup, ref tolsup, ref Pntsup))
                {
                    IntRes2d_Domain DPModif = new(Pntinf, binf, tolinf, Pntsup, bsup, tolsup);
                    Inter.Perform(ITool, DP1, PCurve, DPModif, TolConf, Tol);
                }
                else
                {
                    done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < DP2.FirstParameter())
                {
                    binf = DP2.FirstParameter();
                    Pntinf = DP2.FirstPoint();
                    ft = DP2.FirstTolerance();
                    if (bsup < DP2.FirstParameter())
                    {
                        done = true;
                        return;
                    }
                }
                if (bsup > DP2.LastParameter())
                {
                    bsup = DP2.LastParameter();
                    Pntsup = DP2.LastPoint();
                    lt = DP2.LastTolerance();
                    if (binf > DP2.LastParameter())
                    {
                        done = true;
                        return;
                    }
                }
                IntRes2d_Domain DPModif = new(Pntinf, binf, ft, Pntsup, bsup, lt);
                Inter.Perform(ITool, DP1, PCurve, DPModif, TolConf, Tol);
            }
            this.SetValues(Inter);
        }
        else
        {
            done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between an ellipse and a parabola.</summary>
    public void Perform(in Ellipse2D E, in IntRes2d_Domain DE, in Parabola2D P, in IntRes2d_Domain DP,
                        double TolConf, double Tol)
    {
        this.ResetFields();
        IntCurve_IConicTool ITool = new(E);
        IntCurve_PConic PCurve = new(P);
        PCurve.SetAccuracy(20);
        Inter.SetReversedParameters(ReversedParameters());

        IntRes2d_Domain D = DE;
        if (!DE.IsClosed())
        {
            D.SetEquivalentParameters(DE.FirstParameter(), DE.FirstParameter() + M_PI + M_PI);
        }

        //-- std::cout<<" Parab Elips "<<std::endl;

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D Pntinf = new(), Pntsup = new();
        if (Tol > TolConf)
        {
            maxtol = Tol;
        }
        else
        {
            maxtol = TolConf;
        }

        maxtol = E.MinorRadius() / 10.0;
        Ellipse2D Ep = E;
        Ep.SetMajorRadius(E.MajorRadius() + maxtol);
        Ep.SetMinorRadius(E.MinorRadius() + maxtol);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(P, new Conic(Ep));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, P, maxtol,
                                PARAM_MAX_ON_PARABOLA);
        if (E.MinorRadius() > maxtol)
        {
            Ep.SetMinorRadius(E.MinorRadius() - maxtol);
            Ep.SetMajorRadius(E.MajorRadius() - maxtol);
            theIntAna2d.Perform(P, new Conic(Ep));
            SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, P, maxtol,
                                    PARAM_MAX_ON_PARABOLA);
        }
        //-- std::cout<<" binf,bsup "<<binf<<" "<<bsup<<std::endl;
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(DP))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(DP, ref binf, ref tolinf, ref Pntinf, ref bsup, ref tolsup, ref Pntsup))
                {
                    IntRes2d_Domain DPModif = new(Pntinf, binf, tolinf, Pntsup, bsup, tolsup);
                    Inter.Perform(ITool, D, PCurve, DPModif, TolConf, Tol);
                }
                else
                {
                    done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < DP.FirstParameter())
                {
                    binf = DP.FirstParameter();
                    Pntinf = DP.FirstPoint();
                    ft = DP.FirstTolerance();
                    if (bsup < DP.FirstParameter())
                    {
                        done = true;
                        return;
                    }
                }
                if (bsup > DP.LastParameter())
                {
                    bsup = DP.LastParameter();
                    Pntsup = DP.LastPoint();
                    lt = DP.LastTolerance();
                    if (binf > DP.LastParameter())
                    {
                        done = true;
                        return;
                    }
                }
                //-- std::cout<<"BOUNDED  binf,bsup "<<binf<<" "<<bsup<<std::endl;
                //-- std::cout<<"Domain   "<<DP.FirstParameter()<<" "<<DP.LastParameter()<<std::endl;
                IntRes2d_Domain DPModif = new(Pntinf, binf, ft, Pntsup, bsup, lt);
                Inter.Perform(ITool, D, PCurve, DPModif, TolConf, Tol);
            }
            this.SetValues(Inter);
        }
        else
        {
            done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between a parabola and a hyperbola.</summary>
    public void Perform(in Parabola2D P, in IntRes2d_Domain DP, in Hyperbola2D H, in IntRes2d_Domain DH,
                        double TolConf, double Tol)
    {
        this.ResetFields();
        IntCurve_IConicTool ITool = new(P);
        IntCurve_PConic PCurve = new(H);
        Inter.SetReversedParameters(ReversedParameters());

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D Pntinf = new(), Pntsup = new();
        if (Tol > TolConf)
        {
            maxtol = Tol;
        }
        else
        {
            maxtol = TolConf;
        }
        maxtol *= 100.0;
        if (maxtol < 0.000001)
        {
            maxtol = 0.000001;
        }
        Vector2D Offset = new(maxtol * H.XAxis().Direction().X(), maxtol * H.XAxis().Direction().Y());
        Hyperbola2D Hp = H.Translated(Offset);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(Hp, new Conic(P));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, H, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        Offset.Reverse();
        Hyperbola2D Hm = H.Translated(Offset);
        theIntAna2d.Perform(Hm, new Conic(P));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, H, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(DH))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(DH, ref binf, ref tolinf, ref Pntinf, ref bsup, ref tolsup, ref Pntsup))
                {
                    IntRes2d_Domain DHModif = new(Pntinf, binf, tolinf, Pntsup, bsup, tolsup);
                    Inter.Perform(ITool, DP, PCurve, DHModif, TolConf, Tol);
                }
                else
                {
                    done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < DH.FirstParameter())
                {
                    binf = DH.FirstParameter();
                    Pntinf = DH.FirstPoint();
                    ft = DH.FirstTolerance();
                    if (bsup < DH.FirstParameter())
                    {
                        done = true;
                        return;
                    }
                }
                if (bsup > DH.LastParameter())
                {
                    bsup = DH.LastParameter();
                    Pntsup = DH.LastPoint();
                    lt = DH.LastTolerance();
                    if (binf > DH.LastParameter())
                    {
                        done = true;
                        return;
                    }
                }
                IntRes2d_Domain DHModif = new(Pntinf, binf, ft, Pntsup, bsup, lt);
                Inter.Perform(ITool, DP, PCurve, DHModif, TolConf, Tol);
            }
            this.SetValues(Inter);
        }
        else
        {
            done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between two ellipses.</summary>
    public void Perform(in Ellipse2D E1, in IntRes2d_Domain DE1, in Ellipse2D E2, in IntRes2d_Domain DE2,
                        double TolConf, double Tol)
    {
        this.ResetFields();
        IntCurve_IConicTool ITool = new(E1);
        IntCurve_PConic PCurve = new(E2);
        PCurve.SetAccuracy(20);

        Inter.SetReversedParameters(ReversedParameters());

        if (!DE1.IsClosed())
        {
            IntRes2d_Domain D1 = DE1;
            D1.SetEquivalentParameters(DE1.FirstParameter(), DE1.FirstParameter() + M_PI + M_PI);
            if (!DE2.IsClosed())
            {
                IntRes2d_Domain D2 = DE2;
                D2.SetEquivalentParameters(DE2.FirstParameter(), DE2.FirstParameter() + M_PI + M_PI);
                Inter.Perform(ITool, D1, PCurve, D2, TolConf, Tol);
            }
            else
            {
                Inter.Perform(ITool, D1, PCurve, DE2, TolConf, Tol);
            }
        }
        else
        {
            if (!DE2.IsClosed())
            {
                IntRes2d_Domain D2 = DE2;
                D2.SetEquivalentParameters(DE2.FirstParameter(), DE2.FirstParameter() + M_PI + M_PI);
                Inter.Perform(ITool, DE1, PCurve, D2, TolConf, Tol);
            }
            else
            {
                Inter.Perform(ITool, DE1, PCurve, DE2, TolConf, Tol);
            }
        }
        this.SetValues(Inter);
    }

    //=================================================================================================

    /// <summary>Intersection between an ellipse and a hyperbola.</summary>
    public void Perform(in Ellipse2D E, in IntRes2d_Domain DE, in Hyperbola2D H, in IntRes2d_Domain DH,
                        double TolConf, double Tol)
    {
        this.ResetFields();
        IntCurve_IConicTool ITool = new(E);
        IntCurve_PConic PCurve = new(H);
        PCurve.SetAccuracy(20);
        Inter.SetReversedParameters(ReversedParameters());
        IntRes2d_Domain DEModif = DE;

        if (!DE.IsClosed())
        {
            DEModif.SetEquivalentParameters(DE.FirstParameter(), DE.FirstParameter() + M_PI + M_PI);
        }

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D Pntinf = new(), Pntsup = new();
        maxtol = E.MinorRadius() / 10.0;
        Vector2D Offset = new(maxtol * H.XAxis().Direction().X(), maxtol * H.XAxis().Direction().Y());
        Hyperbola2D Hp = H.Translated(Offset);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(Hp, new Conic(E));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, H, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        Offset.Reverse();
        Hyperbola2D Hm = H.Translated(Offset);
        theIntAna2d.Perform(Hm, new Conic(E));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, H, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(DH))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(DH, ref binf, ref tolinf, ref Pntinf, ref bsup, ref tolsup, ref Pntsup))
                {
                    IntRes2d_Domain DHModif = new(Pntinf, binf, tolinf, Pntsup, bsup, tolsup);
                    Inter.Perform(ITool, DEModif, PCurve, DHModif, TolConf, Tol);
                }
                else
                {
                    done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < DH.FirstParameter())
                {
                    binf = DH.FirstParameter();
                    Pntinf = DH.FirstPoint();
                    ft = DH.FirstTolerance();
                    if (bsup < DH.FirstParameter())
                    {
                        done = true;
                        return;
                    }
                }
                if (bsup > DH.LastParameter())
                {
                    bsup = DH.LastParameter();
                    Pntsup = DH.LastPoint();
                    lt = DH.LastTolerance();
                    if (binf > DH.LastParameter())
                    {
                        done = true;
                        return;
                    }
                }
                if (binf >= bsup)
                {
                    done = true;
                    return;
                }
                IntRes2d_Domain DHModif = new(Pntinf, binf, ft, Pntsup, bsup, lt);
                Inter.Perform(ITool, DEModif, PCurve, DHModif, TolConf, Tol);
            }
            this.SetValues(Inter);
        }
        else
        {
            done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between two hyperbolas.</summary>
    public void Perform(in Hyperbola2D H1, in IntRes2d_Domain DH1, in Hyperbola2D H2, in IntRes2d_Domain DH2,
                        double TolConf, double Tol)
    {
        this.ResetFields();
        IntCurve_IConicTool ITool = new(H1);
        IntCurve_PConic PCurve = new(H2);
        PCurve.SetAccuracy(20);

        Inter.SetReversedParameters(ReversedParameters());

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D Pntinf = new(), Pntsup = new();
        if (Tol > TolConf)
        {
            maxtol = Tol;
        }
        else
        {
            maxtol = TolConf;
        }
        maxtol *= 100.0;
        if (maxtol < 0.000001)
        {
            maxtol = 0.000001;
        }
        Vector2D Offset = new(maxtol * H2.XAxis().Direction().X(), maxtol * H2.XAxis().Direction().Y());
        Hyperbola2D Hp = H2.Translated(Offset);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(Hp, new Conic(H1));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, H2, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        Offset.Reverse();
        Hyperbola2D Hm = H2.Translated(Offset);
        theIntAna2d.Perform(Hm, new Conic(H1));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref Pntinf, ref bsup, ref Pntsup, H2, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(DH2))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(DH2, ref binf, ref tolinf, ref Pntinf, ref bsup, ref tolsup, ref Pntsup))
                {
                    IntRes2d_Domain DHModif = new(Pntinf, binf, tolinf, Pntsup, bsup, tolsup);
                    Inter.Perform(ITool, DH1, PCurve, DHModif, TolConf, Tol);
                }
                else
                {
                    done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < DH2.FirstParameter())
                {
                    binf = DH2.FirstParameter();
                    Pntinf = DH2.FirstPoint();
                    ft = DH2.FirstTolerance();
                }
                if (bsup > DH2.LastParameter())
                {
                    bsup = DH2.LastParameter();
                    Pntsup = DH2.LastPoint();
                    lt = DH2.LastTolerance();
                }

                //-- lbr le 22 sept 97
                //-- binf=-5.6 bsup=-5.3 DH2=(5.6,5.7) -> binf=+5.6 bsup=-5.3
                if (binf >= bsup)
                {
                    done = true;
                    return;
                }
                IntRes2d_Domain DHModif = new(Pntinf, binf, ft, Pntsup, bsup, lt);
                Inter.Perform(ITool, DH1, PCurve, DHModif, TolConf, Tol);
            }
            this.SetValues(Inter);
        }
        else
        {
            done = true;
        }
    }

    //----------------------------------------------------------------------

    private static void SetBinfBsupFromIntAna2d(AnalyticIntersection theIntAna2d,
                                                ref double binf,
                                                ref Point2D Pntinf,
                                                ref double bsup,
                                                ref Point2D Pntsup,
                                                in Parabola2D PR,
                                                double maxtol,
                                                double LIMITE)
    {
        if (theIntAna2d.IsDone)
        {
            if (!theIntAna2d.IsEmpty)
            {
                for (int p = 1; p <= theIntAna2d.PointCount; p++)
                {
                    double param = theIntAna2d.Point(p).ParamOnFirst();

                    if (Math.Abs(param) < LIMITE)
                    {
                        global::Kerf.Engine.Geometry.ElCLib.D1(param, PR, out _, out Vector2D V);
                        double NormeD1 = V.Magnitude();
                        double dparam = 100.0 * maxtol / NormeD1;
                        if (dparam < 1e-3)
                        {
                            dparam = 1e-3;
                        }
                        param -= dparam;

                        if (param < binf)
                        {
                            binf = param;
                            Pntinf = global::Kerf.Engine.Geometry.ElCLib.Value(param, PR);
                        }
                        param += dparam + dparam;
                        if (param > bsup)
                        {
                            bsup = param;
                            Pntsup = global::Kerf.Engine.Geometry.ElCLib.Value(param, PR);
                        }
                    }
                }
            }
        }
    }

    private static void SetBinfBsupFromIntAna2d(AnalyticIntersection theIntAna2d,
                                                ref double binf,
                                                ref Point2D Pntinf,
                                                ref double bsup,
                                                ref Point2D Pntsup,
                                                in Hyperbola2D H,
                                                double maxtol,
                                                double LIMITE)
    {
        if (theIntAna2d.IsDone)
        {
            if (!theIntAna2d.IsEmpty)
            {
                for (int p = 1; p <= theIntAna2d.PointCount; p++)
                {
                    double param = theIntAna2d.Point(p).ParamOnFirst();

                    if (Math.Abs(param) < LIMITE)
                    {
                        global::Kerf.Engine.Geometry.ElCLib.D1(param, H, out _, out Vector2D V);
                        double NormeD1 = V.Magnitude();
                        double dparam = 100.0 * maxtol / NormeD1;
                        if (dparam < 1e-3)
                        {
                            dparam = 1e-3;
                        }
                        param -= dparam;

                        if (param < binf)
                        {
                            binf = param;
                            Pntinf = global::Kerf.Engine.Geometry.ElCLib.Value(param, H);
                        }
                        param += dparam + dparam;
                        if (param > bsup)
                        {
                            bsup = param;
                            Pntsup = global::Kerf.Engine.Geometry.ElCLib.Value(param, H);
                        }
                    }
                }
            }
        }
    }
}
