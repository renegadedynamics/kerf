// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicConic.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicConic.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicConic.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>
/// The intersection of two conics: lines, circles, ellipses, parabolas and hyperbolas, with domains.
/// </summary>
/// <remarks>
/// Circles and ellipses must have closed domains (<see cref="TwoDDomain.WithEquivalentParameters"/>); where
/// they don't, most methods close them on [first, first + 2 PI]. Line/line, line/circle, circle/circle and
/// line/ellipse are solved geometrically (the second file, IntCurve_IntConicConic_1); the other pairs by
/// <see cref="IntImpConicParConic"/> on a parametric domain bounded around the analytic solutions.
/// </remarks>
public sealed partial class IntConicConic : Intersection2D
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

    private readonly IntImpConicParConic _inter = new();

    /// <summary>Empty constructor.</summary>
    public IntConicConic()
    {
    }

    /// <summary>Intersection between two lines.</summary>
    public IntConicConic(in Line2D l1, in TwoDDomain d1, in Line2D l2, in TwoDDomain d2,
                                  double tolConf, double tol)
    {
        Perform(l1, d1, l2, d2, tolConf, tol);
    }

    /// <summary>Intersection between a line and a circle.</summary>
    public IntConicConic(in Line2D l1, in TwoDDomain d1, in Circle2D c2, in TwoDDomain d2,
                                  double tolConf, double tol)
    {
        Perform(l1, d1, c2, d2, tolConf, tol);
    }

    /// <summary>Intersection between a line and an ellipse.</summary>
    public IntConicConic(in Line2D l, in TwoDDomain dl, in Ellipse2D e, in TwoDDomain de,
                                  double tolConf, double tol)
    {
        Perform(l, dl, e, de, tolConf, tol);
    }

    /// <summary>Intersection between a line and a parabola.</summary>
    public IntConicConic(in Line2D l, in TwoDDomain dl, in Parabola2D p, in TwoDDomain dp,
                                  double tolConf, double tol)
    {
        Perform(l, dl, p, dp, tolConf, tol);
    }

    /// <summary>Intersection between a line and a hyperbola.</summary>
    public IntConicConic(in Line2D l, in TwoDDomain dl, in Hyperbola2D h, in TwoDDomain dh,
                                  double tolConf, double tol)
    {
        Perform(l, dl, h, dh, tolConf, tol);
    }

    /// <summary>Intersection between two circles.</summary>
    public IntConicConic(in Circle2D c1, in TwoDDomain d1, in Circle2D c2, in TwoDDomain d2,
                                  double tolConf, double tol)
    {
        SetReversedParameters(false);
        Perform(c1, d1, c2, d2, tolConf, tol);
    }

    /// <summary>Intersection between a circle and an ellipse.</summary>
    public IntConicConic(in Circle2D c, in TwoDDomain dc, in Ellipse2D e, in TwoDDomain de,
                                  double tolConf, double tol)
    {
        Perform(c, dc, e, de, tolConf, tol);
    }

    /// <summary>Intersection between a circle and a parabola.</summary>
    public IntConicConic(in Circle2D c, in TwoDDomain dc, in Parabola2D p, in TwoDDomain dp,
                                  double tolConf, double tol)
    {
        Perform(c, dc, p, dp, tolConf, tol);
    }

    /// <summary>Intersection between a circle and a hyperbola.</summary>
    public IntConicConic(in Circle2D c, in TwoDDomain dc, in Hyperbola2D h, in TwoDDomain dh,
                                  double tolConf, double tol)
    {
        Perform(c, dc, h, dh, tolConf, tol);
    }

    /// <summary>Intersection between two ellipses.</summary>
    public IntConicConic(in Ellipse2D e1, in TwoDDomain d1, in Ellipse2D e2, in TwoDDomain d2,
                                  double tolConf, double tol)
    {
        Perform(e1, d1, e2, d2, tolConf, tol);
    }

    /// <summary>Intersection between an ellipse and a parabola.</summary>
    public IntConicConic(in Ellipse2D e, in TwoDDomain de, in Parabola2D p, in TwoDDomain dp,
                                  double tolConf, double tol)
    {
        Perform(e, de, p, dp, tolConf, tol);
    }

    /// <summary>Intersection between an ellipse and a hyperbola.</summary>
    public IntConicConic(in Ellipse2D e, in TwoDDomain de, in Hyperbola2D h, in TwoDDomain dh,
                                  double tolConf, double tol)
    {
        Perform(e, de, h, dh, tolConf, tol);
    }

    /// <summary>Intersection between two parabolas.</summary>
    public IntConicConic(in Parabola2D p1, in TwoDDomain d1, in Parabola2D p2, in TwoDDomain d2,
                                  double tolConf, double tol)
    {
        Perform(p1, d1, p2, d2, tolConf, tol);
    }

    /// <summary>Intersection between a parabola and a hyperbola.</summary>
    public IntConicConic(in Parabola2D p, in TwoDDomain dp, in Hyperbola2D h, in TwoDDomain dh,
                                  double tolConf, double tol)
    {
        Perform(p, dp, h, dh, tolConf, tol);
    }

    /// <summary>Intersection between two hyperbolas.</summary>
    public IntConicConic(in Hyperbola2D h1, in TwoDDomain dh1, in Hyperbola2D h2, in TwoDDomain dh2,
                                  double tolConf, double tol)
    {
        Perform(h1, dh1, h2, dh2, tolConf, tol);
    }

    private static bool BOUNDED_DOMAIN(in TwoDDomain domain)
    {
        return (domain.HasFirstPoint && domain.HasLastPoint);
    }

    private static bool SET_BOUNDED_DOMAIN(in TwoDDomain domain,
                                           ref double binf,
                                           ref double tolinf,
                                           ref Point2D pntinf,
                                           ref double bsup,
                                           ref double tolsup,
                                           ref Point2D pntsup)
    {
        if (domain.HasFirstPoint)
        {
            if (binf < domain.FirstParameter())
            {
                pntinf = domain.FirstPoint();
                binf = domain.FirstParameter();
                tolinf = domain.FirstTolerance();
            }
        }
        if (domain.HasLastPoint)
        {
            // OCCT compares bsup with the domain's first parameter, not its last one (as is).
            if (bsup > domain.FirstParameter())
            {
                pntsup = domain.LastPoint();
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
    public void Perform(in Line2D l, in TwoDDomain dl, in Parabola2D p, in TwoDDomain dp, double tolConf,
                        double tol)
    {
        this.ResetFields();
        IConicTool iTool = new(l);
        PConic pCurve = new(p);

        pCurve.SetAccuracy(20);
        _inter.SetReversedParameters(ReversedParameters());

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite();
        double bsup = -global::Kerf.Engine.Foundation.Precision.Infinite();
        double maxtol = (tol > tolConf) ? tol : tolConf;
        if (maxtol < 1.0e-7)
        {
            maxtol = 1.0e-7;
        }
        bool wasSet = false;

        Point2D pntinf = new(), pntsup = new();
        AnalyticIntersection theIntAna2d = new();

        maxtol *= 100.0;
        //  if(maxtol<0.000001) maxtol = 0.000001;

        // OCCT offsets the line by (maxtol Dy, maxtol Dx), which is not normal to it (as is).
        Vector2D offset = new(maxtol * l.Direction().Y(), maxtol * l.Direction().X());
        Line2D lp = l.Translated(offset);
        theIntAna2d.Perform(p, new Conic(lp));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, p, maxtol,
                                PARAM_MAX_ON_PARABOLA);

        offset = offset.Reversed();
        Line2D lm = l.Translated(offset);
        theIntAna2d.Perform(p, new Conic(lm));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, p, maxtol,
                                PARAM_MAX_ON_PARABOLA);

        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(dp))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(dp, ref binf, ref tolinf, ref pntinf, ref bsup, ref tolsup, ref pntsup))
                {
                    TwoDDomain dpModif = new(pntinf, binf, tolinf, pntsup, bsup, tolsup);
                    _inter.Perform(iTool, dl, pCurve, dpModif, tolConf, tol);
                }
                else
                {
                    _done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < dp.FirstParameter())
                {
                    binf = dp.FirstParameter();
                    pntinf = dp.FirstPoint();
                    ft = dp.FirstTolerance();
                    if (bsup < dp.FirstParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                if (bsup > dp.LastParameter())
                {
                    bsup = dp.LastParameter();
                    pntsup = dp.LastPoint();
                    lt = dp.LastTolerance();
                    if (binf > dp.LastParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                TwoDDomain dpModif = new(pntinf, binf, ft, pntsup, bsup, lt);
                //      Inter.Perform(ITool,DL,PCurve,DPModif,TolConf,Tol);
                _inter.Perform(iTool, dl, pCurve, dpModif, TOL_EXACT_INTER, TOL_EXACT_INTER);
                this.SetValues(_inter);
                wasSet = true;
                if (this.IsDone && this.PointCount == 0)
                {
                    this.ResetFields();
                    _inter.Perform(iTool, dl, pCurve, dpModif, tolConf, tol);
                    wasSet = false;
                }
            }
            if (!wasSet)
            {
                this.SetValues(_inter);
            }
        }
        else
        {
            _done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between a line and a hyperbola.</summary>
    public void Perform(in Line2D l, in TwoDDomain dl, in Hyperbola2D h, in TwoDDomain dh, double tolConf,
                        double tol)
    {
        this.ResetFields();
        IConicTool iTool = new(l);
        PConic pCurve = new(h);
        pCurve.SetAccuracy(20);

        _inter.SetReversedParameters(ReversedParameters());

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D pntinf = new(), pntsup = new();
        if (tol > tolConf)
        {
            maxtol = tol;
        }
        else
        {
            maxtol = tolConf;
        }
        maxtol *= 100.0;
        if (maxtol < 0.000001)
        {
            maxtol = 0.000001;
        }
        Vector2D offset = new(maxtol * h.XAxis().Direction().X(), maxtol * h.XAxis().Direction().Y());
        Hyperbola2D hp = h.Translated(offset);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(hp, new Conic(l));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, h, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        offset = offset.Reversed();
        Hyperbola2D hm = h.Translated(offset);
        theIntAna2d.Perform(hm, new Conic(l));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, h, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(dh))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(dh, ref binf, ref tolinf, ref pntinf, ref bsup, ref tolsup, ref pntsup))
                {
                    TwoDDomain dhModif = new(pntinf, binf, tolinf, pntsup, bsup, tolsup);
                    _inter.Perform(iTool, dl, pCurve, dhModif, tolConf, tol);
                }
                else
                {
                    _done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < dh.FirstParameter())
                {
                    binf = dh.FirstParameter();
                    pntinf = dh.FirstPoint();
                    ft = dh.FirstTolerance();
                    if (bsup < dh.FirstParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                if (bsup > dh.LastParameter())
                {
                    bsup = dh.LastParameter();
                    pntsup = dh.LastPoint();
                    lt = dh.LastTolerance();
                    if (binf > dh.LastParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                TwoDDomain dhModif = new(pntinf, binf, ft, pntsup, bsup, lt);
                _inter.Perform(iTool, dl, pCurve, dhModif, tolConf, tol);
            }
            this.SetValues(_inter);
        }
        else
        {
            _done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between a circle and a parabola.</summary>
    public void Perform(in Circle2D c, in TwoDDomain dc, in Parabola2D p, in TwoDDomain dp, double tolConf,
                        double tol)
    {
        this.ResetFields();
        IConicTool iTool = new(c);
        PConic pCurve = new(p);
        pCurve.SetAccuracy(20);
        _inter.SetReversedParameters(ReversedParameters());

        TwoDDomain d = dc;
        if (!dc.IsClosed)
        {
            d.SetEquivalentParameters(dc.FirstParameter(), dc.FirstParameter() + M_PI + M_PI);
        }

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D pntinf = new(), pntsup = new();
        maxtol = c.Radius() / 10.0;
        Circle2D cp = c;
        cp = cp.WithRadius(c.Radius() + maxtol);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(p, new Conic(cp));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, p, maxtol,
                                PARAM_MAX_ON_PARABOLA);
        if (c.Radius() > maxtol)
        {
            cp = cp.WithRadius(c.Radius() - maxtol);
            theIntAna2d.Perform(p, new Conic(cp));
            SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, p, maxtol,
                                    PARAM_MAX_ON_PARABOLA);
        }
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(dp))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(dp, ref binf, ref tolinf, ref pntinf, ref bsup, ref tolsup, ref pntsup))
                {
                    TwoDDomain dpModif = new(pntinf, binf, tolinf, pntsup, bsup, tolsup);
                    _inter.Perform(iTool, d, pCurve, dpModif, tolConf, tol);
                }
                else
                {
                    _done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < dp.FirstParameter())
                {
                    binf = dp.FirstParameter();
                    pntinf = dp.FirstPoint();
                    ft = dp.FirstTolerance();
                    if (bsup < dp.FirstParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                if (bsup > dp.LastParameter())
                {
                    bsup = dp.LastParameter();
                    pntsup = dp.LastPoint();
                    lt = dp.LastTolerance();
                    if (binf > dp.LastParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                TwoDDomain dpModif = new(pntinf, binf, ft, pntsup, bsup, lt);
                _inter.Perform(iTool, d, pCurve, dpModif, tolConf, tol);
            }
            this.SetValues(_inter);
        }
        else
        {
            _done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between a circle and an ellipse.</summary>
    public void Perform(in Circle2D c, in TwoDDomain dc, in Ellipse2D e, in TwoDDomain de, double tolConf,
                        double tol)
    {
        this.ResetFields();
        IConicTool iTool = new(c);
        PConic pCurve = new(e);
        pCurve.SetAccuracy(20);

        _inter.SetReversedParameters(ReversedParameters());

        if (!dc.IsClosed)
        {
            TwoDDomain d1 = dc;
            d1.SetEquivalentParameters(dc.FirstParameter(), dc.FirstParameter() + M_PI + M_PI);
            if (!de.IsClosed)
            {
                TwoDDomain d2 = de;
                d2.SetEquivalentParameters(de.FirstParameter(), de.FirstParameter() + M_PI + M_PI);
                _inter.Perform(iTool, d1, pCurve, d2, tolConf, tol);
            }
            else
            {
                _inter.Perform(iTool, d1, pCurve, de, tolConf, tol);
            }
        }
        else
        {
            if (!de.IsClosed)
            {
                TwoDDomain d2 = de;
                d2.SetEquivalentParameters(de.FirstParameter(), de.FirstParameter() + M_PI + M_PI);
                _inter.Perform(iTool, dc, pCurve, d2, tolConf, tol);
            }
            else
            {
                _inter.Perform(iTool, dc, pCurve, de, tolConf, tol);
            }
        }
        this.SetValues(_inter);
    }

    //=================================================================================================

    /// <summary>Intersection between a circle and a hyperbola.</summary>
    public void Perform(in Circle2D c, in TwoDDomain dc, in Hyperbola2D h, in TwoDDomain dh, double tolConf,
                        double tol)
    {
        this.ResetFields();
        IConicTool iTool = new(c);
        PConic pCurve = new(h);
        pCurve.SetAccuracy(20);
        _inter.SetReversedParameters(ReversedParameters());
        TwoDDomain d = dc;

        if (!dc.IsClosed)
        {
            d.SetEquivalentParameters(dc.FirstParameter(), dc.FirstParameter() + M_PI + M_PI);
        }
        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D pntinf = new(), pntsup = new();
        maxtol = c.Radius() / 10.0;
        Vector2D offset = new(maxtol * h.XAxis().Direction().X(), maxtol * h.XAxis().Direction().Y());
        Hyperbola2D hp = h.Translated(offset);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(hp, new Conic(c));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, h, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        offset = offset.Reversed();
        Hyperbola2D hm = h.Translated(offset);
        theIntAna2d.Perform(hm, new Conic(c));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, h, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(dh))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(dh, ref binf, ref tolinf, ref pntinf, ref bsup, ref tolsup, ref pntsup))
                {
                    TwoDDomain dhModif = new(pntinf, binf, tolinf, pntsup, bsup, tolsup);
                    _inter.Perform(iTool, d, pCurve, dhModif, tolConf, tol);
                }
                else
                {
                    _done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < dh.FirstParameter())
                {
                    binf = dh.FirstParameter();
                    pntinf = dh.FirstPoint();
                    ft = dh.FirstTolerance();
                    if (bsup < dh.FirstParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                if (bsup > dh.LastParameter())
                {
                    bsup = dh.LastParameter();
                    pntsup = dh.LastPoint();
                    lt = dh.LastTolerance();
                    if (binf > dh.LastParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                TwoDDomain dhModif = new(pntinf, binf, ft, pntsup, bsup, lt);
                _inter.Perform(iTool, d, pCurve, dhModif, tolConf, tol);
            }
            this.SetValues(_inter);
        }
        else
        {
            _done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between two parabolas.</summary>
    public void Perform(in Parabola2D p1, in TwoDDomain dp1, in Parabola2D p2, in TwoDDomain dp2,
                        double tolConf, double tol)
    {
        this.ResetFields();
        IConicTool iTool = new(p1);
        PConic pCurve = new(p2);
        pCurve.SetAccuracy(20);
        _inter.SetReversedParameters(ReversedParameters());

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D pntinf = new(), pntsup = new();
        if (tol > tolConf)
        {
            maxtol = tol;
        }
        else
        {
            maxtol = tolConf;
        }
        maxtol *= 100.0;
        if (maxtol < 0.000001)
        {
            maxtol = 0.000001;
        }
        Vector2D offset = new(maxtol * p2.MirrorAxis().Direction().X(), maxtol * p2.MirrorAxis().Direction().Y());
        Parabola2D pp = p2.Translated(offset);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(pp, new Conic(p1));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, p2, maxtol,
                                PARAM_MAX_ON_PARABOLA);
        offset = offset.Reversed();
        Parabola2D pm = p2.Translated(offset);
        theIntAna2d.Perform(pm, new Conic(p1));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, p2, maxtol,
                                PARAM_MAX_ON_PARABOLA);
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(dp2))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(dp2, ref binf, ref tolinf, ref pntinf, ref bsup, ref tolsup, ref pntsup))
                {
                    TwoDDomain dpModif = new(pntinf, binf, tolinf, pntsup, bsup, tolsup);
                    _inter.Perform(iTool, dp1, pCurve, dpModif, tolConf, tol);
                }
                else
                {
                    _done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < dp2.FirstParameter())
                {
                    binf = dp2.FirstParameter();
                    pntinf = dp2.FirstPoint();
                    ft = dp2.FirstTolerance();
                    if (bsup < dp2.FirstParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                if (bsup > dp2.LastParameter())
                {
                    bsup = dp2.LastParameter();
                    pntsup = dp2.LastPoint();
                    lt = dp2.LastTolerance();
                    if (binf > dp2.LastParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                TwoDDomain dpModif = new(pntinf, binf, ft, pntsup, bsup, lt);
                _inter.Perform(iTool, dp1, pCurve, dpModif, tolConf, tol);
            }
            this.SetValues(_inter);
        }
        else
        {
            _done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between an ellipse and a parabola.</summary>
    public void Perform(in Ellipse2D e, in TwoDDomain de, in Parabola2D p, in TwoDDomain dp,
                        double tolConf, double tol)
    {
        this.ResetFields();
        IConicTool iTool = new(e);
        PConic pCurve = new(p);
        pCurve.SetAccuracy(20);
        _inter.SetReversedParameters(ReversedParameters());

        TwoDDomain d = de;
        if (!de.IsClosed)
        {
            d.SetEquivalentParameters(de.FirstParameter(), de.FirstParameter() + M_PI + M_PI);
        }

        //-- std::cout<<" Parab Elips "<<std::endl;

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D pntinf = new(), pntsup = new();
        if (tol > tolConf)
        {
            maxtol = tol;
        }
        else
        {
            maxtol = tolConf;
        }

        maxtol = e.MinorRadius() / 10.0;
        Ellipse2D ep = e;
        ep = ep.WithMajorRadius(e.MajorRadius() + maxtol);
        ep = ep.WithMinorRadius(e.MinorRadius() + maxtol);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(p, new Conic(ep));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, p, maxtol,
                                PARAM_MAX_ON_PARABOLA);
        if (e.MinorRadius() > maxtol)
        {
            ep = ep.WithMinorRadius(e.MinorRadius() - maxtol);
            ep = ep.WithMajorRadius(e.MajorRadius() - maxtol);
            theIntAna2d.Perform(p, new Conic(ep));
            SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, p, maxtol,
                                    PARAM_MAX_ON_PARABOLA);
        }
        //-- std::cout<<" binf,bsup "<<binf<<" "<<bsup<<std::endl;
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(dp))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(dp, ref binf, ref tolinf, ref pntinf, ref bsup, ref tolsup, ref pntsup))
                {
                    TwoDDomain dpModif = new(pntinf, binf, tolinf, pntsup, bsup, tolsup);
                    _inter.Perform(iTool, d, pCurve, dpModif, tolConf, tol);
                }
                else
                {
                    _done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < dp.FirstParameter())
                {
                    binf = dp.FirstParameter();
                    pntinf = dp.FirstPoint();
                    ft = dp.FirstTolerance();
                    if (bsup < dp.FirstParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                if (bsup > dp.LastParameter())
                {
                    bsup = dp.LastParameter();
                    pntsup = dp.LastPoint();
                    lt = dp.LastTolerance();
                    if (binf > dp.LastParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                //-- std::cout<<"BOUNDED  binf,bsup "<<binf<<" "<<bsup<<std::endl;
                //-- std::cout<<"Domain   "<<DP.FirstParameter()<<" "<<DP.LastParameter()<<std::endl;
                TwoDDomain dpModif = new(pntinf, binf, ft, pntsup, bsup, lt);
                _inter.Perform(iTool, d, pCurve, dpModif, tolConf, tol);
            }
            this.SetValues(_inter);
        }
        else
        {
            _done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between a parabola and a hyperbola.</summary>
    public void Perform(in Parabola2D p, in TwoDDomain dp, in Hyperbola2D h, in TwoDDomain dh,
                        double tolConf, double tol)
    {
        this.ResetFields();
        IConicTool iTool = new(p);
        PConic pCurve = new(h);
        _inter.SetReversedParameters(ReversedParameters());

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D pntinf = new(), pntsup = new();
        if (tol > tolConf)
        {
            maxtol = tol;
        }
        else
        {
            maxtol = tolConf;
        }
        maxtol *= 100.0;
        if (maxtol < 0.000001)
        {
            maxtol = 0.000001;
        }
        Vector2D offset = new(maxtol * h.XAxis().Direction().X(), maxtol * h.XAxis().Direction().Y());
        Hyperbola2D hp = h.Translated(offset);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(hp, new Conic(p));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, h, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        offset = offset.Reversed();
        Hyperbola2D hm = h.Translated(offset);
        theIntAna2d.Perform(hm, new Conic(p));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, h, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(dh))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(dh, ref binf, ref tolinf, ref pntinf, ref bsup, ref tolsup, ref pntsup))
                {
                    TwoDDomain dhModif = new(pntinf, binf, tolinf, pntsup, bsup, tolsup);
                    _inter.Perform(iTool, dp, pCurve, dhModif, tolConf, tol);
                }
                else
                {
                    _done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < dh.FirstParameter())
                {
                    binf = dh.FirstParameter();
                    pntinf = dh.FirstPoint();
                    ft = dh.FirstTolerance();
                    if (bsup < dh.FirstParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                if (bsup > dh.LastParameter())
                {
                    bsup = dh.LastParameter();
                    pntsup = dh.LastPoint();
                    lt = dh.LastTolerance();
                    if (binf > dh.LastParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                TwoDDomain dhModif = new(pntinf, binf, ft, pntsup, bsup, lt);
                _inter.Perform(iTool, dp, pCurve, dhModif, tolConf, tol);
            }
            this.SetValues(_inter);
        }
        else
        {
            _done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between two ellipses.</summary>
    public void Perform(in Ellipse2D e1, in TwoDDomain de1, in Ellipse2D e2, in TwoDDomain de2,
                        double tolConf, double tol)
    {
        this.ResetFields();
        IConicTool iTool = new(e1);
        PConic pCurve = new(e2);
        pCurve.SetAccuracy(20);

        _inter.SetReversedParameters(ReversedParameters());

        if (!de1.IsClosed)
        {
            TwoDDomain d1 = de1;
            d1.SetEquivalentParameters(de1.FirstParameter(), de1.FirstParameter() + M_PI + M_PI);
            if (!de2.IsClosed)
            {
                TwoDDomain d2 = de2;
                d2.SetEquivalentParameters(de2.FirstParameter(), de2.FirstParameter() + M_PI + M_PI);
                _inter.Perform(iTool, d1, pCurve, d2, tolConf, tol);
            }
            else
            {
                _inter.Perform(iTool, d1, pCurve, de2, tolConf, tol);
            }
        }
        else
        {
            if (!de2.IsClosed)
            {
                TwoDDomain d2 = de2;
                d2.SetEquivalentParameters(de2.FirstParameter(), de2.FirstParameter() + M_PI + M_PI);
                _inter.Perform(iTool, de1, pCurve, d2, tolConf, tol);
            }
            else
            {
                _inter.Perform(iTool, de1, pCurve, de2, tolConf, tol);
            }
        }
        this.SetValues(_inter);
    }

    //=================================================================================================

    /// <summary>Intersection between an ellipse and a hyperbola.</summary>
    public void Perform(in Ellipse2D e, in TwoDDomain de, in Hyperbola2D h, in TwoDDomain dh,
                        double tolConf, double tol)
    {
        this.ResetFields();
        IConicTool iTool = new(e);
        PConic pCurve = new(h);
        pCurve.SetAccuracy(20);
        _inter.SetReversedParameters(ReversedParameters());
        TwoDDomain deModif = de;

        if (!de.IsClosed)
        {
            deModif.SetEquivalentParameters(de.FirstParameter(), de.FirstParameter() + M_PI + M_PI);
        }

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D pntinf = new(), pntsup = new();
        maxtol = e.MinorRadius() / 10.0;
        Vector2D offset = new(maxtol * h.XAxis().Direction().X(), maxtol * h.XAxis().Direction().Y());
        Hyperbola2D hp = h.Translated(offset);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(hp, new Conic(e));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, h, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        offset = offset.Reversed();
        Hyperbola2D hm = h.Translated(offset);
        theIntAna2d.Perform(hm, new Conic(e));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, h, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(dh))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(dh, ref binf, ref tolinf, ref pntinf, ref bsup, ref tolsup, ref pntsup))
                {
                    TwoDDomain dhModif = new(pntinf, binf, tolinf, pntsup, bsup, tolsup);
                    _inter.Perform(iTool, deModif, pCurve, dhModif, tolConf, tol);
                }
                else
                {
                    _done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < dh.FirstParameter())
                {
                    binf = dh.FirstParameter();
                    pntinf = dh.FirstPoint();
                    ft = dh.FirstTolerance();
                    if (bsup < dh.FirstParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                if (bsup > dh.LastParameter())
                {
                    bsup = dh.LastParameter();
                    pntsup = dh.LastPoint();
                    lt = dh.LastTolerance();
                    if (binf > dh.LastParameter())
                    {
                        _done = true;
                        return;
                    }
                }
                if (binf >= bsup)
                {
                    _done = true;
                    return;
                }
                TwoDDomain dhModif = new(pntinf, binf, ft, pntsup, bsup, lt);
                _inter.Perform(iTool, deModif, pCurve, dhModif, tolConf, tol);
            }
            this.SetValues(_inter);
        }
        else
        {
            _done = true;
        }
    }

    //=================================================================================================

    /// <summary>Intersection between two hyperbolas.</summary>
    public void Perform(in Hyperbola2D h1, in TwoDDomain dh1, in Hyperbola2D h2, in TwoDDomain dh2,
                        double tolConf, double tol)
    {
        this.ResetFields();
        IConicTool iTool = new(h1);
        PConic pCurve = new(h2);
        pCurve.SetAccuracy(20);

        _inter.SetReversedParameters(ReversedParameters());

        double binf = global::Kerf.Engine.Foundation.Precision.Infinite(), bsup = -global::Kerf.Engine.Foundation.Precision.Infinite(), maxtol;
        Point2D pntinf = new(), pntsup = new();
        if (tol > tolConf)
        {
            maxtol = tol;
        }
        else
        {
            maxtol = tolConf;
        }
        maxtol *= 100.0;
        if (maxtol < 0.000001)
        {
            maxtol = 0.000001;
        }
        Vector2D offset = new(maxtol * h2.XAxis().Direction().X(), maxtol * h2.XAxis().Direction().Y());
        Hyperbola2D hp = h2.Translated(offset);
        AnalyticIntersection theIntAna2d = new();
        theIntAna2d.Perform(hp, new Conic(h1));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, h2, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        offset = offset.Reversed();
        Hyperbola2D hm = h2.Translated(offset);
        theIntAna2d.Perform(hm, new Conic(h1));
        SetBinfBsupFromIntAna2d(theIntAna2d, ref binf, ref pntinf, ref bsup, ref pntsup, h2, maxtol,
                                PARAM_MAX_ON_HYPERBOLA);
        if (binf <= bsup)
        {
            if (!BOUNDED_DOMAIN(dh2))
            {
                double tolinf = 0.0;
                double tolsup = 0.0;
                if (SET_BOUNDED_DOMAIN(dh2, ref binf, ref tolinf, ref pntinf, ref bsup, ref tolsup, ref pntsup))
                {
                    TwoDDomain dhModif = new(pntinf, binf, tolinf, pntsup, bsup, tolsup);
                    _inter.Perform(iTool, dh1, pCurve, dhModif, tolConf, tol);
                }
                else
                {
                    _done = true;
                    return;
                }
            }
            else
            {
                double ft = 0.0, lt = 0.0;
                if (binf < dh2.FirstParameter())
                {
                    binf = dh2.FirstParameter();
                    pntinf = dh2.FirstPoint();
                    ft = dh2.FirstTolerance();
                }
                if (bsup > dh2.LastParameter())
                {
                    bsup = dh2.LastParameter();
                    pntsup = dh2.LastPoint();
                    lt = dh2.LastTolerance();
                }

                //-- lbr le 22 sept 97
                //-- binf=-5.6 bsup=-5.3 DH2=(5.6,5.7) -> binf=+5.6 bsup=-5.3
                if (binf >= bsup)
                {
                    _done = true;
                    return;
                }
                TwoDDomain dhModif = new(pntinf, binf, ft, pntsup, bsup, lt);
                _inter.Perform(iTool, dh1, pCurve, dhModif, tolConf, tol);
            }
            this.SetValues(_inter);
        }
        else
        {
            _done = true;
        }
    }

    //----------------------------------------------------------------------

    private static void SetBinfBsupFromIntAna2d(AnalyticIntersection intAna2d,
                                                ref double binf,
                                                ref Point2D pntinf,
                                                ref double bsup,
                                                ref Point2D pntsup,
                                                in Parabola2D pr,
                                                double maxtol,
                                                double limite)
    {
        if (intAna2d.IsDone)
        {
            if (!intAna2d.IsEmpty)
            {
                for (int p = 1; p <= intAna2d.PointCount; p++)
                {
                    double param = intAna2d.Point(p).ParamOnFirst();

                    if (Math.Abs(param) < limite)
                    {
                        global::Kerf.Engine.Geometry.ElCLib.D1(param, pr, out _, out Vector2D v);
                        double normeD1 = v.Magnitude();
                        double dparam = 100.0 * maxtol / normeD1;
                        if (dparam < 1e-3)
                        {
                            dparam = 1e-3;
                        }
                        param -= dparam;

                        if (param < binf)
                        {
                            binf = param;
                            pntinf = global::Kerf.Engine.Geometry.ElCLib.Value(param, pr);
                        }
                        param += dparam + dparam;
                        if (param > bsup)
                        {
                            bsup = param;
                            pntsup = global::Kerf.Engine.Geometry.ElCLib.Value(param, pr);
                        }
                    }
                }
            }
        }
    }

    private static void SetBinfBsupFromIntAna2d(AnalyticIntersection intAna2d,
                                                ref double binf,
                                                ref Point2D pntinf,
                                                ref double bsup,
                                                ref Point2D pntsup,
                                                in Hyperbola2D h,
                                                double maxtol,
                                                double limite)
    {
        if (intAna2d.IsDone)
        {
            if (!intAna2d.IsEmpty)
            {
                for (int p = 1; p <= intAna2d.PointCount; p++)
                {
                    double param = intAna2d.Point(p).ParamOnFirst();

                    if (Math.Abs(param) < limite)
                    {
                        global::Kerf.Engine.Geometry.ElCLib.D1(param, h, out _, out Vector2D v);
                        double normeD1 = v.Magnitude();
                        double dparam = 100.0 * maxtol / normeD1;
                        if (dparam < 1e-3)
                        {
                            dparam = 1e-3;
                        }
                        param -= dparam;

                        if (param < binf)
                        {
                            binf = param;
                            pntinf = global::Kerf.Engine.Geometry.ElCLib.Value(param, h);
                        }
                        param += dparam + dparam;
                        if (param > bsup)
                        {
                            bsup = param;
                            pntsup = global::Kerf.Engine.Geometry.ElCLib.Value(param, h);
                        }
                    }
                }
            }
        }
    }
}
