// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntCurveCurveGen.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntCurveCurveGen.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>
/// The intersection between two 2D curves, or the self-intersection of one: conic/conic pairs go to
/// <see cref="IntConicConic"/>, conic/curve pairs to the implicit/parametric intersection, and other
/// pairs to the polygon intersection, interval by interval of continuity of composite curves.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_IntCurveCurveGen.gxx as a generic class (decision 321); its body names the
/// generic classes it uses directly. OCCT's instantiation is <see cref="GInter"/>.
/// </remarks>
public class IntCurveCurveGen<TheCurve, TheCurveTool, TheProjPCur, TheMyImpParTool> : Intersection2D
    where TheCurveTool : ICurveCurveTool<TheCurve>
    where TheProjPCur : IProjectOnPCurveTool<TheCurve>
    where TheMyImpParTool : FunctionWithDerivative,
                            IMyImpParTool<TheMyImpParTool, IConicTool, TheCurve>
{
    private double _param1inf;
    private double _param1sup;
    private double _param2inf;
    private double _param2sup;
    private readonly IntConicConic _intconiconi = new();
    private readonly IntConicCurveGen<TheCurve, TheCurveTool, TheProjPCur, TheMyImpParTool> _intconicurv =
        new();
    private readonly IntPolyPolyGen<TheCurve, TheCurveTool, TheProjPCur> _intcurvcurv = new();

    /// <summary>Empty constructor.</summary>
    public IntCurveCurveGen()
    {
        _param1inf = -global::Kerf.Engine.Foundation.Precision.Infinite();
        _param1sup = global::Kerf.Engine.Foundation.Precision.Infinite();
        _param2inf = -global::Kerf.Engine.Foundation.Precision.Infinite();
        _param2sup = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>Self-intersection of a curve.</summary>
    public IntCurveCurveGen(in TheCurve c, double tolConf, double tol)
        : this()
    {
        Perform(c, tolConf, tol);
    }

    /// <summary>Self-intersection of a curve on a domain.</summary>
    public IntCurveCurveGen(in TheCurve c, in TwoDDomain d, double tolConf, double tol)
        : this()
    {
        Perform(c, d, tolConf, tol);
    }

    /// <summary>Intersection between two curves.</summary>
    public IntCurveCurveGen(in TheCurve c1, in TheCurve c2, double tolConf, double tol)
        : this()
    {
        Perform(c1, c2, tolConf, tol);
    }

    /// <summary>Intersection between two curves, the first on a domain.</summary>
    public IntCurveCurveGen(in TheCurve c1, in TwoDDomain d1, in TheCurve c2, double tolConf, double tol)
        : this()
    {
        Perform(c1, d1, c2, tolConf, tol);
    }

    /// <summary>Intersection between two curves, the second on a domain.</summary>
    public IntCurveCurveGen(in TheCurve c1, in TheCurve c2, in TwoDDomain d2, double tolConf, double tol)
        : this()
    {
        Perform(c1, c2, d2, tolConf, tol);
    }

    /// <summary>Intersection between two curves on their domains.</summary>
    public IntCurveCurveGen(in TheCurve c1, in TwoDDomain d1, in TheCurve c2, in TwoDDomain d2,
                                     double tolConf, double tol)
        : this()
    {
        Perform(c1, d1, c2, d2, tolConf, tol);
    }

    /// <summary>Intersection between two curves on their whole ranges.</summary>
    public void Perform(in TheCurve c1, in TheCurve c2, double tolConf, double tol)
    {
        double tolDomain = tol;
        if (tolConf > tolDomain)
        {
            tolDomain = tolConf;
        }
        Perform(c1, ComputeDomain(c1, tolDomain), c2, ComputeDomain(c2, tolDomain), tolConf, tol);
    }

    /// <summary>Intersection between two curves, the first on a domain.</summary>
    public void Perform(in TheCurve c1, in TwoDDomain d1, in TheCurve c2, double tolConf, double tol)
    {
        double tolDomain = tol;
        if (tolConf > tolDomain)
        {
            tolDomain = tolConf;
        }
        Perform(c1, d1, c2, ComputeDomain(c2, tolDomain), tolConf, tol);
    }

    /// <summary>Intersection between two curves, the second on a domain.</summary>
    public void Perform(in TheCurve c1, in TheCurve c2, in TwoDDomain d2, double tolConf, double tol)
    {
        double tolDomain = tol;
        if (tolConf > tolDomain)
        {
            tolDomain = tolConf;
        }
        Perform(c1, ComputeDomain(c1, tolDomain), c2, d2, tolConf, tol);
    }


    /// <summary>Self-intersection of a curve on its whole range (a conic has none).</summary>
    public void Perform(in TheCurve c, double tolConf, double tol)
    {
        TwoDDomain d1 = new();
        double          tolDomain = tol;
        if (tol < tolConf)
            tolDomain = tolConf;
        CurveKind typ = TheCurveTool.GetType(c);
        switch (typ)
        {
            case Ellipse:
            case Circle:
            case Parabola:
            case Hyperbola:
            case Line:
                ResetFields();
                _done = true;
                return;

            default: {
                double paraminf = TheCurveTool.FirstParameter(c);
                double paramsup = TheCurveTool.LastParameter(c);
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(paraminf) && global::Kerf.Engine.Foundation.Precision.IsInfinite(paramsup))
                {
                    _done = false;
                    return;
                }
                //
                if (paraminf > -global::Kerf.Engine.Foundation.Precision.Infinite())
                {
                    if (paramsup < global::Kerf.Engine.Foundation.Precision.Infinite())
                    {
                        //--         paraminf-----------paramsup
                        d1.SetValues(TheCurveTool.Value(c, paraminf),
                                                  paraminf,
                                                  tolDomain,
                                                  TheCurveTool.Value(c, paramsup),
                                                  paramsup,
                                                  tolDomain);
                    }
                    else
                    {
                        //--        paraminf------------...
                        d1.SetValues(TheCurveTool.Value(c, paraminf), paraminf, tolDomain, true);
                    }
                }
                else
                {
                    if (paramsup < global::Kerf.Engine.Foundation.Precision.Infinite())
                    {
                        //--    ...-----------------paramsup
                        d1.SetValues(TheCurveTool.Value(c, paramsup), paramsup, tolDomain, false);
                    }
                }
                this.ResetFields();
                _intcurvcurv.SetReversedParameters(false);
                _intcurvcurv.Perform(c, d1, tolConf, tol);
                this.SetValues(_intcurvcurv);
                _done = true;
            }
            break;
        }
    }

    /// <summary>Self-intersection of a curve on a domain (a conic has none).</summary>
    public void Perform(in TheCurve c, in TwoDDomain d, double tolConf, double tol)
    {
        CurveKind typ = TheCurveTool.GetType(c);
        switch (typ)
        {
            case Ellipse:
            case Circle:
            case Parabola:
            case Hyperbola:
            case Line:
                ResetFields();
                _done = true;
                return;

            default: {
                this.ResetFields();
                _intcurvcurv.SetReversedParameters(false);
                _intcurvcurv.Perform(c, d, tolConf, tol);
                this.SetValues(_intcurvcurv);
                _done = true;
            }
            break;
        }
    }



    /// <summary>The domain of the whole curve, closed with a period for a circle or an ellipse.</summary>
    public TwoDDomain ComputeDomain(in TheCurve c1, double tolDomain)
    {
        TwoDDomain d1 = new();

        CurveKind typ = TheCurveTool.GetType(c1);
        switch (typ)
        {

            case Ellipse:
            case Circle: {
                //---------------------------------------------------------------
                //-- if the curve is a trimmed curve, first and last parameters
                //-- will be the parameters used to build the domain
                //--
                double firstparameter = TheCurveTool.FirstParameter(c1);
                double lastparameter  = TheCurveTool.LastParameter(c1);

                Point2D p1 = TheCurveTool.Value(c1, firstparameter);
                Point2D p2 = TheCurveTool.Value(c1, lastparameter);
                d1.SetValues(p1, firstparameter, tolDomain, p2, lastparameter, tolDomain);
                d1.SetEquivalentParameters(firstparameter, firstparameter + M_PI + M_PI);
                break;
            }
            default: {
                double paraminf = TheCurveTool.FirstParameter(c1);
                double paramsup = TheCurveTool.LastParameter(c1);
                if (paraminf > -global::Kerf.Engine.Foundation.Precision.Infinite())
                {
                    if (paramsup < global::Kerf.Engine.Foundation.Precision.Infinite())
                    {
                        //--         paraminf-----------paramsup
                        d1.SetValues(TheCurveTool.Value(c1, paraminf),
                                                  paraminf,
                                                  tolDomain,
                                                  TheCurveTool.Value(c1, paramsup),
                                                  paramsup,
                                                  tolDomain);
                    }
                    else
                    {
                        //--        paraminf------------...
                        d1.SetValues(TheCurveTool.Value(c1, paraminf), paraminf, tolDomain, true);
                    }
                }
                else
                {
                    if (paramsup < global::Kerf.Engine.Foundation.Precision.Infinite())
                    {
                        //--    ...-----------------paramsup
                        d1.SetValues(TheCurveTool.Value(c1, paramsup), paramsup, tolDomain, false);
                    }
                }
                break;
            }
        }
        return (d1);
    }

    //----------------------------------------------------------------------
    //-- Perform : Si Une des courbes est Composite Alors decompose les appels
    //----------------------------------------------------------------------
    /// <summary>Intersection between two curves on their domains.</summary>
    public void Perform(in TheCurve c1, in TwoDDomain d1, in TheCurve c2, in TwoDDomain d2, double tolConf, double tol)
    {
        this.ResetFields();
        int nbi1 = TheCurveTool.NbIntervals(c1);
        if (nbi1 > 1)
        {
            _param1inf = TheCurveTool.FirstParameter(c1);
            _param1sup = TheCurveTool.LastParameter(c1);
        }
        else
        {
            _param1inf = (d1.HasFirstPoint) ? (d1.FirstParameter()) : -global::Kerf.Engine.Foundation.Precision.Infinite();
            _param1sup = (d1.HasLastPoint) ? (d1.LastParameter()) : global::Kerf.Engine.Foundation.Precision.Infinite();
        }
        int nbi2 = TheCurveTool.NbIntervals(c2);
        if (nbi2 > 1)
        {
            _param2inf = TheCurveTool.FirstParameter(c2);
            _param2sup = TheCurveTool.LastParameter(c2);
        }
        else
        {
            _param2inf = (d2.HasFirstPoint) ? (d2.FirstParameter()) : -global::Kerf.Engine.Foundation.Precision.Infinite();
            _param2sup = (d2.HasLastPoint) ? (d2.LastParameter()) : global::Kerf.Engine.Foundation.Precision.Infinite();
        }
        if (nbi1 > 1 || nbi2 > 1)
        {
            NCollection_Array1<double> tab1 = new(1, nbi1 + 1);
            NCollection_Array1<double> tab2 = new(1, nbi2 + 1);
            TheCurveTool.Intervals(c1, tab1);
            TheCurveTool.Intervals(c2, tab2);
            InternalCompositePerform(c1, d1, 1, nbi1, tab1, c2, d2, 1, nbi2, tab2, tolConf, tol, true);
            return;
        }
        else
        {
            InternalPerform(c1, d1, c2, d2, tolConf, tol, false);
        }
    }

    //----------------------------------------------------------------------
    //-- InternalPerform
    //-- Suppose des Courbes Lin...Other
    //-- Si Composite == True
    //--     Les Resultats sont Ajoutes
    //-- Sinon
    //--     Les Resultats sont Copies
    //----------------------------------------------------------------------
    private void InternalPerform(in TheCurve c1, in TwoDDomain d1, in TheCurve c2, in TwoDDomain d2, double tolConf, double tol, bool composite)
    {

        CurveKind typ1 = TheCurveTool.GetType(c1);
        CurveKind typ2 = TheCurveTool.GetType(c2);

        switch (typ1)
        {
            case Line:
                switch (typ2)
                {
                    case Line: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi.Perform(TheCurveTool.Line(c1), d1, TheCurveTool.Line(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Circle: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi
                            .Perform(TheCurveTool.Line(c1), d1, TheCurveTool.Circle(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Ellipse: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi
                            .Perform(TheCurveTool.Line(c1), d1, TheCurveTool.Ellipse(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Parabola: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi
                            .Perform(TheCurveTool.Line(c1), d1, TheCurveTool.Parabola(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Hyperbola: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi
                            .Perform(TheCurveTool.Line(c1), d1, TheCurveTool.Hyperbola(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }

                    break;

                    default: {
                        _intconicurv.SetReversedParameters(false);
                        _intconicurv.Perform(TheCurveTool.Line(c1), d1, c2, d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconicurv, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconicurv);
                        }
                    }
                    break;
                }
                break;

            case Circle:
                switch (typ2)
                {
                    case Line: {
                        _intconiconi.SetReversedParameters(true);
                        _intconiconi
                            .Perform(TheCurveTool.Line(c2), d2, TheCurveTool.Circle(c1), d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Circle: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi
                            .Perform(TheCurveTool.Circle(c1), d1, TheCurveTool.Circle(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Ellipse: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi
                            .Perform(TheCurveTool.Circle(c1), d1, TheCurveTool.Ellipse(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Parabola: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi
                            .Perform(TheCurveTool.Circle(c1), d1, TheCurveTool.Parabola(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Hyperbola: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi
                            .Perform(TheCurveTool.Circle(c1), d1, TheCurveTool.Hyperbola(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    default: {
                        _intconicurv.SetReversedParameters(false);
                        _intconicurv.Perform(TheCurveTool.Circle(c1), d1, c2, d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconicurv, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconicurv);
                        }
                    }

                    break;
                }
                break;

            case Ellipse:
                switch (typ2)
                {
                    case Line: {
                        _intconiconi.SetReversedParameters(true);
                        _intconiconi
                            .Perform(TheCurveTool.Line(c2), d2, TheCurveTool.Ellipse(c1), d1, tolConf, tol);

                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Circle: {
                        _intconiconi.SetReversedParameters(true);

                        _intconiconi
                            .Perform(TheCurveTool.Circle(c2), d2, TheCurveTool.Ellipse(c1), d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Ellipse: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi
                            .Perform(TheCurveTool.Ellipse(c1), d1, TheCurveTool.Ellipse(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Parabola: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi
                            .Perform(TheCurveTool.Ellipse(c1), d1, TheCurveTool.Parabola(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Hyperbola: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi
                            .Perform(TheCurveTool.Ellipse(c1), d1, TheCurveTool.Hyperbola(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    default: {
                        _intconicurv.SetReversedParameters(false);
                        _intconicurv.Perform(TheCurveTool.Ellipse(c1), d1, c2, d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconicurv, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconicurv);
                        }
                    }
                    break;
                }
                break;

            case Parabola:
                switch (typ2)
                {
                    case Line: {
                        _intconiconi.SetReversedParameters(true);
                        _intconiconi
                            .Perform(TheCurveTool.Line(c2), d2, TheCurveTool.Parabola(c1), d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Circle: {
                        _intconiconi.SetReversedParameters(true);
                        _intconiconi
                            .Perform(TheCurveTool.Circle(c2), d2, TheCurveTool.Parabola(c1), d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Ellipse: {
                        _intconiconi.SetReversedParameters(true);
                        _intconiconi
                            .Perform(TheCurveTool.Ellipse(c2), d2, TheCurveTool.Parabola(c1), d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Parabola: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi
                            .Perform(TheCurveTool.Parabola(c1), d1, TheCurveTool.Parabola(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Hyperbola: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi
                            .Perform(TheCurveTool.Parabola(c1), d1, TheCurveTool.Hyperbola(c2), d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    default: {
                        _intconicurv.SetReversedParameters(false);
                        _intconicurv.Perform(TheCurveTool.Parabola(c1), d1, c2, d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconicurv, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconicurv);
                        }
                    }
                    break;
                }
                break;

            case Hyperbola:
                switch (typ2)
                {
                    case Line: {
                        _intconiconi.SetReversedParameters(true);
                        _intconiconi
                            .Perform(TheCurveTool.Line(c2), d2, TheCurveTool.Hyperbola(c1), d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Circle: {
                        _intconiconi.SetReversedParameters(true);
                        _intconiconi
                            .Perform(TheCurveTool.Circle(c2), d2, TheCurveTool.Hyperbola(c1), d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Ellipse: {
                        _intconiconi.SetReversedParameters(true);
                        _intconiconi
                            .Perform(TheCurveTool.Ellipse(c2), d2, TheCurveTool.Hyperbola(c1), d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Parabola: {
                        _intconiconi.SetReversedParameters(true);
                        _intconiconi
                            .Perform(TheCurveTool.Parabola(c2), d2, TheCurveTool.Hyperbola(c1), d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    case Hyperbola: {
                        _intconiconi.SetReversedParameters(false);
                        _intconiconi.Perform(TheCurveTool.Hyperbola(c1),
                                                                d1,
                                                                TheCurveTool.Hyperbola(c2),
                                                                d2,
                                                                tolConf,
                                                                tol);
                        if (composite)
                        {
                            this.Append(_intconiconi, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconiconi);
                        }
                    }
                    break;

                    default: {
                        _intconicurv.SetReversedParameters(false);
                        _intconicurv.Perform(TheCurveTool.Hyperbola(c1), d1, c2, d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconicurv, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconicurv);
                        }
                    }
                    break;
                }
                break;

            default:
                switch (typ2)
                {
                    case Line: {
                        _intconicurv.SetReversedParameters(true);
                        _intconicurv.Perform(TheCurveTool.Line(c2), d2, c1, d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconicurv, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconicurv);
                        }
                    }
                    break;

                    case Circle: {
                        _intconicurv.SetReversedParameters(true);
                        _intconicurv.Perform(TheCurveTool.Circle(c2), d2, c1, d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconicurv, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconicurv);
                        }
                    }
                    break;

                    case Ellipse: {
                        _intconicurv.SetReversedParameters(true);
                        _intconicurv.Perform(TheCurveTool.Ellipse(c2), d2, c1, d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconicurv, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconicurv);
                        }
                    }
                    break;

                    case Parabola: {
                        _intconicurv.SetReversedParameters(true);
                        _intconicurv.Perform(TheCurveTool.Parabola(c2), d2, c1, d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconicurv, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconicurv);
                        }
                    }
                    break;

                    case Hyperbola: {
                        _intconicurv.SetReversedParameters(true);
                        _intconicurv.Perform(TheCurveTool.Hyperbola(c2), d2, c1, d1, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intconicurv, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intconicurv);
                        }
                    }
                    break;

                    default: {
                        _intcurvcurv.SetReversedParameters(false);
                        _intcurvcurv.Perform(c1, d1, c2, d2, tolConf, tol);
                        if (composite)
                        {
                            this.Append(_intcurvcurv, _param1inf, _param1sup, _param2inf, _param2sup);
                        }
                        else
                        {
                            this.SetValues(_intcurvcurv);
                        }
                        _done = true;
                    }
                    break;
                }
                break;
        }
    }

    private void InternalCompositePerform_noRecurs(int nbInterC1, in TheCurve c1, int numInterC1, NCollection_Array1<double> tab1, in TwoDDomain d1, int nbInterC2, in TheCurve c2, int numInterC2, NCollection_Array1<double> tab2, in TwoDDomain d2, double tolConf, double tol)
    {

        if (numInterC2 > nbInterC2)
            return;

        TwoDDomain domainC1NumInter = new();
        TwoDDomain domainC2NumInter = new();

        //----------------------------------------------------------------------
        //-- Creation du domaine associe a la portion de C1
        //----------------------------------------------------------------------
        bool   domainIsOK = true;
        double paramInf, paramSup;

        if (nbInterC1 > 1)
        {
            TheCurveTool.GetInterval(c1, numInterC1, tab1, out paramInf, out paramSup);
            //--------------------------------------------------------------
            //-- Verification : Domaine Inclu dans Intervalle de Definition
            //--------------------------------------------------------------

            double u;

            u = d1.FirstParameter();
            if (paramInf < u)
            {
                paramInf = u;
            }

            u = d1.LastParameter();
            if (paramSup > u)
            {
                paramSup = u;
            }

            if ((paramSup - paramInf) > 1e-10)
            {
                domainC1NumInter.SetValues(TheCurveTool.Value(c1, paramInf),
                                                                      paramInf,
                                                                      d1.FirstTolerance(),
                                                                      TheCurveTool.Value(c1, paramSup),
                                                                      paramSup,
                                                                      d1.LastTolerance());
            }
            else
            {
                domainIsOK = false;
            }
        }
        else
        {
            domainC1NumInter = d1;
        }

        //----------------------------------------------------------------------
        //-- Creation du domaine associe a la portion de C2
        //----------------------------------------------------------------------
        if (nbInterC2 > 1)
        {
            TheCurveTool.GetInterval(c2, numInterC2, tab2, out paramInf, out paramSup);
            //--------------------------------------------------------------
            //-- Verification : Domaine Inclu dans Intervalle de Definition
            //--------------------------------------------------------------

            double u;

            u = d2.FirstParameter();
            if (paramInf < u)
            {
                paramInf = u;
            }
            u = d2.LastParameter();

            if (paramSup > u)
            {
                paramSup = u;
            }

            if ((paramSup - paramInf) > 1e-10)
            {
                domainC2NumInter.SetValues(TheCurveTool.Value(c2, paramInf),
                                                                      paramInf,
                                                                      d2.FirstTolerance(),
                                                                      TheCurveTool.Value(c2, paramSup),
                                                                      paramSup,
                                                                      d2.LastTolerance());
            }
            else
            {
                domainIsOK = false;
            }
        }
        else
        {
            domainC2NumInter = d2;
        }

        if (domainIsOK)
        {
            InternalPerform(c2, domainC2NumInter, c1, domainC1NumInter, tolConf, tol, true);
        }
    }

    //-- C1 ou C2 sont des courbes composites
    //--

    private void InternalCompositePerform(in TheCurve c1, in TwoDDomain d1, int xxxNumInterC1, int nbInterC1, NCollection_Array1<double> tab1, in TheCurve c2, in TwoDDomain d2, int xxxNumInterC2, int nbInterC2, NCollection_Array1<double> tab2, double tolConf, double tol, bool recursOnC2)
    {

        int numInterC2 = xxxNumInterC2;
        int numInterC1 = xxxNumInterC1;

        //  bool Arret=false;

        if (numInterC2 > nbInterC2)
            return;

        if (!recursOnC2)
        {
            InternalCompositePerform_noRecurs(nbInterC1,
                                                                                c1,
                                                                                numInterC1,
                                                                                tab1,
                                                                                d1,
                                                                                nbInterC2,
                                                                                c2,
                                                                                numInterC2,
                                                                                tab2,
                                                                                d2,
                                                                                tolConf,
                                                                                tol);
            return;
        }

        for (int i = numInterC1; i <= nbInterC1; i++)
        {
            numInterC1 = i;
            /*
                InternalCompositePerform(C2,D2,NumInterC2,NbInterC2,Tab2,
                            C1,D1,NumInterC1,NbInterC1,Tab1,
                            TolConf,Tol,false);
            */

            InternalCompositePerform_noRecurs(nbInterC2,
                                                                                c2,
                                                                                numInterC2,
                                                                                tab2,
                                                                                d2,
                                                                                nbInterC1,
                                                                                c1,
                                                                                numInterC1,
                                                                                tab1,
                                                                                d1,
                                                                                tolConf,
                                                                                tol);
        }

        if (numInterC2 < nbInterC2)
        {
            numInterC2++;
            numInterC1 = 1;

            InternalCompositePerform(c1,
                                                              d1,
                                                              numInterC1,
                                                              nbInterC1,
                                                              tab1,
                                                              c2,
                                                              d2,
                                                              numInterC2,
                                                              nbInterC2,
                                                              tab2,
                                                              tolConf,
                                                              tol,
                                                              true);
        }
    }



    /// <summary>The minimum number of samples of the polygon intersection.</summary>
    public void SetMinNbSamples(int minNbSamples)
    {
        _intcurvcurv.SetMinNbSamples(minNbSamples);
    }



    public int GetMinNbSamples()
    {
        return _intcurvcurv.GetMinNbSamples();
    }
}
