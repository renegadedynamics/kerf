// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntCurveCurveGen.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntCurveCurveGen.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The intersection between two 2D curves, or the self-intersection of one: conic/conic pairs go to
/// <see cref="IntCurve_IntConicConic"/>, conic/curve pairs to the implicit/parametric intersection, and other
/// pairs to the polygon intersection, interval by interval of continuity of composite curves.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_IntCurveCurveGen.gxx as a generic class (decision 321); its body names the
/// generic classes it uses directly. OCCT's instantiation is <see cref="Geom2dInt_GInter"/>.
/// </remarks>
public class IntCurve_IntCurveCurveGen<TheCurve, TheCurveTool, TheProjPCur, TheMyImpParTool> : IntRes2d_Intersection
    where TheCurveTool : IIntCurve_CurveTool<TheCurve>
    where TheProjPCur : IIntImpParGen_ProjectOnPCurveTool<TheCurve>
    where TheMyImpParTool : FunctionWithDerivative,
                            IIntImpParGen_MyImpParTool<TheMyImpParTool, IntCurve_IConicTool, TheCurve>
{
    private double param1inf;
    private double param1sup;
    private double param2inf;
    private double param2sup;
    private readonly IntCurve_IntConicConic intconiconi = new();
    private readonly IntCurve_IntConicCurveGen<TheCurve, TheCurveTool, TheProjPCur, TheMyImpParTool> intconicurv =
        new();
    private readonly IntCurve_IntPolyPolyGen<TheCurve, TheCurveTool, TheProjPCur> intcurvcurv = new();

    /// <summary>Empty constructor.</summary>
    public IntCurve_IntCurveCurveGen()
    {
        param1inf = -global::Kerf.Engine.Foundation.Precision.Infinite();
        param1sup = global::Kerf.Engine.Foundation.Precision.Infinite();
        param2inf = -global::Kerf.Engine.Foundation.Precision.Infinite();
        param2sup = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>Self-intersection of a curve.</summary>
    public IntCurve_IntCurveCurveGen(in TheCurve C, double TolConf, double Tol)
        : this()
    {
        Perform(C, TolConf, Tol);
    }

    /// <summary>Self-intersection of a curve on a domain.</summary>
    public IntCurve_IntCurveCurveGen(in TheCurve C, in IntRes2d_Domain D, double TolConf, double Tol)
        : this()
    {
        Perform(C, D, TolConf, Tol);
    }

    /// <summary>Intersection between two curves.</summary>
    public IntCurve_IntCurveCurveGen(in TheCurve C1, in TheCurve C2, double TolConf, double Tol)
        : this()
    {
        Perform(C1, C2, TolConf, Tol);
    }

    /// <summary>Intersection between two curves, the first on a domain.</summary>
    public IntCurve_IntCurveCurveGen(in TheCurve C1, in IntRes2d_Domain D1, in TheCurve C2, double TolConf, double Tol)
        : this()
    {
        Perform(C1, D1, C2, TolConf, Tol);
    }

    /// <summary>Intersection between two curves, the second on a domain.</summary>
    public IntCurve_IntCurveCurveGen(in TheCurve C1, in TheCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol)
        : this()
    {
        Perform(C1, C2, D2, TolConf, Tol);
    }

    /// <summary>Intersection between two curves on their domains.</summary>
    public IntCurve_IntCurveCurveGen(in TheCurve C1, in IntRes2d_Domain D1, in TheCurve C2, in IntRes2d_Domain D2,
                                     double TolConf, double Tol)
        : this()
    {
        Perform(C1, D1, C2, D2, TolConf, Tol);
    }

    /// <summary>Intersection between two curves on their whole ranges.</summary>
    public void Perform(in TheCurve C1, in TheCurve C2, double TolConf, double Tol)
    {
        double TolDomain = Tol;
        if (TolConf > TolDomain)
        {
            TolDomain = TolConf;
        }
        Perform(C1, ComputeDomain(C1, TolDomain), C2, ComputeDomain(C2, TolDomain), TolConf, Tol);
    }

    /// <summary>Intersection between two curves, the first on a domain.</summary>
    public void Perform(in TheCurve C1, in IntRes2d_Domain D1, in TheCurve C2, double TolConf, double Tol)
    {
        double TolDomain = Tol;
        if (TolConf > TolDomain)
        {
            TolDomain = TolConf;
        }
        Perform(C1, D1, C2, ComputeDomain(C2, TolDomain), TolConf, Tol);
    }

    /// <summary>Intersection between two curves, the second on a domain.</summary>
    public void Perform(in TheCurve C1, in TheCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        double TolDomain = Tol;
        if (TolConf > TolDomain)
        {
            TolDomain = TolConf;
        }
        Perform(C1, ComputeDomain(C1, TolDomain), C2, D2, TolConf, Tol);
    }


    /// <summary>Self-intersection of a curve on its whole range (a conic has none).</summary>
    public void Perform(in TheCurve C, double TolConf, double Tol)
    {
        IntRes2d_Domain D1 = new();
        double          TolDomain = Tol;
        if (Tol < TolConf)
            TolDomain = TolConf;
        CurveKind typ = TheCurveTool.GetType(C);
        switch (typ)
        {
            case Ellipse:
            case Circle:
            case Parabola:
            case Hyperbola:
            case Line:
                ResetFields();
                done = true;
                return;

            default: {
                double paraminf = TheCurveTool.FirstParameter(C);
                double paramsup = TheCurveTool.LastParameter(C);
                if (global::Kerf.Engine.Foundation.Precision.IsInfinite(paraminf) && global::Kerf.Engine.Foundation.Precision.IsInfinite(paramsup))
                {
                    done = false;
                    return;
                }
                //
                if (paraminf > -global::Kerf.Engine.Foundation.Precision.Infinite())
                {
                    if (paramsup < global::Kerf.Engine.Foundation.Precision.Infinite())
                    {
                        //--         paraminf-----------paramsup
                        D1.SetValues(TheCurveTool.Value(C, paraminf),
                                                  paraminf,
                                                  TolDomain,
                                                  TheCurveTool.Value(C, paramsup),
                                                  paramsup,
                                                  TolDomain);
                    }
                    else
                    {
                        //--        paraminf------------...
                        D1.SetValues(TheCurveTool.Value(C, paraminf), paraminf, TolDomain, true);
                    }
                }
                else
                {
                    if (paramsup < global::Kerf.Engine.Foundation.Precision.Infinite())
                    {
                        //--    ...-----------------paramsup
                        D1.SetValues(TheCurveTool.Value(C, paramsup), paramsup, TolDomain, false);
                    }
                }
                this.ResetFields();
                intcurvcurv.SetReversedParameters(false);
                intcurvcurv.Perform(C, D1, TolConf, Tol);
                this.SetValues(intcurvcurv);
                done = true;
            }
            break;
        }
    }

    /// <summary>Self-intersection of a curve on a domain (a conic has none).</summary>
    public void Perform(in TheCurve C, in IntRes2d_Domain D, double TolConf, double Tol)
    {
        CurveKind typ = TheCurveTool.GetType(C);
        switch (typ)
        {
            case Ellipse:
            case Circle:
            case Parabola:
            case Hyperbola:
            case Line:
                ResetFields();
                done = true;
                return;

            default: {
                this.ResetFields();
                intcurvcurv.SetReversedParameters(false);
                intcurvcurv.Perform(C, D, TolConf, Tol);
                this.SetValues(intcurvcurv);
                done = true;
            }
            break;
        }
    }



    /// <summary>The domain of the whole curve, closed with a period for a circle or an ellipse.</summary>
    public IntRes2d_Domain ComputeDomain(in TheCurve C1, double TolDomain)
    {
        IntRes2d_Domain D1 = new();

        CurveKind typ = TheCurveTool.GetType(C1);
        switch (typ)
        {

            case Ellipse:
            case Circle: {
                //---------------------------------------------------------------
                //-- if the curve is a trimmed curve, first and last parameters
                //-- will be the parameters used to build the domain
                //--
                double firstparameter = TheCurveTool.FirstParameter(C1);
                double lastparameter  = TheCurveTool.LastParameter(C1);

                Point2D P1 = TheCurveTool.Value(C1, firstparameter);
                Point2D P2 = TheCurveTool.Value(C1, lastparameter);
                D1.SetValues(P1, firstparameter, TolDomain, P2, lastparameter, TolDomain);
                D1.SetEquivalentParameters(firstparameter, firstparameter + M_PI + M_PI);
                break;
            }
            default: {
                double paraminf = TheCurveTool.FirstParameter(C1);
                double paramsup = TheCurveTool.LastParameter(C1);
                if (paraminf > -global::Kerf.Engine.Foundation.Precision.Infinite())
                {
                    if (paramsup < global::Kerf.Engine.Foundation.Precision.Infinite())
                    {
                        //--         paraminf-----------paramsup
                        D1.SetValues(TheCurveTool.Value(C1, paraminf),
                                                  paraminf,
                                                  TolDomain,
                                                  TheCurveTool.Value(C1, paramsup),
                                                  paramsup,
                                                  TolDomain);
                    }
                    else
                    {
                        //--        paraminf------------...
                        D1.SetValues(TheCurveTool.Value(C1, paraminf), paraminf, TolDomain, true);
                    }
                }
                else
                {
                    if (paramsup < global::Kerf.Engine.Foundation.Precision.Infinite())
                    {
                        //--    ...-----------------paramsup
                        D1.SetValues(TheCurveTool.Value(C1, paramsup), paramsup, TolDomain, false);
                    }
                }
                break;
            }
        }
        return (D1);
    }

    //----------------------------------------------------------------------
    //-- Perform : Si Une des courbes est Composite Alors decompose les appels
    //----------------------------------------------------------------------
    /// <summary>Intersection between two curves on their domains.</summary>
    public void Perform(in TheCurve C1, in IntRes2d_Domain D1, in TheCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        this.ResetFields();
        int nbi1 = TheCurveTool.NbIntervals(C1);
        if (nbi1 > 1)
        {
            param1inf = TheCurveTool.FirstParameter(C1);
            param1sup = TheCurveTool.LastParameter(C1);
        }
        else
        {
            param1inf = (D1.HasFirstPoint()) ? (D1.FirstParameter()) : -global::Kerf.Engine.Foundation.Precision.Infinite();
            param1sup = (D1.HasLastPoint()) ? (D1.LastParameter()) : global::Kerf.Engine.Foundation.Precision.Infinite();
        }
        int nbi2 = TheCurveTool.NbIntervals(C2);
        if (nbi2 > 1)
        {
            param2inf = TheCurveTool.FirstParameter(C2);
            param2sup = TheCurveTool.LastParameter(C2);
        }
        else
        {
            param2inf = (D2.HasFirstPoint()) ? (D2.FirstParameter()) : -global::Kerf.Engine.Foundation.Precision.Infinite();
            param2sup = (D2.HasLastPoint()) ? (D2.LastParameter()) : global::Kerf.Engine.Foundation.Precision.Infinite();
        }
        if (nbi1 > 1 || nbi2 > 1)
        {
            NCollection_Array1<double> Tab1 = new(1, nbi1 + 1);
            NCollection_Array1<double> Tab2 = new(1, nbi2 + 1);
            TheCurveTool.Intervals(C1, Tab1);
            TheCurveTool.Intervals(C2, Tab2);
            InternalCompositePerform(C1, D1, 1, nbi1, Tab1, C2, D2, 1, nbi2, Tab2, TolConf, Tol, true);
            return;
        }
        else
        {
            InternalPerform(C1, D1, C2, D2, TolConf, Tol, false);
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
    private void InternalPerform(in TheCurve C1, in IntRes2d_Domain D1, in TheCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol, bool Composite)
    {

        CurveKind typ1 = TheCurveTool.GetType(C1);
        CurveKind typ2 = TheCurveTool.GetType(C2);

        switch (typ1)
        {
            case Line:
                switch (typ2)
                {
                    case Line: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi.Perform(TheCurveTool.Line(C1), D1, TheCurveTool.Line(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Circle: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi
                            .Perform(TheCurveTool.Line(C1), D1, TheCurveTool.Circle(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Ellipse: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi
                            .Perform(TheCurveTool.Line(C1), D1, TheCurveTool.Ellipse(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Parabola: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi
                            .Perform(TheCurveTool.Line(C1), D1, TheCurveTool.Parabola(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Hyperbola: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi
                            .Perform(TheCurveTool.Line(C1), D1, TheCurveTool.Hyperbola(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }

                    break;

                    default: {
                        intconicurv.SetReversedParameters(false);
                        intconicurv.Perform(TheCurveTool.Line(C1), D1, C2, D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconicurv, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconicurv);
                        }
                    }
                    break;
                }
                break;

            case Circle:
                switch (typ2)
                {
                    case Line: {
                        intconiconi.SetReversedParameters(true);
                        intconiconi
                            .Perform(TheCurveTool.Line(C2), D2, TheCurveTool.Circle(C1), D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Circle: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi
                            .Perform(TheCurveTool.Circle(C1), D1, TheCurveTool.Circle(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Ellipse: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi
                            .Perform(TheCurveTool.Circle(C1), D1, TheCurveTool.Ellipse(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Parabola: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi
                            .Perform(TheCurveTool.Circle(C1), D1, TheCurveTool.Parabola(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Hyperbola: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi
                            .Perform(TheCurveTool.Circle(C1), D1, TheCurveTool.Hyperbola(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    default: {
                        intconicurv.SetReversedParameters(false);
                        intconicurv.Perform(TheCurveTool.Circle(C1), D1, C2, D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconicurv, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconicurv);
                        }
                    }

                    break;
                }
                break;

            case Ellipse:
                switch (typ2)
                {
                    case Line: {
                        intconiconi.SetReversedParameters(true);
                        intconiconi
                            .Perform(TheCurveTool.Line(C2), D2, TheCurveTool.Ellipse(C1), D1, TolConf, Tol);

                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Circle: {
                        intconiconi.SetReversedParameters(true);

                        intconiconi
                            .Perform(TheCurveTool.Circle(C2), D2, TheCurveTool.Ellipse(C1), D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Ellipse: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi
                            .Perform(TheCurveTool.Ellipse(C1), D1, TheCurveTool.Ellipse(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Parabola: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi
                            .Perform(TheCurveTool.Ellipse(C1), D1, TheCurveTool.Parabola(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Hyperbola: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi
                            .Perform(TheCurveTool.Ellipse(C1), D1, TheCurveTool.Hyperbola(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    default: {
                        intconicurv.SetReversedParameters(false);
                        intconicurv.Perform(TheCurveTool.Ellipse(C1), D1, C2, D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconicurv, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconicurv);
                        }
                    }
                    break;
                }
                break;

            case Parabola:
                switch (typ2)
                {
                    case Line: {
                        intconiconi.SetReversedParameters(true);
                        intconiconi
                            .Perform(TheCurveTool.Line(C2), D2, TheCurveTool.Parabola(C1), D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Circle: {
                        intconiconi.SetReversedParameters(true);
                        intconiconi
                            .Perform(TheCurveTool.Circle(C2), D2, TheCurveTool.Parabola(C1), D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Ellipse: {
                        intconiconi.SetReversedParameters(true);
                        intconiconi
                            .Perform(TheCurveTool.Ellipse(C2), D2, TheCurveTool.Parabola(C1), D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Parabola: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi
                            .Perform(TheCurveTool.Parabola(C1), D1, TheCurveTool.Parabola(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Hyperbola: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi
                            .Perform(TheCurveTool.Parabola(C1), D1, TheCurveTool.Hyperbola(C2), D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    default: {
                        intconicurv.SetReversedParameters(false);
                        intconicurv.Perform(TheCurveTool.Parabola(C1), D1, C2, D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconicurv, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconicurv);
                        }
                    }
                    break;
                }
                break;

            case Hyperbola:
                switch (typ2)
                {
                    case Line: {
                        intconiconi.SetReversedParameters(true);
                        intconiconi
                            .Perform(TheCurveTool.Line(C2), D2, TheCurveTool.Hyperbola(C1), D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Circle: {
                        intconiconi.SetReversedParameters(true);
                        intconiconi
                            .Perform(TheCurveTool.Circle(C2), D2, TheCurveTool.Hyperbola(C1), D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Ellipse: {
                        intconiconi.SetReversedParameters(true);
                        intconiconi
                            .Perform(TheCurveTool.Ellipse(C2), D2, TheCurveTool.Hyperbola(C1), D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Parabola: {
                        intconiconi.SetReversedParameters(true);
                        intconiconi
                            .Perform(TheCurveTool.Parabola(C2), D2, TheCurveTool.Hyperbola(C1), D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    case Hyperbola: {
                        intconiconi.SetReversedParameters(false);
                        intconiconi.Perform(TheCurveTool.Hyperbola(C1),
                                                                D1,
                                                                TheCurveTool.Hyperbola(C2),
                                                                D2,
                                                                TolConf,
                                                                Tol);
                        if (Composite)
                        {
                            this.Append(intconiconi, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconiconi);
                        }
                    }
                    break;

                    default: {
                        intconicurv.SetReversedParameters(false);
                        intconicurv.Perform(TheCurveTool.Hyperbola(C1), D1, C2, D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconicurv, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconicurv);
                        }
                    }
                    break;
                }
                break;

            default:
                switch (typ2)
                {
                    case Line: {
                        intconicurv.SetReversedParameters(true);
                        intconicurv.Perform(TheCurveTool.Line(C2), D2, C1, D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconicurv, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconicurv);
                        }
                    }
                    break;

                    case Circle: {
                        intconicurv.SetReversedParameters(true);
                        intconicurv.Perform(TheCurveTool.Circle(C2), D2, C1, D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconicurv, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconicurv);
                        }
                    }
                    break;

                    case Ellipse: {
                        intconicurv.SetReversedParameters(true);
                        intconicurv.Perform(TheCurveTool.Ellipse(C2), D2, C1, D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconicurv, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconicurv);
                        }
                    }
                    break;

                    case Parabola: {
                        intconicurv.SetReversedParameters(true);
                        intconicurv.Perform(TheCurveTool.Parabola(C2), D2, C1, D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconicurv, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconicurv);
                        }
                    }
                    break;

                    case Hyperbola: {
                        intconicurv.SetReversedParameters(true);
                        intconicurv.Perform(TheCurveTool.Hyperbola(C2), D2, C1, D1, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intconicurv, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intconicurv);
                        }
                    }
                    break;

                    default: {
                        intcurvcurv.SetReversedParameters(false);
                        intcurvcurv.Perform(C1, D1, C2, D2, TolConf, Tol);
                        if (Composite)
                        {
                            this.Append(intcurvcurv, param1inf, param1sup, param2inf, param2sup);
                        }
                        else
                        {
                            this.SetValues(intcurvcurv);
                        }
                        done = true;
                    }
                    break;
                }
                break;
        }
    }

    private void InternalCompositePerform_noRecurs(int NbInterC1, in TheCurve C1, int NumInterC1, NCollection_Array1<double> Tab1, in IntRes2d_Domain D1, int NbInterC2, in TheCurve C2, int NumInterC2, NCollection_Array1<double> Tab2, in IntRes2d_Domain D2, double TolConf, double Tol)
    {

        if (NumInterC2 > NbInterC2)
            return;

        IntRes2d_Domain DomainC1NumInter = new();
        IntRes2d_Domain DomainC2NumInter = new();

        //----------------------------------------------------------------------
        //-- Creation du domaine associe a la portion de C1
        //----------------------------------------------------------------------
        bool   DomainIsOK = true;
        double ParamInf, ParamSup;

        if (NbInterC1 > 1)
        {
            TheCurveTool.GetInterval(C1, NumInterC1, Tab1, out ParamInf, out ParamSup);
            //--------------------------------------------------------------
            //-- Verification : Domaine Inclu dans Intervalle de Definition
            //--------------------------------------------------------------

            double u;

            u = D1.FirstParameter();
            if (ParamInf < u)
            {
                ParamInf = u;
            }

            u = D1.LastParameter();
            if (ParamSup > u)
            {
                ParamSup = u;
            }

            if ((ParamSup - ParamInf) > 1e-10)
            {
                DomainC1NumInter.SetValues(TheCurveTool.Value(C1, ParamInf),
                                                                      ParamInf,
                                                                      D1.FirstTolerance(),
                                                                      TheCurveTool.Value(C1, ParamSup),
                                                                      ParamSup,
                                                                      D1.LastTolerance());
            }
            else
            {
                DomainIsOK = false;
            }
        }
        else
        {
            DomainC1NumInter = D1;
        }

        //----------------------------------------------------------------------
        //-- Creation du domaine associe a la portion de C2
        //----------------------------------------------------------------------
        if (NbInterC2 > 1)
        {
            TheCurveTool.GetInterval(C2, NumInterC2, Tab2, out ParamInf, out ParamSup);
            //--------------------------------------------------------------
            //-- Verification : Domaine Inclu dans Intervalle de Definition
            //--------------------------------------------------------------

            double u;

            u = D2.FirstParameter();
            if (ParamInf < u)
            {
                ParamInf = u;
            }
            u = D2.LastParameter();

            if (ParamSup > u)
            {
                ParamSup = u;
            }

            if ((ParamSup - ParamInf) > 1e-10)
            {
                DomainC2NumInter.SetValues(TheCurveTool.Value(C2, ParamInf),
                                                                      ParamInf,
                                                                      D2.FirstTolerance(),
                                                                      TheCurveTool.Value(C2, ParamSup),
                                                                      ParamSup,
                                                                      D2.LastTolerance());
            }
            else
            {
                DomainIsOK = false;
            }
        }
        else
        {
            DomainC2NumInter = D2;
        }

        if (DomainIsOK)
        {
            InternalPerform(C2, DomainC2NumInter, C1, DomainC1NumInter, TolConf, Tol, true);
        }
    }

    //-- C1 ou C2 sont des courbes composites
    //--

    private void InternalCompositePerform(in TheCurve C1, in IntRes2d_Domain D1, int XXXNumInterC1, int NbInterC1, NCollection_Array1<double> Tab1, in TheCurve C2, in IntRes2d_Domain D2, int XXXNumInterC2, int NbInterC2, NCollection_Array1<double> Tab2, double TolConf, double Tol, bool RecursOnC2)
    {

        int NumInterC2 = XXXNumInterC2;
        int NumInterC1 = XXXNumInterC1;

        //  bool Arret=false;

        if (NumInterC2 > NbInterC2)
            return;

        if (!RecursOnC2)
        {
            InternalCompositePerform_noRecurs(NbInterC1,
                                                                                C1,
                                                                                NumInterC1,
                                                                                Tab1,
                                                                                D1,
                                                                                NbInterC2,
                                                                                C2,
                                                                                NumInterC2,
                                                                                Tab2,
                                                                                D2,
                                                                                TolConf,
                                                                                Tol);
            return;
        }

        for (int i = NumInterC1; i <= NbInterC1; i++)
        {
            NumInterC1 = i;
            /*
                InternalCompositePerform(C2,D2,NumInterC2,NbInterC2,Tab2,
                            C1,D1,NumInterC1,NbInterC1,Tab1,
                            TolConf,Tol,false);
            */

            InternalCompositePerform_noRecurs(NbInterC2,
                                                                                C2,
                                                                                NumInterC2,
                                                                                Tab2,
                                                                                D2,
                                                                                NbInterC1,
                                                                                C1,
                                                                                NumInterC1,
                                                                                Tab1,
                                                                                D1,
                                                                                TolConf,
                                                                                Tol);
        }

        if (NumInterC2 < NbInterC2)
        {
            NumInterC2++;
            NumInterC1 = 1;

            InternalCompositePerform(C1,
                                                              D1,
                                                              NumInterC1,
                                                              NbInterC1,
                                                              Tab1,
                                                              C2,
                                                              D2,
                                                              NumInterC2,
                                                              NbInterC2,
                                                              Tab2,
                                                              TolConf,
                                                              Tol,
                                                              true);
        }
    }



    /// <summary>The minimum number of samples of the polygon intersection.</summary>
    public void SetMinNbSamples(int theMinNbSamples)
    {
        intcurvcurv.SetMinNbSamples(theMinNbSamples);
    }



    public int GetMinNbSamples()
    {
        return intcurvcurv.GetMinNbSamples();
    }
}
