// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_UserIntConicCurveGen.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The intersection between a conic and a parametric curve that may be a conic or composite: conic/conic pairs go
/// to <see cref="IntCurve_IntConicConic"/>, others to the implicit/parametric intersection, interval by interval
/// of continuity of the parametric curve.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_UserIntConicCurveGen.gxx as a generic class (decision 321); OCCT's instantiation
/// is <see cref="Geom2dInt_IntConicCurveOfGInter"/>.
/// </remarks>
public class IntCurve_UserIntConicCurveGen<ThePCurve, ThePCurveTool, TheProjPCur, TheMyImpParTool>
    : IntRes2d_Intersection
    where ThePCurveTool : IIntCurve_CurveTool<ThePCurve>
    where TheProjPCur : IIntImpParGen_ProjectOnPCurveTool<ThePCurve>
    where TheMyImpParTool : FunctionWithDerivative,
                            IIntImpParGen_MyImpParTool<TheMyImpParTool, IntCurve_IConicTool, ThePCurve>
{
    private double param1inf;
    private double param1sup;
    private double param2inf;
    private double param2sup;
    private readonly IntCurve_IntConicConic intconiconi = new();
    private readonly IntCurve_IntConicCurveGen<ThePCurve, ThePCurveTool, TheProjPCur, TheMyImpParTool> intconicurv =
        new();

    /// <summary>Empty constructor.</summary>
    public IntCurve_UserIntConicCurveGen()
    {
        done = false;
    }

    /// <summary>Intersection between a line and a parametric curve.</summary>
    public IntCurve_UserIntConicCurveGen(in Line2D Lin1, in IntRes2d_Domain D1, in ThePCurve C2,
                                         in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        Perform(Lin1, D1, C2, D2, TolConf, Tol);
    }

    /// <summary>Intersection between a circle and a parametric curve.</summary>
    public IntCurve_UserIntConicCurveGen(in Circle2D Circ1, in IntRes2d_Domain D1, in ThePCurve C2,
                                         in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        Perform(Circ1, D1, C2, D2, TolConf, Tol);
    }

    /// <summary>Intersection between a parabola and a parametric curve.</summary>
    public IntCurve_UserIntConicCurveGen(in Parabola2D Parab1, in IntRes2d_Domain D1, in ThePCurve C2,
                                         in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        Perform(Parab1, D1, C2, D2, TolConf, Tol);
    }

    /// <summary>Intersection between an ellipse and a parametric curve.</summary>
    public IntCurve_UserIntConicCurveGen(in Ellipse2D Elips1, in IntRes2d_Domain D1, in ThePCurve C2,
                                         in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        Perform(Elips1, D1, C2, D2, TolConf, Tol);
    }

    /// <summary>Intersection between the main branch of a hyperbola and a parametric curve.</summary>
    public IntCurve_UserIntConicCurveGen(in Hyperbola2D Hyper1, in IntRes2d_Domain D1, in ThePCurve C2,
                                         in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        Perform(Hyper1, D1, C2, D2, TolConf, Tol);
    }

    /// <summary>Intersection between a conic and a parametric curve.</summary>
    public void Perform(in Line2D Lin1, in IntRes2d_Domain D1, in ThePCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        this.ResetFields();
        int NbInterC2 = ThePCurveTool.NbIntervals(C2);
        if (NbInterC2 > 1)
        {
            double ParamInf, ParamSup;
            double D2FirstParam = D2.FirstParameter();
            double D2LastParam  = D2.LastParameter();
            bool   Ok           = true;
            param1inf           = (D1.HasFirstPoint()) ? D1.FirstParameter() : -global::Kerf.Engine.Foundation.Precision.Infinite();
            param1sup           = (D1.HasLastPoint()) ? D1.LastParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();
            param2inf           = ThePCurveTool.FirstParameter(C2);
            param2sup           = ThePCurveTool.LastParameter(C2);
            IntRes2d_Domain DomainC2NumInter = new();

            NCollection_Array1<double> Tab2 = new(1, NbInterC2 + 1);
            ThePCurveTool.Intervals(C2, Tab2);

            for (int NumInterC2 = 1; Ok && (NumInterC2 <= NbInterC2); NumInterC2++)
            {
                ThePCurveTool.GetInterval(C2, NumInterC2, Tab2, out ParamInf, out ParamSup);
                if ((ParamInf > D2LastParam) || (ParamSup < D2FirstParam))
                    Ok = false;
                else
                {
                    if (ParamInf < D2FirstParam)
                        ParamInf = D2FirstParam;
                    if (ParamSup > D2LastParam)
                        ParamSup = D2LastParam;
                    if ((ParamSup - ParamInf) > RealEpsilon())
                    {
                        DomainC2NumInter.SetValues(ThePCurveTool.Value(C2, ParamInf),
                                                                              ParamInf,
                                                                              D2.FirstTolerance(),
                                                                              ThePCurveTool.Value(C2, ParamSup),
                                                                              ParamSup,
                                                                              D2.LastTolerance());
                        InternalPerform(Lin1, D1, C2, DomainC2NumInter, TolConf, Tol, true);
                    }
                }
            }
        }
        else
        {
            InternalPerform(Lin1, D1, C2, D2, TolConf, Tol, false);
        }
    }



    /// <summary>Intersection between a conic and a parametric curve.</summary>
    public void Perform(in Circle2D Circ1, in IntRes2d_Domain D1, in ThePCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        this.ResetFields();
        int NbInterC2 = ThePCurveTool.NbIntervals(C2);
        if (NbInterC2 > 1)
        {
            double ParamInf, ParamSup;
            double D2FirstParam = D2.FirstParameter();
            double D2LastParam  = D2.LastParameter();
            bool   Ok           = true;

            param1inf = (D1.HasFirstPoint()) ? D1.FirstParameter() : -global::Kerf.Engine.Foundation.Precision.Infinite();
            param1sup = (D1.HasLastPoint()) ? D1.LastParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();
            param2inf = ThePCurveTool.FirstParameter(C2);
            param2sup = ThePCurveTool.LastParameter(C2);
            IntRes2d_Domain DomainC2NumInter = new();

            NCollection_Array1<double> Tab2 = new(1, NbInterC2 + 1);
            ThePCurveTool.Intervals(C2, Tab2);

            for (int NumInterC2 = 1; Ok && (NumInterC2 <= NbInterC2); NumInterC2++)
            {
                ThePCurveTool.GetInterval(C2, NumInterC2, Tab2, out ParamInf, out ParamSup);
                if ((ParamInf > D2LastParam) || (ParamSup < D2FirstParam))
                    Ok = false;
                else
                {
                    if (ParamInf < D2FirstParam)
                        ParamInf = D2FirstParam;
                    if (ParamSup > D2LastParam)
                        ParamSup = D2LastParam;
                    if ((ParamSup - ParamInf) > RealEpsilon())
                    {
                        DomainC2NumInter.SetValues(ThePCurveTool.Value(C2, ParamInf),
                                                                              ParamInf,
                                                                              D2.FirstTolerance(),
                                                                              ThePCurveTool.Value(C2, ParamSup),
                                                                              ParamSup,
                                                                              D2.LastTolerance());
                        InternalPerform(Circ1, D1, C2, DomainC2NumInter, TolConf, Tol, true);
                    }
                }
            }
        }
        else
        {
            InternalPerform(Circ1, D1, C2, D2, TolConf, Tol, false);
        }
    }



    /// <summary>Intersection between a conic and a parametric curve.</summary>
    public void Perform(in Parabola2D Parab1, in IntRes2d_Domain D1, in ThePCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        this.ResetFields();
        int NbInterC2 = ThePCurveTool.NbIntervals(C2);
        if (NbInterC2 > 1)
        {
            double ParamInf, ParamSup;
            double D2FirstParam = D2.FirstParameter();
            double D2LastParam  = D2.LastParameter();
            bool   Ok           = true;
            param1inf           = (D1.HasFirstPoint()) ? D1.FirstParameter() : -global::Kerf.Engine.Foundation.Precision.Infinite();
            param1sup           = (D1.HasLastPoint()) ? D1.LastParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();
            param2inf           = ThePCurveTool.FirstParameter(C2);
            param2sup           = ThePCurveTool.LastParameter(C2);
            IntRes2d_Domain DomainC2NumInter = new();

            NCollection_Array1<double> Tab2 = new(1, NbInterC2 + 1);
            ThePCurveTool.Intervals(C2, Tab2);

            for (int NumInterC2 = 1; Ok && (NumInterC2 <= NbInterC2); NumInterC2++)
            {
                ThePCurveTool.GetInterval(C2, NumInterC2, Tab2, out ParamInf, out ParamSup);
                if ((ParamInf > D2LastParam) || (ParamSup < D2FirstParam))
                    Ok = false;
                else
                {
                    if (ParamInf < D2FirstParam)
                        ParamInf = D2FirstParam;
                    if (ParamSup > D2LastParam)
                        ParamSup = D2LastParam;
                    if ((ParamSup - ParamInf) > RealEpsilon())
                    {
                        DomainC2NumInter.SetValues(ThePCurveTool.Value(C2, ParamInf),
                                                                              ParamInf,
                                                                              D2.FirstTolerance(),
                                                                              ThePCurveTool.Value(C2, ParamSup),
                                                                              ParamSup,
                                                                              D2.LastTolerance());
                        InternalPerform(Parab1, D1, C2, DomainC2NumInter, TolConf, Tol, true);
                    }
                }
            }
        }
        else
        {
            InternalPerform(Parab1, D1, C2, D2, TolConf, Tol, false);
        }
    }



    /// <summary>Intersection between a conic and a parametric curve.</summary>
    public void Perform(in Ellipse2D Elips1, in IntRes2d_Domain D1, in ThePCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        this.ResetFields();
        int NbInterC2 = ThePCurveTool.NbIntervals(C2);
        if (NbInterC2 > 1)
        {
            double ParamInf, ParamSup;
            double D2FirstParam = D2.FirstParameter();
            double D2LastParam  = D2.LastParameter();
            bool   Ok           = true;
            param1inf           = (D1.HasFirstPoint()) ? D1.FirstParameter() : -global::Kerf.Engine.Foundation.Precision.Infinite();
            param1sup           = (D1.HasLastPoint()) ? D1.LastParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();
            param2inf           = ThePCurveTool.FirstParameter(C2);
            param2sup           = ThePCurveTool.LastParameter(C2);
            IntRes2d_Domain DomainC2NumInter = new();

            NCollection_Array1<double> Tab2 = new(1, NbInterC2 + 1);
            ThePCurveTool.Intervals(C2, Tab2);

            for (int NumInterC2 = 1; Ok && (NumInterC2 <= NbInterC2); NumInterC2++)
            {
                ThePCurveTool.GetInterval(C2, NumInterC2, Tab2, out ParamInf, out ParamSup);
                if ((ParamInf > D2LastParam) || (ParamSup < D2FirstParam))
                    Ok = false;
                else
                {
                    if (ParamInf < D2FirstParam)
                        ParamInf = D2FirstParam;
                    if (ParamSup > D2LastParam)
                        ParamSup = D2LastParam;
                    if ((ParamSup - ParamInf) > RealEpsilon())
                    {
                        DomainC2NumInter.SetValues(ThePCurveTool.Value(C2, ParamInf),
                                                                              ParamInf,
                                                                              D2.FirstTolerance(),
                                                                              ThePCurveTool.Value(C2, ParamSup),
                                                                              ParamSup,
                                                                              D2.LastTolerance());
                        InternalPerform(Elips1, D1, C2, DomainC2NumInter, TolConf, Tol, true);
                    }
                }
            }
        }
        else
        {
            InternalPerform(Elips1, D1, C2, D2, TolConf, Tol, false);
        }
    }



    /// <summary>Intersection between a conic and a parametric curve.</summary>
    public void Perform(in Hyperbola2D Hyper1, in IntRes2d_Domain D1, in ThePCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        this.ResetFields();
        int NbInterC2 = ThePCurveTool.NbIntervals(C2);
        if (NbInterC2 > 1)
        {
            double ParamInf, ParamSup;
            double D2FirstParam = D2.FirstParameter();
            double D2LastParam  = D2.LastParameter();
            bool   Ok           = true;
            param1inf           = (D1.HasFirstPoint()) ? D1.FirstParameter() : -global::Kerf.Engine.Foundation.Precision.Infinite();
            param1sup           = (D1.HasLastPoint()) ? D1.LastParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();
            param2inf           = ThePCurveTool.FirstParameter(C2);
            param2sup           = ThePCurveTool.LastParameter(C2);
            IntRes2d_Domain DomainC2NumInter = new();

            NCollection_Array1<double> Tab2 = new(1, NbInterC2 + 1);
            ThePCurveTool.Intervals(C2, Tab2);

            for (int NumInterC2 = 1; Ok && (NumInterC2 <= NbInterC2); NumInterC2++)
            {
                ThePCurveTool.GetInterval(C2, NumInterC2, Tab2, out ParamInf, out ParamSup);
                if ((ParamInf > D2LastParam) || (ParamSup < D2FirstParam))
                    Ok = false;
                else
                {
                    if (ParamInf < D2FirstParam)
                        ParamInf = D2FirstParam;
                    if (ParamSup > D2LastParam)
                        ParamSup = D2LastParam;
                    if ((ParamSup - ParamInf) > RealEpsilon())
                    {
                        DomainC2NumInter.SetValues(ThePCurveTool.Value(C2, ParamInf),
                                                                              ParamInf,
                                                                              D2.FirstTolerance(),
                                                                              ThePCurveTool.Value(C2, ParamSup),
                                                                              ParamSup,
                                                                              D2.LastTolerance());
                        InternalPerform(Hyper1, D1, C2, DomainC2NumInter, TolConf, Tol, true);
                    }
                }
            }
        }
        else
        {
            InternalPerform(Hyper1, D1, C2, D2, TolConf, Tol, false);
        }
    }

    //----------------------------------------------------------------------
    //-- InternalPerform
    //-- Suppose des Courbes Lin...Hypr
    //-- Si Composite == True
    //--     Les Resultats sont Ajoutes
    //-- Sinon
    //--     Les Resultats sont Copies
    //----------------------------------------------------------------------
    private void InternalPerform(in Line2D Lin1, in IntRes2d_Domain D1, in ThePCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol, bool Composite)
    {

        CurveKind typ2 = ThePCurveTool.GetType(C2);

        switch (typ2)
        {

            case Line: {
                intconiconi.SetReversedParameters(false);
                intconiconi.Perform(Lin1, D1, ThePCurveTool.Line(C2), D2, TolConf, Tol);
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
                intconiconi.Perform(Lin1, D1, ThePCurveTool.Circle(C2), D2, TolConf, Tol);
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
                intconiconi.Perform(Lin1, D1, ThePCurveTool.Ellipse(C2), D2, TolConf, Tol);
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
                intconiconi.Perform(Lin1, D1, ThePCurveTool.Parabola(C2), D2, TolConf, Tol);
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
                intconiconi.Perform(Lin1, D1, ThePCurveTool.Hyperbola(C2), D2, TolConf, Tol);
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
                intconicurv.Perform(Lin1, D1, C2, D2, TolConf, Tol);
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
    }



    private void InternalPerform(in Circle2D Circ1, in IntRes2d_Domain D1, in ThePCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol, bool Composite)
    {

        CurveKind typ2 = ThePCurveTool.GetType(C2);

        switch (typ2)
        {

            case Line: {
                intconiconi.SetReversedParameters(true);
                intconiconi.Perform(ThePCurveTool.Line(C2), D2, Circ1, D1, TolConf, Tol);
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
                intconiconi.Perform(Circ1, D1, ThePCurveTool.Circle(C2), D2, TolConf, Tol);
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
                intconiconi.Perform(Circ1, D1, ThePCurveTool.Ellipse(C2), D2, TolConf, Tol);
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
                intconiconi.Perform(Circ1, D1, ThePCurveTool.Parabola(C2), D2, TolConf, Tol);
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
                intconiconi.Perform(Circ1, D1, ThePCurveTool.Hyperbola(C2), D2, TolConf, Tol);
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
                intconicurv.Perform(Circ1, D1, C2, D2, TolConf, Tol);
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
    }



    private void InternalPerform(in Ellipse2D Elips1, in IntRes2d_Domain D1, in ThePCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol, bool Composite)
    {

        CurveKind typ2 = ThePCurveTool.GetType(C2);

        switch (typ2)
        {

            case Line: {
                intconiconi.SetReversedParameters(true);
                intconiconi.Perform(ThePCurveTool.Line(C2), D2, Elips1, D1, TolConf, Tol);
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
                intconiconi.Perform(ThePCurveTool.Circle(C2), D2, Elips1, D1, TolConf, Tol);
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
                intconiconi.Perform(Elips1, D1, ThePCurveTool.Ellipse(C2), D2, TolConf, Tol);
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
                intconiconi.Perform(Elips1, D1, ThePCurveTool.Parabola(C2), D2, TolConf, Tol);
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
                intconiconi.Perform(Elips1, D1, ThePCurveTool.Hyperbola(C2), D2, TolConf, Tol);
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
                intconicurv.Perform(Elips1, D1, C2, D2, TolConf, Tol);
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
    }



    private void InternalPerform(in Parabola2D Parab1, in IntRes2d_Domain D1, in ThePCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol, bool Composite)
    {

        CurveKind typ2 = ThePCurveTool.GetType(C2);

        switch (typ2)
        {

            case Line: {
                intconiconi.SetReversedParameters(true);
                intconiconi.Perform(ThePCurveTool.Line(C2), D2, Parab1, D1, TolConf, Tol);
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
                intconiconi.Perform(ThePCurveTool.Circle(C2), D2, Parab1, D1, TolConf, Tol);
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
                intconiconi.Perform(ThePCurveTool.Ellipse(C2), D2, Parab1, D1, TolConf, Tol);
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
                intconiconi.Perform(Parab1, D1, ThePCurveTool.Parabola(C2), D2, TolConf, Tol);
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
                intconiconi.Perform(Parab1, D1, ThePCurveTool.Hyperbola(C2), D2, TolConf, Tol);
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
                intconicurv.Perform(Parab1, D1, C2, D2, TolConf, Tol);
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
    }



    private void InternalPerform(in Hyperbola2D Hyper1, in IntRes2d_Domain D1, in ThePCurve C2, in IntRes2d_Domain D2, double TolConf, double Tol, bool Composite)
    {

        CurveKind typ2 = ThePCurveTool.GetType(C2);

        switch (typ2)
        {

            case Line: {
                intconiconi.SetReversedParameters(true);
                intconiconi.Perform(ThePCurveTool.Line(C2), D2, Hyper1, D1, TolConf, Tol);
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
                intconiconi.Perform(ThePCurveTool.Circle(C2), D2, Hyper1, D1, TolConf, Tol);
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
                intconiconi.Perform(ThePCurveTool.Ellipse(C2), D2, Hyper1, D1, TolConf, Tol);
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
                intconiconi.Perform(ThePCurveTool.Parabola(C2), D2, Hyper1, D1, TolConf, Tol);
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
                intconiconi.Perform(Hyper1, D1, ThePCurveTool.Hyperbola(C2), D2, TolConf, Tol);
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
                intconicurv.Perform(Hyper1, D1, C2, D2, TolConf, Tol);
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
    }

    //--------------------------------------------------------------------------------
}
