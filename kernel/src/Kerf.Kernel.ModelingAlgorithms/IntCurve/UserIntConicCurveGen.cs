// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_UserIntConicCurveGen.gxx
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
/// The intersection between a conic and a parametric curve that may be a conic or composite: conic/conic pairs go
/// to <see cref="IntConicConic"/>, others to the implicit/parametric intersection, interval by interval
/// of continuity of the parametric curve.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_UserIntConicCurveGen.gxx as a generic class (decision 321); OCCT's instantiation
/// is <see cref="IntConicCurveOfGInter"/>.
/// </remarks>
internal class UserIntConicCurveGen<ThePCurve, ThePCurveTool, TheProjPCur, TheMyImpParTool>
    : Intersection2D
    where ThePCurveTool : ICurveCurveTool<ThePCurve>
    where TheProjPCur : IProjectOnPCurveTool<ThePCurve>
    where TheMyImpParTool : FunctionWithDerivative,
                            IMyImpParTool<TheMyImpParTool, IConicTool, ThePCurve>
{
    private double _param1inf;
    private double _param1sup;
    private double _param2inf;
    private double _param2sup;
    private readonly IntConicConic _intconiconi = new();
    private readonly IntConicCurveGen<ThePCurve, ThePCurveTool, TheProjPCur, TheMyImpParTool> _intconicurv =
        new();

    /// <summary>Empty constructor.</summary>
    public UserIntConicCurveGen()
    {
        _done = false;
    }

    /// <summary>Intersection between a line and a parametric curve.</summary>
    public UserIntConicCurveGen(in Line2D lin1, in TwoDDomain d1, in ThePCurve c2,
                                         in TwoDDomain d2, double tolConf, double tol)
    {
        Perform(lin1, d1, c2, d2, tolConf, tol);
    }

    /// <summary>Intersection between a circle and a parametric curve.</summary>
    public UserIntConicCurveGen(in Circle2D circ1, in TwoDDomain d1, in ThePCurve c2,
                                         in TwoDDomain d2, double tolConf, double tol)
    {
        Perform(circ1, d1, c2, d2, tolConf, tol);
    }

    /// <summary>Intersection between a parabola and a parametric curve.</summary>
    public UserIntConicCurveGen(in Parabola2D parab1, in TwoDDomain d1, in ThePCurve c2,
                                         in TwoDDomain d2, double tolConf, double tol)
    {
        Perform(parab1, d1, c2, d2, tolConf, tol);
    }

    /// <summary>Intersection between an ellipse and a parametric curve.</summary>
    public UserIntConicCurveGen(in Ellipse2D elips1, in TwoDDomain d1, in ThePCurve c2,
                                         in TwoDDomain d2, double tolConf, double tol)
    {
        Perform(elips1, d1, c2, d2, tolConf, tol);
    }

    /// <summary>Intersection between the main branch of a hyperbola and a parametric curve.</summary>
    public UserIntConicCurveGen(in Hyperbola2D hyper1, in TwoDDomain d1, in ThePCurve c2,
                                         in TwoDDomain d2, double tolConf, double tol)
    {
        Perform(hyper1, d1, c2, d2, tolConf, tol);
    }

    /// <summary>Intersection between a conic and a parametric curve.</summary>
    public void Perform(in Line2D lin1, in TwoDDomain d1, in ThePCurve c2, in TwoDDomain d2, double tolConf, double tol)
    {
        this.ResetFields();
        int nbInterC2 = ThePCurveTool.NbIntervals(c2);
        if (nbInterC2 > 1)
        {
            double paramInf, paramSup;
            double d2FirstParam = d2.FirstParameter();
            double d2LastParam  = d2.LastParameter();
            bool   ok           = true;
            _param1inf           = (d1.HasFirstPoint) ? d1.FirstParameter() : -global::Kerf.Engine.Foundation.Precision.Infinite();
            _param1sup           = (d1.HasLastPoint) ? d1.LastParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();
            _param2inf           = ThePCurveTool.FirstParameter(c2);
            _param2sup           = ThePCurveTool.LastParameter(c2);
            TwoDDomain domainC2NumInter = new();

            NCollection_Array1<double> tab2 = new(1, nbInterC2 + 1);
            ThePCurveTool.Intervals(c2, tab2);

            for (int numInterC2 = 1; ok && (numInterC2 <= nbInterC2); numInterC2++)
            {
                ThePCurveTool.GetInterval(c2, numInterC2, tab2, out paramInf, out paramSup);
                if ((paramInf > d2LastParam) || (paramSup < d2FirstParam))
                    ok = false;
                else
                {
                    if (paramInf < d2FirstParam)
                        paramInf = d2FirstParam;
                    if (paramSup > d2LastParam)
                        paramSup = d2LastParam;
                    if ((paramSup - paramInf) > RealEpsilon())
                    {
                        domainC2NumInter.SetValues(ThePCurveTool.Value(c2, paramInf),
                                                                              paramInf,
                                                                              d2.FirstTolerance(),
                                                                              ThePCurveTool.Value(c2, paramSup),
                                                                              paramSup,
                                                                              d2.LastTolerance());
                        InternalPerform(lin1, d1, c2, domainC2NumInter, tolConf, tol, true);
                    }
                }
            }
        }
        else
        {
            InternalPerform(lin1, d1, c2, d2, tolConf, tol, false);
        }
    }



    /// <summary>Intersection between a conic and a parametric curve.</summary>
    public void Perform(in Circle2D circ1, in TwoDDomain d1, in ThePCurve c2, in TwoDDomain d2, double tolConf, double tol)
    {
        this.ResetFields();
        int nbInterC2 = ThePCurveTool.NbIntervals(c2);
        if (nbInterC2 > 1)
        {
            double paramInf, paramSup;
            double d2FirstParam = d2.FirstParameter();
            double d2LastParam  = d2.LastParameter();
            bool   ok           = true;

            _param1inf = (d1.HasFirstPoint) ? d1.FirstParameter() : -global::Kerf.Engine.Foundation.Precision.Infinite();
            _param1sup = (d1.HasLastPoint) ? d1.LastParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();
            _param2inf = ThePCurveTool.FirstParameter(c2);
            _param2sup = ThePCurveTool.LastParameter(c2);
            TwoDDomain domainC2NumInter = new();

            NCollection_Array1<double> tab2 = new(1, nbInterC2 + 1);
            ThePCurveTool.Intervals(c2, tab2);

            for (int numInterC2 = 1; ok && (numInterC2 <= nbInterC2); numInterC2++)
            {
                ThePCurveTool.GetInterval(c2, numInterC2, tab2, out paramInf, out paramSup);
                if ((paramInf > d2LastParam) || (paramSup < d2FirstParam))
                    ok = false;
                else
                {
                    if (paramInf < d2FirstParam)
                        paramInf = d2FirstParam;
                    if (paramSup > d2LastParam)
                        paramSup = d2LastParam;
                    if ((paramSup - paramInf) > RealEpsilon())
                    {
                        domainC2NumInter.SetValues(ThePCurveTool.Value(c2, paramInf),
                                                                              paramInf,
                                                                              d2.FirstTolerance(),
                                                                              ThePCurveTool.Value(c2, paramSup),
                                                                              paramSup,
                                                                              d2.LastTolerance());
                        InternalPerform(circ1, d1, c2, domainC2NumInter, tolConf, tol, true);
                    }
                }
            }
        }
        else
        {
            InternalPerform(circ1, d1, c2, d2, tolConf, tol, false);
        }
    }



    /// <summary>Intersection between a conic and a parametric curve.</summary>
    public void Perform(in Parabola2D parab1, in TwoDDomain d1, in ThePCurve c2, in TwoDDomain d2, double tolConf, double tol)
    {
        this.ResetFields();
        int nbInterC2 = ThePCurveTool.NbIntervals(c2);
        if (nbInterC2 > 1)
        {
            double paramInf, paramSup;
            double d2FirstParam = d2.FirstParameter();
            double d2LastParam  = d2.LastParameter();
            bool   ok           = true;
            _param1inf           = (d1.HasFirstPoint) ? d1.FirstParameter() : -global::Kerf.Engine.Foundation.Precision.Infinite();
            _param1sup           = (d1.HasLastPoint) ? d1.LastParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();
            _param2inf           = ThePCurveTool.FirstParameter(c2);
            _param2sup           = ThePCurveTool.LastParameter(c2);
            TwoDDomain domainC2NumInter = new();

            NCollection_Array1<double> tab2 = new(1, nbInterC2 + 1);
            ThePCurveTool.Intervals(c2, tab2);

            for (int numInterC2 = 1; ok && (numInterC2 <= nbInterC2); numInterC2++)
            {
                ThePCurveTool.GetInterval(c2, numInterC2, tab2, out paramInf, out paramSup);
                if ((paramInf > d2LastParam) || (paramSup < d2FirstParam))
                    ok = false;
                else
                {
                    if (paramInf < d2FirstParam)
                        paramInf = d2FirstParam;
                    if (paramSup > d2LastParam)
                        paramSup = d2LastParam;
                    if ((paramSup - paramInf) > RealEpsilon())
                    {
                        domainC2NumInter.SetValues(ThePCurveTool.Value(c2, paramInf),
                                                                              paramInf,
                                                                              d2.FirstTolerance(),
                                                                              ThePCurveTool.Value(c2, paramSup),
                                                                              paramSup,
                                                                              d2.LastTolerance());
                        InternalPerform(parab1, d1, c2, domainC2NumInter, tolConf, tol, true);
                    }
                }
            }
        }
        else
        {
            InternalPerform(parab1, d1, c2, d2, tolConf, tol, false);
        }
    }



    /// <summary>Intersection between a conic and a parametric curve.</summary>
    public void Perform(in Ellipse2D elips1, in TwoDDomain d1, in ThePCurve c2, in TwoDDomain d2, double tolConf, double tol)
    {
        this.ResetFields();
        int nbInterC2 = ThePCurveTool.NbIntervals(c2);
        if (nbInterC2 > 1)
        {
            double paramInf, paramSup;
            double d2FirstParam = d2.FirstParameter();
            double d2LastParam  = d2.LastParameter();
            bool   ok           = true;
            _param1inf           = (d1.HasFirstPoint) ? d1.FirstParameter() : -global::Kerf.Engine.Foundation.Precision.Infinite();
            _param1sup           = (d1.HasLastPoint) ? d1.LastParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();
            _param2inf           = ThePCurveTool.FirstParameter(c2);
            _param2sup           = ThePCurveTool.LastParameter(c2);
            TwoDDomain domainC2NumInter = new();

            NCollection_Array1<double> tab2 = new(1, nbInterC2 + 1);
            ThePCurveTool.Intervals(c2, tab2);

            for (int numInterC2 = 1; ok && (numInterC2 <= nbInterC2); numInterC2++)
            {
                ThePCurveTool.GetInterval(c2, numInterC2, tab2, out paramInf, out paramSup);
                if ((paramInf > d2LastParam) || (paramSup < d2FirstParam))
                    ok = false;
                else
                {
                    if (paramInf < d2FirstParam)
                        paramInf = d2FirstParam;
                    if (paramSup > d2LastParam)
                        paramSup = d2LastParam;
                    if ((paramSup - paramInf) > RealEpsilon())
                    {
                        domainC2NumInter.SetValues(ThePCurveTool.Value(c2, paramInf),
                                                                              paramInf,
                                                                              d2.FirstTolerance(),
                                                                              ThePCurveTool.Value(c2, paramSup),
                                                                              paramSup,
                                                                              d2.LastTolerance());
                        InternalPerform(elips1, d1, c2, domainC2NumInter, tolConf, tol, true);
                    }
                }
            }
        }
        else
        {
            InternalPerform(elips1, d1, c2, d2, tolConf, tol, false);
        }
    }



    /// <summary>Intersection between a conic and a parametric curve.</summary>
    public void Perform(in Hyperbola2D hyper1, in TwoDDomain d1, in ThePCurve c2, in TwoDDomain d2, double tolConf, double tol)
    {
        this.ResetFields();
        int nbInterC2 = ThePCurveTool.NbIntervals(c2);
        if (nbInterC2 > 1)
        {
            double paramInf, paramSup;
            double d2FirstParam = d2.FirstParameter();
            double d2LastParam  = d2.LastParameter();
            bool   ok           = true;
            _param1inf           = (d1.HasFirstPoint) ? d1.FirstParameter() : -global::Kerf.Engine.Foundation.Precision.Infinite();
            _param1sup           = (d1.HasLastPoint) ? d1.LastParameter() : global::Kerf.Engine.Foundation.Precision.Infinite();
            _param2inf           = ThePCurveTool.FirstParameter(c2);
            _param2sup           = ThePCurveTool.LastParameter(c2);
            TwoDDomain domainC2NumInter = new();

            NCollection_Array1<double> tab2 = new(1, nbInterC2 + 1);
            ThePCurveTool.Intervals(c2, tab2);

            for (int numInterC2 = 1; ok && (numInterC2 <= nbInterC2); numInterC2++)
            {
                ThePCurveTool.GetInterval(c2, numInterC2, tab2, out paramInf, out paramSup);
                if ((paramInf > d2LastParam) || (paramSup < d2FirstParam))
                    ok = false;
                else
                {
                    if (paramInf < d2FirstParam)
                        paramInf = d2FirstParam;
                    if (paramSup > d2LastParam)
                        paramSup = d2LastParam;
                    if ((paramSup - paramInf) > RealEpsilon())
                    {
                        domainC2NumInter.SetValues(ThePCurveTool.Value(c2, paramInf),
                                                                              paramInf,
                                                                              d2.FirstTolerance(),
                                                                              ThePCurveTool.Value(c2, paramSup),
                                                                              paramSup,
                                                                              d2.LastTolerance());
                        InternalPerform(hyper1, d1, c2, domainC2NumInter, tolConf, tol, true);
                    }
                }
            }
        }
        else
        {
            InternalPerform(hyper1, d1, c2, d2, tolConf, tol, false);
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
    private void InternalPerform(in Line2D lin1, in TwoDDomain d1, in ThePCurve c2, in TwoDDomain d2, double tolConf, double tol, bool composite)
    {

        CurveKind typ2 = ThePCurveTool.GetType(c2);

        switch (typ2)
        {

            case Line: {
                _intconiconi.SetReversedParameters(false);
                _intconiconi.Perform(lin1, d1, ThePCurveTool.Line(c2), d2, tolConf, tol);
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
                _intconiconi.Perform(lin1, d1, ThePCurveTool.Circle(c2), d2, tolConf, tol);
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
                _intconiconi.Perform(lin1, d1, ThePCurveTool.Ellipse(c2), d2, tolConf, tol);
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
                _intconiconi.Perform(lin1, d1, ThePCurveTool.Parabola(c2), d2, tolConf, tol);
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
                _intconiconi.Perform(lin1, d1, ThePCurveTool.Hyperbola(c2), d2, tolConf, tol);
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
                _intconicurv.Perform(lin1, d1, c2, d2, tolConf, tol);
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
    }



    private void InternalPerform(in Circle2D circ1, in TwoDDomain d1, in ThePCurve c2, in TwoDDomain d2, double tolConf, double tol, bool composite)
    {

        CurveKind typ2 = ThePCurveTool.GetType(c2);

        switch (typ2)
        {

            case Line: {
                _intconiconi.SetReversedParameters(true);
                _intconiconi.Perform(ThePCurveTool.Line(c2), d2, circ1, d1, tolConf, tol);
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
                _intconiconi.Perform(circ1, d1, ThePCurveTool.Circle(c2), d2, tolConf, tol);
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
                _intconiconi.Perform(circ1, d1, ThePCurveTool.Ellipse(c2), d2, tolConf, tol);
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
                _intconiconi.Perform(circ1, d1, ThePCurveTool.Parabola(c2), d2, tolConf, tol);
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
                _intconiconi.Perform(circ1, d1, ThePCurveTool.Hyperbola(c2), d2, tolConf, tol);
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
                _intconicurv.Perform(circ1, d1, c2, d2, tolConf, tol);
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
    }



    private void InternalPerform(in Ellipse2D elips1, in TwoDDomain d1, in ThePCurve c2, in TwoDDomain d2, double tolConf, double tol, bool composite)
    {

        CurveKind typ2 = ThePCurveTool.GetType(c2);

        switch (typ2)
        {

            case Line: {
                _intconiconi.SetReversedParameters(true);
                _intconiconi.Perform(ThePCurveTool.Line(c2), d2, elips1, d1, tolConf, tol);
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
                _intconiconi.Perform(ThePCurveTool.Circle(c2), d2, elips1, d1, tolConf, tol);
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
                _intconiconi.Perform(elips1, d1, ThePCurveTool.Ellipse(c2), d2, tolConf, tol);
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
                _intconiconi.Perform(elips1, d1, ThePCurveTool.Parabola(c2), d2, tolConf, tol);
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
                _intconiconi.Perform(elips1, d1, ThePCurveTool.Hyperbola(c2), d2, tolConf, tol);
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
                _intconicurv.Perform(elips1, d1, c2, d2, tolConf, tol);
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
    }



    private void InternalPerform(in Parabola2D parab1, in TwoDDomain d1, in ThePCurve c2, in TwoDDomain d2, double tolConf, double tol, bool composite)
    {

        CurveKind typ2 = ThePCurveTool.GetType(c2);

        switch (typ2)
        {

            case Line: {
                _intconiconi.SetReversedParameters(true);
                _intconiconi.Perform(ThePCurveTool.Line(c2), d2, parab1, d1, tolConf, tol);
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
                _intconiconi.Perform(ThePCurveTool.Circle(c2), d2, parab1, d1, tolConf, tol);
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
                _intconiconi.Perform(ThePCurveTool.Ellipse(c2), d2, parab1, d1, tolConf, tol);
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
                _intconiconi.Perform(parab1, d1, ThePCurveTool.Parabola(c2), d2, tolConf, tol);
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
                _intconiconi.Perform(parab1, d1, ThePCurveTool.Hyperbola(c2), d2, tolConf, tol);
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
                _intconicurv.Perform(parab1, d1, c2, d2, tolConf, tol);
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
    }



    private void InternalPerform(in Hyperbola2D hyper1, in TwoDDomain d1, in ThePCurve c2, in TwoDDomain d2, double tolConf, double tol, bool composite)
    {

        CurveKind typ2 = ThePCurveTool.GetType(c2);

        switch (typ2)
        {

            case Line: {
                _intconiconi.SetReversedParameters(true);
                _intconiconi.Perform(ThePCurveTool.Line(c2), d2, hyper1, d1, tolConf, tol);
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
                _intconiconi.Perform(ThePCurveTool.Circle(c2), d2, hyper1, d1, tolConf, tol);
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
                _intconiconi.Perform(ThePCurveTool.Ellipse(c2), d2, hyper1, d1, tolConf, tol);
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
                _intconiconi.Perform(ThePCurveTool.Parabola(c2), d2, hyper1, d1, tolConf, tol);
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
                _intconiconi.Perform(hyper1, d1, ThePCurveTool.Hyperbola(c2), d2, tolConf, tol);
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
                _intconicurv.Perform(hyper1, d1, c2, d2, tolConf, tol);
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
    }

    //--------------------------------------------------------------------------------
}
