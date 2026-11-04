// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicCurveGen.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicCurveGen.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>
/// The intersection between a conic (seen as an implicit curve) and a bounded parametric curve.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_IntConicCurveGen.gxx as a generic class (decision 321). Its body names the
/// generic intersector <see cref="Intersector{ImpTool, ParCurve, ParTool, ProjectOnPCurveTool, MyImpParTool}"/>
/// over <see cref="IConicTool"/> directly; OCCT's instantiation is
/// <see cref="TheIntConicCurveOfGInter"/>.
/// </remarks>
internal class IntConicCurveGen<ThePCurve, ThePCurveTool, TheProjPCur, TheMyImpParTool> : Intersection2D
    where ThePCurveTool : ICurveCurveTool<ThePCurve>
    where TheProjPCur : IProjectOnPCurveTool<ThePCurve>
    where TheMyImpParTool : FunctionWithDerivative,
                            IMyImpParTool<TheMyImpParTool, IConicTool, ThePCurve>
{
    /// <summary>Empty constructor.</summary>
    public IntConicCurveGen()
    {
    }

    /// <summary>Intersection between a line and a parametric curve.</summary>
    public IntConicCurveGen(in Line2D l, in TwoDDomain d1, in ThePCurve pCurve,
                                     in TwoDDomain d2, double tolConf, double tol)
    {
        Perform(new IConicTool(l), d1, pCurve, d2, tolConf, tol);
    }

    /// <summary>Intersection between a circle and a parametric curve.</summary>
    public IntConicCurveGen(in Circle2D c, in TwoDDomain d1, in ThePCurve pCurve,
                                     in TwoDDomain d2, double tolConf, double tol)
    {
        if (!d1.IsClosed)
        {
            TwoDDomain d = d1;
            d.SetEquivalentParameters(d1.FirstParameter(), d1.FirstParameter() + M_PI + M_PI);
            Perform(new IConicTool(c), d, pCurve, d2, tolConf, tol);
            //-- throw Standard_ConstructionError("Domaine incorrect");
        }
        else
        {
            Perform(new IConicTool(c), d1, pCurve, d2, tolConf, tol);
        }
    }

    /// <summary>Intersection between an ellipse and a parametric curve.</summary>
    public IntConicCurveGen(in Ellipse2D e, in TwoDDomain d1, in ThePCurve pCurve,
                                     in TwoDDomain d2, double tolConf, double tol)
    {
        if (!d1.IsClosed)
        {
            TwoDDomain d = d1;
            d.SetEquivalentParameters(d1.FirstParameter(), d1.FirstParameter() + M_PI + M_PI);
            Perform(new IConicTool(e), d, pCurve, d2, tolConf, tol);
            //-- throw Standard_ConstructionError("Domaine incorrect");
        }
        else
        {
            Perform(new IConicTool(e), d1, pCurve, d2, tolConf, tol);
        }
    }

    /// <summary>Intersection between a parabola and a parametric curve.</summary>
    public IntConicCurveGen(in Parabola2D prb, in TwoDDomain d1, in ThePCurve pCurve,
                                     in TwoDDomain d2, double tolConf, double tol)
    {
        Perform(new IConicTool(prb), d1, pCurve, d2, tolConf, tol);
    }

    /// <summary>Intersection between the main branch of a hyperbola and a parametric curve.</summary>
    public IntConicCurveGen(in Hyperbola2D h, in TwoDDomain d1, in ThePCurve pCurve,
                                     in TwoDDomain d2, double tolConf, double tol)
    {
        Perform(new IConicTool(h), d1, pCurve, d2, tolConf, tol);
    }

    /// <summary>Intersection between a line and a parametric curve.</summary>
    public void Perform(in Line2D l, in TwoDDomain d1, in ThePCurve pCurve, in TwoDDomain d2,
                        double tolConf, double tol)
    {
        Perform(new IConicTool(l), d1, pCurve, d2, tolConf, tol);
    }

    /// <summary>
    /// Intersection between a circle and a parametric curve. A domain of the circle that is not closed is closed
    /// with a period starting at its first parameter.
    /// </summary>
    public void Perform(in Circle2D c, in TwoDDomain d1, in ThePCurve pCurve, in TwoDDomain d2,
                        double tolConf, double tol)
    {
        if (!d1.IsClosed)
        {
            TwoDDomain d = d1;
            d.SetEquivalentParameters(d1.FirstParameter(), d1.FirstParameter() + M_PI + M_PI);
            Perform(new IConicTool(c), d, pCurve, d2, tolConf, tol);
            //-- throw Standard_ConstructionError("Domaine incorrect");
        }
        else
        {
            Perform(new IConicTool(c), d1, pCurve, d2, tolConf, tol);
        }
    }

    /// <summary>
    /// Intersection between an ellipse and a parametric curve. A domain of the ellipse that is not closed is
    /// closed with a period starting at its first parameter.
    /// </summary>
    public void Perform(in Ellipse2D e, in TwoDDomain d1, in ThePCurve pCurve, in TwoDDomain d2,
                        double tolConf, double tol)
    {
        if (!d1.IsClosed)
        {
            TwoDDomain d = d1;
            d.SetEquivalentParameters(d1.FirstParameter(), d1.FirstParameter() + M_PI + M_PI);
            Perform(new IConicTool(e), d, pCurve, d2, tolConf, tol);
        }
        else
        {
            Perform(new IConicTool(e), d1, pCurve, d2, tolConf, tol);
        }
    }

    /// <summary>Intersection between a parabola and a parametric curve.</summary>
    public void Perform(in Parabola2D prb, in TwoDDomain d1, in ThePCurve pCurve, in TwoDDomain d2,
                        double tolConf, double tol)
    {
        Perform(new IConicTool(prb), d1, pCurve, d2, tolConf, tol);
    }

    /// <summary>Intersection between the main branch of a hyperbola and a parametric curve.</summary>
    public void Perform(in Hyperbola2D h, in TwoDDomain d1, in ThePCurve pCurve, in TwoDDomain d2,
                        double tolConf, double tol)
    {
        Perform(new IConicTool(h), d1, pCurve, d2, tolConf, tol);
    }

    private void Perform(in IConicTool iCurve, in TwoDDomain d1, in ThePCurve pCurve,
                         in TwoDDomain d2, double tolConf, double tol)
    {
        Intersector<IConicTool, ThePCurve, ThePCurveTool, TheProjPCur, TheMyImpParTool>
            myintersection = new();
        myintersection.SetReversedParameters(ReversedParameters());
        myintersection.Perform(iCurve, d1, pCurve, d2, tolConf, tol);
        this.SetValues(myintersection);
    }
}
