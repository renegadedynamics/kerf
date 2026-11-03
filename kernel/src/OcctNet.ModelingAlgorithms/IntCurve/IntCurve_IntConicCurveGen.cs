// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicCurveGen.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntConicCurveGen.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The intersection between a conic (seen as an implicit curve) and a bounded parametric curve.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_IntConicCurveGen.gxx as a generic class (decision 321). Its body names the
/// generic intersector <see cref="IntImpParGen_Intersector{ImpTool, ParCurve, ParTool, ProjectOnPCurveTool, MyImpParTool}"/>
/// over <see cref="IntCurve_IConicTool"/> directly; OCCT's instantiation is
/// <see cref="Geom2dInt_TheIntConicCurveOfGInter"/>.
/// </remarks>
public class IntCurve_IntConicCurveGen<ThePCurve, ThePCurveTool, TheProjPCur, TheMyImpParTool> : IntRes2d_Intersection
    where ThePCurveTool : IIntCurve_CurveTool<ThePCurve>
    where TheProjPCur : IIntImpParGen_ProjectOnPCurveTool<ThePCurve>
    where TheMyImpParTool : FunctionWithDerivative,
                            IIntImpParGen_MyImpParTool<TheMyImpParTool, IntCurve_IConicTool, ThePCurve>
{
    /// <summary>Empty constructor.</summary>
    public IntCurve_IntConicCurveGen()
    {
    }

    /// <summary>Intersection between a line and a parametric curve.</summary>
    public IntCurve_IntConicCurveGen(in Line2D L, in IntRes2d_Domain D1, in ThePCurve PCurve,
                                     in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        Perform(new IntCurve_IConicTool(L), D1, PCurve, D2, TolConf, Tol);
    }

    /// <summary>Intersection between a circle and a parametric curve.</summary>
    public IntCurve_IntConicCurveGen(in Circle2D C, in IntRes2d_Domain D1, in ThePCurve PCurve,
                                     in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        if (!D1.IsClosed())
        {
            IntRes2d_Domain D = D1;
            D.SetEquivalentParameters(D1.FirstParameter(), D1.FirstParameter() + M_PI + M_PI);
            Perform(new IntCurve_IConicTool(C), D, PCurve, D2, TolConf, Tol);
            //-- throw Standard_ConstructionError("Domaine incorrect");
        }
        else
        {
            Perform(new IntCurve_IConicTool(C), D1, PCurve, D2, TolConf, Tol);
        }
    }

    /// <summary>Intersection between an ellipse and a parametric curve.</summary>
    public IntCurve_IntConicCurveGen(in Ellipse2D E, in IntRes2d_Domain D1, in ThePCurve PCurve,
                                     in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        if (!D1.IsClosed())
        {
            IntRes2d_Domain D = D1;
            D.SetEquivalentParameters(D1.FirstParameter(), D1.FirstParameter() + M_PI + M_PI);
            Perform(new IntCurve_IConicTool(E), D, PCurve, D2, TolConf, Tol);
            //-- throw Standard_ConstructionError("Domaine incorrect");
        }
        else
        {
            Perform(new IntCurve_IConicTool(E), D1, PCurve, D2, TolConf, Tol);
        }
    }

    /// <summary>Intersection between a parabola and a parametric curve.</summary>
    public IntCurve_IntConicCurveGen(in Parabola2D Prb, in IntRes2d_Domain D1, in ThePCurve PCurve,
                                     in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        Perform(new IntCurve_IConicTool(Prb), D1, PCurve, D2, TolConf, Tol);
    }

    /// <summary>Intersection between the main branch of a hyperbola and a parametric curve.</summary>
    public IntCurve_IntConicCurveGen(in Hyperbola2D H, in IntRes2d_Domain D1, in ThePCurve PCurve,
                                     in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        Perform(new IntCurve_IConicTool(H), D1, PCurve, D2, TolConf, Tol);
    }

    /// <summary>Intersection between a line and a parametric curve.</summary>
    public void Perform(in Line2D L, in IntRes2d_Domain D1, in ThePCurve PCurve, in IntRes2d_Domain D2,
                        double TolConf, double Tol)
    {
        Perform(new IntCurve_IConicTool(L), D1, PCurve, D2, TolConf, Tol);
    }

    /// <summary>
    /// Intersection between a circle and a parametric curve. A domain of the circle that is not closed is closed
    /// with a period starting at its first parameter.
    /// </summary>
    public void Perform(in Circle2D C, in IntRes2d_Domain D1, in ThePCurve PCurve, in IntRes2d_Domain D2,
                        double TolConf, double Tol)
    {
        if (!D1.IsClosed())
        {
            IntRes2d_Domain D = D1;
            D.SetEquivalentParameters(D1.FirstParameter(), D1.FirstParameter() + M_PI + M_PI);
            Perform(new IntCurve_IConicTool(C), D, PCurve, D2, TolConf, Tol);
            //-- throw Standard_ConstructionError("Domaine incorrect");
        }
        else
        {
            Perform(new IntCurve_IConicTool(C), D1, PCurve, D2, TolConf, Tol);
        }
    }

    /// <summary>
    /// Intersection between an ellipse and a parametric curve. A domain of the ellipse that is not closed is
    /// closed with a period starting at its first parameter.
    /// </summary>
    public void Perform(in Ellipse2D E, in IntRes2d_Domain D1, in ThePCurve PCurve, in IntRes2d_Domain D2,
                        double TolConf, double Tol)
    {
        if (!D1.IsClosed())
        {
            IntRes2d_Domain D = D1;
            D.SetEquivalentParameters(D1.FirstParameter(), D1.FirstParameter() + M_PI + M_PI);
            Perform(new IntCurve_IConicTool(E), D, PCurve, D2, TolConf, Tol);
        }
        else
        {
            Perform(new IntCurve_IConicTool(E), D1, PCurve, D2, TolConf, Tol);
        }
    }

    /// <summary>Intersection between a parabola and a parametric curve.</summary>
    public void Perform(in Parabola2D Prb, in IntRes2d_Domain D1, in ThePCurve PCurve, in IntRes2d_Domain D2,
                        double TolConf, double Tol)
    {
        Perform(new IntCurve_IConicTool(Prb), D1, PCurve, D2, TolConf, Tol);
    }

    /// <summary>Intersection between the main branch of a hyperbola and a parametric curve.</summary>
    public void Perform(in Hyperbola2D H, in IntRes2d_Domain D1, in ThePCurve PCurve, in IntRes2d_Domain D2,
                        double TolConf, double Tol)
    {
        Perform(new IntCurve_IConicTool(H), D1, PCurve, D2, TolConf, Tol);
    }

    private void Perform(in IntCurve_IConicTool ICurve, in IntRes2d_Domain D1, in ThePCurve PCurve,
                         in IntRes2d_Domain D2, double TolConf, double Tol)
    {
        IntImpParGen_Intersector<IntCurve_IConicTool, ThePCurve, ThePCurveTool, TheProjPCur, TheMyImpParTool>
            myintersection = new();
        myintersection.SetReversedParameters(ReversedParameters());
        myintersection.Perform(ICurve, D1, PCurve, D2, TolConf, Tol);
        this.SetValues(myintersection);
    }
}
