// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The signed distance between an implicit conic and the point at a parameter on a 2D curve, with its derivative.
/// </summary>
/// <remarks>Keeps a reference to the curve, as OCCT keeps a pointer.</remarks>
public sealed class Geom2dInt_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter
    : FunctionWithDerivative,
      IIntImpParGen_MyImpParTool<Geom2dInt_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter,
                                 IntCurve_IConicTool, Curve2DAdaptorBase>
{
    private readonly Curve2DAdaptorBase TheParCurve;
    private readonly IntCurve_IConicTool TheImpTool;

    public Geom2dInt_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter(in IntCurve_IConicTool ITool,
                                                                            Curve2DAdaptorBase PC)
    {
        TheImpTool = ITool;
        TheParCurve = PC;
    }

    /// <summary>OCCT's constructor, for the generic intersector.</summary>
    public static Geom2dInt_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter Create(
        in IntCurve_IConicTool IT, in Curve2DAdaptorBase PC) => new(IT, PC);

    /// <summary>The signed distance between the implicit curve and the point at <paramref name="Param"/>.</summary>
    public override bool Value(double Param, out double ApproxDistance)
    {
        ApproxDistance = TheImpTool.Distance(Geom2dInt_Geom2dCurveTool.Value(TheParCurve, Param));
        return (true);
    }

    public override bool Derivative(double Param, out double D_ApproxDistance_DV)
    {
        Vector2D Grad = TheImpTool.GradDistance(Geom2dInt_Geom2dCurveTool.Value(TheParCurve, Param));
        Geom2dInt_Geom2dCurveTool.D1(TheParCurve, Param, out _, out Vector2D TanParCurve);
        D_ApproxDistance_DV = Grad.Dot(TanParCurve);
        return (true);
    }

    public override bool Values(double Param, out double ApproxDistance, out double Deriv)
    {
        this.Value(Param, out ApproxDistance);
        this.Derivative(Param, out Deriv);
        return (true);
    }
}
