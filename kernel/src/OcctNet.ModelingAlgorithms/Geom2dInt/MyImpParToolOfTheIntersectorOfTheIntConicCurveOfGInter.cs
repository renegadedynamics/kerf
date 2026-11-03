// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dInt/Geom2dInt_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Curves;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>
/// The signed distance between an implicit conic and the point at a parameter on a 2D curve, with its derivative.
/// </summary>
/// <remarks>Keeps a reference to the curve, as OCCT keeps a pointer.</remarks>
public sealed class MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter
    : FunctionWithDerivative,
      IMyImpParTool<MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter,
                                 IConicTool, Curve2DAdaptorBase>
{
    private readonly Curve2DAdaptorBase _theParCurve;
    private readonly IConicTool _theImpTool;

    public MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter(in IConicTool iTool,
                                                                            Curve2DAdaptorBase pc)
    {
        _theImpTool = iTool;
        _theParCurve = pc;
    }

    /// <summary>OCCT's constructor, for the generic intersector.</summary>
    public static MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter Create(
        in IConicTool it, in Curve2DAdaptorBase pc) => new(it, pc);

    /// <summary>The signed distance between the implicit curve and the point at <paramref name="param"/>.</summary>
    public override bool Value(double param, out double approxDistance)
    {
        approxDistance = _theImpTool.Distance(Geom2dCurveTool.Value(_theParCurve, param));
        return (true);
    }

    public override bool Derivative(double param, out double D_ApproxDistance_DV)
    {
        Vector2D grad = _theImpTool.GradDistance(Geom2dCurveTool.Value(_theParCurve, param));
        Geom2dCurveTool.D1(_theParCurve, param, out _, out Vector2D tanParCurve);
        D_ApproxDistance_DV = grad.Dot(tanParCurve);
        return (true);
    }

    public override bool Values(double param, out double approxDistance, out double deriv)
    {
        this.Value(param, out approxDistance);
        this.Derivative(param, out deriv);
        return (true);
    }
}
