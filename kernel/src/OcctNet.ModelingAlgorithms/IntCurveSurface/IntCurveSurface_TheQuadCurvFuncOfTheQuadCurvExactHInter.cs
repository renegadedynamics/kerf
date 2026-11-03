// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheQuadCurvFuncOfTheQuadCurvExactHInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_TheQuadCurvFuncOfTheQuadCurvExactHInter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The implicit function of a quadric along a curve adaptor, F(w) = Q(C(w)), with its derivative, whose roots are
/// the intersections of the curve and the quadric.
/// </summary>
public sealed class IntCurveSurface_TheQuadCurvFuncOfTheQuadCurvExactHInter
    : FunctionWithDerivative,
      IIntCurveSurface_TheQuadCurvFunc<IntCurveSurface_TheQuadCurvFuncOfTheQuadCurvExactHInter, CurveAdaptorBase>
{
    private readonly IntSurf_Quadric myQuadric;
    private readonly CurveAdaptorBase myCurve;

    /// <summary>The function of the quadric <paramref name="Q"/> along the curve <paramref name="C"/> (kept by reference).</summary>
    public IntCurveSurface_TheQuadCurvFuncOfTheQuadCurvExactHInter(in IntSurf_Quadric Q, CurveAdaptorBase C)
    {
        myQuadric = Q;
        myCurve = C;
    }

    static IntCurveSurface_TheQuadCurvFuncOfTheQuadCurvExactHInter IIntCurveSurface_TheQuadCurvFunc<IntCurveSurface_TheQuadCurvFuncOfTheQuadCurvExactHInter, CurveAdaptorBase>.Create(
        in IntSurf_Quadric Q, in CurveAdaptorBase C) => new(Q, C);

    public override bool Value(double Param, out double F)
    {
        F = myQuadric.Distance(IntCurveSurface_TheHCurveTool.Value(myCurve, Param));
        return (true);
    }

    public override bool Derivative(double Param, out double D)
    {
        IntCurveSurface_TheHCurveTool.D1(myCurve, Param, out Point3D P, out Vector3D T);
        D = T.Dot(myQuadric.Gradient(P));
        return (true);
    }

    public override bool Values(double Param, out double F, out double D)
    {
        IntCurveSurface_TheHCurveTool.D1(myCurve, Param, out Point3D P, out Vector3D T);
        F = 0.0;
        Vector3D Grad = new();
        myQuadric.ValAndGrad(P, ref F, ref Grad);
        D = T.Dot(Grad);
        return (true);
    }
}
