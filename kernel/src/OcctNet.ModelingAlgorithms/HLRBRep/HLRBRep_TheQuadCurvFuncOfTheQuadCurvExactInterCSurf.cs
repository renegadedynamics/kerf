// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheQuadCurvFuncOfTheQuadCurvExactInterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_TheQuadCurvFuncOfTheQuadCurvExactInterCSurf_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The implicit function of a quadric along a curve adaptor, F(w) = Q(C(w)), with its derivative, whose roots are
/// the intersections of the curve and the quadric.
/// </summary>
public sealed class HLRBRep_TheQuadCurvFuncOfTheQuadCurvExactInterCSurf
    : math_FunctionWithDerivative,
      IIntCurveSurface_TheQuadCurvFunc<HLRBRep_TheQuadCurvFuncOfTheQuadCurvExactInterCSurf, gp_Lin>
{
    private readonly IntSurf_Quadric myQuadric;
    private readonly gp_Lin myCurve;

    /// <summary>The function of the quadric <paramref name="Q"/> along the curve <paramref name="C"/> (kept by reference).</summary>
    public HLRBRep_TheQuadCurvFuncOfTheQuadCurvExactInterCSurf(in IntSurf_Quadric Q, gp_Lin C)
    {
        myQuadric = Q;
        myCurve = C;
    }

    static HLRBRep_TheQuadCurvFuncOfTheQuadCurvExactInterCSurf IIntCurveSurface_TheQuadCurvFunc<HLRBRep_TheQuadCurvFuncOfTheQuadCurvExactInterCSurf, gp_Lin>.Create(
        in IntSurf_Quadric Q, in gp_Lin C) => new(Q, C);

    public override bool Value(double Param, out double F)
    {
        F = myQuadric.Distance(HLRBRep_LineTool.Value(myCurve, Param));
        return (true);
    }

    public override bool Derivative(double Param, out double D)
    {
        HLRBRep_LineTool.D1(myCurve, Param, out gp_Pnt P, out gp_Vec T);
        D = T.Dot(myQuadric.Gradient(P));
        return (true);
    }

    public override bool Values(double Param, out double F, out double D)
    {
        HLRBRep_LineTool.D1(myCurve, Param, out gp_Pnt P, out gp_Vec T);
        F = 0.0;
        gp_Vec Grad = new();
        myQuadric.ValAndGrad(P, ref F, ref Grad);
        D = T.Dot(Grad);
        return (true);
    }
}
