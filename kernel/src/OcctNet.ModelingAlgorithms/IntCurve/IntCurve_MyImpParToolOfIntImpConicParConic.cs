// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_MyImpParToolOfIntImpConicParConic.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_MyImpParToolOfIntImpConicParConic_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The signed distance between an implicit conic and the point at a parameter on a parametric conic, with its
/// derivative.
/// </summary>
/// <remarks>
/// OCCT keeps a pointer to the parametric conic; the port keeps a copy of it (a struct the intersection never
/// changes).
/// </remarks>
public sealed class IntCurve_MyImpParToolOfIntImpConicParConic
    : math_FunctionWithDerivative,
      IIntImpParGen_MyImpParTool<IntCurve_MyImpParToolOfIntImpConicParConic, IntCurve_IConicTool, IntCurve_PConic>
{
    private readonly IntCurve_PConic TheParCurve;
    private readonly IntCurve_IConicTool TheImpTool;

    public IntCurve_MyImpParToolOfIntImpConicParConic(in IntCurve_IConicTool ITool, in IntCurve_PConic PC)
    {
        TheImpTool = ITool;
        TheParCurve = PC;
    }

    /// <summary>OCCT's constructor, for the generic intersector.</summary>
    public static IntCurve_MyImpParToolOfIntImpConicParConic Create(in IntCurve_IConicTool IT, in IntCurve_PConic PC)
        => new(IT, PC);

    /// <summary>The signed distance between the implicit curve and the point at <paramref name="Param"/>.</summary>
    public override bool Value(double Param, out double ApproxDistance)
    {
        ApproxDistance = TheImpTool.Distance(IntCurve_PConicTool.Value(TheParCurve, Param));
        return (true);
    }

    public override bool Derivative(double Param, out double D_ApproxDistance_DV)
    {
        gp_Vec2d Grad = TheImpTool.GradDistance(IntCurve_PConicTool.Value(TheParCurve, Param));
        IntCurve_PConicTool.D1(TheParCurve, Param, out _, out gp_Vec2d TanParCurve);
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
