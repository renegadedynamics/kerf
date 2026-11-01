// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_SearchSing.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_SearchSing.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function whose roots are the parameters where the isos of two curves meet: F(t) = (C2(t) - C1(t)) · (C2'(t) -
/// C1'(t)), with its derivative. ChFi3d_FilBuilder's <c>SplitSurf</c> searches the singular sections of a fillet with it.
/// </summary>
internal sealed class ChFi3d_SearchSing : math_FunctionWithDerivative
{
    private readonly Geom_Curve myC1;
    private readonly Geom_Curve myC2;

    public ChFi3d_SearchSing(Geom_Curve C1, Geom_Curve C2)
    {
        myC1 = C1;
        myC2 = C2;
    }

    public override bool Value(double X, out double F)
    {
        myC1.D1(X, out gp_Pnt P1, out gp_Vec V1);
        myC2.D1(X, out gp_Pnt P2, out gp_Vec V2);
        gp_Vec V = new(P1, P2);
        F = V * (V2 - V1);
        return true;
    }

    public override bool Derivative(double X, out double D)
    {
        myC1.D2(X, out gp_Pnt P1, out gp_Vec V1, out gp_Vec W1);
        myC2.D2(X, out gp_Pnt P2, out gp_Vec V2, out gp_Vec W2);
        gp_Vec V = new(P1, P2), VPrim;
        VPrim = V2 - V1;
        D = VPrim.SquareMagnitude() + (V * (W2 - W1));
        return true;
    }

    public override bool Values(double X, out double F, out double D)
    {
        myC1.D2(X, out gp_Pnt P1, out gp_Vec V1, out gp_Vec W1);
        myC2.D2(X, out gp_Pnt P2, out gp_Vec V2, out gp_Vec W2);
        gp_Vec V = new(P1, P2), VPrim;
        VPrim = V2 - V1;
        F = V * VPrim;
        D = VPrim.SquareMagnitude() + (V * (W2 - W1));
        return true;
    }
}
