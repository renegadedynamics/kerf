// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_InterFunc.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_InterFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function U(t) - FixVal (Fix = 1) or V(t) - FixVal (Fix = 2) of a 2D curve, whose roots locate the
/// parameters where an Adaptor3d_CurveOnSurface crosses a discontinuity of its surface.
/// </summary>
public class Adaptor3d_InterFunc : math_FunctionWithDerivative
{
    private readonly Adaptor2d_Curve2d myCurve2d;
    private readonly double myFixVal;
    private readonly int myFix;

    /// <exception cref="Standard_ConstructionError">Fix is neither 1 nor 2.</exception>
    public Adaptor3d_InterFunc(Adaptor2d_Curve2d C, double FixVal, int Fix)
    {
        myCurve2d = C;
        myFixVal = FixVal;
        myFix = Fix;
        if (Fix != 1 && Fix != 2)
        {
            throw new Standard_ConstructionError();
        }
    }

    public override bool Value(double X, out double F)
    {
        gp_Pnt2d C;
        myCurve2d.D0(X, out C);
        if (myFix == 1)
        {
            F = C.X() - myFixVal;
        }
        else
        {
            F = C.Y() - myFixVal;
        }
        return true;
    }

    public override bool Derivative(double X, out double D)
    {
        double F;
        return Values(X, out F, out D);
    }

    public override bool Values(double X, out double F, out double D)
    {
        gp_Pnt2d C;
        gp_Vec2d DC;
        myCurve2d.D1(X, out C, out DC);
        if (myFix == 1)
        {
            F = C.X() - myFixVal;
            D = DC.X();
        }
        else
        {
            F = C.Y() - myFixVal;
            D = DC.Y();
        }
        return true;
    }
}
