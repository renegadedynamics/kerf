// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_FuncCurNul2d.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function whose roots are the inflection points of a 2D curve: F = (V1 ^ V2) / (|V1| |V2|), V1 and V2 the first
/// and second derivatives, with its analytic derivative.
/// </summary>
internal sealed class GeomLProp_FuncCurNul2d : math_FunctionWithDerivative
{
    private readonly Geom2d_Curve myCurve; // The analyzed curve.

    internal GeomLProp_FuncCurNul2d(Geom2d_Curve theCurve)
    {
        myCurve = theCurve;
    }

    public override bool Value(double theU, out double theF)
    {
        return Values(theU, out theF, out _);
    }

    public override bool Derivative(double theU, out double theD)
    {
        return Values(theU, out _, out theD);
    }

    /// <summary>
    /// F and F' = ((V1 ^ V3) - F (V1.V2 / |V1|^2) - F (V2.V3 / |V2|^2)) / (|V1| |V2|); false if the derivatives are
    /// degenerate. F = F' = 0 where |V2| &lt; 1e-4.
    /// </summary>
    public override bool Values(double theU, out double theF, out double theD)
    {
        myCurve.D3(theU, out gp_Pnt2d _, out gp_Vec2d aDeriv1, out gp_Vec2d aDeriv2, out gp_Vec2d aDeriv3);
        double aCross12 = aDeriv1.Crossed(aDeriv2);
        double aCross13 = aDeriv1.Crossed(aDeriv3);
        double aDot12 = aDeriv1.Dot(aDeriv2);
        double aDot23 = aDeriv2.Dot(aDeriv3);
        double aNorm1 = aDeriv1.Magnitude();
        double aNorm2 = aDeriv2.Magnitude();

        theF = 0.0;
        theD = 0.0;

        if (aNorm2 < 1.0e-4)
        {
            return true;
        }
        else if (aNorm1 * aNorm2 < gp.Resolution())
        {
            return false;
        }
        else
        {
            double aNormProd = aNorm1 * aNorm2;
            theF = aCross12 / aNormProd;
            theD = (aCross13 - aCross12 * aDot12 / (aNorm1 * aNorm1) - aCross12 * aDot23 / (aNorm2 * aNorm2))
                   / aNormProd;
        }

        return true;
    }
}
