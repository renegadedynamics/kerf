// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_FuncCurNul2d.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Properties;

/// <summary>
/// The function whose roots are the inflection points of a 2D curve: F = (V1 ^ V2) / (|V1| |V2|), V1 and V2 the first
/// and second derivatives, with its analytic derivative.
/// </summary>
internal sealed class FuncCurNul2d : FunctionWithDerivative
{
    private readonly Curve2D _curve; // The analyzed curve.

    internal FuncCurNul2d(Curve2D curve)
    {
        _curve = curve;
    }

    public override bool Value(double u, out double f)
    {
        return Values(u, out f, out _);
    }

    public override bool Derivative(double u, out double d)
    {
        return Values(u, out _, out d);
    }

    /// <summary>
    /// F and F' = ((V1 ^ V3) - F (V1.V2 / |V1|^2) - F (V2.V3 / |V2|^2)) / (|V1| |V2|); false if the derivatives are
    /// degenerate. F = F' = 0 where |V2| &lt; 1e-4.
    /// </summary>
    public override bool Values(double u, out double f, out double d)
    {
        _curve.D3(u, out Point2D _, out Vector2D deriv1, out Vector2D deriv2, out Vector2D deriv3);
        double cross12 = deriv1.Crossed(deriv2);
        double cross13 = deriv1.Crossed(deriv3);
        double dot12 = deriv1.Dot(deriv2);
        double dot23 = deriv2.Dot(deriv3);
        double norm1 = deriv1.Magnitude;
        double norm2 = deriv2.Magnitude;

        f = 0.0;
        d = 0.0;

        if (norm2 < 1.0e-4)
        {
            return true;
        }
        else if (norm1 * norm2 < Constants.Resolution())
        {
            return false;
        }
        else
        {
            double normProd = norm1 * norm2;
            f = cross12 / normProd;
            d = (cross13 - cross12 * dot12 / (norm1 * norm1) - cross12 * dot23 / (norm2 * norm2))
                   / normProd;
        }

        return true;
    }
}
