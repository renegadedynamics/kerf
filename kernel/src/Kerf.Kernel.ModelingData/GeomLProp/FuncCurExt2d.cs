// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_FuncCurExt2d.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Properties;

/// <summary>
/// The function whose roots are the extrema of curvature of a 2D curve: F = dKC/dU, KC = (V1 ^ V2) / |V1|^3, with its
/// derivative by finite differences.
/// </summary>
internal sealed class FuncCurExt2d : FunctionWithDerivative
{
    private readonly Curve2D _curve; // The analyzed curve.
    private readonly double _epsX;        // Parametric tolerance for finite differences.

    internal FuncCurExt2d(Curve2D curve, double tol)
    {
        _curve = curve;
        _epsX = tol;
    }

    /// <summary>dKC/dU at the parameter; false if the first derivative is degenerate.</summary>
    public override bool Value(double u, out double f)
    {
        f = 0.0; // OCCT leaves it unset when it returns false

        _curve.D3(u, out Point2D _, out Vector2D deriv1, out Vector2D deriv2, out Vector2D deriv3);
        double cross12 = deriv1.Crossed(deriv2);
        double cross13 = deriv1.Crossed(deriv3);
        double dot12 = deriv1.Dot(deriv2);
        double sqNorm1 = deriv1.SquareMagnitude;
        double norm1 = Math.Sqrt(sqNorm1);
        double norm1Cb = sqNorm1 * norm1;     // ||V1||^3
        double norm1Pow5 = norm1Cb * sqNorm1; // ||V1||^5

        if (norm1Pow5 < Constants.Resolution())
        {
            return false;
        }

        f = cross13 / norm1Cb - 3.0 * cross12 * dot12 / norm1Pow5;
        return true;
    }

    public override bool Derivative(double u, out double d)
    {
        return Values(u, out _, out d);
    }

    /// <summary>F and its derivative by a forward (or, at the end, backward) difference of myEpsX / 100.</summary>
    public override bool Values(double u, out double f, out double d)
    {
        d = 0.0; // OCCT leaves it unset when it returns false
        double fNext;
        double deltaU = _epsX / 100.0;

        if (u + deltaU > _curve.LastParameter)
        {
            deltaU = -deltaU;
        }

        if (!Value(u, out f) || !Value(u + deltaU, out fNext))
        {
            return false;
        }

        d = (fNext - f) / deltaU;
        return true;
    }

    /// <summary>
    /// True if the parameter is a minimum of the radius of curvature: |KC(theU)| &gt; |KC(theU + eps)|.
    /// </summary>
    internal bool IsMinKC(double u)
    {
        double deltaU = _epsX;

        // Compute curvature at theU.
        _curve.D3(u, out Point2D pnt, out Vector2D deriv1, out Vector2D deriv2, out Vector2D deriv3);
        double cross12 = deriv1.Crossed(deriv2);
        double sqNorm1 = deriv1.SquareMagnitude;
        double norm1 = Math.Sqrt(sqNorm1);
        double norm1Cb = sqNorm1 * norm1;

        if (norm1Cb < Constants.Resolution())
        {
            return false;
        }

        double curvCurr = cross12 / norm1Cb;

        if (u + deltaU > _curve.LastParameter)
        {
            deltaU = -deltaU;
        }

        // Compute curvature at the probe point.
        _curve.D3(u + deltaU, out pnt, out deriv1, out deriv2, out deriv3);
        cross12 = deriv1.Crossed(deriv2);
        sqNorm1 = deriv1.SquareMagnitude;
        norm1 = Math.Sqrt(sqNorm1);
        norm1Cb = sqNorm1 * norm1;

        if (norm1Cb < Constants.Resolution())
        {
            return false;
        }

        double curvProbe = cross12 / norm1Cb;

        return Math.Abs(curvCurr) > Math.Abs(curvProbe);
    }
}
