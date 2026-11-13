// Port of: src/ModelingData/TKGeomBase/GeomLProp/GeomLProp_FuncCurNul2d.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Properties;

/// <summary>
/// The function whose roots are the inflection points of a 2D curve, on one smooth interval of it: the derivative of the
/// tangent's angle with respect to the parameter normalised to the interval, F = L (V1 ^ V2) / |V1|^2, L the interval's
/// length, with its analytic derivative.
/// </summary>
/// <remarks>
/// Upstream 8f817f5da5 (decision 34702). 8.0.1's F was (V1 ^ V2) / (|V1| |V2|), set to exactly 0 wherever |V2| was under
/// 1e-4 in the curve's own parameter units, so a nearly straight stretch, or a whole curve with a long parameter range,
/// read as a run of roots, and its ends, where F jumped back to the sine of the angle between V1 and V2, as inflections.
/// This F is continuous wherever the tangent is, and doesn't change with the parameter's scale.
/// </remarks>
internal sealed class FuncCurNul2d : FunctionWithDerivative
{
    private readonly CurveAdaptor2D _curve; // The analyzed curve, restricted to the smooth interval.
    private readonly double _parameterSpan; // The interval's length, which normalises the parameter.
    private readonly Curve2D? _nativeCurve; // The whole curve, for 8.0.1's function under NativeInflections' scope.

    internal FuncCurNul2d(Curve2D curve, double uMin, double uMax)
    {
        _curve = new CurveAdaptor2D(curve, uMin, uMax);
        _parameterSpan = uMax - uMin;
        _nativeCurve = NativeInflections.IsNative ? curve : null;
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
    /// F and F' = L ((T ^ V3) - 2 (T ^ V2) (T . V2) / |V1|) / |V1|, T the unit tangent; false if the tangent is degenerate.
    /// </summary>
    public override bool Values(double u, out double f, out double d)
    {
        if (_nativeCurve is not null)
        {
            return NativeValues(u, out f, out d);
        }

        _curve.D3(u, out Point2D _, out Vector2D deriv1, out Vector2D deriv2, out Vector2D deriv3);
        double speed = deriv1.Magnitude;
        if (speed <= Constants.Resolution())
        {
            f = 0.0;
            d = 0.0;
            return false;
        }

        Vector2D tangent = deriv1 / speed;
        Vector2D d2 = deriv2 / speed;
        Vector2D d3 = deriv3 / speed;
        double turn = tangent.Crossed(d2);
        f = _parameterSpan * turn;
        d = _parameterSpan * (tangent.Crossed(d3) - 2.0 * turn * tangent.Dot(d2));
        return true;
    }

    // 8.0.1's F = (V1 ^ V2) / (|V1| |V2|), 0 where |V2| < 1e-4, and its derivative, for the native recordings' tests only
    private bool NativeValues(double u, out double f, out double d)
    {
        _nativeCurve!.D3(u, out Point2D _, out Vector2D deriv1, out Vector2D deriv2, out Vector2D deriv3);
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

        if (norm1 * norm2 < Constants.Resolution())
        {
            return false;
        }

        double normProd = norm1 * norm2;
        f = cross12 / normProd;
        d = (cross13 - cross12 * dot12 / (norm1 * norm1) - cross12 * dot23 / (norm2 * norm2)) / normProd;
        return true;
    }
}
