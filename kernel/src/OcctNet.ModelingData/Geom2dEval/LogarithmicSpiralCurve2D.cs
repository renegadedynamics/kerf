// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_LogarithmicSpiralCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_LogarithmicSpiralCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A 2D logarithmic (equiangular) spiral, polar equation r = a*exp(b*t):
/// <c>C(t) = O + a*exp(b*t)*cos(t)*XDir + a*exp(b*t)*sin(t)*YDir</c>, where O and XDir come from the axis,
/// YDir is XDir turned by +90 degrees, a &gt; 0 is the scale and b &gt; 0 the growth exponent. The parameter
/// range is (-inf, +inf). The angle between the tangent and the radial direction is constant, atan(1/b).
/// Reversal and transformation are not supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
public sealed class LogarithmicSpiralCurve2D : Curve2D
{
    private Axis2D _position;      // Local coordinate system
    private double _scale;          // Scale factor (> 0)
    private double _growthExponent; // Growth exponent (> 0)

    private static void PowComplexInt(double baseRe, double baseIm, int exp,
                                      out double resRe, out double resIm)
    {
        double aResRe = 1.0;
        double aResIm = 0.0;
        double powRe = baseRe;
        double powIm = baseIm;
        int aExp = exp;
        while (aExp > 0)
        {
            if ((aExp & 1) != 0)
            {
                double tmpRe = aResRe * powRe - aResIm * powIm;
                double tmpIm = aResRe * powIm + aResIm * powRe;
                aResRe = tmpRe;
                aResIm = tmpIm;
            }
            double nextPowRe = powRe * powRe - powIm * powIm;
            double nextPowIm = 2.0 * powRe * powIm;
            powRe = nextPowRe;
            powIm = nextPowIm;
            aExp >>= 1;
        }
        resRe = aResRe;
        resIm = aResIm;
    }

    /// <summary>Creates a logarithmic spiral.</summary>
    /// <exception cref="ConstructionErrorException">theScale &lt;= 0 or theGrowthExponent &lt;= 0.</exception>
    public LogarithmicSpiralCurve2D(in Axis2D position, double scale, double growthExponent)
    {
        _position = position;
        _scale = scale;
        _growthExponent = growthExponent;
        if (scale <= 0.0)
        {
            throw new ConstructionErrorException("Geom2dEval_LogarithmicSpiralCurve: scale must be > 0");
        }
        if (growthExponent <= 0.0)
        {
            throw new ConstructionErrorException(
                "Geom2dEval_LogarithmicSpiralCurve: growth exponent must be > 0");
        }
    }

    /// <summary>The local coordinate system.</summary>
    public Axis2D Position() => _position;

    /// <summary>The scale factor a.</summary>
    public double Scale() => _scale;

    /// <summary>The growth exponent b.</summary>
    public double GrowthExponent() => _growthExponent;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("Geom2dEval_LogarithmicSpiralCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double ReversedParameter(double u)
    {
        throw new MethodNotImplementedException("Geom2dEval_LogarithmicSpiralCurve::ReversedParameter");
    }

    /// <summary>-Precision::Infinite().</summary>
    public override double FirstParameter() => -global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public override double LastParameter() => global::Kerf.Engine.Foundation.Precision.Infinite();

    public override bool IsClosed => false;

    public override bool IsPeriodic => false;

    public override Continuity Continuity() => CN;

    public override bool IsCN(int n) => true;

    public override Point2D EvalD0(double u)
    {
        Xy o = _position.Location().XY();
        Xy xd = _position.Direction().XY();
        Xy yd = new(-xd.Y(), xd.X());
        double exp = _scale * Math.Exp(_growthExponent * u);
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        return new Point2D(o + exp * cosU * xd + exp * sinU * yd);
    }

    public override ResD1 EvalD1(double u)
    {
        Xy o = _position.Location().XY();
        Xy xd = _position.Direction().XY();
        Xy yd = new(-xd.Y(), xd.X());
        double exp = _scale * Math.Exp(_growthExponent * u);
        double b = _growthExponent;
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        ResD1 result;
        result.Point = new Point2D(o + exp * cosU * xd + exp * sinU * yd);
        // D1 = a*e^(b*t)*[(b*cos(t)-sin(t))*XDir + (b*sin(t)+cos(t))*YDir]
        result.D1 = new Vector2D(exp * (b * cosU - sinU) * xd + exp * (b * sinU + cosU) * yd);
        return result;
    }

    public override ResD2 EvalD2(double u)
    {
        Xy o = _position.Location().XY();
        Xy xd = _position.Direction().XY();
        Xy yd = new(-xd.Y(), xd.X());
        double exp = _scale * Math.Exp(_growthExponent * u);
        double b = _growthExponent;
        double b2 = b * b;
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        ResD2 result;
        result.Point = new Point2D(o + exp * cosU * xd + exp * sinU * yd);
        result.D1 = new Vector2D(exp * (b * cosU - sinU) * xd + exp * (b * sinU + cosU) * yd);
        // D2 = a*e^(b*t)*[((b^2-1)*cos(t) - 2*b*sin(t))*XDir
        //                + ((b^2-1)*sin(t) + 2*b*cos(t))*YDir]
        result.D2 = new Vector2D(exp * ((b2 - 1.0) * cosU - 2.0 * b * sinU) * xd
                                  + exp * ((b2 - 1.0) * sinU + 2.0 * b * cosU) * yd);
        return result;
    }

    public override ResD3 EvalD3(double u)
    {
        Xy o = _position.Location().XY();
        Xy xd = _position.Direction().XY();
        Xy yd = new(-xd.Y(), xd.X());
        double exp = _scale * Math.Exp(_growthExponent * u);
        double b = _growthExponent;
        double b2 = b * b;
        double b3 = b2 * b;
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        ResD3 result;
        result.Point = new Point2D(o + exp * cosU * xd + exp * sinU * yd);
        result.D1 = new Vector2D(exp * (b * cosU - sinU) * xd + exp * (b * sinU + cosU) * yd);
        result.D2 = new Vector2D(exp * ((b2 - 1.0) * cosU - 2.0 * b * sinU) * xd
                                  + exp * ((b2 - 1.0) * sinU + 2.0 * b * cosU) * yd);
        // D3 = a*e^(b*t)*[((b^3-3*b)*cos(t) - (3*b^2-1)*sin(t))*XDir
        //                + ((b^3-3*b)*sin(t) + (3*b^2-1)*cos(t))*YDir]
        result.D3 = new Vector2D(exp * ((b3 - 3.0 * b) * cosU - (3.0 * b2 - 1.0) * sinU) * xd
                                  + exp * ((b3 - 3.0 * b) * sinU + (3.0 * b2 - 1.0) * cosU) * yd);
        return result;
    }

    /// <exception cref="RangeErrorException">N &lt; 1.</exception>
    public override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new RangeErrorException("Geom2dEval_LogarithmicSpiralCurve::EvalDN: N must be >= 1");
        }

        // For low orders, delegate to explicit implementations.
        if (n == 1)
        {
            return EvalD1(u).D1;
        }
        if (n == 2)
        {
            return EvalD2(u).D2;
        }
        if (n == 3)
        {
            return EvalD3(u).D3;
        }

        Xy xd = _position.Direction().XY();
        Xy yd = new(-xd.Y(), xd.X());

        double b = _growthExponent;
        double scale = _scale * Math.Exp(b * u);
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        // d^N/dt^N [a*e^(b*t)*(cos(t) + i*sin(t))]
        // = a*e^(b*t) * (b+i)^N * (cos(t) + i*sin(t))
        PowComplexInt(b, 1.0, n, out double powRe, out double powIm);

        double coeffX = powRe * cosU - powIm * sinU;
        double coeffY = powRe * sinU + powIm * cosU;

        return new Vector2D(scale * coeffX * xd + scale * coeffY * yd);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform2D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("Geom2dEval_LogarithmicSpiralCurve::Transform");
    }

    public override Geometry2D Copy() =>
        new LogarithmicSpiralCurve2D(_position, _scale, _growthExponent);
}
