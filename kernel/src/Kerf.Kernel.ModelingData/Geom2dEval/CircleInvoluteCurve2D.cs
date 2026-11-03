// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_CircleInvoluteCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_CircleInvoluteCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// The involute of a circle (gear tooth profiles):
/// <c>C(t) = O + R*(cos(t) + t*sin(t))*XDir + R*(sin(t) - t*cos(t))*YDir</c>, where O and XDir come from the
/// axis, YDir is XDir turned by +90 degrees and R &gt; 0 is the base circle radius. The parameter range is
/// [0, +inf). The curve starts on the base circle with a cusp (D1(0) = 0), and |D1(t)| = R*t.
/// Reversal and transformation are not supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
internal sealed class CircleInvoluteCurve2D : Curve2D
{
    private Axis2D _position; // Local coordinate system
    private double _radius;    // Base circle radius (> 0)

    private static void TrigShiftByQuarterTurns(double sin, double cos, int n,
                                                out double sinShift, out double cosShift)
    {
        switch (n & 3)
        {
            case 0:
                sinShift = sin;
                cosShift = cos;
                break;
            case 1:
                sinShift = cos;
                cosShift = -sin;
                break;
            case 2:
                sinShift = -sin;
                cosShift = -cos;
                break;
            default:
                sinShift = -cos;
                cosShift = sin;
                break;
        }
    }

    /// <summary>Creates the involute of the circle of radius theRadius centred at the axis location.</summary>
    /// <exception cref="ConstructionErrorException">theRadius &lt;= 0.</exception>
    public CircleInvoluteCurve2D(in Axis2D position, double radius)
    {
        _position = position;
        _radius = radius;
        if (radius <= 0.0)
        {
            throw new ConstructionErrorException("Geom2dEval_CircleInvoluteCurve: radius must be > 0");
        }
    }

    /// <summary>The local coordinate system.</summary>
    public Axis2D Position() => _position;

    /// <summary>The base circle radius.</summary>
    public double Radius() => _radius;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("Geom2dEval_CircleInvoluteCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double ReversedParameter(double u)
    {
        throw new MethodNotImplementedException("Geom2dEval_CircleInvoluteCurve::ReversedParameter");
    }

    /// <summary>0.</summary>
    public override double FirstParameter => 0.0;

    /// <summary>Precision::Infinite().</summary>
    public override double LastParameter => global::Kerf.Engine.Foundation.Precision.Infinite();

    public override bool IsClosed => false;

    public override bool IsPeriodic => false;

    public override Continuity Continuity => CN;

    internal override bool IsCN(int n) => true;

    internal override Point2D EvalD0(double u)
    {
        Xy o = _position.Location.XY();
        Xy xd = _position.Direction.XY();
        Xy yd = new(-xd.Y(), xd.X());
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        // C(t) = O + R*(cos(t) + t*sin(t))*XDir + R*(sin(t) - t*cos(t))*YDir
        return new Point2D(o + _radius * (cosU + u * sinU) * xd
                            + _radius * (sinU - u * cosU) * yd);
    }

    internal override ResD1 EvalD1(double u)
    {
        Xy o = _position.Location.XY();
        Xy xd = _position.Direction.XY();
        Xy yd = new(-xd.Y(), xd.X());
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        ResD1 result;
        result.Point =
            new Point2D(o + _radius * (cosU + u * sinU) * xd + _radius * (sinU - u * cosU) * yd);
        // D1 = R*t*cos(t)*XDir + R*t*sin(t)*YDir
        result.D1 = new Vector2D(_radius * u * cosU * xd + _radius * u * sinU * yd);
        return result;
    }

    internal override ResD2 EvalD2(double u)
    {
        Xy o = _position.Location.XY();
        Xy xd = _position.Direction.XY();
        Xy yd = new(-xd.Y(), xd.X());
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        ResD2 result;
        result.Point =
            new Point2D(o + _radius * (cosU + u * sinU) * xd + _radius * (sinU - u * cosU) * yd);
        result.D1 = new Vector2D(_radius * u * cosU * xd + _radius * u * sinU * yd);
        // D2 = R*(cos(t) - t*sin(t))*XDir + R*(sin(t) + t*cos(t))*YDir
        result.D2 =
            new Vector2D(_radius * (cosU - u * sinU) * xd + _radius * (sinU + u * cosU) * yd);
        return result;
    }

    internal override ResD3 EvalD3(double u)
    {
        Xy o = _position.Location.XY();
        Xy xd = _position.Direction.XY();
        Xy yd = new(-xd.Y(), xd.X());
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        ResD3 result;
        result.Point =
            new Point2D(o + _radius * (cosU + u * sinU) * xd + _radius * (sinU - u * cosU) * yd);
        result.D1 = new Vector2D(_radius * u * cosU * xd + _radius * u * sinU * yd);
        result.D2 =
            new Vector2D(_radius * (cosU - u * sinU) * xd + _radius * (sinU + u * cosU) * yd);
        // D3 = R*(-2*sin(t) - t*cos(t))*XDir + R*(2*cos(t) - t*sin(t))*YDir
        result.D3 = new Vector2D(_radius * (-2.0 * sinU - u * cosU) * xd
                                  + _radius * (2.0 * cosU - u * sinU) * yd);
        return result;
    }

    /// <exception cref="RangeErrorException">N &lt; 1.</exception>
    internal override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new RangeErrorException("Geom2dEval_CircleInvoluteCurve::EvalDN: N must be >= 1");
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

        // General: C(t) = R*(cos(t) + t*sin(t))*XD + R*(sin(t) - t*cos(t))*YD
        // By Leibniz rule on the t*trig(t) terms:
        // f^(N)(t) = cos(t+N*Pi/2) + t*sin(t+N*Pi/2) + N*sin(t+(N-1)*Pi/2)
        // g^(N)(t) = sin(t+N*Pi/2) - t*cos(t+N*Pi/2) - N*cos(t+(N-1)*Pi/2)
        Xy xd = _position.Direction.XY();
        Xy yd = new(-xd.Y(), xd.X());

        double sinU = Math.Sin(u);
        double cosU = Math.Cos(u);
        TrigShiftByQuarterTurns(sinU, cosU, n, out double sinN, out double cosN);
        TrigShiftByQuarterTurns(sinU, cosU, n - 1, out double sinNm1, out double cosNm1);

        double fn = cosN + u * sinN + n * sinNm1;
        double gn = sinN - u * cosN - n * cosNm1;

        return new Vector2D(_radius * fn * xd + _radius * gn * yd);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform2D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("Geom2dEval_CircleInvoluteCurve::Transform");
    }

    public override Geometry2D Copy() => new CircleInvoluteCurve2D(_position, _radius);
}
