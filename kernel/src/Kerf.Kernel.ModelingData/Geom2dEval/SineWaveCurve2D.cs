// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_SineWaveCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_SineWaveCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A 2D sine wave: <c>C(t) = O + t*XDir + A*sin(omega*t + phi)*YDir</c>, where O and XDir come from the
/// axis, YDir is XDir turned by +90 degrees, A &gt; 0 is the amplitude, omega &gt; 0 the angular frequency and
/// phi the phase. The parameter range is (-inf, +inf); the curve is not periodic.
/// Reversal and transformation are not supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
internal sealed class SineWaveCurve2D : Curve2D
{
    private Axis2D _position;  // Local coordinate system
    private double _amplitude;  // Wave amplitude (> 0)
    private double _omega;      // Angular frequency (> 0)
    private double _phase;      // Phase shift

    private static double PowInt(double theBase, int exp)
    {
        double res = 1.0;
        for (int i = 0; i < exp; ++i)
        {
            res *= theBase;
        }
        return res;
    }

    private static double SinShiftByQuarterTurns(double sin, double cos, int n)
    {
        switch (n & 3)
        {
            case 0:
                return sin;
            case 1:
                return cos;
            case 2:
                return -sin;
            default:
                return -cos;
        }
    }

    /// <summary>Creates a 2D sine wave curve.</summary>
    /// <exception cref="ConstructionErrorException">theAmplitude &lt;= 0 or theOmega &lt;= 0.</exception>
    public SineWaveCurve2D(in Axis2D position, double amplitude, double omega, double phase = 0.0)
    {
        _position = position;
        _amplitude = amplitude;
        _omega = omega;
        _phase = phase;
        if (amplitude <= 0.0)
        {
            throw new ConstructionErrorException("Geom2dEval_SineWaveCurve: amplitude must be > 0");
        }
        if (omega <= 0.0)
        {
            throw new ConstructionErrorException("Geom2dEval_SineWaveCurve: omega must be > 0");
        }
    }

    /// <summary>The local coordinate system.</summary>
    public Axis2D Position() => _position;

    /// <summary>The amplitude.</summary>
    public double Amplitude() => _amplitude;

    /// <summary>The angular frequency.</summary>
    public double Omega() => _omega;

    /// <summary>The phase shift.</summary>
    public double Phase() => _phase;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("Geom2dEval_SineWaveCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double ReversedParameter(double u)
    {
        throw new MethodNotImplementedException("Geom2dEval_SineWaveCurve::ReversedParameter");
    }

    /// <summary>-Precision::Infinite().</summary>
    public override double FirstParameter => -global::Kerf.Engine.Foundation.Precision.Infinite();

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

        double arg = _omega * u + _phase;

        return new Point2D(o + u * xd + _amplitude * Math.Sin(arg) * yd);
    }

    internal override ResD1 EvalD1(double u)
    {
        Xy o = _position.Location.XY();
        Xy xd = _position.Direction.XY();
        Xy yd = new(-xd.Y(), xd.X());

        double arg = _omega * u + _phase;

        ResD1 result;
        result.Point = new Point2D(o + u * xd + _amplitude * Math.Sin(arg) * yd);
        result.D1 = new Vector2D(xd + _amplitude * _omega * Math.Cos(arg) * yd);
        return result;
    }

    internal override ResD2 EvalD2(double u)
    {
        Xy o = _position.Location.XY();
        Xy xd = _position.Direction.XY();
        Xy yd = new(-xd.Y(), xd.X());

        double arg = _omega * u + _phase;
        double om2 = _omega * _omega;

        ResD2 result;
        result.Point = new Point2D(o + u * xd + _amplitude * Math.Sin(arg) * yd);
        result.D1 = new Vector2D(xd + _amplitude * _omega * Math.Cos(arg) * yd);
        result.D2 = new Vector2D(_amplitude * (-om2) * Math.Sin(arg) * yd);
        return result;
    }

    internal override ResD3 EvalD3(double u)
    {
        Xy o = _position.Location.XY();
        Xy xd = _position.Direction.XY();
        Xy yd = new(-xd.Y(), xd.X());

        double arg = _omega * u + _phase;
        double om2 = _omega * _omega;
        double om3 = om2 * _omega;

        ResD3 result;
        result.Point = new Point2D(o + u * xd + _amplitude * Math.Sin(arg) * yd);
        result.D1 = new Vector2D(xd + _amplitude * _omega * Math.Cos(arg) * yd);
        result.D2 = new Vector2D(_amplitude * (-om2) * Math.Sin(arg) * yd);
        result.D3 = new Vector2D(_amplitude * (-om3) * Math.Cos(arg) * yd);
        return result;
    }

    /// <exception cref="RangeErrorException">N &lt; 1.</exception>
    internal override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new RangeErrorException("Geom2dEval_SineWaveCurve::EvalDN: N must be >= 1");
        }

        Xy xd = _position.Direction.XY();
        Xy yd = new(-xd.Y(), xd.X());

        double arg = _omega * u + _phase;
        double sin = Math.Sin(arg);
        double cos = Math.Cos(arg);
        double omN = PowInt(_omega, n);
        double sinN = SinShiftByQuarterTurns(sin, cos, n);

        Xy result = _amplitude * omN * sinN * yd;
        if (n == 1)
        {
            result += xd;
        }

        return new Vector2D(result);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform2D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("Geom2dEval_SineWaveCurve::Transform");
    }

    public override Geometry2D Copy() => new SineWaveCurve2D(_position, _amplitude, _omega, _phase);
}
