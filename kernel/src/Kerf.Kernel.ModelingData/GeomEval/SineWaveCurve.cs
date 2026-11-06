// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_SineWaveCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_SineWaveCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A 3D sine wave in the XY plane of a local coordinate system, propagating along XDir and oscillating along
/// YDir: <c>C(t) = O + t*XDir + A*sin(omega*t + phi)*YDir</c>, with amplitude A &gt; 0, angular frequency
/// omega &gt; 0 and phase phi. The parameter range is (-inf, +inf); the curve is not periodic.
/// Reversal and transformation are not supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
internal sealed class SineWaveCurve : Curve3D
{
    private Frame3D _position;  // Local coordinate system
    private double _amplitude; // Wave amplitude (> 0)
    private double _omega;     // Angular frequency (> 0)
    private double _phase;     // Phase shift

    /// <summary>Creates a 3D sine wave curve.</summary>
    /// <exception cref="ConstructionErrorException">theAmplitude &lt;= 0 or theOmega &lt;= 0.</exception>
    public SineWaveCurve(in Frame3D position, double amplitude, double omega, double phase = 0.0)
    {
        _position = position;
        _amplitude = amplitude;
        _omega = omega;
        _phase = phase;
        if (amplitude <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_SineWaveCurve: amplitude must be > 0");
        }
        if (omega <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_SineWaveCurve: omega must be > 0");
        }
    }

    /// <summary>The local coordinate system.</summary>
    public Frame3D Position() => _position;

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
        throw new MethodNotImplementedException("GeomEval_SineWaveCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double ReversedParameter(double u)
    {
        throw new MethodNotImplementedException("GeomEval_SineWaveCurve::ReversedParameter");
    }

    /// <summary>-Precision::Infinite().</summary>
    public override double FirstParameter => -global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public override double LastParameter => global::Kerf.Engine.Foundation.Precision.Infinite();

    public override bool IsClosed => false;

    public override bool IsPeriodic => false;

    public override Continuity Continuity => CN;

    internal override bool IsCN(int n) => true;

    internal override Point3D EvalD0(double u)
    {
        Xyz o = _position.Location.XYZ();
        Xyz xd = _position.XDirection.XYZ();
        Xyz yd = _position.YDirection.XYZ();

        double arg = _omega * u + _phase;

        return new Point3D(o + u * xd + _amplitude * Math.Sin(arg) * yd);
    }

    internal override ResD1 EvalD1(double u)
    {
        Xyz o = _position.Location.XYZ();
        Xyz xd = _position.XDirection.XYZ();
        Xyz yd = _position.YDirection.XYZ();

        double arg = _omega * u + _phase;

        ResD1 result;
        result.Point = new Point3D(o + u * xd + _amplitude * Math.Sin(arg) * yd);

        // D1 = XDir + A*omega*cos(omega*t + phi)*YDir
        result.D1 = new Vector3D(xd + _amplitude * _omega * Math.Cos(arg) * yd);

        return result;
    }

    internal override ResD2 EvalD2(double u)
    {
        Xyz o = _position.Location.XYZ();
        Xyz xd = _position.XDirection.XYZ();
        Xyz yd = _position.YDirection.XYZ();

        double arg = _omega * u + _phase;
        double om2 = _omega * _omega;

        ResD2 result;
        result.Point = new Point3D(o + u * xd + _amplitude * Math.Sin(arg) * yd);
        result.D1 = new Vector3D(xd + _amplitude * _omega * Math.Cos(arg) * yd);

        // D2 = -A*omega^2*sin(omega*t + phi)*YDir
        result.D2 = new Vector3D(_amplitude * (-om2) * Math.Sin(arg) * yd);

        return result;
    }

    internal override ResD3 EvalD3(double u)
    {
        Xyz o = _position.Location.XYZ();
        Xyz xd = _position.XDirection.XYZ();
        Xyz yd = _position.YDirection.XYZ();

        double arg = _omega * u + _phase;
        double om2 = _omega * _omega;
        double om3 = om2 * _omega;

        ResD3 result;
        result.Point = new Point3D(o + u * xd + _amplitude * Math.Sin(arg) * yd);
        result.D1 = new Vector3D(xd + _amplitude * _omega * Math.Cos(arg) * yd);
        result.D2 = new Vector3D(_amplitude * (-om2) * Math.Sin(arg) * yd);

        // D3 = -A*omega^3*cos(omega*t + phi)*YDir
        result.D3 = new Vector3D(_amplitude * (-om3) * Math.Cos(arg) * yd);

        return result;
    }

    /// <exception cref="RangeErrorException">N &lt; 1.</exception>
    internal override Vector3D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new RangeErrorException("GeomEval_SineWaveCurve::EvalDN: N must be >= 1");
        }

        Xyz xd = _position.XDirection.XYZ();
        Xyz yd = _position.YDirection.XYZ();

        // d^N/dt^N[t] = 1 if N=1, 0 if N>=2
        // d^N/dt^N[A*sin(omega*t+phi)] = A*omega^N*sin(omega*t+phi+N*Pi/2)
        double omN = 1.0;
        for (int i = 0; i < n; ++i)
        {
            omN *= _omega;
        }
        double phase = _omega * u + _phase + n * M_PI / 2.0;

        Xyz result = _amplitude * omN * Math.Sin(phase) * yd;
        if (n == 1)
        {
            result += xd;
        }

        return new Vector3D(result);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_SineWaveCurve::Transform");
    }

    public override Geometry3D Copy() => new SineWaveCurve(_position, _amplitude, _omega, _phase);
}
