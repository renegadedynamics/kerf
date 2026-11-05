// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_CircularHelixCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_CircularHelixCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A circular helix of radius R &gt; 0 and pitch P (the axial advance per 2*Pi turn; negative for a left-handed
/// helix) about the Z axis of a local coordinate system:
/// <c>C(t) = O + R*cos(t)*XDir + R*sin(t)*YDir + (P*t/(2*Pi))*ZDir</c>. The parameter range is (-inf, +inf); the
/// curve is neither periodic nor closed, and is CN. Reversal and transformation are not supported (they throw
/// <see cref="MethodNotImplementedException"/>).
/// </summary>
internal sealed class CircularHelixCurve : Curve3D
{
    private Frame3D _position; // Local coordinate system
    private double _radius;   // Helix radius (> 0)
    private double _pitch;    // Axial advance per 2*Pi turn

    /// <summary>Creates a circular helix with the given coordinate system, radius and pitch.</summary>
    /// <exception cref="ConstructionErrorException">theRadius &lt;= 0.</exception>
    public CircularHelixCurve(in Frame3D position, double radius, double pitch)
    {
        _position = position;
        _radius = radius;
        _pitch = pitch;
        if (radius <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_CircularHelixCurve: radius must be > 0");
        }
    }

    /// <summary>The local coordinate system.</summary>
    public Frame3D Position() => _position;

    /// <summary>The helix radius.</summary>
    public double Radius() => _radius;

    /// <summary>The pitch (axial advance per 2*Pi turn).</summary>
    public double Pitch() => _pitch;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_CircularHelixCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double ReversedParameter(double u)
    {
        throw new MethodNotImplementedException("GeomEval_CircularHelixCurve::ReversedParameter");
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
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        Xyz o = _position.Location.XYZ();
        Xyz xd = _position.XDirection.XYZ();
        Xyz yd = _position.YDirection.XYZ();
        Xyz zd = _position.Direction.XYZ();
        double zComp = _pitch * u / (2.0 * M_PI);

        return new Point3D(o + _radius * cosU * xd + _radius * sinU * yd + zComp * zd);
    }

    internal override ResD1 EvalD1(double u)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        Xyz o = _position.Location.XYZ();
        Xyz xd = _position.XDirection.XYZ();
        Xyz yd = _position.YDirection.XYZ();
        Xyz zd = _position.Direction.XYZ();
        double zComp = _pitch * u / (2.0 * M_PI);
        double zRate = _pitch / (2.0 * M_PI);

        ResD1 result;
        result.Point = new Point3D(o + _radius * cosU * xd + _radius * sinU * yd + zComp * zd);

        // D1 = -R*sin(t)*XDir + R*cos(t)*YDir + P/(2*Pi)*ZDir
        result.D1 = new Vector3D(_radius * (-sinU) * xd + _radius * cosU * yd + zRate * zd);

        return result;
    }

    internal override ResD2 EvalD2(double u)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        Xyz o = _position.Location.XYZ();
        Xyz xd = _position.XDirection.XYZ();
        Xyz yd = _position.YDirection.XYZ();
        Xyz zd = _position.Direction.XYZ();
        double zComp = _pitch * u / (2.0 * M_PI);
        double zRate = _pitch / (2.0 * M_PI);

        ResD2 result;
        result.Point = new Point3D(o + _radius * cosU * xd + _radius * sinU * yd + zComp * zd);
        result.D1 = new Vector3D(_radius * (-sinU) * xd + _radius * cosU * yd + zRate * zd);

        // D2 = -R*cos(t)*XDir - R*sin(t)*YDir
        result.D2 = new Vector3D(_radius * (-cosU) * xd + _radius * (-sinU) * yd);

        return result;
    }

    internal override ResD3 EvalD3(double u)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        Xyz o = _position.Location.XYZ();
        Xyz xd = _position.XDirection.XYZ();
        Xyz yd = _position.YDirection.XYZ();
        Xyz zd = _position.Direction.XYZ();
        double zComp = _pitch * u / (2.0 * M_PI);
        double zRate = _pitch / (2.0 * M_PI);

        ResD3 result;
        result.Point = new Point3D(o + _radius * cosU * xd + _radius * sinU * yd + zComp * zd);
        result.D1 = new Vector3D(_radius * (-sinU) * xd + _radius * cosU * yd + zRate * zd);
        result.D2 = new Vector3D(_radius * (-cosU) * xd + _radius * (-sinU) * yd);

        // D3 = R*sin(t)*XDir - R*cos(t)*YDir
        result.D3 = new Vector3D(_radius * sinU * xd + _radius * (-cosU) * yd);

        return result;
    }

    /// <exception cref="RangeErrorException">N &lt; 1.</exception>
    internal override Vector3D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new RangeErrorException("GeomEval_CircularHelixCurve::EvalDN: N must be >= 1");
        }

        Xyz xd = _position.XDirection.XYZ();
        Xyz yd = _position.YDirection.XYZ();
        Xyz zd = _position.Direction.XYZ();

        // DN cycles with period 4 in the XY plane:
        // d^N/dt^N[cos(t)] = cos(t + N*Pi/2)
        // d^N/dt^N[sin(t)] = sin(t + N*Pi/2)
        // Z component is P/(2*Pi) for N=1, 0 otherwise.
        double phase = n * M_PI / 2.0;
        double coeffX = _radius * Math.Cos(u + phase);
        double coeffY = _radius * Math.Sin(u + phase);

        Xyz result = coeffX * xd + coeffY * yd;
        if (n == 1)
        {
            result += (_pitch / (2.0 * M_PI)) * zd;
        }

        return new Vector3D(result);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_CircularHelixCurve::Transform");
    }

    public override Geometry3D Copy() => new CircularHelixCurve(_position, _radius, _pitch);
}
