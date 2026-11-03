// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_ArchimedeanSpiralCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_ArchimedeanSpiralCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A 2D Archimedean spiral, polar equation r = a + b*t:
/// <c>C(t) = O + (a + b*t)*cos(t)*XDir + (a + b*t)*sin(t)*YDir</c>, where O and XDir come from the axis, YDir
/// is XDir turned by +90 degrees, a &gt;= 0 is the initial radius and b &gt; 0 the growth rate per radian.
/// The parameter range is [0, +inf); the curve is neither periodic nor closed.
/// Reversal and transformation are not supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
public sealed class ArchimedeanSpiralCurve2D : Curve2D
{
    private Axis2D _position;     // Local coordinate system
    private double _initialRadius; // Initial radius (>= 0)
    private double _growthRate;    // Growth rate per radian (> 0)

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

    /// <summary>Creates an Archimedean spiral.</summary>
    /// <exception cref="ConstructionErrorException">theInitialRadius &lt; 0 or theGrowthRate &lt;= 0.</exception>
    public ArchimedeanSpiralCurve2D(in Axis2D position, double initialRadius, double growthRate)
    {
        _position = position;
        _initialRadius = initialRadius;
        _growthRate = growthRate;
        if (initialRadius < 0.0)
        {
            throw new ConstructionErrorException(
                "Geom2dEval_ArchimedeanSpiralCurve: initial radius must be >= 0");
        }
        if (growthRate <= 0.0)
        {
            throw new ConstructionErrorException("Geom2dEval_ArchimedeanSpiralCurve: growth rate must be > 0");
        }
    }

    /// <summary>The local coordinate system.</summary>
    public Axis2D Position() => _position;

    /// <summary>The initial radius a.</summary>
    public double InitialRadius() => _initialRadius;

    /// <summary>The growth rate b per radian.</summary>
    public double GrowthRate() => _growthRate;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("Geom2dEval_ArchimedeanSpiralCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double ReversedParameter(double u)
    {
        throw new MethodNotImplementedException("Geom2dEval_ArchimedeanSpiralCurve::ReversedParameter");
    }

    /// <summary>0.</summary>
    public override double FirstParameter() => 0.0;

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
        double r = _initialRadius + _growthRate * u;
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        return new Point2D(o + r * cosU * xd + r * sinU * yd);
    }

    public override ResD1 EvalD1(double u)
    {
        Xy o = _position.Location().XY();
        Xy xd = _position.Direction().XY();
        Xy yd = new(-xd.Y(), xd.X());
        double r = _initialRadius + _growthRate * u;
        double b = _growthRate;
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        ResD1 result;
        result.Point = new Point2D(o + r * cosU * xd + r * sinU * yd);
        // D1 = (b*cos(t) - r*sin(t))*XDir + (b*sin(t) + r*cos(t))*YDir
        result.D1 = new Vector2D((b * cosU - r * sinU) * xd + (b * sinU + r * cosU) * yd);
        return result;
    }

    public override ResD2 EvalD2(double u)
    {
        Xy o = _position.Location().XY();
        Xy xd = _position.Direction().XY();
        Xy yd = new(-xd.Y(), xd.X());
        double r = _initialRadius + _growthRate * u;
        double b = _growthRate;
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        ResD2 result;
        result.Point = new Point2D(o + r * cosU * xd + r * sinU * yd);
        result.D1 = new Vector2D((b * cosU - r * sinU) * xd + (b * sinU + r * cosU) * yd);
        // D2 = (-2*b*sin(t) - r*cos(t))*XDir + (2*b*cos(t) - r*sin(t))*YDir
        result.D2 =
            new Vector2D((-2.0 * b * sinU - r * cosU) * xd + (2.0 * b * cosU - r * sinU) * yd);
        return result;
    }

    public override ResD3 EvalD3(double u)
    {
        Xy o = _position.Location().XY();
        Xy xd = _position.Direction().XY();
        Xy yd = new(-xd.Y(), xd.X());
        double r = _initialRadius + _growthRate * u;
        double b = _growthRate;
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);

        ResD3 result;
        result.Point = new Point2D(o + r * cosU * xd + r * sinU * yd);
        result.D1 = new Vector2D((b * cosU - r * sinU) * xd + (b * sinU + r * cosU) * yd);
        result.D2 =
            new Vector2D((-2.0 * b * sinU - r * cosU) * xd + (2.0 * b * cosU - r * sinU) * yd);
        // D3 = (-3*b*cos(t) + r*sin(t))*XDir + (-3*b*sin(t) - r*cos(t))*YDir
        result.D3 =
            new Vector2D((-3.0 * b * cosU + r * sinU) * xd + (-3.0 * b * sinU - r * cosU) * yd);
        return result;
    }

    /// <exception cref="RangeErrorException">N &lt; 1.</exception>
    public override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new RangeErrorException("Geom2dEval_ArchimedeanSpiralCurve::EvalDN: N must be >= 1");
        }

        // By Leibniz rule: d^N/dt^N[r(t)*f(t)] = sum_{k=0}^{N} C(N,k)*r^(k)*f^(N-k)
        // where r(t) = a + b*t, r' = b, r^(k) = 0 for k >= 2.
        // So only k=0 and k=1 contribute: = r(t)*f^(N)(t) + N*b*f^(N-1)(t)
        Xy xd = _position.Direction().XY();
        Xy yd = new(-xd.Y(), xd.X());

        double r = _initialRadius + _growthRate * u;
        double b = _growthRate;

        double sinU = Math.Sin(u);
        double cosU = Math.Cos(u);
        TrigShiftByQuarterTurns(sinU, cosU, n, out double sinN, out double cosN);
        TrigShiftByQuarterTurns(sinU, cosU, n - 1, out double sinNm1, out double cosNm1);

        // X component: r*cos^(N)(t) + N*b*cos^(N-1)(t)
        double coeffX = r * cosN + n * b * cosNm1;
        // Y component: r*sin^(N)(t) + N*b*sin^(N-1)(t)
        double coeffY = r * sinN + n * b * sinNm1;

        return new Vector2D(coeffX * xd + coeffY * yd);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform2D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("Geom2dEval_ArchimedeanSpiralCurve::Transform");
    }

    public override Geometry2D Copy() =>
        new ArchimedeanSpiralCurve2D(_position, _initialRadius, _growthRate);
}
