// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_HelixCurve.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_HelixCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Helix.Geometry;

/// <summary>
/// An adaptor evaluating a helix analytically: a cylindrical helix (constant radius) or a tapered one (the radius
/// changing with a taper angle), clockwise or counter-clockwise, about the Z axis.
/// </summary>
/// <remarks>
/// In cylindrical coordinates: x(t) = r(t) cos t, y(t) = r(t) sin t (negated when counter-clockwise),
/// z(t) = pitch t / 2π, with r(t) = rStart + pitch tan(taper) t / 2π. An adaptor is a working object (decision 21).
/// </remarks>
internal class HelixCurve : CurveAdaptorBase
{
    protected double _first;
    protected double _last;
    protected double _pitch;
    protected double _rStart;
    protected double _taperAngle;
    protected bool _isClockWise;
    protected double _c1;
    protected double _tgBeta;
    protected double _tolAngle;

    /// <summary>A helix of one turn, pitch 1 and radius 1, clockwise.</summary>
    public HelixCurve()
    {
        _first = 0.0;
        _last = 2.0 * M_PI;
        _pitch = 1.0;
        _rStart = 1.0;
        _taperAngle = 0.0;
        _isClockWise = true;
        // Calculate derived parameters
        _c1 = _pitch / _last;
        _tgBeta = 0.0;
        _tolAngle = 1.0e-4;
    }

    /// <summary>A copy of <paramref name="other"/> (OCCT's implicit copy constructor).</summary>
    public HelixCurve(HelixCurve other)
    {
        _first = other._first;
        _last = other._last;
        _pitch = other._pitch;
        _rStart = other._rStart;
        _taperAngle = other._taperAngle;
        _isClockWise = other._isClockWise;
        _c1 = other._c1;
        _tgBeta = other._tgBeta;
        _tolAngle = other._tolAngle;
    }

    /// <summary>Reloads the current parameters (validating them and recomputing the derived values).</summary>
    public void Load()
    {
        Load(_first, _last, _pitch, _rStart, _taperAngle, _isClockWise);
    }

    /// <summary>Sets the helix parameters.</summary>
    /// <exception cref="ConstructionErrorException">
    /// <paramref name="aT1"/> ≥ <paramref name="aT2"/>, a negative pitch or radius, or a taper angle outside (-π/2, π/2).
    /// </exception>
    public void Load(double aT1, double aT2, double aPitch, double aRStart, double aTaperAngle, bool aIsCW)
    {
        const string buf = "HelixGeom_HelixCurve::Load";
        double twoPI, halfPI;
        // Define angular constants
        twoPI = 2.0 * M_PI;
        halfPI = 0.5 * M_PI;
        // Store parameter values
        _first = aT1;
        _last = aT2;
        _pitch = aPitch;
        _rStart = aRStart;
        _taperAngle = aTaperAngle;
        _isClockWise = aIsCW;
        // Validate input parameters
        if (aT1 >= aT2)
        {
            throw new ConstructionErrorException(buf);
        }
        if (_pitch < 0.0)
        {
            throw new ConstructionErrorException(buf);
        }
        if (_rStart < 0.0)
        {
            throw new ConstructionErrorException(buf);
        }
        if (_taperAngle <= -halfPI || _taperAngle >= halfPI)
        {
            throw new ConstructionErrorException(buf);
        }
        // Calculate helix coefficient
        _c1 = _pitch / twoPI;
        // OCCT: myTgBeta isn't reset for a (near) zero taper angle, so reloading a tapered helix as a cylindrical
        // one keeps the old taper (occt-issues.md).
        if (Math.Abs(_taperAngle) > _tolAngle)
        {
            _tgBeta = Math.Tan(_taperAngle);
        }
    }

    public override double FirstParameter() => _first;

    public override double LastParameter() => _last;

    /// <summary>GeomAbs_CN.</summary>
    public override Continuity Continuity() => global::Kerf.Geometry.Continuity.CN;

    /// <summary>1, whatever the continuity.</summary>
    public override int NbIntervals(Continuity s) => 1;

    /// <summary>Stores the first and last parameters in T(1) and T(2).</summary>
    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        t[1] = _first;
        t[2] = _last;
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double Resolution(double r3d) => throw new MethodNotImplementedException("HelixGeom_HelixCurve::Resolution");

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override bool IsClosed => throw new MethodNotImplementedException("HelixGeom_HelixCurve::IsClosed");

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override bool IsPeriodic => throw new MethodNotImplementedException("HelixGeom_HelixCurve::IsPeriodic");

    /// <summary>Throws <see cref="DomainErrorException"/>: the helix isn't periodic.</summary>
    public override double Period() => throw new DomainErrorException("HelixGeom_HelixCurve::Period");

    /// <summary>The point of parameter <paramref name="t"/>.</summary>
    public sealed override Point3D EvalD0(double t)
    {
        double st, ct, x, y, z, a1;
        // Calculate trigonometric values and radius
        ct = Math.Cos(t);
        st = Math.Sin(t);
        a1 = _rStart + _c1 * _tgBeta * t;
        // Calculate Cartesian coordinates
        x = a1 * ct;
        y = a1 * st;
        if (!_isClockWise)
        {
            y = -y;
        }
        z = _c1 * t;
        return new Point3D(x, y, z);
    }

    /// <summary>The point and first derivative at <paramref name="t"/>.</summary>
    internal sealed override Curve3D.ResD1 EvalD1(double t)
    {
        double st, ct, x, y, z, a1, a2;
        // Calculate point and first derivative
        ct = Math.Cos(t);
        st = Math.Sin(t);
        // Calculate radius at parameter t
        a1 = _rStart + _c1 * _tgBeta * t;
        // Calculate point coordinates
        x = a1 * ct;
        y = a1 * st;
        if (!_isClockWise)
        {
            y = -y;
        }
        z = _c1 * t;
        Point3D p = new(x, y, z);
        // Calculate first derivative coefficients
        a1 = _c1 * _tgBeta;
        a2 = _rStart + a1 * t;
        // Calculate first derivative components
        x = a1 * ct - a2 * st;
        y = a1 * st + a2 * ct;
        if (!_isClockWise)
        {
            y = -y;
        }
        z = _c1;
        Vector3D v1 = new(x, y, z);
        return new Curve3D.ResD1 { Point = p, D1 = v1 };
    }

    /// <summary>The point and first two derivatives at <paramref name="t"/>.</summary>
    internal sealed override Curve3D.ResD2 EvalD2(double t)
    {
        double st, ct, x, y, z, a1, a2;
        // Calculate point, first and second derivatives
        ct = Math.Cos(t);
        st = Math.Sin(t);
        // Calculate radius at parameter t
        a1 = _rStart + _c1 * _tgBeta * t;
        // Calculate point coordinates
        x = a1 * ct;
        y = a1 * st;
        if (!_isClockWise)
        {
            y = -y;
        }
        z = _c1 * t;
        Point3D p = new(x, y, z);
        // Calculate first derivative coefficients
        a1 = _c1 * _tgBeta;
        a2 = _rStart + a1 * t;
        // Calculate first derivative components
        x = a1 * ct - a2 * st;
        y = a1 * st + a2 * ct;
        if (!_isClockWise)
        {
            y = -y;
        }
        z = _c1;
        Vector3D v1 = new(x, y, z);
        // Calculate second derivative
        a1 = 2.0 * a1;
        x = -a2 * ct - a1 * st;
        y = -a2 * st - a1 * ct;
        if (!_isClockWise)
        {
            y = -y;
        }
        z = 0.0;
        Vector3D v2 = new(x, y, z);
        return new Curve3D.ResD2 { Point = p, D1 = v1, D2 = v2 };
    }

    /// <summary>The derivative of order 1 or 2 at <paramref name="t"/>; throws <see cref="MethodNotImplementedException"/> for other orders.</summary>
    public sealed override Vector3D EvalDN(double t, int n)
    {
        // Compute derivative based on order
        switch (n)
        {
            case 1:
            {
                Curve3D.ResD1 res = EvalD1(t);
                return res.D1;
            }
            case 2:
            {
                Curve3D.ResD2 res = EvalD2(t);
                return res.D2;
            }
            default:
                throw new MethodNotImplementedException("HelixGeom_HelixCurve::EvalDN");
        }
    }
}
