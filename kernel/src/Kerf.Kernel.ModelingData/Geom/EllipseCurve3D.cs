// Port of: src/ModelingData/TKG3d/Geom/Geom_Ellipse.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Ellipse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;

namespace Kerf.Geometry;

/// <summary>
/// An ellipse in 3D space, placed by a right-handed coordinate system (gp_Ax2) whose origin is its centre and
/// whose X direction is its major axis: P(u) = O + MajorRad * cos(u) * XDir + MinorRad * sin(u) * YDir,
/// u in [0, 2 * PI[, periodic. The major radius may equal the minor radius.
/// </summary>
public sealed class EllipseCurve3D : ConicCurve3D
{
    private double _majorRadius;
    private double _minorRadius;

    /// <summary>An ellipse equal to E.</summary>
    public EllipseCurve3D(in Ellipse3D ellipse)
    {
        _majorRadius = ellipse.MajorRadius;
        _minorRadius = ellipse.MinorRadius;
        _pos = ellipse.Position;
    }

    /// <summary>An ellipse placed by A with the given radii.</summary>
    /// <exception cref="ConstructionErrorException">MajorRadius &lt; MinorRadius, or MinorRadius &lt; 0.</exception>
    public EllipseCurve3D(in Frame3D frame, double majorRadius, double minorRadius)
    {
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        if (majorRadius < minorRadius || minorRadius < 0.0)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        _pos = frame;
    }

    /// <summary>A new, unfrozen ellipse equal to this one.</summary>
    public sealed override Geometry3D Copy()
    {
        EllipseCurve3D e;
        e = new EllipseCurve3D(_pos, _majorRadius, _minorRadius);
        return e;
    }

    /// <summary>Always true.</summary>
    public sealed override bool IsClosed => true;

    /// <summary>Always true, with a period of 2 PI.</summary>
    public sealed override bool IsPeriodic => true;

    /// <summary>Always 0.</summary>
    public sealed override double FirstParameter => 0.0;

    /// <summary>Always 2 PI.</summary>
    public sealed override double LastParameter => 2.0 * M_PI;

    /// <summary>The longer semi-axis.</summary>
    public double MajorRadius => _majorRadius;

    /// <summary>The shorter semi-axis.</summary>
    public double MinorRadius => _minorRadius;

    internal void SetElips(in Ellipse3D e)
    {
        CheckMutable();
        _majorRadius = e.MajorRadius;
        _minorRadius = e.MinorRadius;
        _pos = e.Position;
    }

    /// <exception cref="ConstructionErrorException">MajorRadius &lt; the minor radius.</exception>
    internal void SetMajorRadius(double majorRadius)
    {
        CheckMutable();
        if (majorRadius < _minorRadius)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        else
        {
            _majorRadius = majorRadius;
        }
    }

    /// <exception cref="ConstructionErrorException">MinorRadius &lt; 0 or more than the major radius.</exception>
    internal void SetMinorRadius(double minorRadius)
    {
        CheckMutable();
        if (minorRadius < 0 || _majorRadius < minorRadius)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        else
        {
            _minorRadius = minorRadius;
        }
    }

    /// <summary>The non-persistent ellipse equal to this one.</summary>
    public Ellipse3D Ellipse => new Ellipse3D(_pos, _majorRadius, _minorRadius);

    /// <summary>2 * PI - U.</summary>
    public sealed override double ReversedParameter(double u) => (2.0 * M_PI - u);

    /// <summary>The first directrix (see gp_Elips).</summary>
    /// <exception cref="ConstructionErrorException">The eccentricity is 0.</exception>
    public Axis3D Directrix1
    {
        get
        {
            Ellipse3D ev = new Ellipse3D(_pos, _majorRadius, _minorRadius);
            return ev.Directrix1;
        }
    }

    /// <summary>The second directrix, symmetric to the first about the main axis.</summary>
    /// <exception cref="ConstructionErrorException">The eccentricity is 0.</exception>
    public Axis3D Directrix2
    {
        get
        {
            Ellipse3D ev = new Ellipse3D(_pos, _majorRadius, _minorRadius);
            return ev.Directrix2;
        }
    }

    internal sealed override Point3D EvalD0(double u) => global::Kerf.Engine.Geometry.ElCLib.EllipseValue(u, _pos, _majorRadius, _minorRadius);

    internal sealed override ResD1 EvalD1(double u)
    {
        ResD1 result = new ResD1();
        global::Kerf.Engine.Geometry.ElCLib.EllipseD1(u, _pos, _majorRadius, _minorRadius, out result.Point, out result.D1);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u)
    {
        ResD2 result = new ResD2();
        global::Kerf.Engine.Geometry.ElCLib.EllipseD2(u, _pos, _majorRadius, _minorRadius, out result.Point, out result.D1, out result.D2);
        return result;
    }

    internal sealed override ResD3 EvalD3(double u)
    {
        ResD3 result = new ResD3();
        global::Kerf.Engine.Geometry.ElCLib.EllipseD3(u,
                         _pos,
                         _majorRadius,
                         _minorRadius,
                         out result.Point,
                         out result.D1,
                         out result.D2,
                         out result.D3);
        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">N &lt; 1.</exception>
    internal sealed override Vector3D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw Refusal.State(new UndefinedDerivative3DException());
        }
        return global::Kerf.Engine.Geometry.ElCLib.EllipseDN(u, _pos, _majorRadius, _minorRadius, n);
    }

    /// <summary>sqrt(a² - b²) / a, 0 when the major radius is 0.</summary>
    public sealed override double Eccentricity
    {
        get
        {
            if (_majorRadius == 0.0)
            {
                return 0.0;
            }
            else
            {
                return (Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius)) / _majorRadius;
            }
        }
    }

    /// <summary>The distance between the foci.</summary>
    public double Focal => 2.0 * Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);

    /// <summary>The focus on the positive side of the major axis.</summary>
    public Point3D Focus1
    {
        get
        {
            double c = Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);
            double xp, yp, zp, xd, yd, zd;
            _pos.Location.Coord(out xp, out yp, out zp);
            _pos.XDirection.Coord(out xd, out yd, out zd);
            return new Point3D(xp + c * xd, yp + c * yd, zp + c * zd);
        }
    }

    /// <summary>The focus on the negative side of the major axis.</summary>
    public Point3D Focus2
    {
        get
        {
            double c = Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);
            double xp, yp, zp, xd, yd, zd;
            _pos.Location.Coord(out xp, out yp, out zp);
            _pos.XDirection.Coord(out xd, out yd, out zd);
            return new Point3D(xp - c * xd, yp - c * yd, zp - c * zd);
        }
    }

    /// <summary>The parameter of the ellipse: b² / a (0 when a is 0).</summary>
    public double Parameter
    {
        get
        {
            if (_majorRadius == 0.0)
            {
                return 0.0;
            }
            else
            {
                return (_minorRadius * _minorRadius) / _majorRadius;
            }
        }
    }

    /// <summary>Applies T to this ellipse (the radii are scaled by |T.ScaleFactor()|).</summary>
    protected internal sealed override void Transform(in Transform3D transform)
    {
        CheckMutable();
        _majorRadius = _majorRadius * Math.Abs(transform.ScaleFactor);
        _minorRadius = _minorRadius * Math.Abs(transform.ScaleFactor);
        _pos = _pos.Transformed(transform);
    }

    // Non-mutating twins of the Set methods (Kerf)

    /// <summary>A copy that takes its placement and radii from <paramref name="ellipse"/>; this curve is unchanged.</summary>
    public EllipseCurve3D WithEllipse(in Ellipse3D ellipse)
    {
        EllipseCurve3D copy = (EllipseCurve3D)Copy();
        copy.SetElips(ellipse);
        return copy;
    }

    /// <summary>A copy with this major radius; this curve is unchanged.</summary>
    /// <exception cref="ArgumentException"><paramref name="majorRadius"/> is less than the minor radius.</exception>
    public EllipseCurve3D WithMajorRadius(double majorRadius)
    {
        EllipseCurve3D copy = (EllipseCurve3D)Copy();
        copy.SetMajorRadius(majorRadius);
        return copy;
    }

    /// <summary>A copy with this minor radius; this curve is unchanged.</summary>
    /// <exception cref="ArgumentException"><paramref name="minorRadius"/> is negative, or greater than the major radius.</exception>
    public EllipseCurve3D WithMinorRadius(double minorRadius)
    {
        EllipseCurve3D copy = (EllipseCurve3D)Copy();
        copy.SetMinorRadius(minorRadius);
        return copy;
    }
}
