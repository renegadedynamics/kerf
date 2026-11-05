// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Ellipse.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Ellipse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// An ellipse in the plane, placed by a local coordinate system whose origin is the centre and whose X axis is
/// the major axis. P(U) = O + MajorRadius*cos(U)*XDir + MinorRadius*sin(U)*YDir, U in [0, 2*PI[; the curve is
/// closed and periodic. The major radius may equal the minor radius (a circle) and the minor radius may be 0.
/// </summary>
public sealed class EllipseCurve2D : ConicCurve2D
{
    private double _majorRadius;
    private double _minorRadius;

    /// <summary>A new, unfrozen ellipse equal to this one.</summary>
    public sealed override Geometry2D Copy()
    {
        EllipseCurve2D e;
        e = new EllipseCurve2D(_pos, _majorRadius, _minorRadius);
        return e;
    }

    /// <summary>The ellipse equal to E.</summary>
    public EllipseCurve2D(in Ellipse2D e)
    {
        _majorRadius = e.MajorRadius;
        _minorRadius = e.MinorRadius;
        _pos = e.Axis;
    }

    /// <summary>The ellipse whose major axis is MajorAxis; Sense gives the orientation (true: counterclockwise).</summary>
    /// <exception cref="ConstructionErrorException">MajorRadius &lt; MinorRadius or MinorRadius &lt; 0.</exception>
    public EllipseCurve2D(in Axis2D majorAxis, double majorRadius, double minorRadius, bool sense = true)
    {
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        if (majorRadius < minorRadius || minorRadius < 0.0)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        _pos = new Frame2D(majorAxis, sense);
    }

    /// <summary>The ellipse in the coordinate system Axis, whose X axis is the major axis.</summary>
    /// <exception cref="ConstructionErrorException">MajorRadius &lt; MinorRadius or MinorRadius &lt; 0.</exception>
    public EllipseCurve2D(in Frame2D axis, double majorRadius, double minorRadius)
    {
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        if (majorRadius < minorRadius || minorRadius < 0.0)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        _pos = axis;
    }

    internal void SetElips2d(in Ellipse2D e)
    {
        CheckMutable();
        _majorRadius = e.MajorRadius;
        _minorRadius = e.MinorRadius;
        _pos = e.Axis;
    }

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

    /// <summary>The ellipse as a gp_Elips2d.</summary>
    public Ellipse2D Ellipse => new Ellipse2D(_pos, _majorRadius, _minorRadius);

    /// <summary>2*PI - U.</summary>
    public sealed override double ReversedParameter(double u) => (2.0 * M_PI - u);

    /// <summary>The first directrix: parallel to the Y axis at distance MajorRadius/e from the centre, on the positive X side.</summary>
    /// <exception cref="ConstructionErrorException">The eccentricity is 0 (a circle).</exception>
    public Axis2D Directrix1
    {
        get
        {
            Ellipse2D ev = new Ellipse2D(_pos, _majorRadius, _minorRadius);
            return ev.Directrix1;
        }
    }

    /// <summary>The second directrix, the symmetric of the first about the Y axis.</summary>
    /// <exception cref="ConstructionErrorException">The eccentricity is 0 (a circle).</exception>
    public Axis2D Directrix2
    {
        get
        {
            Ellipse2D ev = new Ellipse2D(_pos, _majorRadius, _minorRadius);
            return ev.Directrix2;
        }
    }

    /// <summary>sqrt(MajorRadius^2 - MinorRadius^2) / MajorRadius, or 0 if MajorRadius is 0.</summary>
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
    public Point2D Focus1
    {
        get
        {
            double c = Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);
            return new Point2D(_pos.Location.X + c * _pos.XDirection.X,
                                _pos.Location.Y + c * _pos.XDirection.Y);
        }
    }

    /// <summary>The focus on the negative side of the major axis.</summary>
    public Point2D Focus2
    {
        get
        {
            double c = Math.Sqrt(_majorRadius * _majorRadius - _minorRadius * _minorRadius);
            return new Point2D(_pos.Location.X - c * _pos.XDirection.X,
                                _pos.Location.Y - c * _pos.XDirection.Y);
        }
    }

    /// <summary>The major radius.</summary>
    public double MajorRadius => _majorRadius;

    /// <summary>The minor radius.</summary>
    public double MinorRadius => _minorRadius;

    /// <summary>The parameter of the ellipse (MinorRadius^2 / MajorRadius), or 0 if MajorRadius is 0.</summary>
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

    /// <summary>0.</summary>
    public sealed override double FirstParameter => 0.0;

    /// <summary>2*PI.</summary>
    public sealed override double LastParameter => 2.0 * M_PI;

    /// <summary>True.</summary>
    public sealed override bool IsClosed => true;

    /// <summary>True.</summary>
    public sealed override bool IsPeriodic => true;

    internal sealed override Point2D EvalD0(double u) => global::Kerf.Engine.Geometry.ElCLib.EllipseValue(u, _pos, _majorRadius, _minorRadius);

    internal sealed override ResD1 EvalD1(double u)
    {
        ResD1 result;
        global::Kerf.Engine.Geometry.ElCLib.EllipseD1(u, _pos, _majorRadius, _minorRadius, out result.Point, out result.D1);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u)
    {
        ResD2 result;
        global::Kerf.Engine.Geometry.ElCLib.EllipseD2(u, _pos, _majorRadius, _minorRadius, out result.Point, out result.D1, out result.D2);
        return result;
    }

    internal sealed override ResD3 EvalD3(double u)
    {
        ResD3 result;
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

    /// <exception cref="UndefinedDerivative2DException">N &lt; 1.</exception>
    internal sealed override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw Refusal.State(new UndefinedDerivative2DException());
        }
        return global::Kerf.Engine.Geometry.ElCLib.EllipseDN(u, _pos, _majorRadius, _minorRadius, n);
    }

    /// <summary>Applies T to the ellipse: the radii are multiplied by |T.ScaleFactor()|.</summary>
    protected internal sealed override void Transform(in Transform2D t)
    {
        CheckMutable();
        _majorRadius = _majorRadius * Math.Abs(t.ScaleFactor);
        _minorRadius = _minorRadius * Math.Abs(t.ScaleFactor);
        _pos = _pos.Transformed(t);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    /// <summary>A copy equal to E.</summary>
    public EllipseCurve2D WithElips2d(in Ellipse2D e)
    {
        EllipseCurve2D copy = (EllipseCurve2D)Copy();
        copy.SetElips2d(e);
        return copy;
    }

    /// <summary>A copy with the major radius MajorRadius.</summary>
    /// <exception cref="ConstructionErrorException">MajorRadius &lt; MinorRadius().</exception>
    public EllipseCurve2D WithMajorRadius(double majorRadius)
    {
        EllipseCurve2D copy = (EllipseCurve2D)Copy();
        copy.SetMajorRadius(majorRadius);
        return copy;
    }

    /// <summary>A copy with the minor radius MinorRadius.</summary>
    /// <exception cref="ConstructionErrorException">MinorRadius &lt; 0 or MajorRadius() &lt; MinorRadius.</exception>
    public EllipseCurve2D WithMinorRadius(double minorRadius)
    {
        EllipseCurve2D copy = (EllipseCurve2D)Copy();
        copy.SetMinorRadius(minorRadius);
        return copy;
    }
}
