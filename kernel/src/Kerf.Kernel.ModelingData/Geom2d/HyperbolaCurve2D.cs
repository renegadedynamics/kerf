// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Hyperbola.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_Hyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// The main branch of a hyperbola in the plane, placed by a local coordinate system whose origin is the centre
/// and whose X axis is the major axis. P(U) = O + MajorRadius*cosh(U)*XDir + MinorRadius*sinh(U)*YDir, U in
/// ]-infinite, +infinite[. The radii may be 0, and the major radius may be less than the minor one.
/// </summary>
public class HyperbolaCurve2D : ConicCurve2D
{
    private double _majorRadius;
    private double _minorRadius;

    /// <summary>A new, unfrozen hyperbola equal to this one.</summary>
    public sealed override Geometry2D Copy()
    {
        HyperbolaCurve2D h;
        h = new HyperbolaCurve2D(_pos, _majorRadius, _minorRadius);
        return h;
    }

    /// <summary>The hyperbola equal to H.</summary>
    public HyperbolaCurve2D(in Hyperbola2D h)
    {
        _majorRadius = h.MajorRadius();
        _minorRadius = h.MinorRadius();
        _pos = h.Axis();
    }

    /// <summary>The hyperbola whose major axis is MajorAxis; Sense gives the orientation (true: counterclockwise).</summary>
    /// <exception cref="ConstructionErrorException">A radius is negative.</exception>
    public HyperbolaCurve2D(in Axis2D majorAxis, double majorRadius, double minorRadius, bool sense = true)
    {
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        if (majorRadius < 0.0 || minorRadius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        _pos = new Frame2D(majorAxis, sense);
    }

    /// <summary>The hyperbola in the coordinate system Axis, whose X axis is the major axis.</summary>
    /// <exception cref="ConstructionErrorException">A radius is negative.</exception>
    public HyperbolaCurve2D(in Frame2D axis, double majorRadius, double minorRadius)
    {
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        if (majorRadius < 0.0 || minorRadius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        _pos = axis;
    }

    internal void SetHypr2d(in Hyperbola2D h)
    {
        CheckMutable();
        _majorRadius = h.MajorRadius();
        _minorRadius = h.MinorRadius();
        _pos = h.Axis();
    }

    internal void SetMajorRadius(double majorRadius)
    {
        CheckMutable();
        if (majorRadius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        else
        {
            _majorRadius = majorRadius;
        }
    }

    internal void SetMinorRadius(double minorRadius)
    {
        CheckMutable();
        if (minorRadius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        else
        {
            _minorRadius = minorRadius;
        }
    }

    /// <summary>The hyperbola as a gp_Hypr2d.</summary>
    public Hyperbola2D Hypr2d() => new Hyperbola2D(_pos, _majorRadius, _minorRadius);

    /// <summary>-U.</summary>
    public sealed override double ReversedParameter(double u) => (-u);

    /// <summary>-Precision::Infinite().</summary>
    public sealed override double FirstParameter() => -global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public sealed override double LastParameter() => global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>False.</summary>
    public sealed override bool IsClosed => false;

    /// <summary>False.</summary>
    public sealed override bool IsPeriodic => false;

    /// <summary>The first asymptote (see gp_Hypr2d::Asymptote1, which has an OCCT bug: occt-issues.md).</summary>
    /// <exception cref="ConstructionErrorException">MajorRadius is 0.</exception>
    public Axis2D Asymptote1()
    {
        Hyperbola2D hv = new Hyperbola2D(_pos, _majorRadius, _minorRadius);
        return hv.Asymptote1();
    }

    /// <summary>The second asymptote (see gp_Hypr2d::Asymptote2, which has an OCCT bug: occt-issues.md).</summary>
    /// <exception cref="ConstructionErrorException">MajorRadius is 0.</exception>
    public Axis2D Asymptote2()
    {
        Hyperbola2D hv = new Hyperbola2D(_pos, _majorRadius, _minorRadius);
        return hv.Asymptote2();
    }

    /// <summary>The conjugate branch on the positive side of the Y axis.</summary>
    public Hyperbola2D ConjugateBranch1()
    {
        Hyperbola2D hv = new Hyperbola2D(_pos, _majorRadius, _minorRadius);
        return hv.ConjugateBranch1();
    }

    /// <summary>The conjugate branch on the negative side of the Y axis.</summary>
    public Hyperbola2D ConjugateBranch2()
    {
        Hyperbola2D hv = new Hyperbola2D(_pos, _majorRadius, _minorRadius);
        return hv.ConjugateBranch2();
    }

    /// <summary>The directrix on the positive side of the X axis.</summary>
    public Axis2D Directrix1()
    {
        Hyperbola2D hv = new Hyperbola2D(_pos, _majorRadius, _minorRadius);
        return hv.Directrix1();
    }

    /// <summary>The directrix on the negative side of the X axis (see gp_Hypr2d::Directrix2, occt-issues.md).</summary>
    public Axis2D Directrix2()
    {
        Hyperbola2D hv = new Hyperbola2D(_pos, _majorRadius, _minorRadius);
        return hv.Directrix2();
    }

    /// <summary>sqrt(MajorRadius^2 + MinorRadius^2) / MajorRadius.</summary>
    /// <exception cref="DomainErrorException">MajorRadius is 0.</exception>
    public sealed override double Eccentricity()
    {
        Standard_DomainError_Raise_if(_majorRadius <= Constants.Resolution(), " ");
        return (Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius)) / _majorRadius;
    }

    /// <summary>The distance between the foci.</summary>
    public double Focal() => 2.0 * Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);

    /// <summary>The focus on the positive side of the major axis.</summary>
    public Point2D Focus1()
    {
        double c = Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);
        Xy pxy = _pos.XDirection().XY();
        pxy.Multiply(c);
        pxy.Add(_pos.Location().XY());
        return new Point2D(pxy);
    }

    /// <summary>The focus on the negative side of the major axis.</summary>
    public Point2D Focus2()
    {
        double c = Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);
        Xy pxy = _pos.XDirection().XY();
        pxy.Multiply(-c);
        pxy.Add(_pos.Location().XY());
        return new Point2D(pxy);
    }

    /// <summary>The major radius.</summary>
    public double MajorRadius() => _majorRadius;

    /// <summary>The minor radius.</summary>
    public double MinorRadius() => _minorRadius;

    /// <summary>The other branch, symmetric about the Y axis.</summary>
    public Hyperbola2D OtherBranch()
    {
        Hyperbola2D hv = new Hyperbola2D(_pos, _majorRadius, _minorRadius);
        return hv.OtherBranch();
    }

    /// <summary>MinorRadius^2 / MajorRadius.</summary>
    /// <exception cref="DomainErrorException">MajorRadius is 0.</exception>
    public double Parameter()
    {
        Standard_DomainError_Raise_if(_majorRadius <= Constants.Resolution(), " ");
        return (_minorRadius * _minorRadius) / _majorRadius;
    }

    public sealed override Point2D EvalD0(double u) => global::Kerf.Engine.Geometry.ElCLib.HyperbolaValue(u, _pos, _majorRadius, _minorRadius);

    public sealed override ResD1 EvalD1(double u)
    {
        ResD1 result;
        global::Kerf.Engine.Geometry.ElCLib.HyperbolaD1(u, _pos, _majorRadius, _minorRadius, out result.Point, out result.D1);
        return result;
    }

    public sealed override ResD2 EvalD2(double u)
    {
        ResD2 result;
        global::Kerf.Engine.Geometry.ElCLib.HyperbolaD2(u, _pos, _majorRadius, _minorRadius, out result.Point, out result.D1, out result.D2);
        return result;
    }

    public sealed override ResD3 EvalD3(double u)
    {
        ResD3 result;
        global::Kerf.Engine.Geometry.ElCLib.HyperbolaD3(u,
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
    public sealed override Vector2D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw new UndefinedDerivative2DException();
        }
        return global::Kerf.Engine.Geometry.ElCLib.HyperbolaDN(u, _pos, _majorRadius, _minorRadius, n);
    }

    /// <summary>Applies T to the hyperbola: the radii are multiplied by |T.ScaleFactor()|.</summary>
    protected internal sealed override void Transform(in Transform2D t)
    {
        CheckMutable();
        _majorRadius = _majorRadius * Math.Abs(t.ScaleFactor());
        _minorRadius = _minorRadius * Math.Abs(t.ScaleFactor());
        _pos.Transform(t);
    }

    // Non-mutating twins of the Set methods (OCCT.NET)

    /// <summary>A copy equal to H.</summary>
    public HyperbolaCurve2D WithHypr2d(in Hyperbola2D h)
    {
        HyperbolaCurve2D copy = (HyperbolaCurve2D)Copy();
        copy.SetHypr2d(h);
        return copy;
    }

    /// <summary>A copy with the major radius MajorRadius.</summary>
    /// <exception cref="ConstructionErrorException">MajorRadius &lt; 0.</exception>
    public HyperbolaCurve2D WithMajorRadius(double majorRadius)
    {
        HyperbolaCurve2D copy = (HyperbolaCurve2D)Copy();
        copy.SetMajorRadius(majorRadius);
        return copy;
    }

    /// <summary>A copy with the minor radius MinorRadius.</summary>
    /// <exception cref="ConstructionErrorException">MinorRadius &lt; 0.</exception>
    public HyperbolaCurve2D WithMinorRadius(double minorRadius)
    {
        HyperbolaCurve2D copy = (HyperbolaCurve2D)Copy();
        copy.SetMinorRadius(minorRadius);
        return copy;
    }
}
