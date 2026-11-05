// Port of: src/ModelingData/TKG3d/Geom/Geom_Hyperbola.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Hyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Geometry;

/// <summary>
/// The positive branch of a hyperbola in 3D space, placed by a right-handed coordinate system (gp_Ax2) whose
/// origin is its centre and whose X direction is its major axis: P(u) = O + MajRad * cosh(u) * XDir +
/// MinRad * sinh(u) * YDir, u in ]-infinite, +infinite[. The major radius may be less than the minor radius.
/// </summary>
public sealed class HyperbolaCurve3D : ConicCurve3D
{
    private double _majorRadius;
    private double _minorRadius;

    /// <summary>A hyperbola equal to H.</summary>
    public HyperbolaCurve3D(in Hyperbola3D h)
    {
        _majorRadius = h.MajorRadius;
        _minorRadius = h.MinorRadius;
        _pos = h.Position;
    }

    /// <summary>A hyperbola placed by A with the given radii.</summary>
    /// <exception cref="ConstructionErrorException">A radius is negative.</exception>
    public HyperbolaCurve3D(in Frame3D a, double majorRadius, double minorRadius)
    {
        _majorRadius = majorRadius;
        _minorRadius = minorRadius;
        if (majorRadius < 0.0 || minorRadius < 0.0)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        _pos = a;
    }

    /// <summary>A new, unfrozen hyperbola equal to this one.</summary>
    public sealed override Geometry3D Copy()
    {
        HyperbolaCurve3D h;
        h = new HyperbolaCurve3D(_pos, _majorRadius, _minorRadius);
        return h;
    }

    public sealed override bool IsClosed => false;

    public sealed override bool IsPeriodic => false;

    public sealed override double FirstParameter => -global::Kerf.Engine.Foundation.Precision.Infinite();

    public sealed override double LastParameter => global::Kerf.Engine.Foundation.Precision.Infinite();

    public double MajorRadius => _majorRadius;

    public double MinorRadius => _minorRadius;

    internal void SetHypr(in Hyperbola3D h)
    {
        CheckMutable();
        _majorRadius = h.MajorRadius;
        _minorRadius = h.MinorRadius;
        _pos = h.Position;
    }

    /// <exception cref="ConstructionErrorException">MajorRadius &lt; 0.</exception>
    internal void SetMajorRadius(double majorRadius)
    {
        CheckMutable();
        if (majorRadius < 0.0)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        else
        {
            _majorRadius = majorRadius;
        }
    }

    /// <exception cref="ConstructionErrorException">MinorRadius &lt; 0.</exception>
    internal void SetMinorRadius(double minorRadius)
    {
        CheckMutable();
        if (minorRadius < 0.0)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        else
        {
            _minorRadius = minorRadius;
        }
    }

    /// <summary>The non-persistent hyperbola equal to this one.</summary>
    public Hyperbola3D Hyperbola => new Hyperbola3D(_pos, _majorRadius, _minorRadius);

    /// <summary>-U.</summary>
    public sealed override double ReversedParameter(double u) => (-u);

    /// <summary>The first asymptote (see gp_Hypr).</summary>
    /// <exception cref="ConstructionErrorException">The major radius is 0.</exception>
    public Axis3D Asymptote1
    {
        get
        {
            Hyperbola3D hv = new Hyperbola3D(_pos, _majorRadius, _minorRadius);
            return hv.Asymptote1;
        }
    }

    /// <summary>The second asymptote (see gp_Hypr).</summary>
    /// <exception cref="ConstructionErrorException">The major radius is 0.</exception>
    public Axis3D Asymptote2
    {
        get
        {
            Hyperbola3D hv = new Hyperbola3D(_pos, _majorRadius, _minorRadius);
            return hv.Asymptote2;
        }
    }

    /// <summary>The conjugate branch on the positive side of the Y axis.</summary>
    public Hyperbola3D ConjugateBranch1()
    {
        Hyperbola3D hv = new Hyperbola3D(_pos, _majorRadius, _minorRadius);
        return hv.ConjugateBranch1();
    }

    /// <summary>The conjugate branch on the negative side of the Y axis.</summary>
    public Hyperbola3D ConjugateBranch2()
    {
        Hyperbola3D hv = new Hyperbola3D(_pos, _majorRadius, _minorRadius);
        return hv.ConjugateBranch2();
    }

    /// <summary>The first directrix (see gp_Hypr).</summary>
    public Axis3D Directrix1
    {
        get
        {
            Hyperbola3D hv = new Hyperbola3D(_pos, _majorRadius, _minorRadius);
            return hv.Directrix1;
        }
    }

    /// <summary>The second directrix (see gp_Hypr).</summary>
    public Axis3D Directrix2
    {
        get
        {
            Hyperbola3D hv = new Hyperbola3D(_pos, _majorRadius, _minorRadius);
            return hv.Directrix2;
        }
    }

    internal sealed override Point3D EvalD0(double u) => global::Kerf.Engine.Geometry.ElCLib.HyperbolaValue(u, _pos, _majorRadius, _minorRadius);

    internal sealed override ResD1 EvalD1(double u)
    {
        ResD1 result = new ResD1();
        global::Kerf.Engine.Geometry.ElCLib.HyperbolaD1(u, _pos, _majorRadius, _minorRadius, out result.Point, out result.D1);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u)
    {
        ResD2 result = new ResD2();
        global::Kerf.Engine.Geometry.ElCLib.HyperbolaD2(u, _pos, _majorRadius, _minorRadius, out result.Point, out result.D1, out result.D2);
        return result;
    }

    internal sealed override ResD3 EvalD3(double u)
    {
        ResD3 result = new ResD3();
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

    /// <exception cref="UndefinedDerivative3DException">N &lt; 1.</exception>
    internal sealed override Vector3D EvalDN(double u, int n)
    {
        if (n < 1)
        {
            throw Refusal.State(new UndefinedDerivative3DException());
        }
        return global::Kerf.Engine.Geometry.ElCLib.HyperbolaDN(u, _pos, _majorRadius, _minorRadius, n);
    }

    /// <summary>sqrt(a² + b²) / a.</summary>
    /// <exception cref="ConstructionErrorException">The major radius is 0.</exception>
    public sealed override double Eccentricity
    {
        get
        {
            Refusal.StateIf(_majorRadius == 0.0, FailureKind.ConstructionError, " ");
            return (Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius)) / _majorRadius;
        }
    }

    /// <summary>The distance between the foci.</summary>
    public double Focal => 2.0 * Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);

    /// <summary>The focus on the positive side of the major axis.</summary>
    public Point3D Focus1
    {
        get
        {
            double c = Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);
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
            double c = Math.Sqrt(_majorRadius * _majorRadius + _minorRadius * _minorRadius);
            double xp, yp, zp, xd, yd, zd;
            _pos.Location.Coord(out xp, out yp, out zp);
            _pos.XDirection.Coord(out xd, out yd, out zd);
            return new Point3D(xp - c * xd, yp - c * yd, zp - c * zd);
        }
    }

    /// <summary>The branch symmetric to this one about the Y axis.</summary>
    public Hyperbola3D OtherBranch()
    {
        Hyperbola3D hv = new Hyperbola3D(_pos, _majorRadius, _minorRadius);
        return hv.OtherBranch();
    }

    /// <summary>The parameter of the hyperbola: b² / a.</summary>
    /// <exception cref="ConstructionErrorException">The major radius is 0.</exception>
    public double Parameter
    {
        get
        {
            Refusal.StateIf(_majorRadius == 0.0, FailureKind.ConstructionError, " ");
            return (_minorRadius * _minorRadius) / _majorRadius;
        }
    }

    /// <summary>Applies T to this hyperbola (the radii are scaled by |T.ScaleFactor()|).</summary>
    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        _majorRadius = _majorRadius * Math.Abs(t.ScaleFactor);
        _minorRadius = _minorRadius * Math.Abs(t.ScaleFactor);
        _pos = _pos.Transformed(t);
    }

    // Non-mutating twins of the Set methods (Kerf)

    public HyperbolaCurve3D WithHypr(in Hyperbola3D h)
    {
        HyperbolaCurve3D copy = (HyperbolaCurve3D)Copy();
        copy.SetHypr(h);
        return copy;
    }

    public HyperbolaCurve3D WithMajorRadius(double majorRadius)
    {
        HyperbolaCurve3D copy = (HyperbolaCurve3D)Copy();
        copy.SetMajorRadius(majorRadius);
        return copy;
    }

    public HyperbolaCurve3D WithMinorRadius(double minorRadius)
    {
        HyperbolaCurve3D copy = (HyperbolaCurve3D)Copy();
        copy.SetMinorRadius(minorRadius);
        return copy;
    }
}
