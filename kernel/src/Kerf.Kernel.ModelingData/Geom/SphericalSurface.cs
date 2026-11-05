// Port of: src/ModelingData/TKG3d/Geom/Geom_SphericalSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_SphericalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Geometry;

/// <summary>
/// A sphere of given radius centred on the origin of its local coordinate system.
/// P(U, V) = O + R * cos(V) * (cos(U) * XDir + sin(U) * YDir) + R * sin(V) * ZDir;
/// U in [0, 2*PI], V in [-PI/2, PI/2].
/// </summary>
public sealed class SphericalSurface : ElementarySurface
{
    private double _radius;

    /// <summary>A sphere of radius Radius whose local coordinate system is A3.</summary>
    /// <exception cref="ConstructionErrorException">Radius &lt; 0.</exception>
    public SphericalSurface(in CoordinateSystem a3, double radius)
    {
        _radius = radius;
        if (radius < 0.0)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        _pos = a3;
    }

    /// <summary>A sphere from a gp sphere.</summary>
    public SphericalSurface(in Sphere3D s)
    {
        _radius = s.Radius;
        _pos = s.Position;
    }

    public sealed override Geometry3D Copy()
    {
        SphericalSurface cs;
        cs = new SphericalSurface(_pos, _radius);
        return cs;
    }

    /// <summary>2*PI - U.</summary>
    public sealed override double UReversedParameter(double u) => (2.0 * M_PI - u);

    /// <summary>-V.</summary>
    public sealed override double VReversedParameter(double v) => (-v);

    /// <summary>The area, 4 * PI * R * R.</summary>
    public double Area => 4.0 * M_PI * _radius * _radius;

    /// <summary>The radius.</summary>
    public double Radius => _radius;

    /// <summary>True.</summary>
    public sealed override bool IsUClosed => true;

    /// <summary>False.</summary>
    public sealed override bool IsVClosed => false;

    /// <summary>True (period 2*PI).</summary>
    public sealed override bool IsUPeriodic => true;

    /// <summary>False.</summary>
    public sealed override bool IsVPeriodic => false;

    /// <exception cref="ConstructionErrorException">R &lt; 0.</exception>
    internal void SetRadius(double r)
    {
        CheckMutable();
        if (r < 0.0)
        {
            throw Refusal.Argument(new ConstructionErrorException());
        }
        _radius = r;
    }

    internal void SetSphere(in Sphere3D s)
    {
        CheckMutable();
        _radius = s.Radius;
        _pos = s.Position;
    }

    /// <summary>A copy of radius R.</summary>
    /// <exception cref="ConstructionErrorException">R &lt; 0.</exception>
    public SphericalSurface WithRadius(double r)
    {
        SphericalSurface copy = (SphericalSurface)Copy();
        copy.SetRadius(r);
        return copy;
    }

    /// <summary>A copy that is the sphere S.</summary>
    public SphericalSurface WithSphere(in Sphere3D s)
    {
        SphericalSurface copy = (SphericalSurface)Copy();
        copy.SetSphere(s);
        return copy;
    }

    /// <summary>U in [0, 2*PI], V in [-PI/2, PI/2].</summary>
    internal sealed override void ParameterBounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = 0.0;
        u2 = M_PI * 2.0;
        v1 = -M_PI / 2.0;
        v2 = M_PI / 2.0;
    }

    /// <summary>
    /// The coefficients of the implicit equation in the absolute system:
    /// A1.X**2 + A2.Y**2 + A3.Z**2 + 2.(B1.X.Y + B2.X.Z + B3.Y.Z) + 2.(C1.X + C2.Y + C3.Z) + D = 0.
    /// </summary>
    internal void Coefficients(out double a1,
                             out double a2,
                             out double a3,
                             out double b1,
                             out double b2,
                             out double b3,
                             out double c1,
                             out double c2,
                             out double c3,
                             out double d)
    {
        Transform3D t = new Transform3D().WithTransformation(_pos);
        double t11 = t.Value(1, 1);
        double t12 = t.Value(1, 2);
        double t13 = t.Value(1, 3);
        double t14 = t.Value(1, 4);
        double t21 = t.Value(2, 1);
        double t22 = t.Value(2, 2);
        double t23 = t.Value(2, 3);
        double t24 = t.Value(2, 4);
        double t31 = t.Value(3, 1);
        double t32 = t.Value(3, 2);
        double t33 = t.Value(3, 3);
        double t34 = t.Value(3, 4);
        a1 = t11 * t11 + t21 * t21 + t31 * t31;
        a2 = t12 * t12 + t22 * t22 + t32 * t32;
        a3 = t13 * t13 + t23 * t23 + t33 * t33;
        b1 = t11 * t12 + t21 * t22 + t31 * t32;
        b2 = t11 * t13 + t21 * t23 + t31 * t33;
        b3 = t12 * t13 + t22 * t23 + t32 * t33;
        c1 = t11 * t14 + t21 * t24 + t31 * t34;
        c2 = t12 * t14 + t22 * t24 + t32 * t34;
        c3 = t13 * t14 + t23 * t24 + t33 * t34;
        d = t14 * t14 + t24 * t24 + t34 * t34 - _radius * _radius;
    }

    internal sealed override Point3D EvalD0(double u, double v)
    {
        Point3D p;
        global::Kerf.Engine.Geometry.ElSLib.SphereD0(u, v, _pos, _radius, out p);
        return p;
    }

    internal sealed override ResD1 EvalD1(double u, double v)
    {
        ResD1 result;
        global::Kerf.Engine.Geometry.ElSLib.SphereD1(u, v, _pos, _radius, out result.Point, out result.D1U, out result.D1V);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u, double v)
    {
        ResD2 result;
        global::Kerf.Engine.Geometry.ElSLib.SphereD2(u,
                        v,
                        _pos,
                        _radius,
                        out result.Point,
                        out result.D1U,
                        out result.D1V,
                        out result.D2U,
                        out result.D2V,
                        out result.D2UV);
        return result;
    }

    internal sealed override ResD3 EvalD3(double u, double v)
    {
        ResD3 result;
        global::Kerf.Engine.Geometry.ElSLib.SphereD3(u,
                        v,
                        _pos,
                        _radius,
                        out result.Point,
                        out result.D1U,
                        out result.D1V,
                        out result.D2U,
                        out result.D2V,
                        out result.D2UV,
                        out result.D3U,
                        out result.D3V,
                        out result.D3UUV,
                        out result.D3UVV);
        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    internal sealed override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw Refusal.State(new UndefinedDerivative3DException());
        }
        return global::Kerf.Engine.Geometry.ElSLib.SphereDN(u, v, _pos, _radius, nu, nv);
    }

    /// <summary>The non-persistent gp sphere with the same geometry.</summary>
    public Sphere3D Sphere => new Sphere3D(_pos, _radius);

    /// <summary>The U iso-parametric half circle (a Geom_TrimmedCurve on a Geom_Circle, in [-PI/2, PI/2]).</summary>
    public sealed override Curve3D UIso(double u)
    {
        CircleCurve3D gc = new CircleCurve3D(global::Kerf.Engine.Geometry.ElSLib.SphereUIso(_pos, _radius, u));
        TrimmedCurve3D iso = new TrimmedCurve3D(gc, -M_PI / 2.0, M_PI / 2);
        return iso;
    }

    /// <summary>The V iso-parametric circle (a Geom_Circle).</summary>
    public sealed override Curve3D VIso(double v)
    {
        CircleCurve3D gc = new CircleCurve3D(global::Kerf.Engine.Geometry.ElSLib.SphereVIso(_pos, _radius, v));
        return gc;
    }

    /// <summary>The volume, 4 * PI * R**3 / 3.</summary>
    public double Volume => (4.0 * M_PI * _radius * _radius * _radius) / 3.0;

    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        _radius = _radius * Math.Abs(t.ScaleFactor);
        _pos = _pos.Transformed(t);
    }
}
