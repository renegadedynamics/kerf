// Port of: src/ModelingData/TKG3d/Geom/Geom_CylindricalSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_CylindricalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// An infinite cylinder of given radius whose axis is the main axis of its local coordinate system.
/// P(U, V) = O + R * (cos(U) * XDir + sin(U) * YDir) + V * ZDir; U in [0, 2*PI], V infinite.
/// </summary>
public sealed class CylindricalSurface : ElementarySurface
{
    private double _radius;

    /// <summary>A cylinder of radius Radius whose local coordinate system is A3.</summary>
    /// <exception cref="ConstructionErrorException">Radius &lt; 0.</exception>
    public CylindricalSurface(in CoordinateSystem a3, double radius)
    {
        _radius = radius;
        if (radius < 0.0)
        {
            throw new ConstructionErrorException();
        }
        _pos = a3;
    }

    /// <summary>A cylinder from a gp cylinder.</summary>
    public CylindricalSurface(in Cylinder c)
    {
        _radius = c.Radius;
        _pos = c.Position;
    }

    public sealed override Geometry3D Copy()
    {
        CylindricalSurface cs;
        cs = new CylindricalSurface(_pos, _radius);
        return cs;
    }

    /// <summary>2*PI - U.</summary>
    public sealed override double UReversedParameter(double u) => (2.0 * M_PI - u);

    /// <summary>-V.</summary>
    public sealed override double VReversedParameter(double v) => (-v);

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

    internal void SetCylinder(in Cylinder c)
    {
        CheckMutable();
        _radius = c.Radius;
        _pos = c.Position;
    }

    /// <exception cref="ConstructionErrorException">R &lt; 0.</exception>
    internal void SetRadius(double r)
    {
        CheckMutable();
        if (r < 0.0)
        {
            throw new ConstructionErrorException();
        }
        _radius = r;
    }

    /// <summary>A copy that is the cylinder C.</summary>
    public CylindricalSurface WithCylinder(in Cylinder c)
    {
        CylindricalSurface copy = (CylindricalSurface)Copy();
        copy.SetCylinder(c);
        return copy;
    }

    /// <summary>A copy of radius R.</summary>
    /// <exception cref="ConstructionErrorException">R &lt; 0.</exception>
    public CylindricalSurface WithRadius(double r)
    {
        CylindricalSurface copy = (CylindricalSurface)Copy();
        copy.SetRadius(r);
        return copy;
    }

    /// <summary>U in [0, 2*PI], V infinite.</summary>
    internal sealed override void ParameterBounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = 0.0;
        u2 = 2.0 * M_PI;
        v1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        v2 = global::Kerf.Engine.Foundation.Precision.Infinite();
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
        a1 = t11 * t11 + t21 * t21;
        a2 = t12 * t12 + t22 * t22;
        a3 = t13 * t13 + t23 * t23;
        b1 = t11 * t12 + t21 * t22;
        b2 = t11 * t13 + t21 * t23;
        b3 = t12 * t13 + t22 * t23;
        c1 = t11 * t14 + t21 * t24;
        c2 = t12 * t14 + t22 * t24;
        c3 = t13 * t14 + t23 * t24;
        d = t14 * t14 + t24 * t24 - _radius * _radius;
    }

    /// <summary>The non-persistent gp cylinder with the same geometry.</summary>
    public Cylinder Cylinder => new Cylinder(_pos, _radius);

    internal sealed override Point3D EvalD0(double u, double v)
    {
        Point3D p;
        global::Kerf.Engine.Geometry.ElSLib.CylinderD0(u, v, _pos, _radius, out p);
        return p;
    }

    internal sealed override ResD1 EvalD1(double u, double v)
    {
        ResD1 result;
        global::Kerf.Engine.Geometry.ElSLib.CylinderD1(u, v, _pos, _radius, out result.Point, out result.D1U, out result.D1V);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u, double v)
    {
        ResD2 result;
        global::Kerf.Engine.Geometry.ElSLib.CylinderD2(u,
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
        global::Kerf.Engine.Geometry.ElSLib.CylinderD3(u,
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
            throw new UndefinedDerivative3DException();
        }
        if (nv > 1)
        {
            return new Vector3D(0.0, 0.0, 0.0);
        }
        else
        {
            return global::Kerf.Engine.Geometry.ElSLib.CylinderDN(u, v, _pos, _radius, nu, nv);
        }
    }

    /// <summary>The U iso-parametric line (a Geom_Line).</summary>
    public sealed override Curve3D UIso(double u)
    {
        LineCurve3D gl = new LineCurve3D(global::Kerf.Engine.Geometry.ElSLib.CylinderUIso(_pos, _radius, u));
        return gl;
    }

    /// <summary>The V iso-parametric circle (a Geom_Circle).</summary>
    public sealed override Curve3D VIso(double v)
    {
        CircleCurve3D gc = new CircleCurve3D(global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(_pos, _radius, v));
        return gc;
    }

    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        _radius = _radius * Math.Abs(t.ScaleFactor);
        _pos = _pos.Transformed(t);
    }

    /// <summary>V scaled by |T.ScaleFactor()| (unless infinite); U unchanged.</summary>
    internal sealed override void TransformParameters(ref double u, ref double v, in Transform3D t)
    {
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(v))
        {
            v *= Math.Abs(t.ScaleFactor);
        }
    }

    /// <summary>An affinity of ratio |T.ScaleFactor()| along V.</summary>
    internal sealed override GeneralTransform2D ParametricTransformation(in Transform3D t)
    {
        GeneralTransform2D t2 = new GeneralTransform2D();
        Axis2D axis = new Axis2D(Constants.Origin2d(), Constants.DX2d());
        t2 = t2.WithAffinity(axis, Math.Abs(t.ScaleFactor));
        return t2;
    }
}
