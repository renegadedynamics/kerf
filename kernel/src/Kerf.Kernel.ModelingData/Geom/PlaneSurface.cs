// Port of: src/ModelingData/TKG3d/Geom/Geom_Plane.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_Plane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// An infinite plane, located in 3D space by a coordinate system (gp_Ax3) whose origin is the origin of the
/// parameters and whose X and Y directions are the U and V parametric directions. The main direction is the
/// normal. P(U, V) = O + U * XDir + V * YDir.
/// </summary>
public sealed class PlaneSurface : ElementarySurface
{
    /// <summary>A plane located by A3: its origin, XDirection and YDirection, and normal.</summary>
    public PlaneSurface(in CoordinateSystem a3)
    {
        _pos = a3;
    }

    /// <summary>A plane from a gp plane.</summary>
    public PlaneSurface(in Plane3D pl)
    {
        _pos = pl.Position;
    }

    /// <summary>A plane through P, normal to V; X and Y directions computed as by gp_Pln.</summary>
    public PlaneSurface(in Point3D p, in Direction3D v)
    {
        Plane3D pl = new Plane3D(p, v);
        _pos = pl.Position;
    }

    /// <summary>The plane A * X + B * Y + C * Z + D = 0.</summary>
    /// <exception cref="ConstructionErrorException">Sqrt(A*A + B*B + C*C) &lt;= gp::Resolution().</exception>
    public PlaneSurface(double a, double b, double c, double d)
    {
        Plane3D pl = new Plane3D(a, b, c, d);
        _pos = pl.Position;
    }

    internal void SetPln(in Plane3D pl)
    {
        CheckMutable();
        _pos = pl.Position;
    }

    /// <summary>A copy that is the plane Pl.</summary>
    public PlaneSurface WithPln(in Plane3D pl)
    {
        PlaneSurface copy = (PlaneSurface)Copy();
        copy.SetPln(pl);
        return copy;
    }

    /// <summary>The non-persistent gp plane with the same geometry.</summary>
    public Plane3D Plane => new Plane3D(Position);

    /// <summary>Reverses the U direction (the X direction of the local coordinate system).</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        _pos = _pos.XReversed();
    }

    /// <summary>-U.</summary>
    public sealed override double UReversedParameter(double u) => (-u);

    /// <summary>Reverses the V direction (the Y direction of the local coordinate system).</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        _pos = _pos.YReversed();
    }

    /// <summary>-V.</summary>
    public sealed override double VReversedParameter(double v) => (-v);

    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        _pos = _pos.Transformed(t);
    }

    /// <summary>False.</summary>
    public sealed override bool IsUClosed => false;

    /// <summary>False.</summary>
    public sealed override bool IsVClosed => false;

    /// <summary>False.</summary>
    public sealed override bool IsUPeriodic => false;

    /// <summary>False.</summary>
    public sealed override bool IsVPeriodic => false;

    /// <summary>Infinite bounds: U1 = V1 = -Precision::Infinite(), U2 = V2 = Precision::Infinite().</summary>
    internal sealed override void ParameterBounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        u2 = global::Kerf.Engine.Foundation.Precision.Infinite();
        v1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        v2 = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>The coefficients of the implicit equation A * X + B * Y + C * Z + D = 0 in the absolute system.</summary>
    internal void Coefficients(out double a, out double b, out double c, out double d)
    {
        Plane3D pl = new Plane3D(Position);
        pl.Coefficients(out a, out b, out c, out d);
    }

    internal sealed override Point3D EvalD0(double u, double v) => global::Kerf.Engine.Geometry.ElSLib.PlaneValue(u, v, _pos);

    internal sealed override ResD1 EvalD1(double u, double v)
    {
        ResD1 result;
        global::Kerf.Engine.Geometry.ElSLib.PlaneD1(u, v, _pos, out result.Point, out result.D1U, out result.D1V);
        return result;
    }

    internal sealed override ResD2 EvalD2(double u, double v)
    {
        ResD2 result;
        global::Kerf.Engine.Geometry.ElSLib.PlaneD1(u, v, _pos, out result.Point, out result.D1U, out result.D1V);
        result.D2U = new Vector3D(0.0, 0.0, 0.0);
        result.D2V = new Vector3D(0.0, 0.0, 0.0);
        result.D2UV = new Vector3D(0.0, 0.0, 0.0);
        return result;
    }

    internal sealed override ResD3 EvalD3(double u, double v)
    {
        ResD3 result;
        global::Kerf.Engine.Geometry.ElSLib.PlaneD1(u, v, _pos, out result.Point, out result.D1U, out result.D1V);
        result.D2U = new Vector3D(0.0, 0.0, 0.0);
        result.D2V = new Vector3D(0.0, 0.0, 0.0);
        result.D2UV = new Vector3D(0.0, 0.0, 0.0);
        result.D3U = new Vector3D(0.0, 0.0, 0.0);
        result.D3V = new Vector3D(0.0, 0.0, 0.0);
        result.D3UUV = new Vector3D(0.0, 0.0, 0.0);
        result.D3UVV = new Vector3D(0.0, 0.0, 0.0);
        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    internal sealed override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw Refusal.State(new UndefinedDerivative3DException());
        }
        if (nu == 0 && nv == 1)
        {
            return new Vector3D(_pos.YDirection);
        }
        else if (nu == 1 && nv == 0)
        {
            return new Vector3D(_pos.XDirection);
        }
        return new Vector3D(0.0, 0.0, 0.0);
    }

    /// <summary>The U iso-parametric line (a Geom_Line).</summary>
    public sealed override Curve3D UIso(double u)
    {
        LineCurve3D gl = new LineCurve3D(global::Kerf.Engine.Geometry.ElSLib.PlaneUIso(_pos, u));
        return gl;
    }

    /// <summary>The V iso-parametric line (a Geom_Line).</summary>
    public sealed override Curve3D VIso(double v)
    {
        LineCurve3D gl = new LineCurve3D(global::Kerf.Engine.Geometry.ElSLib.PlaneVIso(_pos, v));
        return gl;
    }

    /// <summary>(U, V) scaled by |T.ScaleFactor()| (infinite values unchanged).</summary>
    internal sealed override void TransformParameters(ref double u, ref double v, in Transform3D t)
    {
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u))
        {
            u *= Math.Abs(t.ScaleFactor);
        }
        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(v))
        {
            v *= Math.Abs(t.ScaleFactor);
        }
    }

    /// <summary>A scale of |T.ScaleFactor()| about the parametric origin.</summary>
    internal sealed override GeneralTransform2D ParametricTransformation(in Transform3D t)
    {
        Transform2D t2 = new Transform2D().WithScale(Constants.Origin2d(), Math.Abs(t.ScaleFactor));
        return new GeneralTransform2D(t2);
    }

    public sealed override Geometry3D Copy()
    {
        PlaneSurface pl = new PlaneSurface(_pos);
        return pl;
    }
}
