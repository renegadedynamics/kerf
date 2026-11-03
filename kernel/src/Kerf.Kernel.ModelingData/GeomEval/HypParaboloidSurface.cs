// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_HypParaboloidSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_HypParaboloidSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A hyperbolic paraboloid (saddle) with semi-axis lengths A, B &gt; 0, its saddle point at the origin of its
/// coordinate system: <c>P(u,v) = O + u*XDir + v*YDir + (u^2/A^2 - v^2/B^2)*ZDir</c>, u and v in (-inf, +inf).
/// Doubly ruled, neither periodic nor closed. Implicit equation in local coordinates:
/// <c>X^2/A^2 - Y^2/B^2 - Z = 0</c>. Reversal, transformation and iso curves are not supported (they throw
/// <see cref="MethodNotImplementedException"/>).
/// </summary>
internal sealed class HypParaboloidSurface : ElementarySurface
{
    private double _a; // Semi-axis length A (> 0)
    private double _b; // Semi-axis length B (> 0)

    /// <summary>Creates a hyperbolic paraboloid with the given local coordinate system and semi-axes.</summary>
    /// <exception cref="ConstructionErrorException">theA &lt;= 0 or theB &lt;= 0.</exception>
    public HypParaboloidSurface(in CoordinateSystem position, double a, double b)
    {
        _a = a;
        _b = b;
        if (a <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_HypParaboloidSurface: semi-axis A must be > 0");
        }
        if (b <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_HypParaboloidSurface: semi-axis B must be > 0");
        }
        _pos = position;
    }

    /// <summary>The semi-axis length A.</summary>
    public double SemiAxisA() => _a;

    /// <summary>The semi-axis length B.</summary>
    public double SemiAxisB() => _b;

    /// <exception cref="ConstructionErrorException">theA &lt;= 0.</exception>
    internal void SetSemiAxisA(double a)
    {
        CheckMutable();
        if (a <= 0.0)
        {
            throw new ConstructionErrorException(
                "GeomEval_HypParaboloidSurface::SetSemiAxisA: semi-axis A must be > 0");
        }
        _a = a;
    }

    /// <exception cref="ConstructionErrorException">theB &lt;= 0.</exception>
    internal void SetSemiAxisB(double b)
    {
        CheckMutable();
        if (b <= 0.0)
        {
            throw new ConstructionErrorException(
                "GeomEval_HypParaboloidSurface::SetSemiAxisB: semi-axis B must be > 0");
        }
        _b = b;
    }

    /// <summary>A copy with the semi-axis A set to theA.</summary>
    /// <exception cref="ConstructionErrorException">theA &lt;= 0.</exception>
    public HypParaboloidSurface WithSemiAxisA(double a)
    {
        HypParaboloidSurface copy = (HypParaboloidSurface)Copy();
        copy.SetSemiAxisA(a);
        return copy;
    }

    /// <summary>A copy with the semi-axis B set to theB.</summary>
    /// <exception cref="ConstructionErrorException">theB &lt;= 0.</exception>
    public HypParaboloidSurface WithSemiAxisB(double b)
    {
        HypParaboloidSurface copy = (HypParaboloidSurface)Copy();
        copy.SetSemiAxisB(b);
        return copy;
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::VReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double UReversedParameter(double u)
    {
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::UReversedParameter");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double VReversedParameter(double v)
    {
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::VReversedParameter");
    }

    /// <summary>U and V in [-Precision::Infinite(), Precision::Infinite()].</summary>
    internal override void ParameterBounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        u2 = global::Kerf.Engine.Foundation.Precision.Infinite();
        v1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        v2 = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    public override bool IsUClosed => false;

    public override bool IsVClosed => false;

    public override bool IsUPeriodic => false;

    public override bool IsVPeriodic => false;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D UIso(double u)
    {
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::UIso");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D VIso(double v)
    {
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::VIso");
    }

    internal override Point3D EvalD0(double u, double v)
    {
        Point3D o = _pos.Location;
        Direction3D xDir = _pos.XDirection;
        Direction3D yDir = _pos.YDirection;
        Direction3D zDir = _pos.Direction;

        double a2 = _a * _a;
        double b2 = _b * _b;
        double zComp = u * u / a2 - v * v / b2;

        return new Point3D(o.X + u * xDir.X + v * yDir.X + zComp * zDir.X,
                          o.Y + u * xDir.Y + v * yDir.Y + zComp * zDir.Y,
                          o.Z + u * xDir.Z + v * yDir.Z + zComp * zDir.Z);
    }

    internal override ResD1 EvalD1(double u, double v)
    {
        Point3D o = _pos.Location;
        Direction3D xDir = _pos.XDirection;
        Direction3D yDir = _pos.YDirection;
        Direction3D zDir = _pos.Direction;

        double a2 = _a * _a;
        double b2 = _b * _b;
        double zComp = u * u / a2 - v * v / b2;
        double zdU = 2.0 * u / a2;
        double zdV = -2.0 * v / b2;

        ResD1 result;

        // P = O + U*XD + V*YD + (U^2/A^2 - V^2/B^2)*ZD
        result.Point = new Point3D(o.X + u * xDir.X + v * yDir.X + zComp * zDir.X,
                                   o.Y + u * xDir.Y + v * yDir.Y + zComp * zDir.Y,
                                   o.Z + u * xDir.Z + v * yDir.Z + zComp * zDir.Z);

        // dP/du = XD + (2U/A^2)*ZD
        result.D1U = new Vector3D(xDir.X + zdU * zDir.X,
                                 xDir.Y + zdU * zDir.Y,
                                 xDir.Z + zdU * zDir.Z);

        // dP/dv = YD - (2V/B^2)*ZD
        result.D1V = new Vector3D(yDir.X + zdV * zDir.X,
                                 yDir.Y + zdV * zDir.Y,
                                 yDir.Z + zdV * zDir.Z);

        return result;
    }

    internal override ResD2 EvalD2(double u, double v)
    {
        Point3D o = _pos.Location;
        Direction3D xDir = _pos.XDirection;
        Direction3D yDir = _pos.YDirection;
        Direction3D zDir = _pos.Direction;

        double a2 = _a * _a;
        double b2 = _b * _b;
        double zComp = u * u / a2 - v * v / b2;
        double zdU = 2.0 * u / a2;
        double zdV = -2.0 * v / b2;
        double zd2U = 2.0 / a2;
        double zd2V = -2.0 / b2;

        ResD2 result;

        // P
        result.Point = new Point3D(o.X + u * xDir.X + v * yDir.X + zComp * zDir.X,
                                   o.Y + u * xDir.Y + v * yDir.Y + zComp * zDir.Y,
                                   o.Z + u * xDir.Z + v * yDir.Z + zComp * zDir.Z);

        // dP/du = XD + (2U/A^2)*ZD
        result.D1U = new Vector3D(xDir.X + zdU * zDir.X,
                                 xDir.Y + zdU * zDir.Y,
                                 xDir.Z + zdU * zDir.Z);

        // dP/dv = YD - (2V/B^2)*ZD
        result.D1V = new Vector3D(yDir.X + zdV * zDir.X,
                                 yDir.Y + zdV * zDir.Y,
                                 yDir.Z + zdV * zDir.Z);

        // d2P/du2 = (2/A^2)*ZD
        result.D2U = new Vector3D(zd2U * zDir.X, zd2U * zDir.Y, zd2U * zDir.Z);

        // d2P/dv2 = -(2/B^2)*ZD
        result.D2V = new Vector3D(zd2V * zDir.X, zd2V * zDir.Y, zd2V * zDir.Z);

        // d2P/dudv = 0
        result.D2UV = new Vector3D(0.0, 0.0, 0.0);

        return result;
    }

    internal override ResD3 EvalD3(double u, double v)
    {
        Point3D o = _pos.Location;
        Direction3D xDir = _pos.XDirection;
        Direction3D yDir = _pos.YDirection;
        Direction3D zDir = _pos.Direction;

        double a2 = _a * _a;
        double b2 = _b * _b;
        double zComp = u * u / a2 - v * v / b2;
        double zdU = 2.0 * u / a2;
        double zdV = -2.0 * v / b2;
        double zd2U = 2.0 / a2;
        double zd2V = -2.0 / b2;

        ResD3 result;

        // P
        result.Point = new Point3D(o.X + u * xDir.X + v * yDir.X + zComp * zDir.X,
                                   o.Y + u * xDir.Y + v * yDir.Y + zComp * zDir.Y,
                                   o.Z + u * xDir.Z + v * yDir.Z + zComp * zDir.Z);

        // dP/du
        result.D1U = new Vector3D(xDir.X + zdU * zDir.X,
                                 xDir.Y + zdU * zDir.Y,
                                 xDir.Z + zdU * zDir.Z);

        // dP/dv
        result.D1V = new Vector3D(yDir.X + zdV * zDir.X,
                                 yDir.Y + zdV * zDir.Y,
                                 yDir.Z + zdV * zDir.Z);

        // d2P/du2 = (2/A^2)*ZD
        result.D2U = new Vector3D(zd2U * zDir.X, zd2U * zDir.Y, zd2U * zDir.Z);

        // d2P/dv2 = -(2/B^2)*ZD
        result.D2V = new Vector3D(zd2V * zDir.X, zd2V * zDir.Y, zd2V * zDir.Z);

        // d2P/dudv = 0
        result.D2UV = new Vector3D(0.0, 0.0, 0.0);

        // All 3rd derivatives are zero (polynomial of degree 2)
        result.D3U = new Vector3D(0.0, 0.0, 0.0);
        result.D3V = new Vector3D(0.0, 0.0, 0.0);
        result.D3UUV = new Vector3D(0.0, 0.0, 0.0);
        result.D3UVV = new Vector3D(0.0, 0.0, 0.0);

        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    internal override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw new UndefinedDerivative3DException(
                "GeomEval_HypParaboloidSurface::EvalDN: invalid derivative order");
        }

        Direction3D xDir = _pos.XDirection;
        Direction3D yDir = _pos.YDirection;
        Direction3D zDir = _pos.Direction;

        double a2 = _a * _a;
        double b2 = _b * _b;

        // The parametrization is:
        //   P = O + u*XD + v*YD + (u^2/A^2 - v^2/B^2)*ZD
        //
        // XD coeff = (Nu==0 ? u : (Nu==1 ? 1 : 0)) * (Nv==0 ? 1 : 0)
        // YD coeff = (Nu==0 ? 1 : 0) * (Nv==0 ? v : (Nv==1 ? 1 : 0))
        // ZD coeff = (Nv==0 ? d^Nu[u^2/A^2] : 0) + (Nu==0 ? d^Nv[-v^2/B^2] : 0)
        double xCoeff = (nv == 0 ? (nu == 0 ? u : (nu == 1 ? 1.0 : 0.0)) : 0.0);
        double yCoeff = (nu == 0 ? (nv == 0 ? v : (nv == 1 ? 1.0 : 0.0)) : 0.0);

        // ZD coefficient: sum of pure-u and pure-v parts
        double zCoeff = 0.0;

        // d^Nu[u^2/A^2] contribution (only when Nv == 0)
        if (nv == 0)
        {
            if (nu == 0)
            {
                zCoeff += u * u / a2;
            }
            else if (nu == 1)
            {
                zCoeff += 2.0 * u / a2;
            }
            else if (nu == 2)
            {
                zCoeff += 2.0 / a2;
            }
            // else 0 for Nu >= 3
        }

        // d^Nv[-v^2/B^2] contribution (only when Nu == 0)
        if (nu == 0)
        {
            if (nv == 0)
            {
                zCoeff += -v * v / b2;
            }
            else if (nv == 1)
            {
                zCoeff += -2.0 * v / b2;
            }
            else if (nv == 2)
            {
                zCoeff += -2.0 / b2;
            }
            // else 0 for Nv >= 3
        }

        return new Vector3D(xCoeff * xDir.X + yCoeff * yDir.X + zCoeff * zDir.X,
                          xCoeff * xDir.Y + yCoeff * yDir.Y + zCoeff * zDir.Y,
                          xCoeff * xDir.Z + yCoeff * yDir.Z + zCoeff * zDir.Z);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::Transform");
    }

    public override Geometry3D Copy() => new HypParaboloidSurface(_pos, _a, _b);

    /// <summary>
    /// The coefficients of the implicit equation in the absolute Cartesian coordinate system:
    /// <c>A1*X^2 + A2*Y^2 + A3*Z^2 + 2*(B1*X*Y + B2*X*Z + B3*Y*Z) + 2*(C1*X + C2*Y + C3*Z) + D = 0</c>.
    /// </summary>
    public void Coefficients(out double a1, out double a2, out double a3,
                             out double b1, out double b2, out double b3,
                             out double c1, out double c2, out double c3, out double d)
    {
        // In the local coordinate system of the hyperbolic paraboloid:
        //   X_L^2/A^2 - Y_L^2/B^2 - Z_L = 0
        //
        // Transform to global coordinates using:
        //   (X_L, Y_L, Z_L)^T = T * (x, y, z, 1)^T
        // where T is the transformation matrix from pos.
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

        double aA2 = _a * _a;
        double aB2 = _b * _b;

        // Expanding (1/A^2)*(T1i*xi+T14)^2 - (1/B^2)*(T2i*xi+T24)^2 - (T3i*xi+T34) = 0
        // and collecting terms:
        a1 = t11 * t11 / aA2 - t21 * t21 / aB2;
        a2 = t12 * t12 / aA2 - t22 * t22 / aB2;
        a3 = t13 * t13 / aA2 - t23 * t23 / aB2;
        b1 = t11 * t12 / aA2 - t21 * t22 / aB2;
        b2 = t11 * t13 / aA2 - t21 * t23 / aB2;
        b3 = t12 * t13 / aA2 - t22 * t23 / aB2;
        c1 = t11 * t14 / aA2 - t21 * t24 / aB2 - 0.5 * t31;
        c2 = t12 * t14 / aA2 - t22 * t24 / aB2 - 0.5 * t32;
        c3 = t13 * t14 / aA2 - t23 * t24 / aB2 - 0.5 * t33;
        d = t14 * t14 / aA2 - t24 * t24 / aB2 - t34;
    }
}
