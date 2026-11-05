// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_EllipsoidSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_EllipsoidSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A triaxial ellipsoid with semi-axes A, B, C (all &gt; 0), centred on the origin of its coordinate system:
/// <c>P(u,v) = O + A*cos(v)*cos(u)*XDir + B*cos(v)*sin(u)*YDir + C*sin(v)*ZDir</c>, u in [0, 2*Pi] (periodic),
/// v in [-Pi/2, Pi/2]. Implicit equation in local coordinates: <c>X^2/A^2 + Y^2/B^2 + Z^2/C^2 - 1 = 0</c>.
/// Reversal, transformation and iso curves are not supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
internal sealed class EllipsoidSurface : ElementarySurface
{
    private double _a; // Semi-axis along XDir
    private double _b; // Semi-axis along YDir
    private double _c; // Semi-axis along ZDir

    /// <summary>Creates a triaxial ellipsoid with the given local coordinate system and semi-axes.</summary>
    /// <exception cref="ConstructionErrorException">A semi-axis is &lt;= 0.</exception>
    public EllipsoidSurface(in CoordinateSystem position, double a, double b, double c)
    {
        _a = a;
        _b = b;
        _c = c;
        if (a <= 0.0 || b <= 0.0 || c <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_EllipsoidSurface: semi-axis must be > 0");
        }
        _pos = position;
    }

    /// <summary>The semi-axis A (along XDir).</summary>
    public double SemiAxisA() => _a;

    /// <summary>The semi-axis B (along YDir).</summary>
    public double SemiAxisB() => _b;

    /// <summary>The semi-axis C (along ZDir).</summary>
    public double SemiAxisC() => _c;

    /// <exception cref="ConstructionErrorException">theA &lt;= 0.</exception>
    internal void SetSemiAxisA(double a)
    {
        CheckMutable();
        if (a <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_EllipsoidSurface::SetSemiAxisA: value must be > 0");
        }
        _a = a;
    }

    /// <exception cref="ConstructionErrorException">theB &lt;= 0.</exception>
    internal void SetSemiAxisB(double b)
    {
        CheckMutable();
        if (b <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_EllipsoidSurface::SetSemiAxisB: value must be > 0");
        }
        _b = b;
    }

    /// <exception cref="ConstructionErrorException">theC &lt;= 0.</exception>
    internal void SetSemiAxisC(double c)
    {
        CheckMutable();
        if (c <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_EllipsoidSurface::SetSemiAxisC: value must be > 0");
        }
        _c = c;
    }

    /// <summary>A copy with the semi-axis A set to theA.</summary>
    /// <exception cref="ConstructionErrorException">theA &lt;= 0.</exception>
    public EllipsoidSurface WithSemiAxisA(double a)
    {
        EllipsoidSurface copy = (EllipsoidSurface)Copy();
        copy.SetSemiAxisA(a);
        return copy;
    }

    /// <summary>A copy with the semi-axis B set to theB.</summary>
    /// <exception cref="ConstructionErrorException">theB &lt;= 0.</exception>
    public EllipsoidSurface WithSemiAxisB(double b)
    {
        EllipsoidSurface copy = (EllipsoidSurface)Copy();
        copy.SetSemiAxisB(b);
        return copy;
    }

    /// <summary>A copy with the semi-axis C set to theC.</summary>
    /// <exception cref="ConstructionErrorException">theC &lt;= 0.</exception>
    public EllipsoidSurface WithSemiAxisC(double c)
    {
        EllipsoidSurface copy = (EllipsoidSurface)Copy();
        copy.SetSemiAxisC(c);
        return copy;
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_EllipsoidSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_EllipsoidSurface::VReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double UReversedParameter(double u)
    {
        throw new MethodNotImplementedException("GeomEval_EllipsoidSurface::UReversedParameter");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double VReversedParameter(double v)
    {
        throw new MethodNotImplementedException("GeomEval_EllipsoidSurface::VReversedParameter");
    }

    /// <summary>U in [0, 2*Pi], V in [-Pi/2, Pi/2].</summary>
    internal override void ParameterBounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = 0.0;
        u2 = 2.0 * M_PI;
        v1 = -M_PI / 2.0;
        v2 = M_PI / 2.0;
    }

    /// <summary>True: the ellipsoid is closed in U (period 2*Pi).</summary>
    public override bool IsUClosed => true;

    public override bool IsVClosed => false;

    /// <summary>True: the ellipsoid is periodic in U (period 2*Pi).</summary>
    public override bool IsUPeriodic => true;

    public override bool IsVPeriodic => false;

    /// <summary>Not supported (not a standard curve type): throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D UIso(double u)
    {
        throw new MethodNotImplementedException("GeomEval_EllipsoidSurface::UIso");
    }

    /// <summary>Not supported (an ellipse only when A == B): throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D VIso(double v)
    {
        throw new MethodNotImplementedException("GeomEval_EllipsoidSurface::VIso");
    }

    internal override Point3D EvalD0(double u, double v)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        Xyz o = _pos.Location.XYZ();
        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        return new Point3D(o + _a * cosV * cosU * xd + _b * cosV * sinU * yd + _c * sinV * zd);
    }

    internal override ResD1 EvalD1(double u, double v)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        Xyz o = _pos.Location.XYZ();
        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        ResD1 result;
        result.Point =
            new Point3D(o + _a * cosV * cosU * xd + _b * cosV * sinU * yd + _c * sinV * zd);

        // dP/du = -A*cv*su*XD + B*cv*cu*YD
        result.D1U = new Vector3D(_a * cosV * (-sinU) * xd + _b * cosV * cosU * yd);

        // dP/dv = -A*sv*cu*XD - B*sv*su*YD + C*cv*ZD
        result.D1V =
            new Vector3D(_a * (-sinV) * cosU * xd + _b * (-sinV) * sinU * yd + _c * cosV * zd);

        return result;
    }

    internal override ResD2 EvalD2(double u, double v)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        Xyz o = _pos.Location.XYZ();
        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        ResD2 result;
        result.Point =
            new Point3D(o + _a * cosV * cosU * xd + _b * cosV * sinU * yd + _c * sinV * zd);

        // dP/du
        result.D1U = new Vector3D(_a * cosV * (-sinU) * xd + _b * cosV * cosU * yd);

        // dP/dv
        result.D1V =
            new Vector3D(_a * (-sinV) * cosU * xd + _b * (-sinV) * sinU * yd + _c * cosV * zd);

        // d2P/du2 = -A*cv*cu*XD - B*cv*su*YD
        result.D2U = new Vector3D(_a * cosV * (-cosU) * xd + _b * cosV * (-sinU) * yd);

        // d2P/dv2 = -A*cv*cu*XD - B*cv*su*YD - C*sv*ZD
        result.D2V =
            new Vector3D(_a * (-cosV) * cosU * xd + _b * (-cosV) * sinU * yd + _c * (-sinV) * zd);

        // d2P/dudv = A*sv*su*XD - B*sv*cu*YD
        result.D2UV = new Vector3D(_a * sinV * sinU * xd + _b * (-sinV) * cosU * yd);

        return result;
    }

    internal override ResD3 EvalD3(double u, double v)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        Xyz o = _pos.Location.XYZ();
        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        ResD3 result;

        // P
        result.Point =
            new Point3D(o + _a * cosV * cosU * xd + _b * cosV * sinU * yd + _c * sinV * zd);

        // dP/du
        result.D1U = new Vector3D(_a * cosV * (-sinU) * xd + _b * cosV * cosU * yd);

        // dP/dv
        result.D1V =
            new Vector3D(_a * (-sinV) * cosU * xd + _b * (-sinV) * sinU * yd + _c * cosV * zd);

        // d2P/du2
        result.D2U = new Vector3D(_a * cosV * (-cosU) * xd + _b * cosV * (-sinU) * yd);

        // d2P/dv2
        result.D2V =
            new Vector3D(_a * (-cosV) * cosU * xd + _b * (-cosV) * sinU * yd + _c * (-sinV) * zd);

        // d2P/dudv
        result.D2UV = new Vector3D(_a * sinV * sinU * xd + _b * (-sinV) * cosU * yd);

        // d3P/du3 = A*cv*su*XD - B*cv*cu*YD
        result.D3U = new Vector3D(_a * cosV * sinU * xd + _b * cosV * (-cosU) * yd);

        // d3P/dv3 = A*sv*cu*XD + B*sv*su*YD - C*cv*ZD
        result.D3V =
            new Vector3D(_a * sinV * cosU * xd + _b * sinV * sinU * yd + _c * (-cosV) * zd);

        // d3P/du2dv = A*sv*cu*XD + B*sv*su*YD
        result.D3UUV = new Vector3D(_a * sinV * cosU * xd + _b * sinV * sinU * yd);

        // d3P/dudv2 = A*cv*su*XD - B*cv*cu*YD
        result.D3UVV = new Vector3D(_a * cosV * sinU * xd + _b * (-cosV) * cosU * yd);

        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    internal override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw new UndefinedDerivative3DException("GeomEval_EllipsoidSurface::EvalDN: invalid derivative order");
        }

        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        // Using phase-shift identities:
        // d^n/du^n[cos(u)] = cos(u + n*Pi/2)
        // d^n/du^n[sin(u)] = sin(u + n*Pi/2)
        double phaseU = nu * M_PI / 2.0;
        double phaseV = nv * M_PI / 2.0;

        // XD coefficient: A * d^Nv[cos(v)] * d^Nu[cos(u)]
        double coeffX = _a * Math.Cos(v + phaseV) * Math.Cos(u + phaseU);

        // YD coefficient: B * d^Nv[cos(v)] * d^Nu[sin(u)]
        double coeffY = _b * Math.Cos(v + phaseV) * Math.Sin(u + phaseU);

        // ZD coefficient: C * d^Nv[sin(v)] * (Nu == 0 ? 1 : 0)
        double coeffZ = _c * Math.Sin(v + phaseV) * (nu == 0 ? 1.0 : 0.0);

        return new Vector3D(coeffX * xd + coeffY * yd + coeffZ * zd);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_EllipsoidSurface::Transform");
    }

    public override Geometry3D Copy() => new EllipsoidSurface(_pos, _a, _b, _c);

    /// <summary>
    /// The coefficients of the implicit equation in the absolute Cartesian coordinate system:
    /// <c>A1*X^2 + A2*Y^2 + A3*Z^2 + 2*(B1*X*Y + B2*X*Z + B3*Y*Z) + 2*(C1*X + C2*Y + C3*Z) + D = 0</c>.
    /// </summary>
    public void Coefficients(out double a1, out double a2, out double a3,
                             out double b1, out double b2, out double b3,
                             out double c1, out double c2, out double c3, out double d)
    {
        // In the local coordinate system of the ellipsoid:
        // X^2/myA^2 + Y^2/myB^2 + Z^2/myC^2 - 1 = 0
        //
        // i.e. local diagonal coefficients are:
        //   a1 = 1/A^2, a2 = 1/B^2, a3 = 1/C^2, d = -1
        // and all cross/linear terms are zero.
        //
        // We transform from local to global using gp_Trsf from pos,
        // substituting X_local = T11*X + T12*Y + T13*Z + T14, etc.

        double invA2 = 1.0 / (_a * _a);
        double invB2 = 1.0 / (_b * _b);
        double invC2 = 1.0 / (_c * _c);

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

        // A1 = a1*T11^2 + a2*T21^2 + a3*T31^2
        a1 = invA2 * t11 * t11 + invB2 * t21 * t21 + invC2 * t31 * t31;

        // A2 = a1*T12^2 + a2*T22^2 + a3*T32^2
        a2 = invA2 * t12 * t12 + invB2 * t22 * t22 + invC2 * t32 * t32;

        // A3 = a1*T13^2 + a2*T23^2 + a3*T33^2
        a3 = invA2 * t13 * t13 + invB2 * t23 * t23 + invC2 * t33 * t33;

        // B1 = a1*T11*T12 + a2*T21*T22 + a3*T31*T32
        b1 = invA2 * t11 * t12 + invB2 * t21 * t22 + invC2 * t31 * t32;

        // B2 = a1*T11*T13 + a2*T21*T23 + a3*T31*T33
        b2 = invA2 * t11 * t13 + invB2 * t21 * t23 + invC2 * t31 * t33;

        // B3 = a1*T12*T13 + a2*T22*T23 + a3*T32*T33
        b3 = invA2 * t12 * t13 + invB2 * t22 * t23 + invC2 * t32 * t33;

        // C1 = a1*T11*T14 + a2*T21*T24 + a3*T31*T34
        c1 = invA2 * t11 * t14 + invB2 * t21 * t24 + invC2 * t31 * t34;

        // C2 = a1*T12*T14 + a2*T22*T24 + a3*T32*T34
        c2 = invA2 * t12 * t14 + invB2 * t22 * t24 + invC2 * t32 * t34;

        // C3 = a1*T13*T14 + a2*T23*T24 + a3*T33*T34
        c3 = invA2 * t13 * t14 + invB2 * t23 * t24 + invC2 * t33 * t34;

        // D = a1*T14^2 + a2*T24^2 + a3*T34^2 - 1
        d = invA2 * t14 * t14 + invB2 * t24 * t24 + invC2 * t34 * t34 - 1.0;
    }
}
