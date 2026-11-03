// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_ParaboloidSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_ParaboloidSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A circular paraboloid of revolution with focal distance F &gt; 0, its vertex at the origin of its coordinate
/// system: <c>P(u,v) = O + v*cos(u)*XDir + v*sin(u)*YDir + v^2/(4*F)*ZDir</c>, u in [0, 2*Pi] (periodic),
/// v in (-inf, +inf). Implicit equation in local coordinates: <c>X^2 + Y^2 - 4*F*Z = 0</c>.
/// Reversal, transformation and iso curves are not supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
public sealed class ParaboloidSurface : ElementarySurface
{
    private double _focal; // Focal distance (> 0)

    /// <summary>Creates a paraboloid with the given local coordinate system and focal distance.</summary>
    /// <exception cref="ConstructionErrorException">theFocal &lt;= 0.</exception>
    public ParaboloidSurface(in CoordinateSystem position, double focal)
    {
        _focal = focal;
        if (focal <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_ParaboloidSurface: focal distance must be > 0");
        }
        _pos = position;
    }

    /// <summary>The focal distance.</summary>
    public double Focal() => _focal;

    /// <exception cref="ConstructionErrorException">theFocal &lt;= 0.</exception>
    internal void SetFocal(double focal)
    {
        CheckMutable();
        if (focal <= 0.0)
        {
            throw new ConstructionErrorException(
                "GeomEval_ParaboloidSurface::SetFocal: focal distance must be > 0");
        }
        _focal = focal;
    }

    /// <summary>A copy with the focal distance set to theFocal.</summary>
    /// <exception cref="ConstructionErrorException">theFocal &lt;= 0.</exception>
    public ParaboloidSurface WithFocal(double focal)
    {
        ParaboloidSurface copy = (ParaboloidSurface)Copy();
        copy.SetFocal(focal);
        return copy;
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_ParaboloidSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_ParaboloidSurface::VReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double UReversedParameter(double u)
    {
        throw new MethodNotImplementedException("GeomEval_ParaboloidSurface::UReversedParameter");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double VReversedParameter(double v)
    {
        throw new MethodNotImplementedException("GeomEval_ParaboloidSurface::VReversedParameter");
    }

    /// <summary>U in [0, 2*Pi], V in [-Precision::Infinite(), Precision::Infinite()].</summary>
    public override void Bounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = 0.0;
        u2 = 2.0 * M_PI;
        v1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        v2 = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>True: the paraboloid is closed in U (period 2*Pi).</summary>
    public override bool IsUClosed => true;

    public override bool IsVClosed => false;

    /// <summary>True: the paraboloid is periodic in U (period 2*Pi).</summary>
    public override bool IsUPeriodic => true;

    public override bool IsVPeriodic => false;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D UIso(double u)
    {
        throw new MethodNotImplementedException("GeomEval_ParaboloidSurface::UIso");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D VIso(double v)
    {
        throw new MethodNotImplementedException("GeomEval_ParaboloidSurface::VIso");
    }

    public override Point3D EvalD0(double u, double v)
    {
        Point3D o = _pos.Location();
        Direction3D xDir = _pos.XDirection();
        Direction3D yDir = _pos.YDirection();
        Direction3D zDir = _pos.Direction();

        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double zComp = v * v / (4.0 * _focal);

        return new Point3D(o.X() + v * cosU * xDir.X() + v * sinU * yDir.X() + zComp * zDir.X(),
                          o.Y() + v * cosU * xDir.Y() + v * sinU * yDir.Y() + zComp * zDir.Y(),
                          o.Z() + v * cosU * xDir.Z() + v * sinU * yDir.Z() + zComp * zDir.Z());
    }

    public override ResD1 EvalD1(double u, double v)
    {
        Point3D o = _pos.Location();
        Direction3D xDir = _pos.XDirection();
        Direction3D yDir = _pos.YDirection();
        Direction3D zDir = _pos.Direction();

        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double zComp = v * v / (4.0 * _focal);
        double vOver2F = v / (2.0 * _focal);

        ResD1 result;

        // P = O + V*cosU*XD + V*sinU*YD + V^2/(4F)*ZD
        result.Point =
            new Point3D(o.X() + v * cosU * xDir.X() + v * sinU * yDir.X() + zComp * zDir.X(),
                       o.Y() + v * cosU * xDir.Y() + v * sinU * yDir.Y() + zComp * zDir.Y(),
                       o.Z() + v * cosU * xDir.Z() + v * sinU * yDir.Z() + zComp * zDir.Z());

        // dP/du = -V*sinU*XD + V*cosU*YD
        result.D1U = new Vector3D(-v * sinU * xDir.X() + v * cosU * yDir.X(),
                                 -v * sinU * xDir.Y() + v * cosU * yDir.Y(),
                                 -v * sinU * xDir.Z() + v * cosU * yDir.Z());

        // dP/dv = cosU*XD + sinU*YD + V/(2F)*ZD
        result.D1V = new Vector3D(cosU * xDir.X() + sinU * yDir.X() + vOver2F * zDir.X(),
                                 cosU * xDir.Y() + sinU * yDir.Y() + vOver2F * zDir.Y(),
                                 cosU * xDir.Z() + sinU * yDir.Z() + vOver2F * zDir.Z());

        return result;
    }

    public override ResD2 EvalD2(double u, double v)
    {
        Point3D o = _pos.Location();
        Direction3D xDir = _pos.XDirection();
        Direction3D yDir = _pos.YDirection();
        Direction3D zDir = _pos.Direction();

        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double zComp = v * v / (4.0 * _focal);
        double vOver2F = v / (2.0 * _focal);
        double inv2F = 1.0 / (2.0 * _focal);

        ResD2 result;

        // P
        result.Point =
            new Point3D(o.X() + v * cosU * xDir.X() + v * sinU * yDir.X() + zComp * zDir.X(),
                       o.Y() + v * cosU * xDir.Y() + v * sinU * yDir.Y() + zComp * zDir.Y(),
                       o.Z() + v * cosU * xDir.Z() + v * sinU * yDir.Z() + zComp * zDir.Z());

        // dP/du
        result.D1U = new Vector3D(-v * sinU * xDir.X() + v * cosU * yDir.X(),
                                 -v * sinU * xDir.Y() + v * cosU * yDir.Y(),
                                 -v * sinU * xDir.Z() + v * cosU * yDir.Z());

        // dP/dv
        result.D1V = new Vector3D(cosU * xDir.X() + sinU * yDir.X() + vOver2F * zDir.X(),
                                 cosU * xDir.Y() + sinU * yDir.Y() + vOver2F * zDir.Y(),
                                 cosU * xDir.Z() + sinU * yDir.Z() + vOver2F * zDir.Z());

        // d2P/du2 = -V*cosU*XD - V*sinU*YD
        result.D2U = new Vector3D(-v * cosU * xDir.X() - v * sinU * yDir.X(),
                                 -v * cosU * xDir.Y() - v * sinU * yDir.Y(),
                                 -v * cosU * xDir.Z() - v * sinU * yDir.Z());

        // d2P/dv2 = 1/(2F)*ZD
        result.D2V = new Vector3D(inv2F * zDir.X(), inv2F * zDir.Y(), inv2F * zDir.Z());

        // d2P/dudv = -sinU*XD + cosU*YD
        result.D2UV = new Vector3D(-sinU * xDir.X() + cosU * yDir.X(),
                                  -sinU * xDir.Y() + cosU * yDir.Y(),
                                  -sinU * xDir.Z() + cosU * yDir.Z());

        return result;
    }

    public override ResD3 EvalD3(double u, double v)
    {
        Point3D o = _pos.Location();
        Direction3D xDir = _pos.XDirection();
        Direction3D yDir = _pos.YDirection();
        Direction3D zDir = _pos.Direction();

        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double zComp = v * v / (4.0 * _focal);
        double vOver2F = v / (2.0 * _focal);
        double inv2F = 1.0 / (2.0 * _focal);

        ResD3 result;

        // P
        result.Point =
            new Point3D(o.X() + v * cosU * xDir.X() + v * sinU * yDir.X() + zComp * zDir.X(),
                       o.Y() + v * cosU * xDir.Y() + v * sinU * yDir.Y() + zComp * zDir.Y(),
                       o.Z() + v * cosU * xDir.Z() + v * sinU * yDir.Z() + zComp * zDir.Z());

        // dP/du
        result.D1U = new Vector3D(-v * sinU * xDir.X() + v * cosU * yDir.X(),
                                 -v * sinU * xDir.Y() + v * cosU * yDir.Y(),
                                 -v * sinU * xDir.Z() + v * cosU * yDir.Z());

        // dP/dv
        result.D1V = new Vector3D(cosU * xDir.X() + sinU * yDir.X() + vOver2F * zDir.X(),
                                 cosU * xDir.Y() + sinU * yDir.Y() + vOver2F * zDir.Y(),
                                 cosU * xDir.Z() + sinU * yDir.Z() + vOver2F * zDir.Z());

        // d2P/du2
        result.D2U = new Vector3D(-v * cosU * xDir.X() - v * sinU * yDir.X(),
                                 -v * cosU * xDir.Y() - v * sinU * yDir.Y(),
                                 -v * cosU * xDir.Z() - v * sinU * yDir.Z());

        // d2P/dv2
        result.D2V = new Vector3D(inv2F * zDir.X(), inv2F * zDir.Y(), inv2F * zDir.Z());

        // d2P/dudv
        result.D2UV = new Vector3D(-sinU * xDir.X() + cosU * yDir.X(),
                                  -sinU * xDir.Y() + cosU * yDir.Y(),
                                  -sinU * xDir.Z() + cosU * yDir.Z());

        // d3P/du3 = V*sinU*XD - V*cosU*YD
        result.D3U = new Vector3D(v * sinU * xDir.X() - v * cosU * yDir.X(),
                                 v * sinU * xDir.Y() - v * cosU * yDir.Y(),
                                 v * sinU * xDir.Z() - v * cosU * yDir.Z());

        // d3P/dv3 = 0
        result.D3V = new Vector3D(0.0, 0.0, 0.0);

        // d3P/du2dv = -cosU*XD - sinU*YD
        result.D3UUV = new Vector3D(-cosU * xDir.X() - sinU * yDir.X(),
                                   -cosU * xDir.Y() - sinU * yDir.Y(),
                                   -cosU * xDir.Z() - sinU * yDir.Z());

        // d3P/dudv2 = 0
        result.D3UVV = new Vector3D(0.0, 0.0, 0.0);

        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw new UndefinedDerivative3DException("GeomEval_ParaboloidSurface::EvalDN: invalid derivative order");
        }

        Direction3D xDir = _pos.XDirection();
        Direction3D yDir = _pos.YDirection();
        Direction3D zDir = _pos.Direction();

        // The parametrization is:
        //   P = O + v*cos(u)*XD + v*sin(u)*YD + v^2/(4F)*ZD
        //
        // XD component: cos(u + Nu*PI/2) * (Nv==0 ? v : (Nv==1 ? 1 : 0))
        // YD component: sin(u + Nu*PI/2) * (Nv==0 ? v : (Nv==1 ? 1 : 0))
        // ZD component: (Nu==0 ? 1 : 0) * d^Nv[v^2/(4F)]/dv^Nv
        //   where d^Nv[v^2/(4F)]/dv^Nv = (Nv==0 ? v^2/(4F) : (Nv==1 ? v/(2F) : (Nv==2 ? 1/(2F) : 0)))

        // XD and YD terms: derivative of v^1 with respect to v
        double vDerivXY = 0.0;
        if (nv == 0)
        {
            vDerivXY = v;
        }
        else if (nv == 1)
        {
            vDerivXY = 1.0;
        }
        // else aVDerivXY = 0 for Nv >= 2

        // Trig derivatives: d^Nu[cos(u)]/du^Nu = cos(u + Nu*PI/2)
        double phase = u + nu * M_PI_2;
        double cosNu = Math.Cos(phase);
        double sinNu = Math.Sin(phase);

        double xCoeff = cosNu * vDerivXY;
        double yCoeff = sinNu * vDerivXY;

        // ZD term: only present when Nu == 0
        double zCoeff = 0.0;
        if (nu == 0)
        {
            if (nv == 0)
            {
                zCoeff = v * v / (4.0 * _focal);
            }
            else if (nv == 1)
            {
                zCoeff = v / (2.0 * _focal);
            }
            else if (nv == 2)
            {
                zCoeff = 1.0 / (2.0 * _focal);
            }
            // else 0 for Nv >= 3
        }

        return new Vector3D(xCoeff * xDir.X() + yCoeff * yDir.X() + zCoeff * zDir.X(),
                          xCoeff * xDir.Y() + yCoeff * yDir.Y() + zCoeff * zDir.Y(),
                          xCoeff * xDir.Z() + yCoeff * yDir.Z() + zCoeff * zDir.Z());
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_ParaboloidSurface::Transform");
    }

    public override Geometry3D Copy() => new ParaboloidSurface(_pos, _focal);

    /// <summary>
    /// The coefficients of the implicit equation in the absolute Cartesian coordinate system:
    /// <c>A1*X^2 + A2*Y^2 + A3*Z^2 + 2*(B1*X*Y + B2*X*Z + B3*Y*Z) + 2*(C1*X + C2*Y + C3*Z) + D = 0</c>.
    /// </summary>
    public void Coefficients(out double a1, out double a2, out double a3,
                             out double b1, out double b2, out double b3,
                             out double c1, out double c2, out double c3, out double d)
    {
        // In the local coordinate system of the paraboloid:
        //   X_L^2 + Y_L^2 - 4*F*Z_L = 0
        //
        // Transform to global coordinates using:
        //   (X_L, Y_L, Z_L)^T = T * (x, y, z, 1)^T
        // where T is the transformation matrix from pos.
        Transform3D t = new Transform3D();
        t.SetTransformation(_pos);
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

        // Expanding (T11*x + T12*y + T13*z + T14)^2 + (T21*x + T22*y + T23*z + T24)^2
        //   - 4*F*(T31*x + T32*y + T33*z + T34) = 0
        // and collecting terms:
        a1 = t11 * t11 + t21 * t21;
        a2 = t12 * t12 + t22 * t22;
        a3 = t13 * t13 + t23 * t23;
        b1 = t11 * t12 + t21 * t22;
        b2 = t11 * t13 + t21 * t23;
        b3 = t12 * t13 + t22 * t23;
        c1 = t11 * t14 + t21 * t24 - 2.0 * _focal * t31;
        c2 = t12 * t14 + t22 * t24 - 2.0 * _focal * t32;
        c3 = t13 * t14 + t23 * t24 - 2.0 * _focal * t33;
        d = t14 * t14 + t24 * t24 - 4.0 * _focal * t34;
    }
}
