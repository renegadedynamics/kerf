// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_HyperboloidSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_HyperboloidSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Evaluation;

/// <summary>
/// A hyperboloid of revolution of one or two sheets, centred on the origin of its coordinate system.
/// <list type="bullet">
/// <item>One sheet: <c>P(u,v) = O + R1*cosh(v)*cos(u)*XDir + R1*cosh(v)*sin(u)*YDir + R2*sinh(v)*ZDir</c>,
/// implicit equation <c>X^2/R1^2 + Y^2/R1^2 - Z^2/R2^2 = 1</c>.</item>
/// <item>Two sheets (only one sheet is represented):
/// <c>P(u,v) = O + R2*sinh(v)*cos(u)*XDir + R2*sinh(v)*sin(u)*YDir + R1*cosh(v)*ZDir</c>,
/// implicit equation <c>X^2/R2^2 + Y^2/R2^2 - Z^2/R1^2 = -1</c>.</item>
/// </list>
/// u in [0, 2*Pi] (periodic, closed), v in (-inf, +inf). Reversal, transformation and iso curves are not
/// supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
internal sealed class HyperboloidSurface : ElementarySurface
{
    /// <summary>Sheet mode selector.</summary>
    public enum SheetMode
    {
        /// <summary>Hyperboloid of one sheet.</summary>
        OneSheet,

        /// <summary>Hyperboloid of two sheets (one sheet only).</summary>
        TwoSheets,
    }

    private double _r1;
    private double _r2;
    private SheetMode _mode;

    /// <summary>Creates a hyperboloid with the given local coordinate system, radii and sheet mode.</summary>
    /// <exception cref="ConstructionErrorException">theR1 &lt;= 0 or theR2 &lt;= 0.</exception>
    public HyperboloidSurface(in CoordinateSystem position, double r1, double r2,
                                       SheetMode mode = SheetMode.OneSheet)
    {
        _r1 = r1;
        _r2 = r2;
        _mode = mode;
        if (r1 <= 0.0 || r2 <= 0.0)
        {
            throw new ConstructionErrorException();
        }
        _pos = position;
    }

    /// <summary>The radius R1.</summary>
    public double R1() => _r1;

    /// <summary>The radius R2.</summary>
    public double R2() => _r2;

    /// <summary>The sheet mode.</summary>
    public SheetMode Mode() => _mode;

    /// <exception cref="ConstructionErrorException">theR1 &lt;= 0.</exception>
    internal void SetR1(double r1)
    {
        CheckMutable();
        if (r1 <= 0.0)
        {
            throw new ConstructionErrorException();
        }
        _r1 = r1;
    }

    /// <exception cref="ConstructionErrorException">theR2 &lt;= 0.</exception>
    internal void SetR2(double r2)
    {
        CheckMutable();
        if (r2 <= 0.0)
        {
            throw new ConstructionErrorException();
        }
        _r2 = r2;
    }

    internal void SetMode(SheetMode mode)
    {
        CheckMutable();
        _mode = mode;
    }

    /// <summary>A copy with the radius R1 set to theR1.</summary>
    /// <exception cref="ConstructionErrorException">theR1 &lt;= 0.</exception>
    public HyperboloidSurface WithR1(double r1)
    {
        HyperboloidSurface copy = (HyperboloidSurface)Copy();
        copy.SetR1(r1);
        return copy;
    }

    /// <summary>A copy with the radius R2 set to theR2.</summary>
    /// <exception cref="ConstructionErrorException">theR2 &lt;= 0.</exception>
    public HyperboloidSurface WithR2(double r2)
    {
        HyperboloidSurface copy = (HyperboloidSurface)Copy();
        copy.SetR2(r2);
        return copy;
    }

    /// <summary>A copy with the sheet mode set to theMode.</summary>
    public HyperboloidSurface WithMode(SheetMode mode)
    {
        HyperboloidSurface copy = (HyperboloidSurface)Copy();
        copy.SetMode(mode);
        return copy;
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_HyperboloidSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_HyperboloidSurface::VReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double UReversedParameter(double u)
    {
        throw new MethodNotImplementedException("GeomEval_HyperboloidSurface::UReversedParameter");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double VReversedParameter(double v)
    {
        throw new MethodNotImplementedException("GeomEval_HyperboloidSurface::VReversedParameter");
    }

    /// <summary>U in [0, 2*Pi], V in [-Precision::Infinite(), Precision::Infinite()].</summary>
    internal override void ParameterBounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = 0.0;
        u2 = 2.0 * M_PI;
        v1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        v2 = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    /// <summary>True: the hyperboloid is closed in U (period 2*Pi).</summary>
    public override bool IsUClosed => true;

    public override bool IsVClosed => false;

    /// <summary>True: the hyperboloid is periodic in U (period 2*Pi).</summary>
    public override bool IsUPeriodic => true;

    public override bool IsVPeriodic => false;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D UIso(double u)
    {
        throw new MethodNotImplementedException("GeomEval_HyperboloidSurface::UIso");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Curve3D VIso(double v)
    {
        throw new MethodNotImplementedException("GeomEval_HyperboloidSurface::VIso");
    }

    internal override Point3D EvalD0(double u, double v)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double cosH = Math.Cosh(v);
        double sinH = Math.Sinh(v);

        Xyz o = _pos.Location.XYZ();
        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        Xyz p;
        if (_mode == SheetMode.OneSheet)
        {
            // P = O + R1*cosh(v)*cos(u)*XD + R1*cosh(v)*sin(u)*YD + R2*sinh(v)*ZD
            p = o + xd * (_r1 * cosH * cosU) + yd * (_r1 * cosH * sinU) + zd * (_r2 * sinH);
        }
        else
        {
            // P = O + R2*sinh(v)*cos(u)*XD + R2*sinh(v)*sin(u)*YD + R1*cosh(v)*ZD
            p = o + xd * (_r2 * sinH * cosU) + yd * (_r2 * sinH * sinU) + zd * (_r1 * cosH);
        }
        return new Point3D(p);
    }

    internal override ResD1 EvalD1(double u, double v)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double cosH = Math.Cosh(v);
        double sinH = Math.Sinh(v);

        Xyz o = _pos.Location.XYZ();
        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        ResD1 result;
        if (_mode == SheetMode.OneSheet)
        {
            result.Point = new Point3D(o + xd * (_r1 * cosH * cosU) + yd * (_r1 * cosH * sinU)
                                       + zd * (_r2 * sinH));
            // dP/du = R1*cosh(v)*(-sin(u))*XD + R1*cosh(v)*cos(u)*YD
            result.D1U = new Vector3D(xd * (_r1 * cosH * (-sinU)) + yd * (_r1 * cosH * cosU));
            // dP/dv = R1*sinh(v)*cos(u)*XD + R1*sinh(v)*sin(u)*YD + R2*cosh(v)*ZD
            result.D1V =
                new Vector3D(xd * (_r1 * sinH * cosU) + yd * (_r1 * sinH * sinU) + zd * (_r2 * cosH));
        }
        else
        {
            result.Point = new Point3D(o + xd * (_r2 * sinH * cosU) + yd * (_r2 * sinH * sinU)
                                       + zd * (_r1 * cosH));
            // dP/du = R2*sinh(v)*(-sin(u))*XD + R2*sinh(v)*cos(u)*YD
            result.D1U = new Vector3D(xd * (_r2 * sinH * (-sinU)) + yd * (_r2 * sinH * cosU));
            // dP/dv = R2*cosh(v)*cos(u)*XD + R2*cosh(v)*sin(u)*YD + R1*sinh(v)*ZD
            result.D1V =
                new Vector3D(xd * (_r2 * cosH * cosU) + yd * (_r2 * cosH * sinU) + zd * (_r1 * sinH));
        }
        return result;
    }

    internal override ResD2 EvalD2(double u, double v)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double cosH = Math.Cosh(v);
        double sinH = Math.Sinh(v);

        Xyz o = _pos.Location.XYZ();
        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        ResD2 result;
        if (_mode == SheetMode.OneSheet)
        {
            result.Point = new Point3D(o + xd * (_r1 * cosH * cosU) + yd * (_r1 * cosH * sinU)
                                       + zd * (_r2 * sinH));
            result.D1U = new Vector3D(xd * (_r1 * cosH * (-sinU)) + yd * (_r1 * cosH * cosU));
            result.D1V =
                new Vector3D(xd * (_r1 * sinH * cosU) + yd * (_r1 * sinH * sinU) + zd * (_r2 * cosH));
            // d2P/du2 = R1*cosh(v)*(-cos(u))*XD + R1*cosh(v)*(-sin(u))*YD
            result.D2U = new Vector3D(xd * (_r1 * cosH * (-cosU)) + yd * (_r1 * cosH * (-sinU)));
            // d2P/dv2 = R1*cosh(v)*cos(u)*XD + R1*cosh(v)*sin(u)*YD + R2*sinh(v)*ZD
            result.D2V =
                new Vector3D(xd * (_r1 * cosH * cosU) + yd * (_r1 * cosH * sinU) + zd * (_r2 * sinH));
            // d2P/dudv = R1*sinh(v)*(-sin(u))*XD + R1*sinh(v)*cos(u)*YD
            result.D2UV = new Vector3D(xd * (_r1 * sinH * (-sinU)) + yd * (_r1 * sinH * cosU));
        }
        else
        {
            result.Point = new Point3D(o + xd * (_r2 * sinH * cosU) + yd * (_r2 * sinH * sinU)
                                       + zd * (_r1 * cosH));
            result.D1U = new Vector3D(xd * (_r2 * sinH * (-sinU)) + yd * (_r2 * sinH * cosU));
            result.D1V =
                new Vector3D(xd * (_r2 * cosH * cosU) + yd * (_r2 * cosH * sinU) + zd * (_r1 * sinH));
            // d2P/du2 = R2*sinh(v)*(-cos(u))*XD + R2*sinh(v)*(-sin(u))*YD
            result.D2U = new Vector3D(xd * (_r2 * sinH * (-cosU)) + yd * (_r2 * sinH * (-sinU)));
            // d2P/dv2 = R2*sinh(v)*cos(u)*XD + R2*sinh(v)*sin(u)*YD + R1*cosh(v)*ZD
            result.D2V =
                new Vector3D(xd * (_r2 * sinH * cosU) + yd * (_r2 * sinH * sinU) + zd * (_r1 * cosH));
            // d2P/dudv = R2*cosh(v)*(-sin(u))*XD + R2*cosh(v)*cos(u)*YD
            result.D2UV = new Vector3D(xd * (_r2 * cosH * (-sinU)) + yd * (_r2 * cosH * cosU));
        }
        return result;
    }

    internal override ResD3 EvalD3(double u, double v)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        double cosH = Math.Cosh(v);
        double sinH = Math.Sinh(v);

        Xyz o = _pos.Location.XYZ();
        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        ResD3 result;
        if (_mode == SheetMode.OneSheet)
        {
            result.Point = new Point3D(o + xd * (_r1 * cosH * cosU) + yd * (_r1 * cosH * sinU)
                                       + zd * (_r2 * sinH));
            result.D1U = new Vector3D(xd * (_r1 * cosH * (-sinU)) + yd * (_r1 * cosH * cosU));
            result.D1V =
                new Vector3D(xd * (_r1 * sinH * cosU) + yd * (_r1 * sinH * sinU) + zd * (_r2 * cosH));
            result.D2U = new Vector3D(xd * (_r1 * cosH * (-cosU)) + yd * (_r1 * cosH * (-sinU)));
            result.D2V =
                new Vector3D(xd * (_r1 * cosH * cosU) + yd * (_r1 * cosH * sinU) + zd * (_r2 * sinH));
            result.D2UV = new Vector3D(xd * (_r1 * sinH * (-sinU)) + yd * (_r1 * sinH * cosU));
            // d3P/du3 = R1*cosh(v)*sin(u)*XD + R1*cosh(v)*(-cos(u))*YD
            result.D3U = new Vector3D(xd * (_r1 * cosH * sinU) + yd * (_r1 * cosH * (-cosU)));
            // d3P/dv3 = R1*sinh(v)*cos(u)*XD + R1*sinh(v)*sin(u)*YD + R2*cosh(v)*ZD
            result.D3V =
                new Vector3D(xd * (_r1 * sinH * cosU) + yd * (_r1 * sinH * sinU) + zd * (_r2 * cosH));
            // d3P/du2dv = R1*sinh(v)*(-cos(u))*XD + R1*sinh(v)*(-sin(u))*YD
            result.D3UUV = new Vector3D(xd * (_r1 * sinH * (-cosU)) + yd * (_r1 * sinH * (-sinU)));
            // d3P/dudv2 = R1*cosh(v)*(-sin(u))*XD + R1*cosh(v)*cos(u)*YD
            result.D3UVV = new Vector3D(xd * (_r1 * cosH * (-sinU)) + yd * (_r1 * cosH * cosU));
        }
        else
        {
            result.Point = new Point3D(o + xd * (_r2 * sinH * cosU) + yd * (_r2 * sinH * sinU)
                                       + zd * (_r1 * cosH));
            result.D1U = new Vector3D(xd * (_r2 * sinH * (-sinU)) + yd * (_r2 * sinH * cosU));
            result.D1V =
                new Vector3D(xd * (_r2 * cosH * cosU) + yd * (_r2 * cosH * sinU) + zd * (_r1 * sinH));
            result.D2U = new Vector3D(xd * (_r2 * sinH * (-cosU)) + yd * (_r2 * sinH * (-sinU)));
            result.D2V =
                new Vector3D(xd * (_r2 * sinH * cosU) + yd * (_r2 * sinH * sinU) + zd * (_r1 * cosH));
            result.D2UV = new Vector3D(xd * (_r2 * cosH * (-sinU)) + yd * (_r2 * cosH * cosU));
            // d3P/du3 = R2*sinh(v)*sin(u)*XD + R2*sinh(v)*(-cos(u))*YD
            result.D3U = new Vector3D(xd * (_r2 * sinH * sinU) + yd * (_r2 * sinH * (-cosU)));
            // d3P/dv3 = R2*cosh(v)*cos(u)*XD + R2*cosh(v)*sin(u)*YD + R1*sinh(v)*ZD
            result.D3V =
                new Vector3D(xd * (_r2 * cosH * cosU) + yd * (_r2 * cosH * sinU) + zd * (_r1 * sinH));
            // d3P/du2dv = R2*cosh(v)*(-cos(u))*XD + R2*cosh(v)*(-sin(u))*YD
            result.D3UUV = new Vector3D(xd * (_r2 * cosH * (-cosU)) + yd * (_r2 * cosH * (-sinU)));
            // d3P/dudv2 = R2*sinh(v)*(-sin(u))*XD + R2*sinh(v)*cos(u)*YD
            result.D3UVV = new Vector3D(xd * (_r2 * sinH * (-sinU)) + yd * (_r2 * sinH * cosU));
        }
        return result;
    }

    /// <exception cref="UndefinedDerivative3DException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    internal override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw new UndefinedDerivative3DException();
        }

        Xyz xd = _pos.XDirection.XYZ();
        Xyz yd = _pos.YDirection.XYZ();
        Xyz zd = _pos.Direction.XYZ();

        // d^n/du^n[cos(u)] = cos(u + n*PI/2)
        // d^n/du^n[sin(u)] = sin(u + n*PI/2)
        double uShift = u + nu * M_PI_2;
        double dnCosU = Math.Cos(uShift);
        double dnSinU = Math.Sin(uShift);

        // d^n/dv^n[cosh(v)] = (n%2==0 ? cosh(v) : sinh(v))
        // d^n/dv^n[sinh(v)] = (n%2==0 ? sinh(v) : cosh(v))
        double cosH = Math.Cosh(v);
        double sinH = Math.Sinh(v);
        double dnCosH = (nv % 2 == 0) ? cosH : sinH;
        double dnSinH = (nv % 2 == 0) ? sinH : cosH;

        Xyz vec;
        if (_mode == SheetMode.OneSheet)
        {
            // XD coeff = R1 * d^Nv[cosh(v)] * d^Nu[cos(u)]
            // YD coeff = R1 * d^Nv[cosh(v)] * d^Nu[sin(u)]
            // ZD coeff = R2 * d^Nv[sinh(v)] * (Nu==0 ? 1 : 0)
            double xCoeff = _r1 * dnCosH * dnCosU;
            double yCoeff = _r1 * dnCosH * dnSinU;
            double zCoeff = (nu == 0) ? _r2 * dnSinH : 0.0;
            vec = xd * xCoeff + yd * yCoeff + zd * zCoeff;
        }
        else
        {
            // XD coeff = R2 * d^Nv[sinh(v)] * d^Nu[cos(u)]
            // YD coeff = R2 * d^Nv[sinh(v)] * d^Nu[sin(u)]
            // ZD coeff = R1 * d^Nv[cosh(v)] * (Nu==0 ? 1 : 0)
            double xCoeff = _r2 * dnSinH * dnCosU;
            double yCoeff = _r2 * dnSinH * dnSinU;
            double zCoeff = (nu == 0) ? _r1 * dnCosH : 0.0;
            vec = xd * xCoeff + yd * yCoeff + zd * zCoeff;
        }

        return new Vector3D(vec);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_HyperboloidSurface::Transform");
    }

    public override Geometry3D Copy() => new HyperboloidSurface(_pos, _r1, _r2, _mode);

    /// <summary>
    /// The coefficients of the implicit equation in the absolute Cartesian coordinate system:
    /// <c>A1*X^2 + A2*Y^2 + A3*Z^2 + 2*(B1*X*Y + B2*X*Z + B3*Y*Z) + 2*(C1*X + C2*Y + C3*Z) + D = 0</c>.
    /// </summary>
    public void Coefficients(out double a1, out double a2, out double a3,
                             out double b1, out double b2, out double b3,
                             out double c1, out double c2, out double c3, out double d)
    {
        // Local coordinate equation:
        // 1-sheet: X^2/R1^2 + Y^2/R1^2 - Z^2/R2^2 - 1 = 0
        //   => A1_loc=1/R1^2, A2_loc=1/R1^2, A3_loc=-1/R2^2, B_loc=0, C_loc=0, D_loc=-1
        // 2-sheet: X^2/R2^2 + Y^2/R2^2 - Z^2/R1^2 + 1 = 0
        //   => A1_loc=1/R2^2, A2_loc=1/R2^2, A3_loc=-1/R1^2, B_loc=0, C_loc=0, D_loc=+1
        double a1Loc, a2Loc, a3Loc, dLoc;
        if (_mode == SheetMode.OneSheet)
        {
            a1Loc = 1.0 / (_r1 * _r1);
            a2Loc = 1.0 / (_r1 * _r1);
            a3Loc = -1.0 / (_r2 * _r2);
            dLoc = -1.0;
        }
        else
        {
            a1Loc = 1.0 / (_r2 * _r2);
            a2Loc = 1.0 / (_r2 * _r2);
            a3Loc = -1.0 / (_r1 * _r1);
            dLoc = 1.0;
        }

        // Local equation: aA1Loc*X'^2 + aA2Loc*Y'^2 + aA3Loc*Z'^2 + aDLoc = 0
        // where (X',Y',Z') = T * (X,Y,Z), T = SetTransformation(pos).
        Transform3D trsf = new Transform3D().WithTransformation(_pos);
        double t11 = trsf.Value(1, 1);
        double t12 = trsf.Value(1, 2);
        double t13 = trsf.Value(1, 3);
        double t14 = trsf.Value(1, 4);
        double t21 = trsf.Value(2, 1);
        double t22 = trsf.Value(2, 2);
        double t23 = trsf.Value(2, 3);
        double t24 = trsf.Value(2, 4);
        double t31 = trsf.Value(3, 1);
        double t32 = trsf.Value(3, 2);
        double t33 = trsf.Value(3, 3);
        double t34 = trsf.Value(3, 4);

        // General quadric: sum over local axes of aCoeff[i] * (Ti1*X + Ti2*Y + Ti3*Z + Ti4)^2 + aDLoc
        a1 = a1Loc * t11 * t11 + a2Loc * t21 * t21 + a3Loc * t31 * t31;
        a2 = a1Loc * t12 * t12 + a2Loc * t22 * t22 + a3Loc * t32 * t32;
        a3 = a1Loc * t13 * t13 + a2Loc * t23 * t23 + a3Loc * t33 * t33;
        b1 = a1Loc * t11 * t12 + a2Loc * t21 * t22 + a3Loc * t31 * t32;
        b2 = a1Loc * t11 * t13 + a2Loc * t21 * t23 + a3Loc * t31 * t33;
        b3 = a1Loc * t12 * t13 + a2Loc * t22 * t23 + a3Loc * t32 * t33;
        c1 = a1Loc * t11 * t14 + a2Loc * t21 * t24 + a3Loc * t31 * t34;
        c2 = a1Loc * t12 * t14 + a2Loc * t22 * t24 + a3Loc * t32 * t34;
        c3 = a1Loc * t13 * t14 + a2Loc * t23 * t24 + a3Loc * t33 * t34;
        d = a1Loc * t14 * t14 + a2Loc * t24 * t24 + a3Loc * t34 * t34 + dLoc;
    }
}
