// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_HyperboloidSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_HyperboloidSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
/// supported (they throw <see cref="Standard_NotImplemented"/>).
/// </summary>
public sealed class GeomEval_HyperboloidSurface : Geom_ElementarySurface
{
    /// <summary>Sheet mode selector.</summary>
    public enum SheetMode
    {
        /// <summary>Hyperboloid of one sheet.</summary>
        OneSheet,

        /// <summary>Hyperboloid of two sheets (one sheet only).</summary>
        TwoSheets,
    }

    private double myR1;
    private double myR2;
    private SheetMode myMode;

    /// <summary>Creates a hyperboloid with the given local coordinate system, radii and sheet mode.</summary>
    /// <exception cref="Standard_ConstructionError">theR1 &lt;= 0 or theR2 &lt;= 0.</exception>
    public GeomEval_HyperboloidSurface(in gp_Ax3 thePosition, double theR1, double theR2,
                                       SheetMode theMode = SheetMode.OneSheet)
    {
        myR1 = theR1;
        myR2 = theR2;
        myMode = theMode;
        if (theR1 <= 0.0 || theR2 <= 0.0)
        {
            throw new Standard_ConstructionError();
        }
        pos = thePosition;
    }

    /// <summary>The radius R1.</summary>
    public double R1() => myR1;

    /// <summary>The radius R2.</summary>
    public double R2() => myR2;

    /// <summary>The sheet mode.</summary>
    public SheetMode Mode() => myMode;

    /// <exception cref="Standard_ConstructionError">theR1 &lt;= 0.</exception>
    internal void SetR1(double theR1)
    {
        CheckMutable();
        if (theR1 <= 0.0)
        {
            throw new Standard_ConstructionError();
        }
        myR1 = theR1;
    }

    /// <exception cref="Standard_ConstructionError">theR2 &lt;= 0.</exception>
    internal void SetR2(double theR2)
    {
        CheckMutable();
        if (theR2 <= 0.0)
        {
            throw new Standard_ConstructionError();
        }
        myR2 = theR2;
    }

    internal void SetMode(SheetMode theMode)
    {
        CheckMutable();
        myMode = theMode;
    }

    /// <summary>A copy with the radius R1 set to theR1.</summary>
    /// <exception cref="Standard_ConstructionError">theR1 &lt;= 0.</exception>
    public GeomEval_HyperboloidSurface WithR1(double theR1)
    {
        GeomEval_HyperboloidSurface aCopy = (GeomEval_HyperboloidSurface)Copy();
        aCopy.SetR1(theR1);
        return aCopy;
    }

    /// <summary>A copy with the radius R2 set to theR2.</summary>
    /// <exception cref="Standard_ConstructionError">theR2 &lt;= 0.</exception>
    public GeomEval_HyperboloidSurface WithR2(double theR2)
    {
        GeomEval_HyperboloidSurface aCopy = (GeomEval_HyperboloidSurface)Copy();
        aCopy.SetR2(theR2);
        return aCopy;
    }

    /// <summary>A copy with the sheet mode set to theMode.</summary>
    public GeomEval_HyperboloidSurface WithMode(SheetMode theMode)
    {
        GeomEval_HyperboloidSurface aCopy = (GeomEval_HyperboloidSurface)Copy();
        aCopy.SetMode(theMode);
        return aCopy;
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_HyperboloidSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_HyperboloidSurface::VReverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double UReversedParameter(double U)
    {
        throw new Standard_NotImplemented("GeomEval_HyperboloidSurface::UReversedParameter");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double VReversedParameter(double V)
    {
        throw new Standard_NotImplemented("GeomEval_HyperboloidSurface::VReversedParameter");
    }

    /// <summary>U in [0, 2*Pi], V in [-Precision::Infinite(), Precision::Infinite()].</summary>
    public override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = 0.0;
        U2 = 2.0 * M_PI;
        V1 = -Precision.Infinite();
        V2 = Precision.Infinite();
    }

    /// <summary>True: the hyperboloid is closed in U (period 2*Pi).</summary>
    public override bool IsUClosed() => true;

    public override bool IsVClosed() => false;

    /// <summary>True: the hyperboloid is periodic in U (period 2*Pi).</summary>
    public override bool IsUPeriodic() => true;

    public override bool IsVPeriodic() => false;

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override Geom_Curve UIso(double U)
    {
        throw new Standard_NotImplemented("GeomEval_HyperboloidSurface::UIso");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override Geom_Curve VIso(double V)
    {
        throw new Standard_NotImplemented("GeomEval_HyperboloidSurface::VIso");
    }

    public override gp_Pnt EvalD0(double U, double V)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        double aCosH = Math.Cosh(V);
        double aSinH = Math.Sinh(V);

        gp_XYZ anO = pos.Location().XYZ();
        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        gp_XYZ aP;
        if (myMode == SheetMode.OneSheet)
        {
            // P = O + R1*cosh(v)*cos(u)*XD + R1*cosh(v)*sin(u)*YD + R2*sinh(v)*ZD
            aP = anO + aXD * (myR1 * aCosH * aCosU) + aYD * (myR1 * aCosH * aSinU) + aZD * (myR2 * aSinH);
        }
        else
        {
            // P = O + R2*sinh(v)*cos(u)*XD + R2*sinh(v)*sin(u)*YD + R1*cosh(v)*ZD
            aP = anO + aXD * (myR2 * aSinH * aCosU) + aYD * (myR2 * aSinH * aSinU) + aZD * (myR1 * aCosH);
        }
        return new gp_Pnt(aP);
    }

    public override ResD1 EvalD1(double U, double V)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        double aCosH = Math.Cosh(V);
        double aSinH = Math.Sinh(V);

        gp_XYZ anO = pos.Location().XYZ();
        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        ResD1 aResult;
        if (myMode == SheetMode.OneSheet)
        {
            aResult.Point = new gp_Pnt(anO + aXD * (myR1 * aCosH * aCosU) + aYD * (myR1 * aCosH * aSinU)
                                       + aZD * (myR2 * aSinH));
            // dP/du = R1*cosh(v)*(-sin(u))*XD + R1*cosh(v)*cos(u)*YD
            aResult.D1U = new gp_Vec(aXD * (myR1 * aCosH * (-aSinU)) + aYD * (myR1 * aCosH * aCosU));
            // dP/dv = R1*sinh(v)*cos(u)*XD + R1*sinh(v)*sin(u)*YD + R2*cosh(v)*ZD
            aResult.D1V =
                new gp_Vec(aXD * (myR1 * aSinH * aCosU) + aYD * (myR1 * aSinH * aSinU) + aZD * (myR2 * aCosH));
        }
        else
        {
            aResult.Point = new gp_Pnt(anO + aXD * (myR2 * aSinH * aCosU) + aYD * (myR2 * aSinH * aSinU)
                                       + aZD * (myR1 * aCosH));
            // dP/du = R2*sinh(v)*(-sin(u))*XD + R2*sinh(v)*cos(u)*YD
            aResult.D1U = new gp_Vec(aXD * (myR2 * aSinH * (-aSinU)) + aYD * (myR2 * aSinH * aCosU));
            // dP/dv = R2*cosh(v)*cos(u)*XD + R2*cosh(v)*sin(u)*YD + R1*sinh(v)*ZD
            aResult.D1V =
                new gp_Vec(aXD * (myR2 * aCosH * aCosU) + aYD * (myR2 * aCosH * aSinU) + aZD * (myR1 * aSinH));
        }
        return aResult;
    }

    public override ResD2 EvalD2(double U, double V)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        double aCosH = Math.Cosh(V);
        double aSinH = Math.Sinh(V);

        gp_XYZ anO = pos.Location().XYZ();
        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        ResD2 aResult;
        if (myMode == SheetMode.OneSheet)
        {
            aResult.Point = new gp_Pnt(anO + aXD * (myR1 * aCosH * aCosU) + aYD * (myR1 * aCosH * aSinU)
                                       + aZD * (myR2 * aSinH));
            aResult.D1U = new gp_Vec(aXD * (myR1 * aCosH * (-aSinU)) + aYD * (myR1 * aCosH * aCosU));
            aResult.D1V =
                new gp_Vec(aXD * (myR1 * aSinH * aCosU) + aYD * (myR1 * aSinH * aSinU) + aZD * (myR2 * aCosH));
            // d2P/du2 = R1*cosh(v)*(-cos(u))*XD + R1*cosh(v)*(-sin(u))*YD
            aResult.D2U = new gp_Vec(aXD * (myR1 * aCosH * (-aCosU)) + aYD * (myR1 * aCosH * (-aSinU)));
            // d2P/dv2 = R1*cosh(v)*cos(u)*XD + R1*cosh(v)*sin(u)*YD + R2*sinh(v)*ZD
            aResult.D2V =
                new gp_Vec(aXD * (myR1 * aCosH * aCosU) + aYD * (myR1 * aCosH * aSinU) + aZD * (myR2 * aSinH));
            // d2P/dudv = R1*sinh(v)*(-sin(u))*XD + R1*sinh(v)*cos(u)*YD
            aResult.D2UV = new gp_Vec(aXD * (myR1 * aSinH * (-aSinU)) + aYD * (myR1 * aSinH * aCosU));
        }
        else
        {
            aResult.Point = new gp_Pnt(anO + aXD * (myR2 * aSinH * aCosU) + aYD * (myR2 * aSinH * aSinU)
                                       + aZD * (myR1 * aCosH));
            aResult.D1U = new gp_Vec(aXD * (myR2 * aSinH * (-aSinU)) + aYD * (myR2 * aSinH * aCosU));
            aResult.D1V =
                new gp_Vec(aXD * (myR2 * aCosH * aCosU) + aYD * (myR2 * aCosH * aSinU) + aZD * (myR1 * aSinH));
            // d2P/du2 = R2*sinh(v)*(-cos(u))*XD + R2*sinh(v)*(-sin(u))*YD
            aResult.D2U = new gp_Vec(aXD * (myR2 * aSinH * (-aCosU)) + aYD * (myR2 * aSinH * (-aSinU)));
            // d2P/dv2 = R2*sinh(v)*cos(u)*XD + R2*sinh(v)*sin(u)*YD + R1*cosh(v)*ZD
            aResult.D2V =
                new gp_Vec(aXD * (myR2 * aSinH * aCosU) + aYD * (myR2 * aSinH * aSinU) + aZD * (myR1 * aCosH));
            // d2P/dudv = R2*cosh(v)*(-sin(u))*XD + R2*cosh(v)*cos(u)*YD
            aResult.D2UV = new gp_Vec(aXD * (myR2 * aCosH * (-aSinU)) + aYD * (myR2 * aCosH * aCosU));
        }
        return aResult;
    }

    public override ResD3 EvalD3(double U, double V)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        double aCosH = Math.Cosh(V);
        double aSinH = Math.Sinh(V);

        gp_XYZ anO = pos.Location().XYZ();
        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        ResD3 aResult;
        if (myMode == SheetMode.OneSheet)
        {
            aResult.Point = new gp_Pnt(anO + aXD * (myR1 * aCosH * aCosU) + aYD * (myR1 * aCosH * aSinU)
                                       + aZD * (myR2 * aSinH));
            aResult.D1U = new gp_Vec(aXD * (myR1 * aCosH * (-aSinU)) + aYD * (myR1 * aCosH * aCosU));
            aResult.D1V =
                new gp_Vec(aXD * (myR1 * aSinH * aCosU) + aYD * (myR1 * aSinH * aSinU) + aZD * (myR2 * aCosH));
            aResult.D2U = new gp_Vec(aXD * (myR1 * aCosH * (-aCosU)) + aYD * (myR1 * aCosH * (-aSinU)));
            aResult.D2V =
                new gp_Vec(aXD * (myR1 * aCosH * aCosU) + aYD * (myR1 * aCosH * aSinU) + aZD * (myR2 * aSinH));
            aResult.D2UV = new gp_Vec(aXD * (myR1 * aSinH * (-aSinU)) + aYD * (myR1 * aSinH * aCosU));
            // d3P/du3 = R1*cosh(v)*sin(u)*XD + R1*cosh(v)*(-cos(u))*YD
            aResult.D3U = new gp_Vec(aXD * (myR1 * aCosH * aSinU) + aYD * (myR1 * aCosH * (-aCosU)));
            // d3P/dv3 = R1*sinh(v)*cos(u)*XD + R1*sinh(v)*sin(u)*YD + R2*cosh(v)*ZD
            aResult.D3V =
                new gp_Vec(aXD * (myR1 * aSinH * aCosU) + aYD * (myR1 * aSinH * aSinU) + aZD * (myR2 * aCosH));
            // d3P/du2dv = R1*sinh(v)*(-cos(u))*XD + R1*sinh(v)*(-sin(u))*YD
            aResult.D3UUV = new gp_Vec(aXD * (myR1 * aSinH * (-aCosU)) + aYD * (myR1 * aSinH * (-aSinU)));
            // d3P/dudv2 = R1*cosh(v)*(-sin(u))*XD + R1*cosh(v)*cos(u)*YD
            aResult.D3UVV = new gp_Vec(aXD * (myR1 * aCosH * (-aSinU)) + aYD * (myR1 * aCosH * aCosU));
        }
        else
        {
            aResult.Point = new gp_Pnt(anO + aXD * (myR2 * aSinH * aCosU) + aYD * (myR2 * aSinH * aSinU)
                                       + aZD * (myR1 * aCosH));
            aResult.D1U = new gp_Vec(aXD * (myR2 * aSinH * (-aSinU)) + aYD * (myR2 * aSinH * aCosU));
            aResult.D1V =
                new gp_Vec(aXD * (myR2 * aCosH * aCosU) + aYD * (myR2 * aCosH * aSinU) + aZD * (myR1 * aSinH));
            aResult.D2U = new gp_Vec(aXD * (myR2 * aSinH * (-aCosU)) + aYD * (myR2 * aSinH * (-aSinU)));
            aResult.D2V =
                new gp_Vec(aXD * (myR2 * aSinH * aCosU) + aYD * (myR2 * aSinH * aSinU) + aZD * (myR1 * aCosH));
            aResult.D2UV = new gp_Vec(aXD * (myR2 * aCosH * (-aSinU)) + aYD * (myR2 * aCosH * aCosU));
            // d3P/du3 = R2*sinh(v)*sin(u)*XD + R2*sinh(v)*(-cos(u))*YD
            aResult.D3U = new gp_Vec(aXD * (myR2 * aSinH * aSinU) + aYD * (myR2 * aSinH * (-aCosU)));
            // d3P/dv3 = R2*cosh(v)*cos(u)*XD + R2*cosh(v)*sin(u)*YD + R1*sinh(v)*ZD
            aResult.D3V =
                new gp_Vec(aXD * (myR2 * aCosH * aCosU) + aYD * (myR2 * aCosH * aSinU) + aZD * (myR1 * aSinH));
            // d3P/du2dv = R2*cosh(v)*(-cos(u))*XD + R2*cosh(v)*(-sin(u))*YD
            aResult.D3UUV = new gp_Vec(aXD * (myR2 * aCosH * (-aCosU)) + aYD * (myR2 * aCosH * (-aSinU)));
            // d3P/dudv2 = R2*sinh(v)*(-sin(u))*XD + R2*sinh(v)*cos(u)*YD
            aResult.D3UVV = new gp_Vec(aXD * (myR2 * aSinH * (-aSinU)) + aYD * (myR2 * aSinH * aCosU));
        }
        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public override gp_Vec EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Geom_UndefinedDerivative();
        }

        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        // d^n/du^n[cos(u)] = cos(u + n*PI/2)
        // d^n/du^n[sin(u)] = sin(u + n*PI/2)
        double aUShift = U + Nu * M_PI_2;
        double aDnCosU = Math.Cos(aUShift);
        double aDnSinU = Math.Sin(aUShift);

        // d^n/dv^n[cosh(v)] = (n%2==0 ? cosh(v) : sinh(v))
        // d^n/dv^n[sinh(v)] = (n%2==0 ? sinh(v) : cosh(v))
        double aCosH = Math.Cosh(V);
        double aSinH = Math.Sinh(V);
        double aDnCosH = (Nv % 2 == 0) ? aCosH : aSinH;
        double aDnSinH = (Nv % 2 == 0) ? aSinH : aCosH;

        gp_XYZ aVec;
        if (myMode == SheetMode.OneSheet)
        {
            // XD coeff = R1 * d^Nv[cosh(v)] * d^Nu[cos(u)]
            // YD coeff = R1 * d^Nv[cosh(v)] * d^Nu[sin(u)]
            // ZD coeff = R2 * d^Nv[sinh(v)] * (Nu==0 ? 1 : 0)
            double aXCoeff = myR1 * aDnCosH * aDnCosU;
            double aYCoeff = myR1 * aDnCosH * aDnSinU;
            double aZCoeff = (Nu == 0) ? myR2 * aDnSinH : 0.0;
            aVec = aXD * aXCoeff + aYD * aYCoeff + aZD * aZCoeff;
        }
        else
        {
            // XD coeff = R2 * d^Nv[sinh(v)] * d^Nu[cos(u)]
            // YD coeff = R2 * d^Nv[sinh(v)] * d^Nu[sin(u)]
            // ZD coeff = R1 * d^Nv[cosh(v)] * (Nu==0 ? 1 : 0)
            double aXCoeff = myR2 * aDnSinH * aDnCosU;
            double aYCoeff = myR2 * aDnSinH * aDnSinU;
            double aZCoeff = (Nu == 0) ? myR1 * aDnCosH : 0.0;
            aVec = aXD * aXCoeff + aYD * aYCoeff + aZD * aZCoeff;
        }

        return new gp_Vec(aVec);
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_HyperboloidSurface::Transform");
    }

    public override Geom_Geometry Copy() => new GeomEval_HyperboloidSurface(pos, myR1, myR2, myMode);

    /// <summary>
    /// The coefficients of the implicit equation in the absolute Cartesian coordinate system:
    /// <c>A1*X^2 + A2*Y^2 + A3*Z^2 + 2*(B1*X*Y + B2*X*Z + B3*Y*Z) + 2*(C1*X + C2*Y + C3*Z) + D = 0</c>.
    /// </summary>
    public void Coefficients(out double A1, out double A2, out double A3,
                             out double B1, out double B2, out double B3,
                             out double C1, out double C2, out double C3, out double D)
    {
        // Local coordinate equation:
        // 1-sheet: X^2/R1^2 + Y^2/R1^2 - Z^2/R2^2 - 1 = 0
        //   => A1_loc=1/R1^2, A2_loc=1/R1^2, A3_loc=-1/R2^2, B_loc=0, C_loc=0, D_loc=-1
        // 2-sheet: X^2/R2^2 + Y^2/R2^2 - Z^2/R1^2 + 1 = 0
        //   => A1_loc=1/R2^2, A2_loc=1/R2^2, A3_loc=-1/R1^2, B_loc=0, C_loc=0, D_loc=+1
        double aA1Loc, aA2Loc, aA3Loc, aDLoc;
        if (myMode == SheetMode.OneSheet)
        {
            aA1Loc = 1.0 / (myR1 * myR1);
            aA2Loc = 1.0 / (myR1 * myR1);
            aA3Loc = -1.0 / (myR2 * myR2);
            aDLoc = -1.0;
        }
        else
        {
            aA1Loc = 1.0 / (myR2 * myR2);
            aA2Loc = 1.0 / (myR2 * myR2);
            aA3Loc = -1.0 / (myR1 * myR1);
            aDLoc = 1.0;
        }

        // Local equation: aA1Loc*X'^2 + aA2Loc*Y'^2 + aA3Loc*Z'^2 + aDLoc = 0
        // where (X',Y',Z') = T * (X,Y,Z), T = SetTransformation(pos).
        gp_Trsf aTrsf = new gp_Trsf();
        aTrsf.SetTransformation(pos);
        double T11 = aTrsf.Value(1, 1);
        double T12 = aTrsf.Value(1, 2);
        double T13 = aTrsf.Value(1, 3);
        double T14 = aTrsf.Value(1, 4);
        double T21 = aTrsf.Value(2, 1);
        double T22 = aTrsf.Value(2, 2);
        double T23 = aTrsf.Value(2, 3);
        double T24 = aTrsf.Value(2, 4);
        double T31 = aTrsf.Value(3, 1);
        double T32 = aTrsf.Value(3, 2);
        double T33 = aTrsf.Value(3, 3);
        double T34 = aTrsf.Value(3, 4);

        // General quadric: sum over local axes of aCoeff[i] * (Ti1*X + Ti2*Y + Ti3*Z + Ti4)^2 + aDLoc
        A1 = aA1Loc * T11 * T11 + aA2Loc * T21 * T21 + aA3Loc * T31 * T31;
        A2 = aA1Loc * T12 * T12 + aA2Loc * T22 * T22 + aA3Loc * T32 * T32;
        A3 = aA1Loc * T13 * T13 + aA2Loc * T23 * T23 + aA3Loc * T33 * T33;
        B1 = aA1Loc * T11 * T12 + aA2Loc * T21 * T22 + aA3Loc * T31 * T32;
        B2 = aA1Loc * T11 * T13 + aA2Loc * T21 * T23 + aA3Loc * T31 * T33;
        B3 = aA1Loc * T12 * T13 + aA2Loc * T22 * T23 + aA3Loc * T32 * T33;
        C1 = aA1Loc * T11 * T14 + aA2Loc * T21 * T24 + aA3Loc * T31 * T34;
        C2 = aA1Loc * T12 * T14 + aA2Loc * T22 * T24 + aA3Loc * T32 * T34;
        C3 = aA1Loc * T13 * T14 + aA2Loc * T23 * T24 + aA3Loc * T33 * T34;
        D = aA1Loc * T14 * T14 + aA2Loc * T24 * T24 + aA3Loc * T34 * T34 + aDLoc;
    }
}
