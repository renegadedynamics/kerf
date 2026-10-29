// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_LogarithmicSpiralCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_LogarithmicSpiralCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A 2D logarithmic (equiangular) spiral, polar equation r = a*exp(b*t):
/// <c>C(t) = O + a*exp(b*t)*cos(t)*XDir + a*exp(b*t)*sin(t)*YDir</c>, where O and XDir come from the axis,
/// YDir is XDir turned by +90 degrees, a &gt; 0 is the scale and b &gt; 0 the growth exponent. The parameter
/// range is (-inf, +inf). The angle between the tangent and the radial direction is constant, atan(1/b).
/// Reversal and transformation are not supported (they throw <see cref="Standard_NotImplemented"/>).
/// </summary>
public sealed class Geom2dEval_LogarithmicSpiralCurve : Geom2d_Curve
{
    private gp_Ax2d myPosition;      // Local coordinate system
    private double myScale;          // Scale factor (> 0)
    private double myGrowthExponent; // Growth exponent (> 0)

    private static void powComplexInt(double theBaseRe, double theBaseIm, int theExp,
                                      out double theResRe, out double theResIm)
    {
        double aResRe = 1.0;
        double aResIm = 0.0;
        double aPowRe = theBaseRe;
        double aPowIm = theBaseIm;
        int aExp = theExp;
        while (aExp > 0)
        {
            if ((aExp & 1) != 0)
            {
                double aTmpRe = aResRe * aPowRe - aResIm * aPowIm;
                double aTmpIm = aResRe * aPowIm + aResIm * aPowRe;
                aResRe = aTmpRe;
                aResIm = aTmpIm;
            }
            double aNextPowRe = aPowRe * aPowRe - aPowIm * aPowIm;
            double aNextPowIm = 2.0 * aPowRe * aPowIm;
            aPowRe = aNextPowRe;
            aPowIm = aNextPowIm;
            aExp >>= 1;
        }
        theResRe = aResRe;
        theResIm = aResIm;
    }

    /// <summary>Creates a logarithmic spiral.</summary>
    /// <exception cref="Standard_ConstructionError">theScale &lt;= 0 or theGrowthExponent &lt;= 0.</exception>
    public Geom2dEval_LogarithmicSpiralCurve(in gp_Ax2d thePosition, double theScale, double theGrowthExponent)
    {
        myPosition = thePosition;
        myScale = theScale;
        myGrowthExponent = theGrowthExponent;
        if (theScale <= 0.0)
        {
            throw new Standard_ConstructionError("Geom2dEval_LogarithmicSpiralCurve: scale must be > 0");
        }
        if (theGrowthExponent <= 0.0)
        {
            throw new Standard_ConstructionError(
                "Geom2dEval_LogarithmicSpiralCurve: growth exponent must be > 0");
        }
    }

    /// <summary>The local coordinate system.</summary>
    public gp_Ax2d Position() => myPosition;

    /// <summary>The scale factor a.</summary>
    public double Scale() => myScale;

    /// <summary>The growth exponent b.</summary>
    public double GrowthExponent() => myGrowthExponent;

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("Geom2dEval_LogarithmicSpiralCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double ReversedParameter(double U)
    {
        throw new Standard_NotImplemented("Geom2dEval_LogarithmicSpiralCurve::ReversedParameter");
    }

    /// <summary>-Precision::Infinite().</summary>
    public override double FirstParameter() => -Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public override double LastParameter() => Precision.Infinite();

    public override bool IsClosed() => false;

    public override bool IsPeriodic() => false;

    public override GeomAbs_Shape Continuity() => GeomAbs_CN;

    public override bool IsCN(int N) => true;

    public override gp_Pnt2d EvalD0(double U)
    {
        gp_XY anO = myPosition.Location().XY();
        gp_XY aXD = myPosition.Direction().XY();
        gp_XY aYD = new(-aXD.Y(), aXD.X());
        double aExp = myScale * Math.Exp(myGrowthExponent * U);
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        return new gp_Pnt2d(anO + aExp * aCosU * aXD + aExp * aSinU * aYD);
    }

    public override ResD1 EvalD1(double U)
    {
        gp_XY anO = myPosition.Location().XY();
        gp_XY aXD = myPosition.Direction().XY();
        gp_XY aYD = new(-aXD.Y(), aXD.X());
        double aExp = myScale * Math.Exp(myGrowthExponent * U);
        double aB = myGrowthExponent;
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        ResD1 aResult;
        aResult.Point = new gp_Pnt2d(anO + aExp * aCosU * aXD + aExp * aSinU * aYD);
        // D1 = a*e^(b*t)*[(b*cos(t)-sin(t))*XDir + (b*sin(t)+cos(t))*YDir]
        aResult.D1 = new gp_Vec2d(aExp * (aB * aCosU - aSinU) * aXD + aExp * (aB * aSinU + aCosU) * aYD);
        return aResult;
    }

    public override ResD2 EvalD2(double U)
    {
        gp_XY anO = myPosition.Location().XY();
        gp_XY aXD = myPosition.Direction().XY();
        gp_XY aYD = new(-aXD.Y(), aXD.X());
        double aExp = myScale * Math.Exp(myGrowthExponent * U);
        double aB = myGrowthExponent;
        double aB2 = aB * aB;
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        ResD2 aResult;
        aResult.Point = new gp_Pnt2d(anO + aExp * aCosU * aXD + aExp * aSinU * aYD);
        aResult.D1 = new gp_Vec2d(aExp * (aB * aCosU - aSinU) * aXD + aExp * (aB * aSinU + aCosU) * aYD);
        // D2 = a*e^(b*t)*[((b^2-1)*cos(t) - 2*b*sin(t))*XDir
        //                + ((b^2-1)*sin(t) + 2*b*cos(t))*YDir]
        aResult.D2 = new gp_Vec2d(aExp * ((aB2 - 1.0) * aCosU - 2.0 * aB * aSinU) * aXD
                                  + aExp * ((aB2 - 1.0) * aSinU + 2.0 * aB * aCosU) * aYD);
        return aResult;
    }

    public override ResD3 EvalD3(double U)
    {
        gp_XY anO = myPosition.Location().XY();
        gp_XY aXD = myPosition.Direction().XY();
        gp_XY aYD = new(-aXD.Y(), aXD.X());
        double aExp = myScale * Math.Exp(myGrowthExponent * U);
        double aB = myGrowthExponent;
        double aB2 = aB * aB;
        double aB3 = aB2 * aB;
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        ResD3 aResult;
        aResult.Point = new gp_Pnt2d(anO + aExp * aCosU * aXD + aExp * aSinU * aYD);
        aResult.D1 = new gp_Vec2d(aExp * (aB * aCosU - aSinU) * aXD + aExp * (aB * aSinU + aCosU) * aYD);
        aResult.D2 = new gp_Vec2d(aExp * ((aB2 - 1.0) * aCosU - 2.0 * aB * aSinU) * aXD
                                  + aExp * ((aB2 - 1.0) * aSinU + 2.0 * aB * aCosU) * aYD);
        // D3 = a*e^(b*t)*[((b^3-3*b)*cos(t) - (3*b^2-1)*sin(t))*XDir
        //                + ((b^3-3*b)*sin(t) + (3*b^2-1)*cos(t))*YDir]
        aResult.D3 = new gp_Vec2d(aExp * ((aB3 - 3.0 * aB) * aCosU - (3.0 * aB2 - 1.0) * aSinU) * aXD
                                  + aExp * ((aB3 - 3.0 * aB) * aSinU + (3.0 * aB2 - 1.0) * aCosU) * aYD);
        return aResult;
    }

    /// <exception cref="Standard_RangeError">N &lt; 1.</exception>
    public override gp_Vec2d EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Standard_RangeError("Geom2dEval_LogarithmicSpiralCurve::EvalDN: N must be >= 1");
        }

        // For low orders, delegate to explicit implementations.
        if (N == 1)
        {
            return EvalD1(U).D1;
        }
        if (N == 2)
        {
            return EvalD2(U).D2;
        }
        if (N == 3)
        {
            return EvalD3(U).D3;
        }

        gp_XY aXD = myPosition.Direction().XY();
        gp_XY aYD = new(-aXD.Y(), aXD.X());

        double aB = myGrowthExponent;
        double aScale = myScale * Math.Exp(aB * U);
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        // d^N/dt^N [a*e^(b*t)*(cos(t) + i*sin(t))]
        // = a*e^(b*t) * (b+i)^N * (cos(t) + i*sin(t))
        powComplexInt(aB, 1.0, N, out double aPowRe, out double aPowIm);

        double aCoeffX = aPowRe * aCosU - aPowIm * aSinU;
        double aCoeffY = aPowRe * aSinU + aPowIm * aCosU;

        return new gp_Vec2d(aScale * aCoeffX * aXD + aScale * aCoeffY * aYD);
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        throw new Standard_NotImplemented("Geom2dEval_LogarithmicSpiralCurve::Transform");
    }

    public override Geom2d_Geometry Copy() =>
        new Geom2dEval_LogarithmicSpiralCurve(myPosition, myScale, myGrowthExponent);
}
