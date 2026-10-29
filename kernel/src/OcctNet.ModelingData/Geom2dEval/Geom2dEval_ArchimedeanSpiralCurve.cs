// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_ArchimedeanSpiralCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_ArchimedeanSpiralCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A 2D Archimedean spiral, polar equation r = a + b*t:
/// <c>C(t) = O + (a + b*t)*cos(t)*XDir + (a + b*t)*sin(t)*YDir</c>, where O and XDir come from the axis, YDir
/// is XDir turned by +90 degrees, a &gt;= 0 is the initial radius and b &gt; 0 the growth rate per radian.
/// The parameter range is [0, +inf); the curve is neither periodic nor closed.
/// Reversal and transformation are not supported (they throw <see cref="Standard_NotImplemented"/>).
/// </summary>
public sealed class Geom2dEval_ArchimedeanSpiralCurve : Geom2d_Curve
{
    private gp_Ax2d myPosition;     // Local coordinate system
    private double myInitialRadius; // Initial radius (>= 0)
    private double myGrowthRate;    // Growth rate per radian (> 0)

    private static void trigShiftByQuarterTurns(double theSin, double theCos, int theN,
                                                out double theSinShift, out double theCosShift)
    {
        switch (theN & 3)
        {
            case 0:
                theSinShift = theSin;
                theCosShift = theCos;
                break;
            case 1:
                theSinShift = theCos;
                theCosShift = -theSin;
                break;
            case 2:
                theSinShift = -theSin;
                theCosShift = -theCos;
                break;
            default:
                theSinShift = -theCos;
                theCosShift = theSin;
                break;
        }
    }

    /// <summary>Creates an Archimedean spiral.</summary>
    /// <exception cref="Standard_ConstructionError">theInitialRadius &lt; 0 or theGrowthRate &lt;= 0.</exception>
    public Geom2dEval_ArchimedeanSpiralCurve(in gp_Ax2d thePosition, double theInitialRadius, double theGrowthRate)
    {
        myPosition = thePosition;
        myInitialRadius = theInitialRadius;
        myGrowthRate = theGrowthRate;
        if (theInitialRadius < 0.0)
        {
            throw new Standard_ConstructionError(
                "Geom2dEval_ArchimedeanSpiralCurve: initial radius must be >= 0");
        }
        if (theGrowthRate <= 0.0)
        {
            throw new Standard_ConstructionError("Geom2dEval_ArchimedeanSpiralCurve: growth rate must be > 0");
        }
    }

    /// <summary>The local coordinate system.</summary>
    public gp_Ax2d Position() => myPosition;

    /// <summary>The initial radius a.</summary>
    public double InitialRadius() => myInitialRadius;

    /// <summary>The growth rate b per radian.</summary>
    public double GrowthRate() => myGrowthRate;

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("Geom2dEval_ArchimedeanSpiralCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double ReversedParameter(double U)
    {
        throw new Standard_NotImplemented("Geom2dEval_ArchimedeanSpiralCurve::ReversedParameter");
    }

    /// <summary>0.</summary>
    public override double FirstParameter() => 0.0;

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
        double aR = myInitialRadius + myGrowthRate * U;
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        return new gp_Pnt2d(anO + aR * aCosU * aXD + aR * aSinU * aYD);
    }

    public override ResD1 EvalD1(double U)
    {
        gp_XY anO = myPosition.Location().XY();
        gp_XY aXD = myPosition.Direction().XY();
        gp_XY aYD = new(-aXD.Y(), aXD.X());
        double aR = myInitialRadius + myGrowthRate * U;
        double aB = myGrowthRate;
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        ResD1 aResult;
        aResult.Point = new gp_Pnt2d(anO + aR * aCosU * aXD + aR * aSinU * aYD);
        // D1 = (b*cos(t) - r*sin(t))*XDir + (b*sin(t) + r*cos(t))*YDir
        aResult.D1 = new gp_Vec2d((aB * aCosU - aR * aSinU) * aXD + (aB * aSinU + aR * aCosU) * aYD);
        return aResult;
    }

    public override ResD2 EvalD2(double U)
    {
        gp_XY anO = myPosition.Location().XY();
        gp_XY aXD = myPosition.Direction().XY();
        gp_XY aYD = new(-aXD.Y(), aXD.X());
        double aR = myInitialRadius + myGrowthRate * U;
        double aB = myGrowthRate;
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        ResD2 aResult;
        aResult.Point = new gp_Pnt2d(anO + aR * aCosU * aXD + aR * aSinU * aYD);
        aResult.D1 = new gp_Vec2d((aB * aCosU - aR * aSinU) * aXD + (aB * aSinU + aR * aCosU) * aYD);
        // D2 = (-2*b*sin(t) - r*cos(t))*XDir + (2*b*cos(t) - r*sin(t))*YDir
        aResult.D2 =
            new gp_Vec2d((-2.0 * aB * aSinU - aR * aCosU) * aXD + (2.0 * aB * aCosU - aR * aSinU) * aYD);
        return aResult;
    }

    public override ResD3 EvalD3(double U)
    {
        gp_XY anO = myPosition.Location().XY();
        gp_XY aXD = myPosition.Direction().XY();
        gp_XY aYD = new(-aXD.Y(), aXD.X());
        double aR = myInitialRadius + myGrowthRate * U;
        double aB = myGrowthRate;
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        ResD3 aResult;
        aResult.Point = new gp_Pnt2d(anO + aR * aCosU * aXD + aR * aSinU * aYD);
        aResult.D1 = new gp_Vec2d((aB * aCosU - aR * aSinU) * aXD + (aB * aSinU + aR * aCosU) * aYD);
        aResult.D2 =
            new gp_Vec2d((-2.0 * aB * aSinU - aR * aCosU) * aXD + (2.0 * aB * aCosU - aR * aSinU) * aYD);
        // D3 = (-3*b*cos(t) + r*sin(t))*XDir + (-3*b*sin(t) - r*cos(t))*YDir
        aResult.D3 =
            new gp_Vec2d((-3.0 * aB * aCosU + aR * aSinU) * aXD + (-3.0 * aB * aSinU - aR * aCosU) * aYD);
        return aResult;
    }

    /// <exception cref="Standard_RangeError">N &lt; 1.</exception>
    public override gp_Vec2d EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Standard_RangeError("Geom2dEval_ArchimedeanSpiralCurve::EvalDN: N must be >= 1");
        }

        // By Leibniz rule: d^N/dt^N[r(t)*f(t)] = sum_{k=0}^{N} C(N,k)*r^(k)*f^(N-k)
        // where r(t) = a + b*t, r' = b, r^(k) = 0 for k >= 2.
        // So only k=0 and k=1 contribute: = r(t)*f^(N)(t) + N*b*f^(N-1)(t)
        gp_XY aXD = myPosition.Direction().XY();
        gp_XY aYD = new(-aXD.Y(), aXD.X());

        double aR = myInitialRadius + myGrowthRate * U;
        double aB = myGrowthRate;

        double aSinU = Math.Sin(U);
        double aCosU = Math.Cos(U);
        trigShiftByQuarterTurns(aSinU, aCosU, N, out double aSinN, out double aCosN);
        trigShiftByQuarterTurns(aSinU, aCosU, N - 1, out double aSinNm1, out double aCosNm1);

        // X component: r*cos^(N)(t) + N*b*cos^(N-1)(t)
        double aCoeffX = aR * aCosN + N * aB * aCosNm1;
        // Y component: r*sin^(N)(t) + N*b*sin^(N-1)(t)
        double aCoeffY = aR * aSinN + N * aB * aSinNm1;

        return new gp_Vec2d(aCoeffX * aXD + aCoeffY * aYD);
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        throw new Standard_NotImplemented("Geom2dEval_ArchimedeanSpiralCurve::Transform");
    }

    public override Geom2d_Geometry Copy() =>
        new Geom2dEval_ArchimedeanSpiralCurve(myPosition, myInitialRadius, myGrowthRate);
}
