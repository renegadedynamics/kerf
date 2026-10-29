// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_CircleInvoluteCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_CircleInvoluteCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// The involute of a circle (gear tooth profiles):
/// <c>C(t) = O + R*(cos(t) + t*sin(t))*XDir + R*(sin(t) - t*cos(t))*YDir</c>, where O and XDir come from the
/// axis, YDir is XDir turned by +90 degrees and R &gt; 0 is the base circle radius. The parameter range is
/// [0, +inf). The curve starts on the base circle with a cusp (D1(0) = 0), and |D1(t)| = R*t.
/// Reversal and transformation are not supported (they throw <see cref="Standard_NotImplemented"/>).
/// </summary>
public sealed class Geom2dEval_CircleInvoluteCurve : Geom2d_Curve
{
    private gp_Ax2d myPosition; // Local coordinate system
    private double myRadius;    // Base circle radius (> 0)

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

    /// <summary>Creates the involute of the circle of radius theRadius centred at the axis location.</summary>
    /// <exception cref="Standard_ConstructionError">theRadius &lt;= 0.</exception>
    public Geom2dEval_CircleInvoluteCurve(in gp_Ax2d thePosition, double theRadius)
    {
        myPosition = thePosition;
        myRadius = theRadius;
        if (theRadius <= 0.0)
        {
            throw new Standard_ConstructionError("Geom2dEval_CircleInvoluteCurve: radius must be > 0");
        }
    }

    /// <summary>The local coordinate system.</summary>
    public gp_Ax2d Position() => myPosition;

    /// <summary>The base circle radius.</summary>
    public double Radius() => myRadius;

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("Geom2dEval_CircleInvoluteCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double ReversedParameter(double U)
    {
        throw new Standard_NotImplemented("Geom2dEval_CircleInvoluteCurve::ReversedParameter");
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
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        // C(t) = O + R*(cos(t) + t*sin(t))*XDir + R*(sin(t) - t*cos(t))*YDir
        return new gp_Pnt2d(anO + myRadius * (aCosU + U * aSinU) * aXD
                            + myRadius * (aSinU - U * aCosU) * aYD);
    }

    public override ResD1 EvalD1(double U)
    {
        gp_XY anO = myPosition.Location().XY();
        gp_XY aXD = myPosition.Direction().XY();
        gp_XY aYD = new(-aXD.Y(), aXD.X());
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        ResD1 aResult;
        aResult.Point =
            new gp_Pnt2d(anO + myRadius * (aCosU + U * aSinU) * aXD + myRadius * (aSinU - U * aCosU) * aYD);
        // D1 = R*t*cos(t)*XDir + R*t*sin(t)*YDir
        aResult.D1 = new gp_Vec2d(myRadius * U * aCosU * aXD + myRadius * U * aSinU * aYD);
        return aResult;
    }

    public override ResD2 EvalD2(double U)
    {
        gp_XY anO = myPosition.Location().XY();
        gp_XY aXD = myPosition.Direction().XY();
        gp_XY aYD = new(-aXD.Y(), aXD.X());
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        ResD2 aResult;
        aResult.Point =
            new gp_Pnt2d(anO + myRadius * (aCosU + U * aSinU) * aXD + myRadius * (aSinU - U * aCosU) * aYD);
        aResult.D1 = new gp_Vec2d(myRadius * U * aCosU * aXD + myRadius * U * aSinU * aYD);
        // D2 = R*(cos(t) - t*sin(t))*XDir + R*(sin(t) + t*cos(t))*YDir
        aResult.D2 =
            new gp_Vec2d(myRadius * (aCosU - U * aSinU) * aXD + myRadius * (aSinU + U * aCosU) * aYD);
        return aResult;
    }

    public override ResD3 EvalD3(double U)
    {
        gp_XY anO = myPosition.Location().XY();
        gp_XY aXD = myPosition.Direction().XY();
        gp_XY aYD = new(-aXD.Y(), aXD.X());
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        ResD3 aResult;
        aResult.Point =
            new gp_Pnt2d(anO + myRadius * (aCosU + U * aSinU) * aXD + myRadius * (aSinU - U * aCosU) * aYD);
        aResult.D1 = new gp_Vec2d(myRadius * U * aCosU * aXD + myRadius * U * aSinU * aYD);
        aResult.D2 =
            new gp_Vec2d(myRadius * (aCosU - U * aSinU) * aXD + myRadius * (aSinU + U * aCosU) * aYD);
        // D3 = R*(-2*sin(t) - t*cos(t))*XDir + R*(2*cos(t) - t*sin(t))*YDir
        aResult.D3 = new gp_Vec2d(myRadius * (-2.0 * aSinU - U * aCosU) * aXD
                                  + myRadius * (2.0 * aCosU - U * aSinU) * aYD);
        return aResult;
    }

    /// <exception cref="Standard_RangeError">N &lt; 1.</exception>
    public override gp_Vec2d EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Standard_RangeError("Geom2dEval_CircleInvoluteCurve::EvalDN: N must be >= 1");
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

        // General: C(t) = R*(cos(t) + t*sin(t))*XD + R*(sin(t) - t*cos(t))*YD
        // By Leibniz rule on the t*trig(t) terms:
        // f^(N)(t) = cos(t+N*Pi/2) + t*sin(t+N*Pi/2) + N*sin(t+(N-1)*Pi/2)
        // g^(N)(t) = sin(t+N*Pi/2) - t*cos(t+N*Pi/2) - N*cos(t+(N-1)*Pi/2)
        gp_XY aXD = myPosition.Direction().XY();
        gp_XY aYD = new(-aXD.Y(), aXD.X());

        double aSinU = Math.Sin(U);
        double aCosU = Math.Cos(U);
        trigShiftByQuarterTurns(aSinU, aCosU, N, out double aSinN, out double aCosN);
        trigShiftByQuarterTurns(aSinU, aCosU, N - 1, out double aSinNm1, out double aCosNm1);

        double aFN = aCosN + U * aSinN + N * aSinNm1;
        double aGN = aSinN - U * aCosN - N * aCosNm1;

        return new gp_Vec2d(myRadius * aFN * aXD + myRadius * aGN * aYD);
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        throw new Standard_NotImplemented("Geom2dEval_CircleInvoluteCurve::Transform");
    }

    public override Geom2d_Geometry Copy() => new Geom2dEval_CircleInvoluteCurve(myPosition, myRadius);
}
