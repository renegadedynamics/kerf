// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_SineWaveCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2dEval/Geom2dEval_SineWaveCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A 2D sine wave: <c>C(t) = O + t*XDir + A*sin(omega*t + phi)*YDir</c>, where O and XDir come from the
/// axis, YDir is XDir turned by +90 degrees, A &gt; 0 is the amplitude, omega &gt; 0 the angular frequency and
/// phi the phase. The parameter range is (-inf, +inf); the curve is not periodic.
/// Reversal and transformation are not supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
public sealed class Geom2dEval_SineWaveCurve : Geom2d_Curve
{
    private Axis2D myPosition;  // Local coordinate system
    private double myAmplitude;  // Wave amplitude (> 0)
    private double myOmega;      // Angular frequency (> 0)
    private double myPhase;      // Phase shift

    private static double powInt(double theBase, int theExp)
    {
        double aRes = 1.0;
        for (int i = 0; i < theExp; ++i)
        {
            aRes *= theBase;
        }
        return aRes;
    }

    private static double sinShiftByQuarterTurns(double theSin, double theCos, int theN)
    {
        switch (theN & 3)
        {
            case 0:
                return theSin;
            case 1:
                return theCos;
            case 2:
                return -theSin;
            default:
                return -theCos;
        }
    }

    /// <summary>Creates a 2D sine wave curve.</summary>
    /// <exception cref="ConstructionErrorException">theAmplitude &lt;= 0 or theOmega &lt;= 0.</exception>
    public Geom2dEval_SineWaveCurve(in Axis2D thePosition, double theAmplitude, double theOmega, double thePhase = 0.0)
    {
        myPosition = thePosition;
        myAmplitude = theAmplitude;
        myOmega = theOmega;
        myPhase = thePhase;
        if (theAmplitude <= 0.0)
        {
            throw new ConstructionErrorException("Geom2dEval_SineWaveCurve: amplitude must be > 0");
        }
        if (theOmega <= 0.0)
        {
            throw new ConstructionErrorException("Geom2dEval_SineWaveCurve: omega must be > 0");
        }
    }

    /// <summary>The local coordinate system.</summary>
    public Axis2D Position() => myPosition;

    /// <summary>The amplitude.</summary>
    public double Amplitude() => myAmplitude;

    /// <summary>The angular frequency.</summary>
    public double Omega() => myOmega;

    /// <summary>The phase shift.</summary>
    public double Phase() => myPhase;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("Geom2dEval_SineWaveCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double ReversedParameter(double U)
    {
        throw new MethodNotImplementedException("Geom2dEval_SineWaveCurve::ReversedParameter");
    }

    /// <summary>-Precision::Infinite().</summary>
    public override double FirstParameter() => -global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public override double LastParameter() => global::Kerf.Engine.Foundation.Precision.Infinite();

    public override bool IsClosed() => false;

    public override bool IsPeriodic() => false;

    public override Continuity Continuity() => CN;

    public override bool IsCN(int N) => true;

    public override Point2D EvalD0(double U)
    {
        Xy anO = myPosition.Location().XY();
        Xy aXD = myPosition.Direction().XY();
        Xy aYD = new(-aXD.Y(), aXD.X());

        double aArg = myOmega * U + myPhase;

        return new Point2D(anO + U * aXD + myAmplitude * Math.Sin(aArg) * aYD);
    }

    public override ResD1 EvalD1(double U)
    {
        Xy anO = myPosition.Location().XY();
        Xy aXD = myPosition.Direction().XY();
        Xy aYD = new(-aXD.Y(), aXD.X());

        double aArg = myOmega * U + myPhase;

        ResD1 aResult;
        aResult.Point = new Point2D(anO + U * aXD + myAmplitude * Math.Sin(aArg) * aYD);
        aResult.D1 = new Vector2D(aXD + myAmplitude * myOmega * Math.Cos(aArg) * aYD);
        return aResult;
    }

    public override ResD2 EvalD2(double U)
    {
        Xy anO = myPosition.Location().XY();
        Xy aXD = myPosition.Direction().XY();
        Xy aYD = new(-aXD.Y(), aXD.X());

        double aArg = myOmega * U + myPhase;
        double aOm2 = myOmega * myOmega;

        ResD2 aResult;
        aResult.Point = new Point2D(anO + U * aXD + myAmplitude * Math.Sin(aArg) * aYD);
        aResult.D1 = new Vector2D(aXD + myAmplitude * myOmega * Math.Cos(aArg) * aYD);
        aResult.D2 = new Vector2D(myAmplitude * (-aOm2) * Math.Sin(aArg) * aYD);
        return aResult;
    }

    public override ResD3 EvalD3(double U)
    {
        Xy anO = myPosition.Location().XY();
        Xy aXD = myPosition.Direction().XY();
        Xy aYD = new(-aXD.Y(), aXD.X());

        double aArg = myOmega * U + myPhase;
        double aOm2 = myOmega * myOmega;
        double aOm3 = aOm2 * myOmega;

        ResD3 aResult;
        aResult.Point = new Point2D(anO + U * aXD + myAmplitude * Math.Sin(aArg) * aYD);
        aResult.D1 = new Vector2D(aXD + myAmplitude * myOmega * Math.Cos(aArg) * aYD);
        aResult.D2 = new Vector2D(myAmplitude * (-aOm2) * Math.Sin(aArg) * aYD);
        aResult.D3 = new Vector2D(myAmplitude * (-aOm3) * Math.Cos(aArg) * aYD);
        return aResult;
    }

    /// <exception cref="RangeErrorException">N &lt; 1.</exception>
    public override Vector2D EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new RangeErrorException("Geom2dEval_SineWaveCurve::EvalDN: N must be >= 1");
        }

        Xy aXD = myPosition.Direction().XY();
        Xy aYD = new(-aXD.Y(), aXD.X());

        double aArg = myOmega * U + myPhase;
        double aSin = Math.Sin(aArg);
        double aCos = Math.Cos(aArg);
        double aOmN = powInt(myOmega, N);
        double aSinN = sinShiftByQuarterTurns(aSin, aCos, N);

        Xy aResult = myAmplitude * aOmN * aSinN * aYD;
        if (N == 1)
        {
            aResult += aXD;
        }

        return new Vector2D(aResult);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform2D T)
    {
        CheckMutable();
        throw new MethodNotImplementedException("Geom2dEval_SineWaveCurve::Transform");
    }

    public override Geom2d_Geometry Copy() => new Geom2dEval_SineWaveCurve(myPosition, myAmplitude, myOmega, myPhase);
}
