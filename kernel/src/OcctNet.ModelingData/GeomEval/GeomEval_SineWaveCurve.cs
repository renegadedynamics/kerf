// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_SineWaveCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_SineWaveCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A 3D sine wave in the XY plane of a local coordinate system, propagating along XDir and oscillating along
/// YDir: <c>C(t) = O + t*XDir + A*sin(omega*t + phi)*YDir</c>, with amplitude A &gt; 0, angular frequency
/// omega &gt; 0 and phase phi. The parameter range is (-inf, +inf); the curve is not periodic.
/// Reversal and transformation are not supported (they throw <see cref="Standard_NotImplemented"/>).
/// </summary>
public sealed class GeomEval_SineWaveCurve : Geom_Curve
{
    private gp_Ax2 myPosition;  // Local coordinate system
    private double myAmplitude; // Wave amplitude (> 0)
    private double myOmega;     // Angular frequency (> 0)
    private double myPhase;     // Phase shift

    /// <summary>Creates a 3D sine wave curve.</summary>
    /// <exception cref="Standard_ConstructionError">theAmplitude &lt;= 0 or theOmega &lt;= 0.</exception>
    public GeomEval_SineWaveCurve(in gp_Ax2 thePosition, double theAmplitude, double theOmega, double thePhase = 0.0)
    {
        myPosition = thePosition;
        myAmplitude = theAmplitude;
        myOmega = theOmega;
        myPhase = thePhase;
        if (theAmplitude <= 0.0)
        {
            throw new Standard_ConstructionError("GeomEval_SineWaveCurve: amplitude must be > 0");
        }
        if (theOmega <= 0.0)
        {
            throw new Standard_ConstructionError("GeomEval_SineWaveCurve: omega must be > 0");
        }
    }

    /// <summary>The local coordinate system.</summary>
    public gp_Ax2 Position() => myPosition;

    /// <summary>The amplitude.</summary>
    public double Amplitude() => myAmplitude;

    /// <summary>The angular frequency.</summary>
    public double Omega() => myOmega;

    /// <summary>The phase shift.</summary>
    public double Phase() => myPhase;

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_SineWaveCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double ReversedParameter(double U)
    {
        throw new Standard_NotImplemented("GeomEval_SineWaveCurve::ReversedParameter");
    }

    /// <summary>-Precision::Infinite().</summary>
    public override double FirstParameter() => -Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public override double LastParameter() => Precision.Infinite();

    public override bool IsClosed() => false;

    public override bool IsPeriodic() => false;

    public override GeomAbs_Shape Continuity() => GeomAbs_CN;

    public override bool IsCN(int N) => true;

    public override gp_Pnt EvalD0(double U)
    {
        gp_XYZ anO = myPosition.Location().XYZ();
        gp_XYZ aXD = myPosition.XDirection().XYZ();
        gp_XYZ aYD = myPosition.YDirection().XYZ();

        double aArg = myOmega * U + myPhase;

        return new gp_Pnt(anO + U * aXD + myAmplitude * Math.Sin(aArg) * aYD);
    }

    public override ResD1 EvalD1(double U)
    {
        gp_XYZ anO = myPosition.Location().XYZ();
        gp_XYZ aXD = myPosition.XDirection().XYZ();
        gp_XYZ aYD = myPosition.YDirection().XYZ();

        double aArg = myOmega * U + myPhase;

        ResD1 aResult;
        aResult.Point = new gp_Pnt(anO + U * aXD + myAmplitude * Math.Sin(aArg) * aYD);

        // D1 = XDir + A*omega*cos(omega*t + phi)*YDir
        aResult.D1 = new gp_Vec(aXD + myAmplitude * myOmega * Math.Cos(aArg) * aYD);

        return aResult;
    }

    public override ResD2 EvalD2(double U)
    {
        gp_XYZ anO = myPosition.Location().XYZ();
        gp_XYZ aXD = myPosition.XDirection().XYZ();
        gp_XYZ aYD = myPosition.YDirection().XYZ();

        double aArg = myOmega * U + myPhase;
        double aOm2 = myOmega * myOmega;

        ResD2 aResult;
        aResult.Point = new gp_Pnt(anO + U * aXD + myAmplitude * Math.Sin(aArg) * aYD);
        aResult.D1 = new gp_Vec(aXD + myAmplitude * myOmega * Math.Cos(aArg) * aYD);

        // D2 = -A*omega^2*sin(omega*t + phi)*YDir
        aResult.D2 = new gp_Vec(myAmplitude * (-aOm2) * Math.Sin(aArg) * aYD);

        return aResult;
    }

    public override ResD3 EvalD3(double U)
    {
        gp_XYZ anO = myPosition.Location().XYZ();
        gp_XYZ aXD = myPosition.XDirection().XYZ();
        gp_XYZ aYD = myPosition.YDirection().XYZ();

        double aArg = myOmega * U + myPhase;
        double aOm2 = myOmega * myOmega;
        double aOm3 = aOm2 * myOmega;

        ResD3 aResult;
        aResult.Point = new gp_Pnt(anO + U * aXD + myAmplitude * Math.Sin(aArg) * aYD);
        aResult.D1 = new gp_Vec(aXD + myAmplitude * myOmega * Math.Cos(aArg) * aYD);
        aResult.D2 = new gp_Vec(myAmplitude * (-aOm2) * Math.Sin(aArg) * aYD);

        // D3 = -A*omega^3*cos(omega*t + phi)*YDir
        aResult.D3 = new gp_Vec(myAmplitude * (-aOm3) * Math.Cos(aArg) * aYD);

        return aResult;
    }

    /// <exception cref="Standard_RangeError">N &lt; 1.</exception>
    public override gp_Vec EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Standard_RangeError("GeomEval_SineWaveCurve::EvalDN: N must be >= 1");
        }

        gp_XYZ aXD = myPosition.XDirection().XYZ();
        gp_XYZ aYD = myPosition.YDirection().XYZ();

        // d^N/dt^N[t] = 1 if N=1, 0 if N>=2
        // d^N/dt^N[A*sin(omega*t+phi)] = A*omega^N*sin(omega*t+phi+N*Pi/2)
        double aOmN = 1.0;
        for (int i = 0; i < N; ++i)
        {
            aOmN *= myOmega;
        }
        double aPhase = myOmega * U + myPhase + N * M_PI / 2.0;

        gp_XYZ aResult = myAmplitude * aOmN * Math.Sin(aPhase) * aYD;
        if (N == 1)
        {
            aResult += aXD;
        }

        return new gp_Vec(aResult);
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_SineWaveCurve::Transform");
    }

    public override Geom_Geometry Copy() => new GeomEval_SineWaveCurve(myPosition, myAmplitude, myOmega, myPhase);
}
