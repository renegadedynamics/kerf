// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_SineWaveCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_SineWaveCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A 3D sine wave in the XY plane of a local coordinate system, propagating along XDir and oscillating along
/// YDir: <c>C(t) = O + t*XDir + A*sin(omega*t + phi)*YDir</c>, with amplitude A &gt; 0, angular frequency
/// omega &gt; 0 and phase phi. The parameter range is (-inf, +inf); the curve is not periodic.
/// Reversal and transformation are not supported (they throw <see cref="MethodNotImplementedException"/>).
/// </summary>
public sealed class GeomEval_SineWaveCurve : Geom_Curve
{
    private Frame3D myPosition;  // Local coordinate system
    private double myAmplitude; // Wave amplitude (> 0)
    private double myOmega;     // Angular frequency (> 0)
    private double myPhase;     // Phase shift

    /// <summary>Creates a 3D sine wave curve.</summary>
    /// <exception cref="ConstructionErrorException">theAmplitude &lt;= 0 or theOmega &lt;= 0.</exception>
    public GeomEval_SineWaveCurve(in Frame3D thePosition, double theAmplitude, double theOmega, double thePhase = 0.0)
    {
        myPosition = thePosition;
        myAmplitude = theAmplitude;
        myOmega = theOmega;
        myPhase = thePhase;
        if (theAmplitude <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_SineWaveCurve: amplitude must be > 0");
        }
        if (theOmega <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_SineWaveCurve: omega must be > 0");
        }
    }

    /// <summary>The local coordinate system.</summary>
    public Frame3D Position() => myPosition;

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
        throw new MethodNotImplementedException("GeomEval_SineWaveCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double ReversedParameter(double U)
    {
        throw new MethodNotImplementedException("GeomEval_SineWaveCurve::ReversedParameter");
    }

    /// <summary>-Precision::Infinite().</summary>
    public override double FirstParameter() => -global::Kerf.Engine.Foundation.Precision.Infinite();

    /// <summary>Precision::Infinite().</summary>
    public override double LastParameter() => global::Kerf.Engine.Foundation.Precision.Infinite();

    public override bool IsClosed() => false;

    public override bool IsPeriodic() => false;

    public override Continuity Continuity() => CN;

    public override bool IsCN(int N) => true;

    public override Point3D EvalD0(double U)
    {
        Xyz anO = myPosition.Location().XYZ();
        Xyz aXD = myPosition.XDirection().XYZ();
        Xyz aYD = myPosition.YDirection().XYZ();

        double aArg = myOmega * U + myPhase;

        return new Point3D(anO + U * aXD + myAmplitude * Math.Sin(aArg) * aYD);
    }

    public override ResD1 EvalD1(double U)
    {
        Xyz anO = myPosition.Location().XYZ();
        Xyz aXD = myPosition.XDirection().XYZ();
        Xyz aYD = myPosition.YDirection().XYZ();

        double aArg = myOmega * U + myPhase;

        ResD1 aResult;
        aResult.Point = new Point3D(anO + U * aXD + myAmplitude * Math.Sin(aArg) * aYD);

        // D1 = XDir + A*omega*cos(omega*t + phi)*YDir
        aResult.D1 = new Vector3D(aXD + myAmplitude * myOmega * Math.Cos(aArg) * aYD);

        return aResult;
    }

    public override ResD2 EvalD2(double U)
    {
        Xyz anO = myPosition.Location().XYZ();
        Xyz aXD = myPosition.XDirection().XYZ();
        Xyz aYD = myPosition.YDirection().XYZ();

        double aArg = myOmega * U + myPhase;
        double aOm2 = myOmega * myOmega;

        ResD2 aResult;
        aResult.Point = new Point3D(anO + U * aXD + myAmplitude * Math.Sin(aArg) * aYD);
        aResult.D1 = new Vector3D(aXD + myAmplitude * myOmega * Math.Cos(aArg) * aYD);

        // D2 = -A*omega^2*sin(omega*t + phi)*YDir
        aResult.D2 = new Vector3D(myAmplitude * (-aOm2) * Math.Sin(aArg) * aYD);

        return aResult;
    }

    public override ResD3 EvalD3(double U)
    {
        Xyz anO = myPosition.Location().XYZ();
        Xyz aXD = myPosition.XDirection().XYZ();
        Xyz aYD = myPosition.YDirection().XYZ();

        double aArg = myOmega * U + myPhase;
        double aOm2 = myOmega * myOmega;
        double aOm3 = aOm2 * myOmega;

        ResD3 aResult;
        aResult.Point = new Point3D(anO + U * aXD + myAmplitude * Math.Sin(aArg) * aYD);
        aResult.D1 = new Vector3D(aXD + myAmplitude * myOmega * Math.Cos(aArg) * aYD);
        aResult.D2 = new Vector3D(myAmplitude * (-aOm2) * Math.Sin(aArg) * aYD);

        // D3 = -A*omega^3*cos(omega*t + phi)*YDir
        aResult.D3 = new Vector3D(myAmplitude * (-aOm3) * Math.Cos(aArg) * aYD);

        return aResult;
    }

    /// <exception cref="RangeErrorException">N &lt; 1.</exception>
    public override Vector3D EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new RangeErrorException("GeomEval_SineWaveCurve::EvalDN: N must be >= 1");
        }

        Xyz aXD = myPosition.XDirection().XYZ();
        Xyz aYD = myPosition.YDirection().XYZ();

        // d^N/dt^N[t] = 1 if N=1, 0 if N>=2
        // d^N/dt^N[A*sin(omega*t+phi)] = A*omega^N*sin(omega*t+phi+N*Pi/2)
        double aOmN = 1.0;
        for (int i = 0; i < N; ++i)
        {
            aOmN *= myOmega;
        }
        double aPhase = myOmega * U + myPhase + N * M_PI / 2.0;

        Xyz aResult = myAmplitude * aOmN * Math.Sin(aPhase) * aYD;
        if (N == 1)
        {
            aResult += aXD;
        }

        return new Vector3D(aResult);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D T)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_SineWaveCurve::Transform");
    }

    public override Geom_Geometry Copy() => new GeomEval_SineWaveCurve(myPosition, myAmplitude, myOmega, myPhase);
}
