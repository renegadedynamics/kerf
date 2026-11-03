// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_CircularHelixCurve.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_CircularHelixCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A circular helix of radius R &gt; 0 and pitch P (the axial advance per 2*Pi turn; negative for a left-handed
/// helix) about the Z axis of a local coordinate system:
/// <c>C(t) = O + R*cos(t)*XDir + R*sin(t)*YDir + (P*t/(2*Pi))*ZDir</c>. The parameter range is (-inf, +inf); the
/// curve is neither periodic nor closed, and is CN. Reversal and transformation are not supported (they throw
/// <see cref="MethodNotImplementedException"/>).
/// </summary>
public sealed class GeomEval_CircularHelixCurve : Geom_Curve
{
    private Frame3D myPosition; // Local coordinate system
    private double myRadius;   // Helix radius (> 0)
    private double myPitch;    // Axial advance per 2*Pi turn

    /// <summary>Creates a circular helix with the given coordinate system, radius and pitch.</summary>
    /// <exception cref="ConstructionErrorException">theRadius &lt;= 0.</exception>
    public GeomEval_CircularHelixCurve(in Frame3D thePosition, double theRadius, double thePitch)
    {
        myPosition = thePosition;
        myRadius = theRadius;
        myPitch = thePitch;
        if (theRadius <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_CircularHelixCurve: radius must be > 0");
        }
    }

    /// <summary>The local coordinate system.</summary>
    public Frame3D Position() => myPosition;

    /// <summary>The helix radius.</summary>
    public double Radius() => myRadius;

    /// <summary>The pitch (axial advance per 2*Pi turn).</summary>
    public double Pitch() => myPitch;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_CircularHelixCurve::Reverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double ReversedParameter(double U)
    {
        throw new MethodNotImplementedException("GeomEval_CircularHelixCurve::ReversedParameter");
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
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        Xyz anO = myPosition.Location().XYZ();
        Xyz aXD = myPosition.XDirection().XYZ();
        Xyz aYD = myPosition.YDirection().XYZ();
        Xyz aZD = myPosition.Direction().XYZ();
        double aZComp = myPitch * U / (2.0 * M_PI);

        return new Point3D(anO + myRadius * aCosU * aXD + myRadius * aSinU * aYD + aZComp * aZD);
    }

    public override ResD1 EvalD1(double U)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        Xyz anO = myPosition.Location().XYZ();
        Xyz aXD = myPosition.XDirection().XYZ();
        Xyz aYD = myPosition.YDirection().XYZ();
        Xyz aZD = myPosition.Direction().XYZ();
        double aZComp = myPitch * U / (2.0 * M_PI);
        double aZRate = myPitch / (2.0 * M_PI);

        ResD1 aResult;
        aResult.Point = new Point3D(anO + myRadius * aCosU * aXD + myRadius * aSinU * aYD + aZComp * aZD);

        // D1 = -R*sin(t)*XDir + R*cos(t)*YDir + P/(2*Pi)*ZDir
        aResult.D1 = new Vector3D(myRadius * (-aSinU) * aXD + myRadius * aCosU * aYD + aZRate * aZD);

        return aResult;
    }

    public override ResD2 EvalD2(double U)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        Xyz anO = myPosition.Location().XYZ();
        Xyz aXD = myPosition.XDirection().XYZ();
        Xyz aYD = myPosition.YDirection().XYZ();
        Xyz aZD = myPosition.Direction().XYZ();
        double aZComp = myPitch * U / (2.0 * M_PI);
        double aZRate = myPitch / (2.0 * M_PI);

        ResD2 aResult;
        aResult.Point = new Point3D(anO + myRadius * aCosU * aXD + myRadius * aSinU * aYD + aZComp * aZD);
        aResult.D1 = new Vector3D(myRadius * (-aSinU) * aXD + myRadius * aCosU * aYD + aZRate * aZD);

        // D2 = -R*cos(t)*XDir - R*sin(t)*YDir
        aResult.D2 = new Vector3D(myRadius * (-aCosU) * aXD + myRadius * (-aSinU) * aYD);

        return aResult;
    }

    public override ResD3 EvalD3(double U)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        Xyz anO = myPosition.Location().XYZ();
        Xyz aXD = myPosition.XDirection().XYZ();
        Xyz aYD = myPosition.YDirection().XYZ();
        Xyz aZD = myPosition.Direction().XYZ();
        double aZComp = myPitch * U / (2.0 * M_PI);
        double aZRate = myPitch / (2.0 * M_PI);

        ResD3 aResult;
        aResult.Point = new Point3D(anO + myRadius * aCosU * aXD + myRadius * aSinU * aYD + aZComp * aZD);
        aResult.D1 = new Vector3D(myRadius * (-aSinU) * aXD + myRadius * aCosU * aYD + aZRate * aZD);
        aResult.D2 = new Vector3D(myRadius * (-aCosU) * aXD + myRadius * (-aSinU) * aYD);

        // D3 = R*sin(t)*XDir - R*cos(t)*YDir
        aResult.D3 = new Vector3D(myRadius * aSinU * aXD + myRadius * (-aCosU) * aYD);

        return aResult;
    }

    /// <exception cref="RangeErrorException">N &lt; 1.</exception>
    public override Vector3D EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new RangeErrorException("GeomEval_CircularHelixCurve::EvalDN: N must be >= 1");
        }

        Xyz aXD = myPosition.XDirection().XYZ();
        Xyz aYD = myPosition.YDirection().XYZ();
        Xyz aZD = myPosition.Direction().XYZ();

        // DN cycles with period 4 in the XY plane:
        // d^N/dt^N[cos(t)] = cos(t + N*Pi/2)
        // d^N/dt^N[sin(t)] = sin(t + N*Pi/2)
        // Z component is P/(2*Pi) for N=1, 0 otherwise.
        double aPhase = N * M_PI / 2.0;
        double aCoeffX = myRadius * Math.Cos(U + aPhase);
        double aCoeffY = myRadius * Math.Sin(U + aPhase);

        Xyz aResult = aCoeffX * aXD + aCoeffY * aYD;
        if (N == 1)
        {
            aResult += (myPitch / (2.0 * M_PI)) * aZD;
        }

        return new Vector3D(aResult);
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D T)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_CircularHelixCurve::Transform");
    }

    public override Geom_Geometry Copy() => new GeomEval_CircularHelixCurve(myPosition, myRadius, myPitch);
}
