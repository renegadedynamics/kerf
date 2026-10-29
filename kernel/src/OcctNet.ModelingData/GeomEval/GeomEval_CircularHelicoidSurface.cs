// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_CircularHelicoidSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_CircularHelicoidSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A circular helicoid: the ruled surface swept by a line rotating uniformly about an axis while translating
/// along it, <c>S(u,v) = O + v*cos(u)*XDir + v*sin(u)*YDir + (P*u/(2*Pi))*ZDir</c>, with pitch P != 0 (axial
/// advance per 2*Pi turn). u and v in (-inf, +inf); neither periodic nor closed; CN. Reversal, transformation and
/// iso curves are not supported (they throw <see cref="Standard_NotImplemented"/>).
/// </summary>
public sealed class GeomEval_CircularHelicoidSurface : Geom_ElementarySurface
{
    private double myPitch; // Axial advance per 2*Pi turn

    /// <summary>Creates a circular helicoid.</summary>
    /// <exception cref="Standard_ConstructionError">thePitch == 0.</exception>
    public GeomEval_CircularHelicoidSurface(in gp_Ax3 thePosition, double thePitch)
    {
        myPitch = thePitch;
        if (thePitch == 0.0)
        {
            throw new Standard_ConstructionError("GeomEval_CircularHelicoidSurface: pitch must be != 0");
        }
        pos = thePosition;
    }

    /// <summary>The pitch.</summary>
    public double Pitch() => myPitch;

    /// <exception cref="Standard_ConstructionError">thePitch == 0.</exception>
    internal void SetPitch(double thePitch)
    {
        CheckMutable();
        if (thePitch == 0.0)
        {
            throw new Standard_ConstructionError(
                "GeomEval_CircularHelicoidSurface::SetPitch: pitch must be != 0");
        }
        myPitch = thePitch;
    }

    /// <summary>A copy with the pitch set to thePitch.</summary>
    /// <exception cref="Standard_ConstructionError">thePitch == 0.</exception>
    public GeomEval_CircularHelicoidSurface WithPitch(double thePitch)
    {
        GeomEval_CircularHelicoidSurface aCopy = (GeomEval_CircularHelicoidSurface)Copy();
        aCopy.SetPitch(thePitch);
        return aCopy;
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_CircularHelicoidSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_CircularHelicoidSurface::VReverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double UReversedParameter(double U)
    {
        throw new Standard_NotImplemented("GeomEval_CircularHelicoidSurface::UReversedParameter");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double VReversedParameter(double V)
    {
        throw new Standard_NotImplemented("GeomEval_CircularHelicoidSurface::VReversedParameter");
    }

    /// <summary>Infinite bounds for both parameters.</summary>
    public override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = -Precision.Infinite();
        U2 = Precision.Infinite();
        V1 = -Precision.Infinite();
        V2 = Precision.Infinite();
    }

    public override bool IsUClosed() => false;

    public override bool IsVClosed() => false;

    public override bool IsUPeriodic() => false;

    public override bool IsVPeriodic() => false;

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override Geom_Curve UIso(double U)
    {
        throw new Standard_NotImplemented("GeomEval_CircularHelicoidSurface::UIso");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override Geom_Curve VIso(double V)
    {
        throw new Standard_NotImplemented("GeomEval_CircularHelicoidSurface::VIso");
    }

    public override gp_Pnt EvalD0(double U, double V)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        gp_XYZ anO = pos.Location().XYZ();
        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        double aZComp = myPitch * U / (2.0 * M_PI);

        return new gp_Pnt(anO + V * aCosU * aXD + V * aSinU * aYD + aZComp * aZD);
    }

    public override ResD1 EvalD1(double U, double V)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        gp_XYZ anO = pos.Location().XYZ();
        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        double aZComp = myPitch * U / (2.0 * M_PI);
        double aZRate = myPitch / (2.0 * M_PI);

        ResD1 aResult;
        aResult.Point = new gp_Pnt(anO + V * aCosU * aXD + V * aSinU * aYD + aZComp * aZD);

        // dS/du = -v*sin(u)*XDir + v*cos(u)*YDir + P/(2*Pi)*ZDir
        aResult.D1U = new gp_Vec(V * (-aSinU) * aXD + V * aCosU * aYD + aZRate * aZD);

        // dS/dv = cos(u)*XDir + sin(u)*YDir
        aResult.D1V = new gp_Vec(aCosU * aXD + aSinU * aYD);

        return aResult;
    }

    public override ResD2 EvalD2(double U, double V)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        gp_XYZ anO = pos.Location().XYZ();
        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        double aZComp = myPitch * U / (2.0 * M_PI);
        double aZRate = myPitch / (2.0 * M_PI);

        ResD2 aResult;
        aResult.Point = new gp_Pnt(anO + V * aCosU * aXD + V * aSinU * aYD + aZComp * aZD);
        aResult.D1U = new gp_Vec(V * (-aSinU) * aXD + V * aCosU * aYD + aZRate * aZD);
        aResult.D1V = new gp_Vec(aCosU * aXD + aSinU * aYD);

        // d2S/dudu = -v*cos(u)*XDir - v*sin(u)*YDir
        aResult.D2U = new gp_Vec(V * (-aCosU) * aXD + V * (-aSinU) * aYD);

        // d2S/dvdv = 0 (ruled surface)
        aResult.D2V = new gp_Vec(0.0, 0.0, 0.0);

        // d2S/dudv = -sin(u)*XDir + cos(u)*YDir
        aResult.D2UV = new gp_Vec((-aSinU) * aXD + aCosU * aYD);

        return aResult;
    }

    public override ResD3 EvalD3(double U, double V)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);

        gp_XYZ anO = pos.Location().XYZ();
        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        double aZComp = myPitch * U / (2.0 * M_PI);
        double aZRate = myPitch / (2.0 * M_PI);

        ResD3 aResult;
        aResult.Point = new gp_Pnt(anO + V * aCosU * aXD + V * aSinU * aYD + aZComp * aZD);
        aResult.D1U = new gp_Vec(V * (-aSinU) * aXD + V * aCosU * aYD + aZRate * aZD);
        aResult.D1V = new gp_Vec(aCosU * aXD + aSinU * aYD);
        aResult.D2U = new gp_Vec(V * (-aCosU) * aXD + V * (-aSinU) * aYD);
        aResult.D2V = new gp_Vec(0.0, 0.0, 0.0);
        aResult.D2UV = new gp_Vec((-aSinU) * aXD + aCosU * aYD);

        // d3S/du3 = v*sin(u)*XDir - v*cos(u)*YDir
        aResult.D3U = new gp_Vec(V * aSinU * aXD + V * (-aCosU) * aYD);

        // d3S/dv3 = 0
        aResult.D3V = new gp_Vec(0.0, 0.0, 0.0);

        // d3S/du2dv = -cos(u)*XDir - sin(u)*YDir
        aResult.D3UUV = new gp_Vec((-aCosU) * aXD + (-aSinU) * aYD);

        // d3S/dudv2 = 0
        aResult.D3UVV = new gp_Vec(0.0, 0.0, 0.0);

        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public override gp_Vec EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Geom_UndefinedDerivative(
                "GeomEval_CircularHelicoidSurface::EvalDN: invalid derivative order");
        }

        // S(u,v) = O + v*cos(u)*XDir + v*sin(u)*YDir + P*u/(2*Pi)*ZDir
        // The v-dependence is linear, so Nv >= 2 => 0.
        // The u-dependence of the Z term is linear, so Nu >= 2 on Z => 0.
        if (Nv >= 2)
        {
            return new gp_Vec(0.0, 0.0, 0.0);
        }

        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        // d^Nu/du^Nu[cos(u)] = cos(u + Nu*Pi/2)
        // d^Nu/du^Nu[sin(u)] = sin(u + Nu*Pi/2)
        double aPhaseU = Nu * M_PI / 2.0;

        if (Nv == 0)
        {
            // d^Nu/du^Nu of [v*cos(u)*XD + v*sin(u)*YD + P*u/(2*Pi)*ZD]
            double aCoeffX0 = V * Math.Cos(U + aPhaseU);
            double aCoeffY0 = V * Math.Sin(U + aPhaseU);
            gp_XYZ aResult = aCoeffX0 * aXD + aCoeffY0 * aYD;

            // Z term: d^Nu/du^Nu[P*u/(2*Pi)] = P/(2*Pi) if Nu=1, 0 if Nu>=2
            if (Nu == 1)
            {
                aResult += (myPitch / (2.0 * M_PI)) * aZD;
            }
            return new gp_Vec(aResult);
        }

        // Nv == 1: d/dv of d^Nu/du^Nu [v*cos(u)*XD + v*sin(u)*YD]
        // = d^Nu/du^Nu[cos(u)]*XD + d^Nu/du^Nu[sin(u)]*YD
        double aCoeffX = Math.Cos(U + aPhaseU);
        double aCoeffY = Math.Sin(U + aPhaseU);
        return new gp_Vec(aCoeffX * aXD + aCoeffY * aYD);
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_CircularHelicoidSurface::Transform");
    }

    public override Geom_Geometry Copy() => new GeomEval_CircularHelicoidSurface(pos, myPitch);
}
