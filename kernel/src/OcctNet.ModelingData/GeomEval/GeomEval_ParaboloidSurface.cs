// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_ParaboloidSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_ParaboloidSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A circular paraboloid of revolution with focal distance F &gt; 0, its vertex at the origin of its coordinate
/// system: <c>P(u,v) = O + v*cos(u)*XDir + v*sin(u)*YDir + v^2/(4*F)*ZDir</c>, u in [0, 2*Pi] (periodic),
/// v in (-inf, +inf). Implicit equation in local coordinates: <c>X^2 + Y^2 - 4*F*Z = 0</c>.
/// Reversal, transformation and iso curves are not supported (they throw <see cref="Standard_NotImplemented"/>).
/// </summary>
public sealed class GeomEval_ParaboloidSurface : Geom_ElementarySurface
{
    private double myFocal; // Focal distance (> 0)

    /// <summary>Creates a paraboloid with the given local coordinate system and focal distance.</summary>
    /// <exception cref="Standard_ConstructionError">theFocal &lt;= 0.</exception>
    public GeomEval_ParaboloidSurface(in gp_Ax3 thePosition, double theFocal)
    {
        myFocal = theFocal;
        if (theFocal <= 0.0)
        {
            throw new Standard_ConstructionError("GeomEval_ParaboloidSurface: focal distance must be > 0");
        }
        pos = thePosition;
    }

    /// <summary>The focal distance.</summary>
    public double Focal() => myFocal;

    /// <exception cref="Standard_ConstructionError">theFocal &lt;= 0.</exception>
    internal void SetFocal(double theFocal)
    {
        CheckMutable();
        if (theFocal <= 0.0)
        {
            throw new Standard_ConstructionError(
                "GeomEval_ParaboloidSurface::SetFocal: focal distance must be > 0");
        }
        myFocal = theFocal;
    }

    /// <summary>A copy with the focal distance set to theFocal.</summary>
    /// <exception cref="Standard_ConstructionError">theFocal &lt;= 0.</exception>
    public GeomEval_ParaboloidSurface WithFocal(double theFocal)
    {
        GeomEval_ParaboloidSurface aCopy = (GeomEval_ParaboloidSurface)Copy();
        aCopy.SetFocal(theFocal);
        return aCopy;
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_ParaboloidSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_ParaboloidSurface::VReverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double UReversedParameter(double U)
    {
        throw new Standard_NotImplemented("GeomEval_ParaboloidSurface::UReversedParameter");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double VReversedParameter(double V)
    {
        throw new Standard_NotImplemented("GeomEval_ParaboloidSurface::VReversedParameter");
    }

    /// <summary>U in [0, 2*Pi], V in [-Precision::Infinite(), Precision::Infinite()].</summary>
    public override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = 0.0;
        U2 = 2.0 * M_PI;
        V1 = -Precision.Infinite();
        V2 = Precision.Infinite();
    }

    /// <summary>True: the paraboloid is closed in U (period 2*Pi).</summary>
    public override bool IsUClosed() => true;

    public override bool IsVClosed() => false;

    /// <summary>True: the paraboloid is periodic in U (period 2*Pi).</summary>
    public override bool IsUPeriodic() => true;

    public override bool IsVPeriodic() => false;

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override Geom_Curve UIso(double U)
    {
        throw new Standard_NotImplemented("GeomEval_ParaboloidSurface::UIso");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override Geom_Curve VIso(double V)
    {
        throw new Standard_NotImplemented("GeomEval_ParaboloidSurface::VIso");
    }

    public override gp_Pnt EvalD0(double U, double V)
    {
        gp_Pnt anO = pos.Location();
        gp_Dir aXDir = pos.XDirection();
        gp_Dir aYDir = pos.YDirection();
        gp_Dir aZDir = pos.Direction();

        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        double aZComp = V * V / (4.0 * myFocal);

        return new gp_Pnt(anO.X() + V * aCosU * aXDir.X() + V * aSinU * aYDir.X() + aZComp * aZDir.X(),
                          anO.Y() + V * aCosU * aXDir.Y() + V * aSinU * aYDir.Y() + aZComp * aZDir.Y(),
                          anO.Z() + V * aCosU * aXDir.Z() + V * aSinU * aYDir.Z() + aZComp * aZDir.Z());
    }

    public override ResD1 EvalD1(double U, double V)
    {
        gp_Pnt anO = pos.Location();
        gp_Dir aXDir = pos.XDirection();
        gp_Dir aYDir = pos.YDirection();
        gp_Dir aZDir = pos.Direction();

        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        double aZComp = V * V / (4.0 * myFocal);
        double aVOver2F = V / (2.0 * myFocal);

        ResD1 aResult;

        // P = O + V*cosU*XD + V*sinU*YD + V^2/(4F)*ZD
        aResult.Point =
            new gp_Pnt(anO.X() + V * aCosU * aXDir.X() + V * aSinU * aYDir.X() + aZComp * aZDir.X(),
                       anO.Y() + V * aCosU * aXDir.Y() + V * aSinU * aYDir.Y() + aZComp * aZDir.Y(),
                       anO.Z() + V * aCosU * aXDir.Z() + V * aSinU * aYDir.Z() + aZComp * aZDir.Z());

        // dP/du = -V*sinU*XD + V*cosU*YD
        aResult.D1U = new gp_Vec(-V * aSinU * aXDir.X() + V * aCosU * aYDir.X(),
                                 -V * aSinU * aXDir.Y() + V * aCosU * aYDir.Y(),
                                 -V * aSinU * aXDir.Z() + V * aCosU * aYDir.Z());

        // dP/dv = cosU*XD + sinU*YD + V/(2F)*ZD
        aResult.D1V = new gp_Vec(aCosU * aXDir.X() + aSinU * aYDir.X() + aVOver2F * aZDir.X(),
                                 aCosU * aXDir.Y() + aSinU * aYDir.Y() + aVOver2F * aZDir.Y(),
                                 aCosU * aXDir.Z() + aSinU * aYDir.Z() + aVOver2F * aZDir.Z());

        return aResult;
    }

    public override ResD2 EvalD2(double U, double V)
    {
        gp_Pnt anO = pos.Location();
        gp_Dir aXDir = pos.XDirection();
        gp_Dir aYDir = pos.YDirection();
        gp_Dir aZDir = pos.Direction();

        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        double aZComp = V * V / (4.0 * myFocal);
        double aVOver2F = V / (2.0 * myFocal);
        double aInv2F = 1.0 / (2.0 * myFocal);

        ResD2 aResult;

        // P
        aResult.Point =
            new gp_Pnt(anO.X() + V * aCosU * aXDir.X() + V * aSinU * aYDir.X() + aZComp * aZDir.X(),
                       anO.Y() + V * aCosU * aXDir.Y() + V * aSinU * aYDir.Y() + aZComp * aZDir.Y(),
                       anO.Z() + V * aCosU * aXDir.Z() + V * aSinU * aYDir.Z() + aZComp * aZDir.Z());

        // dP/du
        aResult.D1U = new gp_Vec(-V * aSinU * aXDir.X() + V * aCosU * aYDir.X(),
                                 -V * aSinU * aXDir.Y() + V * aCosU * aYDir.Y(),
                                 -V * aSinU * aXDir.Z() + V * aCosU * aYDir.Z());

        // dP/dv
        aResult.D1V = new gp_Vec(aCosU * aXDir.X() + aSinU * aYDir.X() + aVOver2F * aZDir.X(),
                                 aCosU * aXDir.Y() + aSinU * aYDir.Y() + aVOver2F * aZDir.Y(),
                                 aCosU * aXDir.Z() + aSinU * aYDir.Z() + aVOver2F * aZDir.Z());

        // d2P/du2 = -V*cosU*XD - V*sinU*YD
        aResult.D2U = new gp_Vec(-V * aCosU * aXDir.X() - V * aSinU * aYDir.X(),
                                 -V * aCosU * aXDir.Y() - V * aSinU * aYDir.Y(),
                                 -V * aCosU * aXDir.Z() - V * aSinU * aYDir.Z());

        // d2P/dv2 = 1/(2F)*ZD
        aResult.D2V = new gp_Vec(aInv2F * aZDir.X(), aInv2F * aZDir.Y(), aInv2F * aZDir.Z());

        // d2P/dudv = -sinU*XD + cosU*YD
        aResult.D2UV = new gp_Vec(-aSinU * aXDir.X() + aCosU * aYDir.X(),
                                  -aSinU * aXDir.Y() + aCosU * aYDir.Y(),
                                  -aSinU * aXDir.Z() + aCosU * aYDir.Z());

        return aResult;
    }

    public override ResD3 EvalD3(double U, double V)
    {
        gp_Pnt anO = pos.Location();
        gp_Dir aXDir = pos.XDirection();
        gp_Dir aYDir = pos.YDirection();
        gp_Dir aZDir = pos.Direction();

        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        double aZComp = V * V / (4.0 * myFocal);
        double aVOver2F = V / (2.0 * myFocal);
        double aInv2F = 1.0 / (2.0 * myFocal);

        ResD3 aResult;

        // P
        aResult.Point =
            new gp_Pnt(anO.X() + V * aCosU * aXDir.X() + V * aSinU * aYDir.X() + aZComp * aZDir.X(),
                       anO.Y() + V * aCosU * aXDir.Y() + V * aSinU * aYDir.Y() + aZComp * aZDir.Y(),
                       anO.Z() + V * aCosU * aXDir.Z() + V * aSinU * aYDir.Z() + aZComp * aZDir.Z());

        // dP/du
        aResult.D1U = new gp_Vec(-V * aSinU * aXDir.X() + V * aCosU * aYDir.X(),
                                 -V * aSinU * aXDir.Y() + V * aCosU * aYDir.Y(),
                                 -V * aSinU * aXDir.Z() + V * aCosU * aYDir.Z());

        // dP/dv
        aResult.D1V = new gp_Vec(aCosU * aXDir.X() + aSinU * aYDir.X() + aVOver2F * aZDir.X(),
                                 aCosU * aXDir.Y() + aSinU * aYDir.Y() + aVOver2F * aZDir.Y(),
                                 aCosU * aXDir.Z() + aSinU * aYDir.Z() + aVOver2F * aZDir.Z());

        // d2P/du2
        aResult.D2U = new gp_Vec(-V * aCosU * aXDir.X() - V * aSinU * aYDir.X(),
                                 -V * aCosU * aXDir.Y() - V * aSinU * aYDir.Y(),
                                 -V * aCosU * aXDir.Z() - V * aSinU * aYDir.Z());

        // d2P/dv2
        aResult.D2V = new gp_Vec(aInv2F * aZDir.X(), aInv2F * aZDir.Y(), aInv2F * aZDir.Z());

        // d2P/dudv
        aResult.D2UV = new gp_Vec(-aSinU * aXDir.X() + aCosU * aYDir.X(),
                                  -aSinU * aXDir.Y() + aCosU * aYDir.Y(),
                                  -aSinU * aXDir.Z() + aCosU * aYDir.Z());

        // d3P/du3 = V*sinU*XD - V*cosU*YD
        aResult.D3U = new gp_Vec(V * aSinU * aXDir.X() - V * aCosU * aYDir.X(),
                                 V * aSinU * aXDir.Y() - V * aCosU * aYDir.Y(),
                                 V * aSinU * aXDir.Z() - V * aCosU * aYDir.Z());

        // d3P/dv3 = 0
        aResult.D3V = new gp_Vec(0.0, 0.0, 0.0);

        // d3P/du2dv = -cosU*XD - sinU*YD
        aResult.D3UUV = new gp_Vec(-aCosU * aXDir.X() - aSinU * aYDir.X(),
                                   -aCosU * aXDir.Y() - aSinU * aYDir.Y(),
                                   -aCosU * aXDir.Z() - aSinU * aYDir.Z());

        // d3P/dudv2 = 0
        aResult.D3UVV = new gp_Vec(0.0, 0.0, 0.0);

        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public override gp_Vec EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Geom_UndefinedDerivative("GeomEval_ParaboloidSurface::EvalDN: invalid derivative order");
        }

        gp_Dir aXDir = pos.XDirection();
        gp_Dir aYDir = pos.YDirection();
        gp_Dir aZDir = pos.Direction();

        // The parametrization is:
        //   P = O + v*cos(u)*XD + v*sin(u)*YD + v^2/(4F)*ZD
        //
        // XD component: cos(u + Nu*PI/2) * (Nv==0 ? v : (Nv==1 ? 1 : 0))
        // YD component: sin(u + Nu*PI/2) * (Nv==0 ? v : (Nv==1 ? 1 : 0))
        // ZD component: (Nu==0 ? 1 : 0) * d^Nv[v^2/(4F)]/dv^Nv
        //   where d^Nv[v^2/(4F)]/dv^Nv = (Nv==0 ? v^2/(4F) : (Nv==1 ? v/(2F) : (Nv==2 ? 1/(2F) : 0)))

        // XD and YD terms: derivative of v^1 with respect to v
        double aVDerivXY = 0.0;
        if (Nv == 0)
        {
            aVDerivXY = V;
        }
        else if (Nv == 1)
        {
            aVDerivXY = 1.0;
        }
        // else aVDerivXY = 0 for Nv >= 2

        // Trig derivatives: d^Nu[cos(u)]/du^Nu = cos(u + Nu*PI/2)
        double aPhase = U + Nu * M_PI_2;
        double aCosNu = Math.Cos(aPhase);
        double aSinNu = Math.Sin(aPhase);

        double aXCoeff = aCosNu * aVDerivXY;
        double aYCoeff = aSinNu * aVDerivXY;

        // ZD term: only present when Nu == 0
        double aZCoeff = 0.0;
        if (Nu == 0)
        {
            if (Nv == 0)
            {
                aZCoeff = V * V / (4.0 * myFocal);
            }
            else if (Nv == 1)
            {
                aZCoeff = V / (2.0 * myFocal);
            }
            else if (Nv == 2)
            {
                aZCoeff = 1.0 / (2.0 * myFocal);
            }
            // else 0 for Nv >= 3
        }

        return new gp_Vec(aXCoeff * aXDir.X() + aYCoeff * aYDir.X() + aZCoeff * aZDir.X(),
                          aXCoeff * aXDir.Y() + aYCoeff * aYDir.Y() + aZCoeff * aZDir.Y(),
                          aXCoeff * aXDir.Z() + aYCoeff * aYDir.Z() + aZCoeff * aZDir.Z());
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_ParaboloidSurface::Transform");
    }

    public override Geom_Geometry Copy() => new GeomEval_ParaboloidSurface(pos, myFocal);

    /// <summary>
    /// The coefficients of the implicit equation in the absolute Cartesian coordinate system:
    /// <c>A1*X^2 + A2*Y^2 + A3*Z^2 + 2*(B1*X*Y + B2*X*Z + B3*Y*Z) + 2*(C1*X + C2*Y + C3*Z) + D = 0</c>.
    /// </summary>
    public void Coefficients(out double A1, out double A2, out double A3,
                             out double B1, out double B2, out double B3,
                             out double C1, out double C2, out double C3, out double D)
    {
        // In the local coordinate system of the paraboloid:
        //   X_L^2 + Y_L^2 - 4*F*Z_L = 0
        //
        // Transform to global coordinates using:
        //   (X_L, Y_L, Z_L)^T = T * (x, y, z, 1)^T
        // where T is the transformation matrix from pos.
        gp_Trsf aT = new gp_Trsf();
        aT.SetTransformation(pos);
        double T11 = aT.Value(1, 1);
        double T12 = aT.Value(1, 2);
        double T13 = aT.Value(1, 3);
        double T14 = aT.Value(1, 4);
        double T21 = aT.Value(2, 1);
        double T22 = aT.Value(2, 2);
        double T23 = aT.Value(2, 3);
        double T24 = aT.Value(2, 4);
        double T31 = aT.Value(3, 1);
        double T32 = aT.Value(3, 2);
        double T33 = aT.Value(3, 3);
        double T34 = aT.Value(3, 4);

        // Expanding (T11*x + T12*y + T13*z + T14)^2 + (T21*x + T22*y + T23*z + T24)^2
        //   - 4*F*(T31*x + T32*y + T33*z + T34) = 0
        // and collecting terms:
        A1 = T11 * T11 + T21 * T21;
        A2 = T12 * T12 + T22 * T22;
        A3 = T13 * T13 + T23 * T23;
        B1 = T11 * T12 + T21 * T22;
        B2 = T11 * T13 + T21 * T23;
        B3 = T12 * T13 + T22 * T23;
        C1 = T11 * T14 + T21 * T24 - 2.0 * myFocal * T31;
        C2 = T12 * T14 + T22 * T24 - 2.0 * myFocal * T32;
        C3 = T13 * T14 + T23 * T24 - 2.0 * myFocal * T33;
        D = T14 * T14 + T24 * T24 - 4.0 * myFocal * T34;
    }
}
