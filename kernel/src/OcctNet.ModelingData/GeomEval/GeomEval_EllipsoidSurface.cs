// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_EllipsoidSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_EllipsoidSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A triaxial ellipsoid with semi-axes A, B, C (all &gt; 0), centred on the origin of its coordinate system:
/// <c>P(u,v) = O + A*cos(v)*cos(u)*XDir + B*cos(v)*sin(u)*YDir + C*sin(v)*ZDir</c>, u in [0, 2*Pi] (periodic),
/// v in [-Pi/2, Pi/2]. Implicit equation in local coordinates: <c>X^2/A^2 + Y^2/B^2 + Z^2/C^2 - 1 = 0</c>.
/// Reversal, transformation and iso curves are not supported (they throw <see cref="Standard_NotImplemented"/>).
/// </summary>
public sealed class GeomEval_EllipsoidSurface : Geom_ElementarySurface
{
    private double myA; // Semi-axis along XDir
    private double myB; // Semi-axis along YDir
    private double myC; // Semi-axis along ZDir

    /// <summary>Creates a triaxial ellipsoid with the given local coordinate system and semi-axes.</summary>
    /// <exception cref="Standard_ConstructionError">A semi-axis is &lt;= 0.</exception>
    public GeomEval_EllipsoidSurface(in gp_Ax3 thePosition, double theA, double theB, double theC)
    {
        myA = theA;
        myB = theB;
        myC = theC;
        if (theA <= 0.0 || theB <= 0.0 || theC <= 0.0)
        {
            throw new Standard_ConstructionError("GeomEval_EllipsoidSurface: semi-axis must be > 0");
        }
        pos = thePosition;
    }

    /// <summary>The semi-axis A (along XDir).</summary>
    public double SemiAxisA() => myA;

    /// <summary>The semi-axis B (along YDir).</summary>
    public double SemiAxisB() => myB;

    /// <summary>The semi-axis C (along ZDir).</summary>
    public double SemiAxisC() => myC;

    /// <exception cref="Standard_ConstructionError">theA &lt;= 0.</exception>
    internal void SetSemiAxisA(double theA)
    {
        CheckMutable();
        if (theA <= 0.0)
        {
            throw new Standard_ConstructionError("GeomEval_EllipsoidSurface::SetSemiAxisA: value must be > 0");
        }
        myA = theA;
    }

    /// <exception cref="Standard_ConstructionError">theB &lt;= 0.</exception>
    internal void SetSemiAxisB(double theB)
    {
        CheckMutable();
        if (theB <= 0.0)
        {
            throw new Standard_ConstructionError("GeomEval_EllipsoidSurface::SetSemiAxisB: value must be > 0");
        }
        myB = theB;
    }

    /// <exception cref="Standard_ConstructionError">theC &lt;= 0.</exception>
    internal void SetSemiAxisC(double theC)
    {
        CheckMutable();
        if (theC <= 0.0)
        {
            throw new Standard_ConstructionError("GeomEval_EllipsoidSurface::SetSemiAxisC: value must be > 0");
        }
        myC = theC;
    }

    /// <summary>A copy with the semi-axis A set to theA.</summary>
    /// <exception cref="Standard_ConstructionError">theA &lt;= 0.</exception>
    public GeomEval_EllipsoidSurface WithSemiAxisA(double theA)
    {
        GeomEval_EllipsoidSurface aCopy = (GeomEval_EllipsoidSurface)Copy();
        aCopy.SetSemiAxisA(theA);
        return aCopy;
    }

    /// <summary>A copy with the semi-axis B set to theB.</summary>
    /// <exception cref="Standard_ConstructionError">theB &lt;= 0.</exception>
    public GeomEval_EllipsoidSurface WithSemiAxisB(double theB)
    {
        GeomEval_EllipsoidSurface aCopy = (GeomEval_EllipsoidSurface)Copy();
        aCopy.SetSemiAxisB(theB);
        return aCopy;
    }

    /// <summary>A copy with the semi-axis C set to theC.</summary>
    /// <exception cref="Standard_ConstructionError">theC &lt;= 0.</exception>
    public GeomEval_EllipsoidSurface WithSemiAxisC(double theC)
    {
        GeomEval_EllipsoidSurface aCopy = (GeomEval_EllipsoidSurface)Copy();
        aCopy.SetSemiAxisC(theC);
        return aCopy;
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_EllipsoidSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_EllipsoidSurface::VReverse");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double UReversedParameter(double U)
    {
        throw new Standard_NotImplemented("GeomEval_EllipsoidSurface::UReversedParameter");
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    public override double VReversedParameter(double V)
    {
        throw new Standard_NotImplemented("GeomEval_EllipsoidSurface::VReversedParameter");
    }

    /// <summary>U in [0, 2*Pi], V in [-Pi/2, Pi/2].</summary>
    public override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = 0.0;
        U2 = 2.0 * M_PI;
        V1 = -M_PI / 2.0;
        V2 = M_PI / 2.0;
    }

    /// <summary>True: the ellipsoid is closed in U (period 2*Pi).</summary>
    public override bool IsUClosed() => true;

    public override bool IsVClosed() => false;

    /// <summary>True: the ellipsoid is periodic in U (period 2*Pi).</summary>
    public override bool IsUPeriodic() => true;

    public override bool IsVPeriodic() => false;

    /// <summary>Not supported (not a standard curve type): throws <see cref="Standard_NotImplemented"/>.</summary>
    public override Geom_Curve UIso(double U)
    {
        throw new Standard_NotImplemented("GeomEval_EllipsoidSurface::UIso");
    }

    /// <summary>Not supported (an ellipse only when A == B): throws <see cref="Standard_NotImplemented"/>.</summary>
    public override Geom_Curve VIso(double V)
    {
        throw new Standard_NotImplemented("GeomEval_EllipsoidSurface::VIso");
    }

    public override gp_Pnt EvalD0(double U, double V)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        double aCosV = Math.Cos(V);
        double aSinV = Math.Sin(V);

        gp_XYZ anO = pos.Location().XYZ();
        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        return new gp_Pnt(anO + myA * aCosV * aCosU * aXD + myB * aCosV * aSinU * aYD + myC * aSinV * aZD);
    }

    public override ResD1 EvalD1(double U, double V)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        double aCosV = Math.Cos(V);
        double aSinV = Math.Sin(V);

        gp_XYZ anO = pos.Location().XYZ();
        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        ResD1 aResult;
        aResult.Point =
            new gp_Pnt(anO + myA * aCosV * aCosU * aXD + myB * aCosV * aSinU * aYD + myC * aSinV * aZD);

        // dP/du = -A*cv*su*XD + B*cv*cu*YD
        aResult.D1U = new gp_Vec(myA * aCosV * (-aSinU) * aXD + myB * aCosV * aCosU * aYD);

        // dP/dv = -A*sv*cu*XD - B*sv*su*YD + C*cv*ZD
        aResult.D1V =
            new gp_Vec(myA * (-aSinV) * aCosU * aXD + myB * (-aSinV) * aSinU * aYD + myC * aCosV * aZD);

        return aResult;
    }

    public override ResD2 EvalD2(double U, double V)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        double aCosV = Math.Cos(V);
        double aSinV = Math.Sin(V);

        gp_XYZ anO = pos.Location().XYZ();
        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        ResD2 aResult;
        aResult.Point =
            new gp_Pnt(anO + myA * aCosV * aCosU * aXD + myB * aCosV * aSinU * aYD + myC * aSinV * aZD);

        // dP/du
        aResult.D1U = new gp_Vec(myA * aCosV * (-aSinU) * aXD + myB * aCosV * aCosU * aYD);

        // dP/dv
        aResult.D1V =
            new gp_Vec(myA * (-aSinV) * aCosU * aXD + myB * (-aSinV) * aSinU * aYD + myC * aCosV * aZD);

        // d2P/du2 = -A*cv*cu*XD - B*cv*su*YD
        aResult.D2U = new gp_Vec(myA * aCosV * (-aCosU) * aXD + myB * aCosV * (-aSinU) * aYD);

        // d2P/dv2 = -A*cv*cu*XD - B*cv*su*YD - C*sv*ZD
        aResult.D2V =
            new gp_Vec(myA * (-aCosV) * aCosU * aXD + myB * (-aCosV) * aSinU * aYD + myC * (-aSinV) * aZD);

        // d2P/dudv = A*sv*su*XD - B*sv*cu*YD
        aResult.D2UV = new gp_Vec(myA * aSinV * aSinU * aXD + myB * (-aSinV) * aCosU * aYD);

        return aResult;
    }

    public override ResD3 EvalD3(double U, double V)
    {
        double aCosU = Math.Cos(U);
        double aSinU = Math.Sin(U);
        double aCosV = Math.Cos(V);
        double aSinV = Math.Sin(V);

        gp_XYZ anO = pos.Location().XYZ();
        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        ResD3 aResult;

        // P
        aResult.Point =
            new gp_Pnt(anO + myA * aCosV * aCosU * aXD + myB * aCosV * aSinU * aYD + myC * aSinV * aZD);

        // dP/du
        aResult.D1U = new gp_Vec(myA * aCosV * (-aSinU) * aXD + myB * aCosV * aCosU * aYD);

        // dP/dv
        aResult.D1V =
            new gp_Vec(myA * (-aSinV) * aCosU * aXD + myB * (-aSinV) * aSinU * aYD + myC * aCosV * aZD);

        // d2P/du2
        aResult.D2U = new gp_Vec(myA * aCosV * (-aCosU) * aXD + myB * aCosV * (-aSinU) * aYD);

        // d2P/dv2
        aResult.D2V =
            new gp_Vec(myA * (-aCosV) * aCosU * aXD + myB * (-aCosV) * aSinU * aYD + myC * (-aSinV) * aZD);

        // d2P/dudv
        aResult.D2UV = new gp_Vec(myA * aSinV * aSinU * aXD + myB * (-aSinV) * aCosU * aYD);

        // d3P/du3 = A*cv*su*XD - B*cv*cu*YD
        aResult.D3U = new gp_Vec(myA * aCosV * aSinU * aXD + myB * aCosV * (-aCosU) * aYD);

        // d3P/dv3 = A*sv*cu*XD + B*sv*su*YD - C*cv*ZD
        aResult.D3V =
            new gp_Vec(myA * aSinV * aCosU * aXD + myB * aSinV * aSinU * aYD + myC * (-aCosV) * aZD);

        // d3P/du2dv = A*sv*cu*XD + B*sv*su*YD
        aResult.D3UUV = new gp_Vec(myA * aSinV * aCosU * aXD + myB * aSinV * aSinU * aYD);

        // d3P/dudv2 = A*cv*su*XD - B*cv*cu*YD
        aResult.D3UVV = new gp_Vec(myA * aCosV * aSinU * aXD + myB * (-aCosV) * aCosU * aYD);

        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public override gp_Vec EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Geom_UndefinedDerivative("GeomEval_EllipsoidSurface::EvalDN: invalid derivative order");
        }

        gp_XYZ aXD = pos.XDirection().XYZ();
        gp_XYZ aYD = pos.YDirection().XYZ();
        gp_XYZ aZD = pos.Direction().XYZ();

        // Using phase-shift identities:
        // d^n/du^n[cos(u)] = cos(u + n*Pi/2)
        // d^n/du^n[sin(u)] = sin(u + n*Pi/2)
        double aPhaseU = Nu * M_PI / 2.0;
        double aPhaseV = Nv * M_PI / 2.0;

        // XD coefficient: A * d^Nv[cos(v)] * d^Nu[cos(u)]
        double aCoeffX = myA * Math.Cos(V + aPhaseV) * Math.Cos(U + aPhaseU);

        // YD coefficient: B * d^Nv[cos(v)] * d^Nu[sin(u)]
        double aCoeffY = myB * Math.Cos(V + aPhaseV) * Math.Sin(U + aPhaseU);

        // ZD coefficient: C * d^Nv[sin(v)] * (Nu == 0 ? 1 : 0)
        double aCoeffZ = myC * Math.Sin(V + aPhaseV) * (Nu == 0 ? 1.0 : 0.0);

        return new gp_Vec(aCoeffX * aXD + aCoeffY * aYD + aCoeffZ * aZD);
    }

    /// <summary>Not supported: throws <see cref="Standard_NotImplemented"/>.</summary>
    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        throw new Standard_NotImplemented("GeomEval_EllipsoidSurface::Transform");
    }

    public override Geom_Geometry Copy() => new GeomEval_EllipsoidSurface(pos, myA, myB, myC);

    /// <summary>
    /// The coefficients of the implicit equation in the absolute Cartesian coordinate system:
    /// <c>A1*X^2 + A2*Y^2 + A3*Z^2 + 2*(B1*X*Y + B2*X*Z + B3*Y*Z) + 2*(C1*X + C2*Y + C3*Z) + D = 0</c>.
    /// </summary>
    public void Coefficients(out double A1, out double A2, out double A3,
                             out double B1, out double B2, out double B3,
                             out double C1, out double C2, out double C3, out double D)
    {
        // In the local coordinate system of the ellipsoid:
        // X^2/myA^2 + Y^2/myB^2 + Z^2/myC^2 - 1 = 0
        //
        // i.e. local diagonal coefficients are:
        //   a1 = 1/A^2, a2 = 1/B^2, a3 = 1/C^2, d = -1
        // and all cross/linear terms are zero.
        //
        // We transform from local to global using gp_Trsf from pos,
        // substituting X_local = T11*X + T12*Y + T13*Z + T14, etc.

        double aInvA2 = 1.0 / (myA * myA);
        double aInvB2 = 1.0 / (myB * myB);
        double aInvC2 = 1.0 / (myC * myC);

        gp_Trsf T = new gp_Trsf();
        T.SetTransformation(pos);
        double T11 = T.Value(1, 1);
        double T12 = T.Value(1, 2);
        double T13 = T.Value(1, 3);
        double T14 = T.Value(1, 4);
        double T21 = T.Value(2, 1);
        double T22 = T.Value(2, 2);
        double T23 = T.Value(2, 3);
        double T24 = T.Value(2, 4);
        double T31 = T.Value(3, 1);
        double T32 = T.Value(3, 2);
        double T33 = T.Value(3, 3);
        double T34 = T.Value(3, 4);

        // A1 = a1*T11^2 + a2*T21^2 + a3*T31^2
        A1 = aInvA2 * T11 * T11 + aInvB2 * T21 * T21 + aInvC2 * T31 * T31;

        // A2 = a1*T12^2 + a2*T22^2 + a3*T32^2
        A2 = aInvA2 * T12 * T12 + aInvB2 * T22 * T22 + aInvC2 * T32 * T32;

        // A3 = a1*T13^2 + a2*T23^2 + a3*T33^2
        A3 = aInvA2 * T13 * T13 + aInvB2 * T23 * T23 + aInvC2 * T33 * T33;

        // B1 = a1*T11*T12 + a2*T21*T22 + a3*T31*T32
        B1 = aInvA2 * T11 * T12 + aInvB2 * T21 * T22 + aInvC2 * T31 * T32;

        // B2 = a1*T11*T13 + a2*T21*T23 + a3*T31*T33
        B2 = aInvA2 * T11 * T13 + aInvB2 * T21 * T23 + aInvC2 * T31 * T33;

        // B3 = a1*T12*T13 + a2*T22*T23 + a3*T32*T33
        B3 = aInvA2 * T12 * T13 + aInvB2 * T22 * T23 + aInvC2 * T32 * T33;

        // C1 = a1*T11*T14 + a2*T21*T24 + a3*T31*T34
        C1 = aInvA2 * T11 * T14 + aInvB2 * T21 * T24 + aInvC2 * T31 * T34;

        // C2 = a1*T12*T14 + a2*T22*T24 + a3*T32*T34
        C2 = aInvA2 * T12 * T14 + aInvB2 * T22 * T24 + aInvC2 * T32 * T34;

        // C3 = a1*T13*T14 + a2*T23*T24 + a3*T33*T34
        C3 = aInvA2 * T13 * T14 + aInvB2 * T23 * T24 + aInvC2 * T33 * T34;

        // D = a1*T14^2 + a2*T24^2 + a3*T34^2 - 1
        D = aInvA2 * T14 * T14 + aInvB2 * T24 * T24 + aInvC2 * T34 * T34 - 1.0;
    }
}
