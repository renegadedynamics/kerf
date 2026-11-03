// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_HypParaboloidSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomEval/GeomEval_HypParaboloidSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A hyperbolic paraboloid (saddle) with semi-axis lengths A, B &gt; 0, its saddle point at the origin of its
/// coordinate system: <c>P(u,v) = O + u*XDir + v*YDir + (u^2/A^2 - v^2/B^2)*ZDir</c>, u and v in (-inf, +inf).
/// Doubly ruled, neither periodic nor closed. Implicit equation in local coordinates:
/// <c>X^2/A^2 - Y^2/B^2 - Z = 0</c>. Reversal, transformation and iso curves are not supported (they throw
/// <see cref="MethodNotImplementedException"/>).
/// </summary>
public sealed class GeomEval_HypParaboloidSurface : Geom_ElementarySurface
{
    private double myA; // Semi-axis length A (> 0)
    private double myB; // Semi-axis length B (> 0)

    /// <summary>Creates a hyperbolic paraboloid with the given local coordinate system and semi-axes.</summary>
    /// <exception cref="ConstructionErrorException">theA &lt;= 0 or theB &lt;= 0.</exception>
    public GeomEval_HypParaboloidSurface(in CoordinateSystem thePosition, double theA, double theB)
    {
        myA = theA;
        myB = theB;
        if (theA <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_HypParaboloidSurface: semi-axis A must be > 0");
        }
        if (theB <= 0.0)
        {
            throw new ConstructionErrorException("GeomEval_HypParaboloidSurface: semi-axis B must be > 0");
        }
        pos = thePosition;
    }

    /// <summary>The semi-axis length A.</summary>
    public double SemiAxisA() => myA;

    /// <summary>The semi-axis length B.</summary>
    public double SemiAxisB() => myB;

    /// <exception cref="ConstructionErrorException">theA &lt;= 0.</exception>
    internal void SetSemiAxisA(double theA)
    {
        CheckMutable();
        if (theA <= 0.0)
        {
            throw new ConstructionErrorException(
                "GeomEval_HypParaboloidSurface::SetSemiAxisA: semi-axis A must be > 0");
        }
        myA = theA;
    }

    /// <exception cref="ConstructionErrorException">theB &lt;= 0.</exception>
    internal void SetSemiAxisB(double theB)
    {
        CheckMutable();
        if (theB <= 0.0)
        {
            throw new ConstructionErrorException(
                "GeomEval_HypParaboloidSurface::SetSemiAxisB: semi-axis B must be > 0");
        }
        myB = theB;
    }

    /// <summary>A copy with the semi-axis A set to theA.</summary>
    /// <exception cref="ConstructionErrorException">theA &lt;= 0.</exception>
    public GeomEval_HypParaboloidSurface WithSemiAxisA(double theA)
    {
        GeomEval_HypParaboloidSurface aCopy = (GeomEval_HypParaboloidSurface)Copy();
        aCopy.SetSemiAxisA(theA);
        return aCopy;
    }

    /// <summary>A copy with the semi-axis B set to theB.</summary>
    /// <exception cref="ConstructionErrorException">theB &lt;= 0.</exception>
    public GeomEval_HypParaboloidSurface WithSemiAxisB(double theB)
    {
        GeomEval_HypParaboloidSurface aCopy = (GeomEval_HypParaboloidSurface)Copy();
        aCopy.SetSemiAxisB(theB);
        return aCopy;
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void UReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::UReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void VReverse()
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::VReverse");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double UReversedParameter(double U)
    {
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::UReversedParameter");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double VReversedParameter(double V)
    {
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::VReversedParameter");
    }

    /// <summary>U and V in [-Precision::Infinite(), Precision::Infinite()].</summary>
    public override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        U2 = global::Kerf.Engine.Foundation.Precision.Infinite();
        V1 = -global::Kerf.Engine.Foundation.Precision.Infinite();
        V2 = global::Kerf.Engine.Foundation.Precision.Infinite();
    }

    public override bool IsUClosed() => false;

    public override bool IsVClosed() => false;

    public override bool IsUPeriodic() => false;

    public override bool IsVPeriodic() => false;

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Geom_Curve UIso(double U)
    {
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::UIso");
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override Geom_Curve VIso(double V)
    {
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::VIso");
    }

    public override Point3D EvalD0(double U, double V)
    {
        Point3D anO = pos.Location();
        Direction3D aXDir = pos.XDirection();
        Direction3D aYDir = pos.YDirection();
        Direction3D aZDir = pos.Direction();

        double aA2 = myA * myA;
        double aB2 = myB * myB;
        double aZComp = U * U / aA2 - V * V / aB2;

        return new Point3D(anO.X() + U * aXDir.X() + V * aYDir.X() + aZComp * aZDir.X(),
                          anO.Y() + U * aXDir.Y() + V * aYDir.Y() + aZComp * aZDir.Y(),
                          anO.Z() + U * aXDir.Z() + V * aYDir.Z() + aZComp * aZDir.Z());
    }

    public override ResD1 EvalD1(double U, double V)
    {
        Point3D anO = pos.Location();
        Direction3D aXDir = pos.XDirection();
        Direction3D aYDir = pos.YDirection();
        Direction3D aZDir = pos.Direction();

        double aA2 = myA * myA;
        double aB2 = myB * myB;
        double aZComp = U * U / aA2 - V * V / aB2;
        double aZdU = 2.0 * U / aA2;
        double aZdV = -2.0 * V / aB2;

        ResD1 aResult;

        // P = O + U*XD + V*YD + (U^2/A^2 - V^2/B^2)*ZD
        aResult.Point = new Point3D(anO.X() + U * aXDir.X() + V * aYDir.X() + aZComp * aZDir.X(),
                                   anO.Y() + U * aXDir.Y() + V * aYDir.Y() + aZComp * aZDir.Y(),
                                   anO.Z() + U * aXDir.Z() + V * aYDir.Z() + aZComp * aZDir.Z());

        // dP/du = XD + (2U/A^2)*ZD
        aResult.D1U = new Vector3D(aXDir.X() + aZdU * aZDir.X(),
                                 aXDir.Y() + aZdU * aZDir.Y(),
                                 aXDir.Z() + aZdU * aZDir.Z());

        // dP/dv = YD - (2V/B^2)*ZD
        aResult.D1V = new Vector3D(aYDir.X() + aZdV * aZDir.X(),
                                 aYDir.Y() + aZdV * aZDir.Y(),
                                 aYDir.Z() + aZdV * aZDir.Z());

        return aResult;
    }

    public override ResD2 EvalD2(double U, double V)
    {
        Point3D anO = pos.Location();
        Direction3D aXDir = pos.XDirection();
        Direction3D aYDir = pos.YDirection();
        Direction3D aZDir = pos.Direction();

        double aA2 = myA * myA;
        double aB2 = myB * myB;
        double aZComp = U * U / aA2 - V * V / aB2;
        double aZdU = 2.0 * U / aA2;
        double aZdV = -2.0 * V / aB2;
        double aZd2U = 2.0 / aA2;
        double aZd2V = -2.0 / aB2;

        ResD2 aResult;

        // P
        aResult.Point = new Point3D(anO.X() + U * aXDir.X() + V * aYDir.X() + aZComp * aZDir.X(),
                                   anO.Y() + U * aXDir.Y() + V * aYDir.Y() + aZComp * aZDir.Y(),
                                   anO.Z() + U * aXDir.Z() + V * aYDir.Z() + aZComp * aZDir.Z());

        // dP/du = XD + (2U/A^2)*ZD
        aResult.D1U = new Vector3D(aXDir.X() + aZdU * aZDir.X(),
                                 aXDir.Y() + aZdU * aZDir.Y(),
                                 aXDir.Z() + aZdU * aZDir.Z());

        // dP/dv = YD - (2V/B^2)*ZD
        aResult.D1V = new Vector3D(aYDir.X() + aZdV * aZDir.X(),
                                 aYDir.Y() + aZdV * aZDir.Y(),
                                 aYDir.Z() + aZdV * aZDir.Z());

        // d2P/du2 = (2/A^2)*ZD
        aResult.D2U = new Vector3D(aZd2U * aZDir.X(), aZd2U * aZDir.Y(), aZd2U * aZDir.Z());

        // d2P/dv2 = -(2/B^2)*ZD
        aResult.D2V = new Vector3D(aZd2V * aZDir.X(), aZd2V * aZDir.Y(), aZd2V * aZDir.Z());

        // d2P/dudv = 0
        aResult.D2UV = new Vector3D(0.0, 0.0, 0.0);

        return aResult;
    }

    public override ResD3 EvalD3(double U, double V)
    {
        Point3D anO = pos.Location();
        Direction3D aXDir = pos.XDirection();
        Direction3D aYDir = pos.YDirection();
        Direction3D aZDir = pos.Direction();

        double aA2 = myA * myA;
        double aB2 = myB * myB;
        double aZComp = U * U / aA2 - V * V / aB2;
        double aZdU = 2.0 * U / aA2;
        double aZdV = -2.0 * V / aB2;
        double aZd2U = 2.0 / aA2;
        double aZd2V = -2.0 / aB2;

        ResD3 aResult;

        // P
        aResult.Point = new Point3D(anO.X() + U * aXDir.X() + V * aYDir.X() + aZComp * aZDir.X(),
                                   anO.Y() + U * aXDir.Y() + V * aYDir.Y() + aZComp * aZDir.Y(),
                                   anO.Z() + U * aXDir.Z() + V * aYDir.Z() + aZComp * aZDir.Z());

        // dP/du
        aResult.D1U = new Vector3D(aXDir.X() + aZdU * aZDir.X(),
                                 aXDir.Y() + aZdU * aZDir.Y(),
                                 aXDir.Z() + aZdU * aZDir.Z());

        // dP/dv
        aResult.D1V = new Vector3D(aYDir.X() + aZdV * aZDir.X(),
                                 aYDir.Y() + aZdV * aZDir.Y(),
                                 aYDir.Z() + aZdV * aZDir.Z());

        // d2P/du2 = (2/A^2)*ZD
        aResult.D2U = new Vector3D(aZd2U * aZDir.X(), aZd2U * aZDir.Y(), aZd2U * aZDir.Z());

        // d2P/dv2 = -(2/B^2)*ZD
        aResult.D2V = new Vector3D(aZd2V * aZDir.X(), aZd2V * aZDir.Y(), aZd2V * aZDir.Z());

        // d2P/dudv = 0
        aResult.D2UV = new Vector3D(0.0, 0.0, 0.0);

        // All 3rd derivatives are zero (polynomial of degree 2)
        aResult.D3U = new Vector3D(0.0, 0.0, 0.0);
        aResult.D3V = new Vector3D(0.0, 0.0, 0.0);
        aResult.D3UUV = new Vector3D(0.0, 0.0, 0.0);
        aResult.D3UVV = new Vector3D(0.0, 0.0, 0.0);

        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public override Vector3D EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Geom_UndefinedDerivative(
                "GeomEval_HypParaboloidSurface::EvalDN: invalid derivative order");
        }

        Direction3D aXDir = pos.XDirection();
        Direction3D aYDir = pos.YDirection();
        Direction3D aZDir = pos.Direction();

        double aA2 = myA * myA;
        double aB2 = myB * myB;

        // The parametrization is:
        //   P = O + u*XD + v*YD + (u^2/A^2 - v^2/B^2)*ZD
        //
        // XD coeff = (Nu==0 ? u : (Nu==1 ? 1 : 0)) * (Nv==0 ? 1 : 0)
        // YD coeff = (Nu==0 ? 1 : 0) * (Nv==0 ? v : (Nv==1 ? 1 : 0))
        // ZD coeff = (Nv==0 ? d^Nu[u^2/A^2] : 0) + (Nu==0 ? d^Nv[-v^2/B^2] : 0)
        double aXCoeff = (Nv == 0 ? (Nu == 0 ? U : (Nu == 1 ? 1.0 : 0.0)) : 0.0);
        double aYCoeff = (Nu == 0 ? (Nv == 0 ? V : (Nv == 1 ? 1.0 : 0.0)) : 0.0);

        // ZD coefficient: sum of pure-u and pure-v parts
        double aZCoeff = 0.0;

        // d^Nu[u^2/A^2] contribution (only when Nv == 0)
        if (Nv == 0)
        {
            if (Nu == 0)
            {
                aZCoeff += U * U / aA2;
            }
            else if (Nu == 1)
            {
                aZCoeff += 2.0 * U / aA2;
            }
            else if (Nu == 2)
            {
                aZCoeff += 2.0 / aA2;
            }
            // else 0 for Nu >= 3
        }

        // d^Nv[-v^2/B^2] contribution (only when Nu == 0)
        if (Nu == 0)
        {
            if (Nv == 0)
            {
                aZCoeff += -V * V / aB2;
            }
            else if (Nv == 1)
            {
                aZCoeff += -2.0 * V / aB2;
            }
            else if (Nv == 2)
            {
                aZCoeff += -2.0 / aB2;
            }
            // else 0 for Nv >= 3
        }

        return new Vector3D(aXCoeff * aXDir.X() + aYCoeff * aYDir.X() + aZCoeff * aZDir.X(),
                          aXCoeff * aXDir.Y() + aYCoeff * aYDir.Y() + aZCoeff * aZDir.Y(),
                          aXCoeff * aXDir.Z() + aYCoeff * aYDir.Z() + aZCoeff * aZDir.Z());
    }

    /// <summary>Not supported: throws <see cref="MethodNotImplementedException"/>.</summary>
    protected internal override void Transform(in Transform3D T)
    {
        CheckMutable();
        throw new MethodNotImplementedException("GeomEval_HypParaboloidSurface::Transform");
    }

    public override Geom_Geometry Copy() => new GeomEval_HypParaboloidSurface(pos, myA, myB);

    /// <summary>
    /// The coefficients of the implicit equation in the absolute Cartesian coordinate system:
    /// <c>A1*X^2 + A2*Y^2 + A3*Z^2 + 2*(B1*X*Y + B2*X*Z + B3*Y*Z) + 2*(C1*X + C2*Y + C3*Z) + D = 0</c>.
    /// </summary>
    public void Coefficients(out double A1, out double A2, out double A3,
                             out double B1, out double B2, out double B3,
                             out double C1, out double C2, out double C3, out double D)
    {
        // In the local coordinate system of the hyperbolic paraboloid:
        //   X_L^2/A^2 - Y_L^2/B^2 - Z_L = 0
        //
        // Transform to global coordinates using:
        //   (X_L, Y_L, Z_L)^T = T * (x, y, z, 1)^T
        // where T is the transformation matrix from pos.
        Transform3D aT = new Transform3D();
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

        double aA2 = myA * myA;
        double aB2 = myB * myB;

        // Expanding (1/A^2)*(T1i*xi+T14)^2 - (1/B^2)*(T2i*xi+T24)^2 - (T3i*xi+T34) = 0
        // and collecting terms:
        A1 = T11 * T11 / aA2 - T21 * T21 / aB2;
        A2 = T12 * T12 / aA2 - T22 * T22 / aB2;
        A3 = T13 * T13 / aA2 - T23 * T23 / aB2;
        B1 = T11 * T12 / aA2 - T21 * T22 / aB2;
        B2 = T11 * T13 / aA2 - T21 * T23 / aB2;
        B3 = T12 * T13 / aA2 - T22 * T23 / aB2;
        C1 = T11 * T14 / aA2 - T21 * T24 / aB2 - 0.5 * T31;
        C2 = T12 * T14 / aA2 - T22 * T24 / aB2 - 0.5 * T32;
        C3 = T13 * T14 / aA2 - T23 * T24 / aB2 - 0.5 * T33;
        D = T14 * T14 / aA2 - T24 * T24 / aB2 - T34;
    }
}
