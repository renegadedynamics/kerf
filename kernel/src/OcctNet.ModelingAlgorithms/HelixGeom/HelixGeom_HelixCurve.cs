// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_HelixCurve.hxx
// Port of: src/ModelingAlgorithms/TKHelix/HelixGeom/HelixGeom_HelixCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// An adaptor evaluating a helix analytically: a cylindrical helix (constant radius) or a tapered one (the radius
/// changing with a taper angle), clockwise or counter-clockwise, about the Z axis.
/// </summary>
/// <remarks>
/// In cylindrical coordinates: x(t) = r(t) cos t, y(t) = r(t) sin t (negated when counter-clockwise),
/// z(t) = pitch t / 2π, with r(t) = rStart + pitch tan(taper) t / 2π. An adaptor is a working object (decision 21).
/// </remarks>
public class HelixGeom_HelixCurve : Adaptor3d_Curve
{
    protected double myFirst;
    protected double myLast;
    protected double myPitch;
    protected double myRStart;
    protected double myTaperAngle;
    protected bool myIsClockWise;
    protected double myC1;
    protected double myTgBeta;
    protected double myTolAngle;

    /// <summary>A helix of one turn, pitch 1 and radius 1, clockwise.</summary>
    public HelixGeom_HelixCurve()
    {
        myFirst = 0.0;
        myLast = 2.0 * M_PI;
        myPitch = 1.0;
        myRStart = 1.0;
        myTaperAngle = 0.0;
        myIsClockWise = true;
        // Calculate derived parameters
        myC1 = myPitch / myLast;
        myTgBeta = 0.0;
        myTolAngle = 1.0e-4;
    }

    /// <summary>A copy of <paramref name="theOther"/> (OCCT's implicit copy constructor).</summary>
    public HelixGeom_HelixCurve(HelixGeom_HelixCurve theOther)
    {
        myFirst = theOther.myFirst;
        myLast = theOther.myLast;
        myPitch = theOther.myPitch;
        myRStart = theOther.myRStart;
        myTaperAngle = theOther.myTaperAngle;
        myIsClockWise = theOther.myIsClockWise;
        myC1 = theOther.myC1;
        myTgBeta = theOther.myTgBeta;
        myTolAngle = theOther.myTolAngle;
    }

    /// <summary>Reloads the current parameters (validating them and recomputing the derived values).</summary>
    public void Load()
    {
        Load(myFirst, myLast, myPitch, myRStart, myTaperAngle, myIsClockWise);
    }

    /// <summary>Sets the helix parameters.</summary>
    /// <exception cref="ConstructionErrorException">
    /// <paramref name="aT1"/> ≥ <paramref name="aT2"/>, a negative pitch or radius, or a taper angle outside (-π/2, π/2).
    /// </exception>
    public void Load(double aT1, double aT2, double aPitch, double aRStart, double aTaperAngle, bool aIsCW)
    {
        const string buf = "HelixGeom_HelixCurve::Load";
        double aTwoPI, aHalfPI;
        // Define angular constants
        aTwoPI = 2.0 * M_PI;
        aHalfPI = 0.5 * M_PI;
        // Store parameter values
        myFirst = aT1;
        myLast = aT2;
        myPitch = aPitch;
        myRStart = aRStart;
        myTaperAngle = aTaperAngle;
        myIsClockWise = aIsCW;
        // Validate input parameters
        if (aT1 >= aT2)
        {
            throw new ConstructionErrorException(buf);
        }
        if (myPitch < 0.0)
        {
            throw new ConstructionErrorException(buf);
        }
        if (myRStart < 0.0)
        {
            throw new ConstructionErrorException(buf);
        }
        if (myTaperAngle <= -aHalfPI || myTaperAngle >= aHalfPI)
        {
            throw new ConstructionErrorException(buf);
        }
        // Calculate helix coefficient
        myC1 = myPitch / aTwoPI;
        // OCCT: myTgBeta isn't reset for a (near) zero taper angle, so reloading a tapered helix as a cylindrical
        // one keeps the old taper (occt-issues.md).
        if (Math.Abs(myTaperAngle) > myTolAngle)
        {
            myTgBeta = Math.Tan(myTaperAngle);
        }
    }

    public override double FirstParameter() => myFirst;

    public override double LastParameter() => myLast;

    /// <summary>GeomAbs_CN.</summary>
    public override Continuity Continuity() => global::Kerf.Geometry.Continuity.CN;

    /// <summary>1, whatever the continuity.</summary>
    public override int NbIntervals(Continuity S) => 1;

    /// <summary>Stores the first and last parameters in T(1) and T(2).</summary>
    public override void Intervals(NCollection_Array1<double> T, Continuity S)
    {
        T[1] = myFirst;
        T[2] = myLast;
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double Resolution(double R3d) => throw new MethodNotImplementedException("HelixGeom_HelixCurve::Resolution");

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override bool IsClosed() => throw new MethodNotImplementedException("HelixGeom_HelixCurve::IsClosed");

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override bool IsPeriodic() => throw new MethodNotImplementedException("HelixGeom_HelixCurve::IsPeriodic");

    /// <summary>Throws <see cref="DomainErrorException"/>: the helix isn't periodic.</summary>
    public override double Period() => throw new DomainErrorException("HelixGeom_HelixCurve::Period");

    /// <summary>The point of parameter <paramref name="theT"/>.</summary>
    public sealed override Point3D EvalD0(double theT)
    {
        double aST, aCT, aX, aY, aZ, a1;
        // Calculate trigonometric values and radius
        aCT = Math.Cos(theT);
        aST = Math.Sin(theT);
        a1 = myRStart + myC1 * myTgBeta * theT;
        // Calculate Cartesian coordinates
        aX = a1 * aCT;
        aY = a1 * aST;
        if (!myIsClockWise)
        {
            aY = -aY;
        }
        aZ = myC1 * theT;
        return new Point3D(aX, aY, aZ);
    }

    /// <summary>The point and first derivative at <paramref name="theT"/>.</summary>
    public sealed override Geom_Curve.ResD1 EvalD1(double theT)
    {
        double aST, aCT, aX, aY, aZ, a1, a2;
        // Calculate point and first derivative
        aCT = Math.Cos(theT);
        aST = Math.Sin(theT);
        // Calculate radius at parameter t
        a1 = myRStart + myC1 * myTgBeta * theT;
        // Calculate point coordinates
        aX = a1 * aCT;
        aY = a1 * aST;
        if (!myIsClockWise)
        {
            aY = -aY;
        }
        aZ = myC1 * theT;
        Point3D aP = new(aX, aY, aZ);
        // Calculate first derivative coefficients
        a1 = myC1 * myTgBeta;
        a2 = myRStart + a1 * theT;
        // Calculate first derivative components
        aX = a1 * aCT - a2 * aST;
        aY = a1 * aST + a2 * aCT;
        if (!myIsClockWise)
        {
            aY = -aY;
        }
        aZ = myC1;
        Vector3D aV1 = new(aX, aY, aZ);
        return new Geom_Curve.ResD1 { Point = aP, D1 = aV1 };
    }

    /// <summary>The point and first two derivatives at <paramref name="theT"/>.</summary>
    public sealed override Geom_Curve.ResD2 EvalD2(double theT)
    {
        double aST, aCT, aX, aY, aZ, a1, a2;
        // Calculate point, first and second derivatives
        aCT = Math.Cos(theT);
        aST = Math.Sin(theT);
        // Calculate radius at parameter t
        a1 = myRStart + myC1 * myTgBeta * theT;
        // Calculate point coordinates
        aX = a1 * aCT;
        aY = a1 * aST;
        if (!myIsClockWise)
        {
            aY = -aY;
        }
        aZ = myC1 * theT;
        Point3D aP = new(aX, aY, aZ);
        // Calculate first derivative coefficients
        a1 = myC1 * myTgBeta;
        a2 = myRStart + a1 * theT;
        // Calculate first derivative components
        aX = a1 * aCT - a2 * aST;
        aY = a1 * aST + a2 * aCT;
        if (!myIsClockWise)
        {
            aY = -aY;
        }
        aZ = myC1;
        Vector3D aV1 = new(aX, aY, aZ);
        // Calculate second derivative
        a1 = 2.0 * a1;
        aX = -a2 * aCT - a1 * aST;
        aY = -a2 * aST - a1 * aCT;
        if (!myIsClockWise)
        {
            aY = -aY;
        }
        aZ = 0.0;
        Vector3D aV2 = new(aX, aY, aZ);
        return new Geom_Curve.ResD2 { Point = aP, D1 = aV1, D2 = aV2 };
    }

    /// <summary>The derivative of order 1 or 2 at <paramref name="theT"/>; throws <see cref="MethodNotImplementedException"/> for other orders.</summary>
    public sealed override Vector3D EvalDN(double theT, int theN)
    {
        // Compute derivative based on order
        switch (theN)
        {
            case 1:
            {
                Geom_Curve.ResD1 aRes = EvalD1(theT);
                return aRes.D1;
            }
            case 2:
            {
                Geom_Curve.ResD2 aRes = EvalD2(theT);
                return aRes.D2;
            }
            default:
                throw new MethodNotImplementedException("HelixGeom_HelixCurve::EvalDN");
        }
    }
}
