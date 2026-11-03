// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Curve.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//----------------------------------------------------------------------
//-- Various constructors are provided corresponding to
//-- polynomials in Z:
//--    A(std::sin(Theta),std::cos(Theta)) Z**2
//--  + B(std::sin(Theta),std::cos(Theta)) Z
//--  + C(std::sin(Theta),std::cos(Theta))
//--
//-- A Curve is defined on a domain
//--
//-- Value returns the point at parameter U(Theta),V(Theta)
//--       where V is the solution of the polynomial A V**2 + B V + C
//--       (Depending on the case, V+ or V- is taken)
//--
//-- D1u   computes the tangent vector to the curve
//--       and returns false if this computation cannot
//--       be carried out.
//----------------------------------------------------------------------

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// A parametric curve which is the result of the intersection between a cylinder or a cone and a quadric
/// (<see cref="IntAna_IntQuadQuad"/>).
/// </summary>
/// <remarks>
/// A value type (decision 153): OCCT copies it (fixed arrays, IntPatch's lines). The setters are internal and have
/// public <c>With</c> twins. <c>new IntAna_Curve()</c> is OCCT's empty curve (type GeomAbs_OtherSurface);
/// <c>default</c> is all zeros, whose type is GeomAbs_Plane.
/// </remarks>
public partial struct IntAna_Curve : IEquatable<IntAna_Curve>
{
    private double Z0Cte;
    private double Z0Sin;
    private double Z0Cos;
    private double Z0SinSin;
    private double Z0CosCos;
    private double Z0CosSin;
    private double Z1Cte;
    private double Z1Sin;
    private double Z1Cos;
    private double Z1SinSin;
    private double Z1CosCos;
    private double Z1CosSin;
    private double Z2Cte;
    private double Z2Sin;
    private double Z2Cos;
    private double Z2SinSin;
    private double Z2CosCos;
    private double Z2CosSin;
    private bool TwoCurves;
    private bool TakeZPositive;
    private double Tolerance;

    // Internal fields defining the default domain
    private double DomainInf, DomainSup;
    private bool RestrictedInf;
    private bool RestrictedSup;
    private bool firstbounded;
    private bool lastbounded;
    private SurfaceKind typequadric;
    private double RCyl;
    private double Angle;
    private CoordinateSystem Ax3;

    // Trim boundaries
    private double myFirstParameter, myLastParameter;

    /// <summary>The empty curve.</summary>
    public IntAna_Curve()
    {
        Z0Cte = 0.0;
        Z0Sin = 0.0;
        Z0Cos = 0.0;
        Z0SinSin = 0.0;
        Z0CosCos = 0.0;
        Z0CosSin = 0.0;
        Z1Cte = 0.0;
        Z1Sin = 0.0;
        Z1Cos = 0.0;
        Z1SinSin = 0.0;
        Z1CosCos = 0.0;
        Z1CosSin = 0.0;
        Z2Cte = 0.0;
        Z2Sin = 0.0;
        Z2Cos = 0.0;
        Z2SinSin = 0.0;
        Z2CosCos = 0.0;
        Z2CosSin = 0.0;
        TwoCurves = false;
        TakeZPositive = false;
        Tolerance = 0.0;
        DomainInf = 0.0;
        DomainSup = 0.0;
        RestrictedInf = false;
        RestrictedSup = false;
        firstbounded = false;
        lastbounded = false;
        typequadric = OtherSurface;
        RCyl = 0.0;
        Angle = 0.0;
        Ax3 = new CoordinateSystem();
        myFirstParameter = 0.0;
        myLastParameter = 0.0;
    }

    //=================================================================================================
    // function : SetConeQuadValues
    // purpose  : Description of the Cone-Quadric intersection
    //=================================================================================================

    /// <summary>Sets the parameters used to compute points and derivatives on the curve (cone and quadric).</summary>
    internal void SetConeQuadValues(in Cone Cone,
                                    double Qxx,
                                    double Qyy,
                                    double Qzz,
                                    double Qxy,
                                    double Qxz,
                                    double Qyz,
                                    double Qx,
                                    double Qy,
                                    double Qz,
                                    double Q1,
                                    double TOL,
                                    double DomInf,
                                    double DomSup,
                                    bool twocurves,
                                    bool takezpositive)
    {
        Ax3 = Cone.Position();
        RCyl = Cone.RefRadius();

        Angle = Cone.SemiAngle();
        double UnSurTgAngle = 1.0 / (Math.Tan(Cone.SemiAngle()));

        typequadric = global::Kerf.Geometry.SurfaceKind.Cone;

        TwoCurves = twocurves;         //-- two Z values for the same parameter
        TakeZPositive = takezpositive; //-- Take the positive Z on the curve
                                       //--   ( -B + std::sqrt()) and not (-B - std::sqrt())

        Z0Cte = Q1;     //-- Note: we have    Z?Cos std::cos(t)
        Z0Sin = 0.0;    //-- and not         2 Z?Cos std::cos(t) !!!
        Z0Cos = 0.0;    //-- This applies to all parameters
        Z0CosCos = 0.0; //--  i.e. no coefficient 2
        Z0SinSin = 0.0; //--     in front of the CS C S terms
        Z0CosSin = 0.0;

        Z1Cte = 2.0 * (UnSurTgAngle) * Qz;
        Z1Sin = Qy + Qy;
        Z1Cos = Qx + Qx;
        Z1CosCos = 0.0;
        Z1SinSin = 0.0;
        Z1CosSin = 0.0;

        Z2Cte = Qzz * UnSurTgAngle * UnSurTgAngle;
        Z2Sin = (UnSurTgAngle + UnSurTgAngle) * Qyz;
        Z2Cos = (UnSurTgAngle + UnSurTgAngle) * Qxz;
        Z2CosCos = Qxx;
        Z2SinSin = Qyy;
        Z2CosSin = Qxy;

        Tolerance = TOL;
        DomainInf = DomInf;
        DomainSup = DomSup;

        RestrictedInf = RestrictedSup = true; //-- Le Domaine est Borne
        firstbounded = lastbounded = false;

        myFirstParameter = DomainInf;
        myLastParameter = (TwoCurves) ? DomainSup + DomainSup - DomainInf : DomainSup;
    }

    //=================================================================================================
    // function : SetCylinderQuadValues
    // purpose  : Description of the Cylinder-Quadric intersection
    //=================================================================================================

    /// <summary>Sets the parameters used to compute points and derivatives on the curve (cylinder and quadric).</summary>
    internal void SetCylinderQuadValues(in Cylinder Cyl,
                                        double Qxx,
                                        double Qyy,
                                        double Qzz,
                                        double Qxy,
                                        double Qxz,
                                        double Qyz,
                                        double Qx,
                                        double Qy,
                                        double Qz,
                                        double Q1,
                                        double TOL,
                                        double DomInf,
                                        double DomSup,
                                        bool twocurves,
                                        bool takezpositive)
    {
        Ax3 = Cyl.Position();
        RCyl = Cyl.Radius();
        typequadric = SurfaceKind.Cylinder;

        TwoCurves = twocurves;         //-- two Z values for the same parameter
        TakeZPositive = takezpositive; //-- Take the positive Z on the curve
        double RCylmul2 = RCyl + RCyl; //--   ( -B + std::sqrt())

        Z0Cte = Q1;
        Z0Sin = RCylmul2 * Qy;
        Z0Cos = RCylmul2 * Qx;
        Z0CosCos = Qxx * RCyl * RCyl;
        Z0SinSin = Qyy * RCyl * RCyl;
        Z0CosSin = RCyl * RCyl * Qxy;

        Z1Cte = Qz + Qz;
        Z1Sin = RCylmul2 * Qyz;
        Z1Cos = RCylmul2 * Qxz;
        Z1CosCos = 0.0;
        Z1SinSin = 0.0;
        Z1CosSin = 0.0;

        Z2Cte = Qzz;
        Z2Sin = 0.0;
        Z2Cos = 0.0;
        Z2CosCos = 0.0;
        Z2SinSin = 0.0;
        Z2CosSin = 0.0;

        Tolerance = TOL;
        DomainInf = DomInf;
        DomainSup = DomSup;

        RestrictedInf = RestrictedSup = true;
        firstbounded = lastbounded = false;

        myFirstParameter = DomainInf;
        myLastParameter = (TwoCurves) ? DomainSup + DomainSup - DomainInf : DomainSup;
    }

    /// <summary>True if the curve is not infinite at the last parameter or at the first parameter of the domain.</summary>
    public readonly bool IsOpen() => (RestrictedInf && RestrictedSup);

    /// <summary>The parametric domain of the curve.</summary>
    /// <exception cref="DomainErrorException">The domain is not bounded (an empty curve).</exception>
    public readonly void Domain(out double theFirst, out double theLast)
    {
        if (RestrictedInf && RestrictedSup)
        {
            theFirst = myFirstParameter;
            theLast = myLastParameter;
        }
        else
        {
            throw new DomainErrorException("IntAna_Curve::Domain");
        }
    }

    /// <summary>True if the function is constant (never, in OCCT).</summary>
    public readonly bool IsConstant()
    {
        //-- ???  Not easy to decide based only on the parameters.
        return (false);
    }

    /// <summary>True if the domain is open at the beginning.</summary>
    public readonly bool IsFirstOpen() => (firstbounded);

    /// <summary>True if the domain is open at the end.</summary>
    public readonly bool IsLastOpen() => (lastbounded);

    /// <summary>If Flag is true, the curve is not defined at the first parameter of its domain.</summary>
    internal void SetIsFirstOpen(bool Flag)
    {
        firstbounded = Flag;
    }

    /// <summary>If Flag is true, the curve is not defined at the last parameter of its domain.</summary>
    internal void SetIsLastOpen(bool Flag)
    {
        lastbounded = Flag;
    }

    private readonly void InternalUVValue(double theta,
                                          out double Param1,
                                          out double Param2,
                                          out double A,
                                          out double B,
                                          out double C,
                                          out double cost,
                                          out double sint,
                                          ref double SigneSqrtDis)
    {
        double aRelTolp = 1.0 + Epsilon(1.0), aRelTolm = 1.0 - Epsilon(1.0);

        // Infinitesimal step of increasing curve parameter. See comment below.
        double aDT = 100.0 * Epsilon(DomainSup + DomainSup - DomainInf);

        double Theta = theta;
        bool SecondSolution = false;

        if ((Theta < DomainInf * aRelTolm) || ((Theta > DomainSup * aRelTolp) && (!TwoCurves))
            || (Theta > (DomainSup + DomainSup - DomainInf) * aRelTolp))
        {
            SigneSqrtDis = 0.0;
            throw new DomainErrorException("IntAna_Curve::Domain");
        }

        if (Math.Abs(Theta - DomainSup) < aDT)
        {
            // Point of Null-discriminant.
            Theta = DomainSup;
        }
        else if (Theta > DomainSup)
        {
            Theta = DomainSup + DomainSup - Theta;
            SecondSolution = true;
        }

        Param1 = Theta;

        if (!TwoCurves)
        {
            SecondSolution = TakeZPositive;
        }
        //
        cost = Math.Cos(Theta);
        sint = Math.Sin(Theta);
        double aSin2t = Math.Sin(Theta + Theta);
        double aCos2t = Math.Cos(Theta + Theta);

        A =
          Z2Cte + sint * (Z2Sin + sint * Z2SinSin) + cost * (Z2Cos + cost * Z2CosCos) + Z2CosSin * aSin2t;

        // OCCT: (Z2CosSin * Z2CosSin) where the other derivatives have (Z?CosSin + Z?CosSin); it only changes the
        // tolerance aTolD below
        double aDA =
          cost * Z2Sin - sint * Z2Cos + aSin2t * (Z2SinSin - Z2CosCos) + aCos2t * (Z2CosSin * Z2CosSin);

        B =
          Z1Cte + sint * (Z1Sin + sint * Z1SinSin) + cost * (Z1Cos + cost * Z1CosCos) + Z1CosSin * aSin2t;

        double aDB =
          Z1Sin * cost - Z1Cos * sint + aSin2t * (Z1SinSin - Z1CosCos) + aCos2t * (Z1CosSin + Z1CosSin);

        C =
          Z0Cte + sint * (Z0Sin + sint * Z0SinSin) + cost * (Z0Cos + cost * Z0CosCos) + Z0CosSin * aSin2t;

        double aDC =
          Z0Sin * cost - Z0Cos * sint + aSin2t * (Z0SinSin - Z0CosCos) + aCos2t * (Z0CosSin + Z0CosSin);

        double aDiscriminant = B * B - 4.0 * A * C;

        // We consider that infinitesimal dt = aDT.
        // Error of discriminant computation is equal to
        // (d(Disc)/dt)*dt, where 1st derivative d(Disc)/dt = 2*B*aDB - 4*(A*aDC + C*aDA).

        double aTolD = 2.0 * aDT * Math.Abs(B * aDB - 2.0 * (A * aDC + C * aDA));

        if (aDiscriminant < aTolD)
        {
            aDiscriminant = 0.0;
        }

        if (Math.Abs(A) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (Math.Abs(B) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                Param2 = 0.0;
            }
            else
            {
                Param2 = -C / B;
            }
        }
        else
        {
            SigneSqrtDis = (SecondSolution) ? Math.Sqrt(aDiscriminant) : -Math.Sqrt(aDiscriminant);
            Param2 = (-B + SigneSqrtDis) / (A + A);
        }
    }

    /// <summary>The point at parameter theta on the curve.</summary>
    /// <exception cref="DomainErrorException">theta is outside the domain.</exception>
    public readonly Point3D Value(double theta)
    {
        double A, B, C, U, V, sint, cost, SigneSqrtDis;
        //
        SigneSqrtDis = 0.0;
        InternalUVValue(theta, out U, out V, out A, out B, out C, out cost, out sint, ref SigneSqrtDis);
        //-- checked the parameter U and Raises Domain Error if Error
        return (InternalValue(U, V));
    }

    /// <summary>
    /// The point and the first derivative (a finite difference) at parameter theta on the curve. Returns false, with
    /// a null vector, where the derivative can't be computed.
    /// </summary>
    /// <exception cref="DomainErrorException">theta is outside the domain.</exception>
    public readonly bool D1u(double theta, out Point3D Pt, out Vector3D Vec)
    {
        //-- To detect the case where the computation is impossible
        double A, B, C, U, V, sint, cost, SigneSqrtDis;
        // Not set by InternalUVValue when |A| <= PConfusion, where OCCT reads it uninitialised; the test below
        // returns on |A| < 1e-7 before reading it
        SigneSqrtDis = 0.0;
        //
        InternalUVValue(theta, out U, out V, out A, out B, out C, out cost, out sint, ref SigneSqrtDis);
        //
        Pt = Value(theta);
        Vec = new Vector3D(); // OCCT leaves the vector unchanged when it returns false
        if (Math.Abs(A) < 1.0e-7 || Math.Abs(SigneSqrtDis) < 1.0e-10)
        {
            return (false);
        }

        //-- Approximation of the derivative (better than the mathematical computation!)
        double dtheta = (DomainSup - DomainInf) * 1.0e-6;
        double theta2 = theta + dtheta;
        if ((theta2 < DomainInf) || ((theta2 > DomainSup) && (!TwoCurves))
            || (theta2 > (DomainSup + DomainSup - DomainInf + 1.0e-14)))
        {
            dtheta = -dtheta;
            theta2 = theta + dtheta;
        }
        Point3D P2 = Value(theta2);
        dtheta = 1.0 / dtheta;
        Vec.SetCoord((P2.X() - Pt.X()) * dtheta, (P2.Y() - Pt.Y()) * dtheta, (P2.Z() - Pt.Z()) * dtheta);

        return (true);
    }

    //=================================================================================================
    // function : FindParameter
    // purpose  : Projects P to the ALine. Returns the list of parameters as a results
    //            of projection.
    //           Sometimes aline can be self-intersected line (see bug #29807 where
    //            ALine goes through the cone apex).
    //=================================================================================================

    /// <summary>
    /// Appends to theParams the parameters of the point theP on the curve, in ascending order. Nothing is appended
    /// if the "projection" is impossible.
    /// </summary>
    public readonly void FindParameter(in Point3D theP, NCollection_List<double> theParams)
    {
        double aPIpPI = M_PI + M_PI, anEpsAng = 1.0e-8,
               InternalPrecision = 1.0e-8; // precision of internal algorithm of values computation
        double aSqTolPrecision = global::Kerf.Engine.Foundation.Precision.SquareConfusion(); //for boundary points to check their coincidence with others

        double aTheta = 0.0;
        //
        switch (typequadric)
        {
            case SurfaceKind.Cylinder:
                {
                    double aZ;
                    global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(Ax3, RCyl, theP, out aTheta, out aZ);
                }
                break;

            case SurfaceKind.Cone:
                {
                    double aZ;
                    global::Kerf.Engine.Geometry.ElSLib.ConeParameters(Ax3, RCyl, Angle, theP, out aTheta, out aZ);
                }
                break;

            default:
                return;
        }
        //
        if (!firstbounded && (DomainInf > aTheta) && ((DomainInf - aTheta) <= anEpsAng))
        {
            aTheta = DomainInf;
        }
        else if (!lastbounded && (aTheta > DomainSup) && ((aTheta - DomainSup) <= anEpsAng))
        {
            aTheta = DomainSup;
        }
        //
        if (aTheta < DomainInf)
        {
            aTheta = aTheta + aPIpPI;
        }
        else if (aTheta > DomainSup)
        {
            aTheta = aTheta - aPIpPI;
        }

        const int aMaxPar = 5;
        Span<double> aParams = [DomainInf,
                                DomainSup,
                                aTheta,
                                (TwoCurves) ? DomainSup + DomainSup - aTheta : RealLast(),
                                (TwoCurves) ? DomainSup + DomainSup - DomainInf : RealLast()];

        // As OCCT, sorts the first four only (std::sort(aParams, aParams + aMaxPar - 1))
        aParams[..(aMaxPar - 1)].Sort();

        for (int i = 0; i < aMaxPar; i++)
        {
            if (aParams[i] > myLastParameter)
            {
                break;
            }

            if (aParams[i] < myFirstParameter)
            {
                continue;
            }

            if (i != 0 && (aParams[i] - aParams[i - 1]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                continue;
            }

            double U, V, A, B, C, sint, cost, SigneSqrtDis = 0.0;
            InternalUVValue(aParams[i], out U, out V, out A, out B, out C, out cost, out sint, ref SigneSqrtDis);
            Point3D aP = InternalValue(U, V);

            double aSqTol;
            if (aParams[i] == aTheta || (TwoCurves && aParams[i] == DomainSup + DomainSup - aTheta))
            {
                aSqTol = InternalPrecision;
            }
            else
            {
                aSqTol = aSqTolPrecision;
            }

            if (aP.SquareDistance(theP) < aSqTol)
            {
                theParams.Append(aParams[i]);
            }
        }
    }

    private readonly Point3D InternalValue(double U, double _V)
    {
        //-- std::cout<<" ["<<U<<","<<V<<"]";
        double V = _V;
        if (V > 100000.0)
        {
            V = 100000.0;
        }
        if (V < -100000.0)
        {
            V = -100000.0;
        }

        switch (typequadric)
        {
            case SurfaceKind.Cone:
                {
                    //------------------------------------------------
                    //-- Parametrage : X = V * std::cos(U)              ---
                    //--               Y = V * std::sin(U)              ---
                    //--               Z = (V-RCyl) / std::tan(SemiAngle)--
                    //------------------------------------------------
                    //-- Angle Vaut Cone.SemiAngle()
                    return (global::Kerf.Engine.Geometry.ElSLib.ConeValue(U, (V - RCyl) / Math.Sin(Angle), Ax3, RCyl, Angle));
                }

            case SurfaceKind.Cylinder:
                return (global::Kerf.Engine.Geometry.ElSLib.CylinderValue(U, V, Ax3, RCyl));
            case SurfaceKind.Sphere:
                return (global::Kerf.Engine.Geometry.ElSLib.SphereValue(U, V, Ax3, RCyl));
            default:
                return (new Point3D(0.0, 0.0, 0.0));
        }
    }

    /// <summary>Trims this curve.</summary>
    /// <exception cref="DomainErrorException">theLast &lt;= theFirst.</exception>
    internal void SetDomain(double theFirst, double theLast)
    {
        if (theLast <= theFirst)
        {
            throw new DomainErrorException("IntAna_Curve::Domain");
        }
        //
        myFirstParameter = theFirst;
        myLastParameter = theLast;
    }

    // Non-mutating twins (docs/translation-rules.md, section 3)

    /// <summary>A copy set by <see cref="SetConeQuadValues"/>.</summary>
    public readonly IntAna_Curve WithConeQuadValues(in Cone Cone, double Qxx, double Qyy, double Qzz, double Qxy,
                                                    double Qxz, double Qyz, double Qx, double Qy, double Qz, double Q1,
                                                    double TOL, double DomInf, double DomSup, bool twocurves,
                                                    bool takezpositive)
    {
        IntAna_Curve aCopy = this;
        aCopy.SetConeQuadValues(Cone, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1, TOL, DomInf, DomSup, twocurves,
                                takezpositive);
        return aCopy;
    }

    /// <summary>A copy set by <see cref="SetCylinderQuadValues"/>.</summary>
    public readonly IntAna_Curve WithCylinderQuadValues(in Cylinder Cyl, double Qxx, double Qyy, double Qzz,
                                                        double Qxy, double Qxz, double Qyz, double Qx, double Qy,
                                                        double Qz, double Q1, double TOL, double DomInf, double DomSup,
                                                        bool twocurves, bool takezpositive)
    {
        IntAna_Curve aCopy = this;
        aCopy.SetCylinderQuadValues(Cyl, Qxx, Qyy, Qzz, Qxy, Qxz, Qyz, Qx, Qy, Qz, Q1, TOL, DomInf, DomSup, twocurves,
                                    takezpositive);
        return aCopy;
    }

    /// <summary>A copy with the first bound open (or not).</summary>
    public readonly IntAna_Curve WithIsFirstOpen(bool Flag)
    {
        IntAna_Curve aCopy = this;
        aCopy.SetIsFirstOpen(Flag);
        return aCopy;
    }

    /// <summary>A copy with the last bound open (or not).</summary>
    public readonly IntAna_Curve WithIsLastOpen(bool Flag)
    {
        IntAna_Curve aCopy = this;
        aCopy.SetIsLastOpen(Flag);
        return aCopy;
    }

    /// <summary>A copy trimmed to [theFirst, theLast].</summary>
    /// <exception cref="DomainErrorException">theLast &lt;= theFirst.</exception>
    public readonly IntAna_Curve WithDomain(double theFirst, double theLast)
    {
        IntAna_Curve aCopy = this;
        aCopy.SetDomain(theFirst, theLast);
        return aCopy;
    }

    public readonly bool Equals(IntAna_Curve other)
        => Z0Cte.Equals(other.Z0Cte) && Z0Sin.Equals(other.Z0Sin) && Z0Cos.Equals(other.Z0Cos)
           && Z0SinSin.Equals(other.Z0SinSin) && Z0CosCos.Equals(other.Z0CosCos) && Z0CosSin.Equals(other.Z0CosSin)
           && Z1Cte.Equals(other.Z1Cte) && Z1Sin.Equals(other.Z1Sin) && Z1Cos.Equals(other.Z1Cos)
           && Z1SinSin.Equals(other.Z1SinSin) && Z1CosCos.Equals(other.Z1CosCos) && Z1CosSin.Equals(other.Z1CosSin)
           && Z2Cte.Equals(other.Z2Cte) && Z2Sin.Equals(other.Z2Sin) && Z2Cos.Equals(other.Z2Cos)
           && Z2SinSin.Equals(other.Z2SinSin) && Z2CosCos.Equals(other.Z2CosCos) && Z2CosSin.Equals(other.Z2CosSin)
           && TwoCurves == other.TwoCurves && TakeZPositive == other.TakeZPositive
           && Tolerance.Equals(other.Tolerance) && DomainInf.Equals(other.DomainInf)
           && DomainSup.Equals(other.DomainSup) && RestrictedInf == other.RestrictedInf
           && RestrictedSup == other.RestrictedSup && firstbounded == other.firstbounded
           && lastbounded == other.lastbounded && typequadric == other.typequadric && RCyl.Equals(other.RCyl)
           && Angle.Equals(other.Angle) && Ax3.Equals(other.Ax3) && myFirstParameter.Equals(other.myFirstParameter)
           && myLastParameter.Equals(other.myLastParameter);

    public override readonly bool Equals(object? obj) => obj is IntAna_Curve other && Equals(other);

    public override readonly int GetHashCode()
    {
        HashCode aHash = new();
        aHash.Add(Z0Cte);
        aHash.Add(Z1Cte);
        aHash.Add(Z2Cte);
        aHash.Add(DomainInf);
        aHash.Add(DomainSup);
        aHash.Add(typequadric);
        aHash.Add(RCyl);
        aHash.Add(Ax3);
        aHash.Add(myFirstParameter);
        aHash.Add(myLastParameter);
        return aHash.ToHashCode();
    }

    public static bool operator ==(in IntAna_Curve theLeft, in IntAna_Curve theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntAna_Curve theLeft, in IntAna_Curve theRight) => !theLeft.Equals(theRight);

    /// <summary>The surface type, radius, angle, domain and trim of the curve.</summary>
    public override readonly string ToString()
        => Constants.Format("IntAna_Curve(" + typequadric + ")", RCyl, Angle, DomainInf, DomainSup, myFirstParameter,
                     myLastParameter);
}
