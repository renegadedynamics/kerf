// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Curve.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
using Kerf.Engine;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection;

/// <summary>
/// A parametric curve which is the result of the intersection between a cylinder or a cone and a quadric
/// (<see cref="IntQuadQuad"/>).
/// </summary>
/// <remarks>
/// A value type (decision 153): OCCT copies it (fixed arrays, IntPatch's lines). The setters are internal and have
/// public <c>With</c> twins. <c>new IntAna_Curve()</c> is OCCT's empty curve (type GeomAbs_OtherSurface);
/// <c>default</c> is all zeros, whose type is GeomAbs_Plane.
/// </remarks>
internal partial struct Curve : IEquatable<Curve>
{
    private double _z0Cte;
    private double _z0Sin;
    private double _z0Cos;
    private double _z0SinSin;
    private double _z0CosCos;
    private double _z0CosSin;
    private double _z1Cte;
    private double _z1Sin;
    private double _z1Cos;
    private double _z1SinSin;
    private double _z1CosCos;
    private double _z1CosSin;
    private double _z2Cte;
    private double _z2Sin;
    private double _z2Cos;
    private double _z2SinSin;
    private double _z2CosCos;
    private double _z2CosSin;
    private bool _twoCurves;
    private bool _takeZPositive;
    private double _tolerance;

    // Internal fields defining the default domain
    private double _domainInf, _domainSup;
    private bool _restrictedInf;
    private bool _restrictedSup;
    private bool _firstbounded;
    private bool _lastbounded;
    private SurfaceKind _typequadric;
    private double _rCyl;
    private double _angle;
    private CoordinateSystem _ax3;

    // Trim boundaries
    private double _firstParameter, _lastParameter;

    /// <summary>The empty curve.</summary>
    public Curve()
    {
        _z0Cte = 0.0;
        _z0Sin = 0.0;
        _z0Cos = 0.0;
        _z0SinSin = 0.0;
        _z0CosCos = 0.0;
        _z0CosSin = 0.0;
        _z1Cte = 0.0;
        _z1Sin = 0.0;
        _z1Cos = 0.0;
        _z1SinSin = 0.0;
        _z1CosCos = 0.0;
        _z1CosSin = 0.0;
        _z2Cte = 0.0;
        _z2Sin = 0.0;
        _z2Cos = 0.0;
        _z2SinSin = 0.0;
        _z2CosCos = 0.0;
        _z2CosSin = 0.0;
        _twoCurves = false;
        _takeZPositive = false;
        _tolerance = 0.0;
        _domainInf = 0.0;
        _domainSup = 0.0;
        _restrictedInf = false;
        _restrictedSup = false;
        _firstbounded = false;
        _lastbounded = false;
        _typequadric = OtherSurface;
        _rCyl = 0.0;
        _angle = 0.0;
        _ax3 = new CoordinateSystem();
        _firstParameter = 0.0;
        _lastParameter = 0.0;
    }

    //=================================================================================================
    // function : SetConeQuadValues
    // purpose  : Description of the Cone-Quadric intersection
    //=================================================================================================

    /// <summary>Sets the parameters used to compute points and derivatives on the curve (cone and quadric).</summary>
    internal void SetConeQuadValues(in Cone3D cone,
                                    double qxx,
                                    double qyy,
                                    double qzz,
                                    double qxy,
                                    double qxz,
                                    double qyz,
                                    double qx,
                                    double qy,
                                    double qz,
                                    double q1,
                                    double tol,
                                    double domInf,
                                    double domSup,
                                    bool twocurves,
                                    bool takezpositive)
    {
        _ax3 = cone.Position;
        _rCyl = cone.RefRadius;

        _angle = cone.SemiAngle;
        double unSurTgAngle = 1.0 / (Math.Tan(cone.SemiAngle));

        _typequadric = global::Kerf.Geometry.SurfaceKind.Cone;

        _twoCurves = twocurves;         //-- two Z values for the same parameter
        _takeZPositive = takezpositive; //-- Take the positive Z on the curve
                                       //--   ( -B + std::sqrt()) and not (-B - std::sqrt())

        _z0Cte = q1;     //-- Note: we have    Z?Cos std::cos(t)
        _z0Sin = 0.0;    //-- and not         2 Z?Cos std::cos(t) !!!
        _z0Cos = 0.0;    //-- This applies to all parameters
        _z0CosCos = 0.0; //--  i.e. no coefficient 2
        _z0SinSin = 0.0; //--     in front of the CS C S terms
        _z0CosSin = 0.0;

        _z1Cte = 2.0 * (unSurTgAngle) * qz;
        _z1Sin = qy + qy;
        _z1Cos = qx + qx;
        _z1CosCos = 0.0;
        _z1SinSin = 0.0;
        _z1CosSin = 0.0;

        _z2Cte = qzz * unSurTgAngle * unSurTgAngle;
        _z2Sin = (unSurTgAngle + unSurTgAngle) * qyz;
        _z2Cos = (unSurTgAngle + unSurTgAngle) * qxz;
        _z2CosCos = qxx;
        _z2SinSin = qyy;
        _z2CosSin = qxy;

        _tolerance = tol;
        _domainInf = domInf;
        _domainSup = domSup;

        _restrictedInf = _restrictedSup = true; //-- Le Domaine est Borne
        _firstbounded = _lastbounded = false;

        _firstParameter = _domainInf;
        _lastParameter = (_twoCurves) ? _domainSup + _domainSup - _domainInf : _domainSup;
    }

    //=================================================================================================
    // function : SetCylinderQuadValues
    // purpose  : Description of the Cylinder-Quadric intersection
    //=================================================================================================

    /// <summary>Sets the parameters used to compute points and derivatives on the curve (cylinder and quadric).</summary>
    internal void SetCylinderQuadValues(in Cylinder3D cyl,
                                        double qxx,
                                        double qyy,
                                        double qzz,
                                        double qxy,
                                        double qxz,
                                        double qyz,
                                        double qx,
                                        double qy,
                                        double qz,
                                        double q1,
                                        double tol,
                                        double domInf,
                                        double domSup,
                                        bool twocurves,
                                        bool takezpositive)
    {
        _ax3 = cyl.Position;
        _rCyl = cyl.Radius;
        _typequadric = SurfaceKind.Cylinder;

        _twoCurves = twocurves;         //-- two Z values for the same parameter
        _takeZPositive = takezpositive; //-- Take the positive Z on the curve
        double rCylmul2 = _rCyl + _rCyl; //--   ( -B + std::sqrt())

        _z0Cte = q1;
        _z0Sin = rCylmul2 * qy;
        _z0Cos = rCylmul2 * qx;
        _z0CosCos = qxx * _rCyl * _rCyl;
        _z0SinSin = qyy * _rCyl * _rCyl;
        _z0CosSin = _rCyl * _rCyl * qxy;

        _z1Cte = qz + qz;
        _z1Sin = rCylmul2 * qyz;
        _z1Cos = rCylmul2 * qxz;
        _z1CosCos = 0.0;
        _z1SinSin = 0.0;
        _z1CosSin = 0.0;

        _z2Cte = qzz;
        _z2Sin = 0.0;
        _z2Cos = 0.0;
        _z2CosCos = 0.0;
        _z2SinSin = 0.0;
        _z2CosSin = 0.0;

        _tolerance = tol;
        _domainInf = domInf;
        _domainSup = domSup;

        _restrictedInf = _restrictedSup = true;
        _firstbounded = _lastbounded = false;

        _firstParameter = _domainInf;
        _lastParameter = (_twoCurves) ? _domainSup + _domainSup - _domainInf : _domainSup;
    }

    /// <summary>True if the curve is not infinite at the last parameter or at the first parameter of the domain.</summary>
    public readonly bool IsOpen => (_restrictedInf && _restrictedSup);

    /// <summary>The parametric domain of the curve.</summary>
    /// <exception cref="DomainErrorException">The domain is not bounded (an empty curve).</exception>
    public readonly void Domain(out double first, out double last)
    {
        if (_restrictedInf && _restrictedSup)
        {
            first = _firstParameter;
            last = _lastParameter;
        }
        else
        {
            throw new DomainErrorException("IntAna_Curve::Domain");
        }
    }

    /// <summary>True if the function is constant (never, in OCCT).</summary>
    public readonly bool IsConstant
    {
        get
        {
            //-- ???  Not easy to decide based only on the parameters.
            return (false);
        }
    }

    /// <summary>True if the domain is open at the beginning.</summary>
    public readonly bool IsFirstOpen => (_firstbounded);

    /// <summary>True if the domain is open at the end.</summary>
    public readonly bool IsLastOpen => (_lastbounded);

    /// <summary>If Flag is true, the curve is not defined at the first parameter of its domain.</summary>
    internal void SetIsFirstOpen(bool flag)
    {
        _firstbounded = flag;
    }

    /// <summary>If Flag is true, the curve is not defined at the last parameter of its domain.</summary>
    internal void SetIsLastOpen(bool flag)
    {
        _lastbounded = flag;
    }

    private readonly void InternalUVValue(double theta,
                                          out double param1,
                                          out double param2,
                                          out double a,
                                          out double b,
                                          out double c,
                                          out double cost,
                                          out double sint,
                                          ref double signeSqrtDis)
    {
        double relTolp = 1.0 + Epsilon(1.0), relTolm = 1.0 - Epsilon(1.0);

        // Infinitesimal step of increasing curve parameter. See comment below.
        double dt = 100.0 * Epsilon(_domainSup + _domainSup - _domainInf);

        double Theta = theta;
        bool secondSolution = false;

        if ((Theta < _domainInf * relTolm) || ((Theta > _domainSup * relTolp) && (!_twoCurves))
            || (Theta > (_domainSup + _domainSup - _domainInf) * relTolp))
        {
            signeSqrtDis = 0.0;
            throw new DomainErrorException("IntAna_Curve::Domain");
        }

        if (Math.Abs(Theta - _domainSup) < dt)
        {
            // Point of Null-discriminant.
            Theta = _domainSup;
        }
        else if (Theta > _domainSup)
        {
            Theta = _domainSup + _domainSup - Theta;
            secondSolution = true;
        }

        param1 = Theta;

        if (!_twoCurves)
        {
            secondSolution = _takeZPositive;
        }
        //
        cost = Math.Cos(Theta);
        sint = Math.Sin(Theta);
        double sin2t = Math.Sin(Theta + Theta);
        double cos2t = Math.Cos(Theta + Theta);

        a =
          _z2Cte + sint * (_z2Sin + sint * _z2SinSin) + cost * (_z2Cos + cost * _z2CosCos) + _z2CosSin * sin2t;

        // OCCT: (Z2CosSin * Z2CosSin) where the other derivatives have (Z?CosSin + Z?CosSin); it only changes the
        // tolerance aTolD below
        double da =
          cost * _z2Sin - sint * _z2Cos + sin2t * (_z2SinSin - _z2CosCos) + cos2t * (_z2CosSin * _z2CosSin);

        b =
          _z1Cte + sint * (_z1Sin + sint * _z1SinSin) + cost * (_z1Cos + cost * _z1CosCos) + _z1CosSin * sin2t;

        double db =
          _z1Sin * cost - _z1Cos * sint + sin2t * (_z1SinSin - _z1CosCos) + cos2t * (_z1CosSin + _z1CosSin);

        c =
          _z0Cte + sint * (_z0Sin + sint * _z0SinSin) + cost * (_z0Cos + cost * _z0CosCos) + _z0CosSin * sin2t;

        double dc =
          _z0Sin * cost - _z0Cos * sint + sin2t * (_z0SinSin - _z0CosCos) + cos2t * (_z0CosSin + _z0CosSin);

        double discriminant = b * b - 4.0 * a * c;

        // We consider that infinitesimal dt = aDT.
        // Error of discriminant computation is equal to
        // (d(Disc)/dt)*dt, where 1st derivative d(Disc)/dt = 2*B*aDB - 4*(A*aDC + C*aDA).

        double tolD = 2.0 * dt * Math.Abs(b * db - 2.0 * (a * dc + c * da));

        if (discriminant < tolD)
        {
            discriminant = 0.0;
        }

        if (Math.Abs(a) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            if (Math.Abs(b) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                param2 = 0.0;
            }
            else
            {
                param2 = -c / b;
            }
        }
        else
        {
            signeSqrtDis = (secondSolution) ? Math.Sqrt(discriminant) : -Math.Sqrt(discriminant);
            param2 = (-b + signeSqrtDis) / (a + a);
        }
    }

    /// <summary>The point at parameter theta on the curve.</summary>
    /// <exception cref="DomainErrorException">theta is outside the domain.</exception>
    public readonly Point3D Value(double theta)
    {
        double a, b, c, u, v, sint, cost, signeSqrtDis;
        //
        signeSqrtDis = 0.0;
        InternalUVValue(theta, out u, out v, out a, out b, out c, out cost, out sint, ref signeSqrtDis);
        //-- checked the parameter U and Raises Domain Error if Error
        return (InternalValue(u, v));
    }

    /// <summary>
    /// The point and the first derivative (a finite difference) at parameter theta on the curve. Returns false, with
    /// a null vector, where the derivative can't be computed.
    /// </summary>
    /// <exception cref="DomainErrorException">theta is outside the domain.</exception>
    public readonly bool D1u(double theta, out Point3D pt, out Vector3D vec)
    {
        //-- To detect the case where the computation is impossible
        double a, b, c, u, v, sint, cost, signeSqrtDis;
        // Not set by InternalUVValue when |A| <= PConfusion, where OCCT reads it uninitialised; the test below
        // returns on |A| < 1e-7 before reading it
        signeSqrtDis = 0.0;
        //
        InternalUVValue(theta, out u, out v, out a, out b, out c, out cost, out sint, ref signeSqrtDis);
        //
        pt = Value(theta);
        vec = new Vector3D(); // OCCT leaves the vector unchanged when it returns false
        if (Math.Abs(a) < 1.0e-7 || Math.Abs(signeSqrtDis) < 1.0e-10)
        {
            return (false);
        }

        //-- Approximation of the derivative (better than the mathematical computation!)
        double dtheta = (_domainSup - _domainInf) * 1.0e-6;
        double theta2 = theta + dtheta;
        if ((theta2 < _domainInf) || ((theta2 > _domainSup) && (!_twoCurves))
            || (theta2 > (_domainSup + _domainSup - _domainInf + 1.0e-14)))
        {
            dtheta = -dtheta;
            theta2 = theta + dtheta;
        }
        Point3D p2 = Value(theta2);
        dtheta = 1.0 / dtheta;
        vec = vec.WithCoord((p2.X - pt.X) * dtheta, (p2.Y - pt.Y) * dtheta, (p2.Z - pt.Z) * dtheta);

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
    public readonly void FindParameter(in Point3D p, NCollection_List<double> theParams)
    {
        double pIpPI = M_PI + M_PI, epsAng = 1.0e-8,
               internalPrecision = 1.0e-8; // precision of internal algorithm of values computation
        double sqTolPrecision = global::Kerf.Engine.Foundation.Precision.SquareConfusion(); //for boundary points to check their coincidence with others

        double theta = 0.0;
        //
        switch (_typequadric)
        {
            case SurfaceKind.Cylinder:
                {
                    double z;
                    global::Kerf.Engine.Geometry.ElSLib.CylinderParameters(_ax3, _rCyl, p, out theta, out z);
                }
                break;

            case SurfaceKind.Cone:
                {
                    double z;
                    global::Kerf.Engine.Geometry.ElSLib.ConeParameters(_ax3, _rCyl, _angle, p, out theta, out z);
                }
                break;

            default:
                return;
        }
        //
        if (!_firstbounded && (_domainInf > theta) && ((_domainInf - theta) <= epsAng))
        {
            theta = _domainInf;
        }
        else if (!_lastbounded && (theta > _domainSup) && ((theta - _domainSup) <= epsAng))
        {
            theta = _domainSup;
        }
        //
        if (theta < _domainInf)
        {
            theta = theta + pIpPI;
        }
        else if (theta > _domainSup)
        {
            theta = theta - pIpPI;
        }

        const int maxPar = 5;
        Span<double> aParams = [_domainInf,
                                _domainSup,
                                theta,
                                (_twoCurves) ? _domainSup + _domainSup - theta : RealLast(),
                                (_twoCurves) ? _domainSup + _domainSup - _domainInf : RealLast()];

        // As OCCT, sorts the first four only (std::sort(aParams, aParams + aMaxPar - 1))
        aParams[..(maxPar - 1)].Sort();

        for (int i = 0; i < maxPar; i++)
        {
            if (aParams[i] > _lastParameter)
            {
                break;
            }

            if (aParams[i] < _firstParameter)
            {
                continue;
            }

            if (i != 0 && (aParams[i] - aParams[i - 1]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                continue;
            }

            double u, v, a, b, c, sint, cost, signeSqrtDis = 0.0;
            InternalUVValue(aParams[i], out u, out v, out a, out b, out c, out cost, out sint, ref signeSqrtDis);
            Point3D aP = InternalValue(u, v);

            double sqTol;
            if (aParams[i] == theta || (_twoCurves && aParams[i] == _domainSup + _domainSup - theta))
            {
                sqTol = internalPrecision;
            }
            else
            {
                sqTol = sqTolPrecision;
            }

            if (aP.SquareDistanceTo(p) < sqTol)
            {
                theParams.Append(aParams[i]);
            }
        }
    }

    private readonly Point3D InternalValue(double u, double _V)
    {
        //-- std::cout<<" ["<<U<<","<<V<<"]";
        double v = _V;
        if (v > 100000.0)
        {
            v = 100000.0;
        }
        if (v < -100000.0)
        {
            v = -100000.0;
        }

        switch (_typequadric)
        {
            case SurfaceKind.Cone:
                {
                    //------------------------------------------------
                    //-- Parametrage : X = V * std::cos(U)              ---
                    //--               Y = V * std::sin(U)              ---
                    //--               Z = (V-RCyl) / std::tan(SemiAngle)--
                    //------------------------------------------------
                    //-- Angle Vaut Cone.SemiAngle()
                    return (global::Kerf.Engine.Geometry.ElSLib.ConeValue(u, (v - _rCyl) / Math.Sin(_angle), _ax3, _rCyl, _angle));
                }

            case SurfaceKind.Cylinder:
                return (global::Kerf.Engine.Geometry.ElSLib.CylinderValue(u, v, _ax3, _rCyl));
            case SurfaceKind.Sphere:
                return (global::Kerf.Engine.Geometry.ElSLib.SphereValue(u, v, _ax3, _rCyl));
            default:
                return (new Point3D(0.0, 0.0, 0.0));
        }
    }

    /// <summary>Trims this curve.</summary>
    /// <exception cref="DomainErrorException">theLast &lt;= theFirst.</exception>
    internal void SetDomain(double first, double last)
    {
        if (last <= first)
        {
            throw new DomainErrorException("IntAna_Curve::Domain");
        }
        //
        _firstParameter = first;
        _lastParameter = last;
    }

    // Non-mutating twins (docs/translation-rules.md, section 3)

    /// <summary>A copy set by <see cref="SetConeQuadValues"/>.</summary>
    public readonly Curve WithConeQuadValues(in Cone3D cone, double qxx, double qyy, double qzz, double qxy,
                                                    double qxz, double qyz, double qx, double qy, double qz, double q1,
                                                    double tol, double domInf, double domSup, bool twocurves,
                                                    bool takezpositive)
    {
        Curve copy = this;
        copy.SetConeQuadValues(cone, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1, tol, domInf, domSup, twocurves,
                                takezpositive);
        return copy;
    }

    /// <summary>A copy set by <see cref="SetCylinderQuadValues"/>.</summary>
    public readonly Curve WithCylinderQuadValues(in Cylinder3D cyl, double qxx, double qyy, double qzz,
                                                        double qxy, double qxz, double qyz, double qx, double qy,
                                                        double qz, double q1, double tol, double domInf, double domSup,
                                                        bool twocurves, bool takezpositive)
    {
        Curve copy = this;
        copy.SetCylinderQuadValues(cyl, qxx, qyy, qzz, qxy, qxz, qyz, qx, qy, qz, q1, tol, domInf, domSup, twocurves,
                                    takezpositive);
        return copy;
    }

    /// <summary>A copy with the first bound open (or not).</summary>
    public readonly Curve WithIsFirstOpen(bool flag)
    {
        Curve copy = this;
        copy.SetIsFirstOpen(flag);
        return copy;
    }

    /// <summary>A copy with the last bound open (or not).</summary>
    public readonly Curve WithIsLastOpen(bool flag)
    {
        Curve copy = this;
        copy.SetIsLastOpen(flag);
        return copy;
    }

    /// <summary>A copy trimmed to [theFirst, theLast].</summary>
    /// <exception cref="DomainErrorException">theLast &lt;= theFirst.</exception>
    public readonly Curve WithDomain(double first, double last)
    {
        Curve copy = this;
        copy.SetDomain(first, last);
        return copy;
    }

    public readonly bool Equals(Curve other)
        => _z0Cte.Equals(other._z0Cte) && _z0Sin.Equals(other._z0Sin) && _z0Cos.Equals(other._z0Cos)
           && _z0SinSin.Equals(other._z0SinSin) && _z0CosCos.Equals(other._z0CosCos) && _z0CosSin.Equals(other._z0CosSin)
           && _z1Cte.Equals(other._z1Cte) && _z1Sin.Equals(other._z1Sin) && _z1Cos.Equals(other._z1Cos)
           && _z1SinSin.Equals(other._z1SinSin) && _z1CosCos.Equals(other._z1CosCos) && _z1CosSin.Equals(other._z1CosSin)
           && _z2Cte.Equals(other._z2Cte) && _z2Sin.Equals(other._z2Sin) && _z2Cos.Equals(other._z2Cos)
           && _z2SinSin.Equals(other._z2SinSin) && _z2CosCos.Equals(other._z2CosCos) && _z2CosSin.Equals(other._z2CosSin)
           && _twoCurves == other._twoCurves && _takeZPositive == other._takeZPositive
           && _tolerance.Equals(other._tolerance) && _domainInf.Equals(other._domainInf)
           && _domainSup.Equals(other._domainSup) && _restrictedInf == other._restrictedInf
           && _restrictedSup == other._restrictedSup && _firstbounded == other._firstbounded
           && _lastbounded == other._lastbounded && _typequadric == other._typequadric && _rCyl.Equals(other._rCyl)
           && _angle.Equals(other._angle) && _ax3.Equals(other._ax3) && _firstParameter.Equals(other._firstParameter)
           && _lastParameter.Equals(other._lastParameter);

    public override readonly bool Equals(object? obj) => obj is Curve other && Equals(other);

    public override readonly int GetHashCode()
    {
        HashCode hash = new();
        hash.Add(_z0Cte);
        hash.Add(_z1Cte);
        hash.Add(_z2Cte);
        hash.Add(_domainInf);
        hash.Add(_domainSup);
        hash.Add(_typequadric);
        hash.Add(_rCyl);
        hash.Add(_ax3);
        hash.Add(_firstParameter);
        hash.Add(_lastParameter);
        return hash.ToHashCode();
    }

    public static bool operator ==(in Curve left, in Curve right) => left.Equals(right);

    public static bool operator !=(in Curve left, in Curve right) => !left.Equals(right);

    /// <summary>The surface type, radius, angle, domain and trim of the curve.</summary>
    public override readonly string ToString()
        => Constants.Format("IntAna_Curve(" + _typequadric + ")", _rCyl, _angle, _domainInf, _domainSup, _firstParameter,
                     _lastParameter);
}
