// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Gauss.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Gauss.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The global inertia properties of a face, or of the region of 3D space between a face and a point or a plane, by
/// adaptive and non-adaptive 2D Gauss integration.
/// </summary>
/// <remarks>
/// OCCT's <c>add</c> and <c>mult</c> function pointers (switched to the infinity-aware versions when a bound of the
/// face is infinite) are a flag read by the private methods of the same names. The coefficient arrays are spans
/// (empty for OCCT's null pointer).
/// </remarks>
public sealed class BRepGProp_Gauss
{
    /// <summary>The moments of inertia accumulated by the integration.</summary>
    private struct Inertia
    {
        //! Mass of the current system (without density).
        //! May correspond to: length, area, volume.
        public double Mass;

        //! Static moments of inertia.
        public double Ix;
        public double Iy;
        public double Iz;

        //! Quadratic moments of inertia.
        public double Ixx;
        public double Iyy;
        public double Izz;
        public double Ixy;
        public double Ixz;
        public double Iyz;

        /// <summary>Zeroes all values.</summary>
        public void Reset() => this = default;
    }

    /// <summary>The types of geometric objects.</summary>
    public enum BRepGProp_GaussType
    {
        /// <summary>A 3D closed region of space delimited by a surface, a point and a surface, or a plane and a surface.</summary>
        Vinert = 0,

        /// <summary>A face in 3D space.</summary>
        Sinert,
    }

    // If the following is defined the error of algorithm is calculated by static moments
    // #define IS_MIN_DIM (defined: the code under #ifndef IS_MIN_DIM is left out)

    // Minimal value of interval's range for computation | minimal value of "dim" | ...
    private const double EPS_PARAM = 1.0e-12;
    private const double EPS_DIM = 1.0e-30;
    private const double ERROR_ALGEBR_RATIO = 2.0 / 3.0;

    // Maximum of GaussPoints on a subinterval and maximum of subintervals
    private static readonly int GPM = NumericTables.GaussPointsMax();
    private const int SUBS_POWER = 32;
    private static readonly int SM = SUBS_POWER * GPM + 1;

    private readonly BRepGProp_GaussType myType; //!< Type of geometric object
    private bool myIsInf;                        //!< add and mult are AddInf and MultInf

    /// <summary>An integrator for the given type of object.</summary>
    public BRepGProp_Gauss(BRepGProp_GaussType theType)
    {
        myType = theType;
        myIsInf = false;
    }

    // Auxiliary inner functions to perform arithmetic operations.
    private static double Add(double theA, double theB) => theA + theB;

    private static double AddInf(double theA, double theB)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theA))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theB))
            {
                return 0.0;
            }
            else
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theB))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theA))
            {
                return 0.0;
            }
            else
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theA))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theB))
            {
                return 0.0;
            }
            else
            {
                return -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theB))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theA))
            {
                return 0.0;
            }
            else
            {
                return -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        return theA + theB;
    }

    private static double Mult(double theA, double theB) => theA * theB;

    private static double MultInf(double theA, double theB)
    {
        if ((theA == 0.0) || (theB == 0.0)) // strictly zerro (without any tolerances)
        {
            return 0.0;
        }

        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theA))
        {
            if (theB < 0.0)
            {
                return -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theB))
        {
            if (theA < 0.0)
            {
                return -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theA))
        {
            if (theB < 0.0)
            {
                return +global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                return -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theB))
        {
            if (theA < 0.0)
            {
                return +global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                return -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        return theA * theB;
    }

    // OCCT's function pointers add and mult
    private double add(double theA, double theB) => myIsInf ? AddInf(theA, theB) : Add(theA, theB);

    private double mult(double theA, double theB) => myIsInf ? MultInf(theA, theB) : Mult(theA, theB);

    private static int MaxSubs(int theN, int theCoeff = 32) =>
        IntegerLast() / theCoeff < theN ? IntegerLast() : theN * theCoeff + 1;

    private static void Init(math_Vector theOutVec, double theValue, int theFirst = 0, int theLast = 0)
    {
        if (theLast - theFirst == 0)
        {
            theOutVec.Init(theValue);
        }
        else
        {
            for (int i = theFirst; i <= theLast; ++i)
            {
                theOutVec.Value(i) = theValue;
            }
        }
    }

    private static void InitMass(double theValue, int theFirst, int theLast, NCollection_Array1<Inertia>? theArray)
    {
        if (theArray is null)
        {
            return;
        }

        int aFirst = theFirst;
        int aLast = theLast;

        if (theLast - theFirst == 0)
        {
            aFirst = theArray.Lower();
            aLast = theArray.Upper();
        }

        for (int i = aFirst; i <= aLast; ++i)
        {
            theArray.ChangeValue(i).Mass = theValue;
        }
    }

    private static int FillIntervalBounds(double theA,
                                          double theB,
                                          NCollection_Array1<double> theKnots,
                                          int theNumSubs,
                                          ref NCollection_Array1<Inertia> theInerts,
                                          ref math_Vector theParam1,
                                          ref math_Vector theParam2,
                                          ref math_Vector theError,
                                          ref math_Vector? theCommonError)
    {
        int aSize = Max(theKnots.Upper(), MaxSubs(theKnots.Upper() - 1, theNumSubs));

        if (aSize - 1 > theParam1.Upper())
        {
            theInerts = new NCollection_Array1<Inertia>(1, aSize);
            theParam1 = new math_Vector(1, aSize);
            theParam2 = new math_Vector(1, aSize);
            theError = new math_Vector(1, aSize, 0.0);

            if (theCommonError is not null)
            {
                theCommonError = new math_Vector(1, aSize, 0.0);
            }
        }

        int j = 1, k = 1;
        theParam1.Value(j++) = theA;

        int aLength = theKnots.Upper();
        for (int i = 1; i <= aLength; ++i)
        {
            double kn = theKnots[i];
            if (theA < kn)
            {
                if (kn < theB)
                {
                    theParam1.Value(j++) = kn;
                    theParam2.Value(k++) = kn;
                }
                else
                {
                    break;
                }
            }
        }

        theParam2.Value(k) = theB;
        return k;
    }

    private static void computeVInertiaOfElementaryPart(in Point3D thePoint,
                                                        in Vector3D theNormal,
                                                        in Point3D theLocation,
                                                        double theWeight,
                                                        ReadOnlySpan<double> theCoeff,
                                                        bool theIsByPoint,
                                                        ref Inertia theOutInertia)
    {
        double x = thePoint.X() - theLocation.X();
        double y = thePoint.Y() - theLocation.Y();
        double z = thePoint.Z() - theLocation.Z();

        double xn = theNormal.X() * theWeight;
        double yn = theNormal.Y() * theWeight;
        double zn = theNormal.Z() * theWeight;

        if (theIsByPoint)
        {
            /////////////////////                        ///////////////////////
            //    OFV code     //                        //    Initial code   //
            /////////////////////                        ///////////////////////
            // modified by APO

            double dv = x * xn + y * yn + z * zn; // xyz  = x * y * z;
            theOutInertia.Mass += dv / 3.0;       // Ixyi += zn * xyz;
            theOutInertia.Ix += 0.25 * x * dv;    // Iyzi += xn * xyz;
            theOutInertia.Iy += 0.25 * y * dv;    // Ixzi += yn * xyz;
            theOutInertia.Iz += 0.25 * z * dv;    // xi = x * x * x * xn / 3.0;
            x -= theCoeff[0];                     // yi = y * y * y * yn / 3.0;
            y -= theCoeff[1];                     // zi = z * z * z * zn / 3.0;
            z -= theCoeff[2];                     // Ixxi += (yi + zi);
            dv *= 0.2;                            // Iyyi += (xi + zi);
            theOutInertia.Ixy -= x * y * dv;      // Izzi += (xi + yi);
            theOutInertia.Iyz -= y * z * dv;      // x -= Coeff[0];
            theOutInertia.Ixz -= x * z * dv;      // y -= Coeff[1];
            x *= x;                               // z -= Coeff[2];
            y *= y;                               // dv = x * xn + y * yn + z * zn;
            z *= z;                               // dvi +=  dv;
            theOutInertia.Ixx += (y + z) * dv;    // Ixi += x * dv;
            theOutInertia.Iyy += (x + z) * dv;    // Iyi += y * dv;
            theOutInertia.Izz += (x + y) * dv;    // Izi += z * dv;
        }
        else
        { // By plane
            double s = xn * theCoeff[0] + yn * theCoeff[1] + zn * theCoeff[2];

            double d1 = theCoeff[0] * x + theCoeff[1] * y + theCoeff[2] * z - theCoeff[3];

            double d2 = d1 * d1;
            double d3 = d1 * d2 / 3.0;
            double dv = s * d1;

            theOutInertia.Mass += dv;
            theOutInertia.Ix += (x - (theCoeff[0] * d1 * 0.5)) * dv;
            theOutInertia.Iy += (y - (theCoeff[1] * d1 * 0.5)) * dv;
            theOutInertia.Iz += (z - (theCoeff[2] * d1 * 0.5)) * dv;

            double px = x - theCoeff[0] * d1;
            double py = y - theCoeff[1] * d1;
            double pz = z - theCoeff[2] * d1;

            x = px * px * d1 + px * theCoeff[0] * d2 + theCoeff[0] * theCoeff[0] * d3;
            y = py * py * d1 + py * theCoeff[1] * d2 + theCoeff[1] * theCoeff[1] * d3;
            z = pz * pz * d1 + pz * theCoeff[2] * d2 + theCoeff[2] * theCoeff[2] * d3;

            theOutInertia.Ixx += (y + z) * s;
            theOutInertia.Iyy += (x + z) * s;
            theOutInertia.Izz += (x + y) * s;

            d2 *= 0.5;

            x = (py * pz * d1) + (py * theCoeff[2] * d2) + (pz * theCoeff[1] * d2) + (theCoeff[1] * theCoeff[2] * d3);
            y = (px * pz * d1) + (pz * theCoeff[0] * d2) + (px * theCoeff[2] * d2) + (theCoeff[0] * theCoeff[2] * d3);
            z = (px * py * d1) + (px * theCoeff[1] * d2) + (py * theCoeff[0] * d2) + (theCoeff[0] * theCoeff[1] * d3);

            theOutInertia.Ixy -= z * s;
            theOutInertia.Iyz -= x * s;
            theOutInertia.Ixz -= y * s;
        }
    }

    private void computeSInertiaOfElementaryPart(in Point3D thePoint,
                                                 in Vector3D theNormal,
                                                 in Point3D theLocation,
                                                 double theWeight,
                                                 ref Inertia theOutInertia)
    {
        // ds - Jacobien (x, y, z) -> (u, v) = ||n||
        double ds = mult(theNormal.Magnitude(), theWeight);
        double x = add(thePoint.X(), -theLocation.X());
        double y = add(thePoint.Y(), -theLocation.Y());
        double z = add(thePoint.Z(), -theLocation.Z());

        theOutInertia.Mass = add(theOutInertia.Mass, ds);

        double XdS = mult(x, ds);
        double YdS = mult(y, ds);
        double ZdS = mult(z, ds);

        theOutInertia.Ix = add(theOutInertia.Ix, XdS);
        theOutInertia.Iy = add(theOutInertia.Iy, YdS);
        theOutInertia.Iz = add(theOutInertia.Iz, ZdS);
        theOutInertia.Ixy = add(theOutInertia.Ixy, mult(x, YdS));
        theOutInertia.Iyz = add(theOutInertia.Iyz, mult(y, ZdS));
        theOutInertia.Ixz = add(theOutInertia.Ixz, mult(x, ZdS));

        double XXdS = mult(x, XdS);
        double YYdS = mult(y, YdS);
        double ZZdS = mult(z, ZdS);

        theOutInertia.Ixx = add(theOutInertia.Ixx, add(YYdS, ZZdS));
        theOutInertia.Iyy = add(theOutInertia.Iyy, add(XXdS, ZZdS));
        theOutInertia.Izz = add(theOutInertia.Izz, add(XXdS, YYdS));
    }

    private void checkBounds(double theU1, double theU2, double theV1, double theV2)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theU1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(theU2) || global::Kerf.Engine.Foundation.Precision.IsInfinite(theV1)
            || global::Kerf.Engine.Foundation.Precision.IsInfinite(theV2))
        {
            myIsInf = true;
        }
    }

    private void addAndRestoreInertia(in Inertia theInInertia, ref Inertia theOutInertia)
    {
        theOutInertia.Mass = add(theOutInertia.Mass, theInInertia.Mass);
        theOutInertia.Ix = add(theOutInertia.Ix, theInInertia.Ix);
        theOutInertia.Iy = add(theOutInertia.Iy, theInInertia.Iy);
        theOutInertia.Iz = add(theOutInertia.Iz, theInInertia.Iz);
        theOutInertia.Ixx = add(theOutInertia.Ixx, theInInertia.Ixx);
        theOutInertia.Iyy = add(theOutInertia.Iyy, theInInertia.Iyy);
        theOutInertia.Izz = add(theOutInertia.Izz, theInInertia.Izz);
        theOutInertia.Ixy = add(theOutInertia.Ixy, theInInertia.Ixy);
        theOutInertia.Ixz = add(theOutInertia.Ixz, theInInertia.Ixz);
        theOutInertia.Iyz = add(theOutInertia.Iyz, theInInertia.Iyz);
    }

    private void multAndRestoreInertia(double theValue, ref Inertia theInOutInertia)
    {
        theInOutInertia.Mass = mult(theInOutInertia.Mass, theValue);
        theInOutInertia.Ix = mult(theInOutInertia.Ix, theValue);
        theInOutInertia.Iy = mult(theInOutInertia.Iy, theValue);
        theInOutInertia.Iz = mult(theInOutInertia.Iz, theValue);
        theInOutInertia.Ixx = mult(theInOutInertia.Ixx, theValue);
        theInOutInertia.Iyy = mult(theInOutInertia.Iyy, theValue);
        theInOutInertia.Izz = mult(theInOutInertia.Izz, theValue);
        theInOutInertia.Ixy = mult(theInOutInertia.Ixy, theValue);
        theInOutInertia.Ixz = mult(theInOutInertia.Ixz, theValue);
        theInOutInertia.Iyz = mult(theInOutInertia.Iyz, theValue);
    }

    private static void convert(in Inertia theInertia,
                                ref Point3D theOutGravityCenter,
                                ref Matrix3x3 theOutMatrixOfInertia,
                                ref double theOutMass)
    {
        if (Math.Abs(theInertia.Mass) >= EPS_DIM)
        {
            double anInvMass = 1.0 / theInertia.Mass;
            theOutGravityCenter.SetX(theInertia.Ix * anInvMass);
            theOutGravityCenter.SetY(theInertia.Iy * anInvMass);
            theOutGravityCenter.SetZ(theInertia.Iz * anInvMass);

            theOutMass = theInertia.Mass;
        }
        else
        {
            theOutMass = 0.0;
            theOutGravityCenter.SetCoord(0.0, 0.0, 0.0);
        }

        theOutMatrixOfInertia = new Matrix3x3(new Xyz(theInertia.Ixx, -theInertia.Ixy, -theInertia.Ixz),
                                           new Xyz(-theInertia.Ixy, theInertia.Iyy, -theInertia.Iyz),
                                           new Xyz(-theInertia.Ixz, -theInertia.Iyz, theInertia.Izz));
    }

    private static void convert(in Inertia theInertia,
                                ReadOnlySpan<double> theCoeff,
                                bool theIsByPoint,
                                ref Point3D theOutGravityCenter,
                                ref Matrix3x3 theOutMatrixOfInertia,
                                ref double theOutMass)
    {
        convert(theInertia, ref theOutGravityCenter, ref theOutMatrixOfInertia, ref theOutMass);
        if (Math.Abs(theInertia.Mass) >= EPS_DIM && theIsByPoint)
        {
            double anInvMass = 1.0 / theInertia.Mass;
            if (theIsByPoint)
            {
                theOutGravityCenter.SetX(theCoeff[0] + theInertia.Ix * anInvMass);
                theOutGravityCenter.SetY(theCoeff[1] + theInertia.Iy * anInvMass);
                theOutGravityCenter.SetZ(theCoeff[2] + theInertia.Iz * anInvMass);
            }
            else
            {
                theOutGravityCenter.SetX(theInertia.Ix * anInvMass);
                theOutGravityCenter.SetY(theInertia.Iy * anInvMass);
                theOutGravityCenter.SetZ(theInertia.Iz * anInvMass);
            }

            theOutMass = theInertia.Mass;
        }
        else
        {
            // OCCT: a region bounded by a plane always gets a null mass and centre here (occt-issues.md)
            theOutMass = 0.0;
            theOutGravityCenter.SetCoord(0.0, 0.0, 0.0);
        }

        theOutMatrixOfInertia = new Matrix3x3(new Xyz(theInertia.Ixx, theInertia.Ixy, theInertia.Ixz),
                                           new Xyz(theInertia.Ixy, theInertia.Iyy, theInertia.Iyz),
                                           new Xyz(theInertia.Ixz, theInertia.Iyz, theInertia.Izz));
    }

    /// <summary>
    /// The global properties of the region between the bounded face and a point (<paramref name="theIsByPoint"/>,
    /// <paramref name="theCoeff"/> the shift of the point from <paramref name="theLocation"/>) or a plane (its
    /// coefficients, the constant shifted to the location), or of the face itself, by adaptive Gauss integration.
    /// With <paramref name="theEps"/> above 0.001 the integration isn't adaptive.
    /// </summary>
    /// <returns>The relative error, |(M(i+1) - M(i)) / M(i+1)| for two successive steps.</returns>
    public double Compute(BRepGProp_Face theSurface,
                          BRepGProp_Domain theDomain,
                          in Point3D theLocation,
                          double theEps,
                          ReadOnlySpan<double> theCoeff,
                          bool theIsByPoint,
                          ref double theOutMass,
                          ref Point3D theOutGravityCenter,
                          ref Matrix3x3 theOutInertia)
    {
        bool isErrorCalculation = 0.0 > theEps || theEps < 0.001;
        bool isVerifyComputation = 0.0 < theEps && theEps < 0.001;

        double anEpsilon = Math.Abs(theEps);

        Inertia anInertia = new();
        NCollection_Array1<Inertia> anInertiaL = new(1, SM);
        NCollection_Array1<Inertia> anInertiaU = new(1, SM);

        // Prepare Gauss points and weights
        var LGaussP = new math_Vector[2];
        var LGaussW = new math_Vector[2];
        var UGaussP = new math_Vector[2];
        var UGaussW = new math_Vector[2];

        int aNbGaussPoint = RealToInt(Math.Ceiling(ERROR_ALGEBR_RATIO * GPM));

        LGaussP[0] = new math_Vector(1, GPM);
        LGaussP[1] = new math_Vector(1, aNbGaussPoint);
        LGaussW[0] = new math_Vector(1, GPM);
        LGaussW[1] = new math_Vector(1, aNbGaussPoint);

        UGaussP[0] = new math_Vector(1, GPM);
        UGaussP[1] = new math_Vector(1, aNbGaussPoint);
        UGaussW[0] = new math_Vector(1, GPM);
        UGaussW[1] = new math_Vector(1, aNbGaussPoint);

        var L1 = new math_Vector(1, SM);
        var L2 = new math_Vector(1, SM);
        var U1 = new math_Vector(1, SM);
        var U2 = new math_Vector(1, SM);

        var ErrL = new math_Vector(1, SM, 0.0);
        var ErrU = new math_Vector(1, SM, 0.0);
        math_Vector? ErrUL = new math_Vector(1, SM, 0.0);

        // Face parametrization in U and V direction
        double BV1, BV2, BU1, BU2;
        theSurface.Bounds(out BU1, out BU2, out BV1, out BV2);
        checkBounds(BU1, BU2, BV1, BV2);

        //
        int NumSubs = SUBS_POWER;
        TopoDS_Face aF = theSurface.GetFace();
        bool isNaturalRestriction = (aF.NbChildren() == 0); //theSurface.NaturalRestriction();

        double CIx, CIy, CIz, CIxy, CIxz, CIyz;
        Span<double> CDim = stackalloc double[2];
        Span<double> CIxx = stackalloc double[2];
        Span<double> CIyy = stackalloc double[2];
        Span<double> CIzz = stackalloc double[2];

        // Boundary curve parametrization
        double u1 = BU1, u2, l1, l2, lm, lr, l, v;

        // On the boundary curve u-v
        Point2D Puv;
        Vector2D Vuv;
        double Dul; // Dul = Du / Dl

        int iLS, iLSubEnd, iGL, iGLEnd, iL, kL, kLEnd, IL, JL;
        Span<int> NbLGaussP = stackalloc int[2];
        Span<int> LRange = stackalloc int[2];
        int i, iUSubEnd, kU, kUEnd, IU, JU;
        Span<int> NbUGaussP = stackalloc int[2];
        Span<int> URange = stackalloc int[2];
        int UMaxSubs, LMaxSubs;

        double ErrorU, ErrorL, ErrorLMax = 0.0, Eps = 0.0, EpsL = 0.0, EpsU = 0.0;
        iGLEnd = isErrorCalculation ? 2 : 1;

        NbUGaussP[0] = theSurface.SIntOrder(anEpsilon);
        NbUGaussP[1] = RealToInt(Math.Ceiling(ERROR_ALGEBR_RATIO * NbUGaussP[0]));

        NumericTables.GaussPoints(NbUGaussP[0], UGaussP[0]);
        NumericTables.GaussWeights(NbUGaussP[0], UGaussW[0]);
        NumericTables.GaussPoints(NbUGaussP[1], UGaussP[1]);
        NumericTables.GaussWeights(NbUGaussP[1], UGaussW[1]);

        int aNbUSubs = theSurface.SUIntSubs();
        var UKnots = new NCollection_Array1<double>(1, aNbUSubs + 1);
        theSurface.UKnots(UKnots);

        Span<Inertia> aLocal = stackalloc Inertia[2];

        while (isNaturalRestriction || theDomain.More())
        {
            if (isNaturalRestriction)
            {
                NbLGaussP[0] = Min(2 * NbUGaussP[0], NumericTables.GaussPointsMax());
            }
            else
            {
                if (!theSurface.Load(theDomain.Value()))
                {
                    return global::Kerf.Engine.Foundation.Precision.Infinite();
                }

                NbLGaussP[0] = theSurface.LIntOrder(anEpsilon);
            }

            NbLGaussP[1] = RealToInt(Math.Ceiling(ERROR_ALGEBR_RATIO * NbLGaussP[0]));

            NumericTables.GaussPoints(NbLGaussP[0], LGaussP[0]);
            NumericTables.GaussWeights(NbLGaussP[0], LGaussW[0]);
            NumericTables.GaussPoints(NbLGaussP[1], LGaussP[1]);
            NumericTables.GaussWeights(NbLGaussP[1], LGaussW[1]);

            int aNbLSubs = isNaturalRestriction ? theSurface.SVIntSubs() : theSurface.LIntSubs();
            var LKnots = new NCollection_Array1<double>(1, aNbLSubs + 1);

            if (isNaturalRestriction)
            {
                theSurface.VKnots(LKnots);
                l1 = BV1;
                l2 = BV2;
            }
            else
            {
                theSurface.LKnots(LKnots);
                l1 = theSurface.FirstParameter();
                l2 = theSurface.LastParameter();
            }

            ErrorL = 0.0;
            kLEnd = 1;
            JL = 0;

            if (Math.Abs(l2 - l1) > EPS_PARAM)
            {
                iLSubEnd = FillIntervalBounds(l1, l2, LKnots, NumSubs, ref anInertiaL, ref L1, ref L2, ref ErrL, ref ErrUL);
                LMaxSubs = BRepGProp_Gauss.MaxSubs(iLSubEnd);

                if (LMaxSubs > SM)
                {
                    LMaxSubs = SM;
                }

                BRepGProp_Gauss.InitMass(0.0, 1, LMaxSubs, anInertiaL);
                BRepGProp_Gauss.Init(ErrL, 0.0, 1, LMaxSubs);
                BRepGProp_Gauss.Init(ErrUL!, 0.0, 1, LMaxSubs);

                do // while: L
                {
                    if (++JL > iLSubEnd)
                    {
                        LRange[0] = IL = ErrL.Max();
                        LRange[1] = JL;
                        L1.Value(JL) = (L1.Value(IL) + L2.Value(IL)) * 0.5;
                        L2.Value(JL) = L2.Value(IL);
                        L2.Value(IL) = L1.Value(JL);
                    }
                    else
                    {
                        LRange[0] = IL = JL;
                    }

                    if (JL == LMaxSubs || Math.Abs(L2.Value(JL) - L1.Value(JL)) < EPS_PARAM)
                    {
                        if (kLEnd == 1)
                        {
                            anInertiaL.ChangeValue(JL).Reset();
                            ErrL.Value(JL) = 0.0;
                        }
                        else
                        {
                            --JL;
                            EpsL = ErrorL;
                            Eps = EpsL / 0.9;
                            break;
                        }
                    }
                    else
                    {
                        for (kL = 0; kL < kLEnd; kL++)
                        {
                            iLS = LRange[kL];
                            lm = 0.5 * (L2.Value(iLS) + L1.Value(iLS));
                            lr = 0.5 * (L2.Value(iLS) - L1.Value(iLS));

                            CIx = CIy = CIz = CIxy = CIxz = CIyz = 0.0;

                            for (iGL = 0; iGL < iGLEnd; ++iGL)
                            {
                                CDim[iGL] = CIxx[iGL] = CIyy[iGL] = CIzz[iGL] = 0.0;

                                for (iL = 1; iL <= NbLGaussP[iGL]; iL++)
                                {
                                    l = lm + lr * LGaussP[iGL].Value(iL);
                                    if (isNaturalRestriction)
                                    {
                                        v = l;
                                        u2 = BU2;
                                        Dul = LGaussW[iGL].Value(iL);
                                    }
                                    else
                                    {
                                        theSurface.D12d(l, out Puv, out Vuv);
                                        Dul = Vuv.Y() * LGaussW[iGL].Value(iL); // Dul = Du / Dl

                                        if (Math.Abs(Dul) < EPS_PARAM)
                                        {
                                            continue;
                                        }

                                        v = Puv.Y();
                                        u2 = Puv.X();

                                        // Check on cause out off bounds of value current parameter
                                        if (v < BV1)
                                        {
                                            v = BV1;
                                        }
                                        else if (v > BV2)
                                        {
                                            v = BV2;
                                        }

                                        if (u2 < BU1)
                                        {
                                            u2 = BU1;
                                        }
                                        else if (u2 > BU2)
                                        {
                                            u2 = BU2;
                                        }
                                    }

                                    ErrUL!.Value(iLS) = 0.0;
                                    kUEnd = 1;
                                    JU = 0;

                                    if (Math.Abs(u2 - u1) < EPS_PARAM)
                                    {
                                        continue;
                                    }

                                    math_Vector? aDummy = null;
                                    iUSubEnd = FillIntervalBounds(u1, u2, UKnots, NumSubs, ref anInertiaU, ref U1, ref U2, ref ErrU, ref aDummy);

                                    UMaxSubs = BRepGProp_Gauss.MaxSubs(iUSubEnd);

                                    if (UMaxSubs > SM)
                                    {
                                        UMaxSubs = SM;
                                    }

                                    BRepGProp_Gauss.InitMass(0.0, 1, UMaxSubs, anInertiaU);
                                    BRepGProp_Gauss.Init(ErrU, 0.0, 1, UMaxSubs);
                                    ErrorU = 0.0;

                                    do
                                    { // while: U
                                        if (++JU > iUSubEnd)
                                        {
                                            URange[0] = IU = ErrU.Max();
                                            URange[1] = JU;

                                            U1.Value(JU) = (U1.Value(IU) + U2.Value(IU)) * 0.5;
                                            U2.Value(JU) = U2.Value(IU);
                                            U2.Value(IU) = U1.Value(JU);
                                        }
                                        else
                                        {
                                            URange[0] = IU = JU;
                                        }

                                        if (JU == UMaxSubs || Math.Abs(U2.Value(JU) - U1.Value(JU)) < EPS_PARAM)
                                        {
                                            if (kUEnd == 1)
                                            {
                                                ErrU.Value(JU) = 0.0;
                                                anInertiaU.ChangeValue(JU).Reset();
                                            }
                                            else
                                            {
                                                --JU;
                                                EpsU = ErrorU;
                                                Eps = 10.0 * EpsU * Math.Abs((u2 - u1) * Dul);
                                                EpsL = 0.9 * Eps;
                                                break;
                                            }
                                        }
                                        else
                                        {
                                            Point3D aPoint;
                                            Vector3D aNormal;

                                            for (kU = 0; kU < kUEnd; ++kU)
                                            {
                                                aLocal.Clear();
                                                int iUS = URange[kU];
                                                int aLength = iGLEnd - iGL;

                                                double um = 0.5 * (U2.Value(iUS) + U1.Value(iUS));
                                                double ur = 0.5 * (U2.Value(iUS) - U1.Value(iUS));

                                                for (int iGU = 0; iGU < aLength; ++iGU)
                                                {
                                                    for (int iU = 1; iU <= NbUGaussP[iGU]; ++iU)
                                                    {
                                                        double w = UGaussW[iGU].Value(iU);
                                                        double u = um + ur * UGaussP[iGU].Value(iU);

                                                        theSurface.Normal(u, v, out aPoint, out aNormal);

                                                        if (myType == BRepGProp_GaussType.Vinert)
                                                        {
                                                            computeVInertiaOfElementaryPart(aPoint,
                                                                                            aNormal,
                                                                                            theLocation,
                                                                                            w,
                                                                                            theCoeff,
                                                                                            theIsByPoint,
                                                                                            ref aLocal[iGU]);
                                                        }
                                                        else
                                                        {
                                                            if (iGU > 0)
                                                            {
                                                                aLocal[iGU].Mass += (w * aNormal.Magnitude());
                                                            }
                                                            else
                                                            {
                                                                computeSInertiaOfElementaryPart(aPoint,
                                                                                                aNormal,
                                                                                                theLocation,
                                                                                                w,
                                                                                                ref aLocal[iGU]);
                                                            }
                                                        }
                                                    }
                                                }

                                                ref Inertia anUI = ref anInertiaU.ChangeValue(iUS);

                                                anUI.Mass = mult(aLocal[0].Mass, ur);

                                                if (myType == BRepGProp_GaussType.Vinert)
                                                {
                                                    anUI.Ixx = mult(aLocal[0].Ixx, ur);
                                                    anUI.Iyy = mult(aLocal[0].Iyy, ur);
                                                    anUI.Izz = mult(aLocal[0].Izz, ur);
                                                }

                                                if (iGL > 0)
                                                {
                                                    continue;
                                                }

                                                double aDMass = Math.Abs(aLocal[1].Mass - aLocal[0].Mass);

                                                if (myType == BRepGProp_GaussType.Vinert)
                                                {
                                                    aLocal[1].Ixx = Math.Abs(aLocal[1].Ixx - aLocal[0].Ixx);
                                                    aLocal[1].Iyy = Math.Abs(aLocal[1].Iyy - aLocal[0].Iyy);
                                                    aLocal[1].Izz = Math.Abs(aLocal[1].Izz - aLocal[0].Izz);

                                                    anUI.Ix = mult(aLocal[0].Ix, ur);
                                                    anUI.Iy = mult(aLocal[0].Iy, ur);
                                                    anUI.Iz = mult(aLocal[0].Iz, ur);

                                                    anUI.Ixy = mult(aLocal[0].Ixy, ur);
                                                    anUI.Ixz = mult(aLocal[0].Ixz, ur);
                                                    anUI.Iyz = mult(aLocal[0].Iyz, ur);

                                                    ErrU.Value(iUS) = mult(aDMass, ur);
                                                }
                                                else
                                                {
                                                    anUI.Ix = mult(aLocal[0].Ix, ur);
                                                    anUI.Iy = mult(aLocal[0].Iy, ur);
                                                    anUI.Iz = mult(aLocal[0].Iz, ur);
                                                    anUI.Ixx = mult(aLocal[0].Ixx, ur);
                                                    anUI.Iyy = mult(aLocal[0].Iyy, ur);
                                                    anUI.Izz = mult(aLocal[0].Izz, ur);
                                                    anUI.Ixy = mult(aLocal[0].Ixy, ur);
                                                    anUI.Ixz = mult(aLocal[0].Ixz, ur);
                                                    anUI.Iyz = mult(aLocal[0].Iyz, ur);

                                                    ErrU.Value(iUS) = mult(aDMass, ur);
                                                }
                                            }
                                        }

                                        if (JU == iUSubEnd)
                                        {
                                            kUEnd = 2;
                                            ErrorU = ErrU.Value(ErrU.Max());
                                        }
                                    } while ((ErrorU - EpsU > 0.0 && EpsU != 0.0) || kUEnd == 1);

                                    for (i = 1; i <= JU; ++i)
                                    {
                                        Inertia anIU = anInertiaU.Value(i);

                                        CDim[iGL] = add(CDim[iGL], mult(anIU.Mass, Dul));
                                        CIxx[iGL] = add(CIxx[iGL], mult(anIU.Ixx, Dul));
                                        CIyy[iGL] = add(CIyy[iGL], mult(anIU.Iyy, Dul));
                                        CIzz[iGL] = add(CIzz[iGL], mult(anIU.Izz, Dul));
                                    }

                                    if (iGL > 0)
                                    {
                                        continue;
                                    }

                                    ErrUL!.Value(iLS) = ErrorU * Math.Abs((u2 - u1) * Dul);

                                    for (i = 1; i <= JU; ++i)
                                    {
                                        Inertia anIU = anInertiaU.Value(i);

                                        CIx = add(CIx, mult(anIU.Ix, Dul));
                                        CIy = add(CIy, mult(anIU.Iy, Dul));
                                        CIz = add(CIz, mult(anIU.Iz, Dul));

                                        CIxy = add(CIxy, mult(anIU.Ixy, Dul));
                                        CIxz = add(CIxz, mult(anIU.Ixz, Dul));
                                        CIyz = add(CIyz, mult(anIU.Iyz, Dul));
                                    }
                                } // for: iL
                            } // for: iGL

                            ref Inertia aLI = ref anInertiaL.ChangeValue(iLS);

                            aLI.Mass = mult(CDim[0], lr);
                            aLI.Ixx = mult(CIxx[0], lr);
                            aLI.Iyy = mult(CIyy[0], lr);
                            aLI.Izz = mult(CIzz[0], lr);

                            if (iGLEnd == 2)
                            {
                                double aSubDim = Math.Abs(CDim[1] - CDim[0]);

                                if (myType == BRepGProp_GaussType.Vinert)
                                {
                                    ErrorU = ErrUL!.Value(iLS);

                                    CIxx[1] = Math.Abs(CIxx[1] - CIxx[0]);
                                    CIyy[1] = Math.Abs(CIyy[1] - CIyy[0]);
                                    CIzz[1] = Math.Abs(CIzz[1] - CIzz[0]);

                                    ErrL.Value(iLS) = add(mult(aSubDim, lr), ErrorU);
                                }
                                else
                                {
                                    ErrL.Value(iLS) = add(mult(aSubDim, lr), ErrUL!.Value(iLS));
                                }
                            }

                            aLI.Ix = mult(CIx, lr);
                            aLI.Iy = mult(CIy, lr);
                            aLI.Iz = mult(CIz, lr);

                            aLI.Ixy = mult(CIxy, lr);
                            aLI.Ixz = mult(CIxz, lr);
                            aLI.Iyz = mult(CIyz, lr);
                        } // for: (kL)iLS
                    }

                    // Calculate/correct epsilon of computation by current value of dim
                    // That is need for not spend time for
                    if (JL == iLSubEnd)
                    {
                        kLEnd = 2;

                        double DDim = 0.0;
                        for (i = 1; i <= JL; ++i)
                        {
                            DDim += anInertiaL.Value(i).Mass;
                        }

                        DDim = Math.Abs(DDim * anEpsilon);

                        if (DDim > Eps)
                        {
                            Eps = DDim;
                            EpsL = 0.9 * Eps;
                        }
                    }

                    if (kLEnd == 2)
                    {
                        ErrorL = ErrL.Value(ErrL.Max());
                    }
                } while ((ErrorL - EpsL > 0.0 && isVerifyComputation) || kLEnd == 1);

                for (i = 1; i <= JL; i++)
                {
                    addAndRestoreInertia(anInertiaL.Value(i), ref anInertia);
                }

                ErrorLMax = Max(ErrorLMax, ErrorL);
            }

            if (isNaturalRestriction)
            {
                break;
            }

            theDomain.Next();
        }

        if (myType == BRepGProp_GaussType.Vinert)
        {
            convert(anInertia, theCoeff, theIsByPoint, ref theOutGravityCenter, ref theOutInertia, ref theOutMass);
        }
        else
        {
            convert(anInertia, ref theOutGravityCenter, ref theOutInertia, ref theOutMass);
        }

        if (iGLEnd == 2)
        {
            if (theOutMass != 0.0)
            {
                Eps = ErrorLMax / Math.Abs(theOutMass);
            }
            else
            {
                Eps = 0.0;
            }
        }
        else
        {
            Eps = anEpsilon;
        }

        return Eps;
    }

    /// <summary>The global properties of the bounded face, by adaptive Gauss integration (see the general form).</summary>
    /// <exception cref="ProgramErrorException">The integrator isn't of type Sinert.</exception>
    public double Compute(BRepGProp_Face theSurface,
                          BRepGProp_Domain theDomain,
                          in Point3D theLocation,
                          double theEps,
                          ref double theOutMass,
                          ref Point3D theOutGravityCenter,
                          ref Matrix3x3 theOutInertia)
    {
        Standard_ASSERT_RAISE(myType == BRepGProp_GaussType.Sinert, "BRepGProp_Gauss: Incorrect type");

        return Compute(theSurface,
                       theDomain,
                       theLocation,
                       theEps,
                       ReadOnlySpan<double>.Empty,
                       true,
                       ref theOutMass,
                       ref theOutGravityCenter,
                       ref theOutInertia);
    }

    /// <summary>
    /// The global properties of the bounded face, by non-adaptive Gauss integration with numbers of points chosen by
    /// the types of the surface and the curves. Returns without writing the outputs if an edge has no pcurve.
    /// </summary>
    /// <exception cref="ProgramErrorException">The integrator isn't of type Sinert.</exception>
    public void Compute(BRepGProp_Face theSurface,
                        BRepGProp_Domain theDomain,
                        in Point3D theLocation,
                        ref double theOutMass,
                        ref Point3D theOutGravityCenter,
                        ref Matrix3x3 theOutInertia)
    {
        Standard_ASSERT_RAISE(myType == BRepGProp_GaussType.Sinert, "BRepGProp_Gauss: Incorrect type");

        double u1, u2, v1, v2;
        theSurface.Bounds(out u1, out u2, out v1, out v2);
        checkBounds(u1, u2, v1, v2);

        int NbUGaussgp_Pnts = Min(theSurface.UIntegrationOrder(), NumericTables.GaussPointsMax());

        int NbVGaussgp_Pnts = Min(theSurface.VIntegrationOrder(), NumericTables.GaussPointsMax());

        int NbGaussgp_Pnts = Max(NbUGaussgp_Pnts, NbVGaussgp_Pnts);

        // Number of Gauss points for the integration on the face
        var GaussSPV = new math_Vector(1, NbGaussgp_Pnts);
        var GaussSWV = new math_Vector(1, NbGaussgp_Pnts);
        NumericTables.GaussPoints(NbGaussgp_Pnts, GaussSPV);
        NumericTables.GaussWeights(NbGaussgp_Pnts, GaussSWV);

        Inertia anInertia = new();
        for (; theDomain.More(); theDomain.Next())
        {
            if (!theSurface.Load(theDomain.Value()))
            {
                return;
            }

            int NbCGaussgp_Pnts = Min(theSurface.IntegrationOrder(), NumericTables.GaussPointsMax());

            NbCGaussgp_Pnts = Max(NbCGaussgp_Pnts, NbGaussgp_Pnts);

            var GaussCP = new math_Vector(1, NbCGaussgp_Pnts);
            var GaussCW = new math_Vector(1, NbCGaussgp_Pnts);
            NumericTables.GaussPoints(NbCGaussgp_Pnts, GaussCP);
            NumericTables.GaussWeights(NbCGaussgp_Pnts, GaussCW);

            double l1 = theSurface.FirstParameter();
            double l2 = theSurface.LastParameter();
            double lm = 0.5 * (l2 + l1);
            double lr = 0.5 * (l2 - l1);

            Inertia aCInertia = new();
            for (int i = 1; i <= NbCGaussgp_Pnts; ++i)
            {
                double l = lm + lr * GaussCP[i];

                theSurface.D12d(l, out Point2D Puv, out Vector2D Vuv);

                double v = Puv.Y();
                u2 = Puv.X();

                double Dul = Vuv.Y() * GaussCW[i];
                double um = 0.5 * (u2 + u1);
                double ur = 0.5 * (u2 - u1);

                Inertia aLocalInertia = new();
                for (int j = 1; j <= NbGaussgp_Pnts; ++j)
                {
                    double u = add(um, mult(ur, GaussSPV[j]));

                    double aWeight = Dul * GaussSWV[j];

                    theSurface.Normal(u, v, out Point3D aPoint, out Vector3D aNormal);

                    computeSInertiaOfElementaryPart(aPoint, aNormal, theLocation, aWeight, ref aLocalInertia);
                }

                multAndRestoreInertia(ur, ref aLocalInertia);
                addAndRestoreInertia(aLocalInertia, ref aCInertia);
            }

            multAndRestoreInertia(lr, ref aCInertia);
            addAndRestoreInertia(aCInertia, ref anInertia);
        }

        convert(anInertia, ref theOutGravityCenter, ref theOutInertia, ref theOutMass);
    }

    /// <summary>
    /// The global properties of the region between the bounded face and a point or a plane (see the adaptive form for
    /// the coefficients), by non-adaptive Gauss integration. Returns without writing the outputs if an edge has no
    /// pcurve.
    /// </summary>
    /// <exception cref="ProgramErrorException">The integrator isn't of type Vinert.</exception>
    public void Compute(BRepGProp_Face theSurface,
                        BRepGProp_Domain theDomain,
                        in Point3D theLocation,
                        ReadOnlySpan<double> theCoeff,
                        bool theIsByPoint,
                        ref double theOutMass,
                        ref Point3D theOutGravityCenter,
                        ref Matrix3x3 theOutInertia)
    {
        Standard_ASSERT_RAISE(myType == BRepGProp_GaussType.Vinert, "BRepGProp_Gauss: Incorrect type");

        double u1, v1, u2, v2;
        theSurface.Bounds(out u1, out u2, out v1, out v2);
        checkBounds(u1, u2, v1, v2);

        double _u2 = u2; // OCC104

        Inertia anInertia = new();
        for (; theDomain.More(); theDomain.Next())
        {
            if (!theSurface.Load(theDomain.Value()))
            {
                return;
            }

            int aVNbCGaussgp_Pnts = theSurface.VIntegrationOrder();

            int aNbGaussgp_Pnts = Min(Max(theSurface.IntegrationOrder(), aVNbCGaussgp_Pnts), NumericTables.GaussPointsMax());

            var GaussP = new math_Vector(1, aNbGaussgp_Pnts);
            var GaussW = new math_Vector(1, aNbGaussgp_Pnts);
            NumericTables.GaussPoints(aNbGaussgp_Pnts, GaussP);
            NumericTables.GaussWeights(aNbGaussgp_Pnts, GaussW);

            double l1 = theSurface.FirstParameter();
            double l2 = theSurface.LastParameter();
            double lm = 0.5 * (l2 + l1);
            double lr = 0.5 * (l2 - l1);

            Inertia aCInertia = new();
            for (int i = 1; i <= aNbGaussgp_Pnts; ++i)
            {
                double l = lm + lr * GaussP[i];

                theSurface.D12d(l, out Point2D Puv, out Vector2D Vuv);

                u2 = Puv.X();
                u2 = Min(Max(u1, u2), _u2); // OCC104
                double v = Min(Max(Puv.Y(), v1), v2);

                double Dul = Vuv.Y() * GaussW[i];
                double um = 0.5 * (u2 + u1);
                double ur = 0.5 * (u2 - u1);

                Inertia aLocalInertia = new();
                for (int j = 1; j <= aNbGaussgp_Pnts; ++j)
                {
                    double u = um + ur * GaussP[j];
                    double aWeight = Dul * GaussW[j];

                    theSurface.Normal(u, v, out Point3D aPoint, out Vector3D aNormal);

                    computeVInertiaOfElementaryPart(aPoint,
                                                    aNormal,
                                                    theLocation,
                                                    aWeight,
                                                    theCoeff,
                                                    theIsByPoint,
                                                    ref aLocalInertia);
                }

                multAndRestoreInertia(ur, ref aLocalInertia);
                addAndRestoreInertia(aLocalInertia, ref aCInertia);
            }

            multAndRestoreInertia(lr, ref aCInertia);
            addAndRestoreInertia(aCInertia, ref anInertia);
        }

        convert(anInertia, theCoeff, theIsByPoint, ref theOutGravityCenter, ref theOutInertia, ref theOutMass);
    }

    /// <summary>
    /// The global properties of the region between the face, unbounded (its natural restriction), and a point or a
    /// plane, or of the face itself, by non-adaptive Gauss integration.
    /// </summary>
    public void Compute(BRepGProp_Face theSurface,
                        in Point3D theLocation,
                        ReadOnlySpan<double> theCoeff,
                        bool theIsByPoint,
                        ref double theOutMass,
                        ref Point3D theOutGravityCenter,
                        ref Matrix3x3 theOutInertia)
    {
        double LowerU, UpperU, LowerV, UpperV;
        theSurface.Bounds(out LowerU, out UpperU, out LowerV, out UpperV);
        checkBounds(LowerU, UpperU, LowerV, UpperV);

        int UOrder = Min(theSurface.UIntegrationOrder(), NumericTables.GaussPointsMax());
        int VOrder = Min(theSurface.VIntegrationOrder(), NumericTables.GaussPointsMax());

        // Gauss points and weights
        var GaussPU = new math_Vector(1, UOrder);
        var GaussWU = new math_Vector(1, UOrder);
        var GaussPV = new math_Vector(1, VOrder);
        var GaussWV = new math_Vector(1, VOrder);

        NumericTables.GaussPoints(UOrder, GaussPU);
        NumericTables.GaussWeights(UOrder, GaussWU);
        NumericTables.GaussPoints(VOrder, GaussPV);
        NumericTables.GaussWeights(VOrder, GaussWV);

        double um = 0.5 * add(UpperU, LowerU);
        double vm = 0.5 * add(UpperV, LowerV);
        double ur = 0.5 * add(UpperU, -LowerU);
        double vr = 0.5 * add(UpperV, -LowerV);

        Point3D aPoint;
        Vector3D aNormal;

        Inertia anInertia = new();
        for (int j = 1; j <= VOrder; ++j)
        {
            Inertia anInertiaOfElementaryPart = new();
            double v = add(vm, mult(vr, GaussPV[j]));

            for (int i = 1; i <= UOrder; ++i)
            {
                double aWeight = GaussWU[i];
                double u = add(um, mult(ur, GaussPU[i]));
                theSurface.Normal(u, v, out aPoint, out aNormal);

                if (myType == BRepGProp_GaussType.Vinert)
                {
                    computeVInertiaOfElementaryPart(aPoint,
                                                    aNormal,
                                                    theLocation,
                                                    aWeight,
                                                    theCoeff,
                                                    theIsByPoint,
                                                    ref anInertiaOfElementaryPart);
                }
                else // Sinert
                {
                    computeSInertiaOfElementaryPart(aPoint, aNormal, theLocation, aWeight, ref anInertiaOfElementaryPart);
                }
            }

            multAndRestoreInertia(GaussWV[j], ref anInertiaOfElementaryPart);
            addAndRestoreInertia(anInertiaOfElementaryPart, ref anInertia);
        }

        vr = mult(vr, ur);
        anInertia.Ixx = mult(vr, anInertia.Ixx);
        anInertia.Iyy = mult(vr, anInertia.Iyy);
        anInertia.Izz = mult(vr, anInertia.Izz);
        anInertia.Ixy = mult(vr, anInertia.Ixy);
        anInertia.Ixz = mult(vr, anInertia.Ixz);
        anInertia.Iyz = mult(vr, anInertia.Iyz);

        if (myType == BRepGProp_GaussType.Vinert)
        {
            convert(anInertia, theCoeff, theIsByPoint, ref theOutGravityCenter, ref theOutInertia, ref theOutMass);
        }
        else // Sinert
        {
            convert(anInertia, ref theOutGravityCenter, ref theOutInertia, ref theOutMass);
        }

        theOutMass *= vr;
    }

    /// <summary>The global properties of the face, unbounded (its natural restriction), by non-adaptive Gauss integration.</summary>
    /// <exception cref="ProgramErrorException">The integrator isn't of type Sinert.</exception>
    public void Compute(BRepGProp_Face theSurface,
                        in Point3D theLocation,
                        ref double theOutMass,
                        ref Point3D theOutGravityCenter,
                        ref Matrix3x3 theOutInertia)
    {
        Standard_ASSERT_RAISE(myType == BRepGProp_GaussType.Sinert, "BRepGProp_Gauss: Incorrect type");

        Compute(theSurface, theLocation, ReadOnlySpan<double>.Empty, true, ref theOutMass, ref theOutGravityCenter, ref theOutInertia);
    }

    // Standard_ASSERT_RAISE: always checked in the port (rules, section 5)
    private static void Standard_ASSERT_RAISE(bool theCondition, string theMessage)
    {
        if (!theCondition)
        {
            throw new ProgramErrorException(theMessage);
        }
    }
}
