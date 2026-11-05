// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Gauss.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Gauss.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The global inertia properties of a face, or of the region of 3D space between a face and a point or a plane, by
/// adaptive and non-adaptive 2D Gauss integration.
/// </summary>
/// <remarks>
/// OCCT's <c>add</c> and <c>mult</c> function pointers (switched to the infinity-aware versions when a bound of the
/// face is infinite) are a flag read by the private methods of the same names. The coefficient arrays are spans
/// (empty for OCCT's null pointer).
/// </remarks>
internal sealed class MeasurementGauss
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
    private static readonly int Gpm = NumericTables.GaussPointsMax();
    private const int SUBS_POWER = 32;
    private static readonly int Sm = SUBS_POWER * Gpm + 1;

    private readonly BRepGProp_GaussType _type; //!< Type of geometric object
    private bool _isInf;                        //!< add and mult are AddInf and MultInf

    /// <summary>An integrator for the given type of object.</summary>
    public MeasurementGauss(BRepGProp_GaussType type)
    {
        _type = type;
        _isInf = false;
    }

    // Auxiliary inner functions to perform arithmetic operations.
    private static double Add(double a, double b) => a + b;

    private static double AddInf(double a, double b)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(a))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(b))
            {
                return 0.0;
            }
            else
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(b))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(a))
            {
                return 0.0;
            }
            else
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(a))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(b))
            {
                return 0.0;
            }
            else
            {
                return -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(b))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(a))
            {
                return 0.0;
            }
            else
            {
                return -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        return a + b;
    }

    private static double Mult(double a, double b) => a * b;

    private static double MultInf(double a, double b)
    {
        if ((a == 0.0) || (b == 0.0)) // strictly zerro (without any tolerances)
        {
            return 0.0;
        }

        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(a))
        {
            if (b < 0.0)
            {
                return -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(b))
        {
            if (a < 0.0)
            {
                return -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(a))
        {
            if (b < 0.0)
            {
                return +global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                return -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(b))
        {
            if (a < 0.0)
            {
                return +global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            else
            {
                return -global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        return a * b;
    }

    // OCCT's function pointers add and mult
    private double AddCore(double a, double b) => _isInf ? AddInf(a, b) : Add(a, b);

    private double MultCore(double a, double b) => _isInf ? MultInf(a, b) : Mult(a, b);

    private static int MaxSubs(int n, int coeff = 32) =>
        IntegerLast() / coeff < n ? IntegerLast() : n * coeff + 1;

    private static void Init(math_Vector outVec, double value, int first = 0, int last = 0)
    {
        if (last - first == 0)
        {
            outVec.Init(value);
        }
        else
        {
            for (int i = first; i <= last; ++i)
            {
                outVec.Value(i) = value;
            }
        }
    }

    private static void InitMass(double value, int first, int last, NCollection_Array1<Inertia>? array)
    {
        if (array is null)
        {
            return;
        }

        int aFirst = first;
        int aLast = last;

        if (last - first == 0)
        {
            aFirst = array.Lower();
            aLast = array.Upper();
        }

        for (int i = aFirst; i <= aLast; ++i)
        {
            array.ChangeValue(i).Mass = value;
        }
    }

    private static int FillIntervalBounds(double a,
                                          double b,
                                          NCollection_Array1<double> knots,
                                          int numSubs,
                                          ref NCollection_Array1<Inertia> inerts,
                                          ref math_Vector param1,
                                          ref math_Vector param2,
                                          ref math_Vector error,
                                          ref math_Vector? commonError)
    {
        int size = Max(knots.Upper(), MaxSubs(knots.Upper() - 1, numSubs));

        if (size - 1 > param1.Upper())
        {
            inerts = new NCollection_Array1<Inertia>(1, size);
            param1 = new math_Vector(1, size);
            param2 = new math_Vector(1, size);
            error = new math_Vector(1, size, 0.0);

            if (commonError is not null)
            {
                commonError = new math_Vector(1, size, 0.0);
            }
        }

        int j = 1, k = 1;
        param1.Value(j++) = a;

        int length = knots.Upper();
        for (int i = 1; i <= length; ++i)
        {
            double kn = knots[i];
            if (a < kn)
            {
                if (kn < b)
                {
                    param1.Value(j++) = kn;
                    param2.Value(k++) = kn;
                }
                else
                {
                    break;
                }
            }
        }

        param2.Value(k) = b;
        return k;
    }

    private static void ComputeVInertiaOfElementaryPart(in Point3D point,
                                                        in Vector3D normal,
                                                        in Point3D location,
                                                        double weight,
                                                        ReadOnlySpan<double> coeff,
                                                        bool isByPoint,
                                                        ref Inertia outInertia)
    {
        double x = point.X - location.X;
        double y = point.Y - location.Y;
        double z = point.Z - location.Z;

        double xn = normal.X * weight;
        double yn = normal.Y * weight;
        double zn = normal.Z * weight;

        if (isByPoint)
        {
            /////////////////////                        ///////////////////////
            //    OFV code     //                        //    Initial code   //
            /////////////////////                        ///////////////////////
            // modified by APO

            double dv = x * xn + y * yn + z * zn; // xyz  = x * y * z;
            outInertia.Mass += dv / 3.0;       // Ixyi += zn * xyz;
            outInertia.Ix += 0.25 * x * dv;    // Iyzi += xn * xyz;
            outInertia.Iy += 0.25 * y * dv;    // Ixzi += yn * xyz;
            outInertia.Iz += 0.25 * z * dv;    // xi = x * x * x * xn / 3.0;
            x -= coeff[0];                     // yi = y * y * y * yn / 3.0;
            y -= coeff[1];                     // zi = z * z * z * zn / 3.0;
            z -= coeff[2];                     // Ixxi += (yi + zi);
            dv *= 0.2;                            // Iyyi += (xi + zi);
            outInertia.Ixy -= x * y * dv;      // Izzi += (xi + yi);
            outInertia.Iyz -= y * z * dv;      // x -= Coeff[0];
            outInertia.Ixz -= x * z * dv;      // y -= Coeff[1];
            x *= x;                               // z -= Coeff[2];
            y *= y;                               // dv = x * xn + y * yn + z * zn;
            z *= z;                               // dvi +=  dv;
            outInertia.Ixx += (y + z) * dv;    // Ixi += x * dv;
            outInertia.Iyy += (x + z) * dv;    // Iyi += y * dv;
            outInertia.Izz += (x + y) * dv;    // Izi += z * dv;
        }
        else
        { // By plane
            double s = xn * coeff[0] + yn * coeff[1] + zn * coeff[2];

            double d1 = coeff[0] * x + coeff[1] * y + coeff[2] * z - coeff[3];

            double d2 = d1 * d1;
            double d3 = d1 * d2 / 3.0;
            double dv = s * d1;

            outInertia.Mass += dv;
            outInertia.Ix += (x - (coeff[0] * d1 * 0.5)) * dv;
            outInertia.Iy += (y - (coeff[1] * d1 * 0.5)) * dv;
            outInertia.Iz += (z - (coeff[2] * d1 * 0.5)) * dv;

            double px = x - coeff[0] * d1;
            double py = y - coeff[1] * d1;
            double pz = z - coeff[2] * d1;

            x = px * px * d1 + px * coeff[0] * d2 + coeff[0] * coeff[0] * d3;
            y = py * py * d1 + py * coeff[1] * d2 + coeff[1] * coeff[1] * d3;
            z = pz * pz * d1 + pz * coeff[2] * d2 + coeff[2] * coeff[2] * d3;

            outInertia.Ixx += (y + z) * s;
            outInertia.Iyy += (x + z) * s;
            outInertia.Izz += (x + y) * s;

            d2 *= 0.5;

            x = (py * pz * d1) + (py * coeff[2] * d2) + (pz * coeff[1] * d2) + (coeff[1] * coeff[2] * d3);
            y = (px * pz * d1) + (pz * coeff[0] * d2) + (px * coeff[2] * d2) + (coeff[0] * coeff[2] * d3);
            z = (px * py * d1) + (px * coeff[1] * d2) + (py * coeff[0] * d2) + (coeff[0] * coeff[1] * d3);

            outInertia.Ixy -= z * s;
            outInertia.Iyz -= x * s;
            outInertia.Ixz -= y * s;
        }
    }

    private void ComputeSInertiaOfElementaryPart(in Point3D point,
                                                 in Vector3D normal,
                                                 in Point3D location,
                                                 double weight,
                                                 ref Inertia outInertia)
    {
        // ds - Jacobien (x, y, z) -> (u, v) = ||n||
        double ds = MultCore(normal.Magnitude, weight);
        double x = AddCore(point.X, -location.X);
        double y = AddCore(point.Y, -location.Y);
        double z = AddCore(point.Z, -location.Z);

        outInertia.Mass = AddCore(outInertia.Mass, ds);

        double xdS = MultCore(x, ds);
        double ydS = MultCore(y, ds);
        double zdS = MultCore(z, ds);

        outInertia.Ix = AddCore(outInertia.Ix, xdS);
        outInertia.Iy = AddCore(outInertia.Iy, ydS);
        outInertia.Iz = AddCore(outInertia.Iz, zdS);
        outInertia.Ixy = AddCore(outInertia.Ixy, MultCore(x, ydS));
        outInertia.Iyz = AddCore(outInertia.Iyz, MultCore(y, zdS));
        outInertia.Ixz = AddCore(outInertia.Ixz, MultCore(x, zdS));

        double xXdS = MultCore(x, xdS);
        double yYdS = MultCore(y, ydS);
        double zZdS = MultCore(z, zdS);

        outInertia.Ixx = AddCore(outInertia.Ixx, AddCore(yYdS, zZdS));
        outInertia.Iyy = AddCore(outInertia.Iyy, AddCore(xXdS, zZdS));
        outInertia.Izz = AddCore(outInertia.Izz, AddCore(xXdS, yYdS));
    }

    private void CheckBounds(double u1, double u2, double v1, double v2)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1) || global::Kerf.Engine.Foundation.Precision.IsInfinite(u2) || global::Kerf.Engine.Foundation.Precision.IsInfinite(v1)
            || global::Kerf.Engine.Foundation.Precision.IsInfinite(v2))
        {
            _isInf = true;
        }
    }

    private void AddAndRestoreInertia(in Inertia inInertia, ref Inertia outInertia)
    {
        outInertia.Mass = AddCore(outInertia.Mass, inInertia.Mass);
        outInertia.Ix = AddCore(outInertia.Ix, inInertia.Ix);
        outInertia.Iy = AddCore(outInertia.Iy, inInertia.Iy);
        outInertia.Iz = AddCore(outInertia.Iz, inInertia.Iz);
        outInertia.Ixx = AddCore(outInertia.Ixx, inInertia.Ixx);
        outInertia.Iyy = AddCore(outInertia.Iyy, inInertia.Iyy);
        outInertia.Izz = AddCore(outInertia.Izz, inInertia.Izz);
        outInertia.Ixy = AddCore(outInertia.Ixy, inInertia.Ixy);
        outInertia.Ixz = AddCore(outInertia.Ixz, inInertia.Ixz);
        outInertia.Iyz = AddCore(outInertia.Iyz, inInertia.Iyz);
    }

    private void MultAndRestoreInertia(double value, ref Inertia inOutInertia)
    {
        inOutInertia.Mass = MultCore(inOutInertia.Mass, value);
        inOutInertia.Ix = MultCore(inOutInertia.Ix, value);
        inOutInertia.Iy = MultCore(inOutInertia.Iy, value);
        inOutInertia.Iz = MultCore(inOutInertia.Iz, value);
        inOutInertia.Ixx = MultCore(inOutInertia.Ixx, value);
        inOutInertia.Iyy = MultCore(inOutInertia.Iyy, value);
        inOutInertia.Izz = MultCore(inOutInertia.Izz, value);
        inOutInertia.Ixy = MultCore(inOutInertia.Ixy, value);
        inOutInertia.Ixz = MultCore(inOutInertia.Ixz, value);
        inOutInertia.Iyz = MultCore(inOutInertia.Iyz, value);
    }

    private static void Convert(in Inertia inertia,
                                ref Point3D outGravityCenter,
                                ref Matrix3x3 outMatrixOfInertia,
                                ref double outMass)
    {
        if (Math.Abs(inertia.Mass) >= EPS_DIM)
        {
            double invMass = 1.0 / inertia.Mass;
            outGravityCenter = outGravityCenter.WithX(inertia.Ix * invMass);
            outGravityCenter = outGravityCenter.WithY(inertia.Iy * invMass);
            outGravityCenter = outGravityCenter.WithZ(inertia.Iz * invMass);

            outMass = inertia.Mass;
        }
        else
        {
            outMass = 0.0;
            outGravityCenter = outGravityCenter.WithCoord(0.0, 0.0, 0.0);
        }

        outMatrixOfInertia = new Matrix3x3(new Xyz(inertia.Ixx, -inertia.Ixy, -inertia.Ixz),
                                           new Xyz(-inertia.Ixy, inertia.Iyy, -inertia.Iyz),
                                           new Xyz(-inertia.Ixz, -inertia.Iyz, inertia.Izz));
    }

    private static void Convert(in Inertia inertia,
                                ReadOnlySpan<double> coeff,
                                bool isByPoint,
                                ref Point3D outGravityCenter,
                                ref Matrix3x3 outMatrixOfInertia,
                                ref double outMass)
    {
        global::Kerf.Engine.Measurement.MeasurementGauss.Convert(inertia, ref outGravityCenter, ref outMatrixOfInertia, ref outMass);
        // a region bounded by a plane has its mass and centre too (decision 1468; OCCT tested isByPoint here as well,
        // so it cleared them)
        if (Math.Abs(inertia.Mass) >= EPS_DIM)
        {
            double invMass = 1.0 / inertia.Mass;
            if (isByPoint)
            {
                outGravityCenter = outGravityCenter.WithX(coeff[0] + inertia.Ix * invMass);
                outGravityCenter = outGravityCenter.WithY(coeff[1] + inertia.Iy * invMass);
                outGravityCenter = outGravityCenter.WithZ(coeff[2] + inertia.Iz * invMass);
            }
            else
            {
                outGravityCenter = outGravityCenter.WithX(inertia.Ix * invMass);
                outGravityCenter = outGravityCenter.WithY(inertia.Iy * invMass);
                outGravityCenter = outGravityCenter.WithZ(inertia.Iz * invMass);
            }

            outMass = inertia.Mass;
        }
        else
        {
            outMass = 0.0;
            outGravityCenter = outGravityCenter.WithCoord(0.0, 0.0, 0.0);
        }

        outMatrixOfInertia = new Matrix3x3(new Xyz(inertia.Ixx, inertia.Ixy, inertia.Ixz),
                                           new Xyz(inertia.Ixy, inertia.Iyy, inertia.Iyz),
                                           new Xyz(inertia.Ixz, inertia.Iyz, inertia.Izz));
    }

    /// <summary>
    /// The global properties of the region between the bounded face and a point (<paramref name="isByPoint"/>,
    /// <paramref name="coeff"/> the shift of the point from <paramref name="location"/>) or a plane (its
    /// coefficients, the constant shifted to the location), or of the face itself, by adaptive Gauss integration.
    /// With <paramref name="eps"/> above 0.001 the integration isn't adaptive.
    /// </summary>
    /// <returns>The relative error, |(M(i+1) - M(i)) / M(i+1)| for two successive steps.</returns>
    public double Compute(FaceIntegrand surface,
                          Domain domain,
                          in Point3D location,
                          double eps,
                          ReadOnlySpan<double> coeff,
                          bool isByPoint,
                          ref double outMass,
                          ref Point3D outGravityCenter,
                          ref Matrix3x3 outInertia)
    {
        bool isErrorCalculation = 0.0 > eps || eps < 0.001;
        bool isVerifyComputation = 0.0 < eps && eps < 0.001;

        double epsilon = Math.Abs(eps);

        Inertia inertia = new();
        NCollection_Array1<Inertia> inertiaL = new(1, Sm);
        NCollection_Array1<Inertia> inertiaU = new(1, Sm);

        // Prepare Gauss points and weights
        var lGaussP = new math_Vector[2];
        var lGaussW = new math_Vector[2];
        var uGaussP = new math_Vector[2];
        var uGaussW = new math_Vector[2];

        int nbGaussPoint = RealToInt(Math.Ceiling(ERROR_ALGEBR_RATIO * Gpm));

        lGaussP[0] = new math_Vector(1, Gpm);
        lGaussP[1] = new math_Vector(1, nbGaussPoint);
        lGaussW[0] = new math_Vector(1, Gpm);
        lGaussW[1] = new math_Vector(1, nbGaussPoint);

        uGaussP[0] = new math_Vector(1, Gpm);
        uGaussP[1] = new math_Vector(1, nbGaussPoint);
        uGaussW[0] = new math_Vector(1, Gpm);
        uGaussW[1] = new math_Vector(1, nbGaussPoint);

        var L1 = new math_Vector(1, Sm);
        var L2 = new math_Vector(1, Sm);
        var U1 = new math_Vector(1, Sm);
        var U2 = new math_Vector(1, Sm);

        var errL = new math_Vector(1, Sm, 0.0);
        var errU = new math_Vector(1, Sm, 0.0);
        math_Vector? errUL = new math_Vector(1, Sm, 0.0);

        // Face parametrization in U and V direction
        double bv1, bv2, bu1, bu2;
        surface.Bounds(out bu1, out bu2, out bv1, out bv2);
        CheckBounds(bu1, bu2, bv1, bv2);

        //
        int numSubs = SUBS_POWER;
        Face f = surface.GetFace();
        bool isNaturalRestriction = (f.ChildCount == 0); //theSurface.NaturalRestriction();

        double cIx, cIy, cIz, cIxy, cIxz, cIyz;
        Span<double> cDim = stackalloc double[2];
        Span<double> cIxx = stackalloc double[2];
        Span<double> cIyy = stackalloc double[2];
        Span<double> cIzz = stackalloc double[2];

        // Boundary curve parametrization
        double u1 = bu1, u2, l1, l2, lm, lr, l, v;

        // On the boundary curve u-v
        Point2D puv;
        Vector2D vuv;
        double dul; // Dul = Du / Dl

        int iLS, iLSubEnd, iGL, iGLEnd, iL, kL, kLEnd, il, jl;
        Span<int> nbLGaussP = stackalloc int[2];
        Span<int> lRange = stackalloc int[2];
        int i, iUSubEnd, kU, kUEnd, IU, ju;
        Span<int> nbUGaussP = stackalloc int[2];
        Span<int> uRange = stackalloc int[2];
        int uMaxSubs, lMaxSubs;

        double errorU, errorL, errorLMax = 0.0, Eps = 0.0, epsL = 0.0, epsU = 0.0;
        iGLEnd = isErrorCalculation ? 2 : 1;

        nbUGaussP[0] = surface.SIntOrder(epsilon);
        nbUGaussP[1] = RealToInt(Math.Ceiling(ERROR_ALGEBR_RATIO * nbUGaussP[0]));

        NumericTables.GaussPoints(nbUGaussP[0], uGaussP[0]);
        NumericTables.GaussWeights(nbUGaussP[0], uGaussW[0]);
        NumericTables.GaussPoints(nbUGaussP[1], uGaussP[1]);
        NumericTables.GaussWeights(nbUGaussP[1], uGaussW[1]);

        int nbUSubs = surface.SUIntSubs();
        var uKnots = new NCollection_Array1<double>(1, nbUSubs + 1);
        surface.UKnots(uKnots);

        Span<Inertia> local = stackalloc Inertia[2];

        while (isNaturalRestriction || domain.More())
        {
            if (isNaturalRestriction)
            {
                nbLGaussP[0] = Min(2 * nbUGaussP[0], NumericTables.GaussPointsMax());
            }
            else
            {
                if (!surface.Load(domain.Value()))
                {
                    return global::Kerf.Engine.Foundation.Precision.Infinite();
                }

                nbLGaussP[0] = surface.LIntOrder(epsilon);
            }

            nbLGaussP[1] = RealToInt(Math.Ceiling(ERROR_ALGEBR_RATIO * nbLGaussP[0]));

            NumericTables.GaussPoints(nbLGaussP[0], lGaussP[0]);
            NumericTables.GaussWeights(nbLGaussP[0], lGaussW[0]);
            NumericTables.GaussPoints(nbLGaussP[1], lGaussP[1]);
            NumericTables.GaussWeights(nbLGaussP[1], lGaussW[1]);

            int nbLSubs = isNaturalRestriction ? surface.SVIntSubs() : surface.LIntSubs();
            var lKnots = new NCollection_Array1<double>(1, nbLSubs + 1);

            if (isNaturalRestriction)
            {
                surface.VKnots(lKnots);
                l1 = bv1;
                l2 = bv2;
            }
            else
            {
                surface.LKnots(lKnots);
                l1 = surface.FirstParameter();
                l2 = surface.LastParameter();
            }

            errorL = 0.0;
            kLEnd = 1;
            jl = 0;

            if (Math.Abs(l2 - l1) > EPS_PARAM)
            {
                iLSubEnd = FillIntervalBounds(l1, l2, lKnots, numSubs, ref inertiaL, ref L1, ref L2, ref errL, ref errUL);
                lMaxSubs = MeasurementGauss.MaxSubs(iLSubEnd);

                if (lMaxSubs > Sm)
                {
                    lMaxSubs = Sm;
                }

                MeasurementGauss.InitMass(0.0, 1, lMaxSubs, inertiaL);
                MeasurementGauss.Init(errL, 0.0, 1, lMaxSubs);
                MeasurementGauss.Init(errUL!, 0.0, 1, lMaxSubs);

                do // while: L
                {
                    if (++jl > iLSubEnd)
                    {
                        lRange[0] = il = errL.Max();
                        lRange[1] = jl;
                        L1.Value(jl) = (L1.Value(il) + L2.Value(il)) * 0.5;
                        L2.Value(jl) = L2.Value(il);
                        L2.Value(il) = L1.Value(jl);
                    }
                    else
                    {
                        lRange[0] = il = jl;
                    }

                    if (jl == lMaxSubs || Math.Abs(L2.Value(jl) - L1.Value(jl)) < EPS_PARAM)
                    {
                        if (kLEnd == 1)
                        {
                            inertiaL.ChangeValue(jl).Reset();
                            errL.Value(jl) = 0.0;
                        }
                        else
                        {
                            --jl;
                            epsL = errorL;
                            Eps = epsL / 0.9;
                            break;
                        }
                    }
                    else
                    {
                        for (kL = 0; kL < kLEnd; kL++)
                        {
                            iLS = lRange[kL];
                            lm = 0.5 * (L2.Value(iLS) + L1.Value(iLS));
                            lr = 0.5 * (L2.Value(iLS) - L1.Value(iLS));

                            cIx = cIy = cIz = cIxy = cIxz = cIyz = 0.0;

                            for (iGL = 0; iGL < iGLEnd; ++iGL)
                            {
                                cDim[iGL] = cIxx[iGL] = cIyy[iGL] = cIzz[iGL] = 0.0;

                                for (iL = 1; iL <= nbLGaussP[iGL]; iL++)
                                {
                                    l = lm + lr * lGaussP[iGL].Value(iL);
                                    if (isNaturalRestriction)
                                    {
                                        v = l;
                                        u2 = bu2;
                                        dul = lGaussW[iGL].Value(iL);
                                    }
                                    else
                                    {
                                        surface.D12d(l, out puv, out vuv);
                                        dul = vuv.Y * lGaussW[iGL].Value(iL); // Dul = Du / Dl

                                        if (Math.Abs(dul) < EPS_PARAM)
                                        {
                                            continue;
                                        }

                                        v = puv.Y;
                                        u2 = puv.X;

                                        // Check on cause out off bounds of value current parameter
                                        if (v < bv1)
                                        {
                                            v = bv1;
                                        }
                                        else if (v > bv2)
                                        {
                                            v = bv2;
                                        }

                                        if (u2 < bu1)
                                        {
                                            u2 = bu1;
                                        }
                                        else if (u2 > bu2)
                                        {
                                            u2 = bu2;
                                        }
                                    }

                                    errUL!.Value(iLS) = 0.0;
                                    kUEnd = 1;
                                    ju = 0;

                                    if (Math.Abs(u2 - u1) < EPS_PARAM)
                                    {
                                        continue;
                                    }

                                    math_Vector? dummy = null;
                                    iUSubEnd = FillIntervalBounds(u1, u2, uKnots, numSubs, ref inertiaU, ref U1, ref U2, ref errU, ref dummy);

                                    uMaxSubs = MeasurementGauss.MaxSubs(iUSubEnd);

                                    if (uMaxSubs > Sm)
                                    {
                                        uMaxSubs = Sm;
                                    }

                                    MeasurementGauss.InitMass(0.0, 1, uMaxSubs, inertiaU);
                                    MeasurementGauss.Init(errU, 0.0, 1, uMaxSubs);
                                    errorU = 0.0;

                                    do
                                    { // while: U
                                        if (++ju > iUSubEnd)
                                        {
                                            uRange[0] = IU = errU.Max();
                                            uRange[1] = ju;

                                            U1.Value(ju) = (U1.Value(IU) + U2.Value(IU)) * 0.5;
                                            U2.Value(ju) = U2.Value(IU);
                                            U2.Value(IU) = U1.Value(ju);
                                        }
                                        else
                                        {
                                            uRange[0] = IU = ju;
                                        }

                                        if (ju == uMaxSubs || Math.Abs(U2.Value(ju) - U1.Value(ju)) < EPS_PARAM)
                                        {
                                            if (kUEnd == 1)
                                            {
                                                errU.Value(ju) = 0.0;
                                                inertiaU.ChangeValue(ju).Reset();
                                            }
                                            else
                                            {
                                                --ju;
                                                epsU = errorU;
                                                Eps = 10.0 * epsU * Math.Abs((u2 - u1) * dul);
                                                epsL = 0.9 * Eps;
                                                break;
                                            }
                                        }
                                        else
                                        {
                                            Point3D point;
                                            Vector3D normal;

                                            for (kU = 0; kU < kUEnd; ++kU)
                                            {
                                                local.Clear();
                                                int iUS = uRange[kU];
                                                int length = iGLEnd - iGL;

                                                double um = 0.5 * (U2.Value(iUS) + U1.Value(iUS));
                                                double ur = 0.5 * (U2.Value(iUS) - U1.Value(iUS));

                                                for (int iGU = 0; iGU < length; ++iGU)
                                                {
                                                    for (int iU = 1; iU <= nbUGaussP[iGU]; ++iU)
                                                    {
                                                        double w = uGaussW[iGU].Value(iU);
                                                        double u = um + ur * uGaussP[iGU].Value(iU);

                                                        surface.Normal(u, v, out point, out normal);

                                                        if (_type == BRepGProp_GaussType.Vinert)
                                                        {
                                                            ComputeVInertiaOfElementaryPart(point,
                                                                                            normal,
                                                                                            location,
                                                                                            w,
                                                                                            coeff,
                                                                                            isByPoint,
                                                                                            ref local[iGU]);
                                                        }
                                                        else
                                                        {
                                                            if (iGU > 0)
                                                            {
                                                                local[iGU].Mass += (w * normal.Magnitude);
                                                            }
                                                            else
                                                            {
                                                                ComputeSInertiaOfElementaryPart(point,
                                                                                                normal,
                                                                                                location,
                                                                                                w,
                                                                                                ref local[iGU]);
                                                            }
                                                        }
                                                    }
                                                }

                                                ref Inertia ui = ref inertiaU.ChangeValue(iUS);

                                                ui.Mass = MultCore(local[0].Mass, ur);

                                                if (_type == BRepGProp_GaussType.Vinert)
                                                {
                                                    ui.Ixx = MultCore(local[0].Ixx, ur);
                                                    ui.Iyy = MultCore(local[0].Iyy, ur);
                                                    ui.Izz = MultCore(local[0].Izz, ur);
                                                }

                                                if (iGL > 0)
                                                {
                                                    continue;
                                                }

                                                double dMass = Math.Abs(local[1].Mass - local[0].Mass);

                                                if (_type == BRepGProp_GaussType.Vinert)
                                                {
                                                    local[1].Ixx = Math.Abs(local[1].Ixx - local[0].Ixx);
                                                    local[1].Iyy = Math.Abs(local[1].Iyy - local[0].Iyy);
                                                    local[1].Izz = Math.Abs(local[1].Izz - local[0].Izz);

                                                    ui.Ix = MultCore(local[0].Ix, ur);
                                                    ui.Iy = MultCore(local[0].Iy, ur);
                                                    ui.Iz = MultCore(local[0].Iz, ur);

                                                    ui.Ixy = MultCore(local[0].Ixy, ur);
                                                    ui.Ixz = MultCore(local[0].Ixz, ur);
                                                    ui.Iyz = MultCore(local[0].Iyz, ur);

                                                    errU.Value(iUS) = MultCore(dMass, ur);
                                                }
                                                else
                                                {
                                                    ui.Ix = MultCore(local[0].Ix, ur);
                                                    ui.Iy = MultCore(local[0].Iy, ur);
                                                    ui.Iz = MultCore(local[0].Iz, ur);
                                                    ui.Ixx = MultCore(local[0].Ixx, ur);
                                                    ui.Iyy = MultCore(local[0].Iyy, ur);
                                                    ui.Izz = MultCore(local[0].Izz, ur);
                                                    ui.Ixy = MultCore(local[0].Ixy, ur);
                                                    ui.Ixz = MultCore(local[0].Ixz, ur);
                                                    ui.Iyz = MultCore(local[0].Iyz, ur);

                                                    errU.Value(iUS) = MultCore(dMass, ur);
                                                }
                                            }
                                        }

                                        if (ju == iUSubEnd)
                                        {
                                            kUEnd = 2;
                                            errorU = errU.Value(errU.Max());
                                        }
                                    } while ((errorU - epsU > 0.0 && epsU != 0.0) || kUEnd == 1);

                                    for (i = 1; i <= ju; ++i)
                                    {
                                        Inertia iu = inertiaU.Value(i);

                                        cDim[iGL] = AddCore(cDim[iGL], MultCore(iu.Mass, dul));
                                        cIxx[iGL] = AddCore(cIxx[iGL], MultCore(iu.Ixx, dul));
                                        cIyy[iGL] = AddCore(cIyy[iGL], MultCore(iu.Iyy, dul));
                                        cIzz[iGL] = AddCore(cIzz[iGL], MultCore(iu.Izz, dul));
                                    }

                                    if (iGL > 0)
                                    {
                                        continue;
                                    }

                                    errUL!.Value(iLS) = errorU * Math.Abs((u2 - u1) * dul);

                                    for (i = 1; i <= ju; ++i)
                                    {
                                        Inertia iu = inertiaU.Value(i);

                                        cIx = AddCore(cIx, MultCore(iu.Ix, dul));
                                        cIy = AddCore(cIy, MultCore(iu.Iy, dul));
                                        cIz = AddCore(cIz, MultCore(iu.Iz, dul));

                                        cIxy = AddCore(cIxy, MultCore(iu.Ixy, dul));
                                        cIxz = AddCore(cIxz, MultCore(iu.Ixz, dul));
                                        cIyz = AddCore(cIyz, MultCore(iu.Iyz, dul));
                                    }
                                } // for: iL
                            } // for: iGL

                            ref Inertia li = ref inertiaL.ChangeValue(iLS);

                            li.Mass = MultCore(cDim[0], lr);
                            li.Ixx = MultCore(cIxx[0], lr);
                            li.Iyy = MultCore(cIyy[0], lr);
                            li.Izz = MultCore(cIzz[0], lr);

                            if (iGLEnd == 2)
                            {
                                double subDim = Math.Abs(cDim[1] - cDim[0]);

                                if (_type == BRepGProp_GaussType.Vinert)
                                {
                                    errorU = errUL!.Value(iLS);

                                    cIxx[1] = Math.Abs(cIxx[1] - cIxx[0]);
                                    cIyy[1] = Math.Abs(cIyy[1] - cIyy[0]);
                                    cIzz[1] = Math.Abs(cIzz[1] - cIzz[0]);

                                    errL.Value(iLS) = AddCore(MultCore(subDim, lr), errorU);
                                }
                                else
                                {
                                    errL.Value(iLS) = AddCore(MultCore(subDim, lr), errUL!.Value(iLS));
                                }
                            }

                            li.Ix = MultCore(cIx, lr);
                            li.Iy = MultCore(cIy, lr);
                            li.Iz = MultCore(cIz, lr);

                            li.Ixy = MultCore(cIxy, lr);
                            li.Ixz = MultCore(cIxz, lr);
                            li.Iyz = MultCore(cIyz, lr);
                        } // for: (kL)iLS
                    }

                    // Calculate/correct epsilon of computation by current value of dim
                    // That is need for not spend time for
                    if (jl == iLSubEnd)
                    {
                        kLEnd = 2;

                        double dDim = 0.0;
                        for (i = 1; i <= jl; ++i)
                        {
                            dDim += inertiaL.Value(i).Mass;
                        }

                        dDim = Math.Abs(dDim * epsilon);

                        if (dDim > Eps)
                        {
                            Eps = dDim;
                            epsL = 0.9 * Eps;
                        }
                    }

                    if (kLEnd == 2)
                    {
                        errorL = errL.Value(errL.Max());
                    }
                } while ((errorL - epsL > 0.0 && isVerifyComputation) || kLEnd == 1);

                for (i = 1; i <= jl; i++)
                {
                    AddAndRestoreInertia(inertiaL.Value(i), ref inertia);
                }

                errorLMax = Max(errorLMax, errorL);
            }

            if (isNaturalRestriction)
            {
                break;
            }

            domain.Next();
        }

        if (_type == BRepGProp_GaussType.Vinert)
        {
            global::Kerf.Engine.Measurement.MeasurementGauss.Convert(inertia, coeff, isByPoint, ref outGravityCenter, ref outInertia, ref outMass);
        }
        else
        {
            global::Kerf.Engine.Measurement.MeasurementGauss.Convert(inertia, ref outGravityCenter, ref outInertia, ref outMass);
        }

        if (iGLEnd == 2)
        {
            if (outMass != 0.0)
            {
                Eps = errorLMax / Math.Abs(outMass);
            }
            else
            {
                Eps = 0.0;
            }
        }
        else
        {
            Eps = epsilon;
        }

        return Eps;
    }

    /// <summary>The global properties of the bounded face, by adaptive Gauss integration (see the general form).</summary>
    /// <exception cref="ProgramErrorException">The integrator isn't of type Sinert.</exception>
    public double Compute(FaceIntegrand surface,
                          Domain domain,
                          in Point3D location,
                          double eps,
                          ref double outMass,
                          ref Point3D outGravityCenter,
                          ref Matrix3x3 outInertia)
    {
        Standard_ASSERT_RAISE(_type == BRepGProp_GaussType.Sinert, "BRepGProp_Gauss: Incorrect type");

        return Compute(surface,
                       domain,
                       location,
                       eps,
                       ReadOnlySpan<double>.Empty,
                       true,
                       ref outMass,
                       ref outGravityCenter,
                       ref outInertia);
    }

    /// <summary>
    /// The global properties of the bounded face, by non-adaptive Gauss integration with numbers of points chosen by
    /// the types of the surface and the curves. Returns without writing the outputs if an edge has no pcurve.
    /// </summary>
    /// <exception cref="ProgramErrorException">The integrator isn't of type Sinert.</exception>
    public void Compute(FaceIntegrand surface,
                        Domain domain,
                        in Point3D location,
                        ref double outMass,
                        ref Point3D outGravityCenter,
                        ref Matrix3x3 outInertia)
    {
        Standard_ASSERT_RAISE(_type == BRepGProp_GaussType.Sinert, "BRepGProp_Gauss: Incorrect type");

        double u1, u2, v1, v2;
        surface.Bounds(out u1, out u2, out v1, out v2);
        CheckBounds(u1, u2, v1, v2);

        int NbUGaussgp_Pnts = Min(surface.UIntegrationOrder(), NumericTables.GaussPointsMax());

        int NbVGaussgp_Pnts = Min(surface.VIntegrationOrder(), NumericTables.GaussPointsMax());

        int NbGaussgp_Pnts = Max(NbUGaussgp_Pnts, NbVGaussgp_Pnts);

        // Number of Gauss points for the integration on the face
        var gaussSPV = new math_Vector(1, NbGaussgp_Pnts);
        var gaussSWV = new math_Vector(1, NbGaussgp_Pnts);
        NumericTables.GaussPoints(NbGaussgp_Pnts, gaussSPV);
        NumericTables.GaussWeights(NbGaussgp_Pnts, gaussSWV);

        Inertia inertia = new();
        for (; domain.More(); domain.Next())
        {
            if (!surface.Load(domain.Value()))
            {
                return;
            }

            int NbCGaussgp_Pnts = Min(surface.IntegrationOrder(), NumericTables.GaussPointsMax());

            NbCGaussgp_Pnts = Max(NbCGaussgp_Pnts, NbGaussgp_Pnts);

            var gaussCP = new math_Vector(1, NbCGaussgp_Pnts);
            var gaussCW = new math_Vector(1, NbCGaussgp_Pnts);
            NumericTables.GaussPoints(NbCGaussgp_Pnts, gaussCP);
            NumericTables.GaussWeights(NbCGaussgp_Pnts, gaussCW);

            double l1 = surface.FirstParameter();
            double l2 = surface.LastParameter();
            double lm = 0.5 * (l2 + l1);
            double lr = 0.5 * (l2 - l1);

            Inertia cInertia = new();
            for (int i = 1; i <= NbCGaussgp_Pnts; ++i)
            {
                double l = lm + lr * gaussCP[i];

                surface.D12d(l, out Point2D puv, out Vector2D vuv);

                double v = puv.Y;
                u2 = puv.X;

                double dul = vuv.Y * gaussCW[i];
                double um = 0.5 * (u2 + u1);
                double ur = 0.5 * (u2 - u1);

                Inertia localInertia = new();
                for (int j = 1; j <= NbGaussgp_Pnts; ++j)
                {
                    double u = AddCore(um, MultCore(ur, gaussSPV[j]));

                    double weight = dul * gaussSWV[j];

                    surface.Normal(u, v, out Point3D point, out Vector3D normal);

                    ComputeSInertiaOfElementaryPart(point, normal, location, weight, ref localInertia);
                }

                MultAndRestoreInertia(ur, ref localInertia);
                AddAndRestoreInertia(localInertia, ref cInertia);
            }

            MultAndRestoreInertia(lr, ref cInertia);
            AddAndRestoreInertia(cInertia, ref inertia);
        }

        global::Kerf.Engine.Measurement.MeasurementGauss.Convert(inertia, ref outGravityCenter, ref outInertia, ref outMass);
    }

    /// <summary>
    /// The global properties of the region between the bounded face and a point or a plane (see the adaptive form for
    /// the coefficients), by non-adaptive Gauss integration. Returns without writing the outputs if an edge has no
    /// pcurve.
    /// </summary>
    /// <exception cref="ProgramErrorException">The integrator isn't of type Vinert.</exception>
    public void Compute(FaceIntegrand surface,
                        Domain domain,
                        in Point3D location,
                        ReadOnlySpan<double> coeff,
                        bool isByPoint,
                        ref double outMass,
                        ref Point3D outGravityCenter,
                        ref Matrix3x3 outInertia)
    {
        Standard_ASSERT_RAISE(_type == BRepGProp_GaussType.Vinert, "BRepGProp_Gauss: Incorrect type");

        double u1, v1, u2, v2;
        surface.Bounds(out u1, out u2, out v1, out v2);
        CheckBounds(u1, u2, v1, v2);

        double _u2 = u2; // OCC104

        Inertia inertia = new();
        for (; domain.More(); domain.Next())
        {
            if (!surface.Load(domain.Value()))
            {
                return;
            }

            int vNbCGaussgp_Pnts = surface.VIntegrationOrder();

            int nbGaussgp_Pnts = Min(Max(surface.IntegrationOrder(), vNbCGaussgp_Pnts), NumericTables.GaussPointsMax());

            var gaussP = new math_Vector(1, nbGaussgp_Pnts);
            var gaussW = new math_Vector(1, nbGaussgp_Pnts);
            NumericTables.GaussPoints(nbGaussgp_Pnts, gaussP);
            NumericTables.GaussWeights(nbGaussgp_Pnts, gaussW);

            double l1 = surface.FirstParameter();
            double l2 = surface.LastParameter();
            double lm = 0.5 * (l2 + l1);
            double lr = 0.5 * (l2 - l1);

            Inertia cInertia = new();
            for (int i = 1; i <= nbGaussgp_Pnts; ++i)
            {
                double l = lm + lr * gaussP[i];

                surface.D12d(l, out Point2D puv, out Vector2D vuv);

                u2 = puv.X;
                u2 = Min(Max(u1, u2), _u2); // OCC104
                double v = Min(Max(puv.Y, v1), v2);

                double dul = vuv.Y * gaussW[i];
                double um = 0.5 * (u2 + u1);
                double ur = 0.5 * (u2 - u1);

                Inertia localInertia = new();
                for (int j = 1; j <= nbGaussgp_Pnts; ++j)
                {
                    double u = um + ur * gaussP[j];
                    double weight = dul * gaussW[j];

                    surface.Normal(u, v, out Point3D point, out Vector3D normal);

                    ComputeVInertiaOfElementaryPart(point,
                                                    normal,
                                                    location,
                                                    weight,
                                                    coeff,
                                                    isByPoint,
                                                    ref localInertia);
                }

                MultAndRestoreInertia(ur, ref localInertia);
                AddAndRestoreInertia(localInertia, ref cInertia);
            }

            MultAndRestoreInertia(lr, ref cInertia);
            AddAndRestoreInertia(cInertia, ref inertia);
        }

        global::Kerf.Engine.Measurement.MeasurementGauss.Convert(inertia, coeff, isByPoint, ref outGravityCenter, ref outInertia, ref outMass);
    }

    /// <summary>
    /// The global properties of the region between the face, unbounded (its natural restriction), and a point or a
    /// plane, or of the face itself, by non-adaptive Gauss integration.
    /// </summary>
    public void Compute(FaceIntegrand surface,
                        in Point3D location,
                        ReadOnlySpan<double> coeff,
                        bool isByPoint,
                        ref double outMass,
                        ref Point3D outGravityCenter,
                        ref Matrix3x3 outInertia)
    {
        double lowerU, upperU, lowerV, upperV;
        surface.Bounds(out lowerU, out upperU, out lowerV, out upperV);
        CheckBounds(lowerU, upperU, lowerV, upperV);

        int uOrder = Min(surface.UIntegrationOrder(), NumericTables.GaussPointsMax());
        int vOrder = Min(surface.VIntegrationOrder(), NumericTables.GaussPointsMax());

        // Gauss points and weights
        var gaussPU = new math_Vector(1, uOrder);
        var gaussWU = new math_Vector(1, uOrder);
        var gaussPV = new math_Vector(1, vOrder);
        var gaussWV = new math_Vector(1, vOrder);

        NumericTables.GaussPoints(uOrder, gaussPU);
        NumericTables.GaussWeights(uOrder, gaussWU);
        NumericTables.GaussPoints(vOrder, gaussPV);
        NumericTables.GaussWeights(vOrder, gaussWV);

        double um = 0.5 * AddCore(upperU, lowerU);
        double vm = 0.5 * AddCore(upperV, lowerV);
        double ur = 0.5 * AddCore(upperU, -lowerU);
        double vr = 0.5 * AddCore(upperV, -lowerV);

        Point3D point;
        Vector3D normal;

        Inertia inertia = new();
        for (int j = 1; j <= vOrder; ++j)
        {
            Inertia inertiaOfElementaryPart = new();
            double v = AddCore(vm, MultCore(vr, gaussPV[j]));

            for (int i = 1; i <= uOrder; ++i)
            {
                double weight = gaussWU[i];
                double u = AddCore(um, MultCore(ur, gaussPU[i]));
                surface.Normal(u, v, out point, out normal);

                if (_type == BRepGProp_GaussType.Vinert)
                {
                    ComputeVInertiaOfElementaryPart(point,
                                                    normal,
                                                    location,
                                                    weight,
                                                    coeff,
                                                    isByPoint,
                                                    ref inertiaOfElementaryPart);
                }
                else // Sinert
                {
                    ComputeSInertiaOfElementaryPart(point, normal, location, weight, ref inertiaOfElementaryPart);
                }
            }

            MultAndRestoreInertia(gaussWV[j], ref inertiaOfElementaryPart);
            AddAndRestoreInertia(inertiaOfElementaryPart, ref inertia);
        }

        vr = MultCore(vr, ur);
        inertia.Ixx = MultCore(vr, inertia.Ixx);
        inertia.Iyy = MultCore(vr, inertia.Iyy);
        inertia.Izz = MultCore(vr, inertia.Izz);
        inertia.Ixy = MultCore(vr, inertia.Ixy);
        inertia.Ixz = MultCore(vr, inertia.Ixz);
        inertia.Iyz = MultCore(vr, inertia.Iyz);

        if (_type == BRepGProp_GaussType.Vinert)
        {
            global::Kerf.Engine.Measurement.MeasurementGauss.Convert(inertia, coeff, isByPoint, ref outGravityCenter, ref outInertia, ref outMass);
        }
        else // Sinert
        {
            global::Kerf.Engine.Measurement.MeasurementGauss.Convert(inertia, ref outGravityCenter, ref outInertia, ref outMass);
        }

        outMass *= vr;
    }

    /// <summary>The global properties of the face, unbounded (its natural restriction), by non-adaptive Gauss integration.</summary>
    /// <exception cref="ProgramErrorException">The integrator isn't of type Sinert.</exception>
    public void Compute(FaceIntegrand surface,
                        in Point3D location,
                        ref double outMass,
                        ref Point3D outGravityCenter,
                        ref Matrix3x3 outInertia)
    {
        Standard_ASSERT_RAISE(_type == BRepGProp_GaussType.Sinert, "BRepGProp_Gauss: Incorrect type");

        Compute(surface, location, ReadOnlySpan<double>.Empty, true, ref outMass, ref outGravityCenter, ref outInertia);
    }

    // Standard_ASSERT_RAISE: always checked in the port (rules, section 5)
    private static void Standard_ASSERT_RAISE(bool condition, string message)
    {
        if (!condition)
        {
            throw new ProgramErrorException(message);
        }
    }
}
