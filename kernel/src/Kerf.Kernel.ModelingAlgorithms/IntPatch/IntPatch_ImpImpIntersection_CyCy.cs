// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Intersection.ResultType;
using static Kerf.Engine.Intersection.Walking.WalkingSituation;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Patches;

// The cylinder/cylinder intersection (OCCT 7.x's IntPatch_ImpImpIntersection_4.gxx): the analytical cases and the
// walking along U1 of the non-analytical one.
internal sealed partial class ImpImpIntersection
{
    // If std::abs(a) <= aNulValue then it is considered that a = 0.
    private const double ANulValue = 1.0e-11;

    // Every cylinder can be represented by the following equation in parametric form:
    //     S(U,V) = L + R*cos(U)*Xd+R*sin(U)*Yd+V*Zd,
    // where location L, directions Xd, Yd and Zd have type gp_XYZ.
    //
    // Intersection points between two cylinders can be found from the following system:
    //     S1(U1, V1) = S2(U2, V2)
    // or
    //     {X01 + R1*cos(U1)*Xx1 + R1*sin(U1)*Yx1 + V1*Zx1 = X02 + R2*cos(U2)*Xx2 + R2*sin(U2)*Yx2 + V2*Zx2
    //     {Y01 + R1*cos(U1)*Xy1 + R1*sin(U1)*Yy1 + V1*Zy1 = Y02 + R2*cos(U2)*Xy2 + R2*sin(U2)*Yy2 + V2*Zy2   (1)
    //     {Z01 + R1*cos(U1)*Xz1 + R1*sin(U1)*Yz1 + V1*Zz1 = Z02 + R2*cos(U2)*Xz2 + R2*sin(U2)*Yz2 + V2*Zz2
    //
    // The formula (1) can be rewritten as follows
    //     {C11*V1+C21*V2=A11*cos(U1)+B11*sin(U1)+A21*cos(U2)+B21*sin(U2)+D1
    //     {C12*V1+C22*V2=A12*cos(U1)+B12*sin(U1)+A22*cos(U2)+B22*sin(U2)+D2 (2)
    //     {C13*V1+C23*V2=A13*cos(U1)+B13*sin(U1)+A23*cos(U2)+B23*sin(U2)+D3
    //
    // Hereafter we consider that in system
    //     {C11*V1+C21*V2=A11*cos(U1)+B11*sin(U1)+A21*cos(U2)+B21*sin(U2)+D1 (3)
    //     {C12*V1+C22*V2=A12*cos(U1)+B12*sin(U1)+A22*cos(U2)+B22*sin(U2)+D2
    // variables V1 and V2 can be found unambiguously, i.e. determinant
    //             |C11 C21|
    //             |       | != 0
    //             |C12 C22|
    //
    // In this case, variables V1 and V2 can be found as follows:
    //     {V1 = K11*sin(U1)+K21*sin(U2)+L11*cos(U1)+L21*cos(U2)+M1 = K1*cos(U1-FIV1)+L1*cos(U2-PSIV1)+M1      (4)
    //     {V2 = K12*sin(U1)+K22*sin(U2)+L12*cos(U1)+L22*cos(U2)+M2 = K2*cos(U2-FIV2)+L2*cos(U2-PSIV2)+M2
    //
    // Having substituted result of (4) to the 3rd equation of (2), we will obtain equation
    //     cos(U2-FI2) = B*cos(U1-FI1)+C. (5)
    //
    // I.e. when U1 is taken different given values (from domain), correspond U2 value can be computed
    // from equation (5). After that, V1 and V2 can be computed from the system (4) (see
    // CylCylComputeParameters(...) methods).
    //
    // It is important to remark that equation (5) (in general) has two solutions: U2=FI2 +/- f(U1).
    // Therefore, we are getting here two intersection lines.
    private static class ComputationMethods
    {
        // Stores equations coefficients
        internal sealed class StCoeffsValue
        {
            internal readonly math_Vector _mVecA1;
            internal readonly math_Vector _mVecA2;
            internal readonly math_Vector _mVecB1;
            internal readonly math_Vector _mVecB2;
            internal readonly math_Vector _mVecC1;
            internal readonly math_Vector _mVecC2;
            internal readonly math_Vector _mVecD;

            internal double _mK21; // sinU2
            internal double _mK11; // sinU1
            internal double _mL21; // cosU2
            internal double _mL11; // cosU1
            internal double _mM1;  // Free member

            internal double _mK22; // sinU2
            internal double _mK12; // sinU1
            internal double _mL22; // cosU2
            internal double _mL12; // cosU1
            internal double _mM2;  // Free member

            internal double _mK1;
            internal double _mL1;
            internal double _mK2;
            internal double _mL2;

            internal double _mFIV1;
            internal double _mPSIV1;
            internal double _mFIV2;
            internal double _mPSIV2;

            internal double _mB;
            internal double _mC;
            internal double _mFI1;
            internal double _mFI2;

            private enum CoupleOfEquation
            {
                COENONE = 0,
                COE12 = 1,
                COE23 = 2,
                COE13 = 3,
            }

            internal StCoeffsValue(in Cylinder3D cyl1, in Cylinder3D cyl2)
            {
                _mVecA1 = new math_Vector(-cyl1.Radius * cyl1.XAxis.Direction.XYZ());
                _mVecA2 = new math_Vector(cyl2.Radius * cyl2.XAxis.Direction.XYZ());
                _mVecB1 = new math_Vector(-cyl1.Radius * cyl1.YAxis.Direction.XYZ());
                _mVecB2 = new math_Vector(cyl2.Radius * cyl2.YAxis.Direction.XYZ());
                _mVecC1 = new math_Vector(cyl1.Axis.Direction.XYZ());
                _mVecC2 = new math_Vector(cyl2.Axis.Direction.XYZ().Reversed());
                _mVecD = new math_Vector(cyl2.Location.XYZ() - cyl1.Location.XYZ());

                CoupleOfEquation foundCouple = CoupleOfEquation.COENONE;

                double detV1V2 = 0.0;

                double delta1 = _mVecC1[1] * _mVecC2[2] - _mVecC1[2] * _mVecC2[1]; // 1-2
                double delta2 = _mVecC1[2] * _mVecC2[3] - _mVecC1[3] * _mVecC2[2]; // 2-3
                double delta3 = _mVecC1[1] * _mVecC2[3] - _mVecC1[3] * _mVecC2[1]; // 1-3
                double absD1 = Math.Abs(delta1);                             // 1-2
                double absD2 = Math.Abs(delta2);                             // 2-3
                double absD3 = Math.Abs(delta3);                             // 1-3

                if (absD1 >= absD2)
                {
                    if (absD3 > absD1)
                    {
                        foundCouple = CoupleOfEquation.COE13;
                        detV1V2 = delta3;
                    }
                    else
                    {
                        foundCouple = CoupleOfEquation.COE12;
                        detV1V2 = delta1;
                    }
                }
                else
                {
                    if (absD3 > absD2)
                    {
                        foundCouple = CoupleOfEquation.COE13;
                        detV1V2 = delta3;
                    }
                    else
                    {
                        foundCouple = CoupleOfEquation.COE23;
                        detV1V2 = delta2;
                    }
                }

                // In point of fact, every determinant (aDelta1, aDelta2 and aDelta3) is
                // cross-product between directions (i.e. sine of angle).
                // If sine is too small then sine is (approx.) equal to angle itself.
                // Therefore, in this case we should compare sine with angular tolerance.
                // This constant is used for check if axes are parallel (see constructor
                // AxeOperator::AxeOperator(...) in IntAna_QuadQuadGeo.cxx file).
                if (Math.Abs(detV1V2) < global::Kerf.Engine.Foundation.Precision.Angular())
                {
                    throw new FailureException("Error. Exception in divide by zerro (IntCyCyTrim)!!!!");
                }

                switch (foundCouple)
                {
                    case CoupleOfEquation.COE12:
                        break;
                    case CoupleOfEquation.COE23:
                    {
                        RotateCoordinates(_mVecA1);
                        RotateCoordinates(_mVecA2);
                        RotateCoordinates(_mVecB1);
                        RotateCoordinates(_mVecB2);
                        RotateCoordinates(_mVecC1);
                        RotateCoordinates(_mVecC2);
                        RotateCoordinates(_mVecD);
                        break;
                    }
                    case CoupleOfEquation.COE13:
                    {
                        SwapCoordinates23(_mVecA1);
                        SwapCoordinates23(_mVecA2);
                        SwapCoordinates23(_mVecB1);
                        SwapCoordinates23(_mVecB2);
                        SwapCoordinates23(_mVecC1);
                        SwapCoordinates23(_mVecC2);
                        SwapCoordinates23(_mVecD);
                        break;
                    }
                    default:
                        break;
                }

                //------- For V1 (begin)
                // sinU2
                _mK21 = (_mVecC2[2] * _mVecB2[1] - _mVecC2[1] * _mVecB2[2]) / detV1V2;
                // sinU1
                _mK11 = (_mVecC2[2] * _mVecB1[1] - _mVecC2[1] * _mVecB1[2]) / detV1V2;
                // cosU2
                _mL21 = (_mVecC2[2] * _mVecA2[1] - _mVecC2[1] * _mVecA2[2]) / detV1V2;
                // cosU1
                _mL11 = (_mVecC2[2] * _mVecA1[1] - _mVecC2[1] * _mVecA1[2]) / detV1V2;
                // Free member
                _mM1 = (_mVecC2[2] * _mVecD[1] - _mVecC2[1] * _mVecD[2]) / detV1V2;
                //------- For V1 (end)

                //------- For V2 (begin)
                // sinU2
                _mK22 = (_mVecC1[1] * _mVecB2[2] - _mVecC1[2] * _mVecB2[1]) / detV1V2;
                // sinU1
                _mK12 = (_mVecC1[1] * _mVecB1[2] - _mVecC1[2] * _mVecB1[1]) / detV1V2;
                // cosU2
                _mL22 = (_mVecC1[1] * _mVecA2[2] - _mVecC1[2] * _mVecA2[1]) / detV1V2;
                // cosU1
                _mL12 = (_mVecC1[1] * _mVecA1[2] - _mVecC1[2] * _mVecA1[1]) / detV1V2;
                // Free member
                _mM2 = (_mVecC1[1] * _mVecD[2] - _mVecC1[2] * _mVecD[1]) / detV1V2;
                //------- For V1 (end)

                ShortCosForm(_mL11, _mK11, ref _mK1, ref _mFIV1);
                ShortCosForm(_mL21, _mK21, ref _mL1, ref _mPSIV1);
                ShortCosForm(_mL12, _mK12, ref _mK2, ref _mFIV2);
                ShortCosForm(_mL22, _mK22, ref _mL2, ref _mPSIV2);

                double a1 = _mVecC1[3] * _mK21 + _mVecC2[3] * _mK22 - _mVecB2[3], // sinU2
                    a2 = _mVecC1[3] * _mL21 + _mVecC2[3] * _mL22 - _mVecA2[3],    // cosU2
                    b1 = _mVecB1[3] - _mVecC1[3] * _mK11 - _mVecC2[3] * _mK12,    // sinU1
                    b2 = _mVecA1[3] - _mVecC1[3] * _mL11 - _mVecC2[3] * _mL12;    // cosU1

                _mC = _mVecD[3] - _mVecC1[3] * _mM1 - _mVecC2[3] * _mM2; // Free

                double a = 0.0;

                ShortCosForm(b2, b1, ref _mB, ref _mFI1);
                ShortCosForm(a2, a1, ref a, ref _mFI2);

                _mB /= a;
                _mC /= a;
            }

            // OCCT's COE23 case: (x, y, z) := (y, z, x), through a copy of the vector.
            private static void RotateCoordinates(math_Vector v)
            {
                math_Vector vTemp = new(v);
                v[1] = vTemp[2];
                v[2] = vTemp[3];
                v[3] = vTemp[1];
            }

            // OCCT's COE13 case: (x, y, z) := (x, z, y), through a copy of the vector.
            private static void SwapCoordinates23(math_Vector v)
            {
                math_Vector vTemp = new(v);
                v[2] = vTemp[3];
                v[3] = vTemp[2];
            }
        }

        //=======================================================================
        // function : CylCylMonotonicity
        // purpose  : Determines, if U2(U1) function is increasing.
        //=======================================================================
        internal static bool CylCylMonotonicity(double u1par,
                                                int wlIndex,
                                                StCoeffsValue coeffs,
                                                double period,
                                                ref bool isIncreasing)
        {
            // U2(U1) = FI2 + (+/-)acos(B*cos(U1 - FI1) + C);
            // Accordingly,
            // Func. U2(X1) = FI2 + X1;
            // Func. X1(X2) = anArccosFactor*X2;
            // Func. X2(X3) = acos(X3);
            // Func. X3(X4) = B*X4 + C;
            // Func. X4(U1) = cos(U1 - FI1).
            //
            // Consequently,
            // U2(X1) is always increasing.
            // X1(X2) is increasing if anArccosFactor > 0.0 and
            // is decreasing otherwise.
            // X2(X3) is always decreasing.
            // Therefore, U2(X3) is decreasing if anArccosFactor > 0.0 and
            // is increasing otherwise.
            // X3(X4) is increasing if B > 0 and is decreasing otherwise.
            // X4(U1) is increasing if U1 - FI1 in [PI, 2*PI) and
            // is decreasing U1 - FI1 in [0, PI) or if (U1 - FI1 == 2PI).
            // After that, we can predict behaviour of U2(U1) function:
            // if it is increasing or decreasing.

            // For "+/-" sign. If isPlus == TRUE, "+" is chosen, otherwise, we choose "-".
            bool isPlus = false;

            switch (wlIndex)
            {
                case 0:
                    isPlus = true;
                    break;
                case 1:
                    isPlus = false;
                    break;
                default:
                    // throw Standard_Failure("Error. Range Error!!!!");
                    return false;
            }

            double u1Temp = u1par - coeffs._mFI1;
            InscribePoint(0, period, ref u1Temp, 0.0, period, false);

            isIncreasing = true;

            if (((M_PI - u1Temp) < RealSmall()) && (u1Temp < period))
            {
                isIncreasing = false;
            }

            if (coeffs._mB < 0.0)
            {
                isIncreasing = !isIncreasing;
            }

            if (!isPlus)
            {
                isIncreasing = !isIncreasing;
            }

            return true;
        }

        //=======================================================================
        // function : CylCylComputeParameters
        // purpose  : Computes U2 (U-parameter of the 2nd cylinder) and, if theDelta != 0,
        //            estimates the tolerance of U2-computing (estimation result is
        //            assigned to *theDelta value).
        //=======================================================================
        // OCCT's optional pointer theDelta (nullptr by default): this overload is the null pointer, the next one
        // the non-null pointer.
        internal static bool CylCylComputeParameters(double u1par,
                                                     int wlIndex,
                                                     StCoeffsValue coeffs,
                                                     ref double u2)
        {
            double dummy = 0.0;
            return CylCylComputeParameters(u1par, wlIndex, coeffs, ref u2, ref dummy, false);
        }

        internal static bool CylCylComputeParameters(double u1par,
                                                     int wlIndex,
                                                     StCoeffsValue coeffs,
                                                     ref double u2,
                                                     ref double delta)
        {
            return CylCylComputeParameters(u1par, wlIndex, coeffs, ref u2, ref delta, true);
        }

        private static bool CylCylComputeParameters(double u1par,
                                                    int wlIndex,
                                                    StCoeffsValue coeffs,
                                                    ref double u2,
                                                    ref double delta,
                                                    bool hasDelta)
        {
            // This formula is got from some experience and can be changed.
            double tol0 = Min(10.0 * Epsilon(1.0) * coeffs._mB, ANulValue);
            double tol = 1.0 - tol0;

            if (wlIndex < 0 || wlIndex > 1)
            {
                return false;
            }

            double sign = wlIndex != 0 ? -1.0 : 1.0;

            double arg = Math.Cos(u1par - coeffs._mFI1);
            arg = coeffs._mB * arg + coeffs._mC;

            if (arg >= tol)
            {
                if (hasDelta)
                {
                    delta = 0.0;
                }

                arg = 1.0;
            }
            else if (arg <= -tol)
            {
                if (hasDelta)
                {
                    delta = 0.0;
                }

                arg = -1.0;
            }
            else if (hasDelta)
            {
                // There is a case, when
                //   const double aPar = 0.99999999999721167;
                //   const double aFI2 = 2.3593296083566181e-006;

                // Then
                //   aPar - cos(aFI2) == -5.10703e-015 ==> cos(aFI2) == aPar.
                // Theoretically, in this case
                //   aFI2 == +/- acos(aPar).
                // However,
                //   acos(aPar) - aFI2 == 2.16362e-009.
                // Error is quite big.

                // This error should be estimated. Let use following way, which is based
                // on expanding to Taylor series.

                //  acos(p)-acos(p+x) = x/sqrt(1-p*p).

                // If p == (1-d) (when p > 0) or p == (-1+d) (when p < 0) then
                //   acos(p)-acos(p+x) = x/sqrt(d*(2-d)).

                // Here always aTol0 <= d <= 1.0 std::max(x) is considered (!) to be equal to aTol0.
                // In this case
                //   8*aTol0 <= acos(p)-acos(p+x) <= sqrt(2/(2-aTol0)-1),
                //                                               because 0 < aTol0 < 1.
                // E.g. when aTol0 = 1.0e-11,
                //    8.0e-11 <= acos(p)-acos(p+x) < 2.24e-6.

                double aDelta = Min(1.0 - arg, 1.0 + arg);
                Standard_DivideByZero_Raise_if((aDelta * aDelta < RealSmall()) || (aDelta >= 2.0),
                                               "IntPatch_ImpImpIntersection_4.gxx, CylCylComputeParameters()");
                delta = tol0 / Math.Sqrt(aDelta * (2.0 - aDelta));
            }

            u2 = Math.Acos(arg);
            u2 = coeffs._mFI2 + sign * u2;

            return true;
        }

        //=======================================================================
        // function : CylCylComputeParameters
        // purpose  : Computes V1 and V2 (V-parameters of the 1st and 2nd cylinder respectively).
        //=======================================================================
        internal static bool CylCylComputeParameters(double u1,
                                                     double u2,
                                                     StCoeffsValue coeffs,
                                                     ref double v1,
                                                     ref double v2)
        {
            v1 = coeffs._mK21 * Math.Sin(u2) + coeffs._mK11 * Math.Sin(u1) + coeffs._mL21 * Math.Cos(u2)
                    + coeffs._mL11 * Math.Cos(u1) + coeffs._mM1;

            v2 = coeffs._mK22 * Math.Sin(u2) + coeffs._mK12 * Math.Sin(u1) + coeffs._mL22 * Math.Cos(u2)
                    + coeffs._mL12 * Math.Cos(u1) + coeffs._mM2;

            return true;
        }

        //=======================================================================
        // function : CylCylComputeParameters
        // purpose  : Computes U2 (U-parameter of the 2nd cylinder),
        //            V1 and V2 (V-parameters of the 1st and 2nd cylinder respectively).
        //=======================================================================
        internal static bool CylCylComputeParameters(double u1par,
                                                     int wlIndex,
                                                     StCoeffsValue coeffs,
                                                     ref double u2,
                                                     ref double v1,
                                                     ref double v2)
        {
            if (!CylCylComputeParameters(u1par, wlIndex, coeffs, ref u2))
            {
                return false;
            }

            if (!CylCylComputeParameters(u1par, u2, coeffs, ref v1, ref v2))
            {
                return false;
            }

            return true;
        }
    }

    // Cyl-cyl-specific helper: constructed only by IntCyCy
    // and relies on cylinder-cylinder coefficient tables (ComputationMethods::
    // StCoeffsValue) for both SearchOnVBounds' Newton system and the analytical
    // CylCylComputeParameters evaluators used downstream.
    // OCCT keeps const references to the quadrics, coefficients and boxes; the port keeps copies of the structs
    // (never changed while the object lives) and the coefficients by reference.
    private sealed class WorkWithBoundaries
    {
        internal enum SearchBoundType
        {
            SearchNONE = 0,
            SearchV1 = 1,
            SearchV2 = 2,
        }

        internal struct StPInfo
        {
            public StPInfo()
            {
                _surfID = 0;
                _u1 = RealLast();
                _v1 = RealLast();
                _u2 = RealLast();
                _v2 = RealLast();
            }

            // Equal to 0 for 1st surface non-zero for 2nd one.
            internal int _surfID;

            internal double _u1;
            internal double _v1;
            internal double _u2;
            internal double _v2;
        }

        private readonly SurfaceQuadric _quad1;
        private readonly SurfaceQuadric _quad2;
        private readonly ComputationMethods.StCoeffsValue _coeffs;
        private readonly Box2d _uvSurf1;
        private readonly Box2d _uvSurf2;
        private readonly int _nbWLines;
        private readonly double _period;
        private readonly double _tol3D;
        private readonly double _tol2D;
        private readonly bool _isReverse;

        internal WorkWithBoundaries(in SurfaceQuadric quad1,
                                    in SurfaceQuadric quad2,
                                    ComputationMethods.StCoeffsValue coeffs,
                                    in Box2d uvSurf1,
                                    in Box2d uvSurf2,
                                    int nbWLines,
                                    double period,
                                    double tol3D,
                                    double tol2D,
                                    bool isTheReverse)
        {
            _quad1 = quad1;
            _quad2 = quad2;
            _coeffs = coeffs;
            _uvSurf1 = uvSurf1;
            _uvSurf2 = uvSurf2;
            _nbWLines = nbWLines;
            _period = period;
            _tol3D = tol3D;
            _tol2D = tol2D;
            _isReverse = isTheReverse;
            _ = _nbWLines; // OCCT stores it and never reads it
        }

        // Returns parameters of system solved while finding
        // intersection line
        internal ComputationMethods.StCoeffsValue SICoeffs() => _coeffs;

        // Returns quadric correspond to the index theIdx.
        internal SurfaceQuadric GetQSurface(int idx)
        {
            if (idx <= 1)
            {
                return _quad1;
            }

            return _quad2;
        }

        // Returns TRUE in case of reverting surfaces
        internal bool IsReversed => _isReverse;

        // Returns 2D-tolerance
        internal double Get2dTolerance() => _tol2D;

        // Returns 3D-tolerance
        internal double Get3dTolerance() => _tol3D;

        // Returns UV-bounds of 1st surface
        internal Box2d UVS1() => _uvSurf1;

        // Returns UV-bounds of 2nd surface
        internal Box2d UVS2() => _uvSurf2;

        // V(U1) - V_bound on one arccos branch of the analytical intersection curve (OCCT's local struct
        // VBoundDelta in AddBoundaryPoint).
        private readonly struct VBoundDelta : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
        {
            private readonly ComputationMethods.StCoeffsValue _coeffs;
            private readonly int _wlIndex;
            private readonly bool _isV1;
            private readonly double _vBound;

            internal VBoundDelta(ComputationMethods.StCoeffsValue coeffs, int wlIndex, bool isV1, double vBound)
            {
                _coeffs = coeffs;
                _wlIndex = wlIndex;
                _isV1 = isV1;
                _vBound = vBound;
            }

            public bool Value(double x, out double f)
            {
                f = 0.0; // OCCT leaves theF unwritten on failure
                double u2 = 0.0, v1 = 0.0, v2 = 0.0;
                if (!ComputationMethods.CylCylComputeParameters(x, _wlIndex, _coeffs, ref u2, ref v1, ref v2))
                {
                    return false;
                }
                f = (_isV1 ? v1 : v2) - _vBound;
                return true;
            }
        }

        // OCCT's lambda findVBoundCrossing in AddBoundaryPoint.
        private bool FindVBoundCrossing(int wlIndex,
                                        bool isV1,
                                        double vBound,
                                        double uLo,
                                        double uHi,
                                        ref double u1Star)
        {
            if (!(uLo < uHi))
            {
                return false;
            }
            VBoundDelta func = new(_coeffs, wlIndex, isV1, vBound);
            if (!func.Value(uLo, out double flo) || !func.Value(uHi, out double fhi))
            {
                return false;
            }
            if (flo == 0.0)
            {
                u1Star = uLo;
                return true;
            }
            if (fhi == 0.0)
            {
                u1Star = uHi;
                return true;
            }
            if (flo * fhi > 0.0)
            {
                return false; // no bracketed crossing
            }

            global::Kerf.Engine.Numerics.Utilities.MathUtils.Config cfg = new();
            cfg.XTolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
            global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult res = global::Kerf.Engine.Numerics.Roots.MathRoot.Brent(func, uLo, uHi, cfg);
            if (res.Status != global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK || !res.Root.HasValue)
            {
                return false;
            }
            u1Star = res.Root.Value;
            return true;
        }

        //=======================================================================
        // function : AddBoundaryPoint
        // purpose  : Find intersection point on V-boundary.
        //=======================================================================
        // theU1Prev is the walker's previous U1 sample. Together with theU1 it
        // forms the bracket across which the straddle on V_bound was detected and
        // in which the V-boundary crossing is resolved by 1D branch-aware
        // bisection on the analytical intersection curve.
        internal void AddBoundaryPoint(WLine wl,
                                       double u1,
                                       double u1Prev,
                                       double u1Min,
                                       double u2,
                                       double v1,
                                       double v1Prev,
                                       double v2,
                                       double v2Prev,
                                       int wlIndex,
                                       bool flForce,
                                       ref bool isTheFound1,
                                       ref bool isTheFound2)
        {
            _uvSurf1.Get(out double uSurf1f, out double vSurf1f, out double uSurf1l, out double vSurf1l);
            _uvSurf2.Get(out double uSurf2f, out double vSurf2f, out double uSurf2l, out double vSurf2l);

            const int size = 4;
            ReadOnlySpan<double> arrVzad = [vSurf1f, vSurf1l, vSurf2f, vSurf2l];

            Span<StPInfo> uvPoint = [new(), new(), new(), new()];

            // Branch-aware 1D root-finder for V(U1) = V_bound on the analytical
            // intersection curve, used in place of the original rank-deficient 3x3
            // Newton (SearchOnVBounds). The intersection curve is 1D in U1 on each
            // arccos branch, so the correct numerical tool is a bracketed 1D root
            // finder. We use MathRoot::Brent, which converges robustly even when the
            // curve grazes V_bound at a near-tangent (two close crossings on opposite
            // sides of a V-extremum) -- the configuration that makes the 3x3 Newton
            // Jacobian rank-deficient. The only tolerance is Precision::PConfusion()
            // (OCCT standard parametric equality) for termination on U.

            for (int idSurf = 0; idSurf < 4; idSurf += 2)
            {
                double vf = (idSurf == 0) ? v1 : v2,
                    vl = (idSurf == 0) ? v1Prev : v2Prev;

                SearchBoundType ts = (idSurf == 0) ? SearchBoundType.SearchV1 : SearchBoundType.SearchV2;

                for (int idBound = 0; idBound < 2; idBound++)
                {
                    int index = idSurf + idBound;

                    uvPoint[index]._surfID = idSurf;

                    if ((Math.Abs(vf - arrVzad[index]) > _tol2D)
                        && ((vf - arrVzad[index]) * (vl - arrVzad[index]) > 0.0))
                    {
                        continue;
                    }

                    // Segment [aVf, aVl] intersects at least one V-boundary (first or last).
                    // Resolve the crossing by 1D bisection on the analytical curve.
                    double uLo = Min(u1Prev, u1);
                    double uHi = Max(u1Prev, u1);
                    bool res = FindVBoundCrossing(wlIndex,
                                                   ts == SearchBoundType.SearchV1,
                                                   arrVzad[index],
                                                   uLo,
                                                   uHi,
                                                   ref uvPoint[index]._u1);

                    // aUVPoint[anIndex].myU1 is considered to be nearer to theU1 than
                    // to theU1+/-Period
                    if (!res || (uvPoint[index]._u1 >= u1) || (uvPoint[index]._u1 < u1Min))
                    {
                        // Intersection point is not found or out of the domain
                        uvPoint[index]._u1 = RealLast();
                        continue;
                    }
                    else
                    {
                        // intersection point is found

                        ref StPInfo p = ref uvPoint[index];
                        p._u2 = u2;
                        p._v1 = v1;
                        p._v2 = v2;

                        if (!ComputationMethods.CylCylComputeParameters(p._u1,
                                                                        wlIndex,
                                                                        _coeffs,
                                                                        ref p._u2,
                                                                        ref p._v1,
                                                                        ref p._v2))
                        {
                            // Found point is wrong
                            p._u1 = RealLast();
                            continue;
                        }

                        // Point on true V-boundary.
                        if (ts == SearchBoundType.SearchV1)
                        {
                            p._v1 = arrVzad[index];
                        }
                        else
                        { // if(aTS[anIndex] == SearchV2)
                            p._v2 = arrVzad[index];
                        }
                    }
                }
            }

            // Sort with acceding U1-parameter.
            // std::sort(aUVPoint, aUVPoint + aSize): MSVC's insertion sort on 4 elements (ties keep their order).
            MsvcInsertionSort(uvPoint);

            isTheFound1 = isTheFound2 = false;

            // Adding found points on boundary in the WLine.
            for (int i = 0; i < size; i++)
            {
                if (uvPoint[i]._u1 == RealLast())
                {
                    break;
                }

                if (!AddPointIntoWL(_quad1,
                                    _quad2,
                                    _coeffs,
                                    _isReverse,
                                    false,
                                    new Point2D(uvPoint[i]._u1, uvPoint[i]._v1),
                                    new Point2D(uvPoint[i]._u2, uvPoint[i]._v2),
                                    uSurf1f,
                                    uSurf1l,
                                    uSurf2f,
                                    uSurf2l,
                                    vSurf1f,
                                    vSurf1l,
                                    vSurf2f,
                                    vSurf2l,
                                    _period,
                                    wl.Curve(),
                                    wlIndex,
                                    _tol3D,
                                    _tol2D,
                                    flForce))
                {
                    continue;
                }

                if (uvPoint[i]._surfID == 0)
                {
                    isTheFound1 = true;
                }
                else
                {
                    isTheFound2 = true;
                }
            }
        }

        // std::sort of StPInfo by myU1 (operator<) as MSVC does it on up to 32 elements: _Insertion_sort_unchecked,
        // which keeps tied elements in their order (decision 610). OCCT sorts 4 elements here.
        private static void MsvcInsertionSort(Span<StPInfo> arr)
        {
            for (int mid = 1; mid < arr.Length; mid++)
            {
                int hole = mid;
                StPInfo val = arr[mid];
                if (val._u1 < arr[0]._u1)
                {
                    // found new earliest element, move to front
                    for (int k = mid; k > 0; k--)
                    {
                        arr[k] = arr[k - 1];
                    }
                    arr[0] = val;
                }
                else
                {
                    // look for insertion point after first
                    for (int prev = hole - 1; val._u1 < arr[prev]._u1; hole = prev, prev--)
                    {
                        arr[hole] = arr[prev]; // move hole down
                    }
                    arr[hole] = val; // insert element in hole
                }
            }
        }

        //=======================================================================
        // function : BoundariesComputing
        // purpose  : Computes true domain of future intersection curve (allows
        //            avoiding excess iterations)
        //=======================================================================
        internal static bool BoundariesComputing(ComputationMethods.StCoeffsValue coeffs,
                                                 double period,
                                                 BoundRange[] uRange)
        {
            // All comments to this method is related to the comment
            // to ComputationMethods class

            // So, we have the equation
            //     cos(U2-FI2)=B*cos(U1-FI1)+C
            // Evidently,
            //     -1 <= B*cos(U1-FI1)+C <= 1

            if (coeffs._mB > 0.0)
            {
                // -(1+C)/B <= cos(U1-FI1) <= (1-C)/B

                if (coeffs._mB + Math.Abs(coeffs._mC) < -1.0) // OCCT: never true for mB > 0 (a dead branch)
                {
                    //(1-C)/B < -1 or -(1+C)/B > 1  ==> No solution

                    return false;
                }
                else if (coeffs._mB + Math.Abs(coeffs._mC) <= 1.0)
                {
                    //(1-C)/B >= 1 and -(1+C)/B <= -1 ==> U=[0;2*PI]+aFI1
                    uRange[0].Add(coeffs._mFI1);
                    uRange[0].Add(period + coeffs._mFI1);
                }
                else if ((1 + coeffs._mC <= coeffs._mB) && (coeffs._mB <= 1 - coeffs._mC))
                {
                    //(1-C)/B >= 1 and -(1+C)/B >= -1 ==>
                    //(U=[0;aDAngle]+aFI1) || (U=[2*PI-aDAngle;2*PI]+aFI1),
                    // where aDAngle = acos(-(myCoeffs.mC + 1) / myCoeffs.mB)

                    double arg = -(coeffs._mC + 1) / coeffs._mB;
                    if (arg > 1.0)
                    {
                        arg = 1.0;
                    }
                    if (arg < -1.0)
                    {
                        arg = -1.0;
                    }

                    double dAngle = Math.Acos(arg);
                    uRange[0].Add(coeffs._mFI1);
                    uRange[0].Add(dAngle + coeffs._mFI1);
                    uRange[1].Add(period - dAngle + coeffs._mFI1);
                    uRange[1].Add(period + coeffs._mFI1);
                }
                else if ((1 - coeffs._mC <= coeffs._mB) && (coeffs._mB <= 1 + coeffs._mC))
                {
                    //(1-C)/B <= 1 and -(1+C)/B <= -1 ==> U=[aDAngle;2*PI-aDAngle]+aFI1
                    // where aDAngle = acos((1 - myCoeffs.mC) / myCoeffs.mB)

                    double arg = (1 - coeffs._mC) / coeffs._mB;
                    if (arg > 1.0)
                    {
                        arg = 1.0;
                    }
                    if (arg < -1.0)
                    {
                        arg = -1.0;
                    }

                    double dAngle = Math.Acos(arg);
                    uRange[0].Add(dAngle + coeffs._mFI1);
                    uRange[0].Add(period - dAngle + coeffs._mFI1);
                }
                else if (coeffs._mB - Math.Abs(coeffs._mC) >= 1.0)
                {
                    //(1-C)/B <= 1 and -(1+C)/B >= -1 ==>
                    //(U=[aDAngle1;aDAngle2]+aFI1) ||
                    //(U=[2*PI-aDAngle2;2*PI-aDAngle1]+aFI1)
                    // where aDAngle1 = acos((1 - myCoeffs.mC) / myCoeffs.mB),
                    //      aDAngle2 = acos(-(myCoeffs.mC + 1) / myCoeffs.mB).

                    double arg1 = (1 - coeffs._mC) / coeffs._mB,
                        arg2 = -(coeffs._mC + 1) / coeffs._mB;
                    if (arg1 > 1.0)
                    {
                        arg1 = 1.0;
                    }
                    if (arg1 < -1.0)
                    {
                        arg1 = -1.0;
                    }

                    if (arg2 > 1.0)
                    {
                        arg2 = 1.0;
                    }
                    if (arg2 < -1.0)
                    {
                        arg2 = -1.0;
                    }

                    double dAngle1 = Math.Acos(arg1), dAngle2 = Math.Acos(arg2);
                    //(U=[aDAngle1;aDAngle2]+aFI1) ||
                    //(U=[2*PI-aDAngle2;2*PI-aDAngle1]+aFI1)
                    uRange[0].Add(dAngle1 + coeffs._mFI1);
                    uRange[0].Add(dAngle2 + coeffs._mFI1);
                    uRange[1].Add(period - dAngle2 + coeffs._mFI1);
                    uRange[1].Add(period - dAngle1 + coeffs._mFI1);
                }
                else
                {
                    return false;
                }
            }
            else if (coeffs._mB < 0.0)
            {
                // (1-C)/B <= cos(U1-FI1) <= -(1+C)/B

                if (coeffs._mB + Math.Abs(coeffs._mC) > 1.0)
                {
                    // -(1+C)/B < -1 or (1-C)/B > 1 ==> No solutions
                    return false;
                }
                else if (-coeffs._mB + Math.Abs(coeffs._mC) <= 1.0)
                {
                    //  -(1+C)/B >= 1 and (1-C)/B <= -1 ==> U=[0;2*PI]+aFI1
                    uRange[0].Add(coeffs._mFI1);
                    uRange[0].Add(period + coeffs._mFI1);
                }
                else if ((-coeffs._mC - 1 <= coeffs._mB) && (coeffs._mB <= coeffs._mC - 1))
                {
                    //  -(1+C)/B >= 1 and (1-C)/B >= -1 ==>
                    //(U=[0;aDAngle]+aFI1) || (U=[2*PI-aDAngle;2*PI]+aFI1),
                    // where aDAngle = acos((1 - myCoeffs.mC) / myCoeffs.mB)

                    double arg = (1 - coeffs._mC) / coeffs._mB;
                    if (arg > 1.0)
                    {
                        arg = 1.0;
                    }
                    if (arg < -1.0)
                    {
                        arg = -1.0;
                    }

                    double dAngle = Math.Acos(arg);
                    uRange[0].Add(coeffs._mFI1);
                    uRange[0].Add(dAngle + coeffs._mFI1);
                    uRange[1].Add(period - dAngle + coeffs._mFI1);
                    uRange[1].Add(period + coeffs._mFI1);
                }
                // OCCT: -mB - 1 where the comment's bound is -mC - 1
                else if ((coeffs._mC - 1 <= coeffs._mB) && (coeffs._mB <= -coeffs._mB - 1))
                {
                    //  -(1+C)/B <= 1 and (1-C)/B <= -1 ==> U=[aDAngle;2*PI-aDAngle]+aFI1,
                    // where aDAngle = acos(-(myCoeffs.mC + 1) / myCoeffs.mB).

                    double arg = -(coeffs._mC + 1) / coeffs._mB;
                    if (arg > 1.0)
                    {
                        arg = 1.0;
                    }
                    if (arg < -1.0)
                    {
                        arg = -1.0;
                    }

                    double dAngle = Math.Acos(arg);
                    uRange[0].Add(dAngle + coeffs._mFI1);
                    uRange[0].Add(period - dAngle + coeffs._mFI1);
                }
                else if (-coeffs._mB - Math.Abs(coeffs._mC) >= 1.0)
                {
                    //  -(1+C)/B <= 1 and (1-C)/B >= -1 ==>
                    //(U=[aDAngle1;aDAngle2]+aFI1) || (U=[2*PI-aDAngle2;2*PI-aDAngle1]+aFI1),
                    // where aDAngle1 = acos(-(myCoeffs.mC + 1) / myCoeffs.mB),
                    //      aDAngle2 = acos((1 - myCoeffs.mC) / myCoeffs.mB)

                    double arg1 = -(coeffs._mC + 1) / coeffs._mB,
                        arg2 = (1 - coeffs._mC) / coeffs._mB;
                    if (arg1 > 1.0)
                    {
                        arg1 = 1.0;
                    }
                    if (arg1 < -1.0)
                    {
                        arg1 = -1.0;
                    }

                    if (arg2 > 1.0)
                    {
                        arg2 = 1.0;
                    }
                    if (arg2 < -1.0)
                    {
                        arg2 = -1.0;
                    }

                    double dAngle1 = Math.Acos(arg1), dAngle2 = Math.Acos(arg2);
                    uRange[0].Add(dAngle1 + coeffs._mFI1);
                    uRange[0].Add(dAngle2 + coeffs._mFI1);
                    uRange[1].Add(period - dAngle2 + coeffs._mFI1);
                    uRange[1].Add(period - dAngle1 + coeffs._mFI1);
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }

            return true;
        }

        //=======================================================================
        // function : BoundaryEstimation
        // purpose  : Rough estimation of the parameter range.
        //=======================================================================
        internal void BoundaryEstimation(in Cylinder3D cy1,
                                         in Cylinder3D cy2,
                                         ref BoundRange outBoxS1,
                                         ref BoundRange outBoxS2)
        {
            Direction3D d1 = cy1.Axis.Direction, d2 = cy2.Axis.Direction;
            double r1 = cy1.Radius, r2 = cy2.Radius;

            // Let consider a parallelogram. Its edges are parallel to aD1 and aD2.
            // Its altitudes are equal to 2*aR1 and 2*aR2 (diameters of the cylinders).
            // In fact, this parallelogram is a projection of the cylinders to the plane
            // created by the intersected axes aD1 and aD2 (if the axes are skewed then
            // one axis can be translated by parallel shifting till intersection).

            double cosA = d1.Dot(d2);
            double sqSinA = d1.XYZ().CrossSquareMagnitude(d2.XYZ());

            // If sine is small then it can be compared with angle.
            if (sqSinA < global::Kerf.Engine.Foundation.Precision.Angular() * global::Kerf.Engine.Foundation.Precision.Angular())
            {
                return;
            }

            // Half of delta V. Delta V is a distance between
            // projections of two opposite parallelogram vertices
            //(joined by the maximal diagonal) to the cylinder axis.
            double sinA = Math.Sqrt(sqSinA);
            double absCosA = Math.Abs(cosA);
            double hdv1 = (r1 * absCosA + r2) / sinA, hdv2 = (r2 * absCosA + r1) / sinA;

            // V-parameters of intersection point of the axes (in case of skewed axes,
            // see comment above).
            ExtremaLineLine(cy1.Axis, cy2.Axis, cosA, sqSinA, out double v01, out double v02);

            outBoxS1.Add(v01 - hdv1);
            outBoxS1.Add(v01 + hdv1);

            outBoxS2.Add(v02 - hdv2);
            outBoxS2.Add(v02 + hdv2);

            outBoxS1.Enlarge(global::Kerf.Engine.Foundation.Precision.Confusion());
            outBoxS2.Enlarge(global::Kerf.Engine.Foundation.Precision.Confusion());

            _uvSurf1.Get(out double u1, out double v1, out double u2, out double v2);
            outBoxS1.Common(new BoundRange(v1, v2));

            _uvSurf2.Get(out u1, out v1, out u2, out v2);
            outBoxS2.Common(new BoundRange(v1, v2));
        }
    }

    //=======================================================================
    // function : MinMax
    // purpose  : Replaces theParMIN = MIN(theParMIN, theParMAX),
    //                    theParMAX = MAX(theParMIN, theParMAX).
    //=======================================================================
    private static void MinMax(ref double parMIN, ref double parMAX)
    {
        if (parMIN > parMAX)
        {
            double aux = parMAX;
            parMAX = parMIN;
            parMIN = aux;
        }
    }

    //=======================================================================
    // function : ExtremaLineLine
    // purpose  : Computes extrema between the given lines. Returns parameters
    //          on correspond curve (see correspond method for Extrema_ExtElC class).
    //=======================================================================
    private static void ExtremaLineLine(in Axis3D c1,
                                        in Axis3D c2,
                                        double cosA,
                                        double sqSinA,
                                        out double par1,
                                        out double par2)
    {
        Direction3D d1 = c1.Direction, d2 = c2.Direction;

        Xyz l1L2 = c2.Location.XYZ() - c1.Location.XYZ();
        double d1L = d1.XYZ().Dot(l1L2), d2L = d2.XYZ().Dot(l1L2);

        par1 = (d1L - cosA * d2L) / sqSinA;
        par2 = (cosA * d1L - d2L) / sqSinA;
    }

    //=======================================================================
    // function : VBoundaryPrecise
    // purpose  : By default, we shall consider, that V1 and V2 will be increased
    //            if U1 is increased. But if it is not, new V1set and/or V2set
    //            must be computed as [V1current - DeltaV1] (analogically
    //            for V2). This function processes this case.
    //=======================================================================
    private static void VBoundaryPrecise(Matrix matr,
                                         double v1AfterDecrByDelta,
                                         double v2AfterDecrByDelta,
                                         ref double v1Set,
                                         ref double v2Set)
    {
        // Now we are going to define if V1 (and V2) increases
        //(or decreases) when U1 will increase.
        const int nbDim = 3;
        Matrix syst = new(1, nbDim, 1, nbDim);

        syst.SetCol(1, matr.Col(1));
        syst.SetCol(2, matr.Col(2));
        syst.SetCol(3, matr.Col(4));

        // We have the system (see comment to StepComputing(...) function)
        //     {a11*dV1 + a12*dV2 + a14*dU2 = -a13*dU1
        //     {a21*dV1 + a22*dV2 + a24*dU2 = -a23*dU1
        //     {a31*dV1 + a32*dV2 + a34*dU2 = -a33*dU1

        double det = syst.Determinant();

        syst.SetCol(1, matr.Col(3));
        double det1 = syst.Determinant();

        syst.SetCol(1, matr.Col(1));
        syst.SetCol(2, matr.Col(3));

        double det2 = syst.Determinant();

        // Now,
        //     dV1 = -dU1*aDet1/aDet
        //     dV2 = -dU1*aDet2/aDet

        // If U1 is increased then dU1 > 0.
        // If (aDet1/aDet > 0) then dV1 < 0 and
        // V1 will be decreased after increasing U1.

        // We have analogical situation with V2-parameter.

        if (det * det1 > 0.0)
        {
            v1Set = v1AfterDecrByDelta;
        }

        if (det * det2 > 0.0)
        {
            v2Set = v2AfterDecrByDelta;
        }
    }

    //=======================================================================
    // function : DeltaU1Computing
    // purpose  : Computes new step for U1 parameter.
    //=======================================================================
    private static bool DeltaU1Computing(Matrix syst, math_Vector free, ref double deltaU1Found)
    {
        double det = syst.Determinant();

        if (Math.Abs(det) > ANulValue)
        {
            Matrix syst1 = new(syst);
            syst1.SetCol(2, free);

            deltaU1Found = Math.Abs(syst1.Determinant() / det);
            return true;
        }

        return false;
    }

    //=======================================================================
    // function : StepComputing
    // purpose  :
    //
    // Attention!!!:
    //            theMatr must have 3*5-dimension strictly.
    //            For system
    //                {a11*V1+a12*V2+a13*dU1+a14*dU2=b1;
    //                {a21*V1+a22*V2+a23*dU1+a24*dU2=b2;
    //                {a31*V1+a32*V2+a33*dU1+a34*dU2=b3;
    //            theMatr must be following:
    //                (a11 a12 a13 a14 b1)
    //                (a21 a22 a23 a24 b2)
    //                (a31 a32 a33 a34 b3)
    //=======================================================================
    private static bool StepComputing(Matrix matr,
                                      double v1Cur,
                                      double v2Cur,
                                      double deltaV1,
                                      double deltaV2,
                                      ref double deltaU1Found)
    {
        bool isSuccess = false;
        deltaU1Found /* = theDeltaU2Found*/ = RealLast();
        // theV1Found = theV1set;
        // theV2Found = theV2Set;
        const int nbDim = 3;

        Matrix syst = new(1, nbDim, 1, nbDim);
        math_Vector free = new(1, nbDim);

        // By default, increasing V1(U1) and V2(U1) functions is
        // considered
        double v1Set = v1Cur + deltaV1, v2Set = v2Cur + deltaV2;

        // However, what is indeed?
        VBoundaryPrecise(matr, v1Cur - deltaV1, v2Cur - deltaV2, ref v1Set, ref v2Set);

        syst.SetCol(2, matr.Col(3));
        syst.SetCol(3, matr.Col(4));

        for (int i = 0; i < 2; i++)
        {
            if (i == 0)
            { // V1 is known
                syst.SetCol(1, matr.Col(2));
                free.Set(1, nbDim, matr.Col(5) - v1Set * matr.Col(1));
            }
            else
            { // i==1 => V2 is known
                syst.SetCol(1, matr.Col(1));
                free.Set(1, nbDim, matr.Col(5) - v2Set * matr.Col(2));
            }

            double newDU = deltaU1Found;
            if (DeltaU1Computing(syst, free, ref newDU))
            {
                isSuccess = true;
                if (newDU < deltaU1Found)
                {
                    deltaU1Found = newDU;
                }
            }
        }

        if (!isSuccess)
        {
            free.Initialized(matr.Col(5) - v1Set * matr.Col(1) - v2Set * matr.Col(2));
            Matrix syst1 = new(1, nbDim, 1, 2);
            syst1.SetCol(1, syst.Col(2));
            syst1.SetCol(2, syst.Col(3));

            // Now we have overdetermined system.

            double det1 = matr[1, 3] * matr[2, 4] - matr[2, 3] * matr[1, 4];
            double det2 = matr[1, 3] * matr[3, 4] - matr[3, 3] * matr[1, 4];
            double det3 = matr[2, 3] * matr[3, 4] - matr[3, 3] * matr[2, 4];
            double absD1 = Math.Abs(det1);
            double absD2 = Math.Abs(det2);
            double absD3 = Math.Abs(det3);

            if (absD1 >= absD2)
            {
                if (absD1 >= absD3)
                {
                    // Det1
                    if (absD1 <= ANulValue)
                    {
                        return isSuccess;
                    }

                    deltaU1Found = Math.Abs(free[1] * matr[2, 4] - free[2] * matr[1, 4]) / absD1;
                    isSuccess = true;
                }
                else
                {
                    // Det3
                    if (absD3 <= ANulValue)
                    {
                        return isSuccess;
                    }

                    deltaU1Found = Math.Abs(free[2] * matr[3, 4] - free[3] * matr[2, 4]) / absD3;
                    isSuccess = true;
                }
            }
            else
            {
                if (absD2 >= absD3)
                {
                    // Det2
                    if (absD2 <= ANulValue)
                    {
                        return isSuccess;
                    }

                    deltaU1Found = Math.Abs(free[1] * matr[3, 4] - free[3] * matr[1, 4]) / absD2;
                    isSuccess = true;
                }
                else
                {
                    // Det3
                    if (absD3 <= ANulValue)
                    {
                        return isSuccess;
                    }

                    deltaU1Found = Math.Abs(free[2] * matr[3, 4] - free[3] * matr[2, 4]) / absD3;
                    isSuccess = true;
                }
            }
        }

        return isSuccess;
    }

    //=======================================================================
    // function : CyCyAnalyticalIntersect
    // purpose  : Checks if intersection curve is analytical (line, circle, ellipse)
    //            and returns these curves.
    //=======================================================================
    private static bool CyCyAnalyticalIntersect(in SurfaceQuadric quad1,
                                                in SurfaceQuadric quad2,
                                                QuadQuadGeo inter,
                                                double tol,
                                                ref bool empty,
                                                ref bool same,
                                                ref bool multpoint,
                                                NCollection_Sequence<PatchLine> slin,
                                                NCollection_Sequence<PatchPoint> spnt)
    {
        PatchPoint ptsol = new();

        int i;

        WalkingTypeTrans trans1, trans2;
        ResultType typint;

        Ellipse3D elipsol;
        Line3D linsol;

        Cylinder3D cy1 = quad1.Cylinder();
        Cylinder3D cy2 = quad2.Cylinder();

        typint = inter.TypeInter();
        int nbSol = inter.SolutionCount;
        empty = false;
        same = false;

        switch (typint)
        {
            case global::Kerf.Engine.Intersection.ResultType.Empty:
            {
                empty = true;
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Same:
            {
                same = true;
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Point:
            {
                Point3D psol = inter.Point(1);
                ptsol.SetValue(psol, tol, true);

                double U1 = 0.0, V1 = 0.0, U2 = 0.0, V2 = 0.0;
                quad1.Parameters(psol, ref U1, ref V1);
                quad2.Parameters(psol, ref U2, ref V2);

                ptsol.SetParameters(U1, V1, U2, V2);
                spnt.Append(ptsol);
            }
            break;

            case global::Kerf.Engine.Intersection.ResultType.Line:
            {
                Point3D ptref;
                if (nbSol == 1)
                { // Cylinders are tangent to each other by line
                    linsol = inter.Line(1);
                    ptref = linsol.Location;

                    // Radius-vectors
                    Direction3D crb1 = new(new Vector3D(ptref, cy1.Location));
                    Direction3D crb2 = new(new Vector3D(ptref, cy2.Location));

                    // outer normal lines
                    Vector3D norm1 = quad1.Normale(ptref);
                    Vector3D norm2 = quad2.Normale(ptref);
                    WalkingSituation situcyl1;
                    WalkingSituation situcyl2;

                    if (crb1.Dot(crb2) < 0.0)
                    { // centre de courbures "opposes"
                        // ATTENTION!!!
                        //         Normal and Radius-vector of the 1st(!) cylinder
                        //         is used for judging what the situation of the 2nd(!)
                        //         cylinder is.

                        if (norm1.Dot(new Vector3D(crb1)) > 0.0)
                        {
                            situcyl2 = WalkingSituation.Inside;
                        }
                        else
                        {
                            situcyl2 = WalkingSituation.Outside;
                        }

                        if (norm2.Dot(new Vector3D(crb2)) > 0.0)
                        {
                            situcyl1 = WalkingSituation.Inside;
                        }
                        else
                        {
                            situcyl1 = WalkingSituation.Outside;
                        }
                    }
                    else
                    {
                        if (cy1.Radius < cy2.Radius)
                        {
                            if (norm1.Dot(new Vector3D(crb1)) > 0.0)
                            {
                                situcyl2 = WalkingSituation.Inside;
                            }
                            else
                            {
                                situcyl2 = WalkingSituation.Outside;
                            }

                            if (norm2.Dot(new Vector3D(crb2)) > 0.0)
                            {
                                situcyl1 = WalkingSituation.Outside;
                            }
                            else
                            {
                                situcyl1 = WalkingSituation.Inside;
                            }
                        }
                        else
                        {
                            if (norm1.Dot(new Vector3D(crb1)) > 0.0)
                            {
                                situcyl2 = WalkingSituation.Outside;
                            }
                            else
                            {
                                situcyl2 = WalkingSituation.Inside;
                            }

                            if (norm2.Dot(new Vector3D(crb2)) > 0.0)
                            {
                                situcyl1 = WalkingSituation.Inside;
                            }
                            else
                            {
                                situcyl1 = WalkingSituation.Outside;
                            }
                        }
                    }

                    GLine glig = new(linsol, true, situcyl1, situcyl2);
                    slin.Append(glig);
                }
                else
                {
                    for (i = 1; i <= nbSol; i++)
                    {
                        linsol = inter.Line(i);
                        ptref = linsol.Location;
                        Vector3D lsd = new(linsol.Direction);

                        // Theoretically, qwe = +/- 1.0.
                        double qwe = lsd.DotCross(quad2.Normale(ptref), quad1.Normale(ptref));
                        if (qwe > 0.00000001)
                        {
                            trans1 = WalkingTypeTrans.Out;
                            trans2 = WalkingTypeTrans.In;
                        }
                        else if (qwe < -0.00000001)
                        {
                            trans1 = WalkingTypeTrans.In;
                            trans2 = WalkingTypeTrans.Out;
                        }
                        else
                        {
                            trans1 = trans2 = WalkingTypeTrans.Undecided;
                        }

                        GLine glig = new(linsol, false, trans1, trans2);
                        slin.Append(glig);
                    }
                }
            }
            break;

            case Ellipse:
            {
                Vector3D tgt;
                Point3D ptref;
                PatchPoint pmult1 = new(), pmult2 = new();

                elipsol = inter.Ellipse(1);

                Point3D pttang1 = global::Kerf.Engine.Geometry.ElCLib.Value(0.5 * M_PI, elipsol);
                Point3D pttang2 = global::Kerf.Engine.Geometry.ElCLib.Value(1.5 * M_PI, elipsol);

                multpoint = true;
                pmult1.SetValue(pttang1, tol, true);
                pmult2.SetValue(pttang2, tol, true);
                pmult1.SetMultiple(true);
                pmult2.SetMultiple(true);

                double oU1 = 0.0, oV1 = 0.0, oU2 = 0.0, oV2 = 0.0;
                quad1.Parameters(pttang1, ref oU1, ref oV1);
                quad2.Parameters(pttang1, ref oU2, ref oV2);

                pmult1.SetParameters(oU1, oV1, oU2, oV2);
                quad1.Parameters(pttang2, ref oU1, ref oV1);
                quad2.Parameters(pttang2, ref oU2, ref oV2);

                pmult2.SetParameters(oU1, oV1, oU2, oV2);

                // on traite la premiere ellipse

                //-- Calcul de la Transition de la ligne
                global::Kerf.Engine.Geometry.ElCLib.D1(0.0, elipsol, out ptref, out tgt);

                // Theoretically, qwe = +/- |Tgt|.
                double qwe = tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref));
                if (qwe > 0.00000001)
                {
                    trans1 = WalkingTypeTrans.Out;
                    trans2 = WalkingTypeTrans.In;
                }
                else if (qwe < -0.00000001)
                {
                    trans1 = WalkingTypeTrans.In;
                    trans2 = WalkingTypeTrans.Out;
                }
                else
                {
                    trans1 = trans2 = WalkingTypeTrans.Undecided;
                }

                //-- Transition calculee au point 0 -> Trans2 , Trans1
                //-- car ici, on devarit calculer en PI
                GLine glig = new(elipsol, false, trans2, trans1);
                //
                {
                    double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
                    PatchPoint ip = new();
                    Point3D p = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, elipsol);
                    //
                    ip.SetValue(p, tol, false);
                    ip.SetMultiple(false);
                    //
                    quad1.Parameters(p, ref u1, ref v1);
                    quad2.Parameters(p, ref u2, ref v2);

                    ip.SetParameters(u1, v1, u2, v2);
                    //
                    ip.SetParameter(0.0);
                    glig.AddVertex(ip);
                    glig.SetFirstPoint(1);
                    //
                    ip.SetParameter(2.0 * M_PI);
                    glig.AddVertex(ip);
                    glig.SetLastPoint(2);
                }
                //
                pmult1.SetParameter(0.5 * M_PI);
                glig.AddVertex(pmult1);
                //
                pmult2.SetParameter(1.5 * M_PI);
                glig.AddVertex(pmult2);

                //
                slin.Append(glig);

                //-- Transitions calculee au point 0    OK
                //
                // on traite la deuxieme ellipse
                elipsol = inter.Ellipse(2);

                double param1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(elipsol, pttang1);
                double param2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(elipsol, pttang2);
                double parampourtransition = 0.0;
                if (param1 < param2)
                {
                    pmult1.SetParameter(0.5 * M_PI);
                    pmult2.SetParameter(1.5 * M_PI);
                    parampourtransition = M_PI;
                }
                else
                {
                    pmult1.SetParameter(1.5 * M_PI);
                    pmult2.SetParameter(0.5 * M_PI);
                    parampourtransition = 0.0;
                }

                //-- Calcul des transitions de ligne pour la premiere ligne
                global::Kerf.Engine.Geometry.ElCLib.D1(parampourtransition, elipsol, out ptref, out tgt);

                // Theoretically, qwe = +/- |Tgt|.
                qwe = tgt.DotCross(quad2.Normale(ptref), quad1.Normale(ptref));
                if (qwe > 0.00000001)
                {
                    trans1 = WalkingTypeTrans.Out;
                    trans2 = WalkingTypeTrans.In;
                }
                else if (qwe < -0.00000001)
                {
                    trans1 = WalkingTypeTrans.In;
                    trans2 = WalkingTypeTrans.Out;
                }
                else
                {
                    trans1 = trans2 = WalkingTypeTrans.Undecided;
                }

                //-- La transition a ete calculee sur un point de cette ligne
                glig = new GLine(elipsol, false, trans1, trans2);
                //
                {
                    double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
                    PatchPoint ip = new();
                    Point3D p = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, elipsol);
                    //
                    ip.SetValue(p, tol, false);
                    ip.SetMultiple(false);
                    //

                    quad1.Parameters(p, ref u1, ref v1);
                    quad2.Parameters(p, ref u2, ref v2);

                    ip.SetParameters(u1, v1, u2, v2);
                    //
                    ip.SetParameter(0.0);
                    glig.AddVertex(ip);
                    glig.SetFirstPoint(1);
                    //
                    ip.SetParameter(2.0 * M_PI);
                    glig.AddVertex(ip);
                    glig.SetLastPoint(2);
                }
                //
                glig.AddVertex(pmult1);
                glig.AddVertex(pmult2);
                //
                slin.Append(glig);
            }
            break;

            case Parabola:
            case Hyperbola:
                throw new FailureException("IntCyCy(): Wrong intersection type!");

            case Circle:
            // Circle is useful when we will work with trimmed surfaces
            // (two cylinders can be tangent by their basises, e.g. circle)
            case NoGeometricSolution:
            default:
                return false;
        }

        return true;
    }

    //=======================================================================
    // function : ShortCosForm
    // purpose  : Represents theCosFactor*cosA+theSinFactor*sinA as
    //            theCoeff*cos(A-theAngle) if it is possibly (all angles are
    //            in radians).
    //=======================================================================
    private static void ShortCosForm(double cosFactor,
                                     double sinFactor,
                                     ref double coeff,
                                     ref double angle)
    {
        coeff = Math.Sqrt(cosFactor * cosFactor + sinFactor * sinFactor);
        angle = 0.0;
        if (IsEqual(coeff, 0.0))
        {
            angle = 0.0;
            return;
        }

        angle = Math.Acos(Math.Abs(cosFactor / coeff));

        if (sinFactor > 0.0)
        {
            if (IsEqual(cosFactor, 0.0))
            {
                angle = M_PI / 2.0;
            }
            else if (cosFactor < 0.0)
            {
                angle = M_PI - angle;
            }
        }
        else if (IsEqual(sinFactor, 0.0))
        {
            if (cosFactor < 0.0)
            {
                angle = M_PI;
            }
        }
        if (sinFactor < 0.0)
        {
            if (cosFactor > 0.0)
            {
                angle = 2.0 * M_PI - angle;
            }
            else if (IsEqual(cosFactor, 0.0))
            {
                angle = 3.0 * M_PI / 2.0;
            }
            else if (cosFactor < 0.0)
            {
                angle = M_PI + angle;
            }
        }
    }

    //=======================================================================
    // function : InscribePoint
    // purpose  : If theFlForce==TRUE theUGiven will be changed forcefully
    //            even if theUGiven is already inscibed in the boundary
    //            (if it is possible; i.e. if new theUGiven is inscribed
    //            in the boundary, too).
    //=======================================================================
    private static bool InscribePoint(double ufTarget,
                                      double ulTarget,
                                      ref double uGiven,
                                      double tol2D,
                                      double period,
                                      bool flForce)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(uGiven))
        {
            return false;
        }

        if ((ufTarget - uGiven <= tol2D) && (uGiven - ulTarget <= tol2D))
        { // it has already inscribed

            /*
                     Utf    U      Utl
                      +     *       +
            */

            if (flForce)
            {
                double utemp = uGiven + period;
                if ((ufTarget - utemp <= tol2D) && (utemp - ulTarget <= tol2D))
                {
                    uGiven = utemp;
                    return true;
                }

                utemp = uGiven - period;
                if ((ufTarget - utemp <= tol2D) && (utemp - ulTarget <= tol2D))
                {
                    uGiven = utemp;
                }
            }

            return true;
        }

        double uf = ufTarget - tol2D;
        double ul = uf + period;

        uGiven = global::Kerf.Engine.Geometry.ElCLib.InPeriod(uGiven, uf, ul);

        return ((ufTarget - uGiven <= tol2D) && (uGiven - ulTarget <= tol2D));
    }

    //=======================================================================
    // function : InscribeInterval
    // purpose  : Shifts theRange in order to make at least one of its
    //            boundary in the range [theUfTarget, theUlTarget]
    //=======================================================================
    private static bool InscribeInterval(double ufTarget,
                                         double ulTarget,
                                         ref BoundRange range,
                                         double tol2D,
                                         double period)
    {
        double upar = 0.0;
        if (!range.GetMin(ref upar))
        {
            return false;
        }

        double delta = range.Delta();
        if (InscribePoint(ufTarget,
                          ulTarget,
                          ref upar,
                          tol2D,
                          period,
                          (Math.Abs(ulTarget - upar) < tol2D)))
        {
            range.SetVoid();
            range.Add(upar);
            range.Add(upar + delta);
        }
        else
        {
            if (!range.GetMax(ref upar))
            {
                return false;
            }

            if (InscribePoint(ufTarget,
                              ulTarget,
                              ref upar,
                              tol2D,
                              period,
                              (Math.Abs(ufTarget - upar) < tol2D)))
            {
                range.SetVoid();
                range.Add(upar);
                range.Add(upar - delta);
            }
            else
            {
                return false;
            }
        }

        return true;
    }

    //=======================================================================
    // function : ExcludeNearElements
    // purpose  : Checks if theArr contains two almost equal elements.
    //            If it is true then one of equal elements will be excluded
    //            (made infinite).
    //           Returns TRUE if at least one element of theArr has been changed.
    // ATTENTION!!!
    //           1.0 Every not infinite element of theArr is considered to be
    //            in [0, T] interval (where T is the period);
    //           2.0 theArr must be sorted in ascending order.
    //=======================================================================
    private static bool ExcludeNearElements(Span<double> arr,
                                            int nOfMembers,
                                            double uSurf1f,
                                            double uSurf1l,
                                            double tol)
    {
        bool retVal = false;
        for (int i = 1; i < nOfMembers; i++)
        {
            ref double a = ref arr[i];
            ref double b = ref arr[i - 1];

            // Here, anA >= anB

            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(a))
            {
                break;
            }

            if ((a - b) < tol)
            {
                if ((b != 0.0) && (b != uSurf1f) && (b != uSurf1l))
                {
                    a = (a + b) / 2.0;
                }
                else
                {
                    a = b;
                }

                // Make this element infinite an forget it
                //(we will not use it in next iterations).
                b = global::Kerf.Engine.Foundation.Precision.Infinite();
                retVal = true;
            }
        }

        return retVal;
    }

    // std::sort of doubles as MSVC does it on up to 32 elements (_Insertion_sort_unchecked): ties (0.0 and -0.0)
    // keep their order (decision 610). OCCT sorts at most 12 critical points.
    private static void MsvcInsertionSort(Span<double> arr)
    {
        for (int mid = 1; mid < arr.Length; mid++)
        {
            int hole = mid;
            double val = arr[mid];
            if (val < arr[0])
            {
                // found new earliest element, move to front
                for (int k = mid; k > 0; k--)
                {
                    arr[k] = arr[k - 1];
                }
                arr[0] = val;
            }
            else
            {
                // look for insertion point after first
                for (int prev = hole - 1; val < arr[prev]; hole = prev, prev--)
                {
                    arr[hole] = arr[prev]; // move hole down
                }
                arr[hole] = val; // insert element in hole
            }
        }
    }

    //=======================================================================
    // function : AddPointIntoWL
    // purpose  : Surf1 is a surface, whose U-par is variable.
    //           If theFlBefore == TRUE then we enable the U1-parameter
    //            of the added point to be less than U1-parameter of
    //           previously added point (in general U1-parameter is
    //           always increased; therefore, this situation is abnormal).
    //           If theOnlyCheck==TRUE then no point will be added to theLine.
    //=======================================================================
    private static bool AddPointIntoWL(in SurfaceQuadric quad1,
                                       in SurfaceQuadric quad2,
                                       ComputationMethods.StCoeffsValue coeffs,
                                       bool isTheReverse,
                                       bool isThePrecise,
                                       in Point2D pntOnSurf1,
                                       in Point2D pntOnSurf2,
                                       double ufSurf1,
                                       double ulSurf1,
                                       double ufSurf2,
                                       double ulSurf2,
                                       double vfSurf1,
                                       double vlSurf1,
                                       double vfSurf2,
                                       double vlSurf2,
                                       double periodOfSurf1,
                                       LineOn2S line,
                                       int wlIndex,
                                       double tol3D,
                                       double tol2D,
                                       bool flBefore = false,
                                       bool onlyCheck = false)
    {
        // Check if the point is in the domain or can be inscribed in the domain after adjusting.

        Point3D pt1 = quad1.Value(pntOnSurf1.X, pntOnSurf1.Y),
            pt2 = quad2.Value(pntOnSurf2.X, pntOnSurf2.Y);

        double u1par = pntOnSurf1.X;

        // aU1par always increases. Therefore, we must reduce its
        // value in order to continue creation of WLine.
        if (!InscribePoint(ufSurf1,
                           ulSurf1,
                           ref u1par,
                           tol2D,
                           periodOfSurf1,
                           u1par > 0.5 * (ufSurf1 + ulSurf1)))
        {
            return false;
        }

        if ((line.PointCount > 0) && ((ulSurf1 - ufSurf1) >= (periodOfSurf1 - tol2D))
            && (((u1par + periodOfSurf1 - ulSurf1) <= tol2D)
                || ((u1par - periodOfSurf1 - ufSurf1) >= tol2D)))
        {
            // aU1par can be adjusted to both theUlSurf1 and theUfSurf1
            // with equal possibilities. This code fragment allows choosing
            // correct parameter from these two variants.

            double u1, v1;
            if (isTheReverse)
            {
                line.Value(line.PointCount).ParametersOnS2(out u1, out v1);
            }
            else
            {
                line.Value(line.PointCount).ParametersOnS1(out u1, out v1);
            }

            double delta = u1 - u1par;
            if (2.0 * Math.Abs(delta) > periodOfSurf1)
            {
                u1par += Math.CopySign(periodOfSurf1, delta);
            }
        }

        double u2par = pntOnSurf2.X;
        if (!InscribePoint(ufSurf2, ulSurf2, ref u2par, tol2D, periodOfSurf1, false))
        {
            return false;
        }

        double v1par = pntOnSurf1.Y;
        if ((v1par - vlSurf1 > tol2D) || (vfSurf1 - v1par > tol2D))
        {
            return false;
        }

        double v2par = pntOnSurf2.Y;
        if ((v2par - vlSurf2 > tol2D) || (vfSurf2 - v2par > tol2D))
        {
            return false;
        }

        // Get intersection point and add it in the WL
        PntOn2S pnt = new();

        if (isTheReverse)
        {
            pnt.SetValue(new Point3D((pt1.XYZ() + pt2.XYZ()) / 2.0), u2par, v2par, u1par, v1par);
        }
        else
        {
            pnt.SetValue(new Point3D((pt1.XYZ() + pt2.XYZ()) / 2.0), u1par, v1par, u2par, v2par);
        }

        int nbPnts = line.PointCount;
        if (nbPnts > 0)
        {
            double ul, vl;
            PntOn2S plast = line.Value(nbPnts);
            if (isTheReverse)
            {
                plast.ParametersOnS2(out ul, out vl);
            }
            else
            {
                plast.ParametersOnS1(out ul, out vl);
            }

            if (!flBefore && (u1par <= ul))
            {
                // Parameter value must be increased if theFlBefore == FALSE.

                u1par += periodOfSurf1;

                // The condition is as same as in
                // InscribePoint(...) function
                if ((ufSurf1 - u1par > tol2D) || (u1par - ulSurf1 > tol2D))
                {
                    // New aU1par is out of target interval.
                    // Go back to old value.

                    return false;
                }
            }

            if (onlyCheck)
            {
                return true;
            }

            // theTol2D is minimal step along parameter changed.
            // Therefore, if we apply this minimal step two
            // neighbour points will be always "same". Consequently,
            // we should reduce tolerance for IsSame checking.
            double dTol = 1.0 - Epsilon(1.0);
            if (pnt.IsSame(plast, tol3D * dTol, tol2D * dTol))
            {
                line.RemovePoint(nbPnts);
            }
        }

        if (onlyCheck)
        {
            return true;
        }

        line.Add(pnt);

        if (!isThePrecise)
        {
            return true;
        }

        // Try to precise existing WLine
        nbPnts = line.PointCount;
        if (nbPnts >= 3)
        {
            double u1, u2, u3, v;
            if (isTheReverse)
            {
                line.Value(nbPnts).ParametersOnS2(out u3, out v);
                line.Value(nbPnts - 1).ParametersOnS2(out u2, out v);
                line.Value(nbPnts - 2).ParametersOnS2(out u1, out v);
            }
            else
            {
                line.Value(nbPnts).ParametersOnS1(out u3, out v);
                line.Value(nbPnts - 1).ParametersOnS1(out u2, out v);
                line.Value(nbPnts - 2).ParametersOnS1(out u1, out v);
            }

            double stepPrev = u2 - u1;
            double step = u3 - u2;

            int deltaStep = RealToInt(stepPrev / step);

            if ((1 < deltaStep) && (deltaStep < 2000))
            {
                // Add new points in case of non-uniform distribution of existing points
                SeekAdditionalPoints(quad1,
                                     quad2,
                                     line,
                                     coeffs,
                                     wlIndex,
                                     deltaStep,
                                     nbPnts - 2,
                                     nbPnts - 1,
                                     tol2D,
                                     periodOfSurf1,
                                     isTheReverse);
            }
        }

        return true;
    }

    //=======================================================================
    // function : SeekAdditionalPoints
    // purpose  : Inserts additional intersection points between neighbor points.
    //            This action is bone with several iterations. During every iteration,
    //          new point is inserted in middle of every interval.
    //            The process will be finished if NbPoints >= theMinNbPoints.
    //=======================================================================
    private static void SeekAdditionalPoints(in SurfaceQuadric quad1,
                                             in SurfaceQuadric quad2,
                                             LineOn2S? line,
                                             ComputationMethods.StCoeffsValue coeffs,
                                             int wlIndex,
                                             int minNbPoints,
                                             int startPointOnLine,
                                             int endPointOnLine,
                                             double tol2D,
                                             double periodOfSurf2,
                                             bool isTheReverse)
    {
        if (line is null)
        {
            return;
        }

        int nbPoints = endPointOnLine - startPointOnLine + 1;

        double minDeltaParam = tol2D;

        {
            double u1, v1, u2, v2;

            if (isTheReverse)
            {
                line.Value(startPointOnLine).ParametersOnS2(out u1, out v1);
                line.Value(endPointOnLine).ParametersOnS2(out u2, out v2);
            }
            else
            {
                line.Value(startPointOnLine).ParametersOnS1(out u1, out v1);
                line.Value(endPointOnLine).ParametersOnS1(out u2, out v2);
            }

            minDeltaParam = Max(Math.Abs(u2 - u1) / IntToReal(minNbPoints), minDeltaParam);
        }

        int lastPointIndex = endPointOnLine;
        double u1prec = 0.0, v1prec = 0.0, u2prec = 0.0, v2prec = 0.0;

        int nbPointsPrev = 0;
        do
        {
            nbPointsPrev = nbPoints;
            for (int fp = startPointOnLine, lp = 0; fp < lastPointIndex; fp = lp + 1)
            {
                double u1f, v1f; // first point in 1st suraface
                double u1l, v1l; // last  point in 1st suraface

                double u2f, v2f; // first point in 2nd suraface
                double u2l, v2l; // last  point in 2nd suraface

                lp = fp + 1;

                if (isTheReverse)
                {
                    line.Value(fp).ParametersOnS2(out u1f, out v1f);
                    line.Value(lp).ParametersOnS2(out u1l, out v1l);

                    line.Value(fp).ParametersOnS1(out u2f, out v2f);
                    line.Value(lp).ParametersOnS1(out u2l, out v2l);
                }
                else
                {
                    line.Value(fp).ParametersOnS1(out u1f, out v1f);
                    line.Value(lp).ParametersOnS1(out u1l, out v1l);

                    line.Value(fp).ParametersOnS2(out u2f, out v2f);
                    line.Value(lp).ParametersOnS2(out u2l, out v2l);
                }

                if (Math.Abs(u1l - u1f) <= minDeltaParam)
                {
                    // Step is minimal. It is not necessary to divide it.
                    continue;
                }

                u1prec = 0.5 * (u1f + u1l);

                if (!ComputationMethods.CylCylComputeParameters(u1prec,
                                                                wlIndex,
                                                                coeffs,
                                                                ref u2prec,
                                                                ref v1prec,
                                                                ref v2prec))
                {
                    continue;
                }

                MinMax(ref u2f, ref u2l);
                if (!InscribePoint(u2f, u2l, ref u2prec, tol2D, periodOfSurf2, false))
                {
                    continue;
                }

                Point3D p1 = quad1.Value(u1prec, v1prec), p2 = quad2.Value(u2prec, v2prec);
                Point3D pInt = new(0.5 * (p1.XYZ() + p2.XYZ()));

                PntOn2S ip = new();
                if (isTheReverse)
                {
                    ip.SetValue(pInt, u2prec, v2prec, u1prec, v1prec);
                }
                else
                {
                    ip.SetValue(pInt, u1prec, v1prec, u2prec, v2prec);
                }

                line.InsertBefore(lp, ip);

                nbPoints++;
                lastPointIndex++;
            }

            if (nbPoints >= minNbPoints)
            {
                return;
            }
        } while (nbPoints < minNbPoints && (nbPoints != nbPointsPrev));
    }

    //=======================================================================
    // function : CriticalPointsComputing
    // purpose  : theNbCritPointsMax contains true number of critical points.
    //            It must be initialized correctly before function calling
    //=======================================================================
    private static void CriticalPointsComputing(ComputationMethods.StCoeffsValue coeffs,
                                                double uSurf1f,
                                                double uSurf1l,
                                                double uSurf2f,
                                                double uSurf2l,
                                                double period,
                                                double tol2D,
                                                ref int nbCritPointsMax,
                                                Span<double> u1crit)
    {
        //[0...1] - in these points parameter U1 goes through
        //          the seam-edge of the first cylinder.
        //[2...3] - First and last U1 parameter.
        //[4...5] - in these points parameter U2 goes through
        //          the seam-edge of the second cylinder.
        //[6...9] - in these points an intersection line goes through
        //          U-boundaries of the second surface.
        //[10...11] - Boundary of monotonicity interval of U2(U1) function
        //            (see CylCylMonotonicity() function)

        u1crit[0] = 0.0;
        u1crit[1] = period;
        u1crit[2] = uSurf1f;
        u1crit[3] = uSurf1l;

        double cos = Math.Cos(coeffs._mFI2);
        double bsb = Math.Abs(coeffs._mB);
        if ((coeffs._mC - bsb <= cos) && (cos <= coeffs._mC + bsb))
        {
            double arg = (cos - coeffs._mC) / coeffs._mB;
            if (arg > 1.0)
            {
                arg = 1.0;
            }
            if (arg < -1.0)
            {
                arg = -1.0;
            }

            u1crit[4] = -Math.Acos(arg) + coeffs._mFI1;
            u1crit[5] = Math.Acos(arg) + coeffs._mFI1;
        }

        double sf = Math.Cos(uSurf2f - coeffs._mFI2);
        double sl = Math.Cos(uSurf2l - coeffs._mFI2);
        MinMax(ref sf, ref sl);

        // In accorance with pure mathematic, theU1crit[6] and [8]
        // must be -Precision::Infinite() instead of used +Precision::Infinite()
        u1crit[6] = Math.Abs((sl - coeffs._mC) / coeffs._mB) < 1.0
                         ? -Math.Acos((sl - coeffs._mC) / coeffs._mB) + coeffs._mFI1
                         : global::Kerf.Engine.Foundation.Precision.Infinite();
        u1crit[7] = Math.Abs((sf - coeffs._mC) / coeffs._mB) < 1.0
                         ? -Math.Acos((sf - coeffs._mC) / coeffs._mB) + coeffs._mFI1
                         : global::Kerf.Engine.Foundation.Precision.Infinite();
        u1crit[8] = Math.Abs((sf - coeffs._mC) / coeffs._mB) < 1.0
                         ? Math.Acos((sf - coeffs._mC) / coeffs._mB) + coeffs._mFI1
                         : global::Kerf.Engine.Foundation.Precision.Infinite();
        u1crit[9] = Math.Abs((sl - coeffs._mC) / coeffs._mB) < 1.0
                         ? Math.Acos((sl - coeffs._mC) / coeffs._mB) + coeffs._mFI1
                         : global::Kerf.Engine.Foundation.Precision.Infinite();

        u1crit[10] = coeffs._mFI1;
        u1crit[11] = M_PI + coeffs._mFI1;

        // preparative treatment of array. This array must have faled to contain negative
        // infinity number

        for (int i = 0; i < nbCritPointsMax; i++)
        {
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u1crit[i]))
            {
                continue;
            }

            u1crit[i] = u1crit[i] % period; // fmod
            if (u1crit[i] < 0.0)
            {
                u1crit[i] += period;
            }
        }

        // Here all not infinite elements of theU1crit are in [0, thePeriod) range

        do
        {
            MsvcInsertionSort(u1crit[..nbCritPointsMax]);
        } while (ExcludeNearElements(u1crit, nbCritPointsMax, uSurf1f, uSurf1l, tol2D));

        // Here all not infinite elements in theU1crit are different and sorted.

        while (nbCritPointsMax > 0)
        {
            ref double b = ref u1crit[nbCritPointsMax - 1];
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(b))
            {
                nbCritPointsMax--;
                continue;
            }

            // 1st not infinte element is found

            if (nbCritPointsMax == 1)
            {
                break;
            }

            // Here theNbCritPointsMax > 1

            ref double a = ref u1crit[0];

            // Compare 1st and last significant elements of theU1crit
            // They may still differs by period.

            if (Math.Abs(b - a - period) < tol2D)
            { // E.g. anA == 2.0e-17, anB == (thePeriod-1.0e-18)
                a = (a + b - period) / 2.0;
                b = global::Kerf.Engine.Foundation.Precision.Infinite();
                nbCritPointsMax--;
            }

            // Out of "while(theNbCritPointsMax > 0)" cycle.
            break;
        }
    }

    // OCCT's local enum WLFStatus in CyCyNoGeometric.
    private enum WLFStatus
    {
        // No points have been added in WL
        WLFStatus_Absent = 0,
        // WL contains at least one point
        WLFStatus_Exist = 1,
        // WL has been finished in some critical point
        // We should start new line
        WLFStatus_Broken = 2,
    }

    //=================================================================================================

    private static IntStatus CyCyNoGeometric(in Cylinder3D cyl1,
                                             in Cylinder3D cyl2,
                                             WorkWithBoundaries bw,
                                             BoundRange[] range,
                                             int nbOfRanges /*=2*/,
                                             ref bool isTheEmpty,
                                             NCollection_Sequence<PatchLine> slin,
                                             NCollection_Sequence<PatchPoint> sPnt)
    {
        bw.UVS1().Get(out double uSurf1f, out double vSurf1f, out double uSurf1l, out double vSurf1l);
        bw.UVS2().Get(out double uSurf2f, out double vSurf2f, out double uSurf2l, out double vSurf2l);

        BoundRange rangeS1 = new(), rangeS2 = new();
        bw.BoundaryEstimation(cyl1, cyl2, ref rangeS1, ref rangeS2);
        if (rangeS1.IsVoid || rangeS2.IsVoid)
        {
            return IntStatus.IntStatus_OK;
        }

        {
            // Quotation of the message from issue #26894 (author MSV):
            //"We should return fail status from intersector if the result should be an
            // infinite curve of non-analytical type... I propose to define the limit for the
            // extent as the radius divided by 1e+2 and multiplied by 1e+7.
            // Thus, taking into account the number of valuable digits (15), we provide reliable
            // computations with an error not exceeding R/100."
            const double f = 1.0e+5;
            double maxV1Range = f * cyl1.Radius, maxV2Range = f * cyl2.Radius;
            if ((rangeS1.Delta() > maxV1Range) || (rangeS2.Delta() > maxV2Range))
            {
                return IntStatus.IntStatus_InfiniteSectionCurve;
            }
        }
        //
        bool isGoodIntersection = false;
        double optdu = 0.0;
        for (;;)
        {
            // Checking parameters of cylinders in order to define "good intersection"
            //"Good intersection" means that axes of cylinders are almost perpendicular and
            //  one radius is much smaller than the other and small cylinder is "inside" big one.
            const double toMuchCoeff = 3.0;
            const double critAngle = M_PI / 18.0; // 10 degree
            double r1 = cyl1.Radius;
            double r2 = cyl2.Radius;
            double rmin = 0.0, rmax = 0.0;
            // Radius criterion
            if (r1 > toMuchCoeff * r2)
            {
                rmax = r1;
                rmin = r2;
            }
            else if (r2 > toMuchCoeff * r1)
            {
                rmax = r2;
                rmin = r1;
            }
            else
            {
                break;
            }
            // Angle criterion
            Axis3D ax1 = cyl1.Axis;
            Axis3D ax2 = cyl2.Axis;
            if (!ax1.IsNormal(ax2, critAngle))
            {
                break;
            }
            // Placement criterion
            Line3D l1 = new(ax1), l2 = new(ax2);
            double dist = l1.DistanceTo(l2);
            if (dist > rmax / 2.0)
            {
                break;
            }

            isGoodIntersection = true;
            // Estimation of "optimal" du
            // Relative deflection, absolut deflection is Rmin*aDeflection
            double deflection = 0.001;
            int nbP = 3;
            if (rmin * deflection > 1.0e-3)
            {
                double angle = 1.0e0 - deflection;
                angle = 2.0e0 * Math.Acos(angle);
                nbP = (int)(2.0 * M_PI / angle) + 1;
            }
            optdu = 2.0 * M_PI_2 / (double)(nbP - 1);
            break;
        }
        //
        ComputationMethods.StCoeffsValue equationCoeffs = bw.SICoeffs();
        SurfaceQuadric quad1 = bw.GetQSurface(1);
        SurfaceQuadric quad2 = bw.GetQSurface(2);
        bool isReversed = bw.IsReversed;
        double tol2D = bw.Get2dTolerance();
        double tol3D = bw.Get3dTolerance();
        const double period = 2.0 * M_PI;
        int nbMaxPoints = 1000;
        int nbMinPoints = 200;
        double du;
        if (isGoodIntersection)
        {
            du = optdu;
            nbMaxPoints = 200;
            nbMinPoints = 50;
        }
        else
        {
            du = 2.0 * M_PI / nbMaxPoints;
        }
        int nbPts = Min(RealToInt((uSurf1l - uSurf1f) / du) + 1, RealToInt(20.0 * cyl1.Radius));
        int nbPoints = Min(Max(nbMinPoints, nbPts), nbMaxPoints);
        double stepMin = Max(tol2D, global::Kerf.Engine.Foundation.Precision.PConfusion()),
            stepMax = (uSurf1l - uSurf1f > M_PI / 100.0)
                         ? (uSurf1l - uSurf1f) / IntToReal(nbPoints)
                         : uSurf1l - uSurf1f;

        // The main idea of the algorithm is to change U1-parameter
        //(U-parameter of theCyl1) from aU1f to aU1l with some step
        //(step is adaptive) and to obtain set of intersection points.

        for (int i = 0; i < nbOfRanges; i++)
        {
            if (range[i].IsVoid)
            {
                continue;
            }

            InscribeInterval(uSurf1f, uSurf1l, ref range[i], tol2D, period);
        }

        if (range[0].Union(range[1]))
        {
            // Works only if (theNbOfRanges == 2).
            range[1].SetVoid();
        }

        // Critical points are the value of U1-parameter in the points
        // where WL must be decomposed.

        // When U1 goes through critical points its value is set up to this
        // parameter forcefully and the intersection point is added in the line.
        // After that, the WL is broken (next U1 value will be correspond to the new WL).

        // See CriticalPointsComputing(...) function to get detail information about this array.
        const int nbCritPointsMax = 12;
        Span<double> u1crit = stackalloc double[nbCritPointsMax];
        u1crit.Fill(global::Kerf.Engine.Foundation.Precision.Infinite());

        // V-boundary points (both crossings and tangencies) are caught on-the-fly
        // by the walker via AddBoundaryPoint; see the 1D branch-aware root-finder
        // it uses in place of the rank-deficient 3x3 Newton SearchOnVBounds.

        int nbCritPoints = nbCritPointsMax;
        CriticalPointsComputing(equationCoeffs,
                                uSurf1f,
                                uSurf1l,
                                uSurf2f,
                                uSurf2l,
                                period,
                                tol2D,
                                ref nbCritPoints,
                                u1crit);

        // Getting Walking-line

        const int nbWLines = 2;
        Span<bool> isAddedIntoWL = stackalloc bool[nbWLines];
        Span<double> aU2 = stackalloc double[nbWLines];
        Span<double> aV1 = stackalloc double[nbWLines];
        Span<double> aV2 = stackalloc double[nbWLines];
        Span<WLFStatus> wlFindStatus = stackalloc WLFStatus[nbWLines];
        Span<double> v1Prev = stackalloc double[nbWLines];
        Span<double> v2Prev = stackalloc double[nbWLines];
        Span<double> uexpect = stackalloc double[nbWLines];
        Span<bool> isAddingWLEnabled = stackalloc bool[nbWLines];
        Span<double> criticalDelta = stackalloc double[nbCritPointsMax];
        LineOn2S[] l2S = new LineOn2S[nbWLines];
        WLine[] wLine = new WLine[nbWLines];
        for (int curInterval = 0; curInterval < nbOfRanges; curInterval++)
        {
            // Process every continuous region
            for (int i = 0; i < nbWLines; i++)
            {
                isAddedIntoWL[i] = false;
            }

            double uf = 1.0, ul = 0.0;
            if (!range[curInterval].GetBounds(ref uf, ref ul))
            {
                continue;
            }

            bool isDeltaPeriod = IsEqual(ul - uf, period);

            // Inscribe and sort critical points
            for (int i = 0; i < nbCritPoints; i++)
            {
                InscribePoint(uf, ul, ref u1crit[i], 0.0, period, false);
            }

            MsvcInsertionSort(u1crit[..nbCritPoints]);

            while (uf < ul)
            {
                // Change value of U-parameter on the 1st surface from anUf to anUl
                //(anUf will be modified in the cycle body).
                // Step is computed adaptively (see comments below).

                for (int i = 0; i < nbWLines; i++)
                {
                    l2S[i] = new LineOn2S();
                    wLine[i] = new WLine(l2S[i], false);
                    wLine[i].SetCreatingWayInfo(WLine.IntPatch_WLType.WLImpImp);
                    wlFindStatus[i] = WLFStatus.WLFStatus_Absent;
                    isAddingWLEnabled[i] = true;
                    aU2[i] = aV1[i] = aV2[i] = 0.0;
                    v1Prev[i] = v2Prev[i] = 0.0;
                    uexpect[i] = uf;
                }

                criticalDelta.Clear();
                for (int critPID = 0; critPID < nbCritPoints; critPID++)
                { // We are not interested in elements of aCriticalDelta array
                    // if their index is greater than or equal to aNbCritPoints

                    criticalDelta[critPID] = uf - u1crit[critPID];
                }

                double anU1 = uf, minCriticalParam = uf;
                double u1Prev = uf;
                bool isFirst = true;

                while (anU1 <= ul)
                {
                    // Change value of U-parameter on the 1st surface from anUf to anUl
                    //(anUf will be modified in the cycle body). However, this cycle
                    // can be broken if WL goes though some critical point.
                    // Step is computed adaptively (see comments below).

                    for (int i = 0; i < nbCritPoints; i++)
                    {
                        if ((anU1 - u1crit[i]) * criticalDelta[i] < 0.0)
                        {
                            // WL has gone through i-th critical point
                            anU1 = u1crit[i];

                            for (int j = 0; j < nbWLines; j++)
                            {
                                wlFindStatus[j] = WLFStatus.WLFStatus_Broken;
                                uexpect[j] = anU1;
                            }

                            break;
                        }
                    }

                    if (IsEqual(anU1, ul))
                    {
                        for (int i = 0; i < nbWLines; i++)
                        {
                            wlFindStatus[i] = WLFStatus.WLFStatus_Broken;
                            uexpect[i] = anU1;

                            if (isDeltaPeriod)
                            {
                                // if isAddedIntoWL[i] == TRUE WLine contains only one point
                                //(which was end point of previous WLine). If we will
                                // add point found on the current step WLine will contain only
                                // two points. At that both these points will be equal to the
                                // points found earlier. Therefore, new WLine will repeat
                                // already existing WLine. Consequently, it is necessary
                                // to forbid building new line in this case.

                                isAddingWLEnabled[i] = (!isAddedIntoWL[i]);
                            }
                            else
                            {
                                isAddingWLEnabled[i] =
                                    ((tol2D >= (uexpect[i] - anU1)) || (wlFindStatus[i] == WLFStatus.WLFStatus_Absent));
                            }
                        } // for(int i = 0; i < aNbWLines; i++)
                    }
                    else
                    {
                        for (int i = 0; i < nbWLines; i++)
                        {
                            isAddingWLEnabled[i] =
                                ((tol2D >= (uexpect[i] - anU1)) || (wlFindStatus[i] == WLFStatus.WLFStatus_Absent));
                        } // for(int i = 0; i < aNbWLines; i++)
                    }

                    for (int i = 0; i < nbWLines; i++)
                    {
                        int nbPntsWL = wLine[i] is null ? 0 : wLine[i].Curve().PointCount;

                        if ((wlFindStatus[i] == WLFStatus.WLFStatus_Broken) || (wlFindStatus[i] == WLFStatus.WLFStatus_Absent))
                        { // Begin and end of WLine must be on boundary point
                            // or on seam-edge strictly (if it is possible).

                            double tol = tol2D;
                            ComputationMethods.CylCylComputeParameters(anU1, i, equationCoeffs, ref aU2[i], ref tol);
                            InscribePoint(uSurf2f, uSurf2l, ref aU2[i], tol2D, period, false);

                            tol = Max(tol, tol2D);

                            if (Math.Abs(aU2[i]) <= tol)
                            {
                                aU2[i] = 0.0;
                            }
                            else if (Math.Abs(aU2[i] - period) <= tol)
                            {
                                aU2[i] = period;
                            }
                            else if (Math.Abs(aU2[i] - uSurf2f) <= tol)
                            {
                                aU2[i] = uSurf2f;
                            }
                            else if (Math.Abs(aU2[i] - uSurf2l) <= tol)
                            {
                                aU2[i] = uSurf2l;
                            }
                        }
                        else
                        {
                            ComputationMethods.CylCylComputeParameters(anU1, i, equationCoeffs, ref aU2[i]);
                            InscribePoint(uSurf2f, uSurf2l, ref aU2[i], tol2D, period, false);
                        }

                        if (nbPntsWL == 0)
                        { // the line has not contained any points yet
                            if (((uSurf2f + period - uSurf2l) <= 2.0 * tol2D)
                                && ((Math.Abs(aU2[i] - uSurf2f) < tol2D)
                                    || (Math.Abs(aU2[i] - uSurf2l) < tol2D)))
                            {
                                // In this case aU2[i] can have two values: current aU2[i] or
                                // aU2[i]+aPeriod (aU2[i]-aPeriod). It is necessary to choose
                                // correct value.

                                bool isIncreasing = true;
                                ComputationMethods.CylCylMonotonicity(anU1 + stepMin,
                                                                      i,
                                                                      equationCoeffs,
                                                                      period,
                                                                      ref isIncreasing);

                                // If U2(U1) is increasing and U2 is considered to be equal aUSurf2l
                                // then after the next step (when U1 will be increased) U2 will be
                                // increased too. And we will go out of surface boundary.
                                // Therefore, If U2(U1) is increasing then U2 must be equal aUSurf2f.
                                // Analogically, if U2(U1) is decreasing.
                                if (isIncreasing)
                                {
                                    aU2[i] = uSurf2f;
                                }
                                else
                                {
                                    aU2[i] = uSurf2l;
                                }
                            }
                        }
                        else
                        { // aNbPntsWL > 0
                            if (((uSurf2l - uSurf2f) >= period)
                                && ((Math.Abs(aU2[i] - uSurf2f) < tol2D)
                                    || (Math.Abs(aU2[i] - uSurf2l) < tol2D)))
                            { // end of the line
                                double u2prev, v2prev;
                                if (isReversed)
                                {
                                    wLine[i].Curve().Value(nbPntsWL).ParametersOnS1(out u2prev, out v2prev);
                                }
                                else
                                {
                                    wLine[i].Curve().Value(nbPntsWL).ParametersOnS2(out u2prev, out v2prev);
                                }

                                if (2.0 * Math.Abs(u2prev - aU2[i]) > period)
                                {
                                    if (u2prev > aU2[i])
                                    {
                                        aU2[i] += period;
                                    }
                                    else
                                    {
                                        aU2[i] -= period;
                                    }
                                }
                            }
                        }

                        ComputationMethods.CylCylComputeParameters(anU1,
                                                                   aU2[i],
                                                                   equationCoeffs,
                                                                   ref aV1[i],
                                                                   ref aV2[i]);

                        if (isFirst)
                        {
                            v1Prev[i] = aV1[i];
                            v2Prev[i] = aV2[i];
                        }
                    } // for(int i = 0; i < aNbWLines; i++)

                    isFirst = false;

                    // Looking for points into WLine
                    bool isBroken = false;
                    for (int i = 0; i < nbWLines; i++)
                    {
                        if (!isAddingWLEnabled[i])
                        {
                            bool isBoundIntersect = false;
                            if ((Math.Abs(aV1[i] - vSurf1f) <= tol2D)
                                || ((aV1[i] - vSurf1f) * (v1Prev[i] - vSurf1f) < 0.0))
                            {
                                isBoundIntersect = true;
                            }
                            else if ((Math.Abs(aV1[i] - vSurf1l) <= tol2D)
                                     || ((aV1[i] - vSurf1l) * (v1Prev[i] - vSurf1l) < 0.0))
                            {
                                isBoundIntersect = true;
                            }
                            else if ((Math.Abs(aV2[i] - vSurf2f) <= tol2D)
                                     || ((aV2[i] - vSurf2f) * (v2Prev[i] - vSurf2f) < 0.0))
                            {
                                isBoundIntersect = true;
                            }
                            else if ((Math.Abs(aV2[i] - vSurf2l) <= tol2D)
                                     || ((aV2[i] - vSurf2l) * (v2Prev[i] - vSurf2l) < 0.0))
                            {
                                isBoundIntersect = true;
                            }

                            if (wlFindStatus[i] == WLFStatus.WLFStatus_Broken)
                            {
                                isBroken = true;
                            }

                            if (!isBoundIntersect)
                            {
                                continue;
                            }
                            else
                            {
                                uexpect[i] = anU1;
                            }
                        }

                        // True if the current point already in the domain
                        bool isInscribe =
                            ((uSurf2f - aU2[i]) <= tol2D) && ((aU2[i] - uSurf2l) <= tol2D)
                            && ((vSurf1f - aV1[i]) <= tol2D) && ((aV1[i] - vSurf1l) <= tol2D)
                            && ((vSurf2f - aV2[i]) <= tol2D) && ((aV2[i] - vSurf2l) <= tol2D);

                        // isVIntersect == TRUE if intersection line intersects two (!)
                        // V-bounds of cylinder (1st or 2nd - no matter)
                        bool isVIntersect =
                            (((vSurf1f - aV1[i]) * (vSurf1f - v1Prev[i]) < RealSmall())
                             && ((vSurf1l - aV1[i]) * (vSurf1l - v1Prev[i]) < RealSmall()))
                            || (((vSurf2f - aV2[i]) * (vSurf2f - v2Prev[i]) < RealSmall())
                                && ((vSurf2l - aV2[i]) * (vSurf2l - v2Prev[i]) < RealSmall()));

                        // isFound1 == TRUE if intersection line intersects V-bounds
                        //   (First or Last - no matter) of the 1st cylinder
                        // isFound2 == TRUE if intersection line intersects V-bounds
                        //   (First or Last - no matter) of the 2nd cylinder
                        bool isFound1 = false, isFound2 = false;
                        bool isForce = false;

                        if (wlFindStatus[i] == WLFStatus.WLFStatus_Absent)
                        {
                            if (((uSurf2l - uSurf2f) >= period) && (Math.Abs(anU1 - uSurf1l) < tol2D))
                            {
                                isForce = true;
                            }
                        }

                        bw.AddBoundaryPoint(wLine[i],
                                               anU1,
                                               u1Prev,
                                               minCriticalParam,
                                               aU2[i],
                                               aV1[i],
                                               v1Prev[i],
                                               aV2[i],
                                               v2Prev[i],
                                               i,
                                               isForce,
                                               ref isFound1,
                                               ref isFound2);

                        bool isPrevVBound = !isVIntersect
                                            && ((Math.Abs(v1Prev[i] - vSurf1f) <= tol2D)
                                                || (Math.Abs(v1Prev[i] - vSurf1l) <= tol2D)
                                                || (Math.Abs(v2Prev[i] - vSurf2f) <= tol2D)
                                                || (Math.Abs(v2Prev[i] - vSurf2l) <= tol2D));

                        v1Prev[i] = aV1[i];
                        v2Prev[i] = aV2[i];

                        if ((wlFindStatus[i] == WLFStatus.WLFStatus_Exist) && (isFound1 || isFound2) && !isPrevVBound)
                        {
                            wlFindStatus[i] = WLFStatus.WLFStatus_Broken; // start a new line
                        }
                        else if (isInscribe)
                        {
                            if ((wlFindStatus[i] == WLFStatus.WLFStatus_Absent) && (isFound1 || isFound2))
                            {
                                wlFindStatus[i] = WLFStatus.WLFStatus_Exist;
                            }

                            if ((wlFindStatus[i] != WLFStatus.WLFStatus_Broken) || (wLine[i].NbPnts() >= 1)
                                || IsEqual(anU1, ul))
                            {
                                if (wLine[i].NbPnts() > 0)
                                {
                                    double u2p, v2p;
                                    if (isReversed)
                                    {
                                        wLine[i].Point(wLine[i].NbPnts()).ParametersOnS1(out u2p, out v2p);
                                    }
                                    else
                                    {
                                        wLine[i].Point(wLine[i].NbPnts()).ParametersOnS2(out u2p, out v2p);
                                    }

                                    double delta = aU2[i] - u2p;

                                    if (2.0 * Math.Abs(delta) > period)
                                    {
                                        if (delta > 0.0)
                                        {
                                            aU2[i] -= period;
                                        }
                                        else
                                        {
                                            aU2[i] += period;
                                        }
                                    }
                                }

                                if (AddPointIntoWL(quad1,
                                                   quad2,
                                                   equationCoeffs,
                                                   isReversed,
                                                   true,
                                                   new Point2D(anU1, aV1[i]),
                                                   new Point2D(aU2[i], aV2[i]),
                                                   uSurf1f,
                                                   uSurf1l,
                                                   uSurf2f,
                                                   uSurf2l,
                                                   vSurf1f,
                                                   vSurf1l,
                                                   vSurf2f,
                                                   vSurf2l,
                                                   period,
                                                   wLine[i].Curve(),
                                                   i,
                                                   tol3D,
                                                   tol2D,
                                                   isForce))
                                {
                                    if (wlFindStatus[i] == WLFStatus.WLFStatus_Absent)
                                    {
                                        wlFindStatus[i] = WLFStatus.WLFStatus_Exist;
                                    }
                                }
                                else if (!isFound1 && !isFound2)
                                { // We do not add any point while doing this iteration
                                    if (wlFindStatus[i] == WLFStatus.WLFStatus_Exist)
                                    {
                                        wlFindStatus[i] = WLFStatus.WLFStatus_Broken;
                                    }
                                }
                            }
                        }
                        else
                        { // We do not add any point while doing this iteration
                            if (wlFindStatus[i] == WLFStatus.WLFStatus_Exist)
                            {
                                wlFindStatus[i] = WLFStatus.WLFStatus_Broken;
                            }
                        }

                        if (wlFindStatus[i] == WLFStatus.WLFStatus_Broken)
                        {
                            isBroken = true;
                        }
                    } // for(int i = 0; i < aNbWLines; i++)

                    if (isBroken)
                    { // current lines are filled. Go to the next lines
                        uf = anU1;

                        bool isAdded = true;

                        for (int i = 0; i < nbWLines; i++)
                        {
                            if (isAddingWLEnabled[i])
                            {
                                continue;
                            }

                            isAdded = false;

                            bool isFound1 = false, isFound2 = false;

                            bw.AddBoundaryPoint(wLine[i],
                                                   anU1,
                                                   u1Prev,
                                                   minCriticalParam,
                                                   aU2[i],
                                                   aV1[i],
                                                   v1Prev[i],
                                                   aV2[i],
                                                   v2Prev[i],
                                                   i,
                                                   false,
                                                   ref isFound1,
                                                   ref isFound2);

                            if (isFound1 || isFound2)
                            {
                                isAdded = true;
                            }

                            if (wLine[i].NbPnts() > 0)
                            {
                                double u2p, v2p;
                                if (isReversed)
                                {
                                    wLine[i].Point(wLine[i].NbPnts()).ParametersOnS1(out u2p, out v2p);
                                }
                                else
                                {
                                    wLine[i].Point(wLine[i].NbPnts()).ParametersOnS2(out u2p, out v2p);
                                }

                                double delta = aU2[i] - u2p;

                                if (2 * Math.Abs(delta) > period)
                                {
                                    if (delta > 0.0)
                                    {
                                        aU2[i] -= period;
                                    }
                                    else
                                    {
                                        aU2[i] += period;
                                    }
                                }
                            }

                            if (AddPointIntoWL(quad1,
                                               quad2,
                                               equationCoeffs,
                                               isReversed,
                                               true,
                                               new Point2D(anU1, aV1[i]),
                                               new Point2D(aU2[i], aV2[i]),
                                               uSurf1f,
                                               uSurf1l,
                                               uSurf2f,
                                               uSurf2l,
                                               vSurf1f,
                                               vSurf1l,
                                               vSurf2f,
                                               vSurf2l,
                                               period,
                                               wLine[i].Curve(),
                                               i,
                                               tol3D,
                                               tol2D,
                                               false))
                            {
                                isAdded = true;
                            }
                        }

                        if (!isAdded)
                        {
                            // Before breaking WL, we must complete it correctly
                            //(e.g. to prolong to the surface boundary).
                            // Therefore, we take the point last added in some WL
                            //(have maximal U1-parameter) and try to add it in
                            // the current WL.
                            double umaxAdded = RealFirst();

                            {
                                bool isChanged = false;
                                for (int i = 0; i < nbWLines; i++)
                                {
                                    if ((wlFindStatus[i] == WLFStatus.WLFStatus_Absent) || (wLine[i].NbPnts() == 0))
                                    {
                                        continue;
                                    }

                                    double u1c, v1c;
                                    if (isReversed)
                                    {
                                        wLine[i].Curve().Value(wLine[i].NbPnts()).ParametersOnS2(out u1c, out v1c);
                                    }
                                    else
                                    {
                                        wLine[i].Curve().Value(wLine[i].NbPnts()).ParametersOnS1(out u1c, out v1c);
                                    }

                                    umaxAdded = Max(umaxAdded, u1c);
                                    isChanged = true;
                                }

                                if (!isChanged)
                                { // If anUmaxAdded were not changed in previous cycle then
                                    // we would break existing WLines.
                                    break;
                                }
                            }

                            for (int i = 0; i < nbWLines; i++)
                            {
                                if (isAddingWLEnabled[i])
                                {
                                    continue;
                                }

                                ComputationMethods.CylCylComputeParameters(umaxAdded,
                                                                           i,
                                                                           equationCoeffs,
                                                                           ref aU2[i],
                                                                           ref aV1[i],
                                                                           ref aV2[i]);

                                AddPointIntoWL(quad1,
                                               quad2,
                                               equationCoeffs,
                                               isReversed,
                                               true,
                                               new Point2D(umaxAdded, aV1[i]),
                                               new Point2D(aU2[i], aV2[i]),
                                               uSurf1f,
                                               uSurf1l,
                                               uSurf2f,
                                               uSurf2l,
                                               vSurf1f,
                                               vSurf1l,
                                               vSurf2f,
                                               vSurf2l,
                                               period,
                                               wLine[i].Curve(),
                                               i,
                                               tol3D,
                                               tol2D,
                                               false);
                            }
                        }

                        break;
                    }

                    // Step computing

                    {
                        // Step of aU1-parameter is computed adaptively. The algorithm
                        // aims to provide given aDeltaV1 and aDeltaV2 values (if it is
                        // possible because the intersection line can go along V-isoline)
                        // in every iteration. It allows avoiding "flying" intersection
                        // points too far each from other (see issue #24915).

                        double deltaV1 = rangeS1.Delta() / IntToReal(nbPoints),
                            deltaV2 = rangeS2.Delta() / IntToReal(nbPoints);

                        Matrix matr = new(1, 3, 1, 5);

                        double minUexp = RealLast();

                        for (int i = 0; i < nbWLines; i++)
                        {
                            if (tol2D < (uexpect[i] - anU1))
                            {
                                continue;
                            }

                            if (wlFindStatus[i] == WLFStatus.WLFStatus_Absent)
                            {
                                uexpect[i] += stepMax;
                                minUexp = Min(minUexp, uexpect[i]);
                                continue;
                            }
                            //
                            if (isGoodIntersection)
                            {
                                // Use constant step
                                uexpect[i] += stepMax;
                                minUexp = Min(minUexp, uexpect[i]);

                                continue;
                            }
                            //

                            double stepTmp = stepMax;

                            double sinU1 = Math.Sin(anU1), cosU1 = Math.Cos(anU1), sinU2 = Math.Sin(aU2[i]),
                                cosU2 = Math.Cos(aU2[i]);

                            matr.SetCol(1, equationCoeffs._mVecC1);
                            matr.SetCol(2, equationCoeffs._mVecC2);
                            matr.SetCol(3, equationCoeffs._mVecA1 * sinU1 - equationCoeffs._mVecB1 * cosU1);
                            matr.SetCol(4, equationCoeffs._mVecA2 * sinU2 - equationCoeffs._mVecB2 * cosU2);
                            matr.SetCol(5,
                                         equationCoeffs._mVecA1 * cosU1 + equationCoeffs._mVecB1 * sinU1
                                           + equationCoeffs._mVecA2 * cosU2 + equationCoeffs._mVecB2 * sinU2
                                           + equationCoeffs._mVecD);

                            // The main idea is in solving of linearized system (2)
                            //(see description to ComputationMethods class) in order to find new U1-value
                            // to provide new value V1 or V2, which differs from current one by aDeltaV1 or
                            // aDeltaV2 respectively.

                            // While linearizing, following Taylor formulas are used:
                            //     cos(x0+dx) = cos(x0) - sin(x0)*dx
                            //     sin(x0+dx) = sin(x0) + cos(x0)*dx

                            // Consequently cos(U1), cos(U2), sin(U1) and sin(U2) in the system (2)
                            // must be substituted by corresponding values.

                            // ATTENTION!!!
                            // The solution is approximate. More over, all requirements to the
                            // linearization must be satisfied in order to obtain quality result.

                            if (!StepComputing(matr, aV1[i], aV2[i], deltaV1, deltaV2, ref stepTmp))
                            {
                                // To avoid cycling-up
                                uexpect[i] += stepMax;
                                minUexp = Min(minUexp, uexpect[i]);

                                continue;
                            }

                            if (stepTmp < stepMin)
                            {
                                stepTmp = stepMin;
                            }

                            if (stepTmp > stepMax)
                            {
                                stepTmp = stepMax;
                            }

                            uexpect[i] = anU1 + stepTmp;
                            minUexp = Min(minUexp, uexpect[i]);
                        }

                        u1Prev = anU1;
                        anU1 = minUexp;
                    }

                    if (global::Kerf.Engine.Foundation.Precision.PConfusion() >= (ul - anU1))
                    {
                        anU1 = ul;
                    }

                    uf = anU1;

                    for (int i = 0; i < nbWLines; i++)
                    {
                        if (wLine[i].NbPnts() != 1)
                        {
                            isAddedIntoWL[i] = false;
                        }

                        if (anU1 == ul)
                        { // strictly equal. Tolerance is considered above.
                            uexpect[i] = ul;
                        }
                    }
                }

                for (int i = 0; i < nbWLines; i++)
                {
                    if ((wLine[i].NbPnts() == 1) && (!isAddedIntoWL[i]))
                    {
                        isTheEmpty = false;
                        wLine[i].Point(1).Parameters(out double u1, out double v1, out double u2, out double v2);
                        PatchPoint p = new();
                        p.SetParameter(u1);
                        p.SetParameters(u1, v1, u2, v2);
                        p.SetTolerance(tol3D);
                        p.SetValue(wLine[i].Point(1).Value());

                        // Check whether the added point exists.
                        // It is enough to check the last point.
                        if (sPnt.IsEmpty()
                            || !sPnt.Last().PntOn2S().IsSame(p.PntOn2S(), global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            sPnt.Append(p);
                        }
                    }
                    else if (wLine[i].NbPnts() > 1)
                    {
                        bool isGood = true;

                        if (wLine[i].NbPnts() == 2)
                        {
                            PntOn2S pf = wLine[i].Point(1);
                            PntOn2S pl = wLine[i].Point(2);

                            if (pf.IsSame(pl, global::Kerf.Engine.Foundation.Precision.Confusion()))
                            {
                                isGood = false;
                            }
                        }
                        else if (wLine[i].NbPnts() > 2)
                        {
                            // Sometimes points of the WLine are distributed
                            // linearly and uniformly. However, such position
                            // of the points does not always describe the real intersection
                            // curve. I.e. real tangents at the ends of the intersection
                            // curve can significantly deviate from this "line" direction.
                            // Here we are processing this case by inserting additional points
                            // to the beginning/end of the WLine to make it more precise.
                            // See description to the issue #30082.

                            double sqTol3D = tol3D * tol3D;
                            for (int j = 0; j < 2; j++)
                            {
                                // If j == 0 ==> add point at begin of WLine.
                                // If j == 1 ==> add point at end of WLine.

                                for (;;)
                                {
                                    if (wLine[i].NbPnts() >= nbMaxPoints)
                                    {
                                        break;
                                    }

                                    // Take 1st and 2nd point to compute the "line" direction.
                                    // For our convenience, we make 2nd point be the ends of the WLine
                                    // because it will be used for computation of the normals
                                    // to the surfaces.
                                    int idx1 = j != 0 ? wLine[i].NbPnts() - 1 : 2;
                                    int idx2 = j != 0 ? wLine[i].NbPnts() : 1;

                                    Point3D p1 = wLine[i].Point(idx1).Value();
                                    Point3D p2 = wLine[i].Point(idx2).Value();

                                    Vector3D dir = new(p1, p2);

                                    if (dir.SquareMagnitude < sqTol3D)
                                    {
                                        break;
                                    }

                                    // Compute tangent in first/last point of the WLine.
                                    // We do not take into account the flag "isReversed"
                                    // because strict direction of the tangent is not
                                    // important here (we are interested in the tangent
                                    // line itself and nothing to fear if its direction
                                    // is reversed).
                                    Vector3D n1 = quad1.Normale(p2);
                                    Vector3D n2 = quad2.Normale(p2);
                                    Vector3D tg = n1.Crossed(n2);

                                    if (tg.SquareMagnitude < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                                    {
                                        // Tangent zone
                                        break;
                                    }

                                    // Check of the bending
                                    double angle = dir.AngleTo(tg);

                                    if (angle > M_PI_2)
                                    {
                                        angle -= M_PI;
                                    }

                                    if (Math.Abs(angle) > 0.25) // ~ 14deg.
                                    {
                                        int nbPntsPrev = wLine[i].NbPnts();
                                        SeekAdditionalPoints(quad1,
                                                             quad2,
                                                             wLine[i].Curve(),
                                                             equationCoeffs,
                                                             i,
                                                             3,
                                                             idx1,
                                                             idx2,
                                                             tol2D,
                                                             period,
                                                             isReversed);

                                        if (wLine[i].NbPnts() == nbPntsPrev)
                                        {
                                            // No points have been added. ==> Exit from a loop.
                                            break;
                                        }
                                    }
                                    else
                                    {
                                        // Good result has been achieved. ==> Exit from a loop.
                                        break;
                                    }
                                } // for (;;)
                            }
                        }

                        if (isGood)
                        {
                            isTheEmpty = false;
                            isAddedIntoWL[i] = true;
                            SeekAdditionalPoints(quad1,
                                                 quad2,
                                                 wLine[i].Curve(),
                                                 equationCoeffs,
                                                 i,
                                                 nbPoints,
                                                 1,
                                                 wLine[i].NbPnts(),
                                                 tol2D,
                                                 period,
                                                 isReversed);

                            wLine[i].ComputeVertexParameters(tol3D);
                            slin.Append(wLine[i]);
                        }
                    }
                    else
                    {
                        isAddedIntoWL[i] = false;
                    }
                }
            }
        }

        // Delete the points in theSPnt, which
        // lie at least in one of the line in theSlin.
        for (int nbPnt = 1; nbPnt <= sPnt.Length(); nbPnt++)
        {
            for (int nbLin = 1; nbLin <= slin.Length(); nbLin++)
            {
                WLine wLine1 = (slin.Value(nbLin) as WLine)!;

                PntOn2S pntFWL1 = wLine1.Point(1);
                PntOn2S pntLWL1 = wLine1.Point(wLine1.NbPnts());

                PntOn2S pntCur = sPnt.Value(nbPnt).PntOn2S();
                if (pntCur.IsSame(pntFWL1, tol3D) || pntCur.IsSame(pntLWL1, tol3D))
                {
                    sPnt.Remove(nbPnt);
                    nbPnt--;
                    break;
                }
            }
        }

        // Try to add new points in the neighborhood of existing point
        for (int nbPnt = 1; nbPnt <= sPnt.Length(); nbPnt++)
        {
            // Standard algorithm (implemented above) could not find any
            // continuous curve in neighborhood of aPnt2S (e.g. because
            // this curve is too small; see tests\bugs\modalg_5\bug25292_35 and _36).
            // Here, we will try to find several new points nearer to aPnt2S.

            // The algorithm below tries to find two points in every
            // intervals [u1 - aStepMax, u1] and [u1, u1 + aStepMax]
            // and every new point will be in maximal distance from
            // u1.0 If these two points exist they will be joined
            // by the intersection curve.

            PatchPoint pnt2S = sPnt.Value(nbPnt);

            pnt2S.Parameters(out double u1, out double v1, out double u2, out double v2);

            LineOn2S l2Sloc = new();
            WLine wLineLoc = new(l2Sloc, false);
            wLineLoc.SetCreatingWayInfo(WLine.IntPatch_WLType.WLImpImp);

            // Define the index of WLine, which lies the point aPnt2S in.
            int index = 0;

            double uf = 0.0, ul = 0.0, curU2 = 0.0;
            if (isReversed)
            {
                uf = Max(u2 - stepMax, uSurf1f);
                ul = Min(u2 + stepMax, uSurf1l);
                curU2 = u1;
            }
            else
            {
                uf = Max(u1 - stepMax, uSurf1f);
                ul = Min(u1 + stepMax, uSurf1l);
                curU2 = u2;
            }

            double uinf = uf, usup = ul, umid = 0.5 * (uf + ul);

            {
                // Find the value of anIndex variable.
                double delta = RealLast();
                for (int i = 0; i < nbWLines; i++)
                {
                    double u2t = 0.0;
                    if (!ComputationMethods.CylCylComputeParameters(umid, i, equationCoeffs, ref u2t))
                    {
                        continue;
                    }

                    double du2 = Math.Abs(u2t - curU2) % period; // fmod
                    du2 = Min(du2, Math.Abs(du2 - period));
                    if (du2 < delta)
                    {
                        delta = du2;
                        index = i;
                    }
                }
            }

            // Bisection method is used in order to find every new point.
            // I.e. if we need to find intersection point in the interval [anUinf, anUmid]
            // we check the point anUC = 0.5*(anUinf+anUmid). If it is an intersection point
            // we try to find another point in the interval [anUinf, anUC] (because we find the point in
            // maximal distance from anUmid). If it is not then we try to find another point in the
            // interval [anUC, anUmid]. Next iterations will be made analogically.
            // When we find intersection point in the interval [anUmid, anUsup] we try to find
            // another point in the interval [anUC, anUsup] if anUC is intersection point and
            // in the interval [anUmid, anUC], otherwise.

            Span<double> addedPar = [isReversed ? u2 : u1, isReversed ? u2 : u1];

            for (int parID = 0; parID < 2; parID++)
            {
                if (parID == 0)
                {
                    uf = uinf;
                    ul = umid;
                }
                else // if(aParID == 1)
                {
                    uf = umid;
                    ul = usup;
                }

                ref double par1 = ref (parID == 0 ? ref uf : ref ul);
                ref double par2 = ref (parID == 0 ? ref ul : ref uf);

                while (Math.Abs(par2 - par1) > stepMin)
                {
                    double uc = 0.5 * (uf + ul);
                    double u2loc = 0.0, v1loc = 0.0, v2loc = 0.0;
                    bool isDone = ComputationMethods.CylCylComputeParameters(uc,
                                                                             index,
                                                                             equationCoeffs,
                                                                             ref u2loc,
                                                                             ref v1loc,
                                                                             ref v2loc);

                    if (isDone)
                    {
                        if (Math.Abs(v1loc - vSurf1f) <= tol2D)
                        {
                            v1loc = vSurf1f;
                        }

                        if (Math.Abs(v1loc - vSurf1l) <= tol2D)
                        {
                            v1loc = vSurf1l;
                        }

                        if (Math.Abs(v2loc - vSurf2f) <= tol2D)
                        {
                            v2loc = vSurf2f;
                        }

                        if (Math.Abs(v2loc - vSurf2l) <= tol2D)
                        {
                            v2loc = vSurf2l;
                        }

                        isDone = AddPointIntoWL(quad1,
                                                quad2,
                                                equationCoeffs,
                                                isReversed,
                                                true,
                                                new Point2D(uc, v1loc),
                                                new Point2D(u2loc, v2loc),
                                                uSurf1f,
                                                uSurf1l,
                                                uSurf2f,
                                                uSurf2l,
                                                vSurf1f,
                                                vSurf1l,
                                                vSurf2f,
                                                vSurf2l,
                                                period,
                                                wLineLoc.Curve(),
                                                index,
                                                tol3D,
                                                tol2D,
                                                false,
                                                true);
                    }

                    if (isDone)
                    {
                        addedPar[0] = Min(addedPar[0], uc);
                        addedPar[1] = Max(addedPar[1], uc);
                        par2 = uc;
                    }
                    else
                    {
                        par1 = uc;
                    }
                }
            }

            // Fill aWLine by additional points
            if (addedPar[1] - addedPar[0] > stepMin)
            {
                for (int parID = 0; parID < 2; parID++)
                {
                    double u2loc = 0.0, v1loc = 0.0, v2loc = 0.0;
                    ComputationMethods.CylCylComputeParameters(addedPar[parID],
                                                               index,
                                                               equationCoeffs,
                                                               ref u2loc,
                                                               ref v1loc,
                                                               ref v2loc);

                    AddPointIntoWL(quad1,
                                   quad2,
                                   equationCoeffs,
                                   isReversed,
                                   true,
                                   new Point2D(addedPar[parID], v1loc),
                                   new Point2D(u2loc, v2loc),
                                   uSurf1f,
                                   uSurf1l,
                                   uSurf2f,
                                   uSurf2l,
                                   vSurf1f,
                                   vSurf1l,
                                   vSurf2f,
                                   vSurf2l,
                                   period,
                                   wLineLoc.Curve(),
                                   index,
                                   tol3D,
                                   tol2D,
                                   false,
                                   false);
                }

                SeekAdditionalPoints(quad1,
                                     quad2,
                                     wLineLoc.Curve(),
                                     equationCoeffs,
                                     index,
                                     nbMinPoints,
                                     1,
                                     wLineLoc.NbPnts(),
                                     tol2D,
                                     period,
                                     isReversed);

                wLineLoc.ComputeVertexParameters(tol3D);
                slin.Append(wLineLoc);

                sPnt.Remove(nbPnt);
                nbPnt--;
            }
        }

        return IntStatus.IntStatus_OK;
    }

    //=================================================================================================

    private static IntStatus IntCyCy(in SurfaceQuadric quad1,
                                     in SurfaceQuadric quad2,
                                     double tol3D,
                                     double tol2D,
                                     in Box2d uvSurf1,
                                     in Box2d uvSurf2,
                                     ref bool isTheEmpty,
                                     ref bool isTheSameSurface,
                                     ref bool isTheMultiplePoint,
                                     NCollection_Sequence<PatchLine> slin,
                                     NCollection_Sequence<PatchPoint> sPnt)
    {
        isTheEmpty = true;
        isTheSameSurface = false;
        isTheMultiplePoint = false;
        slin.Clear();
        sPnt.Clear();

        Cylinder3D cyl1 = quad1.Cylinder(), cyl2 = quad2.Cylinder();

        QuadQuadGeo inter = new(cyl1, cyl2, tol3D);

        if (!inter.IsDone)
        {
            return IntStatus.IntStatus_Fail;
        }

        if (inter.TypeInter() != NoGeometricSolution)
        {
            if (CyCyAnalyticalIntersect(quad1,
                                        quad2,
                                        inter,
                                        tol3D,
                                        ref isTheEmpty,
                                        ref isTheSameSurface,
                                        ref isTheMultiplePoint,
                                        slin,
                                        sPnt))
            {
                return IntStatus.IntStatus_OK;
            }

            // Analytical handler declined: discard anything it may have appended
            // or flipped so the numerical path starts from a clean output state.
            isTheEmpty = true;
            isTheSameSurface = false;
            isTheMultiplePoint = false;
            slin.Clear();
            sPnt.Clear();
        }

        // Here, intersection line is not an analytical curve(line, circle, ellipsis etc.)

        double[,] usBou = new double[2, 2], vsBou = new double[2, 2]; // const

        uvSurf1.Get(out usBou[0, 0], out vsBou[0, 0], out usBou[0, 1], out vsBou[0, 1]);
        uvSurf2.Get(out usBou[1, 0], out vsBou[1, 0], out usBou[1, 1], out vsBou[1, 1]);

        const double period = 2.0 * M_PI;
        const int nbWLines = 2;

        ComputationMethods.StCoeffsValue equationCoeffs1 = new(cyl1, cyl2);
        ComputationMethods.StCoeffsValue equationCoeffs2 = new(cyl2, cyl1);

        // Boundaries.
        // Intersection result can include two non-connected regions
        //(see WorkWithBoundaries::BoundariesComputing(...) method).
        const int nbOfBoundaries = 2;
        BoundRange[][] uRange = [[new(), new()], [new(), new()]]; // const

        if (!WorkWithBoundaries.BoundariesComputing(equationCoeffs1, period, uRange[0]))
        {
            return IntStatus.IntStatus_OK;
        }

        if (!WorkWithBoundaries.BoundariesComputing(equationCoeffs2, period, uRange[1]))
        {
            return IntStatus.IntStatus_OK;
        }

        // anURange[*] can be in different periodic regions in
        // compare with First-Last surface. E.g. the surface
        // is full cylinder [0, 2*PI] but anURange is [5, 7].
        // Trivial common range computation returns [5, 2*PI] and
        // its summary length is 2*PI-5 == 1.28... only. That is wrong.
        // This problem can be solved by the following
        // algorithm:
        //  1.0 split anURange[*] by the surface boundary;
        //  2.0 shift every new range in order to inscribe it
        //       in [Ufirst, Ulast] of cylinder;
        //  3.0 consider only common ranges between [Ufirst, Ulast]
        //     and new ranges.
        //
        //  In above example, we obtain following:
        //  1.0 two ranges: [5, 2*PI] and [2*PI, 7];
        //  2.0 after shifting: [5, 2*PI] and [0, 7-2*PI];
        //  3.0 Common ranges: ([5, 2*PI] and [0, 2*PI]) == [5, 2*PI],
        //                    ([0, 7-2*PI] and [0, 2*PI]) == [0, 7-2*PI];
        //  4.0 Their summary length is (2*PI-5)+(7-2*PI-0)==7-5==2 ==> GOOD.

        Span<double> sumRange = [0.0, 0.0];
        for (int cid = 0; cid < 2; cid++)
        {
            NCollection_List<BoundRange> listOfRng = new();

            listOfRng.Append(uRange[cid][0]);
            listOfRng.Append(uRange[cid][1]);

            ReadOnlySpan<double> splitArr = [usBou[cid, 0], usBou[cid, 1], 0.0];

            for (int sInd = 0; sInd < 3; sInd++)
            {
                NCollection_List<BoundRange> lstTemp = new(listOfRng);
                listOfRng.Clear();
                foreach (BoundRange rng in lstTemp)
                {
                    rng.Split(splitArr[sInd], listOfRng, period);
                }
            }

            foreach (BoundRange item in listOfRng)
            {
                // OCCT changes the list's element in place; the list is not read afterwards, so a copy is the same.
                BoundRange currRange = item;

                BoundRange boundR = new();
                boundR.Add(usBou[cid, 0]);
                boundR.Add(usBou[cid, 1]);

                if (!InscribeInterval(usBou[cid, 0], usBou[cid, 1], ref currRange, tol2D, period))
                {
                    // If aCurrRange does not have common block with
                    //[Ufirst, Ulast] of cylinder then we will try
                    // to inscribe [Ufirst, Ulast] in the boundaries of aCurrRange.
                    double f = 1.0, l = 0.0;
                    if (!currRange.GetBounds(ref f, ref l))
                    {
                        continue;
                    }

                    if ((l < usBou[cid, 0]))
                    {
                        currRange.Shift(period);
                    }
                    else if (f > usBou[cid, 1])
                    {
                        currRange.Shift(-period);
                    }
                }

                boundR.Common(currRange);

                double delta = boundR.Delta();

                if (delta > 0.0)
                {
                    sumRange[cid] += delta;
                }
            }
        }

        // The bigger range the bigger number of points in Walking-line (WLine)
        // we will be able to add and consequently we will obtain more
        // precise intersection line.
        // Every point of WLine is determined as function from U1-parameter,
        // where U1 is U-parameter on 1st quadric.
        // Therefore, we should use quadric with bigger range as 1st parameter
        // in IntCyCy() function.
        // On the other hand, there is no point in reversing in case of
        // analytical intersection (when result is line, ellipse, point...).
        // This result is independent of the arguments order.
        bool isToReverse = (sumRange[1] > sumRange[0]);

        if (isToReverse)
        {
            WorkWithBoundaries boundWork = new(quad2,
                                                quad1,
                                                equationCoeffs2,
                                                uvSurf2,
                                                uvSurf1,
                                                nbWLines,
                                                period,
                                                tol3D,
                                                tol2D,
                                                true);

            return CyCyNoGeometric(cyl2,
                                   cyl1,
                                   boundWork,
                                   uRange[1],
                                   nbOfBoundaries,
                                   ref isTheEmpty,
                                   slin,
                                   sPnt);
        }
        else
        {
            WorkWithBoundaries boundWork = new(quad1,
                                                quad2,
                                                equationCoeffs1,
                                                uvSurf1,
                                                uvSurf2,
                                                nbWLines,
                                                period,
                                                tol3D,
                                                tol2D,
                                                false);

            return CyCyNoGeometric(cyl1,
                                   cyl2,
                                   boundWork,
                                   uRange[0],
                                   nbOfBoundaries,
                                   ref isTheEmpty,
                                   slin,
                                   sPnt);
        }
    }
}
