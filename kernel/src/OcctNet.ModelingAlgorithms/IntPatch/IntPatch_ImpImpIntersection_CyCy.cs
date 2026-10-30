// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_ImpImpIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntAna_ResultType;
using static Occt.IntSurf_Situation;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

// The cylinder/cylinder intersection (OCCT 7.x's IntPatch_ImpImpIntersection_4.gxx): the analytical cases and the
// walking along U1 of the non-analytical one.
public sealed partial class IntPatch_ImpImpIntersection
{
    // If std::abs(a) <= aNulValue then it is considered that a = 0.
    private const double aNulValue = 1.0e-11;

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
        internal sealed class stCoeffsValue
        {
            internal readonly math_Vector mVecA1;
            internal readonly math_Vector mVecA2;
            internal readonly math_Vector mVecB1;
            internal readonly math_Vector mVecB2;
            internal readonly math_Vector mVecC1;
            internal readonly math_Vector mVecC2;
            internal readonly math_Vector mVecD;

            internal double mK21; // sinU2
            internal double mK11; // sinU1
            internal double mL21; // cosU2
            internal double mL11; // cosU1
            internal double mM1;  // Free member

            internal double mK22; // sinU2
            internal double mK12; // sinU1
            internal double mL22; // cosU2
            internal double mL12; // cosU1
            internal double mM2;  // Free member

            internal double mK1;
            internal double mL1;
            internal double mK2;
            internal double mL2;

            internal double mFIV1;
            internal double mPSIV1;
            internal double mFIV2;
            internal double mPSIV2;

            internal double mB;
            internal double mC;
            internal double mFI1;
            internal double mFI2;

            private enum CoupleOfEquation
            {
                COENONE = 0,
                COE12 = 1,
                COE23 = 2,
                COE13 = 3,
            }

            internal stCoeffsValue(in gp_Cylinder theCyl1, in gp_Cylinder theCyl2)
            {
                mVecA1 = new math_Vector(-theCyl1.Radius() * theCyl1.XAxis().Direction().XYZ());
                mVecA2 = new math_Vector(theCyl2.Radius() * theCyl2.XAxis().Direction().XYZ());
                mVecB1 = new math_Vector(-theCyl1.Radius() * theCyl1.YAxis().Direction().XYZ());
                mVecB2 = new math_Vector(theCyl2.Radius() * theCyl2.YAxis().Direction().XYZ());
                mVecC1 = new math_Vector(theCyl1.Axis().Direction().XYZ());
                mVecC2 = new math_Vector(theCyl2.Axis().Direction().XYZ().Reversed());
                mVecD = new math_Vector(theCyl2.Location().XYZ() - theCyl1.Location().XYZ());

                CoupleOfEquation aFoundCouple = CoupleOfEquation.COENONE;

                double aDetV1V2 = 0.0;

                double aDelta1 = mVecC1[1] * mVecC2[2] - mVecC1[2] * mVecC2[1]; // 1-2
                double aDelta2 = mVecC1[2] * mVecC2[3] - mVecC1[3] * mVecC2[2]; // 2-3
                double aDelta3 = mVecC1[1] * mVecC2[3] - mVecC1[3] * mVecC2[1]; // 1-3
                double anAbsD1 = Math.Abs(aDelta1);                             // 1-2
                double anAbsD2 = Math.Abs(aDelta2);                             // 2-3
                double anAbsD3 = Math.Abs(aDelta3);                             // 1-3

                if (anAbsD1 >= anAbsD2)
                {
                    if (anAbsD3 > anAbsD1)
                    {
                        aFoundCouple = CoupleOfEquation.COE13;
                        aDetV1V2 = aDelta3;
                    }
                    else
                    {
                        aFoundCouple = CoupleOfEquation.COE12;
                        aDetV1V2 = aDelta1;
                    }
                }
                else
                {
                    if (anAbsD3 > anAbsD2)
                    {
                        aFoundCouple = CoupleOfEquation.COE13;
                        aDetV1V2 = aDelta3;
                    }
                    else
                    {
                        aFoundCouple = CoupleOfEquation.COE23;
                        aDetV1V2 = aDelta2;
                    }
                }

                // In point of fact, every determinant (aDelta1, aDelta2 and aDelta3) is
                // cross-product between directions (i.e. sine of angle).
                // If sine is too small then sine is (approx.) equal to angle itself.
                // Therefore, in this case we should compare sine with angular tolerance.
                // This constant is used for check if axes are parallel (see constructor
                // AxeOperator::AxeOperator(...) in IntAna_QuadQuadGeo.cxx file).
                if (Math.Abs(aDetV1V2) < Precision.Angular())
                {
                    throw new Standard_Failure("Error. Exception in divide by zerro (IntCyCyTrim)!!!!");
                }

                switch (aFoundCouple)
                {
                    case CoupleOfEquation.COE12:
                        break;
                    case CoupleOfEquation.COE23:
                    {
                        RotateCoordinates(mVecA1);
                        RotateCoordinates(mVecA2);
                        RotateCoordinates(mVecB1);
                        RotateCoordinates(mVecB2);
                        RotateCoordinates(mVecC1);
                        RotateCoordinates(mVecC2);
                        RotateCoordinates(mVecD);
                        break;
                    }
                    case CoupleOfEquation.COE13:
                    {
                        SwapCoordinates23(mVecA1);
                        SwapCoordinates23(mVecA2);
                        SwapCoordinates23(mVecB1);
                        SwapCoordinates23(mVecB2);
                        SwapCoordinates23(mVecC1);
                        SwapCoordinates23(mVecC2);
                        SwapCoordinates23(mVecD);
                        break;
                    }
                    default:
                        break;
                }

                //------- For V1 (begin)
                // sinU2
                mK21 = (mVecC2[2] * mVecB2[1] - mVecC2[1] * mVecB2[2]) / aDetV1V2;
                // sinU1
                mK11 = (mVecC2[2] * mVecB1[1] - mVecC2[1] * mVecB1[2]) / aDetV1V2;
                // cosU2
                mL21 = (mVecC2[2] * mVecA2[1] - mVecC2[1] * mVecA2[2]) / aDetV1V2;
                // cosU1
                mL11 = (mVecC2[2] * mVecA1[1] - mVecC2[1] * mVecA1[2]) / aDetV1V2;
                // Free member
                mM1 = (mVecC2[2] * mVecD[1] - mVecC2[1] * mVecD[2]) / aDetV1V2;
                //------- For V1 (end)

                //------- For V2 (begin)
                // sinU2
                mK22 = (mVecC1[1] * mVecB2[2] - mVecC1[2] * mVecB2[1]) / aDetV1V2;
                // sinU1
                mK12 = (mVecC1[1] * mVecB1[2] - mVecC1[2] * mVecB1[1]) / aDetV1V2;
                // cosU2
                mL22 = (mVecC1[1] * mVecA2[2] - mVecC1[2] * mVecA2[1]) / aDetV1V2;
                // cosU1
                mL12 = (mVecC1[1] * mVecA1[2] - mVecC1[2] * mVecA1[1]) / aDetV1V2;
                // Free member
                mM2 = (mVecC1[1] * mVecD[2] - mVecC1[2] * mVecD[1]) / aDetV1V2;
                //------- For V1 (end)

                ShortCosForm(mL11, mK11, ref mK1, ref mFIV1);
                ShortCosForm(mL21, mK21, ref mL1, ref mPSIV1);
                ShortCosForm(mL12, mK12, ref mK2, ref mFIV2);
                ShortCosForm(mL22, mK22, ref mL2, ref mPSIV2);

                double aA1 = mVecC1[3] * mK21 + mVecC2[3] * mK22 - mVecB2[3], // sinU2
                    aA2 = mVecC1[3] * mL21 + mVecC2[3] * mL22 - mVecA2[3],    // cosU2
                    aB1 = mVecB1[3] - mVecC1[3] * mK11 - mVecC2[3] * mK12,    // sinU1
                    aB2 = mVecA1[3] - mVecC1[3] * mL11 - mVecC2[3] * mL12;    // cosU1

                mC = mVecD[3] - mVecC1[3] * mM1 - mVecC2[3] * mM2; // Free

                double aA = 0.0;

                ShortCosForm(aB2, aB1, ref mB, ref mFI1);
                ShortCosForm(aA2, aA1, ref aA, ref mFI2);

                mB /= aA;
                mC /= aA;
            }

            // OCCT's COE23 case: (x, y, z) := (y, z, x), through a copy of the vector.
            private static void RotateCoordinates(math_Vector theV)
            {
                math_Vector aVTemp = new(theV);
                theV[1] = aVTemp[2];
                theV[2] = aVTemp[3];
                theV[3] = aVTemp[1];
            }

            // OCCT's COE13 case: (x, y, z) := (x, z, y), through a copy of the vector.
            private static void SwapCoordinates23(math_Vector theV)
            {
                math_Vector aVTemp = new(theV);
                theV[2] = aVTemp[3];
                theV[3] = aVTemp[2];
            }
        }

        //=======================================================================
        // function : CylCylMonotonicity
        // purpose  : Determines, if U2(U1) function is increasing.
        //=======================================================================
        internal static bool CylCylMonotonicity(double theU1par,
                                                int theWLIndex,
                                                stCoeffsValue theCoeffs,
                                                double thePeriod,
                                                ref bool theIsIncreasing)
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

            switch (theWLIndex)
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

            double aU1Temp = theU1par - theCoeffs.mFI1;
            InscribePoint(0, thePeriod, ref aU1Temp, 0.0, thePeriod, false);

            theIsIncreasing = true;

            if (((M_PI - aU1Temp) < RealSmall()) && (aU1Temp < thePeriod))
            {
                theIsIncreasing = false;
            }

            if (theCoeffs.mB < 0.0)
            {
                theIsIncreasing = !theIsIncreasing;
            }

            if (!isPlus)
            {
                theIsIncreasing = !theIsIncreasing;
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
        internal static bool CylCylComputeParameters(double theU1par,
                                                     int theWLIndex,
                                                     stCoeffsValue theCoeffs,
                                                     ref double theU2)
        {
            double aDummy = 0.0;
            return CylCylComputeParameters(theU1par, theWLIndex, theCoeffs, ref theU2, ref aDummy, false);
        }

        internal static bool CylCylComputeParameters(double theU1par,
                                                     int theWLIndex,
                                                     stCoeffsValue theCoeffs,
                                                     ref double theU2,
                                                     ref double theDelta)
        {
            return CylCylComputeParameters(theU1par, theWLIndex, theCoeffs, ref theU2, ref theDelta, true);
        }

        private static bool CylCylComputeParameters(double theU1par,
                                                    int theWLIndex,
                                                    stCoeffsValue theCoeffs,
                                                    ref double theU2,
                                                    ref double theDelta,
                                                    bool theHasDelta)
        {
            // This formula is got from some experience and can be changed.
            double aTol0 = Min(10.0 * Epsilon(1.0) * theCoeffs.mB, aNulValue);
            double aTol = 1.0 - aTol0;

            if (theWLIndex < 0 || theWLIndex > 1)
            {
                return false;
            }

            double aSign = theWLIndex != 0 ? -1.0 : 1.0;

            double anArg = Math.Cos(theU1par - theCoeffs.mFI1);
            anArg = theCoeffs.mB * anArg + theCoeffs.mC;

            if (anArg >= aTol)
            {
                if (theHasDelta)
                {
                    theDelta = 0.0;
                }

                anArg = 1.0;
            }
            else if (anArg <= -aTol)
            {
                if (theHasDelta)
                {
                    theDelta = 0.0;
                }

                anArg = -1.0;
            }
            else if (theHasDelta)
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

                double aDelta = Min(1.0 - anArg, 1.0 + anArg);
                Standard_DivideByZero_Raise_if((aDelta * aDelta < RealSmall()) || (aDelta >= 2.0),
                                               "IntPatch_ImpImpIntersection_4.gxx, CylCylComputeParameters()");
                theDelta = aTol0 / Math.Sqrt(aDelta * (2.0 - aDelta));
            }

            theU2 = Math.Acos(anArg);
            theU2 = theCoeffs.mFI2 + aSign * theU2;

            return true;
        }

        //=======================================================================
        // function : CylCylComputeParameters
        // purpose  : Computes V1 and V2 (V-parameters of the 1st and 2nd cylinder respectively).
        //=======================================================================
        internal static bool CylCylComputeParameters(double theU1,
                                                     double theU2,
                                                     stCoeffsValue theCoeffs,
                                                     ref double theV1,
                                                     ref double theV2)
        {
            theV1 = theCoeffs.mK21 * Math.Sin(theU2) + theCoeffs.mK11 * Math.Sin(theU1) + theCoeffs.mL21 * Math.Cos(theU2)
                    + theCoeffs.mL11 * Math.Cos(theU1) + theCoeffs.mM1;

            theV2 = theCoeffs.mK22 * Math.Sin(theU2) + theCoeffs.mK12 * Math.Sin(theU1) + theCoeffs.mL22 * Math.Cos(theU2)
                    + theCoeffs.mL12 * Math.Cos(theU1) + theCoeffs.mM2;

            return true;
        }

        //=======================================================================
        // function : CylCylComputeParameters
        // purpose  : Computes U2 (U-parameter of the 2nd cylinder),
        //            V1 and V2 (V-parameters of the 1st and 2nd cylinder respectively).
        //=======================================================================
        internal static bool CylCylComputeParameters(double theU1par,
                                                     int theWLIndex,
                                                     stCoeffsValue theCoeffs,
                                                     ref double theU2,
                                                     ref double theV1,
                                                     ref double theV2)
        {
            if (!CylCylComputeParameters(theU1par, theWLIndex, theCoeffs, ref theU2))
            {
                return false;
            }

            if (!CylCylComputeParameters(theU1par, theU2, theCoeffs, ref theV1, ref theV2))
            {
                return false;
            }

            return true;
        }
    }

    // Cyl-cyl-specific helper: constructed only by IntCyCy
    // and relies on cylinder-cylinder coefficient tables (ComputationMethods::
    // stCoeffsValue) for both SearchOnVBounds' Newton system and the analytical
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
                mySurfID = 0;
                myU1 = RealLast();
                myV1 = RealLast();
                myU2 = RealLast();
                myV2 = RealLast();
            }

            // Equal to 0 for 1st surface non-zero for 2nd one.
            internal int mySurfID;

            internal double myU1;
            internal double myV1;
            internal double myU2;
            internal double myV2;
        }

        private readonly IntSurf_Quadric myQuad1;
        private readonly IntSurf_Quadric myQuad2;
        private readonly ComputationMethods.stCoeffsValue myCoeffs;
        private readonly Bnd_Box2d myUVSurf1;
        private readonly Bnd_Box2d myUVSurf2;
        private readonly int myNbWLines;
        private readonly double myPeriod;
        private readonly double myTol3D;
        private readonly double myTol2D;
        private readonly bool myIsReverse;

        internal WorkWithBoundaries(in IntSurf_Quadric theQuad1,
                                    in IntSurf_Quadric theQuad2,
                                    ComputationMethods.stCoeffsValue theCoeffs,
                                    in Bnd_Box2d theUVSurf1,
                                    in Bnd_Box2d theUVSurf2,
                                    int theNbWLines,
                                    double thePeriod,
                                    double theTol3D,
                                    double theTol2D,
                                    bool isTheReverse)
        {
            myQuad1 = theQuad1;
            myQuad2 = theQuad2;
            myCoeffs = theCoeffs;
            myUVSurf1 = theUVSurf1;
            myUVSurf2 = theUVSurf2;
            myNbWLines = theNbWLines;
            myPeriod = thePeriod;
            myTol3D = theTol3D;
            myTol2D = theTol2D;
            myIsReverse = isTheReverse;
            _ = myNbWLines; // OCCT stores it and never reads it
        }

        // Returns parameters of system solved while finding
        // intersection line
        internal ComputationMethods.stCoeffsValue SICoeffs() => myCoeffs;

        // Returns quadric correspond to the index theIdx.
        internal IntSurf_Quadric GetQSurface(int theIdx)
        {
            if (theIdx <= 1)
            {
                return myQuad1;
            }

            return myQuad2;
        }

        // Returns TRUE in case of reverting surfaces
        internal bool IsReversed() => myIsReverse;

        // Returns 2D-tolerance
        internal double Get2dTolerance() => myTol2D;

        // Returns 3D-tolerance
        internal double Get3dTolerance() => myTol3D;

        // Returns UV-bounds of 1st surface
        internal Bnd_Box2d UVS1() => myUVSurf1;

        // Returns UV-bounds of 2nd surface
        internal Bnd_Box2d UVS2() => myUVSurf2;

        // V(U1) - V_bound on one arccos branch of the analytical intersection curve (OCCT's local struct
        // VBoundDelta in AddBoundaryPoint).
        private readonly struct VBoundDelta : MathUtils.IScalarFunction
        {
            private readonly ComputationMethods.stCoeffsValue Coeffs;
            private readonly int WLIndex;
            private readonly bool IsV1;
            private readonly double VBound;

            internal VBoundDelta(ComputationMethods.stCoeffsValue theCoeffs, int theWLIndex, bool theIsV1, double theVBound)
            {
                Coeffs = theCoeffs;
                WLIndex = theWLIndex;
                IsV1 = theIsV1;
                VBound = theVBound;
            }

            public bool Value(double theX, out double theF)
            {
                theF = 0.0; // OCCT leaves theF unwritten on failure
                double aU2 = 0.0, aV1 = 0.0, aV2 = 0.0;
                if (!ComputationMethods.CylCylComputeParameters(theX, WLIndex, Coeffs, ref aU2, ref aV1, ref aV2))
                {
                    return false;
                }
                theF = (IsV1 ? aV1 : aV2) - VBound;
                return true;
            }
        }

        // OCCT's lambda findVBoundCrossing in AddBoundaryPoint.
        private bool findVBoundCrossing(int theWLIndex,
                                        bool theIsV1,
                                        double theVBound,
                                        double theULo,
                                        double theUHi,
                                        ref double theU1Star)
        {
            if (!(theULo < theUHi))
            {
                return false;
            }
            VBoundDelta aFunc = new(myCoeffs, theWLIndex, theIsV1, theVBound);
            if (!aFunc.Value(theULo, out double aFlo) || !aFunc.Value(theUHi, out double aFhi))
            {
                return false;
            }
            if (aFlo == 0.0)
            {
                theU1Star = theULo;
                return true;
            }
            if (aFhi == 0.0)
            {
                theU1Star = theUHi;
                return true;
            }
            if (aFlo * aFhi > 0.0)
            {
                return false; // no bracketed crossing
            }

            MathUtils.Config aCfg = new();
            aCfg.XTolerance = Precision.PConfusion();
            MathUtils.ScalarResult aRes = MathRoot.Brent(aFunc, theULo, theUHi, aCfg);
            if (aRes.Status != MathUtils.Status.OK || !aRes.Root.HasValue)
            {
                return false;
            }
            theU1Star = aRes.Root.Value;
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
        internal void AddBoundaryPoint(IntPatch_WLine theWL,
                                       double theU1,
                                       double theU1Prev,
                                       double theU1Min,
                                       double theU2,
                                       double theV1,
                                       double theV1Prev,
                                       double theV2,
                                       double theV2Prev,
                                       int theWLIndex,
                                       bool theFlForce,
                                       ref bool isTheFound1,
                                       ref bool isTheFound2)
        {
            myUVSurf1.Get(out double aUSurf1f, out double aVSurf1f, out double aUSurf1l, out double aVSurf1l);
            myUVSurf2.Get(out double aUSurf2f, out double aVSurf2f, out double aUSurf2l, out double aVSurf2l);

            const int aSize = 4;
            ReadOnlySpan<double> anArrVzad = [aVSurf1f, aVSurf1l, aVSurf2f, aVSurf2l];

            Span<StPInfo> aUVPoint = [new(), new(), new(), new()];

            // Branch-aware 1D root-finder for V(U1) = V_bound on the analytical
            // intersection curve, used in place of the original rank-deficient 3x3
            // Newton (SearchOnVBounds). The intersection curve is 1D in U1 on each
            // arccos branch, so the correct numerical tool is a bracketed 1D root
            // finder. We use MathRoot::Brent, which converges robustly even when the
            // curve grazes V_bound at a near-tangent (two close crossings on opposite
            // sides of a V-extremum) -- the configuration that makes the 3x3 Newton
            // Jacobian rank-deficient. The only tolerance is Precision::PConfusion()
            // (OCCT standard parametric equality) for termination on U.

            for (int anIDSurf = 0; anIDSurf < 4; anIDSurf += 2)
            {
                double aVf = (anIDSurf == 0) ? theV1 : theV2,
                    aVl = (anIDSurf == 0) ? theV1Prev : theV2Prev;

                SearchBoundType aTS = (anIDSurf == 0) ? SearchBoundType.SearchV1 : SearchBoundType.SearchV2;

                for (int anIDBound = 0; anIDBound < 2; anIDBound++)
                {
                    int anIndex = anIDSurf + anIDBound;

                    aUVPoint[anIndex].mySurfID = anIDSurf;

                    if ((Math.Abs(aVf - anArrVzad[anIndex]) > myTol2D)
                        && ((aVf - anArrVzad[anIndex]) * (aVl - anArrVzad[anIndex]) > 0.0))
                    {
                        continue;
                    }

                    // Segment [aVf, aVl] intersects at least one V-boundary (first or last).
                    // Resolve the crossing by 1D bisection on the analytical curve.
                    double aULo = Min(theU1Prev, theU1);
                    double aUHi = Max(theU1Prev, theU1);
                    bool aRes = findVBoundCrossing(theWLIndex,
                                                   aTS == SearchBoundType.SearchV1,
                                                   anArrVzad[anIndex],
                                                   aULo,
                                                   aUHi,
                                                   ref aUVPoint[anIndex].myU1);

                    // aUVPoint[anIndex].myU1 is considered to be nearer to theU1 than
                    // to theU1+/-Period
                    if (!aRes || (aUVPoint[anIndex].myU1 >= theU1) || (aUVPoint[anIndex].myU1 < theU1Min))
                    {
                        // Intersection point is not found or out of the domain
                        aUVPoint[anIndex].myU1 = RealLast();
                        continue;
                    }
                    else
                    {
                        // intersection point is found

                        ref StPInfo aP = ref aUVPoint[anIndex];
                        aP.myU2 = theU2;
                        aP.myV1 = theV1;
                        aP.myV2 = theV2;

                        if (!ComputationMethods.CylCylComputeParameters(aP.myU1,
                                                                        theWLIndex,
                                                                        myCoeffs,
                                                                        ref aP.myU2,
                                                                        ref aP.myV1,
                                                                        ref aP.myV2))
                        {
                            // Found point is wrong
                            aP.myU1 = RealLast();
                            continue;
                        }

                        // Point on true V-boundary.
                        if (aTS == SearchBoundType.SearchV1)
                        {
                            aP.myV1 = anArrVzad[anIndex];
                        }
                        else
                        { // if(aTS[anIndex] == SearchV2)
                            aP.myV2 = anArrVzad[anIndex];
                        }
                    }
                }
            }

            // Sort with acceding U1-parameter.
            // std::sort(aUVPoint, aUVPoint + aSize): MSVC's insertion sort on 4 elements (ties keep their order).
            MsvcInsertionSort(aUVPoint);

            isTheFound1 = isTheFound2 = false;

            // Adding found points on boundary in the WLine.
            for (int i = 0; i < aSize; i++)
            {
                if (aUVPoint[i].myU1 == RealLast())
                {
                    break;
                }

                if (!AddPointIntoWL(myQuad1,
                                    myQuad2,
                                    myCoeffs,
                                    myIsReverse,
                                    false,
                                    new gp_Pnt2d(aUVPoint[i].myU1, aUVPoint[i].myV1),
                                    new gp_Pnt2d(aUVPoint[i].myU2, aUVPoint[i].myV2),
                                    aUSurf1f,
                                    aUSurf1l,
                                    aUSurf2f,
                                    aUSurf2l,
                                    aVSurf1f,
                                    aVSurf1l,
                                    aVSurf2f,
                                    aVSurf2l,
                                    myPeriod,
                                    theWL.Curve(),
                                    theWLIndex,
                                    myTol3D,
                                    myTol2D,
                                    theFlForce))
                {
                    continue;
                }

                if (aUVPoint[i].mySurfID == 0)
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
        private static void MsvcInsertionSort(Span<StPInfo> theArr)
        {
            for (int aMid = 1; aMid < theArr.Length; aMid++)
            {
                int aHole = aMid;
                StPInfo aVal = theArr[aMid];
                if (aVal.myU1 < theArr[0].myU1)
                {
                    // found new earliest element, move to front
                    for (int k = aMid; k > 0; k--)
                    {
                        theArr[k] = theArr[k - 1];
                    }
                    theArr[0] = aVal;
                }
                else
                {
                    // look for insertion point after first
                    for (int aPrev = aHole - 1; aVal.myU1 < theArr[aPrev].myU1; aHole = aPrev, aPrev--)
                    {
                        theArr[aHole] = theArr[aPrev]; // move hole down
                    }
                    theArr[aHole] = aVal; // insert element in hole
                }
            }
        }

        //=======================================================================
        // function : BoundariesComputing
        // purpose  : Computes true domain of future intersection curve (allows
        //            avoiding excess iterations)
        //=======================================================================
        internal static bool BoundariesComputing(ComputationMethods.stCoeffsValue theCoeffs,
                                                 double thePeriod,
                                                 Bnd_Range[] theURange)
        {
            // All comments to this method is related to the comment
            // to ComputationMethods class

            // So, we have the equation
            //     cos(U2-FI2)=B*cos(U1-FI1)+C
            // Evidently,
            //     -1 <= B*cos(U1-FI1)+C <= 1

            if (theCoeffs.mB > 0.0)
            {
                // -(1+C)/B <= cos(U1-FI1) <= (1-C)/B

                if (theCoeffs.mB + Math.Abs(theCoeffs.mC) < -1.0) // OCCT: never true for mB > 0 (a dead branch)
                {
                    //(1-C)/B < -1 or -(1+C)/B > 1  ==> No solution

                    return false;
                }
                else if (theCoeffs.mB + Math.Abs(theCoeffs.mC) <= 1.0)
                {
                    //(1-C)/B >= 1 and -(1+C)/B <= -1 ==> U=[0;2*PI]+aFI1
                    theURange[0].Add(theCoeffs.mFI1);
                    theURange[0].Add(thePeriod + theCoeffs.mFI1);
                }
                else if ((1 + theCoeffs.mC <= theCoeffs.mB) && (theCoeffs.mB <= 1 - theCoeffs.mC))
                {
                    //(1-C)/B >= 1 and -(1+C)/B >= -1 ==>
                    //(U=[0;aDAngle]+aFI1) || (U=[2*PI-aDAngle;2*PI]+aFI1),
                    // where aDAngle = acos(-(myCoeffs.mC + 1) / myCoeffs.mB)

                    double anArg = -(theCoeffs.mC + 1) / theCoeffs.mB;
                    if (anArg > 1.0)
                    {
                        anArg = 1.0;
                    }
                    if (anArg < -1.0)
                    {
                        anArg = -1.0;
                    }

                    double aDAngle = Math.Acos(anArg);
                    theURange[0].Add(theCoeffs.mFI1);
                    theURange[0].Add(aDAngle + theCoeffs.mFI1);
                    theURange[1].Add(thePeriod - aDAngle + theCoeffs.mFI1);
                    theURange[1].Add(thePeriod + theCoeffs.mFI1);
                }
                else if ((1 - theCoeffs.mC <= theCoeffs.mB) && (theCoeffs.mB <= 1 + theCoeffs.mC))
                {
                    //(1-C)/B <= 1 and -(1+C)/B <= -1 ==> U=[aDAngle;2*PI-aDAngle]+aFI1
                    // where aDAngle = acos((1 - myCoeffs.mC) / myCoeffs.mB)

                    double anArg = (1 - theCoeffs.mC) / theCoeffs.mB;
                    if (anArg > 1.0)
                    {
                        anArg = 1.0;
                    }
                    if (anArg < -1.0)
                    {
                        anArg = -1.0;
                    }

                    double aDAngle = Math.Acos(anArg);
                    theURange[0].Add(aDAngle + theCoeffs.mFI1);
                    theURange[0].Add(thePeriod - aDAngle + theCoeffs.mFI1);
                }
                else if (theCoeffs.mB - Math.Abs(theCoeffs.mC) >= 1.0)
                {
                    //(1-C)/B <= 1 and -(1+C)/B >= -1 ==>
                    //(U=[aDAngle1;aDAngle2]+aFI1) ||
                    //(U=[2*PI-aDAngle2;2*PI-aDAngle1]+aFI1)
                    // where aDAngle1 = acos((1 - myCoeffs.mC) / myCoeffs.mB),
                    //      aDAngle2 = acos(-(myCoeffs.mC + 1) / myCoeffs.mB).

                    double anArg1 = (1 - theCoeffs.mC) / theCoeffs.mB,
                        anArg2 = -(theCoeffs.mC + 1) / theCoeffs.mB;
                    if (anArg1 > 1.0)
                    {
                        anArg1 = 1.0;
                    }
                    if (anArg1 < -1.0)
                    {
                        anArg1 = -1.0;
                    }

                    if (anArg2 > 1.0)
                    {
                        anArg2 = 1.0;
                    }
                    if (anArg2 < -1.0)
                    {
                        anArg2 = -1.0;
                    }

                    double aDAngle1 = Math.Acos(anArg1), aDAngle2 = Math.Acos(anArg2);
                    //(U=[aDAngle1;aDAngle2]+aFI1) ||
                    //(U=[2*PI-aDAngle2;2*PI-aDAngle1]+aFI1)
                    theURange[0].Add(aDAngle1 + theCoeffs.mFI1);
                    theURange[0].Add(aDAngle2 + theCoeffs.mFI1);
                    theURange[1].Add(thePeriod - aDAngle2 + theCoeffs.mFI1);
                    theURange[1].Add(thePeriod - aDAngle1 + theCoeffs.mFI1);
                }
                else
                {
                    return false;
                }
            }
            else if (theCoeffs.mB < 0.0)
            {
                // (1-C)/B <= cos(U1-FI1) <= -(1+C)/B

                if (theCoeffs.mB + Math.Abs(theCoeffs.mC) > 1.0)
                {
                    // -(1+C)/B < -1 or (1-C)/B > 1 ==> No solutions
                    return false;
                }
                else if (-theCoeffs.mB + Math.Abs(theCoeffs.mC) <= 1.0)
                {
                    //  -(1+C)/B >= 1 and (1-C)/B <= -1 ==> U=[0;2*PI]+aFI1
                    theURange[0].Add(theCoeffs.mFI1);
                    theURange[0].Add(thePeriod + theCoeffs.mFI1);
                }
                else if ((-theCoeffs.mC - 1 <= theCoeffs.mB) && (theCoeffs.mB <= theCoeffs.mC - 1))
                {
                    //  -(1+C)/B >= 1 and (1-C)/B >= -1 ==>
                    //(U=[0;aDAngle]+aFI1) || (U=[2*PI-aDAngle;2*PI]+aFI1),
                    // where aDAngle = acos((1 - myCoeffs.mC) / myCoeffs.mB)

                    double anArg = (1 - theCoeffs.mC) / theCoeffs.mB;
                    if (anArg > 1.0)
                    {
                        anArg = 1.0;
                    }
                    if (anArg < -1.0)
                    {
                        anArg = -1.0;
                    }

                    double aDAngle = Math.Acos(anArg);
                    theURange[0].Add(theCoeffs.mFI1);
                    theURange[0].Add(aDAngle + theCoeffs.mFI1);
                    theURange[1].Add(thePeriod - aDAngle + theCoeffs.mFI1);
                    theURange[1].Add(thePeriod + theCoeffs.mFI1);
                }
                // OCCT: -mB - 1 where the comment's bound is -mC - 1
                else if ((theCoeffs.mC - 1 <= theCoeffs.mB) && (theCoeffs.mB <= -theCoeffs.mB - 1))
                {
                    //  -(1+C)/B <= 1 and (1-C)/B <= -1 ==> U=[aDAngle;2*PI-aDAngle]+aFI1,
                    // where aDAngle = acos(-(myCoeffs.mC + 1) / myCoeffs.mB).

                    double anArg = -(theCoeffs.mC + 1) / theCoeffs.mB;
                    if (anArg > 1.0)
                    {
                        anArg = 1.0;
                    }
                    if (anArg < -1.0)
                    {
                        anArg = -1.0;
                    }

                    double aDAngle = Math.Acos(anArg);
                    theURange[0].Add(aDAngle + theCoeffs.mFI1);
                    theURange[0].Add(thePeriod - aDAngle + theCoeffs.mFI1);
                }
                else if (-theCoeffs.mB - Math.Abs(theCoeffs.mC) >= 1.0)
                {
                    //  -(1+C)/B <= 1 and (1-C)/B >= -1 ==>
                    //(U=[aDAngle1;aDAngle2]+aFI1) || (U=[2*PI-aDAngle2;2*PI-aDAngle1]+aFI1),
                    // where aDAngle1 = acos(-(myCoeffs.mC + 1) / myCoeffs.mB),
                    //      aDAngle2 = acos((1 - myCoeffs.mC) / myCoeffs.mB)

                    double anArg1 = -(theCoeffs.mC + 1) / theCoeffs.mB,
                        anArg2 = (1 - theCoeffs.mC) / theCoeffs.mB;
                    if (anArg1 > 1.0)
                    {
                        anArg1 = 1.0;
                    }
                    if (anArg1 < -1.0)
                    {
                        anArg1 = -1.0;
                    }

                    if (anArg2 > 1.0)
                    {
                        anArg2 = 1.0;
                    }
                    if (anArg2 < -1.0)
                    {
                        anArg2 = -1.0;
                    }

                    double aDAngle1 = Math.Acos(anArg1), aDAngle2 = Math.Acos(anArg2);
                    theURange[0].Add(aDAngle1 + theCoeffs.mFI1);
                    theURange[0].Add(aDAngle2 + theCoeffs.mFI1);
                    theURange[1].Add(thePeriod - aDAngle2 + theCoeffs.mFI1);
                    theURange[1].Add(thePeriod - aDAngle1 + theCoeffs.mFI1);
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
        internal void BoundaryEstimation(in gp_Cylinder theCy1,
                                         in gp_Cylinder theCy2,
                                         ref Bnd_Range theOutBoxS1,
                                         ref Bnd_Range theOutBoxS2)
        {
            gp_Dir aD1 = theCy1.Axis().Direction(), aD2 = theCy2.Axis().Direction();
            double aR1 = theCy1.Radius(), aR2 = theCy2.Radius();

            // Let consider a parallelogram. Its edges are parallel to aD1 and aD2.
            // Its altitudes are equal to 2*aR1 and 2*aR2 (diameters of the cylinders).
            // In fact, this parallelogram is a projection of the cylinders to the plane
            // created by the intersected axes aD1 and aD2 (if the axes are skewed then
            // one axis can be translated by parallel shifting till intersection).

            double aCosA = aD1.Dot(aD2);
            double aSqSinA = aD1.XYZ().CrossSquareMagnitude(aD2.XYZ());

            // If sine is small then it can be compared with angle.
            if (aSqSinA < Precision.Angular() * Precision.Angular())
            {
                return;
            }

            // Half of delta V. Delta V is a distance between
            // projections of two opposite parallelogram vertices
            //(joined by the maximal diagonal) to the cylinder axis.
            double aSinA = Math.Sqrt(aSqSinA);
            double anAbsCosA = Math.Abs(aCosA);
            double aHDV1 = (aR1 * anAbsCosA + aR2) / aSinA, aHDV2 = (aR2 * anAbsCosA + aR1) / aSinA;

            // V-parameters of intersection point of the axes (in case of skewed axes,
            // see comment above).
            ExtremaLineLine(theCy1.Axis(), theCy2.Axis(), aCosA, aSqSinA, out double aV01, out double aV02);

            theOutBoxS1.Add(aV01 - aHDV1);
            theOutBoxS1.Add(aV01 + aHDV1);

            theOutBoxS2.Add(aV02 - aHDV2);
            theOutBoxS2.Add(aV02 + aHDV2);

            theOutBoxS1.Enlarge(Precision.Confusion());
            theOutBoxS2.Enlarge(Precision.Confusion());

            myUVSurf1.Get(out double aU1, out double aV1, out double aU2, out double aV2);
            theOutBoxS1.Common(new Bnd_Range(aV1, aV2));

            myUVSurf2.Get(out aU1, out aV1, out aU2, out aV2);
            theOutBoxS2.Common(new Bnd_Range(aV1, aV2));
        }
    }

    //=======================================================================
    // function : MinMax
    // purpose  : Replaces theParMIN = MIN(theParMIN, theParMAX),
    //                    theParMAX = MAX(theParMIN, theParMAX).
    //=======================================================================
    private static void MinMax(ref double theParMIN, ref double theParMAX)
    {
        if (theParMIN > theParMAX)
        {
            double aux = theParMAX;
            theParMAX = theParMIN;
            theParMIN = aux;
        }
    }

    //=======================================================================
    // function : ExtremaLineLine
    // purpose  : Computes extrema between the given lines. Returns parameters
    //          on correspond curve (see correspond method for Extrema_ExtElC class).
    //=======================================================================
    private static void ExtremaLineLine(in gp_Ax1 theC1,
                                        in gp_Ax1 theC2,
                                        double theCosA,
                                        double theSqSinA,
                                        out double thePar1,
                                        out double thePar2)
    {
        gp_Dir aD1 = theC1.Direction(), aD2 = theC2.Direction();

        gp_XYZ aL1L2 = theC2.Location().XYZ() - theC1.Location().XYZ();
        double aD1L = aD1.XYZ().Dot(aL1L2), aD2L = aD2.XYZ().Dot(aL1L2);

        thePar1 = (aD1L - theCosA * aD2L) / theSqSinA;
        thePar2 = (theCosA * aD1L - aD2L) / theSqSinA;
    }

    //=======================================================================
    // function : VBoundaryPrecise
    // purpose  : By default, we shall consider, that V1 and V2 will be increased
    //            if U1 is increased. But if it is not, new V1set and/or V2set
    //            must be computed as [V1current - DeltaV1] (analogically
    //            for V2). This function processes this case.
    //=======================================================================
    private static void VBoundaryPrecise(math_Matrix theMatr,
                                         double theV1AfterDecrByDelta,
                                         double theV2AfterDecrByDelta,
                                         ref double theV1Set,
                                         ref double theV2Set)
    {
        // Now we are going to define if V1 (and V2) increases
        //(or decreases) when U1 will increase.
        const int aNbDim = 3;
        math_Matrix aSyst = new(1, aNbDim, 1, aNbDim);

        aSyst.SetCol(1, theMatr.Col(1));
        aSyst.SetCol(2, theMatr.Col(2));
        aSyst.SetCol(3, theMatr.Col(4));

        // We have the system (see comment to StepComputing(...) function)
        //     {a11*dV1 + a12*dV2 + a14*dU2 = -a13*dU1
        //     {a21*dV1 + a22*dV2 + a24*dU2 = -a23*dU1
        //     {a31*dV1 + a32*dV2 + a34*dU2 = -a33*dU1

        double aDet = aSyst.Determinant();

        aSyst.SetCol(1, theMatr.Col(3));
        double aDet1 = aSyst.Determinant();

        aSyst.SetCol(1, theMatr.Col(1));
        aSyst.SetCol(2, theMatr.Col(3));

        double aDet2 = aSyst.Determinant();

        // Now,
        //     dV1 = -dU1*aDet1/aDet
        //     dV2 = -dU1*aDet2/aDet

        // If U1 is increased then dU1 > 0.
        // If (aDet1/aDet > 0) then dV1 < 0 and
        // V1 will be decreased after increasing U1.

        // We have analogical situation with V2-parameter.

        if (aDet * aDet1 > 0.0)
        {
            theV1Set = theV1AfterDecrByDelta;
        }

        if (aDet * aDet2 > 0.0)
        {
            theV2Set = theV2AfterDecrByDelta;
        }
    }

    //=======================================================================
    // function : DeltaU1Computing
    // purpose  : Computes new step for U1 parameter.
    //=======================================================================
    private static bool DeltaU1Computing(math_Matrix theSyst, math_Vector theFree, ref double theDeltaU1Found)
    {
        double aDet = theSyst.Determinant();

        if (Math.Abs(aDet) > aNulValue)
        {
            math_Matrix aSyst1 = new(theSyst);
            aSyst1.SetCol(2, theFree);

            theDeltaU1Found = Math.Abs(aSyst1.Determinant() / aDet);
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
    private static bool StepComputing(math_Matrix theMatr,
                                      double theV1Cur,
                                      double theV2Cur,
                                      double theDeltaV1,
                                      double theDeltaV2,
                                      ref double theDeltaU1Found)
    {
        bool isSuccess = false;
        theDeltaU1Found /* = theDeltaU2Found*/ = RealLast();
        // theV1Found = theV1set;
        // theV2Found = theV2Set;
        const int aNbDim = 3;

        math_Matrix aSyst = new(1, aNbDim, 1, aNbDim);
        math_Vector aFree = new(1, aNbDim);

        // By default, increasing V1(U1) and V2(U1) functions is
        // considered
        double aV1Set = theV1Cur + theDeltaV1, aV2Set = theV2Cur + theDeltaV2;

        // However, what is indeed?
        VBoundaryPrecise(theMatr, theV1Cur - theDeltaV1, theV2Cur - theDeltaV2, ref aV1Set, ref aV2Set);

        aSyst.SetCol(2, theMatr.Col(3));
        aSyst.SetCol(3, theMatr.Col(4));

        for (int i = 0; i < 2; i++)
        {
            if (i == 0)
            { // V1 is known
                aSyst.SetCol(1, theMatr.Col(2));
                aFree.Set(1, aNbDim, theMatr.Col(5) - aV1Set * theMatr.Col(1));
            }
            else
            { // i==1 => V2 is known
                aSyst.SetCol(1, theMatr.Col(1));
                aFree.Set(1, aNbDim, theMatr.Col(5) - aV2Set * theMatr.Col(2));
            }

            double aNewDU = theDeltaU1Found;
            if (DeltaU1Computing(aSyst, aFree, ref aNewDU))
            {
                isSuccess = true;
                if (aNewDU < theDeltaU1Found)
                {
                    theDeltaU1Found = aNewDU;
                }
            }
        }

        if (!isSuccess)
        {
            aFree.Initialized(theMatr.Col(5) - aV1Set * theMatr.Col(1) - aV2Set * theMatr.Col(2));
            math_Matrix aSyst1 = new(1, aNbDim, 1, 2);
            aSyst1.SetCol(1, aSyst.Col(2));
            aSyst1.SetCol(2, aSyst.Col(3));

            // Now we have overdetermined system.

            double aDet1 = theMatr[1, 3] * theMatr[2, 4] - theMatr[2, 3] * theMatr[1, 4];
            double aDet2 = theMatr[1, 3] * theMatr[3, 4] - theMatr[3, 3] * theMatr[1, 4];
            double aDet3 = theMatr[2, 3] * theMatr[3, 4] - theMatr[3, 3] * theMatr[2, 4];
            double anAbsD1 = Math.Abs(aDet1);
            double anAbsD2 = Math.Abs(aDet2);
            double anAbsD3 = Math.Abs(aDet3);

            if (anAbsD1 >= anAbsD2)
            {
                if (anAbsD1 >= anAbsD3)
                {
                    // Det1
                    if (anAbsD1 <= aNulValue)
                    {
                        return isSuccess;
                    }

                    theDeltaU1Found = Math.Abs(aFree[1] * theMatr[2, 4] - aFree[2] * theMatr[1, 4]) / anAbsD1;
                    isSuccess = true;
                }
                else
                {
                    // Det3
                    if (anAbsD3 <= aNulValue)
                    {
                        return isSuccess;
                    }

                    theDeltaU1Found = Math.Abs(aFree[2] * theMatr[3, 4] - aFree[3] * theMatr[2, 4]) / anAbsD3;
                    isSuccess = true;
                }
            }
            else
            {
                if (anAbsD2 >= anAbsD3)
                {
                    // Det2
                    if (anAbsD2 <= aNulValue)
                    {
                        return isSuccess;
                    }

                    theDeltaU1Found = Math.Abs(aFree[1] * theMatr[3, 4] - aFree[3] * theMatr[1, 4]) / anAbsD2;
                    isSuccess = true;
                }
                else
                {
                    // Det3
                    if (anAbsD3 <= aNulValue)
                    {
                        return isSuccess;
                    }

                    theDeltaU1Found = Math.Abs(aFree[2] * theMatr[3, 4] - aFree[3] * theMatr[2, 4]) / anAbsD3;
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
    private static bool CyCyAnalyticalIntersect(in IntSurf_Quadric Quad1,
                                                in IntSurf_Quadric Quad2,
                                                IntAna_QuadQuadGeo theInter,
                                                double Tol,
                                                ref bool Empty,
                                                ref bool Same,
                                                ref bool Multpoint,
                                                NCollection_Sequence<IntPatch_Line> slin,
                                                NCollection_Sequence<IntPatch_Point> spnt)
    {
        IntPatch_Point ptsol = new();

        int i;

        IntSurf_TypeTrans trans1, trans2;
        IntAna_ResultType typint;

        gp_Elips elipsol;
        gp_Lin linsol;

        gp_Cylinder Cy1 = Quad1.Cylinder();
        gp_Cylinder Cy2 = Quad2.Cylinder();

        typint = theInter.TypeInter();
        int NbSol = theInter.NbSolutions();
        Empty = false;
        Same = false;

        switch (typint)
        {
            case IntAna_Empty:
            {
                Empty = true;
            }
            break;

            case IntAna_Same:
            {
                Same = true;
            }
            break;

            case IntAna_Point:
            {
                gp_Pnt psol = theInter.Point(1);
                ptsol.SetValue(psol, Tol, true);

                double U1 = 0.0, V1 = 0.0, U2 = 0.0, V2 = 0.0;
                Quad1.Parameters(psol, ref U1, ref V1);
                Quad2.Parameters(psol, ref U2, ref V2);

                ptsol.SetParameters(U1, V1, U2, V2);
                spnt.Append(ptsol);
            }
            break;

            case IntAna_Line:
            {
                gp_Pnt ptref;
                if (NbSol == 1)
                { // Cylinders are tangent to each other by line
                    linsol = theInter.Line(1);
                    ptref = linsol.Location();

                    // Radius-vectors
                    gp_Dir crb1 = new(new gp_Vec(ptref, Cy1.Location()));
                    gp_Dir crb2 = new(new gp_Vec(ptref, Cy2.Location()));

                    // outer normal lines
                    gp_Vec norm1 = Quad1.Normale(ptref);
                    gp_Vec norm2 = Quad2.Normale(ptref);
                    IntSurf_Situation situcyl1;
                    IntSurf_Situation situcyl2;

                    if (crb1.Dot(crb2) < 0.0)
                    { // centre de courbures "opposes"
                        // ATTENTION!!!
                        //         Normal and Radius-vector of the 1st(!) cylinder
                        //         is used for judging what the situation of the 2nd(!)
                        //         cylinder is.

                        if (norm1.Dot(new gp_Vec(crb1)) > 0.0)
                        {
                            situcyl2 = IntSurf_Inside;
                        }
                        else
                        {
                            situcyl2 = IntSurf_Outside;
                        }

                        if (norm2.Dot(new gp_Vec(crb2)) > 0.0)
                        {
                            situcyl1 = IntSurf_Inside;
                        }
                        else
                        {
                            situcyl1 = IntSurf_Outside;
                        }
                    }
                    else
                    {
                        if (Cy1.Radius() < Cy2.Radius())
                        {
                            if (norm1.Dot(new gp_Vec(crb1)) > 0.0)
                            {
                                situcyl2 = IntSurf_Inside;
                            }
                            else
                            {
                                situcyl2 = IntSurf_Outside;
                            }

                            if (norm2.Dot(new gp_Vec(crb2)) > 0.0)
                            {
                                situcyl1 = IntSurf_Outside;
                            }
                            else
                            {
                                situcyl1 = IntSurf_Inside;
                            }
                        }
                        else
                        {
                            if (norm1.Dot(new gp_Vec(crb1)) > 0.0)
                            {
                                situcyl2 = IntSurf_Outside;
                            }
                            else
                            {
                                situcyl2 = IntSurf_Inside;
                            }

                            if (norm2.Dot(new gp_Vec(crb2)) > 0.0)
                            {
                                situcyl1 = IntSurf_Inside;
                            }
                            else
                            {
                                situcyl1 = IntSurf_Outside;
                            }
                        }
                    }

                    IntPatch_GLine glig = new(linsol, true, situcyl1, situcyl2);
                    slin.Append(glig);
                }
                else
                {
                    for (i = 1; i <= NbSol; i++)
                    {
                        linsol = theInter.Line(i);
                        ptref = linsol.Location();
                        gp_Vec lsd = new(linsol.Direction());

                        // Theoretically, qwe = +/- 1.0.
                        double qwe = lsd.DotCross(Quad2.Normale(ptref), Quad1.Normale(ptref));
                        if (qwe > 0.00000001)
                        {
                            trans1 = IntSurf_Out;
                            trans2 = IntSurf_In;
                        }
                        else if (qwe < -0.00000001)
                        {
                            trans1 = IntSurf_In;
                            trans2 = IntSurf_Out;
                        }
                        else
                        {
                            trans1 = trans2 = IntSurf_Undecided;
                        }

                        IntPatch_GLine glig = new(linsol, false, trans1, trans2);
                        slin.Append(glig);
                    }
                }
            }
            break;

            case IntAna_Ellipse:
            {
                gp_Vec Tgt;
                gp_Pnt ptref;
                IntPatch_Point pmult1 = new(), pmult2 = new();

                elipsol = theInter.Ellipse(1);

                gp_Pnt pttang1 = ElCLib.Value(0.5 * M_PI, elipsol);
                gp_Pnt pttang2 = ElCLib.Value(1.5 * M_PI, elipsol);

                Multpoint = true;
                pmult1.SetValue(pttang1, Tol, true);
                pmult2.SetValue(pttang2, Tol, true);
                pmult1.SetMultiple(true);
                pmult2.SetMultiple(true);

                double oU1 = 0.0, oV1 = 0.0, oU2 = 0.0, oV2 = 0.0;
                Quad1.Parameters(pttang1, ref oU1, ref oV1);
                Quad2.Parameters(pttang1, ref oU2, ref oV2);

                pmult1.SetParameters(oU1, oV1, oU2, oV2);
                Quad1.Parameters(pttang2, ref oU1, ref oV1);
                Quad2.Parameters(pttang2, ref oU2, ref oV2);

                pmult2.SetParameters(oU1, oV1, oU2, oV2);

                // on traite la premiere ellipse

                //-- Calcul de la Transition de la ligne
                ElCLib.D1(0.0, elipsol, out ptref, out Tgt);

                // Theoretically, qwe = +/- |Tgt|.
                double qwe = Tgt.DotCross(Quad2.Normale(ptref), Quad1.Normale(ptref));
                if (qwe > 0.00000001)
                {
                    trans1 = IntSurf_Out;
                    trans2 = IntSurf_In;
                }
                else if (qwe < -0.00000001)
                {
                    trans1 = IntSurf_In;
                    trans2 = IntSurf_Out;
                }
                else
                {
                    trans1 = trans2 = IntSurf_Undecided;
                }

                //-- Transition calculee au point 0 -> Trans2 , Trans1
                //-- car ici, on devarit calculer en PI
                IntPatch_GLine glig = new(elipsol, false, trans2, trans1);
                //
                {
                    double aU1 = 0.0, aV1 = 0.0, aU2 = 0.0, aV2 = 0.0;
                    IntPatch_Point aIP = new();
                    gp_Pnt aP = ElCLib.Value(0.0, elipsol);
                    //
                    aIP.SetValue(aP, Tol, false);
                    aIP.SetMultiple(false);
                    //
                    Quad1.Parameters(aP, ref aU1, ref aV1);
                    Quad2.Parameters(aP, ref aU2, ref aV2);

                    aIP.SetParameters(aU1, aV1, aU2, aV2);
                    //
                    aIP.SetParameter(0.0);
                    glig.AddVertex(aIP);
                    glig.SetFirstPoint(1);
                    //
                    aIP.SetParameter(2.0 * M_PI);
                    glig.AddVertex(aIP);
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
                elipsol = theInter.Ellipse(2);

                double param1 = ElCLib.Parameter(elipsol, pttang1);
                double param2 = ElCLib.Parameter(elipsol, pttang2);
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
                ElCLib.D1(parampourtransition, elipsol, out ptref, out Tgt);

                // Theoretically, qwe = +/- |Tgt|.
                qwe = Tgt.DotCross(Quad2.Normale(ptref), Quad1.Normale(ptref));
                if (qwe > 0.00000001)
                {
                    trans1 = IntSurf_Out;
                    trans2 = IntSurf_In;
                }
                else if (qwe < -0.00000001)
                {
                    trans1 = IntSurf_In;
                    trans2 = IntSurf_Out;
                }
                else
                {
                    trans1 = trans2 = IntSurf_Undecided;
                }

                //-- La transition a ete calculee sur un point de cette ligne
                glig = new IntPatch_GLine(elipsol, false, trans1, trans2);
                //
                {
                    double aU1 = 0.0, aV1 = 0.0, aU2 = 0.0, aV2 = 0.0;
                    IntPatch_Point aIP = new();
                    gp_Pnt aP = ElCLib.Value(0.0, elipsol);
                    //
                    aIP.SetValue(aP, Tol, false);
                    aIP.SetMultiple(false);
                    //

                    Quad1.Parameters(aP, ref aU1, ref aV1);
                    Quad2.Parameters(aP, ref aU2, ref aV2);

                    aIP.SetParameters(aU1, aV1, aU2, aV2);
                    //
                    aIP.SetParameter(0.0);
                    glig.AddVertex(aIP);
                    glig.SetFirstPoint(1);
                    //
                    aIP.SetParameter(2.0 * M_PI);
                    glig.AddVertex(aIP);
                    glig.SetLastPoint(2);
                }
                //
                glig.AddVertex(pmult1);
                glig.AddVertex(pmult2);
                //
                slin.Append(glig);
            }
            break;

            case IntAna_Parabola:
            case IntAna_Hyperbola:
                throw new Standard_Failure("IntCyCy(): Wrong intersection type!");

            case IntAna_Circle:
            // Circle is useful when we will work with trimmed surfaces
            // (two cylinders can be tangent by their basises, e.g. circle)
            case IntAna_NoGeometricSolution:
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
    private static void ShortCosForm(double theCosFactor,
                                     double theSinFactor,
                                     ref double theCoeff,
                                     ref double theAngle)
    {
        theCoeff = Math.Sqrt(theCosFactor * theCosFactor + theSinFactor * theSinFactor);
        theAngle = 0.0;
        if (IsEqual(theCoeff, 0.0))
        {
            theAngle = 0.0;
            return;
        }

        theAngle = Math.Acos(Math.Abs(theCosFactor / theCoeff));

        if (theSinFactor > 0.0)
        {
            if (IsEqual(theCosFactor, 0.0))
            {
                theAngle = M_PI / 2.0;
            }
            else if (theCosFactor < 0.0)
            {
                theAngle = M_PI - theAngle;
            }
        }
        else if (IsEqual(theSinFactor, 0.0))
        {
            if (theCosFactor < 0.0)
            {
                theAngle = M_PI;
            }
        }
        if (theSinFactor < 0.0)
        {
            if (theCosFactor > 0.0)
            {
                theAngle = 2.0 * M_PI - theAngle;
            }
            else if (IsEqual(theCosFactor, 0.0))
            {
                theAngle = 3.0 * M_PI / 2.0;
            }
            else if (theCosFactor < 0.0)
            {
                theAngle = M_PI + theAngle;
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
    private static bool InscribePoint(double theUfTarget,
                                      double theUlTarget,
                                      ref double theUGiven,
                                      double theTol2D,
                                      double thePeriod,
                                      bool theFlForce)
    {
        if (Precision.IsInfinite(theUGiven))
        {
            return false;
        }

        if ((theUfTarget - theUGiven <= theTol2D) && (theUGiven - theUlTarget <= theTol2D))
        { // it has already inscribed

            /*
                     Utf    U      Utl
                      +     *       +
            */

            if (theFlForce)
            {
                double anUtemp = theUGiven + thePeriod;
                if ((theUfTarget - anUtemp <= theTol2D) && (anUtemp - theUlTarget <= theTol2D))
                {
                    theUGiven = anUtemp;
                    return true;
                }

                anUtemp = theUGiven - thePeriod;
                if ((theUfTarget - anUtemp <= theTol2D) && (anUtemp - theUlTarget <= theTol2D))
                {
                    theUGiven = anUtemp;
                }
            }

            return true;
        }

        double aUf = theUfTarget - theTol2D;
        double aUl = aUf + thePeriod;

        theUGiven = ElCLib.InPeriod(theUGiven, aUf, aUl);

        return ((theUfTarget - theUGiven <= theTol2D) && (theUGiven - theUlTarget <= theTol2D));
    }

    //=======================================================================
    // function : InscribeInterval
    // purpose  : Shifts theRange in order to make at least one of its
    //            boundary in the range [theUfTarget, theUlTarget]
    //=======================================================================
    private static bool InscribeInterval(double theUfTarget,
                                         double theUlTarget,
                                         ref Bnd_Range theRange,
                                         double theTol2D,
                                         double thePeriod)
    {
        double anUpar = 0.0;
        if (!theRange.GetMin(ref anUpar))
        {
            return false;
        }

        double aDelta = theRange.Delta();
        if (InscribePoint(theUfTarget,
                          theUlTarget,
                          ref anUpar,
                          theTol2D,
                          thePeriod,
                          (Math.Abs(theUlTarget - anUpar) < theTol2D)))
        {
            theRange.SetVoid();
            theRange.Add(anUpar);
            theRange.Add(anUpar + aDelta);
        }
        else
        {
            if (!theRange.GetMax(ref anUpar))
            {
                return false;
            }

            if (InscribePoint(theUfTarget,
                              theUlTarget,
                              ref anUpar,
                              theTol2D,
                              thePeriod,
                              (Math.Abs(theUfTarget - anUpar) < theTol2D)))
            {
                theRange.SetVoid();
                theRange.Add(anUpar);
                theRange.Add(anUpar - aDelta);
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
    private static bool ExcludeNearElements(Span<double> theArr,
                                            int theNOfMembers,
                                            double theUSurf1f,
                                            double theUSurf1l,
                                            double theTol)
    {
        bool aRetVal = false;
        for (int i = 1; i < theNOfMembers; i++)
        {
            ref double anA = ref theArr[i];
            ref double anB = ref theArr[i - 1];

            // Here, anA >= anB

            if (Precision.IsInfinite(anA))
            {
                break;
            }

            if ((anA - anB) < theTol)
            {
                if ((anB != 0.0) && (anB != theUSurf1f) && (anB != theUSurf1l))
                {
                    anA = (anA + anB) / 2.0;
                }
                else
                {
                    anA = anB;
                }

                // Make this element infinite an forget it
                //(we will not use it in next iterations).
                anB = Precision.Infinite();
                aRetVal = true;
            }
        }

        return aRetVal;
    }

    // std::sort of doubles as MSVC does it on up to 32 elements (_Insertion_sort_unchecked): ties (0.0 and -0.0)
    // keep their order (decision 610). OCCT sorts at most 12 critical points.
    private static void MsvcInsertionSort(Span<double> theArr)
    {
        for (int aMid = 1; aMid < theArr.Length; aMid++)
        {
            int aHole = aMid;
            double aVal = theArr[aMid];
            if (aVal < theArr[0])
            {
                // found new earliest element, move to front
                for (int k = aMid; k > 0; k--)
                {
                    theArr[k] = theArr[k - 1];
                }
                theArr[0] = aVal;
            }
            else
            {
                // look for insertion point after first
                for (int aPrev = aHole - 1; aVal < theArr[aPrev]; aHole = aPrev, aPrev--)
                {
                    theArr[aHole] = theArr[aPrev]; // move hole down
                }
                theArr[aHole] = aVal; // insert element in hole
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
    private static bool AddPointIntoWL(in IntSurf_Quadric theQuad1,
                                       in IntSurf_Quadric theQuad2,
                                       ComputationMethods.stCoeffsValue theCoeffs,
                                       bool isTheReverse,
                                       bool isThePrecise,
                                       in gp_Pnt2d thePntOnSurf1,
                                       in gp_Pnt2d thePntOnSurf2,
                                       double theUfSurf1,
                                       double theUlSurf1,
                                       double theUfSurf2,
                                       double theUlSurf2,
                                       double theVfSurf1,
                                       double theVlSurf1,
                                       double theVfSurf2,
                                       double theVlSurf2,
                                       double thePeriodOfSurf1,
                                       IntSurf_LineOn2S theLine,
                                       int theWLIndex,
                                       double theTol3D,
                                       double theTol2D,
                                       bool theFlBefore = false,
                                       bool theOnlyCheck = false)
    {
        // Check if the point is in the domain or can be inscribed in the domain after adjusting.

        gp_Pnt aPt1 = theQuad1.Value(thePntOnSurf1.X(), thePntOnSurf1.Y()),
            aPt2 = theQuad2.Value(thePntOnSurf2.X(), thePntOnSurf2.Y());

        double aU1par = thePntOnSurf1.X();

        // aU1par always increases. Therefore, we must reduce its
        // value in order to continue creation of WLine.
        if (!InscribePoint(theUfSurf1,
                           theUlSurf1,
                           ref aU1par,
                           theTol2D,
                           thePeriodOfSurf1,
                           aU1par > 0.5 * (theUfSurf1 + theUlSurf1)))
        {
            return false;
        }

        if ((theLine.NbPoints() > 0) && ((theUlSurf1 - theUfSurf1) >= (thePeriodOfSurf1 - theTol2D))
            && (((aU1par + thePeriodOfSurf1 - theUlSurf1) <= theTol2D)
                || ((aU1par - thePeriodOfSurf1 - theUfSurf1) >= theTol2D)))
        {
            // aU1par can be adjusted to both theUlSurf1 and theUfSurf1
            // with equal possibilities. This code fragment allows choosing
            // correct parameter from these two variants.

            double aU1, aV1;
            if (isTheReverse)
            {
                theLine.Value(theLine.NbPoints()).ParametersOnS2(out aU1, out aV1);
            }
            else
            {
                theLine.Value(theLine.NbPoints()).ParametersOnS1(out aU1, out aV1);
            }

            double aDelta = aU1 - aU1par;
            if (2.0 * Math.Abs(aDelta) > thePeriodOfSurf1)
            {
                aU1par += Math.CopySign(thePeriodOfSurf1, aDelta);
            }
        }

        double aU2par = thePntOnSurf2.X();
        if (!InscribePoint(theUfSurf2, theUlSurf2, ref aU2par, theTol2D, thePeriodOfSurf1, false))
        {
            return false;
        }

        double aV1par = thePntOnSurf1.Y();
        if ((aV1par - theVlSurf1 > theTol2D) || (theVfSurf1 - aV1par > theTol2D))
        {
            return false;
        }

        double aV2par = thePntOnSurf2.Y();
        if ((aV2par - theVlSurf2 > theTol2D) || (theVfSurf2 - aV2par > theTol2D))
        {
            return false;
        }

        // Get intersection point and add it in the WL
        IntSurf_PntOn2S aPnt = new();

        if (isTheReverse)
        {
            aPnt.SetValue(new gp_Pnt((aPt1.XYZ() + aPt2.XYZ()) / 2.0), aU2par, aV2par, aU1par, aV1par);
        }
        else
        {
            aPnt.SetValue(new gp_Pnt((aPt1.XYZ() + aPt2.XYZ()) / 2.0), aU1par, aV1par, aU2par, aV2par);
        }

        int aNbPnts = theLine.NbPoints();
        if (aNbPnts > 0)
        {
            double aUl, aVl;
            IntSurf_PntOn2S aPlast = theLine.Value(aNbPnts);
            if (isTheReverse)
            {
                aPlast.ParametersOnS2(out aUl, out aVl);
            }
            else
            {
                aPlast.ParametersOnS1(out aUl, out aVl);
            }

            if (!theFlBefore && (aU1par <= aUl))
            {
                // Parameter value must be increased if theFlBefore == FALSE.

                aU1par += thePeriodOfSurf1;

                // The condition is as same as in
                // InscribePoint(...) function
                if ((theUfSurf1 - aU1par > theTol2D) || (aU1par - theUlSurf1 > theTol2D))
                {
                    // New aU1par is out of target interval.
                    // Go back to old value.

                    return false;
                }
            }

            if (theOnlyCheck)
            {
                return true;
            }

            // theTol2D is minimal step along parameter changed.
            // Therefore, if we apply this minimal step two
            // neighbour points will be always "same". Consequently,
            // we should reduce tolerance for IsSame checking.
            double aDTol = 1.0 - Epsilon(1.0);
            if (aPnt.IsSame(aPlast, theTol3D * aDTol, theTol2D * aDTol))
            {
                theLine.RemovePoint(aNbPnts);
            }
        }

        if (theOnlyCheck)
        {
            return true;
        }

        theLine.Add(aPnt);

        if (!isThePrecise)
        {
            return true;
        }

        // Try to precise existing WLine
        aNbPnts = theLine.NbPoints();
        if (aNbPnts >= 3)
        {
            double aU1, aU2, aU3, aV;
            if (isTheReverse)
            {
                theLine.Value(aNbPnts).ParametersOnS2(out aU3, out aV);
                theLine.Value(aNbPnts - 1).ParametersOnS2(out aU2, out aV);
                theLine.Value(aNbPnts - 2).ParametersOnS2(out aU1, out aV);
            }
            else
            {
                theLine.Value(aNbPnts).ParametersOnS1(out aU3, out aV);
                theLine.Value(aNbPnts - 1).ParametersOnS1(out aU2, out aV);
                theLine.Value(aNbPnts - 2).ParametersOnS1(out aU1, out aV);
            }

            double aStepPrev = aU2 - aU1;
            double aStep = aU3 - aU2;

            int aDeltaStep = RealToInt(aStepPrev / aStep);

            if ((1 < aDeltaStep) && (aDeltaStep < 2000))
            {
                // Add new points in case of non-uniform distribution of existing points
                SeekAdditionalPoints(theQuad1,
                                     theQuad2,
                                     theLine,
                                     theCoeffs,
                                     theWLIndex,
                                     aDeltaStep,
                                     aNbPnts - 2,
                                     aNbPnts - 1,
                                     theTol2D,
                                     thePeriodOfSurf1,
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
    private static void SeekAdditionalPoints(in IntSurf_Quadric theQuad1,
                                             in IntSurf_Quadric theQuad2,
                                             IntSurf_LineOn2S? theLine,
                                             ComputationMethods.stCoeffsValue theCoeffs,
                                             int theWLIndex,
                                             int theMinNbPoints,
                                             int theStartPointOnLine,
                                             int theEndPointOnLine,
                                             double theTol2D,
                                             double thePeriodOfSurf2,
                                             bool isTheReverse)
    {
        if (theLine is null)
        {
            return;
        }

        int aNbPoints = theEndPointOnLine - theStartPointOnLine + 1;

        double aMinDeltaParam = theTol2D;

        {
            double u1, v1, u2, v2;

            if (isTheReverse)
            {
                theLine.Value(theStartPointOnLine).ParametersOnS2(out u1, out v1);
                theLine.Value(theEndPointOnLine).ParametersOnS2(out u2, out v2);
            }
            else
            {
                theLine.Value(theStartPointOnLine).ParametersOnS1(out u1, out v1);
                theLine.Value(theEndPointOnLine).ParametersOnS1(out u2, out v2);
            }

            aMinDeltaParam = Max(Math.Abs(u2 - u1) / IntToReal(theMinNbPoints), aMinDeltaParam);
        }

        int aLastPointIndex = theEndPointOnLine;
        double U1prec = 0.0, V1prec = 0.0, U2prec = 0.0, V2prec = 0.0;

        int aNbPointsPrev = 0;
        do
        {
            aNbPointsPrev = aNbPoints;
            for (int fp = theStartPointOnLine, lp = 0; fp < aLastPointIndex; fp = lp + 1)
            {
                double U1f, V1f; // first point in 1st suraface
                double U1l, V1l; // last  point in 1st suraface

                double U2f, V2f; // first point in 2nd suraface
                double U2l, V2l; // last  point in 2nd suraface

                lp = fp + 1;

                if (isTheReverse)
                {
                    theLine.Value(fp).ParametersOnS2(out U1f, out V1f);
                    theLine.Value(lp).ParametersOnS2(out U1l, out V1l);

                    theLine.Value(fp).ParametersOnS1(out U2f, out V2f);
                    theLine.Value(lp).ParametersOnS1(out U2l, out V2l);
                }
                else
                {
                    theLine.Value(fp).ParametersOnS1(out U1f, out V1f);
                    theLine.Value(lp).ParametersOnS1(out U1l, out V1l);

                    theLine.Value(fp).ParametersOnS2(out U2f, out V2f);
                    theLine.Value(lp).ParametersOnS2(out U2l, out V2l);
                }

                if (Math.Abs(U1l - U1f) <= aMinDeltaParam)
                {
                    // Step is minimal. It is not necessary to divide it.
                    continue;
                }

                U1prec = 0.5 * (U1f + U1l);

                if (!ComputationMethods.CylCylComputeParameters(U1prec,
                                                                theWLIndex,
                                                                theCoeffs,
                                                                ref U2prec,
                                                                ref V1prec,
                                                                ref V2prec))
                {
                    continue;
                }

                MinMax(ref U2f, ref U2l);
                if (!InscribePoint(U2f, U2l, ref U2prec, theTol2D, thePeriodOfSurf2, false))
                {
                    continue;
                }

                gp_Pnt aP1 = theQuad1.Value(U1prec, V1prec), aP2 = theQuad2.Value(U2prec, V2prec);
                gp_Pnt aPInt = new(0.5 * (aP1.XYZ() + aP2.XYZ()));

                IntSurf_PntOn2S anIP = new();
                if (isTheReverse)
                {
                    anIP.SetValue(aPInt, U2prec, V2prec, U1prec, V1prec);
                }
                else
                {
                    anIP.SetValue(aPInt, U1prec, V1prec, U2prec, V2prec);
                }

                theLine.InsertBefore(lp, anIP);

                aNbPoints++;
                aLastPointIndex++;
            }

            if (aNbPoints >= theMinNbPoints)
            {
                return;
            }
        } while (aNbPoints < theMinNbPoints && (aNbPoints != aNbPointsPrev));
    }

    //=======================================================================
    // function : CriticalPointsComputing
    // purpose  : theNbCritPointsMax contains true number of critical points.
    //            It must be initialized correctly before function calling
    //=======================================================================
    private static void CriticalPointsComputing(ComputationMethods.stCoeffsValue theCoeffs,
                                                double theUSurf1f,
                                                double theUSurf1l,
                                                double theUSurf2f,
                                                double theUSurf2l,
                                                double thePeriod,
                                                double theTol2D,
                                                ref int theNbCritPointsMax,
                                                Span<double> theU1crit)
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

        theU1crit[0] = 0.0;
        theU1crit[1] = thePeriod;
        theU1crit[2] = theUSurf1f;
        theU1crit[3] = theUSurf1l;

        double aCOS = Math.Cos(theCoeffs.mFI2);
        double aBSB = Math.Abs(theCoeffs.mB);
        if ((theCoeffs.mC - aBSB <= aCOS) && (aCOS <= theCoeffs.mC + aBSB))
        {
            double anArg = (aCOS - theCoeffs.mC) / theCoeffs.mB;
            if (anArg > 1.0)
            {
                anArg = 1.0;
            }
            if (anArg < -1.0)
            {
                anArg = -1.0;
            }

            theU1crit[4] = -Math.Acos(anArg) + theCoeffs.mFI1;
            theU1crit[5] = Math.Acos(anArg) + theCoeffs.mFI1;
        }

        double aSf = Math.Cos(theUSurf2f - theCoeffs.mFI2);
        double aSl = Math.Cos(theUSurf2l - theCoeffs.mFI2);
        MinMax(ref aSf, ref aSl);

        // In accorance with pure mathematic, theU1crit[6] and [8]
        // must be -Precision::Infinite() instead of used +Precision::Infinite()
        theU1crit[6] = Math.Abs((aSl - theCoeffs.mC) / theCoeffs.mB) < 1.0
                         ? -Math.Acos((aSl - theCoeffs.mC) / theCoeffs.mB) + theCoeffs.mFI1
                         : Precision.Infinite();
        theU1crit[7] = Math.Abs((aSf - theCoeffs.mC) / theCoeffs.mB) < 1.0
                         ? -Math.Acos((aSf - theCoeffs.mC) / theCoeffs.mB) + theCoeffs.mFI1
                         : Precision.Infinite();
        theU1crit[8] = Math.Abs((aSf - theCoeffs.mC) / theCoeffs.mB) < 1.0
                         ? Math.Acos((aSf - theCoeffs.mC) / theCoeffs.mB) + theCoeffs.mFI1
                         : Precision.Infinite();
        theU1crit[9] = Math.Abs((aSl - theCoeffs.mC) / theCoeffs.mB) < 1.0
                         ? Math.Acos((aSl - theCoeffs.mC) / theCoeffs.mB) + theCoeffs.mFI1
                         : Precision.Infinite();

        theU1crit[10] = theCoeffs.mFI1;
        theU1crit[11] = M_PI + theCoeffs.mFI1;

        // preparative treatment of array. This array must have faled to contain negative
        // infinity number

        for (int i = 0; i < theNbCritPointsMax; i++)
        {
            if (Precision.IsInfinite(theU1crit[i]))
            {
                continue;
            }

            theU1crit[i] = theU1crit[i] % thePeriod; // fmod
            if (theU1crit[i] < 0.0)
            {
                theU1crit[i] += thePeriod;
            }
        }

        // Here all not infinite elements of theU1crit are in [0, thePeriod) range

        do
        {
            MsvcInsertionSort(theU1crit[..theNbCritPointsMax]);
        } while (ExcludeNearElements(theU1crit, theNbCritPointsMax, theUSurf1f, theUSurf1l, theTol2D));

        // Here all not infinite elements in theU1crit are different and sorted.

        while (theNbCritPointsMax > 0)
        {
            ref double anB = ref theU1crit[theNbCritPointsMax - 1];
            if (Precision.IsInfinite(anB))
            {
                theNbCritPointsMax--;
                continue;
            }

            // 1st not infinte element is found

            if (theNbCritPointsMax == 1)
            {
                break;
            }

            // Here theNbCritPointsMax > 1

            ref double anA = ref theU1crit[0];

            // Compare 1st and last significant elements of theU1crit
            // They may still differs by period.

            if (Math.Abs(anB - anA - thePeriod) < theTol2D)
            { // E.g. anA == 2.0e-17, anB == (thePeriod-1.0e-18)
                anA = (anA + anB - thePeriod) / 2.0;
                anB = Precision.Infinite();
                theNbCritPointsMax--;
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

    private static IntStatus CyCyNoGeometric(in gp_Cylinder theCyl1,
                                             in gp_Cylinder theCyl2,
                                             WorkWithBoundaries theBW,
                                             Bnd_Range[] theRange,
                                             int theNbOfRanges /*=2*/,
                                             ref bool isTheEmpty,
                                             NCollection_Sequence<IntPatch_Line> theSlin,
                                             NCollection_Sequence<IntPatch_Point> theSPnt)
    {
        theBW.UVS1().Get(out double aUSurf1f, out double aVSurf1f, out double aUSurf1l, out double aVSurf1l);
        theBW.UVS2().Get(out double aUSurf2f, out double aVSurf2f, out double aUSurf2l, out double aVSurf2l);

        Bnd_Range aRangeS1 = new(), aRangeS2 = new();
        theBW.BoundaryEstimation(theCyl1, theCyl2, ref aRangeS1, ref aRangeS2);
        if (aRangeS1.IsVoid() || aRangeS2.IsVoid())
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
            const double aF = 1.0e+5;
            double aMaxV1Range = aF * theCyl1.Radius(), aMaxV2Range = aF * theCyl2.Radius();
            if ((aRangeS1.Delta() > aMaxV1Range) || (aRangeS2.Delta() > aMaxV2Range))
            {
                return IntStatus.IntStatus_InfiniteSectionCurve;
            }
        }
        //
        bool isGoodIntersection = false;
        double anOptdu = 0.0;
        for (;;)
        {
            // Checking parameters of cylinders in order to define "good intersection"
            //"Good intersection" means that axes of cylinders are almost perpendicular and
            //  one radius is much smaller than the other and small cylinder is "inside" big one.
            const double aToMuchCoeff = 3.0;
            const double aCritAngle = M_PI / 18.0; // 10 degree
            double anR1 = theCyl1.Radius();
            double anR2 = theCyl2.Radius();
            double anRmin = 0.0, anRmax = 0.0;
            // Radius criterion
            if (anR1 > aToMuchCoeff * anR2)
            {
                anRmax = anR1;
                anRmin = anR2;
            }
            else if (anR2 > aToMuchCoeff * anR1)
            {
                anRmax = anR2;
                anRmin = anR1;
            }
            else
            {
                break;
            }
            // Angle criterion
            gp_Ax1 anAx1 = theCyl1.Axis();
            gp_Ax1 anAx2 = theCyl2.Axis();
            if (!anAx1.IsNormal(anAx2, aCritAngle))
            {
                break;
            }
            // Placement criterion
            gp_Lin anL1 = new(anAx1), anL2 = new(anAx2);
            double aDist = anL1.Distance(anL2);
            if (aDist > anRmax / 2.0)
            {
                break;
            }

            isGoodIntersection = true;
            // Estimation of "optimal" du
            // Relative deflection, absolut deflection is Rmin*aDeflection
            double aDeflection = 0.001;
            int aNbP = 3;
            if (anRmin * aDeflection > 1.0e-3)
            {
                double anAngle = 1.0e0 - aDeflection;
                anAngle = 2.0e0 * Math.Acos(anAngle);
                aNbP = (int)(2.0 * M_PI / anAngle) + 1;
            }
            anOptdu = 2.0 * M_PI_2 / (double)(aNbP - 1);
            break;
        }
        //
        ComputationMethods.stCoeffsValue anEquationCoeffs = theBW.SICoeffs();
        IntSurf_Quadric aQuad1 = theBW.GetQSurface(1);
        IntSurf_Quadric aQuad2 = theBW.GetQSurface(2);
        bool isReversed = theBW.IsReversed();
        double aTol2D = theBW.Get2dTolerance();
        double aTol3D = theBW.Get3dTolerance();
        const double aPeriod = 2.0 * M_PI;
        int aNbMaxPoints = 1000;
        int aNbMinPoints = 200;
        double du;
        if (isGoodIntersection)
        {
            du = anOptdu;
            aNbMaxPoints = 200;
            aNbMinPoints = 50;
        }
        else
        {
            du = 2.0 * M_PI / aNbMaxPoints;
        }
        int aNbPts = Min(RealToInt((aUSurf1l - aUSurf1f) / du) + 1, RealToInt(20.0 * theCyl1.Radius()));
        int aNbPoints = Min(Max(aNbMinPoints, aNbPts), aNbMaxPoints);
        double aStepMin = Max(aTol2D, Precision.PConfusion()),
            aStepMax = (aUSurf1l - aUSurf1f > M_PI / 100.0)
                         ? (aUSurf1l - aUSurf1f) / IntToReal(aNbPoints)
                         : aUSurf1l - aUSurf1f;

        // The main idea of the algorithm is to change U1-parameter
        //(U-parameter of theCyl1) from aU1f to aU1l with some step
        //(step is adaptive) and to obtain set of intersection points.

        for (int i = 0; i < theNbOfRanges; i++)
        {
            if (theRange[i].IsVoid())
            {
                continue;
            }

            InscribeInterval(aUSurf1f, aUSurf1l, ref theRange[i], aTol2D, aPeriod);
        }

        if (theRange[0].Union(theRange[1]))
        {
            // Works only if (theNbOfRanges == 2).
            theRange[1].SetVoid();
        }

        // Critical points are the value of U1-parameter in the points
        // where WL must be decomposed.

        // When U1 goes through critical points its value is set up to this
        // parameter forcefully and the intersection point is added in the line.
        // After that, the WL is broken (next U1 value will be correspond to the new WL).

        // See CriticalPointsComputing(...) function to get detail information about this array.
        const int aNbCritPointsMax = 12;
        Span<double> anU1crit = stackalloc double[aNbCritPointsMax];
        anU1crit.Fill(Precision.Infinite());

        // V-boundary points (both crossings and tangencies) are caught on-the-fly
        // by the walker via AddBoundaryPoint; see the 1D branch-aware root-finder
        // it uses in place of the rank-deficient 3x3 Newton SearchOnVBounds.

        int aNbCritPoints = aNbCritPointsMax;
        CriticalPointsComputing(anEquationCoeffs,
                                aUSurf1f,
                                aUSurf1l,
                                aUSurf2f,
                                aUSurf2l,
                                aPeriod,
                                aTol2D,
                                ref aNbCritPoints,
                                anU1crit);

        // Getting Walking-line

        const int aNbWLines = 2;
        Span<bool> isAddedIntoWL = stackalloc bool[aNbWLines];
        Span<double> aU2 = stackalloc double[aNbWLines];
        Span<double> aV1 = stackalloc double[aNbWLines];
        Span<double> aV2 = stackalloc double[aNbWLines];
        Span<WLFStatus> aWLFindStatus = stackalloc WLFStatus[aNbWLines];
        Span<double> aV1Prev = stackalloc double[aNbWLines];
        Span<double> aV2Prev = stackalloc double[aNbWLines];
        Span<double> anUexpect = stackalloc double[aNbWLines];
        Span<bool> isAddingWLEnabled = stackalloc bool[aNbWLines];
        Span<double> aCriticalDelta = stackalloc double[aNbCritPointsMax];
        IntSurf_LineOn2S[] aL2S = new IntSurf_LineOn2S[aNbWLines];
        IntPatch_WLine[] aWLine = new IntPatch_WLine[aNbWLines];
        for (int aCurInterval = 0; aCurInterval < theNbOfRanges; aCurInterval++)
        {
            // Process every continuous region
            for (int i = 0; i < aNbWLines; i++)
            {
                isAddedIntoWL[i] = false;
            }

            double anUf = 1.0, anUl = 0.0;
            if (!theRange[aCurInterval].GetBounds(ref anUf, ref anUl))
            {
                continue;
            }

            bool isDeltaPeriod = IsEqual(anUl - anUf, aPeriod);

            // Inscribe and sort critical points
            for (int i = 0; i < aNbCritPoints; i++)
            {
                InscribePoint(anUf, anUl, ref anU1crit[i], 0.0, aPeriod, false);
            }

            MsvcInsertionSort(anU1crit[..aNbCritPoints]);

            while (anUf < anUl)
            {
                // Change value of U-parameter on the 1st surface from anUf to anUl
                //(anUf will be modified in the cycle body).
                // Step is computed adaptively (see comments below).

                for (int i = 0; i < aNbWLines; i++)
                {
                    aL2S[i] = new IntSurf_LineOn2S();
                    aWLine[i] = new IntPatch_WLine(aL2S[i], false);
                    aWLine[i].SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLImpImp);
                    aWLFindStatus[i] = WLFStatus.WLFStatus_Absent;
                    isAddingWLEnabled[i] = true;
                    aU2[i] = aV1[i] = aV2[i] = 0.0;
                    aV1Prev[i] = aV2Prev[i] = 0.0;
                    anUexpect[i] = anUf;
                }

                aCriticalDelta.Clear();
                for (int aCritPID = 0; aCritPID < aNbCritPoints; aCritPID++)
                { // We are not interested in elements of aCriticalDelta array
                    // if their index is greater than or equal to aNbCritPoints

                    aCriticalDelta[aCritPID] = anUf - anU1crit[aCritPID];
                }

                double anU1 = anUf, aMinCriticalParam = anUf;
                double anU1Prev = anUf;
                bool isFirst = true;

                while (anU1 <= anUl)
                {
                    // Change value of U-parameter on the 1st surface from anUf to anUl
                    //(anUf will be modified in the cycle body). However, this cycle
                    // can be broken if WL goes though some critical point.
                    // Step is computed adaptively (see comments below).

                    for (int i = 0; i < aNbCritPoints; i++)
                    {
                        if ((anU1 - anU1crit[i]) * aCriticalDelta[i] < 0.0)
                        {
                            // WL has gone through i-th critical point
                            anU1 = anU1crit[i];

                            for (int j = 0; j < aNbWLines; j++)
                            {
                                aWLFindStatus[j] = WLFStatus.WLFStatus_Broken;
                                anUexpect[j] = anU1;
                            }

                            break;
                        }
                    }

                    if (IsEqual(anU1, anUl))
                    {
                        for (int i = 0; i < aNbWLines; i++)
                        {
                            aWLFindStatus[i] = WLFStatus.WLFStatus_Broken;
                            anUexpect[i] = anU1;

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
                                    ((aTol2D >= (anUexpect[i] - anU1)) || (aWLFindStatus[i] == WLFStatus.WLFStatus_Absent));
                            }
                        } // for(int i = 0; i < aNbWLines; i++)
                    }
                    else
                    {
                        for (int i = 0; i < aNbWLines; i++)
                        {
                            isAddingWLEnabled[i] =
                                ((aTol2D >= (anUexpect[i] - anU1)) || (aWLFindStatus[i] == WLFStatus.WLFStatus_Absent));
                        } // for(int i = 0; i < aNbWLines; i++)
                    }

                    for (int i = 0; i < aNbWLines; i++)
                    {
                        int aNbPntsWL = aWLine[i] is null ? 0 : aWLine[i].Curve().NbPoints();

                        if ((aWLFindStatus[i] == WLFStatus.WLFStatus_Broken) || (aWLFindStatus[i] == WLFStatus.WLFStatus_Absent))
                        { // Begin and end of WLine must be on boundary point
                            // or on seam-edge strictly (if it is possible).

                            double aTol = aTol2D;
                            ComputationMethods.CylCylComputeParameters(anU1, i, anEquationCoeffs, ref aU2[i], ref aTol);
                            InscribePoint(aUSurf2f, aUSurf2l, ref aU2[i], aTol2D, aPeriod, false);

                            aTol = Max(aTol, aTol2D);

                            if (Math.Abs(aU2[i]) <= aTol)
                            {
                                aU2[i] = 0.0;
                            }
                            else if (Math.Abs(aU2[i] - aPeriod) <= aTol)
                            {
                                aU2[i] = aPeriod;
                            }
                            else if (Math.Abs(aU2[i] - aUSurf2f) <= aTol)
                            {
                                aU2[i] = aUSurf2f;
                            }
                            else if (Math.Abs(aU2[i] - aUSurf2l) <= aTol)
                            {
                                aU2[i] = aUSurf2l;
                            }
                        }
                        else
                        {
                            ComputationMethods.CylCylComputeParameters(anU1, i, anEquationCoeffs, ref aU2[i]);
                            InscribePoint(aUSurf2f, aUSurf2l, ref aU2[i], aTol2D, aPeriod, false);
                        }

                        if (aNbPntsWL == 0)
                        { // the line has not contained any points yet
                            if (((aUSurf2f + aPeriod - aUSurf2l) <= 2.0 * aTol2D)
                                && ((Math.Abs(aU2[i] - aUSurf2f) < aTol2D)
                                    || (Math.Abs(aU2[i] - aUSurf2l) < aTol2D)))
                            {
                                // In this case aU2[i] can have two values: current aU2[i] or
                                // aU2[i]+aPeriod (aU2[i]-aPeriod). It is necessary to choose
                                // correct value.

                                bool isIncreasing = true;
                                ComputationMethods.CylCylMonotonicity(anU1 + aStepMin,
                                                                      i,
                                                                      anEquationCoeffs,
                                                                      aPeriod,
                                                                      ref isIncreasing);

                                // If U2(U1) is increasing and U2 is considered to be equal aUSurf2l
                                // then after the next step (when U1 will be increased) U2 will be
                                // increased too. And we will go out of surface boundary.
                                // Therefore, If U2(U1) is increasing then U2 must be equal aUSurf2f.
                                // Analogically, if U2(U1) is decreasing.
                                if (isIncreasing)
                                {
                                    aU2[i] = aUSurf2f;
                                }
                                else
                                {
                                    aU2[i] = aUSurf2l;
                                }
                            }
                        }
                        else
                        { // aNbPntsWL > 0
                            if (((aUSurf2l - aUSurf2f) >= aPeriod)
                                && ((Math.Abs(aU2[i] - aUSurf2f) < aTol2D)
                                    || (Math.Abs(aU2[i] - aUSurf2l) < aTol2D)))
                            { // end of the line
                                double aU2prev, aV2prev;
                                if (isReversed)
                                {
                                    aWLine[i].Curve().Value(aNbPntsWL).ParametersOnS1(out aU2prev, out aV2prev);
                                }
                                else
                                {
                                    aWLine[i].Curve().Value(aNbPntsWL).ParametersOnS2(out aU2prev, out aV2prev);
                                }

                                if (2.0 * Math.Abs(aU2prev - aU2[i]) > aPeriod)
                                {
                                    if (aU2prev > aU2[i])
                                    {
                                        aU2[i] += aPeriod;
                                    }
                                    else
                                    {
                                        aU2[i] -= aPeriod;
                                    }
                                }
                            }
                        }

                        ComputationMethods.CylCylComputeParameters(anU1,
                                                                   aU2[i],
                                                                   anEquationCoeffs,
                                                                   ref aV1[i],
                                                                   ref aV2[i]);

                        if (isFirst)
                        {
                            aV1Prev[i] = aV1[i];
                            aV2Prev[i] = aV2[i];
                        }
                    } // for(int i = 0; i < aNbWLines; i++)

                    isFirst = false;

                    // Looking for points into WLine
                    bool isBroken = false;
                    for (int i = 0; i < aNbWLines; i++)
                    {
                        if (!isAddingWLEnabled[i])
                        {
                            bool isBoundIntersect = false;
                            if ((Math.Abs(aV1[i] - aVSurf1f) <= aTol2D)
                                || ((aV1[i] - aVSurf1f) * (aV1Prev[i] - aVSurf1f) < 0.0))
                            {
                                isBoundIntersect = true;
                            }
                            else if ((Math.Abs(aV1[i] - aVSurf1l) <= aTol2D)
                                     || ((aV1[i] - aVSurf1l) * (aV1Prev[i] - aVSurf1l) < 0.0))
                            {
                                isBoundIntersect = true;
                            }
                            else if ((Math.Abs(aV2[i] - aVSurf2f) <= aTol2D)
                                     || ((aV2[i] - aVSurf2f) * (aV2Prev[i] - aVSurf2f) < 0.0))
                            {
                                isBoundIntersect = true;
                            }
                            else if ((Math.Abs(aV2[i] - aVSurf2l) <= aTol2D)
                                     || ((aV2[i] - aVSurf2l) * (aV2Prev[i] - aVSurf2l) < 0.0))
                            {
                                isBoundIntersect = true;
                            }

                            if (aWLFindStatus[i] == WLFStatus.WLFStatus_Broken)
                            {
                                isBroken = true;
                            }

                            if (!isBoundIntersect)
                            {
                                continue;
                            }
                            else
                            {
                                anUexpect[i] = anU1;
                            }
                        }

                        // True if the current point already in the domain
                        bool isInscribe =
                            ((aUSurf2f - aU2[i]) <= aTol2D) && ((aU2[i] - aUSurf2l) <= aTol2D)
                            && ((aVSurf1f - aV1[i]) <= aTol2D) && ((aV1[i] - aVSurf1l) <= aTol2D)
                            && ((aVSurf2f - aV2[i]) <= aTol2D) && ((aV2[i] - aVSurf2l) <= aTol2D);

                        // isVIntersect == TRUE if intersection line intersects two (!)
                        // V-bounds of cylinder (1st or 2nd - no matter)
                        bool isVIntersect =
                            (((aVSurf1f - aV1[i]) * (aVSurf1f - aV1Prev[i]) < RealSmall())
                             && ((aVSurf1l - aV1[i]) * (aVSurf1l - aV1Prev[i]) < RealSmall()))
                            || (((aVSurf2f - aV2[i]) * (aVSurf2f - aV2Prev[i]) < RealSmall())
                                && ((aVSurf2l - aV2[i]) * (aVSurf2l - aV2Prev[i]) < RealSmall()));

                        // isFound1 == TRUE if intersection line intersects V-bounds
                        //   (First or Last - no matter) of the 1st cylinder
                        // isFound2 == TRUE if intersection line intersects V-bounds
                        //   (First or Last - no matter) of the 2nd cylinder
                        bool isFound1 = false, isFound2 = false;
                        bool isForce = false;

                        if (aWLFindStatus[i] == WLFStatus.WLFStatus_Absent)
                        {
                            if (((aUSurf2l - aUSurf2f) >= aPeriod) && (Math.Abs(anU1 - aUSurf1l) < aTol2D))
                            {
                                isForce = true;
                            }
                        }

                        theBW.AddBoundaryPoint(aWLine[i],
                                               anU1,
                                               anU1Prev,
                                               aMinCriticalParam,
                                               aU2[i],
                                               aV1[i],
                                               aV1Prev[i],
                                               aV2[i],
                                               aV2Prev[i],
                                               i,
                                               isForce,
                                               ref isFound1,
                                               ref isFound2);

                        bool isPrevVBound = !isVIntersect
                                            && ((Math.Abs(aV1Prev[i] - aVSurf1f) <= aTol2D)
                                                || (Math.Abs(aV1Prev[i] - aVSurf1l) <= aTol2D)
                                                || (Math.Abs(aV2Prev[i] - aVSurf2f) <= aTol2D)
                                                || (Math.Abs(aV2Prev[i] - aVSurf2l) <= aTol2D));

                        aV1Prev[i] = aV1[i];
                        aV2Prev[i] = aV2[i];

                        if ((aWLFindStatus[i] == WLFStatus.WLFStatus_Exist) && (isFound1 || isFound2) && !isPrevVBound)
                        {
                            aWLFindStatus[i] = WLFStatus.WLFStatus_Broken; // start a new line
                        }
                        else if (isInscribe)
                        {
                            if ((aWLFindStatus[i] == WLFStatus.WLFStatus_Absent) && (isFound1 || isFound2))
                            {
                                aWLFindStatus[i] = WLFStatus.WLFStatus_Exist;
                            }

                            if ((aWLFindStatus[i] != WLFStatus.WLFStatus_Broken) || (aWLine[i].NbPnts() >= 1)
                                || IsEqual(anU1, anUl))
                            {
                                if (aWLine[i].NbPnts() > 0)
                                {
                                    double aU2p, aV2p;
                                    if (isReversed)
                                    {
                                        aWLine[i].Point(aWLine[i].NbPnts()).ParametersOnS1(out aU2p, out aV2p);
                                    }
                                    else
                                    {
                                        aWLine[i].Point(aWLine[i].NbPnts()).ParametersOnS2(out aU2p, out aV2p);
                                    }

                                    double aDelta = aU2[i] - aU2p;

                                    if (2.0 * Math.Abs(aDelta) > aPeriod)
                                    {
                                        if (aDelta > 0.0)
                                        {
                                            aU2[i] -= aPeriod;
                                        }
                                        else
                                        {
                                            aU2[i] += aPeriod;
                                        }
                                    }
                                }

                                if (AddPointIntoWL(aQuad1,
                                                   aQuad2,
                                                   anEquationCoeffs,
                                                   isReversed,
                                                   true,
                                                   new gp_Pnt2d(anU1, aV1[i]),
                                                   new gp_Pnt2d(aU2[i], aV2[i]),
                                                   aUSurf1f,
                                                   aUSurf1l,
                                                   aUSurf2f,
                                                   aUSurf2l,
                                                   aVSurf1f,
                                                   aVSurf1l,
                                                   aVSurf2f,
                                                   aVSurf2l,
                                                   aPeriod,
                                                   aWLine[i].Curve(),
                                                   i,
                                                   aTol3D,
                                                   aTol2D,
                                                   isForce))
                                {
                                    if (aWLFindStatus[i] == WLFStatus.WLFStatus_Absent)
                                    {
                                        aWLFindStatus[i] = WLFStatus.WLFStatus_Exist;
                                    }
                                }
                                else if (!isFound1 && !isFound2)
                                { // We do not add any point while doing this iteration
                                    if (aWLFindStatus[i] == WLFStatus.WLFStatus_Exist)
                                    {
                                        aWLFindStatus[i] = WLFStatus.WLFStatus_Broken;
                                    }
                                }
                            }
                        }
                        else
                        { // We do not add any point while doing this iteration
                            if (aWLFindStatus[i] == WLFStatus.WLFStatus_Exist)
                            {
                                aWLFindStatus[i] = WLFStatus.WLFStatus_Broken;
                            }
                        }

                        if (aWLFindStatus[i] == WLFStatus.WLFStatus_Broken)
                        {
                            isBroken = true;
                        }
                    } // for(int i = 0; i < aNbWLines; i++)

                    if (isBroken)
                    { // current lines are filled. Go to the next lines
                        anUf = anU1;

                        bool isAdded = true;

                        for (int i = 0; i < aNbWLines; i++)
                        {
                            if (isAddingWLEnabled[i])
                            {
                                continue;
                            }

                            isAdded = false;

                            bool isFound1 = false, isFound2 = false;

                            theBW.AddBoundaryPoint(aWLine[i],
                                                   anU1,
                                                   anU1Prev,
                                                   aMinCriticalParam,
                                                   aU2[i],
                                                   aV1[i],
                                                   aV1Prev[i],
                                                   aV2[i],
                                                   aV2Prev[i],
                                                   i,
                                                   false,
                                                   ref isFound1,
                                                   ref isFound2);

                            if (isFound1 || isFound2)
                            {
                                isAdded = true;
                            }

                            if (aWLine[i].NbPnts() > 0)
                            {
                                double aU2p, aV2p;
                                if (isReversed)
                                {
                                    aWLine[i].Point(aWLine[i].NbPnts()).ParametersOnS1(out aU2p, out aV2p);
                                }
                                else
                                {
                                    aWLine[i].Point(aWLine[i].NbPnts()).ParametersOnS2(out aU2p, out aV2p);
                                }

                                double aDelta = aU2[i] - aU2p;

                                if (2 * Math.Abs(aDelta) > aPeriod)
                                {
                                    if (aDelta > 0.0)
                                    {
                                        aU2[i] -= aPeriod;
                                    }
                                    else
                                    {
                                        aU2[i] += aPeriod;
                                    }
                                }
                            }

                            if (AddPointIntoWL(aQuad1,
                                               aQuad2,
                                               anEquationCoeffs,
                                               isReversed,
                                               true,
                                               new gp_Pnt2d(anU1, aV1[i]),
                                               new gp_Pnt2d(aU2[i], aV2[i]),
                                               aUSurf1f,
                                               aUSurf1l,
                                               aUSurf2f,
                                               aUSurf2l,
                                               aVSurf1f,
                                               aVSurf1l,
                                               aVSurf2f,
                                               aVSurf2l,
                                               aPeriod,
                                               aWLine[i].Curve(),
                                               i,
                                               aTol3D,
                                               aTol2D,
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
                            double anUmaxAdded = RealFirst();

                            {
                                bool isChanged = false;
                                for (int i = 0; i < aNbWLines; i++)
                                {
                                    if ((aWLFindStatus[i] == WLFStatus.WLFStatus_Absent) || (aWLine[i].NbPnts() == 0))
                                    {
                                        continue;
                                    }

                                    double aU1c, aV1c;
                                    if (isReversed)
                                    {
                                        aWLine[i].Curve().Value(aWLine[i].NbPnts()).ParametersOnS2(out aU1c, out aV1c);
                                    }
                                    else
                                    {
                                        aWLine[i].Curve().Value(aWLine[i].NbPnts()).ParametersOnS1(out aU1c, out aV1c);
                                    }

                                    anUmaxAdded = Max(anUmaxAdded, aU1c);
                                    isChanged = true;
                                }

                                if (!isChanged)
                                { // If anUmaxAdded were not changed in previous cycle then
                                    // we would break existing WLines.
                                    break;
                                }
                            }

                            for (int i = 0; i < aNbWLines; i++)
                            {
                                if (isAddingWLEnabled[i])
                                {
                                    continue;
                                }

                                ComputationMethods.CylCylComputeParameters(anUmaxAdded,
                                                                           i,
                                                                           anEquationCoeffs,
                                                                           ref aU2[i],
                                                                           ref aV1[i],
                                                                           ref aV2[i]);

                                AddPointIntoWL(aQuad1,
                                               aQuad2,
                                               anEquationCoeffs,
                                               isReversed,
                                               true,
                                               new gp_Pnt2d(anUmaxAdded, aV1[i]),
                                               new gp_Pnt2d(aU2[i], aV2[i]),
                                               aUSurf1f,
                                               aUSurf1l,
                                               aUSurf2f,
                                               aUSurf2l,
                                               aVSurf1f,
                                               aVSurf1l,
                                               aVSurf2f,
                                               aVSurf2l,
                                               aPeriod,
                                               aWLine[i].Curve(),
                                               i,
                                               aTol3D,
                                               aTol2D,
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

                        double aDeltaV1 = aRangeS1.Delta() / IntToReal(aNbPoints),
                            aDeltaV2 = aRangeS2.Delta() / IntToReal(aNbPoints);

                        math_Matrix aMatr = new(1, 3, 1, 5);

                        double aMinUexp = RealLast();

                        for (int i = 0; i < aNbWLines; i++)
                        {
                            if (aTol2D < (anUexpect[i] - anU1))
                            {
                                continue;
                            }

                            if (aWLFindStatus[i] == WLFStatus.WLFStatus_Absent)
                            {
                                anUexpect[i] += aStepMax;
                                aMinUexp = Min(aMinUexp, anUexpect[i]);
                                continue;
                            }
                            //
                            if (isGoodIntersection)
                            {
                                // Use constant step
                                anUexpect[i] += aStepMax;
                                aMinUexp = Min(aMinUexp, anUexpect[i]);

                                continue;
                            }
                            //

                            double aStepTmp = aStepMax;

                            double aSinU1 = Math.Sin(anU1), aCosU1 = Math.Cos(anU1), aSinU2 = Math.Sin(aU2[i]),
                                aCosU2 = Math.Cos(aU2[i]);

                            aMatr.SetCol(1, anEquationCoeffs.mVecC1);
                            aMatr.SetCol(2, anEquationCoeffs.mVecC2);
                            aMatr.SetCol(3, anEquationCoeffs.mVecA1 * aSinU1 - anEquationCoeffs.mVecB1 * aCosU1);
                            aMatr.SetCol(4, anEquationCoeffs.mVecA2 * aSinU2 - anEquationCoeffs.mVecB2 * aCosU2);
                            aMatr.SetCol(5,
                                         anEquationCoeffs.mVecA1 * aCosU1 + anEquationCoeffs.mVecB1 * aSinU1
                                           + anEquationCoeffs.mVecA2 * aCosU2 + anEquationCoeffs.mVecB2 * aSinU2
                                           + anEquationCoeffs.mVecD);

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

                            if (!StepComputing(aMatr, aV1[i], aV2[i], aDeltaV1, aDeltaV2, ref aStepTmp))
                            {
                                // To avoid cycling-up
                                anUexpect[i] += aStepMax;
                                aMinUexp = Min(aMinUexp, anUexpect[i]);

                                continue;
                            }

                            if (aStepTmp < aStepMin)
                            {
                                aStepTmp = aStepMin;
                            }

                            if (aStepTmp > aStepMax)
                            {
                                aStepTmp = aStepMax;
                            }

                            anUexpect[i] = anU1 + aStepTmp;
                            aMinUexp = Min(aMinUexp, anUexpect[i]);
                        }

                        anU1Prev = anU1;
                        anU1 = aMinUexp;
                    }

                    if (Precision.PConfusion() >= (anUl - anU1))
                    {
                        anU1 = anUl;
                    }

                    anUf = anU1;

                    for (int i = 0; i < aNbWLines; i++)
                    {
                        if (aWLine[i].NbPnts() != 1)
                        {
                            isAddedIntoWL[i] = false;
                        }

                        if (anU1 == anUl)
                        { // strictly equal. Tolerance is considered above.
                            anUexpect[i] = anUl;
                        }
                    }
                }

                for (int i = 0; i < aNbWLines; i++)
                {
                    if ((aWLine[i].NbPnts() == 1) && (!isAddedIntoWL[i]))
                    {
                        isTheEmpty = false;
                        aWLine[i].Point(1).Parameters(out double u1, out double v1, out double u2, out double v2);
                        IntPatch_Point aP = new();
                        aP.SetParameter(u1);
                        aP.SetParameters(u1, v1, u2, v2);
                        aP.SetTolerance(aTol3D);
                        aP.SetValue(aWLine[i].Point(1).Value());

                        // Check whether the added point exists.
                        // It is enough to check the last point.
                        if (theSPnt.IsEmpty()
                            || !theSPnt.Last().PntOn2S().IsSame(aP.PntOn2S(), Precision.Confusion()))
                        {
                            theSPnt.Append(aP);
                        }
                    }
                    else if (aWLine[i].NbPnts() > 1)
                    {
                        bool isGood = true;

                        if (aWLine[i].NbPnts() == 2)
                        {
                            IntSurf_PntOn2S aPf = aWLine[i].Point(1);
                            IntSurf_PntOn2S aPl = aWLine[i].Point(2);

                            if (aPf.IsSame(aPl, Precision.Confusion()))
                            {
                                isGood = false;
                            }
                        }
                        else if (aWLine[i].NbPnts() > 2)
                        {
                            // Sometimes points of the WLine are distributed
                            // linearly and uniformly. However, such position
                            // of the points does not always describe the real intersection
                            // curve. I.e. real tangents at the ends of the intersection
                            // curve can significantly deviate from this "line" direction.
                            // Here we are processing this case by inserting additional points
                            // to the beginning/end of the WLine to make it more precise.
                            // See description to the issue #30082.

                            double aSqTol3D = aTol3D * aTol3D;
                            for (int j = 0; j < 2; j++)
                            {
                                // If j == 0 ==> add point at begin of WLine.
                                // If j == 1 ==> add point at end of WLine.

                                for (;;)
                                {
                                    if (aWLine[i].NbPnts() >= aNbMaxPoints)
                                    {
                                        break;
                                    }

                                    // Take 1st and 2nd point to compute the "line" direction.
                                    // For our convenience, we make 2nd point be the ends of the WLine
                                    // because it will be used for computation of the normals
                                    // to the surfaces.
                                    int anIdx1 = j != 0 ? aWLine[i].NbPnts() - 1 : 2;
                                    int anIdx2 = j != 0 ? aWLine[i].NbPnts() : 1;

                                    gp_Pnt aP1 = aWLine[i].Point(anIdx1).Value();
                                    gp_Pnt aP2 = aWLine[i].Point(anIdx2).Value();

                                    gp_Vec aDir = new(aP1, aP2);

                                    if (aDir.SquareMagnitude() < aSqTol3D)
                                    {
                                        break;
                                    }

                                    // Compute tangent in first/last point of the WLine.
                                    // We do not take into account the flag "isReversed"
                                    // because strict direction of the tangent is not
                                    // important here (we are interested in the tangent
                                    // line itself and nothing to fear if its direction
                                    // is reversed).
                                    gp_Vec aN1 = aQuad1.Normale(aP2);
                                    gp_Vec aN2 = aQuad2.Normale(aP2);
                                    gp_Vec aTg = aN1.Crossed(aN2);

                                    if (aTg.SquareMagnitude() < Precision.SquareConfusion())
                                    {
                                        // Tangent zone
                                        break;
                                    }

                                    // Check of the bending
                                    double anAngle = aDir.Angle(aTg);

                                    if (anAngle > M_PI_2)
                                    {
                                        anAngle -= M_PI;
                                    }

                                    if (Math.Abs(anAngle) > 0.25) // ~ 14deg.
                                    {
                                        int aNbPntsPrev = aWLine[i].NbPnts();
                                        SeekAdditionalPoints(aQuad1,
                                                             aQuad2,
                                                             aWLine[i].Curve(),
                                                             anEquationCoeffs,
                                                             i,
                                                             3,
                                                             anIdx1,
                                                             anIdx2,
                                                             aTol2D,
                                                             aPeriod,
                                                             isReversed);

                                        if (aWLine[i].NbPnts() == aNbPntsPrev)
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
                            SeekAdditionalPoints(aQuad1,
                                                 aQuad2,
                                                 aWLine[i].Curve(),
                                                 anEquationCoeffs,
                                                 i,
                                                 aNbPoints,
                                                 1,
                                                 aWLine[i].NbPnts(),
                                                 aTol2D,
                                                 aPeriod,
                                                 isReversed);

                            aWLine[i].ComputeVertexParameters(aTol3D);
                            theSlin.Append(aWLine[i]);
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
        for (int aNbPnt = 1; aNbPnt <= theSPnt.Length(); aNbPnt++)
        {
            for (int aNbLin = 1; aNbLin <= theSlin.Length(); aNbLin++)
            {
                IntPatch_WLine aWLine1 = (theSlin.Value(aNbLin) as IntPatch_WLine)!;

                IntSurf_PntOn2S aPntFWL1 = aWLine1.Point(1);
                IntSurf_PntOn2S aPntLWL1 = aWLine1.Point(aWLine1.NbPnts());

                IntSurf_PntOn2S aPntCur = theSPnt.Value(aNbPnt).PntOn2S();
                if (aPntCur.IsSame(aPntFWL1, aTol3D) || aPntCur.IsSame(aPntLWL1, aTol3D))
                {
                    theSPnt.Remove(aNbPnt);
                    aNbPnt--;
                    break;
                }
            }
        }

        // Try to add new points in the neighborhood of existing point
        for (int aNbPnt = 1; aNbPnt <= theSPnt.Length(); aNbPnt++)
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

            IntPatch_Point aPnt2S = theSPnt.Value(aNbPnt);

            aPnt2S.Parameters(out double u1, out double v1, out double u2, out double v2);

            IntSurf_LineOn2S aL2Sloc = new();
            IntPatch_WLine aWLineLoc = new(aL2Sloc, false);
            aWLineLoc.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLImpImp);

            // Define the index of WLine, which lies the point aPnt2S in.
            int anIndex = 0;

            double anUf = 0.0, anUl = 0.0, aCurU2 = 0.0;
            if (isReversed)
            {
                anUf = Max(u2 - aStepMax, aUSurf1f);
                anUl = Min(u2 + aStepMax, aUSurf1l);
                aCurU2 = u1;
            }
            else
            {
                anUf = Max(u1 - aStepMax, aUSurf1f);
                anUl = Min(u1 + aStepMax, aUSurf1l);
                aCurU2 = u2;
            }

            double anUinf = anUf, anUsup = anUl, anUmid = 0.5 * (anUf + anUl);

            {
                // Find the value of anIndex variable.
                double aDelta = RealLast();
                for (int i = 0; i < aNbWLines; i++)
                {
                    double anU2t = 0.0;
                    if (!ComputationMethods.CylCylComputeParameters(anUmid, i, anEquationCoeffs, ref anU2t))
                    {
                        continue;
                    }

                    double aDU2 = Math.Abs(anU2t - aCurU2) % aPeriod; // fmod
                    aDU2 = Min(aDU2, Math.Abs(aDU2 - aPeriod));
                    if (aDU2 < aDelta)
                    {
                        aDelta = aDU2;
                        anIndex = i;
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

            Span<double> anAddedPar = [isReversed ? u2 : u1, isReversed ? u2 : u1];

            for (int aParID = 0; aParID < 2; aParID++)
            {
                if (aParID == 0)
                {
                    anUf = anUinf;
                    anUl = anUmid;
                }
                else // if(aParID == 1)
                {
                    anUf = anUmid;
                    anUl = anUsup;
                }

                ref double aPar1 = ref (aParID == 0 ? ref anUf : ref anUl);
                ref double aPar2 = ref (aParID == 0 ? ref anUl : ref anUf);

                while (Math.Abs(aPar2 - aPar1) > aStepMin)
                {
                    double anUC = 0.5 * (anUf + anUl);
                    double aU2loc = 0.0, aV1loc = 0.0, aV2loc = 0.0;
                    bool isDone = ComputationMethods.CylCylComputeParameters(anUC,
                                                                             anIndex,
                                                                             anEquationCoeffs,
                                                                             ref aU2loc,
                                                                             ref aV1loc,
                                                                             ref aV2loc);

                    if (isDone)
                    {
                        if (Math.Abs(aV1loc - aVSurf1f) <= aTol2D)
                        {
                            aV1loc = aVSurf1f;
                        }

                        if (Math.Abs(aV1loc - aVSurf1l) <= aTol2D)
                        {
                            aV1loc = aVSurf1l;
                        }

                        if (Math.Abs(aV2loc - aVSurf2f) <= aTol2D)
                        {
                            aV2loc = aVSurf2f;
                        }

                        if (Math.Abs(aV2loc - aVSurf2l) <= aTol2D)
                        {
                            aV2loc = aVSurf2l;
                        }

                        isDone = AddPointIntoWL(aQuad1,
                                                aQuad2,
                                                anEquationCoeffs,
                                                isReversed,
                                                true,
                                                new gp_Pnt2d(anUC, aV1loc),
                                                new gp_Pnt2d(aU2loc, aV2loc),
                                                aUSurf1f,
                                                aUSurf1l,
                                                aUSurf2f,
                                                aUSurf2l,
                                                aVSurf1f,
                                                aVSurf1l,
                                                aVSurf2f,
                                                aVSurf2l,
                                                aPeriod,
                                                aWLineLoc.Curve(),
                                                anIndex,
                                                aTol3D,
                                                aTol2D,
                                                false,
                                                true);
                    }

                    if (isDone)
                    {
                        anAddedPar[0] = Min(anAddedPar[0], anUC);
                        anAddedPar[1] = Max(anAddedPar[1], anUC);
                        aPar2 = anUC;
                    }
                    else
                    {
                        aPar1 = anUC;
                    }
                }
            }

            // Fill aWLine by additional points
            if (anAddedPar[1] - anAddedPar[0] > aStepMin)
            {
                for (int aParID = 0; aParID < 2; aParID++)
                {
                    double aU2loc = 0.0, aV1loc = 0.0, aV2loc = 0.0;
                    ComputationMethods.CylCylComputeParameters(anAddedPar[aParID],
                                                               anIndex,
                                                               anEquationCoeffs,
                                                               ref aU2loc,
                                                               ref aV1loc,
                                                               ref aV2loc);

                    AddPointIntoWL(aQuad1,
                                   aQuad2,
                                   anEquationCoeffs,
                                   isReversed,
                                   true,
                                   new gp_Pnt2d(anAddedPar[aParID], aV1loc),
                                   new gp_Pnt2d(aU2loc, aV2loc),
                                   aUSurf1f,
                                   aUSurf1l,
                                   aUSurf2f,
                                   aUSurf2l,
                                   aVSurf1f,
                                   aVSurf1l,
                                   aVSurf2f,
                                   aVSurf2l,
                                   aPeriod,
                                   aWLineLoc.Curve(),
                                   anIndex,
                                   aTol3D,
                                   aTol2D,
                                   false,
                                   false);
                }

                SeekAdditionalPoints(aQuad1,
                                     aQuad2,
                                     aWLineLoc.Curve(),
                                     anEquationCoeffs,
                                     anIndex,
                                     aNbMinPoints,
                                     1,
                                     aWLineLoc.NbPnts(),
                                     aTol2D,
                                     aPeriod,
                                     isReversed);

                aWLineLoc.ComputeVertexParameters(aTol3D);
                theSlin.Append(aWLineLoc);

                theSPnt.Remove(aNbPnt);
                aNbPnt--;
            }
        }

        return IntStatus.IntStatus_OK;
    }

    //=================================================================================================

    private static IntStatus IntCyCy(in IntSurf_Quadric theQuad1,
                                     in IntSurf_Quadric theQuad2,
                                     double theTol3D,
                                     double theTol2D,
                                     in Bnd_Box2d theUVSurf1,
                                     in Bnd_Box2d theUVSurf2,
                                     ref bool isTheEmpty,
                                     ref bool isTheSameSurface,
                                     ref bool isTheMultiplePoint,
                                     NCollection_Sequence<IntPatch_Line> theSlin,
                                     NCollection_Sequence<IntPatch_Point> theSPnt)
    {
        isTheEmpty = true;
        isTheSameSurface = false;
        isTheMultiplePoint = false;
        theSlin.Clear();
        theSPnt.Clear();

        gp_Cylinder aCyl1 = theQuad1.Cylinder(), aCyl2 = theQuad2.Cylinder();

        IntAna_QuadQuadGeo anInter = new(aCyl1, aCyl2, theTol3D);

        if (!anInter.IsDone())
        {
            return IntStatus.IntStatus_Fail;
        }

        if (anInter.TypeInter() != IntAna_NoGeometricSolution)
        {
            if (CyCyAnalyticalIntersect(theQuad1,
                                        theQuad2,
                                        anInter,
                                        theTol3D,
                                        ref isTheEmpty,
                                        ref isTheSameSurface,
                                        ref isTheMultiplePoint,
                                        theSlin,
                                        theSPnt))
            {
                return IntStatus.IntStatus_OK;
            }

            // Analytical handler declined: discard anything it may have appended
            // or flipped so the numerical path starts from a clean output state.
            isTheEmpty = true;
            isTheSameSurface = false;
            isTheMultiplePoint = false;
            theSlin.Clear();
            theSPnt.Clear();
        }

        // Here, intersection line is not an analytical curve(line, circle, ellipsis etc.)

        double[,] aUSBou = new double[2, 2], aVSBou = new double[2, 2]; // const

        theUVSurf1.Get(out aUSBou[0, 0], out aVSBou[0, 0], out aUSBou[0, 1], out aVSBou[0, 1]);
        theUVSurf2.Get(out aUSBou[1, 0], out aVSBou[1, 0], out aUSBou[1, 1], out aVSBou[1, 1]);

        const double aPeriod = 2.0 * M_PI;
        const int aNbWLines = 2;

        ComputationMethods.stCoeffsValue anEquationCoeffs1 = new(aCyl1, aCyl2);
        ComputationMethods.stCoeffsValue anEquationCoeffs2 = new(aCyl2, aCyl1);

        // Boundaries.
        // Intersection result can include two non-connected regions
        //(see WorkWithBoundaries::BoundariesComputing(...) method).
        const int aNbOfBoundaries = 2;
        Bnd_Range[][] anURange = [[new(), new()], [new(), new()]]; // const

        if (!WorkWithBoundaries.BoundariesComputing(anEquationCoeffs1, aPeriod, anURange[0]))
        {
            return IntStatus.IntStatus_OK;
        }

        if (!WorkWithBoundaries.BoundariesComputing(anEquationCoeffs2, aPeriod, anURange[1]))
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

        Span<double> aSumRange = [0.0, 0.0];
        for (int aCID = 0; aCID < 2; aCID++)
        {
            NCollection_List<Bnd_Range> aListOfRng = new();

            aListOfRng.Append(anURange[aCID][0]);
            aListOfRng.Append(anURange[aCID][1]);

            ReadOnlySpan<double> aSplitArr = [aUSBou[aCID, 0], aUSBou[aCID, 1], 0.0];

            for (int aSInd = 0; aSInd < 3; aSInd++)
            {
                NCollection_List<Bnd_Range> aLstTemp = new(aListOfRng);
                aListOfRng.Clear();
                foreach (Bnd_Range aRng in aLstTemp)
                {
                    aRng.Split(aSplitArr[aSInd], aListOfRng, aPeriod);
                }
            }

            foreach (Bnd_Range anItem in aListOfRng)
            {
                // OCCT changes the list's element in place; the list is not read afterwards, so a copy is the same.
                Bnd_Range aCurrRange = anItem;

                Bnd_Range aBoundR = new();
                aBoundR.Add(aUSBou[aCID, 0]);
                aBoundR.Add(aUSBou[aCID, 1]);

                if (!InscribeInterval(aUSBou[aCID, 0], aUSBou[aCID, 1], ref aCurrRange, theTol2D, aPeriod))
                {
                    // If aCurrRange does not have common block with
                    //[Ufirst, Ulast] of cylinder then we will try
                    // to inscribe [Ufirst, Ulast] in the boundaries of aCurrRange.
                    double aF = 1.0, aL = 0.0;
                    if (!aCurrRange.GetBounds(ref aF, ref aL))
                    {
                        continue;
                    }

                    if ((aL < aUSBou[aCID, 0]))
                    {
                        aCurrRange.Shift(aPeriod);
                    }
                    else if (aF > aUSBou[aCID, 1])
                    {
                        aCurrRange.Shift(-aPeriod);
                    }
                }

                aBoundR.Common(aCurrRange);

                double aDelta = aBoundR.Delta();

                if (aDelta > 0.0)
                {
                    aSumRange[aCID] += aDelta;
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
        bool isToReverse = (aSumRange[1] > aSumRange[0]);

        if (isToReverse)
        {
            WorkWithBoundaries aBoundWork = new(theQuad2,
                                                theQuad1,
                                                anEquationCoeffs2,
                                                theUVSurf2,
                                                theUVSurf1,
                                                aNbWLines,
                                                aPeriod,
                                                theTol3D,
                                                theTol2D,
                                                true);

            return CyCyNoGeometric(aCyl2,
                                   aCyl1,
                                   aBoundWork,
                                   anURange[1],
                                   aNbOfBoundaries,
                                   ref isTheEmpty,
                                   theSlin,
                                   theSPnt);
        }
        else
        {
            WorkWithBoundaries aBoundWork = new(theQuad1,
                                                theQuad2,
                                                anEquationCoeffs1,
                                                theUVSurf1,
                                                theUVSurf2,
                                                aNbWLines,
                                                aPeriod,
                                                theTol3D,
                                                theTol2D,
                                                false);

            return CyCyNoGeometric(aCyl1,
                                   aCyl2,
                                   aBoundWork,
                                   anURange[0],
                                   aNbOfBoundaries,
                                   ref isTheEmpty,
                                   theSlin,
                                   theSPnt);
        }
    }
}
