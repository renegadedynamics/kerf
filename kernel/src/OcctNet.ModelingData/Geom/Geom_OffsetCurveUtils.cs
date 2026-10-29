// Port of: src/ModelingData/TKG3d/Geom/Geom_OffsetCurveUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The point and derivatives of a 3D offset curve from those of its basis curve:
/// P(u) = p(u) + Offset * N / ||N||, where N = p'(u) ^ Direction is the local normal. An internal C++ namespace
/// in OCCT, used by Geom_OffsetCurve, GeomAdaptor_Curve and GeomGridEval.
/// </summary>
/// <remarks>
/// OCCT's function templates over the curve type are ported for the instantiations OCCT uses:
/// <see cref="AdjustDerivative(Geom_Curve, int, double, ref gp_Vec, ref gp_Vec, ref gp_Vec, ref gp_Vec, out bool)"/>
/// over <see cref="Geom_Curve"/> and <see cref="Adaptor3d_Curve"/>, and the <c>Evaluate*</c> functions over
/// <see cref="Adaptor3d_Curve"/> (GeomAdaptor_Curve's basis adaptor). Both curve types have the same evaluation
/// members, so the bodies are identical.
/// </remarks>
internal static class Geom_OffsetCurveUtils
{
    /// <summary>
    /// The offset point: theValue is the basis point on input and the offset point on output. False if the
    /// normal p' ^ Dir has a magnitude at most theTolerance.
    /// </summary>
    internal static bool CalculateD0(ref gp_Pnt theValue,
                                     in gp_Vec theD1,
                                     in gp_XYZ theDirXYZ,
                                     double theOffset,
                                     double theTolerance = gp_Resolution)
    {
        gp_XYZ Ndir = (theD1.XYZ()).Crossed(theDirXYZ);
        double R = Ndir.Modulus();
        if (R <= theTolerance)
        {
            return false;
        }
        Ndir.Multiply(theOffset / R);
        theValue.ChangeCoord().Add(Ndir);
        return true;
    }

    /// <summary>The offset point and first derivative (theValue and theD1 are the basis's on input).</summary>
    internal static bool CalculateD1(ref gp_Pnt theValue,
                                     ref gp_Vec theD1,
                                     in gp_Vec theD2,
                                     in gp_XYZ theDirXYZ,
                                     double theOffset,
                                     double theTolerance = gp_Resolution)
    {
        // P(u) = p(u) + Offset * Ndir / R
        // with R = || p' ^ V|| and Ndir = P' ^ direction (local normal direction)

        // P'(u) = p'(u) + (Offset / R**2) * (DNdir/DU * R -  Ndir * (DR/R))

        gp_XYZ Ndir = (theD1.XYZ()).Crossed(theDirXYZ);
        gp_XYZ DNdir = (theD2.XYZ()).Crossed(theDirXYZ);
        double R2 = Ndir.SquareModulus();
        double R = Math.Sqrt(R2);
        double R3 = R * R2;
        double Dr = Ndir.Dot(DNdir);
        if (R3 <= theTolerance)
        {
            if (R2 <= theTolerance)
            {
                return false;
            }
            // We try another computation but the stability is not very good.
            DNdir.Multiply(R);
            DNdir.Subtract(Ndir.Multiplied(Dr / R));
            DNdir.Multiply(theOffset / R2);
        }
        else
        {
            // Same computation as IICURV in EUCLID-IS because the stability is better
            DNdir.Multiply(theOffset / R);
            DNdir.Subtract(Ndir.Multiplied(theOffset * Dr / R3));
        }

        Ndir.Multiply(theOffset / R);
        // P(u)
        theValue.ChangeCoord().Add(Ndir);
        // P'(u)
        theD1.Add(new gp_Vec(DNdir));
        return true;
    }

    /// <summary>
    /// The offset point and first two derivatives (the basis's on input). theIsDirChange: the tangent was
    /// reversed at a singular point (see <c>AdjustDerivative</c>).
    /// </summary>
    internal static bool CalculateD2(ref gp_Pnt theValue,
                                     ref gp_Vec theD1,
                                     ref gp_Vec theD2,
                                     in gp_Vec theD3,
                                     in gp_XYZ theDirXYZ,
                                     double theOffset,
                                     bool theIsDirChange,
                                     double theTolerance = gp_Resolution)
    {
        // P(u) = p(u) + Offset * Ndir / R
        // with R = || p' ^ V|| and Ndir = P' ^ direction (local normal direction)

        // P'(u) = p'(u) + (Offset / R**2) * (DNdir/DU * R -  Ndir * (DR/R))

        // P"(u) = p"(u) + (Offset / R) * (D2Ndir/DU - DNdir * (2.0 * Dr/ R**2) +
        //         Ndir * ( (3.0 * Dr**2 / R**4) - (D2r / R**2)))

        gp_XYZ Ndir = (theD1.XYZ()).Crossed(theDirXYZ);
        gp_XYZ DNdir = (theD2.XYZ()).Crossed(theDirXYZ);
        gp_XYZ D2Ndir = (theD3.XYZ()).Crossed(theDirXYZ);
        double R2 = Ndir.SquareModulus();
        double R = Math.Sqrt(R2);
        double R3 = R2 * R;
        double R4 = R2 * R2;
        double R5 = R3 * R2;
        double Dr = Ndir.Dot(DNdir);
        double D2r = Ndir.Dot(D2Ndir) + DNdir.Dot(DNdir);

        if (R5 <= theTolerance)
        {
            if (R4 <= theTolerance)
            {
                return false;
            }
            // We try another computation but the stability is not very good
            // dixit ISG.
            //  V2 = P" (U) :
            R4 = R2 * R2;
            D2Ndir.Subtract(DNdir.Multiplied(2.0 * Dr / R2));
            D2Ndir.Add(Ndir.Multiplied(((3.0 * Dr * Dr) / R4) - (D2r / R2)));
            D2Ndir.Multiply(theOffset / R);

            // V1 = P' (U) :
            DNdir.Multiply(R);
            DNdir.Subtract(Ndir.Multiplied(Dr / R));
            DNdir.Multiply(theOffset / R2);
        }
        else
        {
            // Same computation as IICURV in EUCLID-IS because the stability is better.
            // V2 = P" (U) :
            D2Ndir.Multiply(theOffset / R);
            D2Ndir.Subtract(DNdir.Multiplied(2.0 * theOffset * Dr / R3));
            D2Ndir.Add(Ndir.Multiplied(theOffset * (((3.0 * Dr * Dr) / R5) - (D2r / R3))));

            // V1 = P' (U) :
            DNdir.Multiply(theOffset / R);
            DNdir.Subtract(Ndir.Multiplied(theOffset * Dr / R3));
        }

        Ndir.Multiply(theOffset / R);
        // P(u)
        theValue.ChangeCoord().Add(Ndir);
        // P'(u) :
        theD1.Add(new gp_Vec(DNdir));
        // P"(u) :
        if (theIsDirChange)
        {
            theD2.Reverse();
        }
        theD2.Add(new gp_Vec(D2Ndir));
        return true;
    }

    /// <summary>The offset point and first three derivatives (the basis's on input, plus its D4).</summary>
    internal static bool CalculateD3(ref gp_Pnt theValue,
                                     ref gp_Vec theD1,
                                     ref gp_Vec theD2,
                                     ref gp_Vec theD3,
                                     in gp_Vec theD4,
                                     in gp_XYZ theDirXYZ,
                                     double theOffset,
                                     bool theIsDirChange,
                                     double theTolerance = gp_Resolution)
    {
        // P(u) = p(u) + Offset * Ndir / R
        // with R = || p' ^ V|| and Ndir = P' ^ direction (local normal direction)

        // P'(u) = p'(u) + (Offset / R**2) * (DNdir/DU * R -  Ndir * (DR/R))

        // P"(u) = p"(u) + (Offset / R) * (D2Ndir/DU - DNdir * (2.0 * Dr/ R**2) +
        //         Ndir * ( (3.0 * Dr**2 / R**4) - (D2r / R**2)))

        // P"'(u) = p"'(u) + (Offset / R) * (D3Ndir - (3.0 * Dr/R**2) * D2Ndir -
        //          (3.0 * D2r / R2) * DNdir + (3.0 * Dr * Dr / R4) * DNdir -
        //          (D3r/R2) * Ndir + (6.0 * Dr * Dr / R4) * Ndir +
        //          (6.0 * Dr * D2r / R4) * Ndir - (15.0 * Dr* Dr* Dr /R6) * Ndir

        gp_XYZ Ndir = (theD1.XYZ()).Crossed(theDirXYZ);
        gp_XYZ DNdir = (theD2.XYZ()).Crossed(theDirXYZ);
        gp_XYZ D2Ndir = (theD3.XYZ()).Crossed(theDirXYZ);
        gp_XYZ D3Ndir = (theD4.XYZ()).Crossed(theDirXYZ);
        double R2 = Ndir.SquareModulus();
        double R = Math.Sqrt(R2);
        double R3 = R2 * R;
        double R4 = R2 * R2;
        double R5 = R3 * R2;
        double R6 = R3 * R3;
        double R7 = R5 * R2;
        double Dr = Ndir.Dot(DNdir);
        double D2r = Ndir.Dot(D2Ndir) + DNdir.Dot(DNdir);
        double D3r = Ndir.Dot(D3Ndir) + 3.0 * DNdir.Dot(D2Ndir);

        if (R7 <= theTolerance)
        {
            if (R6 <= theTolerance)
            {
                return false;
            }
            // OCCT: the DNdir and Ndir coefficients of V3 below don't match the derivative of V2 (see
            // occt-issues.md); V3 is right only where |p'| is constant (Dr = D2r = D3r = 0, e.g. a circle).
            // V3 = P"' (U) :
            D3Ndir.Subtract(D2Ndir.Multiplied(3.0 * Dr / R2));
            D3Ndir.Subtract(DNdir.Multiplied(3.0 * ((D2r / R2) + (Dr * Dr / R4))));
            D3Ndir.Add(
              Ndir.Multiplied(6.0 * Dr * Dr / R4 + 6.0 * Dr * D2r / R4 - 15.0 * Dr * Dr * Dr / R6 - D3r));
            D3Ndir.Multiply(theOffset / R);
            // V2 = P" (U) :
            R4 = R2 * R2;
            // OCCT: subtracts the Ndir term that CalculateD2 adds (see occt-issues.md).
            D2Ndir.Subtract(DNdir.Multiplied(2.0 * Dr / R2));
            D2Ndir.Subtract(Ndir.Multiplied((3.0 * Dr * Dr / R4) - (D2r / R2)));
            D2Ndir.Multiply(theOffset / R);
            // V1 = P' (U) :
            DNdir.Multiply(R);
            DNdir.Subtract(Ndir.Multiplied(Dr / R));
            DNdir.Multiply(theOffset / R2);
        }
        else
        {
            // OCCT: the DNdir and Ndir coefficients of V3 below don't match the derivative of V2 (see
            // occt-issues.md); V3 is right only where |p'| is constant (Dr = D2r = D3r = 0, e.g. a circle).
            // V3 = P"' (U) :
            D3Ndir.Divide(R);
            D3Ndir.Subtract(D2Ndir.Multiplied(3.0 * Dr / R3));
            D3Ndir.Subtract(DNdir.Multiplied((3.0 * ((D2r / R3) + (Dr * Dr) / R5))));
            D3Ndir.Add(
              Ndir.Multiplied(6.0 * Dr * Dr / R5 + 6.0 * Dr * D2r / R5 - 15.0 * Dr * Dr * Dr / R7 - D3r));
            D3Ndir.Multiply(theOffset);
            // V2 = P" (U) :
            D2Ndir.Divide(R);
            D2Ndir.Subtract(DNdir.Multiplied(2.0 * Dr / R3));
            // OCCT: subtracts the Ndir term that CalculateD2 adds (see occt-issues.md).
            D2Ndir.Subtract(Ndir.Multiplied((3.0 * Dr * Dr / R5) - (D2r / R3)));
            D2Ndir.Multiply(theOffset);
            // V1 = P' (U) :
            DNdir.Multiply(theOffset / R);
            DNdir.Subtract(Ndir.Multiplied(theOffset * Dr / R3));
        }

        Ndir.Multiply(theOffset / R);
        // P(u)
        theValue.ChangeCoord().Add(Ndir);
        // P'(u) :
        theD1.Add(new gp_Vec(DNdir));
        // P"(u)
        theD2.Add(new gp_Vec(D2Ndir));
        // P"'(u)
        if (theIsDirChange)
        {
            theD3.Reverse();
        }
        theD3.Add(new gp_Vec(D3Ndir));
        return true;
    }

    // gp::Resolution() as a constant, for the default arguments.
    private const double gp_Resolution = RealSmallValue; // gp::Resolution()

    /// <summary>
    /// Replaces the derivatives at a singular point (first derivative nearly zero) by the first non-null higher
    /// derivatives (Taylor series), oriented by the chord around theU. theMaxDerivative is 3 or 4.
    /// </summary>
    internal static bool AdjustDerivative(Geom_Curve theCurve,
                                          int theMaxDerivative,
                                          double theU,
                                          ref gp_Vec theD1,
                                          ref gp_Vec theD2,
                                          ref gp_Vec theD3,
                                          ref gp_Vec theD4,
                                          out bool theIsDirectionChange)
    {
        double aTol = gp.Resolution();
        const double aMinStep = 1e-7;
        const int aMaxDerivOrder = 3;

        theIsDirectionChange = false;
        double anUinfium = theCurve.FirstParameter();
        double anUsupremum = theCurve.LastParameter();

        const double DivisionFactor = 1.0e-3;
        double du;
        if ((anUsupremum >= RealLast()) || (anUinfium <= RealFirst()))
        {
            du = 0.0;
        }
        else
        {
            du = anUsupremum - anUinfium;
        }

        double aDelta = Max(du * DivisionFactor, aMinStep);

        // Derivative is approximated by Taylor-series
        int anIndex = 1; // Derivative order
        gp_Vec V;

        do
        {
            V = theCurve.EvalDN(theU, ++anIndex);
        } while ((V.SquareMagnitude() <= aTol) && anIndex < aMaxDerivOrder);

        double u;

        if (theU - anUinfium < aDelta)
        {
            u = theU + aDelta;
        }
        else
        {
            u = theU - aDelta;
        }

        gp_Pnt aP1;
        gp_Pnt aP2;
        aP1 = theCurve.EvalD0(Min(theU, u));
        aP2 = theCurve.EvalD0(Max(theU, u));

        gp_Vec V1 = new gp_Vec(aP1, aP2);
        theIsDirectionChange = V.Dot(V1) < 0.0;
        double aSign = theIsDirectionChange ? -1.0 : 1.0;

        theD1 = V * aSign;
        for (int i = 1; i < theMaxDerivative; i++)
        {
            gp_Vec aDN = theCurve.EvalDN(theU, anIndex + i);
            // C++: *(aDeriv[i - 1]) = aDN * aSign, with aDeriv = {&theD2, &theD3, &theD4}
            switch (i)
            {
                case 1:
                    theD2 = aDN * aSign;
                    break;
                case 2:
                    theD3 = aDN * aSign;
                    break;
                default:
                    theD4 = aDN * aSign;
                    break;
            }
        }

        return true;
    }

    /// <inheritdoc cref="AdjustDerivative(Geom_Curve, int, double, ref gp_Vec, ref gp_Vec, ref gp_Vec, ref gp_Vec, out bool)"/>
    internal static bool AdjustDerivative(Adaptor3d_Curve theCurve,
                                          int theMaxDerivative,
                                          double theU,
                                          ref gp_Vec theD1,
                                          ref gp_Vec theD2,
                                          ref gp_Vec theD3,
                                          ref gp_Vec theD4,
                                          out bool theIsDirectionChange)
    {
        double aTol = gp.Resolution();
        const double aMinStep = 1e-7;
        const int aMaxDerivOrder = 3;

        theIsDirectionChange = false;
        double anUinfium = theCurve.FirstParameter();
        double anUsupremum = theCurve.LastParameter();

        const double DivisionFactor = 1.0e-3;
        double du;
        if ((anUsupremum >= RealLast()) || (anUinfium <= RealFirst()))
        {
            du = 0.0;
        }
        else
        {
            du = anUsupremum - anUinfium;
        }

        double aDelta = Max(du * DivisionFactor, aMinStep);

        // Derivative is approximated by Taylor-series
        int anIndex = 1; // Derivative order
        gp_Vec V;

        do
        {
            V = theCurve.EvalDN(theU, ++anIndex);
        } while ((V.SquareMagnitude() <= aTol) && anIndex < aMaxDerivOrder);

        double u;

        if (theU - anUinfium < aDelta)
        {
            u = theU + aDelta;
        }
        else
        {
            u = theU - aDelta;
        }

        gp_Pnt aP1;
        gp_Pnt aP2;
        aP1 = theCurve.EvalD0(Min(theU, u));
        aP2 = theCurve.EvalD0(Max(theU, u));

        gp_Vec V1 = new gp_Vec(aP1, aP2);
        theIsDirectionChange = V.Dot(V1) < 0.0;
        double aSign = theIsDirectionChange ? -1.0 : 1.0;

        theD1 = V * aSign;
        for (int i = 1; i < theMaxDerivative; i++)
        {
            gp_Vec aDN = theCurve.EvalDN(theU, anIndex + i);
            switch (i)
            {
                case 1:
                    theD2 = aDN * aSign;
                    break;
                case 2:
                    theD3 = aDN * aSign;
                    break;
                default:
                    theD4 = aDN * aSign;
                    break;
            }
        }

        return true;
    }

    /// <summary>The offset point of the basis curve at theU; false if the normal is undefined.</summary>
    internal static bool EvaluateD0(double theU,
                                    Adaptor3d_Curve theBasisCurve,
                                    in gp_Dir theDir,
                                    double theOffset,
                                    out gp_Pnt theValue)
    {
        Geom_Curve.ResD1 aD1 = theBasisCurve.EvalD1(theU);
        theValue = aD1.Point;
        return CalculateD0(ref theValue, aD1.D1, theDir.XYZ(), theOffset);
    }

    /// <summary>The offset point and first derivative at theU.</summary>
    internal static bool EvaluateD1(double theU,
                                    Adaptor3d_Curve theBasisCurve,
                                    in gp_Dir theDir,
                                    double theOffset,
                                    out gp_Pnt theValue,
                                    out gp_Vec theD1)
    {
        Geom_Curve.ResD2 aD2 = theBasisCurve.EvalD2(theU);
        theValue = aD2.Point;
        theD1 = aD2.D1;
        return CalculateD1(ref theValue, ref theD1, aD2.D2, theDir.XYZ(), theOffset);
    }

    /// <summary>The offset point and first two derivatives at theU (singular points handled).</summary>
    internal static bool EvaluateD2(double theU,
                                    Adaptor3d_Curve theBasisCurve,
                                    in gp_Dir theDir,
                                    double theOffset,
                                    out gp_Pnt theValue,
                                    out gp_Vec theD1,
                                    out gp_Vec theD2)
    {
        Geom_Curve.ResD3 aD3Result = theBasisCurve.EvalD3(theU);
        theValue = aD3Result.Point;
        theD1 = aD3Result.D1;
        theD2 = aD3Result.D2;
        gp_Vec aD3 = aD3Result.D3;

        bool isDirectionChange = false;
        if (theD1.SquareMagnitude() <= gp.Resolution())
        {
            gp_Vec aDummyD4 = new gp_Vec();
            if (!AdjustDerivative(theBasisCurve, 3, theU, ref theD1, ref theD2, ref aD3, ref aDummyD4, out isDirectionChange))
            {
                return false;
            }
        }

        return CalculateD2(ref theValue, ref theD1, ref theD2, aD3, theDir.XYZ(), theOffset, isDirectionChange);
    }

    /// <summary>The offset point and first three derivatives at theU (singular points handled).</summary>
    internal static bool EvaluateD3(double theU,
                                    Adaptor3d_Curve theBasisCurve,
                                    in gp_Dir theDir,
                                    double theOffset,
                                    out gp_Pnt theValue,
                                    out gp_Vec theD1,
                                    out gp_Vec theD2,
                                    out gp_Vec theD3)
    {
        Geom_Curve.ResD3 aD3Result = theBasisCurve.EvalD3(theU);
        theValue = aD3Result.Point;
        theD1 = aD3Result.D1;
        theD2 = aD3Result.D2;
        theD3 = aD3Result.D3;
        gp_Vec aD4 = theBasisCurve.EvalDN(theU, 4);

        bool isDirectionChange = false;
        if (theD1.SquareMagnitude() <= gp.Resolution())
        {
            if (!AdjustDerivative(theBasisCurve, 4, theU, ref theD1, ref theD2, ref theD3, ref aD4, out isDirectionChange))
            {
                return false;
            }
        }

        return CalculateD3(ref theValue,
                           ref theD1,
                           ref theD2,
                           ref theD3,
                           aD4,
                           theDir.XYZ(),
                           theOffset,
                           isDirectionChange);
    }

    /// <summary>
    /// The derivative of order theN (at least 1) at theU; above 3, the basis curve's (the offset contribution is
    /// neglected, as in OCCT).
    /// </summary>
    internal static bool EvaluateDN(double theU,
                                    Adaptor3d_Curve theBasisCurve,
                                    in gp_Dir theDir,
                                    double theOffset,
                                    int theN,
                                    out gp_Vec theDN)
    {
        gp_Pnt aPnt;
        gp_Vec aDummy;
        switch (theN)
        {
            case 1:
                return EvaluateD1(theU, theBasisCurve, theDir, theOffset, out aPnt, out theDN);
            case 2:
                return EvaluateD2(theU, theBasisCurve, theDir, theOffset, out aPnt, out aDummy, out theDN);
            case 3:
                return EvaluateD3(theU, theBasisCurve, theDir, theOffset, out aPnt, out aDummy, out aDummy, out theDN);
            default:
            {
                // For derivatives > 3, return basis curve derivative (no offset contribution)
                theDN = theBasisCurve.EvalDN(theU, theN);
                return true;
            }
        }
    }
}
