// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_OffsetCurveUtils.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Point and derivatives of a 2D offset curve, P(u) = p(u) + Offset * N / ||N|| with N = (p'(u).Y, -p'(u).X),
/// from the derivatives of the basis curve. Used by Geom2d_OffsetCurve, Adaptor2d_OffsetCurve and (later)
/// Geom2dAdaptor_Curve and Geom2dGridEval. An internal C++ namespace in OCCT.
/// </summary>
/// <remarks>
/// OCCT's templates over the curve type (<c>AdjustDerivative</c>, <c>EvaluateD0</c> ... <c>EvaluateDN</c>) are
/// ported once per type OCCT instantiates them with: Geom2d_Curve and Adaptor2d_Curve2d.
/// </remarks>
internal static class Geom2d_OffsetCurveUtils
{
    /// <summary>
    /// The offset point: theValue is the basis point on input and the offset point on output. False if theD1 is
    /// null (theValue then unchanged).
    /// </summary>
    internal static bool CalculateD0(ref gp_Pnt2d theValue, in gp_Vec2d theD1, double theOffset)
    {
        if (theD1.SquareMagnitude() <= gp.Resolution())
        {
            return false;
        }

        gp_Dir2d aNormal = new gp_Dir2d(theD1.Y(), -theD1.X());
        theValue.ChangeCoord().Add(aNormal.XY() * theOffset);
        return true;
    }

    /// <summary>The offset point and first derivative, from the basis point, D1 and D2 (updated in place).</summary>
    internal static bool CalculateD1(ref gp_Pnt2d theValue,
                                     ref gp_Vec2d theD1,
                                     in gp_Vec2d theD2,
                                     double theOffset)
    {
        // P(u) = p(u) + Offset * Ndir / R
        // with R = || p' ^ Z|| and Ndir = P' ^ Z

        // P'(u) = p'(u) + (Offset / R**2) * (DNdir/DU * R -  Ndir * (DR/R))

        gp_XY Ndir = new gp_XY(theD1.Y(), -theD1.X());
        gp_XY DNdir = new gp_XY(theD2.Y(), -theD2.X());
        double R2 = Ndir.SquareModulus();
        double R = Math.Sqrt(R2);
        double R3 = R * R2;
        double Dr = Ndir.Dot(DNdir);
        if (R3 <= gp.Resolution())
        {
            if (R2 <= gp.Resolution())
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
        theD1.Add(new gp_Vec2d(DNdir));
        return true;
    }

    /// <summary>
    /// The offset point and first two derivatives, from the basis point, D1, D2 (updated in place) and D3.
    /// theIsDirChange reverses D2 (set by <see cref="AdjustDerivative(Geom2d_Curve, int, double, ref gp_Vec2d, ref gp_Vec2d, ref gp_Vec2d, ref gp_Vec2d, out bool)"/>).
    /// </summary>
    internal static bool CalculateD2(ref gp_Pnt2d theValue,
                                     ref gp_Vec2d theD1,
                                     ref gp_Vec2d theD2,
                                     in gp_Vec2d theD3,
                                     bool theIsDirChange,
                                     double theOffset)
    {
        // P(u) = p(u) + Offset * Ndir / R
        // with R = || p' ^ Z|| and Ndir = P' ^ Z

        // P'(u) = p'(u) + (Offset / R**2) * (DNdir/DU * R -  Ndir * (DR/R))

        // P"(u) = p"(u) + (Offset / R) * (D2Ndir/DU - DNdir * (2.0 * Dr/ R**2) +
        //         Ndir * ( (3.0 * Dr**2 / R**4) - (D2r / R**2)))

        gp_XY Ndir = new gp_XY(theD1.Y(), -theD1.X());
        gp_XY DNdir = new gp_XY(theD2.Y(), -theD2.X());
        gp_XY D2Ndir = new gp_XY(theD3.Y(), -theD3.X());
        double R2 = Ndir.SquareModulus();
        double R = Math.Sqrt(R2);
        double R3 = R2 * R;
        double R4 = R2 * R2;
        double R5 = R3 * R2;
        double Dr = Ndir.Dot(DNdir);
        double D2r = Ndir.Dot(D2Ndir) + DNdir.Dot(DNdir);
        if (R5 <= gp.Resolution())
        {
            if (R4 <= gp.Resolution())
            {
                return false;
            }
            // We try another computation but the stability is not very good dixit ISG.
            //  V2 = P" (U) :
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

            // V1 = P' (U)
            DNdir.Multiply(theOffset / R);
            DNdir.Subtract(Ndir.Multiplied(theOffset * Dr / R3));
        }

        Ndir.Multiply(theOffset / R);
        // P(u)
        theValue.ChangeCoord().Add(Ndir);
        // P'(u) :
        theD1.Add(new gp_Vec2d(DNdir));
        // P"(u) :
        if (theIsDirChange)
        {
            theD2.Reverse();
        }
        theD2.Add(new gp_Vec2d(D2Ndir));
        return true;
    }

    /// <summary>
    /// The offset point and first three derivatives, from the basis point, D1, D2, D3 (updated in place) and D4.
    /// theIsDirChange reverses D3.
    /// </summary>
    internal static bool CalculateD3(ref gp_Pnt2d theValue,
                                     ref gp_Vec2d theD1,
                                     ref gp_Vec2d theD2,
                                     ref gp_Vec2d theD3,
                                     in gp_Vec2d theD4,
                                     bool theIsDirChange,
                                     double theOffset)
    {
        // P(u) = p(u) + Offset * Ndir / R
        // with R = || p' ^ Z|| and Ndir = P' ^ Z

        // P'(u)  = p'(u) + (Offset / R**2) * (DNdir/DU * R -  Ndir * (DR/R))

        // P"(u)  = p"(u) + (Offset / R) * (D2Ndir/DU - DNdir * (2.0 * Dr/ R**2) +
        //          Ndir * ( (3.0 * Dr**2 / R**4) - (D2r / R**2)))

        // P"'(u) = p"'(u) + (Offset / R) * (D3Ndir - (3.0 * Dr/R**2 ) * D2Ndir -
        //          (3.0 * D2r / R2) * DNdir) + (3.0 * Dr * Dr / R4) * DNdir -
        //          (D3r/R2) * Ndir + (6.0 * Dr * Dr / R4) * Ndir +
        //          (6.0 * Dr * D2r / R4) * Ndir - (15.0 * Dr* Dr* Dr /R6) * Ndir

        gp_XY Ndir = new gp_XY(theD1.Y(), -theD1.X());
        gp_XY DNdir = new gp_XY(theD2.Y(), -theD2.X());
        gp_XY D2Ndir = new gp_XY(theD3.Y(), -theD3.X());
        gp_XY D3Ndir = new gp_XY(theD4.Y(), -theD4.X());
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

        if (R7 <= gp.Resolution())
        {
            if (R6 <= gp.Resolution())
            {
                return false;
            }
            // We try another computation but the stability is not very good dixit ISG.
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
            // Same computation as IICURV in EUCLID-IS because the stability is better.
            // OCCT: the DNdir and Ndir coefficients of V3 below don't match the derivative of V2 (see
            // occt-issues.md); V3 is right only where |p'| is constant (Dr = D2r = D3r = 0, e.g. a circle).
            // V3 = P"' (U) :
            D3Ndir.Multiply(theOffset / R);
            D3Ndir.Subtract(D2Ndir.Multiplied(3.0 * theOffset * Dr / R3));
            D3Ndir.Subtract(DNdir.Multiplied(((3.0 * theOffset) * ((D2r / R3) + (Dr * Dr) / R5))));
            D3Ndir.Add(Ndir.Multiplied(
                (theOffset * (6.0 * Dr * Dr / R5 + 6.0 * Dr * D2r / R5 - 15.0 * Dr * Dr * Dr / R7 - D3r))));
            // V2 = P" (U) :
            D2Ndir.Multiply(theOffset / R);
            D2Ndir.Subtract(DNdir.Multiplied(2.0 * theOffset * Dr / R3));
            // OCCT: subtracts the Ndir term that CalculateD2 adds (see occt-issues.md).
            D2Ndir.Subtract(Ndir.Multiplied(theOffset * (((3.0 * Dr * Dr) / R5) - (D2r / R3))));
            // V1 = P' (U) :
            DNdir.Multiply(theOffset / R);
            DNdir.Subtract(Ndir.Multiplied(theOffset * Dr / R3));
        }

        Ndir.Multiply(theOffset / R);
        // P(u)
        theValue.ChangeCoord().Add(Ndir);
        // P'(u) :
        theD1.Add(new gp_Vec2d(DNdir));
        // P"(u)
        theD2.Add(new gp_Vec2d(D2Ndir));
        // P"'(u)
        if (theIsDirChange)
        {
            theD3.Reverse();
        }
        theD3.Add(new gp_Vec2d(D3Ndir));
        return true;
    }

    /// <summary>
    /// Replaces null first derivatives at a singular point by the first non-null higher derivative (Taylor
    /// series), oriented along the curve; theIsDirectionChange tells whether it had to be reversed. Computes
    /// theMaxDerivative (3 or 4) derivatives.
    /// </summary>
    internal static bool AdjustDerivative(Geom2d_Curve theCurve,
                                          int theMaxDerivative,
                                          double theU,
                                          ref gp_Vec2d theD1,
                                          ref gp_Vec2d theD2,
                                          ref gp_Vec2d theD3,
                                          ref gp_Vec2d theD4,
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
        gp_Vec2d V;

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

        gp_Pnt2d aP1;
        gp_Pnt2d aP2;
        aP1 = theCurve.EvalD0(Min(theU, u));
        aP2 = theCurve.EvalD0(Max(theU, u));

        gp_Vec2d V1 = new gp_Vec2d(aP1, aP2);
        theIsDirectionChange = V.Dot(V1) < 0.0;
        double aSign = theIsDirectionChange ? -1.0 : 1.0;

        theD1 = V * aSign;
        for (int i = 1; i < theMaxDerivative; i++)
        {
            gp_Vec2d aDN = theCurve.EvalDN(theU, anIndex + i);
            switch (i - 1)
            {
                case 0: theD2 = aDN * aSign; break;
                case 1: theD3 = aDN * aSign; break;
                default: theD4 = aDN * aSign; break;
            }
        }

        return true;
    }

    /// <summary>The adaptor instantiation of <see cref="AdjustDerivative(Geom2d_Curve, int, double, ref gp_Vec2d, ref gp_Vec2d, ref gp_Vec2d, ref gp_Vec2d, out bool)"/>.</summary>
    internal static bool AdjustDerivative(Adaptor2d_Curve2d theCurve,
                                          int theMaxDerivative,
                                          double theU,
                                          ref gp_Vec2d theD1,
                                          ref gp_Vec2d theD2,
                                          ref gp_Vec2d theD3,
                                          ref gp_Vec2d theD4,
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
        gp_Vec2d V;

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

        gp_Pnt2d aP1;
        gp_Pnt2d aP2;
        aP1 = theCurve.EvalD0(Min(theU, u));
        aP2 = theCurve.EvalD0(Max(theU, u));

        gp_Vec2d V1 = new gp_Vec2d(aP1, aP2);
        theIsDirectionChange = V.Dot(V1) < 0.0;
        double aSign = theIsDirectionChange ? -1.0 : 1.0;

        theD1 = V * aSign;
        for (int i = 1; i < theMaxDerivative; i++)
        {
            gp_Vec2d aDN = theCurve.EvalDN(theU, anIndex + i);
            switch (i - 1)
            {
                case 0: theD2 = aDN * aSign; break;
                case 1: theD3 = aDN * aSign; break;
                default: theD4 = aDN * aSign; break;
            }
        }

        return true;
    }

    // EvaluateD0 ... EvaluateDN: the Geom2d_Curve instantiation.

    /// <summary>The offset point at theU from the basis curve's D1; false if the normal is undefined.</summary>
    internal static bool EvaluateD0(double theU, Geom2d_Curve theBasisCurve, double theOffset, out gp_Pnt2d theValue)
    {
        Geom2d_Curve.ResD1 aD1 = theBasisCurve.EvalD1(theU);
        theValue = aD1.Point;
        return CalculateD0(ref theValue, aD1.D1, theOffset);
    }

    /// <summary>The offset point and D1 at theU from the basis curve's D2; false if the computation failed.</summary>
    internal static bool EvaluateD1(double theU,
                                    Geom2d_Curve theBasisCurve,
                                    double theOffset,
                                    out gp_Pnt2d theValue,
                                    out gp_Vec2d theD1)
    {
        Geom2d_Curve.ResD2 aD2 = theBasisCurve.EvalD2(theU);
        theValue = aD2.Point;
        theD1 = aD2.D1;
        return CalculateD1(ref theValue, ref theD1, aD2.D2, theOffset);
    }

    /// <summary>The offset point, D1 and D2 at theU (handling a singular basis point); false if the computation failed.</summary>
    internal static bool EvaluateD2(double theU,
                                    Geom2d_Curve theBasisCurve,
                                    double theOffset,
                                    out gp_Pnt2d theValue,
                                    out gp_Vec2d theD1,
                                    out gp_Vec2d theD2)
    {
        Geom2d_Curve.ResD3 aD3Result = theBasisCurve.EvalD3(theU);
        theValue = aD3Result.Point;
        theD1 = aD3Result.D1;
        theD2 = aD3Result.D2;
        gp_Vec2d aD3 = aD3Result.D3;

        bool isDirectionChange = false;
        if (theD1.SquareMagnitude() <= gp.Resolution())
        {
            gp_Vec2d aDummyD4 = new gp_Vec2d();
            if (!AdjustDerivative(theBasisCurve, 3, theU, ref theD1, ref theD2, ref aD3, ref aDummyD4, out isDirectionChange))
            {
                return false;
            }
        }

        return CalculateD2(ref theValue, ref theD1, ref theD2, aD3, isDirectionChange, theOffset);
    }

    /// <summary>The offset point and D1 to D3 at theU (handling a singular basis point); false if the computation failed.</summary>
    internal static bool EvaluateD3(double theU,
                                    Geom2d_Curve theBasisCurve,
                                    double theOffset,
                                    out gp_Pnt2d theValue,
                                    out gp_Vec2d theD1,
                                    out gp_Vec2d theD2,
                                    out gp_Vec2d theD3)
    {
        Geom2d_Curve.ResD3 aD3Result = theBasisCurve.EvalD3(theU);
        theValue = aD3Result.Point;
        theD1 = aD3Result.D1;
        theD2 = aD3Result.D2;
        theD3 = aD3Result.D3;
        gp_Vec2d aD4 = theBasisCurve.EvalDN(theU, 4);

        bool isDirectionChange = false;
        if (theD1.SquareMagnitude() <= gp.Resolution())
        {
            if (!AdjustDerivative(theBasisCurve, 4, theU, ref theD1, ref theD2, ref theD3, ref aD4, out isDirectionChange))
            {
                return false;
            }
        }

        return CalculateD3(ref theValue, ref theD1, ref theD2, ref theD3, aD4, isDirectionChange, theOffset);
    }

    /// <summary>
    /// The Nth derivative of the offset curve at theU: through EvaluateD1 to D3 up to N = 3, the basis curve's
    /// derivative above (OCCT neglects the offset's contribution there).
    /// </summary>
    internal static bool EvaluateDN(double theU, Geom2d_Curve theBasisCurve, double theOffset, int theN, out gp_Vec2d theDN)
    {
        gp_Pnt2d aPnt;
        gp_Vec2d aDummy;
        switch (theN)
        {
            case 1:
                return EvaluateD1(theU, theBasisCurve, theOffset, out aPnt, out theDN);
            case 2:
                return EvaluateD2(theU, theBasisCurve, theOffset, out aPnt, out aDummy, out theDN);
            case 3:
                return EvaluateD3(theU, theBasisCurve, theOffset, out aPnt, out aDummy, out aDummy, out theDN);
            default:
            {
                // For derivatives > 3, return basis curve derivative (no offset contribution)
                theDN = theBasisCurve.EvalDN(theU, theN);
                return true;
            }
        }
    }

    // EvaluateD0 ... EvaluateDN: the Adaptor2d_Curve2d instantiation.

    /// <summary>The offset point at theU from the basis adaptor's D1; false if the normal is undefined.</summary>
    internal static bool EvaluateD0(double theU, Adaptor2d_Curve2d theBasisCurve, double theOffset, out gp_Pnt2d theValue)
    {
        Geom2d_Curve.ResD1 aD1 = theBasisCurve.EvalD1(theU);
        theValue = aD1.Point;
        return CalculateD0(ref theValue, aD1.D1, theOffset);
    }

    /// <summary>The offset point and D1 at theU from the basis adaptor's D2.</summary>
    internal static bool EvaluateD1(double theU,
                                    Adaptor2d_Curve2d theBasisCurve,
                                    double theOffset,
                                    out gp_Pnt2d theValue,
                                    out gp_Vec2d theD1)
    {
        Geom2d_Curve.ResD2 aD2 = theBasisCurve.EvalD2(theU);
        theValue = aD2.Point;
        theD1 = aD2.D1;
        return CalculateD1(ref theValue, ref theD1, aD2.D2, theOffset);
    }

    /// <summary>The offset point, D1 and D2 at theU from the basis adaptor (handling a singular basis point).</summary>
    internal static bool EvaluateD2(double theU,
                                    Adaptor2d_Curve2d theBasisCurve,
                                    double theOffset,
                                    out gp_Pnt2d theValue,
                                    out gp_Vec2d theD1,
                                    out gp_Vec2d theD2)
    {
        Geom2d_Curve.ResD3 aD3Result = theBasisCurve.EvalD3(theU);
        theValue = aD3Result.Point;
        theD1 = aD3Result.D1;
        theD2 = aD3Result.D2;
        gp_Vec2d aD3 = aD3Result.D3;

        bool isDirectionChange = false;
        if (theD1.SquareMagnitude() <= gp.Resolution())
        {
            gp_Vec2d aDummyD4 = new gp_Vec2d();
            if (!AdjustDerivative(theBasisCurve, 3, theU, ref theD1, ref theD2, ref aD3, ref aDummyD4, out isDirectionChange))
            {
                return false;
            }
        }

        return CalculateD2(ref theValue, ref theD1, ref theD2, aD3, isDirectionChange, theOffset);
    }

    /// <summary>The offset point and D1 to D3 at theU from the basis adaptor (handling a singular basis point).</summary>
    internal static bool EvaluateD3(double theU,
                                    Adaptor2d_Curve2d theBasisCurve,
                                    double theOffset,
                                    out gp_Pnt2d theValue,
                                    out gp_Vec2d theD1,
                                    out gp_Vec2d theD2,
                                    out gp_Vec2d theD3)
    {
        Geom2d_Curve.ResD3 aD3Result = theBasisCurve.EvalD3(theU);
        theValue = aD3Result.Point;
        theD1 = aD3Result.D1;
        theD2 = aD3Result.D2;
        theD3 = aD3Result.D3;
        gp_Vec2d aD4 = theBasisCurve.EvalDN(theU, 4);

        bool isDirectionChange = false;
        if (theD1.SquareMagnitude() <= gp.Resolution())
        {
            if (!AdjustDerivative(theBasisCurve, 4, theU, ref theD1, ref theD2, ref theD3, ref aD4, out isDirectionChange))
            {
                return false;
            }
        }

        return CalculateD3(ref theValue, ref theD1, ref theD2, ref theD3, aD4, isDirectionChange, theOffset);
    }

    /// <summary>The Nth derivative of the offset of the basis adaptor at theU (see the Geom2d_Curve overload).</summary>
    internal static bool EvaluateDN(double theU, Adaptor2d_Curve2d theBasisCurve, double theOffset, int theN, out gp_Vec2d theDN)
    {
        gp_Pnt2d aPnt;
        gp_Vec2d aDummy;
        switch (theN)
        {
            case 1:
                return EvaluateD1(theU, theBasisCurve, theOffset, out aPnt, out theDN);
            case 2:
                return EvaluateD2(theU, theBasisCurve, theOffset, out aPnt, out aDummy, out theDN);
            case 3:
                return EvaluateD3(theU, theBasisCurve, theOffset, out aPnt, out aDummy, out aDummy, out theDN);
            default:
            {
                // For derivatives > 3, return basis curve derivative (no offset contribution)
                theDN = theBasisCurve.EvalDN(theU, theN);
                return true;
            }
        }
    }
}
