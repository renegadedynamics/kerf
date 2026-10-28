// Port of: src/FoundationClasses/TKMath/math/math_DirectPolynomialRoots.hxx
// Port of: src/FoundationClasses/TKMath/math/math_DirectPolynomialRoots.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// All the real roots of a real polynomial of degree &lt;= 4, by direct algebraic methods: Ferrari's method for
/// quartics, Cardano's formula for cubics, and numerically stable formulas for quadratics and linear equations. The
/// roots are polished by Newton-Raphson iterations.
/// </summary>
public class math_DirectPolynomialRoots
{
    // Algorithm References:
    //   - ALGORITHMES NUMERIQUES ANALYSE ET MISE EN OEUVRE, tome 2
    //     (equations et systemes non lineaires) J. VIGNES editions TECHNIP
    //   - Ferrari's method for quartic equations
    //   - Cardano's formula for cubic equations
    //   - Numerically stable quadratic formula
    //   - Newton-Raphson refinement for improved accuracy

    /// Threshold below which coefficients are considered effectively zero
    private const double ZERO_THRESHOLD = 1.0e-30;

    /// Machine epsilon for floating-point arithmetic precision
    private static readonly double MACHINE_EPSILON = RealEpsilon();

    /// Floating-point radix (base) for coefficient scaling operations
    private const double FLOATING_RADIX = 2.0;

    /// Inverse of natural logarithm of radix for exponent calculations
    private static readonly double INV_LOG_RADIX = 1.0 / Math.Log(2.0);

    /// Maximum number of Newton-Raphson refinement iterations
    private const int MAX_NEWTON_ITERATIONS = 10;

    /// Upper limit to prevent numerical overflow in computations
    private const double OVERFLOW_LIMIT = 1.0e+80;

    private bool myDone;           //!< Computation status flag
    private bool myInfiniteStatus; //!< Infinite solutions flag (0*x + 0 = 0)
    private int myNbSol;           //!< Number of real roots found (0 to 4)
    private readonly double[] myRoots = new double[4]; //!< Array storing computed real roots

    /// <summary>The real roots of Ax^4 + Bx^3 + Cx^2 + Dx + E = 0 (Ferrari's method).</summary>
    public math_DirectPolynomialRoots(double theA, double theB, double theC, double theD, double theE)
    {
        myInfiniteStatus = false;
        myDone = true;
        Solve(theA, theB, theC, theD, theE);
    }

    /// <summary>The real roots of Ax^3 + Bx^2 + Cx + D = 0 (Cardano's method with Vieta substitution).</summary>
    public math_DirectPolynomialRoots(double theA, double theB, double theC, double theD)
    {
        myDone = true;
        myInfiniteStatus = false;
        Solve(theA, theB, theC, theD);
    }

    /// <summary>The real roots of Ax^2 + Bx + C = 0.</summary>
    public math_DirectPolynomialRoots(double theA, double theB, double theC)
    {
        myDone = true;
        myInfiniteStatus = false;
        Solve(theA, theB, theC);
    }

    /// <summary>The real root of Ax + B = 0 (none if A = 0, B != 0; infinitely many if A = B = 0).</summary>
    public math_DirectPolynomialRoots(double theA, double theB)
    {
        myDone = true;
        myInfiniteStatus = false;
        Solve(theA, theB);
    }

    /// <summary>True if the computations succeeded.</summary>
    public bool IsDone() => myDone;

    /// <summary>True if there are infinitely many roots (only for 0*x + 0 = 0).</summary>
    public bool InfiniteRoots() => myInfiniteStatus;

    /// <summary>The number of real roots, each counted with its multiplicity. Throws <see cref="StdFail_InfiniteSolutions"/> if infinitely many.</summary>
    public int NbSolutions()
    {
        StdFail_InfiniteSolutions_Raise_if(myInfiniteStatus, " ");
        return myNbSol;
    }

    /// <summary>The root number <paramref name="theIndex"/>, in 1..NbSolutions.</summary>
    public double Value(int theIndex)
    {
        StdFail_InfiniteSolutions_Raise_if(myInfiniteStatus, " ");
        Standard_RangeError_Raise_if((theIndex < 1) || (theIndex > myNbSol), " ");
        return myRoots[theIndex - 1];
    }

    public void Dump(TextWriter theStream)
    {
        theStream.Write("math_DirectPolynomialRoots ");

        if (!myDone)
        {
            theStream.Write(" Not Done \n");
        }
        else if (myInfiniteStatus)
        {
            theStream.Write(" Status = Infinity Roots \n");
        }
        else
        {
            theStream.Write(" Status = Not Infinity Roots \n");
            theStream.Write(" Number of solutions = " + myNbSol + "\n");
            for (int i = 1; i <= myNbSol; i++)
            {
                theStream.Write(" Solution number " + i + " = " + myRoots[i - 1].ToString(CultureInfo.InvariantCulture) + "\n");
            }
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }

    /// <summary>Solves the quartic Ax^4 + Bx^3 + Cx^2 + Dx + E = 0 by Ferrari's method.</summary>
    protected void Solve(double theA, double theB, double theC, double theD, double theE)
    {
        // Check for degree reduction
        if (ShouldReduceDegreeQuartic(theA, theB, theC, theD, theE))
        {
            Solve(theB, theC, theD, theE);
            return;
        }

        // Normalize coefficients
        double A = theB / theA;
        double B = theC / theA;
        double C = theD / theA;
        double D = theE / theA;

        // Scale coefficients to avoid overflow/underflow
        ScaledCoefficients aScaled = default;
        aScaled.ScaleQuartic(A, B, C, D, D);

        // Solve Ferrari's resolvent cubic
        bool aSuccess = false;
        double aY0 = SolveFerrariResolvent(aScaled.A, aScaled.B, aScaled.C, aScaled.D, ref aSuccess);

        if (!aSuccess)
        {
            myDone = false;
            return;
        }

        // Factor into two quadratics
        QuarticFactorization aFactors = FactorQuarticViaFerrari(aScaled.A, aScaled.B, aScaled.C, aScaled.D, aY0);

        // Solve first quadratic
        math_DirectPolynomialRoots aQuadratic1 = new(1.0, aFactors.P1, aFactors.Q1);
        if (!aQuadratic1.IsDone())
        {
            myDone = false;
            return;
        }

        // Solve second quadratic
        math_DirectPolynomialRoots aQuadratic2 = new(1.0, aFactors.P2, aFactors.Q2);
        if (!aQuadratic2.IsDone())
        {
            myDone = false;
            return;
        }

        // Collect all roots
        myNbSol = aQuadratic1.NbSolutions() + aQuadratic2.NbSolutions();
        int aIndex = 0;

        for (int i = 0; i < aQuadratic1.NbSolutions(); ++i)
        {
            myRoots[aIndex++] = aQuadratic1.myRoots[i];
        }

        for (int i = 0; i < aQuadratic2.NbSolutions(); ++i)
        {
            myRoots[aIndex++] = aQuadratic2.myRoots[i];
        }

        // Apply inverse scaling and Newton-Raphson refinement to all roots
        ScaleAndRefineAllRoots(myRoots, myNbSol, aScaled.ScaleFactor, theA, theB, theC, theD, theE);
    }

    /// <summary>Solves the cubic Ax^3 + Bx^2 + Cx + D = 0 by Cardano's method.</summary>
    protected void Solve(double theA, double theB, double theC, double theD)
    {
        // Check for degree reduction
        if (Math.Abs(theA) <= ZERO_THRESHOLD)
        {
            Solve(theB, theC, theD);
            return;
        }

        // Normalize coefficients
        double aBeta = theB / theA;
        double aGamma = theC / theA;
        double aDel = theD / theA;

        // Scale to avoid overflow/underflow
        ScaledCoefficients aScaled = default;
        aScaled.ScaleCubic(aBeta, aGamma, aDel, aDel);

        // Transform to depressed cubic: t^3 + Pt + Q = 0
        double aP1 = aScaled.B;
        double aP2 = -(aScaled.A * aScaled.A) / 3.0;
        double aP = aP1 + aP2;
        double aEp = 5.0 * MACHINE_EPSILON * (Math.Abs(aP1) + Math.Abs(aP2));
        if (Math.Abs(aP) <= aEp)
        {
            aP = 0.0;
        }

        double aQ1 = aScaled.C;
        double aQ2 = -aScaled.A * aScaled.B / 3.0;
        double aQ3 = 2.0 * (aScaled.A * aScaled.A * aScaled.A) / 27.0;
        double aQ = aQ1 + aQ2 + aQ3;
        double aEq = 10.0 * MACHINE_EPSILON * (Math.Abs(aQ1) + Math.Abs(aQ2) + Math.Abs(aQ3));
        if (Math.Abs(aQ) <= aEq)
        {
            aQ = 0.0;
        }

        // Check for overflow
        if (Math.Abs(aP) > OVERFLOW_LIMIT)
        {
            myDone = false;
            return;
        }

        // Compute discriminant
        double aA1 = (aP * aP * aP) / 27.0;
        double aA2 = (aQ * aQ) / 4.0;
        double aDiscr = aA1 + aA2;

        // Special handling for P < 0
        if (aP < 0.0)
        {
            aDiscr = ComputeSpecialDiscriminant(aScaled.A, aScaled.B, aScaled.C, aA1);
        }

        // Solve based on discriminant
        if (aDiscr < 0.0)
        {
            // Three distinct real roots
            myNbSol = 3;
            SolveCubicThreeRealRoots(aScaled.A, aScaled.B, aScaled.C, aP, aQ, aDiscr, myRoots);
        }
        else if (aDiscr > 0.0)
        {
            // One real root
            myNbSol = 1;
            SolveCubicOneRealRoot(aScaled.A, aScaled.C, aP, aQ, aDiscr, myRoots);
        }
        else
        {
            // Multiple roots
            SolveCubicMultipleRoots(aScaled.A, aScaled.B, aScaled.C, aP, aQ, myRoots, out myNbSol);
        }

        // Apply inverse scaling and Newton-Raphson refinement to all roots
        ScaleAndRefineAllRoots(myRoots, myNbSol, aScaled.ScaleFactor, theA, theB, theC, theD);
    }

    /// <summary>Solves the quadratic Ax^2 + Bx + C = 0 with numerically stable formulas.</summary>
    protected void Solve(double theA, double theB, double theC)
    {
        // Check for degree reduction
        if (Math.Abs(theA) <= ZERO_THRESHOLD)
        {
            Solve(theB, theC);
            return;
        }

        // Solve normalized quadratic x^2 + P*x + Q = 0
        double P = theB / theA;
        double Q = theC / theA;

        // Compute discriminant with error bounds
        double aEpsD = 3.0 * MACHINE_EPSILON * (P * P + Math.Abs(4.0 * Q));
        double aDiscrim = P * P - 4.0 * Q;

        if (Math.Abs(aDiscrim) <= aEpsD)
        {
            aDiscrim = 0.0;
        }

        if (aDiscrim < 0.0)
        {
            // No real roots
            myNbSol = 0;
        }
        else if (aDiscrim == 0.0)
        {
            // Double root
            myNbSol = 2;
            myRoots[0] = -0.5 * P;
            myRoots[0] = RefinePolynomialRoot(myRoots[0], 1.0, P, Q);
            myRoots[1] = myRoots[0];
        }
        else
        {
            // Two distinct real roots - use numerically stable formula
            myNbSol = 2;
            if (P > 0.0)
            {
                myRoots[0] = -(P + Math.Sqrt(aDiscrim)) / 2.0;
            }
            else
            {
                myRoots[0] = -(P - Math.Sqrt(aDiscrim)) / 2.0;
            }

            myRoots[0] = RefinePolynomialRoot(myRoots[0], 1.0, P, Q);
            myRoots[1] = Q / myRoots[0];
            myRoots[1] = RefinePolynomialRoot(myRoots[1], 1.0, P, Q);
        }
    }

    /// <summary>Solves the linear equation Ax + B = 0.</summary>
    protected void Solve(double theA, double theB)
    {
        if (Math.Abs(theA) <= ZERO_THRESHOLD)
        {
            if (Math.Abs(theB) <= ZERO_THRESHOLD)
            {
                // 0 = 0: infinite solutions
                myInfiniteStatus = true;
                return;
            }

            // 0*x + B = 0: no solution
            myNbSol = 0;
            return;
        }

        // Normal case: unique solution
        myNbSol = 1;
        myRoots[0] = -theB / theA;
    }

    // The anonymous-namespace helpers of math_DirectPolynomialRoots.cxx.

    // Evaluates polynomial at point X using Horner's method
    // Coefficients: Poly[0]*X^(N-1) + Poly[1]*X^(N-2) + ... + Poly[N-1]
    private static double EvaluatePolynomial(int theN, ReadOnlySpan<double> thePoly, double theX)
    {
        double aResult = thePoly[0];
        for (int i = 1; i < theN; ++i)
        {
            aResult = aResult * theX + thePoly[i];
        }

        return aResult;
    }

    // Evaluates polynomial and its derivative simultaneously
    private static void EvaluatePolynomialWithDerivative(int theN, ReadOnlySpan<double> thePoly, double theX,
                                                         out double theValue, out double theDerivative)
    {
        theValue = thePoly[0] * theX + thePoly[1];
        theDerivative = thePoly[0];

        for (int i = 2; i < theN; ++i)
        {
            theDerivative = theDerivative * theX + theValue;
            theValue = theValue * theX + thePoly[i];
        }
    }

    // Refines root using Newton-Raphson iterations
    private static double RefineRoot(int theN, ReadOnlySpan<double> thePoly, double theInitialGuess)
    {
        double aValue = 0.0;
        double aDerivative = 0.0;
        double aSolution = theInitialGuess;
        double aInitialValue = EvaluatePolynomial(theN, thePoly, theInitialGuess);

        for (int iter = 1; iter < MAX_NEWTON_ITERATIONS; ++iter)
        {
            EvaluatePolynomialWithDerivative(theN, thePoly, aSolution, out aValue, out aDerivative);

            if (Math.Abs(aDerivative) <= ZERO_THRESHOLD)
            {
                break;
            }

            double aDelta = -aValue / aDerivative;

            if (Math.Abs(aDelta) <= MACHINE_EPSILON * Math.Abs(aSolution))
            {
                break;
            }

            aSolution += aDelta;
        }

        // Return improved solution only if it's better
        return (Math.Abs(aValue) <= Math.Abs(aInitialValue)) ? aSolution : theInitialGuess;
    }

    // Polynomial root refinement for any number of coefficients (a variadic template in OCCT)
    private static double RefinePolynomialRoot(double theInitialGuess, params ReadOnlySpan<double> theCoeffs)
    {
        return RefineRoot(theCoeffs.Length, theCoeffs, theInitialGuess);
    }

    // Helper method to scale and refine all roots of a polynomial
    private static void ScaleAndRefineAllRoots(Span<double> theRoots, int theNbRoots, double theScaleFactor,
                                               params ReadOnlySpan<double> theCoeffs)
    {
        for (int i = 0; i < theNbRoots; ++i)
        {
            theRoots[i] *= theScaleFactor;
            theRoots[i] = RefinePolynomialRoot(theRoots[i], theCoeffs);
        }
    }

    // Computes base-2 exponent for coefficient scaling
    private static int ComputeBaseExponent(double theValue)
    {
        if (theValue > 1.0)
        {
            return (int)(Math.Log(theValue) * INV_LOG_RADIX);
        }
        else if (theValue < -1.0)
        {
            return (int)(-Math.Log(-theValue) * INV_LOG_RADIX);
        }

        return 0;
    }

    // Scale coefficients to avoid overflow/underflow
    private struct ScaledCoefficients
    {
        /// Scaled coefficient values
        public double A, B, C, D, E;

        /// Scale factor for converting roots back to original coordinate system
        public double ScaleFactor;

        public void ScaleQuartic(double theA, double theB, double theC, double theD, double theE)
        {
            int aExp = ComputeBaseExponent(theE) / 4;
            ScaleFactor = Math.Pow(FLOATING_RADIX, aExp);
            double aScaleFactor2 = ScaleFactor * ScaleFactor;

            A = theA / ScaleFactor;
            B = theB / aScaleFactor2;
            C = theC / (aScaleFactor2 * ScaleFactor);
            D = theD / (aScaleFactor2 * aScaleFactor2);
            E = theE / (aScaleFactor2 * aScaleFactor2);
        }

        public void ScaleCubic(double theA, double theB, double theC, double theD)
        {
            int aExp = ComputeBaseExponent(theD) / 3;
            ScaleFactor = Math.Pow(FLOATING_RADIX, aExp);
            double aScaleFactor2 = ScaleFactor * ScaleFactor;

            A = theA / ScaleFactor;
            B = theB / aScaleFactor2;
            C = theC / (aScaleFactor2 * ScaleFactor);
            D = theD; // Not used for cubic, but kept for consistency
        }
    }

    // Computes special discriminant for P < 0 cases with improved numerical stability
    private static double ComputeSpecialDiscriminant(double theBeta, double theGamma, double theDel, double theA1)
    {
        double aSigma = theBeta * theGamma / 3.0 - 2.0 * theBeta * theBeta * theBeta / 27.0;
        double aPsi = theGamma * theGamma * (4.0 * theGamma - theBeta * theBeta) / 27.0;

        double aD1;
        if (aSigma >= 0.0)
        {
            aD1 = aSigma + 2.0 * Math.Sqrt(-theA1);
        }
        else
        {
            aD1 = aSigma - 2.0 * Math.Sqrt(-theA1);
        }

        double aD2 = aPsi / aD1;

        if (Math.Abs(theDel - aD1) >= 18.0 * MACHINE_EPSILON * (Math.Abs(theDel) + Math.Abs(aD1))
            && Math.Abs(theDel - aD2) >= 24.0 * MACHINE_EPSILON * (Math.Abs(theDel) + Math.Abs(aD2)))
        {
            return (theDel - aD1) * (theDel - aD2) / 4.0;
        }

        return 0.0;
    }

    // Solves cubic equation with three real roots using trigonometric method
    private static void SolveCubicThreeRealRoots(double theBeta, double theGamma, double theDel, double theP,
                                                 double theQ, double theDiscr, Span<double> theRoots)
    {
        if (theBeta == 0.0 && theQ == 0.0)
        {
            // Special case: x^3 + Px = 0
            theRoots[0] = Math.Sqrt(-theP);
            theRoots[1] = -theRoots[0];
            theRoots[2] = 0.0;
        }
        else
        {
            double aSb = (theBeta >= 0.0) ? 1.0 : -1.0;
            double aOmega = Math.Atan(0.5 * theQ / Math.Sqrt(-theDiscr));
            double aSp3 = Math.Sqrt(-theP / 3.0);
            double aY1 = -2.0 * aSb * aSp3 * Math.Cos(M_PI / 6.0 - aSb * aOmega / 3.0);

            theRoots[0] = -theBeta / 3.0 + aY1;

            if (theBeta * theQ <= 0.0)
            {
                theRoots[1] = -theBeta / 3.0 + 2.0 * aSp3 * Math.Sin(aOmega / 3.0);
            }
            else
            {
                // Alternative formula for better accuracy
                double aDbg = theDel - theBeta * theGamma;
                double aSdbg = (aDbg >= 0.0) ? 1.0 : -1.0;
                double aDen1 = 8.0 * theBeta * theBeta / 9.0 - 4.0 * theBeta * aY1 / 3.0 - 2.0 * theQ / aY1;
                double aDen2 = 2.0 * aY1 * aY1 - theQ / aY1;
                theRoots[1] = aDbg / aDen1 + aSdbg * Math.Sqrt(-27.0 * theDiscr) / aDen2;
            }

            // Use Vieta's formula for the third root
            theRoots[2] = -theDel / (theRoots[0] * theRoots[1]);
        }
    }

    // Solves cubic equation with one real root using Cardano's formula
    private static void SolveCubicOneRealRoot(double theBeta, double theDel, double theP, double theQ,
                                              double theDiscr, Span<double> theRoots)
    {
        double aU = Math.Sqrt(theDiscr) + Math.Abs(theQ / 2.0);
        aU = (aU >= 0.0) ? Math.Pow(aU, 1.0 / 3.0) : -Math.Pow(Math.Abs(aU), 1.0 / 3.0);

        double aH;
        if (theP >= 0.0)
        {
            aH = aU * aU + theP / 3.0 + (theP / aU) * (theP / aU) / 9.0;
        }
        else
        {
            aH = aU * Math.Abs(theQ) / (aU * aU - theP / 3.0);
        }

        if (theBeta * theQ >= 0.0)
        {
            if (Math.Abs(aH) <= RealSmall() && Math.Abs(theQ) <= RealSmall())
            {
                theRoots[0] = -theBeta / 3.0 - aU + theP / (3.0 * aU);
            }
            else
            {
                theRoots[0] = -theBeta / 3.0 - theQ / aH;
            }
        }
        else
        {
            theRoots[0] = -theDel / (theBeta * theBeta / 9.0 + aH - theBeta * theQ / (3.0 * aH));
        }
    }

    // Solves cubic equation with multiple roots (discriminant = 0)
    private static void SolveCubicMultipleRoots(double theBeta, double theGamma, double theDel, double theP,
                                                double theQ, Span<double> theRoots, out int theNbRoots)
    {
        theNbRoots = 3;
        double aSq = (theQ >= 0.0) ? 1.0 : -1.0;
        double aSp3 = Math.Sqrt(-theP / 3.0);

        if (theBeta * theQ <= 0.0)
        {
            theRoots[0] = -theBeta / 3.0 + aSq * aSp3;
            theRoots[1] = theRoots[0];

            if (theBeta * theQ == 0.0)
            {
                theRoots[2] = -theBeta / 3.0 - 2.0 * aSq * aSp3;
            }
            else
            {
                theRoots[2] = -theDel / (theRoots[0] * theRoots[1]);
            }
        }
        else
        {
            theRoots[0] = -theGamma / (theBeta + 3.0 * aSq * aSp3);
            theRoots[1] = theRoots[0];
            theRoots[2] = -theBeta / 3.0 - 2.0 * aSq * aSp3;
        }
    }

    // Determines if quartic should be reduced to lower degree
    private static bool ShouldReduceDegreeQuartic(double theA, double theB, double theC, double theD, double theE)
    {
        if (Math.Abs(theA) <= ZERO_THRESHOLD)
        {
            return true;
        }

        // Modified by jgv, 22.01.09 for numerical stability
        double aMaxCoeff = ZERO_THRESHOLD;
        aMaxCoeff = Max(aMaxCoeff, Math.Abs(theB));
        aMaxCoeff = Max(aMaxCoeff, Math.Abs(theC));
        aMaxCoeff = Max(aMaxCoeff, Math.Abs(theD));
        aMaxCoeff = Max(aMaxCoeff, Math.Abs(theE));

        if (aMaxCoeff > ZERO_THRESHOLD)
        {
            aMaxCoeff = Epsilon(100.0 * aMaxCoeff);
        }

        if (Math.Abs(theA) <= aMaxCoeff)
        {
            double aMaxCoeff1000 = 1000.0 * aMaxCoeff;
            bool aWithA = false;

            if (Math.Abs(theB) > ZERO_THRESHOLD && Math.Abs(theB) <= aMaxCoeff1000)
            {
                aWithA = true;
            }

            if (Math.Abs(theC) > ZERO_THRESHOLD && Math.Abs(theC) <= aMaxCoeff1000)
            {
                aWithA = true;
            }

            if (Math.Abs(theD) > ZERO_THRESHOLD && Math.Abs(theD) <= aMaxCoeff1000)
            {
                aWithA = true;
            }

            if (Math.Abs(theE) > ZERO_THRESHOLD && Math.Abs(theE) <= aMaxCoeff1000)
            {
                aWithA = true;
            }

            return !aWithA;
        }

        return false;
    }

    // Constructs and solves Ferrari's resolvent cubic equation
    private static double SolveFerrariResolvent(double theA, double theB, double theC, double theD, ref bool theSuccess)
    {
        // Construct resolvent cubic: Y^3 + R3*Y^2 + S3*Y + T3 = 0
        double aR3 = -theB;
        double aS3 = theA * theC - 4.0 * theD;
        double aT3 = theD * (4.0 * theB - theA * theA) - theC * theC;

        math_DirectPolynomialRoots aCubicSolver = new(1.0, aR3, aS3, aT3);

        if (!aCubicSolver.IsDone())
        {
            theSuccess = false;
            return 0.0;
        }

        theSuccess = true;

        // Choose the largest root for numerical stability
        double aY0 = aCubicSolver.Value(1);
        for (int i = 2; i <= aCubicSolver.NbSolutions(); ++i)
        {
            if (aCubicSolver.Value(i) > aY0)
            {
                aY0 = aCubicSolver.Value(i);
            }
        }

        return aY0;
    }

    // Structure for quartic factorization into two quadratic equations
    private struct QuarticFactorization
    {
        /// Linear coefficient of first quadratic factor
        public double P1;

        /// Constant coefficient of first quadratic factor
        public double Q1;

        /// Linear coefficient of second quadratic factor
        public double P2;

        /// Constant coefficient of second quadratic factor
        public double Q2;
    }

    // Factors quartic into two quadratics using Ferrari's method
    private static QuarticFactorization FactorQuarticViaFerrari(double theA, double theB, double theC, double theD, double theY0)
    {
        QuarticFactorization aFactors;

        // Compute discriminant and parameters
        double aDiscr = theA * theY0 * 0.5 - theC;
        double aSdiscr = (aDiscr >= 0.0) ? 1.0 : -1.0;

        // Compute P0 and Q0 for the quadratic factors
        double aP0 = theA * theA * 0.25 - theB + theY0;
        aP0 = (aP0 < 0.0) ? 0.0 : Math.Sqrt(aP0);

        double aQ0 = theY0 * theY0 * 0.25 - theD;

        // Handle the case where Q0^2 is very close to zero more robustly
        // This fixes the numerical precision issue on Linux/Windows vs macOS
        if (Math.Abs(aQ0) < 10 * MACHINE_EPSILON)
        {
            aQ0 = 0.0;
        }
        else
        {
            aQ0 = (aQ0 < 0.0) ? 0.0 : Math.Sqrt(aQ0);
        }

        // Form coefficients for the two quadratic equations
        double aAdemi = theA * 0.5;
        double aYdemi = theY0 * 0.5;
        double aSdiscrQ0 = aSdiscr * aQ0;

        aFactors.P1 = aAdemi + aP0;
        aFactors.Q1 = aYdemi + aSdiscrQ0;
        aFactors.P2 = aAdemi - aP0;
        aFactors.Q2 = aYdemi - aSdiscrQ0;

        // Clean up near-zero coefficients
        double anEps = 100.0 * MACHINE_EPSILON;

        if (Math.Abs(aFactors.P1) <= anEps)
        {
            aFactors.P1 = 0.0;
        }

        if (Math.Abs(aFactors.P2) <= anEps)
        {
            aFactors.P2 = 0.0;
        }

        if (Math.Abs(aFactors.Q1) <= anEps)
        {
            aFactors.Q1 = 0.0;
        }

        if (Math.Abs(aFactors.Q2) <= anEps)
        {
            aFactors.Q2 = 0.0;
        }

        return aFactors;
    }
}
