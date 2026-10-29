// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_LineSearch.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Line search algorithms for optimization.</summary>
public static partial class MathUtils
{
    /// <summary>Result of line search operation.</summary>
    public struct LineSearchResult
    {
        /// <summary>True if line search succeeded.</summary>
        public bool IsValid;
        /// <summary>Step size found.</summary>
        public double Alpha;
        /// <summary>Function value at new point.</summary>
        public double FNew;
        /// <summary>Number of function evaluations.</summary>
        public int NbEvals;
    }

    /// <summary>
    /// Backtracking line search with the Armijo condition: finds alpha such that
    /// f(x + alpha*d) &lt;= f(x) + c1*alpha*grad(f).d.
    /// </summary>
    public static LineSearchResult ArmijoBacktrack<Function>(Function theFunc,
                                                             math_Vector theX,
                                                             math_Vector theDir,
                                                             math_Vector theGrad,
                                                             double theFx,
                                                             double theAlphaInit = 1.0,
                                                             double theC1 = 1.0e-4,
                                                             double theRho = 0.5,
                                                             int theMaxIter = 50)
        where Function : IVectorFunction
    {
        LineSearchResult aResult = new();
        aResult.Alpha = theAlphaInit;
        aResult.NbEvals = 0;

        int aLower = theX.Lower();
        int aUpper = theX.Upper();

        // Compute directional derivative: grad(f) . d
        double aDirDeriv = 0.0;
        for (int i = aLower; i <= aUpper; ++i)
        {
            aDirDeriv += theGrad[i] * theDir[i];
        }

        // Ensure descent direction
        if (aDirDeriv >= 0.0)
        {
            // Not a descent direction, return failure
            aResult.IsValid = false;
            return aResult;
        }

        // Temporary vector for new point
        math_Vector aXNew = new(aLower, aUpper);

        for (int k = 0; k < theMaxIter; ++k)
        {
            // Compute new point: x + alpha*d
            for (int i = aLower; i <= aUpper; ++i)
            {
                aXNew[i] = theX[i] + aResult.Alpha * theDir[i];
            }

            // Evaluate function at new point
            if (!theFunc.Value(aXNew, out double aFNew))
            {
                // Function evaluation failed, reduce step
                aResult.Alpha *= theRho;
                ++aResult.NbEvals;
                continue;
            }
            ++aResult.NbEvals;

            // Check Armijo condition: f(x + alpha*d) <= f(x) + c1*alpha*phi'(0)
            if (aFNew <= theFx + theC1 * aResult.Alpha * aDirDeriv)
            {
                aResult.IsValid = true;
                aResult.FNew = aFNew;
                return aResult;
            }

            // Reduce step size
            aResult.Alpha *= theRho;

            // Check for too small step
            if (aResult.Alpha < THE_EPSILON)
            {
                break;
            }
        }

        // Failed to satisfy Armijo condition
        aResult.IsValid = false;
        return aResult;
    }

    /// <summary>
    /// Strong Wolfe line search (Armijo plus curvature |grad(f)(x + alpha*d).d| &lt;= c2*|grad(f).d|), by bracketing
    /// and zoom.
    /// </summary>
    public static LineSearchResult WolfeSearch<Function>(Function theFunc,
                                                         math_Vector theX,
                                                         math_Vector theDir,
                                                         math_Vector theGrad,
                                                         double theFx,
                                                         double theAlphaInit = 1.0,
                                                         double theC1 = 1.0e-4,
                                                         double theC2 = 0.9,
                                                         int theMaxIter = 20)
        where Function : IVectorFunctionWithGradient
    {
        LineSearchResult aResult = new();
        aResult.NbEvals = 0;

        int aLower = theX.Lower();
        int aUpper = theX.Upper();

        // Compute initial directional derivative
        double aPhi0Prime = 0.0;
        for (int i = aLower; i <= aUpper; ++i)
        {
            aPhi0Prime += theGrad[i] * theDir[i];
        }

        if (aPhi0Prime >= 0.0)
        {
            aResult.IsValid = false;
            return aResult;
        }

        math_Vector aXNew = new(aLower, aUpper);
        math_Vector aGradNew = new(aLower, aUpper);

        double aAlphaLo = 0.0;
        double aAlphaHi = theAlphaInit * 2.0;
        double aAlpha = theAlphaInit;

        double aPhiLo = theFx;

        // Phase 1: Bracket finding
        for (int k = 0; k < theMaxIter; ++k)
        {
            // Evaluate at current alpha
            for (int i = aLower; i <= aUpper; ++i)
            {
                aXNew[i] = theX[i] + aAlpha * theDir[i];
            }

            if (!theFunc.Value(aXNew, out double aPhi))
            {
                // Shrink interval
                aAlphaHi = aAlpha;
                aAlpha = 0.5 * (aAlphaLo + aAlphaHi);
                ++aResult.NbEvals;
                continue;
            }
            ++aResult.NbEvals;

            // Check Armijo
            if (aPhi > theFx + theC1 * aAlpha * aPhi0Prime || (k > 0 && aPhi >= aPhiLo))
            {
                // Found bracket [alpha_lo, alpha]
                aAlphaHi = aAlpha;
                break;
            }

            // Evaluate gradient at new point
            if (!theFunc.Gradient(aXNew, aGradNew))
            {
                aResult.IsValid = false;
                return aResult;
            }

            double aPhiPrime = 0.0;
            for (int i = aLower; i <= aUpper; ++i)
            {
                aPhiPrime += aGradNew[i] * theDir[i];
            }

            // Check curvature condition
            if (Math.Abs(aPhiPrime) <= -theC2 * aPhi0Prime)
            {
                aResult.IsValid = true;
                aResult.Alpha = aAlpha;
                aResult.FNew = aPhi;
                return aResult;
            }

            if (aPhiPrime >= 0.0)
            {
                // Found bracket [alpha, alpha_lo]
                aAlphaHi = aAlphaLo;
                aAlphaLo = aAlpha;
                aPhiLo = aPhi;
                break;
            }

            // Increase alpha
            aAlphaLo = aAlpha;
            aPhiLo = aPhi;
            aAlpha = 0.5 * (aAlpha + aAlphaHi);
        }

        // Phase 2: Zoom
        for (int k = 0; k < theMaxIter; ++k)
        {
            // Bisection
            aAlpha = 0.5 * (aAlphaLo + aAlphaHi);

            for (int i = aLower; i <= aUpper; ++i)
            {
                aXNew[i] = theX[i] + aAlpha * theDir[i];
            }

            if (!theFunc.Value(aXNew, out double aPhi))
            {
                aAlphaHi = aAlpha;
                ++aResult.NbEvals;
                continue;
            }
            ++aResult.NbEvals;

            if (aPhi > theFx + theC1 * aAlpha * aPhi0Prime || aPhi >= aPhiLo)
            {
                aAlphaHi = aAlpha;
            }
            else
            {
                if (!theFunc.Gradient(aXNew, aGradNew))
                {
                    break;
                }

                double aPhiPrime = 0.0;
                for (int i = aLower; i <= aUpper; ++i)
                {
                    aPhiPrime += aGradNew[i] * theDir[i];
                }

                if (Math.Abs(aPhiPrime) <= -theC2 * aPhi0Prime)
                {
                    aResult.IsValid = true;
                    aResult.Alpha = aAlpha;
                    aResult.FNew = aPhi;
                    return aResult;
                }

                if (aPhiPrime * (aAlphaHi - aAlphaLo) >= 0.0)
                {
                    aAlphaHi = aAlphaLo;
                }

                aAlphaLo = aAlpha;
                aPhiLo = aPhi;
            }

            // Check for convergence
            if (Math.Abs(aAlphaHi - aAlphaLo) < THE_EPSILON)
            {
                break;
            }
        }

        // Return best found
        aResult.IsValid = true;
        aResult.Alpha = aAlpha;
        aResult.FNew = aPhiLo;
        return aResult;
    }

    /// <summary>
    /// Exact line search by Brent's method: minimizes f(x + alpha*d) over alpha in [-alpha_max, alpha_max].
    /// </summary>
    public static LineSearchResult ExactLineSearch<Function>(Function theFunc,
                                                             math_Vector theX,
                                                             math_Vector theDir,
                                                             double theAlphaMax = 10.0,
                                                             double theTolerance = 1.0e-6,
                                                             int theMaxIter = 100)
        where Function : IVectorFunction
    {
        LineSearchResult aResult = new();
        aResult.NbEvals = 0;

        int aLower = theX.Lower();
        int aUpper = theX.Upper();

        math_Vector aXNew = new(aLower, aUpper);

        // Lambda to evaluate phi(alpha) = f(x + alpha*d)
        bool aEvalPhi(double theAlpha, out double thePhi)
        {
            for (int i = aLower; i <= aUpper; ++i)
            {
                aXNew[i] = theX[i] + theAlpha * theDir[i];
            }
            ++aResult.NbEvals;
            return theFunc.Value(aXNew, out thePhi);
        }

        // Brent's method for 1D minimization
        // Search in [-theAlphaMax, theAlphaMax] to handle both directions
        double aA = -theAlphaMax;
        double aB = theAlphaMax;
        double aX = 0.0; // Start at the current point
        double aW = aX;
        double aV = aX;

        if (!aEvalPhi(aX, out double aFx))
        {
            aResult.IsValid = false;
            return aResult;
        }
        double aFw = aFx;
        double aFv = aFx;

        double aD = 0.0;
        double aE = 0.0;

        for (int anIter = 0; anIter < theMaxIter; ++anIter)
        {
            double aXm = 0.5 * (aA + aB);
            double aTol1 = theTolerance * Math.Abs(aX) + THE_ZERO_TOL / 10.0;
            double aTol2 = 2.0 * aTol1;

            // Check convergence
            if (Math.Abs(aX - aXm) <= (aTol2 - 0.5 * (aB - aA)))
            {
                aResult.IsValid = true;
                aResult.Alpha = aX;
                aResult.FNew = aFx;
                return aResult;
            }

            double aU = 0.0;
            bool aUseParabolic = false;

            // Try parabolic interpolation
            if (Math.Abs(aE) > aTol1)
            {
                double aR = (aX - aW) * (aFx - aFv);
                double aQ = (aX - aV) * (aFx - aFw);
                double aP = (aX - aV) * aQ - (aX - aW) * aR;
                aQ = 2.0 * (aQ - aR);

                if (aQ > 0.0)
                {
                    aP = -aP;
                }
                else
                {
                    aQ = -aQ;
                }

                double aETmp = aE;
                aE = aD;

                if (Math.Abs(aP) < Math.Abs(0.5 * aQ * aETmp) && aP > aQ * (aA - aX) && aP < aQ * (aB - aX))
                {
                    aD = aP / aQ;
                    aU = aX + aD;
                    if ((aU - aA) < aTol2 || (aB - aU) < aTol2)
                    {
                        aD = SignTransfer(aTol1, aXm - aX);
                    }
                    aUseParabolic = true;
                }
            }

            if (!aUseParabolic)
            {
                aE = (aX < aXm) ? (aB - aX) : (aA - aX);
                aD = THE_GOLDEN_SECTION * aE;
            }

            if (Math.Abs(aD) >= aTol1)
            {
                aU = aX + aD;
            }
            else
            {
                aU = aX + SignTransfer(aTol1, aD);
            }

            if (!aEvalPhi(aU, out double aFu))
            {
                aResult.IsValid = false;
                aResult.Alpha = aX;
                aResult.FNew = aFx;
                return aResult;
            }

            // Update bracket
            if (aFu <= aFx)
            {
                if (aU < aX)
                {
                    aB = aX;
                }
                else
                {
                    aA = aX;
                }

                aV = aW;
                aW = aX;
                aX = aU;
                aFv = aFw;
                aFw = aFx;
                aFx = aFu;
            }
            else
            {
                if (aU < aX)
                {
                    aA = aU;
                }
                else
                {
                    aB = aU;
                }

                if (aFu <= aFw || aW == aX)
                {
                    aV = aW;
                    aW = aU;
                    aFv = aFw;
                    aFw = aFu;
                }
                else if (aFu <= aFv || aV == aX || aV == aW)
                {
                    aV = aU;
                    aFv = aFu;
                }
            }
        }

        aResult.IsValid = true;
        aResult.Alpha = aX;
        aResult.FNew = aFx;
        return aResult;
    }

    /// <summary>Quadratic interpolation step for line search from phi(0), phi'(0) and phi(alpha1).</summary>
    public static double QuadraticInterpolation(double thePhi0, double thePhi0Prime, double theAlpha1, double thePhi1)
    {
        // Quadratic: phi(alpha) = phi(0) + phi'(0)*alpha + c*alpha^2
        // where c = (phi(alpha1) - phi(0) - phi'(0)*alpha1) / alpha1^2
        // Minimum at alpha* = -phi'(0) / (2c)
        double aNum = thePhi0Prime * theAlpha1 * theAlpha1;
        double aDenom = 2.0 * (thePhi1 - thePhi0 - thePhi0Prime * theAlpha1);

        if (Math.Abs(aDenom) < THE_ZERO_TOL)
        {
            return 0.5 * theAlpha1;
        }

        double aAlphaNew = -aNum / aDenom;

        // Safeguard: ensure we stay within reasonable bounds
        if (aAlphaNew < 0.1 * theAlpha1)
        {
            aAlphaNew = 0.1 * theAlpha1;
        }
        else if (aAlphaNew > 0.9 * theAlpha1)
        {
            aAlphaNew = 0.5 * theAlpha1;
        }

        return aAlphaNew;
    }

    /// <summary>
    /// Brent's method for 1D minimization along one coordinate of thePoint, in place: the coordinate is modified during
    /// the search and restored if no improvement is found. Returns true if thePoint and theFx were improved.
    /// </summary>
    public static bool BrentAlongCoordinate<Function>(Function theFunc,
                                                      math_Vector thePoint,
                                                      int theDimIdx,
                                                      double theLoBound,
                                                      double theUpBound,
                                                      ref double theFx,
                                                      double theTolerance,
                                                      int theMaxIter,
                                                      ref int theEvalCount)
        where Function : IVectorFunction
    {
        double aOrigCoord = thePoint[theDimIdx];

        // Search interval [aA, aB] in coordinate space (not alpha space)
        double aA = theLoBound;
        double aB = theUpBound;
        double aX = aOrigCoord;
        double aW = aX;
        double aV = aX;

        double aFx = theFx;
        double aFw = aFx;
        double aFv = aFx;

        double aD = 0.0;
        double aE = 0.0;

        for (int anIter = 0; anIter < theMaxIter; ++anIter)
        {
            double aXm = 0.5 * (aA + aB);
            double aTol1 = theTolerance * Math.Abs(aX) + THE_ZERO_TOL / 10.0;
            double aTol2 = 2.0 * aTol1;

            // Check convergence
            if (Math.Abs(aX - aXm) <= (aTol2 - 0.5 * (aB - aA)))
            {
                break;
            }

            double aU = 0.0;
            bool aUseParabolic = false;

            // Try parabolic interpolation
            if (Math.Abs(aE) > aTol1)
            {
                double aR = (aX - aW) * (aFx - aFv);
                double aQ = (aX - aV) * (aFx - aFw);
                double aP = (aX - aV) * aQ - (aX - aW) * aR;
                aQ = 2.0 * (aQ - aR);

                if (aQ > 0.0)
                {
                    aP = -aP;
                }
                else
                {
                    aQ = -aQ;
                }

                double aETmp = aE;
                aE = aD;

                if (Math.Abs(aP) < Math.Abs(0.5 * aQ * aETmp) && aP > aQ * (aA - aX) && aP < aQ * (aB - aX))
                {
                    aD = aP / aQ;
                    aU = aX + aD;
                    if ((aU - aA) < aTol2 || (aB - aU) < aTol2)
                    {
                        aD = SignTransfer(aTol1, aXm - aX);
                    }
                    aUseParabolic = true;
                }
            }

            if (!aUseParabolic)
            {
                aE = (aX < aXm) ? (aB - aX) : (aA - aX);
                aD = THE_GOLDEN_SECTION * aE;
            }

            if (Math.Abs(aD) >= aTol1)
            {
                aU = aX + aD;
            }
            else
            {
                aU = aX + SignTransfer(aTol1, aD);
            }

            // Evaluate: only modify the single coordinate
            thePoint[theDimIdx] = aU;
            if (!theFunc.Value(thePoint, out double aFu))
            {
                // Restore and abort
                thePoint[theDimIdx] = aOrigCoord;
                return false;
            }
            ++theEvalCount;

            // Update bracket
            if (aFu <= aFx)
            {
                if (aU < aX)
                {
                    aB = aX;
                }
                else
                {
                    aA = aX;
                }
                aV = aW;
                aW = aX;
                aX = aU;
                aFv = aFw;
                aFw = aFx;
                aFx = aFu;
            }
            else
            {
                if (aU < aX)
                {
                    aA = aU;
                }
                else
                {
                    aB = aU;
                }
                if (aFu <= aFw || aW == aX)
                {
                    aV = aW;
                    aW = aU;
                    aFv = aFw;
                    aFw = aFu;
                }
                else if (aFu <= aFv || aV == aX || aV == aW)
                {
                    aV = aU;
                    aFv = aFu;
                }
            }
        }

        // Accept if improved
        if (aFx < theFx)
        {
            thePoint[theDimIdx] = aX;
            theFx = aFx;
            return true;
        }

        // Restore original coordinate
        thePoint[theDimIdx] = aOrigCoord;
        return false;
    }
}
