// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_Brent.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Modern scalar root finding algorithms.</summary>
public static partial class MathRoot
{
    /// <summary>
    /// Brent's method for root finding: bisection, secant and inverse quadratic interpolation. f(theLower) and
    /// f(theUpper) must have opposite signs.
    /// </summary>
    public static MathUtils.ScalarResult Brent<Function>(Function theFunc, double theLower, double theUpper, MathUtils.Config? theConfig = null)
        where Function : MathUtils.IScalarFunction
    {
        theConfig ??= new MathUtils.Config();
        MathUtils.ScalarResult aResult = new();

        double aA = theLower;
        double aB = theUpper;

        // Evaluate at endpoints
        if (!theFunc.Value(aA, out double aFa))
        {
            aResult.Status = MathUtils.Status.NumericalError;
            return aResult;
        }
        if (!theFunc.Value(aB, out double aFb))
        {
            aResult.Status = MathUtils.Status.NumericalError;
            return aResult;
        }

        // Check that bracket is valid (sign change)
        if (aFa * aFb > 0.0)
        {
            aResult.Status = MathUtils.Status.InvalidInput;
            return aResult;
        }

        // Ensure |f(a)| >= |f(b)| (b is the better approximation)
        if (Math.Abs(aFa) < Math.Abs(aFb))
        {
            (aA, aB) = (aB, aA);
            (aFa, aFb) = (aFb, aFa);
        }

        double aC = aA; // Previous iterate
        double aFc = aFa;
        double aD = aB - aA; // Step size
        double aE = aD;      // Previous step size

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            double aTol = 2.0 * MathUtils.THE_EPSILON * Math.Abs(aB) + 0.5 * theConfig.XTolerance;
            double aM = 0.5 * (aC - aB);

            if (Math.Abs(aFb) < theConfig.FTolerance || aFb == 0.0 || Math.Abs(aM) <= aTol)
            {
                aResult.Status = MathUtils.Status.OK;
                aResult.Root = aB;
                aResult.Value = aFb;
                return aResult;
            }

            double aS = 0.0; // New approximation

            // Try inverse quadratic interpolation if we have three distinct points
            if (Math.Abs(aFa - aFc) > MathUtils.THE_ZERO_TOL
                && Math.Abs(aFb - aFc) > MathUtils.THE_ZERO_TOL)
            {
                // Inverse quadratic interpolation
                aS = aA * aFb * aFc / ((aFa - aFb) * (aFa - aFc))
                     + aB * aFa * aFc / ((aFb - aFa) * (aFb - aFc))
                     + aC * aFa * aFb / ((aFc - aFa) * (aFc - aFb));
            }
            else
            {
                // Secant method
                aS = aB - aFb * (aB - aA) / (aFb - aFa);
            }

            // Decide whether to accept the interpolation step
            bool aUseInterp = false;

            // Check if s is between (3a+b)/4 and b
            double aBound1 = (3.0 * aA + aB) / 4.0;
            if ((aS > Min(aBound1, aB) && aS < Max(aBound1, aB)))
            {
                // Accept interpolation if step is smaller than half the previous step
                // (ensures convergence rate). Minimum step is enforced later.
                if (Math.Abs(aS - aB) < Math.Abs(aE) / 2.0)
                {
                    aUseInterp = true;
                }
            }

            if (!aUseInterp)
            {
                // Bisection step
                aS = aB + aM;
                aE = aM;
                aD = aM;
            }
            else
            {
                aE = aD;
                aD = aS - aB;
            }

            // Update previous values
            aA = aB;
            aFa = aFb;

            // Compute new point, ensuring minimum step
            if (Math.Abs(aD) > aTol)
            {
                aB = aS;
            }
            else
            {
                aB += (aM > 0.0) ? aTol : -aTol;
            }

            // Evaluate function at new point
            if (!theFunc.Value(aB, out aFb))
            {
                aResult.Status = MathUtils.Status.NumericalError;
                aResult.Root = aB;
                return aResult;
            }

            // Update bracket
            if (aFb * aFc > 0.0)
            {
                aC = aA;
                aFc = aFa;
                aD = aB - aA;
                aE = aD;
            }
            else if (Math.Abs(aFc) < Math.Abs(aFb))
            {
                // Swap b and c if c is better
                aA = aB;
                aFa = aFb;
                (aB, aC) = (aC, aB);
                (aFb, aFc) = (aFc, aFb);
            }
        }

        // Maximum iterations reached
        aResult.Status = MathUtils.Status.MaxIterations;
        aResult.Root = aB;
        aResult.Value = aFb;
        return aResult;
    }
}
