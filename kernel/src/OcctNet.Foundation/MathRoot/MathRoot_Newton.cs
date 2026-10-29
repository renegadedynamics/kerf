// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_Newton.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

public static partial class MathRoot
{
    /// <summary>Newton-Raphson root finding from theGuess (quadratic convergence near a simple root).</summary>
    public static MathUtils.ScalarResult Newton<Function>(Function theFunc, double theGuess, MathUtils.Config? theConfig = null)
        where Function : MathUtils.IScalarFunctionWithDerivative
    {
        theConfig ??= new MathUtils.Config();
        MathUtils.ScalarResult aResult = new();
        double aX = theGuess;
        double aFx = 0.0;
        double aDfx = 0.0;

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            double anXOld = aX;

            // Evaluate function and derivative
            if (!theFunc.Values(aX, out aFx, out aDfx))
            {
                aResult.Status = MathUtils.Status.NumericalError;
                aResult.Root = aX;
                aResult.NbIterations = anIter;
                return aResult;
            }

            // Check for zero derivative (stationary point)
            if (MathUtils.IsZero(aDfx))
            {
                // Try to continue with a small perturbation if not converged
                if (!MathUtils.IsFConverged(aFx, theConfig.FTolerance))
                {
                    aResult.Status = MathUtils.Status.NumericalError;
                    aResult.Root = aX;
                    aResult.Value = aFx;
                    aResult.Derivative = aDfx;
                    aResult.NbIterations = anIter;
                    return aResult;
                }
                // Zero derivative at a root is fine (multiple root)
            }
            else
            {
                // Newton step
                aX -= aFx / aDfx;
            }

            aResult.NbIterations = anIter + 1;

            // Check convergence
            if (MathUtils.IsConverged(anXOld, aX, aFx, theConfig))
            {
                aResult.Status = MathUtils.Status.OK;
                aResult.Root = aX;
                aResult.Value = aFx;
                aResult.Derivative = aDfx;
                return aResult;
            }
        }

        // Maximum iterations reached
        aResult.Status = MathUtils.Status.MaxIterations;
        aResult.Root = aX;
        aResult.Value = aFx;
        aResult.Derivative = aDfx;
        return aResult;
    }

    /// <summary>
    /// Newton-Raphson kept within [theLower, theUpper], falling back to bisection when the bounds bracket a root.
    /// </summary>
    public static MathUtils.ScalarResult NewtonBounded<Function>(Function theFunc, double theGuess, double theLower, double theUpper, MathUtils.Config? theConfig = null)
        where Function : MathUtils.IScalarFunctionWithDerivative
    {
        theConfig ??= new MathUtils.Config();
        MathUtils.ScalarResult aResult = new();

        // Clamp initial guess to bounds
        double aX = MathUtils.Clamp(theGuess, theLower, theUpper);
        double aXLo = theLower;
        double aXHi = theUpper;

        double aFx = 0.0;
        double aDfx = 0.0;

        // Initialize bounds with function values
        if (!theFunc.Values(aXLo, out double aFLo, out _))
        {
            aResult.Status = MathUtils.Status.NumericalError;
            return aResult;
        }
        if (!theFunc.Values(aXHi, out double aFHi, out _))
        {
            aResult.Status = MathUtils.Status.NumericalError;
            return aResult;
        }

        // Check if bounds bracket a root
        bool aBracketed = (aFLo * aFHi < 0.0);

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            double anXOld = aX;

            // Evaluate function and derivative at current point
            if (!theFunc.Values(aX, out aFx, out aDfx))
            {
                aResult.Status = MathUtils.Status.NumericalError;
                aResult.Root = aX;
                aResult.NbIterations = anIter;
                return aResult;
            }

            aResult.NbIterations = anIter + 1;

            // Check convergence
            if (MathUtils.IsFConverged(aFx, theConfig.FTolerance))
            {
                aResult.Status = MathUtils.Status.OK;
                aResult.Root = aX;
                aResult.Value = aFx;
                aResult.Derivative = aDfx;
                return aResult;
            }

            // Compute Newton step
            double aXNew = aX;
            if (!MathUtils.IsZero(aDfx))
            {
                aXNew = aX - aFx / aDfx;
            }

            // Check if Newton step is within bounds
            if (aXNew < aXLo || aXNew > aXHi)
            {
                // Fall back to bisection if bracketed
                if (aBracketed)
                {
                    aXNew = 0.5 * (aXLo + aXHi);
                }
                else
                {
                    // Just clamp to bounds
                    aXNew = MathUtils.Clamp(aXNew, aXLo, aXHi);
                }
            }

            aX = aXNew;

            // Update bracket if root is bracketed
            if (aBracketed)
            {
                if (aFx * aFLo < 0.0)
                {
                    aXHi = anXOld;
                    aFHi = aFx;
                }
                else
                {
                    aXLo = anXOld;
                    aFLo = aFx;
                }
            }

            // Check X convergence
            if (MathUtils.IsXConverged(anXOld, aX, theConfig.XTolerance))
            {
                // Re-evaluate at final position
                theFunc.Values(aX, out aFx, out aDfx);
                aResult.Status = MathUtils.Status.OK;
                aResult.Root = aX;
                aResult.Value = aFx;
                aResult.Derivative = aDfx;
                return aResult;
            }
        }

        // Maximum iterations reached
        aResult.Status = MathUtils.Status.MaxIterations;
        aResult.Root = aX;
        aResult.Value = aFx;
        aResult.Derivative = aDfx;
        return aResult;
    }
}
