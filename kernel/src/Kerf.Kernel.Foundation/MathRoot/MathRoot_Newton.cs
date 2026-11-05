// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_Newton.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Roots;

internal static partial class MathRoot
{
    /// <summary>Newton-Raphson root finding from theGuess (quadratic convergence near a simple root).</summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult Newton<Function>(Function func, double guess, global::Kerf.Engine.Numerics.Utilities.MathUtils.Config? config = null)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
    {
        config ??= new global::Kerf.Engine.Numerics.Utilities.MathUtils.Config();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult result = new();
        double x = guess;
        double fx = 0.0;
        double dfx = 0.0;

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            double xOld = x;

            // Evaluate function and derivative
            if (!func.Values(x, out fx, out dfx))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                result.Root = x;
                result.NbIterations = iter;
                return result;
            }

            // Check for zero derivative (stationary point)
            if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(dfx))
            {
                // Try to continue with a small perturbation if not converged
                if (!global::Kerf.Engine.Numerics.Utilities.MathUtils.IsFConverged(fx, config.FTolerance))
                {
                    result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                    result.Root = x;
                    result.Value = fx;
                    result.Derivative = dfx;
                    result.NbIterations = iter;
                    return result;
                }
                // Zero derivative at a root is fine (multiple root)
            }
            else
            {
                // Newton step
                x -= fx / dfx;
            }

            result.NbIterations = iter + 1;

            // Check convergence
            if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsConverged(xOld, x, fx, config))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                result.Root = x;
                result.Value = fx;
                result.Derivative = dfx;
                return result;
            }
        }

        // Maximum iterations reached
        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
        result.Root = x;
        result.Value = fx;
        result.Derivative = dfx;
        return result;
    }

    /// <summary>
    /// Newton-Raphson kept within [theLower, theUpper], falling back to bisection when the bounds bracket a root.
    /// </summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult NewtonBounded<Function>(Function func, double guess, double lower, double upper, global::Kerf.Engine.Numerics.Utilities.MathUtils.Config? config = null)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
    {
        config ??= new global::Kerf.Engine.Numerics.Utilities.MathUtils.Config();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult result = new();

        // Clamp initial guess to bounds
        double x = global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(guess, lower, upper);
        double xLo = lower;
        double xHi = upper;

        double fx = 0.0;
        double dfx = 0.0;

        // Initialize bounds with function values
        if (!func.Values(xLo, out double fLo, out _))
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
            return result;
        }
        if (!func.Values(xHi, out double fHi, out _))
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
            return result;
        }

        // Check if bounds bracket a root
        bool bracketed = (fLo * fHi < 0.0);

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            double xOld = x;

            // Evaluate function and derivative at current point
            if (!func.Values(x, out fx, out dfx))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                result.Root = x;
                result.NbIterations = iter;
                return result;
            }

            result.NbIterations = iter + 1;

            // Check convergence
            if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsFConverged(fx, config.FTolerance))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                result.Root = x;
                result.Value = fx;
                result.Derivative = dfx;
                return result;
            }

            // Compute Newton step
            double xNew = x;
            if (!global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(dfx))
            {
                xNew = x - fx / dfx;
            }

            // Check if Newton step is within bounds
            if (xNew < xLo || xNew > xHi)
            {
                // Fall back to bisection if bracketed
                if (bracketed)
                {
                    xNew = 0.5 * (xLo + xHi);
                }
                else
                {
                    // Just clamp to bounds
                    xNew = global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(xNew, xLo, xHi);
                }
            }

            x = xNew;

            // Update bracket if root is bracketed
            if (bracketed)
            {
                if (fx * fLo < 0.0)
                {
                    xHi = xOld;
                    fHi = fx;
                }
                else
                {
                    xLo = xOld;
                    fLo = fx;
                }
            }

            // Check X convergence
            if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsXConverged(xOld, x, config.XTolerance))
            {
                // Re-evaluate at final position
                func.Values(x, out fx, out dfx);
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                result.Root = x;
                result.Value = fx;
                result.Derivative = dfx;
                return result;
            }
        }

        // Maximum iterations reached
        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
        result.Root = x;
        result.Value = fx;
        result.Derivative = dfx;
        return result;
    }
}
