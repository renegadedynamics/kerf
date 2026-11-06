// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_Brent.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Numerics.Roots;

/// <summary>Modern scalar root finding algorithms.</summary>
internal static partial class MathRoot
{
    /// <summary>
    /// Brent's method for root finding: bisection, secant and inverse quadratic interpolation. f(theLower) and
    /// f(theUpper) must have opposite signs.
    /// </summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult Brent<Function>(Function func, double lower, double upper, global::Kerf.Engine.Numerics.Utilities.MathUtils.Config? config = null)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        config ??= new global::Kerf.Engine.Numerics.Utilities.MathUtils.Config();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult result = new();

        double a = lower;
        double b = upper;

        // Evaluate at endpoints
        if (!func.Value(a, out double fa))
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
            return result;
        }
        if (!func.Value(b, out double fb))
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
            return result;
        }

        // Check that bracket is valid (sign change)
        if (fa * fb > 0.0)
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput;
            return result;
        }

        // Ensure |f(a)| >= |f(b)| (b is the better approximation)
        if (Math.Abs(fa) < Math.Abs(fb))
        {
            (a, b) = (b, a);
            (fa, fb) = (fb, fa);
        }

        double c = a; // Previous iterate
        double fc = fa;
        double d = b - a; // Step size
        double e = d;      // Previous step size

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            double tol = 2.0 * global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON * Math.Abs(b) + 0.5 * config.XTolerance;
            double m = 0.5 * (c - b);

            if (Math.Abs(fb) < config.FTolerance || fb == 0.0 || Math.Abs(m) <= tol)
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                result.Root = b;
                result.Value = fb;
                return result;
            }

            double s = 0.0; // New approximation

            // Try inverse quadratic interpolation if we have three distinct points
            if (Math.Abs(fa - fc) > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL
                && Math.Abs(fb - fc) > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
            {
                // Inverse quadratic interpolation
                s = a * fb * fc / ((fa - fb) * (fa - fc))
                     + b * fa * fc / ((fb - fa) * (fb - fc))
                     + c * fa * fb / ((fc - fa) * (fc - fb));
            }
            else
            {
                // Secant method
                s = b - fb * (b - a) / (fb - fa);
            }

            // Decide whether to accept the interpolation step
            bool useInterp = false;

            // Check if s is between (3a+b)/4 and b
            double bound1 = (3.0 * a + b) / 4.0;
            if ((s > Min(bound1, b) && s < Max(bound1, b)))
            {
                // Accept interpolation if step is smaller than half the previous step
                // (ensures convergence rate). Minimum step is enforced later.
                if (Math.Abs(s - b) < Math.Abs(e) / 2.0)
                {
                    useInterp = true;
                }
            }

            if (!useInterp)
            {
                // Bisection step
                s = b + m;
                e = m;
                d = m;
            }
            else
            {
                e = d;
                d = s - b;
            }

            // Update previous values
            a = b;
            fa = fb;

            // Compute new point, ensuring minimum step
            if (Math.Abs(d) > tol)
            {
                b = s;
            }
            else
            {
                b += (m > 0.0) ? tol : -tol;
            }

            // Evaluate function at new point
            if (!func.Value(b, out fb))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                result.Root = b;
                return result;
            }

            // Update bracket
            if (fb * fc > 0.0)
            {
                c = a;
                fc = fa;
                d = b - a;
                e = d;
            }
            else if (Math.Abs(fc) < Math.Abs(fb))
            {
                // Swap b and c if c is better
                a = b;
                fa = fb;
                (b, c) = (c, b);
                (fb, fc) = (fc, fb);
            }
        }

        // Maximum iterations reached
        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
        result.Root = b;
        result.Value = fb;
        return result;
    }
}
