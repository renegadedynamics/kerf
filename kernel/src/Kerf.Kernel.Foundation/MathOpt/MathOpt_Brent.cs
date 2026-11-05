// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_Brent.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Optimisation;

/// <summary>Optimization algorithms for scalar and vector functions.</summary>
internal static partial class MathOpt
{
    /// <summary>
    /// Brent's method for 1D minimization on [theLower, theUpper]: golden section search with parabolic
    /// interpolation. Root holds the minimum location, Value the minimum.
    /// </summary>
    public static ScalarResult Brent<Function>(Function func, double lower, double upper, Config? config = null)
        where Function : IScalarFunction
    {
        config ??= new Config();
        ScalarResult result = new();

        double a = lower;
        double b = upper;

        // Initial point using golden section
        double x = a + global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_GOLDEN_SECTION * (b - a);
        double w = x;
        double v = x;

        if (!func.Value(x, out double fx))
        {
            result.Status = Status.NumericalError;
            return result;
        }
        double fw = fx;
        double fv = fx;

        double d = 0.0; // Current step
        double e = 0.0; // Previous step

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            double xm = 0.5 * (a + b);
            double tol1 = config.XTolerance * Math.Abs(x) + global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL / 10.0;
            double tol2 = 2.0 * tol1;

            result.NbIterations = iter + 1;

            // Check convergence
            if (Math.Abs(x - xm) <= (tol2 - 0.5 * (b - a)))
            {
                result.Status = Status.OK;
                result.Root = x;
                result.Value = fx;
                return result;
            }

            double u = 0.0;
            bool useParabolic = false;

            // Try parabolic interpolation if step is large enough
            if (Math.Abs(e) > tol1)
            {
                // Parabolic fit through x, w, v
                double r = (x - w) * (fx - fv);
                double q = (x - v) * (fx - fw);
                double p = (x - v) * q - (x - w) * r;
                q = 2.0 * (q - r);

                if (q > 0.0)
                {
                    p = -p;
                }
                else
                {
                    q = -q;
                }

                double eTmp = e;
                e = d;

                // Check if parabolic step is acceptable
                if (Math.Abs(p) < Math.Abs(0.5 * q * eTmp) && p > q * (a - x) && p < q * (b - x))
                {
                    d = p / q;
                    u = x + d;

                    // Don't evaluate too close to bounds
                    if ((u - a) < tol2 || (b - u) < tol2)
                    {
                        d = global::Kerf.Engine.Numerics.Utilities.MathUtils.SignTransfer(tol1, xm - x);
                    }
                    useParabolic = true;
                }
            }

            if (!useParabolic)
            {
                // Golden section step
                e = (x < xm) ? (b - x) : (a - x);
                d = global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_GOLDEN_SECTION * e;
            }

            // Ensure step is at least aTol1
            if (Math.Abs(d) >= tol1)
            {
                u = x + d;
            }
            else
            {
                u = x + global::Kerf.Engine.Numerics.Utilities.MathUtils.SignTransfer(tol1, d);
            }

            if (!func.Value(u, out double fu))
            {
                result.Status = Status.NumericalError;
                result.Root = x;
                result.Value = fx;
                return result;
            }

            // Update bracket and best points
            if (fu <= fx)
            {
                if (u < x)
                {
                    b = x;
                }
                else
                {
                    a = x;
                }

                v = w;
                w = x;
                x = u;
                fv = fw;
                fw = fx;
                fx = fu;
            }
            else
            {
                if (u < x)
                {
                    a = u;
                }
                else
                {
                    b = u;
                }

                if (fu <= fw || w == x)
                {
                    v = w;
                    w = u;
                    fv = fw;
                    fw = fu;
                }
                else if (fu <= fv || v == x || v == w)
                {
                    v = u;
                    fv = fu;
                }
            }
        }

        // Maximum iterations reached
        result.Status = Status.MaxIterations;
        result.Root = x;
        result.Value = fx;
        return result;
    }

    /// <summary>Golden section search for 1D minimization on [theLower, theUpper] (no parabolic steps).</summary>
    public static ScalarResult Golden<Function>(Function func, double lower, double upper, Config? config = null)
        where Function : IScalarFunction
    {
        config ??= new Config();
        ScalarResult result = new();

        const double r = 0.618033988749895; // (sqrt(5) - 1) / 2
        const double c = 1.0 - r;

        double a = lower;
        double b = upper;

        // Initialize interior points
        double x1 = a + c * (b - a);
        double x2 = a + r * (b - a);

        if (!func.Value(x1, out double f1))
        {
            result.Status = Status.NumericalError;
            return result;
        }
        if (!func.Value(x2, out double f2))
        {
            result.Status = Status.NumericalError;
            return result;
        }

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            // Check convergence
            if ((b - a) < config.XTolerance * (Math.Abs(x1) + Math.Abs(x2)))
            {
                result.Status = Status.OK;
                if (f1 < f2)
                {
                    result.Root = x1;
                    result.Value = f1;
                }
                else
                {
                    result.Root = x2;
                    result.Value = f2;
                }
                return result;
            }

            if (f1 < f2)
            {
                // Minimum is in [a, x2]
                b = x2;
                x2 = x1;
                f2 = f1;
                x1 = a + c * (b - a);
                if (!func.Value(x1, out f1))
                {
                    result.Status = Status.NumericalError;
                    result.Root = x2;
                    result.Value = f2;
                    return result;
                }
            }
            else
            {
                // Minimum is in [x1, b]
                a = x1;
                x1 = x2;
                f1 = f2;
                x2 = a + r * (b - a);
                if (!func.Value(x2, out f2))
                {
                    result.Status = Status.NumericalError;
                    result.Root = x1;
                    result.Value = f1;
                    return result;
                }
            }
        }

        // Maximum iterations reached
        result.Status = Status.MaxIterations;
        if (f1 < f2)
        {
            result.Root = x1;
            result.Value = f1;
        }
        else
        {
            result.Root = x2;
            result.Value = f2;
        }
        return result;
    }

    /// <summary>Brent's method after bracketing a minimum from theGuess with an initial step theStep.</summary>
    public static ScalarResult BrentWithBracket<Function>(Function func, double guess, double step = 1.0, Config? config = null)
        where Function : IScalarFunction
    {
        config ??= new Config();
        ScalarResult result = new();

        // Try to bracket minimum
        MinBracketResult bracket = global::Kerf.Engine.Numerics.Utilities.MathUtils.BracketMinimum(func, guess, guess + step);

        if (!bracket.IsValid)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        // Apply Brent's method on the bracket
        return Brent(func, bracket.A, bracket.C, config);
    }
}
