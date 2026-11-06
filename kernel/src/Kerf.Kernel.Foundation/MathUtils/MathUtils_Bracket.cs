// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Bracket.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Numerics.Utilities;

/// <summary>Root and minimum bracketing algorithms.</summary>
internal static partial class MathUtils
{
    /// <summary>Result of root bracketing operation.</summary>
    public struct BracketResult
    {
        /// <summary>True if valid bracket found.</summary>
        public bool IsValid;
        /// <summary>Lower bound.</summary>
        public double A;
        /// <summary>Upper bound.</summary>
        public double B;
        /// <summary>Function value at A.</summary>
        public double Fa;
        /// <summary>Function value at B.</summary>
        public double Fb;
    }

    /// <summary>Bracket a root by expanding [theA, theB] outward (golden ratio) until a sign change is found.</summary>
    public static BracketResult BracketRoot<Function>(Function func, double a, double b, int maxIter = 50)
        where Function : IScalarFunction
    {
        BracketResult result = new();
        result.A = a;
        result.B = b;

        if (!func.Value(result.A, out result.Fa))
        {
            return result;
        }
        if (!func.Value(result.B, out result.Fb))
        {
            return result;
        }

        for (int i = 0; i < maxIter; ++i)
        {
            if (result.Fa * result.Fb < 0.0)
            {
                result.IsValid = true;
                // Ensure A < B
                if (result.A > result.B)
                {
                    (result.A, result.B) = (result.B, result.A);
                    (result.Fa, result.Fb) = (result.Fb, result.Fa);
                }
                return result;
            }

            // Expand the interval using golden ratio
            if (Math.Abs(result.Fa) < Math.Abs(result.Fb))
            {
                result.A += THE_GOLDEN_RATIO * (result.A - result.B);
                if (!func.Value(result.A, out result.Fa))
                {
                    return result;
                }
            }
            else
            {
                result.B += THE_GOLDEN_RATIO * (result.B - result.A);
                if (!func.Value(result.B, out result.Fb))
                {
                    return result;
                }
            }
        }

        return result;
    }

    /// <summary>Result of minimum bracketing operation.</summary>
    public struct MinBracketResult
    {
        /// <summary>True if valid bracket found (Fb &lt; Fa and Fb &lt; Fc).</summary>
        public bool IsValid;
        /// <summary>Left bound.</summary>
        public double A;
        /// <summary>Middle point (minimum location estimate).</summary>
        public double B;
        /// <summary>Right bound.</summary>
        public double C;
        public double Fa;
        public double Fb;
        public double Fc;
    }

    /// <summary>Options for minimum bracketing.</summary>
    /// <remarks>A class, as the other solver configurations (see MathUtils_Config.cs).</remarks>
    public class MinBracketOptions
    {
        /// <summary>Maximum iterations.</summary>
        public int MaxIterations = 50;
        /// <summary>Enable hard limits for parameter.</summary>
        public bool UseLimits = false;
        /// <summary>Left hard limit (inclusive).</summary>
        public double LeftLimit = 0.0;
        /// <summary>Right hard limit (inclusive).</summary>
        public double RightLimit = 0.0;
        /// <summary>True if FA is precomputed.</summary>
        public bool HasFA = false;
        /// <summary>True if FB is precomputed.</summary>
        public bool HasFB = false;
        /// <summary>Precomputed f(A).</summary>
        public double FA = 0.0;
        /// <summary>Precomputed f(B).</summary>
        public double FB = 0.0;

        public MinBracketOptions()
        {
        }

        public MinBracketOptions(MinBracketOptions other)
        {
            MaxIterations = other.MaxIterations;
            UseLimits = other.UseLimits;
            LeftLimit = other.LeftLimit;
            RightLimit = other.RightLimit;
            HasFA = other.HasFA;
            HasFB = other.HasFB;
            FA = other.FA;
            FB = other.FB;
        }
    }

    internal static partial class detail
    {
        internal static double Limited(double value, MinBracketOptions options)
        {
            if (!options.UseLimits)
            {
                return value;
            }
            return Max(options.LeftLimit, Min(options.RightLimit, value));
        }

        internal static bool LimitAndMayBeSwap<Function>(Function func,
                                                         MinBracketOptions options,
                                                         double a,
                                                         ref double b,
                                                         ref double fb,
                                                         ref double c,
                                                         ref double fc)
            where Function : IScalarFunction
        {
            c = Limited(c, options);
            if (Math.Abs(b - c) < THE_ZERO_TOL)
            {
                return false;
            }
            if (!func.Value(c, out fc))
            {
                return false;
            }

            // Keep B between A and C
            if ((a - b) * (b - c) < 0.0)
            {
                (b, c) = (c, b);
                (fb, fc) = (fc, fb);
            }
            return true;
        }
    }

    /// <summary>
    /// Bracket a minimum by finding three points a &lt; b &lt; c with f(b) &lt; f(a) and f(b) &lt; f(c), using golden
    /// section expansion with parabolic interpolation.
    /// </summary>
    public static MinBracketResult BracketMinimum<Function>(Function func, double a, double b, MinBracketOptions? options = null)
        where Function : IScalarFunction
    {
        options ??= new MinBracketOptions();
        MinBracketResult result = new();
        if (options.MaxIterations < 1)
        {
            return result;
        }
        if (options.UseLimits && options.LeftLimit > options.RightLimit)
        {
            return result;
        }

        result.A = detail.Limited(a, options);
        result.B = detail.Limited(b, options);
        if (Math.Abs(result.A - result.B) < THE_ZERO_TOL)
        {
            return result;
        }

        bool isUseFA =
          options.HasFA && (!options.UseLimits || Math.Abs(result.A - a) < THE_ZERO_TOL);
        bool isUseFB =
          options.HasFB && (!options.UseLimits || Math.Abs(result.B - b) < THE_ZERO_TOL);

        if (isUseFA)
        {
            result.Fa = options.FA;
        }
        else if (!func.Value(result.A, out result.Fa))
        {
            return result;
        }

        if (isUseFB)
        {
            result.Fb = options.FB;
        }
        else if (!func.Value(result.B, out result.Fb))
        {
            return result;
        }

        // Ensure we go downhill from A to B
        if (result.Fb > result.Fa)
        {
            (result.A, result.B) = (result.B, result.A);
            (result.Fa, result.Fb) = (result.Fb, result.Fa);
        }

        // Initial guess for C using golden ratio
        result.C = result.B + THE_GOLDEN_RATIO * (result.B - result.A);
        if (options.UseLimits)
        {
            if (!detail.LimitAndMayBeSwap(func,
                                          options,
                                          result.A,
                                          ref result.B,
                                          ref result.Fb,
                                          ref result.C,
                                          ref result.Fc))
            {
                return result;
            }
        }
        else if (!func.Value(result.C, out result.Fc))
        {
            return result;
        }

        // Keep expanding until we bracket a minimum
        for (int iter = 0; iter < options.MaxIterations && result.Fb >= result.Fc; ++iter)
        {
            // Parabolic extrapolation
            double r = (result.B - result.A) * (result.Fb - result.Fc);
            double q = (result.B - result.C) * (result.Fb - result.Fa);
            double denom = 2.0 * SignTransfer(Max(Math.Abs(q - r), THE_ZERO_TOL), q - r);

            double u = result.B - ((result.B - result.C) * q - (result.B - result.A) * r) / denom;

            double uLim = result.B + 100.0 * (result.C - result.B);
            if (options.UseLimits)
            {
                uLim = detail.Limited(uLim, options);
            }
            double fu = 0.0;

            if ((result.B - u) * (u - result.C) > 0.0)
            {
                // U is between B and C
                if (!func.Value(u, out fu))
                {
                    return result;
                }

                if (fu < result.Fc)
                {
                    result.A = result.B;
                    result.B = u;
                    result.Fa = result.Fb;
                    result.Fb = fu;
                    result.IsValid = true;
                    return result;
                }
                else if (fu > result.Fb)
                {
                    result.C = u;
                    result.Fc = fu;
                    result.IsValid = true;
                    return result;
                }

                // Parabolic step didn't help, use golden section
                u = result.C + THE_GOLDEN_RATIO * (result.C - result.B);
                if (options.UseLimits)
                {
                    if (!detail.LimitAndMayBeSwap(func, options, result.B, ref result.C, ref result.Fc, ref u, ref fu))
                    {
                        return result;
                    }
                }
                else if (!func.Value(u, out fu))
                {
                    return result;
                }
            }
            else if ((result.C - u) * (u - uLim) > 0.0)
            {
                // U is between C and limit
                if (options.UseLimits)
                {
                    if (!detail.LimitAndMayBeSwap(func, options, result.B, ref result.C, ref result.Fc, ref u, ref fu))
                    {
                        return result;
                    }
                }
                else if (!func.Value(u, out fu))
                {
                    return result;
                }

                if (fu < result.Fc)
                {
                    result.B = result.C;
                    result.C = u;
                    u = result.C + THE_GOLDEN_RATIO * (result.C - result.B);
                    result.Fb = result.Fc;
                    result.Fc = fu;
                    if (options.UseLimits)
                    {
                        if (!detail.LimitAndMayBeSwap(func, options, result.B, ref result.C, ref result.Fc, ref u, ref fu))
                        {
                            return result;
                        }
                    }
                    else if (!func.Value(u, out fu))
                    {
                        return result;
                    }
                }
            }
            else if ((u - uLim) * (uLim - result.C) >= 0.0)
            {
                // U is beyond limit
                u = uLim;
                if (options.UseLimits)
                {
                    if (!detail.LimitAndMayBeSwap(func, options, result.B, ref result.C, ref result.Fc, ref u, ref fu))
                    {
                        return result;
                    }
                }
                else if (!func.Value(u, out fu))
                {
                    return result;
                }
            }
            else
            {
                // Default golden section step
                u = result.C + THE_GOLDEN_RATIO * (result.C - result.B);
                if (options.UseLimits)
                {
                    if (!detail.LimitAndMayBeSwap(func, options, result.B, ref result.C, ref result.Fc, ref u, ref fu))
                    {
                        return result;
                    }
                }
                else if (!func.Value(u, out fu))
                {
                    return result;
                }
            }

            // Shift points
            result.A = result.B;
            result.B = result.C;
            result.C = u;
            result.Fa = result.Fb;
            result.Fb = result.Fc;
            result.Fc = fu;
        }

        result.IsValid = (result.Fb < result.Fa && result.Fb < result.Fc);

        // Ensure A < B < C ordering
        if (result.IsValid && result.A > result.C)
        {
            (result.A, result.C) = (result.C, result.A);
            (result.Fa, result.Fc) = (result.Fc, result.Fa);
        }

        if (result.IsValid && !(result.A < result.B && result.B < result.C))
        {
            result.IsValid = false;
        }

        return result;
    }

    /// <summary>Backward-compatible convenience overload with only max-iterations argument.</summary>
    public static MinBracketResult BracketMinimum<Function>(Function func, double a, double b, int maxIter)
        where Function : IScalarFunction
    {
        MinBracketOptions options = new();
        options.MaxIterations = maxIter;
        return BracketMinimum(func, a, b, options);
    }
}
