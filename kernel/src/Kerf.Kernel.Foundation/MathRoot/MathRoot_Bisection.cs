// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_Bisection.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Roots;

internal static partial class MathRoot
{
    /// <summary>Bisection method for root finding; f(theLower) and f(theUpper) must have opposite signs.</summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult Bisection<Function>(Function func, double lower, double upper, global::Kerf.Engine.Numerics.Utilities.MathUtils.Config? config = null)
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

        // Ensure a < b
        if (a > b)
        {
            (a, b) = (b, a);
            (fa, fb) = (fb, fa);
        }

        // Track which endpoint has the negative value
        // We use separate tracking instead of swapping to maintain aA < aB
        bool isNegAtA = (fa < 0.0);

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            // Compute midpoint
            double m = 0.5 * (a + b);

            if (!func.Value(m, out double fm))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                result.Root = m;
                result.NbIterations = iter + 1;
                return result;
            }

            result.NbIterations = iter + 1;

            // Check convergence on function value
            if (Math.Abs(fm) < config.FTolerance)
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                result.Root = m;
                result.Value = fm;
                return result;
            }

            // Check convergence on interval size
            if ((b - a) < config.XTolerance * Max(1.0, Math.Abs(m)))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                result.Root = m;
                result.Value = fm;
                return result;
            }

            // Update bracket based on sign tracking
            bool isNegAtM = (fm < 0.0);
            if (isNegAtM == isNegAtA)
            {
                // Same sign as endpoint A, replace A
                a = m;
                fa = fm;
            }
            else
            {
                // Same sign as endpoint B, replace B
                b = m;
                fb = fm;
            }
        }

        // Maximum iterations reached
        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
        result.Root = 0.5 * (a + b);
        func.Value(result.Root.Value, out double value);
        result.Value = value;
        return result;
    }

    /// <summary>Bisection safeguarded Newton (Newton steps that stay inside the bracket, bisection otherwise).</summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult BisectionNewton<Function>(Function func, double lower, double upper, global::Kerf.Engine.Numerics.Utilities.MathUtils.Config? config = null)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
    {
        config ??= new global::Kerf.Engine.Numerics.Utilities.MathUtils.Config();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult result = new();

        double a = lower;
        double b = upper;

        // Evaluate at endpoints
        if (!func.Values(a, out double fa, out _))
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
            return result;
        }
        if (!func.Values(b, out double fb, out _))
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
            return result;
        }

        // Check that bracket is valid
        if (fa * fb > 0.0)
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput;
            return result;
        }

        // Ensure a < b
        if (a > b)
        {
            (a, b) = (b, a);
            (fa, fb) = (fb, fa);
        }

        // Track which endpoint has the negative value
        bool isNegAtA = (fa < 0.0);

        // Start from midpoint
        double x = 0.5 * (a + b);

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            if (!func.Values(x, out double fx, out double dfx))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                result.Root = x;
                result.NbIterations = iter + 1;
                return result;
            }

            result.NbIterations = iter + 1;

            // Check convergence
            if (Math.Abs(fx) < config.FTolerance)
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                result.Root = x;
                result.Value = fx;
                result.Derivative = dfx;
                return result;
            }

            if ((b - a) < config.XTolerance * Max(1.0, Math.Abs(x)))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                result.Root = x;
                result.Value = fx;
                result.Derivative = dfx;
                return result;
            }

            // Try Newton step
            double xNew = x;
            if (!global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(dfx))
            {
                xNew = x - fx / dfx;
            }

            // Check if Newton step stays within bracket
            if (xNew <= a || xNew >= b)
            {
                // Fall back to bisection
                xNew = 0.5 * (a + b);
            }

            // Update bracket based on current point using sign tracking
            bool isNegAtX = (fx < 0.0);
            if (isNegAtX == isNegAtA)
            {
                // Same sign as endpoint A, replace A
                a = x;
            }
            else
            {
                // Same sign as endpoint B, replace B
                b = x;
            }

            x = xNew;
        }

        // Maximum iterations reached
        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
        result.Root = x;
        func.Values(x, out double value, out double derivative);
        result.Value = value;
        result.Derivative = derivative;
        return result;
    }
}
