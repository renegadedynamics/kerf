// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_Secant.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Roots;

internal static partial class MathRoot
{
    /// <summary>Secant method from two initial points (no derivative needed).</summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult Secant<Function>(Function func, double x0, double x1, global::Kerf.Engine.Numerics.Utilities.MathUtils.Config? config = null)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        config ??= new global::Kerf.Engine.Numerics.Utilities.MathUtils.Config();
        global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult result = new();

        double aX0 = x0;
        double aX1 = x1;

        // Evaluate at initial points
        if (!func.Value(aX0, out double f0))
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
            return result;
        }
        if (!func.Value(aX1, out double f1))
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
            return result;
        }

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            // Check convergence
            if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsFConverged(f1, config.FTolerance))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                result.Root = aX1;
                result.Value = f1;
                return result;
            }

            // Secant step
            double denom = f1 - f0;
            if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(denom))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                result.Root = aX1;
                result.Value = f1;
                return result;
            }

            double xNew = aX1 - f1 * (aX1 - aX0) / denom;

            // Check X convergence
            if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsXConverged(aX1, xNew, config.XTolerance))
            {
                func.Value(xNew, out double fNew);
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                result.Root = xNew;
                result.Value = fNew;
                return result;
            }

            // Update for next iteration
            aX0 = aX1;
            f0 = f1;
            aX1 = xNew;

            if (!func.Value(aX1, out f1))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                result.Root = aX1;
                return result;
            }
        }

        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
        result.Root = aX1;
        result.Value = f1;
        return result;
    }

    /// <summary>Secant method with the second point a small perturbation of theX0.</summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult SecantAuto<Function>(Function func, double x0, global::Kerf.Engine.Numerics.Utilities.MathUtils.Config? config = null)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        // Create second point by small perturbation
        double delta = (Math.Abs(x0) > 1.0) ? 0.01 * x0 : 0.01;
        return Secant(func, x0, x0 + delta, config);
    }
}
