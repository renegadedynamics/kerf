// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_FRPR.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Optimisation;

internal static partial class MathOpt
{
    /// <summary>Conjugate gradient formula selection.</summary>
    public enum ConjugateGradientFormula
    {
        /// <summary>beta = g_new^T g_new / g^T g (original, guaranteed descent).</summary>
        FletcherReeves,
        /// <summary>beta = g_new^T (g_new - g) / g^T g (often faster, may need restarts).</summary>
        PolakRibiere,
        /// <summary>beta = g_new^T (g_new - g) / d^T (g_new - g).</summary>
        HestenesStiefel,
        /// <summary>beta = g_new^T g_new / d^T (g_new - g).</summary>
        DaiYuan,
    }

    /// <summary>Configuration for the FRPR conjugate gradient method.</summary>
    public class FRPRConfig : Config
    {
        /// <summary>Beta formula.</summary>
        public ConjugateGradientFormula Formula = ConjugateGradientFormula.PolakRibiere;
        /// <summary>Restart every N iterations (0 = n, where n is dimension).</summary>
        public int RestartInterval = 0;

        public FRPRConfig()
        {
        }

        public FRPRConfig(double tolerance, int maxIter = 100)
            : base(tolerance, maxIter)
        {
        }

        public FRPRConfig(FRPRConfig other)
            : base(other)
        {
            Formula = other.Formula;
            RestartInterval = other.RestartInterval;
        }
    }

    /// <summary>Fletcher-Reeves-Polak-Ribiere nonlinear conjugate gradient minimization.</summary>
    public static VectorResult FRPR<Function>(Function func, math_Vector startingPoint, FRPRConfig? config = null)
        where Function : IVectorFunctionWithGradient
    {
        config ??= new FRPRConfig();
        VectorResult result = new();

        int lower = startingPoint.Lower();
        int upper = startingPoint.Upper();
        int n = upper - lower + 1;

        // Restart interval
        int restartInterval = (config.RestartInterval > 0) ? config.RestartInterval : n;

        // Current point
        math_Vector x = new(lower, upper);
        x.Initialized(startingPoint);

        if (!func.Value(x, out double fx))
        {
            result.Status = Status.NumericalError;
            return result;
        }

        // Gradient at current point
        math_Vector grad = new(lower, upper);
        if (!func.Gradient(x, grad))
        {
            result.Status = Status.NumericalError;
            return result;
        }

        // Check if already at minimum
        double gradNormSq = 0.0;
        for (int i = lower; i <= upper; ++i)
        {
            gradNormSq += global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(grad[i]);
        }

        if (Math.Sqrt(gradNormSq) < config.FTolerance)
        {
            result.Status = Status.OK;
            result.Solution = x;
            result.Value = fx;
            result.Gradient = grad;
            return result;
        }

        // Search direction (initially steepest descent)
        math_Vector dir = new(lower, upper);
        for (int i = lower; i <= upper; ++i)
        {
            dir[i] = -grad[i];
        }

        // Working vectors
        math_Vector xNew = new(lower, upper);
        math_Vector gradNew = new(lower, upper);
        math_Vector gradDiff = new(lower, upper);

        int restartCount = 0;

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            // Line search
            LineSearchResult lineResult =
              global::Kerf.Engine.Numerics.Utilities.MathUtils.ArmijoBacktrack(func, x, dir, grad, fx, 1.0, 1.0e-4, 0.5, 50);

            if (!lineResult.IsValid || lineResult.Alpha < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON)
            {
                // Line search failed, try steepest descent
                for (int i = lower; i <= upper; ++i)
                {
                    dir[i] = -grad[i];
                }
                lineResult = global::Kerf.Engine.Numerics.Utilities.MathUtils.ArmijoBacktrack(func, x, dir, grad, fx, 1.0, 1.0e-4, 0.5, 50);

                if (!lineResult.IsValid)
                {
                    result.Status = Status.NotConverged;
                    result.Solution = x;
                    result.Value = fx;
                    result.Gradient = grad;
                    return result;
                }
                restartCount = 0; // Reset restart counter after steepest descent
            }

            // Compute new point
            for (int i = lower; i <= upper; ++i)
            {
                xNew[i] = x[i] + lineResult.Alpha * dir[i];
            }

            // Check X convergence
            double maxDiff = 0.0;
            for (int i = lower; i <= upper; ++i)
            {
                maxDiff = Max(maxDiff, Math.Abs(xNew[i] - x[i]));
            }

            // Evaluate gradient at new point
            if (!func.Gradient(xNew, gradNew))
            {
                result.Status = Status.NumericalError;
                result.Solution = x;
                result.Value = fx;
                return result;
            }

            // Check gradient convergence
            double gradNewNormSq = 0.0;
            for (int i = lower; i <= upper; ++i)
            {
                gradNewNormSq += global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(gradNew[i]);
            }

            if (Math.Sqrt(gradNewNormSq) < config.FTolerance)
            {
                result.Status = Status.OK;
                result.Solution = xNew;
                result.Value = lineResult.FNew;
                result.Gradient = gradNew;
                return result;
            }

            if (maxDiff < config.XTolerance)
            {
                result.Status = Status.OK;
                result.Solution = xNew;
                result.Value = lineResult.FNew;
                result.Gradient = gradNew;
                return result;
            }

            // Compute gradient difference for some formulas
            for (int i = lower; i <= upper; ++i)
            {
                gradDiff[i] = gradNew[i] - grad[i];
            }

            // Compute beta based on selected formula
            double beta = 0.0;
            ++restartCount;

            if (restartCount >= restartInterval)
            {
                // Periodic restart with steepest descent
                beta = 0.0;
                restartCount = 0;
            }
            else
            {
                switch (config.Formula)
                {
                    case ConjugateGradientFormula.FletcherReeves:
                        // beta = g_new^T g_new / g^T g
                        if (gradNormSq > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                        {
                            beta = gradNewNormSq / gradNormSq;
                        }
                        break;

                    case ConjugateGradientFormula.PolakRibiere:
                    {
                        // beta = g_new^T (g_new - g) / g^T g
                        double dot = 0.0;
                        for (int i = lower; i <= upper; ++i)
                        {
                            dot += gradNew[i] * gradDiff[i];
                        }
                        if (gradNormSq > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                        {
                            beta = dot / gradNormSq;
                        }
                        // Restart if beta < 0 (PR+ variant)
                        if (beta < 0.0)
                        {
                            beta = 0.0;
                            restartCount = 0;
                        }
                    }
                    break;

                    case ConjugateGradientFormula.HestenesStiefel:
                    {
                        // beta = g_new^T (g_new - g) / d^T (g_new - g)
                        double num = 0.0;
                        double den = 0.0;
                        for (int i = lower; i <= upper; ++i)
                        {
                            num += gradNew[i] * gradDiff[i];
                            den += dir[i] * gradDiff[i];
                        }
                        if (Math.Abs(den) > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                        {
                            beta = num / den;
                        }
                        if (beta < 0.0)
                        {
                            beta = 0.0;
                            restartCount = 0;
                        }
                    }
                    break;

                    case ConjugateGradientFormula.DaiYuan:
                    {
                        // beta = g_new^T g_new / d^T (g_new - g)
                        double den = 0.0;
                        for (int i = lower; i <= upper; ++i)
                        {
                            den += dir[i] * gradDiff[i];
                        }
                        if (Math.Abs(den) > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                        {
                            beta = gradNewNormSq / den;
                        }
                    }
                    break;
                }
            }

            // Update search direction: p = -g_new + beta * p
            for (int i = lower; i <= upper; ++i)
            {
                dir[i] = -gradNew[i] + beta * dir[i];
            }

            // Check if direction is still a descent direction
            double dirDeriv = 0.0;
            for (int i = lower; i <= upper; ++i)
            {
                dirDeriv += gradNew[i] * dir[i];
            }

            if (dirDeriv >= 0.0)
            {
                // Not a descent direction, restart with steepest descent
                for (int i = lower; i <= upper; ++i)
                {
                    dir[i] = -gradNew[i];
                }
                restartCount = 0;
            }

            // Update for next iteration
            x.Initialized(xNew);
            grad.Initialized(gradNew);
            gradNormSq = gradNewNormSq;
            fx = lineResult.FNew;
        }

        // Maximum iterations reached
        result.Status = Status.MaxIterations;
        result.Solution = x;
        result.Value = fx;
        result.Gradient = grad;
        return result;
    }

    /// <summary>The wrapper class local to FRPRNumerical: adds a central-difference gradient to a value-only function.</summary>
    private sealed class FRPRNumerical_FuncWithGradient<Function> : IVectorFunctionWithGradient
        where Function : IVectorFunction
    {
        private Function _func;
        private readonly double _step;

        public FRPRNumerical_FuncWithGradient(Function f, double step)
        {
            _func = f;
            _step = step;
        }

        public bool Value(math_Vector x, out double f) => _func.Value(x, out f);

        public bool Gradient(math_Vector x, math_Vector grad)
        {
            math_Vector xMod = new(x);
            return global::Kerf.Engine.Numerics.Utilities.MathUtils.NumericalGradientAdaptive(_func, xMod, grad, _step);
        }
    }

    /// <summary>FRPR with a numerical (central-difference) gradient for value-only functions.</summary>
    public static VectorResult FRPRNumerical<Function>(Function func,
                                                       math_Vector startingPoint,
                                                       double gradStep = 1.0e-8,
                                                       FRPRConfig? config = null)
        where Function : IVectorFunction
    {
        config ??= new FRPRConfig();
        FRPRNumerical_FuncWithGradient<Function> wrapper = new(func, gradStep);
        return FRPR(wrapper, startingPoint, config);
    }
}
