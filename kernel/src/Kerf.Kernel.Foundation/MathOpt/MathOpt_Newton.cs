// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_Newton.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Optimisation;

internal static partial class MathOpt
{
    /// <summary>Configuration for Newton minimization with Hessian.</summary>
    public class NewtonConfig : Config
    {
        /// <summary>Diagonal regularization for non-positive definite Hessian.</summary>
        public double Regularization = 1.0e-8;
        /// <summary>Whether to use line search (recommended).</summary>
        public bool UseLineSearch = true;

        public NewtonConfig()
        {
        }

        public NewtonConfig(double tolerance, int maxIter = 100)
            : base(tolerance, maxIter)
        {
        }

        public NewtonConfig(NewtonConfig other)
            : base(other)
        {
            Regularization = other.Regularization;
            UseLineSearch = other.UseLineSearch;
        }
    }

    /// <summary>
    /// Newton's method for N-dimensional minimization using the Hessian, with line search and diagonal
    /// regularization when the Hessian system can't be solved.
    /// </summary>
    public static VectorResult Newton<Function>(Function func, math_Vector startingPoint, NewtonConfig? config = null)
        where Function : IVectorFunctionWithHessian
    {
        config ??= new NewtonConfig();
        VectorResult result = new();

        int lower = startingPoint.Lower();
        int upper = startingPoint.Upper();

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
        double gradNorm = 0.0;
        for (int i = lower; i <= upper; ++i)
        {
            gradNorm += global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(grad[i]);
        }
        gradNorm = Math.Sqrt(gradNorm);

        if (gradNorm < config.FTolerance)
        {
            result.Status = Status.OK;
            result.Solution = x;
            result.Value = fx;
            result.Gradient = grad;
            return result;
        }

        // Working vectors and matrices
        math_Vector dir = new(lower, upper);
        math_Vector xNew = new(lower, upper);
        math_Vector gradNew = new(lower, upper);
        Matrix hessian = new(lower, upper, lower, upper);
        math_Vector negGrad = new(lower, upper);

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            // Compute Hessian
            if (!func.Hessian(x, hessian))
            {
                result.Status = Status.NumericalError;
                result.Solution = x;
                result.Value = fx;
                return result;
            }

            // Prepare negative gradient
            for (int i = lower; i <= upper; ++i)
            {
                negGrad[i] = -grad[i];
            }

            // Try to solve H * p = -g
            LinearResult linResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(hessian, negGrad);

            if (!linResult.IsDone)
            {
                // Hessian is singular or not positive definite, add regularization
                double lambda = config.Regularization;
                bool solved = false;

                for (int k = 0; k < 10 && !solved; ++k)
                {
                    Matrix regHessian = new(hessian);
                    for (int i = lower; i <= upper; ++i)
                    {
                        regHessian[i, i] += lambda;
                    }

                    linResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(regHessian, negGrad);
                    if (linResult.IsDone)
                    {
                        solved = true;
                    }
                    else
                    {
                        lambda *= 10.0;
                    }
                }

                if (!solved)
                {
                    // Fall back to steepest descent
                    for (int i = lower; i <= upper; ++i)
                    {
                        dir[i] = -grad[i];
                    }
                    goto perform_line_search;
                }
            }

            dir.Initialized(linResult.Solution!);

            // Check if direction is descent
            {
                double dirDeriv = 0.0;
                for (int i = lower; i <= upper; ++i)
                {
                    dirDeriv += grad[i] * dir[i];
                }

                if (dirDeriv >= 0.0)
                {
                    // Not a descent direction, use steepest descent
                    for (int i = lower; i <= upper; ++i)
                    {
                        dir[i] = -grad[i];
                    }
                }
            }

        perform_line_search:
            if (config.UseLineSearch)
            {
                // Line search
                LineSearchResult lineResult =
                  global::Kerf.Engine.Numerics.Utilities.MathUtils.ArmijoBacktrack(func, x, dir, grad, fx, 1.0, 1.0e-4, 0.5, 50);

                if (!lineResult.IsValid)
                {
                    // Line search failed, try steepest descent
                    for (int i = lower; i <= upper; ++i)
                    {
                        dir[i] = -grad[i];
                    }
                    lineResult =
                      global::Kerf.Engine.Numerics.Utilities.MathUtils.ArmijoBacktrack(func, x, dir, grad, fx, 1.0, 1.0e-4, 0.5, 50);

                    if (!lineResult.IsValid)
                    {
                        result.Status = Status.NotConverged;
                        result.Solution = x;
                        result.Value = fx;
                        result.Gradient = grad;
                        return result;
                    }
                }

                // Compute new point
                for (int i = lower; i <= upper; ++i)
                {
                    xNew[i] = x[i] + lineResult.Alpha * dir[i];
                }
                fx = lineResult.FNew;
            }
            else
            {
                // Full Newton step (no line search)
                for (int i = lower; i <= upper; ++i)
                {
                    xNew[i] = x[i] + dir[i];
                }

                if (!func.Value(xNew, out fx))
                {
                    result.Status = Status.NumericalError;
                    result.Solution = x;
                    return result;
                }
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
                result.Solution = xNew;
                result.Value = fx;
                return result;
            }

            // Check gradient convergence
            gradNorm = 0.0;
            for (int i = lower; i <= upper; ++i)
            {
                gradNorm += global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(gradNew[i]);
            }
            gradNorm = Math.Sqrt(gradNorm);

            if (gradNorm < config.FTolerance)
            {
                result.Status = Status.OK;
                result.Solution = xNew;
                result.Value = fx;
                result.Gradient = gradNew;
                return result;
            }

            if (maxDiff < config.XTolerance)
            {
                result.Status = Status.OK;
                result.Solution = xNew;
                result.Value = fx;
                result.Gradient = gradNew;
                return result;
            }

            // Update for next iteration
            x.Initialized(xNew);
            grad.Initialized(gradNew);
        }

        // Maximum iterations reached
        result.Status = Status.MaxIterations;
        result.Solution = x;
        result.Value = fx;
        result.Gradient = grad;
        return result;
    }

    /// <summary>Modified Newton's method with automatic Hessian regularization: the same as Newton.</summary>
    public static VectorResult NewtonModified<Function>(Function func, math_Vector startingPoint, NewtonConfig? config = null)
        where Function : IVectorFunctionWithHessian
    {
        config ??= new NewtonConfig();
        return Newton(func, startingPoint, config);
    }

    /// <summary>The wrapper class local to NewtonNumericalHessian: adds a finite-difference Hessian.</summary>
    private sealed class NewtonNumericalHessian_FuncWithHessian<Function> : IVectorFunctionWithHessian
        where Function : IVectorFunctionWithGradient
    {
        private Function _func;
        private readonly double _step;

        public NewtonNumericalHessian_FuncWithHessian(Function f, double step)
        {
            _func = f;
            _step = step;
        }

        public bool Value(math_Vector x, out double f) => _func.Value(x, out f);

        public bool Gradient(math_Vector x, math_Vector grad)
        {
            return _func.Gradient(x, grad);
        }

        public bool Hessian(math_Vector x, Matrix hess)
        {
            math_Vector xMod = new(x);
            // OCCT: NumericalHessian fills the matrix from index 1, but Newton's Hessian has the starting point's
            // bounds: with a lower bound other than 1 the Hessian is shifted or out of range.
            return global::Kerf.Engine.Numerics.Utilities.MathUtils.NumericalHessian(_func, xMod, hess, _step);
        }
    }

    /// <summary>Newton's method with a numerical (finite-difference) Hessian.</summary>
    public static VectorResult NewtonNumericalHessian<Function>(Function func,
                                                                math_Vector startingPoint,
                                                                double hessStep = 1.0e-6,
                                                                NewtonConfig? config = null)
        where Function : IVectorFunctionWithGradient
    {
        config ??= new NewtonConfig();
        NewtonNumericalHessian_FuncWithHessian<Function> wrapper = new(func, hessStep);
        return Newton(wrapper, startingPoint, config);
    }

    /// <summary>The wrapper class local to NewtonNumerical: adds a numerical gradient and Hessian.</summary>
    private sealed class NewtonNumerical_FuncWithDerivatives<Function> : IVectorFunctionWithHessian
        where Function : IVectorFunction
    {
        private Function _func;
        private readonly double _gradStep;
        private readonly double _hessStep;

        public NewtonNumerical_FuncWithDerivatives(Function f, double gStep, double hStep)
        {
            _func = f;
            _gradStep = gStep;
            _hessStep = hStep;
        }

        public bool Value(math_Vector x, out double f) => _func.Value(x, out f);

        public bool Gradient(math_Vector x, math_Vector grad)
        {
            math_Vector xMod = new(x);
            return global::Kerf.Engine.Numerics.Utilities.MathUtils.NumericalGradientAdaptive(_func, xMod, grad, _gradStep);
        }

        public bool Hessian(math_Vector x, Matrix hess)
        {
            // Compute Hessian from finite differences of gradient
            int lower = x.Lower();
            int upper = x.Upper();

            math_Vector xMod = new(x);
            math_Vector gradPlus = new(lower, upper);
            math_Vector gradMinus = new(lower, upper);

            for (int j = lower; j <= upper; ++j)
            {
                double xj = xMod[j];

                xMod[j] = xj + _hessStep;
                if (!global::Kerf.Engine.Numerics.Utilities.MathUtils.NumericalGradientAdaptive(_func, xMod, gradPlus, _gradStep))
                {
                    xMod[j] = xj;
                    return false;
                }

                xMod[j] = xj - _hessStep;
                if (!global::Kerf.Engine.Numerics.Utilities.MathUtils.NumericalGradientAdaptive(_func, xMod, gradMinus, _gradStep))
                {
                    xMod[j] = xj;
                    return false;
                }

                xMod[j] = xj;

                for (int i = lower; i <= upper; ++i)
                {
                    hess[i, j] = (gradPlus[i] - gradMinus[i]) / (2.0 * _hessStep);
                }
            }

            // Symmetrize
            for (int i = lower; i <= upper; ++i)
            {
                for (int j = i + 1; j <= upper; ++j)
                {
                    double avg = 0.5 * (hess[i, j] + hess[j, i]);
                    hess[i, j] = avg;
                    hess[j, i] = avg;
                }
            }

            return true;
        }
    }

    /// <summary>Newton's method with numerical gradient and Hessian for value-only functions.</summary>
    public static VectorResult NewtonNumerical<Function>(Function func,
                                                         math_Vector startingPoint,
                                                         double gradStep = 1.0e-8,
                                                         double hessStep = 1.0e-6,
                                                         NewtonConfig? config = null)
        where Function : IVectorFunction
    {
        config ??= new NewtonConfig();
        NewtonNumerical_FuncWithDerivatives<Function> wrapper = new(func, gradStep, hessStep);
        return Newton(wrapper, startingPoint, config);
    }

    /// <summary>Newton's method with box constraints, by projection (as BFGSBounded).</summary>
    public static VectorResult NewtonBounded<Function>(Function func,
                                                       math_Vector startingPoint,
                                                       math_Vector lowerBounds,
                                                       math_Vector upperBounds,
                                                       NewtonConfig? config = null)
        where Function : IVectorFunctionWithHessian
    {
        config ??= new NewtonConfig();
        VectorResult result = new();

        int lower = startingPoint.Lower();
        int upper = startingPoint.Upper();
        int n = upper - lower + 1;

        // Check dimensions
        if (lowerBounds.Length() != n || upperBounds.Length() != n)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        // Lambda to clamp a point to bounds
        void ClampToBounds(math_Vector x)
        {
            for (int i = lower; i <= upper; ++i)
            {
                int bndIdx = lowerBounds.Lower() + (i - lower);
                if (x[i] < lowerBounds[bndIdx])
                {
                    x[i] = lowerBounds[bndIdx];
                }
                if (x[i] > upperBounds[bndIdx])
                {
                    x[i] = upperBounds[bndIdx];
                }
            }
        }

        // Lambda to project gradient (zero components at active bounds)
        void ProjectGradient(math_Vector x, math_Vector grad)
        {
            for (int i = lower; i <= upper; ++i)
            {
                int bndIdx = lowerBounds.Lower() + (i - lower);
                double tol = global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON * Max(1.0, Math.Abs(x[i]));

                if (x[i] - lowerBounds[bndIdx] < tol && grad[i] > 0.0)
                {
                    grad[i] = 0.0;
                }
                if (upperBounds[bndIdx] - x[i] < tol && grad[i] < 0.0)
                {
                    grad[i] = 0.0;
                }
            }
        }

        // Lambda to compute max step to boundary
        double ComputeAlphaMax(math_Vector x, math_Vector dir)
        {
            double alphaMax = 1.0;
            for (int i = lower; i <= upper; ++i)
            {
                int bndIdx = lowerBounds.Lower() + (i - lower);
                if (dir[i] < -global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON)
                {
                    double maxStep = (lowerBounds[bndIdx] - x[i]) / dir[i];
                    alphaMax = Min(alphaMax, maxStep);
                }
                else if (dir[i] > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON)
                {
                    double maxStep = (upperBounds[bndIdx] - x[i]) / dir[i];
                    alphaMax = Min(alphaMax, maxStep);
                }
            }
            return Max(alphaMax, global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON);
        }

        // Current point
        math_Vector aX = new(lower, upper);
        aX.Initialized(startingPoint);
        ClampToBounds(aX);

        if (!func.Value(aX, out double fx))
        {
            result.Status = Status.NumericalError;
            return result;
        }

        // Gradient at current point
        math_Vector aGrad = new(lower, upper);
        if (!func.Gradient(aX, aGrad))
        {
            result.Status = Status.NumericalError;
            return result;
        }
        ProjectGradient(aX, aGrad);

        // Check if already at minimum
        double gradNorm = 0.0;
        for (int i = lower; i <= upper; ++i)
        {
            gradNorm += global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(aGrad[i]);
        }
        gradNorm = Math.Sqrt(gradNorm);

        if (gradNorm < config.FTolerance)
        {
            result.Status = Status.OK;
            result.Solution = aX;
            result.Value = fx;
            result.Gradient = aGrad;
            return result;
        }

        // Working vectors and matrices
        math_Vector aDir = new(lower, upper);
        math_Vector xNew = new(lower, upper);
        math_Vector gradNew = new(lower, upper);
        Matrix hessian = new(lower, upper, lower, upper);
        math_Vector negGrad = new(lower, upper);

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            // Compute Hessian
            if (!func.Hessian(aX, hessian))
            {
                result.Status = Status.NumericalError;
                result.Solution = aX;
                result.Value = fx;
                return result;
            }

            // Prepare negative gradient
            for (int i = lower; i <= upper; ++i)
            {
                negGrad[i] = -aGrad[i];
            }

            // Try to solve H * p = -g
            LinearResult linResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(hessian, negGrad);

            if (!linResult.IsDone)
            {
                // Add regularization
                double lambda = config.Regularization;
                bool solved = false;

                for (int k = 0; k < 10 && !solved; ++k)
                {
                    Matrix regHessian = new(hessian);
                    for (int i = lower; i <= upper; ++i)
                    {
                        regHessian[i, i] += lambda;
                    }

                    linResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(regHessian, negGrad);
                    if (linResult.IsDone)
                    {
                        solved = true;
                    }
                    else
                    {
                        lambda *= 10.0;
                    }
                }

                if (!solved)
                {
                    // Fall back to steepest descent
                    for (int i = lower; i <= upper; ++i)
                    {
                        aDir[i] = -aGrad[i];
                    }
                    goto perform_bounded_line_search;
                }
            }

            aDir.Initialized(linResult.Solution!);

            // Check if direction is descent
            {
                double dirDeriv = 0.0;
                for (int i = lower; i <= upper; ++i)
                {
                    dirDeriv += aGrad[i] * aDir[i];
                }

                if (dirDeriv >= 0.0)
                {
                    for (int i = lower; i <= upper; ++i)
                    {
                        aDir[i] = -aGrad[i];
                    }
                }
            }

        perform_bounded_line_search:
            if (config.UseLineSearch)
            {
                double alphaMax = ComputeAlphaMax(aX, aDir);

                LineSearchResult lineResult =
                  global::Kerf.Engine.Numerics.Utilities.MathUtils.ArmijoBacktrack(func, aX, aDir, aGrad, fx, alphaMax, 1.0e-4, 0.5, 50);

                if (!lineResult.IsValid)
                {
                    // Try steepest descent
                    for (int i = lower; i <= upper; ++i)
                    {
                        aDir[i] = -aGrad[i];
                    }
                    alphaMax = ComputeAlphaMax(aX, aDir);
                    lineResult =
                      global::Kerf.Engine.Numerics.Utilities.MathUtils.ArmijoBacktrack(func, aX, aDir, aGrad, fx, alphaMax, 1.0e-4, 0.5, 50);

                    if (!lineResult.IsValid)
                    {
                        result.Status = Status.NotConverged;
                        result.Solution = aX;
                        result.Value = fx;
                        result.Gradient = aGrad;
                        return result;
                    }
                }

                for (int i = lower; i <= upper; ++i)
                {
                    xNew[i] = aX[i] + lineResult.Alpha * aDir[i];
                }
                ClampToBounds(xNew);

                if (!func.Value(xNew, out fx))
                {
                    result.Status = Status.NumericalError;
                    result.Solution = aX;
                    return result;
                }
            }
            else
            {
                for (int i = lower; i <= upper; ++i)
                {
                    xNew[i] = aX[i] + aDir[i];
                }
                ClampToBounds(xNew);

                if (!func.Value(xNew, out fx))
                {
                    result.Status = Status.NumericalError;
                    result.Solution = aX;
                    return result;
                }
            }

            // Check X convergence
            double maxDiff = 0.0;
            for (int i = lower; i <= upper; ++i)
            {
                maxDiff = Max(maxDiff, Math.Abs(xNew[i] - aX[i]));
            }

            // Evaluate gradient at new point
            if (!func.Gradient(xNew, gradNew))
            {
                result.Status = Status.NumericalError;
                result.Solution = xNew;
                result.Value = fx;
                return result;
            }
            ProjectGradient(xNew, gradNew);

            // Check gradient convergence
            gradNorm = 0.0;
            for (int i = lower; i <= upper; ++i)
            {
                gradNorm += global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(gradNew[i]);
            }
            gradNorm = Math.Sqrt(gradNorm);

            if (gradNorm < config.FTolerance)
            {
                result.Status = Status.OK;
                result.Solution = xNew;
                result.Value = fx;
                result.Gradient = gradNew;
                return result;
            }

            if (maxDiff < config.XTolerance)
            {
                result.Status = Status.OK;
                result.Solution = xNew;
                result.Value = fx;
                result.Gradient = gradNew;
                return result;
            }

            aX.Initialized(xNew);
            aGrad.Initialized(gradNew);
        }

        result.Status = Status.MaxIterations;
        result.Solution = aX;
        result.Value = fx;
        result.Gradient = aGrad;
        return result;
    }
}
