// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_BFGS.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Optimisation;

public static partial class MathOpt
{
    /// <summary>
    /// BFGS quasi-Newton minimization with Armijo backtracking (falling back to steepest descent when the line
    /// search fails).
    /// </summary>
    public static VectorResult BFGS<Function>(Function func, math_Vector startingPoint, Config? config = null)
        where Function : IVectorFunctionWithGradient
    {
        config ??= new Config();
        VectorResult result = new();

        int lower = startingPoint.Lower();
        int upper = startingPoint.Upper();
        int n = upper - lower + 1;

        // Current point and function value
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

        // Check if already at minimum (gradient near zero)
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

        // Initialize inverse Hessian approximation to identity
        Matrix h = new(1, n, 1, n, 0.0);
        for (int i = 1; i <= n; ++i)
        {
            h[i, i] = 1.0;
        }

        // Working vectors
        math_Vector dir = new(lower, upper);     // Search direction
        math_Vector xNew = new(lower, upper);    // New point
        math_Vector gradNew = new(lower, upper); // New gradient
        math_Vector s = new(1, n);                // Step: x_new - x
        math_Vector y = new(1, n);                // Gradient difference: grad_new - grad

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            // Compute search direction: p = -H * grad
            for (int i = 1; i <= n; ++i)
            {
                double sum = 0.0;
                for (int j = 1; j <= n; ++j)
                {
                    sum += h[i, j] * grad[lower + j - 1];
                }
                dir[lower + i - 1] = -sum;
            }

            // Line search with Wolfe conditions
            LineSearchResult lineResult =
              global::Kerf.Engine.Numerics.Utilities.MathUtils.ArmijoBacktrack(func, x, dir, grad, fx, 1.0, 1.0e-4, 0.5, 50);

            if (!lineResult.IsValid || lineResult.Alpha < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON)
            {
                // Line search failed, try steepest descent direction
                for (int i = lower; i <= upper; ++i)
                {
                    dir[i] = -grad[i];
                }
                lineResult = global::Kerf.Engine.Numerics.Utilities.MathUtils.ArmijoBacktrack(func, x, dir, grad, fx, 1.0, 1.0e-4, 0.5, 50);

                if (!lineResult.IsValid)
                {
                    // Both BFGS and steepest descent failed
                    result.Status = Status.NotConverged;
                    result.Solution = x;
                    result.Value = fx;
                    result.Gradient = grad;
                    return result;
                }

                // Reset Hessian to identity after steepest descent step
                for (int i = 1; i <= n; ++i)
                {
                    for (int j = 1; j <= n; ++j)
                    {
                        h[i, j] = (i == j) ? 1.0 : 0.0;
                    }
                }
            }

            // Compute new point
            for (int i = lower; i <= upper; ++i)
            {
                xNew[i] = x[i] + lineResult.Alpha * dir[i];
            }

            // Compute s = x_new - x
            for (int i = 1; i <= n; ++i)
            {
                s[i] = xNew[lower + i - 1] - x[lower + i - 1];
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
                result.Value = lineResult.FNew;
                result.Gradient = gradNew;
                return result;
            }

            // Check X convergence
            double maxDiff = 0.0;
            for (int i = lower; i <= upper; ++i)
            {
                maxDiff = Max(maxDiff, Math.Abs(xNew[i] - x[i]));
            }
            if (maxDiff < config.XTolerance)
            {
                result.Status = Status.OK;
                result.Solution = xNew;
                result.Value = lineResult.FNew;
                result.Gradient = gradNew;
                return result;
            }

            // Compute y = grad_new - grad
            for (int i = 1; i <= n; ++i)
            {
                y[i] = gradNew[lower + i - 1] - grad[lower + i - 1];
            }

            // Compute s^T * y (curvature condition)
            double sy = 0.0;
            for (int i = 1; i <= n; ++i)
            {
                sy += s[i] * y[i];
            }

            // Skip update if curvature condition is not satisfied
            if (sy > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
            {
                // BFGS update: H_new = (I - rho*sy^T) H (I - rho*ys^T) + rho*ss^T
                // where rho = 1 / (s^T y)
                double rho = 1.0 / sy;

                // Compute H * y
                math_Vector hy = new(1, n, 0.0);
                for (int i = 1; i <= n; ++i)
                {
                    for (int j = 1; j <= n; ++j)
                    {
                        hy[i] += h[i, j] * y[j];
                    }
                }

                // Compute y^T * H * y
                double yHy = 0.0;
                for (int i = 1; i <= n; ++i)
                {
                    yHy += y[i] * hy[i];
                }

                // Update H using the formula:
                // H_new = H - (Hy*s^T + s*y^T*H)/(s^T*y) + (1 + y^T*H*y/(s^T*y)) * s*s^T/(s^T*y)
                double factor = 1.0 + rho * yHy;

                for (int i = 1; i <= n; ++i)
                {
                    for (int j = 1; j <= n; ++j)
                    {
                        h[i, j] =
                          h[i, j] - rho * (hy[i] * s[j] + s[i] * hy[j]) + factor * rho * s[i] * s[j];
                    }
                }
            }

            // Update for next iteration
            x.Initialized(xNew);
            grad.Initialized(gradNew);
            fx = lineResult.FNew;
        }

        // Maximum iterations reached
        result.Status = Status.MaxIterations;
        result.Solution = x;
        result.Value = fx;
        result.Gradient = grad;
        return result;
    }

    /// <summary>The wrapper class local to BFGSNumerical: adds a central-difference gradient to a value-only function.</summary>
    private sealed class BFGSNumerical_FuncWithGradient<Function> : IVectorFunctionWithGradient
        where Function : IVectorFunction
    {
        private Function _func;
        private readonly double _step;

        public BFGSNumerical_FuncWithGradient(Function f, double step)
        {
            _func = f;
            _step = step;
        }

        public bool Value(math_Vector x, out double f) => _func.Value(x, out f);

        public bool Gradient(math_Vector x, math_Vector grad)
        {
            math_Vector xMod = new(x); // Make mutable copy
            return global::Kerf.Engine.Numerics.Utilities.MathUtils.NumericalGradientAdaptive(_func, xMod, grad, _step);
        }
    }

    /// <summary>BFGS with a numerical (central-difference, adaptive step) gradient for value-only functions.</summary>
    public static VectorResult BFGSNumerical<Function>(Function func,
                                                       math_Vector startingPoint,
                                                       double gradStep = 1.0e-8,
                                                       Config? config = null)
        where Function : IVectorFunction
    {
        config ??= new Config();
        BFGSNumerical_FuncWithGradient<Function> wrapper = new(func, gradStep);
        return BFGS(wrapper, startingPoint, config);
    }

    /// <summary>L-BFGS (limited-memory BFGS) keeping the theMemorySize most recent {s, y} pairs.</summary>
    public static VectorResult LBFGS<Function>(Function func,
                                               math_Vector startingPoint,
                                               int memorySize = 10,
                                               Config? config = null)
        where Function : IVectorFunctionWithGradient
    {
        config ??= new Config();
        VectorResult result = new();

        int lower = startingPoint.Lower();
        int upper = startingPoint.Upper();
        int n = upper - lower + 1;
        int m = memorySize;

        math_Vector x = new(lower, upper);
        x.Initialized(startingPoint);

        if (!func.Value(x, out double fx))
        {
            result.Status = Status.NumericalError;
            return result;
        }

        math_Vector grad = new(lower, upper);
        if (!func.Gradient(x, grad))
        {
            result.Status = Status.NumericalError;
            return result;
        }

        // Storage for {s, y} pairs (circular buffer)
        List<math_Vector> sVec = new();
        List<math_Vector> yVec = new();
        math_Vector rhoVec = new(0, m - 1, 0.0);
        for (int i = 0; i < m; ++i)
        {
            sVec.Add(new math_Vector(1, n));
            yVec.Add(new math_Vector(1, n));
        }
        int head = 0;  // Index of oldest entry
        int count = 0; // Number of stored pairs

        math_Vector dir = new(lower, upper);
        math_Vector xNew = new(lower, upper);
        math_Vector gradNew = new(lower, upper);
        math_Vector q = new(1, n);
        math_Vector alphaVec = new(0, m - 1);

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            // Check gradient convergence
            double gradNorm = 0.0;
            for (int i = lower; i <= upper; ++i)
            {
                gradNorm += global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(grad[i]);
            }
            if (Math.Sqrt(gradNorm) < config.FTolerance)
            {
                result.Status = Status.OK;
                result.Solution = x;
                result.Value = fx;
                result.Gradient = grad;
                return result;
            }

            // L-BFGS two-loop recursion to compute search direction
            // q = gradient
            for (int i = 1; i <= n; ++i)
            {
                q[i] = grad[lower + i - 1];
            }

            // First loop (backward)
            for (int k = count - 1; k >= 0; --k)
            {
                int idx = (head + k) % m;
                alphaVec[idx] = rhoVec[idx] * global::Kerf.Engine.Numerics.Utilities.MathUtils.DotProduct(sVec[idx], q);
                for (int i = 1; i <= n; ++i)
                {
                    q[i] -= alphaVec[idx] * yVec[idx][i];
                }
            }

            // Initial Hessian: H0 = gamma*I where gamma = s^T y / y^T y
            double gamma = 1.0;
            if (count > 0)
            {
                int lastIdx = (head + count - 1) % m;
                double yy = global::Kerf.Engine.Numerics.Utilities.MathUtils.DotProduct(yVec[lastIdx], yVec[lastIdx]);
                if (yy > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                {
                    gamma = 1.0 / (rhoVec[lastIdx] * yy);
                }
            }

            // r = H0 * q = gamma * q
            math_Vector r = new(1, n);
            for (int i = 1; i <= n; ++i)
            {
                r[i] = gamma * q[i];
            }

            // Second loop (forward)
            for (int k = 0; k < count; ++k)
            {
                int idx = (head + k) % m;
                double beta = rhoVec[idx] * global::Kerf.Engine.Numerics.Utilities.MathUtils.DotProduct(yVec[idx], r);
                for (int i = 1; i <= n; ++i)
                {
                    r[i] += (alphaVec[idx] - beta) * sVec[idx][i];
                }
            }

            // Search direction: p = -r
            for (int i = lower; i <= upper; ++i)
            {
                dir[i] = -r[i - lower + 1];
            }

            // Line search
            LineSearchResult lineResult =
              global::Kerf.Engine.Numerics.Utilities.MathUtils.ArmijoBacktrack(func, x, dir, grad, fx, 1.0, 1.0e-4, 0.5, 50);

            if (!lineResult.IsValid)
            {
                // Fall back to steepest descent
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
                    return result;
                }
                // Reset history after steepest descent
                count = 0;
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
            if (maxDiff < config.XTolerance)
            {
                if (!func.Gradient(xNew, gradNew))
                {
                    gradNew.Initialized(grad);
                }
                result.Status = Status.OK;
                result.Solution = xNew;
                result.Value = lineResult.FNew;
                result.Gradient = gradNew;
                return result;
            }

            // Evaluate gradient at new point
            if (!func.Gradient(xNew, gradNew))
            {
                result.Status = Status.NumericalError;
                result.Solution = x;
                result.Value = fx;
                return result;
            }

            // Store new {s, y} pair
            int newIdx = (head + count) % m;
            for (int i = 1; i <= n; ++i)
            {
                sVec[newIdx][i] = xNew[lower + i - 1] - x[lower + i - 1];
                yVec[newIdx][i] = gradNew[lower + i - 1] - grad[lower + i - 1];
            }

            double sy = global::Kerf.Engine.Numerics.Utilities.MathUtils.DotProduct(sVec[newIdx], yVec[newIdx]);
            if (sy > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
            {
                rhoVec[newIdx] = 1.0 / sy;
                if (count < m)
                {
                    ++count;
                }
                else
                {
                    head = (head + 1) % m;
                }
            }

            // Update for next iteration
            x.Initialized(xNew);
            grad.Initialized(gradNew);
            fx = lineResult.FNew;
        }

        result.Status = Status.MaxIterations;
        result.Solution = x;
        result.Value = fx;
        result.Gradient = grad;
        return result;
    }

    /// <summary>
    /// BFGS with box constraints theLowerBounds &lt;= x &lt;= theUpperBounds, by projection: points are clamped and
    /// gradient components pointing out of an active bound are zeroed.
    /// </summary>
    public static VectorResult BFGSBounded<Function>(Function func,
                                                     math_Vector startingPoint,
                                                     math_Vector lowerBounds,
                                                     math_Vector upperBounds,
                                                     Config? config = null)
        where Function : IVectorFunctionWithGradient
    {
        config ??= new Config();
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

                // At lower bound and gradient points downward -> zero gradient
                if (x[i] - lowerBounds[bndIdx] < tol && grad[i] > 0.0)
                {
                    grad[i] = 0.0;
                }
                // At upper bound and gradient points upward -> zero gradient
                if (upperBounds[bndIdx] - x[i] < tol && grad[i] < 0.0)
                {
                    grad[i] = 0.0;
                }
            }
        }

        // Current point and function value
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

        // Initialize inverse Hessian approximation to identity
        Matrix h = new(1, n, 1, n, 0.0);
        for (int i = 1; i <= n; ++i)
        {
            h[i, i] = 1.0;
        }

        // Working vectors
        math_Vector dir = new(lower, upper);
        math_Vector xNew = new(lower, upper);
        math_Vector gradNew = new(lower, upper);
        math_Vector s = new(1, n);
        math_Vector y = new(1, n);

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            // Compute search direction: p = -H * grad
            for (int i = 1; i <= n; ++i)
            {
                double sum = 0.0;
                for (int j = 1; j <= n; ++j)
                {
                    sum += h[i, j] * aGrad[lower + j - 1];
                }
                dir[lower + i - 1] = -sum;
            }

            // Line search with bounds-aware step
            double alphaMax = 1.0;
            for (int i = lower; i <= upper; ++i)
            {
                int bndIdx = lowerBounds.Lower() + (i - lower);
                if (dir[i] < -global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON)
                {
                    // Moving toward lower bound
                    double maxStep = (lowerBounds[bndIdx] - aX[i]) / dir[i];
                    alphaMax = Min(alphaMax, maxStep);
                }
                else if (dir[i] > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON)
                {
                    // Moving toward upper bound
                    double maxStep = (upperBounds[bndIdx] - aX[i]) / dir[i];
                    alphaMax = Min(alphaMax, maxStep);
                }
            }
            alphaMax = Max(alphaMax, global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON);

            LineSearchResult lineResult =
              global::Kerf.Engine.Numerics.Utilities.MathUtils.ArmijoBacktrack(func, aX, dir, aGrad, fx, alphaMax, 1.0e-4, 0.5, 50);

            if (!lineResult.IsValid || lineResult.Alpha < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON)
            {
                // Fall back to projected steepest descent
                for (int i = lower; i <= upper; ++i)
                {
                    dir[i] = -aGrad[i];
                }
                // Recompute alpha max for steepest descent
                alphaMax = 1.0;
                for (int i = lower; i <= upper; ++i)
                {
                    int bndIdx = lowerBounds.Lower() + (i - lower);
                    if (dir[i] < -global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON)
                    {
                        alphaMax = Min(alphaMax, (lowerBounds[bndIdx] - aX[i]) / dir[i]);
                    }
                    else if (dir[i] > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON)
                    {
                        alphaMax = Min(alphaMax, (upperBounds[bndIdx] - aX[i]) / dir[i]);
                    }
                }
                alphaMax = Max(alphaMax, global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_EPSILON);

                lineResult =
                  global::Kerf.Engine.Numerics.Utilities.MathUtils.ArmijoBacktrack(func, aX, dir, aGrad, fx, alphaMax, 1.0e-4, 0.5, 50);

                if (!lineResult.IsValid)
                {
                    result.Status = Status.NotConverged;
                    result.Solution = aX;
                    result.Value = fx;
                    result.Gradient = aGrad;
                    return result;
                }

                // Reset Hessian
                for (int i = 1; i <= n; ++i)
                {
                    for (int j = 1; j <= n; ++j)
                    {
                        h[i, j] = (i == j) ? 1.0 : 0.0;
                    }
                }
            }

            // Compute and clamp new point
            for (int i = lower; i <= upper; ++i)
            {
                xNew[i] = aX[i] + lineResult.Alpha * dir[i];
            }
            ClampToBounds(xNew);

            // Compute s = x_new - x
            for (int i = 1; i <= n; ++i)
            {
                s[i] = xNew[lower + i - 1] - aX[lower + i - 1];
            }

            // Evaluate at new point
            if (!func.Value(xNew, out double fxNew))
            {
                result.Status = Status.NumericalError;
                result.Solution = aX;
                result.Value = fx;
                return result;
            }

            if (!func.Gradient(xNew, gradNew))
            {
                result.Status = Status.NumericalError;
                result.Solution = aX;
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
                result.Value = fxNew;
                result.Gradient = gradNew;
                return result;
            }

            // Check X convergence
            double maxDiff = 0.0;
            for (int i = lower; i <= upper; ++i)
            {
                maxDiff = Max(maxDiff, Math.Abs(xNew[i] - aX[i]));
            }
            if (maxDiff < config.XTolerance)
            {
                result.Status = Status.OK;
                result.Solution = xNew;
                result.Value = fxNew;
                result.Gradient = gradNew;
                return result;
            }

            // Compute y = grad_new - grad
            for (int i = 1; i <= n; ++i)
            {
                y[i] = gradNew[lower + i - 1] - aGrad[lower + i - 1];
            }

            // Curvature condition
            double sy = 0.0;
            for (int i = 1; i <= n; ++i)
            {
                sy += s[i] * y[i];
            }

            if (sy > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
            {
                double rho = 1.0 / sy;

                math_Vector hy = new(1, n, 0.0);
                for (int i = 1; i <= n; ++i)
                {
                    for (int j = 1; j <= n; ++j)
                    {
                        hy[i] += h[i, j] * y[j];
                    }
                }

                double yHy = 0.0;
                for (int i = 1; i <= n; ++i)
                {
                    yHy += y[i] * hy[i];
                }

                double factor = 1.0 + rho * yHy;
                for (int i = 1; i <= n; ++i)
                {
                    for (int j = 1; j <= n; ++j)
                    {
                        h[i, j] =
                          h[i, j] - rho * (hy[i] * s[j] + s[i] * hy[j]) + factor * rho * s[i] * s[j];
                    }
                }
            }

            aX.Initialized(xNew);
            aGrad.Initialized(gradNew);
            fx = fxNew;
        }

        result.Status = Status.MaxIterations;
        result.Solution = aX;
        result.Value = fx;
        result.Gradient = aGrad;
        return result;
    }
}
