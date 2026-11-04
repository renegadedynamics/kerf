// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_Powell.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Optimisation;

internal static partial class MathOpt
{
    /// <summary>
    /// Powell's conjugate direction method for N-dimensional minimization (gradient-free), starting from the
    /// coordinate axes.
    /// </summary>
    public static VectorResult Powell<Function>(Function func, math_Vector startingPoint, Config? config = null)
        where Function : IVectorFunction
    {
        config ??= new Config();
        VectorResult result = new();

        int lower = startingPoint.Lower();
        int upper = startingPoint.Upper();
        int n = upper - lower + 1;

        // Current point
        math_Vector x = new(lower, upper);
        x.Initialized(startingPoint);

        // Function value at current point
        if (!func.Value(x, out double fx))
        {
            result.Status = Status.NumericalError;
            return result;
        }

        // Initialize direction set to coordinate axes
        // Directions stored as rows of matrix
        Matrix directions = new(1, n, 1, n, 0.0);
        for (int i = 1; i <= n; ++i)
        {
            directions[i, i] = 1.0;
        }

        // Working vectors
        math_Vector dir = new(lower, upper);
        math_Vector xOld = new(lower, upper);
        math_Vector pt = new(lower, upper);
        math_Vector ptt = new(lower, upper);
        math_Vector xit = new(lower, upper);
        _ = pt; // unused, as in C++

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            double fp = fx;
            xOld.Initialized(x);

            // Track largest decrease and its direction index
            double del = 0.0;
            int iBig = 0;

            // Minimize along each direction
            for (int i = 1; i <= n; ++i)
            {
                // Extract direction i
                for (int j = lower; j <= upper; ++j)
                {
                    dir[j] = directions[i, j - lower + 1];
                }

                double fpPrev = fx;

                // Line minimization along direction
                LineSearchResult lineResult =
                  global::Kerf.Engine.Numerics.Utilities.MathUtils.ExactLineSearch(func, x, dir, 10.0, config.XTolerance);

                if (lineResult.IsValid)
                {
                    // Update position
                    for (int j = lower; j <= upper; ++j)
                    {
                        x[j] += lineResult.Alpha * dir[j];
                    }
                    fx = lineResult.FNew;

                    // Track direction with largest decrease
                    double decrease = fpPrev - fx;
                    if (decrease > del)
                    {
                        del = decrease;
                        iBig = i;
                    }
                }
            }

            // Check convergence
            if (2.0 * Math.Abs(fp - fx)
                <= config.FTolerance * (Math.Abs(fp) + Math.Abs(fx) + global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL))
            {
                result.Status = Status.OK;
                result.Solution = x;
                result.Value = fx;
                return result;
            }

            // Construct extrapolated point and new direction
            for (int j = lower; j <= upper; ++j)
            {
                ptt[j] = 2.0 * x[j] - xOld[j];
                xit[j] = x[j] - xOld[j];
            }

            // Evaluate at extrapolated point
            if (!func.Value(ptt, out double fptt))
            {
                // If evaluation fails, continue with current directions
                continue;
            }

            // Check if new direction should be added
            if (fptt < fp)
            {
                double t = 2.0 * (fp - 2.0 * fx + fptt) * global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(fp - fx - del)
                            - del * global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(fp - fptt);

                if (t < 0.0)
                {
                    // Minimize along new direction
                    LineSearchResult lineResult =
                      global::Kerf.Engine.Numerics.Utilities.MathUtils.ExactLineSearch(func, x, xit, 10.0, config.XTolerance);

                    if (lineResult.IsValid)
                    {
                        // Update position
                        for (int j = lower; j <= upper; ++j)
                        {
                            x[j] += lineResult.Alpha * xit[j];
                        }
                        fx = lineResult.FNew;

                        // Replace direction with largest decrease
                        if (iBig > 0)
                        {
                            for (int j = 1; j <= n; ++j)
                            {
                                directions[iBig, j] = directions[n, j];
                                directions[n, j] = xit[lower + j - 1];
                            }
                        }
                    }
                }
            }

            // Check X convergence
            double maxDiff = 0.0;
            for (int j = lower; j <= upper; ++j)
            {
                maxDiff = Max(maxDiff, Math.Abs(x[j] - xOld[j]));
            }
            if (maxDiff < config.XTolerance)
            {
                result.Status = Status.OK;
                result.Solution = x;
                result.Value = fx;
                return result;
            }
        }

        // Maximum iterations reached
        result.Status = Status.MaxIterations;
        result.Solution = x;
        result.Value = fx;
        return result;
    }

    /// <summary>Powell's method with a custom initial direction set (N x N, directions as rows).</summary>
    public static VectorResult PowellWithDirections<Function>(Function func,
                                                              math_Vector startingPoint,
                                                              Matrix initialDirections,
                                                              Config? config = null)
        where Function : IVectorFunction
    {
        config ??= new Config();
        VectorResult result = new();

        int lower = startingPoint.Lower();
        int upper = startingPoint.Upper();
        int n = upper - lower + 1;

        // Validate dimensions
        if (initialDirections.RowNumber() != n || initialDirections.ColNumber() != n)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        math_Vector x = new(lower, upper);
        x.Initialized(startingPoint);

        if (!func.Value(x, out double fx))
        {
            result.Status = Status.NumericalError;
            return result;
        }

        // Copy initial directions
        Matrix directions = new(1, n, 1, n);
        directions.Initialized(initialDirections);

        math_Vector dir = new(lower, upper);
        math_Vector xOld = new(lower, upper);
        math_Vector ptt = new(lower, upper);
        math_Vector xit = new(lower, upper);

        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            double fp = fx;
            xOld.Initialized(x);

            double del = 0.0;
            int iBig = 0;

            for (int i = 1; i <= n; ++i)
            {
                for (int j = lower; j <= upper; ++j)
                {
                    dir[j] = directions[i, j - lower + 1];
                }

                double fpPrev = fx;

                LineSearchResult lineResult =
                  global::Kerf.Engine.Numerics.Utilities.MathUtils.ExactLineSearch(func, x, dir, 10.0, config.XTolerance);

                if (lineResult.IsValid)
                {
                    for (int j = lower; j <= upper; ++j)
                    {
                        x[j] += lineResult.Alpha * dir[j];
                    }
                    fx = lineResult.FNew;

                    double decrease = fpPrev - fx;
                    if (decrease > del)
                    {
                        del = decrease;
                        iBig = i;
                    }
                }
            }

            // Check convergence
            if (2.0 * Math.Abs(fp - fx)
                <= config.FTolerance * (Math.Abs(fp) + Math.Abs(fx) + global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL))
            {
                result.Status = Status.OK;
                result.Solution = x;
                result.Value = fx;
                return result;
            }

            // Construct extrapolated point
            for (int j = lower; j <= upper; ++j)
            {
                ptt[j] = 2.0 * x[j] - xOld[j];
                xit[j] = x[j] - xOld[j];
            }

            if (!func.Value(ptt, out double fptt))
            {
                continue;
            }

            if (fptt < fp)
            {
                double t = 2.0 * (fp - 2.0 * fx + fptt) * global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(fp - fx - del)
                            - del * global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(fp - fptt);

                if (t < 0.0)
                {
                    LineSearchResult lineResult =
                      global::Kerf.Engine.Numerics.Utilities.MathUtils.ExactLineSearch(func, x, xit, 10.0, config.XTolerance);

                    if (lineResult.IsValid)
                    {
                        for (int j = lower; j <= upper; ++j)
                        {
                            x[j] += lineResult.Alpha * xit[j];
                        }
                        fx = lineResult.FNew;

                        if (iBig > 0)
                        {
                            for (int j = 1; j <= n; ++j)
                            {
                                directions[iBig, j] = directions[n, j];
                                directions[n, j] = xit[lower + j - 1];
                            }
                        }
                    }
                }
            }

            double maxDiff = 0.0;
            for (int j = lower; j <= upper; ++j)
            {
                maxDiff = Max(maxDiff, Math.Abs(x[j] - xOld[j]));
            }
            if (maxDiff < config.XTolerance)
            {
                result.Status = Status.OK;
                result.Solution = x;
                result.Value = fx;
                return result;
            }
        }

        result.Status = Status.MaxIterations;
        result.Solution = x;
        result.Value = fx;
        return result;
    }
}
