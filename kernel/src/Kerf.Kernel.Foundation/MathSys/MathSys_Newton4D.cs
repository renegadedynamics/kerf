// Port of: src/FoundationClasses/TKMath/MathSys/MathSys_Newton4D.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;
using Kerf.Geometry;
using Occt;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Numerics.Systems;

// The function contract `bool operator()(double x1, ..., double x4, double f[4], double j[4][4]) const` is
// INewton4DFunction (see MathSys_Newton2D.cs); SolveSurfaceSurfaceExtrema4D's lambda is a private struct functor
// over two ISurfaceD2Evaluator (MathSys_Newton3D.cs).

/// <summary>Optimized 4D Newton-Raphson solver for systems of 4 equations in 4 unknowns.</summary>
internal static partial class MathSys
{
    /// <summary>A 4x4 system with its Jacobian, for <see cref="Solve4D"/> (C++'s operator() contract).</summary>
    public interface INewton4DFunction
    {
        /// <summary>Computes F into theF and the Jacobian into theJ (theJ[i][j] = dF_i/dx_j); false on failure.</summary>
        bool Invoke(double x1, double x2, double x3, double x4,
                    ref Array4<double> f, ref Array4<Array4<double>> j);
    }

    internal static partial class detail
    {
        /// <summary>A row of Solve4x4's augmented matrix: C++'s <c>double[5]</c>, held inline.</summary>
        [InlineArray(5)]
        internal struct Array5
        {
            private double _element0;
        }

        /// <summary>Check that the bounds have valid (min &lt;= max) ranges.</summary>
        internal static bool IsBoundsValid4D(in NewtonBoundsN4 bounds)
        {
            if (!bounds.HasBounds)
            {
                return true;
            }

            return bounds.Min[0] <= bounds.Max[0] && bounds.Min[1] <= bounds.Max[1]
                   && bounds.Min[2] <= bounds.Max[2] && bounds.Min[3] <= bounds.Max[3];
        }

        /// <summary>Check that NewtonOptions fields are positive and valid.</summary>
        internal static bool IsOptionsValid4D(NewtonOptions options)
        {
            return options.FTolerance > 0.0 && options.XTolerance > 0.0 && options.MaxIterations > 0
                   && options.MaxStepRatio > 0.0 && options.SoftBoundsExtension >= 0.0;
        }

        /// <summary>Return the largest domain extent across all 4 dimensions (min 1.0).</summary>
        internal static double MaxDomainSize4D(in NewtonBoundsN4 bounds)
        {
            if (!bounds.HasBounds)
            {
                return 1.0;
            }

            double d0 = bounds.Max[0] - bounds.Min[0];
            double d1 = bounds.Max[1] - bounds.Min[1];
            double d2 = bounds.Max[2] - bounds.Min[2];
            double d3 = bounds.Max[3] - bounds.Min[3];
            return Max(1.0, Max(d0, Max(d1, Max(d2, d3))));
        }

        /// <summary>Clamp solution array to bounds, optionally extending by soft-bounds ratio.</summary>
        internal static void Clamp4D(ref Array4<double> x,
                                     in NewtonBoundsN4 bounds,
                                     bool useSoftBounds,
                                     double softExtRatio)
        {
            if (!bounds.HasBounds)
            {
                return;
            }

            for (int i = 0; i < 4; ++i)
            {
                double ext =
                  useSoftBounds ? (bounds.Max[i] - bounds.Min[i]) * softExtRatio : 0.0;
                x[i] = std_clamp(x[i], bounds.Min[i] - ext, bounds.Max[i] + ext);
            }
        }

        /// <summary>
        /// Solve the 4x4 linear system J*x = -F by Gaussian elimination with partial pivoting; theDelta is written
        /// only when the system is non-singular (true).
        /// </summary>
        internal static bool Solve4x4(in Array4<Array4<double>> theJ, in Array4<double> f, ref Array4<double> delta)
        {
            // Augmented matrix [J | -F]
            Array4<Array5> a = default;
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    a[i][j] = theJ[i][j];
                }
                a[i][4] = -f[i];
            }

            // Forward elimination with partial pivoting
            for (int k = 0; k < 4; ++k)
            {
                // Find pivot
                int maxRow = k;
                double maxVal = Math.Abs(a[k][k]);
                for (int i = k + 1; i < 4; ++i)
                {
                    double val = Math.Abs(a[i][k]);
                    if (val > maxVal)
                    {
                        maxVal = val;
                        maxRow = i;
                    }
                }

                // Check for singularity
                if (maxVal < 1.0e-30)
                {
                    return false;
                }

                // Swap rows if needed
                if (maxRow != k)
                {
                    for (int j = k; j <= 4; ++j)
                    {
                        (a[k][j], a[maxRow][j]) = (a[maxRow][j], a[k][j]);
                    }
                }

                // Eliminate column
                double invPivot = 1.0 / a[k][k];
                for (int i = k + 1; i < 4; ++i)
                {
                    double factor = a[i][k] * invPivot;
                    for (int j = k + 1; j <= 4; ++j)
                    {
                        a[i][j] -= factor * a[k][j];
                    }
                    a[i][k] = 0.0;
                }
            }

            // Back substitution
            for (int i = 3; i >= 0; --i)
            {
                double sum = a[i][4];
                for (int j = i + 1; j < 4; ++j)
                {
                    sum -= a[i][j] * delta[j];
                }
                delta[i] = sum / a[i][i];
            }

            return true;
        }
    }

    /// <summary>
    /// Solve a 4x4 nonlinear system [F1, F2, F3, F4] = 0 by Newton iteration with bounds, using Gaussian
    /// elimination for the linear system at each step and steepest descent when the Jacobian is singular.
    /// </summary>
    public static NewtonResultN4 Solve4D<Function>(Function func,
                                                   in Array4<double> x0,
                                                   in NewtonBoundsN4 bounds,
                                                   NewtonOptions? options = null)
        where Function : INewton4DFunction
    {
        options ??= new NewtonOptions();
        NewtonResultN4 res = new();
        res.X = x0;

        if (!detail.IsOptionsValid4D(options) || !detail.IsBoundsValid4D(bounds))
        {
            res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput;
            return res;
        }

        detail.Clamp4D(ref res.X, bounds, options.AllowSoftBounds, options.SoftBoundsExtension);

        double tolSq = options.FTolerance * options.FTolerance;
        double maxStep = options.MaxStepRatio * detail.MaxDomainSize4D(bounds);

        for (int iter = 0; iter < options.MaxIterations; ++iter)
        {
            res.NbIterations = iter + 1;

            Array4<double> f = default;
            Array4<Array4<double>> j = default;
            if (!func.Invoke(res.X[0], res.X[1], res.X[2], res.X[3], ref f, ref j))
            {
                res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return res;
            }

            // Check convergence using squared norm (avoid sqrt)
            double fNormSq = f[0] * f[0] + f[1] * f[1] + f[2] * f[2] + f[3] * f[3];
            res.ResidualNorm = Math.Sqrt(fNormSq);
            if (fNormSq <= tolSq)
            {
                res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                return res;
            }

            // Solve 4x4 linear system: J * delta = -F
            Array4<double> delta = default;
            if (!detail.Solve4x4(j, f, ref delta))
            {
                // Singular Jacobian - try steepest descent direction: -J^T * F
                Array4<double> grad = Array4<double>.Of(0.0, 0.0, 0.0, 0.0);
                for (int c = 0; c < 4; ++c)
                {
                    for (int r = 0; r < 4; ++r)
                    {
                        grad[c] += j[r][c] * f[r];
                    }
                }

                double gradSq =
                  grad[0] * grad[0] + grad[1] * grad[1] + grad[2] * grad[2] + grad[3] * grad[3];
                if (gradSq < 1.0e-60)
                {
                    res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.Singular;
                    return res;
                }

                double alpha = Min(1.0, Math.Sqrt(fNormSq / gradSq) * 0.1);
                for (int i = 0; i < 4; ++i)
                {
                    delta[i] = -alpha * grad[i];
                }
            }

            // Limit step size to prevent wild oscillations
            double stepNormSq =
              delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2] + delta[3] * delta[3];
            double stepNorm = Math.Sqrt(stepNormSq);
            if (stepNorm > maxStep)
            {
                double scale = maxStep / stepNorm;
                for (int i = 0; i < 4; ++i)
                {
                    delta[i] *= scale;
                }
            }

            // Update and clamp to bounds
            Array4<double> newX = Array4<double>.Of(res.X[0] + delta[0],
                                                             res.X[1] + delta[1],
                                                             res.X[2] + delta[2],
                                                             res.X[3] + delta[3]);
            detail.Clamp4D(ref newX, bounds, options.AllowSoftBounds, options.SoftBoundsExtension);

            res.StepNorm = Math.Sqrt((newX[0] - res.X[0]) * (newX[0] - res.X[0])
                                      + (newX[1] - res.X[1]) * (newX[1] - res.X[1])
                                      + (newX[2] - res.X[2]) * (newX[2] - res.X[2])
                                      + (newX[3] - res.X[3]) * (newX[3] - res.X[3]));
            res.X = newX;

            double scaleRef = Max(
              1.0,
              Max(Math.Abs(res.X[0]),
                  Max(Math.Abs(res.X[1]), Max(Math.Abs(res.X[2]), Math.Abs(res.X[3])))));
            if (res.StepNorm <= options.XTolerance * scaleRef)
            {
                Array4<double> checkF = default;
                Array4<Array4<double>> checkJ = default;
                if (!func.Invoke(res.X[0], res.X[1], res.X[2], res.X[3], ref checkF, ref checkJ))
                {
                    res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                    return res;
                }

                res.ResidualNorm = Math.Sqrt(checkF[0] * checkF[0] + checkF[1] * checkF[1]
                                              + checkF[2] * checkF[2] + checkF[3] * checkF[3]);
                res.Status = (res.ResidualNorm <= options.FTolerance) ? global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK
                                                                           : global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
                return res;
            }
        }

        // Final convergence check after max iterations
        // (braced: C# doesn't allow the loop's local names again in the enclosing block)
        {
            Array4<double> f = default;
            Array4<Array4<double>> j = default;
            if (!func.Invoke(res.X[0], res.X[1], res.X[2], res.X[3], ref f, ref j))
            {
                res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return res;
            }

            res.ResidualNorm = Math.Sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2] + f[3] * f[3]);
            res.Status = (res.ResidualNorm <= options.FTolerance) ? global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK
                                                                       : global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
            return res;
        }
    }

    /// <summary>The lambda of SolveSurfaceSurfaceExtrema4D: the gradient of ||S1 - S2||^2 / 2 and its Hessian.</summary>
    private struct SurfaceSurfaceExtrema4DFunction<TSurfaceEvaluator1, TSurfaceEvaluator2> : INewton4DFunction
        where TSurfaceEvaluator1 : ISurfaceD2Evaluator
        where TSurfaceEvaluator2 : ISurfaceD2Evaluator
    {
        internal TSurfaceEvaluator1 _surf1;
        internal TSurfaceEvaluator2 _surf2;

        public bool Invoke(double u1, double v1, double u2, double v2,
                           ref Array4<double> f, ref Array4<Array4<double>> j)
        {
            _surf1.D2(u1, v1, out Point3D pt1, out Vector3D d1U1, out Vector3D d1V1, out Vector3D d2UU1, out Vector3D d2VV1, out Vector3D d2UV1);
            _surf2.D2(u2, v2, out Point3D pt2, out Vector3D d1U2, out Vector3D d1V2, out Vector3D d2UU2, out Vector3D d2VV2, out Vector3D d2UV2);

            // D = P1 - P2
            Vector3D d = new(pt2, pt1);

            // Function values: gradient of ||S1 - S2||^2
            f[0] = d.Dot(d1U1);
            f[1] = d.Dot(d1V1);
            f[2] = -d.Dot(d1U2);
            f[3] = -d.Dot(d1V2);

            // Jacobian: Hessian of ||S1 - S2||^2
            // Block [0:2, 0:2]: derivatives w.r.t. (u1, v1)
            j[0][0] = d1U1.Dot(d1U1) + d.Dot(d2UU1);
            j[0][1] = d1V1.Dot(d1U1) + d.Dot(d2UV1);
            j[1][0] = j[0][1]; // Symmetric
            j[1][1] = d1V1.Dot(d1V1) + d.Dot(d2VV1);

            // Block [0:2, 2:4]: cross derivatives
            j[0][2] = -d1U2.Dot(d1U1);
            j[0][3] = -d1V2.Dot(d1U1);
            j[1][2] = -d1U2.Dot(d1V1);
            j[1][3] = -d1V2.Dot(d1V1);

            // Block [2:4, 0:2]: cross derivatives (symmetric to above)
            j[2][0] = j[0][2];
            j[2][1] = j[1][2];
            j[3][0] = j[0][3];
            j[3][1] = j[1][3];

            // Block [2:4, 2:4]: derivatives w.r.t. (u2, v2)
            j[2][2] = d1U2.Dot(d1U2) - d.Dot(d2UU2);
            j[2][3] = d1V2.Dot(d1U2) - d.Dot(d2UV2);
            j[3][2] = j[2][3]; // Symmetric
            j[3][3] = d1V2.Dot(d1V2) - d.Dot(d2VV2);
            return true;
        }
    }

    /// <summary>
    /// 4D Newton solver for extrema between two surfaces S1(u1, v1) and S2(u2, v2). The result holds u1, v1, u2,
    /// v2 in X[0..3].
    /// </summary>
    public static NewtonResultN4 SolveSurfaceSurfaceExtrema4D<TSurfaceEvaluator1, TSurfaceEvaluator2>(TSurfaceEvaluator1 surf1,
                                                                                                    TSurfaceEvaluator2 surf2,
                                                                                                    in Array4<double> x0,
                                                                                                    in NewtonBoundsN4 bounds,
                                                                                                    NewtonOptions? options = null)
        where TSurfaceEvaluator1 : ISurfaceD2Evaluator
        where TSurfaceEvaluator2 : ISurfaceD2Evaluator
    {
        SurfaceSurfaceExtrema4DFunction<TSurfaceEvaluator1, TSurfaceEvaluator2> func = new() { _surf1 = surf1, _surf2 = surf2 };

        return Solve4D(func, x0, bounds, options);
    }
}
