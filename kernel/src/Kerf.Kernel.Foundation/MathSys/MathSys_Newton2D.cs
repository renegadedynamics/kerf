// Port of: src/FoundationClasses/TKMath/MathSys/MathSys_Newton2D.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;

using Occt;

namespace Kerf.Engine.Numerics.Systems;

// The C++ function contract `bool operator()(double u, double v, double f[2], double j[2][2]) const` is the
// interface INewton2DFunction, whose Invoke takes the arrays by `ref` as std_array2 values, so a functor writes
// `theF[0] = ...; theJ[0][1] = ...;` as in C++. Solve2D is generic over it: a struct functor gets JIT-specialised
// code with the call inlined, as the C++ template does.

/// <summary>Optimized 2D Newton-Raphson solvers with strict convergence criteria.</summary>
internal static partial class MathSys
{
    /// <summary>A general 2x2 system with its Jacobian, for <see cref="Solve2D"/> (C++'s operator() contract).</summary>
    public interface INewton2DFunction
    {
        /// <summary>Computes F(u, v) into theF and the Jacobian into theJ (theJ[i][j] = dF_i/dx_j); false on failure.</summary>
        bool Invoke(double u, double v, ref Array2<double> f, ref Array2<Array2<double>> j);
    }

    /// <summary>A 2x2 system with a symmetric Jacobian, for <see cref="Solve2DSymmetric"/>.</summary>
    public interface INewton2DSymmetricFunction
    {
        /// <summary>Computes F and the Jacobian's elements J11, J12 (= J21) and J22; false on failure.</summary>
        bool ValueAndJacobian(double u, double v, out double f1, out double f2,
                              out double j11, out double j12, out double j22);

        /// <summary>Computes F only; false on failure.</summary>
        bool Value(double u, double v, out double f1, out double f2);
    }

    internal static partial class detail
    {
        internal const double THE_SINGULAR_DET_TOL = 1.0e-25;
        internal const double THE_CRITICAL_GRAD_SQ = 1.0e-60;
        internal const int THE_LINE_SEARCH_MAX = 8;
        internal const double THE_ARMIJO_C1 = 1.0e-4;

        /// <summary>C++'s std::clamp: theLo if theV &lt; theLo, theHi if theHi &lt; theV, else theV.</summary>
        internal static double std_clamp(double v, double lo, double hi)
        {
            return v < lo ? lo : hi < v ? hi : v;
        }

        /// <summary>Check that NewtonOptions fields are positive and valid.</summary>
        internal static bool IsOptionsValid(NewtonOptions options)
        {
            return options.FTolerance > 0.0 && options.XTolerance > 0.0 && options.MaxIterations > 0
                   && options.MaxStepRatio > 0.0 && options.SoftBoundsExtension >= 0.0;
        }

        /// <summary>Check that the bounds have valid (min &lt;= max) ranges.</summary>
        internal static bool IsBoundsValid2D(in NewtonBoundsN2 bounds)
        {
            if (!bounds.HasBounds)
            {
                return true;
            }
            return bounds.Min[0] <= bounds.Max[0] && bounds.Min[1] <= bounds.Max[1];
        }

        /// <summary>Return the largest domain extent across both dimensions (min 1.0).</summary>
        internal static double MaxDomainSize2D(in NewtonBoundsN2 bounds)
        {
            if (!bounds.HasBounds)
            {
                return 1.0;
            }

            double du = bounds.Max[0] - bounds.Min[0];
            double dv = bounds.Max[1] - bounds.Min[1];
            return Max(1.0, Max(du, dv));
        }

        /// <summary>Clamp solution array to bounds, optionally extending by soft-bounds ratio.</summary>
        internal static void Clamp2D(ref Array2<double> x,
                                     in NewtonBoundsN2 bounds,
                                     bool useSoftBounds,
                                     double softExtRatio)
        {
            if (!bounds.HasBounds)
            {
                return;
            }

            double extU =
              useSoftBounds ? (bounds.Max[0] - bounds.Min[0]) * softExtRatio : 0.0;
            double extV =
              useSoftBounds ? (bounds.Max[1] - bounds.Min[1]) * softExtRatio : 0.0;

            double uMin = bounds.Min[0] - extU;
            double uMax = bounds.Max[0] + extU;
            double vMin = bounds.Min[1] - extV;
            double vMax = bounds.Max[1] + extV;

            x[0] = std_clamp(x[0], uMin, uMax);
            x[1] = std_clamp(x[1], vMin, vMax);
        }

        /// <summary>
        /// Solve a 2x2 symmetric system by eigenvalue decomposition (SVD fallback); more robust than Cramer's rule
        /// for ill-conditioned matrices. theDU and theDV are written only on success.
        /// </summary>
        internal static bool SolveSymmetric2x2SVD(double j11,
                                                  double j12,
                                                  double j22,
                                                  double f1,
                                                  double f2,
                                                  ref double du,
                                                  ref double dv,
                                                  double tol = 1.0e-15)
        {
            double trace = j11 + j22;
            double diff = (j11 - j22) * 0.5;
            double discrim = Math.Sqrt(diff * diff + j12 * j12);

            double lambda1 = trace * 0.5 + discrim;
            double lambda2 = trace * 0.5 - discrim;

            double minLambda = Max(Math.Abs(lambda1), Math.Abs(lambda2)) * tol;
            if (Math.Abs(lambda1) < minLambda && Math.Abs(lambda2) < minLambda)
            {
                return false;
            }

            double v1x = 0.0;
            double v1y = 0.0;
            if (Math.Abs(j12) > Math.Abs(diff))
            {
                double len1 = Math.Sqrt(j12 * j12 + (lambda1 - j11) * (lambda1 - j11));
                if (len1 < tol)
                {
                    return false;
                }

                v1x = j12 / len1;
                v1y = (lambda1 - j11) / len1;
            }
            else
            {
                double len1 = Math.Sqrt((lambda1 - j22) * (lambda1 - j22) + j12 * j12);
                if (len1 < tol)
                {
                    return false;
                }

                v1x = (lambda1 - j22) / len1;
                v1y = j12 / len1;
            }

            double v2x = -v1y;
            double v2y = v1x;

            double b1 = v1x * (-f1) + v1y * (-f2);
            double b2 = v2x * (-f1) + v2y * (-f2);

            double x1 = (Math.Abs(lambda1) > minLambda) ? b1 / lambda1 : 0.0;
            double x2 = (Math.Abs(lambda2) > minLambda) ? b2 / lambda2 : 0.0;

            du = v1x * x1 + v2x * x2;
            dv = v1y * x1 + v2y * x2;
            return true;
        }
    }

    /// <summary>Solve a general 2x2 nonlinear system by Newton iteration.</summary>
    public static NewtonResultN2 Solve2D<Function>(Function func,
                                                   in Array2<double> x0,
                                                   in NewtonBoundsN2 bounds,
                                                   NewtonOptions? options = null)
        where Function : INewton2DFunction
    {
        options ??= new NewtonOptions();
        NewtonResultN2 res = new();
        res.X = x0;

        if (!detail.IsOptionsValid(options) || !detail.IsBoundsValid2D(bounds))
        {
            res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput;
            return res;
        }

        // OCCT: Solve2D clamps to the hard bounds and ignores AllowSoftBounds, which Solve2DSymmetric, Solve3D and
        // Solve4D honour.
        detail.Clamp2D(ref res.X, bounds, false, 0.0);

        double tolSq = options.FTolerance * options.FTolerance;
        double maxStep = options.MaxStepRatio * detail.MaxDomainSize2D(bounds);

        for (int iter = 0; iter < options.MaxIterations; ++iter)
        {
            res.NbIterations = iter + 1;

            Array2<double> f = default;
            Array2<Array2<double>> j = default;
            if (!func.Invoke(res.X[0], res.X[1], ref f, ref j))
            {
                res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return res;
            }

            double fNormSq = f[0] * f[0] + f[1] * f[1];
            res.ResidualNorm = Math.Sqrt(fNormSq);

            if (fNormSq <= tolSq)
            {
                res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                return res;
            }

            double du = 0.0;
            double dv = 0.0;

            double det = j[0][0] * j[1][1] - j[0][1] * j[1][0];
            if (Math.Abs(det) < detail.THE_SINGULAR_DET_TOL)
            {
                double gradU = j[0][0] * f[0] + j[1][0] * f[1];
                double gradV = j[0][1] * f[0] + j[1][1] * f[1];
                double gradSq = gradU * gradU + gradV * gradV;
                if (gradSq < detail.THE_CRITICAL_GRAD_SQ)
                {
                    res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.Singular;
                    return res;
                }

                double alpha = Min(1.0, Math.Sqrt(fNormSq / gradSq) * 0.1);
                du = -alpha * gradU;
                dv = -alpha * gradV;
            }
            else
            {
                double invDet = 1.0 / det;
                du = (-f[0] * j[1][1] + f[1] * j[0][1]) * invDet;
                dv = (-f[1] * j[0][0] + f[0] * j[1][0]) * invDet;
            }

            double stepNormSq = du * du + dv * dv;
            double stepNorm = Math.Sqrt(stepNormSq);
            if (stepNorm > maxStep)
            {
                double scale = maxStep / stepNorm;
                du *= scale;
                dv *= scale;
            }

            Array2<double> newX = Array2<double>.Of(res.X[0] + du, res.X[1] + dv);
            detail.Clamp2D(ref newX, bounds, false, 0.0);

            res.StepNorm = Math.Sqrt((newX[0] - res.X[0]) * (newX[0] - res.X[0])
                                      + (newX[1] - res.X[1]) * (newX[1] - res.X[1]));
            res.X = newX;

            double scaleRef = Max(1.0, Max(Math.Abs(res.X[0]), Math.Abs(res.X[1])));
            if (res.StepNorm <= options.XTolerance * scaleRef)
            {
                Array2<double> checkF = default;
                Array2<Array2<double>> checkJ = default;
                if (!func.Invoke(res.X[0], res.X[1], ref checkF, ref checkJ))
                {
                    res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                    return res;
                }

                res.ResidualNorm = Math.Sqrt(checkF[0] * checkF[0] + checkF[1] * checkF[1]);
                res.Status = (res.ResidualNorm <= options.FTolerance) ? global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK
                                                                           : global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
                return res;
            }
        }

        // Braced: C# doesn't allow the loop's local names again in the enclosing block.
        {
            Array2<double> f = default;
            Array2<Array2<double>> j = default;
            if (!func.Invoke(res.X[0], res.X[1], ref f, ref j))
            {
                res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return res;
            }

            res.ResidualNorm = Math.Sqrt(f[0] * f[0] + f[1] * f[1]);
            res.Status = (res.ResidualNorm <= options.FTolerance) ? global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK
                                                                       : global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
            return res;
        }
    }

    /// <summary>
    /// Solve a 2x2 system with symmetric Jacobian by robust Newton iteration (SVD fallback for a singular
    /// Jacobian, Armijo line search on 0.5*||F||^2 when enabled).
    /// </summary>
    public static NewtonResultN2 Solve2DSymmetric<Function>(Function func,
                                                            in Array2<double> x0,
                                                            in NewtonBoundsN2 bounds,
                                                            NewtonOptions? options = null)
        where Function : INewton2DSymmetricFunction
    {
        options ??= new NewtonOptions();
        NewtonResultN2 res = new();
        res.X = x0;

        if (!detail.IsOptionsValid(options) || !detail.IsBoundsValid2D(bounds))
        {
            res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput;
            return res;
        }

        detail.Clamp2D(ref res.X, bounds, options.AllowSoftBounds, options.SoftBoundsExtension);

        double tolSq = options.FTolerance * options.FTolerance;
        double maxStep = options.MaxStepRatio * detail.MaxDomainSize2D(bounds);

        for (int iter = 0; iter < options.MaxIterations; ++iter)
        {
            res.NbIterations = iter + 1;

            if (!func.ValueAndJacobian(res.X[0], res.X[1], out double f1, out double f2, out double j11, out double j12, out double j22))
            {
                res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return res;
            }

            double fNormSq = f1 * f1 + f2 * f2;
            res.ResidualNorm = Math.Sqrt(fNormSq);

            if (fNormSq <= tolSq)
            {
                res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                return res;
            }

            double du = 0.0;
            double dv = 0.0;

            double det = j11 * j22 - j12 * j12;
            if (Math.Abs(det) < detail.THE_SINGULAR_DET_TOL)
            {
                if (!detail.SolveSymmetric2x2SVD(j11, j12, j22, f1, f2, ref du, ref dv))
                {
                    double gradU = j11 * f1 + j12 * f2;
                    double gradV = j12 * f1 + j22 * f2;
                    double gradSq = gradU * gradU + gradV * gradV;
                    if (gradSq < detail.THE_CRITICAL_GRAD_SQ)
                    {
                        res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.Singular;
                        return res;
                    }

                    double alpha = Min(1.0, Math.Sqrt(fNormSq / gradSq) * 0.1);
                    du = -alpha * gradU;
                    dv = -alpha * gradV;
                }
            }
            else
            {
                double invDet = 1.0 / det;
                du = (-f1 * j22 + f2 * j12) * invDet;
                dv = (-f2 * j11 + f1 * j12) * invDet;
            }

            double stepNormSq = du * du + dv * dv;
            double stepNorm = Math.Sqrt(stepNormSq);
            if (stepNorm > maxStep)
            {
                double scale = maxStep / stepNorm;
                du *= scale;
                dv *= scale;
            }

            // Merit function for line search is phi = 0.5 * ||F||^2.
            // Its gradient is grad(phi) = J^T * F, so directional derivative is grad(phi) . dX.
            double gradPhiU = j11 * f1 + j12 * f2;
            double gradPhiV = j12 * f1 + j22 * f2;
            double dirDeriv = gradPhiU * du + gradPhiV * dv;

            Array2<double> newX = res.X;
            double newResidualNorm = -1.0;

            if (options.EnableLineSearch)
            {
                if (dirDeriv >= 0.0)
                {
                    // Armijo backtracking requires a descent direction for phi; non-negative derivative cannot
                    // provide sufficient decrease.
                    res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NonDescentDirection;
                    return res;
                }

                double phi0 = 0.5 * fNormSq;
                double alpha = 1.0;
                bool isAccepted = false;

                for (int k = 0; k < detail.THE_LINE_SEARCH_MAX; ++k)
                {
                    newX[0] = res.X[0] + alpha * du;
                    newX[1] = res.X[1] + alpha * dv;
                    detail.Clamp2D(ref newX,
                                   bounds,
                                   options.AllowSoftBounds,
                                   options.AllowSoftBounds ? options.SoftBoundsExtension : 0.0);

                    if (!func.Value(newX[0], newX[1], out double tryF1, out double tryF2))
                    {
                        alpha *= 0.5;
                        continue;
                    }

                    double tryFNormSq = tryF1 * tryF1 + tryF2 * tryF2;
                    double tryPhi = 0.5 * tryFNormSq;
                    double armijoBound = phi0 + detail.THE_ARMIJO_C1 * alpha * dirDeriv;
                    if (tryPhi <= armijoBound)
                    {
                        isAccepted = true;
                        newResidualNorm = Math.Sqrt(tryFNormSq);
                        break;
                    }

                    alpha *= 0.5;
                }

                if (!isAccepted)
                {
                    res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NonDescentDirection;
                    return res;
                }
            }
            else
            {
                newX[0] += du;
                newX[1] += dv;
                detail.Clamp2D(ref newX,
                               bounds,
                               options.AllowSoftBounds,
                               options.AllowSoftBounds ? options.SoftBoundsExtension : 0.0);
            }

            res.StepNorm = Math.Sqrt((newX[0] - res.X[0]) * (newX[0] - res.X[0])
                                      + (newX[1] - res.X[1]) * (newX[1] - res.X[1]));
            res.X = newX;

            if (newResidualNorm >= 0.0)
            {
                res.ResidualNorm = newResidualNorm;
            }

            double scaleRef = Max(1.0, Max(Math.Abs(res.X[0]), Math.Abs(res.X[1])));
            if (res.StepNorm <= options.XTolerance * scaleRef)
            {
                if (!func.Value(res.X[0], res.X[1], out double checkF1, out double checkF2))
                {
                    res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                    return res;
                }

                res.ResidualNorm = Math.Sqrt(checkF1 * checkF1 + checkF2 * checkF2);
                res.Status = (res.ResidualNorm <= options.FTolerance) ? global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK
                                                                           : global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
                return res;
            }
        }

        // Braced: C# doesn't allow the loop's local names again in the enclosing block.
        {
            if (!func.Value(res.X[0], res.X[1], out double f1, out double f2))
            {
                res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return res;
            }

            res.ResidualNorm = Math.Sqrt(f1 * f1 + f2 * f2);
            res.Status = (res.ResidualNorm <= options.FTolerance) ? global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK
                                                                       : global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
            return res;
        }
    }
}
