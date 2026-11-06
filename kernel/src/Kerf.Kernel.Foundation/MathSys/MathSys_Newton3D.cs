// Port of: src/FoundationClasses/TKMath/MathSys/MathSys_Newton3D.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;

using Kerf.Geometry;

namespace Kerf.Engine.Numerics.Systems;

// The function contract `bool operator()(double x1, double x2, double x3, double f[3], double j[3][3]) const` is
// INewton3DFunction (see MathSys_Newton2D.cs). The curve and surface evaluators of SolveCurveSurfaceExtrema3D,
// duck-typed in C++, are ICurveD2Evaluator and ISurfaceD2Evaluator; its lambda is a private struct functor.

/// <summary>Optimized 3D Newton-Raphson solver for systems of 3 equations in 3 unknowns.</summary>
internal static partial class MathSys
{
    /// <summary>A 3x3 system with its Jacobian, for <see cref="Solve3D"/> (C++'s operator() contract).</summary>
    public interface INewton3DFunction
    {
        /// <summary>Computes F into theF and the Jacobian into theJ (theJ[i][j] = dF_i/dx_j); false on failure.</summary>
        bool Invoke(double x1, double x2, double x3,
                    ref Array3<double> f, ref Array3<Array3<double>> j);
    }

    /// <summary>A curve evaluator with second derivatives: C++'s "type providing D2(t, P, D1, D2)".</summary>
    public interface ICurveD2Evaluator
    {
        void D2(double t, out Point3D p, out Vector3D d1, out Vector3D d2);
    }

    /// <summary>A surface evaluator with second derivatives: C++'s "type providing D2(u, v, P, D1U, D1V, D2UU, D2VV, D2UV)".</summary>
    public interface ISurfaceD2Evaluator
    {
        void D2(double u, double v, out Point3D p, out Vector3D d1U, out Vector3D d1V,
                out Vector3D d2UU, out Vector3D d2VV, out Vector3D d2UV);
    }

    internal static partial class detail
    {
        /// <summary>Check that the bounds have valid (min &lt;= max) ranges.</summary>
        internal static bool IsBoundsValid3D(in NewtonBoundsN3 bounds)
        {
            if (!bounds.HasBounds)
            {
                return true;
            }

            return bounds.Min[0] <= bounds.Max[0] && bounds.Min[1] <= bounds.Max[1]
                   && bounds.Min[2] <= bounds.Max[2];
        }

        /// <summary>Check that NewtonOptions fields are positive and valid.</summary>
        internal static bool IsOptionsValid3D(NewtonOptions options)
        {
            return options.FTolerance > 0.0 && options.XTolerance > 0.0 && options.MaxIterations > 0
                   && options.MaxStepRatio > 0.0 && options.SoftBoundsExtension >= 0.0;
        }

        /// <summary>Return the largest domain extent across all 3 dimensions (min 1.0).</summary>
        internal static double MaxDomainSize3D(in NewtonBoundsN3 bounds)
        {
            if (!bounds.HasBounds)
            {
                return 1.0;
            }

            double dx = bounds.Max[0] - bounds.Min[0];
            double dy = bounds.Max[1] - bounds.Min[1];
            double dz = bounds.Max[2] - bounds.Min[2];
            return Max(1.0, Max(dx, Max(dy, dz)));
        }

        /// <summary>Clamp solution array to bounds, optionally extending by soft-bounds ratio.</summary>
        internal static void Clamp3D(ref Array3<double> x,
                                     in NewtonBoundsN3 bounds,
                                     bool useSoftBounds,
                                     double softExtRatio)
        {
            if (!bounds.HasBounds)
            {
                return;
            }

            double extX =
              useSoftBounds ? (bounds.Max[0] - bounds.Min[0]) * softExtRatio : 0.0;
            double extY =
              useSoftBounds ? (bounds.Max[1] - bounds.Min[1]) * softExtRatio : 0.0;
            double extZ =
              useSoftBounds ? (bounds.Max[2] - bounds.Min[2]) * softExtRatio : 0.0;

            x[0] = std_clamp(x[0], bounds.Min[0] - extX, bounds.Max[0] + extX);
            x[1] = std_clamp(x[1], bounds.Min[1] - extY, bounds.Max[1] + extY);
            x[2] = std_clamp(x[2], bounds.Min[2] - extZ, bounds.Max[2] + extZ);
        }

        /// <summary>
        /// Solve the 3x3 linear system J*x = -F by Cramer's rule with cofactor expansion; theDelta is written only
        /// when the system is non-singular (true).
        /// </summary>
        internal static bool Solve3x3(in Array3<Array3<double>> j, in Array3<double> f, ref Array3<double> delta)
        {
            // Compute cofactors for first row (used for determinant and inverse)
            double cof00 = j[1][1] * j[2][2] - j[1][2] * j[2][1];
            double cof01 = j[1][2] * j[2][0] - j[1][0] * j[2][2];
            double cof02 = j[1][0] * j[2][1] - j[1][1] * j[2][0];

            // Determinant by first row expansion
            double det = j[0][0] * cof00 + j[0][1] * cof01 + j[0][2] * cof02;
            // Check for singularity
            if (Math.Abs(det) < 1.0e-30)
            {
                return false;
            }

            double invDet = 1.0 / det;

            // Remaining cofactors
            double cof10 = j[0][2] * j[2][1] - j[0][1] * j[2][2];
            double cof11 = j[0][0] * j[2][2] - j[0][2] * j[2][0];
            double cof12 = j[0][1] * j[2][0] - j[0][0] * j[2][1];

            double cof20 = j[0][1] * j[1][2] - j[0][2] * j[1][1];
            double cof21 = j[0][2] * j[1][0] - j[0][0] * j[1][2];
            double cof22 = j[0][0] * j[1][1] - j[0][1] * j[1][0];

            // Solve: delta = -J^(-1) * F = -adjugate(J)^T * F / det
            delta[0] = -(cof00 * f[0] + cof10 * f[1] + cof20 * f[2]) * invDet;
            delta[1] = -(cof01 * f[0] + cof11 * f[1] + cof21 * f[2]) * invDet;
            delta[2] = -(cof02 * f[0] + cof12 * f[1] + cof22 * f[2]) * invDet;
            return true;
        }
    }

    /// <summary>
    /// Solve a 3x3 nonlinear system [F1, F2, F3] = 0 by Newton iteration with bounds, using Cramer's rule for the
    /// linear system at each step and steepest descent when the Jacobian is singular.
    /// </summary>
    public static NewtonResultN3 Solve3D<Function>(Function func,
                                                   in Array3<double> x0,
                                                   in NewtonBoundsN3 bounds,
                                                   NewtonOptions? options = null)
        where Function : INewton3DFunction
    {
        options ??= new NewtonOptions();
        NewtonResultN3 res = new();
        res.X = x0;

        if (!detail.IsOptionsValid3D(options) || !detail.IsBoundsValid3D(bounds))
        {
            res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput;
            return res;
        }

        detail.Clamp3D(ref res.X, bounds, options.AllowSoftBounds, options.SoftBoundsExtension);

        double tolSq = options.FTolerance * options.FTolerance;
        double maxStep = options.MaxStepRatio * detail.MaxDomainSize3D(bounds);

        for (int iter = 0; iter < options.MaxIterations; ++iter)
        {
            res.NbIterations = iter + 1;

            Array3<double> f = default;
            Array3<Array3<double>> j = default;
            if (!func.Invoke(res.X[0], res.X[1], res.X[2], ref f, ref j))
            {
                res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return res;
            }

            // Check convergence using squared norm (avoid sqrt)
            double fNormSq = f[0] * f[0] + f[1] * f[1] + f[2] * f[2];
            res.ResidualNorm = Math.Sqrt(fNormSq);
            if (fNormSq <= tolSq)
            {
                res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
                return res;
            }

            // Solve 3x3 linear system: J * delta = -F
            Array3<double> delta = default;
            if (!detail.Solve3x3(j, f, ref delta))
            {
                // Singular Jacobian - try steepest descent direction: -J^T * F
                double gradX = j[0][0] * f[0] + j[1][0] * f[1] + j[2][0] * f[2];
                double gradY = j[0][1] * f[0] + j[1][1] * f[1] + j[2][1] * f[2];
                double gradZ = j[0][2] * f[0] + j[1][2] * f[1] + j[2][2] * f[2];
                double gradSq = gradX * gradX + gradY * gradY + gradZ * gradZ;
                if (gradSq < 1.0e-60)
                {
                    res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.Singular;
                    return res;
                }

                double alpha = Min(1.0, Math.Sqrt(fNormSq / gradSq) * 0.1);
                delta[0] = -alpha * gradX;
                delta[1] = -alpha * gradY;
                delta[2] = -alpha * gradZ;
            }

            // Limit step size to prevent wild oscillations
            double stepNormSq =
              delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2];
            double stepNorm = Math.Sqrt(stepNormSq);
            if (stepNorm > maxStep)
            {
                double scale = maxStep / stepNorm;
                delta[0] *= scale;
                delta[1] *= scale;
                delta[2] *= scale;
            }

            // Update and clamp to bounds
            Array3<double> newX = Array3<double>.Of(res.X[0] + delta[0],
                                                             res.X[1] + delta[1],
                                                             res.X[2] + delta[2]);
            detail.Clamp3D(ref newX, bounds, options.AllowSoftBounds, options.SoftBoundsExtension);

            res.StepNorm = Math.Sqrt((newX[0] - res.X[0]) * (newX[0] - res.X[0])
                                      + (newX[1] - res.X[1]) * (newX[1] - res.X[1])
                                      + (newX[2] - res.X[2]) * (newX[2] - res.X[2]));
            res.X = newX;

            double scaleRef =
              Max(1.0, Max(Math.Abs(res.X[0]), Max(Math.Abs(res.X[1]), Math.Abs(res.X[2]))));
            if (res.StepNorm <= options.XTolerance * scaleRef)
            {
                Array3<double> checkF = default;
                Array3<Array3<double>> checkJ = default;
                if (!func.Invoke(res.X[0], res.X[1], res.X[2], ref checkF, ref checkJ))
                {
                    res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                    return res;
                }

                res.ResidualNorm =
                  Math.Sqrt(checkF[0] * checkF[0] + checkF[1] * checkF[1] + checkF[2] * checkF[2]);
                res.Status = (res.ResidualNorm <= options.FTolerance) ? global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK
                                                                           : global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
                return res;
            }
        }

        // Final convergence check after max iterations
        // (braced: C# doesn't allow the loop's local names again in the enclosing block)
        {
            Array3<double> f = default;
            Array3<Array3<double>> j = default;
            if (!func.Invoke(res.X[0], res.X[1], res.X[2], ref f, ref j))
            {
                res.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return res;
            }

            res.ResidualNorm = Math.Sqrt(f[0] * f[0] + f[1] * f[1] + f[2] * f[2]);
            res.Status = (res.ResidualNorm <= options.FTolerance) ? global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK
                                                                       : global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations;
            return res;
        }
    }

    /// <summary>The lambda of SolveCurveSurfaceExtrema3D: the gradient of ||C(t) - S(u, v)||^2 / 2 and its Hessian.</summary>
    private struct CurveSurfaceExtrema3DFunction<CurveEvaluator, TSurfaceEvaluator> : INewton3DFunction
        where CurveEvaluator : ICurveD2Evaluator
        where TSurfaceEvaluator : ISurfaceD2Evaluator
    {
        internal CurveEvaluator _curve;
        internal TSurfaceEvaluator _surface;

        public bool Invoke(double t, double u, double v,
                           ref Array3<double> f, ref Array3<Array3<double>> j)
        {
            _curve.D2(t, out Point3D ptC, out Vector3D d1C, out Vector3D d2C);
            _surface.D2(u, v, out Point3D ptS, out Vector3D d1U, out Vector3D d1V, out Vector3D d2UU, out Vector3D d2VV, out Vector3D d2UV);

            // D = C - S (vector from surface point to curve point)
            Vector3D d = new(ptS, ptC);

            // Function values: gradient of ||C - S||^2 (factor 2 dropped)
            f[0] = d.Dot(d1C);
            f[1] = -d.Dot(d1U);
            f[2] = -d.Dot(d1V);

            // Jacobian: Hessian of ||C - S||^2 (without factor 2)
            // J[0][0] = dF1/dt = dC/dt . dC/dt + D . d2C/dt2
            j[0][0] = d1C.Dot(d1C) + d.Dot(d2C);
            // J[0][1] = dF1/du = -dS/du . dC/dt
            j[0][1] = -d1U.Dot(d1C);
            // J[0][2] = dF1/dv = -dS/dv . dC/dt
            j[0][2] = -d1V.Dot(d1C);

            // J[1][0] = dF2/dt (symmetric to J[0][1])
            j[1][0] = j[0][1];
            // J[1][1] = dF2/du = dS/du . dS/du - D . d2S/du2
            j[1][1] = d1U.Dot(d1U) - d.Dot(d2UU);
            // J[1][2] = dF2/dv = dS/dv . dS/du - D . d2S/dudv
            j[1][2] = d1V.Dot(d1U) - d.Dot(d2UV);

            // J[2][0] = dF3/dt (symmetric to J[0][2])
            j[2][0] = j[0][2];
            // J[2][1] = dF3/du (symmetric to J[1][2])
            j[2][1] = j[1][2];
            // J[2][2] = dF3/dv = dS/dv . dS/dv - D . d2S/dv2
            j[2][2] = d1V.Dot(d1V) - d.Dot(d2VV);
            return true;
        }
    }

    /// <summary>
    /// 3D Newton solver for extrema between a curve C(t) and a surface S(u, v): F = ((C-S).dC/dt, (S-C).dS/du,
    /// (S-C).dS/dv). The result holds t, u, v in X[0], X[1], X[2].
    /// </summary>
    public static NewtonResultN3 SolveCurveSurfaceExtrema3D<CurveEvaluator, TSurfaceEvaluator>(CurveEvaluator curve,
                                                                                              TSurfaceEvaluator surface,
                                                                                              in Array3<double> x0,
                                                                                              in NewtonBoundsN3 bounds,
                                                                                              NewtonOptions? options = null)
        where CurveEvaluator : ICurveD2Evaluator
        where TSurfaceEvaluator : ISurfaceD2Evaluator
    {
        CurveSurfaceExtrema3DFunction<CurveEvaluator, TSurfaceEvaluator> func = new() { _curve = curve, _surface = surface };

        return Solve3D(func, x0, bounds, options);
    }
}
