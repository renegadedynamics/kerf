// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_LineSearch.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Numerics.Utilities;

/// <summary>Line search algorithms for optimization.</summary>
internal static partial class MathUtils
{
    /// <summary>Result of line search operation.</summary>
    public struct LineSearchResult
    {
        /// <summary>True if line search succeeded.</summary>
        public bool IsValid;
        /// <summary>Step size found.</summary>
        public double Alpha;
        /// <summary>Function value at new point.</summary>
        public double FNew;
        /// <summary>Number of function evaluations.</summary>
        public int NbEvals;
    }

    /// <summary>
    /// Backtracking line search with the Armijo condition: finds alpha such that
    /// f(x + alpha*d) &lt;= f(x) + c1*alpha*grad(f).d.
    /// </summary>
    public static LineSearchResult ArmijoBacktrack<Function>(Function func,
                                                             math_Vector x,
                                                             math_Vector dir,
                                                             math_Vector grad,
                                                             double fx,
                                                             double alphaInit = 1.0,
                                                             double c1 = 1.0e-4,
                                                             double rho = 0.5,
                                                             int maxIter = 50)
        where Function : IVectorFunction
    {
        LineSearchResult result = new();
        result.Alpha = alphaInit;
        result.NbEvals = 0;

        int lower = x.Lower();
        int upper = x.Upper();

        // Compute directional derivative: grad(f) . d
        double dirDeriv = 0.0;
        for (int i = lower; i <= upper; ++i)
        {
            dirDeriv += grad[i] * dir[i];
        }

        // Ensure descent direction
        if (dirDeriv >= 0.0)
        {
            // Not a descent direction, return failure
            result.IsValid = false;
            return result;
        }

        // Temporary vector for new point
        math_Vector xNew = new(lower, upper);

        for (int k = 0; k < maxIter; ++k)
        {
            // Compute new point: x + alpha*d
            for (int i = lower; i <= upper; ++i)
            {
                xNew[i] = x[i] + result.Alpha * dir[i];
            }

            // Evaluate function at new point
            if (!func.Value(xNew, out double fNew))
            {
                // Function evaluation failed, reduce step
                result.Alpha *= rho;
                ++result.NbEvals;
                continue;
            }
            ++result.NbEvals;

            // Check Armijo condition: f(x + alpha*d) <= f(x) + c1*alpha*phi'(0)
            if (fNew <= fx + c1 * result.Alpha * dirDeriv)
            {
                result.IsValid = true;
                result.FNew = fNew;
                return result;
            }

            // Reduce step size
            result.Alpha *= rho;

            // Check for too small step
            if (result.Alpha < THE_EPSILON)
            {
                break;
            }
        }

        // Failed to satisfy Armijo condition
        result.IsValid = false;
        return result;
    }

    /// <summary>
    /// Strong Wolfe line search (Armijo plus curvature |grad(f)(x + alpha*d).d| &lt;= c2*|grad(f).d|), by bracketing
    /// and zoom.
    /// </summary>
    public static LineSearchResult WolfeSearch<Function>(Function func,
                                                         math_Vector x,
                                                         math_Vector dir,
                                                         math_Vector grad,
                                                         double fx,
                                                         double alphaInit = 1.0,
                                                         double c1 = 1.0e-4,
                                                         double c2 = 0.9,
                                                         int maxIter = 20)
        where Function : IVectorFunctionWithGradient
    {
        LineSearchResult result = new();
        result.NbEvals = 0;

        int lower = x.Lower();
        int upper = x.Upper();

        // Compute initial directional derivative
        double phi0Prime = 0.0;
        for (int i = lower; i <= upper; ++i)
        {
            phi0Prime += grad[i] * dir[i];
        }

        if (phi0Prime >= 0.0)
        {
            result.IsValid = false;
            return result;
        }

        math_Vector xNew = new(lower, upper);
        math_Vector gradNew = new(lower, upper);

        double alphaLo = 0.0;
        double alphaHi = alphaInit * 2.0;
        double alpha = alphaInit;

        double phiLo = fx;

        // Phase 1: Bracket finding
        for (int k = 0; k < maxIter; ++k)
        {
            // Evaluate at current alpha
            for (int i = lower; i <= upper; ++i)
            {
                xNew[i] = x[i] + alpha * dir[i];
            }

            if (!func.Value(xNew, out double phi))
            {
                // Shrink interval
                alphaHi = alpha;
                alpha = 0.5 * (alphaLo + alphaHi);
                ++result.NbEvals;
                continue;
            }
            ++result.NbEvals;

            // Check Armijo
            if (phi > fx + c1 * alpha * phi0Prime || (k > 0 && phi >= phiLo))
            {
                // Found bracket [alpha_lo, alpha]
                alphaHi = alpha;
                break;
            }

            // Evaluate gradient at new point
            if (!func.Gradient(xNew, gradNew))
            {
                result.IsValid = false;
                return result;
            }

            double phiPrime = 0.0;
            for (int i = lower; i <= upper; ++i)
            {
                phiPrime += gradNew[i] * dir[i];
            }

            // Check curvature condition
            if (Math.Abs(phiPrime) <= -c2 * phi0Prime)
            {
                result.IsValid = true;
                result.Alpha = alpha;
                result.FNew = phi;
                return result;
            }

            if (phiPrime >= 0.0)
            {
                // Found bracket [alpha, alpha_lo]
                alphaHi = alphaLo;
                alphaLo = alpha;
                phiLo = phi;
                break;
            }

            // Increase alpha
            alphaLo = alpha;
            phiLo = phi;
            alpha = 0.5 * (alpha + alphaHi);
        }

        // Phase 2: Zoom
        for (int k = 0; k < maxIter; ++k)
        {
            // Bisection
            alpha = 0.5 * (alphaLo + alphaHi);

            for (int i = lower; i <= upper; ++i)
            {
                xNew[i] = x[i] + alpha * dir[i];
            }

            if (!func.Value(xNew, out double phi))
            {
                alphaHi = alpha;
                ++result.NbEvals;
                continue;
            }
            ++result.NbEvals;

            if (phi > fx + c1 * alpha * phi0Prime || phi >= phiLo)
            {
                alphaHi = alpha;
            }
            else
            {
                if (!func.Gradient(xNew, gradNew))
                {
                    break;
                }

                double phiPrime = 0.0;
                for (int i = lower; i <= upper; ++i)
                {
                    phiPrime += gradNew[i] * dir[i];
                }

                if (Math.Abs(phiPrime) <= -c2 * phi0Prime)
                {
                    result.IsValid = true;
                    result.Alpha = alpha;
                    result.FNew = phi;
                    return result;
                }

                if (phiPrime * (alphaHi - alphaLo) >= 0.0)
                {
                    alphaHi = alphaLo;
                }

                alphaLo = alpha;
                phiLo = phi;
            }

            // Check for convergence
            if (Math.Abs(alphaHi - alphaLo) < THE_EPSILON)
            {
                break;
            }
        }

        // Return best found
        result.IsValid = true;
        result.Alpha = alpha;
        result.FNew = phiLo;
        return result;
    }

    /// <summary>
    /// Exact line search by Brent's method: minimizes f(x + alpha*d) over alpha in [-alpha_max, alpha_max].
    /// </summary>
    public static LineSearchResult ExactLineSearch<Function>(Function func,
                                                             math_Vector x,
                                                             math_Vector dir,
                                                             double alphaMax = 10.0,
                                                             double tolerance = 1.0e-6,
                                                             int maxIter = 100)
        where Function : IVectorFunction
    {
        LineSearchResult result = new();
        result.NbEvals = 0;

        int lower = x.Lower();
        int upper = x.Upper();

        math_Vector xNew = new(lower, upper);

        // Lambda to evaluate phi(alpha) = f(x + alpha*d)
        bool AEvalPhi(double alpha, out double phi)
        {
            for (int i = lower; i <= upper; ++i)
            {
                xNew[i] = x[i] + alpha * dir[i];
            }
            ++result.NbEvals;
            return func.Value(xNew, out phi);
        }

        // Brent's method for 1D minimization
        // Search in [-theAlphaMax, theAlphaMax] to handle both directions
        double a = -alphaMax;
        double b = alphaMax;
        double aX = 0.0; // Start at the current point
        double w = aX;
        double v = aX;

        if (!AEvalPhi(aX, out double fx))
        {
            result.IsValid = false;
            return result;
        }
        double fw = fx;
        double fv = fx;

        double d = 0.0;
        double e = 0.0;

        for (int iter = 0; iter < maxIter; ++iter)
        {
            double xm = 0.5 * (a + b);
            double tol1 = tolerance * Math.Abs(aX) + THE_ZERO_TOL / 10.0;
            double tol2 = 2.0 * tol1;

            // Check convergence
            if (Math.Abs(aX - xm) <= (tol2 - 0.5 * (b - a)))
            {
                result.IsValid = true;
                result.Alpha = aX;
                result.FNew = fx;
                return result;
            }

            double u = 0.0;
            bool useParabolic = false;

            // Try parabolic interpolation
            if (Math.Abs(e) > tol1)
            {
                double r = (aX - w) * (fx - fv);
                double q = (aX - v) * (fx - fw);
                double p = (aX - v) * q - (aX - w) * r;
                q = 2.0 * (q - r);

                if (q > 0.0)
                {
                    p = -p;
                }
                else
                {
                    q = -q;
                }

                double eTmp = e;
                e = d;

                if (Math.Abs(p) < Math.Abs(0.5 * q * eTmp) && p > q * (a - aX) && p < q * (b - aX))
                {
                    d = p / q;
                    u = aX + d;
                    if ((u - a) < tol2 || (b - u) < tol2)
                    {
                        d = SignTransfer(tol1, xm - aX);
                    }
                    useParabolic = true;
                }
            }

            if (!useParabolic)
            {
                e = (aX < xm) ? (b - aX) : (a - aX);
                d = THE_GOLDEN_SECTION * e;
            }

            if (Math.Abs(d) >= tol1)
            {
                u = aX + d;
            }
            else
            {
                u = aX + SignTransfer(tol1, d);
            }

            if (!AEvalPhi(u, out double fu))
            {
                result.IsValid = false;
                result.Alpha = aX;
                result.FNew = fx;
                return result;
            }

            // Update bracket
            if (fu <= fx)
            {
                if (u < aX)
                {
                    b = aX;
                }
                else
                {
                    a = aX;
                }

                v = w;
                w = aX;
                aX = u;
                fv = fw;
                fw = fx;
                fx = fu;
            }
            else
            {
                if (u < aX)
                {
                    a = u;
                }
                else
                {
                    b = u;
                }

                if (fu <= fw || w == aX)
                {
                    v = w;
                    w = u;
                    fv = fw;
                    fw = fu;
                }
                else if (fu <= fv || v == aX || v == w)
                {
                    v = u;
                    fv = fu;
                }
            }
        }

        result.IsValid = true;
        result.Alpha = aX;
        result.FNew = fx;
        return result;
    }

    /// <summary>Quadratic interpolation step for line search from phi(0), phi'(0) and phi(alpha1).</summary>
    public static double QuadraticInterpolation(double phi0, double phi0Prime, double alpha1, double phi1)
    {
        // Quadratic: phi(alpha) = phi(0) + phi'(0)*alpha + c*alpha^2
        // where c = (phi(alpha1) - phi(0) - phi'(0)*alpha1) / alpha1^2
        // Minimum at alpha* = -phi'(0) / (2c)
        double num = phi0Prime * alpha1 * alpha1;
        double denom = 2.0 * (phi1 - phi0 - phi0Prime * alpha1);

        if (Math.Abs(denom) < THE_ZERO_TOL)
        {
            return 0.5 * alpha1;
        }

        double alphaNew = -num / denom;

        // Safeguard: ensure we stay within reasonable bounds
        if (alphaNew < 0.1 * alpha1)
        {
            alphaNew = 0.1 * alpha1;
        }
        else if (alphaNew > 0.9 * alpha1)
        {
            alphaNew = 0.5 * alpha1;
        }

        return alphaNew;
    }

    /// <summary>
    /// Brent's method for 1D minimization along one coordinate of thePoint, in place: the coordinate is modified during
    /// the search and restored if no improvement is found. Returns true if thePoint and theFx were improved.
    /// </summary>
    public static bool BrentAlongCoordinate<Function>(Function func,
                                                      math_Vector point,
                                                      int dimIdx,
                                                      double loBound,
                                                      double upBound,
                                                      ref double fx,
                                                      double tolerance,
                                                      int maxIter,
                                                      ref int evalCount)
        where Function : IVectorFunction
    {
        double origCoord = point[dimIdx];

        // Search interval [aA, aB] in coordinate space (not alpha space)
        double a = loBound;
        double b = upBound;
        double x = origCoord;
        double w = x;
        double v = x;

        double aFx = fx;
        double fw = aFx;
        double fv = aFx;

        double d = 0.0;
        double e = 0.0;

        for (int iter = 0; iter < maxIter; ++iter)
        {
            double xm = 0.5 * (a + b);
            double tol1 = tolerance * Math.Abs(x) + THE_ZERO_TOL / 10.0;
            double tol2 = 2.0 * tol1;

            // Check convergence
            if (Math.Abs(x - xm) <= (tol2 - 0.5 * (b - a)))
            {
                break;
            }

            double u = 0.0;
            bool useParabolic = false;

            // Try parabolic interpolation
            if (Math.Abs(e) > tol1)
            {
                double r = (x - w) * (aFx - fv);
                double q = (x - v) * (aFx - fw);
                double p = (x - v) * q - (x - w) * r;
                q = 2.0 * (q - r);

                if (q > 0.0)
                {
                    p = -p;
                }
                else
                {
                    q = -q;
                }

                double eTmp = e;
                e = d;

                if (Math.Abs(p) < Math.Abs(0.5 * q * eTmp) && p > q * (a - x) && p < q * (b - x))
                {
                    d = p / q;
                    u = x + d;
                    if ((u - a) < tol2 || (b - u) < tol2)
                    {
                        d = SignTransfer(tol1, xm - x);
                    }
                    useParabolic = true;
                }
            }

            if (!useParabolic)
            {
                e = (x < xm) ? (b - x) : (a - x);
                d = THE_GOLDEN_SECTION * e;
            }

            if (Math.Abs(d) >= tol1)
            {
                u = x + d;
            }
            else
            {
                u = x + SignTransfer(tol1, d);
            }

            // Evaluate: only modify the single coordinate
            point[dimIdx] = u;
            if (!func.Value(point, out double fu))
            {
                // Restore and abort
                point[dimIdx] = origCoord;
                return false;
            }
            ++evalCount;

            // Update bracket
            if (fu <= aFx)
            {
                if (u < x)
                {
                    b = x;
                }
                else
                {
                    a = x;
                }
                v = w;
                w = x;
                x = u;
                fv = fw;
                fw = aFx;
                aFx = fu;
            }
            else
            {
                if (u < x)
                {
                    a = u;
                }
                else
                {
                    b = u;
                }
                if (fu <= fw || w == x)
                {
                    v = w;
                    w = u;
                    fv = fw;
                    fw = fu;
                }
                else if (fu <= fv || v == x || v == w)
                {
                    v = u;
                    fv = fu;
                }
            }
        }

        // Accept if improved
        if (aFx < fx)
        {
            point[dimIdx] = x;
            fx = aFx;
            return true;
        }

        // Restore original coordinate
        point[dimIdx] = origCoord;
        return false;
    }
}
