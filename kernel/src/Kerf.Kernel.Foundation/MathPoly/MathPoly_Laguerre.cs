// Port of: src/FoundationClasses/TKMath/MathPoly/MathPoly_Laguerre.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Numerics.Polynomials;

// std::complex<double> is System.Numerics.Complex (same principal branch for Sqrt; Abs is the magnitude). Coefficient
// pointers are spans. The 20-element std::arrays of GeneralPolyResult are the inline array MathPoly.RootArray<T>.

/// <summary>Polynomial root finding algorithms using Laguerre's method.</summary>
public static partial class MathPoly
{
    /// <summary>Maximum polynomial degree supported by Laguerre solver.</summary>
    public const int THE_MAX_POLY_DEGREE = 20;

    /// <summary>C++'s <c>std::array&lt;T, THE_MAX_POLY_DEGREE&gt;</c>: 20 values held inline, indexed from 0.</summary>
    [InlineArray(THE_MAX_POLY_DEGREE)]
    public struct RootArray<T>
    {
        private T _element0;
    }

    /// <summary>Result for general polynomial solver: real roots (sorted) and complex roots (conjugate pairs).</summary>
    public struct GeneralPolyResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        public RootArray<double> Roots;
        public RootArray<Complex> ComplexRoots;
        public int NbRoots = 0;
        public int NbComplexRoots = 0;

        public GeneralPolyResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(GeneralPolyResult result) => result.IsDone;

        public static bool operator true(GeneralPolyResult result) => result.IsDone;

        public static bool operator false(GeneralPolyResult result) => !result.IsDone;
    }

    internal static partial class detail
    {
        /// <summary>Evaluate polynomial (c[0] = constant term) and its first two derivatives at complex x.</summary>
        internal static void EvaluatePolynomialWithDerivatives(ReadOnlySpan<double> coeffs,
                                                               int degree,
                                                               Complex x,
                                                               out Complex p,
                                                               out Complex dp,
                                                               out Complex d2P)
        {
            // Horner's method with derivative computation
            p = new Complex(coeffs[degree], 0.0);
            dp = new Complex(0.0, 0.0);
            d2P = new Complex(0.0, 0.0);

            for (int i = degree - 1; i >= 0; --i)
            {
                d2P = d2P * x + dp;
                dp = dp * x + p;
                p = p * x + new Complex(coeffs[i], 0.0);
            }
            d2P *= 2.0;
        }

        /// <summary>Laguerre iteration to find one root of polynomial.</summary>
        internal static Complex LaguerreIteration(ReadOnlySpan<double> coeffs,
                                                  int degree,
                                                  Complex x0,
                                                  double tol,
                                                  int maxIter)
        {
            Complex x = x0;
            double n = (double)degree;

            for (int iter = 0; iter < maxIter; ++iter)
            {
                EvaluatePolynomialWithDerivatives(coeffs, degree, x, out Complex p, out Complex dp, out Complex d2P);

                double absP = Complex.Abs(p);
                if (absP < tol)
                {
                    return x;
                }

                // Laguerre's formula: x_new = x - n*P / (P' +/- sqrt((n-1)*((n-1)*P'^2 - n*P*P'')))
                Complex g = dp / p;
                Complex h = g * g - d2P / p;
                Complex sq = Complex.Sqrt((n - 1.0) * (n * h - g * g));

                // Choose denominator with larger magnitude for stability
                Complex denom1 = g + sq;
                Complex denom2 = g - sq;
                Complex denom = (Complex.Abs(denom1) > Complex.Abs(denom2)) ? denom1 : denom2;

                Complex delta;
                if (Complex.Abs(denom) < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                {
                    // Fallback: simple Newton step
                    if (Complex.Abs(dp) < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                    {
                        delta = new Complex(1.0 + Complex.Abs(x), 0.0);
                    }
                    else
                    {
                        delta = p / dp;
                    }
                }
                else
                {
                    delta = new Complex(n, 0.0) / denom;
                }

                x -= delta;

                // Check convergence
                if (Complex.Abs(delta) < tol * (1.0 + Complex.Abs(x)))
                {
                    return x;
                }
            }

            return x;
        }

        /// <summary>Deflate polynomial by removing a real root (synthetic division by (x - root)).</summary>
        internal static void DeflateReal(Span<double> coeffs, ref int degree, double root)
        {
            // Synthetic division
            double carry = coeffs[degree];
            for (int i = degree - 1; i >= 0; --i)
            {
                double temp = coeffs[i];
                coeffs[i] = carry;
                carry = temp + carry * root;
            }
            --degree;
        }

        /// <summary>Deflate polynomial by removing a complex conjugate pair (division by x^2 - 2 Re(root) x + |root|^2).</summary>
        internal static void DeflateComplex(Span<double> coeffs, ref int degree, Complex root)
        {
            // Quadratic factor: x^2 + b*x + c where b = -2*re, c = re^2 + im^2
            double re = root.Real;
            double im = root.Imaginary;
            double b = -2.0 * re;
            double c = re * re + im * im;

            // Synthetic division by (x^2 + b*x + c)
            Span<double> quotient = stackalloc double[THE_MAX_POLY_DEGREE + 1];
            quotient.Fill(0.0);

            // Work from highest degree down
            quotient[degree - 2] = coeffs[degree];
            if (degree >= 3)
            {
                quotient[degree - 3] = coeffs[degree - 1] - b * quotient[degree - 2];
            }

            for (int i = degree - 4; i >= 0; --i)
            {
                quotient[i] = coeffs[i + 2] - b * quotient[i + 1] - c * quotient[i + 2];
            }

            // Copy quotient back to coefficients
            for (int i = 0; i <= degree - 2; ++i)
            {
                coeffs[i] = quotient[i];
            }
            degree -= 2;
        }

        /// <summary>Refine a real root using Newton-Raphson on the original polynomial.</summary>
        internal static double RefineRealRoot(ReadOnlySpan<double> origCoeffs, int origDegree, double root)
        {
            const int THE_MAX_ITER = 10;
            const double THE_TOL = 1.0e-14;

            double x = root;
            for (int iter = 0; iter < THE_MAX_ITER; ++iter)
            {
                // Evaluate P and P' using Horner
                double p = origCoeffs[origDegree];
                double dp = 0.0;
                for (int i = origDegree - 1; i >= 0; --i)
                {
                    dp = dp * x + p;
                    p = p * x + origCoeffs[i];
                }

                if (Math.Abs(dp) < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                {
                    break;
                }

                double delta = p / dp;
                x -= delta;

                if (Math.Abs(delta) < THE_TOL * (1.0 + Math.Abs(x)))
                {
                    break;
                }
            }
            return x;
        }
    }

    /// <summary>
    /// Solve a polynomial equation a0 + a1*x + ... + an*x^n = 0 (degree 1 to THE_MAX_POLY_DEGREE) by Laguerre's
    /// method with deflation; real roots are refined by Newton on the original polynomial, sorted and deduplicated.
    /// </summary>
    public static GeneralPolyResult Laguerre(ReadOnlySpan<double> coeffs, int degree, double tol = 1.0e-12)
    {
        GeneralPolyResult result = new();

        // Validate input
        if (degree < 1 || degree > THE_MAX_POLY_DEGREE)
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput;
            return result;
        }

        // Check leading coefficient
        if (Math.Abs(coeffs[degree]) < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput;
            return result;
        }

        // Copy coefficients for deflation
        Span<double> workCoeffs = stackalloc double[THE_MAX_POLY_DEGREE + 1];
        for (int i = 0; i <= degree; ++i)
        {
            workCoeffs[i] = coeffs[i];
        }

        // Store original for refinement
        Span<double> origCoeffs = stackalloc double[THE_MAX_POLY_DEGREE + 1];
        for (int i = 0; i <= degree; ++i)
        {
            origCoeffs[i] = coeffs[i];
        }

        int deg = degree;

        // Use different starting points to improve convergence
        // Rotate through starting positions
        ReadOnlySpan<Complex> startPoints = [new Complex(0.0, 0.1),
                                              new Complex(1.0, 0.5),
                                              new Complex(-0.5, 0.3),
                                              new Complex(0.5, -0.3)];

        // Find all roots
        int startIdx = 0;
        while (deg > 0)
        {
            Complex x0 = startPoints[startIdx % 4];
            ++startIdx;

            // Use Laguerre iteration
            Complex root = detail.LaguerreIteration(workCoeffs, deg, x0, tol, 100);

            // Determine if root is real or complex
            double imagPart = Math.Abs(root.Imaginary);
            double realPart = Math.Abs(root.Real);
            double scale = Max(1.0, realPart);

            if (imagPart < tol * scale)
            {
                // Real root
                double realRoot = root.Real;

                // Refine using Newton on original polynomial
                realRoot = detail.RefineRealRoot(origCoeffs, degree, realRoot);

                result.Roots[result.NbRoots++] = realRoot;

                // Deflate
                detail.DeflateReal(workCoeffs, ref deg, realRoot);
            }
            else
            {
                // Complex conjugate pair
                result.ComplexRoots[result.NbComplexRoots++] = root;
                result.ComplexRoots[result.NbComplexRoots++] = Complex.Conjugate(root);

                // Deflate by quadratic
                detail.DeflateComplex(workCoeffs, ref deg, root);
            }
        }

        // Sort real roots
        Span<double> roots = result.Roots;
        roots[..result.NbRoots].Sort();

        // Remove duplicate real roots
        if (result.NbRoots > 1)
        {
            int newCount = 1;
            for (int i = 1; i < result.NbRoots; ++i)
            {
                if (Math.Abs(roots[i] - roots[newCount - 1]) > tol)
                {
                    roots[newCount++] = roots[i];
                }
            }
            result.NbRoots = newCount;
        }

        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
        return result;
    }

    /// <summary>Laguerre for a coefficient array of theSize values (degree theSize - 1).</summary>
    public static GeneralPolyResult LaguerreN(ReadOnlySpan<double> coeffs, int size, double tol = 1.0e-12)
    {
        if (size < 2)
        {
            GeneralPolyResult result = new();
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput;
            return result;
        }
        return Laguerre(coeffs, size - 1, tol);
    }

    /// <summary>Real roots of a*x^6 + b*x^5 + c*x^4 + d*x^3 + e*x^2 + f*x + g = 0 (at most 4 kept).</summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult Sextic(double a, double b, double c, double d, double e, double f, double g)
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult result = new();

        // Handle leading zero coefficient
        if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(a))
        {
            // Reduce to quintic, then use Laguerre
            ReadOnlySpan<double> coeffs5 = [g, f, e, d, c, b];
            GeneralPolyResult genResult5 = Laguerre(coeffs5, 5);
            if (!genResult5.IsDone)
            {
                result.Status = genResult5.Status;
                return result;
            }
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
            result.NbRoots = Min(genResult5.NbRoots, 4); // OCCT: real roots beyond the fourth are dropped
            for (int i = 0; i < result.NbRoots; ++i)
            {
                result.Roots[i] = genResult5.Roots[i];
            }
            return result;
        }

        ReadOnlySpan<double> coeffs = [g, f, e, d, c, b, a];
        GeneralPolyResult genResult = Laguerre(coeffs, 6);

        if (!genResult.IsDone)
        {
            result.Status = genResult.Status;
            return result;
        }

        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
        result.NbRoots = Min(genResult.NbRoots, 4); // OCCT: real roots beyond the fourth are dropped
        for (int i = 0; i < result.NbRoots; ++i)
        {
            result.Roots[i] = genResult.Roots[i];
        }

        return result;
    }

    /// <summary>Real roots of a*x^5 + b*x^4 + c*x^3 + d*x^2 + e*x + f = 0 (at most 4 kept).</summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult Quintic(double a, double b, double c, double d, double e, double f)
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult result = new();

        if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(a))
        {
            // Reduce to quartic - use existing solver
            return Quartic(b, c, d, e, f);
        }

        ReadOnlySpan<double> coeffs = [f, e, d, c, b, a];
        GeneralPolyResult genResult = Laguerre(coeffs, 5);

        if (!genResult.IsDone)
        {
            result.Status = genResult.Status;
            return result;
        }

        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
        result.NbRoots = Min(genResult.NbRoots, 4); // OCCT: a fifth real root is dropped
        for (int i = 0; i < result.NbRoots; ++i)
        {
            result.Roots[i] = genResult.Roots[i];
        }

        return result;
    }

    /// <summary>Solve the octic a0 + a1*x + ... + a8*x^8 = 0 (9 coefficients) by Laguerre's method.</summary>
    public static GeneralPolyResult Octic(ReadOnlySpan<double> coeffs)
    {
        return Laguerre(coeffs, 8);
    }
}
