// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Poly.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Utilities;

/// <summary>Polynomial evaluation and manipulation.</summary>
public static partial class MathUtils
{
    /// <summary>Evaluate polynomial a0 + a1*x + ... (constant term first) using Horner's method.</summary>
    public static double EvalPoly(ReadOnlySpan<double> coeffs, int degree, double x)
    {
        double result = coeffs[degree];
        for (int i = degree - 1; i >= 0; --i)
        {
            result = result * x + coeffs[i];
        }
        return result;
    }

    /// <summary>Evaluate polynomial (constant term first) and its derivative in a single pass.</summary>
    public static void EvalPolyDeriv(ReadOnlySpan<double> coeffs, int degree, double x, out double value, out double deriv)
    {
        value = coeffs[degree];
        deriv = 0.0;
        for (int i = degree - 1; i >= 0; --i)
        {
            deriv = deriv * x + value;
            value = value * x + coeffs[i];
        }
    }

    /// <summary>Evaluate polynomial with coefficients in descending order (leading term first).</summary>
    public static double EvalPolyDesc(ReadOnlySpan<double> coeffs, int degree, double x)
    {
        double result = coeffs[0];
        for (int i = 1; i <= degree; ++i)
        {
            result = result * x + coeffs[i];
        }
        return result;
    }

    /// <summary>Newton-Raphson refinement for a polynomial root (constant term first).</summary>
    public static double RefinePolyRoot(ReadOnlySpan<double> coeffs, int degree, double root, int maxIter = 5)
    {
        double x = root;
        for (int i = 0; i < maxIter; ++i)
        {
            EvalPolyDeriv(coeffs, degree, x, out double f, out double df);

            if (IsZero(df))
            {
                break;
            }

            double dx = f / df;
            x -= dx;

            if (Math.Abs(dx) < THE_EPSILON * Max(1.0, Math.Abs(x)))
            {
                break;
            }
        }
        return x;
    }

    /// <summary>Newton-Raphson refinement with coefficients in descending order (degree at most 4).</summary>
    public static double RefinePolyRootDesc(ReadOnlySpan<double> coeffs, int degree, double root, int maxIter = 5)
    {
        // Convert to ascending order for refinement
        Span<double> asc = stackalloc double[5];
        for (int i = 0; i <= degree; ++i)
        {
            asc[i] = coeffs[degree - i];
        }
        return RefinePolyRoot(asc, degree, root, maxIter);
    }

    /// <summary>Sort roots in ascending order (insertion sort for small arrays).</summary>
    public static void SortRoots(Span<double> roots, int count)
    {
        for (int i = 1; i < count; ++i)
        {
            double key = roots[i];
            int j = i;
            while (j > 0 && roots[j - 1] > key)
            {
                roots[j] = roots[j - 1];
                --j;
            }
            roots[j] = key;
        }
    }

    /// <summary>Remove duplicate sorted roots within tolerance; returns the new count.</summary>
    public static int RemoveDuplicateRoots(Span<double> roots, int count, double tolerance = 1.0e-10)
    {
        if (count <= 1)
        {
            return count;
        }

        int newCount = 1;
        for (int i = 1; i < count; ++i)
        {
            if (Math.Abs(roots[i] - roots[newCount - 1]) > tolerance)
            {
                roots[newCount] = roots[i];
                ++newCount;
            }
        }
        return newCount;
    }

    /// <summary>Depressed cubic: x^3 + bx^2 + cx + d to t^3 + pt + q via x = t - b/3.</summary>
    public static void DepressCubic(double b, double c, double d, out double p, out double q, out double shift)
    {
        shift = b / 3.0;
        double b2 = b * b;
        p = c - b2 / 3.0;
        q = d - b * c / 3.0 + 2.0 * b2 * b / 27.0;
    }

    /// <summary>Depressed quartic: x^4 + bx^3 + cx^2 + dx + e to t^4 + pt^2 + qt + r via x = t - b/4.</summary>
    public static void DepressQuartic(double b, double c, double d, double e,
                                      out double p, out double q, out double r, out double shift)
    {
        shift = b / 4.0;
        double b2 = b * b;
        double b3 = b2 * b;
        double b4 = b2 * b2;

        p = c - 3.0 * b2 / 8.0;
        q = d - b * c / 2.0 + b3 / 8.0;
        r = e - b * d / 4.0 + b2 * c / 16.0 - 3.0 * b4 / 256.0;
    }
}
