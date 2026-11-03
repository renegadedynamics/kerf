// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Core.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Utilities;

/// <summary>Modern math solver utilities: constants and helper functions.</summary>
public static partial class MathUtils
{
    /// <summary>Machine epsilon for double precision (C++'s <c>std::numeric_limits&lt;double&gt;::epsilon()</c>).</summary>
    public const double THE_EPSILON = 2.220446049250313e-16;

    /// <summary>Small value for zero comparisons (more practical than epsilon).</summary>
    public const double THE_ZERO_TOL = 1.0e-15;

    public const double THE_PI = 3.14159265358979323846;

    public const double THE_2PI = 6.28318530717958647692;

    /// <summary>Golden ratio for optimization algorithms.</summary>
    public const double THE_GOLDEN_RATIO = 1.618033988749895;

    /// <summary>Inverse golden ratio (1 - 1/phi).</summary>
    public const double THE_GOLDEN_SECTION = 0.381966011250105;

    /// <summary>Clamp value to range [theLower, theUpper].</summary>
    public static double Clamp(double value, double lower, double upper)
    {
        return (value < lower) ? lower : ((value > upper) ? upper : value);
    }

    /// <summary>Check if value is effectively zero: |theValue| &lt; theTolerance.</summary>
    public static bool IsZero(double value, double tolerance = THE_ZERO_TOL)
    {
        return Math.Abs(value) < tolerance;
    }

    /// <summary>Check if two values are approximately equal (relative tolerance).</summary>
    public static bool IsEqual(double a, double b, double tolerance = THE_ZERO_TOL)
    {
        double diff = Math.Abs(a - b);
        double scale = Max(Max(1.0, Math.Abs(a)), Math.Abs(b));
        return diff < tolerance * scale;
    }

    /// <summary>Safe division avoiding division by zero: theNumerator / theDenominator or theDefault.</summary>
    public static double SafeDiv(double numerator, double denominator, double theDefault = 0.0)
    {
        return IsZero(denominator) ? theDefault : numerator / denominator;
    }

    /// <summary>Sign function: -1 if negative, 0 if zero, +1 if positive (with THE_ZERO_TOL).</summary>
    public static int Sign(double value)
    {
        if (value > THE_ZERO_TOL)
            return 1;
        if (value < -THE_ZERO_TOL)
            return -1;
        return 0;
    }

    /// <summary>Sign transfer function: |theA| with the sign of theB.</summary>
    public static double SignTransfer(double a, double b)
    {
        return (b >= 0.0) ? Math.Abs(a) : -Math.Abs(a);
    }

    public static double Sqr(double value)
    {
        return value * value;
    }

    public static double Cube(double value)
    {
        return value * value * value;
    }

    /// <summary>Cube root with proper sign handling.</summary>
    public static double CubeRoot(double value)
    {
        return (value >= 0.0) ? Math.Cbrt(value) : -Math.Cbrt(-value);
    }

    /// <summary>Check if value is finite (not NaN or Inf).</summary>
    public static bool IsFinite(double value)
    {
        return double.IsFinite(value);
    }

    /// <summary>Compute scaling factor for coefficient normalization (a power of 2 for exact arithmetic).</summary>
    public static double ComputeScaleFactor(ReadOnlySpan<double> coeffs, int count)
    {
        double maxAbs = 0.0;
        for (int i = 0; i < count; ++i)
        {
            maxAbs = Max(maxAbs, Math.Abs(coeffs[i]));
        }

        if (maxAbs < THE_ZERO_TOL || maxAbs > 1.0e15)
        {
            // Find nearest power of 2 for exact scaling
            int exp = FrexpExponent(maxAbs);
            return Math.ScaleB(1.0, -exp + 1);
        }

        return 1.0;
    }

    /// <summary>The exponent C's <c>std::frexp</c> returns: x = m * 2^e with 0.5 &lt;= |m| &lt; 1; 0 for 0, NaN and infinities.</summary>
    internal static int FrexpExponent(double x)
    {
        if (x == 0.0 || !double.IsFinite(x))
        {
            return 0;
        }
        return Math.ILogB(x) + 1;
    }

    /// <summary>Compute dot product of two vectors.</summary>
    public static double DotProduct(math_Vector a, math_Vector b)
    {
        double sum = 0.0;
        int lower = a.Lower();
        int upper = a.Upper();
        for (int i = lower; i <= upper; ++i)
        {
            sum += a[i] * b[i];
        }
        return sum;
    }

    /// <summary>Compute Euclidean norm of a vector.</summary>
    public static double VectorNorm(math_Vector vec)
    {
        double sum = 0.0;
        for (int i = vec.Lower(); i <= vec.Upper(); ++i)
        {
            sum += vec[i] * vec[i];
        }
        return Math.Sqrt(sum);
    }

    /// <summary>Compute infinity norm (maximum absolute value) of a vector.</summary>
    public static double VectorInfNorm(math_Vector vec)
    {
        double max = 0.0;
        for (int i = vec.Lower(); i <= vec.Upper(); ++i)
        {
            max = Max(max, Math.Abs(vec[i]));
        }
        return max;
    }
}
