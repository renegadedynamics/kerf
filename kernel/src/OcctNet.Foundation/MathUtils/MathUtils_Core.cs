// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Core.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
    public static double Clamp(double theValue, double theLower, double theUpper)
    {
        return (theValue < theLower) ? theLower : ((theValue > theUpper) ? theUpper : theValue);
    }

    /// <summary>Check if value is effectively zero: |theValue| &lt; theTolerance.</summary>
    public static bool IsZero(double theValue, double theTolerance = THE_ZERO_TOL)
    {
        return Math.Abs(theValue) < theTolerance;
    }

    /// <summary>Check if two values are approximately equal (relative tolerance).</summary>
    public static bool IsEqual(double theA, double theB, double theTolerance = THE_ZERO_TOL)
    {
        double aDiff = Math.Abs(theA - theB);
        double aScale = Max(Max(1.0, Math.Abs(theA)), Math.Abs(theB));
        return aDiff < theTolerance * aScale;
    }

    /// <summary>Safe division avoiding division by zero: theNumerator / theDenominator or theDefault.</summary>
    public static double SafeDiv(double theNumerator, double theDenominator, double theDefault = 0.0)
    {
        return IsZero(theDenominator) ? theDefault : theNumerator / theDenominator;
    }

    /// <summary>Sign function: -1 if negative, 0 if zero, +1 if positive (with THE_ZERO_TOL).</summary>
    public static int Sign(double theValue)
    {
        if (theValue > THE_ZERO_TOL)
            return 1;
        if (theValue < -THE_ZERO_TOL)
            return -1;
        return 0;
    }

    /// <summary>Sign transfer function: |theA| with the sign of theB.</summary>
    public static double SignTransfer(double theA, double theB)
    {
        return (theB >= 0.0) ? Math.Abs(theA) : -Math.Abs(theA);
    }

    public static double Sqr(double theValue)
    {
        return theValue * theValue;
    }

    public static double Cube(double theValue)
    {
        return theValue * theValue * theValue;
    }

    /// <summary>Cube root with proper sign handling.</summary>
    public static double CubeRoot(double theValue)
    {
        return (theValue >= 0.0) ? Math.Cbrt(theValue) : -Math.Cbrt(-theValue);
    }

    /// <summary>Check if value is finite (not NaN or Inf).</summary>
    public static bool IsFinite(double theValue)
    {
        return double.IsFinite(theValue);
    }

    /// <summary>Compute scaling factor for coefficient normalization (a power of 2 for exact arithmetic).</summary>
    public static double ComputeScaleFactor(ReadOnlySpan<double> theCoeffs, int theCount)
    {
        double aMaxAbs = 0.0;
        for (int i = 0; i < theCount; ++i)
        {
            aMaxAbs = Max(aMaxAbs, Math.Abs(theCoeffs[i]));
        }

        if (aMaxAbs < THE_ZERO_TOL || aMaxAbs > 1.0e15)
        {
            // Find nearest power of 2 for exact scaling
            int anExp = frexpExponent(aMaxAbs);
            return Math.ScaleB(1.0, -anExp + 1);
        }

        return 1.0;
    }

    /// <summary>The exponent C's <c>std::frexp</c> returns: x = m * 2^e with 0.5 &lt;= |m| &lt; 1; 0 for 0, NaN and infinities.</summary>
    internal static int frexpExponent(double theX)
    {
        if (theX == 0.0 || !double.IsFinite(theX))
        {
            return 0;
        }
        return Math.ILogB(theX) + 1;
    }

    /// <summary>Compute dot product of two vectors.</summary>
    public static double DotProduct(math_Vector theA, math_Vector theB)
    {
        double aSum = 0.0;
        int aLower = theA.Lower();
        int aUpper = theA.Upper();
        for (int i = aLower; i <= aUpper; ++i)
        {
            aSum += theA[i] * theB[i];
        }
        return aSum;
    }

    /// <summary>Compute Euclidean norm of a vector.</summary>
    public static double VectorNorm(math_Vector theVec)
    {
        double aSum = 0.0;
        for (int i = theVec.Lower(); i <= theVec.Upper(); ++i)
        {
            aSum += theVec[i] * theVec[i];
        }
        return Math.Sqrt(aSum);
    }

    /// <summary>Compute infinity norm (maximum absolute value) of a vector.</summary>
    public static double VectorInfNorm(math_Vector theVec)
    {
        double aMax = 0.0;
        for (int i = theVec.Lower(); i <= theVec.Upper(); ++i)
        {
            aMax = Max(aMax, Math.Abs(theVec[i]));
        }
        return aMax;
    }
}
