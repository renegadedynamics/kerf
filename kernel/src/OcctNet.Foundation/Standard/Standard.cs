// Port of: src/FoundationClasses/TKernel/Standard/Standard_Real.hxx
// Port of: src/FoundationClasses/TKernel/Standard/Standard_Integer.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// OCCT's free functions on reals and integers (<c>RealSmall()</c>, <c>Square()</c>, <c>IsOdd()</c>, ...).
/// Every OCCT.NET project imports this class with <c>using static</c>, so ported code calls them unqualified,
/// as in C++. Min and Max follow <c>std::min</c>/<c>std::max</c>, not <see cref="Math.Min(double, double)"/>,
/// so they treat NaN as OCCT does.
/// </summary>
public static class Standard
{
    // The Raise_if macros of OCCT. OCCT's release builds compile most of them out (No_Exception); the port
    // always checks, because a clear exception is worth more than garbage geometry.

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_OutOfRange_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_OutOfRange(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_ConstructionError_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_ConstructionError(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_DomainError_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_DomainError(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_DimensionError_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_DimensionError(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NullValue_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_NullValue(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NullObject_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_NullObject(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NotImplemented_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_NotImplemented(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void StdFail_NotDone_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new StdFail_NotDone(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_RangeError_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_RangeError(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_DimensionMismatch_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_DimensionMismatch(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NoSuchObject_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_NoSuchObject(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NumericError_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_NumericError(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NegativeValue_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_NegativeValue(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_TypeMismatch_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_TypeMismatch(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_DivideByZero_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_DivideByZero(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NoMoreObject_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new Standard_NoMoreObject(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void StdFail_InfiniteSolutions_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new StdFail_InfiniteSolutions(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void StdFail_UndefinedDerivative_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new StdFail_UndefinedDerivative(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void gp_VectorWithNullMagnitude_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new gp_VectorWithNullMagnitude(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void math_NotSquare_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new math_NotSquare(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void math_SingularMatrix_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new math_SingularMatrix(message);
        }
    }

    // Standard_Real.hxx

    /// <summary>The smallest positive normal double (DBL_MIN).</summary>
    public const double RealSmallValue = 2.2250738585072014E-308;

    public static double RealSmall() => RealSmallValue;

    /// <summary>The smallest x with 1.0 + x != 1.0 (DBL_EPSILON).</summary>
    public static double RealEpsilon() => 2.220446049250313E-16;

    public static double RealFirst() => -double.MaxValue;

    public static double RealLast() => double.MaxValue;

    public static int RealDigits() => 15;

    public static int RealFirst10Exp() => -307;

    public static int RealLast10Exp() => 308;

    public static int RealMantissa() => 53;

    public static int RealRadix() => 2;

    public static int RealSize() => 64;

    public static double IntToReal(int theValue) => theValue;

    /// <summary>True when the values differ by less than <see cref="RealSmall"/>.</summary>
    public static bool IsEqual(double theValue1, double theValue2) => Math.Abs(theValue1 - theValue2) < RealSmall();

    /// <summary>The distance from <paramref name="theValue"/> to the next double away from zero.</summary>
    public static double Epsilon(double theValue)
        => theValue >= 0.0
               ? Math.BitIncrement(theValue) - theValue
               : theValue - Math.BitDecrement(theValue);

    public static double ACos(double theValue) => Math.Acos(theValue);

    public static double ASin(double theValue) => Math.Asin(theValue);

    public static double ATan(double theValue) => Math.Atan(theValue);

    /// <summary><c>std::atan2(theX, theY)</c>: OCCT names the first argument X, but it is the numerator.</summary>
    public static double ATan2(double theX, double theY) => Math.Atan2(theX, theY);

    public static double ATanh(double theValue) => Math.Atanh(theValue);

    public static double ACosh(double theValue) => Math.Acosh(theValue);

    public static double ASinh(double theValue) => Math.Asinh(theValue);

    public static double Cos(double theValue) => Math.Cos(theValue);

    public static double Sin(double theValue) => Math.Sin(theValue);

    public static double Tan(double theValue) => Math.Tan(theValue);

    public static double Cosh(double theValue) => Math.Cosh(theValue);

    public static double Sinh(double theValue) => Math.Sinh(theValue);

    public static double Tanh(double theValue) => Math.Tanh(theValue);

    public static double Exp(double theValue) => Math.Exp(theValue);

    public static double Log(double theValue) => Math.Log(theValue);

    public static double Log10(double theValue) => Math.Log10(theValue);

    public static double Sqrt(double theValue) => Math.Sqrt(theValue);

    public static double Pow(double theValue, double thePower) => Math.Pow(theValue, thePower);

    public static double Abs(double theValue) => Math.Abs(theValue);

    public static double Ceiling(double theValue) => Math.Ceiling(theValue);

    public static double Floor(double theValue) => Math.Floor(theValue);

    public static double IntegerPart(double theValue) => Math.Truncate(theValue);

    public static double RealPart(double theValue) => Math.Abs(Math.Truncate(theValue) - theValue);

    /// <summary>Rounds half away from zero, as <c>std::round</c>.</summary>
    public static double Round(double theValue) => Math.Round(theValue, MidpointRounding.AwayFromZero);

    public static double NextAfter(double theValue, double theDirection)
    {
        if (double.IsNaN(theValue) || double.IsNaN(theDirection))
        {
            return double.NaN;
        }

        if (theValue == theDirection)
        {
            return theDirection;
        }

        return theDirection > theValue ? Math.BitIncrement(theValue) : Math.BitDecrement(theValue);
    }

    /// <summary>The magnitude of the first argument with the sign of the second (<c>std::copysign</c>).</summary>
    public static double Sign(double theMagnitude, double theSign) => Math.CopySign(theMagnitude, theSign);

    public static double Square(double theValue) => theValue * theValue;

    public static double Max(double theValue1, double theValue2) => theValue1 < theValue2 ? theValue2 : theValue1;

    public static double Min(double theValue1, double theValue2) => theValue2 < theValue1 ? theValue2 : theValue1;

    public static int RealToInt(double theValue)
        => theValue < int.MinValue ? int.MinValue
           : theValue > int.MaxValue ? int.MaxValue
           : (int)theValue;

    public static float RealToShortReal(double theValue)
        => theValue < -float.MaxValue ? -float.MaxValue
           : theValue > float.MaxValue ? float.MaxValue
           : (float)theValue;

    // Standard_Integer.hxx

    public static int Abs(int theValue) => theValue < 0 ? -theValue : theValue;

    /// <summary>As C++ <c>% 2 == 0</c>.</summary>
    public static bool IsEven(int theValue) => theValue % 2 == 0;

    /// <summary>As C++ <c>% 2 == 1</c>, so false for negative odd numbers, exactly as OCCT.</summary>
    public static bool IsOdd(int theValue) => theValue % 2 == 1;

    public static int Max(int theValue1, int theValue2) => theValue1 < theValue2 ? theValue2 : theValue1;

    public static int Min(int theValue1, int theValue2) => theValue2 < theValue1 ? theValue2 : theValue1;

    public static int Modulus(int theValue, int theDivisor) => theValue % theDivisor;

    public static int Square(int theValue) => theValue * theValue;

    public static int IntegerFirst() => int.MinValue;

    public static int IntegerLast() => int.MaxValue;

    public static int IntegerSize() => 32;

    // M_PI and friends, which OCCT takes from math.h

    public const double M_PI = Math.PI;

    public const double M_PI_2 = Math.PI / 2.0;

    public const double M_PI_4 = Math.PI / 4.0;

    public const double M_SQRT2 = 1.4142135623730951;
}
