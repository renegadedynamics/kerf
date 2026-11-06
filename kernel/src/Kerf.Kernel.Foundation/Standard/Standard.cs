// Port of: src/FoundationClasses/TKernel/Standard/Standard_Real.hxx
// Port of: src/FoundationClasses/TKernel/Standard/Standard_Integer.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Foundation;

/// <summary>
/// OCCT's free functions on reals and integers (<c>RealSmall()</c>, <c>Square()</c>, <c>IsOdd()</c>, ...).
/// Every Kerf project imports this class with <c>using static</c>, so ported code calls them unqualified,
/// as in C++. Min and Max follow <c>std::min</c>/<c>std::max</c>, not <see cref="Math.Min(double, double)"/>,
/// so they treat NaN as OCCT does.
/// </summary>
internal static class Standard
{
    // The Raise_if macros of OCCT. OCCT's release builds compile most of them out (No_Exception); the port
    // always checks, because a clear exception is worth more than garbage geometry.

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_OutOfRange_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new OutOfRangeException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_ConstructionError_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new ConstructionErrorException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_DomainError_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new DomainErrorException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_DimensionError_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new DimensionErrorException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NullValue_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new NullValueException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NullObject_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new NullObjectException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NotImplemented_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new MethodNotImplementedException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void StdFail_NotDone_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new NotDoneException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_RangeError_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new RangeErrorException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_DimensionMismatch_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new DimensionMismatchException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NoSuchObject_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new NoSuchObjectException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NumericError_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new NumericErrorException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NegativeValue_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new NegativeValueException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_TypeMismatch_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new TypeMismatchException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_ProgramError_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new ProgramErrorException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_DivideByZero_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new ZeroDivisorException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_NoMoreObject_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new NoMoreObjectException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void Standard_MultiplyDefined_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new MultiplyDefinedException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void StdFail_InfiniteSolutions_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new InfiniteSolutionsException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void StdFail_UndefinedDerivative_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new UndefinedDerivativeException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void gp_VectorWithNullMagnitude_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new NullVectorException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void math_NotSquare_Raise_if(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new NotSquareException(message);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void MathSingularMatrixRaiseIf(bool condition, string? message = null)
    {
        if (condition)
        {
            throw new SingularMatrixException(message);
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

    public static double IntToReal(int value) => value;

    /// <summary>True when the values differ by less than <see cref="RealSmall"/>.</summary>
    public static bool IsEqual(double value1, double value2) => Math.Abs(value1 - value2) < RealSmall();

    /// <summary>The distance from <paramref name="value"/> to the next double away from zero.</summary>
    public static double Epsilon(double value)
        => value >= 0.0
               ? NextAfter(value, RealLast()) - value
               : value - NextAfter(value, RealFirst()); // as OCCT: 0 for RealLast() and RealFirst() (decision 1080)

    public static double ACos(double value) => Math.Acos(value);

    public static double ASin(double value) => Math.Asin(value);

    public static double ATan(double value) => Math.Atan(value);

    /// <summary><c>std::atan2(theX, theY)</c>: OCCT names the first argument X, but it is the numerator.</summary>
    public static double ATan2(double x, double y) => Math.Atan2(x, y);

    public static double ATanh(double value) => Math.Atanh(value);

    public static double ACosh(double value) => Math.Acosh(value);

    public static double ASinh(double value) => Math.Asinh(value);

    public static double Cos(double value) => Math.Cos(value);

    public static double Sin(double value) => Math.Sin(value);

    public static double Tan(double value) => Math.Tan(value);

    public static double Cosh(double value) => Math.Cosh(value);

    public static double Sinh(double value) => Math.Sinh(value);

    public static double Tanh(double value) => Math.Tanh(value);

    public static double Exp(double value) => Math.Exp(value);

    public static double Log(double value) => Math.Log(value);

    public static double Log10(double value) => Math.Log10(value);

    public static double Sqrt(double value) => Math.Sqrt(value);

    public static double Pow(double value, double power) => Math.Pow(value, power);

    public static double Abs(double value) => Math.Abs(value);

    public static double Ceiling(double value) => Math.Ceiling(value);

    public static double Floor(double value) => Math.Floor(value);

    public static double IntegerPart(double value) => Math.Truncate(value);

    public static double RealPart(double value) => Math.Abs(Math.Truncate(value) - value);

    /// <summary>Rounds half away from zero, as <c>std::round</c>.</summary>
    public static double Round(double value) => Math.Round(value, MidpointRounding.AwayFromZero);

    public static double NextAfter(double value, double direction)
    {
        if (double.IsNaN(value) || double.IsNaN(direction))
        {
            return double.NaN;
        }

        if (value == direction)
        {
            return direction;
        }

        return direction > value ? Math.BitIncrement(value) : Math.BitDecrement(value);
    }

    /// <summary>The magnitude of the first argument with the sign of the second (<c>std::copysign</c>).</summary>
    public static double Sign(double magnitude, double sign) => Math.CopySign(magnitude, sign);

    public static double Square(double value) => value * value;

    public static double Max(double value1, double value2) => value1 < value2 ? value2 : value1;

    public static double Min(double value1, double value2) => value2 < value1 ? value2 : value1;

    public static int RealToInt(double value)
        => value < int.MinValue ? int.MinValue
           : value > int.MaxValue ? int.MaxValue
           : (int)value;

    // Standard_ShortReal.hxx

    /// <summary>The largest float (FLT_MAX).</summary>
    public static float ShortRealLast() => float.MaxValue;

    public static float RealToShortReal(double value)
        => value < -float.MaxValue ? -float.MaxValue
           : value > float.MaxValue ? float.MaxValue
           : (float)value;

    // Standard_Integer.hxx

    public static int Abs(int value) => value < 0 ? -value : value;

    /// <summary>As C++ <c>% 2 == 0</c>.</summary>
    public static bool IsEven(int value) => value % 2 == 0;

    /// <summary>As C++ <c>% 2 == 1</c>, so false for negative odd numbers, exactly as OCCT.</summary>
    public static bool IsOdd(int value) => value % 2 == 1;

    public static int Max(int value1, int value2) => value1 < value2 ? value2 : value1;

    public static int Min(int value1, int value2) => value2 < value1 ? value2 : value1;

    public static int Modulus(int value, int divisor) => value % divisor;

    public static int Square(int value) => value * value;

    public static int IntegerFirst() => int.MinValue;

    public static int IntegerLast() => int.MaxValue;

    public static int IntegerSize() => 32;

    // M_PI and friends, which OCCT takes from math.h

    public const double M_PI = Math.PI;

    public const double M_PI_2 = Math.PI / 2.0;

    public const double M_PI_4 = Math.PI / 4.0;

    public const double M_SQRT2 = 1.4142135623730951;

    // C's formatting and conversions, which data exchange's ported code calls as C does

    /// <summary>
    /// C's <c>sprintf(buffer, theFormat, ...)</c> as Microsoft's C runtime writes it, invariant culture: the text with
    /// each conversion (<c>%[flags][width][.precision]type</c>) replaced by the next argument; <c>%%</c> is a percent sign.
    /// </summary>
    internal static string Sprintf(string format, params object[] theArgs)
    {
        var result = new System.Text.StringBuilder();
        int arg = 0;
        for (int i = 0; i < format.Length; i++)
        {
            char aChar = format[i];
            if (aChar != '%')
            {
                result.Append(aChar);
                continue;
            }
            if (i + 1 < format.Length && format[i + 1] == '%')
            {
                result.Append('%');
                i++;
                continue;
            }
            int j = i + 1;
            while (j < format.Length && "-+ #0123456789.hlLzjt".IndexOf(format[j]) >= 0)
            {
                j++;
            }
            string conversion = format.Substring(i, Math.Min(j + 1, format.Length) - i);
            object value = arg < theArgs.Length ? theArgs[arg] : "";
            if (conversion.EndsWith('c'))
            {
                // %c: the character, as a one-character string
                conversion = conversion[..^1] + "s";
                value = value is char charArg ? charArg.ToString() : ((char)Convert.ToInt32(value, System.Globalization.CultureInfo.InvariantCulture)).ToString();
            }
            result.Append(Msg.Sprintf(conversion, value));
            arg++;
            i = j;
        }
        return result.ToString();
    }

    /// <summary>
    /// The character at an index of a C string: the string's character, or the terminating null character at or past
    /// its end, so that ported code reading <c>s[i]</c> on a <c>const char*</c> keeps C's behaviour.
    /// </summary>
    internal static char CStringChar(string theString, int index) =>
        index >= 0 && index < theString.Length ? theString[index] : '\0';

    /// <summary>C's <c>atoi</c>: the integer the text starts with (after white space), 0 if none.</summary>
    internal static int Atoi(string text) => TCollection_AsciiString.Strtol(text, out _);

    /// <summary>OCCT's <c>Atof</c> (C's <c>strtod</c> in the C locale): the real the text starts with, 0 if none.</summary>
    internal static double Atof(string text) => StreamUtilities.Strtod(text, out _);
}
