using System.Globalization;

namespace Kerf.Modeling;

/// <summary>Number formatting that matches Python's, so text output reads as build123d's does.</summary>
internal static class PyFormat
{
    static readonly CultureInfo Inv = CultureInfo.InvariantCulture;

    /// <summary>Python's format(value, ".{precision}g").</summary>
    public static string G(double value, int precision)
    {
        // .NET's G switches to exponent form by the same rule as Python's g; only the letter case differs
        return value.ToString("G" + Math.Max(precision, 1), Inv).Replace("E", "e");
    }

    /// <summary>Python's format(value, ".{precision}f").</summary>
    public static string F(double value, int precision) => value.ToString("F" + precision, Inv);

    /// <summary>Python's repr(value) for a float: the shortest round-trip digits, always with a point or exponent.</summary>
    public static string Repr(double value)
    {
        if (double.IsNaN(value))
            return "nan";
        if (double.IsInfinity(value))
            return value > 0 ? "inf" : "-inf";
        var sign = value < 0 || (value == 0 && double.IsNegative(value)) ? "-" : "";
        if (value == 0)
            return sign + "0.0";

        // Shortest round-trip digits and the decimal point's position: value = 0.digits * 10^point
        var r = Math.Abs(value).ToString("R", Inv);
        int e = r.IndexOf('E');
        int exp = e < 0 ? 0 : int.Parse(r.AsSpan(e + 1), NumberStyles.AllowLeadingSign, Inv);
        var mantissa = e < 0 ? r : r[..e];
        int dot = mantissa.IndexOf('.');
        var intPart = dot < 0 ? mantissa : mantissa[..dot];
        var all = intPart + (dot < 0 ? "" : mantissa[(dot + 1)..]);
        int zeros = all.Length - all.TrimStart('0').Length;
        var digits = all[zeros..].TrimEnd('0');
        int point = intPart.Length + exp - zeros;

        string body;
        if (point > -4 && point <= 16)
        {
            if (point <= 0)
                body = "0." + new string('0', -point) + digits;
            else if (point >= digits.Length)
                body = digits + new string('0', point - digits.Length) + ".0";
            else
                body = digits[..point] + "." + digits[point..];
        }
        else
        {
            var m = digits.Length > 1 ? digits[0] + "." + digits[1..] : digits;
            int x = point - 1;
            body = $"{m}e{(x < 0 ? '-' : '+')}{Math.Abs(x):00}";
        }
        return sign + body;
    }

    /// <summary>
    /// Formats one component of a geometry value as build123d does for a "f" or "g" spec: values within
    /// <see cref="Constants.Tolerance"/> of zero print as zero, and others are rounded to the precision
    /// first. Any other spec, as in build123d, gives Python's repr of the value.
    /// </summary>
    public static string Component(double value, string? spec)
    {
        if (string.IsNullOrEmpty(spec) || spec[^1] is not ('f' or 'g'))
            return Repr(value);
        char kind = spec[^1];
        int point = spec.IndexOf('.');
        int precision = point >= 0
            ? int.Parse(spec.AsSpan(point + 1, spec.Length - point - 2), Inv)
            : kind == 'f' ? 6 : 12;
        double trimmed = Math.Abs(value) > Constants.Tolerance ? Math.Round(value, Math.Min(precision, 15)) : 0.0;
        return kind == 'f' ? F(trimmed, precision) : G(trimmed, precision);
    }
}
