using System.Globalization;

namespace Kerf.Modeling;

/// <summary>
/// Numbers in the modelling layer's text, in .NET's invariant format (unit 10M1 retired the emulation of Python's
/// <c>repr</c> and format specs): the shortest text that reads back as the same double, and components to a .NET
/// numeric format, values within tolerance of zero shown as zero.
/// </summary>
internal static class Formatting
{
    /// <summary>The shortest invariant text that reads back as <paramref name="value"/> ("1", "0.5", "1E-07").</summary>
    public static string Number(double value) => value.ToString(CultureInfo.InvariantCulture);

    /// <summary>A component in <paramref name="format"/> (a .NET numeric format; "G6" when null or empty), zero within
    /// tolerance of zero, so that noise doesn't print as "-0" or "1E-17".</summary>
    public static string Component(double value, string? format)
    {
        double shown = Math.Abs(value) > Constants.Tolerance ? value : 0.0;
        return shown.ToString(string.IsNullOrEmpty(format) ? "G6" : format, CultureInfo.InvariantCulture);
    }
}
