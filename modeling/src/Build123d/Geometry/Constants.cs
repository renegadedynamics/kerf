namespace Build123d;

/// <summary>Tolerances used throughout build123d.</summary>
public static class Constants
{
    /// <summary>The default geometric tolerance (1e-6).</summary>
    public const double Tolerance = 1e-6;

    /// <summary>The number of decimal places <see cref="Tolerance"/> resolves (6).</summary>
    public const int ToleranceDigits = 6;

    /// <summary>A looser tolerance (1e-2), used for example as the default bounding box gap.</summary>
    public const double LooseTolerance = 1e-2;

    /// <summary>Decimal places geometry is rounded to for equality and hashing (5).</summary>
    internal const int KeyDigits = ToleranceDigits - 1;

    internal const double DegToRad = Math.PI / 180.0;
    internal const double RadToDeg = 180.0 / Math.PI;
}
