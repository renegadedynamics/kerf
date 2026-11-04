namespace Kerf.Modeling;

/// <summary>
/// Rounded keys that geometry equality and hashing compare, so values equal to within
/// <see cref="Constants.KeyDigits"/> decimal places are equal and hash alike.
/// </summary>
internal static class GeomKey
{
    // Adding 0.0 turns -0.0 into 0.0, so the two hash alike as they compare equal
    public static double Round(double value) => Math.Round(value, Constants.KeyDigits) + 0.0;

    /// <summary>The rounded components of a quaternion, with the sign fixed so q and -q (the same rotation) agree.</summary>
    public static (double, double, double, double) Quaternion((double X, double Y, double Z, double W) q)
    {
        var (x, y, z, w) = q;
        foreach (var value in (ReadOnlySpan<double>)[x, y, z, w])
        {
            if (Math.Abs(value) > Constants.Tolerance)
            {
                if (value < 0)
                    (x, y, z, w) = (-x, -y, -z, -w);
                break;
            }
        }
        return (Round(x), Round(y), Round(z), Round(w));
    }
}
