using Kerf.Engine.Geometry;
using Occt;

namespace Build123d;

/// <summary>
/// Rounded keys that geometry equality and hashing compare, so values equal to within
/// <see cref="Constants.KeyDigits"/> decimal places are equal and hash alike.
/// </summary>
internal static class GeomKey
{
    // Adding 0.0 turns -0.0 into 0.0, so the two hash alike as they compare equal
    public static double Round(double value) => Math.Round(value, Constants.KeyDigits) + 0.0;

    /// <summary>The rounded components of a quaternion, with the sign fixed so q and -q (the same rotation) agree.</summary>
    public static (double, double, double, double) Quaternion(Quaternion3 q)
    {
        double x = q.X(), y = q.Y(), z = q.Z(), w = q.W();
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
