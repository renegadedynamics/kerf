// Port of: src/FoundationClasses/TKMath/MathPoly/MathPoly_Quadratic.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Numerics.Polynomials;

/// <summary>Polynomial root finding algorithms.</summary>
internal static partial class MathPoly
{
    /// <summary>Solve linear equation a*x + b = 0, handling the degenerate case a = 0.</summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult Linear(double a, double b)
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult result = new();

        if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(a))
        {
            if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(b))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InfiniteSolutions;
            }
            else
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NoSolution;
            }
        }
        else
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
            result.NbRoots = 1;
            result.Roots[0] = -b / a;
        }

        return result;
    }

    /// <summary>
    /// Solve quadratic equation a*x^2 + b*x + c = 0 with numerically stable formulas: 0, 1 or 2 real roots, sorted
    /// in ascending order.
    /// </summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult Quadratic(double a, double b, double c)
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult result = new();

        // Linear case: b*x + c = 0
        if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(a))
        {
            return Linear(b, c);
        }

        // Scale coefficients for numerical stability
        double scale = Max(Max(Math.Abs(a), Math.Abs(b)), Math.Abs(c));
        if (scale < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InfiniteSolutions;
            return result;
        }

        double aA = a / scale;
        double aB = b / scale;
        double aC = c / scale;

        // Compute discriminant
        double disc = aB * aB - 4.0 * aA * aC;

        // Tolerance for discriminant based on coefficient magnitudes
        double discTol = global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL * (aB * aB + Math.Abs(4.0 * aA * aC));

        if (disc < -discTol)
        {
            // No real roots (complex conjugate pair)
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
            result.NbRoots = 0;
            return result;
        }

        if (Math.Abs(disc) <= discTol)
        {
            // Double root
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
            result.NbRoots = 1;
            result.Roots[0] = -aB / (2.0 * aA);
            return result;
        }

        // Two distinct real roots
        // Use numerically stable formula to avoid catastrophic cancellation
        double sqrtDisc = Math.Sqrt(disc);

        // q = -0.5 * (b + sign(b) * sqrt(discriminant))
        double q = -0.5 * (aB + global::Kerf.Engine.Numerics.Utilities.MathUtils.SignTransfer(sqrtDisc, aB));

        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
        result.NbRoots = 2;
        result.Roots[0] = q / aA;
        result.Roots[1] = aC / q;

        // Sort roots in ascending order
        if (result.Roots[0] > result.Roots[1])
        {
            (result.Roots[0], result.Roots[1]) = (result.Roots[1], result.Roots[0]);
        }

        return result;
    }
}
