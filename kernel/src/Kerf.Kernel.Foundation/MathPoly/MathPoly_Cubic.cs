// Port of: src/FoundationClasses/TKMath/MathPoly/MathPoly_Cubic.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Polynomials;

public static partial class MathPoly
{
    /// <summary>
    /// Solve cubic equation a*x^3 + b*x^2 + c*x + d = 0 by Cardano's method with the trigonometric solution and
    /// Newton refinement: 1, 2 or 3 real roots, sorted in ascending order.
    /// </summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult Cubic(double theA, double theB, double c, double d)
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult result = new();

        // Reduce to quadratic if leading coefficient is zero
        if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(theA))
        {
            return Quadratic(theB, c, d);
        }

        // Scale coefficients for numerical stability
        double scale = Max(Max(Max(Math.Abs(theA), Math.Abs(theB)), Math.Abs(c)), Math.Abs(d));
        if (scale < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InfiniteSolutions;
            return result;
        }

        // Normalize to monic cubic: x^3 + px^2 + qx + r = 0
        double p = theB / theA;
        double q = c / theA;
        double r = d / theA;

        // Substitute x = t - p/3 to get depressed cubic: t^3 + at + b = 0
        double p3 = p / 3.0;
        double p3_sq = p3 * p3;
        double a = q - 3.0 * p3_sq;
        double b = r - p3 * q + 2.0 * p3_sq * p3;

        // Discriminant: Delta = (b/2)^2 + (a/3)^3
        double halfB = b / 2.0;
        double thirdA = a / 3.0;
        double disc = halfB * halfB + thirdA * thirdA * thirdA;

        // Tolerance for discriminant
        double discTol =
          global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL * Max(halfB * halfB, Math.Abs(thirdA * thirdA * thirdA));

        // Store original coefficients for refinement
        ReadOnlySpan<double> coeffs = [d, c, theB, theA];

        if (disc > discTol)
        {
            // One real root, two complex conjugate roots
            // Cardano's formula: t = cbrt(-b/2 + sqrt(disc)) + cbrt(-b/2 - sqrt(disc))
            double sqrtDisc = Math.Sqrt(disc);
            double u = global::Kerf.Engine.Numerics.Utilities.MathUtils.CubeRoot(-halfB + sqrtDisc);
            double v = global::Kerf.Engine.Numerics.Utilities.MathUtils.CubeRoot(-halfB - sqrtDisc);

            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
            result.NbRoots = 1;
            result.Roots[0] = u + v - p3;

            // Refine root
            result.Roots[0] = global::Kerf.Engine.Numerics.Utilities.MathUtils.RefinePolyRoot(coeffs, 3, result.Roots[0]);
        }
        else if (disc < -discTol)
        {
            // Three distinct real roots - use trigonometric method (casus irreducibilis)
            // t = 2*sqrt(-a/3) * cos(theta/3 + 2*k*pi/3), k = 0, 1, 2
            // where cos(theta) = -b / (2 * sqrt((-a/3)^3))

            double r_val = Math.Sqrt(-thirdA * thirdA * thirdA);
            double cosArg = global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(-halfB / r_val, -1.0, 1.0);
            double theta = Math.Acos(cosArg);
            double twoSqrtNegA3 = 2.0 * Math.Sqrt(-thirdA);

            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
            result.NbRoots = 3;
            result.Roots[0] = twoSqrtNegA3 * Math.Cos(theta / 3.0) - p3;
            result.Roots[1] = twoSqrtNegA3 * Math.Cos((theta + 2.0 * global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_PI) / 3.0) - p3;
            result.Roots[2] = twoSqrtNegA3 * Math.Cos((theta + 4.0 * global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_PI) / 3.0) - p3;

            // Refine all roots
            for (int i = 0; i < 3; ++i)
            {
                result.Roots[i] = global::Kerf.Engine.Numerics.Utilities.MathUtils.RefinePolyRoot(coeffs, 3, result.Roots[i]);
            }

            // Sort roots
            global::Kerf.Engine.Numerics.Utilities.MathUtils.SortRoots(result.Roots, 3);
        }
        else
        {
            // Discriminant is zero - multiple roots
            double u = global::Kerf.Engine.Numerics.Utilities.MathUtils.CubeRoot(-halfB);

            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

            if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(u))
            {
                // Triple root at x = -p/3
                result.NbRoots = 1;
                result.Roots[0] = -p3;
            }
            else
            {
                // One single root and one double root
                result.NbRoots = 2;
                result.Roots[0] = 2.0 * u - p3; // Single root
                result.Roots[1] = -u - p3;      // Double root

                // Refine roots
                result.Roots[0] = global::Kerf.Engine.Numerics.Utilities.MathUtils.RefinePolyRoot(coeffs, 3, result.Roots[0]);
                result.Roots[1] = global::Kerf.Engine.Numerics.Utilities.MathUtils.RefinePolyRoot(coeffs, 3, result.Roots[1]);

                // Sort roots
                if (result.Roots[0] > result.Roots[1])
                {
                    (result.Roots[0], result.Roots[1]) = (result.Roots[1], result.Roots[0]);
                }
            }
        }

        return result;
    }
}
