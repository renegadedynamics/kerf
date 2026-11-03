// Port of: src/FoundationClasses/TKMath/MathPoly/MathPoly_Quartic.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Polynomials;

public static partial class MathPoly
{
    /// <summary>
    /// Solve quartic equation a*x^4 + b*x^3 + c*x^2 + d*x + e = 0 by Ferrari's method with Newton refinement:
    /// 0 to 4 real roots, sorted in ascending order.
    /// </summary>
    public static global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult Quartic(double theA, double theB, double theC, double theD, double e)
    {
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult result = new();

        // Reduce to cubic if leading coefficient is zero
        if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(theA))
        {
            return Cubic(theB, theC, theD, e);
        }

        // Scale coefficients for numerical stability
        double scale =
          Max(Max(Max(Max(Math.Abs(theA), Math.Abs(theB)), Math.Abs(theC)), Math.Abs(theD)), Math.Abs(e));
        if (scale < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InfiniteSolutions;
            return result;
        }

        // Normalize to monic quartic: x^4 + ax^3 + bx^2 + cx + d = 0
        double a = theB / theA;
        double b = theC / theA;
        double c = theD / theA;
        double d = e / theA;

        // Substitute x = t - a/4 to get depressed quartic: t^4 + pt^2 + qt + r = 0
        double shift = a / 4.0;
        double a2 = a * a;
        double a3 = a2 * a;
        double a4 = a2 * a2;

        double p = b - 3.0 * a2 / 8.0;
        double q = c - a * b / 2.0 + a3 / 8.0;
        double r = d - a * c / 4.0 + a2 * b / 16.0 - 3.0 * a4 / 256.0;

        // Store original coefficients for refinement
        ReadOnlySpan<double> coeffs = [e, theD, theC, theB, theA];

        // u and v are declared here, where C++ declares them below, because C# doesn't allow the loop's u inside
        // their scope; the loop's u is aU.
        double u, v;

        // Special case: biquadratic (q = 0) -> t^4 + pt^2 + r = 0
        if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(q))
        {
            // Substitute u = t^2 to get u^2 + pu + r = 0
            global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult quadResult = Quadratic(1.0, p, r);

            if (!quadResult.IsDone)
            {
                result.Status = quadResult.Status;
                return result;
            }

            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
            result.NbRoots = 0;

            for (int i = 0; i < quadResult.NbRoots; ++i)
            {
                double aU = quadResult.Roots[i];
                if (aU >= -global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                {
                    if (aU <= global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                    {
                        // u = 0 -> t = 0
                        result.Roots[result.NbRoots++] = -shift;
                    }
                    else
                    {
                        // u > 0 -> t = +/-sqrt(u)
                        double sqrtU = Math.Sqrt(aU);
                        result.Roots[result.NbRoots++] = sqrtU - shift;
                        result.Roots[result.NbRoots++] = -sqrtU - shift;
                    }
                }
            }

            // Refine and sort roots
            for (int i = 0; i < result.NbRoots; ++i)
            {
                result.Roots[i] = global::Kerf.Engine.Numerics.Utilities.MathUtils.RefinePolyRoot(coeffs, 4, result.Roots[i]);
            }
            global::Kerf.Engine.Numerics.Utilities.MathUtils.SortRoots(result.Roots, result.NbRoots);
            result.NbRoots = global::Kerf.Engine.Numerics.Utilities.MathUtils.RemoveDuplicateRoots(result.Roots, result.NbRoots);

            return result;
        }

        // Use resolvent: z^3 + 2pz^2 + (p^2 - 4r)z - q^2 = 0
        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult cubicResult = Cubic(1.0, 2.0 * p, p * p - 4.0 * r, -q * q);

        if (!cubicResult.IsDone || cubicResult.NbRoots == 0)
        {
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
            return result;
        }

        // Find a positive root of the resolvent (there's always at least one)
        double z = cubicResult.Roots[cubicResult.NbRoots - 1]; // Largest root

        // Ensure z is positive (or at least non-negative)
        if (z < -global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
        {
            // Try other roots
            for (int i = 0; i < cubicResult.NbRoots; ++i)
            {
                if (cubicResult.Roots[i] >= -global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                {
                    z = Max(0.0, cubicResult.Roots[i]);
                    break;
                }
            }
        }
        z = Max(0.0, z);

        // Now factor: t^4 + pt^2 + qt + r = (t^2 + st + u)(t^2 - st + v)
        // where s = sqrt(z), u = (p + z)/2 + q/(2s), v = (p + z)/2 - q/(2s)

        double s = Math.Sqrt(z);

        if (global::Kerf.Engine.Numerics.Utilities.MathUtils.IsZero(s))
        {
            // Special case: s = 0
            // t^4 + pt^2 + r = (t^2 + u)(t^2 + v) where u + v = p, uv = r
            global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult uvResult = Quadratic(1.0, p, r);
            if (!uvResult.IsDone || uvResult.NbRoots < 2)
            {
                // Degenerate case
                u = p / 2.0;
                v = p / 2.0;
            }
            else
            {
                u = uvResult.Roots[0];
                v = uvResult.Roots[1];
            }

            // Solve t^2 + u = 0 and t^2 + v = 0
            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
            result.NbRoots = 0;

            if (u <= global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
            {
                double sqrt = Math.Sqrt(Max(0.0, -u));
                result.Roots[result.NbRoots++] = sqrt - shift;
                if (sqrt > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                {
                    result.Roots[result.NbRoots++] = -sqrt - shift;
                }
            }

            if (v <= global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
            {
                double sqrt = Math.Sqrt(Max(0.0, -v));
                result.Roots[result.NbRoots++] = sqrt - shift;
                if (sqrt > global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                {
                    result.Roots[result.NbRoots++] = -sqrt - shift;
                }
            }
        }
        else
        {
            // General case
            double halfPPlusZ = (p + z) / 2.0;
            double qOver2S = q / (2.0 * s);

            u = halfPPlusZ - qOver2S;
            v = halfPPlusZ + qOver2S;

            // Solve t^2 + st + u = 0
            global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult quad1 = Quadratic(1.0, s, u);

            // Solve t^2 - st + v = 0
            global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult quad2 = Quadratic(1.0, -s, v);

            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;
            result.NbRoots = 0;

            if (quad1.IsDone)
            {
                for (int i = 0; i < quad1.NbRoots; ++i)
                {
                    result.Roots[result.NbRoots++] = quad1.Roots[i] - shift;
                }
            }

            if (quad2.IsDone)
            {
                for (int i = 0; i < quad2.NbRoots; ++i)
                {
                    result.Roots[result.NbRoots++] = quad2.Roots[i] - shift;
                }
            }
        }

        // Refine all roots using Newton-Raphson
        for (int i = 0; i < result.NbRoots; ++i)
        {
            result.Roots[i] = global::Kerf.Engine.Numerics.Utilities.MathUtils.RefinePolyRoot(coeffs, 4, result.Roots[i]);
        }

        // Sort roots and remove duplicates
        global::Kerf.Engine.Numerics.Utilities.MathUtils.SortRoots(result.Roots, result.NbRoots);
        result.NbRoots = global::Kerf.Engine.Numerics.Utilities.MathUtils.RemoveDuplicateRoots(result.Roots, result.NbRoots);

        return result;
    }
}
