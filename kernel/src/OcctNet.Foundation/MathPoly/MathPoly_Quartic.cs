// Port of: src/FoundationClasses/TKMath/MathPoly/MathPoly_Quartic.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

public static partial class MathPoly
{
    /// <summary>
    /// Solve quartic equation a*x^4 + b*x^3 + c*x^2 + d*x + e = 0 by Ferrari's method with Newton refinement:
    /// 0 to 4 real roots, sorted in ascending order.
    /// </summary>
    public static MathUtils.PolyResult Quartic(double theA, double theB, double theC, double theD, double theE)
    {
        MathUtils.PolyResult aResult = new();

        // Reduce to cubic if leading coefficient is zero
        if (MathUtils.IsZero(theA))
        {
            return Cubic(theB, theC, theD, theE);
        }

        // Scale coefficients for numerical stability
        double aScale =
          Max(Max(Max(Max(Math.Abs(theA), Math.Abs(theB)), Math.Abs(theC)), Math.Abs(theD)), Math.Abs(theE));
        if (aScale < MathUtils.THE_ZERO_TOL)
        {
            aResult.Status = MathUtils.Status.InfiniteSolutions;
            return aResult;
        }

        // Normalize to monic quartic: x^4 + ax^3 + bx^2 + cx + d = 0
        double a = theB / theA;
        double b = theC / theA;
        double c = theD / theA;
        double d = theE / theA;

        // Substitute x = t - a/4 to get depressed quartic: t^4 + pt^2 + qt + r = 0
        double aShift = a / 4.0;
        double a2 = a * a;
        double a3 = a2 * a;
        double a4 = a2 * a2;

        double p = b - 3.0 * a2 / 8.0;
        double q = c - a * b / 2.0 + a3 / 8.0;
        double r = d - a * c / 4.0 + a2 * b / 16.0 - 3.0 * a4 / 256.0;

        // Store original coefficients for refinement
        ReadOnlySpan<double> aCoeffs = [theE, theD, theC, theB, theA];

        // u and v are declared here, where C++ declares them below, because C# doesn't allow the loop's u inside
        // their scope; the loop's u is aU.
        double u, v;

        // Special case: biquadratic (q = 0) -> t^4 + pt^2 + r = 0
        if (MathUtils.IsZero(q))
        {
            // Substitute u = t^2 to get u^2 + pu + r = 0
            MathUtils.PolyResult aQuadResult = Quadratic(1.0, p, r);

            if (!aQuadResult.IsDone())
            {
                aResult.Status = aQuadResult.Status;
                return aResult;
            }

            aResult.Status = MathUtils.Status.OK;
            aResult.NbRoots = 0;

            for (int i = 0; i < aQuadResult.NbRoots; ++i)
            {
                double aU = aQuadResult.Roots[i];
                if (aU >= -MathUtils.THE_ZERO_TOL)
                {
                    if (aU <= MathUtils.THE_ZERO_TOL)
                    {
                        // u = 0 -> t = 0
                        aResult.Roots[aResult.NbRoots++] = -aShift;
                    }
                    else
                    {
                        // u > 0 -> t = +/-sqrt(u)
                        double aSqrtU = Math.Sqrt(aU);
                        aResult.Roots[aResult.NbRoots++] = aSqrtU - aShift;
                        aResult.Roots[aResult.NbRoots++] = -aSqrtU - aShift;
                    }
                }
            }

            // Refine and sort roots
            for (int i = 0; i < aResult.NbRoots; ++i)
            {
                aResult.Roots[i] = MathUtils.RefinePolyRoot(aCoeffs, 4, aResult.Roots[i]);
            }
            MathUtils.SortRoots(aResult.Roots, aResult.NbRoots);
            aResult.NbRoots = MathUtils.RemoveDuplicateRoots(aResult.Roots, aResult.NbRoots);

            return aResult;
        }

        // Use resolvent: z^3 + 2pz^2 + (p^2 - 4r)z - q^2 = 0
        MathUtils.PolyResult aCubicResult = Cubic(1.0, 2.0 * p, p * p - 4.0 * r, -q * q);

        if (!aCubicResult.IsDone() || aCubicResult.NbRoots == 0)
        {
            aResult.Status = MathUtils.Status.NumericalError;
            return aResult;
        }

        // Find a positive root of the resolvent (there's always at least one)
        double z = aCubicResult.Roots[aCubicResult.NbRoots - 1]; // Largest root

        // Ensure z is positive (or at least non-negative)
        if (z < -MathUtils.THE_ZERO_TOL)
        {
            // Try other roots
            for (int i = 0; i < aCubicResult.NbRoots; ++i)
            {
                if (aCubicResult.Roots[i] >= -MathUtils.THE_ZERO_TOL)
                {
                    z = Max(0.0, aCubicResult.Roots[i]);
                    break;
                }
            }
        }
        z = Max(0.0, z);

        // Now factor: t^4 + pt^2 + qt + r = (t^2 + st + u)(t^2 - st + v)
        // where s = sqrt(z), u = (p + z)/2 + q/(2s), v = (p + z)/2 - q/(2s)

        double s = Math.Sqrt(z);

        if (MathUtils.IsZero(s))
        {
            // Special case: s = 0
            // t^4 + pt^2 + r = (t^2 + u)(t^2 + v) where u + v = p, uv = r
            MathUtils.PolyResult aUVResult = Quadratic(1.0, p, r);
            if (!aUVResult.IsDone() || aUVResult.NbRoots < 2)
            {
                // Degenerate case
                u = p / 2.0;
                v = p / 2.0;
            }
            else
            {
                u = aUVResult.Roots[0];
                v = aUVResult.Roots[1];
            }

            // Solve t^2 + u = 0 and t^2 + v = 0
            aResult.Status = MathUtils.Status.OK;
            aResult.NbRoots = 0;

            if (u <= MathUtils.THE_ZERO_TOL)
            {
                double aSqrt = Math.Sqrt(Max(0.0, -u));
                aResult.Roots[aResult.NbRoots++] = aSqrt - aShift;
                if (aSqrt > MathUtils.THE_ZERO_TOL)
                {
                    aResult.Roots[aResult.NbRoots++] = -aSqrt - aShift;
                }
            }

            if (v <= MathUtils.THE_ZERO_TOL)
            {
                double aSqrt = Math.Sqrt(Max(0.0, -v));
                aResult.Roots[aResult.NbRoots++] = aSqrt - aShift;
                if (aSqrt > MathUtils.THE_ZERO_TOL)
                {
                    aResult.Roots[aResult.NbRoots++] = -aSqrt - aShift;
                }
            }
        }
        else
        {
            // General case
            double aHalfPPlusZ = (p + z) / 2.0;
            double aQOver2S = q / (2.0 * s);

            u = aHalfPPlusZ - aQOver2S;
            v = aHalfPPlusZ + aQOver2S;

            // Solve t^2 + st + u = 0
            MathUtils.PolyResult aQuad1 = Quadratic(1.0, s, u);

            // Solve t^2 - st + v = 0
            MathUtils.PolyResult aQuad2 = Quadratic(1.0, -s, v);

            aResult.Status = MathUtils.Status.OK;
            aResult.NbRoots = 0;

            if (aQuad1.IsDone())
            {
                for (int i = 0; i < aQuad1.NbRoots; ++i)
                {
                    aResult.Roots[aResult.NbRoots++] = aQuad1.Roots[i] - aShift;
                }
            }

            if (aQuad2.IsDone())
            {
                for (int i = 0; i < aQuad2.NbRoots; ++i)
                {
                    aResult.Roots[aResult.NbRoots++] = aQuad2.Roots[i] - aShift;
                }
            }
        }

        // Refine all roots using Newton-Raphson
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            aResult.Roots[i] = MathUtils.RefinePolyRoot(aCoeffs, 4, aResult.Roots[i]);
        }

        // Sort roots and remove duplicates
        MathUtils.SortRoots(aResult.Roots, aResult.NbRoots);
        aResult.NbRoots = MathUtils.RemoveDuplicateRoots(aResult.Roots, aResult.NbRoots);

        return aResult;
    }
}
