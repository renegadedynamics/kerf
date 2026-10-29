// Port of: src/FoundationClasses/TKMath/MathPoly/MathPoly_Cubic.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

public static partial class MathPoly
{
    /// <summary>
    /// Solve cubic equation a*x^3 + b*x^2 + c*x + d = 0 by Cardano's method with the trigonometric solution and
    /// Newton refinement: 1, 2 or 3 real roots, sorted in ascending order.
    /// </summary>
    public static MathUtils.PolyResult Cubic(double theA, double theB, double theC, double theD)
    {
        MathUtils.PolyResult aResult = new();

        // Reduce to quadratic if leading coefficient is zero
        if (MathUtils.IsZero(theA))
        {
            return Quadratic(theB, theC, theD);
        }

        // Scale coefficients for numerical stability
        double aScale = Max(Max(Max(Math.Abs(theA), Math.Abs(theB)), Math.Abs(theC)), Math.Abs(theD));
        if (aScale < MathUtils.THE_ZERO_TOL)
        {
            aResult.Status = MathUtils.Status.InfiniteSolutions;
            return aResult;
        }

        // Normalize to monic cubic: x^3 + px^2 + qx + r = 0
        double aP = theB / theA;
        double aQ = theC / theA;
        double aR = theD / theA;

        // Substitute x = t - p/3 to get depressed cubic: t^3 + at + b = 0
        double aP3 = aP / 3.0;
        double aP3_sq = aP3 * aP3;
        double a = aQ - 3.0 * aP3_sq;
        double b = aR - aP3 * aQ + 2.0 * aP3_sq * aP3;

        // Discriminant: Delta = (b/2)^2 + (a/3)^3
        double aHalfB = b / 2.0;
        double aThirdA = a / 3.0;
        double aDisc = aHalfB * aHalfB + aThirdA * aThirdA * aThirdA;

        // Tolerance for discriminant
        double aDiscTol =
          MathUtils.THE_ZERO_TOL * Max(aHalfB * aHalfB, Math.Abs(aThirdA * aThirdA * aThirdA));

        // Store original coefficients for refinement
        ReadOnlySpan<double> aCoeffs = [theD, theC, theB, theA];

        if (aDisc > aDiscTol)
        {
            // One real root, two complex conjugate roots
            // Cardano's formula: t = cbrt(-b/2 + sqrt(disc)) + cbrt(-b/2 - sqrt(disc))
            double aSqrtDisc = Math.Sqrt(aDisc);
            double aU = MathUtils.CubeRoot(-aHalfB + aSqrtDisc);
            double aV = MathUtils.CubeRoot(-aHalfB - aSqrtDisc);

            aResult.Status = MathUtils.Status.OK;
            aResult.NbRoots = 1;
            aResult.Roots[0] = aU + aV - aP3;

            // Refine root
            aResult.Roots[0] = MathUtils.RefinePolyRoot(aCoeffs, 3, aResult.Roots[0]);
        }
        else if (aDisc < -aDiscTol)
        {
            // Three distinct real roots - use trigonometric method (casus irreducibilis)
            // t = 2*sqrt(-a/3) * cos(theta/3 + 2*k*pi/3), k = 0, 1, 2
            // where cos(theta) = -b / (2 * sqrt((-a/3)^3))

            double aR_val = Math.Sqrt(-aThirdA * aThirdA * aThirdA);
            double aCosArg = MathUtils.Clamp(-aHalfB / aR_val, -1.0, 1.0);
            double aTheta = Math.Acos(aCosArg);
            double aTwoSqrtNegA3 = 2.0 * Math.Sqrt(-aThirdA);

            aResult.Status = MathUtils.Status.OK;
            aResult.NbRoots = 3;
            aResult.Roots[0] = aTwoSqrtNegA3 * Math.Cos(aTheta / 3.0) - aP3;
            aResult.Roots[1] = aTwoSqrtNegA3 * Math.Cos((aTheta + 2.0 * MathUtils.THE_PI) / 3.0) - aP3;
            aResult.Roots[2] = aTwoSqrtNegA3 * Math.Cos((aTheta + 4.0 * MathUtils.THE_PI) / 3.0) - aP3;

            // Refine all roots
            for (int i = 0; i < 3; ++i)
            {
                aResult.Roots[i] = MathUtils.RefinePolyRoot(aCoeffs, 3, aResult.Roots[i]);
            }

            // Sort roots
            MathUtils.SortRoots(aResult.Roots, 3);
        }
        else
        {
            // Discriminant is zero - multiple roots
            double aU = MathUtils.CubeRoot(-aHalfB);

            aResult.Status = MathUtils.Status.OK;

            if (MathUtils.IsZero(aU))
            {
                // Triple root at x = -p/3
                aResult.NbRoots = 1;
                aResult.Roots[0] = -aP3;
            }
            else
            {
                // One single root and one double root
                aResult.NbRoots = 2;
                aResult.Roots[0] = 2.0 * aU - aP3; // Single root
                aResult.Roots[1] = -aU - aP3;      // Double root

                // Refine roots
                aResult.Roots[0] = MathUtils.RefinePolyRoot(aCoeffs, 3, aResult.Roots[0]);
                aResult.Roots[1] = MathUtils.RefinePolyRoot(aCoeffs, 3, aResult.Roots[1]);

                // Sort roots
                if (aResult.Roots[0] > aResult.Roots[1])
                {
                    (aResult.Roots[0], aResult.Roots[1]) = (aResult.Roots[1], aResult.Roots[0]);
                }
            }
        }

        return aResult;
    }
}
