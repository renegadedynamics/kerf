// Port of: src/FoundationClasses/TKMath/MathPoly/MathPoly_Quadratic.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Polynomial root finding algorithms.</summary>
public static partial class MathPoly
{
    /// <summary>Solve linear equation a*x + b = 0, handling the degenerate case a = 0.</summary>
    public static MathUtils.PolyResult Linear(double theA, double theB)
    {
        MathUtils.PolyResult aResult = new();

        if (MathUtils.IsZero(theA))
        {
            if (MathUtils.IsZero(theB))
            {
                aResult.Status = MathUtils.Status.InfiniteSolutions;
            }
            else
            {
                aResult.Status = MathUtils.Status.NoSolution;
            }
        }
        else
        {
            aResult.Status = MathUtils.Status.OK;
            aResult.NbRoots = 1;
            aResult.Roots[0] = -theB / theA;
        }

        return aResult;
    }

    /// <summary>
    /// Solve quadratic equation a*x^2 + b*x + c = 0 with numerically stable formulas: 0, 1 or 2 real roots, sorted
    /// in ascending order.
    /// </summary>
    public static MathUtils.PolyResult Quadratic(double theA, double theB, double theC)
    {
        MathUtils.PolyResult aResult = new();

        // Linear case: b*x + c = 0
        if (MathUtils.IsZero(theA))
        {
            return Linear(theB, theC);
        }

        // Scale coefficients for numerical stability
        double aScale = Max(Max(Math.Abs(theA), Math.Abs(theB)), Math.Abs(theC));
        if (aScale < MathUtils.THE_ZERO_TOL)
        {
            aResult.Status = MathUtils.Status.InfiniteSolutions;
            return aResult;
        }

        double aA = theA / aScale;
        double aB = theB / aScale;
        double aC = theC / aScale;

        // Compute discriminant
        double aDisc = aB * aB - 4.0 * aA * aC;

        // Tolerance for discriminant based on coefficient magnitudes
        double aDiscTol = MathUtils.THE_ZERO_TOL * (aB * aB + Math.Abs(4.0 * aA * aC));

        if (aDisc < -aDiscTol)
        {
            // No real roots (complex conjugate pair)
            aResult.Status = MathUtils.Status.OK;
            aResult.NbRoots = 0;
            return aResult;
        }

        if (Math.Abs(aDisc) <= aDiscTol)
        {
            // Double root
            aResult.Status = MathUtils.Status.OK;
            aResult.NbRoots = 1;
            aResult.Roots[0] = -aB / (2.0 * aA);
            return aResult;
        }

        // Two distinct real roots
        // Use numerically stable formula to avoid catastrophic cancellation
        double aSqrtDisc = Math.Sqrt(aDisc);

        // q = -0.5 * (b + sign(b) * sqrt(discriminant))
        double aQ = -0.5 * (aB + MathUtils.SignTransfer(aSqrtDisc, aB));

        aResult.Status = MathUtils.Status.OK;
        aResult.NbRoots = 2;
        aResult.Roots[0] = aQ / aA;
        aResult.Roots[1] = aC / aQ;

        // Sort roots in ascending order
        if (aResult.Roots[0] > aResult.Roots[1])
        {
            (aResult.Roots[0], aResult.Roots[1]) = (aResult.Roots[1], aResult.Roots[0]);
        }

        return aResult;
    }
}
