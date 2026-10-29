// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_Brent.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

/// <summary>Optimization algorithms for scalar and vector functions.</summary>
public static partial class MathOpt
{
    /// <summary>
    /// Brent's method for 1D minimization on [theLower, theUpper]: golden section search with parabolic
    /// interpolation. Root holds the minimum location, Value the minimum.
    /// </summary>
    public static ScalarResult Brent<Function>(Function theFunc, double theLower, double theUpper, Config? theConfig = null)
        where Function : IScalarFunction
    {
        theConfig ??= new Config();
        ScalarResult aResult = new();

        double aA = theLower;
        double aB = theUpper;

        // Initial point using golden section
        double aX = aA + MathUtils.THE_GOLDEN_SECTION * (aB - aA);
        double aW = aX;
        double aV = aX;

        if (!theFunc.Value(aX, out double aFx))
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }
        double aFw = aFx;
        double aFv = aFx;

        double aD = 0.0; // Current step
        double aE = 0.0; // Previous step

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            double aXm = 0.5 * (aA + aB);
            double aTol1 = theConfig.XTolerance * Math.Abs(aX) + MathUtils.THE_ZERO_TOL / 10.0;
            double aTol2 = 2.0 * aTol1;

            aResult.NbIterations = anIter + 1;

            // Check convergence
            if (Math.Abs(aX - aXm) <= (aTol2 - 0.5 * (aB - aA)))
            {
                aResult.Status = Status.OK;
                aResult.Root = aX;
                aResult.Value = aFx;
                return aResult;
            }

            double aU = 0.0;
            bool aUseParabolic = false;

            // Try parabolic interpolation if step is large enough
            if (Math.Abs(aE) > aTol1)
            {
                // Parabolic fit through x, w, v
                double aR = (aX - aW) * (aFx - aFv);
                double aQ = (aX - aV) * (aFx - aFw);
                double aP = (aX - aV) * aQ - (aX - aW) * aR;
                aQ = 2.0 * (aQ - aR);

                if (aQ > 0.0)
                {
                    aP = -aP;
                }
                else
                {
                    aQ = -aQ;
                }

                double aETmp = aE;
                aE = aD;

                // Check if parabolic step is acceptable
                if (Math.Abs(aP) < Math.Abs(0.5 * aQ * aETmp) && aP > aQ * (aA - aX) && aP < aQ * (aB - aX))
                {
                    aD = aP / aQ;
                    aU = aX + aD;

                    // Don't evaluate too close to bounds
                    if ((aU - aA) < aTol2 || (aB - aU) < aTol2)
                    {
                        aD = MathUtils.SignTransfer(aTol1, aXm - aX);
                    }
                    aUseParabolic = true;
                }
            }

            if (!aUseParabolic)
            {
                // Golden section step
                aE = (aX < aXm) ? (aB - aX) : (aA - aX);
                aD = MathUtils.THE_GOLDEN_SECTION * aE;
            }

            // Ensure step is at least aTol1
            if (Math.Abs(aD) >= aTol1)
            {
                aU = aX + aD;
            }
            else
            {
                aU = aX + MathUtils.SignTransfer(aTol1, aD);
            }

            if (!theFunc.Value(aU, out double aFu))
            {
                aResult.Status = Status.NumericalError;
                aResult.Root = aX;
                aResult.Value = aFx;
                return aResult;
            }

            // Update bracket and best points
            if (aFu <= aFx)
            {
                if (aU < aX)
                {
                    aB = aX;
                }
                else
                {
                    aA = aX;
                }

                aV = aW;
                aW = aX;
                aX = aU;
                aFv = aFw;
                aFw = aFx;
                aFx = aFu;
            }
            else
            {
                if (aU < aX)
                {
                    aA = aU;
                }
                else
                {
                    aB = aU;
                }

                if (aFu <= aFw || aW == aX)
                {
                    aV = aW;
                    aW = aU;
                    aFv = aFw;
                    aFw = aFu;
                }
                else if (aFu <= aFv || aV == aX || aV == aW)
                {
                    aV = aU;
                    aFv = aFu;
                }
            }
        }

        // Maximum iterations reached
        aResult.Status = Status.MaxIterations;
        aResult.Root = aX;
        aResult.Value = aFx;
        return aResult;
    }

    /// <summary>Golden section search for 1D minimization on [theLower, theUpper] (no parabolic steps).</summary>
    public static ScalarResult Golden<Function>(Function theFunc, double theLower, double theUpper, Config? theConfig = null)
        where Function : IScalarFunction
    {
        theConfig ??= new Config();
        ScalarResult aResult = new();

        const double aR = 0.618033988749895; // (sqrt(5) - 1) / 2
        const double aC = 1.0 - aR;

        double aA = theLower;
        double aB = theUpper;

        // Initialize interior points
        double aX1 = aA + aC * (aB - aA);
        double aX2 = aA + aR * (aB - aA);

        if (!theFunc.Value(aX1, out double aF1))
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }
        if (!theFunc.Value(aX2, out double aF2))
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            // Check convergence
            if ((aB - aA) < theConfig.XTolerance * (Math.Abs(aX1) + Math.Abs(aX2)))
            {
                aResult.Status = Status.OK;
                if (aF1 < aF2)
                {
                    aResult.Root = aX1;
                    aResult.Value = aF1;
                }
                else
                {
                    aResult.Root = aX2;
                    aResult.Value = aF2;
                }
                return aResult;
            }

            if (aF1 < aF2)
            {
                // Minimum is in [a, x2]
                aB = aX2;
                aX2 = aX1;
                aF2 = aF1;
                aX1 = aA + aC * (aB - aA);
                if (!theFunc.Value(aX1, out aF1))
                {
                    aResult.Status = Status.NumericalError;
                    aResult.Root = aX2;
                    aResult.Value = aF2;
                    return aResult;
                }
            }
            else
            {
                // Minimum is in [x1, b]
                aA = aX1;
                aX1 = aX2;
                aF1 = aF2;
                aX2 = aA + aR * (aB - aA);
                if (!theFunc.Value(aX2, out aF2))
                {
                    aResult.Status = Status.NumericalError;
                    aResult.Root = aX1;
                    aResult.Value = aF1;
                    return aResult;
                }
            }
        }

        // Maximum iterations reached
        aResult.Status = Status.MaxIterations;
        if (aF1 < aF2)
        {
            aResult.Root = aX1;
            aResult.Value = aF1;
        }
        else
        {
            aResult.Root = aX2;
            aResult.Value = aF2;
        }
        return aResult;
    }

    /// <summary>Brent's method after bracketing a minimum from theGuess with an initial step theStep.</summary>
    public static ScalarResult BrentWithBracket<Function>(Function theFunc, double theGuess, double theStep = 1.0, Config? theConfig = null)
        where Function : IScalarFunction
    {
        theConfig ??= new Config();
        ScalarResult aResult = new();

        // Try to bracket minimum
        MinBracketResult aBracket = MathUtils.BracketMinimum(theFunc, theGuess, theGuess + theStep);

        if (!aBracket.IsValid)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        // Apply Brent's method on the bracket
        return Brent(theFunc, aBracket.A, aBracket.C, theConfig);
    }
}
