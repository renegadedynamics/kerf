// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Bracket.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Root and minimum bracketing algorithms.</summary>
public static partial class MathUtils
{
    /// <summary>Result of root bracketing operation.</summary>
    public struct BracketResult
    {
        /// <summary>True if valid bracket found.</summary>
        public bool IsValid;
        /// <summary>Lower bound.</summary>
        public double A;
        /// <summary>Upper bound.</summary>
        public double B;
        /// <summary>Function value at A.</summary>
        public double Fa;
        /// <summary>Function value at B.</summary>
        public double Fb;
    }

    /// <summary>Bracket a root by expanding [theA, theB] outward (golden ratio) until a sign change is found.</summary>
    public static BracketResult BracketRoot<Function>(Function theFunc, double theA, double theB, int theMaxIter = 50)
        where Function : IScalarFunction
    {
        BracketResult aResult = new();
        aResult.A = theA;
        aResult.B = theB;

        if (!theFunc.Value(aResult.A, out aResult.Fa))
        {
            return aResult;
        }
        if (!theFunc.Value(aResult.B, out aResult.Fb))
        {
            return aResult;
        }

        for (int i = 0; i < theMaxIter; ++i)
        {
            if (aResult.Fa * aResult.Fb < 0.0)
            {
                aResult.IsValid = true;
                // Ensure A < B
                if (aResult.A > aResult.B)
                {
                    (aResult.A, aResult.B) = (aResult.B, aResult.A);
                    (aResult.Fa, aResult.Fb) = (aResult.Fb, aResult.Fa);
                }
                return aResult;
            }

            // Expand the interval using golden ratio
            if (Math.Abs(aResult.Fa) < Math.Abs(aResult.Fb))
            {
                aResult.A += THE_GOLDEN_RATIO * (aResult.A - aResult.B);
                if (!theFunc.Value(aResult.A, out aResult.Fa))
                {
                    return aResult;
                }
            }
            else
            {
                aResult.B += THE_GOLDEN_RATIO * (aResult.B - aResult.A);
                if (!theFunc.Value(aResult.B, out aResult.Fb))
                {
                    return aResult;
                }
            }
        }

        return aResult;
    }

    /// <summary>Result of minimum bracketing operation.</summary>
    public struct MinBracketResult
    {
        /// <summary>True if valid bracket found (Fb &lt; Fa and Fb &lt; Fc).</summary>
        public bool IsValid;
        /// <summary>Left bound.</summary>
        public double A;
        /// <summary>Middle point (minimum location estimate).</summary>
        public double B;
        /// <summary>Right bound.</summary>
        public double C;
        public double Fa;
        public double Fb;
        public double Fc;
    }

    /// <summary>Options for minimum bracketing.</summary>
    /// <remarks>A class, as the other solver configurations (see MathUtils_Config.cs).</remarks>
    public class MinBracketOptions
    {
        /// <summary>Maximum iterations.</summary>
        public int MaxIterations = 50;
        /// <summary>Enable hard limits for parameter.</summary>
        public bool UseLimits = false;
        /// <summary>Left hard limit (inclusive).</summary>
        public double LeftLimit = 0.0;
        /// <summary>Right hard limit (inclusive).</summary>
        public double RightLimit = 0.0;
        /// <summary>True if FA is precomputed.</summary>
        public bool HasFA = false;
        /// <summary>True if FB is precomputed.</summary>
        public bool HasFB = false;
        /// <summary>Precomputed f(A).</summary>
        public double FA = 0.0;
        /// <summary>Precomputed f(B).</summary>
        public double FB = 0.0;

        public MinBracketOptions()
        {
        }

        public MinBracketOptions(MinBracketOptions theOther)
        {
            MaxIterations = theOther.MaxIterations;
            UseLimits = theOther.UseLimits;
            LeftLimit = theOther.LeftLimit;
            RightLimit = theOther.RightLimit;
            HasFA = theOther.HasFA;
            HasFB = theOther.HasFB;
            FA = theOther.FA;
            FB = theOther.FB;
        }
    }

    internal static partial class detail
    {
        internal static double Limited(double theValue, MinBracketOptions theOptions)
        {
            if (!theOptions.UseLimits)
            {
                return theValue;
            }
            return Max(theOptions.LeftLimit, Min(theOptions.RightLimit, theValue));
        }

        internal static bool LimitAndMayBeSwap<Function>(Function theFunc,
                                                         MinBracketOptions theOptions,
                                                         double theA,
                                                         ref double theB,
                                                         ref double theFB,
                                                         ref double theC,
                                                         ref double theFC)
            where Function : IScalarFunction
        {
            theC = Limited(theC, theOptions);
            if (Math.Abs(theB - theC) < THE_ZERO_TOL)
            {
                return false;
            }
            if (!theFunc.Value(theC, out theFC))
            {
                return false;
            }

            // Keep B between A and C
            if ((theA - theB) * (theB - theC) < 0.0)
            {
                (theB, theC) = (theC, theB);
                (theFB, theFC) = (theFC, theFB);
            }
            return true;
        }
    }

    /// <summary>
    /// Bracket a minimum by finding three points a &lt; b &lt; c with f(b) &lt; f(a) and f(b) &lt; f(c), using golden
    /// section expansion with parabolic interpolation.
    /// </summary>
    public static MinBracketResult BracketMinimum<Function>(Function theFunc, double theA, double theB, MinBracketOptions? theOptions = null)
        where Function : IScalarFunction
    {
        theOptions ??= new MinBracketOptions();
        MinBracketResult aResult = new();
        if (theOptions.MaxIterations < 1)
        {
            return aResult;
        }
        if (theOptions.UseLimits && theOptions.LeftLimit > theOptions.RightLimit)
        {
            return aResult;
        }

        aResult.A = detail.Limited(theA, theOptions);
        aResult.B = detail.Limited(theB, theOptions);
        if (Math.Abs(aResult.A - aResult.B) < THE_ZERO_TOL)
        {
            return aResult;
        }

        bool isUseFA =
          theOptions.HasFA && (!theOptions.UseLimits || Math.Abs(aResult.A - theA) < THE_ZERO_TOL);
        bool isUseFB =
          theOptions.HasFB && (!theOptions.UseLimits || Math.Abs(aResult.B - theB) < THE_ZERO_TOL);

        if (isUseFA)
        {
            aResult.Fa = theOptions.FA;
        }
        else if (!theFunc.Value(aResult.A, out aResult.Fa))
        {
            return aResult;
        }

        if (isUseFB)
        {
            aResult.Fb = theOptions.FB;
        }
        else if (!theFunc.Value(aResult.B, out aResult.Fb))
        {
            return aResult;
        }

        // Ensure we go downhill from A to B
        if (aResult.Fb > aResult.Fa)
        {
            (aResult.A, aResult.B) = (aResult.B, aResult.A);
            (aResult.Fa, aResult.Fb) = (aResult.Fb, aResult.Fa);
        }

        // Initial guess for C using golden ratio
        aResult.C = aResult.B + THE_GOLDEN_RATIO * (aResult.B - aResult.A);
        if (theOptions.UseLimits)
        {
            if (!detail.LimitAndMayBeSwap(theFunc,
                                          theOptions,
                                          aResult.A,
                                          ref aResult.B,
                                          ref aResult.Fb,
                                          ref aResult.C,
                                          ref aResult.Fc))
            {
                return aResult;
            }
        }
        else if (!theFunc.Value(aResult.C, out aResult.Fc))
        {
            return aResult;
        }

        // Keep expanding until we bracket a minimum
        for (int anIter = 0; anIter < theOptions.MaxIterations && aResult.Fb >= aResult.Fc; ++anIter)
        {
            // Parabolic extrapolation
            double aR = (aResult.B - aResult.A) * (aResult.Fb - aResult.Fc);
            double aQ = (aResult.B - aResult.C) * (aResult.Fb - aResult.Fa);
            double aDenom = 2.0 * SignTransfer(Max(Math.Abs(aQ - aR), THE_ZERO_TOL), aQ - aR);

            double aU = aResult.B - ((aResult.B - aResult.C) * aQ - (aResult.B - aResult.A) * aR) / aDenom;

            double aULim = aResult.B + 100.0 * (aResult.C - aResult.B);
            if (theOptions.UseLimits)
            {
                aULim = detail.Limited(aULim, theOptions);
            }
            double aFu = 0.0;

            if ((aResult.B - aU) * (aU - aResult.C) > 0.0)
            {
                // U is between B and C
                if (!theFunc.Value(aU, out aFu))
                {
                    return aResult;
                }

                if (aFu < aResult.Fc)
                {
                    aResult.A = aResult.B;
                    aResult.B = aU;
                    aResult.Fa = aResult.Fb;
                    aResult.Fb = aFu;
                    aResult.IsValid = true;
                    return aResult;
                }
                else if (aFu > aResult.Fb)
                {
                    aResult.C = aU;
                    aResult.Fc = aFu;
                    aResult.IsValid = true;
                    return aResult;
                }

                // Parabolic step didn't help, use golden section
                aU = aResult.C + THE_GOLDEN_RATIO * (aResult.C - aResult.B);
                if (theOptions.UseLimits)
                {
                    if (!detail.LimitAndMayBeSwap(theFunc, theOptions, aResult.B, ref aResult.C, ref aResult.Fc, ref aU, ref aFu))
                    {
                        return aResult;
                    }
                }
                else if (!theFunc.Value(aU, out aFu))
                {
                    return aResult;
                }
            }
            else if ((aResult.C - aU) * (aU - aULim) > 0.0)
            {
                // U is between C and limit
                if (theOptions.UseLimits)
                {
                    if (!detail.LimitAndMayBeSwap(theFunc, theOptions, aResult.B, ref aResult.C, ref aResult.Fc, ref aU, ref aFu))
                    {
                        return aResult;
                    }
                }
                else if (!theFunc.Value(aU, out aFu))
                {
                    return aResult;
                }

                if (aFu < aResult.Fc)
                {
                    aResult.B = aResult.C;
                    aResult.C = aU;
                    aU = aResult.C + THE_GOLDEN_RATIO * (aResult.C - aResult.B);
                    aResult.Fb = aResult.Fc;
                    aResult.Fc = aFu;
                    if (theOptions.UseLimits)
                    {
                        if (!detail.LimitAndMayBeSwap(theFunc, theOptions, aResult.B, ref aResult.C, ref aResult.Fc, ref aU, ref aFu))
                        {
                            return aResult;
                        }
                    }
                    else if (!theFunc.Value(aU, out aFu))
                    {
                        return aResult;
                    }
                }
            }
            else if ((aU - aULim) * (aULim - aResult.C) >= 0.0)
            {
                // U is beyond limit
                aU = aULim;
                if (theOptions.UseLimits)
                {
                    if (!detail.LimitAndMayBeSwap(theFunc, theOptions, aResult.B, ref aResult.C, ref aResult.Fc, ref aU, ref aFu))
                    {
                        return aResult;
                    }
                }
                else if (!theFunc.Value(aU, out aFu))
                {
                    return aResult;
                }
            }
            else
            {
                // Default golden section step
                aU = aResult.C + THE_GOLDEN_RATIO * (aResult.C - aResult.B);
                if (theOptions.UseLimits)
                {
                    if (!detail.LimitAndMayBeSwap(theFunc, theOptions, aResult.B, ref aResult.C, ref aResult.Fc, ref aU, ref aFu))
                    {
                        return aResult;
                    }
                }
                else if (!theFunc.Value(aU, out aFu))
                {
                    return aResult;
                }
            }

            // Shift points
            aResult.A = aResult.B;
            aResult.B = aResult.C;
            aResult.C = aU;
            aResult.Fa = aResult.Fb;
            aResult.Fb = aResult.Fc;
            aResult.Fc = aFu;
        }

        aResult.IsValid = (aResult.Fb < aResult.Fa && aResult.Fb < aResult.Fc);

        // Ensure A < B < C ordering
        if (aResult.IsValid && aResult.A > aResult.C)
        {
            (aResult.A, aResult.C) = (aResult.C, aResult.A);
            (aResult.Fa, aResult.Fc) = (aResult.Fc, aResult.Fa);
        }

        if (aResult.IsValid && !(aResult.A < aResult.B && aResult.B < aResult.C))
        {
            aResult.IsValid = false;
        }

        return aResult;
    }

    /// <summary>Backward-compatible convenience overload with only max-iterations argument.</summary>
    public static MinBracketResult BracketMinimum<Function>(Function theFunc, double theA, double theB, int theMaxIter)
        where Function : IScalarFunction
    {
        MinBracketOptions anOptions = new();
        anOptions.MaxIterations = theMaxIter;
        return BracketMinimum(theFunc, theA, theB, anOptions);
    }
}
