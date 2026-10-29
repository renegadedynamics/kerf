// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Poly.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Polynomial evaluation and manipulation.</summary>
public static partial class MathUtils
{
    /// <summary>Evaluate polynomial a0 + a1*x + ... (constant term first) using Horner's method.</summary>
    public static double EvalPoly(ReadOnlySpan<double> theCoeffs, int theDegree, double theX)
    {
        double aResult = theCoeffs[theDegree];
        for (int i = theDegree - 1; i >= 0; --i)
        {
            aResult = aResult * theX + theCoeffs[i];
        }
        return aResult;
    }

    /// <summary>Evaluate polynomial (constant term first) and its derivative in a single pass.</summary>
    public static void EvalPolyDeriv(ReadOnlySpan<double> theCoeffs, int theDegree, double theX, out double theValue, out double theDeriv)
    {
        theValue = theCoeffs[theDegree];
        theDeriv = 0.0;
        for (int i = theDegree - 1; i >= 0; --i)
        {
            theDeriv = theDeriv * theX + theValue;
            theValue = theValue * theX + theCoeffs[i];
        }
    }

    /// <summary>Evaluate polynomial with coefficients in descending order (leading term first).</summary>
    public static double EvalPolyDesc(ReadOnlySpan<double> theCoeffs, int theDegree, double theX)
    {
        double aResult = theCoeffs[0];
        for (int i = 1; i <= theDegree; ++i)
        {
            aResult = aResult * theX + theCoeffs[i];
        }
        return aResult;
    }

    /// <summary>Newton-Raphson refinement for a polynomial root (constant term first).</summary>
    public static double RefinePolyRoot(ReadOnlySpan<double> theCoeffs, int theDegree, double theRoot, int theMaxIter = 5)
    {
        double aX = theRoot;
        for (int i = 0; i < theMaxIter; ++i)
        {
            EvalPolyDeriv(theCoeffs, theDegree, aX, out double aF, out double aDf);

            if (IsZero(aDf))
            {
                break;
            }

            double aDx = aF / aDf;
            aX -= aDx;

            if (Math.Abs(aDx) < THE_EPSILON * Max(1.0, Math.Abs(aX)))
            {
                break;
            }
        }
        return aX;
    }

    /// <summary>Newton-Raphson refinement with coefficients in descending order (degree at most 4).</summary>
    public static double RefinePolyRootDesc(ReadOnlySpan<double> theCoeffs, int theDegree, double theRoot, int theMaxIter = 5)
    {
        // Convert to ascending order for refinement
        Span<double> aAsc = stackalloc double[5];
        for (int i = 0; i <= theDegree; ++i)
        {
            aAsc[i] = theCoeffs[theDegree - i];
        }
        return RefinePolyRoot(aAsc, theDegree, theRoot, theMaxIter);
    }

    /// <summary>Sort roots in ascending order (insertion sort for small arrays).</summary>
    public static void SortRoots(Span<double> theRoots, int theCount)
    {
        for (int i = 1; i < theCount; ++i)
        {
            double aKey = theRoots[i];
            int j = i;
            while (j > 0 && theRoots[j - 1] > aKey)
            {
                theRoots[j] = theRoots[j - 1];
                --j;
            }
            theRoots[j] = aKey;
        }
    }

    /// <summary>Remove duplicate sorted roots within tolerance; returns the new count.</summary>
    public static int RemoveDuplicateRoots(Span<double> theRoots, int theCount, double theTolerance = 1.0e-10)
    {
        if (theCount <= 1)
        {
            return theCount;
        }

        int aNewCount = 1;
        for (int i = 1; i < theCount; ++i)
        {
            if (Math.Abs(theRoots[i] - theRoots[aNewCount - 1]) > theTolerance)
            {
                theRoots[aNewCount] = theRoots[i];
                ++aNewCount;
            }
        }
        return aNewCount;
    }

    /// <summary>Depressed cubic: x^3 + bx^2 + cx + d to t^3 + pt + q via x = t - b/3.</summary>
    public static void DepressCubic(double theB, double theC, double theD, out double theP, out double theQ, out double theShift)
    {
        theShift = theB / 3.0;
        double aB2 = theB * theB;
        theP = theC - aB2 / 3.0;
        theQ = theD - theB * theC / 3.0 + 2.0 * aB2 * theB / 27.0;
    }

    /// <summary>Depressed quartic: x^4 + bx^3 + cx^2 + dx + e to t^4 + pt^2 + qt + r via x = t - b/4.</summary>
    public static void DepressQuartic(double theB, double theC, double theD, double theE,
                                      out double theP, out double theQ, out double theR, out double theShift)
    {
        theShift = theB / 4.0;
        double aB2 = theB * theB;
        double aB3 = aB2 * theB;
        double aB4 = aB2 * aB2;

        theP = theC - 3.0 * aB2 / 8.0;
        theQ = theD - theB * theC / 2.0 + aB3 / 8.0;
        theR = theE - theB * theD / 4.0 + aB2 * theC / 16.0 - 3.0 * aB4 / 256.0;
    }
}
