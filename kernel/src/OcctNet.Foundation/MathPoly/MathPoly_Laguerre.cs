// Port of: src/FoundationClasses/TKMath/MathPoly/MathPoly_Laguerre.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Numerics;
using System.Runtime.CompilerServices;

namespace Occt;

// std::complex<double> is System.Numerics.Complex (same principal branch for Sqrt; Abs is the magnitude). Coefficient
// pointers are spans. The 20-element std::arrays of GeneralPolyResult are the inline array MathPoly.RootArray<T>.

/// <summary>Polynomial root finding algorithms using Laguerre's method.</summary>
public static partial class MathPoly
{
    /// <summary>Maximum polynomial degree supported by Laguerre solver.</summary>
    public const int THE_MAX_POLY_DEGREE = 20;

    /// <summary>C++'s <c>std::array&lt;T, THE_MAX_POLY_DEGREE&gt;</c>: 20 values held inline, indexed from 0.</summary>
    [InlineArray(THE_MAX_POLY_DEGREE)]
    public struct RootArray<T>
    {
        private T myElement0;
    }

    /// <summary>Result for general polynomial solver: real roots (sorted) and complex roots (conjugate pairs).</summary>
    public struct GeneralPolyResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        public RootArray<double> Roots;
        public RootArray<Complex> ComplexRoots;
        public int NbRoots = 0;
        public int NbComplexRoots = 0;

        public GeneralPolyResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(GeneralPolyResult theResult) => theResult.IsDone();

        public static bool operator true(GeneralPolyResult theResult) => theResult.IsDone();

        public static bool operator false(GeneralPolyResult theResult) => !theResult.IsDone();
    }

    internal static partial class detail
    {
        /// <summary>Evaluate polynomial (c[0] = constant term) and its first two derivatives at complex x.</summary>
        internal static void EvaluatePolynomialWithDerivatives(ReadOnlySpan<double> theCoeffs,
                                                               int theDegree,
                                                               Complex theX,
                                                               out Complex theP,
                                                               out Complex theDP,
                                                               out Complex theD2P)
        {
            // Horner's method with derivative computation
            theP = new Complex(theCoeffs[theDegree], 0.0);
            theDP = new Complex(0.0, 0.0);
            theD2P = new Complex(0.0, 0.0);

            for (int i = theDegree - 1; i >= 0; --i)
            {
                theD2P = theD2P * theX + theDP;
                theDP = theDP * theX + theP;
                theP = theP * theX + new Complex(theCoeffs[i], 0.0);
            }
            theD2P *= 2.0;
        }

        /// <summary>Laguerre iteration to find one root of polynomial.</summary>
        internal static Complex LaguerreIteration(ReadOnlySpan<double> theCoeffs,
                                                  int theDegree,
                                                  Complex theX0,
                                                  double theTol,
                                                  int theMaxIter)
        {
            Complex aX = theX0;
            double aN = (double)theDegree;

            for (int anIter = 0; anIter < theMaxIter; ++anIter)
            {
                EvaluatePolynomialWithDerivatives(theCoeffs, theDegree, aX, out Complex aP, out Complex aDP, out Complex aD2P);

                double aAbsP = Complex.Abs(aP);
                if (aAbsP < theTol)
                {
                    return aX;
                }

                // Laguerre's formula: x_new = x - n*P / (P' +/- sqrt((n-1)*((n-1)*P'^2 - n*P*P'')))
                Complex aG = aDP / aP;
                Complex aH = aG * aG - aD2P / aP;
                Complex aSq = Complex.Sqrt((aN - 1.0) * (aN * aH - aG * aG));

                // Choose denominator with larger magnitude for stability
                Complex aDenom1 = aG + aSq;
                Complex aDenom2 = aG - aSq;
                Complex aDenom = (Complex.Abs(aDenom1) > Complex.Abs(aDenom2)) ? aDenom1 : aDenom2;

                Complex aDelta;
                if (Complex.Abs(aDenom) < MathUtils.THE_ZERO_TOL)
                {
                    // Fallback: simple Newton step
                    if (Complex.Abs(aDP) < MathUtils.THE_ZERO_TOL)
                    {
                        aDelta = new Complex(1.0 + Complex.Abs(aX), 0.0);
                    }
                    else
                    {
                        aDelta = aP / aDP;
                    }
                }
                else
                {
                    aDelta = new Complex(aN, 0.0) / aDenom;
                }

                aX -= aDelta;

                // Check convergence
                if (Complex.Abs(aDelta) < theTol * (1.0 + Complex.Abs(aX)))
                {
                    return aX;
                }
            }

            return aX;
        }

        /// <summary>Deflate polynomial by removing a real root (synthetic division by (x - root)).</summary>
        internal static void DeflateReal(Span<double> theCoeffs, ref int theDegree, double theRoot)
        {
            // Synthetic division
            double aCarry = theCoeffs[theDegree];
            for (int i = theDegree - 1; i >= 0; --i)
            {
                double aTemp = theCoeffs[i];
                theCoeffs[i] = aCarry;
                aCarry = aTemp + aCarry * theRoot;
            }
            --theDegree;
        }

        /// <summary>Deflate polynomial by removing a complex conjugate pair (division by x^2 - 2 Re(root) x + |root|^2).</summary>
        internal static void DeflateComplex(Span<double> theCoeffs, ref int theDegree, Complex theRoot)
        {
            // Quadratic factor: x^2 + b*x + c where b = -2*re, c = re^2 + im^2
            double aRe = theRoot.Real;
            double aIm = theRoot.Imaginary;
            double aB = -2.0 * aRe;
            double aC = aRe * aRe + aIm * aIm;

            // Synthetic division by (x^2 + b*x + c)
            Span<double> aQuotient = stackalloc double[THE_MAX_POLY_DEGREE + 1];
            aQuotient.Fill(0.0);

            // Work from highest degree down
            aQuotient[theDegree - 2] = theCoeffs[theDegree];
            if (theDegree >= 3)
            {
                aQuotient[theDegree - 3] = theCoeffs[theDegree - 1] - aB * aQuotient[theDegree - 2];
            }

            for (int i = theDegree - 4; i >= 0; --i)
            {
                aQuotient[i] = theCoeffs[i + 2] - aB * aQuotient[i + 1] - aC * aQuotient[i + 2];
            }

            // Copy quotient back to coefficients
            for (int i = 0; i <= theDegree - 2; ++i)
            {
                theCoeffs[i] = aQuotient[i];
            }
            theDegree -= 2;
        }

        /// <summary>Refine a real root using Newton-Raphson on the original polynomial.</summary>
        internal static double RefineRealRoot(ReadOnlySpan<double> theOrigCoeffs, int theOrigDegree, double theRoot)
        {
            const int THE_MAX_ITER = 10;
            const double THE_TOL = 1.0e-14;

            double aX = theRoot;
            for (int anIter = 0; anIter < THE_MAX_ITER; ++anIter)
            {
                // Evaluate P and P' using Horner
                double aP = theOrigCoeffs[theOrigDegree];
                double aDP = 0.0;
                for (int i = theOrigDegree - 1; i >= 0; --i)
                {
                    aDP = aDP * aX + aP;
                    aP = aP * aX + theOrigCoeffs[i];
                }

                if (Math.Abs(aDP) < MathUtils.THE_ZERO_TOL)
                {
                    break;
                }

                double aDelta = aP / aDP;
                aX -= aDelta;

                if (Math.Abs(aDelta) < THE_TOL * (1.0 + Math.Abs(aX)))
                {
                    break;
                }
            }
            return aX;
        }
    }

    /// <summary>
    /// Solve a polynomial equation a0 + a1*x + ... + an*x^n = 0 (degree 1 to THE_MAX_POLY_DEGREE) by Laguerre's
    /// method with deflation; real roots are refined by Newton on the original polynomial, sorted and deduplicated.
    /// </summary>
    public static GeneralPolyResult Laguerre(ReadOnlySpan<double> theCoeffs, int theDegree, double theTol = 1.0e-12)
    {
        GeneralPolyResult aResult = new();

        // Validate input
        if (theDegree < 1 || theDegree > THE_MAX_POLY_DEGREE)
        {
            aResult.Status = MathUtils.Status.InvalidInput;
            return aResult;
        }

        // Check leading coefficient
        if (Math.Abs(theCoeffs[theDegree]) < MathUtils.THE_ZERO_TOL)
        {
            aResult.Status = MathUtils.Status.InvalidInput;
            return aResult;
        }

        // Copy coefficients for deflation
        Span<double> aWorkCoeffs = stackalloc double[THE_MAX_POLY_DEGREE + 1];
        for (int i = 0; i <= theDegree; ++i)
        {
            aWorkCoeffs[i] = theCoeffs[i];
        }

        // Store original for refinement
        Span<double> aOrigCoeffs = stackalloc double[THE_MAX_POLY_DEGREE + 1];
        for (int i = 0; i <= theDegree; ++i)
        {
            aOrigCoeffs[i] = theCoeffs[i];
        }

        int aDeg = theDegree;

        // Use different starting points to improve convergence
        // Rotate through starting positions
        ReadOnlySpan<Complex> aStartPoints = [new Complex(0.0, 0.1),
                                              new Complex(1.0, 0.5),
                                              new Complex(-0.5, 0.3),
                                              new Complex(0.5, -0.3)];

        // Find all roots
        int aStartIdx = 0;
        while (aDeg > 0)
        {
            Complex aX0 = aStartPoints[aStartIdx % 4];
            ++aStartIdx;

            // Use Laguerre iteration
            Complex aRoot = detail.LaguerreIteration(aWorkCoeffs, aDeg, aX0, theTol, 100);

            // Determine if root is real or complex
            double aImagPart = Math.Abs(aRoot.Imaginary);
            double aRealPart = Math.Abs(aRoot.Real);
            double aScale = Max(1.0, aRealPart);

            if (aImagPart < theTol * aScale)
            {
                // Real root
                double aRealRoot = aRoot.Real;

                // Refine using Newton on original polynomial
                aRealRoot = detail.RefineRealRoot(aOrigCoeffs, theDegree, aRealRoot);

                aResult.Roots[aResult.NbRoots++] = aRealRoot;

                // Deflate
                detail.DeflateReal(aWorkCoeffs, ref aDeg, aRealRoot);
            }
            else
            {
                // Complex conjugate pair
                aResult.ComplexRoots[aResult.NbComplexRoots++] = aRoot;
                aResult.ComplexRoots[aResult.NbComplexRoots++] = Complex.Conjugate(aRoot);

                // Deflate by quadratic
                detail.DeflateComplex(aWorkCoeffs, ref aDeg, aRoot);
            }
        }

        // Sort real roots
        Span<double> aRoots = aResult.Roots;
        aRoots[..aResult.NbRoots].Sort();

        // Remove duplicate real roots
        if (aResult.NbRoots > 1)
        {
            int aNewCount = 1;
            for (int i = 1; i < aResult.NbRoots; ++i)
            {
                if (Math.Abs(aRoots[i] - aRoots[aNewCount - 1]) > theTol)
                {
                    aRoots[aNewCount++] = aRoots[i];
                }
            }
            aResult.NbRoots = aNewCount;
        }

        aResult.Status = MathUtils.Status.OK;
        return aResult;
    }

    /// <summary>Laguerre for a coefficient array of theSize values (degree theSize - 1).</summary>
    public static GeneralPolyResult LaguerreN(ReadOnlySpan<double> theCoeffs, int theSize, double theTol = 1.0e-12)
    {
        if (theSize < 2)
        {
            GeneralPolyResult aResult = new();
            aResult.Status = MathUtils.Status.InvalidInput;
            return aResult;
        }
        return Laguerre(theCoeffs, theSize - 1, theTol);
    }

    /// <summary>Real roots of a*x^6 + b*x^5 + c*x^4 + d*x^3 + e*x^2 + f*x + g = 0 (at most 4 kept).</summary>
    public static MathUtils.PolyResult Sextic(double theA, double theB, double theC, double theD, double theE, double theF, double theG)
    {
        MathUtils.PolyResult aResult = new();

        // Handle leading zero coefficient
        if (MathUtils.IsZero(theA))
        {
            // Reduce to quintic, then use Laguerre
            ReadOnlySpan<double> aCoeffs5 = [theG, theF, theE, theD, theC, theB];
            GeneralPolyResult aGenResult5 = Laguerre(aCoeffs5, 5);
            if (!aGenResult5.IsDone())
            {
                aResult.Status = aGenResult5.Status;
                return aResult;
            }
            aResult.Status = MathUtils.Status.OK;
            aResult.NbRoots = Min(aGenResult5.NbRoots, 4); // OCCT: real roots beyond the fourth are dropped
            for (int i = 0; i < aResult.NbRoots; ++i)
            {
                aResult.Roots[i] = aGenResult5.Roots[i];
            }
            return aResult;
        }

        ReadOnlySpan<double> aCoeffs = [theG, theF, theE, theD, theC, theB, theA];
        GeneralPolyResult aGenResult = Laguerre(aCoeffs, 6);

        if (!aGenResult.IsDone())
        {
            aResult.Status = aGenResult.Status;
            return aResult;
        }

        aResult.Status = MathUtils.Status.OK;
        aResult.NbRoots = Min(aGenResult.NbRoots, 4); // OCCT: real roots beyond the fourth are dropped
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            aResult.Roots[i] = aGenResult.Roots[i];
        }

        return aResult;
    }

    /// <summary>Real roots of a*x^5 + b*x^4 + c*x^3 + d*x^2 + e*x + f = 0 (at most 4 kept).</summary>
    public static MathUtils.PolyResult Quintic(double theA, double theB, double theC, double theD, double theE, double theF)
    {
        MathUtils.PolyResult aResult = new();

        if (MathUtils.IsZero(theA))
        {
            // Reduce to quartic - use existing solver
            return Quartic(theB, theC, theD, theE, theF);
        }

        ReadOnlySpan<double> aCoeffs = [theF, theE, theD, theC, theB, theA];
        GeneralPolyResult aGenResult = Laguerre(aCoeffs, 5);

        if (!aGenResult.IsDone())
        {
            aResult.Status = aGenResult.Status;
            return aResult;
        }

        aResult.Status = MathUtils.Status.OK;
        aResult.NbRoots = Min(aGenResult.NbRoots, 4); // OCCT: a fifth real root is dropped
        for (int i = 0; i < aResult.NbRoots; ++i)
        {
            aResult.Roots[i] = aGenResult.Roots[i];
        }

        return aResult;
    }

    /// <summary>Solve the octic a0 + a1*x + ... + a8*x^8 = 0 (9 coefficients) by Laguerre's method.</summary>
    public static GeneralPolyResult Octic(ReadOnlySpan<double> theCoeffs)
    {
        return Laguerre(theCoeffs, 8);
    }
}
