// Port of: src/FoundationClasses/TKMath/MathInteg/MathInteg_Kronrod.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

/// <summary>Gauss-Kronrod quadrature with embedded error estimation.</summary>
public static partial class MathInteg
{
    /// <summary>Configuration for Gauss-Kronrod integration.</summary>
    public class KronrodConfig : IntegConfig
    {
        /// <summary>Number of Gauss points (n); Kronrod uses 2n+1 points.</summary>
        public int NbGaussPoints = 7;
        /// <summary>Whether to use adaptive subdivision.</summary>
        public bool Adaptive = true;

        public KronrodConfig()
        {
        }

        public KronrodConfig(double theTolerance, int theMaxIter = 100)
            : base(theTolerance, theMaxIter)
        {
        }

        public KronrodConfig(KronrodConfig theOther)
            : base(theOther)
        {
            NbGaussPoints = theOther.NbGaussPoints;
            Adaptive = theOther.Adaptive;
        }
    }

    /// <summary>
    /// One application of the (2n+1)-point Gauss-Kronrod rule (n = theNbGauss), with the QUADPACK error estimate.
    /// </summary>
    public static IntegResult KronrodRule<Function>(Function theFunc, double theLower, double theUpper, int theNbGauss = 7)
        where Function : IScalarFunction
    {
        IntegResult aResult = new();

        int aNbKronrod = 2 * theNbGauss + 1;

        math_Vector aGaussP = new(1, theNbGauss);
        math_Vector aGaussW = new(1, theNbGauss);
        math_Vector aKronrodP = new(1, aNbKronrod);
        math_Vector aKronrodW = new(1, aNbKronrod);

        if (!GetKronrodPointsAndWeights(aNbKronrod, aKronrodP, aKronrodW)
            || !GetOrderedGaussPointsAndWeights(theNbGauss, aGaussP, aGaussW))
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }

        double aHalfLen = 0.5 * (theUpper - theLower);
        double aMid = 0.5 * (theUpper + theLower);

        int aNPnt2 = (aNbKronrod + 1) / 2;

        double aGaussVal = 0.0;
        double aKronrodVal = 0.0;

        math_Vector aF1 = new(0, aNPnt2 - 1);
        math_Vector aF2 = new(0, aNPnt2 - 1);

        for (int i = 2; i < aNPnt2; i += 2)
        {
            double aDx = aHalfLen * aKronrodP[i];
            double aVal1 = 0.0, aVal2 = 0.0;

            if (!theFunc.Value(aMid + aDx, out aVal1) || !theFunc.Value(aMid - aDx, out aVal2))
            {
                aResult.Status = Status.NumericalError;
                return aResult;
            }

            aF1[i] = aVal1;
            aF2[i] = aVal2;
            aGaussVal += (aVal1 + aVal2) * aGaussW[i / 2];
            aKronrodVal += (aVal1 + aVal2) * aKronrodW[i];
        }

        if (!theFunc.Value(aMid, out double aFc))
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }

        aKronrodVal += aFc * aKronrodW[aNPnt2];

        if (aNPnt2 % 2 == 0)
        {
            aGaussVal += aFc * aGaussW[aNPnt2 / 2];
        }

        for (int i = 1; i < aNPnt2; i += 2)
        {
            double aDx = aHalfLen * aKronrodP[i];
            double aVal1 = 0.0, aVal2 = 0.0;

            if (!theFunc.Value(aMid + aDx, out aVal1) || !theFunc.Value(aMid - aDx, out aVal2))
            {
                aResult.Status = Status.NumericalError;
                return aResult;
            }

            aF1[i] = aVal1;
            aF2[i] = aVal2;
            aKronrodVal += (aVal1 + aVal2) * aKronrodW[i];
        }

        double aMean = 0.5 * aKronrodVal;
        double aAsc = Math.Abs(aFc - aMean) * aKronrodW[aNPnt2];
        for (int i = 1; i < aNPnt2; ++i)
        {
            aAsc += aKronrodW[i] * (Math.Abs(aF1[i] - aMean) + Math.Abs(aF2[i] - aMean));
        }

        aAsc *= aHalfLen;
        aKronrodVal *= aHalfLen;
        aGaussVal *= aHalfLen;

        double aAbsError = Math.Abs(aKronrodVal - aGaussVal);

        if (aAsc != 0.0 && aAbsError != 0.0)
        {
            double aScale = Math.Pow(200.0 * aAbsError / aAsc, 1.5);
            if (aScale < 1.0)
            {
                aAbsError = Min(aAbsError, aAsc * aScale);
            }
        }

        aResult.Status = Status.OK;
        aResult.Value = aKronrodVal;
        aResult.AbsoluteError = aAbsError;
        aResult.RelativeError = aAbsError / Max(Math.Abs(aKronrodVal), 1.0e-15);
        aResult.NbPoints = aNbKronrod;
        aResult.NbIterations = 1;
        return aResult;
    }

    /// <summary>An interval of the adaptive Kronrod subdivision (a local struct in C++).</summary>
    private struct KronrodInterval
    {
        public double Lower;
        public double Upper;
        public double Value;
        public double Error;

        public KronrodInterval(double theLower, double theUpper, double theValue, double theError)
        {
            Lower = theLower;
            Upper = theUpper;
            Value = theValue;
            Error = theError;
        }
    }

    /// <summary>
    /// Adaptive Gauss-Kronrod integration: bisects the interval with the largest error estimate until the total
    /// error is within Tolerance or MaxIterations is reached. Without Adaptive, one KronrodRule.
    /// </summary>
    public static IntegResult Kronrod<Function>(Function theFunc, double theLower, double theUpper, KronrodConfig? theConfig = null)
        where Function : IScalarFunction
    {
        theConfig ??= new KronrodConfig();
        IntegResult aResult = new();

        if (!theConfig.Adaptive)
        {
            return KronrodRule(theFunc, theLower, theUpper, theConfig.NbGaussPoints);
        }

        IntegResult anInitResult = KronrodRule(theFunc, theLower, theUpper, theConfig.NbGaussPoints);
        if (!anInitResult.IsDone())
        {
            return anInitResult;
        }

        NCollection_DynamicArray<KronrodInterval> aHeap = new();
        aHeap.Append(new KronrodInterval(theLower, theUpper, anInitResult.Value!.Value, anInitResult.AbsoluteError!.Value));

        double aTotalValue = anInitResult.Value.Value;
        double aTotalError = anInitResult.AbsoluteError.Value;
        int aTotalPoints = anInitResult.NbPoints;
        int aIterations = 1;

        while (aIterations < theConfig.MaxIterations)
        {
            if (aTotalError < theConfig.Tolerance * Max(Math.Abs(aTotalValue), 1.0e-15))
            {
                break;
            }

            int aMaxIdx = 0;
            double aMaxError = 0.0;
            for (int i = 0; i < aHeap.Length(); ++i)
            {
                if (aHeap.Value(i).Error > aMaxError)
                {
                    aMaxError = aHeap.Value(i).Error;
                    aMaxIdx = i;
                }
            }

            if (aMaxError < MathUtils.THE_ZERO_TOL)
            {
                break; // No more refinement needed
            }

            KronrodInterval aWorst = aHeap.Value(aMaxIdx);
            double aBisMid = 0.5 * (aWorst.Lower + aWorst.Upper);

            IntegResult aLeftResult = KronrodRule(theFunc, aWorst.Lower, aBisMid, theConfig.NbGaussPoints);
            IntegResult aRightResult = KronrodRule(theFunc, aBisMid, aWorst.Upper, theConfig.NbGaussPoints);

            if (!aLeftResult.IsDone() || !aRightResult.IsDone())
            {
                aResult.Status = Status.NumericalError;
                aResult.Value = aTotalValue;
                aResult.AbsoluteError = aTotalError;
                aResult.NbPoints = aTotalPoints;
                aResult.NbIterations = aIterations;
                return aResult;
            }

            aTotalValue -= aWorst.Value;
            aTotalError -= aWorst.Error;
            aTotalValue += aLeftResult.Value!.Value + aRightResult.Value!.Value;
            aTotalError += aLeftResult.AbsoluteError!.Value + aRightResult.AbsoluteError!.Value;
            aTotalPoints += aLeftResult.NbPoints + aRightResult.NbPoints;
            ++aIterations;

            aHeap.ChangeValue(aMaxIdx) = new KronrodInterval(aWorst.Lower, aBisMid, aLeftResult.Value.Value, aLeftResult.AbsoluteError.Value);
            aHeap.Append(new KronrodInterval(aBisMid, aWorst.Upper, aRightResult.Value.Value, aRightResult.AbsoluteError.Value));
        }

        aResult.Status = Status.OK;
        aResult.Value = aTotalValue;
        aResult.AbsoluteError = aTotalError;
        aResult.RelativeError = aTotalError / Max(Math.Abs(aTotalValue), 1.0e-15);
        aResult.NbPoints = aTotalPoints;
        aResult.NbIterations = aIterations;
        return aResult;
    }

    /// <summary>
    /// Kronrod with automatic order selection: tries single rules of 7, 11, ..., theMaxOrder Gauss points and
    /// returns the first within theTolerance, else falls back to adaptive Kronrod.
    /// </summary>
    public static IntegResult KronrodAuto<Function>(Function theFunc, double theLower, double theUpper,
                                                    double theTolerance = 1.0e-10, int theMaxOrder = 30)
        where Function : IScalarFunction
    {
        // aBestResult is kept as in C++, though it is never returned
        IntegResult aBestResult = new();
        aBestResult.Status = Status.NotConverged;

        for (int aOrder = 7; aOrder <= theMaxOrder; aOrder += 4)
        {
            IntegResult aResult = KronrodRule(theFunc, theLower, theUpper, aOrder);
            if (!aResult.IsDone())
            {
                continue;
            }

            aBestResult = aResult;

            if (aResult.RelativeError.HasValue && aResult.RelativeError.Value < theTolerance)
            {
                return aResult;
            }
        }

        KronrodConfig aConfig = new();
        aConfig.Tolerance = theTolerance;
        aConfig.NbGaussPoints = 7;
        aConfig.Adaptive = true;
        aConfig.MaxIterations = 50;

        return Kronrod(theFunc, theLower, theUpper, aConfig);
    }

    /// <summary>f(a + t/(1-t)) / (1-t)^2 on [0, 1): the integrand of KronrodSemiInfinite (a local class in C++).</summary>
    private struct KronrodSemiInfiniteFunc<Function> : IScalarFunction
        where Function : IScalarFunction
    {
        private Function myFunc;
        private readonly double myA;

        public KronrodSemiInfiniteFunc(Function theF, double theA)
        {
            myFunc = theF;
            myA = theA;
        }

        public bool Value(double theT, out double theF)
        {
            if (theT >= 1.0)
            {
                theF = 0.0;
                return true;
            }

            double aX = myA + theT / (1.0 - theT);
            double aJacob = 1.0 / MathUtils.Sqr(1.0 - theT);

            theF = 0.0;
            if (!myFunc.Value(aX, out double aFx))
            {
                return false;
            }

            theF = aFx * aJacob;
            return true;
        }
    }

    /// <summary>Kronrod integration over [theLower, +infinity) by the substitution x = a + t/(1-t).</summary>
    public static IntegResult KronrodSemiInfinite<Function>(Function theFunc, double theLower, KronrodConfig? theConfig = null)
        where Function : IScalarFunction
    {
        KronrodSemiInfiniteFunc<Function> aTransformed = new(theFunc, theLower);
        return Kronrod(aTransformed, 0.0, 1.0, theConfig);
    }

    /// <summary>f(t/(1-t^2)) (1+t^2)/(1-t^2)^2 on (-1, 1): the integrand of KronrodInfinite (a local class in C++).</summary>
    private struct KronrodInfiniteFunc<Function> : IScalarFunction
        where Function : IScalarFunction
    {
        private Function myFunc;

        public KronrodInfiniteFunc(Function theF)
        {
            myFunc = theF;
        }

        public bool Value(double theT, out double theF)
        {
            if (Math.Abs(theT) >= 1.0)
            {
                theF = 0.0;
                return true;
            }

            double aT2 = theT * theT;
            double aX = theT / (1.0 - aT2);
            double aJacob = (1.0 + aT2) / MathUtils.Sqr(1.0 - aT2);

            theF = 0.0;
            if (!myFunc.Value(aX, out double aFx))
            {
                return false;
            }

            theF = aFx * aJacob;
            return true;
        }
    }

    /// <summary>Kronrod integration over (-infinity, +infinity) by the substitution x = t/(1-t^2).</summary>
    public static IntegResult KronrodInfinite<Function>(Function theFunc, KronrodConfig? theConfig = null)
        where Function : IScalarFunction
    {
        KronrodInfiniteFunc<Function> aTransformed = new(theFunc);
        return Kronrod(aTransformed, -1.0, 1.0, theConfig);
    }
}
